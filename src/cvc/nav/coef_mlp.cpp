/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// coef_mlp.cpp — see coef_mlp.h. Float-equivalent transcription of CoefMLP; this
// TU must be built without -ffast-math / -ffp-contract=fast so the float32 op
// order tracks torch's as closely as possible.
//
// `.cvcnav` layout (little-endian; x86/ARM host = producer and consumer):
//   char  magic[4] = "CVNV"
//   u32   format_version
//   u32   flags
//   u32   in_features
//   u32   out_features
//   u32   num_layers
//   u64   arch_hash
//   per layer:  u32 rows, u32 cols, u32 act, f32 w[rows*cols], f32 b[rows]
//   u32   out_bias_len ; f32 out_bias[out_bias_len]   (raw bias; log(expm1) at load)
//   u32   meta_len     ; char meta[meta_len]          (provenance, ignored here)

#include <cmath>
#include <cstring>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/detail/parallel.h>
#include <fstream>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {

inline float silu(float x) {
  // torch SiLU: x * sigmoid(x).
  const float sig = 1.0f / (1.0f + std::exp(-x));
  return x * sig;
}

inline float softplus(float x) {
  // torch F.softplus(beta=1, threshold=20): linear tail avoids exp overflow.
  return x > 20.0f ? x : std::log1p(std::exp(x));
}

// Read a trivially-copyable value from a little-endian byte cursor, advancing it.
template <class T> T take(const std::uint8_t *&p, const std::uint8_t *end) {
  if (p + sizeof(T) > end)
    throw std::runtime_error("cvc::nav::coef_mlp: truncated .cvcnav (header/field)");
  T v;
  std::memcpy(&v, p, sizeof(T));
  p += sizeof(T);
  return v;
}

void take_floats(const std::uint8_t *&p, const std::uint8_t *end, std::vector<float> &out,
                 std::size_t count) {
  if (p + count * sizeof(float) > end)
    throw std::runtime_error("cvc::nav::coef_mlp: truncated .cvcnav (weight block)");
  out.resize(count);
  std::memcpy(out.data(), p, count * sizeof(float));
  p += count * sizeof(float);
}

} // namespace

std::uint64_t coef_mlp::compute_arch_hash(int in_features, int out_features,
                                          const std::vector<std::uint32_t> &layer_shape_act) {
  // FNV-1a over the architecture descriptors (shape, not weights).
  std::uint64_t h = 1469598103934665603ULL;
  auto mix = [&](std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
      h ^= (v >> (8 * i)) & 0xFF;
      h *= 1099511628211ULL;
    }
  };
  mix(static_cast<std::uint64_t>(in_features));
  mix(static_cast<std::uint64_t>(out_features));
  mix(static_cast<std::uint64_t>(layer_shape_act.size()));
  for (std::uint32_t v : layer_shape_act)
    mix(v);
  return h;
}

void coef_mlp::build_from_bytes(const std::uint8_t *p, std::size_t n) {
  const std::uint8_t *const end = p + n;
  char magic[4];
  if (p + 4 > end)
    throw std::runtime_error("cvc::nav::coef_mlp: empty .cvcnav");
  std::memcpy(magic, p, 4);
  p += 4;
  if (std::memcmp(magic, "CVNV", 4) != 0)
    throw std::runtime_error("cvc::nav::coef_mlp: bad magic (not a .cvcnav file)");
  fmt_ = take<std::uint32_t>(p, end);
  if (fmt_ != kFormatVersion)
    throw std::runtime_error("cvc::nav::coef_mlp: unsupported .cvcnav format version");
  flags_ = take<std::uint32_t>(p, end);
  in_ = static_cast<int>(take<std::uint32_t>(p, end));
  out_ = static_cast<int>(take<std::uint32_t>(p, end));
  const int num_layers = static_cast<int>(take<std::uint32_t>(p, end));
  arch_hash_ = take<std::uint64_t>(p, end);

  std::vector<std::uint32_t> shape_act;
  shape_act.reserve(static_cast<std::size_t>(num_layers) * 3);
  layers_.resize(num_layers);
  int prev = in_;
  for (int i = 0; i < num_layers; ++i) {
    Layer &L = layers_[i];
    L.rows = static_cast<int>(take<std::uint32_t>(p, end));
    L.cols = static_cast<int>(take<std::uint32_t>(p, end));
    L.act = take<std::uint32_t>(p, end);
    if (L.rows <= 0 || L.cols <= 0 || L.cols != prev)
      throw std::runtime_error("cvc::nav::coef_mlp: layer dims inconsistent with the chain");
    take_floats(p, end, L.w, static_cast<std::size_t>(L.rows) * L.cols);
    take_floats(p, end, L.b, static_cast<std::size_t>(L.rows));
    shape_act.push_back(static_cast<std::uint32_t>(L.rows));
    shape_act.push_back(static_cast<std::uint32_t>(L.cols));
    shape_act.push_back(L.act);
    prev = L.rows;
  }
  if (prev != out_)
    throw std::runtime_error("cvc::nav::coef_mlp: last layer does not produce out_features");
  if (compute_arch_hash(in_, out_, shape_act) != arch_hash_)
    throw std::runtime_error("cvc::nav::coef_mlp: arch_hash mismatch (stale/incompatible weights)");

  const std::uint32_t obn = take<std::uint32_t>(p, end);
  std::vector<float> out_bias;
  take_floats(p, end, out_bias, obn);
  out_bias_off_.resize(out_bias.size());
  if (flags_ & kFlagSoftplusLogExpm1) {
    // Fold log(expm1(bias)) once, in float32, matching torch.log(torch.expm1(.)).
    for (std::size_t i = 0; i < out_bias.size(); ++i)
      out_bias_off_[i] = std::log(std::expm1(out_bias[i]));
  } else {
    out_bias_off_ = out_bias; // used as a plain additive offset
  }
  if (static_cast<int>(out_bias_off_.size()) != out_)
    throw std::runtime_error("cvc::nav::coef_mlp: out_bias length != out_features");
  // A meta trailer may follow; it is optional and ignored here.
}

coef_mlp coef_mlp::load_from_memory(const void *data, std::size_t nbytes) {
  coef_mlp m;
  m.build_from_bytes(static_cast<const std::uint8_t *>(data), nbytes);
  return m;
}

coef_mlp::flat_layers coef_mlp::export_flat() const {
  flat_layers fl;
  fl.in = in_;
  fl.out = out_;
  fl.num_layers = static_cast<int>(layers_.size());
  fl.out_bias_off = out_bias_off_;
  for (const Layer &L : layers_) {
    fl.rows.push_back(L.rows);
    fl.cols.push_back(L.cols);
    fl.act.push_back(static_cast<int>(L.act));
    fl.w_off.push_back(static_cast<long>(fl.data.size()));
    fl.data.insert(fl.data.end(), L.w.begin(), L.w.end());
    fl.b_off.push_back(static_cast<long>(fl.data.size()));
    fl.data.insert(fl.data.end(), L.b.begin(), L.b.end());
  }
  return fl;
}

coef_mlp coef_mlp::load(const std::string &path) {
  std::ifstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cvc::nav::coef_mlp: cannot open " + path);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  return load_from_memory(buf.data(), buf.size());
}

void coef_mlp::forward(const float *feats, int n, float *out, int num_threads) const {
  detail::parallel_for(n, num_threads, [&](int s) {
    // Hidden width is small (<= 64 for the shipped net); keep activations on the
    // stack. If a future net exceeds this, bump the cap.
    constexpr int kMaxW = 256;
    float a[kMaxW];
    float b[kMaxW];
    const float *x = feats + static_cast<std::size_t>(s) * in_;
    for (int i = 0; i < in_; ++i)
      a[i] = x[i];
    for (const Layer &L : layers_) {
      for (int o = 0; o < L.rows; ++o) {
        float acc = L.b[o];
        const float *w = L.w.data() + static_cast<std::size_t>(o) * L.cols;
        for (int i = 0; i < L.cols; ++i)
          acc += w[i] * a[i];
        b[o] = (L.act == 1) ? silu(acc) : acc;
      }
      for (int o = 0; o < L.rows; ++o)
        a[o] = b[o];
    }
    float *o = out + static_cast<std::size_t>(s) * out_;
    for (int k = 0; k < out_; ++k)
      o[k] = softplus(a[k] + out_bias_off_[k]);
  });
}

} // namespace nav
} // namespace cvc
