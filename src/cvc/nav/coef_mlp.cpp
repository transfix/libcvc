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
#include <cstdlib>
#include <cstring>
#include <cvc/core/config.h> // CVC_NAV_DATADIR
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/detail/parallel.h>
#include <fstream>
#include <stdexcept>
#include <string>
#ifndef _WIN32
#include <dlfcn.h>
#endif

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

coef_mlp coef_mlp::default_biased() {
  coef_mlp m;
  m.fmt_ = kFormatVersion;
  m.flags_ = kFlagSoftplusLogExpm1;
  m.in_ = 5;
  m.out_ = 3;
  auto mk = [](int r, int c, std::uint32_t act) {
    Layer L;
    L.rows = r;
    L.cols = c;
    L.act = act;
    L.w.assign(static_cast<std::size_t>(r) * c, 0.0f);
    L.b.assign(r, 0.0f);
    return L;
  };
  m.layers_ = {mk(64, 5, 1), mk(64, 64, 1), mk(3, 64, 0)};
  const float bias[3] = {1.0f, 3.0f, 4.0f}; // the CoefMLP bias-toward-basin
  m.out_bias_off_.resize(3);
  for (int i = 0; i < 3; ++i)
    m.out_bias_off_[i] = std::log(std::expm1(bias[i]));
  const std::vector<std::uint32_t> sa = {64, 5, 1, 64, 64, 1, 3, 64, 0};
  m.arch_hash_ = compute_arch_hash(5, 3, sa);
  return m;
}

coef_mlp coef_mlp::from_layers(int in, int out, const std::vector<int> &rows,
                               const std::vector<int> &cols, const std::vector<std::uint32_t> &act,
                               const std::vector<std::vector<float>> &w,
                               const std::vector<std::vector<float>> &b,
                               const std::vector<float> &out_bias_raw) {
  const int num_layers = static_cast<int>(rows.size());
  if (static_cast<int>(cols.size()) != num_layers || static_cast<int>(act.size()) != num_layers ||
      static_cast<int>(w.size()) != num_layers || static_cast<int>(b.size()) != num_layers)
    throw std::runtime_error("cvc::nav::coef_mlp::from_layers: ragged layer arrays");
  coef_mlp m;
  m.fmt_ = kFormatVersion;
  m.flags_ = kFlagSoftplusLogExpm1;
  m.in_ = in;
  m.out_ = out;
  m.layers_.resize(num_layers);
  std::vector<std::uint32_t> sa;
  int prev = in;
  for (int i = 0; i < num_layers; ++i) {
    if (rows[i] <= 0 || cols[i] <= 0 || cols[i] != prev)
      throw std::runtime_error("cvc::nav::coef_mlp::from_layers: layer dims inconsistent");
    if (static_cast<int>(w[i].size()) != rows[i] * cols[i] ||
        static_cast<int>(b[i].size()) != rows[i])
      throw std::runtime_error("cvc::nav::coef_mlp::from_layers: weight size mismatch");
    Layer &L = m.layers_[i];
    L.rows = rows[i];
    L.cols = cols[i];
    L.act = act[i];
    L.w = w[i];
    L.b = b[i];
    sa.push_back(static_cast<std::uint32_t>(rows[i]));
    sa.push_back(static_cast<std::uint32_t>(cols[i]));
    sa.push_back(act[i]);
    prev = rows[i];
  }
  if (prev != out)
    throw std::runtime_error("cvc::nav::coef_mlp::from_layers: last layer does not produce out");
  if (static_cast<int>(out_bias_raw.size()) != out)
    throw std::runtime_error("cvc::nav::coef_mlp::from_layers: out_bias length != out");
  m.out_bias_off_.resize(out);
  for (int i = 0; i < out; ++i)
    m.out_bias_off_[i] = std::log(std::expm1(out_bias_raw[i]));
  m.arch_hash_ = compute_arch_hash(in, out, sa);
  return m;
}

void coef_mlp::save(const std::string &path, const std::string &meta) const {
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("cvc::nav::coef_mlp::save: cannot open " + path);
  auto w32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char *>(&v), 4); };
  auto w64 = [&](std::uint64_t v) { f.write(reinterpret_cast<const char *>(&v), 8); };
  auto wf = [&](const std::vector<float> &v) {
    f.write(reinterpret_cast<const char *>(v.data()),
            static_cast<std::streamsize>(v.size() * sizeof(float)));
  };
  f.write("CVNV", 4);
  w32(fmt_);
  w32(flags_);
  w32(static_cast<std::uint32_t>(in_));
  w32(static_cast<std::uint32_t>(out_));
  w32(static_cast<std::uint32_t>(layers_.size()));
  w64(arch_hash_);
  for (const Layer &L : layers_) {
    w32(static_cast<std::uint32_t>(L.rows));
    w32(static_cast<std::uint32_t>(L.cols));
    w32(L.act);
    wf(L.w);
    wf(L.b);
  }
  // The file stores the RAW out_bias; recover it from the folded offset
  // (log(expm1(raw)) -> raw = softplus(off)) so save/load round-trips.
  std::vector<float> raw(out_bias_off_.size());
  for (std::size_t i = 0; i < raw.size(); ++i)
    raw[i] = (flags_ & kFlagSoftplusLogExpm1) ? softplus(out_bias_off_[i]) : out_bias_off_[i];
  w32(static_cast<std::uint32_t>(raw.size()));
  wf(raw);
  w32(static_cast<std::uint32_t>(meta.size()));
  if (!meta.empty())
    f.write(meta.data(), static_cast<std::streamsize>(meta.size()));
  if (!f)
    throw std::runtime_error("cvc::nav::coef_mlp::save: write failed for " + path);
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

namespace {
inline bool file_exists(const std::string &p) {
  std::ifstream f(p);
  return f.good();
}
} // namespace

std::string coef_mlp::default_weights_path() {
  if (const char *env = std::getenv("CVC_NAV_WEIGHTS")) {
    if (env[0] && file_exists(env))
      return std::string(env);
  }
  const std::string canonical = std::string(CVC_NAV_DATADIR) + "/coef_mlp.cvcnav";
  if (file_exists(canonical))
    return canonical;
#ifndef _WIN32
  // Relocated / cvcpkg install: resolve relative to the loaded libcvc library.
  Dl_info info;
  if (dladdr(reinterpret_cast<const void *>(&coef_mlp::default_weights_path), &info) &&
      info.dli_fname) {
    const std::string so = info.dli_fname;
    const std::size_t slash = so.find_last_of('/');
    if (slash != std::string::npos) {
      const std::string reloc = so.substr(0, slash) + "/../share/cvc/nav/coef_mlp.cvcnav";
      if (file_exists(reloc))
        return reloc;
    }
  }
#endif
  return canonical; // load() will report it if absent
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
