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

// coef_mlp.h — the torch-free coefficient policy for cvc::nav's drive (port P2).
//
// A libtorch-free forward pass of GRL-SNAM's CoefMLP (sdf_nav.py:536-555): a
// 5 -> hidden -> hidden -> 3 MLP with SiLU activations, whose output is
// softplus(net(feat) + log(expm1(bias))) — the (alpha, beta, gamma) the barrier
// / goal-spring / damping the vehicle rollout uses. The net is tiny (a few
// thousand floats), so a plain per-agent matmul is ample; float-equivalent to
// torch (~1e-4), not bit-identical (the sgemm reduction order differs) — the
// same fidelity tier as the sampler.
//
// Weights load from a versioned `.cvcnav` file (see docs/CVCNAV_CPP_PORT_ROADMAP
// §4), written from the trained torch model by grl_snam/tools/coef_export.py.
// The same blob feeds torch, this CPU forward, and a future CUDA forward; an
// arch_hash pins the architecture so a hidden-size change bumps the file. The
// canonical install location is $PREFIX/share/cvc/nav/coef_mlp.cvcnav.

#ifndef __CVC_NAV_COEF_MLP_H__
#define __CVC_NAV_COEF_MLP_H__

#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

class coef_mlp {
public:
  // The `.cvcnav` format this build reads/writes.
  static constexpr std::uint32_t kFormatVersion = 1;
  // flags bit 0: the output is softplus(net + log(expm1(out_bias))) — the
  // CoefMLP bias-toward-a-known-good-basin trick; the raw out_bias is stored and
  // log(expm1(.)) is folded once at load.
  static constexpr std::uint32_t kFlagSoftplusLogExpm1 = 1u << 0;

  // Load from a `.cvcnav` file / an in-memory blob. Throws std::runtime_error on
  // a bad magic, an unsupported format version, or a truncated/short buffer.
  static coef_mlp load(const std::string &path);
  static coef_mlp load_from_memory(const void *data, std::size_t nbytes);

  // The default policy location, resolved in order: the CVC_NAV_WEIGHTS env var,
  // then the baked $PREFIX/share/cvc/nav/coef_mlp.cvcnav, then a path relative to
  // the loaded libcvc library (for a relocated / cvcpkg install). Returns the
  // first that exists, else the baked canonical path (so load() reports it). A
  // pure-C++ host with no explicit weights path can `coef_mlp::load(coef_mlp::
  // default_weights_path())`.
  static std::string default_weights_path();

  // A default, bias-initialized policy (the shipped 5->64->64->3 topology with
  // zero linear weights), so a pure-C++ host can drive with NO trained weights
  // file: net(feat) == 0, so the coefficients are the constant CoefMLP bias
  // basin (alpha, beta, gamma) = (1, 3, 4) — the barrier / goal-spring / damping
  // that navigates out of the box. Load a trained .cvcnav for learned behavior.
  static coef_mlp default_biased();

  // Assemble a policy from explicit dense-layer weights — the torch-free trainer
  // (cvc::nav::coef_trainer) bakes one from its trained params. `w[i]`/`b[i]` are
  // layer i's row-major [rows[i]*cols[i]] / [rows[i]] weights; `act[i]` is 0
  // (identity) or 1 (SiLU). `out_bias_raw` is the RAW basin bias (e.g. {1,3,4});
  // softplus(net + log(expm1(out_bias))) is applied at forward, exactly as a
  // loaded .cvcnav. Throws if the shapes do not chain in -> ... -> out.
  static coef_mlp from_layers(int in, int out, const std::vector<int> &rows,
                              const std::vector<int> &cols, const std::vector<std::uint32_t> &act,
                              const std::vector<std::vector<float>> &w,
                              const std::vector<std::vector<float>> &b,
                              const std::vector<float> &out_bias_raw);

  // Serialize to the versioned `.cvcnav` (byte-identical layout to
  // grl_snam.tools.coef_export.write_coef_mlp), so a policy trained in pure C++
  // persists to the same file torch / the CPU / the CUDA forward all read.
  // Writes the RAW out_bias (log(expm1) is re-folded at load). Throws on I/O
  // failure. `meta` is optional provenance appended to the trailer.
  void save(const std::string &path, const std::string &meta = std::string()) const;

  // feats: [n * in_features()] row-major; out: [n * out_features()] row-major
  // (alpha, beta, gamma). Threaded across the n independent agents
  // (num_threads <= 0 => hardware concurrency).
  void forward(const float *feats, int n, float *out, int num_threads = 0) const;

  int in_features() const { return in_; }
  int out_features() const { return out_; }
  std::uint32_t format_version() const { return fmt_; }
  std::uint32_t flags() const { return flags_; }
  std::uint64_t arch_hash() const { return arch_hash_; }

  // The architecture hash (FNV-1a over in/out/num_layers and each layer's
  // rows/cols/activation) — matches the exporter, so a shape change is caught.
  static std::uint64_t compute_arch_hash(int in_features, int out_features,
                                         const std::vector<std::uint32_t> &layer_shape_act);

  // Flattened weights for a device (CUDA) forward: per-layer metadata + one
  // concatenated buffer (each layer's w[rows*cols] then b[rows]) + the folded
  // output bias offset. Same values the CPU forward() uses.
  struct flat_layers {
    int in = 0, out = 0, num_layers = 0;
    std::vector<int> rows, cols, act;
    std::vector<long> w_off, b_off; // offsets into `data`
    std::vector<float> data;
    std::vector<float> out_bias_off;
  };
  flat_layers export_flat() const;

private:
  struct Layer {
    int rows = 0, cols = 0;  // rows = out dim, cols = in dim (torch Linear [out,in])
    std::uint32_t act = 0;   // 0 = identity, 1 = SiLU
    std::vector<float> w, b; // w[rows*cols] row-major, b[rows]
  };
  std::vector<Layer> layers_;
  std::vector<float> out_bias_off_; // = log(expm1(out_bias)), folded once at load
  int in_ = 0, out_ = 0;
  std::uint32_t fmt_ = 0, flags_ = 0;
  std::uint64_t arch_hash_ = 0;

  void build_from_bytes(const std::uint8_t *p, std::size_t n);
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_COEF_MLP_H__
