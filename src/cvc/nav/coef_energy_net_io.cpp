/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// coef_energy_net_io.cpp — .cvcnm serialization (the write side of
// coef_energy_net.cpp's load_from_memory). Lets a C++-trained CoefEnergyNetMaterial
// (P5 training) be checkpointed into the same container torch, the CPU forward,
// and the CUDA forward all read. Byte layout: little-endian, magic "CVNM", u32
// version, u64 arch_hash, u32 d_tok/nhead/num_layers/d_risk/patch_size, f32
// lam_soft_max/lam_hard_max/mu_lat_max/eps, u32 n_tensors, then per tensor
// (u32 name_len, name, u32 ndim, u32 dims..., f32 data...), u32 meta_len.

#include <cstring>
#include <cvc/nav/coef_energy_net.h>
#include <fstream>
#include <stdexcept>

namespace cvc {
namespace nav {

namespace {
void w32(std::vector<unsigned char> &b, std::uint32_t v) {
  unsigned char t[4];
  std::memcpy(t, &v, 4);
  b.insert(b.end(), t, t + 4);
}
void w64(std::vector<unsigned char> &b, std::uint64_t v) {
  unsigned char t[8];
  std::memcpy(t, &v, 8);
  b.insert(b.end(), t, t + 8);
}
void wf(std::vector<unsigned char> &b, float v) {
  unsigned char t[4];
  std::memcpy(t, &v, 4);
  b.insert(b.end(), t, t + 4);
}
} // namespace

std::vector<unsigned char> coef_energy_net::serialize() const {
  std::vector<unsigned char> b;
  b.insert(b.end(), {'C', 'V', 'N', 'M'});
  w32(b, kFormatVersion);
  w64(b, arch_hash_);
  w32(b, static_cast<std::uint32_t>(d_tok_));
  w32(b, static_cast<std::uint32_t>(nhead_));
  w32(b, static_cast<std::uint32_t>(num_layers_));
  w32(b, static_cast<std::uint32_t>(d_risk_));
  w32(b, static_cast<std::uint32_t>(patch_size_));
  wf(b, lam_soft_max_);
  wf(b, lam_hard_max_);
  wf(b, mu_lat_max_);
  wf(b, eps_);
  w32(b, static_cast<std::uint32_t>(tensors_.size()));
  for (const auto &kv : tensors_) {
    const std::string &name = kv.first;
    const tensor &tn = kv.second;
    w32(b, static_cast<std::uint32_t>(name.size()));
    b.insert(b.end(), name.begin(), name.end());
    w32(b, static_cast<std::uint32_t>(tn.dims.size()));
    for (int d : tn.dims)
      w32(b, static_cast<std::uint32_t>(d));
    for (float x : tn.data)
      wf(b, x);
  }
  w32(b, 0); // meta_len
  return b;
}

void coef_energy_net::save(const std::string &path) const {
  const std::vector<unsigned char> b = serialize();
  std::ofstream f(path, std::ios::binary);
  if (!f)
    throw std::runtime_error("coef_energy_net::save: cannot open " + path);
  f.write(reinterpret_cast<const char *>(b.data()), static_cast<std::streamsize>(b.size()));
  if (!f)
    throw std::runtime_error("coef_energy_net::save: write failed for " + path);
}

} // namespace nav
} // namespace cvc
