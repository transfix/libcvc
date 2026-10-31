/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstring>
#include <cvc/nav/dyn4_ledger.h>
#include <cvc/nav/rollout_plant.h>
#include <stdexcept>
#include <type_traits>

namespace cvc {
namespace nav {

namespace {

template <class S> S softplus_w(S x, S w) {
  const S q = x / w;
  if (diff::s_val(q) > 30.0)
    return x;
  return w * diff::s_log(S(1) + diff::s_exp(q));
}

template <class S>
S step_energy(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
              const dyn4_energy_price &price, const diff::dyn4_state<S> &s) {
  const diff::flat_ground ground{mu};
  const float full = dyn4_full_power(price, p, v);
  const S w = S(full > 0.0f ? 1e-3f * full : 1e-9f);
  const diff::dyn4_ph_lines<S> l = diff::dyn4_ph_lines_at(p, v, ground, s);
  return S(v.hdt) * (S(price.drive) * softplus_w(l.port, w) + S(price.idle));
}

constexpr char kModelMagic[8] = {'C', 'V', 'C', 'D', 'Y', 'N', '4', 'P'};

} // namespace

float dyn4_step_energy(const diff::dyn4_params &p, const diff::bike_veh &v, float mu,
                       const dyn4_energy_price &price, const diff::dyn4_state<float> &s, float *g) {
  if (g) {
    typedef diff::dual<float> D;
    const int n = diff::dyn4_state<float>::size;
    for (int j = 0; j < n; ++j) {
      diff::dyn4_state<D> ds;
      for (int k = 0; k < n; ++k)
        ds[k] = D(s[k], k == j ? 1.0f : 0.0f);
      g[j] = step_energy<D>(p, v, mu, price, ds).d;
    }
  }
  return step_energy<float>(p, v, mu, price, s);
}

void dyn4_rollout_plant::reset(const rollout_batch &b) {
  if (b.n < 0 || (b.n > 0 && (!b.o || !b.psi)))
    throw std::invalid_argument("cvc::nav::dyn4_rollout_plant::reset: needs n >= 0 starts");
  _v = b.veh;
  _p = b.model;
  _price = b.price;
  _mu = b.mu;
  _s.assign(static_cast<std::size_t>(b.n), diff::dyn4_state<float>{});
  for (int i = 0; i < b.n; ++i) {
    diff::dyn4_state<float> &s = _s[static_cast<std::size_t>(i)];
    for (int k = 0; k < diff::dyn4_state<float>::size; ++k)
      s[k] = 0.0f;
    s.x = b.o[2 * i];
    s.y = b.o[2 * i + 1];
    s.psi = b.psi[i];
    if (b.speed)
      s.vx = b.speed[i];
  }
  _lines.assign(_s.size(), plant_energy_lines{});
}

void dyn4_rollout_plant::step(const float *delta_cmd, const float *a_cmd) {
  const diff::flat_ground ground{_mu};
  for (std::size_t i = 0; i < _s.size(); ++i) {
    const diff::dyn4_state<float> s0 = _s[i];
    diff::dyn4_state<float> s1;
    diff::dyn4_plant<float>(_p, _v, ground, s0, delta_cmd[i], a_cmd[i], s1);
    plant_energy_lines l = dyn4_tick_lines(_p, _v, _mu, s0, s1);
    l.drawn = dyn4_step_energy(_p, _v, _mu, _price, s1);
    _lines[i] = l;
    _s[i] = s1;
  }
}

std::vector<std::uint8_t> dyn4_rollout_plant::save() const {
  static_assert(std::is_trivially_copyable<diff::dyn4_state<float>>::value &&
                    std::is_trivially_copyable<plant_energy_lines>::value,
                "save() copies plain values");
  std::vector<std::uint8_t> out(kModelMagic, kModelMagic + sizeof(kModelMagic));
  const std::uint64_t n = _s.size();
  const auto put = [&](const void *p, std::size_t bytes) {
    const auto *b = static_cast<const std::uint8_t *>(p);
    out.insert(out.end(), b, b + bytes);
  };
  put(&n, sizeof(n));
  put(_s.data(), _s.size() * sizeof(_s[0]));
  put(_lines.data(), _lines.size() * sizeof(_lines[0]));
  return out;
}

void dyn4_rollout_plant::restore(const std::vector<std::uint8_t> &state) {
  const std::size_t head = sizeof(kModelMagic) + sizeof(std::uint64_t);
  std::uint64_t n = 0;
  if (state.size() >= head) {
    std::memcpy(&n, state.data() + sizeof(kModelMagic), sizeof(n));
  }
  if (state.size() < head || std::memcmp(state.data(), kModelMagic, sizeof(kModelMagic)) != 0 ||
      n != _s.size() || state.size() != head + n * (sizeof(_s[0]) + sizeof(_lines[0])))
    throw std::invalid_argument(
        "cvc::nav::dyn4_rollout_plant::restore: not a state of this plant's batch");
  std::memcpy(_s.data(), state.data() + head, _s.size() * sizeof(_s[0]));
  std::memcpy(_lines.data(), state.data() + head + _s.size() * sizeof(_s[0]),
              _lines.size() * sizeof(_lines[0]));
}

} // namespace nav
} // namespace cvc
