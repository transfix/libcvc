/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// detail/material_ext.h: the material rollouts' external force (material_ext, material.h),
// step by step. ext_agent gives F_ext and J = dF/do at each step of one agent's rollout from
// whichever source the channel has; ext_reverse is the reverse's side, summing dL/dF_ext over
// a held window and chaining it through J into the position adjoint at the step it was
// sampled. Host only: the rollouts that use it run on the CPU.

#ifndef CVC_NAV_DETAIL_MATERIAL_EXT_H
#define CVC_NAV_DETAIL_MATERIAL_EXT_H

#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/material.h>
#include <stdexcept>
#include <string>

namespace cvc {
namespace nav {
namespace detail {

// Throws std::invalid_argument unless `e` is a well-formed channel for a batch of B agents
// over max_H steps (see material_ext). `g_fext` is a VJP's per-step output, which needs the
// same stride. `who` names the rollout in the message.
inline void validate_ext(const material_ext &e, int B, int max_H, const float *g_fext,
                         const char *who) {
  const auto bad = [&](const char *why) {
    throw std::invalid_argument(std::string(who) + ": material_ext " + why);
  };
  const int sources = (e.patch ? 1 : 0) + (e.F ? 1 : 0) + (e.sample ? 1 : 0);
  if (sources != 1)
    bad("needs exactly one source: patch, F or sample");
  if (e.J && !e.F)
    bad("gives J without F");
  if (e.hold < 1)
    bad("needs hold >= 1");
  if (e.patch && e.K < 1)
    bad("needs K >= 1");
  if ((e.F || e.patch_of_step || g_fext) && e.steps < max_H)
    bad("needs steps >= max(H) for its per-step arrays");
  if (e.patch && e.patch_of_step)
    for (long i = 0; i < static_cast<long>(B) * e.steps; ++i)
      if (e.patch_of_step[i] < 0 || e.patch_of_step[i] >= e.K)
        bad("has a patch index outside [0, K)");
}

// One agent's external force. at(s, ...) must visit the steps in order 0, 1, ...: the
// callback's held sample carries from one step to the next.
struct ext_agent {
  const material_ext *e;
  int b, Hp, Wp;
  float o0x, o0y;
  float hF[2] = {0.0f, 0.0f}, hJ[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int hat = 0;

  ext_agent(const material_ext *ext, int b_, int Hp_, int Wp_, float o0x_, float o0y_)
      : e(ext), b(b_), Hp(Hp_), Wp(Wp_), o0x(o0x_), o0y(o0y_) {}

  // F_ext and J at step s, the pose there (ox, oy); `sampled` is the step whose pose they
  // were sampled at (s, but for a held callback sample).
  void at(int s, float ox, float oy, float F[2], float J[4], int &sampled) {
    if (e->patch) {
      const int k = e->patch_of_step ? e->patch_of_step[static_cast<long>(b) * e->steps + s] : 0;
      const long chan = static_cast<long>(Hp) * Wp;
      const float *px = e->patch + (static_cast<long>(b) * e->K + k) * 2 * chan;
      const float *py = px + chan;
      const float offx = ox - o0x, offy = oy - o0y;
      F[0] = patch_sample_channel(px, Hp, Wp, offx, offy);
      F[1] = patch_sample_channel(py, Hp, Wp, offx, offy);
      patch_sample_channel_grad(px, Hp, Wp, offx, offy, J[0], J[1]);
      patch_sample_channel_grad(py, Hp, Wp, offx, offy, J[2], J[3]);
      sampled = s;
    } else if (e->F) {
      const long i = static_cast<long>(b) * e->steps + s;
      F[0] = e->F[2 * i];
      F[1] = e->F[2 * i + 1];
      for (int k = 0; k < 4; ++k)
        J[k] = e->J ? e->J[4 * i + k] : 0.0f;
      sampled = s;
    } else {
      if (s % e->hold == 0) {
        hF[0] = hF[1] = 0.0f;
        hJ[0] = hJ[1] = hJ[2] = hJ[3] = 0.0f;
        e->sample(e->user, b, s, ox, oy, hF, hJ);
        hat = s;
      }
      F[0] = hF[0];
      F[1] = hF[1];
      for (int k = 0; k < 4; ++k)
        J[k] = hJ[k];
      sampled = hat;
    }
  }
};

// The reverse's side: visit the steps T-1, ..., 0 after each step's own reverse, with that
// step's dL/dF_ext (gF), its record (sampled, J) and the position adjoint dL/do_s.
struct ext_reverse {
  float win[2] = {0.0f, 0.0f};
  void at(int s, int sampled, const float J[4], const float gF[2], float &aox, float &aoy) {
    win[0] += gF[0];
    win[1] += gF[1];
    if (sampled == s) {
      aox += win[0] * J[0] + win[1] * J[2];
      aoy += win[0] * J[1] + win[1] * J[3];
      win[0] = win[1] = 0.0f;
    }
  }
};

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // CVC_NAV_DETAIL_MATERIAL_EXT_H
