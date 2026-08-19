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

// drive.cpp — see drive.h. Float-equivalent (not bit-identical) transcription of
// the torch drive numerics (sdf_nav.py: SDFField.sample, coef_feats,
// bicycle_rollout). This TU must be built without -ffast-math /
// -ffp-contract=fast so the float32 op order tracks torch's as closely as
// possible: a fused-multiply-add or a fast reciprocal widens the residual past
// the ~1-ULP target, and the carrot FSM (a later phase) makes threshold
// decisions on these values.

#include <algorithm>
#include <cmath>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/drive.h>
#include <vector>

namespace cvc {
namespace nav {

namespace {

// normalized (centered) position -> [-1,1] grid coord, in float32 (SDFField.sample).
inline void grid_coords(const field_stack &f, float onx, float ony, float &gx, float &gy) {
  const float S = static_cast<float>(f.S);
  const float wx = onx / S + static_cast<float>(f.cx);
  const float wy = ony / S + static_cast<float>(f.cy);
  gx = 2.0f * (wx - static_cast<float>(f.mnx)) / static_cast<float>(f.mxx - f.mnx) - 1.0f;
  gy = 2.0f * (wy - static_cast<float>(f.mny)) / static_cast<float>(f.mxy - f.mny) - 1.0f;
}

// Bilinear sample of one field plane at grid (gx,gy) in [-1,1] — torch
// grid_sampler bilinear, align_corners=True, border padding, float32. Writes the
// three channels raw (phi, normal_x, normal_y before renorm).
inline void sample_plane(const field_stack &f, int plane, float gx, float gy, float &phi, float &nx,
                         float &ny) {
  const float Wf1 = static_cast<float>(f.W - 1);
  const float Hf1 = static_cast<float>(f.H - 1);
  float ix = (gx + 1.0f) * 0.5f * Wf1;
  float iy = (gy + 1.0f) * 0.5f * Hf1;
  ix = std::min(std::max(ix, 0.0f), Wf1);
  iy = std::min(std::max(iy, 0.0f), Hf1);
  const int ix0 = static_cast<int>(std::floor(ix));
  const int iy0 = static_cast<int>(std::floor(iy));
  const float wx1 = ix - static_cast<float>(ix0);
  const float wx0 = 1.0f - wx1;
  const float wy1 = iy - static_cast<float>(iy0);
  const float wy0 = 1.0f - wy1;
  const float nw = wx0 * wy0, ne = wx1 * wy0, sw = wx0 * wy1, se = wx1 * wy1;
  const int cx0 = std::min(std::max(ix0, 0), f.W - 1);
  const int cx1 = std::min(std::max(ix0 + 1, 0), f.W - 1);
  const int cy0 = std::min(std::max(iy0, 0), f.H - 1);
  const int cy1 = std::min(std::max(iy0 + 1, 0), f.H - 1);
  const long HW = static_cast<long>(f.H) * f.W;
  const float *pl = f.data + static_cast<long>(plane) * 3 * HW;
  const long nwi = static_cast<long>(cy0) * f.W + cx0;
  const long nei = static_cast<long>(cy0) * f.W + cx1;
  const long swi = static_cast<long>(cy1) * f.W + cx0;
  const long sei = static_cast<long>(cy1) * f.W + cx1;
  const float *ph = pl;
  const float *px = pl + HW;
  const float *py = pl + 2 * HW;
  phi = ph[nwi] * nw + ph[nei] * ne + ph[swi] * sw + ph[sei] * se;
  nx = px[nwi] * nw + px[nei] * ne + px[swi] * sw + px[sei] * se;
  ny = py[nwi] * nw + py[nei] * ne + py[swi] * sw + py[sei] * se;
}

// Sample phi + the UNIT outward normal at a normalized position (SDFField.sample).
inline void sample_unit(const field_stack &f, int plane, float onx, float ony, float &phi,
                        float &nx, float &ny) {
  float gx, gy, rnx, rny;
  grid_coords(f, onx, ony, gx, gy);
  sample_plane(f, plane, gx, gy, phi, rnx, rny);
  const float mag = std::sqrt(rnx * rnx + rny * rny) + 1e-6f;
  nx = rnx / mag;
  ny = rny / mag;
}

// IPC barrier derivative _ipc_dbdd (sdf_nav.py:292): d clamped to [1e-6,inf);
// zero at/above d_hat.
inline float ipc_dbdd(float d, float d_hat) {
  const float dc = d < 1e-6f ? 1e-6f : d; // clamp_min(1e-6)
  if (!(dc < d_hat))
    return 0.0f; // where(d < d_hat, val, 0)
  return (d_hat - dc) * (2.0f * std::log(dc / d_hat) - d_hat / dc) + 1.0f;
}

} // namespace

void sdf_sample(const field_stack &f, const float *on, int n, const int *map_id, float *phi_out,
                float *normal_out, int num_threads) {
  detail::parallel_for(n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float phi, nx, ny;
    sample_unit(f, plane, on[2 * i], on[2 * i + 1], phi, nx, ny);
    phi_out[i] = phi;
    normal_out[2 * i] = nx;
    normal_out[2 * i + 1] = ny;
  });
}

void coef_feats(const field_stack &f, const float *on, const float *goal, int n, const int *map_id,
                float *feat_out, int num_threads) {
  detail::parallel_for(n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float phi, nx, ny;
    sample_unit(f, plane, on[2 * i], on[2 * i + 1], phi, nx, ny);
    const float dx = goal[2 * i] - on[2 * i];
    const float dy = goal[2 * i + 1] - on[2 * i + 1];
    const float gd = std::sqrt(dx * dx + dy * dy);
    const float inv = 1.0f / (gd + 1e-6f);
    const float gdx = dx * inv, gdy = dy * inv;
    float *fo = feat_out + static_cast<std::size_t>(i) * 5;
    fo[0] = phi;
    fo[1] = gd;
    fo[2] = gdx;
    fo[3] = gdy;
    fo[4] = gdx * nx + gdy * ny; // gdir . unit_normal
  });
}

void bicycle_rollout(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                     const float *al, const float *be, const float *ga, int n, const int *map_id,
                     const veh_params &v, float *minclr_out, int num_threads) {
  const float hdt = v.dt / static_cast<float>(v.nsub);
  const float tan_dmax = std::tan(v.delta_max);
  const float rr = v.rr, d_hat = v.d_hat, vmax = v.vmax, L = v.L, dmax = v.delta_max;
  const float a_max = v.a_max, a_lat_max = v.a_lat_max, k_steer = v.k_steer;
  const float sp_min = v.allow_reverse ? -0.25f * vmax : 0.0f;
  const float v_creep_cap = 0.5f * std::sqrt(a_lat_max * L / tan_dmax);

  detail::parallel_for(n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float ox = o[2 * i], oy = o[2 * i + 1];
    float thi = th[i], spi = sp[i];
    const float gx = goal[2 * i], gy = goal[2 * i + 1];
    const float ali = al[i], bei = be[i], gai = ga[i];
    float minclr = 9.9f;

    for (int s = 0; s < v.nsub; ++s) {
      float phi, nx, ny;
      sample_unit(f, plane, ox, oy, phi, nx, ny);
      const float d = phi - rr;
      minclr = std::min(minclr, d);

      const float ipc = ipc_dbdd(d, d_hat);
      const float Fbar_x = -(ali * ipc) * nx;
      const float Fbar_y = -(ali * ipc) * ny;
      const float Fgoal_x = -bei * (ox - gx);
      const float Fgoal_y = -bei * (oy - gy);
      const float Fx = Fbar_x + Fgoal_x;
      const float Fy = Fbar_y + Fgoal_y;

      const float ch = std::cos(thi), sh = std::sin(thi);
      // head = (ch, sh); left = (-sh, ch)
      float a_long = (Fx * ch + Fy * sh) - gai * spi;
      a_long = std::min(std::max(a_long, -a_max), a_max);

      const float tgx = gx - ox, tgy = gy - oy;
      float L_d = std::sqrt(tgx * tgx + tgy * tgy);
      if (L_d < 1e-6f)
        L_d = 1e-6f;
      const float ang = std::atan2(tgy, tgx) - thi;
      const float sin_a = std::sin(ang);
      const float cos_a = std::cos(ang);
      const bool behind = cos_a < 0.0f;
      const float turn_sign = sin_a >= 0.0f ? 1.0f : -1.0f;
      float L_d_eff = 1.2f * spi + 4.0f * L;
      if (L_d_eff < 4.0f * L)
        L_d_eff = 4.0f * L;
      L_d_eff = std::min(L_d, L_d_eff);
      float delta = behind ? turn_sign * dmax : std::atan2(2.0f * L * sin_a, L_d_eff);

      // steering barrier bias: repulsive-only part of the IPC derivative.
      const float ipc_rep = ipc < 0.0f ? ipc : 0.0f; // clamp(max=0)
      const float Frep_x = -(ali * ipc_rep) * nx;
      const float Frep_y = -(ali * ipc_rep) * ny;
      delta = delta + k_steer * std::tanh(Frep_x * (-sh) + Frep_y * ch); // F_rep . left
      delta = std::min(std::max(delta, -dmax), dmax);

      float kappa = std::fabs(std::tan(delta)) / L;
      const float kfloor = tan_dmax / (L * 400.0f);
      if (kappa < kfloor)
        kappa = kfloor;
      const float v_corner = std::sqrt(a_lat_max / kappa);

      float ds = d - 0.5f * rr;
      if (ds < 0.0f)
        ds = 0.0f;
      const float v_stop = std::sqrt(2.0f * a_max * ds);
      const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
      float approach = -(nx * ch + ny * sh) * motion_sign;
      approach = std::min(std::max(approach, 0.0f), 1.0f);
      const float v_stop_dir = approach > 0.05f ? v_stop / std::max(approach, 0.05f) : vmax;
      float v_lim = std::min(vmax, std::min(v_corner, v_stop_dir));

      const bool hard_steer = std::fabs(delta) >= 0.7f * dmax;
      const bool can_move = d > 0.25f * rr;
      const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
      v_lim = std::max(v_lim, v_floor);

      const float v_creep = std::min(0.5f * v_corner, v_creep_cap);
      if (behind) {
        float fbh = Fbar_x * ch + Fbar_y * sh; // (F_bar . head), clamp(max=0)
        if (fbh > 0.0f)
          fbh = 0.0f;
        a_long = (v_creep - spi) / hdt + fbh;
      }
      const bool stuck_turning = hard_steer && can_move && std::fabs(spi) < 0.06f;
      if (stuck_turning)
        a_long = std::max(a_long, (0.08f - spi) / hdt);
      if (v.allow_reverse) {
        const float head_on = std::min(std::max(-(nx * ch + ny * sh), 0.0f), 1.0f);
        const bool nose_blocked = behind && (head_on > 0.6f) && (d < 0.5f * rr + 0.02f);
        if (nose_blocked) {
          a_long = (-0.10f - spi) / hdt;
          delta = -delta;
        }
      }
      a_long = std::min(std::max(a_long, -a_max), a_max);
      a_long = std::min(a_long, (v_lim - spi) / hdt);
      a_long = std::min(std::max(a_long, -a_max), a_max);

      spi = spi + hdt * a_long;
      spi = std::min(std::max(spi, sp_min), vmax);
      float sp2 = spi * spi;
      if (sp2 < 1e-9f)
        sp2 = 1e-9f;
      const float d_cap = std::atan(a_lat_max * L / sp2);
      delta = std::min(std::max(delta, -d_cap), d_cap);
      thi = thi + hdt * (spi / L) * std::tan(delta);
      const float ch2 = std::cos(thi), sh2 = std::sin(thi);
      ox = ox + hdt * spi * ch2;
      oy = oy + hdt * spi * sh2;
    }

    o[2 * i] = ox;
    o[2 * i + 1] = oy;
    th[i] = thi;
    sp[i] = spi;
    minclr_out[i] = minclr;
  });
}

void drive_step(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                float *minclr_out, int num_threads) {
  // sample + features -> coefficients -> rollout. The intermediate buffers are
  // O(n); a CUDA drive keeps them in registers and fuses this into one launch.
  std::vector<float> feat(static_cast<std::size_t>(n) * 5);
  coef_feats(f, o, carrot, n, map_id, feat.data(), num_threads);
  std::vector<float> coef(static_cast<std::size_t>(n) * 3);
  model.forward(feat.data(), n, coef.data(), num_threads);
  std::vector<float> al(n), be(n), ga(n);
  for (int i = 0; i < n; ++i) {
    al[i] = coef[3 * i + 0];
    be[i] = coef[3 * i + 1];
    ga[i] = coef[3 * i + 2];
  }
  bicycle_rollout(f, o, th, sp, carrot, al.data(), be.data(), ga.data(), n, map_id, v, minclr_out,
                  num_threads);
}

} // namespace nav
} // namespace cvc
