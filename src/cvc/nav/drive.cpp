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
#include <cvc/nav/material.h>
#include <stdexcept>
#include <utility>
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

// Bilinear sample of a single-plane grip raster (grl_snam.material.FrictionField
// .sample) — the same coordinate chain and weight order as sample_plane, one
// channel, no normalization. Returns mu; 1 is the reference dry surface.
inline float sample_grip(const friction_field &g, int plane, float onx, float ony) {
  const float S = static_cast<float>(g.S);
  const float wx = onx / S + static_cast<float>(g.cx);
  const float wy = ony / S + static_cast<float>(g.cy);
  const float gx =
      2.0f * (wx - static_cast<float>(g.mnx)) / static_cast<float>(g.mxx - g.mnx) - 1.0f;
  const float gy =
      2.0f * (wy - static_cast<float>(g.mny)) / static_cast<float>(g.mxy - g.mny) - 1.0f;
  const float Wf1 = static_cast<float>(g.W - 1);
  const float Hf1 = static_cast<float>(g.H - 1);
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
  const int cx0 = std::min(std::max(ix0, 0), g.W - 1);
  const int cx1 = std::min(std::max(ix0 + 1, 0), g.W - 1);
  const int cy0 = std::min(std::max(iy0, 0), g.H - 1);
  const int cy1 = std::min(std::max(iy0 + 1, 0), g.H - 1);
  const long HW = static_cast<long>(g.H) * g.W;
  const float *pl = g.data + static_cast<long>(plane) * HW;
  return pl[static_cast<long>(cy0) * g.W + cx0] * (wx0 * wy0) +
         pl[static_cast<long>(cy0) * g.W + cx1] * (wx1 * wy0) +
         pl[static_cast<long>(cy1) * g.W + cx0] * (wx0 * wy1) +
         pl[static_cast<long>(cy1) * g.W + cx1] * (wx1 * wy1);
}

// The virtual steer limit once the INNER wheel's mechanical lock binds
// (sdf_nav.ackermann_delta_max). `t <= 0` leaves delta_max exactly as passed.
inline float locked_delta_max(float L, float delta_max, float t) {
  if (!(t > 0.0f))
    return delta_max;
  return std::atan(L / (L / std::tan(delta_max) + 0.5f * t));
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

// CVC_USING_CUDA, not CVC_ENABLE_CUDA: the latter is a CMake variable, and the
// PUBLIC compile define the cvc target actually gets is CVC_USING_CUDA
// (src/cvc/CMakeLists.txt). Guarding on the wrong name meant BOTH this TU and
// drive.cu defined the symbol, which links fine on macOS (no CUDA, so drive.cu
// is never compiled) and fails on every platform that has it.
#ifndef CVC_USING_CUDA
// Without a CUDA build there is no device path at all; drive.cu supplies the
// real probe when there is.
bool drive_cuda_available() { return false; }
#endif

// Every per-agent kernel in this file costs a few MICROSECONDS an item: one
// bilinear sample, a 5-64-64-3 MLP, one integrator step. detail::parallel_for
// spawns and joins its pool on every call (~3.5ms on a 20-core box), so at demo
// fan-outs it charged an order of magnitude more to parallelise than to run --
// measured on nav_drive_step, 5 agents: 3.86ms threaded against 0.41ms inline.
// The crossover is around 4k agents, so that is the price of admission here.
constexpr int kDriveMinItemsPerThread = 4096;

// With an injected pool the per-call cost is a wake + barrier (microseconds, no
// std::thread spawn), so parallelising pays off immediately — pass grain 1 and let
// the pool split the fan-out. Without a pool we're back to spawn-per-call, where
// inline wins until ~kDriveMinItemsPerThread items (the table above).
template <class F>
inline void drive_parallel_for(thread_pool *pool, int n, int num_threads, F &&fn) {
  detail::parallel_for(pool, n, num_threads, std::forward<F>(fn),
                       pool ? 1 : kDriveMinItemsPerThread);
}

void sdf_sample(const field_stack &f, const float *on, int n, const int *map_id, float *phi_out,
                float *normal_out, int num_threads, thread_pool *pool) {
  drive_parallel_for(pool, n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float phi, nx, ny;
    sample_unit(f, plane, on[2 * i], on[2 * i + 1], phi, nx, ny);
    phi_out[i] = phi;
    normal_out[2 * i] = nx;
    normal_out[2 * i + 1] = ny;
  });
}

void coef_feats(const field_stack &f, const float *on, const float *goal, int n, const int *map_id,
                float *feat_out, int num_threads, const friction_field *grip, float mu_lookahead,
                int mu_probes, thread_pool *pool) {
  const bool has_grip = grip != nullptr && grip->data != nullptr;
  const std::size_t stride = has_grip ? 6 : 5;
  drive_parallel_for(pool, n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float phi, nx, ny;
    sample_unit(f, plane, on[2 * i], on[2 * i + 1], phi, nx, ny);
    const float dx = goal[2 * i] - on[2 * i];
    const float dy = goal[2 * i + 1] - on[2 * i + 1];
    const float gd = std::sqrt(dx * dx + dy * dy);
    const float inv = 1.0f / (gd + 1e-6f);
    const float gdx = dx * inv, gdy = dy * inv;
    float *fo = feat_out + static_cast<std::size_t>(i) * stride;
    fo[0] = phi;
    fo[1] = gd;
    fo[2] = gdx;
    fo[3] = gdy;
    fo[4] = gdx * nx + gdy * ny; // gdir . unit_normal
    if (has_grip) {
      // Worst grip from here to the carrot: min over the probe, inclusive of
      // `on`, clamped so it never reaches past the carrot. An unbounded probe
      // would make every agent in the world permanently believe it is on ice.
      const int gplane = (map_id && grip->M > 1) ? map_id[i] : 0;
      const float reach = gd < mu_lookahead ? gd : mu_lookahead;
      float worst = sample_grip(*grip, gplane, on[2 * i], on[2 * i + 1]);
      const int P = mu_probes < 1 ? 1 : mu_probes;
      for (int k = 1; k <= P; ++k) {
        const float t = (static_cast<float>(k) / static_cast<float>(P)) * reach;
        const float m = sample_grip(*grip, gplane, on[2 * i] + t * gdx, on[2 * i + 1] + t * gdy);
        worst = m < worst ? m : worst;
      }
      fo[5] = worst;
    }
  });
}

namespace {

// The rollout body, shared by the plain and material entry points. `mat` is
// nullptr for the plain path — the guarded material blocks then never execute
// and the float op sequence is EXACTLY the pre-material code (byte-identical;
// asserted by NavMaterial.NullMaterialIsByteIdentical). With material, the
// force joins the F-sum AND the steering bias — see material.h for why both.
void rollout_impl(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                  const float *al, const float *be, const float *ga, int n, const int *map_id,
                  const veh_params &v, const material_drive *mat, float *minclr_out,
                  int num_threads, thread_pool *pool = nullptr) {
  const float hdt = v.dt / static_cast<float>(v.nsub);
  const float rr = v.rr, d_hat = v.d_hat, vmax = v.vmax, L = v.L;
  // t == 0 returns delta_max unchanged, so tan_dmax and every threshold built
  // from it stay bit-identical on the legacy path.
  const float dmax = locked_delta_max(v.L, v.delta_max, v.track_width);
  const float tan_dmax = std::tan(dmax);
  const float a_max = v.a_max, a_lat_max = v.a_lat_max, k_steer = v.k_steer;
  const float sp_min = v.allow_reverse ? -0.25f * vmax : 0.0f;
  const float v_creep_cap = 0.5f * std::sqrt(a_lat_max * L / tan_dmax);
  const bool has_fp = v.n_body > 0 && v.body_offsets != nullptr;
  const bool has_grip = v.grip != nullptr && v.grip->data != nullptr;

  drive_parallel_for(pool, n, num_threads, [&](int i) {
    const int plane = map_id ? map_id[i] : 0;
    float ox = o[2 * i], oy = o[2 * i + 1];
    float thi = th[i], spi = sp[i];
    const float gx = goal[2 * i], gy = goal[2 * i + 1];
    const float ali = al[i], bei = be[i], gai = ga[i];
    float minclr = 9.9f;

    for (int s = 0; s < v.nsub; ++s) {
      // Hoisted above the sample because the footprint places its discs along
      // the heading. A value-identical move: cos/sin of the same thi.
      const float ch = std::cos(thi), sh = std::sin(thi);
      float nx, ny, d, gov_rr;
      float Fbar_x, Fbar_y, Frep_x = 0.0f, Frep_y = 0.0f;
      if (!has_fp) {
        float phi;
        sample_unit(f, plane, ox, oy, phi, nx, ny);
        d = phi - rr;
        const float ipc = ipc_dbdd(d, d_hat);
        Fbar_x = -(ali * ipc) * nx;
        Fbar_y = -(ali * ipc) * ny;
        const float ipc_rep = ipc < 0.0f ? ipc : 0.0f; // clamp(max=0)
        Frep_x = -(ali * ipc_rep) * nx;
        Frep_y = -(ali * ipc_rep) * ny;
        gov_rr = rr;
      } else {
        // Multi-disc footprint. Clearance is the MIN over discs (and the
        // governor then steers by THAT disc's normal — "am I driving into the
        // nearest wall" is a question about the binding disc); the barrier
        // force is their SUM, so the nose is pushed off a wall the rear axle
        // cannot see. Ties take the first disc, matching torch's argmin.
        d = 3.0e38f;
        nx = 0.0f;
        ny = 0.0f;
        Fbar_x = 0.0f;
        Fbar_y = 0.0f;
        for (int b = 0; b < v.n_body; ++b) {
          const float off = v.body_offsets[b];
          float bphi, bnx, bny;
          sample_unit(f, plane, ox + off * ch, oy + off * sh, bphi, bnx, bny);
          const float bd = bphi - v.body_rr;
          const float bipc = ipc_dbdd(bd, d_hat);
          const float alg = v.body_gain == 1.0f ? ali : ali * v.body_gain;
          Fbar_x += -(alg * bipc) * bnx;
          Fbar_y += -(alg * bipc) * bny;
          const float brep = bipc < 0.0f ? bipc : 0.0f;
          Frep_x += -(alg * brep) * bnx;
          Frep_y += -(alg * brep) * bny;
          if (bd < d) {
            d = bd;
            nx = bnx;
            ny = bny;
          }
        }
        gov_rr = v.body_rr;
      }
      minclr = std::min(minclr, d);

      // Friction-limited actuator envelope. BOTH limits are grip-limited, so
      // both scale with mu together — that coupling is what makes ice read as
      // ice. No grip => the constants pass through untouched, not `* 1.0f`.
      float a_max_e = a_max, a_lat_e = a_lat_max, creep_cap = v_creep_cap;
      if (has_grip) {
        const int gplane = (map_id && v.grip->M > 1) ? map_id[i] : 0;
        const float mu = sample_grip(*v.grip, gplane, ox, oy);
        a_max_e = a_max * mu;
        a_lat_e = a_lat_max * mu;
        creep_cap = 0.5f * std::sqrt(a_lat_e * (L / tan_dmax));
      }
      const float Fgoal_x = -bei * (ox - gx);
      const float Fgoal_y = -bei * (oy - gy);
      float Fx = Fbar_x + Fgoal_x;
      float Fy = Fbar_y + Fgoal_y;

      // Material forces (torch reference: sdf_nav._material_force). Sampled at
      // the current pose AFTER the geometry sample, matching the Python order.
      float Fmat_x = 0.0f, Fmat_y = 0.0f;
      if (mat) {
        const int mplane = (map_id && mat->stack->M > 1) ? map_id[i] : 0;
        float mrisk, mphi, mgrx, mgry, mgpx, mgpy;
        detail::material_sample_point(*mat->stack, mplane, ox, oy, mrisk, mphi, mgrx, mgry, mgpx,
                                      mgpy);
        (void)mrisk;
        // db = -sigmoid(k * (d_hat_m - phi_m)); F_hard = ((-lam)*db)*grad.
        const float db = -(1.0f / (1.0f + std::exp(-(mat->k_sharp * (mat->d_hat_m - mphi)))));
        const float ls = mat->lam_soft[i], lh = mat->lam_hard[i];
        const float fsx = -ls * mgrx;
        const float fsy = -ls * mgry;
        const float fhx = (-lh * db) * mgpx;
        const float fhy = (-lh * db) * mgpy;
        Fmat_x = fsx + fhx;
        Fmat_y = fsy + fhy;
        Fx = Fx + Fmat_x;
        Fy = Fy + Fmat_y;
      }

      // head = (ch, sh); left = (-sh, ch)   [ch/sh hoisted above the sample]
      float a_long = (Fx * ch + Fy * sh) - gai * spi;
      a_long = std::min(std::max(a_long, -a_max_e), a_max_e);

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

      // steering barrier bias: repulsive-only part of the IPC derivative,
      // computed with the sample above (summed per disc in the footprint path).
      if (mat) {
        // ((F_rep + F_mat) . left) — material steers, not only brakes.
        const float sx = Frep_x + Fmat_x;
        const float sy = Frep_y + Fmat_y;
        delta = delta + k_steer * std::tanh(sx * (-sh) + sy * ch);
      } else {
        delta = delta + k_steer * std::tanh(Frep_x * (-sh) + Frep_y * ch); // F_rep . left
      }
      delta = std::min(std::max(delta, -dmax), dmax);

      float kappa = std::fabs(std::tan(delta)) / L;
      const float kfloor = tan_dmax / (L * 400.0f);
      if (kappa < kfloor)
        kappa = kfloor;
      const float v_corner = std::sqrt(a_lat_e / kappa);

      float ds = d - 0.5f * gov_rr;
      if (ds < 0.0f)
        ds = 0.0f;
      // The +1e-24f mirrors the torch reference, which needs it because
      // sqrt'(0) x clamp'(0) = 0 x inf = NaN poisons its backward pass. There is
      // no autograd here, but matching keeps the residual at zero rather than
      // ~1e-12 at the stop margin; the legacy branch keeps its exact form.
      const float v_stop = (has_fp || has_grip) ? std::sqrt(2.0f * a_max_e * ds + 1e-24f)
                                                : std::sqrt(2.0f * a_max * ds);
      const float motion_sign = spi >= 0.0f ? 1.0f : -1.0f;
      float approach = -(nx * ch + ny * sh) * motion_sign;
      approach = std::min(std::max(approach, 0.0f), 1.0f);
      const float v_stop_dir = approach > 0.05f ? v_stop / std::max(approach, 0.05f) : vmax;
      float v_lim = std::min(vmax, std::min(v_corner, v_stop_dir));

      const bool hard_steer = std::fabs(delta) >= 0.7f * dmax;
      const bool can_move = d > 0.25f * gov_rr;
      const float v_floor = (hard_steer && can_move) ? 0.08f : 0.0f;
      v_lim = std::max(v_lim, v_floor);

      const float v_creep = std::min(0.5f * v_corner, creep_cap);
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
        const bool nose_blocked = behind && (head_on > 0.6f) && (d < 0.5f * gov_rr + 0.02f);
        if (nose_blocked) {
          a_long = (-0.10f - spi) / hdt;
          delta = -delta;
        }
      }
      a_long = std::min(std::max(a_long, -a_max_e), a_max_e);
      a_long = std::min(a_long, (v_lim - spi) / hdt);
      a_long = std::min(std::max(a_long, -a_max_e), a_max_e);

      spi = spi + hdt * a_long;
      spi = std::min(std::max(spi, sp_min), vmax);
      float sp2 = spi * spi;
      if (sp2 < 1e-9f)
        sp2 = 1e-9f;
      const float d_cap = std::atan(a_lat_e * L / sp2);
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

} // namespace

void bicycle_rollout(const field_stack &f, float *o, float *th, float *sp, const float *goal,
                     const float *al, const float *be, const float *ga, int n, const int *map_id,
                     const veh_params &v, float *minclr_out, int num_threads, thread_pool *pool) {
  rollout_impl(f, o, th, sp, goal, al, be, ga, n, map_id, v, nullptr, minclr_out, num_threads,
               pool);
}

void bicycle_rollout_material(const field_stack &f, float *o, float *th, float *sp,
                              const float *goal, const float *al, const float *be, const float *ga,
                              int n, const int *map_id, const veh_params &v,
                              const material_drive &mat, float *minclr_out, int num_threads) {
  rollout_impl(f, o, th, sp, goal, al, be, ga, n, map_id, v, mat.stack ? &mat : nullptr, minclr_out,
               num_threads);
}

void carrot_step(const float *o, const float *goal, const float *th, float *sp, const float *phi,
                 const float *nrm, const fsm_state &s, int n, const carrot_params &p,
                 float *carrot_out, int num_threads, thread_pool *pool) {
  (void)phi; // phi is sampled with nrm by the caller; the FSM uses only nrm
  constexpr int SEEK = 0, WALL = 1;
  drive_parallel_for(pool, n, num_threads, [&](int i) {
    const float ox = o[2 * i], oy = o[2 * i + 1];
    const float gx = goal[2 * i], gy = goal[2 * i + 1];
    const float nx = nrm[2 * i], ny = nrm[2 * i + 1];
    const float dgx = gx - ox, dgy = gy - oy;
    const float dg = std::sqrt(dgx * dgx + dgy * dgy);
    const float inv = 1.0f / (dg + 1e-6f);
    const float gdx = dgx * inv, gdy = dgy * inv;
    const float tangx = -ny, tangy = nx; // wall tangent
    const bool tracked = s.tracking[i] && s.active[i];

    // Branch 1 — stall accounting (non-tracking closing test vs tracking ring).
    const bool closing = dg < s.best[i] - 1e-3f;
    const int stall_nt = closing ? 0 : s.stall[i] + 1;
    const float best_nt = closing ? dg : s.best[i];
    const int slot = s.hist_count[i] % 40;
    if (tracked) {
      s.pos_hist[i * 80 + slot * 2] = ox;
      s.pos_hist[i * 80 + slot * 2 + 1] = oy;
    }
    const int oldest = (s.hist_count[i] + 1) % 40;
    const float phx = s.pos_hist[i * 80 + oldest * 2];
    const float phy = s.pos_hist[i * 80 + oldest * 2 + 1];
    const float moved = std::sqrt((ox - phx) * (ox - phx) + (oy - phy) * (oy - phy));
    const bool have = s.hist_count[i] >= 40;
    const bool frozen = have && (moved < 0.15f) && (dg > p.reach_tol);
    const int stall_tk = have ? (frozen ? s.stall[i] + 1 : 0) : s.stall[i];
    const float best_tk = std::min(s.best[i], dg);
    if (tracked)
      s.hist_count[i] += 1;
    s.stall[i] = s.tracking[i] ? stall_tk : stall_nt;
    s.best[i] = s.tracking[i] ? best_tk : best_nt;

    // Branch 2 — seek -> wall entry.
    if (s.mode[i] == SEEK && s.stall[i] > 70) {
      s.dhit[i] = dg;
      s.wall_entry[2 * i] = ox;
      s.wall_entry[2 * i + 1] = oy;
      s.we_valid[i] = 1;
      s.turn[i] = (tangx * gdx + tangy * gdy) >= 0.0f ? 1.0f : -1.0f;
      s.mode[i] = WALL;
      s.stall[i] = 0;
    }

    // Branch 3 — carrot placement (uses the just-updated mode).
    float cx, cy;
    if (s.mode[i] == WALL) {
      const float twx = s.turn[i] * tangx, twy = s.turn[i] * tangy;
      cx = ox + (0.6f * twx + 0.4f * nx) * 1.6f;
      cy = oy + (0.6f * twy + 0.4f * ny) * 1.6f;
    } else {
      const float m = std::min(1.8f, dg);
      cx = ox + gdx * m;
      cy = oy + gdy * m;
    }

    // Branch 4 — wall exit (affects the next tick's mode).
    const float wex = ox - s.wall_entry[2 * i], wey = oy - s.wall_entry[2 * i + 1];
    const bool esc_tk = s.we_valid[i] && (std::sqrt(wex * wex + wey * wey) > 2.0f);
    const bool exit_tk = esc_tk || (s.stall[i] > 240);
    const bool exit_nt = (dg < s.dhit[i] - 1.2f) || (s.stall[i] > 240);
    if (s.mode[i] == WALL && (s.tracking[i] ? exit_tk : exit_nt)) {
      s.mode[i] = SEEK;
      s.best[i] = dg;
      s.stall[i] = 0;
    }

    // Branch 5 — parked (brake; carrot straight ahead so steering error is 0).
    if (s.parked[i]) {
      float spv = sp[i] - p.a_max * p.dt;
      if (spv < 0.0f)
        spv = 0.0f;
      sp[i] = spv;
      const float ax = std::cos(th[i]), ay = std::sin(th[i]);
      const float m = std::max(1e-3f, spv * 2.0f);
      cx = ox + ax * m;
      cy = oy + ay * m;
    }

    carrot_out[2 * i] = cx;
    carrot_out[2 * i + 1] = cy;
  });
}

void drive_step(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                float *minclr_out, int num_threads, thread_pool *pool) {
  // sample + features -> coefficients -> rollout. The intermediate buffers are
  // O(n); a CUDA drive keeps them in registers and fuses this into one launch.
  // The feature stride is the MODEL's, never an assumption: a net widened for
  // grip (sdf_nav.widen_coef_mlp) takes 6. A mismatch is a hard error rather
  // than a short buffer quietly feeding garbage into the first layer.
  const int in_w = model.in_features();
  if (in_w != 5 && in_w != 6)
    throw std::runtime_error("cvc::nav::drive_step: coef_mlp input width must be 5 or 6");
  const bool want_grip = in_w == 6;
  if (want_grip && !(v.grip && v.grip->data))
    throw std::runtime_error(
        "cvc::nav::drive_step: model takes 6 features but veh_params.grip is null");
  std::vector<float> feat(static_cast<std::size_t>(n) * in_w);
  coef_feats(f, o, carrot, n, map_id, feat.data(), num_threads, want_grip ? v.grip : nullptr,
             v.mu_lookahead, v.mu_probes, pool);
  std::vector<float> coef(static_cast<std::size_t>(n) * 3);
  model.forward(feat.data(), n, coef.data(), num_threads, pool);
  std::vector<float> al(n), be(n), ga(n);
  for (int i = 0; i < n; ++i) {
    al[i] = coef[3 * i + 0];
    be[i] = coef[3 * i + 1];
    ga[i] = coef[3 * i + 2];
  }
  bicycle_rollout(f, o, th, sp, carrot, al.data(), be.data(), ga.data(), n, map_id, v, minclr_out,
                  num_threads, pool);
}

void drive_step_material(const field_stack &f, float *o, float *th, float *sp, const float *carrot,
                         const coef_mlp &model, int n, const int *map_id, const veh_params &v,
                         const material_drive &mat, float *minclr_out, int num_threads) {
  // The feature stride is the MODEL's, never an assumption: a net widened for
  // grip (sdf_nav.widen_coef_mlp) takes 6. A mismatch is a hard error rather
  // than a short buffer quietly feeding garbage into the first layer.
  const int in_w = model.in_features();
  if (in_w != 5 && in_w != 6)
    throw std::runtime_error("cvc::nav::drive_step_material: coef_mlp input width must be 5 or 6");
  const bool want_grip = in_w == 6;
  if (want_grip && !(v.grip && v.grip->data))
    throw std::runtime_error(
        "cvc::nav::drive_step_material: model takes 6 features but veh_params.grip is null");
  std::vector<float> feat(static_cast<std::size_t>(n) * in_w);
  coef_feats(f, o, carrot, n, map_id, feat.data(), num_threads, want_grip ? v.grip : nullptr,
             v.mu_lookahead, v.mu_probes);
  std::vector<float> coef(static_cast<std::size_t>(n) * 3);
  model.forward(feat.data(), n, coef.data(), num_threads);
  std::vector<float> al(n), be(n), ga(n);
  for (int i = 0; i < n; ++i) {
    al[i] = coef[3 * i + 0];
    be[i] = coef[3 * i + 1];
    ga[i] = coef[3 * i + 2];
  }
  bicycle_rollout_material(f, o, th, sp, carrot, al.data(), be.data(), ga.data(), n, map_id, v, mat,
                           minclr_out, num_threads);
}

} // namespace nav
} // namespace cvc
