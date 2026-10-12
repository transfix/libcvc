/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// geom_rollout.cpp — see geom_rollout.h. The geometry rollout
// (integrate_surrogate_v2, explicit or semi-implicit Euler), its reverse-mode
// adjoint, and the multi-start robustness penalty L_multi. Reuses the CVC_HD IPC
// primitives from detail/material_rollout.h. Built with -ffp-contract=off (CMake).

#include <algorithm>
#include <cmath>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/geom_rollout.h>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

void integrate_surrogate_v2(float *o, float *v, const float *goal, const float *C, const float *R,
                            const std::uint8_t *mask, const float *alphas, const float *beta,
                            const float *gamma, const float *rr, const float *d_hat,
                            const float *dt, const int *H, int B, int N,
                            const geom_rollout_params &p, float *min_clear, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);

  detail::parallel_for(B, num_threads, [&](int b) {
    float ox = o[2 * b], oy = o[2 * b + 1], vx = v[2 * b], vy = v[2 * b + 1];
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], dh = d_hat[b], dtb = dt[b];
    const float rr_eff = p.margin_factor * rr[b];
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;
    float minclr = std::numeric_limits<float>::infinity();

    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
      float fbar_x = 0.0f, fbar_y = 0.0f, dmin = std::numeric_limits<float>::infinity();
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        fbar_x += -(ab[j] * dbdd) * nhx;
        fbar_y += -(ab[j] * dbdd) * nhy;
        if (d < dmin)
          dmin = d;
      }
      if (dmin < minclr)
        minclr = dmin;
      const float ax = (fbar_x + fgoal_x - ga * vx) / p.mass;
      const float ay = (fbar_y + fgoal_y - ga * vy) / p.mass;
      if (p.semi_implicit) {
        vx = vx + active * dtb * ax; // v_{n+1} first ...
        vy = vy + active * dtb * ay;
        ox = ox + active * dtb * vx; // ... then the position steps with it
        oy = oy + active * dtb * vy;
      } else {
        const float onx = ox + active * dtb * vx; // OLD v (explicit Euler)
        const float ony = oy + active * dtb * vy;
        vx = vx + active * dtb * ax;
        vy = vy + active * dtb * ay;
        ox = onx;
        oy = ony;
      }
    }
    o[2 * b] = ox;
    o[2 * b + 1] = oy;
    v[2 * b] = vx;
    v[2 * b + 1] = vy;
    min_clear[b] = minclr;
  });
}

void integrate_surrogate_v2_vjp(const float *o0, const float *v0, const float *goal, const float *C,
                                const float *R, const std::uint8_t *mask, const float *alphas,
                                const float *beta, const float *gamma, const float *rr,
                                const float *d_hat, const float *dt, const int *H, int B, int N,
                                const geom_rollout_params &p, const float *g_oT, const float *g_vT,
                                const float *g_min_clear, float *g_alphas, float *g_beta,
                                float *g_gamma, int num_threads) {
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);

  detail::parallel_for(B, num_threads, [&](int b) {
    const float gx = goal[2 * b], gy = goal[2 * b + 1];
    const float be = beta[b], ga = gamma[b], dh = d_hat[b], dtb = dt[b];
    const float rr_eff = p.margin_factor * rr[b];
    const float *Cb = C + static_cast<long>(b) * N * 2;
    const float *Rb = R + static_cast<long>(b) * N;
    const std::uint8_t *mb = mask + static_cast<long>(b) * N;
    const float *ab = alphas + static_cast<long>(b) * N;

    // forward recompute: store trajectory + locate the min_clear argmin
    std::vector<float> Ox(max_H + 1), Oy(max_H + 1), Vx(max_H + 1), Vy(max_H + 1);
    Ox[0] = o0[2 * b];
    Oy[0] = o0[2 * b + 1];
    Vx[0] = v0[2 * b];
    Vy[0] = v0[2 * b + 1];
    float minclr = std::numeric_limits<float>::infinity();
    int s_star = -1, j_star = -1;
    for (int s = 0; s < max_H; ++s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
      const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
      float fbar_x = 0.0f, fbar_y = 0.0f, dmin = std::numeric_limits<float>::infinity();
      int dmin_j = -1;
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        fbar_x += -(ab[j] * dbdd) * nhx;
        fbar_y += -(ab[j] * dbdd) * nhy;
        if (d < dmin) {
          dmin = d;
          dmin_j = j;
        }
      }
      if (dmin < minclr) {
        minclr = dmin;
        s_star = s;
        j_star = dmin_j;
      }
      const float ax = (fbar_x + fgoal_x - ga * vx) / p.mass;
      const float ay = (fbar_y + fgoal_y - ga * vy) / p.mass;
      Vx[s + 1] = vx + active * dtb * ax;
      Vy[s + 1] = vy + active * dtb * ay;
      if (p.semi_implicit) {
        Ox[s + 1] = ox + active * dtb * Vx[s + 1];
        Oy[s + 1] = oy + active * dtb * Vy[s + 1];
      } else {
        Ox[s + 1] = ox + active * dtb * vx;
        Oy[s + 1] = oy + active * dtb * vy;
      }
    }

    const float gmin = g_min_clear ? g_min_clear[b] : 0.0f;
    float aox = g_oT ? g_oT[2 * b] : 0.0f, aoy = g_oT ? g_oT[2 * b + 1] : 0.0f;
    float avx = g_vT ? g_vT[2 * b] : 0.0f, avy = g_vT ? g_vT[2 * b + 1] : 0.0f;

    for (int s = max_H - 1; s >= 0; --s) {
      const float active = (s < H[b]) ? 1.0f : 0.0f;
      const float c = active * dtb;
      const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
      // Semi-implicit: o_{s+1} = o_s + c*v_{s+1}, so v_{s+1}'s total adjoint
      // also collects c*aox through the position step. Explicit: o_{s+1} = o_s +
      // c*v_s, so that term lands on v_s instead.
      const float avpx = p.semi_implicit ? avx + c * aox : avx;
      const float avpy = p.semi_implicit ? avy + c * aoy : avy;
      // gF = adjoint of the force F (a = F/mass; v_{s+1} = v_s + c*a)
      const float gF_x = (avpx * c) / p.mass, gF_y = (avpy * c) / p.mass;
      float go_x = aox, go_y = aoy; // d o_{s+1}/d o_s = 1
      // d v_{s+1}/d v_s (+ d o_{s+1}/d v_s when explicit)
      float gv_x = p.semi_implicit ? avpx : aox * c + avx;
      float gv_y = p.semi_implicit ? avpy : aoy * c + avy;
      // F = fbar + fgoal - ga*v
      g_gamma[b] += -(gF_x * vx + gF_y * vy);
      gv_x += -ga * gF_x;
      gv_y += -ga * gF_y;
      g_beta[b] += -(gF_x * (ox - gx) + gF_y * (oy - gy));
      go_x += -be * gF_x;
      go_y += -be * gF_y;
      for (int j = 0; j < N; ++j) {
        if (!mb[j])
          continue;
        const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
        const float r0 = std::sqrt(dx * dx + dy * dy);
        const float r = r0 < 1e-9f ? 1e-9f : r0;
        const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
        const float dbdd = detail::ipc_dbdd_pw(d, dh);
        g_alphas[b * N + j] += -(gF_x * dbdd * nhx + gF_y * dbdd * nhy);
        const float g_dbdd = -ab[j] * (gF_x * nhx + gF_y * nhy);
        const float g_nhx = -(ab[j] * dbdd) * gF_x, g_nhy = -(ab[j] * dbdd) * gF_y;
        const float g_d = g_dbdd * detail::ipc_dbdd_pw_grad(d, dh);
        go_x += g_d * nhx;
        go_y += g_d * nhy;
        if (r0 > 1e-9f) {
          const float dot = nhx * g_nhx + nhy * g_nhy;
          go_x += (g_nhx - nhx * dot) / r;
          go_y += (g_nhy - nhy * dot) / r;
        }
      }
      if (s == s_star && j_star >= 0) {
        const float dx = ox - Cb[2 * j_star], dy = oy - Cb[2 * j_star + 1];
        float r = std::sqrt(dx * dx + dy * dy);
        if (r < 1e-9f)
          r = 1e-9f;
        go_x += gmin * (dx / r);
        go_y += gmin * (dy / r);
      }
      aox = go_x;
      aoy = go_y;
      avx = gv_x;
      avy = gv_y;
    }
  });
}

namespace {

void check_frac_range(const multi_start_params &p) {
  if (!(0.0 <= p.frac_lo && p.frac_lo <= p.frac_hi && p.frac_hi < 1.0))
    throw std::invalid_argument(
        "cvc::nav::multi_start: the frac range must satisfy 0 <= frac_lo <= frac_hi < 1, got (" +
        std::to_string(p.frac_lo) + ", " + std::to_string(p.frac_hi) + ")");
}

// K back-to-back copies of a [B, per] block: rollout k's agent b is row k*B + b.
template <class T> std::vector<T> tile(const T *src, int B, int per, int K) {
  const std::size_t n = static_cast<std::size_t>(B) * per;
  std::vector<T> out(n * K);
  for (int k = 0; k < K; ++k)
    std::copy(src, src + n, out.begin() + n * k);
  return out;
}

} // namespace

void multi_start_fracs(const multi_start_params &p, int B, float *fracs) {
  check_frac_range(p);
  if (p.ms_count <= 0 || B <= 0)
    return;
  const std::size_t n = static_cast<std::size_t>(p.ms_count) * B;
  const float lo = static_cast<float>(p.frac_lo);
  if (!(p.frac_hi > p.frac_lo)) {
    std::fill(fracs, fracs + n, lo); // degenerate: no draw
    return;
  }
  // torch.rand's CPU float32 draw: at::mt19937 (seeded like std::mt19937) feeds
  // uniform_real_distribution<float>, which keeps the LOW 24 bits. Then the
  // Python's lo + (hi - lo) * u, with both scalars rounded to float.
  const float span = static_cast<float>(p.frac_hi - p.frac_lo);
  std::mt19937 eng(p.seed);
  for (std::size_t i = 0; i < n; ++i) {
    const float u = static_cast<float>(eng() & 0xFFFFFFu) * (1.0f / 16777216.0f);
    const float t = span * u;
    fracs[i] = lo + t;
  }
}

double multi_start_penalty(const float *alphas, const float *beta, const float *gamma,
                           const float *o0, const float *v0, const float *goal, const float *C,
                           const float *R, const std::uint8_t *mask, const float *rr,
                           const float *d_hat, const float *dt, const int *H, int B, int N,
                           const multi_start_params &p, float *g_alphas, float *g_beta,
                           float *g_gamma, int num_threads) {
  check_frac_range(p);
  if (N == 0 || B == 0 || p.ms_count <= 0)
    return 0.0;
  // A degenerate range makes all ms_count starts the same start, and the mean of
  // identical rollouts is that rollout: evaluate it once.
  const int K = (p.frac_hi > p.frac_lo) ? p.ms_count : 1;
  const int KB = K * B;
  multi_start_params pk = p;
  pk.ms_count = K;
  std::vector<float> fracs(static_cast<std::size_t>(KB));
  multi_start_fracs(pk, B, fracs.data());

  // Nearest valid obstacle at o0 (data), once per agent.
  std::vector<float> dmin0(B, std::numeric_limits<float>::infinity()), n0(2 * B, 0.0f);
  for (int b = 0; b < B; ++b) {
    const float rr_eff = p.margin_factor * rr[b];
    const float ox = o0[2 * b], oy = o0[2 * b + 1];
    for (int j = 0; j < N; ++j) {
      if (!mask[b * N + j])
        continue;
      const float dx = ox - C[(b * N + j) * 2], dy = oy - C[(b * N + j) * 2 + 1];
      float r = std::sqrt(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      const float d = r - (R[b * N + j] + rr_eff);
      if (d < dmin0[b]) {
        dmin0[b] = d;
        n0[2 * b] = dx / r;
        n0[2 * b + 1] = dy / r;
      }
    }
  }

  // The (data) start of rollout k, agent b: frac of the clearance toward the
  // nearest obstacle, with the feasibility fallback. v_ms = v0.
  std::vector<float> o_ms(2 * static_cast<std::size_t>(KB)), dt_ms(KB);
  std::vector<int> H_ms(KB, p.ms_h);
  for (int k = 0; k < K; ++k) {
    for (int b = 0; b < B; ++b) {
      const int i = k * B + b;
      const float rr_eff = p.margin_factor * rr[b];
      const float ox = o0[2 * b], oy = o0[2 * b + 1];
      float omsx = ox, omsy = oy;
      // No valid obstacle: no clearance to score (+inf -> softplus 0, zero
      // grad), so the start is moot. Stay at o0 rather than form inf * 0 = NaN,
      // which the adjoint would spread into this agent's beta/gamma grads.
      if (std::isfinite(dmin0[b])) {
        const float fd = fracs[i] * dmin0[b];
        const float stepx = fd * n0[2 * b], stepy = fd * n0[2 * b + 1];
        omsx = ox - stepx;
        omsy = oy - stepy;
        // feasibility: if o_ms penetrates any obstacle, step half AWAY instead
        float dms = std::numeric_limits<float>::infinity();
        for (int j = 0; j < N; ++j) {
          if (!mask[b * N + j])
            continue;
          const float dx = omsx - C[(b * N + j) * 2], dy = omsy - C[(b * N + j) * 2 + 1];
          float r = std::sqrt(dx * dx + dy * dy);
          if (r < 1e-9f)
            r = 1e-9f;
          const float d = r - (R[b * N + j] + rr_eff);
          if (d < dms)
            dms = d;
        }
        if (dms < 0.0f) {
          omsx = ox + 0.5f * stepx;
          omsy = oy + 0.5f * stepy;
        }
      }
      o_ms[2 * i] = omsx;
      o_ms[2 * i + 1] = omsy;
      dt_ms[i] = p.ms_dt_mult * dt[b];
    }
  }

  // K > 1: tile the per-agent inputs so all K*B rollouts run as one batch.
  const float *v0_k = v0, *goal_k = goal, *C_k = C, *R_k = R, *al_k = alphas, *be_k = beta,
              *ga_k = gamma, *rr_k = rr, *dh_k = d_hat;
  const std::uint8_t *mask_k = mask;
  std::vector<float> t_v0, t_goal, t_C, t_R, t_al, t_be, t_ga, t_rr, t_dh;
  std::vector<std::uint8_t> t_mask;
  if (K > 1) {
    v0_k = (t_v0 = tile(v0, B, 2, K)).data();
    goal_k = (t_goal = tile(goal, B, 2, K)).data();
    C_k = (t_C = tile(C, B, 2 * N, K)).data();
    R_k = (t_R = tile(R, B, N, K)).data();
    mask_k = (t_mask = tile(mask, B, N, K)).data();
    al_k = (t_al = tile(alphas, B, N, K)).data();
    be_k = (t_be = tile(beta, B, 1, K)).data();
    ga_k = (t_ga = tile(gamma, B, 1, K)).data();
    rr_k = (t_rr = tile(rr, B, 1, K)).data();
    dh_k = (t_dh = tile(d_hat, B, 1, K)).data();
  }

  const geom_rollout_params gp{p.margin_factor, p.mass, p.semi_implicit};
  std::vector<float> o_run(o_ms), v_run(v0_k, v0_k + 2 * static_cast<std::size_t>(KB)),
      min_clear(KB);
  integrate_surrogate_v2(o_run.data(), v_run.data(), goal_k, C_k, R_k, mask_k, al_k, be_k, ga_k,
                         rr_k, dh_k, dt_ms.data(), H_ms.data(), KB, N, gp, min_clear.data(),
                         num_threads);

  double L = 0.0;
  for (int i = 0; i < KB; ++i) {
    const float x = -min_clear[i] / p.tau;
    L += (x > 20.0f) ? x : std::log1p(std::exp(x)); // softplus(-clr/tau)
  }
  L /= (double)KB;

  if (g_alphas || g_beta || g_gamma) {
    std::vector<float> g_clr(KB);
    for (int i = 0; i < KB; ++i) {
      const float s = 1.0f / (1.0f + std::exp(-(-min_clear[i] / p.tau))); // sigmoid(-clr/tau)
      g_clr[i] = -s / (p.tau * (float)KB);                                // d L / d clr
    }
    const std::vector<float> v_ms(v0_k, v0_k + 2 * static_cast<std::size_t>(KB));
    if (K == 1 && g_alphas && g_beta && g_gamma) {
      integrate_surrogate_v2_vjp(o_ms.data(), v_ms.data(), goal_k, C_k, R_k, mask_k, al_k, be_k,
                                 ga_k, rr_k, dh_k, dt_ms.data(), H_ms.data(), KB, N, gp, nullptr,
                                 nullptr, g_clr.data(), g_alphas, g_beta, g_gamma, num_threads);
    } else {
      // Per-rollout grads, then summed over the K copies of each agent.
      std::vector<float> s_al(static_cast<std::size_t>(KB) * N, 0.0f), s_be(KB, 0.0f),
          s_ga(KB, 0.0f);
      integrate_surrogate_v2_vjp(o_ms.data(), v_ms.data(), goal_k, C_k, R_k, mask_k, al_k, be_k,
                                 ga_k, rr_k, dh_k, dt_ms.data(), H_ms.data(), KB, N, gp, nullptr,
                                 nullptr, g_clr.data(), s_al.data(), s_be.data(), s_ga.data(),
                                 num_threads);
      for (int k = 0; k < K; ++k) {
        for (int b = 0; b < B; ++b) {
          const int i = k * B + b;
          if (g_alphas)
            for (int j = 0; j < N; ++j)
              g_alphas[b * N + j] += s_al[static_cast<std::size_t>(i) * N + j];
          if (g_beta)
            g_beta[b] += s_be[i];
          if (g_gamma)
            g_gamma[b] += s_ga[i];
        }
      }
    }
  }
  return L;
}

} // namespace nav
} // namespace cvc
