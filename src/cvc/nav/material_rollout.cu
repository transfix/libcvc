/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// material_rollout.cu — the CUDA twin of material_rollout.cpp: the obstacle-list
// material surrogate rollout forward + VJP on the GPU, one thread per agent. The
// per-step math is the SAME detail/material_rollout.h CVC_HD (__host__ __device__)
// primitives the CPU path uses, so the CPU finite-difference gradcheck
// (nav_material_rollout_grad_test) validates this device adjoint too; the parity
// test (nav_material_rollout_cuda_test) confirms CUDA == CPU. Compiled with
// -fmad=false --prec-div=true --prec-sqrt=true --ftz=false (CMake) so the device
// float ops match the host (the nav .cu precise-math discipline).

#include <cmath>
#include <cuda_runtime.h>
#include <cvc/nav/detail/material_rollout.h>
#include <cvc/nav/material.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {
namespace nav {

namespace {

constexpr int MAX_H_CUDA = 64; // device backward stores the trajectory in local mem

void cuda_check(cudaError_t e, const char *what) {
  if (e != cudaSuccess)
    throw std::runtime_error(std::string("cvc::nav material CUDA: ") + what + ": " +
                             cudaGetErrorString(e));
}

__global__ void mat_fwd_k(float *o, float *v, const float *goal, const float *C, const float *R,
                          const unsigned char *mask, const float *alphas, const float *beta,
                          const float *gamma, const float *lam_soft, const float *lam_hard,
                          const float *patch, const float *rr, const float *d_hat, const float *dt,
                          const int *H, int B, int N, int Hp, int Wp, float margin_factor,
                          float mass, float d_hat_sdf, float k_sharp, int max_H, float *min_clear,
                          float *cum_risk, float *hard_count, float *arc_length) {
  const int b = blockIdx.x * blockDim.x + threadIdx.x;
  if (b >= B)
    return;
  const long patch_stride = static_cast<long>(6) * Hp * Wp, chan = static_cast<long>(Hp) * Wp;
  float ox = o[2 * b], oy = o[2 * b + 1], vx = v[2 * b], vy = v[2 * b + 1];
  const float gx = goal[2 * b], gy = goal[2 * b + 1];
  const float be = beta[b], ga = gamma[b], ls = lam_soft[b], lh = lam_hard[b];
  const float dh = d_hat[b], dtb = dt[b], rr_eff = margin_factor * rr[b];
  const float o0x = ox, o0y = oy;
  const float *pr = patch + static_cast<long>(b) * patch_stride;
  const float *pr_risk = pr, *pr_phi = pr + chan, *pr_grx = pr + 2 * chan;
  const float *pr_gry = pr + 3 * chan, *pr_gpx = pr + 4 * chan, *pr_gpy = pr + 5 * chan;
  const float *Cb = C + static_cast<long>(b) * N * 2;
  const float *Rb = R + static_cast<long>(b) * N;
  const unsigned char *mb = mask + static_cast<long>(b) * N;
  const float *ab = alphas + static_cast<long>(b) * N;
  float minclr = INFINITY, cum = 0.0f, hard = 0.0f, arc = 0.0f;

  for (int s = 0; s < max_H; ++s) {
    const float active = (s < H[b]) ? 1.0f : 0.0f;
    const float offx = ox - o0x, offy = oy - o0y;
    float risk_val = detail::patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
    float sdf_val = detail::patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
    const float grx = detail::patch_sample_channel(pr_grx, Hp, Wp, offx, offy);
    const float gry = detail::patch_sample_channel(pr_gry, Hp, Wp, offx, offy);
    const float gpx = detail::patch_sample_channel(pr_gpx, Hp, Wp, offx, offy);
    const float gpy = detail::patch_sample_channel(pr_gpy, Hp, Wp, offx, offy);
    risk_val = fminf(fmaxf(risk_val, 0.0f), 1.0f);
    sdf_val = fminf(fmaxf(sdf_val, 0.0f), 50.0f);
    const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
    float fgeom_x = 0.0f, fgeom_y = 0.0f, dmin = INFINITY;
    for (int j = 0; j < N; ++j) {
      if (!mb[j])
        continue;
      const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
      float r = sqrtf(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
      const float dbdd = detail::ipc_dbdd_pw(d, dh);
      fgeom_x += -(ab[j] * dbdd) * nhx;
      fgeom_y += -(ab[j] * dbdd) * nhy;
      if (d < dmin)
        dmin = d;
    }
    if (dmin < minclr)
      minclr = dmin;
    const float fsoft_x = -ls * grx, fsoft_y = -ls * gry;
    const float db = detail::sdf_barrier_db(sdf_val, k_sharp, d_hat_sdf);
    const float fhard_x = -lh * db * gpx, fhard_y = -lh * db * gpy;
    const float ftot_x = fgoal_x + fgeom_x + fsoft_x + fhard_x - ga * vx;
    const float ftot_y = fgoal_y + fgeom_y + fsoft_y + fhard_y - ga * vy;
    const float ax = ftot_x / mass, ay = ftot_y / mass;
    const float vnx = vx + active * dtb * ax, vny = vy + active * dtb * ay;
    const float onx = ox + active * dtb * vnx, ony = oy + active * dtb * vny;
    const float sdx = onx - ox, sdy = ony - oy;
    const float disp = sqrtf(sdx * sdx + sdy * sdy);
    arc += active * disp;
    cum += active * risk_val * disp;
    hard += active * (sdf_val < 1.0f ? 1.0f : 0.0f);
    vx = vnx;
    vy = vny;
    ox = onx;
    oy = ony;
  }
  o[2 * b] = ox;
  o[2 * b + 1] = oy;
  v[2 * b] = vx;
  v[2 * b + 1] = vy;
  min_clear[b] = minclr;
  cum_risk[b] = cum;
  hard_count[b] = hard;
  arc_length[b] = arc;
}

__global__ void mat_vjp_k(const float *o0, const float *v0, const float *goal, const float *C,
                          const float *R, const unsigned char *mask, const float *alphas,
                          const float *beta, const float *gamma, const float *lam_soft,
                          const float *lam_hard, const float *patch, const float *rr,
                          const float *d_hat, const float *dt, const int *H, int B, int N, int Hp,
                          int Wp, float margin_factor, float mass, float d_hat_sdf, float k_sharp,
                          int max_H, const float *g_oT, const float *g_vT, const float *g_min_clear,
                          const float *g_cum_risk, const float *g_arc, float *g_alphas,
                          float *g_beta, float *g_gamma, float *g_lam_soft, float *g_lam_hard) {
  const int b = blockIdx.x * blockDim.x + threadIdx.x;
  if (b >= B)
    return;
  const long patch_stride = static_cast<long>(6) * Hp * Wp, chan = static_cast<long>(Hp) * Wp;
  const float gx = goal[2 * b], gy = goal[2 * b + 1];
  const float be = beta[b], ga = gamma[b], ls = lam_soft[b], lh = lam_hard[b];
  const float dh = d_hat[b], dtb = dt[b], rr_eff = margin_factor * rr[b];
  const float o0x = o0[2 * b], o0y = o0[2 * b + 1];
  const float *pr = patch + static_cast<long>(b) * patch_stride;
  const float *pr_risk = pr, *pr_phi = pr + chan, *pr_grx = pr + 2 * chan;
  const float *pr_gry = pr + 3 * chan, *pr_gpx = pr + 4 * chan, *pr_gpy = pr + 5 * chan;
  const float *Cb = C + static_cast<long>(b) * N * 2;
  const float *Rb = R + static_cast<long>(b) * N;
  const unsigned char *mb = mask + static_cast<long>(b) * N;
  const float *ab = alphas + static_cast<long>(b) * N;

  float Ox[MAX_H_CUDA + 1], Oy[MAX_H_CUDA + 1], Vx[MAX_H_CUDA + 1], Vy[MAX_H_CUDA + 1];
  Ox[0] = o0x;
  Oy[0] = o0y;
  Vx[0] = v0[2 * b];
  Vy[0] = v0[2 * b + 1];
  float minclr = INFINITY;
  int s_star = -1, j_star = -1;
  for (int s = 0; s < max_H; ++s) {
    const float active = (s < H[b]) ? 1.0f : 0.0f;
    const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
    const float offx = ox - o0x, offy = oy - o0y;
    float risk_val = detail::patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
    float sdf_val = detail::patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
    const float grx = detail::patch_sample_channel(pr_grx, Hp, Wp, offx, offy);
    const float gry = detail::patch_sample_channel(pr_gry, Hp, Wp, offx, offy);
    const float gpx = detail::patch_sample_channel(pr_gpx, Hp, Wp, offx, offy);
    const float gpy = detail::patch_sample_channel(pr_gpy, Hp, Wp, offx, offy);
    risk_val = fminf(fmaxf(risk_val, 0.0f), 1.0f);
    sdf_val = fminf(fmaxf(sdf_val, 0.0f), 50.0f);
    const float fgoal_x = -be * (ox - gx), fgoal_y = -be * (oy - gy);
    float fgeom_x = 0.0f, fgeom_y = 0.0f, dmin = INFINITY;
    int dmin_j = -1;
    for (int j = 0; j < N; ++j) {
      if (!mb[j])
        continue;
      const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
      float r = sqrtf(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
      const float dbdd = detail::ipc_dbdd_pw(d, dh);
      fgeom_x += -(ab[j] * dbdd) * nhx;
      fgeom_y += -(ab[j] * dbdd) * nhy;
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
    const float db = detail::sdf_barrier_db(sdf_val, k_sharp, d_hat_sdf);
    const float ftot_x = fgoal_x + fgeom_x - ls * grx - lh * db * gpx - ga * vx;
    const float ftot_y = fgoal_y + fgeom_y - ls * gry - lh * db * gpy - ga * vy;
    const float ax = ftot_x / mass, ay = ftot_y / mass;
    Vx[s + 1] = vx + active * dtb * ax;
    Vy[s + 1] = vy + active * dtb * ay;
    Ox[s + 1] = ox + active * dtb * Vx[s + 1];
    Oy[s + 1] = oy + active * dtb * Vy[s + 1];
  }

  const float gmin = g_min_clear ? g_min_clear[b] : 0.0f;
  const float gcum = g_cum_risk ? g_cum_risk[b] : 0.0f;
  const float garc = g_arc ? g_arc[b] : 0.0f;
  float aox = g_oT ? g_oT[2 * b] : 0.0f, aoy = g_oT ? g_oT[2 * b + 1] : 0.0f;
  float avx = g_vT ? g_vT[2 * b] : 0.0f, avy = g_vT ? g_vT[2 * b + 1] : 0.0f;

  for (int s = max_H - 1; s >= 0; --s) {
    const float active = (s < H[b]) ? 1.0f : 0.0f;
    const float ox = Ox[s], oy = Oy[s], vx = Vx[s], vy = Vy[s];
    const float offx = ox - o0x, offy = oy - o0y;
    float risk_raw = detail::patch_sample_channel(pr_risk, Hp, Wp, offx, offy);
    float sdf_raw = detail::patch_sample_channel(pr_phi, Hp, Wp, offx, offy);
    const float grx = detail::patch_sample_channel(pr_grx, Hp, Wp, offx, offy);
    const float gry = detail::patch_sample_channel(pr_gry, Hp, Wp, offx, offy);
    const float gpx = detail::patch_sample_channel(pr_gpx, Hp, Wp, offx, offy);
    const float gpy = detail::patch_sample_channel(pr_gpy, Hp, Wp, offx, offy);
    const float risk_val = fminf(fmaxf(risk_raw, 0.0f), 1.0f);
    const float sdf_val = fminf(fmaxf(sdf_raw, 0.0f), 50.0f);
    const float db = detail::sdf_barrier_db(sdf_val, k_sharp, d_hat_sdf);

    const float onx = Ox[s + 1], ony = Oy[s + 1];
    const float sdx = onx - ox, sdy = ony - oy;
    const float disp = sqrtf(sdx * sdx + sdy * sdy);
    const float g_disp = garc * active + gcum * active * risk_val;
    float ddx = 0.0f, ddy = 0.0f;
    if (disp > 0.0f) {
      ddx = sdx / disp;
      ddy = sdy / disp;
    }
    float go_x = -g_disp * ddx, go_y = -g_disp * ddy;
    const float g_risk_val = gcum * active * disp;

    const float aon_x = aox + g_disp * ddx, aon_y = aoy + g_disp * ddy;
    go_x += aon_x;
    go_y += aon_y;
    const float gvn_x = avx + aon_x * active * dtb, gvn_y = avy + aon_y * active * dtb;
    float gv_x = gvn_x, gv_y = gvn_y;
    const float ga_x = gvn_x * active * dtb, ga_y = gvn_y * active * dtb;
    const float gftot_x = ga_x / mass, gftot_y = ga_y / mass;

    g_gamma[b] += -(gftot_x * vx + gftot_y * vy);
    gv_x += -ga * gftot_x;
    gv_y += -ga * gftot_y;
    g_beta[b] += -(gftot_x * (ox - gx) + gftot_y * (oy - gy));
    go_x += -be * gftot_x;
    go_y += -be * gftot_y;
    g_lam_soft[b] += -(gftot_x * grx + gftot_y * gry);
    float g_grx = -ls * gftot_x, g_gry = -ls * gftot_y;
    g_lam_hard[b] += -(gftot_x * db * gpx + gftot_y * db * gpy);
    const float g_db = -lh * (gftot_x * gpx + gftot_y * gpy);
    float g_gpx = -lh * db * gftot_x, g_gpy = -lh * db * gftot_y;
    float g_sdf_val = g_db * detail::sdf_barrier_db_grad(sdf_val, k_sharp, d_hat_sdf);

    for (int j = 0; j < N; ++j) {
      if (!mb[j])
        continue;
      const float dx = ox - Cb[2 * j], dy = oy - Cb[2 * j + 1];
      const float r0 = sqrtf(dx * dx + dy * dy);
      const float r = r0 < 1e-9f ? 1e-9f : r0;
      const float nhx = dx / r, nhy = dy / r, d = r - (Rb[j] + rr_eff);
      const float dbdd = detail::ipc_dbdd_pw(d, dh);
      g_alphas[b * N + j] += -(gftot_x * dbdd * nhx + gftot_y * dbdd * nhy);
      const float g_dbdd = -ab[j] * (gftot_x * nhx + gftot_y * nhy);
      const float g_nhx = -(ab[j] * dbdd) * gftot_x, g_nhy = -(ab[j] * dbdd) * gftot_y;
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
      float r = sqrtf(dx * dx + dy * dy);
      if (r < 1e-9f)
        r = 1e-9f;
      go_x += gmin * (dx / r);
      go_y += gmin * (dy / r);
    }
    const float g_risk_raw = g_risk_val * ((risk_raw > 0.0f && risk_raw < 1.0f) ? 1.0f : 0.0f);
    const float g_sdf_raw = g_sdf_val * ((sdf_raw > 0.0f && sdf_raw < 50.0f) ? 1.0f : 0.0f);
    const float *chs[6] = {pr_risk, pr_phi, pr_grx, pr_gry, pr_gpx, pr_gpy};
    const float gcs[6] = {g_risk_raw, g_sdf_raw, g_grx, g_gry, g_gpx, g_gpy};
    for (int c = 0; c < 6; ++c) {
      float jx, jy;
      detail::patch_sample_channel_grad(chs[c], Hp, Wp, offx, offy, jx, jy);
      go_x += gcs[c] * jx;
      go_y += gcs[c] * jy;
    }
    aox = go_x;
    aoy = go_y;
    avx = gv_x;
    avy = gv_y;
  }
}

// device-buffer RAII-ish helper
template <class T> T *dmalloc(std::size_t n) {
  T *p = nullptr;
  cuda_check(cudaMalloc(&p, n * sizeof(T)), "malloc");
  return p;
}
template <class T> void h2d(T *d, const T *h, std::size_t n) {
  cuda_check(cudaMemcpy(d, h, n * sizeof(T), cudaMemcpyHostToDevice), "H2D");
}
template <class T> void d2h(T *h, const T *d, std::size_t n) {
  cuda_check(cudaMemcpy(h, d, n * sizeof(T), cudaMemcpyDeviceToHost), "D2H");
}

} // namespace

bool material_rollout_cuda_available() {
  int c = 0;
  return cudaGetDeviceCount(&c) == cudaSuccess && c > 0;
}

void integrate_surrogate_material_cuda(
    float *o, float *v, const float *goal, const float *C, const float *R, const std::uint8_t *mask,
    const float *alphas, const float *beta, const float *gamma, const float *lam_soft,
    const float *lam_hard, const float *rollout_patch, const float *rr, const float *d_hat,
    const float *dt, const int *H, int B, int N, int Hp, int Wp, const surrogate_material_params &p,
    float *min_clear, float *cum_risk, float *hard_count, float *arc_length) {
  if (!material_rollout_cuda_available())
    throw std::runtime_error("integrate_surrogate_material_cuda: no CUDA device");
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);
  const long patch_n = static_cast<long>(B) * 6 * Hp * Wp, bn = static_cast<long>(B) * N;
  float *d_o = dmalloc<float>(2 * B), *d_v = dmalloc<float>(2 * B), *d_goal = dmalloc<float>(2 * B);
  float *d_C = dmalloc<float>(bn * 2), *d_R = dmalloc<float>(bn), *d_al = dmalloc<float>(bn);
  float *d_be = dmalloc<float>(B), *d_ga = dmalloc<float>(B), *d_ls = dmalloc<float>(B),
        *d_lh = dmalloc<float>(B);
  float *d_patch = dmalloc<float>(patch_n), *d_rr = dmalloc<float>(B), *d_dh = dmalloc<float>(B),
        *d_dt = dmalloc<float>(B);
  unsigned char *d_mask = dmalloc<unsigned char>(bn);
  int *d_H = dmalloc<int>(B);
  float *d_mc = dmalloc<float>(B), *d_cr = dmalloc<float>(B), *d_hc = dmalloc<float>(B),
        *d_arc = dmalloc<float>(B);
  h2d(d_o, o, 2 * B);
  h2d(d_v, v, 2 * B);
  h2d(d_goal, goal, 2 * B);
  h2d(d_C, C, bn * 2);
  h2d(d_R, R, bn);
  h2d(d_al, alphas, bn);
  h2d(d_be, beta, B);
  h2d(d_ga, gamma, B);
  h2d(d_ls, lam_soft, B);
  h2d(d_lh, lam_hard, B);
  h2d(d_patch, rollout_patch, patch_n);
  h2d(d_rr, rr, B);
  h2d(d_dh, d_hat, B);
  h2d(d_dt, dt, B);
  h2d(d_mask, reinterpret_cast<const unsigned char *>(mask), bn);
  h2d(d_H, H, B);
  const int T = 128, G = (B + T - 1) / T;
  mat_fwd_k<<<G, T>>>(d_o, d_v, d_goal, d_C, d_R, d_mask, d_al, d_be, d_ga, d_ls, d_lh, d_patch,
                      d_rr, d_dh, d_dt, d_H, B, N, Hp, Wp, p.margin_factor, p.mass, p.d_hat_sdf,
                      p.k_sharp, max_H, d_mc, d_cr, d_hc, d_arc);
  cuda_check(cudaGetLastError(), "fwd launch");
  cuda_check(cudaDeviceSynchronize(), "fwd sync");
  d2h(o, d_o, 2 * B);
  d2h(v, d_v, 2 * B);
  d2h(min_clear, d_mc, B);
  d2h(cum_risk, d_cr, B);
  d2h(hard_count, d_hc, B);
  d2h(arc_length, d_arc, B);
  for (void *pp : {(void *)d_o,     (void *)d_v,  (void *)d_goal, (void *)d_C,  (void *)d_R,
                   (void *)d_al,    (void *)d_be, (void *)d_ga,   (void *)d_ls, (void *)d_lh,
                   (void *)d_patch, (void *)d_rr, (void *)d_dh,   (void *)d_dt, (void *)d_mask,
                   (void *)d_H,     (void *)d_mc, (void *)d_cr,   (void *)d_hc, (void *)d_arc})
    cudaFree(pp);
}

void integrate_surrogate_material_vjp_cuda(
    const float *o0, const float *v0, const float *goal, const float *C, const float *R,
    const std::uint8_t *mask, const float *alphas, const float *beta, const float *gamma,
    const float *lam_soft, const float *lam_hard, const float *rollout_patch, const float *rr,
    const float *d_hat, const float *dt, const int *H, int B, int N, int Hp, int Wp,
    const surrogate_material_params &p, const float *g_oT, const float *g_vT,
    const float *g_min_clear, const float *g_cum_risk, const float *g_arc_length, float *g_alphas,
    float *g_beta, float *g_gamma, float *g_lam_soft, float *g_lam_hard) {
  if (!material_rollout_cuda_available())
    throw std::runtime_error("integrate_surrogate_material_vjp_cuda: no CUDA device");
  int max_H = 0;
  for (int b = 0; b < B; ++b)
    max_H = std::max(max_H, H[b]);
  if (max_H > MAX_H_CUDA)
    throw std::runtime_error("integrate_surrogate_material_vjp_cuda: max(H) > 64 unsupported");
  const long patch_n = static_cast<long>(B) * 6 * Hp * Wp, bn = static_cast<long>(B) * N;
  auto up = [&](const float *h, int n) -> float * {
    if (!h)
      return nullptr;
    float *d = dmalloc<float>(n);
    h2d(d, h, n);
    return d;
  };
  float *d_o0 = up(o0, 2 * B), *d_v0 = up(v0, 2 * B), *d_goal = up(goal, 2 * B);
  float *d_C = up(C, bn * 2), *d_R = up(R, bn), *d_al = up(alphas, bn);
  float *d_be = up(beta, B), *d_ga = up(gamma, B), *d_ls = up(lam_soft, B), *d_lh = up(lam_hard, B);
  float *d_patch = up(rollout_patch, patch_n), *d_rr = up(rr, B), *d_dh = up(d_hat, B),
        *d_dt = up(dt, B);
  unsigned char *d_mask = dmalloc<unsigned char>(bn);
  h2d(d_mask, reinterpret_cast<const unsigned char *>(mask), bn);
  int *d_H = dmalloc<int>(B);
  h2d(d_H, H, B);
  float *d_goT = up(g_oT, 2 * B), *d_gvT = up(g_vT, 2 * B), *d_gmc = up(g_min_clear, B),
        *d_gcr = up(g_cum_risk, B), *d_garc = up(g_arc_length, B);
  // grad outputs seeded from host (the ADD-into contract)
  float *d_gal = up(g_alphas, bn), *d_gbe = up(g_beta, B), *d_gga = up(g_gamma, B),
        *d_gls = up(g_lam_soft, B), *d_glh = up(g_lam_hard, B);

  const int T = 128, G = (B + T - 1) / T;
  mat_vjp_k<<<G, T>>>(d_o0, d_v0, d_goal, d_C, d_R, d_mask, d_al, d_be, d_ga, d_ls, d_lh, d_patch,
                      d_rr, d_dh, d_dt, d_H, B, N, Hp, Wp, p.margin_factor, p.mass, p.d_hat_sdf,
                      p.k_sharp, max_H, d_goT, d_gvT, d_gmc, d_gcr, d_garc, d_gal, d_gbe, d_gga,
                      d_gls, d_glh);
  cuda_check(cudaGetLastError(), "vjp launch");
  cuda_check(cudaDeviceSynchronize(), "vjp sync");
  d2h(g_alphas, d_gal, bn);
  d2h(g_beta, d_gbe, B);
  d2h(g_gamma, d_gga, B);
  d2h(g_lam_soft, d_gls, B);
  d2h(g_lam_hard, d_glh, B);
  for (void *pp :
       {(void *)d_o0,  (void *)d_v0,  (void *)d_goal, (void *)d_C,   (void *)d_R,     (void *)d_al,
        (void *)d_be,  (void *)d_ga,  (void *)d_ls,   (void *)d_lh,  (void *)d_patch, (void *)d_rr,
        (void *)d_dh,  (void *)d_dt,  (void *)d_mask, (void *)d_H,   (void *)d_goT,   (void *)d_gvT,
        (void *)d_gmc, (void *)d_gcr, (void *)d_garc, (void *)d_gal, (void *)d_gbe,   (void *)d_gga,
        (void *)d_gls, (void *)d_glh})
    if (pp)
      cudaFree(pp);
}

} // namespace nav
} // namespace cvc
