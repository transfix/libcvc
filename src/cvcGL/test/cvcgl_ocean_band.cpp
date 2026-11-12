/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ocean_band — the physics band residual (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, B2 and
// B6, O7b): how far the sea physics feels (cvc::ocean::sea_surface, the CPU) is from the sea
// lsystem_coast draws (OceanFFT, the GPU), and the band size M chosen against it.
//
// lsystem_coast's three cascades (251, 83 and 29 m tiles, the default 9 m/s wind sea) run on the
// GPU at 256 x 256 and are read back. On the CPU the same h0 runs through the physics band at
// several sizes, and the two are compared three ways:
//   1. On the GPU's own lattice (band.grid = 256): the CPU keeps the lowest M x M modes, so the
//      difference is the waves the band drops. With every mode kept it is only float arithmetic:
//      the check that the CPU's twin computes what the GPU computes.
//   2. Read between lattice points (the band on its own grid, by Catmull-Rom splines, at the GPU's
//      texels): the truncation and the splines together, what sea_surface::height gives physics.
//   3. The drawn sum: the three cascades' heights where the shader samples them (the texel nearest
//      the point, lsystem_coast's oDisp), against sea_surface's height over the same points. The
//      shader's nearest-texel steps put a floor under this that no band can go below; the band
//      with every mode measures it. The column for the shader's sampling before O7b (the texel at
//      or before the point, whose steps trail the field by half a texel) shows why it changed.
// It prints the table the roadmap records and fails if the CPU's twin disagrees with the GPU, if
// keeping more modes ever makes it worse, or if the default band misses the drawn sea by much more
// than that floor.

#include "OceanFFT.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/sea_surface.h>
#include <memory>
#include <vector>
#include <vtkOpenGLRenderWindow.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {

int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

constexpr int N = 256;         // lsystem_coast's FFT_N
constexpr double kChop = 1.15; // OceanFFT::chop, lsystem_coast's coast.water.chop default

// lsystem_coast's default wind sea, as its fftSea() gives the cascades (the swell is the shore
// layer's, not theirs).
cvc::ocean::sea_state coastSea() {
  cvc::ocean::sea_state s;
  s.wind.speed_mps = 9.0;
  s.wind.fetch_m = 120000.0;
  s.wind.dir_rad = std::atan2(0.55, 1.0);
  s.depth_m = 1000.0;
  s.swell.hs_m = 0.0;
  return s;
}

// Cascades as a sea: the band at `modes` x `modes` on a `grid` lattice, no sideways move (so
// height() is the cascades' height, read by the splines), no shore.
std::unique_ptr<cvc::ocean::sea_surface> seaOf(const std::vector<cvc::ocean::h0_field> &h0s,
                                               int modes, int grid) {
  cvc::ocean::sea_params p;
  p.band = {modes, grid};
  p.chop = 0.0;
  p.point_iterations = 0;
  std::vector<cvc::ocean::sea_cascade> cs;
  for (const auto &h0 : h0s)
    cs.push_back({h0, 0.0, 0.0});
  return std::make_unique<cvc::ocean::sea_surface>(std::move(cs), p);
}

struct Rms {
  double signal = 0.0, miss = 0.0;
  double ratio() const { return signal > 0.0 ? miss / signal : 0.0; }
};

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "oceanband");
  // A window smaller than the FFT on purpose: OceanFFT must not be clipped by the renderer's
  // scissor box (it disables the scissor test for its passes).
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.render(); // initialise the GL context before allocating float RTs
  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  if (!ow) {
    std::printf("FAIL: render window is not a vtkOpenGLRenderWindow (no OpenGL2 backend)\n");
    return 1;
  }

  const cvc::ocean::sea_state sea = coastSea();
  const std::vector<cvc::ocean::cascade> bands =
      cvc::ocean::make_cascades({251.0, 83.0, 29.0}, 0xC0A57ull);
  std::vector<cvc::ocean::h0_field> h0s;
  std::vector<std::unique_ptr<OceanFFT>> gpu;
  for (const cvc::ocean::cascade &b : bands) {
    h0s.push_back(cvc::ocean::build_h0(N, b, sea));
    gpu.push_back(std::make_unique<OceanFFT>(N));
    gpu.back()->depth_m = static_cast<float>(sea.depth_m);
    gpu.back()->chop = static_cast<float>(kChop);
    gpu.back()->setH0(h0s.back());
    if (!gpu.back()->init(ow)) {
      std::printf("SKIP: float render targets unavailable in this context\n");
      return 0;
    }
  }
  auto readback = [&](double t) {
    std::vector<std::vector<float>> tex;
    for (auto &g : gpu) {
      g->step(t);
      tex.push_back(g->readbackDisplacement());
    }
    return tex;
  };
  const char *names[3] = {"251 m", "83 m", "29 m"};

  // 1. On the GPU's lattice: the waves each band size drops.
  const int kModes[] = {8, 16, 32, 64, 128, 256};
  const double kTimes[] = {7.5, 37.25};
  std::vector<std::vector<Rms>> heightAt(3, std::vector<Rms>(6)), moveAt(3, std::vector<Rms>(6));
  for (double t : kTimes) {
    const std::vector<std::vector<float>> tex = readback(t);
    for (int c = 0; c < 3; ++c)
      for (int mi = 0; mi < 6; ++mi) {
        const auto s = seaOf({h0s[std::size_t(c)]}, kModes[mi], N);
        const cvc::ocean::sea_surface::band_frame f = s->band(0, t);
        for (int i = 0; i < N * N; ++i) {
          const float *g = &tex[std::size_t(c)][std::size_t(i) * 4];
          const double gh = g[1], gx = g[0] / kChop, gy = g[2] / kChop;
          heightAt[c][mi].signal += gh * gh;
          heightAt[c][mi].miss += (gh - f.eta[i]) * (gh - f.eta[i]);
          moveAt[c][mi].signal += gx * gx + gy * gy;
          moveAt[c][mi].miss += (gx - f.dx[i]) * (gx - f.dx[i]) + (gy - f.dy[i]) * (gy - f.dy[i]);
        }
      }
  }
  const double samples = double(N) * N * (sizeof kTimes / sizeof kTimes[0]);
  for (int c = 0; c < 3; ++c)
    for (int mi = 0; mi < 6; ++mi)
      for (Rms *r : {&heightAt[c][mi], &moveAt[c][mi]}) {
        r->signal = std::sqrt(r->signal / samples);
        r->miss = std::sqrt(r->miss / samples);
      }
  std::printf("\n1. The waves the band drops, on the GPU's lattice (height, RMS of GPU - CPU over "
              "the RMS height; %d x %d texels at t = 7.5 and 37.25 s):\n",
              N, N);
  std::printf("  %-7s %10s |", "cascade", "height rms");
  for (int m : kModes)
    std::printf("  M=%-5d", m);
  std::printf("\n");
  for (int c = 0; c < 3; ++c) {
    std::printf("  %-7s %8.4f m |", names[c], heightAt[c][0].signal);
    for (int mi = 0; mi < 6; ++mi)
      std::printf(" %6.2f%%", 100.0 * heightAt[c][mi].ratio());
    std::printf("\n");
  }
  for (int c = 0; c < 3; ++c) {
    char msg[200];
    std::snprintf(msg, sizeof msg,
                  "cascade %s: with every mode the CPU's height is the GPU's to %.4f%% and its "
                  "sideways move to %.4f%% (float arithmetic only)",
                  names[c], 100.0 * heightAt[c][5].ratio(), 100.0 * moveAt[c][5].ratio());
    check(heightAt[c][5].ratio() < 1e-3 && moveAt[c][5].ratio() < 1e-3, msg);
    bool monotone = true;
    for (int mi = 1; mi < 6; ++mi)
      monotone = monotone && heightAt[c][mi].miss <= heightAt[c][mi - 1].miss * 1.0001 + 1e-9;
    std::snprintf(msg, sizeof msg, "cascade %s: keeping more modes never misses by more", names[c]);
    check(monotone, msg);
  }
  check(heightAt[0][2].ratio() < 1e-3 && heightAt[1][2].ratio() < 1e-3,
        "the 251 m and 83 m cascades fit in 32 modes: nothing lost but float arithmetic");

  // 2-3. Between lattice points, and the drawn sum, for the band sizes worth choosing from (and
  // every mode, the floor the shader's sampling sets).
  struct Choice {
    int modes, grid;
  };
  const Choice kChoices[] = {{32, 64}, {32, 128}, {64, 64}, {64, 128}, {256, 256}};
  const double t = 37.25;
  const std::vector<std::vector<float>> tex = readback(t);
  // The drawn sum at points spread over the 251 m tile (a prime step, so they fall all over each
  // cascade's texels): each cascade at the texel nearest the point, as the shader samples it, or
  // at the texel whose corner is at or before it, as it sampled before O7b.
  const int P = 211;
  auto gpuSum = [&](double x, double y, bool centred) {
    double h = 0.0;
    for (int c = 0; c < 3; ++c) {
      const double L = bands[std::size_t(c)].tile_m, off = centred ? 0.5 : 0.0;
      const double u = x / L * N + off, v = y / L * N + off;
      const int i = int(((long long)std::floor(u) % N + N) % N);
      const int j = int(((long long)std::floor(v) % N + N) % N);
      h += tex[std::size_t(c)][(std::size_t(j) * N + std::size_t(i)) * 4 + 1];
    }
    return h;
  };
  std::printf("\n2-3. Read between lattice points (Catmull-Rom, per cascade, at the GPU's texels) "
              "and the drawn sum (the three cascades where the shader samples them), at t = %.2f "
              "s:\n",
              t);
  std::printf("  %-12s %-25s %-34s %s\n", "band", "per cascade 251/83/29 m", "drawn sum",
              "drawn as before O7b (corner texel)");
  const cvc::ocean::band_params def{};
  double chosenMiss = 0.0, floorMiss = 0.0, sumRms = 0.0;
  for (const Choice &ch : kChoices) {
    double perCascade[3];
    for (int c = 0; c < 3; ++c) {
      const auto s = seaOf({h0s[std::size_t(c)]}, ch.modes, ch.grid);
      const double L = bands[std::size_t(c)].tile_m;
      double sig = 0.0, miss = 0.0;
      for (int r = 0; r < N; r += 2)
        for (int col = 0; col < N; col += 2) {
          const double g = tex[std::size_t(c)][(std::size_t(r) * N + std::size_t(col)) * 4 + 1];
          const double cpu = s->height(col * L / N, r * L / N, t);
          sig += g * g;
          miss += (g - cpu) * (g - cpu);
        }
      perCascade[c] = sig > 0.0 ? std::sqrt(miss / sig) : 0.0;
    }
    const auto whole = seaOf(h0s, ch.modes, ch.grid);
    double sig = 0.0, miss = 0.0, missC = 0.0;
    for (int j = 0; j < P; ++j)
      for (int i = 0; i < P; ++i) {
        const double x = i * 251.0 / P + 0.37, y = j * 251.0 / P + 0.61;
        const double cpu = whole->height(x, y, t);
        const double g = gpuSum(x, y, true), gc = gpuSum(x, y, false);
        sig += g * g;
        miss += (g - cpu) * (g - cpu);
        missC += (gc - cpu) * (gc - cpu);
      }
    const double n = double(P) * P;
    const double rmsSig = std::sqrt(sig / n), rmsMiss = std::sqrt(miss / n),
                 rmsMissC = std::sqrt(missC / n);
    std::printf("  %3d on %-5d %6.2f%% %6.2f%% %6.2f%%   %6.2f%% (%5.1f mm of %.3f m)   %6.2f%% "
                "(%5.1f mm)\n",
                ch.modes, ch.grid, 100.0 * perCascade[0], 100.0 * perCascade[1],
                100.0 * perCascade[2], 100.0 * rmsMiss / rmsSig, 1000.0 * rmsMiss, rmsSig,
                100.0 * rmsMissC / rmsSig, 1000.0 * rmsMissC);
    if (ch.modes == def.modes && ch.grid == def.grid) {
      chosenMiss = rmsMiss;
      sumRms = rmsSig;
    }
    if (ch.modes == N)
      floorMiss = rmsMiss;
  }
  std::printf("\n");
  // The default band (band_params' default) is held to within 10 % of the floor: what it drops is
  // small beside the steps of the shader's own sampling.
  char msg[200];
  std::snprintf(msg, sizeof msg,
                "the default band (%d modes on %d) misses the drawn sea by %.2f%% RMS, within 10%% "
                "of the floor every mode sets (%.2f%%)",
                def.modes, def.grid, 100.0 * chosenMiss / sumRms, 100.0 * floorMiss / sumRms);
  check(sumRms > 0.0 && chosenMiss <= 1.1 * floorMiss, msg);

  std::printf("%s: cvcgl_ocean_band (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
