/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ocean_fft — correctness tests for the spectral GPU FFT ocean (OceanFFT,
// the ABYSSAL port; docs/roadmap/OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md §A.9).
//
// Checks the invariants a BROKEN FFT would violate — a wrong butterfly index,
// twiddle sign, bit-reversal or conjugate-symmetry shows up as a non-zero-mean
// height, a Jacobian mean != 1, NaNs, or a field that does not respond to time
// or to the wind knob. All checks report explicitly and the program RETURNS
// non-zero on any failure: assert() is a NO-OP under NDEBUG (Release / cvcpkg),
// so a test that assert()s would pass vacuously (see cvcgl_volume_range.cpp).

#include "OceanCascades.h"
#include "OceanFFT.h"
#include "OceanFoam.h"
#include "OceanSpray.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/algo/wind_field.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/ocean/fetch.h>
#include <cvc/ocean/h0.h>
#include <cvc/ocean/shore.h>
#include <cvc/ocean/whitecaps.h>
#include <memory>
#include <vector>
#include <vtkOpenGLRenderWindow.h>
#include <vtkTextureObject.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

namespace {
int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

struct Stat {
  double mean = 0, rms = 0, mn = 0, mx = 0;
  bool finite = true;
};
Stat compStat(const std::vector<float> &d, int c, int stride) {
  double s = 0, s2 = 0, mn = 1e30, mx = -1e30;
  size_t n = 0;
  bool finite = true;
  for (size_t i = c; i < d.size(); i += stride) {
    float v = d[i];
    if (!std::isfinite(v)) {
      finite = false;
      continue;
    }
    s += v;
    s2 += double(v) * v;
    mn = std::min(mn, double(v));
    mx = std::max(mx, double(v));
    ++n;
  }
  Stat st;
  st.mean = n ? s / n : 0;
  st.rms = n ? std::sqrt(s2 / n) : 0;
  st.mn = mn;
  st.mx = mx;
  st.finite = finite;
  return st;
}
double heightRMS(OceanFFT &o, double t) {
  o.step(t);
  return compStat(o.readbackDisplacement(), 1, 4).rms; // channel 1 = height
}
} // namespace

int main() {
  // Offscreen GL context, the cvcGL-idiomatic way (mirrors cvcgl_shadow_volume).
  cvc::app app;
  SceneGraph sg(app, "octest");
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.render(); // initialise the GL context before allocating float RTs

  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  if (!ow) {
    std::printf("FAIL: render window is not a vtkOpenGLRenderWindow (no OpenGL2 backend)\n");
    return 1;
  }

  const int N = 64; // FFT correctness is N-independent; 64 keeps the test fast
  // One cascade over the whole band, h0 from the shared CPU spectrum (cvc/ocean/h0.h).
  cvc::ocean::cascade band;
  band.tile_m = 200.0;
  band.seed = 7;
  cvc::ocean::sea_state calm;
  calm.wind.speed_mps = 11.0;
  calm.wind.dir_rad = std::atan2(0.55, 1.0);
  const cvc::ocean::h0_field h0 = cvc::ocean::build_h0(N, band, calm);
  OceanFFT ocean(N);
  check(ocean.setH0(h0), "setH0 takes an h0 of the ocean's resolution (held until init)");
  check(!ocean.setH0(cvc::ocean::build_h0(N * 2, band, calm)), "setH0 refuses another resolution");
  if (!ocean.init(ow)) {
    // No float render targets here -> cannot exercise the pipeline. Treat as a
    // skip (0), not a failure: the demo's documented degrade path is a flat sea.
    std::printf("SKIP: float render targets unavailable in this context\n");
    return 0;
  }

  // 1. Phase-0: the multi-pass float-RT ping-pong reads back bit-exact.
  check(ocean.selfTest(), "selfTest: RGBA32F multi-pass ping-pong reads back bit-exact");

  // 2-3. FFT output invariants at a fixed time.
  ocean.step(2.0);
  const std::vector<float> d = ocean.readbackDisplacement();
  check(d.size() == static_cast<size_t>(N) * N * 4, "readback size == N*N*4");
  const Stat h = compStat(d, 1, 4); // height
  const Stat j = compStat(d, 3, 4); // Jacobian
  check(h.finite && j.finite, "displacement + Jacobian are finite (no NaN/Inf)");
  check(std::fabs(h.mean) < 0.05, "height field is zero-mean (a correct inverse FFT)");
  check(h.rms > 1e-3 && h.rms < 100.0, "height RMS is non-trivial and bounded");
  check((h.mx - h.mn) > 1e-2, "height field varies spatially (not constant / not a spike)");
  check(std::fabs(j.mean - 1.0) < 0.05, "Jacobian mean ~= 1.0 (undisturbed-surface invariant)");

  // 4. Determinism: the noise is hash-derived, so same params + same time must
  //    reproduce the field bit-for-bit (what makes a replay/benchmark trustworthy).
  OceanFFT ocean2(N);
  ocean2.setH0(cvc::ocean::build_h0(N, band, calm));
  if (ocean2.init(ow)) {
    ocean2.step(2.0);
    const std::vector<float> d2 = ocean2.readbackDisplacement();
    double maxdiff = 0;
    for (size_t i = 0; i < d.size() && i < d2.size(); ++i)
      maxdiff = std::max(maxdiff, double(std::fabs(d[i] - d2[i])));
    check(maxdiff < 1e-4, "two instances, same params + time -> identical field (deterministic)");
  }

  // 5. Time evolution: the surface must actually move with world time.
  ocean.step(2.5);
  const std::vector<float> dt = ocean.readbackDisplacement();
  double change = 0;
  for (size_t i = 1; i < d.size() && i < dt.size(); i += 4)
    change += std::fabs(dt[i] - d[i]);
  check(change > 1e-2, "height field evolves with world time");

  // 6. Live knob response: a stronger wind must respectralize to bigger waves.
  const double rmsCalm = heightRMS(ocean, 3.0); // 11 m/s
  cvc::ocean::sea_state storm = calm;
  storm.wind.speed_mps = 25.0;
  check(ocean.setH0(cvc::ocean::build_h0(N, band, storm)), "setH0 after init re-uploads h0");
  const double rmsStorm = heightRMS(ocean, 3.0);
  check(rmsStorm > rmsCalm * 1.2,
        "higher wind speed -> larger wave RMS via setH0() (the spectrum knob is live)");
  std::printf("  (calm rms=%.4f, storm rms=%.4f)\n", rmsCalm, rmsStorm);

  // 7. No h0: a flat sea (the degrade path before the spectrum is known).
  OceanFFT flat(N);
  if (flat.init(ow)) {
    flat.step(1.0);
    const Stat fh = compStat(flat.readbackDisplacement(), 1, 4);
    check(fh.finite && fh.rms == 0.0, "an ocean with no h0 is flat");
  }

  // 8. Persistent foam (OceanFoam, OCEAN-AND-WEATHER-ROADMAP.md section 10.2): a disc of tile 0
  //    folds (J 0.5) for half a second and then stops. Its foam builds while it folds and decays
  //    at its rate after, as cvc::ocean::foam_step says, frame by frame; tile 1 never folds.
  {
    const int FN = 32;
    std::vector<float> fold(static_cast<size_t>(FN) * FN * 4, 0.0f), calm = fold;
    for (int y = 0; y < FN; ++y)
      for (int x = 0; x < FN; ++x) {
        const size_t i = (static_cast<size_t>(y) * FN + x) * 4;
        const bool in = (x - 10) * (x - 10) + (y - 12) * (y - 12) < 25;
        fold[i + 3] = in ? 0.5f : 1.0f;
        calm[i + 3] = 1.0f;
      }
    auto dFold = oceanFloatTexture(ow, FN, FN, 4, fold.data());
    auto dCalm = oceanFloatTexture(ow, FN, FN, 4, calm.data());
    OceanFoam foam(FN, 2);
    foam.bias = 0.8;
    foam.ramp = 0.1;
    if (dFold && dCalm && foam.init(ow)) {
      cvc::ocean::foam_state cpu; // the disc's centre on the CPU
      const double dt = 1.0 / 60.0;
      for (int i = 0; i < 30; ++i) {
        foam.step({dFold, dCalm}, dt);
        cvc::ocean::foam_step(cpu, cvc::ocean::foam_inject(0.5, 0.8, 0.1), dt, foam.params);
      }
      const std::vector<float> a = foam.readback();
      auto at = [&](const std::vector<float> &f, int tile, int x, int y, int c) {
        return f[(static_cast<size_t>(y) * FN * 2 + tile * FN + x) * 4 + c];
      };
      check(a.size() == static_cast<size_t>(FN) * FN * 2 * 4, "the foam atlas is two tiles wide");
      check(std::fabs(at(a, 0, 10, 12, 0) - cpu.foam) < 1e-4 &&
                std::fabs(at(a, 0, 10, 12, 1) - cpu.bubbles) < 1e-4 && at(a, 0, 10, 12, 2) == 1.0f,
            "half a second of folding: foam, bubbles and crest as foam_step has them");
      check(at(a, 0, 25, 25, 0) == 0.0f && at(a, 1, 10, 12, 0) == 0.0f,
            "no foam where nothing folds, nor in the other cascade's tile");
      for (int i = 0; i < 150; ++i) { // 2.5 s with the fold gone
        foam.step({dCalm, dCalm}, dt);
        cvc::ocean::foam_step(cpu, 0.0, dt, foam.params);
      }
      const std::vector<float> b = foam.readback();
      std::printf("  (foam after the fold: GPU %.5f, CPU %.5f, e^-1 x before %.5f; bubbles %.5f / "
                  "%.5f; spray %.2e)\n",
                  at(b, 0, 10, 12, 0), cpu.foam, std::exp(-1.0) * at(a, 0, 10, 12, 0),
                  at(b, 0, 10, 12, 1), cpu.bubbles, at(b, 0, 10, 12, 3));
      check(
          std::fabs(at(b, 0, 10, 12, 0) - cpu.foam) < 1e-4 &&
              std::fabs(at(b, 0, 10, 12, 1) - cpu.bubbles) < 1e-4 && at(b, 0, 10, 12, 2) == 0.0f,
          "the fold gone, the foam persists and decays at its rate (e^-1 in 2.5 s), as on the CPU");
      check(at(b, 0, 10, 12, 0) > 0.3f && at(b, 0, 10, 12, 0) < 0.4f,
            "2.5 s after the fold the foam is still there, a third of what it was");
      foam.clear();
      check(foam.readback()[(static_cast<size_t>(12) * FN * 2 + 10) * 4] == 0.0f,
            "clear() empties the atlas");
    } else {
      std::printf("SKIP the foam: its float targets are unavailable\n");
    }
    // On a real sea: a storm's cascade folds somewhere, and its foam stays finite.
    OceanFoam sea(N, 1);
    sea.bias = 1.0;
    if (sea.init(ow)) {
      for (int i = 0; i < 20; ++i) {
        ocean.step(3.0 + i / 60.0);
        sea.step({ocean.displacement()}, 1.0 / 60.0);
      }
      const Stat fs = compStat(sea.readback(), 0, 4);
      std::printf("  (a storm's foam: mean %.4f, max %.4f)\n", fs.mean, fs.mx);
      check(fs.finite && fs.mx > 0.0 && fs.mx <= 1.0, "a storm's folds leave foam, within [0, 1]");
    }
  }

  // 9. OceanCascades: three cascades side by side in atlases, every pass doing all of them, give
  //    each cascade's OceanFFT field (the same h0, the same time), in 2 + 2 log2 N passes where
  //    three OceanFFTs take three times 3 + 4 log2 N. A foam atlas reads their Jacobians from the
  //    displacement atlas as from three textures.
  {
    const double tiles[3] = {251.0, 83.0, 29.0};
    OceanCascades all(N, 3);
    std::vector<std::unique_ptr<OceanFFT>> one;
    bool ok = all.init(ow);
    for (int c = 0; c < 3 && ok; ++c) {
      cvc::ocean::cascade b;
      b.tile_m = tiles[c];
      b.seed = 11 + c;
      const cvc::ocean::h0_field h = cvc::ocean::build_h0(N, b, calm);
      ok = all.setH0(c, h);
      one.push_back(std::make_unique<OceanFFT>(N));
      ok = ok && one.back()->setH0(h) && one.back()->init(ow);
    }
    check(!all.setH0(3, cvc::ocean::build_h0(N, band, calm)) &&
              !all.setH0(0, cvc::ocean::build_h0(N * 2, band, calm)),
          "OceanCascades refuses a fourth cascade or another resolution");
    if (ok) {
      const long long before = all.passesRun();
      all.step(2.25);
      const long long passes = all.passesRun() - before;
      const std::vector<float> atlas = all.readbackDisplacement();
      double worst = 0.0, scale = 0.0;
      for (int c = 0; c < 3; ++c) {
        one[c]->step(2.25);
        const std::vector<float> d = one[c]->readbackDisplacement();
        for (int y = 0; y < N; ++y)
          for (int x = 0; x < N; ++x)
            for (int k = 0; k < 4; ++k) {
              const float a = atlas[(static_cast<size_t>(y) * N * 3 + c * N + x) * 4 + k];
              const float b = d[(static_cast<size_t>(y) * N + x) * 4 + k];
              worst = std::max(worst, double(std::fabs(a - b)));
              scale = std::max(scale, double(std::fabs(b)));
            }
      }
      std::printf("  (OceanCascades vs three OceanFFTs: worst difference %.3g over a largest value "
                  "%.3g; %lld passes)\n",
                  worst, scale, passes);
      check(atlas.size() == static_cast<size_t>(N) * N * 3 * 4 && worst <= 1e-4 * scale,
            "OceanCascades' tiles are each cascade's OceanFFT field");
      check(passes == 2 + 2 * static_cast<long long>(std::lround(std::log2(N))),
            "all three cascades in 2 + 2 log2 N passes");
      OceanFoam foam3(N, 2);
      OceanFoam foam1(N, 2);
      if (foam3.init(ow) && foam1.init(ow)) {
        foam3.bias = foam1.bias = 1.0;
        for (int i = 0; i < 10; ++i) {
          foam3.step(all.displacement(), 1.0 / 60.0);
          foam1.step({one[0]->displacement(), one[1]->displacement()}, 1.0 / 60.0);
        }
        const std::vector<float> f3 = foam3.readback(), f1 = foam1.readback();
        double fw = 0.0;
        for (std::size_t i = 0; i < f3.size() && i < f1.size(); ++i)
          fw = std::max(fw, double(std::fabs(f3[i] - f1[i])));
        check(f3.size() == f1.size() && fw < 1e-4,
              "the foam reads the cascades' Jacobians from the atlas as from their textures");
      }
      // Two cascades active (adaptive quality's lower tiers): the first two tiles move on, each
      // still its OceanFFT field, and the third keeps what it held.
      all.setActive(2);
      all.step(3.5);
      const std::vector<float> two = all.readbackDisplacement();
      double worst2 = 0.0, kept = 0.0;
      for (int c = 0; c < 3; ++c) {
        std::vector<float> d;
        if (c < 2) {
          one[c]->step(3.5);
          d = one[c]->readbackDisplacement();
        }
        for (int y = 0; y < N; ++y)
          for (int x = 0; x < N; ++x)
            for (int k = 0; k < 4; ++k) {
              const std::size_t i = (static_cast<size_t>(y) * N * 3 + c * N + x) * 4 + k;
              if (c < 2)
                worst2 = std::max(
                    worst2,
                    double(std::fabs(two[i] - d[(static_cast<size_t>(y) * N + x) * 4 + k])));
              else
                kept = std::max(kept, double(std::fabs(two[i] - atlas[i])));
            }
      }
      check(all.active() == 2 && worst2 <= 1e-4 * scale && kept == 0.0,
            "with two active, the first two cascades step and the third is left alone");
      all.setActive(5);
      check(all.active() == 3, "setActive clamps to the cascades there are");
    }
  }

  // OceanSpray's inputs worked out on the CPU (section 10.5): the pulse's phases, from which the
  // shader works out the whole wind's pulse when a particle rose; the most a gust raises the wind
  // by; the grid of where the waves may break.
  {
    std::printf("== OceanSpray's inputs ==\n");
    double worst = 0.0;
    for (double t : {0.0, 17.3, 3600.7, 86400.25, 1.0e6 + 0.3})
      for (double age : {0.0, 0.9, 1.8}) {
        const std::array<float, 3> ph = oceanSprayPulse(t);
        const double p = 0.5 * std::sin(ph[0] - 0.21 * age) +
                         0.3 * std::sin(ph[1] - 0.21 * 2.37 * age) +
                         0.2 * std::sin(ph[2] - 0.21 * 5.1 * age);
        worst = std::max(worst, std::fabs(p - cvc::algo::gust_pulse(t - age)));
      }
    std::printf("  the pulse from sprayPulse, as the shader works it out: off by %.2e at most\n",
                worst);
    check(worst < 1e-5, "sprayPulse gives the wind's pulse when a particle rose (gust_pulse)");
    const cvc::algo::wind_state gusty{10.0, 0.4, 0.6};
    double most = 0.0;
    for (int i = 0; i < 20000; ++i)
      most = std::max(most, cvc::algo::wind_gust(gusty, 37.0 * i, -11.0 * i, 0.37 * i));
    std::printf("  the strongest gust of 20000 at gustiness 0.6: x %.3f (bound x %.3f)\n", most,
                oceanSprayGustMax(0.6));
    check(most <= oceanSprayGustMax(0.6), "no gust raises the wind past oceanSprayGustMax");
    // A 4 x 4 shore grid 10 m a cell: land in its first two columns, 1 m deep in the third, 50 m
    // in the fourth (and past it); the waves fade over 20 m of depth.
    cvc::ocean::shore_grid g;
    g.rows = g.cols = 4;
    g.cell = 10.0;
    for (int r = 0; r < 4; ++r)
      for (float d : {-1.0f, -1.0f, 1.0f, 50.0f})
        g.depth.push_back(d);
    std::array<float, kSprayGrid * kSprayGrid> open{}, lee{};
    oceanSprayWeights(g, nullptr, 0.0, 20.0, 0.0, 0.0, 10.0, open);
    cvc::ocean::fetch_field f; // 100 m of fetch everywhere, against 400 m fully grown
    f.rows = f.cols = 4;
    f.cell = 10.0;
    f.fetch_m.assign(16, 100.0f);
    oceanSprayWeights(g, &f, 400.0, 20.0, 0.0, 0.0, 10.0, lee);
    std::printf("  the weights along a row: land %.2f, 1 m deep %.2f, deep %.2f, past the grid "
                "%.2f; in the lee (a quarter of the full fetch) %.2f\n",
                open[0], open[2], open[3], open[15], lee[15]);
    check(open[0] == 0.0f && open[2] == 0.0f && std::fabs(open[3] - 1.0f) < 1e-6f &&
              std::fabs(open[15] - 1.0f) < 1e-6f && std::fabs(lee[15] - 0.5f) < 1e-6f,
          "the spray's weights: none on land or in the shallows, all on deep water, sqrt(F / "
          "full) in the lee");
  }

  std::printf("%s: cvcgl_ocean_fft (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
