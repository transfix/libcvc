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

#include "OceanFFT.h"

#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
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
  OceanFFT ocean(N);
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
  const double rmsCalm = heightRMS(ocean, 3.0); // default wind (11 m/s)
  ocean.windSpeed = 25.0f;
  ocean.rebuildSpectrum();
  const double rmsStorm = heightRMS(ocean, 3.0);
  check(rmsStorm > rmsCalm * 1.2,
        "higher wind_speed -> larger wave RMS via rebuildSpectrum() (spectrum knob is live)");
  std::printf("  (calm rms=%.4f, storm rms=%.4f)\n", rmsCalm, rmsStorm);

  std::printf("%s: cvcgl_ocean_fft (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
