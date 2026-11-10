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
#include <cvc/ocean/h0.h>
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

  std::printf("%s: cvcgl_ocean_fft (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
