/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ewave — the GPU eWave window (EWave, docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md C1, C3,
// O8b, O8c) against the CPU one it must agree with (cvc::ocean::ewave, O8a, O8c).
//
//   1. The same scenario on both at 256 x 256: a drop beside a straight no-flux wall (its ghost
//      cells), a beach band, the sponge, both dampings, a whole-field source half way, and a
//      change of dt. The GPU works in float and the CPU in double, so the two differ by round-off
//      through two transforms a step: the RMS difference over the RMS height must stay under
//      1e-3, for h and for phi.
//   2. The wall on the GPU: the CPU test's ring off a half-plane wall (ocean_ewave_test,
//      ARingReflectsOffAStraightWall), under no-flux walls (O8c: the even image, a crest
//      returns a crest, 35 passes a step) and zero walls (O8a: the odd image, inverted, 34),
//      each against the CPU.
//   3. The cost: the passes a step runs, and the GPU time per step at 256^2 (a timer query where
//      the context has one, and the wall time with the queue drained), beside OceanFFT's one
//      cascade for scale.
// It SKIPs (rc 0) when the context has no float render targets, as cvcgl_ocean_fft does. All
// checks report explicitly and the program returns non-zero on any failure (assert() is a no-op in
// Release).

#include "EWave.h"
#include "OceanFFT.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/ocean/ewave.h>
#include <cvc/ocean/h0.h>
#include <string>
#include <vector>
#include <vtkOpenGLRenderTimer.h>
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

constexpr int N = 256;

// A Gaussian hump of height a and width sigma at (x0, y0), as a whole field (the CPU test's).
std::vector<double> gaussian(const cvc::ocean::ewave &w, double x0, double y0, double a,
                             double sigma) {
  const int n = w.size();
  std::vector<double> h(std::size_t(n) * std::size_t(n));
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double dx = w.x_of(i) - x0, dy = w.y_of(j) - y0;
      h[std::size_t(j) * std::size_t(n) + std::size_t(i)] =
          a * std::exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
    }
  return h;
}

// RMS of the GPU's field minus the CPU's, over the CPU's RMS. `comp` picks h (0) or phi (1) from
// the GPU's (h, phi) pairs.
double relRms(const std::vector<float> &gpu, const std::vector<double> &cpu, int comp) {
  double d2 = 0.0, s2 = 0.0;
  for (std::size_t c = 0; c < cpu.size(); ++c) {
    const double d = double(gpu[c * 2 + comp]) - cpu[c];
    d2 += d * d;
    s2 += cpu[c] * cpu[c];
  }
  return s2 > 0.0 ? std::sqrt(d2 / s2) : std::sqrt(d2);
}

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "ewavetest");
  // A window smaller than the field on purpose: EWave must not be clipped by the renderer's
  // scissor box (its passes disable the scissor test).
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.render(); // initialise the GL context before allocating float RTs
  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  if (!ow) {
    std::printf("FAIL: render window is not a vtkOpenGLRenderWindow (no OpenGL2 backend)\n");
    return 1;
  }

  // ── 1. The same scenario on the GPU and the CPU ────────────────────────────
  cvc::ocean::ewave_params p;
  p.n = N;
  p.dx = 0.25;
  p.origin_x = p.origin_y = -32.0;
  p.depth = 1000.0;
  p.damping_k2 = 1e-3;
  p.damping = 0.02;
  p.sponge_cells = 16.0;
  p.sponge_rate = 2.0;
  p.beach_rate = 10.0;
  cvc::ocean::ewave cpu(p);
  EWave gpu(p);
  if (!gpu.init(ow)) {
    std::printf("SKIP: float render targets unavailable in this context\n");
    return 0;
  }
  // The wall: mask 0 over x >= 0. A beach band across 18 <= y < 26 m on the open side, the mask
  // ramping from 1 down to 0.3 and back.
  std::vector<double> mask(std::size_t(N) * N, 1.0);
  for (int j = 0; j < N; ++j)
    for (int i = 0; i < N; ++i) {
      const double x = cpu.x_of(i), y = cpu.y_of(j);
      double m = 1.0;
      if (x >= 0.0)
        m = 0.0;
      else if (y >= 18.0 && y < 26.0)
        m = 1.0 - 0.7 * (1.0 - std::fabs(y - 22.0) / 4.0);
      mask[std::size_t(j) * N + std::size_t(i)] = m;
    }
  cpu.set_mask(mask);
  gpu.setMask(mask);

  // A drop 4 m from the wall; the GPU has it in h at once, as the CPU does.
  cpu.add_height(-4.0, 0.0, 0.1, 1.0);
  const long long beforeDrop = gpu.passesRun();
  gpu.addDrop(-4.0, 0.0, 0.1, 1.0);
  // The wall is no-flux (the default): the CPU's ghost cells, and the GPU's ghost pass.
  std::vector<char> ghost(mask.size(), 0);
  for (const cvc::ocean::ewave_ghost &g : cpu.ghosts())
    ghost[g.cell] = 1;
  std::printf("  (no-flux wall: %zu ghost cells, a band of %d)\n", cpu.ghosts().size(),
              p.ghost_cells);
  check(!cpu.ghosts().empty(), "the no-flux wall has ghost cells");
  check(gpu.passesRun() - beforeDrop == 2,
        "addDrop renders two passes with ghosts: the drop and the ghost refill");
  {
    const std::vector<float> z = gpu.readbackState();
    check(z.size() == std::size_t(N) * N * 2, "readbackState: n x n (h, phi) pairs");
    if (z.size() != std::size_t(N) * N * 2)
      return 1;
    const double r = relRms(z, cpu.height(), 0);
    std::printf("  (the drop as added: GPU vs CPU %.2e of its RMS)\n", r);
    check(r < 1e-6, "addDrop puts the same bump into h as add_height, at once");
  }

  vtkTextureObject *home = nullptr;
  double worstH = 0.0, worstPhi = 0.0;
  for (int s = 1; s <= 48; ++s) {
    const double dt = s <= 24 ? 0.2 : 0.1;
    if (s == 20) {
      // A whole-field source (C2's shape), half on the beach band.
      const std::vector<double> dh = gaussian(cpu, -12.0, 20.0, -0.05, 0.75);
      cpu.add_heights(dh);
      gpu.addHeights(dh);
    }
    const long long before = gpu.passesRun();
    cpu.step(dt);
    gpu.step(dt);
    if (s == 1) {
      // passesRun() counts the passes as they are rendered (the butterflies by OceanButterfly's
      // own counter), so this measures step() against 4 log2 n + 2 (+ 1) worked out here.
      const long long measured = gpu.passesRun() - before;
      std::printf("  (passes per step at %d^2 with walls, counted as rendered: %lld; "
                  "passesPerStep() %d)\n",
                  N, measured, gpu.passesPerStep());
      check(measured == 4 * 8 + 3 && gpu.passesPerStep() == 4 * 8 + 3,
            "one step at 256^2 with no-flux walls renders 35 passes: 2 x 16 butterflies, the "
            "update, real space and the ghosts");
      home = gpu.heightTexture();
    }
    if (s % 6 == 0) {
      const std::vector<float> z = gpu.readbackState();
      const double rh = relRms(z, cpu.height(), 0), rp = relRms(z, cpu.phi(), 1);
      worstH = std::max(worstH, rh);
      worstPhi = std::max(worstPhi, rp);
      std::printf("  step %2d (t = %.1f s): RMS(GPU - CPU) / RMS(CPU): h %.2e, phi %.2e\n", s,
                  cpu.time(), rh, rp);
    }
  }
  check(gpu.heightTexture() == home, "after a step the state is always in the same texture");
  check(std::fabs(gpu.time() - cpu.time()) < 1e-12, "the GPU window keeps the same time");
  check(worstH < 1e-3, "GPU and CPU heights agree: RMS difference under 1e-3 of the RMS height");
  check(worstPhi < 1e-3, "GPU and CPU potentials agree: RMS difference under 1e-3 of the RMS");
  {
    const std::vector<float> h = gpu.readbackHeight();
    double beyond = 0.0;
    for (int j = 0; j < N; ++j)
      for (int i = 0; i < N; ++i)
        if (cpu.x_of(i) >= 0.0 && !ghost[std::size_t(j) * N + std::size_t(i)])
          beyond = std::max(beyond, double(std::fabs(h[std::size_t(j) * N + std::size_t(i)])));
    check(beyond == 0.0, "nothing on the wall beyond the ghosts on the GPU (zeroed every step)");
  }

  // ── 2. The ring reflects off the wall on the GPU ───────────────────────────
  // The CPU test's scenario: no damping, sponge 16 cells, a Gaussian (sigma 0.5 m) 6 m from a
  // half-plane wall, here stepped at 0.25 s (exact for any dt) to t = 13 s, under both kinds of
  // wall. A no-flux wall returns the even image of the free field about the shore, half a cell
  // before the first wall cell (cell i mirrors onto 255 - i): a crest comes back a crest. A zero
  // wall returns the odd image (best fitted about x = dx / 2, 257 - i): a crest comes back a
  // trough.
  for (const cvc::ocean::ewave_walls kind :
       {cvc::ocean::ewave_walls::zero, cvc::ocean::ewave_walls::no_flux}) {
    const bool noFlux = kind == cvc::ocean::ewave_walls::no_flux;
    cvc::ocean::ewave_params q;
    q.n = N;
    q.dx = 0.25;
    q.origin_x = q.origin_y = -32.0;
    q.sponge_cells = 16.0;
    q.walls = kind;
    cvc::ocean::ewave ref(q);
    EWave wall(q), free_(q);
    if (!wall.init(ow) || !free_.init(ow)) {
      check(false, "a second and third GPU window initialise");
      continue;
    }
    std::vector<double> m(std::size_t(N) * N, 1.0);
    for (int j = 0; j < N; ++j)
      for (int i = 0; i < N; ++i)
        if (ref.x_of(i) >= 0.0)
          m[std::size_t(j) * N + std::size_t(i)] = 0.0;
    wall.setMask(m);
    ref.set_mask(m);
    std::vector<char> isGhost(m.size(), 0);
    for (const cvc::ocean::ewave_ghost &g : ref.ghosts())
      isGhost[g.cell] = 1;
    const std::vector<double> drop = gaussian(ref, -6.0, 0.0, 0.1, 0.5);
    const std::vector<double> zero(std::size_t(N) * N, 0.0);
    wall.setState(drop, zero);
    free_.setState(drop, zero);
    ref.set_state(drop, zero);
    double beyond = 0.0;
    long long passes = 0;
    for (int s = 1; s <= 52; ++s) {
      const long long before = wall.passesRun();
      wall.step(0.25);
      passes = wall.passesRun() - before;
      free_.step(0.25);
      ref.step(0.25);
      const std::vector<float> h = wall.readbackHeight();
      for (std::size_t c = 0; c < h.size(); ++c)
        if (m[c] == 0.0 && !isGhost[c])
          beyond = std::max(beyond, double(std::fabs(h[c])));
    }
    // At t = 13 s the reflected ring is 7 m back from the wall. Compare x < 0 with the image,
    // the reflection's part for scale, and the reflection (wall - free) against the mirrored
    // free field for its sign.
    const std::vector<float> hw = wall.readbackHeight(), hf = free_.readbackHeight();
    double err = 0.0, norm = 0.0, refl = 0.0, rm = 0.0, mm = 0.0, d2 = 0.0, s2 = 0.0;
    for (int j = 16; j < 240; ++j)
      for (int i = 16; i < 128; ++i) {
        const std::size_t c = std::size_t(j) * N + std::size_t(i),
                          mi = std::size_t(j) * N + std::size_t(noFlux ? 255 - i : 257 - i),
                          me = std::size_t(j) * N + std::size_t(255 - i);
        const double img = noFlux ? double(hf[c]) + double(hf[mi]) : double(hf[c]) - double(hf[mi]);
        err += (hw[c] - img) * (hw[c] - img);
        norm += img * img;
        refl += double(hf[mi]) * hf[mi];
        rm += (double(hw[c]) - hf[c]) * hf[me];
        mm += double(hf[me]) * hf[me];
      }
    for (std::size_t c = 0; c < hw.size(); ++c) {
      d2 += (hw[c] - ref.height()[c]) * (hw[c] - ref.height()[c]);
      s2 += ref.height()[c] * ref.height()[c];
    }
    const double rel = std::sqrt(err / norm), share = std::sqrt(refl / norm), sign = rm / mm,
                 vsCpu = std::sqrt(d2 / s2);
    std::printf("  wall (%s, %zu ghosts, %lld passes a step): beyond the ghosts at most %.1e m; "
                "x < 0 against the %s image %.1f %% RMS (the reflection is %.0f %% of that "
                "field's RMS); the reflection against the mirrored wave: slope %+.3f; against "
                "the CPU wall %.2e\n",
                noFlux ? "no_flux" : "zero", ref.ghosts().size(), passes, beyond,
                noFlux ? "even" : "odd", 100.0 * rel, 100.0 * share, sign, vsCpu);
    if (noFlux) {
      check(passes == 4 * 8 + 3, "a no-flux wall step renders 35 passes");
      check(beyond == 0.0, "no-flux: nothing on the wall beyond the ghosts on the GPU");
      check(share > 0.5 && rel < 0.08,
            "no-flux: the field before the wall is the even image on the GPU, to 8 %");
      check(sign > 0.9 && sign < 1.1, "no-flux: a crest reflects as a crest on the GPU");
      check(vsCpu < 1e-3, "no-flux: the GPU wall run matches the CPU's to 1e-3 RMS");
    } else {
      check(passes == 4 * 8 + 2, "a zero wall step renders 34 passes (no ghosts)");
      check(beyond == 0.0, "zero: the ring does not cross the wall on the GPU");
      check(share > 0.5 && rel < 0.25,
            "zero: the field before the wall is the odd image on the GPU, to 25 %");
      check(sign < -0.5, "zero: a crest reflects as a trough on the GPU");
      check(vsCpu < 1e-3, "zero: the GPU wall run matches the CPU's to 1e-3 RMS");
    }
  }

  // ── 3. The cost of a step ───────────────────────────────────────────────────
  {
    cvc::ocean::ewave_params c; // the C3 window: 256^2 at 0.5 m
    EWave w(c);
    OceanFFT fft(N);
    if (!w.init(ow) || !fft.init(ow)) {
      check(false, "the timing windows initialise");
    } else {
      w.addDrop(64.0, 64.0, 0.1, 1.0);
      for (int s = 0; s < 10; ++s) // warm the programs and the driver
        w.step(1.0 / 30.0);
      for (int s = 0; s < 10; ++s)
        fft.step(s / 30.0);
      ow->WaitForCompletion();
      // Which GPU the context is on (a laptop may have two).
      const std::string caps = ow->ReportCapabilities();
      const std::size_t at = caps.find("OpenGL renderer string:");
      if (at != std::string::npos)
        std::printf("  (%s)\n", caps.substr(at, caps.find('\n', at) - at).c_str());
      const int K = 200;
      auto timeIt = [&](auto &&body, double &gpuMs) {
        gpuMs = -1.0;
        vtkOpenGLRenderTimer timer;
        const bool query = vtkOpenGLRenderTimer::IsSupported();
        const auto t0 = std::chrono::steady_clock::now();
        if (query)
          timer.Start();
        for (int s = 0; s < K; ++s)
          body(s);
        if (query)
          timer.Stop();
        ow->WaitForCompletion();
        const auto t1 = std::chrono::steady_clock::now();
        if (query) {
          for (int spin = 0; spin < 1000 && !timer.Ready(); ++spin)
            ow->WaitForCompletion();
          if (timer.Ready())
            gpuMs = timer.GetElapsedMilliseconds() / K;
        }
        return std::chrono::duration<double, std::milli>(t1 - t0).count() / K;
      };
      double ewGpu = 0.0, fftGpu = 0.0;
      const double ewWall = timeIt([&](int) { w.step(1.0 / 30.0); }, ewGpu);
      const double fftWall = timeIt([&](int s) { fft.step(s / 30.0); }, fftGpu);
      std::printf("  cost at %d^2: eWave step %.3f ms GPU (timer query), %.3f ms wall; one "
                  "OceanFFT cascade %.3f ms GPU, %.3f ms wall (%d steps each)\n",
                  N, ewGpu, ewWall, fftGpu, fftWall, K);
      const std::vector<float> h = w.readbackHeight();
      bool finite = !h.empty();
      for (float v : h)
        finite = finite && std::isfinite(v);
      check(finite, "after the timed steps the field is finite");
      check(w.passesPerStep() == 4 * 8 + 2, "without walls a step is 34 passes (no ghost pass)");
    }
  }

  std::printf("%s: cvcgl_ewave (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
