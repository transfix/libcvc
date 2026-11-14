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
//   1. The same scenario on both at 256 x 256: drops beside a straight no-flux wall and a round
//      island (the symmetric step's ghosts, the island's with fractional weights), a beach band,
//      the sponge, both dampings, a whole-field source half way, steps of 0.2 s (two substeps of
//      0.1 s), then a frame dt that jumps between 0.03 and 0.17 s (steps of no substep and of
//      two, a drop before one of none): both windows must take the same substeps and carry the
//      same remainder. The GPU works in float and the CPU in double, so the two differ by
//      round-off: the RMS difference over the RMS height must stay under 1e-3, for h and phi,
//      and round the island.
//   2. The wall on the GPU: the CPU test's ring off a half-plane wall (ocean_ewave_test,
//      ARingReflectsOffAStraightWall), under no-flux walls (the even image, a crest returns a
//      crest) and zero walls (O8a: the odd image, inverted), each against the CPU.
//   3. Stable: broadband noise in a closed disc with an island, no damping, beach or sponge, for
//      10000 steps of a dt that changes every step (uniform in [0.6, 1.6] of the stable limit
//      with the substep at the limit, and a frame dt in [1/60, 1/30] s at the default substep):
//      the water's energy (the CPU's energy() of the GPU's state) never rises above its start.
//   4. The cost: the passes a step runs (without walls, and with them at the default substep and
//      at 1/30 s), and the GPU time per step at
//      256^2 (a timer query where the context has one, and the wall time with the queue
//      drained), beside OceanFFT's one cascade for scale.
// It SKIPs (rc 0) when the context has no float render targets, as cvcgl_ocean_fft does. All
// checks report explicitly and the program returns non-zero on any failure (assert() is a no-op in
// Release).

#include "EWave.h"
#include "OceanFFT.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
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

// The passes a step of `sub` substeps should run, worked out here: 4 log2 n + 2 without ghosts;
// with them 4 log2 n + 3 per substep (twice with damping_k2) plus the pass home, which a step of
// no substeps needs only when a source pass has moved the state.
int expectedPasses(const cvc::ocean::ewave_params &p, bool ghosts, int sub, bool home = true) {
  const int l = int(std::lround(std::log2(double(p.n))));
  if (!ghosts)
    return 4 * l + 2;
  if (sub == 0)
    return home ? 0 : 1;
  return sub * (p.damping_k2 > 0.0 ? 2 : 1) * (4 * l + 3) + 1;
}

// A fixed xorshift's next value, uniform in [0, 1).
double uniform(std::uint64_t &x) {
  x ^= x << 13;
  x ^= x >> 7;
  x ^= x << 17;
  return double(x >> 11) / 9007199254740992.0;
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
  p.beach_rate = 10.0; // walls are no_flux, the default
  p.substep = 0.1;
  cvc::ocean::ewave cpu(p);
  EWave gpu(p);
  if (!gpu.init(ow)) {
    std::printf("SKIP: float render targets unavailable in this context\n");
    return 0;
  }
  // The wall: mask 0 over x >= 0. A round island 2.5 m across at (-10, -8) m, whose ghosts read
  // fractional bilinear weights with wall corners zeroed (the straight wall's are all (1, 0, 0,
  // 0)), so the GPU's corner order, weight layout and reader slots are checked against the CPU's.
  // A beach band across 18 <= y < 26 m on the open side, the mask ramping from 1 down to 0.3 and
  // back.
  std::vector<double> mask(std::size_t(N) * N, 1.0);
  for (int j = 0; j < N; ++j)
    for (int i = 0; i < N; ++i) {
      const double x = cpu.x_of(i), y = cpu.y_of(j);
      double m = 1.0;
      if (x >= 0.0 || (x + 10.0) * (x + 10.0) + (y + 8.0) * (y + 8.0) < 1.25 * 1.25)
        m = 0.0;
      else if (y >= 18.0 && y < 26.0)
        m = 1.0 - 0.7 * (1.0 - std::fabs(y - 22.0) / 4.0);
      mask[std::size_t(j) * N + std::size_t(i)] = m;
    }
  cpu.set_mask(mask);
  gpu.setMask(mask);

  // A drop 4 m from the wall and one 1.5 m from the island's shore; the GPU has them in h at
  // once, as the CPU does.
  cpu.add_height(-4.0, 0.0, 0.1, 1.0);
  cpu.add_height(-10.0, -5.25, 0.08, 0.75);
  const long long beforeDrop = gpu.passesRun();
  gpu.addDrop(-4.0, 0.0, 0.1, 1.0);
  const long long dropPasses = gpu.passesRun() - beforeDrop;
  gpu.addDrop(-10.0, -5.25, 0.08, 0.75);
  std::size_t fractional = 0, zeroedCorner = 0, tapered = 0;
  for (const cvc::ocean::ewave_ghost &g : cpu.ghosts()) {
    int inside = 0, zero = 0;
    for (double w : g.w) {
      inside += w > 0.0 && w < 1.0;
      zero += w == 0.0;
    }
    fractional += inside > 0;
    zeroedCorner += inside > 0 && zero > 0;
    tapered += (g.w[0] + g.w[1]) + (g.w[2] + g.w[3]) < 1.0 - 1e-12;
  }
  std::printf("  (no-flux walls: %zu ghost cells; %zu with fractional weights, %zu of them with a "
              "wall corner zeroed; %zu tapered)\n",
              cpu.ghosts().size(), fractional, zeroedCorner, tapered);
  check(!cpu.ghosts().empty() && gpu.ghosts() == cpu.ghosts().size(),
        "the GPU window has the CPU's ghost cells");
  check(fractional > 0 && zeroedCorner > 0,
        "the island's ghosts have fractional weights, some with a wall corner zeroed");
  check(dropPasses == 1, "addDrop renders one pass (the walls hold no state to refill)");
  {
    const std::vector<float> z = gpu.readbackState();
    check(z.size() == std::size_t(N) * N * 2, "readbackState: n x n (h, phi) pairs");
    if (z.size() != std::size_t(N) * N * 2)
      return 1;
    const double r = relRms(z, cpu.height(), 0);
    std::printf("  (the drops as added: GPU vs CPU %.2e of their RMS)\n", r);
    check(r < 1e-6, "addDrop puts the same bumps into h as add_height, at once");
  }

  vtkTextureObject *home = nullptr;
  double worstH = 0.0, worstPhi = 0.0, worstIsland = 0.0;
  bool passesOk = true, pendingOk = true;
  int zeroSteps = 0;
  for (int s = 1; s <= 48; ++s) {
    // Steps of 0.2 s (two substeps of 0.1 s), then a frame dt that jumps between 0.03 and 0.17 s:
    // a step of no substeps (0.03 s carried), then one of two.
    const double dt = s <= 24 ? 0.2 : (s % 2 == 1 ? 0.03 : 0.17);
    bool sourced = false;
    if (s == 20) {
      // A whole-field source (C2's shape), half on the beach band.
      const std::vector<double> dh = gaussian(cpu, -12.0, 20.0, -0.05, 0.75);
      cpu.add_heights(dh);
      gpu.addHeights(dh);
      sourced = true;
    }
    if (s == 31) {
      // A drop before a step of no substeps: the state must still come home.
      cpu.add_height(-6.0, 10.0, 0.05, 0.75);
      gpu.addDrop(-6.0, 10.0, 0.05, 0.75);
      sourced = true;
    }
    const int sub = gpu.substeps(dt), cpuSub = cpu.substeps(dt);
    const int planned = gpu.passesPerStep(dt);
    const long long before = gpu.passesRun();
    cpu.step(dt);
    gpu.step(dt);
    const long long measured = gpu.passesRun() - before;
    zeroSteps += sub == 0;
    if (s == 1 || s == 26 || s == 31)
      std::printf("  (a step of %.2f s at %d^2 with walls and damping_k2: %d substeps, %lld "
                  "passes counted as rendered; passesPerStep() %d)\n",
                  dt, N, sub, measured, planned);
    passesOk = passesOk && measured == expectedPasses(p, true, sub, !sourced) &&
               planned == expectedPasses(p, true, sub, !sourced) && sub == cpuSub;
    pendingOk = pendingOk && gpu.pending() == cpu.pending();
    if (s == 1)
      home = gpu.heightTexture();
    if (s % 6 == 0) {
      const std::vector<float> z = gpu.readbackState();
      const double rh = relRms(z, cpu.height(), 0), rp = relRms(z, cpu.phi(), 1);
      worstH = std::max(worstH, rh);
      worstPhi = std::max(worstPhi, rp);
      // And within 3 m of the island's centre, where the fractional weights act.
      double d2 = 0.0, s2 = 0.0;
      for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
          const double x = cpu.x_of(i) + 10.0, y = cpu.y_of(j) + 8.0;
          if (x * x + y * y >= 9.0)
            continue;
          const std::size_t c = std::size_t(j) * N + std::size_t(i);
          const double d = double(z[c * 2]) - cpu.height()[c];
          d2 += d * d;
          s2 += cpu.height()[c] * cpu.height()[c];
        }
      const double ri = std::sqrt(d2 / s2);
      worstIsland = std::max(worstIsland, ri);
      std::printf("  step %2d (t = %.1f s): RMS(GPU - CPU) / RMS(CPU): h %.2e, phi %.2e; h near "
                  "the island %.2e\n",
                  s, cpu.time(), rh, rp, ri);
    }
  }
  check(passesOk, "each step of s substeps renders s (4 log2 n + 3) passes, twice with damping_k2, "
                  "plus one (141 at dt 0.2 s: two substeps of 0.1 s; a step of no substeps 0, or "
                  "1 after a source), and takes the CPU's substeps");
  check(zeroSteps > 0 && pendingOk,
        "a jittered dt carries the same remainder on the GPU as on the CPU, steps of no "
        "substeps included");
  check(worstIsland < 1e-3, "GPU and CPU heights agree round the island (fractional ghost "
                            "weights): RMS difference under 1e-3 of the RMS there");
  check(gpu.heightTexture() == home, "after a step the state is always in the same texture");
  check(std::fabs(gpu.time() - cpu.time()) < 1e-12, "the GPU window keeps the same time");
  check(worstH < 1e-3, "GPU and CPU heights agree: RMS difference under 1e-3 of the RMS height");
  check(worstPhi < 1e-3, "GPU and CPU potentials agree: RMS difference under 1e-3 of the RMS");
  {
    const std::vector<float> z = gpu.readbackState();
    double onWalls = 0.0;
    for (std::size_t c = 0; c < mask.size(); ++c)
      if (mask[c] == 0.0)
        onWalls = std::max(onWalls,
                           std::max(double(std::fabs(z[c * 2])), double(std::fabs(z[c * 2 + 1]))));
    check(onWalls == 0.0, "the walls hold h = phi = 0 on the GPU after a step");
  }

  // ── 2. The ring reflects off the wall on the GPU ───────────────────────────
  // The CPU test's scenario: no damping, sponge 16 cells, a Gaussian (sigma 0.5 m) 6 m from a
  // half-plane wall, here stepped at 0.25 s (two substeps) to t = 13 s, under both kinds of wall.
  // A no-flux wall returns the even image of the free field about the shore, half a cell before
  // the first wall cell (cell i mirrors onto 255 - i): a crest comes back a crest. A zero wall
  // returns the odd image (best fitted about x = dx / 2, 257 - i): a crest comes back a trough.
  for (const cvc::ocean::ewave_walls kind :
       {cvc::ocean::ewave_walls::zero, cvc::ocean::ewave_walls::no_flux}) {
    const bool noFlux = kind == cvc::ocean::ewave_walls::no_flux;
    cvc::ocean::ewave_params q;
    q.n = N;
    q.dx = 0.25;
    q.origin_x = q.origin_y = -32.0;
    q.sponge_cells = 16.0;
    q.walls = kind;
    q.substep = 0.125; // two substeps a step
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
    const std::vector<double> drop = gaussian(ref, -6.0, 0.0, 0.1, 0.5);
    const std::vector<double> zero(std::size_t(N) * N, 0.0);
    wall.setState(drop, zero);
    free_.setState(drop, zero);
    ref.set_state(drop, zero);
    double onWall = 0.0;
    long long passes = 0;
    for (int s = 1; s <= 52; ++s) {
      const long long before = wall.passesRun();
      wall.step(0.25);
      passes = wall.passesRun() - before;
      free_.step(0.25);
      ref.step(0.25);
      const std::vector<float> h = wall.readbackHeight();
      for (std::size_t c = 0; c < h.size(); ++c)
        if (m[c] == 0.0)
          onWall = std::max(onWall, double(std::fabs(h[c])));
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
    std::printf("  wall (%s, %zu ghosts, %lld passes a step): on the wall at most %.1e m; x < 0 "
                "against the %s image %.2f %% RMS (the reflection is %.0f %% of that field's "
                "RMS); the reflection against the mirrored wave: slope %+.3f; against the CPU "
                "wall %.2e\n",
                noFlux ? "no_flux" : "zero", ref.ghosts().size(), passes, onWall,
                noFlux ? "even" : "odd", 100.0 * rel, 100.0 * share, sign, vsCpu);
    check(passes == expectedPasses(q, noFlux, 2),
          noFlux ? "a no-flux wall step at 0.25 s renders 2 x 35 + 1 = 71 passes"
                 : "a zero wall step renders 34 passes (the exact step)");
    check(onWall == 0.0, "nothing on the wall on the GPU");
    if (noFlux) {
      check(share > 0.5 && rel < 0.03,
            "no-flux: the field before the wall is the even image on the GPU, to 3 %");
      check(sign > 0.95 && sign < 1.05, "no-flux: a crest reflects as a crest on the GPU");
      check(vsCpu < 1e-3, "no-flux: the GPU wall run matches the CPU's to 1e-3 RMS");
    } else {
      check(share > 0.5 && rel < 0.25,
            "zero: the field before the wall is the odd image on the GPU, to 25 %");
      check(sign < -0.5, "zero: a crest reflects as a trough on the GPU");
      check(vsCpu < 1e-3, "zero: the GPU wall run matches the CPU's to 1e-3 RMS");
    }
  }

  // ── 3. Stable on the GPU: broadband noise in a closed basin ─────────────────
  // A disc of water 12 m across with an island, on a 64^2 window of 0.25 m cells, no damping,
  // beach or sponge, from uniform noise in h (a fixed xorshift): the water's energy, the CPU's
  // energy() of the GPU's state, every 500 steps. With a dt that changes every step: uniform in
  // [0.6, 1.6] of the stable limit with the substep at the limit (0 to 2 substeps a step), and a
  // frame dt uniform in [1/60, 1/30] s at the default substep (1/60 s).
  const double omMax = std::sqrt(9.80665 * std::sqrt(2.0) * 3.14159265358979 / 0.25);
  const double limit = (1.8 / omMax) * (1.0 - 1e-9);
  struct jitter {
    double substep, lo, hi;
  };
  for (const jitter &jt :
       {jitter{limit, 0.6 * limit, 1.6 * limit}, jitter{1.0 / 60.0, 1.0 / 60.0, 1.0 / 30.0}}) {
    cvc::ocean::ewave_params b;
    b.n = 64;
    b.dx = 0.25;
    b.sponge_cells = 0.0;
    b.substep = jt.substep;
    cvc::ocean::ewave probe(b);
    EWave w(b);
    if (!w.init(ow)) {
      check(false, "the basin window initialises");
    } else {
      std::vector<double> m(64 * 64, 1.0);
      for (int j = 0; j < 64; ++j)
        for (int i = 0; i < 64; ++i) {
          const double r2 = (i - 31.5) * (i - 31.5) + (j - 31.5) * (j - 31.5);
          if (r2 > 24.0 * 24.0 || (i - 31.5) * (i - 31.5) + (j - 20.0) * (j - 20.0) < 9.0)
            m[std::size_t(j) * 64 + std::size_t(i)] = 0.0;
        }
      w.setMask(m);
      probe.set_mask(m);
      std::uint64_t x = 0x9e3779b97f4a7c15ull;
      std::vector<double> h0(m.size(), 0.0), zero(m.size(), 0.0);
      for (std::size_t c = 0; c < m.size(); ++c) {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        if (m[c] > 0.0)
          h0[c] = 0.01 * (2.0 * double(x >> 11) / 9007199254740992.0 - 1.0);
      }
      {
        w.setState(h0, zero);
        probe.set_state(h0, zero);
        const double e0 = probe.energy();
        double hi = 1.0, lo = 1.0, last = 1.0;
        long long subs = 0;
        int most = 0, least = 1 << 30;
        std::uint64_t r = 0x2545f4914f6cdd1dull;
        const int steps = 10000;
        for (int s = 1; s <= steps; ++s) {
          const double dt = jt.lo + (jt.hi - jt.lo) * uniform(r);
          const int sub = w.substeps(dt);
          subs += sub;
          most = std::max(most, sub);
          least = std::min(least, sub);
          w.step(dt);
          if (s % 500 != 0)
            continue;
          const std::vector<float> z = w.readbackState();
          std::vector<double> hh(m.size()), pp(m.size());
          for (std::size_t c = 0; c < m.size(); ++c) {
            hh[c] = z[c * 2];
            pp[c] = z[c * 2 + 1];
          }
          probe.set_state(hh, pp);
          last = probe.energy() / e0;
          if (!(last <= 1e30))
            last = 1e30;
          hi = std::max(hi, last);
          lo = std::min(lo, last);
        }
        std::printf("  stable on the GPU, disc with an island, %d steps of dt uniform in "
                    "[%.4f, %.4f] s, substep %.4f s (omega_max dt_s %.2f): %d to %d substeps a "
                    "step, %lld in all; the energy in %.4f..%.4f of its start, ends %.4f\n",
                    steps, jt.lo, jt.hi, cvc::ocean::ewave_substep(b),
                    omMax * cvc::ocean::ewave_substep(b), least, most, subs, lo, hi, last);
        check(hi < 1.0 + 1e-4, "with a dt that changes every step, the GPU's energy never rises "
                               "above its start (float round-off)");
      }
    }
  }

  // ── 4. The cost of a step ───────────────────────────────────────────────────
  {
    cvc::ocean::ewave_params c; // the C3 window: 256^2 at 0.5 m
    cvc::ocean::ewave_params c30 = c;
    c30.substep = 1.0 / 30.0; // for a 30 Hz loop: one substep a step
    EWave w(c), walled(c), walled30(c30);
    OceanFFT fft(N);
    if (!w.init(ow) || !walled.init(ow) || !walled30.init(ow) || !fft.init(ow)) {
      check(false, "the timing windows initialise");
    } else {
      // The walled window: a wavy coast over the right half, as ocean_ewave_test's StepCost.
      std::vector<double> m(std::size_t(N) * N, 1.0);
      for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i)
          if (i >= 128 + int(12.0 * std::sin(j * 0.05)))
            m[std::size_t(j) * N + std::size_t(i)] = 0.0;
      walled.setMask(m);
      walled30.setMask(m);
      w.addDrop(64.0, 64.0, 0.1, 1.0);
      walled.addDrop(40.0, 64.0, 0.1, 1.0);
      walled30.addDrop(40.0, 64.0, 0.1, 1.0);
      for (int s = 0; s < 10; ++s) { // warm the programs and the driver
        w.step(1.0 / 30.0);
        walled.step(1.0 / 30.0);
        walled30.step(1.0 / 30.0);
      }
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
      double ewGpu = 0.0, wallGpu = 0.0, wall30Gpu = 0.0, fftGpu = 0.0;
      const double ewWall = timeIt([&](int) { w.step(1.0 / 30.0); }, ewGpu);
      const double wallWall = timeIt([&](int) { walled.step(1.0 / 30.0); }, wallGpu);
      const double wall30Wall = timeIt([&](int) { walled30.step(1.0 / 30.0); }, wall30Gpu);
      const double fftWall = timeIt([&](int s) { fft.step(s / 30.0); }, fftGpu);
      std::printf("  cost at %d^2, dt 1/30 s: eWave step %.3f ms GPU (timer query), %.3f ms wall "
                  "(%d passes); with a no-flux coast (%zu ghosts), the default substep 1/60 s: "
                  "%.3f ms GPU, %.3f ms wall (%d passes); substep 1/30 s: %.3f ms GPU, %.3f ms "
                  "wall (%d passes); one OceanFFT cascade %.3f ms GPU, %.3f ms wall (%d steps "
                  "each)\n",
                  N, ewGpu, ewWall, w.passesPerStep(1.0 / 30.0), walled.ghosts(), wallGpu, wallWall,
                  walled.passesPerStep(1.0 / 30.0), wall30Gpu, wall30Wall,
                  walled30.passesPerStep(1.0 / 30.0), fftGpu, fftWall, K);
      const std::vector<float> h = walled.readbackHeight();
      bool finite = !h.empty();
      for (float v : h)
        finite = finite && std::isfinite(v);
      check(finite, "after the timed steps the walled field is finite");
      check(w.passesPerStep(1.0 / 30.0) == 4 * 8 + 2,
            "without walls a step is 34 passes (the exact step)");
      check(walled.substeps(1.0 / 30.0) == 2 &&
                walled.passesPerStep(1.0 / 30.0) == 2 * (4 * 8 + 3) + 1,
            "with no-flux walls and the default substep (1/60 s) a 1/30 s step is two substeps: "
            "2 x 35 passes and the pass home, 71");
      check(walled30.substeps(1.0 / 30.0) == 1 &&
                walled30.passesPerStep(1.0 / 30.0) == 4 * 8 + 3 + 1,
            "with a substep of 1/30 s a 1/30 s step is one substep: 35 passes and the pass home, "
            "36");
    }
  }

  std::printf("%s: cvcgl_ewave (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
