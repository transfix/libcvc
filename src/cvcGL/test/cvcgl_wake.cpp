/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_wake — bodies make waves on the GPU (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md C2, C3,
// O9): the shared immersion code (cvc/physics/immersion.h) turned into sources (WakeSource) for
// the GPU's eWave window (EWave), and the window moving by whole cells.
//
//   1. Sources: WakeSource's dh holds the volume, -(V(t) - V(t - dt)) / dx^2, and a body that does
//      not move makes none.
//   2. The shift: a shifted window holds its waves where they were in the world, exactly, with
//      the strip that enters flat, and steps on as an unshifted one does away from its edges.
//   3. The Kelvin wake (Phase C's exit): a box footprint (2 x 1 m), moved at constant speed U
//      across still water, immersed column by column and fed to EWave each step, leaves a wake
//      whose envelope half-angle, read from the readback as ocean_ewave_test reads it on the
//      CPU, is about arcsin(1/3) = 19.47 degrees.
//   4. A rock's shadow: a disc masked to 0 in a train of waves from a wavemaker casts a shadow
//      behind it: the wave height there against the same run without the rock.
// It SKIPs (rc 0) when the context has no float render targets, as cvcgl_ewave does. All checks
// report explicitly and the program returns non-zero on any failure.

#include "EWave.h"
#include "WakeSource.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/ocean/ewave.h>
#include <cvc/physics/fluid_surface.h>
#include <cvc/physics/immersion.h>
#include <string>
#include <vector>
#include <vtkOpenGLRenderWindow.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace phys = cvc::physics;

namespace {

int g_fail = 0;
void check(bool ok, const char *msg) {
  std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", msg);
  if (!ok)
    ++g_fail;
}

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;

double fit_angle(const std::vector<double> &ss, const std::vector<double> &ys) {
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  const double k = double(ss.size());
  for (std::size_t q = 0; q < ss.size(); ++q) {
    sx += ss[q];
    sy += ys[q];
    sxx += ss[q] * ss[q];
    sxy += ss[q] * ys[q];
  }
  return std::atan((k * sxy - sx * sy) / (k * sxx - sx * sx)) * 180.0 / kPi;
}

struct kelvin_reading {
  std::vector<double> fitted;
  double peak = 0.0;
  double seconds = 0.0;
  bool ok = false;
};

// ocean_ewave_test's Kelvin scenario, with a box in place of its Gaussian and the GPU in place of
// the CPU: the footprint ramps in over `ramp` seconds (its draft from 0 to 5 cm, smoothly), travels
// `travel` metres along +x at u, and the rows across the track 16-44 m behind it are read. On a
// row, the envelope's edge at a threshold is the outermost lateral distance where |h| exceeds that
// fraction of the row's maximum; its fitted slope against the distance behind gives the
// half-angle. `peak` is the fitted angle of the row maxima.
kelvin_reading kelvin_wake(vtkOpenGLRenderWindow *ow, double u, const std::vector<double> &fracs) {
  kelvin_reading out;
  const int n = 512;
  cvc::ocean::ewave_params p;
  p.n = n;
  p.dx = 0.5;
  p.origin_x = p.origin_y = -0.25 * n;
  p.sponge_cells = 12.0;
  EWave w(p);
  if (!w.init(ow))
    return out;
  WakeSource src(p);
  const phys::immersion_body box(phys::shape{phys::box{{1.0, 0.5, 0.25}}});
  phys::still_surface still(0.0);
  phys::immersion_water water;
  water.surface = &still;
  const double dt = 0.2, ramp = 8.0, travel = 140.0, x0 = p.origin_x + 10.0;
  const int steps = int(travel / u / dt + 0.5);
  std::vector<double> dh;
  double xs = x0;
  const auto t0 = std::chrono::steady_clock::now();
  for (int s = 0; s <= steps; ++s) {
    const double t = s * dt;
    xs = x0 + u * t;
    const double tr = std::min(t / ramp, 1.0), draft = 0.05 * tr * tr * (3.0 - 2.0 * tr);
    water.time = t;
    src.begin();
    if (draft > 0.0)
      src.add(box, {xs, 0.0, 0.25 - draft}, phys::quat{}, water);
    if (src.finish(dh))
      w.addHeights(dh);
    if (s < steps)
      w.step(dt);
  }
  const std::vector<float> h = w.readbackHeight();
  out.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (h.size() != std::size_t(n) * n)
    return out;
  std::vector<double> ss, peak;
  std::vector<std::vector<double>> edge(fracs.size());
  for (double s = 16.0; s <= 44.0; s += 2.0) {
    const int i = int((xs - s - p.origin_x) / p.dx + 0.5);
    double m = 0.0, ym = 0.0;
    for (int j = 16; j < n - 16; ++j) {
      const double v = std::fabs(h[std::size_t(j) * n + std::size_t(i)]);
      if (v > m) {
        m = v;
        ym = std::fabs(p.origin_y + j * p.dx);
      }
    }
    for (std::size_t f = 0; f < fracs.size(); ++f) {
      double y = 0.0;
      for (int j = 16; j < n - 16; ++j)
        if (std::fabs(h[std::size_t(j) * n + std::size_t(i)]) > fracs[f] * m)
          y = std::max(y, std::fabs(p.origin_y + j * p.dx));
      edge[f].push_back(y);
    }
    ss.push_back(xs - (p.origin_x + i * p.dx));
    peak.push_back(ym);
  }
  for (std::size_t f = 0; f < fracs.size(); ++f)
    out.fitted.push_back(fit_angle(ss, edge[f]));
  out.peak = fit_angle(ss, peak);
  out.ok = true;
  return out;
}

double rms(const std::vector<float> &h, int n, const cvc::ocean::ewave_params &p, double x0,
           double x1, double y0, double y1) {
  double s = 0.0;
  int k = 0;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const double x = p.origin_x + i * p.dx, y = p.origin_y + j * p.dx;
      if (x >= x0 && x <= x1 && y >= y0 && y <= y1) {
        const double v = h[std::size_t(j) * n + std::size_t(i)];
        s += v * v;
        ++k;
      }
    }
  return k ? std::sqrt(s / k) : 0.0;
}

} // namespace

int main() {
  cvc::app app;
  SceneGraph sg(app, "waketest");
  SceneRenderer sr(sg, 64, 64, /*offscreen=*/true);
  sr.render(); // initialise the GL context before allocating float RTs
  auto *ow = vtkOpenGLRenderWindow::SafeDownCast(sr.renderWindow());
  if (!ow) {
    std::printf("FAIL: render window is not a vtkOpenGLRenderWindow (no OpenGL2 backend)\n");
    return 1;
  }
  {
    cvc::ocean::ewave_params p;
    p.n = 16;
    p.sponge_cells = 0.0;
    EWave probe(p);
    if (!probe.init(ow)) {
      std::printf("SKIP: float render targets unavailable in this context\n");
      return 0;
    }
  }

  // ── 1. The sources hold the volume ──────────────────────────────────────────
  {
    cvc::ocean::ewave_params p;
    p.n = 64;
    p.dx = 0.5;
    p.origin_x = p.origin_y = -16.0;
    WakeSource src(p);
    const phys::immersion_body crate(phys::shape{phys::box{{0.6, 0.45, 0.4}}});
    phys::still_surface still(0.0);
    phys::immersion_water water;
    water.surface = &still;
    std::vector<double> dh;
    const phys::quat turned{0.9, 0.1, 0.2, std::sqrt(1.0 - 0.81 - 0.01 - 0.04)};
    src.begin();
    src.add(crate, {0.3, -0.2, 0.6}, turned, water); // above the water: nothing
    const bool first = src.finish(dh);
    double worst = 0.0;
    double v0 = src.volume();
    for (double z : {0.3, 0.1, 0.1, -0.2}) {
      src.begin();
      src.add(crate, {0.3, -0.2, z}, turned, water);
      const double v1 = src.volume();
      double sum = 0.0;
      if (src.finish(dh))
        for (double d : dh)
          sum += d;
      else
        sum = 0.0;
      worst = std::max(worst, std::fabs(sum * p.dx * p.dx + (v1 - v0)));
      v0 = v1;
    }
    std::printf("  (sources: the largest miss of sum(dh) dx^2 against -(V(t) - V(t - dt)): %.2e "
                "m^3; the crate's volume under at the end %.4f m^3)\n",
                worst, v0);
    check(!first, "the first step only records zeta");
    check(worst < 1e-12, "a body's source holds the volume it pushes under or lets up");
    src.begin();
    src.add(crate, {0.3, -0.2, -0.2}, turned, water);
    check(!src.finish(dh), "a body that stays still makes no source");
  }

  // ── 2. The window moves by whole cells ─────────────────────────────────────
  {
    cvc::ocean::ewave_params p;
    p.n = 256;
    p.dx = 0.5;
    p.origin_x = p.origin_y = -64.0;
    EWave a(p), b(p);
    if (!a.init(ow) || !b.init(ow)) {
      check(false, "two shift windows initialise");
    } else {
      a.addDrop(3.0, -2.0, 0.1, 1.5);
      b.addDrop(3.0, -2.0, 0.1, 1.5);
      for (int s = 0; s < 10; ++s) {
        a.step(0.1);
        b.step(0.1);
      }
      const int di = 7, dj = -5;
      const long long before = b.passesRun();
      b.shift(di, dj);
      check(b.passesRun() - before == 1, "a shift is one pass");
      check(b.params().origin_x == p.origin_x + di * p.dx &&
                b.params().origin_y == p.origin_y + dj * p.dx,
            "the shift moves the origin by whole cells");
      const std::vector<float> za = a.readbackState(), zb = b.readbackState();
      const int n = p.n;
      bool same = true, flat = true;
      for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
          const int si = i + di, sj = j + dj;
          const std::size_t c = (std::size_t(j) * n + i) * 2;
          if (si >= 0 && si < n && sj >= 0 && sj < n) {
            const std::size_t sc = (std::size_t(sj) * n + si) * 2;
            same = same && zb[c] == za[sc] && zb[c + 1] == za[sc + 1];
          } else {
            flat = flat && zb[c] == 0.0f && zb[c + 1] == 0.0f;
          }
        }
      check(same, "after a shift each cell holds what was at its place in the world, exactly");
      check(flat, "the strip that enters is flat");
      for (int s = 0; s < 20; ++s) {
        a.step(0.1);
        b.step(0.1);
      }
      // Away from the edges they agree, to float round-off (the transforms see the field at other
      // texels) and the sponge, which sits elsewhere in the world for each and reaches the
      // interior a little through the spectral operator.
      const std::vector<float> ha = a.readbackHeight(), hb = b.readbackHeight();
      double d2 = 0.0, s2 = 0.0;
      for (int j = 40; j < n - 40; ++j)
        for (int i = 40; i < n - 40; ++i) {
          const double va = ha[std::size_t(j + dj) * n + (i + di)], vb = hb[std::size_t(j) * n + i];
          d2 += (va - vb) * (va - vb);
          s2 += va * va;
        }
      const double rel = std::sqrt(d2 / s2);
      std::printf("  (after 2 s more, the shifted window against the unshifted one: %.2e of the "
                  "RMS, away from the edges)\n",
                  rel);
      check(rel < 1e-3, "a shifted window steps on as an unshifted one does (to 1e-3)");
    }
  }

  // ── 3. A box hull's Kelvin wake on the GPU ─────────────────────────────────
  {
    const double kelvin = std::asin(1.0 / 3.0) * 180.0 / kPi;
    const std::vector<double> fracs = {0.3, 0.4, 0.5};
    for (double u : {2.5, 3.0}) {
      const kelvin_reading r = kelvin_wake(ow, u, fracs);
      if (!r.ok) {
        check(false, "the Kelvin window initialises and reads back");
        continue;
      }
      double worst = 0.0;
      for (std::size_t f = 0; f < fracs.size(); ++f) {
        std::printf("  U %.1f m/s (Froude number on its 2 m length %.2f), edge at %.0f %% of the "
                    "row maximum: half-angle %.2f deg (Kelvin %.2f)\n",
                    u, u / std::sqrt(kG * 2.0), 100.0 * fracs[f], r.fitted[f], kelvin);
        worst = std::max(worst, std::fabs(r.fitted[f] - kelvin));
      }
      std::printf("  U %.1f m/s: the row maxima at %.2f deg [%.1f s, 512^2, %d steps]\n", u, r.peak,
                  r.seconds, int(140.0 / u / 0.2 + 0.5));
      char msg[160];
      std::snprintf(msg, sizeof msg,
                    "a box moved at %.1f m/s leaves a wake at 19.5 deg (30-50 %% edges within "
                    "2 deg)",
                    u);
      check(worst < 2.0, msg);
    }
  }

  // ── 4. A rock casts a wave shadow ──────────────────────────────────────────
  // A wavemaker 40 m upwave of a rock 8 m across (mask 0 in the disc) sends waves 4 m long; the
  // RMS height behind the rock, 8-20 m downwave within 2 m of the axis, over 20 s once the train
  // has arrived, against the same run without the rock. And to the side, 12-16 m off the axis,
  // where the rock's shadow does not reach.
  {
    cvc::ocean::ewave_params p;
    p.n = 256;
    p.dx = 0.5;
    p.origin_x = p.origin_y = -64.0;
    p.sponge_cells = 16.0;
    const int n = p.n;
    const double lambda = 4.0, k = 2.0 * kPi / lambda, om = std::sqrt(kG * k), R = 4.0, xs = -40.0,
                 amp = 0.05;
    double behind[2] = {0.0, 0.0}, side[2] = {0.0, 0.0};
    bool ran = true;
    for (int rock = 0; rock < 2 && ran; ++rock) {
      EWave w(p);
      if (!w.init(ow)) {
        ran = false;
        break;
      }
      std::vector<double> mask(std::size_t(n) * n, 1.0), line(std::size_t(n) * n, 0.0);
      for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i) {
          const double x = p.origin_x + i * p.dx, y = p.origin_y + j * p.dx;
          if (rock && x * x + y * y <= R * R)
            mask[std::size_t(j) * n + i] = 0.0;
          // The wavemaker: a line across y (tapered off before the sponge), a Gaussian 0.5 m
          // wide in x.
          const double taper =
              std::fabs(y) < 40.0 ? 1.0 : std::max(0.0, 1.0 - (std::fabs(y) - 40.0) / 8.0);
          line[std::size_t(j) * n + i] = taper * std::exp(-(x - xs) * (x - xs) / (2.0 * 0.25));
        }
      w.setMask(mask);
      const double dt = 0.1;
      std::vector<double> dh(line.size());
      double sb = 0.0, sside = 0.0;
      int kb = 0;
      for (int s = 1; s <= 700; ++s) {
        const double t = s * dt;
        // dh = the wavemaker's rate times dt: amp om cos(om t) over the line.
        const double ramp = std::min(1.0, t / 5.0);
        const double a = ramp * amp * om * std::cos(om * t) * dt;
        for (std::size_t c = 0; c < dh.size(); ++c)
          dh[c] = a * line[c];
        w.addHeights(dh);
        w.step(dt);
        if (t >= 50.0) {
          const std::vector<float> h = w.readbackHeight();
          const double b = rms(h, n, p, 8.0, 20.0, -2.0, 2.0),
                       sd = rms(h, n, p, 8.0, 20.0, 12.0, 16.0);
          sb += b * b;
          sside += sd * sd;
          ++kb;
        }
      }
      behind[rock] = std::sqrt(sb / kb);
      side[rock] = std::sqrt(sside / kb);
    }
    if (!ran) {
      check(false, "the shadow windows initialise");
    } else {
      const double shadow = behind[1] / behind[0], aside = side[1] / side[0];
      std::printf("  rock shadow (waves 4 m long, a rock 8 m across): RMS height 8-20 m behind it "
                  "%.4f m with the rock, %.4f m without (%.0f %%); 12-16 m to the side %.0f %%\n",
                  behind[1], behind[0], 100.0 * shadow, 100.0 * aside);
      check(behind[0] > 0.005, "the wavemaker's train reaches the rock's place");
      check(shadow < 0.6, "behind the rock the waves are lower: a shadow (under 60 % of the open "
                          "sea's height)");
      check(aside > 0.7 && aside < 1.4, "to the side the waves are about as high as without it");
    }
  }

  std::printf("%s: cvcgl_wake (%d check%s failed)\n", g_fail == 0 ? "PASS" : "FAIL", g_fail,
              g_fail == 1 ? "" : "s");
  return g_fail == 0 ? 0 : 1;
}
