/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// fe_lab's interval mesher (mesh.region: interval) on the SDF of a ball, v = |x| - 12 over
// [-16, 16]^3 at 33^3 voxels (one unit per voxel). The interval [-6, -2] is the spherical shell
// 6 <= r <= 10, whose volume is 4/3 pi (10^3 - 6^3) = 3284.0:
//   - tetrahedralize_interval (the one helper every interval mesh goes through) gives the same
//     mesh with the bounds in either order; every tet corner samples inside the interval to a
//     voxel; the tets' volume is the shell's; the boundary is two sheets -- one at r = 6, one at
//     r = 10 -- and after orient_tets + tet_boundary_surface + compute_vertex_normals (what
//     fe_lab does) every tet is right-handed and each sheet's normals point out of the shell (the
//     outer away from the centre, the inner into the cavity). Both radii are whole voxels, so
//     grid points sit exactly on both bounds (ties: (0, 0, 10) has v = -2): the helper must keep
//     the shell whole through them;
//   - Poisson holds u = 0 on both sheets and nothing between them, and comes within 30% of the
//     shell's analytic peak (today's LBIE leaves slivers by the concave inner sheet: ~22% low),
//     and the heat problem's hot floor is held at 1;
//   - an interval that cuts no voxel, or an empty one, says so ("no tets: the interval ...");
//   - the interval is inclusive on integer data: a 0/1 mask in [0.5, 1] (the bound on its
//     maximum) meshes {v >= 0.5} -- the same mesh as [0.5, 2] and as [1, 2] (the lower bound on
//     the 1s), one sheet crossing at 0.5 -- and labels [1, 3] keep labels 1 and 3, watertight (two
//     sheets): a bound voxels sit on is meshed half way to the next value beyond it, so labels
//     [1, 3] and [2, 3] are the meshes of [0.5, 3.5] and [1.5, 3.5];
//   - the fe_lab node meshes and solves the shell from `interval: [-6, -2]`, reports a miss in its
//     status (keeping the last mesh), and region solid meshes the ball as before; a bound written
//     in the same tick as the region keeps its value (the binding moves its bar to it);
//   - a mask node from `interval: [0.5, 2]` keeps the bound past its data (its bar pinned at the
//     axis' end) and meshes once; an editor drag followed by Tet-mesh within the remesh debounce
//     meshes once, not again when that job ends.
// These are also the acceptance tests for the follow-up that routes the helper through a fixed
// cvc::tetrahedralize2 interval path.
//
// No rendering: the node's tick is run directly. Plain main, non-zero on failure (NDEBUG is defined
// in Release, so no assert()).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/mesh_nodes.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/utility/algorithm.h> // cvc::tetrahedralize, for a solid mesh to compare
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <exception>
#include <numeric>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#ifdef CVC_ENABLE_LIBIGL
#include <cvc/geometry/fem.h>
#endif

using cvc::ariadne::Scene;
using cvc::ariadne::SceneNode;
using cvc::ariadne::Value;
using cvc::gl::SceneGraph;
using cvc::gl::ariadne::RealizedScene;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr unsigned kN = 33;              // voxels per axis
constexpr double kHalf = 16.0;           // the box is [-kHalf, kHalf]^3: one unit per voxel
constexpr double kR = 12.0;              // the ball's radius
constexpr double kLo = -6.0, kHi = -2.0; // the interval: the shell 6 <= r <= 10
constexpr double kRin = kR + kLo, kRout = kR + kHi; // 6, 10
constexpr double kVoxel = 2.0 * kHalf / (kN - 1);   // 1

cvc::volume ball_sdf(cvc::app &app) {
  cvc::volume vol(app, cvc::dimension(kN, kN, kN), cvc::Float,
                  cvc::bounding_box(-kHalf, -kHalf, -kHalf, kHalf, kHalf, kHalf));
  for (unsigned k = 0; k < kN; ++k)
    for (unsigned j = 0; j < kN; ++j)
      for (unsigned i = 0; i < kN; ++i) {
        const double x = -kHalf + kVoxel * i, y = -kHalf + kVoxel * j, z = -kHalf + kVoxel * k;
        vol(i, j, k, std::sqrt(x * x + y * y + z * z) - kR);
      }
  return vol;
}

// Integer data on the same grid (UChar): `f` of the radius at each voxel.
template <typename F> cvc::volume radial_uchar(cvc::app &app, F f) {
  cvc::volume vol(app, cvc::dimension(kN, kN, kN), cvc::UChar,
                  cvc::bounding_box(-kHalf, -kHalf, -kHalf, kHalf, kHalf, kHalf));
  for (unsigned k = 0; k < kN; ++k)
    for (unsigned j = 0; j < kN; ++j)
      for (unsigned i = 0; i < kN; ++i) {
        const double x = -kHalf + kVoxel * i, y = -kHalf + kVoxel * j, z = -kHalf + kVoxel * k;
        vol(i, j, k, f(std::sqrt(x * x + y * y + z * z)));
      }
  return vol;
}
// A 0/1 mask: the ball r <= 8.
constexpr double kMaskR = 8.0;
cvc::volume mask_ball(cvc::app &app) {
  return radial_uchar(app, [](double r) { return r <= kMaskR ? 1.0 : 0.0; });
}
// A label volume of concentric shells three voxels thick: label floor(r / 3), at most 5.
cvc::volume label_shells(cvc::app &app) {
  return radial_uchar(app, [](double r) { return std::min(5.0, std::floor(r / 3.0)); });
}

double radius(const cvc::geometry::point_t &p) {
  return std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
}

// The volume's value at p (trilinear), p clamped into its box.
double sample(const cvc::volume &vol, const cvc::geometry::point_t &p) {
  const auto c = [](double x) { return std::min(std::max(x, -kHalf), kHalf); };
  try {
    return vol.interpolate(c(p[0]), c(p[1]), c(p[2]));
  } catch (const std::exception &) {
    return std::nan("");
  }
}

// Connected components of a triangle surface's referenced vertices: a label per vertex (-1 when
// unreferenced) and the count.
std::vector<int> components(const cvc::geometry &s, int &count) {
  const std::size_t nv = s.const_points().size();
  std::vector<std::size_t> parent(nv);
  std::iota(parent.begin(), parent.end(), std::size_t(0));
  const auto find = [&](std::size_t v) {
    while (parent[v] != v)
      v = parent[v] = parent[parent[v]];
    return v;
  };
  std::vector<char> used(nv, 0);
  for (const cvc::geometry::tri_t &t : s.const_tris())
    for (int e = 0; e < 3; ++e) {
      used[t[e]] = 1;
      parent[find(t[e])] = find(t[(e + 1) % 3]);
    }
  std::vector<int> label(nv, -1), root_label(nv, -1);
  count = 0;
  for (std::size_t v = 0; v < nv; ++v) {
    if (!used[v])
      continue;
    const std::size_t r = find(v);
    if (root_label[r] < 0)
      root_label[r] = count++;
    label[v] = root_label[r];
  }
  return label;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}
void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}
double num(cvc::app &app, const std::string &path) {
  try {
    return cvc::state::instance(app)(path).value<double>();
  } catch (...) {
    return std::nan("");
  }
}
bool starts(const std::string &s, const std::string &prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}
Value scalar(const std::string &s) {
  Value v;
  v.kind = Value::Kind::Scalar;
  v.scalar = s;
  return v;
}
Value seq(const std::vector<std::string> &items) {
  Value v;
  v.kind = Value::Kind::Sequence;
  for (const std::string &s : items)
    v.items.push_back(scalar(s));
  return v;
}

// The radii of a tet mesh's corners: [min, max].
std::pair<double, double> corner_radii(const cvc::geometry &tm) {
  double lo = HUGE_VAL, hi = -HUGE_VAL;
  for (const cvc::geometry::tet_t &t : tm.const_tets())
    for (int c = 0; c < 4; ++c) {
      const double r = radius(tm.const_points()[t[c]]);
      lo = std::min(lo, r);
      hi = std::max(hi, r);
    }
  return {lo, hi};
}

// The boundary of a tet mesh of a shell round the origin, as fe_lab builds it (orient_tets +
// tet_boundary_surface + compute_vertex_normals): its sheet count, and how many of its vertices'
// normals point out of the shell -- away from the centre on the outermost sheet, towards it on
// an inner one -- of how many.
int shell_sheets(const cvc::geometry &tm, std::size_t &good, std::size_t &n) {
  good = n = 0;
  cvc::geometry t(tm);
  cvc::orient_tets(t);
  cvc::geometry s = cvc::tet_boundary_surface(t);
  cvc::compute_vertex_normals(s);
  int count = 0;
  const std::vector<int> label = components(s, count);
  std::vector<double> rsum(static_cast<std::size_t>(std::max(count, 0)), 0.0);
  std::vector<std::size_t> rn(rsum.size(), 0);
  for (std::size_t v = 0; v < label.size(); ++v)
    if (label[v] >= 0) {
      rsum[std::size_t(label[v])] += radius(s.const_points()[v]);
      ++rn[std::size_t(label[v])];
    }
  int outer = 0;
  for (int c = 1; c < count; ++c)
    if (rsum[std::size_t(c)] / double(rn[std::size_t(c)]) >
        rsum[std::size_t(outer)] / double(rn[std::size_t(outer)]))
      outer = c;
  const cvc::geometry::normals_t &nn = s.const_normals();
  for (std::size_t v = 0; v < label.size() && v < nn.size(); ++v) {
    if (label[v] < 0)
      continue;
    const cvc::geometry::point_t &p = s.const_points()[v];
    const double d = nn[v][0] * p[0] + nn[v][1] * p[1] + nn[v][2] * p[2];
    ++n;
    good += (label[v] == outer ? d > 0.0 : d < 0.0) ? 1 : 0;
  }
  return count;
}

// The shell's Poisson solution for -lap u = 1, u = 0 at r = a and r = b: -r^2/6 + A/r + B.
double shell_poisson(double r, double a, double b) {
  const double A = -(a + b) * a * b / 6.0;
  const double B = a * a / 6.0 - A / a;
  return -r * r / 6.0 + A / r + B;
}

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  const cvc::volume vol = ball_sdf(app);

#ifndef CVC_ENABLE_MESHER
  std::printf("skip: libcvc built without CVC_ENABLE_MESHER\n");
  return 0;
#else
  std::printf("== tetrahedralize_interval: the shell 6 <= r <= 10 ==\n");
  cvc::geometry tm;
  try {
    tm = cvc::gl::ariadne::tetrahedralize_interval(vol, kHi, kLo); // the upper bound first
  } catch (const std::exception &e) {
    std::printf("     threw: %s\n", e.what());
  }
  const std::size_t ntets = tm.const_tets().size();
  chk(ntets > 1000, "the shell is meshed (" + std::to_string(ntets) + " tets)");
  {
    cvc::geometry again;
    try {
      again = cvc::gl::ariadne::tetrahedralize_interval(vol, kLo, kHi);
    } catch (const std::exception &) {
    }
    chk(again.const_tets().size() == ntets &&
            again.const_points().size() == tm.const_points().size(),
        "the bounds in either order give the same mesh");
  }

  // Every tet corner samples inside the interval, to a voxel (|grad v| = 1: one value unit).
  std::vector<char> corner(tm.const_points().size(), 0);
  for (const cvc::geometry::tet_t &t : tm.const_tets())
    for (int c = 0; c < 4; ++c)
      corner[t[c]] = 1;
  double vmin = HUGE_VAL, vmax = -HUGE_VAL;
  std::size_t outside = 0, corners = 0;
  for (std::size_t i = 0; i < corner.size(); ++i) {
    if (!corner[i])
      continue;
    ++corners;
    const double v = sample(vol, tm.const_points()[i]);
    vmin = std::min(vmin, v);
    vmax = std::max(vmax, v);
    if (!(v >= kLo - kVoxel && v <= kHi + kVoxel))
      ++outside;
  }
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "every tet corner samples within [-6, -2] to a voxel (%zu corners in "
                "[%.3f, %.3f], %zu outside)",
                corners, vmin, vmax, outside);
  chk(corners > 0 && outside == 0, buf);

  // orient_tets (as fe_lab does): every tet right-handed, the shell's volume.
  cvc::orient_tets(tm);
  const std::vector<double> vols = cvc::tet_volumes(tm);
  double total = 0.0;
  std::size_t flat = 0;
  for (double v : vols) {
    total += v;
    if (!(v > 0.0))
      ++flat;
  }
  const double exact = 4.0 / 3.0 * kPi * (kRout * kRout * kRout - kRin * kRin * kRin);
  std::snprintf(buf, sizeof(buf), "after orient_tets every tet is right-handed (%zu not)", flat);
  chk(!vols.empty() && flat == 0, buf);
  std::snprintf(buf, sizeof(buf), "the tets fill the shell: volume %.1f vs %.1f exact (%+.1f%%)",
                total, exact, 100.0 * (total - exact) / exact);
  chk(std::fabs(total - exact) <= 0.08 * exact, buf);

  // The boundary: two sheets, at r = 6 and r = 10, wound out of the shell.
  cvc::geometry surf = cvc::tet_boundary_surface(tm);
  cvc::compute_vertex_normals(surf);
  int count = 0;
  const std::vector<int> label = components(surf, count);
  std::vector<double> rsum(static_cast<std::size_t>(std::max(count, 0)), 0.0);
  std::vector<std::size_t> rn(rsum.size(), 0);
  for (std::size_t v = 0; v < label.size(); ++v)
    if (label[v] >= 0) {
      rsum[std::size_t(label[v])] += radius(surf.const_points()[v]);
      ++rn[std::size_t(label[v])];
    }
  std::snprintf(buf, sizeof(buf), "the boundary is two sheets (%d component%s)", count,
                count == 1 ? "" : "s");
  chk(count == 2, buf);
  if (count == 2) {
    const double r0 = rsum[0] / double(rn[0]), r1 = rsum[1] / double(rn[1]);
    const double rin = std::min(r0, r1), rout = std::max(r0, r1);
    const int inner = r0 < r1 ? 0 : 1;
    std::snprintf(buf, sizeof(buf), "... the inner at r = %.2f (6), the outer at r = %.2f (10)",
                  rin, rout);
    chk(std::fabs(rin - kRin) <= 0.5 * kVoxel && std::fabs(rout - kRout) <= 0.5 * kVoxel, buf);
    // compute_vertex_normals from the windings: out of the shell on both sheets.
    std::size_t good = 0, n = 0;
    const cvc::geometry::normals_t &nn = surf.const_normals();
    for (std::size_t v = 0; v < label.size() && v < nn.size(); ++v) {
      if (label[v] < 0)
        continue;
      const cvc::geometry::point_t &p = surf.const_points()[v];
      const double d = nn[v][0] * p[0] + nn[v][1] * p[1] + nn[v][2] * p[2];
      ++n;
      if (label[v] == inner ? d < 0.0 : d > 0.0)
        ++good;
    }
    std::snprintf(buf, sizeof(buf),
                  "... their normals point out of the shell: the outer away from the centre, "
                  "the inner into the cavity (%zu of %zu)",
                  good, n);
    chk(n > 0 && good == n, buf);
  }

#ifdef CVC_ENABLE_LIBIGL
  std::printf("== the FEM solves on the shell ==\n");
  {
    const cvc::fem::domain dom = cvc::fem::domain::VOLUME;
    const std::vector<cvc::index_t> bnd = cvc::fem::boundary_vertices(tm, dom);
    std::size_t on_inner = 0, on_outer = 0, deep = 0;
    for (cvc::index_t v : bnd) {
      const double r = radius(tm.const_points()[v]);
      (r < 0.5 * (kRin + kRout) ? on_inner : on_outer)++;
      deep += (r > kRin + 0.75 * kVoxel && r < kRout - 0.75 * kVoxel) ? 1 : 0;
    }
    chk(on_inner > 0 && on_outer > 0 && deep == 0,
        "fem::boundary_vertices finds both sheets and nothing between them (" +
            std::to_string(on_inner) + " inner, " + std::to_string(on_outer) + " outer, " +
            std::to_string(deep) + " deep inside)");
    cvc::fem::dirichlet bc;
    bc.vertices = bnd;
    bc.values.push_back(0.0);
    const std::vector<double> u = cvc::fem::solve_poisson(tm, {1.0}, bc, dom);
    double umax = -HUGE_VAL, umin = HUGE_VAL, err = 0.0, bmax = 0.0;
    std::size_t n = 0;
    for (std::size_t i = 0; i < u.size(); ++i) {
      if (!corner[i])
        continue;
      umax = std::max(umax, u[i]);
      umin = std::min(umin, u[i]);
      const double r = std::min(std::max(radius(tm.const_points()[i]), kRin), kRout);
      err += std::fabs(u[i] - shell_poisson(r, kRin, kRout));
      ++n;
    }
    err /= double(std::max<std::size_t>(n, 1));
    for (cvc::index_t v : bnd)
      bmax = std::max(bmax, std::fabs(u[v]));
    // u'(r) = -r/3 - A/r^2 = 0 at r^3 = -3A = (a + b) a b / 2.
    const double peak = shell_poisson(std::cbrt((kRin + kRout) * kRin * kRout / 2.0), kRin, kRout);
    // The bounds hold today's LBIE: its tets on the shell's concave inner sheet include slivers
    // (counted below; the convex solid ball has none), which pull the discrete peak ~22% under the
    // analytic one. A mesher with no slivers (transfix/libcvc#808, or the fixed tetrahedralize2)
    // must land well inside them.
    std::snprintf(buf, sizeof(buf),
                  "poisson: u = 0 on both sheets (max |u| there %.2g), u >= %.3f inside, peak %.3f "
                  "vs %.3f exact, mean error %.3f",
                  bmax, umin, umax, peak, err);
    chk(u.size() == tm.const_points().size() && bmax <= 1e-9 && umin >= -0.05 * peak &&
            std::fabs(umax - peak) <= 0.3 * peak && err <= 0.25 * peak,
        buf);
    {
      std::size_t slivers = 0, near_inner = 0;
      const double mean = total / double(std::max<std::size_t>(vols.size(), 1));
      for (std::size_t t = 0; t < vols.size(); ++t) {
        if (!(vols[t] < 1e-3 * mean))
          continue;
        ++slivers;
        cvc::geometry::point_t c = {{0.0, 0.0, 0.0}};
        for (int k = 0; k < 4; ++k)
          for (int d = 0; d < 3; ++d)
            c[d] += 0.25 * tm.const_points()[tm.const_tets()[t][k]][d];
        near_inner += radius(c) < 0.5 * (kRin + kRout) ? 1 : 0;
      }
      std::printf("     note: %zu slivers (under 1e-3 of the mean tet volume), %zu of them nearer "
                  "the inner sheet\n",
                  slivers, near_inner);
    }

    // Heat: the hot floor (the boundary's lowest 5%, which on a shell is the outer sheet's
    // bottom cap) held at 1, the rest insulated; heat flows up through the shell.
    double zlo = HUGE_VAL, zhi = -HUGE_VAL;
    for (cvc::index_t v : bnd) {
      zlo = std::min(zlo, tm.const_points()[v][2]);
      zhi = std::max(zhi, tm.const_points()[v][2]);
    }
    cvc::fem::dirichlet hot;
    for (cvc::index_t v : bnd)
      if (tm.const_points()[v][2] <= zlo + 0.05 * (zhi - zlo))
        hot.vertices.push_back(v);
    hot.values.push_back(1.0);
    cvc::fem::heat_params hp;
    hp.kappa = 1.0 * (2.0 * kRout) * (2.0 * kRout); // fe_lab's relative kappa 1
    hp.dt = 0.01;
    hp.steps = 20;
    const std::vector<double> h =
        cvc::fem::solve_heat(tm, std::vector<double>(tm.const_points().size(), 0.0), hp, hot, dom);
    double hmax = -HUGE_VAL, top = 0.0;
    for (std::size_t i = 0; i < h.size(); ++i)
      if (corner[i]) {
        hmax = std::max(hmax, h[i]);
        if (tm.const_points()[i][2] > 0.8 * kRout)
          top = std::max(top, h[i]);
      }
    std::size_t floor_ok = 0;
    for (cvc::index_t v : hot.vertices)
      floor_ok += std::fabs(h[v] - 1.0) <= 1e-9 ? 1 : 0;
    std::snprintf(buf, sizeof(buf),
                  "heat: the hot floor (%zu vertices) is held at 1, max %.4f; the top warms "
                  "(%.3f)",
                  hot.vertices.size(), hmax, top);
    chk(!hot.vertices.empty() && floor_ok == hot.vertices.size() && hmax <= 1.0 + 1e-6 &&
            top > 0.0 && top < 1.0,
        buf);
  }
#endif

  std::printf("== intervals that mesh nothing ==\n");
  const auto message = [&](double a, double b) {
    try {
      cvc::gl::ariadne::tetrahedralize_interval(vol, a, b);
    } catch (const std::exception &e) {
      return std::string(e.what());
    }
    return std::string("(no exception)");
  };
  {
    const std::string miss = message(20.0, 30.0);
    chk(starts(miss,
               "no tets: the interval [20, 30] does not cut the volume (its values span [-12"),
        "an interval past the volume's values says so: '" + miss + "'");
    const std::string empty = message(-4.0, -4.0);
    chk(starts(empty, "no tets: the interval [-4, -4] is empty"),
        "equal bounds say the interval is empty: '" + empty + "'");
  }

  std::printf("== the interval is inclusive: voxels on a bound are inside ==\n");
  {
    // A 0/1 mask with its upper bound on the data's maximum (a bar at the right end of an
    // editor's axis) or past it ("0.5 and up"): both mesh {v >= 0.5}, the ball crossing at 0.5.
    // The v = 1 voxels on the bound used to count as outside: "does not cut the volume".
    const cvc::volume mask = mask_ball(app);
    cvc::geometry at_end, past_end;
    std::string why;
    try {
      at_end = cvc::gl::ariadne::tetrahedralize_interval(mask, 0.5, 1.0);
      past_end = cvc::gl::ariadne::tetrahedralize_interval(mask, 2.0, 0.5);
    } catch (const std::exception &e) {
      why = e.what();
    }
    chk(!at_end.const_tets().empty(),
        "a 0/1 mask meshes in [0.5, 1] (" +
            (why.empty() ? std::to_string(at_end.const_tets().size()) + " tets" : why) + ")");
    chk(past_end.const_tets().size() == at_end.const_tets().size() &&
            past_end.const_points().size() == at_end.const_points().size(),
        "... the same mesh as [0.5, 2]: a bound no voxel lies past bounds nothing");
    {
      // The lower bound on the 1s ([1, 2]): they are inside, the bound meshed half way to the 0s.
      cvc::geometry on_ones;
      std::string w;
      try {
        on_ones = cvc::gl::ariadne::tetrahedralize_interval(mask, 1.0, 2.0);
      } catch (const std::exception &e) {
        w = e.what();
      }
      chk(!on_ones.const_tets().empty() &&
              on_ones.const_tets().size() == at_end.const_tets().size() &&
              on_ones.const_points().size() == at_end.const_points().size(),
          "... and [1, 2], its lower bound on the 1s, meshes them the same (" +
              (w.empty() ? std::to_string(on_ones.const_tets().size()) + " tets" : w) + ")");
    }
    {
      // ... which is the solid mesh of {v > 0.5}: the bound on the maximum adds no sheet or crease.
      cvc::geometry solid;
      try {
        solid = cvc::tetrahedralize(mask, 0.5, cvc::DUALLIB, cvc::NO_IMPROVE,
                                    cvc::BSPLINE_CONVOLUTION, 1);
      } catch (const std::exception &) {
      }
      std::size_t good = 0, n = 0;
      const int count = at_end.const_tets().empty() ? 0 : shell_sheets(at_end, good, n);
      std::snprintf(buf, sizeof(buf),
                    "... and the solid mesh of v > 0.5 (%zu vs %zu tets): one sheet (%d), its "
                    "normals outward (%zu of %zu)",
                    at_end.const_tets().size(), solid.const_tets().size(), count, good, n);
      chk(!solid.const_tets().empty() && solid.const_tets().size() == at_end.const_tets().size() &&
              count == 1 && n > 0 && good == n,
          buf);
    }
    {
      // Both bounds on the data's ends ([0, 1] on the mask): every voxel is in the interval, so
      // the whole box is meshed, as for an interval past both ends ([-20, 20] on the SDF ball).
      cvc::geometry all, wider;
      why.clear();
      try {
        all = cvc::gl::ariadne::tetrahedralize_interval(mask, 0.0, 1.0);
        wider = cvc::gl::ariadne::tetrahedralize_interval(vol, -20.0, 20.0);
      } catch (const std::exception &e) {
        why = e.what();
      }
      chk(!all.const_tets().empty() && all.const_tets().size() == wider.const_tets().size(),
          "[0, 1] on the mask meshes the whole box like [-20, 20] on the ball (" +
              (why.empty() ? std::to_string(all.const_tets().size()) + " vs " +
                                 std::to_string(wider.const_tets().size()) + " tets"
                           : why) +
              ")");
    }

    // Labels 0..5 in shells three voxels thick: [1, 3] keeps labels 1 and 3 (3 <= r < 12), not
    // just label 2 (6 <= r < 9), and stays watertight through the voxels sitting on both bounds.
    const cvc::volume labels = label_shells(app);
    cvc::geometry lt;
    why.clear();
    try {
      lt = cvc::gl::ariadne::tetrahedralize_interval(labels, 1.0, 3.0);
    } catch (const std::exception &e) {
      why = e.what();
    }
    const std::pair<double, double> rr = corner_radii(lt);
    std::snprintf(buf, sizeof(buf),
                  "labels [1, 3]: the shell spans r in [%.2f, %.2f] (labels 1..3: 3 <= r < 12)%s%s",
                  rr.first, rr.second, why.empty() ? "" : " -- ", why.c_str());
    chk(!lt.const_tets().empty() && rr.first < 4.0 && rr.second > 11.0, buf);
    std::size_t good = 0, n = 0;
    const int ls = lt.const_tets().empty() ? 0 : shell_sheets(lt, good, n);
    std::snprintf(buf, sizeof(buf),
                  "... two sheets (%d) through the voxels on both bounds, their normals out of the "
                  "shell (%zu of %zu)",
                  ls, good, n);
    chk(ls == 2 && n > 0 && good == n, buf);
    // The rule: a bound voxels sit on is meshed half way to the nearest value beyond it, so on
    // integer data [a, b] is [a - 0.5, b + 0.5] -- here and when EVERY voxel in the interval sits
    // on a bound ([2, 3]: labels 2 and 3 only, nothing strictly between).
    const auto same_as = [&](double a, double b, double ha, double hb) {
      cvc::geometry m, h;
      std::string w;
      try {
        m = cvc::gl::ariadne::tetrahedralize_interval(labels, a, b);
        h = cvc::gl::ariadne::tetrahedralize_interval(labels, ha, hb);
      } catch (const std::exception &e) {
        w = e.what();
      }
      std::size_t g2 = 0, n2 = 0;
      const int s2 = m.const_tets().empty() ? 0 : shell_sheets(m, g2, n2);
      std::snprintf(buf, sizeof(buf),
                    "labels [%g, %g] is the mesh of [%g, %g] (%zu vs %zu tets), two sheets (%d), "
                    "normals out of the shell (%zu of %zu)%s%s",
                    a, b, ha, hb, m.const_tets().size(), h.const_tets().size(), s2, g2, n2,
                    w.empty() ? "" : " -- ", w.c_str());
      chk(!m.const_tets().empty() && m.const_tets().size() == h.const_tets().size() &&
              m.const_points().size() == h.const_points().size() && s2 == 2 && n2 > 0 && g2 == n2,
          buf);
    };
    same_as(1.0, 3.0, 0.5, 3.5);
    same_as(2.0, 3.0, 1.5, 3.5);
  }

#ifdef CVC_ENABLE_LIBIGL
  std::printf("== the fe_lab node ==\n");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  if (!cvc::gl::ariadne::has_scene_node_type("fe_lab")) {
    chk(false, "fe_lab is registered");
  } else {
    const std::string path = "cvcgl_fe_interval_ball.rawiv";
    cvc::writeVolumeFile(app, vol, path);
    SceneGraph sg(app, "fei");
    Scene scene;
    SceneNode d;
    d.id = "domain";
    d.type = "fe_lab";
    d.source_file = path;
    d.props.kind = Value::Kind::Map;
    d.props.entries.emplace_back("interval", seq({"-6", "-2"}));
    d.props.entries.emplace_back("inside", scalar("below"));
    d.props.entries.emplace_back("auto_mesh", scalar("true"));
    d.props.entries.emplace_back("auto_remesh", scalar("false"));
    scene.nodes.push_back(d);
    std::vector<std::string> warnings;
    RealizedScene r = cvc::gl::ariadne::realize_scene(sg, scene, "fei", &warnings);
    for (const std::string &w : warnings)
      std::printf("     warning: %s\n", w.c_str());
    auto dn = sg.getGraphics("domain");
    if (!dn) {
      chk(false, "fe_lab realized");
    } else {
      const std::string fb = dn->stateName("fe");
      const auto tick = [&] {
        for (const auto &t : r.custom_ticks)
          t(nullptr);
      };
      // Tick until the request was taken and its job applied; bounded at ~2 minutes.
      const auto settle = [&] {
        for (int i = 0; i < 12000; ++i) {
          tick();
          if (val(app, fb + ".request").empty() && val(app, fb + ".busy") == "0")
            return true;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
      };
      chk(val(app, fb + ".mesh.region") == "interval" && num(app, fb + ".mesh.isovalue") == kLo &&
              num(app, fb + ".mesh.isovalue2") == kHi,
          "interval: [-6, -2] seeds mesh.region interval, isovalue -6, isovalue2 -2");
      {
        // The transfer-function model holds both bounds as iso1 / iso2 over the volume's range.
        const std::string T = dn->stateName("transfer_function");
        cvc::tf::model m;
        const double lo = vol.min(), hi = vol.max();
        const auto at = [&](const char *id) {
          const int i = m.find_isocontour(id);
          return i < 0 ? std::nan("") : cvc::tf::to_data(m.isocontours[std::size_t(i)].at, lo, hi);
        };
        chk(cvc::tf::from_string(val(app, T), m) && m.isocontours.size() == 2 &&
                std::fabs(at("iso1") - kLo) < 1e-6 && std::fabs(at("iso2") - kHi) < 1e-6 &&
                val(app, T + ".interval") == "iso1,iso2",
            "its transfer function holds iso1 at -6 and iso2 at -2, <tf>.interval = '" +
                val(app, T + ".interval") + "'");
      }
      chk(settle(), "auto_mesh: the interval mesh job finished");
      const std::string status = val(app, fb + ".status");
      const double tets = num(app, fb + ".stats.tets");
      chk(starts(status,
                 "meshed: " + std::to_string(static_cast<long long>(ntets)) + " tets in [-6, -2]"),
          "the node meshed the shell, the same mesh as the helper (status '" + status + "')");

      put(app, fb + ".problem", "poisson");
      put(app, fb + ".request", "solve");
      const bool solved = settle();
      const double fmin = num(app, fb + ".field.min"), fmax = num(app, fb + ".field.max");
      const double peak = 2.0; // shell_poisson's maximum on [6, 10], ~2.01 (the bound: see above)
      std::snprintf(buf, sizeof(buf), "the node's Poisson solve: field [%.3g, %.3f] (%s)", fmin,
                    fmax, val(app, fb + ".status").c_str());
      chk(solved && std::fabs(fmin) <= 0.05 * peak && std::fabs(fmax - peak) <= 0.3 * peak, buf);
      put(app, fb + ".problem", "heat");
      put(app, fb + ".heat.steps", "5");
      put(app, fb + ".request", "solve");
      const bool heated = settle();
      const double hmax = num(app, fb + ".field.max");
      chk(heated && hmax >= 1.0 - 1e-9 && hmax <= 1.0 + 1e-6,
          "the node's heat solve holds its hot floor at 1 (max " + val(app, fb + ".field.max") +
              ")");

      // An interval past the volume: the status says why, the last mesh stays.
      put(app, fb + ".mesh.isovalue", "20");
      put(app, fb + ".mesh.isovalue2", "30");
      put(app, fb + ".request", "mesh");
      const bool missed = settle();
      const std::string why = val(app, fb + ".status");
      chk(missed && starts(why, "error: no tets: the interval [20, 30] does not cut the volume") &&
              num(app, fb + ".stats.tets") == tets,
          "a missing interval reports it and keeps the last mesh (status '" + why + "')");

      // Region solid: the ball r < 12 as before (isovalue 0, inside below), not a shell.
      put(app, fb + ".mesh.isovalue", "0");
      put(app, fb + ".mesh.region", "solid");
      put(app, fb + ".request", "mesh");
      const bool solid = settle();
      const std::string ss = val(app, fb + ".status");
      chk(solid && starts(ss, "meshed: ") && ss.find(" in [") == std::string::npos &&
              num(app, fb + ".stats.tets") > 0.0 && num(app, fb + ".stats.tets") != tets,
          "region solid meshes the whole ball again (status '" + ss + "')");

      // A bound written in the same tick as the region keeps its value and moves its bar: the
      // binding must not lower the bars' old positions over it (iso1 sat at the end, 15.71, after
      // the [20, 30] step). Then the next Tet-mesh meshes what the state says.
      const std::string T = dn->stateName("transfer_function");
      const auto bar = [&](const char *id) {
        cvc::tf::model m;
        if (!cvc::tf::from_string(val(app, T), m) || m.find_isocontour(id) < 0)
          return std::nan("");
        return cvc::tf::to_data(m.isocontours[std::size_t(m.find_isocontour(id))].at, vol.min(),
                                vol.max());
      };
      chk(num(app, fb + ".mesh.isovalue") == 0.0 && std::fabs(bar("iso1")) < 1e-6,
          "isovalue 0 written with region solid stays 0 (mesh.isovalue " +
              val(app, fb + ".mesh.isovalue") + ") and moved iso1 there");
      put(app, fb + ".mesh.isovalue2", "-3");
      put(app, fb + ".mesh.region", "interval");
      tick();
      chk(num(app, fb + ".mesh.isovalue2") == -3.0 && std::fabs(bar("iso2") + 3.0) < 1e-6 &&
              val(app, T + ".interval") == "iso1,iso2",
          "isovalue2 -3 written with region interval stays -3 (mesh.isovalue2 " +
              val(app, fb + ".mesh.isovalue2") + ") and takes iso2 off the end");
    }
    std::remove(path.c_str());

    // A 0/1 mask meshed as `interval: [0.5, 2]` ("0.5 and up"), auto_mesh, auto_remesh with a
    // short debounce. The bound past the data is kept (its bar pinned at the axis' end): attaching
    // neither rewrites it with the data's maximum nor arms a remesh, so exactly one mesh runs.
    const std::string mpath = "cvcgl_fe_interval_mask.rawiv";
    cvc::writeVolumeFile(app, mask_ball(app), mpath);
    SceneGraph mg(app, "fem");
    Scene ms;
    SceneNode md;
    md.id = "mask";
    md.type = "fe_lab";
    md.source_file = mpath;
    md.props.kind = Value::Kind::Map;
    md.props.entries.emplace_back("interval", seq({"0.5", "2"}));
    md.props.entries.emplace_back("auto_mesh", scalar("true"));
    md.props.entries.emplace_back("remesh_debounce", scalar("0.05"));
    ms.nodes.push_back(md);
    std::vector<std::string> mwarn;
    RealizedScene mr = cvc::gl::ariadne::realize_scene(mg, ms, "fem", &mwarn);
    for (const std::string &w : mwarn)
      std::printf("     warning: %s\n", w.c_str());
    auto mn = mg.getGraphics("mask");
    if (!mn) {
      chk(false, "fe_lab realized the mask");
    } else {
      const std::string mb = mn->stateName("fe"), mT = mn->stateName("transfer_function");
      // Tick until idle (no job, no request), then on for 0.3 s -- six debounces -- counting the
      // mesh jobs that started; bounded at ~2 minutes.
      const auto run = [&](int &jobs) {
        jobs = 0;
        bool was_busy = val(app, mb + ".busy") == "1";
        int idle_ticks = 0;
        for (int i = 0; i < 12000; ++i) {
          for (const auto &t : mr.custom_ticks)
            t(nullptr);
          const bool busy = val(app, mb + ".busy") == "1";
          jobs += busy && !was_busy ? 1 : 0;
          was_busy = busy;
          idle_ticks = !busy && val(app, mb + ".request").empty() ? idle_ticks + 1 : 0;
          if (idle_ticks >= 30)
            return true;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return false;
      };
      cvc::tf::model m;
      const bool parsed = cvc::tf::from_string(val(app, mT), m);
      const int i2 = parsed ? m.find_isocontour("iso2") : -1;
      chk(num(app, mb + ".mesh.isovalue2") == 2.0 && i2 >= 0 &&
              m.isocontours[std::size_t(i2)].at == 1.0,
          "attaching keeps mesh.isovalue2 2 past the data's maximum 1 (" +
              val(app, mb + ".mesh.isovalue2") + "), iso2 pinned at the axis' end");
      int jobs = 0;
      const bool ran = run(jobs);
      const std::string status = val(app, mb + ".status");
      chk(ran && jobs == 1 && starts(status, "meshed: ") &&
              status.find(" in [0.5, 2]") != std::string::npos &&
              num(app, mb + ".mesh.isovalue2") == 2.0,
          "auto_mesh meshes [0.5, 2] once and nothing re-meshes it (" + std::to_string(jobs) +
              " job(s), status '" + status + "')");

      // An editor drag arms the debounced remesh; Tet-mesh pressed within the debounce meshes the
      // new bounds, and that is the remesh: it must not run again once that job is done.
      if (cvc::tf::from_string(val(app, mT), m) && m.find_isocontour("iso1") >= 0) {
        m.isocontours[std::size_t(m.find_isocontour("iso1"))].at = 0.4;
        put(app, mT, cvc::tf::to_string(m));
      }
      for (const auto &t : mr.custom_ticks) // the binding lowers the drag and arms the remesh
        t(nullptr);
      const bool lowered = std::fabs(num(app, mb + ".mesh.isovalue") - 0.4) < 1e-12 &&
                           val(app, mb + ".request").empty();
      put(app, mb + ".request", "mesh"); // fe_controls' Tet-mesh
      const bool reran = run(jobs);
      const std::string again = val(app, mb + ".status");
      chk(lowered && reran && jobs == 1 && again.find(" in [0.4, 2]") != std::string::npos,
          "a drag then Tet-mesh within the debounce meshes once (" + std::to_string(jobs) +
              " job(s), status '" + again + "')");
    }
    std::remove(mpath.c_str());
  }
#endif

  std::printf("%s (%d failure%s)\n", fails ? "FAILED" : "OK", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
#endif
}
