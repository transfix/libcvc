/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the legacy SecondaryStructures)
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::find_secondary_structures and secondary_stepper (cvc/geometry/secondary_structures.h):
// skel.cpp's compute_secondary_structures() and buildAllGeometry() as one coroutine over the
// cocone core, with the call's own context in place of the legacy Skel object.

#include "secondary.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/secondary_structures_params.h>
#include <optional>

namespace cvc {

namespace {

constexpr std::size_t BUILD_UNITS = 16; // tet_complex_builder units per unit

std::string fmt_count(const char *what, std::size_t n) {
  char buf[256];
  std::snprintf(buf, sizeof buf, what, n);
  return buf;
}

void check_params(const char *who, const secondary_params &params) {
  const std::vector<std::string> problems = params_validate(params);
  if (problems.empty())
    return;
  std::string m = std::string(who) + ": invalid parameters: ";
  for (std::size_t i = 0; i < problems.size(); ++i)
    m += (i ? "; " : "") + problems[i];
  throw secondary_error(m);
}

} // namespace

struct secondary_stepper::impl {
  app *ctx = nullptr;
  std::optional<geometry> source; // the surface, until its points are read
  secondary_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;
  const char *phase_name = "checking";
  double prog = 0.0;

  std::optional<tet_complex_builder> builder;
  cocone::Triangulation triang;
  cocone::diagnostics diag;
  cocone::Graph graph;
  secondary_detail::skeleton_builder sk;
  std::size_t duplicates = 0;
  secondary_structures result;

  double stage_lo = 0.0, stage_hi = 0.0;
  std::size_t stage_units = 0, stage_estimate = 1;

  cocone::step_task task;

  void stage(const char *name, double lo, double hi, std::size_t estimate) {
    phase_name = name;
    stage_lo = lo;
    stage_hi = hi;
    stage_units = 0;
    stage_estimate = std::max<std::size_t>(estimate, 1);
  }
  void note_progress(double within) {
    const double p = stage_lo + (stage_hi - stage_lo) * std::min(1.0, std::max(0.0, within));
    prog = std::max(prog, std::min(p, 1.0));
  }

  cocone::step_task run();
  cocone::step_task read_points(std::vector<std::array<double, 3>> &pts);
  cocone::step_task triangulate(std::vector<std::array<double, 3>> pts, double lo, double hi);
};

// The surface's vertices its faces use (all of its points when it has no faces), in lexicographic
// order with exact duplicates merged, as the legacy std::set<Point> held them.
cocone::step_task secondary_stepper::impl::read_points(std::vector<std::array<double, 3>> &pts) {
  const geometry::points_t &gp = source->const_points();
  const geometry::tris_t &tris = source->const_tris();
  const geometry::quads_t &quads = source->const_quads();
  const std::size_t n = gp.size();
  std::vector<char> used(n, tris.empty() && quads.empty() ? 1 : 0);
  auto use = [&](geometry::index_t i, std::size_t face) {
    if (i >= n)
      throw secondary_error(fmt_count("find_secondary_structures: face %zu names a point past the "
                                      "surface's points",
                                      face));
    used[std::size_t(i)] = 1;
  };
  for (std::size_t f = 0; f < tris.size(); ++f) {
    for (geometry::index_t i : tris[f])
      use(i, f);
    if (f % 1024 == 1023)
      SK_UNIT;
  }
  for (std::size_t f = 0; f < quads.size(); ++f) {
    for (geometry::index_t i : quads[f])
      use(i, tris.size() + f);
    if (f % 1024 == 1023)
      SK_UNIT;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (!used[i])
      continue;
    const auto &q = gp[i];
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2]))
      throw secondary_error(fmt_count("find_secondary_structures: point %zu has a coordinate that "
                                      "is not finite",
                                      i));
    pts.push_back({q[0], q[1], q[2]});
    if (i % 1024 == 1023)
      SK_UNIT;
  }
  source.reset();
  SK_RUN(cocone::sliced_sort(
      pts, [](const std::array<double, 3> &a, const std::array<double, 3> &b) { return a < b; }));
  const std::size_t before = pts.size();
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  duplicates = before - pts.size();
}

// Build a tet_complex of pts into triang, BUILD_UNITS of the builder's units per unit, its phases
// reported as ours.
cocone::step_task secondary_stepper::impl::triangulate(std::vector<std::array<double, 3>> pts,
                                                       double lo, double hi) {
  using namespace cvc::cocone;
  stage(phase_name, lo, lo, 1);
  builder.emplace(std::move(pts), tet_complex_options{});
  for (;;) {
    bool ready = false;
    try {
      ready = builder->step(BUILD_UNITS);
    } catch (const tet_complex_error &e) {
      std::string what = e.what();
      const std::string prefix = "tet_complex: ";
      if (what.rfind(prefix, 0) == 0)
        what = what.substr(prefix.size());
      throw secondary_error("find_secondary_structures: " + what);
    }
    phase_name = builder->phase();
    prog = std::max(prog, lo + (hi - lo) * builder->progress());
    if (ready)
      break;
    SK_UNIT;
  }
  tet_complex t = builder->take();
  builder.reset();
  triang.reset(std::move(t));
  stage("building", hi, hi + 0.02, triang.fill_total());
  while (!triang.fill_one())
    SK_UNIT;
}

cocone::step_task secondary_stepper::impl::run() {
  using namespace cvc::cocone;
  check_params("find_secondary_structures", params);
  if (!tet_complex_available())
    throw secondary_error("find_secondary_structures: libcvc was built without Shewchuk's "
                          "predicates (CVC_WITH_SHEWCHUK), which the triangulation needs");
  const secondary_params &P = params;

  // ---- 1. the points and their Delaunay triangulation ----
  std::vector<std::array<double, 3>> pts;
  SK_RUN(read_points(pts));
  SK_RUN(triangulate(std::move(pts), 0.0, 0.25));
  double at = 0.27;
  const std::size_t C = triang.num_cells(), V = triang.number_of_vertices();
  stage("voronoi", at, at + 0.04, C + V);
  SK_RUN(initialize(triang));
  SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
  at += 0.04;

  // ---- 2. Tight Cocone ----
  stage("tight cocone", at, at + 0.30, 16 * C);
  SK_RUN(tcocone(P.cocone_phi, P.sharp_phi, P.flat_phi, P.flatness_ratio, triang));
  at += 0.30;

  // ---- 3. the inner medial axis ----
  stage("medial axis", at, at + 0.10, 8 * C);
  SK_RUN(secondary_detail::compute_inner_medial_axis(triang, P.medial_theta, P.medial_ratio));
  at += 0.10;

  // ---- 4. U1, U2 and the skeleton graph ----
  stage("u1", at, at + 0.06, 4 * C);
  SK_RUN(compute_u1(triang, diag));
  at += 0.06;
  stage("skeleton", at, at + 0.06, 2 * C);
  SK_RUN(secondary_detail::add_u1(triang, sk));
  at += 0.06;
  stage("u2", at, at + 0.06, 3 * C);
  SK_RUN(compute_u2(triang, graph, diag));
  SK_RUN(secondary_detail::add_u2(graph, sk));
  SK_RUN(secondary_detail::component_areas(sk));
  at += 0.06;
  graph = Graph();
  triang.reset(tet_complex{}); // the skeleton holds everything later stages read
  result.skeleton = std::move(sk.s);
  sk = secondary_detail::skeleton_builder();

  // ---- 5, 6. the sheets, the helices and the output ----
  const double sel_lo = at;
  SK_RUN(secondary_detail::select(*ctx, P, result, [&](const char *name, std::size_t units) {
    const double lo = std::string(name) == "sheets"    ? sel_lo
                      : std::string(name) == "helices" ? sel_lo + 0.3 * (1.0 - sel_lo)
                                                       : sel_lo + 0.8 * (1.0 - sel_lo);
    const double hi = std::string(name) == "sheets"    ? sel_lo + 0.3 * (1.0 - sel_lo)
                      : std::string(name) == "helices" ? sel_lo + 0.8 * (1.0 - sel_lo)
                                                       : 1.0;
    stage(name, lo, hi, units);
  }));

  // Warnings: what the legacy code wrote to files or the console, and what it did silently.
  auto warn = [&](std::size_t n, const char *what) {
    if (n)
      result.warnings.push_back(fmt_count(what, n));
  };
  warn(duplicates, "%zu duplicate points were merged");
  warn(diag.dirty_cells, "%zu cells' circumcentres or radii were not finite (approximated by a "
                         "facet's circumcentre)");
  warn(diag.degenerate_VF, "%zu degenerate Voronoi facets were skipped by U2");
  warn(diag.no_target_VE, "%zu U2 flows found no Voronoi edge to cross");
  warn(diag.incorrect_intersection, "%zu U2 flows met a Voronoi edge in no single point");
  warn(diag.same_p, "%zu U2 flows did not advance (a potential loop)");
  warn(diag.degenerate_polygon, "%zu Voronoi facets tested by U1 had all their Voronoi vertices "
                                "on one line or point (cospherical cells)");
  warn(diag.bad_chain, "%zu U2 chains stopped early (degenerate flow)");
}

secondary_stepper::secondary_stepper(app &ctx, const geometry &surface, secondary_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->ctx = &ctx;
  _impl->source.emplace(surface); // shares the arrays; the points are read in the first step
  _impl->params = params;
}

secondary_stepper::~secondary_stepper() = default;
secondary_stepper::secondary_stepper(secondary_stepper &&) noexcept = default;
secondary_stepper &secondary_stepper::operator=(secondary_stepper &&) noexcept = default;

bool secondary_stepper::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.failed) {
    if (m.cancel.load(std::memory_order_acquire))
      return true;
    std::rethrow_exception(m.error);
  }
  if (m.finished)
    return true;
  try {
    if (!m.task)
      m.task = m.run();
    for (std::size_t n = 0; budget == 0 || n < budget; ++n) {
      if (m.cancel.load(std::memory_order_acquire))
        break;
      if (m.task.resume()) {
        m.finished = true;
        m.phase_name = "done";
        m.prog = 1.0;
        m.task = cocone::step_task();
        return true;
      }
      if (++m.stage_units % 64 == 0)
        m.note_progress(double(m.stage_units) / double(m.stage_estimate));
    }
    if (m.cancel.load(std::memory_order_acquire)) {
      m.error = std::make_exception_ptr(secondary_error("find_secondary_structures: cancelled"));
      m.failed = true;
      m.phase_name = "failed";
      m.task = cocone::step_task(); // frees every frame, the triangulation and the partial result
      m.triang.reset(tet_complex{});
      return true;
    }
    return false;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    m.phase_name = "failed";
    m.task = cocone::step_task();
    throw;
  }
}

bool secondary_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *secondary_stepper::phase() const { return _impl ? _impl->phase_name : "done"; }
double secondary_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string secondary_stepper::status() const {
  if (!_impl)
    return "done";
  if (_impl->failed) {
    try {
      std::rethrow_exception(_impl->error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (_impl->finished) {
    char buf[160];
    std::snprintf(buf, sizeof buf, "done: %zu helices, %zu sheets", _impl->result.helices.size(),
                  _impl->result.sheets.size());
    return buf;
  }
  char buf[96];
  std::snprintf(buf, sizeof buf, "%s %d%%", _impl->phase_name, int(_impl->prog * 100));
  return buf;
}

void secondary_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

secondary_structures secondary_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished)
    throw secondary_error("find_secondary_structures: take() before the stepper is done");
  secondary_structures r = std::move(_impl->result);
  _impl.reset();
  return r;
}

secondary_structures find_secondary_structures(app &ctx, const geometry &surface,
                                               const secondary_params &params) {
  secondary_stepper s(ctx, surface, params);
  s.step(0);
  return s.take();
}

secondary_structures select_secondary_structures(app &ctx, secondary_skeleton skeleton,
                                                 const secondary_params &params) {
  check_params("select_secondary_structures", params);
  secondary_detail::check_skeleton(skeleton);
  secondary_structures r;
  r.skeleton = std::move(skeleton);
  cocone::step_task t = secondary_detail::select(ctx, params, r);
  while (!t.resume()) {
  }
  return r;
}

std::unique_ptr<result_job<secondary_structures>>
find_secondary_structures_job(app &ctx, const geometry &surface, secondary_params params) {
  return make_result_job(std::make_unique<secondary_stepper>(ctx, surface, params));
}

} // namespace cvc
