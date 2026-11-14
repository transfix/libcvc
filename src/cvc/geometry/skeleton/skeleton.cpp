/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (the legacy skeletonize())
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

// cvc::skeletonize and skeleton_stepper (cvc/geometry/skeleton.h): Skeletonization.cpp's
// skeletonize() and build_simple_skel() as one coroutine over the cocone core (cocone.h), with
// the call's own context (skeleton_context) in place of the legacy globals.

#include "cocone.h"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_params.h>
#include <map>
#include <optional>

namespace cvc {

namespace {

using cocone::Point;

constexpr double BB_SCALE = 1.0; // the legacy box: the points' box, grown by its span each way
constexpr std::size_t BUILD_UNITS = 16; // tet_complex_builder units per skeleton unit

// A sheet's colour: a fixed palette by sheet rank (the legacy code drew rand() colours).
geometry::color_t sheet_colour(int rank) {
  static constexpr double palette[8][3] = {
      {0.90, 0.60, 0.00}, {0.35, 0.70, 0.90}, {0.00, 0.60, 0.50}, {0.95, 0.90, 0.25},
      {0.00, 0.45, 0.70}, {0.80, 0.40, 0.00}, {0.80, 0.60, 0.70}, {0.60, 0.60, 0.60}};
  const double *c = palette[rank % 8];
  return geometry::color_t{{c[0], c[1], c[2]}};
}

std::string fmt_count(const char *what, std::size_t n) {
  char buf[256];
  std::snprintf(buf, sizeof buf, what, n);
  return buf;
}

// Points merged by their coordinates into a geometry's points.
class point_index {
public:
  explicit point_index(geometry &g) : _g(g) {}
  geometry::index_t operator()(const Point &p, const geometry::color_t &colour) {
    auto [it, fresh] = _ids.try_emplace(p, geometry::index_t(_g.points().size()));
    if (fresh) {
      _g.points().push_back(geometry::point_t{{p.x(), p.y(), p.z()}});
      _g.colors().push_back(colour);
    }
    return it->second;
  }
  void clear() { _ids.clear(); }

private:
  geometry &_g;
  std::map<Point, geometry::index_t> _ids;
};

} // namespace

// The per-call context of §1.3: everything the legacy code kept in globals or locals of
// skeletonize(), and the coroutine that runs it.
struct skeleton_stepper::impl {
  std::vector<std::array<double, 3>> points;
  std::optional<geometry> source; // the geometry to read the points from, until they are read
  skeleton_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;
  const char *phase_name = "checking";
  double prog = 0.0;

  // the context
  std::optional<tet_complex_builder> builder;
  cocone::Triangulation triang;
  cocone::diagnostics diag;
  std::vector<double> bounding_box;
  std::vector<cocone::Point> robust_points;
  cocone::Skel skel;
  cocone::Graph graph;
  std::size_t duplicates = 0, robust_dropped = 0;
  skeleton result;

  // progress: the current stage's span of [0, 1] and its units
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
  cocone::step_task triangulate(std::vector<std::array<double, 3>> pts, double lo, double hi);
  cocone::step_task output();
};

// Build a tet_complex of pts into triang, BUILD_UNITS of the builder's units per unit, its
// phases reported as ours.
cocone::step_task skeleton_stepper::impl::triangulate(std::vector<std::array<double, 3>> pts,
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
      throw skeleton_error("skeletonize: " + what);
    }
    phase_name = builder->phase();
    prog = std::max(prog, lo + (hi - lo) * builder->progress());
    if (ready)
      break;
    SK_UNIT;
  }
  tet_complex t = builder->take();
  builder.reset();
  duplicates += std::size_t(t.num_points() - t.num_vertices());
  triang.reset(std::move(t));
  stage("building", hi, hi + 0.02, triang.fill_total());
  while (!triang.fill_one())
    SK_UNIT;
}

cocone::step_task skeleton_stepper::impl::run() {
  using namespace cvc::cocone;
  // ---- the input and the parameters ----
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "skeletonize: invalid parameters: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw skeleton_error(m);
    }
  }
  if (!tet_complex_available())
    throw skeleton_error("skeletonize: libcvc was built without Shewchuk's predicates "
                         "(CVC_WITH_SHEWCHUK), which the triangulation needs");
  if (source) {
    const geometry::points_t &gp = source->const_points();
    points.reserve(gp.size());
    for (std::size_t i = 0; i < gp.size(); ++i) {
      points.push_back({gp[i][0], gp[i][1], gp[i][2]});
      if (i % 1024 == 1023)
        SK_UNIT;
    }
    source.reset();
  }

  const skeleton_params &P = params;
  const double tri_span = P.robust ? 0.15 : 0.25;
  double at = 0.0;

  // ---- 1. Robust Cocone (optional): the points on a smooth surface ----
  std::vector<std::array<double, 3>> tri_points;
  if (P.robust) {
    SK_RUN(triangulate(std::move(points), 0.0, tri_span));
    at = tri_span + 0.02;
    stage("voronoi", at, at + 0.03, triang.num_cells() + triang.number_of_vertices());
    SK_RUN(initialize(triang));
    SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
    at += 0.03;
    stage("robust cocone", at, at + 0.05, 6 * triang.num_cells());
    SK_RUN(robust_cocone(P.big_ball_ratio, P.theta_ff, P.theta_if, triang, robust_points));
    at += 0.05;
    robust_dropped = triang.number_of_vertices() - robust_points.size();
    tri_points.reserve(robust_points.size());
    for (std::size_t i = 0; i < robust_points.size(); ++i) {
      tri_points.push_back({robust_points[i].x(), robust_points[i].y(), robust_points[i].z()});
      if (i % 1024 == 1023)
        SK_UNIT;
    }
    if (tri_points.size() < 4)
      throw skeleton_error(fmt_count("skeletonize: Robust Cocone kept %zu points, fewer than the "
                                     "four a triangulation needs; try robust = false or a larger "
                                     "big_ball_ratio",
                                     tri_points.size()));
  } else {
    tri_points = std::move(points);
  }

  // ---- 2. the Delaunay triangulation, and the legacy box around its points ----
  {
    double lo3[3] = {DBL_MAX, DBL_MAX, DBL_MAX}, hi3[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    for (std::size_t i = 0; i < tri_points.size(); ++i) {
      for (int k = 0; k < 3; ++k) {
        lo3[k] = std::min(lo3[k], tri_points[i][std::size_t(k)]);
        hi3[k] = std::max(hi3[k], tri_points[i][std::size_t(k)]);
      }
      if (i % 1024 == 1023)
        SK_UNIT;
    }
    bounding_box.clear();
    for (int k = 0; k < 3; ++k) {
      bounding_box.push_back(lo3[k] - BB_SCALE * (hi3[k] - lo3[k]));
      bounding_box.push_back(hi3[k] + BB_SCALE * (hi3[k] - lo3[k]));
    }
  }
  SK_RUN(triangulate(std::move(tri_points), at, at + tri_span));
  at += tri_span + 0.02;
  const std::size_t C = triang.num_cells(), V = triang.number_of_vertices();

  // ---- the attributes, the Voronoi vertices and the cell radii ----
  stage("voronoi", at, at + 0.04, C + V);
  SK_RUN(initialize(triang));
  SK_RUN(compute_voronoi_vertex_and_cell_radius(triang, diag));
  at += 0.04;

  // ---- 3. Tight Cocone ----
  stage("tight cocone", at, at + 0.30, 16 * C);
  SK_RUN(tcocone(P.cocone_phi, P.sharp_phi, P.flat_phi, P.flatness_ratio, triang));
  at += 0.30;

  // ---- 4. the medial axis ----
  stage("medial axis", at, at + 0.08, 4 * C);
  SK_RUN(compute_medial_axis(triang, P.medial_theta, P.medial_ratio));
  at += 0.08;

  // ---- 5. U1 and its planar clusters: the sheets ----
  stage("sheets", at, at + 0.08, 4 * C);
  SK_RUN(compute_u1(triang, diag));
  SK_RUN(cluster_planar_patches(triang, skel));
  at += 0.08;

  // ---- 6. U2, its graph and its polylines: the strands ----
  stage("strands", at, at + 0.08, 3 * C);
  SK_RUN(compute_u2(triang, graph, diag));
  SK_RUN(port_graph_to_skel(graph, skel));
  at += 0.08;

  // ---- 7. clean-up: stars, the clusters by area, the small ones ----
  stage("clean-up", at, at + 0.03, C + skel.pl.size());
  SK_RUN(star(triang, graph, skel));
  SK_RUN(sort_cluster_by_area(skel));
  SK_RUN(filter_small_clusters(skel, P.sheet_count, P.threshold, P.discard_small));
  at += 0.03;

  stage("output", at, 1.0, skel.pl.size() + skel.L.size() + 1);
  SK_RUN(output());
}

// build_simple_skel: the clusters and polylines as geometry.
cocone::step_task skeleton_stepper::impl::output() {
  using namespace cvc::cocone;
  skeleton &r = result;
  r.clusters = int(skel.pl.size());
  const geometry::color_t blue{{0.0, 0.0, 1.0}}, green{{0.0, 1.0, 0.0}}, red{{1.0, 0.0, 0.0}};

  // The sheets, and the stars of the clusters that are not sheets.
  point_index sheet_ids(r.sheets), strand_ids(r.strands);
  struct star_seg {
    int cluster;
    Point a, b;
  };
  std::vector<star_seg> stars;
  int sheet_rank = 0;
  for (int i = 0; i < (int)skel.sorted_pl_id.size(); i++) {
    SK_UNIT;
    const int cl_id = skel.sorted_pl_id[std::size_t(i)];
    if (!skel.is_big_pl[std::size_t(cl_id)]) {
      const Point c = skel.pl_C[std::size_t(cl_id)];
      for (const Point &b : skel.active_bdy[std::size_t(cl_id)])
        stars.push_back({cl_id, c, b});
      continue;
    }
    const geometry::color_t colour = sheet_colour(sheet_rank);
    sheet_ids.clear(); // points are merged within a sheet
    for (const Polygon &pg : skel.pl[std::size_t(cl_id)]) {
      const vector<Point> &v = pg.ordered_v_list;
      for (std::size_t k = 1; k + 1 < v.size(); ++k) {
        if (v[0] == v[k] || v[k] == v[k + 1] || v[0] == v[k + 1])
          continue; // zero area: coincident Voronoi vertices
        r.sheets.tris().push_back(geometry::tri_t{
            {sheet_ids(v[0], colour), sheet_ids(v[k], colour), sheet_ids(v[k + 1], colour)}});
        r.sheet_of_tri.push_back(sheet_rank);
      }
    }
    ++sheet_rank;
  }
  r.sheet_clusters = sheet_rank;

  // The polylines inside the box, as segments.
  int polyline = 0;
  for (int i = 0; i < (int)skel.L.size(); i++) {
    SK_UNIT;
    const Polyline &l = skel.L[std::size_t(i)];
    if (skel.L_invalid[std::size_t(i)])
      continue;
    bool is_far = false;
    for (const Point &p : l.ordered_v_list)
      if (is_outside_bounding_box(p, bounding_box))
        is_far = true;
    if (is_far)
      continue;
    for (std::size_t j = 0; j + 1 < l.ordered_v_list.size(); j++) {
      r.strands.lines().push_back(geometry::line_t{
          {strand_ids(l.ordered_v_list[j], green), strand_ids(l.ordered_v_list[j + 1], green)}});
      r.strand_of_line.push_back(polyline);
    }
    ++polyline;
  }
  r.polylines = polyline;
  for (const star_seg &s : stars) {
    r.strands.lines().push_back(geometry::line_t{{strand_ids(s.a, blue), strand_ids(s.b, blue)}});
    r.strand_of_line.push_back(-1 - s.cluster);
  }

  // The legacy protein-helix strips: per polyline, its VVs outside the sheets with cell radius in
  // (2, 3).
  if (params.helix_window) {
    point_index helix_ids(r.helix);
    for (int i = 0; i < (int)skel.L.size(); i++) {
      SK_UNIT;
      if (skel.L_invalid[std::size_t(i)])
        continue;
      std::optional<geometry::index_t> prev;
      for (std::size_t j = 0; j < skel.L[std::size_t(i)].cell_list.size(); j++) {
        Cell_handle c = skel.L[std::size_t(i)].cell_list[j];
        if (c->big_pl())
          continue;
        if (std::sqrt(c->cell_radius()) >= 2.5 + 0.5 || std::sqrt(c->cell_radius()) <= 2.5 - 0.5)
          continue;
        const geometry::index_t id =
            helix_ids(skel.L[std::size_t(i)].ordered_v_list[j], is_maxima(c) ? red : green);
        if (prev)
          r.helix.lines().push_back(geometry::line_t{{*prev, id}});
        prev = id;
      }
    }
  }

  // Warnings: what the legacy code wrote to files or the console.
  auto warn = [&](std::size_t n, const char *what) {
    if (n)
      r.warnings.push_back(fmt_count(what, n));
  };
  warn(duplicates, "%zu duplicate points were merged");
  warn(robust_dropped, "Robust Cocone left out %zu points");
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

skeleton_stepper::skeleton_stepper(std::vector<std::array<double, 3>> points,
                                   skeleton_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->points = std::move(points);
  _impl->params = params;
}

skeleton_stepper::skeleton_stepper(const geometry &points_or_mesh, skeleton_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->source.emplace(points_or_mesh); // shares the arrays; the points are read in the first step
  _impl->params = params;
}

skeleton_stepper::~skeleton_stepper() = default;
skeleton_stepper::skeleton_stepper(skeleton_stepper &&) noexcept = default;
skeleton_stepper &skeleton_stepper::operator=(skeleton_stepper &&) noexcept = default;

bool skeleton_stepper::step(std::size_t budget) {
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
      m.error = std::make_exception_ptr(skeleton_error("skeletonize: cancelled"));
      m.failed = true;
      m.phase_name = "failed";
      m.task = cocone::step_task(); // frees every frame, the triangulation and the partial skeleton
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

bool skeleton_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *skeleton_stepper::phase() const { return _impl ? _impl->phase_name : "done"; }
double skeleton_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string skeleton_stepper::status() const {
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
    std::snprintf(buf, sizeof buf, "done: %d strands, %d sheets (%d clusters)",
                  _impl->result.polylines, _impl->result.sheet_clusters, _impl->result.clusters);
    return buf;
  }
  char buf[96];
  std::snprintf(buf, sizeof buf, "%s %d%%", _impl->phase_name, int(_impl->prog * 100));
  return buf;
}

void skeleton_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

skeleton skeleton_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished)
    throw skeleton_error("skeletonize: take() before the stepper is done");
  skeleton r = std::move(_impl->result);
  _impl.reset();
  return r;
}

skeleton skeletonize(const std::vector<std::array<double, 3>> &points,
                     const skeleton_params &params) {
  skeleton_stepper s(points, params);
  s.step(0);
  return s.take();
}

skeleton skeletonize(const geometry &points_or_mesh, const skeleton_params &params) {
  skeleton_stepper s(points_or_mesh, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<skeleton_stepper> skeletonize_stepper(std::vector<std::array<double, 3>> points,
                                                      skeleton_params params) {
  return std::make_unique<skeleton_stepper>(std::move(points), params);
}

std::unique_ptr<result_job<skeleton>> skeletonize_job(std::vector<std::array<double, 3>> points,
                                                      skeleton_params params) {
  return make_result_job(std::make_unique<skeleton_stepper>(std::move(points), params));
}

std::unique_ptr<result_job<skeleton>> skeletonize_job(const geometry &points_or_mesh,
                                                      skeleton_params params) {
  return make_result_job(std::make_unique<skeleton_stepper>(points_or_mesh, params));
}

} // namespace cvc
