/*
  Copyright 2007-2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (TightCocone)
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

// cvc::tight_cocone and tight_cocone_stepper (cvc/geometry/tight_cocone.h): TightCocone's
// surfaceReconstruction(const cvcgeom_t &, const Parameters &) (tight_cocone.cpp:567-722) as one
// coroutine over the shared cocone core: tight_cocone_front, then extract_surface
// (../cocone/tight_cocone_stages.h), with the call's own context in place of the legacy globals.

#include "../cocone/tight_cocone_stages.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cvc/geometry/tight_cocone.h>
#include <cvc/geometry/tight_cocone_params.h>
#include <optional>

namespace cvc {

namespace {

std::string fmt_count(const char *what, std::size_t n) {
  char buf[256];
  std::snprintf(buf, sizeof buf, what, n);
  return buf;
}

} // namespace

// The per-call context of §1.3, and the coroutine that runs it.
struct tight_cocone_stepper::impl : cocone::stage_progress {
  std::vector<std::array<double, 3>> points;
  std::optional<geometry> source; // the geometry to read the points from, until they are read
  tight_cocone_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;

  // the context
  cocone::Triangulation triang;
  cocone::diagnostics diag;
  cocone::front_counts front;
  cocone::surface_counts surf;
  tight_cocone_result result;

  cocone::step_task task;

  cocone::step_task run();
};

cocone::step_task tight_cocone_stepper::impl::run() {
  using namespace cvc::cocone;
  // ---- the input and the parameters ----
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "tight_cocone: invalid parameters: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw tight_cocone_error(m);
    }
  }
  if (!tet_complex_available())
    throw tight_cocone_error("tight_cocone: libcvc was built without Shewchuk's predicates "
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
  result.points = points.size();

  // ---- 1-3. Robust Cocone (optional), the triangulation and Tight Cocone's marking ----
  SK_RUN(tight_cocone_front<tight_cocone_error>(std::exchange(points, {}), params, "tight_cocone",
                                                triang, diag, *this, 0.0, 0.90, front));

  // ---- 4. the surface ----
  stage("surface", 0.90, 1.0, 2 * triang.num_cells() + triang.complex().points().size());
  SK_RUN(extract_surface(triang, front.input_of_point, result.surface, result.point_index, surf));

  tight_cocone_result &r = result;
  r.duplicates = front.duplicates;
  r.robust_dropped = front.robust_dropped;
  r.cells = triang.num_finite_cells();
  r.inside_cells = surf.inside_cells;
  triang.reset(tet_complex{}); // the result holds everything

  // Warnings: what the legacy code wrote to the console or did silently.
  auto warn = [&](std::size_t n, const char *what) {
    if (n)
      r.warnings.push_back(fmt_count(what, n));
  };
  warn(r.duplicates, "%zu duplicate points were merged");
  warn(r.robust_dropped, "Robust Cocone left out %zu points");
  warn(diag.dirty_cells, "%zu cells' circumcentres or radii were not finite (approximated by a "
                         "facet's circumcentre)");
  warn(surf.one_sided, "%zu surface facets have the same side of the surface on both sides");
  const std::size_t distinct = r.points - r.duplicates - r.robust_dropped;
  if (r.surface.num_tris() == 0)
    r.warnings.push_back("Tight Cocone found no surface");
  else
    warn(distinct - std::min(distinct, r.point_index.size()),
         "%zu points are on no surface triangle");
}

tight_cocone_stepper::tight_cocone_stepper(std::vector<std::array<double, 3>> points,
                                           tight_cocone_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->points = std::move(points);
  _impl->params = params;
}

tight_cocone_stepper::tight_cocone_stepper(const geometry &points_or_mesh,
                                           tight_cocone_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->source.emplace(points_or_mesh); // shares the arrays; the points are read in the first step
  _impl->params = params;
}

tight_cocone_stepper::~tight_cocone_stepper() = default;
tight_cocone_stepper::tight_cocone_stepper(tight_cocone_stepper &&) noexcept = default;
tight_cocone_stepper &tight_cocone_stepper::operator=(tight_cocone_stepper &&) noexcept = default;

bool tight_cocone_stepper::step(std::size_t budget) {
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
      m.error = std::make_exception_ptr(tight_cocone_error("tight_cocone: cancelled"));
      m.failed = true;
      m.phase_name = "failed";
      m.task = cocone::step_task(); // frees every frame, the triangulation and the partial surface
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

bool tight_cocone_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *tight_cocone_stepper::phase() const { return _impl ? _impl->phase_name : "done"; }
double tight_cocone_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string tight_cocone_stepper::status() const {
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
    std::snprintf(buf, sizeof buf, "done: %zu triangles over %zu points",
                  _impl->result.surface.num_tris(), _impl->result.point_index.size());
    return buf;
  }
  char buf[96];
  std::snprintf(buf, sizeof buf, "%s %d%%", _impl->phase_name, int(_impl->prog * 100));
  return buf;
}

void tight_cocone_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

tight_cocone_result tight_cocone_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished)
    throw tight_cocone_error("tight_cocone: take() before the stepper is done");
  tight_cocone_result r = std::move(_impl->result);
  _impl.reset();
  return r;
}

tight_cocone_result tight_cocone(const std::vector<std::array<double, 3>> &points,
                                 const tight_cocone_params &params) {
  tight_cocone_stepper s(points, params);
  s.step(0);
  return s.take();
}

tight_cocone_result tight_cocone(const geometry &points_or_mesh,
                                 const tight_cocone_params &params) {
  tight_cocone_stepper s(points_or_mesh, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<tight_cocone_result>>
tight_cocone_job(std::vector<std::array<double, 3>> points, tight_cocone_params params) {
  return make_result_job(std::make_unique<tight_cocone_stepper>(std::move(points), params));
}

std::unique_ptr<result_job<tight_cocone_result>> tight_cocone_job(const geometry &points_or_mesh,
                                                                  tight_cocone_params params) {
  return make_result_job(std::make_unique<tight_cocone_stepper>(points_or_mesh, params));
}

} // namespace cvc
