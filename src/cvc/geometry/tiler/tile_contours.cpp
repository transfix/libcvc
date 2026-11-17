/*
  Copyright 2026 The University of Texas at Austin

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

// cvc::tile_contours and tile_stepper (inc/cvc/geometry/tiler.h): the stack in, the vendored
// tiler's slices built from it (every vertex snapped onto the stack's grid), tiler::tile() run a
// unit at a time, and its triangles out as one cvc::geometry per component. Slice index k is
// the tiler's z = k, as the legacy reader set it; world z comes back at the end.

#include <cvc/geometry/tiler.h>

#ifdef CVC_WITH_PLANAR
#include <cvc/geometry/planar.h>

#include "Contour.h"
#include "Slice.h"
#include "tiler.h"
#endif

#include <boost/regex.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <exception>
#include <map>
#include <sstream>

namespace cvc {

std::vector<std::string> tile_params_problems(const tile_params &p) {
  std::vector<std::string> problems;
  if (p.z_first >= 0 && p.z_last >= 0 && p.z_first > p.z_last)
    problems.push_back("z_first = " + std::to_string(p.z_first) + " is after z_last = " +
                       std::to_string(p.z_last));
  for (const char *field : {"include", "exclude"}) {
    const std::vector<std::string> &patterns = std::string(field) == "include" ? p.include : p.exclude;
    for (std::size_t i = 0; i < patterns.size(); ++i) {
      try {
        boost::regex r(patterns[i], boost::regex::perl);
        (void)r;
      } catch (const boost::regex_error &e) {
        problems.push_back(std::string(field) + "[" + std::to_string(i) + "] = '" + patterns[i] +
                           "' is not a regular expression: " + e.what());
      }
    }
  }
  if (!p.color.empty() && p.color.size() != 3)
    problems.push_back("color: " + std::to_string(p.color.size()) +
                       " values, not three (r, g, b)");
  for (double c : p.color)
    if (!(c >= 0 && c <= 1))
      problems.push_back("color: each value must be in [0, 1]");
  if (!(p.collinear_epsilon >= 0) || !std::isfinite(p.collinear_epsilon))
    problems.push_back("collinear_epsilon must be finite and not negative");
  if (!(p.curation_delta >= 0) || !std::isfinite(p.curation_delta))
    problems.push_back("curation_delta must be finite and not negative");
  if (!std::isfinite(p.correspondence_overlap))
    problems.push_back("correspondence_overlap must be finite");
  if (p.remove_intersections && !(p.curation_delta > 0))
    problems.push_back("remove_intersections needs curation_delta > 0");
  return problems;
}

#ifdef CVC_WITH_PLANAR

namespace {

using tiler::Contour;
using tiler::Contour_handle;
using tiler::Number_type;
using tiler::Point_2;
using tiler::Point_3;
using tiler::Polygon_2;
using tiler::Slice;
using tiler::step_task;
using tiler::Triangle;

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? sep : "") + v[i];
  return out;
}

std::vector<boost::regex> compile(const std::vector<std::string> &patterns) {
  std::vector<boost::regex> out;
  for (const std::string &s : patterns)
    out.emplace_back(s, boost::regex::perl);
  return out;
}

bool any_match(const std::vector<boost::regex> &res, const std::string &name) {
  for (const boost::regex &r : res)
    if (boost::regex_match(name, r))
      return true;
  return false;
}

} // namespace

struct tile_stepper::impl {
  contour_stack stack;
  tile_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;

  tiler::tiler_context ctx;
  std::vector<Slice> slices; // slices[i] is slice index first + i
  int first = 0, last = -1;
  std::map<int, double> slice_z; // the world z of each index tiled
  tiler::Tiler_options options;
  tiler::tile_output out;
  tile_result result;
  std::size_t vertices = 0, triangles = 0;

  step_task task;

  step_task run();
  step_task build_slices();
  step_task assemble();
  // The world z of tiler z t: between the planes of floor(t) and floor(t) + 1.
  double world_z(double t) const;
};

double tile_stepper::impl::world_z(double t) const {
  const double k0 = std::floor(t);
  auto lo = slice_z.find(int(k0));
  if (lo == slice_z.end())
    return t; // not reached: every index tiled has a plane
  if (t == k0)
    return lo->second;
  auto hi = slice_z.find(int(k0) + 1);
  if (hi == slice_z.end())
    return lo->second;
  return lo->second + (t - k0) * (hi->second - lo->second);
}

// The stack's slices in the range, as the vendored tiler's Slices: each contour's vertices snapped
// onto the stack's grid, made counterclockwise, and kept if they make a simple polygon. A unit per
// vertex.
step_task tile_stepper::impl::build_slices() {
  ctx.stage("snapping", 0.0, 0.05);
  tiler::Logger logger = ctx.logger("tiler.tile_contours");

  const std::vector<boost::regex> inc = compile(params.include), exc = compile(params.exclude);
  auto keep = [&](const std::string &name) {
    return (inc.empty() || any_match(inc, name)) && !any_match(exc, name);
  };

  // The grid of everything tiled.
  double xmin = HUGE_VAL, ymin = HUGE_VAL, xmax = -HUGE_VAL, ymax = -HUGE_VAL;
  std::size_t total = 0;
  for (int k = first; k <= last; ++k) {
    auto s = stack.slices.find(k);
    if (s == stack.slices.end())
      continue;
    for (const auto &comp : s->second.components) {
      if (!keep(comp.first))
        continue;
      for (const contour_polygon &poly : comp.second) {
        for (std::size_t i = 0; i < poly.size(); ++i) {
          xmin = std::min(xmin, poly.xy[2 * i]);
          xmax = std::max(xmax, poly.xy[2 * i]);
          ymin = std::min(ymin, poly.xy[2 * i + 1]);
          ymax = std::max(ymax, poly.xy[2 * i + 1]);
        }
        total += poly.size();
      }
    }
  }
  if (total == 0)
    co_return;
  const planar::grid g = planar::grid::fit(xmin, ymin, xmax, ymax);

  std::size_t done = 0;
  for (int k = first; k <= last; ++k) {
    Slice &slice = slices[std::size_t(k - first)];
    auto s = stack.slices.find(k);
    if (s == stack.slices.end())
      continue;
    for (const auto &comp : s->second.components) {
      const std::string &name = comp.first;
      if (!keep(name))
        continue;
      for (const contour_polygon &poly : comp.second) {
        Polygon_2 P;
        for (std::size_t i = 0; i < poly.size(); ++i) {
          const planar::pointd w = g.to_world(g.snap(poly.xy[2 * i], poly.xy[2 * i + 1]));
          const Point_2 p(w.x, w.y, Number_type(k));
          if (P.size() == 0 || !(P[P.size() - 1] == p))
            P.push_back(p);
          ++done;
          if (done % 64 == 0)
            ctx.advance(double(done) / double(total));
          SK_UNIT;
        }
        while (P.size() > 1 && P[0] == P[P.size() - 1])
          P.erase(P.vertices_end() - 1);
        if (P.size() < 3) {
          TILER_WARN(logger, "Skipping a contour of fewer than 3 distinct vertices: slice = "
                                 << k << " component = " << name);
          continue;
        }
        if (!P.is_simple()) {
          TILER_WARN(logger, "Skipping non-simple contour: slice = " << k << " component = "
                                                                     << name);
          continue;
        }
        if (P.is_clockwise_oriented())
          P.reverse_orientation();
        const Contour::Info info(Number_type(k), name, name);
        slice.push_back(name, Contour::create(P, info));
        vertices += P.size();
      }
    }
  }
}

// The tiler's triangles as one geometry per component: vertices merged by position, normals the
// area-weighted mean of their triangles', the component's colour. A unit per triangle.
step_task tile_stepper::impl::assemble() {
  ctx.stage("assembling", 0.95, 1.0);
  std::size_t total = 0, done = 0;
  for (const auto &entry : out.triangles)
    total += entry.second.size();

  for (auto &entry : out.triangles) {
    const std::string &name = entry.first;
    std::vector<double> color = params.color;
    if (color.size() != 3) {
      const int ci = stack.component_index(name);
      color = ci >= 0 ? stack.components[std::size_t(ci)].color : std::vector<double>{1, 1, 1};
      if (color.size() != 3)
        color = {1, 1, 1};
    }
    geometry geom;
    geometry::points_t &pts = geom.points();
    geometry::tris_t &tris = geom.tris();
    std::map<std::array<double, 3>, geometry::index_t> index;
    for (const Triangle &t : entry.second) {
      geometry::tri_t tri;
      for (int i = 0; i < 3; ++i) {
        const Point_3 &p = t[i];
        const std::array<double, 3> key{p.x(), p.y(), world_z(p.z())};
        auto it = index.find(key);
        if (it == index.end()) {
          it = index.emplace(key, geometry::index_t(pts.size())).first;
          pts.push_back(geometry::point_t{{key[0], key[1], key[2]}});
        }
        tri[i] = it->second;
      }
      tris.push_back(tri);
      ++triangles;
      if (++done % 64 == 0 && total > 0)
        ctx.advance(double(done) / double(total));
      SK_UNIT;
    }
    // Normals and colours.
    geometry::normals_t &normals = geom.normals();
    normals.assign(pts.size(), geometry::normal_t{{0, 0, 0}});
    for (const geometry::tri_t &tri : tris) {
      const auto &a = pts[tri[0]], &b = pts[tri[1]], &c = pts[tri[2]];
      const double ux = b[0] - a[0], uy = b[1] - a[1], uz = b[2] - a[2];
      const double vx = c[0] - a[0], vy = c[1] - a[1], vz = c[2] - a[2];
      const double n[3] = {uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx};
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          normals[tri[i]][j] += n[j];
    }
    for (auto &n : normals) {
      const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
      if (len > 0)
        for (int j = 0; j < 3; ++j)
          n[j] /= len;
    }
    geom.colors().assign(pts.size(), geometry::color_t{{color[0], color[1], color[2]}});
    result.components.emplace(name, std::move(geom));
  }
}

step_task tile_stepper::impl::run() {
  ctx.stage("snapping", 0.0, 0.05);
  const std::vector<std::string> pproblems = tile_params_problems(params);
  if (!pproblems.empty())
    throw tiler_error("tile_contours: " + join(pproblems, "; "));
  const std::vector<std::string> sproblems = contour_problems(stack);
  if (!sproblems.empty())
    throw tiler_error("tile_contours: the stack: " + join(sproblems, "; "));

  if (stack.slices.empty())
    co_return;
  const int lo = params.z_first >= 0 ? params.z_first : stack.slices.begin()->first;
  const int hi = params.z_last >= 0 ? params.z_last : stack.slices.rbegin()->first;
  if (lo > hi)
    co_return;
  first = lo - (params.cap_ends ? 1 : 0);
  last = hi + (params.cap_ends ? 1 : 0);

  // Each index's plane: its own, or between (or beyond) the nearest slices' that the stack has.
  for (int k = first; k <= last; ++k) {
    if (stack.slices.count(k)) {
      slice_z[k] = stack.slice_z(k);
      continue;
    }
    auto above = stack.slices.lower_bound(k);
    auto below = above == stack.slices.begin() ? stack.slices.end() : std::prev(above);
    if (above != stack.slices.end() && below != stack.slices.end()) {
      const double za = stack.slice_z(above->first), zb = stack.slice_z(below->first);
      slice_z[k] = zb + (za - zb) * double(k - below->first) / double(above->first - below->first);
    } else if (below != stack.slices.end()) {
      slice_z[k] = stack.slice_z(below->first) + stack.z_spacing * double(k - below->first);
    } else {
      slice_z[k] = stack.slice_z(above->first) - stack.z_spacing * double(above->first - k);
    }
  }

  // curation_delta must stay below every spacing tiled (the legacy CLI checked it against -z).
  double spacing_sum = 0;
  for (int k = first; k < last; ++k) {
    const double dz = std::fabs(slice_z[k + 1] - slice_z[k]);
    spacing_sum += dz;
    if (!(params.curation_delta < dz))
      throw tiler_error("tile_contours: curation_delta = " + std::to_string(params.curation_delta) +
                        " is not below the spacing " + std::to_string(dz) + " of slices " +
                        std::to_string(k) + " and " + std::to_string(k + 1));
  }

  slices.assign(std::size_t(last - first + 1), Slice());
  SK_RUN(build_slices());

  options.collinear_epsilon() = params.collinear_epsilon;
  options.contour_curation_delta() = params.curation_delta;
  options.remove_intersections() = params.remove_intersections;
  options.correspondence_overlap() = params.correspondence_overlap;
  // The world spacing of one index, for intersection removal's z tolerance (the legacy -z): the
  // mean over the range, which is the spacing of an evenly spaced stack.
  options.z_scale() = last > first ? spacing_sum / double(last - first) : stack.z_spacing;

  if (vertices > 0)
    SK_RUN(tiler::tile(slices, options, ctx, out));
  slices.clear();
  SK_RUN(assemble());
  result.warnings = ctx.log.warnings();
}

tile_stepper::tile_stepper(contour_stack stack, tile_params params)
    : _impl(std::make_unique<impl>()) {
  _impl->stack = std::move(stack);
  _impl->params = std::move(params);
}

tile_stepper::~tile_stepper() = default;
tile_stepper::tile_stepper(tile_stepper &&) noexcept = default;
tile_stepper &tile_stepper::operator=(tile_stepper &&) noexcept = default;

bool tile_stepper::step(std::size_t budget) {
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
        m.ctx.phase = "done";
        m.ctx.progress = 1.0;
        m.task = step_task();
        return true;
      }
    }
    if (m.cancel.load(std::memory_order_acquire)) {
      m.error = std::make_exception_ptr(tiler_error("tile_contours: cancelled"));
      m.failed = true;
      m.ctx.phase = "failed";
      m.task = step_task(); // frees every frame, the slices and the partial surfaces
      m.slices.clear();
      m.out.triangles.clear();
      return true;
    }
    return false;
  } catch (const tiler_error &) {
    m.error = std::current_exception();
    m.failed = true;
    m.ctx.phase = "failed";
    m.task = step_task();
    throw;
  } catch (const std::exception &e) {
    // The vendored code throws std::logic_error and friends; they leave as tiler_error.
    m.error = std::make_exception_ptr(tiler_error(std::string("tile_contours: ") + e.what()));
    m.failed = true;
    m.ctx.phase = "failed";
    m.task = step_task();
    std::rethrow_exception(m.error);
  }
}

bool tile_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *tile_stepper::phase() const { return _impl ? _impl->ctx.phase : "done"; }
std::string tile_stepper::label() const { return _impl ? _impl->ctx.label : std::string(); }
double tile_stepper::progress() const { return _impl ? _impl->ctx.progress : 1.0; }

std::string tile_stepper::status() const {
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
    std::snprintf(buf, sizeof buf, "done: %zu surfaces, %zu triangles, %zu warnings",
                  _impl->result.components.size(), _impl->triangles,
                  _impl->result.warnings.size());
    return buf;
  }
  char buf[160];
  if (_impl->ctx.label.empty())
    std::snprintf(buf, sizeof buf, "%s %d%%", _impl->ctx.phase, int(_impl->ctx.progress * 100));
  else
    std::snprintf(buf, sizeof buf, "%s %s %d%%", _impl->ctx.phase, _impl->ctx.label.c_str(),
                  int(_impl->ctx.progress * 100));
  return buf;
}

void tile_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

tile_result tile_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished)
    throw tiler_error("tile_contours: take() before the stepper is done");
  tile_result r = std::move(_impl->result);
  _impl.reset();
  return r;
}

#else // CVC_WITH_PLANAR

struct tile_stepper::impl {};

tile_stepper::tile_stepper(contour_stack, tile_params) {}
tile_stepper::~tile_stepper() = default;
tile_stepper::tile_stepper(tile_stepper &&) noexcept = default;
tile_stepper &tile_stepper::operator=(tile_stepper &&) noexcept = default;

namespace {
[[noreturn]] void unavailable() {
  throw tiler_error("tile_contours: libcvc was built without cvc::planar (Clipper2, CDT and "
                    "Shewchuk's predicates)");
}
} // namespace

bool tile_stepper::step(std::size_t) { unavailable(); }
bool tile_stepper::done() const { return false; }
const char *tile_stepper::phase() const { return "failed"; }
std::string tile_stepper::label() const { return std::string(); }
double tile_stepper::progress() const { return 0.0; }
std::string tile_stepper::status() const { return "failed: built without cvc::planar"; }
void tile_stepper::cancel() {}
tile_result tile_stepper::take() { unavailable(); }

#endif // CVC_WITH_PLANAR

tile_result tile_contours(const contour_stack &stack, const tile_params &params) {
  tile_stepper s(stack, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<tile_stepper> tile_contours_stepper(contour_stack stack, tile_params params) {
  return std::make_unique<tile_stepper>(std::move(stack), std::move(params));
}

std::unique_ptr<result_job<tile_result>> tile_contours_job(contour_stack stack,
                                                           tile_params params) {
  return make_result_job(std::make_unique<tile_stepper>(std::move(stack), std::move(params)));
}

} // namespace cvc
