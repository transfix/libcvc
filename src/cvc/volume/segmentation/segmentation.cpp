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

// cvc::segment_stepper (cvc/volume/segmentation.h): checks the input, hands it to the vendored
// GenSeg (genseg_context, genseg.cpp) as the legacy entry did, and turns what it leaves behind
// into the result's volumes, a unit at a time.

#include "critical_points.h"
#include "genseg.h"

#include <atomic>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
#include <exception>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

namespace cvc {

namespace {

// Units between two looks at cancel() and the progress, inside one step. A per-voxel pass spreads
// a slice over the app's compute pool in pieces of genseg::for_voxels_piece voxels: 32 of them a
// slice, so one fork-join barrier covers enough work to be worth it on a busy machine (8 a slice
// measured about twice as slow there). A slice is still well under a tenth of a second.
constexpr std::size_t slice_units = 262144;

std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

} // namespace

// The run: the stepper's state, the vendored context, and the coroutine that drives both.
struct segment_stepper::impl {
  volume in;
  point_classes seeds;
  seeded_segmentation_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;
  double prog = 0.0;

  genseg::genseg_context ctx;
  genseg::critical_seeds critical; // seed_mode::critical_points
  std::size_t user_classes = 0;    // the point classes (before the critical points' two)
  std::vector<int> class_of_seed; // per seed of ctx.seed_list, flattened: its class
  std::vector<std::string> names;
  std::vector<std::string> warnings;
  std::optional<seeded_segmentation> result;

  genseg::task task; // last, so it is destroyed first

  impl(const volume &v, point_classes s, seeded_segmentation_params p)
      : in(v), seeds(std::move(s)), params(p) {
    ctx.begin("checking");
  }

  genseg::task run();
  void note_progress();
  const char *phase() const {
    if (failed)
      return "failed";
    if (finished)
      return "done";
    return ctx.phase;
  }
};

genseg::task segment_stepper::impl::run() {
  ctx.begin("checking");
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "segment_seeded: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw segmentation_error(m);
    }
  }
  if (in.voxelType() == Undefined)
    throw segmentation_error("segment_seeded: the volume's voxel type is undefined");
  if (in.XDim() > uint64(INT_MAX) || in.YDim() > uint64(INT_MAX) || in.ZDim() > uint64(INT_MAX))
    throw segmentation_error("segment_seeded: the volume is too large");
  const int X = int(in.XDim()), Y = int(in.YDim()), Z = int(in.ZDim());
  const std::string dims = fmt("%dx%dx%d", X, Y, Z);

  // The classes of the variable and timestep asked for, in order, as VolumeGridRover sent them.
  // seed_mode::critical_points adds two after them.
  const bool critical_mode = params.automatic == seed_mode::critical_points;
  const std::size_t max_classes = critical_mode ? 252 : 254;
  std::size_t points = 0;
  std::unordered_map<std::size_t, std::size_t> class_at; // voxel -> the class placing it last
  for (const point_class &c : seeds.classes) {
    if (c.variable != params.variable || c.timestep != params.timestep)
      continue;
    if (names.size() == max_classes)
      throw segmentation_error(fmt("segment_seeded: more than %zu point classes of variable %d, "
                                   "timestep %d%s",
                                   max_classes, params.variable, params.timestep,
                                   critical_mode ? " (critical_points adds two)" : ""));
    const std::size_t cls = names.size();
    names.push_back(c.name);
    ctx.seed_list.emplace_back();
    if (c.points.size() % 3 != 0)
      throw segmentation_error("segment_seeded: class '" + c.name +
                               "' has a point list that is not index triples");
    if (c.points.empty())
      warnings.push_back("class '" + c.name + "' has no points, so it labels nothing");
    for (std::size_t p = 0; p + 2 < c.points.size(); p += 3) {
      const int i = c.points[p], j = c.points[p + 1], k = c.points[p + 2];
      if (i < 0 || j < 0 || k < 0 || i >= X || j >= Y || k >= Z)
        throw segmentation_error(fmt("segment_seeded: seed (%d, %d, %d) of class '%s' is outside "
                                     "the %s volume",
                                     i, j, k, c.name.c_str(), dims.c_str()));
      ctx.seed_list.back().push_back(genseg::INTVEC{i, j, k});
      class_of_seed.push_back(int(cls));
      const std::size_t v = (std::size_t(k) * Y + j) * X + i;
      auto it = class_at.find(v);
      if (it == class_at.end())
        class_at.emplace(v, cls);
      else if (it->second != cls) {
        warnings.push_back(fmt("voxel (%d, %d, %d) is a point of classes '%s' and '%s'; '%s' has "
                               "it",
                               i, j, k, names[it->second].c_str(), c.name.c_str(), c.name.c_str()));
        it->second = cls;
      }
      ++points;
      GS_UNIT_OF(ctx);
    }
  }
  // With critical points, the point classes may have none: the critical points then seed the run
  // alone (critical_seeds throws if there are none either).
  if (points == 0 && !critical_mode)
    throw segmentation_error(fmt("segment_seeded: no seeds: no point class of variable %d, "
                                 "timestep %d has a point",
                                 params.variable, params.timestep));
  user_classes = names.size();
  if (critical_mode) {
    names.push_back("feature");
    names.push_back("background");
    genseg::configure(critical, params);
    critical.need_seeds = points == 0;
    critical.who = "segment_seeded";
    ctx.critical = &critical;
  }

  // read_data: the voxels as floats.
  ctx.begin("normalising");
  GS_RUN(genseg::read_voxels(ctx, in, "segment_seeded"));
  const std::size_t n = ctx.voxel_count();
  ctx.tlow = float(params.t_low);
  ctx.thigh = float(params.t_high);
  ctx.data_unit_thresholds = !params.normalized_thresholds;
  ctx.ALPHA = float(-params.gamma);
  ctx.automatic_seeds = params.automatic == seed_mode::local_maxima;

  GS_RUN(ctx.run());

  ctx.begin("writing");
  seeded_segmentation &r = result.emplace(in.ctx());
  const dimension d(X, Y, Z);
  r.labels = volume(in.ctx(), d, UChar, in.boundingBox());
  r.class_names = names;
  r.class_voxels.assign(names.size(), 0);
  {
    // Each voxel's label, and each class's count (each piece's, then their sum).
    unsigned char *lab = r.labels.data_ptr();
    const std::size_t classes = names.size();
    std::mutex join;
    const auto label = [&](std::size_t b, std::size_t e) {
      std::vector<std::size_t> count(classes, 0);
      for (std::size_t v = b; v < e; ++v) {
        const unsigned char s = ctx.seed_index[v];
        if (s < 254 && s < classes) {
          lab[v] = s;
          ++count[s];
        } else {
          lab[v] = seeded_segmentation::unlabelled;
        }
      }
      const std::lock_guard<std::mutex> hold(join);
      for (std::size_t c = 0; c < classes; ++c)
        r.class_voxels[c] += count[c];
    };
    GS_FOR_VOXELS_OF(ctx, n, label);
    // The border, in x-fastest order.
    std::size_t v = 0;
    for (int k = 0; k < Z; ++k)
      for (int j = 0; j < Y; ++j)
        for (int i = 0; i < X; ++i, ++v) {
          if (ctx.tdata[v] == genseg::genseg_context::MAX_TIME + 1)
            r.border.push_back({i, j, k});
          GS_UNIT_OF(ctx);
        }
  }
  // A seed below t_low labels only itself and the neighbours it stops at. The critical points'
  // seeds get one warning per class, not one each.
  {
    for (std::size_t c = 0; c < ctx.seed_list.size(); ++c) {
      std::size_t below = 0;
      for (const genseg::INTVEC &p : ctx.seed_list[c]) {
        const std::size_t v = (std::size_t(p.z) * Y + p.y) * X + p.x;
        if (ctx.dataset[v] < ctx.tlow) {
          if (c < user_classes)
            warnings.push_back(fmt("seed (%d, %d, %d) of class '%s' is below t_low, so its front "
                                   "stops at once",
                                   p.x, p.y, p.z, names[c].c_str()));
          ++below;
        }
        GS_UNIT_OF(ctx);
      }
      if (c >= user_classes && below > 0)
        warnings.push_back(fmt("%zu of the %zu critical points' seeds of class '%s' are below "
                               "t_low, so their fronts stop at once",
                               below, ctx.seed_list[c].size(), names[c].c_str()));
    }
  }
  if (critical_mode) {
    r.automatic_seeds = critical.feature.size() + critical.background.size();
    r.classified_seeds = r.automatic_seeds;
  } else {
    r.automatic_seeds = std::size_t(ctx.seeds_found - ctx.seeds_displaced);
    r.classified_seeds = std::size_t(ctx.seeds_found - ctx.seeds_displaced - ctx.seeds_left);
  }
  r.t_low_normalized = ctx.tlow;
  r.t_high_normalized = ctx.thigh;
  ctx.tdata.release();
  ctx.seed_index.release();
  ctx.dataset.release();

  if (params.class_volumes) {
    // Every class's volume (zero), then one pass copying each labelled voxel into its class's.
    const unsigned char *src = in.data_ptr();
    const unsigned char *lab = r.labels.data_ptr();
    const std::size_t size = in.voxelSize();
    for (std::size_t c = 0; c < names.size(); ++c)
      r.classes.emplace_back(in.ctx(), d, in.voxelType(), in.boundingBox());
    std::vector<unsigned char *> dst;
    for (volume &cv : r.classes)
      dst.push_back(cv.data_ptr());
    const auto copy = [&](std::size_t b, std::size_t e) {
      for (std::size_t v = b; v < e; ++v)
        if (lab[v] < dst.size())
          std::memcpy(dst[lab[v]] + v * size, src + v * size, size);
    };
    GS_FOR_VOXELS_OF(ctx, n, copy);
  }
  r.warnings = std::move(warnings);
}

void segment_stepper::impl::note_progress() {
  // Each phase's share of [0, 1], and how far through it the run is.
  const double n = double(ctx.voxel_count() ? ctx.voxel_count() : 1);
  const std::string_view ph = ctx.phase;
  double lo = 0.0, hi = 0.0, within = 0.0;
  if (ph == "checking") {
    lo = 0.0, hi = 0.01;
    within = double(ctx.units) / double(class_of_seed.size() + 1);
  } else if (ph == "normalising") {
    lo = 0.01, hi = 0.2;
    within = double(ctx.units) / (5.0 * n);
  } else if (ph == "seeding") {
    lo = 0.2, hi = 0.35;
    within = double(ctx.units) /
             ((ctx.automatic_seeds ? 7.0 : 2.0) * n +
              (ctx.critical ? double(critical.expected_units(ctx.voxel_count())) : 0.0));
  } else if (ph == "classifying") {
    lo = 0.35, hi = 0.6;
    within = ctx.seeds_found > 0 ? 1.0 - double(ctx.SeedNum) / double(ctx.seeds_found) : 1.0;
  } else if (ph == "marching") {
    lo = 0.6, hi = 0.9;
    within = double(ctx.units) / (2.0 * n);
  } else if (ph == "writing") {
    lo = 0.9, hi = 1.0;
    within = double(ctx.units) /
             ((2.0 + (params.class_volumes ? 1.0 : 0.0)) * n + double(class_of_seed.size()));
  }
  const double p = lo + (hi - lo) * std::min(1.0, std::max(0.0, within));
  prog = std::max(prog, std::min(p, 1.0));
}

segment_stepper::segment_stepper(const volume &vol, point_classes seeds,
                                 seeded_segmentation_params params)
    : _impl(std::make_unique<impl>(vol, std::move(seeds), params)) {}

segment_stepper::~segment_stepper() = default;
segment_stepper::segment_stepper(segment_stepper &&) noexcept = default;
segment_stepper &segment_stepper::operator=(segment_stepper &&) noexcept = default;

bool segment_stepper::step(std::size_t budget) {
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
    std::size_t left = budget == 0 ? SIZE_MAX : budget;
    while (left > 0) {
      if (m.cancel.load(std::memory_order_acquire))
        break;
      const std::size_t slice = std::min(left, slice_units);
      m.ctx.units_left = slice;
      if (m.task.resume()) {
        m.task = genseg::task();
        m.finished = true;
        m.prog = 1.0;
        return true;
      }
      if (left != SIZE_MAX)
        left -= slice;
      m.note_progress();
    }
    if (m.cancel.load(std::memory_order_acquire)) {
      m.error = std::make_exception_ptr(segmentation_error("segment_seeded: cancelled"));
      m.failed = true;
      m.task = genseg::task(); // frees every frame
      m.ctx = genseg::genseg_context();
      m.result.reset();
      return true;
    }
    return false;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    m.task = genseg::task();
    throw;
  }
}

bool segment_stepper::done() const { return !_impl || _impl->finished || _impl->failed; }
const char *segment_stepper::phase() const { return _impl ? _impl->phase() : "done"; }
double segment_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string segment_stepper::status() const {
  if (!_impl)
    return "done";
  const impl &m = *_impl;
  if (m.failed) {
    try {
      std::rethrow_exception(m.error);
    } catch (const std::exception &e) {
      return std::string("failed: ") + e.what();
    } catch (...) {
      return "failed";
    }
  }
  if (m.finished && m.result) {
    std::size_t labelled = 0;
    for (std::size_t c : m.result->class_voxels)
      labelled += c;
    return fmt("done: %zu classes, %zu voxels labelled, %zu border voxels",
               m.result->class_names.size(), labelled, m.result->border.size());
  }
  return fmt("%s %d%%", m.phase(), int(m.prog * 100));
}

void segment_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

bool segment_stepper::labels_so_far(volume &labels) const {
  if (!_impl || _impl->finished || _impl->failed)
    return false;
  const impl &m = *_impl;
  const std::string_view ph = m.ctx.phase;
  // seed_index is whole from the sweep on: before it, GetSeeds is still writing it.
  if ((ph != "classifying" && ph != "marching") || m.ctx.seed_index.size() == 0)
    return false;
  const dimension d(m.in.XDim(), m.in.YDim(), m.in.ZDim());
  if (labels.voxelType() != UChar || labels.voxel_dimensions() != d) {
    labels = volume(m.in.ctx(), d, UChar, m.in.boundingBox());
  } else if (labels.boundingBox() != m.in.boundingBox()) {
    labels.boundingBox(m.in.boundingBox());
  }
  unsigned char *out = labels.data_ptr();
  const unsigned char *in = m.ctx.seed_index.data();
  const std::size_t n = m.ctx.seed_index.size();
  const std::size_t classes = m.names.size();
  for (std::size_t v = 0; v < n; ++v) {
    const unsigned char s = in[v];
    out[v] = s < 254 && s < classes ? s : seeded_segmentation::unlabelled;
  }
  return true;
}

seeded_segmentation segment_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished || !_impl->result)
    throw segmentation_error("segment_seeded: take() before the stepper is done");
  seeded_segmentation r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

seeded_segmentation segment_seeded(const volume &vol, const point_classes &seeds,
                                   const seeded_segmentation_params &params) {
  segment_stepper s(vol, seeds, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<segment_stepper> segment_seeded_stepper(const volume &vol, point_classes seeds,
                                                        seeded_segmentation_params params) {
  return std::make_unique<segment_stepper>(vol, std::move(seeds), params);
}

std::unique_ptr<result_job<seeded_segmentation>>
segment_seeded_job(const volume &vol, point_classes seeds, seeded_segmentation_params params) {
  return make_result_job(std::make_unique<segment_stepper>(vol, std::move(seeds), params));
}

} // namespace cvc
