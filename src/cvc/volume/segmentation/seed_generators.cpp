/*
  Copyright 2005-2011, 2026 The University of Texas at Austin

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

// cvc/volume/seed_generators.h: VolUtils' criticalPntsToSeed (SegSubunit's FindCriticalPoints)
// and SeedProduce / SeedProduceRegardingAxis (transfix/volrover edabb85^, src/VolUtils), ported.
// What changed:
//   - no files: the legacy tools read a .rawiv and a .vgr and wrote maxcrit.vgr, saddlecrit.vgr
//     or the rotated seeds' .vgr; these take and return cvc::volume and cvc::point_classes;
//   - capsid_critical_points is a stepper (§1.9) on the GenSeg port's coroutines; the two lists
//     come back as two classes of one point_classes; the volume's minimum and maximum start from
//     its first voxel (the legacy started from -/+100000); a gradient magnitude that is the same
//     everywhere gives no saddles (the legacy divided by zero);
//   - rotated_seeds takes the axis and a point on it as parameters. SeedProduceRegardingAxis read
//     them from a file of points (the axis through their mean and the first point), and summed
//     each point's y where the mean's z belonged; SeedProduce used pi = 3.1415926. A rotated point
//     goes to the nearest voxel by floor(x + 0.5): the legacy (int)(x + 0.5) truncated towards 0,
//     so points up to a voxel outside the volume's low faces landed on index 0.

#include "critical_points.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cvc/volume/seed_generators.h>
#include <cvc/volume/seed_generators_params.h>
#include <cvc/volume/segmentation.h>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace cvc {

namespace {

constexpr std::size_t slice_units = 4096;

std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

} // namespace

// ---- capsid_critical_points
// ----------------------------------------------------------------------

struct capsid_critical_points_stepper::impl {
  volume in;
  capsid_seed_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;
  double prog = 0.0;

  genseg::genseg_context ctx;
  std::vector<std::size_t> maxima, saddles;
  std::optional<point_classes> result;

  genseg::task task; // last, so it is destroyed first

  impl(const volume &v, capsid_seed_params p) : in(v), params(p) { ctx.begin("checking"); }

  genseg::task run();
  genseg::task local_maxima(const float *f, float tlow, genseg::buffer<unsigned char> &mask,
                            long histogram[256], long &count);
  void note_progress();
  const char *phase() const {
    if (failed)
      return "failed";
    if (finished)
      return "done";
    return ctx.phase;
  }
};

// FindCriticalPoints' scan: the voxels of `f` whose dataset value is above tlow and which no voxel
// of their 3x3x3 block exceeds, with a histogram of their f values.
genseg::task capsid_critical_points_stepper::impl::local_maxima(const float *f, float tlow,
                                                                genseg::buffer<unsigned char> &mask,
                                                                long histogram[256], long &count) {
  const int X = ctx.XDIM, Y = ctx.YDIM, Z = ctx.ZDIM;
  const float *d = ctx.dataset.data();
  auto I = [X, Y](int i, int j, int k) {
    return (std::size_t(k) * std::size_t(Y) + std::size_t(j)) * std::size_t(X) + std::size_t(i);
  };
  for (int b = 0; b < 256; ++b)
    histogram[b] = 0;
  count = 0;
  for (int k = 0; k < Z; k++)
    for (int j = 0; j < Y; j++)
      for (int i = 0; i < X; i++) {
        const std::size_t v = I(i, j, k);
        mask[v] = 0;
        if (d[v] > tlow) {
          const float tmp = f[v];
          bool higher = false;
          for (int z = std::max(k - 1, 0); z <= std::min(k + 1, Z - 1); z++)
            for (int y = std::max(j - 1, 0); y <= std::min(j + 1, Y - 1); y++)
              for (int x = std::max(i - 1, 0); x <= std::min(i + 1, X - 1); x++)
                if (tmp < f[I(x, y, z)])
                  higher = true;
          if (!higher) {
            mask[v] = 1;
            histogram[std::clamp(int(tmp + 0.5f), 0, 255)] += 1;
            ++count;
          }
        }
        GS_UNIT_OF(ctx);
      }
}

// The histogram's threshold: the value above which more than `keep` of the counted voxels lie
// (scanning down from 255), but at least tlow + 20.
static float histogram_threshold(const long histogram[256], long keep, float tlow) {
  long number = 0;
  int k;
  for (k = 255; k >= 0; k--) {
    number += histogram[k];
    if (number > keep)
      break;
  }
  float t = float(k);
  if (t < tlow + 20.0f)
    t = tlow + 20.0f;
  return t;
}

genseg::task capsid_critical_points_stepper::impl::run() {
  ctx.begin("checking");
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "capsid_critical_points: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw segmentation_error(m);
    }
  }
  ctx.begin("normalising");
  GS_RUN(genseg::read_voxels(ctx, in, "capsid_critical_points"));
  const std::size_t n = ctx.voxel_count();
  // criticalPntsToSeed's rescale to 0..255 (in double, as it read VolMagick's voxels).
  float mn = ctx.dataset[0], mx = ctx.dataset[0];
  for (std::size_t v = 0; v < n; ++v) {
    mn = std::min(mn, ctx.dataset[v]);
    mx = std::max(mx, ctx.dataset[v]);
    GS_UNIT_OF(ctx);
  }
  for (std::size_t v = 0; v < n; ++v) {
    ctx.dataset[v] = float(((double(ctx.dataset[v]) - mn) / (double(mx) - mn)) * 255.0);
    GS_UNIT_OF(ctx);
  }
  const float tlow = params.normalized_thresholds
                         ? float(params.t_low)
                         : float(((params.t_low - mn) / (double(mx) - mn)) * 255.0);

  ctx.begin("maxima");
  const long tri_num =
      3000L * (long(params.h) * params.h + long(params.k) * params.k + long(params.h) * params.k);
  genseg::buffer<unsigned char> mask;
  mask.allocate(n);
  long histogram[256];
  long count = 0;
  GS_RUN(local_maxima(ctx.dataset.data(), tlow, mask, histogram, count));
  float thigh = histogram_threshold(histogram, tri_num, tlow);
  for (std::size_t v = 0; v < n; ++v) {
    if (mask[v] == 1 && ctx.dataset[v] > thigh)
      maxima.push_back(v);
    GS_UNIT_OF(ctx);
  }

  ctx.begin("saddles");
  const long kept = long(maxima.size());
  if (kept < std::min(50000L, tri_num / 5)) {
    // The gradient magnitude (GetImgGradient: GenSeg's operator over 16), zero where the 3x3x3
    // block touches a zero voxel, rescaled to 0..255.
    const int X = ctx.XDIM, Y = ctx.YDIM, Z = ctx.ZDIM;
    auto I = [X, Y](int i, int j, int k) {
      return (std::size_t(k) * std::size_t(Y) + std::size_t(j)) * std::size_t(X) + std::size_t(i);
    };
    const float *d = ctx.dataset.data();
    genseg::buffer<float> g;
    g.allocate(n);
    float ming = 999999.0f, maxg = 0.0f;
    for (int z = 0; z < Z; z++)
      for (int y = 0; y < Y; y++)
        for (int x = 0; x < X; x++) {
          const std::size_t v = I(x, y, z);
          float value = 0.0f;
          if (d[v] != 0) {
            bool zero = false;
            for (int k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++)
              for (int j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++)
                for (int i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
                  if (d[I(i, j, k)] == 0)
                    zero = true;
            if (!zero) {
              float gx = 0.0f, gy = 0.0f, gz = 0.0f;
              for (int j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++)
                for (int k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
                  const float t =
                      d[I(std::min(x + 1, X - 1), j, k)] - d[I(std::max(x - 1, 0), j, k)];
                  gx += t;
                  if (j == y || k == z)
                    gx += t;
                  if (j == y && k == z)
                    gx += 2.0f * t;
                }
              for (int i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
                for (int k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
                  const float t =
                      d[I(i, std::min(y + 1, Y - 1), k)] - d[I(i, std::max(y - 1, 0), k)];
                  gy += t;
                  if (i == x || k == z)
                    gy += t;
                  if (i == x && k == z)
                    gy += 2.0f * t;
                }
              for (int i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
                for (int j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++) {
                  const float t =
                      d[I(i, j, std::min(z + 1, Z - 1))] - d[I(i, j, std::max(z - 1, 0))];
                  gz += t;
                  if (i == x || j == y)
                    gz += t;
                  if (i == x && j == y)
                    gz += 2.0f * t;
                }
              value = std::sqrt(gx * gx + gy * gy + gz * gz) / 16.0f;
            }
          }
          g[v] = value;
          maxg = std::max(maxg, value);
          ming = std::min(ming, value);
          GS_UNIT_OF(ctx);
        }
    if (maxg > ming) {
      for (std::size_t v = 0; v < n; ++v) {
        g[v] = 255.0f * (g[v] - ming) / (maxg - ming);
        GS_UNIT_OF(ctx);
      }
      GS_RUN(local_maxima(g.data(), tlow, mask, histogram, count));
      const float t2 = histogram_threshold(histogram, kept, tlow);
      for (std::size_t v = 0; v < n; ++v) {
        if (mask[v] == 1 && g[v] > t2)
          saddles.push_back(v);
        GS_UNIT_OF(ctx);
      }
    }
  }
  ctx.dataset.release();

  point_classes &r = result.emplace();
  const std::size_t X = std::size_t(ctx.XDIM), Y = std::size_t(ctx.YDIM);
  for (int c = 0; c < 2; ++c) {
    const std::vector<std::size_t> &list = c == 0 ? maxima : saddles;
    point_class pc;
    pc.name = c == 0 ? "maxima" : "saddles";
    pc.color = c == 0 ? std::vector<double>{1.0, 1.0, 0.0} : std::vector<double>{0.0, 1.0, 1.0};
    pc.variable = params.variable;
    pc.timestep = params.timestep;
    pc.points.reserve(3 * list.size());
    for (std::size_t s = 0; s < list.size(); ++s) {
      const std::size_t v = list[s];
      pc.points.insert(pc.points.end(), {int(v % X), int((v / X) % Y), int(v / (X * Y))});
      GS_UNIT_OF(ctx);
    }
    r.classes.push_back(std::move(pc));
  }
}

void capsid_critical_points_stepper::impl::note_progress() {
  const double n = double(ctx.voxel_count() ? ctx.voxel_count() : 1);
  const std::string_view ph = ctx.phase;
  double lo = 0.0, hi = 0.0, within = 0.0;
  if (ph == "normalising") {
    lo = 0.01, hi = 0.3, within = double(ctx.units) / (3.0 * n);
  } else if (ph == "maxima") {
    lo = 0.3, hi = 0.6, within = double(ctx.units) / (2.0 * n);
  } else if (ph == "saddles") {
    lo = 0.6, hi = 1.0, within = double(ctx.units) / (4.0 * n);
  }
  const double p = lo + (hi - lo) * std::min(1.0, std::max(0.0, within));
  prog = std::max(prog, std::min(p, 1.0));
}

capsid_critical_points_stepper::capsid_critical_points_stepper(const volume &vol,
                                                               capsid_seed_params params)
    : _impl(std::make_unique<impl>(vol, params)) {}
capsid_critical_points_stepper::~capsid_critical_points_stepper() = default;
capsid_critical_points_stepper::capsid_critical_points_stepper(
    capsid_critical_points_stepper &&) noexcept = default;
capsid_critical_points_stepper &
capsid_critical_points_stepper::operator=(capsid_critical_points_stepper &&) noexcept = default;

bool capsid_critical_points_stepper::step(std::size_t budget) {
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
      m.error = std::make_exception_ptr(segmentation_error("capsid_critical_points: cancelled"));
      m.failed = true;
      m.task = genseg::task();
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

bool capsid_critical_points_stepper::done() const {
  return !_impl || _impl->finished || _impl->failed;
}
const char *capsid_critical_points_stepper::phase() const {
  return _impl ? _impl->phase() : "done";
}
double capsid_critical_points_stepper::progress() const { return _impl ? _impl->prog : 1.0; }
std::string capsid_critical_points_stepper::status() const {
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
  if (m.finished)
    return fmt("done: %zu maxima, %zu saddles", m.maxima.size(), m.saddles.size());
  return fmt("%s %d%%", m.phase(), int(m.prog * 100));
}
void capsid_critical_points_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}
point_classes capsid_critical_points_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished || !_impl->result)
    throw segmentation_error("capsid_critical_points: take() before the stepper is done");
  point_classes r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

point_classes capsid_critical_points(const volume &vol, const capsid_seed_params &params) {
  capsid_critical_points_stepper s(vol, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<point_classes>> capsid_critical_points_job(const volume &vol,
                                                                      capsid_seed_params params) {
  return make_result_job(std::make_unique<capsid_critical_points_stepper>(vol, params));
}

// ---- rotated_seeds
// -------------------------------------------------------------------------------

namespace {

// SeedProduce's class colours, cycled.
constexpr std::array<unsigned, 13> legacy_colors = {
    0xffff00, 0xff0000, 0x00ff00, 0xff00ff, 0x0000ff, 0xff5500, 0x336699,
    0x00ffff, 0xc0c0c0, 0x800000, 0x800080, 0x808000, 0x008080};

double span_of(double lo, double hi, std::uint64_t dim) {
  return dim > 1 ? (hi - lo) / double(dim - 1) : 1.0;
}

} // namespace

point_classes rotated_seeds(const point_classes &seeds, const dimension &from_dims,
                            const bounding_box &from_box, const volume &to,
                            const rotation_seed_params &params) {
  const std::vector<std::string> problems = params_validate(params);
  if (!problems.empty()) {
    std::string m = "rotated_seeds: ";
    for (std::size_t i = 0; i < problems.size(); ++i)
      m += (i ? "; " : "") + problems[i];
    throw segmentation_error(m);
  }
  if (from_dims.XDim() == 0 || from_dims.YDim() == 0 || from_dims.ZDim() == 0)
    throw segmentation_error("rotated_seeds: the seeds' grid is empty");
  if (to.XDim() == 0 || to.YDim() == 0 || to.ZDim() == 0)
    throw segmentation_error("rotated_seeds: the target volume is empty");
  const bounding_box &tb = to.boundingBox();
  const double fmin[3] = {from_box.minx, from_box.miny, from_box.minz};
  const double fspan[3] = {span_of(from_box.minx, from_box.maxx, from_dims.XDim()),
                           span_of(from_box.miny, from_box.maxy, from_dims.YDim()),
                           span_of(from_box.minz, from_box.maxz, from_dims.ZDim())};
  const double tmin[3] = {tb.minx, tb.miny, tb.minz};
  const double tspan[3] = {span_of(tb.minx, tb.maxx, to.XDim()),
                           span_of(tb.miny, tb.maxy, to.YDim()),
                           span_of(tb.minz, tb.maxz, to.ZDim())};
  const std::int64_t tdim[3] = {std::int64_t(to.XDim()), std::int64_t(to.YDim()),
                                std::int64_t(to.ZDim())};
  double c[3];
  if (params.center.empty()) {
    c[0] = (tb.minx + tb.maxx) * 0.5;
    c[1] = (tb.miny + tb.maxy) * 0.5;
    c[2] = (tb.minz + tb.maxz) * 0.5;
  } else {
    for (int a = 0; a < 3; ++a)
      c[a] = params.center[std::size_t(a)];
  }
  double v[3] = {params.axis[0], params.axis[1], params.axis[2]};
  const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  for (double &e : v)
    e /= len;

  point_classes out;
  const double pi = std::acos(-1.0);
  std::vector<std::array<std::array<double, 3>, 3>> rot(std::size_t(params.folds));
  for (int n = 0; n < params.folds; ++n) {
    // Rodrigues' rotation by alpha about v (SeedProduceRegardingAxis' defineRotationMatrix).
    const double alpha = 2.0 * pi * n / params.folds, cs = std::cos(alpha), sn = std::sin(alpha),
                 t = 1.0 - cs;
    auto &m = rot[std::size_t(n)];
    m[0] = {v[0] * v[0] * t + cs, v[0] * v[1] * t - v[2] * sn, v[0] * v[2] * t + v[1] * sn};
    m[1] = {v[0] * v[1] * t + v[2] * sn, v[1] * v[1] * t + cs, v[1] * v[2] * t - v[0] * sn};
    m[2] = {v[0] * v[2] * t - v[1] * sn, v[1] * v[2] * t + v[0] * sn, v[2] * v[2] * t + cs};
    point_class pc;
    pc.name = "Class " + std::to_string(n);
    const unsigned rgb = legacy_colors[std::size_t(n) % legacy_colors.size()];
    pc.color = {((rgb >> 16) & 0xff) / 255.0, ((rgb >> 8) & 0xff) / 255.0, (rgb & 0xff) / 255.0};
    pc.variable = params.variable;
    pc.timestep = params.timestep;
    out.classes.push_back(std::move(pc));
  }
  for (const point_class &cls : seeds.classes) {
    if (cls.variable != params.variable || cls.timestep != params.timestep)
      continue;
    for (std::size_t p = 0; p + 2 < cls.points.size(); p += 3) {
      double d[3];
      for (int a = 0; a < 3; ++a)
        d[a] = fmin[a] + cls.points[p + std::size_t(a)] * fspan[a] - c[a];
      for (int n = 0; n < params.folds; ++n) {
        const auto &m = rot[std::size_t(n)];
        std::int64_t idx[3];
        bool inside = true;
        for (int a = 0; a < 3; ++a) {
          const double w = m[std::size_t(a)][0] * d[0] + m[std::size_t(a)][1] * d[1] +
                           m[std::size_t(a)][2] * d[2] + c[a];
          idx[a] = std::int64_t(std::floor((w - tmin[a]) / tspan[a] + 0.5));
          inside = inside && idx[a] >= 0 && idx[a] < tdim[a];
        }
        if (inside)
          out.classes[std::size_t(n)].points.insert(out.classes[std::size_t(n)].points.end(),
                                                    {int(idx[0]), int(idx[1]), int(idx[2])});
      }
    }
  }
  return out;
}

point_classes rotated_seeds(const point_classes &seeds, const volume &to,
                            const rotation_seed_params &params) {
  return rotated_seeds(seeds, to.voxel_dimensions(), to.boundingBox(), to, params);
}

} // namespace cvc
