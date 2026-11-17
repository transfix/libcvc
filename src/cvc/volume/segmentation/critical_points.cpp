/******************************************************************************
                                Copyright

This code is developed within the Computational Visualization Center at The
University of Texas at Austin.

This code has been made available to you under the auspices of a Lesser General
Public License (LGPL) (http://www.ices.utexas.edu/cvc/software/license.html)
and terms that you have agreed to.

Upon accepting the LGPL, we request you agree to acknowledge the use of use of
the code that results in any published work, including scientific papers,
films, and videotapes by citing the following references:

C. Bajaj, Z. Yu, M. Auer
Volumetric Feature Extraction and Visualization of Tomographic Molecular Imaging
Journal of Structural Biology, Volume 144, Issues 1-2, October 2003, Pages
132-143.

If you desire to use this code for a profit venture, or if you do not wish to
accept LGPL, but desire usage of this code, please contact Chandrajit Bajaj
(bajaj@ices.utexas.edu) at the Computational Visualization Center at The
University of Texas at Austin for a different license.
******************************************************************************/

// SegMed's GVF_Compute (transfix/volrover src/Segmentation/SegMed/GVF.cpp) as
// genseg::critical_seeds (critical_points.h says what changed), and cvc::critical_point_seeds
// (cvc/volume/segmentation.h) around it. Built with -ffp-contract=off (/fp:precise on MSVC), as
// the rest of the port: the critical points depend on the signs of float dot products.

#include "critical_points.h"

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace cvc {

namespace {

// Units between two looks at cancel() and the progress, inside one step.
constexpr std::size_t slice_units = 4096;

std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

// Voxel v of `data`, of type `t`, as a float (read_data converted each voxel the same way).
float voxel_as_float(const unsigned char *data, data_type t, std::size_t v) {
  switch (t) {
  case UChar:
    return float(data[v]);
  case UShort:
    return float(reinterpret_cast<const unsigned short *>(data)[v]);
  case UInt:
    return float(reinterpret_cast<const unsigned int *>(data)[v]);
  case Float:
    return reinterpret_cast<const float *>(data)[v];
  case Double:
    return float(reinterpret_cast<const double *>(data)[v]);
  case UInt64:
    return float(reinterpret_cast<const std::uint64_t *>(data)[v]);
  case Char:
    return float(reinterpret_cast<const signed char *>(data)[v]);
  case Int:
    return float(reinterpret_cast<const int *>(data)[v]);
  case Int64:
    return float(reinterpret_cast<const std::int64_t *>(data)[v]);
  default:
    return 0.0f;
  }
}

} // namespace

namespace genseg {

task read_voxels(genseg_context &ctx, const volume &in, const char *who) {
  if (in.voxelType() == Undefined)
    throw segmentation_error(fmt("%s: the volume's voxel type is undefined", who));
  if (in.XDim() > uint64(INT_MAX) || in.YDim() > uint64(INT_MAX) || in.ZDim() > uint64(INT_MAX))
    throw segmentation_error(fmt("%s: the volume is too large", who));
  ctx.XDIM = int(in.XDim());
  ctx.YDIM = int(in.YDim());
  ctx.ZDIM = int(in.ZDim());
  const std::size_t n = ctx.voxel_count();
  const std::size_t X = std::size_t(ctx.XDIM), Y = std::size_t(ctx.YDIM);
  ctx.dataset.allocate(n);
  const unsigned char *data = in.data_ptr();
  const data_type t = in.voxelType();
  bool constant = true;
  for (std::size_t v = 0; v < n; ++v) {
    const float f = voxel_as_float(data, t, v);
    if (!std::isfinite(f))
      throw segmentation_error(fmt("%s: voxel (%zu, %zu, %zu) is %s", who, v % X, (v / X) % Y,
                                   v / (X * Y), std::isnan(f) ? "NaN" : "not finite as a float"));
    ctx.dataset[v] = f;
    if (f != ctx.dataset[0])
      constant = false;
    GS_UNIT_OF(ctx);
  }
  if (constant)
    throw segmentation_error(fmt("%s: the volume is constant (every voxel is %g), so it has "
                                 "nothing to segment",
                                 who, double(ctx.dataset[0])));
}

void configure(critical_seeds &s, const seeded_segmentation_params &p) {
  s.field = p.cp_field;
  s.diffusion.iterations = p.cp_iterations;
  s.diffusion.k = p.cp_k;
  s.diffusion.weights = p.cp_weights;
  s.diffusion.normalize = true;
  s.feature_threshold = p.cp_feature_threshold;
  s.background_threshold = p.cp_background_threshold;
  s.data_unit_thresholds = !p.normalized_thresholds;
  s.sample = p.cp_sample;
}

std::size_t critical_seeds::expected_units(std::size_t n) const {
  const std::size_t it = std::size_t(std::max(diffusion.iterations, 0));
  return (field == critical_field::gradient_magnitude ? 2 * n : 0) // GetGradient
         + 2 * n                                                   // the initial field
         + (3 + it) * n                                            // the diffusion
         + n                                                       // the critical points
         + 4 * n;                                                  // the thinning's scans
}

task critical_seeds::run(genseg_context &ctx) {
  const std::size_t n = ctx.voxel_count();
  feature.clear();
  background.clear();
  maxima = minima = 0;
  // The thresholds, rescaled as normalise() rescaled the voxels (infinities stay infinite).
  _feature = float(feature_threshold);
  _background = float(background_threshold);
  if (data_unit_thresholds) {
    _feature = 255 * (_feature - ctx.minraw) / (ctx.maxraw - ctx.minraw);
    _background = 255 * (_background - ctx.minraw) / (ctx.maxraw - ctx.minraw);
  }

  // GVF_Compute: GetGradient(edge_mag), then InitGVF on it (SegMed), or on the volume (the paper).
  const float *f = ctx.dataset.data();
  if (field == critical_field::gradient_magnitude) {
    _edge.allocate(n);
    GS_RUN(ctx.GetGradient(_edge.data()));
    f = _edge.data();
  }
  GS_RUN(initial_field(ctx, f));
  _edge.release();
  GS_RUN(diffuse(ctx));
  GS_RUN(classify(ctx));
  if (need_seeds && feature.empty() && background.empty())
    throw segmentation_error(fmt("%s: no seeds: no point class has a point, and the volume has no "
                                 "critical points",
                                 who));
}

// InitGVF(): the gradient of f by GetGradient's 3x3x3 operator, divided by its longest vector's
// length (unless every vector has the same length).
task critical_seeds::initial_field(genseg_context &ctx, const float *f) {
  const int X = ctx.XDIM, Y = ctx.YDIM, Z = ctx.ZDIM;
  auto I = [X, Y](int i, int j, int k) {
    return (std::size_t(k) * std::size_t(Y) + std::size_t(j)) * std::size_t(X) + std::size_t(i);
  };
  _field = vector_field(X, Y, Z);
  int x, y, z, i, j, k;
  float gradient, grad_x, grad_y, grad_z;
  float maxgrad = 0, mingrad = 999999;
  for (z = 0; z < Z; z++)
    for (y = 0; y < Y; y++)
      for (x = 0; x < X; x++) {
        grad_x = 0.0;
        for (j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++)
          for (k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
            const float d = f[I(std::min(x + 1, X - 1), j, k)] - f[I(std::max(x - 1, 0), j, k)];
            grad_x += d;
            if (j == y || k == z)
              grad_x += d;
            if (j == y && k == z)
              grad_x += 2.0f * d;
          }
        grad_y = 0.0;
        for (i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
          for (k = std::max(z - 1, 0); k <= std::min(z + 1, Z - 1); k++) {
            const float d = f[I(i, std::min(y + 1, Y - 1), k)] - f[I(i, std::max(y - 1, 0), k)];
            grad_y += d;
            if (i == x || k == z)
              grad_y += d;
            if (i == x && k == z)
              grad_y += 2.0f * d;
          }
        grad_z = 0.0;
        for (i = std::max(x - 1, 0); i <= std::min(x + 1, X - 1); i++)
          for (j = std::max(y - 1, 0); j <= std::min(y + 1, Y - 1); j++) {
            const float d = f[I(i, j, std::min(z + 1, Z - 1))] - f[I(i, j, std::max(z - 1, 0))];
            grad_z += d;
            if (i == x || j == y)
              grad_z += d;
            if (i == x && j == y)
              grad_z += 2.0f * d;
          }
        const std::size_t v = I(x, y, z);
        _field.x[v] = grad_x;
        _field.y[v] = grad_y;
        _field.z[v] = grad_z;
        gradient = std::sqrt(grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
        if (gradient < mingrad)
          mingrad = gradient;
        if (gradient > maxgrad)
          maxgrad = gradient;
        GS_UNIT_OF(ctx);
      }
  if (mingrad < maxgrad) {
    const std::size_t n = _field.size();
    for (std::size_t v = 0; v < n; v++) {
      _field.x[v] /= maxgrad;
      _field.y[v] /= maxgrad;
      _field.z[v] /= maxgrad;
      GS_UNIT_OF(ctx);
    }
  }
}

// gvfflow(): cvc::gradient_vector_diffusion, run a slice at a time on ctx's units.
task critical_seeds::diffuse(genseg_context &ctx) {
  gradient_vector_diffusion_params p = diffusion;
  p.normalize = true;
  gradient_vector_diffusion_stepper gvd(std::move(_field), p);
  _field = vector_field();
  for (;;) {
    if (ctx.units_left == 0)
      co_yield 0;
    const std::size_t before = gvd.units();
    const bool done = gvd.step(ctx.units_left);
    const std::size_t used = gvd.units() - before;
    ctx.units += used;
    ctx.units_left = used >= ctx.units_left ? 0 : ctx.units_left - used;
    if (done)
      break;
  }
  _field = gvd.take();
  if (ctx.units_left == 0)
    co_yield 0;
}

// The critical points (GVF_Compute's search, cvc::critical_point_at), into the two classes by the
// paper's §4 thresholds, then each class thinned.
task critical_seeds::classify(genseg_context &ctx) {
  const int X = ctx.XDIM, Y = ctx.YDIM, Z = ctx.ZDIM;
  const std::size_t n = ctx.voxel_count();
  std::vector<unsigned char> fmask(n), bmask(n);
  std::size_t v = 0;
  for (int z = 0; z < Z; z++)
    for (int y = 0; y < Y; y++)
      for (int x = 0; x < X; x++, v++) {
        const critical_kind kind = critical_point_at(_field, x, y, z);
        const float value = ctx.dataset[v];
        unsigned char fm = 0, bm = 0;
        if (kind == critical_kind::maximum) {
          ++maxima;
          if (field == critical_field::image) {
            if (value < _feature)
              bm = 1;
            else
              fm = 1;
          }
        } else if (kind == critical_kind::minimum) {
          ++minima;
          if (value > _background)
            fm = 1;
          else
            bm = 1;
        }
        fmask[v] = fm;
        bmask[v] = bm;
        GS_UNIT_OF(ctx);
      }
  _field = vector_field();
  GS_RUN(thin(ctx, fmask, feature));
  GS_RUN(thin(ctx, bmask, background));
}

// GVF_Compute's sampling and GVF_Cluster(): from each critical voxel not yet visited, in x-fastest
// order, a depth-first walk through the run of critical voxels 26-connected to it keeps the first
// and then one voxel in every sample + 1 it visits (the count running on across the walk's
// branches, as the legacy global did). The kept voxels, in x-fastest order, go to `out`. mask: 1 a
// critical voxel not yet visited, 0 none (or visited), 2 kept.
task critical_seeds::thin(genseg_context &ctx, std::vector<unsigned char> &mask,
                          std::vector<std::size_t> &out) {
  const int X = ctx.XDIM, Y = ctx.YDIM, Z = ctx.ZDIM;
  const std::size_t n = ctx.voxel_count();
  const std::size_t sy = std::size_t(X), sz = std::size_t(X) * std::size_t(Y);
  if (sample > 0) {
    struct frame {
      int i, j, k;
      int next; // the next of the 27 offsets, z slowest, x fastest
    };
    std::vector<frame> stack;
    int count = 0;
    std::size_t v = 0;
    for (int z = 0; z < Z; z++)
      for (int y = 0; y < Y; y++)
        for (int x = 0; x < X; x++, v++) {
          GS_UNIT_OF(ctx);
          if (mask[v] != 1)
            continue;
          mask[v] = 2;
          count = 0;
          stack.push_back({x, y, z, 0});
          while (!stack.empty()) {
            frame &top = stack.back();
            if (top.next == 27) {
              stack.pop_back();
              continue;
            }
            const int t = top.next++;
            const int l = top.k + t / 9 - 1, m = top.j + (t / 3) % 3 - 1, o = top.i + t % 3 - 1;
            GS_UNIT_OF(ctx);
            if (l < 0 || l >= Z || m < 0 || m >= Y || o < 0 || o >= X)
              continue;
            const std::size_t q = std::size_t(l) * sz + std::size_t(m) * sy + std::size_t(o);
            if (mask[q] != 1)
              continue;
            if (count < sample) {
              count++;
              mask[q] = 0;
            } else {
              mask[q] = 2;
              count = 0;
            }
            stack.push_back({o, m, l, 0});
          }
        }
  } else {
    for (std::size_t v = 0; v < n; v++) {
      if (mask[v] == 1)
        mask[v] = 2;
      GS_UNIT_OF(ctx);
    }
  }
  for (std::size_t v = 0; v < n; v++) {
    if (mask[v] == 2)
      out.push_back(v);
    GS_UNIT_OF(ctx);
  }
}

} // namespace genseg

// ---- cvc::critical_point_seeds -----------------------------------------------------------------

struct critical_point_seeds_stepper::impl {
  volume in;
  seeded_segmentation_params params;

  std::atomic<bool> cancel{false};
  bool finished = false, failed = false;
  std::exception_ptr error;
  double prog = 0.0;

  genseg::genseg_context ctx;
  genseg::critical_seeds seeds;
  std::optional<point_classes> result;

  genseg::task task; // last, so it is destroyed first

  impl(const volume &v, seeded_segmentation_params p) : in(v), params(p) { ctx.begin("checking"); }

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

genseg::task critical_point_seeds_stepper::impl::run() {
  ctx.begin("checking");
  {
    const std::vector<std::string> problems = params_validate(params);
    if (!problems.empty()) {
      std::string m = "critical_point_seeds: ";
      for (std::size_t i = 0; i < problems.size(); ++i)
        m += (i ? "; " : "") + problems[i];
      throw segmentation_error(m);
    }
  }
  ctx.begin("normalising");
  GS_RUN(genseg::read_voxels(ctx, in, "critical_point_seeds"));
  GS_RUN(ctx.normalise());

  ctx.begin("seeding");
  genseg::configure(seeds, params);
  GS_RUN(seeds.run(ctx));
  ctx.dataset.release();

  point_classes &r = result.emplace();
  for (int c = 0; c < 2; ++c) {
    const std::vector<std::size_t> &list = c == 0 ? seeds.feature : seeds.background;
    point_class pc;
    pc.name = c == 0 ? "feature" : "background";
    pc.color = c == 0 ? std::vector<double>{1.0, 0.0, 0.0} : std::vector<double>{0.0, 0.0, 1.0};
    pc.variable = params.variable;
    pc.timestep = params.timestep;
    pc.points.reserve(3 * list.size());
    const std::size_t X = std::size_t(ctx.XDIM), Y = std::size_t(ctx.YDIM);
    for (std::size_t s = 0; s < list.size(); ++s) {
      const std::size_t v = list[s];
      pc.points.insert(pc.points.end(), {int(v % X), int((v / X) % Y), int(v / (X * Y))});
      GS_UNIT_OF(ctx);
    }
    r.classes.push_back(std::move(pc));
  }
}

void critical_point_seeds_stepper::impl::note_progress() {
  const double n = double(ctx.voxel_count() ? ctx.voxel_count() : 1);
  const std::string_view ph = ctx.phase;
  double lo = 0.0, hi = 0.0, within = 0.0;
  if (ph == "checking") {
    lo = 0.0, hi = 0.01;
  } else if (ph == "normalising") {
    lo = 0.01, hi = 0.1;
    within = double(ctx.units) / (3.0 * n);
  } else if (ph == "seeding") {
    lo = 0.1, hi = 1.0;
    within = double(ctx.units) / double(seeds.expected_units(ctx.voxel_count()) + 1);
  }
  const double p = lo + (hi - lo) * std::min(1.0, std::max(0.0, within));
  prog = std::max(prog, std::min(p, 1.0));
}

critical_point_seeds_stepper::critical_point_seeds_stepper(const volume &vol,
                                                           seeded_segmentation_params params)
    : _impl(std::make_unique<impl>(vol, params)) {}

critical_point_seeds_stepper::~critical_point_seeds_stepper() = default;
critical_point_seeds_stepper::critical_point_seeds_stepper(
    critical_point_seeds_stepper &&) noexcept = default;
critical_point_seeds_stepper &
critical_point_seeds_stepper::operator=(critical_point_seeds_stepper &&) noexcept = default;

bool critical_point_seeds_stepper::step(std::size_t budget) {
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
      m.error = std::make_exception_ptr(segmentation_error("critical_point_seeds: cancelled"));
      m.failed = true;
      m.task = genseg::task(); // frees every frame
      m.ctx = genseg::genseg_context();
      m.seeds = genseg::critical_seeds();
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

bool critical_point_seeds_stepper::done() const {
  return !_impl || _impl->finished || _impl->failed;
}
const char *critical_point_seeds_stepper::phase() const { return _impl ? _impl->phase() : "done"; }
double critical_point_seeds_stepper::progress() const { return _impl ? _impl->prog : 1.0; }

std::string critical_point_seeds_stepper::status() const {
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
  if (m.finished && m.result)
    return fmt("done: %zu feature and %zu background seeds (%zu maxima, %zu minima)",
               m.seeds.feature.size(), m.seeds.background.size(), m.seeds.maxima, m.seeds.minima);
  return fmt("%s %d%%", m.phase(), int(m.prog * 100));
}

void critical_point_seeds_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

point_classes critical_point_seeds_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->finished || !_impl->result)
    throw segmentation_error("critical_point_seeds: take() before the stepper is done");
  point_classes r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

point_classes critical_point_seeds(const volume &vol, const seeded_segmentation_params &params) {
  critical_point_seeds_stepper s(vol, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<point_classes>>
critical_point_seeds_job(const volume &vol, seeded_segmentation_params params) {
  return make_result_job(std::make_unique<critical_point_seeds_stepper>(vol, params));
}

} // namespace cvc
