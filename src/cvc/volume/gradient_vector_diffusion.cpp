/*
  Copyright 2003, 2026 The University of Texas at Austin

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

// cvc::gradient_vector_diffusion (cvc/volume/gradient_vector_diffusion.h): SegMed's gvfflow()
// (transfix/volrover src/Segmentation/SegMed/GVF.cpp, at the sha in
// tools/volrover-goldens/volrover.sha) as a stepper. Its result equals the legacy code's
// (src/cvc/tests/gradient_vector_diffusion_test.cpp, against tools/volrover-goldens' segmed
// goldens). What changed:
//   - the globals (velocity, XDIM, ...) are the stepper's members, and the legacy six exit(0)s on
//     a failed malloc are gone (std::vector throws std::bad_alloc);
//   - the six hand-written neighbour blocks are one function, computing in the legacy order
//     (the sums of weights and of moves added in the same order), and exp() is computed in double,
//     which rounds to the same float on every platform's libm;
//   - which field weighs the neighbours is a parameter: SegMed's is the field before diffusion (the
//     legacy `velocity`, never updated inside the loop), SecStruct's the field being diffused;
//   - each iteration swaps the two buffers instead of copying the new field back;
//   - a field of zero vectors is left alone by the normalisation (the legacy code divided by 0).
// Built with -ffp-contract=off (/fp:precise on MSVC): an FMA would change the field's last bits.

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cvc/volume/gradient_vector_diffusion.h>
#include <cvc/volume/gradient_vector_diffusion_params.h>
#include <exception>
#include <optional>
#include <string>
#include <utility>

namespace cvc {

namespace {

// Units between two looks at cancel() inside one step.
constexpr std::size_t slice_units = 4096;

} // namespace

struct gradient_vector_diffusion_stepper::impl {
  enum class stage { checking, diffusing, finding_max, dividing, finished };

  gradient_vector_diffusion_params params;
  vector_field cur;  // the field being diffused
  vector_field next; // the iteration's output
  vector_field init; // the weighing field with diffusion_weights::initial
  stage at = stage::checking;
  std::size_t v = 0;       // the voxel the current loop is at
  int i = 0, j = 0, k = 0; // its coordinates (diffusing)
  int iteration = 0;
  float kapa = 2.0f;
  float maxsq = 0.0f; // the largest squared length (normalising)
  float maxlen = 0.0f;

  std::atomic<bool> cancel{false};
  bool failed = false;
  std::exception_ptr error;
  std::size_t units = 0;
  double prog = 0.0;
  std::optional<vector_field> result;

  impl(vector_field f, gradient_vector_diffusion_params p) : params(p), cur(std::move(f)) {}

  std::size_t n() const { return cur.size(); }
  bool unit(); // one unit; true once the routine is done
  void begin_diffusion();
  void update_voxel();
  const char *phase() const {
    if (failed)
      return "failed";
    switch (at) {
    case stage::checking:
      return "checking";
    case stage::diffusing:
      return "diffusing";
    case stage::finding_max:
    case stage::dividing:
      return "normalising";
    case stage::finished:
      return "done";
    }
    return "done";
  }
  void note_progress() {
    const double nn = double(n() ? n() : 1);
    const double total = nn * (1.0 + double(params.iterations) + (params.normalize ? 2.0 : 0.0));
    prog = std::max(prog, std::min(1.0, double(units) / total));
  }
};

void gradient_vector_diffusion_stepper::impl::begin_diffusion() {
  kapa = float(params.k);
  if (params.weights == diffusion_weights::initial)
    init = cur;
  next = vector_field(cur.nx, cur.ny, cur.nz);
  at = stage::diffusing;
  v = 0;
  i = j = k = 0;
  iteration = 0;
}

// One voxel of one iteration of gvfflow().
void gradient_vector_diffusion_stepper::impl::update_voxel() {
  const int X = cur.nx, Y = cur.ny, Z = cur.nz;
  const std::size_t sy = std::size_t(X), sz = std::size_t(X) * std::size_t(Y);
  const std::size_t xp = i < X - 1 ? v + 1 : v, xm = i > 0 ? v - 1 : v;
  const std::size_t yp = j < Y - 1 ? v + sy : v, ym = j > 0 ? v - sy : v;
  const std::size_t zp = k < Z - 1 ? v + sz : v, zm = k > 0 ? v - sz : v;
  const vector_field &w = params.weights == diffusion_weights::initial ? init : cur;
  const float *u = cur.x.data(), *vv = cur.y.data(), *ww = cur.z.data();
  const float cx = w.x[v], cy = w.y[v], cz = w.z[v];
  const float tmp = std::sqrt(cx * cx + cy * cy + cz * cz);
  if (tmp == 0) {
    next.x[v] = (u[xp] + u[xm] + u[yp] + u[ym] + u[zp] + u[zm]) / 6.0f;
    next.y[v] = (vv[xp] + vv[xm] + vv[yp] + vv[ym] + vv[zp] + vv[zm]) / 6.0f;
    next.z[v] = (ww[xp] + ww[xm] + ww[yp] + ww[ym] + ww[zp] + ww[zm]) / 6.0f;
    return;
  }
  // Eq. 3: exp(k (cos a - 1)), 0 when the neighbour's vector is zero.
  auto weight = [&](std::size_t n) -> float {
    const float nx = w.x[n], ny = w.y[n], nz = w.z[n];
    const float temp = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (temp == 0)
      return 0.0f;
    const float arg = kapa * ((cx * nx + cy * ny + cz * nz) / (tmp * temp) - 1);
    return (float)std::exp(double(arg));
  };
  float up = weight(zp), down = weight(zm), left = weight(ym), right = weight(yp);
  float back = weight(xm), front = weight(xp);
  const float sum = right + left + up + down + front + back;
  if (sum != 0) {
    right /= sum;
    left /= sum;
    up /= sum;
    down /= sum;
    front /= sum;
    back /= sum;
  }
  auto move = [&](const float *c) {
    const float c0 = c[v];
    return c0 + front * (c[xp] - c0) + back * (c[xm] - c0) + right * (c[yp] - c0) +
           left * (c[ym] - c0) + up * (c[zp] - c0) + down * (c[zm] - c0);
  };
  next.x[v] = move(u);
  next.y[v] = move(vv);
  next.z[v] = move(ww);
}

bool gradient_vector_diffusion_stepper::impl::unit() {
  switch (at) {
  case stage::checking: {
    if (v == 0) {
      const std::vector<std::string> problems = params_validate(params);
      if (!problems.empty()) {
        std::string m = "gradient_vector_diffusion: ";
        for (std::size_t p = 0; p < problems.size(); ++p)
          m += (p ? "; " : "") + problems[p];
        throw gradient_vector_diffusion_error(m);
      }
      if (cur.nx < 0 || cur.ny < 0 || cur.nz < 0)
        throw gradient_vector_diffusion_error("gradient_vector_diffusion: a negative dimension");
      const std::size_t want = cur.size();
      if (cur.x.size() != want || cur.y.size() != want || cur.z.size() != want)
        throw gradient_vector_diffusion_error(
            "gradient_vector_diffusion: the components hold " + std::to_string(cur.x.size()) +
            ", " + std::to_string(cur.y.size()) + " and " + std::to_string(cur.z.size()) +
            " values, not the " + std::to_string(cur.nx) + "x" + std::to_string(cur.ny) + "x" +
            std::to_string(cur.nz) + " grid's " + std::to_string(want));
      if (want == 0) {
        at = stage::finished;
        return true;
      }
    }
    if (!std::isfinite(cur.x[v]) || !std::isfinite(cur.y[v]) || !std::isfinite(cur.z[v])) {
      const std::size_t X = std::size_t(cur.nx), Y = std::size_t(cur.ny);
      char buf[160];
      std::snprintf(buf, sizeof buf,
                    "gradient_vector_diffusion: the vector at (%zu, %zu, %zu) is not finite", v % X,
                    (v / X) % Y, v / (X * Y));
      throw gradient_vector_diffusion_error(buf);
    }
    if (++v == n()) {
      if (params.iterations > 0) {
        begin_diffusion();
      } else {
        at = params.normalize ? stage::finding_max : stage::finished;
        v = 0;
        maxsq = 0.0f;
      }
    }
    return at == stage::finished;
  }
  case stage::diffusing: {
    update_voxel();
    ++v;
    if (++i == cur.nx) {
      i = 0;
      if (++j == cur.ny) {
        j = 0;
        ++k;
      }
    }
    if (v == n()) {
      std::swap(cur, next);
      v = 0;
      i = j = k = 0;
      if (++iteration == params.iterations) {
        next = vector_field();
        init = vector_field();
        at = params.normalize ? stage::finding_max : stage::finished;
        maxsq = 0.0f;
      }
    }
    return at == stage::finished;
  }
  case stage::finding_max: {
    const float g = cur.x[v] * cur.x[v] + cur.y[v] * cur.y[v] + cur.z[v] * cur.z[v];
    if (g > maxsq)
      maxsq = g;
    if (++v == n()) {
      maxlen = std::sqrt(maxsq);
      v = 0;
      at = maxlen > 0 ? stage::dividing : stage::finished;
    }
    return at == stage::finished;
  }
  case stage::dividing: {
    cur.x[v] = cur.x[v] / maxlen;
    cur.y[v] = cur.y[v] / maxlen;
    cur.z[v] = cur.z[v] / maxlen;
    if (++v == n())
      at = stage::finished;
    return at == stage::finished;
  }
  case stage::finished:
    return true;
  }
  return true;
}

gradient_vector_diffusion_stepper::gradient_vector_diffusion_stepper(
    vector_field initial, gradient_vector_diffusion_params params)
    : _impl(std::make_unique<impl>(std::move(initial), params)) {}

gradient_vector_diffusion_stepper::~gradient_vector_diffusion_stepper() = default;
gradient_vector_diffusion_stepper::gradient_vector_diffusion_stepper(
    gradient_vector_diffusion_stepper &&) noexcept = default;
gradient_vector_diffusion_stepper &gradient_vector_diffusion_stepper::operator=(
    gradient_vector_diffusion_stepper &&) noexcept = default;

bool gradient_vector_diffusion_stepper::step(std::size_t budget) {
  if (!_impl)
    return true;
  impl &m = *_impl;
  if (m.failed) {
    if (m.cancel.load(std::memory_order_acquire))
      return true;
    std::rethrow_exception(m.error);
  }
  if (m.result)
    return true;
  try {
    std::size_t left = budget == 0 ? SIZE_MAX : budget;
    while (left > 0) {
      if (m.cancel.load(std::memory_order_acquire))
        break;
      const std::size_t slice = std::min(left, slice_units);
      for (std::size_t s = 0; s < slice; ++s) {
        ++m.units;
        if (m.unit()) {
          m.result.emplace(std::move(m.cur));
          m.cur = vector_field();
          m.prog = 1.0;
          return true;
        }
      }
      if (left != SIZE_MAX)
        left -= slice;
      m.note_progress();
    }
    if (m.cancel.load(std::memory_order_acquire)) {
      m.error = std::make_exception_ptr(
          gradient_vector_diffusion_error("gradient_vector_diffusion: cancelled"));
      m.failed = true;
      m.cur = m.next = m.init = vector_field();
      return true;
    }
    return false;
  } catch (...) {
    m.error = std::current_exception();
    m.failed = true;
    m.cur = m.next = m.init = vector_field();
    throw;
  }
}

bool gradient_vector_diffusion_stepper::done() const {
  return !_impl || _impl->result.has_value() || _impl->failed;
}
const char *gradient_vector_diffusion_stepper::phase() const {
  return _impl ? _impl->phase() : "done";
}
double gradient_vector_diffusion_stepper::progress() const { return _impl ? _impl->prog : 1.0; }
std::size_t gradient_vector_diffusion_stepper::units() const { return _impl ? _impl->units : 0; }

std::string gradient_vector_diffusion_stepper::status() const {
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
  char buf[160];
  if (m.result) {
    std::snprintf(buf, sizeof buf, "done: %dx%dx%d field, %d iterations", m.result->nx,
                  m.result->ny, m.result->nz, m.params.iterations);
    return buf;
  }
  if (m.at == impl::stage::diffusing)
    std::snprintf(buf, sizeof buf, "diffusing (iteration %d of %d) %d%%", m.iteration + 1,
                  m.params.iterations, int(m.prog * 100));
  else
    std::snprintf(buf, sizeof buf, "%s %d%%", m.phase(), int(m.prog * 100));
  return buf;
}

void gradient_vector_diffusion_stepper::cancel() {
  if (_impl)
    _impl->cancel.store(true, std::memory_order_release);
}

vector_field gradient_vector_diffusion_stepper::take() {
  if (_impl && _impl->failed)
    std::rethrow_exception(_impl->error);
  if (!_impl || !_impl->result)
    throw gradient_vector_diffusion_error(
        "gradient_vector_diffusion: take() before the stepper is done");
  vector_field r = std::move(*_impl->result);
  _impl.reset();
  return r;
}

vector_field gradient_vector_diffusion(vector_field initial,
                                       const gradient_vector_diffusion_params &params) {
  gradient_vector_diffusion_stepper s(std::move(initial), params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<vector_field>>
gradient_vector_diffusion_job(vector_field initial, gradient_vector_diffusion_params params) {
  return make_result_job(
      std::make_unique<gradient_vector_diffusion_stepper>(std::move(initial), params));
}

critical_kind critical_point_at(const vector_field &f, int i, int j, int k) {
  const std::size_t c = f.index(i, j, k);
  bool nonzero = false, towards = false, away = false;
  for (int l = std::max(k - 1, 0); l <= std::min(k + 1, f.nz - 1); l++)
    for (int n = std::max(j - 1, 0); n <= std::min(j + 1, f.ny - 1); n++)
      for (int m = std::max(i - 1, 0); m <= std::min(i + 1, f.nx - 1); m++) {
        if (m == i && n == j && l == k)
          continue;
        const std::size_t q = f.index(m, n, l);
        const float d = (m - i) * f.x[q] + (n - j) * f.y[q] + (l - k) * f.z[q];
        if (d != 0)
          nonzero = true;
        if (d < 0)
          towards = true;
        else if (d > 0)
          away = true;
      }
  if (f.x[c] == 0 && f.y[c] == 0 && f.z[c] == 0 && !nonzero)
    return critical_kind::none;
  if (!towards)
    return critical_kind::minimum;
  if (!away)
    return critical_kind::maximum;
  return critical_kind::none;
}

} // namespace cvc
