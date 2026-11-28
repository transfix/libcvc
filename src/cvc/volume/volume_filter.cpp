/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Filters on a volume the app holds. See inc/cvc/volume/volume_filter.h.
//
// Each filter is an engine that does one row of voxels (nx of them) per unit of work, so a step is
// whole rows. The contrast, anisotropic and GDTV engines are VolMagick's
// voxels::contrastEnhancement, voxels::anisotropicDiffusion and voxels::gdtvFilter
// (src/cvc/processing/) cut into rows: the same expressions on the same voxel types, in the same
// order where the order matters, so the result is theirs voxel for voxel (volume_filter_test checks
// it on every voxel type it runs). This file and those are built without FMA contraction
// (src/cvc/CMakeLists.txt), so no compiler fuses one side's multiply-adds differently from the
// other's.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_filter.h>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {

namespace {

// What every filter's engine has: its input, the rows, and its count of rows done out of all.
struct filter_engine {
  std::shared_ptr<const volume> in;
  std::int64_t nx = 0, ny = 0, nz = 0, rows = 0; // rows: ny * nz
  std::uint64_t done_rows = 0, total_rows = 1;   // over every pass, for progress()
  bool finished = false;

  explicit filter_engine(std::shared_ptr<const volume> v) : in(std::move(v)) {
    nx = static_cast<std::int64_t>(in->XDim());
    ny = static_cast<std::int64_t>(in->YDim());
    nz = static_cast<std::int64_t>(in->ZDim());
    rows = ny * nz;
  }
  virtual ~filter_engine() = default;

  // One row of the current pass; sets `finished` after the last.
  virtual void row() = 0;
  virtual const char *phase() const = 0;
  virtual std::string summary() const = 0; // status() once done
  virtual std::shared_ptr<volume> result() = 0;

  bool step(std::size_t budget) {
    std::size_t voxels = 0;
    while (!finished) {
      row();
      ++done_rows;
      voxels += static_cast<std::size_t>(nx);
      if (budget && voxels >= budget)
        break;
    }
    return finished;
  }
  double progress() const {
    if (finished)
      return 1.0;
    return std::min(1.0, static_cast<double>(done_rows) / static_cast<double>(total_rows));
  }
  std::string dims() const {
    return std::to_string(nx) + " x " + std::to_string(ny) + " x " + std::to_string(nz) + " voxels";
  }
};

// A volume of its own: the input's grid and description, its voxels deep-copied, so writing them
// never touches the input's (which other nodes are showing).
std::shared_ptr<volume> own_copy(const volume &in) {
  auto out = std::make_shared<volume>(in);
  out->voxels::copy();
  return out;
}

// ---- bilateral ----------------------------------------------------------------------------------

struct bilateral_engine final : filter_engine {
  std::shared_ptr<volume> out;
  std::int64_t next = 0; // the next row (j + k * ny)
  // The weights, as voxels::bilateralFilter makes them.
  int r = 2, diam = 5;
  double range = 0.0;
  double radiometric[256] = {};
  std::vector<double> spatial;

  bilateral_engine(std::shared_ptr<const volume> v, const volume_filter_params &p)
      : filter_engine(std::move(v)) {
    out = own_copy(*in);
    total_rows = static_cast<std::uint64_t>(rows);
    r = static_cast<int>(p.radius);
    diam = 2 * r + 1;
    range = in->max() - in->min();
    if (!(range > 0.0)) { // constant: nothing to blend, and no scale to blend on
      finished = true;
      return;
    }
    for (int c = 0; c < 256; ++c) {
      const double f = c * (range / 255.0);
      radiometric[c] = std::exp(f * f / (-p.radiometric_sigma * p.radiometric_sigma * 2.0));
    }
    spatial.reserve(static_cast<std::size_t>(diam) * diam * diam);
    for (int k = -r; k <= r; ++k)
      for (int j = -r; j <= r; ++j)
        for (int i = -r; i <= r; ++i)
          spatial.push_back(std::exp(static_cast<double>(k * k + j * j + i * i) /
                                     (-p.spatial_sigma * p.spatial_sigma * 2.0)));
  }

  // voxels::bilateralFilter's loop for (j, k).
  void row() override {
    const std::int64_t j = next % ny, k = next / ny;
    const volume &src = *in;
    for (std::int64_t i = 0; i < nx; ++i) {
      const double fsample = src(i, j, k);
      double sum = 0.0, denom = 0.0;
      for (int z = 0; z < diam; ++z) {
        const std::int64_t kk = k + z - r;
        if (kk < 0 || kk >= nz)
          continue;
        for (int y = 0; y < diam; ++y) {
          const std::int64_t jj = j + y - r;
          if (jj < 0 || jj >= ny)
            continue;
          for (int x = 0; x < diam; ++x) {
            const std::int64_t ii = i + x - r;
            if (ii < 0 || ii >= nx)
              continue;
            const double v = src(ii, jj, kk);
            const double nd = std::fabs(fsample - v) / range * 255.0;
            const int c = nd < 255.0 ? static_cast<int>(nd) : 255; // NaN too
            const double w = radiometric[c] * spatial[(z * diam + y) * diam + x];
            denom += w;
            sum += w * v;
          }
        }
      }
      (*out)(i, j, k, sum / denom);
    }
    if (++next >= rows) {
      out->unsetMinMax(); // its range is the filtered voxels', taken when asked
      finished = true;
    }
  }
  const char *phase() const override { return "bilateral"; }
  std::string summary() const override {
    return "bilateral: " + dims() + ", radius " + std::to_string(r);
  }
  std::shared_ptr<volume> result() override { return std::move(out); }
};

// ---- contrast enhancement -----------------------------------------------------------------------

// voxels::contrastEnhancement (src/cvc/processing/contrast_enhancement.cpp), a row at a time:
//   mapping     the volume mapped to [0, 255] (voxels::map), and copied into the five working
//               volumes (the legacy's upmin, upmax, downmin, downmax, imgavg);
//   up, down    each slice in turn (bottom to top, then top to bottom): the local minima, maxima
//               and means carried over from the slice before, row by row, then
//               contrastEnhancementSlice's sweeps of that slice, row by row: its working rows
//               taken in, the bottom-up sweeps, the top-down sweeps, and the minima and maxima put
//               back;
//   stretching  each voxel stretched between its local minimum and maximum;
//   restoring   the stretched volume's range found (voxels::calcMinMax's scan), then the volume
//               mapped back to the input's range (voxels::map).
// Every working value is written in the volume's voxel type, as the legacy's are.
struct contrast_engine final : filter_engine {
  enum class pass { mapping, up, down, stretching, scanning, restoring };
  enum class part { carry, take_in, bottom_up, top_down, put_back };
  pass at = pass::mapping;
  part sub = part::take_in;
  std::int64_t k = 0;    // the slice (up and down)
  std::int64_t done = 0; // rows done in the current pass or part
  double resistor = 0.95;
  std::shared_ptr<volume> img; // the result, mapped and restored in place
  std::shared_ptr<volume> upmin, upmax, downmin, downmax, imgavg;
  voxels tmpmin, tmpmax, lcmin, lcmax; // one slice each
  double origmin = 0.0, origmax = 0.0;
  bool flat = false;         // the input is constant: voxels::map fills with the middle
  double scale = 1.0;        // mapping's
  double lo = 0.0, hi = 0.0; // the stretched volume's range (scanning)
  double back = 1.0;         // restoring's scale
  bool flat_back = false;

  contrast_engine(std::shared_ptr<const volume> v, const volume_filter_params &p)
      : filter_engine(std::move(v)), tmpmin(in->ctx(), dimension(nx, ny, 1), in->voxelType()),
        tmpmax(in->ctx(), dimension(nx, ny, 1), in->voxelType()),
        lcmin(in->ctx(), dimension(nx, ny, 1), in->voxelType()),
        lcmax(in->ctx(), dimension(nx, ny, 1), in->voxelType()) {
    resistor = std::min(1.0, std::max(0.0, p.contrast_resistor));
    img = own_copy(*in);
    upmin = own_copy(*in);
    upmax = own_copy(*in);
    downmin = own_copy(*in);
    downmax = own_copy(*in);
    imgavg = own_copy(*in);
    // The range of our own copy: the input's (its copy carries a range the input was given, as the
    // legacy's this->min() would read it), without touching the shared input's cache.
    origmin = img->min();
    origmax = img->max();
    const double range = origmax - origmin;
    flat = range == 0.0;
    if (!flat)
      scale = (255.0 - 0.0) / range;
    // mapping, scanning, stretching and restoring take a row each; each slice of up and down takes
    // ny for each of its sweeps (take_in, bottom_up, top_down, put_back), and ny more to carry the
    // slice before over, but the first.
    const std::uint64_t r = static_cast<std::uint64_t>(rows), y = static_cast<std::uint64_t>(ny),
                        z = static_cast<std::uint64_t>(nz);
    total_rows = 4 * r + 2 * (4 * y * z + y * (z - 1));
  }

  // The mapping's value for v (voxels::map(0, 255)).
  double mapped(double v) const { return 0.0 + (v - origmin) * scale; }

  void map_row(std::int64_t j, std::int64_t kk) {
    for (std::int64_t i = 0; i < nx; ++i) {
      const double nv = flat ? (0.0 + 255.0) / 2.0 : mapped((*img)(i, j, kk));
      (*img)(i, j, kk, nv);
      const double w = (*img)(i, j, kk); // as the voxel type holds it, as the legacy's copies do
      (*upmin)(i, j, kk, w);
      (*upmax)(i, j, kk, w);
      (*downmin)(i, j, kk, w);
      (*downmax)(i, j, kk, w);
      (*imgavg)(i, j, kk, w);
    }
  }

  // The slice before's local values carried over (contrastEnhancement's "propagation from lower
  // slice" / "from upper slice"), for row j of slice k, from slice `from`.
  void carry_row(volume &mn, volume &mx, std::int64_t j, std::int64_t from) {
    volume &avg = *imgavg;
    const double r = resistor;
    for (std::int64_t i = 0; i < nx; ++i) {
      avg(i, j, k, avg(i, j, k) + r * (avg(i, j, from) - avg(i, j, k)));
      if (mn(i, j, from) < mn(i, j, k))
        mn(i, j, k, mn(i, j, k) + r * (mn(i, j, from) - mn(i, j, k)));
      if (mx(i, j, from) > mx(i, j, k))
        mx(i, j, k, mx(i, j, k) + r * (mx(i, j, from) - mx(i, j, k)));
    }
  }

  // contrastEnhancementSlice's sweeps along a row of slice k: a forward sweep (each voxel from the
  // one before it) and a backward one, on the mean and on the slice's minima and maxima `mn`, `mx`
  // (tmpmin/tmpmax or lcmin/lcmax).
  void along_row(voxels &mn, voxels &mx, std::int64_t j) {
    volume &avg = *imgavg;
    const double r = resistor;
    const int xdim = static_cast<int>(nx);
    for (int i = 1; i < xdim; i++) {
      avg(i, j, k, avg(i, j, k) + r * (avg(i - 1, j, k) - avg(i, j, k)));
      if (mn(i - 1, j, 0) < mn(i, j, 0))
        mn(i, j, 0, mn(i, j, 0) + r * (mn(i - 1, j, 0) - mn(i, j, 0)));
      if (mx(i - 1, j, 0) > mx(i, j, 0))
        mx(i, j, 0, mx(i, j, 0) + r * (mx(i - 1, j, 0) - mx(i, j, 0)));
    }
    for (int i = xdim - 2; i >= 0; i--) {
      avg(i, j, k, avg(i, j, k) + r * (avg(i + 1, j, k) - avg(i, j, k)));
      if (mn(i + 1, j, 0) < mn(i, j, 0))
        mn(i, j, 0, mn(i, j, 0) + r * (mn(i + 1, j, 0) - mn(i, j, 0)));
      if (mx(i + 1, j, 0) > mx(i, j, 0))
        mx(i, j, 0, mx(i, j, 0) + r * (mx(i + 1, j, 0) - mx(i, j, 0)));
    }
  }

  // contrastEnhancementSlice's sweep across rows, into row j from row `from` (j - 1 bottom-up,
  // j + 1 top-down).
  void across_row(voxels &mn, voxels &mx, std::int64_t j, std::int64_t from) {
    volume &avg = *imgavg;
    const double r = resistor;
    for (std::int64_t i = 0; i < nx; i++) {
      avg(i, j, k, avg(i, j, k) + r * (avg(i, from, k) - avg(i, j, k)));
      if (mn(i, from, 0) < mn(i, j, 0))
        mn(i, j, 0, mn(i, j, 0) + r * (mn(i, from, 0) - mn(i, j, 0)));
      if (mx(i, from, 0) > mx(i, j, 0))
        mx(i, j, 0, mx(i, j, 0) + r * (mx(i, from, 0) - mx(i, j, 0)));
    }
  }

  // One row of slice k's current part, on the pass's minima `mn` and maxima `mx`.
  void slice_row(volume &mn, volume &mx) {
    switch (sub) {
    case part::carry: // only past the pass's first slice
      carry_row(mn, mx, done, at == pass::up ? k - 1 : k + 1);
      break;
    case part::take_in:
      for (std::int64_t i = 0; i < nx; i++) {
        lcmin(i, done, 0, mn(i, done, k));
        lcmax(i, done, 0, mx(i, done, k));
        tmpmin(i, done, 0, mn(i, done, k));
        tmpmax(i, done, 0, mx(i, done, k));
      }
      break;
    case part::bottom_up: // row 0, then rows 1.. from the row below
      if (done > 0)
        across_row(tmpmin, tmpmax, done, done - 1);
      along_row(tmpmin, tmpmax, done);
      break;
    case part::top_down: { // row ny - 1, then rows below it from the row above
      const std::int64_t j = ny - 1 - done;
      if (j < ny - 1)
        across_row(lcmin, lcmax, j, j + 1);
      along_row(lcmin, lcmax, j);
      break;
    }
    case part::put_back:
      for (std::int64_t i = 0; i < nx; i++) {
        mn(i, done, k, std::min(lcmin(i, done, 0), tmpmin(i, done, 0)));
        mx(i, done, k, std::max(lcmax(i, done, 0), tmpmax(i, done, 0)));
      }
      break;
    }
    if (++done < ny)
      return;
    done = 0;
    switch (sub) {
    case part::carry:
      sub = part::take_in;
      return;
    case part::take_in:
      sub = part::bottom_up;
      return;
    case part::bottom_up:
      sub = part::top_down;
      return;
    case part::top_down:
      sub = part::put_back;
      return;
    case part::put_back:
      break;
    }
    // The slice is done: the next one, or the next pass.
    const bool up = at == pass::up;
    k += up ? 1 : -1;
    sub = part::carry;
    if (up && k >= nz) {
      at = pass::down;
      k = nz - 1;
      sub = part::take_in; // the first slice of a pass carries nothing over
    } else if (!up && k < 0) {
      at = pass::stretching;
    }
  }

  // contrastEnhancement's stretching of one voxel row.
  void stretch_row(std::int64_t j, std::int64_t kk) {
    double lmin, lmax, im, avg, window, a, b, c, alpha;
    for (std::int64_t i = 0; i < nx; i++) {
      lmin = std::min((*upmin)(i, j, kk), (*downmin)(i, j, kk));
      lmax = std::max((*upmax)(i, j, kk), (*downmax)(i, j, kk));
      im = (*img)(i, j, kk);
      avg = (*imgavg)(i, j, kk);
      window = lmax - lmin;
      window = std::sqrt(window * (510 - window));

      if (lmin != lmax) {
        im = window * (im - lmin) / (lmax - lmin);
        avg = window * (avg - lmin) / (lmax - lmin);
      }

      alpha = (avg - im) / (181.019 * window);
      if (alpha != 0) {
        a = 0.707 * alpha;
        b = 1.414 * alpha * (im - window) - 1;
        c = 0.707 * alpha * im * (im - 2 * window) + im;
        double discriminant = b * b - 4 * a * c;
        if (discriminant >= 0)
          (*imgavg)(i, j, kk, lmin + (-b - std::sqrt(discriminant)) / (2 * a));
        else
          (*imgavg)(i, j, kk, im + lmin); // the legacy's fallback for a negative discriminant
      } else
        (*imgavg)(i, j, kk, im + lmin);
    }
  }

  // voxels::calcMinMax's scan (scan_min_max) of one row: from voxel 0, a value replaces the lowest
  // when below it, the highest when above (so a NaN is skipped anywhere but voxel 0).
  void scan_row(std::int64_t j, std::int64_t kk) {
    for (std::int64_t i = 0; i < nx; i++) {
      const double v = (*imgavg)(i, j, kk);
      if (j == 0 && kk == 0 && i == 0) {
        lo = hi = v;
        continue;
      }
      if (v < lo)
        lo = v;
      if (v > hi)
        hi = v;
    }
  }

  void restore_row(std::int64_t j, std::int64_t kk) {
    for (std::int64_t i = 0; i < nx; i++) {
      const double nv =
          flat_back ? (origmin + origmax) / 2.0 : origmin + ((*imgavg)(i, j, kk) - lo) * back;
      (*img)(i, j, kk, nv);
    }
  }

  void row() override {
    const std::int64_t j = done % ny, kk = done / ny;
    switch (at) {
    case pass::mapping:
      map_row(j, kk);
      if (++done >= rows) {
        done = 0;
        at = pass::up;
        k = 0;
        sub = part::take_in; // slice 0 carries nothing over
      }
      return;
    case pass::up:
      slice_row(*upmin, *upmax);
      return;
    case pass::down:
      slice_row(*downmin, *downmax);
      return;
    case pass::stretching:
      stretch_row(j, kk);
      if (++done >= rows) {
        done = 0;
        at = pass::scanning;
      }
      return;
    case pass::scanning:
      scan_row(j, kk);
      if (++done >= rows) {
        done = 0;
        at = pass::restoring;
        const double range = hi - lo; // voxels::map(origmin, origmax) on the stretched volume
        flat_back = range == 0.0;
        if (!flat_back)
          back = (origmax - origmin) / range;
      }
      return;
    case pass::restoring:
      restore_row(j, kk);
      if (++done >= rows) {
        // voxels::map leaves the range it mapped to as the volume's
        img->min(origmin);
        img->max(origmax);
        upmin.reset();
        upmax.reset();
        downmin.reset();
        downmax.reset();
        imgavg.reset();
        finished = true;
      }
      return;
    }
  }

  const char *phase() const override {
    switch (at) {
    case pass::mapping:
      return "contrast: mapping";
    case pass::up:
      return "contrast: up";
    case pass::down:
      return "contrast: down";
    case pass::stretching:
      return "contrast: stretching";
    case pass::scanning:
    case pass::restoring:
      break;
    }
    return "contrast: restoring";
  }
  std::string summary() const override {
    char r[32];
    std::snprintf(r, sizeof r, "%g", resistor);
    return "contrast: " + dims() + ", resistor " + r;
  }
  std::shared_ptr<volume> result() override { return std::move(img); }
};

// ---- anisotropic diffusion ----------------------------------------------------------------------

// voxels::anisotropicDiffusion's CPU path (src/cvc/processing/anisotropic_diffusion.cpp), a row at
// a time: each iteration reads `cur` and writes every voxel of `next` (the legacy's tempt), then
// the two trade places, as its copy back does.
struct anisotropic_engine final : filter_engine {
  std::shared_ptr<volume> cur, next;
  std::int64_t iterations = 20, m = 0, at = 0; // the iteration, and the next row in it

  anisotropic_engine(std::shared_ptr<const volume> v, const volume_filter_params &p)
      : filter_engine(std::move(v)) {
    iterations = p.anisotropic_iterations;
    cur = own_copy(*in);
    next = own_copy(*in);
    total_rows = static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(iterations);
  }

  void row() override {
    const double K_para = 3;
    const double Lamda_para = 0.16f;
    const std::int64_t j = at % ny, k = at / ny;
    const volume &src = *cur;
    volume &dst = *next;
    double cn, cs, ce, cw, cu, cd;
    double delta_n, delta_s, delta_e, delta_w, delta_u, delta_d;
    for (std::int64_t i = 0; i < nx; i++) {
      if (j < ny - 1)
        delta_s = src(i, j + 1, k) - src(i, j, k);
      else
        delta_s = 0.0;
      if (j > 0)
        delta_n = src(i, j - 1, k) - src(i, j, k);
      else
        delta_n = 0.0;
      if (i < nx - 1)
        delta_e = src(i + 1, j, k) - src(i, j, k);
      else
        delta_e = 0.0;
      if (i > 0)
        delta_w = src(i - 1, j, k) - src(i, j, k);
      else
        delta_w = 0.0;
      if (k < nz - 1)
        delta_u = src(i, j, k + 1) - src(i, j, k);
      else
        delta_u = 0.0;
      if (k > 0)
        delta_d = src(i, j, k - 1) - src(i, j, k);
      else
        delta_d = 0.0;

      cn = 1.0f / (1.0f + ((delta_n * delta_n) / (K_para * K_para)));
      cs = 1.0f / (1.0f + ((delta_s * delta_s) / (K_para * K_para)));
      ce = 1.0f / (1.0f + ((delta_e * delta_e) / (K_para * K_para)));
      cw = 1.0f / (1.0f + ((delta_w * delta_w) / (K_para * K_para)));
      cu = 1.0f / (1.0f + ((delta_u * delta_u) / (K_para * K_para)));
      cd = 1.0f / (1.0f + ((delta_d * delta_d) / (K_para * K_para)));

      dst(i, j, k,
          src(i, j, k) + Lamda_para * (cn * delta_n + cs * delta_s + ce * delta_e + cw * delta_w +
                                       cu * delta_u + cd * delta_d));
    }
    if (++at < rows)
      return;
    at = 0;
    std::swap(cur, next);
    if (++m >= iterations) {
      next.reset();
      cur->unsetMinMax(); // its range is the diffused voxels', taken when asked
      finished = true;
    }
  }
  const char *phase() const override { return "anisotropic"; }
  std::string summary() const override {
    return "anisotropic: " + dims() + ", " + std::to_string(iterations) + " iterations";
  }
  std::shared_ptr<volume> result() override { return std::move(cur); }
};

// ---- GDTV ---------------------------------------------------------------------------------------

float gdtv_phi(float x, float q) { return ((2 - q) * std::pow(x, 1 - q)); }

// voxels::gdtvFilter's CPU path (src/cvc/processing/gdtv_filter.cpp), a row at a time:
//   gradient    the gradient magnitude of the input, as floats (Gradient, or Gradient2 over 26
//               neighbours);
//   filtering   each iteration (filtGDTV / filtGDTV2) computes every voxel from the last
//               iteration's (`cur`) and the input into `next`, as a float first, as the legacy's
//               funcvalue holds it, then in the volume's voxel type, as its copy back writes it;
//               then the two trade places.
struct gdtv_engine final : filter_engine {
  bool gradient_done = false;
  bool full = false; // twenty-six neighbours
  double exponent = 1.0;
  float q = 1.0f, lbda = 1e-4f;
  std::int64_t iterations = 1, m = 0, at = 0;
  std::vector<float> grad; // the gradient magnitude, as the legacy's Float volume holds it
  std::shared_ptr<volume> cur, next;

  gdtv_engine(std::shared_ptr<const volume> v, const volume_filter_params &p)
      : filter_engine(std::move(v)) {
    full = p.gdtv_neighbours == volume_gdtv_neighbours::twenty_six;
    // VolumeRover2's dialog calls gdtvFilter(2.0 - exponent, lambda, ...); both reach filtGDTV as
    // floats.
    exponent = p.gdtv_exponent;
    q = static_cast<float>(2.0 - p.gdtv_exponent);
    lbda = static_cast<float>(p.gdtv_lambda);
    iterations = p.gdtv_iterations;
    grad.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(nx), 0.0f);
    cur = own_copy(*in);
    next = own_copy(*in);
    total_rows = static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(iterations + 1);
  }

  std::size_t idx(std::int64_t i, std::int64_t j, std::int64_t k) const {
    return static_cast<std::size_t>(i + nx * (j + ny * k));
  }
  double G(std::int64_t i, std::int64_t j, std::int64_t k) const { return grad[idx(i, j, k)]; }

  // Gradient's six differences, or Gradient2's twenty-six, for row (j, k).
  void gradient_row(std::int64_t j, std::int64_t k) {
    const volume &input = *in;
    const std::int64_t X = nx, Y = ny, Z = nz;
    for (std::int64_t i = 0; i < nx; i++) {
      double sum = 0.0;
      if (!full) {
        float val[6];
        for (int l = 0; l < 6; l++)
          val[l] = 0.0;
        if (i + 1 < X)
          val[0] = input(i + 1, j, k) - input(i, j, k);
        if (i >= 1)
          val[1] = input(i - 1, j, k) - input(i, j, k);
        if (j + 1 < Y)
          val[2] = input(i, j + 1, k) - input(i, j, k);
        if (j >= 1)
          val[3] = input(i, j - 1, k) - input(i, j, k);
        if (k + 1 < Z)
          val[4] = input(i, j, k + 1) - input(i, j, k);
        if (k >= 1)
          val[5] = input(i, j, k - 1) - input(i, j, k);
        for (int l = 0; l < 6; l++)
          sum += val[l] * val[l];
      } else {
        float val[26];
        for (int l = 0; l < 26; l++)
          val[l] = 0.0;
        if (i + 1 < X) {
          val[0] = input(i + 1, j, k) - input(i, j, k);
          if (j + 1 < Y) {
            val[6] = input(i + 1, j + 1, k) - input(i, j, k);
            if (k + 1 < Z)
              val[18] = input(i + 1, j + 1, k + 1) - input(i, j, k);
            if (k >= 1)
              val[19] = input(i + 1, j + 1, k - 1) - input(i, j, k);
          }
          if (k + 1 < Z)
            val[7] = input(i + 1, j, k + 1) - input(i, j, k);
          if (j >= 1) {
            val[8] = input(i + 1, j - 1, k) - input(i, j, k);
            if (k + 1 < Z)
              val[20] = input(i + 1, j - 1, k + 1) - input(i, j, k);
            if (k >= 1)
              val[21] = input(i + 1, j - 1, k - 1) - input(i, j, k);
          }
          if (k >= 1)
            val[9] = input(i + 1, j, k - 1) - input(i, j, k);
        }
        if (i >= 1) {
          val[1] = input(i - 1, j, k) - input(i, j, k);
          if (j + 1 < Y) {
            val[10] = input(i - 1, j + 1, k) - input(i, j, k);
            if (k + 1 < Z)
              val[22] = input(i - 1, j + 1, k + 1) - input(i, j, k);
            if (k >= 1)
              val[23] = input(i - 1, j + 1, k - 1) - input(i, j, k);
          }
          if (k + 1 < Z)
            val[11] = input(i - 1, j, k + 1) - input(i, j, k);
          if (j >= 1) {
            val[12] = input(i - 1, j - 1, k) - input(i, j, k);
            if (k + 1 < Z)
              val[24] = input(i - 1, j - 1, k + 1) - input(i, j, k);
            if (k >= 1)
              val[25] = input(i - 1, j - 1, k - 1) - input(i, j, k);
          }
          if (k >= 1)
            val[13] = input(i - 1, j, k - 1) - input(i, j, k);
        }
        if (j + 1 < Y)
          val[2] = input(i, j + 1, k) - input(i, j, k);
        if (j >= 1)
          val[3] = input(i, j - 1, k) - input(i, j, k);
        if (k + 1 < Z) {
          val[4] = input(i, j, k + 1) - input(i, j, k);
          if (j + 1 < Y)
            val[14] = input(i, j + 1, k + 1) - input(i, j, k);
          if (j >= 1)
            val[15] = input(i, j - 1, k + 1) - input(i, j, k);
        }
        if (k >= 1) {
          val[5] = input(i, j, k - 1) - input(i, j, k);
          if (j + 1 < Y)
            val[16] = input(i, j + 1, k - 1) - input(i, j, k);
          if (j >= 1)
            val[17] = input(i, j - 1, k - 1) - input(i, j, k);
        }
        for (int l = 0; l < 6; l++)
          sum += val[l] * val[l];
        for (int l = 6; l < 18; l++)
          sum += val[l] * val[l] / 2.0;
        for (int l = 18; l < 26; l++)
          sum += val[l] * val[l] / 3.0;
      }
      grad[idx(i, j, k)] = static_cast<float>(std::sqrt(sum));
    }
  }

  // One of filtGDTV's terms: phi(g)/g at the voxel plus phi/g at the neighbour, or phi(eps)/eps
  // there when the neighbour's gradient is 0. A double, as the legacy's expression is: it becomes
  // a float only where it is stored, after filtGDTV2's halving or thirding.
  double term(double g0, double gn) const {
    const float epsilon = 0.0001;
    if (gn != 0)
      return gdtv_phi(g0, q) / g0 + gdtv_phi(gn, q) / gn;
    return gdtv_phi(g0, q) / g0 + gdtv_phi(epsilon, q) / epsilon;
  }

  // filtGDTV's voxel (i, j, k): its six neighbours' weights, normalised with lambda's, and the
  // weighted sum of the last iteration's neighbours and the input.
  double six(std::int64_t i, std::int64_t j, std::int64_t k) const {
    const volume &funcval = *cur;
    const std::uint64_t X = static_cast<std::uint64_t>(nx), Y = static_cast<std::uint64_t>(ny),
                        Z = static_cast<std::uint64_t>(nz);
    const std::uint64_t ui = static_cast<std::uint64_t>(i), uj = static_cast<std::uint64_t>(j),
                        uk = static_cast<std::uint64_t>(k);
    float temp[7];
    for (int l = 0; l < 7; l++)
      temp[l] = 0.0;
    const double g = G(i, j, k);
    if (g != 0.0) {
      if ((ui < X - 1))
        temp[0] = term(g, G(i + 1, j, k));
      if ((ui > 1))
        temp[1] = term(g, G(i - 1, j, k));
      if ((uj < Y - 1))
        temp[2] = term(g, G(i, j + 1, k));
      if ((uj > 1))
        temp[3] = term(g, G(i, j - 1, k));
      if ((uk < Z - 1))
        temp[4] = term(g, G(i, j, k + 1));
      if ((uk > 1))
        temp[5] = term(g, G(i, j, k - 1));
    }
    temp[6] = lbda;
    float sum = 0.0;
    for (int l = 0; l < 7; l++)
      sum += temp[l];
    for (int l = 0; l < 7; l++)
      temp[l] = temp[l] / sum;

    double temple = 0.0;
    if ((ui < X - 1))
      temple += funcval(i + 1, j, k) * temp[0];
    if ((ui > 1))
      temple += funcval(i - 1, j, k) * temp[1];
    if ((uj < Y - 1))
      temple += funcval(i, j + 1, k) * temp[2];
    if ((uj > 1))
      temple += funcval(i, j - 1, k) * temp[3];
    if ((uk < Z - 1))
      temple += funcval(i, j, k + 1) * temp[4];
    if ((uk > 1))
      temple += funcval(i, j, k - 1) * temp[5];
    temple += (*in)(i, j, k) * temp[6];
    return temple;
  }

  // filtGDTV2's voxel: the twenty-six neighbours, the edge ones' terms halved and the corner
  // ones' thirded.
  double twenty_six(std::int64_t i, std::int64_t j, std::int64_t k) const {
    const volume &funcval = *cur;
    const std::uint64_t X = static_cast<std::uint64_t>(nx), Y = static_cast<std::uint64_t>(ny),
                        Z = static_cast<std::uint64_t>(nz);
    const std::uint64_t ui = static_cast<std::uint64_t>(i), uj = static_cast<std::uint64_t>(j),
                        uk = static_cast<std::uint64_t>(k);
    float temp[27];
    for (int l = 0; l < 27; l++)
      temp[l] = 0.0;
    const double g = G(i, j, k);
    if (g != 0.0) {
      if ((ui < X - 1)) {
        temp[0] = term(g, G(i + 1, j, k));
        if ((uj < Y - 1)) {
          temp[6] = 0.5 * term(g, G(i + 1, j + 1, k));
          if ((uk < Z - 1))
            temp[18] = term(g, G(i + 1, j + 1, k + 1)) / 3.0;
          if ((uk > 1))
            temp[19] = term(g, G(i + 1, j + 1, k - 1)) / 3.0;
        }
        if ((uj > 1)) {
          temp[7] = 0.5 * term(g, G(i + 1, j - 1, k));
          if ((uk < Z - 1))
            temp[20] = term(g, G(i + 1, j - 1, k + 1)) / 3.0;
          if ((uk > 1))
            temp[21] = term(g, G(i + 1, j - 1, k - 1)) / 3.0;
        }
        if ((uk < Z - 1))
          temp[8] = 0.5 * term(g, G(i + 1, j, k + 1));
        if ((uk > 1))
          temp[9] = 0.5 * term(g, G(i + 1, j, k - 1));
      }
      if ((ui > 1)) {
        temp[1] = term(g, G(i - 1, j, k));
        if ((uj < Y - 1)) {
          temp[10] = 0.5 * term(g, G(i - 1, j + 1, k));
          if ((uk < Z - 1))
            temp[22] = term(g, G(i - 1, j + 1, k + 1)) / 3.0;
          if ((uk > 1))
            temp[23] = term(g, G(i - 1, j + 1, k - 1)) / 3.0;
        }
        if ((uj > 1)) {
          temp[11] = 0.5 * term(g, G(i - 1, j - 1, k));
          if ((uk < Z - 1))
            temp[24] = term(g, G(i - 1, j - 1, k + 1)) / 3.0;
          if ((uk > 1))
            temp[25] = term(g, G(i - 1, j - 1, k - 1)) / 3.0;
        }
        if ((uk < Z - 1))
          temp[12] = 0.5 * term(g, G(i - 1, j, k + 1));
        if ((uk > 1))
          temp[13] = 0.5 * term(g, G(i - 1, j, k - 1));
      }
      if ((uj < Y - 1))
        temp[2] = term(g, G(i, j + 1, k));
      if ((uj > 1))
        temp[3] = term(g, G(i, j - 1, k));
      if ((uk < Z - 1)) {
        temp[4] = term(g, G(i, j, k + 1));
        if ((uj < Y - 1))
          temp[14] = 0.5 * term(g, G(i, j + 1, k + 1));
        if ((uj > 1))
          temp[15] = 0.5 * term(g, G(i, j - 1, k + 1));
      }
      if ((uk > 1)) {
        temp[5] = term(g, G(i, j, k - 1));
        if ((uj < Y - 1))
          temp[16] = 0.5 * term(g, G(i, j + 1, k - 1));
        if ((uj > 1))
          temp[17] = 0.5 * term(g, G(i, j - 1, k - 1));
      }
    }
    temp[26] = lbda;
    float sum = 0.0;
    for (int l = 0; l < 27; l++)
      sum += temp[l];
    for (int l = 0; l < 27; l++)
      temp[l] = temp[l] / sum;

    double temple = 0.0;
    if ((ui < X - 1)) {
      temple += funcval(i + 1, j, k) * temp[0];
      if ((uj < Y - 1)) {
        temple += funcval(i + 1, j + 1, k) * temp[6];
        if ((uk < Z - 1))
          temple += funcval(i + 1, j + 1, k + 1) * temp[18];
        if ((uk > 1))
          temple += funcval(i + 1, j + 1, k - 1) * temp[19];
      }
      if ((uj > 1)) {
        temple += funcval(i + 1, j - 1, k) * temp[7];
        if ((uk < Z - 1))
          temple += funcval(i + 1, j - 1, k + 1) * temp[20];
        if ((uk > 1))
          temple += funcval(i + 1, j - 1, k - 1) * temp[21];
      }
      if ((uk < Z - 1))
        temple += funcval(i + 1, j, k + 1) * temp[8];
      if ((uk > 1))
        temple += funcval(i + 1, j, k - 1) * temp[9];
    }
    if ((ui > 1)) {
      temple += funcval(i - 1, j, k) * temp[1];
      if ((uj < Y - 1)) {
        temple += funcval(i - 1, j + 1, k) * temp[10];
        if ((uk < Z - 1))
          temple += funcval(i - 1, j + 1, k + 1) * temp[22];
        if ((uk > 1))
          temple += funcval(i - 1, j + 1, k - 1) * temp[23];
      }
      if ((uj > 1)) {
        temple += funcval(i - 1, j - 1, k) * temp[11];
        if ((uk < Z - 1))
          temple += funcval(i - 1, j - 1, k + 1) * temp[24];
        if ((uk > 1))
          temple += funcval(i - 1, j - 1, k - 1) * temp[25];
      }
      if ((uk < Z - 1))
        temple += funcval(i - 1, j, k + 1) * temp[12];
      if ((uk > 1))
        temple += funcval(i - 1, j, k - 1) * temp[13];
    }
    if ((uj < Y - 1))
      temple += funcval(i, j + 1, k) * temp[2];
    if ((uj > 1))
      temple += funcval(i, j - 1, k) * temp[3];
    if ((uk < Z - 1)) {
      temple += funcval(i, j, k + 1) * temp[4];
      if ((uj < Y - 1))
        temple += funcval(i, j + 1, k + 1) * temp[14];
      if ((uj > 1))
        temple += funcval(i, j - 1, k + 1) * temp[15];
    }
    if ((uk > 1)) {
      temple += funcval(i, j, k - 1) * temp[5];
      if ((uj < Y - 1))
        temple += funcval(i, j + 1, k - 1) * temp[16];
      if ((uj > 1))
        temple += funcval(i, j - 1, k - 1) * temp[17];
    }
    temple += (*in)(i, j, k) * temp[26];
    return temple;
  }

  void row() override {
    const std::int64_t j = at % ny, k = at / ny;
    if (!gradient_done) {
      gradient_row(j, k);
      if (++at >= rows) {
        at = 0;
        gradient_done = true;
      }
      return;
    }
    volume &dst = *next;
    for (std::int64_t i = 0; i < nx; i++) {
      // funcvalue(i, j, k, temple) holds it as a float; funcval(i, j, k, funcvalue(i, j, k))
      // writes that in the volume's type.
      const float f = static_cast<float>(full ? twenty_six(i, j, k) : six(i, j, k));
      dst(i, j, k, static_cast<double>(f));
    }
    if (++at < rows)
      return;
    at = 0;
    std::swap(cur, next);
    if (++m >= iterations) {
      next.reset();
      grad.clear();
      grad.shrink_to_fit();
      cur->unsetMinMax(); // its range is the filtered voxels', taken when asked
      finished = true;
    }
  }
  const char *phase() const override {
    return gradient_done ? "gdtv: filtering" : "gdtv: gradient";
  }
  std::string summary() const override {
    char e[64];
    std::snprintf(e, sizeof e, "exponent %g, lambda %g", exponent, static_cast<double>(lbda));
    return "gdtv: " + dims() + ", " + e + ", " + std::to_string(iterations) + " iterations, " +
           (full ? "twenty-six" : "six") + " neighbours";
  }
  std::shared_ptr<volume> result() override { return std::move(cur); }
};

} // namespace

struct volume_filter_stepper::impl {
  std::unique_ptr<filter_engine> engine;
  bool taken = false;
};

volume_filter_stepper::volume_filter_stepper(std::shared_ptr<const volume> in,
                                             const volume_filter_params &p)
    : _impl(std::make_unique<impl>()) {
  if (!in)
    throw std::invalid_argument("volume_filter: no volume");
  if (in->XDim() == 0 || in->YDim() == 0 || in->ZDim() == 0)
    throw std::invalid_argument("volume_filter: the volume is empty");
  const std::vector<std::string> problems = params_validate(p);
  if (!problems.empty()) {
    std::string why = "volume_filter: ";
    for (std::size_t i = 0; i < problems.size(); ++i)
      why += (i ? "; " : "") + problems[i];
    throw std::invalid_argument(why);
  }
  switch (p.filter) {
  case volume_filter_kind::bilateral:
    _impl->engine = std::make_unique<bilateral_engine>(std::move(in), p);
    break;
  case volume_filter_kind::contrast:
    _impl->engine = std::make_unique<contrast_engine>(std::move(in), p);
    break;
  case volume_filter_kind::anisotropic:
    _impl->engine = std::make_unique<anisotropic_engine>(std::move(in), p);
    break;
  case volume_filter_kind::gdtv:
    _impl->engine = std::make_unique<gdtv_engine>(std::move(in), p);
    break;
  }
  if (!_impl->engine)
    throw std::invalid_argument("volume_filter: no such filter");
}
volume_filter_stepper::~volume_filter_stepper() = default;
volume_filter_stepper::volume_filter_stepper(volume_filter_stepper &&) noexcept = default;
volume_filter_stepper &
volume_filter_stepper::operator=(volume_filter_stepper &&) noexcept = default;

bool volume_filter_stepper::step(std::size_t budget) { return _impl->engine->step(budget); }

bool volume_filter_stepper::done() const { return _impl->engine->finished; }

double volume_filter_stepper::progress() const { return _impl->engine->progress(); }

const char *volume_filter_stepper::phase() const { return _impl->engine->phase(); }

std::string volume_filter_stepper::status() const {
  const filter_engine &e = *_impl->engine;
  if (e.finished)
    return e.summary();
  return std::string(e.phase()) + " " + std::to_string(static_cast<int>(100.0 * e.progress())) +
         "%";
}

std::shared_ptr<const volume> volume_filter_stepper::take() {
  if (!_impl->engine->finished)
    throw std::logic_error("volume_filter: take() before the filter is done");
  if (_impl->taken)
    throw std::logic_error("volume_filter: take() after the result was taken");
  _impl->taken = true;
  return std::shared_ptr<const volume>(_impl->engine->result());
}

std::unique_ptr<result_job<std::shared_ptr<const volume>>>
volume_filter_job(std::shared_ptr<const volume> in, const volume_filter_params &p) {
  return make_result_job(volume_filter_stepper(std::move(in), p));
}

volume filter_volume(const volume &in, const volume_filter_params &p) {
  volume_filter_stepper s(std::make_shared<const volume>(in), p);
  s.step(0);
  return *s.take();
}

} // namespace cvc
