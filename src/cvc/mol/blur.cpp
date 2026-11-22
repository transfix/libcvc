/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol::blur (blur.h): TexMol's GOABlur::blurAtoms and BlurMapsDataManager::getBoundingBox
// (transfix/TexMol ed20eae, src/Blurmaps), the electron density only, as a stepper. The arithmetic
// is GOABlur's type for type, so the sums come out the same: see blur.h.

#include <atomic>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/mol/blur.h>
#include <exception>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace mol {

namespace {

std::string join(const std::vector<std::string> &v, const char *sep) {
  std::string out;
  for (std::size_t i = 0; i < v.size(); ++i)
    out += (i ? sep : "") + v[i];
  return out;
}

// One atom as GOABlur held it: a float position and radius.
struct blob {
  float pos[3];
  float radius;
};

// An atom's splat: its centre (in world units, double), radius, and the grid points it reaches.
struct splat {
  double c[3];
  float radius;
  unsigned lo[3], hi[3]; // [lo, hi) along each axis
};

} // namespace

struct blur_stepper::impl {
  enum class stage { checking, splatting, done, failed, cancelled };

  app &ctx;
  std::vector<blob> atoms;
  blur_params p;
  stage st = stage::checking;
  std::atomic<bool> cancel_requested{false};
  std::exception_ptr error;

  // The box, built a unit per atom in "checking".
  std::size_t scanned = 0;
  float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
  float max_radius = 0;

  // The grid.
  unsigned dim[3] = {0, 0, 0};
  float span[3] = {0, 0, 0};
  double log_cutoff = 0;
  std::vector<float> dens;

  // "splatting": the atom being splatted and its next grid point.
  std::size_t next = 0; // the atom being splatted
  bool in_atom = false;
  splat s{};
  unsigned j = 0, k = 0, l = 0;
  std::size_t units_done = 0, units_total = 0;
  std::size_t atoms_total_units = 0; // for progress before the grid exists

  impl(app &c, const molecule &m, blur_params params) : ctx(c), p(std::move(params)) {
    atoms.reserve(m.atoms.size());
    for (const atom &a : m.atoms)
      atoms.push_back({{static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z)},
                       static_cast<float>(a.radius)});
  }

  // The grid points the atom reaches, and its centre (GOABlur::blurAtoms, its loop's head).
  splat splat_of(const blob &b) const {
    splat out{};
    out.radius = b.radius;
    const double reach = static_cast<double>(b.radius) * std::sqrt(1.0 + log_cutoff / p.blobbiness);
    for (int a = 0; a < 3; ++a) {
      double c = (b.pos[a] - lo[a]) / span[a]; // float arithmetic, as GOABlur's
      if (p.snap_centres)
        c = (c - std::floor(c)) >= 0.5 ? std::ceil(c) : std::floor(c);
      int amin = static_cast<int>(c - (reach / span[a]) - 1);
      amin = amin < 0 ? 0 : amin;
      int amax = static_cast<int>(c + (reach / span[a]) + 1);
      amax = amax > static_cast<int>(dim[a]) ? static_cast<int>(dim[a]) : amax;
      out.lo[a] = static_cast<unsigned>(amin);
      out.hi[a] = static_cast<unsigned>(amax < amin ? amin : amax);
      out.c[a] = p.snap_centres ? lo[a] + c * span[a] : static_cast<double>(b.pos[a]);
    }
    return out;
  }

  static std::size_t count(const splat &sp) {
    std::size_t n = 1;
    for (int a = 0; a < 3; ++a)
      n *= sp.hi[a] - sp.lo[a];
    return n;
  }

  // "checking": the parameters, then the box a unit per atom, then the grid.
  void check_step(std::size_t &budget) {
    if (scanned == 0) {
      const std::vector<std::string> bad = params_validate(p);
      if (!bad.empty())
        throw std::invalid_argument("blur: " + join(bad, "; "));
      if (atoms.empty())
        throw std::invalid_argument("blur: the molecule has no atoms");
    }
    while (scanned < atoms.size() && budget > 0) {
      const blob &b = atoms[scanned];
      for (int a = 0; a < 3; ++a) {
        lo[a] = scanned == 0 ? b.pos[a] : (b.pos[a] < lo[a] ? b.pos[a] : lo[a]);
        hi[a] = scanned == 0 ? b.pos[a] : (b.pos[a] > hi[a] ? b.pos[a] : hi[a]);
      }
      max_radius = b.radius > max_radius ? b.radius : max_radius;
      ++scanned;
      --budget;
    }
    if (scanned < atoms.size())
      return;
    const float pad = static_cast<float>(p.padding) * max_radius;
    for (int a = 0; a < 3; ++a) {
      lo[a] -= pad;
      hi[a] += pad;
      if (p.dims.empty()) {
        const double d = (hi[a] - lo[a]) / p.resolution + 3;
        if (!(d < 65536.0))
          throw std::invalid_argument("blur: the grid would have " + std::to_string(d) +
                                      " points along an axis; raise resolution");
        dim[a] = static_cast<unsigned>(d);
      } else {
        dim[a] = static_cast<unsigned>(p.dims[static_cast<std::size_t>(a)]);
      }
      span[a] = (hi[a] - lo[a]) / static_cast<float>(dim[a] - 1);
      if (!(span[a] > 0))
        throw std::invalid_argument("blur: the box is flat along an axis (a single atom with "
                                    "radius 0, or padding 0)");
    }
    // GOABlur's eps is a float, which getMaxRadiusOfAtomPotential takes as a double.
    log_cutoff = std::log(static_cast<double>(static_cast<float>(p.cutoff)));
    dens.assign(static_cast<std::size_t>(dim[0]) * dim[1] * dim[2], 0.0f);
    for (const blob &b : atoms)
      units_total += b.radius < 0.0000001f ? 1 : count(splat_of(b));
    st = stage::splatting;
  }

  void splat_step(std::size_t &budget) {
    const std::size_t sx = dim[0], sxy = static_cast<std::size_t>(dim[0]) * dim[1];
    const double blobby = p.blobbiness;
    while (budget > 0 && next < atoms.size()) {
      if (!in_atom) {
        const blob &b = atoms[next];
        if (b.radius < 0.0000001f) { // GOABlur's evalDensity gives such an atom nothing
          ++next;
          ++units_done;
          --budget;
          continue;
        }
        s = splat_of(b);
        if (count(s) == 0) {
          ++next;
          continue;
        }
        l = s.lo[2];
        k = s.lo[1];
        j = s.lo[0];
        in_atom = true;
      }
      const double r0 = static_cast<double>(s.radius * s.radius); // float product, as GOABlur's
      while (budget > 0) {
        const float px = lo[0] + j * span[0], py = lo[1] + k * span[1], pz = lo[2] + l * span[2];
        const double dx = s.c[0] - px, dy = s.c[1] - py, dz = s.c[2] - pz;
        const double r = dx * dx + dy * dy + dz * dz;
        const double expval = blobby * r / r0 - blobby;
        dens[j + k * sx + l * sxy] += static_cast<float>(std::exp(expval));
        ++units_done;
        --budget;
        if (++j < s.hi[0])
          continue;
        j = s.lo[0];
        if (++k < s.hi[1])
          continue;
        k = s.lo[1];
        if (++l < s.hi[2])
          continue;
        in_atom = false;
        ++next;
        break;
      }
    }
    if (next >= atoms.size())
      st = stage::done;
  }
};

blur_stepper::blur_stepper(app &ctx, const molecule &m, blur_params params)
    : _impl(std::make_unique<impl>(ctx, m, std::move(params))) {}
blur_stepper::~blur_stepper() = default;
blur_stepper::blur_stepper(blur_stepper &&) noexcept = default;
blur_stepper &blur_stepper::operator=(blur_stepper &&) noexcept = default;

bool blur_stepper::step(std::size_t budget) {
  impl &d = *_impl;
  if (d.st == impl::stage::failed)
    std::rethrow_exception(d.error);
  if (d.st == impl::stage::done || d.st == impl::stage::cancelled)
    return true;
  if (d.cancel_requested.load(std::memory_order_acquire)) {
    d.st = impl::stage::cancelled;
    d.dens.clear();
    d.dens.shrink_to_fit();
    return true;
  }
  std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
  try {
    while (left > 0 && d.st != impl::stage::done) {
      if (d.st == impl::stage::checking)
        d.check_step(left);
      else
        d.splat_step(left);
      if (d.st == impl::stage::checking && d.scanned < d.atoms.size())
        break; // out of budget mid-scan
    }
  } catch (...) {
    d.error = std::current_exception();
    d.st = impl::stage::failed;
    d.dens.clear();
    throw;
  }
  return d.st == impl::stage::done;
}

bool blur_stepper::done() const { return _impl->st == impl::stage::done; }

const char *blur_stepper::phase() const {
  switch (_impl->st) {
  case impl::stage::checking:
    return "checking";
  case impl::stage::splatting:
    return "splatting";
  case impl::stage::done:
    return "done";
  case impl::stage::failed:
    return "failed";
  case impl::stage::cancelled:
    return "cancelled";
  }
  return "failed";
}

double blur_stepper::progress() const {
  const impl &d = *_impl;
  if (d.st == impl::stage::done)
    return 1.0;
  // Checking is the first 1%; splatting the rest, by grid points visited.
  if (d.st == impl::stage::checking)
    return d.atoms.empty() ? 0.0 : 0.01 * static_cast<double>(d.scanned) / d.atoms.size();
  if (d.units_total == 0)
    return 0.01;
  return 0.01 + 0.99 * static_cast<double>(d.units_done) / static_cast<double>(d.units_total);
}

std::string blur_stepper::status() const {
  const impl &d = *_impl;
  if (d.st == impl::stage::splatting)
    return "splatting atom " + std::to_string(d.next + 1) + " of " + std::to_string(d.atoms.size());
  if (d.st == impl::stage::done)
    return std::to_string(d.dim[0]) + "x" + std::to_string(d.dim[1]) + "x" +
           std::to_string(d.dim[2]) + " grid, " + std::to_string(d.atoms.size()) + " atoms";
  return phase();
}

void blur_stepper::cancel() { _impl->cancel_requested.store(true, std::memory_order_release); }

volume blur_stepper::take() {
  impl &d = *_impl;
  if (d.st == impl::stage::failed)
    std::rethrow_exception(d.error);
  if (d.st != impl::stage::done)
    throw std::logic_error("blur_stepper::take: not done");
  volume v(d.ctx, reinterpret_cast<const unsigned char *>(d.dens.data()),
           dimension(d.dim[0], d.dim[1], d.dim[2]), Float,
           bounding_box(d.lo[0], d.lo[1], d.lo[2], d.hi[0], d.hi[1], d.hi[2]));
  d.dens.clear();
  d.dens.shrink_to_fit();
  return v;
}

volume blur(app &ctx, const molecule &m, const blur_params &params) {
  blur_stepper s(ctx, m, params);
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<volume>> blur_job(app &ctx, const molecule &m,
                                             const blur_params &params) {
  return make_result_job(std::make_unique<blur_stepper>(ctx, m, params));
}

} // namespace mol
} // namespace cvc
