/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's torsion angles (cvc/mol/torsions.h).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cvc/mol/torsions.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace mol {

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kBond = 2.0; // Å: C(i-1)-N or O3'(i-1)-P still bonded

constexpr const char *kPurines[] = {"A", "G", "I", "DA", "DG", "DI"};
constexpr const char *kPyrimidines[] = {"C", "U", "T", "DC", "DU", "DT"};

template <std::size_t N> bool one_of(const std::string &s, const char *const (&names)[N]) {
  for (const char *n : names)
    if (s == n)
      return true;
  return false;
}

// A residue's atoms: its first and one past its last.
struct span {
  std::size_t first = 0, last = 0;
};

using point = std::array<double, 3>;

// The residue's atom called `name`, its prime written ' or *; null when it has none.
const atom *find(const molecule &m, const span &r, const char *name) {
  if (r.first >= r.last)
    return nullptr;
  std::string alt = name;
  for (char &c : alt)
    if (c == '\'')
      c = '*';
  for (std::size_t i = r.first; i < r.last; ++i) {
    const std::string &n = m.atoms[i].name;
    if (n == name || n == alt)
      return &m.atoms[i];
  }
  return nullptr;
}

point at(const atom *a) { return {a->x, a->y, a->z}; }

double dist(const atom *a, const atom *b) {
  return std::sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y) +
                   (a->z - b->z) * (a->z - b->z));
}

double torsion(const atom *a, const atom *b, const atom *c, const atom *d) {
  return a && b && c && d ? dihedral(at(a), at(b), at(c), at(d)) : kNaN;
}

} // namespace

residue_torsions::residue_torsions()
    : phi(kNaN), psi(kNaN), omega(kNaN), alpha(kNaN), beta(kNaN), gamma(kNaN), delta(kNaN),
      epsilon(kNaN), zeta(kNaN), chi(kNaN), nu{kNaN, kNaN, kNaN, kNaN, kNaN}, eta(kNaN),
      theta(kNaN) {}

double dihedral(const point &a, const point &b, const point &c, const point &d) {
  const point b1{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  const point b2{c[0] - b[0], c[1] - b[1], c[2] - b[2]};
  const point b3{d[0] - c[0], d[1] - c[1], d[2] - c[2]};
  const auto cross = [](const point &u, const point &v) {
    return point{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
  };
  const auto dot = [](const point &u, const point &v) {
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  };
  const point n1 = cross(b1, b2), n2 = cross(b2, b3);
  const double len2 = std::sqrt(dot(b2, b2));
  if (dot(n1, n1) == 0.0 || dot(n2, n2) == 0.0 || len2 == 0.0)
    return kNaN;
  // atan2(|b2| b1 . (b2 x b3), (b1 x b2) . (b2 x b3)): IUPAC's sign.
  const double y = len2 * dot(b1, n2), x = dot(n1, n2);
  return std::atan2(y, x);
}

struct torsion_stepper::impl {
  std::shared_ptr<const molecule> mol;
  std::vector<span> residues;
  std::vector<residue_torsions> out;
  std::size_t cursor = 0; // the next atom while finding residues, then the next residue
  bool listing = true, finished = false, cancelled = false;
  std::atomic<bool> cancel_requested{false};

  explicit impl(std::shared_ptr<const molecule> m) : mol(std::move(m)) {
    if (!mol)
      throw std::invalid_argument("torsions: no molecule");
  }

  static bool same_residue(const atom &a, const atom &b) {
    return a.model == b.model && a.chain == b.chain && a.residue_seq == b.residue_seq &&
           a.insertion == b.insertion;
  }

  void list_step(std::size_t &left) {
    const std::vector<atom> &atoms = mol->atoms;
    while (cursor < atoms.size() && left > 0) {
      if (residues.empty() || !same_residue(atoms[residues.back().first], atoms[cursor]))
        residues.push_back({cursor, cursor});
      residues.back().last = cursor + 1;
      ++cursor;
      --left;
    }
    if (cursor == atoms.size()) {
      listing = false;
      cursor = 0;
      out.reserve(residues.size());
    }
  }

  bool same_chain(const span &a, const span &b) const {
    const atom &p = mol->atoms[a.first], &q = mol->atoms[b.first];
    return p.model == q.model && p.chain == q.chain;
  }

  // The residue before (or after) residue k when bonded to it, as `bonded` tells.
  template <class F> const span *neighbour(std::size_t k, int dir, F &&bonded) const {
    if ((dir < 0 && k == 0) || (dir > 0 && k + 1 >= residues.size()))
      return nullptr;
    const span &o = residues[dir < 0 ? k - 1 : k + 1];
    if (!same_chain(o, residues[k]))
      return nullptr;
    return bonded(dir < 0 ? o : residues[k], dir < 0 ? residues[k] : o) ? &o : nullptr;
  }

  void measure(std::size_t k) {
    const molecule &m = *mol;
    const span &r = residues[k];
    const atom &first = m.atoms[r.first];
    residue_torsions t;
    t.first_atom = r.first;
    t.model = first.model;
    t.chain = first.chain;
    t.residue_seq = first.residue_seq;
    t.insertion = first.insertion;
    t.residue = first.residue;
    const bool purine = one_of(first.residue, kPurines);
    const bool pyrimidine = one_of(first.residue, kPyrimidines);
    const atom *n = find(m, r, "N"), *ca = find(m, r, "CA"), *c = find(m, r, "C");
    if (n && ca && c && !purine && !pyrimidine) {
      t.type = residue_torsions::kind::amino_acid;
      const auto peptide = [&](const span &a, const span &b) {
        const atom *ac = find(m, a, "C"), *bn = find(m, b, "N");
        return ac && bn && dist(ac, bn) <= kBond;
      };
      const span *prev = neighbour(k, -1, peptide), *next = neighbour(k, 1, peptide);
      if (prev) {
        const atom *pc = find(m, *prev, "C"), *pca = find(m, *prev, "CA");
        t.phi = torsion(pc, n, ca, c);
        t.omega = torsion(pca, pc, n, ca);
      }
      if (next)
        t.psi = torsion(n, ca, c, find(m, *next, "N"));
    } else if (purine || pyrimidine || find(m, r, "C4'")) {
      t.type = residue_torsions::kind::nucleotide;
      const auto linked = [&](const span &a, const span &b) {
        const atom *o3 = find(m, a, "O3'"), *p = find(m, b, "P");
        return o3 && p && dist(o3, p) <= kBond;
      };
      const span *prev = neighbour(k, -1, linked), *next = neighbour(k, 1, linked);
      const atom *p = find(m, r, "P"), *o5 = find(m, r, "O5'"), *c5 = find(m, r, "C5'"),
                 *c4 = find(m, r, "C4'"), *c3 = find(m, r, "C3'"), *o3 = find(m, r, "O3'"),
                 *o4 = find(m, r, "O4'"), *c1 = find(m, r, "C1'"), *c2 = find(m, r, "C2'");
      const atom *po3 = prev ? find(m, *prev, "O3'") : nullptr;
      const atom *pc4 = prev ? find(m, *prev, "C4'") : nullptr;
      const atom *np = next ? find(m, *next, "P") : nullptr;
      const atom *no5 = next ? find(m, *next, "O5'") : nullptr;
      const atom *nc4 = next ? find(m, *next, "C4'") : nullptr;
      t.alpha = torsion(po3, p, o5, c5);
      t.beta = torsion(p, o5, c5, c4);
      t.gamma = torsion(o5, c5, c4, c3);
      t.delta = torsion(c5, c4, c3, o3);
      t.epsilon = torsion(c4, c3, o3, np);
      t.zeta = torsion(c3, o3, np, no5);
      t.nu = {torsion(c4, o4, c1, c2), torsion(o4, c1, c2, c3), torsion(c1, c2, c3, c4),
              torsion(c2, c3, c4, o4), torsion(c3, c4, o4, c1)};
      if (purine)
        t.chi = torsion(o4, c1, find(m, r, "N9"), find(m, r, "C4"));
      else if (pyrimidine)
        t.chi = torsion(o4, c1, find(m, r, "N1"), find(m, r, "C2"));
      t.eta = torsion(pc4, p, c4, np);
      t.theta = torsion(p, c4, np, nc4);
    }
    out.push_back(std::move(t));
  }
};

torsion_stepper::torsion_stepper(std::shared_ptr<const molecule> m)
    : _impl(std::make_unique<impl>(std::move(m))) {}
torsion_stepper::~torsion_stepper() = default;
torsion_stepper::torsion_stepper(torsion_stepper &&) noexcept = default;
torsion_stepper &torsion_stepper::operator=(torsion_stepper &&) noexcept = default;

bool torsion_stepper::step(std::size_t budget) {
  impl &d = *_impl;
  if (d.finished || d.cancelled)
    return true;
  if (d.cancel_requested.load(std::memory_order_acquire)) {
    d.cancelled = true;
    d.out.clear();
    return true;
  }
  std::size_t left = budget == 0 ? std::numeric_limits<std::size_t>::max() : budget;
  if (d.listing)
    d.list_step(left);
  while (!d.listing && d.cursor < d.residues.size() && left > 0) {
    d.measure(d.cursor);
    ++d.cursor;
    --left;
  }
  if (!d.listing && d.cursor >= d.residues.size())
    d.finished = true;
  return d.finished;
}

bool torsion_stepper::done() const { return _impl->finished; }

const char *torsion_stepper::phase() const {
  const impl &d = *_impl;
  return d.cancelled ? "cancelled" : d.finished ? "done" : d.listing ? "residues" : "measuring";
}

double torsion_stepper::progress() const {
  const impl &d = *_impl;
  if (d.finished)
    return 1.0;
  const double n = static_cast<double>(d.mol->atoms.size());
  if (d.listing)
    return n == 0 ? 0.0 : 0.5 * static_cast<double>(d.cursor) / n;
  return 0.5 + 0.5 * static_cast<double>(d.cursor) /
                   static_cast<double>(std::max<std::size_t>(1, d.residues.size()));
}

void torsion_stepper::cancel() { _impl->cancel_requested.store(true, std::memory_order_release); }

std::vector<residue_torsions> torsion_stepper::take() {
  if (!_impl->finished)
    throw std::logic_error("torsion_stepper::take: not done");
  return std::move(_impl->out);
}

std::vector<residue_torsions> torsions(const molecule &m) {
  torsion_stepper s(std::make_shared<const molecule>(m));
  s.step(0);
  return s.take();
}

std::unique_ptr<result_job<std::vector<residue_torsions>>>
torsions_job(std::shared_ptr<const molecule> m) {
  return make_result_job(std::make_unique<torsion_stepper>(std::move(m)));
}

} // namespace mol
} // namespace cvc
