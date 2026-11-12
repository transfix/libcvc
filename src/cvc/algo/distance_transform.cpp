/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// distance_transform.cpp — the exact Euclidean distance transform (see distance_transform.h).
//
// Moved here from cvc/nav's grid_nav.cpp unchanged, so cvc::world can use it too: a line-for-line
// transcription of GRL-SNAM's sdf_nav._edt2. Do NOT "tidy" the arithmetic grouping, the tie-break
// comparisons or the scan orders: the doubles are bit-identical to the Python's, and nav_test's
// NavEdtExact checks them bit for bit against a verbatim copy of the column-then-row kernel. This
// TU must be built without -ffast-math / -ffp-contract=fast.

#include <algorithm>
#include <cvc/algo/distance_transform.h>

namespace cvc {
namespace algo {

namespace {

constexpr double EDT_INF = edt_far;

// Felzenszwalb & Huttenlocher 1-D squared-distance transform over a single
// strided line: reads f[q * fs] for q in [0, n), writes d[q * ds]. This is the
// scalar kernel sdf_nav._edt1d, unchanged except that the envelope scratch
// (v: n ints, z: n + 1 doubles) is the caller's, not two heap vectors per line.
void edt1d_line(const double *f, int n, int fs, double *d, int ds, int *v, double *z) {
  int k = 0;
  v[0] = 0;
  z[0] = -EDT_INF;
  z[1] = EDT_INF;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q * fs] + static_cast<double>(q * q)) -
                (f[v[k] * fs] + static_cast<double>(v[k] * v[k]))) /
               static_cast<double>(2 * q - 2 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q * fs] + static_cast<double>(q * q)) -
           (f[v[k] * fs] + static_cast<double>(v[k] * v[k]))) /
          static_cast<double>(2 * q - 2 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = EDT_INF;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q)
      ++k;
    int diff = q - v[k];
    d[q * ds] = static_cast<double>(diff * diff) + f[v[k] * fs];
  }
}

// ─── Exact fast path of the separable EDT for a BINARY mask ─────────────────
//
// sdf_nav._edt2 runs edt1d_line down every column and then across every row of
// f = mask ? 0 : EDT_INF. That input structure makes most of the generic
// kernel's work provably irrelevant, and the code below computes the SAME
// doubles with far less of it (nav_test's NavEdtExact compares it bit-for-bit
// against a verbatim copy of the column-then-row kernel on randomized grids):
//
// 1. Column pass. In a column holding at least one seed (f == 0), the lower
//    envelope of the seed parabolas is exact — their breakpoints
//    (q^2 - v^2) / (2q - 2v) = (q + v) / 2 are half-integers the double
//    division represents exactly — while any EDT_INF parabola's breakpoint
//    against a seed lies at |s| ~ 1e20 / (2n), far outside [0, n), so it never
//    wins a cell. The output is therefore exactly (distance to the nearest seed
//    in the column)^2, which two linear integer sweeps produce (cache-friendly,
//    row by row, instead of a stride-`cols` walk per column). A column with NO
//    seed is all-EDT_INF; its output (1e20 plus envelope rounding) depends only
//    on its length and is computed once with the generic kernel.
//
// 2. Row pass. Its input is the column output: exact small squares from seeded
//    columns and values >= 1e20 from seedless ones. By the same bound, every
//    breakpoint between a >= 1e20 parabola and a finite one lies beyond
//    ~1e20 / (2n): a >= 1e20 parabola is never selected for q in [0, n), it
//    never pops a finite one, and the next finite parabola pops it (and any
//    other >= 1e20 ones above the last finite entry) before comparing against
//    the finite breakpoints, which it therefore sees unchanged. So running the
//    kernel's arithmetic — identical expressions, comparisons and order — over
//    just the seeded columns reproduces every output bit. When NO column has a
//    seed the whole grid is seed-free and each row falls back to the generic
//    kernel.
//
// All buffers are per-thread scratch reused across calls (no per-line or
// per-call allocation beyond the returned result), so cvc::nav's build_sdf_batch
// and pooled sim_world rebuild keep one set per worker.

constexpr int kEdtFar = 1 << 29; // "no seed yet" run length; kEdtFar + rows < INT_MAX

struct EdtScratch {
  std::vector<int> up;              // [rows*cols] nearest opposite-class cell at/above, in rows
  std::vector<int> run_s, run_n;    // [cols] running distance to the nearest seed / non-seed
  std::vector<int> g_s, g_n;        // [cols] this row's column distances
  std::vector<int> fin_s, fin_n;    // columns holding a seed / a non-seed, ascending
  std::vector<int> v;               // envelope stack, [max(rows, cols) + 1]
  std::vector<double> z, hv, fv;    // breakpoints, f + q^2, f at each stack entry
  std::vector<double> colinf, line; // seed-free fallback: all-EDT_INF column output, a line
  bool colinf_ok = false;

  void fit(int rows, int cols) {
    const std::size_t n = static_cast<std::size_t>(rows) * cols;
    const std::size_t L = static_cast<std::size_t>(std::max(rows, cols)) + 1;
    if (up.size() < n)
      up.resize(n);
    for (auto *b : {&run_s, &run_n, &g_s, &g_n})
      if (b->size() < static_cast<std::size_t>(cols))
        b->resize(cols);
    if (v.size() < L)
      v.resize(L);
    for (auto *b : {&z, &hv, &fv, &line})
      if (b->size() < L)
        b->resize(L);
    if (colinf.size() < static_cast<std::size_t>(rows))
      colinf.resize(rows);
    colinf_ok = false;
  }
};

// Row pass restricted to the seeded columns `fin` (see (2) above). `g` holds
// this row's column-pass distances (valid at the fin columns). The expressions
// are edt1d_line's with f[q] = (double)(g*g) — the column pass's exact output —
// and h = f + q^2 kept per stack entry instead of re-read through the stride.
void edt_row_seeded(const int *g, const int *fin, int nfin, int cols, double *out, EdtScratch &S) {
  int *v = S.v.data();
  double *z = S.z.data(), *hv = S.hv.data(), *fv = S.fv.data();
  int k = 0;
  {
    const int q = fin[0];
    const double f = static_cast<double>(g[q] * g[q]);
    v[0] = q;
    fv[0] = f;
    hv[0] = f + static_cast<double>(q * q);
  }
  z[0] = -EDT_INF;
  z[1] = EDT_INF;
  for (int j = 1; j < nfin; ++j) {
    const int q = fin[j];
    const double f = static_cast<double>(g[q] * g[q]);
    const double h = f + static_cast<double>(q * q);
    double s = (h - hv[k]) / static_cast<double>(2 * q - 2 * v[k]);
    while (s <= z[k]) {
      --k;
      s = (h - hv[k]) / static_cast<double>(2 * q - 2 * v[k]);
    }
    ++k;
    v[k] = q;
    fv[k] = f;
    hv[k] = h;
    z[k] = s;
    z[k + 1] = EDT_INF;
  }
  k = 0;
  for (int q = 0; q < cols; ++q) {
    while (z[k + 1] < q)
      ++k;
    const int diff = q - v[k];
    out[q] = static_cast<double>(diff * diff) + fv[k];
  }
}

// Row r of a seed-free grid: every column output is colinf[r], so the row pass
// is the generic kernel over that constant line.
void edt_row_seedfree(int r, int rows, int cols, double *out, EdtScratch &S) {
  if (!S.colinf_ok) {
    std::fill(S.line.begin(), S.line.begin() + rows, EDT_INF);
    edt1d_line(S.line.data(), rows, 1, S.colinf.data(), 1, S.v.data(), S.z.data());
    S.colinf_ok = true;
  }
  std::fill(S.line.begin(), S.line.begin() + cols, S.colinf[r]);
  edt1d_line(S.line.data(), cols, 1, out, 1, S.v.data(), S.z.data());
}

// One row of the top-down sweep. rs / rn are the per-column run lengths to the
// nearest seed / non-seed at or above this row (kEdtFar + k when none yet); u
// receives the run to the nearest cell of the OTHER class. Branch-free and
// restrict-qualified so it vectorizes.
void edt_sweep_down(const std::uint8_t *__restrict m, int *__restrict rs, int *__restrict rn,
                    int *__restrict u, int cols) {
  for (int c = 0; c < cols; ++c) {
    const int s = m[c] != 0;
    const int a = (rs[c] + 1) & (s - 1); // 0 at a seed, else one row further
    const int b = (rn[c] + 1) & -s;      // 0 at a non-seed, else one row further
    rs[c] = a;
    rn[c] = b;
    u[c] = a + b; // exactly one of a, b is 0
  }
}

// One row of the bottom-up sweep: advances the runs as above and combines them
// with the top-down run u into the row's column-pass distances — to the nearest
// seed (gs, 0 at a seed) and to the nearest non-seed (gn, 0 at a non-seed).
void edt_sweep_up(const std::uint8_t *__restrict m, const int *__restrict u, int *__restrict rs,
                  int *__restrict rn, int *__restrict gs, int *__restrict gn, int cols) {
  for (int c = 0; c < cols; ++c) {
    const int s = m[c] != 0;
    const int a = (rs[c] + 1) & (s - 1);
    const int b = (rn[c] + 1) & -s;
    rs[c] = a;
    rn[c] = b;
    gs[c] = std::min(u[c], a);
    gn[c] = std::min(u[c], b);
  }
}

// The binary EDT of `mask` (seed = nonzero) and, when want_n, of its complement
// (seed = zero), one output row at a time. For each row r (bottom-up) it asks
// dst(r, ds, dn) where to write the two rows (dn is ignored unless want_n), fills
// them, then calls done(r). Seeds of the complement are exactly the non-seeds,
// so one forward array serves both: up[i] is the run length to the nearest cell
// of the OTHER class at or above i (a cell's own class is at distance 0).
template <class Dst, class Done>
void edt_binary(const std::uint8_t *mask, int rows, int cols, bool want_n, EdtScratch &S, Dst dst,
                Done done) {
  S.fit(rows, cols);
  int *up = S.up.data();
  int *rs = S.run_s.data(), *rn = S.run_n.data();
  std::fill(rs, rs + cols, kEdtFar);
  std::fill(rn, rn + cols, kEdtFar);
  for (int r = 0; r < rows; ++r) { // forward (top-down) sweep
    const std::uint8_t *m = mask + static_cast<std::size_t>(r) * cols;
    int *u = up + static_cast<std::size_t>(r) * cols;
    edt_sweep_down(m, rs, rn, u, cols);
  }
  // A column holds a seed (non-seed) iff its run is finite after the last row.
  S.fin_s.clear();
  S.fin_n.clear();
  for (int c = 0; c < cols; ++c) {
    if (rs[c] < kEdtFar)
      S.fin_s.push_back(c);
    if (rn[c] < kEdtFar)
      S.fin_n.push_back(c);
  }
  const int ns = static_cast<int>(S.fin_s.size()), nn = static_cast<int>(S.fin_n.size());
  std::fill(rs, rs + cols, kEdtFar); // reused as the backward (bottom-up) runs
  std::fill(rn, rn + cols, kEdtFar);
  int *gs = S.g_s.data(), *gn = S.g_n.data();
  for (int r = rows - 1; r >= 0; --r) {
    const std::uint8_t *m = mask + static_cast<std::size_t>(r) * cols;
    const int *u = up + static_cast<std::size_t>(r) * cols;
    edt_sweep_up(m, u, rs, rn, gs, gn, cols);
    double *ds = nullptr, *dn = nullptr;
    dst(r, ds, dn);
    if (ns)
      edt_row_seeded(gs, S.fin_s.data(), ns, cols, ds, S);
    else
      edt_row_seedfree(r, rows, cols, ds, S);
    if (want_n) {
      if (nn)
        edt_row_seeded(gn, S.fin_n.data(), nn, cols, dn, S);
      else
        edt_row_seedfree(r, rows, cols, dn, S);
    }
    done(r);
  }
}

} // namespace

std::vector<double> edt2_squared(const std::uint8_t *mask, int rows, int cols) {
  if (rows <= 0 || cols <= 0)
    return {};
  std::vector<double> out(static_cast<std::size_t>(rows) * cols);
  // Separable, as sdf_nav._edt2: columns (the y-axis) first, then rows — via the
  // exact binary fast path above.
  // The call's own scratch (sized to the grid, as the output is): nothing kept between calls.
  EdtScratch scratch;
  edt_binary(
      mask, rows, cols, false, scratch,
      [&](int r, double *&ds, double *&) { ds = out.data() + static_cast<std::size_t>(r) * cols; },
      [](int) {});
  return out;
}

void edt2_squared_both(const std::uint8_t *mask, int rows, int cols, double *to_seed,
                       double *to_other) {
  if (rows <= 0 || cols <= 0)
    return;
  EdtScratch scratch;
  edt_binary(
      mask, rows, cols, true, scratch,
      [&](int r, double *&ds, double *&dn) {
        ds = to_seed + static_cast<std::size_t>(r) * cols;
        dn = to_other + static_cast<std::size_t>(r) * cols;
      },
      [](int) {});
}

void edt2_squared_rows(const std::uint8_t *mask, int rows, int cols, bool want_other,
                       const edt_row_dst &dst, const edt_row_done &done) {
  if (rows <= 0 || cols <= 0)
    return;
  EdtScratch scratch;
  edt_binary(mask, rows, cols, want_other, scratch, dst, done);
}

} // namespace algo
} // namespace cvc
