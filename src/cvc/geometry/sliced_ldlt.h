/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A sparse symmetric LDL^T factorization and solve, a slice at a time (internal to libcvc's FEM
// code). The up-looking algorithm (as in T. A. Davis's LDL): an elimination-tree pass for the
// column counts, then each column of L from the rows of A above the diagonal, along the column's
// reach in the tree. Every loop is over columns, so a step can stop between any two of them. The
// fill-reducing ordering is given (an AMD ordering, as Eigen::SimplicialLDLT computes one), so the
// factor is the one Eigen's would be, up to rounding.

#ifndef CVC_GEOMETRY_SLICED_LDLT_H
#define CVC_GEOMETRY_SLICED_LDLT_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace cvc {
namespace sliced_detail {

// An n x n matrix in compressed-column form: column j's rows are i[p[j] .. p[j+1]), values x.
// For the factorization A must be symmetric with both triangles stored (only the entries on or
// above the diagonal of the permuted matrix are read).
struct csc {
  int n = 0;
  std::vector<int> p, i;
  std::vector<double> x;
};

// y += A x (A as stored).
void multiply_add(const csc &a, const std::vector<double> &x, std::vector<double> &y);

// Builds a csc from (row, col, value) triplets, summing duplicates, a slice at a time: a count
// pass over the triplets, a prefix sum, a fill pass, then each column's rows sorted and merged.
class csc_builder {
public:
  csc_builder(int n, std::vector<int> rows, std::vector<int> cols, std::vector<double> values);
  // Do about `budget` units (a triplet counted or placed, a column sorted and merged per entry);
  // true once built.
  bool step(std::size_t budget);
  double progress() const;
  csc take();

private:
  int _n;
  std::vector<int> _r, _c;
  std::vector<double> _v;
  int _stage = 0;
  std::size_t _pos = 0;
  csc _out;
  std::vector<int> _next;
};

// P A P^T = L D L^T for a given permutation (perm[k] = the row/column of A that comes k-th),
// a column at a time. A unit is about one multiply-add.
class ldlt {
public:
  ldlt(const csc *a, std::vector<int> perm);
  // True once factored. Throws std::runtime_error on a zero pivot (a singular matrix).
  bool step(std::size_t budget);
  bool done() const { return _stage == 2; }
  double progress() const;
  const char *phase() const { return _stage == 0 ? "symbolic" : "numeric"; }

  // Solve A x = b in place, a slice at a time: start_solve(b), then solve_step(budget) until it
  // returns true; the result is solution(). Several solves may follow one factorization.
  void start_solve(const std::vector<double> &b);
  bool solve_step(std::size_t budget);
  const std::vector<double> &solution() const { return _x; }
  // The whole solve at once (for small batches).
  void solve(std::vector<double> &b);

private:
  const csc *_a;
  std::vector<int> _perm, _pinv;
  int _n = 0;
  int _stage = 0; // 0 symbolic, 1 numeric, 2 done
  int _k = 0;
  // symbolic
  std::vector<int> _parent, _flag, _lnz;
  // numeric
  std::vector<int> _lp, _li, _pattern;
  std::vector<double> _lx, _d, _y;
  std::uint64_t _work = 0, _expected = 1;
  // solve
  std::vector<double> _x, _w;
  int _solve_stage = 3, _j = 0;
};

} // namespace sliced_detail
} // namespace cvc

#endif // CVC_GEOMETRY_SLICED_LDLT_H
