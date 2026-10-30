/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See sliced_ldlt.h.

#include "sliced_ldlt.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace cvc {
namespace sliced_detail {

void multiply_add(const csc &a, const std::vector<double> &x, std::vector<double> &y) {
  for (int j = 0; j < a.n; ++j) {
    const double xj = x[std::size_t(j)];
    if (xj == 0.0)
      continue;
    for (int p = a.p[std::size_t(j)]; p < a.p[std::size_t(j) + 1]; ++p)
      y[std::size_t(a.i[std::size_t(p)])] += a.x[std::size_t(p)] * xj;
  }
}

// --- csc_builder --------------------------------------------------------------------------------

csc_builder::csc_builder(int n, std::vector<int> rows, std::vector<int> cols,
                         std::vector<double> values)
    : _n(n), _r(std::move(rows)), _c(std::move(cols)), _v(std::move(values)) {
  _out.n = n;
  _out.p.assign(std::size_t(n) + 1, 0);
}

bool csc_builder::step(std::size_t budget) {
  const std::size_t nt = _r.size();
  while (budget > 0) {
    if (_stage == 0) { // count each column's entries
      const std::size_t end = std::min(nt, _pos + budget);
      for (std::size_t t = _pos; t < end; ++t)
        ++_out.p[std::size_t(_c[t]) + 1];
      budget -= end - _pos;
      _pos = end;
      if (_pos < nt)
        return false;
      for (int j = 0; j < _n; ++j) // O(n), once
        _out.p[std::size_t(j) + 1] += _out.p[std::size_t(j)];
      _next.assign(_out.p.begin(), _out.p.end() - 1);
      _out.i.resize(nt);
      _out.x.resize(nt);
      _stage = 1;
      _pos = 0;
    } else if (_stage == 1) { // place them
      const std::size_t end = std::min(nt, _pos + budget);
      for (std::size_t t = _pos; t < end; ++t) {
        const int at = _next[std::size_t(_c[t])]++;
        _out.i[std::size_t(at)] = _r[t];
        _out.x[std::size_t(at)] = _v[t];
      }
      budget -= end - _pos;
      _pos = end;
      if (_pos < nt)
        return false;
      _r = _c = std::vector<int>();
      _v = std::vector<double>();
      _next.assign(1, 0); // _next[0]: where the merged entries go next
      _stage = 2;
      _pos = 0;
    } else if (_stage == 2) { // sort each column by row and sum duplicates, compacting
      std::vector<std::pair<int, double>> col;
      while (budget > 0 && _pos < std::size_t(_n)) {
        const std::size_t j = _pos++;
        const int b = _out.p[j], e = _out.p[j + 1];
        col.clear();
        for (int q = b; q < e; ++q)
          col.emplace_back(_out.i[std::size_t(q)], _out.x[std::size_t(q)]);
        std::sort(col.begin(), col.end(),
                  [](const auto &l, const auto &r) { return l.first < r.first; });
        int w = _next[0];
        _out.p[j] = w;
        for (std::size_t q = 0; q < col.size(); ++q) {
          if (q > 0 && col[q].first == col[q - 1].first) {
            _out.x[std::size_t(w) - 1] += col[q].second;
            continue;
          }
          _out.i[std::size_t(w)] = col[q].first;
          _out.x[std::size_t(w)] = col[q].second;
          ++w;
        }
        // The next column's start is read before it is overwritten: p[j + 1] still holds the
        // unmerged start, kept in _out.p until the loop reaches it.
        _next[0] = w;
        const std::size_t units = col.size() + 1;
        budget = units >= budget ? 0 : budget - units;
      }
      if (_pos < std::size_t(_n))
        return false;
      _out.p[std::size_t(_n)] = _next[0];
      _out.i.resize(std::size_t(_next[0]));
      _out.x.resize(std::size_t(_next[0]));
      _stage = 3;
    } else {
      return true;
    }
  }
  return _stage == 3;
}

double csc_builder::progress() const {
  const double nt = std::max<double>(1.0, double(_r.size()));
  switch (_stage) {
  case 0:
    return 0.3 * double(_pos) / nt;
  case 1:
    return 0.3 + 0.3 * double(_pos) / nt;
  case 2:
    return 0.6 + 0.4 * double(_pos) / std::max(1.0, double(_n));
  default:
    return 1.0;
  }
}

csc csc_builder::take() { return std::move(_out); }

// --- ldlt ---------------------------------------------------------------------------------------

ldlt::ldlt(const csc *a, std::vector<int> perm) : _a(a), _perm(std::move(perm)) {
  _n = a->n;
  if (int(_perm.size()) != _n)
    throw std::runtime_error("ldlt: the permutation does not match the matrix");
  _pinv.assign(std::size_t(_n), 0);
  for (int k = 0; k < _n; ++k)
    _pinv[std::size_t(_perm[std::size_t(k)])] = k;
  _parent.assign(std::size_t(_n), -1);
  _flag.assign(std::size_t(_n), -1);
  _lnz.assign(std::size_t(_n), 0);
}

bool ldlt::step(std::size_t budget) {
  const csc &a = *_a;
  while (budget > 0 && _stage < 2) {
    if (_stage == 0) {
      // The elimination tree and each column's count, a column at a time.
      const int k = _k++;
      _parent[std::size_t(k)] = -1;
      _flag[std::size_t(k)] = k;
      _lnz[std::size_t(k)] = 0;
      const int kk = _perm[std::size_t(k)];
      std::size_t units = 1;
      for (int p = a.p[std::size_t(kk)]; p < a.p[std::size_t(kk) + 1]; ++p) {
        int i = _pinv[std::size_t(a.i[std::size_t(p)])];
        ++units;
        if (i >= k)
          continue;
        for (; _flag[std::size_t(i)] != k; i = _parent[std::size_t(i)]) {
          if (_parent[std::size_t(i)] == -1)
            _parent[std::size_t(i)] = k;
          ++_lnz[std::size_t(i)];
          _flag[std::size_t(i)] = k;
          ++units;
        }
      }
      budget = units >= budget ? 0 : budget - units;
      if (_k == _n) {
        _lp.assign(std::size_t(_n) + 1, 0);
        _expected = 1;
        for (int j = 0; j < _n; ++j) {
          _lp[std::size_t(j) + 1] = _lp[std::size_t(j)] + _lnz[std::size_t(j)];
          _expected += std::uint64_t(_lnz[std::size_t(j)]) * std::uint64_t(_lnz[std::size_t(j)]) +
                       std::uint64_t(_lnz[std::size_t(j)]) + 1;
        }
        _li.assign(std::size_t(_lp[std::size_t(_n)]), 0);
        _lx.assign(std::size_t(_lp[std::size_t(_n)]), 0.0);
        _d.assign(std::size_t(_n), 0.0);
        _y.assign(std::size_t(_n), 0.0);
        _pattern.assign(std::size_t(_n), 0);
        std::fill(_flag.begin(), _flag.end(), -1);
        std::fill(_lnz.begin(), _lnz.end(), 0);
        _stage = 1;
        _k = 0;
      }
      continue;
    }
    // Numeric: row k of L (column k of L^T) from column k of the permuted A, along its reach.
    const int k = _k++;
    _y[std::size_t(k)] = 0.0;
    int top = _n;
    _flag[std::size_t(k)] = k;
    _lnz[std::size_t(k)] = 0;
    const int kk = _perm[std::size_t(k)];
    std::size_t units = 1;
    for (int p = a.p[std::size_t(kk)]; p < a.p[std::size_t(kk) + 1]; ++p) {
      int i = _pinv[std::size_t(a.i[std::size_t(p)])];
      if (i > k)
        continue;
      _y[std::size_t(i)] += a.x[std::size_t(p)];
      int len = 0;
      for (; _flag[std::size_t(i)] != k; i = _parent[std::size_t(i)]) {
        _pattern[std::size_t(len++)] = i;
        _flag[std::size_t(i)] = k;
      }
      while (len > 0)
        _pattern[std::size_t(--top)] = _pattern[std::size_t(--len)];
    }
    double dk = _y[std::size_t(k)];
    _y[std::size_t(k)] = 0.0;
    for (; top < _n; ++top) {
      const int i = _pattern[std::size_t(top)];
      const double yi = _y[std::size_t(i)];
      _y[std::size_t(i)] = 0.0;
      const int p2 = _lp[std::size_t(i)] + _lnz[std::size_t(i)];
      for (int p = _lp[std::size_t(i)]; p < p2; ++p)
        _y[std::size_t(_li[std::size_t(p)])] -= _lx[std::size_t(p)] * yi;
      const double lki = yi / _d[std::size_t(i)];
      dk -= lki * yi;
      _li[std::size_t(p2)] = k;
      _lx[std::size_t(p2)] = lki;
      ++_lnz[std::size_t(i)];
      units += std::size_t(p2 - _lp[std::size_t(i)]) + 1;
    }
    if (dk == 0.0 || !std::isfinite(dk))
      throw std::runtime_error("the system is singular: is every connected part of the mesh held "
                               "by a Dirichlet condition?");
    _d[std::size_t(k)] = dk;
    _work += units;
    budget = units >= budget ? 0 : budget - units;
    if (_k == _n) {
      _y = std::vector<double>();
      _pattern = std::vector<int>();
      _flag = std::vector<int>();
      _stage = 2;
    }
  }
  return _stage == 2;
}

double ldlt::progress() const {
  if (_stage == 0)
    return _n > 0 ? 0.1 * double(_k) / double(_n) : 0.0;
  if (_stage == 1)
    return 0.1 + 0.9 * std::min(1.0, double(_work) / double(_expected));
  return 1.0;
}

void ldlt::start_solve(const std::vector<double> &b) {
  _x.assign(std::size_t(_n), 0.0);
  for (int k = 0; k < _n; ++k)
    _x[std::size_t(k)] = b[std::size_t(_perm[std::size_t(k)])];
  _solve_stage = 0;
  _j = 0;
}

bool ldlt::solve_step(std::size_t budget) {
  while (budget > 0 && _solve_stage < 3) {
    if (_solve_stage == 0) { // L y = b
      const int j = _j++;
      const double xj = _x[std::size_t(j)];
      for (int p = _lp[std::size_t(j)]; p < _lp[std::size_t(j) + 1]; ++p)
        _x[std::size_t(_li[std::size_t(p)])] -= _lx[std::size_t(p)] * xj;
      const std::size_t units = std::size_t(_lp[std::size_t(j) + 1] - _lp[std::size_t(j)]) + 1;
      budget = units >= budget ? 0 : budget - units;
      if (_j == _n) {
        _solve_stage = 1;
        _j = 0;
      }
    } else if (_solve_stage == 1) { // D z = y
      const int end = int(std::min<std::size_t>(std::size_t(_n), std::size_t(_j) + budget));
      for (int j = _j; j < end; ++j)
        _x[std::size_t(j)] /= _d[std::size_t(j)];
      budget -= std::size_t(end - _j);
      _j = end;
      if (_j == _n) {
        _solve_stage = 2;
        _j = _n;
      }
    } else { // L^T x = z, then back to A's order
      const int j = --_j;
      double xj = _x[std::size_t(j)];
      for (int p = _lp[std::size_t(j)]; p < _lp[std::size_t(j) + 1]; ++p)
        xj -= _lx[std::size_t(p)] * _x[std::size_t(_li[std::size_t(p)])];
      _x[std::size_t(j)] = xj;
      const std::size_t units = std::size_t(_lp[std::size_t(j) + 1] - _lp[std::size_t(j)]) + 1;
      budget = units >= budget ? 0 : budget - units;
      if (_j == 0) {
        _w.assign(std::size_t(_n), 0.0);
        for (int k = 0; k < _n; ++k)
          _w[std::size_t(_perm[std::size_t(k)])] = _x[std::size_t(k)];
        _x.swap(_w);
        _solve_stage = 3;
      }
    }
  }
  return _solve_stage == 3;
}

void ldlt::solve(std::vector<double> &b) {
  start_solve(b);
  while (!solve_step(std::size_t(1) << 30)) {
  }
  b = _x;
}

} // namespace sliced_detail
} // namespace cvc
