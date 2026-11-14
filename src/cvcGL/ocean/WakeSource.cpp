/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "WakeSource.h"

#include <algorithm>
#include <cmath>
#include <limits>

WakeSource::WakeSource(const cvc::ocean::ewave_params &window, int samples) {
  m_grid.nx = m_grid.ny = window.n;
  m_grid.dx = window.dx;
  m_grid.origin_x = window.origin_x;
  m_grid.origin_y = window.origin_y;
  m_grid.samples = samples;
  m_zeta.assign(m_grid.count(), 0.0);
  m_last.assign(m_grid.count(), 0.0);
  m_work.assign(m_grid.count(), 0.0);
  m_row.assign(m_grid.count(), 0.0);
}

void WakeSource::shift(int di, int dj) {
  if (di == 0 && dj == 0)
    return;
  m_grid.origin_x += double(di) * m_grid.dx;
  m_grid.origin_y += double(dj) * m_grid.dx;
  if (!m_haveLast)
    return;
  // Cell (i, j) takes what was at (i + di, j + dj); a cell that enters is marked NaN, and
  // finish() gives it this step's zeta as its last.
  const int n = m_grid.nx;
  const double fresh = std::numeric_limits<double>::quiet_NaN();
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const int si = i + di, sj = j + dj;
      m_work[std::size_t(j) * n + i] =
          si >= 0 && si < n && sj >= 0 && sj < n ? m_last[std::size_t(sj) * n + si] : fresh;
    }
  m_last.swap(m_work);
}

void WakeSource::begin() {
  std::fill(m_zeta.begin(), m_zeta.end(), 0.0);
  m_volume = 0.0;
}

double WakeSource::add(const cvc::physics::immersion_body &body, const cvc::physics::vec3 &position,
                       const cvc::physics::quat &rotation,
                       const cvc::physics::immersion_water &water) {
  const double v = cvc::physics::add_immersion(body, position, rotation, water, m_grid, m_zeta);
  m_volume += v;
  return v;
}

bool WakeSource::finish(std::vector<double> &dh) {
  const int n = m_grid.nx;
  if (!m_haveLast) {
    m_last = m_zeta;
    m_haveLast = true;
    return false;
  }
  // The raw source and the box it is non-zero in.
  int i0 = n, i1 = -1, j0 = n, j1 = -1;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const std::size_t c = std::size_t(j) * n + i;
      double &last = m_last[c];
      if (std::isnan(last))
        last = m_zeta[c];
      const double d = -(m_zeta[c] - last);
      m_work[c] = d;
      if (d != 0.0) {
        i0 = std::min(i0, i), i1 = std::max(i1, i);
        j0 = std::min(j0, j), j1 = std::max(j1, j);
      }
    }
  m_last.swap(m_zeta);
  if (i1 < 0)
    return false;
  // [1 2 1] / 4 along x, then along y, over the box grown by a cell (cells past the window's
  // edge are dropped: they are in the sponge).
  dh.assign(m_grid.count(), 0.0);
  i0 = std::max(0, i0 - 1), i1 = std::min(n - 1, i1 + 1);
  j0 = std::max(0, j0 - 1), j1 = std::min(n - 1, j1 + 1);
  std::vector<double> &row = m_row;
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      const std::size_t c = std::size_t(j) * n + i;
      const double l = i > 0 ? m_work[c - 1] : 0.0, r = i + 1 < n ? m_work[c + 1] : 0.0;
      row[c] = 0.25 * l + 0.5 * m_work[c] + 0.25 * r;
    }
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      const std::size_t c = std::size_t(j) * n + i;
      const double b = j > 0 ? row[c - n] : 0.0, t = j + 1 < n ? row[c + n] : 0.0;
      dh[c] = 0.25 * b + 0.5 * row[c] + 0.25 * t;
    }
  // The row pass's box is cleared for the next step, which reads only its own box.
  for (int j = j0; j <= j1; ++j)
    std::fill(row.begin() + std::size_t(j) * n + i0, row.begin() + std::size_t(j) * n + i1 + 1,
              0.0);
  return true;
}
