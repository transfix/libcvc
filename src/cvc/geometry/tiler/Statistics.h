/*
  Copyright 2008-2012, 2026 The University of Texas at Austin

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

// Vendored from VolumeRover's ContourTiler (John Edwards, UT Austin; transfix/volrover
// src/ContourTiler at 1cf0fbd7) and ported onto libcvc's tiler kernel and cvc::planar
// (VOLROVER-LIBRARY-PORTS-ROADMAP.md §3).

#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <iostream>

namespace cvc::tiler {

/// /f$\sqrt{\frac{1}{N} \left(\sum_{i=1}^N x_i^2\right) - \overline{x}^2}/f$
/// is used to calculate the standard deviation.
///
/// No checks are made for overflow!  This class should not be used for critical
/// calculations.
template <typename T>
class Statistics
{
public:
  Statistics() : _N(0), _sum(0), _squares(0) {}
  ~Statistics() {}

  void add(T sample)
  {
    ++_N;
    _sum += sample;
    _squares += (sample * sample);
    _min = (_min<sample)?_min:sample;
    _max = (_max>sample)?_max:sample;
  }

  size_t N() const
  { return _N; }

  T mean() const
  { return _sum / (T) _N; }

  T variance() const
  {
    T avg = mean();
    return _squares / (T) _N - avg * avg;
  }

  T std_dev() const
  { return sqrt(variance()); }

  T min() const
  { return _min; }

  T max() const
  { return _max; }

private:
  size_t _N;
  T _sum;
  T _squares;
  T _min;
  T _max;
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Statistics<T>& stats)
{
  out << "Sample size = " << stats.N() << " Average = " << stats.mean() << " Standard deviation = " << stats.std_dev()
    << " Min = " << stats.min() << " Max = " << stats.max();
  return out;
}

} // namespace cvc::tiler

#endif
