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

#ifndef __COLOR_H__
#define __COLOR_H__

namespace cvc::tiler {

class Color
{
public:
  Color() {}
  Color(double r, double g, double b) 
    : _r((unsigned char)(r*255.0)), _g((unsigned char)(g*255.0)), _b((unsigned char)(b*255.0)) {}
  ~Color() {}

  double r() const { return _r / 255.0; }
  double g() const { return _g / 255.0; }
  double b() const { return _b / 255.0; }

  bool operator==(const Color& c) const
  { return _r == c._r && _g == c._g && _b == c._b; }

private:  
  unsigned char _r, _g, _b;
};

inline std::size_t hash_value(const Color& color)
{
  std::size_t seed = 0;
  boost::hash_combine(seed, color.r());
  boost::hash_combine(seed, color.g());
  boost::hash_combine(seed, color.b());
  return seed;
}

} // namespace cvc::tiler

#endif
