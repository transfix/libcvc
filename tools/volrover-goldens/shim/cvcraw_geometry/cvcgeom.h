// The piece of legacy cvcraw_geometry that legacy SecondaryStructures uses: cvcgeom_t's point,
// triangle, line and colour arrays (skel.cpp reads the input surface's points and triangles; op.cpp
// fills the helix, sheet, skeleton and curve geometries). The real cvcgeom_t (transfix/volrover
// inc/cvcraw_geometry) needs volrover's core library to build. Part of tools/volrover-goldens:
// never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_CVCGEOM_H
#define VOLROVER_GOLDENS_SHIM_CVCGEOM_H

#include <array>
#include <vector>

namespace cvcraw_geometry {

class cvcgeom_t {
public:
  typedef std::array<double, 3> point_t;
  typedef std::array<double, 3> color_t;
  typedef std::array<unsigned int, 3> triangle_t;
  typedef std::array<unsigned int, 2> line_t;

  std::vector<point_t> &points() { return _points; }
  const std::vector<point_t> &points() const { return _points; }
  std::vector<color_t> &colors() { return _colors; }
  const std::vector<color_t> &colors() const { return _colors; }
  std::vector<triangle_t> &triangles() { return _triangles; }
  const std::vector<triangle_t> &triangles() const { return _triangles; }
  std::vector<line_t> &lines() { return _lines; }
  const std::vector<line_t> &lines() const { return _lines; }

private:
  std::vector<point_t> _points;
  std::vector<color_t> _colors;
  std::vector<triangle_t> _triangles;
  std::vector<line_t> _lines;
};

} // namespace cvcraw_geometry

#endif
