// The piece of legacy cvcraw_geometry's cvcgeom_t that legacy PocketTunnel and SecondaryStructures
// use: the points, colours, triangles and lines of their input and output geometries (PocketTunnel
// reads its input's points and returns points, colours and triangles; SecondaryStructures' skel.cpp
// reads the input surface's points and triangles, and op.cpp fills the helix, sheet, skeleton and
// curve geometries). The real cvcgeom_t (transfix/volrover inc/cvcraw_geometry/cvcgeom.h) needs
// volrover's core library to build; its types are the same (doubles, 64-bit indices). Part of
// tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_CVCGEOM_H
#define VOLROVER_GOLDENS_SHIM_CVCGEOM_H

#include <array>
#include <cstdint>
#include <vector>

#ifndef CVCGEOM_NAMESPACE
#define CVCGEOM_NAMESPACE cvcraw_geometry
#endif

namespace CVCGEOM_NAMESPACE {

class cvcgeom_t {
public:
  typedef double scalar_t;
  typedef uint64_t index_t;
  typedef std::array<scalar_t, 3> point_t;
  typedef std::array<scalar_t, 3> color_t;
  typedef std::array<index_t, 3> triangle_t;
  typedef std::array<index_t, 2> line_t;
  typedef std::vector<point_t> points_t;
  typedef std::vector<color_t> colors_t;
  typedef std::vector<triangle_t> triangles_t;
  typedef std::vector<line_t> lines_t;

  points_t &points() { return _points; }
  colors_t &colors() { return _colors; }
  triangles_t &triangles() { return _triangles; }
  lines_t &lines() { return _lines; }
  const points_t &points() const { return _points; }
  const colors_t &colors() const { return _colors; }
  const triangles_t &triangles() const { return _triangles; }
  const lines_t &lines() const { return _lines; }

private:
  points_t _points;
  colors_t _colors;
  triangles_t _triangles;
  lines_t _lines;
};

} // namespace CVCGEOM_NAMESPACE

#endif
