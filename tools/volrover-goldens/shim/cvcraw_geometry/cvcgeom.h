// The piece of legacy cvcraw_geometry's cvcgeom_t that legacy PocketTunnel uses: the points it
// reads its input from, and the points, colours and triangles of the geometry it returns. The real
// cvcgeom_t (transfix/volrover inc/cvcraw_geometry/cvcgeom.h) needs volrover's core library to
// build; its types are the same (doubles, 64-bit indices). Part of tools/volrover-goldens: never
// built into libcvc.

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
  typedef std::vector<point_t> points_t;
  typedef std::vector<color_t> colors_t;
  typedef std::vector<triangle_t> triangles_t;

  points_t &points() { return _points; }
  colors_t &colors() { return _colors; }
  triangles_t &triangles() { return _triangles; }
  const points_t &points() const { return _points; }
  const colors_t &colors() const { return _colors; }
  const triangles_t &triangles() const { return _triangles; }

private:
  points_t _points;
  colors_t _colors;
  triangles_t _triangles;
};

} // namespace CVCGEOM_NAMESPACE

#endif
