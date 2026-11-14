// The one piece of legacy cvcraw_geometry that legacy Skeletonization uses: Geometry's triangle
// vertex array, which skeletonize() reads as its point cloud (m_NumTriVerts points, three floats
// each in m_TriVerts). The real Geometry (transfix/volrover inc/cvcraw_geometry) needs LBIE and
// volrover's core library to build. Part of tools/volrover-goldens: never built into libcvc.

#ifndef VOLROVER_GOLDENS_SHIM_GEOMETRY_H
#define VOLROVER_GOLDENS_SHIM_GEOMETRY_H

#include <boost/shared_array.hpp>

class Geometry {
public:
  boost::shared_array<float> m_TriVerts;
  unsigned int m_NumTriVerts = 0;
};

#endif
