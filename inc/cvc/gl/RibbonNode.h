/*
  Copyright 2026 The University of Texas at Austin

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

#ifndef CVC_GL_RIBBON_NODE_H
#define CVC_GL_RIBBON_NODE_H

#include <cvc/gl/StreamingGeometryNode.h>
#include <mutex>
#include <vector>

namespace cvc {
namespace gl {

// ---------------------------------------------------------------------------
// RibbonNode -- a flat ribbon (a vehicle track, a planned route) along a centre
// line of points, streamed into reserved capacity.
//
// Each centre point k owns two vertices, left (2k) and right (2k + 1), offset
// halfWidth either side of the line in the XY plane (normal +Z, at the centre's
// own z); segment k is triangles 2k and 2k + 1. Joints are mitred (the offset
// follows the bisector, stretched up to 2x so the ribbon keeps its width round
// a bend).
//
//   append(x, y, z)          a TRACK: writes the new centre's two vertices and
//                            re-mitres the previous centre -- one 4-vertex
//                            (48-byte) upload -- and grows the draw range.
//   assign(xyz, n)           a ROUTE replan: rewrites centres [0, n) in one
//                            sub-range upload; a longer old route's stale tail
//                            is hidden by the draw range, never rewritten.
//   setVisibleCenters(a, b)  show only centre-line positions [a, b], in
//                            FRACTIONAL centre units (centerAtArcLength maps arc
//                            length to them). The whole part becomes the draw
//                            range, the fraction an exact cut by a fragment
//                            clip (uniforms) -- per frame, with no upload.
//
// CAPACITY: append/assign past the capacity GROW the ribbon (capacity doubles,
// or to n for a longer assign) instead of throwing: the ribbon owns its
// topology, and a track must not lose history. A growth is one full re-upload
// (counted in streamStats().fullUploads); size the capacity so it is rare.
//
// Any thread, like StreamingGeometryNode; calls on one ribbon serialise on its
// own lock. Material (colour, ambient/diffuse) is GeometryNode's.
class RibbonNode : public StreamingGeometryNode {
public:
  // setVisibleCenters `last` meaning "to the end of the line".
  static constexpr double kToEnd = 1.0e300;

  RibbonNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
             std::size_t capacityCenters, float halfWidth, const cvc::bounding_box &reservedBounds,
             StreamingMapperKind mapper = StreamingMapperKind::Auto);

  // Append one centre point; returns its index.
  std::size_t append(float x, float y, float z);
  // Replace the whole centre line with n centres (3n floats). Resets the
  // visible window to the whole line. n == 0 is clearCenters().
  void assign(const float *xyz, std::size_t nCenters);
  // Show only [first, last] of the centre line (fractional centre indices,
  // clamped to it). Persistent across append; reset by assign/clearCenters.
  void setVisibleCenters(double first, double last = kToEnd);
  // Drop every centre: nothing drawn, nothing uploaded.
  void clearCenters();

  std::size_t centerCount() const;
  std::size_t centerCapacity() const;
  float halfWidth() const;
  // Change the width: rewrites (one upload of) every assigned centre.
  void setHalfWidth(float halfWidth);
  // Length of the centre line, and the fractional centre index at arc length
  // s along it (clamped), for setVisibleCenters.
  double arcLength() const;
  double centerAtArcLength(double s) const;
  // The two vertices of centre k as written ([lx, ly, lz, rx, ry, rz]).
  std::vector<float> centerVertices(std::size_t k) const;

private:
  static StreamingLayout ribbonLayout(std::size_t centers, const cvc::bounding_box &bounds,
                                      StreamingMapperKind kind);
  void growLocked(std::size_t minCenters);
  void computeVerticesLocked(std::size_t k, float out[6]) const;
  void writeCentersLocked(std::size_t first, std::size_t last); // vertices of [first, last]
  void updateWindowLocked();

  mutable std::mutex m_ribbonMutex;
  std::vector<float> m_centers; // 3 per centre, capacity-sized
  std::vector<double> m_arc;    // cumulative arc length per centre
  std::size_t m_count = 0;
  std::size_t m_capacity = 0;
  float m_half = 1.0f;
  double m_visFirst = 0.0, m_visLast = kToEnd;
  std::vector<float> m_scratch;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_RIBBON_NODE_H
