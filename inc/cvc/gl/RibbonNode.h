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

#include <cstdint>
#include <cvc/gl/StreamingGeometryNode.h>
#include <memory>
#include <mutex>
#include <vector>

namespace cvc {
namespace gl {

struct RibbonValueTexture;

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
//   replaceFrom(k, xyz, n)   centres [k, k + n) replaced and any past them
//                            dropped: a line whose front is fixed and whose
//                            tail moves (a track whose last point is the live
//                            position). Uploads centres [k - 1, k + n) only;
//                            append is replaceFrom(count, p, 1).
//
// WIDTH: one half width for the whole ribbon (the constructor's, setHalfWidth),
// or one per centre (replaceFrom's halfWidths); a segment tapers between its
// two centres' widths.
//
// COLOUR: the material's (GeometryNode::setColor), or BY VALUE: setValueRamp
// colours each centre by its value (replaceFrom's values) through a ramp of up
// to kMaxRampStops stops, interpolated along each segment and drawn flat and
// unlit. The values live on the GPU as a float texture beside the points; a
// write uploads its own centres' texels (at most three sub-images), never the
// whole texture, and changing the ramp is uniforms (no upload).
//
// CAPACITY: append/assign past the capacity GROW the ribbon (capacity doubles,
// or to n for a longer assign) instead of throwing: the ribbon owns its
// topology, and a track must not lose history. A growth is one full re-upload
// (counted in streamStats().fullUploads); size the capacity so it is rare.
//
// BOUNDS: the reserved box given at construction is a MINIMUM. append, assign
// and setHalfWidth grow it to cover every centre written (plus the mitred
// width), staged with the points, so a track that drives out of its box is
// never frustum-culled or depth-clipped. It never shrinks on its own.
//
// Any thread, like StreamingGeometryNode; calls on one ribbon serialise on its
// own lock. Material (colour, ambient/diffuse) is GeometryNode's.
class RibbonNode : public StreamingGeometryNode {
public:
  // setVisibleCenters `last` meaning "to the end of the line".
  static constexpr double kToEnd = 1.0e300;
  // The most stops a value ramp holds.
  static constexpr std::size_t kMaxRampStops = 8;

  // One stop of a value ramp: at `value`, the colour (r, g, b).
  struct RampStop {
    float value = 0.0f;
    float r = 1.0f, g = 1.0f, b = 1.0f;
  };

  // What the value texture has uploaded, for tests and HUDs.
  struct ValueStats {
    std::uint64_t fullUploads = 0; // texture (re)creations: first draw, a growth
    std::uint64_t uploads = 0;     // sub-range uploads
    std::uint64_t uploadCalls = 0; // glTexSubImage2D calls they took (at most 3 each)
    std::uint64_t bytes = 0;       // bytes in both
  };

  RibbonNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
             std::size_t capacityCenters, float halfWidth, const cvc::bounding_box &reservedBounds,
             StreamingMapperKind mapper = StreamingMapperKind::Auto);
  ~RibbonNode() override;

  // Append one centre point; returns its index.
  std::size_t append(float x, float y, float z);
  // Replace the whole centre line with n centres (3n floats). Resets the
  // visible window to the whole line. n == 0 is clearCenters().
  void assign(const float *xyz, std::size_t nCenters);
  // Replace centres [first, first + n) with n centres (3n floats) and drop any
  // past them; `first` is clamped to centerCount(), so first == centerCount()
  // appends. `halfWidths` and `values` (n each, or null) are the centres' own
  // half widths (null: the ribbon's halfWidth()) and values (null: 0). Uploads
  // centres [first - 1, first + n) only (centre first - 1 is re-mitred). The
  // visible window is kept, as append keeps it.
  void replaceFrom(std::size_t first, const float *xyz, std::size_t n,
                   const float *halfWidths = nullptr, const float *values = nullptr);
  // Show only [first, last] of the centre line (fractional centre indices,
  // clamped to it). Persistent across append; reset by assign/clearCenters.
  void setVisibleCenters(double first, double last = kToEnd);
  // Drop every centre: nothing drawn, nothing uploaded.
  void clearCenters();

  std::size_t centerCount() const;
  std::size_t centerCapacity() const;
  float halfWidth() const;
  // Change the width: every centre takes it (per-centre widths included), and
  // every assigned centre is rewritten (one upload).
  void setHalfWidth(float halfWidth);
  // Centre k's own half width and value (0 past the line).
  float centerHalfWidth(std::size_t k) const;
  float centerValue(std::size_t k) const;

  // Colour by value through `stops` (ascending values; at most kMaxRampStops,
  // the rest ignored; below the first and above the last the end colours
  // hold). Empty: back to the material colour. Uniforms only, from any thread,
  // except the first non-empty ramp: it installs the shader code (before the
  // node is attached, or on the owner thread) and the values then go up whole
  // once, at the next draw.
  void setValueRamp(const std::vector<RampStop> &stops);
  std::vector<RampStop> valueRamp() const;
  ValueStats valueStats() const;
  // Length of the centre line, and the fractional centre index at arc length
  // s along it (clamped), for setVisibleCenters.
  double arcLength() const;
  double centerAtArcLength(double s) const;
  // The two vertices of centre k as written ([lx, ly, lz, rx, ry, rz]).
  std::vector<float> centerVertices(std::size_t k) const;

protected:
  // The value texture: made and updated on the render thread, bound per draw.
  void beforeDraw(vtkRenderer *renderer) override;
  void updateShaderProgram(vtkShaderProgram *program) override;

private:
  static StreamingLayout ribbonLayout(std::size_t centers, const cvc::bounding_box &bounds,
                                      StreamingMapperKind kind);
  void growLocked(std::size_t minCenters);
  void computeVerticesLocked(std::size_t k, float out[6]) const;
  void writeCentersLocked(std::size_t first, std::size_t last); // vertices of [first, last]
  void fitBoundsLocked(std::size_t first, std::size_t last);    // grow the box over them
  void updateWindowLocked();
  void replaceFromLocked(std::size_t first, const float *xyz, std::size_t n,
                         const float *halfWidths, const float *values);
  void arcFromLocked(std::size_t first); // m_arc[first, count) from the centres
  void markValuesLocked(std::size_t first, std::size_t last); // values [first, last] to upload

  mutable std::mutex m_ribbonMutex;
  std::vector<float> m_centers; // 3 per centre, capacity-sized
  std::vector<float> m_halfs;   // each centre's half width, capacity-sized
  std::vector<float> m_values;  // each centre's value, capacity-sized
  std::vector<double> m_arc;    // cumulative arc length per centre
  std::size_t m_count = 0;
  std::size_t m_capacity = 0;
  float m_half = 1.0f;
  double m_visFirst = 0.0, m_visLast = kToEnd;
  std::vector<float> m_scratch;
  std::vector<RampStop> m_ramp;
  bool m_rampShader = false;                 // the value-colour shader code is installed
  std::unique_ptr<RibbonValueTexture> m_tex; // made by the first setValueRamp
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_RIBBON_NODE_H
