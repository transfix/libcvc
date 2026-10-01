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

#ifndef CVC_GL_DRAPED_LINK_NODE_H
#define CVC_GL_DRAPED_LINK_NODE_H

#include <atomic>
#include <cstdint>
#include <cvc/gl/HeightFieldTexture.h>
#include <cvc/gl/StreamingGeometryNode.h>
#include <memory>
#include <mutex>

namespace cvc {
namespace gl {

// ---------------------------------------------------------------------------
// DrapedLinkNode -- a ribbon between two ground points that follows the
// terrain, computed entirely in the vertex shader.
//
// The mesh is a FIXED template of `stations` centre points, (t, side) with t in
// [0, 1] and side = +-1, uploaded once. The vertex shader places each template
// vertex between the two endpoint uniforms, offsets it sideways by the half
// width, and lifts it onto the HeightFieldTexture (plus `lift`): the drawn
// centre line is piecewise linear between the stations' draped heights
// (centerAt() is the same polyline on the CPU). Moving a link is therefore two
// uniforms -- setEndpoints() -- and NO upload, however often it moves.
//
// The draping is spliced in with `#define vertexMC <draped>` ahead of VTK's
// clipping and position code (at //VTK::Clip::Impl, which precedes
// //VTK::PositionVC::Impl), so every later use -- the clip-plane distances of
// a parent's setClipChildren, VTK's position code, the shadow baker's depth
// pass -- sees the draped point: vertexMC is a read-only input on the classic
// mapper and a local on the low-memory one, and the #define covers both. The
// drape is an internal shader replacement (GeometryNode): a caller's own
// replacements compose with it and clearShaderReplacements() keeps it.
//
// Colour and opacity are fragment uniforms (setStyle), unlit, so restyling a
// link per frame (signal quality, jamming) does not bump the property MTime --
// which would re-bake the shadow maps. Opacity < 1 additionally moves the
// actor to the translucent pass (a property change, made only when crossing 1).
//
// The template's own coordinates are near the origin, so the node always draws
// with RESERVED bounds or VTK would frustum-cull it. By default they are
// DERIVED: the height field's extent padded by the width and lift, re-derived
// when setStyle changes those and, lazily (when VTK next asks for the bounds,
// so a culled link recovers too), whenever the height field's heights change.
// setReservedBounds() pins your own box instead -- setStyle and height changes
// then leave it alone -- until refreshReservedBounds() hands it back.
class DrapedLinkNode : public StreamingGeometryNode {
public:
  DrapedLinkNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
                 std::shared_ptr<HeightFieldTexture> heights, int stations = 24,
                 StreamingMapperKind mapper = StreamingMapperKind::Auto);

  // The two ground points (z comes from the height field). Any thread, latest
  // wins: two uniforms, no upload.
  void setEndpoints(float x0, float y0, float x1, float y1);
  // Half width and lift above the ground (world units), colour and opacity.
  // Any thread; uniforms (see the class note on opacity).
  void setStyle(float halfWidth, float lift, float r, float g, float b, float opacity = 1.0f);
  // Re-derive the reserved bounds from the height field's current extent, and
  // un-pin a box set with setReservedBounds(). Any thread.
  void refreshReservedBounds();

  int stations() const { return m_stations; }
  std::shared_ptr<HeightFieldTexture> heightField() const { return m_heights; }
  // Where the drawn centre line is at parameter t in [0, 1]: the GPU's
  // piecewise-linear drape, evaluated with HeightFieldTexture::sample.
  void centerAt(double t, double out[3]) const;

protected:
  void beforeDraw(vtkRenderer *renderer) override;
  void beforeComputeBounds() override;
  void updateShaderProgram(vtkShaderProgram *program) override;

private:
  cvc::bounding_box derivedBounds() const;

  std::shared_ptr<HeightFieldTexture> m_heights;
  const int m_stations;
  mutable std::mutex m_linkMutex;
  float m_a[2] = {0, 0}, m_b[2] = {0, 0};
  float m_half = 1.0f, m_lift = 0.5f;
  bool m_translucent = false;
  // HeightFieldTexture::generation() the derived bounds were last taken at.
  std::atomic<std::uint64_t> m_seenGeneration{0};
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_DRAPED_LINK_NODE_H
