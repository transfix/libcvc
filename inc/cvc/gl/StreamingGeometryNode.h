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

#ifndef CVC_GL_STREAMING_GEOMETRY_NODE_H
#define CVC_GL_STREAMING_GEOMETRY_NODE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cvc/gl/GeometryNode.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class vtkCallbackCommand;
class vtkRenderer;
class vtkShaderProgram;

namespace cvc {
namespace gl {

struct StreamingMapperCore;

// Which VTK mapper family a streaming node draws through. Auto asks VTK's
// object factory exactly as GeometryNode does (vtkPolyDataMapper::New()):
// vtkOpenGLPolyDataMapper on desktop GL, vtkOpenGLLowMemoryPolyDataMapper on
// GLES3/WebGL2. Classic / LowMemory force one family (an A/B switch, and how
// the native tests cover the low-memory path).
enum class StreamingMapperKind { Auto, Classic, LowMemory };

// The fixed shape of a streaming node: how many points it can hold, the
// triangles over them (indices < capacity_points; drawn in this order, so a
// draw range is a range of this list), the box the node promises to stay inside
// (it is what VTK culls and clips by -- see setReservedBounds), and the one
// normal every point carries (overlays are flat; per-point normals would need
// their own stream).
struct StreamingLayout {
  std::size_t capacity_points = 0;
  std::vector<std::array<std::uint32_t, 3>> triangles;
  cvc::bounding_box reserved_bounds;
  std::array<float, 3> normal{{0.0f, 0.0f, 1.0f}};
  StreamingMapperKind mapper = StreamingMapperKind::Auto;
};

// What a streaming node has done, for tests and HUDs. Read on any thread.
struct StreamStats {
  std::uint64_t writes = 0;        // writePoints() calls accepted
  std::uint64_t pointsWritten = 0; // points those calls staged
  std::uint64_t applies = 0;       // owner-thread applies that moved staged state into VTK
  std::uint64_t uploads = 0;       // sub-range GPU uploads the mapper issued
  std::uint64_t uploadBytes = 0;   // bytes in those uploads
  std::uint64_t uploadCalls = 0;   // GL calls they took (1, or up to 3 on GLES)
  std::uint64_t fullUploads = 0;   // whole-mesh uploads VTK did (first draw, relayout, ...)
};

// ---------------------------------------------------------------------------
// StreamingGeometryNode -- a mesh of FIXED capacity whose point positions are
// rewritten piecewise, from any thread, with each frame uploading only what
// changed.
//
// GeometryNode::updateVertices rewrites and re-uploads the whole mesh; per
// frame, for dozens of overlays, that is the cost that made a demo main-thread
// bound. Here a producer (a geometry worker, the sim) calls writePoints() with
// just the changed range. The range is copied into a staging buffer under the
// node's lock, dirty ranges merge, and ONE coalesced apply per frame
// (SceneGraph::postEventCoalesced) moves the merged range into VTK on the owner
// thread. The streaming mapper then uploads that range alone: one
// glBufferSubData, or at most three glTexSubImage2D on GLES/WebGL2 -- once per
// frame, however many render passes (shadows) draw the node.
//
// Everything that changes per frame without new points is NOT an upload:
//   * setDrawRange  -- which triangles are drawn (an append-only track's
//                      length, a route's consumed front). Applied with the
//                      points, so a range never shows points not yet written.
//   * setUniform    -- shader inputs for your replacements, stored under a
//                      mutex and pushed when the node is drawn.
//
// Contract:
//   * The layout (capacity, triangles, normal) is fixed. writePoints beyond the
//     capacity THROWS std::out_of_range and stages nothing: a generic node
//     cannot know how to extend its topology. Subclasses that do know (a
//     RibbonNode) grow with relayout().
//   * Points start at the origin. Triangles over points not yet written are
//     degenerate there; keep them out of the draw range (or inside the reserved
//     bounds) until written.
//   * Positions are float32 with VTK's coordinate shift/scale disabled, so the
//     shader sees the true coordinate. Fine for a local frame a few km across;
//     use a node transform for a far-off origin.
//   * Bounds come from the reserved box, never from the points: writes do not
//     call Modified(), so VTK never re-walks them. Keep the content inside it
//     (a RibbonNode grows its box to fit what it appends; a DrapedLinkNode
//     derives its box from the height field).
//   * NOT PICKABLE by default. VTK's CPU pickers (SceneRenderer::pickWorld's
//     vtkCellPicker) intersect the polydata, not what the shader draws: the
//     reserved-but-unwritten capacity, triangles outside the draw range and a
//     DrapedLinkNode's undraped template would all be hit while the drawn
//     overlay was missed. setPickable(true) opts in for a node whose drawn
//     triangles ARE its polydata (no draw range, no vertex-shader placement).
//   * Shadows. Streamed writes bump no MTime, so they would never re-bake a
//     shadow map on their own. A node that casts (GraphicsNode::setCastsShadow,
//     the default here) therefore marks its actor modified when an apply
//     changes what it draws -- points written, a relayout, a new draw range --
//     and the scene's baker re-bakes it at its update interval (nothing is
//     re-uploaded). RibbonNode and DrapedLinkNode are flat overlays and do not
//     cast by default, so streaming them never costs a bake.
//   * Re-showing a hidden node costs ONE full upload: setVisible(false) removes
//     the prop from the renderer, and VTK releases its buffers. Material
//     changes (GeometryNode::setColor, ...) upload nothing but bump the
//     property MTime, which re-bakes shadows; per frame, prefer uniforms in
//     your shader replacements (DrapedLinkNode::setStyle does).
//   * Rendered as a surface. setGeometry / updateVertices / updateColors /
//     updateNormals / setRenderMode are GeometryNode API that replaces the mesh
//     wholesale; they defeat streaming and are not meant for this node.
//
// Threading: writePoints / setDrawRange / setReservedBounds / setUniform are
// callable from ANY thread. On the owner thread writes apply immediately;
// elsewhere they are applied by the next processEvents(). With no scene
// attached they are only STAGED -- nothing touches VTK on the caller's thread --
// and are applied on the owner thread as soon as the node is attached. The
// node must be owned by a std::shared_ptr (as every addGraphicsChild node is):
// the queued apply is guarded by a weak_ptr.
//
// Known costs, deliberately left for later:
//   * relayout() (a RibbonNode growing) rebuilds the vtkPolyData in the apply,
//     on the owner thread, in O(capacity) -- size capacities so growth is rare.
//   * setUniform takes effect at the next draw, a staged range at the next
//     apply: an OFF-thread producer that changes both (RibbonNode's fractional
//     clip and its draw range) can show one frame with the new uniforms over the
//     old range. On the owner thread both land together.
class StreamingGeometryNode : public GeometryNode {
public:
  // setDrawRange count meaning "every triangle from firstTri on".
  static constexpr std::size_t kAllTriangles = static_cast<std::size_t>(-1);

  // Throws std::invalid_argument if a triangle indexes past capacity_points.
  StreamingGeometryNode(cvc::app &ctx, const std::string &statePath, const std::string &name,
                        const StreamingLayout &layout);
  ~StreamingGeometryNode() override;

  std::size_t capacityPoints() const;
  std::size_t triangleCount() const;
  // The family this node actually draws through (Auto already resolved).
  StreamingMapperKind mapperKind() const { return m_kind; }

  // Stage points [firstPoint, firstPoint + n) from n tightly packed xyz
  // triples. Any thread; latest write of a point wins. Throws
  // std::out_of_range if the range exceeds the capacity (nothing is staged).
  void writePoints(std::size_t firstPoint, const float *xyz, std::size_t n);

  // Draw only triangles [firstTri, firstTri + triCount) of the layout (clamped
  // to it). Any thread, latest wins; no upload. Default: every triangle.
  void setDrawRange(std::size_t firstTri, std::size_t triCount = kAllTriangles);

  // A uniform for your shader replacements (declare it there). Any thread;
  // stored under a mutex and pushed each time the node is drawn, in every
  // pass, skipped by programs that optimise it out. No event, no upload.
  void setUniform(const std::string &name, float v);
  void setUniform(const std::string &name, float x, float y, float z);

  // The box VTK culls and clips this node by (and getBoundingBox()). Any thread.
  // Setting it PINS it: a subclass that derives its own box (DrapedLinkNode)
  // stops re-deriving it until asked to (DrapedLinkNode::refreshReservedBounds).
  void setReservedBounds(const cvc::bounding_box &bounds);
  cvc::bounding_box reservedBounds() const;

  // Whether VTK's pickers may hit this node (default false; see the class
  // note). Any thread.
  void setPickable(bool pickable);
  bool pickable() const;

  StreamStats streamStats() const;

  cvc::bounding_box getBoundingBox() const override;

  // GLSL that yields the drawn point's index in a vertex shader replacement:
  // gl_VertexID on the classic mapper (an indexed draw), the low-memory
  // mapper's own pointId otherwise.
  const char *pointIdGLSL() const;

protected:
  // Replace the layout (capacity, triangles, normal, reserved bounds; the mapper
  // kind is fixed at construction). Staged points [0, min(old, new) capacity)
  // are kept. Any thread; applied with the next apply as one full upload. For
  // subclasses that own their topology and grow it.
  void relayout(const StreamingLayout &layout);

  // Reserved-bounds plumbing for subclasses. Any thread; staged and applied
  // like a write. growReservedBounds unions `bounds` into the current box (a
  // no-op when it already fits) and never pins; stageDerivedBounds replaces the
  // box unless the caller pinned one with setReservedBounds (returns false
  // then); unpinReservedBounds hands the box back to the subclass.
  void growReservedBounds(const cvc::bounding_box &bounds);
  bool stageDerivedBounds(const cvc::bounding_box &bounds);
  // stageDerivedBounds without the apply: for a subclass that commits under a
  // lock of its own and calls requestApply() once it has released it.
  bool commitDerivedBounds(const cvc::bounding_box &bounds);
  // Apply what is staged: inline on the owner thread, else posted to it.
  void requestApply();
  void unpinReservedBounds();
  bool reservedBoundsPinned() const;
  // Owner/render thread only, e.g. from beforeComputeBounds(): replace the box
  // the mapper reports right now (no apply round trip), unless pinned.
  void setDerivedBoundsNow(const cvc::bounding_box &bounds);

  // Render thread, GL context current, at the start of every draw of this node.
  virtual void beforeDraw(vtkRenderer *renderer);
  // Render/owner thread, each time VTK asks the mapper for its bounds (frustum
  // culling, clipping range) -- also for a node that ends up culled, which is
  // what makes it the place to re-derive bounds that may have gone stale.
  virtual void beforeComputeBounds();
  // After the stored uniforms were pushed to a program about to draw this node
  // (every pass). Bind textures here (GeometryNode::bindShaderTextures).
  virtual void updateShaderProgram(vtkShaderProgram *program);

  // Staged-but-unapplied work goes to the owner thread once a scene appears.
  void onSceneGraphChanged() override;

private:
  struct UniformValue {
    int n = 1;
    float v[3] = {0, 0, 0};
  };

  void buildPolyData(const StreamingLayout &layout, const float *points, std::size_t nPoints);
  void scheduleApply();
  void applyPending();
  bool hasPendingLocked() const;
  static void onUpdateShader(vtkObject *caller, unsigned long eid, void *clientData,
                             void *callData);
  void requestRenderIfAttached();

  StreamingMapperKind m_kind = StreamingMapperKind::Classic;
  StreamingMapperCore *m_core = nullptr; // owned by the mapper (GeometryNode::mapper())

  // Producer-side state, guarded by m_mutex.
  mutable std::mutex m_mutex;
  std::vector<float> m_staging; // 3 * capacity floats: the latest value of every point
  std::size_t m_stagingCapacity = 0;
  std::size_t m_dirtyLo = 0, m_dirtyHi = 0; // merged staged-but-unapplied point range
  bool m_drawRangePending = false;
  std::size_t m_drawFirst = 0, m_drawCount = kAllTriangles;
  bool m_boundsPending = false;
  bool m_boundsPinned = false;
  cvc::bounding_box m_bounds;
  std::unique_ptr<StreamingLayout> m_pendingLayout;
  std::size_t m_triangleCount = 0;
  // The SceneGraph::postEventCoalesced key of this node's applies: the address
  // of a private member, so no other producer can post under it (a node
  // pointer would collide with anyone coalescing "per node").
  const char m_applyKey = 0;
  std::atomic<bool> m_pickable{false};
  bool m_pickablePending = false; // guarded by m_mutex; applied with the rest

  // Draw-time uniforms, guarded by m_uniformMutex.
  mutable std::mutex m_uniformMutex;
  std::map<std::string, UniformValue> m_uniforms;
  vtkSmartPointer<vtkCallbackCommand> m_uniformCb;

  std::atomic<std::uint64_t> m_writes{0};
  std::atomic<std::uint64_t> m_pointsWritten{0};
  std::atomic<std::uint64_t> m_applies{0};
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_STREAMING_GEOMETRY_NODE_H
