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

// LodGraphicsNode.h -- a scene-graph node that owns a mesh LOD ladder and draws
// exactly one rung, chosen each frame by camera distance.
//
// It holds one GeometryNode child per rung (rung 0 = finest) plus the pyramid's
// monotone world_error_m ladder, and is itself a prop-less container. select(view)
// runs the pure math in <cvc/lod/select.h> -- bound-nearest distance to the node's
// world bounding sphere, then the hysteretic select_rung over the ladder -- and
// makes exactly the chosen rung's actor visible. A SceneGraph-level pass
// (SceneGraph::selectLOD) or the caller invokes select() per frame; nothing here
// draws, so the selection is testable with no GL context.
//
// --- Two kinds of visibility, kept apart ------------------------------------
//
// The node's OWN visibility (setVisible, and its ancestors') is the user's: a
// hidden LOD node, or one under a hidden parent, draws nothing, and select()
// does not bring it back. WHICH rung draws is the selection's. The two meet in
// one place: a rung's actor is visible exactly when it is the active rung AND
// the node is visible in the hierarchy (GraphicsNode::isVisibleInHierarchy).
// SceneNode::setVisible would otherwise pass a parent's "show" down to every
// rung at once; this node overrides that propagation, so re-showing a parent
// shows one rung, not the whole ladder. The rung actors are re-derived whenever
// either side can have moved -- the node's own flag, ANY ancestor's (even when
// this node's flag already matched, e.g. it was attached under a hidden
// parent), re-attachment to a renderer or a new parent, and every select() --
// so a caller that selects only when the camera moves still draws the right
// thing.
//
// --- Rung switching and GPU memory ------------------------------------------
//
// Every rung's actor stays REGISTERED with the renderer for the node's whole
// life; a switch only flips vtkProp::SetVisibility. The alternative,
// Add/RemoveViewProp, is what SceneNode::setVisible does, and RemoveViewProp
// calls ReleaseGraphicsResources on the prop -- every switch back would
// re-upload the rung's buffers. The trade-off: a rung that has drawn once keeps
// its vertex buffers resident, so the GPU holds up to the whole ladder (about
// 1 / (1 - mesh_ratio), i.e. ~1.5x rung 0 at the default ratio of 0.35) rather
// than one rung. A rung that has never drawn uploads nothing. The renderer's
// prop count is therefore constant across switches, and invisible rungs are
// skipped by the draw, the shadow bake and camera framing alike.
//
// --- Progressive attach -------------------------------------------------------
//
// A pyramid takes time to build. setBase(mesh) shows the finest mesh at once as
// a one-rung ladder; appendRungs(pyramid), later, adds the coarser rungs of a
// pyramid built from that same mesh WITHOUT re-creating rung 0 -- its node, its
// style and any buffers it has already uploaded are kept. setPyramid(pyramid) is
// the one-shot equivalent and always rebuilds every rung.
//
// Threading: like every scene node, populate and select on the scene's owner
// thread (the one that renders).
//
// LOD is a render proxy only: a rung is never read by a nav/material/RF path.

#ifndef __CVC_GL_LOD_GRAPHICS_NODE_H__
#define __CVC_GL_LOD_GRAPHICS_NODE_H__

#include <cstdint>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <functional>
#include <memory>
#include <vector>

class vtkRenderer;

namespace cvc {
namespace gl {

class GeometryNode;

// What one SceneGraph::selectLOD pass did, for a HUD or a profile line. Reuse
// one instance across frames: the histogram keeps its storage.
struct lod_stats {
  int nodes = 0;   // LodGraphicsNodes with at least one rung that the pass visited
  int hidden = 0;  // of those, hidden by their own or an ancestor's visibility
  int changes = 0; // nodes whose active rung changed -- selectLOD's return value
  // rung_nodes[k] = nodes whose active rung is k (hidden ones included), so the
  // entries sum to `nodes`. Sized to the deepest active rung seen.
  std::vector<int> rung_nodes;
  // Triangles the VISIBLE nodes draw at their active rungs, and what the same
  // nodes would draw at rung 0 -- the LOD-off cost. Their ratio is the saving.
  std::uint64_t drawn_tris = 0;
  std::uint64_t full_tris = 0;

  void reset(); // zero everything, keeping rung_nodes' capacity
};

class LodGraphicsNode : public GraphicsNode {
public:
  // Applied to every rung's GeometryNode -- see setRungStyle.
  using RungStyle = std::function<void(GeometryNode &)>;

  LodGraphicsNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "lod");
  ~LodGraphicsNode() override;

  // Populate from a built mesh pyramid: one GeometryNode child per rung (named
  // "lod0", "lod1", ...) and the ladder select() uses. Removes and destroys any
  // prior rungs first, so calling it again replaces the ladder rather than
  // growing it. Resets the selection: selectedRung() is -1 and rung 0 draws
  // until the first select().
  void setPyramid(const cvc::lod::mesh_pyramid &pyr);

  // Progressive attach, step 1: a one-rung ladder holding just `finest` (world
  // error 0), drawn at once. Replaces any prior rungs, like setPyramid.
  void setBase(const cvc::geometry &finest);

  // Progressive attach, step 2: take the coarser rungs of `pyr`, a pyramid built
  // from the current rung 0. When pyr.rungs[0] has the same drawable content as
  // rung 0 (points, cells, normals, colours, uvs -- copies of one cvc::geometry
  // share storage, so the usual case is a pointer check, not a scan), rung 0's
  // node is KEPT with its style and uploaded buffers, and only rungs 1.. are
  // rebuilt from `pyr`, replacing any coarser rungs already attached; the
  // current selection carries over. Otherwise -- different content, or no rungs
  // yet -- it falls back to setPyramid(pyr). Returns true when rung 0 was kept.
  // An empty pyramid changes nothing and returns false.
  bool appendRungs(const cvc::lod::mesh_pyramid &pyr);

  // A styling step run on every rung's GeometryNode: the existing ones now, and
  // each one created later (setPyramid, setBase, appendRungs) right after its
  // geometry is set and before it can draw. Colour, material, shader
  // replacements and the like therefore match across rungs, so a switch changes
  // only the triangles. An empty function stops styling new rungs; it does not
  // undo what was already applied.
  void setRungStyle(RungStyle style);

  int rungCount() const { return static_cast<int>(m_rungs.size()); }
  double rungError(int k) const;
  // Triangle count of rung k (cvc::geometry::num_tris), 0 out of range.
  std::uint64_t rungTriangles(int k) const;
  // Rung k's node, or null out of range. For inspection; the ladder owns it.
  std::shared_ptr<GeometryNode> rung(int k) const;

  // The rung chosen by the last select() or setRung(): -1 before the first one,
  // after setPyramid/setBase, and when the ladder is empty. This is also the
  // hysteresis history select() starts from.
  int selectedRung() const { return m_current; }
  // The rung that draws while the node is visible: selectedRung(), or rung 0
  // before any selection has been made; -1 when the ladder is empty.
  int activeRung() const;

  // Choose the affordable rung for `view` and make it the active rung. Returns
  // the rung index (0 = finest), or -1 when the ladder is empty. The choice is
  // made whether or not the node is visible -- the hysteresis history stays
  // continuous, so re-showing it draws the right rung at once -- but a hidden
  // node's rungs stay hidden.
  int select(const cvc::lod::view_params &view);

  // Pin the active rung to `k` (clamped into the ladder), bypassing the
  // selection -- what SceneGraph::setLODEnabled(false) uses to force rung 0, and
  // what a "freeze LOD at rung N" debug switch wants. The next select() resumes
  // from it. Returns the rung, or -1 when the ladder is empty.
  int setRung(int k);

  // Untransformed extent of rung 0 (the full-detail mesh) -- what distance and
  // culling should see regardless of which rung currently draws.
  cvc::bounding_box getBoundingBox() const override;

  // Also re-derives the rung actors: the node may have come back under a
  // different (hidden or shown) parent.
  void addToRenderer(vtkRenderer *renderer) override;

protected:
  vtkProp *getProp() override; // prop-less container; the rungs are children
  // Re-applies the rung policy instead of showing/hiding every rung; other
  // children, if any, get the usual propagation.
  void propagateVisible(bool visible) override;
  // Re-applies the rung policy when an ancestor's visibility changed but this
  // node's flag did not.
  void ancestorVisibilityChanged() override;

private:
  void clearRungs(std::size_t keep = 0); // remove and destroy rungs [keep, n)
  void addRung(const cvc::geometry &g, double worldError);
  bool isRung(const SceneNode *child) const;
  // Show exactly the active rung's actor iff the node is visible in the
  // hierarchy; hide every other rung. Never Add/RemoveViewProp.
  void applyRungVisibility();

  std::vector<std::shared_ptr<GeometryNode>> m_rungs;
  std::vector<double> m_worldError;  // monotone; matches m_rungs
  std::vector<std::uint64_t> m_tris; // per-rung triangle counts
  // rung 0's mesh, shared (copy-on-write) with its node -- appendRungs compares
  // an incoming pyramid's rung 0 against it.
  std::unique_ptr<cvc::geometry> m_base;
  RungStyle m_style;
  int m_current = -1; // no rung selected yet
};

// A cvc::lod::view_params for what `renderer`'s active camera sees this frame:
// eye = camera position, viewport_h_px = the renderer's viewport height in
// pixels (its share of the window, so a split view gets its own), and the
// projection. Perspective: tan_half_fov from vtkCamera's ViewAngle, which is the
// VERTICAL angle in degrees (converted when UseHorizontalViewAngle is on).
// Parallel: ortho_px_per_m = viewport_h_px / (2 * ParallelScale), because
// ParallelScale is half the view height in world units. The error budget,
// hysteresis and z_near come from `base` (a quality preset, typically), so
// call this once per frame with the same base and hand the result to
// SceneGraph::selectLOD. A null renderer, or one with no window yet (viewport
// size 0), leaves the corresponding fields of `base` untouched. So does a
// renderer with no active camera yet (IsActiveCameraCreated() false, i.e. before
// its first render unless the caller set one): this never creates the camera,
// because a camera created outside the render is not auto-framed by VTK, so the
// eye and projection come from `base` for that frame.
cvc::lod::view_params make_view_params(vtkRenderer *renderer,
                                       const cvc::lod::view_params &base = cvc::lod::view_params());

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_LOD_GRAPHICS_NODE_H__
