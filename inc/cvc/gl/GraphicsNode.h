#ifndef GRAPHICSNODE_H
#define GRAPHICSNODE_H

#include <any>
#include <array>
#include <atomic>
#include <boost/signals2.hpp>
#include <cstdint>
#include <cvc/core/world_units.h>
#include <cvc/gl/SceneNode.h>
#include <cvc/volume/bounding_box.h>
#include <map>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <vtkMatrix4x4.h>
#include <vtkPlaneCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

class vtkActor2D;
class vtkPlane;

namespace cvc {
class geometry;
class volume;
} // namespace cvc

namespace cvc {
namespace gl {
class BBoxNode;

/**
 * @brief Abstract base class for all graphics objects in the scene
 *
 * GraphicsNode provides common functionality for all renderable graphics objects:
 * - Transformation (position, rotation, scale)
 * - Hierarchical structure (parent/child relationships)
 * - Metadata storage
 * - Bounding box display
 * - Visibility control
 * - State tree synchronization (via state_object inheritance)
 * - Clipping planes based on bounding box
 *
 * Subclasses must implement:
 * - getBoundingBox() - return the untransformed bounding box
 * - getProp() - return the VTK prop for rendering
 * - handleStateChanged() - respond to state tree changes
 */

class GraphicsNode : public SceneNode {
public:
  GraphicsNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "");
  virtual ~GraphicsNode();

  // Identity and naming
  void setName(const std::string &name) { m_name = name; }
  std::string getName() const { return m_name; }

  // Pure virtual methods that subclasses must implement
  virtual cvc::bounding_box
  getBoundingBox() const = 0; // Return untransformed bounding box of THIS node only

  // Get combined bounding box (this node + all children)
  cvc::bounding_box getCombinedBoundingBox() const;

  // Transform management
  void setTransform(vtkMatrix4x4 *matrix);
  void setTransform(const double matrix[16]); // Row-major 4x4 matrix
  vtkMatrix4x4 *getTransform() { return m_transform; }
  const vtkMatrix4x4 *getTransform() const { return m_transform; }

  // The per-frame POSE path: set this node's local transform to a row-major 4x4
  // (the same layout as setTransform(const double[16])), for scenes that move
  // many nodes every frame -- vehicles, agents, anything driven by a simulation.
  //
  // What it does differently from setTransform, and why:
  //   * No state-string round trip on the calling thread. setTransform formats
  //     16 doubles, writes the state tree inline, and its own "matrix" handler
  //     re-parses the echo and runs the transform cascade a second time. Here the
  //     matrix is applied once and PUBLISHED through the scene's state_publisher
  //     (as setPosition publishes), which formats it on the flushing thread; the
  //     value it writes comes back as an echo and is dropped. The state tree
  //     still ends up holding the pose ("matrix"), eventually-consistent like
  //     every published value.
  //   * Thread-safe. On the scene's owner thread (or with no scene) it applies
  //     immediately. From any other thread nothing is touched there: the matrix
  //     is parked in a one-slot mailbox and applied on the owner thread at the
  //     next processEvents(). Latest wins -- a simulation thread posing faster
  //     than the frame rate queues ONE apply per node per frame, not one per
  //     call -- and an owner-thread call drops any older parked pose.
  //   * Change-detected. An identical matrix is a no-op: no cascade, no actor
  //     Modified() (which would make the shadow baker re-bake), no publish.
  //     The one exception: if the publisher has shed values since this node
  //     last published, the unchanged pose is published again (state only),
  //     because the shed value may have been ours.
  //   * Safe to mix with setTransform/resetTransform and with external writes
  //     to "matrix". Once a node has published a pose, those paths also queue
  //     the matrix they set, so a pose still waiting to be flushed cannot be
  //     written over the newer value later.
  //   * Like every move, it fires transformChanged once; the SceneGraph folds all
  //     the moves between two pumps into ONE world-bounds walk.
  // The result -- local, world and actor matrices, children, bounds -- is
  // exactly what setTransform produces for the same matrix.
  void setPoseMatrix(const double matrix[16]);

  // Convenience transform methods
  void setPosition(double x, double y, double z);
  void setRotation(double x, double y, double z); // Euler angles in degrees
  void setScale(double x, double y, double z);
  void resetTransform(); // Set to identity matrix

  // Get world transform (accumulated from all parents)
  vtkSmartPointer<vtkMatrix4x4> getWorldTransform() const;

  // Point transforms between this node's LOCAL (object) space and WORLD space.
  // localToWorld composes with getWorldTransform(); worldToLocal applies its
  // inverse. The inverse is computed lazily and cached (invalidated whenever the
  // node's world transform changes), so a per-frame pose cascade pays nothing and
  // only an actual query -- a pick, a coordinate readout -- pays the one Invert().
  // out and in may alias. Homogeneous w is normalised, so a projective transform
  // is handled correctly; an affine scene transform leaves w == 1.
  void localToWorld(const double local[3], double world[3]) const;
  void worldToLocal(const double world[3], double local[3]) const;

  // A point in this node's LOCAL coordinate frame reported as a real-world
  // coordinate in the given regime: local -> world (this node's transform) ->
  // canonical metres -> the active display unit (m/km or ft/mi). This is the
  // "click a graphic in its own coordinate transform, tell me where that is in
  // kilometres/miles" path once a pick has resolved a hit to a local point.
  cvc::world_units::coordinate localPointToReal(const double local[3],
                                                const cvc::world_units &units) const;

  // The node's bounding box in WORLD space: its untransformed getBoundingBox()
  // pushed through the full chain of local transforms (getWorldTransform()), then
  // re-fit to an axis-aligned box. Because the world matrix is kept current
  // top-down as ancestors are added, moved or removed, this is a RELIABLE world
  // footprint for a node at any depth -- you never have to walk or compose the
  // parent chain yourself. getCombinedWorldBoundingBox() does the same for this
  // node together with all its descendants.
  cvc::bounding_box getWorldBoundingBox() const;
  cvc::bounding_box getCombinedWorldBoundingBox() const;

  // Whether this node counts toward the scene's world bounds
  // (SceneGraph::computeGraphicsBounds). False for a light: it draws nothing,
  // yet its box would sit at the light's position and drag the bounds out to it.
  bool contributesToSceneBounds() const { return m_contributesToSceneBounds; }

  // The node's real-world size in the given regime: the extents of its
  // world-space bounding box (this node, or with all descendants when
  // includeChildren) converted through world_units. The three components share
  // one unit (m/km or ft/mi), chosen from the largest, so a measured dimension
  // reads naturally. This is the "how big is this graphic, really" answer, taken
  // reliably through the whole chain of local transforms. A degenerate/empty box
  // yields a zero size in the regime's base unit.
  cvc::world_units::coordinate realDimensions(const cvc::world_units &units,
                                              bool includeChildren = true) const;

  // Fired ONCE when this node's transform changes (setPosition/setRotation/
  // setScale/setTransform/setPoseMatrix/resetTransform, or a state-driven move)
  // — not once per recursively-updated child. The SceneGraph connects to it to
  // recompute the world bounds/grid so the world box tracks a node that moves
  // out of it; it only flags the scene, and the walk itself runs once per
  // processEvents() however many nodes moved. Emitted on the thread that made
  // the move, so a slot must be thread-safe.
  boost::signals2::signal<void(GraphicsNode *)> transformChanged;

  // Hierarchical structure

  // Template factory method for creating child graphics nodes
  // Automatically constructs the proper state path based on parent's state
  // Usage: auto node = parent->addGraphicsChild<GeometryNode>("myGeom");
  // Extra arguments are forwarded to T's constructor after (app, statePath,
  // name), for node types that need more to exist (a StreamingGeometryNode's
  // layout, a RibbonNode's capacity):
  //        auto track = parent->addGraphicsChild<RibbonNode>("track", 1024, 1.5f, box);
  template <typename T, typename... Args>
  std::shared_ptr<T> addGraphicsChild(const std::string &name, Args &&...args) {
    static_assert(std::is_base_of<GraphicsNode, T>::value, "T must be derived from GraphicsNode");

    // Construct state path: {parent_path}.children.{name}
    std::string childStatePath = getState().fullName() + ".children." + name;

    // Create the child node with proper state path and name
    auto child =
        std::make_shared<T>(this->app(), childStatePath, name, std::forward<Args>(args)...);

    // Add to children using the non-template version
    addGraphicsChild(std::static_pointer_cast<GraphicsNode>(child));

    return child;
  }

  // Non-template version for adding existing nodes (virtual to allow override)
  virtual void addGraphicsChild(std::shared_ptr<GraphicsNode> child);

  // Generic template method for creating child graphics nodes with data
  // Usage: auto geomNode = parent->createChild<GeometryNode>("name", geomData);
  //        auto volNode = parent->createChild<VolumeNode>("name", volData);
  template <typename NodeType, typename DataType>
  std::shared_ptr<NodeType> createChild(const std::string &name, const DataType &data) {
    static_assert(std::is_base_of<GraphicsNode, NodeType>::value,
                  "NodeType must be derived from GraphicsNode");

    // Create the child node using the template factory
    auto child = addGraphicsChild<NodeType>(name);

    // Set the data using the generic setData method
    child->setData(data);

    return child;
  }

  // Overload for creating child without data (uses NullGraphicNode)
  std::shared_ptr<GraphicsNode> createChild(const std::string &name);

  virtual void removeGraphicsChild(std::shared_ptr<GraphicsNode> child);
  std::shared_ptr<GraphicsNode> findChildByName(const std::string &name);
  const std::vector<std::shared_ptr<GraphicsNode>> &getGraphicsChildren() const {
    return m_graphicsChildren;
  }

  // Metadata management
  void setMetadata(const std::string &key, const std::any &value);
  std::any getMetadata(const std::string &key) const;
  bool hasMetadata(const std::string &key) const;
  const std::map<std::string, std::any> &getAllMetadata() const { return m_metadata; }

  // Shadow casting. A node that does not cast is left out of the shadow maps --
  // it is still lit and still RECEIVES shadows -- and, the reason to use it,
  // changing it (a pose, a colour, a texture) no longer makes the shadow baker
  // re-render the scene: VTK re-bakes when ANY prop changed, the scene's baker
  // only when a caster or a light did. Mark what moves every frame and casts
  // little (vehicles over a city, overlays, a fog layer). Translucent geometry
  // never casts in VTK, so marking it costs nothing on screen.
  //
  // Default true. Applies to this node, its graphics descendants, and those
  // added later. Any thread: applied on the owner thread (inline there, at the
  // next processEvents() from another), and castsShadow() reports it from then.
  void setCastsShadow(bool casts);
  bool castsShadow() const { return m_castsShadow.load(std::memory_order_relaxed); }
  // The same flag on a vtkProp a host put in the renderer itself (owner thread).
  // A prop casts unless marked.
  static void setPropCastsShadow(vtkProp *prop, bool casts);
  static bool propCastsShadow(vtkProp *prop);

  // Bounding box visibility
  void setShowBBox(bool show);
  bool getShowBBox() const { return m_showBBox; }

  // Bounding box color
  void setBBoxColor(double r, double g, double b);
  void getBBoxColor(double &r, double &g, double &b) const;

  // Bounding box extent labels
  void setShowExtentLabels(bool show);
  bool getShowExtentLabels() const;
  void setExtentLabelColor(double r, double g, double b);
  void getExtentLabelColor(double &r, double &g, double &b) const;
  void setExtentLabelFontSize(int size);
  int getExtentLabelFontSize() const;

  // ── Clipping ──────────────────────────────────────────────────────────────
  // A node is clipped by its own planes (setClipPlanes) and by every ancestor's:
  // their setClipPlanes, and the box of an ancestor with setClipChildren. All are
  // half-spaces that keep the side a plane's normal points INTO,
  // n . (x - origin) >= 0 (VTK's convention, and cvc::volren::cut_plane's). They
  // follow each owner as it moves and as its box changes (every node reports that
  // through ownBoundsChanged()), reach children added later and leave children
  // removed. A renderer honours at most maxClipPlanes() of them, nearest first:
  // the node's own, then its parent's, and so on up. VTK 9.5's low-memory mapper
  // (the GLES3/WebGL2 default, or CVCGL_LOWMEM_MAPPER=force) honours none.

  // One clip half-space in a node's LOCAL frame (origin + normal; the normal
  // need not be unit length).
  struct ClipPlane {
    std::array<double, 3> origin{0.0, 0.0, 0.0};
    std::array<double, 3> normal{0.0, 0.0, 1.0};
    bool operator==(const ClipPlane &o) const { return origin == o.origin && normal == o.normal; }
  };
  // The six planes that keep the inside of box `b` (normals pointing in).
  static std::vector<ClipPlane> boxClipPlanes(const cvc::bounding_box &b);

  // Clip this node AND everything below it to the intersection of `planes`, in
  // this node's local frame (they move with it). Empty clears them. Moving the
  // same number of planes is cheap (a slider dragging one: updated in place);
  // changing the number re-hands every node below. Mirrored in the
  // "clip_planes" state key as px,py,pz,nx,ny,nz per plane.
  void setClipPlanes(const std::vector<ClipPlane> &planes);
  const std::vector<ClipPlane> &clipPlanes() const { return m_ownClip; }

  // setClipChildren(true): everything below this node is clipped to this node's
  // own getBoundingBox() (its six faces, under its world transform). The node
  // itself is not. Mirrored in the "clip_children" state key.
  void setClipChildren(bool clip);
  bool getClipChildren() const { return m_clipChildren; }
  // The six world-space box planes setClipChildren clips to.
  vtkPlaneCollection *getClipBoxPlanes() const { return m_clipPlanes; }

  // How many planes clip this node (its own + its ancestors'), before the cap.
  int clipPlaneCount() const;
  // The world-space planes handed to this node's renderer (at most
  // maxClipPlanes()), or null when none are.
  vtkPlaneCollection *getAppliedClipPlanes() const;
  // How many clip planes this node's renderer honours: 6 for VTK's poly-data
  // mapper, 8 for its GPU volume mapper and for cvc::volren, 0 where none (VTK's
  // low-memory mapper; a node that draws nothing itself, e.g. a group or a
  // LodGraphicsNode, whose rungs are its children and are clipped as such).
  virtual int maxClipPlanes() const { return 0; }

  // Label control
  void setShowLabel(bool show);
  bool getShowLabel() const { return m_showLabel; }
  void setLabelText(const std::string &text);
  std::string getLabelText() const { return m_labelText; }
  void setLabelSize(int size);
  int getLabelSize() const { return m_labelSize; }
  void setLabelColor(double r, double g, double b);
  void getLabelColor(double &r, double &g, double &b) const;

  // Override visibility to sync with metadata. virtual so nodes that own extra
  // actors (GridNode's planes + tick labels, AxisNode's axes) can hide ALL of
  // them — the base only knows about the single getProp() actor + the label.
  virtual void setVisible(bool visible);

  // True when this node AND every graphics ancestor are visible, i.e. whether
  // the node can draw at all. isVisible() is only the node's own flag, which a
  // hidden parent does not always reach: a child added under an already-hidden
  // parent keeps its own `true`. A walk up the parent chain; cheap at scene
  // depths.
  bool isVisibleInHierarchy() const;

  // Override update to handle transform changes
  void update() override;

  // Override addToRenderer/removeFromRenderer to handle bbox
  void addToRenderer(vtkRenderer *renderer) override;
  void removeFromRenderer(vtkRenderer *renderer) override;

protected:
  // isRoot=true (the default, used by the public setters + state handlers) fires
  // transformChanged after the subtree is updated; the child recursion passes
  // false so the signal fires once per user move, not once per node.
  void updateTransform(bool isRoot = true);
  void updateBoundingBoxNode(); // Update bbox node with current bounds + transform
  void updateLabel();           // Update label position and properties
  void updateClipPlanes();      // Update clip planes based on bounding box and transform
  // A subclass calls this when its own getBoundingBox() changed (new data, new
  // bounds): re-fits the bbox outline and the planes it clips its children to.
  void ownBoundsChanged();

  // Generic helper to apply world transform to a vector of VTK props
  void applyWorldTransformToProps(const std::vector<vtkProp *> &props);

  // Apply transform to VTK prop - subclasses should override to apply to their specific prop type
  virtual void applyTransformToVTK();

  // Hand this node's renderer the world-space planes it clips by (at most
  // maxClipPlanes()), or null for none. Called when that SET changes; a plane
  // moving in place is not a call (the renderer reads the planes it holds).
  // Subclasses that draw override it, with maxClipPlanes().
  virtual void applyClipPlanes(vtkPlaneCollection *planes);

  // Re-derive what this node is clipped by and hand it to the renderer, then the
  // same for everything below whose inherited planes changed. Owner thread.
  void propagateClip();

  // Protected members for subclass access
  std::string m_name;
  vtkSmartPointer<vtkMatrix4x4> m_transform;
  // Cached object-to-world state, refreshed top-down by updateTransform().
  // Both exist to keep the per-frame pose path ALLOCATION-FREE: it used to make
  // a fresh vtkMatrix4x4 (inside getWorldTransform, which also recursed up to
  // the root) and a fresh vtkTransform for every node the cascade touched,
  // every frame. Reusing them is what takes the per-node cost down.
  vtkSmartPointer<vtkMatrix4x4> m_worldMatrix;
  // Cached world->object inverse, computed lazily on the first worldToLocal /
  // localPointToReal after a move and reused until the next one. Kept off the
  // hot pose path (updateTransform only flags it dirty) because picking and
  // coordinate readouts are rare relative to per-frame poses. mutable so the
  // const query methods can fill it on demand.
  mutable vtkSmartPointer<vtkMatrix4x4> m_worldInverse;
  mutable bool m_worldInverseDirty = true;
  // Full state paths for the transform keys, resolved ONCE. getState(name) costs
  // ~8 us per call, which was a third of the price of moving a node.
  std::string m_pathPosition, m_pathRotation, m_pathScale, m_pathMatrix;
  // The last value this node published for each key. The node is the source of
  // truth for its own transform, so when the publisher's write comes back
  // through handleStateChanged it must be recognised as an echo and ignored —
  // otherwise every pose runs the whole cascade a second time. An EXTERNAL write
  // (the dashboard, a script) differs from what we published and still applies.
  std::string m_echoPosition, m_echoRotation, m_echoScale, m_echoMatrix;
  vtkSmartPointer<vtkTransform> m_worldXf;
  vtkSmartPointer<vtkTransform> m_vtkTransform; // VTK transform wrapper for m_transform
  std::vector<std::shared_ptr<GraphicsNode>> m_graphicsChildren;
  GraphicsNode *m_parent; // Weak pointer to parent for world transform calculation
  std::map<std::string, std::any> m_metadata;
  std::atomic<bool> m_castsShadow{true}; // see setCastsShadow
  // Plain flags where the bounds walks used to ask dynamic_cast, once per node
  // per walk: does this node count toward the scene bounds (not a LightNode),
  // and does its getCombinedBoundingBox() include its own box (a NullGraphicNode
  // may frame only its children).
  bool m_contributesToSceneBounds = true;
  bool m_combinedIncludesOwnBounds = true;
  bool m_showBBox;
  std::shared_ptr<BBoxNode> m_bboxNode;

  // Label members
  bool m_showLabel;
  std::string m_labelText;
  int m_labelSize;
  double m_labelColor[3];
  vtkSmartPointer<vtkActor2D> m_labelActor;

  // Clipping planes
  bool m_clipChildren; // Whether to clip children to this node's bounding box
  vtkSmartPointer<vtkPlaneCollection> m_clipPlanes;          // Collection of 6 planes
  std::array<vtkSmartPointer<vtkPlane>, 6> m_clipPlaneArray; // Individual planes for updates

  // State change handler override
  virtual void handleStateChanged(const std::string &childState) override;

private:
  // setPoseMatrix plumbing. applyPoseMatrix runs on the owner thread only; the
  // mailbox below hands a pose made on another thread to it.
  void applyPoseMatrix(const double matrix[16]);
  void applyParkedPose();
  void publishPoseMatrix(const double matrix[16]);
  // Called after any OTHER path puts a new matrix on this node and writes it to
  // state directly (setTransform, resetTransform, an external "matrix" write).
  // Re-queues the node's current matrix so it replaces a pose that is still
  // waiting in the publisher, or one a flush is writing right now. Without this,
  // that older pose would be written over the newer value, and a node that then
  // stays still would never correct it.
  void supersedePublishedMatrix();
  // setPoseMatrix has published through a scene publisher at least once, so a
  // pose of ours may be queued there. Never cleared, because "nothing queued
  // any more" cannot be known without racing the flush. It stays false for a
  // node that only uses setTransform, so that node's behaviour is unchanged.
  // Atomic because setTransform may be called on any thread.
  std::atomic<bool> m_matrixPublished{false};
  // The publisher's dropped() count read just BEFORE our last pose publish
  // (owner thread only). If dropped() has moved since then, the publisher may
  // have shed our pose, so an unchanged pose is published again instead of
  // being skipped as a no-op. See applyPoseMatrix.
  std::uint64_t m_poseDropMark = 0;
  std::mutex m_poseMutex;                // guards m_parkedPose + m_posePending writes
  std::array<double, 16> m_parkedPose{}; // the latest off-thread pose, not yet applied
  // A parked pose is waiting (and one apply is queued for it). Atomic so the
  // owner-thread fast path can see "nothing parked" without taking the lock.
  std::atomic<bool> m_posePending{false};
  // True only while a scene-less node writes its own pose to state directly:
  // the handler that write fires runs inline on this thread and is our echo.
  bool m_writingPose = false;

  // Clip plumbing (owner thread). The vtkPlane objects are SHARED down the tree:
  // a descendant's renderer holds its ancestors' plane objects, so moving a plane
  // in place reaches every renderer clipped by it with no re-hand.
  void applyOwnClip(const std::vector<ClipPlane> &planes); // no state write
  void updateOwnClipWorld(); // re-pose the own planes from local to world, in place
  // What everything below this node is clipped by: its own planes, its box under
  // setClipChildren, then what it inherited -- nearest first.
  std::vector<vtkSmartPointer<vtkPlane>> clipPassdown() const;
  std::vector<ClipPlane> m_ownClip;                       // local frame, as set
  std::vector<vtkSmartPointer<vtkPlane>> m_ownClipWorld;  // the same, world space
  std::vector<vtkSmartPointer<vtkPlane>> m_inheritedClip; // the parent's clipPassdown()
  std::vector<vtkPlane *> m_appliedClipList;              // what the renderer was handed
  vtkSmartPointer<vtkPlaneCollection> m_appliedClip;      // ... as a collection
  bool m_clipCapWarned = false;                           // logged once that the cap dropped planes
};

} // namespace gl
} // namespace cvc

#endif // GRAPHICSNODE_H
