#ifndef GRAPHICSNODE_H
#define GRAPHICSNODE_H

#include <any>
#include <array>
#include <boost/signals2.hpp>
#include <cvc/gl/SceneNode.h>
#include <cvc/volume/bounding_box.h>
#include <map>
#include <string>
#include <vtkMatrix4x4.h>
#include <vtkPlaneCollection.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

class vtkActor2D;
class BBoxNode;
class vtkPlane;
class VolumeNode;
class NullGraphicNode;

namespace cvc {
class geometry;
class volume;
} // namespace cvc

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

  // Convenience transform methods
  void setPosition(double x, double y, double z);
  void setRotation(double x, double y, double z); // Euler angles in degrees
  void setScale(double x, double y, double z);
  void resetTransform(); // Set to identity matrix

  // Get world transform (accumulated from all parents)
  vtkSmartPointer<vtkMatrix4x4> getWorldTransform() const;

  // Fired ONCE when this node's transform changes (setPosition/setRotation/
  // setScale/setTransform/resetTransform, or a state-driven move) — not once per
  // recursively-updated child. The SceneGraph connects to it to recompute the
  // world bounds/grid so the world box tracks a node that moves out of it.
  boost::signals2::signal<void(GraphicsNode *)> transformChanged;

  // Hierarchical structure

  // Template factory method for creating child graphics nodes
  // Automatically constructs the proper state path based on parent's state
  // Usage: auto node = parent->addGraphicsChild<GeometryNode>("myGeom");
  template <typename T> std::shared_ptr<T> addGraphicsChild(const std::string &name) {
    static_assert(std::is_base_of<GraphicsNode, T>::value, "T must be derived from GraphicsNode");

    // Construct state path: {parent_path}.children.{name}
    std::string childStatePath = getState().fullName() + ".children." + name;

    // Create the child node with proper state path and name
    auto child = std::make_shared<T>(this->app(), childStatePath, name);

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

  // Clipping plane control
  void setClipChildren(bool clip);
  bool getClipChildren() const { return m_clipChildren; }
  vtkPlaneCollection *getClipPlanes() const { return m_clipPlanes; }

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

  // Generic helper to apply world transform to a vector of VTK props
  void applyWorldTransformToProps(const std::vector<vtkProp *> &props);

  // Apply transform to VTK prop - subclasses should override to apply to their specific prop type
  virtual void applyTransformToVTK();

  // Apply clip planes to children - called when clipChildren changes
  void applyClipPlanesToChildren();

  // Apply clip planes to this node's mapper/prop - subclasses override if they support clipping
  virtual void applyClipPlanes(vtkPlaneCollection *planes);

  // Protected members for subclass access
  std::string m_name;
  vtkSmartPointer<vtkMatrix4x4> m_transform;
  // Cached object-to-world state, refreshed top-down by updateTransform().
  // Both exist to keep the per-frame pose path ALLOCATION-FREE: it used to make
  // a fresh vtkMatrix4x4 (inside getWorldTransform, which also recursed up to
  // the root) and a fresh vtkTransform for every node the cascade touched,
  // every frame. Reusing them is what takes the per-node cost down.
  vtkSmartPointer<vtkMatrix4x4> m_worldMatrix;
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
};

#endif // GRAPHICSNODE_H
