#ifndef CVC_GL_LIGHT_NODE_H
#define CVC_GL_LIGHT_NODE_H

#include <cvc/gl/GraphicsNode.h>
#include <cvc/volume/bounding_box.h>
#include <string>

namespace cvc {
namespace gl {

// ------------
// LightNode
// ------------
// A light that is an actual NODE IN THE SCENE GRAPH, not a description held off
// to one side.
//
// SceneGraph's original lights were LightDesc records applied straight to the
// vtkRenderer. That worked, but it meant a light was not a scene object: it did
// not appear in the hierarchy, could not be parented to anything, had no
// transform, and had no state node of its own — so "move the lamp with the
// vehicle" or "script this one light" were both impossible. A LightNode is a
// GraphicsNode, so it inherits all of that for free:
//
//   * POSITION comes from the node transform, which means PARENTING WORKS.
//     Parent a spot to a moving actor and it travels with it — a headlight, a
//     follow-spot, a lamp on a vehicle — with no per-frame bookkeeping.
//   * STATE comes from SceneNode's state_object, so every property below lives
//     under the node's own state path and is scriptable and replicable like any
//     other node.
//   * VISIBILITY is the node's: hiding a LightNode turns the light off, which is
//     what everyone expects hiding a light to do.
//
// SceneGraph finds these by traversing the graph (collectNodes<LightNode>), so
// simply adding one to the scene lights it; there is nothing else to register.
//
// KIND, and the shadow rule that goes with it:
//   Spot        positional + cone < 90. Casts a shadow, and the cone IS the
//               shadow-map frustum, so a tight cone spends the map on the
//               subject. This is the one to reach for.
//   Directional a sun. Casts, but VTK fits its shadow map to the WHOLE scene
//               bounding box, so one map is stretched across everything.
//   Fill        positional + cone >= 90. Deliberately casts NO shadow (VTK's
//               LightCreatesShadow drops it), for lighting the environment
//               outside the cones at zero cost to shadow quality.
class LightNode : public GraphicsNode {
public:
  enum class Kind { Spot, Directional, Fill };

  LightNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "light");
  ~LightNode() override;

  // What sort of light this is (state "kind": spot|directional|fill).
  void setKind(Kind k);
  Kind kind() const;

  // Where it AIMS, in world coordinates (state "target_x/y/z"). The light's own
  // position is the node's transform, so aim and placement are separate: move
  // the node (or its parent) to move the lamp, set the target to re-aim it.
  void setTarget(double x, double y, double z);
  void target(double &x, double &y, double &z) const;

  // Cone half-angle in degrees (state "cone"). Clamped to (0, 89.5] for Spot —
  // VTK silently drops a positional light at 90+ from the shadow bake, so a
  // "90 degree spot" would mean no shadow rather than a wide one. Fill is
  // exempt: it wants exactly that exclusion.
  void setCone(double deg);
  double cone() const;

  // Direction for a Directional light, as a compass bearing and height angle
  // (state "azimuth"/"elevation"), matching SceneGraph::addDirectionalLight.
  void setDirection(double azimuthDeg, double elevationDeg);

  void setColor(double r, double g, double b); // state "color_r/g/b"
  void color(double &r, double &g, double &b) const;
  void setIntensity(double i); // state "intensity"
  double intensity() const;

  // True when this light contributes a shadow map (see the Kind notes).
  bool castsShadow() const;

  // World position of the light, i.e. the node transform applied to the origin.
  // This is what makes parenting work, and what SceneGraph feeds to VTK.
  void worldPosition(double &x, double &y, double &z) const;

  // Hiding a light must actually turn it off. The renderer's light set is
  // rebuilt from the graph rather than mutated in place, so visibility — unlike
  // a GeometryNode's, which VTK honours on the actor — only takes effect once
  // the scene rebuilds. Overridden to trigger that rebuild, so setVisible()
  // needs no accompanying lightsChanged() call to mean anything.
  void setVisible(bool visible) override;

  // A light draws nothing itself (its gizmo, if any, is a separate node), so it
  // has no prop and contributes no bounds — a lamp hung far off stage must not
  // drag the scene's bounding box out with it.
  vtkProp *getProp() override { return nullptr; }
  cvc::bounding_box getBoundingBox() const override { return cvc::bounding_box(); }

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void seedState();
  void readAllFromState();
  void notifyScene(); // ask the owning scene to rebuild its lights

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_LIGHT_NODE_H
