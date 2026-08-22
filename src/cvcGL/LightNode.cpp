// LightNode — a light that is a real scene-graph node (see the header for why).

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <stdexcept>
#include <vtkMatrix4x4.h>

namespace cvc {
namespace gl {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;

const char *kindToString(LightNode::Kind k) {
  switch (k) {
  case LightNode::Kind::Spot:
    return "spot";
  case LightNode::Kind::Directional:
    return "directional";
  case LightNode::Kind::Fill:
    return "fill";
  }
  return "spot";
}
LightNode::Kind kindFromString(const std::string &s) {
  if (s == "directional")
    return LightNode::Kind::Directional;
  if (s == "fill")
    return LightNode::Kind::Fill;
  return LightNode::Kind::Spot;
}
} // namespace

struct LightNode::Impl {
  LightNode::Kind kind = LightNode::Kind::Spot;
  double tx = 0, ty = 0, tz = 0;
  double cone = 30.0;
  double az = 0, el = 45.0; // Directional only
  double r = 1, g = 1, b = 1;
  double intensity = 1.0;
  bool inStateApply = false; // guard: state -> setter -> state
};

LightNode::LightNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
    : GraphicsNode(ctx, statePath, name), m_impl(std::make_unique<Impl>()) {
  seedState();
}

LightNode::~LightNode() = default;

void LightNode::seedState() {
  Impl &s = *m_impl;
  getState("kind").value(std::string(kindToString(s.kind)));
  getState("target_x").value(s.tx);
  getState("target_y").value(s.ty);
  getState("target_z").value(s.tz);
  getState("cone").value(s.cone);
  getState("azimuth").value(s.az);
  getState("elevation").value(s.el);
  getState("color_r").value(s.r);
  getState("color_g").value(s.g);
  getState("color_b").value(s.b);
  getState("intensity").value(s.intensity);
}

void LightNode::readAllFromState() {
  Impl &s = *m_impl;
  try {
    s.kind = kindFromString(getState("kind").value());
    s.tx = getState("target_x").value<double>();
    s.ty = getState("target_y").value<double>();
    s.tz = getState("target_z").value<double>();
    s.cone = getState("cone").value<double>();
    s.az = getState("azimuth").value<double>();
    s.el = getState("elevation").value<double>();
    s.r = getState("color_r").value<double>();
    s.g = getState("color_g").value<double>();
    s.b = getState("color_b").value<double>();
    s.intensity = getState("intensity").value<double>();
  } catch (const std::exception &) {
    // partially-initialised state: keep what we have
  }
}

void LightNode::handleStateChanged(const std::string &childState) {
  // Pose keys are the base class's business; anything else is ours.
  GraphicsNode::handleStateChanged(childState);
  if (m_impl->inStateApply)
    return;
  m_impl->inStateApply = true;
  readAllFromState();
  m_impl->inStateApply = false;
  notifyScene();
}

void LightNode::notifyScene() {
  // The scene rebuilds its whole light set from the graph; batching in
  // SceneGraph keeps that to one rebuild even when several lights change.
  if (SceneGraph *sg = getSceneGraph())
    sg->lightsChanged();
}

void LightNode::setVisible(bool visible) {
  const bool was = isVisible();
  GraphicsNode::setVisible(visible);
  // Only rebuild on an actual change: applyLights() drops and recreates every
  // vtkLight and re-bakes a shadow map per caster, which is far too expensive to
  // spend on a no-op set.
  if (was != visible)
    notifyScene();
}

void LightNode::setKind(Kind k) {
  if (k == m_impl->kind)
    return;
  m_impl->kind = k;
  getState("kind").value(std::string(kindToString(k)));
  notifyScene();
}
LightNode::Kind LightNode::kind() const { return m_impl->kind; }

void LightNode::setTarget(double x, double y, double z) {
  Impl &s = *m_impl;
  if (x == s.tx && y == s.ty && z == s.tz)
    return; // change-gated: a rig rebuild re-asserts the same values constantly
  s.tx = x;
  s.ty = y;
  s.tz = z;
  getState("target_x").value(x);
  getState("target_y").value(y);
  getState("target_z").value(z);
  notifyScene();
}
void LightNode::target(double &x, double &y, double &z) const {
  x = m_impl->tx;
  y = m_impl->ty;
  z = m_impl->tz;
}

void LightNode::setCone(double deg) {
  {
    const double want = (m_impl->kind == Kind::Fill) ? std::max(90.0, std::min(179.0, deg))
                                                     : std::max(0.5, std::min(89.5, deg));
    if (want == m_impl->cone)
      return;
  }
  // Fill WANTS >= 90 (that is how it opts out of the shadow bake); a Spot must
  // stay below it or VTK drops the shadow entirely.
  m_impl->cone = (m_impl->kind == Kind::Fill) ? std::max(90.0, std::min(179.0, deg))
                                              : std::max(0.5, std::min(89.5, deg));
  getState("cone").value(m_impl->cone);
  notifyScene();
}
double LightNode::cone() const { return m_impl->cone; }

void LightNode::setDirection(double azimuthDeg, double elevationDeg) {
  m_impl->az = azimuthDeg;
  m_impl->el = elevationDeg;
  getState("azimuth").value(azimuthDeg);
  getState("elevation").value(elevationDeg);
  notifyScene();
}

void LightNode::setColor(double r, double g, double b) {
  Impl &s = *m_impl;
  if (r == s.r && g == s.g && b == s.b)
    return;
  s.r = r;
  s.g = g;
  s.b = b;
  getState("color_r").value(r);
  getState("color_g").value(g);
  getState("color_b").value(b);
  notifyScene();
}
void LightNode::color(double &r, double &g, double &b) const {
  r = m_impl->r;
  g = m_impl->g;
  b = m_impl->b;
}

void LightNode::setIntensity(double i) {
  if (i == m_impl->intensity)
    return;
  m_impl->intensity = i;
  getState("intensity").value(i);
  notifyScene();
}
double LightNode::intensity() const { return m_impl->intensity; }

bool LightNode::castsShadow() const {
  // Matches VTK's LightCreatesShadow(): everything except a >= 90 degree cone.
  return m_impl->kind != Kind::Fill;
}

void LightNode::worldPosition(double &x, double &y, double &z) const {
  // The node transform applied to the local origin — this is what makes a light
  // parented to a moving actor travel with it.
  if (m_worldMatrix) {
    x = m_worldMatrix->GetElement(0, 3);
    y = m_worldMatrix->GetElement(1, 3);
    z = m_worldMatrix->GetElement(2, 3);
  } else {
    x = y = z = 0.0;
  }
}

} // namespace gl
} // namespace cvc
