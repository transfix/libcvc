// ScreenTextHud — screen-space text overlay for cvcGL viewers (see the header).
// A lean vtkTextActor wrapper: no state binding, no observers — the host frame
// loop drives it (caption tables, status lines). FpsHud is the state-bound
// overlay; this is the direct one.

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
#include <stdexcept>
#include <vtkCoordinate.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace cvc {
namespace gl {

struct ScreenTextHud::Impl {
  vtkSmartPointer<vtkTextActor> actor;
  vtkSmartPointer<vtkRenderer> renderer;
  bool visible = true; // caller intent; AND'ed with hasText
  bool hasText = false;

  // Mirror of every setting, so a write can be skipped when nothing changed.
  // A caption table re-asserts the same string every frame; without this gate
  // that would be one state write (and one observer fan-out, and one replication
  // message) per frame for a caption that changes twice a minute.
  std::string text;
  double px = 0.5, py = 0.06;
  int fontSize = 18;
  double r = 0.95, g = 0.93, b = 0.88;
  double opacity = 1.0;
  bool centered = true;

  void applyVisibility() { actor->SetVisibility(visible && hasText ? 1 : 0); }
};

std::string ScreenTextHud::viewerStatePath(const std::string &scenePrefix,
                                           const std::string &viewerName, const std::string &name) {
  return scenePrefix + ".viewers." + viewerName + ".hud." + name;
}

ScreenTextHud::ScreenTextHud(cvc::app &ctx, const std::string &statePath, SceneRenderer *viewer)
    : cvc::state_object<ScreenTextHud>(ctx, statePath), m_impl(new Impl) {
  // Synchronous reactions on the calling thread, like FpsHud and
  // CameraController: this is driven from the render thread.
  this->setInstanceThreading(false);
  m_impl->actor = vtkSmartPointer<vtkTextActor>::New();
  m_impl->renderer = viewer ? viewer->renderer() : nullptr;

  // Normalized-viewport placement so positions survive resizes and capture
  // sizes; lower-third caption band by default.
  m_impl->actor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
  m_impl->actor->GetPositionCoordinate()->SetValue(0.5, 0.06);

  vtkTextProperty *tp = m_impl->actor->GetTextProperty();
  tp->SetFontFamilyToArial();
  tp->SetFontSize(18);
  tp->SetColor(0.95, 0.93, 0.88); // warm white
  tp->SetOpacity(1.0);
  tp->SetJustificationToCentered();
  tp->SetVerticalJustificationToBottom();
  tp->SetShadow(1); // readable over any scene, light or dark

  m_impl->actor->SetVisibility(0); // hidden until text is set
  if (m_impl->renderer)
    m_impl->renderer->AddActor2D(m_impl->actor);
  seedState();
}

ScreenTextHud::ScreenTextHud(SceneRenderer &viewer, const std::string &name)
    : ScreenTextHud(viewer.scene().appContext(),
                    viewerStatePath(viewer.scene().getStatePrefix(), viewer.name(), name),
                    &viewer) {}

void ScreenTextHud::seedState() {
  Impl &s = *m_impl;
  getState("text").value(s.text);
  getState("pos_x").value(s.px);
  getState("pos_y").value(s.py);
  getState("font_size").value(s.fontSize);
  getState("color_r").value(s.r);
  getState("color_g").value(s.g);
  getState("color_b").value(s.b);
  getState("opacity").value(s.opacity);
  getState("centered").value(s.centered ? 1 : 0);
  getState("visible").value(s.visible ? 1 : 0);
}

void ScreenTextHud::readAllFromState() {
  Impl &s = *m_impl;
  try {
    s.text = getState("text").value();
    s.px = getState("pos_x").value<double>();
    s.py = getState("pos_y").value<double>();
    s.fontSize = getState("font_size").value<int>();
    s.r = getState("color_r").value<double>();
    s.g = getState("color_g").value<double>();
    s.b = getState("color_b").value<double>();
    s.opacity = getState("opacity").value<double>();
    s.centered = getState("centered").value<int>() != 0;
    s.visible = getState("visible").value<int>() != 0;
  } catch (const std::exception &) {
    // partially-initialised state: keep what we have
  }
}

// Push the mirrored settings onto the VTK actor. Used when state changed
// underneath us (a script, a peer); the setters apply directly.
void ScreenTextHud::applyToActor() {
  Impl &s = *m_impl;
  s.actor->SetInput(s.text.c_str());
  s.hasText = !s.text.empty();
  s.actor->GetPositionCoordinate()->SetValue(s.px, s.py);
  vtkTextProperty *tp = s.actor->GetTextProperty();
  tp->SetFontSize(s.fontSize);
  tp->SetColor(s.r, s.g, s.b);
  tp->SetOpacity(s.opacity);
  if (s.centered)
    tp->SetJustificationToCentered();
  else
    tp->SetJustificationToLeft();
  s.applyVisibility();
}

void ScreenTextHud::handleStateChanged(const std::string &) {
  readAllFromState();
  applyToActor();
}

ScreenTextHud::~ScreenTextHud() {
  if (m_impl->renderer)
    m_impl->renderer->RemoveActor2D(m_impl->actor);
}

void ScreenTextHud::setText(const std::string &text) {
  if (text == m_impl->text)
    return; // the caption-table case: same string every frame, no state write
  m_impl->text = text;
  m_impl->actor->SetInput(text.c_str());
  m_impl->hasText = !text.empty();
  m_impl->applyVisibility();
  getState("text").value(text);
}

void ScreenTextHud::setPosition(double nx, double ny) {
  if (nx == m_impl->px && ny == m_impl->py)
    return;
  m_impl->px = nx;
  m_impl->py = ny;
  m_impl->actor->GetPositionCoordinate()->SetValue(nx, ny);
  getState("pos_x").value(nx);
  getState("pos_y").value(ny);
}

void ScreenTextHud::setFontSize(int points) {
  if (points == m_impl->fontSize)
    return;
  m_impl->fontSize = points;
  m_impl->actor->GetTextProperty()->SetFontSize(points);
  getState("font_size").value(points);
}

void ScreenTextHud::setColor(double r, double g, double b) {
  if (r == m_impl->r && g == m_impl->g && b == m_impl->b)
    return;
  m_impl->r = r;
  m_impl->g = g;
  m_impl->b = b;
  m_impl->actor->GetTextProperty()->SetColor(r, g, b);
  getState("color_r").value(r);
  getState("color_g").value(g);
  getState("color_b").value(b);
}

void ScreenTextHud::setOpacity(double alpha) {
  if (alpha == m_impl->opacity)
    return;
  m_impl->opacity = alpha;
  m_impl->actor->GetTextProperty()->SetOpacity(alpha);
  getState("opacity").value(alpha);
}

void ScreenTextHud::setCentered(bool centered) {
  if (centered == m_impl->centered)
    return;
  m_impl->centered = centered;
  getState("centered").value(centered ? 1 : 0);
  vtkTextProperty *tp = m_impl->actor->GetTextProperty();
  if (centered)
    tp->SetJustificationToCentered();
  else
    tp->SetJustificationToLeft();
}

void ScreenTextHud::setVisible(bool on) {
  if (on == m_impl->visible)
    return;
  m_impl->visible = on;
  m_impl->applyVisibility();
  getState("visible").value(on ? 1 : 0);
}

} // namespace gl
} // namespace cvc
