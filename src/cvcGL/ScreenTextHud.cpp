// ScreenTextHud — screen-space text overlay for cvcGL viewers (see the header).
// A lean vtkTextActor wrapper: no state binding, no observers — the host frame
// loop drives it (caption tables, status lines). FpsHud is the state-bound
// overlay; this is the direct one.

#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
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

  void applyVisibility() { actor->SetVisibility(visible && hasText ? 1 : 0); }
};

ScreenTextHud::ScreenTextHud(SceneRenderer &viewer) : m_impl(new Impl) {
  m_impl->actor = vtkSmartPointer<vtkTextActor>::New();
  m_impl->renderer = viewer.renderer();

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
}

ScreenTextHud::~ScreenTextHud() {
  if (m_impl->renderer)
    m_impl->renderer->RemoveActor2D(m_impl->actor);
}

void ScreenTextHud::setText(const std::string &text) {
  m_impl->actor->SetInput(text.c_str());
  m_impl->hasText = !text.empty();
  m_impl->applyVisibility();
}

void ScreenTextHud::setPosition(double nx, double ny) {
  m_impl->actor->GetPositionCoordinate()->SetValue(nx, ny);
}

void ScreenTextHud::setFontSize(int points) {
  m_impl->actor->GetTextProperty()->SetFontSize(points);
}

void ScreenTextHud::setColor(double r, double g, double b) {
  m_impl->actor->GetTextProperty()->SetColor(r, g, b);
}

void ScreenTextHud::setOpacity(double alpha) {
  m_impl->actor->GetTextProperty()->SetOpacity(alpha);
}

void ScreenTextHud::setCentered(bool centered) {
  vtkTextProperty *tp = m_impl->actor->GetTextProperty();
  if (centered)
    tp->SetJustificationToCentered();
  else
    tp->SetJustificationToLeft();
}

void ScreenTextHud::setVisible(bool on) {
  m_impl->visible = on;
  m_impl->applyVisibility();
}

} // namespace gl
} // namespace cvc
