#include "pycvc_scene.h"

#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/volume.h>
#include <stdexcept>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkProp.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkWindowToImageFilter.h>

namespace pycvc {

Scene::Scene(const std::shared_ptr<cvc::app> &app, const std::string &state_prefix)
    : app_(app), sg_(app ? std::make_shared<SceneGraph>(*app, state_prefix) : nullptr) {
  if (!app)
    throw std::invalid_argument("pycvc.Scene: null app handle");
}
Scene::~Scene() = default;

namespace {
// A concrete GraphicsNode that renders a caller-supplied vtkProp (e.g. a
// Python-built vtkActor). cvcGL's GraphicsNode is abstract (getProp/getBoundingBox
// are virtual); this adapter just hands back the prop it was given, so a Python
// VTK object flows into the C++ render tree via getProp(). Modeled on
// NullGraphicNode. vtkSmartPointer keeps the prop alive while the node lives.
class PropNode : public GraphicsNode {
public:
  PropNode(cvc::app &ctx, const std::string &statePath, const std::string &name)
      : GraphicsNode(ctx, statePath, name) {}
  void setProp(vtkProp *p) { m_prop = p; }
  void setBounds(const cvc::bounding_box &b) { m_bounds = b; }
  vtkProp *heldProp() const { return m_prop; }
  cvc::bounding_box getBoundingBox() const override { return m_bounds; }

protected:
  vtkProp *getProp() override { return m_prop; }

private:
  vtkSmartPointer<vtkProp> m_prop;
  cvc::bounding_box m_bounds;
};
} // namespace

void Scene::add_geometry(const std::string &name, const cvc::geometry &g) {
  sg_->addGraphics(name, g);
}

void Scene::add_prop(const std::string &name, vtkProp *prop, double minx, double miny, double minz,
                     double maxx, double maxy, double maxz) {
  if (!prop)
    throw std::invalid_argument("pycvc Scene.add_prop: null prop");
  // addGraphicsChild<T>(name) builds the node with the proper "..children.name"
  // state path and adds it to the render tree (works for a C++ type like
  // PropNode; only a *Python* subclass couldn't be make_shared'd this way).
  auto node = sg_->getGraphicsRoot()->addGraphicsChild<PropNode>(name);
  node->setBounds(cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz));
  node->setProp(prop);
  sg_->registerGraphics(name, node); // also expose it in the flat name lookup
}

vtkProp *Scene::prop(const std::string &name) const {
  auto pn = std::dynamic_pointer_cast<PropNode>(sg_->getGraphics(name));
  return pn ? pn->heldProp() : nullptr;
}
void Scene::add_volume(const std::string &name, const cvc::volume &v) { sg_->addGraphics(name, v); }
void Scene::pump() { sg_->processEvents(); }
std::size_t Scene::num_graphics() const { return sg_->getAllGraphics().size(); }
bool Scene::has(const std::string &name) const { return static_cast<bool>(sg_->getGraphics(name)); }

void Scene::show(const std::string &title, int width, int height) {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  window->AddRenderer(renderer);
  window->SetSize(width, height);
  window->SetWindowName(title.c_str());
  sg_->setRenderer(renderer); // attaches all node actors
  sg_->processEvents();
  renderer->ResetCamera();
  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(window);
  window->Render();
  interactor->Start(); // blocks until the window closes
}

void Scene::render_png(const std::string &path, int width, int height) {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  window->SetOffScreenRendering(1);
  window->AddRenderer(renderer);
  window->SetSize(width, height);
  sg_->setRenderer(renderer);
  sg_->processEvents();
  renderer->ResetCamera();
  window->Render();
  vtkNew<vtkWindowToImageFilter> w2i;
  w2i->SetInput(window);
  w2i->Update();
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(w2i->GetOutputPort());
  writer->Write();
}

} // namespace pycvc
