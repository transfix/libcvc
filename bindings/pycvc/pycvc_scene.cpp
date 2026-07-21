#include "pycvc_scene.h"

#include "pycvc_geometry.h"
#include "pycvc_volume.h"

#include <cvc/gl/SceneGraph.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkWindowToImageFilter.h>

namespace pycvc {

Scene::Scene() : sg_(std::make_shared<SceneGraph>()) {}
Scene::~Scene() = default;

void Scene::add_geometry(const std::string &name, const Geometry &g) {
  sg_->addGraphics(name, g.native());
}
void Scene::add_volume(const std::string &name, const Volume &v) {
  sg_->addGraphics(name, v.native());
}
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
