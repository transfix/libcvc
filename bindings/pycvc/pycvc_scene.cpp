#include "pycvc_scene.h"

#include "pycvc_geometry.h"
#include "pycvc_volume.h"

#include <cvc/gl/SceneGraph.h>

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

} // namespace pycvc
