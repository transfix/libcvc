#include "pycvc_scene.h"

#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/volume/bounding_box.h>
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
  void setProp(vtkProp *p) {
    m_prop = p;
    // If this node was already attached to a renderer BEFORE the prop was set —
    // the incremental-add path: addGraphicsChild -> addToRenderer runs while
    // getProp() is still null, then setProp is called — attach the prop now so it
    // renders in the LIVE scene (not only after a fresh setRenderer, e.g.
    // render_png). m_renderer / m_visible / runOnMainThread are the SceneNode's.
    if (m_renderer && m_visible && p) {
      vtkRenderer *r = m_renderer;
      vtkProp *prop = p;
      runOnMainThread([r, prop]() { r->AddViewProp(prop); });
    }
  }
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

void render_png(SceneGraph &sg, const std::string &path, int width, int height) {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  window->SetOffScreenRendering(1);
  window->AddRenderer(renderer);
  window->SetSize(width, height);
  sg.setRenderer(renderer); // attaches every node's actor
  sg.processEvents();
  renderer->ResetCamera();
  window->Render();
  vtkNew<vtkWindowToImageFilter> w2i;
  w2i->SetInput(window);
  w2i->Update();
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputConnection(w2i->GetOutputPort());
  writer->Write();
  // Release the GL/offscreen context now, while the render window is still fully
  // alive, rather than leaving it to VTK's static teardown at process exit (which
  // segfaults on some offscreen backends). The scene keeps no ref to this window.
  sg.setRenderer(nullptr);
  window->Finalize();
}

void show(SceneGraph &sg, const std::string &title, int width, int height) {
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> window;
  window->AddRenderer(renderer);
  window->SetSize(width, height);
  window->SetWindowName(title.c_str());
  sg.setRenderer(renderer);
  sg.processEvents();
  renderer->ResetCamera();
  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(window);
  window->Render();
  interactor->Start(); // blocks until the window closes
}

void add_prop(SceneGraph &sg, const std::string &name, vtkProp *prop, double minx, double miny,
              double minz, double maxx, double maxy, double maxz, const std::string &parent) {
  if (!prop)
    throw std::invalid_argument("pycvc_gl.add_prop: null prop");
  // Attach under `parent` (default the graphics root) so the prop inherits that
  // node's transform — e.g. a building mesh as a child of the terrain node.
  std::shared_ptr<GraphicsNode> parentNode =
      parent.empty() ? sg.getGraphicsRoot() : sg.getGraphics(parent);
  if (!parentNode)
    throw std::invalid_argument("pycvc_gl.add_prop: no parent node named '" + parent + "'");
  // addGraphicsChild<T>(name) builds the node with the proper "..children.name"
  // state path and adds it to the render tree under the parent.
  auto node = parentNode->addGraphicsChild<PropNode>(name);
  node->setBounds(cvc::bounding_box(minx, miny, minz, maxx, maxy, maxz));
  node->setProp(prop);
  sg.registerGraphics(name, node); // also expose it in the flat name lookup
}

vtkProp *prop(SceneGraph &sg, const std::string &name) {
  auto pn = std::dynamic_pointer_cast<PropNode>(sg.getGraphics(name));
  return pn ? pn->heldProp() : nullptr;
}

} // namespace pycvc
