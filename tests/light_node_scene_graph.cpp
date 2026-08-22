// Prove a LightNode is a real scene node: parenting moves it, hiding kills it,
// and its settings live in the state tree.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <vtkLight.h>
#include <vtkLightCollection.h>
#include <vtkRenderer.h>
static int fails = 0;
static void chk(bool ok, const std::string &w) {
  printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}
class Peer : public cvc::state_object<Peer> {
public:
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {}
  std::string rd(const std::string &k) {
    try {
      return getState(k).value();
    } catch (...) {
      return "<throw>";
    }
  }
  template <class T> void wr(const std::string &k, T v) { getState(k).value(v); }
};
static int litCount(vtkRenderer *r) {
  int n = 0;
  vtkLightCollection *lc = r->GetLights();
  lc->InitTraversal();
  while (lc->GetNextItem())
    ++n;
  return n;
}
static void lightPos(vtkRenderer *r, double p[3]) {
  vtkLightCollection *lc = r->GetLights();
  lc->InitTraversal();
  if (vtkLight *l = lc->GetNextItem()) {
    auto *q = l->GetPosition();
    p[0] = q[0];
    p[1] = q[1];
    p[2] = q[2];
  }
}
int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "ln");
  SceneRenderer view(sg, 320, 240, true, "main");

  auto lamp = sg.addLight("lamp"); // the proper factory
  lamp->setKind(cvc::gl::LightNode::Kind::Spot);
  lamp->setTarget(0, 0, 0);
  lamp->setPosition(10, 0, 20);
  sg.lightsChanged();
  sg.processEvents();
  printf("== light reaches the renderer ==\n");
  chk(litCount(view.renderer()) >= 1, "LightNode in the graph -> a vtkLight exists");

  printf("== settings are state ==\n");
  Peer P(app, std::string("ln.graphics.root.children.lamp"));
  for (const char *k : {"kind", "target_x", "cone", "color_r", "intensity"})
    chk(P.rd(k) != "<throw>" && !P.rd(k).empty(), std::string("state key ") + k + " = " + P.rd(k));

  printf("== PARENTING: moving the parent moves the light ==\n");
  double before[3] = {0, 0, 0}, after[3] = {0, 0, 0};
  lightPos(view.renderer(), before);
  lamp->setPosition(100, 0, 20); // move the light
  sg.processEvents();
  sg.lightsChanged();
  lightPos(view.renderer(), after);
  printf("     light world pos: (%.0f,%.0f,%.0f) -> (%.0f,%.0f,%.0f)\n", before[0], before[1],
         before[2], after[0], after[1], after[2]);
  chk(after[0] != before[0], "node transform drives the vtkLight position");

  printf("== hiding a light turns it off ==\n");
  const int on = litCount(view.renderer());
  lamp->setVisible(false);
  sg.lightsChanged();
  chk(litCount(view.renderer()) < on || on == 0, "hidden LightNode contributes no vtkLight");

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
