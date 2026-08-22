// Prove a LightNode is a real scene node: parenting moves it, hiding kills it,
// and its settings live in the state tree.
//
// The parenting case is the whole reason LightNode exists. A light held beside
// the scene as a plain record cannot ride along with a moving actor; a light
// that is a NODE can. So the test below parents a light under another node and
// moves the PARENT — moving the light directly would prove nothing a LightDesc
// could not already do.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/gl/GraphicsNode.h>
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
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {
    // A passive probe must not spawn handler threads (state_object defaults to
    // threaded); every class under test already sets this false.
    this->setInstanceThreading(false);
  }
  // Reads never throw — cvc::state auto-creates a missing child and value()
  // returns "" — so absence shows up as an empty string, not an exception.
  std::string rd(const std::string &k) { return getState(k).value(); }
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
  for (const char *k : {"kind", "target_x", "cone", "color_r", "intensity"}) {
    const std::string v = P.rd(k);
    chk(!v.empty(), std::string("seeded: ") + k + " = " + v);
  }

  printf("== moving a light moves its vtkLight ==\n");
  double before[3] = {0, 0, 0}, after[3] = {0, 0, 0};
  lightPos(view.renderer(), before);
  lamp->setPosition(100, 0, 20); // move the light itself
  sg.processEvents();
  sg.lightsChanged();
  lightPos(view.renderer(), after);
  printf("     light world pos: (%.0f,%.0f,%.0f) -> (%.0f,%.0f,%.0f)\n", before[0], before[1],
         before[2], after[0], after[1], after[2]);
  chk(after[0] != before[0], "node transform drives the vtkLight position");

  printf("== PARENTING: moving the PARENT moves the light ==\n");
  // Hide the standalone lamp so exactly one light is live and the position read
  // back from the collection is unambiguously the parented one's.
  lamp->setVisible(false);
  auto rig = sg.addGraphics("rig"); // empty node, pure hierarchy
  auto head = rig->addGraphicsChild<cvc::gl::LightNode>("headlight");
  head->setKind(cvc::gl::LightNode::Kind::Spot);
  head->setTarget(0, 0, 0);
  head->setPosition(0, 0, 5); // 5 units above its parent, in PARENT space
  sg.processEvents();
  sg.lightsChanged();
  chk(litCount(view.renderer()) == 1, "a light parented under another node is found by traversal");

  double px = 0, py = 0, pz = 0, qx = 0, qy = 0, qz = 0;
  double vbefore[3] = {0, 0, 0}, vafter[3] = {0, 0, 0};
  head->worldPosition(px, py, pz);
  lightPos(view.renderer(), vbefore);

  rig->setPosition(200, 0, 0); // move the PARENT, never the light
  sg.processEvents();
  sg.lightsChanged();
  head->worldPosition(qx, qy, qz);
  lightPos(view.renderer(), vafter);

  printf("     parent x 0->200; light world x %.1f -> %.1f; vtkLight x %.1f -> %.1f\n", px, qx,
         vbefore[0], vafter[0]);
  chk(std::fabs((qx - px) - 200.0) < 1e-6,
      "parent transform composes into the light's world position");
  chk(std::fabs((vafter[0] - vbefore[0]) - 200.0) < 1e-6,
      "the parent's motion reaches the vtkLight");

  printf("== hiding a light turns it off ==\n");
  const int on = litCount(view.renderer());
  chk(on >= 1, "precondition: a light is live before we hide it");
  // Deliberately NO lightsChanged() here. setVisible() must be sufficient on its
  // own — kicking the scene by hand is exactly what hid the bug where hiding a
  // light left it lit until something unrelated rebuilt the light set.
  head->setVisible(false);
  // No "|| on == 0" escape hatch either: with lights DEFINED but hidden,
  // applyLights deliberately withholds the fallback headlight, so this must
  // reach zero rather than merely decrease.
  chk(litCount(view.renderer()) == 0, "setVisible(false) alone turns the light off");
  head->setVisible(true);
  chk(litCount(view.renderer()) == on, "setVisible(true) alone brings it back");

  printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
