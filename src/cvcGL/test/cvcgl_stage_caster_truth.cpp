// The rig's advertised shadow-caster count must equal what VTK actually bakes.
//
// StageLighting::shadowCasterCount() claimed "fill and wash deliberately do
// not" cast, and the panel printed that number — but both were built as
// Kind::Spot with cones under 90, which is exactly the condition VTK's
// LightCreatesShadow() ACCEPTS. So the rig reported 2 casters while the
// renderer baked a depth map for seven, and every extra wash light silently
// added a full scene re-render per bake.
//
// A comment cannot enforce that. This does: it counts the renderer's lights
// with VTK's own predicate and compares against the advertised number, across a
// sweep of wash counts. It also pins the intent directly — fill and wash must
// report castsShadow() == false — so a future change back to Spot fails here
// rather than quietly costing frames.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/LightNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <string>

static int fails = 0;
static void chk(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok)
    ++fails;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "sct");
  SceneRenderer view(sg, 160, 120, /*offscreen=*/true, "main");
  cvc::gl::StageLighting rig(sg);
  rig.setStage(0, 0, 0, 10);

  // Count exactly what SceneGraph hands VTK: a LightNode that is visible and
  // reports castsShadow(). LightNode::castsShadow mirrors LightCreatesShadow —
  // positional with cone < 90, or directional.
  auto bakedCasters = [&]() {
    int n = 0;
    for (const auto &name : rig.lightNames())
      if (auto ln = std::dynamic_pointer_cast<cvc::gl::LightNode>(sg.getGraphics(name)))
        if (ln->isVisible() && ln->castsShadow())
          ++n;
    return n;
  };

  std::printf("== advertised count matches the real one, across the wash sweep ==\n");
  for (int wash : {0, 1, 2, 4, 8}) {
    rig.setWash(0.3, wash, 1.8);
    sg.processEvents();
    const int said = rig.shadowCasterCount();
    const int real = bakedCasters();
    chk(said == real, "wash=" + std::to_string(wash) + ": reports " + std::to_string(said) +
                          ", VTK bakes " + std::to_string(real));
  }

  std::printf("== the wash must not add casters at all ==\n");
  rig.setWash(0.3, 0, 1.8);
  sg.processEvents();
  const int base = bakedCasters();
  rig.setWash(0.3, 8, 1.8);
  sg.processEvents();
  chk(bakedCasters() == base,
      "8 wash lights cost 0 extra shadow maps (was +8, one depth re-render each)");

  std::printf("== roles: only key and back cast ==\n");
  struct {
    const char *name;
    bool casts;
  } kExpect[] = {
      {"stage_key", true}, {"stage_back", true}, {"stage_fill", false}, {"stage_wash_0", false}};
  for (const auto &e : kExpect) {
    auto ln = std::dynamic_pointer_cast<cvc::gl::LightNode>(sg.getGraphics(e.name));
    if (!ln) {
      chk(false, std::string(e.name) + " exists");
      continue;
    }
    chk(ln->castsShadow() == e.casts,
        std::string(e.name) + (e.casts ? " casts" : " does NOT cast"));
  }

  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
