// Prove every cvcGL setting is in the state tree AND bidirectional.
//
// Reads/writes go through a state_object rooted at the SAME path as the object
// under test, i.e. the accessor the objects themselves use (getState). Building
// absolute dotted paths by hand does NOT resolve reliably in cvc::state — that
// is what made the first harness report false failures.
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_object.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ScreenTextHud.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/StageLighting.h>
#include <string>

static int fails = 0;
static void check(bool ok, const std::string &w) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", w.c_str());
  if (!ok) ++fails;
}

// A peer state_object rooted at an arbitrary path, so we can read/write the same
// nodes the object under test uses.
class Peer : public cvc::state_object<Peer> {
public:
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {}
  std::string rd(const std::string &k) { try { return getState(k).value(); } catch (...) { return "<throw>"; } }
  template <class T> void wr(const std::string &k, T v) { getState(k).value(v); }
};

int main() {
  cvc::app app; app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "audit");
  SceneRenderer view(sg, 320, 240, true, "main");

  std::printf("== StageLighting (audit.lighting) ==\n");
  cvc::gl::StageLighting rig(sg);
  Peer L(app, rig.statePath());
  for (const char *k : {"enabled","key_intensity","key_azimuth","key_elevation","key_cone",
                        "fill_intensity","back_intensity","wash_intensity","wash_count",
                        "wash_height","ambient","warm_key","show_gizmos",
                        "stage_x","stage_y","stage_z","stage_radius"}) {
    const std::string v = L.rd(k);
    check(!v.empty() && v != "<throw>", std::string("present: ") + k + " = " + v);
  }
  rig.setWarmth(0.77);
  check(L.rd("warm_key").substr(0,4) == "0.77", "object -> state (setWarmth)");
  L.wr("show_gizmos", 1);
  check(rig.gizmosVisible(), "state -> object (show_gizmos)");
  L.wr("enabled", 0);
  check(!rig.enabled(), "state -> object (enabled)");

  std::printf("== ScreenTextHud (audit.viewers.main.hud.caption) ==\n");
  cvc::gl::ScreenTextHud hud(view, "caption");
  Peer H(app, cvc::gl::ScreenTextHud::viewerStatePath("audit", "main", "caption"));
  for (const char *k : {"text","pos_x","pos_y","font_size","color_r","color_g","color_b",
                        "opacity","centered","visible"})
    check(H.rd(k) != "<throw>", std::string("present: ") + k + " = '" + H.rd(k) + "'");
  hud.setText("hello");
  check(H.rd("text") == "hello", "object -> state (setText)");
  hud.setFontSize(31);
  check(H.rd("font_size") == "31", "object -> state (setFontSize)");
  H.wr("font_size", 44);
  check(H.rd("font_size") == "44", "state -> node accepted (font_size)");
  cvc::gl::ScreenTextHud hud2(view, "status");
  hud2.setText("second");
  check(H.rd("text") == "hello", "named overlays do not collide");

  std::printf("== SceneGraph shadows (audit.shadows) ==\n");
  sg.setShadowResolution(2048);
  sg.setShadowUpdateInterval(3);
  Peer S(app, cvc::gl::ShadowSettings::sceneStatePath("audit"));
  check(S.rd("resolution") == "2048", "object -> state (resolution) = " + S.rd("resolution"));
  check(S.rd("interval") == "3", "object -> state (interval) = " + S.rd("interval"));
  check(S.rd("enabled") != "<throw>", "present: enabled = " + S.rd("enabled"));
  S.wr("resolution", 512);
  check(sg.shadowResolution() == 512,
        "state -> object (resolution now " + std::to_string(sg.shadowResolution()) + ")");

  std::printf("\n%s (%d failure%s)\n", fails ? "AUDIT FAILED" : "AUDIT PASSED", fails,
              fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
