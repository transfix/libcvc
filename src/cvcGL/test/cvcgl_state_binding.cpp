// Prove every cvcGL setting is in the state tree AND bidirectional.
//
// Reads/writes go through a state_object rooted at the SAME path as the object
// under test, i.e. the accessor the objects themselves use (getState). Building
// absolute dotted paths by hand does NOT resolve reliably in cvc::state — that
// is what made the first harness report false failures.
#include <cmath> // fabs — used for the numeric (not string) float comparison
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
  if (!ok)
    ++fails;
}

// A peer state_object rooted at an arbitrary path, so we can read/write the same
// nodes the object under test uses.
class Peer : public cvc::state_object<Peer> {
public:
  Peer(cvc::app &c, const std::string &p) : cvc::state_object<Peer>(c, p) {
    // A passive probe must not spawn handler threads. state_object defaults to
    // threaded, so otherwise every wr() starts a boost::thread and the dtor can
    // block up to 5s per outstanding handler joining them — a multi-second
    // teardown stall on a loaded runner. Every class under test already sets
    // this false; the probe was the only threaded state_object in the process.
    this->setInstanceThreading(false);
  }
  // No throw-detection: cvc::state AUTO-CREATES a missing child and value()
  // returns an empty string, so a read never throws. A "did it throw?" check is
  // therefore vacuous — absence reads as "", and the assertions below compare
  // against the real seeded default instead.
  std::string rd(const std::string &k) { return getState(k).value(); }
  template <class T> void wr(const std::string &k, T v) { getState(k).value(v); }
};

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  SceneGraph sg(app, "audit");
  SceneRenderer view(sg, 320, 240, true, "main");

  std::printf("== StageLighting (audit.lighting) ==\n");
  cvc::gl::StageLighting rig(sg);
  // Derive the path INDEPENDENTLY rather than asking the object under test where
  // it lives. rig.statePath() returning the wrong subtree would silently break
  // every host that addresses the rig by scene prefix, yet a Peer that followed
  // the object to the wrong address would still report every key present.
  check(rig.statePath() == "audit.lighting",
        "rig roots at <prefix>.lighting, got " + rig.statePath());
  Peer L(app, cvc::gl::StageLighting::sceneStatePath(sg.getStatePrefix()));
  for (const char *k : {"enabled", "key_intensity", "key_azimuth", "key_elevation", "key_cone",
                        "fill_intensity", "back_intensity", "wash_intensity", "wash_count",
                        "wash_height", "ambient", "warm_key", "show_gizmos", "stage_x", "stage_y",
                        "stage_z", "stage_radius", "env_intensity", "gizmo_beam_alpha"}) {
    const std::string v = L.rd(k);
    // Non-empty is the real assertion: an unseeded key auto-creates and reads as
    // "", so this fails if seedState() ever forgets one.
    check(!v.empty(), std::string("seeded: ") + k + " = " + v);
  }
  // Compare NUMERICALLY. Values reach state via lexical_cast at 17 significant
  // digits, so the old substr(0,4)=="0.77" accepted anything in [0.77, 0.78) —
  // setWarmth could have been off by up to 1.3% and stayed green.
  rig.setWarmth(0.77);
  const double warm = std::stod(L.rd("warm_key"));
  check(std::fabs(warm - 0.77) < 1e-12, "object -> state (setWarmth) = " + std::to_string(warm));
  // Drive BOTH directions: asserting only the non-default value means flipping a
  // default silently turns the check vacuous.
  L.wr("show_gizmos", 0);
  check(!rig.gizmosVisible(), "state -> object (gizmos off)");
  L.wr("show_gizmos", 1);
  check(rig.gizmosVisible(), "state -> object (gizmos on)");
  L.wr("enabled", 1);
  check(rig.enabled(), "state -> object (enabled on)");
  L.wr("enabled", 0);
  check(!rig.enabled(), "state -> object (enabled off)");

  std::printf("== ScreenTextHud (audit.viewers.main.hud.caption) ==\n");
  cvc::gl::ScreenTextHud hud(view, "caption");
  Peer H(app, cvc::gl::ScreenTextHud::viewerStatePath("audit", "main", "caption"));
  // Assert the SEEDED DEFAULTS, not mere presence: an unseeded key auto-creates
  // and reads as "", so a "did it throw?" check passed even with seedState()
  // deleted outright. "text" is the one key whose default is legitimately empty.
  for (const char *k : {"pos_x", "pos_y", "font_size", "color_r", "color_g", "color_b", "opacity",
                        "centered", "visible"}) {
    const std::string v = H.rd(k);
    check(!v.empty(), std::string("seeded: ") + k + " = '" + v + "'");
  }
  check(H.rd("font_size") == "18", "default: font_size = 18, got '" + H.rd("font_size") + "'");
  check(H.rd("text").empty(), "default: text is empty");
  hud.setText("hello");
  check(H.rd("text") == "hello", "object -> state (setText)");
  hud.setFontSize(31);
  check(H.rd("font_size") == "31", "object -> state (setFontSize)");
  H.wr("font_size", 44);
  check(H.rd("font_size") == "44", "state round-trips (font_size)");
  // Prove the SECOND overlay owns its own node. Checking only that the first was
  // not clobbered passes even if hud2 failed to bind state at all.
  cvc::gl::ScreenTextHud hud2(view, "status");
  hud2.setText("second");
  Peer H2(app, cvc::gl::ScreenTextHud::viewerStatePath("audit", "main", "status"));
  check(H2.rd("text") == "second", "second overlay owns its own node = '" + H2.rd("text") + "'");
  check(H.rd("text") == "hello", "first overlay untouched = '" + H.rd("text") + "'");

  std::printf("== SceneGraph shadows (audit.shadows) ==\n");
  sg.setShadowResolution(2048);
  sg.setShadowUpdateInterval(3);
  Peer S(app, cvc::gl::ShadowSettings::sceneStatePath("audit"));
  // Read each value ONCE into a local: C++ leaves the order of check()'s two
  // arguments unspecified, so reading twice can print a different value than the
  // one tested and produce an unreproducible failure message.
  const std::string res = S.rd("resolution"), iv = S.rd("interval"), en = S.rd("enabled");
  check(res == "2048", "object -> state (resolution) = " + res);
  check(iv == "3", "object -> state (interval) = " + iv);
  check(en == "0", "default: shadows disabled, got '" + en + "'");
  // shadowsEnabled() was never exercised — drive the flag both ways.
  S.wr("enabled", 1);
  check(sg.shadowsEnabled(), "state -> object (shadows on)");
  S.wr("enabled", 0);
  check(!sg.shadowsEnabled(), "state -> object (shadows off)");
  S.wr("resolution", 512);
  check(sg.shadowResolution() == 512,
        "state -> object (resolution now " + std::to_string(sg.shadowResolution()) + ")");

  std::printf("\n%s (%d failure%s)\n", fails ? "AUDIT FAILED" : "AUDIT PASSED", fails,
              fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
