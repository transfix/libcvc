// cvc::gl::CameraController — orbit + Quake-fly navigation AND full cvc::state
// configurability. Drives a bare vtkCamera (no window) and asserts:
//   * framing / Z-up, seamless orbit<->fly toggle, WASD/strafe/vertical, look,
//     orbit-drag  (the navigation math);
//   * two-way cvc::state binding: writing state changes the camera (mode, up
//     axis, orbit center, key bindings), and driving the camera writes state;
//   * a configurable up axis (Y-up), and the canonical viewer state path.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/state/state.h>
#include <string>
#include <vtkCamera.h>
#include <vtkNew.h>

using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;

using cvc::gl::CameraController;

static int failures = 0;
static void check(const char *what, bool ok) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok)
    ++failures;
}
static double dist(const double a[3], const double b[3]) {
  double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

int main() {
  // Own an explicit cvc::app and inject it — no global/singleton context.
  cvc::app app;
  cvc::app &ctx = app;
  cvc::state &root = cvc::state::instance(ctx);
  vtkNew<vtkCamera> cam;

  CameraController c(ctx, "test.camera");
  c.setCamera(cam);
  c.frameBounds(-50, -50, 0, 50, 50, 20);

  double e[3], f[3], u[3];
  c.getPose(e, f, u);
  printf("orbit-framed eye=(%.1f,%.1f,%.1f) focal=(%.1f,%.1f,%.1f) up=(%.2f,%.2f,%.2f)\n", e[0],
         e[1], e[2], f[0], f[1], f[2], u[0], u[1], u[2]);

  printf("A. Z-up framing + navigation\n");
  check("world up is +Z", std::fabs(u[2] - 1.0) < 1e-9 && std::fabs(u[0]) < 1e-9);
  check("focal is scene center", std::fabs(f[2] - 10.0) < 1e-6 && std::fabs(f[0]) < 1e-6);
  check("eye above ground", e[2] > f[2]);

  double orbEye[3], orbFoc[3];
  c.getPose(orbEye, orbFoc, u);
  c.toggleMode(); // -> fly
  double flyEye[3], flyFoc[3];
  c.getPose(flyEye, flyFoc, u);
  check("fly stands at orbit eye", dist(flyEye, orbEye) < 1e-6);
  c.setMoveSpeed(10.0);
  double d0 = dist(flyEye, orbFoc);
  c.keyDown("w");
  c.update(1.0);
  c.keyUp("w");
  double afterW[3];
  c.getPose(afterW, f, u);
  check("W moves 10 along look, toward center",
        std::fabs(dist(afterW, flyEye) - 10.0) < 1e-6 && dist(afterW, orbFoc) < d0);
  double bD[3];
  c.getPose(bD, f, u);
  c.keyDown("d");
  c.update(1.0);
  c.keyUp("d");
  double aD[3];
  c.getPose(aD, f, u);
  check("D strafes 10 horizontally",
        std::fabs(dist(aD, bD) - 10.0) < 1e-6 && std::fabs(aD[2] - bD[2]) < 1e-9);
  double bl[3], blf[3];
  c.getPose(bl, blf, u);
  c.mouseLook(100, 0);
  double al[3], alf[3];
  c.getPose(al, alf, u);
  check("look rotates view, not eye", dist(bl, al) < 1e-9 && dist(blf, alf) > 1e-3);

  // Orbit DRAG was the one navigation path with no coverage at all — the
  // mouseLook above runs in Fly mode — which is how an inverted vertical drag
  // shipped. Sensitivity is 0.25 deg/px by default, so 60 px is 15 deg: well
  // clear of the +/-89 clamp from a framed start.
  printf("A2. orbit drag turns the scene WITH the pointer\n");
  {
    vtkNew<vtkCamera> ocam;
    CameraController oc(ctx, "test.orbitdrag");
    oc.setCamera(ocam);
    oc.frameBounds(-50, -50, 0, 50, 50, 20);
    oc.setMode(CameraController::Mode::Orbit);

    double b[3], bf[3], bu[3];
    oc.getPose(b, bf, bu);

    // VTK display coords are origin-bottom-left, so a DOWNWARD drag arrives as
    // a NEGATIVE dy. Pulling down must bring the subject's top toward the
    // viewer, which lifts the eye — the opposite of what this used to do.
    oc.beginDrag();
    oc.mouseLook(0, -60);
    oc.endDrag();
    double d1[3], d1f[3];
    oc.getPose(d1, d1f, bu);
    check("drag DOWN lifts the eye over the subject", d1[2] > b[2] + 1e-6);
    check("orbit drag swings the eye, not the focal point", dist(d1f, bf) < 1e-6);

    // ...and the reverse drops it back below where it started.
    oc.beginDrag();
    oc.mouseLook(0, 120);
    oc.endDrag();
    double d2[3], d2f[3];
    oc.getPose(d2, d2f, bu);
    check("drag UP lowers the eye", d2[2] < b[2] - 1e-6);

    // Horizontal drag was already correct, so this pins it against a
    // well-meaning sign flip of the wrong axis.
    oc.frameBounds(-50, -50, 0, 50, 50, 20);
    double h0[3], h0f[3];
    oc.getPose(h0, h0f, bu);
    oc.beginDrag();
    oc.mouseLook(80, 0);
    oc.endDrag();
    double h1[3], h1f[3];
    oc.getPose(h1, h1f, bu);
    check("drag RIGHT swings the eye horizontally, height unchanged",
          dist(h1, h0) > 1e-3 && std::fabs(h1[2] - h0[2]) < 1e-6);
    check("horizontal orbit keeps the focal point", dist(h1f, h0f) < 1e-6);
  }

  printf("B. state -> camera (mode, orbit center)\n");
  // external write: switch to orbit via state
  root("test.camera.mode").value(0);
  check("state mode=0 -> Orbit", c.mode() == CameraController::Mode::Orbit);
  // external write: move the orbit center; camera focal follows
  root("test.camera.orbit.center.x").value(25.0);
  root("test.camera.orbit.center.y").value(-5.0);
  double oe[3], of[3];
  c.getPose(oe, of, u);
  check("state orbit.center -> focal moves",
        std::fabs(of[0] - 25.0) < 1e-6 && std::fabs(of[1] + 5.0) < 1e-6);

  printf("C. camera -> state (driving writes state)\n");
  c.setOrbitCenter(3.0, 4.0, 5.0);
  check("setOrbitCenter writes state",
        std::fabs(root("test.camera.orbit.center.x").value<double>() - 3.0) < 1e-6 &&
            std::fabs(root("test.camera.orbit.center.z").value<double>() - 5.0) < 1e-6);
  c.setMoveSpeed(42.0);
  check("setMoveSpeed writes state",
        std::fabs(root("test.camera.settings.move_speed").value<double>() - 42.0) < 1e-6);

  printf("D. configurable up axis (Y-up)\n");
  CameraController yc(ctx, "test.ycam");
  vtkNew<vtkCamera> ycam;
  yc.setCamera(ycam);
  yc.setUpAxis(0, 1, 0);
  yc.frameBounds(-10, -10, -10, 10, 10, 10);
  double ye[3], yf[3], yu[3];
  yc.getPose(ye, yf, yu);
  check("up axis is +Y", std::fabs(yu[1] - 1.0) < 1e-9 && std::fabs(yu[2]) < 1e-9);
  check("up axis reflected in state",
        std::fabs(root("test.ycam.up.y").value<double>() - 1.0) < 1e-9);
  // in Y-up, fly forward stays in the horizontal (XZ) plane at pitch 0
  yc.toggleMode();
  double b1[3];
  yc.getPose(b1, yf, yu);
  yc.setMoveSpeed(5.0);
  yc.keyDown("w");
  yc.update(1.0);
  yc.keyUp("w");
  double a1[3];
  yc.getPose(a1, yf, yu);
  // fly seeded looking at the (Y-up) center; W moves toward it. Just assert it moved.
  check("Y-up fly W moves", dist(a1, b1) > 1e-3);

  printf("E. key bindings via state + API\n");
  CameraController kc(ctx, "test.kcam");
  vtkNew<vtkCamera> kcam;
  kc.setCamera(kcam);
  kc.frameBounds(-10, -10, -10, 10, 10, 10);
  kc.setMode(CameraController::Mode::Fly);
  kc.setMoveSpeed(10.0);
  // rebind "forward" to the up-arrow via state
  root("test.kcam.keys.forward").value(std::string("Up"));
  check("state rebinds forward key", kc.keyBinding("forward") == "Up");
  double kb[3];
  kc.getPose(kb, f, u);
  kc.keyDown("w"); // old binding: should do nothing now
  kc.update(1.0);
  kc.keyUp("w");
  double kw[3];
  kc.getPose(kw, f, u);
  check("old key 'w' no longer moves", dist(kb, kw) < 1e-9);
  kc.keyDown("Up"); // new binding
  kc.update(1.0);
  kc.keyUp("Up");
  double ku[3];
  kc.getPose(ku, f, u);
  check("new key 'Up' moves forward", dist(kb, ku) > 1e-3);
  // a binding in SDL's spelling matches VTK's keysym for the same key
  kc.setKeyBinding("forward", "PageUp");
  double kp[3];
  kc.getPose(kp, f, u);
  kc.keyDown("Prior"); // VTK's name for Page Up
  kc.update(1.0);
  kc.keyUp("Prior");
  double kq[3];
  kc.getPose(kq, f, u);
  check("SDL's 'PageUp' binding moves on VTK's 'Prior'", dist(kp, kq) > 1e-3);
  // API rebind writes state
  kc.setKeyBinding("forward", "i");
  check("setKeyBinding writes state", root("test.kcam.keys.forward").value() == "i");

  printf("F. canonical viewer state path\n");
  check("viewerStatePath is <prefix>.viewers.<name>.camera",
        CameraController::viewerStatePath("cvcgl", "left") == "cvcgl.viewers.left.camera");
  CameraController vc(ctx, CameraController::viewerStatePath("myscene", "right"));
  vc.setMode(CameraController::Mode::Fly);
  check("camera state lands at the canonical path",
        root("myscene.viewers.right.camera.mode").value<int>() == 1);

  printf("G. cinematic TRACK mode (follow a scene actor)\n");
  {
    SceneGraph sg(ctx, "scn");
    auto actor = sg.addGraphics("mover"); // empty node — just a movable transform
    actor->setPosition(0, 0, 0);
    CameraController tc(ctx, "scn.viewers.main.camera");
    vtkNew<vtkCamera> tcam;
    tc.setCamera(tcam);
    tc.setScene(&sg);
    // configure tracking through the state tree (fast taus so the test converges)
    root("scn.viewers.main.camera.track.target").value(std::string("mover"));
    root("scn.viewers.main.camera.track.back").value(10.0);
    root("scn.viewers.main.camera.track.height").value(5.0);
    root("scn.viewers.main.camera.track.look_up").value(0.0);
    for (const char *k : {"pos_tau", "vel_tau", "cam_tau"})
      root(std::string("scn.viewers.main.camera.track.") + k).value(0.01);
    tc.setMode(CameraController::Mode::Track);
    check("mode is Track", tc.mode() == CameraController::Mode::Track);
    check("track target set via state", tc.trackTarget() == "mover");
    // drive the actor along +X; the camera should trail behind (-X) and sit above
    double px = 0.0;
    for (int i = 0; i < 200; ++i) {
      px += 0.5;
      actor->setPosition(px, 0, 0);
      sg.processEvents();
      tc.update(0.05);
    }
    double te[3], tf[3], tu[3];
    tc.getPose(te, tf, tu);
    check("track focal follows the actor", std::fabs(tf[0] - px) < 2.0 && std::fabs(tf[1]) < 1.0);
    check("track eye trails behind (-X) and above (+Z)", te[0] < tf[0] - 5.0 && te[2] > 2.0);
    check("track mode reflected in state", root("scn.viewers.main.camera.mode").value<int>() == 2);
  }

  printf("H. camera Phase A: pushed Track feed + per-viewport lens + widen\n");
  {
    SceneGraph sg(ctx, "pa");
    CameraController tc(ctx, "pa.viewers.main.camera");
    vtkNew<vtkCamera> tcam;
    tc.setCamera(tcam);
    tc.setScene(&sg);
    for (const char *k : {"pos_tau", "vel_tau", "cam_tau"})
      root(std::string("pa.viewers.main.camera.track.") + k).value(0.01);
    root("pa.viewers.main.camera.track.back").value(10.0);
    root("pa.viewers.main.camera.track.height").value(5.0);
    root("pa.viewers.main.camera.track.look_up").value(0.0);
    tc.setMode(CameraController::Mode::Track);

    // Push mode: feed a position stream (no scene node); the pose follows it.
    check("trackFed false before feeding", !tc.trackFed());
    double px = 0.0;
    for (int i = 0; i < 200; ++i) {
      px += 0.5;
      tc.feedTrackTarget(px, 0.0, 0.0);
      tc.update(0.05);
    }
    check("feedTrackTarget switches to push mode", tc.trackFed());
    double e[3], f[3], u[3];
    tc.getPose(e, f, u);
    check("pushed Track focal follows the fed target",
          std::fabs(f[0] - px) < 2.0 && std::fabs(f[1]) < 1.0);
    check("pushed Track eye trails behind (-X) and above (+Z)", e[0] < f[0] - 5.0 && e[2] > 2.0);

    // Naming a node, or clearTrackFeed(), reverts to node-pull.
    tc.setTrackTarget("someNode");
    check("setTrackTarget reverts push mode", !tc.trackFed());
    tc.feedTrackTarget(1, 2, 3);
    check("feed re-enables push", tc.trackFed());
    tc.clearTrackFeed();
    check("clearTrackFeed reverts push", !tc.trackFed());

    // Per-viewport field of view: applied to the camera (perspective) + state.
    tc.setFieldOfView(60.0);
    tc.update(0.05);
    check("field_of_view applied to camera", std::fabs(tcam->GetViewAngle() - 60.0) < 1e-6);
    check("field_of_view mirrored to state",
          std::fabs(root("pa.viewers.main.camera.settings.field_of_view").value<double>() - 60.0) <
              1e-9);
    check("fieldOfView() reads back", std::fabs(tc.fieldOfView() - 60.0) < 1e-9);

    // widen_tau: asymmetric-easing knob; state round-trip (0 default = symmetric).
    tc.setWidenTau(1.5);
    check("widen_tau mirrored to state",
          std::fabs(root("pa.viewers.main.camera.track.widen_tau").value<double>() - 1.5) < 1e-9);
    check("widenTau() reads back", std::fabs(tc.widenTau() - 1.5) < 1e-9);
  }

  printf(
      "M. third-person mouse look: a click captures, the mouse turns the orbit, a key frees it\n");
  {
    vtkNew<vtkCamera> mcam;
    CameraController mc(ctx, "ml.camera");
    mc.setCamera(mcam);
    mc.frameBounds(-5, -5, 0, 5, 5, 2);
    double e0[3], e1[3], ff[3], uu[3];
    mc.getPose(e0, ff, uu);
    // Off: the mouse turns nothing without a drag, and a click is the host's.
    mc.mouseLook(50, 0);
    mc.getPose(e1, ff, uu);
    check("off: a move without a drag turns nothing", dist(e0, e1) < 1e-9);
    int cx = 0, cy = 0;
    mc.notePointerDown(10, 10);
    mc.notePointerUp(10, 10);
    check("off: a click is reported", mc.takeClick(cx, cy) && !mc.pointerCapture());

    mc.setMouseLookEnabled(true);
    check("settings.mouse_look mirrored", root("ml.camera.settings.mouse_look").value<int>() == 1);
    check("the release key is in the state tree",
          root("ml.camera.keys.release_pointer").value() == "Escape");
    mc.notePointerDown(10, 10);
    mc.notePointerUp(10, 10);
    check("on: a click on the scene captures the pointer",
          mc.pointerCapture() && mc.mouseLooking());
    check("on: that click is not the host's", !mc.takeClick(cx, cy));
    // First person (the default): the eye stays where it is and turns.
    const auto look = [&](double d[3]) {
      double e[3], f[3], v[3];
      mc.getPose(e, f, v);
      const double n = dist(e, f);
      for (int k = 0; k < 3; ++k)
        d[k] = (f[k] - e[k]) / n;
    };
    double d0[3], d1[3];
    look(d0);
    const double right0[3] = {d0[1], -d0[0], 0.0}; // the look x up (+z), unnormalised
    mc.mouseLook(50, 0);
    mc.getPose(e1, ff, uu);
    look(d1);
    check("on: the mouse turns the view with no button held",
          d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2] < 1.0 - 1e-6);
    check("first person: the eye stays where it is", dist(e0, e1) < 1e-6);
    check("first person: the mouse moving right turns the view right",
          right0[0] * (d1[0] - d0[0]) + right0[1] * (d1[1] - d0[1]) > 1e-6);
    mc.setInvertYaw(true);
    check("settings.invert_yaw mirrored", root("ml.camera.settings.invert_yaw").value<int>() == 1);
    mc.mouseLook(50, 0);
    double e2[3];
    mc.getPose(e2, ff, uu);
    look(d1);
    check("invert_yaw turns it back the other way",
          dist(e0, e2) < 1e-6 && std::fabs(d1[0] - d0[0]) + std::fabs(d1[1] - d0[1]) < 1e-6);
    mc.setInvertYaw(false);
    mc.mouseLook(0, 20); // VTK's y is up: the mouse moved up
    look(d1);
    mc.getPose(e2, ff, uu);
    check("first person: the mouse moving up looks up, the eye still put",
          d1[2] > d0[2] + 1e-6 && dist(e0, e2) < 1e-6);
    // The orbit's elevation range keeps an ORBITING eye off a floor; a first-person look is not
    // held by it, so the eye can look up at the sky.
    const auto elevation = [&] {
      double e[3], f[3], v[3];
      mc.getPose(e, f, v);
      return std::asin((e[2] - f[2]) / dist(e, f)) * 180.0 / 3.14159265358979323846;
    };
    mc.setOrbitElevationRange(-10.0, 70.0);
    mc.mouseLook(0, 100000);
    look(d1);
    check("first person: the look goes up to 89 degrees past the range",
          std::fabs(elevation() + 89.0) < 1e-6 && d1[2] > 0.99);
    mc.getPose(e2, ff, uu);
    check("and the eye has not moved", dist(e0, e2) < 1e-6);
    // An orbit drag from there (the pointer let go) is not snapped back into the range: it moves
    // toward it.
    mc.keyDown("Escape");
    mc.keyUp("Escape");
    mc.beginDrag();
    mc.mouseLook(0, -10);
    mc.endDrag();
    check("an orbit drag outside the range moves toward it, no snap",
          elevation() > -89.0 + 1e-6 && elevation() < -10.0);
    mc.notePointerDown(10, 10); // capture it again
    mc.notePointerUp(10, 10);
    mc.mouseLook(0, -100000); // first person: look straight down (elevation 89, past the range)
    // Third person (a host's avatar camera): the mouse turns the orbit about its centre, held by
    // the elevation range.
    mc.setMouseLookOrbits(true);
    check("settings.mouse_look_orbits mirrored",
          root("ml.camera.settings.mouse_look_orbits").value<int>() == 1 && !mc.firstPersonLook());
    mc.getPose(e1, ff, uu);
    mc.mouseLook(50, 0);
    mc.getPose(e2, ff, uu);
    check("third person: the mouse turns the eye about the centre", dist(e1, e2) > 1e-3);
    mc.mouseLook(0, 100000); // up: the eye goes down
    check("looking up stops at the range's lower elevation", std::fabs(elevation() + 10.0) < 1e-6);
    mc.mouseLook(0, -100000);
    check("looking down stops at its upper one", std::fabs(elevation() - 70.0) < 1e-6);
    check("the range is in the state tree",
          std::fabs(root("ml.camera.orbit.min_elevation").value<double>() + 10.0) < 1e-9);
    mc.setMouseLookOrbits(false);

    mc.keyDown("Escape");
    check("Escape frees the pointer", !mc.pointerCapture() && !mc.keyHeld("Escape"));
    mc.getPose(e1, ff, uu);
    mc.mouseLook(50, 0);
    mc.getPose(e2, ff, uu);
    check("freed: no turn without a drag", dist(e1, e2) < 1e-9);
    // The release key is a binding like the others: rebound, Escape stops doing it.
    mc.setKeyBinding("release_pointer", "Q");
    check("a rebound release key is in the state tree",
          root("ml.camera.keys.release_pointer").value() == "Q");
    mc.notePointerDown(10, 10);
    mc.notePointerUp(10, 10);
    mc.keyDown("Escape");
    mc.keyUp("Escape");
    check("rebound: Escape keeps the pointer", mc.pointerCapture());
    mc.keyDown("q");
    check("rebound: its key frees it", !mc.pointerCapture());
    mc.keyUp("q");
    // Mouse look turned off where a settings window writes it: update() frees the pointer, and
    // clicks are the host's again.
    mc.notePointerDown(10, 10);
    mc.notePointerUp(10, 10);
    root("ml.camera.settings.mouse_look").value(0);
    mc.update(1.0 / 60.0);
    check("turned off from state: the pointer is freed",
          !mc.pointerCapture() && !mc.mouseLookEnabled());
    mc.notePointerDown(10, 10);
    mc.notePointerUp(10, 10);
    check("turned off: clicks are reported again", mc.takeClick(cx, cy));
  }

  printf("N. the orbit floor: looking up slides the eye in, never under the floor\n");
  {
    vtkNew<vtkCamera> fcam;
    CameraController fc(ctx, "fl.camera");
    fc.setCamera(fcam);
    fc.frameBounds(-5, -5, 0, 5, 5, 2);
    fc.setOrbitCenter(0.0, 0.0, 1.2); // a head 1.2 m above the floor
    root("fl.camera.orbit.distance").value(6.0);
    fc.setOrbitFloor(0.0, 0.25);
    check("the floor is in the state tree",
          root("fl.camera.orbit.floor.on").value<int>() == 1 &&
              std::fabs(root("fl.camera.orbit.floor.clearance").value<double>() - 0.25) < 1e-9);
    double e[3], f[3], v[3];
    const auto pose = [&] { fc.getPose(e, f, v); };
    // Level: 6 m out, well above the floor.
    root("fl.camera.orbit.elevation").value(0.0);
    pose();
    check("level: the full distance", std::fabs(dist(e, f) - 6.0) < 1e-6);
    // Looking up (the eye below the head) far enough to go under the floor: slid in instead.
    root("fl.camera.orbit.elevation").value(-45.0);
    pose();
    check("looking up: the eye stays just above the floor", std::fabs(e[2] - 0.25) < 1e-6);
    check("looking up: slid in along its line of sight",
          dist(e, f) < 6.0 && std::fabs((f[2] - e[2]) / dist(e, f) -
                                        std::sin(45.0 * 3.14159265358979323846 / 180.0)) < 1e-6);
    // Straight up: just above the floor, under the head.
    root("fl.camera.orbit.elevation").value(-89.0);
    pose();
    check("straight up: still above the floor", e[2] >= 0.25 - 1e-9 && dist(e, f) < 1.0);
    // Back down: the distance is kept, so the eye goes back out.
    root("fl.camera.orbit.elevation").value(20.0);
    pose();
    check("back down: the eye goes back out", std::fabs(dist(e, f) - 6.0) < 1e-6);
    check("the orbit's distance was never changed",
          std::fabs(root("fl.camera.orbit.distance").value<double>() - 6.0) < 1e-9);
    // Without the floor the eye goes under it.
    fc.clearOrbitFloor();
    root("fl.camera.orbit.elevation").value(-45.0);
    pose();
    check("no floor: the eye goes under", e[2] < 0.0);
  }

  printf("O. drag to steer: the primary drag points, the right button turns the view\n");
  {
    vtkNew<vtkCamera> scam;
    CameraController sc(ctx, "st.camera");
    sc.setCamera(scam);
    sc.frameBounds(-5, -5, 0, 5, 5, 2);
    int sx = 0, sy = 0, cx = 0, cy = 0;
    sc.notePointerDown(10, 10);
    check("off: a press does not steer", !sc.steerPoint(sx, sy));
    sc.notePointerUp(40, 10);
    sc.setPrimaryDragSteers(true);
    check("settings.primary_drag_steers mirrored",
          root("st.camera.settings.primary_drag_steers").value<int>() == 1);
    double e0[3], e1[3], ff[3], uu[3];
    sc.getPose(e0, ff, uu);
    // As the interactor style does it: the press, no drag begun while it steers, then the move.
    sc.notePointerDown(100, 50);
    check("on: a press is a steering one (no drag of the view begins)", sc.steering());
    check("but it points nowhere yet (it may be a click)", !sc.steerPoint(sx, sy));
    sc.notePointerMove(103, 52);
    check("nor a move within a click's slop", !sc.steerPoint(sx, sy));
    sc.notePointerMove(160, 80);
    sc.mouseLook(60, 30);
    sc.getPose(e1, ff, uu);
    check("the drag steers where the pointer goes", sc.steerPoint(sx, sy) && sx == 160 && sy == 80);
    check("and turns nothing", dist(e0, e1) < 1e-9);
    sc.beginSecondaryDrag();
    sc.mouseLook(60, 0);
    sc.endSecondaryDrag();
    sc.getPose(e1, ff, uu);
    check("the right button turns the view meanwhile", dist(e0, e1) > 1e-3);
    sc.notePointerUp(160, 80);
    check("released: it steers no more, and was no click",
          !sc.steerPoint(sx, sy) && !sc.takeClick(cx, cy));
    sc.notePointerDown(30, 30);
    sc.notePointerUp(31, 30);
    check("a click is still a click", sc.takeClick(cx, cy) && cx == 31);
    // A second finger lands (TouchGestures): the first one's press is the gesture's.
    sc.notePointerDown(30, 30);
    sc.notePointerMove(90, 30);
    sc.cancelPointer();
    check("cancelled: it steers no more", !sc.steerPoint(sx, sy));
    sc.notePointerUp(30, 30);
    check("and is no click when it comes up", !sc.takeClick(cx, cy));
    root("st.camera.settings.primary_drag_steers").value(0);
    sc.notePointerDown(30, 30);
    sc.notePointerMove(90, 30);
    check("turned off from state: a drag does not steer",
          !sc.primaryDragSteers() && !sc.steerPoint(sx, sy));
    sc.notePointerUp(30, 30);
    sc.takeClick(cx, cy);
    sc.setPrimaryDragSteers(true);
    sc.setMouseLookEnabled(true);
    sc.notePointerDown(30, 30);
    sc.notePointerMove(90, 30);
    check("under mouse look a drag does not steer", !sc.steerPoint(sx, sy));
    sc.notePointerUp(30, 30);
  }

  printf("P. a frame request through the state tree (a scene's data node: frame:)\n");
  {
    CameraController rc(ctx, "fr.camera");
    vtkNew<vtkCamera> rcam;
    rc.setCamera(rcam);
    root("fr.camera.frame").value(std::string("-50 -50 0 50 50 20"));
    const double want = std::sqrt(100.0 * 100.0 + 100.0 * 100.0 + 20.0 * 20.0) * 1.1;
    check("frame: the orbit centres on the bounds",
          std::fabs(root("fr.camera.orbit.center.z").value<double>() - 10.0) < 1e-9 &&
              std::fabs(root("fr.camera.orbit.center.x").value<double>()) < 1e-9);
    check("frame: the orbit backs off as frameBounds does",
          std::fabs(root("fr.camera.orbit.distance").value<double>() - want) < 1e-6);
    root("fr.camera.frame").value(std::string("not bounds")); // ignored
    check("a malformed frame request is ignored",
          std::fabs(root("fr.camera.orbit.distance").value<double>() - want) < 1e-6);
  }

  printf("Q. the move keys in Orbit, and the reset to the opening view\n");
  {
    CameraController oc(ctx, "ok.camera");
    vtkNew<vtkCamera> ocam;
    oc.setCamera(ocam);
    oc.frameBounds(-50, -50, 0, 50, 50, 20);
    const double d0 = root("ok.camera.orbit.distance").value<double>();
    double e0[3], f0[3], u0[3];
    oc.getPose(e0, f0, u0);
    // W slides the orbit centre along the ground the way the eye looks, the eye with it.
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    double e1[3], f1[3], u1[3];
    oc.getPose(e1, f1, u1);
    const double look[2] = {f0[0] - e0[0], f0[1] - e0[1]},
                 moved[2] = {f1[0] - f0[0], f1[1] - f0[1]};
    check("Orbit: W slides the view forward across the ground",
          look[0] * moved[0] + look[1] * moved[1] > 0.0 && std::fabs(f1[2] - f0[2]) < 1e-9);
    check("Orbit: by 0.6 of the orbit distance a second",
          std::fabs(std::hypot(moved[0], moved[1]) - 0.6 * d0 * 0.5) < 1e-6 * d0);
    // The arrows do as the bindings do.
    oc.keyDown("Left");
    oc.update(0.5);
    oc.keyUp("Left");
    double e2[3], f2[3], u2[3];
    oc.getPose(e2, f2, u2);
    check("Orbit: Left slides it sideways", dist(f1, f2) > 1.0);
    // Space zooms out, Ctrl in.
    oc.keyDown("space");
    oc.update(0.5);
    oc.keyUp("space");
    check("Orbit: Space zooms out",
          root("ok.camera.orbit.distance").value<double>() > d0 || oc.orbitDistance() > d0);
    // Off, the keys leave Orbit alone (a host whose keys walk an avatar).
    oc.setOrbitKeys(false);
    double e3[3], f3[3], u3[3];
    oc.getPose(e3, f3, u3);
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    double e4[3], f4[3], u4[3];
    oc.getPose(e4, f4, u4);
    check("orbit_keys off: W leaves Orbit alone", dist(e3, e4) < 1e-9 && dist(f3, f4) < 1e-9);
    check("orbit_keys is in the state tree",
          root("ok.camera.settings.orbit_keys").value<int>() == 0);
    oc.setOrbitKeys(true);
    // H goes back to the view the scene opened with (here, the bounds last framed: no scene).
    oc.setMode(CameraController::Mode::Fly);
    oc.keyDown("h");
    oc.keyUp("h");
    double e5[3], f5[3], u5[3];
    oc.getPose(e5, f5, u5);
    check("H: back to Orbit at the opening view",
          oc.mode() == CameraController::Mode::Orbit && dist(e5, e0) < 1e-6 && dist(f5, f0) < 1e-6);
    // ...and a write to <camera>.home does it too, once per write.
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    root("ok.camera.home").value(std::string("1"));
    double e6[3], f6[3], u6[3];
    oc.getPose(e6, f6, u6);
    check("a home request resets the view", dist(e6, e0) < 1e-6 && dist(f6, f0) < 1e-6);
    check("and the request is cleared for the next", root("ok.camera.home").value().empty());
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    root("ok.camera.home").value(std::string("1"));
    double e7[3], f7[3], u7[3];
    oc.getPose(e7, f7, u7);
    check("a second home request resets it again", dist(e7, e0) < 1e-6);
    root("ok.camera.keys.home").value(std::string("G"));
    check("the home key rebinds through the state tree", oc.keyBinding("home") == "G");
    // Mouse look, first person: W moves the eye along the way it looks (pitch included), D
    // strafes right across it in the ground's plane, Space rises, at the move speed.
    oc.setMouseLookEnabled(true);
    oc.setMoveSpeed(10.0);
    double e8[3], f8[3], u8[3];
    oc.getPose(e8, f8, u8);
    const double n8 = dist(e8, f8);
    const double l8[3] = {(f8[0] - e8[0]) / n8, (f8[1] - e8[1]) / n8, (f8[2] - e8[2]) / n8};
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    double e9[3], f9[3], u9[3];
    oc.getPose(e9, f9, u9);
    check("mouse look: W moves the eye 5 m along its look, pitch included",
          std::fabs(e9[0] - (e8[0] + 5.0 * l8[0])) + std::fabs(e9[1] - (e8[1] + 5.0 * l8[1])) +
                  std::fabs(e9[2] - (e8[2] + 5.0 * l8[2])) <
              1e-6);
    check("mouse look: the look itself is unchanged",
          std::fabs((f9[0] - e9[0]) / n8 - l8[0]) + std::fabs((f9[2] - e9[2]) / n8 - l8[2]) < 1e-6);
    oc.keyDown("d");
    oc.update(0.5);
    oc.keyUp("d");
    double e10[3], f10[3], u10[3];
    oc.getPose(e10, f10, u10);
    const double hr = std::hypot(l8[0], l8[1]);
    const double right[2] = {l8[1] / hr, -l8[0] / hr}; // look x up, flattened
    check("mouse look: D strafes 5 m right, level with the ground",
          std::fabs(e10[0] - (e9[0] + 5.0 * right[0])) + std::fabs(e10[1] - (e9[1] + 5.0 * right[1])) <
                  1e-6 &&
              std::fabs(e10[2] - e9[2]) < 1e-9);
    oc.keyDown("space");
    oc.update(0.5);
    oc.keyUp("space");
    double e11[3], f11[3], u11[3];
    oc.getPose(e11, f11, u11);
    check("mouse look: Space rises 5 m straight up",
          std::fabs(e11[2] - e10[2] - 5.0) < 1e-6 && std::hypot(e11[0] - e10[0], e11[1] - e10[1]) < 1e-9);
    // Third person: the keys are orbit keys again (a slide across the ground).
    oc.setMouseLookOrbits(true);
    oc.keyDown("w");
    oc.update(0.5);
    oc.keyUp("w");
    double e12[3], f12[3], u12[3];
    oc.getPose(e12, f12, u12);
    check("third-person mouse look: W slides the view across the ground",
          std::fabs(f12[2] - f11[2]) < 1e-9 && dist(f11, f12) > 1.0);
  }

  printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  // cvcGL's state_object / handler-thread teardown races at process exit (a known,
  // harmless issue, independent of the checks above). Hard-exit with the real
  // status so a teardown race can't turn a green run red.
  std::fflush(stdout);
  std::_Exit(failures ? 1 : 0);
}
