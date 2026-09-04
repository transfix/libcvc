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
#include <cvc/core/state.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
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

  printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  // cvcGL's state_object / handler-thread teardown races at process exit (a known,
  // harmless issue, independent of the checks above). Hard-exit with the real
  // status so a teardown race can't turn a green run red.
  std::fflush(stdout);
  std::_Exit(failures ? 1 : 0);
}
