// cvc::gl::CameraController — orbit + Quake-fly navigation AND full cvc::state
// configurability. Drives a bare vtkCamera (no window) and asserts:
//   * framing / Z-up, seamless orbit<->fly toggle, WASD/strafe/vertical, look,
//     orbit-drag  (the navigation math);
//   * two-way cvc::state binding: writing state changes the camera (mode, up
//     axis, orbit center, key bindings), and driving the camera writes state;
//   * a configurable up axis (Y-up), and the canonical viewer state path.
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/CameraController.h>

#include <cmath>
#include <cstdio>
#include <string>

#include <vtkCamera.h>
#include <vtkNew.h>

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
  printf("orbit-framed eye=(%.1f,%.1f,%.1f) focal=(%.1f,%.1f,%.1f) up=(%.2f,%.2f,%.2f)\n", e[0], e[1],
         e[2], f[0], f[1], f[2], u[0], u[1], u[2]);

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
  check("D strafes 10 horizontally", std::fabs(dist(aD, bD) - 10.0) < 1e-6 &&
                                         std::fabs(aD[2] - bD[2]) < 1e-9);
  double bl[3], blf[3];
  c.getPose(bl, blf, u);
  c.mouseLook(100, 0);
  double al[3], alf[3];
  c.getPose(al, alf, u);
  check("look rotates view, not eye", dist(bl, al) < 1e-9 && dist(blf, alf) > 1e-3);

  printf("B. state -> camera (mode, orbit center)\n");
  // external write: switch to orbit via state
  root("test.camera.mode").value(0);
  check("state mode=0 -> Orbit", c.mode() == CameraController::Mode::Orbit);
  // external write: move the orbit center; camera focal follows
  root("test.camera.orbit.center.x").value(25.0);
  root("test.camera.orbit.center.y").value(-5.0);
  double oe[3], of[3];
  c.getPose(oe, of, u);
  check("state orbit.center -> focal moves", std::fabs(of[0] - 25.0) < 1e-6 &&
                                                 std::fabs(of[1] + 5.0) < 1e-6);

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

  printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
