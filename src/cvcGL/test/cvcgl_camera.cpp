// Standalone validation of cvc::gl::CameraController math (no renderer/window;
// drives a bare vtkCamera and checks the pose).
#include <cvc/gl/CameraController.h>

#include <cmath>
#include <cstdio>

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
static void pose(CameraController &c, double e[3], double f[3], double u[3]) {
  c.getPose(e, f, u);
}

int main() {
  vtkNew<vtkCamera> cam;
  CameraController c;
  c.setCamera(cam);

  // Frame a 100x100x20 island centered at origin, ground at z=0.
  c.frameBounds(-50, -50, 0, 50, 50, 20);
  double e[3], f[3], u[3];
  pose(c, e, f, u);
  printf("orbit-framed eye=(%.1f,%.1f,%.1f) focal=(%.1f,%.1f,%.1f) up=(%.2f,%.2f,%.2f)\n",
         e[0], e[1], e[2], f[0], f[1], f[2], u[0], u[1], u[2]);

  printf("A. Z-up + orbit framing\n");
  check("world up is +Z", std::fabs(u[2] - 1.0) < 1e-9 && std::fabs(u[0]) < 1e-9);
  check("focal is scene center", std::fabs(f[0]) < 1e-6 && std::fabs(f[1]) < 1e-6 &&
                                     std::fabs(f[2] - 10.0) < 1e-6);
  check("eye above ground (elevated view)", e[2] > f[2]);
  double camPos[3];
  cam->GetPosition(camPos);
  check("camera actually driven", dist(camPos, e) < 1e-6);

  printf("B. seamless orbit->fly toggle\n");
  double orbEye[3], orbFoc[3];
  pose(c, orbEye, orbFoc, u);
  c.toggleMode();
  double flyEye[3], flyFoc[3];
  pose(c, flyEye, flyFoc, u);
  check("fly stands at the orbit eye", dist(flyEye, orbEye) < 1e-6);
  // fly focal is eye + unit look dir toward the old center → same direction
  double vOrb[3] = {orbFoc[0] - orbEye[0], orbFoc[1] - orbEye[1], orbFoc[2] - orbEye[2]};
  double lOrb = std::sqrt(vOrb[0] * vOrb[0] + vOrb[1] * vOrb[1] + vOrb[2] * vOrb[2]);
  double vFly[3] = {flyFoc[0] - flyEye[0], flyFoc[1] - flyEye[1], flyFoc[2] - flyEye[2]};
  double cosang = (vOrb[0] * vFly[0] + vOrb[1] * vFly[1] + vOrb[2] * vFly[2]) / lOrb; // vFly is unit
  check("fly looks the same direction as orbit did", cosang > 0.9999);

  printf("C. fly WASD (Z-up)\n");
  c.setMoveSpeed(10.0);
  // W for 1s → move forward toward the look direction (distance to center shrinks)
  double d0 = dist(flyEye, orbFoc);
  c.keyDown("w");
  c.update(1.0);
  c.keyUp("w");
  double afterW[3];
  pose(c, afterW, f, u);
  check("W moves ~10 units along look", std::fabs(dist(afterW, flyEye) - 10.0) < 1e-6);
  check("W moves toward the center", dist(afterW, orbFoc) < d0);

  // D (strafe right) for 1s → horizontal move, z unchanged
  double beforeD[3];
  pose(c, beforeD, f, u);
  c.keyDown("d");
  c.update(1.0);
  c.keyUp("d");
  double afterD[3];
  pose(c, afterD, f, u);
  check("D strafes ~10 units", std::fabs(dist(afterD, beforeD) - 10.0) < 1e-6);
  check("D strafe is horizontal (z fixed)", std::fabs(afterD[2] - beforeD[2]) < 1e-9);

  // Space (world up) for 1s → +Z only
  double beforeU[3];
  pose(c, beforeU, f, u);
  c.keyDown("space");
  c.update(1.0);
  c.keyUp("space");
  double afterUp[3];
  pose(c, afterUp, f, u);
  check("Space rises +10 in Z only", std::fabs(afterUp[2] - beforeU[2] - 10.0) < 1e-6 &&
                                         std::fabs(afterUp[0] - beforeU[0]) < 1e-9);

  printf("D. mouse-look changes view direction\n");
  double bl[3], blf[3];
  pose(c, bl, blf, u);
  c.mouseLook(100, 0); // yaw
  double al[3], alf[3];
  pose(c, al, alf, u);
  check("look does not move eye", dist(bl, al) < 1e-9);
  check("look rotates the focal", dist(blf, alf) > 1e-3);

  printf("E. orbit drag rotates around center at fixed radius\n");
  c.toggleMode(); // back to orbit
  double oe[3], of[3];
  pose(c, oe, of, u);
  double r0 = dist(oe, of);
  c.beginDrag();
  c.mouseLook(80, 0);
  c.endDrag();
  double oe2[3], of2[3];
  pose(c, oe2, of2, u);
  check("orbit keeps the center fixed", dist(of, of2) < 1e-6);
  check("orbit keeps radius constant", std::fabs(dist(oe2, of2) - r0) < 1e-6);
  check("orbit actually rotated the eye", dist(oe, oe2) > 1e-3);

  printf("\n%s (%d failures)\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
