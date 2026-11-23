// CameraController's authored viewpoint and lens: lookAt(eye, target), setOrthographic and
// setClipRange (docs/ARIADNE.md §5.18, Cameras, builds a scene camera's `position:`, `target:`,
// `orthographic:`, `near:` and `far:` on them).
//
// Pins:
//   A. lookAt in Orbit puts the vtkCamera exactly at the eye looking at the target, as the orbit's
//      own pose (centre, distance, azimuth, elevation in state), so the wheel goes on from it;
//   B. lookAt in Fly stands at the eye facing the target; an eye on the target moves only the
//      centre; a straight-down look keeps the orbit's elevation limit and the target;
//   C. orthographic: parallel projection in Orbit, half as tall as tan(fov / 2) times the
//      distance, so the wheel zooms it; Map takes over and gives it back; off restores
//      perspective; "settings.orthographic" round-trips through state;
//   D. a fixed clipping range holds over updates, and 0, 0 (the default) fits it again;
//   E. a new controller is as before: perspective, the range fitted.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/state/state.h>
#include <string>
#include <vtkCamera.h>
#include <vtkNew.h>

using cvc::gl::CameraController;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

static bool at3(const double a[3], double x, double y, double z, double tol = 1e-9) {
  return std::fabs(a[0] - x) < tol && std::fabs(a[1] - y) < tol && std::fabs(a[2] - z) < tol;
}
static std::string fmt3(const double a[3]) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "(%.6f, %.6f, %.6f)", a[0], a[1], a[2]);
  return buf;
}
static double stateD(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value<double>();
}
static double dist3(const double a[3], const double b[3]) {
  return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) +
                   (a[2] - b[2]) * (a[2] - b[2]));
}

static void lookAt(cvc::app &app) {
  vtkNew<vtkCamera> vc;
  CameraController c(app, "vp");
  c.setZoomSmoothing(0.0);
  c.setCamera(vc);
  c.frameBounds(-5, -5, -5, 5, 5, 5);

  std::printf("A. lookAt in Orbit\n");
  const double eye[3] = {10.0, -4.0, 6.0}, target[3] = {1.0, 2.0, 0.5};
  c.lookAt(eye, target);
  c.update(0.016);
  double p[3], f[3];
  vc->GetPosition(p);
  vc->GetFocalPoint(f);
  chk(at3(p, 10.0, -4.0, 6.0, 1e-9), "the camera is at the eye, got " + fmt3(p));
  chk(at3(f, 1.0, 2.0, 0.5, 1e-9), "looking at the target, got " + fmt3(f));
  chk(c.mode() == CameraController::Mode::Orbit, "still Orbit");
  chk(std::fabs(stateD(app, "vp.orbit.center.x") - 1.0) < 1e-12 &&
          std::fabs(stateD(app, "vp.orbit.center.y") - 2.0) < 1e-12 &&
          std::fabs(stateD(app, "vp.orbit.center.z") - 0.5) < 1e-12,
      "the orbit's centre in state is the target");
  chk(std::fabs(stateD(app, "vp.orbit.distance") - dist3(eye, target)) < 1e-9,
      "its distance the eye's");
  for (int i = 0; i < 10; ++i)
    c.update(0.016);
  vc->GetPosition(p);
  chk(at3(p, 10.0, -4.0, 6.0, 1e-9), "and it holds over updates");
  const double before = c.orbitDistance();
  c.mouseWheel(1.0);
  c.update(0.016);
  vc->GetPosition(p);
  vc->GetFocalPoint(f);
  chk(c.orbitDistance() < before && at3(f, 1.0, 2.0, 0.5, 1e-9),
      "the wheel goes on from it: closer, still looking at the target");
  const double toward[3] = {(p[0] - f[0]) / dist3(p, f), (p[1] - f[1]) / dist3(p, f),
                            (p[2] - f[2]) / dist3(p, f)};
  const double was[3] = {(eye[0] - target[0]) / dist3(eye, target),
                         (eye[1] - target[1]) / dist3(eye, target),
                         (eye[2] - target[2]) / dist3(eye, target)};
  chk(at3(toward, was[0], was[1], was[2], 1e-9), "from the same direction");

  std::printf("B. lookAt in Fly, an eye on the target, straight down\n");
  c.setMode(CameraController::Mode::Fly);
  const double feye[3] = {-3.0, 7.0, 2.0}, ftarget[3] = {4.0, 7.0, 2.0};
  c.lookAt(feye, ftarget);
  c.update(0.016);
  vc->GetPosition(p);
  vc->GetFocalPoint(f);
  const double d = dist3(p, f);
  chk(at3(p, -3.0, 7.0, 2.0, 1e-9), "Fly stands at the eye, got " + fmt3(p));
  chk(d > 0.0 && std::fabs((f[0] - p[0]) / d - 1.0) < 1e-9 && std::fabs(f[1] - p[1]) < 1e-9 &&
          std::fabs(f[2] - p[2]) < 1e-9,
      "facing the target (+x), got focal " + fmt3(f));
  c.setMode(CameraController::Mode::Orbit);
  const double same[3] = {2.0, 2.0, 2.0};
  c.lookAt(eye, target);
  const double dBefore = c.orbitDistance();
  c.lookAt(same, same);
  c.update(0.016);
  vc->GetFocalPoint(f);
  chk(at3(f, 2.0, 2.0, 2.0, 1e-9) && std::fabs(c.orbitDistance() - dBefore) < 1e-9,
      "an eye on the target moves only the centre");
  const double above[3] = {0.0, 0.0, 20.0}, below[3] = {0.0, 0.0, 0.0};
  c.lookAt(above, below);
  c.update(0.016);
  vc->GetFocalPoint(f);
  chk(at3(f, 0.0, 0.0, 0.0, 1e-9) && stateD(app, "vp.orbit.elevation") <= 89.0 + 1e-9,
      "straight down keeps the target and the orbit's 89-degree limit");
}

static void lens(cvc::app &app) {
  vtkNew<vtkCamera> vc;
  CameraController c(app, "lens");
  c.setZoomSmoothing(0.0);
  c.setCamera(vc);
  c.frameBounds(-5, -5, -5, 5, 5, 5);
  c.update(0.016);

  std::printf("E. a new controller is as before\n");
  double n = -1.0, fa = -1.0;
  chk(!c.orthographic() && !vc->GetParallelProjection(), "perspective by default");
  chk(!c.clipRange(n, fa) && n == 0.0 && fa == 0.0, "the clipping range fitted by default");

  std::printf("C. orthographic\n");
  c.setFieldOfView(40.0);
  c.setOrthographic(true);
  c.update(0.016);
  const double want = std::tan(0.5 * 40.0 * 3.14159265358979 / 180.0) * c.orbitDistance();
  chk(vc->GetParallelProjection() != 0, "parallel projection in Orbit");
  chk(std::fabs(vc->GetParallelScale() - want) < 1e-9,
      "half as tall as tan(fov / 2) times the distance: " + std::to_string(vc->GetParallelScale()) +
          " vs " + std::to_string(want));
  chk(stateD(app, "lens.settings.orthographic") == 1.0, "settings.orthographic = 1 in state");
  const double s0 = vc->GetParallelScale();
  c.mouseWheel(1.0);
  c.update(0.016);
  chk(vc->GetParallelScale() < s0, "the wheel zooms it");
  c.setMode(CameraController::Mode::Map);
  c.update(0.016);
  chk(vc->GetParallelProjection() != 0 && c.orthographic(), "Map: parallel, its own scale");
  c.setMode(CameraController::Mode::Orbit);
  c.update(0.016);
  chk(vc->GetParallelProjection() != 0 &&
          std::fabs(vc->GetParallelScale() -
                    std::tan(0.5 * 40.0 * 3.14159265358979 / 180.0) * c.orbitDistance()) < 1e-9,
      "back in Orbit: orthographic again, at the orbit's scale");
  cvc::state::instance(app)("lens.settings.orthographic").value(0);
  c.update(0.016);
  chk(!c.orthographic() && !vc->GetParallelProjection(),
      "settings.orthographic = 0 through state: perspective again");

  std::printf("D. a fixed clipping range\n");
  c.setClipRange(0.5, 200.0);
  c.update(0.016);
  double r[2];
  vc->GetClippingRange(r);
  chk(c.clipRange(n, fa) && n == 0.5 && fa == 200.0, "clipRange reports it");
  chk(std::fabs(r[0] - 0.5) < 1e-12 && std::fabs(r[1] - 200.0) < 1e-12,
      "the camera's range is 0.5 .. 200");
  c.mouseWheel(1.0);
  for (int i = 0; i < 5; ++i)
    c.update(0.016);
  vc->GetClippingRange(r);
  chk(std::fabs(r[0] - 0.5) < 1e-12 && std::fabs(r[1] - 200.0) < 1e-12,
      "and holds as the camera moves");
  chk(stateD(app, "lens.settings.clip_near") == 0.5 &&
          stateD(app, "lens.settings.clip_far") == 200.0,
      "settings.clip_near / clip_far in state");
  c.setClipRange(0.0, 0.0);
  chk(!c.clipRange(n, fa), "0, 0 fits it again");
  c.setClipRange(5.0, 1.0);
  chk(!c.clipRange(n, fa), "so does far <= near");
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  lookAt(app);
  lens(app);
  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
