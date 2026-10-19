// CameraController Map mode holds the MAP pose -- frameMap() is not a one-off
// camera write that the next update() replaces with the orbit pose.
//
// It was: getPose() had no Map branch, so applyToCamera() -- which update() runs
// whenever the pose, the camera or the scene moved -- pushed the ORBIT pose onto
// the map camera. frameMap(100, 200, ...) put the eye at (100, 200, 1000); one
// update() later it sat at the default orbit eye, about (4.3, -7.5, 5.0), still
// in parallel projection. Pans and zooms wrote the vtkCamera directly, so they
// were undone the same way.
//
// Pins:
//   A. frameMap + update(): the eye stays straight above the framed point, north
//      up, parallel projection at the framed scale -- over many updates;
//   B. pan and zoom move the map pose: a drag moves the view against the pointer
//      on both axes by exactly the parallel scale's world-per-pixel, the wheel
//      scales it; both survive update() and are mirrored to state "map.*";
//   C. Orbit -> Map -> Orbit returns to the same orbit pose with perspective
//      restored; Map -> Orbit -> Map to the same map; Fly likewise; Tab cannot
//      leave Map; Track eases in from the orbit view, not from straight down;
//   D. the state tree: an outside write while in Map (a settings slider) keeps
//      Map, "mode" = 3 / 0 enters / leaves it, "map.*" moves it, frameBounds()
//      frames the footprint in Map;
//   E. with a scene, the idle path of update() holds in Map: still frames cost
//      nothing, a pan or zoom applies once, an outside camera move is put back;
//   F. the width fit re-fits the map pose across a resize until the user zooms
//      (skipped when the renderer reports no size).
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/state/state.h>
#include <string>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::SceneGraph;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

static bool at3(const double a[3], double x, double y, double z, double tol = 1e-9) {
  return std::fabs(a[0] - x) < tol && std::fabs(a[1] - y) < tol && std::fabs(a[2] - z) < tol;
}
static bool same3(const double a[3], const double b[3], double tol = 1e-9) {
  return at3(a, b[0], b[1], b[2], tol);
}
static std::string fmt3(const double a[3]) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), "(%.3f, %.3f, %.3f)", a[0], a[1], a[2]);
  return buf;
}

struct Pose {
  double eye[3], focal[3], up[3];
};
static Pose poseOf(const CameraController &c) {
  Pose p;
  c.getPose(p.eye, p.focal, p.up);
  return p;
}
static bool samePose(const Pose &a, const Pose &b) {
  return same3(a.eye, b.eye) && same3(a.focal, b.focal) && same3(a.up, b.up);
}

// Is the vtkCamera showing the map centred on (cx, cy) at parallel scale `scale`?
static bool cameraShowsMap(vtkCamera *cam, double cx, double cy, double scale) {
  double pos[3], foc[3], vu[3];
  cam->GetPosition(pos);
  cam->GetFocalPoint(foc);
  cam->GetViewUp(vu);
  return std::fabs(pos[0] - cx) < 1e-9 && std::fabs(pos[1] - cy) < 1e-9 && pos[2] > 0.0 &&
         at3(foc, cx, cy, 0.0) && at3(vu, 0.0, 1.0, 0.0) && cam->GetParallelProjection() &&
         std::fabs(cam->GetParallelScale() - scale) < 1e-9;
}
// Is the vtkCamera at the controller's pose?
static bool cameraAt(vtkCamera *cam, const Pose &p) {
  double pos[3], foc[3];
  cam->GetPosition(pos);
  cam->GetFocalPoint(foc);
  return same3(pos, p.eye) && same3(foc, p.focal);
}

// NaN for a missing key, so a regression reads as a FAIL rather than a throw.
static double stateD(cvc::app &app, const std::string &key) {
  try {
    return cvc::state::instance(app)(key).value<double>();
  } catch (...) {
    return std::nan("");
  }
}

// Counts vtkRenderer::ResetCameraClippingRange() calls.
struct ResetCounter {
  int n = 0;
  vtkNew<vtkCallbackCommand> cb;
  explicit ResetCounter(vtkRenderer *r) {
    cb->SetClientData(this);
    cb->SetCallback(
        [](vtkObject *, unsigned long, void *cd, void *) { ++static_cast<ResetCounter *>(cd)->n; });
    r->AddObserver(vtkCommand::ResetCameraClippingRangeEvent, cb);
  }
  int take() {
    const int k = n;
    n = 0;
    return k;
  }
};

static cvc::geometry box(double x, double y, double z, double s) {
  cvc::geometry g;
  const double v[8][3] = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
                          {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
  for (const auto &p : v)
    g.points().push_back({x + s * p[0], y + s * p[1], z + s * p[2]});
  const int f[12][3] = {{0, 1, 2}, {0, 2, 3}, {4, 6, 5}, {4, 7, 6}, {0, 4, 5}, {0, 5, 1},
                        {1, 5, 6}, {1, 6, 2}, {2, 6, 7}, {2, 7, 3}, {3, 7, 4}, {3, 4, 0}};
  for (const auto &t : f)
    g.tris().push_back({static_cast<unsigned int>(t[0]), static_cast<unsigned int>(t[1]),
                        static_cast<unsigned int>(t[2])});
  return g;
}

// A bare camera: no renderer, no window, no scene -- so update() re-applies the
// pose every frame, and a pan converts pixels at the 1-pixel-high fallback.
static void bare(cvc::app &app) {
  vtkNew<vtkCamera> vc;
  CameraController c(app, "mapcam");
  c.setCamera(vc);

  std::printf("A. frameMap + update() hold the map pose\n");
  c.frameMap(100.0, 200.0, 50.0);
  chk(c.mode() == CameraController::Mode::Map, "frameMap enters Map");
  chk(cameraShowsMap(vc, 100.0, 200.0, 50.0), "frameMap shows (100, 200) at scale 50");
  c.update(0.016);
  double pos[3];
  vc->GetPosition(pos);
  chk(at3(pos, 100.0, 200.0, 1000.0),
      "one update(): the eye stays at (100, 200, 1000), got " + fmt3(pos));
  chk(cameraShowsMap(vc, 100.0, 200.0, 50.0), "...looking straight down, north up, at scale 50");
  for (int i = 0; i < 30; ++i)
    c.update(0.016);
  chk(cameraShowsMap(vc, 100.0, 200.0, 50.0), "and after 30 more");
  Pose p = poseOf(c);
  chk(at3(p.eye, 100.0, 200.0, 1000.0) && at3(p.focal, 100.0, 200.0, 0.0) &&
          at3(p.up, 0.0, 1.0, 0.0),
      "getPose() reports the map pose");
  chk(stateD(app, "mapcam.mode") == 3.0, "state mode = 3");
  chk(stateD(app, "mapcam.map.center.x") == 100.0 && stateD(app, "mapcam.map.center.y") == 200.0 &&
          stateD(app, "mapcam.map.scale") == 50.0,
      "state map.center = (100, 200), map.scale = 50");
  chk(stateD(app, "mapcam.pose.eye.x") == 100.0 && stateD(app, "mapcam.pose.eye.y") == 200.0,
      "the mirrored live pose is the map's");

  std::printf("B. pan and zoom move the map pose\n");
  // 1 px of a 1-px-high viewport spans 2 * scale = 100 world units. The view moves
  // against the pointer on both axes (VTK display y grows upward), so the
  // grabbed point stays under it.
  c.beginDrag();
  c.mouseLook(3, 2);
  c.endDrag();
  c.update(0.016);
  chk(cameraShowsMap(vc, 100.0 - 300.0, 200.0 - 200.0, 50.0),
      "drag right 3, up 2: the view moves left 300, down 200");
  c.beginPan(); // middle-drag pans the same way
  c.mouseLook(-1, -1);
  c.endPan();
  c.update(0.016);
  chk(cameraShowsMap(vc, -100.0, 100.0, 50.0), "middle-drag left 1, down 1: right 100, up 100");
  c.mouseLook(5, 5); // not dragging: nothing
  c.update(0.016);
  chk(cameraShowsMap(vc, -100.0, 100.0, 50.0), "a hover (no button) does not pan");
  c.mouseWheel(1.0);
  c.update(0.016);
  chk(cameraShowsMap(vc, -100.0, 100.0, 45.0), "wheel forward: scale 50 -> 45, held by update()");
  c.dolly(-1.0);
  c.update(0.016);
  chk(cameraShowsMap(vc, -100.0, 100.0, 50.0), "dolly back zooms back out to 50");
  c.mouseWheel(2.0);
  const double zoomed = 50.0 * 0.81;
  chk(std::fabs(stateD(app, "mapcam.map.scale") - zoomed) < 1e-9 &&
          stateD(app, "mapcam.map.center.x") == -100.0 &&
          stateD(app, "mapcam.map.center.y") == 100.0,
      "state map.* follows the pan and the zoom");

  std::printf("C. mode switches keep each mode's pose\n");
  c.frameBounds(-50, -50, 0, 50, 50, 20); // in Map: frames the footprint, see D
  c.setMode(CameraController::Mode::Orbit);
  c.beginDrag();
  c.mouseLook(40, -30); // somewhere other than frameBounds' default view
  c.endDrag();
  c.mouseWheel(-2.0);
  const Pose orbit = poseOf(c);
  chk(cameraAt(vc, orbit) && !vc->GetParallelProjection(), "orbiting, in perspective");

  c.frameMap(10.0, 20.0, 30.0);
  c.update(0.016);
  chk(cameraShowsMap(vc, 10.0, 20.0, 30.0), "Orbit -> frameMap -> update: the map");
  c.beginDrag();
  c.mouseLook(1, 0);
  c.endDrag();
  c.mouseWheel(1.0);
  const Pose map = poseOf(c);
  const double mapX = 10.0 - 60.0, mapY = 20.0, mapScale = 27.0;
  c.update(0.016);
  chk(cameraShowsMap(vc, mapX, mapY, mapScale), "panned and zoomed");

  c.setMode(CameraController::Mode::Orbit);
  chk(samePose(poseOf(c), orbit), "Map -> Orbit: the orbit pose it had");
  c.update(0.016);
  chk(cameraAt(vc, orbit), "...on the camera after update()");
  chk(!vc->GetParallelProjection(), "...in perspective again");
  c.beginDrag();
  c.mouseLook(-25, 10);
  c.endDrag();
  const Pose orbit2 = poseOf(c);

  c.setMode(CameraController::Mode::Map);
  chk(samePose(poseOf(c), map), "Orbit -> Map: the map pose it had");
  c.update(0.016);
  chk(cameraShowsMap(vc, mapX, mapY, mapScale), "...on the camera after update(), same scale");
  c.toggleMode();
  chk(c.mode() == CameraController::Mode::Map, "Tab does not leave Map");
  c.setMode(CameraController::Mode::Orbit);
  chk(samePose(poseOf(c), orbit2), "and back to the orbit it was left at");

  c.setMode(CameraController::Mode::Fly);
  c.setMoveSpeed(10.0);
  c.keyDown("w");
  c.update(1.0);
  c.keyUp("w");
  const Pose fly = poseOf(c);
  c.setMode(CameraController::Mode::Map);
  c.update(0.016);
  chk(cameraShowsMap(vc, mapX, mapY, mapScale), "Fly -> Map: the same map");
  c.setMode(CameraController::Mode::Fly);
  chk(samePose(poseOf(c), fly), "Map -> Fly: the fly pose it had");
  c.update(0.016);
  chk(cameraAt(vc, fly) && !vc->GetParallelProjection(), "...on the camera, in perspective");

  // Track starts by easing from the view it inherits. From the map that would be
  // a line of sight straight down the world up -- the chase view's view-up.
  c.setMode(CameraController::Mode::Orbit);
  const Pose orbit3 = poseOf(c);
  c.setMode(CameraController::Mode::Map);
  c.setMode(CameraController::Mode::Track);
  const Pose track = poseOf(c);
  chk(same3(track.eye, orbit3.eye) && same3(track.focal, orbit3.focal),
      "Map -> Track eases in from the orbit view, not from straight down");
  chk(!vc->GetParallelProjection(), "...in perspective");
}

static void stateTree(cvc::app &app) {
  std::printf("D. the state tree\n");
  cvc::state &root = cvc::state::instance(app);
  vtkNew<vtkCamera> vc;
  CameraController c(app, "mapstate");
  c.setCamera(vc);
  c.frameBounds(-50, -50, 0, 50, 50, 20);
  c.setMode(CameraController::Mode::Orbit);
  const Pose orbit = poseOf(c);
  c.frameMap(7.0, 8.0, 9.0);

  root("mapstate.settings.move_speed").value(3.0); // e.g. a Camera-menu slider
  chk(c.mode() == CameraController::Mode::Map, "an outside write in Map stays in Map");
  c.update(0.016);
  chk(cameraShowsMap(vc, 7.0, 8.0, 9.0), "...on the same map");

  root("mapstate.map.center.x").value(-4.0);
  root("mapstate.map.scale").value(12.0);
  c.update(0.016);
  chk(cameraShowsMap(vc, -4.0, 8.0, 12.0), "state map.center.x / map.scale move the map");

  root("mapstate.mode").value(0);
  chk(c.mode() == CameraController::Mode::Orbit, "state mode = 0 leaves Map");
  c.update(0.016);
  chk(cameraAt(vc, orbit) && !vc->GetParallelProjection(), "...for the orbit pose, in perspective");
  root("mapstate.mode").value(3);
  chk(c.mode() == CameraController::Mode::Map, "state mode = 3 enters Map");
  c.update(0.016);
  chk(cameraShowsMap(vc, -4.0, 8.0, 12.0), "...for the map pose");

  // frameBounds() in Map frames the box's footprint from above. No viewport
  // aspect here, so it fits the height: 0.55 * 50 (y extent).
  c.frameBounds(0, 0, 0, 100, 50, 10);
  c.update(0.016);
  chk(c.mode() == CameraController::Mode::Map, "frameBounds keeps the mode");
  chk(cameraShowsMap(vc, 50.0, 25.0, 27.5), "frameBounds in Map: the footprint, from above");
}

static void withScene(cvc::app &app) {
  std::printf("E. update()'s idle path in Map\n");
  vtkNew<vtkRenderer> ren; // outlives the scene, which detaches its props from it
  SceneGraph sg(app, "mapidle");
  sg.setDiagnosticChromeVisible(false);
  sg.setRenderer(ren);
  sg.addGraphics("a", box(0, 0, 0, 10));
  sg.processEvents();

  CameraController c(app, "mapidle.camera");
  c.setScene(&sg);
  c.setRenderer(ren);
  c.setCamera(ren->GetActiveCamera());
  c.setPoseMirrorHz(1000.0); // mirror on every update, if it were going to
  c.frameBounds(0, 0, 0, 10, 10, 10);
  c.frameMap(5.0, 5.0, 20.0);
  sg.processEvents();
  vtkCamera *vc = ren->GetActiveCamera();
  ResetCounter resets(ren);
  c.update(0.016);
  chk(cameraShowsMap(vc, 5.0, 5.0, 20.0), "update() after frameMap: the map");
  resets.take();

  int mirrored = 0;
  boost::signals2::scoped_connection conn =
      cvc::state::instance(app)("mapidle.camera").childChanged.connect([&](const std::string &) {
        ++mirrored;
      });
  const vtkMTimeType camT = vc->GetMTime();
  for (int i = 0; i < 20; ++i) {
    sg.processEvents();
    c.update(0.016);
  }
  chk(resets.take() == 0, "20 idle updates: no clipping-range reset");
  chk(vc->GetMTime() == camT, "camera untouched");
  chk(mirrored == 0, "no pose mirror writes (" + std::to_string(mirrored) + ")");

  c.beginDrag();
  c.mouseLook(0, 1);
  c.endDrag();
  chk(resets.take() == 1, "a pan applies (one re-fit)");
  c.update(0.016);
  chk(resets.take() == 0, "update() has nothing left to do");
  chk(cameraShowsMap(vc, 5.0, -35.0, 20.0), "and holds the panned map");
  c.mouseWheel(-1.0);
  chk(resets.take() == 1, "a zoom applies (one re-fit)");
  c.update(0.016);
  chk(resets.take() == 0 && cameraShowsMap(vc, 5.0, -35.0, 20.0 / 0.9),
      "update() holds the zoomed map");

  vc->SetPosition(1.0, 2.0, 3.0); // someone else moves the camera
  vc->SetParallelScale(99.0);
  c.update(0.016);
  chk(resets.take() == 1 && cameraShowsMap(vc, 5.0, -35.0, 20.0 / 0.9),
      "an outside camera change is put back to the map pose");
}

static void resize(cvc::app &app) {
  std::printf("F. the width fit follows a resize until the user zooms\n");
  vtkNew<vtkRenderWindow> win;
  vtkNew<vtkRenderer> ren;
  win->AddRenderer(ren);
  win->SetSize(400, 200);
  const int *sz = ren->GetSize();
  if (!sz || sz[0] != 400 || sz[1] != 200) {
    std::printf("  skipped: the renderer reports no size without a rendered window\n");
    return;
  }
  CameraController c(app, "mapfit");
  c.setRenderer(ren);
  c.setCamera(ren->GetActiveCamera());
  vtkCamera *vc = ren->GetActiveCamera();
  c.frameMap(0.0, 0.0, 10.0, 40.0); // 80 x 20 rect in a 2:1 viewport: width-bound
  c.update(0.016);
  chk(cameraShowsMap(vc, 0.0, 0.0, 20.0), "2:1 viewport: scale 40 / 2 = 20");
  c.beginDrag();
  c.mouseLook(20, 0); // 20 px of a 200-px-high viewport spans 0.2 * 2 * 20 = 8 units
  c.endDrag();
  c.update(0.016);
  chk(cameraShowsMap(vc, -4.0, 0.0, 20.0), "a 20 px drag pans 2 * scale * 20 / 200 = 4");
  win->SetSize(200, 400);
  c.update(0.016);
  chk(cameraShowsMap(vc, -4.0, 0.0, 80.0), "rotated to 1:2: re-fitted to 40 / 0.5 = 80");
  chk(stateD(app, "mapfit.map.scale") == 80.0, "and mirrored to state");
  c.mouseWheel(1.0);
  win->SetSize(400, 200);
  c.update(0.016);
  chk(cameraShowsMap(vc, -4.0, 0.0, 72.0), "after a zoom the framing is the user's: no re-fit");
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  bare(app);
  stateTree(app);
  withScene(app);
  resize(app);
  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
