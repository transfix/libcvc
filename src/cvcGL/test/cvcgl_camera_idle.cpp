// CameraController::update() on a still camera over a still scene must cost
// nothing -- and must still re-fit the clipping range the moment it could have
// gone stale.
//
// update() used to call applyToCamera() every frame. Re-setting an unchanged
// pose is a no-op for vtkCamera, but the vtkRenderer::ResetCameraClippingRange()
// that comes with it re-derives every visible prop's bounds -- a mapper Update
// plus a cell-bounds pass for each mesh rewritten since the last frame. In the
// demo3 wasm profile that was 1-1.5 ms a frame (plus ~2 ms of throttled pose
// mirroring: 14 state writes, each a path lookup), with the camera standing still.
//
// Pins:
//   A. idle frames: no clipping-range reset, no camera change, no pose mirror;
//   B. the range is re-fitted when the scene could have grown with the camera
//      still: a node added (registered or not), a registered node moved, a mesh
//      replaced, a prop shown;
//   C. any pose change, any outside change to the camera, still re-applies;
//   D. no scene: the old every-frame behaviour;
//   E. with GL (skipped without): rendering itself -- shadows on -- does not
//      disturb the idle path;
//   F. nodes that change what is drawn without touching the renderer's prop
//      list: a RibbonNode whose box grows as its track drives away from a still
//      camera (its far end stays inside [near, far]), and an LOD node shown,
//      hidden or switched to another rung.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/RibbonNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/lod/pyramid.h>
#include <string>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkNew.h>
#include <vtkRenderer.h>

using cvc::gl::CameraController;
using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

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

// The point `dist` ahead of the camera, on its line of sight.
static void ahead(vtkCamera *cam, double dist, double out[3]) {
  double pos[3], dop[3];
  cam->GetPosition(pos);
  cam->GetDirectionOfProjection(dop);
  for (int i = 0; i < 3; ++i)
    out[i] = pos[i] + dist * dop[i];
}

// Does the camera's [near, far] slab contain the point?
static bool inSlab(vtkCamera *cam, const double p[3]) {
  double pos[3], dop[3], cr[2];
  cam->GetPosition(pos);
  cam->GetDirectionOfProjection(dop);
  cam->GetClippingRange(cr);
  const double d = (p[0] - pos[0]) * dop[0] + (p[1] - pos[1]) * dop[1] + (p[2] - pos[2]) * dop[2];
  return d >= cr[0] && d <= cr[1];
}

static void headless(cvc::app &app) {
  std::printf("A. idle frames cost nothing\n");
  vtkNew<vtkRenderer> ren; // outlives the scene, which detaches its props from it
  SceneGraph sg(app, "idle");
  sg.setDiagnosticChromeVisible(false);
  sg.setRenderer(ren);
  auto a = sg.addGraphics("a", box(0, 0, 0, 10));
  sg.processEvents();

  CameraController cam(app, "idle.camera");
  cam.setScene(&sg);
  cam.setRenderer(ren);
  cam.setCamera(ren->GetActiveCamera());
  cam.setPoseMirrorHz(1000.0); // mirror on every update, if it were going to
  cam.frameBounds(0, 0, 0, 10, 10, 10);
  sg.processEvents();
  ResetCounter resets(ren);
  cam.update(0.016); // settles anything frameBounds left pending
  resets.take();

  int mirrored = 0;
  boost::signals2::scoped_connection conn =
      cvc::state::instance(app)("idle.camera").childChanged.connect([&](const std::string &) {
        ++mirrored;
      });
  vtkCamera *vc = ren->GetActiveCamera();
  const vtkMTimeType camT = vc->GetMTime();
  for (int i = 0; i < 20; ++i) {
    sg.processEvents();
    cam.update(0.016);
  }
  chk(resets.take() == 0, "20 idle updates: no clipping-range reset");
  chk(vc->GetMTime() == camT, "camera untouched");
  chk(mirrored == 0, "no pose mirror writes (" + std::to_string(mirrored) + ")");

  std::printf("B. the range follows the scene while the camera holds still\n");
  double farPt[3];
  ahead(vc, 400.0, farPt); // far beyond the scene, straight ahead
  sg.addGraphics("far", box(farPt[0] - 5, farPt[1] - 5, farPt[2] - 5, 10));
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "a registered node added: re-fitted");
  chk(inSlab(vc, farPt), "and the new node lies inside [near, far]");
  for (int i = 0; i < 5; ++i)
    cam.update(0.016);
  chk(resets.take() == 0, "then idle again");

  double movedPt[3];
  ahead(vc, 900.0, movedPt);
  a->setPosition(movedPt[0] - 5, movedPt[1] - 5, movedPt[2] - 5); // its box spans [0, 10]
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "a registered node moved: re-fitted");
  chk(inSlab(vc, movedPt), "and the moved node lies inside [near, far]");

  // Not registered: a direct child of the graphics root (demo3's vehicle slots).
  auto loose = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("loose");
  sg.processEvents();
  cam.update(0.016);
  resets.take();
  double loosePt[3];
  ahead(vc, 1500.0, loosePt);
  loose->setGeometry(box(loosePt[0] - 5, loosePt[1] - 5, loosePt[2] - 5, 10));
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "an unregistered node's mesh replaced: re-fitted");
  chk(inSlab(vc, loosePt), "and it lies inside [near, far]");

  loose->setVisible(false);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "a prop hidden: re-fitted");
  loose->setVisible(true);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "and shown again: re-fitted");

  // An unregistered node MOVING is the documented blind spot (its transform is
  // not tracked); markContentChanged() is the way to say so.
  loose->setPosition(0.0, 0.0, -50.0);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 0, "an unregistered node moving alone is not seen");
  sg.markContentChanged();
  cam.update(0.016);
  chk(resets.take() == 1, "markContentChanged(): re-fitted");

  std::printf("C. pose and camera changes still apply\n");
  cam.beginDrag();
  cam.mouseLook(10, 0);
  cam.endDrag();
  resets.take(); // mouseLook applies on its own
  cam.update(0.016);
  chk(resets.take() == 0, "orbit drag applied by mouseLook; update has nothing left to do");
  double e0[3], f0[3], u0[3];
  cam.getPose(e0, f0, u0);
  vc->SetPosition(1.0, 2.0, 3.0); // someone else moves the camera
  cam.update(0.016);
  double e1[3];
  vc->GetPosition(e1);
  chk(resets.take() == 1 && e1[0] == e0[0] && e1[1] == e0[1] && e1[2] == e0[2],
      "an outside camera move is overridden by the controller's pose, as before");
  cam.setMode(CameraController::Mode::Fly);
  resets.take();
  cam.keyDown("w");
  cam.update(0.5);
  cam.keyUp("w");
  chk(resets.take() == 1, "fly motion re-applies");
  cam.update(0.016);
  chk(resets.take() == 0, "and stops when the key is up");
  mirrored = 0;
  for (int i = 0; i < 3; ++i)
    cam.update(0.016);
  chk(mirrored == 0, "idle again: no mirror");
  cam.keyDown("w");
  cam.update(0.1);
  cam.keyUp("w");
  chk(mirrored > 0, "a moving pose is still mirrored");
  double mx = cvc::state::instance(app)("idle.camera.pose.eye.x").value<double>();
  double ex[3], fx[3], ux[3];
  cam.getPose(ex, fx, ux);
  chk(mx == ex[0], "mirror holds the live pose");

  std::printf("D. no scene: re-fit every frame, as before\n");
  vtkNew<vtkRenderer> ren2;
  CameraController bare(app, "idle.bare");
  bare.setRenderer(ren2);
  bare.setCamera(ren2->GetActiveCamera());
  ResetCounter resets2(ren2);
  for (int i = 0; i < 4; ++i)
    bare.update(0.016);
  chk(resets2.take() == 4, "4 updates, 4 resets");
}

static void streamingAndLod(cvc::app &app) {
  std::printf("F. streaming boxes and LOD rungs\n");
  vtkNew<vtkRenderer> ren;
  SceneGraph sg(app, "idlestream");
  sg.setDiagnosticChromeVisible(false);
  sg.setRenderer(ren);
  sg.addGraphics("a", box(0, 0, 0, 10));
  auto track = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::RibbonNode>(
      "track", 8, 1.0f, cvc::bounding_box(0, 0, 0, 10, 10, 1));
  track->append(1, 5, 0.5f);
  track->append(9, 5, 0.5f);
  sg.processEvents();

  CameraController cam(app, "idlestream.camera");
  cam.setScene(&sg);
  cam.setRenderer(ren);
  cam.setCamera(ren->GetActiveCamera());
  cam.frameBounds(0, 0, 0, 10, 10, 10);
  sg.processEvents();
  cam.update(0.016);
  ResetCounter resets(ren);
  for (int i = 0; i < 3; ++i)
    cam.update(0.016);
  chk(resets.take() == 0, "idle with a ribbon in the scene");

  // Drive the track straight away from the still camera, 400 units out. Its
  // box grows with it (RibbonNode keeps the box over its centres), which VTK
  // clips by -- and which changes no MTime.
  vtkCamera *vc = ren->GetActiveCamera();
  double farEnd[3] = {0, 0, 0};
  for (int k = 1; k <= 40; ++k) {
    ahead(vc, 10.0 * k + 20.0, farEnd);
    track->append(static_cast<float>(farEnd[0]), static_cast<float>(farEnd[1]),
                  static_cast<float>(farEnd[2]));
    sg.processEvents();
    cam.update(0.016);
  }
  chk(resets.take() > 0, "the growing ribbon re-fitted the clipping range");
  chk(inSlab(vc, farEnd), "and its far end lies inside [near, far]");
  for (int i = 0; i < 3; ++i)
    cam.update(0.016);
  chk(resets.take() == 0, "idle again once it stops");

  // An LOD node draws its rungs by SetVisibility on their actors, which the
  // renderer's prop list does not show.
  auto lod = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::LodGraphicsNode>("lod");
  cvc::lod::mesh_pyramid pyr;
  pyr.rungs.push_back(box(-300, -300, -300, 10));
  pyr.rungs.push_back(box(-300, -300, -300, 10));
  pyr.world_error_m = {0.0, 1.0};
  lod->setPyramid(pyr);
  sg.processEvents();
  cam.update(0.016);
  resets.take();
  lod->setVisible(false);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "hiding an LOD node re-fits");
  lod->setVisible(true);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "showing it again re-fits");
  lod->setRung(1);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 1, "switching its rung re-fits");
  lod->setRung(1);
  sg.processEvents();
  cam.update(0.016);
  chk(resets.take() == 0, "re-selecting the same rung does not");
}

static bool canRasterise(cvc::app &app) {
  SceneGraph sg(app, "idlecontrol");
  sg.setDiagnosticChromeVisible(false);
  auto g = sg.getGraphicsRoot()->addGraphicsChild<GeometryNode>("control");
  g->setGeometry(box(-1, -1, -1, 2));
  SceneRenderer sr(sg, 48, 48, /*offscreen=*/true);
  sr.setBackground(0.0, 0.0, 0.0);
  sr.setCamera(4, 4, 4, 0, 0, 0, 0, 0, 1, 40.0, 0.5, 100.0);
  const std::vector<unsigned char> rgb = sr.frameRGB();
  for (unsigned char c : rgb)
    if (c > 20)
      return true;
  return false;
}

static void rendered(cvc::app &app) {
  std::printf("E. rendering does not disturb the idle path\n");
  if (!canRasterise(app)) {
    const char *require = std::getenv("CVC_REQUIRE_RENDER");
    if (require && *require && std::string(require) != "0") {
      std::printf("  CVC_REQUIRE_RENDER is set, but this build did not rasterise\n");
      chk(false, "rasterise");
    } else {
      std::printf("  skipped: this build did not rasterise\n");
    }
    return;
  }
  SceneGraph sg(app, "idlegl");
  sg.setDiagnosticChromeVisible(false);
  for (int i = 0; i < 4; ++i)
    sg.addGraphics("b" + std::to_string(i), box(i * 15.0, 0, 0, 10));
  sg.processEvents();
  SceneRenderer view(sg, 96, 72, /*offscreen=*/true, "main");
  sg.addDirectionalLight(-40.0, 50.0);
  const bool shadows = sg.setShadowsEnabled(true);
  CameraController cam(view);
  cam.frameBounds(0, 0, 0, 55, 10, 10);
  ResetCounter resets(view.renderer());
  for (int i = 0; i < 3; ++i) { // settle: first renders, shadow keys on props
    sg.processEvents();
    cam.update(0.016);
    view.render();
  }
  resets.take();
  vtkCamera *vc = view.renderer()->GetActiveCamera();
  const vtkMTimeType camT = vc->GetMTime();
  for (int i = 0; i < 10; ++i) {
    sg.processEvents();
    cam.update(0.016);
    view.render();
  }
  chk(resets.take() == 0, std::string("10 rendered idle frames") +
                              (shadows ? " (shadows on)" : "") + ": no clipping-range reset");
  chk(vc->GetMTime() == camT, "rendering left the camera's MTime alone");
}

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  headless(app);
  streamingAndLod(app);
  rendered(app);
  std::printf("\n%s (%d failure%s)\n", fails ? "FAILED" : "PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}
