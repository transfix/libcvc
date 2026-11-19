// CameraController Slice mode (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 item 2): a
// parallel view straight down one axis, with VolumeGridRover's screen axes.
//
// Pins:
//   A. frameSlice down each axis: the eye looks down it, the screen's right and up are the legacy
//      XY / XZ / ZY views', parallel projection, the box's face fitted, held over updates;
//   B. the state tree: mode 4, slice.axis by name (a view's name too), slice.center / slice.scale
//      mirrored, an outside write of the axis or the mode moves the camera;
//   C. a drag pans along the view's own right and up, nothing turns; the movement keys slide;
//   D. the wheel at the pointer keeps the world point under it, zooming at once and through the
//      glide; the plain wheel zooms about the centre;
//   E. Tab cannot leave Slice; Orbit -> Slice -> Orbit returns to the same orbit; resetView in
//      Slice stays in Slice, refitted; the fit follows a resize until the user zooms.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/state/state.h>
#include <string>
#include <vtkCamera.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

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
  std::snprintf(buf, sizeof(buf), "(%.4f, %.4f, %.4f)", a[0], a[1], a[2]);
  return buf;
}
static double stateD(cvc::app &app, const std::string &key) {
  try {
    return cvc::state::instance(app)(key).value<double>();
  } catch (...) {
    return std::nan("");
  }
}
static std::string stateS(cvc::app &app, const std::string &key) {
  return cvc::state::instance(app)(key).value();
}

// The world point the camera shows at display (x, y) of a w x h viewport whose origin is (0, 0).
static void worldAt(vtkCamera *cam, double x, double y, int w, int h, double out[3]) {
  double pos[3], foc[3], vu[3];
  cam->GetPosition(pos);
  cam->GetFocalPoint(foc);
  cam->GetViewUp(vu);
  const double d[3] = {foc[0] - pos[0], foc[1] - pos[1], foc[2] - pos[2]};
  // right = d x up
  double r[3] = {d[1] * vu[2] - d[2] * vu[1], d[2] * vu[0] - d[0] * vu[2],
                 d[0] * vu[1] - d[1] * vu[0]};
  const double rl = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
  for (double &c : r)
    c /= rl;
  const double per = 2.0 * cam->GetParallelScale() / h;
  for (int c = 0; c < 3; ++c)
    out[c] = foc[c] + r[c] * (x - 0.5 * w) * per + vu[c] * (y - 0.5 * h) * per;
}

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  std::printf("A. frameSlice down each axis\n");
  {
    vtkNew<vtkCamera> vc;
    CameraController c(app, "slicecam");
    c.setZoomSmoothing(0.0);
    c.setCamera(vc);
    struct row {
      int axis;
      double dir[3], up[3], halfH;
      const char *name;
    };
    // The box [0, 10] x [0, 20] x [0, 30]: down z the face is 10 x 20 (half height 10), down y
    // 10 x 30 (15), down x 30 x 20 (10). No renderer: the fit is the half height.
    const row rows[] = {{2, {0, 0, -1}, {0, 1, 0}, 10.0, "z: x right, y up (XY)"},
                        {1, {0, 1, 0}, {0, 0, 1}, 15.0, "y: x right, z up (XZ)"},
                        {0, {1, 0, 0}, {0, 1, 0}, 10.0, "x: z right, y up (ZY)"}};
    for (const row &r : rows) {
      c.setSliceAxis(r.axis);
      c.frameSlice(0, 0, 0, 10, 20, 30);
      for (int i = 0; i < 5; ++i)
        c.update(0.016);
      double pos[3], foc[3], vu[3];
      vc->GetPosition(pos);
      vc->GetFocalPoint(foc);
      vc->GetViewUp(vu);
      const double len =
          std::sqrt((foc[0] - pos[0]) * (foc[0] - pos[0]) + (foc[1] - pos[1]) * (foc[1] - pos[1]) +
                    (foc[2] - pos[2]) * (foc[2] - pos[2]));
      const double d[3] = {(foc[0] - pos[0]) / len, (foc[1] - pos[1]) / len,
                           (foc[2] - pos[2]) / len};
      chk(c.mode() == CameraController::Mode::Slice && c.sliceAxis() == r.axis,
          std::string("Slice down ") + r.name);
      chk(at3(d, r.dir[0], r.dir[1], r.dir[2]) && at3(vu, r.up[0], r.up[1], r.up[2]),
          "looking down the axis, its up the view's up: " + fmt3(d) + " up " + fmt3(vu));
      chk(at3(foc, 5, 10, 15), "centred on the box: " + fmt3(foc));
      chk(vc->GetParallelProjection() && std::fabs(vc->GetParallelScale() - r.halfH) < 1e-9,
          "parallel, the face fitted: scale " + std::to_string(vc->GetParallelScale()));
      chk(len > 30.0, "the eye stands in front of the box");
    }
  }

  std::printf("B. the state tree\n");
  {
    vtkNew<vtkCamera> vc;
    CameraController c(app, "sc2");
    c.setZoomSmoothing(0.0);
    c.setCamera(vc);
    c.frameSlice(0, 0, 0, 10, 20, 30);
    chk(stateD(app, "sc2.mode") == 4.0, "mode = 4");
    chk(stateS(app, "sc2.slice.axis") == "z", "slice.axis = z");
    chk(stateD(app, "sc2.slice.center.x") == 5.0 && stateD(app, "sc2.slice.center.z") == 15.0 &&
            stateD(app, "sc2.slice.scale") == 10.0,
        "slice.center and slice.scale mirrored");
    cvc::state::instance(app)("sc2.slice.axis").value(std::string("xz"));
    c.update(0.016);
    double vu[3];
    vc->GetViewUp(vu);
    chk(c.sliceAxis() == 1 && at3(vu, 0, 0, 1), "slice.axis = xz looks down y");
    cvc::state::instance(app)("sc2.mode").value(0);
    c.update(0.016);
    chk(c.mode() == CameraController::Mode::Orbit && !vc->GetParallelProjection(),
        "mode = 0 leaves Slice, perspective back");
    cvc::state::instance(app)("sc2.mode").value(4);
    c.update(0.016);
    chk(c.mode() == CameraController::Mode::Slice && vc->GetParallelProjection(),
        "mode = 4 enters it again");
    cvc::state::instance(app)("sc2.slice.scale").value(4.0);
    c.update(0.016);
    chk(std::fabs(vc->GetParallelScale() - 4.0) < 1e-9, "slice.scale moves the zoom");
  }

  std::printf("C. pan and keys\n");
  {
    vtkNew<vtkCamera> vc;
    CameraController c(app, "sc3");
    c.setZoomSmoothing(0.0);
    c.setCamera(vc);
    c.setSliceAxis(0); // down x: right is +z, up is +y
    c.frameSlice(0, 0, 0, 10, 20, 30);
    const double s = vc->GetParallelScale();
    // No renderer: one pixel spans 2 * scale. The view moves against the drag.
    c.beginDrag();
    c.mouseLook(1, 2);
    c.endDrag();
    c.update(0.016);
    double foc[3], vu[3];
    vc->GetFocalPoint(foc);
    vc->GetViewUp(vu);
    chk(at3(foc, 5, 10 - 4 * s, 15 - 2 * s), "a drag pans along z and y: " + fmt3(foc));
    chk(at3(vu, 0, 1, 0), "and turns nothing");
    c.keyDown("d");
    c.update(0.5);
    c.keyUp("d");
    double after[3];
    vc->GetFocalPoint(after);
    chk(after[2] > foc[2] && std::fabs(after[1] - foc[1]) < 1e-9 &&
            std::fabs(after[0] - foc[0]) < 1e-9,
        "holding right slides along the view's right (+z)");
  }

  std::printf("D. the wheel at the pointer\n");
  {
    vtkNew<vtkRenderWindow> win;
    win->SetSize(200, 100);
    win->SetOffScreenRendering(1);
    vtkNew<vtkRenderer> ren;
    win->AddRenderer(ren);
    CameraController c(app, "sc4");
    c.setZoomSmoothing(0.0);
    c.setRenderer(ren);
    c.setCamera(ren->GetActiveCamera());
    vtkCamera *vc = ren->GetActiveCamera();
    const int *sz = ren->GetSize();
    if (!sz || sz[0] != 200 || sz[1] != 100) {
      std::printf("  skipped: the renderer reports no size here\n");
    } else {
      c.frameSlice(0, 0, 0, 10, 20, 30);
      double before[3], after[3];
      worldAt(vc, 150, 80, 200, 100, before);
      c.mouseWheelAt(3.0, 150, 80);
      worldAt(vc, 150, 80, 200, 100, after);
      chk(at3(after, before[0], before[1], before[2], 1e-9),
          "zoomed in, the point under the pointer stays: " + fmt3(before) + " -> " + fmt3(after));
      chk(vc->GetParallelScale() < 10.0, "and the view zoomed in");
      // Through the glide.
      c.setZoomSmoothing(0.15);
      worldAt(vc, 30, 20, 200, 100, before);
      c.mouseWheelAt(-4.0, 30, 20);
      bool held = true;
      for (int i = 0; i < 120; ++i) {
        c.update(1.0 / 60.0);
        worldAt(vc, 30, 20, 200, 100, after);
        held = held && at3(after, before[0], before[1], before[2], 1e-9);
      }
      chk(held && !c.zooming(), "through the glide, every frame");
      // The plain wheel zooms about the centre.
      c.setZoomSmoothing(0.0);
      double f0[3], f1[3];
      vc->GetFocalPoint(f0);
      c.mouseWheel(2.0);
      vc->GetFocalPoint(f1);
      chk(at3(f0, f1[0], f1[1], f1[2]), "mouseWheel keeps the centre");
      // The fit follows a resize until the user zooms.
      c.frameSlice(0, 0, 0, 10, 20, 30); // down z: 10 x 20, in 200 x 100: the height fits, 10
      chk(std::fabs(vc->GetParallelScale() - 10.0) < 1e-9, "fitted at 200 x 100");
      win->SetSize(100, 200); // now the width binds: 5 / 0.5 = 10 ... and 10 tall: 10
      c.update(0.016);
      win->SetSize(50, 200); // aspect 0.25: half width 5 needs scale 20
      c.update(0.016);
      chk(std::fabs(vc->GetParallelScale() - 20.0) < 1e-9,
          "re-fitted to a narrow view: " + std::to_string(vc->GetParallelScale()));
      c.mouseWheel(1.0);
      const double zoomed = vc->GetParallelScale();
      win->SetSize(200, 100);
      c.update(0.016);
      chk(std::fabs(vc->GetParallelScale() - zoomed) < 1e-9, "after a zoom the framing stays");
    }
  }

  std::printf("E. modes\n");
  {
    vtkNew<vtkCamera> vc;
    CameraController c(app, "sc5");
    c.setZoomSmoothing(0.0);
    c.setCamera(vc);
    c.frameBounds(0, 0, 0, 10, 20, 30);
    double e0[3], f0[3], u0[3];
    c.getPose(e0, f0, u0);
    c.setMode(CameraController::Mode::Slice);
    double foc[3];
    vc->GetFocalPoint(foc);
    chk(at3(foc, 5, 10, 15) && vc->GetParallelProjection(),
        "frameBounds seeded the slice pose: Slice shows the box's centre");
    c.toggleMode();
    chk(c.mode() == CameraController::Mode::Slice, "Tab cannot leave Slice");
    c.setMode(CameraController::Mode::Orbit);
    double e1[3], f1[3], u1[3];
    c.getPose(e1, f1, u1);
    chk(at3(e1, e0[0], e0[1], e0[2]) && at3(f1, f0[0], f0[1], f0[2]),
        "Orbit -> Slice -> Orbit is the same orbit");
    c.setMode(CameraController::Mode::Slice);
    c.mouseWheel(5.0);
    c.resetView();
    chk(c.mode() == CameraController::Mode::Slice &&
            std::fabs(vc->GetParallelScale() - 10.0) < 1e-9,
        "resetView in Slice stays in Slice, refitted");
  }

  std::printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
