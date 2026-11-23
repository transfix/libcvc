// cvcgl_ariadne_view_stacking — an Ariadne `viewport` widget's view is drawn into a texture of its
// own and shown as an image in its window (Viewport::setTextureSize, docs/ARIADNE.md §5.18), so its
// window stacks, clips and scrolls it like any of its content. Run as a host runs a document:
// AriRuntime + ImGuiBackend over an offscreen SceneRenderer. Checked, in the frame's pixels:
//   - a window in front of a view's window covers the view;
//   - a view's window in front of another window shows the view over it (when views were drawn
//     under the whole UI, the window behind drew over the view);
//   - the first frame a view's widget is drawn shows the view, framed on the scene;
//   - a view cut off by its window shows the top of what the same view draws in full, and a view
//     scrolled part way out of its window shows the matching rows: the window clips and scrolls the
//     picture, which keeps its size (three views of one named camera, the same size);
//   - a view's texture is the widget's size, and nothing else of it reaches the window;
//   - a pick in the scrolled view, at a place on screen, lands where the same pick in the full view
//     lands (SceneViews::pick maps the window's pixels to the view's texture).
// Skips (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/Viewport.h>
#include <cvc/gl/ViewportManager.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_views.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <vtkRenderer.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // the windows' rects and their order
#endif

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

// Two views and two plain windows. Under's view (magenta) has Cover in front of it; Over's view
// (cyan) is in front of the plain window Behind. ImGui puts a window that appears in front of those
// before it, so each later window is in front of the one it overlaps.
const char *kStackDoc = R"(meta:
  name: view stacking
  min_libcvc: 3.7.0

windows:
  - window: Under
    id: under_window
    pos: [20, 40]
    size: [320, 280]
    children:
      - { type: viewport, view: under, clear_color: [0.9, 0.1, 0.8], chrome: false, shadows: false, input: false, zoom: 0.4, width: 280, height: 210 }
  - window: Cover
    id: cover_window
    pos: [200, 140]
    size: [260, 220]
    children:
      - text: "in front of the view"
  - window: Behind
    id: behind_window
    pos: [500, 40]
    size: [260, 280]
    children:
      - text: "behind the view's window"
  - window: Over
    id: over_window
    pos: [620, 140]
    size: [320, 280]
    children:
      - { type: viewport, view: over, clear_color: [0.1, 0.8, 0.9], chrome: false, shadows: false, input: false, zoom: 0.4, width: 280, height: 210 }

scene:
  chrome: false
  nodes:
    - { node: box, type: geometry, source: { box: { size: [2, 2, 2] } }, material: { color: [0.95, 0.85, 0.1] } }
)";

// Three views of one named camera, all 280 x 240: Full in a window with room for it, Clipped in a
// window that cuts it off at the bottom, Scrolled below three lines of text in a short window the
// test scrolls down.
const char *kScrollDoc = R"(meta:
  name: view clipping
  min_libcvc: 3.7.0

windows:
  - window: Full
    id: full_window
    pos: [20, 40]
    size: [320, 330]
    children:
      - { type: viewport, view: full, camera: shared, clear_color: [0.2, 0.2, 0.25], chrome: false, shadows: false, input: pick, width: 280, height: 240 }
  - window: Clipped
    id: clipped_window
    pos: [360, 40]
    size: [320, 180]
    children:
      - { type: viewport, view: clipped, camera: shared, clear_color: [0.2, 0.2, 0.25], chrome: false, shadows: false, input: false, width: 280, height: 240 }
  - window: Scrolled
    id: scrolled_window
    pos: [360, 300]
    size: [320, 200]
    children:
      - text: "one"
      - text: "two"
      - text: "three"
      - { type: viewport, view: scrolled, camera: shared, clear_color: [0.2, 0.2, 0.25], chrome: false, shadows: false, input: pick, width: 280, height: 240 }
      - text: "after"

scene:
  chrome: false
  cameras:
    - camera: shared
      azimuth: 30
      elevation: 25
  nodes:
    - { node: box, type: geometry, source: { box: { size: [2, 2, 2] } }, material: { color: [0.95, 0.85, 0.1] } }
    - { node: ball, type: geometry, source: { sphere: 0.8 }, transform: { position: [2.5, 0, 0] }, material: { color: [0.1, 0.3, 0.9] } }
)";

// A rect in the window's pixels, y down (ImGui's).
struct Rect {
  double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
  bool empty() const { return !(x1 > x0) || !(y1 > y0); }
  Rect cut(const Rect &o) const {
    return {std::max(x0, o.x0), std::max(y0, o.y0), std::min(x1, o.x1), std::min(y1, o.y1)};
  }
  Rect inset(double d) const { return {x0 + d, y0 + d, x1 - d, y1 - d}; }
  double area() const { return empty() ? 0.0 : (x1 - x0) * (y1 - y0); }
};

// Where a view shows: its region, y down.
Rect viewRect(cvc::gl::Viewport &vp) {
  double r[4];
  vp.region(r);
  return {r[0] * kWidth, (1.0 - r[3]) * kHeight, r[2] * kWidth, (1.0 - r[1]) * kHeight};
}

const unsigned char *px(const cvcgl_test::Frame &f, int x, int ydown) {
  return f.at(x, f.h - 1 - ydown);
}

// Pixels in `r` within 40 of `rgb`.
long countColour(const cvcgl_test::Frame &f, const Rect &r, const int rgb[3]) {
  long n = 0;
  for (int y = static_cast<int>(std::ceil(r.y0)); y < static_cast<int>(r.y1); ++y)
    for (int x = static_cast<int>(std::ceil(r.x0)); x < static_cast<int>(r.x1); ++x) {
      const unsigned char *p = px(f, x, y);
      n += std::abs(p[0] - rgb[0]) <= 40 && std::abs(p[1] - rgb[1]) <= 40 &&
                   std::abs(p[2] - rgb[2]) <= 40
               ? 1
               : 0;
    }
  return n;
}

// Pixels in `r` of the yellow box, lit or in shade.
long countBox(const cvcgl_test::Frame &f, const Rect &r) {
  long n = 0;
  for (int y = static_cast<int>(std::ceil(r.y0)); y < static_cast<int>(r.y1); ++y)
    for (int x = static_cast<int>(std::ceil(r.x0)); x < static_cast<int>(r.x1); ++x) {
      const unsigned char *p = px(f, x, y);
      n += p[0] > 60 && p[1] > 50 && p[0] > 2 * p[2] && p[1] > 2 * p[2] ? 1 : 0;
    }
  return n;
}

#ifdef CVC_ENABLE_IMGUI
// An Ariadne window's ImGui window, by its id ("title###id"), and its place in the display order
// (later is in front).
ImGuiWindow *findWindow(const char *id, int *order = nullptr) {
  ImGuiContext &g = *ImGui::GetCurrentContext();
  const std::string tail = std::string("###") + id;
  for (int i = 0; i < g.Windows.Size; ++i) {
    ImGuiWindow *w = g.Windows[i];
    const std::string name = w->Name;
    if (name.size() >= tail.size() &&
        name.compare(name.size() - tail.size(), tail.size(), tail) == 0) {
      if (order)
        *order = i;
      return w;
    }
  }
  return nullptr;
}

Rect windowRect(const ImGuiWindow *w) {
  return {w->Pos.x, w->Pos.y, w->Pos.x + w->Size.x, w->Pos.y + w->Size.y};
}
Rect clipRect(const ImGuiWindow *w) {
  return {w->InnerClipRect.Min.x, w->InnerClipRect.Min.y, w->InnerClipRect.Max.x,
          w->InnerClipRect.Max.y};
}
#endif

// An offscreen host, as the other Ariadne view tests make one.
struct Host {
  cvc::app app;
  std::unique_ptr<cvc::gl::SceneGraph> sg;
  std::unique_ptr<cvc::gl::SceneRenderer> view;
  std::unique_ptr<cvc::gl::CameraController> cam;
  std::unique_ptr<cvc::gl::ImGuiOverlay> ui;
  explicit Host(const char *prefix) {
    app.properties("system.log_verbosity", "0");
    sg = std::make_unique<cvc::gl::SceneGraph>(app, prefix);
    sg->setDiagnosticChromeVisible(false);
    view = std::make_unique<cvc::gl::SceneRenderer>(*sg, kWidth, kHeight, true, "main");
    view->setBackground(0.0, 0.0, 0.0);
    cam = std::make_unique<cvc::gl::CameraController>(*view);
    ui = std::make_unique<cvc::gl::ImGuiOverlay>(*view);
    if (!ui->enabled())
      return;
    ui->attachCamera(*cam);
    ui->setVisible(true);
    ui->setTouchMode(false);
    ui->setUiScale(1.0f);
    ui->setToggleButtonEnabled(false);
    ui->setPanelsOpen(true);
    ui->setFixedDeltaTime(static_cast<float>(kDt));
  }
};

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_view_stacking";
  fs::create_directories(dir);
  const fs::path stackDoc = dir / "stack.ari", scrollDoc = dir / "scroll.ari";
  std::ofstream(stackDoc) << kStackDoc;
  std::ofstream(scrollDoc) << kScrollDoc;
  const int magenta[3] = {230, 26, 204}, cyan[3] = {26, 204, 230};

  // ── stacking, and the first frame ─────────────────────────────────────────────────────────────
  {
    Host h("stack");
    if (!h.ui->enabled()) {
      std::printf("SKIP: no ImGui overlay\n");
      return require ? 1 : 0;
    }
    cvc::gl::ariadne::AriRuntime rt(*h.view, *h.cam, *h.ui);
    for (const std::string &w : rt.load(stackDoc.string()))
      check(false, "the stacking document loads without warnings", w);
    cvc::gl::ViewportManager &vm = h.view->viewportManager();

    // The first frame a view's widget is drawn already shows the view: its colour fills the rect
    // where it shows, and the box it is framed on is in it.
    int first = -1;
    cvcgl_test::Frame f;
    for (int frame = 0; frame < 6 && first < 0; ++frame) {
      rt.frame(kDt);
      f = cvcgl_test::grab(*h.view); // this frame's render, widget walk and all
      if (!h.ui->enabled()) {
        std::printf("SKIP: nothing rasterises\n");
        return require ? 1 : 0;
      }
      if (vm.hasViewport("over") && vm.viewport("over").visible())
        first = frame;
    }
    check(first >= 0, "the view's widget is drawn within the first frames", std::to_string(first));
    if (first < 0)
      return cvcgl_test::finish("cvcgl_ariadne_view_stacking");
    {
      cvc::gl::Viewport &over = vm.viewport("over");
      const Rect r = viewRect(over).inset(2.0);
      const long shown = countColour(f, r, cyan), box = countBox(f, r);
      std::printf("  first frame (%d): cyan %ld, box %ld of %.0f px\n", first, shown, box,
                  r.area());
      check(shown + box > 0.9 * r.area(), "the view's first frame shows the view, not its window",
            std::to_string(shown + box) + " of " + std::to_string(r.area()));
      check(box > 500, "and the box it is framed on", std::to_string(box));
      int ts[2];
      over.textureSize(ts);
      check(over.textured() && ts[0] == 280 && ts[1] == 210,
            "the view draws into a texture the widget's size",
            std::to_string(ts[0]) + " x " + std::to_string(ts[1]));
      double vp[4];
      over.renderer()->GetViewport(vp);
      check(vp[0] == 0.0 && vp[1] == 0.0 && std::fabs(vp[2] * kWidth - 280.0) < 1e-6 &&
                std::fabs(vp[3] * kHeight - 210.0) < 1e-6,
            "its renderer draws at the texture's rect");
    }
    for (int frame = 0; frame < 4; ++frame)
      rt.step(kDt);

    ImGui::SetCurrentContext(h.ui->imguiContext());
    int oUnder = -1, oCover = -1, oBehind = -1, oOver = -1;
    ImGuiWindow *under = findWindow("under_window", &oUnder);
    ImGuiWindow *cover = findWindow("cover_window", &oCover);
    ImGuiWindow *behind = findWindow("behind_window", &oBehind);
    ImGuiWindow *overW = findWindow("over_window", &oOver);
    check(under && cover && behind && overW, "the four windows are ImGui's");
    if (!under || !cover || !behind || !overW || !vm.hasViewport("under"))
      return cvcgl_test::finish("cvcgl_ariadne_view_stacking");
    check(oCover > oUnder && oOver > oBehind,
          "Cover is in front of Under, and Over in front of Behind");
    f = cvcgl_test::grab(*h.view);
    if (const char *png = std::getenv("CVC_STACKING_PNG"))
      h.view->writePNG(png);

    // A window in front of a view covers it.
    const Rect underView = viewRect(vm.viewport("under"));
    const Rect covered = underView.cut(windowRect(cover)).inset(3.0);
    const long viewUnder = countColour(f, covered, magenta);
    const long viewBeside =
        countColour(f, underView.inset(3.0), magenta) + countBox(f, underView.inset(3.0));
    std::printf("  under the Cover window: magenta %ld of %.0f px; in all the view %ld\n",
                viewUnder, covered.area(), viewBeside);
    check(!covered.empty() && covered.area() > 2000.0, "Cover overlaps Under's view");
    check(viewUnder == 0, "a window in front of a view covers it", std::to_string(viewUnder));
    check(viewBeside > 0.5 * underView.area(), "the rest of that view shows");

    // A view's window in front of another window shows the view over it.
    const Rect overView = viewRect(vm.viewport("over"));
    const Rect overBehind = overView.cut(windowRect(behind)).inset(3.0);
    const long shownOver = countColour(f, overBehind, cyan) + countBox(f, overBehind);
    std::printf("  over the Behind window: the view's %ld of %.0f px\n", shownOver,
                overBehind.area());
    check(!overBehind.empty() && overBehind.area() > 2000.0, "Over's view overlaps Behind");
    check(shownOver > 0.9 * overBehind.area(),
          "a view's window in front of another window shows the view over it",
          std::to_string(shownOver) + " of " + std::to_string(overBehind.area()));
    h.view->close(); // ~AriRuntime runs with the view closed
  }

  // ── clipped and scrolled views ────────────────────────────────────────────────────────────────
  {
    Host h("clip");
    cvc::gl::ariadne::AriRuntime rt(*h.view, *h.cam, *h.ui);
    for (const std::string &w : rt.load(scrollDoc.string()))
      check(false, "the clipping document loads without warnings", w);
    for (int frame = 0; frame < 6; ++frame)
      rt.step(kDt);
    cvc::gl::ViewportManager &vm = h.view->viewportManager();
    check(vm.hasViewport("full") && vm.hasViewport("clipped") && vm.hasViewport("scrolled"),
          "the three views exist");
    if (!vm.hasViewport("full") || !vm.hasViewport("clipped") || !vm.hasViewport("scrolled"))
      return cvcgl_test::finish("cvcgl_ariadne_view_stacking");
    cvc::gl::Viewport &full = vm.viewport("full"), &clipped = vm.viewport("clipped");
    cvc::gl::Viewport &scrolled = vm.viewport("scrolled");
    ImGui::SetCurrentContext(h.ui->imguiContext());
    ImGuiIO &io = ImGui::GetIO();

    // Scroll the Scrolled window to its end, the wheel over its text (not over the view, whose
    // camera the wheel would zoom).
    ImGuiWindow *sw = findWindow("scrolled_window");
    check(sw != nullptr, "the Scrolled window is ImGui's");
    if (!sw)
      return cvcgl_test::finish("cvcgl_ariadne_view_stacking");
    const Rect before = viewRect(scrolled);
    io.AddMousePosEvent(sw->Pos.x + 40.0f, clipRect(sw).y0 + 8.0f);
    rt.step(kDt);
    for (int i = 0; i < 6; ++i) {
      io.AddMouseWheelEvent(0.0f, -3.0f);
      rt.step(kDt);
    }
    io.AddMousePosEvent(-1.0f, -1.0f);
    for (int frame = 0; frame < 3; ++frame)
      rt.step(kDt);
    const Rect after = viewRect(scrolled);
    check(after.y0 < before.y0 - 20.0, "the wheel scrolled the window, view and all",
          std::to_string(before.y0) + " -> " + std::to_string(after.y0));

    const cvcgl_test::Frame f = cvcgl_test::grab(*h.view);
    if (const char *png = std::getenv("CVC_CLIPPING_PNG"))
      h.view->writePNG(png);
    int ts[2];
    clipped.textureSize(ts);
    check(ts[0] == 280 && ts[1] == 240, "a clipped view still draws at the widget's size",
          std::to_string(ts[0]) + " x " + std::to_string(ts[1]));

    // Pixels of view `v` against the same place in the full view, over the part of `v` its window
    // shows (less 2 px at the window's clip edges): how many differ by more than 8.
    const Rect fullRect = viewRect(full);
    auto compare = [&](cvc::gl::Viewport &v, const char *windowId, long &shown) {
      ImGuiWindow *w = findWindow(windowId);
      shown = 0;
      if (!w)
        return -1L;
      const Rect vr = viewRect(v);
      const Rect seen = vr.cut(clipRect(w)).inset(2.0);
      long off = 0;
      for (int y = static_cast<int>(std::ceil(seen.y0)); y < static_cast<int>(seen.y1); ++y)
        for (int x = static_cast<int>(std::ceil(seen.x0)); x < static_cast<int>(seen.x1); ++x) {
          const int fx = static_cast<int>(std::lround(x - vr.x0 + fullRect.x0));
          const int fy = static_cast<int>(std::lround(y - vr.y0 + fullRect.y0));
          const unsigned char *a = px(f, x, y), *b = px(f, fx, fy);
          ++shown;
          off += std::max({std::abs(a[0] - b[0]), std::abs(a[1] - b[1]), std::abs(a[2] - b[2])}) > 8
                     ? 1
                     : 0;
        }
      return off;
    };
    long seenClipped = 0, seenScrolled = 0;
    const long offClipped = compare(clipped, "clipped_window", seenClipped);
    const long offScrolled = compare(scrolled, "scrolled_window", seenScrolled);
    std::printf("  clipped: %ld of %ld px differ from the full view; scrolled: %ld of %ld\n",
                offClipped, seenClipped, offScrolled, seenScrolled);
    const Rect clipView = viewRect(clipped);
    ImGuiWindow *cw = findWindow("clipped_window");
    check(cw && clipView.y1 > clipRect(cw).y1 + 20.0, "the Clipped window cuts its view off");
    check(seenClipped > 20000 && offClipped >= 0 && offClipped <= seenClipped / 1000,
          "a clipped view shows the top of the same picture, uncut and unsquashed",
          std::to_string(offClipped) + " of " + std::to_string(seenClipped));
    check(after.y0 < clipRect(sw).y0 - 20.0, "the scrolled view runs past its window's top");
    check(seenScrolled > 10000 && offScrolled >= 0 && offScrolled <= seenScrolled / 1000,
          "a scrolled view shows the matching rows of the same picture",
          std::to_string(offScrolled) + " of " + std::to_string(seenScrolled));

    // A pick on screen in the scrolled view lands where the same pick in the full view does: the
    // box's centre, as the full view draws it, mapped to where the scrolled view shows it.
    vtkRenderer *ren = full.renderer();
    ren->SetWorldPoint(0.0, 0.0, 0.0, 1.0);
    ren->WorldToDisplay();
    double d[3], onFull[2];
    ren->GetDisplayPoint(d);
    full.rendererToWindow(d[0], d[1], onFull);
    // The same texel in the scrolled view: its region is the full view's moved.
    double sr[4], fr[4];
    scrolled.region(sr);
    full.region(fr);
    const double sx = onFull[0] + (sr[0] - fr[0]) * kWidth;
    const double sy = onFull[1] + (sr[1] - fr[1]) * kHeight;
    const Rect seen = viewRect(scrolled).cut(clipRect(sw));
    const double syDown = kHeight - sy;
    check(sx > seen.x0 && sx < seen.x1 && syDown > seen.y0 && syDown < seen.y1,
          "the box's centre is in the part of the scrolled view that shows");
    cvc::gl::ariadne::SceneViews::PickHit inFull, inScrolled;
    const bool gotFull = rt.views().pick("full", onFull[0], onFull[1], inFull);
    const bool gotScrolled = rt.views().pick("scrolled", sx, sy, inScrolled);
    const double dist = std::fabs(inFull.world[0] - inScrolled.world[0]) +
                        std::fabs(inFull.world[1] - inScrolled.world[1]) +
                        std::fabs(inFull.world[2] - inScrolled.world[2]);
    check(gotFull && gotScrolled && inFull.node == "box" && inScrolled.node == "box" && dist < 1e-3,
          "a pick in the scrolled view lands where it does in the full view",
          inScrolled.node + " " + std::to_string(dist));
    h.view->close();
  }
  return cvcgl_test::finish("cvcgl_ariadne_view_stacking");
#endif
}
