// ImGuiBackend's §3.0.3b window sizing on a bare ImGui context (no GL): a window with no size fits
// its content when it first appears (it used to stay at ImGui's 32 px minimum, because §11.4
// persisted the size ImGui had before it measured and seeded it back), within the viewport, and
// scrolls when its content does not fit. Windows with no position flow down the left edge instead
// of stacking. An authored size and a persisted geometry still win.
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow: a window's rect, content size and scroll range
#endif

namespace ari = cvc::ariadne;

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

#ifdef CVC_ENABLE_IMGUI
// What a test reads back about one window after the last frame.
struct Win {
  bool found = false;
  float x = 0, y = 0, w = 0, h = 0;
  bool scroll_x = false, scroll_y = false; // a scrollbar is showing
  float scroll_max_x = 0, scroll_max_y = 0;
  std::string geometry; // the persisted tree.<id>.geometry
  std::string str() const {
    std::ostringstream os;
    os << "[" << x << "," << y << " " << w << "x" << h << (scroll_x ? " sx" : "")
       << (scroll_y ? " sy" : "") << " geom=" << geometry << "]";
    return os.str();
  }
};

// The stage_lighting component's shape: a heading, ten labeled sliders, a separator.
ari::Widget lighting(const std::string &id) {
  std::vector<ari::Widget> kids{ari::text("Key light")};
  const char *labels[] = {"Intensity", "Azimuth", "Elevation", "Cone",        "Fill",
                          "Back",      "Ambient", "Warmth",    "Environment", "Stage radius"};
  for (const char *l : labels)
    kids.push_back(ari::slider_float(l, std::string("lighting.") + l, 0.0, 1.0));
  kids.push_back(ari::separator());
  kids.push_back(ari::text("Fill / back / ambient"));
  ari::Widget w = ari::window("Stage lighting", kids);
  w.id = id;
  return w;
}

ari::Widget camera(const std::string &id) {
  ari::Widget w =
      ari::window("Camera", {ari::slider_float("Look sensitivity", "cam.look", 0.02, 2.0),
                             ari::slider_float("Move speed", "cam.move", 1.0, 100.0),
                             ari::slider_float("Field of view", "cam.fov", 20.0, 120.0),
                             ari::separator(), ari::checkbox("Invert pitch", "cam.invert"),
                             ari::checkbox("Drag pans", "cam.pans"),
                             ari::combo("Projection", "cam.proj", {"perspective", "orthographic"}),
                             ari::color_widget("Background", "cam.bg")});
  w.id = id;
  return w;
}

ari::Widget volren(const std::string &id) {
  ari::Widget w =
      ari::window("Volume raycast", {ari::slider_int("Steps", "vr.steps", 64, 1024),
                                     ari::slider_float("Ambient", "vr.ambient", 0, 1),
                                     ari::slider_int("Supersample (AA)", "vr.ss", 1, 4),
                                     ari::checkbox("Volumetric shadows", "vr.shadows")});
  w.id = id;
  return w;
}

ari::Extent px(float v) { return ari::Extent{ari::Unit::Px, v}; }

// Run `root` for `frames` frames on a fresh `dw` x `dh` context. `setup` may preset state before
// the first frame; `each` runs before every frame (frame index). Returns every window in `ids`.
std::map<std::string, Win> run(const ari::Widget &root, const std::vector<std::string> &ids,
                               float dw = 1400.0f, float dh = 820.0f, int frames = 6,
                               const std::function<void(cvc::app &)> &setup = {},
                               const std::function<void(cvc::app &, int)> &each = {}) {
  ImGuiContext *ctx = ImGui::CreateContext();
  // libcvcGL may carry its own copy of imgui (a static imgui, as on Windows): hand it this context.
  cvc::gl::ImGuiBackend::setImGuiContext(ctx);
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(dw, dh);
  io.IniFilename = nullptr;
  unsigned char *pixels = nullptr;
  int tw = 0, th = 0;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &tw, &th); // builds the font atlas; no renderer needed
  std::map<std::string, Win> out;
  {
    cvc::app app;
    ari::Runtime rt(app, "ui");
    cvc::gl::ImGuiBackend be;
    rt.set_backend(&be);
    if (setup)
      setup(app);
    rt.set_root(root);
    for (int f = 0; f < frames; ++f) {
      if (each)
        each(app, f);
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      rt.render();
      ImGui::Render();
    }
    for (const std::string &id : ids) {
      Win w;
      for (ImGuiWindow *iw : GImGui->Windows) {
        // Ariadne names a window "title###id"; match on the stable id part.
        const std::string n = iw->Name;
        const std::size_t at = n.find("###");
        if (at == std::string::npos || n.substr(at + 3) != id)
          continue;
        w.found = iw->WasActive;
        w.x = iw->Pos.x;
        w.y = iw->Pos.y;
        w.w = iw->Size.x;
        w.h = iw->Size.y;
        w.scroll_x = iw->ScrollbarX;
        w.scroll_y = iw->ScrollbarY;
        w.scroll_max_x = iw->ScrollMax.x;
        w.scroll_max_y = iw->ScrollMax.y;
      }
      w.geometry = cvc::state::instance(app)("ui.tree." + id + ".geometry").value();
      out[id] = w;
    }
  }
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
  ImGui::DestroyContext(ctx);
  return out;
}

bool overlaps(const Win &a, const Win &b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

bool on_screen(const Win &w, float dw, float dh) {
  return w.x >= 0 && w.y >= 0 && w.x + w.w <= dw + 0.5f && w.y + w.h <= dh + 0.5f;
}

std::string geom_of(const Win &w) {
  std::ostringstream os;
  os << static_cast<long>(w.x + 0.5f) << "," << static_cast<long>(w.y + 0.5f) << ","
     << static_cast<long>(w.w + 0.5f) << "," << static_cast<long>(w.h + 0.5f);
  return os.str();
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  // The repro (volren_bunny.ari's three component windows, none with a pos: or a size:): each one
  // opens at its content's size, shows all of it, and none hides another.
  {
    const auto r = run(ari::group({lighting("lighting"), camera("camera"), volren("volren")}),
                       {"lighting", "camera", "volren"});
    for (const auto &kv : r) {
      const Win &w = kv.second;
      chk(w.found, kv.first + ": drawn");
      chk(w.w > 200 && w.h > 100,
          kv.first + ": opens at its content's size, not a stub " + w.str());
      chk(w.scroll_max_x == 0 && w.scroll_max_y == 0,
          kv.first + ": shows all of its content (nothing to scroll) " + w.str());
      chk(on_screen(w, 1400, 820), kv.first + ": inside the viewport " + w.str());
      chk(w.geometry == geom_of(w),
          kv.first + ": persists the size it fitted to (" + w.geometry + ")");
    }
    chk(!overlaps(r.at("lighting"), r.at("camera")) &&
            !overlaps(r.at("lighting"), r.at("volren")) &&
            !overlaps(r.at("camera"), r.at("volren")),
        "windows with no pos: do not stack on each other");
  }

  // A persisted geometry still wins over the fit.
  {
    const auto r = run(ari::group({lighting("W")}), {"W"}, 1400, 820, 6, [](cvc::app &app) {
      cvc::state::instance(app)("ui.tree.W.geometry").value(std::string("100,120,300,200"));
    });
    const Win &w = r.at("W");
    chk(w.x == 100 && w.y == 120 && w.w == 300 && w.h == 200,
        "a persisted geometry places and sizes the window " + w.str());
    chk(w.scroll_y, "and content taller than it scrolls " + w.str());
  }

  // An authored size wins too, and content that does not fit it scrolls, both ways.
  {
    ari::Widget win = lighting("S");
    win.size.w = px(180);
    win.size.h = px(140);
    win.children.push_back(ari::text("a line of text much wider than the window it is drawn in"));
    const Win w = run(ari::group({win}), {"S"}).at("S");
    chk(w.w == 180 && w.h == 140, "an authored size: sizes the window " + w.str());
    chk(w.scroll_y && w.scroll_max_y > 0, "taller content scrolls vertically " + w.str());
    chk(w.scroll_x && w.scroll_max_x > 0, "wider content scrolls horizontally " + w.str());
  }

  // A viewport too small for the content: the window fits the viewport and scrolls.
  {
    const Win w = run(ari::group({lighting("T")}), {"T"}, 420, 200).at("T");
    chk(on_screen(w, 420, 200), "a window fits a small viewport " + w.str());
    chk(w.h > 100, "using the room it has " + w.str());
    chk(w.scroll_y, "and scrolls the rest " + w.str());
    chk(w.scroll_max_x == 0, "its scrollbar does not cover the content's width " + w.str());
  }

  // An authored pos: with no size: the window opens there, at its content's size.
  {
    ari::Widget win = camera("P");
    win.has_pos = true;
    win.pos_x = 500;
    win.pos_y = 300;
    const Win w = run(ari::group({win}), {"P"}).at("P");
    chk(w.x == 500 && w.y == 300, "an authored pos: places a fitted window " + w.str());
    chk(w.w > 200 && w.scroll_max_x == 0 && w.scroll_max_y == 0,
        "which fits its content " + w.str());
  }

  // Windows that first appear later (closable, opened at frames 8 and 11) fit too, and take the
  // next places in the flow, after the windows already open, though they come first in the
  // document.
  {
    ari::Widget later = volren("later");
    ari::Widget later2 = camera("later2");
    later.closable = later2.closable = true;
    auto open = [](cvc::app &app, const char *id, const char *v) {
      cvc::state::instance(app)(std::string("ui.tree.") + id + ".open").value(std::string(v));
    };
    const auto r = run(
        ari::group({later2, later, lighting("first")}), {"first", "later", "later2"}, 1400, 820, 16,
        [&](cvc::app &app) {
          open(app, "later", "0");
          open(app, "later2", "0");
        },
        [&](cvc::app &app, int f) {
          if (f == 8)
            open(app, "later", "1");
          if (f == 11)
            open(app, "later2", "1");
        });
    const Win &a = r.at("first"), &b = r.at("later"), &c = r.at("later2");
    chk(b.found && b.w > 200 && b.scroll_max_y == 0, "a window opened later fits " + b.str());
    chk(c.found && c.w > 200 && c.scroll_max_y == 0, "so does one opened after it " + c.str());
    chk(!overlaps(a, b) && !overlaps(a, c) && !overlaps(b, c),
        "and none lands on another " + a.str() + " " + b.str() + " " + c.str());
  }

  // A row split in a window with no size: the panes keep their heights and get a usable width.
  {
    ari::Widget g = ari::group({ari::slider_float("Top slider", "s.top", 0, 1),
                                ari::slider_float("Bottom slider", "s.bottom", 0, 1)});
    g.id = "split";
    g.layout.kind = ari::LayoutKind::Grid;
    g.layout.resizable = true;
    g.layout.row_heights = {ari::Track{ari::Unit::Px, 60.0f}, ari::Track{ari::Unit::Px, 90.0f}};
    ari::Widget win = ari::window("Split", {g});
    win.id = "SW";
    const Win w = run(ari::group({win}), {"SW"}).at("SW");
    chk(w.w > 250, "a row split is as wide as a fitted slider row " + w.str());
    chk(w.h >= 150, "and as tall as its panes " + w.str());
    chk(w.scroll_max_y == 0, "with nothing to scroll " + w.str());
  }

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
