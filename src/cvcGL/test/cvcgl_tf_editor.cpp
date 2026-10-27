/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ImGuiBackend::tf_editor on a bare ImGui context (no window, no GL), driven through an Ariadne
// Runtime by synthesized mouse and key events, checking what lands in cvc::state: node drags
// (committed every frame, or once on release with interactive_updates: false), pinned endpoints,
// an outside write cancelling a drag, the zoom strip, double-click add, Delete, the context menu,
// the colour picker's OK / Cancel, Reset, .vinay export / import, two independent editors,
// drawing with every overlay published, and a mesher's interval shaded between its bars (read
// back from the frame's draw list and logged text).
//
// Pointer targets come from the cvc::tf mapping the widget itself uses, over the canvas rect the
// widget leaves as ImGui's last item (the zoom strip sits one item spacing above it). The pointer
// moves in whole pixels, like a real one.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <cvc/tf/editor.h>
#include <cvc/tf/model.h>
#include <cvc/tf/overlays.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#define IMGUI_DEFINE_MATH_OPERATORS // must precede imgui.h (imgui_internal asserts it)
#include <imgui.h>
#include <imgui_internal.h> // the open popup's window and the hovered id, to pick menu items
#endif

namespace ari = cvc::ariadne;
namespace tf = cvc::tf;

namespace {

#ifdef CVC_ENABLE_IMGUI

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

bool close_to(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

ari::Value scalar(const std::string &s) {
  ari::Value v;
  v.scalar = s;
  return v;
}

// A tf_editor bound to `bind` whose view persists under tree.<id>.view.
ari::Widget editor(const std::string &bind, const std::string &id,
                   std::vector<std::pair<std::string, std::string>> props = {}) {
  ari::Widget w = ari::tf_editor("Transfer function " + id, bind);
  w.id = id;
  for (auto &p : props)
    w.props.entries.emplace_back(p.first, scalar(p.second));
  return w;
}

// A bare ImGui context, current here and in libcvcGL (which may carry its own copy of a static
// imgui). Declared first in Host so it outlives the runtime and the backend.
struct Context {
  ImGuiContext *ctx = nullptr;
  Context() {
    ctx = ImGui::CreateContext();
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1000.0f, 1000.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
  }
  ~Context() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr); // with a shared imgui this clears ours too
    ImGui::DestroyContext(ctx);
  }
  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;
};

// One app + Runtime + ImGuiBackend whose root renders into a pinned host window. After each
// frame, `canvas_min/max` hold the LAST tf_editor's canvas rect.
class Host {
public:
  Host() : rt(app, "ui") { rt.set_backend(&be); }

  void root(std::vector<ari::Widget> kids) {
    rt.set_root(ari::group(std::move(kids)));
    frame();
    frame(); // the host window exists (and is hoverable) from the second frame
  }

  void frame() {
    ImGuiIO &io = ImGui::GetIO();
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(720.0f, 990.0f), ImGuiCond_Always);
    ImGui::Begin("host", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar |
                     ImGuiWindowFlags_NoScrollWithMouse);
    if (log_text)
      ImGui::LogToBuffer(); // every text the frame submits, tooltips included
    rt.render();
    canvas_min = ImGui::GetItemRectMin();
    canvas_max = ImGui::GetItemRectMax();
    if (log_text) {
      text = GImGui->LogBuffer.c_str();
      ImGui::LogFinish();
    }
    ImGui::End();
    ImGui::Render();
  }

  // Set to capture each frame's submitted text in `text`.
  bool log_text = false;
  std::string text;

  // The widget's geometry, for the cvc::tf mapping functions.
  tf::layout layout() const {
    tf::layout l;
    l.canvas_x = canvas_min.x;
    l.canvas_y = canvas_min.y;
    l.canvas_w = canvas_max.x - canvas_min.x;
    l.canvas_h = canvas_max.y - canvas_min.y;
    l.strip_x = l.canvas_x;
    l.strip_w = l.canvas_w;
    l.strip_h = 20.0f;
    l.strip_y = l.canvas_y - ImGui::GetStyle().ItemSpacing.y - l.strip_h;
    return l;
  }

  void move(float x, float y) {
    ImGui::GetIO().AddMousePosEvent(std::floor(x), std::floor(y));
    frame();
  }
  void button(int b, bool down) {
    ImGui::GetIO().AddMouseButtonEvent(b, down);
    frame();
  }
  void press() { button(0, true); }
  void release() { button(0, false); }
  void click(float x, float y) {
    move(x, y);
    press();
    release();
  }
  void key(ImGuiKey k) {
    ImGui::GetIO().AddKeyEvent(k, true);
    frame();
    ImGui::GetIO().AddKeyEvent(k, false);
    frame();
  }
  void type(const std::string &text) {
    ImGui::GetIO().AddInputCharactersUTF8(text.c_str());
    frame();
  }
  // Press at `from`, move to `to` in `steps` frames (each step well past the drag threshold), and
  // stay held; release() ends it.
  void drag_to(float fx, float fy, float tx, float ty, int steps) {
    move(fx, fy);
    press();
    for (int i = 1; i <= steps; ++i) {
      const float u = static_cast<float>(i) / static_cast<float>(steps);
      move(fx + (tx - fx) * u, fy + (ty - fy) * u);
    }
  }
  void right_click(float x, float y) {
    move(x, y);
    button(1, true);
    button(1, false);
    frame(); // a popup sizes itself on a hidden first frame
    frame();
  }
  bool popup_open() const { return GImGui->OpenPopupStack.Size > 0; }
  static ImGuiWindow *top_popup() {
    ImGuiContext &g = *GImGui;
    return g.OpenPopupStack.Size > 0 ? g.OpenPopupStack.back().Window : nullptr;
  }
  // The id of the item `label` submitted directly in the innermost open popup, or of the item
  // `child` inside its PushID(`label`) scope (a picker's parts).
  static ImGuiID popup_id(const char *label, const char *child = nullptr) {
    ImGuiWindow *w = top_popup();
    if (!w)
      return 0;
    return child ? ImHashStr(child, 0, w->GetID(label)) : w->GetID(label);
  }
  // Click the item `id` of the innermost open popup: probe the popup's window (every 3 px down,
  // every 10 px across) until ImGui reports that item hovered. False when there is no popup or no
  // such (enabled) item.
  bool click_in_popup(ImGuiID id) {
    ImGuiWindow *w = top_popup();
    if (!w || id == 0)
      return false;
    frame(); // a popup fits its content (a new error line, say) a frame late
    // The window's own position and size each step: it may still be resizing.
    for (float dy = 2.0f; dy < w->Size.y - 1.0f; dy += 3.0f)
      for (float dx = 6.0f; dx < w->Size.x - 1.0f; dx += 10.0f) {
        move(w->Pos.x + dx, w->Pos.y + dy);
        if (GImGui->HoveredId == id) {
          press();
          release();
          frame();
          return true;
        }
      }
    return false;
  }
  bool click_menu_item(const char *label) { return click_in_popup(popup_id(label)); }

  void put(const std::string &path, const std::string &v) {
    cvc::state::instance(app)(path).value(v);
  }
  std::string get(const std::string &path) { return cvc::state::instance(app)(path).value(); }
  tf::model model(const std::string &path) {
    tf::model m;
    tf::from_string(get(path), m);
    return m;
  }
  // The persisted zoom window of view id `id` (0,1 when none was written).
  void zoom(const std::string &id, double &lo, double &hi) {
    lo = 0.0;
    hi = 1.0;
    tf::range_from_string(get("ui.tree." + id + ".view.zoom"), lo, hi);
  }

  ImVec2 canvas_min, canvas_max;

private:
  Context ctx;
  cvc::app app;
  cvc::gl::ImGuiBackend be;
  ari::Runtime rt;
};

// The default model: alpha (0,0) (0.25,0.75) (0.75,0.25) (1,1); colour at 0, 0.5, 0.75, 1.
const tf::model kSeed = tf::model::default_model();

// Pointer coordinates of an alpha node / an empty canvas point under the default view.
void at_alpha(const tf::layout &l, const tf::alpha_node &n, float &x, float &y) {
  const tf::view v;
  x = tf::x_of(l, v, n.at);
  y = tf::y_of(l, n.a);
}

void canvas_is_last_item(Host &h, float want_h) {
  const tf::layout l = h.layout();
  chk(l.canvas_w > 300.0f && close_to(l.canvas_h, want_h, 0.5),
      "the editor leaves its canvas as the last item (" + std::to_string(l.canvas_w) + " x " +
          std::to_string(l.canvas_h) + " px)");
}

void interactive_drag() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  canvas_is_last_item(h, 160.0f);
  const tf::layout l = h.layout();
  const tf::view v;
  float x, y;
  at_alpha(l, kSeed.alpha[1], x, y);
  h.drag_to(x, y, x + 60.0f, y + 30.0f, 3);
  const tf::model mid = h.model("ui.tf_a");
  chk(mid.alpha.size() == 4 && mid.alpha[1].at > kSeed.alpha[1].at + 0.03,
      "an interactive drag commits while the button is still held");
  h.move(x + 80.0f, y + 40.0f);
  h.release();
  const tf::model end = h.model("ui.tf_a");
  const double want_t = tf::t_of(l, v, x + 80.0f);
  const double want_a = tf::a_of(l, y + 40.0f);
  chk(end.alpha.size() == 4 && close_to(end.alpha[1].at, want_t, 2e-3) &&
          close_to(end.alpha[1].a, want_a, 2e-3),
      "the dragged interior node follows the pointer (at " +
          std::to_string(end.alpha.size() > 1 ? end.alpha[1].at : -1.0) + ", want " +
          std::to_string(want_t) + ")");
  chk(end.color.size() == kSeed.color.size() && end.alpha[2].at == kSeed.alpha[2].at,
      "the other nodes stay put");

  // A click with no motion selects without editing.
  const std::string before = h.get("ui.tf_a");
  at_alpha(l, end.alpha[2], x, y);
  h.click(x, y);
  chk(h.get("ui.tf_a") == before, "a click on a node edits nothing");
}

void delayed_drag() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a", {{"interactive_updates", "false"}})});
  const tf::layout l = h.layout();
  const tf::view v;
  const std::string seed = h.get("ui.tf_a");
  float x, y;
  at_alpha(l, kSeed.alpha[1], x, y);
  h.drag_to(x, y, x + 60.0f, y + 30.0f, 4);
  chk(h.get("ui.tf_a") == seed, "interactive_updates: false writes nothing mid-drag");
  h.release();
  const tf::model end = h.model("ui.tf_a");
  chk(end.alpha.size() == 4 && close_to(end.alpha[1].at, tf::t_of(l, v, x + 60.0f), 2e-3) &&
          close_to(end.alpha[1].a, tf::a_of(l, y + 30.0f), 2e-3),
      "...and commits the drag once, on release");

  // A write from outside during a drag (a script, another view) drops the drag.
  const std::string outside = tf::to_string(tf::model::preset("grayscale"));
  at_alpha(l, end.alpha[1], x, y);
  h.drag_to(x, y, x + 30.0f, y - 20.0f, 2);
  h.put("ui.tf_a", outside);
  h.move(x + 50.0f, y - 30.0f);
  h.move(x + 70.0f, y - 30.0f);
  h.release();
  chk(h.get("ui.tf_a") == outside, "an outside write mid-drag cancels the drag");
}

void pinned_endpoints() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  float x, y;
  at_alpha(l, kSeed.alpha[0], x, y); // (0, 0): the canvas's bottom-left corner
  h.drag_to(x + 2.0f, y - 2.0f, x + 82.0f, y - 42.0f, 3);
  h.release();
  const tf::model m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && m.alpha[0].at == 0.0 && m.alpha[0].a > 0.1f,
      "the first alpha node keeps t = 0 while its opacity drags");
  chk(m.color.front().at == 0.0 && m.color.back().at == 1.0, "the colour endpoints stay pinned");
}

void zoom_strip() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const std::string seed = h.get("ui.tf_a");
  // The low range bar sits on the strip's left edge.
  const float x = tf::strip_x_of(l, 0.0) + 1.0f, y = l.strip_y + 0.5f * l.strip_h;
  h.drag_to(x, y, x + 120.0f, y, 4);
  h.release();
  double lo = 0.0, hi = 1.0;
  h.zoom("ed_a", lo, hi);
  const double want = tf::strip_t_of(l, x + 120.0f - 1.0f);
  chk(close_to(lo, want, 2e-3) && hi == 1.0,
      "dragging the low range bar persists tree.ed_a.view.zoom (lo " + std::to_string(lo) +
          ", want " + std::to_string(want) + ")");
  chk(h.get("ui.tf_a") == seed, "the zoom is view state: the model is untouched");

  // The canvas now maps the zoom window: the node at t = 0.25 is under x_of with that view.
  tf::view v;
  v.zoom_lo = lo;
  v.zoom_hi = hi;
  const float nx = tf::x_of(l, v, kSeed.alpha[1].at), ny = tf::y_of(l, kSeed.alpha[1].a);
  h.drag_to(nx, ny, nx + 40.0f, ny, 3);
  h.release();
  const tf::model m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && close_to(m.alpha[1].at, tf::t_of(l, v, nx + 40.0f), 2e-3),
      "a drag on the zoomed canvas moves through the zoom window");
}

void double_click_and_delete() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  // Empty canvas: between the colour bars at 0.5 and 0.75, far from every alpha node. Whole pixels,
  // since the node lands exactly under the pointer.
  const float x = std::floor(tf::x_of(l, v, 0.6)), y = std::floor(tf::y_of(l, 0.9f));
  h.click(x, y);
  h.press();
  h.release();
  tf::model m = h.model("ui.tf_a");
  bool found = false;
  for (const tf::alpha_node &n : m.alpha)
    found =
        found || (close_to(n.at, tf::t_of(l, v, x), 2e-3) && close_to(n.a, tf::a_of(l, y), 2e-3));
  chk(m.alpha.size() == 5 && found, "a double-click on empty canvas adds an alpha node there");

  // Select the interior node at 0.75 and press Delete with the pointer over the canvas.
  int idx = -1;
  for (std::size_t i = 0; i < m.alpha.size(); ++i)
    if (m.alpha[i].at == 0.75)
      idx = static_cast<int>(i);
  float nx = 0.f, ny = 0.f;
  if (idx >= 0)
    at_alpha(l, m.alpha[static_cast<std::size_t>(idx)], nx, ny);
  h.click(nx, ny);
  h.key(ImGuiKey_Delete);
  m = h.model("ui.tf_a");
  bool gone = true;
  for (const tf::alpha_node &n : m.alpha)
    gone = gone && n.at != 0.75;
  chk(idx >= 0 && m.alpha.size() == 4 && gone, "Delete removes the selected interior node");

  // An endpoint is protected.
  at_alpha(l, m.alpha.front(), nx, ny);
  h.click(nx + 2.0f, ny - 2.0f);
  h.key(ImGuiKey_Delete);
  chk(h.model("ui.tf_a").alpha.size() == 4, "Delete leaves an endpoint in place");
}

void context_menu() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  const std::string seed = h.get("ui.tf_a");

  h.right_click(tf::x_of(l, v, 0.6), tf::y_of(l, 0.9f));
  chk(h.popup_open(), "a right-click on the canvas opens the context menu");
  chk(h.get("ui.tf_a") == seed, "opening the menu edits nothing");
  chk(h.click_menu_item("Opacity cubed") && h.model("ui.tf_a").opacity_cubed,
      "Opacity cubed toggles the model's flag");
  chk(!h.popup_open(), "choosing an item closes the menu");

  float x, y;
  at_alpha(l, kSeed.alpha[1], x, y);
  h.right_click(x, y);
  chk(h.click_menu_item("Delete") && h.model("ui.tf_a").alpha.size() == 3,
      "Delete in the menu removes the node under the right-click");

  // Dismissing the menu (a click outside it) changes nothing.
  const std::string before = h.get("ui.tf_a");
  h.right_click(tf::x_of(l, v, 0.4), tf::y_of(l, 0.5f));
  h.click(990.0f, 990.0f);
  h.frame();
  chk(!h.popup_open() && h.get("ui.tf_a") == before, "dismissing the menu edits nothing");
}

void colour_edit() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  const std::string seed = h.get("ui.tf_a");
  // The red colour bar at 0.5, half way up: no alpha node is near.
  const float x = tf::x_of(l, v, kSeed.color[1].at), y = l.canvas_y + 0.5f * l.canvas_h;

  h.right_click(x, y);
  chk(h.click_menu_item("Edit colour..."), "Edit colour... is offered on a colour bar");
  h.frame();
  h.frame(); // the picker opens once the menu has closed
  chk(h.click_in_popup(Host::popup_id("##picker", "sv")) &&
          h.click_in_popup(Host::popup_id("Cancel")) && h.get("ui.tf_a") == seed,
      "Cancel leaves the colour as it was (ColorTable2 turned it black)");

  h.right_click(x, y);
  h.click_menu_item("Edit colour...");
  h.frame();
  h.frame();
  const bool picked =
      h.click_in_popup(Host::popup_id("##picker", "sv")) && h.click_in_popup(Host::popup_id("OK"));
  const tf::model m = h.model("ui.tf_a");
  const tf::color_node &c = m.color[1];
  chk(picked && m.color.size() == kSeed.color.size() && (c.r != 1.f || c.g != 0.f || c.b != 0.f),
      "OK applies the picked colour to that node");
}

// The colour and opacity lists and the isocontour positions agree (a .vinay keeps no more).
bool same_look(const tf::model &a, const tf::model &b) {
  if (a.color.size() != b.color.size() || a.alpha.size() != b.alpha.size() ||
      a.isocontours.size() != b.isocontours.size())
    return false;
  for (std::size_t i = 0; i < a.color.size(); ++i)
    if (!close_to(a.color[i].at, b.color[i].at, 1e-9) ||
        !close_to(a.color[i].r, b.color[i].r, 1e-6) ||
        !close_to(a.color[i].g, b.color[i].g, 1e-6) || !close_to(a.color[i].b, b.color[i].b, 1e-6))
      return false;
  for (std::size_t i = 0; i < a.alpha.size(); ++i)
    if (!close_to(a.alpha[i].at, b.alpha[i].at, 1e-9) ||
        !close_to(a.alpha[i].a, b.alpha[i].a, 1e-6))
      return false;
  for (std::size_t i = 0; i < a.isocontours.size(); ++i)
    if (!close_to(a.isocontours[i].at, b.isocontours[i].at, 1e-9))
      return false;
  return true;
}

void vinay_files() {
  Host h;
  tf::model seed = tf::model::preset("hot");
  tf::add_isocontour(seed, 0.3);
  seed.normalize();
  h.put("ui.tf_a", tf::to_string(seed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  const std::string before = h.get("ui.tf_a");
  const std::string path =
      (std::filesystem::temp_directory_path() / "cvcgl_tf_editor_test.vinay").string();
  std::remove(path.c_str());
  const float x = tf::x_of(l, v, 0.55), y = tf::y_of(l, 0.85f);

  h.right_click(x, y);
  h.click_menu_item("Export .vinay...");
  h.frame();
  h.frame(); // the path field takes the keyboard as the popup appears
  h.type(path);
  const bool exported = h.click_in_popup(Host::popup_id("OK"));
  chk(exported && std::ifstream(path).good() && !h.popup_open(),
      "Export .vinay... writes the file and closes");
  chk(h.get("ui.tf_a") == before, "...without touching the model");

  h.right_click(x, y);
  chk(h.click_menu_item("Reset") && h.model("ui.tf_a") == tf::model::default_model(),
      "Reset restores ColorTable2's default transfer function");

  h.right_click(x, y);
  h.click_menu_item("Import .vinay..."); // the field still holds the exported path
  h.frame();
  h.frame();
  h.click_in_popup(Host::popup_id("OK"));
  chk(same_look(h.model("ui.tf_a"), seed) && !h.popup_open(),
      "Import .vinay... reads the exported look back");

  std::remove(path.c_str());
  const std::string imported = h.get("ui.tf_a");
  h.right_click(x, y);
  h.click_menu_item("Import .vinay...");
  h.frame();
  h.frame();
  const bool tried = h.click_in_popup(Host::popup_id("OK"));
  chk(tried && h.popup_open() && h.get("ui.tf_a") == imported,
      "a failed import keeps the popup open (its error inline) and the model as it was");
  chk(h.click_in_popup(Host::popup_id("Cancel")) && !h.popup_open(), "Cancel closes it");
}

void two_editors() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.put("ui.tf_b", tf::to_string(kSeed));
  const ari::Widget a = editor("tf_a", "ed_a"), b = editor("tf_b", "ed_b");
  h.root({a, b}); // the last item is b's canvas
  const std::string seed = h.get("ui.tf_a");
  tf::layout l = h.layout();
  float x, y;
  at_alpha(l, kSeed.alpha[1], x, y);
  h.drag_to(x, y, x + 50.0f, y, 3);
  h.release();
  const std::string b_after = h.get("ui.tf_b");
  chk(b_after != seed && h.get("ui.tf_a") == seed, "a drag in one editor edits only its model");

  h.root({b, a}); // now a's canvas is last
  l = h.layout();
  at_alpha(l, kSeed.alpha[1], x, y);
  h.drag_to(x, y, x + 30.0f, y + 20.0f, 3);
  h.release();
  chk(h.get("ui.tf_a") != seed && h.get("ui.tf_b") == b_after,
      "...and the other editor's drag edits only its own");

  const float sx = tf::strip_x_of(l, 1.0) - 1.0f, sy = l.strip_y + 0.5f * l.strip_h;
  h.drag_to(sx, sy, sx - 100.0f, sy, 3);
  h.release();
  double alo, ahi, blo, bhi;
  h.zoom("ed_a", alo, ahi);
  h.zoom("ed_b", blo, bhi);
  chk(ahi < 0.95 && blo == 0.0 && bhi == 1.0, "each editor keeps its own zoom window");
}

// Two editors on ONE model in one window (a document including the component twice): each keeps
// its own drag, so a drag in the second is not ended by the first.
void same_model_twice() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  const ari::Widget a = editor("tf_a", "ed_a"), b = editor("tf_a", "ed_a2");
  h.root({a});
  const tf::layout la = h.layout(); // the first editor stays where it is when a second follows
  h.root({a, b});
  const tf::layout lb = h.layout();
  const tf::view v;
  float x, y;
  at_alpha(la, kSeed.alpha[1], x, y);
  h.drag_to(x, y, x + 50.0f, y, 3);
  h.release();
  tf::model m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && close_to(m.alpha[1].at, tf::t_of(la, v, x + 50.0f), 2e-3) &&
          close_to(m.alpha[1].a, kSeed.alpha[1].a, 2e-3),
      "the first of two editors on one model drags on its own");
  at_alpha(lb, m.alpha[1], x, y);
  h.drag_to(x, y, x + 30.0f, y + 20.0f, 3);
  h.release();
  m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && close_to(m.alpha[1].at, tf::t_of(lb, v, x + 30.0f), 2e-3) &&
          close_to(m.alpha[1].a, tf::a_of(lb, y + 20.0f), 2e-3),
      "...and so does the second");
}

// Index of the colour / alpha node at `t` (to the string form's precision), or -1.
template <typename List> int node_at(const List &nodes, double t) {
  for (std::size_t i = 0; i < nodes.size(); ++i)
    if (close_to(nodes[i].at, t, 1e-7))
      return static_cast<int>(i);
  return -1;
}

bool same_rgb(const tf::color_node &a, const tf::color_node &b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

// A write from outside (a replicated peer, a script) inserts a node ahead of the one an open popup
// or the selection was taken on, so that node's index moves up one: the colour picker's OK, the
// menu's Delete and the Delete key must still act on THAT node, and on nothing once it is gone.
void outside_insert_keeps_targets() {
  Host h;
  const std::string seed = tf::to_string(kSeed);
  h.put("ui.tf_a", seed);
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  // The peer's model: the seed plus a colour and an alpha node at 0.1.
  tf::model peer = kSeed;
  tf::add_color(peer, 0.1);
  tf::add_alpha(peer, 0.1, 0.5f);
  peer.normalize();
  const int peer_c01 = node_at(peer.color, 0.1);

  // Edit colour... on the red bar at 0.5 (colour node 1); the peer inserts while the picker is up.
  const float cx = tf::x_of(l, v, kSeed.color[1].at), cy = l.canvas_y + 0.5f * l.canvas_h;
  h.right_click(cx, cy);
  h.click_menu_item("Edit colour...");
  h.frame();
  h.frame();
  h.put("ui.tf_a", tf::to_string(peer));
  h.frame();
  const bool picked =
      h.click_in_popup(Host::popup_id("##picker", "sv")) && h.click_in_popup(Host::popup_id("OK"));
  tf::model m = h.model("ui.tf_a");
  int c01 = node_at(m.color, 0.1), c05 = node_at(m.color, 0.5);
  chk(picked && peer_c01 >= 0 && m.color.size() == peer.color.size() && c01 >= 0 && c05 >= 0 &&
          same_rgb(m.color[static_cast<std::size_t>(c01)],
                   peer.color[static_cast<std::size_t>(peer_c01)]) &&
          !same_rgb(m.color[static_cast<std::size_t>(c05)], kSeed.color[1]),
      "the colour picker's OK recolours the node it was opened on after an outside insert");

  // The menu's Delete on the alpha node at 0.25 (alpha node 1).
  h.put("ui.tf_a", seed);
  h.frame();
  float x, y;
  at_alpha(l, kSeed.alpha[1], x, y);
  h.right_click(x, y);
  h.put("ui.tf_a", tf::to_string(peer));
  h.frame();
  const bool deleted = h.click_menu_item("Delete");
  m = h.model("ui.tf_a");
  chk(deleted && m.alpha.size() == 4 && node_at(m.alpha, 0.25) < 0 && node_at(m.alpha, 0.1) >= 0,
      "the menu's Delete removes the node it was opened on after an outside insert");

  // The Delete key on the selected alpha node at 0.75 (alpha node 2).
  h.put("ui.tf_a", seed);
  h.frame();
  at_alpha(l, kSeed.alpha[2], x, y);
  h.click(x, y);
  h.put("ui.tf_a", tf::to_string(peer));
  h.frame();
  h.key(ImGuiKey_Delete);
  m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && node_at(m.alpha, 0.75) < 0 && node_at(m.alpha, 0.25) >= 0 &&
          node_at(m.alpha, 0.1) >= 0,
      "the Delete key removes the selected node after an outside insert");

  // The peer removes the node the picker was opened on: OK has nothing left to recolour.
  h.put("ui.tf_a", seed);
  h.frame();
  tf::model gone = kSeed;
  gone.color.erase(gone.color.begin() + 1);
  gone.normalize();
  const std::string gone_s = tf::to_string(gone);
  h.right_click(cx, cy);
  h.click_menu_item("Edit colour...");
  h.frame();
  h.frame();
  h.put("ui.tf_a", gone_s);
  h.frame();
  h.click_in_popup(Host::popup_id("##picker", "sv"));
  h.click_in_popup(Host::popup_id("OK"));
  chk(h.get("ui.tf_a") == gone_s, "...and recolours nothing once the peer removed that node");
  h.click_in_popup(Host::popup_id("Cancel"));
  chk(!h.popup_open(), "Cancel still closes the picker");
}

// The editor's own Add (menu) inserts a node ahead of the selected one: the Delete key must still
// remove the node that was selected.
void own_insert_keeps_selection() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  float x, y;
  at_alpha(l, kSeed.alpha[2], x, y); // 0.75
  h.click(x, y);
  // Empty canvas at t = 0.1, high up: Add > Alpha node puts a node there (index 1).
  h.right_click(tf::x_of(l, v, 0.1), tf::y_of(l, 0.9f));
  const bool added = h.click_menu_item("Add") && h.click_menu_item("Alpha node");
  tf::model m = h.model("ui.tf_a");
  chk(added && m.alpha.size() == 5 && node_at(m.alpha, 0.75) >= 0,
      "Add > Alpha node adds a node ahead of the selected one");
  h.move(x, y);
  h.key(ImGuiKey_Delete);
  m = h.model("ui.tf_a");
  chk(m.alpha.size() == 4 && node_at(m.alpha, 0.75) < 0 && node_at(m.alpha, 0.25) >= 0,
      "...and the Delete key still removes the node that was selected");
}

// Two editors on different models whose instance ids fall back to the same "##" label (the hidden
// label idiom): their keys "<path>###tf" differ only before the "###", where ImGui's string hash
// restarts, so the editor must not hash the key as an ImGui label.
void hidden_label_ids() {
  Host h;
  h.put("ui.tf_a", tf::to_string(kSeed));
  h.put("ui.tf_b", tf::to_string(kSeed));
  const std::string seed = h.get("ui.tf_b");
  const ari::Widget a = ari::tf_editor("##tf", "tf_a"), b = ari::tf_editor("##tf", "tf_b");
  h.root({a});
  const tf::layout la = h.layout();
  h.root({a, b});
  const tf::view v;
  const float x = std::floor(tf::x_of(la, v, 0.6)), y = std::floor(tf::y_of(la, 0.9f));
  h.click(x, y);
  h.press();
  h.release();
  chk(h.model("ui.tf_a").alpha.size() == 5 && h.get("ui.tf_b") == seed,
      "a double-click in one of two '##tf' editors adds a node to its model only");
}

void overlays_and_isocontours() {
  Host h;
  tf::model m = kSeed;
  tf::isocontour bone;
  bone.id = "bone";
  bone.at = 0.4;
  tf::isocontour skin;
  skin.id = "skin";
  skin.at = 0.62;
  skin.has_rgb = true;
  skin.r = 1.0f;
  skin.g = 0.8f;
  skin.b = 0.6f;
  m.isocontours = {bone, skin};
  m.normalize();
  h.put("ui.tf_a", tf::to_string(m));

  std::vector<double> bins(256);
  for (std::size_t i = 0; i < bins.size(); ++i)
    bins[i] = i == 0 ? 50000.0 : 100.0 + 80.0 * std::sin(0.05 * static_cast<double>(i));
  tf::spectrum_overlay sp;
  for (int i = 0; i < 64; ++i) {
    const double s = i / 63.0;
    sp.area.push_back(s * (1.0 - s));
    sp.min_volume.push_back(s);
    sp.max_volume.push_back(2.0); // a constant function: nothing to normalize by
    sp.gradient.push_back(s * s);
  }
  tf::tree_overlay tree;
  tree.vertices = {{0.5f, 0.0}, {0.5f, 40.0}, {0.2f, 90.0}, {0.8f, 100.0}};
  tree.edges = {{0, 1}, {1, 2}, {1, 3}};
  h.put("ui.tf_a.range", tf::range_to_string(0.0, 100.0));
  h.put("ui.tf_a.histogram", tf::histogram_to_string(bins));
  h.put("ui.tf_a.contour_spectrum", tf::spectrum_to_string(sp));
  h.put("ui.tf_a.contour_tree", tf::tree_to_string(tree));
  h.put("ui.tf_a.overlay_status", "computing contour tree");
  for (const char *layer : {"histogram", "contour_spectrum", "contour_tree", "info"})
    h.put(std::string("ui.tree.ed_a.view.") + layer, "1");

  h.root({editor("tf_a", "ed_a", {{"height", "200"}})});
  canvas_is_last_item(h, 200.0f);
  const tf::layout l = h.layout();
  const tf::view v;
  const std::string seed = h.get("ui.tf_a");
  // Hover every kind of handle (tooltips, highlights) and the strip; flip layers between frames.
  h.move(tf::x_of(l, v, bone.at), l.canvas_y + 0.5f * l.canvas_h);
  h.move(tf::x_of(l, v, 0.5), l.canvas_y + 0.5f * l.canvas_h);
  float x, y;
  at_alpha(l, kSeed.alpha[2], x, y);
  h.move(x, y);
  h.move(tf::strip_x_of(l, 0.0) + 1.0f, l.strip_y + 0.5f * l.strip_h);
  h.put("ui.tree.ed_a.view.log_histogram", "1");
  h.frame();
  h.put("ui.tree.ed_a.view.color", "0");
  h.frame();
  h.put("ui.tree.ed_a.view.color", "1");
  h.put("ui.tf_a.contour_tree", "not json");
  h.frame();
  chk(h.get("ui.tf_a") == seed, "drawing every overlay (and a malformed one) edits nothing");

  // An isocontour bar drags anywhere and keeps its id.
  const float bx = tf::x_of(l, v, bone.at), by = l.canvas_y + 0.5f * l.canvas_h;
  h.drag_to(bx, by, bx + 50.0f, by, 3);
  h.release();
  const tf::model end = h.model("ui.tf_a");
  const int i = end.find_isocontour("bone");
  chk(i >= 0 && close_to(end.isocontours[static_cast<std::size_t>(i)].at,
                         tf::t_of(l, v, bx + 50.0f), 2e-3),
      "an isocontour bar drags to the pointer, keeping its id");
  chk(end.find_isocontour("skin") >= 0 && end.isocontours.size() == 2,
      "the other isocontour is untouched");
}

// The vertices of the last frame drawn in `c` (an exact ImU32) in the host window's draw list
// (kept until the window's next Begin): their x / y extent and the index of the first (-1 when
// none).
struct Quad {
  int count = 0, first = -1;
  float x0 = 1e30f, x1 = -1e30f, y0 = 1e30f, y1 = -1e30f;
};
Quad vertices_of(ImU32 c) {
  Quad q;
  const ImGuiWindow *w = ImGui::FindWindowByName("host");
  if (const ImDrawList *dl = w ? w->DrawList : nullptr) {
    for (int i = 0; i < dl->VtxBuffer.Size; ++i) {
      const ImDrawVert &v = dl->VtxBuffer[i];
      if (v.col != c)
        continue;
      if (q.first < 0)
        q.first = i;
      ++q.count;
      q.x0 = std::min(q.x0, v.pos.x);
      q.x1 = std::max(q.x1, v.pos.x);
      q.y0 = std::min(q.y0, v.pos.y);
      q.y1 = std::max(q.y1, v.pos.y);
    }
  }
  return q;
}

// fe_lab's meshed interval (<tf>.interval): the band between its two bars, translucent magenta
// under the bars and curves, following the zoom and a dragged bar; none when an id is missing or
// the isocontours are hidden; the info table and the tooltip mark the bounds.
void interval_band() {
  Host h;
  tf::model m = kSeed;
  tf::isocontour a, b, c;
  a.id = "skin";
  a.at = 0.3;
  b.id = "bone";
  b.at = 0.6;
  c.id = "lung";
  c.at = 0.85;
  m.isocontours = {a, b, c};
  m.normalize();
  h.put("ui.tf_a", tf::to_string(m));
  h.put("ui.tf_a.interval", "bone,skin"); // isovalue's bar first: either order
  h.root({editor("tf_a", "ed_a")});
  const tf::layout l = h.layout();
  const tf::view v;
  const ImU32 band = ImGui::GetColorU32(ImVec4(1.f, 0.f, 1.f, tf::kIntervalAlpha));
  const ImU32 bar = ImGui::GetColorU32(ImVec4(1.f, 0.f, 1.f, 1.f));
  Quad q = vertices_of(band);
  const float xa = tf::x_of(l, v, a.at), xb = tf::x_of(l, v, b.at);
  chk(q.count == 4 && close_to(q.x0, xa, 0.5) && close_to(q.x1, xb, 0.5) &&
          close_to(q.y0, l.canvas_y, 0.5) && close_to(q.y1, l.canvas_y + l.canvas_h, 0.5),
      "the band spans the canvas height between the two bars (x " + std::to_string(q.x0) + ".." +
          std::to_string(q.x1) + ", want " + std::to_string(xa) + ".." + std::to_string(xb) + ")");
  const Quad bars = vertices_of(bar);
  chk(q.first >= 0 && bars.first > q.first, "...drawn under the isocontour bars (and the curves)");

  // A bar dragged this frame takes the band with it.
  const float y = l.canvas_y + 0.5f * l.canvas_h;
  h.drag_to(xb, y, xb + 40.0f, y, 3);
  q = vertices_of(band);
  const tf::model moved = h.model("ui.tf_a");
  const int ib = moved.find_isocontour("bone");
  const float xb2 = ib < 0 ? -1.f : tf::x_of(l, v, moved.isocontours[std::size_t(ib)].at);
  chk(q.count == 4 && close_to(q.x1, xb2, 0.5) && xb2 > xb + 30.0f,
      "dragging a bound's bar moves the band's edge with it");
  h.release();

  // The tooltip and the info table name the bounds.
  h.log_text = true;
  h.put("ui.tree.ed_a.view.info", "1");
  h.frame();
  chk(h.text.find("skin (interval lo)") != std::string::npos &&
          h.text.find("bone (interval hi)") != std::string::npos &&
          h.text.find("lung (interval") == std::string::npos,
      "the information table marks the lower and upper bounds");
  h.move(xa, y);
  for (int i = 0; i < 20; ++i) // the tooltip waits for the pointer to settle
    h.frame();
  chk(h.text.find("lower bound of the meshed interval") != std::string::npos,
      "hovering a bound's bar says which bound it is");
  h.log_text = false;
  h.put("ui.tree.ed_a.view.info", "0");

  // Through the zoom window: [0.5, 1] puts skin's bar off the left edge.
  h.put("ui.tree.ed_a.view.zoom", tf::range_to_string(0.5, 1.0));
  h.frame();
  tf::view z;
  z.zoom_lo = 0.5;
  q = vertices_of(band);
  chk(q.count == 4 && q.x0 <= l.canvas_x && q.x0 >= l.canvas_x - 1.5f &&
          close_to(q.x1, tf::x_of(l, z, moved.isocontours[std::size_t(ib)].at), 0.5),
      "the band follows the zoom window (clamped at the canvas edge)");
  h.put("ui.tree.ed_a.view.zoom", tf::range_to_string(0.0, 1.0));

  // No band: an id the model lacks, an empty interval, hidden isocontours.
  h.put("ui.tf_a.interval", "bone,gone");
  h.frame();
  chk(vertices_of(band).count == 0, "no band when an id is not in the model");
  h.put("ui.tf_a.interval", "");
  h.frame();
  chk(vertices_of(band).count == 0, "no band when nothing is published");
  h.put("ui.tf_a.interval", "skin,bone");
  h.put("ui.tree.ed_a.view.isocontours", "0");
  h.frame();
  chk(vertices_of(band).count == 0, "no band while the isocontours are hidden");
  h.put("ui.tree.ed_a.view.isocontours", "1");
  h.frame();
  chk(vertices_of(band).count == 4, "...and back with them");
}

#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  interactive_drag();
  delayed_drag();
  pinned_endpoints();
  zoom_strip();
  double_click_and_delete();
  context_menu();
  colour_edit();
  vinay_files();
  two_editors();
  same_model_twice();
  outside_insert_keeps_targets();
  own_insert_keeps_selection();
  hidden_label_ids();
  overlays_and_isocontours();
  interval_band();
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
