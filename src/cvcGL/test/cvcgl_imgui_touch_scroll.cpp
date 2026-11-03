// Scrolling with a finger (cvc::gl::TouchScroll), on a bare ImGui context (no window, no GL), with
// the touches ImGuiOverlay hands ImGui in touch mode:
//  - the glide: a fling at v covers v / friction whatever the frame rate, and none slows it at 0;
//    a finger's speed is measured over its last 100 ms, so one held still before it lifts has none;
//  - a drag that starts on a button scrolls the window with the finger, and the button does not
//    fire; a finger that lifts while moving leaves the content gliding, four times as far at a
//    friction of 2 as at 8, until it stops;
//  - a press on content that glides stops it and is not a tap;
//  - a drag keeps its meaning on a slider, a text field being edited, a title bar and an item
//    that calls keepTouchDrag, while a field the press would only start editing is scrolled past;
//  - a drag in a child pane scrolls the pane, not its window;
//  - with touch_scroll off a drag scrolls nothing;
//  - UiSettings seeds ui.touch_scroll and ui.scroll_friction and hands its owner an edit of them.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/gl/Settings.h>
#include <cvc/gl/TouchScroll.h>
#include <cvc/gl/ariadne/ImGuiBackend.h> // setImGuiContext: cvcGL's imgui sees this context
#include <cvc/state/state.h>
#include <functional>
#include <string>

#ifdef CVC_ENABLE_IMGUI
#include <cfloat>
#include <imgui.h>
#include <imgui_internal.h> // ImGuiWindow: Scroll, ScrollMax
#endif

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

std::string num(double v) { return std::to_string(v); }

void glideAlone() {
  using cvc::gl::KineticScroll;
  // 1000 px/s at a friction of 2 covers 500 px, less the 10 px it would still cover under the
  // stop speed of 20 px/s: 490, at 60 or 144 frames a second.
  for (const float fps : {60.0f, 144.0f}) {
    KineticScroll k;
    k.fling(1000.0f);
    float d = 0.0f;
    int frames = 0;
    while (k.gliding() && frames < 100000) {
      d += k.step(1.0f / fps, 2.0f, 20.0f);
      ++frames;
    }
    chk(std::fabs(d - 490.0f) < 3.0f, "a 1000 px/s fling at friction 2 glides 490 px at " +
                                          num(fps) + " fps (got " + num(d) + ")");
  }
  KineticScroll k;
  k.fling(-1000.0f);
  float d = 0.0f;
  for (int i = 0; i < 60; ++i)
    d += k.step(1.0f / 60.0f, 0.0f, 20.0f);
  chk(std::fabs(d + 1000.0f) < 1.0f && k.gliding(),
      "with no friction it keeps its speed (1000 px up in a second, still gliding)");

  cvc::gl::VelocityTracker t;
  for (int i = 0; i <= 12; ++i)
    t.add(i / 60.0, 0.0f, 10.0f * i);
  float vx = 0.0f, vy = 0.0f;
  t.velocity(12 / 60.0, 0.1f, vx, vy);
  chk(std::fabs(vy - 600.0f) < 1.0f && vx == 0.0f,
      "10 px a frame at 60 fps is 600 px/s (got " + num(vy) + ")");
  for (int i = 13; i <= 24; ++i) // held still for 200 ms
    t.add(i / 60.0, 0.0f, 120.0f);
  t.velocity(24 / 60.0, 0.1f, vx, vy);
  chk(vy == 0.0f, "a finger held still before it lifts has no speed");
}

#ifdef CVC_ENABLE_IMGUI

constexpr float kDt = 1.0f / 60.0f;

struct Ui {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::gl::TouchScroll ts;
  std::function<void()> draw;
  Ui() {
    ImGui::SetCurrentContext(ctx);
    cvc::gl::ImGuiBackend::setImGuiContext(ctx); // TouchScroll runs in cvcGL, on its own imgui
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800.0f, 600.0f);
    io.IniFilename = nullptr;
    io.ConfigWindowsMoveFromTitleBarOnly = true; // as ImGuiOverlay sets for touch
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  }
  ~Ui() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame() {
    ImGui::GetIO().DeltaTime = kDt;
    ImGui::NewFrame();
    ts.beginFrame();
    draw();
    ts.endFrame();
    ImGui::Render();
  }
  void frames(int n) {
    for (int i = 0; i < n; ++i)
      frame();
  }
  // A finger, as ImGuiOverlay hands it on: down at a point, moved, lifted (then hovering nothing).
  void down(float x, float y) {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMousePosEvent(x, y);
    io.AddMouseButtonEvent(0, true);
    frame();
  }
  void move(float x, float y) {
    ImGui::GetIO().AddMousePosEvent(x, y);
    frame();
  }
  void up() {
    ImGuiIO &io = ImGui::GetIO();
    io.AddMouseButtonEvent(0, false);
    io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
    frame();
    frame(); // the cleared position trickles into the next frame
  }
  // Drags from (x, y0) to (x, y1) in `steps` frames, and lifts after `hold` still frames.
  void drag(float x, float y0, float y1, int steps, int hold) {
    down(x, y0);
    for (int i = 1; i <= steps; ++i)
      move(x, y0 + (y1 - y0) * static_cast<float>(i) / static_cast<float>(steps));
    for (int i = 0; i < hold; ++i)
      frame();
    up();
  }
  ImGuiWindow *window(const char *name) const { return ImGui::FindWindowByName(name); }
  float scroll(const char *name) const {
    ImGuiWindow *w = window(name);
    return w ? w->Scroll.y : -1.0f;
  }
};

// A window of 100 buttons at (0, 0), 300 x 300, so it scrolls ~2000 px; `top` draws first.
struct List {
  int clicks = 0;
  std::function<void()> top;
  void operator()() {
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300.0f, 300.0f), ImGuiCond_Always);
    ImGui::Begin("List");
    if (top)
      top();
    for (int i = 0; i < 100; ++i)
      if (ImGui::Button(("Item " + std::to_string(i)).c_str(), ImVec2(-1.0f, 0.0f)))
        ++clicks;
    ImGui::End();
  }
};

void dragAndFling() {
  Ui ui;
  List list;
  ui.draw = [&] { list(); };
  ui.frames(3);
  chk(ui.window("List") && ui.window("List")->ScrollMax.y > 1000.0f, "the list scrolls");

  // A drag from a button: once past the 6 px threshold the content follows the finger 1:1, from
  // where the finger was then (ImGui hands a touch press over a frame late), and the button does
  // not fire.
  ui.down(150.0f, 250.0f);
  float at4 = 0.0f;
  for (int i = 1; i <= 10; ++i) {
    ui.move(150.0f, 250.0f - 15.0f * static_cast<float>(i));
    if (i == 4)
      at4 = ui.scroll("List");
  }
  ui.frame(); // the last move lands
  chk(std::fabs(ui.scroll("List") - at4 - 90.0f) <= 1.0f,
      "past the threshold the content follows the finger: 90 px for its last 90 (got " +
          num(ui.scroll("List") - at4) + ")");
  chk(ui.scroll("List") > 100.0f && ui.scroll("List") <= 150.0f,
      "a 150 px drag up from a button scrolls it all, less the part before it was a scroll (" +
          num(ui.scroll("List")) + ")");
  ui.frames(10);
  ui.up();
  chk(list.clicks == 0, "the button the drag began on did not fire");
  const float rest = ui.scroll("List");
  ui.frames(30);
  chk(ui.scroll("List") == rest && !ui.ts.gliding(),
      "a finger held still before it lifts leaves no glide");

  // A tap is still a tap.
  ui.down(150.0f, 200.0f);
  ui.up();
  chk(list.clicks == 1, "a tap on a button fires it");

  // Flings: 30 px a frame, lifted at speed. The glide at friction 8 is a quarter of that at 2.
  float glide[2] = {0.0f, 0.0f};
  const float friction[2] = {2.0f, 8.0f};
  for (int i = 0; i < 2; ++i) {
    ui.ts.setSettings({true, friction[i]});
    ui.down(150.0f, 280.0f);
    for (int s = 1; s <= 6; ++s)
      ui.move(150.0f, 280.0f - 30.0f * static_cast<float>(s));
    ImGuiIO &io = ImGui::GetIO();
    io.AddMouseButtonEvent(0, false);
    io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
    ui.frame(); // lifted: the glide begins
    const float at = ui.scroll("List");
    chk(ui.ts.gliding(),
        "a finger lifted at speed leaves the content gliding (friction " + num(friction[i]) + ")");
    int n = 0;
    while (ui.ts.gliding() && n < 1000) {
      ui.frame();
      ++n;
    }
    ui.frame(); // the last asked scroll lands
    glide[i] = ui.scroll("List") - at;
    chk(!ui.ts.gliding() && n < 1000, "the glide stops by itself (" + num(n) + " frames)");
    // Back to the top for the next one.
    ImGui::SetScrollY(ui.window("List"), 0.0f);
    ui.frames(2);
  }
  chk(glide[0] > 400.0f, "a fling at friction 2 glides on (" + num(glide[0]) + " px)");
  const float ratio = glide[1] / glide[0];
  chk(ratio > 0.2f && ratio < 0.3f,
      "at friction 8 it glides a quarter as far as at 2 (" + num(ratio) + ")");
  chk(list.clicks == 1, "no fling fired a button");

  // A press on content gliding at speed stops it, and is not a tap on what it brought under the
  // finger.
  ui.ts.setSettings({true, 2.0f});
  ui.down(150.0f, 280.0f);
  for (int s = 1; s <= 6; ++s)
    ui.move(150.0f, 280.0f - 30.0f * static_cast<float>(s));
  ImGuiIO &io = ImGui::GetIO();
  io.AddMouseButtonEvent(0, false);
  io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
  ui.frames(6);
  chk(ui.ts.gliding(), "gliding");
  ui.down(150.0f, 150.0f);
  const float caught = ui.scroll("List");
  ui.up();
  ui.frames(10);
  chk(!ui.ts.gliding() && std::fabs(ui.scroll("List") - caught) <= 1.0f,
      "a press stops the glide where it is");
  chk(list.clicks == 1, "the press that stopped it was not a tap");
}

void dragsThatStayTheWidgets() {
  // A slider at the top: a drag that begins on it moves the slider, never the list.
  {
    Ui ui;
    List list;
    float value = 0.5f;
    list.top = [&] {
      ImGui::SetNextItemWidth(-1.0f);
      ImGui::SliderFloat("##s", &value, 0.0f, 1.0f);
    };
    ui.draw = [&] { list(); };
    ui.frames(3);
    const ImGuiWindow *w = ui.window("List");
    const float y = w->Pos.y + w->TitleBarHeight + ImGui::GetStyle().WindowPadding.y + 8.0f;
    ui.drag(150.0f, y, y + 120.0f, 8, 2);
    chk(ui.scroll("List") == 0.0f, "a drag from a slider leaves the list where it was");
  }
  // A title bar: the window's.
  {
    Ui ui;
    List list;
    ui.draw = [&] { list(); };
    ui.frames(3);
    ui.drag(150.0f, 8.0f, 108.0f, 8, 2);
    chk(ui.scroll("List") == 0.0f, "a drag on the title bar scrolls nothing");
  }
  // An item that keeps its drag, beside one that does not.
  for (const bool keep : {true, false}) {
    Ui ui;
    List list;
    list.top = [&] {
      ImGui::InvisibleButton("##canvas", ImVec2(260.0f, 120.0f));
      if (keep)
        cvc::gl::keepTouchDrag();
    };
    ui.draw = [&] { list(); };
    ui.frames(3);
    ui.drag(150.0f, 100.0f, 40.0f, 6, 2);
    if (keep)
      chk(ui.scroll("List") == 0.0f, "a drag on an item that calls keepTouchDrag is its own");
    else
      chk(ui.scroll("List") > 30.0f, "the same item without keepTouchDrag scrolls with the drag");
  }
  // A text field: scrolled past when the press would only start editing it, its own once it is
  // being edited.
  {
    Ui ui;
    List list;
    char buf[64] = "hello";
    list.top = [&] {
      ImGui::SetNextItemWidth(-1.0f);
      ImGui::InputText("##f", buf, sizeof buf);
    };
    ui.draw = [&] { list(); };
    ui.frames(3);
    const ImGuiWindow *w = ui.window("List");
    const float y = w->Pos.y + w->TitleBarHeight + ImGui::GetStyle().WindowPadding.y + 8.0f;
    const ImGuiID field = ImGui::GetIDWithSeed("##f", nullptr, w->ID);
    ui.drag(150.0f, y, y - 60.0f, 6, 10); // held still before it lifts: no glide
    chk(ui.scroll("List") > 30.0f, "a drag that begins on a field not being edited scrolls");
    chk(GImGui->ActiveId != field, "and leaves the field unedited");
    ImGui::SetScrollY(ui.window("List"), 0.0f);
    ui.frames(2);
    ui.down(150.0f, y); // a tap: editing
    ui.up();
    chk(GImGui->ActiveId == field, "a tap on the field edits it");
    ui.drag(150.0f, y, y - 60.0f, 6, 10);
    chk(ui.scroll("List") == 0.0f, "a drag in the field being edited is the field's");
  }
}

void childPanes() {
  Ui ui;
  ui.draw = [&] {
    ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(300.0f, 300.0f), ImGuiCond_Always);
    ImGui::Begin("Outer");
    ImGui::BeginChild("pane", ImVec2(0.0f, 120.0f), ImGuiChildFlags_Borders);
    for (int i = 0; i < 40; ++i)
      ImGui::TextUnformatted(("line " + std::to_string(i)).c_str());
    ImGui::EndChild();
    for (int i = 0; i < 40; ++i)
      ImGui::TextUnformatted(("row " + std::to_string(i)).c_str());
    ImGui::End();
  };
  ui.frames(3);
  ImGuiWindow *outer = ui.window("Outer");
  ImGuiWindow *pane =
      outer ? outer->DC.ChildWindows.empty() ? nullptr : outer->DC.ChildWindows[0] : nullptr;
  chk(pane && pane->ScrollMax.y > 0.0f && outer->ScrollMax.y > 0.0f,
      "both the pane and its window scroll");
  if (!pane)
    return;
  const float y = pane->Pos.y + 100.0f;
  ui.drag(150.0f, y, y - 60.0f, 6, 2);
  chk(pane->Scroll.y > 30.0f && outer->Scroll.y == 0.0f,
      "a drag in the pane scrolls the pane, not its window");
  const float below = pane->Pos.y + pane->Size.y + 60.0f;
  ui.drag(150.0f, below, below - 60.0f, 6, 2);
  chk(outer->Scroll.y > 30.0f, "a drag below it scrolls the window");
}

void off() {
  Ui ui;
  List list;
  ui.draw = [&] { list(); };
  ui.ts.setSettings({false, 2.0f});
  ui.frames(3);
  ui.drag(150.0f, 250.0f, 100.0f, 10, 2);
  chk(ui.scroll("List") == 0.0f, "with touch_scroll off a drag scrolls nothing");
  cvc::gl::TouchScroll t;
  t.setSettings({true, -3.0f});
  chk(t.settings().friction == 0.0f, "a negative friction is none");
  t.setSettings({true, NAN});
  chk(t.settings().friction == 2.0f, "a friction that is not a number is the default");
}

#endif // CVC_ENABLE_IMGUI

void settingsInState() {
  cvc::app app;
  using cvc::gl::UiSettings;
  int applied = 0;
  UiSettings::Values last;
  UiSettings ui(app, "ui.touch_scroll_test", [&](UiSettings::Values v) {
    ++applied;
    last = v;
  });
  chk(cvc::state::instance(app)("ui.touch_scroll_test.touch_scroll").value<int>() == 1,
      "ui.touch_scroll is seeded on");
  chk(cvc::state::instance(app)("ui.touch_scroll_test.scroll_friction").value<double>() == 2.0,
      "ui.scroll_friction is seeded at 2");
  chk(applied == 0, "the seed is not handed to the owner");
  cvc::state::instance(app)("ui.touch_scroll_test.scroll_friction").value(6.5);
  chk(applied > 0 && last.scrollFriction == 6.5, "an edit of the friction reaches the owner");
  cvc::state::instance(app)("ui.touch_scroll_test.touch_scroll").value(0);
  chk(!last.touchScroll, "and so does turning it off");
}

} // namespace

int main() {
  glideAlone();
#ifdef CVC_ENABLE_IMGUI
  dragAndFling();
  dragsThatStayTheWidgets();
  childPanes();
  off();
#else
  std::printf("skip: cvcGL built without imgui\n");
#endif
  settingsInState();
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
}
