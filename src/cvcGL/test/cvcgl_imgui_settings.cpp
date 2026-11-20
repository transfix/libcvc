// ImGuiBackend's settings editor (Kind::Settings) on a bare ImGui context (no GL): dragging a
// key's slider writes the key as it moves and runs on: once, at the release; the key's reset then
// shows, and a click on it puts its default back; a text field is applied on Enter, not when it
// loses focus; and each row has the editor its metadata asks for.
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <map>
#include <string>
#include <utility>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
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
// The ImGui backend, keeping each row's editor and reset rects from the last frame.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  struct Row {
    ImVec2 e0, e1, r0, r1;
    ari::SettingKind kind = ari::SettingKind::Text;
    bool has_reset() const { return r1.x > r0.x; }
    ImVec2 editor_at(float f) const {
      return ImVec2(e0.x + f * (e1.x - e0.x), 0.5f * (e0.y + e1.y));
    }
    ImVec2 reset_mid() const { return ImVec2(0.5f * (r0.x + r1.x), 0.5f * (r0.y + r1.y)); }
  };
  std::map<std::string, Row> rows;
  ari::SettingsEdit settings(const char *label, const char *id,
                             const ari::SettingsSpec &spec) override {
    rows.clear();
    return ImGuiBackend::settings(label, id, spec);
  }

protected:
  void settings_row_drawn(const ari::SettingsNode &n, const float e[4], const float r[4]) override {
    Row &row = rows[n.id];
    row.e0 = ImVec2(e[0], e[1]);
    row.e1 = ImVec2(e[2], e[3]);
    row.r0 = ImVec2(r[0], r[1]);
    row.r1 = ImVec2(r[2], r[3]);
    row.kind = n.kind;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(800.0f, 600.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(std::move(root));
  }
  ~Session() {
    cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
    ImGui::DestroyContext(ctx);
  }
  void frame() {
    ImGui::GetIO().DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    rt.render();
    ImGui::Render();
  }
  void move(ImVec2 p) {
    ImGui::GetIO().AddMousePosEvent(p.x, p.y);
    frame();
  }
  void press(bool down) {
    ImGui::GetIO().AddMouseButtonEvent(0, down);
    frame();
  }
  void click(ImVec2 p) {
    move(p);
    press(true);
    press(false);
  }
  void key(ImGuiKey k) {
    ImGui::GetIO().AddKeyEvent(k, true);
    frame();
    ImGui::GetIO().AddKeyEvent(k, false);
    frame();
  }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  ari::Widget ed = ari::settings("Tuning", "cfg");
  ed.settings_open = true;
  ed.on = "tuned";
  ari::Widget win = ari::window("Settings", {ed});
  win.has_pos = true;
  win.size.w = {ari::Unit::Px, 520.0f};
  win.size.h = {ari::Unit::Px, 420.0f};
  Session s(win);
  int fired = 0;
  s.rt.on("tuned", [&] { ++fired; });
  cvc::state &st = cvc::state::instance(s.app);
  st("ui.cfg.nav.vmax").value("10");
  st("ui.cfg.nav.vmax").comment("the top speed");
  st("ui.cfg.nav.vmax").property("unit", "m/s").property("min", "0").property("max", "40");
  st("ui.cfg.nav.vmax").property("default", "10");
  st("ui.cfg.nav.gain").value("1");
  st("ui.cfg.nav.gain").property("unit", "1").property("min", "0.01").property("max", "10");
  st("ui.cfg.nav.on").value("1");
  st("ui.cfg.nav.on").property("min", "0").property("max", "1");
  st("ui.cfg.nav.mode").value("b");
  st("ui.cfg.nav.mode").property("choices", "a,b,c");
  st("ui.cfg.name").value("alpha");
  st("ui.cfg.speed").value("3");
  st("ui.cfg.speed").property("access", "telemetry");
  for (int f = 0; f < 3; ++f)
    s.move(ImVec2(5.0f, 590.0f));

  chk(s.be.rows.size() == 6, "every key has a row (" + std::to_string(s.be.rows.size()) + ")");
  const auto kind_is = [&](const char *id, ari::SettingKind k) {
    return s.be.rows.count(id) && s.be.rows[id].kind == k;
  };
  chk(kind_is("nav.vmax", ari::SettingKind::Number) &&
          kind_is("nav.gain", ari::SettingKind::Number) &&
          kind_is("nav.on", ari::SettingKind::Toggle) &&
          kind_is("nav.mode", ari::SettingKind::Choice) &&
          kind_is("name", ari::SettingKind::Text) && kind_is("speed", ari::SettingKind::Readout),
      "each row has the editor its metadata asks for");
  const RecordingBackend::Row vmax = s.be.rows["nav.vmax"];
  chk(vmax.e1.x - vmax.e0.x > 100.0f, "the slider fills the editor column");
  chk(!vmax.has_reset(), "at its default it offers no reset");
  chk(s.be.rows["name"].e0.y < vmax.e0.y, "a key under the root comes before the groups");

  // Press the slider at 60% of its width and drag it to its right end: the key follows the drag
  // (0..40, so about 24, then 40), and on: runs once, at the release.
  s.move(vmax.editor_at(0.6f));
  s.press(true);
  const double mid = std::strtod(st("ui.cfg.nav.vmax").value().c_str(), nullptr);
  chk(mid > 20.0 && mid < 28.0,
      "a press sets the value under it (" + st("ui.cfg.nav.vmax").value() + ")");
  s.move(vmax.editor_at(0.99f));
  const double end = std::strtod(st("ui.cfg.nav.vmax").value().c_str(), nullptr);
  chk(end > 37.0 && end <= 40.0,
      "the drag writes as it moves (" + st("ui.cfg.nav.vmax").value() + ")");
  s.rt.drain();
  chk(fired == 0, "on: waits for the release");
  s.press(false);
  s.rt.drain();
  chk(fired == 1, "the release runs on: once (" + std::to_string(fired) + ")");

  // Now off its default, the row offers a reset; a click on it writes the default back.
  s.move(ImVec2(5.0f, 590.0f));
  chk(s.be.rows["nav.vmax"].has_reset(), "a changed key shows its reset");
  s.click(s.be.rows["nav.vmax"].reset_mid());
  s.rt.drain();
  chk(st("ui.cfg.nav.vmax").value() == "10",
      "the reset writes the default (" + st("ui.cfg.nav.vmax").value() + ")");
  chk(fired == 2, "and runs on:");
  s.move(ImVec2(5.0f, 590.0f));
  chk(!s.be.rows["nav.vmax"].has_reset(), "back at its default, no reset");

  // A text field takes the edit on Enter; leaving it without one puts the value back.
  const RecordingBackend::Row name = s.be.rows["name"];
  s.click(name.editor_at(0.97f));
  ImGui::GetIO().AddInputCharactersUTF8("2");
  s.frame();
  s.click(ImVec2(5.0f, 590.0f)); // focus leaves the field
  s.frame();
  chk(st("ui.cfg.name").value() == "alpha", "leaving the field does not apply it");
  s.click(name.editor_at(0.97f));
  ImGui::GetIO().AddInputCharactersUTF8("2");
  s.frame();
  s.key(ImGuiKey_Enter);
  chk(st("ui.cfg.name").value() == "alpha2",
      "Enter applies it (" + st("ui.cfg.name").value() + ")");

  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
