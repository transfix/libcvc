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

// ImGuiBackend's context menus (docs/ARIADNE.md §5.28) on a bare ImGui context (no GL), driven by
// the synthesized mouse and keys: a right-click on a field offers Cut, Copy, Paste and Select all
// (the selection, or all of it, and a paste where the cursor was); on a text, Copy and Select
// text..., whose window copies a part; on a text view, Copy line and Copy all; on the bell's entry
// and the Notifications window's line, the whole message. On a touch screen a long press opens the
// menu, and the finger's release does not pick an item. The clipboard is ImGui's own, in memory.
// On a picture (§5.29), Save image..., which asks the host's Save dialog where, and the file is
// written there.
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/notify.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <thread>
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
struct Rect {
  ImVec2 lo, hi;
  ImVec2 mid() const { return ImVec2((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f); }
};

// The ImGui backend, keeping where each thing was drawn last: the fields, the texts, the menus'
// items (by label), the bell and the bell's entries.
struct RecordingBackend : cvc::gl::ImGuiBackend {
  std::map<std::string, Rect> at;
  std::map<std::string, Rect> items; // this frame's context menu items
  std::map<std::string, Rect> drawn; // ... and the last frame's
  void record(const std::string &k) { at[k] = {ImGui::GetItemRectMin(), ImGui::GetItemRectMax()}; }
  void begin_frame() override {
    drawn.swap(items);
    items.clear();
    ImGuiBackend::begin_frame();
  }
  void context_item_drawn(const char *label, const float r[4]) override {
    items[label] = {ImVec2(r[0], r[1]), ImVec2(r[2], r[3])};
  }
  ari::StringEdit input_text(const char *label, const char *key, const std::string &current,
                             bool masked) override {
    const ari::StringEdit e = ImGuiBackend::input_text(label, key, current, masked);
    record(std::string("field:") + label);
    return e;
  }
  ari::MessageEdit message_input(const char *label, const char *key, const std::string &current,
                                 const ari::MessageInputOptions &o) override {
    const ari::MessageEdit e = ImGuiBackend::message_input(label, key, current, o);
    record(std::string("message:") + label);
    return e;
  }
  void text_line(const char *t) override {
    ImGuiBackend::text_line(t);
    record(std::string("text:") + t);
  }
  void text_value(const char *label, const std::string &value) override {
    ImGuiBackend::text_value(label, value);
    record(std::string("value:") + label);
  }
  ari::TextViewEvent text_view(const char *label, const char *key, const std::string &text,
                               const ari::TextViewOptions &o) override {
    const ari::TextViewEvent e = ImGuiBackend::text_view(label, key, text, o);
    record("view");
    return e;
  }
  bool begin_tray_menu(const TrayIcon &icon) override {
    const bool open = ImGuiBackend::begin_tray_menu(icon);
    if (!open)
      record("bell"); // the bar's item; once open, the menu is the current window
    return open;
  }
  bool menu_item_action(const char *label) override {
    const bool clicked = ImGuiBackend::menu_item_action(label);
    record(std::string("menu:") + label);
    return clicked;
  }
  // §5.29 a picture is laid out as a box of its size rather than uploaded (there is no GL here),
  // and the Save dialog is the test's: it keeps the request and answers when told.
  bool draw_image(const char *, float width) override {
    ImGui::Dummy(ImVec2(width, width * 0.5f));
    record("image");
    return true;
  }
  ari::SavePathRequest save_req;
  ari::SavePathDone save_done;
  bool ask_save_path(const ari::SavePathRequest &req, ari::SavePathDone done) override {
    save_req = req;
    save_done = std::move(done);
    return true;
  }
};

struct Session {
  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::app app;
  ari::Runtime rt{app, ""};
  RecordingBackend be;
  explicit Session(ari::Widget root) {
    ImGui::SetCurrentContext(ctx);
    cvc::gl::ImGuiBackend::setImGuiContext(ctx);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplaySize = ImVec2(900.0f, 700.0f);
    io.IniFilename = nullptr;
    unsigned char *px = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&px, &w, &h); // builds the font atlas; no renderer needed
    rt.set_backend(&be);
    rt.set_root(std::move(root));
    frames(3);
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
    rt.drain();
  }
  void frames(int n) {
    for (int i = 0; i < n; ++i)
      frame();
  }
  void move(ImVec2 p) {
    ImGui::GetIO().AddMousePosEvent(p.x, p.y);
    frame();
  }
  void click(ImVec2 p, int button = 0) {
    ImGuiIO &io = ImGui::GetIO();
    move(p);
    io.AddMouseButtonEvent(button, true);
    frame();
    io.AddMouseButtonEvent(button, false);
    frames(2);
  }
  // A right-click on what was drawn as K; then the menu is up, its items in be.drawn.
  void menu_on(const std::string &k) { click(be.at[k].mid(), 1); }
  bool has_item(const std::string &label) const { return be.drawn.count(label) != 0; }
  void pick(const std::string &label) {
    if (!has_item(label)) {
      chk(false, "the menu has " + label);
      return;
    }
    click(be.drawn[label].mid());
  }
  void key(ImGuiKey k, bool shift = false) {
    ImGuiIO &io = ImGui::GetIO();
    if (shift)
      io.AddKeyEvent(ImGuiMod_Shift, true);
    io.AddKeyEvent(k, true);
    frame();
    io.AddKeyEvent(k, false);
    if (shift)
      io.AddKeyEvent(ImGuiMod_Shift, false);
    frame();
  }
  std::string val(const char *path) { return cvc::state::instance(app)(path).value(); }
  void set(const char *path, const std::string &v) { cvc::state::instance(app)(path).value(v); }
};

std::string clip() {
  const char *t = ImGui::GetClipboardText();
  return t ? t : "";
}

ari::Widget placed(ari::Widget w, float x, float y, float wd, float h) {
  w.size.w = {ari::Unit::Px, wd};
  w.size.h = {ari::Unit::Px, h};
  w.has_pos = true;
  w.pos_x = x;
  w.pos_y = y;
  return w;
}
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  // ---- a field: Copy, Select all + Cut, Paste where the cursor was, Paste over a selection ----
  {
    ari::Widget name = ari::input_text("Name", "name");
    ari::Widget pw = ari::input_text("Secret", "secret");
    pw.password = true;
    ari::Widget caption = ari::text("a caption");
    ari::Widget room = ari::text_bound("Room", "room");
    ari::Widget say = ari::message_input("##say", "draft");
    say.multiline = true;
    Session s(placed(ari::window("Form", {name, pw, caption, room, say}), 20, 20, 460, 300));
    s.set("name", "hello world");
    s.set("secret", "hunter2");
    s.set("room", "standup");
    s.frames(3);

    s.menu_on("field:Name");
    chk(s.has_item("Cut") && s.has_item("Copy") && s.has_item("Paste") && s.has_item("Select all"),
        "a right-click on a field: Cut, Copy, Paste, Select all");
    s.pick("Copy");
    chk(clip() == "hello world", "Copy with nothing selected copies the whole field: " + clip());
    chk(s.val("name") == "hello world", "and leaves it as it was");

    s.menu_on("field:Name");
    s.pick("Select all");
    s.frames(3);
    s.menu_on("field:Name"); // the field has the focus back, all of it selected
    s.pick("Cut");
    s.frames(3);
    chk(clip() == "hello world", "Cut takes the selection to the clipboard: " + clip());
    s.key(ImGuiKey_Enter);
    s.frames(2);
    chk(s.val("name").empty(), "and out of the field, which commits it: '" + s.val("name") + "'");

    ImGui::SetClipboardText("pasted\nline");
    s.menu_on("field:Name");
    s.pick("Paste");
    s.frames(3);
    s.key(ImGuiKey_Enter);
    s.frames(2);
    chk(s.val("name") == "pastedline",
        "Paste puts the clipboard in the field, a line field without its line break: " +
            s.val("name"));

    // Click into the field (the cursor at its end), select "line" with Shift+Left, paste over it.
    s.click(s.be.at["field:Name"].mid());
    s.key(ImGuiKey_End);
    for (int i = 0; i < 4; ++i)
      s.key(ImGuiKey_LeftArrow, /*shift=*/true);
    ImGui::SetClipboardText(" text");
    s.menu_on("field:Name");
    s.pick("Paste");
    s.frames(3);
    s.key(ImGuiKey_Enter);
    s.frames(2);
    chk(s.val("name") == "pasted text", "Paste replaces the selection: " + s.val("name"));
    // ImGui's own undo has it: Ctrl+Z (Cmd+Z on a Mac) in the field brings back what the paste
    // replaced. With ConfigMacOSXBehaviors (on Apple), AddKeyEvent swaps Cmd and Ctrl, so the key
    // a Mac's Cmd arrives as, ImGuiMod_Super, is the one that becomes the shortcut key.
    s.click(s.be.at["field:Name"].mid());
    ImGuiIO &io = ImGui::GetIO();
    const ImGuiKey shortcut = io.ConfigMacOSXBehaviors ? ImGuiMod_Super : ImGuiMod_Ctrl;
    io.AddKeyEvent(shortcut, true);
    s.key(ImGuiKey_Z);
    io.AddKeyEvent(shortcut, false);
    s.key(ImGuiKey_Enter);
    s.frames(2);
    chk(s.val("name") == "pastedline", "and ImGui's undo takes the paste back: " + s.val("name"));

    // A password field: no Cut or Copy, though Paste works.
    ImGui::SetClipboardText("unchanged");
    s.menu_on("field:Secret");
    s.pick("Copy");
    chk(clip() == "unchanged", "a password's Copy is disabled: " + clip());
    s.key(ImGuiKey_Escape);
    s.frames(2);

    // A multi-line message field keeps the line break, without the CR.
    ImGui::SetClipboardText("one\r\ntwo");
    s.menu_on("message:##say");
    s.pick("Paste");
    s.frames(3);
    s.click(s.be.at["text:a caption"].mid()); // focus leaves: the draft is committed
    s.frames(2);
    chk(s.val("draft") == "one\ntwo", "a message field pastes its lines: " + s.val("draft"));

    // Text: Copy (a value's copies the value), and Select text... opens the selection window.
    s.menu_on("text:a caption");
    chk(s.has_item("Copy") && s.has_item("Select text..."), "a text's menu: Copy, Select text...");
    s.pick("Copy");
    chk(clip() == "a caption", "a text's Copy: " + clip());
    s.menu_on("value:Room");
    s.pick("Copy");
    chk(clip() == "standup", "a value's Copy copies the value: " + clip());
    s.menu_on("text:a caption");
    s.pick("Select text...");
    s.frames(2);
    chk(s.has_item("Copy##cvc.select") && s.has_item("Close##cvc.select"),
        "Select text... opens the window with the text");
    ImGui::SetClipboardText("");
    s.pick("Select all##cvc.select");
    s.frames(3);
    s.pick("Copy##cvc.select");
    chk(clip() == "a caption", "its Select all and Copy copy it: " + clip());
    s.pick("Close##cvc.select");
    s.frames(2);
    chk(!s.has_item("Copy##cvc.select"), "Close closes it");

    // A long press on a touch screen opens the menu; the finger's release picks nothing in it.
    // In touch mode a window moves by its title bar only (ImGuiOverlay), so a finger held on its
    // text is not a window's drag.
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    ImGui::SetClipboardText("before");
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
    s.move(s.be.at["text:a caption"].mid());
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
    io.AddMouseButtonEvent(0, true);
    s.frames(40); // 0.67 s, still
    chk(s.has_item("Copy"), "a long press opens the text's menu");
    // The menu opens at the finger; lift it over the menu's first item.
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
    io.AddMousePosEvent(s.be.drawn["Copy"].mid().x, s.be.drawn["Copy"].mid().y);
    s.frame();
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
    io.AddMouseButtonEvent(0, false);
    s.frames(2);
    chk(clip() == "before" && s.has_item("Copy"),
        "the release that ends the long press picks nothing: the menu stays: " + clip());
    io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
    s.pick("Copy");
    chk(clip() == "a caption", "a tap on Copy then copies: " + clip());
    io.AddMouseSourceEvent(ImGuiMouseSource_Mouse);
  }

  // ---- a text view: Copy line (the line under the pointer), Copy all ----
  {
    ari::Widget log = ari::text_view("##log", "log");
    Session s(placed(ari::window("Chat", {log}), 20, 20, 420, 300));
    s.set("log", "[10:01] ana: alpha\n[10:02] bo: beta\n[10:03] cy: gamma");
    s.frames(4);
    const Rect v = s.be.at["view"];
    const float lh = ImGui::GetTextLineHeightWithSpacing();
    const float pad = ImGui::GetStyle().WindowPadding.y;
    s.click(ImVec2(v.lo.x + 40.0f, v.lo.y + pad + 1.5f * lh), 1); // the second line
    chk(s.has_item("Copy line") && s.has_item("Copy all"), "a text view's menu: Copy line, all");
    s.pick("Copy line");
    chk(clip() == "[10:02] bo: beta", "Copy line copies the line under the pointer: " + clip());
    s.click(ImVec2(v.lo.x + 40.0f, v.lo.y + pad + 0.5f * lh), 1);
    s.pick("Copy all");
    chk(clip() == s.val("log"), "Copy all copies the whole text");
  }

  // ---- notifications: the bell's entry and the Notifications window's line copy the message ----
  {
    ari::Widget bar = ari::menubar({ari::menu("File", {}), ari::tray_bell()});
    Session s(ari::group({bar}));
    cvc::notification n;
    n.text = "build failed\nstep 3: link error";
    n.source = "ci";
    cvc::notify(s.app, n);
    s.frames(3);
    chk(s.be.at.count("bell") != 0, "the bell is in the tray");
    s.click(s.be.at["bell"].mid());
    std::string entry;
    for (const auto &kv : s.be.at)
      if (kv.first.find("build failed") != std::string::npos && kv.first.rfind("menu:", 0) == 0)
        entry = kv.first;
    chk(!entry.empty(), "the bell's menu lists the entry");
    s.menu_on(entry);
    chk(s.has_item("Copy"), "a right-click on the entry: Copy");
    s.pick("Copy");
    chk(clip() == n.text, "it copies the whole message, every line: " + clip());

    s.set("tree._notifications.open", "1");
    s.frames(3);
    std::string line;
    for (const auto &kv : s.be.at)
      if (kv.first.find("build failed") != std::string::npos && kv.first.rfind("text:", 0) == 0)
        line = kv.first;
    chk(!line.empty(), "the Notifications window shows the entry");
    ImGui::SetClipboardText("");
    s.menu_on(line);
    s.pick("Copy");
    chk(clip() == n.text, "its line copies the whole message too: " + clip());
  }

  // ---- §5.29 a picture: Save image..., the host's Save dialog, the file written there ----
  {
    namespace fs = std::filesystem;
    const fs::path dir =
        fs::temp_directory_path() /
        ("cvcgl_imgui_save_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(dir);
    const std::string png = std::string("\x89PNG\r\n\x1a\n", 8) + "the picture's own bytes";
    std::ofstream(dir / "upload", std::ios::binary) << png;
    Session s(placed(ari::window("Post", {ari::image_widget("pic", "file://upload", 96)}), 20, 20,
                     300, 200));
    s.rt.set_media_base(dir.string());
    s.frames(3);
    chk(s.be.at.count("image") != 0, "the picture is drawn");
    s.menu_on("image");
    chk(s.has_item("Save image..."), "a right-click on a picture: Save image...");
    s.pick("Save image...");
    for (int i = 0; i < 1000 && !s.be.save_done; ++i) {
      s.frame();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    chk(static_cast<bool>(s.be.save_done), "the host's Save dialog is asked, once the bytes came");
    chk(s.be.save_req.name == "upload.png",
        "the name from the URL, the extension from the bytes: " + s.be.save_req.name);
    chk(s.be.save_req.title == "Save image", "titled Save image: " + s.be.save_req.title);
    const fs::path chosen = dir / "mine.png";
    if (s.be.save_done)
      s.be.save_done(chosen.string(), "");
    for (int i = 0; i < 1000 && !fs::exists(chosen); ++i) {
      s.frame();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    {
      std::ifstream in(chosen, std::ios::binary);
      const std::string got{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
      chk(got == png, "the file is written where the user chose");
    }
    std::error_code ec;
    fs::remove_all(dir, ec);
  }

  std::printf("%s (%d failure%s)\n", failures ? "FAILED" : "passed", failures,
              failures == 1 ? "" : "s");
  return failures ? 1 : 0;
#endif
}
