// ImGuiBackend and pictures by URI (docs/ARIADNE.md §5.11) on a bare ImGui context (no GL): an
// image: widget whose src is a file:// URI is published to the backend once it has loaded, as an
// RGBA raster fitted to its box, under the cache's name, and drawn at the box's width; until then
// (and on failure) a placeholder of the same box is. GL texture upload is draw_image's existing
// path (cvcgl_stream_texture covers it), so this backend records draw_image instead of calling GL.
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <map>
#include <string>
#include <thread>
#include <vector>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#include <imgui_internal.h> // FindWindowByName: the window's draw list
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
struct RecordingBackend : cvc::gl::ImGuiBackend {
  std::map<std::string, cvc::image> published;
  std::map<std::string, float> drawn; // name -> width, this frame
  void set_image(const std::string &name, const cvc::image &img) override {
    published[name] = img;
    ImGuiBackend::set_image(name, img);
  }
  // §5.12: what each backdrop was asked to draw; the base draws it (the picture cleared, since
  // drawing it would need a GL texture), so the draw-list split and the group's area are real.
  std::vector<cvc::ariadne::Backdrop> backdrops;
  std::vector<ImVec2> backdrop_size; // each closed backdrop's area, in order
  void begin_backdrop(const cvc::ariadne::Backdrop &bd) override {
    backdrops.push_back(bd);
    cvc::ariadne::Backdrop blank = bd;
    blank.image.clear();
    ImGuiBackend::begin_backdrop(blank);
  }
  void end_backdrop() override {
    ImGuiBackend::end_backdrop();
    const ImVec2 mn = ImGui::GetItemRectMin(), mx = ImGui::GetItemRectMax();
    backdrop_size.push_back(ImVec2(mx.x - mn.x, mx.y - mn.y));
  }
  bool draw_image(const char *name, float width) override {
    const auto it = published.find(name);
    if (it == published.end())
      return false;
    drawn[name] = width;
    ImGui::Dummy(ImVec2(width, width * it->second.height() / it->second.width()));
    return true;
  }
};
#endif

} // namespace

int main() {
#ifndef CVC_ENABLE_IMGUI
  std::printf("skip: cvcGL built without ImGui\n");
  return 0;
#else
  const std::filesystem::path dir =
      std::filesystem::temp_directory_path() / "cvcgl_imgui_media_test";
  std::filesystem::create_directories(dir);
  cvc::image src(64, 32, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
  for (int i = 0; i < 64 * 32; ++i) {
    unsigned char *p = src.data() + i * 4;
    p[0] = 200, p[1] = 40, p[2] = 90, p[3] = 255;
  }
  cvc::write_image(src, (dir / "pic.png").string());

  ImGuiContext *ctx = ImGui::CreateContext();
  cvc::gl::ImGuiBackend::setImGuiContext(ctx);
  ImGuiIO &io = ImGui::GetIO();
  io.DisplaySize = ImVec2(800.0f, 600.0f);
  io.IniFilename = nullptr;
  unsigned char *px = nullptr;
  int fw = 0, fh = 0;
  io.Fonts->GetTexDataAsRGBA32(&px, &fw, &fh);
  {
    cvc::app app;
    ari::Runtime rt(app, "ui");
    rt.set_media_base(dir.string());
    RecordingBackend be;
    rt.set_backend(&be);
    ari::LoadResult lr = ari::load_string("windows:\n"
                                          "  - window: Media\n"
                                          "    children:\n"
                                          "      - image: avatar\n"
                                          "        src: file://pic.png\n"
                                          "        size: [48, 48]\n"
                                          "        fit: cover\n"
                                          "        shape: circle\n"
                                          "        status: s.pic\n"
                                          "      - image: gone\n"
                                          "        src: file://gone.png\n"
                                          "        size: [96, 32]\n"
                                          "        status: s.gone\n");
    chk(lr.ok, "the document loads");
    rt.set_root(std::move(lr.root));
    const auto frame = [&] {
      be.drawn.clear();
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      rt.render();
      ImGui::Render();
    };
    const auto val = [&](const std::string &k) {
      try {
        return cvc::state::instance(app)("ui." + k).value();
      } catch (...) {
        return std::string();
      }
    };
    frame();
    bool placeholder = false;
    for (const auto &kv : be.drawn)
      placeholder = placeholder || kv.first.rfind("media:placeholder:loading:48x48:circle", 0) == 0;
    chk(placeholder || val("s.pic") == "ready", "the first frame draws a loading placeholder");
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < end &&
           (val("s.pic") != "ready" || val("s.gone") != "failed")) {
      frame();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    frame();
    chk(val("s.pic") == "ready", "the picture loads (" + val("s.pic") + ")");
    chk(val("s.gone") == "failed", "a missing file fails (" + val("s.gone") + ")");
    std::string pic;
    for (const auto &kv : be.drawn)
      if (kv.first.rfind("media:placeholder:", 0) != 0 && kv.first.rfind("media:", 0) == 0)
        pic = kv.first;
    chk(!pic.empty() && be.drawn[pic] == 48.0f, "it is drawn under the cache's name, 48 px wide");
    const cvc::image &img = be.published[pic];
    chk(img.width() == 32 && img.height() == 32,
        "fitted to the square box, as large as the source allows (" + std::to_string(img.width()) +
            "x" + std::to_string(img.height()) + ")");
    chk(img.format() == cvc::image::pixel_format::RGBA && img.data()[3] == 0 &&
            img.data()[(16 * 32 + 16) * 4 + 3] == 255,
        "RGBA, transparent outside the circle, opaque inside");
    bool failed_ph = false;
    for (const auto &kv : be.drawn)
      failed_ph = failed_ph || kv.first == "media:placeholder:failed:96x32";
    chk(failed_ph, "the missing picture draws the failed placeholder of its box");
  }
  { // §5.12 a window's and a group's background, around their content
    cvc::app app;
    ari::Runtime rt(app, "ui");
    rt.set_media_base(dir.string());
    RecordingBackend be;
    rt.set_backend(&be);
    ari::LoadResult lr =
        ari::load_string("windows:\n"
                         "  - window: Profile\n"
                         "    background: {src: file://pic.png, opacity: 0.5}\n"
                         "    children:\n"
                         "      - type: group\n"
                         "        size: [auto, 120]\n"
                         "        background: {src: file://pic.png, fit: contain}\n"
                         "        children: [Grace Hopper]\n");
    chk(lr.ok, "the background document loads");
    rt.set_root(std::move(lr.root));
    const auto frame = [&] {
      be.backdrops.clear();
      be.backdrop_size.clear();
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      rt.render();
      ImGui::Render();
    };
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
      frame();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while (std::chrono::steady_clock::now() < end &&
             (be.backdrops.size() != 2 || be.backdrops[0].image.empty()));
    frame();
    chk(be.backdrops.size() == 2 && be.backdrop_size.size() == 2, "two backdrops, both closed");
    if (be.backdrops.size() == 2 && be.backdrop_size.size() == 2) {
      chk(be.backdrops[0].window && !be.backdrops[1].window,
          "the window's first, then the group's");
      chk(be.backdrops[0].image.rfind("media:", 0) == 0 &&
              be.backdrops[0].image == be.backdrops[1].image,
          "both name the loaded picture (" + be.backdrops[0].image + ")");
      chk(be.backdrops[0].opacity == 0.5f && be.backdrops[1].fit == cvc::ariadne::ImageFit::Contain,
          "with their opacity and fit");
      chk(be.backdrop_size[0].y >= 120.0f, "the group's area takes its authored 120 px height (" +
                                               std::to_string(be.backdrop_size[0].y) + ")");
      // The split was merged back: the window's draw list is whole again (one channel).
      const ImGuiWindow *win = ImGui::FindWindowByName("Profile###Profile");
      if (!win)
        win = ImGui::FindWindowByName("Profile");
      chk(win && win->DrawList->_Splitter._Count <= 1 && win->DrawList->CmdBuffer.Size > 0,
          "the split draw list merged back into one");
    }
  }
  { // the card component (§6.5) in ImGui: its avatar, at its 40 px, beside the title
    const std::filesystem::path src_ariadne =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "cvc" /
        "ariadne";
    ari::register_cvc_uri_handler({src_ariadne.string()});
    cvc::app app;
    ari::Runtime rt(app, "ui");
    rt.set_media_base(dir.string());
    RecordingBackend be;
    rt.set_backend(&be);
    cvc::state::instance(app)("ui.post.name").value(std::string("Grace Hopper"));
    cvc::state::instance(app)("ui.post.avatar").value(std::string("file://pic.png"));
    ari::LoadResult lr = ari::load_string("import: cvc://components/card.ari\n"
                                          "windows:\n"
                                          "  - window: Feed\n"
                                          "    children:\n"
                                          "      - include: card\n"
                                          "        args: {id: post, avatar: post.avatar,"
                                          " title: post.name}\n");
    chk(lr.ok && !lr.root.children.empty(), "a document with a card loads");
    rt.set_root(std::move(lr.root));
    const auto frame = [&] {
      be.drawn.clear();
      io.DeltaTime = 1.0f / 60.0f;
      ImGui::NewFrame();
      rt.render();
      ImGui::Render();
    };
    bool avatar = false;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!avatar && std::chrono::steady_clock::now() < end) {
      frame();
      for (const auto &kv : be.drawn)
        avatar = avatar || (kv.first.rfind("media:placeholder:", 0) != 0 &&
                            kv.first.rfind("media:", 0) == 0 && kv.second == 40.0f);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    chk(avatar, "the card's avatar is drawn at 40 px");
  }
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
  ImGui::DestroyContext(ctx);
  std::filesystem::remove_all(dir);
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
