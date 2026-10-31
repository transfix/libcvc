// ImGuiBackend and pictures by URI (docs/ARIADNE.md §5.11) on a bare ImGui context (no GL): an
// image: widget whose src is a file:// URI is published to the backend once it has loaded, as an
// RGBA raster fitted to its box, under the cache's name, and drawn at the box's width; until then
// (and on failure) a placeholder of the same box is. GL texture upload is draw_image's existing
// path (cvcgl_stream_texture covers it), so this backend records draw_image instead of calling GL.
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/ImGuiBackend.h>
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <filesystem>
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
struct RecordingBackend : cvc::gl::ImGuiBackend {
  std::map<std::string, cvc::image> published;
  std::map<std::string, float> drawn; // name -> width, this frame
  void set_image(const std::string &name, const cvc::image &img) override {
    published[name] = img;
    ImGuiBackend::set_image(name, img);
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
  cvc::gl::ImGuiBackend::setImGuiContext(nullptr);
  ImGui::DestroyContext(ctx);
  std::filesystem::remove_all(dir);
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
#endif
}
