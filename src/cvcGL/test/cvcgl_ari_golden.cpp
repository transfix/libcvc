// Golden-image tests for the Ariadne ImGui/VTK backend (CVCGL-UI-DSL-ROADMAP.md §16.6, phase 1):
// each .ari document in the golden directory is run through AriRuntime + ImGuiBackend exactly as a
// host runs it, rendered offscreen with the overlay visible, and its frame is compared with the
// PNG of the same name.
//
//   cvcgl_ari_golden <golden dir> <output dir> [document name ...]
//
// What makes a capture repeatable: a pinned UI font (golden/fonts, never a system font), UI scale
// 1 with touch mode off, the overlay's show/hide button hidden and its panels open (a view of at
// least 700 px on its short side, so the small-screen rule never closes them), ImGui's frame delta
// pinned, and several frames at a fixed dt before the capture, so new windows have finished
// appearing. The test then renders one frame more and requires it to be identical to the capture.
//
// Goldens are made under Mesa llvmpipe, the rasterizer of the Linux and Windows CI jobs. Other
// rasterizers, and other Mesa versions, draw anti-aliased edges and shading a little differently,
// so every frame is compared within kTolerance, which was measured to absorb that and still fail a
// one-step value change. A platform golden <name>.<variant>.png, if committed, is used instead of
// <name>.png (CVC_GOLDEN_VARIANT, default "macos" on Apple). Every comparison prints how its
// differences spread, so a CI log shows what each rasterizer actually does.
//
//   CVC_UPDATE_GOLDEN=1    write the goldens instead of comparing (the reference ones only under
//                          llvmpipe: LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe)
//   CVC_GOLDEN_VARIANT=v   compare with (or write) <name>.v.png where one exists
//
// On a failed comparison <output dir>/<name>.actual.png and <name>.diff.png are written. Skips
// (rc 0) where nothing rasterises, unless CVC_REQUIRE_RENDER=1.
#include "golden_util.h"
#include "streaming_test_util.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <filesystem>
#include <string>
#include <vector>
#include <vtkOpenGLRenderWindow.h>
#ifdef CVC_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace fs = std::filesystem;
using cvcgl_golden::Image;
using cvcgl_golden::Tolerance;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 960, kHeight = 720; // short side >= 700: the overlay keeps its panels open
constexpr double kDt = 1.0 / 30.0;         // the host frame step AND ImGui's frame delta
constexpr int kMinFrames = 4;              // frames before the capture (ImGui auto-fit needs 2)
const double kBackground[3] = {0.09, 0.10, 0.12};

// One tolerance for every rasterizer, from these measurements against the goldens (Mesa 25.3.6
// llvmpipe, 960x720 = 691200 px; pixels differing by more than 8 / more than 40 in a channel):
//   Mesa 25.2.8 llvmpipe (Ubuntu's)    up to 1312 / 0  (anti-aliased edges, the shaded sphere)
//   NVIDIA GTX 1650                    up to 1 / 0
//   the Agents slider 64 -> 65         232 / 144       (one step of a widget's value)
//   the Widgets window moved 1 px down 15240 / 12167
// So +-40 a channel, with at most 0.002% of the pixels (13) beyond it, holds across Mesa versions
// and GPUs and still fails a one-step value change or a widget moved by a pixel.
const Tolerance kTolerance{40, 0.00002};

// A scripted pointer for documents whose interesting state needs one (an open menu, a tooltip).
// Coordinates are ImGui's: px from the top-left of the view. Before frame `frame` the pointer moves
// to (x, y) and, when button >= 0, presses (down) or releases that button. One event a frame,
// because ImGui applies at most one button change per frame.
struct PointerStep {
  int frame;
  float x, y;
  int button; // -1: move only
  bool down;
};
struct GoldenCase {
  const char *name;
  std::vector<PointerStep> pointer;
};
const GoldenCase kCases[] = {
    // Click "File" on the menu bar: the dropdown opens on the press and stays open.
    {"menus",
     {{1, 22.0f, 10.0f, -1, false}, {2, 22.0f, 10.0f, 0, true}, {3, 22.0f, 10.0f, 0, false}}},
    // Rest over the "Reset" button: its tooltip shows (no hover delay on the backend's tooltips).
    {"widgets", {{1, 52.0f, 369.0f, -1, false}}},
};

const GoldenCase *findCase(const std::string &name) {
  for (const GoldenCase &c : kCases)
    if (name == c.name)
      return &c;
  return nullptr;
}

std::string glRenderer(cvc::gl::SceneRenderer &view) {
  auto *win = vtkOpenGLRenderWindow::SafeDownCast(view.renderWindow());
  if (!win)
    return {};
  const std::string caps = win->ReportCapabilities();
  const char *key = "OpenGL renderer string:";
  const auto p = caps.find(key);
  if (p == std::string::npos)
    return {};
  std::string r = caps.substr(p + std::strlen(key), caps.find('\n', p) - p - std::strlen(key));
  r.erase(0, r.find_first_not_of(' '));
  return r;
}

std::string goldenVariant() {
  if (const char *v = std::getenv("CVC_GOLDEN_VARIANT"))
    return v;
#ifdef __APPLE__
  return "macos";
#else
  return "";
#endif
}

// Pixels whose colour is not the view's background: proof that something (the UI) was drawn.
double drawnFraction(const Image &img) {
  if (img.empty())
    return 0.0;
  long n = 0;
  for (size_t i = 0; i + 2 < img.rgb.size(); i += 3) {
    bool bg = true;
    for (int k = 0; k < 3; ++k)
      bg = bg && std::abs(int(img.rgb[i + k]) - int(kBackground[k] * 255.0 + 0.5)) <= 2;
    n += bg ? 0 : 1;
  }
  return double(n) / double(img.w * img.h);
}

#ifdef CVC_ENABLE_IMGUI
void injectPointer(cvc::gl::ImGuiOverlay &ui, const GoldenCase *gc, int frame) {
  if (!gc)
    return;
  // The overlay's context is the current one; adopting it explicitly is what makes this work
  // where the test links its own copy of imgui (see ImGuiOverlay::imguiContext()).
  ImGui::SetCurrentContext(ui.imguiContext());
  ImGuiIO &io = ImGui::GetIO();
  for (const PointerStep &s : gc->pointer) {
    if (s.frame != frame)
      continue;
    io.AddMousePosEvent(s.x, s.y);
    if (s.button >= 0)
      io.AddMouseButtonEvent(s.button, s.down);
  }
}
#endif

// False when the overlay cannot draw on this GL context at all (the caller stops: a skip).
bool runDocument(const fs::path &doc, const fs::path &goldenDir, const fs::path &outDir,
                 bool update) {
  const std::string name = doc.stem().string();
  std::printf("== %s\n", name.c_str());
  const GoldenCase *gc = findCase(name);
  int frames = kMinFrames;
  if (gc)
    for (const PointerStep &s : gc->pointer)
      frames = std::max(frames, s.frame + 3);

  cvc::app app;
  cvc::gl::SceneGraph sg(app, "golden");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  view.setBackground(kBackground[0], kBackground[1], kBackground[2]);
  cvc::gl::CameraController cam(view);
  cam.setMode(cvc::gl::CameraController::Mode::Orbit);
  cvc::gl::ImGuiOverlay ui(view);
  if (!check(ui.enabled(), name + ": the ImGui overlay is enabled"))
    return true;
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setTouchMode(false); // also sets the UI scale to 1
  ui.setUiScale(1.0f);
  ui.setToggleButtonEnabled(false);
  ui.setPanelsOpen(true);
  ui.setFixedDeltaTime(static_cast<float>(kDt));

  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  const std::vector<std::string> loadWarnings = rt.load(doc.string());
  for (const std::string &w : loadWarnings)
    std::printf("  load: %s\n", w.c_str());
  check(loadWarnings.empty(), name + ": the document loads without warnings");

  for (int f = 0; f < frames; ++f) {
#ifdef CVC_ENABLE_IMGUI
    injectPointer(ui, gc, f);
#endif
    rt.step(kDt);
  }
  // ImGui's GL backend initialises at a first render. Where it cannot (VTK fell back to EGL with
  // no X display, which imgui's GLX-based loader cannot reach) nothing here can be tested.
  if (!ui.enabled())
    return false;
  rt.frame(kDt);
  const Image actual =
      cvcgl_golden::fromFrameRGB(view.frameRGB(), view.frameWidth(), view.frameHeight());
  rt.frame(kDt);
  const Image next =
      cvcgl_golden::fromFrameRGB(view.frameRGB(), view.frameWidth(), view.frameHeight());
  for (const std::string &w : rt.take_warnings())
    std::printf("  runtime: %s\n", w.c_str());

  const std::string renderer = glRenderer(view);
  const bool llvmpipe = renderer.find("llvmpipe") != std::string::npos;
  std::printf("  %d frames at %dx%d on %s\n", frames + 2, kWidth, kHeight,
              renderer.empty() ? "(unknown renderer)" : renderer.c_str());

  check(!actual.empty() && actual.w == kWidth && actual.h == kHeight,
        name + ": captured a " + std::to_string(kWidth) + "x" + std::to_string(kHeight) + " frame");
  const double drawn = drawnFraction(actual);
  check(drawn > 0.01, name + ": the frame is more than the background",
        std::to_string(100.0 * drawn) + "% of pixels drawn");
  const cvcgl_golden::Comparison settled = cvcgl_golden::compare(next, actual, {0, 0.0});
  check(settled.pass, name + ": one more frame draws the same pixels (the UI has settled)",
        cvcgl_golden::describe(settled, {0, 0.0}));

  // Which golden, and how strictly.
  const std::string variant = goldenVariant();
  const fs::path reference = goldenDir / (name + ".png");
  const fs::path platform = goldenDir / (name + "." + variant + ".png");
  fs::path golden = reference;
  const Tolerance tol = kTolerance;
  if (update) {
    if (!variant.empty()) {
      golden = platform;
    } else if (!llvmpipe) {
      check(false, name + ": refusing to write a reference golden from " + renderer,
            "make them under Mesa llvmpipe (LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe), or "
            "set CVC_GOLDEN_VARIANT to write a platform golden");
      return true;
    }
  } else if (!variant.empty() && fs::exists(platform)) {
    golden = platform;
  }
  std::printf("  golden %s (channel +-%d, at most %.4f%% of pixels beyond it)\n",
              golden.filename().string().c_str(), tol.channel, 100.0 * tol.maxFraction);
  std::string report;
  const bool ok = cvcgl_golden::checkGolden(name, actual, golden.string(), outDir.string(), tol,
                                            update, report);
  check(ok, name + (update ? ": golden written" : ": matches its golden"), report);
  return true;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <golden dir> <output dir> [document name ...]\n", argv[0]);
    return 2;
  }
  const fs::path goldenDir = argv[1], outDir = argv[2];
  std::vector<std::string> only(argv + 3, argv + argc);
  cvcgl_test::disableSwapThrottle();

#ifndef CVC_ENABLE_IMGUI
  std::printf("  skipped: cvcGL was built without CVC_ENABLE_IMGUI (no ImGui backend to render)\n");
  return cvcgl_test::finish("cvcgl_ari_golden");
#else
  if (!cvc::gl::ariadne::AriRuntime::have_yaml()) {
    std::printf("  skipped: libcvc was built without the .ari YAML loader\n");
    return cvcgl_test::finish("cvcgl_ari_golden");
  }
  const fs::path font = goldenDir / "fonts" / "LiberationSans-Regular.ttf";
  if (!check(fs::exists(font), "the pinned UI font is present", font.string()))
    return cvcgl_test::finish("cvcgl_ari_golden");
  cvc::gl::ImGuiOverlay::setUiFontPath(font.string()); // before any overlay is constructed

  {
    cvc::app probe;
    if (!cvcgl_test::renderAvailable(probe))
      return cvcgl_test::finish("cvcgl_ari_golden");
  }

  std::vector<fs::path> docs;
  for (const fs::directory_entry &e : fs::directory_iterator(goldenDir))
    if (e.is_regular_file() && e.path().extension() == ".ari")
      docs.push_back(e.path());
  std::sort(docs.begin(), docs.end());
  check(!docs.empty(), "found .ari documents in " + goldenDir.string());
  for (const GoldenCase &c : kCases) // a pointer script whose document is gone is a stale entry
    check(fs::exists(goldenDir / (std::string(c.name) + ".ari")),
          std::string("the pointer script '") + c.name + "' has its document");

  const bool update = cvcgl_golden::envFlag("CVC_UPDATE_GOLDEN");
  for (const fs::path &doc : docs) {
    if (!only.empty() && std::find(only.begin(), only.end(), doc.stem().string()) == only.end())
      continue;
    if (!runDocument(doc, goldenDir, outDir, update)) {
      const char *why = "the ImGui overlay could not initialise its OpenGL backend on this context "
                        "(e.g. VTK's EGL fallback with no X display)";
      if (cvcgl_golden::envFlag("CVC_REQUIRE_RENDER"))
        check(false, std::string("CVC_REQUIRE_RENDER is set, but ") + why);
      else
        std::printf("  skipped: %s\n", why);
      break;
    }
  }
  return cvcgl_test::finish("cvcgl_ari_golden");
#endif
}
