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
#include <cstdint>
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
// and GPUs and still fails a one-step value change or a widget moved by a pixel. macOS CI's Apple
// Software Renderer left 3-6 px beyond 40 on windows / menus / grid but 40 on widgets and 467 on
// scene (edges and shading), so those two have macOS goldens (<name>.macos.png) instead.
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

// The first pixel two frames disagree on, with both colours ("" when they are identical).
std::string firstDifference(const Image &a, const Image &b) {
  if (a.empty() || b.empty() || a.w != b.w || a.h != b.h)
    return "";
  for (int y = 0; y < a.h; ++y)
    for (int x = 0; x < a.w; ++x) {
      const unsigned char *p = a.at(x, y), *q = b.at(x, y);
      if (p[0] != q[0] || p[1] != q[1] || p[2] != q[2]) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "first at (%d,%d): %d,%d,%d then %d,%d,%d", x, y, p[0], p[1],
                      p[2], q[0], q[1], q[2]);
        return buf;
      }
    }
  return "";
}

#ifdef CVC_ENABLE_IMGUI
// What ImGui handed the GL backend for the last rendered frame: its draw lists (vertices, indices,
// commands and their clip rects and textures) and every texture's size and pixels. Two frames with
// equal fingerprints asked GL to draw the same UI, so a pixel difference between them comes from
// the rasterizer, not from the UI.
struct DrawFingerprint {
  uint64_t lists = 0, textures = 0;
  int vertices = 0;
  std::string textureSizes;
  bool sameUi(const DrawFingerprint &o) const {
    return lists == o.lists && textures == o.textures && vertices == o.vertices;
  }
};

DrawFingerprint drawFingerprint(cvc::gl::ImGuiOverlay &ui) {
  DrawFingerprint fp;
  ImGui::SetCurrentContext(ui.imguiContext());
  const ImDrawData *dd = ImGui::GetDrawData();
  if (!dd || !dd->Valid)
    return fp;
  auto fnv = [](uint64_t &h, const void *data, size_t n) {
    const auto *b = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < n; ++i)
      h = (h ^ b[i]) * 1099511628211ull;
  };
  fp.lists = fp.textures = 1469598103934665603ull;
  for (const ImDrawList *dl : dd->CmdLists) {
    fnv(fp.lists, dl->VtxBuffer.Data, dl->VtxBuffer.Size * sizeof(ImDrawVert));
    fnv(fp.lists, dl->IdxBuffer.Data, dl->IdxBuffer.Size * sizeof(ImDrawIdx));
    for (const ImDrawCmd &c : dl->CmdBuffer) {
      const ImTextureID tex = c.GetTexID();
      fnv(fp.lists, &c.ClipRect, sizeof c.ClipRect);
      fnv(fp.lists, &tex, sizeof tex);
      fnv(fp.lists, &c.VtxOffset, sizeof c.VtxOffset);
      fnv(fp.lists, &c.IdxOffset, sizeof c.IdxOffset);
      fnv(fp.lists, &c.ElemCount, sizeof c.ElemCount);
    }
    fp.vertices += dl->VtxBuffer.Size;
  }
  if (dd->Textures)
    for (const ImTextureData *t : *dd->Textures) {
      fnv(fp.textures, &t->Width, sizeof t->Width);
      fnv(fp.textures, &t->Height, sizeof t->Height);
      if (t->Pixels)
        fnv(fp.textures, t->Pixels, static_cast<size_t>(t->GetSizeInBytes()));
      fp.textureSizes += (fp.textureSizes.empty() ? "" : " ") + std::to_string(t->Width) + "x" +
                         std::to_string(t->Height);
    }
  return fp;
}

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
#ifdef CVC_ENABLE_IMGUI
  const DrawFingerprint actualUi = drawFingerprint(ui);
#endif
  rt.frame(kDt);
  const Image next =
      cvcgl_golden::fromFrameRGB(view.frameRGB(), view.frameWidth(), view.frameHeight());
#ifdef CVC_ENABLE_IMGUI
  const DrawFingerprint nextUi = drawFingerprint(ui);
#endif
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
  std::string settledDetail = cvcgl_golden::describe(settled, {0, 0.0});
  const std::string firstDiff = firstDifference(actual, next);
  if (!firstDiff.empty())
    settledDetail += "; " + firstDiff;
#ifdef CVC_ENABLE_IMGUI
  if (actualUi.sameUi(nextUi)) {
    settledDetail += "; ImGui drew the same UI both times (" + std::to_string(nextUi.vertices) +
                     " vertices, textures " + nextUi.textureSizes + ")";
  } else {
    std::string what = actualUi.lists != nextUi.lists ? "draw lists" : "";
    if (actualUi.textures != nextUi.textures)
      what += what.empty() ? "textures" : " and textures";
    settledDetail += "; ImGui's " + what + " changed between them (" +
                     std::to_string(actualUi.vertices) + " -> " + std::to_string(nextUi.vertices) +
                     " vertices, textures " + actualUi.textureSizes + " -> " + nextUi.textureSizes +
                     ")";
  }
#endif
  check(settled.pass, name + ": one more frame draws the same pixels (the UI has settled)",
        settledDetail);

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
  // A build with nothing to render skips, except where CVC_REQUIRE_RENDER says this job renders:
  // there a missing ImGui or YAML loader is a failure, never a quiet pass.
  auto skip = [](const std::string &why) {
    if (cvcgl_golden::envFlag("CVC_REQUIRE_RENDER"))
      check(false, "CVC_REQUIRE_RENDER is set, but " + why);
    else
      std::printf("  skipped: %s\n", why.c_str());
    return cvcgl_test::finish("cvcgl_ari_golden");
  };

#ifndef CVC_ENABLE_IMGUI
  return skip("cvcGL was built without CVC_ENABLE_IMGUI (no ImGui backend to render)");
#else
  if (!cvc::gl::ariadne::AriRuntime::have_yaml())
    return skip("libcvc was built without the .ari YAML loader");
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
    if (!runDocument(doc, goldenDir, outDir, update))
      return skip("the ImGui overlay could not initialise its OpenGL backend on this context "
                  "(e.g. VTK's EGL fallback with no X display)");
  }
  return cvcgl_test::finish("cvcgl_ari_golden");
#endif
}
