// cvcgl_ari_frame_bench — what a frame of an Ariadne document costs, offscreen: the document runs
// through AriRuntime + ImGuiBackend as a host runs it (as cvcgl_ari_golden does), and each frame's
// step is timed until the GPU has finished it (vtkRenderWindow::WaitForCompletion), so the time is
// the frame's whole cost, CPU and GPU. Not a test: a measuring tool, run by hand (the numbers in
// the shell's SH2 pull request, views drawn into textures, came from it).
//
//   cvcgl_ari_frame_bench [--width W] [--height H] [--warmup N] [--frames N] [--rounds R]
//                         doc.ari [doc.ari ...]
//
// For each document: --warmup frames (60) untimed, then --rounds (3) of --frames (300) timed; it
// prints each round's mean and the median, the 90th percentile and the mean over all of them, in
// milliseconds a frame. 1280 x 800 by default.
#include <algorithm>
#include <chrono>
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
#include <vtkRenderWindow.h>

namespace {

constexpr double kDt = 1.0 / 30.0;

struct Options {
  int width = 1280, height = 800, warmup = 60, frames = 300, rounds = 3;
  std::vector<std::string> docs;
};

bool parse(int argc, char **argv, Options &o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto num = [&](int &out) {
      if (i + 1 >= argc)
        return false;
      out = std::atoi(argv[++i]);
      return out > 0;
    };
    if (a == "--width") {
      if (!num(o.width))
        return false;
    } else if (a == "--height") {
      if (!num(o.height))
        return false;
    } else if (a == "--warmup") {
      if (!num(o.warmup))
        return false;
    } else if (a == "--frames") {
      if (!num(o.frames))
        return false;
    } else if (a == "--rounds") {
      if (!num(o.rounds))
        return false;
    } else if (!a.empty() && a[0] == '-') {
      return false;
    } else {
      o.docs.push_back(a);
    }
  }
  return !o.docs.empty();
}

int run(const Options &o, const std::string &doc) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "bench");
  cvc::gl::SceneRenderer view(sg, o.width, o.height, /*offscreen=*/true, "main");
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setTouchMode(false);
  ui.setUiScale(1.0f);
  ui.setToggleButtonEnabled(false);
  ui.setPanelsOpen(true);
  ui.setFixedDeltaTime(static_cast<float>(kDt));
  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  rt.add_component_path((std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
                         "cvc" / "ariadne")
                            .string());
  const std::vector<std::string> warnings = rt.load(doc);
  for (const std::string &w : warnings)
    std::fprintf(stderr, "  load: %s\n", w.c_str());
  vtkRenderWindow *win = view.renderWindow();
  for (int f = 0; f < o.warmup; ++f)
    rt.step(kDt);
  win->WaitForCompletion();
  std::vector<double> all;
  std::string rounds;
  for (int r = 0; r < o.rounds; ++r) {
    double sum = 0.0;
    for (int f = 0; f < o.frames; ++f) {
      const auto t0 = std::chrono::steady_clock::now();
      rt.step(kDt);
      win->WaitForCompletion();
      const double ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      all.push_back(ms);
      sum += ms;
    }
    char b[32];
    std::snprintf(b, sizeof b, "%s%.2f", rounds.empty() ? "" : " ", sum / o.frames);
    rounds += b;
  }
  std::vector<double> sorted = all;
  std::sort(sorted.begin(), sorted.end());
  double mean = 0.0;
  for (double ms : all)
    mean += ms;
  mean /= static_cast<double>(all.size());
  const double median = sorted[sorted.size() / 2];
  const double p90 = sorted[std::min(sorted.size() - 1, sorted.size() * 9 / 10)];
  std::printf("%-28s median %7.2f  p90 %7.2f  mean %7.2f ms/frame  (rounds: %s; %dx%d)\n",
              std::filesystem::path(doc).filename().string().c_str(), median, p90, mean,
              rounds.c_str(), o.width, o.height);
  std::fflush(stdout);
  view.close(); // ~AriRuntime runs after this with the view closed
  return 0;
}

} // namespace

int main(int argc, char **argv) {
  Options o;
  if (!parse(argc, argv, o)) {
    std::fprintf(stderr,
                 "usage: %s [--width W] [--height H] [--warmup N] [--frames N] [--rounds R] "
                 "doc.ari [doc.ari ...]\n",
                 argv[0]);
    return 2;
  }
  int rc = 0;
  for (const std::string &doc : o.docs)
    rc |= run(o, doc);
  return rc;
}
