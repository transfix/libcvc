// cvcgl_ariadne_shared_volume — the shared-volume demo (examples/ariadne/documents/
// volume_shared.ari; docs/roadmap/DATA_FROM_STATE_TREE.md, D1) through AriRuntime, as arigl runs
// it:
//   - the document loads without warnings; the app makes the bunny's distance field (data: bunny),
//     and the main scene's volren and the grid show that one volume: the volren's volume, the
//     grid's grid.volume and the dataset's volume are one pointer;
//   - Filter > Smooth's program runs volume.filter-start beside the frame, and on_tick puts the
//     result back as the dataset's volume (data-put): the volren and the grid both follow it, and
//     the dataset's rev goes up;
//   - Original (data-reload) makes the bunny again, and both follow that too.
// Offscreen; skips where nothing rasterises (or this build makes no distance field) unless
// CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/data.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

constexpr int kWidth = 1280, kHeight = 720;
constexpr double kDt = 1.0 / 30.0;

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

std::shared_ptr<const cvc::volume> volume_at(cvc::app &app, const std::string &path) {
  cvc::state *s = cvc::state::instance(app).findThroughLinks(path);
  if (!s)
    return nullptr;
  const boost::any d = s->data();
  const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&d);
  return p ? *p : nullptr;
}

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
#ifndef CVC_ENABLE_IMGUI
  std::printf("SKIP: no ImGui\n");
  return require ? 1 : 0;
#else
  const fs::path src = fs::path(__FILE__).parent_path().parent_path().parent_path();
  const fs::path doc =
      src.parent_path() / "examples" / "ariadne" / "documents" / "volume_shared.ari";
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::SceneGraph sg(app, "vs");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, kWidth, kHeight, /*offscreen=*/true, "main");
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : 0;
  }
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setFixedDeltaTime(static_cast<float>(kDt));

  cvc::gl::ariadne::AriRuntime rt(view, cam, ui);
  rt.add_component_path((src / "cvc" / "ariadne").string());
  for (const std::string &w : rt.load(doc.string()))
    check(false, "the demo loads without warnings", w);

  const std::string prefix = sg.getStatePrefix();
  const std::string bunny = prefix + ".data.bunny";
  const std::string render = prefix + ".graphics.root.children.render";
  const std::string grid = prefix + ".graphics.root.children.grid";
  const auto step_until = [&](const std::function<bool()> &done, double seconds) {
    const auto t0 = std::chrono::steady_clock::now();
    while (!done()) {
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > seconds)
        return false;
      rt.step(kDt);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    rt.step(kDt);
    return true;
  };

  // The app makes the bunny's distance field off the frame thread.
  step_until(
      [&] {
        const std::string st = cvc::ariadne::data_status(app, bunny);
        return st == "ready" || st == "error";
      },
      300.0);
  if (cvc::ariadne::data_status(app, bunny) != "ready") {
    const std::string error = val(app, cvc::ariadne::DataStore::of(app)->resolve(bunny) + ".error");
    std::printf("SKIP: the distance field was not made (%s)\n", error.c_str());
    return require && error.find("CVC_ENABLE_SDF") == std::string::npos ? 1 : 0;
  }
  step_until([&] { return volume_at(app, render + ".volume") != nullptr; }, 10.0);
  const std::shared_ptr<const cvc::volume> made = volume_at(app, bunny + ".volume");
  check(made && volume_at(app, render + ".volume") == made &&
            volume_at(app, grid + ".grid.volume") == made,
        "the main scene's volren and the grid show the app's volume: one pointer",
        val(app, render + ".source.status"));
  auto vr = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(sg.getGraphics("render"));
  check(vr && vr->volumeCount() == 1, "the volren raycasts it");

  // Filter > Smooth: the job runs beside the frame, and on_tick puts its result back.
  // The dataset's rev, read at its store node (a read of <prefix>.data.bunny.rev with state(path)
  // would make a node under the document's link).
  const auto rev_now = [&] {
    return val(app, cvc::ariadne::DataStore::of(app)->resolve(bunny) + ".rev");
  };
  const std::string rev = rev_now();
  rt.run_program("(volume.filter-start \"data.bunny.volume\" (dict \"filter\" \"bilateral\" "
                 "\"radius\" 3 \"spatial_sigma\" 2.0 \"radiometric_sigma\" 1000.0) "
                 "\"filtering\")");
  const bool stored = step_until(
      [&] {
        const std::string pid = val(app, prefix + ".filtering.pid");
        return !pid.empty() && val(app, prefix + ".filtering.stored") == pid;
      },
      300.0);
  check(stored, "the filter runs as a job and on_tick takes its result in",
        val(app, prefix + ".filtering.status"));
  step_until([&] { return volume_at(app, render + ".volume") != made; }, 10.0);
  const std::shared_ptr<const cvc::volume> smoothed = volume_at(app, bunny + ".volume");
  check(smoothed && smoothed != made && rev_now() != rev,
        "data-put puts the filtered volume as the dataset's, its rev up");
  check(smoothed && volume_at(app, render + ".volume") == smoothed &&
            volume_at(app, grid + ".grid.volume") == smoothed && vr->volumeCount() == 1,
        "the volren and the grid both follow the filtered volume: still one pointer");
  check(smoothed && made && smoothed->XDim() == made->XDim() && smoothed->min() > made->min(),
        "the smoothed distance field is shallower inside");

  // Original: the app makes the bunny again.
  rt.run_program("(data-reload \"bunny\")");
  step_until(
      [&] {
        const auto now = volume_at(app, bunny + ".volume");
        return now && now != smoothed && cvc::ariadne::data_status(app, bunny) == "ready";
      },
      300.0);
  step_until([&] { return volume_at(app, render + ".volume") != smoothed; }, 10.0);
  const std::shared_ptr<const cvc::volume> again = volume_at(app, bunny + ".volume");
  check(again && again != smoothed && volume_at(app, render + ".volume") == again &&
            volume_at(app, grid + ".grid.volume") == again,
        "Original makes the bunny again, and both follow it");
  return cvcgl_test::finish("cvcgl_ariadne_shared_volume");
#endif
}
