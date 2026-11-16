// arigl snapshot --mp4 / --seconds (gla::record_frames over cvc::media::VideoWriter), offscreen: a
// tiny scene with a clock, a physics world and a ball over the ground is recorded for 1 s at 10
// frames a second. These checks pin that:
//   1. the file is an MP4 (ftyp, moov and mdat boxes) and, decoded back through the same FFmpeg,
//      holds 10 frames of 160 x 120 at 10 fps;
//   2. the scene clock ran exactly the recorded second (app.world_clock()), whatever the wall time;
//   3. the ball fell: its body moved, and the first and last frames differ;
//   4. --png beside the video writes the last frame;
//   5. the same through an ffmpeg program on PATH, where there is one;
//   6. a bad size or a program that isn't there fails with a reason, never a crash.
// Without any video encoder (no FFmpeg in cvcMedia and no ffmpeg program) it skips, saying why.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/record.h>
#include <cvc/gl/ariadne/scene_realize.h> // RealizedScene: the ball's body
#include <cvc/media/video_writer.h>
#include <cvc/physics/world.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

namespace fs = std::filesystem;
namespace gla = cvc::gl::ariadne;
namespace media = cvc::media;
using Backend = media::VideoWriterOptions::Backend;

namespace {

struct Checks {
  int fails = 0;
  void operator()(bool ok, const std::string &what) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok)
      ++fails;
  }
};

const char *kDoc = R"(meta: {min_libcvc: 3.4.0}
scene:
  clock: { scale: 1.0 }
  physics: {}
  background: { top: [0.2, 0.3, 0.5], bottom: [0.6, 0.7, 0.8] }
  nodes:
    - node: ground
      source: { box: { size: [10, 10, 0.5] } }
      transform: { position: [0, 0, -0.25] }
      material: { color: [0.3, 0.6, 0.3] }
      body: static
    - node: ball
      source: { sphere: 0.8 }
      transform: { position: [0, 0, 6] }
      material: { color: [0.9, 0.2, 0.1] }
      body: { density: 500 }
)";

// The top-level boxes of an MP4 (ISO BMFF): each a 32-bit big-endian size and a four-letter type.
std::set<std::string> top_level_boxes(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  const std::vector<unsigned char> b((std::istreambuf_iterator<char>(in)),
                                     std::istreambuf_iterator<char>());
  std::set<std::string> out;
  std::size_t at = 0;
  while (at + 8 <= b.size()) {
    std::uint64_t size = (std::uint64_t(b[at]) << 24) | (std::uint64_t(b[at + 1]) << 16) |
                         (std::uint64_t(b[at + 2]) << 8) | std::uint64_t(b[at + 3]);
    const std::string type(reinterpret_cast<const char *>(&b[at + 4]), 4);
    if (size == 1 && at + 16 <= b.size()) { // a 64-bit size follows
      size = 0;
      for (int k = 0; k < 8; ++k)
        size = (size << 8) | b[at + 8 + k];
    } else if (size == 0) {
      size = b.size() - at; // to the end of the file
    }
    if (size < 8)
      break;
    out.insert(type);
    at += size;
  }
  return out;
}

double mean_abs_diff(const std::vector<unsigned char> &a, const std::vector<unsigned char> &b) {
  if (a.size() != b.size() || a.empty())
    return -1.0;
  double sum = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
    sum += std::abs(int(a[i]) - int(b[i]));
  return sum / a.size();
}

double ball_height(gla::AriRuntime &rt) {
  const gla::RealizedScene *s = rt.realized();
  if (!s || !s->physics)
    return std::nan("");
  const auto id = s->body("ball");
  return id.valid() ? s->physics->state(id).position.z : std::nan("");
}

void record_case(Checks &check, const fs::path &dir, const std::string &doc, Backend backend,
                 const std::string &name) {
  std::printf("== record 1 s at 10 fps (%s) ==\n", name.c_str());
  const fs::path mp4 = dir / ("record_" + name + ".mp4");
  const fs::path png = dir / ("record_" + name + ".png");
  fs::remove(mp4);
  fs::remove(png);

  cvc::app app;
  cvc::gl::SceneGraph sg(app, "record");
  cvc::gl::SceneRenderer view(sg, 160, 120, /*offscreen=*/true, "main");
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  ui.setVisible(true);
  gla::AriRuntime rt(view, cam, ui);
  for (const std::string &w : rt.load(doc))
    std::printf("  warning: %s\n", w.c_str());
  const double t0 = app.world_clock().t();
  const double z0 = ball_height(rt);

  gla::RecordOptions opt;
  opt.mp4 = mp4.string();
  opt.png = png.string();
  opt.seconds = 1.0;
  opt.fps = 10;
  opt.backend = backend;
  const gla::RecordResult r = gla::record_frames(rt, view, opt);
  check(r.ok, "recorded" + (r.ok ? " with " + r.encoder : ": " + r.error));
  if (!r.ok) {
    view.close();
    return;
  }
  check(r.frames == 10, "10 frames recorded (" + std::to_string(r.frames) + ")");
  check(r.width == 160 && r.height == 120, "at 160 x 120");
  check(std::abs(r.scene_seconds - 1.0) < 1e-9, "the run covered 1 s of scene time");

  // 2. The scene clock ran the recorded second.
  const double t1 = app.world_clock().t();
  std::printf("  world clock %.4f -> %.4f\n", t0, t1);
  check(std::abs((t1 - t0) - 1.0) < 0.02, "the scene clock advanced 1 s");

  // 3. The ball fell (it starts 6 m up; in 1 s it falls ~4.9 m, onto the ground).
  const double z1 = ball_height(rt);
  std::printf("  ball z %.3f -> %.3f\n", z0, z1);
  if (cvc::physics::backend_available(cvc::physics::backend::jolt))
    check(z0 - z1 > 2.0, "the ball's body fell");
  else
    std::printf("  skipped the fall: no Jolt in this build\n");

  // 1. An MP4 on disk.
  std::error_code ec;
  check(fs::is_regular_file(mp4, ec) && fs::file_size(mp4, ec) > 0, "the .mp4 is written");
  const std::set<std::string> boxes = top_level_boxes(mp4);
  check(boxes.count("ftyp") && boxes.count("moov") && boxes.count("mdat"),
        "it is an MP4: ftyp, moov and mdat boxes");

  // ... decoded back.
  if (media::have_video_library()) {
    const media::VideoInfo v = media::probe_video(mp4.string(), /*keep_first_and_last=*/true);
    check(v.ok, "decoded back" + (v.ok ? " (" + v.codec + ")" : ": " + v.error));
    check(v.frames == 10, "10 frames decode (" + std::to_string(v.frames) + ")");
    check(v.width == 160 && v.height == 120,
          "160 x 120 (" + std::to_string(v.width) + " x " + std::to_string(v.height) + ")");
    check(std::abs(v.fps - 10.0) < 0.01, "at 10 fps (" + std::to_string(v.fps) + ")");
    const double d = mean_abs_diff(v.first_rgb, v.last_rgb);
    std::printf("  first vs last frame: mean |difference| %.2f\n", d);
    if (cvc::physics::backend_available(cvc::physics::backend::jolt))
      check(d > 0.5, "the first and last frames differ: the ball moved in the video");
  } else {
    std::printf("  skipped decoding it: this cvcMedia has no FFmpeg\n");
  }

  // 4. The last frame as a PNG.
  check(fs::is_regular_file(png, ec) && fs::file_size(png, ec) > 0, "--png wrote the last frame");
  view.close();
}

} // namespace

int main() {
  Checks check;
  if (!gla::AriRuntime::have_yaml()) {
    std::printf("cvcgl_ari_record: SKIP (libcvc built without yaml-cpp)\n");
    return 0;
  }

  // 6. Failures say why, whatever is installed.
  {
    media::VideoWriter w;
    media::VideoWriterOptions odd;
    odd.width = 161;
    odd.height = 120;
    bool opened = w.open((fs::temp_directory_path() / "cvcgl_ari_record_odd.mp4").string(), odd);
    check(!opened && w.error().find("even") != std::string::npos,
          "an odd width is refused: " + w.error());
    media::VideoWriterOptions gone;
    gone.width = 160;
    gone.height = 120;
    gone.backend = Backend::Program;
    gone.program = (fs::temp_directory_path() / "cvcgl_ari_record_no_such_ffmpeg").string();
    opened = w.open((fs::temp_directory_path() / "cvcgl_ari_record_gone.mp4").string(), gone);
    check(!opened && !w.error().empty(), "a missing ffmpeg program is refused: " + w.error());
    check(!w.write_rgb(nullptr, 0), "nothing is written to a writer that did not open");
  }

  media::VideoWriterOptions size;
  size.width = 160;
  size.height = 120;
  size.fps = 10;
  std::string why;
  const std::string encoder = media::video_encoder(size, &why);
  if (encoder.empty()) {
    std::printf("cvcgl_ari_record: SKIP (no video encoder: %s)\n", why.c_str());
    return check.fails ? 1 : 0;
  }
  std::printf("encoder: %s\n", encoder.c_str());

  const fs::path dir = fs::temp_directory_path() / "cvcgl_ari_record";
  fs::create_directories(dir);
  const fs::path doc = dir / "falling_ball.ari";
  std::ofstream(doc) << kDoc;

  record_case(check, dir, doc.string(), Backend::Automatic, "auto");
  // 5. The program path, where there is an ffmpeg on PATH.
  media::VideoWriterOptions program = size;
  program.backend = Backend::Program;
  if (!media::video_encoder(program).empty())
    record_case(check, dir, doc.string(), Backend::Program, "program");
  else
    std::printf("skipped the ffmpeg program: none on PATH\n");

  std::printf("cvcgl_ari_record: %s\n", check.fails ? "FAILED" : "PASSED");
  return check.fails ? 1 : 0;
}
