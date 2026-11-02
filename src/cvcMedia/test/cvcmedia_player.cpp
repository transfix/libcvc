// cvcMedia's FFmpeg player (cvc/media/player.h) on a 2 s VP8 + Opus clip and an Opus tone: it opens
// to Ready with its first picture (scaled into the box), plays with the clock moving and new
// pictures coming, seeks, pauses, ends, and fails cleanly on a missing file or bytes that are no
// media. Sound is decoded and timed but not played (PlayerOptions::audio false: no device needed).
// Without FFmpeg in the build it checks the player is reported absent.
#include <chrono>
#include <cstdio>
#include <cvc/media/player.h>
#include <fstream>
#include <functional>
#include <string>
#include <thread>

namespace am = cvc::ariadne::media;

namespace {

int failures = 0;

void chk(bool ok, const std::string &what) {
  std::printf("%s %s\n", ok ? "ok  " : "FAIL", what.c_str());
  if (!ok)
    ++failures;
}

bool wait_for(const std::function<bool()> &cond, double seconds = 5.0) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < end) {
    if (cond())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return cond();
}

std::unique_ptr<am::Player> open(const std::string &file, bool video) {
  am::PlayerSource src;
  src.uri = "file://" + file;
  src.base = CVC_MEDIA_TEST_DATA;
  src.video = video;
  src.frame_w = 64;
  src.frame_h = 36;
  cvc::media::PlayerOptions opt;
  opt.audio = false;
  return cvc::media::make_ffmpeg_player(src, opt);
}

} // namespace

int main() {
  if (!cvc::media::have_ffmpeg_player()) {
    chk(!cvc::media::register_ariadne_player(), "without FFmpeg nothing is registered");
    std::printf("skip: cvcMedia built without FFmpeg\n");
    return failures ? 1 : 0;
  }
  { // the clip: open, first picture, play, seek, pause, end
    std::unique_ptr<am::Player> p = open("clip.webm", true);
    chk(p != nullptr, "a player for the clip");
    chk(wait_for([&] { return p->state() == am::PlayerState::Ready; }), "it opens to ready");
    chk(p->duration() > 1.9 && p->duration() < 2.1,
        "its duration is the clip's 2 s (" + std::to_string(p->duration()) + ")");
    cvc::image f;
    long gen = 0;
    chk(wait_for([&] { return p->frame(f, gen); }), "its first picture shows before playing");
    chk(f.width() <= 64 && f.height() <= 36 && f.width() >= 60,
        "scaled into the 64x36 box (" + std::to_string(f.width()) + "x" +
            std::to_string(f.height()) + ")");
    chk(f.format() == cvc::image::pixel_format::RGBA, "as RGBA");
    p->play();
    chk(wait_for([&] { return p->state() == am::PlayerState::Playing; }), "it plays");
    const long first = gen;
    bool ok = wait_for([&] {
      p->frame(f, gen);
      return gen > first + 2 && p->position() > 0.2;
    });
    chk(ok, "the clock moves and new pictures come (" + std::to_string(p->position()) + " s)");
    p->seek(1.5);
    ok = wait_for([&] { return p->position() >= 1.45; });
    chk(ok, "a seek moves the clock (" + std::to_string(p->position()) + " s)");
    p->pause();
    chk(wait_for([&] { return p->state() == am::PlayerState::Paused; }), "it pauses");
    const double at = p->position();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    chk(std::abs(p->position() - at) < 0.02, "and the clock stops while paused");
    p->play();
    ok = wait_for(
        [&] {
          p->frame(f, gen); // the widget takes each picture, as the Runtime does every frame
          return p->state() == am::PlayerState::Ended;
        },
        4.0);
    chk(ok, "it plays to the end");
    p->play();
    ok = wait_for([&] { return p->state() == am::PlayerState::Playing && p->position() < 1.0; });
    chk(ok, "play after the end starts it over");
  }
  { // sound only: no picture, but a clock
    std::unique_ptr<am::Player> p = open("tone.opus", false);
    chk(wait_for([&] { return p->state() == am::PlayerState::Ready; }), "the tone opens");
    chk(p->duration() > 1.4 && p->duration() < 1.6, "1.5 s long");
    p->play();
    chk(wait_for([&] { return p->position() > 0.2; }), "and its clock runs");
    cvc::image f;
    long gen = 0;
    chk(!p->frame(f, gen), "with no picture");
  }
  { // failures
    std::unique_ptr<am::Player> p = open("no-such-clip.webm", true);
    const bool failed = wait_for([&] { return p->state() == am::PlayerState::Failed; });
    chk(failed, "a missing file fails");
    chk(p->error().find("cannot open") != std::string::npos, "saying why (" + p->error() + ")");
    std::unique_ptr<am::Player> q = open("checker.png", false);
    const bool no_audio = wait_for([&] { return q->state() == am::PlayerState::Failed; });
    chk(no_audio && !q->error().empty(), "a picture is no audio and fails (" + q->error() + ")");
  }
  chk(cvc::media::register_ariadne_player() && am::have_player_factory(),
      "register_ariadne_player makes it Ariadne's player");
  am::register_player_factory(nullptr);
  std::printf("%s\n", failures ? "FAILED" : "PASSED");
  return failures ? 1 : 0;
}
