// cvcMedia's SDL3 audio output in a program with no window (ari-media's case): SDL's audio
// subsystem comes up alone (the video subsystem never does), a device opens, f32 PCM queues and
// flushes, and the process has loaded no GL, VTK or ImGui. CTest runs it on SDL's dummy drivers, so
// it needs no sound card; a build without SDL3 checks only that the output is reported absent.
#undef NDEBUG
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cvc/media/playback/audio_output.h>
#include <fstream>
#include <string>
#include <thread>
#include <vector>
#if CVC_MEDIA_TEST_SDL
#include <SDL3/SDL_init.h>
#endif

namespace pb = cvc::media::playback;

int main() {
  if (!pb::have_audio_output()) {
    assert(!pb::open_audio_output().sink);
    assert(pb::list_audio_playback_devices().empty());
    std::printf("skip: cvcMedia built without SDL3 -- the output is absent, as it says\n");
    return 0;
  }
  pb::audio_output_open out = pb::open_audio_output({48000, 2});
  if (!out.sink) {
    std::printf("skip: no playback device (set SDL_AUDIO_DRIVER=dummy)\n");
    return 0;
  }
  assert(out.sample_rate == 48000 && out.channels == 2);
#if CVC_MEDIA_TEST_SDL
  assert(SDL_WasInit(SDL_INIT_AUDIO) != 0);
  assert(SDL_WasInit(SDL_INIT_VIDEO) == 0 && "audio needs no video subsystem");
#endif
  std::vector<float> tone(4800 * 2, 0.25f); // 0.1 s of stereo
  assert(out.sink->play(tone.data(), tone.size() * sizeof(float)));
  assert(!out.sink->play(tone.data(), 3) && "a partial frame is refused");
  out.sink->flush();
  assert(out.sink->device_period_s() > 0.0 && "the device reports its buffer");
  out.sink.reset();

  // Pull mode: the device asks for its audio, in whole frames, without the app pushing any.
  std::atomic<std::size_t> pulled{0};
  std::atomic<bool> whole{true};
  pb::audio_output_open pull =
      pb::open_audio_output_pull({48000, 1}, [&](float *buf, std::size_t frames) {
        for (std::size_t i = 0; i < frames; ++i)
          buf[i] = 0.1f;
        if (frames == 0)
          whole = false;
        pulled += frames;
      });
  assert(pull.sink && pull.channels == 1);
  for (int i = 0; i < 200 && pulled.load() < 4800; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(pulled.load() >= 4800 && "the device pulled 0.1 s by itself");
  assert(whole.load());
  pull.sink.reset(); // stops the callbacks before `pulled` goes
  assert(!pb::open_audio_output_pull({48000, 1}, nullptr).sink && "a pull needs a fill");
#if defined(__linux__)
  // Nothing GL-shaped came in with it.
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line))
    for (const char *lib : {"libGL", "libEGL", "libvtk", "libimgui", "libcvcGL"})
      if (line.find(lib) != std::string::npos) {
        std::printf("FAIL: %s is loaded: %s\n", lib, line.c_str());
        return 1;
      }
#endif
  std::printf("ok: SDL3 audio played with no window, no video subsystem and no GL\n");
  return 0;
}
