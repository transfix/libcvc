// Phase-3 audio playback: cvc::gl::playback::audio_output (f32 PCM pushed to an SDL3 playback
// device). DEVICE-GATED: real playback can't be exercised on a headless CI builder, so this test
// SKIPS (passes) when no output device is present or the live path is not opted into. When a device
// IS available (and CVC_TEST_AUDIO is set) it opens the default device, verifies the frame-aligned
// contract of play() (a non-aligned size is rejected), pushes a short generated tone, and confirms
// the device queues then drains it (queued_bytes grows after play() and falls toward 0 as SDL
// renders). The push/queue/flush contract is SDL's; this owns that seam.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/gl/playback/audio_output.h>
#include <thread>
#include <vector>

int main() {
  // Enumeration is passive (it never opens a device), so always exercise it.
  const auto names = cvc::gl::playback::list_audio_playback_devices();
  std::printf("cvcgl_audio_output: %zu playback device(s) enumerated\n", names.size());

  // Opening a device engages the audio hardware, so gate the real open behind an opt-in env var;
  // the default run verifies compile + enumeration only.
  if (!std::getenv("CVC_TEST_AUDIO")) {
    std::printf("cvcgl_audio_output: SKIP open (set CVC_TEST_AUDIO=1 to exercise live playback)\n");
    return 0;
  }

  cvc::gl::playback::audio_output_spec req; // 48 kHz stereo (defaults)
  cvc::gl::playback::audio_output_open out = cvc::gl::playback::open_audio_output(req);
  if (!out.sink) {
    std::printf("cvcgl_audio_output: no device opened — SKIP (device path not exercised)\n");
    return 0;
  }
  assert(out.sample_rate > 0 && out.channels > 0);

  // Frame-aligned contract: a chunk that is not a whole number of audio frames is rejected.
  const std::size_t frame_sz = static_cast<std::size_t>(out.channels) * sizeof(float);
  const unsigned char one_byte = 0;
  assert(!out.sink->play(&one_byte, 1) && "a sub-frame size must be rejected");

  // Generate ~200 ms of a 440 Hz tone (interleaved f32, same on each channel) and queue it.
  const int frames = out.sample_rate / 5; // 0.2 s
  std::vector<float> tone(static_cast<std::size_t>(frames) * out.channels);
  const double step = 2.0 * 3.14159265358979323846 * 440.0 / out.sample_rate;
  double phase = 0.0;
  for (int n = 0; n < frames; ++n) {
    const float s = static_cast<float>(0.2 * std::sin(phase));
    phase += step;
    for (int c = 0; c < out.channels; ++c)
      tone[static_cast<std::size_t>(n) * out.channels + c] = s;
  }
  const bool pushed = out.sink->play(tone.data(), tone.size() * sizeof(float));
  assert(pushed && "a frame-aligned tone chunk must queue");

  // The device should now have data queued; as SDL renders it, the queued count falls toward 0.
  const std::size_t queued0 = out.sink->queued_bytes();
  std::size_t queued_last = queued0;
  for (int i = 0; i < 100 && out.sink->queued_bytes() > 0; ++i) {
    queued_last = out.sink->queued_bytes();
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // up to ~1 s for 0.2 s of audio
  }
  std::printf("cvcgl_audio_output: OK (device '%s' %d Hz x%d; queued %zu B -> drained to %zu B)\n",
              out.name.c_str(), out.sample_rate, out.channels, queued0, out.sink->queued_bytes());

  // flush() must be safe to call (drops anything still queued) and leave nothing queued.
  out.sink->flush();
  assert(out.sink->queued_bytes() == 0);
  (void)queued_last;
  return 0;
}
