// Phase-3 audio capture: cvc::gl::capture::audio_source (an SDL3 recording device as a stream
// frame_source). DEVICE-GATED: real microphone capture can't be exercised on a headless CI builder,
// so this test SKIPS (passes) when no device is present or the live open is not opted into; when a
// device IS available (and CVC_TEST_AUDIO is set) it verifies the open path, the reported format,
// and that fill() either produces a full f32 chunk or cleanly skips (no crash, no OOB). The
// frame-transport side (producer_thread, pool, drop-at-source, join-on-close) and the pure audio
// sources are covered hardware-free by stream_test; this owns the SDL audio seam.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/stream/frame_source.h>
#include <cvc/gl/capture/audio_source.h>
#include <thread>
#include <vector>

int main() {
  // Enumeration is passive (it never activates a device), so always exercise it.
  const auto names = cvc::gl::capture::list_audio_capture_devices();
  std::printf("cvcgl_audio_source: %zu recording device(s) enumerated\n", names.size());

  // Opening a device ACTIVATES the microphone, so don't grab it on every CI/test run on a machine
  // that happens to have one. Gate the real open behind an opt-in env var; the default run verifies
  // compile + enumeration only.
  if (!std::getenv("CVC_TEST_AUDIO")) {
    std::printf("cvcgl_audio_source: SKIP open (set CVC_TEST_AUDIO=1 to exercise live capture)\n");
    return 0;
  }

  cvc::gl::capture::audio_capture_spec req; // 48 kHz stereo, 1024-frame chunks (defaults)
  cvc::gl::capture::audio_capture_open mic = cvc::gl::capture::open_audio_capture(req);
  if (!mic.source) {
    std::printf("cvcgl_audio_source: no device opened — SKIP (device path not exercised)\n");
    return 0;
  }

  // A device opened: the result must describe a sane f32 format, and the source's frame_bytes()
  // must match frames*channels*4 (what start_producer validates against the stream slab).
  assert(mic.sample_rate > 0 && mic.channels > 0 && mic.frames_per_chunk > 0);
  const std::size_t need = mic.source->frame_bytes();
  assert(need == static_cast<std::size_t>(mic.frames_per_chunk) *
                     static_cast<std::size_t>(mic.channels) * sizeof(float));

  // Wait (bounded) for the device to buffer a full chunk and verify the available->pull path: each
  // fill() is either a clean skip (device warming up / not a full chunk yet) or a FULL f32 chunk of
  // exactly frame_bytes — never a partial size. A full chunk should arrive within ~2 s on a live
  // device; a muted/absent-permission device opens but never fills, so treat a timeout as a skip.
  // Collect SEVERAL full chunks (not just one) so the pts-monotonic check actually spans successive
  // frames. A chunk should arrive every ~frames/sr seconds on a live device; a muted/absent device
  // opens but never fills, so a timeout with few/no chunks is treated as a skip (device-gated).
  std::vector<std::uint8_t> buf(need, 0u);
  int chunks = 0;
  double prev_pts = -1.0;
  for (int i = 0; i < 400 && chunks < 3; ++i) {
    const cvc::ariadne::stream::produced_frame pf = mic.source->fill(buf.data(), buf.size());
    assert(pf.bytes == 0 || pf.bytes == need); // never a partial chunk
    if (pf.bytes == need) {
      assert(pf.pts_seconds > prev_pts); // strictly monotonic across successive chunks
      prev_pts = pf.pts_seconds;
      ++chunks;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  std::printf("cvcgl_audio_source: OK (device '%s' %d Hz x%d, %d-frame chunks; live chunks: %d)\n",
              mic.name.c_str(), mic.sample_rate, mic.channels, mic.frames_per_chunk, chunks);
  return 0;
}
