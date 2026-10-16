// Phase-0 (conference): cvc::gl::playback::audio_stream_player auto-pumps an f32-PCM stream into an
// audio_output sink on its own thread. Headless + hardware-free: a FakeSink (an audio_output that
// records what it was handed) stands in for the SDL device, so this proves the pump end-to-end with
// no SDL and no audio hardware. (The real SDL device path is device-gated, like
// cvcgl_audio_source.)

// cvcpkg builds tests Release (NDEBUG), which would make every assert() vacuous.
#undef NDEBUG
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/stream/frame.h>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/core/app.h>
#include <cvc/gl/playback/audio_output.h>
#include <cvc/gl/playback/audio_stream_player.h>
#include <memory>
#include <thread>
#include <vector>

namespace cs = cvc::ariadne::stream;
namespace pb = cvc::gl::playback;

// A stand-in for an SDL playback device: records every chunk's size. Thread-safe (the pump calls
// play() on its own thread; the test reads the counters from main).
struct FakeSink : pb::audio_output {
  std::atomic<std::uint64_t> chunks{0};
  std::atomic<std::uint64_t> bytes{0};
  bool play(const void *data, std::size_t n) override {
    (void)data;
    chunks.fetch_add(1, std::memory_order_relaxed);
    bytes.fetch_add(n, std::memory_order_relaxed);
    return true;
  }
  std::size_t queued_bytes() const override { return 0; }
  void flush() override {}
};

static cs::format_desc pcm_f32(int rate, int ch, std::size_t bytes) {
  cs::format_desc f;
  f.kind = cs::frame_kind::audio_pcm;
  f.codec = "f32";
  f.sample_rate = rate;
  f.channels = ch;
  f.bytes = bytes;
  return f;
}

int main() {
  cvc::app app;
  const int rate = 48000, ch = 2;
  const std::size_t chunk = 240 * static_cast<std::size_t>(ch) * sizeof(float); // 240 stereo frames

  cs::stream_params p;
  p.id = "mic";
  p.format = pcm_f32(rate, ch, chunk);
  p.subscriber_depth = 8;
  p.expected_subscribers = 1;
  std::unique_ptr<cs::stream> s = cs::stream::open(app, p);
  assert(s && "stream::open failed");

  FakeSink fake;
  pb::audio_stream_player player(*s, fake); // borrowed-sink ctor (no SDL needed)
  assert(player.playing() && "a non-null sink means the player is live");

  // Publish N chunks; confirm each is pumped to the sink before the next, so the
  // test is deterministic (no ring drops) and proves the pump moves every chunk.
  const int N = 8;
  for (int i = 0; i < N; ++i) {
    auto buf = std::make_shared<std::vector<std::uint8_t>>(chunk, static_cast<std::uint8_t>(i + 1));
    s->channel().publish_external(buf->data(), buf->size(), 0.005 * i, buf);
    const std::uint64_t want = static_cast<std::uint64_t>(i + 1);
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (fake.chunks.load() < want && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(fake.chunks.load() == want && "pump did not play the chunk in time");
  }

  assert(fake.chunks.load() == static_cast<std::uint64_t>(N));
  assert(fake.bytes.load() == static_cast<std::uint64_t>(N) * chunk);
  assert(player.frames_played() == static_cast<std::uint64_t>(N));

  player.stop();
  player.stop(); // idempotent
  s.reset();

  std::printf("cvcgl_audio_player: OK (%d chunks, %llu bytes)\n", N,
              (unsigned long long)fake.bytes.load());
  return 0;
}
