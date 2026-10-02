// Phase-3 Ariadne stream widget: the render-thread pull step of cvc::gl::ariadne's "stream_view"
// widget (next_stream_frame_image). Hardware-free: a synthetic_source feeds a real video stream and
// this verifies the frame -> cvc::image conversion, the dimensions/format, and the seq dedup +
// monotonicity (a stale re-pull returns false and never rewinds seq). The ImGui/GL draw side
// (set_image + draw_image + ImGui::Image) is the existing image-viewer path, covered by
// cvcgl_stream_texture; this owns the new stream->image pull logic.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cvc/ariadne/stream/stream.h>
#include <cvc/ariadne/stream/stream_channel.h>
#include <cvc/ariadne/stream/stream_registry.h>
#include <cvc/ariadne/stream/synthetic_source.h>
#include <cvc/core/app.h>
#include <cvc/gl/ariadne/stream_view_widget.h>
#include <cvc/image/image.h>
#include <memory>
#include <thread>

namespace sx = cvc::ariadne::stream;
namespace ga = cvc::gl::ariadne;

int main() {
  cvc::app app;

  sx::stream_params p;
  p.id = "camview_test";
  p.format.kind = sx::frame_kind::video_raw;
  p.format.codec = "rgba8";
  p.format.w = 8;
  p.format.h = 8;
  p.format.stride = 8 * 4; // dense rgba8 -> frame_bytes = 256
  std::unique_ptr<sx::stream> s = sx::stream::open(app, p);
  assert(s && "stream::open failed");
  s->start_producer(std::make_unique<sx::synthetic_source>(8, 8, 120.0), 120.0);

  sx::stream_channel *ch = sx::stream_registry::for_app(app).lookup(s->token());
  assert(ch && "stream not registered under its token");
  std::shared_ptr<sx::subscription> sub = ch->subscribe(sx::deliver_mode::latest);
  assert(sub && "subscribe failed");

  // First frame: a dense 8x8 rgba8 image; seq advances off the -1 sentinel.
  std::int64_t seq = -1;
  cvc::image img;
  bool got = false;
  for (int i = 0; i < 400 && !got; ++i) {
    if (ga::next_stream_frame_image(*sub, seq, img))
      got = true;
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  assert(got && "no frame produced in ~0.8s");
  assert(img.width() == 8 && img.height() == 8);
  assert(img.format() == cvc::image::pixel_format::RGBA);
  assert(seq >= 0);

  // Seq dedup + monotonicity: a pull that finds no newer frame returns false and leaves seq
  // untouched (no stale re-delivery); a pull that DOES return true strictly advanced seq.
  std::int64_t prev = seq;
  int newer = 0;
  for (int i = 0; i < 50; ++i) {
    const std::int64_t before = seq;
    if (ga::next_stream_frame_image(*sub, seq, img)) {
      assert(seq > prev && "a new frame must strictly advance seq");
      prev = seq;
      ++newer;
    } else {
      assert(seq == before && "no-new-frame must not rewind or advance seq");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  s->close(); // join the producer before the stream/channel tear down
  std::printf("cvcgl_stream_view: OK (8x8 rgba8; first seq=%lld, %d newer frames seen)\n",
              static_cast<long long>(prev), newer);
  return 0;
}
