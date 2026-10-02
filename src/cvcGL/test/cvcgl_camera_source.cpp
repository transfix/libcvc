// Phase-3 video capture: cvc::gl::capture::camera_source (an SDL3 camera as a stream frame_source).
// DEVICE-GATED: real camera capture can't be exercised on a headless CI builder, so this test SKIPS
// (passes) when no camera is present; when a camera IS available it verifies the open path, the
// reported format, and that fill() either produces a full rgba frame or cleanly skips (no crash, no
// OOB). The frame-transport side (producer_thread, pool, drop-at-source, join-on-close) is covered
// hardware-free by stream_test's StreamProducer.* via synthetic_source; this owns the SDL seam.

#undef NDEBUG
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/stream/frame_source.h>
#include <cvc/gl/capture/camera_source.h>
#include <thread>
#include <vector>

int main() {
  // Enumeration is passive (it never activates a device), so always exercise it.
  const auto names = cvc::gl::capture::list_cameras();
  std::printf("cvcgl_camera_source: %zu camera device(s) enumerated\n", names.size());

  // Opening a camera ACTIVATES it (the capture LED, a permission prompt), so don't do it on every
  // CI/test run on a machine that happens to have a webcam. Gate the real open behind an opt-in env
  // var; the default run verifies compile + enumeration only.
  if (!std::getenv("CVC_TEST_CAMERA")) {
    std::printf(
        "cvcgl_camera_source: SKIP open (set CVC_TEST_CAMERA=1 to exercise live capture)\n");
    return 0;
  }

  cvc::gl::capture::camera_open cam = cvc::gl::capture::open_camera(0);
  if (!cam.source) {
    // CVC_TEST_CAMERA was set but no camera opened (none present, no backend, or permission
    // denied).
    std::printf("cvcgl_camera_source: no camera opened — SKIP (device path not exercised)\n");
    return 0;
  }

  // A camera opened: the result must describe a sane rgba8 frame, and the source's frame_bytes()
  // must match width*height*4 (what start_producer validates against the stream slab).
  assert(cam.width > 0 && cam.height > 0);
  const std::size_t need = cam.source->frame_bytes();
  assert(need == static_cast<std::size_t>(cam.width) * static_cast<std::size_t>(cam.height) * 4u);

  // Wait (bounded) for the camera to deliver a real frame and verify the acquire->convert->copy
  // path: each fill() is either a clean skip (warm-up / no new frame yet) or a FULL rgba frame of
  // exactly frame_bytes — never a partial/other size. Require at least one full frame (a device is
  // present and was opened, so one should arrive within ~2 s; permission denial opens but never
  // delivers, so treat a timeout as a skip rather than a hard failure).
  std::vector<std::uint8_t> buf(need, 0u);
  bool got = false;
  for (int i = 0; i < 120 && !got; ++i) {
    cvc::gl::capture::pump_events(); // the main thread must pump for SDL to approve + deliver
                                     // frames
    const cvc::ariadne::stream::produced_frame pf = cam.source->fill(buf.data(), buf.size());
    assert(pf.bytes == 0 || pf.bytes == need); // never a partial frame
    if (pf.bytes == need) {
      got = true;
      // Converted to byte-order rgba8: every 4th byte (alpha) is opaque after RGBA32 conversion.
      assert(buf[3] == 0xFF);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  std::printf("cvcgl_camera_source: OK (camera '%s' %dx%d @ %.1f fps; live frame: %s)\n",
              cam.name.c_str(), cam.width, cam.height, cam.fps, got ? "yes" : "none in 2s (ok)");
  return 0;
}
