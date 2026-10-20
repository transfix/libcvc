/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_CAPTURE_CAMERA_SOURCE_H__
#define __CVC_GL_CAPTURE_CAMERA_SOURCE_H__

// Phase-3 VIDEO capture: an SDL3 camera as a cvc::ariadne::stream::frame_source (see
// docs/STREAMING.md). This is ONE member of the generalized video-source family — a camera device;
// the virtual members (a multiplexer that selects among video sources, an ffmpeg process-pipe
// source) are pure/cvc-core and need no SDL. Capture lives in cvcGL because SDL3 is cvcGL's
// peripheral seam (keyboard/mouse/gamepad/audio/camera); the cvc monolith does not link SDL.
//
// The returned object is just a frame_source producing dense rgba8 frames — a stream and its sinks
// never know the frames came from a camera. Drive it with stream::start_producer():
//
//   auto cam = cvc::gl::capture::open_camera(0);
//   if (cam.source) {
//     cvc::ariadne::stream::stream_params p;
//     p.id = "cam0";
//     p.format.kind = cvc::ariadne::stream::frame_kind::video_raw;
//     p.format.codec = "rgba8";
//     p.format.w = cam.width; p.format.h = cam.height; p.format.stride = cam.width * 4;
//     auto s = cvc::ariadne::stream::stream::open(app, p);
//     s->start_producer(std::move(cam.source), cam.fps > 0 ? cam.fps : 30.0);
//   }
//
// SDL is kept out of this header: the SDL types and the capture loop live in the .cpp, gated on
// CVC_ENABLE_SDL. With no SDL in the build, open_camera() returns an empty result and
// list_cameras() an empty list (graceful absence, exactly like the SDL input seam) — callers branch
// on `result.source == nullptr`.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

// Full definition (NOT a forward declaration): camera_open holds a unique_ptr<frame_source>, so
// ~camera_open needs the complete type — a bare forward-decl would make any TU that holds a
// camera_open fail to compile at scope exit. frame_source.h pulls only <cstddef>/<cstdint> (no
// SDL), so the "SDL stays out of this header" invariant is preserved.
#include <cvc/ariadne/stream/frame_source.h>

namespace cvc {
namespace gl {
namespace capture {

// The outcome of open_camera(): the frame_source plus the format the caller should size its stream
// to (width/height — the requested size when a hint was passed, else the camera's negotiated size).
// The source converts the camera's native pixel format to rgba8 and RESCALES to width x height, so
// the stream's w/h must equal width/height here.
struct camera_open {
  std::unique_ptr<cvc::ariadne::stream::frame_source>
      source; // null on failure / no SDL / no device
  int width = 0;
  int height = 0;
  double fps = 0.0; // the camera's frame rate, 0 if the backend did not report one
  std::string name; // the opened device's human-readable name
};

// Open camera `device_index` (0 = the first enumerated device) and return a frame_source that
// yields dense rgba8 frames. `want_w`/`want_h` (and optional `want_fps`), when > 0, request that
// capture size: the source rescales the camera's frames to it, so the caller can size its stream up
// front (pass 0 to adopt the camera's own negotiated size).
//
// `native_aspect` (default false): when true, `want_w`/`want_h` are a MAX BUDGET rather than an exact
// size — the camera's frames are scaled to FIT WITHIN that budget PRESERVING the device's native
// aspect ratio (never upscaling), and each frame reports its true size via frame_source::cur_width/
// cur_height. Use this for a self-describing codec (jpeg_encode_source) so a portrait / 16:9 camera
// keeps its real shape instead of being stretched to w x h. When false (the default) the frames are
// stretched to exactly want_w x want_h — required for a fixed-size RAW rgba stream. Returns an empty
// result
// ({nullptr, 0, 0, …}) if SDL is unavailable, the index is out of range, or the open fails.
//
// PERMISSION. On a native build SDL resolves camera permission synchronously-ish: open_camera pumps
// events and waits briefly, and returns empty if access is not granted in time. On a BROWSER (wasm)
// build permission is a getUserMedia prompt resolved asynchronously by the event loop, so
// open_camera does NOT wait — it returns a ready source immediately and the source simply produces
// no frames until the user approves (fill() skips meanwhile). Either way, SOMETHING on the main
// thread must pump events (see pump_events) for approval and frames to arrive.
//
// THREAD CONTRACT: open_camera(), list_cameras(), and destruction of the stream/source returned
// here all touch the SDL camera subsystem (SDL_InitSubSystem/SDL_QuitSubSystem are NOT
// thread-safe), so call them — and destroy the owning stream — on the SDL MAIN thread (the thread
// that drives SDL / pumps its events), the same thread stream::close()/~stream run on. Only the
// per-frame capture inside fill() runs on the producer thread, using SDL calls documented
// thread-safe.
camera_open open_camera(int device_index = 0, int want_w = 0, int want_h = 0, double want_fps = 0.0,
                        bool native_aspect = false);

// Human-readable names of the available camera devices (index aligns with open_camera). Empty when
// SDL is unavailable or no camera is present. Call on the SDL main thread (see open_camera).
std::vector<std::string> list_cameras();

// Pump the SDL event queue once (a no-op when SDL is unavailable). SDL delivers camera device
// approval and frame readiness through its event system, so SOMETHING on the app's main thread must
// pump events for a camera source to produce frames. A cvcGL application already does this every
// frame via its SDL input loop, so it need not call this. A host with no SDL event loop (or a test)
// calls pump_events() periodically on the main thread while a camera source runs. Must be called on
// the thread that drives SDL (the main/owner thread), NOT the producer thread.
void pump_events();

} // namespace capture
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_CAPTURE_CAMERA_SOURCE_H__
