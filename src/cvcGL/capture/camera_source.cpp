/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/ariadne/stream/frame_source.h>
#include <cvc/gl/capture/camera_source.h>

#if CVC_ENABLE_SDL
#include <SDL3/SDL_camera.h>
#include <SDL3/SDL_events.h> // SDL_PumpEvents
#include <SDL3/SDL_init.h>
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_stdinc.h> // SDL_free, Uint64
#include <SDL3/SDL_surface.h>
#include <SDL3/SDL_timer.h> // SDL_Delay
#include <algorithm>         // std::min / std::max (native-aspect fit)
#include <cstdint>
#include <cstdio>
#include <cstring>
#endif

namespace cvc {
namespace gl {
namespace capture {

#if CVC_ENABLE_SDL
namespace {
namespace st = cvc::ariadne::stream;

// An SDL3 camera as a frame_source producing dense rgba8. Owns the SDL_Camera and one reference to
// the (refcounted) SDL camera subsystem, both released in the dtor — which is where
// stream::start_producer's join-before-free guarantee has the producer thread already stopped, so
// no fill() runs during teardown.
class sdl_camera_source : public st::frame_source {
public:
  // `native_aspect`: when true, w/h are a MAX BUDGET and each frame is scaled to fit within it
  // preserving the camera's native aspect ratio (cur_width/cur_height report the real size); when
  // false, frames are stretched to exactly w x h (a fixed-size raw stream). See open_camera().
  sdl_camera_source(SDL_Camera *cam, int w, int h, bool native_aspect)
      : cam_(cam), w_(w), h_(h), native_(native_aspect), cur_w_(w), cur_h_(h) {}
  ~sdl_camera_source() override {
    if (scaled_)
      SDL_DestroySurface(scaled_);
    if (cam_)
      SDL_CloseCamera(cam_);
    SDL_QuitSubSystem(SDL_INIT_CAMERA); // balances the InitSubSystem open_camera handed to us
  }

  std::size_t frame_bytes() const override {
    return static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_) * 4u;
  }

  st::produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    st::produced_frame out; // bytes = 0 by default: "no frame this tick" (producer keeps running)
    const std::size_t budget = frame_bytes();
    if (!cam_ || budget == 0 || buf == nullptr || cap < budget)
      return out;

    Uint64 ts_ns = 0;
    SDL_Surface *raw = SDL_AcquireCameraFrame(cam_, &ts_ns);
    if (!raw)
      return out; // no new frame yet (warming up, or permission still pending): skip, don't block

    // Convert the camera's native pixel format to byte-order RGBA, then scale to the output size and
    // copy densely into the slab (the surface's pitch may exceed w*4). native_ decides the output
    // size: when false, stretch to exactly w_ x h_ (a fixed-size raw stream); when true, fit the
    // native frame within the w_ x h_ budget PRESERVING its aspect ratio (never upscaling), so a
    // portrait / 16:9 camera keeps its true shape (a self-describing-codec wrapper encodes at
    // cur_width/cur_height). getUserMedia's exact delivered size is unknown until frames flow, so
    // this is computed per frame from the delivered size. The same-size case does no scaling.
    SDL_Surface *rgba = (raw->format == SDL_PIXELFORMAT_RGBA32)
                            ? raw
                            : SDL_ConvertSurface(raw, SDL_PIXELFORMAT_RGBA32);
    int ow = w_, oh = h_;
    if (native_ && rgba && rgba->w > 0 && rgba->h > 0) {
      const double s = std::min(std::min(static_cast<double>(w_) / rgba->w,
                                         static_cast<double>(h_) / rgba->h),
                                1.0); // fit within the budget, never upscale
      ow = std::max(1, std::min(w_, static_cast<int>(rgba->w * s + 0.5)));
      oh = std::max(1, std::min(h_, static_cast<int>(rgba->h * s + 0.5)));
    }
    SDL_Surface *frame = rgba;
    if (rgba && (rgba->w != ow || rgba->h != oh) && ow > 0 && oh > 0) {
      if (scaled_ && (scaled_->w != ow || scaled_->h != oh)) {
        SDL_DestroySurface(scaled_); // output size changed (native camera size settled) -> re-make
        scaled_ = nullptr;
      }
      if (!scaled_)
        scaled_ = SDL_CreateSurface(ow, oh, SDL_PIXELFORMAT_RGBA32);
      // SDL_BlitSurfaceScaled resamples src -> the whole dst; nullptr rects = full surfaces.
      frame = (scaled_ && SDL_BlitSurfaceScaled(rgba, nullptr, scaled_, nullptr, SDL_SCALEMODE_LINEAR))
                  ? scaled_
                  : nullptr; // scale failed -> skip this frame (bytes=0), never an OOB copy
    }
    if (frame && frame->w == ow && frame->h == oh && frame->pixels) {
      const int row = ow * 4;
      const auto *src = static_cast<const std::uint8_t *>(frame->pixels);
      for (int y = 0; y < oh; ++y)
        std::memcpy(buf + static_cast<std::size_t>(y) * row,
                    src + static_cast<std::size_t>(y) * frame->pitch, static_cast<std::size_t>(row));
      cur_w_ = ow;
      cur_h_ = oh;
      out.bytes = static_cast<std::size_t>(ow) * static_cast<std::size_t>(oh) * 4u;
      out.pts_seconds = static_cast<double>(ts_ns) / 1.0e9;
    }
    if (rgba && rgba != raw)
      SDL_DestroySurface(rgba);
    SDL_ReleaseCameraFrame(cam_, raw);
    return out; // bytes stays 0 (skip) if the conversion/scale failed
  }

  int cur_width() const override { return cur_w_; }
  int cur_height() const override { return cur_h_; }

private:
  SDL_Camera *cam_;
  int w_;
  int h_;
  bool native_;                   // true: fit-to-budget preserving aspect; false: stretch to w_ x h_
  int cur_w_, cur_h_;             // actual size of the last produced frame (cur_width/cur_height)
  SDL_Surface *scaled_ = nullptr; // lazily-created output-size scratch for rescaling off-size frames
};
} // namespace
#endif // CVC_ENABLE_SDL

camera_open open_camera(int device_index, int want_w, int want_h, double want_fps,
                        bool native_aspect) {
  camera_open result;
#if CVC_ENABLE_SDL
  if (!SDL_InitSubSystem(SDL_INIT_CAMERA))
    return result; // refcounted; the source's dtor (or the cleanup below) quits it
  bool handed_off = false;
  int count = 0;
  SDL_CameraID *ids = SDL_GetCameras(&count);
  if (ids && device_index >= 0 && device_index < count) {
    const SDL_CameraID id = ids[device_index];
    const char *nm = SDL_GetCameraName(id);
    std::string name = nm ? nm : std::string();

#if defined(__EMSCRIPTEN__)
    // Browser camera (getUserMedia). Permission AND the device format resolve ASYNCHRONOUSLY through
    // the browser event loop, so in a single-threaded wasm build we must NOT block for them: the
    // event loop cannot run while we spin, so the grant would never arrive and the bounded native
    // wait below would always time out (and SDL_GetCameraSupportedFormats reports nothing until the
    // stream is live). So request the desired size, open WITHOUT waiting, and hand back a source
    // sized to the request immediately. fill() skips (SDL_AcquireCameraFrame returns null) until the
    // user approves and frames flow, and whatever size getUserMedia actually delivers is rescaled to
    // the request by fill(); the app's normal per-frame SDL_PumpEvents (pump_events) services the
    // approval + frame-ready events.
    const int rw = want_w > 0 ? want_w : 640;
    const int rh = want_h > 0 ? want_h : 480;
    SDL_CameraSpec want{};
    want.format = SDL_PIXELFORMAT_RGBA32; // byte-order RGBA; fill() converts anything else regardless
    want.width = rw;
    want.height = rh;
    if (want_fps > 0.0) {
      want.framerate_numerator = static_cast<int>(want_fps + 0.5);
      want.framerate_denominator = 1;
    }
    SDL_Camera *cam = SDL_OpenCamera(id, &want);
    if (cam) {
      result.width = rw;
      result.height = rh;
      result.fps = want_fps > 0.0 ? want_fps : 30.0;
      result.name = std::move(name);
      result.source = std::make_unique<sdl_camera_source>(cam, rw, rh, native_aspect);
      handed_off = true; // the source now owns `cam` and the subsystem ref
    }
#else
    // Webcams usually DEFAULT to MJPG (compressed): SDL then hands fill() the raw JPEG bytes in an
    // SDL_PIXELFORMAT_MJPG surface, and SDL_ConvertSurface does NOT decode JPEG — it reinterprets
    // the compressed bytes as pixels (garbage: vertical stripes). SDL also has no built-in camera
    // MJPG decoder here. So enumerate the device's formats and REQUEST an uncompressed one; SDL
    // then delivers raw YUY2/RGB frames that convert cleanly to rgba8. Pick the uncompressed format
    // with the best smoothness×resolution, capping fps at 30 so we don't trade away a lot of
    // resolution for >30 fps — but strongly PREFER an exact match for the requested w x h when one is
    // given (so the caller's stream size is honoured; fill() rescales any residual mismatch). If the
    // device is compressed-ONLY (no uncompressed format), fall back to SDL's default (nullptr).
    SDL_CameraSpec chosen{};
    bool have_spec = false;
    {
      int fc = 0;
      SDL_CameraSpec **fmts = SDL_GetCameraSupportedFormats(id, &fc);
      double best = -1.0;
      for (int i = 0; fmts && i < fc; ++i) {
        const SDL_CameraSpec &s = *fmts[i];
        if (s.format == SDL_PIXELFORMAT_MJPG || s.format == SDL_PIXELFORMAT_UNKNOWN)
          continue; // compressed/unknown: SDL_ConvertSurface cannot decode it to rgba8
        const double fps =
            (s.framerate_denominator > 0)
                ? static_cast<double>(s.framerate_numerator) / s.framerate_denominator
                : 0.0;
        double score = (fps > 30.0 ? 30.0 : fps) * static_cast<double>(s.width) *
                       static_cast<double>(s.height);
        if (want_w > 0 && want_h > 0 && s.width == want_w && s.height == want_h)
          score += 1.0e12; // honour the caller's requested size when the device offers it
        if (score > best) {
          best = score;
          chosen = s;
          have_spec = true;
        }
      }
      if (fmts)
        SDL_free(fmts);
      if (have_spec)
        std::fprintf(stderr, "[camera] requesting %s %dx%d @ %.3g fps (uncompressed)\n",
                     SDL_GetPixelFormatName(chosen.format), chosen.width, chosen.height,
                     chosen.framerate_denominator > 0
                         ? static_cast<double>(chosen.framerate_numerator) /
                               chosen.framerate_denominator
                         : 0.0);
      else
        std::fprintf(stderr, "[camera] no uncompressed format available; using device default "
                             "(frames may be undecodable)\n");
    }
    SDL_Camera *cam = SDL_OpenCamera(id, have_spec ? &chosen : nullptr);
    if (cam) {
      // SDL delivers camera access asynchronously: the device is unusable (and GetCameraFormat
      // reports nothing) until it is APPROVED, which arrives through the event system. Pump events
      // and wait, bounded, for the permission to resolve (approved or denied) so the returned
      // source is ready to capture. A denial leaves a source that simply never produces (fill()
      // skips).
      for (int i = 0;
           i < 100 && SDL_GetCameraPermissionState(cam) == SDL_CAMERA_PERMISSION_STATE_PENDING;
           ++i) {
        SDL_PumpEvents();
        SDL_Delay(10); // up to ~1 s
      }
      SDL_CameraSpec spec;
      if (SDL_GetCameraPermissionState(cam) == SDL_CAMERA_PERMISSION_STATE_APPROVED &&
          SDL_GetCameraFormat(cam, &spec) && spec.width > 0 && spec.height > 0) {
        // Honour the caller's requested size when given (fill() rescales the camera frames to it);
        // otherwise adopt the camera's negotiated size.
        const int sw = (want_w > 0 ? want_w : spec.width);
        const int sh = (want_h > 0 ? want_h : spec.height);
        result.width = sw;
        result.height = sh;
        result.fps =
            (spec.framerate_denominator > 0)
                ? static_cast<double>(spec.framerate_numerator) / spec.framerate_denominator
                : 0.0;
        result.name = std::move(name);
        result.source = std::make_unique<sdl_camera_source>(cam, sw, sh, native_aspect);
        handed_off = true; // the source now owns `cam` and the subsystem ref
      } else {
        SDL_CloseCamera(cam);
      }
    }
#endif // __EMSCRIPTEN__
  }
  if (ids)
    SDL_free(ids);
  if (!handed_off)
    SDL_QuitSubSystem(SDL_INIT_CAMERA); // nothing took the ref we added above
#else
  (void)device_index;
  (void)want_w;
  (void)want_h;
  (void)want_fps;
  (void)native_aspect;
#endif
  return result;
}

void pump_events() {
#if CVC_ENABLE_SDL
  SDL_PumpEvents(); // process camera device approval + frame-ready notifications (main thread)
#endif
}

std::vector<std::string> list_cameras() {
  std::vector<std::string> names;
#if CVC_ENABLE_SDL
  if (!SDL_InitSubSystem(SDL_INIT_CAMERA))
    return names;
  int count = 0;
  SDL_CameraID *ids = SDL_GetCameras(&count);
  if (ids) {
    for (int i = 0; i < count; ++i) {
      const char *nm = SDL_GetCameraName(ids[i]);
      names.emplace_back(nm ? nm : "");
    }
    SDL_free(ids);
  }
  SDL_QuitSubSystem(SDL_INIT_CAMERA);
#endif
  return names;
}

} // namespace capture
} // namespace gl
} // namespace cvc
