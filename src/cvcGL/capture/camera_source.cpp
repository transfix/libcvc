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
#include <cstdint>
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
  sdl_camera_source(SDL_Camera *cam, int w, int h) : cam_(cam), w_(w), h_(h) {}
  ~sdl_camera_source() override {
    if (cam_)
      SDL_CloseCamera(cam_);
    SDL_QuitSubSystem(SDL_INIT_CAMERA); // balances the InitSubSystem open_camera handed to us
  }

  std::size_t frame_bytes() const override {
    return static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_) * 4u;
  }

  st::produced_frame fill(std::uint8_t *buf, std::size_t cap) override {
    st::produced_frame out; // bytes = 0 by default: "no frame this tick" (producer keeps running)
    const std::size_t need = frame_bytes();
    if (!cam_ || need == 0 || buf == nullptr || cap < need)
      return out;

    Uint64 ts_ns = 0;
    SDL_Surface *raw = SDL_AcquireCameraFrame(cam_, &ts_ns);
    if (!raw)
      return out; // no new frame yet (warming up, or permission still pending): skip, don't block

    // Convert the camera's native pixel format to byte-order RGBA (no rescale — same w/h), then
    // copy it densely into the slab (the camera surface's pitch may exceed w*4).
    SDL_Surface *rgba = (raw->format == SDL_PIXELFORMAT_RGBA32)
                            ? raw
                            : SDL_ConvertSurface(raw, SDL_PIXELFORMAT_RGBA32);
    if (rgba && rgba->w == w_ && rgba->h == h_ && rgba->pixels) {
      const int row = w_ * 4;
      const auto *src = static_cast<const std::uint8_t *>(rgba->pixels);
      for (int y = 0; y < h_; ++y)
        std::memcpy(buf + static_cast<std::size_t>(y) * row,
                    src + static_cast<std::size_t>(y) * rgba->pitch, static_cast<std::size_t>(row));
      out.bytes = need;
      out.pts_seconds = static_cast<double>(ts_ns) / 1.0e9;
    }
    if (rgba && rgba != raw)
      SDL_DestroySurface(rgba);
    SDL_ReleaseCameraFrame(cam_, raw);
    return out; // bytes stays 0 (skip) if the conversion failed or the size disagreed
  }

private:
  SDL_Camera *cam_;
  int w_;
  int h_;
};
} // namespace
#endif // CVC_ENABLE_SDL

camera_open open_camera(int device_index) {
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
    SDL_Camera *cam = SDL_OpenCamera(id, nullptr); // let SDL pick the device's native format
    if (cam) {
      // SDL delivers camera access asynchronously: the device is unusable (and GetCameraFormat
      // reports nothing) until it is APPROVED, which arrives through the event system. Pump events
      // and wait, bounded, for the permission to resolve (approved or denied) so the returned
      // source is ready to capture. A denial leaves a source that simply never produces (fill()
      // skips).
      for (int i = 0; i < 100 && SDL_GetCameraPermissionState(cam) == 0; ++i) {
        SDL_PumpEvents();
        SDL_Delay(10); // up to ~1 s
      }
      SDL_CameraSpec spec;
      if (SDL_GetCameraPermissionState(cam) == 1 && SDL_GetCameraFormat(cam, &spec) &&
          spec.width > 0 && spec.height > 0) {
        result.width = spec.width;
        result.height = spec.height;
        result.fps =
            (spec.framerate_denominator > 0)
                ? static_cast<double>(spec.framerate_numerator) / spec.framerate_denominator
                : 0.0;
        result.name = std::move(name);
        result.source = std::make_unique<sdl_camera_source>(cam, spec.width, spec.height);
        handed_off = true; // the source now owns `cam` and the subsystem ref
      } else {
        SDL_CloseCamera(cam);
      }
    }
  }
  if (ids)
    SDL_free(ids);
  if (!handed_off)
    SDL_QuitSubSystem(SDL_INIT_CAMERA); // nothing took the ref we added above
#else
  (void)device_index;
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
