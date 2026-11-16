/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A document's scene recorded on its clock — see inc/cvc/gl/ariadne/record.h.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/record.h>
#include <thread>
#include <vector>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkPNGWriter.h>

namespace cvc {
namespace gl {
namespace ariadne {

namespace {

// The frame frameRGB read back (bottom-up rows, VTK's order) as a PNG.
bool write_png(const std::string &path, const std::vector<unsigned char> &rgb, int w, int h) {
  if (rgb.size() != static_cast<std::size_t>(w) * h * 3)
    return false;
  vtkNew<vtkImageData> img;
  img->SetDimensions(w, h, 1);
  img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
  std::memcpy(img->GetScalarPointer(), rgb.data(), rgb.size());
  vtkNew<vtkPNGWriter> writer;
  writer->SetFileName(path.c_str());
  writer->SetInputData(img);
  writer->Write();
  return writer->GetErrorCode() == 0;
}

// How many frames of the runtime one recorded frame of `dt` seconds runs as. The scene clock runs
// at most max_steps_per_advance quanta a frame (its guard against a stalled frame) and drops the
// rest, so a frame longer than that (under 15 fps at the default 1/120 s quantum and speed 1) runs
// as several shorter ones, and every 1/fps of it lands on the clock.
int sub_frames(const cvc::world_clock &clock, double dt) {
  const cvc::world_clock::config cfg = clock.current_config();
  const double speed = std::max(1.0, clock.scale());
  double cap = cfg.max_steps_per_advance * cfg.fixed_dt; // the world seconds a frame may run
  if (clock.cap_scales_with_rate())
    cap *= std::ceil(speed);
  const double room = cap - cfg.fixed_dt; // a banked remainder may add up to a quantum
  if (!(room > 0))
    return 1;
  return std::max(1, static_cast<int>(std::ceil(dt * speed / room - 1e-9)));
}

} // namespace

cvc::media::VideoWriterOptions record_writer_options(const RecordOptions &opt, int width,
                                                     int height) {
  cvc::media::VideoWriterOptions w;
  w.width = width;
  w.height = height;
  w.fps = opt.fps > 0 ? opt.fps : 30;
  w.bottom_up = true; // SceneRenderer::frameRGB's rows
  w.backend = opt.backend;
  return w;
}

RecordResult record_frames(AriRuntime &rt, SceneRenderer &view, const RecordOptions &opt) {
  RecordResult out;
  const int fps = opt.fps > 0 ? opt.fps : 30;
  const double dt = 1.0 / fps;
  const int frames = std::max(1, static_cast<int>(std::lround(opt.seconds * fps)));
  out.width = view.frameWidth();
  out.height = view.frameHeight();

  cvc::media::VideoWriter video;
  if (!opt.mp4.empty()) {
    if (!video.open(opt.mp4, record_writer_options(opt, out.width, out.height))) {
      out.error = opt.mp4 + ": " + video.error();
      return out;
    }
    out.encoder = video.encoder();
  }

  rt.set_fixed_clock(true); // the scene clock steps dt a frame, not the wall time
  const auto period = std::chrono::duration<double>(dt);
  std::vector<unsigned char> rgb;
  const int total = std::max(0, opt.lead_frames) + frames;
  const cvc::world_clock &clock = rt.app_runtime().app().world_clock();
  for (int f = 0; f < total; ++f) {
    const auto start = std::chrono::steady_clock::now();
    if (opt.before_frame)
      opt.before_frame();
    // The clock's speed can change from frame to frame (the document's Speed control).
    const int subs = sub_frames(clock, dt);
    for (int s = 0; s < subs && (s == 0 || !rt.should_close()); ++s)
      rt.frame(dt / subs);
    out.scene_seconds += dt;
    // (quit code) ends the run at the frame it quit on, as a snapshot's does: that frame is the
    // last one recorded.
    const bool last = f + 1 == total || rt.should_close();
    const bool recorded = f >= total - frames;
    if (recorded && (video.is_open() || (last && !opt.png.empty()))) {
      rgb = view.frameRGB(); // renders it
      if (video.is_open() && !video.write_rgb(rgb.data(), rgb.size())) {
        out.error = opt.mp4 + ": " + video.error();
        return out;
      }
    } else {
      rt.render();
    }
    if (recorded)
      ++out.frames;
    if (last)
      break;
    if (opt.pace)
      std::this_thread::sleep_until(start + period);
  }
  if (video.is_open() && !video.close()) {
    out.error = opt.mp4 + ": " + video.error();
    return out;
  }
  if (!opt.png.empty()) {
    if (rgb.empty()) // the document quit during the lead frames: the frame it quit on
      rgb = view.frameRGB();
    if (!write_png(opt.png, rgb, out.width, out.height)) {
      out.error = opt.png + ": could not write the PNG";
      return out;
    }
  }
  out.ok = true;
  return out;
}

} // namespace ariadne
} // namespace gl
} // namespace cvc
