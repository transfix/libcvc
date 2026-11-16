/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#pragma once

// A document's scene, run on its clock and recorded: `arigl snapshot --mp4 FILE --seconds S`
// (docs/ARIADNE.md, the hosts' command line). Each frame steps the scene clock by exactly 1 / fps
// of scene time, whatever the wall time (AriRuntime::set_fixed_clock), so the physics, the clock's
// tickers (water, clouds) and the custom nodes' ticks advance with it, and a slow machine makes the
// same video, only more slowly. (The clock runs at most max_steps_per_advance quanta a frame, so a
// recorded frame longer than that, under 15 fps by default, runs as several of the runtime's.) Each
// frame is read back from the offscreen view and encoded (cvc::media::VideoWriter); the last can go
// to a PNG as well. The clock's own speed (its scale:, a Speed control) applies, as in a run.

#include <cvc/media/video_writer.h>
#include <functional>
#include <string>

namespace cvc {
namespace gl {
class SceneRenderer;
namespace ariadne {
class AriRuntime;

struct RecordOptions {
  std::string mp4;     // the video to write ("" = run the frames and record none)
  std::string png;     // also write the last frame here ("" = no PNG)
  double seconds = 0;  // scene time recorded: round(seconds * fps) frames, at least one
  int fps = 30;        // frames a second, of scene time and of the video
  int lead_frames = 0; // frames run first, each 1 / fps of scene time too, and not recorded
  // Pace the frames no faster than real time, as a snapshot's are, so a live stream's producer and
  // the document's background work keep up with them. Off, they run as fast as they render.
  bool pace = true;
  std::function<void()> before_frame; // the host's own work each frame (arigl's conference pump)
  cvc::media::VideoWriterOptions::Backend backend =
      cvc::media::VideoWriterOptions::Backend::Automatic;
};

struct RecordResult {
  bool ok = false;
  std::string error;   // why not
  std::string encoder; // what encoded the video ("" without one)
  int frames = 0;      // frames recorded (fewer than asked when the document quit)
  int width = 0, height = 0;
  double scene_seconds = 0; // scene time the run covered, lead frames included
};

// Run and record rt's loaded document, drawn by `view` (the SceneRenderer rt draws in, offscreen).
// Leaves rt's clock fixed. Stops early when the document quits (AriRuntime::should_close).
RecordResult record_frames(AriRuntime &rt, SceneRenderer &view, const RecordOptions &opt);

// The size and backend `opt` records with, as VideoWriterOptions: what a host checks before it
// loads the document (cvc::media::video_encoder).
cvc::media::VideoWriterOptions record_writer_options(const RecordOptions &opt, int width,
                                                     int height);

} // namespace ariadne
} // namespace gl
} // namespace cvc
