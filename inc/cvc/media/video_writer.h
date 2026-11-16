/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_VIDEO_WRITER_H__
#define __CVC_MEDIA_VIDEO_WRITER_H__

// cvcMedia's video writer: RGB frames in, an MP4 file out (arigl snapshot --mp4). Two ways to
// encode, tried in this order by Backend::Automatic:
//
//   1. In process, through the FFmpeg libraries cvcMedia's player links (libavcodec, libavformat,
//      libswscale), with an H.264 encoder they have: libx264, else libopenh264.
//   2. An `ffmpeg` program on PATH, fed raw RGB on its stdin; it picks its own MP4 codec (H.264
//      when it has libx264, else MPEG-4 Part 2).
//   3. In process with FFmpeg's own MPEG-4 Part 2 encoder (cvcpkg's ffmpeg-lgpl has no H.264
//      encoder): a valid MP4 most players play, though a browser's <video> may not.
//
// With none of them (a cvcMedia built without FFmpeg and no ffmpeg program, or a browser),
// video_encoder() is "" and says why, and open() fails the same way. FFmpeg stays an optional,
// shared-library dependency of cvcMedia alone (src/cvcMedia/CMakeLists.txt).
//
// Frames are RGB, 3 bytes a pixel, width x height of them, rows top-down unless `bottom_up` (GL's
// and VTK's readback order, SceneRenderer::frameRGB). Each frame lasts 1 / fps seconds.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace media {

struct VideoWriterOptions {
  int width = 0, height = 0; // pixels; both even (H.264's 4:2:0 halves them)
  int fps = 30;
  bool bottom_up = false; // the rows come bottom-up: flipped as they are written
  enum class Backend {
    Automatic, // as above: in process with H.264, else the program, else in process MPEG-4
    Library,   // in process only (H.264, else MPEG-4)
    Program    // the ffmpeg program only
  };
  Backend backend = Backend::Automatic;
  std::string program; // Backend::Program / Automatic: the ffmpeg to run ("" = ffmpeg on PATH)
};

// Whether this cvcMedia was built with FFmpeg's libraries (the in-process encoder).
bool have_video_library();

// The encoder open() would use with `opt` ("libx264", "mpeg4", "ffmpeg program (PATH)"), or ""
// when there is none, with the reason in `why`.
std::string video_encoder(const VideoWriterOptions &opt, std::string *why = nullptr);

// The `ffmpeg` program PATH names, or "" (never in a browser).
std::string find_ffmpeg_program();

class VideoWriter {
public:
  VideoWriter();
  ~VideoWriter(); // close()s an open file
  VideoWriter(const VideoWriter &) = delete;
  VideoWriter &operator=(const VideoWriter &) = delete;

  // Create `path` (replacing it). False with error() set.
  bool open(const std::string &path, const VideoWriterOptions &opt);
  // One frame: width * height * 3 bytes. False with error() set (the writer is then closed).
  bool write_rgb(const unsigned char *rgb, std::size_t bytes);
  // Flush the encoder and finish the file. False with error() set. Safe to call twice.
  bool close();

  bool is_open() const;
  const std::string &error() const;
  const std::string &encoder() const; // what open() chose (video_encoder's names)
  int frames() const;                 // frames written

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// What a video file holds, decoded frame by frame through the same FFmpeg libraries: for tests
// and tools. ok is false (with `error`) when the file can't be read, or without FFmpeg.
struct VideoInfo {
  bool ok = false;
  std::string error;
  std::string format; // the container, as FFmpeg names it ("mov,mp4,m4a,3gp,3g2,mj2")
  std::string codec;  // "h264", "mpeg4", ...
  int width = 0, height = 0;
  int frames = 0;                                 // decoded
  double fps = 0;                                 // the stream's average frame rate
  double duration = 0;                            // seconds: the last frame's end
  std::vector<unsigned char> first_rgb, last_rgb; // top-down RGB, when asked for
};
VideoInfo probe_video(const std::string &path, bool keep_first_and_last = false);

} // namespace media
} // namespace cvc

#endif // __CVC_MEDIA_VIDEO_WRITER_H__
