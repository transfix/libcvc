/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_PLAYER_H__
#define __CVC_MEDIA_PLAYER_H__

// cvcMedia's player for Ariadne's video: and audio: widgets (docs/ARIADNE.md §5.13): FFmpeg demuxes
// and decodes (libavformat, libavcodec), frames are scaled to the widget's box (libswscale) and
// sound is resampled to 48 kHz stereo f32 (libswresample) and played through SDL3's audio
// subsystem (cvc::media::playback::audio_output). Sound is the clock; with no sound (no audio
// stream, no device, muted is still sound) the wall clock is.
//
// FFmpeg is linked as shared libraries (cvcpkg's ffmpeg-lgpl: LGPL-2.1, the decoders only), so
// the LGPL's terms are kept by dynamic linking; cvcMedia itself is LGPL-2.1 like libcvc. Without
// FFmpeg in the build, have_ffmpeg_player() is false and register_ariadne_player() registers
// nothing (the widgets then say `unsupported`).
//
// The source is fetched whole first (ariadne::media::fetch_bytes: the host's network policy and
// credentials, file://, cvc://, state://), at most 64 MiB, and read from memory.

#include <cvc/ariadne/media_player.h>
#include <memory>

namespace cvc {
namespace media {

// Whether this cvcMedia was built with FFmpeg.
bool have_ffmpeg_player();

// Make cvcMedia's player the one Ariadne's video: and audio: widgets use
// (ariadne::media::register_player_factory). Returns false, registering nothing, without FFmpeg.
// ari-media and arigl call it at setup.
bool register_ariadne_player();

struct PlayerOptions {
  bool audio = true; // play sound through SDL3 (false: decode and time it, play nothing: tests)
};

// A player for `src` (null without FFmpeg). Construct and use it on one thread (the frame thread):
// SDL's audio device is opened and closed there, as SDL wants.
std::unique_ptr<cvc::ariadne::media::Player>
make_ffmpeg_player(const cvc::ariadne::media::PlayerSource &src, const PlayerOptions &opt = {});

} // namespace media
} // namespace cvc

#endif // __CVC_MEDIA_PLAYER_H__
