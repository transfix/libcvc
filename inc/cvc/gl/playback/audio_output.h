/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__
#define __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__

// The SDL3 audio output moved to cvcMedia, libcvc's GL-free media library, so a terminal program
// can play sound without GL (cvc/media/playback/audio_output.h). cvcGL links cvcMedia, and these
// are its old names for the same types and functions.

#include <cvc/media/playback/audio_output.h>

namespace cvc {
namespace gl {
namespace playback {

using cvc::media::playback::audio_output;
using cvc::media::playback::audio_output_open;
using cvc::media::playback::audio_output_spec;
using cvc::media::playback::list_audio_playback_devices;
using cvc::media::playback::open_audio_output;

} // namespace playback
} // namespace gl
} // namespace cvc

#endif // __CVC_GL_PLAYBACK_AUDIO_OUTPUT_H__
