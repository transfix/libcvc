/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_WEB_AUDIO_H__
#define __CVC_MEDIA_WEB_AUDIO_H__

#include <cvc/ariadne/stream/frame_source.h>
#include <cvc/media/playback/audio_output.h>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
namespace media {
namespace web_audio {

// ----------------
// web_audio: a browser's mic and speaker through AudioWorklets (ARIADNE_CONFERENCE_ROADMAP §5j)
// ----------------
// SDL's Web Audio backend plays and captures through a ScriptProcessorNode, whose callback runs
// on the page's main thread, in 43 ms blocks, between the frames of arigl's loop. A long frame
// makes both late, and the speaker plays a gap. In a hidden tab the loop draws no frames and the
// page's timers run about once a second, so the mic is read and sent about once a second.
//
// Here the audio thread does the device's work, in two AudioWorklet processors
// (src/cvcMedia/web/cvc-audio-worklet.js). They hold no wasm and share no memory with it: blocks
// of 20 ms go over each node's MessagePort. So this works in a single-threaded build, where the
// page has one thread for the program, and on a page that is not cross-origin isolated. The wasm
// side still runs on the main thread, but on each block's message, which the browser delivers
// between frames and in a hidden tab at full rate:
//
//   - capture: each block goes into the source's queue, and the stream's producer fills from it at
//     once (frame_source::set_ready_callback), not at its next timer tick;
//   - playback: the playback node keeps a ring of the mix. On each of its ticks the sink pulls
//     what the device took from FILL and posts it back, so the ring stays TARGET ahead of the
//     speaker. A main thread busy for less than TARGET leaves the speaker playing. When the ring
//     runs dry, TARGET grows by the silence it played and a block (to 400 ms at most), and it
//     shrinks a block after 10 s without running dry;
//   - a clock: each block's message runs every clock's function first, so a program can do its
//     audio work at the audio's pace (the conference sends its mic and feeds its jitter buffers
//     there).
//
// The page's one AudioContext runs at 48 kHz where the browser allows it; a capture or an output
// at another rate, or with more than two channels, is not opened here, and its caller uses SDL.
// The context starts suspended until the page is touched (a browser's autoplay policy); the
// arigl-web page resumes every AudioContext on the first click, key or touch.
//
// Off by the environment: CVC_WEB_AUDIO=sdl keeps SDL for everything (the A/B). Natively nothing
// here opens, and a clock never ticks.

// Start loading the worklets into the page's audio context, once; true when they are loaded or
// loading. Asynchronous: a capture or an output opened while they load gets its node once they
// have loaded (a few milliseconds). If they fail to load, a device opened before that stays silent
// and one opened after uses SDL; the console says so.
bool prepare();

// The worklets are loaded.
bool ready();

// What this page's web audio is: "ready", "loading", "off (CVC_WEB_AUDIO=sdl)", "unsupported
// (...)", or "failed: ...". "native" off a browser.
std::string status();

// The mic: a FIFO frame_source of FRAMES_PER_CHUNK frames of CHANNELS interleaved f32 at RATE,
// fed by the capture node. The browser asks for the mic's permission here, and until it is given
// the source produces nothing. Like SDL's mic, it drops the oldest audio past 250 ms queued.
// Null unless the worklets are loaded or loading (prepare()), or when the format is not the
// context's.
std::unique_ptr<cvc::ariadne::stream::frame_source> open_capture(int rate, int channels,
                                                                 int frames_per_chunk);

// The speaker, pulling its audio from FILL (open_audio_output_pull's contract: FILL writes the
// frames asked for, silence included). FILL runs on the main thread, on the playback node's
// ticks, after the clocks. device_period_s() is the block (20 ms); queued_bytes() is what the
// ring holds. play() posts audio ahead of FILL (dropped while the worklets load). Null as for
// open_capture.
std::unique_ptr<cvc::media::playback::audio_output>
open_output_pull(int rate, int channels, const cvc::media::playback::audio_pull_fn &fill);

// A function run on every block's message (each capture block and each playback tick, about 50
// a second for each node open), on the main thread, between frames. The owner keeps the clock;
// destroying it stops the calls.
class audio_clock {
public:
  explicit audio_clock(std::function<void()> tick);
  ~audio_clock();
  audio_clock(const audio_clock &) = delete;
  audio_clock &operator=(const audio_clock &) = delete;

  void run(); // the page's call (do not call)

private:
  std::function<void()> _tick;
  bool _running = false; // in _tick now (a tick that opens a node does not re-enter it)
};

} // namespace web_audio
} // namespace media
} // namespace cvc

#endif // __CVC_MEDIA_WEB_AUDIO_H__
