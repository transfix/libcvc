/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_ARIADNE_CONFERENCE_VOICE_MONITOR_H__
#define __CVC_ARIADNE_CONFERENCE_VOICE_MONITOR_H__

#include <cvc/media/voice_mixer.h>
#include <deque>
#include <map>
#include <string>

namespace cvc {
class app;
namespace ariadne {
namespace conference {

// ----------------
// voice_monitor: a conference client's jitter buffers, for its document to show and to tune
// ----------------
// The host (arigl's ConferenceSession) calls step() each frame with its cvc::voice_mixer. Every
// period_s it samples each peer's jitter buffer and publishes, under PREFIX (the document's state
// prefix; nothing here is sent to the room):
//   media.<pid>.jb.buffered   what it held, ms, over the last history samples, oldest first,
//                             comma-separated (a chart: series)
//   media.<pid>.jb.target     its target at each of those samples, likewise
//   media.<pid>.jb.now        one line: its target and what it holds now, and since it joined how
//                             often it ran dry (and how often that was late audio) and how much
//                             was trimmed
//   audio.device_ms           the output's block
//   audio.ahead_ms            what the output holds beyond the mixer (a browser's AudioWorklet
//                             ring; 0 for SDL's device)
//   audio.window_s            how long the history covers
// It reads the tuning under PREFIX.audio.jb, which describe() creates with the metadata a
// settings: editor reads (cvc/state/state_key_meta.h), and applies a change with
// voice_mixer::set_params:
//   mode            adaptive (targets grow after late audio and relax back) or fixed
//   adaptive.min, .start, .max   seconds: the least target, a new peer's, the greatest
//   adaptive.relax  seconds a voice plays without running dry before its target shrinks a step
//   fixed.delay     seconds: every target (fixed)
// The device's floor (voice_mixer::set_delay_floor) still holds: a target below it is raised.
// PREFIX.audio.reset set to 1 puts every target back to start (voice_mixer::reset_targets), and
// back to 0. cvc://components/audio_buffers.ari is the window over all of it.
class voice_monitor {
public:
  voice_monitor(cvc::app &app, std::string prefix, double period_s = 0.2,
                std::size_t history = 150);

  // The tuning keys at their defaults (or as the document left them), described. Call once, at
  // join.
  void describe();

  // Sample and publish (at most every period_s) and apply the tuning, at NOW (seconds, any
  // monotonic clock). DEVICE_S is the output's block, AHEAD_S what it holds beyond the mixer.
  void step(cvc::voice_mixer &mx, double now, double device_s, double ahead_s);

  // Forget PID's history (it left); its keys go with its media subtree.
  void forget(const std::string &pid);

  // The mixer parameters the tuning keys ask for (for tests and the host's log).
  cvc::voice_mixer::params wanted() const { return _applied; }

private:
  struct history {
    std::deque<int> buffered, target; // ms
  };
  std::string key(const std::string &rel) const;
  cvc::voice_mixer::params read_tuning();

  cvc::app &_app;
  std::string _prefix;
  double _period_s;
  std::size_t _history;
  double _next = -1.0;
  bool _have_applied = false;
  cvc::voice_mixer::params _applied;
  std::map<std::string, history> _peers;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_VOICE_MONITOR_H__
