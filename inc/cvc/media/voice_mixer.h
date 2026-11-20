/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_MEDIA_VOICE_MIXER_H__
#define __CVC_MEDIA_VOICE_MIXER_H__

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {

// ----------------
// voice_mixer: every peer's voice in one output, each through a jitter buffer
// ----------------
// A conference client hears every peer through one playback device. Peers' audio arrives in
// bursts: the sender captures in blocks (a browser's in 43 ms blocks), sends once a frame, and
// the network and the relay add their own jitter. Played as it lands, a voice breaks up wherever a
// packet is late. So each peer has a jitter buffer: its audio starts playing once TARGET seconds
// of it are buffered, and the device pulls from the buffers at its own pace (pull()).
//
// When a playing peer's buffer runs dry, the voice fades out and that peer waits for TARGET
// again. If its audio comes back soon (within late_window_s), the buffer was too small for the
// jitter on that link, and TARGET grows a step; after relax_after_s of playing without running
// dry it shrinks a step, back toward min_delay_s. A peer that stops sending (the sender sends
// nothing through a silence) runs dry too, but sends nothing soon after, so its target stands. A
// burst after a stall (more than twice TARGET buffered, and at least TARGET plus trim_slack_s)
// drops its oldest audio down to TARGET, cross-fading across the cut; that keeps the delay from
// growing.
//
// THREADING: push(), set_gain() and the readers run on the app's thread; pull() runs on the audio
// device's (SDL's audio thread natively; the page's audio callback in a single-threaded browser
// build). One mutex covers both. TIMES are seconds on any one monotonic clock, the same for every
// call (steady_clock in the conference host); the tests pass their own.
class voice_mixer {
public:
  struct params {
    double start_delay_s = 0.08; // a new peer's target
    double min_delay_s = 0.04;   // the least target
    double max_delay_s = 0.40;   // the greatest target
    double step_s = 0.02;        // the target grows by this after a late underrun
    double relax_after_s = 10.0; // and shrinks by it after this long playing without one
    double late_window_s = 0.30; // audio back this soon after an underrun: the underrun was late
    double trim_slack_s = 0.12;  // more than target + max(this, target): drop down to the target
    double max_buffer_s = 2.0;   // a peer never holds more than this
    double fade_s = 0.0025;      // the ramp on a voice's start and on its running dry
  };

  struct peer_stats {
    double target_s = 0.0;             // the buffer's target now
    double buffered_s = 0.0;           // what it holds now
    bool playing = false;              // being pulled from (false: filling toward the target)
    std::uint64_t starts = 0;          // times it started playing
    std::uint64_t underruns = 0;       // times it ran dry while playing
    std::uint64_t late_underruns = 0;  // of those, the ones its audio came back soon after
    std::uint64_t trimmed_frames = 0;  // frames dropped as a burst beyond the target
    std::uint64_t overflow_frames = 0; // frames dropped at max_buffer_s
    std::uint64_t played_frames = 0;   // frames mixed into the output
  };

  // The output's format: RATE frames a second of CHANNELS interleaved floats. Every push() is in
  // it.
  voice_mixer(int rate, int channels);
  voice_mixer(int rate, int channels, const params &p);

  int rate() const noexcept { return _rate; }
  int channels() const noexcept { return _channels; }
  params parameters() const;

  // Raise the least and the starting targets to at least MIN_S and START_S, e.g. for a device
  // that pulls in large blocks (a browser's 43 ms). Peers already known keep a target at least
  // MIN_S.
  void set_delay_floor(double min_s, double start_s);

  // Append N interleaved samples of PEER's audio, decoded, arriving at NOW. A trailing partial
  // frame is dropped.
  void push(const std::string &peer, const float *samples, std::size_t n, double now);

  // PEER's gain in the mix (its volume times its distance gain; 0 consumes it unheard). 1 until
  // set.
  void set_gain(const std::string &peer, float gain);

  // Forget PEER (it left).
  void remove(const std::string &peer);

  // Mix the next FRAMES frames of every peer into OUT (FRAMES * channels floats), clamped to
  // [-1, 1]; silence where no peer plays. Returns whether any peer contributed. For the device's
  // audio callback.
  bool pull(float *out, std::size_t frames, double now);

  // What PEER has buffered, in seconds (0 for a peer not known): with the device's own latency,
  // how far behind its latest audio the speaker is (for A/V sync).
  double buffered_s(const std::string &peer) const;

  // PEER's counters; false for a peer not known.
  bool stats(const std::string &peer, peer_stats &out) const;

  // The peers it knows.
  std::vector<std::string> peers() const;

private:
  struct peer {
    std::deque<float> buf;
    std::vector<float> cut; // a trim's head, cross-faded out of (pull's scratch)
    float gain = 1.0f;
    double target_s = 0.0;
    bool playing = false;
    bool fade_in = false;      // the next samples pulled start the voice: ramp them up
    double underrun_at = -1.0; // when it last ran dry while playing (-1: not since its audio came)
    double last_push = -1.0;   // when its audio last arrived
    double clean_s = 0.0;      // seconds played since its target last changed or it ran dry
    peer_stats st;
  };

  peer &at(const std::string &id); // _mu held
  double seconds(std::size_t samples) const;
  std::size_t samples(double seconds) const;
  void trim(peer &p, std::size_t keep); // _mu held

  int _rate, _channels;
  params _p;
  mutable std::mutex _mu;
  std::map<std::string, peer> _peers;
};

} // namespace cvc

#endif // __CVC_MEDIA_VOICE_MIXER_H__
