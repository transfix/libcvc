/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cmath>
#include <cvc/media/voice_mixer.h>

namespace cvc {

voice_mixer::voice_mixer(int rate, int channels) : voice_mixer(rate, channels, params()) {}

voice_mixer::voice_mixer(int rate, int channels, const params &p)
    : _rate(rate > 0 ? rate : 48000), _channels(channels > 0 ? channels : 1), _p(p) {
  _p.min_delay_s = std::max(0.0, _p.min_delay_s);
  _p.max_delay_s = std::max(_p.max_delay_s, _p.min_delay_s);
  _p.start_delay_s = std::min(std::max(_p.start_delay_s, _p.min_delay_s), _p.max_delay_s);
}

voice_mixer::params voice_mixer::parameters() const {
  std::lock_guard<std::mutex> lk(_mu);
  return _p;
}

void voice_mixer::set_delay_floor(double min_s, double start_s) {
  std::lock_guard<std::mutex> lk(_mu);
  _p.min_delay_s = std::max(_p.min_delay_s, min_s);
  _p.max_delay_s = std::max(_p.max_delay_s, _p.min_delay_s);
  _p.start_delay_s =
      std::min(std::max({_p.start_delay_s, start_s, _p.min_delay_s}), _p.max_delay_s);
  for (auto &kv : _peers)
    kv.second.target_s = std::max(kv.second.target_s, _p.min_delay_s);
}

double voice_mixer::seconds(std::size_t n) const {
  return static_cast<double>(n) / static_cast<double>(_channels) / static_cast<double>(_rate);
}

std::size_t voice_mixer::samples(double s) const {
  if (s <= 0.0)
    return 0;
  return static_cast<std::size_t>(std::llround(s * static_cast<double>(_rate))) *
         static_cast<std::size_t>(_channels);
}

voice_mixer::peer &voice_mixer::at(const std::string &id) {
  auto it = _peers.find(id);
  if (it == _peers.end()) {
    it = _peers.emplace(id, peer()).first;
    it->second.target_s = _p.start_delay_s;
  }
  return it->second;
}

void voice_mixer::trim(peer &p, std::size_t keep) {
  if (p.buf.size() <= keep)
    return;
  const std::size_t drop = p.buf.size() - keep;
  p.buf.erase(p.buf.begin(), p.buf.begin() + static_cast<std::ptrdiff_t>(drop));
}

void voice_mixer::push(const std::string &id, const float *s, std::size_t n, double now) {
  n -= n % static_cast<std::size_t>(_channels);
  if (!s || n == 0)
    return;
  std::lock_guard<std::mutex> lk(_mu);
  peer &p = at(id);
  if (p.underrun_at >= 0.0) {
    // It ran dry while playing and its audio is back: soon after, the audio was late (the buffer
    // was too small for this link); long after, the peer had stopped sending.
    if (now - p.underrun_at <= _p.late_window_s) {
      p.target_s = std::min(_p.max_delay_s, p.target_s + _p.step_s);
      ++p.st.late_underruns;
    }
    p.underrun_at = -1.0;
  }
  p.buf.insert(p.buf.end(), s, s + n);
  const std::size_t cap = samples(_p.max_buffer_s);
  if (cap > 0 && p.buf.size() > cap) {
    p.st.overflow_frames += (p.buf.size() - cap) / static_cast<std::size_t>(_channels);
    trim(p, cap);
  }
  p.last_push = now;
}

void voice_mixer::set_gain(const std::string &id, float gain) {
  std::lock_guard<std::mutex> lk(_mu);
  at(id).gain = gain > 0.0f ? gain : 0.0f;
}

void voice_mixer::remove(const std::string &id) {
  std::lock_guard<std::mutex> lk(_mu);
  _peers.erase(id);
}

bool voice_mixer::pull(float *out, std::size_t frames, double now) {
  const std::size_t ch = static_cast<std::size_t>(_channels);
  const std::size_t n = frames * ch;
  if (!out || n == 0)
    return false;
  std::fill(out, out + n, 0.0f);
  const std::size_t fade = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::llround(_p.fade_s * static_cast<double>(_rate))));
  const double pulled_s = static_cast<double>(frames) / static_cast<double>(_rate);
  bool any = false;
  std::lock_guard<std::mutex> lk(_mu);
  for (auto &kv : _peers) {
    peer &p = kv.second;
    const std::size_t target = samples(p.target_s);
    if (!p.playing) {
      if (p.buf.empty())
        continue;
      // It starts once its target is buffered, or once its audio has stopped coming (the tail of
      // a talk spurt shorter than the target plays out rather than waiting for the next one).
      const bool idle = p.last_push >= 0.0 && now - p.last_push > _p.late_window_s;
      if (p.buf.size() < target && !idle)
        continue;
      p.playing = true;
      p.fade_in = true;
      ++p.st.starts;
    }
    // A burst after a stall, past twice the target (and at least trim_slack_s past it): back to
    // the target, cross-fading from the audio that would have played into the audio kept.
    const std::size_t slack = samples(std::max(_p.trim_slack_s, p.target_s));
    std::size_t xf = 0; // frames of `cut` to cross-fade out of
    if (p.buf.size() > target + slack) {
      p.st.trimmed_frames += (p.buf.size() - target) / ch;
      if (!p.fade_in) {
        xf = std::min(fade, (p.buf.size() - target) / ch);
        p.cut.assign(p.buf.begin(), p.buf.begin() + static_cast<std::ptrdiff_t>(xf * ch));
      }
      trim(p, target);
    }
    const std::size_t k = std::min(n, p.buf.size());
    const std::size_t kf = k / ch;
    const bool dry = k < n;
    xf = std::min(xf, kf);
    for (std::size_t f = 0; f < kf; ++f) {
      float g = p.gain;
      if (p.fade_in && f < fade)
        g *= static_cast<float>(f + 1) / static_cast<float>(fade);
      if (dry && kf - f <= fade)
        g *= static_cast<float>(kf - f) / static_cast<float>(fade + 1);
      const float w = f < xf ? static_cast<float>(f + 1) / static_cast<float>(xf + 1) : 1.0f;
      for (std::size_t c = 0; c < ch; ++c) {
        float v = p.buf[f * ch + c];
        if (f < xf)
          v = v * w + p.cut[f * ch + c] * (1.0f - w);
        out[f * ch + c] += v * g;
      }
    }
    p.buf.erase(p.buf.begin(), p.buf.begin() + static_cast<std::ptrdiff_t>(k));
    p.st.played_frames += kf;
    if (kf > 0) {
      p.fade_in = false;
      if (p.gain > 0.0f)
        any = true;
    }
    if (dry) {
      p.playing = false;
      p.underrun_at = now;
      p.clean_s = 0.0;
      ++p.st.underruns;
    } else {
      p.clean_s += pulled_s;
      if (p.clean_s >= _p.relax_after_s) {
        p.target_s = std::max(_p.min_delay_s, p.target_s - _p.step_s);
        p.clean_s = 0.0;
      }
    }
  }
  for (std::size_t i = 0; i < n; ++i)
    out[i] = out[i] > 1.0f ? 1.0f : (out[i] < -1.0f ? -1.0f : out[i]);
  return any;
}

double voice_mixer::buffered_s(const std::string &id) const {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(id);
  return it == _peers.end() ? 0.0 : seconds(it->second.buf.size());
}

bool voice_mixer::stats(const std::string &id, peer_stats &out) const {
  std::lock_guard<std::mutex> lk(_mu);
  auto it = _peers.find(id);
  if (it == _peers.end())
    return false;
  out = it->second.st;
  out.target_s = it->second.target_s;
  out.buffered_s = seconds(it->second.buf.size());
  out.playing = it->second.playing;
  return true;
}

std::vector<std::string> voice_mixer::peers() const {
  std::lock_guard<std::mutex> lk(_mu);
  std::vector<std::string> ids;
  ids.reserve(_peers.size());
  for (const auto &kv : _peers)
    ids.push_back(kv.first);
  return ids;
}

} // namespace cvc
