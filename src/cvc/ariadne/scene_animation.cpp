/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
#include <algorithm>
#include <cmath>
#include <cvc/ariadne/scene_animation.h>

namespace cvc {
namespace ariadne {

double SceneAnimation::length() const {
  if (duration > 0.0)
    return duration;
  double end = 0.0;
  for (const SceneAnimationTrack &t : tracks)
    if (!t.times.empty())
      end = std::max(end, t.times.back());
  return end;
}

int node_key_width(const std::string &key) {
  if (key == "position" || key == "rotation" || key == "scale" || key == "color")
    return 3;
  if (key == "opacity" || key == "visible")
    return 1;
  return 0;
}

void sample_track(const SceneAnimationTrack &track, double t, std::vector<double> &out) {
  const std::size_t n = std::min(track.times.size(), track.values.size());
  if (n == 0) {
    out.clear();
    return;
  }
  if (n == 1 || !(t > track.times.front())) { // NaN holds the first
    out = track.values.front();
    return;
  }
  if (t >= track.times[n - 1]) {
    out = track.values[n - 1];
    return;
  }
  // The keyframes either side: times[i] <= t < times[i + 1].
  const std::size_t i =
      static_cast<std::size_t>(std::upper_bound(track.times.begin(), track.times.begin() + n, t) -
                               track.times.begin()) -
      1;
  const std::vector<double> &a = track.values[i], &b = track.values[i + 1];
  out.resize(a.size());
  if (track.interpolation == SceneAnimationTrack::interp::step) {
    out = a;
    return;
  }
  const double h = track.times[i + 1] - track.times[i];
  const double u = h > 0.0 ? (t - track.times[i]) / h : 1.0;
  if (track.interpolation == SceneAnimationTrack::interp::linear) {
    for (std::size_t k = 0; k < a.size(); ++k)
      out[k] = a[k] + (b[k] - a[k]) * u;
    return;
  }
  // smooth: the cubic Hermite through a and b, its tangents Catmull-Rom's (the slope from the key
  // before to the key after), flat at the first and last keys.
  const auto tangent = [&](std::size_t j, std::size_t k) {
    if (j == 0 || j + 1 >= n)
      return 0.0;
    const double span = track.times[j + 1] - track.times[j - 1];
    return span > 0.0 ? (track.values[j + 1][k] - track.values[j - 1][k]) / span : 0.0;
  };
  const double u2 = u * u, u3 = u2 * u;
  const double h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2,
               h11 = u3 - u2;
  for (std::size_t k = 0; k < a.size(); ++k)
    out[k] = h00 * a[k] + h10 * h * tangent(i, k) + h01 * b[k] + h11 * h * tangent(i + 1, k);
}

void advance_animation(const SceneAnimation &animation, double speed, double dt,
                       SceneAnimationPlayhead &head) {
  if (!head.playing)
    return;
  const double len = animation.length();
  if (!(len > 0.0)) {
    head.time = 0.0;
    return;
  }
  const double d = std::isfinite(speed) && std::isfinite(dt) ? speed * dt : 0.0;
  switch (animation.loop) {
  case SceneAnimation::loop_mode::once: {
    const double t = head.time + d;
    if (t >= len) {
      head.time = len;
      head.playing = false;
    } else if (t <= 0.0 && d < 0.0) {
      head.time = 0.0;
      head.playing = false;
    } else {
      head.time = std::max(0.0, t);
    }
    break;
  }
  case SceneAnimation::loop_mode::loop: {
    double t = std::fmod(head.time + d, len);
    if (t < 0.0)
      t += len;
    head.time = t;
    break;
  }
  case SceneAnimation::loop_mode::pingpong: {
    // One period there and back is 2 len; place the playhead on it, move, fold back.
    double u = (head.forward ? head.time : 2.0 * len - head.time) + d;
    u = std::fmod(u, 2.0 * len);
    if (u < 0.0)
      u += 2.0 * len;
    head.forward = u <= len;
    head.time = head.forward ? u : 2.0 * len - u;
    break;
  }
  }
}

} // namespace ariadne
} // namespace cvc
