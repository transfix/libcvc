/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
#ifndef CVC_ARIADNE_SCENE_ANIMATION_H
#define CVC_ARIADNE_SCENE_ANIMATION_H
// Keyframe animation (docs/ARIADNE.md §5.4, "Animations"): a scene's `animations:`, each a set of
// tracks that move a node's key (its position, rotation, scale, colour, opacity or visibility) or
// write a state key, along keyframes over the scene clock's time. This header is the data and
// the arithmetic, free of any renderer: the loader fills a SceneAnimation, sample_track says what a
// track holds at a time, advance_animation moves a playhead. cvcGL's realize_scene / tick_scene
// play them (scene_realize.h).
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// One track: a node's key, or a state key, along its keyframes.
struct SceneAnimationTrack {
  // node: <id> names every realized node with that id (each instance's part, each repeat: copy's
  // child); node: <id>/<child>/... names the child under that node alone. Empty for a state track.
  std::string node;
  // A node track's key: position, rotation (degrees about x, then y, then z), scale (each three
  // numbers), color (three, 0..1), opacity (one, 0..1) or visible (one: shown above 0.5).
  std::string key;
  // state: <path>, resolved as a bind is: the track writes its value there (a number, or numbers
  // joined by commas). Empty for a node track.
  std::string state;
  // Between keyframes: a straight line (linear), the earlier key's value until the next (step), or
  // a curve through every key that eases in and out at the first and last (smooth: a cubic Hermite
  // spline with Catmull-Rom tangents, flat at the ends).
  enum class interp { linear, step, smooth };
  interp interpolation = interp::linear;
  std::vector<double> times;               // ascending, seconds into the animation
  std::vector<std::vector<double>> values; // one per time, all the same length
};

struct SceneAnimation {
  std::string id; // its keys are <prefix>.animations.<id>.time / .playing / .speed
  // At the end: stop (once), start over (loop), or turn back (pingpong).
  enum class loop_mode { once, loop, pingpong };
  loop_mode loop = loop_mode::once;
  bool autoplay = true;  // playing from the start
  double speed = 1.0;    // seconds of animation a second of the scene clock; below 0, backwards
  double duration = 0.0; // its length when above 0; else its last keyframe's time
  std::vector<SceneAnimationTrack> tracks;
  // How long it is: duration, or the latest keyframe of any track.
  double length() const;
};

// The track's value at `t` seconds into its animation, into `out` (resized to the track's width).
// Before its first keyframe it holds the first's value, after its last the last's. Empty for a
// track with no keyframes.
void sample_track(const SceneAnimationTrack &track, double t, std::vector<double> &out);

// Where an animation is: `time` in [0, length()], its direction (pingpong), and whether it plays.
struct SceneAnimationPlayhead {
  double time = 0.0;
  bool forward = true;
  bool playing = true;
};

// Move the playhead `dt` seconds of the scene clock at `speed`: a once animation stops at the end
// it runs into (playing false); a loop wraps; a pingpong turns back at each end. An animation of no
// length holds at 0. Does nothing while not playing.
void advance_animation(const SceneAnimation &animation, double speed, double dt,
                       SceneAnimationPlayhead &head);

// How many numbers a node track's key takes: 3 for position, rotation, scale and color, 1 for
// opacity and visible, 0 for a key a node track cannot move.
int node_key_width(const std::string &key);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SCENE_ANIMATION_H
