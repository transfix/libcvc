/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics::pose_tracker — the last two states of a set of bodies, for a
// view to draw between.
//
// A world steps in fixed quanta and a display runs at its own rate. Drawing each
// body at its newest state judders whenever the two rates beat. The fix
// (world_clock.h) is to draw between the last two states at the fraction of a
// step the clock reports: lerp(previous, newest, alpha). The tracker keeps those
// two states and does the lerp.
//
//   cvc::physics::pose_tracker poses(w);   // per world and view; no globals
//   poses.track(box);
//   // every frame, on the thread that drives the clock:
//   const auto s = clock.advance(wall_dt);
//   poses.follow(clock);                   // steps the world, keeping its last two states
//   for (const auto &p : poses.sample(poses.alpha(clock, s.alpha)))
//     draw(p.id, p.pose);
//
// The view lags the world by one step: alpha 0 is the previous state, and the
// newest is reached just as the next step lands. That is the price of never
// extrapolating, and it is the same lag world_clock.h's render loop has.
//
// Stepping: follow() steps the world itself, in two parts when several steps are
// due, so the two samples are the last two ticks. Use it instead of
// world::follow(), not with it: each keeps its own cursor on the clock, so the
// world would take every due step twice. A world stepped some other way (by
// step() on a simulation thread) calls capture() after each step instead.
//
// Threads: thread-safe. One thread may step and capture while others sample; a
// sample sees whole captures, never half of one. The world must outlive the
// tracker.
//
// Bodies: a body removed from the world drops out at the next capture. A body
// just tracked, or snap()ped after a teleport, is drawn at its newest state until
// a second capture one step later gives it a previous one.

#ifndef __CVC_PHYSICS_POSE_TRACKER_H__
#define __CVC_PHYSICS_POSE_TRACKER_H__

#include <cstddef>
#include <cstdint>
#include <cvc/physics/types.h>
#include <memory>
#include <optional>
#include <vector>

namespace cvc {

class world_clock;

namespace physics {

class world;

// Where a body is: its origin in world metres and its rotation.
struct pose {
  vec3 position;
  quat rotation;
};

// The pose a fraction alpha of the way from a to b: the position lerped, the
// rotation along the shorter arc (b's sign is flipped when the quaternions'
// dot product is negative, then the lerp is normalized). alpha 0 gives a, 1
// gives b, exactly. Normalized lerp rather than slerp: the angular rate is not
// quite constant, but over one step's rotation the difference is far below a
// pixel, and the ends are exact.
pose interpolate(const pose &a, const pose &b, double alpha);

// The row-major 4x4 matrix of a pose (rotation, then translation), the layout
// cvc::gl::GraphicsNode::setPoseMatrix takes.
void pose_matrix(const pose &p, double out[16]);

class pose_tracker {
public:
  explicit pose_tracker(world &w);
  ~pose_tracker();
  pose_tracker(const pose_tracker &) = delete;
  pose_tracker &operator=(const pose_tracker &) = delete;

  // Start or stop tracking a body. track() throws cvc::physics_error for an id
  // the world does not hold; tracking a body twice is a no-op.
  void track(body_id id);
  void untrack(body_id id);
  bool tracking(body_id id) const;
  std::size_t size() const;

  // Read every tracked body under one world lock (world::states) and make that
  // the newest sample; the newest becomes the previous one. A second capture at
  // the same tick changes nothing.
  void capture();

  // Run the steps due on `clock` since the last call and capture so that the two
  // samples are the world's last two ticks. Returns the number of steps run. The
  // first call anchors at the clock's tick, captures, and runs none. Throws
  // cvc::unsupported_exception unless the world's fixed_dt is a whole number of
  // the clock's quanta (cvc::app's clock defaults to 1/120 s and a world to
  // 0.01 s: configure one of them).
  std::uint64_t follow(const cvc::world_clock &clock);

  // The fraction of the way from the previous sample to the newest at which to
  // draw, given the alpha clock.advance() returned (a fraction of one clock
  // quantum). It accounts for a world that steps once every several quanta. In
  // [0, 1). Throws like follow() for a mismatched clock.
  double alpha(const cvc::world_clock &clock, double clock_alpha) const;

  // Draw a body at its newest state, with no previous one, until the capture
  // after next: call it with world::set_state (a teleport, before or after the
  // step that follows), so the view does not sweep through the space in between.
  void snap(body_id id);

  struct sample {
    body_id id;
    physics::pose pose;
  };
  // Every tracked body that has been captured, at alpha (clamped to [0, 1])
  // between its previous and newest samples, in the order the bodies were
  // tracked. A body whose previous sample is not exactly one step before its
  // newest is drawn at its newest.
  std::vector<sample> sample_all(double alpha) const;
  // The same, into `out` (cleared first), reusing its storage across frames.
  void sample_all(double alpha, std::vector<sample> &out) const;
  // One body; empty if it is not tracked or not captured yet.
  std::optional<physics::pose> sample_one(body_id id, double alpha) const;

  // The world tick of the newest capture (0 before the first).
  std::uint64_t tick() const;

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_POSE_TRACKER_H__
