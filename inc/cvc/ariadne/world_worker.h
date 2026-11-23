/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#ifndef __CVC_ARIADNE_WORLD_WORKER_H__
#define __CVC_ARIADNE_WORLD_WORKER_H__

#include <cstdint>
#include <cvc/core/namespace.h>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
class world_clock;
namespace ariadne {

class SceneWorld;
class holon;

// A drawn host's scene world, stepped off its frame's thread (FLEETS_AND_NIGHT_ROADMAP.md D3) in
// a local server-role holon (ARIADNE_WORLDS.md §18.2, "Local is the same"; OW7's first part).
//
// The worker attaches `world` to a holon of its own (server role, physics on, at the world's
// rate) and runs a thread that keeps the holon's clock on the host's: tick for tick behind
// `leader`, one quantum at a time (lockstep), so the world takes the same steps, in the same
// order, as the host's own follow() would. The host keeps driving `leader` (its transport, the
// sim.* keys) and everything else that reads it.
//
// The world is deferred (SceneWorld::set_deferred) while the worker has it: a step writes no key,
// and the host calls SceneWorld::publish_due() on its own thread. Everything the host does with
// the world (a pump that reaches it, sync and publish, reading its vehicles and bodies to draw
// them, a restart) runs under lock(). The worker takes the same lock for each step and lets a
// host that is waiting for it go first, so the host waits at most one step.
//
// Headless hosts, the gym and the relay don't use it: they step synchronously, bit for bit.
// Caller-owned; destroy it before the world (and before what `capture` reads).
class world_worker {
public:
  struct options {
    // After each step, on the worker, under the lock: a view's pose_tracker::capture.
    std::function<void()> capture;
    // How far (quanta) the world may fall behind `leader` before behind() tells the host to hold
    // its clock: a world too heavy for its speed runs slower, as it would on the frame's thread,
    // instead of drifting further behind. 0.25 s at 120 Hz.
    std::uint64_t max_lag = 30;
  };

  // Throws std::runtime_error (saying why) when the holon cannot take the world: a build without
  // threads (single-thread wasm) or without Jolt, or a world whose quantum is no holon rate.
  world_worker(SceneWorld &world, const cvc::world_clock &leader, options opt);
  world_worker(SceneWorld &world, const cvc::world_clock &leader);
  // Stops and joins the thread, then hands the world back synchronous (and published).
  ~world_worker();
  world_worker(const world_worker &) = delete;
  world_worker &operator=(const world_worker &) = delete;

  // The host's hold on the world, released when it goes. It may outlive the worker (a frame that
  // drops its world, or a failed one, while it holds it): it then only lets go.
  class frame_lock {
  public:
    frame_lock() = default;
    explicit frame_lock(world_worker &w);
    ~frame_lock();
    frame_lock(frame_lock &&o) noexcept;
    frame_lock &operator=(frame_lock &&o) noexcept;
    frame_lock(const frame_lock &) = delete;
    frame_lock &operator=(const frame_lock &) = delete;

  private:
    struct impl;
    std::shared_ptr<impl> _d;
    friend class world_worker;
  };
  frame_lock lock() { return frame_lock(*this); }

  // Quanta the world has still to run to reach `leader`'s tick.
  std::uint64_t lag() const;
  // lag() at or past max_lag: the host holds its clock (advances it by no time) this frame.
  bool behind() const;
  // Steps the worker has run.
  std::uint64_t steps() const;
  // Why the worker stopped: a step (or the capture) threw. Empty while it runs. The world steps no
  // more; the host stops its physics, as when its own follow() throws.
  std::string error() const;
  // Block until the world has run every quantum `leader` had when called, or `timeout_seconds`
  // pass; true when it got there. Not under lock().
  bool wait_caught_up(double timeout_seconds) const;

  // The holon the world runs in (its clock is the world's).
  holon &local_holon();
  const SceneWorld &world() const;

private:
  struct impl;
  std::shared_ptr<impl> _impl;
};

} // namespace ariadne
} // namespace cvc

#endif
