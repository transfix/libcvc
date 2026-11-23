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

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_worker.h>
#include <cvc/core/world_clock.h>
#include <cvc/physics/world.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace cvc {
namespace ariadne {

struct world_worker::impl {
  SceneWorld &world;
  const cvc::world_clock &leader;
  options opt;
  std::unique_ptr<holon> h;

  // The world: one step on the worker, or the host's frame_lock.
  std::mutex mu;
  // A host waiting for `mu` goes first: the worker takes it only while none waits.
  std::mutex gate_mu;
  std::condition_variable gate;
  std::atomic<int> want{0};

  std::atomic<bool> run{true};
  std::atomic<std::uint64_t> steps{0};
  // The holon's tick after the worker's last step or seek: what the world has run up to.
  std::atomic<std::uint64_t> at{0};
  mutable std::mutex at_mu;
  mutable std::condition_variable at_cv;
  std::string failed; // under at_mu: why the loop stopped
  std::thread thread;

  impl(SceneWorld &w, const cvc::world_clock &l, options o)
      : world(w), leader(l), opt(std::move(o)) {}

  void note_at(std::uint64_t tick) {
    {
      std::lock_guard<std::mutex> g(at_mu);
      at.store(tick);
    }
    at_cv.notify_all();
  }

  // The world under `mu`, a waiting host first; false once stopped.
  bool take() {
    for (;;) {
      {
        std::unique_lock<std::mutex> g(gate_mu);
        gate.wait(g, [&] { return want.load() == 0 || !run.load(); });
      }
      if (!run.load())
        return false;
      if (mu.try_lock())
        return true;
      std::this_thread::yield(); // a host took it between the gate and here: wait for it again
    }
  }

  void loop() {
    while (run.load()) {
      const std::uint64_t target = leader.tick();
      if (target == at.load()) {
        // Caught up: wake when the host's clock moves (or after 20 ms, to notice a stop).
        leader.wait_for_tick_change(target, 0.02);
        continue;
      }
      if (!take())
        return;
      std::lock_guard<std::mutex> held(mu, std::adopt_lock);
      // Again under the lock: the host may have reset or moved its clock meanwhile (sim.restart
      // goes back to tick 0). Back, the holon's clock follows it, and the world's follower anchors
      // again at its next step; ahead by one or more, one quantum runs.
      const std::uint64_t t = leader.tick(), own = h->tick();
      if (t < own) {
        h->clock().seek_tick(t);
        note_at(t);
        continue;
      }
      if (t == own) {
        note_at(own);
        continue;
      }
      try {
        h->advance(0.0); // lockstep: exactly one quantum, the world's step in the holon's
      } catch (const std::exception &e) {
        stop_with(e.what());
        return;
      } catch (...) {
        stop_with("a step threw");
        return;
      }
      steps.fetch_add(1);
      note_at(h->tick());
    }
  }

  void stop_with(const std::string &why) {
    {
      std::lock_guard<std::mutex> g(at_mu);
      failed = why;
    }
    run.store(false);
    at_cv.notify_all();
  }
};

world_worker::world_worker(SceneWorld &world, const cvc::world_clock &leader)
    : world_worker(world, leader, options{}) {}

world_worker::world_worker(SceneWorld &world, const cvc::world_clock &leader, options opt)
    : _impl(std::make_shared<impl>(world, leader, std::move(opt))) {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
  throw std::runtime_error("world_worker: this build has no threads");
#else
  impl &d = *_impl;
  const double dt = world.world().config().fixed_dt;
  if (std::fabs(dt - leader.fixed_dt()) > 1e-12)
    throw std::runtime_error("world_worker: the world steps " + std::to_string(dt) +
                             " s, its host's clock " + std::to_string(leader.fixed_dt()) +
                             " s; it runs on the host's thread");
  holon_config cfg;
  cfg.physics = true; // the world's bodies, which attach_world takes the avatars into
  try {
    cfg = holon_config_at(cfg, std::round(1.0 / dt));
  } catch (const std::invalid_argument &e) {
    throw std::runtime_error(std::string("world_worker: ") + e.what());
  }
  // Lockstep: each advance() runs exactly one quantum, whatever the wall time.
  cfg.clock.lockstep_steps = 1;
  d.h = std::make_unique<holon>(holon_role::server, cfg);
  d.h->clock().set_mode(cvc::world_clock::mode::lockstep);
  d.h->clock().seek_tick(leader.tick()); // the host's tick: the world's follower goes on from it
  std::string why;
  if (!d.h->attach_world(&world, &why))
    throw std::runtime_error("world_worker: " + why);
  d.at.store(d.h->tick());
  // Anchored here, at the host's tick: a world its host never followed would otherwise spend the
  // holon's first quantum anchoring (SceneWorld::follow runs none on its first call).
  world.follow(d.h->clock());
  // A restart (on the host's thread, under its lock) takes the holon's clock to the host's tick and
  // anchors the world there, as the first follow() after a restart on the host's thread would.
  world.set_deferred(true, d.opt.capture, [&d] {
    const std::uint64_t t = d.leader.tick();
    d.h->clock().seek_tick(t);
    d.world.follow(d.h->clock());
    d.note_at(t);
  });
  try {
    d.thread = std::thread([&d] { d.loop(); });
  } catch (...) {
    world.set_deferred(false);
    d.h->attach_world(nullptr);
    throw;
  }
#endif
}

world_worker::~world_worker() {
  impl &d = *_impl;
  {
    std::lock_guard<std::mutex> g(d.gate_mu);
    d.run.store(false);
  }
  d.gate.notify_all();
  if (d.thread.joinable())
    d.thread.join();
  // The world back on its host: synchronous, what ran published. The holon goes now, though a
  // frame_lock may keep the rest until it lets go.
  d.h->attach_world(nullptr);
  d.world.set_deferred(false);
  d.h.reset();
}

// What a frame_lock holds: the worker's state, kept alive while it holds it.
struct world_worker::frame_lock::impl {
  std::shared_ptr<world_worker::impl> w;
  explicit impl(std::shared_ptr<world_worker::impl> p) : w(std::move(p)) {
    w->want.fetch_add(1);
    w->mu.lock();
  }
  ~impl() {
    w->mu.unlock();
    {
      std::lock_guard<std::mutex> g(w->gate_mu);
      w->want.fetch_sub(1);
    }
    w->gate.notify_all();
  }
};

world_worker::frame_lock::frame_lock(world_worker &w) : _d(std::make_shared<impl>(w._impl)) {}

world_worker::frame_lock::~frame_lock() = default;

world_worker::frame_lock::frame_lock(frame_lock &&o) noexcept = default;

world_worker::frame_lock &world_worker::frame_lock::operator=(frame_lock &&o) noexcept = default;

std::uint64_t world_worker::lag() const {
  const std::uint64_t t = _impl->leader.tick(), at = _impl->at.load();
  return t > at ? t - at : 0;
}

bool world_worker::behind() const { return lag() >= _impl->opt.max_lag; }

std::uint64_t world_worker::steps() const { return _impl->steps.load(); }

std::string world_worker::error() const {
  std::lock_guard<std::mutex> g(_impl->at_mu);
  return _impl->failed;
}

bool world_worker::wait_caught_up(double timeout_seconds) const {
  const impl &d = *_impl;
  const std::uint64_t target = d.leader.tick();
  std::unique_lock<std::mutex> g(d.at_mu);
  return d.at_cv.wait_for(g, std::chrono::duration<double>(timeout_seconds),
                          [&] { return d.at.load() >= target; });
}

holon &world_worker::local_holon() { return *_impl->h; }

const SceneWorld &world_worker::world() const { return _impl->world; }

} // namespace ariadne
} // namespace cvc
