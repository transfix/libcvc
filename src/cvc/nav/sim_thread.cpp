/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// sim_thread.cpp — see sim_thread.h.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>

namespace cvc {
namespace nav {

namespace {

using steady = std::chrono::steady_clock;

double nav_dt(const sim_world &w) { return static_cast<double>(w.current_config().veh.dt); }

// The hz form's private clock: one quantum per navigator step, run at hz steps
// per wall second. A cap of 8 per advance keeps the old "fell behind -> no
// spiral of death" behaviour, now with the dropped quanta counted.
cvc::world_clock::config private_clock(const sim_world &w, double hz) {
  cvc::world_clock::config c;
  const double dt = nav_dt(w);
  c.fixed_dt = (dt > 0.0 && std::isfinite(dt)) ? dt : 0.06;
  c.scale = (hz > 0.0 ? hz : 60.0) * c.fixed_dt;
  c.max_steps_per_advance = 8;
  return c;
}

} // namespace

sim_thread::sim_thread(sim_world &world, cvc::world_clock &clock, options opt)
    : world_(world), clock_(&clock), drive_(opt.drive_clock),
      follower_(cvc::ticks_per_step(nav_dt(world), clock.fixed_dt()), opt.phase, clock.tick()) {}

sim_thread::sim_thread(sim_world &world, double hz)
    : world_(world), own_clock_(new cvc::world_clock(private_clock(world, hz))),
      clock_(own_clock_.get()), drive_(true), follower_(1, 0, 0) {}

sim_thread::~sim_thread() { stop(); }

void sim_thread::start() {
  if (run_.exchange(true))
    return; // already running
  // Anchor on the caller's thread, before the worker exists: steps that fall
  // due from here on are run (none from before start() is replayed as a
  // burst), and a driver advancing the clock right after start() cannot race
  // the worker's first look at it.
  follower_.reset(clock_->tick());
  thr_ = std::thread(&sim_thread::run, this);
}

void sim_thread::stop() {
  if (!run_.exchange(false))
    return;
  if (thr_.joinable())
    thr_.join();
}

void sim_thread::retarget(int i, float gx_n, float gy_n) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmds_.emplace_back([this, i, gx_n, gy_n]() { world_.retarget(i, gx_n, gy_n); });
}

void sim_thread::set_paused(bool paused) {
  paused_.store(paused);
  if (own_clock_)
    own_clock_->set_mode(paused ? cvc::world_clock::mode::paused : cvc::world_clock::mode::live);
}

void sim_thread::set_rate(double hz) {
  if (own_clock_)
    own_clock_->set_scale((hz > 0.0 ? hz : 60.0) * own_clock_->fixed_dt());
}

std::shared_ptr<const sim_thread::snapshot> sim_thread::read() const {
  return std::atomic_load(&latest_); // one atomic shared_ptr load; never blocks the sim
}

std::shared_ptr<const sim_thread::snapshot> sim_thread::make_snapshot() const {
  auto s = std::make_shared<snapshot>();
  const int n = world_.size();
  s->tick = world_.tick();
  s->n = n;
  s->pos.resize(static_cast<std::size_t>(n) * 2);
  s->heading.resize(n);
  s->speed.resize(n);
  s->mode.resize(n);
  s->reached.resize(n);
  world_.snapshot(s->pos.data(), s->heading.data(), s->speed.data(), s->mode.data(),
                  s->reached.data());
  return s;
}

void sim_thread::run() {
  // Live-scene edits are applied on this thread, so the sim_world is only ever
  // mutated here: at the top of every loop and before every step.
  auto drain = [this] {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    while (!cmds_.empty()) {
      cmds_.front()();
      cmds_.pop_front();
    }
  };

  // Publish an initial frame so a reader has something before the first step.
  std::atomic_store(&latest_, std::shared_ptr<const snapshot>(make_snapshot()));
  const std::uint64_t k = follower_.period();
  auto last = steady::now();

  while (run_.load()) {
    drain();

    if (drive_) {
      // The one driver of this clock: advance it by the wall time since the
      // last pass. What runs is decided by the clock, never by this loop's
      // timing, so a slow pass changes how many steps run now, not what they
      // compute.
      const auto now = steady::now();
      const double wall = std::chrono::duration<double>(now - last).count();
      last = now;
      const cvc::world_clock::step_result r = clock_->advance(wall);
      if (r.dropped_steps > 0)
        behind_.fetch_add(static_cast<long>(r.dropped_steps), std::memory_order_relaxed);
    }

    const std::uint64_t now_tick = clock_->tick();
    std::uint64_t due = follower_.take(now_tick);
    if (paused_.load())
      due = 0; // this simulation alone is paused: its due steps are skipped, not banked
    for (std::uint64_t i = 0; i < due && run_.load(); ++i) {
      drain();
      world_.step(0);
      std::atomic_store(&latest_, std::shared_ptr<const snapshot>(make_snapshot()));
      ticks_.fetch_add(1, std::memory_order_relaxed);
    }
    if (due > 0) {
      const std::uint64_t since = (now_tick + k - follower_.phase()) % k;
      last_step_tick_.store(now_tick - since, std::memory_order_relaxed);
    }

    if (drive_) {
      // Sleep until about when the next step falls due in wall time, so the
      // core is released for the renderer; bounded both ways so a pause or a
      // speed change is noticed promptly.
      double wait = 0.002;
      const double scale = clock_->scale();
      if (!paused_.load() && clock_->current_mode() == cvc::world_clock::mode::live &&
          scale > 0.0) {
        const std::uint64_t t = clock_->tick();
        const std::uint64_t since = (t + k - follower_.phase()) % k;
        const double world_left =
            static_cast<double>(k - since) * clock_->fixed_dt() - clock_->pending_seconds();
        wait = world_left / scale;
      }
      wait = std::min(std::max(wait, 0.0002), 0.02);
      std::this_thread::sleep_for(std::chrono::duration<double>(wait));
    } else {
      // Follow: wake when the driver moves the clock (or after 20 ms, to
      // notice stop() and queued commands).
      clock_->wait_for_tick_change(now_tick, 0.02);
    }
  }
}

} // namespace nav
} // namespace cvc
