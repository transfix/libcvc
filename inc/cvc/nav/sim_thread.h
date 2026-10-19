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

// sim_thread.h — run a sim_world off the render thread (port P7).
//
// The C++ counterpart of grl_snam/sim_thread.py: a dedicated worker advances a
// sim_world at a fixed rate and publishes an immutable Snapshot each tick by an
// atomic shared_ptr swap, which a renderer reads lock-free (it sees the whole
// previous frame or the whole next one — never a torn mix, because the published
// frame is never mutated). Live-scene edits flow the other way through a command
// queue drained at the top of each tick. The step spends its time in the
// (thread-safe, self-contained) sim_world kernels, so a C++ renderer draws on
// another core while the sim runs — no Python, no GIL. See
// docs/CVCNAV_CPP_PORT_ROADMAP.md P7.
//
// Time comes from a cvc::world_clock. Given the app's clock, the worker FOLLOWS
// it: the frame loop drives the clock (cvc::world_clock_binding::drive) and the
// navigator steps every veh.dt / fixed_dt quanta, so speed, pause, single-step
// and lockstep are the clock's controls, shared with every other simulation on
// it. Given only a rate (the original form), the worker drives a private clock
// at that rate. Either way the steps a run takes are fixed by the clock's
// ticks, not by how the worker's passes happen to be timed.

#ifndef __CVC_NAV_SIM_THREAD_H__
#define __CVC_NAV_SIM_THREAD_H__

#include <atomic>
#include <cstdint>
#include <cvc/core/clock_follower.h>
#include <cvc/core/world_clock.h>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace cvc {
namespace nav {

class sim_world;

class sim_thread {
public:
  // One published frame. Immutable after publish; poses are in WORLD metres.
  struct snapshot {
    long tick = 0;
    int n = 0;
    std::vector<float> pos;            // [n*2] world
    std::vector<float> heading;        // [n]
    std::vector<float> speed;          // [n] world m/s
    std::vector<int> mode;             // [n] 0 seek / 1 wall
    std::vector<std::uint8_t> reached; // [n]
  };

  struct options {
    // Drive the clock from wall time on this thread. Only for a clock nothing
    // else advances (a headless run with no frame loop); otherwise exactly one
    // driver -- usually the frame loop, through cvc::world_clock_binding -- owns
    // advance(), and this thread only follows.
    bool drive_clock = false;
    // The navigator steps on ticks t with t % k == phase, k = veh.dt / fixed_dt.
    std::uint64_t phase = 0;
  };

  // Follow a shared clock (cvc/core/world_clock.h): one navigator step every
  // k = veh.dt / clock.fixed_dt() quanta, run as the clock's driver advances
  // it, so this simulation shares that clock's speed, pause, single-step and
  // lockstep with everything else on it. Throws cvc::unsupported_exception if
  // veh.dt is not a whole number of quanta. Borrows `world` and `clock`.
  sim_thread(sim_world &world, cvc::world_clock &clock, options opt);
  sim_thread(sim_world &world, cvc::world_clock &clock) : sim_thread(world, clock, options{}) {}

  // The original rate-paced form: `hz` navigator steps per wall second. It now
  // runs on a private clock whose quantum is veh.dt, driven by this thread at a
  // speed of hz * veh.dt (60 Hz at 0.06 s is 3.6x real time), so pause and rate
  // go through the clock and a stall is reported in behind(). Borrows `world`.
  sim_thread(sim_world &world, double hz);
  ~sim_thread();

  sim_thread(const sim_thread &) = delete;
  sim_thread &operator=(const sim_thread &) = delete;

  void start();
  void stop(); // idempotent; joins the worker

  // Producer API (any thread): applied at the top of the next tick.
  void retarget(int i, float gx_n, float gy_n);

  // Pause THIS simulation. On the private clock (the hz form) that pauses the
  // clock. On a shared clock the clock is left alone -- pausing everything is
  // the clock's own control (world_clock_binding's sim.paused) -- and this
  // navigator's due steps are skipped until it resumes.
  void set_paused(bool paused);

  // Steps per wall second, for the private clock (the hz form): its speed
  // becomes hz * veh.dt. A no-op on a shared clock, whose speed is the clock's.
  void set_rate(double hz);

  // The clock this thread follows (its private one in the hz form).
  cvc::world_clock &clock() { return *clock_; }

  // Lock-free latest frame (an atomic shared_ptr load); null before the first
  // tick completes.
  std::shared_ptr<const snapshot> read() const;

  // Navigator steps run.
  long ticks() const { return ticks_.load(std::memory_order_relaxed); }
  // Quanta dropped by the clock this thread drives because it fell behind; 0
  // when it only follows (the driver owns that count).
  long behind() const { return behind_.load(std::memory_order_relaxed); }
  // The clock tick of the last navigator step, for presentation:
  // clock_follower::alpha(clock().tick(), clock_alpha) interpolates from it.
  std::uint64_t last_step_tick() const { return last_step_tick_.load(std::memory_order_relaxed); }

  // Block until the worker has run every step due up to the clock's current
  // tick and published the result, or until timeout_s passes. A deterministic
  // capture calls this after driving the clock and before read(), so each frame
  // shows exactly the steps its quanta hold. Returns false on timeout or if the
  // thread is not running.
  bool wait_caught_up(double timeout_s) const;

private:
  sim_world &world_;
  std::unique_ptr<cvc::world_clock> own_clock_; // the hz form's private clock
  cvc::world_clock *clock_ = nullptr;
  bool drive_ = false;
  cvc::clock_follower follower_;
  std::atomic<bool> run_{false};
  std::atomic<bool> paused_{false};
  std::atomic<long> ticks_{0};
  std::atomic<long> behind_{0};
  std::atomic<std::uint64_t> last_step_tick_{0};
  std::atomic<std::uint64_t> seen_tick_{0}; // the clock tick the worker has caught up to
  std::shared_ptr<const snapshot> latest_;  // published via std::atomic_store
  std::mutex cmd_mtx_;
  std::deque<std::function<void()>> cmds_;
  std::thread thr_;

  void run();
  std::shared_ptr<const snapshot> make_snapshot() const;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_THREAD_H__
