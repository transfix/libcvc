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

#ifndef __CVC_NAV_SIM_THREAD_H__
#define __CVC_NAV_SIM_THREAD_H__

#include <atomic>
#include <cstdint>
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

  // Borrows `world` (the caller keeps it alive until the thread is stopped).
  sim_thread(sim_world &world, double hz);
  ~sim_thread();

  sim_thread(const sim_thread &) = delete;
  sim_thread &operator=(const sim_thread &) = delete;

  void start();
  void stop(); // idempotent; joins the worker

  // Producer API (any thread): applied at the top of the next tick.
  void retarget(int i, float gx_n, float gy_n);
  void set_paused(bool paused);
  void set_rate(double hz);

  // Lock-free latest frame (an atomic shared_ptr load); null before the first
  // tick completes.
  std::shared_ptr<const snapshot> read() const;
  long ticks() const { return ticks_.load(std::memory_order_relaxed); }
  long behind() const { return behind_.load(std::memory_order_relaxed); }

private:
  sim_world &world_;
  std::atomic<double> period_;
  std::atomic<bool> run_{false};
  std::atomic<bool> paused_{false};
  std::atomic<long> ticks_{0};
  std::atomic<long> behind_{0};
  std::shared_ptr<const snapshot> latest_; // published via std::atomic_store
  std::mutex cmd_mtx_;
  std::deque<std::function<void()>> cmds_;
  std::thread thr_;

  void run();
  std::shared_ptr<const snapshot> make_snapshot() const;
};

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_SIM_THREAD_H__
