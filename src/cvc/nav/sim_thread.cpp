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

#include <chrono>
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>

namespace cvc {
namespace nav {

using clock = std::chrono::steady_clock;

sim_thread::sim_thread(sim_world &world, double hz) : world_(world) {
  period_.store(1.0 / (hz > 0.0 ? hz : 60.0));
}

sim_thread::~sim_thread() { stop(); }

void sim_thread::start() {
  if (run_.exchange(true))
    return; // already running
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

void sim_thread::set_paused(bool paused) { paused_.store(paused); }

void sim_thread::set_rate(double hz) { period_.store(1.0 / (hz > 0.0 ? hz : 60.0)); }

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
  // Publish an initial frame so a reader has something before the first step.
  std::atomic_store(&latest_, std::shared_ptr<const snapshot>(make_snapshot()));
  auto next = clock::now();
  while (run_.load()) {
    // Top-of-tick: drain live-scene edits (applied on this thread, so the
    // sim_world is only ever mutated here).
    {
      std::lock_guard<std::mutex> lk(cmd_mtx_);
      while (!cmds_.empty()) {
        cmds_.front()();
        cmds_.pop_front();
      }
    }
    if (paused_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      next = clock::now();
      continue;
    }

    world_.step(0);
    std::atomic_store(&latest_, std::shared_ptr<const snapshot>(make_snapshot()));
    ticks_.fetch_add(1, std::memory_order_relaxed);

    const auto period = std::chrono::duration<double>(period_.load());
    next += std::chrono::duration_cast<clock::duration>(period);
    const auto now = clock::now();
    if (next > now) {
      std::this_thread::sleep_for(next - now); // releases the core; renderer runs
    } else {
      behind_.fetch_add(1, std::memory_order_relaxed);
      next = now; // fell behind -> no spiral of death
    }
  }
}

} // namespace nav
} // namespace cvc
