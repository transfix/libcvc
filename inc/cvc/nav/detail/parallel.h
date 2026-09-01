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

// detail/parallel.h — the shared work-splitter for cvc::nav's batched kernels.
//
// A dynamically-scheduled parallel-for: a shared atomic counter hands out
// indices so uneven per-item cost (some A* queries expand far more nodes than
// others; some agents ray-cast farther before an occluder) self-balances. The
// calling thread is one of the workers, so num_threads==1 runs inline with no
// thread spawn. Extracted from grid_nav.cpp so the belief-sensing / SDF /
// planning kernels AND the torch-free drive (drive.cpp, and later drive.cu's
// host launcher) share one implementation — see docs/CVCNAV_CPP_PORT_ROADMAP.md.

#ifndef __CVC_NAV_DETAIL_PARALLEL_H__
#define __CVC_NAV_DETAIL_PARALLEL_H__

#include <atomic>
#include <cvc/core/thread_pool.h>
#include <functional>
#include <thread>
#include <utility>
#include <vector>

namespace cvc {
namespace nav {
namespace detail {

// Run fn(i) for i in [0, n) across `num_threads` workers (<=0 => hardware
// concurrency). The calling thread participates, so num_threads==1 runs inline.
//
// This spawn-per-call form is the FALLBACK, used when no thread pool is injected
// (headless tools, the trainer, tests). It creates and joins threads on every
// call, so `min_items_per_thread` -- the caller's estimate of how much work an
// item is worth -- keeps a cheap fan-out inline: the spawn is ~180us per worker
// on a 20-core box (~3.5ms of pure std::thread churn), a bargain for an A* batch
// (each item is a whole search) and ruinous for a per-tick kernel whose items are
// microseconds. Prefer the pool overload below when a cvc::thread_pool is
// available (the sim path threads one through) -- it pays NO per-call spawn.
//
// Measured on the fused drive (nav_drive_step, 192^2 field, one f32 sample +
// a 5-64-64-3 MLP + one bicycle step per agent), auto-threaded vs inline:
//
//     agents      5       8      64     256    1024    4096   16384
//     auto     3.86    6.94   16.34   13.02   19.77   21.55   38.81  ms
//     inline   0.41    0.91    2.52    2.36    9.24   20.37   76.75  ms
//
// Inline wins by ~9x at demo sizes and does not lose until ~4k agents. The
// default of 1 is the historical behaviour, so no existing caller moves.
template <class F> void parallel_for(int n, int num_threads, F &&fn, int min_items_per_thread = 1) {
  if (n <= 0)
    return;
  int nt = num_threads > 0 ? num_threads : static_cast<int>(std::thread::hardware_concurrency());
  if (nt < 1)
    nt = 1;
  if (nt > n)
    nt = n;
  if (min_items_per_thread > 1) {
    const int afford = n / min_items_per_thread;
    nt = afford < 1 ? 1 : (nt < afford ? nt : afford);
  }
  if (nt == 1) {
    for (int i = 0; i < n; ++i)
      fn(i);
    return;
  }
  std::atomic<int> next{0};
  auto worker = [&]() {
    for (int i = next.fetch_add(1); i < n; i = next.fetch_add(1))
      fn(i);
  };
  std::vector<std::thread> pool;
  pool.reserve(nt - 1);
  for (int t = 0; t < nt - 1; ++t)
    pool.emplace_back(worker);
  worker();
  for (auto &th : pool)
    th.join();
}

// Pool-aware form: dispatch fn(i) for i in [0, n) through an INJECTED
// cvc::thread_pool (persistent workers, NO per-call thread spawn) when one is
// given; fall back to the spawn-per-call form above when `pool` is null. This is
// what the sim hot path uses — the pool is owned elsewhere (e.g. cvc::app) and
// passed down by reference, so the kernels stay free of any global/singleton
// state. `min_items_per_thread` bounds the participant count for a cheap fan-out
// exactly as the fallback does (a tiny fan-out runs on few workers, or inline),
// and `num_threads > 0` further caps it; <= 0 (with a fine grain) uses the pool.
template <class F>
void parallel_for(cvc::thread_pool *pool, int n, int num_threads, F &&fn,
                  int min_items_per_thread = 1) {
  if (n <= 0)
    return;
  if (pool) {
    int cap = num_threads > 0 ? num_threads : 0; // 0 => whole pool
    if (min_items_per_thread > 1) {
      const int afford = n / min_items_per_thread;
      const int grain = afford < 1 ? 1 : afford; // grain==1 => the pool runs it inline
      cap = (cap <= 0) ? grain : (cap < grain ? cap : grain);
    }
    pool->parallel_for(n, std::function<void(int)>(std::forward<F>(fn)), cap);
    return;
  }
  parallel_for(n, num_threads, std::forward<F>(fn), min_items_per_thread);
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_PARALLEL_H__
