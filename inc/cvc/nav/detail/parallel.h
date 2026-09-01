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
// (headless tools, the trainer, tests). In a hot loop it creates and joins
// threads on every call — prefer the pool overload below whenever a
// cvc::thread_pool is available (the sim path threads one through). See
// docs/CVCNAV_CPP_PORT_ROADMAP.md.
template <class F> void parallel_for(int n, int num_threads, F &&fn) {
  if (n <= 0)
    return;
  int nt = num_threads > 0 ? num_threads : static_cast<int>(std::thread::hardware_concurrency());
  if (nt < 1)
    nt = 1;
  if (nt > n)
    nt = n;
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
// cvc::thread_pool (persistent workers, no per-call thread spawn) when one is
// given; fall back to the spawn-per-call form above when `pool` is null. This is
// what the sim hot path uses — the pool is owned elsewhere (e.g. cvc::app) and
// passed down by reference, so the kernels stay free of any global/singleton
// state. `num_threads > 0` becomes a per-call participant cap on the pool; <= 0
// uses the whole pool.
template <class F> void parallel_for(cvc::thread_pool *pool, int n, int num_threads, F &&fn) {
  if (n <= 0)
    return;
  if (pool) {
    pool->parallel_for(n, std::function<void(int)>(std::forward<F>(fn)),
                       num_threads > 0 ? num_threads : 0);
    return;
  }
  parallel_for(n, num_threads, std::forward<F>(fn));
}

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_PARALLEL_H__
