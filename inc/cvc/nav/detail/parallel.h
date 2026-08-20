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
#include <thread>
#include <vector>

namespace cvc {
namespace nav {
namespace detail {

// Run fn(i) for i in [0, n) across `num_threads` workers (<=0 => hardware
// concurrency). The calling thread participates, so num_threads==1 runs inline.
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

} // namespace detail
} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_DETAIL_PARALLEL_H__
