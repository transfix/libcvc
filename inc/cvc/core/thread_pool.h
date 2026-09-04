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

#ifndef __CVC_CORE_THREAD_POOL_H__
#define __CVC_CORE_THREAD_POOL_H__

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace cvc {

// A persistent-worker thread pool for fork-join data parallelism.
//
// Spawning a fresh std::thread for every parallel region (the pattern in
// cvc::nav::detail::parallel_for's inline fallback) is fine once, but murder in
// a hot loop: a 60 Hz sim tick that fans out ~6 times pays thousands of thread
// creations per second AND floods every core, starving any concurrent thread
// (e.g. a render loop). This pool creates its workers ONCE and reuses them, so a
// parallel_for is just a wake + a barrier.
//
// It is a plain object with NO global/singleton instance: the owner (typically a
// cvc::app) constructs one and hands out a reference; callers that want to
// parallelize borrow that reference. This keeps the low-level compute kernels
// free of any global state — they parallelize through whatever pool they are
// given, or run inline when given none.
//
// parallel_for() is the single primitive. It runs fn(i) for i in [0, n) across
// the pool and blocks until every index is done. The CALLING thread participates
// (it is never idle waiting on the workers), and a shared atomic cursor hands out
// indices so uneven per-item cost self-balances. It is re-entrant: a parallel_for
// invoked from inside a pool task runs inline instead of deadlocking on workers
// that are busy with the outer job.
//
// parallel_for() is safe to call from any number of threads concurrently — the
// intended use for a shared pool like app::computePool(). The pool runs ONE
// fan-out at a time: concurrent orchestrators serialize, each later caller
// blocking until the in-flight job drains before its own is posted (they do not
// lend their thread to the in-flight job while they wait). Total throughput is
// bounded by the pool's workers either way; the serialization only adds latency
// when several threads fan out at once.
class thread_pool {
public:
  // Create `n_workers` background workers. 0 (the default) picks
  // hardware_concurrency() - 1, since the calling thread is itself a worker; a
  // pool with 0 workers is legal and runs everything inline on the caller.
  explicit thread_pool(unsigned n_workers = 0);
  ~thread_pool();

  thread_pool(const thread_pool &) = delete;
  thread_pool &operator=(const thread_pool &) = delete;

  // Total parallelism available: background workers plus the calling thread.
  unsigned concurrency() const { return static_cast<unsigned>(_workers.size()) + 1u; }

  // Run fn(i) for each i in [0, n), returning only once all have completed.
  // `max_par` caps the TOTAL participants (including the caller) for this one
  // call; <= 0 means use the whole pool. A cheap, fine-grained region can pass a
  // small cap so it doesn't wake every worker. n <= 0 is a no-op.
  void parallel_for(int n, const std::function<void(int)> &fn, int max_par = 0);

private:
  void worker_loop(int worker_index);

  // One fan-out. Lives on the caller's stack for the duration of a parallel_for;
  // workers only touch it between the wake and their final `remaining` decrement.
  struct job {
    int n = 0;
    const std::function<void(int)> *fn = nullptr;
    std::atomic<int> cursor{0};    // next index to claim
    std::atomic<int> remaining{0}; // participants (caller + enlisted workers) still running
    int enlisted = 0;              // worker indices [0, enlisted) take part
  };

  std::vector<std::thread> _workers;
  std::mutex _post_mtx; // serializes orchestrators: held from job post to drain+clear
  std::mutex _mtx;
  std::condition_variable _wake; // workers park here until a job is posted
  std::condition_variable _done; // parallel_for parks here until the job drains
  job *_job = nullptr;           // current job (points into the caller's stack)
  std::uint64_t _epoch = 0;      // bumped per posted job so a worker detects a new one
  bool _stop = false;
};

} // namespace cvc

#endif // __CVC_CORE_THREAD_POOL_H__
