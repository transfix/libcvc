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
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace cvc {

// What a fan-out carries from the thread that starts it onto the workers that run its
// chunks. state_exec's async calls install one while a kernel runs, so the CPU time its
// parallel_for chunks use is charged to the calling process and the process's allocation
// limit holds in them too (docs/STATE_EXEC_EXCEPTIONS.md 3.8). The thread that starts the
// fan-out runs chunks as well; those count as its own work, outside run_chunks.
class fanout_context {
public:
  virtual ~fanout_context() = default;
  // Run `chunks`, a worker's share of the fan-out, on that worker. It must call `chunks`
  // exactly once, which never throws.
  virtual void run_chunks(const std::function<void()> &chunks) noexcept = 0;
};

// The context this thread's fan-outs carry (null: none). A worker running chunks under a
// context has it as its own, so a fan-out nested in a chunk carries it too.
fanout_context *current_fanout_context();

// RAII: this thread's fan-outs carry `ctx` while the scope lives; the previous one after.
class fanout_context_scope {
public:
  explicit fanout_context_scope(fanout_context *ctx);
  ~fanout_context_scope();
  fanout_context_scope(const fanout_context_scope &) = delete;
  fanout_context_scope &operator=(const fanout_context_scope &) = delete;

private:
  fanout_context *outer_;
};

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
// intended use for a shared pool like app::computePool(). Concurrent fan-outs run
// side by side: each posted job joins a FIFO list, and a free worker takes part in
// the oldest job that still has unclaimed indices and room under its cap, so a
// fan-out started while another is in flight begins at once on its own caller and
// on whatever workers are free (or become free), instead of waiting for the first
// to drain. A post wakes only as many idle workers as the job can enlist (a capped
// fan-out does not wake the whole pool).
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
  //
  // Exception safety: if one or more fn(i) throw, the fan-out still joins fully (no
  // worker is left running, no `remaining` count is stranded), the remaining indices
  // are drained without invoking fn, and the FIRST thrown exception is re-thrown from
  // parallel_for on the caller's thread. A throw thus propagates to the caller exactly
  // as a serial loop would, instead of escaping a worker thread (std::terminate) or
  // unwinding the caller past the still-running workers. Which index "wins" under
  // concurrent throws is unspecified; fn should not rely on all indices having run when
  // any throws.
  void parallel_for(int n, const std::function<void(int)> &fn, int max_par = 0);

private:
  void worker_loop(int worker_index);

  // One fan-out. Lives on the caller's stack for the duration of a parallel_for. A worker
  // joins it only while it is linked into the pool's job list (under _mtx), and touches it
  // only between joining and its final `remaining` decrement; the caller unlinks it before it
  // waits for `remaining` to drain, so no worker can join once the caller's wait begins.
  struct job {
    int n = 0;
    const std::function<void(int)> *fn = nullptr;
    std::atomic<int> cursor{0};       // next index to claim
    std::atomic<int> remaining{0};    // participants (caller + joined workers) still running
    int enlisted = 0;                 // most workers that may join (guarded by _mtx)
    int joined = 0;                   // workers that have joined (guarded by _mtx)
    std::atomic<bool> aborted{false}; // a task threw; remaining indices drain without running fn
    std::exception_ptr eptr;          // first exception thrown by any task (guarded by emtx)
    std::mutex emtx;                  // guards eptr's one-time set
    fanout_context *ctx = nullptr;    // the poster's context: workers run their chunks under it
    job *next = nullptr;              // the job list (guarded by _mtx)
  };

  // Claim and run indices from `j` on the calling thread until the cursor drains.
  // noexcept: a task exception is caught, the FIRST one is stored in j.eptr and the
  // job is marked aborted (later indices drain without invoking fn), so the exception
  // can be re-thrown once from the orchestrator after the whole fan-out has joined —
  // never out of a worker thread (which would std::terminate) or mid-fan-out (which
  // would unwind the caller's stack job while workers still reference it).
  void run_indices(job &j) noexcept;

  // The oldest linked job a worker may still join (room under its cap, indices left), or null.
  // Caller holds _mtx.
  job *joinable_job_locked() const;
  void unlink_locked(job &j);

  std::vector<std::thread> _workers;
  std::mutex _mtx;
  std::condition_variable _wake; // idle workers park here until a joinable job is posted
  std::condition_variable _done; // parallel_for parks here until its job drains
  job *_head = nullptr;          // posted jobs, oldest first (each on its caller's stack)
  job *_tail = nullptr;
  unsigned _idle = 0; // workers parked on _wake
  bool _stop = false;
};

} // namespace cvc

#endif // __CVC_CORE_THREAD_POOL_H__
