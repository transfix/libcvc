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

// thread_pool.cpp — see thread_pool.h.
//
// Sync model: workers park on _wake until _epoch advances (a job posted). Each
// participant (the caller and every enlisted worker) claims indices from the
// job's atomic cursor until it is drained, then decrements the job's `remaining`
// participant count with release semantics; the caller waits on _done for
// `remaining` to reach 0 (acquire), which establishes happens-before between the
// workers' writes and the caller's return. The job lives on the caller's stack:
// a worker only touches it between waking and its final decrement, and the caller
// clears _job only after `remaining` hits 0, so no worker races the stack unwind.
//
// There is ONE job slot, so `remaining` can only drain if every enlisted worker
// eventually wakes and adopts the job that is still in the slot. _post_mtx keeps
// that invariant across orchestrator threads: a caller holds it from posting its
// job until the job has drained and the slot is cleared, so a second concurrent
// parallel_for blocks rather than overwriting _job/_epoch out from under a
// slow-to-wake worker (which would strand the first caller in _done.wait), and a
// finishing caller's `_job = nullptr` can never stomp a freshly posted job.

#include <cvc/core/thread_pool.h>
#include <exception>

namespace cvc {

namespace {
// Set while a thread is executing work for a given pool. A parallel_for that
// finds itself already inside its own pool runs inline rather than posting a
// nested job the (busy) workers could never pick up — which would deadlock.
thread_local const thread_pool *t_active_pool = nullptr;
// The context this thread's fan-outs carry (fanout_context_scope).
thread_local fanout_context *t_fanout_context = nullptr;
} // namespace

fanout_context *current_fanout_context() { return t_fanout_context; }

fanout_context_scope::fanout_context_scope(fanout_context *ctx) : outer_(t_fanout_context) {
  t_fanout_context = ctx;
}

fanout_context_scope::~fanout_context_scope() { t_fanout_context = outer_; }

thread_pool::thread_pool(unsigned n_workers) {
  unsigned n = n_workers;
  if (n_workers == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    n = hc > 1 ? hc - 1u : 0u; // the calling thread is the +1st participant
  }
  _workers.reserve(n);
  for (unsigned i = 0; i < n; ++i)
    _workers.emplace_back([this, i] { worker_loop(static_cast<int>(i)); });
}

thread_pool::~thread_pool() {
  {
    std::lock_guard<std::mutex> lk(_mtx);
    _stop = true;
  }
  _wake.notify_all();
  for (auto &w : _workers)
    if (w.joinable())
      w.join();
}

void thread_pool::run_indices(job &j) noexcept {
  for (int i = j.cursor.fetch_add(1, std::memory_order_relaxed); i < j.n;
       i = j.cursor.fetch_add(1, std::memory_order_relaxed)) {
    // A task already threw: drain the remaining indices without running work, so the
    // fan-out still joins promptly and every participant reaches its `remaining`
    // decrement. (Acquire pairs with the release store in the catch below.)
    if (j.aborted.load(std::memory_order_acquire))
      continue;
    try {
      (*j.fn)(i);
    } catch (...) {
      // Latch the first exception; it is re-thrown once from the orchestrator after
      // the whole fan-out has joined. Never let it escape this thread.
      std::lock_guard<std::mutex> lk(j.emtx);
      if (!j.eptr)
        j.eptr = std::current_exception();
      j.aborted.store(true, std::memory_order_release);
    }
  }
}

void thread_pool::worker_loop(int worker_index) {
  std::uint64_t seen = 0;
  for (;;) {
    job *j = nullptr;
    {
      std::unique_lock<std::mutex> lk(_mtx);
      _wake.wait(lk, [&] { return _stop || _epoch != seen; });
      if (_stop)
        return;
      seen = _epoch;
      j = _job;
      // Not enlisted for this job (a capped fan-out): go back to sleep.
      if (j && worker_index >= j->enlisted)
        j = nullptr;
    }
    if (!j)
      continue;

    const thread_pool *prev = t_active_pool;
    t_active_pool = this;
    if (fanout_context *ctx = j->ctx) {
      // The poster's context, as this thread's own while it runs the chunks.
      ctx->run_chunks([this, j, ctx] {
        fanout_context_scope scope(ctx);
        run_indices(*j);
      });
    } else {
      run_indices(*j); // noexcept: task throws are latched into *j, not propagated here
    }
    t_active_pool = prev;

    // Last participant out wakes the caller. (No j access past this point.)
    if (j->remaining.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      std::lock_guard<std::mutex> lk(_mtx);
      _done.notify_one();
    }
  }
}

void thread_pool::parallel_for(int n, const std::function<void(int)> &fn, int max_par) {
  if (n <= 0)
    return;

  // Trivial, workerless, or re-entrant: just run it on the caller. A task throw
  // propagates straight to our caller (there are no worker threads to strand and no
  // shared stack job to unwind past); we only have to restore t_active_pool first so a
  // later parallel_for on this thread is not stuck on the re-entrant path forever.
  if (n == 1 || _workers.empty() || t_active_pool == this) {
    const thread_pool *prev = t_active_pool;
    t_active_pool = this;
    try {
      for (int i = 0; i < n; ++i)
        fn(i);
    } catch (...) {
      t_active_pool = prev;
      throw;
    }
    t_active_pool = prev;
    return;
  }

  // One posted job at a time: concurrent orchestrators serialize here, each
  // holding the lock until its job has fully drained and the slot is cleared.
  std::lock_guard<std::mutex> post_lk(_post_mtx);

  // Enlist workers besides the caller: capped by the pool size, by max_par (which
  // counts the caller), and by the work available (no idle wakeups).
  int enlisted = static_cast<int>(_workers.size());
  if (max_par > 0 && max_par - 1 < enlisted)
    enlisted = max_par - 1;
  if (enlisted > n - 1)
    enlisted = n - 1;
  if (enlisted < 0)
    enlisted = 0;

  job j;
  j.n = n;
  j.fn = &fn;
  j.cursor.store(0, std::memory_order_relaxed);
  j.remaining.store(enlisted + 1, std::memory_order_relaxed); // + the caller
  j.enlisted = enlisted;
  j.ctx = t_fanout_context;

  {
    std::lock_guard<std::mutex> lk(_mtx);
    _job = &j;
    ++_epoch;
  }
  _wake.notify_all();

  const thread_pool *prev = t_active_pool;
  t_active_pool = this;
  run_indices(j); // noexcept: task throws are latched into j, re-thrown below once joined
  t_active_pool = prev;

  // If the caller isn't the last one out, wait for the workers to drain.
  if (j.remaining.fetch_sub(1, std::memory_order_acq_rel) != 1) {
    std::unique_lock<std::mutex> lk(_mtx);
    _done.wait(lk, [&] { return j.remaining.load(std::memory_order_acquire) == 0; });
  }
  {
    std::lock_guard<std::mutex> lk(_mtx);
    _job = nullptr;
  }

  // The whole fan-out has joined (remaining hit 0, which the acquire load above
  // synchronizes with), so j.eptr is fully published and no worker touches j anymore.
  // Re-throw the first task exception, if any, on the orchestrator's thread — matching
  // the semantics of a serial loop that let the throw escape.
  std::exception_ptr eptr;
  {
    std::lock_guard<std::mutex> lk(j.emtx);
    eptr = j.eptr;
  }
  if (eptr)
    std::rethrow_exception(eptr);
}

} // namespace cvc
