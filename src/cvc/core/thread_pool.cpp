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
// Sync model: a parallel_for links its job (on the caller's stack) into a FIFO list under _mtx
// and wakes at most as many idle workers as the job may enlist. A worker, under _mtx, joins the
// oldest linked job that still has room under its cap and unclaimed indices: it bumps the job's
// `joined` and its `remaining` participant count, then, without the lock, claims indices from
// the job's atomic cursor until it drains, and decrements `remaining` with release semantics.
// Once the caller has run out of indices itself it unlinks the job under _mtx (no worker can
// join after that, and every join happened before it, under the same mutex), drops its own share
// of `remaining`, and waits on _done for it to reach 0 (acquire), which establishes
// happens-before between the workers' writes and the caller's return. A worker never touches a
// job after its final decrement (the last one out only locks _mtx to notify _done), so the
// caller's stack unwind cannot race it.
//
// Several jobs may be linked at once: concurrent orchestrators no longer wait for one another.
// Each caller always runs its own job's indices, so every fan-out makes progress even when every
// worker is busy in an older one; workers that finish a job move to the next joinable one before
// they sleep.

#include <algorithm>
#include <cvc/core/thread_class.h>
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

thread_pool::job *thread_pool::joinable_job_locked() const {
  for (job *j = _head; j; j = j->next)
    if (j->joined < j->enlisted && j->cursor.load(std::memory_order_relaxed) < j->n)
      return j;
  return nullptr;
}

void thread_pool::unlink_locked(job &j) {
  job *prev = nullptr;
  for (job *p = _head; p; prev = p, p = p->next) {
    if (p != &j)
      continue;
    (prev ? prev->next : _head) = p->next;
    if (_tail == p)
      _tail = prev;
    p->next = nullptr;
    return;
  }
}

void thread_pool::worker_loop(int /*worker_index*/) {
  // A process class set before this worker started (Linux, where the class is per thread).
  detail::apply_process_class_here();
  for (;;) {
    job *j = nullptr;
    {
      std::unique_lock<std::mutex> lk(_mtx);
      for (;;) {
        if (_stop)
          return;
        j = joinable_job_locked();
        if (j)
          break;
        ++_idle;
        _wake.wait(lk);
        --_idle;
      }
      // Joined under the lock, while the job is linked: its caller can't have begun waiting.
      ++j->joined;
      j->remaining.fetch_add(1, std::memory_order_relaxed);
    }

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

    // Last participant out wakes the callers. (No j access past this point.) _done is shared by
    // every caller waiting on its own job, so wake them all; each re-checks its own count.
    if (j->remaining.fetch_sub(1, std::memory_order_acq_rel) == 1) {
      std::lock_guard<std::mutex> lk(_mtx);
      _done.notify_all();
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

  // Workers that may join besides the caller: capped by the pool size, by max_par (which
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
  j.remaining.store(1, std::memory_order_relaxed); // the caller; workers add themselves
  j.enlisted = enlisted;
  j.ctx = t_fanout_context;

  if (enlisted > 0) {
    unsigned wake = 0;
    {
      std::lock_guard<std::mutex> lk(_mtx);
      (_tail ? _tail->next : _head) = &j;
      _tail = &j;
      wake = std::min<unsigned>(_idle, static_cast<unsigned>(enlisted));
    }
    // Wake only the workers the job can use; busy ones find it when they finish theirs.
    if (wake >= _workers.size())
      _wake.notify_all();
    else
      for (unsigned w = 0; w < wake; ++w)
        _wake.notify_one();
  }

  const thread_pool *prev = t_active_pool;
  t_active_pool = this;
  run_indices(j); // noexcept: task throws are latched into j, re-thrown below once joined
  t_active_pool = prev;

  if (enlisted > 0) {
    // No worker joins from here on; then wait for the ones that did.
    std::unique_lock<std::mutex> lk(_mtx);
    unlink_locked(j);
    if (j.remaining.fetch_sub(1, std::memory_order_acq_rel) != 1)
      _done.wait(lk, [&] { return j.remaining.load(std::memory_order_acquire) == 0; });
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
