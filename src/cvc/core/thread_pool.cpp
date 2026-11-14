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

// Re-entrancy and contexts are the pool's own: it knows its workers by id (an immutable list)
// and, under _mtx, the callers running a posted job's indices and the contexts threads set on it.
// No thread_local: two pools on one thread, or two apps' pools, never see each other's.

fanout_context_scope::fanout_context_scope(thread_pool &pool, fanout_context *ctx) : pool_(pool) {
  std::lock_guard<std::mutex> lk(pool_._mtx);
  pool_._scopes.emplace_back(std::this_thread::get_id(), ctx);
}

fanout_context_scope::~fanout_context_scope() {
  const std::thread::id self = std::this_thread::get_id();
  std::lock_guard<std::mutex> lk(pool_._mtx);
  for (auto it = pool_._scopes.end(); it != pool_._scopes.begin();)
    if ((--it)->first == self) {
      pool_._scopes.erase(it); // this thread's latest: scopes nest
      return;
    }
}

thread_pool::thread_pool(unsigned n_workers) {
  unsigned n = n_workers;
  if (n_workers == 0) {
    const unsigned hc = std::thread::hardware_concurrency();
    n = hc > 1 ? hc - 1u : 0u; // the calling thread is the +1st participant
  }
  _worker_ctx.assign(n, nullptr); // before any worker can run chunks
  _workers.reserve(n);
  for (unsigned i = 0; i < n; ++i)
    _workers.emplace_back([this, i] { worker_loop(static_cast<int>(i)); });
  // No job can be posted before this returns, so no one reads the list before it is complete.
  _worker_ids.reserve(n);
  for (const std::thread &w : _workers)
    _worker_ids.push_back(w.get_id());
}

int thread_pool::worker_index_here() const {
  const std::thread::id self = std::this_thread::get_id();
  for (std::size_t i = 0; i < _worker_ids.size(); ++i)
    if (_worker_ids[i] == self)
      return static_cast<int>(i);
  return -1;
}

fanout_context *thread_pool::scope_context_locked(std::thread::id self) const {
  for (auto it = _scopes.rbegin(); it != _scopes.rend(); ++it)
    if (it->first == self)
      return it->second;
  return nullptr;
}

bool thread_pool::caller_here() {
  const std::thread::id self = std::this_thread::get_id();
  const int high = _caller_high.load(std::memory_order_relaxed);
  for (int i = 0; i < high; ++i)
    if (_caller_slots[i].load(std::memory_order_relaxed) == self)
      return true;
  if (_caller_overflow_count.load(std::memory_order_relaxed) == 0)
    return false;
  std::lock_guard<std::mutex> lk(_mtx);
  return std::find(_caller_overflow.begin(), _caller_overflow.end(), self) !=
         _caller_overflow.end();
}

void thread_pool::add_caller_locked(std::thread::id self) {
  for (int i = 0; i < kCallerSlots; ++i)
    if (_caller_slots[i].load(std::memory_order_relaxed) == std::thread::id()) {
      _caller_slots[i].store(self, std::memory_order_relaxed);
      if (i >= _caller_high.load(std::memory_order_relaxed))
        _caller_high.store(i + 1, std::memory_order_relaxed);
      return;
    }
  _caller_overflow.push_back(self);
  _caller_overflow_count.store(static_cast<int>(_caller_overflow.size()),
                               std::memory_order_relaxed);
}

void thread_pool::remove_caller_locked(std::thread::id self) {
  for (int i = 0; i < kCallerSlots; ++i)
    if (_caller_slots[i].load(std::memory_order_relaxed) == self) {
      _caller_slots[i].store(std::thread::id(), std::memory_order_relaxed);
      return;
    }
  _caller_overflow.erase(std::find(_caller_overflow.begin(), _caller_overflow.end(), self));
  _caller_overflow_count.store(static_cast<int>(_caller_overflow.size()),
                               std::memory_order_relaxed);
}

fanout_context *thread_pool::fanout_context_here() {
  if (const int w = worker_index_here(); w >= 0)
    return _worker_ctx[static_cast<std::size_t>(w)];
  std::lock_guard<std::mutex> lk(_mtx);
  return scope_context_locked(std::this_thread::get_id());
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

void thread_pool::worker_loop(int worker_index) {
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

    // A fan-out nested in these chunks runs inline: this thread is one of the pool's workers.
    if (fanout_context *ctx = j->ctx) {
      // The poster's context, as this worker's own while it runs the chunks.
      fanout_context *&mine = _worker_ctx[static_cast<std::size_t>(worker_index)];
      ctx->run_chunks([this, j, ctx, &mine] {
        mine = ctx;
        run_indices(*j);
        mine = nullptr;
      });
    } else {
      run_indices(*j); // noexcept: task throws are latched into *j, not propagated here
    }

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

  // Trivial, workerless, or re-entrant (on one of this pool's workers, or nested in a job this
  // thread posted to it): just run it on the caller. A task throw propagates straight to our
  // caller (there are no worker threads to strand and no shared stack job to unwind past).
  const auto run_inline = [&] {
    for (int i = 0; i < n; ++i)
      fn(i);
  };
  if (n == 1 || _workers.empty() || worker_index_here() >= 0 || caller_here()) {
    run_inline();
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

  // No worker may join (max_par 1): the caller runs every index, as inline.
  if (enlisted == 0) {
    run_inline();
    return;
  }

  job j;
  j.n = n;
  j.fn = &fn;
  j.cursor.store(0, std::memory_order_relaxed);
  j.remaining.store(1, std::memory_order_relaxed); // the caller; workers add themselves
  j.enlisted = enlisted;
  j.caller = std::this_thread::get_id();

  unsigned wake = 0;
  {
    std::lock_guard<std::mutex> lk(_mtx);
    j.ctx = scope_context_locked(j.caller);
    add_caller_locked(j.caller);
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

  run_indices(j); // noexcept: task throws are latched into j, re-thrown below once joined

  {
    // No worker joins from here on; then wait for the ones that did.
    std::unique_lock<std::mutex> lk(_mtx);
    unlink_locked(j);
    remove_caller_locked(j.caller);
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
