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

#include <cvc/core/thread_pool.h>

namespace cvc {

namespace {
// Set while a thread is executing work for a given pool. A parallel_for that
// finds itself already inside its own pool runs inline rather than posting a
// nested job the (busy) workers could never pick up — which would deadlock.
thread_local const thread_pool *t_active_pool = nullptr;
} // namespace

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
    for (int i = j->cursor.fetch_add(1, std::memory_order_relaxed); i < j->n;
         i = j->cursor.fetch_add(1, std::memory_order_relaxed))
      (*j->fn)(i);
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

  // Trivial, workerless, or re-entrant: just run it on the caller.
  if (n == 1 || _workers.empty() || t_active_pool == this) {
    const thread_pool *prev = t_active_pool;
    t_active_pool = this;
    for (int i = 0; i < n; ++i)
      fn(i);
    t_active_pool = prev;
    return;
  }

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

  {
    std::lock_guard<std::mutex> lk(_mtx);
    _job = &j;
    ++_epoch;
  }
  _wake.notify_all();

  const thread_pool *prev = t_active_pool;
  t_active_pool = this;
  for (int i = j.cursor.fetch_add(1, std::memory_order_relaxed); i < j.n;
       i = j.cursor.fetch_add(1, std::memory_order_relaxed))
    fn(i);
  t_active_pool = prev;

  // If the caller isn't the last one out, wait for the workers to drain.
  if (j.remaining.fetch_sub(1, std::memory_order_acq_rel) != 1) {
    std::unique_lock<std::mutex> lk(_mtx);
    _done.wait(lk, [&] { return j.remaining.load(std::memory_order_acquire) == 0; });
  }
  std::lock_guard<std::mutex> lk(_mtx);
  _job = nullptr;
}

} // namespace cvc
