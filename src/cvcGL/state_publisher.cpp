/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/state_publisher.h>
#include <utility>
#include <vector>

namespace cvc {
namespace gl {

state_publisher::state_publisher(cvc::app &ctx) : m_ctx(ctx) {}

state_publisher::~state_publisher() {
  stop();
  flush(); // never drop queued state on the floor
}

void state_publisher::publish(const std::string &path, std::string value) {
  std::lock_guard<std::mutex> lock(m_mutex);
  auto it = m_pending.find(path);
  if (it != m_pending.end()) {
    // Superseded before it was ever written — this is the win for animation.
    it->second = std::move(value);
    m_coalesced.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // Reservoir sampling (Algorithm R) over the offers in this flush window, so
  // every offered path is retained with the same probability regardless of when
  // it arrived. Evicting a random incumbent instead would still favour late
  // arrivals, and the head of the scene would effectively never be published.
  ++m_seen;
  if (m_pending.size() >= m_maxPending) {
    const std::size_t j = static_cast<std::size_t>(m_rng() % m_seen);
    m_dropped.fetch_add(1, std::memory_order_relaxed);
    if (j >= m_maxPending || m_keys.empty())
      return; // this offer loses its place in the reservoir
    m_pending.erase(m_keys[j]);
    m_pending.emplace(path, std::move(value));
    m_keys[j] = path;
    return;
  }
  m_keys.push_back(path);
  m_pending.emplace(path, std::move(value));
}

void state_publisher::flush() {
  std::unordered_map<std::string, std::string> batch;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_pending.empty())
      return;
    batch.swap(m_pending); // hold the lock only for the swap
    m_keys.clear();
    m_seen = 0; // new sampling window
  }
  for (auto &kv : batch) {
    // operator()(path) resolves (creating as needed) and value() fires
    // valueChanged only on an actual change — the same path pycvc's state_set
    // uses. Node handlers marshal to their owner thread, so a write from here
    // is safe.
    cvc::state::instance(m_ctx)(kv.first).value(kv.second);
  }
  m_written.fetch_add(batch.size(), std::memory_order_relaxed);
}

void state_publisher::set_max_pending(std::size_t n) {
  std::lock_guard<std::mutex> lock(m_mutex);
  m_maxPending = n ? n : 1;
}

std::size_t state_publisher::max_pending() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_maxPending;
}

std::size_t state_publisher::pending() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_pending.size();
}

void state_publisher::start(double hz) {
  if (m_running.exchange(true))
    return; // already running
  const double rate = hz > 0.0 ? hz : 30.0;
  // Our own dedicated thread (see the header for why not the app pool).
  m_thread = std::thread([this, rate]() { worker(rate); });
}

void state_publisher::stop() {
  m_running.store(false);
  m_wake.notify_all();
  // JOIN before returning: the worker holds a raw `this`, so it must not run past
  // our lifetime. join() is the whole synchronization — no hand-shake to race.
  // Idempotent: both destructors call stop(); after the first join the thread is no
  // longer joinable, so the second call skips it.
  if (m_thread.joinable())
    m_thread.join();
}

void state_publisher::worker(double hz) {
  const auto period = std::chrono::duration<double>(1.0 / hz);
  // A drain must never take the process down: an exception escaping this thread
  // function would std::terminate (the app pool used to catch this for us). A failed
  // write is logged and ends the worker — the scene's state simply stops updating.
  try {
    while (m_running.load()) {
      {
        // Wait on the condition variable rather than sleeping, so stop() is
        // immediate instead of costing up to a full period.
        std::unique_lock<std::mutex> lock(m_mutex);
        m_wake.wait_for(lock, std::chrono::duration_cast<std::chrono::milliseconds>(period),
                        [this]() { return !m_running.load(); });
      }
      flush();
    }
    flush(); // drain whatever arrived during shutdown
  } catch (const std::exception &e) {
    m_ctx.log(1, std::string("state_publisher worker stopped on exception: ") + e.what());
  } catch (...) {
    m_ctx.log(1, "state_publisher worker stopped on unknown exception");
  }
}

} // namespace gl
} // namespace cvc
