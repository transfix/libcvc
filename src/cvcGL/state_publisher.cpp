/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <utility>
#include <vector>

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/gl/context.h>
#include <cvc/gl/state_publisher.h>

namespace cvc {
namespace gl {

state_publisher &state_publisher::instance() {
  // Function-local static: constructed on first use, after cvc::gl::context()
  // is usable, and destroyed in reverse order at exit.
  static state_publisher pub(cvc::gl::context());
  // Self-starting: a publisher nobody started is a queue that never drains, and
  // the scene's state would silently stop matching the scene. Callers that want
  // a different cadence call stop()/start(hz); callers that want it synchronous
  // (tests, teardown) call flush().
  static const bool started = [] {
    pub.start();
    return true;
  }();
  (void)started;
  return pub;
}

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
  } else {
    m_pending.emplace(path, std::move(value));
  }
}

void state_publisher::flush() {
  std::unordered_map<std::string, std::string> batch;
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_pending.empty())
      return;
    batch.swap(m_pending); // hold the lock only for the swap
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

std::size_t state_publisher::pending() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_pending.size();
}

void state_publisher::start(double hz) {
  if (m_running.exchange(true))
    return; // already running
  const double rate = hz > 0.0 ? hz : 30.0;
  // Drawn from the app's pool rather than a thread of our own.
  m_ctx.startThreadPooled("cvcgl.state_publisher", [this, rate]() { worker(rate); });
}

void state_publisher::stop() {
  if (!m_running.exchange(false))
    return;
  m_wake.notify_all();
}

void state_publisher::worker(double hz) {
  const auto period = std::chrono::duration<double>(1.0 / hz);
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
}

} // namespace gl
} // namespace cvc
