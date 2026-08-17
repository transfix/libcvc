/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_GL_STATE_PUBLISHER_H__
#define __CVC_GL_STATE_PUBLISHER_H__

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

namespace cvc {
class app;
namespace gl {

// ---------------------------------------------------------------------------
// cvc::gl::state_publisher — keep scene state in the state tree, off the render
// path.
// ---------------------------------------------------------------------------
// Scene changes MUST reach the state tree: it is what the dashboards, the
// scripting surface and persistence all read. But doing it inline made moving a
// node startlingly expensive. Measured on one leaf GraphicsNode::setPosition:
//
//     ostringstream                  1.6 us
//     getState("position") lookup    8.2 us
//     value() when it changes       21.9 us   (lookup + signal)
//     whole setPosition             27.4 us
//
// so ~80% of posing a node was the state write, and worse, the resulting
// valueChanged came back through handleStateChanged and ran the whole transform
// cascade a SECOND time. Animating a few hundred nodes paid all of that per
// frame.
//
// This decouples the two. Writers enqueue (path, value); a background thread
// applies them at a fixed cadence, COALESCED so only the last value written to
// a path in a window is actually stored. Animation is exactly the case that
// benefits: a node posed 60 times a second between flushes collapses to one
// write, and the state tree still ends up holding the current value.
//
// What this deliberately does NOT promise: that a value is visible in the state
// tree the instant you set it. It is eventually-consistent, bounded by the flush
// interval. Anything needing read-after-write (a test, a script that sets then
// immediately reads) should call flush() first.
class state_publisher {
public:
  // The shared publisher cvcGL's nodes use.
  static state_publisher &instance();

  explicit state_publisher(cvc::app &ctx);
  ~state_publisher();

  state_publisher(const state_publisher &) = delete;
  state_publisher &operator=(const state_publisher &) = delete;

  // Queue a write. Cheap: two string copies and a hash insert. Later writes to
  // the same path overwrite earlier ones — the point of the exercise.
  void publish(const std::string &path, std::string value);

  // Apply everything queued, on the CALLING thread. Used by the worker, and by
  // anyone who needs the tree current right now (tests, teardown).
  void flush();

  // Start/stop the background flusher. `hz` is the flush cadence; matching the
  // world clock's rate keeps state updates in step with simulation ticks rather
  // than with however fast the renderer happens to be running.
  //
  // The worker is drawn from the APP'S EXISTING THREAD POOL
  // (app::startThreadPooled), not spawned here — cvc::app already owns a bounded
  // pool, and a library quietly standing up its own threads alongside it is how
  // a process ends up oversubscribed. One long-lived task, not one per write.
  void start(double hz = 30.0);
  void stop();
  bool running() const { return m_running.load(); }

  // Observability — a queue that silently grows is worse than no queue.
  std::size_t pending() const;
  std::uint64_t written() const { return m_written.load(); }
  std::uint64_t coalesced() const { return m_coalesced.load(); }

private:
  void worker(double hz);

  cvc::app &m_ctx;
  mutable std::mutex m_mutex;
  std::unordered_map<std::string, std::string> m_pending;
  std::condition_variable m_wake;
  std::atomic<bool> m_running{false};
  std::atomic<std::uint64_t> m_written{0};
  std::atomic<std::uint64_t> m_coalesced{0};
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_STATE_PUBLISHER_H__
