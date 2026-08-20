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
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

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
  // One publisher per scene, owned by its SceneGraph and constructed with the
  // scene's app (SceneGraph::publisher()). There is deliberately no process-wide
  // instance() — a node reaches its publisher through its SceneGraph, so scene
  // state stays in the app the scene actually runs under, not a global.
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
  // The worker runs on the publisher's OWN dedicated thread, joined in stop().
  // Deliberately NOT the app thread pool: a publisher's worker is long-lived (it
  // runs for the scene's whole life), and one publisher exists per scene, so pinning
  // a bounded-pool slot per scene would starve the pool and, past maxPoolSize live
  // scenes, deadlock — a queued long-lived task can only run when another pool task
  // returns, which a permanent worker never does. It is ONE mostly-idle thread per
  // live scene (not one per write), joined deterministically at teardown.
  void start(double hz = 30.0);
  void stop();
  bool running() const { return m_running.load(); }

  // ── back pressure ────────────────────────────────────────────────────────
  // Coalescing bounds the queue by DISTINCT PATHS, which under load is every
  // animated node — so it is not a bound at all when the worker falls behind.
  // This is: past the cap, updates are SHED rather than queued.
  //
  // Shedding is safe precisely because the queue is eventually consistent and
  // every animated node republishes next frame: a dropped value is superseded,
  // never lost. What it must not do is STARVE, and that is subtler than it
  // looks. Refusing the newcomer starves the tail of the scene outright, since
  // publish order is stable frame to frame. Evicting a random INCUMBENT is not
  // fair either — it still favours late arrivals, because an early path has to
  // survive every subsequent eviction: with a cap of 64 and 500 offers, the
  // first path survives a window only ~0.1% of the time.
  //
  // So admission is RESERVOIR SAMPLING: with N offers in a window and a cap of
  // k, every offer ends up retained with the same probability k/N, whatever its
  // position. That is what makes "eventually consistent" actually true.
  void set_max_pending(std::size_t n);
  std::size_t max_pending() const;

  // Observability — a queue that silently grows, or silently sheds, is worse
  // than no queue. Same principle as world_clock reporting dropped_steps.
  std::size_t pending() const;
  std::uint64_t written() const { return m_written.load(); }
  std::uint64_t coalesced() const { return m_coalesced.load(); }
  std::uint64_t dropped() const { return m_dropped.load(); }

private:
  void worker(double hz);

  cvc::app &m_ctx;
  mutable std::mutex m_mutex;
  std::unordered_map<std::string, std::string> m_pending;
  // Pending keys, kept alongside the map purely so eviction can be O(1) AND
  // uniform. Probing random hash buckets does not work: the map keeps a large
  // bucket_count from earlier peaks, so at a low load factor the probes miss,
  // eviction quietly fails and the "cap" stops capping anything.
  std::vector<std::string> m_keys;
  std::condition_variable m_wake;
  std::thread m_thread; // the drain worker; joined in stop() so it never outlives us
  std::atomic<bool> m_running{false};
  std::atomic<std::uint64_t> m_written{0};
  std::atomic<std::uint64_t> m_coalesced{0};
  std::atomic<std::uint64_t> m_dropped{0};
  std::size_t m_maxPending = 8192;
  std::uint64_t m_seen = 0; // new-path offers this window (reservoir denominator)
  std::minstd_rand m_rng{12345};
};

} // namespace gl
} // namespace cvc

#endif // __CVC_GL_STATE_PUBLISHER_H__
