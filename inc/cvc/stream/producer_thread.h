/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_STREAM_PRODUCER_THREAD_H__
#define __CVC_STREAM_PRODUCER_THREAD_H__

#include <atomic>
#include <cstdint>
#include <functional>
#include <thread>

// producer_thread — a dedicated worker that drives a stream producer at a fixed
// rate (roadmap STATE_BINARY_STREAMING.md §3.4).
//
// This is the ONLY sanctioned host for a long-running publish() loop. It owns a
// raw std::thread with a self-managed run flag and joins deterministically in
// stop()/dtor, mirroring cvc::nav::sim_thread. It must NOT be a compute-pool job
// and must NOT be submitted to app::startThreadPooled: the compute pool runs one
// job at a time and holds its post mutex until the job drains, so a
// never-returning producer loop would deadlock the entire compute subsystem
// (nav ticks, http/fetch, every launch_pool_task); the bounded background-task
// pool would likewise be starved of a slot.
//
// The per-tick callback returns true to keep running (false to stop). It runs on
// the dedicated thread and must be self-contained: acquire+fill a pool slab,
// channel.publish(), and post lightweight events via async_scheduler::
// post_message. It MUST NOT touch the DSL evaluator or write the state tree
// directly (post events; let the pump thread apply them).
//
// Lifetime contract: whatever the tick captures (the stream/channel/pool) must
// outlive the producer_thread. Owners stop() (which joins) the producer before
// destroying those objects.

namespace cvc {
namespace stream {

class producer_thread {
public:
  // `tick` is invoked once per period; returning false stops the loop. `hz` is
  // the target tick rate (clamped to a sane positive value).
  producer_thread(std::function<bool()> tick, double hz);
  ~producer_thread(); // stop()

  producer_thread(const producer_thread &) = delete;
  producer_thread &operator=(const producer_thread &) = delete;

  void start(); // no-op if already started
  void stop();  // idempotent; joins the worker
  void set_rate(double hz);

  bool running() const noexcept { return run_.load(std::memory_order_relaxed); }
  long ticks() const noexcept { return ticks_.load(std::memory_order_relaxed); }
  long behind() const noexcept { return behind_.load(std::memory_order_relaxed); }

private:
  void run();

  std::function<bool()> tick_;
  std::atomic<double> period_;
  std::atomic<bool> run_{false};
  std::atomic<long> ticks_{0};
  std::atomic<long> behind_{0};
  std::thread thr_;
};

} // namespace stream
} // namespace cvc

#endif // __CVC_STREAM_PRODUCER_THREAD_H__
