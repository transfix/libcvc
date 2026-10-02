/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/ariadne/stream/producer_thread.h>

namespace cvc {
namespace ariadne {
namespace stream {

namespace {
using clock = std::chrono::steady_clock;
double clamp_hz(double hz) { return hz > 0.0 ? hz : 60.0; }
} // namespace

producer_thread::producer_thread(std::function<bool()> tick, double hz)
    : tick_(std::move(tick)), period_(1.0 / clamp_hz(hz)) {}

producer_thread::~producer_thread() { stop(); }

void producer_thread::start() {
  if (thr_.joinable())
    return; // a thread already exists (running, or self-finished but not yet reaped by stop())
  run_.store(true);
  thr_ = std::thread(&producer_thread::run, this);
}

void producer_thread::stop() {
  // Gate the join on the THREAD, not on run_: a source that self-stopped (run() cleared run_ so
  // running() stays honest) leaves a finished-but-still-joinable thread, and it MUST be joined here
  // (or ~std::thread on a joinable thread calls std::terminate). Idempotent: after the join thr_ is
  // no longer joinable, so a second stop()/the dtor is a no-op.
  run_.store(false);
  if (thr_.joinable())
    thr_.join();
}

void producer_thread::set_rate(double hz) { period_.store(1.0 / clamp_hz(hz)); }

void producer_thread::run() {
  auto next = clock::now();
  while (run_.load()) {
    if (tick_ && !tick_()) {
      run_.store(false); // producer signalled end-of-stream: keep running() honest for a
      break;             // self-stopped thread (stop()/dtor still join the finished thread)
    }
    ticks_.fetch_add(1, std::memory_order_relaxed);

    const auto period = std::chrono::duration<double>(period_.load());
    next += std::chrono::duration_cast<clock::duration>(period);
    const auto now = clock::now();
    if (next > now) {
      std::this_thread::sleep_for(next - now); // releases the core
    } else {
      behind_.fetch_add(1, std::memory_order_relaxed);
      next = now; // fell behind -> no spiral of death
    }
  }
}

} // namespace stream
} // namespace ariadne
} // namespace cvc
