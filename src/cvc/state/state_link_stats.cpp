/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <chrono>
#include <cvc/state/state_link_stats.h>

namespace cvc {

void state_link_stats::note_rx(std::size_t bytes) noexcept {
  _bytes_in.fetch_add(bytes, std::memory_order_relaxed);
  _frames_in.fetch_add(1, std::memory_order_relaxed);
}

void state_link_stats::note_tx(std::size_t bytes) noexcept {
  _bytes_out.fetch_add(bytes, std::memory_order_relaxed);
  _frames_out.fetch_add(1, std::memory_order_relaxed);
}

std::uint64_t state_link_stats::next_ping_seq() noexcept {
  return _pings.fetch_add(1, std::memory_order_relaxed) + 1;
}

void state_link_stats::note_rtt(std::int64_t rtt_ns) noexcept {
  if (rtt_ns < 0)
    return; // a clock that went backwards: not a measurement
  const std::int64_t us = rtt_ns / 1000;
  _pongs.fetch_add(1, std::memory_order_relaxed);
  _rtt_last_us.store(us, std::memory_order_relaxed);
  std::int64_t mn = _rtt_min_us.load(std::memory_order_relaxed);
  while ((mn < 0 || us < mn) &&
         !_rtt_min_us.compare_exchange_weak(mn, us, std::memory_order_relaxed)) {
  }
  std::lock_guard<std::mutex> lk(_rtt_mu);
  _rtt_ring[_rtt_next] = us;
  _rtt_next = (_rtt_next + 1) % kRttWindow;
  if (_rtt_count < kRttWindow)
    ++_rtt_count;
}

void state_link_stats::note_peer_clock(std::int64_t peer_wall_ns, std::int64_t rtt_ns,
                                       std::int64_t local_wall_ns) noexcept {
  if (peer_wall_ns <= 0 || rtt_ns < 0)
    return;
  if (local_wall_ns <= 0)
    local_wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::system_clock::now().time_since_epoch())
                        .count();
  const std::int64_t offset_us = (peer_wall_ns - (local_wall_ns - rtt_ns / 2)) / 1000;
  std::lock_guard<std::mutex> lk(_rtt_mu);
  _clock_ring[_clock_next] = {rtt_ns / 1000, offset_us};
  _clock_next = (_clock_next + 1) % kRttWindow;
  if (_clock_count < kRttWindow)
    ++_clock_count;
}

void state_link_stats::note_media_sent() noexcept {
  _media_sent.fetch_add(1, std::memory_order_relaxed);
}

void state_link_stats::note_media_dropped() noexcept {
  _media_dropped.fetch_add(1, std::memory_order_relaxed);
}

state_link_stats::counters state_link_stats::read() const noexcept {
  counters c;
  c.media_sent = _media_sent.load(std::memory_order_relaxed);
  c.media_dropped = _media_dropped.load(std::memory_order_relaxed);
  c.bytes_in = _bytes_in.load(std::memory_order_relaxed);
  c.bytes_out = _bytes_out.load(std::memory_order_relaxed);
  c.frames_in = _frames_in.load(std::memory_order_relaxed);
  c.frames_out = _frames_out.load(std::memory_order_relaxed);
  c.pings_sent = _pings.load(std::memory_order_relaxed);
  c.pongs_received = _pongs.load(std::memory_order_relaxed);
  const auto ms = [](std::int64_t us) { return us < 0 ? -1.0 : static_cast<double>(us) / 1000.0; };
  {
    std::lock_guard<std::mutex> lk(_rtt_mu);
    if (_rtt_count > 0) {
      std::array<std::int64_t, kRttWindow> w = _rtt_ring;
      std::sort(w.begin(), w.begin() + _rtt_count);
      const int n = _rtt_count;
      const std::int64_t med = (n % 2) ? w[n / 2] : (w[n / 2 - 1] + w[n / 2]) / 2;
      c.rtt_ms = ms(med);
    }
    if (_clock_count > 0) { // the reading with the shortest round trip
      const auto *best = &_clock_ring[0];
      for (int i = 1; i < _clock_count; ++i)
        if (_clock_ring[i].first < best->first)
          best = &_clock_ring[i];
      c.peer_clock = true;
      c.clock_rtt_ms = static_cast<double>(best->first) / 1000.0;
      c.clock_offset_ms = static_cast<double>(best->second) / 1000.0;
    }
  }
  c.rtt_last_ms = ms(_rtt_last_us.load(std::memory_order_relaxed));
  c.rtt_min_ms = ms(_rtt_min_us.load(std::memory_order_relaxed));
  return c;
}

std::string state_link_rate_meter::key_of(const state_link_report &r) {
  return r.transport + ":" + r.remote_node_id + (r.accepted ? ":in" : "");
}

std::map<std::string, state_link_rate_meter::rate>
state_link_rate_meter::update(const std::vector<state_link_report> &reports, double now_s) {
  std::map<std::string, rate> out;
  std::map<std::string, sample> next;
  for (const state_link_report &r : reports) {
    const std::string key = key_of(r);
    rate rt;
    auto it = _last.find(key);
    // A link whose counters went DOWN is a new connection under the same key: start over.
    if (it != _last.end() && now_s > it->second.t && r.stats.bytes_in >= it->second.bytes_in &&
        r.stats.bytes_out >= it->second.bytes_out) {
      const double dt = now_s - it->second.t;
      rt.in_bps = static_cast<double>(r.stats.bytes_in - it->second.bytes_in) / dt;
      rt.out_bps = static_cast<double>(r.stats.bytes_out - it->second.bytes_out) / dt;
    }
    out[key] = rt;
    next[key] = sample{r.stats.bytes_in, r.stats.bytes_out, now_s};
  }
  _last.swap(next); // links no longer reported are forgotten
  return out;
}

} // namespace cvc
