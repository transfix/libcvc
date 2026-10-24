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

#ifndef __CVC_STATE_LINK_STATS_H__
#define __CVC_STATE_LINK_STATS_H__

#include <atomic>
#include <cstdint>
#include <cvc/core/namespace.h>
#include <map>
#include <string>
#include <vector>

namespace cvc {

// ----------------
// cvc::state_link_stats
// ----------------
// The quality of ONE transport connection (a link): traffic both ways and the round-trip time
// measured by the transport's Ping/Pong probe (state_transport.proto). Every connection of
// state_transport_ws and state_transport_grpc owns one, so a client's link to its relay, a relay's
// link to each client, and a server's links to other servers (delegation peers included) are all
// measured the same way. Counters are lock-free: the reader thread notes inbound bytes, any writer
// notes outbound bytes, and the pinger notes round trips concurrently.
//
// RTT: rtt_ms is an exponentially weighted moving average (alpha 1/4) of the measured round trips,
// so one slow probe does not swing it; last and min are kept beside it. All negative until the
// first Pong.
class state_link_stats {
public:
  void note_rx(std::size_t bytes) noexcept;
  void note_tx(std::size_t bytes) noexcept;
  // The next Ping's sequence number (and counts the Ping as sent).
  std::uint64_t next_ping_seq() noexcept;
  // A Pong arrived: rtt_ns is now minus the echoed sent_ns, on the sender's own clock.
  void note_rtt(std::int64_t rtt_ns) noexcept;

  struct counters {
    std::uint64_t bytes_in = 0, bytes_out = 0;
    std::uint64_t frames_in = 0, frames_out = 0;
    std::uint64_t pings_sent = 0, pongs_received = 0;
    double rtt_ms = -1.0, rtt_last_ms = -1.0, rtt_min_ms = -1.0;
  };
  counters read() const noexcept;

private:
  std::atomic<std::uint64_t> _bytes_in{0}, _bytes_out{0};
  std::atomic<std::uint64_t> _frames_in{0}, _frames_out{0};
  std::atomic<std::uint64_t> _pings{0}, _pongs{0};
  // RTTs in microseconds (atomic integers; -1 = none yet).
  std::atomic<std::int64_t> _rtt_ewma_us{-1}, _rtt_last_us{-1}, _rtt_min_us{-1};
};

// One link as a transport reports it (state_transport::link_reports()).
struct state_link_report {
  std::string transport;      // "ws" | "grpc"
  std::string remote_node_id; // the peer's node id (from its Hello); "" before the Hello
  bool accepted = false;      // this end ACCEPTED the connection (a relay's link to a client)
  double age_s = 0.0;         // since the connection opened
  double idle_s = 0.0;        // since the last inbound frame
  state_link_stats::counters stats;
};

// Turns successive cumulative link reports into rates. update() is called periodically (say once a
// second) with the transports' current reports; it returns each link's bytes-per-second since the
// previous call, keyed like key_of(). A link seen for the first time reports 0 (no interval yet).
class state_link_rate_meter {
public:
  struct rate {
    double in_bps = 0.0, out_bps = 0.0;
  };
  std::map<std::string, rate> update(const std::vector<state_link_report> &reports, double now_s);
  // "<transport>:<remote node id>" (+ ":in" for an accepted link), unique per live link.
  static std::string key_of(const state_link_report &r);

private:
  struct sample {
    std::uint64_t bytes_in = 0, bytes_out = 0;
    double t = 0.0;
  };
  std::map<std::string, sample> _last;
};

} // namespace cvc

#endif // __CVC_STATE_LINK_STATS_H__
