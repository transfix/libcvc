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

// cvc::state_link_stats / state_link_rate_meter: the per-connection counters and round-trip math
// behind the transports' link reports, without a network.

#include <cvc/state/state_link_stats.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::state_link_rate_meter;
using cvc::state_link_report;
using cvc::state_link_stats;

TEST(StateLinkStatsTest, CountsTrafficBothWays) {
  state_link_stats s;
  s.note_rx(100);
  s.note_rx(50);
  s.note_tx(7);
  const auto c = s.read();
  EXPECT_EQ(c.bytes_in, 150u);
  EXPECT_EQ(c.frames_in, 2u);
  EXPECT_EQ(c.bytes_out, 7u);
  EXPECT_EQ(c.frames_out, 1u);
}

TEST(StateLinkStatsTest, RoundTripMedianMinAndLast) {
  state_link_stats s;
  EXPECT_LT(s.read().rtt_ms, 0.0) << "no measurement before the first Pong";
  EXPECT_EQ(s.next_ping_seq(), 1u);
  EXPECT_EQ(s.next_ping_seq(), 2u);
  s.note_rtt(40'000'000); // 40 ms
  s.note_rtt(80'000'000); // 80 ms: median of two = their mean, 60 ms
  s.note_rtt(-5);         // a clock that went backwards is ignored
  auto c = s.read();
  EXPECT_EQ(c.pings_sent, 2u);
  EXPECT_EQ(c.pongs_received, 2u);
  EXPECT_DOUBLE_EQ(c.rtt_ms, 60.0);
  EXPECT_DOUBLE_EQ(c.rtt_last_ms, 80.0);
  EXPECT_DOUBLE_EQ(c.rtt_min_ms, 40.0);
}

// One stall (a probe answered only after a frozen tab woke) barely moves the median, and leaves the
// window entirely after kRttWindow more samples.
TEST(StateLinkStatsTest, MedianShrugsOffAStall) {
  state_link_stats s;
  for (int i = 0; i < 4; ++i)
    s.note_rtt(50'000'000);
  s.note_rtt(14'800'000'000); // a 14.8 s stall
  for (int i = 0; i < 3; ++i)
    s.note_rtt(50'000'000);
  EXPECT_DOUBLE_EQ(s.read().rtt_ms, 50.0) << "7 of 8 recent samples are 50 ms";
  EXPECT_DOUBLE_EQ(s.read().rtt_last_ms, 50.0);
}

TEST(StateLinkStatsTest, RateMeterTurnsCountersIntoRates) {
  state_link_rate_meter m;
  state_link_report r;
  r.transport = "ws";
  r.remote_node_id = "alice";
  r.accepted = true;
  r.stats.bytes_in = 1000;
  r.stats.bytes_out = 4000;
  const std::string key = state_link_rate_meter::key_of(r);
  EXPECT_EQ(key, "ws:alice:in");

  auto first = m.update({r}, 10.0);
  EXPECT_DOUBLE_EQ(first[key].in_bps, 0.0) << "no interval yet";

  r.stats.bytes_in = 3000;  // +2000 in 2 s
  r.stats.bytes_out = 4400; // +400 in 2 s
  auto second = m.update({r}, 12.0);
  EXPECT_DOUBLE_EQ(second[key].in_bps, 1000.0);
  EXPECT_DOUBLE_EQ(second[key].out_bps, 200.0);

  // A reconnect under the same key restarts its counters: no negative or bogus rate.
  r.stats.bytes_in = 10;
  r.stats.bytes_out = 10;
  auto third = m.update({r}, 13.0);
  EXPECT_DOUBLE_EQ(third[key].in_bps, 0.0);
  EXPECT_DOUBLE_EQ(third[key].out_bps, 0.0);

  // A link that is no longer reported is forgotten.
  auto gone = m.update({}, 14.0);
  EXPECT_TRUE(gone.empty());
}
