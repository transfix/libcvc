/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 6 micro-benchmark for the distributed-state ingest path.
// Gated on env var CVC_DISTRIBUTED_STATE_BENCH=1 so it runs only on
// demand. When the env var is not set, every test is a SUCCEED no-op
// so the suite stays green in normal CI runs. When enabled, results
// are printed to stdout in a stable parseable format:
//
//   BENCH <name> N=<count> wall_ns_total=<n> wall_ns_per_op=<n>
//
// The harness measures wall time only; nothing here makes
// correctness assertions about throughput, because absolute numbers
// vary by host. The intent is to:
//   - exercise the hot path (ingest_remote, journal append, codec
//     registry lookups, message bus dedup) under repeated load;
//   - produce numbers a human or a CI dashboard can compare against
//     prior runs;
//   - catch order-of-magnitude regressions when run manually.

#include <cvc/app.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

bool bench_enabled() {
  const char *v = std::getenv("CVC_DISTRIBUTED_STATE_BENCH");
  return v && std::string(v) == "1";
}

cvc::state_mutation make_set_value(const std::string &origin,
                                   std::uint64_t seq, const std::string &path,
                                   const std::string &val) {
  cvc::state_mutation m;
  m.cluster_id = "cA";
  m.tree_id = "default";
  m.origin_node_id = origin;
  m.sequence = seq;
  m.mutation_id = origin + ":" + std::to_string(seq);
  m.path = path;
  m.op = cvc::state_mutation_op::set_value;
  m.type_name = "std::string";
  m.string_value = val;
  m.latest_value_only = true;
  return m;
}

void report(const char *name, std::size_t n, std::uint64_t wall_ns) {
  std::uint64_t per = (n == 0) ? 0u : wall_ns / n;
  std::printf("BENCH %s N=%zu wall_ns_total=%llu wall_ns_per_op=%llu\n", name,
              n,
              static_cast<unsigned long long>(wall_ns),
              static_cast<unsigned long long>(per));
}

} // namespace

TEST(StateDistributedBench, IngestRemoteUniquePaths) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  sh.attach();

  constexpr std::size_t N = 20000;
  // Pre-build mutations so the loop measures only the hot path.
  std::vector<cvc::state_mutation> ms;
  ms.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    ms.push_back(make_set_value("nodeB", static_cast<std::uint64_t>(i + 1),
                                "p." + std::to_string(i), "v"));
  }

  auto t0 = std::chrono::steady_clock::now();
  std::size_t applied = 0;
  for (auto &m : ms) {
    if (sh.ingest_remote(m).applied)
      ++applied;
  }
  auto t1 = std::chrono::steady_clock::now();
  EXPECT_EQ(applied, N);
  std::uint64_t wall_ns =
      static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
              .count());
  report("ingest_remote_unique_paths", N, wall_ns);
}

TEST(StateDistributedBench, IngestRemoteDuplicateDedup) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  sh.attach();

  // First wave: prime the dedup state.
  constexpr std::size_t N = 10000;
  for (std::size_t i = 0; i < N; ++i) {
    sh.ingest_remote(make_set_value(
        "nodeB", static_cast<std::uint64_t>(i + 1), "p.x", "v"));
  }

  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    // Re-ingest the same (origin, lc): dedup hot path.
    sh.ingest_remote(make_set_value(
        "nodeB", static_cast<std::uint64_t>(i + 1), "p.x", "v"));
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns =
      static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
              .count());
  EXPECT_GE(sh.total_remote_duplicates(), N);
  report("ingest_remote_dedup", N, wall_ns);
}

TEST(StateDistributedBench, MessageBusAdmit) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::state_message_bus bus;

  constexpr std::size_t N = 20000;
  std::vector<cvc::state_message> msgs;
  msgs.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    cvc::state_message m;
    m.cluster_id = "cA";
    m.origin_node_id = "nodeB";
    m.message_id = "msg-" + std::to_string(i);
    m.path = "p." + std::to_string(i);
    m.content_type = "text/plain";
    m.string_value = "v";
    msgs.push_back(std::move(m));
  }

  auto t0 = std::chrono::steady_clock::now();
  for (auto &m : msgs)
    (void)bus.admit(m);
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns =
      static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
              .count());
  EXPECT_EQ(bus.total_admitted(), N);
  report("message_bus_admit", N, wall_ns);
}
