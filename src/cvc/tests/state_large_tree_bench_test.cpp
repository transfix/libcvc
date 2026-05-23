/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Large-tree benchmark: measures ingest throughput and snapshot
// performance with trees of 10k–100k nodes. Gated on env var
// CVC_DISTRIBUTED_STATE_BENCH=1 so it runs only on demand.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>

namespace {

bool bench_enabled() {
  const char *v = std::getenv("CVC_DISTRIBUTED_STATE_BENCH");
  return v && std::string(v) == "1";
}

cvc::state_mutation make_set_value(const std::string &origin, std::uint64_t seq,
                                   const std::string &path, const std::string &val) {
  cvc::state_mutation m;
  m.cluster_id = "bench";
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

} // namespace

TEST(LargeTreeBench, Ingest10kNodes) {
  if (!bench_enabled()) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_BENCH=1 to enable";
  }
  cvc::app ctx;
  cvc::state_cluster_shard shard(ctx, "bench", "local");
  shard.attach();

  constexpr int N = 10000;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i) {
    auto m = make_set_value("remote", static_cast<std::uint64_t>(i + 1),
                            "tree.node_" + std::to_string(i), "value_" + std::to_string(i));
    shard.ingest_remote(m);
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  std::printf("BENCH Ingest10kNodes N=%d wall_ns_total=%lld wall_ns_per_op=%lld\n", N,
              static_cast<long long>(ns), static_cast<long long>(ns / N));
  SUCCEED();
}

TEST(LargeTreeBench, Ingest100kNodes) {
  if (!bench_enabled()) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_BENCH=1 to enable";
  }
  cvc::app ctx;
  cvc::state_cluster_shard shard(ctx, "bench", "local");
  shard.attach();

  constexpr int N = 100000;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i) {
    auto m = make_set_value("remote", static_cast<std::uint64_t>(i + 1),
                            "tree.node_" + std::to_string(i), "value_" + std::to_string(i));
    shard.ingest_remote(m);
  }
  auto t1 = std::chrono::steady_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  std::printf("BENCH Ingest100kNodes N=%d wall_ns_total=%lld wall_ns_per_op=%lld\n", N,
              static_cast<long long>(ns), static_cast<long long>(ns / N));
  SUCCEED();
}

TEST(LargeTreeBench, DrainAndPublish10k) {
  if (!bench_enabled()) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_BENCH=1 to enable";
  }
  cvc::app ctx;
  cvc::state_transport_inproc transport;
  cvc::state_cluster_shard shard(ctx, "bench", "local");
  shard.attach();
  transport.register_shard(&shard);
  shard.set_transport(&transport);

  // Populate locally.
  constexpr int N = 10000;
  for (int i = 0; i < N; ++i) {
    cvc::state::instance(ctx)("tree.node_" + std::to_string(i))
        .value(std::string("val_" + std::to_string(i)));
  }

  auto t0 = std::chrono::steady_clock::now();
  std::size_t pumped = transport.pump_all();
  auto t1 = std::chrono::steady_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  std::printf("BENCH DrainAndPublish10k pumped=%zu wall_ns_total=%lld wall_ns_per_op=%lld\n",
              pumped, static_cast<long long>(ns),
              pumped > 0 ? static_cast<long long>(ns / static_cast<long long>(pumped)) : 0LL);
  transport.unregister_shard(&shard);
  SUCCEED();
}

TEST(LargeTreeBench, Snapshot10kNodes) {
  if (!bench_enabled()) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_BENCH=1 to enable";
  }
  cvc::app ctx;
  cvc::state_cluster_shard shard(ctx, "bench", "local");
  shard.attach();

  constexpr int N = 10000;
  for (int i = 0; i < N; ++i) {
    auto m = make_set_value("remote", static_cast<std::uint64_t>(i + 1),
                            "tree.node_" + std::to_string(i), "value_" + std::to_string(i));
    shard.ingest_remote(m);
  }

  auto t0 = std::chrono::steady_clock::now();
  auto snap = shard.snapshot("tree");
  auto t1 = std::chrono::steady_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  std::printf("BENCH Snapshot10kNodes entries=%zu wall_ns_total=%lld\n", snap.size(),
              static_cast<long long>(ns));
  EXPECT_GE(snap.size(), 1u);
  SUCCEED();
}
