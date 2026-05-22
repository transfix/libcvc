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

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
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
  std::printf("BENCH %s N=%zu wall_ns_total=%llu wall_ns_per_op=%llu\n", name, n,
              static_cast<unsigned long long>(wall_ns), static_cast<unsigned long long>(per));
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
    ms.push_back(
        make_set_value("nodeB", static_cast<std::uint64_t>(i + 1), "p." + std::to_string(i), "v"));
  }

  auto t0 = std::chrono::steady_clock::now();
  std::size_t applied = 0;
  for (auto &m : ms) {
    if (sh.ingest_remote(m).applied)
      ++applied;
  }
  auto t1 = std::chrono::steady_clock::now();
  EXPECT_EQ(applied, N);
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
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
    sh.ingest_remote(make_set_value("nodeB", static_cast<std::uint64_t>(i + 1), "p.x", "v"));
  }

  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    // Re-ingest the same (origin, lc): dedup hot path.
    sh.ingest_remote(make_set_value("nodeB", static_cast<std::uint64_t>(i + 1), "p.x", "v"));
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
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
    m.message_id = "msg_" + std::to_string(i);
    m.path = "p." + std::to_string(i);
    m.content_type = "text/plain";
    m.string_value = "v";
    msgs.push_back(std::move(m));
  }

  auto t0 = std::chrono::steady_clock::now();
  for (auto &m : msgs)
    (void)bus.admit(m);
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_EQ(bus.total_admitted(), N);
  report("message_bus_admit", N, wall_ns);
}

// ---------------------------------------------------------------------
// Large-tree routing benchmarks + correctness (Phase 6 bullet 5)
// ---------------------------------------------------------------------

#include <atomic>
#include <cvc/state_peer_registry.h>
#include <cvc/state_transport_inproc.h>
#include <memory>
#include <random>
#include <vector>

namespace {

std::vector<std::string> generate_paths(std::size_t n, std::uint32_t seed = 1) {
  // Deep paths with limited branching so prefix matching has work
  // to do but the same prefixes recur across many leaves.
  static const char *roots[] = {"vol", "geom", "mesh", "cam", "scene", "ui", "fx", "net"};
  static const char *mid[] = {"a", "b", "c", "d", "e"};
  std::mt19937 rng(seed);
  std::vector<std::string> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    std::string p = roots[rng() % 8];
    p += '.';
    p += mid[rng() % 5];
    p += '.';
    p += "node";
    p += std::to_string(rng() % 1024);
    p += '.';
    p += "leaf";
    p += std::to_string(i);
    out.push_back(std::move(p));
  }
  return out;
}

std::string node_name(std::size_t i) { return "n" + std::to_string(i); }

std::vector<std::string> peer_subscription(std::size_t i) {
  // Half the peers subscribe to a narrow root prefix; the other
  // half are match-all. This mirrors a realistic pane / panel
  // layout where each viewer cares about one subtree.
  static const char *roots[] = {"vol", "geom", "mesh", "cam", "scene", "ui", "fx", "net"};
  if (i % 2 == 0)
    return {std::string(roots[i % 8])};
  return {}; // match-all
}

} // namespace

TEST(StateDistributedBench, AnyPrefixMatchesScales) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  std::vector<std::string> prefixes = {"vol",     "geom.a", "mesh.b.c", "cam.d.node5",
                                       "scene.e", "ui.a",   "fx.b",     "net.c.d.e"};
  auto paths = generate_paths(20000, 7);

  auto t0 = std::chrono::steady_clock::now();
  std::size_t hits = 0;
  for (const auto &p : paths) {
    if (cvc::state_peer_registry::any_prefix_matches(prefixes, p))
      ++hits;
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  // Force `hits` to stay live for the optimizer.
  EXPECT_LE(hits, paths.size());
  report("any_prefix_matches_8x20k", paths.size(), wall_ns);
}

TEST(StateDistributedBench, RegistryShouldDeliverScales) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::state_peer_registry reg;
  constexpr std::size_t kPeers = 256;
  for (std::size_t i = 0; i < kPeers; ++i) {
    reg.add_peer(node_name(i), "cA", std::string(), peer_subscription(i));
  }
  auto paths = generate_paths(2000, 11);

  auto t0 = std::chrono::steady_clock::now();
  std::size_t deliveries = 0;
  for (const auto &p : paths) {
    for (std::size_t i = 0; i < kPeers; ++i) {
      if (reg.should_deliver(node_name(i), p))
        ++deliveries;
    }
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_GT(deliveries, 0u);
  // Per-call cost in ns:
  report("should_deliver_256peers_2000paths", paths.size() * kPeers, wall_ns);
}

TEST(StateDistributedBench, PublishMessageFanout) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  constexpr std::size_t kPeers = 64;
  cvc::state_transport_inproc t;
  std::vector<std::unique_ptr<cvc::app>> apps;
  std::vector<std::unique_ptr<cvc::state_cluster_shard>> shards;
  apps.reserve(kPeers);
  shards.reserve(kPeers);
  for (std::size_t i = 0; i < kPeers; ++i) {
    apps.push_back(std::make_unique<cvc::app>());
    shards.push_back(std::make_unique<cvc::state_cluster_shard>(*apps.back(), "cA", node_name(i)));
    shards.back()->attach();
    t.register_shard(shards.back().get());
    t.peers().add_peer(node_name(i), "cA", std::string(), peer_subscription(i));
  }

  auto paths = generate_paths(2000, 19);
  auto t0 = std::chrono::steady_clock::now();
  std::uint64_t pub = 0;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    cvc::state_message m;
    m.cluster_id = "cA";
    m.origin_node_id = node_name(0);
    m.message_id = "m" + std::to_string(i);
    m.path = paths[i];
    m.content_type = cvc::state_message::MIME_TEXT;
    m.string_value = "v";
    auto s = t.publish_message(m);
    pub += s.delivered;
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_GT(pub, 0u);
  report("publish_message_fanout_64peers_2000msgs", paths.size(), wall_ns);
}

TEST(StateDistributedBench, PublishMutationFanout) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  constexpr std::size_t kPeers = 64;
  cvc::state_transport_inproc t;
  std::vector<std::unique_ptr<cvc::app>> apps;
  std::vector<std::unique_ptr<cvc::state_cluster_shard>> shards;
  apps.reserve(kPeers);
  shards.reserve(kPeers);
  for (std::size_t i = 0; i < kPeers; ++i) {
    apps.push_back(std::make_unique<cvc::app>());
    shards.push_back(std::make_unique<cvc::state_cluster_shard>(*apps.back(), "cA", node_name(i)));
    shards.back()->attach();
    t.register_shard(shards.back().get());
    t.peers().add_peer(node_name(i), "cA", std::string(), peer_subscription(i));
  }

  auto paths = generate_paths(1000, 23);
  auto t0 = std::chrono::steady_clock::now();
  std::uint64_t delivered = 0;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    auto s =
        t.publish(make_set_value(node_name(0), static_cast<std::uint64_t>(i + 1), paths[i], "v"));
    delivered += s.delivered;
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_GT(delivered, 0u);
  report("publish_mutation_fanout_64peers_1000muts", paths.size(), wall_ns);
}

// ---- Always-on correctness companions (run in normal CI) -------------

TEST(StateDistributedRoutingScale, RegistryFiltersAtScale) {
  // Verify subscription routing returns the right set of peers for
  // a non-trivial mix (256 peers, 8 root-prefix subscribers each
  // covering 1/8 of paths, plus match-all peers). Without this,
  // the benchmark could silently degrade into "delivers to nobody".
  cvc::state_peer_registry reg;
  constexpr std::size_t kPeers = 256;
  std::size_t match_all = 0;
  for (std::size_t i = 0; i < kPeers; ++i) {
    auto subs = peer_subscription(i);
    if (subs.empty())
      ++match_all;
    reg.add_peer(node_name(i), "cA", std::string(), subs);
  }
  // For path "vol.x.y": every match-all peer + every "vol"-rooted
  // peer should pass; nothing else.
  std::size_t pass = 0;
  for (std::size_t i = 0; i < kPeers; ++i) {
    if (reg.should_deliver(node_name(i), "vol.x.y"))
      ++pass;
  }
  // 256/8 = 32 peers per root, but only the i%2==0 half subscribe;
  // those with root index i%8==0 and i%2==0 → indices {0,8,16,...248} = 32.
  // Plus all match-all peers (kPeers/2 = 128).
  std::size_t narrow = 0;
  for (std::size_t i = 0; i < kPeers; ++i) {
    auto subs = peer_subscription(i);
    if (!subs.empty() && subs[0] == "vol")
      ++narrow;
  }
  EXPECT_EQ(pass, narrow + match_all);
  EXPECT_GT(narrow, 0u);
  EXPECT_EQ(match_all, kPeers / 2);
}

TEST(StateDistributedRoutingScale, TransportSkipsFilteredPeers) {
  // End-to-end: with 16 peers (mix of narrow + match-all), a
  // publish_message to "vol.x" must land on exactly the narrow-vol
  // peers + the match-all peers, and nobody else. This locks the
  // transport+registry contract that the routing benchmarks rely on.
  constexpr std::size_t kPeers = 16;
  cvc::state_transport_inproc t;
  std::vector<std::unique_ptr<cvc::app>> apps;
  std::vector<std::unique_ptr<cvc::state_cluster_shard>> shards;
  apps.reserve(kPeers);
  shards.reserve(kPeers);
  std::vector<std::atomic<int>> hits(kPeers);
  for (auto &h : hits)
    h.store(0);

  for (std::size_t i = 0; i < kPeers; ++i) {
    apps.push_back(std::make_unique<cvc::app>());
    shards.push_back(std::make_unique<cvc::state_cluster_shard>(*apps.back(), "cA", node_name(i)));
    shards.back()->attach();
    t.register_shard(shards.back().get());
    t.peers().add_peer(node_name(i), "cA", std::string(), peer_subscription(i));
    auto *hp = &hits[i];
    shards.back()->message_bus().subscribe("",
                                           [hp](const cvc::state_message &) { hp->fetch_add(1); });
  }

  cvc::state_message m;
  m.cluster_id = "cA";
  m.origin_node_id = node_name(0);
  m.message_id = "m1";
  m.path = "vol.x";
  m.content_type = cvc::state_message::MIME_TEXT;
  m.string_value = "v";
  t.publish_message(m);

  std::size_t expected_hits = 0;
  for (std::size_t i = 1; i < kPeers; ++i) { // skip origin
    auto subs = peer_subscription(i);
    if (subs.empty() || subs[0] == "vol")
      ++expected_hits;
  }
  std::size_t actual_hits = 0;
  for (std::size_t i = 0; i < kPeers; ++i) {
    if (i == 0) {
      EXPECT_EQ(hits[i].load(), 0); // origin never hears its own
      continue;
    }
    auto subs = peer_subscription(i);
    bool should = subs.empty() || subs[0] == "vol";
    EXPECT_EQ(hits[i].load(), should ? 1 : 0)
        << "peer " << i << " subs=" << (subs.empty() ? std::string("*") : subs[0]);
    actual_hits += static_cast<std::size_t>(hits[i].load());
  }
  EXPECT_EQ(actual_hits, expected_hits);
}

// ---- Phase 7: Production benchmarks ----

#include <cvc/state_blob_store.h>
#include <cvc/state_delta_codec.h>
#include <cvc/state_distributed_admin.h>

TEST(StateDistributedBench, BlobStorePutGet) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::memory_state_blob_store store;
  constexpr std::size_t N = 5000;
  constexpr std::size_t BLOB_SIZE = 4096;

  std::vector<std::vector<unsigned char>> blobs(N);
  std::vector<std::string> digests(N);
  for (std::size_t i = 0; i < N; ++i) {
    blobs[i].resize(BLOB_SIZE);
    for (std::size_t j = 0; j < BLOB_SIZE; ++j)
      blobs[i][j] = static_cast<unsigned char>((i * 7 + j * 3) & 0xFF);
  }

  // PUT
  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    auto ref = store.put(blobs[i]);
    digests[i] = ref.digest;
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t put_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("blob_store_put_5000x4KB", N, put_ns);

  // GET
  t0 = std::chrono::steady_clock::now();
  std::vector<unsigned char> out;
  for (std::size_t i = 0; i < N; ++i) {
    ASSERT_TRUE(store.get(digests[i], out));
  }
  t1 = std::chrono::steady_clock::now();
  std::uint64_t get_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("blob_store_get_5000x4KB", N, get_ns);
  EXPECT_EQ(store.size(), N);
}

TEST(StateDistributedBench, CallbackLatency) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  sh.attach();

  constexpr std::size_t N = 10000;
  std::uint64_t cb_count = 0;
  a.root()("bench_cb").valueChanged.connect([&]() { ++cb_count; });

  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    sh.ingest_remote(make_set_value("nodeB", static_cast<std::uint64_t>(i + 1), "bench_cb",
                                    "v" + std::to_string(i)));
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_EQ(cb_count, N);
  report("callback_latency_10000", N, wall_ns);
}

TEST(StateDistributedBench, AdminSnapshotOverhead) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  sh.attach();
  cvc::state_peer_registry reg;
  cvc::state_message_bus bus;
  cvc::memory_state_blob_store blobs;

  for (std::size_t i = 0; i < 128; ++i)
    reg.add_peer(node_name(i), "cA");

  cvc::state_distributed_admin admin;
  admin.attach_shard(&sh);
  admin.attach_peer_registry(&reg);
  admin.attach_message_bus(&bus);
  admin.attach_blob_store(&blobs);

  constexpr std::size_t N = 5000;
  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    auto snap = admin.snapshot();
    auto text = admin.to_text(snap);
    ASSERT_FALSE(text.empty());
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("admin_snapshot_128peers", N, wall_ns);
}

TEST(StateDistributedBench, SlowPeerIsolation) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  constexpr std::size_t kPeers = 32;
  cvc::state_transport_inproc t;
  std::vector<std::unique_ptr<cvc::app>> apps;
  std::vector<std::unique_ptr<cvc::state_cluster_shard>> shards;
  apps.reserve(kPeers);
  shards.reserve(kPeers);
  for (std::size_t i = 0; i < kPeers; ++i) {
    apps.push_back(std::make_unique<cvc::app>());
    shards.push_back(std::make_unique<cvc::state_cluster_shard>(*apps.back(), "cA", node_name(i)));
    shards.back()->attach();
    t.register_shard(shards.back().get());
    t.peers().add_peer(node_name(i), "cA");
  }

  // Mark half the peers slow.
  for (std::size_t i = 0; i < kPeers / 2; ++i)
    t.mark_peer_slow(shards[i].get());

  auto paths = generate_paths(2000, 31);
  auto t0 = std::chrono::steady_clock::now();
  std::uint64_t delivered = 0;
  for (std::size_t i = 0; i < paths.size(); ++i) {
    auto s = t.publish(
        make_set_value(node_name(kPeers - 1), static_cast<std::uint64_t>(i + 1), paths[i], "v"));
    delivered += s.delivered;
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  EXPECT_GT(delivered, 0u);
  EXPECT_GT(t.total_quarantined_mutations(), 0u);
  report("slow_peer_isolation_32peers_half_slow", paths.size(), wall_ns);
}

TEST(StateDistributedBench, DeltaCodecThroughput) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }
  cvc::state_delta_codec encoder;
  cvc::state_delta_codec decoder;

  constexpr std::size_t kBlobSize = 65536; // 64 KB volume slice
  constexpr std::size_t N = 1000;

  // Build a series of "volume slices" where each iteration changes
  // a small number of voxels.
  std::vector<unsigned char> current(kBlobSize, 0);
  std::mt19937 rng(42);

  auto t0 = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < N; ++i) {
    // Mutate 16 random bytes.
    for (int j = 0; j < 16; ++j)
      current[rng() % kBlobSize] = static_cast<unsigned char>(rng() & 0xFF);

    auto encoded = encoder.encode("vol.slice", current);
    std::vector<unsigned char> decoded;
    bool ok = decoder.decode("vol.slice", encoded, decoded);
    ASSERT_TRUE(ok);
    ASSERT_EQ(decoded, current);
  }
  auto t1 = std::chrono::steady_clock::now();
  std::uint64_t wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("delta_codec_64KB_1000iters", N, wall_ns);
}
