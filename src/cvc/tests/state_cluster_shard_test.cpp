/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

cvc::state_mutation make_set_value(const std::string &origin, std::uint64_t seq,
                                   const std::string &path, const std::string &val) {
  cvc::state_mutation m;
  m.cluster_id = "testCluster";
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

TEST(StateClusterShardTest, ConstructAndAccessors) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  EXPECT_EQ(sh.cluster_id(), "clusterA");
  EXPECT_EQ(sh.local_node_id(), "nodeA");
  EXPECT_EQ(sh.root_path(), "");
  EXPECT_FALSE(sh.is_attached());
  EXPECT_EQ(sh.published_cursor(), 0u);
  EXPECT_FALSE(sh.enforce_authority());
  // Builtin codecs registered.
  EXPECT_TRUE(sh.codecs().has("std::string"));
  EXPECT_TRUE(sh.codecs().has("int32_t"));
}

TEST(StateClusterShardTest, AttachDetachIdempotent) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  EXPECT_TRUE(sh.is_attached());
  sh.attach(); // idempotent
  EXPECT_TRUE(sh.is_attached());
  sh.detach();
  EXPECT_FALSE(sh.is_attached());
  sh.detach(); // idempotent
}

TEST(StateClusterShardTest, LocalChangesAreJournaled) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  cvc::state::instance(a)("sh.local.path").value(std::string("hello"));
  cvc::state::instance(a)("sh.local.path").value(std::string("world"));
  // Drain. First value() on a freshly-created child is lost (see
  // adapter doc), so only the second set is journaled.
  auto drained = sh.drain_local();
  ASSERT_GE(drained.size(), 1u);
  EXPECT_EQ(drained.back().origin_node_id, "nodeA");
  EXPECT_EQ(drained.back().path, "sh.local.path");
  EXPECT_EQ(drained.back().string_value, "world");
  EXPECT_GT(sh.published_cursor(), 0u);

  // Draining again returns nothing.
  auto drained2 = sh.drain_local();
  EXPECT_TRUE(drained2.empty());
}

TEST(StateClusterShardTest, IngestRemoteAppliesAndDeduplicates) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  auto m = make_set_value("nodeB", 1, "sh.remote.a", "rval");
  auto r1 = sh.ingest_remote(m);
  EXPECT_TRUE(r1.applied);
  EXPECT_FALSE(r1.duplicate);
  EXPECT_EQ(cvc::state::instance(a)("sh.remote.a").value(), "rval");

  // Replay same mutation -> duplicate.
  auto r2 = sh.ingest_remote(m);
  EXPECT_FALSE(r2.applied);
  EXPECT_TRUE(r2.duplicate);

  // Replica metadata.
  EXPECT_EQ(sh.replica().last_applied("nodeB"), 1u);
  EXPECT_EQ(sh.replica().clock_component("nodeB"), 1u);
}

TEST(StateClusterShardTest, IngestRemoteDoesNotLoopBackToJournal) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  // Seed a local mutation so the publish cursor moves.
  cvc::state::instance(a)("sh.x").value(std::string("seed"));
  cvc::state::instance(a)("sh.x").value(std::string("seed2"));
  auto local1 = sh.drain_local();
  EXPECT_FALSE(local1.empty());

  // Now apply a remote mutation; it must not appear in drain_local.
  auto m = make_set_value("nodeB", 5, "sh.x", "from-remote");
  auto r = sh.ingest_remote(m);
  EXPECT_TRUE(r.applied);
  auto local2 = sh.drain_local();
  for (const auto &mm : local2) {
    EXPECT_NE(mm.origin_node_id, "nodeB");
  }
}

TEST(StateClusterShardTest, AuthorityEnforcementRejectsForeignWrites) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.authority().delegate("owned", "clusterB");
  sh.set_enforce_authority(true);
  EXPECT_TRUE(sh.enforce_authority());

  auto m = make_set_value("nodeB", 1, "owned.leaf", "no");
  auto r = sh.ingest_remote(m);
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(r.reject_reason.empty());

  // A path not covered by any delegation is allowed through.
  auto m2 = make_set_value("nodeB", 2, "free.path", "yes");
  auto r2 = sh.ingest_remote(m2);
  EXPECT_TRUE(r2.applied);
}

TEST(StateClusterShardTest, AuthorityOwnedByThisClusterAccepted) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.authority().delegate("here", "clusterA");
  sh.set_enforce_authority(true);
  auto m = make_set_value("nodeB", 1, "here.leaf", "ok");
  auto r = sh.ingest_remote(m);
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
}

TEST(StateClusterShardTest, RewindPublishCursorReemits) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  cvc::state::instance(a)("rw.path").value(std::string("a"));
  cvc::state::instance(a)("rw.path").value(std::string("b"));
  cvc::state::instance(a)("rw.path").value(std::string("c"));
  auto d1 = sh.drain_local();
  ASSERT_FALSE(d1.empty());
  EXPECT_TRUE(sh.drain_local().empty());

  sh.rewind_publish_cursor(0);
  auto d2 = sh.drain_local();
  EXPECT_EQ(d1.size(), d2.size());
}

TEST(StateClusterShardTest, IngestRemoteScopedToRootPath) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  auto m = make_set_value("nodeB", 1, "scoped.deep.leaf", "v");
  auto r = sh.ingest_remote(m);
  EXPECT_TRUE(r.applied);
  EXPECT_EQ(cvc::state::instance(a)("scoped.deep.leaf").value(), "v");
}

TEST(StateClusterShardTest, DetachStopsLocalJournaling) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  cvc::state::instance(a)("det.path").value(std::string("a"));
  cvc::state::instance(a)("det.path").value(std::string("b"));
  auto d = sh.drain_local();
  EXPECT_FALSE(d.empty());
  std::uint64_t cursor = sh.published_cursor();

  sh.detach();
  cvc::state::instance(a)("det.path").value(std::string("c"));
  cvc::state::instance(a)("det.path").value(std::string("d"));
  auto d2 = sh.drain_local();
  EXPECT_TRUE(d2.empty());
  EXPECT_EQ(sh.published_cursor(), cursor);
}

TEST(StateClusterShardStressTest, OptionalConcurrentIngestStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run shard "
                    "stress tests";
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  const int kThreads = 4;
  const int kPerThread = 2000;
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      std::string origin = "peer" + std::to_string(t);
      for (int i = 0; i < kPerThread; ++i) {
        auto m = make_set_value(origin, static_cast<std::uint64_t>(i + 1),
                                "ns." + origin + ".k" + std::to_string(i), "v" + std::to_string(i));
        sh.ingest_remote(m);
        // And ingest the same mutation again to exercise dedup.
        sh.ingest_remote(m);
      }
    });
  }
  for (auto &th : threads)
    th.join();

  for (int t = 0; t < kThreads; ++t) {
    std::string origin = "peer" + std::to_string(t);
    EXPECT_EQ(sh.replica().last_applied(origin), static_cast<std::uint64_t>(kPerThread));
  }
}

TEST(StateClusterShardPerformanceTest, OptionalIngestThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run shard "
                    "performance smoke tests";
  }
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  const int kIters = 5000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i) {
    auto m = make_set_value("peerX", static_cast<std::uint64_t>(i + 1),
                            "perf.k" + std::to_string(i % 64), "v" + std::to_string(i));
    sh.ingest_remote(m);
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  std::cerr << "[cluster_shard perf] " << kIters << " ingest in " << secs << "s ("
            << (kIters / secs) << "/s)\n";
  EXPECT_LT(secs, 30.0);
}

// ---- Phase 5 tests ----

TEST(StateClusterShardTest, WritePolicyDeniesUnauthorizedOrigin) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.write_policy().allow("locked", {"trusted"});
  sh.set_enforce_write_policy(true);

  auto m_ok = make_set_value("trusted", 1, "locked.x", "v");
  auto r_ok = sh.ingest_remote(m_ok);
  EXPECT_TRUE(r_ok.applied);

  auto m_deny = make_set_value("intruder", 2, "locked.y", "v");
  auto r_deny = sh.ingest_remote(m_deny);
  EXPECT_FALSE(r_deny.applied);
  EXPECT_TRUE(r_deny.rejected);
  EXPECT_FALSE(r_deny.reject_reason.empty());
  EXPECT_EQ(1u, sh.total_remote_rejected());
}

TEST(StateClusterShardTest, WritePolicyAllowsUncoveredPaths) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.write_policy().allow("only.this", {"trusted"});
  sh.set_enforce_write_policy(true);

  auto m = make_set_value("anyone", 1, "elsewhere.k", "v");
  auto r = sh.ingest_remote(m);
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
}

TEST(StateClusterShardTest, ConflictResolutionDropsLoser) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_resolve_conflicts(true);

  // Both mutations write the same path with the same sequence
  // (concurrent). The deterministic winner is the greater
  // (origin_node_id, sequence) pair.
  auto m_low = make_set_value("aaa", 1, "k", "low");
  auto m_high = make_set_value("zzz", 1, "k", "high");

  // Apply zzz first, then aaa loses.
  EXPECT_TRUE(sh.ingest_remote(m_high).applied);
  auto r = sh.ingest_remote(m_low);
  EXPECT_FALSE(r.applied);
  EXPECT_FALSE(r.rejected);
  EXPECT_EQ(1u, sh.total_conflicts_detected());
  EXPECT_EQ(1u, sh.total_conflicts_lost());
}

TEST(StateClusterShardTest, ConflictResolutionDisabledAlwaysApplies) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  // Default: not enabled.
  auto m_high = make_set_value("zzz", 1, "k", "high");
  auto m_low = make_set_value("aaa", 2, "k", "low");
  EXPECT_TRUE(sh.ingest_remote(m_high).applied);
  EXPECT_TRUE(sh.ingest_remote(m_low).applied);
  EXPECT_EQ(0u, sh.total_conflicts_detected());
}

TEST(StateClusterShardTest, MetricsCountersAdvance) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  EXPECT_EQ(0u, sh.total_remote_applied());
  auto m = make_set_value("nodeB", 1, "p", "v");
  sh.ingest_remote(m);
  EXPECT_EQ(1u, sh.total_remote_applied());
  // duplicate
  sh.ingest_remote(m);
  EXPECT_EQ(1u, sh.total_remote_duplicates());
}

// ---- Phase 6: subtree delegation routing ----

TEST(StateClusterShardTest, DelegationEnforcementRejectsForeignWrites) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.delegation().delegate("simulation", "clusterB", "grpc://h:1");
  sh.set_enforce_delegation(true);

  auto m = make_set_value("nodeC", 1, "simulation.volume", "v");
  auto r = sh.ingest_remote(m);
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_NE(r.reject_reason.find("delegated to cluster 'clusterB'"), std::string::npos)
      << r.reject_reason;
  EXPECT_EQ(1u, sh.total_remote_rejected());
  EXPECT_EQ(1u, sh.total_delegation_routed());
  EXPECT_EQ(0u, sh.total_delegation_expired());
}

TEST(StateClusterShardTest, DelegationEnforcementAllowsLocalCluster) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  // Delegation that maps to the local cluster: NOT a foreign route.
  sh.delegation().delegate("ours", "clusterA");
  sh.set_enforce_delegation(true);

  auto m = make_set_value("nodeB", 1, "ours.x", "v");
  EXPECT_TRUE(sh.ingest_remote(m).applied);
  EXPECT_EQ(0u, sh.total_delegation_routed());
}

TEST(StateClusterShardTest, DelegationEnforcementAllowsUncoveredPaths) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.delegation().delegate("simulation", "clusterB");
  sh.set_enforce_delegation(true);

  // Path outside any delegation prefix: stays local.
  auto m = make_set_value("nodeB", 1, "controls.knob", "v");
  EXPECT_TRUE(sh.ingest_remote(m).applied);
}

TEST(StateClusterShardTest, DelegationEnforcementRejectsOnExpiredLease) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  // Inject a controllable clock.
  std::uint64_t now = 1000;
  sh.delegation().set_clock([&now]() { return now; });
  sh.delegation().delegate("simulation", "clusterB", "", /*lease=*/500);
  sh.set_enforce_delegation(true);

  // Before expiry: rejected as foreign-routed.
  now = 1499;
  auto r1 = sh.ingest_remote(make_set_value("nodeC", 1, "simulation.x", "v"));
  EXPECT_TRUE(r1.rejected);
  EXPECT_EQ(1u, sh.total_delegation_routed());

  // After expiry: rejected as expired.
  now = 1600;
  auto r2 = sh.ingest_remote(make_set_value("nodeC", 2, "simulation.x", "v"));
  EXPECT_TRUE(r2.rejected);
  EXPECT_NE(r2.reject_reason.find("expired"), std::string::npos) << r2.reject_reason;
  EXPECT_EQ(1u, sh.total_delegation_expired());
}

TEST(StateClusterShardTest, DelegationDisabledDoesNotReject) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.delegation().delegate("simulation", "clusterB");
  // enforce_delegation defaults to false.

  auto m = make_set_value("nodeC", 1, "simulation.x", "v");
  EXPECT_TRUE(sh.ingest_remote(m).applied);
  EXPECT_EQ(0u, sh.total_delegation_routed());
}

TEST(StateClusterShardTest, RoutePathClassifiesPaths) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.delegation().delegate("simulation", "clusterB");

  auto loc = sh.route_path("controls.x");
  EXPECT_EQ(loc.kind, cvc::state_delegation_manager::route_kind::local);

  auto rem = sh.route_path("simulation.volume");
  EXPECT_EQ(rem.kind, cvc::state_delegation_manager::route_kind::remote);
  EXPECT_EQ(rem.cluster_id, "clusterB");
}

TEST(StateClusterShardTest, AuthorityTransferUpdatesRouting) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.delegation().delegate("p", "clusterB");
  sh.set_enforce_delegation(true);

  auto r1 = sh.ingest_remote(make_set_value("nodeC", 1, "p.x", "v"));
  EXPECT_TRUE(r1.rejected);
  EXPECT_NE(r1.reject_reason.find("clusterB"), std::string::npos);

  // Transfer authority.
  sh.delegation().delegate("p", "clusterC");
  auto r2 = sh.ingest_remote(make_set_value("nodeC", 2, "p.x", "v"));
  EXPECT_TRUE(r2.rejected);
  EXPECT_NE(r2.reject_reason.find("clusterC"), std::string::npos);

  // Transfer back to local.
  sh.delegation().delegate("p", "clusterA");
  auto r3 = sh.ingest_remote(make_set_value("nodeC", 3, "p.x", "v"));
  EXPECT_TRUE(r3.applied);
}

TEST(StateClusterShardTest, DelegationRevocationRestoresLocal) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.delegation().delegate("simulation", "clusterB");
  sh.set_enforce_delegation(true);

  EXPECT_TRUE(sh.ingest_remote(make_set_value("nodeC", 1, "simulation.x", "v")).rejected);
  EXPECT_TRUE(sh.delegation().revoke("simulation"));
  EXPECT_TRUE(sh.ingest_remote(make_set_value("nodeC", 2, "simulation.x", "v")).applied);
}

TEST(StateClusterShardTest, InlinePayloadOffloadToBlobStore) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  // Set up blob store and threshold.
  cvc::memory_state_blob_store store;
  sh.set_blob_store(&store);
  sh.set_max_inline_payload_bytes(10); // offload values > 10 bytes

  // Write a small value (should stay inline).
  cvc::state::instance(a)("test.small").value(std::string("hi"));
  cvc::state::instance(a)("test.small").value(std::string("hello")); // 5 bytes
  auto drained = sh.drain_local();
  bool found_small = false;
  for (auto &m : drained) {
    if (m.path == "test.small") {
      found_small = true;
      EXPECT_EQ(m.payload.kind, cvc::state_payload_kind::none);
      EXPECT_EQ(m.string_value, "hello");
    }
  }
  EXPECT_TRUE(found_small);

  // Write a large value (should be offloaded to blob store).
  std::string big_value(200, 'X');
  cvc::state::instance(a)("test.big").value(std::string("init"));
  cvc::state::instance(a)("test.big").value(big_value);
  auto drained2 = sh.drain_local();
  bool found_big = false;
  for (auto &m : drained2) {
    if (m.path == "test.big") {
      found_big = true;
      EXPECT_EQ(m.payload.kind, cvc::state_payload_kind::blob);
      EXPECT_TRUE(m.string_value.empty());
      EXPECT_FALSE(m.payload.blob.digest.empty());
      EXPECT_EQ(m.payload.blob.size_bytes, 200u);
      // Verify we can retrieve the blob from the store.
      std::vector<unsigned char> retrieved;
      EXPECT_TRUE(store.get(m.payload.blob.digest, retrieved));
      EXPECT_EQ(retrieved.size(), 200u);
      EXPECT_EQ(retrieved, std::vector<unsigned char>(200, 'X'));
    }
  }
  EXPECT_TRUE(found_big);
}

TEST(StateClusterShardTest, InlinePayloadOffloadDisabledByDefault) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  // No blob store set — large values should stay inline.
  std::string big_value(200, 'Y');
  cvc::state::instance(a)("test.noblobstore").value(std::string("init"));
  cvc::state::instance(a)("test.noblobstore").value(big_value);
  auto drained = sh.drain_local();
  for (auto &m : drained) {
    if (m.path == "test.noblobstore") {
      EXPECT_EQ(m.payload.kind, cvc::state_payload_kind::none);
      EXPECT_EQ(m.string_value, big_value);
    }
  }
}
