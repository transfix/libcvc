/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_distributed_admin.h>

#include <cvc/app.h>
#include <cvc/state_blob_store.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_peer_registry.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

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

} // namespace

TEST(StateDistributedAdmin, EmptyReportWhenNothingAttached) {
  cvc::state_distributed_admin admin;
  auto r = admin.snapshot();
  EXPECT_FALSE(r.shard.attached);
  EXPECT_FALSE(r.bus.attached);
  EXPECT_FALSE(r.blobs.attached);
  EXPECT_TRUE(r.delegations.empty());
  EXPECT_TRUE(r.peers.empty());

  std::string text = admin.to_text(r);
  EXPECT_NE(text.find("[shard]"), std::string::npos);
  EXPECT_NE(text.find("detached"), std::string::npos);
  EXPECT_NE(text.find("[blobs]"), std::string::npos);
}

TEST(StateDistributedAdmin, ShardSnapshotMirrorsLiveCounters) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  sh.attach();
  sh.set_enforce_delegation(true);
  sh.delegation().delegate("foreign", "cB", "grpc://h:1");

  cvc::state_distributed_admin admin;
  admin.attach_shard(&sh);

  // Before any traffic.
  auto r0 = admin.snapshot();
  ASSERT_TRUE(r0.shard.attached);
  EXPECT_EQ(r0.shard.cluster_id, "cA");
  EXPECT_EQ(r0.shard.node_id, "nodeA");
  EXPECT_TRUE(r0.shard.enforce_delegation);
  EXPECT_EQ(r0.shard.total_remote_applied, 0u);
  EXPECT_EQ(r0.shard.total_delegation_routed, 0u);
  ASSERT_EQ(r0.delegations.size(), 1u);
  EXPECT_EQ(r0.delegations[0].prefix, "foreign");
  EXPECT_EQ(r0.delegations[0].cluster_id, "cB");

  // One foreign reject + one local apply.
  EXPECT_TRUE(sh.ingest_remote(make_set_value("nodeC", 1, "foreign.x", "v"))
                  .rejected);
  EXPECT_TRUE(
      sh.ingest_remote(make_set_value("nodeC", 2, "ours.x", "v")).applied);

  auto r1 = admin.snapshot();
  EXPECT_EQ(r1.shard.total_remote_applied, 1u);
  EXPECT_EQ(r1.shard.total_remote_rejected, 1u);
  EXPECT_EQ(r1.shard.total_delegation_routed, 1u);
}

TEST(StateDistributedAdmin, BusSnapshotMirrorsCounters) {
  cvc::state_message_bus bus;
  cvc::state_distributed_admin admin;
  admin.attach_message_bus(&bus);

  auto r = admin.snapshot();
  ASSERT_TRUE(r.bus.attached);
  EXPECT_EQ(r.bus.total_admitted, 0u);
  EXPECT_EQ(r.bus.total_dispatched, 0u);
}

TEST(StateDistributedAdmin, PeerSnapshotIncludesAllPeers) {
  cvc::state_peer_registry peers;
  peers.add_peer("n1", "cA", "ep1", {"a", "b"});
  peers.add_peer("n2", "cB", "ep2", {});
  peers.note_seen("n1", 1234);

  cvc::state_distributed_admin admin;
  admin.attach_peer_registry(&peers);

  auto r = admin.snapshot();
  ASSERT_EQ(r.peers.size(), 2u);
  // Order is unspecified; find both.
  bool saw_n1 = false, saw_n2 = false;
  for (auto &p : r.peers) {
    if (p.node_id == "n1") {
      saw_n1 = true;
      EXPECT_EQ(p.cluster_id, "cA");
      EXPECT_EQ(p.endpoint, "ep1");
      EXPECT_EQ(p.subscriptions.size(), 2u);
      EXPECT_EQ(p.last_seen_ns, 1234u);
    } else if (p.node_id == "n2") {
      saw_n2 = true;
      EXPECT_EQ(p.cluster_id, "cB");
    }
  }
  EXPECT_TRUE(saw_n1 && saw_n2);
}

TEST(StateDistributedAdmin, BlobReportMirrorsStore) {
  cvc::memory_state_blob_store store;
  std::vector<unsigned char> a{1, 2, 3, 4};
  std::vector<unsigned char> b{9, 9, 9};
  auto refA = store.put(a);
  store.put(b);

  cvc::state_distributed_admin admin;
  admin.attach_blob_store(&store);
  auto r = admin.snapshot();
  ASSERT_TRUE(r.blobs.attached);
  EXPECT_EQ(r.blobs.count, 2u);
  EXPECT_EQ(r.blobs.bytes_stored, 7u);
  EXPECT_FALSE(refA.digest.empty());
}

TEST(StateDistributedAdmin, ToTextContainsAllSections) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  cvc::state_message_bus bus;
  cvc::state_peer_registry peers;
  cvc::memory_state_blob_store store;

  cvc::state_distributed_admin admin;
  admin.attach_shard(&sh);
  admin.attach_message_bus(&bus);
  admin.attach_peer_registry(&peers);
  admin.attach_blob_store(&store);

  std::string txt = admin.to_text();
  EXPECT_NE(txt.find("[shard]"), std::string::npos);
  EXPECT_NE(txt.find("cluster_id=cA"), std::string::npos);
  EXPECT_NE(txt.find("node_id=nodeA"), std::string::npos);
  EXPECT_NE(txt.find("[delegations]"), std::string::npos);
  EXPECT_NE(txt.find("[peers]"), std::string::npos);
  EXPECT_NE(txt.find("[bus]"), std::string::npos);
  EXPECT_NE(txt.find("[blobs]"), std::string::npos);
  // No "detached" since all four are attached.
  EXPECT_EQ(txt.find("detached"), std::string::npos);
}

TEST(StateDistributedAdmin, GcBlobsRemovesUnreferenced) {
  cvc::memory_state_blob_store store;
  auto refA = store.put({1, 2, 3});
  auto refB = store.put({4, 5, 6, 7});
  auto refC = store.put({8});
  ASSERT_EQ(store.size(), 3u);
  ASSERT_EQ(store.bytes_stored(), 8u);

  cvc::state_distributed_admin admin;
  admin.attach_blob_store(&store);

  std::unordered_set<std::string> live{refA.digest, refC.digest};
  auto g = admin.gc_blobs(live);
  EXPECT_EQ(g.scanned, 3u);
  EXPECT_EQ(g.removed, 1u);
  EXPECT_EQ(g.bytes_freed, 4u);
  EXPECT_EQ(store.size(), 2u);
  EXPECT_TRUE(store.has(refA.digest));
  EXPECT_FALSE(store.has(refB.digest));
  EXPECT_TRUE(store.has(refC.digest));
}

TEST(StateDistributedAdmin, GcBlobsAllLivePreservesAll) {
  cvc::memory_state_blob_store store;
  auto refA = store.put({1, 2, 3});
  auto refB = store.put({4, 5, 6, 7});

  cvc::state_distributed_admin admin;
  admin.attach_blob_store(&store);

  auto g = admin.gc_blobs({refA.digest, refB.digest});
  EXPECT_EQ(g.scanned, 2u);
  EXPECT_EQ(g.removed, 0u);
  EXPECT_EQ(g.bytes_freed, 0u);
  EXPECT_EQ(store.size(), 2u);
}

TEST(StateDistributedAdmin, GcBlobsEmptyLiveSetClearsStore) {
  cvc::memory_state_blob_store store;
  store.put({1, 2, 3});
  store.put({4, 5});

  cvc::state_distributed_admin admin;
  admin.attach_blob_store(&store);
  auto g = admin.gc_blobs({});
  EXPECT_EQ(g.scanned, 2u);
  EXPECT_EQ(g.removed, 2u);
  EXPECT_EQ(g.bytes_freed, 5u);
  EXPECT_EQ(store.size(), 0u);
  EXPECT_EQ(store.bytes_stored(), 0u);
}

TEST(StateDistributedAdmin, GcBlobsNoStoreReturnsZero) {
  cvc::state_distributed_admin admin;
  std::unordered_set<std::string> live{"deadbeef"};
  auto g = admin.gc_blobs(live);
  EXPECT_EQ(g.scanned, 0u);
  EXPECT_EQ(g.removed, 0u);
  EXPECT_EQ(g.bytes_freed, 0u);
}

TEST(StateDistributedAdmin, DetachReplacesPointerWithNullptr) {
  cvc::memory_state_blob_store store;
  cvc::state_distributed_admin admin;
  admin.attach_blob_store(&store);
  EXPECT_EQ(admin.blob_store(), &store);
  admin.attach_blob_store(nullptr);
  EXPECT_EQ(admin.blob_store(), nullptr);
  auto r = admin.snapshot();
  EXPECT_FALSE(r.blobs.attached);
}

TEST(StateDistributedAdmin, ReportIsCopyable) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "cA", "nodeA");
  cvc::state_distributed_admin admin;
  admin.attach_shard(&sh);
  auto r1 = admin.snapshot();
  auto r2 = r1; // copy
  EXPECT_EQ(r1.shard.cluster_id, r2.shard.cluster_id);
  EXPECT_EQ(r1.delegations.size(), r2.delegations.size());
}
