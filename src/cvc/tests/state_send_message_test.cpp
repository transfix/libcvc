/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 2 tests: cluster-agnostic state::sendMessage().
// The contract is that callers send a message on a node and never
// name a cluster_id; the default shard registered for the app
// context derives the owning cluster from its authority map and
// routes accordingly.

#include <atomic>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

struct collected {
  std::vector<cvc::state_message> messages;
  void operator()(const cvc::state_message &m) { messages.push_back(m); }
};

} // namespace

TEST(StateSendMessage, NoDefaultShardYieldsStructuredNoOp) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("scene.geometry");
  auto r = n.sendMessage("hello");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::no_shard);
  EXPECT_EQ(r.resolved_path, "scene.geometry");
  EXPECT_EQ(r.local_admitted, 0u);
  EXPECT_EQ(r.peers_delivered, 0u);
}

TEST(StateSendMessage, DeliversToLocalSubscriberOnDefaultShard) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();
  collected sink;
  shard.message_bus().subscribe("scene", std::ref(sink));

  auto &n = cvc::state::instance(a)("scene.geometry");
  auto r = n.sendMessage("hello", "text/plain");

  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
  EXPECT_EQ(r.resolved_path, "scene.geometry");
  EXPECT_EQ(r.owner_cluster_id, "alpha");
  EXPECT_TRUE(r.owner_is_local);
  EXPECT_EQ(r.local_admitted, 1u);
  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].path, "scene.geometry");
  EXPECT_EQ(sink.messages[0].string_value, "hello");
  EXPECT_EQ(sink.messages[0].cluster_id, "alpha");
  EXPECT_EQ(sink.messages[0].origin_node_id, "node_1");
}

TEST(StateSendMessage, FollowsLinkChainBeforeAddressing) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();
  collected sink;
  shard.message_bus().subscribe("data", std::ref(sink));

  auto &root = cvc::state::instance(a);
  // Ensure the link target exists, then point the alias at it.
  (void)root("data.world.geometry");
  root("scene.geometry").linkTo("data.world.geometry");

  auto r = root("scene.geometry").sendMessage("hi");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
  EXPECT_EQ(r.resolved_path, "data.world.geometry");
  ASSERT_EQ(sink.messages.size(), 1u);
  EXPECT_EQ(sink.messages[0].path, "data.world.geometry");
}

TEST(StateSendMessage, BrokenLinkIsReported) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();
  auto &n = cvc::state::instance(a)("scene.geometry");
  n.linkTo("nowhere.in.tree");
  auto r = n.sendMessage("x");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::broken_link);
  EXPECT_EQ(r.local_admitted, 0u);
}

TEST(StateSendMessage, CycleIsReported) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();
  auto &root = cvc::state::instance(a);
  root("p").linkTo("q");
  root("q").linkTo("p");
  auto r = root("p").sendMessage("x");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::cycle_detected);
  EXPECT_EQ(r.local_admitted, 0u);
}

TEST(StateSendMessage, CallerNeverNamesClusterId_ApiCompileCheck) {
  // Compile-time guard: the only sendMessage overload takes
  // (payload, content_type=, hop_budget=, binary=); there is no
  // cluster parameter the caller could pass. This test exists so a
  // future refactor that re-introduces cluster_id into the signature
  // fails the build here. (`binary` selects the text-vs-bytes wire
  // slot, not a cluster.)
  using FnPtr = cvc::state::send_message_result (cvc::state::*)(
      const std::string &, const std::string &, std::size_t, bool);
  FnPtr p = &cvc::state::sendMessage;
  (void)p;
  SUCCEED();
}

TEST(StateSendMessage, AuthorityMapStampsOwnerClusterId) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();
  // Delegate scene.* to a foreign cluster. The caller-side
  // sendMessage MUST stamp that as the owner cluster_id even
  // though we never name it in the call.
  shard.publish_delegation("scene", "beta", "inproc://beta", /*lease*/ 0);

  collected local_sink;
  shard.message_bus().subscribe("scene", std::ref(local_sink));

  auto r = cvc::state::instance(a)("scene.geometry").sendMessage("payload");
  // Owner is beta, not alpha; we still report "delivered" because
  // there is no transport configured (publish_message no-op),
  // but the local bus must NOT admit foreign-cluster messages.
  EXPECT_EQ(r.owner_cluster_id, "beta");
  EXPECT_FALSE(r.owner_is_local);
  EXPECT_EQ(r.local_admitted, 0u);
  EXPECT_EQ(local_sink.messages.size(), 0u);
  // Without a transport, the structured no_transport status is
  // returned so callers can distinguish "delivered nowhere" from
  // "delivered locally".
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::no_transport);
}

TEST(StateSendMessage, CrossClusterDeliversViaTransport) {
  // Two shards in the same cluster "alpha" wired through an
  // in-process transport. Sending on shard A reaches shard B's
  // subscribers without the caller naming the cluster.
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "alpha", "A");
  cvc::state_cluster_shard sB(aB, "alpha", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  sA.set_transport(&t);
  sB.set_transport(&t);

  collected sinkB;
  sB.message_bus().subscribe("chat", std::ref(sinkB));

  // Important: stamp a stable message_id so dedup is meaningful
  // and the inproc transport actually fans out.
  auto &n = cvc::state::instance(aA)("chat.lobby");
  auto r = n.sendMessage("hi from A");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
  EXPECT_EQ(r.owner_cluster_id, "alpha");
  EXPECT_TRUE(r.owner_is_local);
  EXPECT_EQ(r.local_admitted, 1u);
  // Inproc transport fans out synchronously; it does not track
  // `peers` in publish_message_stats, only `delivered`.
  EXPECT_GE(r.peers_delivered, 1u);
  ASSERT_EQ(sinkB.messages.size(), 1u);
  EXPECT_EQ(sinkB.messages[0].path, "chat.lobby");
  EXPECT_EQ(sinkB.messages[0].cluster_id, "alpha");
  EXPECT_EQ(sinkB.messages[0].origin_node_id, "A");
}

// The default shard is each app's own: two apps, in both orders, each find theirs; a later app
// (made after another is gone, perhaps at the same address) starts with none.
TEST(StateSendMessage, DefaultShardIsPerApp) {
  for (int order = 0; order < 2; ++order) {
    cvc::app a, b;
    cvc::state_cluster_shard sA(a, "alpha", "A");
    cvc::state_cluster_shard sB(b, "alpha", "B");
    if (order == 0) {
      sA.attach();
      EXPECT_EQ(cvc::state_cluster_shard::default_for(b), nullptr);
      sB.attach();
    } else {
      sB.attach();
      EXPECT_EQ(cvc::state_cluster_shard::default_for(a), nullptr);
      sA.attach();
    }
    EXPECT_EQ(cvc::state_cluster_shard::default_for(a), &sA);
    EXPECT_EQ(cvc::state_cluster_shard::default_for(b), &sB);
    sA.detach();
    EXPECT_EQ(cvc::state_cluster_shard::default_for(a), nullptr);
    EXPECT_EQ(cvc::state_cluster_shard::default_for(b), &sB);
  }
  for (int round = 0; round < 3; ++round) {
    auto app = std::make_unique<cvc::app>();
    EXPECT_EQ(cvc::state_cluster_shard::default_for(*app), nullptr) << "round " << round;
    cvc::state_cluster_shard s(*app, "alpha", "N");
    s.attach(); // destroyed attached, before its app
  }
}

TEST(StateSendMessage, DefaultShardAppsOnThreadsAtOnce) {
  constexpr int kApps = 4;
  std::vector<std::string> seen(kApps);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &seen] {
      cvc::app app;
      cvc::state_cluster_shard shard(app, "alpha", "node_" + std::to_string(i));
      shard.attach();
      collected sink;
      shard.message_bus().subscribe("chat", std::ref(sink));
      auto r = cvc::state::instance(app)("chat.lobby").sendMessage("hi");
      seen[i] = cvc::state_cluster_shard::default_for(app)->local_node_id() + " " +
                std::to_string(r.local_admitted) + " " +
                (sink.messages.empty() ? std::string("none") : sink.messages[0].origin_node_id);
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i) {
    const std::string node = "node_" + std::to_string(i);
    EXPECT_EQ(seen[i], node + " 1 " + node);
  }
}

// A shard's message ids carry a random tag of its own, so a shard made again under the same node
// id repeats none, and a peer's bus (which drops an id it has seen from that origin) takes the
// new shard's messages.
TEST(StateSendMessage, MessageIdsSurviveAShardMadeAgain) {
  cvc::app a, b;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sB(b, "alpha", "B");
  sB.attach();
  t.register_shard(&sB);
  sB.set_transport(&t);
  collected sinkB;
  sB.message_bus().subscribe("chat", std::ref(sinkB));

  for (int round = 0; round < 2; ++round) {
    cvc::state_cluster_shard sA(a, "alpha", "A");
    sA.attach();
    t.register_shard(&sA);
    sA.set_transport(&t);
    cvc::state::instance(a)("chat.lobby").sendMessage("round " + std::to_string(round));
    t.unregister_shard(&sA);
  }
  ASSERT_EQ(sinkB.messages.size(), 2u) << "the second shard's first message was dropped";
  EXPECT_NE(sinkB.messages[0].message_id, sinkB.messages[1].message_id);
  EXPECT_EQ(sinkB.messages[1].string_value, "round 1");
}

TEST(StateSendMessage, DefaultShardIsFirstWriterWins) {
  cvc::app a;
  cvc::state_cluster_shard s1(a, "alpha", "A");
  cvc::state_cluster_shard s2(a, "alpha", "B");
  s1.attach();
  s2.attach();
  EXPECT_EQ(cvc::state_cluster_shard::default_for(a), &s1);
  // After s1 detaches, s2 can take over by re-installing.
  s1.detach();
  EXPECT_EQ(cvc::state_cluster_shard::default_for(a), nullptr);
  s2.install_as_default();
  EXPECT_EQ(cvc::state_cluster_shard::default_for(a), &s2);
}
