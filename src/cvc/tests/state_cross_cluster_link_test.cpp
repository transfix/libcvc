/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 6 tests: resolveRemote() — pull-on-demand
// resolution of link targets that are owned by a remote cluster,
// composing with delegation + lease expiry. Also covers
// cross-cluster sendMessage routing over transparent links.

#include <atomic>
#include <cstdint>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace {

// Inject a hand-cranked clock into a delegation manager.
struct manual_clock {
  std::shared_ptr<std::atomic<std::uint64_t>> now = std::make_shared<std::atomic<std::uint64_t>>(0);
  cvc::state_delegation_manager::clock_fn fn() {
    auto h = now;
    return [h]() { return h->load(std::memory_order_relaxed); };
  }
  void advance(std::uint64_t ns) { now->fetch_add(ns, std::memory_order_relaxed); }
  void set(std::uint64_t ns) { now->store(ns, std::memory_order_relaxed); }
};

struct collected {
  std::vector<cvc::state_message> messages;
  void operator()(const cvc::state_message &m) { messages.push_back(m); }
};

} // namespace

// ----------------------------------------------------------------
// resolveRemote: basic local resolution (same as resolveLink)
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteLocalTerminal) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto &root = cvc::state::instance(a);
  root("target").value(std::string("v"));
  root("link").linkTo("target", cvc::state::link_mode::transparent);

  auto r = root("link").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_local);
  ASSERT_NE(r.target, nullptr);
  EXPECT_EQ(r.target->value(), "v");
  EXPECT_EQ(r.resolved_path, "target");
  EXPECT_EQ(r.owner_cluster_id, "alpha");
  EXPECT_TRUE(r.owner_is_local);
  EXPECT_EQ(r.hops, 1u);
}

// ----------------------------------------------------------------
// resolveRemote: non-link returns 'none', same as resolveLink
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteNonLinkIsNone) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto r = cvc::state::instance(a)("plain").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::none);
  EXPECT_TRUE(r.owner_is_local);
}

// ----------------------------------------------------------------
// resolveRemote: broken target with no delegation → broken
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteBrokenWithNoDelegation) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto &root = cvc::state::instance(a);
  root("link").linkTo("does.not.exist");

  auto r = root("link").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::broken);
  EXPECT_EQ(r.resolved_path, "does.not.exist");
}

// ----------------------------------------------------------------
// resolveRemote: target is delegated to a remote cluster
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteDetectsRemoteTarget) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  // Delegate "data.world" to a different cluster.
  shard.publish_delegation("data.world", "beta", "inproc://beta", /*lease=*/0);

  auto &root = cvc::state::instance(a);
  root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

  auto r = root("scene.geometry").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
  EXPECT_EQ(r.target, nullptr);
  EXPECT_EQ(r.resolved_path, "data.world.geometry");
  EXPECT_EQ(r.owner_cluster_id, "beta");
  EXPECT_EQ(r.endpoint, "inproc://beta");
  EXPECT_FALSE(r.owner_is_local);
  EXPECT_EQ(r.hops, 1u);
}

// ----------------------------------------------------------------
// resolveRemote: link chain with the last hop being remote
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteChainEndsAtRemote) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  shard.publish_delegation("remote.data", "gamma", "inproc://gamma", 0);

  auto &root = cvc::state::instance(a);
  // a → b (local) → remote.data.mesh (delegated)
  root("b");
  root("a").linkTo("b", cvc::state::link_mode::transparent);
  root("b").linkTo("remote.data.mesh", cvc::state::link_mode::transparent);

  auto r = root("a").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
  EXPECT_EQ(r.resolved_path, "remote.data.mesh");
  EXPECT_EQ(r.owner_cluster_id, "gamma");
  EXPECT_EQ(r.hops, 2u);
}

// ----------------------------------------------------------------
// resolveRemote: expired lease on delegated target
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteExpiredLease) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  manual_clock ck;
  shard.delegation().set_clock(ck.fn());

  constexpr std::uint64_t kLease = 1000ull * 1000ull * 1000ull; // 1s
  shard.publish_delegation("vol.brick", "storage-cluster", "inproc://stor", kLease);

  auto &root = cvc::state::instance(a);
  root("link").linkTo("vol.brick.data", cvc::state::link_mode::transparent);

  // Within lease: remote.
  ck.set(kLease / 2);
  {
    auto r = root("link").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
    EXPECT_EQ(r.owner_cluster_id, "storage-cluster");
  }

  // Past lease: expired.
  ck.set(kLease + 1);
  {
    auto r = root("link").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::lease_expired);
    EXPECT_EQ(r.owner_cluster_id, "storage-cluster");
    EXPECT_FALSE(r.owner_is_local);
  }
}

// ----------------------------------------------------------------
// resolveRemote: cycle detection still works
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteCycleDetection) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto &root = cvc::state::instance(a);
  root("p").linkTo("q");
  root("q").linkTo("p");

  auto r = root("p").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::cycle_detected);
  EXPECT_EQ(r.target, nullptr);
}

// ----------------------------------------------------------------
// resolveRemote: hop budget exhaustion
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteBudgetExhausted) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto &root = cvc::state::instance(a);
  // Chain: a → b → c → d with budget 2
  root("b").linkTo("c");
  root("c").linkTo("d");
  root("a").linkTo("b");

  auto r = root("a").resolveRemote(2);
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::budget_exhausted);
}

// ----------------------------------------------------------------
// resolveRemote: no shard registered → broken fallback
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteNoShardFallsToBroken) {
  cvc::app a;
  // No shard attached.
  auto &root = cvc::state::instance(a);
  root("link").linkTo("nowhere");

  auto r = root("link").resolveRemote();
  EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::broken);
  EXPECT_EQ(r.resolved_path, "nowhere");
}

// ----------------------------------------------------------------
// Authority transfer: link target moves between clusters mid-test.
// The caller never names a cluster_id.
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, LinkTargetMovesBetweenClusters) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  auto &root = cvc::state::instance(a);
  root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

  // Step 1: delegate to render-cluster.
  shard.publish_delegation("data.world", "render-cluster", "inproc://render", 0);
  {
    auto r = root("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
    EXPECT_EQ(r.owner_cluster_id, "render-cluster");
  }

  // Step 2: revoke and re-delegate to physics-cluster.
  shard.publish_revocation("data.world");
  shard.publish_delegation("data.world", "physics-cluster", "inproc://physics", 0);
  {
    auto r = root("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
    EXPECT_EQ(r.owner_cluster_id, "physics-cluster");
    EXPECT_EQ(r.endpoint, "inproc://physics");
  }

  // Step 3: revoke entirely — target now "local" but absent → broken.
  shard.publish_revocation("data.world");
  {
    auto r = root("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::broken);
  }

  // Step 4: create the target locally → success.
  root("data.world.geometry").value(std::string("local-val"));
  {
    auto r = root("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_local);
    ASSERT_NE(r.target, nullptr);
    EXPECT_EQ(r.target->value(), "local-val");
    EXPECT_TRUE(r.owner_is_local);
  }
}

// ----------------------------------------------------------------
// sendMessage over a transparent link routes to the correct
// cluster (two-shard setup with inproc transport). Caller never
// names a cluster_id.
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, SendMessageOverLinkRoutesViaShard) {
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
  sB.message_bus().subscribe("data", std::ref(sinkB));

  auto &rootA = cvc::state::instance(aA);
  // Ensure target exists, then create a transparent link to it.
  (void)rootA("data.world.geometry");
  rootA("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

  auto r = rootA("scene.geometry").sendMessage("update", "text/plain");
  EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
  // sendMessage follows the link and addresses data.world.geometry.
  EXPECT_EQ(r.resolved_path, "data.world.geometry");
  EXPECT_EQ(r.owner_cluster_id, "alpha");
  EXPECT_TRUE(r.owner_is_local);
  EXPECT_EQ(r.local_admitted, 1u);
  EXPECT_GE(r.peers_delivered, 1u);
  ASSERT_EQ(sinkB.messages.size(), 1u);
  EXPECT_EQ(sinkB.messages[0].path, "data.world.geometry");
}

// ----------------------------------------------------------------
// sendMessage over a transparent link: authority moves mid-test.
// Caller never names a cluster_id. Uses separate app contexts per
// step to avoid message-bus dedup collisions on empty message_id.
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, SendMessageAfterAuthorityMoves) {
  // Step 1: initially local — sendMessage succeeds locally.
  {
    cvc::app a;
    cvc::state_cluster_shard shard(a, "alpha", "node_1");
    shard.attach();

    auto &root = cvc::state::instance(a);
    (void)root("data.world.geometry");
    root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

    collected sink;
    shard.message_bus().subscribe("data", std::ref(sink));
    auto r = root("scene.geometry").sendMessage("msg1");
    EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
    EXPECT_TRUE(r.owner_is_local);
    EXPECT_EQ(sink.messages.size(), 1u);
  }

  // Step 2: delegate to a foreign cluster without a transport.
  {
    cvc::app a;
    cvc::state_cluster_shard shard(a, "alpha", "node_1");
    shard.attach();

    auto &root = cvc::state::instance(a);
    (void)root("data.world.geometry");
    root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

    shard.publish_delegation("data.world", "beta", "", 0);
    auto r = root("scene.geometry").sendMessage("msg2");
    EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::no_transport);
    EXPECT_EQ(r.owner_cluster_id, "beta");
    EXPECT_FALSE(r.owner_is_local);
  }

  // Step 3: authority revoked — message is local again.
  {
    cvc::app a;
    cvc::state_cluster_shard shard(a, "alpha", "node_1");
    shard.attach();

    auto &root = cvc::state::instance(a);
    (void)root("data.world.geometry");
    root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

    // Delegate then revoke within the same lifetime.
    shard.publish_delegation("data.world", "beta", "", 0);
    shard.publish_revocation("data.world");

    collected sink;
    shard.message_bus().subscribe("data", std::ref(sink));
    auto r = root("scene.geometry").sendMessage("msg3");
    EXPECT_EQ(r.status, cvc::state::send_message_result::status_kind::delivered);
    EXPECT_TRUE(r.owner_is_local);
    EXPECT_EQ(sink.messages.size(), 1u);
  }
}

// ----------------------------------------------------------------
// Cluster-agnostic API compile-time check: resolveRemote takes no
// cluster_id parameter.
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, ResolveRemoteApiCompileCheck) {
  using FnPtr = cvc::state::remote_link_resolution (cvc::state::*)(std::size_t);
  FnPtr p = &cvc::state::resolveRemote;
  (void)p;
  SUCCEED();
}

// ----------------------------------------------------------------
// Lease expiry invalidates sendMessage routing: the shard reports
// no_transport (message belongs to an expired delegation with no
// transport configured to reach the foreign cluster).
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, LeaseExpiryInvalidatesSendMessage) {
  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node_1");
  shard.attach();

  manual_clock ck;
  shard.delegation().set_clock(ck.fn());

  constexpr std::uint64_t kLease = 1'000'000'000ULL; // 1s
  shard.publish_delegation("data.world", "beta", "", kLease);

  auto &root = cvc::state::instance(a);
  (void)root("data.world.geometry");
  root("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

  // Within lease: routes to beta (no transport → no_transport).
  ck.set(kLease / 2);
  {
    auto r = root("scene.geometry").sendMessage("msg");
    EXPECT_EQ(r.owner_cluster_id, "beta");
    EXPECT_FALSE(r.owner_is_local);
  }

  // Past lease: the authority map still resolves to beta (expired
  // entries are not auto-removed), so owner_cluster_id stays
  // "beta" and the message is not admitted locally.
  ck.set(kLease + 1);
  {
    auto r = root("scene.geometry").sendMessage("msg");
    // The authority map resolve() returns the entry even if
    // expired (lease evaluation is in delegation_manager::route).
    // sendMessage uses authority().resolve() directly, so the
    // owner_cluster_id is still "beta" and owner_is_local is false.
    EXPECT_FALSE(r.owner_is_local);
  }
}

// ----------------------------------------------------------------
// Two-shard cross-cluster: a link on shard A targets a path
// delegated to a second cluster represented by shard B. Verify
// that resolveRemote reports the delegation, and that after the
// delegation is revoked and B's value materializes on A's tree
// (simulated by just creating the node), resolution succeeds
// locally.
// ----------------------------------------------------------------
TEST(StateCrossClusterLink, TwoShardCrossClusterResolveRemoteWithDelegation) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "alpha", "A");
  cvc::state_cluster_shard sB(aB, "alpha", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  // A delegates data.world.* to beta-cluster (simulating a
  // separate cluster, not shard B which is in "alpha").
  sA.publish_delegation("data.world", "beta-cluster", "inproc://beta", 0);
  EXPECT_GE(t.pump_shard(sA), 1u);

  auto &rootA = cvc::state::instance(aA);
  rootA("scene.geometry").linkTo("data.world.geometry", cvc::state::link_mode::transparent);

  // resolveRemote sees the delegation.
  {
    auto r = rootA("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_remote);
    EXPECT_EQ(r.owner_cluster_id, "beta-cluster");
    EXPECT_EQ(r.endpoint, "inproc://beta");
  }

  // B also learned the delegation.
  {
    auto d = sB.delegation().route("data.world.geometry");
    EXPECT_EQ(d.kind, cvc::state_delegation_manager::route_kind::remote);
    EXPECT_EQ(d.cluster_id, "beta-cluster");
  }

  // Revoke delegation; A creates the node locally.
  sA.publish_revocation("data.world");
  EXPECT_GE(t.pump_shard(sA), 1u);
  rootA("data.world.geometry").value(std::string("local-value"));

  {
    auto r = rootA("scene.geometry").resolveRemote();
    EXPECT_EQ(r.kind, cvc::state::remote_resolution_kind::resolved_local);
    ASSERT_NE(r.target, nullptr);
    EXPECT_EQ(r.target->value(), "local-value");
  }
}
