/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 6 integration tests: subtree delegation propagated through
// the live in-process transport. These exercise the wiring added
// to state_cluster_shard (publish_delegation/publish_revocation
// plus the delegate_subtree/revoke_delegation handler in
// ingest_remote) end-to-end. Closes the survey gap "delegation
// metadata travels over the wire, not just inside one shard".

#include <atomic>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_delegation_manager.h>
#include <cvc/core/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>

namespace {

// Inject a hand-cranked clock into a delegation manager. Returns
// a pointer the test can advance freely; the captured shared_ptr
// keeps the storage alive for the whole shard lifetime.
struct manual_clock {
  std::shared_ptr<std::atomic<std::uint64_t>> now = std::make_shared<std::atomic<std::uint64_t>>(0);
  cvc::state_delegation_manager::clock_fn fn() {
    auto h = now;
    return [h]() { return h->load(std::memory_order_relaxed); };
  }
  void advance(std::uint64_t ns) { now->fetch_add(ns, std::memory_order_relaxed); }
  void set(std::uint64_t ns) { now->store(ns, std::memory_order_relaxed); }
};

} // namespace

// ----------------
// A delegates a subtree; B and C learn about it via the transport
// and route the same paths to the same target cluster.
// ----------------
TEST(StateDistributedDelegationIntegration, DelegationPropagatesViaTransport) {
  cvc::app aA, aB, aC;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "A", "A");
  cvc::state_cluster_shard sB(aB, "A", "B");
  cvc::state_cluster_shard sC(aC, "A", "C");
  sA.attach();
  sB.attach();
  sC.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);
  t.register_shard(&sC);

  // A is the local cluster for everyone in this transport (single
  // logical cluster "A" with three nodes), so a "remote" route
  // here must come from a delegation to a *different* cluster id.
  sA.publish_delegation("vol.brick", "B-cluster",
                        /*endpoint=*/"inproc://b",
                        /*lease=*/0);

  // Locally A is already updated.
  {
    auto r = sA.delegation().route("vol.brick.x");
    EXPECT_EQ(r.kind, cvc::state_delegation_manager::route_kind::remote);
    EXPECT_EQ(r.cluster_id, "B-cluster");
    EXPECT_EQ(r.matched_prefix, "vol.brick");
  }

  // Pump A so peers receive the control-plane mutation.
  EXPECT_GE(t.pump_shard(sA), 1u);

  for (auto *peer : {&sB, &sC}) {
    auto r = peer->delegation().route("vol.brick.x");
    EXPECT_EQ(r.kind, cvc::state_delegation_manager::route_kind::remote);
    EXPECT_EQ(r.cluster_id, "B-cluster");
    EXPECT_EQ(r.endpoint, "inproc://b");
    EXPECT_EQ(r.matched_prefix, "vol.brick");
    EXPECT_EQ(peer->total_delegations_applied(), 1u);
  }

  // Paths outside the prefix stay local on every peer.
  EXPECT_TRUE(sA.delegation().is_local("scene.geometry"));
  EXPECT_TRUE(sB.delegation().is_local("scene.geometry"));
  EXPECT_TRUE(sC.delegation().is_local("scene.geometry"));
}

// ----------------
// Revocations propagate the same way and clear remote routing.
// ----------------
TEST(StateDistributedDelegationIntegration, RevocationPropagates) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "A", "A");
  cvc::state_cluster_shard sB(aB, "A", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  sA.publish_delegation("vol.brick", "B-cluster", "", 0);
  EXPECT_GE(t.pump_shard(sA), 1u);
  ASSERT_EQ(sB.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::remote);

  sA.publish_revocation("vol.brick");
  EXPECT_GE(t.pump_shard(sA), 1u);

  EXPECT_TRUE(sA.delegation().is_local("vol.brick.x"));
  EXPECT_TRUE(sB.delegation().is_local("vol.brick.x"));
  EXPECT_EQ(sA.total_revocations_applied(), 1u);
  EXPECT_EQ(sB.total_revocations_applied(), 1u);
}

// ----------------
// Lease expiry is deterministic with an injected clock. The
// receiver applies the duration relative to its own clock; we
// install matched manual clocks on both sides so the test is
// reproducible.
// ----------------
TEST(StateDistributedDelegationIntegration, LeaseExpiryIsDeterministic) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "A", "A");
  cvc::state_cluster_shard sB(aB, "A", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  manual_clock ck;
  sA.delegation().set_clock(ck.fn());
  sB.delegation().set_clock(ck.fn());

  constexpr std::uint64_t kLease = 1000ull * 1000ull * 1000ull; // 1s in ns
  sA.publish_delegation("vol.brick", "B-cluster", "", kLease);
  EXPECT_GE(t.pump_shard(sA), 1u);

  // Within the lease window, both peers route remote.
  ck.set(kLease / 2);
  EXPECT_EQ(sA.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::remote);
  EXPECT_EQ(sB.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::remote);

  // Past the lease horizon, both peers report expired.
  ck.set(kLease + 1);
  EXPECT_EQ(sA.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::expired);
  EXPECT_EQ(sB.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::expired);
}

// ----------------
// AuthorityTransfer: the authoritative cluster for a path can be
// observed to migrate over time without any test code naming
// cluster_id at the route site.
// ----------------
TEST(StateDistributedDelegationIntegration, AuthorityTransfer) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "A", "A");
  cvc::state_cluster_shard sB(aB, "A", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  sA.publish_delegation("data.world.geometry", "renderCluster", "", 0);
  EXPECT_GE(t.pump_shard(sA), 1u);
  EXPECT_EQ(sB.delegation().route("data.world.geometry.mesh").cluster_id, "renderCluster");

  sA.publish_revocation("data.world.geometry");
  EXPECT_GE(t.pump_shard(sA), 1u);
  EXPECT_TRUE(sB.delegation().is_local("data.world.geometry.mesh"));

  sA.publish_delegation("data.world.geometry", "physicsCluster", "", 0);
  EXPECT_GE(t.pump_shard(sA), 1u);
  EXPECT_EQ(sB.delegation().route("data.world.geometry.mesh").cluster_id, "physicsCluster");
}

// ----------------
// With set_enforce_delegation(true), once a peer learns a subtree
// has been delegated to a foreign cluster, ordinary value writes
// to that subtree from a non-authoritative origin are rejected.
// ----------------
TEST(StateDistributedDelegationIntegration, EnforcementInteraction) {
  cvc::app aA, aB;
  cvc::state_transport_inproc t;
  cvc::state_cluster_shard sA(aA, "A", "A");
  cvc::state_cluster_shard sB(aB, "A", "B");
  sA.attach();
  sB.attach();
  t.register_shard(&sA);
  t.register_shard(&sB);

  sB.set_enforce_delegation(true);

  sA.publish_delegation("vol.brick", "remoteCluster", "", 0);
  EXPECT_GE(t.pump_shard(sA), 1u);
  ASSERT_EQ(sB.delegation().route("vol.brick.x").kind,
            cvc::state_delegation_manager::route_kind::remote);

  // Now A tries to push an ordinary value-write into the delegated
  // subtree; B must reject it because the path is not local.
  cvc::state_mutation m;
  m.cluster_id = "A";
  m.origin_node_id = "A";
  m.sequence = 99;
  m.path = "vol.brick.x";
  m.op = cvc::state_mutation_op::set_value;
  m.string_value = "v";
  m.type_name = "std::string";
  auto r = sB.ingest_remote(m);
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_GE(sB.total_delegation_routed(), 1u);
}
