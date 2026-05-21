/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cstdint>
#include <cvc/state_delegation_manager.h>
#include <gtest/gtest.h>

using cvc::state_delegation_manager;

namespace {

// Mock clock advancing only when the test asks. Pointer-based so a
// single shared counter can drive a lambda passed to set_clock.
struct mock_clock {
  std::uint64_t now = 0;
  state_delegation_manager::clock_fn fn() {
    return [this]() { return now; };
  }
};

} // namespace

TEST(StateDelegationManager, NoDelegationsClassifiesEverythingLocal) {
  state_delegation_manager m("self");
  auto r = m.route("anything.at.all");
  EXPECT_EQ(r.kind, state_delegation_manager::route_kind::local);
  EXPECT_TRUE(r.cluster_id.empty());
  EXPECT_TRUE(r.matched_prefix.empty());
}

TEST(StateDelegationManager, ForeignClusterWithoutLeaseIsRemote) {
  state_delegation_manager m("self");
  m.delegate("sim", "other");
  auto r = m.route("sim.volume.x");
  EXPECT_EQ(r.kind, state_delegation_manager::route_kind::remote);
  EXPECT_EQ(r.cluster_id, "other");
  EXPECT_EQ(r.matched_prefix, "sim");
  EXPECT_EQ(r.expires_at_ns, 0u);
}

TEST(StateDelegationManager, OwnClusterDelegationIsLocal) {
  state_delegation_manager m("self");
  m.delegate("only.mine", "self", "grpc://localhost:1");
  auto r = m.route("only.mine.x");
  EXPECT_EQ(r.kind, state_delegation_manager::route_kind::local);
  EXPECT_EQ(r.cluster_id, "self");
}

TEST(StateDelegationManager, LongestPrefixWins) {
  state_delegation_manager m("self");
  m.delegate("a", "shallow");
  m.delegate("a.b.c", "deep");

  EXPECT_EQ(m.route("a.b.c.d").cluster_id, "deep");
  EXPECT_EQ(m.route("a.b").cluster_id, "shallow");
  EXPECT_EQ(m.route("a.x").cluster_id, "shallow");
}

TEST(StateDelegationManager, RootDelegationCoversEverything) {
  state_delegation_manager m("self");
  m.delegate("", "umbrella");
  EXPECT_EQ(m.route("anything").cluster_id, "umbrella");
  EXPECT_EQ(m.route("nested.path.here").cluster_id, "umbrella");
}

TEST(StateDelegationManager, DotBoundaryRequiredForPrefixMatch) {
  state_delegation_manager m("self");
  m.delegate("sim", "other");
  // "simulation" must NOT match "sim".
  EXPECT_EQ(m.route("simulation.x").kind, state_delegation_manager::route_kind::local);
  // exact match still routes
  EXPECT_EQ(m.route("sim").kind, state_delegation_manager::route_kind::remote);
}

TEST(StateDelegationManager, LeaseExpiresAndIsClassifiedExpired) {
  mock_clock clk;
  clk.now = 1000;
  state_delegation_manager m("self", clk.fn());
  m.delegate("simulation", "other", "grpc://h:1", /*lease=*/500);

  // Just before expiry.
  clk.now = 1499;
  EXPECT_EQ(m.route("simulation.v").kind, state_delegation_manager::route_kind::remote);

  // Past expiry.
  clk.now = 1501;
  auto r = m.route("simulation.v");
  EXPECT_EQ(r.kind, state_delegation_manager::route_kind::expired);
  EXPECT_EQ(r.cluster_id, "other");
}

TEST(StateDelegationManager, RenewExtendsLease) {
  mock_clock clk;
  clk.now = 1000;
  state_delegation_manager m("self", clk.fn());
  m.delegate("p", "other", "", 500);
  clk.now = 1600; // expired
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::expired);
  // Renew at clk=1600 for another 500ns.
  ASSERT_TRUE(m.renew("p", 500));
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::remote);
  clk.now = 2200;
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::expired);
}

TEST(StateDelegationManager, RenewFailsWhenNoEntry) {
  state_delegation_manager m("self");
  EXPECT_FALSE(m.renew("nonexistent", 100));
}

TEST(StateDelegationManager, RevokeRemovesEntry) {
  state_delegation_manager m("self");
  m.delegate("p", "other");
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::remote);
  EXPECT_TRUE(m.revoke("p"));
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::local);
  EXPECT_FALSE(m.revoke("p"));
}

TEST(StateDelegationManager, AuthorityTransferReassignsCluster) {
  state_delegation_manager m("self");
  m.delegate("p", "first");
  EXPECT_EQ(m.route("p.x").cluster_id, "first");
  m.delegate("p", "second"); // re-delegate same prefix
  EXPECT_EQ(m.route("p.x").cluster_id, "second");
}

TEST(StateDelegationManager, InfiniteLeaseNeverExpires) {
  mock_clock clk;
  clk.now = 0;
  state_delegation_manager m("self", clk.fn());
  m.delegate("p", "other", "", /*lease=*/0);
  clk.now = std::uint64_t(1) << 62;
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::remote);
}

TEST(StateDelegationManager, RenewToInfiniteClearsExpiry) {
  mock_clock clk;
  clk.now = 1000;
  state_delegation_manager m("self", clk.fn());
  m.delegate("p", "other", "", 500);
  ASSERT_TRUE(m.renew("p", 0));
  clk.now = std::uint64_t(1) << 62;
  EXPECT_EQ(m.route("p.x").kind, state_delegation_manager::route_kind::remote);
}

TEST(StateDelegationManager, EndpointPropagatesThroughRouteDecision) {
  state_delegation_manager m("self");
  m.delegate("p", "other", "grpc://h:9");
  auto r = m.route("p.deep.path");
  EXPECT_EQ(r.endpoint, "grpc://h:9");
}

TEST(StateDelegationManager, IsLocalConvenience) {
  state_delegation_manager m("self");
  m.delegate("foreign", "other");
  EXPECT_TRUE(m.is_local("home.x"));
  EXPECT_FALSE(m.is_local("foreign.x"));
}
