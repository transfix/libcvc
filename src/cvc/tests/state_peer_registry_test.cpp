/*
  Copyright 2026 The University of Texas at Austin
  Phase 5 — state_peer_registry tests.
*/

#include <cvc/state_peer_registry.h>

#include <gtest/gtest.h>

using cvc::state_peer_registry;

TEST(StatePeerRegistry, PrefixMatchesEmptyMatchesAll) {
  EXPECT_TRUE(state_peer_registry::prefix_matches("", "anything"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("", ""));
}

TEST(StatePeerRegistry, PrefixMatchesExactAndDotBoundary) {
  EXPECT_TRUE(state_peer_registry::prefix_matches("a", "a"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("a", "a.b"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("a.b", "a.b.c"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a", "ab"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a.b", "a.bc"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a.b", "a"));
}

TEST(StatePeerRegistry, AnyPrefixMatchesEmptyListIsMatchAll) {
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({}, "x.y"));
}

TEST(StatePeerRegistry, AnyPrefixMatchesNonEmptyFilters) {
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({"a", "x"}, "a.b"));
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({"a", "x"}, "x"));
  EXPECT_FALSE(state_peer_registry::any_prefix_matches({"a", "x"}, "y"));
}

TEST(StatePeerRegistry, AddRemoveHas) {
  state_peer_registry r;
  EXPECT_FALSE(r.has_peer("n1"));
  r.add_peer("n1", "c", "addr");
  EXPECT_TRUE(r.has_peer("n1"));
  EXPECT_EQ(1u, r.size());
  // re-add overwrites; size stays 1.
  r.add_peer("n1", "c", "addr");
  EXPECT_EQ(1u, r.size());
  EXPECT_TRUE(r.remove_peer("n1"));
  EXPECT_FALSE(r.remove_peer("n1"));
  EXPECT_FALSE(r.has_peer("n1"));
  EXPECT_EQ(0u, r.size());
}

TEST(StatePeerRegistry, ShouldDeliverUnknownPeerIsTrue) {
  state_peer_registry r;
  EXPECT_TRUE(r.should_deliver("unknown_node", "any.path"));
}

TEST(StatePeerRegistry, ShouldDeliverEmptySubscriptionsIsMatchAll) {
  state_peer_registry r;
  r.add_peer("n1", "c", "");
  EXPECT_TRUE(r.should_deliver("n1", "a.b"));
  EXPECT_TRUE(r.should_deliver("n1", ""));
}

TEST(StatePeerRegistry, ShouldDeliverFiltersByPrefix) {
  state_peer_registry r;
  r.add_peer("n1", "c", "");
  EXPECT_TRUE(r.set_subscriptions("n1", {"alpha", "beta.gamma"}));
  EXPECT_TRUE(r.should_deliver("n1", "alpha"));
  EXPECT_TRUE(r.should_deliver("n1", "alpha.x.y"));
  EXPECT_TRUE(r.should_deliver("n1", "beta.gamma.z"));
  EXPECT_FALSE(r.should_deliver("n1", "beta"));
  EXPECT_FALSE(r.should_deliver("n1", "alphax"));
  EXPECT_FALSE(r.should_deliver("n1", "gamma"));
}

TEST(StatePeerRegistry, CountersAndSnapshot) {
  state_peer_registry r;
  r.add_peer("n1", "c", "addr1");
  r.note_mutation_delivered("n1");
  r.note_mutation_delivered("n1");
  r.note_message_delivered("n1");
  r.note_delivery_filtered("n1");
  r.note_seen("n1", 12345);
  auto snap = r.snapshot();
  ASSERT_EQ(1u, snap.size());
  EXPECT_EQ("n1", snap[0].node_id);
  EXPECT_EQ(2u, snap[0].mutations_delivered);
  EXPECT_EQ(1u, snap[0].messages_delivered);
  EXPECT_EQ(1u, snap[0].deliveries_filtered);
  EXPECT_EQ(12345u, snap[0].last_seen_ns);
}

TEST(StatePeerRegistry, ClearWipesAll) {
  state_peer_registry r;
  r.add_peer("a", "c", "");
  r.add_peer("b", "c", "");
  r.clear();
  EXPECT_EQ(0u, r.size());
  EXPECT_FALSE(r.has_peer("a"));
}
