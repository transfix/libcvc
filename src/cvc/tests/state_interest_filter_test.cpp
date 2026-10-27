/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 4a: receiver-side interest filter.
//
// Verifies that a state_cluster_shard whose enforce_interest() is
// true drops inbound mutations / messages whose paths are not
// covered by a registered interest prefix, while preserving the
// previous "mirror everything" behavior when the filter is off.

#include <atomic>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_message.h>
#include <cvc/state/state_message_bus.h>
#include <gtest/gtest.h>
#include <string>

namespace {

cvc::state_mutation make_set_value(const std::string &origin, std::uint64_t seq,
                                   const std::string &path, const std::string &val) {
  cvc::state_mutation m;
  m.cluster_id = "test-cluster";
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

TEST(StateInterestFilterTest, DefaultPermissiveMatchesPreviousBehavior) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();

  EXPECT_FALSE(sh.enforce_interest());
  EXPECT_TRUE(sh.interests().empty());

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "scene.foo", "v"));
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
  EXPECT_EQ(sh.total_remote_filtered_out(), 0u);
  EXPECT_EQ(cvc::state::instance(a)("scene.foo").value(), "v");
}

TEST(StateInterestFilterTest, EnforceWithEmptyInterestSetRejectsAll) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  std::size_t before = cvc::state::instance(a).numChildren();

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "scene.foo", "v"));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_NE(r.reject_reason.find("interest"), std::string::npos);
  EXPECT_EQ(sh.total_remote_filtered_out(), 1u);

  // Tree must NOT have auto-vivified the new path.
  EXPECT_EQ(cvc::state::instance(a).numChildren(), before);
}

TEST(StateInterestFilterTest, MatchingPrefixIsAdmitted) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("scene");

  auto r1 = sh.ingest_remote(make_set_value("nodeB", 1, "scene", "root"));
  EXPECT_TRUE(r1.applied) << r1.reject_reason;

  auto r2 = sh.ingest_remote(make_set_value("nodeB", 2, "scene.foo", "v"));
  EXPECT_TRUE(r2.applied) << r2.reject_reason;
  EXPECT_EQ(sh.total_remote_filtered_out(), 0u);
  EXPECT_EQ(cvc::state::instance(a)("scene.foo").value(), "v");
}

TEST(StateInterestFilterTest, SiblingPathIsRejected) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("scene.lights");

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "scene.cameras", "cam"));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_EQ(sh.total_remote_filtered_out(), 1u);
}

TEST(StateInterestFilterTest, DotBoundaryPreventsScenerySpoof) {
  // "scene" must NOT match "scenery": prefix matching is
  // dot-segment-aware.
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("scene");

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "scenery.path", "x"));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
  EXPECT_EQ(sh.total_remote_filtered_out(), 1u);
}

TEST(StateInterestFilterTest, EmptyPrefixMatchesEverything) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("");

  auto r1 = sh.ingest_remote(make_set_value("nodeB", 1, "a.b.c", "v"));
  EXPECT_TRUE(r1.applied);
  auto r2 = sh.ingest_remote(make_set_value("nodeB", 2, "x", "y"));
  EXPECT_TRUE(r2.applied);
  EXPECT_EQ(sh.total_remote_filtered_out(), 0u);
}

TEST(StateInterestFilterTest, RemoveInterestStopsAccepting) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("scene");
  EXPECT_TRUE(sh.remove_interest("scene"));

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "scene.foo", "v"));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
}

TEST(StateInterestFilterTest, ClearInterestsRejectsAllWhenEnforced) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("a");
  sh.add_interest("b");
  sh.clear_interests();
  EXPECT_TRUE(sh.interests().empty());

  auto r = sh.ingest_remote(make_set_value("nodeB", 1, "a.x", "v"));
  EXPECT_FALSE(r.applied);
  EXPECT_TRUE(r.rejected);
}

TEST(StateInterestFilterTest, FilteredMutationCanLandAfterAddingInterest) {
  // Crucially the seen-set is NOT advanced when we filter out, so
  // a re-publish after add_interest must succeed.
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);

  auto m = make_set_value("nodeB", 7, "lazy.path", "v");
  auto r1 = sh.ingest_remote(m);
  EXPECT_FALSE(r1.applied);
  EXPECT_TRUE(r1.rejected);

  sh.add_interest("lazy");
  auto r2 = sh.ingest_remote(m);
  EXPECT_TRUE(r2.applied) << r2.reject_reason;
  EXPECT_FALSE(r2.duplicate);
  EXPECT_EQ(cvc::state::instance(a)("lazy.path").value(), "v");
}

TEST(StateInterestFilterTest, AddInterestIsIdempotent) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.add_interest("scene");
  sh.add_interest("scene");
  sh.add_interest(".scene."); // gets normalized
  auto v = sh.interests();
  ASSERT_EQ(v.size(), 1u);
  EXPECT_EQ(v[0], "scene");
}

TEST(StateInterestFilterTest, IngestRemoteMessageRespectsFilter) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("scene");

  std::atomic<int> hits{0};
  sh.message_bus().subscribe("", [&](const cvc::state_message &) { hits.fetch_add(1); });

  cvc::state_message in_msg;
  in_msg.cluster_id = "clusterA";
  in_msg.origin_node_id = "nodeB";
  in_msg.message_id = "m1";
  in_msg.path = "scene.event";
  EXPECT_TRUE(sh.ingest_remote_message(in_msg));
  EXPECT_EQ(hits.load(), 1);

  cvc::state_message out_msg = in_msg;
  out_msg.message_id = "m2";
  out_msg.path = "logs.event";
  EXPECT_FALSE(sh.ingest_remote_message(out_msg));
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(sh.total_remote_filtered_out(), 1u);
}

// A relay asks admits() / admits_message() whether its own shard would take a frame before
// re-fanning it, so each answer must match what ingest then does with the same frame.
TEST(StateInterestFilterTest, AdmitsMirrorsIngestGate) {
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true);
  sh.add_interest("conf");

  const auto check = [&](const cvc::state_mutation &m, bool want) {
    EXPECT_EQ(sh.admits(m), want) << m.path;
    auto r = sh.ingest_remote(m);
    EXPECT_EQ(r.applied, want) << m.path << ": " << r.reject_reason;
    EXPECT_EQ(r.rejected, !want) << m.path;
  };

  check(make_set_value("nodeB", 1, "conf.rooms.r.y", "v"), true);
  check(make_set_value("nodeB", 2, "ui.docs.x", "v"), false);
  check(make_set_value("nodeB", 3, "conference.x", "v"), false); // dot-segment boundary

  // Delegation ops pass whatever their path, as on ingest's control-plane fast path.
  cvc::state_mutation d = make_set_value("nodeB", 4, "ui.docs", "");
  d.op = cvc::state_mutation_op::delegate_subtree;
  d.string_value = "clusterB";
  check(d, true);
  cvc::state_mutation rv = make_set_value("nodeB", 5, "ui.docs", "");
  rv.op = cvc::state_mutation_op::revoke_delegation;
  check(rv, true);

  // With enforcement off everything is admitted, as ingest then takes everything.
  sh.set_enforce_interest(false);
  check(make_set_value("nodeB", 6, "ui.docs.x", "v"), true);

  // Messages: the cluster check, then interest.
  sh.set_enforce_interest(true);
  const auto check_message = [&](const std::string &id, const std::string &cluster,
                                 const std::string &path, bool want) {
    cvc::state_message msg;
    msg.cluster_id = cluster;
    msg.origin_node_id = "nodeB";
    msg.message_id = id;
    msg.path = path;
    EXPECT_EQ(sh.admits_message(msg), want) << id;
    EXPECT_EQ(sh.ingest_remote_message(msg), want) << id;
  };
  check_message("m1", "clusterA", "conf.event", true);
  check_message("m2", "clusterA", "ui.event", false);
  check_message("m3", "clusterB", "conf.event", false);
  check_message("m4", "", "conf.event", true); // an unstamped message is this cluster's
  sh.set_enforce_interest(false);
  check_message("m5", "clusterA", "ui.event", true);
  check_message("m6", "clusterB", "ui.event", false); // the cluster check is not interest
}

TEST(StateInterestFilterTest, DelegationOpsBypassInterestFilter) {
  // Control-plane mutations are routing metadata, not value
  // writes; they must always flow regardless of interest set.
  cvc::app a;
  cvc::state_cluster_shard sh(a, "clusterA", "nodeA");
  sh.attach();
  sh.set_enforce_interest(true); // no interests => block all data

  cvc::state_mutation d;
  d.cluster_id = "clusterA";
  d.tree_id = "default";
  d.origin_node_id = "nodeB";
  d.sequence = 1;
  d.mutation_id = "nodeB:1";
  d.path = "off.limits";
  d.op = cvc::state_mutation_op::delegate_subtree;
  d.string_value = "clusterB"; // owner cluster
  auto r = sh.ingest_remote(d);
  EXPECT_TRUE(r.applied) << r.reject_reason;
  EXPECT_EQ(sh.total_remote_filtered_out(), 0u);
}
