/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 3: link-graph static analyzer
// (state_distributed_admin::link_cycles).
//
// Verifies cycle detection over the link graph rooted at a state
// tree. The analyzer must:
//   - return no cycles when no link nodes form a closed loop;
//   - detect self-loops (single-element cycles);
//   - detect 2-link and longer cycles;
//   - enumerate multiple disjoint cycles in one call;
//   - ignore broken links (target missing) and links to non-link
//     terminal nodes;
//   - produce stable, deterministic ordering across runs.

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_distributed_admin.h>

#include <gtest/gtest.h>

#include <algorithm>

using cvc::state;
using cvc::state_distributed_admin;

namespace {

bool contains_cycle(const state_distributed_admin::link_cycles_result &r,
                    const std::vector<std::string> &expected) {
  for (const auto &c : r.cycles) {
    if (c.size() != expected.size())
      continue;
    // The analyzer rotates each cycle to start at the
    // lexicographically smallest path, so an expected cycle is a
    // match iff it equals one of the rotations of the reported
    // cycle. The expected vector is constructed already rotated.
    if (c == expected)
      return true;
  }
  return false;
}

} // namespace

TEST(StateLinkCycles, EmptyTreeYieldsNoCycles) {
  cvc::app a;
  auto &root = state::instance(a);
  auto r = state_distributed_admin::link_cycles(root);
  EXPECT_EQ(r.cycles.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 0u);
}

TEST(StateLinkCycles, NoLinksYieldsNoCycles) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene.geometry");
  root("data.world.geometry").value(std::string("v"));

  auto r = state_distributed_admin::link_cycles(root);
  EXPECT_EQ(r.cycles.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 0u);
}

TEST(StateLinkCycles, LinkToTerminalIsNotACycle) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  root("scene.geometry").linkTo("data.world.geometry");

  auto r = state_distributed_admin::link_cycles(root);
  EXPECT_EQ(r.cycles.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateLinkCycles, BrokenLinkIsNotACycle) {
  cvc::app a;
  auto &root = state::instance(a);
  root("scene.geometry").linkTo("missing.target.path");

  auto r = state_distributed_admin::link_cycles(root);
  EXPECT_EQ(r.cycles.size(), 0u);
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateLinkCycles, SelfLoopIsReported) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");
  n.linkTo("scene.geometry");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  ASSERT_EQ(r.cycles[0].size(), 1u);
  EXPECT_EQ(r.cycles[0][0], "scene.geometry");
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateLinkCycles, TwoLinkCycleIsReported) {
  cvc::app a;
  auto &root = state::instance(a);
  // a -> b, b -> a
  root("scene.a").linkTo("scene.b");
  root("scene.b").linkTo("scene.a");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  // Rotated to start at lexicographically smallest: scene.a.
  EXPECT_TRUE(contains_cycle(r, {"scene.a", "scene.b"}));
  EXPECT_EQ(r.link_nodes_scanned, 2u);
}

TEST(StateLinkCycles, ThreeLinkCycleIsReportedInTraversalOrder) {
  cvc::app a;
  auto &root = state::instance(a);
  // a -> b -> c -> a
  root("scene.a").linkTo("scene.b");
  root("scene.b").linkTo("scene.c");
  root("scene.c").linkTo("scene.a");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  EXPECT_TRUE(contains_cycle(r, {"scene.a", "scene.b", "scene.c"}));
  EXPECT_EQ(r.link_nodes_scanned, 3u);
}

TEST(StateLinkCycles, MultipleDisjointCyclesAreEnumerated) {
  cvc::app a;
  auto &root = state::instance(a);
  // Cycle 1: x -> y -> x
  root("first.x").linkTo("first.y");
  root("first.y").linkTo("first.x");
  // Cycle 2: p -> q -> r -> p
  root("second.p").linkTo("second.q");
  root("second.q").linkTo("second.r");
  root("second.r").linkTo("second.p");
  // Plus a non-cycle link to a terminal.
  root("data.terminal").value(std::string("v"));
  root("third.acyclic").linkTo("data.terminal");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 2u);
  EXPECT_TRUE(contains_cycle(r, {"first.x", "first.y"}));
  EXPECT_TRUE(contains_cycle(r, {"second.p", "second.q", "second.r"}));
  EXPECT_EQ(r.link_nodes_scanned, 6u);

  // Stable ordering: cycles are sorted by their first element.
  ASSERT_GE(r.cycles.size(), 2u);
  EXPECT_LT(r.cycles[0].front(), r.cycles[1].front());
}

TEST(StateLinkCycles, CycleAtRootSubtreeIsHandled) {
  cvc::app a;
  auto &root = state::instance(a);
  // Pure two-cycle at top level: nodes "alpha" and "beta" each
  // have empty parent name (root), so their fullName is just
  // "alpha" and "beta".
  root("alpha").linkTo("beta");
  root("beta").linkTo("alpha");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  EXPECT_TRUE(contains_cycle(r, {"alpha", "beta"}));
}

TEST(StateLinkCycles, NonLinkBranchesDoNotInflateScanCount) {
  cvc::app a;
  auto &root = state::instance(a);
  // Many non-link nodes, only two are links forming a cycle.
  for (int i = 0; i < 16; ++i) {
    std::string p = "tree.node_" + std::to_string(i);
    root(p).value(std::string("v"));
  }
  root("ring.x").linkTo("ring.y");
  root("ring.y").linkTo("ring.x");

  auto r = state_distributed_admin::link_cycles(root);
  EXPECT_EQ(r.link_nodes_scanned, 2u);
  ASSERT_EQ(r.cycles.size(), 1u);
  EXPECT_TRUE(contains_cycle(r, {"ring.x", "ring.y"}));
}

TEST(StateLinkCycles, TailLinkIntoCycleIsNotItselfACycleMember) {
  cvc::app a;
  auto &root = state::instance(a);
  // Cycle: a -> b -> a. Tail link c -> a feeds the cycle but is
  // not part of it (out-degree 1, no edge back into c).
  root("ring.a").linkTo("ring.b");
  root("ring.b").linkTo("ring.a");
  root("tail.c").linkTo("ring.a");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  EXPECT_TRUE(contains_cycle(r, {"ring.a", "ring.b"}));
  EXPECT_EQ(r.link_nodes_scanned, 3u);
}

TEST(StateLinkCycles, RootSelfLinkIsReportedAsSingleElementCycle) {
  cvc::app a;
  auto &root = state::instance(a);
  root.linkTo(".");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  ASSERT_EQ(r.cycles[0].size(), 1u);
  EXPECT_EQ(r.cycles[0][0], root.fullName());
  EXPECT_EQ(r.link_nodes_scanned, 1u);
}

TEST(StateLinkCycles, CycleThroughRootIsDetected) {
  cvc::app a;
  auto &root = state::instance(a);
  // root -> scene.a -> root
  root.linkTo("scene.a");
  root("scene.a").linkTo(".");

  auto r = state_distributed_admin::link_cycles(root);
  ASSERT_EQ(r.cycles.size(), 1u);
  ASSERT_EQ(r.cycles[0].size(), 2u);
  // Rotated to start at lex-smallest; root.fullName() is "" which
  // sorts before "scene.a", so root path comes first.
  EXPECT_EQ(r.cycles[0][0], root.fullName());
  EXPECT_EQ(r.cycles[0][1], "scene.a");
  EXPECT_EQ(r.link_nodes_scanned, 2u);
}
