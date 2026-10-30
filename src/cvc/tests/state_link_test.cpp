/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 1: link-node API and local link resolver.
//
// These tests exercise the cluster-agnostic link surface added to
// state: linkTo/clearLink/isLink/linkTarget plus resolveLink with
// hop budget, broken-link detection, and cycle detection. Cluster
// routing through the authority map and OOB sendMessage land in
// the next slice.

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>

namespace {

using cvc::state;
using kind_t = state::link_resolution_kind;

cvc::app &fresh_app() {
  static thread_local cvc::app *ctx = nullptr;
  ctx = new cvc::app(); // leak intentional: gtest fixtures cycle in seconds
  return *ctx;
}

} // namespace

TEST(StateLink, NodeIsNotALinkByDefault) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");
  EXPECT_FALSE(n.isLink());
  EXPECT_TRUE(n.linkTarget().empty());

  auto r = n.resolveLink();
  EXPECT_EQ(r.kind, kind_t::none);
  EXPECT_EQ(r.target, &n);
  EXPECT_EQ(r.hops, 0u);
}

TEST(StateLink, LinkToResolvesToTargetNode) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &target = root("data.world.geometry");
  target.value(std::string("payload"));
  auto &link = root("scene.geometry");

  link.linkTo("data.world.geometry");
  EXPECT_TRUE(link.isLink());
  EXPECT_EQ(link.linkTarget(), "data.world.geometry");

  auto r = link.resolveLink();
  EXPECT_EQ(r.kind, kind_t::resolved);
  EXPECT_EQ(r.target, &target);
  EXPECT_EQ(r.hops, 1u);
  ASSERT_GE(r.visited.size(), 2u);
  EXPECT_EQ(r.visited.front(), "scene.geometry");
  EXPECT_EQ(r.visited.back(), "data.world.geometry");
}

TEST(StateLink, LinkToNormalizesPath) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry").value(std::string("v"));
  auto &link = root("scene.geometry");

  link.linkTo("  .data..world.geometry. ");
  EXPECT_EQ(link.linkTarget(), "data.world.geometry");
  EXPECT_EQ(link.resolveLink().kind, kind_t::resolved);
}

TEST(StateLink, ClearLinkRemovesLinkMark) {
  cvc::app a;
  auto &root = state::instance(a);
  root("data.world.geometry");
  auto &link = root("scene.geometry");
  link.linkTo("data.world.geometry");
  ASSERT_TRUE(link.isLink());

  EXPECT_TRUE(link.clearLink());
  EXPECT_FALSE(link.isLink());
  EXPECT_TRUE(link.linkTarget().empty());

  // Idempotent: a second clear is a no-op and reports false.
  EXPECT_FALSE(link.clearLink());
}

TEST(StateLink, BrokenLinkIsReported) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &link = root("scene.geometry");
  link.linkTo("data.world.geometry"); // target does not exist

  auto r = link.resolveLink();
  EXPECT_EQ(r.kind, kind_t::broken);
  EXPECT_EQ(r.target, nullptr);
  // findDescendant must NOT have created the missing node.
  EXPECT_EQ(root.findDescendant("data.world.geometry"), nullptr);
}

TEST(StateLink, ChainOfLinksResolvesToTerminal) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &terminal = root("c.real");
  terminal.value(std::string("end"));

  root("a").linkTo("b");
  root("b").linkTo("c.real");

  auto r = root("a").resolveLink();
  EXPECT_EQ(r.kind, kind_t::resolved);
  EXPECT_EQ(r.target, &terminal);
  EXPECT_EQ(r.hops, 2u);
  ASSERT_EQ(r.visited.size(), 3u);
  EXPECT_EQ(r.visited[0], "a");
  EXPECT_EQ(r.visited[1], "b");
  EXPECT_EQ(r.visited[2], "c.real");
}

TEST(StateLink, SelfLinkIsCycleDetected) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("loop");
  n.linkTo("loop");

  auto r = n.resolveLink();
  EXPECT_EQ(r.kind, kind_t::cycle_detected);
  EXPECT_EQ(r.target, nullptr);
  EXPECT_GE(r.hops, 1u);
}

TEST(StateLink, TwoNodeCycleIsDetected) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p").linkTo("q");
  root("q").linkTo("p");

  auto r = root("p").resolveLink();
  EXPECT_EQ(r.kind, kind_t::cycle_detected);
  EXPECT_EQ(r.target, nullptr);
  // The cycle is detected on the second hop returning to "p".
  EXPECT_GE(r.visited.size(), 3u);
  EXPECT_EQ(r.visited.front(), "p");
  EXPECT_EQ(r.visited.back(), "p");
}

TEST(StateLink, HopBudgetExhaustionBeatsCycle) {
  cvc::app a;
  auto &root = state::instance(a);
  // Linear chain a0 -> a1 -> a2 -> ... -> a9 (terminal). Budget 3
  // stops before reaching a9 and is reported as budget_exhausted,
  // not cycle_detected (no path is revisited).
  for (int i = 0; i < 9; ++i) {
    std::string from = "a" + std::to_string(i);
    std::string to = "a" + std::to_string(i + 1);
    root(from).linkTo(to);
  }
  root("a9").value(std::string("terminal"));

  auto r = root("a0").resolveLink(/*hop_budget=*/3);
  EXPECT_EQ(r.kind, kind_t::budget_exhausted);
  EXPECT_EQ(r.target, nullptr);
  EXPECT_EQ(r.hops, 3u);
}

TEST(StateLink, ResolveLinkDoesNotCreateMissingNodes) {
  cvc::app a;
  auto &root = state::instance(a);
  root("real").linkTo("ghost.subpath");

  ASSERT_EQ(root.findDescendant("ghost.subpath"), nullptr);
  auto r = root("real").resolveLink();
  EXPECT_EQ(r.kind, kind_t::broken);
  EXPECT_EQ(root.findDescendant("ghost.subpath"), nullptr);
  EXPECT_EQ(root.findDescendant("ghost"), nullptr);
}

TEST(StateLink, FindDescendantHandlesEmptyAndRoot) {
  cvc::app a;
  auto &root = state::instance(a);
  EXPECT_EQ(root.findDescendant(""), &root);
  EXPECT_EQ(root.findDescendant("."), &root);
  EXPECT_EQ(root.findDescendant("missing.path"), nullptr);
}

TEST(StateLink, LinkToRootViaDotMarker) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");
  n.linkTo(".");
  EXPECT_TRUE(n.isLink());
  EXPECT_EQ(n.linkTarget(), ".");

  auto r = n.resolveLink();
  EXPECT_EQ(r.kind, kind_t::resolved);
  EXPECT_EQ(r.target, &root);
  EXPECT_EQ(r.hops, 1u);
  ASSERT_GE(r.visited.size(), 2u);
  EXPECT_EQ(r.visited.front(), "scene.geometry");
  EXPECT_EQ(r.visited.back(), root.fullName()); // typically ""
}

TEST(StateLink, LinkToRootSeparatorOnlyVariantsNormalizeToDot) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");

  // All separator-only inputs canonicalize to ".".
  n.linkTo("..");
  EXPECT_EQ(n.linkTarget(), ".");
  n.linkTo("  .  ");
  EXPECT_EQ(n.linkTarget(), ".");
  n.linkTo("...");
  EXPECT_EQ(n.linkTarget(), ".");
}

TEST(StateLink, LinkToEmptyStringStillClears) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("scene.geometry");
  n.linkTo("data.target");
  ASSERT_TRUE(n.isLink());

  // Empty / whitespace-only input is "clear", not "link to root".
  n.linkTo("");
  EXPECT_FALSE(n.isLink());
  EXPECT_TRUE(n.linkTarget().empty());

  n.linkTo("data.target");
  ASSERT_TRUE(n.isLink());
  n.linkTo("   ");
  EXPECT_FALSE(n.isLink());
}

TEST(StateLink, RootLinkChainResolvesThroughRoot) {
  cvc::app a;
  auto &root = state::instance(a);
  // a -> root -> (root is not a link, terminal)
  auto &an = root("scene.a");
  an.linkTo(".");

  auto r = an.resolveLink();
  EXPECT_EQ(r.kind, kind_t::resolved);
  EXPECT_EQ(r.target, &root);
}

TEST(StateLink, RootSelfLinkIsCycle) {
  cvc::app a;
  auto &root = state::instance(a);
  // Make root itself a link pointing at root.
  root.linkTo(".");
  ASSERT_TRUE(root.isLink());

  auto r = root.resolveLink();
  EXPECT_EQ(r.kind, kind_t::cycle_detected);
  EXPECT_EQ(r.target, nullptr);
}

// findThroughLinks / childThroughLinks: a path spelled THROUGH a transparent link on an interior
// node reaches the target's descendants (an Ariadne `link:` hole on a subtree).

TEST(StateLinkPath, FindDescendantStopsAtTheLinkNode) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("1"));
  root("hole").linkTo("s", state::link_mode::transparent);
  // The physical walk sees only the link node's own (absent) child.
  EXPECT_EQ(root.findDescendant("hole.a"), nullptr);
}

TEST(StateLinkPath, FindThroughLinksReachesTheTargetsDescendants) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("1"));
  root("s.b.c").value(std::string("2"));
  root("hole").linkTo("s", state::link_mode::transparent);
  EXPECT_EQ(root.findThroughLinks("hole.a"), &root("s.a"));
  EXPECT_EQ(root.findThroughLinks("hole.b.c"), &root("s.b.c"));
  // The last hop is followed too: the hole itself reads as its target.
  EXPECT_EQ(root.findThroughLinks("hole"), &root("s"));
  EXPECT_EQ(root.findThroughLinks("hole.missing"), nullptr);
  // A read creates nothing, under the hole or the target.
  EXPECT_TRUE(root("hole").children().empty());
  EXPECT_EQ(root.findDescendant("s.missing"), nullptr);
}

TEST(StateLinkPath, FindThroughLinksIgnoresTheLinksOwnChildren) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("target"));
  root("hole.a").value(std::string("local"));
  root("hole").linkTo("s", state::link_mode::transparent);
  EXPECT_EQ(root.findThroughLinks("hole.a")->value(), "target");
}

TEST(StateLinkPath, OpaqueLinksAreNotFollowed) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("target"));
  root("hole.a").value(std::string("own"));
  root("hole").linkTo("s", state::link_mode::opaque);
  EXPECT_EQ(root.findThroughLinks("hole.a")->value(), "own");
  EXPECT_EQ(root.findThroughLinks("hole"), &root("hole"));
}

TEST(StateLinkPath, ABrokenLinkIsWalkedAsAPlainNode) {
  cvc::app a;
  auto &root = state::instance(a);
  root("hole.a").value(std::string("own"));
  root("hole").linkTo("no.such.node", state::link_mode::transparent);
  EXPECT_EQ(root.findThroughLinks("hole.a")->value(), "own");
}

TEST(StateLinkPath, ChainsAndLinksInsideTheTargetAreFollowed) {
  cvc::app a;
  auto &root = state::instance(a);
  root("c.x").value(std::string("deep"));
  root("b").linkTo("c", state::link_mode::transparent);
  root("hole").linkTo("b", state::link_mode::transparent);
  EXPECT_EQ(root.findThroughLinks("hole.x")->value(), "deep");
  // A transparent link inside the granted subtree is part of what it grants.
  root("s.inner").linkTo("c", state::link_mode::transparent);
  root("h2").linkTo("s", state::link_mode::transparent);
  EXPECT_EQ(root.findThroughLinks("h2.inner.x")->value(), "deep");
}

TEST(StateLinkPath, ACycleIsWalkedAsAPlainNode) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p").linkTo("q", state::link_mode::transparent);
  root("q").linkTo("p", state::link_mode::transparent);
  root("p.a").value(std::string("own"));
  EXPECT_EQ(root.findThroughLinks("p.a")->value(), "own");
}

TEST(StateLinkPath, AWritableLinkTakesDescendantWritesOnTheTarget) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("1"));
  root("hole").linkTo("s", state::link_mode::transparent).setLinkWritable(true);
  EXPECT_EQ(&root.childThroughLinks("hole.a"), &root("s.a"));
  root.childThroughLinks("hole.new.leaf").value(std::string("x"));
  ASSERT_NE(root.findDescendant("s.new.leaf"), nullptr);
  EXPECT_EQ(root.findDescendant("s.new.leaf")->value(), "x");
  EXPECT_TRUE(root("hole").children().empty()); // nothing landed on the hole
}

TEST(StateLinkPath, AReadOnlyLinkKeepsDescendantWritesLocal) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.a").value(std::string("1"));
  root("hole").linkTo("s", state::link_mode::transparent);
  root.childThroughLinks("hole.a").value(std::string("2"));
  root.childThroughLinks("hole.new").value(std::string("x"));
  EXPECT_EQ(root("s.a").value(), "1");
  EXPECT_EQ(root.findDescendant("s.new"), nullptr);
  EXPECT_EQ(root.findDescendant("hole.a")->value(), "2"); // the hole's own child
  // Reads still see the target.
  EXPECT_EQ(root.findThroughLinks("hole.a")->value(), "1");
  // Refusing instead: nothing is written anywhere.
  EXPECT_THROW(root.childThroughLinks("hole.b", true), std::exception);
  EXPECT_EQ(root.findDescendant("hole.b"), nullptr);
  EXPECT_EQ(root.findDescendant("s.b"), nullptr);
}

TEST(StateLinkPath, AnUnresolvableWritableLinkRefusesDescendantWrites) {
  cvc::app a;
  auto &root = state::instance(a);
  root("hole").linkTo("no.such.node", state::link_mode::transparent).setLinkWritable(true);
  EXPECT_THROW(root.childThroughLinks("hole.a"), std::exception);
  EXPECT_EQ(root.findDescendant("hole.a"), nullptr);
  EXPECT_EQ(root.findDescendant("no.such.node"), nullptr);
}

TEST(StateLinkPath, TheLastHopIsNotFollowedForAWrite) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s").value(std::string("1"));
  root("hole").linkTo("s", state::link_mode::transparent);
  // value(v) decides for the link node itself: a read-only one keeps the write.
  EXPECT_EQ(&root.childThroughLinks("hole"), &root("hole"));
  root.childThroughLinks("hole").value(std::string("2"));
  EXPECT_EQ(root("s").value(), "1");
}
