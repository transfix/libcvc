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

#include <cvc/app.h>
#include <cvc/state.h>

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
