/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 slice 4b: link visibility mode (opaque vs transparent)
// and resolvedValue() read-through helper.

#include <cvc/app.h>
#include <cvc/state.h>

#include <gtest/gtest.h>

TEST(StateLinkModeTest, DefaultsToOpaque) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta");
  EXPECT_TRUE(n.isLink());
  EXPECT_EQ(n.linkMode(), cvc::state::link_mode::opaque);
}

TEST(StateLinkModeTest, LinkToWithModeRecordsBoth) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta", cvc::state::link_mode::transparent);
  EXPECT_TRUE(n.isLink());
  EXPECT_EQ(n.linkTarget(), "beta");
  EXPECT_EQ(n.linkMode(), cvc::state::link_mode::transparent);
}

TEST(StateLinkModeTest, SetLinkModeFiresLinkChanged) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta");
  int fired = 0;
  n.linkChanged.connect([&]() { ++fired; });
  n.setLinkMode(cvc::state::link_mode::transparent);
  EXPECT_EQ(fired, 1);
  // Idempotent: same mode does not refire.
  n.setLinkMode(cvc::state::link_mode::transparent);
  EXPECT_EQ(fired, 1);
}

TEST(StateLinkModeTest, ClearLinkResetsModeToOpaque) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta", cvc::state::link_mode::transparent);
  EXPECT_TRUE(n.clearLink());
  EXPECT_FALSE(n.isLink());
  EXPECT_EQ(n.linkMode(), cvc::state::link_mode::opaque);
}

TEST(StateLinkModeTest, ResolvedValueOnNonLinkReturnsOwnValue) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.value(std::string("own"));
  EXPECT_EQ(n.resolvedValue(), "own");
}

TEST(StateLinkModeTest, ResolvedValueOnOpaqueLinkReturnsOwnValue) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.value(std::string("own"));
  n.linkTo("target");  // opaque by default
  EXPECT_EQ(n.resolvedValue(), "own");
}

TEST(StateLinkModeTest, ResolvedValueOnTransparentLinkFollowsTarget) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.value(std::string("own"));
  n.linkTo("target", cvc::state::link_mode::transparent);
  EXPECT_EQ(n.resolvedValue(), "target-val");
}

TEST(StateLinkModeTest, ResolvedValueFollowsChainOfTransparentLinks) {
  cvc::app a;
  cvc::state::instance(a)("c").value(std::string("c-val"));
  cvc::state::instance(a)("b").linkTo("c",
                                       cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").linkTo("b",
                                       cvc::state::link_mode::transparent);
  EXPECT_EQ(cvc::state::instance(a)("a").resolvedValue(), "c-val");
}

TEST(StateLinkModeTest, ResolvedValueOnBrokenLinkFallsBackToOwnValue) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.value(std::string("own"));
  n.linkTo("does.not.exist", cvc::state::link_mode::transparent);
  EXPECT_EQ(n.resolvedValue(), "own");
}

TEST(StateLinkModeTest, ResolvedValueOnSelfCycleFallsBackToOwnValue) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.value(std::string("own"));
  n.linkTo("alpha", cvc::state::link_mode::transparent);
  EXPECT_EQ(n.resolvedValue(), "own");
}

TEST(StateLinkModeTest, ResolvedValueRespectsHopBudget) {
  cvc::app a;
  cvc::state::instance(a)("c").value(std::string("c-val"));
  cvc::state::instance(a)("b").linkTo("c",
                                       cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").linkTo("b",
                                       cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").value(std::string("own"));
  // Budget exhausted before terminal -> fall back to own value.
  EXPECT_EQ(cvc::state::instance(a)("a").resolvedValue(/*hop_budget=*/0),
            "own");
}

TEST(StateLinkModeTest, ChangingTargetWithLinkToOverloadFiresOnce) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  int fired = 0;
  n.linkChanged.connect([&]() { ++fired; });
  n.linkTo("beta", cvc::state::link_mode::transparent);
  EXPECT_EQ(fired, 1);
  EXPECT_EQ(n.linkTarget(), "beta");
  EXPECT_EQ(n.linkMode(), cvc::state::link_mode::transparent);
}
