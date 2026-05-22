/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8: writable transparent links route writes to the resolved target.

#include <cvc/app.h>
#include <cvc/exception.h>
#include <cvc/state.h>
#include <gtest/gtest.h>

TEST(StateWritableLinkTest, DefaultLinkIsNotWritable) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta", cvc::state::link_mode::transparent);
  EXPECT_FALSE(n.linkWritable());
}

TEST(StateWritableLinkTest, NonWritableTransparentLinkAcceptsWriteOnLinkNode) {
  // Historical behavior: writes to a transparent link without the
  // writable flag land on the link node's own _value.
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  EXPECT_NO_THROW(n.value(std::string("own")));
  EXPECT_EQ(cvc::state::instance(a)("target").value(), "target-val");
  // resolvedValue still follows the transparent link to the target.
  EXPECT_EQ(n.resolvedValue(), "target-val");
}

TEST(StateWritableLinkTest, WritableTransparentLinkRoutesWriteToTarget) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  n.value(std::string("new-val"));
  EXPECT_EQ(cvc::state::instance(a)("target").value(), "new-val");
  EXPECT_EQ(n.resolvedValue(), "new-val");
}

TEST(StateWritableLinkTest, OpaqueLinkIgnoresWritableFlag) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::opaque);
  n.setLinkWritable(true); // ignored for opaque
  n.value(std::string("own"));
  EXPECT_EQ(cvc::state::instance(a)("target").value(), "target-val");
  EXPECT_EQ(n.value(), "own");
}

TEST(StateWritableLinkTest, WriteThroughFiresTargetValueChanged) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("target-val"));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  int target_fired = 0;
  cvc::state::instance(a)("target").valueChanged.connect([&]() { ++target_fired; });
  n.value(std::string("new-val"));
  EXPECT_EQ(target_fired, 1);
}

TEST(StateWritableLinkTest, WriteThroughChainResolvesToTerminalTarget) {
  cvc::app a;
  cvc::state::instance(a)("c").value(std::string("c-val"));
  cvc::state::instance(a)("b").linkTo("c", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("b").setLinkWritable(true);
  cvc::state::instance(a)("a").linkTo("b", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").setLinkWritable(true);
  cvc::state::instance(a)("a").value(std::string("written"));
  EXPECT_EQ(cvc::state::instance(a)("c").value(), "written");
}

TEST(StateWritableLinkTest, WriteThroughBrokenLinkThrows) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("does-not-exist", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  EXPECT_THROW(n.value(std::string("v")), cvc::read_only_error);
}

TEST(StateWritableLinkTest, WriteThroughCycleThrows) {
  cvc::app a;
  cvc::state::instance(a)("a").linkTo("b", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").setLinkWritable(true);
  cvc::state::instance(a)("b").linkTo("a", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("b").setLinkWritable(true);
  EXPECT_THROW(cvc::state::instance(a)("a").value(std::string("v")), cvc::read_only_error);
}

TEST(StateWritableLinkTest, ClearLinkResetsWritableFlag) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  EXPECT_TRUE(n.linkWritable());
  n.clearLink();
  EXPECT_FALSE(n.linkWritable());
}

TEST(StateWritableLinkTest, SetLinkWritableFiresLinkChanged) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("beta", cvc::state::link_mode::transparent);
  int fired = 0;
  n.linkChanged.connect([&]() { ++fired; });
  n.setLinkWritable(true);
  EXPECT_EQ(fired, 1);
  // Idempotent: same value does not refire.
  n.setLinkWritable(true);
  EXPECT_EQ(fired, 1);
  n.setLinkWritable(false);
  EXPECT_EQ(fired, 2);
}

TEST(StateWritableLinkTest, ResolvedValueReflectsWrittenValue) {
  cvc::app a;
  cvc::state::instance(a)("target").value(std::string("orig"));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  n.value(std::string("updated"));
  EXPECT_EQ(n.resolvedValue(), "updated");
  EXPECT_EQ(cvc::state::instance(a)("target").resolvedValue(), "updated");
}
