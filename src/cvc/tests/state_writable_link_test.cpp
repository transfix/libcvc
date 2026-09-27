/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8: writable transparent links route writes to the resolved target.

#include <boost/any.hpp>
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/state.h>
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

// ---- Typed (value<T>) writes route the same way the string setter does ----

TEST(StateWritableLinkTest, TypedWriteThroughWritableTransparentLinkRoutesToTarget) {
  cvc::app a;
  cvc::state::instance(a)("target").value<int>(1);
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  // A TYPED write must land on the target, not on the link node's own value.
  n.value<int>(9);
  auto &target = cvc::state::instance(a)("target");
  EXPECT_EQ(target.value<int>(), 9);
  EXPECT_EQ(n.resolvedValue(), "9");
  // Routing goes through the TEMPLATED overload, so the target keeps the
  // typed valueTypeName -- NOT the "std::string" the string overload records.
  EXPECT_EQ(target.valueTypeName(), a.dataTypeName<int>());
  EXPECT_NE(target.valueTypeName(), std::string("std::string"));
}

TEST(StateWritableLinkTest, TypedWriteThroughChainResolvesToTerminalTarget) {
  cvc::app a;
  cvc::state::instance(a)("c").value<int>(0);
  cvc::state::instance(a)("b").linkTo("c", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("b").setLinkWritable(true);
  cvc::state::instance(a)("a").linkTo("b", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").setLinkWritable(true);
  cvc::state::instance(a)("a").value<int>(42);
  EXPECT_EQ(cvc::state::instance(a)("c").value<int>(), 42);
}

TEST(StateWritableLinkTest, NonWritableTransparentLinkAcceptsTypedWriteOnLinkNode) {
  // Historical behavior parity with the string setter: a typed write to a
  // transparent link without the writable flag lands on the link node itself.
  cvc::app a;
  cvc::state::instance(a)("target").value<int>(7);
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  EXPECT_NO_THROW(n.value<int>(3));
  EXPECT_EQ(cvc::state::instance(a)("target").value<int>(), 7); // target untouched
  EXPECT_EQ(n.value<int>(), 3);                                 // link node's own value
}

TEST(StateWritableLinkTest, TypedWriteThroughBrokenLinkThrows) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("does-not-exist", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  EXPECT_THROW(n.value<int>(5), cvc::read_only_error);
}

TEST(StateWritableLinkTest, TypedWriteThroughCycleThrows) {
  cvc::app a;
  cvc::state::instance(a)("a").linkTo("b", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").setLinkWritable(true);
  cvc::state::instance(a)("b").linkTo("a", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("b").setLinkWritable(true);
  EXPECT_THROW(cvc::state::instance(a)("a").value<int>(5), cvc::read_only_error);
}

// ---- data()/resolvedData() follow links the same way value() does ----

TEST(StateWritableLinkTest, DataWriteThroughWritableTransparentLinkRoutesToTarget) {
  cvc::app a;
  cvc::state::instance(a)("target").data(boost::any(int(1)));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  n.data(boost::any(int(9)));
  EXPECT_EQ(cvc::state::instance(a)("target").data<int>(), 9);
  // resolvedData reads through the transparent link to the target's data.
  EXPECT_EQ(boost::any_cast<int>(n.resolvedData()), 9);
}

TEST(StateWritableLinkTest, DataWriteThroughBrokenLinkThrows) {
  cvc::app a;
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("does-not-exist", cvc::state::link_mode::transparent);
  n.setLinkWritable(true);
  EXPECT_THROW(n.data(boost::any(int(5))), cvc::read_only_error);
}

TEST(StateWritableLinkTest, NonWritableTransparentLinkAcceptsDataWriteOnLinkNode) {
  // Historical parity: a data write to a transparent link without the
  // writable flag lands on the link node itself, target untouched.
  cvc::app a;
  cvc::state::instance(a)("target").data(boost::any(int(7)));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  EXPECT_NO_THROW(n.data(boost::any(int(3))));
  EXPECT_EQ(cvc::state::instance(a)("target").data<int>(), 7);
  EXPECT_EQ(n.data<int>(), 3);
}

TEST(StateWritableLinkTest, ResolvedDataFollowsTransparentLinkOnReadOnlyLink) {
  // A read-only (non-writable) transparent link still READS through to the
  // target via resolvedData(), mirroring resolvedValue().
  cvc::app a;
  cvc::state::instance(a)("target").data(boost::any(int(11)));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::transparent);
  EXPECT_EQ(boost::any_cast<int>(n.resolvedData()), 11);
}

TEST(StateWritableLinkTest, OpaqueLinkResolvedDataReturnsOwnData) {
  // Opaque links do not follow: resolvedData() returns the link node's own
  // data, and a data write lands on the link node itself.
  cvc::app a;
  cvc::state::instance(a)("target").data(boost::any(int(11)));
  auto &n = cvc::state::instance(a)("alpha");
  n.linkTo("target", cvc::state::link_mode::opaque);
  n.setLinkWritable(true); // ignored for opaque
  n.data(boost::any(int(4)));
  EXPECT_EQ(cvc::state::instance(a)("target").data<int>(), 11);
  EXPECT_EQ(boost::any_cast<int>(n.resolvedData()), 4);
}

TEST(StateWritableLinkTest, DataWriteThroughChainResolvesToTerminalTarget) {
  cvc::app a;
  cvc::state::instance(a)("c").data(boost::any(int(0)));
  cvc::state::instance(a)("b").linkTo("c", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("b").setLinkWritable(true);
  cvc::state::instance(a)("a").linkTo("b", cvc::state::link_mode::transparent);
  cvc::state::instance(a)("a").setLinkWritable(true);
  cvc::state::instance(a)("a").data(boost::any(int(42)));
  EXPECT_EQ(cvc::state::instance(a)("c").data<int>(), 42);
}
