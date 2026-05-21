/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Per-node expiry tests.
//
// expireAt / expireAfter / clearExpiry / hasExpiry / expiryTime /
// isExpired / sweepExpired and the `expiring` signal.

#include <atomic>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <chrono>
#include <cvc/app.h>
#include <cvc/state.h>
#include <gtest/gtest.h>
#include <thread>

namespace {

using cvc::state;
namespace pt = boost::posix_time;

pt::ptime now_utc() { return pt::microsec_clock::universal_time(); }

} // namespace

TEST(StateExpiry, NoExpiryByDefault) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  EXPECT_FALSE(n.hasExpiry());
  EXPECT_FALSE(n.isExpired());
  EXPECT_TRUE(n.expiryTime().is_not_a_date_time());
}

TEST(StateExpiry, ExpireAtPastTimeMakesNodeExpired) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  n.expireAt(now_utc() - pt::seconds(1));
  EXPECT_TRUE(n.hasExpiry());
  EXPECT_TRUE(n.isExpired());
}

TEST(StateExpiry, ExpireAtFutureTimeIsNotYetExpired) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  n.expireAt(now_utc() + pt::hours(1));
  EXPECT_TRUE(n.hasExpiry());
  EXPECT_FALSE(n.isExpired());
}

TEST(StateExpiry, ClearExpiryRemovesMark) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  n.expireAt(now_utc() - pt::seconds(1));
  ASSERT_TRUE(n.hasExpiry());
  n.clearExpiry();
  EXPECT_FALSE(n.hasExpiry());
  EXPECT_FALSE(n.isExpired());
}

TEST(StateExpiry, RootExpireAtIsNoOp) {
  cvc::app a;
  auto &root = state::instance(a);
  root.expireAt(now_utc() - pt::hours(1));
  EXPECT_FALSE(root.hasExpiry());
  EXPECT_FALSE(root.isExpired());
}

TEST(StateExpiry, SweepExpiredRemovesExpiredChild) {
  cvc::app a;
  auto &root = state::instance(a);
  root("a.keep").value("k");
  root("a.gone").expireAt(now_utc() - pt::seconds(1));

  std::size_t removed = root.sweepExpired();
  EXPECT_EQ(removed, 1u);

  // gone is detached: looking it up via findDescendant returns null.
  EXPECT_EQ(root.findDescendant("a.gone"), nullptr);
  // keep survives
  EXPECT_NE(root.findDescendant("a.keep"), nullptr);
}

TEST(StateExpiry, SweepExpiredFiresExpiringBeforeDestroyed) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.gone");

  std::atomic<int> order{0};
  int expiring_order = -1;
  int destroyed_order = -1;
  n.expiring.connect([&]() { expiring_order = ++order; });
  n.destroyed.connect([&]() { destroyed_order = ++order; });

  n.expireAt(now_utc() - pt::seconds(1));
  root.sweepExpired();

  EXPECT_EQ(expiring_order, 1);
  EXPECT_EQ(destroyed_order, 2);
}

TEST(StateExpiry, ExpiredSubtreeIsRemovedWithChildren) {
  cvc::app a;
  auto &root = state::instance(a);
  root("a.b.c").value("deep");
  root("a.b.d").value("deeper");
  root("a.b").expireAt(now_utc() - pt::seconds(1));

  std::size_t removed = root.sweepExpired();
  // a.b removed; its descendants were not individually expired so
  // the recursive count is just the one removal at this layer.
  EXPECT_EQ(removed, 1u);

  EXPECT_EQ(root.findDescendant("a.b"), nullptr);
  EXPECT_EQ(root.findDescendant("a.b.c"), nullptr);
  EXPECT_EQ(root.findDescendant("a.b.d"), nullptr);
  // sibling path still works
  EXPECT_NE(root.findDescendant("a"), nullptr);
}

TEST(StateExpiry, NestedExpiriesAreAllReportedFromRootSweep) {
  cvc::app a;
  auto &root = state::instance(a);
  root("x.y.z").expireAt(now_utc() - pt::seconds(1));
  root("x.y").expireAt(now_utc() - pt::seconds(1));
  root("x.other").value("survives");

  std::size_t removed = root.sweepExpired();
  // Post-order: z then y. Two distinct removals.
  EXPECT_EQ(removed, 2u);
  EXPECT_EQ(root.findDescendant("x.y"), nullptr);
  EXPECT_EQ(root.findDescendant("x.y.z"), nullptr);
  EXPECT_NE(root.findDescendant("x.other"), nullptr);
}

TEST(StateExpiry, ExpireAfterMatchesExpireAtPlusDuration) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  pt::ptime before = now_utc();
  n.expireAfter(pt::milliseconds(50));
  pt::ptime after = now_utc();
  pt::ptime t = n.expiryTime();
  ASSERT_TRUE(n.hasExpiry());
  EXPECT_GE(t, before + pt::milliseconds(50));
  EXPECT_LE(t, after + pt::milliseconds(50));
}

TEST(StateExpiry, ClearExpiryPreventsSweepRemoval) {
  cvc::app a;
  auto &root = state::instance(a);
  auto &n = root("a.b");
  n.expireAt(now_utc() - pt::seconds(1));
  n.clearExpiry();

  std::size_t removed = root.sweepExpired();
  EXPECT_EQ(removed, 0u);
  EXPECT_NE(root.findDescendant("a.b"), nullptr);
}

TEST(StateExpiry, SweepIsIdempotentWhenNothingExpired) {
  cvc::app a;
  auto &root = state::instance(a);
  root("a.b.c").value("v");
  EXPECT_EQ(root.sweepExpired(), 0u);
  EXPECT_EQ(root.sweepExpired(), 0u);
  EXPECT_NE(root.findDescendant("a.b.c"), nullptr);
}

TEST(StateExpiry, RealWaitThenSweepRemovesNode) {
  cvc::app a;
  auto &root = state::instance(a);
  root("a.short").expireAfter(pt::milliseconds(20));
  EXPECT_FALSE(root.findDescendant("a.short") == nullptr);

  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  std::size_t removed = root.sweepExpired();
  EXPECT_EQ(removed, 1u);
  EXPECT_EQ(root.findDescendant("a.short"), nullptr);
}
