/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Cross-node lifetime tests for the owning accessors (findDescendantShared / sharedChild / handle)
// and the pin they provide against a concurrent sweepExpired().
//
// The bare accessors operator() / findDescendant hand back a state& / state* whose owner is the
// parent's child map, so a concurrent sweepExpired() (or reset(false)) can free the node under a
// caller that still holds the bare reference — a use-after-free. The owning accessors return the
// map's shared_ptr, which PINS the node: a concurrent sweep can then only unlink it, never free it.
// See docs/roadmap/STATE_LIFETIME_AND_ATOMICITY.md (Phase 1 / Option A).
//
// Run the concurrent case under TSan/ASan for the strongest signal (this repo needs `setarch -R` to
// disable ASLR under TSan — see the TSan gotcha in the memory index).

#include <atomic>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

using cvc::state;
namespace pt = boost::posix_time;

pt::ptime now_utc() { return pt::microsec_clock::universal_time(); }

} // namespace

// A pin defers destruction: sweepExpired() unlinks the node from the tree synchronously, but the
// node itself is not freed (and `destroyed` does not fire) until the last owning pin drops. This is
// the one observable behavior change of Option A, and it is what keeps a pinned node safe.
TEST(StateLifetime, PinDefersDestruction) {
  cvc::app a;
  auto &root = state::instance(a);
  root("a.gone").value("V");

  std::atomic<int> destroyed{0};
  root("a.gone").destroyed.connect([&]() { destroyed.fetch_add(1); });

  state::state_ptr pin = root.findDescendantShared("a.gone");
  ASSERT_TRUE(static_cast<bool>(pin));

  root("a.gone").expireAt(now_utc() - pt::seconds(1));
  root.sweepExpired();

  EXPECT_EQ(root.findDescendant("a.gone"), nullptr) << "swept node must be unlinked immediately";
  EXPECT_EQ(destroyed.load(), 0) << "a pinned node must not be destroyed while a handle holds it";
  EXPECT_EQ(pin->value(), "V") << "the pinned node's own storage stays readable after its unlink";

  pin.reset();
  EXPECT_EQ(destroyed.load(), 1) << "destruction is deferred to the last owning reference";
}

// The confirmed cross-node hazard (H1): a pinned LEAF stays readable even after its INTERMEDIATE
// ancestor is swept. With a bare state* the value() read below would be a use-after-free.
TEST(StateLifetime, PinnedLeafSurvivesAncestorSweep) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.item.leaf").value("V");

  state::state_ptr leaf = root.findDescendantShared("s.item.leaf");
  ASSERT_TRUE(static_cast<bool>(leaf));

  // Expire + sweep the INTERMEDIATE ancestor: the whole s.item subtree is unlinked and (for the
  // unpinned parts) freed.
  root("s.item").expireAt(now_utc() - pt::seconds(1));
  root.sweepExpired();
  EXPECT_EQ(root.findDescendant("s.item"), nullptr);
  EXPECT_EQ(root.findDescendant("s.item.leaf"), nullptr);

  // Reading the pinned leaf's OWN storage is safe even though its parent was freed. (We
  // deliberately do NOT call leaf->fullName(): that walks the freed _parent chain — the documented
  // ancestor-walk residual that the state:// resolver now avoids by deriving its canonical path
  // from strings.)
  EXPECT_EQ(leaf->value(), "V");
}

// The WRITE analogue: mutating a node (value()/data()) internally walks its _parent chain
// (fullName() + parent()->childChanged()), so a safe write after an ancestor sweep needs the whole
// ANCESTOR CHAIN pinned — not just the leaf. This is what the state:// store path now does via
// sharedChild(path, &chain); a leaf-only pin would UAF here.
TEST(StateLifetime, PinnedChainMakesWriteSafeAfterAncestorSweep) {
  cvc::app a;
  auto &root = state::instance(a);
  root("s.item.leaf").value("V");

  std::vector<state::state_ptr> chain; // pins root-child .. leaf
  state::state_ptr leaf = root.sharedChild("s.item.leaf", &chain);
  ASSERT_TRUE(static_cast<bool>(leaf));
  ASSERT_FALSE(chain.empty());

  root("s.item").expireAt(now_utc() - pt::seconds(1));
  root.sweepExpired();
  EXPECT_EQ(root.findDescendant("s.item"), nullptr) << "intermediate unlinked";

  // Writing the leaf walks its _parent chain; with the chain pinned every ancestor is still alive,
  // so the write is UAF-free (with only a leaf pin this would be a use-after-free).
  leaf->value("W");
  EXPECT_EQ(leaf->value(), "W");
}

// sharedChild is the create-or-get owning analogue of operator(): same path semantics, but pins.
TEST(StateLifetime, SharedChildCreatesAndPins) {
  cvc::app a;
  auto &root = state::instance(a);

  state::state_ptr n = root.sharedChild("x.y.z");
  ASSERT_TRUE(static_cast<bool>(n));
  n->value("42");
  // Same node as operator() reaches.
  EXPECT_EQ(root("x.y.z").value(), "42");
  EXPECT_EQ(root.sharedChild("x.y.z")->value(), "42");
  // An empty childname returns this node itself (like operator()).
  EXPECT_EQ(root.sharedChild("").get(), &root);
}

// Readers pin+read leaves via the owning accessor (the resolver's shape) while a sweeper churns the
// subtree — expire the intermediate, sweep (freeing the unpinned leaves), recreate. Under the fix,
// findDescendantShared either returns a pinned node (safe to read) or nullptr (already unlinked);
// it never yields a dangling pointer, so there is no crash/UAF. (With bare findDescendant this
// races on a freed node.) NOTE: a plain run only catches a UAF if it happens to corrupt memory
// observably; run under ASan/TSan (this repo needs `setarch -R` for TSan) for a reliable signal.
TEST(StateLifetime, ConcurrentPinnedReadVsSweep) {
  cvc::app a;
  auto &root = state::instance(a);
  constexpr int kLeaves = 16;
  constexpr int kIters = 300;

  const auto seed = [&]() {
    for (int i = 0; i < kLeaves; ++i)
      root("s.item." + std::to_string(i)).value("v" + std::to_string(i));
  };
  seed();

  std::atomic<bool> stop{false};
  std::atomic<long> reads{0};

  auto reader = [&]() {
    while (!stop.load(std::memory_order_relaxed)) {
      for (int i = 0; i < kLeaves; ++i) {
        state::state_ptr p = root.findDescendantShared("s.item." + std::to_string(i));
        if (p) {
          const std::string v = p->value(); // touch pinned storage — must never UAF
          if (!v.empty())
            reads.fetch_add(1, std::memory_order_relaxed);
        }
      }
    }
  };

  std::vector<std::thread> readers;
  for (int t = 0; t < 4; ++t)
    readers.emplace_back(reader);

  for (int it = 0; it < kIters; ++it) {
    root("s.item").expireAt(now_utc() - pt::seconds(1));
    root.sweepExpired();
    seed();
  }

  stop.store(true, std::memory_order_relaxed);
  for (auto &t : readers)
    t.join();

  EXPECT_GT(reads.load(), 0) << "readers should have observed live values across the churn";
}
