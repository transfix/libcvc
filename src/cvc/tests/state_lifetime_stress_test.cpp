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
#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
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

// --- removeChild (state-delete's unlink, td 4.8; HZ-5's rows) ---

// One child goes, and nothing else: an expired sibling and an expired node elsewhere stay until a
// sweep, which then finds both.
TEST(StateLifetime, RemoveChildUnlinksOnlyThatChild) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p.a.x").value("A");
  root("p.b").value("B");
  root("p.c").value("C");
  root("q.z").value("Z");
  root("p.b").expireAt(now_utc() - pt::seconds(1));
  root("q.z").expireAt(now_utc() - pt::seconds(1));

  EXPECT_TRUE(root("p").removeChild("a"));
  EXPECT_EQ(root.findDescendant("p.a"), nullptr);
  EXPECT_EQ(root.findDescendant("p.a.x"), nullptr);
  ASSERT_NE(root.findDescendant("p.b"), nullptr) << "an expired sibling is not swept";
  ASSERT_NE(root.findDescendant("q.z"), nullptr) << "an expired node elsewhere is not swept";
  EXPECT_EQ(root.findDescendant("p.c")->value(), "C");
  EXPECT_EQ(root("p").childNames(), (std::vector<std::string>{"b", "c"}));

  EXPECT_FALSE(root("p").removeChild("a")) << "already gone";
  EXPECT_FALSE(root("p").removeChild("missing"));
  EXPECT_FALSE(root("p").removeChild("c.x")) << "a direct child's name, not a path";
  EXPECT_EQ(root.sweepExpired(), 2u);
}

// The child is unlinked at once, but freed only when the last pin on it goes, and a pinned
// descendant outlives it.
TEST(StateLifetime, RemoveChildDefersDestructionWhilePinned) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p.a.x").value("X");
  root("p.a").value("A");

  std::atomic<int> destroyed{0}, kid_destroyed{0};
  root("p.a").destroyed.connect([&]() { destroyed.fetch_add(1); });
  root("p.a.x").destroyed.connect([&]() { kid_destroyed.fetch_add(1); });

  state::state_ptr pin = root.findDescendantShared("p.a");
  state::state_ptr kid = root.findDescendantShared("p.a.x");
  ASSERT_TRUE(static_cast<bool>(pin) && static_cast<bool>(kid));

  ASSERT_TRUE(root("p").removeChild("a"));
  EXPECT_EQ(root.findDescendant("p.a"), nullptr) << "unlinked at once";
  EXPECT_EQ(destroyed.load(), 0) << "not freed while pinned";
  EXPECT_EQ(pin->value(), "A");
  EXPECT_EQ(pin->findDescendant("x"), kid.get()) << "its subtree is intact while it is pinned";

  pin.reset();
  EXPECT_EQ(destroyed.load(), 1) << "freed with its last pin";
  EXPECT_EQ(kid_destroyed.load(), 0) << "a pinned descendant outlives it";
  EXPECT_EQ(kid->value(), "X");
  kid.reset();
  EXPECT_EQ(kid_destroyed.load(), 1);
}

// The parent's childChanged fires once, up the chain as any change does; `expiring` never fires.
// A child that is not there fires nothing.
TEST(StateLifetime, RemoveChildEmitsChildChangedNotExpiring) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p.a.x").value("X");

  std::vector<std::string> at_p, at_root;
  std::atomic<int> expiring{0};
  root("p").childChanged.connect([&](const std::string &path) { at_p.push_back(path); });
  root.childChanged.connect([&](const std::string &path) { at_root.push_back(path); });
  root("p.a").expiring.connect([&]() { expiring.fetch_add(1); });
  root("p.a.x").expiring.connect([&]() { expiring.fetch_add(1); });

  const pt::ptime before = now_utc();
  ASSERT_TRUE(root("p").removeChild("a"));
  EXPECT_EQ(at_p, (std::vector<std::string>{"a"}));
  EXPECT_EQ(at_root, (std::vector<std::string>{"p.a"}));
  EXPECT_EQ(expiring.load(), 0);
  EXPECT_GE(root("p").lastMod(), before) << "a structural change stamps the parent";

  EXPECT_FALSE(root("p").removeChild("a"));
  EXPECT_EQ(at_p.size(), 1u) << "nothing removed, nothing emitted";
}

// --- the pinned walks (findThroughLinksShared / childThroughLinksShared, td 4.8) ---

// The read walk returns the node a read sees, owned, and `pins` gets every hop below the start and
// each link's target chain: everything a write to the result walks.
TEST(StateLifetime, PinnedReadWalkHoldsEveryHopAndLinkChain) {
  cvc::app a;
  auto &root = state::instance(a);
  root("x.s.a.b").value("V");
  root("hole").linkTo("x.s", state::link_mode::transparent);

  std::vector<state::state_ptr> pins;
  state::state_ptr n = root.findThroughLinksShared("hole.a.b", &pins);
  ASSERT_TRUE(static_cast<bool>(n));
  EXPECT_EQ(n.get(), &root("x.s.a.b"));
  std::vector<state *> got;
  for (const state::state_ptr &p : pins)
    got.push_back(p.get());
  EXPECT_EQ(got, (std::vector<state *>{&root("hole"), &root("x"), &root("x.s"), &root("x.s.a"),
                                       &root("x.s.a.b")}));
  EXPECT_EQ(pins.back(), n);

  // The last hop is followed too; a missing segment is null and creates nothing.
  pins.clear();
  EXPECT_EQ(root.findThroughLinksShared("hole", &pins).get(), &root("x.s"));
  EXPECT_EQ(pins.size(), 3u) << "the link node, then its target's chain";
  EXPECT_FALSE(static_cast<bool>(root.findThroughLinksShared("hole.a.missing")));
  EXPECT_EQ(root.findDescendant("x.s.a.missing"), nullptr);

  // The start is the caller's: an empty path returns it and pins nothing.
  pins.clear();
  EXPECT_EQ(root.findThroughLinksShared("", &pins).get(), &root);
  EXPECT_TRUE(pins.empty());
  // The raw walk is the same walk.
  EXPECT_EQ(root.findThroughLinks("hole.a.b"), &root("x.s.a.b"));
}

// The write walk lands where state-set writes: through a writable link on the target's children,
// creating what is missing, with the same pins.
TEST(StateLifetime, PinnedWriteWalkHoldsEveryHopAndLinkChain) {
  cvc::app a;
  auto &root = state::instance(a);
  root("x.s").value("");
  root("hole").linkTo("x.s", state::link_mode::transparent);
  root("hole").setLinkWritable(true);

  std::vector<state::state_ptr> pins;
  state::state_ptr n = root.childThroughLinksShared("hole.n.m", false, &pins);
  ASSERT_TRUE(static_cast<bool>(n));
  EXPECT_EQ(n.get(), root.findDescendant("x.s.n.m")) << "made under the target";
  std::vector<state *> got;
  for (const state::state_ptr &p : pins)
    got.push_back(p.get());
  EXPECT_EQ(got, (std::vector<state *>{&root("hole"), &root("x"), &root("x.s"), &root("x.s.n"),
                                       &root("x.s.n.m")}));
  EXPECT_EQ(&root.childThroughLinks("hole.n.m"), n.get()) << "the raw walk is the same walk";
  EXPECT_EQ(root.childThroughLinksShared("", false, &pins).get(), &root);
}

// The start-pinned lookups take no hold on their start: an empty path returns the caller's own
// hold (here a non-owning alias, as a host's root not owned by a shared_ptr would be), and
// `pins` gets the hops below it.
TEST(StateLifetime, StartPinnedWalksReturnTheCallersHold) {
  cvc::app a;
  auto &root = state::instance(a);
  root("p.q").value("Q");
  const state::state_ptr alias(state::state_ptr(), &root);
  ASSERT_EQ(alias.use_count(), 0);

  std::vector<state::state_ptr> pins;
  state::state_ptr q = root.findDescendantShared(alias, "p.q", &pins);
  ASSERT_TRUE(static_cast<bool>(q));
  EXPECT_EQ(q->value(), "Q");
  ASSERT_EQ(pins.size(), 2u);
  EXPECT_EQ(pins[0].get(), &root("p"));
  EXPECT_EQ(pins[1], q);
  EXPECT_FALSE(static_cast<bool>(root.findDescendantShared(alias, "p.missing")));

  state::state_ptr self = root.findDescendantShared(alias, " . ");
  EXPECT_EQ(self.get(), &root);
  EXPECT_EQ(self.use_count(), 0) << "the caller's alias, not a new reference";

  pins.clear();
  state::state_ptr made = root.sharedChild(alias, "p.r.s", &pins);
  EXPECT_EQ(made.get(), root.findDescendant("p.r.s"));
  EXPECT_EQ(pins.size(), 3u);
  EXPECT_EQ(root.sharedChild(alias, "").use_count(), 0);
}

// A node reached by a pinned walk stays readable, and writable, after an ancestor is removed: its
// pins hold the whole chain a write emits through.
TEST(StateLifetime, PinnedWalkResultSurvivesRemoveChild) {
  cvc::app a;
  auto &root = state::instance(a);
  root("x.s.a").value("V");
  root("hole").linkTo("x.s", state::link_mode::transparent);
  root("hole").setLinkWritable(true);

  std::vector<state::state_ptr> pins;
  state::state_ptr n = root.childThroughLinksShared("hole.a", false, &pins);
  ASSERT_TRUE(static_cast<bool>(n));
  ASSERT_TRUE(root.removeChild("x")) << "the target's ancestor";
  EXPECT_EQ(root.findDescendant("x"), nullptr);
  EXPECT_EQ(n->value(), "V");
  n->value("W"); // emits through x.s and x, held by the pins
  EXPECT_EQ(n->value(), "W");
  EXPECT_EQ(n->fullName(), "x.s.a");
}

// Readers walk through a link and write what they reach while another thread removes and re-makes
// the target: a walk gives a held node or null, never a freed one. Run under ASan or TSan for the
// strongest signal; here the proof is that the readers reached nodes and nothing crashed.
TEST(StateLifetime, PinnedWalksAgainstConcurrentRemoveChild) {
  cvc::app a;
  auto &root = state::instance(a);
  constexpr int kIters = 300;
  const auto make = [&]() {
    root("x.s.a.b").value("v");
    root("x.s.c").value("c");
  };
  make();
  root("hole").linkTo("x.s", state::link_mode::transparent);

  std::atomic<bool> stop{false};
  std::atomic<long> reached{0};
  const auto reader = [&]() {
    std::vector<state::state_ptr> pins;
    while (!stop.load(std::memory_order_relaxed)) {
      pins.clear();
      if (state::state_ptr n = root.findThroughLinksShared("hole.a.b", &pins)) {
        if (!n->value().empty())
          reached.fetch_add(1, std::memory_order_relaxed);
        n->value("r"); // emits up the held chain
      }
      if (state::state_ptr c = root.findThroughLinksShared("hole.c"))
        (void)c->value();
    }
  };
  std::vector<std::thread> readers;
  for (int t = 0; t < 3; ++t)
    readers.emplace_back(reader);
  // Churn once a reader is in (a busy box may start them late), with a generous ceiling.
  const auto t0 = std::chrono::steady_clock::now();
  while (reached.load() == 0 && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(30))
    std::this_thread::yield();
  for (int it = 0; it < kIters; ++it) {
    root("x").removeChild("s");
    make();
  }
  stop.store(true, std::memory_order_relaxed);
  for (std::thread &t : readers)
    t.join();
  std::printf("reached %ld nodes over %d removals\n", reached.load(), kIters);
  EXPECT_GT(reached.load(), 0);
  EXPECT_NE(root.findDescendant("x.s.a.b"), nullptr);
}
