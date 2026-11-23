/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::state's change counts (revision, shapeRevision) and cvc::state_key, the lookup they let a
// frame hold: what a held lookup finds is always what a new walk would.

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key.h>
#include <gtest/gtest.h>
#include <string>

namespace {

TEST(StateRevision, MovesWithEachChangeOfTheValueOrData) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  cvc::state &k = root("a.b");
  const std::uint64_t r0 = k.revision();
  k.value("1");
  EXPECT_EQ(k.revision(), r0 + 1);
  k.value("1"); // the same value: nothing fires, nothing moves
  EXPECT_EQ(k.revision(), r0 + 1);
  k.value(2);
  EXPECT_EQ(k.revision(), r0 + 2);
  k.value(2);
  EXPECT_EQ(k.revision(), r0 + 2);
  k.data(std::string("payload"));
  EXPECT_EQ(k.revision(), r0 + 3);
  EXPECT_TRUE(k.compareAndSetValue("2", "3"));
  EXPECT_EQ(k.revision(), r0 + 4);
  EXPECT_FALSE(k.compareAndSetValue("2", "4"));
  EXPECT_EQ(k.revision(), r0 + 4);
  k.touch(); // fires as a change
  EXPECT_EQ(k.revision(), r0 + 5);
  // Another node's write leaves it.
  root("a.c").value("x");
  EXPECT_EQ(k.revision(), r0 + 5);
}

TEST(StateRevision, AWriteThroughALinkMovesTheTarget) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  cvc::state &target = root("t");
  cvc::state &hole = root("h");
  hole.linkTo("t", cvc::state::link_mode::transparent);
  hole.setLinkWritable(true);
  const std::uint64_t t0 = target.revision(), h0 = hole.revision();
  hole.value("through");
  EXPECT_EQ(target.value(), "through");
  EXPECT_EQ(target.revision(), t0 + 1);
  EXPECT_EQ(hole.revision(), h0);
}

TEST(StateShapeRevision, MovesWhenTheTreeChangesShape) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("a.b").value("1");
  cvc::state &b = root("a.b");
  std::uint64_t s = root.shapeRevision();
  EXPECT_EQ(b.shapeRevision(), s) << "one count for the whole tree";

  b.value("2"); // a write: the same shape
  root("a.b");  // a walk to what is there: the same shape
  EXPECT_EQ(root.shapeRevision(), s);

  root("a.c"); // a node made
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  root.findDescendant("a")->removeChild("c"); // removed
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();

  cvc::state &l = root("l");
  s = root.shapeRevision();
  l.linkTo("a"); // a link made
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  l.setLinkMode(cvc::state::link_mode::transparent);
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  l.setLinkWritable(true);
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  l.linkTo("a.b"); // moved
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  l.clearLink();
  EXPECT_GT(root.shapeRevision(), s);
  s = root.shapeRevision();
  root("a").reset(false); // its children go
  EXPECT_GT(root.shapeRevision(), s);

  // Another app's tree has a count of its own.
  cvc::app other;
  const std::uint64_t mine = root.shapeRevision();
  cvc::state::instance(other)("x.y");
  EXPECT_EQ(root.shapeRevision(), mine);
}

TEST(StateKey, HoldsALookupUntilTheTreeChangesShape) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("x.y").value("1");
  cvc::state_key k("x.y");
  cvc::state *first = k.find(root);
  ASSERT_EQ(first, root.findDescendant("x.y"));
  const std::uint64_t walks = k.walks();
  EXPECT_EQ(k.find(root), first);
  EXPECT_EQ(k.find(app), first);
  root("x.y").value("2"); // a write: still held
  EXPECT_EQ(k.find(app), first);
  EXPECT_LE(k.walks(), walks + 1) << "app and root lookups may each walk once";

  // Removed and made again: a new node, found by a new walk.
  root.findDescendant("x")->removeChild("y");
  EXPECT_EQ(k.find(app), nullptr);
  root("x.y").value("3");
  cvc::state *again = k.find(app);
  ASSERT_NE(again, nullptr);
  EXPECT_EQ(again->value(), "3");
  EXPECT_EQ(again, root.findDescendant("x.y"));
}

TEST(StateKey, ReadsAndWritesThroughLinksAsTheWalksDo) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("s.a").value("target's");
  cvc::state &hole = root("h");
  hole.linkTo("s", cvc::state::link_mode::transparent);
  hole.setLinkWritable(true);

  cvc::state_key r("h.a"), w("h.b"), f("h.a"), g("h.c");
  ASSERT_NE(r.read(app), nullptr);
  EXPECT_EQ(r.read(app), root.findThroughLinks("h.a"));
  EXPECT_EQ(r.read(app)->value(), "target's");
  EXPECT_EQ(f.find(app), root.findDescendant("h.a")) << "find follows no link";
  w.write(app).value("written");
  EXPECT_EQ(root.findDescendant("s.b")->value(), "written") << "a writable hole's target";
  g.get(app).value("local");
  EXPECT_EQ(root.findDescendant("h.c")->value(), "local") << "get makes the link node's own";

  // The link moves: the held read follows it.
  root("u.a").value("other's");
  hole.linkTo("u", cvc::state::link_mode::transparent);
  EXPECT_EQ(r.read(app)->value(), "other's");
}

TEST(StateKey, AMissingPathIsFoundOnceItIsMade) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  cvc::state_key k("later.on");
  EXPECT_EQ(k.read(app), nullptr);
  EXPECT_EQ(k.read(app), nullptr);
  root("later.on").value("1");
  ASSERT_NE(k.read(app), nullptr);
  EXPECT_EQ(k.read(app)->value(), "1");
}

TEST(StateKey, KindsAndPathsAreTheirOwn) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("p.q").value("q");
  root("p.r").value("r");
  cvc::state_key k("p.q");
  EXPECT_EQ(k.find(app)->value(), "q");
  EXPECT_EQ(k.get(app).value(), "q");
  k.set_path("p.r");
  EXPECT_EQ(k.find(app)->value(), "r");
  k.set_path("p.r"); // the same path keeps what it holds
  const std::uint64_t walks = k.walks();
  k.find(app);
  EXPECT_EQ(k.walks(), walks);
  k.forget();
  k.find(app);
  EXPECT_EQ(k.walks(), walks + 1);
}

TEST(StateKey, AWriteThroughAHeldNodeSignalsItsAncestors) {
  // The key pins the way to its node, so the node's write reaches its ancestors' signals even
  // after the branch left the tree.
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  cvc::state_key k("deep.er.node");
  cvc::state &n = k.write(app);
  int seen = 0;
  auto c = root("deep").childChanged.connect([&](const std::string &) { ++seen; });
  n.value("1");
  EXPECT_EQ(seen, 1);
  c.disconnect();
  root.removeChild("deep");
  n.value("2"); // held: no freed parent is walked
  EXPECT_EQ(n.value(), "2");
}

} // namespace
