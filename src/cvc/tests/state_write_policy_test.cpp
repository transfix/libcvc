/*
  Copyright 2026 The University of Texas at Austin
  Phase 5 — state_write_policy tests.
*/

#include <cvc/core/state_write_policy.h>
#include <gtest/gtest.h>

using cvc::state_write_policy;

TEST(StateWritePolicy, NoEntryAllowsAll) {
  state_write_policy p;
  auto d = p.authorize("a.b.c", "anyone");
  EXPECT_TRUE(d.allowed);
  EXPECT_FALSE(d.had_policy);
  EXPECT_TRUE(d.matched_prefix.empty());
}

TEST(StateWritePolicy, AllowExactPrefix) {
  state_write_policy p;
  p.allow("a.b", {"writer1", "writer2"});
  auto ok = p.authorize("a.b", "writer1");
  EXPECT_TRUE(ok.allowed);
  EXPECT_TRUE(ok.had_policy);
  EXPECT_EQ("a.b", ok.matched_prefix);

  auto deny = p.authorize("a.b", "stranger");
  EXPECT_FALSE(deny.allowed);
  EXPECT_TRUE(deny.had_policy);
  EXPECT_FALSE(deny.reject_reason.empty());
}

TEST(StateWritePolicy, LongestPrefixWins) {
  state_write_policy p;
  p.allow("a", {"root_writer"});
  p.allow("a.b", {"sub_writer"});

  // root_writer cannot write under a.b — the longer prefix wins.
  auto deny = p.authorize("a.b.c", "root_writer");
  EXPECT_FALSE(deny.allowed);
  EXPECT_EQ("a.b", deny.matched_prefix);

  auto ok = p.authorize("a.b.c", "sub_writer");
  EXPECT_TRUE(ok.allowed);
  EXPECT_EQ("a.b", ok.matched_prefix);

  // The shorter prefix still applies when no deeper one matches.
  auto ok2 = p.authorize("a.x", "root_writer");
  EXPECT_TRUE(ok2.allowed);
  EXPECT_EQ("a", ok2.matched_prefix);
}

TEST(StateWritePolicy, EmptyAllowedSetIsLockdown) {
  state_write_policy p;
  p.allow("locked", {});
  auto d = p.authorize("locked.path", "anyone");
  EXPECT_FALSE(d.allowed);
  EXPECT_TRUE(d.had_policy);
}

TEST(StateWritePolicy, RootPrefixCoversEverything) {
  state_write_policy p;
  p.allow("", {"only_me"});
  EXPECT_TRUE(p.authorize("anything", "only_me").allowed);
  EXPECT_FALSE(p.authorize("anything", "stranger").allowed);
}

TEST(StateWritePolicy, RevokeRemovesEntry) {
  state_write_policy p;
  p.allow("a.b", {"w"});
  EXPECT_EQ(1u, p.size());
  EXPECT_TRUE(p.revoke("a.b"));
  EXPECT_FALSE(p.revoke("a.b"));
  EXPECT_EQ(0u, p.size());
  // After revoke, authorize returns no-policy.
  auto d = p.authorize("a.b", "anyone");
  EXPECT_TRUE(d.allowed);
  EXPECT_FALSE(d.had_policy);
}

TEST(StateWritePolicy, ClearWipesAll) {
  state_write_policy p;
  p.allow("a", {"x"});
  p.allow("b", {"y"});
  p.clear();
  EXPECT_EQ(0u, p.size());
  EXPECT_TRUE(p.authorize("a", "x").allowed);
  EXPECT_FALSE(p.authorize("a", "x").had_policy);
}

TEST(StateWritePolicy, AllowReplacesPriorAllowedSet) {
  state_write_policy p;
  p.allow("a", {"old_writer"});
  p.allow("a", {"new_writer"});
  EXPECT_EQ(1u, p.size());
  EXPECT_FALSE(p.authorize("a", "old_writer").allowed);
  EXPECT_TRUE(p.authorize("a", "new_writer").allowed);
}
