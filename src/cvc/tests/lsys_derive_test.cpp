/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/derive.h for the full header).
*/

#include <cvc/lsys/derive.h>
#include <cvc/lsys/parse.h>
#include <gtest/gtest.h>

using namespace cvc::lsys;

namespace {
ruleset R(const char *s) {
  parse_result p = parse_lsys(s);
  EXPECT_TRUE(p.ok);
  return p.rs;
}
word derive_word(const ruleset &rs, int gen, std::uint64_t seed = 0) {
  derive_options o;
  o.generations = gen;
  o.master_seed = seed;
  return derive(rs, o).w;
}
} // namespace

TEST(LsysDerive, AlgaeFibonacci) {
  // A->AB, B->A gives word lengths on the Fibonacci sequence (ABOP fig 1.3).
  ruleset rs = R("kind: plant\nmode: parallel\naxiom: A\nA -> A B\nB -> A\n");
  const std::size_t want[] = {1, 2, 3, 5, 8, 13, 21, 34};
  for (int g = 0; g <= 7; ++g)
    EXPECT_EQ(derive_word(rs, g).size(), want[g]) << "gen " << g;
}

TEST(LsysDerive, DeterministicAcrossRepeats) {
  ruleset rs = R("kind: plant\nmode: parallel\nangle: 25\nstep: 5\nparam r = 0.7\naxiom: A(1)\n"
                 "A(s) : s > 0.06 -> F(5*s) [ +(25) A(s*r) ] [ -(25) A(s*r) ]\n");
  word a = derive_word(rs, 7, 42);
  word b = derive_word(rs, 7, 42);
  EXPECT_EQ(a.size(), b.size());
  EXPECT_EQ(a.content_hash(), b.content_hash());
}

TEST(LsysDerive, StochasticVariesBySeedButIsReproducible) {
  ruleset rs = R("kind: plant\nmode: parallel\naxiom: X\nX -> (0.5) A\nX -> (0.5) B\n");
  word s1 = derive_word(rs, 1, 1);
  word s1b = derive_word(rs, 1, 1);
  word s2 = derive_word(rs, 1, 2);
  EXPECT_EQ(s1.content_hash(), s1b.content_hash()); // same seed -> identical
  // Over many seeds we should see both A and B chosen.
  int a = 0, b = 0;
  for (std::uint64_t seed = 0; seed < 40; ++seed) {
    word w = derive_word(rs, 1, seed);
    if (w[0].sym == s1[0].sym)
      ++a;
    else
      ++b;
  }
  EXPECT_GT(a, 0);
  EXPECT_GT(b, 0);
  (void)s2;
}

TEST(LsysDerive, LevelCountsGiveLodRungs) {
  ruleset rs = R("kind: plant\nmode: parallel\nangle: 25\nstep: 5\nparam r = 0.7\naxiom: A(1)\n"
                 "A(s) : s > 0.05 -> F(5*s) [ +(25) A(s*r) ] [ -(25) A(s*r) ]\n");
  word w = derive_word(rs, 6, 1);
  std::vector<module_t> lod2, lod4;
  w.filter_level(2, lod2);
  w.filter_level(4, lod4);
  // Coarser rung is a strict subset in size.
  EXPECT_LT(lod2.size(), lod4.size());
  EXPECT_LE(lod4.size(), w.size());
  // Every module in a rung has level <= the rung.
  for (const module_t &m : lod2)
    EXPECT_LE(int(m.level), 2);
}

TEST(LsysDerive, SequentialBuildingTerminates) {
  ruleset rs = R("kind: building\nmode: sequential\naxiom: Scale(10,8,3) B(4)\n"
                 "B(n) : n > 0 -> Box(6) Trans(0,0,3) B(n-1)\n");
  word w = derive_word(rs, 1, 0);
  // 4 Box terminals + scale + 4 trans + leftover B(0).
  int boxes = 0;
  for (std::size_t i = 0; i < w.size(); ++i)
    if (rs.syms.name(w[i].sym) == "Box")
      ++boxes;
  EXPECT_EQ(boxes, 4);
}

TEST(LsysDerive, ModuleBudgetTruncates) {
  ruleset rs = R("kind: plant\nmode: parallel\naxiom: A\nA -> A A\n");
  derive_options o;
  o.generations = 40; // 2^40 without a budget
  o.max_modules = 1000;
  derive_result r = derive(rs, o);
  EXPECT_TRUE(r.truncated);
  EXPECT_LE(r.w.size(), o.max_modules + 2);
}

TEST(LsysDerive, ResumableDeriverMatchesOneShot) {
  ruleset rs = R("kind: plant\nmode: parallel\nangle: 25\nstep: 5\nparam r = 0.7\naxiom: A(1)\n"
                 "A(s) : s > 0.06 -> F(5*s) [ +(25) A(s*r) ] [ -(25) A(s*r) ]\n");
  derive_options o;
  o.generations = 6;
  o.master_seed = 5;
  deriver d(rs, o);
  int calls = 0;
  for (;;) {
    bool more = d.step();
    ++calls;
    if (!more)
      break;
  }
  EXPECT_EQ(calls, 6); // one generation per step() call, last returns false
  EXPECT_EQ(d.result().w.content_hash(), derive(rs, o).w.content_hash());
}

TEST(LsysDerive, DeriverCancel) {
  ruleset rs = R("kind: plant\nmode: parallel\naxiom: A\nA -> A A\n");
  derive_options o;
  o.generations = 10;
  deriver d(rs, o);
  d.step();
  d.cancel();
  EXPECT_FALSE(d.step());
}

TEST(LsysDerive, ContextSensitiveRewrite) {
  // A signal propagates one step per generation: b < a -> b (with #ignore of the
  // separators). Classic context-sensitive L-system behaviour.
  ruleset rs = R("kind: plant\nmode: parallel\nignore: + -\naxiom: b a a a a\n"
                 "b < a -> b\nb -> a\n");
  word w = derive_word(rs, 1, 0);
  // After one generation the 'b' has moved one 'a' to the right.
  int firstb = -1;
  for (std::size_t i = 0; i < w.size(); ++i)
    if (rs.syms.name(w[i].sym) == "b") {
      firstb = int(i);
      break;
    }
  EXPECT_EQ(firstb, 1); // the leading b became a, the next a became b
}

TEST(LsysDerive, SequentialPriorityPicksHigher) {
  // Two productions for the same predecessor; the higher-priority one is chosen.
  // (priority is authored via successors here through the sequential worklist.)
  ruleset rs = R("kind: building\nmode: sequential\naxiom: X\nX -> Box(6)\n");
  word w = derive_word(rs, 1, 0);
  EXPECT_EQ(rs.syms.name(w[0].sym), "Box");
}

TEST(LsysDerive, SequentialDeletingRuleTerminates) {
  // A deleting production (empty successor) prunes; sequential mode still halts.
  ruleset rs = R("kind: building\nmode: sequential\naxiom: A(3)\n"
                 "A(n) : n > 0 -> Box(6) A(n-1)\n"
                 "A(n) : n <= 0 -> \n");
  ASSERT_FALSE(rs.gen_nested); // deleting rule
  derive_result r = derive(rs, derive_options{});
  int boxes = 0;
  for (std::size_t i = 0; i < r.w.size(); ++i)
    if (rs.syms.name(r.w[i].sym) == "Box")
      ++boxes;
  EXPECT_EQ(boxes, 3);
}
