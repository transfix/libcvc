/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/expr.h for the full header).
*/

#include <cvc/lsys/expr.h>
#include <cvc/lsys/grammar.h>
#include <gtest/gtest.h>
#include <map>

using namespace cvc::lsys;

namespace {
double run(const char *src, std::map<std::string, double> env, bool *err = nullptr) {
  std::vector<diagnostic> d;
  expr e = compile_expr(src, &d, "t");
  if (err)
    *err = !d.empty();
  return e.eval([&](const std::string &n) {
    auto it = env.find(n);
    return it == env.end() ? 0.0 : it->second;
  });
}
} // namespace

TEST(LsysExpr, ArithmeticAndPrecedence) {
  EXPECT_DOUBLE_EQ(run("1+2*3", {}), 7.0);
  EXPECT_DOUBLE_EQ(run("(1+2)*3", {}), 9.0);
  EXPECT_DOUBLE_EQ(run("10-2-3", {}), 5.0);
  EXPECT_DOUBLE_EQ(run("8/2/2", {}), 2.0);
  EXPECT_DOUBLE_EQ(run("-s*0.5", {{"s", 4}}), -2.0);
}

TEST(LsysExpr, VariablesAndFunctions) {
  EXPECT_DOUBLE_EQ(run("s*r", {{"s", 2}, {"r", 0.7}}), 1.4);
  EXPECT_DOUBLE_EQ(run("pow(s,3)", {{"s", 2}}), 8.0);
  EXPECT_DOUBLE_EQ(run("min(s,n)", {{"s", 2}, {"n", 3}}), 2.0);
  EXPECT_DOUBLE_EQ(run("max(s,n)", {{"s", 2}, {"n", 3}}), 3.0);
  EXPECT_DOUBLE_EQ(run("clamp(5,0,3)", {}), 3.0);
  EXPECT_DOUBLE_EQ(run("abs(-4)", {}), 4.0);
  EXPECT_DOUBLE_EQ(run("sqrt(9)", {}), 3.0);
  EXPECT_DOUBLE_EQ(run("floor(2.9)", {}), 2.0);
}

TEST(LsysExpr, ComparisonsAndLogic) {
  EXPECT_DOUBLE_EQ(run("s>0", {{"s", 2}}), 1.0);
  EXPECT_DOUBLE_EQ(run("s<0", {{"s", 2}}), 0.0);
  EXPECT_DOUBLE_EQ(run("n>=3 && s>1", {{"n", 3}, {"s", 2}}), 1.0);
  EXPECT_DOUBLE_EQ(run("n==3", {{"n", 3}}), 1.0);
  EXPECT_DOUBLE_EQ(run("n!=3", {{"n", 3}}), 0.0);
  EXPECT_DOUBLE_EQ(run("0 || 1", {}), 1.0);
}

TEST(LsysExpr, EmptyIsAbsent) {
  std::vector<diagnostic> d;
  expr e = compile_expr("   ", &d, "e");
  EXPECT_TRUE(e.empty());
  EXPECT_TRUE(e.eval_guard(nullptr)); // empty guard == always true
  EXPECT_TRUE(d.empty());
}

TEST(LsysExpr, UnknownVariableIsZero) { EXPECT_DOUBLE_EQ(run("q+1", {}), 1.0); }

TEST(LsysExpr, SourceRetainedForRoundTrip) {
  std::vector<diagnostic> d;
  expr e = compile_expr("  s * r + 1 ", &d, "e");
  EXPECT_EQ(e.source(), "s * r + 1"); // trimmed, otherwise verbatim
}

TEST(LsysExpr, BadExpressionReportsError) {
  bool err = false;
  run("1 + * 2", {}, &err);
  EXPECT_TRUE(err);
}
