/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph composition (inc/cvc/ph/compose.h) against its reference implementation, CVC-Lab/phast
// (C-PHAST's code, by Shubham Bhardwaj; docs/roadmap/PH-COMPOSE-ROADMAP.md).
//
// tests/data/ph_compose_golden.json was generated once, outside CI, by
// src/cvc/tools/cphast_fixtures.py from phast's own torch modules in float64: four DC microgrids
// (linear DGUs and RL lines on a chain, the same network with a fifth DGU added at bus 0 or bus 2,
// and three copies of a learned DGU law), each with its "cvc-ph-system/1" description, 200 steps of
// phast's Euler-coupled split step, the stored energy along the way, and (learned) energies and
// efforts at 16 states. Known laws must match to rounding; learned laws (Picard solves) closely.
// The structural and accuracy checks that need no golden are in ph_compose_test.cpp.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/ph/compose.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace pt = boost::property_tree;
using namespace cvc::ph;

fs::path golden_path() {
  return fs::path(__FILE__).parent_path() / "data" / "ph_compose_golden.json";
}

const pt::ptree &golden() {
  static const pt::ptree t = [] {
    pt::ptree r;
    pt::read_json(golden_path().string(), r);
    return r;
  }();
  return t;
}

vecd numbers(const pt::ptree &t) {
  vecd out;
  if (t.empty()) {
    out.push_back(std::stod(t.data()));
    return out;
  }
  for (const auto &kv : t) {
    vecd v = numbers(kv.second);
    out.insert(out.end(), v.begin(), v.end());
  }
  return out;
}

std::string sci(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.3e", v);
  return b;
}

std::string to_json(const pt::ptree &t) {
  std::ostringstream o;
  pt::write_json(o, t, false);
  return o.str();
}

const pt::ptree &golden_case(const std::string &name) {
  for (const auto &kv : golden().get_child("cases"))
    if (kv.second.get<std::string>("name") == name)
      return kv.second;
  throw std::runtime_error("no golden case " + name);
}

// Max |a - b| over a rollout against the golden states.
struct parity {
  double state = 0, energy = 0;
};

parity replay(const pt::ptree &c) {
  const composite s = parse_system(to_json(c.get_child("system")));
  transition_opts o;
  o.dt = c.get<double>("dt");
  o.substeps = c.get<int>("substeps");
  o.coupling = coupling_scheme_of(c.get<std::string>("coupling"));
  if (auto sv = c.get_child_optional("solve")) {
    o.solve.max_iter = sv->get<int>("max_iter");
    o.solve.tol = sv->get<double>("tol");
  }
  const int steps = c.get<int>("steps");
  const rollout_result r =
      rollout(s, o, numbers(c.get_child("x0")), numbers(c.get_child("u")), steps);
  parity p;
  int k = 0;
  for (const auto &row : c.get_child("x")) {
    const vecd want = numbers(row.second);
    EXPECT_EQ(want.size(), r.x[k].size());
    for (size_t i = 0; i < want.size(); ++i)
      p.state = std::max(p.state, std::fabs(want[i] - r.x[k][i]));
    ++k;
  }
  EXPECT_EQ(k, steps + 1);
  k = 0;
  for (const auto &h : c.get_child("H")) {
    p.energy = std::max(p.energy, std::fabs(std::stod(h.second.data()) - s.energy(r.x[k].data())));
    ++k;
  }
  return p;
}

TEST(PhComposeParity, KnownChainMatchesPhast) {
  const parity p = replay(golden_case("known_chain4"));
  RecordProperty("max_state_error", sci(p.state));
  EXPECT_LT(p.state, 1e-12);
  EXPECT_LT(p.energy, 1e-12);
}

TEST(PhComposeParity, AddedDgusMatchPhastAtEitherBus) {
  vecd end0, end2;
  for (const char *name : {"known_add_bus0", "known_add_bus2"}) {
    const parity p = replay(golden_case(name));
    RecordProperty(std::string("max_state_error_") + name, sci(p.state));
    EXPECT_LT(p.state, 1e-12) << name;
    EXPECT_LT(p.energy, 1e-12) << name;
  }
  // The two candidates share every law and start; only the receiving bus differs, and so do their
  // futures.
  const auto &a = golden_case("known_add_bus0"), &b = golden_case("known_add_bus2");
  EXPECT_EQ(to_json(a.get_child("x0")), to_json(b.get_child("x0")));
  EXPECT_EQ(to_json(a.get_child("system.laws")), to_json(b.get_child("system.laws")));
  const vecd xa = numbers(a.get_child("x").back().second),
             xb = numbers(b.get_child("x").back().second);
  double d = 0;
  for (size_t i = 0; i < xa.size(); ++i)
    d = std::max(d, std::fabs(xa[i] - xb[i]));
  EXPECT_GT(d, 1e-3);
}

TEST(PhComposeParity, LearnedLawsMatchPhast) {
  const pt::ptree &c = golden_case("learned_chain3");
  const composite s = parse_system(to_json(c.get_child("system")));
  double eH = 0, ee = 0;
  for (const auto &kv : c.get_child("points")) {
    const vecd x = numbers(kv.second.get_child("x")), want = numbers(kv.second.get_child("e"));
    vecd e(s.state_dim());
    s.effort(x.data(), e.data());
    for (size_t i = 0; i < e.size(); ++i)
      ee = std::max(ee, std::fabs(e[i] - want[i]));
    eH = std::max(eH, std::fabs(s.energy(x.data()) - kv.second.get<double>("H")));
  }
  RecordProperty("max_energy_error_points", sci(eH));
  RecordProperty("max_effort_error_points", sci(ee));
  EXPECT_LT(eH, 1e-13);
  EXPECT_LT(ee, 1e-13);
  const parity p = replay(c);
  RecordProperty("max_state_error", sci(p.state));
  EXPECT_LT(p.state, 1e-10);
  EXPECT_LT(p.energy, 1e-10);
}

} // namespace
