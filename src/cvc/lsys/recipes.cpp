/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <cvc/lsys/parse.h>
#include <cvc/lsys/recipes.h>
#include <map>
#include <stdexcept>

namespace cvc {
namespace lsys {

namespace {

// ── Trees & shrubs (parallel, turtle alphabet) ───────────────────────────────

// Excurrent conifer: strong central leader (A), whorled lateral branches (B).
const char *kPine = R"(# cvc lsystem v1 -- excurrent conifer [Honda 1971]
name: pine_monopodial
kind: plant
mode: parallel
angle: 20
tilt: 35
roll: 90
step: 6
width: 0.8
taper: 0.14
param r = 0.7
axiom: A(1)
A(s) : s > 0.08 -> !(s*0.8) ;(0) F(6*s) [ &(38) B(s*0.5) ] [ &(38) / / / B(s*0.5) ] / A(s*0.82)
B(s) : s > 0.05 -> !(s*0.4) ;(1) F(4*s) [ +(20) B(s*r) ] [ -(20) B(s*r) ] ;(2) L(2.5*s)
)";

// Sympodial spreading deciduous: ternary re-branching [Prusinkiewicz 2003].
const char *kOak = R"(# cvc lsystem v1 -- sympodial spreading deciduous
name: oak_sympodial
kind: plant
mode: parallel
angle: 30
tilt: 28
roll: 94
step: 5
width: 1
taper: 0.16
build_gen: 6
param r = 0.72
axiom: A(1)
A(s) : s > 0.1 -> !(s) ;(0) F(5*s) [ &(30) / A(s*r) ] [ &(30) / / / A(s*r) ] [ &(30) / / / / / A(s*r) ] ;(2) L(3*s)
)";

// Slender birch: gently forking, thin.
const char *kBirch = R"(# cvc lsystem v1 -- slender birch
name: birch_slender
kind: plant
mode: parallel
angle: 12
tilt: 20
roll: 90
step: 7
width: 0.5
taper: 0.12
build_gen: 9
param r = 0.8
axiom: A(1)
A(s) : s > 0.05 -> !(s*0.5) ;(0) F(7*s) [ +(12) B(s*0.4) ] [ -(12) B(s*0.4) ] A(s*0.88)
B(s) : s > 0.04 -> !(s*0.3) ;(1) F(3*s) ;(2) L(1.5*s)
)";

// Dense low shrub.
const char *kShrub = R"(# cvc lsystem v1 -- dense shrub
name: shrub_bush
kind: plant
mode: parallel
angle: 35
tilt: 30
roll: 90
step: 2
width: 0.3
taper: 0.18
build_gen: 6
param r = 0.7
axiom: A(1)
A(s) : s > 0.08 -> !(s*0.4) ;(1) F(2*s) [ +(35) A(s*r) ] [ -(35) A(s*r) ] [ &(35) A(s*r) ] ;(2) L(2*s)
)";

// ── Rocks & buildings (sequential, scope alphabet) ───────────────────────────

// A boulder: three overlapping fractured chunks [Peytavie 2009 -- visual fake].
const char *kBoulder = R"(# cvc lsystem v1 -- boulder cluster (visual, not stable)
name: boulder_cluster
kind: rock
mode: sequential
axiom: Rock
Rock -> [ Scale(1.4, 1.1, 0.9) Box(3) ] [ Trans(0.5, 0.3, 0.3) Rot(18, 12, 6) Scale(0.9, 1, 0.7) Box(3) ] [ Trans(-0.4, 0.4, 0.2) Rot(-16, 24, 0) Scale(0.7, 0.7, 1) Box(3) ]
)";

// A multi-storey reinforced-concrete office block.
const char *kOffice = R"(# cvc lsystem v1 -- office block (concrete)
name: office_block
kind: building
mode: sequential
axiom: Scale(14, 11, 3) Bldg(6)
Bldg(n) : n > 0 -> Box(6) Trans(0, 0, 3) Bldg(n-1)
)";

// A small two-storey brick house.
const char *kHouse = R"(# cvc lsystem v1 -- brick house
name: brick_house
kind: building
mode: sequential
axiom: Scale(9, 7, 3) House(2)
House(n) : n > 0 -> Box(7) Trans(0, 0, 3) House(n-1)
)";

const std::map<std::string, const char *> &table() {
  static const std::map<std::string, const char *> t = {
      {"pine_monopodial", kPine}, {"oak_sympodial", kOak},       {"birch_slender", kBirch},
      {"shrub_bush", kShrub},     {"boulder_cluster", kBoulder}, {"office_block", kOffice},
      {"brick_house", kHouse},
  };
  return t;
}

} // namespace

std::vector<std::string> recipe_names() {
  std::vector<std::string> out;
  for (const auto &kv : table())
    out.push_back(kv.first);
  return out;
}

bool has_recipe(const std::string &name) { return table().count(name) != 0; }

std::string recipe_source(const std::string &name) {
  auto it = table().find(name);
  return it == table().end() ? std::string() : std::string(it->second);
}

ruleset load_recipe(const std::string &name) {
  auto it = table().find(name);
  if (it == table().end())
    throw std::out_of_range("unknown lsys recipe: " + name);
  parse_result pr = parse_lsys(it->second);
  if (!pr.ok) {
    std::string msg = "built-in recipe failed to parse: " + name;
    for (const diagnostic &d : pr.diags)
      if (d.sev == diagnostic::level::error)
        msg += "\n  line " + std::to_string(d.line) + ": " + d.message;
    throw std::runtime_error(msg);
  }
  return pr.rs;
}

} // namespace lsys
} // namespace cvc
