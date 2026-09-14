/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.
  Licensed under the GNU LGPL v2.1 (see cvc/lsys/emit.h for the full header).
*/

// The rendered SVG element count is a deterministic function of the interpreted
// geometry, so we assert it two independent ways: from the structure counts and
// from the rendered element tags. (This is what makes PR A's visual output a
// tested artifact rather than a decoration.)

#include <cvc/lsys/derive.h>
#include <cvc/lsys/emit.h>
#include <cvc/lsys/interp.h>
#include <cvc/lsys/recipes.h>
#include <gtest/gtest.h>
#include <string>

using namespace cvc::lsys;

namespace {
std::size_t count_occurrences(const std::string &hay, const std::string &needle) {
  std::size_t c = 0, p = 0;
  while ((p = hay.find(needle, p)) != std::string::npos) {
    ++c;
    ++p;
  }
  return c;
}
} // namespace

TEST(LsysSvg, ElementCountMatchesGeometry) {
  ruleset rs = load_recipe("pine_monopodial");
  derive_options o;
  o.generations = 6;
  structure s = interpret(rs, derive(rs, o).w);

  std::string svg = emit_svg(s, svg_options{});
  const std::size_t lines = count_occurrences(svg, "<line");
  const std::size_t circles = count_occurrences(svg, "<circle");
  const std::size_t rects = count_occurrences(svg, "<rect") - 1; // one background rect

  EXPECT_EQ(lines, s.segments.size());
  EXPECT_EQ(circles, s.leaves.size());
  EXPECT_EQ(rects, s.boxes.size());
  EXPECT_EQ(lines + circles + rects, svg_element_count(s));
  EXPECT_NE(svg.find("<svg"), std::string::npos);
  EXPECT_NE(svg.find("</svg>"), std::string::npos);
}

TEST(LsysSvg, BuildingRendersRects) {
  ruleset rs = load_recipe("office_block");
  structure s = interpret(rs, derive(rs, derive_options{}).w);
  std::string svg = emit_svg(s, svg_options{});
  EXPECT_EQ(count_occurrences(svg, "<rect") - 1, s.boxes.size());
  EXPECT_GT(s.boxes.size(), std::size_t(0));
}

TEST(LsysSvg, StatsCountBySymbolAndLevel) {
  ruleset rs = load_recipe("pine_monopodial");
  derive_options o;
  o.generations = 5;
  word w = derive(rs, o).w;
  structure s = interpret(rs, w);
  stats st = compute_stats(rs, w, s);
  EXPECT_EQ(st.modules, w.size());
  EXPECT_EQ(st.segments, s.segments.size());
  EXPECT_GT(st.by_symbol.count("F"), std::size_t(0)); // the forward op appears
  std::uint32_t total = 0;
  for (auto v : st.level_counts)
    total += v;
  EXPECT_EQ(total, w.size());
}

TEST(LsysSvg, ColorModesAndProjectionsRender) {
  ruleset rs = load_recipe("oak_sympodial");
  structure s = interpret(rs, derive(rs,
                                     [] {
                                       derive_options o;
                                       o.generations = 4;
                                       return o;
                                     }())
                                  .w);
  for (svg_color col : {svg_color::order, svg_color::level, svg_color::klass}) {
    for (projection pr : {projection::front, projection::side, projection::top}) {
      svg_options so;
      so.color = col;
      so.proj = pr;
      std::string svg = emit_svg(s, so);
      EXPECT_EQ(count_occurrences(svg, "<line"), s.segments.size());
      EXPECT_NE(svg.find("</svg>"), std::string::npos);
    }
  }
}

TEST(LsysSvg, MeshOffIsWellFormed) {
  ruleset rs = load_recipe("pine_monopodial");
  structure s = interpret(rs, derive(rs,
                                     [] {
                                       derive_options o;
                                       o.generations = 4;
                                       return o;
                                     }())
                                  .w);
  std::string off = emit_off(s);
  EXPECT_EQ(off.compare(0, 3, "OFF"), 0);
  // Header line "OFF\n<V> <F> 0"; V and F must be > 0 for a tree with segments.
  EXPECT_GT(s.segments.size(), std::size_t(0));
  EXPECT_NE(off.find('\n'), std::string::npos);
}
