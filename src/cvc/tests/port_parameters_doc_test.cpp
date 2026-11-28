/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// docs/PORT_PARAMETERS.md holds the VolumeRover ports' parameter tables as
// cvc::params_markdown<P>() prints them. This test reads the doc from the source tree and checks
// each table against the generator, so a field added, renamed, re-ranged or re-documented in a
// param_table fails here until the doc's table is replaced with what the generator prints (a
// failure prints it). It also checks that the doc and the list below name the same structs, so a
// table added to the doc, or a struct added here, needs its other half. Runs natively and as a
// wasm-mt binary under node (ci.yml's ports lane).
//
// The tables are compared only where the standard library has floating std::to_chars. Without it
// (libc++ on Emscripten, older macOS targets) param_detail::format_double prints the shortest of
// %.15g .. %.17g, which spells some numbers longer (1000000000 for to_chars' 1e+09), so a range
// cannot match the doc there. The check on the struct list runs everywhere.

#include <charconv>
#include <cvc/core/param_table.h>
#include <cvc/geometry/pocket_tunnel_params.h>
#include <cvc/geometry/secondary_structures_params.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/gradient_vector_diffusion_params.h>
#include <cvc/volume/seed_generators_params.h>
#include <cvc/volume/segmentation_params.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_contours.h>
#include <cvc/volume/volume_grid_segment.h>
#include <cvc/volume/volume_slice.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

const std::string kDoc = std::string(CVC_REPO_DIR) + "/docs/PORT_PARAMETERS.md";

// Whether format_double prints through std::to_chars, as it did for the doc: its own condition.
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
constexpr bool kShortestNumbers = true;
#else
constexpr bool kShortestNumbers = false;
#endif

struct generated_table {
  std::string name; // the doc's "### <name>" heading: the struct's name without cvc::
  std::string markdown;
};

template <class P> generated_table table(const char *name) {
  return {name, cvc::params_markdown<P>()};
}

// Every table the doc holds, in its order.
std::vector<generated_table> generated() {
  return {
      table<cvc::skeleton_params>("skeleton_params"),
      table<cvc::tile_params>("tile_params"),
      table<cvc::seeded_segmentation_params>("seeded_segmentation_params"),
      table<cvc::gradient_vector_diffusion_params>("gradient_vector_diffusion_params"),
      table<cvc::capsid_seed_params>("capsid_seed_params"),
      table<cvc::rotation_seed_params>("rotation_seed_params"),
      table<cvc::volume_slice_params>("volume_slice_params"),
      table<cvc::volume_grid_params>("volume_grid_params"),
      table<cvc::volume_grid_display_params>("volume_grid_display_params"),
      table<cvc::volume_grid_source_params>("volume_grid_source_params"),
      table<cvc::volume_grid_segment_view_params>("volume_grid_segment_view_params"),
      table<cvc::class_surface_params>("class_surface_params"),
      table<cvc::volume_grid_contour_params>("volume_grid_contour_params"),
      table<cvc::grid_contour_object>("grid_contour_object"),
      table<cvc::pocket_tunnel_params>("pocket_tunnel_params"),
      table<cvc::secondary_params>("secondary_params"),
  };
}

// The doc's tables by heading: under each "### <name>", the first run of lines that start with
// '|', each ended with '\n' as params_markdown ends its rows. A checkout with CRLF line ends reads
// the same. `headings` gets every "### " heading in order, with or without a table.
std::map<std::string, std::string> doc_tables(std::vector<std::string> &headings) {
  std::map<std::string, std::string> tables;
  std::ifstream in(kDoc);
  std::string line, current;
  bool in_table = false;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.rfind("### ", 0) == 0) {
      current = line.substr(4);
      in_table = false;
      headings.push_back(current);
      tables[current];
      continue;
    }
    if (current.empty())
      continue;
    if (line.rfind('|', 0) == 0) {
      tables[current] += line + "\n";
      in_table = true;
    } else if (in_table) {
      current.clear();
      in_table = false;
    }
  }
  return tables;
}

TEST(PortParametersDoc, TheDocReads) {
  std::ifstream in(kDoc);
  ASSERT_TRUE(in.good()) << "cannot read " << kDoc;
}

TEST(PortParametersDoc, EachTableIsWhatTheGeneratorPrints) {
  if (!kShortestNumbers)
    GTEST_SKIP() << "no floating std::to_chars here, so format_double spells some numbers longer "
                    "than the doc does";
  std::vector<std::string> headings;
  const std::map<std::string, std::string> doc = doc_tables(headings);
  ASSERT_FALSE(headings.empty()) << kDoc << " has no \"### \" headings";
  for (const generated_table &t : generated()) {
    SCOPED_TRACE(t.name);
    const auto it = doc.find(t.name);
    if (it == doc.end()) {
      ADD_FAILURE() << "docs/PORT_PARAMETERS.md has no \"### " << t.name
                    << "\" section; cvc::params_markdown<cvc::" << t.name << ">() prints:\n"
                    << t.markdown;
      continue;
    }
    if (it->second != t.markdown)
      ADD_FAILURE() << "docs/PORT_PARAMETERS.md's table for " << t.name
                    << " is not what cvc::params_markdown<cvc::" << t.name
                    << ">() prints. Replace the doc's table with this:\n"
                    << t.markdown;
  }
}

TEST(PortParametersDoc, TheDocAndTheTestNameTheSameStructs) {
  std::vector<std::string> headings;
  doc_tables(headings);
  std::set<std::string> known;
  for (const generated_table &t : generated())
    EXPECT_TRUE(known.insert(t.name).second) << t.name << " is listed twice";
  std::set<std::string> seen;
  for (const std::string &h : headings) {
    EXPECT_TRUE(seen.insert(h).second) << "docs/PORT_PARAMETERS.md has \"### " << h << "\" twice";
    EXPECT_TRUE(known.count(h))
        << "docs/PORT_PARAMETERS.md has a \"### " << h
        << "\" table that this test does not check: add it to generated() in "
           "port_parameters_doc_test.cpp";
  }
  for (const std::string &name : known)
    EXPECT_TRUE(seen.count(name)) << "docs/PORT_PARAMETERS.md has no \"### " << name
                                  << "\" section for cvc::params_markdown<cvc::" << name << ">()";
}

} // namespace
