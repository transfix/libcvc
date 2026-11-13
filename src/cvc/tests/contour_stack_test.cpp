/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::contour_stack (cvc/geometry/contours.h, contour_intrinsics.h): the VolumeRover ports' P0d
// rows for it (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7).
//   - the model: add, components, slice planes, contour_problems;
//   - T8: contour_component's and contour_read_params' tables, struct to tree to struct, defaults
//     seeded, properties written, bad values reported;
//   - T9: every format round-trips, the legacy samples (src/cvc/tests/data/contours) read, and
//     truncated and garbage files raise with the file's name;
//   - T6: two apps keep their own handlers; two threads with different parameters read what each
//     reads alone; A then B equals B then A;
//   - the DSL: contours.info and params.* on stacks on the state tree (no intrinsic reads or writes
//     files), in an environment and from an Ariadne document through the app's own provider
//     (install_contour_intrinsics).
// Runs natively and under node (the ports lane, T14).

#include "../utility/xml_lite.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/param_dsl.h>
#include <cvc/geometry/contour_intrinsics.h>
#include <cvc/geometry/contours.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace se = cvc::state_exec;
using steady = std::chrono::steady_clock;

fs::path data_dir() { return fs::path(__FILE__).parent_path() / "data" / "contours"; }

// A fresh directory of its own for one test, removed afterwards.
struct scratch_dir {
  fs::path path;
  explicit scratch_dir(const std::string &name) {
    path = fs::temp_directory_path() / ("cvc_contours_" + name + "_" +
                                        std::to_string(steady::now().time_since_epoch().count()));
    fs::create_directories(path);
  }
  ~scratch_dir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  std::string file(const std::string &name) const { return (path / name).generic_string(); }
};

std::string slurp(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

void spit(const fs::path &p, const std::string &text) {
  std::ofstream out(p, std::ios::binary | std::ios::trunc);
  out << text;
}

cvc::contour_polygon poly(std::vector<double> xy) {
  cvc::contour_polygon p;
  p.xy = std::move(xy);
  return p;
}

// Three slices, two components, a hole, a hidden component and awkward numbers.
cvc::contour_stack sample_stack() {
  cvc::contour_stack s;
  s.z_spacing = 0.05;
  s.add(2, "axon", {0, 0, 1, 0, 1, 1, 0, 1});
  s.add(2, "axon", {0.25, 0.25, 0.25, 0.75, 0.75, 0.75, 0.75, 0.25}); // clockwise: a hole
  s.add(3, "axon", {0.1, 0.2, 1.0 / 3.0, 0.2, 0.3, 1e-7, 0.1, 0.9});
  s.add(3, "dendrite", {5, 5, 6, 5, 5.5, 6.25});
  s.add(5, "dendrite", {-5, -5, -4, -5, -4.5, -4.125});
  s.components[0].color = {1, 0, 0};
  s.components[1].color = {0, 0.5, 0.25};
  s.components[1].visible = false;
  return s;
}

void expect_near_xy(const cvc::contour_polygon &got, const std::vector<double> &want,
                    double tol = 1e-12) {
  ASSERT_EQ(got.xy.size(), want.size());
  for (std::size_t i = 0; i < want.size(); ++i)
    EXPECT_NEAR(got.xy[i], want[i], tol) << "coordinate " << i;
}

// ---- the model ----

TEST(ContourStack, AddKeepsComponentsInOrderAndSlicesByIndex) {
  cvc::contour_stack s = sample_stack();
  EXPECT_EQ(s.indices(), (std::vector<int>{2, 3, 5}));
  EXPECT_EQ(s.component_names(), (std::vector<std::string>{"axon", "dendrite"}));
  EXPECT_EQ(s.component_index("dendrite"), 1);
  EXPECT_EQ(s.component_index("nothing"), -1);
  EXPECT_EQ(s.polygons(2, "axon").size(), 2u);
  EXPECT_TRUE(s.polygons(4, "axon").empty());
  EXPECT_TRUE(s.polygons(2, "nothing").empty());
  EXPECT_EQ(s.polygon_count(), 5u);
  EXPECT_EQ(s.vertex_count(), 18u);
  EXPECT_DOUBLE_EQ(s.slice_z(3), 0.15);
  s.slices[3].z = 7.5;
  EXPECT_DOUBLE_EQ(s.slice_z(3), 7.5);
  s.z_origin = 1;
  EXPECT_DOUBLE_EQ(s.slice_z(4), 1.2);
  EXPECT_GT(s.polygons(2, "axon")[0].signed_area(), 0.0);
  EXPECT_LT(s.polygons(2, "axon")[1].signed_area(), 0.0);
  EXPECT_DOUBLE_EQ(s.polygons(2, "axon")[0].signed_area(), 1.0);
  cvc::contour_component c;
  c.name = "axon";
  c.color = {0, 0, 1};
  s.set_component(c);
  EXPECT_EQ(s.components.size(), 2u);
  EXPECT_EQ(s.components[0].color, (std::vector<double>{0, 0, 1}));
  c.name = "glia";
  s.set_component(c);
  EXPECT_EQ(s.component_names().back(), "glia");
  EXPECT_THROW(s.add(1, "", {0, 0, 1, 0, 0, 1}), std::invalid_argument);
  EXPECT_THROW(s.add(1, "axon", {0, 0, 1}), std::invalid_argument);
  c.name.clear();
  EXPECT_THROW(s.set_component(c), std::invalid_argument);
}

TEST(ContourStack, ProblemsNameTheSliceAndComponent) {
  EXPECT_TRUE(cvc::contour_problems(sample_stack()).empty());
  cvc::contour_stack s = sample_stack();
  s.slices[2].components["axon"].push_back(poly({0, 0, 1, 1}));
  s.slices[3].components["ghost"].push_back(poly({0, 0, 1, 0, 0, 1}));
  s.slices[5].components["dendrite"].push_back(poly({0, 0, NAN, 0, 0, 1}));
  s.components[1].color = {2, 0, 0};
  s.components.push_back(s.components[0]);
  s.z_spacing = 0;
  const std::vector<std::string> p = cvc::contour_problems(s);
  const auto has = [&](const std::string &what) {
    for (const std::string &x : p)
      if (x.find(what) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(has("z_spacing = 0 is not a positive number"));
  EXPECT_TRUE(has("two components are named 'axon'"));
  EXPECT_TRUE(has("component 'dendrite': its colour is not three values in [0, 1]"));
  EXPECT_TRUE(has("slice 2, component 'axon', polygon 2: 2 vertices; a polygon needs three"));
  EXPECT_TRUE(has("slice 3, component 'ghost': no such component"));
  EXPECT_TRUE(has("slice 5, component 'dendrite', polygon 1: a coordinate is not a number"));
}

// ---- T8: the tables on the state tree ----

TEST(ContourParams, ComponentRoundTripsThroughTheTree) {
  cvc::app app;
  cvc::contour_component c;
  c.name = "axon 7";
  c.color = {0.25, 1, 1.0 / 3.0};
  c.visible = false;
  cvc::state &node = app.root()("doc.components.axon");
  cvc::params_to_state(c, node);
  EXPECT_EQ(node("name").value(), "axon 7");
  EXPECT_EQ(node("visible").value(), "0");
  EXPECT_EQ(node("color").property("min"), "0");
  EXPECT_EQ(node("color").property("max"), "1");
  EXPECT_EQ(node("color").property("type"), "list");
  const cvc::param_result<cvc::contour_component> back =
      cvc::params_from_state<cvc::contour_component>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value, c) << "bit for bit, the third of a colour included";
  // A bad value is reported, not replaced by the default.
  node("color").value("[2, 0, 0]");
  node("visible").value("maybe");
  const auto bad = cvc::params_from_state<cvc::contour_component>(node);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("visible: 'maybe' is not a bool"), std::string::npos)
      << bad.message();
  EXPECT_NE(bad.message().find("color[0] = 2 is above its maximum 1"), std::string::npos)
      << bad.message();
  // The table's own check: a name, and three colour values.
  cvc::contour_component empty;
  empty.color = {1, 1};
  const std::vector<std::string> v = cvc::params_validate(empty);
  ASSERT_EQ(v.size(), 2u);
  EXPECT_EQ(v[0], "name: a component needs a name");
  EXPECT_EQ(v[1], "color: 2 values, not three (r, g, b)");
}

TEST(ContourParams, ReadParamsSeedDefaultsAndRoundTrip) {
  cvc::app app;
  cvc::state &node = app.root()("doc.read");
  // Missing keys are seeded with the defaults, with their properties.
  const auto seeded = cvc::params_from_state<cvc::contour_read_params>(node);
  ASSERT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(node("first").value(), "-1");
  EXPECT_EQ(node("min_points").value(), "3");
  EXPECT_EQ(node("apply_transforms").value(), "1");
  EXPECT_EQ(node("include").value(), "[]");
  EXPECT_EQ(node("first").property("min"), "-1");
  EXPECT_FALSE(node("first").has_property("max")) << "no upper bound";
  EXPECT_FALSE(node("z_spacing").property("doc").empty());
  // Every field, struct to tree to struct.
  cvc::contour_read_params p;
  p.first = 4;
  p.last = 9;
  p.include = {"d0[0-9]", "axon.*"};
  p.exclude = {"domain1"};
  p.min_points = 5;
  p.z_spacing = 0.07;
  p.apply_transforms = false;
  p.keep_open = true;
  cvc::params_to_state(p, node);
  const auto back = cvc::params_from_state<cvc::contour_read_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.first, 4);
  EXPECT_EQ(back.value.last, 9);
  EXPECT_EQ(back.value.include, p.include);
  EXPECT_EQ(back.value.exclude, p.exclude);
  EXPECT_EQ(back.value.min_points, 5);
  EXPECT_EQ(back.value.z_spacing, 0.07);
  EXPECT_FALSE(back.value.apply_transforms);
  EXPECT_TRUE(back.value.keep_open);
  // And through the DSL's dict.
  const auto dict = cvc::params_from_value<cvc::contour_read_params>(cvc::params_to_dict(p));
  ASSERT_TRUE(dict.ok()) << dict.message();
  EXPECT_EQ(dict.value.include, p.include);
  EXPECT_EQ(dict.value.z_spacing, p.z_spacing);
  // Out of range, a first after last, and a pattern that isn't a regular expression.
  p.first = 9;
  p.last = 4;
  p.min_points = -2;
  p.include = {"a("};
  const std::vector<std::string> v = cvc::params_validate(p);
  const std::string all = [&] {
    std::string s;
    for (const std::string &x : v)
      s += x + "\n";
    return s;
  }();
  EXPECT_NE(all.find("min_points = -2 is below its minimum 0"), std::string::npos) << all;
  EXPECT_NE(all.find("first = 9 is after last = 4"), std::string::npos) << all;
  EXPECT_NE(all.find("include[0]: 'a(' is not a regular expression"), std::string::npos) << all;
}

// ---- the XML reader under the Reconstruct format ----

TEST(XmlLite, ParsesWhatReconstructWrites) {
  const cvc::xml_lite::element e = cvc::xml_lite::parse(
      "\xEF\xBB\xBF<?xml version=\"1.0\"?>\n<!DOCTYPE Section SYSTEM \"section.dtd\" [\n"
      "  <!ENTITY x \"]>\">\n]>\n<!-- a comment -->\n"
      "<Section index='3' thickness=\"0.05\">\n"
      "  <Transform xcoef=\" 0 1 0 0 0 0\">\n"
      "    <Contour name=\"a &amp; b &#x3b1;&#946;\" points=\"1 2,&#10;3 4\"/>\n"
      "  </Transform>\n"
      "  <Note><![CDATA[<raw> & text]]> more</Note>\n"
      "</Section>\n<!-- after -->\n");
  EXPECT_EQ(e.name, "Section");
  EXPECT_EQ(e.attribute("index", ""), "3");
  EXPECT_EQ(e.attribute("missing", "fallback"), "fallback");
  ASSERT_EQ(e.children.size(), 2u);
  const cvc::xml_lite::element *c = e.children[0].child("Contour");
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->attribute("name", ""), "a & b \xCE\xB1\xCE\xB2");
  EXPECT_EQ(c->attribute("points", ""), "1 2,\n3 4");
  EXPECT_EQ(c->line, 8u);
  EXPECT_EQ(e.child("Note")->text, "<raw> & text more");
  EXPECT_EQ(cvc::xml_lite::escape("<a href=\"x\">&'"), "&lt;a href=&quot;x&quot;&gt;&amp;&apos;");
  const auto fails = [](const std::string &doc, const std::string &says) {
    try {
      cvc::xml_lite::parse(doc);
      ADD_FAILURE() << "parsed: " << doc;
    } catch (const cvc::xml_lite::parse_error &err) {
      EXPECT_NE(std::string(err.what()).find(says), std::string::npos) << err.what();
    }
  };
  fails("", "no root element");
  fails("<a><b></a>", "</a> closes <b>");
  fails("<a x=1/>", "is not quoted");
  fails("<a x=\"1\" x=\"2\"/>", "appears twice");
  fails("<a>&bogus;</a>", "unknown entity");
  fails("<a>\n\n<b>", "is not closed at line 3");
  fails("<a/><b/>", "more after the root element");
  fails("<a><!-- open", "an unterminated comment");
}

// ---- T9: the formats ----

class ContourFiles : public ::testing::Test {
protected:
  cvc::app app;
};

TEST_F(ContourFiles, TheAppReadsAndWritesEveryBuiltInFormat) {
  EXPECT_EQ(cvc::contour_read_extensions(app),
            (std::vector<std::string>{".config", ".contour", ".cvcc", ".ser"}));
  EXPECT_EQ(cvc::contour_write_extensions(app),
            (std::vector<std::string>{".config", ".contour", ".cvcc", ".pcd", ".ser"}));
}

TEST_F(ContourFiles, NativeRoundTripsEverythingBitForBit) {
  scratch_dir dir("native");
  cvc::contour_stack s = sample_stack();
  s.z_origin = -0.125;
  s.slices[5].z = 3.0 / 7.0;
  s.slices[9]; // an empty slice
  s.components.push_back(cvc::contour_component{"unused \"quoted\"\tname", {0.5, 0.5, 0.5}, true});
  cvc::write_contours(app, s, dir.file("s.cvcc"));
  const cvc::contour_stack back = cvc::read_contours(app, dir.file("s.cvcc"));
  EXPECT_EQ(back, s);
  // Upper-case extensions find the same handler.
  cvc::write_contours(app, s, dir.file("UPPER.CVCC"));
  EXPECT_EQ(cvc::read_contours(app, dir.file("UPPER.CVCC")), s);
}

TEST_F(ContourFiles, ReconstructRoundTripsGeometryColoursAndVisibility) {
  scratch_dir dir("ser");
  const cvc::contour_stack s = sample_stack();
  cvc::write_contours(app, s, dir.file("series.ser"));
  for (int n : {2, 3, 5})
    EXPECT_TRUE(fs::exists(dir.path / ("series." + std::to_string(n))));
  const cvc::contour_stack back = cvc::read_contours(app, dir.file("series.ser"));
  EXPECT_EQ(back, s) << "an identity transform reads every coordinate back exactly";
  // Writing a series replaces it: a section the new stack lacks goes.
  cvc::contour_stack fewer = s;
  fewer.slices.erase(5);
  cvc::write_contours(app, fewer, dir.file("series.ser"));
  EXPECT_FALSE(fs::exists(dir.path / "series.5"));
  EXPECT_EQ(cvc::read_contours(app, dir.file("series.ser")).indices(), (std::vector<int>{2, 3}));
  // Reconstruct numbers its sections from 0.
  cvc::contour_stack negative = s;
  negative.add(-1, "axon", {0, 0, 1, 0, 0, 1});
  EXPECT_THROW(cvc::write_contours(app, negative, dir.file("neg.ser")), cvc::contour_file_error);
}

TEST_F(ContourFiles, TilingConfigRoundTripsOneComponent) {
  scratch_dir dir("config");
  cvc::contour_stack s;
  s.z_spacing = 0.5;
  s.add(1, "stack", {0, 0, 1, 0, 1, 1.25, 0, 1.25});
  s.add(1, "stack", {3, 3, 4, 3, 3.5, 4});
  s.add(3, "stack", {0.1, 0.1, 0.9, 0.1, 0.9, 1.1});
  cvc::write_contours(app, s, dir.file("stack.config"));
  EXPECT_TRUE(fs::exists(dir.path / "stack2.pts")) << "every slice in the range has a file";
  cvc::contour_stack back = cvc::read_contours(app, dir.file("stack.config"));
  cvc::contour_stack want = s;
  want.slices[2]; // the empty slice in between, which the program needs
  EXPECT_EQ(back, want);
  // Unevenly spaced slices keep their planes.
  s.slices[3].z = 10;
  cvc::write_contours(app, s, dir.file("stack.config"));
  back = cvc::read_contours(app, dir.file("stack.config"));
  for (int n : {1, 2, 3})
    EXPECT_NEAR(back.slice_z(n), s.slice_z(n), 1e-12) << "slice " << n;
  EXPECT_THROW(cvc::write_contours(app, s, dir.file("has space.config")), cvc::contour_file_error);
}

TEST_F(ContourFiles, Ctr2sufRoundTripsMaterialsAndOrientation) {
  scratch_dir dir("ctr2suf");
  const cvc::contour_stack s = sample_stack();
  cvc::write_contours(app, s, dir.file("s.contour"));
  const cvc::contour_stack back = cvc::read_contours(app, dir.file("s.contour"));
  // Names and colours aren't in the format: component k is material k + 1.
  EXPECT_EQ(back.component_names(), (std::vector<std::string>{"material_1", "material_2"}));
  EXPECT_EQ(back.indices(), s.indices());
  EXPECT_DOUBLE_EQ(back.z_spacing, 0.05);
  for (int n : s.indices())
    for (std::size_t c = 0; c < s.components.size(); ++c) {
      const auto want = s.polygons(n, s.components[c].name);
      const auto got = back.polygons(n, "material_" + std::to_string(c + 1));
      ASSERT_EQ(got.size(), want.size()) << "slice " << n << " component " << c;
      for (std::size_t i = 0; i < want.size(); ++i)
        EXPECT_EQ(got[i], want[i]) << "the hole stays clockwise";
    }
}

TEST_F(ContourFiles, PointCloudListsEveryVertex) {
  scratch_dir dir("pcd");
  cvc::contour_stack s;
  s.z_spacing = 0.5;
  s.add(2, "a", {0, 0, 1, 0, 0.5, 1});
  s.add(2, "b", {5, 5, 6, 5, 5, 6});
  s.add(4, "a", {0.25, 0, 1, 0, 1, 1});
  cvc::write_contours(app, s, dir.file("s.pcd"));
  EXPECT_EQ(slurp(dir.path / "s.pcd"),
            "9\n0 0 1\n1 0 1\n0.5 1 1\n5 5 1\n6 5 1\n5 6 1\n0.25 0 2\n1 0 2\n1 1 2\n");
  EXPECT_THROW(cvc::read_contours(app, dir.file("s.pcd")), cvc::contour_file_error)
      << "a point cloud is written only";
}

TEST_F(ContourFiles, TheLegacyReconstructSeriesReads) {
  const std::string ser = (data_dir() / "reconstruct" / "series.ser").generic_string();
  const cvc::contour_stack s = cvc::read_contours(app, ser);
  EXPECT_DOUBLE_EQ(s.z_spacing, 0.05);
  EXPECT_EQ(s.indices(), (std::vector<int>{1, 2, 3}));
  ASSERT_EQ(s.component_names(), (std::vector<std::string>{"d01", "spine & head"}));
  EXPECT_EQ(s.components[0].color, (std::vector<double>{1, 0, 0}));
  EXPECT_TRUE(s.components[0].visible);
  EXPECT_EQ(s.components[1].color, (std::vector<double>{0, 0.5, 1}));
  EXPECT_FALSE(s.components[1].visible);
  // The image's outline and the open ruler aren't objects.
  EXPECT_EQ(s.polygon_count(), 4u);
  // Section 1: a translation, and the repeated last point dropped.
  expect_near_xy(s.polygons(1, "d01").at(0), {1, 3, 2, 3, 2, 4, 1, 4}, 0);
  // Section 2: a scale. The legacy reader's -a0 + a1 x gave (5, 4) here, not (1, 1).
  expect_near_xy(s.polygons(2, "d01").at(0), {1, 1, 2, 1, 2, 2, 1, 2}, 0);
  expect_near_xy(s.polygons(2, "spine & head").at(0), {7, 7, 8, 7, 7.5, 8}, 0);
  // Section 3: a polynomial, inverted by Newton's method.
  expect_near_xy(s.polygons(3, "d01").at(0), {10, 10, 20, 10, 20, 20, 10, 20}, 1e-10);
  // As stored, and with the open trace.
  cvc::contour_read_params raw;
  raw.apply_transforms = false;
  raw.keep_open = true;
  raw.min_points = 2;
  const cvc::contour_stack r = cvc::read_contours(app, ser, raw);
  expect_near_xy(r.polygons(2, "d01").at(0), {3, 2, 5, 2, 5, 4, 3, 4}, 0);
  expect_near_xy(r.polygons(2, "ruler").at(0), {0, 0, 10, 0}, 0);
}

TEST_F(ContourFiles, TheLegacyTilingSeriesRead) {
  const cvc::contour_stack s =
      cvc::read_contours(app, (data_dir() / "tiling" / "slices.config").generic_string());
  EXPECT_EQ(s.component_names(), (std::vector<std::string>{"slice"}));
  EXPECT_EQ(s.indices(), (std::vector<int>{1, 2, 3}));
  EXPECT_DOUBLE_EQ(s.z_spacing, 0.5) << "DETZ replaces the files' z";
  EXPECT_DOUBLE_EQ(s.z_origin, 0.0);
  ASSERT_EQ(s.polygons(1, "slice").size(), 2u);
  expect_near_xy(s.polygons(1, "slice")[1], {3.7, 2.3, 4.8, 3.4, -2.9, -19}, 0);
  EXPECT_TRUE(s.slices.at(3).components.empty()) << "slice 3 is one point giving its z";
  // No DETZ: the planes are the files' z = 0, 1, 3.
  const cvc::contour_stack u =
      cvc::read_contours(app, (data_dir() / "tiling" / "uneven.config").generic_string());
  EXPECT_DOUBLE_EQ(u.slice_z(1), 0.0);
  EXPECT_DOUBLE_EQ(u.slice_z(2), 1.0);
  EXPECT_DOUBLE_EQ(u.slice_z(3), 3.0);
  EXPECT_DOUBLE_EQ(u.z_spacing, 1.5);
  EXPECT_FALSE(std::isnan(u.slices.at(2).z)) << "slice 2 is off the spacing";
  EXPECT_TRUE(std::isnan(u.slices.at(3).z));
}

TEST_F(ContourFiles, TheLegacyVolumeGridRoverContourReads) {
  const cvc::contour_stack s =
      cvc::read_contours(app, (data_dir() / "ctr2suf" / "object.contour").generic_string());
  EXPECT_EQ(s.component_names(), (std::vector<std::string>{"material_1"}));
  EXPECT_EQ(s.indices(), (std::vector<int>{1, 2}));
  EXPECT_DOUBLE_EQ(s.z_spacing, 1.0);
  expect_near_xy(s.polygons(1, "material_1").at(0), {0, 0, 1, 0, 1, 1, 0, 1}, 0);
  expect_near_xy(s.polygons(2, "material_1").at(0), {0, 0, 1, 0, 0.5, 1}, 0);
}

TEST_F(ContourFiles, ReadParamsSelectSlicesComponentsAndPolygons) {
  const std::string ser = (data_dir() / "reconstruct" / "series.ser").generic_string();
  cvc::contour_read_params p;
  p.first = 2;
  p.last = 2;
  EXPECT_EQ(cvc::read_contours(app, ser, p).indices(), (std::vector<int>{2}));
  p = {};
  p.include = {"d0[0-9]"};
  EXPECT_EQ(cvc::read_contours(app, ser, p).component_names(), (std::vector<std::string>{"d01"}));
  p = {};
  p.exclude = {"d.*"};
  EXPECT_EQ(cvc::read_contours(app, ser, p).component_names(),
            (std::vector<std::string>{"spine & head"}));
  p = {};
  p.min_points = 4;
  const cvc::contour_stack four = cvc::read_contours(app, ser, p);
  EXPECT_TRUE(four.polygons(2, "spine & head").empty()) << "the triangle is dropped";
  EXPECT_EQ(four.indices(), (std::vector<int>{1, 2, 3})) << "its slice is kept";
  p = {};
  p.z_spacing = 0.07;
  EXPECT_DOUBLE_EQ(cvc::read_contours(app, ser, p).z_spacing, 0.07);
  p = {};
  p.include = {"("};
  EXPECT_THROW(cvc::read_contours(app, ser, p), std::invalid_argument);
}

// Each case: a file name, its text, and what the error must say.
struct bad_file {
  std::string name, text, says;
};

TEST_F(ContourFiles, TruncatedAndGarbageFilesRaise) {
  scratch_dir dir("bad");
  cvc::write_contours(app, sample_stack(), dir.file("good.cvcc"));
  const std::string json = slurp(dir.path / "good.cvcc");
  cvc::write_contours(app, sample_stack(), dir.file("good.contour"));
  const std::string ctr = slurp(dir.path / "good.contour");
  const std::string series = slurp(data_dir() / "reconstruct" / "series.ser");
  const std::string section = slurp(data_dir() / "reconstruct" / "series.1");
  const std::vector<bad_file> cases = {
      {"half.cvcc", json.substr(0, json.size() / 2), "not JSON"},
      {"garbage.cvcc", "\x01\x02 not json at all", "not JSON"},
      {"other.cvcc", "{\"format\": \"something\"}", "not a libcvc contour stack"},
      {"newer.cvcc", "{\"format\": \"cvc.contour_stack\", \"version\": 2, \"z_spacing\": 1}",
       "version 2 is newer"},
      {"nan.cvcc", "{\"format\": \"cvc.contour_stack\", \"version\": 1, \"z_spacing\": \"abc\"}",
       "z_spacing: 'abc' is not a number"},
      {"half.contour", ctr.substr(0, ctr.size() / 2), "the file ends before"},
      {"tilted.contour", "1\n1 0 1 2\n0 0\n", "is not a plane z = constant"},
      {"open.contour", "1\n0 0 1 1\n3 2\n0 0 1\n1 0 1\n0 1 1\n0 1 1 0\n1 2 1 0\n",
       "the edges of material 1 don't close"},
      {"letters.contour", "1\n0 0 1 zz\n", "'zz' is not a number"},
      {"half.ser", series.substr(0, series.size() / 2), "is not XML"},
      {"plain.ser", "<NotASeries/>", "not a Reconstruct series"},
      {"missing.config", "SUFFIX: .pts\nSLICE_RANGE: 1 2\n", "no PREFIX:"},
      {"norange.config", "PREFIX: slice\n", "no SLICE_RANGE:"},
      {"backwards.config", "PREFIX: slice\nSLICE_RANGE: 3 1\n", "is not a range"},
      {"absent.config", "PREFIX: absent\nSLICE_RANGE: 1 1\n", "can't open"},
  };
  for (const bad_file &b : cases) {
    spit(dir.path / b.name, b.text);
    try {
      cvc::read_contours(app, dir.file(b.name));
      ADD_FAILURE() << b.name << " read";
    } catch (const cvc::contour_file_error &e) {
      const std::string what = e.what();
      EXPECT_NE(what.find(b.says), std::string::npos) << b.name << ": " << what;
      EXPECT_NE(what.find(b.name), std::string::npos) << b.name << ": names the file: " << what;
    }
  }
  // A series whose sections are broken: half a section, odd coordinates, a bad coefficient.
  spit(dir.path / "broken.ser", series);
  spit(dir.path / "broken.1", section.substr(0, section.size() / 2));
  EXPECT_THROW(cvc::read_contours(app, dir.file("broken.ser")), cvc::contour_file_error);
  spit(dir.path / "odd.ser", series);
  spit(dir.path / "odd.1", "<Section thickness=\"0.05\"><Transform><Contour name=\"a\" "
                           "points=\"1 2, 3 4, 5\"/></Transform></Section>");
  EXPECT_THROW(cvc::read_contours(app, dir.file("odd.ser")), cvc::contour_file_error);
  spit(dir.path / "coef.ser", series);
  spit(dir.path / "coef.1", "<Section><Transform xcoef=\"0 1 x 0 0 0\"><Contour name=\"a\" "
                            "points=\"1 2, 3 4, 5 6\"/></Transform></Section>");
  EXPECT_THROW(cvc::read_contours(app, dir.file("coef.ser")), cvc::contour_file_error);
  // A .pts file with letters, and one whose vertices aren't on one plane.
  spit(dir.path / "p.config", "PREFIX: p\nSLICE_RANGE: 1 1\n");
  spit(dir.path / "p1.pts", "3\n0 0 0\n1 0 0\n0 one 0\n");
  EXPECT_THROW(cvc::read_contours(app, dir.file("p.config")), cvc::contour_file_error);
  spit(dir.path / "p1.pts", "3\n0 0 0\n1 0 0\n0 1 0.5\n");
  EXPECT_THROW(cvc::read_contours(app, dir.file("p.config")), cvc::contour_file_error);
  // A missing file, an unknown extension, and no extension.
  EXPECT_THROW(cvc::read_contours(app, dir.file("nothing.cvcc")), cvc::contour_file_error);
  EXPECT_THROW(cvc::read_contours(app, dir.file("x.xyz")), cvc::contour_file_error);
  EXPECT_THROW(cvc::read_contours(app, dir.file("noext")), cvc::contour_file_error);
  // A stack with problems isn't written.
  cvc::contour_stack s = sample_stack();
  s.slices[2].components["axon"].push_back(poly({0, 0}));
  EXPECT_THROW(cvc::write_contours(app, s, dir.file("bad.cvcc")), std::invalid_argument);
  EXPECT_THROW(cvc::write_contours(app, sample_stack(), dir.file("x.xyz")),
               cvc::contour_file_error);
}

// ---- T6: no singletons ----

struct fake_io : cvc::contour_file_io {
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  bool can_write() const override { return false; }
  cvc::contour_stack read(cvc::app &, const std::string &,
                          const cvc::contour_read_params &) const override {
    cvc::contour_stack s;
    s.add(0, "fake", {0, 0, 1, 0, 0, 1});
    return s;
  }
  std::string _id = "fake : v1";
  std::vector<std::string> _ext{".fake"};
};

TEST(ContourNoSingletons, TwoAppsKeepTheirOwnHandlers) {
  cvc::app a, b;
  ASSERT_TRUE(
      cvc::contour_handlers(a).insert(boost::shared_ptr<cvc::contour_file_io>(new fake_io)));
  EXPECT_EQ(cvc::read_contours(a, "anything.fake").component_names(),
            (std::vector<std::string>{"fake"}));
  EXPECT_THROW(cvc::read_contours(b, "anything.fake"), cvc::contour_file_error)
      << "b never saw a's handler";
  EXPECT_EQ(cvc::contour_handlers(a).size(), cvc::contour_handlers(b).size() + 1);
  // Inserting the same id again adds nothing.
  EXPECT_FALSE(
      cvc::contour_handlers(a).insert(boost::shared_ptr<cvc::contour_file_io>(new fake_io)));
  // A third app, made after, has the built-ins and nothing of a's.
  cvc::app c;
  EXPECT_EQ(cvc::contour_handlers(c).size(), cvc::contour_handlers(b).size());
}

TEST(ContourNoSingletons, TwoThreadsReadWhatEachReadsAlone) {
  const std::string ser = (data_dir() / "reconstruct" / "series.ser").generic_string();
  cvc::app app;
  cvc::contour_read_params pa, pb;
  pb.apply_transforms = false;
  pb.include = {"d01"};
  pb.first = 2;
  const cvc::contour_stack alone_a = cvc::read_contours(app, ser, pa);
  const cvc::contour_stack alone_b = cvc::read_contours(app, ser, pb);
  ASSERT_NE(alone_a, alone_b);
  // A then B, and B then A.
  EXPECT_EQ(cvc::read_contours(app, ser, pb), alone_b);
  EXPECT_EQ(cvc::read_contours(app, ser, pa), alone_a);
  std::atomic<int> mismatches{0};
  scratch_dir dir("threads");
  const auto run = [&](const cvc::contour_read_params &p, const cvc::contour_stack &want,
                       const std::string &out) {
    for (int i = 0; i < 20; ++i) {
      const cvc::contour_stack got = cvc::read_contours(app, ser, p);
      if (got != want)
        ++mismatches;
      cvc::write_contours(app, got, out);
      if (cvc::read_contours(app, out) != want)
        ++mismatches;
    }
  };
  std::thread ta(run, std::cref(pa), std::cref(alone_a), dir.file("a.cvcc"));
  std::thread tb(run, std::cref(pb), std::cref(alone_b), dir.file("b.cvcc"));
  ta.join();
  tb.join();
  EXPECT_EQ(mismatches.load(), 0);
}

// ---- the DSL ----

class ContourDsl : public ::testing::Test {
protected:
  void SetUp() override {
    ictx.sched = &app.exec_scheduler();
    ictx.root = &cvc::state::instance(app);
    ictx.tracker = &tracker;
    ictx.host = &app.exec_async_host();
    ictx.proc = se::make_process();
    ictx.proc->status = se::process_status::ready;
    env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    cvc::register_contour_intrinsics(env, ictx, app);
  }
  // Run `program` to its end; its exit error, "" when none.
  std::string run(const std::string &program) {
    se::execute_options eo;
    eo.env = env;
    se::async_scheduler &sched = app.exec_scheduler();
    const int pid = sched.execute(program, eo);
    const auto t0 = steady::now();
    while (std::chrono::duration<double>(steady::now() - t0).count() < 30.0) {
      const auto info = sched.get_process_info(pid);
      if (!info || se::is_finished(info->status))
        return info ? info->exit_error.value_or(std::string()) : std::string();
      sched.drain_ingress();
      sched.wake_awaiting();
      sched.sync_step();
      sched.sync_run(200000, 0.1);
    }
    return "timed out";
  }
  std::string key(const std::string &k) { return app.root()(k).value(); }

  cvc::app app;
  se::memory_tracker tracker;
  se::intrinsics_context ictx;
  se::environment_ptr env;
};

TEST_F(ContourDsl, InfoOnTheTree) {
  // The rest of libcvc puts a stack on the tree, as a data object a program stored or as the
  // stack itself; the DSL reads it there.
  const std::string ser = (data_dir() / "reconstruct" / "series.ser").generic_string();
  cvc::contour_read_params only;
  only.include = {"d01"};
  const cvc::contour_stack read = cvc::read_contours(app, ser, only);
  app.root()("doc.contours").data(boost::any(cvc::contour_stack_value(read)));
  ASSERT_EQ(run("(let ((i (contours.info \"doc.contours\")))"
                "  (state-set \"r.first\" (get-attr i \"first\"))"
                "  (state-set \"r.last\" (get-attr i \"last\"))"
                "  (state-set \"r.polygons\" (get-attr i \"polygons\"))"
                "  (state-set \"r.name\" (get-attr (nth (get-attr i \"components\") 0) \"name\")))"
                "(state-data-set \"doc.copy\" (state-data-get \"doc.contours\"))"
                "(state-set \"r.copy\" (get-attr (contours.info (state-data-get \"doc.copy\")) "
                "\"vertices\"))"),
            "");
  EXPECT_EQ(key("r.first"), "1");
  EXPECT_EQ(key("r.last"), "3");
  EXPECT_EQ(key("r.polygons"), "3");
  EXPECT_EQ(key("r.name"), "d01");
  EXPECT_EQ(key("r.copy"), std::to_string(read.vertex_count()));
  // A C++ host's stack on the data channel serves as an argument too.
  app.root()("doc.cpp").data(boost::any(sample_stack()));
  ASSERT_EQ(run("(state-set \"r.cpp\" (get-attr (contours.info \"doc.cpp\") \"vertices\"))"), "");
  EXPECT_EQ(key("r.cpp"), "18");
}

TEST_F(ContourDsl, NoIntrinsicReadsOrWritesFiles) {
  // Reading and writing files is the rest of libcvc's, not the DSL's.
  for (const char *fn : {"contours.read", "contours.write", "contours.formats"}) {
    ASSERT_EQ(run(std::string("(state-set \"r.fn\" (try (") + fn +
                  " \"doc.x\" \"x.cvcc\") (catch (Exception e) (get-attr e \"message\"))))"),
              "");
    EXPECT_NE(key("r.fn").find(fn), std::string::npos) << fn << " is bound: " << key("r.fn");
  }
}

TEST_F(ContourDsl, ParamsAndErrors) {
  ASSERT_EQ(
      run("(begin"
          "  (state-set \"r.min\" (get-attr (params.defaults \"contours.read\") "
          "\"min_points\"))"
          "  (state-set \"r.check\" (params.check \"contours.read\" (dict \"first\" -5)))"
          "  (state-set \"r.comp\" (params.check \"contours.component\" (dict \"name\" \"\")))"
          "  (state-set \"r.arity\" (try (contours.info \"doc.x\" 2)"
          "                           (catch (TypeError e) (get-attr e \"message\"))))"
          "  (state-set \"r.nodata\" (try (contours.info \"doc.nothing\")"
          "                            (catch (ValueError e) (get-attr e \"message\"))))"
          "  (state-set \"r.type\" (try (contours.info 5)"
          "                          (catch (TypeError e) (get-attr e \"message\")))))"),
      "");
  EXPECT_EQ(key("r.min"), "3");
  EXPECT_NE(key("r.check").find("first = -5 is below its minimum -1"), std::string::npos)
      << key("r.check");
  EXPECT_NE(key("r.comp").find("a component needs a name"), std::string::npos) << key("r.comp");
  EXPECT_NE(key("r.arity").find("contours.info: takes a stack"), std::string::npos)
      << key("r.arity");
  EXPECT_NE(key("r.nodata").find("no state node at 'doc.nothing'"), std::string::npos)
      << key("r.nodata");
  EXPECT_NE(key("r.type").find("a data object or a state path"), std::string::npos)
      << key("r.type");
}

// ---- Ariadne: the app's own provider ----

struct ClickBackend : cvc::ariadne::Backend {
  std::string click;
  cvc::ariadne::Capabilities capabilities() const override {
    cvc::ariadne::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const cvc::ariadne::Size &, float,
                    bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const cvc::ariadne::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *) override {}
  void text_value(const char *, const std::string &) override {}
  void separator() override {}
  bool button(const char *l) override {
    if (click == l) {
      click.clear();
      return true;
    }
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  cvc::ariadne::BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  cvc::ariadne::BoolEdit checkbox(const char *, bool) override { return {}; }
  cvc::ariadne::IntEdit slider_int(const char *, const char *, int, int, int) override {
    return {};
  }
  cvc::ariadne::DoubleEdit slider_double(const char *, const char *, double, double, double,
                                         const char *) override {
    return {};
  }
  cvc::ariadne::IndexEdit combo(const char *, int, const std::vector<std::string> &) override {
    return {};
  }
  cvc::ariadne::StringEdit input_text(const char *, const char *, const std::string &,
                                      bool) override {
    return {};
  }
};

// Click `label` in a one-button document on `app`, and run frames until the action has run.
void click_once(cvc::app &app, const std::string &action) {
  cvc::ariadne::Runtime rt(app, "");
  ClickBackend be;
  rt.set_backend(&be);
  rt.set_root(cvc::ariadne::group({cvc::ariadne::button("Go", action)}));
  be.click = "Go";
  for (int i = 0; i < 20; ++i) {
    rt.render();
    rt.drain();
  }
}

TEST(ContourAriadne, AnActionReadsTheTreeThroughTheAppsOwnProvider) {
  const std::string action =
      "(state-set \"doc.count\" (get-attr (contours.info \"doc.contours\") \"polygons\"))";
  cvc::app with, without;
  for (cvc::app *a : {&with, &without})
    a->root()("doc.contours").data(boost::any(sample_stack()));
  cvc::install_contour_intrinsics(with);
  cvc::install_contour_intrinsics(with); // keyed: a second install replaces the first
  click_once(with, action);
  EXPECT_EQ(with.root()("doc.count").value(), "5");
  // Another app's documents never got them.
  click_once(without, action);
  EXPECT_TRUE(without.root()("doc.count").value().empty());
  EXPECT_EQ(cvc::find_param_table(without, "contours.read"), nullptr);
  ASSERT_NE(cvc::find_param_table(with, "contours.read"), nullptr);
  cvc::ariadne::clear_action_intrinsics(with);
}

} // namespace
