/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc/volume/volume_grid_segment.h, the data side of VolumeGridRover's Segmentation tab (docs/
// roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, §5, P9), against the §1.7 rows that apply:
//   - the run: volume_grid_segment_stepper gives segment_seeded's labels and class_surfaces'
//     surfaces of them, bit for bit, at every budget (T7), with its phases named, its progress
//     shared between the two, a cancel in either part leaving nothing; the labels so far while it
//     marches (segment_stepper::labels_so_far);
//   - the rows: coloured as the point classes that seeded them, the paper's feature and
//     background classes, written to and read back from the state tree (T8);
//   - color_labels: the see-through layer over a slice;
//   - sessions: which keys of the tab a session keeps; the view's parameter table (T8);
//   - T6: two apps and two threads share nothing.
// Runs natively and under node (the ports lane, T14).

#include "resumable_harness.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/segmentation_params.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_segment.h>
#include <gtest/gtest.h>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

struct bump {
  double x, y, z, sigma;
};

// A Float volume of n^3 voxels: Gaussian bumps of height 1.
cvc::volume blobs(cvc::app &a, int n, const std::vector<bump> &bumps) {
  cvc::volume v(a, cvc::dimension(n, n, n), cvc::Float,
                cvc::bounding_box(0, 0, 0, n - 1, n - 1, n - 1));
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double s = 0.0;
        for (const bump &b : bumps) {
          const double d2 = (i - b.x) * (i - b.x) + (j - b.y) * (j - b.y) + (k - b.z) * (k - b.z);
          s += std::exp(-d2 / (2 * b.sigma * b.sigma));
        }
        v(static_cast<cvc::uint64>(i), static_cast<cvc::uint64>(j), static_cast<cvc::uint64>(k),
          s);
      }
  return v;
}

cvc::point_classes two_classes(int n) {
  cvc::point_classes pc;
  cvc::point_class a, b;
  a.name = "left";
  a.color = {1.0, 0.0, 0.0};
  a.points = {n / 4, n / 2, n / 2};
  b.name = "right";
  b.color = {0.0, 0.5, 1.0};
  b.points = {3 * n / 4, n / 2, n / 2};
  pc.classes = {a, b};
  return pc;
}

cvc::seeded_segmentation_params params() {
  cvc::seeded_segmentation_params p;
  p.t_low = 0.15;
  p.t_high = 0.9;
  p.gamma = -0.04; // fronts reach past the bumps' flanks
  return p;
}

bool same_volume(const cvc::volume &a, const cvc::volume &b) {
  return static_cast<const cvc::voxels &>(a) == static_cast<const cvc::voxels &>(b) &&
         a.boundingBox() == b.boundingBox();
}

bool same_seg(const cvc::seeded_segmentation &a, const cvc::seeded_segmentation &b) {
  if (!same_volume(a.labels, b.labels) || a.classes.size() != b.classes.size())
    return false;
  for (std::size_t c = 0; c < a.classes.size(); ++c)
    if (!same_volume(a.classes[c], b.classes[c]))
      return false;
  return a.border == b.border && a.class_names == b.class_names &&
         a.class_voxels == b.class_voxels && a.warnings == b.warnings;
}

bool same_surf(const cvc::class_surface_result &a, const cvc::class_surface_result &b) {
  if (a.labels != b.labels || a.voxels != b.voxels || a.surfaces.size() != b.surfaces.size())
    return false;
  for (std::size_t i = 0; i < a.surfaces.size(); ++i)
    if (a.surfaces[i].points() != b.surfaces[i].points() ||
        a.surfaces[i].tris() != b.surfaces[i].tris() ||
        a.surfaces[i].normals() != b.surfaces[i].normals())
      return false;
  return true;
}

bool same(const cvc::volume_grid_segment_result &a, const cvc::volume_grid_segment_result &b) {
  return same_seg(a.segmentation, b.segmentation) && same_surf(a.surfaces, b.surfaces);
}

cvc::volume_grid_segment_result run(const cvc::volume &v, const cvc::point_classes &pc,
                                    const cvc::seeded_segmentation_params &p,
                                    const cvc::class_surface_params &s = {}, bool surfaces = true,
                                    std::size_t budget = 0) {
  cvc::volume_grid_segment_stepper st(v, pc, p, s, surfaces);
  while (!st.step(budget)) {
  }
  return st.take();
}

// ---- the run ------------------------------------------------------------------------------------

TEST(VolumeGridSegment, TheRunIsTheSegmentationThenItsSurfaces) {
  cvc::app a;
  const int n = 24;
  const cvc::volume v = blobs(a, n, {{6, 12, 12, 3.0}, {18, 12, 12, 3.0}});
  const cvc::point_classes pc = two_classes(n);
  const cvc::volume_grid_segment_result r = run(v, pc, params());
  const cvc::seeded_segmentation ref = cvc::segment_seeded(v, pc, params());
  EXPECT_TRUE(same_seg(r.segmentation, ref));
  EXPECT_TRUE(same_surf(r.surfaces, cvc::class_surfaces(ref.labels)));
  ASSERT_EQ(r.surfaces.labels, (std::vector<int>{0, 1}));
  EXPECT_EQ(r.surfaces.voxels[0], ref.class_voxels[0]);
  EXPECT_GT(r.surfaces.surfaces[0].tris().size(), 0u);
  std::cout << "two blobs 24^3: " << ref.class_voxels[0] << " + " << ref.class_voxels[1]
            << " voxels, " << r.surfaces.triangle_count() << " triangles\n";
  // Without surfaces: the labels alone.
  const cvc::volume_grid_segment_result bare = run(v, pc, params(), {}, false);
  EXPECT_TRUE(same_seg(bare.segmentation, ref));
  EXPECT_TRUE(bare.surfaces.labels.empty());
  // The surface parameters reach the surfaces.
  cvc::class_surface_params rough;
  rough.smoothing = 0;
  EXPECT_TRUE(same_surf(run(v, pc, params(), rough).surfaces,
                        cvc::class_surfaces(ref.labels, rough)));
}

TEST(VolumeGridSegment, StepperIsResumable) {
  cvc::app a;
  const int n = 20;
  const cvc::volume v = blobs(a, n, {{5, 10, 10, 2.5}, {15, 10, 10, 2.5}});
  const cvc::point_classes pc = two_classes(n);
  cvc::class_surface_params s;
  s.smoothing = 2;
  cvc::test::t7_subject<cvc::volume_grid_segment_stepper, cvc::volume_grid_segment_result> t;
  t.name = "segment + class surfaces 20^3";
  t.make = [&] { return std::make_unique<cvc::volume_grid_segment_stepper>(v, pc, params(), s); };
  t.take = [](cvc::volume_grid_segment_stepper &st) { return st.take(); };
  t.equal = same;
  t.blocking = [&] { return run(v, pc, params(), s); };
  const cvc::test::t7_report r = cvc::test::run_t7(t);
  for (const char *phase : {"normalising", "marching", "writing", "surfaces: extracting",
                            "surfaces: smoothing", "surfaces: normals"})
    EXPECT_NE(std::find(r.phases.begin(), r.phases.end(), phase), r.phases.end()) << phase;
  EXPECT_LT(r.slowest_step_ms, 1000.0);
}

TEST(VolumeGridSegment, ProgressIsSharedAndACancelLeavesNothing) {
  cvc::app a;
  const int n = 20;
  const cvc::volume v = blobs(a, n, {{5, 10, 10, 2.5}, {15, 10, 10, 2.5}});
  const cvc::point_classes pc = two_classes(n);
  {
    cvc::volume_grid_segment_stepper st(v, pc, params());
    double last = 0.0;
    bool saw_surfaces = false;
    while (!st.step(256)) {
      const double q = st.progress();
      EXPECT_GE(q, last);
      last = q;
      if (std::string(st.phase()).rfind("surfaces", 0) == 0) {
        EXPECT_GE(q, 0.75) << "the labels' share is three quarters";
        saw_surfaces = true;
      } else {
        EXPECT_LE(q, 0.75);
      }
    }
    EXPECT_TRUE(saw_surfaces);
    EXPECT_EQ(st.progress(), 1.0);
    EXPECT_NE(st.status().find("2 surfaces"), std::string::npos) << st.status();
  }
  // A cancel while it marches, and one while it makes the surfaces.
  for (const char *where : {"marching", "surfaces: extracting"}) {
    auto job = cvc::make_result_job(
        std::make_unique<cvc::volume_grid_segment_stepper>(v, pc, params()));
    while (!job->done() && std::string(job->phase()) != where)
      job->step(16);
    ASSERT_EQ(std::string(job->phase()), where);
    job->cancel();
    EXPECT_TRUE(job->step(16));
    EXPECT_TRUE(job->cancelled());
    EXPECT_FALSE(job->has_result());
  }
  // A cancel within a step at budget 0: the stepper looks at it between its pieces.
  cvc::volume_grid_segment_stepper st(v, pc, params());
  st.cancel();
  EXPECT_TRUE(st.step(0));
  EXPECT_EQ(std::string(st.phase()), "failed");
  EXPECT_THROW((void)st.take(), cvc::segmentation_error);
  // A failure: no seeds.
  cvc::volume_grid_segment_stepper bad(v, cvc::point_classes{}, params());
  EXPECT_THROW(bad.step(0), cvc::segmentation_error);
  EXPECT_EQ(std::string(bad.phase()), "failed");
}

TEST(VolumeGridSegment, TheLabelsSoFarWhileItMarches) {
  cvc::app a;
  const int n = 24;
  const cvc::volume v = blobs(a, n, {{6, 12, 12, 3.0}, {18, 12, 12, 3.0}});
  const cvc::point_classes pc = two_classes(n);
  // segment_stepper itself: none before the sweep, then the seeds' and the fronts', none once done.
  {
    cvc::segment_stepper st(v, pc, params());
    cvc::volume out(a);
    EXPECT_FALSE(st.labels_so_far(out));
    std::size_t seen = 0, last_labelled = 0;
    bool grew = false;
    while (!st.step(64)) {
      const std::string ph = st.phase();
      cvc::volume snap(a);
      const bool have = st.labels_so_far(snap);
      EXPECT_EQ(have, ph == "classifying" || ph == "marching") << ph;
      if (!have)
        continue;
      ++seen;
      ASSERT_EQ(snap.voxelType(), cvc::UChar);
      ASSERT_EQ(snap.XDim(), cvc::uint64(n));
      EXPECT_EQ(snap.boundingBox(), v.boundingBox());
      std::size_t labelled = 0;
      for (std::size_t i = 0; i < std::size_t(n) * n * n; ++i) {
        const unsigned char l = snap.data_ptr()[i];
        EXPECT_TRUE(l == 0 || l == 1 || l == cvc::seeded_segmentation::unlabelled);
        labelled += l != cvc::seeded_segmentation::unlabelled;
      }
      EXPECT_GE(labelled, last_labelled) << "fronts only advance";
      grew = grew || labelled > last_labelled;
      last_labelled = labelled;
    }
    EXPECT_GT(seen, 0u);
    EXPECT_TRUE(grew);
    cvc::volume after(a);
    EXPECT_FALSE(st.labels_so_far(after));
    // The last labels so far are the result's labels (or fewer, the last steps unseen).
    const cvc::seeded_segmentation r = st.take();
    std::size_t final_labelled = 0;
    for (std::size_t c : r.class_voxels)
      final_labelled += c;
    EXPECT_LE(last_labelled, final_labelled);
  }
  // The run keeps them for the node: every step (live_seconds 0), from any thread.
  cvc::volume_grid_segment_stepper st(v, pc, params(), {}, true, 0.0);
  EXPECT_EQ(st.live_labels(), nullptr);
  std::uint64_t last = 0;
  bool rose = false;
  while (!st.step(64)) {
    const std::uint64_t ver = st.live_version();
    EXPECT_GE(ver, last);
    rose = rose || ver > last;
    last = ver;
  }
  EXPECT_TRUE(rose);
  ASSERT_NE(st.live_labels(), nullptr);
  // Live labels never change the result.
  EXPECT_TRUE(same(st.take(), run(v, pc, params())));
  // Off by default (-1): none kept.
  cvc::volume_grid_segment_stepper off(v, pc, params());
  off.step(0);
  EXPECT_EQ(off.live_labels(), nullptr);
  EXPECT_EQ(off.live_version(), 0u);
}

// ---- the rows -----------------------------------------------------------------------------------

TEST(VolumeGridSegment, RowsAreColouredAsTheirSeeds) {
  cvc::app a;
  const int n = 20;
  const cvc::volume v = blobs(a, n, {{5, 10, 10, 2.5}, {15, 10, 10, 2.5}});
  cvc::point_classes pc = two_classes(n);
  cvc::point_class other; // another timestep's: not a seed of this run
  other.name = "later";
  other.timestep = 1;
  other.color = {0.2, 0.2, 0.2};
  other.points = {1, 1, 1};
  pc.classes.insert(pc.classes.begin(), other);
  const cvc::volume_grid_segment_result r = run(v, pc, params());
  const std::vector<cvc::segment_class_row> rows =
      cvc::segment_class_rows(pc, 0, 0, r.segmentation, r.surfaces);
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[0].name, "left");
  EXPECT_EQ(rows[0].color, (std::array<double, 3>{1.0, 0.0, 0.0}));
  EXPECT_EQ(rows[1].name, "right");
  EXPECT_EQ(rows[1].color, (std::array<double, 3>{0.0, 0.5, 1.0}));
  EXPECT_TRUE(rows[0].visible && rows[1].visible);
  EXPECT_EQ(rows[0].voxels, r.segmentation.class_voxels[0]);
  EXPECT_EQ(rows[1].triangles, r.surfaces.surfaces[1].tris().size());

  // The paper's mode: two more classes, feature in gold and background hidden.
  cvc::seeded_segmentation_params cp = params();
  cp.automatic = cvc::seed_mode::critical_points;
  const cvc::volume_grid_segment_result c = run(v, two_classes(n), cp, {}, false);
  const std::vector<cvc::segment_class_row> crow =
      cvc::segment_class_rows(two_classes(n), 0, 0, c.segmentation);
  ASSERT_EQ(crow.size(), 4u);
  EXPECT_EQ(crow[2].name, "feature");
  EXPECT_TRUE(crow[2].visible);
  EXPECT_EQ(crow[3].name, "background");
  EXPECT_FALSE(crow[3].visible);
  EXPECT_EQ(crow[0].triangles, 0u) << "no surfaces made";
}

TEST(VolumeGridSegment, RowsOnTheStateTree) {
  cvc::app a;
  cvc::state &rows = cvc::state::instance(a)("doc.grid.segment.rows");
  std::vector<cvc::segment_class_row> r(3);
  r[0] = {"a", {1, 0, 0}, true, 10, 4};
  r[1] = {"b", {0, 1, 0}, false, 20, 8};
  r[2] = {"c", {0, 0, 1}, true, 30, 12};
  cvc::write_segment_rows(rows, r);
  EXPECT_EQ(rows.value(), "3");
  EXPECT_EQ(rows("1")("name").value(), "b");
  EXPECT_EQ(rows("1")("color").value(), "0,1,0");
  EXPECT_EQ(rows("1")("visible").value(), "0");
  EXPECT_EQ(rows("2")("voxels").value(), "30");
  EXPECT_EQ(rows("2")("triangles").value(), "12");
  // A list's edits come back: a colour and a shown flag; a bad value is a problem, kept as was.
  rows("0")("color").value("0.5, 0.25, 1");
  rows("1")("visible").value("1");
  rows("2")("color").value("red");
  std::vector<std::string> problems;
  EXPECT_TRUE(cvc::read_segment_rows(rows, r, &problems));
  EXPECT_EQ(r[0].color, (std::array<double, 3>{0.5, 0.25, 1.0}));
  EXPECT_TRUE(r[1].visible);
  EXPECT_EQ(r[2].color, (std::array<double, 3>{0, 0, 1}));
  ASSERT_EQ(problems.size(), 1u);
  EXPECT_NE(problems[0].find("rows.2.color"), std::string::npos);
  problems.clear();
  rows("2")("color").value("0,0,1");
  EXPECT_FALSE(cvc::read_segment_rows(rows, r, &problems)) << "nothing changed";
  EXPECT_TRUE(problems.empty());
  // Fewer rows: the rest go.
  r.resize(1);
  cvc::write_segment_rows(rows, r);
  EXPECT_EQ(rows.value(), "1");
  EXPECT_EQ(rows.findDescendant("2"), nullptr);
}

// ---- the colour layer ---------------------------------------------------------------------------

TEST(VolumeGridSegment, ColorLabelsIsASeeThroughLayer) {
  cvc::volume_slice_data s;
  s.width = 3;
  s.height = 2;
  // Row 0 (the bottom): class 0, class 1, unlabelled; row 1: class 2 (hidden), class 9 (no row), 0.
  s.values = {0, 1, 255, 2, 9, 0};
  std::vector<cvc::segment_class_row> rows(3);
  rows[0].color = {1, 0, 0};
  rows[1].color = {0, 0.5, 1};
  rows[2].color = {0, 1, 0};
  rows[2].visible = false;
  const std::vector<std::uint8_t> px = cvc::color_labels(s, rows, 0.5);
  ASSERT_EQ(px.size(), 3u * 2u * 4u);
  const auto at = [&px](int col, int img_row) { return &px[(img_row * 3 + col) * 4]; };
  // Image order: the slice's top row (row 1) first.
  EXPECT_EQ(at(0, 0)[3], 0) << "a hidden class is clear";
  EXPECT_EQ(at(1, 0)[3], 0) << "a class with no row is clear";
  EXPECT_EQ(at(2, 0)[0], 255);
  EXPECT_EQ(at(2, 0)[3], 128);
  EXPECT_EQ(at(0, 1)[0], 255);
  EXPECT_EQ(at(0, 1)[1], 0);
  EXPECT_EQ(at(0, 1)[3], 128);
  EXPECT_EQ(at(1, 1)[1], 128);
  EXPECT_EQ(at(1, 1)[2], 255);
  EXPECT_EQ(at(2, 1)[3], 0) << "unlabelled is clear";
  EXPECT_EQ(cvc::color_labels(s, rows, 7.0)[(1 * 3 + 0) * 4 + 3], 255) << "opacity clamped";
  EXPECT_EQ(cvc::color_labels(s, rows, -1.0)[(1 * 3 + 0) * 4 + 3], 0);
}

// ---- sessions and the view's table --------------------------------------------------------------

TEST(VolumeGridSegment, SessionsKeepTheTabsParameters) {
  for (const char *k : {"grid.segment.t_low", "grid.segment.gamma", "grid.segment.automatic",
                        "grid.segment.opacity", "grid.segment.show_surfaces",
                        "grid.segment.mesh.smoothing", "grid.segment.mesh.close"}) {
    EXPECT_TRUE(cvc::volume_grid_segment_session_key(k)) << k;
    EXPECT_TRUE(cvc::volume_grid_session_key(k)) << k << " (the grid's own predicate)";
  }
  for (const char *k : {"grid.segment.rows.0.color", "grid.segment.info.classes",
                        "grid.segment.labels", "grid.segment.mesh", "grid.segment",
                        "grid.status", "grid.segment.surfaces.0"})
    EXPECT_FALSE(cvc::volume_grid_segment_session_key(k)) << k;
}

TEST(VolumeGridSegment, ViewParameterTable) {
  cvc::app a;
  cvc::state &s = cvc::state::instance(a)("doc.grid.segment");
  const auto read = cvc::params_from_state<cvc::volume_grid_segment_view_params>(s);
  ASSERT_TRUE(read.ok()) << read.message();
  EXPECT_EQ(s("show").value(), "1");
  EXPECT_EQ(s("opacity").value(), "0.5");
  EXPECT_EQ(s("live").value(), "1");
  s("opacity").value("2");
  EXPECT_FALSE(cvc::params_validate(
                   cvc::params_from_state<cvc::volume_grid_segment_view_params>(s).value)
                   .empty());
  // The run's and the view's tables share the subtree, and their names never meet.
  const std::vector<std::string> run = cvc::param_names<cvc::seeded_segmentation_params>();
  for (const std::string &v : cvc::param_names<cvc::volume_grid_segment_view_params>())
    EXPECT_EQ(std::find(run.begin(), run.end(), v), run.end()) << v;
}

// ---- T6: no singletons --------------------------------------------------------------------------

TEST(VolumeGridSegment, TwoAppsAndTwoThreadsShareNothing) {
  cvc::app a1, a2;
  const cvc::volume v1 = blobs(a1, 20, {{5, 10, 10, 2.5}, {15, 10, 10, 2.5}});
  const cvc::volume v2 = blobs(a2, 18, {{6, 9, 9, 3.0}, {12, 9, 9, 2.0}});
  const cvc::point_classes p1 = two_classes(20), p2 = two_classes(18);
  const auto alone1 = run(v1, p1, params());
  const auto alone2 = run(v2, p2, params());
  std::optional<cvc::volume_grid_segment_result> t1, t2;
  std::thread th1([&] { t1.emplace(run(v1, p1, params(), {}, true, 128)); });
  std::thread th2([&] { t2.emplace(run(v2, p2, params(), {}, true, 64)); });
  th1.join();
  th2.join();
  EXPECT_TRUE(same(*t1, alone1));
  EXPECT_TRUE(same(*t2, alone2));
  EXPECT_EQ(&t1->segmentation.labels.ctx(), &a1);
  EXPECT_EQ(&t2->segmentation.labels.ctx(), &a2);
}

} // namespace
