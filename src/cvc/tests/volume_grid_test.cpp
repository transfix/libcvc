/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc/volume/volume_grid.h, the data side of VolumeGridRover: the VolumeRover ports' P8 rows for
// it (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §4):
//   - T8: the node's three parameter tables on the state tree; the class rows a list: binds;
//   - the editor: points marked, toggled, selected, moved, deleted and undone, on one plane at a
//     time, and only the shown variable and timestep's;
//   - the gestures: picks (down, drag, up, with Shift / Ctrl / Alt) turned into edits, as
//     VolumeGridRover's slice canvases turned mouse events;
//   - T9 / T5: sessions round-trip, and missing, garbage and wrong files raise naming the file;
//   - T6: two editors share nothing; two threads editing their own give what each gives alone.
// Runs natively and under node (the ports lane, T14).

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume_grid.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
using cvc::grid_pick;
using cvc::point_class_editor;
using cvc::point_ref;
using cvc::selection_mode;
using cvc::slice_axis;

struct scratch_dir {
  fs::path path;
  explicit scratch_dir(const std::string &name) {
    path = fs::temp_directory_path() /
           ("cvc_grid_" + name + "_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(path);
  }
  ~scratch_dir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  std::string file(const std::string &n) const { return (path / n).string(); }
};

cvc::point_classes two_classes() {
  cvc::point_classes pc;
  cvc::point_class a;
  a.name = "membrane";
  a.radius = 1.5;
  cvc::point_class b;
  b.name = "background";
  b.color = {0, 0, 1};
  pc.classes = {a, b};
  return pc;
}

point_class_editor editor(int n = 16) {
  point_class_editor ed;
  ed.set_dims({n, n, n});
  ed.set_classes(two_classes(), false);
  return ed;
}

grid_pick pick(grid_pick::phase_t ph, slice_axis a, int i, int j, int k, const char *mods = "") {
  grid_pick p;
  p.phase = ph;
  p.axis = a;
  p.ijk = {i, j, k};
  p.inside = i >= 0 && j >= 0 && k >= 0 && i < 16 && j < 16 && k < 16;
  cvc::parse_grid_pick_mods(mods, p);
  return p;
}
constexpr grid_pick::phase_t kDown = grid_pick::phase_t::down;
constexpr grid_pick::phase_t kDrag = grid_pick::phase_t::drag;
constexpr grid_pick::phase_t kUp = grid_pick::phase_t::up;

std::vector<int> pts(const point_class_editor &ed, std::size_t c) {
  return ed.classes().classes[c].points;
}

// ---- T8: the parameter tables ----

TEST(VolumeGridParams, TheTablesRoundTripThroughTheTree) {
  cvc::app app;
  cvc::state &grid = app.root()("doc.graphics.root.children.grid.grid");
  cvc::volume_grid_params p;
  p.view = cvc::grid_view::zy;
  p.tool = cvc::grid_tool::navigate;
  p.slice_x = 3;
  p.slice_z = 40;
  cvc::params_to_state(p, grid);
  EXPECT_EQ(grid("view").value(), "zy");
  EXPECT_EQ(grid("slice.x").value(), "3");
  EXPECT_EQ(grid("slice.y").value(), "-1");
  EXPECT_EQ(grid("slice.z").property("unit"), "voxel");
  EXPECT_EQ(grid("tool").property("choices"), "navigate,points,probe");
  const auto back = cvc::params_from_state<cvc::volume_grid_params>(grid);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.view, cvc::grid_view::zy);
  EXPECT_EQ(back.value.slice_z, 40);

  cvc::state &display = grid("display");
  const auto seeded = cvc::params_from_state<cvc::volume_grid_display_params>(display);
  ASSERT_TRUE(seeded.ok()) << seeded.message();
  EXPECT_EQ(display("point_scale").value(), "1");
  EXPECT_EQ(display("show_points").value(), "1");
  display("point_scale").value("40");
  const auto bad = cvc::params_from_state<cvc::volume_grid_display_params>(display);
  EXPECT_FALSE(bad.ok());
  EXPECT_NE(bad.message().find("point_scale = 40 is above its maximum 10"), std::string::npos)
      << bad.message();

  cvc::state &source = grid("source");
  source("volume").value("data.head.volume");
  source("var").value("-1");
  const auto src = cvc::params_from_state<cvc::volume_grid_source_params>(source);
  EXPECT_NE(src.message().find("var = -1 is below its minimum 0"), std::string::npos)
      << src.message();
  EXPECT_EQ(src.value.volume, "data.head.volume");
  EXPECT_EQ(src.value.points, "");
}

TEST(VolumeGridParams, ViewsLookDownTheLegacyAxes) {
  EXPECT_EQ(cvc::grid_view_axis(cvc::grid_view::xy), slice_axis::z);
  EXPECT_EQ(cvc::grid_view_axis(cvc::grid_view::xz), slice_axis::y);
  EXPECT_EQ(cvc::grid_view_axis(cvc::grid_view::zy), slice_axis::x);
  for (slice_axis a : {slice_axis::x, slice_axis::y, slice_axis::z})
    EXPECT_EQ(cvc::grid_view_axis(cvc::grid_view_of(a)), a);
  EXPECT_EQ(cvc::grid_plane_axes(slice_axis::z), std::make_pair(0, 1)); // XY: x right, y up
  EXPECT_EQ(cvc::grid_plane_axes(slice_axis::y), std::make_pair(0, 2)); // XZ: x right, z up
  EXPECT_EQ(cvc::grid_plane_axes(slice_axis::x), std::make_pair(2, 1)); // ZY: z right, y up
}

// ---- the editor ----

TEST(PointClassEditor, ToggleAddsThenRemovesOnThePlaneOnly) {
  point_class_editor ed = editor();
  EXPECT_EQ(ed.toggle(0, slice_axis::z, {4, 5, 6}), point_class_editor::toggled::added);
  EXPECT_EQ(pts(ed, 0), (std::vector<int>{4, 5, 6}));
  // Radius 1.5: a click one voxel off on the same plane finds it ...
  EXPECT_EQ(ed.toggle(0, slice_axis::z, {5, 5, 6}), point_class_editor::toggled::removed);
  EXPECT_TRUE(pts(ed, 0).empty());
  ed.add(0, {4, 5, 6});
  // ... but not from another plane (z = 7), nor down another axis's plane through another voxel.
  EXPECT_EQ(ed.toggle(0, slice_axis::z, {4, 5, 7}), point_class_editor::toggled::added);
  EXPECT_EQ(ed.classes().classes[0].size(), 2u);
  // Class 1 (radius 1) only finds its own exact voxel, and never class 0's points.
  EXPECT_EQ(ed.toggle(1, slice_axis::z, {4, 5, 6}), point_class_editor::toggled::added);
  EXPECT_EQ(ed.toggle(1, slice_axis::z, {5, 5, 6}), point_class_editor::toggled::added);
  EXPECT_EQ(ed.classes().classes[1].size(), 2u);
  EXPECT_EQ(ed.points_on(slice_axis::z, 6).size(), 3u);
  EXPECT_EQ(ed.points_on(slice_axis::x, 4).size(), 3u);
  EXPECT_EQ(ed.points_on(slice_axis::y, 5).size(), 4u);
}

TEST(PointClassEditor, ThePointScaleWidensThePick) {
  point_class_editor ed = editor();
  ed.add(1, {8, 8, 8});
  EXPECT_FALSE(ed.nearest(slice_axis::z, {10, 8, 8}, 1.0));
  EXPECT_TRUE(ed.nearest(slice_axis::z, {10, 8, 8}, 2.5));
  EXPECT_DOUBLE_EQ(ed.pick_radius(1, 2.5), 2.5);
  EXPECT_DOUBLE_EQ(ed.pick_radius(1, 0.1), 0.5) << "at least the voxel itself";
}

TEST(PointClassEditor, BadInputThrowsWithAMessage) {
  point_class_editor ed = editor();
  EXPECT_THROW(ed.add(5, {0, 0, 0}), std::out_of_range);
  try {
    ed.add(0, {16, 0, 0});
    FAIL() << "a voxel outside the volume";
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("(16, 0, 0) lies outside the volume (16 x 16 x 16)"),
              std::string::npos)
        << e.what();
  }
  EXPECT_THROW(ed.set_dims({4, 0, 4}), std::invalid_argument);
  EXPECT_FALSE(ed.remove({0, {1, 1, 1}}));
  EXPECT_FALSE(ed.remove({9, {1, 1, 1}}));
  EXPECT_FALSE(ed.add(0, {1, 1, 1}) && ed.add(0, {1, 1, 1})) << "a class holds a voxel once";
}

TEST(PointClassEditor, SelectReplacesAddsAndRemoves) {
  point_class_editor ed = editor();
  for (int i = 0; i < 5; ++i)
    ed.add(0, {i * 2, 3, 7});
  ed.add(1, {1, 9, 7});
  ed.add(1, {1, 9, 8}); // another plane
  EXPECT_EQ(ed.select_rect(slice_axis::z, 7, 0, 0, 4, 10, selection_mode::replace), 4u);
  EXPECT_TRUE(ed.selected({1, {1, 9, 7}}));
  EXPECT_FALSE(ed.selected({1, {1, 9, 8}}));
  // Corners in either order, edges included.
  EXPECT_EQ(ed.select_rect(slice_axis::z, 7, 8, 3, 6, 3, selection_mode::add), 6u);
  EXPECT_EQ(ed.select_rect(slice_axis::z, 7, 0, 0, 2, 3, selection_mode::remove), 4u);
  EXPECT_FALSE(ed.selected({0, {0, 3, 7}}));
  ed.select({0, {0, 3, 7}}, selection_mode::add);
  EXPECT_EQ(ed.selection().size(), 5u);
  ed.select({0, {0, 3, 7}}, selection_mode::replace);
  EXPECT_EQ(ed.selection().size(), 1u);
  ed.select({0, {15, 15, 15}}, selection_mode::add); // no such point: nothing
  EXPECT_EQ(ed.selection().size(), 1u);
  ed.select_all();
  EXPECT_EQ(ed.selection().size(), 7u);
  const cvc::point_classes sel = ed.selection_classes();
  EXPECT_EQ(sel.point_count(), 7u);
  EXPECT_EQ(sel.classes[1].name, "background");
  ed.clear_selection();
  EXPECT_TRUE(ed.selection().empty());
}

TEST(PointClassEditor, MovesStayInsideAndMerge) {
  point_class_editor ed = editor();
  ed.add(0, {1, 1, 4});
  ed.add(0, {3, 1, 4});
  ed.add(0, {14, 1, 4});
  ed.select_rect(slice_axis::z, 4, 0, 0, 3, 3, selection_mode::replace);
  // Looking down y, the plane's right is x and its up is z: (dr, du) = (+2, +5) moves x and z.
  ed.clear_selection();
  ed.select({0, {1, 1, 4}}, selection_mode::add);
  EXPECT_EQ(ed.move_selection(slice_axis::y, 2, 5), std::make_pair(2, 5));
  EXPECT_TRUE(ed.classes().classes[0].contains(3, 1, 9));
  // Cut short at the edge: x 3 can go 12 further at most.
  EXPECT_EQ(ed.move_selection(slice_axis::z, 40, 0), std::make_pair(12, 0));
  EXPECT_TRUE(ed.selected({0, {15, 1, 9}}));
  // Onto another point of its class: they merge.
  ed.move_selection(slice_axis::y, -1, -5);
  EXPECT_EQ(ed.classes().classes[0].size(), 2u);
  EXPECT_EQ(ed.selection().size(), 1u);
  EXPECT_TRUE(ed.selected({0, {14, 1, 4}}));
  EXPECT_EQ(ed.move_selection(slice_axis::z, 0, 0), std::make_pair(0, 0));
}

TEST(PointClassEditor, DeleteAndUndo) {
  point_class_editor ed = editor();
  ed.add(0, {1, 2, 3});
  ed.add(0, {2, 2, 3});
  ed.add(1, {5, 5, 3});
  const cvc::point_classes three = ed.classes();
  ed.select_rect(slice_axis::z, 3, 0, 0, 15, 15, selection_mode::replace);
  ed.select({0, {2, 2, 3}}, selection_mode::remove);
  EXPECT_EQ(ed.delete_selected(), 2u);
  EXPECT_EQ(ed.classes().point_count(), 1u);
  EXPECT_EQ(ed.delete_selected(), 0u);
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.classes(), three);
  EXPECT_TRUE(ed.undo()); // the third add
  EXPECT_EQ(ed.classes().point_count(), 2u);
  EXPECT_TRUE(ed.undo());
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.classes().point_count(), 0u);
  EXPECT_FALSE(ed.undo()) << "set_classes(.., false) made no step";
  for (int i = 0; i < 100; ++i)
    ed.add(0, {i % 16, i / 16, 0});
  EXPECT_EQ(ed.undo_depth(), point_class_editor::undo_limit);
}

TEST(PointClassEditor, OnlyTheShownVariableAndTimestepAreEdited) {
  point_class_editor ed = editor();
  cvc::point_classes pc = ed.classes();
  pc.classes[1].timestep = 2;
  pc.classes[1].points = {4, 4, 4};
  ed.set_classes(pc);
  EXPECT_FALSE(ed.shown(1));
  EXPECT_TRUE(ed.points_on(slice_axis::z, 4).empty());
  EXPECT_FALSE(ed.nearest(slice_axis::z, {4, 4, 4}, 1.0));
  EXPECT_THROW(ed.toggle(1, slice_axis::z, {4, 4, 4}), std::invalid_argument);
  ed.set_shown(0, 2);
  EXPECT_EQ(ed.points_on(slice_axis::z, 4).size(), 1u);
  ed.select_all();
  EXPECT_EQ(ed.selection().size(), 1u);
  ed.set_shown(0, 0); // its points can no longer be selected
  EXPECT_TRUE(ed.selection().empty());
}

// ---- the gestures ----

TEST(PointClassGestures, AClickTogglesAPointOfTheCurrentClass) {
  point_class_editor ed = editor();
  cvc::point_class_gestures g;
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::z, 4, 5, 6));
  EXPECT_TRUE(g.active());
  cvc::grid_pick_outcome o =
      g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::z, 4, 5, 6));
  EXPECT_FALSE(g.active());
  EXPECT_TRUE(o.points_changed);
  EXPECT_EQ(o.note, "added (4, 5, 6) to membrane");
  EXPECT_EQ(pts(ed, 0), (std::vector<int>{4, 5, 6}));
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::z, 5, 5, 6));
  o = g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::z, 5, 5, 6));
  EXPECT_EQ(o.note, "removed (4, 5, 6) from membrane") << "the point under the pointer";
  EXPECT_TRUE(pts(ed, 0).empty());
  // No class: the click says so and adds nothing.
  g.apply(ed, cvc::grid_tool::points, -1, 1.0, pick(kDown, slice_axis::z, 5, 5, 6));
  o = g.apply(ed, cvc::grid_tool::points, -1, 1.0, pick(kUp, slice_axis::z, 5, 5, 6));
  EXPECT_FALSE(o.points_changed);
  EXPECT_NE(o.note.find("no class"), std::string::npos);
  // Past the volume's edge: nothing.
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::z, 20, 5, 6));
  o = g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::z, 20, 5, 6));
  EXPECT_FALSE(o.points_changed);
  EXPECT_EQ(ed.classes().point_count(), 0u);
}

TEST(PointClassGestures, ADragIsARubberBandWithTheLegacyModifiers) {
  point_class_editor ed = editor();
  for (int i = 0; i < 8; ++i)
    ed.add(i % 2, {i * 2, 6, 3}); // looking down y at 6: (r, u) = (x, z) = (2i, 3)
  cvc::point_class_gestures g;
  const auto drag = [&](int r0, int r1, const char *mods) {
    g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::y, r0, 6, 0, mods));
    cvc::grid_pick_outcome o =
        g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDrag, slice_axis::y, r1, 6, 5, mods));
    EXPECT_TRUE(o.band);
    EXPECT_EQ(o.band_axis, slice_axis::y);
    EXPECT_EQ(o.band_index, 6);
    EXPECT_EQ(o.band_rect, (std::array<std::int64_t, 4>{r0, 0, r1, 5}));
    return g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::y, r1, 6, 5, mods));
  };
  cvc::grid_pick_outcome o = drag(0, 6, "");
  EXPECT_EQ(o.note, "4 points selected");
  EXPECT_TRUE(o.selection_changed);
  EXPECT_FALSE(o.points_changed);
  EXPECT_EQ(drag(10, 12, "shift").note, "6 points selected");
  EXPECT_EQ(drag(0, 2, "ctrl").note, "4 points selected");
  EXPECT_EQ(drag(4, 4 + 2, "alt").note, "2 points selected");
  EXPECT_EQ(ed.classes().point_count(), 8u) << "selecting edits nothing";
  // A Shift / Ctrl click picks one point.
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::y, 0, 6, 3, "shift"));
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::y, 0, 6, 3, "shift"));
  EXPECT_TRUE(ed.selected({0, {0, 6, 3}}));
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::y, 0, 6, 3, "ctrl"));
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::y, 0, 6, 3, "ctrl"));
  EXPECT_FALSE(ed.selected({0, {0, 6, 3}}));
}

TEST(PointClassGestures, ShiftCtrlDragMovesTheSelectionAsOneUndoStep) {
  point_class_editor ed = editor();
  ed.add(0, {2, 2, 5});
  ed.add(1, {3, 2, 5});
  ed.select_all();
  const std::size_t depth = ed.undo_depth();
  cvc::point_class_gestures g;
  g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kDown, slice_axis::z, 2, 2, 5, "shift,ctrl"));
  for (int step = 1; step <= 4; ++step) {
    const cvc::grid_pick_outcome o =
        g.apply(ed, cvc::grid_tool::points, 0, 1.0,
                pick(kDrag, slice_axis::z, 2 + step, 2 + step, 5, "shift,ctrl"));
    EXPECT_TRUE(o.points_changed);
  }
  const cvc::grid_pick_outcome o =
      g.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::z, 6, 6, 5, "shift,ctrl"));
  EXPECT_EQ(o.note, "moved 2 points");
  EXPECT_TRUE(ed.classes().classes[0].contains(6, 6, 5));
  EXPECT_TRUE(ed.classes().classes[1].contains(7, 6, 5));
  EXPECT_EQ(ed.undo_depth(), depth + 1);
  ed.undo();
  EXPECT_TRUE(ed.classes().classes[0].contains(2, 2, 5));
}

TEST(PointClassGestures, NavigateMovesTheOtherPlanesAndProbeEditsNothing) {
  point_class_editor ed = editor();
  cvc::point_class_gestures g;
  cvc::grid_pick_outcome o =
      g.apply(ed, cvc::grid_tool::navigate, 0, 1.0, pick(kDown, slice_axis::x, 9, 3, 12));
  EXPECT_EQ(o.slice, (std::array<std::int64_t, 3>{-1, 3, 12}));
  o = g.apply(ed, cvc::grid_tool::navigate, 0, 1.0, pick(kDrag, slice_axis::x, 9, 30, -4));
  EXPECT_EQ(o.slice, (std::array<std::int64_t, 3>{-1, 15, 0})) << "clamped to the volume";
  g.apply(ed, cvc::grid_tool::navigate, 0, 1.0, pick(kUp, slice_axis::x, 9, 30, -4));
  for (grid_pick::phase_t ph : {kDown, kDrag, kUp})
    o = g.apply(ed, cvc::grid_tool::probe, 0, 1.0, pick(ph, slice_axis::z, 1, 1, 1));
  EXPECT_FALSE(o.points_changed);
  EXPECT_EQ(ed.classes().point_count(), 0u);
  // A drag or an up with no down before it starts nothing.
  cvc::point_class_gestures fresh;
  o = fresh.apply(ed, cvc::grid_tool::points, 0, 1.0, pick(kUp, slice_axis::z, 1, 1, 1));
  EXPECT_FALSE(o.points_changed);
  EXPECT_EQ(ed.classes().point_count(), 0u);
}

TEST(PointClassGestures, PhasesAndModifiersParse) {
  grid_pick::phase_t ph{};
  EXPECT_TRUE(cvc::parse_grid_pick_phase("drag", ph));
  EXPECT_EQ(ph, kDrag);
  EXPECT_FALSE(cvc::parse_grid_pick_phase("hover", ph));
  grid_pick p;
  cvc::parse_grid_pick_mods("ctrl,alt", p);
  EXPECT_FALSE(p.shift);
  EXPECT_TRUE(p.ctrl);
  EXPECT_TRUE(p.alt);
  cvc::parse_grid_pick_mods("", p);
  EXPECT_FALSE(p.ctrl || p.alt || p.shift);
}

// ---- class rows ----

TEST(PointClassRows, RowsRoundTripAndReconcileListEdits) {
  cvc::app app;
  cvc::state &rows = app.root()("doc.grid.classes");
  cvc::point_classes pc = two_classes();
  pc.classes[0].points = {1, 2, 3, 4, 5, 6};
  pc.classes[1].points = {7, 7, 7};
  cvc::write_point_class_rows(rows, pc);
  EXPECT_EQ(rows.value(), "2");
  EXPECT_EQ(rows("0.name").value(), "membrane");
  EXPECT_EQ(rows("0.color").value(), "1,0,0");
  EXPECT_EQ(rows("0.radius").value(), "1.5");
  EXPECT_EQ(rows("0.points").value(), "2");
  std::vector<std::string> problems;
  std::vector<cvc::point_class> back = cvc::read_point_class_rows(rows, 0, 0, &problems);
  EXPECT_TRUE(problems.empty());
  EXPECT_EQ(cvc::reconcile_point_class_rows(pc, back), pc);

  // A row edited in place keeps its points.
  rows("1.name").value("bg");
  rows("1.color").value("0.5,0.5,0.5");
  cvc::point_classes next =
      cvc::reconcile_point_class_rows(pc, cvc::read_point_class_rows(rows, 0, 0));
  EXPECT_EQ(next.classes[1].name, "bg");
  EXPECT_EQ(next.classes[1].points, pc.classes[1].points);
  EXPECT_EQ(next.classes[1].color, (std::vector<double>{0.5, 0.5, 0.5}));
  pc = next;

  // A list's Add: a row with only some fields, the rest defaulted (variable and timestep the shown
  // ones).
  rows("2.name").value("class 3");
  rows.value("3");
  next = cvc::reconcile_point_class_rows(pc, cvc::read_point_class_rows(rows, 1, 4));
  ASSERT_EQ(next.classes.size(), 3u);
  EXPECT_TRUE(next.classes[2].points.empty());
  EXPECT_EQ(next.classes[2].variable, 1);
  EXPECT_EQ(next.classes[2].timestep, 4);
  EXPECT_EQ(next.classes[2].color, cvc::point_class_palette(2));
  EXPECT_EQ(next.classes[0].points, pc.classes[0].points);
  pc = next;

  // A list's Remove of the first row moves the others up: each keeps its own points.
  cvc::write_point_class_rows(rows, pc);
  cvc::point_classes removed = pc;
  removed.classes.erase(removed.classes.begin());
  cvc::write_point_class_rows(rows, removed);
  EXPECT_FALSE(rows.findDescendant("2")) << "rows past the count go";
  for (cvc::point_class &c : removed.classes)
    c.points.clear();
  cvc::write_point_class_rows(rows, removed); // the keys hold no points
  next = cvc::reconcile_point_class_rows(pc, cvc::read_point_class_rows(rows, 0, 0));
  ASSERT_EQ(next.classes.size(), 2u);
  EXPECT_EQ(next.classes[0].name, "bg");
  EXPECT_EQ(next.classes[0].points, pc.classes[1].points);
  EXPECT_TRUE(next.classes[1].points.empty()); // class 3 never had any

  // Bad values are reported and defaulted.
  rows("0.radius").value("-2");
  rows("0.color").value("red");
  rows("0.variable").value("x");
  problems.clear();
  back = cvc::read_point_class_rows(rows, 0, 0, &problems);
  ASSERT_EQ(problems.size(), 3u);
  EXPECT_NE(problems[0].find("classes.0.color: 'red' is not r,g,b"), std::string::npos);
  EXPECT_EQ(back[0].radius, 1.0);
  rows.value("lots");
  problems.clear();
  EXPECT_TRUE(cvc::read_point_class_rows(rows, 0, 0, &problems).empty());
  EXPECT_EQ(problems.size(), 1u);
}

// ---- T9 / T5: sessions ----

cvc::volume_grid_session a_session() {
  cvc::volume_grid_session s;
  s.keys = {{"grid.view", "xz"},      {"grid.tool", "navigate"},
            {"grid.slice.z", "7"},    {"grid.display.point_scale", "2"},
            {"grid.source.var", "1"}, {"grid.classes.current", "1"}};
  s.transfer_function = "{\"color\":[[0,0,0,0],[1,1,1,1]]}";
  s.classes = two_classes();
  s.classes.classes[0].points = {1, 2, 3, 4, 5, 6};
  s.classes.classes[1].timestep = 3;
  return s;
}

TEST(VolumeGridSession, RoundTripsThroughAFile) {
  scratch_dir dir("session");
  const cvc::volume_grid_session s = a_session();
  const std::string f = dir.file("grid.session.json");
  cvc::write_volume_grid_session(s, f);
  const cvc::volume_grid_session back = cvc::read_volume_grid_session(f);
  EXPECT_EQ(back.keys, s.keys);
  EXPECT_EQ(back.transfer_function, s.transfer_function);
  EXPECT_EQ(back.classes, s.classes);
  const std::string json = cvc::volume_grid_session_json(s);
  EXPECT_NE(json.find("\"format\": \"cvc.volume_grid.session\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"points\": [1, 2, 3, 4, 5, 6]"), std::string::npos) << json;
  // An empty session too.
  const cvc::volume_grid_session empty =
      cvc::volume_grid_session_from_json(cvc::volume_grid_session_json({}));
  EXPECT_TRUE(empty.keys.empty());
  EXPECT_TRUE(empty.classes.empty());
}

TEST(VolumeGridSession, OnlyTheSessionsKeysGoIn) {
  EXPECT_TRUE(cvc::volume_grid_session_key("grid.slice.y"));
  EXPECT_TRUE(cvc::volume_grid_session_key("grid.display.grey"));
  EXPECT_TRUE(cvc::volume_grid_session_key("grid.source.time"));
  // Where the data come from is the document's, not the session's.
  for (const char *k : {"grid.source.volume", "grid.source.points", "grid.source.session",
                        "grid.cursor.i", "grid.pick.seq", "grid.request", "grid.busy",
                        "grid.info.dims", "grid.classes.0.name", "transfer_function", "grid"})
    EXPECT_FALSE(cvc::volume_grid_session_key(k)) << k;
  cvc::volume_grid_session s = a_session();
  s.keys["grid.request"] = "load";
  EXPECT_THROW(cvc::write_volume_grid_session(s, "unused.json"), std::invalid_argument);
}

TEST(VolumeGridSession, BadFilesRaiseNamingTheFile) {
  scratch_dir dir("bad");
  const auto raises = [](const std::string &path, const std::string &want) {
    try {
      cvc::read_volume_grid_session(path);
      ADD_FAILURE() << path << " read";
    } catch (const std::runtime_error &e) {
      const std::string w = e.what();
      EXPECT_NE(w.find(path), std::string::npos) << w;
      EXPECT_NE(w.find(want), std::string::npos) << w;
    }
  };
  raises(dir.file("missing.json"), "can't read it");
  const auto write = [&dir](const std::string &name, const std::string &text) {
    std::ofstream(dir.file(name), std::ios::binary) << text;
    return dir.file(name);
  };
  raises(write("garbage.json", "{\"format\": \"cvc.volume_grid.session\", \"keys\": {"),
         "not JSON");
  raises(write("other.json", "{\"format\": \"something else\"}"), "not a volume_grid session");
  raises(write("v2.json", "{\"format\": \"cvc.volume_grid.session\", \"version\": 2}"),
         "version 2");
  raises(write("key.json",
               "{\"format\": \"cvc.volume_grid.session\", \"keys\": {\"grid.busy\": \"1\"}}"),
         "'grid.busy' is not a key a session keeps");
  raises(write("twice.json", "{\"format\": \"cvc.volume_grid.session\", \"classes\": "
                             "[{\"name\": \"a\", \"points\": [1, 2, 3, 1, 2, 3]}]}"),
         "twice");
  raises(write("nan.json", "{\"format\": \"cvc.volume_grid.session\", \"classes\": "
                           "[{\"name\": \"a\", \"radius\": \"wide\"}]}"),
         "classes[0].radius: 'wide' is not a number");
  // Truncated at every length: raises, never crashes.
  const std::string full = cvc::volume_grid_session_json(a_session());
  for (std::size_t n = 0; n + 1 < full.size(); n += 7)
    EXPECT_THROW(cvc::volume_grid_session_from_json(full.substr(0, n)), std::runtime_error) << n;
  // An unwritable file.
  EXPECT_THROW(cvc::write_volume_grid_session(a_session(), dir.file("no/such/dir/s.json")),
               std::runtime_error);
}

// ---- T6: no singletons ----

cvc::point_classes script(int seed) {
  point_class_editor ed = editor();
  cvc::point_class_gestures g;
  for (int n = 0; n < 200; ++n) {
    const int i = (n * 7 + seed) % 16, j = (n * 3 + seed * 5) % 16, k = (n + seed) % 4;
    const slice_axis a = static_cast<slice_axis>((n + seed) % 3);
    g.apply(ed, cvc::grid_tool::points, n % 2, 1.0, pick(kDown, a, i, j, k));
    g.apply(ed, cvc::grid_tool::points, n % 2, 1.0, pick(kUp, a, i, j, k));
    if (n % 37 == 0) {
      ed.select_rect(a, k, 0, 0, 8, 8, selection_mode::replace);
      ed.move_selection(a, 1, 1);
    }
    if (n % 53 == 0)
      ed.delete_selected();
  }
  return ed.classes();
}

TEST(VolumeGridNoSingletons, TwoEditorsShareNothing) {
  point_class_editor a = editor(), b = editor();
  a.add(0, {1, 1, 1});
  a.select_all();
  EXPECT_TRUE(b.classes().point_count() == 0 && b.selection().empty() && b.undo_depth() == 0);
  // A then B equals B then A equals each alone.
  const cvc::point_classes s1 = script(1), s2 = script(2);
  EXPECT_EQ(script(2), s2);
  EXPECT_EQ(script(1), s1);
  EXPECT_NE(s1, s2);
}

TEST(VolumeGridNoSingletons, TwoThreadsGetWhatEachGetsAlone) {
  const cvc::point_classes alone1 = script(3), alone2 = script(4);
  cvc::point_classes t1, t2;
  std::thread a([&t1] { t1 = script(3); });
  std::thread b([&t2] { t2 = script(4); });
  a.join();
  b.join();
  EXPECT_EQ(t1, alone1);
  EXPECT_EQ(t2, alone2);
}

TEST(VolumeGridNoSingletons, TwoAppsKeepTheirOwnRows) {
  cvc::app a, b;
  cvc::write_point_class_rows(a.root()("doc.grid.classes"), two_classes());
  EXPECT_EQ(a.root()("doc.grid.classes").value(), "2");
  EXPECT_FALSE(b.root().findDescendant("doc.grid.classes"));
}

} // namespace
