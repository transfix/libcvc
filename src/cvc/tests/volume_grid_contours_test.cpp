/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc/volume/volume_grid_contours.h, VolumeGridRover's contours: the VolumeRover ports' P10 rows
// for them (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §4):
//   - T2: the spline through the control points, the frame, what problems() reports;
//   - the editor: curves drawn per object, extended at their active or nearer end, closed, opened,
//     moved, selected, deleted and undone;
//   - the gestures: picks turned into those edits, as VolumeGridRover's contour marking did;
//   - T8: the display table and the object rows a list: binds, reconciled as rows are edited;
//   - stacks: the curves as a contour_stack per plane orientation, and a stack's polygons back;
//   - T9 / T5: the JSON round-trips bit for bit, and bad JSON raises saying why;
//   - T3 / T4: a tiled tube is closed, two-manifold and genus 0, faces out, and lies between the
//     planes; the same curves on XZ or ZY planes, or on a frame of other spacing and origin, give
//     the same surface placed there;
//   - T7: the stepper matches the blocking call bit for bit at budgets 1, 64, random and 0, its
//     progress rises in [0, 1], and a cancel leaves no result;
//   - T6: two threads tiling their own give what each gives alone; two editors share nothing.
// Runs natively and under node (the ports lane, T14).

#include <chrono>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/tiler.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_contours.h>
#include <gtest/gtest.h>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

using cvc::contour_pick;
using cvc::contour_selection_mode;
using cvc::grid_contour_editor;
using cvc::grid_contour_object;
using cvc::grid_contours;
using cvc::grid_curve;
using cvc::grid_curve_ref;
using cvc::grid_point_ref;
using cvc::slice_axis;

#ifndef CVC_WITH_PLANAR
#define SKIP_WITHOUT_PLANAR() GTEST_SKIP() << "libcvc was built without cvc::planar"
#else
#define SKIP_WITHOUT_PLANAR()                                                                      \
  do {                                                                                             \
  } while (0)
#endif

constexpr double pi = 3.14159265358979323846;

// A coordinate on a 2^-8 grid, unchanged by the tiler's snapping and by power-of-two scaling.
double q8(double v) { return std::round(v * 256) / 256; }

std::vector<double> ellipse(double cx, double cy, double a, double b, int n, double phase = 0) {
  std::vector<double> ru;
  for (int i = 0; i < n; ++i) {
    const double t = 2 * pi * i / n;
    const double w = 1 + 0.05 * std::cos(3 * t + phase);
    ru.push_back(q8(cx + a * w * std::cos(t)));
    ru.push_back(q8(cy + b * w * std::sin(t)));
  }
  return ru;
}

cvc::grid_frame frame64(double spacing = 1.0, double origin = 0.0) {
  cvc::grid_frame f;
  f.dims = {64, 64, 64};
  f.origin = {origin, origin, origin};
  f.spacing = {spacing, spacing, spacing};
  return f;
}

grid_contour_object object(const std::string &name, std::vector<double> color = {1, 0, 0}) {
  grid_contour_object o;
  o.name = name;
  o.color = std::move(color);
  return o;
}

// A tube of `slices` closed curves on the planes looking down `axis`, in a frame of `spacing`
// from `origin` (the curves' coordinates placed the same way in voxel units).
grid_contours tube(slice_axis axis, int slices, int n, double spacing = 1.0, double origin = 0.0) {
  grid_contours c;
  c.frame = frame64(spacing, origin);
  grid_contour_object o = object("tube", {0.2, 0.8, 0.3});
  for (int k = 0; k < slices; ++k) {
    grid_curve cv;
    cv.axis = axis;
    cv.index = 10 + k;
    cv.ru = ellipse(20 + 0.125 * k, 24, 6 + 0.25 * (k % 2), 5, n, 0.4 * k);
    for (double &v : cv.ru)
      v = origin + v * spacing;
    cv.closed = true;
    o.curves.push_back(std::move(cv));
  }
  c.objects.push_back(std::move(o));
  return c;
}

contour_pick pick(contour_pick::phase_t ph, double r, double u, std::int64_t index = 12,
                  slice_axis a = slice_axis::z) {
  contour_pick p;
  p.phase = ph;
  p.axis = a;
  p.index = index;
  p.r = r;
  p.u = u;
  p.inside = true;
  return p;
}

cvc::contour_pick_outcome click(cvc::grid_contour_gestures &g, grid_contour_editor &ed, long cur,
                                double r, double u, bool shift = false, bool ctrl = false) {
  contour_pick d = pick(contour_pick::phase_t::down, r, u);
  d.shift = shift;
  d.ctrl = ctrl;
  g.apply(ed, cur, 1.0, false, d);
  contour_pick up = pick(contour_pick::phase_t::up, r, u);
  up.shift = shift;
  up.ctrl = ctrl;
  return g.apply(ed, cur, 1.0, false, up);
}

// ---- T2 -----------------------------------------------------------------------------------------

TEST(GridContourModel, TheFramePlacesPlanes) {
  cvc::grid_frame f;
  f.dims = {10, 20, 30};
  f.origin = {1, 2, 3};
  f.spacing = {0.5, 0.25, 2};
  EXPECT_DOUBLE_EQ(f.plane(slice_axis::x, 4), 3.0);
  EXPECT_DOUBLE_EQ(f.plane(slice_axis::z, -1), 1.0) << "not clamped";
  EXPECT_EQ(f.nearest_index(slice_axis::y, 2.0 + 0.25 * 7 + 0.1), 7);
  EXPECT_EQ(f.nearest_index(slice_axis::z, 3 + 2 * 40.0), 40) << "past the last, not clamped";
}

TEST(GridContourModel, LinearDrawsTheControlPoints) {
  const std::vector<double> ru{0, 0, 1, 0, 1, 1, 1, 1, 0, 1};
  // The repeated (1, 1) goes; a closed curve's last point equal to its first goes too.
  EXPECT_EQ(cvc::sample_grid_curve(ru, false, cvc::contour_interpolation::linear, 5),
            (std::vector<double>{0, 0, 1, 0, 1, 1, 0, 1}));
  EXPECT_EQ(
      cvc::sample_grid_curve({0, 0, 1, 0, 0, 1, 0, 0}, true, cvc::contour_interpolation::linear, 5),
      (std::vector<double>{0, 0, 1, 0, 0, 1}));
  // A spline of two points, or with one sample per span, is the control points.
  EXPECT_EQ(cvc::sample_grid_curve({0, 0, 3, 4}, false, cvc::contour_interpolation::spline, 8),
            (std::vector<double>{0, 0, 3, 4}));
  EXPECT_EQ(cvc::sample_grid_curve(ru, false, cvc::contour_interpolation::spline, 1),
            (std::vector<double>{0, 0, 1, 0, 1, 1, 0, 1}));
  EXPECT_THROW(cvc::sample_grid_curve({0, 0, 1}, false, cvc::contour_interpolation::linear, 1),
               std::invalid_argument);
  EXPECT_THROW(cvc::sample_grid_curve(ru, false, cvc::contour_interpolation::spline, 0),
               std::invalid_argument);
  EXPECT_THROW(cvc::sample_grid_curve(ru, false, cvc::contour_interpolation::spline,
                                      cvc::grid_contour_max_samples + 1),
               std::invalid_argument);
}

TEST(GridContourModel, TheSplinePassesThroughItsControlPoints) {
  const std::vector<double> sq{0, 0, 4, 0, 4, 4, 0, 4};
  const std::vector<double> closed =
      cvc::sample_grid_curve(sq, true, cvc::contour_interpolation::spline, 5);
  ASSERT_EQ(closed.size(), 2u * 4 * 5) << "four spans of five, the first point not repeated";
  const std::vector<double> open =
      cvc::sample_grid_curve(sq, false, cvc::contour_interpolation::spline, 5);
  ASSERT_EQ(open.size(), 2u * (3 * 5 + 1)) << "three spans of five and the last point";
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(closed[2 * 5 * i], sq[2 * i]) << "control point " << i << " is on the curve";
    EXPECT_EQ(closed[2 * 5 * i + 1], sq[2 * i + 1]);
  }
  EXPECT_EQ(open[open.size() - 2], 0);
  EXPECT_EQ(open[open.size() - 1], 4);
  // A closed square's spline bulges out past its edges between the corners, and stays near them.
  bool out = false;
  for (std::size_t i = 0; i + 1 < closed.size(); i += 2) {
    out = out || closed[i] < 0 || closed[i] > 4 || closed[i + 1] < 0 || closed[i + 1] > 4;
    EXPECT_GT(closed[i], -1.0);
    EXPECT_LT(closed[i], 5.0);
  }
  EXPECT_TRUE(out);
  // Symmetric: the square's spline is symmetric about x = 2.
  for (std::size_t i = 0; i + 1 < closed.size(); i += 2) {
    bool mirrored = false;
    for (std::size_t j = 0; j + 1 < closed.size(); j += 2)
      mirrored = mirrored || (std::abs(closed[j] - (4 - closed[i])) < 1e-9 &&
                              std::abs(closed[j + 1] - closed[i + 1]) < 1e-9);
    EXPECT_TRUE(mirrored) << "sample " << i / 2;
  }
}

TEST(GridContourModel, ProblemsNameWhatIsWrong) {
  grid_contours c;
  EXPECT_TRUE(cvc::grid_contours_problems(c).empty());
  c.objects.push_back(object("a"));
  c.objects.push_back(object("a"));
  c.objects.push_back(object(""));
  c.objects[0].color = {1, 2, 0};
  c.objects[1].samples = 0;
  grid_curve odd;
  odd.ru = {1, 2, 3};
  grid_curve tiny;
  tiny.ru = {0, 0, 1, 1};
  tiny.closed = true;
  grid_curve empty;
  grid_curve nan;
  nan.ru = {0, std::nan("")};
  c.objects[0].curves = {odd, tiny, empty, nan};
  c.frame.spacing[1] = 0;
  c.frame.dims[2] = 0;
  const std::vector<std::string> p = cvc::grid_contours_problems(c);
  const auto has = [&p](const std::string &s) {
    for (const std::string &m : p)
      if (m.find(s) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(has("another object has that name"));
  EXPECT_TRUE(has("no name"));
  EXPECT_TRUE(has("outside [0, 1]"));
  EXPECT_TRUE(has("samples 0"));
  EXPECT_TRUE(has("odd number"));
  EXPECT_TRUE(has("fewer than three"));
  EXPECT_TRUE(has("no points"));
  EXPECT_TRUE(has("isn't finite"));
  EXPECT_TRUE(has("spacing[1]"));
  EXPECT_TRUE(has("dims[2]"));
}

// ---- tracing ------------------------------------------------------------------------------------

// A w x h slice down z whose values are the distance from (cx, cy) less r (a disk's SDF), plus a
// second disk when r2 > 0.
cvc::volume_slice_data disks(int w, int h, double cx, double cy, double r, double cx2 = 0,
                             double cy2 = 0, double r2 = 0) {
  cvc::volume_slice_data s;
  s.axis = slice_axis::z;
  s.width = w;
  s.height = h;
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      double d = std::hypot(x - cx, y - cy) - r;
      if (r2 > 0)
        d = std::min(d, std::hypot(x - cx2, y - cy2) - r2);
      s.values.push_back(d);
    }
  return s;
}

double area_of(const std::vector<double> &ru) {
  double a = 0;
  for (std::size_t i = 0, n = ru.size() / 2; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    a += ru[2 * i] * ru[2 * j + 1] - ru[2 * j] * ru[2 * i + 1];
  }
  return 0.5 * a;
}

TEST(GridContourTrace, ADiskTracesToOneLoopAroundIt) {
  cvc::grid_frame f = frame64(0.5, 10.0);
  const std::vector<std::vector<double>> loops =
      cvc::trace_slice(disks(40, 40, 20, 18, 9), f, 0.0, 0.1);
  ASSERT_EQ(loops.size(), 1u);
  const std::vector<double> &l = loops[0];
  EXPECT_GT(area_of(l), 0) << "counter-clockwise";
  // In object units: centre (10 + 20 * 0.5, 10 + 18 * 0.5), radius 9 * 0.5.
  for (std::size_t i = 0; i + 1 < l.size(); i += 2)
    EXPECT_NEAR(std::hypot(l[i] - 20, l[i + 1] - 19), 4.5, 0.1);
  EXPECT_NEAR(area_of(l), pi * 4.5 * 4.5, 0.05 * pi * 4.5 * 4.5);
  // A coarser tolerance keeps fewer points; a large min_area drops it.
  const std::size_t fine = l.size();
  EXPECT_LT(cvc::trace_slice(disks(40, 40, 20, 18, 9), f, 0.0, 1.0)[0].size(), fine);
  EXPECT_TRUE(cvc::trace_slice(disks(40, 40, 20, 18, 9), f, 0.0, 0.1, 1000).empty());
  EXPECT_THROW(cvc::trace_slice(disks(4, 4, 2, 2, 1), f, std::nan("")), std::invalid_argument);
  EXPECT_THROW(cvc::trace_slice(disks(4, 4, 2, 2, 1), f, 0, -1), std::invalid_argument);
}

TEST(GridContourTrace, TwoBlobsLargestFirstAndTheEdgeCloses) {
  const cvc::grid_frame f = frame64();
  const std::vector<std::vector<double>> loops =
      cvc::trace_slice(disks(48, 32, 12, 16, 8, 36, 16, 4), f, 0.0);
  ASSERT_EQ(loops.size(), 2u);
  EXPECT_GT(area_of(loops[0]), area_of(loops[1]));
  // A disk cut by the slice's edge closes along the edge (half a voxel past the last sample).
  const std::vector<std::vector<double>> cut = cvc::trace_slice(disks(20, 20, 0, 10, 6), f, 0.0);
  ASSERT_EQ(cut.size(), 1u);
  double lo = 1e9;
  for (std::size_t i = 0; i < cut[0].size(); i += 2)
    lo = std::min(lo, cut[0][i]);
  EXPECT_GE(lo, -1.0);
  EXPECT_LE(lo, 0.0);
  // Nothing below: no loop; everything below: one loop around the slice.
  EXPECT_TRUE(cvc::trace_slice(disks(10, 10, 5, 5, 2), f, -100.0).empty());
  EXPECT_EQ(cvc::trace_slice(disks(10, 10, 5, 5, 2), f, 100.0).size(), 1u);
  // The curves trace onto a frame's planes and tile.
  grid_contour_editor ed;
  ed.set_frame(f);
  ed.add_object(object("blob"));
  std::vector<grid_curve> curves;
  for (int k = 0; k < 4; ++k)
    for (std::vector<double> &ru : cvc::trace_slice(disks(32, 32, 16, 16, 6 + k % 2), f, 0.0)) {
      grid_curve cv;
      cv.index = 3 + k;
      cv.ru = std::move(ru);
      cv.closed = true;
      curves.push_back(std::move(cv));
    }
  ed.add_curves(0, curves);
  EXPECT_EQ(ed.contours().objects[0].curves.size(), 4u);
  EXPECT_THROW(ed.add_curves(1, curves), std::out_of_range);
#ifdef CVC_WITH_PLANAR
  EXPECT_GT(cvc::tile_grid_contours(ed.contours()).surfaces.at("blob").num_tris(), 20u);
#endif
}

// ---- the editor ---------------------------------------------------------------------------------

TEST(GridContourEditor, PointsExtendTheActiveCurveThenClose) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  EXPECT_EQ(ed.add_object(object("axon")), 0u);
  const grid_point_ref a = ed.add_point(0, slice_axis::z, 12, 3, 4);
  EXPECT_EQ(a.curve, 0u);
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 12, 9, 4.5).point, 1u);
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 12, 6, 9).point, 2u);
  ASSERT_TRUE(ed.active());
  EXPECT_TRUE(ed.active()->back);
  EXPECT_EQ(ed.selection(), (std::vector<grid_point_ref>{{0, 0, 2}})) << "the new point alone";
  EXPECT_EQ(ed.contours().objects[0].curves[0].ru, (std::vector<double>{3, 4, 9, 4.5, 6, 9}));
  EXPECT_TRUE(ed.close_active());
  EXPECT_TRUE(ed.contours().objects[0].curves[0].closed);
  EXPECT_FALSE(ed.active()) << "a closed curve is extended no more";
  // The next point starts a new curve: the closed one takes none.
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 12, 30, 30).curve, 1u);
  // On another plane: another curve.
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 13, 30, 30).curve, 2u);
  EXPECT_EQ(ed.add_point(0, slice_axis::y, 12, 30, 30).curve, 3u);
  EXPECT_EQ(ed.curves_on(slice_axis::z, 12).size(), 2u);
  EXPECT_FALSE(ed.close_curve({0, 1})) << "one point can't close";
  EXPECT_THROW(ed.add_point(3, slice_axis::z, 12, 1, 1), std::out_of_range);
  EXPECT_THROW(ed.add_point(0, slice_axis::z, 12, std::nan(""), 1), std::invalid_argument);
  EXPECT_THROW(ed.add_object(object("axon")), std::invalid_argument);
  EXPECT_THROW(ed.add_object(object("")), std::invalid_argument);
}

TEST(GridContourEditor, WithNoActiveCurveAPointGoesToTheNearerEnd) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  for (double r : {10.0, 20.0, 30.0})
    ed.add_point(0, slice_axis::z, 5, r, 10);
  ed.set_active(std::nullopt);
  // Near the first point: before it.
  const grid_point_ref p = ed.add_point(0, slice_axis::z, 5, 5, 10);
  EXPECT_EQ(p.point, 0u);
  ASSERT_TRUE(ed.active());
  EXPECT_FALSE(ed.active()->back);
  // The active end now is the front: the next goes there too, even near the back.
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 5, 31, 10).point, 0u);
  // A new curve on request.
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 5, 31, 11, true).curve, 1u);
  ed.set_active(std::nullopt);
  // Two open curves: the one whose end is nearest takes it.
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 5, 32, 12).curve, 1u);
  EXPECT_EQ(ed.contours().objects[0].curves[0].size(), 5u);
  // An active curve of another object doesn't take it.
  ed.add_object(object("mito"));
  ed.add_point(1, slice_axis::z, 5, 40, 40);
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 5, 41, 40).object, 0u);
}

TEST(GridContourEditor, CurvesOpenDeleteAndClear) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  ed.add_object(object("mito"));
  for (int k = 0; k < 3; ++k) {
    for (double r : {1.0, 5.0, 3.0})
      ed.add_point(0, slice_axis::z, k, r, r * k);
    ed.close_active();
  }
  ed.add_point(1, slice_axis::z, 0, 9, 9);
  ed.select({0, 2, 1}, contour_selection_mode::replace);
  EXPECT_TRUE(ed.open_curve({0, 2}));
  ASSERT_TRUE(ed.active());
  EXPECT_EQ(ed.active()->curve, (grid_curve_ref{0, 2}));
  EXPECT_FALSE(ed.open_curve({0, 2})) << "already open";
  // Deleting curve 1 moves curve 2 (selected, active) down one.
  EXPECT_TRUE(ed.delete_curve({0, 1}));
  EXPECT_EQ(ed.selection(), (std::vector<grid_point_ref>{{0, 1, 1}}));
  EXPECT_EQ(ed.active()->curve, (grid_curve_ref{0, 1}));
  EXPECT_FALSE(ed.delete_curve({0, 9}));
  EXPECT_EQ(ed.clear_object(0), 2u);
  EXPECT_TRUE(ed.selection().empty());
  EXPECT_FALSE(ed.active());
  EXPECT_EQ(ed.contours().objects[1].curves.size(), 1u);
  EXPECT_TRUE(ed.remove_object(0));
  EXPECT_EQ(ed.contours().objects.size(), 1u);
  EXPECT_EQ(ed.contours().objects[0].name, "mito");
  // Undo walks back through each step.
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.contours().objects.size(), 2u);
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.contours().objects[0].curves.size(), 2u);
}

TEST(GridContourEditor, SelectMoveAndDelete) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  ed.add_object(object("hidden"));
  for (double r : {10.0, 20.0, 30.0, 40.0})
    ed.add_point(0, slice_axis::z, 7, r, 10);
  ed.close_active();
  ed.add_point(1, slice_axis::z, 7, 25, 10);
  std::vector<grid_contour_object> objs = ed.contours().objects;
  objs[1].visible = false;
  ed.set_objects(objs);
  // A rectangle over the middle two (and the hidden object's point, which isn't taken).
  EXPECT_EQ(ed.select_rect(slice_axis::z, 7, 15, 0, 35, 20, contour_selection_mode::replace), 2u);
  EXPECT_EQ(ed.select_rect(slice_axis::z, 8, 0, 0, 64, 64, contour_selection_mode::add), 0u)
      << "another plane";
  EXPECT_EQ(ed.selection().size(), 2u);
  ed.select({0, 0, 0}, contour_selection_mode::add);
  ed.select({0, 0, 1}, contour_selection_mode::remove);
  EXPECT_EQ(ed.selection(), (std::vector<grid_point_ref>{{0, 0, 0}, {0, 0, 2}}));
  ed.select({1, 0, 0}, contour_selection_mode::add);
  EXPECT_EQ(ed.selection().size(), 2u) << "a hidden object's point isn't selected";
  ed.move_selection(1, -2);
  EXPECT_EQ(ed.contours().objects[0].curves[0].ru,
            (std::vector<double>{11, 8, 20, 10, 31, 8, 40, 10}));
  ed.select_all();
  EXPECT_EQ(ed.selection().size(), 4u) << "every point of the visible objects";
  // Delete two of four: the closed curve, left with two, opens.
  ed.select_rect(slice_axis::z, 7, 0, 0, 25, 20, contour_selection_mode::replace);
  EXPECT_EQ(ed.delete_selected(), 2u);
  EXPECT_EQ(ed.contours().objects[0].curves[0].ru, (std::vector<double>{31, 8, 40, 10}));
  EXPECT_FALSE(ed.contours().objects[0].curves[0].closed);
  // Delete the rest: the curve goes.
  ed.select_all();
  EXPECT_EQ(ed.delete_selected(), 2u);
  EXPECT_TRUE(ed.contours().objects[0].curves.empty());
  EXPECT_EQ(ed.contours().objects[1].curves.size(), 1u);
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.contours().objects[0].curves[0].size(), 2u);
}

TEST(GridContourEditor, DeletingTheLastPointKeepsDrawingFromTheOneBefore) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  ed.add_point(0, slice_axis::z, 3, 1, 1);
  ed.add_point(0, slice_axis::z, 3, 2, 2);
  ed.add_point(0, slice_axis::z, 3, 9, 9); // a slip
  EXPECT_EQ(ed.delete_selected(), 1u);
  ASSERT_TRUE(ed.active());
  EXPECT_EQ(ed.add_point(0, slice_axis::z, 3, 3, 1).point, 2u);
  EXPECT_EQ(ed.contours().objects[0].curves[0].ru, (std::vector<double>{1, 1, 2, 2, 3, 1}));
}

TEST(GridContourEditor, UndoIsBoundedAndSetContoursChecks) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  for (std::size_t i = 0; i < grid_contour_editor::undo_limit + 10; ++i)
    ed.add_point(0, slice_axis::z, 1, static_cast<double>(i), 0);
  EXPECT_EQ(ed.undo_depth(), grid_contour_editor::undo_limit);
  grid_contours bad;
  bad.objects.push_back(object(""));
  EXPECT_THROW(ed.set_contours(bad), std::invalid_argument);
  const std::uint64_t v = ed.version();
  ed.set_contours(tube(slice_axis::z, 3, 8));
  EXPECT_GT(ed.version(), v);
  EXPECT_TRUE(ed.undo());
  EXPECT_EQ(ed.contours().objects[0].name, "axon");
}

// ---- gestures -----------------------------------------------------------------------------------

TEST(GridContourGestures, ClicksDrawACurveAndTheFirstPointClosesIt) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  cvc::grid_contour_gestures g;
  EXPECT_NE(click(g, ed, 0, 10, 10).note.find("started a curve of axon on z = 12"),
            std::string::npos);
  click(g, ed, 0, 20, 10);
  const cvc::contour_pick_outcome third = click(g, ed, 0, 15, 20);
  EXPECT_TRUE(third.changed);
  EXPECT_NE(third.note.find("added point 3"), std::string::npos) << third.note;
  // A click on the last point (selected, the active end) does nothing but select it.
  click(g, ed, 0, 15.2, 20.1);
  EXPECT_FALSE(ed.contours().objects[0].curves[0].closed);
  EXPECT_EQ(ed.contours().objects[0].curves[0].size(), 3u);
  // On the first: it closes.
  const cvc::contour_pick_outcome close = click(g, ed, 0, 10.3, 9.8);
  EXPECT_NE(close.note.find("closed a curve of axon"), std::string::npos) << close.note;
  EXPECT_TRUE(ed.contours().objects[0].curves[0].closed);
  // Shift+click starts a new curve; with no object, or a hidden one, nothing is added.
  click(g, ed, 0, 40, 40);
  click(g, ed, 0, 45, 40, true);
  EXPECT_EQ(ed.contours().objects[0].curves.size(), 3u);
  EXPECT_NE(click(g, ed, -1, 50, 50).note.find("no object"), std::string::npos);
  std::vector<grid_contour_object> objs = ed.contours().objects;
  objs[0].visible = false;
  ed.set_objects(objs);
  EXPECT_NE(click(g, ed, 0, 50, 50).note.find("hidden"), std::string::npos);
  // Past the volume's edge.
  objs[0].visible = true;
  ed.set_objects(objs);
  contour_pick d = pick(contour_pick::phase_t::down, -5, 3);
  d.inside = false;
  g.apply(ed, 0, 1.0, false, d);
  d.phase = contour_pick::phase_t::up;
  EXPECT_NE(g.apply(ed, 0, 1.0, false, d).note.find("edge"), std::string::npos);
}

TEST(GridContourGestures, AClickOnAPointMakesItsEndActiveAndItsObjectCurrent) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  ed.add_object(object("mito"));
  for (double r : {10.0, 20.0, 30.0})
    ed.add_point(1, slice_axis::z, 12, r, 10);
  ed.set_active(std::nullopt);
  cvc::grid_contour_gestures g;
  const cvc::contour_pick_outcome o = click(g, ed, 0, 10, 10);
  EXPECT_EQ(o.object, 1) << "the node makes mito current";
  ASSERT_TRUE(ed.active());
  EXPECT_FALSE(ed.active()->back) << "the first point: the front";
  click(g, ed, 1, 5, 10);
  EXPECT_EQ(ed.contours().objects[1].curves[0].ru,
            (std::vector<double>{5, 10, 10, 10, 20, 10, 30, 10}));
  // A middle point: selected, no end active.
  click(g, ed, 1, 20, 10);
  EXPECT_FALSE(ed.active());
  EXPECT_EQ(ed.selection(), (std::vector<grid_point_ref>{{1, 0, 2}}));
  // Ctrl+click takes it from the selection; Shift+click adds.
  click(g, ed, 1, 20, 10, false, true);
  EXPECT_TRUE(ed.selection().empty());
  click(g, ed, 1, 30, 10, true);
  click(g, ed, 1, 5, 10, true);
  EXPECT_EQ(ed.selection().size(), 2u);
}

TEST(GridContourGestures, DragsMoveAPointOrDrawABand) {
  grid_contour_editor ed;
  ed.set_frame(frame64());
  ed.add_object(object("axon"));
  for (double r : {10.0, 20.0, 30.0})
    ed.add_point(0, slice_axis::z, 12, r, 10);
  ed.clear_selection();
  cvc::grid_contour_gestures g;
  const std::size_t depth = ed.undo_depth();
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::down, 20, 10));
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::drag, 20.1, 10)); // a tremor
  EXPECT_EQ(ed.undo_depth(), depth);
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::drag, 22, 13));
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::drag, 23, 15));
  EXPECT_NE(g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::up, 23, 15)).note.find("moved"),
            std::string::npos);
  EXPECT_EQ(ed.point({0, 0, 1}), (std::array<double, 2>{23, 15}));
  EXPECT_EQ(ed.undo_depth(), depth + 1) << "one undo step for the drag";
  // A band from empty space selects what it spans.
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::down, 0, 0));
  const cvc::contour_pick_outcome b =
      g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::drag, 25, 20));
  EXPECT_TRUE(b.band);
  EXPECT_EQ(b.band_rect, (std::array<double, 4>{0, 0, 25, 20}));
  EXPECT_NE(g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::up, 25, 20))
                .note.find("2 points selected"),
            std::string::npos);
  // A drag from a selected point moves the selection; with snap, by whole voxels.
  g.apply(ed, 0, 1.0, true, pick(contour_pick::phase_t::down, 10, 10));
  g.apply(ed, 0, 1.0, true, pick(contour_pick::phase_t::drag, 12.4, 9.2));
  g.apply(ed, 0, 1.0, true, pick(contour_pick::phase_t::up, 12.4, 9.2));
  EXPECT_EQ(ed.point({0, 0, 0}), (std::array<double, 2>{12, 9}));
  EXPECT_EQ(ed.point({0, 0, 1}), (std::array<double, 2>{25, 14}));
  EXPECT_EQ(ed.point({0, 0, 2}), (std::array<double, 2>{30, 10}));
  // A drag or up with no down starts nothing; a pick on another plane mid-gesture is ignored.
  EXPECT_FALSE(g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::up, 1, 1)).changed);
  g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::down, 40, 40));
  EXPECT_FALSE(g.apply(ed, 0, 1.0, false, pick(contour_pick::phase_t::up, 40, 40, 13)).changed);
}

TEST(GridContourGestures, SnapPutsNewPointsOnVoxelCentres) {
  grid_contour_editor ed;
  cvc::grid_frame f = frame64(0.5, 1.0);
  ed.set_frame(f);
  ed.add_object(object("axon"));
  cvc::grid_contour_gestures g;
  g.apply(ed, 0, 0.5, true, pick(contour_pick::phase_t::down, 3.3, 2.2));
  g.apply(ed, 0, 0.5, true, pick(contour_pick::phase_t::up, 3.3, 2.2));
  EXPECT_EQ(ed.point({0, 0, 0}), (std::array<double, 2>{3.5, 2.0}));
}

// ---- T8: parameters and rows --------------------------------------------------------------------

TEST(GridContourParams, TheDisplayTableRoundTripsThroughTheTree) {
  cvc::app app;
  cvc::state &node = app.root()("grid.contour");
  cvc::volume_grid_contour_params p;
  p.point_size = 1.25;
  p.snap = true;
  p.source = "data.c.contours";
  cvc::params_to_state(p, node);
  const cvc::param_result<cvc::volume_grid_contour_params> r =
      cvc::params_from_state<cvc::volume_grid_contour_params>(node);
  ASSERT_TRUE(r.ok()) << r.message();
  EXPECT_EQ(r.value.point_size, 1.25);
  EXPECT_TRUE(r.value.snap);
  EXPECT_EQ(r.value.source, "data.c.contours");
  EXPECT_EQ(node("point_size").property("unit"), "voxel");
  node("surface_opacity").value("2");
  EXPECT_FALSE(cvc::params_from_state<cvc::volume_grid_contour_params>(node).ok());
}

TEST(GridContourRows, RowsRoundTripAndReconcileListEdits) {
  cvc::app app;
  cvc::state &rows = app.root()("grid.objects");
  grid_contours c = tube(slice_axis::z, 2, 8);
  c.objects.push_back(object("mito", {0, 0, 1}));
  c.objects[1].interpolation = cvc::contour_interpolation::spline;
  c.objects[1].samples = 7;
  c.objects[1].visible = false;
  cvc::write_contour_object_rows(rows, c);
  EXPECT_EQ(rows.value(), "2");
  EXPECT_EQ(rows("0.curves").value(), "2");
  EXPECT_EQ(rows("0.points").value(), "16");
  EXPECT_EQ(rows("1.interpolation").value(), "spline");
  std::vector<std::string> problems;
  std::vector<grid_contour_object> back = cvc::read_contour_object_rows(rows, &problems);
  EXPECT_TRUE(problems.empty());
  ASSERT_EQ(back.size(), 2u);
  EXPECT_EQ(back[1].samples, 7);
  EXPECT_FALSE(back[1].visible);
  EXPECT_EQ(back[1].color, (std::vector<double>{0, 0, 1}));
  // A row renamed keeps its curves.
  rows("0.name").value("axon");
  back = cvc::read_contour_object_rows(rows);
  std::vector<grid_contour_object> next = cvc::reconcile_contour_object_rows(c, back);
  EXPECT_EQ(next[0].name, "axon");
  EXPECT_EQ(next[0].curves.size(), 2u);
  // A row added (as a list's Add writes it) starts empty; one removed takes its curves along.
  rows("2.name").value("object 3");
  rows.value("3");
  next = cvc::reconcile_contour_object_rows(c, cvc::read_contour_object_rows(rows));
  ASSERT_EQ(next.size(), 3u);
  EXPECT_TRUE(next[0].curves.empty()) << "renamed and resized at once: no name matches";
  EXPECT_TRUE(next[2].curves.empty());
  // Bad values are reported and take the default.
  rows("1.samples").value("999");
  rows("1.interpolation").value("cubic");
  rows("1.color").value("red");
  problems.clear();
  back = cvc::read_contour_object_rows(rows, &problems);
  EXPECT_EQ(problems.size(), 3u);
  EXPECT_EQ(back[1].samples, 5);
  // Empty and repeated names are made unique.
  std::vector<grid_contour_object> named{object("a"), object(""), object("a")};
  next = cvc::reconcile_contour_object_rows(grid_contours(), named);
  EXPECT_EQ(next[0].name, "a");
  EXPECT_EQ(next[1].name, "object 2");
  EXPECT_EQ(next[2].name, "object 3");
  // Writing fewer rows removes the rest.
  cvc::write_contour_object_rows(rows, grid_contours());
  EXPECT_EQ(rows.value(), "0");
  EXPECT_EQ(rows.findDescendant("0"), nullptr);
}

// ---- stacks -------------------------------------------------------------------------------------

TEST(GridContourStacks, ClosedCurvesOfOneOrientationMakeAStack) {
  grid_contours c = tube(slice_axis::z, 3, 6, 0.5, 2.0);
  c.objects.push_back(object("mito", {0, 0, 1}));
  grid_curve open;
  open.axis = slice_axis::z;
  open.index = 11;
  open.ru = {3, 3, 4, 4};
  c.objects[1].curves.push_back(open);
  grid_curve side = c.objects[0].curves[0];
  side.axis = slice_axis::x;
  c.objects[0].curves.push_back(side);
  std::size_t n_open = 0;
  const cvc::contour_stack s = cvc::grid_contours_stack(c, slice_axis::z, &n_open);
  EXPECT_EQ(n_open, 1u);
  EXPECT_EQ(s.component_names(), (std::vector<std::string>{"tube", "mito"}));
  EXPECT_EQ(s.indices(), (std::vector<int>{10, 11, 12}));
  EXPECT_EQ(s.polygon_count(), 3u);
  EXPECT_EQ(s.z_origin, 2.0);
  EXPECT_EQ(s.z_spacing, 0.5);
  EXPECT_DOUBLE_EQ(s.slice_z(11), 7.5);
  EXPECT_EQ(s.polygons(10, "tube")[0].xy, c.objects[0].curves[0].ru);
  EXPECT_TRUE(cvc::contour_problems(s).empty());
  // The other orientation, and a spline drawn.
  EXPECT_EQ(cvc::grid_contours_stack(c, slice_axis::x).polygon_count(), 1u);
  c.objects[0].interpolation = cvc::contour_interpolation::spline;
  c.objects[0].samples = 4;
  EXPECT_EQ(cvc::grid_contours_stack(c).polygons(10, "tube")[0].size(), 24u);
  c.objects[0].name.clear();
  EXPECT_THROW(cvc::grid_contours_stack(c), std::invalid_argument);
}

TEST(GridContourStacks, AStacksPolygonsComeBackOnTheNearestPlanes) {
  const grid_contours c = tube(slice_axis::y, 4, 8, 0.5, 2.0);
  const cvc::contour_stack s = cvc::grid_contours_stack(c, slice_axis::y);
  std::vector<std::string> warnings;
  const grid_contours back = cvc::grid_contours_from_stack(s, c.frame, slice_axis::y, &warnings);
  EXPECT_TRUE(warnings.empty());
  EXPECT_EQ(back, c);
  // A stack of another spacing: slices land on the nearest planes, and the far ones are said.
  cvc::contour_stack t = s;
  t.z_spacing = 0.7;
  warnings.clear();
  const grid_contours moved = cvc::grid_contours_from_stack(t, c.frame, slice_axis::y, &warnings);
  EXPECT_EQ(moved.objects[0].curves[1].index,
            cvc::grid_frame(c.frame).nearest_index(slice_axis::y, t.slice_z(11)));
  EXPECT_FALSE(warnings.empty());
  cvc::contour_stack bad;
  bad.add(1, "x", {0, 0, 1, 1});
  EXPECT_THROW(cvc::grid_contours_from_stack(bad, c.frame), std::invalid_argument);
}

// ---- T9 / T5: JSON ------------------------------------------------------------------------------

TEST(GridContourFiles, JsonRoundTripsBitForBit) {
  grid_contours c = tube(slice_axis::z, 3, 7, 0.1, -3.3);
  c.objects.push_back(object("odd \"name\"\n", {0.1, 0.2, 0.30000000000000004}));
  c.objects[1].interpolation = cvc::contour_interpolation::spline;
  c.objects[1].samples = 9;
  c.objects[1].visible = false;
  grid_curve open;
  open.axis = slice_axis::x;
  open.index = -2;
  open.ru = {1.0 / 3, 2.0 / 3};
  c.objects[1].curves.push_back(open);
  c.objects.push_back(object("empty"));
  const std::string json = cvc::grid_contours_json(c);
  EXPECT_EQ(cvc::grid_contours_from_json(json), c);
  EXPECT_EQ(cvc::grid_contours_json(cvc::grid_contours_from_json(json)), json);
  EXPECT_EQ(cvc::grid_contours_from_json(cvc::grid_contours_json(grid_contours())),
            grid_contours());
}

TEST(GridContourFiles, BadJsonRaisesSayingWhy) {
  const auto why = [](const std::string &json) {
    try {
      cvc::grid_contours_from_json(json, "t.json");
    } catch (const std::runtime_error &e) {
      return std::string(e.what());
    }
    return std::string("(read)");
  };
  EXPECT_NE(why("{nope").find("t.json: not JSON"), std::string::npos);
  EXPECT_NE(why("{\"format\": \"other\"}").find("not volume grid contours"), std::string::npos);
  const std::string head = "{\"format\": \"cvc.volume_grid.contours\", ";
  EXPECT_NE(why(head + "\"version\": 7}").find("version 7"), std::string::npos);
  EXPECT_NE(why(head + "\"objects\": [{\"name\": \"a\", \"curves\": [{\"axis\": \"w\"}]}]}")
                .find("axis: 'w'"),
            std::string::npos);
  EXPECT_NE(why(head + "\"objects\": [{\"name\": \"a\", \"curves\": [{\"axis\": \"z\", "
                       "\"closed\": true, \"points\": [0, 0, 1, 1]}]}]}")
                .find("fewer than three"),
            std::string::npos);
  EXPECT_NE(why(head + "\"frame\": {\"dims\": [1, 2]}}").find("not three"), std::string::npos);
  EXPECT_NE(
      why(head + "\"objects\": [{\"name\": \"a\", \"interpolation\": \"cubic\"}]}").find("cubic"),
      std::string::npos);
  grid_contours bad;
  bad.objects.push_back(object(""));
  EXPECT_THROW(cvc::grid_contours_json(bad), std::invalid_argument);
}

TEST(GridContourFiles, ASessionKeepsTheContoursAndTheirKeys) {
  cvc::volume_grid_session s;
  s.keys["grid.tool"] = "contours";
  s.keys["grid.objects.current"] = "1";
  s.keys["grid.contour.point_size"] = "0.8";
  s.keys["grid.tile.curation_delta"] = "0.01";
  s.contours = tube(slice_axis::z, 2, 5, 0.1, 1.0 / 3);
  s.contours.objects.push_back(object("mito", {0, 0, 1}));
  const std::string json = cvc::volume_grid_session_json(s);
  const cvc::volume_grid_session back = cvc::volume_grid_session_from_json(json);
  EXPECT_EQ(back.contours, s.contours) << "bit for bit, through the session's JSON tree";
  EXPECT_EQ(back.keys, s.keys);
  // Where the contours come from is the document's; the job's keys are transient.
  EXPECT_FALSE(cvc::volume_grid_session_key("grid.contour.source"));
  EXPECT_FALSE(cvc::volume_grid_session_key("grid.tile.busy"));
  EXPECT_TRUE(cvc::volume_grid_session_key("grid.contour.auto_tile"));
  // A session without contours reads as before, and writes no "contours".
  cvc::volume_grid_session plain;
  EXPECT_EQ(cvc::volume_grid_session_json(plain).find("contours"), std::string::npos);
  EXPECT_TRUE(
      cvc::volume_grid_session_from_json(cvc::volume_grid_session_json(plain)).contours.empty());
  // Bad contours in a session raise naming them.
  std::string broken = json;
  broken.replace(broken.find("\"closed\": true"), 14, "\"closed\": 7");
  try {
    cvc::volume_grid_session_from_json(broken, "s.json");
    ADD_FAILURE() << "read";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("s.json: contours"), std::string::npos) << e.what();
  }
}

// ---- T3 / T4: tiling ----------------------------------------------------------------------------

struct mesh_check {
  std::size_t vertices = 0, edges = 0, faces = 0;
  bool manifold = true;           // every edge in exactly two triangles
  double volume = 0;              // signed: positive when the triangles face out
  double lo = 1e300, hi = -1e300; // along the axis
};

mesh_check check(const cvc::geometry &g, int axis) {
  mesh_check m;
  // Vertices welded by position (the tiler shares them already; this doesn't trust it).
  std::map<std::array<double, 3>, std::size_t> weld;
  std::vector<std::size_t> id(g.num_points());
  for (std::size_t i = 0; i < g.num_points(); ++i) {
    const auto &p = g.points()[i];
    id[i] = weld.emplace(std::array<double, 3>{p[0], p[1], p[2]}, weld.size()).first->second;
    m.lo = std::min(m.lo, p[axis]);
    m.hi = std::max(m.hi, p[axis]);
  }
  m.vertices = weld.size();
  std::map<std::pair<std::size_t, std::size_t>, int> edges;
  for (const auto &t : g.tris()) {
    const auto &a = g.points()[t[0]], &b = g.points()[t[1]], &c = g.points()[t[2]];
    m.volume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                 a[2] * (b[0] * c[1] - b[1] * c[0])) /
                6.0;
    for (int k = 0; k < 3; ++k) {
      std::size_t u = id[t[k]], v = id[t[(k + 1) % 3]];
      if (u > v)
        std::swap(u, v);
      ++edges[{u, v}];
    }
  }
  m.faces = g.num_tris();
  m.edges = edges.size();
  for (const auto &e : edges)
    m.manifold = m.manifold && e.second == 2;
  return m;
}

TEST(GridContourTiling, ATubeIsAClosedGenusZeroSurfaceFacingOut) {
  SKIP_WITHOUT_PLANAR();
  const grid_contours c = tube(slice_axis::z, 6, 24);
  const cvc::grid_tile_result r = cvc::tile_grid_contours(c);
  EXPECT_TRUE(r.warnings.empty()) << r.warnings.front();
  ASSERT_EQ(r.surfaces.count("tube"), 1u);
  const mesh_check m = check(r.surfaces.at("tube"), 2);
  EXPECT_TRUE(m.manifold);
  EXPECT_EQ(static_cast<long>(m.vertices) - static_cast<long>(m.edges) + static_cast<long>(m.faces),
            2)
      << "genus 0";
  EXPECT_GT(m.volume, 0.0) << "the triangles face out";
  // A prism of the sections, about: 6 sections of an ellipse 6.1 x 5 (pi a b), spacing 1, plus
  // the caps out to the next planes.
  EXPECT_NEAR(m.volume, pi * 6.125 * 5 * 6, 0.25 * pi * 6.125 * 5 * 6);
  EXPECT_GE(m.lo, 9.0 - 1e-9) << "the caps reach no further than the next planes";
  EXPECT_LE(m.hi, 16.0 + 1e-9);
  const cvc::geometry &g = r.surfaces.at("tube");
  ASSERT_EQ(g.normals().size(), g.points().size());
  ASSERT_EQ(g.colors().size(), g.points().size());
  EXPECT_NEAR(g.colors()[0][1], 0.8, 1e-6) << "the object's colour";
}

TEST(GridContourTiling, TheOtherOrientationsGiveTheSameSurfacePlacedThere) {
  SKIP_WITHOUT_PLANAR();
  const cvc::geometry z = cvc::tile_grid_contours(tube(slice_axis::z, 5, 16)).surfaces.at("tube");
  for (slice_axis a : {slice_axis::y, slice_axis::x}) {
    const cvc::geometry g = cvc::tile_grid_contours(tube(a, 5, 16)).surfaces.at("tube");
    ASSERT_EQ(g.num_points(), z.num_points());
    ASSERT_EQ(g.num_tris(), z.num_tris());
    const std::pair<int, int> ru = cvc::grid_plane_axes(a);
    const int w = static_cast<int>(a);
    for (std::size_t i = 0; i < g.num_points(); ++i) {
      EXPECT_EQ(g.points()[i][ru.first], z.points()[i][0]);
      EXPECT_EQ(g.points()[i][ru.second], z.points()[i][1]);
      EXPECT_EQ(g.points()[i][w], z.points()[i][2]);
      EXPECT_EQ(g.normals()[i][w], z.normals()[i][2]);
    }
    const mesh_check m = check(g, w);
    EXPECT_TRUE(m.manifold);
    EXPECT_GT(m.volume, 0.0) << cvc::slice_axis_name(a) << ": the triangles face out";
    EXPECT_NEAR(m.volume, check(z, 2).volume, 1e-6);
  }
}

TEST(GridContourTiling, AnyFrameGivesTheSameSurfaceScaled) {
  SKIP_WITHOUT_PLANAR();
  // In voxel units the tiler sees the same stack, so the surface is the same, scaled and moved:
  // exactly, for a power-of-two spacing.
  const cvc::geometry unit =
      cvc::tile_grid_contours(tube(slice_axis::z, 5, 16)).surfaces.at("tube");
  const cvc::geometry scaled =
      cvc::tile_grid_contours(tube(slice_axis::z, 5, 16, 1.0 / 64, 3.5)).surfaces.at("tube");
  ASSERT_EQ(scaled.num_tris(), unit.num_tris());
  for (std::size_t i = 0; i < unit.num_points(); ++i)
    for (int k = 0; k < 3; ++k)
      EXPECT_EQ(scaled.points()[i][k], 3.5 + unit.points()[i][k] / 64) << i << " " << k;
  // Without the scaling the tiler's absolute tolerances would see 1/64-voxel coordinates; the
  // lengths in the parameters scale with them (a curation_delta of a fifth of a voxel is allowed).
  cvc::tile_params p;
  p.curation_delta = 0.2 / 64;
  EXPECT_NO_THROW(cvc::tile_grid_contours(tube(slice_axis::z, 5, 16, 1.0 / 64), p));
  p.curation_delta = 2.0 / 64;
  EXPECT_THROW(cvc::tile_grid_contours(tube(slice_axis::z, 5, 16, 1.0 / 64), p), cvc::tiler_error)
      << "past the slice spacing";
}

TEST(GridContourTiling, OrientationsJoinAndOpenCurvesAreSaid) {
  SKIP_WITHOUT_PLANAR();
  grid_contours c = tube(slice_axis::z, 4, 12);
  const grid_contours side = tube(slice_axis::x, 4, 12);
  for (const grid_curve &cv : side.objects[0].curves)
    c.objects[0].curves.push_back(cv);
  grid_curve open;
  open.axis = slice_axis::z;
  open.index = 30;
  open.ru = {1, 1, 5, 5};
  c.objects[0].curves.push_back(open);
  const cvc::grid_tile_result r = cvc::tile_grid_contours(c);
  const cvc::geometry one = cvc::tile_grid_contours(tube(slice_axis::z, 4, 12)).surfaces.at("tube");
  EXPECT_EQ(r.surfaces.at("tube").num_tris(), 2 * one.num_tris());
  ASSERT_FALSE(r.warnings.empty());
  EXPECT_NE(r.warnings.front().find("1 open curve of 'tube' left out"), std::string::npos)
      << r.warnings.front();
}

TEST(GridContourTiling, BadInputThrows) {
  SKIP_WITHOUT_PLANAR();
  grid_contours none;
  none.objects.push_back(object("a"));
  EXPECT_THROW(cvc::tile_grid_contours(none), cvc::tiler_error);
  grid_contours bad = tube(slice_axis::z, 3, 8);
  bad.objects[0].samples = 0;
  EXPECT_THROW(cvc::tile_grid_contours(bad), cvc::tiler_error);
  cvc::tile_params p;
  p.curation_delta = -1;
  EXPECT_THROW(cvc::tile_grid_contours(tube(slice_axis::z, 3, 8), p), cvc::tiler_error);
}

// ---- T7: resumable ------------------------------------------------------------------------------

std::vector<double> flat(const cvc::grid_tile_result &r) {
  std::vector<double> out;
  for (const auto &[name, g] : r.surfaces) {
    for (const auto &p : g.points())
      out.insert(out.end(), {p[0], p[1], p[2]});
    for (const auto &t : g.tris())
      out.insert(out.end(), {double(t[0]), double(t[1]), double(t[2])});
  }
  return out;
}

grid_contours two_orientations() {
  grid_contours c = tube(slice_axis::z, 5, 20);
  const grid_contours side = tube(slice_axis::y, 4, 16); // not a temporary in the loop's range
  for (const grid_curve &cv : side.objects[0].curves)
    c.objects[0].curves.push_back(cv);
  return c;
}

TEST(GridContourTiling, TheStepperMatchesTheBlockingCallAtEveryBudget) {
  SKIP_WITHOUT_PLANAR();
  const grid_contours c = two_orientations();
  const std::vector<double> want = flat(cvc::tile_grid_contours(c));
  std::mt19937 rng(7);
  for (int mode = 0; mode < 4; ++mode) {
    cvc::grid_tile_stepper s(c);
    double last = 0;
    std::set<std::string> phases, labels;
    std::size_t steps = 0;
    for (;;) {
      std::size_t budget = mode == 0 ? 1 : mode == 1 ? 64 : mode == 2 ? 1 + rng() % 500 : 0;
      const bool done = s.step(budget);
      ++steps;
      EXPECT_GE(s.progress(), last);
      EXPECT_LE(s.progress(), 1.0);
      last = s.progress();
      phases.insert(s.phase());
      if (!s.label().empty())
        labels.insert(s.label().substr(0, 2));
      if (done)
        break;
    }
    EXPECT_EQ(s.progress(), 1.0);
    EXPECT_EQ(std::string(s.phase()), "done");
    EXPECT_EQ(flat(s.take()), want) << "budget mode " << mode;
    if (mode < 3) {
      EXPECT_GT(steps, 2u);
      EXPECT_TRUE(phases.count("tiling"));
      EXPECT_TRUE(labels.count("XY")) << "the label names the orientation";
      EXPECT_TRUE(labels.count("XZ"));
    }
  }
}

TEST(GridContourTiling, ACancelLeavesNoResultAndTheJobHandsOneOut) {
  SKIP_WITHOUT_PLANAR();
  const grid_contours c = two_orientations();
  cvc::grid_tile_stepper s(c);
  for (int i = 0; i < 20; ++i)
    s.step(16);
  s.cancel();
  EXPECT_TRUE(s.step(16));
  EXPECT_TRUE(s.done());
  EXPECT_TRUE(s.take().surfaces.empty());

  std::unique_ptr<cvc::result_job<cvc::grid_tile_result>> job = cvc::tile_grid_contours_job(c);
  while (!job->done())
    job->step(256);
  ASSERT_TRUE(job->has_result()) << job->status();
  EXPECT_EQ(flat(job->take_result()), flat(cvc::tile_grid_contours(c)));
  std::unique_ptr<cvc::result_job<cvc::grid_tile_result>> stopped = cvc::tile_grid_contours_job(c);
  stopped->step(16);
  stopped->cancel();
  stopped->step(16);
  EXPECT_TRUE(stopped->done());
  EXPECT_FALSE(stopped->has_result());
}

TEST(GridContourTiling, TheSlowestStepIsRecorded) {
  SKIP_WITHOUT_PLANAR();
  const grid_contours c = two_orientations();
  cvc::grid_tile_stepper s(c);
  double slowest = 0;
  while (true) {
    const auto t0 = std::chrono::steady_clock::now();
    const bool done = s.step(4096);
    slowest = std::max(
        slowest,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    if (done)
      break;
  }
  std::cout << "[ T7 ] grid_tile_stepper slowest step at budget 4096: " << slowest << " ms\n";
  EXPECT_LT(slowest, 2000.0);
}

// ---- T6: no singletons --------------------------------------------------------------------------

TEST(GridContourNoSingletons, TwoThreadsGetWhatEachGetsAlone) {
  SKIP_WITHOUT_PLANAR();
  const grid_contours a = tube(slice_axis::z, 5, 16), b = tube(slice_axis::x, 7, 12, 0.5, 1.0);
  const std::vector<double> wa = flat(cvc::tile_grid_contours(a)),
                            wb = flat(cvc::tile_grid_contours(b));
  std::vector<double> ga, gb;
  std::thread ta([&] { ga = flat(cvc::tile_grid_contours(a)); });
  std::thread tb([&] { gb = flat(cvc::tile_grid_contours(b)); });
  ta.join();
  tb.join();
  EXPECT_EQ(ga, wa);
  EXPECT_EQ(gb, wb);
}

TEST(GridContourNoSingletons, TwoEditorsShareNothing) {
  grid_contour_editor e1, e2;
  e1.set_frame(frame64());
  e2.set_frame(frame64());
  e1.add_object(object("a"));
  e2.add_object(object("b"));
  e1.add_point(0, slice_axis::z, 1, 1, 1);
  EXPECT_TRUE(e2.contours().objects[0].curves.empty());
  EXPECT_EQ(e2.undo_depth(), 1u);
  EXPECT_FALSE(e2.active());
}

} // namespace
