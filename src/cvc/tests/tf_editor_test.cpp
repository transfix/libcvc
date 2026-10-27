/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Unit tests for cvc::tf's GL-free editor (cvc/tf/editor.h): the canvas / strip mappings, hit
// testing, every model edit, the drag gesture, describe() and the raster preview.

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/tf/editor.h>
#include <cvc/tf/overlays.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::tf;

namespace {

// A canvas whose extents are exact: x = 10 + 200 t, y = 30 + 100 (1 - a), bottom row 130; the
// strip above it maps x = 10 + 200 t over rows 5..24.
layout make_layout() {
  layout l;
  l.canvas_x = 10.f;
  l.canvas_y = 30.f;
  l.canvas_w = 201.f;
  l.canvas_h = 101.f;
  l.strip_x = 10.f;
  l.strip_y = 5.f;
  l.strip_w = 201.f;
  l.strip_h = 20.f;
  return l;
}

color_node cn(double at, float r, float g, float b) {
  color_node n;
  n.at = at;
  n.r = r;
  n.g = g;
  n.b = b;
  return n;
}

alpha_node an(double at, float a) {
  alpha_node n;
  n.at = at;
  n.a = a;
  return n;
}

isocontour ic(const std::string &id, double at) {
  isocontour i;
  i.id = id;
  i.at = at;
  return i;
}

// black -> red -> green; opacity 0 / .75 / .25 / 1 (ColorTable2's default opacity ramp). Colour
// bars at x = 10, 110, 210; alpha nodes at (10,130), (60,55), (160,105), (210,30).
model make_model() {
  model m;
  m.color = {cn(0.0, 0, 0, 0), cn(0.5, 1, 0, 0), cn(1.0, 0, 1, 0)};
  m.alpha = {an(0.0, 0.f), an(0.25, 0.75f), an(0.75, 0.25f), an(1.0, 1.f)};
  return m;
}

// A view showing only the ramp (for pixel checks of one layer at a time).
view bare_view() {
  view v;
  v.show_opacity = false;
  v.show_isocontours = false;
  v.show_histogram = false;
  v.show_contour_spectrum = false;
  v.show_contour_tree = false;
  return v;
}

struct px {
  int r, g, b, a;
};
px pixel(const std::vector<std::uint8_t> &img, int w, int x, int y) {
  const std::size_t i = (static_cast<std::size_t>(y) * w + x) * 4;
  return {img[i], img[i + 1], img[i + 2], img[i + 3]};
}
bool is(const px &p, int r, int g, int b) { return p.r == r && p.g == g && p.b == b && p.a == 255; }

} // namespace

// --- view ---------------------------------------------------------------------------------

TEST(TfEditorView, EqualityCoversEveryField) {
  const view a;
  view b;
  EXPECT_TRUE(a == b);
  b.zoom_lo = 0.1;
  EXPECT_TRUE(a != b);
  b = a;
  b.zoom_hi = 0.9;
  EXPECT_NE(a, b);
  b = a;
  b.show_color = false;
  EXPECT_NE(a, b);
  b = a;
  b.show_opacity = false;
  EXPECT_NE(a, b);
  b = a;
  b.show_isocontours = false;
  EXPECT_NE(a, b);
  b = a;
  b.show_histogram = false;
  EXPECT_NE(a, b);
  b = a;
  b.show_contour_spectrum = true;
  EXPECT_NE(a, b);
  b = a;
  b.show_contour_tree = true;
  EXPECT_NE(a, b);
  b = a;
  b.log_histogram = true;
  EXPECT_NE(a, b);
  b = a;
  b.show_info = true;
  EXPECT_NE(a, b);
}

TEST(TfEditorView, ClampZoomOrdersClampsAndKeepsMinimumSpan) {
  view v;
  v.zoom_lo = 0.8;
  v.zoom_hi = 0.2;
  clamp_zoom(v);
  EXPECT_DOUBLE_EQ(v.zoom_lo, 0.2);
  EXPECT_DOUBLE_EQ(v.zoom_hi, 0.8);

  v.zoom_lo = -1.0;
  v.zoom_hi = 3.0;
  clamp_zoom(v);
  EXPECT_DOUBLE_EQ(v.zoom_lo, 0.0);
  EXPECT_DOUBLE_EQ(v.zoom_hi, 1.0);

  v.zoom_lo = v.zoom_hi = 0.5; // widened about its centre
  clamp_zoom(v);
  EXPECT_NEAR(v.zoom_hi - v.zoom_lo, kMinZoomSpan, 1e-12);
  EXPECT_NEAR(0.5 * (v.zoom_lo + v.zoom_hi), 0.5, 1e-12);

  v.zoom_lo = v.zoom_hi = 0.0; // ... but kept inside [0,1]
  clamp_zoom(v);
  EXPECT_DOUBLE_EQ(v.zoom_lo, 0.0);
  EXPECT_DOUBLE_EQ(v.zoom_hi, kMinZoomSpan);
  v.zoom_lo = v.zoom_hi = 1.0;
  clamp_zoom(v);
  EXPECT_DOUBLE_EQ(v.zoom_lo, 1.0 - kMinZoomSpan);
  EXPECT_DOUBLE_EQ(v.zoom_hi, 1.0);

  v.zoom_lo = std::nan("");
  v.zoom_hi = std::nan("");
  clamp_zoom(v);
  EXPECT_DOUBLE_EQ(v.zoom_lo, 0.0);
  EXPECT_DOUBLE_EQ(v.zoom_hi, 1.0);

  // Idempotent, including on a window built at exactly the minimum span.
  v.zoom_lo = 0.3;
  v.zoom_hi = 0.3 + kMinZoomSpan;
  clamp_zoom(v);
  const view once = v;
  clamp_zoom(v);
  EXPECT_EQ(v, once);
}

// --- mapping --------------------------------------------------------------------------------

TEST(TfEditorMapping, CanvasAndStripWithoutZoom) {
  const layout l = make_layout();
  const view v;
  EXPECT_FLOAT_EQ(x_of(l, v, 0.0), 10.f);
  EXPECT_FLOAT_EQ(x_of(l, v, 0.5), 110.f);
  EXPECT_FLOAT_EQ(x_of(l, v, 1.0), 210.f);
  EXPECT_NEAR(t_of(l, v, 60.f), 0.25, 1e-9);
  EXPECT_NEAR(t_of(l, v, x_of(l, v, 0.37)), 0.37, 1e-6);
  EXPECT_FLOAT_EQ(y_of(l, 1.f), 30.f);
  EXPECT_FLOAT_EQ(y_of(l, 0.f), 130.f);
  EXPECT_NEAR(a_of(l, 55.f), 0.75f, 1e-6f);
  EXPECT_NEAR(a_of(l, y_of(l, 0.42f)), 0.42f, 1e-6f);
  EXPECT_FLOAT_EQ(strip_x_of(l, 0.25), 60.f);
  EXPECT_NEAR(strip_t_of(l, 160.f), 0.75, 1e-9);
}

TEST(TfEditorMapping, CanvasFollowsTheZoomWindowAndTheStripDoesNot) {
  const layout l = make_layout();
  view v;
  v.zoom_lo = 0.25;
  v.zoom_hi = 0.75;
  EXPECT_FLOAT_EQ(x_of(l, v, 0.25), 10.f);
  EXPECT_FLOAT_EQ(x_of(l, v, 0.5), 110.f);
  EXPECT_FLOAT_EQ(x_of(l, v, 0.75), 210.f);
  EXPECT_NEAR(t_of(l, v, 210.f), 0.75, 1e-9);
  EXPECT_NEAR(t_of(l, v, 60.f), 0.375, 1e-9);
  EXPECT_LT(x_of(l, v, 0.0), 10.f); // outside the window maps off the canvas
  EXPECT_FLOAT_EQ(strip_x_of(l, 0.5), 110.f);
}

// --- hit testing ---------------------------------------------------------------------------

TEST(TfEditorHitTest, FindsEachKindOfHandle) {
  const layout l = make_layout();
  const view v;
  model m = make_model();
  m.isocontours = {ic("bone", 0.3)}; // a bar at x = 70

  handle h = hit_test(m, v, l, 60.f, 55.f);
  EXPECT_EQ(h.kind, part::alpha);
  EXPECT_EQ(h.index, 1);
  h = hit_test(m, v, l, 64.f, 51.f); // inside the 10x10 square
  EXPECT_EQ(h.kind, part::alpha);
  h = hit_test(m, v, l, 66.f, 55.f); // outside it
  EXPECT_FALSE(h.valid());

  h = hit_test(m, v, l, 70.f, 90.f);
  EXPECT_EQ(h.kind, part::isocontour);
  EXPECT_EQ(h.index, 0);
  EXPECT_EQ(h.id, "bone");

  h = hit_test(m, v, l, 110.f, 70.f);
  EXPECT_EQ(h.kind, part::color);
  EXPECT_EQ(h.index, 1);

  EXPECT_FALSE(hit_test(m, v, l, 35.f, 100.f).valid());  // empty canvas
  EXPECT_FALSE(hit_test(m, v, l, 400.f, 400.f).valid()); // nowhere near
}

TEST(TfEditorHitTest, PriorityIsAlphaThenIsocontourThenColour) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  m.alpha = {an(0.0, 0.f), an(0.5, 0.5f), an(1.0, 1.f)}; // node 1 at (110, 80)
  m.isocontours = {ic("iso1", 0.5)};                     // on the colour bar at x = 110

  EXPECT_EQ(hit_test(m, v, l, 110.f, 80.f).kind, part::alpha);
  EXPECT_EQ(hit_test(m, v, l, 112.f, 40.f).kind, part::isocontour);

  v.show_opacity = false; // hidden nodes are not pickable
  EXPECT_EQ(hit_test(m, v, l, 110.f, 80.f).kind, part::isocontour);
  v.show_isocontours = false;
  const handle c = hit_test(m, v, l, 110.f, 80.f);
  EXPECT_EQ(c.kind, part::color);
  EXPECT_EQ(c.index, 1);
  v.show_color = false;
  EXPECT_FALSE(hit_test(m, v, l, 110.f, 80.f).valid());
}

TEST(TfEditorHitTest, BarTolerancesAndBottomHandle) {
  const layout l = make_layout();
  const view v;
  const model m = make_model(); // colour bar 1 at x = 110; handle rows 120..130
  EXPECT_EQ(hit_test(m, v, l, 113.f, 70.f).kind, part::color);  // within kBarGrab
  EXPECT_FALSE(hit_test(m, v, l, 114.f, 70.f).valid());         // beyond it, mid-height
  EXPECT_EQ(hit_test(m, v, l, 114.f, 125.f).kind, part::color); // on the 10x10 handle
  EXPECT_EQ(hit_test(m, v, l, 115.f, 121.f).kind, part::color); // its corner
  EXPECT_FALSE(hit_test(m, v, l, 116.f, 125.f).valid());        // beside it
  EXPECT_FALSE(hit_test(m, v, l, 114.f, 118.f).valid());        // above it
  EXPECT_EQ(hit_test(m, v, l, 110.f, 30.f).kind, part::color);  // the bar's top row
}

TEST(TfEditorHitTest, TopmostOfOverlappingHandlesWins) {
  const layout l = make_layout();
  const view v;
  model m = make_model();
  m.alpha = {an(0.0, 0.f), an(0.5, 0.5f), an(0.52, 0.5f), an(1.0, 1.f)}; // x = 110 and 114
  const handle h = hit_test(m, v, l, 112.f, 80.f);
  EXPECT_EQ(h.kind, part::alpha);
  EXPECT_EQ(h.index, 2);
  EXPECT_EQ(hit_test(m, v, l, 106.f, 80.f).index, 1); // only node 1 covers it
}

TEST(TfEditorHitTest, EdgeHandlesAreGrabbableWhole) {
  const layout l = make_layout();
  const view v;
  const model m = make_model();
  const handle h = hit_test(m, v, l, 6.f, 133.f); // left of / below the canvas, on alpha node 0
  EXPECT_EQ(h.kind, part::alpha);
  EXPECT_EQ(h.index, 0);
  EXPECT_EQ(hit_test(m, v, l, 214.f, 27.f).index, 3); // alpha node 3, top-right corner
}

// A node dragged against a pinned endpoint stops kMinNodeGap (sub-pixel) short of it. It must stay
// grabbable, selectable and deletable, so an endpoint yields to an interior node of its list under
// the same point, at either end.
TEST(TfEditorHitTest, ColourBarOnAnEndpointStaysGrabbable) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  // Bars at x = 10, 110, 160, 210 (the default ramp's layout).
  m.color = {cn(0.0, 0, 0, 0), cn(0.5, 1, 0, 0), cn(0.75, 1, 1, 0), cn(1.0, 0, 1, 0)};
  gesture g;
  ASSERT_EQ(g.press(m, v, l, 160.f, 70.f).index, 2); // the yellow bar ...
  EXPECT_TRUE(g.drag(m, v, l, 500.f, 70.f));         // ... dragged past the right edge
  g.release();
  ASSERT_GT(m.color[2].at, 1.0 - 2 * kMinNodeGap);

  handle h = hit_test(m, v, l, 210.f, 70.f); // on the bar
  EXPECT_EQ(h.kind, part::color);
  EXPECT_EQ(h.index, 2);
  EXPECT_EQ(hit_test(m, v, l, 210.f, 125.f).index, 2); // on its handle
  // It drags back off the endpoint, which is then pickable again.
  ASSERT_EQ(g.press(m, v, l, 210.f, 70.f).index, 2);
  EXPECT_TRUE(g.drag(m, v, l, 160.f, 70.f));
  g.release();
  EXPECT_NEAR(m.color[2].at, 0.75, 1e-5);
  EXPECT_DOUBLE_EQ(m.color[3].at, 1.0);
  EXPECT_EQ(hit_test(m, v, l, 210.f, 70.f).index, 3);

  // It deletes.
  EXPECT_TRUE(move_color(m, 2, 1.0));
  h = hit_test(m, v, l, 210.f, 70.f);
  EXPECT_TRUE(remove(m, h));
  ASSERT_EQ(m.color.size(), 3u);
  EXPECT_DOUBLE_EQ(m.color[2].at, 1.0);

  // The left end.
  EXPECT_TRUE(move_color(m, 1, -1.0));
  h = hit_test(m, v, l, 10.f, 70.f);
  EXPECT_EQ(h.kind, part::color);
  EXPECT_EQ(h.index, 1);
  EXPECT_TRUE(remove(m, h));
  EXPECT_EQ(m.color.size(), 2u);
}

TEST(TfEditorHitTest, AlphaNodeOnAnEndpointStaysGrabbable) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  gesture g;
  // Node 2 (160, 105) dragged into the top-right corner, onto endpoint 3 (210, 30).
  ASSERT_EQ(g.press(m, v, l, 160.f, 105.f).index, 2);
  EXPECT_TRUE(g.drag(m, v, l, 500.f, -50.f));
  g.release();
  ASSERT_GT(m.alpha[2].at, 1.0 - 2 * kMinNodeGap);
  ASSERT_FLOAT_EQ(m.alpha[2].a, 1.f);

  handle h = hit_test(m, v, l, 210.f, 30.f);
  EXPECT_EQ(h.kind, part::alpha);
  EXPECT_EQ(h.index, 2);
  ASSERT_EQ(g.press(m, v, l, 210.f, 30.f).index, 2);
  EXPECT_TRUE(g.drag(m, v, l, 160.f, 105.f));
  g.release();
  EXPECT_NEAR(m.alpha[2].at, 0.75, 1e-5);
  EXPECT_NEAR(m.alpha[2].a, 0.25f, 1e-5f);
  EXPECT_DOUBLE_EQ(m.alpha[3].at, 1.0);
  EXPECT_FLOAT_EQ(m.alpha[3].a, 1.f);
  EXPECT_EQ(hit_test(m, v, l, 210.f, 30.f).index, 3);

  EXPECT_TRUE(move_alpha(m, 2, 1.0, 1.f));
  h = hit_test(m, v, l, 210.f, 30.f);
  EXPECT_EQ(h.index, 2);
  EXPECT_TRUE(remove(m, h));
  ASSERT_EQ(m.alpha.size(), 3u);

  // The left end: node 1 (60, 55) onto endpoint 0 (10, 130).
  EXPECT_TRUE(move_alpha(m, 1, 0.0, 0.f));
  h = hit_test(m, v, l, 10.f, 130.f);
  EXPECT_EQ(h.kind, part::alpha);
  EXPECT_EQ(h.index, 1);
  EXPECT_TRUE(remove(m, h));
  EXPECT_EQ(m.alpha.size(), 2u);
}

TEST(TfEditorHitTest, ZoomBarsLiveInTheStrip) {
  const layout l = make_layout();
  view v;
  v.zoom_lo = 0.25; // strip x = 60
  v.zoom_hi = 0.75; // strip x = 160
  const model m = make_model();
  EXPECT_EQ(hit_test(m, v, l, 60.f, 15.f).kind, part::zoom_lo);
  EXPECT_EQ(hit_test(m, v, l, 162.f, 8.f).kind, part::zoom_hi);
  // The strip never picks canvas items, and the canvas never picks the range bars.
  EXPECT_FALSE(hit_test(m, v, l, 110.f, 15.f).valid());
  EXPECT_NE(hit_test(m, v, l, 60.f, 80.f).kind, part::zoom_lo);

  // Bars a few px apart: the nearer one wins.
  v.zoom_lo = 0.5;  // x = 110
  v.zoom_hi = 0.51; // x = 112
  EXPECT_EQ(hit_test(m, v, l, 110.5f, 15.f).kind, part::zoom_lo);
  EXPECT_EQ(hit_test(m, v, l, 111.6f, 15.f).kind, part::zoom_hi);

  layout no_strip = l;
  no_strip.strip_h = 0.f;
  EXPECT_FALSE(hit_test(m, v, no_strip, 110.f, 15.f).valid());
}

// --- edits -------------------------------------------------------------------------------------

TEST(TfEditorEdits, AddAlphaInsertsSortedAndRefusesCrowding) {
  model m = make_model();
  EXPECT_TRUE(add_alpha(m, 0.5, 0.4f));
  ASSERT_EQ(m.alpha.size(), 5u);
  EXPECT_DOUBLE_EQ(m.alpha[2].at, 0.5);
  EXPECT_FLOAT_EQ(m.alpha[2].a, 0.4f);
  for (std::size_t i = 1; i < m.alpha.size(); ++i)
    EXPECT_LT(m.alpha[i - 1].at, m.alpha[i].at);

  EXPECT_FALSE(add_alpha(m, 0.0, 0.5f));                     // on the endpoint
  EXPECT_FALSE(add_alpha(m, 0.5 + 0.5 * kMinNodeGap, 0.1f)); // on a node
  EXPECT_FALSE(add_alpha(m, 7.0, 0.2f));                     // clamps onto the 1 endpoint
  EXPECT_FALSE(add_alpha(m, std::nan(""), 0.2f));
  EXPECT_FALSE(add_alpha(m, 0.6, std::nanf("")));
  EXPECT_EQ(m.alpha.size(), 5u);

  EXPECT_TRUE(add_alpha(m, 0.6, 3.f)); // opacity clamped
  EXPECT_FLOAT_EQ(m.alpha[3].a, 1.f);
}

TEST(TfEditorEdits, AddColorInterpolatesTheRamp) {
  model m = make_model();
  EXPECT_TRUE(add_color(m, 0.25)); // half way from black to red
  ASSERT_EQ(m.color.size(), 4u);
  EXPECT_DOUBLE_EQ(m.color[1].at, 0.25);
  EXPECT_NEAR(m.color[1].r, 0.5f, 1e-6f);
  EXPECT_NEAR(m.color[1].g, 0.f, 1e-6f);
  EXPECT_NEAR(m.color[1].b, 0.f, 1e-6f);
  EXPECT_TRUE(add_color(m, 0.75)); // half way from red to green
  EXPECT_NEAR(m.color[3].r, 0.5f, 1e-6f);
  EXPECT_NEAR(m.color[3].g, 0.5f, 1e-6f);
  EXPECT_FALSE(add_color(m, 0.0));
  EXPECT_FALSE(add_color(m, 1.0));
  EXPECT_FALSE(add_color(m, 0.25));
  EXPECT_EQ(m.color.size(), 5u);
}

TEST(TfEditorEdits, AddIsocontourGivesFreshIdsInOrder) {
  model m = make_model();
  const std::string a = add_isocontour(m, 0.6);
  const std::string b = add_isocontour(m, 0.3);
  EXPECT_FALSE(a.empty());
  EXPECT_FALSE(b.empty());
  EXPECT_NE(a, b);
  ASSERT_EQ(m.isocontours.size(), 2u);
  EXPECT_EQ(m.isocontours[0].id, b);
  EXPECT_EQ(m.isocontours[1].id, a);
  EXPECT_EQ(m.find_isocontour(a), 1);
  const std::string c = add_isocontour(m, 4.0); // clamped
  EXPECT_DOUBLE_EQ(m.isocontours[static_cast<std::size_t>(m.find_isocontour(c))].at, 1.0);
  const std::string d = add_isocontour(m, 0.6); // a shared position is allowed
  EXPECT_FALSE(d.empty());
  EXPECT_TRUE(add_isocontour(m, std::nan("")).empty());
  EXPECT_EQ(m.isocontours.size(), 4u);
}

TEST(TfEditorEdits, RemoveProtectsEndpoints) {
  model m = make_model();
  handle h;
  h.kind = part::color;
  h.index = 0;
  EXPECT_FALSE(remove(m, h));
  h.index = 2;
  EXPECT_FALSE(remove(m, h));
  h.index = 7;
  EXPECT_FALSE(remove(m, h));
  h.index = 1;
  EXPECT_TRUE(remove(m, h));
  EXPECT_EQ(m.color.size(), 2u);
  EXPECT_FALSE(remove(m, h)); // now both are endpoints

  h.kind = part::alpha;
  h.index = 0;
  EXPECT_FALSE(remove(m, h));
  h.index = 3;
  EXPECT_FALSE(remove(m, h));
  h.index = 2;
  EXPECT_TRUE(remove(m, h));
  ASSERT_EQ(m.alpha.size(), 3u);
  EXPECT_DOUBLE_EQ(m.alpha[1].at, 0.25);

  m.isocontours = {ic("a", 0.2), ic("b", 0.4)};
  h.kind = part::isocontour;
  h.index = 0; // stale: the id wins
  h.id = "b";
  EXPECT_TRUE(remove(m, h));
  ASSERT_EQ(m.isocontours.size(), 1u);
  EXPECT_EQ(m.isocontours[0].id, "a");
  h.id = "zzz";
  EXPECT_FALSE(remove(m, h));
  h.id.clear(); // no id: by index
  EXPECT_TRUE(remove(m, h));
  EXPECT_TRUE(m.isocontours.empty());

  h.kind = part::zoom_lo;
  EXPECT_FALSE(remove(m, h));
  EXPECT_FALSE(remove(m, handle()));
}

TEST(TfEditorEdits, RecolourNodesAndIsocontours) {
  model m = make_model();
  const float rgb[3] = {2.f, -1.f, 0.5f};
  EXPECT_TRUE(set_color(m, 1, rgb));
  EXPECT_FLOAT_EQ(m.color[1].r, 1.f);
  EXPECT_FLOAT_EQ(m.color[1].g, 0.f);
  EXPECT_FLOAT_EQ(m.color[1].b, 0.5f);
  EXPECT_FALSE(set_color(m, 1, rgb)); // unchanged
  EXPECT_TRUE(set_color(m, 0, rgb));  // an endpoint's colour does edit
  EXPECT_FALSE(set_color(m, 3, rgb));
  EXPECT_FALSE(set_color(m, -1, rgb));

  m.isocontours = {ic("bone", 0.25)};
  const float white[3] = {0.2f, 0.4f, 0.6f};
  EXPECT_TRUE(set_isocontour_rgb(m, "bone", white));
  EXPECT_TRUE(m.isocontours[0].has_rgb);
  EXPECT_FLOAT_EQ(m.isocontours[0].g, 0.4f);
  EXPECT_FALSE(set_isocontour_rgb(m, "bone", white));
  EXPECT_FALSE(set_isocontour_rgb(m, "skin", white));
  EXPECT_TRUE(clear_isocontour_rgb(m, "bone"));
  EXPECT_FALSE(m.isocontours[0].has_rgb);
  EXPECT_FALSE(clear_isocontour_rgb(m, "bone"));
  float ramp[3], drawn[3];
  m.color_at(0.25, ramp);
  m.isocontour_rgb(m.isocontours[0], drawn); // back to the ramp colour
  EXPECT_NEAR(drawn[0], ramp[0], 1e-6f);
  EXPECT_NEAR(drawn[1], ramp[1], 1e-6f);
  EXPECT_NEAR(drawn[2], ramp[2], 1e-6f);
}

TEST(TfEditorEdits, MoveColorStaysBetweenNeighboursAndPinsEndpoints) {
  model m = make_model();
  m.color.insert(m.color.begin() + 1, cn(0.3, 0, 0, 1)); // 0, .3, .5, 1
  EXPECT_TRUE(move_color(m, 2, 0.8));
  EXPECT_DOUBLE_EQ(m.color[2].at, 0.8);
  EXPECT_TRUE(move_color(m, 1, 0.95)); // would pass node 2: stops just short
  EXPECT_LT(m.color[1].at, m.color[2].at);
  EXPECT_NEAR(m.color[1].at, 0.8, 2 * kMinNodeGap);
  EXPECT_FALSE(move_color(m, 1, 0.99)); // already against it
  EXPECT_TRUE(move_color(m, 2, 5.0));   // clamped short of the 1 endpoint
  EXPECT_LT(m.color[2].at, 1.0);
  EXPECT_GT(m.color[2].at, 0.999);
  EXPECT_FLOAT_EQ(m.color[1].b, 1.f); // identity kept: moving never recolours

  EXPECT_FALSE(move_color(m, 0, 0.3)); // endpoints pinned
  EXPECT_FALSE(move_color(m, 3, 0.3));
  EXPECT_DOUBLE_EQ(m.color[0].at, 0.0);
  EXPECT_DOUBLE_EQ(m.color[3].at, 1.0);
  EXPECT_FALSE(move_color(m, 1, std::nan("")));
  EXPECT_FALSE(move_color(m, 9, 0.5));
}

TEST(TfEditorEdits, MoveAlphaClampsAndEndpointsKeepTheirPosition) {
  model m = make_model();
  EXPECT_TRUE(move_alpha(m, 0, 0.5, 0.3f)); // endpoint: opacity only
  EXPECT_DOUBLE_EQ(m.alpha[0].at, 0.0);
  EXPECT_FLOAT_EQ(m.alpha[0].a, 0.3f);
  EXPECT_FALSE(move_alpha(m, 3, 0.2, 5.f)); // already at opacity 1, position pinned
  EXPECT_TRUE(move_alpha(m, 1, 0.9, 0.6f)); // interior: clamped below node 2 (0.75)
  EXPECT_LT(m.alpha[1].at, 0.75);
  EXPECT_NEAR(m.alpha[1].at, 0.75, 2 * kMinNodeGap);
  EXPECT_FLOAT_EQ(m.alpha[1].a, 0.6f);
  EXPECT_TRUE(move_alpha(m, 2, -3.0, -1.f)); // clamped above node 1, opacity to 0
  EXPECT_GT(m.alpha[2].at, m.alpha[1].at);
  EXPECT_FLOAT_EQ(m.alpha[2].a, 0.f);
  EXPECT_FALSE(move_alpha(m, 2, m.alpha[2].at, 0.f));
  EXPECT_FALSE(move_alpha(m, 4, 0.5, 0.5f));
  EXPECT_FALSE(move_alpha(m, 1, 0.5, std::nanf("")));
  for (std::size_t i = 1; i < m.alpha.size(); ++i)
    EXPECT_LT(m.alpha[i - 1].at, m.alpha[i].at);
}

TEST(TfEditorEdits, MoveIsocontourResortsAndKeepsIdentity) {
  model m = make_model();
  m.isocontours = {ic("a", 0.2), ic("b", 0.4)};
  m.isocontours[0].has_rgb = true;
  m.isocontours[0].r = 0.25f;
  EXPECT_TRUE(move_isocontour(m, "a", 0.6));
  ASSERT_EQ(m.isocontours.size(), 2u);
  EXPECT_EQ(m.isocontours[0].id, "b");
  EXPECT_EQ(m.isocontours[1].id, "a");
  EXPECT_DOUBLE_EQ(m.isocontours[1].at, 0.6);
  EXPECT_TRUE(m.isocontours[1].has_rgb); // the override travels with it
  EXPECT_FLOAT_EQ(m.isocontours[1].r, 0.25f);
  EXPECT_TRUE(move_isocontour(m, "a", -2.0));
  EXPECT_EQ(m.isocontours[0].id, "a");
  EXPECT_DOUBLE_EQ(m.isocontours[0].at, 0.0);
  EXPECT_FALSE(move_isocontour(m, "a", 0.0));
  EXPECT_FALSE(move_isocontour(m, "nope", 0.5));
  EXPECT_FALSE(move_isocontour(m, "b", std::nan("")));
}

// --- the gesture ----------------------------------------------------------------------------

TEST(TfEditorGesture, DragColourBar) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  gesture g;
  const handle h = g.press(m, v, l, 110.f, 70.f);
  EXPECT_EQ(h.kind, part::color);
  EXPECT_TRUE(g.active());
  EXPECT_FALSE(g.model_changed());
  EXPECT_TRUE(g.drag(m, v, l, 150.f, 90.f)); // y is irrelevant for a bar
  EXPECT_NEAR(m.color[1].at, 0.7, 1e-6);
  EXPECT_TRUE(g.model_changed());
  EXPECT_TRUE(g.drag(m, v, l, 500.f, 70.f)); // past the end: stops before the endpoint
  EXPECT_LT(m.color[1].at, 1.0);
  EXPECT_GT(m.color[1].at, 0.999);
  EXPECT_FALSE(g.drag(m, v, l, 600.f, 70.f));
  EXPECT_FALSE(g.view_changed());
  g.release();
  EXPECT_FALSE(g.active());
  EXPECT_TRUE(g.model_changed()); // still readable for a release-time commit
  EXPECT_FALSE(g.drag(m, v, l, 110.f, 70.f));

  // A press off the bar's centre keeps that offset: no motion, no edit.
  m = make_model();
  g.press(m, v, l, 112.f, 70.f);
  EXPECT_FALSE(g.model_changed()); // reset by press()
  EXPECT_FALSE(g.drag(m, v, l, 112.f, 70.f));
  EXPECT_DOUBLE_EQ(m.color[1].at, 0.5);
  EXPECT_TRUE(g.drag(m, v, l, 152.f, 70.f));
  EXPECT_NEAR(m.color[1].at, 0.7, 1e-6);
}

TEST(TfEditorGesture, DragEndpointColourBarMovesNothing) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  gesture g;
  EXPECT_EQ(g.press(m, v, l, 210.f, 70.f).index, 2);
  EXPECT_FALSE(g.drag(m, v, l, 150.f, 70.f));
  EXPECT_DOUBLE_EQ(m.color[2].at, 1.0);
  EXPECT_FALSE(g.model_changed());
}

TEST(TfEditorGesture, DragAlphaNodes) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  gesture g;
  EXPECT_EQ(g.press(m, v, l, 60.f, 55.f).index, 1);
  EXPECT_TRUE(g.drag(m, v, l, 90.f, 40.f));
  EXPECT_NEAR(m.alpha[1].at, 0.4, 1e-6);
  EXPECT_NEAR(m.alpha[1].a, 0.9f, 1e-5f);
  EXPECT_TRUE(g.drag(m, v, l, 200.f, -50.f)); // past node 2 and above the canvas
  EXPECT_LT(m.alpha[1].at, m.alpha[2].at);
  EXPECT_FLOAT_EQ(m.alpha[1].a, 1.f);
  g.release();

  // The opacity of an endpoint drags; its position does not.
  EXPECT_EQ(g.press(m, v, l, 210.f, 30.f).index, 3);
  EXPECT_TRUE(g.drag(m, v, l, 150.f, 80.f));
  EXPECT_DOUBLE_EQ(m.alpha[3].at, 1.0);
  EXPECT_NEAR(m.alpha[3].a, 0.5f, 1e-5f);
  g.release();
}

TEST(TfEditorGesture, DragIsocontourAcrossAnotherKeepsTheGrab) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  m.isocontours = {ic("a", 0.3), ic("b", 0.6)}; // bars at x = 70 and 130
  gesture g;
  const handle h = g.press(m, v, l, 70.f, 70.f);
  EXPECT_EQ(h.kind, part::isocontour);
  EXPECT_EQ(h.id, "a");
  EXPECT_TRUE(g.drag(m, v, l, 170.f, 70.f));
  EXPECT_EQ(m.isocontours[1].id, "a");
  EXPECT_NEAR(m.isocontours[1].at, 0.8, 1e-6);
  EXPECT_EQ(g.grabbed().id, "a");
  EXPECT_EQ(g.grabbed().index, 1);
  EXPECT_TRUE(g.drag(m, v, l, 30.f, 70.f));
  EXPECT_EQ(m.isocontours[0].id, "a");
  EXPECT_EQ(g.grabbed().index, 0);
  EXPECT_NEAR(m.isocontours[0].at, 0.1, 1e-6);
  EXPECT_DOUBLE_EQ(m.isocontours[1].at, 0.6); // the other one never moved
}

TEST(TfEditorGesture, ZoomBarsChangeOnlyTheView) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  const model before = m;
  gesture g;
  EXPECT_EQ(g.press(m, v, l, 10.f, 15.f).kind, part::zoom_lo);
  EXPECT_FALSE(g.drag(m, v, l, 110.f, 15.f)); // the model did not change ...
  EXPECT_DOUBLE_EQ(v.zoom_lo, 0.5);           // ... the view did
  EXPECT_TRUE(g.view_changed());
  EXPECT_FALSE(g.model_changed());
  g.drag(m, v, l, 400.f, 15.f); // cannot pass the other bar
  EXPECT_NEAR(v.zoom_lo, 1.0 - kMinZoomSpan, 1e-12);
  EXPECT_DOUBLE_EQ(v.zoom_hi, 1.0);
  g.release();
  EXPECT_TRUE(g.view_changed());
  EXPECT_TRUE(m == before);

  view w;
  EXPECT_EQ(g.press(m, w, l, 210.f, 15.f).kind, part::zoom_hi);
  EXPECT_FALSE(g.view_changed());
  g.drag(m, w, l, 110.f, 15.f);
  EXPECT_DOUBLE_EQ(w.zoom_hi, 0.5);
  g.drag(m, w, l, -100.f, 15.f);
  EXPECT_DOUBLE_EQ(w.zoom_lo, 0.0);
  EXPECT_NEAR(w.zoom_hi, kMinZoomSpan, 1e-12);
  g.release();
}

TEST(TfEditorGesture, EmptySpaceAndCancel) {
  const layout l = make_layout();
  view v;
  model m = make_model();
  gesture g;
  EXPECT_FALSE(g.press(m, v, l, 35.f, 100.f).valid());
  EXPECT_FALSE(g.active());
  EXPECT_FALSE(g.drag(m, v, l, 80.f, 80.f));
  EXPECT_FALSE(g.model_changed());

  g.press(m, v, l, 110.f, 70.f);
  EXPECT_TRUE(g.drag(m, v, l, 130.f, 70.f));
  g.cancel(); // abandoned: nothing left to commit
  EXPECT_FALSE(g.active());
  EXPECT_FALSE(g.model_changed());
  EXPECT_FALSE(g.view_changed());
}

// --- describe -------------------------------------------------------------------------------

TEST(TfEditorDescribe, WithoutOverlays) {
  const model m = make_model();
  const isocontour iso = ic("bone", 0.25);
  isocontour_info info = describe(m, iso, 100.0, 300.0, nullptr);
  EXPECT_EQ(info.id, "bone");
  EXPECT_DOUBLE_EQ(info.at, 0.25);
  EXPECT_DOUBLE_EQ(info.isovalue, 150.0); // data units
  EXPECT_TRUE(std::isnan(info.area));
  EXPECT_TRUE(std::isnan(info.min_volume));
  EXPECT_TRUE(std::isnan(info.max_volume));
  EXPECT_TRUE(std::isnan(info.gradient));
  EXPECT_EQ(info.components, -1);

  const overlays empty;
  info = describe(m, iso, 100.0, 300.0, &empty);
  EXPECT_TRUE(std::isnan(info.area));
  EXPECT_EQ(info.components, -1);
}

TEST(TfEditorDescribe, InterpolatesTheSpectrumAndCountsComponents) {
  const model m = make_model();
  overlays ov;
  ov.has_range = true;
  ov.range_lo = 100.0;
  ov.range_hi = 300.0;
  for (int i = 0; i <= 10; ++i) { // 11 samples, one per 20 data units
    ov.spectrum.area.push_back(i);
    ov.spectrum.min_volume.push_back(10.0 * i);
    ov.spectrum.max_volume.push_back(100.0 - 10.0 * i);
    ov.spectrum.gradient.push_back(2.0 * i);
  }
  // Two maxima (300, 250) joining at 200 above a minimum at 100.
  ov.tree.vertices = {{0.5f, 300.0}, {0.2f, 250.0}, {0.8f, 200.0}, {0.5f, 100.0}};
  ov.tree.edges = {{0, 2}, {1, 2}, {2, 3}};

  isocontour_info info = describe(m, ic("a", 0.25), 100.0, 300.0, &ov); // isovalue 150
  EXPECT_DOUBLE_EQ(info.isovalue, 150.0);
  EXPECT_NEAR(info.area, 2.5, 1e-12);
  EXPECT_NEAR(info.min_volume, 25.0, 1e-12);
  EXPECT_NEAR(info.max_volume, 75.0, 1e-12);
  EXPECT_NEAR(info.gradient, 5.0, 1e-12);
  EXPECT_EQ(info.components, 1);

  info = describe(m, ic("b", 0.6), 100.0, 300.0, &ov); // isovalue 220: both maxima
  EXPECT_NEAR(info.area, 6.0, 1e-12);
  EXPECT_EQ(info.components, 2);

  // An explicit domain wider than the data: the spectrum is still read by data value, and an
  // isovalue beyond the data takes the end sample.
  info = describe(m, ic("c", 1.0), 0.0, 400.0, &ov); // isovalue 400
  EXPECT_NEAR(info.area, 10.0, 1e-12);
  EXPECT_EQ(info.components, 0);

  // No published range: the samples are taken to span the domain.
  ov.has_range = false;
  info = describe(m, ic("d", 0.5), 0.0, 10.0, &ov);
  EXPECT_NEAR(info.area, 5.0, 1e-12);
}

// --- the preview raster ---------------------------------------------------------------------

TEST(TfEditorPreview, SizeAndDegenerateRequests) {
  const model m = make_model();
  const view v;
  EXPECT_EQ(render_preview(m, v, nullptr, 0.0, 1.0, 64, 16).size(), 64u * 16u * 4u);
  EXPECT_TRUE(render_preview(m, v, nullptr, 0.0, 1.0, 0, 16).empty());
  EXPECT_TRUE(render_preview(m, v, nullptr, 0.0, 1.0, 16, -1).empty());
  EXPECT_EQ(render_preview(m, v, nullptr, 5.0, 5.0, 8, 8).size(), 8u * 8u * 4u);
}

TEST(TfEditorPreview, OversizeRequestsAreRefusedNotWrapped) {
  const model m = make_model();
  const view v;
  // More bytes than a vector can hold on any target: refused (empty), neither wrapped nor thrown.
  EXPECT_TRUE(render_preview(m, v, nullptr, 0.0, 1.0, INT_MAX, INT_MAX).empty());
  // On a 32-bit size_t (wasm32, Win32) w*h*4 wrapped to 0 bytes and to 256 KiB here, and the
  // raster wrote far past either. (On 64-bit these are legal 4 GiB rasters, too big for a test.)
  if constexpr (sizeof(std::size_t) < 8) {
    EXPECT_TRUE(render_preview(m, v, nullptr, 0.0, 1.0, 32768, 32768).empty());
    EXPECT_TRUE(render_preview(m, v, nullptr, 0.0, 1.0, 65536, 16385).empty());
  }
}

// render_preview takes the view as given, so its zoom window may be narrower than clamp_zoom()
// allows: a bar far off such a window maps beyond long's range. It must be skipped, not rounded
// (the UCRT's lround returns 0 there, which drew a stray bar at column 0).
TEST(TfEditorPreview, FarOffCanvasIsocontourDrawsNothing) {
  model m = make_model();
  view v = bare_view();
  v.show_isocontours = true;
  v.zoom_lo = 0.5;
  v.zoom_hi = 0.5 + 1e-9;
  const int w = 64, h = 16;
  const std::vector<std::uint8_t> bare = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  m.isocontours = {ic("left", 0.0), ic("right", 1.0)}; // x = -/+ 3.15e10
  EXPECT_EQ(render_preview(m, v, nullptr, 0.0, 1.0, w, h), bare);
  m.isocontours = {ic("on", 0.5)}; // x = 0: still drawn
  EXPECT_TRUE(is(pixel(render_preview(m, v, nullptr, 0.0, 1.0, w, h), w, 0, 0), 255, 0, 255));
}

TEST(TfEditorPreview, ColourRampFollowsTheZoomWindow) {
  model m;
  m.color = {cn(0.0, 0, 0, 0), cn(1.0, 1, 1, 1)};
  m.alpha = {an(0.0, 1.f), an(1.0, 1.f)};
  view v = bare_view();
  const int w = 33, h = 8;
  std::vector<std::uint8_t> img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 0, 3), 0, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, w - 1, 3), 255, 255, 255));
  EXPECT_TRUE(is(pixel(img, w, 16, 0), 128, 128, 128));

  v.zoom_lo = 0.5;
  img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 0, 3), 128, 128, 128));
  EXPECT_TRUE(is(pixel(img, w, w - 1, 3), 255, 255, 255));

  v = bare_view();
  v.show_color = false; // flat grey
  img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 0, 3), 128, 128, 128));
  EXPECT_TRUE(is(pixel(img, w, w - 1, 3), 128, 128, 128));
}

TEST(TfEditorPreview, AlphaCheckerShowsTransparency) {
  model m;
  m.color = {cn(0.0, 1, 0, 0), cn(1.0, 1, 0, 0)};
  m.alpha = {an(0.0, 0.f), an(1.0, 0.f)};
  const view v = bare_view();
  const int w = 16, h = 8;
  std::vector<std::uint8_t> img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 0, 0), 255, 0, 0)); // the bare ramp by default
  preview_options opt;
  opt.alpha_checker = true;
  img = render_preview(m, v, nullptr, 0.0, 1.0, w, h, opt);
  EXPECT_TRUE(is(pixel(img, w, 0, 0), 115, 115, 115)); // a dark check
  EXPECT_TRUE(is(pixel(img, w, 4, 0), 191, 191, 191)); // its light neighbour
}

TEST(TfEditorPreview, IsocontourBarsAndOpacityCurve) {
  model m;
  m.color = {cn(0.0, 0, 0, 1), cn(1.0, 0, 0, 1)}; // blue
  m.alpha = {an(0.0, 1.f), an(1.0, 1.f)};         // the curve runs along the top row
  m.isocontours = {ic("a", 0.5)};
  const int w = 65, h = 16;
  view v = bare_view();
  v.show_isocontours = true;
  std::vector<std::uint8_t> img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 32, 0), 255, 0, 255));   // the magenta bar
  EXPECT_TRUE(is(pixel(img, w, 32, h - 1), 0, 255, 0)); // its green foot
  EXPECT_TRUE(is(pixel(img, w, 31, h - 1), 0, 255, 0));
  EXPECT_TRUE(is(pixel(img, w, 10, 0), 0, 0, 255)); // no curve shown
  v.show_opacity = true;
  img = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 10, 0), 0, 0, 0)); // the opacity curve
  EXPECT_TRUE(is(pixel(img, w, 10, 5), 0, 0, 255));
}

TEST(TfEditorPreview, HistogramLinearClipAndLog) {
  model m;
  m.color = {cn(0.0, 1, 1, 1), cn(1.0, 1, 1, 1)};
  m.alpha = {an(0.0, 1.f), an(1.0, 1.f)};
  overlays ov;
  ov.has_range = true;
  ov.range_lo = 0.0;
  ov.range_hi = 4.0;
  ov.histogram = {1, 10, 4, 1}; // the spike clips at 1.5 * 4 = 6 above the baseline 1
  view v = bare_view();
  v.show_histogram = true;
  v.show_color = false; // grey background, opaque black bars
  const int w = 40, h = 21;
  std::vector<std::uint8_t> img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 15, 0), 0, 0, 0));     // bin 1: clipped, full height
  EXPECT_TRUE(is(pixel(img, w, 25, h - 1), 0, 0, 0)); // bin 2: 60%
  EXPECT_TRUE(is(pixel(img, w, 25, 8), 0, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, 25, 7), 128, 128, 128));
  EXPECT_TRUE(is(pixel(img, w, 5, h - 1), 128, 128, 128)); // bin 0: the baseline

  v.log_histogram = true;
  img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 5, h - 1), 0, 0, 0)); // log(2)/log(11) of the height
  EXPECT_TRUE(is(pixel(img, w, 5, 0), 128, 128, 128));

  v.log_histogram = false;
  v.show_color = true; // translucent bars over the ramp
  img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  const px p = pixel(img, w, 15, 10);
  EXPECT_GT(p.r, 0);
  EXPECT_LT(p.r, 255);
  EXPECT_TRUE(is(pixel(img, w, 5, 10), 255, 255, 255));

  v.show_histogram = false;
  img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 15, 10), 255, 255, 255));
}

TEST(TfEditorPreview, ContourTreeAndSpectrumCurves) {
  model m;
  m.color = {cn(0.0, 1, 1, 1), cn(1.0, 1, 1, 1)};
  m.alpha = {an(0.0, 1.f), an(1.0, 1.f)};
  overlays ov;
  ov.has_range = true;
  ov.range_lo = 0.0;
  ov.range_hi = 4.0;
  ov.tree.vertices = {{0.f, 2.0}, {1.f, 2.0}}; // a vertical arc at value 2 (column 20)
  ov.tree.edges = {{0, 1}};
  for (int i = 0; i <= 10; ++i) {
    ov.spectrum.area.push_back(i);         // a rising diagonal
    ov.spectrum.min_volume.push_back(3.0); // constant: not drawn
    ov.spectrum.max_volume.push_back(3.0);
    ov.spectrum.gradient.push_back(3.0);
  }
  view v = bare_view();
  v.show_color = false;
  const int w = 41, h = 21;

  v.show_contour_tree = true;
  std::vector<std::uint8_t> img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 20, 10), 0, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, 20, 0), 128, 128, 128)); // the 10% inset
  EXPECT_TRUE(is(pixel(img, w, 20, h - 1), 128, 128, 128));

  v.show_contour_tree = false;
  v.show_contour_spectrum = true;
  img = render_preview(m, v, &ov, 0.0, 4.0, w, h);
  EXPECT_TRUE(is(pixel(img, w, 0, h - 1), 255, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, w - 1, 0), 255, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, 20, 10), 255, 0, 0));
  EXPECT_TRUE(is(pixel(img, w, w - 1, h - 1), 128, 128, 128));
  EXPECT_TRUE(is(pixel(img, w, 20, 0), 128, 128, 128));
}

// --- the meshed interval (<tf>.interval) ------------------------------------------------------

TEST(TfEditorInterval, NamesTwoDistinctIsocontoursInEitherOrder) {
  model m = make_model();
  m.isocontours = {ic("a", 0.25), ic("x,y", 0.5), ic("b", 0.75)};
  int a = -1, b = -1;
  EXPECT_TRUE(interval_isocontours(m, "a,b", a, b));
  EXPECT_EQ(a, 0);
  EXPECT_EQ(b, 2);
  EXPECT_TRUE(interval_isocontours(m, "b,a", a, b));
  EXPECT_EQ(a, 2);
  EXPECT_EQ(b, 0);
  // An id holding a comma: every split is tried.
  EXPECT_TRUE(interval_isocontours(m, "x,y,b", a, b));
  EXPECT_EQ(a, 1);
  EXPECT_EQ(b, 2);
  EXPECT_TRUE(interval_isocontours(m, "a,x,y", a, b));
  EXPECT_EQ(a, 0);
  EXPECT_EQ(b, 1);
  for (const char *bad : {"", "a", "a,", ",b", "a,a", "a,zz", "zz,b", "a;b"})
    EXPECT_FALSE(interval_isocontours(m, bad, a, b)) << bad;
}

TEST(TfEditorPreview, IntervalShadesExactlyTheBandBetweenItsBars) {
  model m;
  m.color = {cn(0.0, 1, 1, 1), cn(1.0, 1, 1, 1)}; // white
  m.alpha = {an(0.0, 1.f), an(1.0, 1.f)};
  m.isocontours = {ic("a", 0.25), ic("b", 0.75), ic("c", 0.9)}; // columns 16, 48, 58
  const int w = 65, h = 16;
  view v = bare_view();
  v.show_isocontours = true;
  const std::vector<std::uint8_t> plain = render_preview(m, v, nullptr, 0.0, 1.0, w, h);
  preview_options opt;
  opt.interval_a = "b"; // the bounds in either order
  opt.interval_b = "a";
  const std::vector<std::uint8_t> img = render_preview(m, v, nullptr, 0.0, 1.0, w, h, opt);
  // Inside: white under a quarter of magenta, on every row (the feet are the bars' own).
  const int shade = static_cast<int>(std::lround((1.f - kIntervalAlpha) * 255.f));
  int inside = 0, outside_changed = 0;
  for (int x = 0; x < w; ++x)
    for (int y = 0; y < h; ++y) {
      const px p = pixel(img, w, x, y), q = pixel(plain, w, x, y);
      const bool same = p.r == q.r && p.g == q.g && p.b == q.b;
      if (x > 17 && x < 47) { // clear of the bars and their 3-column green feet
        if (is(p, 255, shade, 255))
          ++inside;
      } else if (x < 15 || x > 49) {
        if (!same)
          ++outside_changed;
      }
    }
  EXPECT_EQ(inside, (47 - 18) * h) << "every pixel strictly between the bars is shaded";
  EXPECT_EQ(outside_changed, 0) << "nothing outside the band changes";
  EXPECT_TRUE(is(pixel(img, w, 17, 3), 255, shade, 255)); // beside a bar, above its foot
  EXPECT_TRUE(is(pixel(img, w, 47, 3), 255, shade, 255));
  EXPECT_TRUE(is(pixel(img, w, 15, 3), 255, 255, 255)); // just outside
  EXPECT_TRUE(is(pixel(img, w, 49, 3), 255, 255, 255));
  EXPECT_TRUE(is(pixel(img, w, 16, 3), 255, 0, 255)); // the bars stay on top
  EXPECT_TRUE(is(pixel(img, w, 48, 3), 255, 0, 255));
  EXPECT_TRUE(is(pixel(img, w, 58, 3), 255, 0, 255)); // c is not a bound
  EXPECT_TRUE(is(pixel(img, w, 53, 3), 255, 255, 255));

  // Through the zoom window: [0.5, 1] puts a off the left edge and b at column 32.
  view z = v;
  z.zoom_lo = 0.5;
  const std::vector<std::uint8_t> zoomed = render_preview(m, z, nullptr, 0.0, 1.0, w, h, opt);
  EXPECT_TRUE(is(pixel(zoomed, w, 0, 3), 255, shade, 255));
  EXPECT_TRUE(is(pixel(zoomed, w, 20, 3), 255, shade, 255));
  EXPECT_TRUE(is(pixel(zoomed, w, 40, 3), 255, 255, 255));

  // No band with the isocontours hidden, or when an id is not in the model.
  view hidden = v;
  hidden.show_isocontours = false;
  EXPECT_EQ(render_preview(m, hidden, nullptr, 0.0, 1.0, w, h, opt),
            render_preview(m, hidden, nullptr, 0.0, 1.0, w, h));
  preview_options gone = opt;
  gone.interval_b = "zz";
  EXPECT_EQ(render_preview(m, v, nullptr, 0.0, 1.0, w, h, gone), plain);
}
