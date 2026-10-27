/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tf editor — the GL-free half of the Ariadne `tf_editor` widget: canvas geometry, hit
// testing, the drag gesture and every model edit the widget offers, shared by the ImGui widget
// (cvcGL), the terminal/composed fallback (core) and the unit tests. Nothing here draws.
//
// Interaction (a port of VolumeRover2's ColorTable2, with its quirks fixed):
//  - The canvas maps a ZOOM WINDOW [zoom_lo, zoom_hi] of the normalized axis across its width; the
//    zoom strip above it shows the whole axis with two range bars that set that window.
//  - Pick priority, nearest first: alpha nodes, isocontour bars, colour bars (ColorTable2's depth
//    order); a colour/alpha endpoint yields to an interior node over it. Range bars live in the
//    strip. Handles are 10x10 px; a bar is grabbable over its full height within +/-3 px.
//  - Dragging moves the grabbed node by IDENTITY (no ColorTable2 leapfrog): colour and alpha nodes
//    are clamped between their neighbours; the first and last colour/alpha nodes are pinned to
//    t = 0 / 1 (an alpha endpoint's opacity still drags); isocontours move freely in [0,1].
//  - Edits: add an alpha node at (t, a), a colour node at t (colour interpolated from its
//    neighbours), an isocontour at t; delete any node except a colour/alpha endpoint; recolour a
//    colour node or an isocontour.

#ifndef CVC_TF_EDITOR_H
#define CVC_TF_EDITOR_H

#include <cstdint>
#include <cvc/tf/model.h>
#include <string>
#include <vector>

namespace cvc {
namespace tf {

struct overlays; // cvc/tf/overlays.h

// What a handle refers to.
enum class part { none, color, alpha, isocontour, zoom_lo, zoom_hi };

// A grabbed / hovered handle. `index` is into the model's list for color/alpha/isocontour (the
// isocontour is also named by `id`, which survives re-sorting).
struct handle {
  part kind = part::none;
  int index = -1;
  std::string id;
  bool valid() const { return kind != part::none; }
};

// Screen geometry of one widget instance, in pixels (y grows downward). The strip may have zero
// height (hidden).
struct layout {
  float canvas_x = 0.f, canvas_y = 0.f, canvas_w = 0.f, canvas_h = 0.f;
  float strip_x = 0.f, strip_y = 0.f, strip_w = 0.f, strip_h = 0.f;
};

// Per-instance view state that is NOT part of the model (the core persists it per widget).
struct view {
  double zoom_lo = 0.0, zoom_hi = 1.0; // the normalized window the canvas shows
  bool show_color = true;              // colour ramp background + colour bars
  bool show_opacity = true;            // the opacity curve + its nodes
  bool show_isocontours = true;        // isocontour bars
  bool show_histogram = true;
  bool show_contour_spectrum = false;
  bool show_contour_tree = false;
  bool log_histogram = false; // log(1+count) scale instead of ColorTable2's clipped linear scale
  bool show_info = false;     // the per-isocontour information table (ColorTable2's InfoDialog)
  bool operator==(const view &o) const;
  bool operator!=(const view &o) const { return !(*this == o); }
};

// The smallest zoom window (normalized span) the range bars can make.
constexpr double kMinZoomSpan = 0.01;
// Handle half-size and bar grab tolerance, px.
constexpr float kHandleHalf = 5.f;
constexpr float kBarGrab = 3.f;
// The closest (normalized) a moved or added colour/alpha node may come to another, so the lists
// stay strictly ordered; ~0.1 px even at the deepest zoom on a 1000 px canvas.
constexpr double kMinNodeGap = 1e-6;

// Clamp a view's zoom window into [0,1] with at least kMinZoomSpan between the bars.
void clamp_zoom(view &v);

// --- mapping ------------------------------------------------------------------------------
// Canvas x <-> normalized t through the zoom window; canvas y <-> opacity (1 at the top).
float x_of(const layout &l, const view &v, double t);
double t_of(const layout &l, const view &v, float x);
float y_of(const layout &l, float a);
float a_of(const layout &l, float y);
// Strip x <-> normalized t over the whole axis.
float strip_x_of(const layout &l, double t);
double strip_t_of(const layout &l, float x);

// --- hit testing ---------------------------------------------------------------------------
// The handle under (x, y), honouring the view's visibility toggles; part::none for empty space.
// A point inside the strip tests only the range bars (the nearer one); anywhere else tests the
// canvas, grown by kHandleHalf so edge handles are grabbable whole. Within one kind the topmost
// (last-drawn, i.e. highest index) handle wins, except that a colour/alpha endpoint yields to an
// interior node of its list under the same point: a node dragged against an endpoint stops
// kMinNodeGap (sub-pixel) short of it and must stay grabbable, selectable and deletable (the
// endpoint is pickable again once the node moves off it). A bar is hit within kBarGrab of its x
// over the canvas height, or on its 10x10 handle at the canvas bottom.
handle hit_test(const model &m, const view &v, const layout &l, float x, float y);

// --- edits -------------------------------------------------------------------------------------
// Each returns true when the model changed. Positions are normalized and clamped to [0,1]; a NaN
// position or value changes nothing.
// add_alpha / add_color refuse a position within kMinNodeGap of an existing node of their list
// (so never on an endpoint). An isocontour may share a position.
bool add_alpha(model &m, double t, float a);
bool add_color(model &m, double t);             // colour = ramp at t
std::string add_isocontour(model &m, double t); // returns the new id ("" if not added)
bool remove(model &m, const handle &h);         // colour/alpha endpoints are protected
bool set_color(model &m, int color_index, const float rgb[3]);
bool set_isocontour_rgb(model &m, const std::string &id, const float rgb[3]);
bool clear_isocontour_rgb(model &m, const std::string &id); // back to the ramp colour
// Move a node. Colour/alpha nodes are clamped strictly between their neighbours (endpoints pinned
// in t); an isocontour moves anywhere in [0,1] (the list is re-sorted, the id is kept).
bool move_color(model &m, int index, double t);
bool move_alpha(model &m, int index, double t, float a);
bool move_isocontour(model &m, const std::string &id, double t);

// --- the drag gesture -----------------------------------------------------------------------
// One per widget instance: press() grabs the handle under the pointer (or nothing), drag() moves
// it — mutating the model, or the view for a range bar — and release() ends it.
//
// The press remembers where on the handle it landed, so the handle keeps that offset from the
// pointer instead of jumping to it: a click with no motion edits nothing. Colour/alpha nodes are
// tracked by index (their clamping keeps the order), isocontours by id. The range bars keep at
// least kMinZoomSpan between them (a bar stops at the other rather than pushing it).
class gesture {
public:
  // Grab the handle under (x, y). Returns it (invalid for empty space, which grabs nothing).
  // Resets view_changed() / model_changed().
  handle press(const model &m, const view &v, const layout &l, float x, float y);
  // Move the grabbed handle to the pointer. Returns true if the MODEL changed (a zoom bar changes
  // only `v`, reported through view_changed()).
  bool drag(model &m, view &v, const layout &l, float x, float y);
  // End the drag. view_changed() / model_changed() keep describing the finished gesture until the
  // next press(), so a release-time commit can read them after this.
  void release();
  // Abandon the drag without further edits (e.g. the model was replaced from outside); unlike
  // release() it also clears the changed flags, so nothing stale gets committed.
  void cancel() {
    release();
    _view_changed = _model_changed = false;
  }

  bool active() const { return _grab.valid(); }
  const handle &grabbed() const { return _grab; }
  bool view_changed() const { return _view_changed; }
  // Whether this gesture changed the model since press() (the release-commit edge).
  bool model_changed() const { return _model_changed; }

private:
  handle _grab;
  float _dx = 0.f, _dy = 0.f; // pointer minus handle centre at press()
  bool _view_changed = false;
  bool _model_changed = false;
};

// --- information about one isocontour (ColorTable2's InfoDialog, with data-unit isovalues) -----
struct isocontour_info {
  std::string id;
  double at = 0.0;       // normalized
  double isovalue = 0.0; // data units
  // Contour-spectrum values interpolated at the isovalue (NaN when no spectrum is published), in
  // the spectrum's own (physical) units; the samples are uniform over the overlays' range
  // ([lo, hi] when no range is published), and an isovalue outside it takes the nearest end.
  double area = 0.0, min_volume = 0.0, max_volume = 0.0, gradient = 0.0;
  // Connected components of the level set (contour-tree arcs spanning the isovalue; -1 when no
  // contour tree is published).
  int components = -1;
};
// `lo`/`hi`: the model's effective domain (effective_domain()), which turns `iso.at` into data.
isocontour_info describe(const model &m, const isocontour &iso, double lo, double hi,
                         const overlays *ov);

// --- the meshed interval (<tf>.interval) -----------------------------------------------------
// A mesher that meshes the shell between two isocontours (fe_lab's mesh.region: interval)
// publishes their ids beside the model as "<id_a>,<id_b>". The isocontours it names in `m`:
// true with `a` / `b` set to their indices when `interval` names two distinct isocontours of `m`
// (an id may itself hold a comma: every split is tried), else false (an empty string, a missing
// id). The bounds come in either order.
bool interval_isocontours(const model &m, const std::string &interval, int &a, int &b);
// The opacity of the band both editors shade between the interval's bars (in magenta, the bars'
// colour).
constexpr float kIntervalAlpha = 0.25f;

// --- a raster preview (the terminal / composed fallback) ------------------------------------
struct preview_options {
  // Composite the colour ramp over a checkerboard by the opacity at each column, so transparent
  // ranges read as transparent (ColorTable2 drew the bare ramp).
  bool alpha_checker = false;
  // The meshed interval: the ids of its two bounding isocontours (TfEditorInput's interval_a /
  // interval_b). When both are isocontours of the model, the columns between their bars are
  // shaded a translucent magenta, over the ramp and the histogram and under everything else.
  std::string interval_a, interval_b;
};
// Render the canvas into `w`x`h` RGBA8 pixels (row 0 at the top, opaque): the colour ramp under the
// view's zoom window (flat grey with show_color off), then far to near the histogram (bars over
// the zoom window, ColorTable2's linear scale clipped at 1.5x the second-largest bin, or
// log(1+count)), the meshed interval's band (opt.interval_a/b), the contour tree (black, 10%
// vertical inset), the four spectrum curves (each
// min-max normalized; area red, min_volume green, max_volume blue, gradient yellow), isocontour
// bars (magenta, green foot) and the opacity polyline (black). `lo`/`hi` is the effective domain
// the canvas spans; the overlays are placed by their data values. Returns w*h*4 bytes, or an empty
// vector when w or h is not positive or w*h*4 exceeds what a byte vector can hold (its max_size():
// 2^31-1 on a 32-bit size_t such as wasm32 or Win32, where e.g. 32768x32768 is refused, never
// wrapped). A size within that bound that cannot be allocated throws std::bad_alloc.
std::vector<std::uint8_t> render_preview(const model &m, const view &v, const overlays *ov,
                                         double lo, double hi, int w, int h);
std::vector<std::uint8_t> render_preview(const model &m, const view &v, const overlays *ov,
                                         double lo, double hi, int w, int h,
                                         const preview_options &opt);

} // namespace tf
} // namespace cvc

#endif // CVC_TF_EDITOR_H
