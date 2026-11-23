/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridRover's contours (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P10): curves drawn
// per object on the planes of a volume grid, linear or spline, edited with undo, exported as a
// cvc::contour_stack, and tiled into one surface per object by the contour tiler (cvc/geometry/
// tiler.h). No GL in it: cvcGL's `volume_grid` node (VolumeGridContours) draws and edits through
// it, so every edit is testable here and runs the same natively and in wasm.
//
// The model, as the legacy SurfRecon::Contour had it:
//   - an object (grid_contour_object) is a name, a colour, how its curves draw (linear, or a spline
//     resampled `samples` times per span) and its curves;
//   - a curve (grid_curve) lies on one plane: looking down `axis` at slice `index`, its control
//     points in the plane's (r, u) object coordinates (cvc/volume/volume_slice.h's planes: down z,
//     x right and y up; down y, x right and z up; down x, z right and y up). It is open while it is
//     being drawn, and closed once its last point joins its first;
//   - the frame (grid_frame) is the volume grid the curves were drawn on (its dims, origin and
//     spacing), so a stack or a surface can be made from the curves alone.
//
//   cvc::grid_contour_editor ed;
//   ed.set_frame(cvc::grid_frame_of(cvc::grid_of(vol)));
//   ed.add_object({"axon", {0.2, 0.8, 0.3}});
//   ed.add_point(0, cvc::slice_axis::z, 12, 3.0, 4.0);   // starts a curve on z = 12
//   ed.add_point(0, cvc::slice_axis::z, 12, 9.0, 4.5);   // ... and extends it
//   ed.add_point(0, cvc::slice_axis::z, 12, 6.0, 9.0);
//   ed.close_active();                                     // the triangle closes
//   cvc::contour_stack s = cvc::grid_contours_stack(ed.contours());  // the z planes' curves
//   cvc::grid_tile_result t = cvc::tile_grid_contours(ed.contours());  // a surface per object
//
// Everything here holds no state of its own: no statics, no globals (§1.3). An editor is one
// node's; two editors share nothing.

#ifndef CVC_VOLUME_VOLUME_GRID_CONTOURS_H
#define CVC_VOLUME_VOLUME_GRID_CONTOURS_H

#include <array>
#include <boost/describe/enum.hpp>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tiler.h>
#include <cvc/volume/volume_slice.h>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class state;

// ---- the model ----------------------------------------------------------------------------------

// How an object's curves draw between their control points: straight, or a spline through them
// (a centripetal Catmull-Rom, periodic on a closed curve), `samples` points per span.
enum class contour_interpolation { linear, spline };
BOOST_DESCRIBE_ENUM(contour_interpolation, linear, spline)

// The volume grid curves are drawn on: dims voxels along x, y, z, voxel 0's centre at `origin`,
// `spacing` between neighbouring centres (slice_grid's lo and spacing).
struct grid_frame {
  std::array<int, 3> dims{1, 1, 1};
  std::array<double, 3> origin{0.0, 0.0, 0.0};
  std::array<double, 3> spacing{1.0, 1.0, 1.0};

  // Where slice `index` looking down `a` lies along `a` (not clamped: a curve may lie past the
  // last).
  double plane(slice_axis a, std::int64_t index) const;
  // The slice whose plane is nearest `coord` along `a` (not clamped).
  std::int64_t nearest_index(slice_axis a, double coord) const;
  bool operator==(const grid_frame &o) const {
    return dims == o.dims && origin == o.origin && spacing == o.spacing;
  }
  bool operator!=(const grid_frame &o) const { return !(*this == o); }
};
// The frame of a slice_grid (its dims, lo and spacing).
grid_frame grid_frame_of(const slice_grid &g);

// One curve on one plane: its control points as r0 u0 r1 u1 ... in the plane's object coordinates.
struct grid_curve {
  slice_axis axis = slice_axis::z;
  std::int64_t index = 0;
  std::vector<double> ru;
  bool closed = false;

  std::size_t size() const { return ru.size() / 2; }
  bool operator==(const grid_curve &o) const {
    return axis == o.axis && index == o.index && ru == o.ru && closed == o.closed;
  }
  bool operator!=(const grid_curve &o) const { return !(*this == o); }
};

// One object: what VolumeGridRover's object list held. Its metadata are values a node keeps on the
// state tree (the object rows, below); its curves are data.
struct grid_contour_object {
  std::string name;
  std::vector<double> color{1.0, 1.0, 1.0}; // r, g, b in [0, 1]
  contour_interpolation interpolation = contour_interpolation::linear;
  int samples = 5; // points per span of a spline (the legacy default); 1 draws the control points
  bool visible = true;
  std::vector<grid_curve> curves;

  bool operator==(const grid_contour_object &o) const {
    return name == o.name && color == o.color && interpolation == o.interpolation &&
           samples == o.samples && visible == o.visible && curves == o.curves;
  }
  bool operator!=(const grid_contour_object &o) const { return !(*this == o); }
};

struct grid_contours {
  grid_frame frame;
  std::vector<grid_contour_object> objects;

  // The index of the object named `name`, or -1.
  long object_index(const std::string &name) const;
  std::size_t curve_count() const;
  std::size_t point_count() const;
  bool empty() const { return objects.empty(); }
  bool operator==(const grid_contours &o) const { return frame == o.frame && objects == o.objects; }
  bool operator!=(const grid_contours &o) const { return !(*this == o); }
};

// The most points a spline draws per span.
constexpr int grid_contour_max_samples = 64;

// What is wrong with the contours, each a sentence naming the object and curve; empty when nothing
// is: an object without a name or two of one name; a colour that isn't three values in [0, 1];
// samples outside [1, grid_contour_max_samples]; a curve with an odd number of coordinates, a
// coordinate that isn't finite, no points, or closed with fewer than three; a frame whose dims are
// below 1 or whose origin or spacing isn't finite (spacing positive).
std::vector<std::string> grid_contours_problems(const grid_contours &c);

// The polyline a curve draws: its control points (linear, or fewer than three), or the spline
// through them, `samples` points per span (the control point first). A closed curve's closing span
// is drawn too, but its first point is not repeated. Throws std::invalid_argument for an odd number
// of coordinates or samples outside [1, grid_contour_max_samples].
std::vector<double> sample_grid_curve(const std::vector<double> &ru, bool closed,
                                      contour_interpolation interpolation, int samples);

// ---- stacks -------------------------------------------------------------------------------------

// The closed curves on the planes looking down `axis` as a contour_stack (what the tiler tiles and
// the contour formats write): one component per object (its name, colour, visibility; every object,
// in order, even one with no curve there), each curve as drawn (sample_grid_curve), its x and y the
// plane's (r, u), slices by plane index, z_origin and z_spacing the frame's along `axis`. Open
// curves are left out; `open` (if given) gets how many. Throws std::invalid_argument when
// grid_contours_problems finds anything.
contour_stack grid_contours_stack(const grid_contours &c, slice_axis axis = slice_axis::z,
                                  std::size_t *open = nullptr);

// A stack's polygons as closed, linear curves on the planes looking down `axis` of `frame`: each
// slice on the plane nearest its z, one object per component (its name, colour, visibility).
// `warnings` (if given) gets a sentence per slice whose z lies off the frame's planes (by more than
// a hundredth of a spacing, or past the volume). Throws std::invalid_argument when contour_problems
// finds anything.
grid_contours grid_contours_from_stack(const contour_stack &s, const grid_frame &frame,
                                       slice_axis axis = slice_axis::z,
                                       std::vector<std::string> *warnings = nullptr);

// ---- tracing ------------------------------------------------------------------------------------

// The closed curves where slice `s` crosses `iso` (VolumeGridRover's isocontouring): marching
// squares over the slice's samples, a saddle split by its centre's value, each loop around the
// samples below `iso` (past the slice's edge counts as above, so every loop closes), in the plane's
// object coordinates of `f`, counter-clockwise, simplified (Douglas-Peucker) to within `tolerance`
// voxels, and loops enclosing less than `min_area` square voxels left out. Largest first. Throws
// std::invalid_argument for a non-finite iso or a negative tolerance or area.
std::vector<std::vector<double>> trace_slice(const volume_slice_data &s, const grid_frame &f,
                                             double iso, double tolerance = 0.5,
                                             double min_area = 4.0);

// ---- the editor ---------------------------------------------------------------------------------

// A curve, and a control point of one: indices into objects, curves and points. Valid until the
// next edit that removes curves or points.
struct grid_curve_ref {
  std::size_t object = 0, curve = 0;
  bool operator==(const grid_curve_ref &o) const { return object == o.object && curve == o.curve; }
  bool operator!=(const grid_curve_ref &o) const { return !(*this == o); }
  bool operator<(const grid_curve_ref &o) const {
    return object != o.object ? object < o.object : curve < o.curve;
  }
};
struct grid_point_ref {
  std::size_t object = 0, curve = 0, point = 0;
  grid_curve_ref curve_ref() const { return {object, curve}; }
  bool operator==(const grid_point_ref &o) const {
    return object == o.object && curve == o.curve && point == o.point;
  }
  bool operator!=(const grid_point_ref &o) const { return !(*this == o); }
  bool operator<(const grid_point_ref &o) const {
    if (object != o.object)
      return object < o.object;
    return curve != o.curve ? curve < o.curve : point < o.point;
  }
};

// How a selection changes (as point classes'): replaced, added to, or taken from.
enum class contour_selection_mode { replace, add, remove };
BOOST_DESCRIBE_ENUM(contour_selection_mode, replace, add, remove)

class grid_contour_editor {
public:
  static constexpr std::size_t undo_limit = 64;

  // The frame new curves are drawn in (the volume grid). The curves are kept as they are.
  void set_frame(const grid_frame &f);
  const grid_frame &frame() const { return _c.frame; }

  const grid_contours &contours() const { return _c; }
  // Replace the contours (their frame too). Clears the selection and the active curve; with
  // `undoable` the old contours are an undo step. Throws std::invalid_argument when
  // grid_contours_problems finds anything.
  void set_contours(grid_contours c, bool undoable = true);
  // Bumped by every change to the contours (curves or metadata), and to the selection.
  std::uint64_t version() const { return _version; }
  std::uint64_t selection_version() const { return _selection_version; }

  // Add an object (an undo step). Returns its index. Throws std::invalid_argument for an empty name
  // or one already used.
  std::size_t add_object(grid_contour_object o);
  // Replace every object's metadata (name, colour, interpolation, samples, visibility) and keep
  // their curves: what the object rows describe (reconcile_contour_object_rows).
  void set_objects(std::vector<grid_contour_object> objects, bool undoable = true);

  // The curves on the plane looking down `a` at `index`, of visible objects only when `visible`.
  std::vector<grid_curve_ref> curves_on(slice_axis a, std::int64_t index,
                                        bool visible = true) const;
  const grid_curve &curve(const grid_curve_ref &r) const;
  // A control point's (r, u).
  std::array<double, 2> point(const grid_point_ref &r) const;

  // The control point of a visible object (only of `object` when it is not -1) on that plane
  // nearest (r, u) within `radius` (object units); none when nothing is.
  std::optional<grid_point_ref> nearest_point(slice_axis a, std::int64_t index, double r, double u,
                                              double radius, long object = -1) const;

  // The curve the next point extends, and at which end (back: after its last point; front: before
  // its first). None after the curve closes, or when its point is deleted.
  struct active_curve {
    grid_curve_ref curve;
    bool back = true;
  };
  const std::optional<active_curve> &active() const { return _active; }
  void set_active(std::optional<active_curve> a);

  // Add a control point to object `object` on the plane looking down `a` at `index` (an undo step):
  //   - with `new_curve`, as the first point of a new curve;
  //   - else at the active curve's end, when it is an open curve of that object on that plane;
  //   - else at the nearer end of the object's open curve on that plane whose end is nearest;
  //   - else as the first point of a new curve.
  // The point is selected (alone) and its curve made active at that end. Throws std::out_of_range
  // for no such object and std::invalid_argument for a coordinate that isn't finite.
  grid_point_ref add_point(std::size_t object, slice_axis a, std::int64_t index, double r, double u,
                           bool new_curve = false);
  // Close an open curve of three or more points (an undo step). False when it can't be.
  bool close_curve(const grid_curve_ref &c);
  // Close the active curve. False when there is none, or it can't be.
  bool close_active();
  // Open a closed curve again, its active end after its last point (an undo step). False when it
  // isn't closed.
  bool open_curve(const grid_curve_ref &c);
  // Remove a curve, or every curve of an object (an undo step). Return how many went.
  bool delete_curve(const grid_curve_ref &c);
  std::size_t clear_object(std::size_t object);
  // Remove an object and its curves (an undo step). False when there is no such object.
  bool remove_object(std::size_t object);
  // Add curves to an object (one undo step), as they are. Throws std::out_of_range for no such
  // object and std::invalid_argument when grid_contours_problems would find them wrong.
  void add_curves(std::size_t object, std::vector<grid_curve> curves);

  // Move a control point to (r, u). With `undoable` the contours before it are an undo step (a
  // drag makes one at its start, with checkpoint(), and moves with undoable false after).
  void move_point(const grid_point_ref &p, double r, double u, bool undoable = true);
  // Move every selected point by (dr, du) in the plane (object units).
  void move_selection(double dr, double du, bool undoable = true);

  // The selection, sorted. Only points of visible objects are ever selected.
  const std::vector<grid_point_ref> &selection() const { return _selection; }
  bool selected(const grid_point_ref &p) const;
  void select(const grid_point_ref &p, contour_selection_mode m);
  // The control points of visible objects on the plane looking down `a` at `index` inside the
  // rectangle between (r0, u0) and (r1, u1), edges included: replaced, added to or taken from the
  // selection. Returns how many are selected.
  std::size_t select_rect(slice_axis a, std::int64_t index, double r0, double u0, double r1,
                          double u1, contour_selection_mode m);
  void select_all();
  void clear_selection();
  // Remove the selected points (an undo step): a curve left with none goes, and a closed curve
  // left with fewer than three opens. Returns how many points went.
  std::size_t delete_selected();

  void checkpoint();
  bool undo();
  std::size_t undo_depth() const { return _undo.size(); }

private:
  void push_undo();
  void changed();
  void drop_unselectable();

  grid_contours _c;
  std::vector<grid_point_ref> _selection;
  std::optional<active_curve> _active;
  std::deque<grid_contours> _undo;
  std::uint64_t _version = 1, _selection_version = 1;
};

// ---- gestures -----------------------------------------------------------------------------------

// One pick from a slice view on the plane looking down `axis` at `index`: where it met the plane,
// in the plane's object coordinates (r, u), and whether that lies inside the volume.
struct contour_pick {
  enum class phase_t { down, drag, up };
  phase_t phase = phase_t::down;
  slice_axis axis = slice_axis::z;
  std::int64_t index = 0;
  double r = 0.0, u = 0.0;
  bool inside = false;
  bool shift = false, ctrl = false, alt = false;
};

// What a pick did, for the node to show.
struct contour_pick_outcome {
  bool band = false; // a rubber band being dragged, on the pick's plane
  slice_axis band_axis = slice_axis::z;
  std::int64_t band_index = 0;
  std::array<double, 4> band_rect{0.0, 0.0, 0.0, 0.0}; // r0 u0 r1 u1
  bool changed = false, selection_changed = false;
  long object = -1; // a control point of this object was clicked: the node makes it current
  std::string note; // what it did ("added point 3 to axon on z = 12"), or ""
};

// The contours tool's slice-canvas gestures, as VolumeGridRover drew contours:
//   - click on the plane: a point of the current object (add_point: at the active curve's end, else
//     at the nearer end of its open curve on that plane, else a new curve); Shift+click starts a
//     new curve;
//   - click on the active curve's other end (three points or more): the curve closes;
//   - click on another control point: it is selected, and an end of an open curve becomes the
//     active end (the next point goes there); Shift / Ctrl / Alt+click (de)selects it;
//   - drag from a control point: the point moves (the selection moves when the point is in it), one
//     undo step per drag;
//   - drag elsewhere: a rubber band selecting control points (Shift adds, Ctrl or Alt takes away).
// `radius` is how near (object units) a click must land to take a control point. With `snap`, new
// and moved points land on voxel centres. One gesture at a time: a down starts one, drags continue
// it, an up ends it; a drag or up with no down starts nothing.
class grid_contour_gestures {
public:
  contour_pick_outcome apply(grid_contour_editor &ed, long current, double radius, bool snap,
                             const contour_pick &p);
  bool active() const { return _active; }
  void reset() { _active = false; }

private:
  enum class kind { click, band, move };
  bool _active = false;
  kind _kind = kind::click;
  contour_selection_mode _mode = contour_selection_mode::replace;
  slice_axis _axis = slice_axis::z;
  std::int64_t _index = 0;
  double _r0 = 0.0, _u0 = 0.0, _lr = 0.0, _lu = 0.0; // where it started, where it is now
  std::optional<grid_point_ref> _on;                 // the control point it started on
  bool _moved = false;
};

// ---- object rows --------------------------------------------------------------------------------

// The keys a `list:` widget binds for the objects, under a node (<node>.grid.objects): the row
// count at the node and row i's fields at <i>.name, <i>.color ("r,g,b"), <i>.interpolation (linear
// | spline), <i>.samples, <i>.visible, and <i>.curves and <i>.points (how many it has; written,
// never read).
//
// Rows from the keys: a missing or unparseable field takes its default (the colour from the point
// classes' palette), and each problem is appended to `problems`.
std::vector<grid_contour_object>
read_contour_object_rows(state &rows, std::vector<std::string> *problems = nullptr);
// Write the objects as rows, removing rows past the count.
void write_contour_object_rows(state &rows, const grid_contours &c);
// The objects the rows describe, their curves carried over from `old`: with as many rows as
// objects, row i is object i renamed or restyled; otherwise each row takes the curves of the first
// old object of its name no earlier row took, and a row matching none starts empty. A row without a
// name, or with a name an earlier row took, is named "object <n>" (its row number from 1, or the
// first such name free).
std::vector<grid_contour_object>
reconcile_contour_object_rows(const grid_contours &old,
                              const std::vector<grid_contour_object> &rows);

// ---- parameters (§1.5) --------------------------------------------------------------------------

// How a volume_grid node draws and edits its contours: the keys under <node>.grid.contour.
struct volume_grid_contour_params {
  bool show = true;           // the curves on the slices
  bool control_points = true; // their control points, as disks
  double point_size = 0.6;    // a control point's radius, in voxels; a click takes one within 1.5x
  double line_width = 2.0;    // the curves', in pixels
  bool snap = false;          // new and moved points on voxel centres
  bool show_3d = true;        // every slice's curves in the 3D view
  bool show_surfaces = true;  // the tiled surfaces, in the 3D view
  double surface_opacity = 1.0;
  bool auto_tile = false; // tile each time the contours are replaced or traced
  std::string source; // a state key whose data channel holds contours (a cvc::contour_stack or a
                      // cvc::grid_contours): they replace the grid's each time they change
};

#ifndef SWIG
template <>
struct param_table<volume_grid_contour_params> : param_fields<volume_grid_contour_params> {
  using P = volume_grid_contour_params;
  static constexpr auto fields = std::tuple{
      field("show", &P::show, doc("draw the curves on the slices")),
      field("control_points", &P::control_points, doc("draw the curves' control points")),
      field("point_size", &P::point_size, unit("voxel"), range(0.1, 10.0),
            doc("a control point's radius; a click takes a point within 1.5 times it")),
      field("line_width", &P::line_width, unit("pixel"), range(0.5, 20.0),
            doc("the curves' width")),
      field("snap", &P::snap, doc("put new and moved points on voxel centres")),
      field("show_3d", &P::show_3d, doc("draw every slice's curves in the 3D view")),
      field("show_surfaces", &P::show_surfaces, doc("draw the tiled surfaces in the 3D view")),
      field("surface_opacity", &P::surface_opacity, range(0.0, 1.0),
            doc("the tiled surfaces' opacity")),
      field("auto_tile", &P::auto_tile,
            doc("tile each time the contours are replaced (a source, a session, a data write) "
                "or traced")),
      field("source", &P::source,
            doc("the state key whose data channel holds contours (a contour_stack or the grid's "
                "curves): they replace the grid's each time they change")),
  };
};

template <> struct param_table<grid_contour_object> : param_fields<grid_contour_object> {
  using P = grid_contour_object;
  static constexpr auto fields = std::tuple{
      field("name", &P::name, doc("the object's name")),
      field("color", &P::color, range(0.0, 1.0), doc("its colour: r, g, b in [0, 1]")),
      field("interpolation", &P::interpolation,
            doc("how its curves draw between their control points: linear or spline")),
      field("samples", &P::samples, range(1, grid_contour_max_samples),
            doc("points per span of a spline")),
      field("visible", &P::visible, doc("whether its curves draw")),
  };
};
#endif // SWIG

// ---- tiling -------------------------------------------------------------------------------------

struct grid_tile_result {
  // One surface per object tiled (its name), in object coordinates: the closed curves of each plane
  // orientation tiled as a stack (grid_contours_stack) and joined. Triangles face outward, with
  // each vertex's normal and colour.
  std::map<std::string, geometry> surfaces;
  // The tiler's warnings, each naming the orientation ("XY: tiler.tile: ..."), and one per object
  // whose open curves were left out.
  std::vector<std::string> warnings;
};

// The surfaces of the contours' closed curves, through cvc::tile_contours, per plane orientation
// (down z, then y, then x: each orientation with a closed curve is its own stack). Coordinates go
// to the tiler in units of the frame's smallest spacing, from its origin, so that the tiler's
// absolute tolerances (made for values of order 1 to 10^4) hold for any volume; `params`' lengths
// stay in object units (curation_delta and collinear_epsilon and correspondence_overlap are scaled
// with them), and its slice range applies to every orientation. Throws tiler_error for invalid
// parameters, contours grid_contours_problems finds wrong, or no closed curve at all, and as
// tile_contours throws.
grid_tile_result tile_grid_contours(const grid_contours &c, const tile_params &params = {});

// tile_grid_contours a little at a time (§1.9): each orientation's tile_stepper in turn. Phases are
// the tile_stepper's; label() names the orientation and the slices ("XY slices 3 - 4"); progress
// weighs each orientation by its vertices. Constructing one checks its input (and throws as
// tile_grid_contours does) and converts the curves: O(points). cancel() is the one thread-safe
// member.
class grid_tile_stepper {
public:
  grid_tile_stepper(const grid_contours &c, tile_params params = {});
  ~grid_tile_stepper();
  grid_tile_stepper(grid_tile_stepper &&) noexcept;
  grid_tile_stepper &operator=(grid_tile_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const;
  std::string label() const;
  double progress() const;
  std::string status() const;
  void cancel();
  grid_tile_result take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<result_job<grid_tile_result>> tile_grid_contours_job(const grid_contours &c,
                                                                     tile_params params = {});

// ---- files --------------------------------------------------------------------------------------

// The contours as JSON:
//   { "format": "cvc.volume_grid.contours", "version": 1,
//     "frame": { "dims": [x, y, z], "origin": [x, y, z], "spacing": [x, y, z] },
//     "objects": [ { "name": ..., "color": [r, g, b], "interpolation": "linear", "samples": 5,
//                    "visible": true,
//                    "curves": [ { "axis": "z", "index": 12, "closed": true,
//                                  "points": [r, u, r, u, ...] }, ... ] }, ... ] }
// Numbers in their shortest round-trip form, so the contours read back are the same bit for bit.
// Throws std::invalid_argument when grid_contours_problems finds anything.
std::string grid_contours_json(const grid_contours &c);
// Read them back. Throws std::runtime_error starting with `what` and saying what is wrong: not
// JSON, not contours, a field of the wrong kind, contours with problems.
grid_contours grid_contours_from_json(const std::string &json,
                                      const std::string &what = "contours");

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_GRID_CONTOURS_H
