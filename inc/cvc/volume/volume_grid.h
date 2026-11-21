/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The data side of VolumeGridRover (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P8): what
// cvcGL's `volume_grid` scene node edits and keeps, with no GL in it, so every edit is testable
// here and runs the same natively and in wasm:
//
//   - the node's parameters, declared once as tables (§1.5): volume_grid_params (the view switch,
//     the tool, the slice indices), volume_grid_display_params and volume_grid_source_params (the
//     state keys the node takes its volume, points and sessions from);
//   - point_class_editor: point classes marked, selected, moved and deleted on the planes of a
//     volume, with undo;
//   - point_class_gestures: the picks a slice view writes (down, drag, up, with Shift / Ctrl /
//     Alt), turned into those edits as VolumeGridRover's slice canvases turned mouse events;
//   - the class rows a `list:` widget binds (<node>.grid.classes.<i>.name ...), reconciled with the
//     classes as rows are edited, added and removed;
//   - sessions: the node's keys, its transfer function and its classes in one JSON file.
//
//   cvc::point_class_editor ed;
//   ed.set_dims({64, 64, 64});
//   ed.set_classes(classes);
//   ed.toggle(0, cvc::slice_axis::z, {12, 40, 7});      // adds the voxel to class 0, or removes
//                                                        // class 0's point under it on z = 7
//   ed.select_rect(cvc::slice_axis::z, 7, 0, 0, 20, 50, cvc::selection_mode::replace);
//   ed.move_selection(cvc::slice_axis::z, 2, -1);        // 2 voxels right, 1 down, in the plane
//   ed.delete_selected();
//   ed.undo();
//
// The planes are cvc/volume/volume_slice.h's: looking down z, x right and y up (the XY view); down
// y, x right and z up (XZ); down x, z right and y up (ZY). A plane coordinate pair (r, u) is a
// voxel's indices along those two axes.
//
// Everything here holds no state of its own: no statics, no globals (§1.3). An editor is one
// node's; two editors share nothing.

#ifndef CVC_VOLUME_VOLUME_GRID_H
#define CVC_VOLUME_VOLUME_GRID_H

#include <array>
#include <boost/describe/enum.hpp>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume_slice.h>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
class state;

// ---- parameters (§1.5) ------------------------------------------------------------------------

// The slice the view switch shows: VolumeGridRover's tabs. xy looks down z, xz down y, zy down x.
enum class grid_view { xy, xz, zy };
BOOST_DESCRIBE_ENUM(grid_view, xy, xz, zy)

// The axis a view looks down, and the view that looks down an axis.
slice_axis grid_view_axis(grid_view v);
grid_view grid_view_of(slice_axis a);

// What a click in a slice view does: navigate moves the other two planes to the voxel clicked;
// points marks, selects and moves point classes (point_class_gestures); probe only reads the voxel.
// (P10 adds contours.)
enum class grid_tool { navigate, points, probe };
BOOST_DESCRIBE_ENUM(grid_tool, navigate, points, probe)

// The keys under <node>.grid: the switch, the tool and where the three planes are.
struct volume_grid_params {
  grid_view view = grid_view::xy;
  grid_tool tool = grid_tool::points;
  std::int64_t slice_x = -1; // -1: the middle plane, written back as its number
  std::int64_t slice_y = -1;
  std::int64_t slice_z = -1;
};

// How the slices and their overlays draw: the keys under <node>.grid.display.
struct volume_grid_display_params {
  double point_scale = 1.0; // every class's radius times this, for drawing and for picking
  bool show_points = true;
  bool crosshairs = true; // where the other two planes cross each slice
  bool grey = false;      // the slices in grey, not through the transfer function
};

// Where the grid's data are: the keys under <node>.grid.source. Each path is a state key whose
// data channel holds the data, relative to the document's scope (a leading '/' is the root's): the
// app put it there (a document's data: entry, a program, a filter, another node), and the node
// follows it as it changes. The node reads no file.
struct volume_grid_source_params {
  std::string volume;  // a cvc::volume (or a slice_source): what the planes cut
  std::string points;  // point classes: they replace the grid's each time they change
  std::string session; // a volume_grid_session: applied each time it changes
  int var = 0;         // the variable and timestep the volume holds: the classes of others are
  int time = 0;        // kept, not shown
};

#ifndef SWIG
template <> struct param_table<volume_grid_params> : param_fields<volume_grid_params> {
  using P = volume_grid_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("view", &P::view,
            doc("the slice the view switch shows: xy (down z), xz (down y) or zy (down x)")),
      field("tool", &P::tool,
            doc("what a click in a slice view does: navigate moves the other planes there, "
                "points marks and selects point classes, probe only reads the voxel")),
      field("slice.x", &P::slice_x, unit("voxel"), range(-1.0, none),
            doc("the plane looking down x (the ZY view), from 0; -1 is the middle one")),
      field("slice.y", &P::slice_y, unit("voxel"), range(-1.0, none),
            doc("the plane looking down y (the XZ view), from 0; -1 is the middle one")),
      field("slice.z", &P::slice_z, unit("voxel"), range(-1.0, none),
            doc("the plane looking down z (the XY view), from 0; -1 is the middle one")),
  };
};

template <>
struct param_table<volume_grid_display_params> : param_fields<volume_grid_display_params> {
  using P = volume_grid_display_params;
  static constexpr auto fields = std::tuple{
      field("point_scale", &P::point_scale, range(0.1, 10.0),
            doc("every class's radius times this, as its points draw and as a click finds them")),
      field("show_points", &P::show_points, doc("draw the point classes on the slices")),
      field("crosshairs", &P::crosshairs, doc("draw where the other two planes cross each slice")),
      field("grey", &P::grey, doc("draw the slices in grey, not through the transfer function")),
  };
};

template <>
struct param_table<volume_grid_source_params> : param_fields<volume_grid_source_params> {
  using P = volume_grid_source_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("volume", &P::volume,
            doc("the state key whose data channel holds the volume the planes cut (relative to "
                "the document; a leading / is the root's), followed as it changes")),
      field("points", &P::points,
            doc("the state key whose data channel holds point classes: they replace the grid's "
                "each time they change")),
      field("session", &P::session,
            doc("the state key whose data channel holds a session: applied each time it "
                "changes")),
      field("var", &P::var, range(0, none),
            doc("the variable the volume holds (the classes of others are kept, not shown)")),
      field("time", &P::time, range(0, none),
            doc("the timestep the volume holds (the classes of others are kept, not shown)")),
  };
};
#endif // SWIG

// ---- the editor -------------------------------------------------------------------------------

// A point of a class: the class's index in point_classes::classes and the voxel. A class holds a
// voxel once, so the pair names one point, and stays valid while other points come and go.
struct point_ref {
  std::size_t cls = 0;
  std::array<int, 3> ijk{0, 0, 0};
  bool operator==(const point_ref &o) const { return cls == o.cls && ijk == o.ijk; }
  bool operator!=(const point_ref &o) const { return !(*this == o); }
  bool operator<(const point_ref &o) const { return cls != o.cls ? cls < o.cls : ijk < o.ijk; }
};

// How a selection changes: replaced, added to, or taken from.
enum class selection_mode { replace, add, remove };
BOOST_DESCRIBE_ENUM(selection_mode, replace, add, remove)

// A plane's two in-plane axes, right then up (slice_plane_axes), as a pair of indices into ijk.
std::pair<int, int> grid_plane_axes(slice_axis a);

class point_class_editor {
public:
  // How many undo steps are kept.
  static constexpr std::size_t undo_limit = 64;

  // The volume the points mark, x y z voxels. Points outside it are refused by add() and toggle()
  // and kept inside by move_selection(). Throws std::invalid_argument for a dimension below 1.
  void set_dims(const std::array<int, 3> &dims);
  const std::array<int, 3> &dims() const { return _dims; }
  bool inside(const std::array<int, 3> &ijk) const;

  // The variable and timestep shown: classes of another variable or timestep are kept, saved and
  // reconciled, but neither drawn, nor picked, nor selected.
  void set_shown(int variable, int timestep);
  int shown_variable() const { return _variable; }
  int shown_timestep() const { return _timestep; }
  bool shown(std::size_t cls) const;

  const point_classes &classes() const { return _pc; }
  // Replace the classes. Clears the selection; with `undoable`, the old classes are an undo step.
  void set_classes(point_classes pc, bool undoable = true);
  // Bumped by every change to the classes (points or metadata), and to the selection.
  std::uint64_t version() const { return _version; }
  std::uint64_t selection_version() const { return _selection_version; }

  // The points of the shown classes on the plane looking down `a` at `index`, in class order.
  std::vector<point_ref> points_on(slice_axis a, std::int64_t index) const;

  // The radius a class's points draw and are picked within, in voxels: its radius times `scale`.
  double pick_radius(std::size_t cls, double scale) const;
  // The point of a shown class nearest voxel `v` on the plane through it looking down `a` (only of
  // class `cls` when it is not -1), within that class's pick_radius(scale); none when nothing is.
  std::optional<point_ref> nearest(slice_axis a, const std::array<int, 3> &v, double scale,
                                   long cls = -1) const;

  // Add voxel `v` to class `cls`. False when the class already has it. Throws std::out_of_range
  // for no such class, and std::invalid_argument for a voxel outside the dims.
  bool add(std::size_t cls, const std::array<int, 3> &v);
  // Remove a point. False when there is no such point.
  bool remove(const point_ref &r);

  // VolumeGridRover's double-click: class `cls`'s point nearest `v` on the plane looking down `a`,
  // within its pick radius, is removed; else `v` is added to the class. `which` (if given) gets
  // the point added or removed. Throws as add() does, and std::invalid_argument for a class of a
  // variable or timestep not shown.
  enum class toggled { added, removed };
  toggled toggle(std::size_t cls, slice_axis a, const std::array<int, 3> &v, double scale = 1.0,
                 point_ref *which = nullptr);

  // The selection, sorted. Only points of shown classes are ever selected.
  const std::vector<point_ref> &selection() const { return _selection; }
  bool selected(const point_ref &r) const;
  void select(const point_ref &r, selection_mode m);
  // The points of the shown classes on the plane looking down `a` at `index` whose plane
  // coordinates (r, u) lie in the rectangle between (r0, u0) and (r1, u1), either corner first,
  // edges included: replaced, added to or taken from the selection. Returns how many are selected.
  std::size_t select_rect(slice_axis a, std::int64_t index, int r0, int u0, int r1, int u1,
                          selection_mode m);
  void select_all();
  void clear_selection();
  // The selection's classes and points as a point_classes (every class, empty where none is
  // selected), for the data channel.
  point_classes selection_classes() const;

  // Move every selected point by (dr, du) along the plane's right and up axes, the move cut short
  // so that every one stays inside the dims. A point landing on another point of its class merges
  // with it. Returns the move made. With `undoable` the classes before it are an undo step (a drag
  // makes one at its start and moves with undoable false after).
  std::pair<int, int> move_selection(slice_axis a, int dr, int du, bool undoable = true);
  // Remove the selected points. Returns how many went.
  std::size_t delete_selected();

  // Push the classes as they are now as an undo step (before a run of non-undoable edits).
  void checkpoint();
  // Back to the classes before the last undoable edit. False when there is none.
  bool undo();
  std::size_t undo_depth() const { return _undo.size(); }

private:
  void push_undo();
  void changed();
  void drop_unselectable();

  std::array<int, 3> _dims{1, 1, 1};
  int _variable = 0, _timestep = 0;
  point_classes _pc;
  std::vector<point_ref> _selection;
  std::deque<point_classes> _undo;
  std::uint64_t _version = 1, _selection_version = 1;
};

// ---- gestures ---------------------------------------------------------------------------------

// One pick from a slice view (scene_views.h's pick.* keys): where the pointer is on the plane
// looking down `axis`, as a voxel (indices rounded, not clamped: a band's corner may lie past the
// volume's edge) and whether that voxel is inside the volume.
struct grid_pick {
  enum class phase_t { down, drag, up };
  phase_t phase = phase_t::down;
  slice_axis axis = slice_axis::z;
  std::array<std::int64_t, 3> ijk{0, 0, 0};
  bool inside = false;
  bool shift = false, ctrl = false, alt = false;
};

// Parses a pick's phase ("down", "drag", "up") and modifiers ("shift,ctrl"); false for another
// phase.
bool parse_grid_pick_phase(const std::string &text, grid_pick::phase_t &out);
void parse_grid_pick_mods(const std::string &text, grid_pick &out);

// What a pick did, for the node to show.
struct grid_pick_outcome {
  // navigate: the planes to move (the two axes other than the pick's, -1 where unchanged).
  std::array<std::int64_t, 3> slice{-1, -1, -1};
  // A rubber band being dragged: its plane, and its corners in plane coordinates.
  bool band = false;
  slice_axis band_axis = slice_axis::z;
  std::int64_t band_index = 0;
  std::array<std::int64_t, 4> band_rect{0, 0, 0, 0}; // r0 u0 r1 u1
  bool points_changed = false, selection_changed = false;
  std::string note; // what it did ("added (12, 40, 7) to membrane"), or ""
};

// VolumeGridRover's slice-canvas gestures, on picks:
//   - points: a click (down and up on one voxel) toggles a point of the current class (removes its
//     point under the pointer, else adds one); a drag is a rubber band that selects the shown
//     classes' points on the plane, Shift adding to the selection and Ctrl or Alt taking from it;
//     a click with Shift or Ctrl / Alt selects or deselects the point under the pointer;
//     Shift+Ctrl or Shift+Alt drags the selection (one undo step per drag);
//   - navigate: a click or drag moves the other two planes to the voxel;
//   - probe: nothing (the node publishes every pick's voxel as the probe, whatever the tool).
// One gesture per view at a time: a down starts one, drags continue it, an up ends it. A pick of
// another phase without a down (a lost down) starts nothing.
class point_class_gestures {
public:
  // `current` is the class a click adds to (-1: none, so a click adds nothing); `scale` the
  // display's point_scale.
  grid_pick_outcome apply(point_class_editor &ed, grid_tool tool, long current, double scale,
                          const grid_pick &p);
  // Whether a gesture is under way (a down seen, its up not yet).
  bool active() const { return _active; }
  // Drop a gesture under way (the tool changed, the volume reloaded).
  void reset() { _active = false; }

private:
  enum class kind { click, band, move, navigate };
  bool _active = false;
  kind _kind = kind::click;
  selection_mode _mode = selection_mode::replace;
  slice_axis _axis = slice_axis::z;
  std::array<std::int64_t, 3> _start{0, 0, 0};
  std::array<std::int64_t, 3> _last{0, 0, 0};
  bool _moved = false;
};

// ---- class rows -------------------------------------------------------------------------------

// The keys a `list:` widget binds for the classes, under a node (<node>.grid.classes): the row
// count at the node itself and row i's fields at <i>.name, <i>.color ("r,g,b"), <i>.radius,
// <i>.variable, <i>.timestep, and <i>.points (how many points it has; written, never read).
//
// Rows from the keys: a missing or unparseable colour, radius, variable or timestep takes the
// class's default (variable and timestep: `variable` and `timestep`), and each problem is
// appended to `problems`.
std::vector<point_class> read_point_class_rows(state &rows, int variable, int timestep,
                                               std::vector<std::string> *problems = nullptr);
// Write the classes as rows (the count, then each row's fields), removing rows past the count.
void write_point_class_rows(state &rows, const point_classes &pc);
// The classes the rows now describe, their points carried over from `old`: with as many rows as
// classes, row i is class i renamed or restyled (a list's edit of one row); otherwise each row
// takes the points of the first old class of its name, variable and timestep no earlier row took
// (a row added or removed, or the table rewritten), and a row matching none starts empty.
point_classes reconcile_point_class_rows(const point_classes &old,
                                         const std::vector<point_class> &rows);

// ---- sessions ---------------------------------------------------------------------------------

// A node's session (§1.5, "Sessions"): its keys (paths relative to the node, values as text), the
// transfer function it colours through (a cvc::tf::model's string) and its point classes.
struct volume_grid_session {
  std::map<std::string, std::string> keys;
  std::string transfer_function;
  point_classes classes;
};

// The keys a session keeps: the parameter tables' (grid.view, grid.tool, grid.slice.*,
// grid.display.*, grid.source.var and grid.source.time) and grid.classes.current. Where the data
// come from (grid.source.volume, points and session) is the document's, and transient keys (the
// cursor, picks, published info and job keys, requests) never go in one.
bool volume_grid_session_key(const std::string &path);

// Write `s` as JSON:
//   { "format": "cvc.volume_grid.session", "version": 1, "keys": { "grid.view": "xy", ... },
//     "transfer_function": "...",
//     "classes": [ { "name": ..., "color": [r, g, b], "radius": ..., "variable": ...,
//                    "timestep": ..., "points": [i, j, k, ...] }, ... ] }
// Throws std::invalid_argument when point_class_problems finds anything, and std::runtime_error
// naming the file when it can't be written.
void write_volume_grid_session(const volume_grid_session &s, const std::string &filename);
std::string volume_grid_session_json(const volume_grid_session &s);
// Read one. Throws std::runtime_error naming the file and what is wrong: missing, not JSON, not
// a session, a key outside volume_grid_session_key, classes with problems.
volume_grid_session read_volume_grid_session(const std::string &filename);
volume_grid_session volume_grid_session_from_json(const std::string &json,
                                                  const std::string &what = "session");

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_GRID_H
