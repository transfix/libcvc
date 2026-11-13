/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::contour_stack: planar contours on a stack of parallel slices, the type the contour tiler
// and VolumeGridRover's contours share (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3, P0d).
//
//   cvc::contour_stack s;
//   s.z_spacing = 0.05;
//   s.add(3, "axon", {0, 0, 1, 0, 1, 1, 0, 1});   // slice 3, component "axon", a unit square
//   cvc::write_contours(app, s, "series.ser");
//   cvc::contour_stack t = cvc::read_contours(app, "series.ser");
//
// The model:
//   - slices keyed by their z index; slice k's plane is at z = z_origin + k * z_spacing in world
//     units, unless the slice gives its own z (a Tiling series spaced unevenly);
//   - each slice maps a component name to closed planar polygons, x and y in world units;
//   - each component has a colour and a visibility (contour_component), in the stack's order.
//
// Files. The readers and writers are each app's own (contour_handlers(app), a
// cvc::file_handler_registry): two apps never see each other's, and a host adds formats to its
// app alone (§1.3). The built-in formats, by extension:
//   .ser      Reconstruct: the series file and its numbered section files beside it (name.1,
//             name.2, ...), read and written. Traces are mapped through the inverse of their
//             section's transform into world coordinates, as Reconstruct shows them.
//   .config   the 1996 Tiling program's configuration (PREFIX / SUFFIX / SLICE_RANGE / DETZ) and
//             the numbered .pts files it names, read and written; one component.
//   .contour  Tao Ju's ctr2suf contours, read and written; components become materials.
//   .pcd      VolumeGridRover's point cloud: every vertex, no connectivity; written only.
//   .cvcc     libcvc's own (JSON), read and written; keeps everything, for sessions.
//
// Everything here holds no state of its own: no statics, no globals.

#ifndef CVC_GEOMETRY_CONTOURS_H
#define CVC_GEOMETRY_CONTOURS_H

#include <cstddef>
#include <cvc/core/file_handler_registry.h>
#include <cvc/core/param_table.h>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {
class app;

// A file that can't be read or written: missing, malformed, or of a format no handler serves.
// The message names the file and what was wrong with it.
class contour_file_error : public std::runtime_error {
public:
  explicit contour_file_error(const std::string &what) : std::runtime_error(what) {}
};

// One closed planar polygon on a slice: its vertices in order as x0 y0 x1 y1 ..., in world units.
// The closing edge back to the first vertex is implied; the first vertex is not repeated.
struct contour_polygon {
  std::vector<double> xy;

  std::size_t size() const { return xy.size() / 2; }
  // The area enclosed, positive when the vertices run counter-clockwise.
  double signed_area() const;
  bool operator==(const contour_polygon &o) const { return xy == o.xy; }
  bool operator!=(const contour_polygon &o) const { return !(*this == o); }
};

// One slice: its polygons by component name, and its plane when the stack's spacing doesn't give
// it.
struct contour_slice {
  std::map<std::string, std::vector<contour_polygon>> components;
  // The slice's plane in world units when it is not z_origin + index * z_spacing; NaN (the
  // default) when it is.
  double z = std::numeric_limits<double>::quiet_NaN();

  bool operator==(const contour_slice &o) const;
  bool operator!=(const contour_slice &o) const { return !(*this == o); }
};

// A component: one object traced through the slices. Its name, colour and visibility are values a
// node keeps on the state tree (param_table<contour_component>, §1.5).
struct contour_component {
  std::string name;
  std::vector<double> color{1.0, 1.0, 1.0}; // r, g, b in [0, 1]
  bool visible = true;

  bool operator==(const contour_component &o) const {
    return name == o.name && color == o.color && visible == o.visible;
  }
  bool operator!=(const contour_component &o) const { return !(*this == o); }
};

struct contour_stack {
  std::map<int, contour_slice> slices; // by z index
  double z_spacing = 1.0;              // world units between consecutive indices
  double z_origin = 0.0;               // the plane of index 0
  // The components, in order. add() and the readers keep an entry here for every name a slice
  // uses; contour_problems() reports a name that has none.
  std::vector<contour_component> components;

  // Slice `index`'s plane in world units: its own z, or z_origin + index * z_spacing.
  double slice_z(int index) const;
  // Add a polygon (x0 y0 x1 y1 ...) to slice `index` under `component`, creating the slice and,
  // when it is new, the component (white, visible). Throws std::invalid_argument for an odd number
  // of coordinates or an empty name.
  void add(int index, const std::string &component, const std::vector<double> &xy);
  // Add `c`, or replace the component of that name in its place. Throws std::invalid_argument for
  // an empty name.
  void set_component(const contour_component &c);
  // The index into `components` of the one named `name`, or -1.
  int component_index(const std::string &name) const;
  // Slice `index`'s polygons of `component` (a copy; empty when there are none).
  std::vector<contour_polygon> polygons(int index, const std::string &component) const;
  // The z indices, ascending.
  std::vector<int> indices() const;
  // The components' names, in order.
  std::vector<std::string> component_names() const;
  std::size_t polygon_count() const;
  std::size_t vertex_count() const;
  bool empty() const { return slices.empty(); }

  bool operator==(const contour_stack &o) const;
  bool operator!=(const contour_stack &o) const { return !(*this == o); }
};

// What is wrong with a stack, each problem a sentence naming the slice and component; empty when
// nothing is: a polygon with fewer than three vertices, an odd number of coordinates or a
// coordinate that isn't finite; a component a slice uses that `components` lacks, or two components
// of one name; a colour that isn't three values in [0, 1]; a z_spacing that isn't positive and
// finite; a slice z or z_origin that isn't finite.
std::vector<std::string> contour_problems(const contour_stack &s);

// How read_contours reads. Every reader applies the slice range, the component filters,
// min_points and z_spacing; apply_transforms and keep_open are Reconstruct's.
struct contour_read_params {
  int first = -1; // the first slice index kept (-1: from the first in the file)
  int last = -1;  // the last slice index kept (-1: to the last)
  // Component names, as regular expressions (Perl syntax, matched against the whole name), as the
  // tiler's -c and -x: a component is kept when it matches an include (or there is none) and no
  // exclude.
  std::vector<std::string> include;
  std::vector<std::string> exclude;
  int min_points = 3;           // polygons with fewer vertices are dropped (their slice is kept)
  double z_spacing = 0;         // > 0: the slice spacing, in place of the file's
  bool apply_transforms = true; // Reconstruct: traces in world coordinates (false: as stored)
  bool keep_open = false;       // Reconstruct: keep open traces (closed="false") as polygons
};

#ifndef SWIG
template <> struct param_table<contour_component> : param_fields<contour_component> {
  using P = contour_component;
  static constexpr auto fields = std::tuple{
      field("name", &P::name, doc("the component's name")),
      field("color", &P::color, range(0.0, 1.0), doc("its colour: r, g, b in [0, 1]")),
      field("visible", &P::visible, doc("whether it is drawn")),
  };
  static void check(const P &c, std::vector<std::string> &problems) {
    if (c.name.empty())
      problems.push_back("name: a component needs a name");
    if (c.color.size() != 3)
      problems.push_back("color: " + std::to_string(c.color.size()) +
                         " values, not three (r, g, b)");
  }
};

template <> struct param_table<contour_read_params> : param_fields<contour_read_params> {
  using P = contour_read_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("first", &P::first, range(-1, none), doc("the first slice index read (-1: the first)")),
      field("last", &P::last, range(-1, none), doc("the last slice index read (-1: the last)")),
      field("include", &P::include,
            doc("component names to keep, as regular expressions (none: every one)")),
      field("exclude", &P::exclude, doc("component names to drop, as regular expressions")),
      field("min_points", &P::min_points, range(0, none),
            doc("polygons with fewer vertices are dropped")),
      field("z_spacing", &P::z_spacing, range(0.0, none),
            doc("the slice spacing in world units, in place of the file's (0: the file's)")),
      field("apply_transforms", &P::apply_transforms,
            doc("Reconstruct: map traces through their section transforms (world coordinates)")),
      field("keep_open", &P::keep_open,
            doc("Reconstruct: keep open traces as polygons (they are skipped by default)")),
  };
  // A first after last, and patterns that aren't regular expressions.
  static void check(const P &p, std::vector<std::string> &problems);
};
#endif // SWIG

// One format's reader and writer. A handler serves the extensions it lists (".ser"), spelled as
// read_contours looks them up: the file name's last extension, lower-cased, with its dot.
struct contour_file_io {
  virtual ~contour_file_io() = default;
  // A unique, freeform name ("reconstruct_ser : v1").
  virtual const std::string &id() const = 0;
  virtual const std::vector<std::string> &extensions() const = 0;
  virtual bool can_read() const = 0;
  virtual bool can_write() const = 0;
  // The stack in `filename`. A reader may skip what `p` leaves out (a slice range, open traces);
  // read_contours applies the rest. Throws contour_file_error. The default throws "can't read".
  virtual contour_stack read(app &ctx, const std::string &filename,
                             const contour_read_params &p) const;
  // Write `s` to `filename` (and the files that go with it). Throws contour_file_error. The
  // default throws "can't write".
  virtual void write(app &ctx, const contour_stack &s, const std::string &filename) const;
};

// The app's contour formats. The built-in ones are registered the first time an app asks; add a
// handler with contour_handlers(app).insert(...). Safe from any thread.
file_handler_registry<contour_file_io> &contour_handlers(app &ctx);

// The extensions the app's handlers read, and write, sorted.
std::vector<std::string> contour_read_extensions(app &ctx);
std::vector<std::string> contour_write_extensions(app &ctx);

// Read the stack in `filename` with the first of the app's handlers for its extension that
// succeeds, then keep what `p` selects. Throws std::invalid_argument when `p` isn't valid, and
// contour_file_error when no handler can read the file (the message lists each handler's error).
contour_stack read_contours(app &ctx, const std::string &filename,
                            const contour_read_params &p = {});

// Write `s` to `filename` with the app's handler for its extension. Throws std::invalid_argument
// when contour_problems(s) finds anything, and contour_file_error when the file can't be written.
void write_contours(app &ctx, const contour_stack &s, const std::string &filename);

} // namespace cvc

#endif // CVC_GEOMETRY_CONTOURS_H
