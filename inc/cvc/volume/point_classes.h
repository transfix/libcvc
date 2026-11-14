/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::point_classes: classes of voxel points, the seeds GenSeg's segmentation grows from and the
// points VolumeGridRover marks (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, §5, P0d).
//
//   cvc::point_classes pc;
//   pc.add("membrane", 12, 40, 7);          // a voxel, by its i j k indices
//   pc.add("background", 0, 0, 0);
//   cvc::write_point_classes(app, pc, "seeds.vgr");
//   cvc::point_classes back = cvc::read_point_classes(app, "seeds.vgr");
//
// The model: each class has a name, a colour, a display radius, the variable and timestep of the
// volume it marks, and its points as voxel indices i0 j0 k0 i1 j1 k1 .... A class is known by its
// name, variable and timestep together, as VolumeGridRover kept one list of classes per variable
// and timestep. The class metadata are values a node keeps on the state tree
// (param_table<point_class>, §1.5); the points go on the data channel.
//
// Files. The readers and writers are each app's own (point_class_handlers(app), a
// cvc::file_handler_registry), so two apps never see each other's (§1.3). The built-in formats:
//   .vgr   VolumeGridRover's point classes (XML), read and written, with a radius attribute the
//          legacy file didn't have;
//   .pts   VolumeGridRover's point list: blocks of a count and that many object coordinates, one
//          class a block, read only. The volume's dims and bounding box (point_class_read_params)
//          turn the coordinates into voxel indices as the legacy did.
//
// Everything here holds no state of its own: no statics, no globals.

#ifndef CVC_VOLUME_POINT_CLASSES_H
#define CVC_VOLUME_POINT_CLASSES_H

#include <cstddef>
#include <cvc/core/file_handler_registry.h>
#include <cvc/core/param_table.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {
class app;

// A file that can't be read or written: missing, malformed, or of a format no handler serves.
// The message names the file and what was wrong with it.
class point_class_file_error : public std::runtime_error {
public:
  explicit point_class_file_error(const std::string &what) : std::runtime_error(what) {}
};

struct point_class {
  std::string name;
  std::vector<double> color{1.0, 0.0, 0.0}; // r, g, b in [0, 1]
  double radius = 1.0;                      // how large its points draw, in voxels
  int variable = 0;                         // the volume's variable and timestep it marks
  int timestep = 0;
  std::vector<int> points; // voxel indices: i0 j0 k0 i1 j1 k1 ...

  std::size_t size() const { return points.size() / 3; }
  // Whether voxel (i, j, k) is one of its points.
  bool contains(int i, int j, int k) const;
  bool operator==(const point_class &o) const {
    return name == o.name && color == o.color && radius == o.radius && variable == o.variable &&
           timestep == o.timestep && points == o.points;
  }
  bool operator!=(const point_class &o) const { return !(*this == o); }
};

struct point_classes {
  std::vector<point_class> classes; // in order

  // The index of the class named `name` of that variable and timestep, or -1.
  int class_index(const std::string &name, int variable = 0, int timestep = 0) const;
  // Add voxel (i, j, k) to the class, creating it (with the next colour of a fixed palette) when
  // it is new. False when the class already had the point. Throws std::invalid_argument for an
  // empty name, a negative index, variable or timestep.
  bool add(const std::string &name, int i, int j, int k, int variable = 0, int timestep = 0);
  // Remove voxel (i, j, k) from the class. False when it had no such point (or there is no such
  // class); the class stays, empty or not.
  bool remove(const std::string &name, int i, int j, int k, int variable = 0, int timestep = 0);
  // Add `c`, or replace the class of that name, variable and timestep in its place. Throws
  // std::invalid_argument for an empty name.
  void set_class(const point_class &c);
  // The classes' names, in order (a name used for several variables or timesteps appears once
  // for each).
  std::vector<std::string> class_names() const;
  std::size_t point_count() const;
  bool empty() const { return classes.empty(); }

  bool operator==(const point_classes &o) const { return classes == o.classes; }
  bool operator!=(const point_classes &o) const { return !(*this == o); }
};

// What is wrong with the classes, each problem a sentence naming the class; empty when nothing is:
// no name, two classes of one name, variable and timestep, a colour that isn't three values in
// [0, 1], a radius that isn't positive and finite, a negative variable or timestep, a point list
// that isn't index triples, a negative index, a point listed twice, and, given the volume's
// dims (x, y, z), a point outside it.
std::vector<std::string> point_class_problems(const point_classes &pc,
                                              const std::vector<int> &dims = {});

// The colour the palette gives the n-th class a reader or add() creates.
std::vector<double> point_class_palette(std::size_t n);

// How read_point_classes reads.
struct point_class_read_params {
  // The volume's dims (x, y, z voxels) and bounding box (xmin ymin zmin xmax ymax zmax). With
  // both, .pts object coordinates become voxel indices as VolumeGridRover computed them, and every
  // point read must lie inside the dims. Without, .pts coordinates are rounded to the nearest
  // index.
  std::vector<int> dims;
  std::vector<double> bbox;
  int variable = 0; // the variable and timestep .pts classes mark
  int timestep = 0;
};

#ifndef SWIG
template <> struct param_table<point_class> : param_fields<point_class> {
  using P = point_class;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("name", &P::name, doc("the class's name")),
      field("color", &P::color, range(0.0, 1.0), doc("its colour: r, g, b in [0, 1]")),
      field("radius", &P::radius, unit("voxel"), range(0.0, none),
            doc("how large its points draw")),
      field("variable", &P::variable, range(0, none), doc("the volume's variable it marks")),
      field("timestep", &P::timestep, range(0, none), doc("the volume's timestep it marks")),
  };
  static void check(const P &c, std::vector<std::string> &problems) {
    if (c.name.empty())
      problems.push_back("name: a class needs a name");
    if (c.color.size() != 3)
      problems.push_back("color: " + std::to_string(c.color.size()) +
                         " values, not three (r, g, b)");
    if (!(c.radius > 0))
      problems.push_back("radius: a class's points need a positive radius");
  }
};

template <> struct param_table<point_class_read_params> : param_fields<point_class_read_params> {
  using P = point_class_read_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("dims", &P::dims, unit("voxel"), range(1, none),
            doc("the volume's dims, x y z (with bbox: .pts coordinates become voxels)")),
      field("bbox", &P::bbox, doc("the volume's bounding box, xmin ymin zmin xmax ymax zmax")),
      field("variable", &P::variable, range(0, none), doc("the variable .pts classes mark")),
      field("timestep", &P::timestep, range(0, none), doc("the timestep .pts classes mark")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (!p.dims.empty() && p.dims.size() != 3)
      problems.push_back("dims: " + std::to_string(p.dims.size()) + " values, not three (x, y, z)");
    if (!p.bbox.empty() && p.bbox.size() != 6)
      problems.push_back("bbox: " + std::to_string(p.bbox.size()) +
                         " values, not six (xmin ymin zmin xmax ymax zmax)");
    if (p.bbox.size() == 6)
      for (int a = 0; a < 3; ++a)
        if (!(p.bbox[a + 3] > p.bbox[a]))
          problems.push_back("bbox: its max " + std::string(1, "xyz"[a]) + " is not above its min");
    if (p.dims.empty() != p.bbox.empty())
      problems.push_back("dims and bbox go together");
  }
};
#endif // SWIG

// One format's reader and writer, as contour_file_io is (cvc/geometry/contours.h).
struct point_class_file_io {
  virtual ~point_class_file_io() = default;
  virtual const std::string &id() const = 0;
  virtual const std::vector<std::string> &extensions() const = 0;
  virtual bool can_read() const = 0;
  virtual bool can_write() const = 0;
  // The classes in `filename`. Throws point_class_file_error. The default throws "can't read".
  virtual point_classes read(app &ctx, const std::string &filename,
                             const point_class_read_params &p) const;
  // The default throws "can't write".
  virtual void write(app &ctx, const point_classes &pc, const std::string &filename) const;
};

// The app's point-class formats, the built-in ones registered the first time an app asks. Safe
// from any thread.
file_handler_registry<point_class_file_io> &point_class_handlers(app &ctx);

std::vector<std::string> point_class_read_extensions(app &ctx);
std::vector<std::string> point_class_write_extensions(app &ctx);

// Read the classes in `filename` with the first of the app's handlers for its extension that
// succeeds; a point or class listed twice is read once. Throws std::invalid_argument when `p`
// isn't valid, and point_class_file_error when no handler can read the file or what it holds has
// problems (point_class_problems with p.dims: a point outside the volume, say).
point_classes read_point_classes(app &ctx, const std::string &filename,
                                 const point_class_read_params &p = {});

// Write `pc` to `filename` with the app's handler for its extension. Throws std::invalid_argument
// when point_class_problems(pc) finds anything, and point_class_file_error when the file can't be
// written.
void write_point_classes(app &ctx, const point_classes &pc, const std::string &filename);

} // namespace cvc

#endif // CVC_VOLUME_POINT_CLASSES_H
