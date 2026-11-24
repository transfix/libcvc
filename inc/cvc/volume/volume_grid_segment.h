/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// The data side of VolumeGridRover's Segmentation tab (docs/roadmap/VOLROVER-LIBRARY-PORTS-
// ROADMAP.md §4 and §5, P9): what cvcGL's `volume_grid` node runs and shows when its `segment`
// request segments the grid's volume from its point classes, with no GL in it, so all of it is
// testable here and runs the same natively and in wasm:
//
//   - volume_grid_segment_view_params, a parameter table (§1.5): how the result draws, the labels
//     on the slices (show, opacity) and the classes' surfaces in the 3D view (show_surfaces,
//     surface_opacity), and whether the labels draw while the fronts advance (live). The run's own
//     parameters are cvc::seeded_segmentation_params' and cvc::class_surface_params' tables;
//   - volume_grid_segment_stepper: cvc::segment_stepper, then cvc::class_surfaces_stepper on its
//     labels, as one resumable stepper (§1.9) with one progress bar and one Stop, which also keeps
//     the labels so far for the slices to draw while it marches;
//   - segment_class_rows: the result's classes as rows a `list:` or a `repeat:` binds (name,
//     colour, shown, voxels, triangles), coloured as the point classes that seeded them;
//   - color_labels: a slice of the labels as the see-through colour layer drawn over the plane.
//
// Everything here holds no state of its own: no statics, no globals (§1.3).

#ifndef CVC_VOLUME_VOLUME_GRID_SEGMENT_H
#define CVC_VOLUME_VOLUME_GRID_SEGMENT_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_slice.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
class state;

// ---- parameters (§1.5) ------------------------------------------------------------------------

// How the segmentation draws: keys under <node>.grid.segment beside seeded_segmentation_params'.
struct volume_grid_segment_view_params {
  bool show = true;             // the labels on the slices, in the classes' colours
  double opacity = 0.5;         // how much of the slice a label hides
  bool show_surfaces = true;    // each class's surface in the 3D view
  double surface_opacity = 1.0; // the surfaces'
  bool make_surfaces = true;    // a run makes the surfaces too (class_surfaces after the labels)
  bool live = true;             // the labels draw while the fronts advance (a copy of the labels
                                // a few times a second)
};

#ifndef SWIG
template <>
struct param_table<volume_grid_segment_view_params>
    : param_fields<volume_grid_segment_view_params> {
  using P = volume_grid_segment_view_params;
  static constexpr auto fields = std::tuple{
      field("show", &P::show, doc("draw the labels on the slices, in the classes' colours")),
      field("opacity", &P::opacity, range(0.0, 1.0),
            doc("how much of the slice a label hides (0: none, 1: all)")),
      field("show_surfaces", &P::show_surfaces, doc("draw each class's surface in the 3D view")),
      field("surface_opacity", &P::surface_opacity, range(0.0, 1.0),
            doc("the class surfaces' opacity in the 3D view")),
      field("make_surfaces", &P::make_surfaces,
            doc("a run makes each class's surface after its labels (class_surfaces)")),
      field("live", &P::live,
            doc("draw the labels while the fronts advance (a copy of the labels a few times a "
                "second)")),
  };
};
#endif // SWIG

// Whether `path` (relative to the node, as volume_grid_session_key's) is a key of the Segmentation
// tab a session keeps: grid.segment.<seeded_segmentation_params' and the view's fields> and
// grid.segment.mesh.<class_surface_params' fields> (how the surfaces are made).
bool volume_grid_segment_session_key(const std::string &path);

// ---- the run ----------------------------------------------------------------------------------

struct volume_grid_segment_result {
  explicit volume_grid_segment_result(app &ctx) : segmentation(ctx) {}
  seeded_segmentation segmentation;
  // Each class's surface, by its label (class n is label n): empty when the run made none.
  class_surface_result surfaces;
};

// segment_stepper on (vol, seeds, seg), then (with `surfaces`) class_surfaces_stepper on its
// labels with `surf`, as one stepper (§1.9). Phases: segment_stepper's ("checking", "normalising",
// "seeding", "classifying", "marching", "writing"), then class_surfaces_stepper's as "surfaces:
// reading", "surfaces: extracting", "surfaces: smoothing" and "surfaces: normals". Progress: the
// labels' share of [0, 1] is 0.75 with surfaces (1 without), the surfaces' the rest. A step runs
// the inner steppers a piece at a time (at most 4096 units each), so cancel() ends it within a
// piece, even at budget 0. The result is the two steppers', whatever the schedule.
//
// Live labels: with live_seconds >= 0, every live_seconds of the run (0: after every step) while
// segment_stepper::labels_so_far has some, the step copies them into a new volume that
// live_labels() hands out (from any thread) until the next. The copy is the step's, outside its
// budget (a voxel at a time, about a millisecond per million voxels).
class volume_grid_segment_stepper {
public:
  volume_grid_segment_stepper(const volume &vol, point_classes seeds,
                              seeded_segmentation_params seg = {}, class_surface_params surf = {},
                              bool surfaces = true, double live_seconds = -1.0);
  ~volume_grid_segment_stepper();
  volume_grid_segment_stepper(volume_grid_segment_stepper &&) noexcept;
  volume_grid_segment_stepper &operator=(volume_grid_segment_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();                     // thread-safe
  volume_grid_segment_result take(); // once done; moved out, so call it once

  // The labels so far (above); null before the first copy. Thread-safe, and safe while another
  // thread steps or take()s: they outlive the run's own state until the stepper goes. The version
  // rises with each copy.
  std::shared_ptr<const volume> live_labels() const;
  std::uint64_t live_version() const;

private:
  struct impl;
  struct shared; // cancel() and the live labels: kept apart from impl, which take() frees
  std::shared_ptr<shared> _shared;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<volume_grid_segment_stepper>
volume_grid_segment_stepper_of(const volume &vol, point_classes seeds,
                               seeded_segmentation_params seg = {}, class_surface_params surf = {},
                               bool surfaces = true, double live_seconds = -1.0);

// ---- the result's classes ---------------------------------------------------------------------

// One class of a segmentation as the tab lists it.
struct segment_class_row {
  std::string name;
  std::array<double, 3> color{1.0, 0.0, 0.0};
  bool visible = true; // its labels and its surface drawn
  std::size_t voxels = 0;
  std::size_t triangles = 0;
};

// The rows of a segmentation's classes (`r.class_names`, class n the n-th), coloured as the classes
// of `seeds` of `variable` and `timestep` that seeded them, in their order (segment_seeded's), and
// with seed_mode::critical_points' two last classes, "feature" in gold and "background" in blue,
// the background not shown (its surface would wrap the whole volume). A class past both (a seeds
// edited since) takes point_classes' palette. Voxels from r.class_voxels; triangles from
// `surfaces` (class n's is label n's surface).
std::vector<segment_class_row> segment_class_rows(const point_classes &seeds, int variable,
                                                  int timestep, const seeded_segmentation &r,
                                                  const class_surface_result &surfaces = {});

// The keys a `list:` or `repeat:` binds for the rows, under a node (<node>.grid.segment.rows): the
// count at the node itself and row i's fields at <i>.name, <i>.color ("r,g,b"), <i>.visible (1 or
// 0), <i>.voxels and <i>.triangles. Rows past the count are removed; the count is written last.
void write_segment_rows(state &rows, const std::vector<segment_class_row> &r);
// The colours and shown flags the keys hold now, into `r` (rows the keys lack, or whose values do
// not parse, keep theirs; each problem appended to `problems`). True when any changed.
bool read_segment_rows(state &rows, std::vector<segment_class_row> &r,
                       std::vector<std::string> *problems = nullptr);

// A slice of the labels (extract_slice of a segmentation's labels) as RGBA bytes in image order
// (the top row, the slice's last, first, as color_slice gives them): a voxel of class n (its value)
// in rows[n]'s colour with alpha `opacity` when that row is shown, every other voxel (unlabelled,
// a class with no row or not shown) clear. `opacity` is clamped to [0, 1].
std::vector<std::uint8_t> color_labels(const volume_slice_data &labels,
                                       const std::vector<segment_class_row> &rows, double opacity);

} // namespace cvc

#endif // CVC_VOLUME_VOLUME_GRID_SEGMENT_H
