/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridSegmentation — the Segmentation tab of a VolumeGridNode (docs/roadmap/
// VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, §5, P9): the grid's `segment` request runs cvc::
// segment_seeded on the grid's volume, seeded by its point classes (the shown variable and
// timestep's), then cvc::class_surfaces on the labels, as one sliced task on the app's scheduler
// (cvc::volume_grid_segment_stepper, §1.9), while the slices stay live. The node owns one and calls
// it from its tick; the volume_grid.ari component's Segmentation tab binds its keys.
//
// WHAT IT DRAWS. On each plane a `labels` child of the VolumeSliceNode: a quad over the plane's
// voxels textured with the labels in their classes' colours (cvc::color_labels), see-through by
// grid.segment.opacity, pushed back in depth less than the slice, so the plane's points and lines
// stay on top. While the fronts advance (grid.segment.live) the labels so far draw there, a few
// times a second. In the 3D view, a `segments` group under the grid's `surfaces` group holds each
// class's surface (`class_<n>`, cvc::class_surfaces) in its colour; slice views hide `surfaces`.
//
// KEYS (under the node's state path):
//   grid.segment.*        cvc::seeded_segmentation_params' table (t_low, t_high, gamma, automatic,
//                         normalized_thresholds, class_volumes, cp_*); variable and timestep are
//                         written as the grid's shown ones (grid.source.var and time) when a run
//                         starts, since the shown classes are the seeds. t_low's and t_high's "min"
//                         and "max" properties are the volume's data range (0 and 255 with
//                         normalized_thresholds), for a slider's range: key
//   grid.segment.*        cvc::volume_grid_segment_view_params' table: show, opacity,
//   show_surfaces,
//                         surface_opacity, make_surfaces, live
//   grid.segment.mesh.*   cvc::class_surface_params' table (smoothing, lambda, mu, close, ...)
//   grid.segment.rows     the last result's classes as rows a list: or repeat: binds: the count,
//                         then <i>.name, <i>.color ("r,g,b"), <i>.visible, <i>.voxels and
//                         <i>.triangles; editing a colour or visible recolours or hides that class
//   grid.segment.info.*   published: classes, labelled, border, automatic_seeds, classified_seeds,
//                         t_low_normalized, t_high_normalized, triangles; grid.segment.warnings
//                         (the count, then <i>)
//   grid.request          `segment` starts a run (a run under way is left alone, and says so);
//                         `stop` stops it at its next step, keeping the last result
//   grid.busy, stoppable, progress, phase, status, stats.work_ms, pid   the §1.9 job keys
// DATA CHANNEL (the last result; a stopped run keeps it): grid.segment.labels (a std::shared_ptr<
// const cvc::volume>, UChar, 255 none), grid.segment.classes (a std::shared_ptr<const std::vector<
// cvc::volume>>: each class's voxels, zero elsewhere, named after the class; Ariadne's data-save
// writes them as one multi-variable file, VolumeGridRover's SegTmp.rawv), grid.segment.border (a
// std::shared_ptr<const cvc::point_classes>, one class "border") and grid.segment.surfaces.<n>
// (class n's cvc::geometry).
//
// A new volume (the grid adopting another) stops a run and clears the result, whose labels no
// longer fit. No statics, no globals: two grids, or two apps, share nothing (§1.3).
#ifndef CVC_GL_VOLUMEGRIDSEGMENTATION_H
#define CVC_GL_VOLUMEGRIDSEGMENTATION_H

#include <array>
#include <atomic>
#include <cstdint>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/segmentation.h>
#include <cvc/volume/volume_grid_segment.h>
#include <cvc/volume/volume_slice.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class job_keys;
template <class R> class result_job;
namespace gl {

class GeometryNode;
class NullGraphicNode;
class VolumeGridNode;

class VolumeGridSegmentation {
public:
  explicit VolumeGridSegmentation(VolumeGridNode &grid);
  ~VolumeGridSegmentation(); // stops a run

  // Every key of the tab given its default, once (the node's first tick: a document's props seeded
  // theirs before it).
  void seed();
  // A key under grid.segment changed (`key` relative to grid.segment). Any thread.
  void keyChanged(const std::string &key);
  // `segment` or `stop` (with what follows it on the request line). False for another command,
  // and for `stop` when no run is under way (so another tab's job may take it).
  bool request(const std::string &cmd, const std::string &arg);
  // The grid took another volume: stop a run, clear the result.
  void volumeChanged();
  // Drive the run (progress, live labels, the result when it ends), apply the keys, and redraw the
  // label layers and surfaces that changed. On the owner thread, after the planes' own tick.
  // Returns true when the drawing changed.
  bool tick();

  bool running() const { return static_cast<bool>(m_job); }
  // For hosts and tests: the plane's label layer, the 3D group, the rows, the drawn labels (the
  // result's, or the labels so far while it runs), and how many times a layer was recoloured.
  std::shared_ptr<GeometryNode> labelsOverlay(cvc::slice_axis a) const;
  std::shared_ptr<NullGraphicNode> segments() const { return m_segments; }
  const std::vector<cvc::segment_class_row> &rows() const { return m_rows; }
  std::shared_ptr<const cvc::volume> shownLabels() const;
  bool showingLiveLabels() const { return m_live != nullptr && m_job != nullptr; }
  std::size_t layerUploads() const { return m_layerUploads; }

private:
  void status(const std::string &s);
  void launch();
  void apply(cvc::volume_grid_segment_result r);
  void clear();
  void ensureLayers();
  void readView();
  void readRows();
  void syncSurfaces();
  bool syncLayers();
  void publishInfo(const cvc::seeded_segmentation &s, std::size_t triangles);

  VolumeGridNode &m_grid;
  std::unique_ptr<cvc::job_keys> m_keys;
  bool m_seeded = false;

  // The run.
  std::shared_ptr<cvc::result_job<cvc::volume_grid_segment_result>> m_job;
  std::shared_ptr<cvc::volume_grid_segment_stepper> m_stepper; // its live labels
  int m_task = -1;
  cvc::point_classes m_runSeeds; // the seeds of the run under way, for its rows
  int m_runVar = 0, m_runTime = 0;
  std::vector<cvc::segment_class_row> m_liveRows; // its classes' colours while it marches
  std::shared_ptr<const cvc::volume> m_live;
  std::uint64_t m_liveSeen = 0;

  // The last result.
  std::shared_ptr<const cvc::volume> m_labels;
  std::vector<cvc::segment_class_row> m_rows;
  std::vector<std::shared_ptr<GeometryNode>> m_classNodes; // by class (null: no surface)
  std::size_t m_surfaceKeys = 0;                           // grid.segment.surfaces.<n> written

  // Drawing.
  cvc::volume_grid_segment_view_params m_view;
  std::array<std::shared_ptr<GeometryNode>, 3> m_layers;
  std::array<std::string, 3> m_layerSignature;
  std::size_t m_layerUploads = 0;
  std::shared_ptr<NullGraphicNode> m_segments;
  std::string m_surfaceSignature;
  std::string m_rangeSignature; // the thresholds' min and max, as last written

  std::atomic<std::uint64_t> m_viewVersion{1}, m_rowsVersion{1};
  std::uint64_t m_viewSeen = 0, m_rowsSeen = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLUMEGRIDSEGMENTATION_H
