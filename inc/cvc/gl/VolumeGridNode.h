/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridNode — VolumeGridRover rebuilt as a scene node (docs/roadmap/VOLROVER-LIBRARY-PORTS-
// ROADMAP.md §4, P8): three VolumeSliceNode planes of one volume, one down each axis, with the
// point classes, crosshairs and a rubber band drawn on them as overlays, and the editing that
// VolumeGridRover's slice canvases did, driven by state keys. Ariadne's `type: volume_grid`
// (cvc/gl/ariadne/volume_grid_nodes.h) makes one; the volume_grid.ari component lays out its views
// and panel.
//
//     auto grid = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolumeGridNode>("grid");
//     grid->setSource(std::make_shared<cvc::slice_source>(vol));
//     ... in the render loop, before rendering: grid->tick();
//
// CHILDREN. `slice_xy`, `slice_xz` and `slice_zy` (VolumeSliceNode, down z, y and x) colour through
// the grid's own <node>.transfer_function, so one tf_editor drives all three; each holds two
// overlay children, `points` (the classes' points on that plane, as disks of their radius in
// voxels, unlit) and `lines` (the selection's rings, the crosshairs where the other two planes
// cross it, and a rubber band being dragged). `surfaces` is an empty group for what later phases
// put in the 3D view (P9's segmented classes, P10's tiled contours).
//
// KEYS, under the node's state path (all strings, as everywhere on the tree):
//   grid.view, grid.tool, grid.slice.{x,y,z}   cvc::volume_grid_params: the switch's view, what a
//                  click does, the three planes (-1 the middle, written back as its number; a
//                  plane past the last is drawn as the last and written back). Each slice key's
//                  "max" property is its last plane's number, so a slider takes its range from it
//   grid.display.* cvc::volume_grid_display_params: point_scale, show_points, crosshairs, grey
//   grid.source.*  cvc::volume_grid_source_params: what `load` reads (file, var, time)
//   grid.classes   the point classes' metadata as rows a list: binds: the count, then <i>.name,
//                  <i>.color ("r,g,b"), <i>.radius, <i>.variable, <i>.timestep, <i>.points (the
//                  point count, written); grid.classes.current, the class a click adds to (-1
//                  none). Editing, adding or removing rows edits the classes (an undo step)
//   grid.pick.*    written by a slice view (a viewport's `pick:`): each new seq is a gesture step
//                  (cvc::point_class_gestures) under grid.tool
//   grid.pointer.* written by a slice view's `pointer:`: where the pointer is, the probe
//   grid.cursor.{i,j,k,x,y,z,value,inside}  published: the probe (where the pointer is, or the
//                  last pick)
//   grid.selection.count  published: how many points are selected
//   grid.request   one command, consumed (written back as ""): load | save_classes <file> |
//                  load_classes <file> (replace the classes) | load_points <file> (add a file's
//                  classes and points, .pts or .vgr) | delete_selected | select_all |
//                  select_none | undo | step <n> (the shown view's plane, n planes on; Left /
//                  Right step 1 and PageUp / PageDown 5 in the legacy) | save_session <file> |
//                  load_session <file>. A relative file is the document's directory's
//                  (setBaseDir); a session is cvc/volume/volume_grid.h's JSON
//   grid.busy, stoppable, progress, phase, status, stats.work_ms   the §1.9 job keys
//                  (cvc/core/job_keys.h); status says what the last request or gesture did
//   grid.info.{dims,min,max,span,file}  published: the volume's
//   transfer_function   the cvc::tf::model all three planes colour through
// DATA CHANNEL: grid.points (std::shared_ptr<const cvc::point_classes>; a write from outside, a
// DSL program's state-data-set or pycvc, replaces the classes), grid.selection (the selected
// points as a point_classes) and grid.volume (std::shared_ptr<const cvc::volume>, while the
// volume is in memory; empty for a file read plane by plane).
#ifndef CVC_GL_VOLUMEGRIDNODE_H
#define CVC_GL_VOLUMEGRIDNODE_H

#include <array>
#include <atomic>
#include <cstdint>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_slice.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class volume;
namespace gl {

class GeometryNode;
class VolumeSliceNode;

class VolumeGridNode : public NullGraphicNode {
public:
  VolumeGridNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolumeGridNode() override;

  // What the planes cut: a volume in memory or a file read plane by plane. Creates the children on
  // the first call; `file` is what info.file says, `var` and `time` the variable and timestep it
  // holds (the classes of others are kept but not shown). Resets the selection and any gesture
  // under way, and sets the editor's dims (points outside them stay, reported in status). Owner
  // thread.
  void setSource(std::shared_ptr<cvc::slice_source> src, const std::string &file = "", int var = 0,
                 int time = 0);
  std::shared_ptr<cvc::slice_source> source() const { return m_source; }

  // How `request: load` reads grid.source.*: returns the source, or null with `error` set.
  // Ariadne's realizer sets one that resolves the file against the document's directory and reads a
  // large file plane by plane. Without one, load reads the file whole.
  using loader = std::function<std::shared_ptr<cvc::slice_source>(
      const cvc::volume_grid_source_params &, std::string &error)>;
  void setLoader(loader l) { m_loader = std::move(l); }
  // Where a relative file in a request resolves (the document's directory; "" the working one).
  void setBaseDir(const std::string &dir) { m_baseDir = dir; }
  const std::string &baseDir() const { return m_baseDir; }

  // The plane looking down `a`, and the surfaces group; null before the first setSource.
  std::shared_ptr<VolumeSliceNode> slice(cvc::slice_axis a) const;
  std::shared_ptr<NullGraphicNode> surfaces() const { return m_surfaces; }
  // The overlay children of the plane looking down `a` ("points" and "lines").
  std::shared_ptr<GeometryNode> pointsOverlay(cvc::slice_axis a) const;
  std::shared_ptr<GeometryNode> linesOverlay(cvc::slice_axis a) const;
  // The child names: "slice_zy", "slice_xz", "slice_xy" (down x, y, z).
  static const char *sliceName(cvc::slice_axis a);

  // The volume the transfer function's histogram is taken over: the volume in memory, or (a file
  // read plane by plane) the first plane shown, over the file's range. Null before setSource.
  std::shared_ptr<const cvc::volume> histogramVolume() const { return m_histogramVolume; }

  // The point classes and their editor (for hosts and tests; an edit made here is published on the
  // next tick).
  cvc::point_class_editor &editor() { return m_editor; }
  const cvc::point_class_editor &editor() const { return m_editor; }

  // Apply what changed since the last call: the keys, picks, the pointer, requests, rows and the
  // data channel; then redraw the overlays if they changed and tick the planes. Call once per frame
  // before rendering, on the owner thread. Returns true when the drawing changed.
  bool tick();

  // How many times the overlays were rebuilt (for tests).
  std::size_t overlayBuilds() const { return m_overlayBuilds; }

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  void ensureChildren();
  void publishInfo();
  void publishPoints(bool rows);
  void publishSelection();
  void status(const std::string &s);
  void syncSlices();
  void readKeys();
  void fixCurrent();
  void readRows();
  void adoptData();
  void consumePick();
  void consumePointer();
  void runRequest(const std::string &request);
  void cursorFrom(const std::string &base);
  bool voxelOf(const double world[3], std::array<std::int64_t, 3> &ijk) const;
  std::string resolvePath(const std::string &file) const;
  void rebuildOverlays();

  std::shared_ptr<cvc::slice_source> m_source;
  std::shared_ptr<const cvc::volume> m_histogramVolume;
  std::string m_file;
  int m_var = 0, m_time = 0; // what the source holds (the editor shows these classes)
  loader m_loader;
  std::string m_baseDir;
  std::array<std::shared_ptr<VolumeSliceNode>, 3> m_slices;
  std::array<std::shared_ptr<GeometryNode>, 3> m_points, m_lines;
  std::shared_ptr<NullGraphicNode> m_surfaces;
  cvc::point_class_editor m_editor;
  cvc::point_class_gestures m_gestures;
  cvc::volume_grid_params m_params;
  cvc::volume_grid_display_params m_display;
  std::array<std::int64_t, 3> m_index{0, 0, 0}; // the planes drawn
  bool m_grey = false;
  cvc::grid_pick_outcome m_band; // the rubber band being dragged (band false: none)

  // Bumped by handleStateChanged (any thread) and compared on tick (the owner thread).
  std::atomic<std::uint64_t> m_keysVersion{1}, m_rowsVersion{1}, m_pickVersion{1},
      m_pointerVersion{1}, m_requestVersion{1};
  std::uint64_t m_keysSeen = 0, m_rowsSeen = 0, m_pickSeen = 0, m_pointerSeen = 0,
                m_requestSeen = 0;
  std::string m_lastPickSeq, m_lastPointerSeq;
  std::string m_autoCurrent; // what fixCurrent last wrote to grid.classes.current
  std::shared_ptr<const cvc::point_classes> m_published; // what grid.points holds, as written
  std::uint64_t m_pointsVersionShown = 0, m_selectionVersionShown = 0;
  std::string m_overlaySignature;
  std::size_t m_overlayBuilds = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLUMEGRIDNODE_H
