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
// DATA. The node reads and writes no file: it views data the app holds on the state tree, and
// follows them as they change. grid.source.volume names the state key whose data channel holds
// the volume (a document's data: entry, a program's, a filter's, another node's); the planes cut
// that volume itself, its voxels shared, never copied, so the main scene and the grid show one
// volume, and a new volume written there redraws both. grid.source.points and grid.source.session
// name where point classes and sessions come from the same way, and the node publishes its own
// (grid.points, grid.session) for the app to save (Ariadne's data-save).
//
//     auto grid = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolumeGridNode>("grid");
//     grid->ensureChildren();
//     grid->getState("grid.source.volume").value("/work.volume"); // where the app keeps it
//     cvc::state::instance(app)("work.volume").data(boost::any(volume_ptr));
//     ... in the render loop, before rendering: grid->tick();
//
// or, with the volume in hand, grid->setSource(std::make_shared<cvc::slice_source>(vol)).
//
// CHILDREN. `slice_xy`, `slice_xz` and `slice_zy` (VolumeSliceNode, down z, y and x) colour through
// the grid's own <node>.transfer_function, so one tf_editor drives all three; each holds two
// overlay children, `points` (the classes' points on that plane, as disks of their radius in
// voxels, unlit) and `lines` (the selection's rings, the crosshairs where the other two planes
// cross it, and a rubber band being dragged), then the contours' two (VolumeGridContours).
// `surfaces` is a group for what shows in the 3D view alone: the contours' curves and their tiled
// surfaces (VolumeGridContours), P9's segmented classes.
//
// KEYS, under the node's state path (all strings, as everywhere on the tree):
//   grid.view, grid.tool, grid.slice.{x,y,z}   cvc::volume_grid_params: the switch's view, what a
//                  click does, the three planes (-1 the middle, written back as its number; a
//                  plane past the last is drawn as the last and written back). Each slice key's
//                  "max" property is its last plane's number, so a slider takes its range from it
//   grid.display.* cvc::volume_grid_display_params: point_scale, show_points, crosshairs, grey
//   grid.source.*  cvc::volume_grid_source_params: volume, points and session, the state keys
//                  the node follows (relative to its scope, setScope; a leading '/' is the
//                  root's), and var and time, the variable and timestep the volume holds. The
//                  volume's key holds a std::shared_ptr<const cvc::volume> (a data: entry's), a
//                  std::shared_ptr<cvc::volume>, a cvc::volume or a std::shared_ptr<
//                  cvc::slice_source> (a host's large file read plane by plane); the points' key
//                  a cvc::point_classes (as those hold it), which replace the classes (an undo
//                  step) each time they change; the session's a cvc::volume_grid_session, applied
//                  each time it changes
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
//   grid.request   one command, consumed (written back as ""): delete_selected | select_all |
//                  select_none | undo | step <n> (the shown view's plane, n planes on; Left /
//                  Right step 1 and PageUp / PageDown 5 in the legacy), and the contours'
//                  (VolumeGridContours.h: tile, close_curve, ...; with the contours tool the
//                  first four act on the contours)
//   grid.objects, grid.contour.*, grid.tile.*   the contours' (VolumeGridContours.h)
//   grid.busy, stoppable, progress, phase, status, stats.work_ms   the §1.9 job keys
//                  (cvc/core/job_keys.h); status says what the last request or gesture did
//   grid.info.{dims,min,max,span,source}  published: the volume's (source: the key it is at)
//   transfer_function   the cvc::tf::model all three planes colour through
// DATA CHANNEL: grid.points (std::shared_ptr<const cvc::point_classes>; a write from outside, a
// DSL program's state-data-set or pycvc, replaces the classes), grid.selection (the selected
// points as a point_classes), grid.session (std::shared_ptr<const cvc::volume_grid_session>: the
// session as it stands, kept current) and grid.volume (the std::shared_ptr<const cvc::volume> the
// planes cut: the very pointer grid.source.volume held, while the volume is in memory; empty for
// a slice_source read plane by plane), and the contours' (grid.curves, grid.contours,
// grid.surfaces.<object>: VolumeGridContours.h).
#ifndef CVC_GL_VOLUMEGRIDNODE_H
#define CVC_GL_VOLUMEGRIDNODE_H

#include <array>
#include <atomic>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <cstdint>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_slice.h>
#include <functional>
#include <memory>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <vector>

namespace cvc {
class volume;
namespace gl {

class GeometryNode;
class VolumeGridContours;
class VolumeSliceNode;

class VolumeGridNode : public NullGraphicNode {
public:
  VolumeGridNode(cvc::app &ctx, const std::string &statePath, const std::string &name);
  ~VolumeGridNode() override;

  // The children (the three planes, their overlays, the surfaces group) and the probe's keys:
  // made once, by the first call or the first volume. A host that names its views by the planes
  // calls it before the volume is there (Ariadne's realizer does).
  void ensureChildren();

  // What the planes cut, handed in directly (a host with the volume in hand; grid.source.volume
  // is how an app hands it over the tree): a volume in memory or a file read plane by plane.
  // `label` is what info.source says, `var` and `time` the variable and timestep it holds (the
  // classes of others are kept but not shown). Resets the selection and any gesture under way,
  // and sets the editor's dims (points outside them stay, reported in status). Owner thread.
  void setSource(std::shared_ptr<cvc::slice_source> src, const std::string &label = "", int var = 0,
                 int time = 0);
  std::shared_ptr<cvc::slice_source> source() const { return m_source; }

  // The scope a relative grid.source.* key is resolved in (an Ariadne document's prefix, as its
  // binds are); "" the root. A leading '/' is always the root's.
  void setScope(const std::string &prefix) { m_scope = prefix; }
  const std::string &scope() const { return m_scope; }
  // The absolute state path grid.source.<which> ("volume", "points" or "session") names, or "".
  std::string sourcePath(const std::string &which);

  // The session as it stands: the session keys, the transfer function and the classes (what
  // grid.session holds).
  cvc::volume_grid_session session();

  // The plane looking down `a`, and the surfaces group; null before ensureChildren.
  std::shared_ptr<VolumeSliceNode> slice(cvc::slice_axis a) const;
  std::shared_ptr<NullGraphicNode> surfaces() const { return m_surfaces; }
  // The overlay children of the plane looking down `a` ("points" and "lines").
  std::shared_ptr<GeometryNode> pointsOverlay(cvc::slice_axis a) const;
  std::shared_ptr<GeometryNode> linesOverlay(cvc::slice_axis a) const;
  // The child names: "slice_zy", "slice_xz", "slice_xy" (down x, y, z).
  static const char *sliceName(cvc::slice_axis a);

  // The volume the transfer function's histogram is taken over: the volume in memory (the one
  // the app holds), or (a file read plane by plane) the first plane shown, over the file's range.
  // Null before the first volume.
  std::shared_ptr<const cvc::volume> histogramVolume() const { return m_histogramVolume; }

  // The point classes and their editor (for hosts and tests; an edit made here is published on the
  // next tick).
  cvc::point_class_editor &editor() { return m_editor; }
  const cvc::point_class_editor &editor() const { return m_editor; }

  // The contours: their editor, overlays, surfaces and tile job (VolumeGridContours.h).
  VolumeGridContours &contours() { return *m_contours; }
  const VolumeGridContours &contours() const { return *m_contours; }

  // Apply what changed since the last call: the keys, picks, the pointer, requests, rows and the
  // data channel; then redraw the overlays if they changed and tick the planes. Call once per frame
  // before rendering, on the owner thread. Returns true when the drawing changed.
  bool tick();

  // How many times the overlays were rebuilt (for tests).
  std::size_t overlayBuilds() const { return m_overlayBuilds; }

protected:
  void handleStateChanged(const std::string &childState) override;

private:
  // One followed source: the key's path as last resolved, and the node and its data's time.
  struct followed {
    std::string path, node;
    boost::posix_time::ptime mod;
    // What the key held: a write within the clock's resolution leaves mod as it was (Windows'
    // clock ticks in milliseconds), so the payload's type and the object a pointer holds tell too.
    std::type_index type = typeid(void);
    const void *id = nullptr;
  };
  void adopt(std::shared_ptr<cvc::slice_source> src, std::shared_ptr<const cvc::volume> vol,
             const std::string &label, int var, int time);
  bool follow(const char *which, followed &f, boost::any &payload);
  void followSources();
  void adoptPoints(const boost::any &payload, const std::string &from);
  void applySession(const cvc::volume_grid_session &s, const std::string &from);
  void publishSession();
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
  void rebuildOverlays();

  std::shared_ptr<cvc::slice_source> m_source;
  std::shared_ptr<const cvc::volume> m_histogramVolume;
  std::string m_label;
  int m_var = 0, m_time = 0; // what the source holds (the editor shows these classes)
  std::string m_scope;
  followed m_followVolume, m_followPoints, m_followSession;
  std::string m_waitingFor; // the status last said the node is waiting for a volume here
  bool m_seeded = false;    // the parameter keys have their defaults
  std::string m_sessionSignature;
  std::array<std::shared_ptr<VolumeSliceNode>, 3> m_slices;
  std::array<std::shared_ptr<GeometryNode>, 3> m_points, m_lines;
  std::shared_ptr<NullGraphicNode> m_surfaces;
  cvc::point_class_editor m_editor;
  cvc::point_class_gestures m_gestures;
  std::unique_ptr<VolumeGridContours> m_contours;
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
