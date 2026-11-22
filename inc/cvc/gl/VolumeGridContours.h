/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// VolumeGridContours -- a VolumeGridNode's contours (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §4, P10): curves drawn per object on the grid's planes with the `contours` tool, linear or
// spline, kept on the state tree, exported as contour stacks, and tiled by the contour tiler as a
// sliced task into a surface per object in the 3D view. The node owns one and calls it from its
// tick, picks and requests; the editing itself is cvc::grid_contour_editor and
// cvc::grid_contour_gestures (cvc/volume/volume_grid_contours.h), so it is tested without GL.
//
// KEYS, under the grid node's state path:
//   grid.objects   the objects' metadata as rows a list: binds: the count, then <i>.name,
//                  <i>.color ("r,g,b"), <i>.interpolation (linear | spline), <i>.samples,
//                  <i>.visible, and <i>.curves, <i>.points, <i>.triangles (written);
//                  grid.objects.current, the object a click draws (-1 none). Editing, adding or
//                  removing rows edits the objects (an undo step)
//   grid.contour.* cvc::volume_grid_contour_params: show, control_points, point_size, line_width,
//                  snap, show_3d, show_surfaces, surface_opacity, auto_tile, and source (a state
//                  key, relative to the node's scope, whose data channel holds a contour_stack or
//                  a grid_contours: they replace the grid's each time they change)
//   grid.contour.selected, grid.contour.info, grid.contour.active   published: how many control
//                  points are selected; "3 objects, 12 curves (1 open), 240 points"; the curve
//                  the next click extends ("axon on z = 12, 5 points" or "")
//   grid.tile.*    cvc::tile_params (lengths in object units), and the tile job's keys
//                  (cvc/core/job_keys.h: busy, stoppable, status, progress, phase, stats.work_ms,
//                  pid), label (the orientation and slices being tiled), warnings (how many;
//                  warnings.<i> each). Under grid.tile, not grid.busy: a tile runs beside a
//                  segmentation, each with its own progress
//   grid.request   (the node's) tile | stop_tile (and stop, while a tile runs) | close_curve |
//                  new_curve | delete_curve | clear_object | trace <iso> [first last [step]] (the
//                  curves where the shown view's planes cross iso, added to the current object:
//                  VolumeGridRover's isocontouring; the shown plane alone by default); with the
//                  contours tool also delete_selected, select_all, select_none and undo, on the
//                  contours (otherwise the point classes')
// DATA CHANNEL: grid.curves (std::shared_ptr<const cvc::grid_contours>: every curve as drawn; a
// write from outside, a program's state-data-set or pycvc, replaces them), grid.contours (the z
// planes' closed curves as a std::shared_ptr<const cvc::contour_stack>, splines drawn: what
// contour_lab's source: { node: <grid> }, tiler.tile and the contour formats take),
// grid.contours.xz and grid.contours.zy (the y and x planes'), grid.surfaces.<object> (each tiled
// surface, a cvc::geometry; '.' and white space in a name are '_').
//
// CHILDREN: on each slice, `contours` (the curves on that plane, lines) and `contour_points` (their
// control points, disks, and the selection's rings); in the grid's `surfaces` group, `contours`
// (every plane's curves, for the 3D view) and `tiles` (a surface per object tiled).
#ifndef CVC_GL_VOLUMEGRIDCONTOURS_H
#define CVC_GL_VOLUMEGRIDCONTOURS_H

#include <array>
#include <atomic>
#include <boost/any.hpp>
#include <cstdint>
#include <cvc/state/followed_key.h>
#include <cvc/volume/volume_grid_contours.h>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace cvc {
class job_keys;
template <class R> class result_job;
namespace gl {

class GeometryNode;
class NullGraphicNode;
class VolumeGridNode;

class VolumeGridContours {
public:
  explicit VolumeGridContours(VolumeGridNode &grid);
  ~VolumeGridContours();
  VolumeGridContours(const VolumeGridContours &) = delete;
  VolumeGridContours &operator=(const VolumeGridContours &) = delete;

  // The overlays on each plane and the 3D view's children, once the grid's children exist.
  void ensureChildren();
  // The volume grid the curves are drawn on: a volume adopted. The curves keep their coordinates.
  void setFrame(const cvc::grid_frame &f);

  // Whether a key under grid. (`k`, without the "grid." prefix) is one of the contours': rows,
  // grid.objects.current, grid.contour.*, grid.tile.*. Any thread.
  bool stateChanged(const std::string &k);

  // A pick on a slice while the tool is contours (the node parsed it: plane, (r, u), modifiers).
  void pick(const cvc::contour_pick &p);
  // Drop a gesture under way (the tool changed, the volume reloaded).
  void resetGesture();

  // A grid.request command. True when it is the contours' (and done, or the status says why
  // not); `tool` whether the contours tool is the grid's, so the shared commands (delete_selected,
  // select_all, select_none, undo) are the contours'.
  bool request(const std::string &cmd, const std::string &arg, bool tool);

  // Apply what changed since the last call (keys, rows, the source, the data channel), publish,
  // poll the tile job, and redraw the overlays if they changed. `planes` are the planes drawn (x,
  // y, z). Owner thread, once per frame, after the node's own tick work. True when the drawing
  // changed.
  bool tick(const std::array<std::int64_t, 3> &planes);

  // The contours as they stand, and their replacement (a session applied).
  const cvc::grid_contours &contours() const;
  void setContours(const cvc::grid_contours &c, const std::string &from);

  cvc::grid_contour_editor &editor() { return m_editor; }
  const cvc::grid_contour_editor &editor() const { return m_editor; }

  // A tile running, asked for, or due (auto_tile): for a host waiting on the scene's work.
  bool busy() const;
  // Stop a tile under way, keeping the surfaces the grid had.
  void stopTile();

  // The children (for hosts and tests): a plane's curves and control points, every plane's curves
  // in 3D, and the tiled surfaces' group.
  std::shared_ptr<GeometryNode> curvesOverlay(cvc::slice_axis a) const;
  std::shared_ptr<GeometryNode> pointsOverlay(cvc::slice_axis a) const;
  std::shared_ptr<GeometryNode> curves3d() const { return m_curves3d; }
  std::shared_ptr<NullGraphicNode> tiles() const { return m_tiles; }
  std::size_t overlayBuilds() const { return m_overlayBuilds; }

  // The tile job's label, copied out after each step (any thread).
  struct Label {
    void set(std::string s);
    std::string get() const;

  private:
    mutable std::mutex m;
    std::string text;
  };

private:
  void seed();
  void status(const std::string &s);
  std::string resolve(const std::string &key) const;
  void followSource();
  void adoptPayload(const boost::any &payload, const std::string &from);
  void adoptData();
  void readRows();
  void readKeys();
  void fixCurrent();
  long current() const;
  void publish();
  void publishSelection();
  void writeTriangleRows();
  void trace(const std::string &arg);
  void launchTile();
  void pollTile();
  void applyTile(cvc::grid_tile_result r);
  void clearSurfaces();
  void syncSurfaces();
  void rebuildOverlays(const std::array<std::int64_t, 3> &planes);
  double pickRadius(cvc::slice_axis a) const;

  VolumeGridNode &m_grid;
  cvc::grid_contour_editor m_editor;
  cvc::grid_contour_gestures m_gestures;
  cvc::volume_grid_contour_params m_params;
  bool m_haveFrame = false;
  bool m_seeded = false;
  bool m_newCurve = false;    // new_curve: the next click starts a curve
  cvc::followed_key m_follow; // grid.contour.source
  boost::any m_pending; // a stack that came before the frame: converted once the frame is known
  std::string m_pendingFrom;
  bool m_tileDue = false; // auto_tile asked for a tile on the next tick
  std::array<std::shared_ptr<GeometryNode>, 3> m_curves, m_points;
  std::shared_ptr<GeometryNode> m_curves3d;
  std::shared_ptr<NullGraphicNode> m_tiles;
  cvc::contour_pick_outcome m_band;

  std::unique_ptr<cvc::job_keys> m_keys;
  std::shared_ptr<cvc::result_job<cvc::grid_tile_result>> m_job;
  std::shared_ptr<Label> m_label;
  std::vector<double> m_jobColor;
  int m_task = -1;
  std::map<std::string, std::string> m_surfaceKeys; // object name -> its key under grid.surfaces
  std::map<std::string, std::weak_ptr<GeometryNode>> m_surfaceNodes;
  std::map<std::string, std::size_t> m_surfaceTris;
  std::string m_surfaceSignature;

  std::atomic<std::uint64_t> m_keysVersion{1}, m_rowsVersion{1};
  std::uint64_t m_keysSeen = 0, m_rowsSeen = 0;
  std::string m_autoCurrent;
  std::shared_ptr<const cvc::grid_contours> m_published;
  std::uint64_t m_versionShown = 0, m_selectionShown = 0;
  std::string m_overlaySignature;
  std::size_t m_overlayBuilds = 0;
};

} // namespace gl
} // namespace cvc

#endif // CVC_GL_VOLUMEGRIDCONTOURS_H
