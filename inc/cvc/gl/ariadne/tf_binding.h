/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// tf_binding — connects a scene volume node's renderer-agnostic transfer-function MODEL (a
// cvc::tf::model string at `<node>.transfer_function`) to the renderer that draws it, and
// publishes the overlays a `tf_editor` shows beside it. This is what lets one `tf_editor` widget —
// or a script, or another peer — drive every volume renderer through the same key:
//
//   volren    model -> volren.volumes.0.transfer_function.{color,opacity} (raw units over the
//             domain), volren.volumes.0.isosurfaces (one per isocontour; colour = override or the
//             ramp, opacity = override or 1, shininess kept per id), tf_auto_domain
//   volslice  model -> volslice.transfer_function.{color,opacity}, raw units over the domain, so
//             the TF is fixed in data units (the legacy renderer's semantics). volslice.window is
//             only the slice LUT's span: VolSliceNode samples those tables over it, so a window
//             someone moves re-lowers nothing. An explicit domain is also written as the window
//             (the LUT's 256 entries then cover it) while the window is the binding's: unset, or
//             still what the binding last wrote (or the document's window it was seeded with).
//             Once anyone else changes the window the binding yields it and never writes it
//             again; going back to auto clears the window only while it is the binding's.
//   volume    model -> transfer_function.{color,opacity} (the VTK VolumeNode's own keys; it
//             re-reads them live)
//   fe_lab    ONE isocontour -> fe.mesh.isovalue, then a debounced fe.request = mesh (the remesh
//             takes seconds, so it waits for the drag to settle and for a running job to finish).
//             The isocontour is followed by id -- the lowest at attach, the lowest again if that
//             one is deleted -- so another isocontour passing it never swaps which one drives the
//             mesh. A fe.mesh.isovalue written by someone else (fe_controls' slider) moves that
//             same isocontour, so the editor and the slider agree. Isovalues within 1e-9 of the
//             domain span (the position <-> data-value round trip's rounding) are the same
//             isovalue: neither side is rewritten and no remesh runs. A bound past the volume's
//             range is legal (the mesher takes it as bounding nothing) but the model holds
//             positions in [0, 1] only: it shows as its bar pinned at that end, and a pinned bar
//             stands for any bound at or past its end -- the key keeps its true value through
//             attach and every other edit, and nothing re-meshes for it; dragging the bar off the
//             end writes the bar's value. Attaching never re-meshes (auto_mesh or Tet-mesh meshes
//             what the keys hold). A remesh the binding armed is dropped once any mesh job
//             (Tet-mesh, a script's request) starts with the region and bounds the keys hold
//             (fe.last_mesh, fe_mesh_signature in mesh_nodes.h).
//             With fe.mesh.region = interval a SECOND isocontour, followed by its own id, drives
//             fe.mesh.isovalue2 the same way (both directions, the same tolerance, a debounced
//             remesh on either move); the two ids stay distinct (the second adopts the lowest
//             isocontour other than the first's, and the first, re-adopting, skips the second's).
//             The bars may cross: the mesher sorts the bounds. In interval mode the model always
//             holds both: switching to interval with fewer than two isocontours -- or deleting
//             one of only two -- adds one (next_isocontour_id) at that bound's current isovalue
//             and writes the model; with more, the switch adopts one and fe.mesh.isovalue2 takes
//             its value -- unless someone wrote fe.mesh.isovalue2 in the same tick: a bound written
//             with the region keeps its value and moves its bar. A switch never re-meshes by
//             itself (fe_controls' Tet-mesh does, as after a slider move). Switching back to solid
//             stops following the second (it stays in the model). The
//             pair in use is published as <tf>.interval = "<id of isovalue's>,<id of
//             isovalue2's>" ("" in solid mode), so an editor can shade the meshed band.
//             fe_lab loads its source on its lane (an SDF after realize, another file on a
//             `load`), so its binding follows tf_binding_options::volume_source: until a volume
//             arrives there is no range and no overlay; each new one is followed as a VolumeNode's
//             setVolume is, and on its range the followed bars move to the keys' values (the keys
//             are never rewritten for a range, and nothing re-meshes).
//
// and publishes <tf>.range (the `auto` domain: volren / volslice / fe_lab the volume's range,
// volume the node's data_min/data_max), <tf>.histogram (started at attach), and
// <tf>.contour_spectrum / <tf>.contour_tree when a widget asks for them (<tf>.overlay_request).
// Overlays are computed by overlay tasks on the app's state_exec scheduler
// (cvc::tf::schedule_overlays, cvc/tf/overlay_tasks.h) from a copy-on-write copy of the volume,
// with no thread of the binding's own: sliced across frames on the scheduler thread in a
// single-threaded wasm build (or with overlay_threading = cooperative), elsewhere in chunks on the
// app's background pool that the task parks on. So something must pump app.exec_scheduler() for
// overlays to arrive -- every Ariadne Runtime does, in its drain(), while any process is live on
// the scheduler, once per app frame however many Runtimes there are (an AppRuntime drains it
// before the scene's tick); a host with no Runtime pumps it itself each frame
// (cvc::pump_exec_frame, cvc/core/async_lane.h), and a test or a headless tool calls
// wait_overlays(). Each task's results reach the binding through a mutex-guarded mailbox its
// deliver hook fills (on the scheduler's thread); tick() takes them in and publishes them. A
// volume the node replaces (volume: a setVolume, told by the node's shared owner of the volume --
// whatever its range, and never by an address a new volume could reuse) cancels the tasks
// computing from the old one, drops any of their results still waiting, and unpublishes the old
// layers at once (cvc::tf::clear_overlays): none is drawn on the new range, or left beside a layer
// the new volume lacks. Attaching clears the layers already beside the key the same way (a hot
// reload's predecessor's: they need not describe this volume). Overlays are re-binned onto
// <tf>.range when it differs from the range they were computed over, so a layer is always uniform
// over the range published beside it.
//
// <tf>.overlay_status: while a task runs it is the task's -- the binding writes "computing
// <layers>..." once when it starts one, then the task's own progress notes ("contour tree:
// sorting 42%", at most ~10 Hz) replace it and the binding leaves it alone; idle, the binding
// writes why any layer of the current volume is missing: every task's reason since the volume was
// set, as one "unavailable: a; b" note ("" when none), so a later layer's success never erases an
// earlier one's reason (a layer is not retried until the volume changes).
//
// A fully transparent model lowers to EMPTY colour/opacity tables, the renderers' "no transfer
// function" path. Each frame at most one lowering runs, however many writes landed, and only when
// the model string (or, for an `auto` domain, the range) changed: the volren raycast and shadow
// rebuild an edit costs are paid once per frame, not once per key.
//
// Everything a binding writes goes through cvc::state on the render thread (tick, attach,
// wait_overlays); the task's deliver hook only fills the mailbox, which it holds instead of the
// binding (a task can outlive its binding until the cancel). A binding is owned by the
// RealizedScene's custom_ticks, so it dies with the scene; its destructor cancels its live tasks
// (cvc::tf::cancel_overlays: no thread to join), so nothing is published for it afterwards, and
// leaves overlay_status idle instead of a progress note of theirs nobody will finish -- unless a
// newer binding owns the key (a hot reload attaches the new one before the old one dies). A tick()
// after the node is gone does the same, and the binding is a no-op from then on. The
// app must outlive the binding, as it must the node. The tasks carry a scheduler owner tag unique
// to the binding, "tf#<n>:<model key>"; a kill_owner of it stops them, and the layers they were
// computing then stay unpublished ("unavailable: the overlay task was stopped") until the volume
// changes.

#ifndef CVC_GL_ARIADNE_TF_BINDING_H
#define CVC_GL_ARIADNE_TF_BINDING_H

#include <cvc/tf/model.h>
#include <cvc/tf/overlay_tasks.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/volume.h>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace gl {
class GraphicsNode;
namespace ariadne {
struct RealizedScene;

// The renderer a binding drives.
enum class tf_target { volren, volslice, volume, fe_lab };

struct tf_binding_options {
  // The model the key is seeded with when it is empty (the document's transfer_function:, the
  // node's isosurfaces:, or a default).
  cvc::tf::model seed;
  // The renderer already draws `seed` (the realizer configured it from a legacy transfer_function:
  // or from none), so nothing is lowered until the key holds something else: an existing document
  // renders exactly as it did before the model existed. False lowers the seed at attach.
  bool seed_is_applied = false;
  // Per-isocontour volren shininess, by isocontour id (from the document's isosurfaces:).
  std::map<std::string, float> shininess;
  // Overlay computation (thumbnail size, bins, samples).
  cvc::tf::overlay_options overlays;
  // How the overlay tasks run: automatic = threaded (background-pool chunks) where threads exist,
  // else cooperative (sliced across frames on the scheduler thread); see cvc/tf/overlay_tasks.h.
  cvc::tf::overlay_threading overlay_threading = cvc::tf::overlay_threading::automatic;
  // fe_lab: wait this long after the last isovalue change before re-meshing (seconds).
  double remesh_debounce = 0.5;
  // fe_lab: re-mesh automatically when an isocontour moves (else only write fe.mesh.isovalue).
  bool auto_remesh = true;
  // fe_lab: the ids of the isocontours fe.mesh.isovalue and (in interval mode) fe.mesh.isovalue2
  // follow from the start, when the model has them (fe_lab's seed names them iso1 and iso2). Empty
  // or absent: the adoption rule (the lowest isocontour; for the second bound, the lowest other
  // than the first bound's).
  std::string fe_isocontour, fe_isocontour2;
  // fe_lab: the volume the node meshes now, asked every tick; null while it has none (an SDF still
  // being computed: no range, no overlays). A new shared owner is a new volume, followed as a
  // VolumeNode's setVolume is, and the bars the bounds follow move to where the keys' values sit
  // on its range: the keys keep their values and nothing re-meshes. Unset: `vol` for good.
  std::function<std::shared_ptr<const cvc::volume>()> volume_source;
};

class tf_binding;

// Attach a binding for `node` (its model key is node.stateName("transfer_function")) to the
// realized scene: seeds the model, applies it once (unless opt.seed_is_applied), publishes the
// range, starts the histogram, and registers its per-frame tick in out.custom_ticks. The key is
// seeded when it is empty, and also when it still holds the seed an earlier attach in this process
// gave it (a document hot-reload: nobody edited it, so the document's new transfer function wins);
// any other value -- an edit, a restored session -- is kept and lowered. `vol` is the
// volume the node renders (fe_lab: its source volume); it is copied (copy-on-write) for the
// overlay tasks. Never throws; problems are appended to `warnings`. Returns the binding (also
// owned by the tick).
std::shared_ptr<tf_binding> attach_transfer_function(RealizedScene &out,
                                                     std::shared_ptr<GraphicsNode> node,
                                                     tf_target target, const cvc::volume &vol,
                                                     tf_binding_options opt,
                                                     std::vector<std::string> *warnings = nullptr);

// The live binding whose model key is `model_path` (an absolute state path) in `app`, or null —
// for a host or a test that needs the binding a realizer attached (e.g. to wait_overlays()).
std::shared_ptr<tf_binding> find_transfer_function(cvc::app &app, const std::string &model_path);

class tf_binding {
public:
  tf_binding(std::shared_ptr<GraphicsNode> node, tf_target target, const cvc::volume &vol,
             tf_binding_options opt, std::vector<std::string> *warnings = nullptr);
  ~tf_binding();
  tf_binding(const tf_binding &) = delete;
  tf_binding &operator=(const tf_binding &) = delete;

  // Render thread, once per frame: apply a changed model (one lowering per frame however many
  // writes landed), publish the overlays the tasks delivered, start newly requested overlay tasks,
  // and run the fe_lab remesh debounce. Once the node is gone it stops the overlay tasks (as the
  // destructor does) and is a no-op from then on.
  void tick();

  // The absolute state path of the model key (`<node state path>.transfer_function`).
  const std::string &model_path() const { return _model_path; }
  tf_target target() const { return _target; }

  // For tests and headless tools that have no Ariadne Runtime pumping the scheduler: pump
  // app.exec_scheduler() (cvc::tf::wait_overlays, which runs every process on it) until each
  // overlay task this binding has started finishes or `timeout_s` passes in all, then take their
  // results in and publish them at once, as tick() would (a no-op once the node is gone). True
  // when every task finished. Starts nothing: a request tick() has not seen yet waits for it.
  // Render thread only, never from inside a scheduler step.
  bool wait_overlays(double timeout_s = 60.0);

private:
  struct impl;
  std::unique_ptr<impl> _impl;
  std::string _model_path;
  tf_target _target;
};

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_TF_BINDING_H
