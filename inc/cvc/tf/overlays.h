/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::tf overlays — the data layers the `tf_editor` widget draws under its curves: a histogram,
// the contour spectrum and the contour tree of the volume (or the field on a mesh) a transfer
// function is applied to.
//
// They are supplied FROM OUTSIDE the widget (roadmap CVCGL-UI-DSL §10.3; ColorTable2's own TODO):
// whoever owns the volume computes them — the cvcGL scene realizer for a scene volume node, or an
// application through compute_overlays() / publish_overlays() — and publishes them as state keys
// beside the model key T:
//
//   T.range             "min,max"       the data range (the model's `auto` domain)
//   T.histogram         [c0,c1,...]     bin counts, uniform over T.range
//   T.contour_spectrum  {"area":[...],"min_volume":[...],"max_volume":[...],"gradient":[...]}
//                                       samples uniform over T.range
//   T.contour_tree      {"vertices":[[x,value],...],"edges":[[a,b],...]}
//                                       value in data units, x the tree layout in [0,1]
//   T.overlay_status    free text while computing / when unavailable ("" when idle)
//   T.overlay_request   CSV of lazily computed overlays a widget wants ("contour_spectrum,
//                       contour_tree"); producers compute those on first request
//
// The histogram is cheap and always computed; the contour spectrum and tree are costly, so a
// producer computes them only once requested.

#ifndef CVC_TF_OVERLAYS_H
#define CVC_TF_OVERLAYS_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class state;
class volume;
struct contour_mesh; // cvc/volume/contour_mesh.h
namespace tf {

// The contour spectrum (Bajaj, Pascucci & Schikore 1997): signature functions of the isovalue,
// each sampled at `size()` isovalues uniform over the data range.
struct spectrum_overlay {
  std::vector<double> area;       // isosurface area
  std::vector<double> min_volume; // volume enclosed below the isovalue
  std::vector<double> max_volume; // volume enclosed above the isovalue
  std::vector<double> gradient;   // mean gradient magnitude over the isosurface
  std::size_t size() const { return area.size(); }
  bool empty() const { return area.empty(); }
};

// A contour tree laid out for drawing: each vertex has a horizontal layout coordinate x in [0,1]
// (ColorTable2 draws the tree with value along the TF axis and x vertically) and its scalar value
// in data units; edges index vertices.
struct tree_overlay {
  struct vertex {
    float x = 0.f;
    double value = 0.0;
  };
  std::vector<vertex> vertices;
  std::vector<std::pair<int, int>> edges;
  bool empty() const { return edges.empty(); }
  // Connected components of the level set at `value` — the arcs that span it, in either
  // orientation: min(v_a, v_b) <= value < max(v_a, v_b), and at the tree's maximum value the arcs
  // ending there (cvc::contour_tree::components_at's rule). 0 outside the tree's value range.
  int components_at(double value) const;
};

// Every overlay for one transfer function. An empty member was not published.
struct overlays {
  bool has_range = false;
  double range_lo = 0.0, range_hi = 1.0;
  std::vector<double> histogram;
  spectrum_overlay spectrum;
  tree_overlay tree;
  std::string status;
};

// Which overlays to compute.
enum overlay_bits : unsigned {
  overlay_histogram = 1u << 0,
  overlay_contour_spectrum = 1u << 1,
  overlay_contour_tree = 1u << 2,
  overlay_all = 0x7u
};

// "histogram" / "contour_spectrum" / "contour_tree" <-> bits (CSV for several).
unsigned overlay_bits_from_csv(const std::string &csv);
std::string overlay_bits_to_csv(unsigned bits);

// --- codecs (the state strings above) ----------------------------------------------------------
// Writers are lossless (the shortest of %.15g..%.17g that reads back exactly; a tree's layout x
// at float precision) and locale-independent.
// Readers accept any JSON whitespace and return false on anything malformed: a range needs two
// finite numbers with lo <= hi (a "[lo,hi]" array is accepted too); histogram bins must be finite
// and >= 0; a spectrum needs all four arrays, of equal length, finite; tree edges must index
// existing vertices. A failed read clears `bins` / `out` and leaves `lo` / `hi` untouched.
std::string range_to_string(double lo, double hi);
bool range_from_string(const std::string &s, double &lo, double &hi);
std::string histogram_to_string(const std::vector<double> &bins);
bool histogram_from_string(const std::string &s, std::vector<double> &bins);
std::string spectrum_to_string(const spectrum_overlay &s);
bool spectrum_from_string(const std::string &s, spectrum_overlay &out);
std::string tree_to_string(const tree_overlay &t);
bool tree_from_string(const std::string &s, tree_overlay &out);

// --- computing them from a volume --------------------------------------------------------------
struct overlay_options {
  int histogram_bins = 256;
  int spectrum_samples = 256;
  // The contour tree and spectrum are computed on the volume resampled so its longest axis is at
  // most this many voxels (0 = full resolution) — VolumeRover2 computed them on a thumbnail too.
  int max_dim = 128;
  // The contour tree is simplified by persistence to at most this many nodes before it is laid
  // out (0 = unsimplified): a noisy volume has a node per local extremum, far more than a widget
  // can draw or a state string should carry.
  int tree_max_nodes = 1500;
  // The range the layers are computed over, in place of the data's own [min, max]: for a producer
  // that publishes its own range and histogram (a FEM node's colour range, say), so the spectrum's
  // samples line up with them. The range phase then costs nothing, and the range comes back as
  // given (a reversed one swapped).
  bool has_range = false;
  double range_lo = 0.0, range_hi = 1.0;
};

// Compute the requested overlays (bits) of `vol`'s first variable over [vol.min(), vol.max()].
// Always fills the range (unless the volume is empty or its range is not finite). The histogram
// bins the full-resolution volume; the spectrum and tree use `opt.max_dim`. A layer that cannot be
// computed (an axis too short for the spectrum, NaN voxels for the tree, a constant volume, ...)
// stays empty and `status` says why ("unavailable: ..."); it never throws. Safe to call off the
// render thread (it reads `vol` through a shallow copy, so not even `vol`'s cached min/max or
// histogram are written; the caller must not mutate the voxels meanwhile). It runs an overlay_job
// (below) to completion.
overlays compute_overlays(const volume &vol, unsigned bits = overlay_all,
                          const overlay_options &opt = {});

// The same for a scalar field on a mesh (cvc/volume/contour_mesh.h: one value per point, linear
// over each tetrahedron, or each triangle of a surface): the range of the finite values, their
// histogram, and the mesh contour spectrum and contour tree (cvc::compute_contour_spectrum and
// cvc::compute_contour_tree on a contour_mesh; max_dim does not apply). On a surface the spectrum's
// area is the contours' length and its volumes are areas.
overlays compute_overlays(const contour_mesh &mesh, unsigned bits = overlay_all,
                          const overlay_options &opt = {});

// compute_overlays, a slice at a time: what an overlay task steps on the app's state_exec scheduler
// (cvc/tf/overlay_tasks.h), so no layer needs a thread of its own.
//
// The phases run in order, each only when requested:
//   range             [vol.min(), vol.max()] exactly as compute_overlays reads it: the volume's set
//                     min/max when both are set, else voxels::calcMinMax's scan, replicated a slice
//                     at a time (1 unit per voxel) on the job's own copy (the caller's cache is
//                     never written);
//   histogram         1 unit per voxel of the full-resolution volume;
//   contour spectrum  a cvc::contour_spectrum_builder (its units);
//   contour tree      a cvc::contour_tree_builder, then contour_tree_simplifier::to_nodes(
//                     tree_max_nodes) (their units), then layout_contour_tree, which is one-shot
//                     (O(nodes) of the simplified tree) and counts as one unit.
// step(budget) does about `budget` units and returns true once every requested layer is done. It
// ends early where phase() changes, since a unit's cost changes with it (a vertex swept costs a
// few hundred voxels scanned): a budget sized for one phase runs into the next by at most one
// piece of the tree builder or simplifier (1024 units; they are stepped in such pieces). Budget 0
// runs to the end (in internal slices, so a cancel still lands). A layer that fails stays empty
// with a note in status, as in
// compute_overlays, which runs a job to completion: both give identical overlays whatever the
// stepping schedule. step never throws (a constructor can, on out of memory).
//
// cancel() is the one thread-safe member: the next step -- or the running one, at its next check
// (between slices of at most `budget`, or 2^16, units) -- returns true with nothing more computed,
// and the builders and the volume copy are let go at once; done() and cancelled() are true from
// the cancel() on. A cancelled job's status() is "cancelled". The job keeps a shallow copy of the
// volume until the last builder has its own (do not write the voxels meanwhile). It may move
// between threads between steps; the other members are for the thread that steps it.
//
// A job on a mesh runs the same phases over its values (the range: one unit per value scanned;
// the histogram: one per value) and the mesh builders (cvc::mesh_contour_spectrum_builder and
// cvc::mesh_contour_tree_builder, their units, with the same phase names). It keeps the shared
// mesh until the last builder has it.
class overlay_job {
public:
  explicit overlay_job(const volume &vol, unsigned bits = overlay_all,
                       const overlay_options &opt = {});
  overlay_job(std::shared_ptr<const contour_mesh> mesh, unsigned bits = overlay_all,
              const overlay_options &opt = {});
  ~overlay_job();
  overlay_job(const overlay_job &) = delete;
  overlay_job &operator=(const overlay_job &) = delete;

  bool step(std::size_t budget);
  bool done() const;
  void cancel();
  bool cancelled() const;
  unsigned bits() const;
  // In [0,1] over every requested layer (weighted by their expected cost), monotone; exactly 1
  // once done (frozen when cancelled).
  double progress() const;
  // The phase the next unit of work belongs to: "range: scanning", "histogram: binning",
  // "contour spectrum: domain" / "resampling" / "integrating", "contour tree: preparing" /
  // "sorting" / "join tree" / "split tree" / "merging" / "simplifying" / "laying out", then
  // "done" (or "cancelled"). A static string, so a caller pacing its budget by time can tell a
  // phase change -- where a unit's cost changes, by up to ~500x -- by pointer.
  const char *phase() const;
  // While running: the phase and how far the current layer is ("contour tree: sorting 42%");
  // once done: the final overlays.status ("" when every layer succeeded); "cancelled".
  std::string status() const;
  // The overlays so far (each layer appears once complete); everything once done().
  const overlays &result() const;
  overlays take(); // moved out; call once, after done()

  // How many overlay_jobs exist right now in this process (for tests and leak checks).
  static std::size_t instances();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// Write `ov` beside the model key `tf` (tf.range, tf.histogram, ... per the table above). Empty
// members are not written, so a later publish of only the spectrum keeps an earlier histogram.
// The status is always written ("" = idle), so a publish clears an earlier "computing ..." note.
// A key whose value would not change is not rewritten (no spurious change signals).
void publish_overlays(state &tf, const overlays &ov);

// Unpublish the layers `bits` names (tf.histogram, tf.contour_spectrum, tf.contour_tree) from
// beside `tf`: each one holding a value is set to "". For a producer whose layers no longer
// describe the volume (it was replaced), since publish_overlays never clears a layer. tf.range and
// tf.overlay_status are left alone; a key that is empty or absent is not written (or created).
void clear_overlays(state &tf, unsigned bits = overlay_all);

// Read whatever overlays are published beside `tf` (missing / malformed keys stay empty). Never
// creates a node; follows transparent links.
overlays read_overlays(state &tf);

} // namespace tf
} // namespace cvc

#endif // CVC_TF_OVERLAYS_H
