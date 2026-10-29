/*
  Copyright 2003-2026 The University of Texas at Austin

        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::compute_contour_tree — the contour tree of a scalar volume, ported from VolumeRover's
// `contourtree` library (HeightField.h / computeCT, UT Austin 2003, LGPL-2.1), which VolumeRover2's
// ColorTable2 drew under its transfer function.
//
// The contour tree (Carr, Snoeyink & Axen, "Computing contour trees in all dimensions", 2003)
// tracks how the connected components of the level sets {f = w} appear, merge, split and vanish as
// w sweeps the value range: leaves are local minima and maxima, interior nodes are the saddles
// where components join or split, and every arc spans the values over which one component exists.
// So the number of arcs spanning w is the number of connected isosurface components at w.
//
// The field is the piecewise-linear interpolant on the original's tetrahedral mesh: each cube split
// into 6 tetrahedra along the (+1,+1,-1) diagonal, i.e. the 14-neighbour stencil ±x, ±y, ±z,
// ±(1,1,0), ±(0,1,-1), ±(1,0,-1), ±(1,1,-1) (kept exactly, so trees match VolumeRover's). Vertices
// are ordered by value with ties broken by linear voxel index (simulation of simplicity). Join and
// split trees come from two union-find sweeps and are merged with Carr's leaf condition (up-degree
// in the join tree + down-degree in the split tree == 1) — the original re-queued only same-tree
// leaves, which builds wrong trees on 2D data and thin slabs; this port fixes it. Flat arrays, 23
// bytes per voxel at the peak (the sweeps; the vertex sort's scratch, 16 bytes per voxel for 32-bit
// types and 24 for 64-bit ones, becomes their arrays or is given back before they start),
// 64-bit-safe sizes (fewer than 2^32 voxels), any voxel type, no globals, no threads, re-entrant.
// Everything runs as a resumable builder (below); the one-shot functions run it to the end.

#ifndef CVC_VOLUME_CONTOUR_TREE_H
#define CVC_VOLUME_CONTOUR_TREE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/core/exception.h>
#include <cvc/core/types.h>
#include <functional>
#include <memory>
#include <vector>

namespace cvc {
class voxels;
struct contour_mesh; // cvc/volume/contour_mesh.h

CVC_DEF_EXCEPTION(contour_tree_error);

struct contour_tree {
  // minimum/maximum: a leaf; join: components merge going DOWN (up-degree >= 2); split: one
  // component splits going up (down-degree >= 2); join_split: both (common on 2D data).
  enum class node_kind : std::uint8_t { minimum, maximum, join, split, join_split };
  struct node {
    uint64 voxel = 0;   // linear voxel index, i + j*XDim + k*XDim*YDim, of the volume built on
    double value = 0.0; // the scalar value there, in data units
    node_kind kind = node_kind::minimum;
  };
  struct arc {
    std::uint32_t hi = 0, lo = 0; // node indices; nodes[hi] is above nodes[lo] in (value, voxel)
  };

  std::array<uint64, 3> dims{{0, 0, 0}};
  double value_min = 0.0, value_max = 0.0; // the field's range
  // Sorted by (value, voxel), so nodes.front() is the global minimum and nodes.back() the global
  // maximum, and an arc's hi is its larger index. On a line a maximum has two arcs down (and a
  // minimum two up); with two or more axes every extremum is a leaf.
  std::vector<node> nodes;
  std::vector<arc> arcs;

  bool empty() const { return arcs.empty(); }
  // Connected components of the level set {f = w}: the arcs with lo.value <= w < hi.value (at the
  // global maximum, the arcs ending there from below; arcs lying inside a plateau at the top are
  // pieces of one component and do not count). Exact between node values; at a node's own value
  // it counts the components just above it.
  std::size_t components_at(double w) const;
  // The indices of those arcs.
  std::vector<std::uint32_t> arcs_at(double w) const;
};

struct contour_tree_options {
  // Resample (trilinear) so no axis exceeds this many voxels before building (0 = full resolution).
  // Each longer axis becomes exactly max_dim, shorter ones are kept; samples sit where
  // voxels::resize puts them, and integer types truncate as it does. dims and node voxels then
  // refer to the resampled grid. Resampling never makes a NaN: a tap of weight 0 is left out, an
  // infinity outweighs every finite value it is blended with (so each sample it is a tap of takes
  // it), +inf against -inf takes the nearer tap along that axis (the lower one at the midpoint),
  // and finite doubles too far apart to subtract are blended without overflowing.
  uint64 max_dim = 0;
  // VolumeRover2 parity: build on q = trunc(255*(v-min)/(max-min)) (the original took uchar data
  // only). Node values are still reported in data units (the bin's lower edge).
  bool quantize_u8 = false;
  // Called with progress in [0,1] (monotone, 1.0 last); return false to cancel (compute throws
  // contour_tree_error). A builder calls it once at the end of every step that does work;
  // compute_contour_tree steps in slices of 2^16 units when it is set, so it is called a few
  // times per million voxels.
  std::function<bool(double)> progress;
};

// The contour tree of `vol`. Throws contour_tree_error for an empty volume, an Undefined voxel
// type, 2^32 or more voxels, a NaN value, or on cancel. A constant volume yields one node, no arcs.
// Infinite values are accepted, at full resolution and resampled alike: +inf orders above and -inf
// below every finite value (equal ones tie, broken by voxel index like any plateau), so they become
// extrema with node values (and value_min / value_max) of +-inf. Only quantize_u8 needs a finite
// range; it throws otherwise.
contour_tree compute_contour_tree(const voxels &vol, const contour_tree_options &opt = {});

// The same over a raw x-fastest float field of nx*ny*nz values (any axis may be 1).
contour_tree compute_contour_tree(const float *values, uint64 nx, uint64 ny, uint64 nz,
                                  const contour_tree_options &opt = {});

// Prune the tree by persistence: repeatedly remove the leaf arc (leaf to its saddle) with the
// smallest value span while that span is below `min_persistence` (a fraction of the value range),
// suppressing saddles that become regular. Never removes the last arc.
// A leaf goes only while its saddle keeps another arc on the leaf's side (Carr, Snoeyink & Axen's
// simplification), so every prune cancels an extremum-saddle pair and the result is still a
// contour tree; such a leaf always exists while two or more arcs remain. Kept nodes keep their
// order, voxel and value; kinds are recomputed. dims and the value range are copied.
// A span is the values' distance: 0 between equal values (two infinities of one sign included, as
// in a plateau at +inf) and +inf from an infinity to anything else. An infinite span is never
// below the threshold, so with an infinite value range any positive min_persistence prunes every
// finite span and keeps the infinite ones. A NaN node value makes the tree malformed (it throws).
contour_tree simplify_contour_tree(const contour_tree &t, double min_persistence);
// The same, pruning until at most `max_nodes` nodes remain (keeps the most persistent features;
// leaves of infinite span go too, after every finite one, equal spans in node order). A prune may
// also suppress its saddle, so the result can land one under the budget; it never goes below the
// last arc's two nodes.
contour_tree simplify_contour_tree_to(const contour_tree &t, std::size_t max_nodes);

// A drawing layout: the horizontal coordinate in [0,1] of every node (the value is the other axis),
// VolumeRover2's look (the root at the global maximum, each child offset by
// (f_child - f_parent) / tan(theta), theta in [60,120) degrees) with a SEEDED generator, so a tree
// always lays out the same on every platform. Siblings may overlap, as in the original. A tree of
// fewer than two nodes (or with every x equal) gets 0.5 throughout. One-shot and O(nodes + arcs):
// meant for a simplified tree (a few thousand nodes), not the raw tree of a noisy volume.
std::vector<float> layout_contour_tree(const contour_tree &t, std::uint32_t seed = 1);

// --- resumable building -------------------------------------------------------------------------
//
// The work of compute_contour_tree and simplify_contour_tree(_to), done a slice at a time, for a
// caller that must not block: an Ariadne overlay task (cvc/tf/overlay_tasks.h) steps it a few
// milliseconds per frame on the app's state_exec scheduler, which is how single-threaded wasm
// computes a tree with no thread at all. step(budget) does at most `budget` units of work and
// returns true once the result is ready; budget 0 runs to the end. The one-shot functions above
// run these to completion, so both forms build identical trees whatever the stepping schedule.
//
// A unit is one item of whatever loop is running: a voxel scanned (the NaN check, the range, the
// quantizing), a resampling tap or resampled voxel, a histogram bin cleared or summed, a key
// counted or moved in one radix pass, a vertex swept (its <= 14 neighbours and their union-find
// finds, at most 32 hops each under union by rank), a vertex flagged or peeled, one hop along a
// removed chain or along an arc, a node or arc rebuilt, one element of a new array written once in
// order before a loop writes it in random order (so its page faults are spread evenly rather than
// thousands landing in one step), one KiB of a spent array given back to the system, and, in the
// simplifier, a node or arc of its setup, rebuild or release, or one heap level of a pop (a pop
// costs 1 + log2 of the heap's size). Every phase is sliced this way -- the input checks, NaN scan,
// resampling, range and quantizing ("preparing"), the vertex sort (an LSD radix sort, which orders
// by (value, voxel) exactly: one counting pass when the data span at most 2^16 values, else up to
// 3 passes for 32-bit keys and 6 for 64-bit ones), both sweeps, the merge, the pruning -- and the
// two loops whose single iteration has no bound (the combine's lazy splice and the walk along an
// arc) stop mid-walk and resume. So a step never does more than `budget` units: no overrun at all
// in units. What runs outside the count is O(1) per phase change: allocating an array (no page is
// touched; working arrays of 1 MiB or more come from VirtualAlloc / mmap, so that a spent one goes
// back 64 KiB at a time instead of in one O(n) free; on wasm, where free() is O(1), they come
// from operator new).
//
// Units are not equal in time; on 128^3 volumes on a desktop x64 (a loaded one), a step of 4096
// units took about 10 us preparing, 20-60 us sorting, 0.3 ms merging and 1.3-2.5 ms in a sweep, so
// the sweeps (~0.5 us per vertex) set the pace: size a time slice by them, or by the time the
// last steps took. The slowest step seen at 4096 units was 3.5-5 ms of thread time (a sweep); a
// simplifier pruning step took at most 1.4 ms. A run is about 15n units for n voxels: preparing n
// (+ n for a NaN scan of a float input, resampling and quantizing each), sorting (3 + passes) n,
// the join tree 2n, the split tree n, merging 4n plus O(n) hops, and ~n/40 to give memory back.
// Stepping 4096 units at a time cost +0.1% (noise) to +4% (smooth data) over budget 0 in thread
// time on 128^3 floats (medians of 7; the machine's own noise was about +-5%).
//
// A builder made from voxels keeps a shallow copy (the buffer is shared copy-on-write, so do not
// write it through data_ptr() while the builder runs; the voxels' cvc::app must outlive the
// builder) and drops it as soon as it no longer reads the input (which frees the buffer only if
// the caller let go of it); the float* form borrows `values` until it is done. All real work
// happens in step(): constructing one only stores its arguments. Destroying one mid-way cancels
// it and frees what it holds at once (as any container would; a failed builder keeps its memory
// until then). Not thread-safe, but it may move between threads between steps. A moved-from
// builder or simplifier is empty: done() is true, step() returns true, progress() is 1, phase()
// is "done" and take() throws.

class contour_tree_builder {
public:
  explicit contour_tree_builder(const voxels &vol, const contour_tree_options &opt = {});
  contour_tree_builder(const float *values, uint64 nx, uint64 ny, uint64 nz,
                       const contour_tree_options &opt = {});
  ~contour_tree_builder();
  contour_tree_builder(contour_tree_builder &&) noexcept;
  contour_tree_builder &operator=(contour_tree_builder &&) noexcept;

  // Throws contour_tree_error where compute_contour_tree would (the input checks on the first
  // step, NaN once the scan reaches it, "out of memory", or "cancelled"); after a throw the
  // builder is done, phase() is "failed" and every later step (and take) rethrows. opt.progress
  // is called once at the end of every step that does work and may cancel, even on the step that
  // finishes. Once done, step() returns true without calling it.
  bool step(std::size_t budget);
  bool done() const;
  double progress() const; // in [0,1], monotone; exactly 1 once the tree is ready
  // "preparing", "sorting", "join tree", "split tree", "merging" or "done" (the phase the next
  // step works on; a constant volume goes straight from "preparing" to "done"), or "failed".
  const char *phase() const;
  // The tree, once step() has returned true (moved out: call once; a second call, or a call
  // before it is ready, throws contour_tree_error).
  contour_tree take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// simplify_contour_tree / simplify_contour_tree_to, a slice at a time: the setup (a node or arc
// per unit: checks, degrees, adjacency lists, the initial heap), the pruning (1 + log2(heap size)
// units per heap pop), the rebuild (a node or arc per unit) and giving the working arrays back.
// The factories only store the tree (pass it by move); the simplifier keeps it until destroyed.
class contour_tree_simplifier {
public:
  static contour_tree_simplifier by_persistence(contour_tree t, double min_persistence);
  static contour_tree_simplifier to_nodes(contour_tree t, std::size_t max_nodes);
  ~contour_tree_simplifier();
  contour_tree_simplifier(contour_tree_simplifier &&) noexcept;
  contour_tree_simplifier &operator=(contour_tree_simplifier &&) noexcept;

  // Throws contour_tree_error where simplify_contour_tree would (a malformed tree, "out of
  // memory"); then it is done and every later step (and take) rethrows.
  bool step(std::size_t budget);
  bool done() const;
  double progress() const; // in [0,1], monotone; exactly 1 once done
  // "preparing", "pruning", "rebuilding" or "done" (straight away when there is nothing to prune),
  // or "failed".
  const char *phase() const;
  contour_tree take(); // once done; moved out, call once

private:
  contour_tree_simplifier();
  struct impl;
  std::unique_ptr<impl> _impl;
};

// --- on a simplicial mesh (cvc/volume/contour_mesh.h) -----------------------------------------
//
// The contour tree of a piecewise-linear field on a mesh: the same algorithm over the graph of the
// elements' edges (the tetrahedra's when the mesh has any, else the triangles') instead of the
// grid's stencil, with vertices ordered by (value, point index). A node's `voxel` is its point
// index and dims is {points, 1, 1}. A point no element uses is left out, and so is a point with a
// NaN value, with its edges (the field is not defined there). On a grid's own
// tetrahedra (each cube split into 6 along the (+1,+1,-1) diagonal, points numbered as voxels) it
// builds exactly compute_contour_tree's tree. A mesh in several pieces gets a tree per piece.
//
// The level sets of a field on a domain that is not simply connected (a surface with holes, a
// solid with a tunnel) form a Reeb graph with loops, which no tree holds. Where the merge finds
// no leaf it takes the highest remaining vertex as one, so the result is still a tree, but its
// component counts are only approximate across such a loop.
//
// max_dim and quantize_u8 do not apply; progress does. Throws contour_tree_error for a missing
// mesh, a value count other than the point count, an element naming a point that does not exist,
// 2^32 or more points, a mesh with neither tetrahedra nor triangles, no edge left without a NaN
// end, or on cancel. Infinite values order as on a grid. A field constant over the used points
// yields one node and no arcs.
contour_tree compute_contour_tree(const contour_mesh &mesh, const contour_tree_options &opt = {});

// compute_contour_tree on a mesh, a slice at a time, like contour_tree_builder (the same phase
// names). A unit is an element counted or listed, a point checked, a vertex moved by one pass of
// the sort (a bottom-up merge sort, so it can stop anywhere), a vertex swept plus one per
// neighbour it looks at, a leaf merged (a forced one also scans the remaining vertices once), and
// one per vertex, arc or hop of the reduction. A step may run past its budget by one vertex's
// neighbours. The builder holds the shared mesh until it is done; destroying it mid-way cancels.
class mesh_contour_tree_builder {
public:
  explicit mesh_contour_tree_builder(std::shared_ptr<const contour_mesh> mesh,
                                     const contour_tree_options &opt = {});
  ~mesh_contour_tree_builder();
  mesh_contour_tree_builder(mesh_contour_tree_builder &&) noexcept;
  mesh_contour_tree_builder &operator=(mesh_contour_tree_builder &&) noexcept;

  bool step(std::size_t budget);
  bool done() const;
  double progress() const;
  const char *phase() const;
  contour_tree take();

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace cvc

#endif // CVC_VOLUME_CONTOUR_TREE_H
