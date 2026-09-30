/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// simplify.h -- native (VTK-free) triangle-mesh simplification via Quadric Error
// Metrics edge collapse (Garland & Heckbert, SIGGRAPH '97).
//
// This is the mesh-decimation half of the LOD pyramid: rung 0 is the full mesh,
// coarser rungs are simplify()'d copies with a MEASURED world-space error so the
// pure selection math in <cvc/lod/select.h> can place each rung's switch radius
// (see world_error_for_switch_radius). It is hermetic -- no VTK, no GL, no I/O --
// so it runs in pycvc, headless bake tools, and wasm, unlike the VTK
// vtkQuadricDecimation the nav demos currently inline.
//
// Only SURFACE_TRI geometry is simplified. Quads are triangulated on entry;
// tets/hexs/lines and any non-positional attributes not listed below are dropped
// from the result (the coarse rung is a render proxy, never a simulation input --
// the LOD invariant is that selection may not alter correctness, so nav/material
// paths keep reading rung 0). The collapse is a half-edge collapse, so every
// output vertex is an input vertex -- at its own position, with its own uv,
// color and normal attributes (when present and aligned with the vertex array)
// carried through verbatim -- and every output vertex is used by an output
// triangle. Normals are recomputed only when the input has none (or on
// request), so a rung switch does not re-shade the surfaces it did not change.
//
// Unwelded input is decimated as if welded (see simplify_params::weld_seams):
// a triangle soup or a mesh split apart whose attributes agree across the
// splits decimates as the welded mesh it forms, and parts that touch, or faces
// split for hard-edge normals and uv seams, collapse together without opening
// the seam -- while no attribute is ever attached to a position other than its
// own.
//
// The reported error is METRIC: simplify_result::world_error is a sampled,
// symmetric Hausdorff distance between the input and the result, in the mesh's
// own length unit (metres for scene geometry), and it scales linearly with the
// mesh (a 10x larger copy reports a 10x larger error).

#ifndef __CVC_GEOMETRY_SIMPLIFY_H__
#define __CVC_GEOMETRY_SIMPLIFY_H__

#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <vector>

namespace cvc {

class thread_pool;

// How far to simplify, and the fidelity guards. target_tris wins when non-zero;
// otherwise target_ratio (fraction of the input triangle count) is used. The
// collapse loop also stops early if the next cheapest collapse's error proxy
// exceeds max_error (when >= 0), so a ladder can be built either to a triangle
// budget or to an error budget.
struct simplify_params {
  std::uint64_t target_tris = 0; // absolute target triangle count (0 => use target_ratio)
  double target_ratio = 0.5;     // target as a fraction of input tris, in (0,1]
  // Error budget, in world units (< 0 => ignore). The loop stops when the next
  // cheapest collapse's proxy sqrt(cost / weight) exceeds it: the quadric cost
  // normalised by the total weight of the planes it sums, i.e. the weighted RMS
  // distance of the surviving vertex to those planes. It is a stopping rule
  // only; the result's world_error is the Hausdorff measure, which this proxy
  // typically under-reports.
  double max_error = -1.0;
  // Pin open boundaries -- and both sides of every attribute seam (see
  // weld_seams) -- with a constraint quadric: a plane through each such
  // triangle side, perpendicular to its triangle, so borders and seams keep
  // their line (a seam still collapses along itself).
  bool preserve_boundary = true;
  // Strength of that constraint relative to the (area-weighted) face quadrics.
  // Each constrained side's plane is weighted boundary_weight * |edge|^2, so
  // face and constraint terms share units and the collapse order is
  // scale-invariant.
  double boundary_weight = 100.0;
  double max_normal_flip_deg = // reject a collapse that would rotate any incident face normal
      75.0;                    // by more than this (fold-over / self-intersection guard)
  // Recompute smooth per-vertex normals on the result even when the input
  // carries normals. By default the input normals are carried (bit-for-bit) so
  // the coarse rung shades like rung 0; an input without normals always gets
  // freshly computed ones.
  bool recompute_normals = false;
  // Collapse on the welded topology. First, input vertices bit-identical in
  // position AND in every carried attribute (uv, color, and normals unless
  // recompute_normals) become one vertex: a triangle soup, or a mesh split
  // apart, whose attributes agree across the splits is then the welded mesh
  // itself (its vertices numbered by each position's smallest input index,
  // which is what breaks collapse ties). Vertices that still share a position
  // -- they differ in an attribute (a uv seam, a hard-edge normal, a color
  // boundary), or belong to DIFFERENT connected components within the
  // coincidence tolerance of each other (a roof mesh sitting on its walls; see
  // seam_epsilon) -- move as one, and the edges between them are seams. A
  // collapse that drops a seam position maps each of its vertices onto the
  // vertex of the kept position it shares a vanishing triangle with, and is
  // refused unless each of them that a surviving triangle still uses has
  // exactly one such partner: a seam collapses only along itself, each side
  // onto its own side, so it never opens and no uv, color or normal is ever
  // re-attached to another position. (So a mesh whose attributes jump at EVERY
  // vertex -- per-face colors, or per-face normals on a curved surface -- can
  // only give up its open border; with recompute_normals the input normals do
  // not count.) Only vertex-to-vertex contacts are detected: a vertex touching
  // the interior of another part's edge or face (a T-junction) is not welded.
  // Coincidence is transitive, so a chain of vertices each within tolerance of
  // the next welds together. false decimates every connected component on its
  // own, over the input's own vertex indices.
  bool weld_seams = true;
  // Coincidence tolerance across components, world units (bit-identical
  // positions always weld). >= 0 is used as given, for every vertex pair: a
  // tolerance wider than a part's own edges welds that part's vertices
  // together through its neighbours, and its triangles that weld shut leave
  // the result (they still count toward world_error). < 0 (the default) is
  // local: 1e-6 of the bounding-box diagonal, but for each vertex no more than
  // 1e-3 of its shortest incident edge, and two vertices weld only within both
  // their tolerances -- so a large scene cannot weld the vertices of its fine
  // parts (separate parts a few of their own edge lengths apart stay
  // separate), nor weld a part's own triangles shut. A vertex whose incident
  // edges all have zero length does not near-weld.
  double seam_epsilon = -1.0;
};

struct simplify_result {
  std::uint64_t in_tris = 0;
  std::uint64_t out_tris = 0;
  std::uint64_t collapses = 0;
  // Sampled symmetric Hausdorff distance between the input surface and the
  // result, in world units: exactly sampled_hausdorff(input, result), except
  // that zero-area needles -- input triangles with two corners at one
  // position, which the result leaves out (see simplify) -- are not part of the
  // input surface either. Every other input triangle counts, including one
  // that welds shut within the coincidence tolerance. 0 when the result is the
  // input surface. It is exact at the samples, so it is a LOWER bound on the
  // continuous distance (see sampled_hausdorff). Finite for a measurable input:
  // the loop never removes the last triangle. +infinity for one
  // sampled_hausdorff cannot measure (a non-finite coordinate), which is
  // returned unchanged. Feeds cvc::lod's world_error_m ladder directly.
  double world_error = 0.0;
  bool hit_error_limit = false; // true if max_error stopped the loop before the tri target
  // Input vertices that share their collapse position with another input
  // vertex under weld_seams (0 when the input is returned unchanged).
  std::uint64_t seam_vertices = 0;
};

// Returns a simplified copy of `mesh`. `mesh` is not modified. The input is
// returned unchanged when it is already at or below the target or has no
// triangles (world_error 0), when all its triangles weld away (world_error 0),
// and when a vertex a triangle uses has a non-finite coordinate (world_error
// +infinity). Otherwise the result has no NaNs, degenerate triangles or unused
// vertices, never more triangles than the input, and never loses its last
// triangle (a target of 0 decimates as far as the guards allow). Triangles
// whose corners weld together -- zero-area needles (two corners at one
// position), and triangles welded shut within the coincidence tolerance (see
// seam_epsilon) -- are left out of it. No collapse folds two faces onto the
// same three positions or puts a third triangle on an edge that had two (the
// link condition, over the welded topology) -- except that a closed surface
// decimated as far as it goes can end as a two-sided triangle.
//
// `pool` (optional) is used to fan the embarrassingly-parallel work -- per-face
// plane quadrics, the per-vertex quadric gather, the seam search, and the
// Hausdorff measurement -- across the app's compute workers
// (mesh.ctx().computePool()). The greedy collapse loop is inherently serial and
// always runs on the caller. Passing null keeps everything on the caller
// (deterministic and dependency-free); the result is identical either way.
geometry simplify(const geometry &mesh, const simplify_params &params = simplify_params(),
                  simplify_result *out = nullptr, thread_pool *pool = nullptr);

// Progressive mode: ONE greedy collapse pass that snapshots the mesh as the
// triangle count reaches each of `targets` (absolute triangle counts, any order;
// 0 means "as far as the guards allow"). Returns one snapshot per target, in the
// order given, and fills (*out)[k] for snapshot k when `out` is non-null.
//
// The greedy collapse order does not depend on the target -- only the stop test
// reads it -- so snapshot k is BIT-IDENTICAL (points, tris, uvs, colors, normals,
// and every simplify_result field) to simplify(mesh, p) with p = params and
// p.target_tris = targets[k], at the cost of the coarsest target alone.
// params.target_tris / target_ratio are ignored; every other field applies. A
// target at or above the input count returns the input unchanged, as simplify
// does. `pool` is used exactly as in simplify.
std::vector<geometry> simplify_progressive(const geometry &mesh,
                                           const std::vector<std::uint64_t> &targets,
                                           const simplify_params &params = simplify_params(),
                                           std::vector<simplify_result> *out = nullptr,
                                           thread_pool *pool = nullptr);

// Sampled symmetric Hausdorff distance between the triangle surfaces of `a` and
// `b` (quads are fan-triangulated; other elements are ignored) -- the measure
// simplify reports as world_error. Each surface is sampled at every vertex used
// by a triangle, every unique edge midpoint and every face centroid; each sample
// takes its exact point-to-triangle distance to the other surface (accelerated
// by a bounding-volume hierarchy, so wildly mixed triangle sizes -- or a mesh
// that lists its vertices in no spatial order -- cost no more than uniform
// ones), and the result is the largest over both directions. It is exact at
// the samples, so it never over-reports: it is a lower bound on the continuous
// Hausdorff distance, short of it by at most the sample spacing (under half a
// triangle's longest edge). Where the worst point is a vertex -- the corners of
// feature-aligned, architectural meshes -- the two agree; on a strongly curved
// surface decimated coarsely the worst point can fall between samples (on
// synthetic bumpy terrain the dense distance was up to ~1.25x this value;
// denser fixed lattices barely help there). Distances below the round-off
// floor -- 1e-9 of the larger surface's bounding-box diagonal, or 2^-48 (16
// ulps) of its largest coordinate magnitude, whichever is larger -- count as
// zero, so two coincident surfaces measure exactly 0 wherever they sit. 0 when
// both surfaces are empty; +infinity when exactly one is, or when a vertex a
// triangle uses has a non-finite coordinate (or the coordinates are so large
// that a squared distance overflows). `pool` fans the samples and the tree
// builds; the result is identical with or without it.
double sampled_hausdorff(const geometry &a, const geometry &b, thread_pool *pool = nullptr);

} // namespace cvc

#endif // __CVC_GEOMETRY_SIMPLIFY_H__
