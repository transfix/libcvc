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
// surviving vertex keeps its input position exactly; its uv, color and normal
// attributes, when present and aligned with the vertex array, are carried
// through verbatim. Normals are recomputed only when the input has none (or on
// request), so a rung switch does not re-shade the surfaces it did not change.
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
  bool preserve_boundary = true; // pin open boundaries with a perpendicular constraint quadric
  // Strength of that constraint relative to the (area-weighted) face quadrics.
  // Each boundary edge's plane is weighted boundary_weight * |edge|^2, so face
  // and boundary terms share units and the collapse order is scale-invariant.
  double boundary_weight = 100.0;
  double max_normal_flip_deg = // reject a collapse that would rotate any incident face normal
      75.0;                    // by more than this (fold-over / self-intersection guard)
  // Recompute smooth per-vertex normals on the result even when the input
  // carries normals. By default the input normals are carried (bit-for-bit) so
  // the coarse rung shades like rung 0; an input without normals always gets
  // freshly computed ones.
  bool recompute_normals = false;
  // Never collapse a vertex whose position coincides (within seam_epsilon) with
  // a vertex of a DIFFERENT connected component. Unwelded parts that touch -- a
  // roof mesh sitting on its walls, faces split for hard-edge normals -- are then
  // decimated without opening gaps along the seam. Such a vertex may still
  // absorb its neighbours (it never moves). Only vertex-to-vertex contacts are
  // detected: a vertex touching the interior of another part's edge or face (a
  // T-junction) is not locked. A locked seam keeps every vertex, so a finely
  // subdivided seam is never thinned, which bounds how far such a mesh decimates.
  bool lock_component_seams = true;
  double seam_epsilon = -1.0; // coincidence tolerance, world units (< 0 => 1e-6 * bbox diagonal)
};

struct simplify_result {
  std::uint64_t in_tris = 0;
  std::uint64_t out_tris = 0;
  std::uint64_t collapses = 0;
  // Sampled symmetric Hausdorff distance between the input surface and the
  // result, in world units (see sampled_hausdorff for the sampling). 0 when
  // nothing was collapsed. Feeds cvc::lod's world_error_m ladder directly.
  double world_error = 0.0;
  bool hit_error_limit = false;      // true if max_error stopped the loop before the tri target
  std::uint64_t locked_vertices = 0; // vertices pinned by lock_component_seams
};

// Returns a simplified copy of `mesh`. `mesh` is not modified. On a mesh that is
// already at or below the target, or that has no triangles, the input is returned
// unchanged (with world_error 0). Never produces NaNs, degenerate triangles, or a
// mesh with more triangles than the input.
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
// takes its exact point-to-triangle distance to the other surface (uniform-grid
// accelerated), and the result is the largest over both directions. It is exact
// at the samples and can under-report the continuous distance by at most the
// sample spacing (under half a triangle's longest edge). Distances below 1e-9 of
// the larger surface's bounding-box diagonal are round-off and count as zero, so
// two coincident surfaces measure exactly 0. 0 when both surfaces are empty,
// +infinity when exactly one is. `pool` fans the samples; the result is
// identical with or without it.
double sampled_hausdorff(const geometry &a, const geometry &b, thread_pool *pool = nullptr);

} // namespace cvc

#endif // __CVC_GEOMETRY_SIMPLIFY_H__
