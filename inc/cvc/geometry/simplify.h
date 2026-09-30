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
// paths keep reading rung 0). Per-vertex normals are recomputed on the result;
// uv and color attributes, when present and aligned with the vertex array, are
// carried from the surviving endpoint of each collapse.

#ifndef __CVC_GEOMETRY_SIMPLIFY_H__
#define __CVC_GEOMETRY_SIMPLIFY_H__

#include <cstdint>
#include <cvc/geometry/geometry.h>

namespace cvc {

class thread_pool;

// How far to simplify, and the fidelity guards. target_tris wins when non-zero;
// otherwise target_ratio (fraction of the input triangle count) is used. The
// collapse loop also stops early if the next cheapest collapse would introduce a
// world-space error above max_error (when >= 0), so a ladder can be built either
// to a triangle budget or to an error budget.
struct simplify_params {
  std::uint64_t target_tris = 0;  // absolute target triangle count (0 => use target_ratio)
  double target_ratio = 0.5;      // target as a fraction of input tris, in (0,1]
  double max_error = -1.0;        // stop when the next collapse error exceeds this (< 0 => ignore)
  bool preserve_boundary = true;  // pin open boundaries with a perpendicular constraint quadric
  double boundary_weight = 100.0; // strength of that constraint relative to face quadrics
  double max_normal_flip_deg =    // reject a collapse that would rotate any incident face normal
      75.0;                       // by more than this (fold-over / self-intersection guard)
};

struct simplify_result {
  std::uint64_t in_tris = 0;
  std::uint64_t out_tris = 0;
  std::uint64_t collapses = 0;
  // A conservative world-space error estimate for the produced rung: sqrt of the
  // largest quadric error paid by any accepted collapse. Monotone with the amount
  // of simplification, so it feeds cvc::lod's world_error_m ladder directly.
  double world_error = 0.0;
  bool hit_error_limit = false; // true if max_error stopped the loop before the tri target
};

// Returns a simplified copy of `mesh`. `mesh` is not modified. On a mesh that is
// already at or below the target, or that has no triangles, the input is returned
// unchanged (with world_error 0). Never produces NaNs, degenerate triangles, or a
// mesh with more triangles than the input.
//
// `pool` (optional) is used to fan the embarrassingly-parallel setup — per-face
// plane quadrics and the per-vertex quadric gather — across the app's compute
// workers (mesh.ctx().computePool()). The greedy collapse loop is inherently
// serial and always runs on the caller. Passing null keeps everything on the
// caller (deterministic and dependency-free); the result is identical either way.
geometry simplify(const geometry &mesh, const simplify_params &params = simplify_params(),
                  simplify_result *out = nullptr, thread_pool *pool = nullptr);

} // namespace cvc

#endif // __CVC_GEOMETRY_SIMPLIFY_H__
