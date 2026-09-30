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

// pyramid.h -- build per-asset LOD ladders (rung 0 = source, coarser rungs are
// decimated/downsampled copies) for the three renderable asset kinds.
//
// This is the BUILDER half the roadmap named cvc::lod::pyramid_builder: the pure
// selection math in <cvc/lod/select.h> consumes a monotone world_error_m[] ladder
// plus per-rung sizes, and this produces exactly that from a source asset. Each
// builder is hermetic (no VTK/GL/I/O): meshes decimate via one progressive
// cvc::simplify_progressive (QEM) pass, volumes downsample via
// cvc::vol_downsample, images box-average via image::resized(box). The
// embarrassingly-parallel work -- independent volume/image rungs, and the
// per-face/per-sample/per-voxel/per-pixel kernels -- fans over the app compute
// pool.
//
// rung 0 is the FINEST and world_error_m[0] == 0; the ladder is monotone
// non-decreasing (forced with a running max), which is the property select_rung
// relies on. LOD is a render proxy only: nav/material/RF paths must keep reading
// rung 0, never a pyramid rung.

#ifndef __CVC_LOD_PYRAMID_H__
#define __CVC_LOD_PYRAMID_H__

#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/image/image.h>
#include <cvc/volume/volume.h>
#include <vector>

namespace cvc {
class thread_pool;

namespace lod {

// How many rungs to build and how fast to coarsen. A builder stops early when the
// next rung would fall below the per-kind floor.
struct pyramid_params {
  int max_rungs = 4;                // coarser rungs to ADD beyond rung 0
  double mesh_ratio = 0.35;         // mesh rung k targets round(src_tris * mesh_ratio^k)
  std::uint64_t mesh_min_tris = 64; // stop adding mesh rungs at/below this triangle count
  bool preserve_boundary = true;    // mesh: pin open borders (see cvc::simplify)
  // mesh: decimate unwelded input as the surface it forms -- touching parts,
  // hard-edge/uv splits, triangle soup -- without opening seams (see
  // cvc::simplify_params::weld_seams); false decimates each connected component
  // on its own, which opens gaps wherever parts touch.
  bool weld_seams = true;
  double seam_epsilon = -1.0; // mesh: weld tolerance, world units (< 0 => cvc::simplify's default)
  unsigned vol_factor = 2;          // volume rung k is src downsampled by vol_factor^k per axis
  std::uint64_t vol_min_dim = 8;    // stop when any axis would fall below this
  unsigned img_factor = 2;          // image rung k is src downsampled by img_factor^k per axis
  int img_min_dim = 4;              // stop when any axis would fall below this
  // Image assets carry no intrinsic world size, so rung k's world_error is
  // img_world_per_texel * img_factor^k. The bake step scales this by the raster's
  // world extent (metres per source texel) when it writes the lod_index.
  double img_world_per_texel = 1.0;
};

struct mesh_pyramid {
  std::vector<geometry> rungs; // rungs[0] = source (finest)
  // Per rung, monotone; world_error_m[0] == 0. Rung k's value is the sampled
  // symmetric Hausdorff distance between the source and that rung (see
  // cvc::sampled_hausdorff), in the mesh's length unit (metres for scene
  // geometry), lifted by the running max. It is exact at the samples -- a
  // LOWER bound on the continuous distance: on feature-aligned (architectural)
  // meshes the worst point is a sampled corner and the two agree, but on a
  // strongly curved surface decimated coarsely the true worst point can fall
  // between samples, and a switch radius derived from this value can then
  // exceed the pixel budget by that margin (by up to ~25% on synthetic bumpy
  // terrain).
  std::vector<double> world_error_m;
};
struct volume_pyramid {
  std::vector<volume> rungs;
  std::vector<double> world_error_m; // ~ the coarser rung's voxel world-size
};
struct image_pyramid {
  std::vector<image> rungs;
  std::vector<double> world_error_m; // ~ the coarser rung's texel world-size
};

// Build a mesh LOD ladder by QEM decimation of the source. One progressive
// collapse pass snapshots every rung on its way to the coarsest target, so rung k
// is bit-identical to cvc::simplify(src) at rung k's target (with
// preserve_boundary, weld_seams and seam_epsilon from `params` and
// simplify_params defaults otherwise: input normals carried) at the cost of the
// coarsest rung alone. A rung that would not have fewer triangles than the one
// before it (the collapse guards stalled short of its target) is left out, so
// the ladder can hold fewer than max_rungs coarse rungs. `pool`, when given,
// fans the per-face setup and each rung's error measurement over the app compute
// workers; the result is identical either way.
mesh_pyramid build_mesh_pyramid(const geometry &src,
                                const pyramid_params &params = pyramid_params(),
                                thread_pool *pool = nullptr);

// Build a volume LOD ladder by integer-factor downsampling.
volume_pyramid build_volume_pyramid(const volume &src,
                                    const pyramid_params &params = pyramid_params(),
                                    thread_pool *pool = nullptr);

// Build an image/texture mip ladder by box-average downsampling.
image_pyramid build_image_pyramid(const image &src, const pyramid_params &params = pyramid_params(),
                                  thread_pool *pool = nullptr);

} // namespace lod
} // namespace cvc

#endif // __CVC_LOD_PYRAMID_H__
