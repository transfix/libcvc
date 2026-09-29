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

// pyramid.cpp -- per-asset LOD ladder builders.

#include <algorithm>
#include <cmath>
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/simplify.h>
#include <cvc/lod/pyramid.h>
#include <cvc/volume/volume_ops.h>
#include <functional>
#include <vector>

namespace cvc {
namespace lod {
namespace {

// Independent rungs are the embarrassingly-parallel unit: build rung (1+k) into
// slot k. With a pool the slots fan out (each build is self-contained, writing
// only its own slot); without one they run in order. build(k) must touch nothing
// but slot k.
void build_rungs(thread_pool *pool, int n, const std::function<void(int)> &build) {
  if (pool && n > 1)
    pool->parallel_for(n, build);
  else
    for (int k = 0; k < n; ++k)
      build(k);
}

// Append coarse rungs to `out_*`, forcing the world-error ladder monotone with a
// running max (select_rung tolerates a non-monotone ladder but silently, so make
// it explicit here).
template <class T>
void append_monotone(std::vector<T> &out_rungs, std::vector<double> &out_err, std::vector<T> &rungs,
                     const std::vector<double> &errs) {
  double prev = out_err.empty() ? 0.0 : out_err.back();
  for (std::size_t k = 0; k < rungs.size(); ++k) {
    out_rungs.push_back(std::move(rungs[k]));
    prev = std::max(prev, errs[k]);
    out_err.push_back(prev);
  }
}

} // namespace

mesh_pyramid build_mesh_pyramid(const geometry &src, const pyramid_params &params,
                                thread_pool *pool) {
  mesh_pyramid out;
  out.rungs.push_back(src); // rung 0 = source
  out.world_error_m.push_back(0.0);

  const std::uint64_t src_tris = src.num_tris();
  std::vector<std::uint64_t> targets;
  double ratio = 1.0;
  for (int k = 1; k <= params.max_rungs; ++k) {
    ratio *= params.mesh_ratio;
    std::uint64_t t = std::uint64_t(std::llround(double(src_tris) * ratio));
    if (t < params.mesh_min_tris || t == 0 || t >= src_tris)
      break;
    targets.push_back(t);
  }
  const int R = int(targets.size());
  if (R == 0)
    return out;

  std::vector<geometry> rungs(R, src); // overwritten below (copies carry src.ctx())
  std::vector<double> errs(R, 0.0);
  // Fan the rungs; each simplify runs serial internally so the pool is used at
  // exactly one level (rungs OR the per-rung setup), never nested.
  build_rungs(pool, R, [&](int k) {
    simplify_params sp;
    sp.target_tris = targets[k];
    sp.preserve_boundary = params.preserve_boundary;
    simplify_result res;
    rungs[k] = simplify(src, sp, &res, pool && R >= 2 ? nullptr : pool);
    errs[k] = res.world_error;
  });
  append_monotone(out.rungs, out.world_error_m, rungs, errs);
  return out;
}

volume_pyramid build_volume_pyramid(const volume &src, const pyramid_params &params,
                                    thread_pool *pool) {
  volume_pyramid out;
  out.rungs.push_back(src);
  out.world_error_m.push_back(0.0);

  const dimension d0 = src.voxel_dimensions();
  const unsigned f = std::max(2u, params.vol_factor);
  std::vector<unsigned> factors;
  unsigned cur = 1;
  for (int k = 1; k <= params.max_rungs; ++k) {
    cur *= f;
    dimension dk(std::max<std::uint64_t>(1, d0.xdim / cur),
                 std::max<std::uint64_t>(1, d0.ydim / cur),
                 std::max<std::uint64_t>(1, d0.zdim / cur));
    if (dk.xdim < params.vol_min_dim || dk.ydim < params.vol_min_dim ||
        dk.zdim < params.vol_min_dim)
      break;
    factors.push_back(cur);
  }
  const int R = int(factors.size());
  if (R == 0)
    return out;

  std::vector<volume> rungs(R, src);
  std::vector<double> errs(R, 0.0);
  build_rungs(pool, R, [&](int k) {
    rungs[k] = vol_downsample(src, factors[k], factors[k], factors[k]);
    // world error ~ the coarser voxel's world size (max axis span / dim).
    const bounding_box &bb = rungs[k].boundingBox();
    const dimension dk = rungs[k].voxel_dimensions();
    errs[k] = std::max({bb.XSpan(dk), bb.YSpan(dk), bb.ZSpan(dk)});
  });
  append_monotone(out.rungs, out.world_error_m, rungs, errs);
  return out;
}

image_pyramid build_image_pyramid(const image &src, const pyramid_params &params,
                                  thread_pool *pool) {
  image_pyramid out;
  out.rungs.push_back(src);
  out.world_error_m.push_back(0.0);

  const unsigned f = std::max(2u, params.img_factor);
  std::vector<std::pair<int, double>> plan; // (downsample factor, world error)
  unsigned cur = 1;
  for (int k = 1; k <= params.max_rungs; ++k) {
    cur *= f;
    int w = src.width() / int(cur), h = src.height() / int(cur);
    if (w < params.img_min_dim || h < params.img_min_dim)
      break;
    plan.push_back({int(cur), params.img_world_per_texel * double(cur)});
  }
  const int R = int(plan.size());
  if (R == 0)
    return out;

  std::vector<image> rungs(R, src);
  std::vector<double> errs(R, 0.0);
  build_rungs(pool, R, [&](int k) {
    int c = plan[k].first;
    rungs[k] = src.resized(src.width() / c, src.height() / c, image::resize_filter::box);
    errs[k] = plan[k].second;
  });
  append_monotone(out.rungs, out.world_error_m, rungs, errs);
  return out;
}

} // namespace lod
} // namespace cvc
