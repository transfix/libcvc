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

// scene_view.h -- the flat, structure-of-arrays proxy table, and the ONLY thing
// a culler ever sees.
//
// A culler never sees a scene graph. It sees columns the caller owns and
// maintains, addressed by proxy_id. That inversion is what every large engine
// converged on (Unreal's PrimitiveBounds, Frostbite's grid blocks, Unity's
// BatchCullingContext) and what makes cvc::vis testable without a renderer:
// a test fills a few vectors and points the columns at them.
//
// scene_view is a NON-OWNING view. The columns must outlive every cull that
// reads them; the renderer adapter fills them once and re-points them each
// frame the topology changes.

#ifndef __CVC_VIS_SCENE_VIEW_H__
#define __CVC_VIS_SCENE_VIEW_H__

#include <cstdint>
#include <cvc/vis/types.h>

namespace cvc {
namespace vis {

struct scene_view {
  std::uint32_t count = 0;

  // Required: one world-space AABB per proxy. Everything else is optional.
  const aabb *bounds = nullptr;

  // Optional per-proxy layer bit. A proxy is visible to a view only if
  // (layer[i] & view.layer_mask) != 0. nullptr => every proxy is in layer 0
  // (bit 0 set), matching a view whose default mask has bit 0 set.
  const std::uint32_t *layer = nullptr;

  // Optional tighter bounding sphere per proxy. When present it drives the
  // distance / small-feature test (a mesh's true sphere is smaller than its
  // box's half-diagonal); when null, bounding_sphere(bounds[i]) is used.
  const sphere *spheres = nullptr;

  std::uint32_t layer_of(proxy_id i) const noexcept { return layer ? layer[i] : 1u; }

  sphere sphere_of(proxy_id i) const noexcept {
    return spheres ? spheres[i] : bounding_sphere(bounds[i]);
  }
};

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_SCENE_VIEW_H__
