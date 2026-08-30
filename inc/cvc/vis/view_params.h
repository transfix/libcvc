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

// view_params.h -- one view for one frame: its frustum, its projection scalars,
// and the thresholds that decide the distance / small-feature cut.
//
// The projection math is NOT re-derived here. `proj` is a cvc::lod::view_params,
// and this module calls cvc::lod::k_px / screen_radius_px / bound_distance_m for
// the screen-size arithmetic (VISIBILITY-AND-LOD-ROADMAP section 15.4 is
// identical to LSYSTEM-LABORATORY section 8.5 -- the roadmap flags a second
// definition of k_px as risk #13, "silently changes the visible set"). One
// source of truth: cvc::lod. cvc::vis owns the frustum; cvc::lod owns the
// projection.

#ifndef __CVC_VIS_VIEW_PARAMS_H__
#define __CVC_VIS_VIEW_PARAMS_H__

#include <cstdint>
#include <cvc/lod/select.h>
#include <cvc/vis/types.h>

namespace cvc {
namespace vis {

struct view_params {
  frustum f;

  // Eye position, viewport height, fov and z_near live here (bound_distance_m
  // and screen_radius_px read them). Keep proj.eye in sync with the eye the
  // frustum was built from.
  cvc::lod::view_params proj;

  // A proxy is considered only if (scene_view::layer_of(i) & layer_mask) != 0.
  // The default keeps every proxy; a shadow or portal-narrowed view narrows it.
  std::uint32_t layer_mask = ~0u;

  // Distance and small-feature culling are ONE test (section 15.4): a proxy is
  // culled when its projected WIDTH falls below this many pixels. 0 disables it
  // (frustum-only). The roadmap's default is 1.5 px -- enough to clear pebbles
  // and grass at range without touching trees or buildings.
  double min_screen_px = 1.5;
};

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_VIEW_PARAMS_H__
