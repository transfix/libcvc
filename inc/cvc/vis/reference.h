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

// reference.h -- the reference culler (the oracle), and the conservativeness
// checker that validates every faster culler against it.
//
// The oracle exists BEFORE anything is validated against it (VISIBILITY-AND-LOD-
// ROADMAP section 13, PR 1). It is deliberately the slowest and simplest
// implementation: a flat sweep of every proxy through
//   1. the layer mask,
//   2. the exact frustum test (aabb_vs_frustum), and
//   3. the distance / small-feature test (one test, section 15.4), reusing the
//      cvc::lod projection primitives.
// No acceleration structure, no SoA, no stages. Its only jobs are to be
// obviously correct and to define what "visible" means, so a quadtree culler or
// a SIMD kernel can be checked for EQUALITY (same set) and, more importantly,
// for CONSERVATIVENESS (it never drops a proxy the oracle keeps).

#ifndef __CVC_VIS_REFERENCE_H__
#define __CVC_VIS_REFERENCE_H__

#include <cstddef>
#include <cvc/vis/scene_view.h>
#include <cvc/vis/types.h>
#include <cvc/vis/view_params.h>
#include <cvc/vis/visible_set.h>
#include <vector>

namespace cvc {
namespace vis {

// True iff proxy `i` survives every stage of `view`. Shared by the oracle and
// the checker so there is exactly one definition of "visible".
bool reference_visible(const scene_view &s, const view_params &view, proxy_id i, float *out_dist_m,
                       float *out_screen_px) noexcept;

// Brute-force cull. Fills `out` (cleared first) with the surviving ids, in
// ascending id order, and their bound-nearest distance and projected radius.
void reference_cull(const scene_view &s, const view_params &view, visible_set &out);

// How many proxies the oracle keeps but `candidate` dropped -- i.e. the count of
// proxies a faster culler WRONGLY culled. This must be 0 for any correct culler;
// a non-zero result is a hole in the frame. `candidate` need not be sorted.
//
// (The reverse -- proxies the candidate keeps but the oracle drops -- is benign
// over-draw and is NOT a violation; conservative cullers are allowed to draw a
// few extra objects. Use reference_cull + a set compare when you want exact
// equality instead.)
std::size_t conservativeness_violations(const std::vector<proxy_id> &candidate, const scene_view &s,
                                        const view_params &view);

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_REFERENCE_H__
