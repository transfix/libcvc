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

#include <algorithm>
#include <cvc/vis/reference.h>

namespace cvc {
namespace vis {

bool reference_visible(const scene_view &s, const view_params &view, proxy_id i, float *out_dist_m,
                       float *out_screen_px) noexcept {
  // 1. Layer mask.
  if ((s.layer_of(i) & view.layer_mask) == 0u)
    return false;

  // 2. Exact frustum test. A flat sweep never propagates a mask, so pass the
  // full one each time.
  if (aabb_vs_frustum(s.bounds[i], view.f) == frustum_test::outside)
    return false;

  // 3. Distance / small-feature test == projected width below min_screen_px,
  // reusing the cvc::lod projection primitives (one source of truth for k_px).
  const sphere sp = s.sphere_of(i);
  const double centre[3] = {sp.c[0], sp.c[1], sp.c[2]};
  const double dist = cvc::lod::bound_distance_m(centre, sp.r, view.proj);
  const double screen_px = cvc::lod::screen_radius_px(sp.r, dist, view.proj);

  if (view.min_screen_px > 0.0 && 2.0 * screen_px < view.min_screen_px)
    return false;

  if (out_dist_m)
    *out_dist_m = static_cast<float>(dist);
  if (out_screen_px)
    *out_screen_px = static_cast<float>(screen_px);
  return true;
}

void reference_cull(const scene_view &s, const view_params &view, visible_set &out) {
  out.clear();
  out.reserve(s.count);
  for (proxy_id i = 0; i < s.count; ++i) {
    float dist = 0.0f, px = 0.0f;
    if (reference_visible(s, view, i, &dist, &px))
      out.push(i, dist, px);
  }
  // Ascending id order is guaranteed by the loop; callers rely on it for a
  // cheap set compare against another culler's sorted output.
}

std::size_t conservativeness_violations(const std::vector<proxy_id> &candidate, const scene_view &s,
                                        const view_params &view) {
  // Mark which ids the candidate kept, then count oracle-visible proxies the
  // candidate failed to keep. O(count) plus one pass over the candidate.
  std::vector<char> kept(s.count, 0);
  for (proxy_id id : candidate)
    if (id < s.count)
      kept[id] = 1;

  std::size_t violations = 0;
  for (proxy_id i = 0; i < s.count; ++i)
    if (!kept[i] && reference_visible(s, view, i, nullptr, nullptr))
      ++violations;
  return violations;
}

} // namespace vis
} // namespace cvc
