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

// visible_set.h -- a cull's output: the surviving proxy ids plus the per-proxy
// numbers the LOD selector and the renderer want, so nothing is recomputed
// downstream (VISIBILITY-AND-LOD-ROADMAP section 6.1: cull and LOD are one
// traversal). The three columns are parallel and grow together.

#ifndef __CVC_VIS_VISIBLE_SET_H__
#define __CVC_VIS_VISIBLE_SET_H__

#include <cstddef>
#include <cvc/vis/types.h>
#include <vector>

namespace cvc {
namespace vis {

struct visible_set {
  std::vector<proxy_id> ids;
  std::vector<float> dist_m;    // bound-nearest eye distance, metres
  std::vector<float> screen_px; // projected radius, pixels

  void clear() noexcept {
    ids.clear();
    dist_m.clear();
    screen_px.clear();
  }

  void reserve(std::size_t n) {
    ids.reserve(n);
    dist_m.reserve(n);
    screen_px.reserve(n);
  }

  void push(proxy_id id, float dist, float px) {
    ids.push_back(id);
    dist_m.push_back(dist);
    screen_px.push_back(px);
  }

  std::size_t size() const noexcept { return ids.size(); }
  bool empty() const noexcept { return ids.empty(); }
};

} // namespace vis
} // namespace cvc

#endif // __CVC_VIS_VISIBLE_SET_H__
