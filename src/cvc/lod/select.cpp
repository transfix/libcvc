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
#include <cmath>
#include <cvc/lod/select.h>
#include <limits>

namespace cvc {
namespace lod {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Clamp a rung index into [0, nrungs).
inline int clamp_rung(int r, int nrungs) noexcept {
  if (r < 0)
    return 0;
  if (r >= nrungs)
    return nrungs - 1;
  return r;
}

} // namespace

// --- View ------------------------------------------------------------------

view_params preset_view(quality_preset q) noexcept {
  view_params v;
  switch (q) {
  case quality_preset::pristine:
    v.desired_pixel_error = 1.0;
    v.hysteresis = 0.15;
    break;
  case quality_preset::balanced:
    v.desired_pixel_error = 2.0;
    v.hysteresis = 0.15;
    break;
  case quality_preset::aggressive:
    v.desired_pixel_error = 3.5;
    v.hysteresis = 0.20;
    break;
  }
  return v;
}

double k_px(const view_params &vp) noexcept {
  const double t = vp.tan_half_fov;
  if (!(t > 0.0))
    return 0.0;
  return vp.viewport_h_px / (2.0 * t);
}

double screen_error_px(double world_error_m, double dist_m, const view_params &vp) noexcept {
  if (!(dist_m > 0.0))
    return kInf;
  return world_error_m * k_px(vp) / dist_m;
}

double screen_radius_px(double radius_m, double dist_m, const view_params &vp) noexcept {
  if (!(dist_m > 0.0))
    return kInf;
  return radius_m * k_px(vp) / dist_m;
}

double switch_radius_m(double world_error_m, const view_params &vp) noexcept {
  // A rung with no world error is affordable at any distance, including zero.
  if (!(world_error_m > 0.0))
    return 0.0;
  // A non-positive error budget can never be met by a rung that has error.
  if (!(vp.desired_pixel_error > 0.0))
    return kInf;
  return world_error_m * k_px(vp) / vp.desired_pixel_error;
}

double world_error_for_switch_radius(double radius_m, const view_params &vp) noexcept {
  const double k = k_px(vp);
  if (!(k > 0.0))
    return 0.0;
  if (!(radius_m > 0.0))
    return 0.0;
  return radius_m * vp.desired_pixel_error / k;
}

double impostor_switch_radius_m(double radius_m, double impostor_px,
                                const view_params &vp) noexcept {
  // A zero-width object has already collapsed to a billboard at any distance.
  if (!(radius_m > 0.0))
    return 0.0;
  // A non-positive width threshold means "never switch".
  if (!(impostor_px > 0.0))
    return kInf;
  const double k = k_px(vp);
  // A degenerate camera projects nothing; do not silently turn the whole world
  // into impostors -- treat it as "never switch", matching switch_radius_m's
  // refusal to declare a rung affordable under a non-positive error budget.
  if (!(k > 0.0))
    return kInf;
  // width_px = 2 * k_px * r / d; solve width_px == impostor_px for d.
  return 2.0 * k * radius_m / impostor_px;
}

double bound_distance_m(const double centre[3], double radius_m, const view_params &vp) noexcept {
  const double dx = centre[0] - vp.eye[0];
  const double dy = centre[1] - vp.eye[1];
  const double dz = centre[2] - vp.eye[2];
  const double d = std::sqrt(dx * dx + dy * dy + dz * dz) - radius_m;
  const double floor_m = vp.z_near > 0.0 ? vp.z_near : 0.0;
  return d > floor_m ? d : floor_m;
}

// --- Ladders ---------------------------------------------------------------

bool ladder_is_monotonic(const double *world_error_m, int nrungs) noexcept {
  if (world_error_m == nullptr || nrungs <= 1)
    return true;
  for (int i = 1; i < nrungs; ++i)
    if (world_error_m[i] < world_error_m[i - 1])
      return false;
  return true;
}

int select_rung(double dist_m, const double *world_error_m, int nrungs, int current,
                const view_params &vp) noexcept {
  if (world_error_m == nullptr || nrungs <= 0)
    return 0;
  if (nrungs == 1)
    return 0;

  const double widen = 1.0 + (vp.hysteresis > 0.0 ? vp.hysteresis : 0.0);

  // The coarsest rung affordable at the plain boundary, and the coarsest
  // affordable at the widened one. widened_target <= plain_target always, and
  // [widened_target, plain_target] is the hold band that kills the oscillation.
  //
  // `err` is a running max over the ladder, so a non-monotonic ladder still
  // yields exactly one transition per distance shell instead of skipping or
  // oscillating across a rung (VISIBILITY-AND-LOD-ROADMAP 6.1).
  // ladder_is_monotonic() is how a caller finds out that this happened.
  int plain_target = 0;
  int widened_target = 0;
  double err = world_error_m[0];
  for (int L = 1; L < nrungs; ++L) {
    err = std::max(err, world_error_m[L]);
    const double r = switch_radius_m(err, vp);
    if (dist_m >= r)
      plain_target = L;
    if (dist_m >= r * widen)
      widened_target = L;
  }

  if (current < 0)
    return plain_target; // no history: pick outright, no hysteresis
  const int cur = clamp_rung(current, nrungs);
  if (cur < widened_target)
    return widened_target; // clear of the widened boundary -- step out to coarser
  if (cur > plain_target)
    return plain_target; // back inside the plain boundary -- step in to finer
  return cur;            // inside the band -- hold
}

// --- Budget ----------------------------------------------------------------

budget preset_budget(budget_profile p) noexcept {
  budget b;
  switch (p) {
  case budget_profile::desktop_large:
    b.max_props = 48;
    b.max_tris = 2500000;
    b.max_bytes = 700ull << 20;
    break;
  case budget_profile::desktop_default:
    b.max_props = 48;
    b.max_tris = 1600000;
    b.max_bytes = 380ull << 20;
    break;
  case budget_profile::wasm:
    b.max_props = 24;
    b.max_tris = 700000;
    b.max_bytes = 90ull << 20;
    break;
  }
  return b;
}

double priority(const candidate &c) noexcept {
  const double km = c.dist_m / 1000.0;
  return c.projected_area / (1.0 + km * km);
}

bool draws(const candidate &c, int rung) noexcept {
  if (c.tris_per_rung == nullptr || c.nrungs <= 0)
    return false;
  if (rung < 0 || rung >= c.nrungs)
    return false;
  return c.tris_per_rung[rung] > 0;
}

namespace {

// A candidate's rung bounds after the 0-is-finest convention is enforced:
// `finest` is what distance wants, `coarsest` is the fallback floor, and
// finest <= coarsest. A caller that fills the published section 8.5 fields
// backwards gets a sane clamp rather than an empty promotion range.
struct rung_span {
  int finest;
  int coarsest;
};

rung_span span_of(const candidate &c) noexcept {
  const int n = c.nrungs > 0 ? c.nrungs : 1;
  int finest = clamp_rung(c.desired_rung, n);
  int coarsest = clamp_rung(c.min_rung, n);
  if (coarsest < finest)
    std::swap(coarsest, finest);
  return {finest, coarsest};
}

// Both cost lookups bounds-check against nrungs rather than trusting the
// clamp upstream: a candidate with nrungs == 0 and a non-null cost array is a
// caller bug, and reading [0] out of it is the kind of bug that only shows up
// under a sanitizer on someone else's machine.
bool in_ladder(const candidate &c, int rung) noexcept { return rung >= 0 && rung < c.nrungs; }

std::uint64_t tris_at(const candidate &c, int rung) noexcept {
  return (c.tris_per_rung != nullptr && in_ladder(c, rung)) ? c.tris_per_rung[rung] : 0;
}

std::uint64_t bytes_at(const candidate &c, int rung) noexcept {
  return (c.bytes_per_rung != nullptr && in_ladder(c, rung)) ? c.bytes_per_rung[rung] : 0;
}

std::uint32_t props_at(const candidate &c, int rung) noexcept {
  return draws(c, rung) ? c.props_when_drawn : 0u;
}

// The core solve, shared by the free function and the reusable solver. `order`
// is scratch: it is resized and overwritten, never read on entry.
void solve_impl(const std::vector<candidate> &cands, const budget &b, plan &out,
                std::vector<std::uint32_t> &order) {
  const std::size_t n = cands.size();
  out.rung.assign(n, 0);
  out.props = 0;
  out.tris = 0;
  out.bytes = 0;
  out.binding = bound::none;
  if (n == 0)
    return;

  // Everything starts at its coarsest allowed rung.
  for (std::size_t i = 0; i < n; ++i) {
    const rung_span s = span_of(cands[i]);
    out.rung[i] = s.coarsest;
    out.props += props_at(cands[i], s.coarsest);
    out.tris += tris_at(cands[i], s.coarsest);
    out.bytes += bytes_at(cands[i], s.coarsest);
  }

  // An over-budget baseline is reported, not clamped: the caller needs to know
  // that even the coarsest world does not fit, and which ceiling says so.
  if (out.props > b.max_props) {
    out.binding = bound::props;
    return;
  }
  if (out.tris > b.max_tris) {
    out.binding = bound::triangles;
    return;
  }
  if (out.bytes > b.max_bytes) {
    out.binding = bound::bytes;
    return;
  }

  order.resize(n);
  for (std::size_t i = 0; i < n; ++i)
    order[i] = static_cast<std::uint32_t>(i);

  // Descending priority, with group_id and then input index as tie-breaks, so
  // the plan is a pure function of the inputs on every platform and every
  // std::sort implementation.
  std::sort(order.begin(), order.end(), [&cands](std::uint32_t a, std::uint32_t c) {
    const double pa = priority(cands[a]);
    const double pc = priority(cands[c]);
    if (pa != pc)
      return pa > pc;
    if (cands[a].group_id != cands[c].group_id)
      return cands[a].group_id < cands[c].group_id;
    return a < c;
  });

  for (std::uint32_t idx : order) {
    const candidate &c = cands[idx];
    const rung_span s = span_of(c);
    while (out.rung[idx] > s.finest) {
      const int from = out.rung[idx];
      const int to = from - 1;

      // Signed, then narrowed: out.props is unsigned and a stray underflow here
      // would read as an astronomically large prop count and refuse forever.
      const std::int64_t props_delta =
          static_cast<std::int64_t>(props_at(c, to)) - static_cast<std::int64_t>(props_at(c, from));
      const std::int64_t props_signed = static_cast<std::int64_t>(out.props) + props_delta;
      const std::uint32_t props_next =
          props_signed > 0 ? static_cast<std::uint32_t>(props_signed) : 0u;
      const std::uint64_t tris_next = out.tris - tris_at(c, from) + tris_at(c, to);
      const std::uint64_t bytes_next = out.bytes - bytes_at(c, from) + bytes_at(c, to);

      bound refused = bound::none;
      if (props_next > b.max_props)
        refused = bound::props;
      else if (tris_next > b.max_tris)
        refused = bound::triangles;
      else if (bytes_next > b.max_bytes)
        refused = bound::bytes;

      if (refused != bound::none) {
        // The first refusal in priority order is the one the overlay shows.
        if (out.binding == bound::none)
          out.binding = refused;
        break;
      }

      out.rung[idx] = to;
      out.props = props_next;
      out.tris = tris_next;
      out.bytes = bytes_next;
    }
  }
}

} // namespace

plan solve(const std::vector<candidate> &cands, const budget &b) {
  plan p;
  std::vector<std::uint32_t> order;
  solve_impl(cands, b, p, order);
  return p;
}

void solver::solve(const std::vector<candidate> &cands, const budget &b, plan &out) {
  solve_impl(cands, b, out, order_);
}

void solver::reserve(std::size_t n) { order_.reserve(n); }

// --- Fades -----------------------------------------------------------------

double fade_alpha(double elapsed_s, double tau_s) noexcept {
  if (!(tau_s > 0.0))
    return 1.0;
  if (!(elapsed_s > 0.0))
    return 0.0;
  const double a = 1.0 - std::exp(-elapsed_s / tau_s);
  return a < 1.0 ? a : 1.0;
}

} // namespace lod
} // namespace cvc
