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

// select.h -- single-process level-of-detail selection math.
//
// Pure arithmetic: no VTK, no GL, no I/O, and no allocation on the hot path.
// Two consumers share it and neither may fork it:
//
//   * nav_city_swarm / nav_fog_ghost -- the Austin bundle, whose merged
//     buildings actor is 978,242 triangles and whose agent glyphs rasterize a
//     full mesh at 2 px (docs/roadmap/RENDER_PERF_ROADMAP.md, phases 1-2);
//   * the L-System Laboratory, which selects terrain chunks, vegetation bands,
//     rocks and building shells against the same knobs
//     (docs/roadmap/LSYSTEM-LABORATORY-ROADMAP.md section 8.5).
//
// The vocabulary is deliberately the modernization roadmap's own, so the
// networked streaming layer (volrover3 section 22.1: cvc::lod::pyramid_builder,
// lod_index, the CvcLod service) slots in later without renaming anything a
// demo already uses. `desired_pixel_error` is section 22.1.6's knob, and the
// promotion priority is section 22.1.6's "projected screen area / distance^2".
// This header claims ONLY the selection math. Nothing here streams, caches,
// evicts, decimates or draws.
//
// --- Two conventions that are easy to get backwards ------------------------
//
// 1. RUNG INDEX 0 IS THE FINEST. Rungs count downward in quality exactly as
//    the roadmap's ladders are written (T0..T4, A0..A4, B0..B2, R0..R2), so
//    `world_error_m` is non-decreasing in the rung index. `candidate::min_rung`
//    is therefore the *minimum detail* -- the COARSEST rung a group may fall
//    back to -- and is numerically >= `desired_rung`. "Promote" means move
//    toward finer, i.e. DECREASE the index. The field names are the published
//    section 8.5 API; the direction is this paragraph.
//
// 2. SCREEN ERROR FALLS WITH DISTANCE. Section 8.5 describes switch_radius_m
//    as "the distance at which world_error_m first exceeds
//    desired_pixel_error", which reads backwards: err_px = k_px * err_world /
//    dist shrinks as the camera retreats. The returned radius is the crossover
//    -- the distance at or beyond which the rung's error has fallen TO
//    `desired_pixel_error`, and hence the distance at which the rung becomes
//    affordable. The arithmetic is unchanged; only the sentence was wrong.
//
// --- Where the two sibling roadmaps disagreed, and what shipped ------------
//
// PR #249 landed two independent selectors. They are reconciled here rather
// than implemented twice; cvc::vis (VISIBILITY-AND-LOD-ROADMAP section 6.1)
// should call this header, not re-derive it.
//
//   distance     6.1 wins: bound-NEAREST, max(z_near, |eye-c| - r), never the
//                centre. bound_distance_m() computes it; select_rung takes the
//                result, so a caller that has only a centre distance still
//                compiles.
//   error budget 8.5 / 22.1.6 win on the NAME (`desired_pixel_error`) and on
//                the default (2.0 px). 6.1's tau = 4 px is a preset, not a
//                contradiction -- see quality_preset.
//   hysteresis   8.5 wins: ONE-SIDED. The coarsen boundary is widened by
//                `hysteresis`; the refine boundary is not. 6.1's symmetric
//                +/-8% band has the same anti-oscillation property but drifts
//                the effective error budget in both directions.
//   monotonicity 6.1 wins, and it is load-bearing: a ladder whose error is not
//                monotonic in the rung index can skip a rung or oscillate
//                across a shell. select_rung forces it with a running max and
//                ladder_is_monotonic reports it.
//   fades        8.5 wins: a time constant off world dt (section 22.4.3's rule
//                that every smoothing constant is 1 - exp(-dt/tau), never a
//                per-frame ratio), not 6.1's 4-6 frame count. 6.1's
//                `deterministic` kill-switch is kept: tau <= 0 fades instantly,
//                so headless and batch renders stay frame-exact.
//
// One deliberate departure from the published section 8.5 signature: `solve`
// takes its candidates by CONST reference, not the mutable one the roadmap
// prints. The selector reads the scene and writes only the plan, so "LOD may
// never alter simulation correctness" is enforced by the type system rather
// than by review. Nothing needed the mutability.
//
// LOD may never alter simulation correctness (volrover3 section 22, :7734).
// Nothing in this header is reachable from a nav or material export path, and
// a world exported with the camera at the coarsest rung must be byte-identical
// to one exported at the finest.

#ifndef __CVC_LOD_SELECT_H__
#define __CVC_LOD_SELECT_H__

#include <cstddef>
#include <cstdint>
#include <vector>

namespace cvc {
namespace lod {

// --- View ------------------------------------------------------------------

// One camera, one frame. Deterministic: select_rung and solve are pure
// functions of their arguments and this struct.
struct view_params {
  double eye[3] = {0.0, 0.0, 0.0};
  double viewport_h_px = 800.0;
  double tan_half_fov = 0.41421356237309503; // tan(22.5 deg) -- 45 deg vertical
  double desired_pixel_error = 2.0;          // 22.1.6's knob; 8.5's default
  double hysteresis = 0.15;                  // widens the coarsen radius only
  double z_near = 0.1;                       // floor for bound-nearest distance
};

// Quality presets. `desired_pixel_error` per RENDER_PERF_ROADMAP phase 1;
// `pristine` is also section 22.1.6's documented 1.0 default, and `aggressive`
// doubles as the wasm profile (section 8.5's wasm column: 3.5 px, 0.20
// hysteresis).
enum class quality_preset { pristine, balanced, aggressive };

view_params preset_view(quality_preset) noexcept;

// h / (2 * tan(fov/2)) -- pixels per unit of (world size / distance). Both
// roadmaps call this k_px and both agree on it.
double k_px(const view_params &) noexcept;

// Screen-space error of a rung whose world-space error is `world_error_m`,
// seen at `dist_m`. Section 8.5: world_error * h / (2 * dist * tan(fov/2)).
double screen_error_px(double world_error_m, double dist_m, const view_params &) noexcept;

// Apparent radius in pixels of a bounding sphere of radius `radius_m`.
// Section 6.1's screen_px = k_px * r / d. Representation changes (mesh to
// impostor) switch on THIS, not on the error metric: a billboard has no
// meaningful geometric error.
double screen_radius_px(double radius_m, double dist_m, const view_params &) noexcept;

// The crossover distance at which `world_error_m` costs exactly
// `desired_pixel_error` on screen. At or beyond it the rung is affordable.
// Returns 0.0 for a zero or negative error (a rung with no error is affordable
// everywhere) and +inf if `desired_pixel_error` is non-positive.
double switch_radius_m(double world_error_m, const view_params &) noexcept;

// The inverse of switch_radius_m: the world-space error a rung must have for
// its crossover to land at `radius_m`. Section 8.4 publishes its ladders as
// switch RADII ("B0 <= 200 m, B1 <= 900 m", "T1 <= 480 m") rather than as
// Hausdorff errors, and hand-converting them at each call site is how the two
// drift apart.
//
// Author ONCE against the REFERENCE view the radii were published for -- 800 px
// at 2.0 px error, i.e. preset_view(quality_preset::balanced) -- and store the
// resulting errors in the ladder. Selection then scales every boundary together
// with the runtime `desired_pixel_error` and viewport height, which is what
// makes the presets do anything. Re-deriving the ladder at the runtime preset
// instead pins all boundaries to the published radii and silently neuters the
// preset, and it is an easy mistake to make because it still looks correct.
double world_error_for_switch_radius(double radius_m, const view_params &) noexcept;

// The distance beyond which a bounding sphere of radius `radius_m` projects to a
// WIDTH narrower than `impostor_px` -- the mesh -> impostor REPRESENTATION
// switch. Section 6.1 makes this a width threshold, not an error one, "because a
// billboard has no meaningful geometric error", so it is a separate crossover
// from switch_radius_m and the two are used together: pick the rung by error out
// to here, then draw a camera-facing card past it. Width is 2*radius, so the
// crossover is where 2 * k_px * r / d == impostor_px; the roadmap's default
// `impostor_px` is 32.
//
// This is the second and last selection decision the module owns, so a consumer
// (the agent LOD of RENDER_PERF phase 2, the A3 vegetation card of section 8.4)
// never re-derives the 2x. An impostor CAN instead be encoded as the coarsest
// rung of the ladder with an authored error via world_error_for_switch_radius();
// use that when the impostor competes for the triangle budget like any rung, and
// use this when the switch is purely a screen-size decision.
//
// Returns 0 for a non-positive radius (a zero-width object is always an
// impostor) and +inf if `impostor_px` is non-positive or the camera is
// degenerate (the switch never fires).
double impostor_switch_radius_m(double radius_m, double impostor_px, const view_params &) noexcept;

// Bound-NEAREST distance from the eye to a bounding sphere, floored at
// `z_near` (VISIBILITY-AND-LOD-ROADMAP section 6.1). Using the centre instead
// over-refines big groups and under-refines small ones; using the raw nearest
// point divides by ~0 when the camera is inside the bound.
double bound_distance_m(const double centre[3], double radius_m, const view_params &) noexcept;

// --- Ladders ---------------------------------------------------------------

// True when world_error_m[0..nrungs) is non-decreasing -- the property that
// guarantees exactly one transition per distance shell. select_rung forces it
// with a running max regardless; call this at bake or load time to find the
// ladder that needs fixing rather than silently rounding it off.
bool ladder_is_monotonic(const double *world_error_m, int nrungs) noexcept;

// Hysteretic rung choice. `current` is last frame's rung, or a negative value
// for "no history" (which selects without hysteresis -- the correct behaviour
// for a group entering the frustum, and what a headless render passes).
//
// Coarsening requires clearing the widened boundary r[L] * (1 + hysteresis);
// refining requires falling back inside the plain boundary r[L]. Between the
// two the rung is held, so a camera parked on a boundary cannot oscillate.
//
// Returns 0 when nrungs <= 0 or world_error_m is null.
int select_rung(double dist_m, const double *world_error_m, int nrungs, int current,
                const view_params &) noexcept;

// --- Budget ----------------------------------------------------------------

// A client's per-frame ceiling. `max_tris` sits under volrover3 section
// 20.13.7's 4 M maxTrianglesVisible client default with headroom for the
// shadow re-render, which rasterizes the caster set a second time.
struct budget {
  std::uint32_t max_props = 48; // measured by cvcgl_prop_sweep, not asserted (8.7)
  std::uint64_t max_tris = 2500000;
  std::uint64_t max_bytes = 700ull << 20;
};

// Platform profiles from section 8.5's threshold table.
enum class budget_profile { desktop_large, desktop_default, wasm };

budget preset_budget(budget_profile) noexcept;

// One group competing for the budget: a terrain tile, a scatter cell, a
// building batch, or an agent LOD bucket. `tris_per_rung` and `bytes_per_rung`
// are caller-owned arrays of `nrungs` entries; the candidate does not own them
// and they must outlive the solve.
struct candidate {
  std::uint32_t group_id = 0; // tile or scatter-cell morton id -- the tie-break key
  int nrungs = 0;
  int desired_rung = 0;        // what distance alone wants (finest affordable)
  int min_rung = 0;            // COARSEST fallback allowed; >= desired_rung
  double projected_area = 0.0; // solid-angle proxy; any unit, used consistently
  double dist_m = 0.0;
  const std::uint64_t *tris_per_rung = nullptr;
  const std::uint64_t *bytes_per_rung = nullptr;

  // Prop slots this group consumes while it draws. The two roadmaps draw the
  // same tile grid two different ways and both are legitimate, so the cost is
  // a field rather than an assumption:
  //
  //   1  RENDER_PERF_ROADMAP phase 1 -- one GeometryNode per non-empty tile.
  //      Austin's 4 km span at 128 m tiles has ~337 occupied tiles, so the
  //      prop ceiling binds long before the triangle ceiling does. That is a
  //      real constraint on that shape, not a bug, and the solve reports it.
  //   0  LSYSTEM-LABORATORY-ROADMAP 8.6 -- the group is drawn through a shared
  //      fixed-capacity merged actor, so 1024 tiles cost the ~14 actors of the
  //      capacity table and the tile count stops mattering.
  //
  // A group that does not draw at its selected rung costs nothing either way.
  std::uint32_t props_when_drawn = 1;
};

// Which ceiling stopped the solve.
enum class bound : int { none = 0, props = 1, triangles = 2, bytes = 3 };

struct plan {
  std::vector<int> rung; // one entry per candidate, in input order
  std::uint32_t props = 0;
  std::uint64_t tris = 0;
  std::uint64_t bytes = 0;
  bound binding = bound::none; // the ceiling that refused the top-priority promotion
};

// Section 22.1.6's "projected screen area / distance^2", regularized so a group
// at the eye does not divide by zero and so the knee sits at 1 km:
//
//     score = projected_area / (1 + (dist_m / 1000)^2)
//
// Section 8.5 publishes the regularized form; section 22.1.6 publishes the
// ideal one. They agree to within a constant beyond ~1 km, which is where the
// ordering matters.
double priority(const candidate &) noexcept;

// A group occupies a prop slot only if it actually draws: the coarsest rungs of
// several ladders are folded into another actor (A4 vegetation becomes a
// terrain splat tint, HLOD sectors become terrain albedo) and cost 0 triangles
// and 0 props. So tris_per_rung[rung] == 0 means "not drawn", and that is the
// whole rule.
bool draws(const candidate &, int rung) noexcept;

// Greedy: every candidate starts at `min_rung`; candidates are visited in
// descending priority (ties broken by group_id, then input order, so the result
// is a pure function of the inputs on every platform) and promoted one rung at
// a time toward `desired_rung` while all three ceilings hold. A candidate that
// cannot take its next step is left where it is and the solve moves on -- a
// cheaper group may still fit -- so the packing is tight rather than truncated
// at the first refusal.
//
// `binding` records the ceiling that refused the FIRST (highest-priority)
// promotion, which is the one worth showing in the LOD overlay's budget bars.
// It stays `none` when every candidate reached `desired_rung`.
//
// If the all-coarsest baseline already exceeds a ceiling, no promotion happens,
// `binding` names that ceiling, and the returned totals report the overflow
// honestly rather than clamping it away.
plan solve(const std::vector<candidate> &, const budget &);

// Same solve, reusing out.rung's storage and the solver's own scratch, so a
// per-frame call allocates nothing after the first. `out` is fully overwritten.
class solver {
public:
  void solve(const std::vector<candidate> &, const budget &, plan &out);

  // Pre-size the scratch for `n` candidates.
  void reserve(std::size_t n);

private:
  std::vector<std::uint32_t> order_;
};

// --- Fades -----------------------------------------------------------------

// Cross-fade weight as a TIME CONSTANT off world dt (volrover3 section 22.4.3),
// never as a per-frame ratio. tau_s <= 0 returns 1.0 -- the instant switch that
// section 6.5's `deterministic` mode requires so headless and batch renders are
// frame-exact.
double fade_alpha(double elapsed_s, double tau_s) noexcept;

} // namespace lod
} // namespace cvc

#endif // __CVC_LOD_SELECT_H__
