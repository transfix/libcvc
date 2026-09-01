/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick.

  VolMagick is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  VolMagick is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// grid_nav.h — belief-space grid navigation kernels for GRL-SNAM.
//
// A bit-identical C++ port of the hot path in GRL-SNAM's pure-numpy planner
// (grl_snam/planner.py) and SDF builder (sdf_nav.py). These are the four
// functions that dominate the real-time tick loop — an exact Euclidean
// distance transform, 8-connected A*, Bresenham line-of-sight and string-pull
// simplification — plus the footprint→SDF field they feed. See
// GRL-SNAM/docs/PERFORMANCE.md for the measured 43-77x per-kernel speedups and
// the fidelity guardrails this port is written against:
//
//   * the EDT keeps its parabola-envelope arithmetic in float64 (a float32
//     tie-break flips the gradient on medial axes);
//   * A* reproduces Python heapq's (f, g, node, parent) pop order, so it
//     returns the byte-identical path among equal-cost ties;
//   * g is an exact sum of 1.0 and sqrt(2) terms and h is libm hypot — this
//     header's translation unit MUST be compiled without -ffast-math and
//     without -ffp-contract=fast or the traces stop reproducing;
//   * nearest_free preserves planner._nearest_free's top-left-biased scan
//     order verbatim (it is not actually a nearest search, and "cleaning it
//     up" moves start/goal cells and changes every route).
//
// Everything operates on raw row-major rasters (pointer + rows + cols) so the
// core has no dependency on the rest of libcvc and can be threaded across
// independent agents (PERFORMANCE.md stage 4). The pycvc bindings
// (bindings/pycvc/pycvc_nav.i) marshal numpy arrays into these signatures.

#ifndef __CVC_NAV_GRID_NAV_H__
#define __CVC_NAV_GRID_NAV_H__

#include <cstdint>
#include <utility>
#include <vector>

namespace cvc {
class thread_pool; // injected fork-join executor
namespace nav {

// ─── Exact Euclidean distance transform ─────────────────────────────────────

// Squared Euclidean distance (in grid cells²) from each cell to the nearest set
// cell of `mask` (row-major, rows×cols, nonzero = set), via the separable
// Felzenszwalb & Huttenlocher lower-envelope transform. float64 throughout;
// bit-identical to sdf_nav._edt2. Cells with no set cell in the grid receive
// ~1e20. Returns a row-major rows×cols buffer.
std::vector<double> edt2_squared(const std::uint8_t *mask, int rows, int cols);

// Footprint occupancy → normalized signed distance field + unit outward
// normals, mirroring sdf_nav.build_sdf. `occ` is row-major rows×cols with
// nonzero = inside a building; `phi` > 0 OUTSIDE buildings and 0 at walls;
// (normal_x, normal_y) is the unit gradient of phi — the direction of
// increasing clearance, i.e. (dphi/dcol, dphi/drow). All three grids are
// float32, row-major rows×cols. bounds = (min_x, min_y, max_x, max_y) in world
// units; `scale` maps world → the normalized regime.
struct sdf_field {
  int rows = 0;
  int cols = 0;
  std::vector<float> phi;      // signed distance, normalized
  std::vector<float> normal_x; // unit gradient, column (x) component
  std::vector<float> normal_y; // unit gradient, row (y) component
};

sdf_field build_sdf(const std::uint8_t *occ, int rows, int cols, double min_x, double min_y,
                    double max_x, double max_y, double scale);

// ─── Belief-space grid navigation (planner.py) ──────────────────────────────

// Binary dilation by `cells` 4-connected steps (planner.inflate). `occ` is
// row-major rows×cols with nonzero = blocked; returns a row-major rows×cols
// uint8 grid of 0/1. `cells` <= 0 returns a plain 0/1 copy of `occ`.
std::vector<std::uint8_t> inflate(const std::uint8_t *occ, int rows, int cols, int cells);

// Bresenham line-of-sight (planner._line_of_sight): true iff no blocked cell
// lies on the inclusive segment from (ar, ac) to (br, bc). Endpoints are
// (row, col). Callers guarantee endpoints are in bounds, exactly as the Python
// reference does.
bool line_of_sight(const std::uint8_t *occ, int rows, int cols, int ar, int ac, int br, int bc);

// Snap (r, c) to the nearest free cell using planner._nearest_free's exact
// (top-left-biased) scan order and `max_radius` bound. Returns {row, col};
// {-1, -1} means None (no free cell within the radius). (r, c) is first clamped
// into the grid, matching the Python reference.
std::pair<int, int> nearest_free(const std::uint8_t *occ, int rows, int cols, int r, int c,
                                 int max_radius = 12);

// Per-cell A* surcharge that biases a route toward standoff from obstacles --
// the `cost` argument astar() takes. Float-equivalent to sdf_nav.clearance_cost:
//
//     clearance = sqrt(edt2_squared(occ))            // cells to nearest blocked
//     cost      = gamma * max(0, d_safe - clearance) // grid-step units
//
// A cell with >= `d_safe` cells of clearance pays nothing; the price ramps
// linearly to `gamma * d_safe` at a wall. Because the surcharge is ADDITIVE and
// never forbids a cell, a corridor narrower than `d_safe` throughout degrades to
// the shortest path rather than stranding the agent.
//
// `d_safe` is in CELLS, so the same number is a different distance on every map
// -- the procedural city's cells are ~2 m, a 1024-cell Austin raster's ~0.6 m.
// State the standoff you want in metres and divide by the cell size.
//
// Measured (procedural city squad, route-guided reach, n=20 x 5 seeds): the
// defaults lift reach ~0.80 -> ~0.90 and give visibly smoother, wall-avoiding
// paths. Two costs to know before switching a demo to it: the standoff route is
// LONGER, so it only pays when the tick budget can finish it, and a per-cell
// cost makes each agent's search cost-specific, which disables A* batching
// (measured on the Python squad: 119.9 -> 163.3 ms/tick at n=8, grid 192).
std::vector<double> clearance_cost(const std::uint8_t *occ, int rows, int cols, double d_safe = 6.0,
                                   double gamma = 1.5);

// 8-connected A* with corner-cut prevention over the free cells of `occ`
// (row-major rows×cols, nonzero = blocked). Reproduces Python heapq's
// (f, g, node, parent) pop order, returning the byte-identical optimal path
// among equal-cost ties. `start` and `goal` (row, col) are snapped via
// nearest_free exactly as planner.astar does. `cost` is an optional per-cell
// surcharge (row-major rows×cols, in units of grid steps, added on entry) or
// nullptr for the plain shortest path. Returns the path flattened as
// [r0, c0, r1, c1, ...]; an empty vector means None (unreachable, or a snapped
// endpoint was None).
std::vector<int> astar(const std::uint8_t *occ, int rows, int cols, int start_r, int start_c,
                       int goal_r, int goal_c, const double *cost = nullptr);

// String-pull (planner.simplify): keep only the corners needed to preserve
// line of sight. `path` is flattened [r0, c0, r1, c1, ...] of length 2*n;
// returns the simplified path flattened the same way. Paths shorter than three
// points are returned unchanged.
std::vector<int> simplify(const std::uint8_t *occ, int rows, int cols, const int *path, int n);

// ─── Batched, threaded per-agent kernels (PERFORMANCE.md stage 4) ────────────
//
// The agents are independent, so a sense tick's N replans/rebuilds fan out
// across cores. Each single-agent kernel already allocates all of its working
// state locally, so the batch is data-race-free by construction and every
// result is byte-identical to the equivalent serial call.

// One independent A* query over its own occupancy plane. All queries in a batch
// share rows/cols (the raster) but not the contents (each agent's belief map).
struct astar_query {
  const std::uint8_t *occ; // row-major rows*cols, nonzero = blocked
  int start_r, start_c;
  int goal_r, goal_c;
  const double *cost; // optional per-cell surcharge (rows*cols) or nullptr
};

// Run `queries` in parallel across `num_threads` workers (<=0 => hardware
// concurrency). Returns one flattened [r0,c0,...] path per query (empty ==
// unreachable), in input order.
std::vector<std::vector<int>> astar_batch(const std::vector<astar_query> &queries, int rows,
                                          int cols, int num_threads = 0);

// Build one SDF field per occupancy plane in parallel. `occs[i]` is agent i's
// footprint occupancy (row-major rows*cols); bounds and scale are shared.
std::vector<sdf_field> build_sdf_batch(const std::vector<const std::uint8_t *> &occs, int rows,
                                       int cols, double min_x, double min_y, double max_x,
                                       double max_y, double scale, int num_threads = 0);

// Dilate N occupancy planes in parallel (each row-major rows*cols, nonzero =
// blocked) by `cells` 4-connected steps. Returns N dilated planes (row-major
// 0/1), in input order — each byte-identical to the serial inflate.
std::vector<std::vector<std::uint8_t>> inflate_batch(const std::vector<const std::uint8_t *> &occs,
                                                     int rows, int cols, int cells,
                                                     int num_threads = 0);

// ─── Batched belief sensing (grl_snam BeliefGrid.sense) ──────────────────────
//
// The last per-agent Python hole in the real-time tick: N agents ray-cast the
// truth grid and fold the result into a log-odds occupancy belief. This is a
// bit-identical batched port of grl_snam/belief.py BeliefGrid.sense, sensing N
// agents into K belief planes selected per agent by an `agent_map`, so the
// shared/clustered/private belief modes are one data parameter (K = 1 / groups
// / N). It threads across PLANES: distinct planes are disjoint memory, and the
// agents mapped to one plane are folded sequentially in ascending index — which
// is exactly what N serial BeliefGrid.sense calls do (each adds its per-cell
// log-odds delta then clamps, and the next reads the clamped result), so the
// result is bit-identical in every mode, with parallelism = the plane count.
//
// Bit-identity rests on the same discipline as the other kernels (this TU is
// compiled without -ffast-math / -ffp-contract=fast) plus: float64 cos/sin/hypot
// (system libm, which numpy dispatches to), std::rint under the default
// round-half-to-even, a march-and-break DDA reproducing the vectorized
// first-stop/free/occ masks, and log-odds accumulated as f32-store/f64-add of
// the double constant (the np.add.at-onto-float32 semantics), free before occ.

// K row-major rows*cols belief planes, all mutated IN PLACE. `logodds` is the
// log-odds memory; `last_visible`/`ever_seen` are numpy-bool (0/1) FoV masks;
// `version[k]` bumps once per agent whose sense flips any cell's occupied bit.
struct belief_planes {
  float *logodds = nullptr;             // [K*rows*cols]
  std::uint8_t *last_visible = nullptr; // [K*rows*cols]
  std::uint8_t *ever_seen = nullptr;    // [K*rows*cols]
  std::int32_t *version = nullptr;      // [K]
  int K = 0;
};

// Per-agent pose + sensor cone; every array has length n (the adapter broadcasts
// scalar sensor params). `agent_map[i]` selects the belief plane agent i writes.
struct sense_agents {
  const double *pos = nullptr;             // [n*2] (x,y) world, interleaved
  const double *heading = nullptr;         // [n] rad
  const double *range_m = nullptr;         // [n] world units
  const double *fov_rad = nullptr;         // [n] rad
  const std::int32_t *n_rays = nullptr;    // [n]
  const std::int32_t *agent_map = nullptr; // [n] -> plane in [0, K)
  int n = 0;
};

// Sense `ag.n` agents into `planes`, bit-identically to `ag.n` sequential
// BeliefGrid.sense calls in ascending agent index (agent i -> plane
// agent_map[i]). `truth` is the row-major rows*cols static occupancy (nonzero =
// blocked). `peer_boxes` is an optional [n*kmax*4] block of per-agent
// (r0,r1,c0,c1) HALF-OPEN cell rects (self-excluded, already clamped;
// zero-area = padding), `mover_boxes` an optional [n_movers*4] shared block;
// both, where present, occlude rays AND deposit +l_occ AND enter the FoV,
// exactly as the reference's truth_now composition does — so a caller that wants
// peers kept OUT of the belief (the shared swarm) simply passes none.
// bounds = (min_x, min_y, max_x, max_y); cell_w/cell_h are derived internally as
// (max-min)/(dim-1), matching BeliefGrid.sense. `flips_out[i]` receives agent
// i's flip count. num_threads <= 0 => hardware concurrency (capped at K).
void sense_batch(const std::uint8_t *truth, int rows, int cols, double min_x, double min_y,
                 double max_x, double max_y, const sense_agents &ag, const std::int32_t *peer_boxes,
                 int kmax, const std::int32_t *mover_boxes, int n_movers,
                 const belief_planes &planes, double l_occ, double l_free, double l_clamp,
                 std::int32_t *flips_out, int num_threads = 0, thread_pool *pool = nullptr);

// ─── Fixed-radius neighbour search (CGAL Kd_tree) ────────────────────────────
//
// For an N-body crowd, "which peers are within an agent's sensor range" is a
// fixed-radius all-neighbours query. A uniform grid degrades to O(N^2) on a
// dense cluster; a Kd_tree stays robust. Implemented in spatial.cpp against
// CGAL's Kd_tree + Fuzzy_sphere (exact, epsilon=0), which libcvc already links
// — so this header stays CGAL-free (the signature is plain STL).

// Neighbours in CSR form: point i's neighbours are indices[offsets[i] ..
// offsets[i+1]).
struct neighbor_csr {
  std::vector<int> offsets; // size n+1
  std::vector<int> indices; // concatenated neighbour indices
};

// Every OTHER point within `radius` (Euclidean, inclusive) of each of the `n`
// 2-D points in `positions` (row-major [n][2], (x,y)). Neighbour indices are
// returned ascending, so the result is deterministic.
neighbor_csr neighbors_within_radius(const double *positions, int n, double radius);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_GRID_NAV_H__
