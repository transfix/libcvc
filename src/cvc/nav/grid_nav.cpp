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

// grid_nav.cpp — see grid_nav.h for the design and the fidelity guardrails.
//
// Every routine here is a line-for-line transcription of the GRL-SNAM Python
// reference (grl_snam/planner.py and sdf_nav.py). Where the Python does integer
// arithmetic the C++ does integer arithmetic and casts once, so the float
// results are bit-identical. Do NOT "tidy" the arithmetic grouping, the
// tie-break comparisons, or the scan orders: they are load-bearing (a different
// but equally optimal path or a flipped medial-axis gradient is a silent
// digital-twin divergence, not a rounding difference). This TU must be built
// without -ffast-math / -ffp-contract=fast.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <cvc/nav/detail/grid_math.h>
#include <cvc/nav/detail/parallel.h>
#include <cvc/nav/grid_nav.h>
#include <limits>
#include <queue>
#include <thread>

namespace cvc {
namespace nav {

namespace {

constexpr double EDT_INF = 1e20;

// Felzenszwalb & Huttenlocher 1-D squared-distance transform over a single
// strided line: reads f[q * fs] for q in [0, n), writes d[q * ds]. This is the
// scalar kernel sdf_nav._edt1d, unchanged.
void edt1d_line(const double *f, int n, int fs, double *d, int ds) {
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -EDT_INF;
  z[1] = EDT_INF;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q * fs] + static_cast<double>(q * q)) -
                (f[v[k] * fs] + static_cast<double>(v[k] * v[k]))) /
               static_cast<double>(2 * q - 2 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q * fs] + static_cast<double>(q * q)) -
           (f[v[k] * fs] + static_cast<double>(v[k] * v[k]))) /
          static_cast<double>(2 * q - 2 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = EDT_INF;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q)
      ++k;
    int diff = q - v[k];
    d[q * ds] = static_cast<double>(diff * diff) + f[v[k] * fs];
  }
}

// grad1d (np.gradient transcription) lives in detail/grid_math.h — shared
// with material.cpp; it was extracted from here unchanged.
using detail::grad1d;

} // namespace

std::vector<double> edt2_squared(const std::uint8_t *mask, int rows, int cols) {
  const int n = rows * cols;
  std::vector<double> f(n);
  for (int i = 0; i < n; ++i)
    f[i] = mask[i] ? 0.0 : EDT_INF;

  // Separable: sdf_nav._edt2 transforms columns (down each column, the y-axis)
  // first, then rows (across each row, the x-axis). Column stride is `cols`,
  // row stride is 1.
  std::vector<double> tmp(n);
  for (int c = 0; c < cols; ++c)
    edt1d_line(&f[c], rows, cols, &tmp[c], cols);
  std::vector<double> out(n);
  for (int r = 0; r < rows; ++r)
    edt1d_line(&tmp[r * cols], cols, 1, &out[r * cols], 1);
  return out;
}

sdf_field build_sdf(const std::uint8_t *occ, int rows, int cols, double min_x, double /*min_y*/,
                    double max_x, double /*max_y*/, double scale) {
  const int n = rows * cols;
  const std::vector<double> d_out = edt2_squared(occ, rows, cols); // to building
  std::vector<std::uint8_t> inv(n);
  for (int i = 0; i < n; ++i)
    inv[i] = occ[i] ? 0 : 1;
  const std::vector<double> d_in = edt2_squared(inv.data(), rows, cols); // to free

  const double cell_w = (max_x - min_x) / static_cast<double>(cols - 1);

  sdf_field field;
  field.rows = rows;
  field.cols = cols;
  field.phi.resize(n);
  for (int i = 0; i < n; ++i) {
    const double phi_w = (std::sqrt(d_out[i]) - std::sqrt(d_in[i])) * cell_w;
    field.phi[i] = static_cast<float>(phi_w * scale);
  }

  // Unit outward normal = normalized np.gradient(phi). gy = d/drow (axis 0),
  // gx = d/dcol (axis 1); normal is (gx, gy) / |grad|. All float32.
  const float *phi = field.phi.data();
  field.normal_x.resize(n);
  field.normal_y.resize(n);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c) {
      const int i = r * cols + c;
      const float gx = grad1d(phi + r * cols, c, cols, 1);
      const float gy = grad1d(phi + c, r, rows, cols);
      const float gmag = std::sqrt(gx * gx + gy * gy) + 1e-9f;
      field.normal_x[i] = gx / gmag;
      field.normal_y[i] = gy / gmag;
    }
  return field;
}

std::vector<std::uint8_t> inflate(const std::uint8_t *occ, int rows, int cols, int cells) {
  const int n = rows * cols;
  std::vector<std::uint8_t> out(n);
  for (int i = 0; i < n; ++i)
    out[i] = occ[i] ? 1 : 0;

  const int steps = std::max(0, cells);
  for (int it = 0; it < steps; ++it) {
    std::vector<std::uint8_t> grown(out);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const int idx = r * cols + c;
        std::uint8_t v = out[idx];
        if (r > 0)
          v |= out[idx - cols];
        if (r < rows - 1)
          v |= out[idx + cols];
        if (c > 0)
          v |= out[idx - 1];
        if (c < cols - 1)
          v |= out[idx + 1];
        grown[idx] = v ? 1 : 0;
      }
    out.swap(grown);
  }
  return out;
}

bool line_of_sight(const std::uint8_t *occ, int /*rows*/, int cols, int ar, int ac, int br,
                   int bc) {
  const int dr = std::abs(br - ar);
  const int dc = std::abs(bc - ac);
  const int sr = (br > ar) ? 1 : -1;
  const int sc = (bc > ac) ? 1 : -1;
  int err = dr - dc;
  int r = ar, c = ac;
  while (true) {
    if (occ[r * cols + c])
      return false;
    if (r == br && c == bc)
      return true;
    const int e2 = 2 * err;
    if (e2 > -dc) {
      err -= dc;
      r += sr;
    }
    if (e2 < dr) {
      err += dr;
      c += sc;
    }
  }
}

std::pair<int, int> nearest_free(const std::uint8_t *occ, int rows, int cols, int r, int c,
                                 int max_radius) {
  r = std::min(std::max(r, 0), rows - 1);
  c = std::min(std::max(c, 0), cols - 1);
  if (!occ[r * cols + c])
    return {r, c};
  for (int rad = 1; rad <= max_radius; ++rad)
    for (int dr = -rad; dr <= rad; ++dr)
      for (const int dc : {-rad, rad}) {
        const std::pair<int, int> cands[2] = {{r + dr, c + dc}, {r + dc, c + dr}};
        for (const auto &p : cands) {
          const int rr = p.first, cc = p.second;
          if (rr >= 0 && rr < rows && cc >= 0 && cc < cols && !occ[rr * cols + cc])
            return {rr, cc};
        }
      }
  return {-1, -1};
}

namespace {

// One open-set entry. Ordering reproduces Python heapq's comparison of the
// tuple (f, g, node, parent): f, then g, then node=(r,c), then parent, encoded
// as the row-major key `pk` (which preserves the parent tuple's lexicographic
// order; -1 == None, sorting before any real cell). The (f, g, r, c) prefix is
// already a strict total order over any two entries that can coexist in the
// heap — a node is only ever re-pushed with a strictly smaller g — so `pk`
// never actually decides a pop; it is carried only for exactness.
struct AStarNode {
  double f;
  double g;
  int r;
  int c;
  int pk;
};

struct AStarWorse {
  // std::priority_queue keeps the "greatest" element on top, so return true
  // when `a` should pop AFTER `b`, i.e. a's tuple is greater than b's.
  bool operator()(const AStarNode &a, const AStarNode &b) const {
    if (a.f != b.f)
      return a.f > b.f;
    if (a.g != b.g)
      return a.g > b.g;
    if (a.r != b.r)
      return a.r > b.r;
    if (a.c != b.c)
      return a.c > b.c;
    return a.pk > b.pk;
  }
};

// Per-thread reusable A* scratch. Replaces the per-call `vector(ncells)` alloc +
// O(ncells) fill of the came/g_best arrays with a "generation stamp": a cell
// belongs to THIS search iff its stamp equals the current gen, so a new search
// is just `++gen` (O(1)) instead of clearing the arrays. Held `thread_local`, so
// each worker in astar_batch reuses one buffer across the queries it runs. The
// values and comparisons are identical to the flat-array version — bit-identical
// results — this only changes how "unvisited" / "g_best = +inf" is represented.
struct AStarScratch {
  std::vector<long long> came_gen; // came_val[k] belongs to this search iff ==gen
  std::vector<long long> g_gen;    // g_val[k]  belongs to this search iff ==gen
  std::vector<int> came_val;       // parent cell key (-1 == None)
  std::vector<double> g_val;       // best g so far
  long long gen = 0;
  int n = 0;
  // Prepare for a search over `need` cells and return this search's generation.
  long long begin(int need) {
    if (n < need) {
      n = need;
      came_gen.assign(n, 0);
      g_gen.assign(n, 0);
      came_val.resize(n);
      g_val.resize(n);
      gen = 0; // the assigns above zeroed every stamp, so restart the counter
    }
    return ++gen;
  }
};

} // namespace

std::vector<int> astar(const std::uint8_t *occ, int rows, int cols, int start_r, int start_c,
                       int goal_r, int goal_c, const double *cost) {
  const std::pair<int, int> start = nearest_free(occ, rows, cols, start_r, start_c);
  const std::pair<int, int> goal = nearest_free(occ, rows, cols, goal_r, goal_c);
  if (start.first < 0 || goal.first < 0)
    return {};

  static const double SQRT2 = std::sqrt(2.0);
  const int gr = goal.first, gc = goal.second;
  auto h = [&](int r, int c) {
    return std::hypot(static_cast<double>(r - gr), static_cast<double>(c - gc));
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, AStarWorse> open;
  // Closed-set / cost arrays keyed by row-major cell index, held per-thread and
  // reused across searches via a generation stamp (see AStarScratch). Values and
  // pop order are identical to Python's `came`/`g_best` dicts: a cell is "in
  // came" iff came_gen[k] == GEN (came_val holds the parent key, -1 == None), and
  // g_best is g_val[k] iff g_gen[k] == GEN else +inf.
  static thread_local AStarScratch S;
  const int ncells = rows * cols;
  const long long GEN = S.begin(ncells);
  const int start_key = start.first * cols + start.second;
  const int goal_key = gr * cols + gc;
  open.push({h(start.first, start.second), 0.0, start.first, start.second, -1});
  S.g_gen[start_key] = GEN;
  S.g_val[start_key] = 0.0;

  while (!open.empty()) {
    const AStarNode cur = open.top();
    open.pop();
    const int node_key = cur.r * cols + cur.c;
    if (S.came_gen[node_key] == GEN)
      continue;
    S.came_gen[node_key] = GEN;
    S.came_val[node_key] = cur.pk;
    if (node_key == goal_key) {
      std::vector<int> keys;
      keys.push_back(node_key);
      int p = S.came_val[node_key];
      while (p != -1) {
        keys.push_back(p);
        p = S.came_val[p];
      }
      std::vector<int> path;
      path.reserve(keys.size() * 2);
      for (auto it = keys.rbegin(); it != keys.rend(); ++it) {
        path.push_back(*it / cols);
        path.push_back(*it % cols);
      }
      return path;
    }
    const int r = cur.r, c = cur.c;
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0)
          continue;
        const int nr = r + dr, nc = c + dc;
        if (!(nr >= 0 && nr < rows && nc >= 0 && nc < cols) || occ[nr * cols + nc])
          continue;
        if (dr && dc && (occ[(r + dr) * cols + c] || occ[r * cols + (c + dc)]))
          continue; // no corner cutting
        double ng = cur.g + ((dr && dc) ? SQRT2 : 1.0);
        if (cost)
          ng += cost[nr * cols + nc];
        const int nkey = nr * cols + nc;
        const double gb =
            (S.g_gen[nkey] == GEN) ? S.g_val[nkey] : std::numeric_limits<double>::infinity();
        if (ng < gb) {
          S.g_gen[nkey] = GEN;
          S.g_val[nkey] = ng;
          open.push({ng + h(nr, nc), ng, nr, nc, node_key});
        }
      }
  }
  return {};
}

std::vector<int> simplify(const std::uint8_t *occ, int rows, int cols, const int *path, int n) {
  if (n < 3)
    return std::vector<int>(path, path + 2 * n);

  std::vector<int> out;
  out.push_back(path[0]);
  out.push_back(path[1]);
  int anchor = 0;
  for (int i = 2; i < n; ++i) {
    if (!line_of_sight(occ, rows, cols, path[2 * anchor], path[2 * anchor + 1], path[2 * i],
                       path[2 * i + 1])) {
      out.push_back(path[2 * (i - 1)]);
      out.push_back(path[2 * (i - 1) + 1]);
      anchor = i - 1;
    }
  }
  out.push_back(path[2 * (n - 1)]);
  out.push_back(path[2 * (n - 1) + 1]);
  return out;
}

namespace {

using cvc::nav::detail::parallel_for; // the shared work-splitter (detail/parallel.h)

} // namespace

std::vector<std::vector<int>> astar_batch(const std::vector<astar_query> &queries, int rows,
                                          int cols, int num_threads) {
  std::vector<std::vector<int>> out(queries.size());
  parallel_for(static_cast<int>(queries.size()), num_threads, [&](int i) {
    const astar_query &q = queries[i];
    out[i] = astar(q.occ, rows, cols, q.start_r, q.start_c, q.goal_r, q.goal_c, q.cost);
  });
  return out;
}

std::vector<sdf_field> build_sdf_batch(const std::vector<const std::uint8_t *> &occs, int rows,
                                       int cols, double min_x, double min_y, double max_x,
                                       double max_y, double scale, int num_threads) {
  std::vector<sdf_field> out(occs.size());
  parallel_for(static_cast<int>(occs.size()), num_threads, [&](int i) {
    out[i] = build_sdf(occs[i], rows, cols, min_x, min_y, max_x, max_y, scale);
  });
  return out;
}

std::vector<std::vector<std::uint8_t>> inflate_batch(const std::vector<const std::uint8_t *> &occs,
                                                     int rows, int cols, int cells,
                                                     int num_threads) {
  std::vector<std::vector<std::uint8_t>> out(occs.size());
  parallel_for(static_cast<int>(occs.size()), num_threads,
               [&](int i) { out[i] = inflate(occs[i], rows, cols, cells); });
  return out;
}

// ─── Belief sensing (BeliefGrid.sense) ───────────────────────────────────────

namespace {

// Point-in-any-box over a [m*4] block of (r0,r1,c0,c1) HALF-OPEN cell rects; a
// zero-area rect (r0>=r1 || c0>=c1) is a padding slot and is skipped. This is
// byte-identical to rasterizing the boxes into truth and indexing (the boxes
// are the caller's peer/mover footprints, already cellified Python-side exactly
// as BeliefGrid does), so no float surface is added.
inline bool in_boxes(const std::int32_t *b, int m, int rr, int cc) {
  for (int t = 0; t < m; ++t) {
    const std::int32_t *q = b + 4 * t;
    if (q[0] >= q[1] || q[2] >= q[3])
      continue;
    if (rr >= q[0] && rr < q[1] && cc >= q[2] && cc < q[3])
      return true;
  }
  return false;
}

// Per-agent scratch: generation-stamped free/occ visit counts (a new agent is
// O(1) via ++gen, not an O(rows*cols) clear — cf. the A* scratch), plus the list
// of cells this agent's cone touched (its FoV). thread_local, one per worker.
struct SenseScratch {
  std::vector<int> free_cnt, occ_cnt;
  std::vector<long long> tgen;
  std::vector<int> touched;
  long long gen = 0;
  int n = 0;
  long long begin(int need) {
    if (n < need) {
      n = need;
      free_cnt.assign(n, 0);
      occ_cnt.assign(n, 0);
      tgen.assign(n, 0);
      gen = 0;
    }
    touched.clear();
    return ++gen;
  }
};

// One agent's ray-cast output: the cells its cone touched (in first-touch
// order) with per-cell free/occ visit counts. Plane-INDEPENDENT — the raycast
// reads only `truth` and the agent's own boxes, so it is order-free and safe to
// run for every agent in parallel; the plane-dependent, order-dependent log-odds
// fold is a separate pass (sense_batch). `oob` = start cell off the grid (that
// agent updates nothing and blanks its plane's last_visible).
struct agent_sense {
  std::vector<int> cells;  // touched grid indices, first-touch order
  std::vector<int> free_c; // parallel to cells: rays that passed through free
  std::vector<int> occ_c;  // parallel to cells: rays that stopped on a hit
  bool oob = true;
};

// Ray-cast one agent (BeliefGrid.sense's DDA, belief.py:118-155) into `out`. No
// log-odds / version / last_visible / ever_seen touched here — this is the
// order-free half, the expensive part, and the whole point of the split.
void raycast_agent(int i, const std::uint8_t *truth, int rows, int cols, double min_x, double min_y,
                   double max_x, double max_y, const sense_agents &ag,
                   const std::int32_t *peer_boxes, int kmax, const std::int32_t *mover_boxes,
                   int n_movers, SenseScratch &S, agent_sense &out) {
  out.cells.clear();
  out.free_c.clear();
  out.occ_c.clear();
  const double cell_w = (max_x - min_x) / (cols - 1); // == belief.py:109-110
  const double cell_h = (max_y - min_y) / (rows - 1);
  const double cxf = (ag.pos[2 * i] - min_x) / (max_x - min_x) * (cols - 1);
  const double cyf = (ag.pos[2 * i + 1] - min_y) / (max_y - min_y) * (rows - 1);
  const int r0 = static_cast<int>(std::rint(cyf)); // == int(round(...)) half-to-even
  const int c0 = static_cast<int>(std::rint(cxf));
  if (!(r0 >= 0 && r0 < rows && c0 >= 0 && c0 < cols)) {
    out.oob = true; // belief.py:113-115 early return
    return;
  }
  out.oob = false;

  const long long GEN = S.begin(rows * cols);
  const double rangem = ag.range_m[i], fov = ag.fov_rad[i], head = ag.heading[i];
  const int nr = ag.n_rays[i];
  const int n_steps = static_cast<int>(rangem / cell_w) + 1; // int() truncation
  const std::int32_t *pbox = kmax ? peer_boxes + static_cast<long>(i) * kmax * 4 : nullptr;

  for (int j = 0; j < nr; ++j) {
    const double a = head + ((double)j / (double)std::max(nr, 1) - 0.5) * fov; // belief.py:121
    const double dx = std::cos(a), dy = std::sin(a);
    double sr = dy * (cell_w / cell_h), sc = dx;
    const double nrm = std::max(std::max(std::fabs(sr), std::fabs(sc)), 1e-9);
    sr /= nrm;
    sc /= nrm;
    for (int s = 0; s < n_steps; ++s) {
      const int rr = static_cast<int>(std::rint((double)r0 + sr * (double)s));
      const int cc = static_cast<int>(std::rint((double)c0 + sc * (double)s));
      const double dist = std::hypot((double)(rr - r0) * cell_h, (double)(cc - c0) * cell_w);
      if (!(rr >= 0 && rr < rows && cc >= 0 && cc < cols && dist <= rangem))
        break; // !inside -> stop this ray, mark no occ (occ requires a hit, which is in-range)
      const int k = rr * cols + cc;
      if (S.tgen[k] != GEN) {
        S.tgen[k] = GEN;
        S.free_cnt[k] = 0;
        S.occ_cnt[k] = 0;
        S.touched.push_back(k);
      }
      const bool occ = truth[k] || in_boxes(pbox, kmax, rr, cc) ||
                       in_boxes(mover_boxes, n_movers, rr, cc); // peers/movers are hits (R1)
      if (occ) {
        ++S.occ_cnt[k];
        break; // occlusion truncates the ray
      }
      ++S.free_cnt[k];
    }
  }

  // Copy the touched cells + counts out in first-touch order, so the later fold
  // visits them in exactly the order the serial reference accumulated them.
  out.cells.reserve(S.touched.size());
  out.free_c.reserve(S.touched.size());
  out.occ_c.reserve(S.touched.size());
  for (int k : S.touched) {
    out.cells.push_back(k);
    out.free_c.push_back(S.free_cnt[k]);
    out.occ_c.push_back(S.occ_cnt[k]);
  }
}

} // namespace

void sense_batch(const std::uint8_t *truth, int rows, int cols, double min_x, double min_y,
                 double max_x, double max_y, const sense_agents &ag, const std::int32_t *peer_boxes,
                 int kmax, const std::int32_t *mover_boxes, int n_movers,
                 const belief_planes &planes, double l_occ, double l_free, double l_clamp,
                 std::int32_t *flips_out, int num_threads, thread_pool *pool) {
  const int N = ag.n;

  // Phase A — RAYCAST every agent in parallel. The raycast reads only `truth`
  // and the agent's own boxes, so it is order-free and fully parallel across ALL
  // N agents (not just across planes). This is what unblocks shared mode (M=1):
  // there the whole cost is the raycast, and the old plane-keyed loop gave it a
  // single worker (K=1). Bit-identity is untouched — a plane-independent raycast
  // produces the same touched cells + counts regardless of when it runs.
  std::vector<agent_sense> res(N);
  parallel_for(pool, N, num_threads, [&](int i) {
    thread_local SenseScratch S;
    raycast_agent(i, truth, rows, cols, min_x, min_y, max_x, max_y, ag, peer_boxes, kmax,
                  mover_boxes, n_movers, S, res[i]);
  });

  // Bucket agents by plane, preserving ascending global index within each — the
  // exact order N serial sense() calls would fold in.
  std::vector<std::vector<int>> by_plane(planes.K);
  for (int i = 0; i < N; ++i)
    by_plane[ag.agent_map[i]].push_back(i);

  // Phase B — FOLD per plane, agents SEQUENTIAL in ascending index: accumulate
  // each agent's per-cell delta (Model C) then clamp, reading the previous
  // agent's clamped value — exactly as N serial BeliefGrid.sense calls do.
  // Distinct planes are disjoint memory (parallel across planes); a plane's
  // agents are ordered. This half is cheap, so shared mode's single-plane serial
  // fold costs almost nothing while its raycast just ran N-way parallel.
  const float clampf = static_cast<float>(l_clamp);
  parallel_for(pool, planes.K, num_threads, [&](int p) {
    const long base = static_cast<long>(p) * rows * cols;
    const std::vector<int> *last_cells = nullptr; // final non-OOB agent's FoV
    for (int i : by_plane[p]) {
      const agent_sense &a = res[i];
      if (a.oob) {
        flips_out[i] = 0;
        last_cells = nullptr; // an OOB last agent blanks last_visible
        continue;
      }
      int flips = 0;
      for (std::size_t t = 0; t < a.cells.size(); ++t) {
        const int k = a.cells[t];
        planes.ever_seen[base + k] = 1; // FoV, monotonic OR (plane-serial here => race-free)
        // Model C: running delta is f32, each add is (float)((double)df + CONST)
        // with the constant kept double — the np.add.at-onto-float32 semantics.
        float df = 0.0f;
        for (int u = 0; u < a.free_c[t]; ++u)
          df = static_cast<float>(static_cast<double>(df) + l_free);
        for (int u = 0; u < a.occ_c[t]; ++u)
          df = static_cast<float>(static_cast<double>(df) + l_occ);
        const float old = planes.logodds[base + k];
        float nv = static_cast<float>(old + df); // np.clip(logodds + delta, ...)
        nv = std::min(std::max(nv, -clampf), clampf);
        if ((old > 0.0f) != (nv > 0.0f))
          ++flips;
        planes.logodds[base + k] = nv;
      }
      if (flips)
        ++planes.version[p];
      flips_out[i] = flips;
      last_cells = &a.cells;
    }
    // last_visible is last-agent-wins (belief.py:162): blank the plane, then
    // stamp the final non-OOB agent's FoV.
    std::uint8_t *lv = planes.last_visible + base;
    std::memset(lv, 0, static_cast<std::size_t>(rows) * cols);
    if (last_cells)
      for (int k : *last_cells)
        lv[k] = 1;
  });
}

} // namespace nav
} // namespace cvc
