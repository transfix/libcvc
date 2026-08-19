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

#include <cvc/nav/grid_nav.h>

#include <algorithm>
#include <atomic>
#include <cmath>
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
void edt1d_line(const double *f, int n, int fs, double *d, int ds)
{
  std::vector<int> v(n);
  std::vector<double> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -EDT_INF;
  z[1] = EDT_INF;
  for (int q = 1; q < n; ++q)
  {
    double s = ((f[q * fs] + static_cast<double>(q * q)) -
                (f[v[k] * fs] + static_cast<double>(v[k] * v[k]))) /
               static_cast<double>(2 * q - 2 * v[k]);
    while (s <= z[k])
    {
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
  for (int q = 0; q < n; ++q)
  {
    while (z[k + 1] < q)
      ++k;
    int diff = q - v[k];
    d[q * ds] = static_cast<double>(diff * diff) + f[v[k] * fs];
  }
}

// np.gradient along one axis (edge_order=1, unit spacing), evaluated in float32
// at index i of a length-L line whose neighbours are `stride` apart in `a`.
inline float grad1d(const float *a, int i, int L, int stride)
{
  if (i == 0)
    return a[stride] - a[0];
  if (i == L - 1)
    return a[i * stride] - a[(i - 1) * stride];
  return (a[(i + 1) * stride] - a[(i - 1) * stride]) / 2.0f;
}

} // namespace

std::vector<double> edt2_squared(const std::uint8_t *mask, int rows, int cols)
{
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

sdf_field build_sdf(const std::uint8_t *occ, int rows, int cols, double min_x,
                    double /*min_y*/, double max_x, double /*max_y*/,
                    double scale)
{
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
  for (int i = 0; i < n; ++i)
  {
    const double phi_w = (std::sqrt(d_out[i]) - std::sqrt(d_in[i])) * cell_w;
    field.phi[i] = static_cast<float>(phi_w * scale);
  }

  // Unit outward normal = normalized np.gradient(phi). gy = d/drow (axis 0),
  // gx = d/dcol (axis 1); normal is (gx, gy) / |grad|. All float32.
  const float *phi = field.phi.data();
  field.normal_x.resize(n);
  field.normal_y.resize(n);
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
    {
      const int i = r * cols + c;
      const float gx = grad1d(phi + r * cols, c, cols, 1);
      const float gy = grad1d(phi + c, r, rows, cols);
      const float gmag = std::sqrt(gx * gx + gy * gy) + 1e-9f;
      field.normal_x[i] = gx / gmag;
      field.normal_y[i] = gy / gmag;
    }
  return field;
}

std::vector<std::uint8_t> inflate(const std::uint8_t *occ, int rows, int cols,
                                  int cells)
{
  const int n = rows * cols;
  std::vector<std::uint8_t> out(n);
  for (int i = 0; i < n; ++i)
    out[i] = occ[i] ? 1 : 0;

  const int steps = std::max(0, cells);
  for (int it = 0; it < steps; ++it)
  {
    std::vector<std::uint8_t> grown(out);
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c)
      {
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

bool line_of_sight(const std::uint8_t *occ, int /*rows*/, int cols, int ar,
                   int ac, int br, int bc)
{
  const int dr = std::abs(br - ar);
  const int dc = std::abs(bc - ac);
  const int sr = (br > ar) ? 1 : -1;
  const int sc = (bc > ac) ? 1 : -1;
  int err = dr - dc;
  int r = ar, c = ac;
  while (true)
  {
    if (occ[r * cols + c])
      return false;
    if (r == br && c == bc)
      return true;
    const int e2 = 2 * err;
    if (e2 > -dc)
    {
      err -= dc;
      r += sr;
    }
    if (e2 < dr)
    {
      err += dr;
      c += sc;
    }
  }
}

std::pair<int, int> nearest_free(const std::uint8_t *occ, int rows, int cols,
                                 int r, int c, int max_radius)
{
  r = std::min(std::max(r, 0), rows - 1);
  c = std::min(std::max(c, 0), cols - 1);
  if (!occ[r * cols + c])
    return {r, c};
  for (int rad = 1; rad <= max_radius; ++rad)
    for (int dr = -rad; dr <= rad; ++dr)
      for (const int dc : {-rad, rad})
      {
        const std::pair<int, int> cands[2] = {{r + dr, c + dc}, {r + dc, c + dr}};
        for (const auto &p : cands)
        {
          const int rr = p.first, cc = p.second;
          if (rr >= 0 && rr < rows && cc >= 0 && cc < cols &&
              !occ[rr * cols + cc])
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
struct AStarNode
{
  double f;
  double g;
  int r;
  int c;
  int pk;
};

struct AStarWorse
{
  // std::priority_queue keeps the "greatest" element on top, so return true
  // when `a` should pop AFTER `b`, i.e. a's tuple is greater than b's.
  bool operator()(const AStarNode &a, const AStarNode &b) const
  {
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

} // namespace

std::vector<int> astar(const std::uint8_t *occ, int rows, int cols, int start_r,
                       int start_c, int goal_r, int goal_c, const double *cost)
{
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
  // Flat closed-set / cost arrays keyed by the row-major cell index: identical
  // values and pop order to Python's `came`/`g_best` dicts, but O(1) with no
  // hashing. came: -2 = unvisited (== "not in came"), -1 = None (the start's
  // parent), >=0 = parent cell key. g_best defaults to +inf (== dict.get miss).
  const int ncells = rows * cols;
  std::vector<int> came(ncells, -2);
  std::vector<double> g_best(ncells, std::numeric_limits<double>::infinity());
  const int start_key = start.first * cols + start.second;
  const int goal_key = gr * cols + gc;
  open.push({h(start.first, start.second), 0.0, start.first, start.second, -1});
  g_best[start_key] = 0.0;

  while (!open.empty())
  {
    const AStarNode cur = open.top();
    open.pop();
    const int node_key = cur.r * cols + cur.c;
    if (came[node_key] != -2)
      continue;
    came[node_key] = cur.pk;
    if (node_key == goal_key)
    {
      std::vector<int> keys;
      keys.push_back(node_key);
      int p = came[node_key];
      while (p != -1)
      {
        keys.push_back(p);
        p = came[p];
      }
      std::vector<int> path;
      path.reserve(keys.size() * 2);
      for (auto it = keys.rbegin(); it != keys.rend(); ++it)
      {
        path.push_back(*it / cols);
        path.push_back(*it % cols);
      }
      return path;
    }
    const int r = cur.r, c = cur.c;
    for (int dr = -1; dr <= 1; ++dr)
      for (int dc = -1; dc <= 1; ++dc)
      {
        if (dr == 0 && dc == 0)
          continue;
        const int nr = r + dr, nc = c + dc;
        if (!(nr >= 0 && nr < rows && nc >= 0 && nc < cols) ||
            occ[nr * cols + nc])
          continue;
        if (dr && dc && (occ[(r + dr) * cols + c] || occ[r * cols + (c + dc)]))
          continue; // no corner cutting
        double ng = cur.g + ((dr && dc) ? SQRT2 : 1.0);
        if (cost)
          ng += cost[nr * cols + nc];
        const int nkey = nr * cols + nc;
        if (ng < g_best[nkey])
        {
          g_best[nkey] = ng;
          open.push({ng + h(nr, nc), ng, nr, nc, node_key});
        }
      }
  }
  return {};
}

std::vector<int> simplify(const std::uint8_t *occ, int rows, int cols,
                          const int *path, int n)
{
  if (n < 3)
    return std::vector<int>(path, path + 2 * n);

  std::vector<int> out;
  out.push_back(path[0]);
  out.push_back(path[1]);
  int anchor = 0;
  for (int i = 2; i < n; ++i)
  {
    if (!line_of_sight(occ, rows, cols, path[2 * anchor], path[2 * anchor + 1],
                       path[2 * i], path[2 * i + 1]))
    {
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

// Run fn(i) for i in [0, n) across `num_threads` workers (<=0 => hardware
// concurrency). A shared atomic counter hands out indices, so uneven per-item
// cost (some A* queries expand far more nodes than others) self-balances. The
// calling thread is one of the workers, so num_threads==1 runs inline.
template <class F> void parallel_for(int n, int num_threads, F &&fn)
{
  if (n <= 0)
    return;
  int nt = num_threads > 0
               ? num_threads
               : static_cast<int>(std::thread::hardware_concurrency());
  if (nt < 1)
    nt = 1;
  if (nt > n)
    nt = n;
  if (nt == 1)
  {
    for (int i = 0; i < n; ++i)
      fn(i);
    return;
  }
  std::atomic<int> next{0};
  auto worker = [&]() {
    for (int i = next.fetch_add(1); i < n; i = next.fetch_add(1))
      fn(i);
  };
  std::vector<std::thread> pool;
  pool.reserve(nt - 1);
  for (int t = 0; t < nt - 1; ++t)
    pool.emplace_back(worker);
  worker();
  for (auto &th : pool)
    th.join();
}

} // namespace

std::vector<std::vector<int>>
astar_batch(const std::vector<astar_query> &queries, int rows, int cols,
            int num_threads)
{
  std::vector<std::vector<int>> out(queries.size());
  parallel_for(static_cast<int>(queries.size()), num_threads, [&](int i) {
    const astar_query &q = queries[i];
    out[i] = astar(q.occ, rows, cols, q.start_r, q.start_c, q.goal_r, q.goal_c,
                   q.cost);
  });
  return out;
}

std::vector<sdf_field>
build_sdf_batch(const std::vector<const std::uint8_t *> &occs, int rows,
                int cols, double min_x, double min_y, double max_x,
                double max_y, double scale, int num_threads)
{
  std::vector<sdf_field> out(occs.size());
  parallel_for(static_cast<int>(occs.size()), num_threads, [&](int i) {
    out[i] = build_sdf(occs[i], rows, cols, min_x, min_y, max_x, max_y, scale);
  });
  return out;
}

} // namespace nav
} // namespace cvc
