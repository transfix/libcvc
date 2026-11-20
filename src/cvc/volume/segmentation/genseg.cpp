/******************************************************************************
                                Copyright

This code is developed within the Computational Visualization Center at The
University of Texas at Austin.

This code has been made available to you under the auspices of a Lesser General
Public License (LGPL) (http://www.ices.utexas.edu/cvc/software/license.html)
and terms that you have agreed to.

Upon accepting the LGPL, we request you agree to acknowledge the use of use of
the code that results in any published work, including scientific papers,
films, and videotapes by citing the following references:

C. Bajaj, Z. Yu, M. Auer
Volumetric Feature Extraction and Visualization of Tomographic Molecular Imaging
Journal of Structural Biology, Volume 144, Issues 1-2, October 2003, Pages
132-143.

If you desire to use this code for a profit venture, or if you do not wish to
accept LGPL, but desire usage of this code, please contact Chandrajit Bajaj
(bajaj@ices.utexas.edu) at the Computational Visualization Center at The
University of Texas at Austin for a different license.
******************************************************************************/

// GenSeg's segment.cpp (transfix/volrover src/Segmentation/GenSeg, at the sha in
// tools/volrover-goldens/volrover.sha) and read_data's normalisation (inout.cpp), ported as
// genseg_context (genseg.h). It labels every voxel as the legacy code did, voxel for voxel
// (src/cvc/tests/segmentation_test.cpp, against the goldens tools/volrover-goldens made from the
// legacy build). What changed:
//   - the globals are the context's members;
//   - Segment(), GetGradient(), GetSeeds() and SearchIndex() are coroutines that yield a unit at a
//     time (GS_UNIT): a voxel visited, a seed placed, a voxel popped from a heap, a step of a walk;
//   - Segment() reads no file and writes none: its input is the context's dataset, and its
//     outputs (seed_index, and tdata's MAX_TIME + 1 border marks) are left in the context for
//     cvc::segment_stepper to turn into volumes;
//   - GetTime() and T_GetTime() were each six hand-unrolled copies of one block, one per face
//     neighbour; they are loops over the six neighbours, in the legacy order (x - 1, x + 1,
//     y - 1, y + 1, z - 1, z + 1), the neighbours of a neighbour in the same order;
//   - a voxel's step cost, 0.01 exp(ALPHA |grad|), is computed once, where the legacy code
//     computed it again for each test, up to 36 times a voxel popped; exp() is computed in
//     double, which gives the same float on every platform's libm;
//   - the heaps hold a voxel's linear index (computed in std::size_t, so a volume of more than
//     2^31 voxels does not overflow) instead of its three coordinates;
//   - the threshold sweep keeps the automatic seeds it has yet to classify in a vector, and each
//     search clears only the times the previous search set. The legacy code kept a linked list
//     and, before each search, flood-filled every seed in it (InitialTime()), which cleared the
//     same voxels: every voxel a search sets is reached from one of its seeds through voxels it
//     set;
//   - the threshold sweep skips the searches that would reach no labelled voxel: after each seed
//     it classified, the legacy sweep searched again at every threshold from 255 down, and on a
//     large map those searches, which change nothing, took nearly all its time (Segment() says
//     how it knows where to start);
//   - the per-voxel passes (the rescale and its extremes, the gradient, the step costs, the
//     smoothing and its local maxima, the sweep's levels) run in pieces spread over the app's
//     compute pool (for_voxels, GS_FOR_VOXELS_OF): each voxel's result depends only on the
//     buffers the pass reads, and the extremes are combined by min and max, so the labels are
//     the same on any number of threads; the marches and the sweep's searches stay sequential;
//   - read_data's min and max start from the first voxel, not from -/+99999999;
//   - three fixes of undefined behaviour: popping an empty heap ends the march (the legacy code
//     read before the heap's arrays, and on some inputs crashed); a heap that would overflow its
//     arrays grows them (the legacy code wrote past their end, which only duplicates of voxels
//     stuck at MAX_TIME can reach); and the threshold sweep stops once the threshold is below 0,
//     where every voxel passes and every later search would find nothing again (the legacy loop
//     went on down to tlow);
//   - with automatic_seeds off, GetSeeds() marks no voxel;
//   - seed_mode::critical_points (critical_points.h, SegMed's seeds): "seeding" appends their two
//     classes to seed_list after the point classes and places their seeds first, so a point
//     class's seed wins a voxel they share.
// The heap is still the legacy binary heap, sift rules and all. Ties are everywhere (every seed
// starts at time 0, every border voxel waits at MAX_TIME + 1), the order in which they come out
// decides the labels where fronts meet, and any other heap breaks the parity.

#include "genseg.h"

#include "critical_points.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cvc/core/thread_pool.h>
#include <mutex>

namespace cvc::genseg {

namespace {

constexpr int gs_max(int x, int y) { return x > y ? x : y; }
constexpr int gs_min(int x, int y) { return x < y ? x : y; }

} // namespace

#define GS_UNIT() GS_UNIT_OF(*this)

void min_heap::allocate(std::size_t n) {
  t.allocate(n);
  v.allocate(n);
  size = 0;
}

void min_heap::release() {
  t.release();
  v.release();
  size = 0;
}

void min_heap::grow() {
  const std::size_t n = t.size(), m = n < 1024 ? 1024 : 2 * n;
  buffer<float> nt;
  buffer<std::size_t> nv;
  nt.allocate(m);
  nv.allocate(m);
  if (n > 0) {
    std::memcpy(nt.data(), t.data(), n * sizeof(float));
    std::memcpy(nv.data(), v.data(), n * sizeof(std::size_t));
  }
  t = std::move(nt);
  v = std::move(nv);
}

void min_heap::push(std::size_t voxel, float time) {
  if (size >= t.size())
    grow();
  std::size_t p = ++size;
  while (p > 1) {
    const std::size_t parent = p / 2;
    if (time < t[parent - 1]) {
      t[p - 1] = t[parent - 1];
      v[p - 1] = v[parent - 1];
      p = parent;
    } else {
      break;
    }
  }
  t[p - 1] = time;
  v[p - 1] = voxel;
}

void min_heap::append(std::size_t voxel, float time) {
  if (size >= t.size())
    grow();
  t[size] = time;
  v[size] = voxel;
  ++size;
}

bool min_heap::pop(std::size_t &voxel, float &time) {
  if (size == 0)
    return false;
  voxel = v[0];
  time = t[0];
  const std::size_t lv = v[size - 1];
  const float lt = t[size - 1];
  --size;
  // Sift the last entry down from the root. A right child at index `size` is the slot the last
  // entry just left, which still holds it: the legacy code read it there too.
  std::size_t p = 1;
  while (p <= size / 2) {
    const std::size_t l = 2 * p, r = 2 * p + 1;
    if (t[l - 1] <= t[r - 1] && t[l - 1] < lt) {
      t[p - 1] = t[l - 1];
      v[p - 1] = v[l - 1];
      p = l;
    } else if (t[l - 1] > t[r - 1] && t[r - 1] < lt) {
      t[p - 1] = t[r - 1];
      v[p - 1] = v[r - 1];
      p = r;
    } else {
      break;
    }
  }
  t[p - 1] = lt;
  v[p - 1] = lv;
  return true;
}

void genseg_context::neighbours(const voxel_at &c, voxel_at out[6]) const {
  const std::size_t sy = std::size_t(XDIM), sz = std::size_t(XDIM) * YDIM;
  out[0] = c.x > 0 ? voxel_at{c.x - 1, c.y, c.z, c.v - 1} : c;
  out[1] = c.x < XDIM - 1 ? voxel_at{c.x + 1, c.y, c.z, c.v + 1} : c;
  out[2] = c.y > 0 ? voxel_at{c.x, c.y - 1, c.z, c.v - sy} : c;
  out[3] = c.y < YDIM - 1 ? voxel_at{c.x, c.y + 1, c.z, c.v + sy} : c;
  out[4] = c.z > 0 ? voxel_at{c.x, c.y, c.z - 1, c.v - sz} : c;
  out[5] = c.z < ZDIM - 1 ? voxel_at{c.x, c.y, c.z + 1, c.v + sz} : c;
}

task genseg_context::run() {
  GS_RUN(normalise());
  GS_RUN(Segment());
}

// read_data's normalisation (inout.cpp): the voxels' minimum and maximum, then every voxel
// rescaled to 0..255. With data_unit_thresholds, tlow and thigh are rescaled the same way.
void for_voxels(thread_pool *pool, std::size_t begin, std::size_t end,
                const std::function<void(std::size_t, std::size_t)> &body) {
  const std::size_t count = end > begin ? end - begin : 0;
  const std::size_t pieces = (count + for_voxels_piece - 1) / for_voxels_piece;
  if (!pool || pieces < 2 || pool->concurrency() < 2) {
    if (count)
      body(begin, end);
    return;
  }
  pool->parallel_for(int(pieces), [&](int i) {
    const std::size_t b = begin + std::size_t(i) * for_voxels_piece;
    body(b, end - b > for_voxels_piece ? b + for_voxels_piece : end);
  });
}

// The minimum and maximum (each piece's, then theirs: the same whatever the order), then every
// voxel rescaled to 0..255.
task genseg_context::normalise() {
  const std::size_t n = voxel_count();
  maxraw = dataset[0];
  minraw = dataset[0];
  std::mutex join;
  const auto extremes = [&](std::size_t b, std::size_t e) {
    float lo = dataset[b], hi = dataset[b];
    for (std::size_t v = b; v < e; v++) {
      if (dataset[v] > hi)
        hi = dataset[v];
      if (dataset[v] < lo)
        lo = dataset[v];
    }
    const std::lock_guard<std::mutex> hold(join);
    if (hi > maxraw)
      maxraw = hi;
    if (lo < minraw)
      minraw = lo;
  };
  GS_FOR_VOXELS_OF(*this, n, extremes);
  const auto rescale = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++)
      dataset[v] = 255 * (dataset[v] - minraw) / (maxraw - minraw);
  };
  GS_FOR_VOXELS_OF(*this, n, rescale);
  if (data_unit_thresholds) {
    tlow = 255 * (tlow - minraw) / (maxraw - minraw);
    thigh = 255 * (thigh - minraw) / (maxraw - minraw);
  }
}

task genseg_context::step_costs() {
  const std::size_t n = voxel_count();
  const auto costs = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++)
      step_cost[v] = 0.01f * (float)std::exp(double(ALPHA * step_cost[v]));
  };
  GS_FOR_VOXELS_OF(*this, n, costs);
}

task genseg_context::Segment() {
  const std::size_t n = voxel_count();
  std::size_t v;
  float t;

  t_high = thigh;

  // The gradient magnitude, then each voxel's step cost in its place.
  step_cost.allocate(n);
  GS_RUN(GetGradient(step_cost.data()));
  GS_RUN(step_costs());

  begin("seeding");
  seed_index.allocate(n);
  GS_RUN(GetSeeds(seed_index.data()));

  // seed_mode::critical_points: the feature and background classes after the point classes'.
  const std::size_t user_classes = seed_list.size();
  if (critical) {
    GS_RUN(critical->run(*this));
    for (int c = 0; c < 2; c++) {
      const std::vector<std::size_t> &cls = c == 0 ? critical->feature : critical->background;
      seed_list.emplace_back();
      seed_list.back().reserve(cls.size());
      for (std::size_t s = 0; s < cls.size(); s++) {
        const voxel_at p = at(cls[s]);
        seed_list.back().push_back(INTVEC{p.x, p.y, p.z});
        GS_UNIT();
      }
    }
  }

  heap.allocate(n);
  tdata.allocate(n);
  SeedNum = 0;
  for (v = 0; v < n; v++) {
    tdata[v] = MAX_TIME;
    if (seed_index[v] == 254) {
      SeedNum++;
      unclassified.push_back(v);
    }
    GS_UNIT();
  }
  seeds_found = SeedNum;

  // The point classes' seeds. (The legacy code also pushed them on the heap, which the sweep
  // and the final march each cleared before using it.) The critical points' classes, after the
  // point classes in seed_list, are placed first, so a point class's seed wins a shared voxel.
  for (std::size_t o = 0; o < seed_list.size(); o++) {
    const std::size_t num = (o + user_classes) % seed_list.size();
    for (const INTVEC &seed : seed_list[num]) {
      v = (std::size_t(seed.z) * YDIM + std::size_t(seed.y)) * XDIM + std::size_t(seed.x);
      if (seed_index[v] == 254) {
        SeedNum--;
        seeds_displaced++;
      }
      seed_index[v] = (unsigned char)num;
      tdata[v] = 0.0f;
      GS_UNIT();
    }
  }

  // The threshold sweep: from 255 down to tlow, each automatic seed takes the class of the first
  // labelled front its own front reaches through voxels at or above the threshold; the class then
  // spreads from it through voxels at or above that threshold.
  //
  // After each seed it classifies, the legacy sweep went back up to 255 and searched again at
  // each threshold on the way down; a search that reaches no labelled voxel changes nothing, and
  // most of them reached none. A search at threshold t can reach one only if, through voxels a
  // front passes at t, some unclassified seed connects to a voxel next to a labelled one (a path
  // through labelled voxels counts too: the voxel before the first of them is next to it). The
  // component tree of those voxels by level (sweep_tree()) says, with the labels so far
  // (sweep_labelled()), the highest such t (reach_level()), and the sweep starts there. Every
  // search it skips would have found nothing, so the labels are the legacy ones. (A search can
  // still find nothing where the tree saw a path, when the times along it pass MAX_TIME or the
  // walk back to its seed sticks; then the sweep goes down a threshold at a time, as before.)
  begin("classifying");
  if (SeedNum > 0) {
    T_heap.allocate(n);
    T_tdata.allocate(n);
    const auto clear = [&](std::size_t b, std::size_t e) {
      for (std::size_t u = b; u < e; u++)
        T_tdata[u] = MAX_TIME;
    };
    GS_FOR_VOXELS_OF(*this, n, clear);
    GS_RUN(sweep_tree());
  }
  t_low = 255.0f;
  bool jump = sweep_shortcut;
  log_labels = sweep_shortcut;
  while (SeedNum > 0) {
    if (jump) {
      jump = false;
      const int level = reach_level();
      if (level < 0)
        break; // every search down to tlow would find nothing
      // level >= sweep_floor >= tlow and >= 0, so the legacy loop came down to it.
      if (level < 255)
        t_low = float(level);
    }
    GS_RUN(SearchIndex());
    if (search_index == 255) {
      t_low = t_low - 1.0f;
      if (t_low < tlow)
        break;
      // Below 0 no voxel is below the threshold, so every later search finds what this one did,
      // nothing, and changes nothing: the legacy loop went on down to tlow for nothing.
      else if (t_low < -1.0f)
        break;
      else
        continue;
    }
    t_high = t_low;
    t_low = 255.0f;
    jump = sweep_shortcut;

    v = search_voxel;
    heap.clear();
    tdata[v] = 0.0f;
    if (seed_index[v] == 254) {
      SeedNum--;
      if (log_labels)
        classified_log.push_back(std::uint32_t(v));
    }
    seed_index[v] = search_index;
    if (log_labels)
      labelled_log.push_back(std::uint32_t(v));
    heap.push(v, 0.0f);
    while (heap.pop(v, t) && t < MAX_TIME) {
      GetTime(at(v));
      GS_UNIT();
    }
    if (log_labels)
      GS_RUN(sweep_labelled());
  }
  log_labels = false;
  seeds_left = SeedNum;
  tree_level = {};
  tree_parent = {};
  tree_seeds = {};
  tree_marked = {};
  labelled_log = {};
  classified_log = {};

  // The final march: every labelled voxel starts a front at time 0, through voxels at or above
  // tlow.
  begin("marching");
  heap.clear();
  for (v = 0; v < n; v++) {
    tdata[v] = MAX_TIME;
    if (seed_index[v] < 254) {
      heap.append(v, 0.0f);
      tdata[v] = 0.0f;
    }
    GS_UNIT();
  }
  t_high = tlow;
  while (heap.pop(v, t) && t < MAX_TIME) {
    GetTime(at(v));
    marched++;
    GS_UNIT();
  }

  // What only the march needed goes back now.
  heap.release();
  T_heap.release();
  T_tdata.release();
  step_cost.release();
  unclassified = {};
  touched = {};
}

// The class of the first labelled front that the unclassified automatic seeds' fronts reach,
// through voxels at or above t_low, and the seed that front came from: search_index (255: none)
// and search_voxel.
task genseg_context::SearchIndex() {
  int i, j, k;
  int x0, y0, z0;
  int x1, y1, z1;
  float t;
  std::size_t v;

  // Clear the times the previous search set.
  for (std::size_t u : touched) {
    T_tdata[u] = MAX_TIME;
    GS_UNIT();
  }
  touched.clear();

  // The seeds still unclassified start at time 0, in the order they were found.
  T_heap.clear();
  std::size_t kept = 0;
  for (std::size_t s = 0; s < unclassified.size(); s++) {
    v = unclassified[s];
    if (seed_index[v] == 254) {
      unclassified[kept++] = v;
      T_heap.append(v, 0.0f);
      T_tdata[v] = 0.0f;
      touched.push_back(v);
    }
    GS_UNIT();
  }
  unclassified.resize(kept);

  T_index = 255;
  voxel_at m{};
  while (true) {
    if (!T_heap.pop(v, t) || t >= MAX_TIME) {
      search_index = 255;
      co_return;
    }
    m = at(v);
    T_GetTime(m);
    GS_UNIT();
    if (T_index < 254)
      break;
  }

  // Walk down the times from where the front met the labelled one, to the seed it came from.
  x0 = m.x;
  y0 = m.y;
  z0 = m.z;
  x1 = x0;
  y1 = y0;
  z1 = z0;
  t = T_tdata[m.v];
  const std::size_t sy = std::size_t(XDIM), sz = std::size_t(XDIM) * YDIM;
  while (t > 0) {
    for (k = gs_max(0, z0 - 1); k <= gs_min(ZDIM - 1, z0 + 1); k++)
      for (j = gs_max(0, y0 - 1); j <= gs_min(YDIM - 1, y0 + 1); j++)
        for (i = gs_max(0, x0 - 1); i <= gs_min(XDIM - 1, x0 + 1); i++) {
          const float u = T_tdata[std::size_t(k) * sz + std::size_t(j) * sy + std::size_t(i)];
          if (u < t) {
            t = u;
            x1 = i;
            y1 = j;
            z1 = k;
          }
        }
    if (x0 == x1 && y0 == y1 && z0 == z1) {
      search_index = 255;
      co_return;
    }
    x0 = x1;
    y0 = y1;
    z0 = z1;
    t = T_tdata[std::size_t(z0) * sz + std::size_t(y0) * sy + std::size_t(x0)];
    GS_UNIT();
  }
  search_voxel = std::size_t(z0) * sz + std::size_t(y0) * sy + std::size_t(x0);
  search_index = T_index;
}

// One voxel popped by a march: each face neighbour not yet reached and not labelled takes the
// popped voxel's class, and an arrival time from its own neighbours, or MAX_TIME + 1 (a border:
// the front stops there) when one of them is below t_high or of another class.
void genseg_context::GetTime(const voxel_at &m) {
  const unsigned char index = seed_index[m.v];
  voxel_at nb[6], nn[6];
  neighbours(m, nb);
  for (const voxel_at &c : nb) {
    if (tdata[c.v] != MAX_TIME || seed_index[c.v] < 254)
      continue;
    bool boundary = false;
    float value = MAX_TIME;
    neighbours(c, nn);
    for (const voxel_at &u : nn) {
      if (dataset[u.v] < t_high || (seed_index[u.v] < 254 && seed_index[u.v] != index))
        boundary = true;
      else if (value > step_cost[u.v] + tdata[u.v])
        value = step_cost[u.v] + tdata[u.v];
    }
    tdata[c.v] = boundary ? MAX_TIME + 1 : value;
    if (seed_index[c.v] == 254) {
      SeedNum--;
      if (log_labels)
        classified_log.push_back(std::uint32_t(c.v));
    }
    seed_index[c.v] = index;
    if (log_labels)
      labelled_log.push_back(std::uint32_t(c.v));
    heap.push(c.v, tdata[c.v]);
  }
}

// One voxel popped by the sweep's search: the first face neighbour with a class ends the search
// with that class (T_index); each other one not yet reached gets a time from its own neighbours,
// or MAX_TIME + 1 when one of them is below t_low.
void genseg_context::T_GetTime(const voxel_at &m) {
  voxel_at nb[6], nn[6];
  neighbours(m, nb);
  for (const voxel_at &c : nb) {
    if (seed_index[c.v] < 254) {
      T_index = seed_index[c.v];
      return;
    }
    if (T_tdata[c.v] != MAX_TIME)
      continue;
    bool boundary = false;
    float value = MAX_TIME;
    neighbours(c, nn);
    for (const voxel_at &u : nn) {
      if (dataset[u.v] < t_low)
        boundary = true;
      else if (value > step_cost[u.v] + T_tdata[u.v])
        value = step_cost[u.v] + T_tdata[u.v];
    }
    T_tdata[c.v] = boundary ? MAX_TIME + 1 : value;
    touched.push_back(c.v);
    T_heap.push(c.v, T_tdata[c.v]);
  }
}

// v's face neighbours inside the volume (neighbours() stands v in for the missing ones).
int genseg_context::face_neighbours(std::size_t v, std::size_t out[6]) const {
  voxel_at nb[6];
  neighbours(at(v), nb);
  int k = 0;
  for (const voxel_at &u : nb)
    if (u.v != v)
      out[k++] = u.v;
  return k;
}

// The component tree of the voxels a search front can pass (Segment()'s threshold sweep). A
// front passes voxel c at threshold t when every value neighbours(c) gives is at least t
// (T_GetTime()): at every threshold up to c's level, the floor of the least of them. The
// automatic seeds are at 256 (fronts start there at any threshold); levels below sweep_floor,
// where the sweep never searches, are left out. Built as Berger, Geraud, Levillain, Widynski and
// Baillard, "Effective component tree computation with application to pattern recognition in
// astronomical imaging" (ICIP 2007): the voxels come in highest level first, each becoming the
// parent of the roots of its neighbours' sets so far; then, from the root down, every voxel's
// parent moves up to the voxel that names its node. Then the unclassified seeds are counted in
// every node above theirs, and the voxels next to a labelled one (the point classes' seeds) mark
// theirs.
task genseg_context::sweep_tree() {
  const std::size_t n = voxel_count();
  constexpr std::uint32_t none = 0xffffffffu;
  sweep_shortcut = n < std::size_t(none);
  if (!sweep_shortcut)
    co_return;
  // The sweep searches at 255 (even when tlow is above it), then on down to tlow; below 0 every
  // voxel passes, as at 0.
  sweep_floor = int(std::ceil(double(tlow)));
  sweep_floor = sweep_floor < 0 ? 0 : (sweep_floor > 255 ? 255 : sweep_floor);
  tree_level.assign(n, tree_out);
  const auto levels = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++) {
      int l = 256;
      if (seed_index[v] != 254) {
        voxel_at nb[6];
        neighbours(at(v), nb);
        float least = dataset[nb[0].v];
        for (int i = 1; i < 6; i++)
          if (dataset[nb[i].v] < least)
            least = dataset[nb[i].v];
        l = least >= 255.0f ? 255 : (least < 0.0f ? -1 : int(std::floor(least)));
      }
      if (l >= sweep_floor)
        tree_level[v] = std::uint16_t(l);
    }
  };
  GS_FOR_VOXELS_OF(*this, n, levels);
  std::size_t at_level[257] = {};
  for (std::size_t v = 0; v < n; v++) {
    if (tree_level[v] != tree_out)
      at_level[tree_level[v]]++;
    GS_UNIT();
  }
  std::size_t next[257] = {}, total = 0;
  for (int l = 256; l >= sweep_floor; l--) {
    next[l] = total;
    total += at_level[l];
  }
  std::vector<std::uint32_t> order(total);
  for (std::size_t v = 0; v < n; v++) {
    if (tree_level[v] != tree_out)
      order[next[tree_level[v]]++] = std::uint32_t(v);
    GS_UNIT();
  }

  tree_parent.assign(n, none);
  std::vector<std::uint32_t> zpar(n, none);
  for (const std::uint32_t p : order) {
    tree_parent[p] = p;
    zpar[p] = p;
    std::size_t nb[6];
    const int k = face_neighbours(p, nb);
    for (int i = 0; i < k; i++) {
      if (zpar[nb[i]] == none)
        continue;
      std::uint32_t r = std::uint32_t(nb[i]);
      while (zpar[r] != r) {
        zpar[r] = zpar[zpar[r]];
        r = zpar[r];
      }
      if (r != p) {
        tree_parent[r] = p;
        zpar[r] = p;
      }
    }
    GS_UNIT();
  }
  for (std::size_t i = total; i-- > 0;) {
    const std::uint32_t p = order[i], q = tree_parent[p];
    if (tree_level[tree_parent[q]] == tree_level[q])
      tree_parent[p] = tree_parent[q];
    GS_UNIT();
  }
  order = {};

  tree_seeds = std::move(zpar);
  std::fill(tree_seeds.begin(), tree_seeds.end(), 0u);
  tree_marked.assign(n, 0);
  for (std::size_t &q : qualifying)
    q = 0;
  for (const std::size_t s : unclassified) {
    if (seed_index[s] == 254)
      for (std::uint32_t c = tree_node(std::uint32_t(s));; c = tree_parent[c]) {
        tree_seeds[c]++;
        if (tree_parent[c] == c)
          break;
      }
    GS_UNIT();
  }
  for (std::size_t v = 0; v < n; v++) {
    if (seed_index[v] < 254) {
      std::size_t nb[6];
      const int k = face_neighbours(v, nb);
      for (int i = 0; i < k; i++)
        if (tree_level[nb[i]] != tree_out)
          mark_up(std::uint32_t(nb[i]));
    }
    GS_UNIT();
  }
}

// The voxel that names v's node: v's parent when it is at v's level (not at a root), else v.
std::uint32_t genseg_context::tree_node(std::uint32_t v) const {
  const std::uint32_t p = tree_parent[v];
  return p != v && tree_level[p] == tree_level[v] ? p : v;
}

// v, in the tree, is next to a labelled voxel: its node and those above it hold one. Each node is
// marked once, so all the marking of a run costs a pass over the tree.
void genseg_context::mark_up(std::uint32_t v) {
  for (std::uint32_t c = tree_node(v); !tree_marked[c]; c = tree_parent[c]) {
    tree_marked[c] = 1;
    if (tree_seeds[c] > 0)
      qualifying[tree_level[c]]++;
    if (tree_parent[c] == c)
      break;
  }
}

// What the sweep labelled since the last call: each automatic seed among it leaves the count of
// every node above its own (at most 257 of them), and each labelled voxel's neighbours mark
// theirs.
task genseg_context::sweep_labelled() {
  for (const std::uint32_t s : classified_log) {
    for (std::uint32_t c = tree_node(s);; c = tree_parent[c]) {
      if (--tree_seeds[c] == 0 && tree_marked[c])
        qualifying[tree_level[c]]--;
      if (tree_parent[c] == c)
        break;
    }
    GS_UNIT();
  }
  classified_log.clear();
  for (const std::uint32_t v : labelled_log) {
    std::size_t nb[6];
    const int k = face_neighbours(v, nb);
    for (int i = 0; i < k; i++)
      if (tree_level[nb[i]] != tree_out)
        mark_up(std::uint32_t(nb[i]));
    GS_UNIT();
  }
  labelled_log.clear();
}

// The highest threshold at which SearchIndex() would reach a labelled voxel, MAX_TIME aside: the
// highest level with a node that holds both an unclassified seed and a voxel next to a labelled
// one. A search front starts at every unclassified seed and passes exactly the voxels at or above
// its threshold, so at level l it covers the nodes of level l that hold a seed, and it ends at the
// first labelled voxel next to a voxel it reached (256: a seed is next to one itself). -1 when no
// level down to sweep_floor has such a node.
int genseg_context::reach_level() const {
  for (int l = 256; l >= sweep_floor; l--)
    if (qualifying[l] > 0)
      return l;
  return -1;
}

// Each voxel's gradient magnitude (the legacy 3x3x3 stencil, in the legacy order), and their
// minimum and maximum (each piece's, then theirs); then every one rescaled to 0..255.
task genseg_context::GetGradient(float *edge_mag) {
  float maxgrad = 0, mingrad = 999999.0;
  std::mutex join;
  const auto gradients = [&](std::size_t b, std::size_t e) {
    float lo = 999999.0, hi = 0;
    const std::size_t xy = std::size_t(XDIM) * YDIM;
    for (std::size_t v = b; v < e; v++) {
      const int x = int(v % XDIM), y = int((v / XDIM) % YDIM), z = int(v / xy);
      int i, j, k;
      float grad_x, grad_y, grad_z;
      grad_x = 0.0;
      for (j = gs_max(y - 1, 0); j <= gs_min(y + 1, YDIM - 1); j++)
        for (k = gs_max(z - 1, 0); k <= gs_min(z + 1, ZDIM - 1); k++) {
          grad_x += dataset[IndexVect(gs_min(x + 1, XDIM - 1), j, k)] -
                    dataset[IndexVect(gs_max(x - 1, 0), j, k)];
          if (j == y || k == z)
            grad_x += dataset[IndexVect(gs_min(x + 1, XDIM - 1), j, k)] -
                      dataset[IndexVect(gs_max(x - 1, 0), j, k)];
          if (j == y && k == z)
            grad_x += 2.0f * (dataset[IndexVect(gs_min(x + 1, XDIM - 1), j, k)] -
                              dataset[IndexVect(gs_max(x - 1, 0), j, k)]);
        }

      grad_y = 0.0;
      for (i = gs_max(x - 1, 0); i <= gs_min(x + 1, XDIM - 1); i++)
        for (k = gs_max(z - 1, 0); k <= gs_min(z + 1, ZDIM - 1); k++) {
          grad_y += dataset[IndexVect(i, gs_min(y + 1, YDIM - 1), k)] -
                    dataset[IndexVect(i, gs_max(y - 1, 0), k)];
          if (i == x || k == z)
            grad_y += dataset[IndexVect(i, gs_min(y + 1, YDIM - 1), k)] -
                      dataset[IndexVect(i, gs_max(y - 1, 0), k)];
          if (i == x && k == z)
            grad_y += 2.0f * (dataset[IndexVect(i, gs_min(y + 1, YDIM - 1), k)] -
                              dataset[IndexVect(i, gs_max(y - 1, 0), k)]);
        }

      grad_z = 0.0;
      for (i = gs_max(x - 1, 0); i <= gs_min(x + 1, XDIM - 1); i++)
        for (j = gs_max(y - 1, 0); j <= gs_min(y + 1, YDIM - 1); j++) {
          grad_z += dataset[IndexVect(i, j, gs_min(z + 1, ZDIM - 1))] -
                    dataset[IndexVect(i, j, gs_max(z - 1, 0))];
          if (i == x || j == y)
            grad_z += dataset[IndexVect(i, j, gs_min(z + 1, ZDIM - 1))] -
                      dataset[IndexVect(i, j, gs_max(z - 1, 0))];
          if (i == x && j == y)
            grad_z += 2.0f * (dataset[IndexVect(i, j, gs_min(z + 1, ZDIM - 1))] -
                              dataset[IndexVect(i, j, gs_max(z - 1, 0))]);
        }

      const float gradient = (float)std::sqrt(grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
      edge_mag[v] = gradient;
      if (gradient < lo)
        lo = gradient;
      if (gradient > hi)
        hi = gradient;
    }
    const std::lock_guard<std::mutex> hold(join);
    if (lo < mingrad)
      mingrad = lo;
    if (hi > maxgrad)
      maxgrad = hi;
  };
  GS_FOR_VOXELS_OF(*this, voxel_count(), gradients);

  if (mingrad < maxgrad) {
    const auto rescale = [&](std::size_t b, std::size_t e) {
      for (std::size_t v = b; v < e; v++)
        edge_mag[v] = 255.0f * (edge_mag[v] - mingrad) / (maxgrad - mingrad);
    };
    GS_FOR_VOXELS_OF(*this, voxel_count(), rescale);
  }
}

// The automatic seeds: the local maxima above t_high of the volume smoothed twice (each pass
// reads one buffer and writes the other, so a voxel's value never depends on its neighbours'
// order).
task genseg_context::GetSeeds(unsigned char *result) {
  const std::size_t n = voxel_count();
  if (!automatic_seeds) {
    const auto none = [&](std::size_t b, std::size_t e) {
      for (std::size_t v = b; v < e; v++)
        result[v] = 255;
    };
    GS_FOR_VOXELS_OF(*this, n, none);
    co_return;
  }

  buffer<float> smth_img, temp;
  smth_img.allocate(n);
  temp.allocate(n);
  const std::size_t xy = std::size_t(XDIM) * YDIM;
  const auto copy_in = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++)
      smth_img[v] = dataset[v];
  };
  GS_FOR_VOXELS_OF(*this, n, copy_in);

  const auto smooth = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++) {
      const int i = int(v % XDIM), j = int((v / XDIM) % YDIM), k = int(v / xy);
      temp[v] = 0.4f * smth_img[IndexVect(i, j, k)] +
                0.1f * (smth_img[IndexVect(gs_max(0, i - 1), j, k)] +
                        smth_img[IndexVect(gs_min(XDIM - 1, i + 1), j, k)] +
                        smth_img[IndexVect(i, gs_max(0, j - 1), k)] +
                        smth_img[IndexVect(i, gs_min(YDIM - 1, j + 1), k)] +
                        smth_img[IndexVect(i, j, gs_max(0, k - 1))] +
                        smth_img[IndexVect(i, j, gs_min(ZDIM - 1, k + 1))]);
    }
  };
  const auto copy_back = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++)
      smth_img[v] = temp[v];
  };
  for (int u = 0; u < 2; u++) {
    GS_FOR_VOXELS_OF(*this, n, smooth);
    GS_FOR_VOXELS_OF(*this, n, copy_back);
  }

  const auto maxima = [&](std::size_t b, std::size_t e) {
    for (std::size_t v = b; v < e; v++) {
      const int i = int(v % XDIM), j = int((v / XDIM) % YDIM), k = int(v / xy);
      result[v] = 255;
      const float tmp = smth_img[v];
      if (tmp > t_high) {
        int u = 0;
        for (int l = gs_max(k - 1, 0); l <= gs_min(k + 1, ZDIM - 1); l++)
          for (int nn = gs_max(j - 1, 0); nn <= gs_min(j + 1, YDIM - 1); nn++)
            for (int m = gs_max(i - 1, 0); m <= gs_min(i + 1, XDIM - 1); m++) {
              if (tmp < smth_img[IndexVect(m, nn, l)])
                u = 1;
            }
        if (u == 0)
          result[v] = 254;
      }
    }
  };
  GS_FOR_VOXELS_OF(*this, n, maxima);
}

} // namespace cvc::genseg
