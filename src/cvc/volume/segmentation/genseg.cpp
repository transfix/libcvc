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
//   - read_data's min and max start from the first voxel, not from -/+99999999;
//   - three fixes of undefined behaviour: popping an empty heap ends the march (the legacy code
//     read before the heap's arrays, and on some inputs crashed); a heap that would overflow its
//     arrays grows them (the legacy code wrote past their end, which only duplicates of voxels
//     stuck at MAX_TIME can reach); and the threshold sweep stops once the threshold is below 0,
//     where every voxel passes and every later search would find nothing again (the legacy loop
//     went on down to tlow);
//   - with automatic_seeds off, GetSeeds() marks no voxel.
// The heap is still the legacy binary heap, sift rules and all. Ties are everywhere (every seed
// starts at time 0, every border voxel waits at MAX_TIME + 1), the order in which they come out
// decides the labels where fronts meet, and any other heap breaks the parity.

#include "genseg.h"

#include <cmath>
#include <cstring>

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
task genseg_context::normalise() {
  const std::size_t n = voxel_count();
  maxraw = dataset[0];
  minraw = dataset[0];
  for (std::size_t v = 0; v < n; v++) {
    if (dataset[v] > maxraw)
      maxraw = dataset[v];
    if (dataset[v] < minraw)
      minraw = dataset[v];
    GS_UNIT();
  }
  for (std::size_t v = 0; v < n; v++) {
    dataset[v] = 255 * (dataset[v] - minraw) / (maxraw - minraw);
    GS_UNIT();
  }
  if (data_unit_thresholds) {
    tlow = 255 * (tlow - minraw) / (maxraw - minraw);
    thigh = 255 * (thigh - minraw) / (maxraw - minraw);
  }
}

task genseg_context::step_costs() {
  const std::size_t n = voxel_count();
  for (std::size_t v = 0; v < n; v++) {
    step_cost[v] = 0.01f * (float)std::exp(double(ALPHA * step_cost[v]));
    GS_UNIT();
  }
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
  // and the final march each cleared before using it.)
  for (std::size_t num = 0; num < seed_list.size(); num++) {
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
  begin("classifying");
  if (SeedNum > 0) {
    T_heap.allocate(n);
    T_tdata.allocate(n);
    for (v = 0; v < n; v++) {
      T_tdata[v] = MAX_TIME;
      GS_UNIT();
    }
  }
  t_low = 255.0f;
  while (SeedNum > 0) {
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

    v = search_voxel;
    heap.clear();
    tdata[v] = 0.0f;
    if (seed_index[v] == 254)
      SeedNum--;
    seed_index[v] = search_index;
    heap.push(v, 0.0f);
    while (heap.pop(v, t) && t < MAX_TIME) {
      GetTime(at(v));
      GS_UNIT();
    }
  }
  seeds_left = SeedNum;

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
    if (seed_index[c.v] == 254)
      SeedNum--;
    seed_index[c.v] = index;
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

task genseg_context::GetGradient(float *edge_mag) {
  int x, y, z;
  int i, j, k;
  float gradient, grad_x, grad_y, grad_z;
  float maxgrad, mingrad;

  maxgrad = 0;
  mingrad = 999999.0;
  for (z = 0; z < ZDIM; z++)
    for (y = 0; y < YDIM; y++)
      for (x = 0; x < XDIM; x++) {
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

        gradient = (float)std::sqrt(grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
        edge_mag[IndexVect(x, y, z)] = gradient;

        if (gradient < mingrad)
          mingrad = gradient;
        if (gradient > maxgrad)
          maxgrad = gradient;
        GS_UNIT();
      }

  if (mingrad < maxgrad) {
    for (z = 0; z < ZDIM; z++)
      for (y = 0; y < YDIM; y++)
        for (x = 0; x < XDIM; x++) {

          edge_mag[IndexVect(x, y, z)] =
              255.0f * (edge_mag[IndexVect(x, y, z)] - mingrad) / (maxgrad - mingrad);
          GS_UNIT();
        }
  }
}

task genseg_context::GetSeeds(unsigned char *result) {
  int i, j, k;
  int m, n, l, u;
  float tmp;
  buffer<float> smth_img, temp;

  if (!automatic_seeds) {
    for (std::size_t v = 0; v < voxel_count(); v++) {
      result[v] = 255;
      GS_UNIT();
    }
    co_return;
  }

  smth_img.allocate(voxel_count());
  temp.allocate(voxel_count());
  for (k = 0; k < ZDIM; k++)
    for (j = 0; j < YDIM; j++)
      for (i = 0; i < XDIM; i++) {
        smth_img[IndexVect(i, j, k)] = dataset[IndexVect(i, j, k)];
        GS_UNIT();
      }

  for (u = 0; u < 2; u++) {
    for (k = 0; k < ZDIM; k++)
      for (j = 0; j < YDIM; j++)
        for (i = 0; i < XDIM; i++) {
          temp[IndexVect(i, j, k)] = 0.4f * smth_img[IndexVect(i, j, k)] +
                                     0.1f * (smth_img[IndexVect(gs_max(0, i - 1), j, k)] +
                                             smth_img[IndexVect(gs_min(XDIM - 1, i + 1), j, k)] +
                                             smth_img[IndexVect(i, gs_max(0, j - 1), k)] +
                                             smth_img[IndexVect(i, gs_min(YDIM - 1, j + 1), k)] +
                                             smth_img[IndexVect(i, j, gs_max(0, k - 1))] +
                                             smth_img[IndexVect(i, j, gs_min(ZDIM - 1, k + 1))]);
          GS_UNIT();
        }
    for (k = 0; k < ZDIM; k++)
      for (j = 0; j < YDIM; j++)
        for (i = 0; i < XDIM; i++) {
          smth_img[IndexVect(i, j, k)] = temp[IndexVect(i, j, k)];
          GS_UNIT();
        }
  }

  for (k = 0; k < ZDIM; k++)
    for (j = 0; j < YDIM; j++)
      for (i = 0; i < XDIM; i++) {

        result[IndexVect(i, j, k)] = 255;
        tmp = smth_img[IndexVect(i, j, k)];
        if (tmp > t_high) {
          u = 0;
          for (l = gs_max(k - 1, 0); l <= gs_min(k + 1, ZDIM - 1); l++)
            for (n = gs_max(j - 1, 0); n <= gs_min(j + 1, YDIM - 1); n++)
              for (m = gs_max(i - 1, 0); m <= gs_min(i + 1, XDIM - 1); m++) {
                if (tmp < smth_img[IndexVect(m, n, l)])
                  u = 1;
              }

          if (u == 0)
            result[IndexVect(i, j, k)] = 254;
        }
        GS_UNIT();
      }
}

} // namespace cvc::genseg
