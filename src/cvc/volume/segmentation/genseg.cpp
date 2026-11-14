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
// tools/volrover-goldens/volrover.sha) and read_data's normalisation (inout.cpp), vendored as
// genseg_context (genseg.h). What changed, and only that:
//   - the globals are the context's members, and the functions its member functions;
//   - Segment(), GetGradient(), GetSeeds() and SearchIndex() (with InitialTime() inlined) are
//     coroutines that yield a unit at a time (GS_UNIT): a voxel visited, a seed placed, a voxel
//     popped from a heap, a step of a walk;
//   - Segment() reads no file and writes none: its input is the context's dataset, and its
//     outputs (seed_index, and tdata's MAX_TIME + 1 border marks) are left in the context for
//     cvc::segment_stepper to turn into volumes; the printf()s are gone;
//   - malloc()ed arrays are buffers, and the seed lists vectors (their pointers are indices);
//   - indices are computed in std::size_t (IndexVect), so a volume of more than 2^31 voxels does
//     not overflow;
//   - the max()/min() macros are the functions gs_max()/gs_min(), and exp() is computed in
//     double (gs_exp) on every platform;
//   - read_data's min and max start from the first voxel, not from -/+99999999;
//   - popping an empty heap ends the march (the legacy code read before the heap's arrays), and
//     a heap that would overflow its arrays grows them (the legacy code wrote past their end;
//     only duplicates of voxels stuck at MAX_TIME can do that);
//   - with automatic_seeds off, GetSeeds() marks no voxel;
//   - the threshold sweep stops once the threshold is below 0, where every voxel passes and each
//     later search would find nothing again (the legacy loop went on down to tlow, the same
//     result after up to millions of searches for a tlow far below the data).
// Everything else, the hand-unrolled neighbour updates included, is as it was, so the labels
// match the legacy library's voxel for voxel (src/cvc/tests/segmentation_test.cpp, against the
// goldens tools/volrover-goldens made).

#include "genseg.h"

#include <cmath>
#include <cstring>

// The legacy tests read `a < t || b < 254 && b != index`, which is `a < t || (b < 254 && ...)`;
// they stay as they were.
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wparentheses"
#endif

namespace cvc::genseg {

namespace {

constexpr int gs_max(int x, int y) { return x > y ? x : y; }
constexpr int gs_min(int x, int y) { return x < y ? x : y; }
// exp() in double, cast to float where the legacy code cast it: the same on every platform's
// libm to the last bit of the float, which a float exp() is not.
inline double gs_exp(float x) { return std::exp(double(x)); }

} // namespace

#define GS_UNIT() GS_UNIT_OF(*this)

void MinHeapS::allocate(std::size_t n) {
  x.allocate(n);
  y.allocate(n);
  z.allocate(n);
  t.allocate(n);
  size = 0;
}

void MinHeapS::grow() {
  const std::size_t n = t.size(), m = n < 1024 ? 1024 : 2 * n;
  buffer<int> nx, ny, nz;
  buffer<float> nt;
  nx.allocate(m);
  ny.allocate(m);
  nz.allocate(m);
  nt.allocate(m);
  if (n > 0) {
    std::memcpy(nx.data(), x.data(), n * sizeof(int));
    std::memcpy(ny.data(), y.data(), n * sizeof(int));
    std::memcpy(nz.data(), z.data(), n * sizeof(int));
    std::memcpy(nt.data(), t.data(), n * sizeof(float));
  }
  x = std::move(nx);
  y = std::move(ny);
  z = std::move(nz);
  t = std::move(nt);
}

void MinHeapS::release() {
  x.release();
  y.release();
  z.release();
  t.release();
  size = 0;
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

task genseg_context::Segment() {
  int i, j, k;
  int num;
  int Seed_x, Seed_y, Seed_z;
  unsigned char index;
  int critical_end;
  int critical_start;

  t_high = thigh;

  ImgGrad.allocate(voxel_count());
  GS_RUN(GetGradient(ImgGrad.data()));

  begin("seeding");
  seed_index.allocate(voxel_count());
  GS_RUN(GetSeeds(seed_index.data()));

  min_heap.allocate(voxel_count());
  tdata.allocate(voxel_count());
  T_min_heap.allocate(voxel_count());
  T_tdata.allocate(voxel_count());
  IndexArray.allocate(voxel_count());

  SeedNum = 0;
  critical_start = -1;
  critical_end = -1;
  for (k = 0; k < ZDIM; k++)
    for (j = 0; j < YDIM; j++)
      for (i = 0; i < XDIM; i++) {
        tdata[IndexVect(i, j, k)] = MAX_TIME;
        T_tdata[IndexVect(i, j, k)] = MAX_TIME;

        if (seed_index[IndexVect(i, j, k)] == 254) {
          SeedNum++;
          critical.push_back(SDPNT{i, j, k, -1});
          if (critical_start == -1) {
            critical_end = int(critical.size()) - 1;
            critical_start = critical_end;
          } else {
            critical[critical_end].next = int(critical.size()) - 1;
            critical_end = critical[critical_end].next;
          }
        }
        GS_UNIT();
      }
  seeds_found = SeedNum;

  PhaseNum = 1;
  min_heap.size = 0;

  for (num = 0; num < int(seed_list.size()); num++) {
    for (const INTVEC &seed_tmp : seed_list[num]) {
      Seed_x = seed_tmp.x;
      Seed_y = seed_tmp.y;
      Seed_z = seed_tmp.z;
      if (seed_index[IndexVect(Seed_x, Seed_y, Seed_z)] == 254) {
        SeedNum--;
        seeds_displaced++;
      }
      seed_index[IndexVect(Seed_x, Seed_y, Seed_z)] = num;
      tdata[IndexVect(Seed_x, Seed_y, Seed_z)] = 0.0;
      InsertHeap(Seed_x, Seed_y, Seed_z);
      GS_UNIT();
    }
  }

  begin("classifying");
  t_low = 255.0f;
  while (SeedNum > 0) {

    GS_RUN(SearchIndex(critical_start));
    index = search_index;
    i = search_m;
    j = search_n;
    k = search_l;
    if (index == 255) {
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

    min_heap.size = 0;
    tdata[IndexVect(i, j, k)] = 0.0;
    if (seed_index[IndexVect(i, j, k)] == 254)
      SeedNum--;
    seed_index[IndexVect(i, j, k)] = index;
    InsertHeap(i, j, k);

    while (1) {
      GetMinimum();
      if (min_t >= MAX_TIME)
        break;
      GetTime();
      GS_UNIT();
    }
  }

  seeds_left = SeedNum;
  begin("marching");
  min_heap.size = 0;
  for (k = 0; k < ZDIM; k++)
    for (j = 0; j < YDIM; j++)
      for (i = 0; i < XDIM; i++) {
        tdata[IndexVect(i, j, k)] = MAX_TIME;
        if (seed_index[IndexVect(i, j, k)] < 254) {
          min_heap.x[min_heap.size] = i;
          min_heap.y[min_heap.size] = j;
          min_heap.z[min_heap.size] = k;
          min_heap.t[min_heap.size] = 0.0;
          min_heap.size++;
          tdata[IndexVect(i, j, k)] = 0.0;
        }
        GS_UNIT();
      }

  PhaseNum = 2;
  t_high = tlow;
  while (1) {

    GetMinimum();

    if (min_t >= MAX_TIME)
      break;
    GetTime();
    marched++;
    GS_UNIT();
  }

  // The legacy code wrote its outputs here; the stepper reads them from the context. What only
  // the march needed goes back now.
  min_heap.release();
  T_min_heap.release();
  T_tdata.release();
  IndexArray.release();
  ImgGrad.release();
  critical.clear();
  critical.shrink_to_fit();
}

// The class of the first voxel of a labelled front that the automatic seeds' fronts reach,
// through voxels at or above t_low, and the seed it was reached from: search_index (255: none)
// and search_m, search_n, search_l. `critical_start` is the head of the automatic seeds' list,
// passed by value as the legacy pointer was: the nodes no longer unclassified are unlinked,
// except the head itself.
task genseg_context::SearchIndex(int critical_start) {
  int i, j, k;
  int x0, y0, z0;
  int x1, y1, z1;
  float t;
  int critical_tmp;
  int critical_prv;

  critical_tmp = critical_start;
  while (critical_tmp != -1) {
    i = critical[critical_tmp].x;
    j = critical[critical_tmp].y;
    k = critical[critical_tmp].z;
    // InitialTime(i, j, k): clear the previous search's times around the seed.
    {
      int xx = i, yy = j, zz = k;
      int ii, jj, kk;
      int x, y, z;

      T_tdata[IndexVect(xx, yy, zz)] = MAX_TIME;
      start_ptr = 0;
      end_ptr = 1;
      IndexArray[0].x = xx;
      IndexArray[0].y = yy;
      IndexArray[0].z = zz;

      while (start_ptr < end_ptr) {
        x = IndexArray[start_ptr].x;
        y = IndexArray[start_ptr].y;
        z = IndexArray[start_ptr].z;
        start_ptr++;

        for (kk = gs_max(z - 1, 0); kk <= gs_min(z + 1, ZDIM - 1); kk++)
          for (jj = gs_max(y - 1, 0); jj <= gs_min(y + 1, YDIM - 1); jj++)
            for (ii = gs_max(x - 1, 0); ii <= gs_min(x + 1, XDIM - 1); ii++) {
              if (T_tdata[IndexVect(ii, jj, kk)] != MAX_TIME) {
                T_tdata[IndexVect(ii, jj, kk)] = MAX_TIME;
                IndexArray[end_ptr].x = ii;
                IndexArray[end_ptr].y = jj;
                IndexArray[end_ptr].z = kk;
                end_ptr++;
              }
            }
        GS_UNIT();
      }
    }
    critical_tmp = critical[critical_tmp].next;
  }

  T_min_heap.size = 0;
  critical_tmp = critical_start;
  critical_prv = critical_start;
  while (critical_tmp != -1) {
    i = critical[critical_tmp].x;
    j = critical[critical_tmp].y;
    k = critical[critical_tmp].z;

    if (seed_index[IndexVect(i, j, k)] == 254) {
      T_min_heap.x[T_min_heap.size] = i;
      T_min_heap.y[T_min_heap.size] = j;
      T_min_heap.z[T_min_heap.size] = k;
      T_min_heap.t[T_min_heap.size] = 0.0;
      T_tdata[IndexVect(i, j, k)] = 0.0;
      T_min_heap.size++;

      if (critical_tmp != critical_start)
        critical_prv = critical[critical_prv].next;
    } else {
      if (critical_tmp == critical_start) {
        critical_start = critical[critical_start].next;
        critical_prv = critical_start;
      } else
        critical[critical_prv].next = critical[critical_tmp].next;
    }

    critical_tmp = critical[critical_tmp].next;
    GS_UNIT();
  }

  T_index = 255;
  while (1) {

    T_GetMinimum();

    if (min_t >= MAX_TIME) {
      search_index = 255;
      co_return;
    }

    T_GetTime();
    GS_UNIT();

    if (T_index < 254)
      break;
  }

  x0 = min_x;
  y0 = min_y;
  z0 = min_z;
  x1 = x0;
  y1 = y0;
  z1 = z0;
  t = T_tdata[IndexVect(x0, y0, z0)];

  while (t > 0) {

    for (k = gs_max(0, z0 - 1); k <= gs_min(ZDIM - 1, z0 + 1); k++)
      for (j = gs_max(0, y0 - 1); j <= gs_min(YDIM - 1, y0 + 1); j++)
        for (i = gs_max(0, x0 - 1); i <= gs_min(XDIM - 1, x0 + 1); i++) {
          if (T_tdata[IndexVect(i, j, k)] < t) {
            t = T_tdata[IndexVect(i, j, k)];
            x1 = i;
            y1 = j;
            z1 = k;
          }
        }
    if (x0 == x1 && y0 == y1 && z0 == z1) {
      search_index = 255;
      co_return;
    } else {
      x0 = x1;
      y0 = y1;
      z0 = z1;
      t = T_tdata[IndexVect(x0, y0, z0)];
    }
    GS_UNIT();
  }
  search_m = x0;
  search_n = y0;
  search_l = z0;
  search_index = T_index;
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

// GetMinimum() .. T_GetTime() follow as they were, but for the empty-heap guard in the two
// GetMinimum()s and the growth guard in the two InsertHeap()s.

void genseg_context::GetMinimum() {
  if (min_heap.size <= 0) { // the legacy code read before the arrays
    min_t = MAX_TIME;
    return;
  }
  int pointer, left, right;
  int x, y, z;
  float t;

  min_x = min_heap.x[0];
  min_y = min_heap.y[0];
  min_z = min_heap.z[0];
  min_t = min_heap.t[0];

  x = min_heap.x[min_heap.size - 1];
  y = min_heap.y[min_heap.size - 1];
  z = min_heap.z[min_heap.size - 1];
  t = min_heap.t[min_heap.size - 1];

  min_heap.size--;

  pointer = 1;
  while (pointer <= min_heap.size / 2) {
    left = 2 * pointer;
    right = 2 * pointer + 1;
    if ((min_heap.t[left - 1] <= min_heap.t[right - 1]) && (min_heap.t[left - 1] < t)) {
      min_heap.x[pointer - 1] = min_heap.x[left - 1];
      min_heap.y[pointer - 1] = min_heap.y[left - 1];
      min_heap.z[pointer - 1] = min_heap.z[left - 1];
      min_heap.t[pointer - 1] = min_heap.t[left - 1];
      pointer = left;
    } else if ((min_heap.t[left - 1] > min_heap.t[right - 1]) && (min_heap.t[right - 1] < t)) {
      min_heap.x[pointer - 1] = min_heap.x[right - 1];
      min_heap.y[pointer - 1] = min_heap.y[right - 1];
      min_heap.z[pointer - 1] = min_heap.z[right - 1];
      min_heap.t[pointer - 1] = min_heap.t[right - 1];
      pointer = right;
    } else
      break;
  }

  min_heap.x[pointer - 1] = x;
  min_heap.y[pointer - 1] = y;
  min_heap.z[pointer - 1] = z;
  min_heap.t[pointer - 1] = t;
}

void genseg_context::InsertHeap(int x, int y, int z) {
  int pointer, parent;
  float t;

  if (min_heap.size >= int(min_heap.t.size())) // the legacy code wrote past the end
    min_heap.grow();
  t = tdata[IndexVect(x, y, z)];
  min_heap.size++;
  pointer = min_heap.size;

  while (pointer > 1) {
    if (pointer % 2 == 0) {
      parent = pointer / 2;
      if (t < min_heap.t[parent - 1]) {
        min_heap.x[pointer - 1] = min_heap.x[parent - 1];
        min_heap.y[pointer - 1] = min_heap.y[parent - 1];
        min_heap.z[pointer - 1] = min_heap.z[parent - 1];
        min_heap.t[pointer - 1] = min_heap.t[parent - 1];
        pointer = parent;
      } else
        break;
    } else if (pointer % 2 == 1) {
      parent = (pointer - 1) / 2;
      if (t < min_heap.t[parent - 1]) {
        min_heap.x[pointer - 1] = min_heap.x[parent - 1];
        min_heap.y[pointer - 1] = min_heap.y[parent - 1];
        min_heap.z[pointer - 1] = min_heap.z[parent - 1];
        min_heap.t[pointer - 1] = min_heap.t[parent - 1];
        pointer = parent;
      } else
        break;
    }
  }

  min_heap.x[pointer - 1] = x;
  min_heap.y[pointer - 1] = y;
  min_heap.z[pointer - 1] = z;
  min_heap.t[pointer - 1] = t;
}

void genseg_context::GetTime() {
  int tempt_x, tempt_y, tempt_z;
  int tx, ty, tz;
  float intens, value;
  char boundary;
  unsigned char index;

  index = seed_index[IndexVect(min_x, min_y, min_z)];

  tempt_x = gs_max(min_x - 1, 0);
  tempt_y = min_y;
  tempt_z = min_z;

  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {
    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_x = gs_min(min_x + 1, XDIM - 1);
  tempt_y = min_y;
  tempt_z = min_z;
  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {

    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_y = gs_max(min_y - 1, 0);
  tempt_x = min_x;
  tempt_z = min_z;
  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {

    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_y = gs_min(min_y + 1, YDIM - 1);
  tempt_x = min_x;
  tempt_z = min_z;
  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {

    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_z = gs_max(min_z - 1, 0);
  tempt_x = min_x;
  tempt_y = min_y;
  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {

    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_z = gs_min(min_z + 1, ZDIM - 1);
  tempt_x = min_x;
  tempt_y = min_y;
  if (tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME &&
      seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] >= 254) {

    boundary = 0;
    value = MAX_TIME;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_high ||
        seed_index[IndexVect(tx, ty, tz)] < 254 && seed_index[IndexVect(tx, ty, tz)] != index)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + tdata[IndexVect(tx, ty, tz)])
        value = intens + tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] == 254)
      SeedNum--;
    seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] = index;
    InsertHeap(tempt_x, tempt_y, tempt_z);
  }
}

void genseg_context::T_GetMinimum() {
  if (T_min_heap.size <= 0) { // the legacy code read before the arrays
    min_t = MAX_TIME;
    return;
  }
  int pointer, left, right;
  int x, y, z;
  float t;

  min_x = T_min_heap.x[0];
  min_y = T_min_heap.y[0];
  min_z = T_min_heap.z[0];
  min_t = T_min_heap.t[0];

  x = T_min_heap.x[T_min_heap.size - 1];
  y = T_min_heap.y[T_min_heap.size - 1];
  z = T_min_heap.z[T_min_heap.size - 1];
  t = T_min_heap.t[T_min_heap.size - 1];

  T_min_heap.size--;

  pointer = 1;
  while (pointer <= T_min_heap.size / 2) {
    left = 2 * pointer;
    right = 2 * pointer + 1;
    if ((T_min_heap.t[left - 1] <= T_min_heap.t[right - 1]) && (T_min_heap.t[left - 1] < t)) {
      T_min_heap.x[pointer - 1] = T_min_heap.x[left - 1];
      T_min_heap.y[pointer - 1] = T_min_heap.y[left - 1];
      T_min_heap.z[pointer - 1] = T_min_heap.z[left - 1];
      T_min_heap.t[pointer - 1] = T_min_heap.t[left - 1];
      pointer = left;
    } else if ((T_min_heap.t[left - 1] > T_min_heap.t[right - 1]) &&
               (T_min_heap.t[right - 1] < t)) {
      T_min_heap.x[pointer - 1] = T_min_heap.x[right - 1];
      T_min_heap.y[pointer - 1] = T_min_heap.y[right - 1];
      T_min_heap.z[pointer - 1] = T_min_heap.z[right - 1];
      T_min_heap.t[pointer - 1] = T_min_heap.t[right - 1];
      pointer = right;
    } else
      break;
  }

  T_min_heap.x[pointer - 1] = x;
  T_min_heap.y[pointer - 1] = y;
  T_min_heap.z[pointer - 1] = z;
  T_min_heap.t[pointer - 1] = t;
}

void genseg_context::T_InsertHeap(int x, int y, int z) {
  int pointer, parent;
  float t;

  if (T_min_heap.size >= int(T_min_heap.t.size())) // the legacy code wrote past the end
    T_min_heap.grow();
  t = T_tdata[IndexVect(x, y, z)];
  T_min_heap.size++;
  pointer = T_min_heap.size;

  while (pointer > 1) {
    if (pointer % 2 == 0) {
      parent = pointer / 2;
      if (t < T_min_heap.t[parent - 1]) {
        T_min_heap.x[pointer - 1] = T_min_heap.x[parent - 1];
        T_min_heap.y[pointer - 1] = T_min_heap.y[parent - 1];
        T_min_heap.z[pointer - 1] = T_min_heap.z[parent - 1];
        T_min_heap.t[pointer - 1] = T_min_heap.t[parent - 1];
        pointer = parent;
      } else
        break;
    } else if (pointer % 2 == 1) {
      parent = (pointer - 1) / 2;
      if (t < T_min_heap.t[parent - 1]) {
        T_min_heap.x[pointer - 1] = T_min_heap.x[parent - 1];
        T_min_heap.y[pointer - 1] = T_min_heap.y[parent - 1];
        T_min_heap.z[pointer - 1] = T_min_heap.z[parent - 1];
        T_min_heap.t[pointer - 1] = T_min_heap.t[parent - 1];
        pointer = parent;
      } else
        break;
    }
  }

  T_min_heap.x[pointer - 1] = x;
  T_min_heap.y[pointer - 1] = y;
  T_min_heap.z[pointer - 1] = z;
  T_min_heap.t[pointer - 1] = t;
}

void genseg_context::T_GetTime() {
  int tempt_x, tempt_y, tempt_z;
  int tx, ty, tz;
  float intens, value;
  char boundary;

  tempt_x = gs_max(min_x - 1, 0);
  tempt_y = min_y;
  tempt_z = min_z;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_x = gs_min(min_x + 1, XDIM - 1);
  tempt_y = min_y;
  tempt_z = min_z;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_y = gs_max(min_y - 1, 0);
  tempt_x = min_x;
  tempt_z = min_z;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_y = gs_min(min_y + 1, YDIM - 1);
  tempt_x = min_x;
  tempt_z = min_z;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_z = gs_max(min_z - 1, 0);
  tempt_x = min_x;
  tempt_y = min_y;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }

  tempt_z = gs_min(min_z + 1, ZDIM - 1);
  tempt_x = min_x;
  tempt_y = min_y;
  if (seed_index[IndexVect(tempt_x, tempt_y, tempt_z)] < 254) {
    T_index = seed_index[IndexVect(tempt_x, tempt_y, tempt_z)];
    return;
  } else if (T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] == MAX_TIME) {

    value = MAX_TIME;
    boundary = 0;

    tx = gs_max(tempt_x - 1, 0);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tx = gs_min(tempt_x + 1, XDIM - 1);
    ty = tempt_y;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_max(tempt_y - 1, 0);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    ty = gs_min(tempt_y + 1, YDIM - 1);
    tx = tempt_x;
    tz = tempt_z;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_max(tempt_z - 1, 0);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    tz = gs_min(tempt_z + 1, ZDIM - 1);
    tx = tempt_x;
    ty = tempt_y;
    if (dataset[IndexVect(tx, ty, tz)] < t_low)
      boundary = 1;
    else {
      intens = 0.01f * (float)gs_exp(ALPHA * ImgGrad[IndexVect(tx, ty, tz)]);
      if (value > intens + T_tdata[IndexVect(tx, ty, tz)])
        value = intens + T_tdata[IndexVect(tx, ty, tz)];
    }

    if (boundary == 1)
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = MAX_TIME + 1;
    else
      T_tdata[IndexVect(tempt_x, tempt_y, tempt_z)] = value;

    T_InsertHeap(tempt_x, tempt_y, tempt_z);
  }
}

} // namespace cvc::genseg
