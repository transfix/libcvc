// cgal_ab_qhull.cpp -- cgal_ab_test's Qhull side (cgal_ab.h), alone in its translation unit so
// libqhull_r's macros never meet CGAL's headers.
//
// qhull_batch is libcvc's 3D Delaunay as the VolumeRover ports will run it: one reentrant
// qh_new_qhull call with `d Qbb Qt Qz`. qhull_incremental is the P1 spike (the ports plan, §2):
// the skeleton port wants to insert points one at a time, so a long triangulation can run as a
// resumable stepper. It seeds a hull with a few points and adds the rest with qh_setdelaunay +
// qh_findbestfacet (or qh_findbest from the newest facet) + qh_addpoint, then triangulates as
// Qt does. cgal_ab_permissive.cpp compares both with each other and with CGAL.

#include "cgal_ab.h"

#include <algorithm>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <numeric>
#include <vector>

extern "C" {
#include "libqhull_r/qhull_ra.h"
}

namespace cgal_ab {
namespace {

// One qhT per triangulation: every libqhull_r call keeps its state there.
struct qh_session {
  std::unique_ptr<qhT> holder{new qhT};
  qh_session() { qh_zero(holder.get(), stderr); }
  ~qh_session() {
    qh_freeqhull(holder.get(), !qh_ALL);
    int curlong = 0, totlong = 0;
    qh_memfreeshort(holder.get(), &curlong, &totlong);
  }
  qhT *get() { return holder.get(); }
};

// The lower (Delaunay) facets as tetrahedra of input indices. id_of maps a vertex's point to its
// input index, or -1 for Qz's point at infinity (whose facets are upper ones anyway).
template <typename IdOf> void collect(qhT *qh, IdOf id_of, qhull_tets &out) {
  facetT *facet;
  vertexT *vertex, **vertexp;
  FORALLfacets {
    if (facet->upperdelaunay)
      continue;
    std::array<int, 4> t{};
    int k = 0;
    bool at_infinity = false;
    FOREACHvertex_(facet->vertices) {
      const int id = id_of(vertex->point);
      at_infinity |= id < 0;
      if (k < 4)
        t[k] = id;
      ++k;
    }
    if (at_infinity)
      continue;
    if (k != 4) {
      out.exitcode = -2; // a non-simplicial lower facet: Qt / qh_triangulate should have split it
      continue;
    }
    std::sort(t.begin(), t.end());
    out.tets.push_back(t);
  }
  std::sort(out.tets.begin(), out.tets.end());
}

std::uint64_t spread_bits(std::uint64_t v) { // 21 bits -> every third bit of 63
  v &= 0x1fffff;
  v = (v | v << 32) & 0x1f00000000ffffULL;
  v = (v | v << 16) & 0x1f0000ff0000ffULL;
  v = (v | v << 8) & 0x100f00f00f00f00fULL;
  v = (v | v << 4) & 0x10c30c30c30c30c3ULL;
  v = (v | v << 2) & 0x1249249249249249ULL;
  return v;
}

// Insertion order: Morton (Z-curve) keys on a 2^21 grid over the bounding box.
std::vector<int> morton_order(const std::vector<double> &xyz) {
  const int n = int(xyz.size() / 3);
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  for (int i = 0; i < n; ++i)
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], xyz[3 * i + k]);
      hi[k] = std::max(hi[k], xyz[3 * i + k]);
    }
  std::vector<std::uint64_t> key(n);
  for (int i = 0; i < n; ++i) {
    std::uint64_t code = 0;
    for (int k = 0; k < 3; ++k) {
      const double span = hi[k] > lo[k] ? hi[k] - lo[k] : 1.0;
      const auto q = std::uint64_t((xyz[3 * i + k] - lo[k]) / span * 2097151.0);
      code |= spread_bits(q) << k;
    }
    key[i] = code;
  }
  std::vector<int> order(n);
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return key[a] < key[b]; });
  return order;
}

// qh_findbestfacet, but starting the directed search at `start` instead of qh.facet_list.
facetT *find_from(qhT *qh, pointT *point, facetT *start, realT *bestdist, boolT *isoutside) {
  int numpart = 0, totpart = 0;
  facetT *best =
      qh_findbest(qh, point, start, False, !qh_ISnewfacets, False, bestdist, isoutside, &totpart);
  if (*bestdist < -qh->DISTround) {
    best = qh_findfacet_all(qh, point, !qh_NOupper, bestdist, isoutside, &numpart);
    if (!*isoutside && best->upperdelaunay)
      best = qh_findbest(qh, point, best, False, False, False, bestdist, isoutside, &totpart);
  }
  return best;
}

} // namespace

qhull_tets qhull_batch(const std::vector<double> &xyz) {
  qhull_tets out;
  qh_session s;
  qhT *qh = s.get();
  std::vector<double> pts = xyz; // qh_new_qhull keeps pointers into its input
  const int n = int(pts.size() / 3);
  char cmd[] = "qhull d Qbb Qt Qz";
  out.exitcode = qh_new_qhull(qh, 3, n, pts.data(), False, cmd, nullptr, stderr);
  if (out.exitcode == 0)
    collect(
        qh,
        [&](pointT *p) {
          const int id = qh_pointid(qh, p);
          return id >= 0 && id < n ? id : -1;
        },
        out);
  return out;
}

qhull_tets qhull_incremental(const std::vector<double> &xyz, insert_order order, bool warm_start) {
  qhull_tets out;
  const int n = int(xyz.size() / 3);
  std::vector<int> perm(n);
  if (order == insert_order::morton) {
    perm = morton_order(xyz);
  } else {
    std::iota(perm.begin(), perm.end(), 0);
  }
  // The seed starts with the point farthest from the origin: Qz puts its point at infinity 1.1x
  // the seed's largest paraboloid value up, so no later point may lift above the seed's highest.
  // That point also sets Qhull's roundoff estimates for the whole cloud.
  int far = 0;
  double far_d = -1;
  for (int i = 0; i < n; ++i) {
    const double d =
        xyz[3 * i] * xyz[3 * i] + xyz[3 * i + 1] * xyz[3 * i + 1] + xyz[3 * i + 2] * xyz[3 * i + 2];
    if (d > far_d) {
      far_d = d;
      far = i;
    }
  }
  perm.erase(std::find(perm.begin(), perm.end(), far));
  perm.insert(perm.begin(), far);

  const int seed_n = std::min(n, 32);
  std::vector<double> seed(3 * seed_n);
  for (int i = 0; i < seed_n; ++i)
    for (int k = 0; k < 3; ++k)
      seed[3 * i + k] = xyz[3 * perm[i] + k];
  // The added points, lifted: qh_addpoint keeps pointers to them, so they never move.
  std::vector<double> lifted(4 * std::size_t(n - seed_n));

  qh_session s;
  qhT *qh = s.get();
  char cmd[] = "qhull d Qbb Qz"; // Qt is applied once, after the last point (qh_triangulate)
  out.exitcode = qh_new_qhull(qh, 3, seed_n, seed.data(), False, cmd, nullptr, stderr);
  if (out.exitcode != 0)
    return out;

  // qh_addpoint reports errors by longjmp to qh.errexit, as qh_new_qhull's own calls do.
  qh->NOerrexit = False;
  const int jumped = setjmp(qh->errexit);
  if (jumped) {
    qh->NOerrexit = True;
    out.exitcode = jumped;
    return out;
  }
  for (int k = seed_n; k < n; ++k) {
    double *p = &lifted[4 * std::size_t(k - seed_n)];
    p[0] = xyz[3 * perm[k]];
    p[1] = xyz[3 * perm[k] + 1];
    p[2] = xyz[3 * perm[k] + 2];
    qh_setdelaunay(qh, 4, 1, p); // the paraboloid coordinate, scaled as Qbb scaled the seed
    realT bestdist = 0;
    boolT isoutside = False;
    facetT *facet;
    if (warm_start && qh->facet_tail && qh->facet_tail->previous)
      facet = find_from(qh, p, qh->facet_tail->previous, &bestdist, &isoutside);
    else
      facet = qh_findbestfacet(qh, p, !qh_ALL, &bestdist, &isoutside);
    if (!isoutside) {
      ++out.not_added; // a duplicate, or a point Qhull's roundoff puts on the hull
      continue;
    }
    if (!qh_addpoint(qh, p, facet, False))
      break;
  }
  qh_triangulate(qh);
  qh->NOerrexit = True;

  const double *added_lo = lifted.data(), *added_hi = lifted.data() + lifted.size();
  collect(
      qh,
      [&](pointT *p) {
        if (p >= added_lo && p < added_hi)
          return perm[seed_n + int((p - added_lo) / 4)];
        const int id = qh_pointid(qh, p);
        return id >= 0 && id < seed_n ? perm[id] : -1;
      },
      out);
  return out;
}

} // namespace cgal_ab
