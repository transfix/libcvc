// cgal_ab.h -- what cgal_ab_test's translation units share (README.md).
//
// cgal_ab.cpp has libcvc's own replacements (nanoflann) and main(); cgal_ab_permissive.cpp the
// VolumeRover ports' permissive libraries (Qhull, Clipper2, CDT, Shewchuk's predicates) against
// CGAL; cgal_ab_qhull.cpp the Qhull side, alone, because libqhull_r's macros (True, False,
// FORALLfacets, ...) must not meet CGAL's headers.

#ifndef CVC_TOOLS_CGAL_AB_H
#define CVC_TOOLS_CGAL_AB_H

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace cgal_ab {

// One PASS/FAIL line; any FAIL makes the run exit non-zero.
void report(bool ok, const std::string &name, const std::string &detail = {});

// Wall time of f, in milliseconds.
double ms_of(const std::function<void()> &f);

// One TIME line: both sides' times and A's over B's.
void report_time(const std::string &name, const char *a, double ta, const char *b, double tb);

// Deterministic generator (xorshift32), so every run compares the same inputs.
struct rng {
  unsigned s;
  unsigned next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  double uniform(double lo, double hi) { return lo + (hi - lo) * (next() / 4294967296.0); }
};

// The VolumeRover ports' rows (cgal_ab_permissive.cpp).
void ab_permissive(rng &r);

// --- Qhull (cgal_ab_qhull.cpp) ------------------------------------------------------------

// A 3D Delaunay triangulation as input indices: each tetrahedron sorted, the list sorted.
struct qhull_tets {
  int exitcode = -1;
  std::vector<std::array<int, 4>> tets;
  int not_added = 0; // incremental only: points qh_findbestfacet found inside the hull
};

// One batch qh_new_qhull call, `d Qbb Qt Qz`, as libcvc runs it.
qhull_tets qhull_batch(const std::vector<double> &xyz);

// The P1 spike (VOLROVER-LIBRARY-PORTS-ROADMAP.md §2): the same triangulation built one point
// at a time with qh_setdelaunay + qh_findbestfacet + qh_addpoint, in Morton (Z-curve) order or
// in input order. warm_start begins each point location at the newest facet instead of the
// head of the facet list.
enum class insert_order { input, morton };
qhull_tets qhull_incremental(const std::vector<double> &xyz, insert_order order, bool warm_start);

} // namespace cgal_ab

#endif
