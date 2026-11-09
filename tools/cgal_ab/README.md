# cgal_ab: A/B against CGAL

libcvc does not use CGAL. CGAL's algorithm packages are GPL-3+, and libcvc is LGPL-2.1, so each
CGAL use is replaced by a permissive library: transfix/libcvc#1080 for libcvc's own two, and the
VolumeRover ports plan (`docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md`) for the ports. This
program keeps the CGAL side alive only to compare against. It runs each replacement beside the
CGAL code it replaced, on the same deterministic inputs, and checks results and times.

| Check | A: CGAL | B: libcvc | Passes when |
|---|---|---|---|
| `neighbors ...` | `Kd_tree` + `Fuzzy_sphere` (epsilon 0), as `nav/spatial.cpp` had it | `nav::neighbors_within_radius` (nanoflann) | the neighbour lists are identical, on uniform, lattice (exact boundary distances) and clustered points with duplicates |
| `projection ...` | `AABB_tree::closest_point` (exact) | `project_points_to_triangles` (nanoflann + exact closest point) | no point lands closer than exact, and distances are identical when every triangle is a candidate. With k = 10 it reports how often the nearest-vertex heuristic is exact, and by how much it misses |

`TIME` lines give both sides' wall time. Nothing asserts on them.

**Licence.** This executable links CGAL, so as built it is a GPL-3+ combined work. It is never
installed, packaged or shipped, and it builds only on request:

```bash
cmake -S . -B build -DCVC_BUILD_CGAL_AB=ON -DCVC_BUILD_TESTS=OFF   # needs CGAL, GMP and MPFR
cmake --build build --target cgal_ab_test
build/bin/cgal_ab_test
```

CI runs it in `ci.yml`'s `cgal-ab` job when the code it compares changes. Each VolumeRover port
adds its rows here as it lands (row T16 of the ports plan's test matrix):
- Qhull's Delaunay against `Delaunay_triangulation_3`;
- the planar module against CGAL's Booleans, offsets, constrained Delaunay and straight skeleton.
