# cgal_ab: A/B against CGAL

libcvc does not use CGAL. CGAL's algorithm packages are GPL-3+, and libcvc is LGPL-2.1, so each
CGAL use is replaced by a permissive library: transfix/libcvc#1080 for libcvc's own two, and the
VolumeRover ports plan (`docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md`) for the ports. This
program keeps the CGAL side alive only to compare against. It runs each replacement beside the
CGAL code it replaced, on the same deterministic inputs, and checks results and times.

libcvc's own replacements (`cgal_ab.cpp`):

| Check | A: CGAL | B: libcvc | Passes when |
|---|---|---|---|
| `neighbors ...` | `Kd_tree` + `Fuzzy_sphere` (epsilon 0), as `nav/spatial.cpp` had it | `nav::neighbors_within_radius` (nanoflann) | the neighbour lists are identical, on uniform, lattice (exact boundary distances) and clustered points with duplicates |
| `projection ...` | `AABB_tree::closest_point` (exact) | `project_points_to_triangles` (nanoflann + exact closest point) | no point lands closer than exact, and distances are identical when every triangle is a candidate. With k = 10 it reports how often the nearest-vertex heuristic is exact, and by how much it misses |

The VolumeRover ports' permissive libraries (`cgal_ab_permissive.cpp`, row T16 of the ports
plan's test matrix; Qhull alone in `cgal_ab_qhull.cpp`, so its macros never meet CGAL's
headers):

| Check | A: CGAL | B: permissive | Passes when |
|---|---|---|---|
| `predicates ...` | `orientation`, `side_of_oriented_circle`, `side_of_oriented_sphere` (Epick, exact) | Shewchuk's `orient2d`, `incircle`, `orient3d`, `insphere` (cvcpkg `shewchuk-predicates`) | every sign agrees, on random, nearly degenerate (points rounded onto a line, circle, plane or sphere) and exactly degenerate (small integer) inputs. `orient3d` and `insphere` have the opposite sign convention to CGAL's |
| `delaunay3 ...` | `Delaunay_triangulation_3` | reentrant Qhull, `d Qbb Qt Qz` | points in general position (uniform clouds to 100k points, a Gaussian clump): identical tetrahedra. Cospherical subsets (a cube and its centre, a 10^3 lattice, points rounded onto a sphere), where many triangulations are Delaunay: Qhull's tiles the hull (no face in three tetrahedra, CGAL's volume) with circumspheres empty to a relative 1e-9 |
| `cdt ...` | `Constrained_Delaunay_triangulation_2` | CDT (cvcpkg `cdt`) | identical triangles, with and without constraints (a 300-edge star polygon among 5,000 points); on a cocircular grid, the same triangle count and empty circumcircles |
| `clipper2 <op> ...` | `Polygon_set_2` (Epeck) | Clipper2 `Union`, `Intersect`, `Difference`, `Xor` | orthogonal polygons, whose new vertices are all grid points: the same area exactly. Star polygons: the area within one unit per unit of boundary (Clipper2 rounds new vertices to its grid). The same outer and hole counts, except for symmetric differences, whose pieces touch at points: CGAL keeps those as one polygon with holes |
| `clipper2 offset ...` | straight-skeleton offsets (`create_*_skeleton_and_offset_polygons_with_holes_2`) | `InflatePaths`, mitred (limit 10^6) | the area within one unit per unit of boundary, and the same counts, for convex polygons at any depth and star polygons before their first edge event. Past an edge event the straight skeleton (a wavefront) and a mitred offset are different polygons by design: those are `INFO` lines |
| `clipper2 minkowski ...` | `minkowski_sum_2` | `MinkowskiSum` and a union | convex + convex: the same area exactly; convex + star: within one unit per unit of boundary |

cvc::planar (`cgal_ab_planar.cpp`, `inc/cvc/geometry/planar.h`), the ContourTiler port's planar
geometry (the ports plan's P3a), against the CGAL calls ContourTiler makes today. These rows run
cvc::planar's own functions, which is what the tiler will call, where the rows above run the
libraries directly. Each compares the areas, the counts of outer rings and holes, and the area of
the symmetric difference between the two results (CGAL's exact vertices rounded to the grid),
within one grid unit per unit of boundary:

| Check | A: CGAL | B: cvc::planar | Passes when |
|---|---|---|---|
| `planar <op> ...` | `Polygon_set_2` (Epeck) | `unite`, `intersect`, `subtract`, `exclusive_or` | skylines: equal areas exactly; stars, polygons with holes and dense contour pairs: within the tolerance; the same counts except for symmetric differences (as above) |
| `planar wavefront_offset ...` | `create_{interior,exterior}_skeleton_and_offset_polygons_with_holes_2` | `wavefront_offset` | within the tolerance at every depth: convex and star polygons before and past their edge events, and dense contours (ContourTiler's input, whose short edges collapse at once) |
| `planar offset (one pass) ...` | the same | `offset`, mitred, limit 10^6 | asserted only where a single pass must agree (convex polygons, and offsets before the first edge event); `INFO` elsewhere |
| `planar minkowski_sum ...` | `minkowski_sum_2` | `minkowski_sum` | convex + convex: equal areas exactly; convex + star: within the tolerance |
| `planar delaunay ...` | `Delaunay_triangulation_2` (Epick) | `delaunay` | identical triangles, uniform points to 32k |
| `planar triangulate ...` | `Constrained_Delaunay_triangulation_2` of the rings, its faces inside the polygon (as ContourTiler keeps them) | `triangulate` | identical triangles: stars, dense contours, a contour with two holes |
| `planar convex_partition ...` | `approx_convex_partition_2` (Hertel-Mehlhorn), and `greene_approx_convex_partition_2`, ContourTiler's fallback | `convex_partition` | cvc::planar's pieces pass CGAL's own `convex_partition_is_valid_2` and Hertel-Mehlhorn's bound (2r + 1 pieces for r reflex vertices). The piece counts, and whether the pieces equal CGAL's, are reported |
| `planar sibson_coordinates ...` | `natural_neighbor_coordinates_2` and `linear_interpolation` on `Delaunay_triangulation_2` | `sibson_coordinates`, `sibson_interpolate` | every weight and interpolated value within 1e-9, over 500 queries; the same queries inside and outside the hull |

`TIME` lines give both sides' wall time. Nothing asserts on them, nor on the `INFO` lines, nor on
the `SPIKE` lines below.

**What the rows found (2026-11-09, Windows, MSVC 19.44, CGAL 6.0.1)**, for P1 and P3a:
- Shewchuk's predicates agree with CGAL's exact signs in all 1.2 million cases, at about the same
  speed (0.7-2.9x).
- Qhull's 3D Delaunay is identical to CGAL's in general position, but about 3-4x slower (100k
  points: 0.76 s for CGAL, 2.8 s for Qhull).
- On degenerate input Qhull is not exact: `Qt` leaves flat tetrahedra where exactly cospherical
  points merged (1,049 of 5,423 on the 10^3 lattice), and on points rounded onto a sphere its
  precision model merges everything into one facet and fans it, a valid tiling but not the exact
  Delaunay triangulation (no tetrahedron's circumsphere is empty by exact predicates). Of the
  options tried, `Q0 Qt Qz` (no merging) gives the lattice cleanly but fails on the sphere, and
  `QJ Qz` gives the sphere exactly but leaves flat tetrahedra on the lattice.
- CDT is identical to CGAL's constrained Delaunay, 0.7-0.9x its speed.
- Clipper2 agrees with CGAL's Booleans, Minkowski sums and early offsets to within its rounding,
  at 1.7-90x CGAL's speed (slower only on the Minkowski sums, where it unions all the swept
  quadrilaterals).

**What the cvc::planar rows found (2026-11-11, Windows, MSVC 19.44, CGAL 6.0.1)**, for P3b:
- The Booleans agree with `Polygon_set_2` in all 32 cases: exactly on the skylines, and within a
  tenth of the tolerance elsewhere, polygons with holes and dense contour pairs included, at about
  20-60x CGAL's speed.
- A single mitred pass (`offset`) differs from the straight skeleton's offset in 2 of the 50 offset
  cases, both deep inward offsets of stars, by 0.4% and 7.4% of the area. A single pass agrees
  with the skeleton past an edge event whose collapse leaves a convex join (it resolves the loop
  the vanished edge leaves exactly as the skeleton does), but not past one that leaves a reflex
  join, where the skeleton mitres the neighbours and a single pass bounds the region by the
  vanished edge's line instead.
- `wavefront_offset` agrees with the skeleton in all 50 cases, those two included (symmetric
  difference 0 after rounding), at about 27x CGAL's speed. It steps from event to event on a grid
  2^20 times finer than the caller's and rounds once at the end; with every step rounded to the
  caller's grid instead, the error grew to about two grid units per unit of boundary on the dense
  contours, past the tolerance.
- The Minkowski sums agree, exactly for convex pairs.
- The triangulations are CGAL's exactly: Delaunay (at about a third of CGAL's speed) and the
  constrained triangulation of polygons, holes included (1.5x CGAL's speed).
- Sibson coordinates equal CGAL's natural-neighbour coordinates to 1e-14, at about a third of its
  speed.
- The convex partitions are valid but never CGAL's: Hertel-Mehlhorn on CDT's (Delaunay)
  triangulation gives 15-45% fewer pieces than CGAL's approx_convex_partition_2, which runs it on a
  constrained triangulation that is not Delaunay (on the dense contours, 50-206 pieces against
  CGAL's 82-300; Greene's are as many as CGAL's). ContourTiler fills untiled regions from these
  pieces, not from the straight skeleton (the ports plan, §3), so the port's fills will differ
  there.

**The incremental Qhull spike (`SPIKE` lines).** The skeleton port plans to insert points one at a
time, so a long triangulation can run as a resumable stepper (the ports plan, §2). The `delaunay3`
rows also build each triangulation with `qhull_incremental`: a 32-point seed (`d Qbb Qz`, led by
the point farthest from the origin, because Qz's point at infinity sits 1.1x above the seed's
highest lifted point), then `qh_setdelaunay` + `qh_findbestfacet` + `qh_addpoint` for each other
point in Morton order, then `qh_triangulate` as `Qt` does. "Warm start" locates each point from
the newest facet (`qh_findbest`) instead of the head of the facet list. The cold variants are
skipped above 50k points. Locally:
- In general position the incremental triangulation is identical to the batch one, and to CGAL's,
  on every cloud.
- Warm start costs 1.1x the batch time at 2k-20k points and 2.5-3x at 100k. Locating from the head
  of the facet list costs 2.4x at 20k and 32x at 100k, so P1 needs the warm start.
- On the degenerate inputs the incremental and batch results differ, as two valid triangulations
  of cospherical points may.

**Licence.** This executable links CGAL, so as built it is a GPL-3+ combined work. It is never
installed, packaged or shipped, and it builds only on request:

```bash
# needs CGAL, GMP and MPFR, and the qhull, clipper2, cdt and shewchuk-predicates packages
cmake -S . -B build -DCVC_BUILD_CGAL_AB=ON -DCVC_BUILD_TESTS=OFF
cmake --build build --target cgal_ab_test
build/bin/cgal_ab_test
```

CI runs it in `ci.yml`'s `cgal-ab` job when the code it compares changes. Each VolumeRover port
adds its rows here as it lands: the skeletons built on Qhull's and CGAL's triangulations (P1), and
the planar module and the tiled surfaces against CGAL's (P3a, P3b).
