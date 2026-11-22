# volrover-goldens: the legacy libraries' outputs, for the ports' parity tests

The VolumeRover library ports (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) each start by
matching the legacy library's output on committed inputs: row T1 of §1.7. This directory makes
those outputs reproducibly:

1. `make_goldens.py` fetches transfix/volrover at the commit in `volrover.sha`. It is a shallow,
   sparse checkout of just the directories the libraries need.
2. It builds the legacy libraries with this directory's `CMakeLists.txt`. That build is never
   part of libcvc's.
3. It runs them on the committed inputs (`inputs/<library>/`).
4. It writes the goldens to `src/cvc/tests/data/volrover-goldens/<library>/`, where the ports'
   tests read them.

GPL code may be built here, and only here. The libraries that need CGAL (Skeletonization,
ContourTiler, PocketTunnel) build against a CGAL where CMake finds it (cvcpkg's, in CI). Nothing built here is linked into
libcvc, cvcGL or pycvc, and nothing here is installed or packaged.

```bash
python3 tools/volrover-goldens/make_goldens.py            # write the goldens
python3 tools/volrover-goldens/make_goldens.py --check    # make them again and compare
python3 tools/volrover-goldens/make_inputs.py             # rewrite the inputs (then commit them)
```

You need git, cmake, a C++ compiler and Python 3.8 or later. Ninja is used when it is on PATH.
`--work DIR` says where the checkout and the build go (default: the system temp directory).
`--volrover DIR` uses an existing checkout instead of fetching one.

CI's `volrover goldens` job runs `--check` on Linux whenever this directory or the goldens change.
When the check fails, it uploads the goldens it made.

## GenSeg

GenSeg is `src/Segmentation/GenSeg`, the seeded segmentation of §5. It is the first library here
because it has no CGAL.

- **What is built:** the legacy `segment.cpp` (the fast marching) and `inout.cpp` (its rawiv
  reader and writer), as they are at the pinned commit, with their warnings off.
  `genseg_driver.cpp` stands in for `general_segmentation.cpp`, the XML-RPC entry. It builds the
  seed lists the same way and calls `read_data` and `Segment` as that entry did.
- **What is shimmed:** `shim/` replaces three things the legacy files include:
  - VolMagick's `VolumeFileInfo` (the rawiv header, used only for the world coordinates in the
    border point cloud);
  - VolMagick's `big_endian()`;
  - the `XmlRpcValue` declaration in `genseg.h`.

  The real VolMagick needs volrover's whole core library to build, and GenSeg reads every voxel
  by hand anyway.
- **Inputs:** `inputs/genseg/<case>.raw` (float32, little-endian, x fastest) and `<case>.json`
  (dimensions, `tlow`, `thigh`, and seeds as `[class, x, y, z]`). `make_inputs.py` wrote them, and
  they are committed, so the goldens never depend on a machine's `exp()`. The script writes each
  input as the big-endian float rawiv file the legacy reader expects, on the unit grid. The legacy
  reader then normalizes the data to 0..255, so `tlow` and `thigh` are on that scale.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/genseg/`:
  - `<case>.labels.u8` holds the class of every voxel (uint8, x fastest; 255 means no class).
    It is read from the legacy `<prefix>_subunit_NN.rawiv` files: a voxel is in class NN where
    that file is nonzero. Every labelled voxel is at or above `tlow`, so it is above 0.
  - `<case>.indices.txt` is the legacy `<prefix>_indices.txt`: the voxels where two fronts met.
  - `<case>.json` records the case, the volrover commit, the platform it was made on, each
    class's voxel count, and the files' SHA-256.

| Case | Size | What it covers |
|---|---|---|
| `two_blobs` | 24³ | two touching blobs, one seed each |
| `three_classes_noisy` | 24³ | three classes on noisy data |
| `two_seeds_one_class` | 20³ | a class with two seeds |
| `auto_seeds_sweep` | 28³ | automatic seeds away from the user seeds: two chains of blobs seeded at one end, a blob between them, an isolated blob, a seed on the volume's face, a class with no seed |
| `blob_touching_shell` | 24³ | a blob touching a noisy shell, a seed in each |
| `noisy_many_seeds` | 24³ | heavy noise on a broad blob: 316 automatic seeds, 302 of them classified by the threshold sweep |

In the first three cases every automatic seed (GenSeg's local maxima) lies under a user seed, so
GenSeg's threshold sweep, which gives each other automatic seed a class, never runs. The last three
make it run, and `noisy_many_seeds` runs it hundreds of times. With noise 0.5 instead of 0.45 that
case makes the legacy executable crash: a seed the sweep classifies has nowhere to go, and the
legacy march then pops an empty heap, reading before its arrays (the port ends that march instead;
segmentation_test's `RunsWhereTheLegacyLibraryCrashed`).

**Platforms:** the goldens were made on Windows with MSVC 2022. CI's check on ubuntu-24.04 with
GCC then made the same bytes for the first three cases (2026-11-10, transfix/libcvc#1115).

The roadmap (§5) says the legacy GenSeg "doesn't work properly today". These goldens record what
it does, so the port can first match it voxel for voxel, before the port fixes it.

## SegMed

SegMed is `src/Segmentation/SegMed`, the paper's automatic seeding: the critical points of the
gradient field after anisotropic gradient vector diffusion (the roadmap's §5, P6).

- **What is built:** `GVF.cpp` (the diffusion and the critical points) and `inout.cpp` (the rawiv
  reader), from copies in the build tree with two patches, each of which must apply or the
  configure fails:
  - `GVF_Compute`'s `return;` right after `InitGVF` (`GVF.cpp:97`) goes. It skipped everything
    after the initial field, so at the pinned commit the diffusion, the critical points and their
    clustering never ran: the goldens are of that dead code, which is what the port ports;
  - `read_data` opens its file `"rb"`, not `"r"` (the same on POSIX; text mode on Windows mangles
    the floats).

  `segmed_driver.cpp` stands in for `medical_segmentation.cpp`, the XML-RPC entry. It includes the
  patched `GVF.cpp`, so it can also read the diffused field from the legacy globals.
  `segment.cpp` (a march from 13 voxels hard-coded for one heart dataset) and `diffuse.cpp`
  (uncalled) are left out.
- **Inputs:** `inputs/segmed/<case>.raw` (float32, little-endian, x fastest) and `<case>.json`
  (dimensions and `tlow`, on the 0..255 scale the legacy reader rescales to). `make_inputs.py
  --only segmed` wrote them.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/segmed/`:
  - `<case>.critical.txt`: `GVF_Compute`'s critical list, `x y z` a line, in its order (its
    field's minima above `tlow`, each 26-connected run thinned to one voxel in 21);
  - `<case>.field.f32`: the field `gvfflow()` leaves, x, y, z float32 per voxel. `--check`
    compares it to within 1e-6, not byte for byte: the legacy `exp()` is the platform's `expf`;
  - `<case>.json`: the case, the volrover commit, the platform, the count and the SHA-256.

| Case | Size | What it covers |
|---|---|---|
| `many_blobs` | 20³ | eight small blobs, every source (`tlow` -1): 22 points, 41 before the thinning |
| `plateau` | 20³ | a flat-topped plateau, every source: the background is one long run, thinned |
| `noisy_blob` | 18³ | a broad blob under heavy noise, `tlow` 20 |
| `noisy_slab` | 20³ | a noisy flat-topped slab, `tlow` 20 |

`critical_points_test` (`MatchSegMed`) checks `cvc::gradient_vector_diffusion` on SegMed's
initial field against the field (bit for bit on MSVC), and `cvc::critical_point_seeds` with
SegMed's parameters (`cp_field` gradient_magnitude, `cp_weights` initial, `cp_sample` 20,
`cp_background_threshold` the `tlow`) against the list, point for point.

**Platforms:** the SegMed goldens were made on Windows with MSVC 2022.

## Skeletonization

Skeletonization is `src/Skeletonization`, the skeleton of a point cloud (the roadmap's §2). It
needs CGAL, so this build needs CGAL, GMP, MPFR and Boost where CMake finds them: for example
cvcpkg's `cgal gmp mpfr boost` in a prefix on `CMAKE_PREFIX_PATH`, as CI's job installs them.
`--libs genseg` builds GenSeg alone.

- **What is built:** every legacy source but `PolyTess.cpp` (the ear-clipper nothing calls,
  whose notice forbids commercial use; `shim/Skeletonization/PolyTess.h` stands in for its header)
  and `main.cpp.unused`. They are compiled from copies in the build tree, patched at configure
  time. A patch that does not apply fails the configure. The patches:
  - **Five `CGAL_assertion()`s with side effects become plain calls:** `has_vertex()` in
    `medax.cpp` and `tcocone.cpp`, and `find_acceptor()` in `u1.cpp` and `u2.cpp`. These calls
    fill the indices the code then uses. A release build compiles the assertions out, which leaves
    those indices at -1 (or uninitialised), so the code reads and writes out of bounds. The port
    makes the same calls.
  - **`is_cospherical_pair()` returns false when one of the two cells is infinite,** as the port
    does. It used to test the infinite vertex's default-constructed (uninitialised) point.
  - **The helix strips `build_simple_skel()` returns get alpha 0.75 instead of 1,** so that
    `skeleton_driver.cpp` can tell them from the polyline segments, which share their colours.
- **What is shimmed:** `shim/cvcraw_geometry/Geometry.h`, the one piece of the legacy Geometry the
  library reads: its float vertex array.
- **Inputs:** `inputs/skeleton/<case>.pts` (a point a line, each coordinate the exact decimal of a
  float, so float and double parsers read the same value) and `<case>.json` (the parameters,
  legacy names, defaults when empty). `make_inputs.py --only skeleton` wrote them.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/skeleton/`:
  - `<case>.skel.txt`: what the legacy `skeletonize()` returned, a line per element: `L` polyline
    segments, `S` star segments, `H` helix strips and `P` sheet polygons
    (`skeleton_driver.cpp` gives the format);
  - `<case>.json`: the counts, the volrover commit, the platform and the SHA-256.

| Case | Points | What it covers |
|---|---|---|
| `torus` | 768 | a jittered torus: the legacy output is empty, because its core circle is a closed loop |
| `capsule` | 941 | a jittered capsule: one strand along the axis |
| `dumbbell` | 1,236 | two spheres joined by a rod: one strand through all three |
| `box_grid` | 154 | a box's surface on a lattice, exactly cospherical everywhere: strands, stars and two sheets |

`skeleton_test` (`MatchesTheLegacyLibrary`) compares the port with these goldens:
- **exactly (to 1e-9)** on the capsule, the torus and the box: every polyline and star segment,
  every sheet point and the sheets' area. The triangulation is CGAL's, cell for cell, even on the
  cospherical box. Closed loops are left out of the comparison, because the legacy code dropped
  them;
- **as a superset** on the dumbbell. Where U2's chains merge, which pieces are kept depends on the
  order the saddles are traced in. The legacy code used its cells' memory order (CGAL's allocation
  history); the port uses a geometric one. So the port has all 362 legacy segments, and 1 more.

**Platforms:** the skeleton goldens are the ones CI's check made on ubuntu-24.04 with GCC and
cvcpkg's CGAL (two runs made the same bytes), so `--check` passes there and only there. A Windows
build (MSVC 2022, CGAL 6.0.1) makes the same elements to 1e-9 on the capsule, the torus and the
box, but not the same bytes: the last digits differ, and so does the order of tied sheet polygons.
Its dumbbell differs more, because the cells' memory order is the allocator's: 361 segments, one
of them not in Linux's 362, and two of Linux's missing. The port contains both platforms'
dumbbell segments, and its test compares coordinates to 1e-9, so it passes against either set.

## PocketTunnel

PocketTunnel is `src/PocketTunnel`, the pockets, tunnels and voids of a molecular surface
(docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2, T5). TexMol carries a copy; the goldens are made
from VolumeRover's, which is the same at the pinned commit. It needs CGAL, as Skeletonization does
(`--libs pockettunnel` builds it alone).

- **What is built:** every legacy source but `main.cpp.unused`, two of them from patched copies in
  the build tree (a patch that does not apply fails the configure):
  - **`tcocone.cpp`** is Skeletonization's, with the same two `CGAL_assertion()`s whose
    `has_vertex()` calls fill the indices the code then uses: plain calls, as the port makes them;
  - **`pocket_tunnel.cpp`** calls the driver's `pocket_tunnel_driver_report()` after
    `detect_handle()`. `pocket_tunnel_fromsurf()` keeps its triangulation and clusters in locals;
    the hook reads them, and changes nothing.

  The legacy build's `-DHUGE=1000000` is left out: no source uses it, and it breaks MSVC's own
  `HUGE`.
- **What is shimmed:** `shim/cvcraw_geometry/cvcgeom.h`, the points, colours and triangles of the
  legacy `cvcgeom_t`; and `shim/PEDetection/Timer.h`, which `datastruct.h` includes and does not
  use.
- **Inputs:** `inputs/pockettunnel/<case>.pts` (float-exact, as the skeleton's; the legacy entry
  reads its points as floats) and `<case>.json` (`num_pockets` and `num_tunnels`, VolumeRover2's
  3 and 3). `make_inputs.py --only pockettunnel` wrote the four balls: each has chambers cut out
  of it, joined to the outside by narrower necks, sampled on the solid's boundary. `1grl.pts` is a
  protein's surface, which needs libcvc to make: PDB 1GRL's heavy atoms blurred by
  `cvc::mol::blur` (blobbiness -0.2, a 3 Å grid) and `cvc::iso` at 10, the points as floats.
  `pocket_tunnel_test`'s `OnAProtein` makes it again natively, checks the committed copy against
  it, and writes it with `CVC_POCKETTUNNEL_WRITE_1GRL=<path>`. x86 Linux and Windows make the same
  2,609 points. On aarch64 (macOS CI) the mesher's multiply-adds are fused and it makes 2,606, so
  there the test checks only the count, to 1%.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/pockettunnel/`:
  - `<case>.ptv.txt`: every cluster outside the surface (kind, mouths, volume, cells, maxima), the
    geometry `pocket_tunnel_fromsurf()` returned (its points with their colours, its triangles,
    each with the cluster it bounds and its class), and Tight Cocone's marking (the cells it marked
    outside, the facets it left on the surface); `pockettunnel_driver.cpp` gives the format;
  - `<case>.json`: the counts, the volrover commit, the platform and the SHA-256.

  `--check` compares a remade `.ptv.txt` by content: each point's colour by its coordinates, the
  triangles as a set, volumes to within 1e-9.

| Case | Points | What it covers |
|---|---|---|
| `pocket` | 1,923 | a chamber with one neck: one pocket |
| `tunnel` | 2,074 | a chamber with two necks: one tunnel (two mouths) |
| `three_chambers` | 3,487 | a tunnel and two pockets of different sizes in one ball |
| `cavity` | 1,873 | a chamber with no neck: Tight Cocone marks it inside, so there is no cluster |
| `1grl` | 2,609 | PDB 1GRL (a GroEL subunit) at a 3 Å map's resolution: a tunnel (two mouths) and a pocket of over a thousand Å³ each, and three pockets of one cell |

**The legacy result depends on the cells' addresses.** CGAL's facet iterator reports a facet from
the cell at the lower address, and Tight Cocone's peeling (`create_surface`, `grow_boundary`) is
greedy: it starts from the first infinite cell it meets and walks in that order. Where its choices
are close, the cells it marks outside differ with the order. Within one process a second
triangulation of the same points, placed elsewhere in the heap, can be marked differently: on
`tunnel`, one Windows run in five marked 33 more cells outside, which made five pockets of one or
two cells. So the driver runs the legacy entry once a process, and records the marking with the
result.

`pocket_tunnel_test` compares the port with these goldens at two levels:
- **exactly after Tight Cocone** (`MatchesTheLegacyLibraryAfterTightCocone`): given the legacy
  marking (`pocket_tunnel_marked.h`), the port's own stages (S_max, the mouths, the output) make
  every cluster with the same kind, mouths, cells and maxima, volumes to 1e-12, every shown
  triangle with its cluster and class, and every point's colour, on all five cases;
- **end to end** (`MatchesTheLegacyLibrary`): the port's Tight Cocone walks in tet_complex's
  spatial order. On `pocket`, `tunnel` and `cavity` it marks the same cells and the whole result
  is the legacy's: on `tunnel`, a tunnel of 3,085 cells and five pockets of one or two. On
  `three_chambers` each cluster has 3 to 6 more cells than the legacy's (the tunnel 3,105 to
  3,099, volume 10.619 to 10.614). On `1grl` the large pocket is the legacy's, cell for cell, and
  the tunnel has 317 cells to the legacy's 307 (1,195 Å³ to 1,180); both find three one-cell
  pockets. Every cluster of more than a hundred cells keeps its kind and mouths, and its cells
  and volume to 5%.

**Platforms:** as with Skeletonization, the goldens are the ones CI's check made on ubuntu-24.04
with GCC and cvcpkg's CGAL, so `--check` passes there and only there. A Windows build (MSVC 2022,
CGAL 6.0.1) makes the same files in a new process each time, but its heap orders the cells
differently, and on three cases it marks other cells outside:
- `tunnel`: 33 fewer, and a single tunnel of 3,083 cells;
- `three_chambers`: 41 more, the clusters the port's to the cell;
- `1grl`: 67 more, a tunnel of 309 cells and 17 one-cell pockets.
Given either platform's marking, the port's stages make that platform's result exactly.

## ContourTiler

ContourTiler is `src/ContourTiler`, the contour tiler of the roadmap's §3. It needs CGAL, as
Skeletonization does (`--libs tiler` builds it alone).

- **What is built:** the library's sources, but for its tools (ecs, smfconvert, its command line),
  `contour_graph.cpp` (unused; its reader needs ContourFilter) and `minkowski.cpp` (unused).
  `tiler_driver.cpp` stands in for `main.cpp`: it reads a Reconstruct series with the legacy reader
  (`xml/Ser_reader`, as the command line's `.ser` path did) and calls `tile()`.
- **What is shimmed:** `shim/log4cplus`, a log4cplus that logs nothing, so the build needs no
  log4cplus.
- **What is patched** (from copies in the build tree; a patch that does not apply fails the
  configure):
  - `print_utils.cpp`: `raw_print_tiles_impl` writes coordinates with 17 digits (its commented-out
    `setprecision`, back in), not the stream's default 6, and each vertex's z as the tiler's own (a
    contour vertex's is its section index), not `(z + 0.5) * z_scale`;
  - `offset_polygon.cpp` and `skeleton.cpp`: CGAL 6's straight-skeleton functions return
    `std::shared_ptr` where CGAL 5's returned `boost::shared_ptr`;
  - `tiler_output.cpp`: `Callback::untiled_region`, the debug hook called on every untiled region,
    falls off its end without returning its `bool`. That is undefined behaviour, and clang's code
    aborts there, on every series with an untiled region (a cap or a branch). It returns true, as
    `Callback::tile_added` does.
- **Compilers:** MSVC cannot compile the legacy kernel (`Cartesian_25`'s conversions to CGAL's
  points); clang-cl and GCC can. On Windows, configure with `CXX=clang-cl` (the MSVC-built GMP and
  MPFR link). MSVC's standard library leaves out `std::ptr_fun` and `std::unary_function` under
  C++17; `_HAS_AUTO_PTR_ETC` and a forced `<functional>` bring them back.
- **Inputs:** `inputs/tiler/<case>/<case>.ser` and its numbered sections, each trace closed,
  counterclockwise (the legacy curation requires it) and in an identity transform, with
  coordinates multiples of 2^-8, so the port's snapping grid holds them unchanged; and
  `inputs/tiler/<case>.json`, the section range and the legacy parameters (`curation_delta`,
  `collinear_epsilon`, `remove_intersections`, `z_scale`). `make_inputs.py --only tiler` wrote
  them.
- **Goldens:** for each case and component, in `src/cvc/tests/data/volrover-goldens/tiler/`:
  - `<case>.<component>.tiles.txt`: the legacy `.rawc` without its colours: a line with the vertex
    and triangle counts, the vertices (x y z), then the triangles (vertex indices), in the legacy
    order;
  - `<case>.json`: the counts, the volrover commit, the platform and the SHA-256s.

| Case | Sections | What it covers |
|---|---|---|
| `tube` | 1-6 | one component tiled between contour vertices only: no caps, no untiled regions |
| `branch` | 0-7 | a component that branches (one contour, then two) and an annulus (a contour and its hole), with caps where the range passes their ends: correspondence, branching, tiling regions and untiled regions; the legacy fails to trace one of the annulus's untiled regions ("No chord was legal") |
| `touching` | 1-5 | two components whose traces overlap: curation (dilate, subtract, erode) and the removal of intersections between components' tiles |

`tiler_test` (`TilerLegacy.*`) compares the port with these goldens, as the parity §9 item 2
settled (2026-11-17):
- **exactly** on `tube`: the same 330 triangles over the same 196 vertices;
- **exactly away from untiled regions** on `branch`: every triangle whose corners are all contour
  vertices is the legacy's (494 of 494 and 268 of 268), and the topology is the same; the untiled
  regions' fill (from cvc::planar's convex partition, not CGAL's) is within a quarter of the slice
  spacing (0.13 measured);
- **by topology and distance** on `touching`: Clipper2's Booleans rebuild the overlapping contours
  where the legacy used CGAL's exact ones, so the surfaces are within twice the curation distance
  (1.03 times it measured), with the same topology.

**Platforms:** the tiler goldens were made on Windows with clang-cl 19 and CGAL 6.0.1, and
`--check` there makes the same files. GCC 16 compiles the same sources.
