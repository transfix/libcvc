# VolumeRover Library Ports — Roadmap (sketch)

**Status:** sketch, 2026-11-07, first merged as transfix/libcvc#1057. Revised three times on
2026-11-08:
1. GenSeg added, and ContourTiler recorded as LGPL.
2. Joe's rules for every port added (§1.2–§1.7): no singletons, an API pycvc wraps, far more
   unit tests, every parameter and piece of data on the state tree, DSL intrinsics, and
   (then) a separate library for GPL code.
3. Joe's answers:
   - **no GPL code at all:** permissive libraries replace every GPL CGAL package (§1.2), so there
     is no GPL library and no `-gpl` package, and libcvc's own GPL CGAL use is already gone
     (transfix/libcvc#1080);
   - **wasm from the first PR** of every phase (§1.8);
   - **everything under an app context, no singletons anywhere in libcvc** (§1.3), done before
     the ports;
   - VolumeGridRover keeps the legacy view switch (§4);
   - GenSeg's default seeding as recommended (§5).

Revised on 2026-11-09 for Joe's "make them resumable like we have for the FEM routines": every
long routine is a resumable stepper. It runs on the Ariadne scheduler in slices, publishes its
progress for a progress bar, and stays interactive on one thread (§1.9). Also on 2026-11-09, for
Joe's "remove the legacy xmlrpc ... since we are switching to arisrv": libcvc's XML-RPC goes,
arisrv replaces it, and so do the legacy remote modes and F2Dock's XML-RPC server (§1.10).

Nothing of the ports has landed. §9 lists the decisions still open.
**Based on:** libcvc `origin/master @ 921c4254` (§1's facts) and `@ 7a09366f` (the rest),
transfix/libcvc#1080 (libcvc without CGAL), and volrover `master @ b4a172c` (the legacy
libraries live under `src/<Lib>` and `inc/<Lib>` there).
**Scope:** four legacy VolumeRover libraries, ported into libcvc the way `mesh_lab` / `fe_lab`
were: a headless `cvc::` API, a `cvc` CLI command, pycvc bindings, DSL intrinsics and an Ariadne
node type with a component, native and wasm.
- Skeletonization.
- Tiling, with its successor ContourTiler.
- GenSeg, the seeded segmentation, with the automatic seeding from its sibling SegMed.
- VolumeGridRover, which becomes an Ariadne component with embedded GL views and drives the
  other three.
**Companions:**
- [ARIADNE.md](../ARIADNE.md): §5.4 custom node types, §5.5 request keys, §5.18 the `viewport`
  widget, `scenes:` and named cameras, §6.5 the component library.
- [STATE_API.md](../STATE_API.md): per-app roots, the data channel, properties.
- [STATE_EXEC_DEVELOPER_GUIDE.md](../STATE_EXEC_DEVELOPER_GUIDE.md): §6 "Async Functions", §11
  the standard library.
- [PYCVC_STATE_EXEC_PARITY.md](../PYCVC_STATE_EXEC_PARITY.md): pycvc wraps the real types.
- [LIBIGL-GEOMETRY-FE-ROADMAP.md](LIBIGL-GEOMETRY-FE-ROADMAP.md): the `mesh_lab` / `fe_lab`
  precedent.
- [TRANSFER_FUNCTIONS.md](../TRANSFER_FUNCTIONS.md): the `tf_editor`.
- `inc/cvc/core/sliced_task.h`, `fem::stepped_solve` (`inc/cvc/geometry/fem.h`) and
  `contour_tree_builder` (`inc/cvc/volume/contour_tree.h`): the resumable pattern every long
  routine follows (§1.9).
- transfix/libcvc#123: the other VolUtils leftovers.

---

## 0. Summary

- **Rules every port follows** (§1):
  - **No GPL code** (settled, §1.2). CGAL's kernel, 2D polygons and number types are LGPL, but
    every CGAL algorithm package these ports use is GPL. Permissive libraries replace all of
    them:
    - reentrant Qhull for 3D Delaunay;
    - Clipper2 for polygon Booleans, offsets and Minkowski sums;
    - CDT for 2D constrained Delaunay;
    - Boost.Polygon's segment Voronoi diagram for the medial axis;
    - Shewchuk's robust predicates;
    - Eigen for least squares.

    So no port uses CGAL at all, there is no GPL library, and no `-gpl` package is needed. libcvc
    itself already dropped CGAL: transfix/libcvc#1080 replaced its two GPL uses (Spatial
    Searching) with nanoflann.
  - **No singletons, anywhere in libcvc** (§1.3). Everything lives under a `cvc::app` or in
    objects the caller owns.
    - In the ports: Skeletonization's global bounding box, ContourTiler's log4cplus loggers and
      `Controls::instance()`, and GenSeg's 25 globals all go.
    - In libcvc itself: a full audit lists its process-wide state, including Ariadne's node-type
      and intrinsic registries and the file I/O handler maps. P0a moves all of it under the app,
      before any port lands.
    - A CI scan of the object files and two-at-once tests keep it that way.
  - **One API shape** (§1.4): free functions on value types, a params struct and a result
    struct, a resumable stepper with the job around it, exceptions. SWIG wraps the header itself,
    so pycvc has the same names.
  - **The state tree holds everything** (§1.5). Every parameter is a key, declared once in a
    parameter table that also gives the defaults, units and ranges, the CLI flags, the Python
    keywords and the DSL dict keys. Inputs and results sit on the data channel at documented
    keys.
  - **DSL intrinsics** (§1.6): each port's operations, such as `skeleton.compute`, `tiler.tile`
    and `segment.seeded`. Each runs as a sliced task, so an Ariadne action can start one under
    its default limits.
  - **Tests** (§1.7): a fixed matrix of 16 kinds of test per port, from legacy parity and exact
    topology through concurrency, jobs, state round trips and I/O corpora to the DSL, pycvc, the
    CLI, wasm, the licence check and an A/B of every replacement against the CGAL code it
    replaces (`tools/cgal_ab`, started in transfix/libcvc#1080).
  - **Wasm from the start** (§1.8): every library a port uses builds for wasm-mt, and every
    phase's tests run under node in CI from its first PR. The browser demo comes with the node
    type, not in a later web phase.
  - **Resumable by construction** (§1.9). Every long routine is a stepper, as
    `fem::stepped_solve` and `contour_tree_builder` are: `step(budget)` in small units, with
    every phase sliced. On single-threaded wasm it runs about 4 ms a frame on the scheduler
    thread, and in pool chunks where there are threads. It publishes `progress` and `phase` for a
    progress bar, and stops at its next step. The blocking function runs the stepper to the end,
    so both give the same result.
  - **No XML-RPC** (§1.10). libcvc's `cvc::xmlrpc`, the vendored XmlRpc++, `cvc server` /
    `cvc client` and every legacy remote mode go. Remote state and remote compute are arisrv's:
    its state transports, `room-call` and the job service.
- **Skeletonization** (§2) is a clean port of about 8k lines of LGPL-2.1 code.
  - Its only heavy dependency was CGAL's 3D Delaunay (GPL). Reentrant Qhull replaces it, behind
    a small tetrahedral complex that presents the calls the cocone code makes.
  - It needs its global state removed, a few bug fixes, and its sheets kept rather than dropped.
  - It lands as `cvc::skeletonize`, `cvc skeleton`, `skeleton.compute`, and a `skeleton`
    request on `mesh_lab`.
  - Its cocone code is copied almost verbatim in TightCocone, PocketTunnel and Curation, so it
    becomes one shared `cvc::cocone` core.
- **Tiling** (§3) means ContourTiler, not the old `src/Tiling`.
  - The old tiler has not been built since 2011, and it carries a Purdue SHASTRA notice that
    forbids commercial use.
  - ContourTiler (LGPL) reimplements the same 1996 algorithm for many components. It is the
    biggest job here: it uses about 60 CGAL headers, most of them GPL packages and exact GMP
    number types.
  - The port swaps that layer for a small planar-geometry module of libcvc's own: integer
    coordinates, exact predicates, Clipper2, CDT and a Boost.Polygon medial axis. The tiler logic
    stays as it was.
  - It lands as `cvc::tile_contours`, `cvc tile`, `tiler.tile`, and a `contour_lab` node, on a
    shared `cvc::contour_stack` type.
- **VolumeGridRover** (§4) is a rebuild, not a translation.
  - Its Qt6 code is mostly stubs: contours cannot be created, and tiling, SDF and the medial
    axis are commented out. Its GL is 2005-era.
  - What carries over is the design: XY, XZ and ZY slice views with the legacy switch between
    them, a voxel probe, point classes, contours drawn per object, and tiling them into surfaces.
  - It becomes a `volume_grid` node plus a `volume_grid.ari` component of embedded `viewport`s:
    one slice view with the XY / XZ / ZY switch beside a 3D view by default, or all four at once.
- **GenSeg** (§5) is a small port, about 2.8k lines of LGPL code. It implements §2.2 of Bajaj,
  Yu and Auer, *J. Struct. Biol.* 144 (2003): multi-seed fast marching, where fronts slow at
  edges and fronts of different classes stop where they meet. VolumeGridRover's point classes
  are its seeds.
  - It doesn't work properly today:
    - it reads its input and names its outputs through dangling `c_str()` pointers;
    - it runs on about 25 globals;
    - it passes results through files in the current directory, and nothing loads them back.
  - The port is first made to match the legacy output voxel for voxel. Then it gains the paper's
    fully automatic seeding: SegMed's critical points of the diffused gradient field.
  - `volume_grid` keeps GenSeg's local maxima as the default seeding until the paper mode
    matches the legacy results on real maps.
  - It lands as `cvc::segment_seeded`, `cvc segment` and `segment.seeded`, plus a Segmentation
    tab in `volume_grid`.
- **Most of the GL viewer already exists** (§6): the `viewport` widget, tags, named cameras and
  `scenes:`. Four new pieces are needed, all reusable:
  1. an axis-aligned `volume_slice` node;
  2. a `mode: slice` orthographic camera;
  3. pointer events from a viewport into state (the open "click-to-world picking" item);
  4. overlays that stay on top of a slice.
- **Order** (§7):
  - P0a moves all of libcvc's process-wide state under the app.
  - P0b builds the shared pieces: the parameter table, the result-owning job, the pycvc job
    wrapper, the singleton scan and the legacy-goldens script.
  - P0c brings in the permissive libraries, each built for wasm-mt and checked for singletons.
  - P0d adds the shared contour and point-class types.
  - Then four tracks can start at once: the Skeletonization core, the ContourTiler core, the
    GenSeg core and the Ariadne viewport pieces.
  - VolumeGridRover follows the viewport pieces. Its Tile button follows the tiler, and its
    Segment button follows GenSeg.
- **Decisions for Joe** (§9):
  - how the new libraries arrive (done as recommended in cy-pca/cvcpkg#578);
  - how close ContourTiler must match its legacy output;
  - the API break P0a needs: settled 2026-11-09 as one 4.0 release, costing 8 downstream call
    sites;
  - names;
  - what may stay process-wide (signal traps, SDL's event queue, curl / ImageMagick / Jolt / HDF5
    init, CUDA device state).

---

## 1. The shape every port takes

### 1.1 Layers

The `mesh_lab` / `fe_lab` work set the pattern. Each library lands as up to six layers, and each
layer is useful on its own. Every layer builds and runs on wasm-mt as well as natively (§1.8).

| Layer | Where | What it is |
|---|---|---|
| 1. Core | `src/cvc/<area>/<lib>/` behind `inc/cvc/<area>/<lib>.h` | A headless `cvc::` API in the §1.4 shape, on `cvc::volume` / `cvc::geometry`. No Qt, no GL, no GPL code (§1.2), no singletons (§1.3), no `exit()`, no files read or written inside the algorithm. Long jobs are `cvc::sliced_job`s ([`sliced_task.h`](../../inc/cvc/core/sliced_task.h)), so they run cancellable and with progress on the background pool natively, and a few ms a frame on single-threaded wasm. |
| 2. CLI | `src/cvc-cli/main.cpp` | One `cvc` subcommand per library (`cvc skeleton`, `cvc tile`, `cvc segment`), with the legacy executables' options kept as aliases. Flags and help come from the parameter table (§1.5). |
| 3. Python | `bindings/pycvc` | SWIG wraps the public header (§1.4), so the C++ names carry over. |
| 4. DSL intrinsics | Registered on the app (§1.6) | Async functions for `cvc exec`, pycvc's `Exec`, arisrv programs and Ariadne hosts. |
| 5. Node type | `src/cvcGL/ariadne/` + `inc/cvc/gl/ariadne/` | A custom scene node, registered on the app (§1.3). It owns its data, runs the core's stepper as a sliced task on the app's state_exec scheduler (§1.9), keeps its parameters and data on the state tree (§1.5), and publishes `busy`, `stoppable`, `status`, `progress` and `phase`. Commands are a `request` key (ARIADNE.md §5.5). |
| 6. Component | `src/cvc/ariadne/components/<name>.ari` + an `INDEX.ari` entry | A window that binds those keys, so a document drives the node with no host C++. It takes the args `node`, `title` and `id`, like every node-scoped component since b2f54578. |

### 1.2 No GPL code: permissive libraries replace CGAL (settled 2026-11-08)

**Why CGAL can't stay.** CGAL's kernel, 2D polygons and number types are LGPL-3+. Every CGAL
algorithm package these ports use is GPL-3+ (CGAL's package overview):
- 3D Triangulations and the 3D Triangulation Data Structure;
- 2D Triangulations, including constrained Delaunay;
- 2D Regularized Boolean Set-Operations;
- 2D Arrangements and 2D Intersection of Curves (the sweep line);
- 2D Straight Skeleton and Polygon Offsetting;
- 2D Minkowski Sums;
- 2D Polygon Partitioning;
- the Linear and Quadratic Programming Solver;
- dD Spatial Searching;
- 2D Boolean Operations on Nef Polygons.

GMP and MPFR are LGPL, so they never made anything GPL, but they have no wasm-mt build, so the
exact number types (`Gmpz`, `Gmpq`, `Lazy_exact_nt`) block wasm. libcvc is LGPL-2.1, and its
headers say "version 2.1" with no "or later". So the ports drop even CGAL's LGPL-3+ kernel, and
libcvc keeps no CGAL at all. transfix/libcvc#1080 already did this for libcvc itself: its only
CGAL use was Spatial Searching (`nav/spatial.cpp`, `geometry/project_verts.h`), now nanoflann.

**The replacements**

| Need | CGAL (licence) | Replacement (licence) | Notes |
|---|---|---|---|
| 3D Delaunay, for Skeletonization's cocone | `Delaunay_triangulation_3` + TDS_3 (GPL) | Reentrant Qhull, `libqhull_r` (Qhull licence, permissive) | All state lives in one `qhT` per call, so no singletons. cvcpkg's qhull 8.0.2 gains wasm and wasm-mt in cy-pca/cvcpkg#578. Run as `d Qbb Qt Qz`: measured on a cube's corners plus its centre, that gives the exact 12 tetrahedra, while `QJ` or `Qt` alone add flat slivers (15–22 tetrahedra), and `QJ` with `QR0` varies from run to run (its seed is the clock). |
| Tetrahedral complex | TDS_3 (GPL) | `cvc::tet_complex`, new: cells, neighbours, mirror indices, the infinite cells of the hull, circumcentres | A small adapter presents the subset of the `Delaunay_triangulation_3` API the vendored cocone code calls, so that code changes little. |
| 2D polygon Booleans | Boolean set ops, Nef polygons (GPL) | Clipper2 2.0.1 (Boost Software Licence 1.0) | Exact on `int64` coordinates. New cvcpkg recipe for every platform, wasm and wasm-mt included (cy-pca/cvcpkg#578). 2.0 also triangulates polygons with holes (`clipper.triangulation.h`), but it takes no interior constraint segments, so CDT stays for the medial-axis fill. |
| Polygon offsets, Minkowski sums | Straight skeleton offsetting, Minkowski sums (GPL) | Clipper2 `InflatePaths`, `MinkowskiSum` | A single mitred pass differs from the straight skeleton past an edge event that leaves a reflex join (P0c's finding). `cvc::planar::wavefront_offset` (P3a) steps Clipper2 from event to event on a 2^20-times finer grid and matches CGAL's skeleton offsets in all 50 A/B cases |
| 2D constrained / plain Delaunay | 2D Triangulations (GPL) | CDT 2.0.1, `artem-ogre/CDT` (MPL-2.0, header-only; cvcpkg's `cdt`, cy-pca/cvcpkg#597) | Its robust predicates are William Lenthe's (BSD-3). MPL-2.0 is file-level copyleft: the CDT files stay MPL-2.0, which is compatible with LGPL code around them. The recipe patches its mutable statics away: a never-written `static double log2_constant` (`Triangulation.hpp:2337`) becomes `static const`, and Lenthe's error bounds, which had dynamic initialisers, are initialised from constant expressions. |
| Convex partition | 2D Polygon Partitioning (GPL) | CDT triangles, merged by Hertel–Mehlhorn where convex pieces are needed | `cvc::planar::convex_partition` (P3a). Valid by CGAL's own check, but never CGAL's pieces: Hertel–Mehlhorn on CDT's Delaunay triangulation gives 15–45% fewer pieces than `approx_convex_partition_2` on CGAL's non-Delaunay one (§3, parity) |
| Medial axis of untiled regions | Straight skeleton (GPL) | Boost.Polygon's segment Voronoi diagram (Boost Software Licence; Boost is already a dependency) | A true medial axis, not the straight skeleton's approximation of one, so the output differs from the legacy (§3, §9 item 2). Its input is `int32`, on the same snapping grid. `cvc::planar::medial_axis` (P3a): equal to CGAL's skeleton on convex polygons, within 1.7–7.4% of a polygon's size of it elsewhere, 14x faster. ContourTiler's live fill turned out not to use the skeleton at all (§3, "What P3a found") |
| Segment intersections, arrangements | Arrangements, the sweep line (GPL) | Boost.Geometry (Boost Software Licence) turns and self-intersection checks, or a small sweep on the exact predicates | Contours are small, so the simple one first. |
| Natural-neighbour interpolation, Sibson gradients, QP | Interpolation, the QP solver (GPL) | Sibson coordinates on CDT's Delaunay, Eigen least squares (MPL-2.0; already a dependency with libigl) | Only `interp.cpp`, 150 lines. |
| Exact predicates and numbers | `Gmpz`, `Gmpq`, `Lazy_exact_nt`, `MP_Float` (LGPL, and GMP) | Integer coordinates on a fixed grid, plus Shewchuk's robust predicates (public domain) | No GMP, so wasm builds. |
| k-nearest neighbours, if a port needs them | Spatial Searching (GPL) | nanoflann (BSD-2-Clause), vendored by #1080 | cvcpkg's `nanoflann` (cy-pca/cvcpkg#597) replaces the vendored copy once published |

**Considered and rejected**
- **Geogram** (BSD-3) has fast, exact, parallel 3D Delaunay and 2D CDT, and it builds with
  Emscripten. But it needs `GEO::initialize()`, which sets up process-wide logger, command-line
  and factory singletons. That breaks §1.3.
- **TetGen** is AGPL-3.
- **Shewchuk's Triangle** forbids commercial use.
- **VTK's Delaunay** classes are not robust on degenerate input, and VTK keeps process-wide
  factories.
- **The C++ straight-skeleton codes** found (Surfer2, monos) are GPL-3. No permissive C++
  straight skeleton turned up, which is why the medial axis comes from a Voronoi diagram.

**Rules for every third-party library a port brings in**
- Its licence is permissive or file-level copyleft (BSD, MIT, Boost, MPL-2.0, Qhull's). Never GPL
  or AGPL, and never "non-commercial".
- It builds for wasm-mt.
- It holds no mutable process-wide state, checked with the same scan as libcvc's own code (§1.3).
- It gets an entry in `THIRD_PARTY_NOTICES.md`, with its notice.
- It arrives as a cvcpkg package, never vendored into libcvc and never through an installed
  header (§9 item 1). nanoflann, vendored by #1080, moves to its package once published.

**The A/B against CGAL.** Every replacement is checked against the CGAL code it replaces, on the
same inputs, for results and for time.
- **Where it lives.** `tools/cgal_ab/` holds an optional test executable, `cgal_ab_test`, built
  only with `-DCVC_BUILD_CGAL_AB=ON`. It links CGAL, so as built it is GPL-3+: it is never
  installed, packaged or shipped, and libcvc itself still never links CGAL.
- **CI.** The `cgal-ab` job builds it natively on Linux whenever the compared code changes, and
  posts the PASS / FAIL and TIME lines to the job summary.
- **Seams.** Each port keeps the boundary between its algorithm and the replaced library narrow,
  so the A/B can drive the same algorithm from both sides:
  - Skeletonization's cocone code runs on `cvc::tet_complex`. The A/B builds one from Qhull and one
    from CGAL's `Delaunay_triangulation_3`, then compares the two triangulations (same tetrahedra in
    general position, empty circumspheres), the skeletons from each, and the times.
  - The tiler's `cvc::planar` operations (Booleans, offsets, constrained triangulation, medial
    axis) each get a CGAL counterpart in the harness. Results are compared by area, topology and
    Hausdorff distance, since the medial axis and the straight skeleton differ by design.
- **Already done.** transfix/libcvc#1080 started the harness with libcvc's own two replacements.
  Locally (Windows, CGAL 6.0.1), all checks pass:
  - nanoflann's fixed-radius neighbours equal CGAL's Kd_tree in all 15 cases (uniform, lattice
    with exact boundary distances, a clump with duplicates), 1.1–2.1× faster;
  - the nearest-vertex projection with k = 10 lands at CGAL's exact AABB closest distance on every
    query (a sphere's random and near-surface points, a sliver strip), 1.1–2.6× faster.

**If something can't be replaced after all.** The earlier plan stays as the fallback: a separate
`cvc_gpl` library that libcvc never links, `CGAL_LICENSE_ERROR` on the LGPL targets, the GPL
nodes reaching the LGPL ones only through the state tree, and `-gpl` packages. Nothing in this
plan needs it.

### 1.3 No singletons, anywhere in libcvc

**The rule.** All state lives under a `cvc::app` or in objects the caller owns. This holds for
libcvc, cvcGL, pycvc, the ports and every library they bring in. None of these may appear:

- namespace-scope or class-static mutable variables;
- function-local `static` variables, unless they are constant tables initialised from constants;
- `instance()` / `getInstance()` accessors, and registries reached without an `app`;
- `thread_local` used to carry context past a function's arguments;
- `srand` / `rand`, an unseeded `std::random_device`, or any process-global random generator;
- static counters (debug file numbering, id generators), static caches;
- the current working directory (no relative-path writes), or environment variables read inside
  an algorithm.

**What replaces them**
- **Registries, caches and counters** become members of `cvc::app` (or, for Ariadne, of its
  `AppRuntime`), and the functions that reach them take the app.
- **Each port's legacy globals** go into a context struct per call or per job
  (`skeleton_context`, `tiler_context`, `genseg_context`).
- **Random number generators** live in the context, seeded from a `seed` parameter.
- **The `cvc::app`** comes in explicitly. Volumes and geometry already hold their `app&`.
- **Logging** goes to `app.log()`, or into a result's `warnings`.

The state tree already works this way. libcvc 3.1.0 removed the global `cvcapp` / `cvcstate`
macros and the zero-arg `app::instance()` / `state::instance()`, and every root comes from its
`app`. What survived that release is below.

**libcvc's process-wide state today** (audit of master, 2026-11-08). The audit swept
`src/cvc`, `src/cvcGL`, `inc/cvc`, `src/cvc-cli` and pycvc's C++, without the tests and examples.
Its full list, with every file and line, is P0a's checklist.

| Subsystem | Mutable objects | of them locks | `thread_local`s |
|---|---|---|---|
| ariadne | 47 | 14 | 1 |
| cvcGL | 28 | 4 | 6 |
| state_exec | 15 | 3 | 30 |
| state | 11 | 2 | 4 |
| net | 9 | 2 | 0 |
| core | 8 | 1 | 9 |
| image | 5 | 1 | 0 |
| model, volren (CUDA included) | 4 each | 1 (volren) | 0 |
| geometry, processing, pycvc | 3 each | 1 (pycvc) | 2 (pycvc) |
| volume, nav, world, cli | 2 each | 1 (volume) | 3 (nav), 3 (world) |
| xmlrpc, lod, physics, tf | 1 each | 1 (physics) | 2 (physics) |
| ph, media | 0 | 0 | 1, 2 |
| **total** | **152** | **31** | **63** |

In vendored code, one is in use: LBIE's octree reseeds the C library's global generator with
`srand(time(NULL))` (`cvc-mesher/LBIE/octree.cpp:5502`), so mesh-quality improvement is
nondeterministic and disturbs any other `rand()` user.

**What the audit found beyond style.** These are bugs, and they go first:
1. **Duplicated handlers.** Every `cvc::app` constructor (`app.cpp:177`, through
   `registerDefaultHandlers`) inserts a fresh set of handler objects into the process-global volume
   and geometry I/O maps, with no lock and no de-duplication. The per-thread apps in
   `state_volume_codec.cpp` do it too.
2. **Registries hold callbacks bound to one app.** Action-intrinsic providers capture an `app*`;
   URI handlers capture one app's state root (`state://` and the cached `http` handler); the net
   and OIDC host cells hold one app. The last app to register wins, and the captures dangle once
   that app is gone. No library code calls `clear_action_intrinsics`, so the stateless cvcGL
   providers pile up, one set per `AriRuntime`.
3. **Stale per-app tables.** Some per-app tables are process maps keyed by `app*` that are never
   erased when the app dies: `stream_registry`, `exec_group`'s `single_noted` and `tf_binding`'s
   `seed_memory`. A new app at a reused address inherits the old one's entries.

**The groups, and what P0a does with each**
- **Registries:** node types, shader presets, widgets, `.ari` blocks, data decoders, action
  intrinsics, URI schemes, the volume / geometry / model / image I/O handlers, terrain and city
  generators, the player factory, script engines, the state startup hooks and the compression
  codecs. Each becomes a member of `cvc::app` (or of `AppRuntime`), and the free registration and
  lookup functions take the app.
- **Per-app side tables keyed by `app*`:** data stores, stream registries, cluster shards, live
  groups, tf bindings. They become plain app members.
- **Configuration cells:** the net and OIDC hosts, the HTTP client and options provider, the play
  policy, the URL opener and notice, browser hooks, resolver byte caps, the threaded-host flag,
  ImGui font settings, and the VTK mapper policies. They move onto the app, or onto the object
  they configure.
- **Caches:** generated-world building libraries, the HTTP single-flight table, shader sources,
  pycvc's interned names. They become app members. A cache of immutable content is shared only by
  passing it in explicitly.
- **Counters and ids:** sequence numbers, owner tags, scheduler and registry serials. Each moves
  to the object that hands it out (the app, a scheduler, a cycle registry). An evaluator that runs
  without an app gets its ids from its scheduler or registry.
- **Random generators:** `rand()` in smoothing, the net verbs' and `math.random`'s thread-local
  engines, LBIE's `srand`. Each becomes an engine in its owning object, seeded from a parameter.
- **`thread_local`s:** per-thread scratch buffers (nav, world, ph, TurboJPEG handles) and
  scope-bound re-entrancy guards hold nothing between calls, so they stay (§9 item 5). The ones
  that keep state across calls or carry context implicitly move:
  - the per-thread `cvc::app` in `state_volume_codec.cpp` (the codec signature gains an app);
  - state_exec's per-thread cycle registry and its caches keyed by registry id;
  - the per-thread failure records;
  - `math.random`'s fallback engine.
- **Resources the process holds once:** signal dispositions, process priority, the environment and
  working directory, SDL's event queue and its window and timer routes, the X11 clipboard
  connection, curl / ImageMagick / Jolt / HDF5 / gRPC global initialisation and locks, and the
  CUDA devices' constant tables and memory. libcvc can't make these per app.
  - They go on one documented list (§9 item 5).
  - Each is wrapped so that libcvc keeps no state of its own about it beyond what the resource
    demands (a reference count, a lock).
  - Environment and working-directory writes go away where they can: `web_document`'s `chdir`
    becomes an explicit base directory.

**The hardest moves** (the audit's judgement):
- the state startup hooks and their static-initialisation registrars (`xmlrpc` starts itself
  through one, which goes with XML-RPC itself, §1.10);
- the URI resolver, whose `resolve()` / `resolve_async()` / `resolve_to_file()` are app-free and
  called from about 66 places in Ariadne and cvcGL;
- the geometry, model and image I/O, whose `read_geometry(filename)`, `read_model(filename)` and
  `image::load(path)` take no app and are used by pycvc, the CLI and cvcGL;
- `cvc::net`'s client and browser cells;
- the CUDA caches, where per-app copies would duplicate device memory;
- state_exec's ids and per-thread fallbacks, which sit on hot paths, where evaluators are meant to
  work without an app.

**What the ports bring, and what goes**

| Port | Singletons and globals in the legacy code |
|---|---|
| Skeletonization | the global `bounding_box` vector, never cleared; `srand(0)` / `rand()` for colours; degenerate-case debug files in the current directory |
| ContourTiler | log4cplus: 156 `getInstance` calls and 104 function-static loggers in 48 files; ContourFilter's `Controls::instance()`, which the app's `.ser` reader goes through; static counters naming debug files |
| GenSeg | about 25 globals; `extern VolMagick::VolumeFileInfo loadedVolumeInfo`, set by the reader; output files in the current directory |
| SegMed | not audited yet (P6 does it) |
| VolumeGridRover | nothing carries over; its `QSettings` cache directory and temp files stay behind |

ContourTiler's logging goes through a shim whose logger is a member of `tiler_context`. Each
vendored function that logs gets the context passed in, which is a mechanical change across the
48 files. Trace output nobody needs is compiled out instead.

**How it is checked**
- **A CI scan of the object files, for all of libcvc.** For every object of libcvc, cvcGL,
  pycvc and the vendored libraries, list the writable data symbols: `nm` on Linux, the `.data` /
  `.bss` symbols, including function-local statics and their guard variables. Fail on any that
  is not on the allowlist.
  - The allowlist starts as the audit's list. As P0a lands it shrinks to §9 item 5's short list
    of resources the process itself holds once. A new entry needs a reason in review.
  - This catches what a grep misses: a static inside a macro, or a vendored template.
- **Concurrency tests** (§1.7 T6): two jobs with different parameters on two threads give exactly
  what each gives alone, under the TSan lane (`sanitizers / tsan`).
- **Order tests:** running A then B, B then A, and each alone all give the same answers. This
  catches leftover state such as the bounding box.
- **Two apps:** two `cvc::app`s in one process, each with its own node types, intrinsics, I/O
  handlers and documents, never see each other's. Every subsystem P0a moves gets such a test.

### 1.4 The API

**The shape of each port's public header**

- **Functions:** free functions in `cvc::` on value types (`cvc::volume`, `cvc::geometry`,
  `cvc::contour_stack`, `cvc::point_classes`), in the form `result f(const input &, const params
  & = {})`.
  - No out-parameters, no raw owning pointers, and no callbacks in the main entry.
  - The `cvc::app` arrives inside the value types, or as an explicit first argument. Never from a
    global.
- **A params struct** of plain fields: numbers, bools, strings, enums, and `std::vector`s of them.
  - Its defaults match the legacy defaults.
  - It has a parameter table (§1.5), which gives it `from_state`, `to_state` and `validate()`.
    `validate()` returns a list of what is wrong, not just a bool.
- **A result struct** of values: volumes, geometry, vectors, a `warnings` list and timings.
- **A stepper and a job** (§1.9):
  - `f_stepper` is the routine itself, resumable: `step(budget)`, `phase()`, `progress()`,
    `status()`, `cancel()` and `take()`, like `fem::stepped_solve`.
  - `std::unique_ptr<result_job<R>> f_job(input, params)` wraps it. `cvc::result_job<R>` (P0b) is
    a `sliced_job` that owns the stepper and its input (shared, read-only), and hands out the
    result with `result()` once `done()`.
  - The blocking function is the stepper run with budget 0, so the three can't drift apart.
- **Errors** are exceptions derived from `std::runtime_error`. The message says what was wrong
  with the input, for example `segment_seeded: seed (12, 40, 7) is outside the 64x64x32 volume`.
  - Never `exit()`, `abort()`, or a printed message followed by a bad result.
  - A degenerate case the legacy wrote debug files for becomes a warning in the result.
- **Determinism:** the same input and parameters give the same output, bit for bit, on one
  platform and build. Anything random takes a `seed` parameter.
- **Concurrency:** functions are safe to call concurrently (§1.3). Inside, they may use
  `app.computePool().parallel_for`.

**pycvc**

- **The real header, wrapped.** SWIG wraps the port's header directly, as it already does
  `pycvc_algorithm.h`, and as the state_exec parity work does (PYCVC_STATE_EXEC_PARITY.md).
  - `cvc::segment_seeded` is `pycvc.segment_seeded`, and `seeded_segmentation_params` is a
    class with the same fields.
  - There is no facade layer. The shape rules above are what make this work: plain structs,
    value returns, `std::vector` / `std::map` with `%template`, and enums wrapped as enums.
- **Keywords.** Every params struct takes keywords, generated from its table:
  `pycvc.segment_seeded(vol, seeds, t_low=40, t_high=200)`. An unknown keyword raises
  `TypeError` and lists the valid ones.
- **Arrays.** Volumes and label volumes come back as zero-copy numpy views (`ArrayView`), as
  volumes already do.
- **The GIL** is released for the computation (`gil_release`, as in `pycvc_algorithm` and
  state_exec), so other Python threads keep running.
- **Jobs.** `sliced_job` isn't wrapped today. P0b wraps `result_job<R>` once, generically, with
  `step`, `done`, `cancel`, `progress`, `phase`, `status` and `result`. A notebook steps a job
  between cells, or runs it with a progress callback; the GIL is released inside each step.
- **State.** `from_state` and `to_state` take a `pycvc.State`, following the parity doc's rule
  that tree arguments are `State`s.
- **No current app.** pycvc already has no module-global current app (`pycvc_context.h`), and
  the ports add none.
- **The browser.** pycvc-gl's wasm build (`pycvc-gl-cp312`) gets the same bindings, so a notebook
  in the browser calls the ports too.

### 1.5 Parameters and data on the state tree

**The rule.** Every parameter a user or a document can set is a state key. Every input and result
a node holds is on the state tree. So a document, a DSL program, pycvc and an arisrv client all
see and set the same things. Nothing lives only in a C++ member or a widget.

**The parameter table.** `mesh_lab` today seeds and reads each key by hand: one
`seed_num(a, base, "smooth.iterations", ...)` per key in `mesh_nodes.cpp`. So each parameter's
name, default and type are written in three places: the struct, the seed call and the read. The
ports instead declare each params struct once, as a table (`inc/cvc/core/param_table.h`, P0b):

```cpp
template <> struct param_table<seeded_segmentation_params> {
  static constexpr auto fields = std::tuple{
      field("t_low", &seeded_segmentation_params::t_low, unit("data"),
            doc("voxels below this are never reached")),
      field("gamma", &seeded_segmentation_params::gamma, range(-1.0, 0.0),
            doc("front speed exp(gamma * |grad I|)")),
      field("automatic", &seeded_segmentation_params::automatic,
            doc("automatic seeds added to the point classes")),  // enum: choices from its names
      // ...
  };
};
```

Boost.Describe would do as the reflection underneath; cvcpkg's Boost 1.86 has it. From one
table, generic code gives:
- **`to_state(p, node)` and `from_state(node)`:** one key per field under the node, for example
  `<node>.grid.segment.t_low`.
  - Missing keys are seeded with the default.
  - A value that doesn't parse is reported in `status`, not silently replaced by the default.
- **State properties on each key:** `unit`, `min`, `max`, `doc`, and `choices` for enums. These
  use the existing property mechanism (STATE_API.md, "Property Management"), so a component or a
  generic inspector can label and bound a slider without hard-coding anything.
- **The CLI flags and their help.** Legacy aliases are added by hand.
- **pycvc keywords and `from_dict`.**
- **The DSL dict keys an intrinsic accepts,** and `(params.defaults "segment.seeded")`.
- **A generated parameter table** for each port's docs.

**Bulk data goes on the data channel.** State values are strings (STATE_EXEC_DEVELOPER_GUIDE.md
§12.5), so bulk data goes on the data channel instead: `state::data()`, which is
`state-data-get` in the DSL and a `data_object` there.

| Node | Values (keys) | Data channel |
|---|---|---|
| `mesh_lab` (skeleton request) | `mesh_ops.skeleton.*` params, `info.strands`, `info.sheets`, warnings | `<node>.skeleton.strands`, `<node>.skeleton.sheets` (geometry) |
| `contour_lab` | params, `source.node`, each component's name, colour and visibility | `<node>.tiler.contours` (`contour_stack`), `<node>.tiler.surfaces.<component>` (geometry) |
| `volume_grid` | params, `view`, slice indices, cursor, tool; class and object metadata (name, colour, radius) as lists a `repeat:` can bind | `<node>.grid.volume`, `<node>.grid.points` (`point_classes`), `<node>.grid.contours` (`contour_stack`), `<node>.grid.segment.labels`, `<node>.grid.segment.classes`, `<node>.grid.surfaces.<object>` |

Nodes can read their inputs from the tree too: a node with `source: { node: <id> }` reads that
node's documented data key, so `contour_lab` can tile a `volume_grid`'s contours.

**When the tree is read and written.** The core algorithm never touches the tree. A job reads its
parameters when it starts, on the scheduler thread, and writes its results when it is done, in
`on_done`. Async kernels must work this way anyway: they never touch the state tree. A parameter
changed while a job runs applies to the next run, and `status` says so.

**Sessions.** A node's subtree is its session.
- `request: save_session <f>` writes the values as JSON (`state::json()`), and the data through
  the port's I/O handlers.
- `load_session` reverses it.

### 1.6 DSL intrinsics

**How they work.** Each operation runs its job (§1.9) as a sliced task on the app's scheduler
(`schedule_sliced_job`, `cvc/core/sliced_task.h`), not as a `register_async_fn` kernel that
blocks one pool call to the end. A blocking kernel has nowhere to run on single-threaded wasm,
and a whole skeleton or segmentation can't fit in one call's budget. Each operation has three
forms:
- **`(<op> args...)`** returns the result. The calling process parks on the task's completion,
  costing the scheduler nothing, while the task runs in slices beside every other process. For
  scripts: `cvc exec`, pycvc's `Exec`, arisrv programs.
- **`(<op>-async args...)`** returns a future at once, for a program that does other work and
  `(await f)`s later.
- **`(<op>-start args... target)`** returns the task's pid at once. Under the state path
  `target`, the task publishes the keys a node publishes (§1.9: `busy`, `status`, `progress`,
  `phase`) plus its `pid`, and writes the result to `target.result` on the data channel. This is
  the form for Ariadne actions.

The task shows in `(ps)` as `<op>/cooperative <subject>` or `<op>/threaded <subject>`.
`(job.cancel pid)`, or killing the process, cancels the job at its next step. A task started
from a document carries the document's owner tag, so closing the document stops it.

**Proposed intrinsics** (module-prefixed like the standard library; names are §9 item 4):

| Intrinsic | Does |
|---|---|
| `(skeleton.compute g [params])` | Skeleton of a geometry. Returns a dict of `strands` and `sheets` data objects and `warnings` |
| `(tiler.tile stack [params])` | Tiles a `contour_stack`. Returns a dict from component name to geometry |
| `(segment.seeded vol classes [params])` | §5's segmentation. Returns a dict of `labels`, `classes` and `border` |
| `(segment.critical-points vol [params])` | The paper's seeds as a two-class `point_classes` |
| `(contours.read path)`, `(contours.write stack path)` | `contour_stack` I/O, any format §3 lists |
| `(point-classes.read path)`, `(point-classes.write pc path)`, `(point-classes.add pc class i j k)` | Point-class I/O and edits. These are small, so they are plain synchronous natives |
| `(params.defaults name)`, `(params.check name dict)` | Defaults and validation from a parameter table, synchronous |
| `(job.cancel pid)` | Cancels a task started by any `-start` form, at its next step |

`skeleton.compute`, `tiler.tile`, `segment.seeded` and `segment.critical-points` each also come
as `-async` and `-start`, as above.

**Arguments and results**
- An input is a data object (from `state-data-get`) or a state path string such as
  `"doc.grid.volume"`. Paths are resolved in the marshal step, as the async rules require.
- Parameters are a dict, or a state path to a parameter subtree read through the table.
- Results are fresh data objects. The program stores them with `state-data-set`.

**Registration.** One function per port takes the app and an environment, for example
`cvc::register_segmentation_intrinsics(app, env)`. Every host gets them: `cvc exec`, the app's
exec scheduler, pycvc's `Exec`, and Ariadne's lanes through the app's own provider list (§1.3).

**Budgets in Ariadne.** An Ariadne action runs with `ariadne.limits.action_max_time` (0.1 s) and
`action_max_steps` (5000). Those limits bound the action, not the work it starts:
- `(<op>-start ...)` returns in microseconds, so an `on:` action can start a skeleton or a
  segmentation under the default limits. Nothing has to be raised.
- The work runs in its own process, which has no step, time or memory limit. A sliced task runs
  for thousands of frames, but each frame's share is bounded: about `slice_ms` (4 ms) of work on
  the scheduler thread in cooperative mode, or nothing at all in threaded mode, where it waits on
  a pool chunk. Its memory is C++-side, not DSL heap.
- A button binds to the same keys a node's request does: a `progress:` widget on
  `target.progress` labelled by `target.phase`, and a Stop button calling `(job.cancel)` on
  `target.pid`.
- Only the waiting form, `(<op> ...)`, would hold an action past its limit. Each intrinsic's
  documentation says to use `-start` in actions, and T10 checks that `-start` runs under the
  default limits.

Inside Ariadne, a node's `request` key stays the usual way to run a port: it starts the same job,
and the node publishes the same keys.

**What doesn't need intrinsics.** Node edits such as point toggles and contour edits are state
writes (`pick.*`, `request`), and a DSL program can already script them by writing keys.

### 1.7 Tests

**The rule.** Every port lands with every row of this matrix that applies to it. A phase isn't
done until its rows pass on Linux, Windows, macOS and wasm-mt. The new test executables join the
sanitizer lanes' `SAN_CXX_TESTS` list, so they also run under the `tsan` and `asan-ubsan` lanes.

| # | Kind | What it checks | Examples |
|---|---|---|---|
| T1 | Legacy parity | The port's output equals the legacy library's on committed inputs. The legacy runs are scripted (`tools/volrover-goldens/`, at a pinned volrover sha, built with its own CGAL: GPL code runs there to make test data, and none of it is linked into libcvc), so the goldens can be regenerated | GenSeg labels voxel for voxel; skeleton strand and sheet counts, with vertices to 1e-9 on inputs in general position; tiler surfaces by topology and Hausdorff distance (§3) |
| T2 | Units | Each internal piece on its own | `tet_complex` against brute-force Delaunay checks (empty circumspheres); cocone poles, the medial-axis filter, U1 clustering; the planar module's predicates, snapping, Booleans and medial axis; tiler correspondence and branching; the fast-marching heap order, the 26-neighbour update, the gradient |
| T3 | Known shapes | Exact topology on synthetic inputs | a torus gives one closed strand; a slab one sheet; a capsule one segment; two cylinders' contours give two genus-0 tubes; two blobs give two labels |
| T4 | Invariants | Properties that hold for any input, over seeded generated inputs | the skeleton is unchanged by rigid motion and point order; a tiled surface is closed and manifold, does not intersect itself, and has every vertex on a contour plane; every labelled voxel connects to a seed of its class through voxels above `t_low`; labels never cross a ridge |
| T5 | Bad input | Exceptions with useful messages; never a crash or an exit | empty input, one point, coplanar and cospherical points, one slice, an open or self-intersecting contour, no seeds, a seed outside the volume, a constant volume, NaN voxels, the wrong voxel type |
| T6 | No singletons | §1.3 | two threads with different parameters give the same results as each alone (TSan); A then B equals B then A equals each alone; two apps in one process |
| T7 | Resumable | The stepper matches the blocking call bit for bit at step budgets of 1, 64, a seeded random budget per step, and 0. A cancel in each phase leaves no result and no leak (ASan). Progress rises monotonically within [0, 1], and the phases are named. The slowest step at the default budget is recorded natively and under node, and stays under the port's bound (§1.9). Stepping costs at most a few percent over budget 0. The node's demo with `CVC_SLICED_THREADING=cooperative` keeps drawing frames while its job runs | skeleton: a step inserts at most a few points, however large the cloud; GenSeg: a step pops a bounded number of voxels, however large the volume |
| T8 | State tree | For every field, struct to tree to struct round-trips. Defaults are seeded, properties written, and a bad value reported. A node's request produces its result at the documented data keys, with `status` | |
| T9 | I/O | Every reader and writer round-trips, the legacy sample files read, and a small corpus of truncated and garbage files raises | `.ser`, `.config` + `.pts`, `.contour`, `.pcd`, `.vgr`, `.pts` |
| T10 | DSL | Each intrinsic's three forms in a `state_exec` test, cooperative and threaded, with a kill and a `job.cancel` part-way through. One Ariadne document starts it with `-start` from an `on:` action under the default limits (not raised): its progress keys rise while the frame loop runs, and the result lands at `target.result` | |
| T11 | pycvc | Each function and params class from Python: keywords, numpy views, another Python thread running during the call (GIL released), and a job cancel | |
| T12 | CLI | Each subcommand on sample input, the legacy flag aliases, and the outputs read back | |
| T13 | Ariadne | A golden image per demo document; scripted picks through state writes (`volume_grid`); headless runs | |
| T14 | Wasm | T1–T10 run as wasm-mt gtest binaries under node; the node type's demo document runs in arigl-web under headless Chrome | |
| T15 | Licence | Every library in the port's dependency closure has a permissive or MPL-2.0 licence, and a notice in `THIRD_PARTY_NOTICES.md`; no CGAL header is included anywhere in libcvc, cvcGL or pycvc (a grep in CI; `tools/cgal_ab` is the one place allowed) | |
| T16 | A/B against CGAL | The port's replacements against the CGAL code they replace, on the same inputs, in `cgal_ab_test` (§1.2): results compared exactly where they must agree and by tolerance where they differ by design, and both sides timed | Qhull vs `Delaunay_triangulation_3` and the skeletons built on each; `cvc::planar` vs CGAL's Booleans, offsets, CDT and straight skeleton, and the tiled surfaces from each |

ContourTiler's 12 switched-off CppUnitLite2 tests, with their expected outputs, are converted to
gtest and count toward T1 and T2. `test_intersection.cpp` alone is 2,252 lines.

### 1.8 Wasm from the start

**The rule.** Every port builds for wasm-mt (the `wasm32-emscripten`, `-pthread` target that
libcvc, arigl-web and the libcvc-examples-web gallery use) from its first PR, and its tests run
there in CI (T14). There is no later web phase.

**Why this is possible now.**
- With CGAL gone (§1.2), nothing in the ports needs GMP or MPFR, which have no wasm-mt build.
- The replacements are plain C or C++, and none of them needs threads or a filesystem.
- GenSeg was always CGAL-free.

**What it takes**
- **A wasm-mt build of every new library.** cy-pca/cvcpkg#578 adds wasm and wasm-mt to qhull, and
  a new clipper2 recipe for every platform, wasm and wasm-mt included. CDT, Shewchuk's predicates
  and nanoflann are header-only cvcpkg packages for every platform (cy-pca/cvcpkg#597).
  - Checked locally with emsdk: both libraries build for wasm-mt with `-pthread`.
  - A smoke program linking them, with CDT, runs under node 22. Qhull's cube-plus-centre gives
    12 tetrahedra, and two triangulations on two threads equal the sequential ones (13,040 and
    19,597 tetrahedra). Clipper2's union, intersection, difference, offset and Minkowski sum give
    the expected areas. CDT triangulates a constrained square into 4 triangles.
- **A `ports (wasm-mt, node)` CI lane**, modelled on the existing `physics replay (wasm-mt,
  node)` lane: build libcvc for wasm-mt and each port's gtest binaries, then run them under
  node.
- **Each node type's demo** in the arigl-web lane (headless Chrome), with its golden image.
- **Resumable steppers** (§1.9) are what make the long runs usable on single-threaded wasm: about
  4 ms of work a frame on the scheduler thread, with the page interactive throughout. Wasm-mt and
  native builds run the same steppers in pool chunks.
- **The browser pycvc** (`pycvc-gl-cp312`, wasm) gets the bindings in the same PR as native
  pycvc.

### 1.9 Long routines are resumable (settled 2026-11-09)

**The rule (Joe, 2026-11-09).** Every routine that can take longer than a frame is a resumable
stepper. It runs on the app's scheduler in slices (`cvc/core/sliced_task.h`), publishes its
progress for a progress bar, stops on request, and keeps the frame loop interactive on one
thread. Single-threaded wasm has no other thread.

libcvc already works this way: `fem::stepped_solve` (Poisson, heat, eigenmodes),
`contour_tree_builder`, `contour_spectrum_builder`, `sdf_stepper`, `mesher_stepper`,
`resize_stepper`, `tet_boundary_stepper` and `world_builder`. `fe_lab` and the transfer function
editor's overlays drive them. The ports follow those classes rather than inventing their own.

**The stepper.** There is one class per routine, beside its blocking function: for example
`skeleton_stepper` beside `cvc::skeletonize`.
- **`step(budget)`** does at most about `budget` units and returns true once done; budget 0 runs
  to the end. A unit is one item of whatever loop is running: a point inserted, a cell or facet
  visited, a voxel popped from the heap, a contour vertex, an FFT line. Each costs at most about a
  microsecond.
- **Every phase is sliced,** the input checks, setup and clean-up included. A loop whose single
  iteration has no bound, such as a walk along a graph or a region grown from one seed, stops
  mid-walk and resumes, as `contour_tree_builder`'s walk along an arc does. What runs outside the
  count is O(1) per phase change.
- **Reporting:**
  - `phase()`, a static string such as `"inserting"`, `"cocone"` or `"marching"`;
  - `progress()`, in [0, 1] and monotone;
  - `status()`;
  - `take()` for the result.
  A failure is an exception the step throws once and rethrows afterwards. `cancel()` is the one
  thread-safe member.
- **The blocking function runs its stepper with budget 0.** So the two can't drift apart, and the
  result is the same, bit for bit, whatever the step schedule (T7).
- **The stepper is the per-call context of §1.3.** All the routine's state is its own, and its
  input is shared and read-only. It is not thread-safe, but it may move between threads between
  steps, which is what threaded mode does.

**Third-party calls that can't yield.** Qhull, Clipper2, CDT, Boost.Polygon's Voronoi diagram and
PocketFFT each run a call to the end. A port either reaches them in increments small enough to
bound a step, or calls them on pieces small enough to count as one unit:
- **Qhull:** incremental insertion, a few points per unit, in a spatial order. `qh_setdelaunay`
  lifts the points, `qh_findbestfacet` locates one, and `qh_addpoint` inserts it (§2).
- **Clipper2, CDT and the medial axis:** one call per slice pair and component (§3), bounded by
  the contour sizes.
- **PocketFFT:** a 3D transform as three passes of 1D transforms, each over a slab of planes (the
  TexMol plan's F2Dock).

Each call that stays indivisible is listed in its port's section with its measured worst case,
and T7 fails a step that overruns the port's bound. None of them may become the single-thread
fallback for a whole phase. If a library can't be reached in small enough pieces, the port
replaces that call.

**The job around it.** `cvc::result_job<R>` (P0b) adapts any stepper to `cvc::sliced_job`. It owns
the stepper, forwards `step`, `phase`, `progress`, `status` and `cancel`, and hands out `result()`
once done. `schedule_sliced_job` runs it in one of two modes:
- **cooperative** (single-threaded wasm, or `CVC_SLICED_THREADING=cooperative`): about
  `slice_ms` (4 ms) of work per frame on the scheduler thread. The budget is sized from the
  measured cost, and restarts small at each phase change.
- **threaded** (native, and wasm-mt served cross-origin isolated): chunks of about `chunk_ms`
  (250 ms) on the app's background pool, so the scheduler pays nothing.

Either way, `on_progress` (at most every 100 ms) and `on_done` run on the scheduler thread, so they
may write state.

**What a node publishes,** as `fe_lab` does (`mesh_nodes.cpp`), under its state name:
- `busy`, `stoppable`, `status`, `progress` (0–100), `phase` (`"inserting 42%"`, or `""` when
  idle) and `stats.work_ms`;
- its request key starts a job (`request = "segment"`), and `stop` cancels it at the next step,
  keeping what the node had;
- its component shows a `progress:` widget bound to `progress` and labelled by `phase`, with a
  Stop button visible while `stoppable`, as `fe_controls` does.

The DSL's `-start` forms publish the same keys under their target (§1.6).

**Measured, not assumed.** Each port's T7 records:
- the slowest step at the default budget, natively and under node;
- the overhead of stepping against budget 0 (`contour_tree_builder` measured +0.1% to +4%);
- a frame-time run of the node's demo in cooperative mode: the frame loop keeps drawing while
  the job runs, and no frame gives the job more than `slice_ms` plus the port's worst indivisible
  piece.

### 1.10 No XML-RPC (settled 2026-11-09)

**The rule (Joe, 2026-11-09):** remove the legacy XML-RPC, and F2Dock's server's use of it, since
the network side is moving to arisrv. No port brings an XML-RPC mode back.

**What libcvc has today.** It is built only with `CVC_USING_XMLRPC` (off by default, and off in
the cvcpkg recipes), but parts of it reach every build:
- the vendored XmlRpc++ (`src/xmlrpc`, `inc/xmlrpc`, about 2.8k lines);
- `cvc::xmlrpc`: `xmlrpc_server.cpp` serves `cvcstate_get_value`, `cvcstate_set_json`,
  `cvcstate_get_children` and the like, on a thread started from the `__system.xmlrpc` keys, and
  `xmlrpc_client.cpp` calls them;
- `cvc server` and `cvc client` in the CLI, and the `XmlRpcServerTerminate` exception;
- in every build, `cvc/utility/utility.h` includes `<xmlrpc/XmlRpc.h>` and declares
  `XmlRpcValue`-returning call helpers, `XMLRPC_DEFAULT_PORT` (23196) and
  `get_xmlrpc_host_and_port`. That is why the SDK always installs `inc/xmlrpc`.

**Who uses it**
- F2Dock's `F2DockServer`, which serves TexMol's `F2DockClient` (the TexMol plan's §3.1).
- VolumeRover2's GenSeg remote mode (§5).
- Before removal, a search of the downstream repos (volrover3, F2Dock, TexMol, cypkg, nicotina,
  the examples) confirms there is no other user.

**What replaces it.** What XML-RPC did is all on arisrv now:
- reading and writing another process's state: arisrv's state replication and transports
  (WebSocket, gRPC);
- calling a method on a server: `room-call` (ARIADNE_SCRIPTING_HOSTS S2);
- running remote compute: `cvc::jobs` (the TexMol plan's §5), whose kinds are this plan's
  steppers.

**When**
- The removal is part of the 4.0 break (§9 item 3), because `utility.h`'s helpers and the
  installed `inc/xmlrpc` headers are public API. It goes in P0a's state-startup group: the
  `xmlrpc` startup registrar is deleted, not converted.
- F2Dock's side follows the TexMol plan. The service runs the legacy engine executable, which
  never touched XML-RPC, and `F2DockServer` is retired (its F3). F2Dock then builds against a
  libcvc without `cvc::xmlrpc`.

**Checked by** T15's grep, extended: no `XmlRpc` header or symbol in libcvc, cvcGL, pycvc or the
CLI. The removal PR deletes `utility_test`'s XML-RPC cases and the CLI and build docs' sections
(CLI_GUIDE.md, FULL_BUILD_CVCPKG.md, TESTING.md).

---

## 2. Skeletonization

**What it is.** `src/Skeletonization` (7.9k lines; `tcocone.cpp` is 1,883 of them), Goswami and
Bajaj, UT 2008, LGPL-2.1 headers like libcvc. `Skeletonization::skeletonize(geom, params)`
(`Skeletonization.cpp:162`) reads only the input's vertices, so in practice it takes a point
cloud. It then runs:

1. an optional Robust Cocone filter of the points;
2. a 3D Delaunay triangulation;
3. Tight Cocone;
4. the medial axis;
5. U1, the unstable manifolds of index-1 saddles, clustered into planar patches (the sheets);
6. U2, a graph turned into polylines (the strands);
7. clean-up (`star`, sort by area, drop small clusters).

The result is `Simple_skel`: line strips plus polygons, each vertex with a colour. Its CGAL use
is EPICK plus `Delaunay_triangulation_3` with custom vertex and cell bases. There is no Qt (its
CMake calls `SetupQt()` for nothing), and no GL, GSL or VolMagick. No paper is cited in the code.
The Dey–Goswami cocone and Goswami–Dey–Bajaj unstable-manifold lineage is inferred.

**Without CGAL** (§1.2)
- **The triangulation:** reentrant Qhull's Delaunay of the points, run as `d Qbb Qt Qz`, which is
  deterministic.
  - On a cube's corners plus its centre that gives the exact 12 tetrahedra.
  - Joggling (`QJ`) adds flat slivers and is seeded from the clock. Slivers put Voronoi vertices
    far away, which is what the cocone and medial axis must not see.
  - `tet_complex` still flags any near-flat cell. The A/B against `Delaunay_triangulation_3` (T16)
    decides on real degenerate samples whether that is enough.
- **`cvc::tet_complex`** holds it: cells with their four vertices and four neighbours, the mirror
  index across each facet, infinite cells closing the convex hull, per-cell and per-facet flag
  arrays (the legacy cell base's fields), and cached circumcentres (the Voronoi vertices).
- **An adapter** gives the vendored cocone code the handles and calls it uses (`vertex(i)`,
  `neighbor(i)`, `mirror_index`, `is_infinite`, `finite_cells`, `incident_cells`, `dual`), so
  the 7.9k lines change little.
- **Exact predicates** (orientation, insphere) come from Shewchuk's code, and the few
  constructions use doubles, as EPICK did.
- **Parity:** on inputs in general position, Qhull and CGAL give the same Delaunay, so T1 holds
  exactly. On degenerate inputs (cospherical grid samples) they may break ties differently, so T1
  compares topology there.

**Resumable** (§1.9). `skeleton_stepper` runs the seven stages above as phases, each a loop that
stops at its budget:
- **`"ordering"`:** the bounding box and Qhull's `Qbb` scale, then a spatial order of the points (a
  Hilbert or BRIO sort, sliced like `contour_tree_builder`'s radix sort), so each insertion's
  point location walks a short way. A unit is a point.
- **`"inserting"`:** the Delaunay triangulation built point by point, rather than in one
  `qh_new_qhull` call. It starts with an initial simplex, then for each point `qh_findbestfacet`
  and `qh_addpoint` on the lifted coordinates (`qh_setdelaunay`). A unit is a point. Qhull's
  closing `qh_triangulate` (for `Qt`) is the one indivisible call, linear in the facets, and
  measured in T7. The Robust Cocone pre-filter's own Delaunay is built the same way.
- **`"building"`:** `tet_complex` filled from Qhull's facets, a facet per unit.
- **`"circumcentres"`, `"poles"`, `"cocone"`, `"medial axis"`:** loops over cells, vertices and
  facets, an element per unit.
- **`"sheets"` (U1) and `"strands"` (U2):** union-find over facets and walks along the graph. A
  walk stops mid-way and resumes.
- **`"clean-up"`:** a cluster or polyline per unit.

The incremental build has to prove itself in P1's first PR. It must give the batch build's
triangulation (exactly in general position, the same topology on the cospherical sample), with
its time measured against the batch build in `cgal_ab_test`. If it can't, the triangulation
becomes libcvc's own incremental Delaunay on Shewchuk's predicates, behind the same
`tet_complex`. One batch `qh_new_qhull` call is never the single-thread fallback, because a large
cloud's batch build runs far past a frame.

**What the port fixes on the way in**

- A global `bounding_box` vector gets six `push_back`s per call and is never cleared, so every
  call after the first uses the first call's box. It moves into `skeleton_context` (§1.3), and
  the T6 order test guards it.
- `srand(0)` / `rand()` pick the colours, which changes the process's C random state. Colours
  become a deterministic function of the cluster index, or are left to the renderer.
- Degenerate cases write debug files (`degen_tetra`, `*.degen`, `*.degen_VF`) to the current
  directory. They become warnings in the result.
- The `exit(1)` paths are only in debug writers and `main.cpp.unused`, so they go with them.
- The strand pass keeps Voronoi-vertex strips only when the cell radius is in a hard-coded
  (2.0, 3.0) window. That is protein-helix specific. Make it an off-by-default parameter, or
  leave it to a SecondaryStructures port.
- The VolumeRover2 dialog never initialises Flat Phi (`SkeletonizationDialog.cpp:87-90`) and
  hard-codes `DEFAULT_SHARP`. Both become ordinary parameters, in the table.
- VolumeRover2 drops the sheets and keeps only strand vertices, with no connectivity. The port
  returns both.
  - A sheet polygon looks like a Voronoi face, which is convex, so a fan would triangulate it
    (check this).
  - `PolyTess`, the legacy ear-clipper, is unused and carries a non-commercial licence, so it
    stays behind.

**One cocone core, not four.** Skeletonization's `tcocone.cpp`, `robust_cc.cpp`, `intersect.cpp`
and `init.cpp` differ from the copies in TightCocone, PocketTunnel and Curation by 10–25 lines
each. Port them once, as an internal `cvc::cocone` module on `tet_complex`. A later TightCocone
port (surface reconstruction from points) and PocketTunnel then reuse it, rather than vendoring a
fifth copy.

**API sketch** (`inc/cvc/geometry/skeleton.h`)

```cpp
namespace cvc {
struct skeleton_params {                       // + param_table<skeleton_params> (§1.5)
  bool robust = false;                         // Robust Cocone pre-filter
  double big_ball_ratio = 1.0 / 16, theta_if = 5 deg, theta_ff = 10 deg;
  double cocone_phi = pi / 8, sharp_phi = 2 * pi / 3, flat_phi = pi / 3, flatness_ratio = 1.44;
  double medial_theta = pi / 8, medial_ratio = 64;
  bool discard_small = false; double threshold = 0.1; int min_cluster_count = 2;
  bool helix_window = false;                   // the legacy (2.0, 3.0) strand filter
};
struct skeleton {
  geometry strands;   // lines() + per-vertex colours
  geometry sheets;    // the planar patches, triangulated; one colour per patch
  std::vector<int> sheet_of_tri, strand_of_line;
  std::vector<std::string> warnings;
};
skeleton skeletonize(const geometry &points_or_mesh, const skeleton_params & = {});
class skeleton_stepper;                        // step(budget), phase, progress, take (§1.9)
std::unique_ptr<skeleton_stepper> skeletonize_stepper(geometry in, skeleton_params p);
std::unique_ptr<result_job<skeleton>> skeletonize_job(geometry in, skeleton_params p);
}
```

The defaults match the legacy `Parameters` (`Skeletonization.h:42-54`) and the dialog.

**Surfaces**

- **CLI:** `cvc skeleton in.{off,ply,raw,rawc} -o out.{vtk,raw} [--robust ...]`. The legacy
  tool's flags (`-r -bbr -mr -hc -thif -thff -plc -medth -medr`, from `main.cpp.unused`) are
  accepted as aliases.
- **pycvc:** `pycvc.skeletonize(geom, robust=True, ...)` and `skeletonize_job`.
- **DSL:** `(skeleton.compute g params)`, with `-async` and `-start` (§1.6).
- **Ariadne:** a `skeleton` request on `mesh_lab`, run as a sliced task that publishes
  `mesh_ops.skeleton.{busy,progress,phase,status}` for the Skeleton section's progress bar and
  Stop button (§1.9).
  - Its parameters live under `<node>.mesh_ops.skeleton.*`, and its results on the data channel
    (§1.5).
  - It adds two children, `<id>_strands` and `<id>_sheets`, tagged `skeleton`, so a `viewport`
    can show the skeleton alone.
  - `mesh_tools.ari` gains a Skeleton section. A volume reaches it through an isosurface, as
    `fe_lab` does today.
  - The previous revision made it a node of its own only to keep GPL code out of cvcGL. That
    reason is gone.

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1 | strand and sheet counts and vertices against the legacy library on three committed point clouds in general position; topology on one cospherical grid sample |
| T2 | `tet_complex` (every circumsphere empty, neighbours and mirror indices consistent, the hull closed by infinite cells); poles, cocone flags, the medial-axis filter, U1 clustering and U2 strands, one at a time |
| T3 | torus, slab, capsule, and two spheres joined by a rod |
| T4 | unchanged by rigid motion and point order |
| T5 | fewer than four points, coplanar, collinear and cospherical points, duplicates |
| T6–T16 | as the matrix |

---

## 3. Contour tiling

There are two legacy tilers, and only one is worth porting.

- **`src/Tiling`: do not port it.** This is the 1996 Bajaj–Coyle–Lin tiler ("Arbitrary Topology
  Shape Reconstruction from Planar Cross Sections", GMIP 58(6)).
  - No CMakeLists has added its directory since the 2011 switch to ContourTiler, and nothing
    active calls it.
  - Its code is SHASTRA/Purdue software whose notice (`main.cpp:1-12`) allows educational use
    only and forbids commercial use, so it cannot go into libcvc.
  - It also has about 35 mutable globals and 136 statics with a partial reset, 75 `exit()` calls
    in library code, and a temp-file round trip inside `surfaceFromContour`.
  - Keep only its contour file format (`.config` with `PREFIX` / `SUFFIX` / `SLICE_RANGE`, plus
    numbered `.pts` files) as a reader. That is a format, not code.
- **`src/ContourTiler`: port this one.** It is John Edwards' reimplementation of the same
  algorithm, extended to many components, with removal of intersections between components (the
  README targets neuropil "AxonsDendrites" serial-section data). It is built by default and
  reached from VolumeRover2's Tools > Contour Tiler.
  - Licence: LGPL like libcvc (Joe, 2026-11-08). Its files carry no header, so vendoring adds
    libcvc's.
  - Size: about 24k source and 13k header lines, tests and tools included.
  - No Qt, GL, VolMagick or GSL. Errors are exceptions, not `exit()`, apart from
    `smfconvert.cpp`, a tool file that sits in the library's source list.

**ContourTiler's CGAL, and what replaces it.** This is the largest single job in the plan. The
library includes about 60 different CGAL headers, and its basic types (`Point_2`, `Polygon_2`,
`Polygon_with_holes_2`) come from CGAL's kernel throughout. The GPL packages are concentrated in
a few files:

| Files | CGAL used | Replacement |
|---|---|---|
| `skeleton.cpp` (1,185 lines: `medial_axis_stable`, `triangulate_to_medial_axis`, the untiled regions) | Straight skeleton, offset builder, CDT, partition, natural neighbours, Delaunay 2D | Boost.Polygon medial axis; CDT; Hertel–Mehlhorn |
| `interp.cpp` (150 lines) | Natural-neighbour coordinates, Sibson gradient fitting, the QP solver | Sibson coordinates on CDT's Delaunay; Eigen least squares |
| `offset_polygon.cpp`, `minkowski.cpp` | Straight-skeleton offsets, Minkowski sums, approximated offsets | `cvc::planar::wavefront_offset` (Clipper2 stepped through the skeleton's events), `minkowski_sum` |
| `polygon_utils.cpp`, `augment.cpp`, `contour_utils.h` | CDT, partition, sweep line, arrangements | CDT; Boost.Geometry turns or a small sweep |
| `arrange_polygons.cpp`, `polygon_difference.cpp`, `polygon_intersection.cpp`, `Hierarchy.h` | Boolean set ops on `Lazy_exact_nt<Gmpz>` | Clipper2 on `int64` |
| `common.h`, `CGAL_hash.h`, `bso_rational_nt.h` | Kernels: Cartesian, EPECK with sqrt, Nef's extended kernels, `Gmpq`, `MP_Float` | The planar module's types |
| everything | `Point_2/3`, `Segment_2/3`, `Polygon_2`, `Bbox_2` (LGPL) | The planar module's types |

**The plan: a planar-geometry module, then the tiler on top.**
1. **`cvc::planar`** (`src/cvc/geometry/planar/`), new and small:
   - points on a fixed `int64` grid, with a per-stack scale and origin chosen from the stack's
     bounding box, so every contour vertex snaps exactly once;
   - Shewchuk's orientation and in-circle predicates;
   - polygons and polygons with holes;
   - Clipper2 Booleans and offsets;
   - CDT triangulation;
   - the Boost.Polygon medial axis, sampled to polylines;
   - Sibson interpolation;
   - its own T2 tests against brute force.
2. **ContourTiler vendored on top of it.** Its typedefs in `common.h` point at `cvc::planar`
   types, and the GPL calls in the files above are rewritten. The tiler logic itself
   (correspondence, tiling regions, branching, intersection removal) stays as it was.
3. **Parity.** The medial axis replaces the straight skeleton, and integers replace exact
   rationals, so the untiled regions' fill and the intersection removal can differ slightly from
   the legacy. T1 therefore compares surfaces by topology (genus and components per surface) and
   by Hausdorff distance below a tolerance. Tiles away from untiled regions should match exactly.
   §9 item 2 asks whether that is enough.
   - **What P3a found (2026-11-11).** ContourTiler's live untiled-region code does not use the
     straight skeleton. `medial_axis_stable` (`skeleton.cpp`, called from
     `tiler_operations.cpp:1509`) splits each untiled region into convex pieces with
     `approx_convex_partition_2` (Greene's as the fallback), puts a vertex at each piece's
     centroid and one on each diagonal between two pieces (where the line between their centroids
     crosses it, or its midpoint), and lifts them off the slice planes by natural-neighbour
     interpolation (`interp.cpp`). The straight-skeleton fill,
     `triangulate_to_medial_axis`, is never called. So the medial axis does not move the fill;
     the partition does. `tools/cgal_ab`: cvc::planar's triangulations and Sibson coordinates
     equal CGAL's (exactly, and to 1e-14), but its convex partition never equals CGAL's (15–45%
     fewer pieces, from a Delaunay rather than a plain constrained triangulation underneath). An
     untiled region's interior vertices will differ from the legacy's; its boundary, and every
     tile outside untiled regions, need not.

**ContourTiler's other rough edges, and what the port does with them**

| As it is | In libcvc |
|---|---|
| `tile(begin, end, comp2color, options)` mutates the input slices and writes `.rawc` files; it returns nothing (`tiler.cpp:555-576`) | Takes a const `contour_stack` and returns one `cvc::geometry` per component; files only from the CLI |
| log4cplus in 48 files: 156 `getInstance` calls, 104 function-static loggers | A shim whose logger is a member of `tiler_context`, passed to the functions that log (§1.3); no process-wide logger |
| Two `.ser` readers. The app's goes through ContourFilter's `Controls::instance()` singleton, which is reconfigured on every read, has hard-coded ignored names, and drops a whole component if one contour is non-simple. The CLI's uses tinyxml | Port the tinyxml reader into `contour_stack` I/O. The singleton stays behind. ContourFilter's smoothing waits until someone needs it |
| GCC-only flags baked in (`-g -pthread -frounding-math`, `-rdynamic`) | Gone with CGAL |
| 12 `test_*.cpp` on CppUnitLite2, switched off (`test_intersection.cpp` alone is 2,252 lines, with expected outputs) | Converted to gtest: T1 and T2 coverage for free, with tolerances where §3's parity note applies |
| `cvcraw_geometry::contours_t` stores `ContourTiler::Slice`, a reverse dependency | `cvc::contour_stack` is the shared type, and the tiler converts at its edge |
| Static counters for debug file names, and `_otv.g` / `_verts.g` debug outputs | Dropped, or a `debug_dir` parameter |

**The shared contour type** (`inc/cvc/geometry/contours.h`), used by the tiler and by
VolumeGridRover:

- slices keyed by z index, with a z spacing (the legacy hard-codes 0.05 in
  `cvcraw_geometry/contours.cpp:72-83`) and an origin;
- each slice maps a component name to closed planar polygons;
- each component has a colour.
- **Readers:** Reconstruct `.ser` plus its numbered section files, and the Tiling `.config` plus
  `.pts`.
- **Writers:** Tao Ju's `.contour` (ctr2suf), `.pcd`, and a libcvc-native file for
  VolumeGridRover sessions.
- I/O handlers registered on the app (§1.3), with `contours.read` / `contours.write` intrinsics
  and T9 corpora.

**API sketch** (`inc/cvc/geometry/tiler.h`)

```cpp
namespace cvc {
struct tile_params {                              // + param_table<tile_params>
  double z_scale = 1, collinear_epsilon = 0, curation_delta = 1e-8;
  bool remove_intersections = false, interp_untiled_regions = true;
  double correspondence_overlap = -1;
  std::vector<std::string> include, exclude;      // component name regexes (-c / -x)
  int z_first = -1, z_last = -1;                  // -s
};
struct tile_result {
  std::map<std::string, geometry> components;
  std::vector<std::string> warnings;
};
tile_result tile_contours(const contour_stack &, const tile_params & = {});
class tile_stepper;                               // step(budget), phase, progress, take (§1.9)
std::unique_ptr<tile_stepper> tile_contours_stepper(contour_stack in, tile_params p);
std::unique_ptr<result_job<tile_result>> tile_contours_job(contour_stack in, tile_params p);
}
```

**Resumable** (§1.9). Progress counts slice pairs of components, so it is real, but a step is
smaller than a pair:
- **`"snapping"`:** every contour vertex onto the `int64` grid, a vertex per unit.
- **`"tiling"`:** per slice pair and component, the tiler's stages in order: correspondence,
  tiling regions, branching, untiled regions and intersection removal. Within each, a unit is a
  contour vertex or a triangle emitted.
  - Each Clipper2 Boolean or offset, CDT triangulation and Boost.Polygon medial axis is called on
    one pair's contours. That call is the indivisible piece, and its cost grows with those
    contours' vertex count.
  - T7 records the worst pair in the committed `.ser` series. A pair whose contours are too large
    for a frame is split before the call: Clipper2 and CDT run per connected region, and the
    medial axis per untiled region.
- **`"assembling"`:** the per-component surfaces stitched and their normals computed, a triangle
  per unit.

**Surfaces**

- **CLI:** `cvc tile in.ser -o outdir`, keeping `cl_parse`'s flags. The VolumeRover2 dialog passes
  them as free text, by default `-C 0.01 -e 1e-15 -o raw`.
- **pycvc:** `pycvc.tile_contours(stack, ...)` and `tile_contours_job`.
- **DSL:** `(tiler.tile stack params)`, with `-async` and `-start` (§1.6).
- **Ariadne:** a `contour_lab` node type. Its `tile` request runs as a sliced task and publishes
  §1.9's keys under `<node>.tiler`.
  - Its input is a file (`source: { file }`), or another node's contours from the state tree
    (`source: { node: <id> }`).
  - It draws a stack's contours as polylines in their component colours (tag `contours`). On a
    `tile` request it adds the surfaces as children (tag `tiles`).
  - Its keys live under `<node>.tiler.*`, and its surfaces on the data channel.
  - The `contour_tiler.ari` component has a component picker, the slice range, the parameters,
    Tile and Stop, and a progress bar labelled with the slice pair being tiled.

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1–T2 | the 12 converted legacy tests; surfaces against the legacy library on two committed `.ser` series, by topology and Hausdorff distance; `cvc::planar` against brute force |
| T3 | a cylinder stack gives a genus-0 tube; a branching Y gives one surface; two touching components with `remove_intersections` give two disjoint surfaces |
| T4 | closed and manifold, no self-intersection, every vertex on a contour plane, volume within 2% of the prism estimate |
| T5 | one slice, empty slices, an open contour, a self-intersecting contour, reversed orientation, coordinates at the edge of the snapping grid |
| T9 | every `contour_stack` format round-trips; a Reconstruct series reads |
| T6–T16 | as the matrix |

---

## 4. VolumeGridRover as an Ariadne component

**What the legacy tool is today.** It is a 7,002-line `VolumeGridRover.cpp`.

- **Views:** XY, XZ and ZY slice views, one at a time. A tab widget (`m_SliceCanvasTab` in
  `VolumeGridRoverBase.ui`, tabs "XY", "XZ" and "ZY") switches between them, and the widget docks
  beside VolumeRover's main 3D view. Each tab is a `SliceCanvas` (a QGLViewer) with an
  orthographic camera, no rotation, middle-button zoom, a depth slider and Reset View.
- **Probe:** hovering fills a probe with the voxel's index, object coordinates, RGBA and value.
- **Point classes:**
  - double-click toggles a point;
  - a rubber band selects, with Shift / Ctrl / Alt modifiers, and a drag moves the selection;
  - Delete removes it; the arrow keys and PageUp / PageDown step slices;
  - classes save as `.vgr` XML (loading is stubbed) and points load from `.pts`.
- **Segmentation:** the point classes are seeds for GenSeg (§5), run locally or over XML-RPC. It
  leaves `SegTmp.rawv` in the current directory, and nothing in VolumeRover2 loads it.
- **Contours:** per-object contours on slices, with interpolation, export (`.pcd` and `.contour`
  work; `.cnt`, `.bspline` and `.matc` were never ported to Qt6) and tiling into surfaces.

In the Qt6 code, though, most of the contour side is dead:

- you cannot create a contour (`addContourSlot` returns early) or enter contour mode (the tab
  signal is never connected);
- the Load button is disconnected;
- tiling, SDF2D, the medial axis, curation and EM are commented out or compiled out.

VolumeGridRover never calls either tiler.

Rendering: a `GL_R8` slice texture with a 256-entry transfer-function texture through an ARBfp1.0
program (paletted and SGI colour-table fallbacks), immediate-mode quads, `gluDisk` glyphs and
`GL_SELECT` picking. Each depth change re-reads one slice through a VolMagick subvolume read,
which is the out-of-core path. The memory-mapped readers are not even built.

**So this port is a rebuild of what VolumeGridRover was for, not a translation of the widget.**
None of its Qt or GL code carries over. Its data model, file formats, interaction design (the view
switch included) and a few algorithms do. volrover's history from before the Qt6 migration may
hold the last working contour and tiling code. Read it before P10; the shallow clone used for
this sketch has none of it.

**The node: `type: volume_grid`** (`inc/cvc/gl/ariadne/volume_grid_nodes.h`)

- It reads `source: { file, var, time }` as a volume node does.
- **Children:**
  - three `volume_slice` nodes (§6 item 1), tagged `slice_xy`, `slice_xz` and `slice_zy`;
  - per slice, overlays carrying the same tag: point-class glyphs, contour polylines, crosshairs
    where the other two planes cross, and the segmentation's labels as a see-through colour
    layer;
  - a `<id>_surfaces` group for tiled surfaces and segmented classes' surfaces (tag `surfaces`).
- One scene, tagged groups: the XY view shows `slice_xy`, and the 3D view shows all three planes
  with the surfaces, the classic orthoslicer.
- Edits run in C++ in the node, where they are fast and testable. The DSL only wires widgets to
  keys.
- No singletons: two `volume_grid` nodes in one document, or in two apps, share nothing (T6).

Keys under `<node>.grid.*`. The parameters among them come from parameter tables (§1.5), with
their units and ranges as properties. Bulk data is on the data channel:

| Key | Meaning |
|---|---|
| `source.file`, `source.var`, `source.time` | what `load` reads; `info.dims`, `info.min`, `info.max` and `info.span` are published |
| `view` | which slice the switch shows: `xy`, `xz` or `zy` (the legacy tabs); a document, the DSL or pycvc can set it |
| `slice.x`, `slice.y`, `slice.z` | voxel indices; written by the sliders, the keys and clicks |
| `cursor.{i,j,k,x,y,z,value}` | published on hover: the probe |
| `tool` | `navigate`, `points`, `contours`, `probe` |
| `pick.*` | Written by the views (`pick:` on the viewport, §6 item 3). The tick consumes each new pick according to `tool`: `points` toggles a point (it adds one, or removes the nearest within `display.point_size` on this slice); `contours` appends to the current object's contour on this slice, at its nearer end; a Shift-drag selects with a rubber band |
| `classes`, `classes.current` | point-class metadata: name, colour, radius, as a list a `repeat:` can bind |
| `objects`, `objects.current` | contour-object metadata: name, colour, interpolation (`linear` or `spline`), samples |
| `display.point_size`, `display.show_points`, `display.grey`, `display.background` | display (a parameter table) |
| `segment.t_low`, `segment.t_high`, `segment.gamma`, `segment.seeds` (`local_maxima`, the default; `critical_points`; `none` for the point classes alone), `segment.show`, `segment.opacity` | §5's parameters, from `param_table<seeded_segmentation_params>`, and how the labels draw |
| `tile.*` | §3's parameters, from `param_table<tile_params>` |
| `request` | `load`, `save_classes <f>`, `load_classes <f>`, `load_points <f>`, `save_contours <f>`, `load_contours <f>`, `delete_selected`, `tile`, `segment`, `save_segments <f>`, `save_session <f>`, `load_session <f>`, `undo`, `stop` |
| `busy`, `stoppable`, `status`, `progress`, `phase`, `stats.work_ms` | published while a load, segmentation or tiling runs as a sliced task (§1.9); the component's progress bar and Stop button bind them, and the views stay live meanwhile: slices can be scrolled and points placed while a segmentation marches |
| data: `volume`, `points`, `contours`, `segment.labels`, `segment.classes`, `surfaces.<object>` | the loaded volume, the `point_classes`, the `contour_stack`, the segmentation's and the tiler's results (§1.5) |
| `<node>.transfer_function` (+ `.histogram` …) | as on every volume node, so the `tf_editor` colours the slices |

**Data and algorithms**

- **Point classes:** a small `cvc::point_classes` type (classes × voxel points, per variable and
  timestep), with `.vgr` read and write and a `.pts` reader. The legacy `.vgr` form is
  `<pointclassdoc><pointclass name color variable timestep><point>"x y z"</point>`. The points
  live on the data channel; the class metadata are keys.
- **Contours:** the §3 `contour_stack`, one component per object.
- **Splines:** the legacy resampled with GSL splines every frame. Use a small natural-cubic or
  Catmull-Rom spline in libcvc instead, computed when a contour changes, so wasm needs no GSL.
- **Tile:** the §3 tiler on the drawn objects. The surfaces appear in the 3D view in the objects'
  colours. This finally wires up the `tilingComplete` the legacy declared and never emitted.
- **Ported later, on demand, on §1.2's libraries:**
  - SDF2D, the 2D signed distance of contour polygons (nanoflann for the nearest edges), for a
    "show SDF" overlay;
  - `medax.h`, a 2D medial axis, from `cvc::planar`'s Boost.Polygon medial axis;
  - Kinney's b-spline fit (`bspline_fit.cpp`, not compiled today);
  - slice isolines through libcontour's 2D path, already in-tree (`cvc-mesher/contour/conplot2d`).
- **Segment:** §5's `segment_seeded`, with the point classes as seeds.
  - The labels show on the slices in the classes' colours.
  - Each class's surface shows in the 3D view, as an isosurface of its mask.
  - `save_segments` writes the per-class volumes the legacy left in the current directory.
  - This also finally loads the result, which VolumeRover2 never did.

**The component: `volume_grid.ari`**

- **`volume_grid`:** one window holding:
  - the tool as radio buttons;
  - **the views, in one of two layouts** (the `layout` arg):
    - **`switch`** (the default, as the legacy): one slice viewport in `mode: slice` with
      `input: pick`, under an XY / XZ / ZY switch bound to `grid.view`, beside a 3D orbit view;
    - **`quad`**: XY, XZ and ZY slice viewports and the 3D view in a 2×2 grid, all at once.
  - under each slice view, a slider bound to its axis's `grid.slice.*`, with the index in the
    view's caption;
  - a side panel with tabs:
    - Probe;
    - Point classes: list, colour, radius, add, delete, save, load;
    - Contours: objects, colour, interpolation, samples, save, load, Tile;
    - Segmentation: thresholds, γ, seed mode, Segment, Stop, label opacity, save;
    - Transfer function: the `transfer_function_editor` unit.
  - Slider ranges and labels come from the keys' properties (§1.5), not from the component.
- **`volume_grid_views`** and **`volume_grid_tools`:** the views alone and the panel alone, for a
  document that lays them out itself.
- **Args:** `node`, `title`, `id`, `view_size`, `layout` (`switch` or `quad`).
- **Demo:** `examples/ariadne/documents/volume_grid.ari` on a shipped volume, natively and in
  arigl-web, with a golden image per layout and an interaction test that scripts picks and the
  view switch through state writes.

---

## 5. Seeded segmentation (GenSeg)

**What it is.** `src/Segmentation/GenSeg` is about 2.8k lines; `segment.cpp` is 1,969 of them. It
is CVC code under the LGPL and uses no CGAL. Its headers ask users to cite the paper it
implements:

> C. Bajaj, Z. Yu, M. Auer, "Volumetric feature extraction and visualization of tomographic
> molecular imaging", *Journal of Structural Biology* 144(1–2):132–143, 2003,
> doi:10.1016/j.jsb.2003.09.037 (a PDF is on Bajaj's UT page:
> <https://cs.utexas.edu/~bajaj/papers/2003/journal/sdarticle04.pdf>).

The method is the paper's §2.2, "Boundary segmentation". GenSeg is what VolumeGridRover's
Segmentation tab runs, with the point classes as seeds (`generalSegmentation`, `genseg.h:48`).

**The method in the paper**

- **Fast marching:** fronts grow by fast marching (‖∇T‖·F = 1) at speed F = exp(γ‖∇I‖), with
  γ < 0, so they slow at edges.
- **Seeds:** the critical points of the gradient field after anisotropic gradient vector
  diffusion (§2.1, Eqs. 2–3). Maxima seed the feature group, minima seed the background, and
  saddles are dropped. In the paper's §4, maxima below a threshold move to the background and
  minima above one move to the feature.
- **Stopping:** fronts of one group merge when they meet, and fronts of different groups stop at
  their common boundary. That gives fast marching the stopping criterion it otherwise lacks.
- **More groups:** more than two groups give multi-material segmentation.

**What GenSeg does: a user-seeded, multi-class variant**

1. Normalise the volume to 0–255. Take the gradient magnitude by central differences, normalised
   to 0–255.
2. Find automatic seeds. These are the local maxima above `t_high` of a lightly smoothed copy
   (two passes of a 7-point average), not the paper's diffused-gradient critical points.
3. Place each class's user seeds (the point classes) at time 0.
4. Classify the automatic seeds. A threshold sweeps down from 255 to `t_low`, and each seed takes
   the class whose front reaches it first through voxels above the threshold.
5. Run multi-label fast marching from every labelled voxel through voxels above `t_low`. The time
   step is `0.01·exp(0.12·|∇I|)`, which is the paper's speed with γ = −0.12. A voxel where two
   classes meet is a border voxel.
6. Write, per class, `<prefix>_subunit_<n>.rawiv`: the class's voxels with their intensity and
   zero elsewhere. Also write the border voxels as `_indices.txt` and `_indices.pcd`.

**SegMed is the paper's automatic mode.** `src/Segmentation/SegMed` is about 3.1k lines and cites
the same paper. Its `GVF_Compute` (`GVF.cpp:69`) finds the critical points of the diffused
gradient field, and its `Segment` marches from them into `<file>.seg.rawiv`. Nothing in
VolumeRover2 calls it.

**Why it doesn't work properly today**

- **Dangling pointers:** `generalSegmentation` keeps `string(params[0]).c_str()` and
  `string(params[4]).c_str()`, which point into temporaries destroyed at the end of each
  statement. It then opens the input and names the outputs through them, which is undefined
  behaviour on every run.
- **Global state:** about 25 globals, plus an `extern VolMagick::VolumeFileInfo loadedVolumeInfo`
  that the reader sets. Only one run at a time per process is safe. They all move into
  `genseg_context` (§1.3).
- **Files instead of memory:**
  - it reads its input from a file and writes its outputs to the current directory;
  - VolumeGridRover then regex-scans the current directory for `*_subunit_N.rawiv`, deletes those
    files and writes `SegTmp.rawv`, which nothing loads;
  - both worker threads read Qt widgets off the GUI thread.
- **Error handling:** `exit(0)` when an output file can't be opened, and the critical-point list
  leaks.
- **Units:** thresholds are on the 0–255 normalised scale, not in the data's units.
- **Remote mode:** a remote mode goes over XML-RPC.

**API sketch** (`inc/cvc/volume/segmentation.h`)

```cpp
namespace cvc {
enum class seed_mode { none, local_maxima /* GenSeg */, critical_points /* the paper, SegMed */ };
struct seeded_segmentation_params {   // + param_table<seeded_segmentation_params> (§1.5)
  double t_low = 0, t_high = 0;      // data units...
  bool normalized_thresholds = false; // ...or the legacy 0-255 scale
  double gamma = -0.12;               // speed F = exp(gamma * |grad I|), |grad I| on 0..255
  seed_mode automatic = seed_mode::local_maxima;  // added to the user's seeds; the default stays
                                                  // local_maxima until the paper mode matches
  // critical_points: diffusion iterations, k (Eq. 3), the feature / background thresholds
};
struct seeded_segmentation {
  volume labels;                      // uint8: class index, or unlabelled / border
  std::vector<volume> classes;        // the legacy _subunit_<n>: the class's voxels, zero elsewhere
  std::vector<std::array<int, 3>> border;
  std::vector<std::string> warnings;
};
seeded_segmentation segment_seeded(const volume &, const point_classes &seeds,
                                   const seeded_segmentation_params & = {});
class segment_stepper;                // step(budget), phase, progress, take (§1.9)
std::unique_ptr<segment_stepper>
segment_seeded_stepper(volume in, point_classes seeds, seeded_segmentation_params p);
std::unique_ptr<result_job<seeded_segmentation>>
segment_seeded_job(volume in, point_classes seeds, seeded_segmentation_params p);
point_classes critical_point_seeds(const volume &, const seeded_segmentation_params & = {});
std::unique_ptr<result_job<point_classes>>
critical_point_seeds_job(volume in, seeded_segmentation_params p);
}
```

**Resumable** (§1.9). Every step of GenSeg is a loop over voxels or a heap, so the whole run slices
finely:
- **`"normalising"`:** the 0–255 rescale and the central-difference gradient, a voxel per unit.
- **`"seeding"`:** the two 7-point smoothing passes and the local-maximum scan, a voxel per unit.
  In paper mode, the gradient vector diffusion's iterations (`cvc::gradient_vector_diffusion`)
  and the critical-point search are sliced the same way, a voxel update per unit, so hundreds of
  iterations on a large map never hold a frame.
- **`"classifying"`:** the threshold sweep from 255 down to `t_low`, a voxel reached per unit.
- **`"marching"`:** multi-label fast marching. The heap, the arrival times and the labels belong
  to the stepper, so a step pops at most a budget of voxels and the next one carries on.
- **`"writing"`:** the labels, the per-class volumes and the border list, a voxel per unit.

`volume_grid` draws the labels as the front advances. At most a few times a second, at the end of
a slice (cooperative) or a pool chunk (threaded), the task copies the labels so far for the node
to draw. It never reads them while a chunk is writing them.

**Order of work**

1. **Match the legacy output.** Vendor GenSeg's marching into `genseg_context`, with in-memory
   input and output. Keep the hand-unrolled 26-neighbour update as it is for this release.
   - Run the legacy code once on small synthetic volumes with fixed seeds: two blobs, a blob
     touching a shell, and a noisy one. The `tools/volrover-goldens/` script does this and
     commits the expected label volumes.
   - The port must match them voxel for voxel, natively and on wasm, before any clean-up.
2. **Clean up:** compact the unrolled update, take thresholds in data units, and pick the heap.
   The parity tests guard each change.
3. **Add the paper's automatic mode.** Audit SegMed for singletons (§1.3), then port its
   diffused-gradient critical points as `seed_mode::critical_points` and
   `critical_point_seeds`: maxima join the feature, minima the background, with the paper's §4
   threshold reclassification.
   - Without a click, this gives the paper's fully automatic two-class segmentation. With point
     classes, it gives more groups.
   - The gradient vector diffusion lands as a shared `cvc::gradient_vector_diffusion`, since
     SecStruct, SegMonomer and the paper's §2.3 volumetric skeletons use the same field.
4. **Seed generators:** fold in transfix/libcvc#123's seed generators (`criticalPntsToSeed`,
   `SeedProduce*`).
5. **The default seeding** (settled 2026-11-08): `volume_grid` and the CLI keep GenSeg's local
   maxima until the paper mode matches the legacy results on real maps. Then the default switches
   to `critical_points`, in its own PR, with the comparison in its description.

**Surfaces**

- **CLI:** `cvc segment in.rawiv --seeds classes.vgr --tlow 40 --thigh 200 -o out`. It writes the
  labels volume and the per-class volumes. `--auto critical-points` runs the paper's mode with no
  seeds file.
- **pycvc:** `pycvc.segment_seeded(vol, seeds, t_low=40, ...)`, `segment_seeded_job` and
  `critical_point_seeds`.
- **DSL:** `(segment.seeded vol classes params)` and `(segment.critical-points vol params)`, each
  with `-async` and `-start` (§1.6).
- **Ariadne:** the Segmentation tab and the `segment` request in `volume_grid` (§4).

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1 | voxel-for-voxel labels against the legacy library on the three committed volumes; SegMed's critical points against its legacy output |
| T2 | heap order, the 26-neighbour update, the gradient, the 7-point smoothing, the threshold sweep |
| T3 | two blobs give two labels; one seed of each class in one blob splits at the ridge; a shell around a blob gives two nested labels |
| T4 | every labelled voxel connects to a seed of its class through voxels above `t_low`; no label crosses a ridge above `t_high`; border voxels touch two classes |
| T5 | no seeds, a seed outside the volume, a seed below `t_low`, a constant volume, NaN voxels, every voxel type |
| T6–T16 | as the matrix |

**Not in this effort:** the virus-specific siblings SegCapsid, SegSubunit and SegMonomer, and
SecStruct. Together they are about 30k lines. Each would reuse the fast-marching core and the
gradient vector diffusion. The paper's §2.3 volumetric skeleton (a skeleton magnitude map traced
with Canny's hysteresis) is also a later item. It complements §2's cocone skeleton, which needs a
surface sample rather than a volume.

---

## 6. Ariadne pieces VolumeGridRover needs

Most of the GL-viewer work is already on master. What exists, and what is new:

**Already there**

- **The `viewport` widget** (ARIADNE.md §5.18, `src/cvcGL/ariadne/scene_views.cpp`):
  - a view of the scene inside a window, with its own camera at
    `<prefix>.viewers.<view>.camera`;
  - nodes picked by `tags:` (`show:` / `hide:` / a `bind:` key), and per-view `override:`
    restyling;
  - named cameras, `scenes:` (separate SceneGraphs shown by the views that name them), and a
    `caption`.

  It is a mirror viewport of the main one, drawn in a rect cut out of the window's background,
  so it scrolls and clips with its window. `scene_view.ari` is it in a window of its own.
- **Custom node types** with request keys and sliced jobs (`mesh_nodes.cpp` is the template).
  After P0a they register on the app, not in a process-wide table (§1.3).
- **The transfer function editor** (`tf_editor`, `transfer_function_editor` unit). Every volume
  node already publishes its model, histogram, contour spectrum and contour tree, so a volume
  grid node gets the colour map editor for free.
- **Subvolume reads:** `volume_file_io::readVolumeFile(ctx, vol, file, var, time, subvolbox)`, the
  VolMagick heritage, so a slice of a large file can be read without reading the volume. This
  replaces VolumeGridRover's memory-mapped `MappedRawIVFile` / `MappedRawVFile` for display.

**New, and reusable outside VolumeGridRover**

1. **An axis-aligned slice node** (`type: volume_slice`): one plane of a volume, as a textured
   quad coloured through the node's transfer function, at `<node>.slice.{axis,index}`.
   - It uploads one 2D texture per slice change (read by region for big files), so stepping
     through a 2k³ volume never needs the volume on the GPU.
   - Today the only axis-aligned cut is fe_lab's `<id>_slice`, an FE field on tetrahedra.
     `volslice` slices face the camera, which is a different thing.
   - Useful on its own: the classic three-plane orthoslicer next to a `volren`.
2. **A slice camera mode for the viewport** (`mode: slice`, `axis: x | y | z`, the axis bindable
   to a key so one viewport can follow the view switch): parallel projection looking down the
   axis, no rotation; drag pans, the wheel zooms about the pointer, and double-click refits the
   slice.
   - `mode: map` is a top-down view along z only, and it orbits, so it is not enough.
   - The camera controller already has the parallel-projection plumbing for `map`.
3. **Pointer events from a viewport to state** (`input: pick`, `pick: <key>`, `on_pick:`). A click
   or drag in the view writes where it landed, and the left button no longer turns the camera.
   Middle and right still pan, and the wheel still zooms.
   - In slice mode, the hit is the slice plane, exact and cheap: `<key>.{x,y,z}` in world units,
     `<key>.{i,j,k}` in voxels, `<key>.value`, `<key>.button`, `<key>.mods`, and `<key>.phase`
     (`down`, `drag`, `up`).
   - In an orbit view, it is a VTK prop pick (`<key>.node`, the id of the node hit).
   - `on_pick:` runs an action program with those keys set, so tools can be written in the DSL.
   - This is the "click-to-world picking in a viewport" item already on the libcvc list (L0d).
     Build it once, here.
   - *Built for orbit views (L0d):* `input: pick`, `pick:` (`x`, `y`, `z`, `hit`, `node`, `button`,
     `mods`, `phase`, `view`, `seq`) and `on_pick:`. The slice mode's `{i,j,k}` and `value` come
     with the slice camera mode.
4. **Overlays that stay on top of a slice:** point glyphs, polylines and crosshairs drawn as child
   geometry nodes of the slice, with a coincident-topology offset (or a later render layer) so
   they never z-fight the textured quad. `point_size` / `line_width` already exist as override
   fields.
5. **Widgets the side panel needs** (check each against the backend before planning around it):
   a list bound to a state array with add and remove (the `repeat:` the card component uses), a
   colour picker for a class colour, radio buttons for the tool, and a tab bar or segmented
   buttons bound to a key for the view switch. Anything missing is a small ImGui widget in
   `ImGuiBackend.cpp` with an FTXUI placeholder.

Items 2 and 3 are the critical path for VolumeGridRover. Item 1 is the bulk of the rendering
work. Items 4 and 5 are small. All of them run in arigl-web too.

---

## 7. Phases

Sizes are relative: S is a PR or two, M a few, L a short series, XL a longer one. Every phase
lands with its §1.7 rows, its pycvc bindings, its DSL intrinsics and its wasm-mt build and tests,
not as follow-ups.

| Phase | Size | What lands | Needs |
|---|---|---|---|
| P0a libcvc under the app | XL | Every item of §1.3's audit (152 objects, and the `thread_local`s that keep state) moved onto `cvc::app` (or the `AppRuntime`), each subsystem its own PR with a two-apps test. In order: (1) the file I/O registries, fixing the duplicated handlers (volume first, whose API already takes the app, then geometry, model, image); (2) Ariadne's registries and cells (action intrinsics, node types and shader presets, widgets, `.ari` blocks, data decoders, URI schemes and caps, net and OIDC hosts, media player factory and policy); (3) state's startup hooks and static-init registrars (xmlrpc's auto-start is deleted with XML-RPC, §1.10), the compression registry, `state_volume_codec`'s per-thread app; (4) `cvc::net`, cvcGL's static configuration, the world generators; (5) state_exec's ids and persistent `thread_local`s, core's counters; (6) CUDA and the process-wide list (§9 item 5). The public functions that reached a process-wide table take the app (§9 item 3). The object-file scan in CI starts with the audit's list as its allowlist and ends with only §9 item 5's | — |
| P0b Shared pieces | M | `cvc::param_table` (§1.5), with its state, property, CLI, pycvc and DSL generators and its own T8 tests; `cvc::result_job<R>`, the adapter from any stepper to `sliced_job` (§1.4, §1.9); a helper that publishes §1.9's node keys from a task, and the generic `-start` / `-async` / waiting intrinsic forms plus `job.cancel` (§1.6), so each port only registers its stepper; the T7 harness (budgets 1, 64, random and 0, the slowest-step record, the cooperative frame-time run); pycvc's generic `result_job` wrapper; the `ports (wasm-mt, node)` CI lane (§1.8); `tools/volrover-goldens/` (builds the legacy libraries at a pinned volrover sha and writes the T1 goldens) | — |
| P0c Permissive libraries | M | Qhull (wasm and wasm-mt) and Clipper2 as cvcpkg packages, in cy-pca/cvcpkg#578, open; CDT (with its `static const` fix), Shewchuk's predicates and nanoflann as header-only cvcpkg packages (cy-pca/cvcpkg#597), not vendored. Each with its notice in `THIRD_PARTY_NOTICES.md`, the singleton scan clean, a smoke test natively and under node, and its first `cgal_ab_test` rows (T16) | — |
| P0d Shared types | S | `cvc::contour_stack` and `cvc::point_classes`, with readers and writers registered on the app, their parameter tables, intrinsics (`contours.*`, `point-classes.*`, `params.*`), pycvc, and T6, T8, T9, T11 and T14 | P0a, P0b |
| P1 Skeleton core | M | `cvc::tet_complex` on Qhull with its adapter, built by incremental insertion, and its first PR proving that against the batch build (§2); vendor into `src/cvc/geometry/skeleton/` with the shared cocone core and `skeleton_context`; the §2 fixes; `skeleton_stepper` with every phase sliced, `cvc::skeletonize` and its job; `cvc skeleton`; pycvc; `skeleton.compute`; §2's tests | P0a–P0c |
| P2 Skeleton in Ariadne | S | The `mesh_lab` `skeleton` request, children and `mesh_tools` section; a demo document with a golden image, native and in arigl-web | P1 |
| P3a Planar module | M | `cvc::planar`: the snapping grid, predicates, polygons, Clipper2 Booleans and offsets, CDT, the Boost.Polygon medial axis, Sibson interpolation; T2 against brute force | P0c |
| P3b Tiler core | L | Vendor ContourTiler into `src/cvc/geometry/tiler/` on `cvc::planar`, with the context-held logger; in-memory output, const input; `tile_stepper` slicing within a slice pair (§3); its 12 tests as gtest; `cvc tile`; pycvc; `tiler.tile`; §3's tests | P3a, P0d |
| P4 Tiler in Ariadne | S | `contour_lab`, reading a file or another node's contours, and `contour_tiler.ari`, with a demo on a small redistributable Reconstruct series (find one), native and in arigl-web | P3b |
| P5 Segmentation core | M | §5 steps 1–2: GenSeg vendored into `src/cvc/volume/segmentation/` with `genseg_context` and in-memory input and output; `segment_stepper` with every phase sliced (§5); voxel-for-voxel parity, then the clean-up; `cvc segment`; pycvc; `segment.seeded`; §5's tests | P0a, P0b, P0d |
| P6 Paper mode | S–M | §5 steps 3–4: the SegMed audit, `cvc::gradient_vector_diffusion` as a stepper, `seed_mode::critical_points` and `critical_point_seeds`, `segment.critical-points`, and #123's seed generators. The default-seeding switch is its own later PR (§5 step 5) | P5 |
| P7 Viewport pieces | M | §6: the `volume_slice` node, `mode: slice` with a bindable axis, `input: pick` / `pick:` / `on_pick:`, the overlay offset, the missing widgets. Each with tests, plus an orthoslicer demo next to `volren_bunny`, native and in arigl-web | P0a |
| P8 volume_grid v1 | M | Navigation, the view switch and the quad layout, probe, point classes (mark, select, move, delete, save, load), sessions, the transfer function; parameters from tables and data on the data channel; golden images and scripted-pick tests; two grids in one document (T6) | P7, P0d |
| P9 volume_grid v2 | M | Segmentation tab: `segment`, labels on the slices, class surfaces in the 3D view, `save_segments` | P8, P5 |
| P10 volume_grid v3 | M | Contours: draw and edit per object, linear and spline, export; `tile` puts surfaces in the 3D view | P8, P3b |
| P11 Wrap-up | S | Docs: ARIADNE.md §5.4 rows, INDEX.ari entries, the generated parameter tables. volrover3 picks it all up through libcvc. Retire the legacy libraries from volrover `master` once parity holds, together with #123's VolUtils leftovers | P2, P4, P9, P10 |

**Parallel tracks.**
- P0a, P0b and P0c touch different files, so they can run at once. P0d follows P0a and P0b.
- Once P0 is in, P1, P3a, P5 and P7 share no files and need nothing from each other, so they can
  run as parallel sessions. P3b follows P3a.

**Separate items afterwards:**
- SDF2D and contour curation;
- TightCocone and PocketTunnel on the shared cocone core;
- the virus segmentations (SegCapsid, SegSubunit, SegMonomer) and SecStruct, on §5's fast
  marching and gradient vector diffusion;
- the paper's §2.3 volumetric skeletons.

---

## 8. What is deliberately left behind

- **CGAL, in every form**, and with it GMP and MPFR (§1.2). The legacy builds in
  `tools/volrover-goldens/` still use it to make test data; nothing in libcvc links it.
- **The old `src/Tiling` code:** its licence, the globals, the `exit()`s and the temp files. Its
  contour format survives as a reader.
- **All of VolumeGridRover's Qt and GL:**
  - `SliceCanvas`, the three slice renderers (ARB fragment program, paletted texture, SGI colour
    table), immediate mode, `GL_SELECT` picking;
  - the `QThread` workers that read widgets off the GUI thread;
  - the `QSettings` cache directory and the temp files in the current directory.

  Its view switch, though, carries over (§4).
- **The memory-mapped readers** (`MappedVolumeFile`, `lfmap.c`), which are not built today.
  Region reads of one slice cover the same need, and a small cache of recent slices, owned by the
  node, covers scrubbing.
- **`PolyTess`:** unused, with a non-commercial licence.
- **ContourTiler's tools** (Sandbox, Decimate, ecs, smfconvert, ContourNear) and its debug output
  files.
- **log4cplus and its process-wide loggers,** and **ContourFilter's `Controls::instance()`**,
  with its smoothing, unless a dataset needs the smoothing. If one does, it is ported without the
  singleton.
- **EM clustering**, compiled out today for "serious memory problems".
- **GenSeg's XML-RPC remote mode and its file round trip.** A remote run is a `segment.seeded`
  job on an arisrv host (§1.10).
- **libcvc's XML-RPC** (§1.10): `cvc::xmlrpc`, the vendored XmlRpc++, `cvc server` / `cvc client`,
  `utility.h`'s XML-RPC helpers, and the `__system.xmlrpc` keys.
- **Skeletonization's helix-specific strand window and its `#if 0` helix block.** The window
  becomes an off-by-default parameter. The block fits better with a SecondaryStructures port.

---

## 9. Decisions for Joe

**Settled on 2026-11-08**
- ContourTiler is LGPL like libcvc, and GenSeg is in scope (§5).
- Every port has no singletons, an API pycvc wraps, its parameters and data on the state tree, DSL
  intrinsics and the §1.7 tests (§1.3–§1.7).
- No GPL code: permissive libraries replace CGAL's GPL packages, so there is no GPL library and no
  `-gpl` package (§1.2). libcvc's own GPL CGAL use is gone (transfix/libcvc#1080).
- Wasm from the first PR of every phase (§1.8).
- Everything under an app context, in all of libcvc, before the ports (§1.3, P0a).
- VolumeGridRover keeps the legacy XY / XZ / ZY switch as its default layout, with `quad` as the
  other (§4).
- `volume_grid` keeps GenSeg's local maxima as its default seeding until the paper mode matches
  the legacy results on real maps (§5).

**Settled on 2026-11-09**
- Every long routine is a resumable stepper, as the FEM solves are. It runs on the scheduler in
  slices with progress for the UI, and stays interactive on one thread. An Ariadne action starts
  one with a `-start` intrinsic under the default limits (§1.6, §1.9).
- No XML-RPC: libcvc's goes in the 4.0 break, and arisrv's transports, `room-call` and job
  service replace it (§1.10).
- The API break is one 4.0 release. Downstream it costs 8 call sites in cvcdbg and volrover3
  (§9 item 3).

**Still open**

1. **How the new libraries arrive. Settled 2026-11-09:** as cvcpkg packages, never vendored
   (Joe: "dont vendor third party libraries in the libcvc repo!! make cvcpkg packages for
   them").
   - Qhull and Clipper2 are cy-pca/cvcpkg#578.
   - The header-only CDT, Shewchuk's predicates and nanoflann are cy-pca/cvcpkg#597.
   - nanoflann's vendored copy (#1080) goes once its package is published.
2. **How close ContourTiler must match the legacy.** The medial axis replaces the straight
   skeleton, and integer coordinates replace exact rationals, so untiled regions can fill
   slightly differently (§3). **Recommended:** T1 by topology and a Hausdorff tolerance, exact
   away from untiled regions. If exact parity matters for some dataset, say which.
   - P3a's A/B (2026-11-11) shifts where the difference comes from: the tiler's live fill uses a
     convex partition, not the straight skeleton, and cvc::planar's partition has different and
     fewer pieces than CGAL's (§3, "What P3a found"). Exact parity in untiled regions would mean
     reproducing CGAL's own non-Delaunay constrained triangulation; the recommendation stands.
3. **The API break P0a needs. Settled 2026-11-09: one 4.0 release** (Joe: "Yes, one 4.0").
   - **What changes.** Public free functions that reach a process-wide table today gain an app
     parameter, and the old forms go:
     - Ariadne's registration functions (`register_scene_node_type`,
       `register_action_intrinsics`, `register_widget_type`, `register_ari_block`,
       `register_data_decoder`, `register_shader_preset`, `register_script_engine`,
       `register_player_factory`, the terrain and city generators);
     - the URI resolver (`register_uri_handler` and friends, `resolve()`, `resolve_async()`,
       `resolve_to_file()`);
     - `read_geometry(filename)`, `read_model(filename)` and `read_image(path)`;
     - `net::send(req)`;
     - the volume codec's signature, and the rest of §1.3's audit;
     - XML-RPC, removed (§1.10).

     No wrapper keeps a hidden global alive, because that would be a singleton, and a staged
     deprecation would keep the audit's three bugs alive until the old forms went.
   - **How it ships:**
     1. Tag a last 3.x on master first. Fixes needed during the series go on a `release/3.x`
        branch.
     2. P0a lands on master one subsystem per PR. Each PR adds its two-apps test (T6) and
        updates libcvc's own callers in the same change: cvcGL, ari, arigl, arisrv, the CLI,
        pycvc, the examples and the tests.
     3. Downstream repos stay on 3.x until 4.0 is tagged, then move, with a migration note per
        subsystem in the release notes.
   - **What it costs downstream,** from a search of each repo's default branch on 2026-11-09 for
     the functions above. Ariadne documents call none of them, so `.ari` files are unaffected.

     | Repo (head) | Changes for 4.0 |
     |---|---|
     | CVC-DBG/cvcdbg (`097af43`) | 3 `read_model` and 2 `read_image` calls (`demos/nav_common.cpp`, `demos/dbg_austin_live3.cpp`, `src/cvc/dbg/scene_ingest.cpp`) |
     | transfix/volrover, `volrover3/` (`1cf0fbd`) | 3 `read_geometry` calls (`volrover3/MainWindow.cpp`). The legacy VolumeRover2 tree has its own XmlRpc++ copy, not libcvc's |
     | transfix/F2Dock (`20e9f1d`) | `F2DockServer` builds on libcvc's XML-RPC. It is deleted by the TexMol plan's F3, before or with 4.0 |
     | transfix/TexMol (`e621ebc`) | none: `F2DockClient` uses TexMol's own XmlRpc++ copy, and is retired |
     | cy-pca/nicotina (`32c2ea0`), cy-pca/cypkg (`baf52c6`), cy-pca/cyberpcangel-cvcgl (`e7c9efc`), CVC-Lab/GRL-SNAM (`9e0ea8b`) | none |

     So the break costs 8 call sites in 2 downstream repos, plus F2DockServer's removal. Nearly
     all the work is inside libcvc: about 105 calls to the Ariadne registration functions
     across 49 files (most in tests and examples), and the about 66 `resolve()` call sites in
     Ariadne and cvcGL (§1.3).
4. **Names.**
   - Node types: `volume_grid`, `contour_lab`, `volume_slice`.
   - Functions: `cvc::skeletonize`, `cvc::tile_contours`, `cvc::segment_seeded`,
     `cvc::critical_point_seeds`.
   - Modules: `cvc::tet_complex`, `cvc::planar`, `cvc::cocone`.
   - Commands: `cvc skeleton`, `cvc tile`, `cvc segment`.
   - The intrinsics in §1.6.
5. **What may stay process-wide.** Some state belongs to the process, not to libcvc, so no app can
   own it:
   - signal dispositions and process priority;
   - the environment and working directory;
   - SDL's single event queue and its window and timer routes;
   - the X11 clipboard connection;
   - curl's, ImageMagick's, Jolt's, HDF5's and gRPC's global initialisation and locks;
   - CUDA's per-device constant tables and memory.

   Separately, some `thread_local`s keep nothing between calls: scratch buffers and re-entrancy
   guards.

   **Recommended:**
   - A short documented list, the scan's only allowlist, of exactly these process resources, each
     wrapped so that libcvc keeps nothing of its own about it beyond a reference count or a lock.
   - Per-thread scratch and scope-bound guards are allowed. Every other `thread_local` moves.
   - The CUDA volume cache becomes per device, owned by whichever object owns the device (not
     duplicated per app), and it is on the list.
   - Writes to the environment or the working directory go wherever an explicit argument can
     replace them.
