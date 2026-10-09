# libigl for Geometry Processing and Finite Elements — Roadmap

**Status:** phase 1 landed and verified, 2026-10-09; phases 2–4 are planned. libcvc: branch
`feat/libigl-geometry` (based on `origin/master @ 188489b5`). cvcpkg: the recipes are in
[cy-pca/cvcpkg#145](https://github.com/cy-pca/cvcpkg/pull/145) (branch `recipes/eigen-libigl`);
publishing the bundles to cvcpkg.org was approved on 2026-10-09 and follows that merge (§7).
**Pins:** Eigen 5.0.1 + libigl v2.6.4, libigl **core module only**, header-only, MPL-2.0.
**Public API:** [`inc/cvc/geometry/mesh_ops.h`](../../inc/cvc/geometry/mesh_ops.h) and
[`inc/cvc/geometry/fem.h`](../../inc/cvc/geometry/fem.h); reference in
[GEOMETRY_API.md — Mesh Operations and Finite Elements](../GEOMETRY_API.md#mesh-operations-and-finite-elements-libigl).
**Companions:** [ARIADNE.md](../ARIADNE.md) (the `mesh_lab` / `fe_lab` node types),
[WATER-RENDERING-ROADMAP.md](WATER-RENDERING-ROADMAP.md) D-W6,
[VISIBILITY-AND-LOD-ROADMAP.md](VISIBILITY-AND-LOD-ROADMAP.md) layer (c), and the volrover3 roadmap
(`cvc-engagement-docs/modernization/2026-08-11-volrover3-roadmap.md`) §20.15 and §26.

Status words used below: **landed** means in the working tree of these branches now, built and
tested (§2.8 lists what was run); it is not merged or published yet. **Planned** means a later
phase.

---

## 0. Executive summary

### The question

> *"How might we leverage libigl to upgrade our geometry processing and finite element work in
> the libcvc world?"*

### The answer

Use libigl's MPL-2.0 core, header-only, behind an Eigen-free libcvc API, for the three things
libcvc does badly or not at all today:

1. **Exact spatial queries:** closest point, ray hit, signed distance, generalized winding number
   and point-in-tet.
2. **Differential geometry on surfaces:** normals, orientation, curvature, implicit smoothing and
   geodesics.
3. **Linear finite elements** on the tetrahedral meshes the LBIE mesher already produces.

Keep libcvc's own QEM simplifier and the LBIE mesher, which are better than anything in libigl's
permissive core. Never enable libigl's copyleft or restricted modules.

Four facts decided it (libcvc at `188489b5`, before this change):

- **There is no FE consumer anywhere in the workspace.** `cvc::tetrahedralize()` and friends
  produce tets, but nothing assembles or solves on them. pycvc `%ignore`s `tets()`, `functions()`
  and `curvatures()` (`bindings/pycvc/pycvc.i:481-498`). cvcGL draws tets as edges under
  `// TODO: Implement tetrahedral mesh rendering` (`src/cvcGL/GeometryNode.cpp:1076-1094`). No
  compiled code fills `geometry::functions()` or `curvatures()`. libigl's `cotmatrix`,
  `massmatrix`, `grad` and `min_quad_with_fixed` on tets are a few hundred lines away from a
  working P1 layer.
- **Several existing routines are wrong, not just slow** (§3). `project()` clamps the edge
  parameter to 1.0 every time, `reorient()` never changes winding, one smoothing pass ignores
  `delta`, tet orientation is lost on decode, `hex_faces` emits bow-tie quads, and point location
  in meshes above 10,000 tets rebuilds a CGAL tree on every call.
- **libigl's exact replacements also work where CGAL is switched off** (wasm builds use
  `DISABLE_CGAL=ON`), and they shrink the set of GPL-3.0 CGAL headers compiled into an
  LGPL-2.1-only library (§6).
- **The cost is contained.** The templates are compiled only in private translation units, so no
  Eigen type reaches installed headers, SWIG, cvcGL or downstream CMake configs. The known traps
  (Windows export table, CGAL's `FindEigen3`, thread pools, the Apache-2.0 `BFloat16.h` and the
  COLAMD/SuperLU notices) each have a mitigation or a follow-up in §2.2, or an owner decision in
  §6.

### Recommendations, in order

1. Commit phase 1 (§2, landed and verified in one integration build) and open the PRs: libcvc
   `feat/libigl-geometry`, and the cvcpkg recipes against cy-pca/cvcpkg.
2. Publish the `eigen` and `libigl` bundles (§7, a production publish that needs a go-ahead).
   Then add them to the libcvc recipe and CI, with a guard that the feature actually resolved ON
   (P2.1).
3. Fix the tet data path at its source before trusting any FE number (P2.2).
4. Expose the new API through pycvc (P2.3). For GRL-SNAM it is the only route, because the PyPI
   `libigl` wheel requires scipy.
5. Then do node-aware picking, the wasm rollout, CGAL reduction, the volrover3 §26 backend and
   deformation (phases 3 and 4).
6. Keep FE scoped to scalar PDEs until someone owns an elasticity validation case (P4.4).

---

## 1. Decisions already taken

| Decision | Choice | Why (evidence) |
|---|---|---|
| Versions | Eigen **5.0.1**, libigl **v2.6.4** | v2.6.4 is the latest libigl (2026-09-15) and pins Eigen 5.0.1 since v2.6.3 (`cmake/recipes/external/eigen.cmake`). VTK's bundled Eigen 3.4.0 is renamed (`#define Eigen vtkeigen`, `deps/include/vtk-9.5/vtkeigen/eigen/Core:14`), so there is no link-level clash and no reason to pin 3.4.0. 3.4.0 also has the emscripten SIMD bug (Eigen#2514). |
| libigl packaging | Header-only core, full top-level `include/igl` tree | Static mode instantiates only double/int types: 0 of 2845 core instantiations take `Eigen::Map`, and libcvc indices are `uint64_t`. A static `igl.lib` is also single-config Release (LNK2038 for Debug consumers). |
| Installed names | `igl::core` from `<prefix>/lib/cmake/libigl/libigl-config.cmake`; `Eigen3::Eigen` from `<prefix>/share/eigen3/cmake` | Upstream installs `igl::igl_core` into `lib/cmake/igl`. Both are fixed in the recipe (§2.1). |
| Modules | Core only | Copyleft (CGAL GPL-3+, TetGen AGPL-3+) and restricted (Triangle non-commercial, MATLAB, MOSEK) modules are incompatible with LGPL-2.1-only distribution (§6). |
| libcvc option | `CVC_ENABLE_LIBIGL`, default ON, auto-OFF when the packages are missing; PUBLIC compile definition `CVC_ENABLE_LIBIGL` | libcvc names options `CVC_ENABLE_*`; `CVC_WITH_*` has 0 hits in the tree. |
| Header boundary | Eigen/libigl are included **only** from `src/cvc/geometry/igl/*.cpp` | Never from `inc/`, cvcGL, Ariadne, tests, or the Eigen-free `mesh_ops.cpp` / `fem.cpp`. This keeps SWIG, the SDK and `cvcConfig.cmake` Eigen-free and gives one libigl thread pool per process. |
| Where igl lives | libcvc core, not cvcGL | cvcGL is a static archive by default and would export the dependency (the X11 trap, `src/cvcGL/CMakeLists.txt:206-231`). cvcGL, Ariadne and pycvc call the core API. |

---

## 2. Phase 1 — what this change delivers *(landed)*

### 2.1 cvcpkg recipes: `eigen` 5.0.1 and `libigl` 2.6.4 *(in review: cy-pca/cvcpkg#145)*

Two new main-set recipes, in [cy-pca/cvcpkg#145](https://github.com/cy-pca/cvcpkg/pull/145).
Both were built and verified locally: Windows native, Windows-hosted wasm (with a node smoke
run), and WSL Ubuntu 24.04 native. Until the publish runbook (§7) has run, the catalog has no
`eigen`, `eigen3` or `libigl` package, so libcvc CI builds with `CVC_ENABLE_LIBIGL` auto-OFF.

**eigen.** Tarball `https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz`,
sha256 `e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec` (Homebrew and
conda-forge agree). It uses Eigen's own CMake install with these switches:

- Off: `BUILD_TESTING`, `EIGEN_BUILD_TESTING`, `EIGEN_BUILD_BLAS`, `EIGEN_BUILD_LAPACK`,
  `EIGEN_BUILD_DOC`, `EIGEN_BUILD_DEMOS`, `EIGEN_BUILD_BTL`, `EIGEN_BUILD_SPBENCH`. As a top-level
  project Eigen otherwise builds BLAS/LAPACK, tests, docs and demos.
- `-DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF`. Eigen 5.0.1 forces CMP0090 NEW and then calls
  `export(PACKAGE Eigen3)` (`CMakeLists.txt:13-21, 263`). Without the flag every build writes a
  user package-registry entry (HKCU on sandipaws, `~/.cmake/packages` on the fleet) that points at
  a temporary build tree. `CMAKE_EXPORT_NO_PACKAGE_REGISTRY` is ignored under CMP0090 NEW.
- `-DEIGEN_PRERELEASE_VERSION=`, so the installed `Eigen/Version` says `5.0.1`, not `5.0.1-dev`.

`depends.runtime: []` must be explicit, otherwise cmake and ninja leak into `required_deps`
(`tests/unit/test_required_deps_no_host_tools.py:88`). Eigen 5's version file **rejects
`find_package(Eigen3 3.x)`**, so consumers use a version-less `find_package(Eigen3 CONFIG)`.

**libigl.** Tarball `https://github.com/libigl/libigl/archive/refs/tags/v2.6.4.tar.gz`, sha256
`09fa1b9b44e0ecbd8fbaa17ec8d9ac0cb4efcedcb2427970f8a4e90bc96e0970` (same hash on two downloads).
`eigen` is a **runtime** dependency, because the installed config calls
`find_dependency(Eigen3)` and the cross helpers search only `CVC_DEPS_PREFIX`. The build works
around five upstream install bugs:

| Upstream behaviour | Recipe fix |
|---|---|
| `eigen.cmake` never calls `find_package`; it FetchContents Eigen unless `Eigen3::Eigen` already exists | `-DCMAKE_PROJECT_libigl_INCLUDE=<file containing find_package(Eigen3 CONFIG REQUIRED)>` plus `-DFETCHCONTENT_FULLY_DISCONNECTED=ON` |
| Every `LIBIGL_*` module defaults ON in a top-level build | Every module, copyleft and restricted option explicitly OFF; `-DLIBIGL_USE_STATIC_LIBRARY=OFF`; `CMAKE_DISABLE_FIND_PACKAGE_{Matlab,MOSEK,BLAS}=ON` |
| Export name is `igl::igl_core`, because `igl_install.cmake:19` reads the undefined `${module_export}` | `-Dmodule_export=core`, then assert that `LibiglConfigTargets.cmake` contains `add_library(igl::core ` |
| Config is in `lib/cmake/igl` (not on CMake's search path); the version file is misnamed and says 2.5.0 | Move to `lib/cmake/libigl`; generate `libigl-config-version.cmake` from `CVC_VERSION` |
| A header-only install skips `raytri.c`, `IO` and the five `Singular_Value_Decomposition_*.hpp`, which breaks 35 core headers including `AABB.h` and `signed_distance.h` | Copy every **top-level** file of `include/igl` after install. Never copy the module subdirectories (`copyleft/`, `triangle/`, `matlab/`, `mosek/`, `embree/`, `opengl/`, ...). Assert that `include/igl/raytri.c` exists and that no `include/Eigen` leaked in. |

`LIBIGL_PARALLEL_FOR_BACKEND` is `SERIAL` for `wasm` (its `igl::core` then exports
`IGL_PARALLEL_FOR_FORCE_SERIAL`) and `POOL` elsewhere, including `wasm-mt`. In header-only mode a
consumer could still force serial by defining `IGL_PARALLEL_FOR_FORCE_SERIAL` itself, but in a
static link shared with other `igl::core` users that is an ODR violation (P3.2), so libcvc follows
the bundle's backend. A SERIAL bundle cannot be undone.

The draft matrices on the cvcpkg branch:
- eigen: linux, macos, windows, freebsd, openbsd, netbsd, wasm (linux- and windows-hosted),
  wasm-mt, wasi and cosmo.
- libigl: the same minus wasi and cosmo. WASI is excluded because `parallel_for.h` includes
  `<thread>` unconditionally.
- The libigl matrix must be a subset of eigen's (closure rule in
  `scripts/validate_all_recipes.py`).

Both bundles ship their licence texts in `share/licenses/<name>/`.

The smoke test builds a consumer that links `igl::core` and exercises `cotmatrix`/`massmatrix`
plus an `AABB`/`signed_distance` case, which covers the missing-file bug. On Windows the smoke
runs **inside `build.ps1` while vcvars is active**, because `test.sh` exits 0 when `cl` is not on
PATH.

### 2.2 libcvc build integration: `CVC_ENABLE_LIBIGL` and the `cvc_igl` helper *(landed)*

The find, condensed from `src/cvc/CMakeLists.txt` (the file is authoritative):

```cmake
option(CVC_ENABLE_LIBIGL "libigl/Eigen geometry processing and P1 finite elements ..." ON)
if(CVC_ENABLE_LIBIGL)
  set(_cvc_prefer_config "${CMAKE_FIND_PACKAGE_PREFER_CONFIG}")
  set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)   # CGAL's FindEigen3 must not win
  find_package(Eigen3 CONFIG QUIET)          # no version: Eigen 5 rejects 3.x requests
  find_package(libigl CONFIG QUIET)
  set(CMAKE_FIND_PACKAGE_PREFER_CONFIG "${_cvc_prefer_config}")
  if(TARGET Eigen3::Eigen AND TARGET igl::core)
    message(STATUS "libigl ${libigl_VERSION} + Eigen ${Eigen3_VERSION} found - mesh_ops/fem ENABLED")
  else()
    message(STATUS "libigl/Eigen not found - mesh_ops/fem compile to stubs (CVC_ENABLE_LIBIGL OFF)")
    set(CVC_ENABLE_LIBIGL OFF)
  endif()
endif()
```

The rules behind it:

- **The CGAL `FindEigen3` trap.** `find_package(CGAL CONFIG)` (`src/cvc/CMakeLists.txt:527`)
  appends CGAL's module directory to `CMAKE_MODULE_PATH`. Its `FindEigen3.cmake` parses
  `Eigen/src/Core/util/Macros.h` for a version, but Eigen 5 moved the version to `Eigen/Version`.
  libigl's config calls `find_dependency(Eigen3 REQUIRED)` with no `CONFIG`, so module mode wins
  and configure **fails hard, even under `QUIET`**. Preferring config fixes it. Any downstream that
  finds `cvc` first (cvcGL standalone, pycvc) re-finds CGAL (`CMake/cvcConfig.cmake.in:16-18`)
  and needs the same guard if it ever finds libigl itself.
- **Windows exports.** `cvc` relies on `WINDOWS_EXPORT_ALL_SYMBOLS` (about 34,905 symbols
  before this change, against the 65,535 PE limit). When `cvc` is a shared library the four igl
  translation units (`src/cvc/geometry/igl/*.cpp`) go into a private STATIC helper, `cvc_igl`,
  linked `$<BUILD_INTERFACE:cvc_igl>`. The export scan covers only cvc's own objects, so the
  template instantiations stay out of the `.def` file. `cvc_igl` is PIC with hidden visibility
  (inline functions included), links only `PRIVATE igl::core Eigen3::Eigen`, and inherits only the
  directory-scope defaults (C++ standard, MSVC runtime, directory-wide definitions such as the
  mesher block's `NDEBUG`), never `cvc`'s target include directories or definitions: its TUs
  include no cvc header. It is neither installed nor exported. Measured on the integration build:
  `cvc.dll` exports 47,084 symbols, against 46,836 for the same tree without the new objects,
  so the whole feature adds 248.
  **Every public `cvc::` function body must therefore live in a cvc-owned translation unit**
  (`mesh_ops.cpp`, `fem.cpp`, Eigen-free) that calls `cvc::igl_detail::*` in the helper
  (`src/cvc/geometry/igl/igl_detail.h`). A public function defined inside the helper would not be
  exported, and pycvc/cvcGL would fail with LNK2019. Static `cvc` builds (the wasm builds among
  them) compile the same igl TUs into `cvc` itself, so their objects ship inside `libcvc.a`. The
  choice follows `cvc`'s actual target type, not `BUILD_SHARED_LIBS`, which no wasm configure
  turns off. Both packages are header-only, so neither reaches `cvcTargets.cmake`.
- **The bridge copies, it does not `Map`.** `igl::AABB<DerivedV,3>` and `signed_distance`
  instantiate on the exact `DerivedV` type. V is copied to `Eigen::MatrixXd` and F/T to
  `Eigen::MatrixXi`, with an `INT_MAX` check (the uint64 → int narrowing is unavoidable anyway).
  Reads go through `const_points()` / `const_tris()` / `const_tets()`, because the non-const
  accessors copy-on-write detach and reset the cached extents (`geometry.h:138-189`).
- **One thread pool, not capped.** libigl 2.6.4's POOL backend (`parallel_for.h`) is a
  heap-allocated singleton that is never deleted. It is created lazily, on the first loop that
  actually runs in parallel, and sized by `igl::default_num_threads()`: a Meyers singleton that
  reads the `IGL_NUM_THREADS` environment variable **once**, falls back to
  `std::thread::hardware_concurrency()`, and keeps whatever the first call decided (a nonzero
  argument to that first call wins over both). libcvc never calls it with an argument, so the
  pool has `IGL_NUM_THREADS` workers if the variable is set before the first igl call, and
  hardware-concurrency workers otherwise. `IGL_NUM_THREADS=1` makes every loop serial. The pool
  runs beside OpenMP (linked PUBLIC) and `cvc::thread_pool`, so heavy igl work alongside them
  can oversubscribe the CPU. Nested `parallel_for` calls run serially, and keeping igl out of
  cvcGL keeps it to one pool per process on Windows. **Follow-up:** a cap set by libcvc, for
  example a `std::call_once` in the `igl_detail` entry points that calls
  `igl::default_num_threads(N)` with N from a libcvc setting.
- **Threads on wasm.** On single-threaded wasm (no `-pthread`) the igl TUs compile with
  `IGL_PARALLEL_FOR_FORCE_SERIAL`, which matches the SERIAL wasm `libigl` bundle (its `igl::core`
  exports the same define). With pthreads (wasm-mt) they keep `igl::core`'s backend, POOL, so the
  same pool rules apply; see P3.2 for what that means on the browser main thread.
- **Hygiene.** Wrap the unguarded `#define NOMINMAX` (`inc/cvc/core/config.h.cmake:22`) in
  `#ifndef`, because `igl::core` exports `-DNOMINMAX` on MSVC (C4005 noise, not an error). The
  mesher block defines `NDEBUG` for the whole directory (`add_compile_definitions` in
  `src/cvc/CMakeLists.txt`), so Eigen asserts are off even in Debug, and tests must use
  `EXPECT_*`/`ASSERT_*`. `EIGEN_MPL2_ONLY` does nothing in Eigen 5.0.1 (only its CHANGELOG names
  it) and is no MPL-only guarantee: `Eigen/Core` still includes the Apache-2.0 `BFloat16.h`, and
  the sparse solvers bring in COLAMD and SuperLU code (§6). Nothing in libcvc relies on it.

### 2.3 Public API: `mesh_ops.h` *(landed)*

[`inc/cvc/geometry/mesh_ops.h`](../../inc/cvc/geometry/mesh_ops.h) has no Eigen or libigl
types. Every function is always declared. Without `CVC_ENABLE_LIBIGL` each one throws
`cvc::mesh_ops_unavailable`, except the colormap functions (`colormap_rgb`, `robust_range`,
`colormap_from_string`, `to_string`). `mesh_ops_available()` reports which build you have. Its
conventions:

- Quads are split as (a,b,c)+(a,c,d), and "triangle index" spans `tris()` and then the split quads.
- Tets of either orientation are accepted.
- Indices are validated (`cvc::mesh_ops_error`).
- Lengths are in the mesh's own unit.

| Area | API | libigl underneath |
|---|---|---|
| Normals, orientation | `compute_vertex_normals(g, normal_weighting::{UNIFORM,AREA,ANGLE})`; `orient_outward(g)` re-winds triangles per component and returns the flip count | `per_vertex_normals`; `bfs_orient` + `orient_outward` |
| Repair | `repair(g, repair_params)` → `repair_report`: weld, drop degenerate/unreferenced elements, orient; remaps aligned per-vertex arrays and `lines/tets/hexs` | Weld: `igl_detail::weld_vertices` in `igl_surface.cpp` (merges vertices closer than `weld_epsilon`). Degenerate-face removal and unreferenced-vertex compaction are cvc's own code in `mesh_ops.cpp`, not `igl::remove_unreferenced`. Orient: as above |
| Curvature | `compute_curvature(g)` fills `curvatures()` with (k1, k2); `vertex_curvature(g, MEAN/GAUSSIAN/MAX_PRINCIPAL/MIN_PRINCIPAL)`. Sign: a sphere of radius r gives H ≈ 1/r. | `principal_curvature`, `gaussian_curvature`, `cotmatrix` + `massmatrix` |
| Smoothing | `smooth(g, smooth_params)` in double precision: `COTAN_IMPLICIT` (scale-independent step t = λ·d², optional area/centroid preservation), `UNIFORM_LAPLACIAN`, `TAUBIN`; optional fixed boundary | `cotmatrix` + `min_quad_with_fixed` (implicit step) |
| Spatial queries | `mesh_locator` (built once, immutable, thread-safe queries): `closest_points`, `intersect_ray`, `signed_distance` (exact distance, fast-winding-number sign, robust to holes), `winding_number`; one-shot `closest_points()` / `signed_distance()` | `AABB`, `fast_winding_number` |
| Geodesics | `geodesic_solver` (heat method, cached factorizations), `geodesic_distance()` | `heat_geodesics_precompute/solve` |
| Tet meshes | `tet_volumes`, `orient_tets` (flip negative tets in place), `tet_boundary_surface` (outward, keeps all points so per-vertex fields map 1:1), `tet_isosurface`, `slice_tets` (filled planar cut with interpolated fields) | `volume`, `boundary_facets`, `marching_tets` (libigl has no `slice_tets.h`; the cut is `marching_tets` on a plane function) |
| Colour | `colormap_rgb(values, colormap_kind, lo, hi)` → packed RGB8; `robust_range` (percentiles); 8 maps | `colormap` tables when available; otherwise 17-point samples of the same tables in `mesh_ops.cpp`. `JET` (analytic MATLAB jet) and `GRAY` are cvc's own in every build |

### 2.4 Public API: `fem.h` *(landed)*

[`inc/cvc/geometry/fem.h`](../../inc/cvc/geometry/fem.h) provides linear (P1) elements on
`tets()` (`domain::VOLUME`) or the triangle surface (`domain::SURFACE`); `AUTO` picks VOLUME when
tets exist. Degenerate elements are dropped from assembly (`fem.h` states the test), and fields
are per-vertex `std::vector<double>`.

- Operators: `stiffness_matrix` (K = −L, symmetric PSD), `mass_matrix` (lumped or consistent),
  `gradient` (per-element), `boundary_vertices`. Matrices are returned in triplet form
  (`fem::sparse_matrix`).
- Solves:
  - `solve_poisson` (K u = M f with a `fem::dirichlet`; an empty boundary condition throws
    because the problem is singular);
  - `solve_laplace` (harmonic interpolation);
  - `solve_heat` (backward Euler, one factorization reused, a progress/cancel callback, an empty
    boundary condition means insulated);
  - `laplacian_eigenmodes` (k smallest pairs of K x = λ M x, intended for k ≤ ~20). It does
    **not** use `igl::eigs`: in a probe on a sphere (10,242 vertices, k = 10) that returned
    repeated, non-orthogonal modes on the degenerate spectrum, reported "Failed to converge" and
    took 11.6 s. libcvc instead solves the dense problem up to 400 active vertices and runs a
    shift-invert subspace iteration on Eigen's `SimplicialLDLT` above that (1.8 s on the same
    case).
- **Scope: scalar PDEs only.** libigl has no linear-elastic or neo-Hookean assembly (P4.4).

### 2.5 `SDF_IGL` and an exact `project()` *(landed)*

- **`SDF_IGL = 2`** is appended to `cvc::sdf_algorithm` (`inc/cvc/core/types.h:115-118`) and to
  pycvc's mirror (`bindings/pycvc/pycvc_algorithm.h:35`). It samples the volume's own node grid
  (index `i + j*X + k*X*Y`, spacing `(max-min)/(dim-1)`) against `tris()` plus the split quads. It
  needs no power-of-two size, and the sign comes from the fast winding number. The existing
  backends lose sign on damaged meshes: up to 17.2 % (v1) and 5.18 % (v2) wrong-sign voxels at 90 %
  triangle removal (`docs/SDF_LIBRARY.md:683-705`). Cost is O(N_samples · log F), so a 256³ grid
  is about 16.7 M AABB queries, run on libigl's thread pool, which is not capped (§2.2).
  A default-constructed bounding box means the geometry's extents for `SDF_IGL` only; `SDF_V1`
  and `SDF_V2` need an explicit box. `SDF_V1` stays the API default and `SDF_V2` the demos'.
  `cvc sdf -a igl` and `cvc bunny --volume -a igl` select it in cvc-cli. Ariadne exposes it as
  `source: { sdf: { algorithm: v1 | v2 | igl } }`, which falls back to `v2` with a warning on a
  build without libigl (see [ARIADNE.md](../ARIADNE.md)).
- **`geometry::project()`** moves every point to its exact closest point on
  `input.tri_surface()` through `mesh_locator` when `CVC_ENABLE_LIBIGL` is ON, so projection now
  works on `DISABLE_CGAL` builds that have libigl, wasm included. If the reference has no
  triangles nothing moves. Without libigl it falls back to the CGAL heuristic when CGAL is found
  (`CVC_GEOMETRY_ENABLE_PROJECT`), with the defects in §3, and is a silent no-op when neither is
  present.

### 2.6 cvcGL: scalar fields and TETS boundary rendering *(landed)*

Before this change `GeometryNode` coloured vertices only with literal RGB from `colors()`
(`GeometryNode.cpp:1141-1174` at `188489b5`), never from `functions()` or `curvatures()`, and
demos hand-rolled ramps (`terrain_lab.cpp:230-236`). What landed, in
`inc/cvc/gl/GeometryNode.h`:

- **Scalar fields.**
  - The API is `setScalarField(perVertex, colormap_kind, lo, hi)`, `setScalarRange`,
    `setColorMap`, `clearScalarField`, `hasScalarField` and `scalarRange`.
  - Colours are baked on the CPU through `cvc::colormap_rgb` into the same 3-channel uchar array
    `colors()` uses, so there is no lookup table. That keeps WebGL output identical; LUT-mapped
    float scalars are untested on the low-memory mapper. The colours are re-applied across
    `render_mode` and `use_single_color` rebuilds.
  - State keys bind the field from Ariadne and ImGui with no glue code:
    - `color_by`: `none` | `colors` | `function` | `k1` | `k2` | `mean` | `gaussian`. The
      curvature kinds compute `curvatures()` once with `cvc::compute_curvature` when the geometry
      has none and libcvc has libigl.
    - `colormap`.
    - `scalar_min` / `scalar_max`: empty means auto, which is a robust percentile range for the
      curvature kinds.
  - LOD caveat: `simplify_progressive` carries only uvs, colours and normals
    (`simplify.cpp:1297-1299, 1639-1641`), so rungs ≥ 1 need the field recomputed per rung or
    transferred by closest point.
- **TETS mode.**
  - Draws the tets' boundary surface (`tet_boundary_surface()`), falling back to the tet edges
    without libigl. Because that surface keeps all points, a per-vertex FE field colours it
    directly.
  - Interior fields are shown by a separate cut geometry from `cvc::slice_tets()`, which the
    `fe_lab` node builds (§2.7). That is the only portable cut on WebGL: `maxClipPlanes()` returns 0
    under the low-memory mapper (`GeometryNode.cpp:124-131`).
  - HEXS mode stays a wireframe; libigl has no hex support.

### 2.7 Ariadne: `mesh_lab` / `fe_lab` nodes and `mesh_tools` / `fe_controls` *(landed)*

The design copies the forest_trees idiom (seed keys from the node's props, poll them in
`custom_ticks`, compare a signature) and keeps the keys node-scoped like volren:

- **Node types** `mesh_lab` and `fe_lab` (`src/cvcGL/ariadne/mesh_nodes.cpp`) are registered from
  `register_cvcgl_extensions`, in `CVC_ENABLE_LIBIGL` builds only. Their keys live at
  `<prefix>.graphics.root.children.<id>.mesh_ops.*` and `….fe.*`.
- **Commands** use a request key. A button program writes `mesh_ops.request = "smooth"` (or
  `fe.request = "mesh" | "solve" | "stop"`). The tick consumes the key, runs the kernel on the
  node's `async_lane` (off the render thread, with no state or VTK access), applies the result on
  the render thread, and publishes `busy`, `status` and `stats.*`. Geometry results go through
  `setGeometry`. Colour-only results (curvature, geodesic distance, an FE field) go through
  `GeometryNode::setScalarField` / `setColorMap` / `clearScalarField`, the CPU-baked colormap of
  §2.6, after `setUseSingleColor(false)`; `updateColors` is not used. Polling in the tick, rather
  than a synchronous `state_object` handler, keeps the work out of the VTK render pass, because
  widget commits happen inside the render walk.
- **Components** are `src/cvc/ariadne/components/mesh_tools.ari` (bound to a `mesh_lab` node named
  `mesh`) and `fe_controls.ari` (bound to an `fe_lab` node named `domain`). Both are installed and
  embedded for wasm by glob, with no CMake list edit.
- **`fe_lab`** tet-meshes its volume with the in-tree LGPL `cvc::tetrahedralize` (LBIE), never
  `igl::copyleft::tetgen`, then calls `cvc::fem` and shows the field on the boundary surface plus
  a movable slice. LBIE meshes `{v > isovalue}`, so for an SDF source (negative inside) `fe_lab`
  meshes `-v` above `-isovalue` by default (`mesh.inside: below`); that is what puts the tets
  inside the object (open question 8). Its `heat.kappa` is relative: the diffusivity passed to
  `cvc::fem::solve_heat` is κ·extent², so one setting behaves the same at any mesh scale.
- **Host gaps found:**
  - `AriRuntime` never called `register_cvcgl_extensions`, so pycvc-hosted documents could not
    realize custom node types. This change adds the call.
  - `AppRuntime::ensure_intrinsics` snapshots the verb lists once (`app_runtime.cpp:51-60`), so
    verbs registered after the first are unreachable. This is not addressed here; it is needed
    before an optional `(mesh-run NODE OP)` verb.
  - Under FTXUI the panels are read-only, because that backend never commits edits or fires
    buttons.
  - See [ARIADNE.md](../ARIADNE.md).

### 2.8 Tests and verification *(landed)*

- **Core gtests** (`src/cvc/tests`): `mesh_ops_test` (suite `MeshOps`, 30 tests at the
  integration build) and `fem_test` (suite `Fem`, 14). Both executables are added, with their
  `TEST_TARGETS` entries, inside `if(CVC_ENABLE_LIBIGL)` in `src/cvc/tests/CMakeLists.txt`;
  linking and `cvc_discover_tests` sit behind `if(TARGET …)` guards. They cover, among others:
  - sphere curvature (H ≈ 1/r, K ≈ 1/r²);
  - exact closest point and signed distance against an analytic sphere, and `SDF_IGL` against
    the analytic sphere SDF;
  - `orient_outward` on a flipped mesh;
  - tet volume positivity after `orient_tets`;
  - boundary-surface orientation;
  - Poisson on a unit cube against the analytic solution;
  - heat decay;
  - Laplacian eigenmodes on a 2,562-vertex sphere (the iterative path).

  They exist only in `CVC_ENABLE_LIBIGL` builds and assert `mesh_ops_available()`, so the
  `mesh_ops_unavailable` stubs are **not** unit-tested; the cvcGL tests' no-libigl branches
  touch them only indirectly. **Follow-up:** an always-built test that, when
  `!mesh_ops_available()`, checks that e.g. `compute_vertex_normals` throws
  `cvc::mesh_ops_unavailable` while `colormap_rgb` and `to_string(colormap_kind)` still work.
- **cvcGL tests** are standalone `check()` executables: `cvcgl_scalar_field`, `cvcgl_tet_surface`,
  and the extended `cvcgl_ariadne_realize`. The headless ones go on the `REMOVE_ITEM` list that
  exempts them from the GL resource lock. `cvcgl_ariadne_realize` drives the real node types:
  `mesh_lab` decimates the bunny from 69,473 to 2,000 faces and colours it by Gaussian curvature
  and by geodesic distance; `fe_lab` tet-meshes the bunny SDF (5,490 tets) and runs Poisson and
  heat.
- **Integration build** (Windows, MSVC, Release, shared, `build-libcvc-igl`): the full suite,
  3,888 tests, passes except two cvc-cli tests that need ImageMagick, which that configure has
  off (environmental). `cvc.dll` exports 47,084 symbols against 46,836 without the new objects
  (+248), so `cvc_igl` keeps the template instantiations out of the export table.
- **Not yet pinned by a test:** which side of the isovalue LBIE meshes (open question 8). It was
  checked by hand with the built CLI; a gtest (a sphere SDF, then `tetrahedralize(-sdf, 0)`, then
  every tet centroid inside) is a follow-up.

### 2.9 Phase 1 status

| Item | Status |
|---|---|
| `mesh_ops.h`, `fem.h` public API | landed |
| This roadmap, the GEOMETRY_API.md section, THIRD_PARTY_NOTICES entries, the WATER D-W6 note | landed |
| cvcpkg `eigen` / `libigl` recipes, built and verified on Windows native, Windows-hosted wasm (node smoke) and WSL Ubuntu 24.04 native | in review ([cy-pca/cvcpkg#145](https://github.com/cy-pca/cvcpkg/pull/145)); publish approved 2026-10-09 |
| `CVC_ENABLE_LIBIGL`, `cvc_igl` helper, `src/cvc/geometry/igl/*.cpp`, `mesh_ops.cpp` / `fem.cpp` | landed (libcvc `feat/libigl-geometry`) |
| `SDF_IGL` (API, pycvc enum, cvc-cli `-a igl`), exact `project()` | landed |
| GeometryNode scalar fields, TETS boundary surface | landed |
| Ariadne `mesh_lab` / `fe_lab` (including the slice), `mesh_tools` / `fe_controls`, `mesh_lab.ari` demo | landed |
| libigl thread-pool cap | planned (§2.2) |
| MEDIT `.mesh` / Gmsh `.msh` I/O handlers | planned (P2.5) |
| Bundles published on cvcpkg.org; libcvc recipe + CI wired | planned (P2.1, §7) |

---

## 3. Current-state audit (libcvc at `188489b5`)

Nothing in libcvc used Eigen or libigl before this change. The only "eigen" code is a hand-rolled
3×3 solver in `SDF/UsefulMath/LinearAlgebra.cpp`, which is not compiled
(`src/cvc/CMakeLists.txt:770`).

| Routine | Where | Defect | Replacement | Addressed |
|---|---|---|---|---|
| `geometry::project` | `inc/cvc/geometry/project_verts.h:159-161, 193-195`; gate `src/cvc/CMakeLists.txt:526-532`; `geometry.cpp:523-531` | `r = max(1.0, min(0.0, r))` always yields 1.0, so the edge fallback snaps to the segment end. The parameter uses `P0+PS` instead of `PS-P0`. Only triangles incident to the nearest of 10 vertices are tested. CGAL-only (GPL-3.0 `K_neighbor_search`), and a **silent no-op** without CGAL, including wasm. The test (`geometry_test.cpp:1014-1045`) only checks that something moved. | `mesh_locator::closest_points` | landed (libigl path; the CGAL heuristic remains the fallback without libigl) |
| `geometry::reorient` | `geometry.h:271-273`, `geometry.cpp:468-519` | Header TODO: flips per-vertex *normals* only, never the winding; order-dependent; quads not handled | `cvc::orient_outward` | landed (new API); legacy kept |
| `calculate_surf_normals` | `geometry.cpp:266-302` | Position-keyed `std::map` (merges seam vertices); averages unit normals without renormalizing; degenerate faces bias toward +x (`utility.h:179-188`) | `compute_vertex_normals(ANGLE)` | landed (new API) |
| `geometry::smoothing` | `src/cvc/processing/smoothing.cpp` | float32 buffers written back for every vertex (`:158-168`, `:652-663`); a fixed `#define maxIndex 20` passes (`:14`); the SMOOTHING pass ignores `delta` and replaces each vertex with the mean of face centroids (`:559-565`), so it shrinks; the perturb passes read stale indices (`:358-360`, `:621-623`) and call `rand()` inside OpenMP (`:614-634`); `SmoothingWithBoundaryFixed` aborts with `free(): invalid pointer` in local Debug (`COVERAGE_IMPROVEMENT_PLAN.md:236`) | `cvc::smooth` (double, implicit, scale-independent) | landed (new API); legacy bugs planned (P2.6) |
| Curvature | `SDF/Geometry/MeshDerivatives.cpp` | Not compiled (`src/cvc/CMakeLists.txt:738, 793`); k1/k2 never computed (`:485`); writes `thetasum.txt` / `amixed.txt` to the CWD; boundary vertices use 2π; `acos` unclamped. `geometry::curvatures()` is filled by no compiled code. | `compute_curvature`, `vertex_curvature` | landed; delete the dead file (P2.6) |
| SDF v1 | `algorithm.cpp:161-250`, wrapper `:187-190`, `:205-223` | Copies `tris()` only (quads ignored). Probable registration error: the library samples `(size+1)³` nodes over [min, max], the wrapper drops the last node per axis but labels the volume with the full bbox, a stretch of `size/(size-1)` (code reading; confirm with a sphere test). | `SDF_IGL` as reference | landed (SDF_IGL); fix planned (P2.6) |
| `tet_faces` | `algorithm.cpp:860-916` | Faces keyed **and returned** as sorted indices, so winding is inconsistent (2 of a tet's 4 faces flipped) | `tet_boundary_surface` | landed (new API); fix planned (P2.2) |
| `hex_faces` | `algorithm.cpp:919-993` (`:985-990`) | Returns sorted 4-tuples; a sorted quad such as (0,1,5,4) → (0,1,4,5) is a **bow-tie**. The test (`geometry_test.cpp:4079-4106`) checks only count and range. | key on the sorted tuple, emit the original cyclic order | planned (P2.2) |
| `decode_tets_from_triangles` | `algorithm.cpp:997-1049` (`std::set` at `:1039-1046`) | Rebuilds each tet from a sorted set ("vertex order may not match original, but that's okay"), so about half the tets come out negatively oriented. Signed volumes and gradients need positive orientation. | decode from tri0 + 4th vertex; `orient_tets` as a safety net | `orient_tets` landed; source fix planned (P2.2) |
| TETRA2 / DOUBLE | `algorithm.cpp:562-585` + `cvc-mesher/Mesher/mesher.cpp:143-149` | `TETRA2→VOLUME_TET→TETRA` and `DOUBLE→MIXED→SINGLE`: `tetrahedralize2` has never produced a TETRA2 mesh through the geometry API | carry the mesh type explicitly | planned (P2.2) |
| `find_tets_containing_point` | `algorithm.cpp:1766-1926` (hexes `:1937+`) | O(n) per query; above 10,000 tets it builds `extract_surface` plus a **CGAL AABB tree per call** (`:1774-1797`); `do_intersect(Point_3)` tests on-surface, not the parity the comment claims (`:1801-1804`) | persistent `tet_locator` (`AABB` over V,T + `in_element`) | planned (P2.4) |
| cvcraw writer | `src/cvc/volume/cvcraw_io.cpp:326-418` (`num_elems = tris.size()/4` at `:342`) | Ignores `tets()` / `hexs()`, so a `cvc::tetrahedralize()` result written to `.raw` loses every element | write the legacy encoding from `tets()`; add `.mesh` / `.msh` | planned (P2.2, P2.5) |
| OFF I/O | `src/cvc/geometry/off_io.cpp:64-85, 173-183` | Rigid parser; the writer drops quads, colours and normals at `setprecision(6)` | `igl::readOFF` / `writeOFF` (optional) | planned, low |
| pycvc | `bindings/pycvc/pycvc.i:481-498` | `tets`, `functions` and `curvatures` are `%ignore`d; there are no triangle or tet views | zero-copy views | planned (P2.3) |
| `GeometryNode` | `src/cvcGL/GeometryNode.cpp:1076-1118, 1141-1174` | TETS/HEXS are wireframe TODOs; no scalar colouring | §2.6 | landed |
| Picking | `SceneRenderer.cpp:153-171` | A new `vtkCellPicker` per call returns only a world point (node, triangle and barycentrics discarded); needs a rendered GL frame; no locator; can hit grid/axis/bbox chrome | node-aware AABB pick | planned (P3.1) |
| `geometry` context | `geometry.cpp:50` vs `geometry.h:330` | The default ctor sets `_ctx = nullptr` despite "never null"; `quality_improve` dereferences it | new APIs take no `app&` | n/a |

**What stays, and why:**

- **`cvc::simplify`** (`simplify.cpp`, 1856 lines): half-edge QEM with seam and attribute welding,
  boundary quadrics, deterministic progressive snapshots that `lod::build_mesh_pyramid` depends on
  (`pyramid.cpp:85-95`), and a sampled Hausdorff.
- **The LBIE mesher**: octree dual contouring and interval/tet/hex meshing, which libigl's MPL core
  does not have. Its tet generator is AGPL TetGen.
- The tet/hex quality metrics.
- The grid PDE filters (`anisotropic_diffusion`, `gdtv_filter`, `bilateral_filter`).
- The `cvc::nav` EDT/A* stacks with bit-exact contracts.

---

## 4. Phase plan (phase 2 onward)

Effort: S < 1 day, M a few days, L a week or more. Items are in priority order within each phase.

| ID | Item | Value | Effort | Risk | Depends on |
|---|---|---|---|---|---|
| P2.1 | Recipe + CI wiring with an ON guard | Blocker for shipping | S | M (all-or-nothing installs) | §7 publish |
| P2.2 | Tet data path fixed at the source | High (FE correctness) | M | M (test expectations) | — |
| P2.3 | pycvc bindings + GRL-SNAM slice-only SDF | High | M | M (SWIG traps) | P2.1 |
| P2.4 | Persistent `tet_locator` | Medium-high | S | L | — |
| P2.5 | `.mesh` / `.msh` FE I/O | Medium | S | L | — |
| P2.6 | Legacy cleanups (smoothing, MeshDerivatives, SDF v1) | Medium | S each | L | — |
| P3.1 | Node-aware AABB picking (`cvc::vis` query_ray) | High | M | M | phase 1 |
| P3.2 | wasm / wasm-mt rollout | High (browser VolRover) | M | M-H (threads) | P2.1 |
| P3.3 | Crease normals | Medium | M | M | — |
| P3.4 | LSCM / harmonic UVs | Medium | M | M | repair |
| P4.1 | CGAL GPL exposure reduction | Medium (licence) | M | M (nav contract) | P2.4 |
| P4.2 | volrover3 §26 `geometry_ops` LGPL-safe backend | Medium-high | L (S per op) | L-M | — |
| P4.3 | Deformation (ARAP / BBW / biharmonic) | Medium | L | M | P2.2 |
| P4.4 | FE beyond scalar PDEs | Depends on owner | M-L | H | P2.2, P2.5 |
| P4.5 | Optional permissive modules (predicates, spectra) | Low-medium | S-M | L-M | — |

### Phase 2 — ship it and make FE trustworthy

**P2.1 Wire eigen + libigl into the libcvc recipe and CI.** Only after release **and** debug
shared bundles exist for linux/x86_64, macos/arm64 and windows/x86_64 (§7). Snippet for
`cvcpkg/recipes/libcvc/recipe.yaml` `depends.runtime`:

```yaml
    # Eigen + libigl (header-only, MPL-2.0 core) back cvc::mesh_ops / cvc::fem
    # (CVC_ENABLE_LIBIGL). Build-only like pocketfft -- nothing under inc/cvc
    # includes them -- and declared under runtime for the same reason: build.sh
    # hands CMake only CVC_DEPS_PREFIX. eigen is listed explicitly although
    # libigl requires it: ci-recipe.yml still installs the archived CLI.
    - name: eigen
      platforms: [linux, macos, windows]
    - name: libigl
      platforms: [linux, macos, windows]
```

- **The all-or-nothing trap.** `cvcpkg install-deps` fails the whole install if any dependency
  lacks a bundle for the target (platform, arch, build_type, link). The `cvcpkg-install` action
  then returns an empty `path`, and configure fails later naming an innocent package (the
  openblas/libiimod precedent in `recipe.yaml`). There is **no debug→release fallback**, and
  `release.yml` / `nightly.yml` install Debug, so publish `config=debug` too, even though the
  headers are identical.
- **Scoping.** Mirror the entries in `libcvc-cuda/recipe.yaml` (and fix its stale `fftw3`), and
  bump `cvc_revision`. Add `wasm-mt` to `platforms:` only once wasm-mt static bundles exist (P3.2):
  the runtime list is baked into the wasm-mt bundle's `required_deps`.
- **Configure sites.** Add `-DCVC_ENABLE_LIBIGL=ON` to `cvcpkg/recipes/libcvc/build.sh` and
  `build.ps1`, the inline configures in `publish-cvcpkg.yml` (`:453`, `:485`, `:1216`, `:1863`),
  `release.yml:170`, and `ci.yml` (`:411`, `:692`, `:893`). The publish workflow does not run
  `build.sh`.
- **The guard.** An auto-OFF build passes green against the stubs, so the package jobs need a
  hard check. Preferred: a `CVC_REQUIRE_LIBIGL` switch that turns the auto-off into
  `FATAL_ERROR`, set in CI and publish (the fail-loud `CVC_FFT_PROVIDER` precedent). It is not
  implemented yet. The fallback is to grep the configure log for the stable substring
  `libigl/Eigen not found` (the full line is `libigl/Eigen not found - mesh_ops/fem compile to
  stubs (CVC_ENABLE_LIBIGL OFF)`), or to require the success line `mesh_ops/fem ENABLED`.
  `CMakeCache.txt` cannot be used: the auto-off is a plain `set()`, so the cache entry still
  reads ON.
- **Value:** every native lane builds the real code. **Effort:** S. **Risk:** M; a missed
  configure site ships a stub bundle silently, which is exactly what the guard catches.

**P2.2 Fix the tet data path at the source.**

- `decode_tets_from_triangles`: decode from the first triangle plus the fourth vertex, or flip by
  signed volume. `orient_tets()` then becomes a safety net.
- Carry TETRA2/DOUBLE through `geometry_type` (or confirm the downgrade is intentional).
- Make the cvcraw writer emit `tets()` / `hexs()`.
- Re-implement `tet_faces` on `boundary_facets` (outward winding) and make `hex_faces` emit the
  original cyclic order.
- First, a spike to count inverted and zero-volume tets in real `tetrahedralize()` output.
  `OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md:211` flags `.tets()` population as unconfirmed, and
  `COVERAGE_IMPROVEMENT_PLAN.md:204, :237` records an empty `tetrahedralize2` result and several
  improve-method segfaults.
- **Value:** high; FE results on LBIE meshes are not trustworthy until this lands. **Effort:** M.
  **Risk:** M; face order and winding change expectations in `geometry_test.cpp` (`:4051-4110`,
  `:4220-4330`, `:4598-4700`).

**P2.3 pycvc bindings.**

- Wrap `mesh_ops` / `fem` behind the PUBLIC `CVC_ENABLE_LIBIGL` define, using the stub pattern of
  `pycvc_algorithm.cpp:35-121`.
- Add `tris_ptr()` / `tets_ptr()` / `functions_ptr()` non-detaching pins to `geometry` (only
  points, colours, uvs and tangents have them today), then zero-copy numpy views of triangles, tets,
  functions and curvatures. Un-ignore what the views replace.
- Release the GIL for long solves.
- Watch the SWIG traps: an `%ignore` silently eats an `%extend`, and nested structs come back
  opaque.
- **GRL-SNAM slice-only SDF.** `sdf_nav.py:186-240` runs `pycvc.sdf(SDF_V2)` over a
  512×512×48 volume and keeps one slice, so 1/48 of the work is used. A `--source igl` option
  calling `mesh_locator::signed_distance` on the 512² slice-plane points does about 48× less work,
  with a robust sign on city meshes. It must go through pycvc: the PyPI `libigl` wheel requires
  scipy, which GRL-SNAM forbids. It is an additional *source*; the bit-exact EDT/`build_sdf`
  contracts are untouched.
- A batched `intersect_ray` overload would also replace `wall_vis.py`'s per-ray Python loop.
- **Value:** high. **Effort:** M. **Risk:** M.

**P2.4 Persistent `tet_locator`.** A class built once (AABB over V,T) with `locate(q)`, a batch
`locate(Q)` (`in_element`, −1 for not found), and barycentric interpolation of `functions()`.
Hexes go through a 5- or 6-tet split that respects `decode_hexs_from_quads`' vertex order. The old
`find_*_containing_point` stay as thin wrappers. It is an additive `mesh_ops.h` change. It is
needed for probing FE fields and for the volrover3 §20.15.6 cage lookup. **Value:** medium-high,
O(log n) instead of O(n) plus a per-call tree. **Effort:** S. **Risk:** L.

**P2.5 FE interchange I/O.** `geometry_file_io` handlers for MEDIT `.mesh` (`igl::readMESH` /
`writeMESH`) and Gmsh `.msh` (`igl::readMSH` / `writeMSH`; v2.6.3 fixed an MSH tag vulnerability,
#2537). They fill `tets()` and set `VOLUME_TET` (registration `geometry_file_io.cpp:212-219`,
handler pattern `off_io.cpp:192-197`). This lets LBIE output reach external solvers and solved
fields come back, and it gives the FE tests a fixture format. **Effort:** S. **Risk:** L; the
first registered handler wins, so check for collisions with Assimp extensions
(`geometry_file_io.cpp:159-167`).

**P2.6 Legacy cleanups.**

- Fix the smoothing perturb stale-index and `rand()`-in-OpenMP bugs, and document that the
  SMOOTHING pass ignores `delta`.
- Delete `SDF/Geometry/MeshDerivatives.*`.
- Confirm or fix the SDF v1 registration with a sphere test, using `SDF_IGL` as the reference.
- Let `calculate_surf_normals` delegate to `compute_vertex_normals` when libigl is present.
- Strengthen `ProjectToTargetSurface` with exact-distance assertions.
- Add `igl::qslim` as an A/B oracle for `cvc::simplify` in a bench target, not a test gate.
- Wire the remaining `SDF_IGL` consumer, the volrover3 `SDFDialog`. (`cvc sdf -a igl`,
  `cvc bunny --volume -a igl` and the Ariadne `sdf:` source's `algorithm` key landed in phase 1.)
- Make `cvc::sdf()` substitute `geom.extents()` for a default-constructed box for every
  algorithm, as `algorithm.h` already promises; today only `SDF_IGL` does, and `SDF_V1` /
  `SDF_V2` turn the all-zero box into a degenerate volume.

### Phase 3 — interaction, rendering, browser

**P3.1 Node-aware AABB picking.** An additive API that keeps `pickWorld`
(`VISIBILITY-AND-LOD-ROADMAP.md:1966`):

- `GraphicsNode::pickLocal`, `SceneGraph::pickRay(originW, dirW, PickHit&)` (GL-free, headless
  testable) and `SceneRenderer::pickNode(x, y, PickHit&)`.
- `PickHit` carries {node, triangle, barycentrics, local, world, t}.
- `GeometryNode` caches a core `mesh_locator`, invalidated by `setGeometry` and `updateVertices`.
  After a deform it is rebuilt from the vtkPoints buffer, because `m_geometry` goes stale there
  (`GeometryNode.cpp:599-649`).
- `LodGraphicsNode` picks rung 0. Streaming nodes and VolRen/VolSlice return false. Chrome is
  never hit. Clip planes and visibility are respected.
- This is the narrow phase for the reserved `cvc::vis` layer (c) `query_ray`
  (`VISIBILITY-AND-LOD-ROADMAP.md:920, 938`). It enables click-to-geodesic in `mesh_lab`, a
  `pick_node` Python wrapper and a `gl-pick` Ariadne verb (the input bridge to `post_pointer` is a
  known gap).
- **Value:** high; O(log n) on the 978k-triangle Austin mesh, headless and wasm-safe.
  **Effort:** M. **Risk:** M; draw/pick parity under clip planes and LOD, and the owner-thread
  rule for VTK.

**P3.2 wasm / wasm-mt rollout.**

- Publish wasm (SERIAL) and wasm-mt (POOL) bundles (§7). Append `eigen libigl` to the hard-coded
  dependency loops: `publish-cvcgl-wasm.yml:87`, `deploy-pages.yml:67`,
  `publish-pycvc-gl-wasm.yml`, `cvcpkg/recipes/cvcgl-examples/build-wasm.sh`,
  `pycvc-gl-cp312/build-wasm.{sh,ps1}`, `docs/FULL_BUILD_CVCPKG.md:165`. Then add `wasm-mt` to
  the libcvc recipe scoping. Until then the wasm builds keep working through auto-OFF.
- **Threads (decided).** The backend follows the bundles, so libcvc and every other `igl::core`
  consumer in a static link compile the same `parallel_for` body: a define added only in libcvc
  would be an ODR violation there. Single-threaded wasm is **serial**: the wasm bundle is SERIAL
  and exports `IGL_PARALLEL_FOR_FORCE_SERIAL`, and libcvc's igl TUs define it too. wasm-mt
  (pthreads) uses the **pool**, as the wasm-mt bundle is built. That pool starts
  `default_num_threads()` workers on the first parallel loop, while the examples pre-spawn only
  `-sPTHREAD_POOL_SIZE=4` (`src/cvcGL/examples/CMakeLists.txt:222`), and a pthread beyond that
  pool cannot start until the browser main thread yields. So on wasm-mt, never run `mesh_ops` /
  `fem` / `SDF_IGL` on the browser main thread with more igl workers than the pthread pool: run
  them on an `async_lane` (as `mesh_lab` / `fe_lab` do), or set `IGL_NUM_THREADS` no higher than
  the pool. A libcvc-side cap (§2.2 follow-up) would make this automatic.
- On single-threaded wasm the `async_lane` runs deferred on the render thread, so keep demo sizes
  small.
- **Value:** high; browser VolRover and the gallery get the same operations. **Effort:** M.
  **Risk:** M-H; thread deadlocks and code size (measure).

**P3.3 Crease normals.**

- A core `split_creases(geometry, degrees, &source_vertex)` built on `igl::per_corner_normals`
  (threshold in **degrees**; `igl::sharp_edges` takes **radians**), producing split vertices plus
  a map back to the source vertex.
- A GeometryNode `crease_angle` key. `updateVertices` / `updateColors` accept source-sized arrays
  and expand them through the map, so the wind and crowd fast paths survive.
- Split **after** LOD (in `RungStyle`), never before: a pre-split mesh stalls the ladder at
  rung 0.
- **Effort:** M. **Risk:** M.

**P3.4 LSCM / harmonic UVs.**

- A core `auto_uv(geometry&, method)` using `igl::lscm` (free boundary needs a single disk-like
  component) or `boundary_loop` + `map_vertices_to_circle` + `harmonic`. Ariadne would get
  `material: { texture: x, uv: lscm }`.
- The bunny has several holes and is not a disk: demonstrate on a patch or heightfield, and add
  cutting or charts later.
- **Effort:** M. **Risk:** M; requires manifold input, so repair first.

### Phase 4 — scope growth

**P4.1 Reduce CGAL GPL exposure.** GPL-3.0-or-later CGAL headers are compiled into libcvc whenever
CGAL is found:

- `project_verts.h:61-66` (`K_neighbor_search`), no longer used by `project()` in libigl builds
  but still compiled as its fallback; drop it once libigl is required;
- `algorithm.cpp:62-73` (`AABB_tree`), retired by P2.4;
- `nav/spatial.cpp:41-48` (`Kd_tree`, `Fuzzy_sphere`), which **remains**. It is fixed-radius
  search, and libigl offers only k-NN, so it needs a permissive k-d tree (for example nanoflann)
  or a hash grid that preserves the byte-identical neighbour contract (`spatial.cpp:23-35`).

Once all three are gone, `DISABLE_CGAL=ON` can become the default and the gmp/mpfr runtime
dependencies drop out. **Value:** medium. **Effort:** M. **Risk:** M (the nav contract).

**P4.2 volrover3 §26 `geometry_ops`, LGPL-safe backend.** §26.2 rules GPL-3.0 vcglib out of the
libcvc library half. The MPL-coverable subset ships in libcvc on libigl:

- Hausdorff, with a certified upper bound via the per-triangle overload plus an AABB.
- Curvature.
- Attribute transfer (closest point plus barycentric).
- Measure (area, volume, centroid).
- Poisson-disk sampling (`blue_noise`).
- Connected components.
- Topological hole fill (a fan, no fairing).
- Clean.
- Ambient occlusion.
- Marching cubes.
- Loop/upsample subdivision.
- LSCM/harmonic UV.

Screened Poisson reconstruction, ball pivoting, isotropic remeshing and exact booleans stay
app-side or come from other permissively licensed libraries. **Effort:** L overall, S per op.
**Risk:** L-M.

**P4.3 Deformation for volrover3 §20.15.6.** A `cvc::deform` namespace:

- ARAP (`arap_precomputation` / `arap_solve`);
- BBW and `biharmonic_coordinates` on an LBIE cage tet mesh;
- MVC for closed cages;
- `lbs_matrix` / `dqs` for runtime skinning without Python vertex traffic.

The roadmap's "libigl has this" (§20.15.6) holds for every item. The volrover3 roadmap prefers
OZZ for skinning. **Effort:** L. **Risk:** M; per-frame solve cost, and the cage lookup needs
P2.4.

**P4.4 FE beyond scalar PDEs.** libigl has **no** linear-elastic or neo-Hookean stiffness assembly
(only ARAP-family geometric energies), no hex elements, and no tet-mesh optimizer. Options:

| Option | What | Cost / caveat |
|---|---|---|
| (a) Own P1 linear elasticity in `cvc::fem` | Per-tet 12×12 `K_e = V_e Bᵀ D B` from `igl::grad`; assembled with Eigen; Dirichlet via `min_quad_with_fixed`; von Mises as a per-vertex scalar for GeometryNode | M. Linear tets lock in near-incompressible materials. Needs a validation case (a cantilever against beam theory). Displacement is a vector field, and `geometry` has one scalar `functions()` slot, so the return types are separate or a named-field store is added. |
| (b) ARAP-family dynamics (`igl::arap`) | Plausible deformation, not physical stress | S-M; no material units |
| (c) External solver | `.mesh` / `.msh` out, fields back in (P2.5) | Check each solver's licence before linking; exchanging files keeps it at arm's length |
| (d) PBD/XPBD or a physics engine | volrover3 §20.15 already plans soft bodies this way | Outside libcvc's FE layer |

Recommendation: (c) until a physics owner commits to a validation case, then (a). **Risk:** H if
(a) starts without that case.

**P4.5 Optional permissive modules as future recipes.**

- **`igl::predicates`** (Shewchuk, public domain): exact `orient3d`, `insphere`, self-intersection
  and Delaunay helpers. It exports global `extern "C"` `orient3d` and friends, which can clash with
  other vendored copies of the predicates.
- **`igl::spectra`** (Spectra, MPL-2.0): proper sparse eigensolvers. Core `igl::eigs` is naive
  power iteration and supports only small-magnitude pairs well (`eigs.h:34-45`), which is why
  `laplacian_eigenmodes` does not use it (§2.4): it runs its own shift-invert subspace iteration.
  Spectra would matter if large k or many very large meshes become a use case.
- Both need their own source staging (`FETCHCONTENT_SOURCE_DIR_*` or separate recipes) and an
  export-set patch, because upstream installs only `igl_core`. **Effort:** S-M.

---

## 5. What not to do, and why

- **Do not replace `cvc::simplify` with `igl::decimate` / `igl::qslim`.** Both need edge-manifold
  int32 input, carry no uv/colour/normal attributes, and weld no seams. The LOD pyramid depends on
  `simplify`'s bit-identical progressive snapshots. `LSYSTEM-LABORATORY-ROADMAP.md:121` already
  decided "Neither". Use qslim only as a bench oracle.
- **Do not enable any libigl copyleft or restricted module**, in libcvc or in the main-set bundle:
  - `copyleft/cgal` (GPL-3+, including `mesh_boolean`);
  - `copyleft/tetgen` (AGPL-3+);
  - `copyleft/marching_cubes_tables.h` (GNU Library GPL v2; use core `igl::marching_cubes`);
  - `triangle` (non-commercial, which also hosts `scaf`, `cdt` and `triangulate`);
  - `matlab` and `mosek` (proprietary).
- **Do not reach for TetGen, Triangle or CGAL booleans to fill gaps.** Tet-meshing stays on LBIE;
  the volrover3 roadmap names fTetWild as the fallback for surface-only input. Exact booleans stay
  out of libcvc.
- **Do not install the PyPI `libigl` wheel into a shared cvcpkg prefix.** It drops Eigen 3.4.0
  into `<prefix>/include/Eigen` (shadowing `include/eigen3` for `-I<prefix>/include` consumers),
  plus Embree libraries and MSVC runtime DLLs into `bin/`. It also redistributes the copyleft/cgal,
  tetgen and Triangle modules. It requires scipy (excluded from GRL-SNAM), and its sdist is not
  hermetic. Python access goes through pycvc.
- **Do not include Eigen or libigl in installed headers, cvcGL, Ariadne or tests.** That brings
  SWIG/consumer dependencies, `.def` bloat and one thread pool per DLL.
- **Do not instantiate libigl on `Eigen::Map` over `cvc::geometry` buffers.** Copy to
  `MatrixXd` / `MatrixXi`.
- **Do not mix VTK's `vtkeigen` or vcglib's bundled `eigenlib` with `<Eigen/...>` in one
  translation unit.** They share include guards, and `#define Eigen vtkeigen` is a global macro.
- **Do not build libigl static for libcvc** (§1), and **do not pin Eigen 3.4.0 "for VTK"** (wrong
  premise, and the emscripten SIMD bug).
- **Do not replace LBIE, the grid PDE filters or the nav EDT/A* contracts with libigl.**

---

## 6. Licensing

libcvc is **LGPL-2.1-only** (no "or later").

- **MPL-2.0 core is compatible.** MPL-2.0 §3.3 lets Covered Software be combined into a Larger Work
  under a Secondary License, and §1.12 lists the GNU LGPL v2.1. The exception is files marked
  "Incompatible With Secondary Licenses"; a search of Eigen 5.0.1 and libigl `include/` finds none.
  The MPL files stay MPL. libcvc does not modify them, and the obligation to tell recipients where
  the source is (§3.2(a)) is met by the cvcpkg bundles, which ship the full headers, plus the
  [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md) entries.
- **Third-party code inside the MPL packages, as compiled by this change.** This is the include
  closure of the four `src/cvc/geometry/igl/*.cpp` TUs, scanned over the upstream sources
  following every `#include` regardless of `#if`. Every item is recorded in
  [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md).
  - Eigen, always included: `BFloat16.h` (Apache-2.0, included by `Eigen/Core:205`), `Half.h`
    (permissive notice) and `AlignedBox.h`'s `transform()` (BSD-3-Clause, through libigl's
    `AABB.h`).
  - Eigen's sparse solvers, which `<Eigen/Sparse>` and libigl's `min_quad_with_fixed` bring in:
    `OrderingMethods/Eigen_Colamd.h` carries the University of Florida **COLAMD** notice, which
    must be retained "and made accessible to the end-user", together with the availability line.
    Unlike `Amd.h` it has no MPL relicensing grant. `SparseCore/SparseColEtree.h` and the
    `SparseLU/SparseLU_*.h` kernels carry the **SuperLU** (Xerox, 1994) notice, which must be
    retained along with a notice that the code was modified (the files already state it).
    `min_quad_with_fixed` instantiates a `SparseLU` and a `SparseQR` with `COLAMDOrdering`, so this
    code is emitted, not just included.
  - Eigen on SSE/NEON targets: `LU/arch/InverseSize4.h` (Intel 2001; the notice must appear in all
    copies), emitted only for fixed-size 4x4 inverses.
  - Eigen, MPL-2.0 with a stated outside origin and no other terms: `Amd.h` and
    `SimplicialCholesky_impl.h` (Timothy A. Davis's CSparse/LDL, licensed for distribution under
    the MPL-2.0), `IncompleteLUT.h` (SPARSKIT, relicensed with Yousef Saad's permission),
    `EigenSolver.h` / `RealSchur.h` (public-domain JAMA), and `unsupported/Eigen/SparseExtra`,
    which libigl's `min_quad_with_fixed.h` includes.
  - Eigen's BLAS, LAPACKE and MKL backends (BSD-3-Clause) compile only with `EIGEN_USE_*`, which
    libcvc does not define.
  - libigl: `FastWindingNumberForSoups.h` (MIT), `raytri.c` (public domain; included through
    `AABB.h`) and the matplotlib tables in `colormap.cpp` (CC0).
  - `colormap.cpp` also holds a `turbo` table, linked in the file to Google's 2019 Turbo colormap
    (Google published its reference Turbo tables under Apache-2.0) and also used for libigl's
    `jet`, and a `parula` table that matches MATLAB's default map. libigl attributes neither
    beyond the file's MPL header. **cvc's own fallback is no way out:** without libigl,
    `cvc::colormap_rgb` interpolates 17-point samples (`mesh_ops.cpp`) taken *from these same
    tables*, so every libcvc build, `CVC_ENABLE_LIBIGL=OFF` included, carries data derived from
    them, and THIRD_PARTY_NOTICES.md records it for both. cvc's `JET` is its own analytic MATLAB
    jet in every build and never used libigl's. The only real remedy is independently sourced
    data: keep the CC0 matplotlib maps; take Turbo from Google's reference (Apache-2.0, so it joins
    D-L1) or drop it; drop PARULA or replace it with an independently sourced map. This is part
    of D-L1.
  - The COLAMD, SuperLU and Intel notices are attribution terms. Installing
    THIRD_PARTY_NOTICES.md with the binaries, as the libcvc and cvcGL installs already do, is how
    they are met; every libcvc bundle built with `CVC_ENABLE_LIBIGL=ON` must keep shipping it.
  - Present in the libigl package but **not** compiled into libcvc: the
    `Singular_Value_Decomposition_*.hpp` kernels (BSD-2-Clause, reached only through `svd3x3`,
    `polar_svd3x3` and `fit_rotations`; their notice is kept in THIRD_PARTY_NOTICES.md because P4.3
    ARAP would reach them), `exact_geodesic.cpp` (MIT), `tri_tri_intersect.h` (MIT), `tinyply.h`
    (public domain) and `marching_cubes.cpp` (adapted from public-domain code). Add notices if a
    later phase pulls them in.
- **Owner decision (D-L1): the Apache-2.0 `BFloat16.h`.**
  - Eigen's `COPYING.README` calls its non-MPL files "MPL2-compatible". The FSF considers Apache-2.0
    incompatible with GPLv2 because of its patent-termination and indemnity terms, and the same
    reasoning is commonly applied to LGPL-2.1-only.
  - Facts that bear on it:
    1. No Eigen version avoids the file: Eigen 3.4.1 and VTK's `vtkeigen` 3.4.0 carry it too.
    2. VTK already ships it inside its own DLLs in every cvcGL process, but `cvc.dll` would be the
       first LGPL-2.1-only binary to compile it.
    3. libcvc never uses `Eigen::bfloat16`, so its inline code is probably not emitted. That is a
       technical observation, not a licence answer; check with `dumpbin /symbols` / `nm`.
  - The notice is recorded either way. **Needs sign-off before the first published libcvc bundle
    built with `CVC_ENABLE_LIBIGL=ON`.**
- **Owner decision (D-L2): existing CGAL exposure.** GPL-3.0-or-later CGAL headers (`AABB_tree.h`,
  `K_neighbor_search.h`, `Kd_tree.h`, `Fuzzy_sphere.h`, `Search_traits_2.h`; `Simple_cartesian.h`
  and `Bbox_3.h` are LGPL-3.0-or-later) are compiled into libcvc whenever CGAL is found, and
  THIRD_PARTY_NOTICES.md does not list CGAL. This predates this change; P4.1 reduces it.
- **Recipe licence fields.** Each recipe's `license` field and the comment above it are
  authoritative. They must describe the whole bundle, not only what libcvc compiles:
  - eigen ships, besides the MPL-2.0 files: Apache-2.0 (`BFloat16.h`), BSD-3-Clause (the Intel
    backends, `AlignedBox.h`'s `transform()`), the permissive notices of `Half.h` and
    `InverseSize4.h`, COLAMD (`Eigen_Colamd.h`), SuperLU (`SparseColEtree.h`, `SparseLU_*.h`) and
    MINPACK (`unsupported/Eigen/src/NonLinearOptimization`, `LevenbergMarquardt`). libcvc
    compiles `unsupported/Eigen/SparseExtra` (MPL-2.0) through libigl's `min_quad_with_fixed.h`,
    but no MINPACK-derived header.
  - libigl ships, besides MPL-2.0: MIT (`FastWindingNumberForSoups.h`, `exact_geodesic.cpp`,
    `tri_tri_intersect.h`), BSD-2-Clause (the SVD kernels), CC0 (the matplotlib colormap
    tables) and public-domain files (`raytri.c`, `tinyply.h`).
  - GitHub reports libigl as GPL-3.0 only because it detects `LICENSE.GPL`, which covers the
    copyleft modules.
  - The bundles install the upstream licence texts in `share/licenses/eigen/` and
    `share/licenses/libigl/`, which is where the full Apache-2.0 text comes from.

---

## 7. Publish runbook for the new recipes

Merging to cy-pca/cvcpkg master **publishes nothing**. Every dispatch below is a **production
publish to cvcpkg.org and needs an explicit go-ahead from the owner, per dispatch.** Never push
master to `prod` to get recipes built; `prod` deploys the server.

1. **Before the PR.** In `cvcpkg-libigl-wt`, run from source (`PYTHONPATH=src`, never
   `pip install -e`):
   - `python packaging/validate.py recipes/eigen recipes/libigl`
   - `python scripts/validate_all_recipes.py`
   - the three unit tests (`test_recipe_schema.py`, `test_required_deps_no_host_tools.py`,
     `test_validate_all_recipes.py`)
   - a local Windows build of eigen, then libigl, with the same `--prefix` (`--local --recipes-dir
     recipes`, `CVC_JOBS=4`; avoid `--incremental`). The libigl smoke build runs inside
     `build.ps1`. The Windows-hosted `wasm` entries (`build-wasm.ps1`) can be checked the same
     way. Done for this branch: Windows native, Windows-hosted wasm (with the node smoke) and WSL
     Ubuntu 24.04 native all built and passed. Rerun after any recipe change.
   - `cvcpkg pack` for both, because `package.files` is enforced at pack/publish, not at build.

   Author the PR as Joe Rivera, with no AI attribution.
2. **Native fleet** (linux/x86_64, BSDs, windows via sandipaws, which must stay awake):
   ```sh
   gh workflow run populate-server.yml --repo cy-pca/cvcpkg --ref master \
     -f recipes="eigen libigl" -f platforms=linux,freebsd,netbsd,openbsd,windows \
     -f arch=x86_64 -f config=all -f link=shared
   ```
   `config=all` covers release and debug (or run `release`, then `debug`). The job pushes the
   recipes, then `submit-dag --skip-existing` and `follow-dag`. A follow-dag timeout is not a
   failure; check the catalog.
3. **wasm / wasm-mt** (linux cross, static is forced):
   ```sh
   gh workflow run populate-server.yml --repo cy-pca/cvcpkg --ref master \
     -f recipes="eigen libigl" -f platforms=wasm,wasm-mt -f arch=wasm32 -f config=release -f link=static
   ```
   Only needed for P3.2. `wasi` and `cosmo` are eigen-only and optional.
4. **macOS.** populate-server's own macOS jobs pass **neither `recipes` nor `ref`** and would
   rebuild everything from `prod`, so dispatch directly with `ref=master`, eigen before libigl:
   ```sh
   gh workflow run macos-build.yml --repo cy-pca/cvcpkg \
     -f recipes="eigen libigl" -f ref=master -f config=release -f link=shared
   gh workflow run macos-build.yml --repo cy-pca/cvcpkg \
     -f recipes="eigen libigl" -f ref=master -f config=debug -f link=shared
   # optional x86_64: add -f runner=macos-15-intel
   ```
5. **linux/arm64** (optional; publish-cvcpkg's arm lane is experimental):
   ```sh
   gh workflow run linux-arm-build.yml --repo cy-pca/cvcpkg -f recipes="eigen libigl" -f ref=master
   ```
   The default `ref` is `prod`. `windows-build.yml` (GitHub windows-2022, `-f ref=master`) is the
   alternative if sandipaws is unavailable.
6. **Verify:**
   ```sh
   curl -s "https://cvcpkg.org/v1/packages?name=eigen&limit=1000"
   curl -s "https://cvcpkg.org/v1/packages?name=libigl&limit=1000"
   ```
   Confirm linux/x86_64, macos/arm64 and windows/x86_64 × {release, debug} × shared at
   `5.0.1+cvc.1` / `2.6.4+cvc.1`, plus wasm32 static for P3.2. Optionally run
   `cvcpkg install-deps cvcpkg/recipes/libcvc` into a scratch prefix with the P2.1 change applied.
7. **Only then** land P2.1 in transfix/libcvc. libcvc itself is published by its own
   `publish-cvcpkg.yml` (a `v*` tag or a dispatch), which is a separate go-ahead.

---

## 8. Open questions

1. **D-L1:** sign-off on compiling Eigen's Apache-2.0 `BFloat16.h` into LGPL-2.1-only `cvc.dll`,
   and on the unattributed `turbo` / `parula` colormap data. That data is in every libcvc build:
   libigl's tables with `CVC_ENABLE_LIBIGL`, and cvc's 17-point samples of the same tables
   without it, so cvc's fallback is not an alternative. The choice is between accepting them and
   replacing them with independently sourced data (§6).
2. **D-L2:** should THIRD_PARTY_NOTICES.md record the CGAL headers already compiled in, and is
   making `DISABLE_CGAL` the default (after P4.1) a goal?
3. Should `CVC_WITH_LIBIGL` (the originally requested spelling) be accepted as an alias of
   `CVC_ENABLE_LIBIGL`?
4. Keep the `-Dmodule_export=core` workaround, or carry an upstreamable patch (EXPORT_NAME, project
   version, config directory)? A patch needs `*.patch -text` in cvcpkg's `.gitattributes`.
5. *Answered:* single-threaded wasm is serial (matching the SERIAL bundle); wasm-mt uses the pool
   (matching the POOL bundle), with igl work kept off the browser main thread or
   `IGL_NUM_THREADS` kept within the pthread pool (P3.2). Still open: whether libcvc should cap the
   pool itself (§2.2 follow-up).
6. FE data model: one scalar `functions()` per vertex, with no per-element or named fields. Extend
   `cvc::geometry` with a named attribute store, or keep returning fields separately as `fem.h`
   does?
7. Normalize the legacy "tet as 4 triangles in `tris()`" encoding to `tets()` on read/write
   (cvcraw), and is the TETRA2/DOUBLE downgrade intentional (P2.2)?
8. *Answered (2026-10-09):* LBIE meshes `{v > isovalue}`. It stores `-v` and `-isovalue`
   internally and fills below (`LBIE/octree.cpp`, `LBIE_Mesher.h`). The built cvc-cli confirms it
   on the bunny SDF: the raw, negative-inside SDF at isovalue 0 meshes the box *outside* the bunny,
   and the negated SDF meshes the bunny. `fe_lab` therefore negates by default for SDF sources
   (`mesh.inside: below`), and GEOMETRY_API.md's examples negate before `tetrahedralize()`. A gtest
   that pins the side is a follow-up (§2.8).
9. Target physics for FE: is elasticity actually wanted, and by whom (P4.4)?
10. *Answered for scalar fields:* the FE layer meets WATER D-W6's condition, and the first FE demo
    is the cheaper view: `fe_lab` in `src/cvcGL/examples/mesh_lab.ari`, showing the field on the
    GeometryNode tet boundary surface plus a `slice_tets` cut. W10's `water_column` stays where
    the WATER roadmap puts it (last).
11. Should vcglib gain an `eigen` runtime dependency? Its config already does
    `find_dependency(Eigen3 QUIET)`, but its headers reportedly use a bundled 3.4.0 `eigenlib/` by
    relative path; check with a real vcglib bundle first.
12. Eigen 5.0.1 has small fixed-size performance regressions that are fixed only on unreleased
    master (Eigen#3083, #3117). Measure the curvature and AABB paths before calling them a
    bottleneck.
