# VolumeRover and TexMol as Ariadne Applications — Roadmap (sketch)

**Status:** 2026-11-28. Joe accepted every §9 recommendation, then moved PEDetection from "left
behind" to a port, V11 (§9). CC1, PP1, V0, V1 and M0b have started, in parallel sessions; nothing
of them has landed yet.

**Asked for (Joe, 2026-11-27):** "surely we arent done with the porting. we need whatever is left
in volrover and texmol that we havent ported, then we need new versions of both volrover and texmol
as ariadne applications and component collections".

**Based on:**
- libcvc `origin/master @ f152617d7`;
- transfix/volrover `@ 9c9ddec`: the legacy libraries under `src/<Lib>` and `inc/<Lib>`, the legacy
  app `src/VolumeRover2`, and `volrover3/`;
- transfix/TexMol `@ e621ebc`, as read for transfix/libcvc#1090;
- cy-pca/cyconv `docs/plan/PLAN.md @ 65cd3ef7` (the arirov, arimol and aridock plan).

**Scope.** Three parts:
1. The VolumeRover libraries that the first port plan left out (§2).
2. The TexMol phases not yet built (§3).
3. Component collections (§4), and arirov and arimol as Ariadne applications built from them (§5,
   §6), with the Ariadne pieces both need (§7).

**Companions:**
- [VOLROVER-LIBRARY-PORTS-ROADMAP.md](VOLROVER-LIBRARY-PORTS-ROADMAP.md): its §1 rules govern every
  port here, and its §7 "separate items afterwards" are part of §2.
- [TEXMOL-F2DOCK-ARISRV-ROADMAP.md](TEXMOL-F2DOCK-ARISRV-ROADMAP.md): its §8 phases are §3's list.
- [ARIADNE_SHELL.md](ARIADNE_SHELL.md): the shell that runs `.ari` documents as applications.
- [ARIADNE_SCRIPTING_HOSTS.md](ARIADNE_SCRIPTING_HOSTS.md): §7's "VolRover as an Ariadne app" and
  its phase D2. This plan replaces both.
- [ARIADNE.md](../ARIADNE.md) §6: `import:`, `include:`, `load:`, `link:` holes and the component
  library.
- cyconv's PLAN.md: §7 (arirov) and §8 (arimol), which this plan replaces for these two apps (§9
  item 1); §5's dev tools and §10's SDK items, which it keeps.

---

## 0. Summary

**Where the ports stand.**
- VolumeRover: every phase of the first plan is on master.
  - P0a–P0d: the app context, the shared pieces, the permissive libraries and the shared types.
  - P1, P2: the skeleton (transfix/libcvc#1157, #1402).
  - P3a, P3b, P4: the planar module, the contour tiler and `contour_lab` (#1164, #1332, #1397).
  - P5, P6: GenSeg and its paper mode (#1248, #1292).
  - P7: the viewport pieces (#1249).
  - P8–P10: `volume_grid` with its Probe, Segmentation and Contours tabs (#1400, #1401, #1423, #1443,
    #1452, #1451).
  - P11's docs are transfix/libcvc#1604. Its last item, retiring the legacy libraries from
    volrover, moves here (§5.4).
- TexMol: M0 part 1 (`cvc::mol`, #1427), T1 part 1 (`cvc::mol::blur`, #1428) and T5 (PocketTunnel
  and SecondaryStructures, #1439 and #1440) are on master. So is the skeleton of PDB 1GRL (#1430).
  Everything else in that plan's §8 is not started.

**What is left of VolumeRover** (§2). Every menu item of VolumeRover2 was checked against libcvc.
- The four volume filters and LBIE's meshing and quality improvement are already in libcvc. V0
  gave the three that lacked it the job form an Ariadne window drives, and the window.
- Not in libcvc, with a port planned (V1–V9): Tight Cocone as a feature, surface curation,
  super-secondary structures, the HLS surface, MMHLS, the level-set segmentations, the virus-map
  segmentations, SecStruct's density-based secondary structure, and SweetMesh's hex meshes. SDF2D
  (V10) waits until someone wants it.
- Sangmin Park's pulmonary-embolism detection (PEDetection) is V11. It starts with a feasibility
  step, because it has never run in volrover's build.
- ET Reconstruction is reserved by its authors ("All right are reserved") and links GPL FFTW, so it
  stays behind unless they relicense it.
- Left behind: ContourFilter, SweetLBIE, the multi-tile server, the ray tracer and SegServ (§2.3).

**What is left of TexMol** (§3): M0 part 2 with the molecular scene nodes, T1 part 2, T2 (whose
HLevelSet is the same port as VolumeRover's HLS surface), T3, T4, T5's CLI, pycvc and intrinsics,
T6, J0, R1–R3, F0–F4 and C1, as that plan has them.

**The apps** (§4–§6).
- Every feature ends as a component in a *collection*: `cvc://components/volume/`, `mesh/`, `mol/`
  and `dev/`. The `cvc://` resolver already reaches subdirectories; the tests and the wasm build
  need small changes (§4).
- arirov and arimol are `.ari` documents that assemble those components: a menu bar, a main view of
  the app's data, windows from the collections, and a data browser. They carry no native code,
  because a document cannot bring node types or intrinsics of its own; everything native is in
  libcvc (§1).
- No converter. VolumeRover2 is Qt3/Qt4-era code that cyconv's D6 already excludes, volrover3's ten
  dialogs mostly exist as components already, and every feature needs a new window over a new
  `cvc::` API anyway. cyconv's dev tools (state inspector, exec console) and its widget items stay
  on the list, as libcvc work (§7).
- Each app runs alone in `arigl`, `ariglx` and arigl-web first, and in the shell once the shell
  draws a mounted document's scene (SH3) and gives it the main view (SH6).
- When arirov passes VolumeRover2's parity list, VolumeRover2 and the legacy libraries retire from
  volrover's master (§5.4). When arimol passes TexMol's, TexMol's Qt app does the same (§6.3).

**Order** (§8). Three tracks can run at once:
1. collections, parameter panels and the missing widgets, then arirov v1 from what exists today;
2. the VolumeRover ports, each adding its window to arirov;
3. TexMol's M0 part 2 (the molecular nodes) and T1 part 2, then arimol v1.

---

## 1. Rules

The VolumeRover plan's §1 applies to every port here, unchanged:
- no GPL code, so no CGAL algorithm packages and no FFTW;
- no singletons, with everything on `cvc::app`;
- one API shape that pycvc wraps;
- parameters and data on the state tree;
- DSL intrinsics;
- the §1.7 test matrix, with an A/B against the legacy library where one builds;
- wasm from the first PR;
- every long routine a resumable stepper;
- no XML-RPC.

This plan adds five rules.

1. **A port ends in a component.** A port is done when its window is in a collection, with a demo
   document, a golden image and an arigl-web smoke case, as P2, P4 and P8–P10 did with
   `mesh_tools`, `contour_tiler` and `volume_grid`.
2. **Components view data on the state tree.** A component never loads a file and never copies
   data. It views what the app put on the state tree and follows changes, as `volren`,
   `volume_slice` and `volume_grid` share one volume since transfix/libcvc#1446. Opening a file is
   the app's job (§7.4).
3. **Native code lives in libcvc.** A document cannot register node types, widgets or intrinsics:
   - there is no plugin or `dlopen` path in the hosts;
   - pycvc binds neither `register_scene_node_type` nor `register_widget_type`;
   - a mounted module's `python:` block does not run (ARIADNE_SCRIPTING_HOSTS.md P1b, "left");
   - sandboxed principals refuse Python outright.

   So every node type, intrinsic and job kind an app needs is registered by
   `register_cvcgl_extensions` or its like, and the apps are documents only.
4. **An app runs the same alone and mounted** (ARIADNE_SHELL.md §1). It finds its state at its own
   prefix in both cases, and it asks for what it needs beyond that (`requires:`, and SH7's grants).
5. **Python is optional in an app.** arirov keeps volrover3's scripting where `ariglx` runs it, but
   no feature depends on it, because a mounted app's Python does not run until P1b's `load:`ed
   modules land.

---

## 2. What is left of VolumeRover

### 2.1 How this was counted

- Every library under volrover's `src/` was compared with libcvc master, and every action in
  VolumeRover2's menus (`src/VolumeRover2/CVCMainWindow.cpp:397-490`) was traced to the library it
  calls.
- volrover's CI configures with `-DDISABLE_CGAL=ON` (`.github/workflows/ci.yml:136,146,165`). So CI
  has never built TightCocone, Curation, SuperSecondaryStructures, any of `src/Segmentation`,
  ContourFilter or VolumeGridRover. HLevelSet, SweetMesh and LBIE are built.
- Several dialog slots use Qt3 APIs that Qt6 lacks (`QDir::absPath`, `.ascii()`; for example
  `CVCMainWindow.cpp:2898-2901`, `ReconstructionDialog.cpp:138-200`), so those features don't run in
  today's VolumeRover2 either.

### 2.2 The inventory

"Own lines" leave out vendored code and generated tables. "Cocone copies" are the files that
libcvc's `src/cvc/geometry/cocone/` already holds, lifted for PocketTunnel and
SecondaryStructures.

| Library | What it computes | Own lines | Licence | Dependencies | VolumeRover2 menu | In libcvc |
|---|---|---:|---|---|---|---|
| TightCocone | A watertight surface from a point cloud (Tight Cocone, optionally Robust Cocone); also boundary points from a volume | about 825 unique, plus 3.3k cocone copies and 1.7k of GVF / segmentation code | UT LGPL-2.1, all 21 files | CGAL Delaunay_triangulation_3 (GPL); Boost | Tools › Tight Cocone | The core, with no public API and no surface extraction |
| Curation | Tight Cocone, then pockets and tunnels by stable manifolds, then all but N filled | about 4.4k, plus cocone copies | UT LGPL-2.1, all 29 files | As TightCocone | Tools › Surface Curation | Part: PocketTunnel's private `grow_maximum`, `detect_handle`; no merge-ratio clubbing, no `curate_tr` |
| SuperSecondaryStructures | Tight Cocone, then the inside cells split into stable-manifold segments; the N largest kept | about 2.6k, plus cocone copies | UT LGPL on 2 files, none on 22 | As TightCocone | Tools › SuperSecondary Structures | No |
| HLevelSet | A cubic B-spline level-set surface from points (Xu–Li), and atoms' SES, as a volume | 10,755 compiled | UT LGPL-2.1 | VolMagick, Boost | Tools › HLS Surface | No (TexMol's T2 plans it as `mol.hls`) |
| MMHLS | Multi-domain meshes with gaps: per-label HLS volumes, partition surfaces, non-intersecting meshes | about 6.8k (plus 2k of case tables) | LGPL-2.1 on 19 files, none on 9 | HLevelSet, FastContouring, Qt | MMHLS › GenerateRawV, GenerateMesh | Part: `cvc::class_surfaces` (no HLS, no gaps) |
| MSLevelSet, HigherOrderSegmentation, MultiphaseSegmentation | Mumford–Shah (Chan–Vese) level-set segmentation: two-phase, with higher-order terms, and multiphase (n functions, 2^n phases); one inheritance chain | 1,623 + 1,791 + 1,270 | LGPL-2.1, © Technical University of Denmark | CUDA only (legacy cutil, texture references) | MSLevelSet, HOLevelSet and MultiPhase Segmentation | No |
| SegCapsid, SegMonomer, SegSubunit | Virus maps: capsid layers from icosahedral symmetry, monomers, subunits by local symmetry | 8.7k + 3.8k + 11.7k | The CVC "LGPL" notice with a commercial-licence clause and no version; some Xcode "All rights reserved" stubs | XmlRpc++ for parameters | Tools › Segment Virus Map | Building blocks: gradient vector diffusion, GenSeg's critical points and fast marching |
| SecStruct | Helices and sheets from a density map (structure tensor, helix and sheet hunters, cylinder fits) | 6.2k | Mixed: the CVC notice; some files have only an author banner or nothing | CGAL kernel only (LGPL); XmlRpc++ | Secondary Structure Elucidation, tab 0 | No. `cvc::find_secondary_structures` is tab 1, the surface-based method |
| SweetMesh | A hex-mesh structure (Jacobians, normalisation, `.rawhs` I/O) and LBIE's run with aspect-ratio histograms | 2.7k | LGPL-2.1 on 12 files, none on 9 | Qt Core, Boost | Tools › LBIE Mesh Generation | Part: LBIE meshing and quality statistics; no hex-mesh structure |
| Reconstruction | Iterative B-spline electron tomography (CIMOR) from SPIDER tilt series | 16.5k | "All right are reserved by The National Key Lab of Scientific and Engineering Computing, CAS" | FFTW3 (GPL), Qt3 APIs | ET Reconstruction | Only filtered back-projection (`vol_back_project`), a different algorithm |
| SDF2D (in VolumeGridRover) | A 2D signed distance image of contour polygons | 762 | UT LGPL-2.1 | CGAL Spatial_searching and Partition_2 (GPL) | None (its callers are commented out) | No; `cvc::planar` and nanoflann are the parts it needs |
| SweetLBIE | Experimental octree hex meshing | 485 | None | SweetMesh | None | No |
| PEDetection | Pulmonary-embolism detection on contrast CT (Sangmin Park, 2006): materials, lungs, heart, arteries, clots | about 54k compiled (plus 9.6k of tables) | UT LGPL-2.1 | Pre-standard STL (`<map.h>`) | None | No |
| ContourFilter | Reconstruct `.ser` reading and spline smoothing by simulated annealing | 5.3k | None; an Eberly snippet is "All Rights Reserved" | STL | None directly | The `.ser` reader (`cvc/geometry/contours.h`) |

**Already in libcvc,** so not in the table:
- the four filters: anisotropic diffusion, bilateral, contrast enhancement and GDTV
  (`cvc/volume/voxels.h`, `src/cvc/processing/`, the CLI's `bilateral`, `contrast`, `anisotropic`
  and `gdtv`). Only bilateral had the job form (`volume_filter_kind`, `volume_filter.h:52`), and
  its header said the other three "come later". V0 gave them theirs;
- LBIE's quality improvement (`cvc::geometry::quality_improve`, the stepper in `Octree`), which the
  legacy dialog calls with `GEO_FLOW` only;
- VolMagick, LBIE, FastContouring, libcontour, the contour tree, SDF, libiso, volren, the
  skeleton, both tilers, GenSeg, PocketTunnel and SecondaryStructures.

### 2.3 Left behind

- **Reconstruction,** unless its authors relicense it. It is "All right are reserved" with no
  licence, links GPL FFTW, and blocks on `getchar()` about 31 times. Its build is off and its
  `add_subdirectory` is misspelled (`src/CMakeLists.txt:166-170`). libcvc keeps filtered
  back-projection. §9 item 4.
- **ContourFilter,** as the VolumeRover plan's §8 already says.
  - It is Justin Kinney's `reconstruct2contourtiler` (2009). It reads Reconstruct `.ser` series,
    fits splines to each contour, smooths them by simulated annealing and writes the tiler's input.
  - No VolumeRover2 dialog calls it; ContourTiler only includes one of its headers.
  - Kinney's files carry no licence, and he is not a CVC author, so §9 item 3's precedent does not
    cover them. It also keeps its settings in a `Controls::instance()` singleton and holds a
    point-to-segment distance marked "All Rights Reserved" (Eberly, `parameter.cc:46-52`).
  - libcvc already reads `.ser` (`cvc/geometry/contours.h`). If smoothing noisy traced contours is
    wanted, it would be a clean implementation of the method in P10's Contours tab, not a port.
- **SweetLBIE:** an unfinished start on Zhang and Bajaj's adaptive hex meshing, which LBIE, already
  in libcvc, implements in full.
  - `generateOctree` lays a uniform grid, and three of its four subdivision patterns
    (`dividePattern_edge`, `_face` and `_all` in `meshGeneration.cpp`) are empty.
  - VolumeRover2 includes its header but never calls it.
  - It has no licence header (§9 item 3 would cover that), and it names constructors as types,
    which modern compilers reject.
- **The multi-tile server** (XML-RPC to a display wall, §1.10), **the ray tracer** (it writes
  `test.cnf` to the current directory; `volren` and "Save image" cover the need), **SegServ** (the
  XML-RPC front of the segmentations) and the **Slice Rendering** dialog (a stub).
- **The Qt and GL code:** the Histogram widget (`tf_editor`'s histogram layer replaces it),
  QGLViewer, ColorTable2, the renderers.

### 2.4 The ports

Each one follows the VolumeRover plan's §1 and ends with its component (§1 rule 1). The numbering
continues from that plan's P11.

- **V0 Filter jobs (S). Built.** Contrast enhancement, anisotropic diffusion and GDTV are
  `volume_filter_kind` values with their steppers and parameter tables, as bilateral is. A
  `volume/volume_filters.ari` component drives all four. VolumeRover2's defaults (anisotropic 20
  iterations; bilateral 200, 1.5, 2; contrast 0.95; GDTV exponent 1, λ 1e-4, one iteration, six
  neighbours) are the parameter tables' defaults. What landed:
  - the three filters are fields of the one `volume_filter_params` table (`contrast.resistor`,
    `anisotropic.iterations`, `gdtv.exponent`, `gdtv.lambda`, `gdtv.iterations`,
    `gdtv.neighbours`), so `volume.filter`, `cvc filter`, `pycvc.filter_volume` and
    `pycvc.volume_filter_job` take them as they take bilateral's;
  - each stepper is VolMagick's one-shot cut into rows. `volume_filter_test` checks each against
    the one-shot byte for byte, on float, byte, unsigned short and double volumes and on volumes
    one voxel thin, at step budgets 1, 64, random and 0. Both sides are built without FMA
    contraction;
  - the GDTV exponent is the dialog's: it calls `gdtvFilter(2 - exponent, λ, iterations,
    neighbourhood - 1)`, and its field defaults to 1.0 (`GDTVFilterDialog.cpp:52`). This plan's
    first draft said 2, which is q = 0;
  - `volume_filters.ari` views `data.<data>.volume`, runs `volume.filter-start`, and puts the result
    back with `data-put` (its Apply, or `volume-filters-step` from `on_tick`). Its demo is
    `examples/ariadne/documents/volume_filters.ari` on the Visible Human CT, with a golden
    (`src/cvcGL/test/golden/volume_filters.ari`) and an arigl-web case. The golden harness waits on
    a document's `golden.wait` key, for DSL jobs no node knows of;
  - the panel is written by hand; it moves to PP1's generated `volume_filter_*_params` units
    once PP1 lands.
- **V1 Tight Cocone (S–M).** The core is in libcvc; what is missing:
  - a public `cvc::tight_cocone` with a parameter table (robust, big-ball ratio, the two angles,
    cocone φ, flat φ, flatness). The skeleton's defaults already match (`cvc/geometry/skeleton.h:69-79`);
  - the surface extraction, a port of `wt_to_geometry` (`src/TightCocone/op.cpp:306-397`): the facets
    whose `cocone_flag` is set, oriented by the outside cell;
  - one shared triangulation step in place of the three copies in `skeleton.cpp`,
    `pocket_tunnel.cpp` and `secondary_structures.cpp`;
  - a `result_job` stepper, `cvc cocone`, `cocone.tight`, pycvc and `mesh/tight_cocone.ari`.

  The legacy library's volume path (boundary points from a volume through GVF and fast marching)
  comes second, on libcvc's `gradient_vector_diffusion` and segmentation code. The legacy robust
  mode never resets its bounding box between its two triangulations (`tight_cocone.cpp:602-700`);
  the A/B records whether that changes results and the port resets it.
- **V2 Surface curation (S),** on V1 and PocketTunnel's stable manifolds:
  - the merge-ratio clubbing (`smax.cpp:139-170`), `curate_tr`, and the count of pockets and
    tunnels to keep;
  - the result is a new geometry on the state tree, and the input stays;
  - the legacy dialog stores the merge ratio as an `int` (`CurationDialog.cpp:110`), and the port
    takes a `double`.
- **V3 Super-secondary structures (S),** on V1 and the same stable manifolds:
  - the segments come back as geometries on the state tree, not as `.off` and `.rawc` files in the
    current directory;
  - the legacy code reads flatness, cocone φ and flat φ but calls `tcocone` with constants
    (`supersecondary_structures.cpp:214-216, 332`). The port uses the parameters, and the A/B runs
    with the constants;
  - licence: 22 of its files have no header (§9 item 3).
- **V4 HLS surface (M).** HLevelSet, ported once for VolumeRover's HLS Surface and TexMol's T2
  (`mol.hls`):
  - the Xu–Li surface from points and the atoms' SES, each a stepper over the narrow band, with the
    iteration count a parameter;
  - the file-scope statics (`HLevelSet_Recon2.cpp:139-143`) go into a context, the 12 MB of stack
    arrays (`HLevelSet_SES.cpp:131`) onto the heap, and the `output.txt` write goes;
  - TexMol's copy (6.5k lines in #1090's count) and volrover's (10.7k compiled) differ. The port
    starts from a diff of the two and keeps the superset;
  - VolumeRover2's dialog computes the surface and then discards it (`CVCMainWindow.cpp:2027,
    2056-2072`). In arirov the result is a volume on the state tree, with its isosurface in the view.
- **V5 Multi-domain meshes (M),** MMHLS on V4 and `cvc::class_surfaces`:
  - a label volume gives per-label HLS volumes, then non-intersecting meshes with a gap;
  - no manifest files and no `.rawiv`-suffix stripping (`generateRawV/main.cpp:192-194`); inputs and
    outputs are state-tree data;
  - licence: 9 of its files have no header (§9 item 3).
- **V6 Level-set segmentation (L).** MSLevelSet, HigherOrderSegmentation and MultiphaseSegmentation
  as one family with one parameter table and a mode:
  - two-phase, higher-order and multiphase (n functions, 2^n phases);
  - **a CPU stepper first,** sliced by subvolume block, because the wasm rule needs one and the
    legacy code is CUDA only. The CUDA kernels come back as an option on the per-device cache
    (VolumeRover plan §9 item 5), with the CPU path as their A/B;
  - the legacy run writes `tmp/*Phi.rawiv` next to the input and takes the last volume opened
    (`CVCMainWindow.cpp:2782-2789, 2896-2905`). In arirov the input is the current volume and the
    level set is live data on the state tree, drawn on the slices as it evolves, as P9's labels are.
- **V7 Virus-map segmentation (L).** SegCapsid, SegMonomer and SegSubunit on P5 and P6's parts
  (gradient vector diffusion, critical points, multi-seed fast marching):
  - the XmlRpc++ parameter and result values become parameter tables and `result_job` results;
  - outputs are label volumes on the state tree, shown on the slices and as class surfaces, as P9's
    segmentation is. The legacy outputs were never loaded back;
  - the near-copies of `diffuse`, `criticalPnts` and `inout` across the three are already libcvc
    code, so they go;
  - the build hazards go with the port: a global `XmlRpcServer` and an `int main` compiled into
    libSegCapsid (`SegCapsid.cpp:87, 627`), and two definitions of `SingleCapsidSegment`;
  - licence: the commercial-licence clause and the Xcode stubs (§9 item 3).
- **V8 SecStruct (M).** Density-based helices and sheets (structure tensor, hunters, cylinder fits),
  beside the surface-based `cvc::find_secondary_structures`:
  - the CGAL kernel and number types are replaced with libcvc's own types;
  - its four `.rawc` outputs become geometries on the state tree;
  - it shares T5's component, `mesh/secondary_structures.ari`, as a second method, which is how
    VolumeRover2's dialog shows the two (tabs 0 and 1);
  - licence: mixed headers (§9 item 3).
- **V9 Hex meshes (S–M).** SweetMesh's hex-mesh structure, with its Jacobian quality measures and
  `.rawhs` reader and writer, as a `cvc::geometry` mesh kind. LBIE's hex output and its quality
  histograms show in `fe_controls`, which gives arirov the legacy LBIE dialog. Licence: 9 files with
  no header (§9 item 3).
- **V10 SDF2D (S, optional).** A 2D signed distance of contour polygons on `cvc::planar` and
  nanoflann, as a `grid-contours.sdf` action in P10's Contours tab. Its legacy callers are commented
  out and contour curation never existed in volrover, so this is new function, not parity. It waits
  until someone needs it.
- **V11 Pulmonary-embolism detection (L, behind a feasibility step).** Sangmin Park's PEDetection
  (UT LGPL-2.1 on every file, 2006, advisor Bajaj). Joe, 2026-11-28: "it would be nice to get this
  working if possible/practical".
  - **What it does** (`src/PEDetection/main.cpp`, `Control.cpp`, `VesselSeg.cpp`):
    1. anisotropic diffusion of the CT volume;
    2. gradient vectors, and GVF seed points;
    3. second derivatives;
    4. EM classification into eight materials from the histogram, seeded by the GVF points;
    5. `VesselExtraction`: the lungs, the heart, the pulmonary arteries traced by chains of maximal
       spheres from the heart outward, seed-point vessel extraction and skeletonisation;
    6. `PE_Detection` on the thickened vessel volume: clots are darker regions inside the
       contrast-filled arteries.
  - **What libcvc already has:** anisotropic diffusion, bilateral filtering, gradient vector
    diffusion, critical points and seed generators, fast marching, the skeleton, marching cubes and
    octrees.
  - **Where the new code is.** 53.7k lines are compiled.
    - The new algorithm is the EM classification (about 2k lines with its initialisation) and
      `VesselSeg.cpp` (26.8k lines, 215 member functions).
    - `TFGeneration.cpp` (7.8k) generates transfer functions for rendering, and `Control.cpp`
      (4.3k) is mostly debug image writers.
    - About 90k more lines of backups and alternatives (`VesselSeg_backup*.cpp`, `VesselSeg2D*.cpp`,
      `Skeleton_Org.cpp` and others) are not compiled.
  - **Why it needs a feasibility step:**
    - it is off in volrover's build ("doesn't work for most compilers") and uses pre-standard
      headers (`<map.h>`, `<iostream.h>`, `<fstream.h>`);
    - its compiled code has 207 `exit()` calls, keeps every parameter in a global (`*_gi`, `*_gf`,
      `VesselSeg_g`), picks its stages with `#define`s in `CompileOptions.h`, and writes
      intermediate files beside the input;
    - it reads the eight EM clusters by fixed index (lungs 1–2, soft tissue 3, muscle 4, vessels
      5–6; `Control.cpp`, around line 1000), as tuned on the 2006 Houston scans (512×512×181,
      clipped to `uchar`), so whether it generalises is unknown;
    - those scans are not available here.
  - **V11a, the feasibility step (S–M):**
    - build the legacy library and a driver in a goldens tool, as `tools/texmol-goldens` and the
      volrover goldens do, with only the fixes it needs to compile;
    - run it on public CT pulmonary angiograms that have expert PE masks (FUMPE or CAD-PE; check
      their licences before any is committed or fetched by CI);
    - report whether its classification, lungs, artery tree and detections are sensible, and how its
      detections compare with the masks.

    It ends in a report and Joe's go or stop. Nothing is ported before then.
  - **V11b–V11d, if it goes:**
    - V11b: the EM material classification as a `cvc::volume` job, useful beyond CT, with each
      cluster's role found from the data rather than by a fixed index;
    - V11c: the lung, heart and artery segmentation and the detection, as resumable steppers whose
      results are label volumes and a candidate list on the state tree, with an A/B against V11a's
      outputs;
    - V11d: `volume/pe_detection.ari` with a demo, and arirov's window for it. That window is new
      function, since VolumeRover2 never had one.
  - It is research code. Its component and docs say it is not for clinical use.

---

## 3. What is left of TexMol

The TexMol plan's §8 stands; this table is its status on 2026-11-27.

| Phase | Status | What is left |
|---|---|---|
| T0 decisions | Open | The library list, TexMol#7's relicensing, UT's rulings on `libmol` and `ElementInformation.h` (cyconv's D13) |
| M0 molecules | Part 1 built (#1427) | **Part 2:** the group tree, torsions, the writers, CLI, pycvc, intrinsics, the A/B against TexMol's parser, and **the molecular scene nodes** (ball-and-stick, impostor spheres, secondary-structure ribbons, surfaces; cyconv's SDK-15). arimol's view needs these first (§6) |
| T1 surfaces | Part 1 built (#1428, the blur density) | **Part 2:** the hydrophobicity and electrostatic densities, colour, the multilevel blur, the molecular surfaces, SAS, skin regions, F2D generation |
| T2 SES and pockets | Not started | HLevelSet is V4 here, ported once; Pocket on `cvc::sdf` and `cvc::iso` |
| T3 docking analysis | Not started | InterfaceStats, MolecularCharacteristics, the shared `cvc::mol` filters (needs T0's rulings) |
| T4 docking inputs and results | Not started | F2DockClient's parameter structs, `.inp` writers and `.out` parser |
| T5 tunnels and secondary structure | Cores built (#1439, #1440) | The CLI, pycvc, DSL intrinsics and a `mesh_lab` request for both, and their components (`mesh/pocket_tunnel.ari`, `mesh/secondary_structures.ari`) |
| T6 small pieces | Not started | Curvatures, ColorSurfaceByFunction, the DX volume reader |
| J0, J1 jobs | Not started | `cvc::jobs`, then the server fronts and `jobs.ari` (the job panel, §7.5) |
| R1–R3, C1 rooms | Not started | Many rooms, room features, science scenes, cyprty's rooms |
| F0–F4 F2Dock | Not started | `libf2dock`, its A/B, the docking service, `cvc::dock` |

The arisrvx stages S0, S1, S2, S3 and S5 are built (ARIADNE_SCRIPTING_HOSTS.md's phase table).

This plan adds two things to the TexMol plan:
- each T phase ends in a component in `mol/` (§1 rule 1), which is arimol's window for it;
- T2's HLevelSet is V4, shared with arirov.

The TexMol plan's open decisions (its §9 items 1, 2, 3, 5 and 6, and T0) stay open there.

---

## 4. Component collections

### 4.1 Today

- `src/cvc/ariadne/components/` holds 42 components and `INDEX.ari` in one flat directory. It is
  installed to `share/libcvc/ariadne/components`, embedded in every wasm host, and reached as
  `cvc://components/<name>.ari`.
- The `cvc` URI handler joins the whole path after `cvc://` to each search directory
  (`src/cvc/ariadne/uri.cpp:396-404`). So `cvc://components/volume/volume_grid.ari` already
  resolves, and `cvc://worlds/` is a second tree beside `components/`.
- `INDEX.ari` is data only: a `catalog:` list with one entry per component. Its one reader is the
  test `AriadneComponents.ShippedComponentsLoadClean` (`src/cvc/tests/ariadne_loader_test.cpp:5238-5316`).
- Unit names are one namespace across all imports; a clash warns (`loader.cpp:2753-2754`).
- Components that drive a node take the node as an arg since b2f545786 (`node`, `title`, `id`).
- Domain groups exist only as name prefixes: `volume_grid*`, `nav_lab_*`, `world_*`, `swarm_*`.

### 4.2 The design

- **A collection is a subdirectory:** `cvc://components/<collection>/<name>.ari`, with its own
  `INDEX.ari`. The top `INDEX.ari` lists the collections and the components still at the top.
- **What a collection holds:**
  - its components;
  - their demo documents, listed in their `INDEX.ari` entries' `demos:` (#1604 started that);
  - their tests and goldens;
  - their gallery entries.

  Its node types and intrinsics are in libcvc (§1 rule 3); the collection's `INDEX.ari` names them
  in each entry, so a document can check them with `requires:`.
- **Old paths keep working.** A component that moves leaves a one-line document at its old path
  that imports the new one. `import:` merges the imported units, so every document that names the
  old path still gets the same units. The forwarders go in a later major release.
- **Unit names.** Existing units keep their names. New units start with their collection's name
  where the plain name could clash (`mol_surface`, not `surface`).
- **The build and test changes:**
  - `ShippedComponentsLoadClean` walks the tree recursively and keys entries by relative path;
  - the wasm relink triggers' `file(GLOB … "*.ari")` become `GLOB_RECURSE`
    (`src/arigl/CMakeLists.txt:316, 357, 379-380`);
  - `install(DIRECTORY)` and Emscripten's embed are recursive already;
  - the `cvc` handler rejects `..` and absolute paths. It doesn't today (`uri.cpp:396-413`), and
    collections make relative imports between directories common.

### 4.3 The collections

| Collection | Moves into it | Comes from the ports |
|---|---|---|
| `volume/` | `volume_grid`, `volume_grid_segmentation`, `volren_controls`, `volslice_controls`, `transfer_function` | `volume_filters` (V0), `hls_surface` (V4), `level_set` (V6), `virus_segmentation` (V7), `multi_domain_mesh` (V5), `pe_detection` (V11) |
| `mesh/` | `mesh_tools` (with the skeleton), `fe_controls` (LBIE), `contour_tiler` | `tight_cocone` (V1), `curation` (V2), `super_secondary` (V3), `pocket_tunnel` and `secondary_structures` (T5, V8), hex meshes in `fe_controls` (V9) |
| `mol/` | — | the molecule view and its styles (M0 part 2), surfaces (T1), pockets (T2), characteristics and interfaces (T3), curvature and colouring (T6), docking (F3, from aridock) |
| `dev/` | `processes`, `py_console` | the state inspector and exec console (§7.3), `jobs` (J1, §7.5) |

The shell, worlds, nav, conference and vehicle components stay where they are. The mechanism
serves them later; moving them is not part of this plan.

---

## 5. arirov

### 5.1 What it is

- **A document,** `arirov/main.ari` (where it lives is §9 item 2), that:
  - opens with an empty main view and the File, View, Tools, Segmentation, Meshing and Help menus;
  - keeps the app's datasets on the state tree under its prefix: volumes, geometries, contour
    stacks, label volumes;
  - has a **data browser** window: each dataset with show and hide, rename, save and close, and the
    *current volume*, which every volume tool takes as its input (VolumeRover2's "Select Current
    Volume"; #1443's Source combo is the same pattern in `volume_grid`);
  - opens each tool as a window `include:`d from a collection, bound to the current dataset;
  - runs each tool as a lab request or a `-start` job, with its progress in the window and in a jobs
    window, and its result as a new dataset.
- **Running it:** `arigl arirov/main.ari`, `ariglx run arirov/main.ari` (with Python), an arigl-web
  page, and the shell's catalog once SH3 and SH6 are built.
- **volrover3's dev tools come with it:** the state inspector, the exec console with its process
  table, and the jobs window from `dev/`, and `py_console` where Python runs.

### 5.2 Parity with VolumeRover2

| VolumeRover2 | arirov | Status |
|---|---|---|
| File › Open, Save, Close | The data browser and `file_dialog` (§7.2), reading through the app's file handlers onto the state tree (§7.4) | Needs SDK-4 and §7.4 |
| File › Save / Load Property Map | Save and load the app's settings subtree | New, small |
| File › Save Image | "Save image..." in a view's right-click menu (#1533) | Built |
| File › Options | `viewer_settings`; the shell's Settings once built | Built in part |
| View › Select Current Volume | The data browser's current volume | Needs SDK-7 |
| View › Show Volume Grid Rover | `volume/volume_grid` with its Probe, Segmentation and Contours tabs | Built (P8–P10) |
| View › background colours, geometry view options, wire cube, clip to bounding box | `viewer_settings` (grid, axis, scene bounds), `mesh_tools` | Grid, axis and bounds built; the background colour, wire cube and clipping to the box are small additions |
| Tools › the four filters | `volume/volume_filters` | Built (V0) |
| Tools › Segment Virus Map | `volume/virus_segmentation` | V7 |
| Tools › Secondary Structure Elucidation | `mesh/secondary_structures`, both methods | T5's component, then V8 |
| Tools › Show Histogram | `tf_editor`'s histogram layer | Built |
| Tools › Surface Curation | `mesh/curation` | V2 |
| Tools › HLS Surface | `volume/hls_surface` | V4 |
| Tools › Pocket/Tunnel Detection | `mesh/pocket_tunnel` | T5's component |
| Tools › Skeletonization | `mesh/mesh_tools`' Skeleton section | Built (P2) |
| Tools › Tight Cocone | `mesh/tight_cocone` | V1 |
| Tools › Contour Tiler | `mesh/contour_tiler` and `volume_grid`'s Contours tab | Built (P4, P10) |
| Tools › LBIE Mesh Generation, Quality Improvement | `mesh/fe_controls` | Built; hex meshes are V9 |
| Tools › SuperSecondary Structures | `mesh/super_secondary` | V3 |
| MMHLS › GenerateRawV, GenerateMesh | `volume/multi_domain_mesh` | V5 |
| MSLevelSet, HOLevelSet, MultiPhase Segmentation | `volume/level_set` | V6 |
| GenSeg (VolumeGridRover's segmentation) | `volume_grid`'s Segmentation tab, with paper mode | Built (P5, P6, P9) |
| ET Reconstruction | — | Left behind (§2.3) |
| Ray Traced Image, Sync Multi-Tile Server | — | Left behind (§2.3) |
| Help › About | A window from the app's `meta:` | Small |

**From volrover3** (cyconv's Appendix B): the grid, axis and bounding box; camera settings and
rigs (`camera_controls`); SDF and isosurface (the `sdf` data generator, `mesh_tools`); procedural
shapes; geometry properties; the transfer function dock; the Python console and its jobs; the
state tree, dashboard and thread monitor (§7.3); the seven example scripts under `ariglx`; and
settings as a state subtree.

### 5.3 Phases

- **AR1 arirov v1 (M):** the document from what master has today, plus V0 and T5's components:
  - the volume grid, volume rendering and slicing, the transfer function and the filters;
  - the skeleton, both tilers, LBIE and its quality improvement, GenSeg;
  - pockets, tunnels and secondary structures;
  - the data browser, File › Open and the dev tools;
  - standalone in `arigl`, `ariglx` and arigl-web, with a golden of its main window and a scripted
    run of each tool.
- **AR2 arirov v2 (rolling):** each V port adds its window when it lands. The parity table above is
  a test: a scripted run opens each tool on a fixture and checks its result against the core's own
  test.
- **AR3 arirov in the shell, and the retirement (M):**
  - arirov in the shell's catalog, mounted, with its scene in the main view (SH3, SH6), its keys and
    pointer granted (SH7), and its icon (SH10);
  - once AR2's parity holds, VolumeRover2 and every legacy library with a libcvc port retire from
    volrover's master, with #123's VolUtils leftovers. That is P11's last item. volrover3 retires
    when arirov passes its checklist too.

### 5.4 What arirov leaves out on purpose

The libraries in §2.3 and their menu items; Qt, VTK and PySide6; `vrhost.main_window()`, which has
no meaning without Qt (its two scripts are rewritten against the document, as cyconv §7 planned).

---

## 6. arimol

### 6.1 What it is

- **A document,** `arimol/main.ari`, built the same way as arirov: the app's molecules, surfaces
  and volumes on the state tree, a data browser, a main view, and windows from `mol/`, `volume/`,
  `mesh/` and `dev/`.
- **Its view needs the molecular scene nodes first** (M0 part 2): ball-and-stick, impostor spheres,
  secondary-structure ribbons and molecular surfaces, as cvcGL node types in libcvc (cyconv's D10
  recommended libcvc; §1 rule 3 makes it necessary).
- **Its file formats:** molecules (`.pdb`, `.pqr`, `.xyz`, `.xyzr`; `.pdb1` and `.pts` in M0 part 2;
  `.goa` if T0 keeps it), surfaces (libcvc's geometry formats), volumes (libcvc's, plus `.dx` from
  T6). `.c2c` stays behind (the TexMol plan's §2.4).

### 6.2 Parity with TexMol

TexMol's main window has 13 menus and 80 actions, and 14 of its dialogs are blank stubs in its Qt6
build (cyconv §2.5). arimol's parity list is what TexMol computes and shows, not its widgets:

| TexMol | arimol | Status |
|---|---|---|
| Molecule rendering: ball-and-stick, impostors, secondary structure, surfaces | `mol/molecule` styles over the molecular nodes | M0 part 2 |
| Surfaces: blur, SAS, skin, area and volume | `mol/surface` | T1 part 2 |
| HLS surface (SES) | `volume/hls_surface`, with an atoms input | V4 / T2 |
| Pockets (morphological) | `mol/pockets` | T2 |
| Pocket-Tunnel | `mesh/pocket_tunnel` | T5's component |
| Secondary structures | `mesh/secondary_structures` | T5's component |
| Molecular characteristics, interfaces, the docking filters | `mol/characteristics` | T3 |
| Curvatures, Coulomb colouring | `mol/surface` options | T6 |
| Volumes (`.rawiv`, `.mrc`, `.dx`) | `volume/` | Built, `.dx` is T6 |
| Lighting, viewing parameters, image and movie save | `stage_lighting`, `viewer_settings`, "Save image" and "Save video" (#1533) | Built |
| F2Dock (the six dialogs) | aridock's documents, mounted (cyconv D8) | F3, F4 |
| Download PDB | A fetch into the app's data through the shell's network policy | SH11 |
| GB and PB energy | — | Only if rewritten (TexMol plan §2.3) |
| NURBS | — | Left behind with TexMol's renderers |

### 6.3 Phases

- **AM1 arimol v1 (M–L):** molecules with their styles, surfaces, HLS, pockets, tunnels and
  secondary structures, volumes, the data browser and the dev tools; standalone first. Needs M0
  part 2, T1 part 2, V4 and T5's components.
- **AM2 arimol analysis and docking (M):** T3, T4 and T6's windows; aridock mounted for docking,
  locally and against the F2Dock service (F3, F4).
- **AM3 arimol in the shell, and the retirement (S–M):** as AR3; then TexMol's Qt app retires from
  its master.

---

## 7. Ariadne pieces the apps need

### 7.1 The shell

Built: SH0, SH1 (`mounts:`, `app-open`, budgets, `shell.ari`), SH2, SH4 and SH8. ARIADNE_SHELL.md's
status line still says nothing is built; its next edit should fix that. Both apps need, for AR3 and
AM3:
- **SH3:** a mounted document's scene. Today a mount gets no `scene:`, `data:` or `keys:`
  (`app_runtime.cpp:1176-1184`), and both apps are scenes.
- **SH6:** the main view for the front application.
- **SH7:** grants, so a mounted app gets keys and the pointer.
- **SH9:** `contributes:`, for the apps' menus in the shell's menu bar.
- **SH10:** the catalog with icons (`meta.icons`), and the shell in place of arigl's gallery.

Standalone use needs none of them, which is why AR1 and AM1 come first.

### 7.2 Widgets

The `.ari` schema's built-in widgets include `menubar`, `menu`, `window`, `tabs`, `list`, `plot`,
`heatmap`, `tf_editor` and the inputs, and `viewport` and right-click menus exist too. The apps
also need, using cyconv's SDK numbers:
- **SDK-7:** `tree` and `table`, with selection bound to state. The data browser and the dev tools
  are built on them.
- **SDK-4:** `file_dialog` (a native listing within the document's access; the browser's file picker
  and OPFS), `message_box` and modal windows.
- **SDK-3:** a toolbar, a status bar, menu shortcuts and icons.
- **SDK-5:** `input_float` with step, decimals and suffix, and an editable `combo`.
- **SDK-8, SDK-9:** state and exec introspection for the dev tools.

### 7.3 Dev tools

cyconv §5's state inspector (a `tree` of the state with a property `table`, typed write-back) and
exec console (DSL input and `processes.ari`'s process table) are `dev/` components, written directly
on SDK-7 instead of converted from volrover3's widgets. `py_console` (scripting P2) and
`processes.ari` (SH0) exist already.

### 7.4 Opening files

Components never load files (§1 rule 2), so the app does: File › Open runs `file_dialog`, reads the
file through the app's handlers (volume, geometry, contour, molecule), and puts the result on the
state tree as a dataset, where the data browser lists it. `data-put` (#1446) does the state-tree
half today. What is new is a verb that reads a path the user picked into a dataset, natively and
in the browser (OPFS or an upload), within the document's file access.

### 7.5 Parameter panels from parameter tables

Every port has a `cvc::param_table`, and P0b generates its state keys, properties, CLI, pycvc and
DSL from it. **PP1** adds one more generator: a parameter table's widget list as an `.ari` unit
(`slider_float` or `input_float` with its range and step, `checkbox`, `combo` for an enum,
`input_text`), bound to the port's parameter keys. Each port's window then starts from its
generated panel, with its run and stop buttons and its result picker added by hand. That makes the
windows in §5 and §6 small. The jobs window is `jobs.ari`, planned by the TexMol plan's J1.

### 7.6 Documentation fixes found on the way

- ARIADNE_SHELL.md's status line ("Nothing in it is built").
- ARIADNE.md §6.5 still says most components use a fixed node id, and its table misses seven
  components that `INDEX.ari` lists.
- ARIADNE.md says the provider registry is process-wide, while `ariadne.h` says it is per app.

---

## 8. Phases

Sizes as in the VolumeRover plan: S is a PR or two, M a few, L a short series.

| Phase | Size | What lands | Needs |
|---|---|---|---|
| CC1 Collections | S–M | §4.2: `volume/`, `mesh/` and `dev/` with forwarders at the old paths, per-collection `INDEX.ari`, the recursive test and globs, the resolver's containment check | — |
| PP1 Parameter panels | S–M | §7.5's generator, with a test that every port's table yields a panel that loads clean | — |
| SDK-7 | L | `tree`, `table`, selection bound to state, in ImGui, FTXUI and tvision | — |
| SDK-4 + open | M | `file_dialog`, `message_box`, modal windows, §7.4's open verb | — |
| SDK-3, SDK-5 | M | Toolbar, status bar, shortcuts, icons; `input_float`, editable `combo` | — |
| DT1 Dev tools | M | §7.3's state inspector and exec console in `dev/` | SDK-7, SDK-8, SDK-9 |
| V0 Filter jobs | S | §2.4. Built | — |
| V1 Tight Cocone | S–M | §2.4 | — |
| V2 Curation | S | §2.4 | V1 |
| V3 Super-secondary structures | S | §2.4 | V1, §9 item 3 |
| V4 HLS surface | M | §2.4; also TexMol's T2 HLevelSet | — |
| V5 Multi-domain meshes | M | §2.4 | V4, §9 item 3 |
| V6 Level-set segmentation | L | §2.4, CPU first | — |
| V7 Virus-map segmentation | L | §2.4 | §9 item 3 |
| V8 SecStruct | M | §2.4 | T5's component, §9 item 3 |
| V9 Hex meshes | S–M | §2.4 | §9 item 3 |
| V10 SDF2D | S | §2.4, when wanted | — |
| V11a PEDetection feasibility | S–M | §2.4: the legacy build, runs on public CT angiograms, a report | — |
| V11b–V11d PEDetection | L | §2.4 | V11a and Joe's go |
| T5b | S | T5's CLI, pycvc, intrinsics, `mesh_lab` request and components | CC1 |
| M0b | L | M0 part 2 with the molecular scene nodes | — |
| T1b | M–L | T1 part 2 | — |
| T2–T4, T6, J0, R, F, C | | As the TexMol plan's §8, each ending in its `mol/` component | As there |
| AR1 arirov v1 | M | §5.3 | CC1, PP1, SDK-7, SDK-4 + open, DT1, V0, T5b |
| AR2 arirov v2 | rolling | §5.3, a window per V phase | AR1 and each V phase |
| AR3 arirov in the shell, retirement | M | §5.3 | AR2's parity, SH3, SH6, SH7, SH10 |
| AM1 arimol v1 | M–L | §6.3 | CC1, M0b, T1b, V4, T5b, AR1's data browser |
| AM2 arimol analysis and docking | M | §6.3 | AM1, T3, T4, T6, F3 |
| AM3 arimol in the shell, retirement | S–M | §6.3 | AM2, as AR3 |

**Parallel tracks.**
- CC1, PP1, SDK-7, SDK-4, V0, V1, V4, V6, V11a and M0b share no files and can start at once.
- The V ports run as separate sessions, as P1–P10 did. V2 and V3 follow V1; V5 follows V4.
- AR1 is the first app milestone. It mostly assembles what exists, so it shows the app shape early
  and makes every later port land as a window in a running app.
- The shell phases (SH3, SH6, SH7, SH9, SH10) follow ARIADNE_SHELL.md's own plan, and AR3 and AM3
  wait for them.

---

## 9. Decisions for Joe

**Answered 2026-11-28.** Joe accepted every recommendation below ("Yes, all of them"). He then
moved PEDetection out of item 5: "this was pulmonary embolism detection by sangmin park, it would be
nice to get this working if possible/practical". It is V11 (§2.4).

1. **Build the apps from component collections, not by conversion.**
   - cyconv's §7 and §8 build arirov and arimol by converting their Qt UIs with a probe and a
     bridge, and wait on its D9–D14. Nothing of it is built.
   - This plan builds them as documents over collections. Every feature needs a new window over a
     new `cvc::` API anyway, VolumeRover2 is the Qt3/Qt4-era code cyconv's D6 excludes, and
     volrover3's ten dialogs mostly exist as components already.

   **Recommended:** collections. cyconv keeps its converter for other Qt apps if one comes up. Its
   dev tools and SDK items become libcvc work here (§7.2, §7.3). Its D9 (Python extension modules)
   and D11 (cp312) are moot for these two apps. D10 (molecular nodes in libcvc) is adopted, and
   D12–D14 stay with F2Dock.
2. **Where the apps live.**
   - cyconv's D3 put them in their legacy repos: arirov in transfix/volrover and arimol in
     transfix/TexMol.
   - With no native code (§1 rule 3), an app is a few documents. Kept in libcvc, as
     `cvc://apps/<app>/main.ari` beside `cvc://components/` and `cvc://worlds/`, they are tested by
     the same CI as the components they use, ship in one release with them, and are in the shell's
     catalog with no extra package.

   **Recommended:** libcvc. The legacy repos keep their legacy code until AR3 and AM3 retire it.
3. **The licences of the files with no header.** The precedent is Joe's ruling of 2026-11-21 on
   SecondaryStructures ("It is also LGPL-2.1 like the rest"). The question is whether the same holds
   for:
   - SuperSecondaryStructures (22 files);
   - MMHLS (9 files);
   - SweetMesh (9 files);
   - SecStruct's helix and sheet hunters and cylinder fits;
   - the virus segmentations' CVC notice with its commercial-licence clause and Xcode stubs.

   **Recommended:** yes, each recorded per library as SecondaryStructures' was, since they are UT CVC
   code in the same repository. V3, V5, V7, V8 and V9 wait for this.
4. **ET Reconstruction.** It is "All right are reserved" by CAS, with no licence, and links GPL FFTW.
   **Recommended:** leave it behind unless its authors relicense it. libcvc's filtered back-projection
   stays.
5. **What else stays behind:** ContourFilter, SweetLBIE, the multi-tile server, the ray tracer and
   SegServ (§2.3). **Recommended:** as listed. PEDetection was on this list; it is now V11.
6. **The level-set segmentations' CUDA.** **Recommended:** a CPU stepper first, for wasm and the A/B,
   then the CUDA kernels as an option on the per-device cache.
7. **The collections.** **Recommended:** domain collections (`volume/`, `mesh/`, `mol/`, `dev/`) that
   both apps assemble, with forwarders at the old paths. The alternative is one collection per app
   (`volrover/`, `texmol/`), which would split shared windows such as the volume grid between them.
8. **Names:** arirov and arimol (from cyconv's D3); the collections `volume`, `mesh`, `mol` and
   `dev`; `cvc::tight_cocone` and `cvc cocone`; the phase names CC1, PP1, V0–V11, AR1–AR3 and
   AM1–AM3.
9. **Where to start.** **Recommended:** CC1, PP1, V0, V1 and M0b first, in parallel sessions, then
   AR1 as soon as CC1, PP1 and the two widget phases land.
