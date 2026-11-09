# TexMol Libraries, libf2dock, and arisrv as a Many-Room Compute and Scene Server — Roadmap (sketch)

**Status:** sketch, 2026-11-09. Nothing has landed. §9 lists the decisions that need Joe.
**Asked for (Joe, 2026-11-09):** port TexMol's legacy libraries; port the F2Dock server to arisrv
and Ariadne documents, with TexMol talking to it; consider conference mode for this; make arisrv
general for TexMol and VolumeRover compute tasks; and load scientific visualizations and digital
twins in different rooms with different scenes, in the conference mode cyprty uses.
**Based on:**
- libcvc `origin/master @ 016a2038`;
- transfix/TexMol and transfix/F2Dock at their default branches (F2Dock `20e9f1d`), read
  2026-11-09;
- cy-pca/cyprty (deployed relay: arisrv 3.7.10, room `standup`);
- cy-pca/cyconv `docs/plan/PLAN.md` revision 2.

**Companions, and how this fits with them:**
- [VOLROVER-LIBRARY-PORTS-ROADMAP.md](VOLROVER-LIBRARY-PORTS-ROADMAP.md) (merged in #1074). Its §1
  rules govern every port here too:
  - no GPL code;
  - no singletons anywhere in libcvc;
  - one API shape pycvc wraps;
  - parameters and data on the state tree;
  - DSL intrinsics;
  - the test matrix, including the A/B against what is replaced;
  - wasm from the first PR.

  This plan does not repeat them; §1 lists only what it adds.
- [ARIADNE_SCRIPTING_HOSTS.md](ARIADNE_SCRIPTING_HOSTS.md) §6.5: `arisrvx`, stages S0–S7 (S0 is
  built, #1046).
- [ARIADNE_WORLDS.md](ARIADNE_WORLDS.md): rooms (§10, RM1–RM2), features (§11, CF1–CF2), sections
  and assets by hash (§3, §12, AD1–AD2), persistence (§8, WD8).
- [ARIADNE_CONFERENCE_ROADMAP.md](ARIADNE_CONFERENCE_ROADMAP.md),
  [CONFERENCE_SHARING.md](CONFERENCE_SHARING.md) (named streams SH0),
  [ARIADNE_DATA_ROADMAP.md](ARIADNE_DATA_ROADMAP.md) (remote data sources).
- **cyconv's PLAN.md** owns the apps:
  - **arimol** (TexMol as Ariadne documents), **arirov** (VolumeRover) and **aridock** (F2Dock);
  - decisions D1–D14;
  - the F2Dock server design (§9.3): `arisrvx --python -m f2dock_server`, one service layer
    behind both `room-call` exports and FastAPI `/v1/*`, `dock.jobs.<id>.*` published and scoped
    per principal, worker processes, sqlite.

  **This plan is the libcvc side of that work,** and it keeps D8. Two things in PLAN.md are stale:
  it says arisrv has no `--doc` and that #1026 is open (PLAN.md:1108, 1134). S0 is built, and
  #1026 merged on 2026-11-06.

---

## 0. Summary

- **TexMol** (§2) is about 714k lines, but most of it is vendored or duplicates libcvc. Its
  science is about 68k lines of UT-LGPL headless C++, and Qt appears only in the GUI libraries.
  - **Port** in this order:
    1. molecules: PDBParser and GOAFileTypes;
    2. the Blurmaps surface engine, which also generates F2Dock's F2D input;
    3. HLevelSet's solvent-excluded surface;
    4. Pocket, rebuilt on `cvc::sdf` and `cvc::iso`;
    5. the docking-analysis set: interface statistics, molecular characteristics, and the clash,
       desolvation, Lennard-Jones and residue-contact filters, of which TexMol and F2Dock each
       carry a copy, so they are ported once and shared;
    6. the F2Dock client's Qt-free parameter and result code.
  - **PocketTunnel and SecondaryStructures** follow on the VolumeRover plan's shared cocone core.
  - **Leave behind:**
    - the renderers and GUI;
    - the restored copies of libraries libcvc already has;
    - code with licence blockers: Decimation's SHASTRA and IMSL code (missing from TexMol's
      notices), C2C, levmar, KIFMM3D, FastSummation;
    - the dead code.
  - **The GB and PB solvers** come only if rewritten without NFFT, PETSc and the GPL FMM.
  - **On the server,** each port becomes a job kind, matching TexMol's own batch commands (about
    45 `TexMol -<cmd>` modes).

- **F2Dock** (§3) needs more than a move: today there is no engine library and no real server.
  - **The engine** is globbed into the executable, with all defaults inside `main()`.
    - It keeps its work queues in globals, so two jobs in one process corrupt each other.
    - It calls `exit()` 55 times, including on a malformed input. One bad job kills the host.
    - It mutates its own inputs. The multi-state goldens bake that in.
  - **The XML-RPC "server" never calls the engine.** It writes a shell script that runs the
    F2Dock, GB-Rerank, MolSurf and PDB2PQR executables, and runs it inside `Submit`, blocking.
    - Its job markers race, so one job's finish can mark another's done.
    - A failed run reports COMPLETED.
    - A client's parameter text can name any file the server user can read or write.
  - **The plan follows cyconv's D12:**
    1. bug fixes in place (F0);
    2. then `libf2dock`, a reentrant library with a context, a parameter table, progress by
       rotation, cancel, and results by value (F1);
    3. an A/B against the legacy executable (F2);
    4. then the service on the job layer (F3);
    5. `cvc::dock` inside libcvc later (F4).
- **arisrv becomes a many-room server** (§4) that hosts **scene rooms** and **compute services**
  side by side.
  - **Today:** one room per process. Its write gate, chat, area of interest and holon are all
    built for one room. Server documents (S0) exist, but browsers drop what they publish, and a
    room's scene carries only primitives and meshes.
  - **What this plan adds:**
    - **Many rooms on one relay,** each with its own world document (the worlds plan's RM1–RM2,
      brought forward).
    - **Room features**, so a room can be a walk-around conference room (cyprty), a workspace
      with no avatars or media, or both (CF1, and cyconv's SDK-19).
    - **Room scenes rich enough for science.** Volumes, molecules, `data:` sources, skeletons and
      tiled surfaces go into the room wire format as sections with assets by hash (AD1). Digital
      twins are live data sources bound into a room's scene.
    - **One generic job service, `cvc::jobs`,** that F2Dock docking, the VolumeRover ports
      (skeleton, segmentation, tiling) and the TexMol computations all use. It has submit,
      status, progress, cancel, paged results and durable records. It runs in-process for libcvc's
      own sliced jobs and in worker processes for engines that aren't safe in the host yet.
    - **The same service in three forms:** `room-call` exports for native and browser clients
      (S2), FastAPI for HTTP (S1, cyconv's D8), and DSL intrinsics in server documents, so a room
      can run a pipeline itself.
- **TexMol talks to the server** as arimol, through the room: subscribe, submit, watch progress,
  and see results appear in the room's scene for everyone in it.
  - The legacy TexMol `F2DockClient` (XML-RPC, a 37-argument `Submit`) gets a shim only if the
    Qt app has to keep working (§9 item 4).
- **Order** (§8):
  - The VolumeRover plan's P0a (libcvc under the app) comes first, because many rooms in one
    process is exactly where process-wide state breaks.
  - Then the arisrvx stages the services need: S1, S2, S5, S6, as cyconv's D14 orders them.
  - Then rooms, room features, the job service and the scene format, in parallel with the library
    ports.
- **Decisions for Joe** (§9): native C++ job service vs Python-only, rooms per process, the legacy
  F2Dock client, MolSurf / PDB2PQR in-process, the arisrvx order, and names.

---

## 1. What this plan adds to the VolumeRover plan's rules

Every port here follows the VolumeRover plan's §1 as written. Five additions:

1. **Molecular types in libcvc.** Molecules, quadrature points, F2D / PQR / PDB readers, and
   molecular scene node types go in libcvc, as cyconv's D10 (SDK-15) recommends. Then arimol,
   aridock, the server and the browser share one implementation. Readers are I/O handlers
   registered on the app, as the VolumeRover plan's §1.3 requires.
2. **Long work is a `result_job`,** as the VolumeRover plan's P0b defines it. **A job that can run
   on a server is also a `cvc::jobs` kind** (§5):
   - its parameter table is the job's spec;
   - its progress and phase are the job's;
   - its result struct is what the job stores.
3. **Engines that can't yet run safely in the host run in worker processes.** Legacy F2Dock exits
   on bad input and holds process-wide queues, so until `libf2dock` passes the no-singletons scan
   and the bad-input tests (T5, T6), the service runs it out of process. The job layer makes both
   look the same to clients.
4. **The server accepts data, never paths.** Inputs arrive as blobs by sha256 (cyconv's SDK-17) or
   come from the room's own assets. A job never names a server-side file. That closes the legacy
   server's read-and-write-anything hole.
5. **The A/B row (T16) compares against the legacy program, not only CGAL.**
   - `libf2dock` against the legacy F2Dock executable;
   - each TexMol port against its TexMol original;
   - with the same split between exact agreement and agreement by tolerance.

---

## 2. TexMol's libraries

### 2.1 What TexMol is

transfix/TexMol (master `e621ebc`) is UT CVC's molecular visualization and computation package,
in a Qt6 modernization fork. Each library has `src/<Lib>` and `inc/<Lib>`.

**Size.** About 714k source lines, mostly not TexMol's own science:
- a 262k-line lookup table in `LBIE_lib`;
- glew (35k), three copies of `glext.h` (about 21k) and log4cplus (17k);
- a group of "compat" libraries restored after an earlier modernization phase deleted them, which
  libcvc already provides: Geometry, the volume and geometry file types, SDF v2, LBIE, libcontour,
  contouring, the contour tree, and XmlRpc++.

**Qt** appears only in DataManager, the app, ColorTable2, Histogram, QGLViewer and the F2Dock
dialogs. The compute libraries are headless.

**Batch mode.** The app has about 45 commands (`TexMol -<cmd>`, `Server.cpp:156-441`), among them
`-getPockets`, `-getSurfaceUsingHLS`, `-getMolecularCharacteristics` and `-blur`. They are the
natural list of job kinds (§5).

**What works in the Qt6 code**, judged from the code, with nothing run:
- **Wired:** pockets, HLS pockets, Pocket-Tunnel, secondary structures (with CGAL and LAPACK), and
  the F2Dock client.
- **Stubbed:** Construct Surface, Construct Volume, depth-coloured volume, curvatures, area and
  volume, and the scripting dialogs.
- **Unavailable by default:** GB energy (needs NFFT) and PB energy (needs PETSc).

### 2.2 What to port

Each lands as the VolumeRover plan's ports do: a headless `cvc::` core, CLI, pycvc, DSL
intrinsics, a job kind, and Ariadne components, wasm included.

| Library | Lines | Licence | What it gives libcvc | What has to change on the way in |
|---|---:|---|---|---|
| **PDBParser** + **GOAFileTypes** | 12.4k | UT LGPL-2.1 | Molecules: atoms in groups, residues, chains, radii, charges, torsions; PDB, PQR, XYZ, XYZR and PTS I/O. Every other port needs it (M0 in §8) | The parser's file-level statics (`GOAFileIO.cpp:42-45`, `:210-212`) and the self-registering file types (`PDBFile.h:47`) go: readers become I/O handlers on the app |
| **Blurmaps**, with **Summation**, **DPG**, **SphericalPatchIntersections** | 15.6k | UT LGPL-2.1 (DPG's cuckoo hash: permissive, attribution) | Gaussian-blob density maps and multilevel blurring; molecular surfaces; SAS; skin regions; area and volume; interface atoms and surfaces; curvature grids; **F2D generation for F2Dock** (`F2dGenerator`) | Debug files written to the current directory go (`BlurMapsDataManager.cpp:680`, `GOABlur.cpp:52`, `MultilevelBlur.cpp:41,76`, …). Output becomes `cvc::volume` and `cvc::geometry`. DPG's spatial grid may give way to nanoflann |
| **HLevelSet** | 6.5k | UT LGPL-2.1 | Solvent-excluded surfaces by a higher-order level set with B-spline coefficients | File-scope globals (`HLevelSet_SES.cpp:77-82`) go into a context. `iterationNum` is `float` there but `extern int` in `MainWindow.cpp:159`, which is undefined behaviour. Remove its `output.txt` writes |
| **Pocket** | 0.15k | UT LGPL-2.1 | Pockets by morphological closing on signed distance fields | Rewritten on `cvc::sdf` and `cvc::iso`, about 60 lines, with no file round trips (today every step goes through a file, `Pocket.cpp:47-74`) |
| **Docking analysis:** **InterfaceStats**, **MolecularCharacteristics**, and the filters **fast-clash**, **fast-hydro** (pseudo-Gsol), **fast-LJ**, **fast-resCont** | 11.8k | UT LGPL-2.1, plus files with no header; `ElementInformation.h` is UT GPL in TexMol (TexMol#7 relicenses it) | CAPRI criteria; interface area, volume, gap, planarity and circularity; residue contacts; per-molecule statistics; the octree filters and scores | **F2Dock carries its own copies of the same filters.** Port them once, as `cvc::mol` filters that `libf2dock` and the analysis both use (§3.2). pthreads become the app's pool; 39 `exit()`s become exceptions. The element table waits on cyconv's D13 |
| **F2DockClient's Qt-free core** | about 6.5k | UT LGPL-2.1 | The docking, rerank, F2D-generation and quad-generation parameter structs, the `.inp` writers, and the `.out` result parser (`summaryResults.cpp`), so legacy results and inputs load into arimol | The XML-RPC transport, the Qt dialogs and the "Nagger" polling thread stay behind; the job service replaces them (§5). Fix on the way: `dockControl.cpp:416` checks the wrong pointer; `return NULL` into `std::string` (`f2dockMasterControl.cpp:269-281`) |
| **PocketTunnel** | 5.0k | UT LGPL-2.1 | Pockets, tunnels and voids by stable manifolds, with surface curation | Needs the VolumeRover plan's `cvc::cocone` core on Qhull (its cocone files are the copies that plan merges). `exit()`s and `debug_output/` writes go |
| **SecondaryStructures** | 9.0k | **no licence header** (UT to confirm) | Helices and sheets from a volume's isosurface, via the cocone medial axis | On the same cocone core; it is the "SecondaryStructures port" the VolumeRover plan leaves Skeletonization's helix window to. `MainWindow.cpp:2267,2276` keep dangling `c_str()` pointers; its `.rawc` and `.off` writes go |
| **Curvatures**, **ColorSurfaceByFunction** | 1.1k | UT LGPL-2.1 | Analytic curvature of a sum-of-Gaussians surface (not libcvc's mesh fit); Coulomb colouring of surface vertices | Small; they sit beside `mesh_ops` |

### 2.3 Only if rewritten

- **GBEnergy and PB2** (generalized Born; boundary-element Poisson–Boltzmann). They stand on NFFT
  with its GPL `fastsum`, PETSc, and Lexing Ying's GPL KIFMM3D. A port means a new fast-summation
  base, and is a project of its own.
- **MultiresolutionSurface,** which links GPL FFTW; it could move to PocketFFT.
- **libCG** (coarse-graining), which has no licence headers and uses GPL levmar; Eigen could
  replace levmar.
- **ASpline and TriElement,** only if a boundary-element solver is planned.
- **AssemblyUtils and VirusAssembly,** which only test executables use.

### 2.4 What stays behind

- **The GUI and renderers:** DataManager, the GL renderers (impostors, splatting, volume library,
  objects, NURBS), the OpenGL viewer, Histogram, QGLViewer, ColorTable2 (libcvc has `tf_editor`),
  glew, log4cplus, the app, and the F2Dock dialogs. Ariadne documents and cvcGL replace them, in
  arimol (cyconv).
  - cvcGL has no molecule renderer yet. The molecular scene nodes (M0 in §8) are where impostor
    spheres and ball-and-stick go.
- **The restored copies of libcvc's own libraries.** VolumeFileTypes' DX reader is the one gap,
  and it becomes a libcvc volume I/O handler.
- **Licence blockers:**
  - **Decimation** has SHASTRA "no commercial use" code (`Bezier/BezierMath.*`, `Linear/root.c`)
    and IMSL "All Rights Reserved" code (`Misc/dscal.c`, `dsvdc.c`). It is built and linked into
    TexMol by default, and **TexMol's THIRD_PARTY_NOTICES.md does not list either** (§2.5).
    libcvc's `cvc::simplify` replaces it.
  - **C2C** carries a GPL arithlib and LizardTech's GPL libdjvu++; it only loads legacy `.c2c`
    surfaces.
  - **levmar** (GPL), **KIFMM3D** (GPL), **FastSummation** (its `fastsum.c` is GPL).
- **Dead code:** Docking and F2DockAnimator (they can't build), PatchComplex, Interface (an older
  copy of the filters), MolSurfAPI (its own `WARNING.txt` says to remove it), vis-lib,
  DynamicAdaptiveGrid and the small helpers.

### 2.5 Found on the way (for TexMol's own tracker)

- **Licence notices.** TexMol's `THIRD_PARTY_NOTICES.md` omits:
  - Decimation's SHASTRA and IMSL notices;
  - VRender (GPL) and gpc (non-commercial), which QGLViewer builds by default;
  - the GPL FFTW that MultiresolutionSurface links.

  TexMol#7 (open) handles the UT GPL headers, not these.
- **Undefined behaviour:**
  - HLevelSet's `iterationNum` type mismatch;
  - SecondaryStructures' dangling `c_str()` file names.
- **F2DockClient bugs:**
  - Its command-line path busy-waits on `clock()` for "3 s", which is about 3,000 s on Windows,
    where `CLOCKS_PER_SEC` is 1000, and can spin forever (`commandLineControl.cpp:53-57`).
  - Its "Nagger" thread polls every 60 s, never stops, and downloads results on the GUI thread.
  - Status and output calls can go to a different server than `Submit`.
- **Stale comment:** `src/TexMol/CMakeLists.txt` says libcvc replaces the compat libraries, yet
  links them.

### 2.6 How the TexMol ports sit with the rest

- **Molecules first (M0).** PDBParser and GOAFileTypes, with the molecular scene nodes, are the
  base for arimol, `libf2dock`'s inputs and the server's docking scenes.
- **The filters once.** The `cvc::mol` filters serve the analysis here and, after F4, `cvc::dock`.
  Until then `libf2dock` keeps its own copies, and an A/B (T16) checks that the two agree.
- **Cocone last.** PocketTunnel and SecondaryStructures wait for the VolumeRover plan's P1
  (`cvc::tet_complex` and the cocone core on Qhull).
- **Job kinds.** Every port is a job kind (§5): `mol.surface`, `mol.blur`, `mol.hls`,
  `mol.pockets`, `mol.characteristics`, `dock.interface`, `mol.tunnels`, `mol.secondary`. TexMol's
  batch commands map onto them.

---

## 3. F2Dock: from executable to `libf2dock`, then `cvc::dock`

### 3.1 What it is today

**Code.** About 12.6k lines in `src/f2dock`, of which `Docking.cpp` is 8,372 and its
`dockingMain` alone about 1,900. Static libraries for the filters and utilities:
- `fast-LJ`, `fast-clash`, `fast-hydro`, `fast-GB`, `fast-resCont`;
- `PG-range`, `fft-utils`, `math`, `vol`, `misc-ident`;
- the F3 modules.

All of them are globbed into one executable. `libmol` (~20k lines) is not built.

**The pipeline:**
1. Read two molecules (F2D: atoms plus skin pseudo-atoms; PQR; QUAD quadrature points) and a
   rotation set.
2. Build Gaussian-gridded shape, electrostatics, hydrophobicity and charge terms, and FFT the
   receptor's once.
3. For each rotation: rotate the ligand's grid, forward FFT, multiply, backward FFT, then scan the
   correlation for peaks and pass them through the on-the-fly filters (LJ, clash, pseudo-Gsol,
   dispersion), translational clustering and a top-N heap.
4. Merge, filter poses, apply the cluster penalty, and optionally rerank.
5. Write a 32-column pose table.

GB-Rerank is a separate executable that rescores that output.

**FFT.** PocketFFT by default (BSD-3), or FFTW opt-in (GPL). Transforms are single-threaded;
parallelism is across rotations. PocketFFT measures 1.3–1.7× slower than FFTW: 0.046 s for one
120³ transform.

**Cost.**
- **Memory:** about 0.2 GB shared plus 0.37–0.46 GB per worker thread at numFreq 120, so about
  2 GB at 4 threads.
- **Time:** the goldens record 65–99 s for 54 rotations on one thread (1–1.8 s per rotation). The
  examples' rotation set has 4,392 rotations, and the full sets go to 232,020. So a production run
  is tens of minutes to hours.

**State that makes it single-job.**
- The rotation work queue and its lock are globals (`Docking.cpp:47-48`), re-initialised by each
  job.
- The pose queue's counters are function statics (`:4606`).
- The adaptive pseudo-Gsol cutoff keeps function statics that are never reset (`:6227`), so even a
  second job run after the first differs.
- `srand(time(NULL))` runs in `main` and in the cuckoo hash.
- 55 `exit()` calls, including in filter constructors on an unreadable PQR or a parameter file
  missing its QUAD keys.
- `dockingMain` centres and rotates the caller's coordinates in place, and rewrites the caller's
  grid parameters and rotations.

**Bugs found reading it** (fix in F0):
- **Auto-detect never applies.** The complex-type detection uses `==` where it means `=`
  (`F2Dock.cpp:2158-2175`). An auto-detected antibody or enzyme complex always runs as generic, so
  the A / E presets and rerank filters never apply.
- **Multi-state runs keep one table.** The output file is opened inside `dockingMain`, so a
  multi-state run (F3 flex or domain) keeps only its last state's pose table, contrary to the
  migration doc. And the second state sees coordinates already centred by the first.
- **Crashes on bad input:** a key with no value dereferences NULL in the parser (`:858`);
  `GetOutput` with an index outside 0–9 returns a NULL `std::string`.
- **Hidden parameters.** Modules re-read the parameter file for keys the main parser never sees,
  such as `staticMoleculeQUAD`.
- **Namespace squat.** F2Dock's FFT seam is declared in `namespace cvc::fft`
  (`inc/fft-utils/fft_provider.h:68`), which collides as soon as it moves into libcvc.
- **Silent GPL link.** NFFT3 (GPL as far as checked) is linked in silently whenever pkg-config
  finds it, and the licence banner doesn't count it.
- **A missing data file:** `deg15.mtx`, which every example uses, is not installed.
- **The goldens use GPL code.** They were recorded with FFTW plus sparse FFT, so the default
  (PocketFFT) build is not what they test.
- **Untested:** the goldens all run one thread, so multithreaded determinism is never checked.

**The server** (`F2DockServer`, XML-RPC on libcvc's optional `cvc::xmlrpc`):
- `Submit` takes 37 positional values (flag / contents pairs for each input file, the job type)
  and returns an id.
- `getJobStatus` takes the id as a *string* and returns 1, 2 or 3; unknown ids read as SUBMITTED.
- `GetOutput` takes the id as an *int* and a file index.
- It never calls the engine. It writes `job<id>.scr` and runs it with `std::system`, synchronously
  on the local platform, or hands it to `qsub` on the PRISM2 cluster.
- State is marker files written by Perl scripts regenerated in the working directory for each
  submission, so concurrent jobs overwrite each other's markers.
- No auth, TLS, cancel, progress beyond three states, failure state, queue, per-job directory or
  cleanup.
- **TexMol's client** (`F2DockClient`, per the 2010 tutorial) polls `getJobStatus` from a "Nagger"
  thread and fetches outputs by index.

### 3.2 The plan

**F0, fixes in place (in transfix/F2Dock).**
- Fix the auto-detect, the NULL dereference and the `GetOutput` index.
- Open one output per state.
- Install `deg15.mtx`.
- Rename the FFT namespace (to `f2dock::fft`).
- Stop NFFT3 auto-linking; it is opt-in only, and counted.
- Re-record the goldens with PocketFFT, and add multithreaded golden runs.

Each changed result is called out in its PR: the auto-detect fix changes antibody and enzyme
runs, and the centring fix changes the multi-state goldens.

**F1, `libf2dock`, following cyconv's D12.** The engine as a library in the F2Dock repo, in the
VolumeRover plan's §1.4 shape:
- **Inputs:** two in-memory molecules (coordinates, charges, radii, atom kinds, names and residue
  numbers), quadrature points, the rotation set, and the optional RMSD reference and
  efficient-grid sizes. The parsers become reader functions, so there is no file I/O in the
  algorithm.
- **Parameters:** `dock_params`, about 140 scalars plus the module-private keys, with a parameter
  table. The defaults leave `main()`, and the antibody / enzyme presets become a function.
- **A per-job `dock_context`** owns everything that is global today:
  - the rotation cursor and lock, and the pose queue;
  - the adaptive cutoff state;
  - a seeded generator;
  - the grids, plans and filters, and per-worker scratch;
  - a cancel flag, progress counters and a warnings sink.
- **The job:** `std::unique_ptr<result_job<dock_result>> dock_job(inputs, params)`.
  - Phases: validate, setup, rotation scan, merge and cluster, pose filtering, rerank.
  - **The progress unit is one rotation,** counted by the cursor, so pruned rotations count too.
  - Cancel is checked between rotations and between filtered poses.
- **Results by value:** poses (rank, score and its components, a 3×4 transform, conformation,
  RMSD) and statistics. The legacy `.out` writer becomes a separate function.
- **Threads:** raw pthreads give way to `app.computePool()`, with stable worker indices for the
  per-worker scratch the filters index by thread.
- **No exits:** `exit()` becomes exceptions, and the `printf`s become the warnings sink and
  `app.log()`.
- **Determinism:** the adaptive pseudo-Gsol cutoff is shared by all workers, so with more than one
  thread its result can depend on scheduling (unverified). F1 makes it per-job, and offers a
  deterministic variant that does not depend on thread order. That variant is new behaviour,
  compared by tolerance, not exactly.
- **Wasm:** a rotation costs 0.5–1.8 s natively, far too coarse to slice on single-threaded wasm.
  So `libf2dock` targets wasm-mt only, where it runs on the pool as natively. Browser memory caps
  the thread count (about 0.4 GB each).
- **Licence:** `ElementInformation.h` is on the default path (hydrophobicity, pseudo-Gsol), and
  its relicensing from GPL has no record. It and `libmol` wait on UT's rulings (cyconv's D13). If
  the ruling goes the wrong way, the hydrophobicity table is rebuilt from its published source
  (Kapcha and Rossky).

**F2, the A/B.** `libf2dock` runs against the legacy F2Dock executable on the four golden
complexes and on new ones:
- poses and scores equal, after F0, at the goldens' tolerance;
- the same pose sets at one thread and at N threads, with the deterministic cutoff;
- times for both.

This is the VolumeRover plan's T16, with the legacy program as side A.

**F3, the service** (§5). Docking becomes a `cvc::jobs` kind, `dock`.
- Before F1 passes its no-singletons scan and its bad-input tests, the service runs the legacy
  executable in a worker process, as cyconv's §9.3 describes.
- After that, it runs `libf2dock` in-process or in a worker, by configuration.
- GB-Rerank becomes a second kind, `dock.rerank`, on the same results.

**F4, `cvc::dock`.** When `libf2dock` has settled, its implementation moves into libcvc behind the
same API (cyconv's D12). Then the browser's arimol, the server and every host share one engine,
and the docking intrinsics (`dock.submit`, `dock.score`) become ordinary libcvc intrinsics.

**Input preparation.** Today PDB2PQR (Python) and MolSurf (F2D, QUAD and RAWN generation) run as
external programs, which the legacy server shells out to.
- **TexMol's Blurmaps already has an `F2dGenerator`,** so the F2D step comes in-process with the
  Blurmaps port (§2.2). That is what makes docking in the browser possible: a browser can't run
  MolSurf.
- **QUAD (surface quadrature) generation:** where its source lives is unverified. MolSurf is not
  among TexMol's libraries. It is ported or rewritten on Blurmaps' skin surfaces once found.
- **PDB2PQR** stays a worker kind (`mol.pqr`) until a native replacement is worth writing
  (§9 item 5).

---

## 4. arisrv: many rooms, each a scene, each able to host compute

### 4.1 Today (origin/master 016a2038)

- **One room per process** (`--room NAME`, default `standup`). The write gate, the area of
  interest, chat, the holon and the room's world are each built with that one name. Hosting N
  rooms means N processes on N port sets, and there is no lobby.
- **Transports:** gRPC state (7700), gRPC media and streams (7701), WebSocket state (7702), and the
  WS↔gRPC bridge. No TLS; browsers come in through a `wss://` proxy, as cyprty's does.
- **Writes are bound per participant; reads are not.** Every client gets the whole relayed tree.
  The area of interest filters others' poses and media by distance, which the conference roadmap
  calls "delivery policy, not access control".
- **Identity:** one shared room token per relay, and Ed25519 device keys pinned on first use.
  There is no OIDC on the server.
- **Server documents (S0, #1046)** run `init:` and `on_tick:` at 60 Hz on the relay loop, and
  `server: publish:` makes namespaces the document can write. But **clients drop those
  namespaces:** arigl's interest is `conf` only, until S2's `relay: subscribe:`.
  - A document can call async functions, but arisrv registers none.
  - There is no Python, no HTTP, no persistence, and no reload.
- **A room's scene** (`conf.rooms.<room>.scene.*`) carries the environment, light rigs, the
  primitives (plane, box, sphere), meshes by URI, transforms and plain materials. Everything else
  is dropped with a warning: heightfields, volumes, `data:` sources, custom node types, children,
  textures and shaders.
- **Asking the server for work:**
  - The C++ chat request / reply is the only request path; cyconv calls it the template.
  - `room-call` doesn't exist.
  - Wire messages never reach DSL `msg-recv`, because the shard bus and the scheduler's bus are
    separate.
- **cyprty** is one document, a copy of `conference_roster.ari` at 3.7.9. Its page has no room
  parameter, so every browser joins `standup`. Its relay is the arisrv 3.7.10 bundle, from before
  S0 and WD1.

### 4.2 The target

**One relay, many rooms** (worlds plan RM1–RM2, brought forward).
- **Rooms are declared.** The server document declares its rooms. Each room names its world
  document or defines one inline, and each gets its own write gate, area of interest, chat, holon,
  clock and server document.
- **A room is an object.** The relay's room-shaped pieces become one `room_host` per room, on the
  app (the VolumeRover plan's §1.3).
- **A lobby** (`conf.lobby.rooms.<id>.{title, world, features, capacity, occupancy, access}`) is
  what a client reads first.
- **Rooms appear at run time** from server programs, or from participants where the rules allow.
- **Bound reads per room:** a client receives only the rooms it joined.
- **Joining picks a room.** `room-join` gets real transport wiring, so a client can join a room
  other than the relay's default and move between rooms. The page and `arigl` take a room
  parameter.

**Room features decide what a room runs** (worlds plan CF1, and cyconv's SDK-19).

| Feature | Brings | Example rooms |
|---|---|---|
| `presence` | participants, roster, chat | all of them |
| `avatars` | avatars, the holon's physics, area of interest | cyprty's conference room, a walk-around exhibit |
| `media` | cameras, microphones, screen and app share (SH0–SH6) | meetings |
| `scene` | a shared scene document with science node types (below) | a molecule review, a volume lab, a digital twin |
| `compute` | the job service (§5), scoped to the room | a docking lab, a segmentation workspace |

A room is any combination of these.
- A **workspace** is `presence` plus `scene` plus `compute`, with no avatars or media: cyconv's
  "app-server mode".
- A **docking lab** for a group adds `avatars` and `media`, so people meet beside the pose browser.

The holon, the area of interest and the media router run only in rooms that ask for them.

**Room scenes for science and for digital twins.** The room's world travels as sections with
assets by hash (worlds plan §3 and AD1). This plan adds the node types science needs:
- **volumes:** `volume_slice` and `volren`, from region reads on the server or a hashed volume
  asset;
- **molecules:** atoms and bonds, surfaces, poses (cyconv's SDK-15);
- **geometry from the ports:** skeleton strands and sheets, tiled surfaces, segmentation labels;
- **`data:` sources:** the data roadmap's remote sources (`cvc+ws://`, `cvc+grpc://`) bound into
  the scene.

**A digital twin** is a room whose scene binds live data:
- a server-side `data:` source, or an entity fed by an external feed (worlds plan WD3), writes the
  room's state;
- every client's scene follows it.

Clients fetch only what they need (AD2).

**What a room can have.** Several scenes per room as separate viewports or layers, not one merged
world:
- the room's world, where avatars walk;
- the science scene, in a `viewport` (ARIADNE.md §5.18);
- a job's result scene.

`scenes:` already exists on the client side; the room publishes which ones it carries.

**Clients:**
- arimol, arirov and aridock, through `ariglx` or arigl-web;
- cyprty, as one of them.

All need S2's `relay: subscribe:` to see published namespaces such as `jobs` and `dock`.
Browsers need it over WS.

### 4.3 cyprty as the front door

cyprty becomes the lobby and the conference rooms for this work:
- **The lobby** is a room with the room list.
- **Doors lead to:**
  - a **docking lab**: the receptor and the top poses in the scene, a job panel, and video;
  - a **volume lab**: a `volume_grid` scene, with segmentation and skeleton jobs;
  - a **twin**: a site or a device, bound to live data.

**What that needs, beyond §4.2:**
- a room parameter on cyprty's page;
- a relay at or past S0 / WD1 (3.7.10 is older), with the room-scene section format;
- cyprty's own light rig moved into the world document (master already says the rig belongs
  there).

---

## 5. One job service: `cvc::jobs`

Every server-side computation goes through one job layer, so a client handles a docking run, a
segmentation and a skeleton the same way. cyconv names this layer F13 ("the job layer the server
runs on"). This plan puts it in libcvc, so arisrv, `arisrvx`, the CLI and local apps share it.

**Job kinds.** Each kind has a name, a spec, a result type, an executor and limits.
- **The name:** `dock`, `dock.rerank`, `segment.seeded`, `skeleton.compute`, `tiler.tile`, and the
  TexMol kinds (§2).
- **The spec** is the port's parameter table, plus inputs given as blob hashes or room assets.
- **The result type** is the port's result struct, stored in pages.
- **The executor** is in-process (a `result_job` on the server's pool) or a worker process.
- **The limits:** time, memory, concurrency, and a quota per principal.

**What a job publishes.** Per room, scoped by S6 to the jobs a principal may see:
- `jobs.<id>.{kind, owner, room, state, phase, progress, created, started, finished, error}`;
- a small `summary` (for docking, the best poses);
- `pages.<n>`: a few hundred rows each, one data blob per page, never a leaf per row.

Clients that only watch read the state tree. Nothing polls.

**Calls.** S2's `room-call` exports, with `ctx.principal`, an idempotency key and deadlines:
- `jobs.submit(kind, spec)` returns an id at once, so an hours-long job never rides a call
  deadline;
- `jobs.cancel(id)`;
- `jobs.list(filter)`;
- `jobs.result(id, page)`;
- `jobs.blobs_needed(hashes)` and the blob upload, client to server only (SDK-17).

**The same service in three fronts, so they can't drift apart:**
- `room-call` exports for native and browser clients;
- **FastAPI** routes in `arisrvx` (S1, cyconv's D8): `POST /v1/jobs`, `GET /v1/jobs/{id}`, paged
  results, `DELETE` to cancel, `PUT /v1/blobs/{sha256}`;
- **DSL intrinsics** in server documents, `jobs.submit-async` and friends, so a room's document
  can run a pipeline: PDB to PQR to F2D to dock to rerank to scene.

**Results into the scene.** A job's result can be placed into the room's scene: the top poses as
molecule nodes, a skeleton, segmentation labels on a slice. Everyone in the room sees it, and the
job's provenance stays on the node.

**Durability.** A job record and its result pages go to a store on the server: sqlite and files,
per cyconv's §9.3.
- On restart the server republishes `jobs.*`, and clients re-sync through the join snapshot.
- This is the job layer's own store, separate from world persistence (worlds plan WD8), though
  they may share a backend.

**Workers.**
- **In-process** for libcvc's own sliced jobs: the VolumeRover ports and the TexMol ports, once
  they pass their no-singletons and bad-input rows.
- **Worker processes** for the rest: legacy F2Dock, anything that can `exit()`, and any Python
  engine. A worker reports progress and pages up a pipe, and the server turns them into state
  writes.
- **Cancel** sets the engine's cancel flag, then kills the worker after a grace period.

**Security.**
- **Principals** from S5's authenticate hook; **read scope** from S6 (cyconv's D14 pulls S6
  forward for exactly this).
- **Rooms** carry their own access (open, token, invitation or private).
- **Data, not paths:** a job never names a server path (§1 item 4).
- **Limits** per principal and per room.

**Native or Python** (§9 item 1).
- cyconv's design puts the F2Dock service in Python on `arisrvx`, which keeps FastAPI natural.
- This plan puts the job layer in C++ in libcvc, so plain `arisrv`, the CLI and the browser have
  it with no Python, and wasm-mt hosts can run jobs locally under the same schema.
- Python services register kinds on the same layer. **Recommended:** the C++ layer, with Python
  kinds on it; cyconv's F2Dock package becomes a set of kinds rather than its own service.

---

## 6. Ariadne pieces this needs (beyond the VolumeRover plan's §6)

1. **`room_host`:** the relay's per-room objects, many per process (§4.2).
2. **The lobby section and room access**, from the worlds plan §10.
3. **Room features in `server:` and `requires:`** (CF1): `presence`, `avatars`, `media`, `scene`,
   `compute`.
4. **Room scene sections for the science node types**, with assets by hash (§4.2, AD1).
5. **`relay: subscribe:`** on clients, native and browser (S2).
6. **`room-call`** (S2), with `ctx.principal` (S5) and per-principal read scope (S6).
7. **Uploads by hash, client to server only** (cyconv's SDK-17): relay-addressed messages, or
   `room-call` with bytes arguments up to tens of MB, chunked.
8. **Named streams over WS** (SH0, cyconv's SDK-18), for live views such as an FFT score field or
   a job's partial results.
9. **Wire messages into DSL `msg-recv`,** so server and client documents can talk without C++.
10. **A job panel component** (`cvc://components/jobs.ari`): a list with progress, cancel,
    results and "place in scene", bound to `jobs.*`.

---

## 7. Tests

Every port and service here carries the VolumeRover plan's matrix (T1–T16). Additions:

| # | Kind | What it checks |
|---|---|---|
| R1 | Rooms | Two rooms in one relay: separate gates, chat, area of interest, clocks and server documents. A client in room A never receives room B's tree (bound reads). Rooms created and closed at run time. Two apps in one process, each with rooms (the no-singletons rule) |
| R2 | Room scenes | Each science node type round-trips through the room wire format. A late joiner gets the whole scene by hashes. A digital-twin source updates every client's scene |
| J1 | Jobs | Submit, progress, cancel in every phase, paged results, failure codes. Idempotent resubmit. Quotas. Restart republishes `jobs.*` from the store. An in-process job and a worker job look the same to the client |
| J2 | Job security | A principal sees only its own and shared jobs (S6). No spec can name a server path. A worker that exits or crashes becomes a failed job, not a dead server |
| J3 | Fronts | The same job through `room-call`, FastAPI and a server document's intrinsic gives the same records |
| C1 | cyprty | The lobby lists rooms; a browser walks from the lobby into a docking lab and sees a job's poses appear (headless Chrome, as the arigl-web lane does) |

---

## 8. Phases

Sizes: S is a PR or two, M a few, L a short series, XL a longer one.

| Phase | Size | What lands | Needs |
|---|---|---|---|
| T0 TexMol decisions | S | The library list in §2 settled, TexMol#7's relicensing merged, the UT rulings on `libmol` and `ElementInformation.h` (cyconv's D13) | — |
| T1 Surfaces | L | Blurmaps with Summation, DPG and SphericalPatchIntersections: densities, molecular surfaces, SAS, skin regions, F2D generation; `mol.surface` / `mol.blur` kinds | M0, VolumeRover P0a–P0b |
| T2 SES and pockets | M | HLevelSet; Pocket rebuilt on `cvc::sdf` and `cvc::iso`; `mol.hls` / `mol.pockets` | T1 |
| T3 Docking analysis | M | InterfaceStats, MolecularCharacteristics, and the `cvc::mol` filters (clash, pseudo-Gsol, LJ, residue contacts) shared with F2Dock, with an A/B against `libf2dock`'s copies | M0, the D13 rulings |
| T4 Docking inputs and results | S | F2DockClient's Qt-free parameter structs, `.inp` writers and `.out` parser, so legacy jobs load into arimol | M0 |
| T5 Tunnels and secondary structure | M | PocketTunnel and SecondaryStructures on the cocone core | VolumeRover P1, UT on SecondaryStructures' licence |
| T6 Small pieces | S | Curvatures, ColorSurfaceByFunction, the DX volume reader | T1 |
| F0 F2Dock fixes | S–M | §3.2 F0, in transfix/F2Dock | — |
| F1 `libf2dock` | L | §3.2 F1 | F0 |
| F2 F2Dock A/B | S | §3.2 F2 | F1 |
| M0 Molecular types | M | Molecules, F2D / PQR / PDB / QUAD readers on the app, molecule scene nodes (cyconv's SDK-15) | VolumeRover P0a |
| J0 `cvc::jobs` core | M | Kinds, specs from parameter tables, in-process executor on `result_job`, the published `jobs.*` layout, the store; works in a local app and the CLI with no server | VolumeRover P0b |
| S1, S2, S5, S6 | per ARIADNE_SCRIPTING_HOSTS | `arisrvx` and FastAPI; `room-call` and `relay: subscribe:`; principals; read scope (cyconv's D14 order) | VolumeRover P0a |
| J1 Jobs on the server | M | `room-call` and FastAPI fronts on J0, worker processes, uploads by hash (SDK-17), the job panel component | J0, S2, S5, S6 |
| R1 Many rooms | L | `room_host`, the lobby, run-time rooms, bound reads per room, real `room-join` wiring, room parameters for arigl and arigl-web | VolumeRover P0a, S6 |
| R2 Room features | M | `presence` / `avatars` / `media` / `scene` / `compute` (CF1), workspaces with no holon or media (SDK-19) | R1 |
| R3 Science scenes | L | Room scene sections for volumes, molecules, port geometry and `data:` sources, assets by hash (AD1), fetch by need (AD2) | R1, M0 |
| F3 Docking service | M | `dock` and `dock.rerank` kinds: legacy executable in a worker first, then `libf2dock` | J1, F1 for in-process |
| C1 cyprty rooms | M | The lobby and the docking lab, volume lab and twin rooms, a relay past S0 / WD1, the page's room parameter | R2, R3, J1, F3 |
| F4 `cvc::dock` | L | `libf2dock`'s implementation inside libcvc, docking intrinsics, the browser's arimol docking in wasm-mt | F2, M0 |

**Parallel tracks.**
- F0–F2 run in the F2Dock repo, beside everything else.
- T0 and T1–T6 run beside the VolumeRover ports, sharing their P0. T5 waits for the VolumeRover
  plan's P1.
- J0, M0 and the arisrvx stages can run at once.
- R1 waits for the VolumeRover plan's P0a: many rooms in one process is where shared state breaks.

---

## 9. Decisions for Joe

1. **The job layer: C++ in libcvc, or Python on `arisrvx` only?**
   - cyconv's §9.3 has the F2Dock service in Python.
   - This plan puts `cvc::jobs` in C++, so plain arisrv, the CLI and wasm hosts have it, and
     Python registers kinds on it.

   **Recommended:** C++, with FastAPI from `arisrvx` and the F2Dock package as kinds. That amends
   cyconv's D8 without reversing it: the server still runs in `arisrvx`, with FastAPI.
2. **Many rooms in one process, or a process per room behind a lobby?**
   - One process is simpler to operate and is what the worlds plan describes.
   - Processes isolate a crashed room.

   **Recommended:** one process with many rooms, and compute isolated by worker processes, where
   crashes actually come from.
3. **The arisrvx order.** cyconv's D14 is S0 → S1 → S2 + S5 + S6 → S7. This plan needs the same,
   with R1 right after S6. Confirm D14.
4. **The legacy TexMol `F2DockClient`.** Keep the Qt app docking against the new service through
   an XML-RPC shim that maps the 37-argument `Submit` onto `jobs.submit("dock", …)`? Or retire it
   for arimol?

   **Recommended:** retire it. arimol replaces it, and the shim would preserve a protocol with no
   auth.
5. **Input preparation in-process.** Bring MolSurf's F2D and QUAD generation into libcvc (§3.2),
   and replace PDB2PQR (Python) with a native step or a worker kind?

   **Recommended:** MolSurf in-process with the TexMol ports, because it gates docking in the
   browser. PDB2PQR as a worker kind first.
6. **Names.**
   - Services and types: `cvc::jobs`, `room_host`.
   - Room features: `presence`, `avatars`, `media`, `scene`, `compute`.
   - Job kinds: `dock`, `dock.rerank`.
   - The F2Dock libraries: `libf2dock`, then `cvc::dock`.
