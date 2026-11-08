# VolumeRover Library Ports — Roadmap (sketch)

**Status:** sketch, 2026-11-07, first merged as transfix/libcvc#1057. Revised 2026-11-08: GenSeg
added and ContourTiler recorded as LGPL. Then Joe's rules for every port were added (§1.2–§1.7):
- the GPL-licensed parts go in a separate library;
- no singletons;
- an API that pycvc can wrap;
- far more unit tests;
- every parameter and every piece of data on the state tree;
- DSL intrinsics.

Nothing has landed. §9 lists the decisions still open.
**Based on:** libcvc `origin/master @ 921c4254` (the state tree, state_exec, pycvc and registry
facts in §1) and `@ 7a09366f` (the rest), and volrover `master @ b4a172c` (the legacy libraries
live under `src/<Lib>` and `inc/<Lib>` there).
**Scope:** four legacy VolumeRover libraries, ported into libcvc the way `mesh_lab` / `fe_lab`
were: a headless `cvc::` API, a CLI command, pycvc bindings, DSL intrinsics and an Ariadne node
type with a component.
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
- transfix/libcvc#123: the other VolUtils leftovers.

---

## 0. Summary

- **Rules every port follows** (§1, new in this revision):
  - **The GPL-licensed parts go in a separate library** (settled, §1.2). The library is
    `cvc_gpl`, with `cvcgl_gpl` for its Ariadne node types and `pycvc_gpl` for its bindings.
    Skeletonization and ContourTiler land there, because the CGAL packages they use are GPL.
    libcvc, cvcGL and pycvc stay LGPL, and they never link the GPL libraries. GenSeg,
    VolumeGridRover and the shared types stay in libcvc.
  - **No singletons** (§1.3): no mutable global, static, `instance()` or hidden registry in a
    port, vendored code included. A CI scan of the object files checks this, and so do tests
    that run two jobs at once.
    - The legacy code has plenty to remove: Skeletonization's global bounding box, ContourTiler's
      log4cplus loggers and `Controls::instance()`, and GenSeg's 25 globals.
    - The libcvc extension points the ports register with are process-wide statics today:
      Ariadne's node-type and intrinsic registries and the file I/O handler maps. P0 gives each
      a per-app form.
  - **One API shape** (§1.4):
    - free functions on value types;
    - a params struct and a result struct;
    - a job that owns its result;
    - exceptions, never `exit()`.

    SWIG wraps the header itself, so pycvc has the same names.
  - **The state tree holds everything** (§1.5):
    - Every parameter is a key. Each params struct is declared once in a parameter table, which
      also gives the defaults, the units and ranges as state properties, the CLI flags, the
      Python keywords and the DSL dict keys.
    - Inputs and results sit on the data channel at documented keys. The GPL nodes read the LGPL
      nodes' data there, so neither library links the other.
  - **DSL intrinsics** (§1.6): each port's operations are state_exec async functions, such as
    `skeleton.compute`, `tiler.tile` and `segment.seeded`. Scripts, pycvc's `Exec` and Ariadne
    hosts can call them.
  - **Tests** (§1.7): a fixed matrix of 14 kinds of test per port. It runs from legacy parity
    and exact topology through concurrency, jobs, state round trips and I/O corpora to the DSL,
    pycvc, the CLI and the licence boundary.
- **Skeletonization** (§2) is a clean port.
  - It is about 8k lines of LGPL-2.1 code whose only real dependency is CGAL's 3D Delaunay. It
    needs its global state removed, a few bug fixes, and its sheets kept rather than dropped.
  - It lands in `cvc_gpl` as `cvc::skeletonize`, `cvc-gpl skeleton` and `skeleton.compute`, and
    as a `skeleton` node that reads a `mesh_lab` node's geometry from the state tree.
  - Its cocone code is copied almost verbatim in TightCocone, PocketTunnel and Curation, so it
    becomes one shared `cvc::cocone` core.
- **Tiling** (§3) means ContourTiler, not the old `src/Tiling`.
  - The old tiler has not been built since 2011, and it carries a Purdue SHASTRA notice that
    forbids commercial use.
  - ContourTiler reimplements the same 1996 algorithm for many components, and it is the one
    VolumeRover2 uses. It is heavy CGAL, using exact boolean ops and so GMP and MPFR, and it is
    LGPL like libcvc.
  - It lands in `cvc_gpl` as `cvc::tile_contours`, `cvc-gpl tile`, `tiler.tile` and a
    `contour_lab` node. The shared `cvc::contour_stack` type it builds on stays in libcvc.
- **VolumeGridRover** (§4) is a rebuild, not a translation.
  - Its Qt6 code is mostly stubs: contours cannot be created, and tiling, SDF and the medial
    axis are commented out. Its GL is 2005-era.
  - What carries over is the design: three axis slice views, a voxel probe, point classes,
    contours drawn per object, and tiling them into surfaces.
  - It becomes a `volume_grid` node plus a `volume_grid.ari` component of four embedded
    `viewport`s. It is LGPL, and it reaches the tiler only through a `contour_lab` node and the
    state tree.
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
  - It lands in libcvc as `cvc::segment_seeded`, `cvc segment` and `segment.seeded`, plus a
    Segmentation tab in `volume_grid`.
- **Most of the GL viewer already exists** (§6): the `viewport` widget, tags, named cameras and
  `scenes:`. Four new pieces are needed, all reusable:
  1. an axis-aligned `volume_slice` node;
  2. a `mode: slice` orthographic camera;
  3. pointer events from a viewport into state (the open "click-to-world picking" item);
  4. overlays that stay on top of a slice.
- **Order** (§7):
  - P0 is larger now. It builds the shared pieces the rules need: the parameter table, the
    result-owning job, per-app registries, the GPL library targets, the singleton scan, the
    pycvc job wrapper and the legacy-goldens script.
  - Then four tracks can start at once: the Skeletonization core, the ContourTiler core, the
    GenSeg core and the Ariadne viewport pieces.
  - VolumeGridRover follows the viewport pieces. Its Tile button follows the tiler, and its
    Segment button follows GenSeg.
- **Decisions for Joe** (§9):
  - the web;
  - the existing GPL CGAL uses in libcvc;
  - whether every existing caller moves off the process-wide registries now or later;
  - GPL packaging;
  - the layout, the seeding default and the names.

---

## 1. The shape every port takes

### 1.1 Layers

The `mesh_lab` / `fe_lab` work set the pattern. Each library lands as up to six layers, and each
layer is useful on its own. "GPL" below means the layer lives in the separate GPL library (§1.2)
because the code uses a GPL CGAL package.

| Layer | Where | What it is |
|---|---|---|
| 1. Core | libcvc: `src/cvc/<area>/<lib>/` behind `inc/cvc/<area>/<lib>.h`. GPL: `src/cvc_gpl/<lib>/` behind `inc/cvc/gpl/<lib>.h` | A headless `cvc::` API in the §1.4 shape, on `cvc::volume` / `cvc::geometry`. No Qt, no GL, no singletons (§1.3), no `exit()`, no files read or written inside the algorithm. Long jobs are `cvc::sliced_job`s ([`sliced_task.h`](../../inc/cvc/core/sliced_task.h)), so they run cancellable and with progress on the background pool natively, and a few ms a frame on single-threaded wasm. |
| 2. CLI | `src/cvc-cli/main.cpp` (`cvc`); GPL: `cvc-gpl` (§9 item 4) | One subcommand per library (`cvc segment`, `cvc-gpl skeleton`, `cvc-gpl tile`), with the legacy executables' options kept as aliases. Flags and help come from the parameter table (§1.5). |
| 3. Python | `bindings/pycvc` (`pycvc`); GPL: `pycvc_gpl` | SWIG wraps the public header (§1.4), so the C++ names carry over. |
| 4. DSL intrinsics | Registered on the app (§1.6) | Async functions for `cvc exec`, pycvc's `Exec`, arisrv programs and Ariadne hosts. |
| 5. Node type | `src/cvcGL/ariadne/` + `inc/cvc/gl/ariadne/`; GPL: `cvcgl_gpl` | A custom scene node, registered on the app (§1.3). It owns its data, runs the core on its own `cvc::async_lane` or the state_exec scheduler, keeps its parameters and data on the state tree (§1.5), and publishes `busy`, `status`, `progress` and `phase`. Commands are a `request` key (ARIADNE.md §5.5). |
| 6. Component | `src/cvc/ariadne/components/<name>.ari` + an `INDEX.ari` entry | A window that binds those keys, so a document drives the node with no host C++. It takes the args `node`, `title` and `id`, like every node-scoped component since b2f54578. |

### 1.2 The GPL-licensed parts: a separate library (settled 2026-11-08)

**Why.** The ported code itself is LGPL: Skeletonization, ContourTiler, GenSeg, SegMed and
VolumeGridRover. So are GMP and MPFR: GMP is dual LGPL-3+ / GPL-2+, MPFR is LGPL-3+, and linking
them leaves libcvc LGPL. What brings in the GPL is CGAL. Its kernel, number types and support
library are LGPL-3+, but most of its algorithm packages are GPL-3+:
- Skeletonization uses 3D Triangulations;
- ContourTiler uses 2D Boolean operations, Arrangements, constrained Delaunay and the straight
  skeleton.

Dropping FFTW to make the bundle LGPL-2.1 shows the licence matters. So any code that includes a
GPL CGAL package goes in a library of its own, and libcvc stays LGPL.

**What goes where**

| | libcvc, cvcGL, pycvc (LGPL) | `cvc_gpl`, `cvcgl_gpl`, `pycvc_gpl` (GPL-3+ as built) |
|---|---|---|
| Core | GenSeg and SegMed (`segment_seeded`, `gradient_vector_diffusion`, the fast marching), `contour_stack`, `point_classes`, their I/O, `param_table`, `result_job` | the shared cocone core, `skeletonize`, `tile_contours`; later TightCocone, PocketTunnel, SDF2D and the 2D medial axis |
| CLI | `cvc segment`, contour and point-class conversion | `cvc-gpl skeleton`, `cvc-gpl tile` |
| Python | `pycvc` | `pycvc_gpl`, which uses pycvc's types through SWIG's `%import` |
| DSL | `segment.*`, `contours.*`, `point-classes.*`, `params.*` | `skeleton.*`, `tiler.*` |
| Ariadne | `volume_slice`, `volume_grid`, `mode: slice`, picking | the `skeleton` and `contour_lab` node types and their components |

**Rules**

- **One direction only.** `cvc_gpl` links libcvc, never the reverse. Nothing in libcvc, cvcGL or
  pycvc includes a GPL-library header.
- **The compiler enforces it.** The LGPL targets compile with `CGAL_LICENSE_ERROR` defined, so
  including a GPL CGAL package there is a compile error. These are CGAL's licence-check macros;
  check that the CGAL version cvcpkg ships honours them.
  - This goes on the new port directories at once.
  - It goes on all of libcvc once the two existing GPL uses are dealt with (§9 item 2).
  - A CI lane also builds with `CVC_ENABLE_GPL=OFF`.
- **No link between the Ariadne libraries.** The Ariadne nodes reach each other's data only
  through the state tree.
  - `volume_grid` never calls the tiler. It publishes its contours at a data key, and a
    `contour_lab` node configured with `source: { node: <grid id> }` tiles them (§4).
  - A `skeleton` node reads a `mesh_lab` node's geometry the same way.
  - A host that wants the GPL nodes calls `register_cvcgl_gpl_extensions(app)`.
- **Licence notices.** The vendored sources keep their LGPL headers. `cvc_gpl` as built is
  GPL-3+ because of CGAL. `src/cvc_gpl/` carries a COPYING and a README that say so.
- **Builds.** `CVC_ENABLE_GPL` turns the GPL libraries on. It is on by default where CGAL is
  found and off on wasm, where CGAL is off anyway. How releases ship them is §9 item 4.
- **Existing GPL code in libcvc.** libcvc already compiles CGAL's Spatial Searching (also GPL)
  into `nav/spatial.cpp` and `project_verts.h` whenever CGAL is found. Those two either move
  into `cvc_gpl` or switch to a permissive k-nearest-neighbour search (§9 item 2).

### 1.3 No singletons

**The rule.** A port holds no mutable state outside the objects its caller owns. None of these
may appear anywhere in a port, vendored code included:

- namespace-scope or class-static mutable variables;
- function-local `static` variables, unless they are constant tables initialised from constants;
- `instance()` / `getInstance()` accessors, and registries reached without an `app`;
- `thread_local` used to carry context past a function's arguments;
- `srand` / `rand`, an unseeded `std::random_device`, or CGAL's `get_default_random()`;
- static counters (debug file numbering), static caches;
- the current working directory (no relative-path writes), or environment variables read inside
  an algorithm.

**What replaces them**

- A context struct per call or per job (`skeleton_context`, `tiler_context`, `genseg_context`).
  The entry point creates it, and it holds everything the legacy code kept globally.
- Random number generators live in the context, seeded from a `seed` parameter.
- The `cvc::app` comes in explicitly. Volumes and geometry already hold their `app&`.
- Logging goes to `app.log()`, or into the result's `warnings`.

**What has to go, per port**

| Port | Singletons and globals in the legacy code |
|---|---|
| Skeletonization | the global `bounding_box` vector, never cleared; `srand(0)` / `rand()` for colours; degenerate-case debug files in the current directory |
| ContourTiler | log4cplus: 156 `getInstance` calls and 104 function-static loggers in 48 files; ContourFilter's `Controls::instance()`, which the app's `.ser` reader goes through; static counters naming debug files |
| GenSeg | about 25 globals; `extern VolMagick::VolumeFileInfo loadedVolumeInfo`, set by the reader; output files in the current directory |
| SegMed | not audited yet (P6 does it) |
| VolumeGridRover | nothing carries over; its `QSettings` cache directory and temp files stay behind |

ContourTiler's logging goes through a shim whose logger is a member of `tiler_context`. Each
vendored function that logs gets the context passed in. That is a mechanical change across the
48 files. Trace output nobody needs can be compiled out instead.

**libcvc's side.** The state tree already works this way: the zero-arg `state::instance()` was
removed, and every root comes from its `app` (STATE_API.md, "Per-app state roots"). But the
extension points the ports register with are still process-wide statics:

- Ariadne's node-type registry: `register_scene_node_type`, the function-static
  `node_registry()` in `scene_realize.cpp`. This is so even though `register_cvcgl_extensions`
  takes an `app&`.
- Ariadne's action-intrinsic providers: `register_action_intrinsics`, a function-static vector in
  `ariadne.cpp`.
- The volume and geometry file I/O handler maps: `volume_file_io::handlerMap()` and
  `geometry_file_io::get_handlers()`, both function-local statics. This is so even though
  `register_rawiv_io(app&)` and its siblings take an `app&`.

P0 gives each of these a per-app form, on `cvc::app` or, for Ariadne, its `AppRuntime`. The ports
register only there. Moving the existing callers off the process-wide forms is a separate cleanup
(§9 item 3).

**How it is checked**

- **A CI scan of the object files.** For each port's objects, list the writable data symbols:
  `nm` on Linux, the `.data` / `.bss` symbols, including function-local statics and their guard
  variables. Fail on any that is not on a short reviewed allowlist, such as CGAL's or Boost's
  own. This catches what a grep misses: a static inside a macro, or a vendored template.
- **Concurrency tests** (§1.7 T6): two jobs with different parameters on two threads give exactly
  what each gives alone, under the TSan lane (`sanitizers / tsan`).
- **Order tests:** running A then B, B then A, and each alone all give the same answers. This
  catches leftover state such as the bounding box.
- **Two apps:** two `cvc::app`s in one process, each with its own nodes and registrations, never
  see each other's.

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
- **A job:** `std::unique_ptr<result_job<R>> f_job(input, params)`.
  - `cvc::result_job<R>` (P0) is a `sliced_job` that owns its input (shared, read-only) and its
    result, read with `result()` once `done()`. The earlier sketch's `out` reference parameter
    is gone.
  - The blocking function is the job run to the end, so the two can't drift apart.
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
- **Jobs.** `sliced_job` isn't wrapped today. P0 wraps `result_job<R>` once, generically, with
  `step`, `done`, `cancel`, `progress`, `phase`, `status` and `result`. That makes every port's
  job usable from Python with progress and cancel, beside the DSL async functions available from
  an `Exec`.
- **State.** `from_state` and `to_state` take a `pycvc.State`, following the parity doc's rule
  that tree arguments are `State`s.
- **No current app.** pycvc already has no module-global current app (`pycvc_context.h`), and
  the ports add none.

### 1.5 Parameters and data on the state tree

**The rule.** Every parameter a user or a document can set is a state key. Every input and result
a node holds is on the state tree. So a document, a DSL program, pycvc and an arisrv client all
see and set the same things. Nothing lives only in a C++ member or a widget.

**The parameter table.** `mesh_lab` today seeds and reads each key by hand: one
`seed_num(a, base, "smooth.iterations", ...)` per key in `mesh_nodes.cpp`. So each parameter's
name, default and type are written in three places: the struct, the seed call and the read. The
ports instead declare each params struct once, as a table (`inc/cvc/core/param_table.h`, P0):

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

Boost.Describe would do as the reflection underneath, if cvcpkg's Boost has it (1.77+). From one
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
| `skeleton` | params, `source.node`, `info.strands`, `info.sheets`, warnings | `<node>.skeleton.strands`, `<node>.skeleton.sheets` (geometry) |
| `contour_lab` | params, `source.node`, each component's name, colour and visibility | `<node>.tiler.contours` (`contour_stack`), `<node>.tiler.surfaces.<component>` (geometry) |
| `volume_grid` | params, slice indices, cursor, tool; class and object metadata (name, colour, radius) as lists a `repeat:` can bind | `<node>.grid.volume`, `<node>.grid.points` (`point_classes`), `<node>.grid.contours` (`contour_stack`), `<node>.grid.segment.labels`, `<node>.grid.segment.classes` |

Nodes read their inputs from the tree too. A node with `source: { node: <id> }` reads that node's
documented data key. That is how the GPL nodes use the LGPL nodes' data without either library
linking the other.

**When the tree is read and written.** The core algorithm never touches the tree. A job reads its
parameters when it starts, on the scheduler thread, and writes its results when it is done, in
`on_done`. Async kernels must work this way anyway: they never touch the state tree. A parameter
changed while a job runs applies to the next run, and `status` says so.

**Sessions.** A node's subtree is its session.
- `request: save_session <f>` writes the values as JSON (`state::json()`), and the data through
  the port's I/O handlers.
- `load_session` reverses it.

### 1.6 DSL intrinsics

**How they work.** Each port registers its operations as state_exec async functions
(`register_async_fn`; STATE_EXEC_DEVELOPER_GUIDE.md §6, "Async Functions"):
- the calling process parks while the kernel runs on the app's background pool;
- `<name>-async` returns a future;
- `call.post` sends progress to `<name>.progress`;
- killing the process cancels the job.

**Proposed intrinsics** (module-prefixed like the standard library; names are §9 item 7):

| Intrinsic | Library | Does |
|---|---|---|
| `(skeleton.compute g [params])` | GPL | Skeleton of a geometry. Returns a dict of `strands` and `sheets` data objects and `warnings` |
| `(tiler.tile stack [params])` | GPL | Tiles a `contour_stack`. Returns a dict from component name to geometry |
| `(segment.seeded vol classes [params])` | libcvc | §5's segmentation. Returns a dict of `labels`, `classes` and `border` |
| `(segment.critical-points vol [params])` | libcvc | The paper's seeds as a two-class `point_classes` |
| `(contours.read path)`, `(contours.write stack path)` | libcvc | `contour_stack` I/O, any format §3 lists |
| `(point-classes.read path)`, `(point-classes.write pc path)`, `(point-classes.add pc class i j k)` | libcvc | Point-class I/O and edits. These are small, so they are plain synchronous natives |
| `(params.defaults name)`, `(params.check name dict)` | libcvc | Defaults and validation from a parameter table, synchronous |

**Arguments and results**
- An input is a data object (from `state-data-get`) or a state path string such as
  `"doc.grid.volume"`. Paths are resolved in the marshal step, as the async rules require.
- Parameters are a dict, or a state path to a parameter subtree read through the table.
- Results are fresh data objects. The program stores them with `state-data-set`.

**Registration.** Each library has one function that takes the app and an environment:
`cvc::register_segmentation_intrinsics(app, env)` and `cvc::gpl::register_intrinsics(app, env)`.
Every host gets them: `cvc exec`, the app's exec scheduler, pycvc's `Exec`, and Ariadne's lanes
through the per-app provider list (§1.3).

**Budgets in Ariadne.** An Ariadne action runs with `ariadne.limits.action_max_time` (0.1 s) and
`action_max_steps` (5000). An async call's run time counts against `max_time`. So a skeleton or a
segmentation called from an `on:` action is killed unless the host raises those limits (ARIADNE.md
§2.2). Inside Ariadne, the node's `request` key stays the normal way to run them. The intrinsics
serve:
- scripts: `cvc exec`, pycvc's `Exec` and arisrv programs;
- hosts that raise the limits;
- tests.

Each intrinsic's documentation says so.

**What doesn't need intrinsics.** Node edits such as point toggles and contour edits are state
writes (`pick.*`, `request`), and a DSL program can already script them by writing keys.

### 1.7 Tests

**The rule.** Every port lands with every row of this matrix that applies to it. A phase isn't
done until its rows pass on Linux, Windows and macOS. The new test executables join the sanitizer
lanes' `SAN_CXX_TESTS` list, so they also run under the `tsan` and `asan-ubsan` lanes.

| # | Kind | What it checks | Examples |
|---|---|---|---|
| T1 | Legacy parity | The port's output equals the legacy library's on committed inputs. The legacy runs are scripted (`tools/volrover-goldens/`, at a pinned volrover sha), so the goldens can be regenerated | GenSeg labels voxel for voxel; skeleton strand and sheet counts, with vertices to 1e-9; tiler meshes equal after sorting the vertices |
| T2 | Units | Each internal piece on its own | cocone poles, the medial-axis filter, U1 clustering; tiler correspondence, branching, intersection removal; the fast-marching heap order, the 26-neighbour update, the gradient |
| T3 | Known shapes | Exact topology on synthetic inputs | a torus gives one closed strand; a slab one sheet; a capsule one segment; two cylinders' contours give two genus-0 tubes; two blobs give two labels |
| T4 | Invariants | Properties that hold for any input, over seeded generated inputs | the skeleton is unchanged by rigid motion and point order; a tiled surface is closed and manifold, does not intersect itself, and has every vertex on a contour plane; every labelled voxel connects to a seed of its class through voxels above `t_low`; labels never cross a ridge |
| T5 | Bad input | Exceptions with useful messages; never a crash or an exit | empty input, one point, coplanar points, one slice, an open or self-intersecting contour, no seeds, a seed outside the volume, a constant volume, NaN voxels, the wrong voxel type |
| T6 | No singletons | §1.3 | two threads with different parameters give the same results as each alone (TSan); A then B equals B then A equals each alone; two apps in one process |
| T7 | Jobs | A sliced job matches the blocking call at step budgets of 1, 64 and unbounded. A cancel in each phase leaves no result and no leak (ASan). Progress rises monotonically within [0, 1], and the phases are named | |
| T8 | State tree | For every field, struct to tree to struct round-trips. Defaults are seeded, properties written, and a bad value reported. A node's request produces its result at the documented data keys, with `status` | |
| T9 | I/O | Every reader and writer round-trips, the legacy sample files read, and a small corpus of truncated and garbage files raises | `.ser`, `.config` + `.pts`, `.contour`, `.pcd`, `.vgr`, `.pts` |
| T10 | DSL | Each intrinsic in a `state_exec` test, on an inline host and on the async scheduler, with a kill part-way through and a progress message; one Ariadne document using it with raised limits | |
| T11 | pycvc | Each function and params class from Python: keywords, numpy views, another Python thread running during the call (GIL released), and a job cancel | |
| T12 | CLI | Each subcommand on sample input, the legacy flag aliases, and the outputs read back | |
| T13 | Ariadne | A golden image per demo document; scripted picks through state writes (`volume_grid`); headless runs | |
| T14 | Licence boundary | libcvc, cvcGL and pycvc build with `CVC_ENABLE_GPL=OFF`; `CGAL_LICENSE_ERROR` is set on the LGPL targets; the object-file scan passes | |

ContourTiler's 12 switched-off CppUnitLite2 tests, with their expected outputs, are converted to
gtest and count toward T1 and T2. `test_intersection.cpp` alone is 2,252 lines.

### 1.8 How the legacy code goes in

This is done as for `geometry/SDF` and `geometry/cvc-mesher`.

- Vendor the legacy sources mostly as they are, under a subdirectory with its own
  `.clang-format` (off). Put a small adapter between them and libcvc's types, like
  `cvc-mesher/VolMagickCompat.h` and `LBIE/geoframe_adapter.cpp`. The public header is new, small
  and in libcvc's style (§1.4).
- Replace the legacy globals with the context struct (§1.3) before anything else, so that the
  parity tests (T1) run against code that can already run twice at once.
- Drop the legacy file formats from the algorithm path. Readers and writers become libcvc I/O
  handlers, registered per app (§1.3), and the algorithm takes in-memory types.

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

The result is `Simple_skel`: line strips plus polygons, each vertex with a colour. The only heavy
dependency is CGAL: EPICK plus `Delaunay_triangulation_3`, a GPL package, so it goes in `cvc_gpl`
(§1.2). There is no Qt (its CMake calls `SetupQt()` for nothing), and no GL, GSL or VolMagick. No
paper is cited in the code. The Dey–Goswami cocone and Goswami–Dey–Bajaj unstable-manifold
lineage is inferred.

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
each. Port them once, as an internal `cvc::cocone` module in `cvc_gpl`. A later TightCocone port
(surface reconstruction from points) and PocketTunnel then reuse it, rather than vendoring a
fifth copy.

**API sketch** (`inc/cvc/gpl/skeleton.h`)

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
std::unique_ptr<result_job<skeleton>> skeletonize_job(geometry in, skeleton_params p);
}
```

The defaults match the legacy `Parameters` (`Skeletonization.h:42-54`) and the dialog.

**Surfaces**

- **CLI:** `cvc-gpl skeleton in.{off,ply,raw,rawc} -o out.{vtk,raw} [--robust ...]`. The legacy
  tool's flags (`-r -bbr -mr -hc -thif -thff -plc -medth -medr`, from `main.cpp.unused`) are
  accepted as aliases.
- **pycvc:** `pycvc_gpl.skeletonize(geom, robust=True, ...)` and `skeletonize_job`.
- **DSL:** `(skeleton.compute g params)`.
- **Ariadne:** a `skeleton` node type in `cvcgl_gpl`.
  - It reads its input from `source: { node: <mesh_lab id> }`, through that node's geometry data
    key, or from `source: { file }`.
  - Its parameters live under `<node>.skeleton.*`, and its results on the data channel (§1.5).
  - It adds two children, `<id>_strands` and `<id>_sheets`, tagged `skeleton`, so a `viewport`
    can show the skeleton alone.
  - Its component, `skeleton.ari`, is listed only where the node type is registered. Check how
    the component index gates a component on a node type.
  - A volume reaches it through an isosurface, as `fe_lab` does today.
  - It is a node of its own, not a request on `mesh_lab`, because `mesh_lab` is in LGPL cvcGL.

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1 | strand and sheet counts and vertices against the legacy library on three committed point clouds |
| T2 | poles, cocone flags, medial-axis filter, U1 clustering and U2 strands, one at a time |
| T3 | torus, slab, capsule, and two spheres joined by a rod |
| T4 | unchanged by rigid motion and point order |
| T5 | fewer than four points, coplanar and collinear points, duplicates |
| T6–T14 | as the matrix |

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
    libcvc's. It uses GPL CGAL packages, so it goes in `cvc_gpl` (§1.2).
  - Size: about 24k source and 13k header lines, tests and tools included.
  - No Qt, GL, VolMagick or GSL. Errors are exceptions, not `exit()`, apart from
    `smfconvert.cpp`, a tool file that sits in the library's source list.

**ContourTiler's rough edges, and what the port does with them**

| As it is | In libcvc |
|---|---|
| `tile(begin, end, comp2color, options)` mutates the input slices and writes `.rawc` files; it returns nothing (`tiler.cpp:555-576`) | Takes a const `contour_stack` and returns one `cvc::geometry` per component; files only from the CLI |
| log4cplus in 48 files: 156 `getInstance` calls, 104 function-static loggers | A shim whose logger is a member of `tiler_context`, passed to the functions that log (§1.3); no process-wide logger |
| Two `.ser` readers. The app's goes through ContourFilter's `Controls::instance()` singleton, which is reconfigured on every read, has hard-coded ignored names, and drops a whole component if one contour is non-simple. The CLI's uses tinyxml | Port the tinyxml reader into `contour_stack` I/O. The singleton stays behind. ContourFilter's smoothing waits until someone needs it |
| Exact number types for the boolean ops (`Lazy_exact_nt<Gmpz>`, `Gmpq`, `MP_Float`), so GMP and MPFR (LGPL, but with no wasm build) | Native only at first (§9 item 1) |
| GCC-only flags baked in (`-g -pthread -frounding-math`, `-rdynamic`) | `SetupCGAL` does what is needed per compiler |
| 12 `test_*.cpp` on CppUnitLite2, switched off (`test_intersection.cpp` alone is 2,252 lines, with expected outputs) | Converted to gtest: T1 and T2 coverage for free |
| `cvcraw_geometry::contours_t` stores `ContourTiler::Slice`, a reverse dependency | `cvc::contour_stack`, in libcvc, is the shared type, and the tiler converts at its edge |
| Static counters for debug file names, and `_otv.g` / `_verts.g` debug outputs | Dropped, or a `debug_dir` parameter |

**The shared contour type** (`inc/cvc/geometry/contours.h`, in libcvc), used by the tiler and by
VolumeGridRover:

- slices keyed by z index, with a z spacing (the legacy hard-codes 0.05 in
  `cvcraw_geometry/contours.cpp:72-83`) and an origin;
- each slice maps a component name to closed planar polygons;
- each component has a colour.
- **Readers:** Reconstruct `.ser` plus its numbered section files, and the Tiling `.config` plus
  `.pts`.
- **Writers:** Tao Ju's `.contour` (ctr2suf), `.pcd`, and a libcvc-native file for
  VolumeGridRover sessions.
- I/O handlers registered per app (§1.3), with `contours.read` / `contours.write` intrinsics
  and T9 corpora.

**API sketch** (`inc/cvc/gpl/tiler.h`)

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
std::unique_ptr<result_job<tile_result>> tile_contours_job(contour_stack in, tile_params p);
}
```

One step of the job is one slice pair of one component, so progress is real.

**Surfaces**

- **CLI:** `cvc-gpl tile in.ser -o outdir`, keeping `cl_parse`'s flags. The VolumeRover2 dialog
  passes them as free text, by default `-C 0.01 -e 1e-15 -o raw`.
- **pycvc:** `pycvc_gpl.tile_contours(stack, ...)` and `tile_contours_job`.
- **DSL:** `(tiler.tile stack params)`.
- **Ariadne:** a `contour_lab` node type in `cvcgl_gpl`.
  - Its input is a file (`source: { file }`), or another node's contours from the state tree
    (`source: { node: <volume_grid id> }`, which reads `<grid>.grid.contours`).
  - It draws a stack's contours as polylines in their component colours (tag `contours`). On a
    `tile` request it adds the surfaces as children (tag `tiles`).
  - Its keys live under `<node>.tiler.*`, and its surfaces on the data channel.
  - The `contour_tiler.ari` component has a component picker, the slice range, the parameters,
    Tile and Stop.

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1–T2 | the 12 converted legacy tests; meshes against the legacy library on two committed `.ser` series |
| T3 | a cylinder stack gives a genus-0 tube; a branching Y gives one surface; two touching components with `remove_intersections` give two disjoint surfaces |
| T4 | closed and manifold, no self-intersection, every vertex on a contour plane, volume within 2% of the prism estimate |
| T5 | one slice, empty slices, an open contour, a self-intersecting contour, reversed orientation |
| T9 | every `contour_stack` format round-trips; a Reconstruct series reads |
| T6–T14 | as the matrix |

---

## 4. VolumeGridRover as an Ariadne component

**What the legacy tool is today.** It is a 7,002-line `VolumeGridRover.cpp`.

- **Views:** XY, XZ and ZY slice views shown one at a time in tabs. Each is a `SliceCanvas` (a
  QGLViewer) with an orthographic camera, no rotation, middle-button zoom, a depth slider and
  Reset View.
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
None of its Qt or GL code carries over. Its data model, file formats, interaction design and a
few algorithms do. volrover's history from before the Qt6 migration may hold the last working
contour and tiling code. Read it before P10; the shallow clone used for this sketch has none of
it.

**The node: `type: volume_grid`** (`inc/cvc/gl/ariadne/volume_grid_nodes.h`, in LGPL cvcGL)

- It reads `source: { file, var, time }` as a volume node does.
- **Children:**
  - three `volume_slice` nodes (§6 item 1), tagged `slice_xy`, `slice_xz` and `slice_zy`;
  - per slice, overlays carrying the same tag: point-class glyphs, contour polylines, crosshairs
    where the other two planes cross, and the segmentation's labels as a see-through colour
    layer;
  - a `<id>_surfaces` group for the segmented classes' surfaces (tag `surfaces`). Tiled surfaces
    come from the `contour_lab` node and carry the same tag.
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
| `slice.x`, `slice.y`, `slice.z` | voxel indices; written by the sliders, the keys and clicks |
| `cursor.{i,j,k,x,y,z,value}` | published on hover: the probe |
| `tool` | `navigate`, `points`, `contours`, `probe` |
| `pick.*` | Written by the views (`pick:` on the viewport, §6 item 3). The tick consumes each new pick according to `tool`: `points` toggles a point (it adds one, or removes the nearest within `display.point_size` on this slice); `contours` appends to the current object's contour on this slice, at its nearer end; a Shift-drag selects with a rubber band |
| `classes`, `classes.current` | point-class metadata: name, colour, radius, as a list a `repeat:` can bind |
| `objects`, `objects.current` | contour-object metadata: name, colour, interpolation (`linear` or `spline`), samples |
| `display.point_size`, `display.show_points`, `display.grey`, `display.background` | display (a parameter table) |
| `segment.t_low`, `segment.t_high`, `segment.gamma`, `segment.seeds` (`none` for the point classes alone, `local_maxima`, `critical_points`), `segment.show`, `segment.opacity` | §5's parameters, from `param_table<seeded_segmentation_params>`, and how the labels draw |
| `request` | `load`, `save_classes <f>`, `load_classes <f>`, `load_points <f>`, `save_contours <f>`, `load_contours <f>`, `delete_selected`, `segment`, `save_segments <f>`, `save_session <f>`, `load_session <f>`, `undo`, `stop` |
| `busy`, `status`, `progress`, `phase` | published |
| data: `volume`, `points`, `contours`, `segment.labels`, `segment.classes` | the loaded volume, the `point_classes`, the `contour_stack`, the segmentation's results (§1.5) |
| `<node>.transfer_function` (+ `.histogram` …) | as on every volume node, so the `tf_editor` colours the slices |

**Data and algorithms**

- **Point classes:** a small `cvc::point_classes` type in libcvc (classes × voxel points, per
  variable and timestep), with `.vgr` read and write and a `.pts` reader. The legacy `.vgr` form
  is `<pointclassdoc><pointclass name color variable timestep><point>"x y z"</point>`. The
  points live on the data channel; the class metadata are keys.
- **Contours:** the §3 `contour_stack`, one component per object.
- **Splines:** the legacy resampled with GSL splines every frame. libcvc does not link GSL on
  wasm, so use a small natural-cubic or Catmull-Rom spline in libcvc, computed when a contour
  changes.
- **Tile, through the state tree.** `volume_grid` is LGPL and never links the tiler (§1.2). The
  `volume_grid` component takes a `tiler:` arg naming a `contour_lab` node whose source is this
  grid. When the arg is given, the component shows a Tile button that writes `request: tile` to
  that node. The surfaces appear in the 3D view in the objects' colours. This finally wires up the
  `tilingComplete` the legacy declared and never emitted.
- **Ported later, on demand:**
  - SDF2D, the 2D signed distance of contour polygons (CGAL k-nearest-neighbour search, so GPL),
    for a "show SDF" overlay;
  - `medax.h`, a 2D medial axis through CGAL's segment Delaunay graph (GPL);
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
  - a 2×2 grid of viewports: XY, XZ and ZY in `mode: slice` with `input: pick`, and a 3D orbit
    view;
  - under each slice view, a slider bound to `grid.slice.{z,y,x}`, with the index in the view's
    caption;
  - a side panel with tabs:
    - Probe;
    - Point classes: list, colour, radius, add, delete, save, load;
    - Contours: objects, colour, interpolation, samples, save, load, and Tile when `tiler:` is
      given;
    - Segmentation: thresholds, γ, seed mode, Segment, Stop, label opacity, save;
    - Transfer function: the `transfer_function_editor` unit.
  - Slider ranges and labels come from the keys' properties (§1.5), not from the component.
- **`volume_grid_views`** and **`volume_grid_tools`:** the views alone and the panel alone, for a
  document that lays them out itself.
- **Args:** `node`, `title`, `id`, `view_size`, `layout` (`quad`, or `tabs` like the legacy),
  `tiler`.
- **Demo:** `examples/ariadne/documents/volume_grid.ari` on a shipped volume, with a golden image
  and an interaction test that scripts picks through state writes. A second demo adds a
  `contour_lab` for Tile, where the GPL extensions are built.

---

## 5. Seeded segmentation (GenSeg)

**What it is.** `src/Segmentation/GenSeg` is about 2.8k lines; `segment.cpp` is 1,969 of them. It
is CVC code under the LGPL, uses no CGAL, and so stays in libcvc. Its headers ask users to cite
the paper it implements:

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
  seed_mode automatic = seed_mode::local_maxima;  // added to the user's seeds
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
std::unique_ptr<result_job<seeded_segmentation>>
segment_seeded_job(volume in, point_classes seeds, seeded_segmentation_params p);
point_classes critical_point_seeds(const volume &, const seeded_segmentation_params & = {});
}
```

One step of the job is a fixed number of heap pops, so a big volume marches without stalling a
frame.

**Order of work**

1. **Match the legacy output.** Vendor GenSeg's marching into `genseg_context`, with in-memory
   input and output. Keep the hand-unrolled 26-neighbour update as it is for this release.
   - Run the legacy code once on small synthetic volumes with fixed seeds: two blobs, a blob
     touching a shell, and a noisy one. The `tools/volrover-goldens/` script does this and
     commits the expected label volumes.
   - The port must match them voxel for voxel before any clean-up.
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

**Surfaces**

- **CLI:** `cvc segment in.rawiv --seeds classes.vgr --tlow 40 --thigh 200 -o out`. It writes the
  labels volume and the per-class volumes. `--auto critical-points` runs the paper's mode with no
  seeds file.
- **pycvc:** `pycvc.segment_seeded(vol, seeds, t_low=40, ...)`, `segment_seeded_job` and
  `critical_point_seeds`.
- **DSL:** `(segment.seeded vol classes params)` and `(segment.critical-points vol params)`.
- **Ariadne:** the Segmentation tab and the `segment` request in `volume_grid` (§4).

**Tests** (§1.7)

| Row | Cases |
|---|---|
| T1 | voxel-for-voxel labels against the legacy library on the three committed volumes; SegMed's critical points against its legacy output |
| T2 | heap order, the 26-neighbour update, the gradient, the 7-point smoothing, the threshold sweep |
| T3 | two blobs give two labels; one seed of each class in one blob splits at the ridge; a shell around a blob gives two nested labels |
| T4 | every labelled voxel connects to a seed of its class through voxels above `t_low`; no label crosses a ridge above `t_high`; border voxels touch two classes |
| T5 | no seeds, a seed outside the volume, a seed below `t_low`, a constant volume, NaN voxels, every voxel type |
| T6–T13 | as the matrix |

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
  The ports register theirs through the per-app registry P0 adds (§1.3).
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
2. **A slice camera mode for the viewport** (`mode: slice`, `axis: x | y | z`): parallel
   projection looking down the axis, no rotation; drag pans, the wheel zooms about the pointer,
   and double-click refits the slice.
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
4. **Overlays that stay on top of a slice:** point glyphs, polylines and crosshairs drawn as child
   geometry nodes of the slice, with a coincident-topology offset (or a later render layer) so
   they never z-fight the textured quad. `point_size` / `line_width` already exist as override
   fields.
5. **Widgets the side panel needs** (check each against the backend before planning around it):
   a list bound to a state array with add and remove (the `repeat:` the card component uses), a
   colour picker for a class colour, and radio buttons for the tool. Anything missing is a small
   ImGui widget in `ImGuiBackend.cpp` with an FTXUI placeholder.

Items 2 and 3 are the critical path for VolumeGridRover. Item 1 is the bulk of the rendering
work. Items 4 and 5 are small.

---

## 7. Phases

Sizes are relative: S is a PR or two, M a few, L a short series. Every phase lands with its
§1.7 rows, its pycvc bindings and its DSL intrinsics, not as follow-ups.

| Phase | Size | What lands | Needs |
|---|---|---|---|
| P0a Shared pieces | M | `cvc::param_table` (§1.5), with its state, property, CLI, pycvc and DSL generators and its own T8 tests; `cvc::result_job<R>` (§1.4); per-app registries for Ariadne node types, action intrinsics and the volume / geometry I/O handlers (§1.3), used by new code; pycvc's generic `result_job` wrapper; the object-file singleton scan in CI; `tools/volrover-goldens/` (builds the legacy libraries at a pinned volrover sha and writes the T1 goldens) | — |
| P0b GPL library | S | The `cvc_gpl`, `cvcgl_gpl` and `pycvc_gpl` targets and `cvc-gpl` behind `CVC_ENABLE_GPL`; `CGAL_LICENSE_ERROR` on the new LGPL directories; a CI lane with `CVC_ENABLE_GPL=OFF` (T14); COPYING and README; the packaging chosen in §9 item 4 | — |
| P0c Shared types | S | `cvc::contour_stack` and `cvc::point_classes`, with readers and writers registered per app, their parameter tables, intrinsics (`contours.*`, `point-classes.*`, `params.*`), pycvc, and T6, T8, T9 and T11; a spike building CGAL's `Delaunay_triangulation_3` for wasm-mt with `CGAL_DISABLE_GMP`, and Boost.Multiprecision as the exact fallback | P0a |
| P1 Skeleton core | M | In `cvc_gpl`: vendor into `src/cvc_gpl/skeleton/` with the shared cocone core and `skeleton_context`; the §2 fixes; `cvc::skeletonize` and its job; `cvc-gpl skeleton`; `pycvc_gpl`; `skeleton.compute`; §2's tests | P0a, P0b |
| P2 Skeleton in Ariadne | S | The `skeleton` node in `cvcgl_gpl` reading a `mesh_lab` node from the state tree; `skeleton.ari`; a demo document with a golden image | P1 |
| P3 Tiler core | L | In `cvc_gpl`: vendor into `src/cvc_gpl/tiler/` with the context-held logger; in-memory output, const input; its 12 tests as gtest; `cvc-gpl tile`; `pycvc_gpl`; `tiler.tile`; §3's tests | P0a, P0b, P0c |
| P4 Tiler in Ariadne | S | `contour_lab` in `cvcgl_gpl`, reading a file or another node's contours, and `contour_tiler.ari`, with a demo on a small redistributable Reconstruct series (find one) | P3 |
| P5 Segmentation core | M | §5 steps 1–2: GenSeg vendored into `src/cvc/volume/segmentation/` with `genseg_context` and in-memory input and output; voxel-for-voxel parity, then the clean-up; `cvc segment`; pycvc; `segment.seeded`; §5's tests. CGAL-free, so it runs on wasm | P0a, P0c |
| P6 Paper mode | S–M | §5 steps 3–4: the SegMed audit, `cvc::gradient_vector_diffusion`, `seed_mode::critical_points` and `critical_point_seeds`, `segment.critical-points`, and #123's seed generators | P5 |
| P7 Viewport pieces | M | §6: the `volume_slice` node, `mode: slice`, `input: pick` / `pick:` / `on_pick:`, the overlay offset, the missing widgets. Each with tests, plus an orthoslicer demo next to `volren_bunny` | P0a (registries) |
| P8 volume_grid v1 | M | Navigation, probe, point classes (mark, select, move, delete, save, load), sessions, the transfer function, quad or tab layout; parameters from tables and data on the data channel; golden image and scripted-pick tests; two grids in one document (T6) | P7, P0c |
| P9 volume_grid v2 | M | Segmentation tab: `segment`, labels on the slices, class surfaces in the 3D view, `save_segments` | P8, P5 |
| P10 volume_grid v3 | M | Contours: draw and edit per object, linear and spline, export, published at `grid.contours`; the `tiler:` arg and Tile through a `contour_lab` | P8, P4 |
| P11 Web and wrap-up | S–M | arigl-web for the LGPL side (`volume_grid` with Segment but without Tile), and the GPL side only if the P0c spike worked and §9 item 1 allows it. Docs: ARIADNE.md §5.4 rows, INDEX.ari entries, the generated parameter tables. volrover3 picks it all up through libcvc. Retire the legacy libraries from volrover `master` once parity holds, together with #123's VolUtils leftovers | P2, P4, P9, P10 |

**Parallel tracks.** Once P0 is in, P1, P3, P5 and P7 share no files and need nothing from each
other, so they can run as parallel sessions. P0a, P0b and P0c can also overlap: P0c only needs
P0a's parameter table.

**Separate items afterwards:**
- SDF2D, the 2D medial axis and contour curation, in `cvc_gpl`;
- TightCocone and PocketTunnel on the shared cocone core, in `cvc_gpl`;
- the virus segmentations (SegCapsid, SegSubunit, SegMonomer) and SecStruct, on §5's fast
  marching and gradient vector diffusion;
- the paper's §2.3 volumetric skeletons;
- moving existing callers off the process-wide registries (§9 item 3).

---

## 8. What is deliberately left behind

- **The old `src/Tiling` code:** its licence, the globals, the `exit()`s and the temp files. Its
  contour format survives as a reader.
- **All of VolumeGridRover's Qt and GL:**
  - `SliceCanvas`, the three slice renderers (ARB fragment program, paletted texture, SGI colour
    table), immediate mode, `GL_SELECT` picking;
  - the `QThread` workers that read widgets off the GUI thread;
  - the `QSettings` cache directory and the temp files in the current directory.
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
- **GenSeg's XML-RPC remote mode and its file round trip.** A remote run is the node on an arisrv
  host.
- **Skeletonization's helix-specific strand window and its `#if 0` helix block.** The window
  becomes an off-by-default parameter. The block fits better with a SecondaryStructures port.

---

## 9. Decisions for Joe

**Settled on 2026-11-08**
- ContourTiler is LGPL like libcvc.
- GenSeg is in scope (§5).
- The GPL-licensed parts go in a separate library (§1.2).
- Every port has no singletons (§1.3) and an API pycvc wraps (§1.4).
- Every port keeps its parameters and data on the state tree (§1.5), has DSL intrinsics (§1.6),
  and carries the §1.7 tests.

**Still open**

1. **The web.** CGAL is off on wasm today (`DISABLE_CGAL=ON`), because GMP and MPFR have no
   wasm-mt build. That is about availability, not licence. A web build of the GPL side would be
   a GPL web bundle of its own. The options:
   - ship skeletons and tiling native-only;
   - try the P0c spike. Skeletonization needs only EPICK plus 3D Delaunay, which should build
     without GMP. ContourTiler's exact boolean ops are much harder.

   **Recommended:** native-only, and run the spike for Skeletonization alone.
2. **libcvc's existing GPL CGAL uses.** `nav/spatial.cpp` and `project_verts.h` use CGAL's
   Spatial Searching (GPL) whenever CGAL is found. The options:
   - (a) move them into `cvc_gpl`;
   - (b) replace them with a permissive k-nearest-neighbour search, such as nanoflann (BSD;
     check that it is in cvcpkg).

   **Recommended: (b).** nav is core and shouldn't need the GPL library. Then
   `CGAL_LICENSE_ERROR` can go on all of libcvc. This is its own PR, independent of the ports.
3. **The process-wide registries.** P0a adds per-app forms of Ariadne's node-type and intrinsic
   registries and of the I/O handler maps, and the ports use only those. Should every existing
   caller move too, in this effort? **Recommended:** later, as its own PR series. The existing
   callers keep working, and the ports never touch the process-wide forms.
4. **GPL packaging.** The existing packages stay LGPL. The options for the GPL side:
   - (a) a `libcvc-gpl` cvcpkg package with `cvc_gpl`, `cvcgl_gpl`, `pycvc_gpl`, the `cvc-gpl`
     CLI, and an `arigl-gpl` that registers the GPL node types;
   - (b) one build switch that links the GPL libraries into `cvc` and `arigl` themselves, which
     makes those binaries GPL in that build.

   **Recommended: (a).** Distinct names for distinct licences, so nobody ships GPL code under an
   LGPL package by accident.
5. **The VolumeGridRover layout.** Four views at once (recommended), or the legacy tabs. The
   `layout` arg can offer both.
6. **GenSeg's default seeding.** Once the paper mode lands, should `volume_grid` default to it or
   to GenSeg's local maxima? **Recommended:** keep GenSeg's local maxima until the paper mode
   matches the legacy results on real maps, then switch.
7. **Names.**
   - Libraries and tools: `cvc_gpl`, `cvcgl_gpl`, `pycvc_gpl`, `cvc-gpl`, `arigl-gpl`.
   - Node types: `volume_grid`, `contour_lab`, `volume_slice`, `skeleton`.
   - Functions: `cvc::skeletonize`, `cvc::tile_contours`, `cvc::segment_seeded`,
     `cvc::critical_point_seeds`.
   - Commands: `cvc segment`, `cvc-gpl skeleton`, `cvc-gpl tile`.
   - The intrinsics in §1.6.
