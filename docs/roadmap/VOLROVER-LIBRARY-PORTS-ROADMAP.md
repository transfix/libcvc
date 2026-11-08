# VolumeRover Library Ports — Roadmap (sketch)

**Status:** sketch, 2026-11-07. Nothing has landed and no decisions are final; §8 lists the ones
that need Joe. **Based on:** libcvc `origin/master @ 7a09366f` and volrover `master @ b4a172c`
(the legacy libraries live under `src/<Lib>` and `inc/<Lib>` there).
**Scope:** three legacy VolumeRover libraries, Skeletonization, Tiling (with its successor
ContourTiler) and VolumeGridRover, ported into libcvc the way `mesh_lab` / `fe_lab` were: a
headless `cvc::` API, a `cvc` CLI command, pycvc bindings and an Ariadne node type with a
component. VolumeGridRover becomes an Ariadne component with embedded GL views.
**Companions:** [ARIADNE.md](../ARIADNE.md) §5.4 (custom node types), §5.5 (request keys),
§5.18 (the `viewport` widget, `scenes:`, named cameras), §6.5 (the component library);
[LIBIGL-GEOMETRY-FE-ROADMAP.md](LIBIGL-GEOMETRY-FE-ROADMAP.md) (the `mesh_lab` / `fe_lab`
precedent); [TRANSFER_FUNCTIONS.md](../TRANSFER_FUNCTIONS.md) (the `tf_editor`, the ColorTable2
port: the last legacy Qt widget turned into an Ariadne widget); transfix/libcvc#123 (the other
VolUtils leftovers).

---

## 0. Summary

- **Skeletonization** (§2) is a clean port. It is about 8k lines of LGPL-2.1 code whose only
  real dependency is CGAL's 3D Delaunay. It needs a global-state fix, a few bug fixes, and its
  sheets kept rather than dropped. It lands as `cvc::skeletonize`, `cvc skeleton`, and a
  `skeleton` request on `mesh_lab`. Its cocone code is copied almost verbatim in TightCocone,
  PocketTunnel and Curation, so it becomes one shared `cvc::cocone` core.
- **Tiling** (§3) means ContourTiler, not the old `src/Tiling`. The old tiler has not been built
  since 2011 and carries a Purdue SHASTRA notice that forbids commercial use. ContourTiler
  reimplements the same 1996 algorithm for many components and is the one VolumeRover2 uses. It
  is heavy CGAL (exact boolean ops, so GMP and MPFR). It lands as `cvc::tile_contours`,
  `cvc tile`, and a `contour_lab` node, on a new shared `cvc::contour_stack` type.
- **VolumeGridRover** (§4) is a rebuild, not a translation. Its Qt6 code is mostly stubs:
  contours cannot be created, and tiling, SDF and medial axis are commented out. Its GL is
  2005-era. What carries over is the design: three axis slice views, a voxel probe, point
  classes, contours drawn per object, and tiling them into surfaces. It becomes a `volume_grid`
  node plus a `volume_grid.ari` component of four embedded `viewport`s.
- **Most of the GL viewer already exists** (§5): the `viewport` widget, tags, named cameras and
  `scenes:`. Four new pieces are needed, all reusable:
  1. an axis-aligned `volume_slice` node;
  2. a `mode: slice` orthographic camera;
  3. pointer events from a viewport into state (the open "click-to-world picking" item);
  4. overlays that stay on top of a slice.
- **Order** (§6): three tracks can start at once, each with nothing to wait on: the
  Skeletonization core, the ContourTiler core, and the Ariadne viewport pieces. VolumeGridRover
  follows the viewport pieces, and its Tile button follows the tiler.
- **Decisions for Joe** (§8): CGAL's GPL packages versus the LGPL bundle, ContourTiler's missing
  licence header, native-only versus a wasm spike, and segmentation in or out.

---

## 1. The shape every port takes

The `mesh_lab` / `fe_lab` work set the pattern. Each library lands as up to five layers, and
each layer is useful on its own:

| Layer | Where | What it is |
|---|---|---|
| 1. Core | `src/cvc/<area>/<lib>/` (vendored legacy code) behind `inc/cvc/<area>/<lib>.h` | A headless `cvc::` API on `cvc::volume` / `cvc::geometry`. No Qt, no GL, no globals, no `exit()`, no config files read inside the algorithm. Long jobs are `cvc::sliced_job`s ([`sliced_task.h`](../../inc/cvc/core/sliced_task.h)) so they run cancellable and with progress on the background pool natively and a few ms a frame on single-threaded wasm. |
| 2. CLI | `src/cvc-cli/main.cpp` | One `cvc` subcommand per library (`cvc skeleton`, `cvc tile`), the legacy executables' options kept. |
| 3. Python | `bindings/pycvc` | The core API, so notebooks and GRL-SNAM-style scripts can call it. |
| 4. Node type | `src/cvcGL/ariadne/` + `inc/cvc/gl/ariadne/`, registered in `register_cvcgl_extensions` | A custom scene node (`register_scene_node_type`) that owns the data, runs the core on its own `cvc::async_lane` / the state_exec scheduler, and publishes `busy`, `status`, `progress`, `phase` beside its parameters under `<node>.<ns>.*`, seeded from the node's props. Commands are a `request` key (ARIADNE.md §5.5). |
| 5. Component | `src/cvc/ariadne/components/<name>.ari` + an `INDEX.ari` entry | A window that binds those keys, so a document drives the node with no host C++. Args `node`, `title`, `id` like every node-scoped component since b2f54578. |

How the legacy code goes in, as with `geometry/SDF` and `geometry/cvc-mesher`:

- Vendor the legacy sources mostly as they are, under a subdirectory with its own
  `.clang-format` (off), and put a small adapter between them and libcvc's types (like
  `cvc-mesher/VolMagickCompat.h` and `LBIE/geoframe_adapter.cpp`). The public header is new,
  small and in libcvc's style.
- Replace legacy global state with a context struct the entry point owns, so two jobs can run at
  once (two nodes, or a node and a CLI call in one process).
- Drop the legacy file formats from the algorithm path. Readers and writers become libcvc I/O
  handlers next to the existing ones, and the algorithm takes in-memory types.
- Each port lands with gtests on small synthetic inputs (exact topology checks, not just "it
  ran"), a CLI test, and a golden image for its demo document (ARIADNE.md §12).

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

The result is `Simple_skel`: line strips plus polygons, each vertex with a colour. The only
heavy dependency is CGAL (EPICK plus `Delaunay_triangulation_3`). There is no Qt (its CMake calls
`SetupQt()` for nothing), and no GL, GSL or VolMagick. No paper is cited in the code. The
Dey–Goswami cocone and Goswami–Dey–Bajaj unstable-manifold lineage is inferred.

**What the port fixes on the way in**

- A global `bounding_box` vector gets six `push_back`s per call and is never cleared, so every
  call after the first uses the first call's box. It moves into a context struct.
- `srand(0)` / `rand()` pick the colours, which changes the process's C random state. Colours
  become deterministic per cluster, or are left to the renderer.
- Degenerate cases write debug files (`degen_tetra`, `*.degen`, `*.degen_VF`) to the current
  directory. They become warnings in `status`.
- The `exit(1)` paths are only in debug writers and `main.cpp.unused`, so they go with them.
- The strand pass keeps Voronoi-vertex strips only when the cell radius is in a hard-coded
  (2.0, 3.0) window. That is protein-helix specific: make it an off-by-default option, or leave
  it to a SecondaryStructures port.
- The VolumeRover2 dialog never initialises Flat Phi (`SkeletonizationDialog.cpp:87-90`) and
  hard-codes `DEFAULT_SHARP`. Both become ordinary parameters.
- VolumeRover2 drops the sheets and keeps only strand vertices, with no connectivity. The port
  returns both. A sheet polygon looks like a Voronoi face, which is convex, so a fan would
  triangulate it (check this). `PolyTess`, the legacy ear-clipper, is unused and carries a
  non-commercial licence, so it stays behind.

**One cocone core, not four.** Skeletonization's `tcocone.cpp`, `robust_cc.cpp`,
`intersect.cpp` and `init.cpp` differ from the copies in TightCocone, PocketTunnel and Curation by
10–25 lines each. Port them once as an internal `cvc::cocone` module. A later TightCocone port
(surface reconstruction from points) and PocketTunnel then reuse it rather than vendoring a
fifth copy.

**API sketch** (`inc/cvc/geometry/skeleton.h`)

```cpp
namespace cvc {
struct skeleton_params {
  bool robust = false;                         // Robust Cocone pre-filter
  double big_ball_ratio = 1.0 / 16, theta_if = 5 deg, theta_ff = 10 deg;
  double cocone_phi = pi / 8, sharp_phi = 2 * pi / 3, flat_phi = pi / 3, flatness_ratio = 1.44;
  double medial_theta = pi / 8, medial_ratio = 64;
  bool discard_small = false; double threshold = 0.1; int min_cluster_count = 2;
};
struct skeleton {
  geometry strands;   // lines() + per-vertex colours
  geometry sheets;    // the planar patches, triangulated; one colour per patch
  std::vector<int> sheet_of_tri, strand_of_line;
};
skeleton skeletonize(const geometry &points_or_mesh, const skeleton_params & = {});
std::unique_ptr<sliced_job> skeletonize_job(geometry in, skeleton_params p, skeleton &out);
}
```

The defaults match the legacy `Parameters` (`Skeletonization.h:42-54`) and the dialog.

**Surfaces**

- **CLI:** `cvc skeleton in.{off,ply,raw,rawc} -o out.{vtk,raw} [--robust ...]`. The legacy
  tool's flags (`-r -bbr -mr -hc -thif -thff -plc -medth -medr`, from `main.cpp.unused`) are
  accepted as aliases.
- **pycvc:** `cvc.skeletonize(geom, **params)`.
- **Ariadne:** a `skeleton` request on `mesh_lab`, with its parameters under
  `<node>.mesh_ops.skeleton.*`. It adds two children, `<id>_strands` and `<id>_sheets`, tagged
  `skeleton`, so a `viewport` can show the skeleton alone. `mesh_tools.ari` gains a Skeleton
  section. A volume reaches it through an isosurface, as fe_lab does today.

---

## 3. Contour tiling

There are two legacy tilers, and only one is worth porting.

- **`src/Tiling`: do not port it.** This is the 1996 Bajaj–Coyle–Lin tiler ("Arbitrary
  Topology Shape Reconstruction from Planar Cross Sections", GMIP 58(6)).
  - No CMakeLists has added its directory since the 2011 switch to ContourTiler, and nothing
    active calls it.
  - Its code is SHASTRA/Purdue software whose notice (`main.cpp:1-12`) allows educational use
    only and forbids commercial use, so it cannot go into an LGPL library.
  - It also has about 35 mutable globals and 136 statics with a partial reset, 75 `exit()`
    calls in library code, and a temp-file round trip inside `surfaceFromContour`.
  - Keep only its contour file format (`.config` with `PREFIX` / `SUFFIX` / `SLICE_RANGE`, plus
    numbered `.pts` files) as a reader. That is a format, not code.
- **`src/ContourTiler`: port this one.** It is John Edwards' reimplementation of the same
  algorithm, extended to many components with removal of intersections between components
  (the README targets neuropil "AxonsDendrites" serial-section data). It is built by default
  and reached from VolumeRover2's Tools > Contour Tiler.
  - Size: about 24k source and 13k header lines, tests and tools included.
  - No Qt, GL, VolMagick or GSL. Errors are exceptions, not `exit()`, apart from `smfconvert.cpp`,
    a tool file that sits in the library's source list.

**ContourTiler's rough edges, and what the port does with them**

| As it is | In libcvc |
|---|---|
| `tile(begin, end, comp2color, options)` mutates the input slices and writes `.rawc` files; it returns nothing (`tiler.cpp:555-576`) | Takes a const `contour_stack` and returns one `cvc::geometry` per component; files only from the CLI |
| log4cplus in 48 files: 156 `getInstance` calls, 104 function-static loggers | A shim header mapping its logger calls to libcvc's log, so each vendored file changes by one include |
| Two `.ser` readers: the app's goes through ContourFilter's `Controls::instance()` singleton (reconfigured on every read, hard-coded ignored names, drops a whole component if one contour is non-simple); the CLI's uses tinyxml | Port the tinyxml reader into `contour_stack` I/O. ContourFilter's smoothing waits until someone needs it |
| Exact number types for the boolean ops (`Lazy_exact_nt<Gmpz>`, `Gmpq`, `MP_Float`), so GMP and MPFR | Native only at first (§8 item 3) |
| GCC-only flags baked in (`-g -pthread -frounding-math`, `-rdynamic`) | `SetupCGAL` does what is needed per compiler |
| 12 `test_*.cpp` on CppUnitLite2, switched off (`test_intersection.cpp` alone is 2,252 lines, with expected outputs) | Converted to gtest: free regression coverage for the port |
| `cvcraw_geometry::contours_t` stores `ContourTiler::Slice`, a reverse dependency | `cvc::contour_stack` is the shared type, and the tiler converts at its edge |
| Static counters for debug file names, and `_otv.g` / `_verts.g` debug outputs | Dropped, or a `debug_dir` option |

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

**API sketch**

```cpp
namespace cvc {
struct tile_params {
  double z_scale = 1, collinear_epsilon = 0, curation_delta = 1e-8;
  bool remove_intersections = false, interp_untiled_regions = true;
  double correspondence_overlap = -1;
  std::vector<std::string> include, exclude;      // component name regexes (-c / -x)
  int z_first = -1, z_last = -1;                  // -s
};
struct tile_result { std::map<std::string, geometry> components; std::vector<std::string> warnings; };
tile_result tile_contours(const contour_stack &, const tile_params & = {});
std::unique_ptr<sliced_job> tile_contours_job(contour_stack in, tile_params p, tile_result &out);
}
```

One step of the job is one slice pair of one component, so progress is real.

**Surfaces**

- **CLI:** `cvc tile in.ser -o outdir`, keeping `cl_parse`'s flags. The VolumeRover2 dialog passes
  them as free text, default `-C 0.01 -e 1e-15 -o raw`.
- **pycvc:** `cvc.tile_contours(...)`.
- **Ariadne:** a `contour_lab` node type. It draws a stack's contours as polylines in their
  component colours (tag `contours`) and, on a `tile` request, the surfaces as children (tag
  `tiles`). Its keys live under `<node>.tiler.*`. The `contour_tiler.ari` component has a
  component picker, the slice range, the parameters, Tile and Stop.

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
- **Segmentation:** the point classes are seeds for GenSeg, run locally or over XML-RPC. It
  leaves `SegTmp.rawv` in the current directory, and nothing in VolumeRover2 loads it.
- **Contours:** per-object contours on slices, with interpolation, export (`.pcd` and
  `.contour` work; `.cnt`, `.bspline` and `.matc` were never ported to Qt6) and tiling into
  surfaces.

In the Qt6 code, though, most of the contour side is dead:

- you cannot create a contour (`addContourSlot` returns early) or enter contour mode (the tab
  signal is never connected);
- the Load button is disconnected;
- tiling, SDF2D, the medial axis, curation and EM are commented out or compiled out.

VolumeGridRover never calls either tiler.

Rendering: a `GL_R8` slice texture with a 256-entry transfer-function texture through an
ARBfp1.0 program (paletted and SGI colour-table fallbacks), immediate-mode quads, `gluDisk` glyphs
and `GL_SELECT` picking. Each depth change re-reads one slice through a VolMagick subvolume read,
which is the out-of-core path. The memory-mapped readers are not even built.

**So this port is a rebuild of what VolumeGridRover was for, not a translation of the widget.**
None of its Qt or GL code carries over. Its data model, file formats, interaction design and a
few algorithms do. volrover's history from before the Qt6 migration may hold the last working
contour and tiling code. Read it before phase 7; the shallow clone used for this sketch has none
of it.

**The node: `type: volume_grid`** (`inc/cvc/gl/ariadne/volume_grid_nodes.h`)

- It reads `source: { file, var, time }` as a volume node does.
- **Children:**
  - three `volume_slice` nodes (§5 item 1), tagged `slice_xy`, `slice_xz` and `slice_zy`;
  - per slice, overlays carrying the same tag: point-class glyphs, contour polylines, and
    crosshairs where the other two planes cross;
  - a `<id>_surfaces` group for tiled surfaces (tag `surfaces`).
- One scene, tagged groups: the XY view shows `slice_xy`, and the 3D view shows all three planes
  with the surfaces, the classic orthoslicer.
- Edits run in C++ in the node, where they are fast and testable. The DSL only wires widgets to
  keys.

Keys under `<node>.grid.*`:

| Key | Meaning |
|---|---|
| `source.file`, `source.var`, `source.time` | what `load` reads; `info.dims`, `info.min`, `info.max`, `info.span` published |
| `slice.x`, `slice.y`, `slice.z` | voxel indices; written by the sliders, the keys and clicks |
| `cursor.{i,j,k,x,y,z,value}` | published on hover: the probe |
| `tool` | `navigate`, `points`, `contours`, `probe` |
| `pick.*` | written by the views (`pick:` on the viewport, §5 item 3); the tick consumes each new pick according to `tool`: `points` toggles a point (adds one, or removes the nearest within `display.point_size` on this slice), `contours` appends to the current object's contour on this slice at its nearer end, and a Shift-drag selects with a rubber band |
| `classes`, `classes.current` | point classes: name, colour, radius |
| `objects`, `objects.current` | contour objects: name, colour, interpolation (`linear` or `spline`), samples |
| `display.point_size`, `display.show_points`, `display.grey`, `display.background` | display |
| `request` | `load`, `save_classes <f>`, `load_classes <f>`, `load_points <f>`, `save_contours <f>`, `load_contours <f>`, `delete_selected`, `tile`, `undo`, `stop` |
| `busy`, `status`, `progress`, `phase` | published |
| `<node>.transfer_function` (+ `.histogram` …) | as on every volume node, so the `tf_editor` colours the slices |

**Data and algorithms**

- **Point classes:** a small `cvc::point_classes` type (classes × voxel points, per variable and
  timestep) with `.vgr` read and write and a `.pts` reader. The legacy `.vgr` form is
  `<pointclassdoc><pointclass name color variable timestep><point>"x y z"</point>`.
- **Contours:** the §3 `contour_stack`, one component per object.
- **Splines:** the legacy resampled with GSL splines every frame. libcvc does not link GSL on
  wasm, so use a small natural-cubic or Catmull-Rom spline in libcvc, computed when a contour
  changes.
- **Tile:** the §3 tiler on the drawn objects. The surfaces appear in the 3D view in the objects'
  colours. This finally wires up the `tilingComplete` the legacy declared and never emitted.
- **Ported later, on demand:**
  - SDF2D, the 2D signed distance of contour polygons (CGAL k-nearest-neighbour search), for a
    "show SDF" overlay;
  - `medax.h`, a 2D medial axis through CGAL's segment Delaunay graph;
  - Kinney's b-spline fit (`bspline_fit.cpp`, not compiled today);
  - slice isolines through libcontour's 2D path, already in-tree (`cvc-mesher/contour/conplot2d`).
- **Not in scope: segmentation.** GenSeg is 35.6k lines in `src/Segmentation`. Until it is
  ported, VolumeGridRover saves point classes as seeds, and a later Segmentation port adds a
  `segment` request. #123's seed generators (`criticalPntsToSeed`, `SeedProduce*`) belong with
  it.

**The component: `volume_grid.ari`**

- **`volume_grid`:** one window holding:
  - the tool as radio buttons;
  - a 2×2 grid of viewports: XY, XZ and ZY in `mode: slice` with `input: pick`, and a 3D orbit
    view;
  - under each slice view, a slider bound to `grid.slice.{z,y,x}`, with the index in the
    view's caption;
  - a side panel with tabs: Probe, Point classes (list, colour, radius, add, delete, save,
    load), Contours (objects, colour, interpolation, samples, save, load, Tile) and Transfer
    function (the `transfer_function_editor` unit).
- **`volume_grid_views`** and **`volume_grid_tools`:** the views alone and the panel alone, for
  a document that lays them out itself.
- **Args:** `node`, `title`, `id`, `view_size`, `layout` (`quad`, or `tabs` like the legacy).
- **Demo:** `examples/ariadne/documents/volume_grid.ari` on a shipped volume, with a golden image
  and an interaction test that scripts picks through state writes.

---

## 5. Ariadne pieces VolumeGridRover needs

Most of the GL-viewer work is already on master. What exists, and what is new:

**Already there**

- **The `viewport` widget** (ARIADNE.md §5.18, `src/cvcGL/ariadne/scene_views.cpp`): a view of
  the scene inside a window, with its own camera at `<prefix>.viewers.<view>.camera`, nodes
  picked by `tags:` (`show:` / `hide:` / a `bind:` key), per-view `override:` restyling, named
  cameras, `scenes:` (separate SceneGraphs shown by the views that name them) and a `caption`.
  It is a mirror viewport of the main one, drawn in a rect cut out of the window's background,
  so it scrolls and clips with its window. `scene_view.ari` is it in a window of its own.
- **Custom node types** with request keys and sliced jobs (`mesh_nodes.cpp` is the template).
- **The transfer function editor** (`tf_editor`, `transfer_function_editor` unit). Every volume
  node already publishes its model, histogram, contour spectrum and contour tree, so a volume
  grid node gets the colour map editor for free.
- **Subvolume reads:** `volume_file_io::readVolumeFile(ctx, vol, file, var, time, subvolbox)`, the
  VolMagick heritage, so a slice of a large file can be read without reading the volume. This
  replaces VolumeGridRover's memory-mapped `MappedRawIVFile` / `MappedRawVFile` for display.

**New, and reusable outside VolumeGridRover**

1. **An axis-aligned slice node** (`type: volume_slice`): one plane of a volume, as a textured
   quad coloured through the node's transfer function, at `<node>.slice.{axis,index}`. It
   uploads one 2D texture per slice change (read by region for big files), so stepping through a
   2k³ volume never needs the volume on the GPU. Today the only axis-aligned cut is fe_lab's
   `<id>_slice` (an FE field on tetrahedra); `volslice` slices face the camera, which is a
   different thing. Useful on its own: the classic three-plane orthoslicer next to a `volren`.
2. **A slice camera mode for the viewport** (`mode: slice`, `axis: x | y | z`): parallel
   projection looking down the axis, no rotation, drag pans, the wheel zooms about the pointer,
   double-click refits the slice. `mode: map` is a top-down view along z only and orbits, so it
   is not enough. The camera controller already has the parallel-projection plumbing for `map`.
3. **Pointer events from a viewport to state** (`input: pick`, `pick: <key>`, `on_pick:`): a click
   or drag in the view writes where it landed, and the left button no longer turns the camera
   (middle/right still pan, the wheel still zooms). In slice mode the hit is the slice plane,
   exact and cheap: `<key>.{x,y,z}` in world units, `<key>.{i,j,k}` in voxels, `<key>.value`,
   `<key>.button`, `<key>.mods`, `<key>.phase` (`down`, `drag`, `up`). In an orbit view it is a VTK
   prop pick (`<key>.node`, the node id hit). `on_pick:` runs an action program with those keys
   set, so tools can be written in the DSL. This is the "click-to-world picking in a viewport"
   item already on the libcvc list (L0d); build it once, here.
4. **Overlays that stay on top of a slice:** point glyphs, polylines and crosshairs drawn as child
   geometry nodes of the slice, with a coincident-topology offset (or a later render layer) so they
   never z-fight the textured quad. `point_size` / `line_width` already exist as override fields.
5. **Widgets the side panel needs** (check each against the backend before planning around it):
   a list bound to a state array with add/remove (the `repeat:` the card component uses), a
   colour picker for a class colour, and radio buttons for the tool. Anything missing is a small
   ImGui widget in `ImGuiBackend.cpp` with an FTXUI placeholder.

Items 2 and 3 are the critical path for VolumeGridRover; item 1 is the bulk of the rendering
work; items 4 and 5 are small.

---

## 6. Phases

Sizes are relative: S is a PR or two, M a few, L a short series.

| Phase | Size | What lands | Needs |
|---|---|---|---|
| P0 Groundwork | S | `cvc::contour_stack` with its readers and writers; `cvc::point_classes` with `.vgr` / `.pts`; the §8 answers; a spike building CGAL's `Delaunay_triangulation_3` for wasm-mt with `CGAL_DISABLE_GMP` and Boost.Multiprecision as the exact fallback | — |
| P1 Skeleton core | M | Vendor into `src/cvc/geometry/skeleton/` with the shared cocone core. Add the context struct, the §2 fixes, `cvc::skeletonize` and its sliced job. gtests: a torus gives one closed strand; a thin slab one sheet; a capsule one segment; two calls in one process agree (the global-box bug); output is deterministic. Also `cvc skeleton` and pycvc | P0 decision 1 |
| P2 Skeleton in Ariadne | S | `mesh_lab` `skeleton` request, children, mesh_tools section; a demo document with a golden image | P1 |
| P3 Tiler core | L | Vendor into `src/cvc/geometry/tiler/` with the log4cplus shim; in-memory output, const input; its 12 tests as gtest; `cvc tile`; pycvc | P0 |
| P4 Tiler in Ariadne | S | `contour_lab` and `contour_tiler.ari`, with a demo on a small redistributable Reconstruct series (find one) | P3 |
| P5 Viewport pieces | M | §5: the `volume_slice` node, `mode: slice`, `input: pick` / `pick:` / `on_pick:`, overlay offset, missing widgets. Each with tests, plus an orthoslicer demo next to `volren_bunny` | — |
| P6 volume_grid v1 | M | Navigation, probe, point classes (mark, select, move, delete, save, load), the transfer function, quad or tab layout; golden image and scripted-pick tests | P5, P0 |
| P7 volume_grid v2 | M | Contours: draw and edit per object, linear and spline, export; `tile` puts surfaces in the 3D view | P6, P3 |
| P8 Web and wrap-up | S–M | arigl-web for whatever is CGAL-free (volume_grid without Tile, and the rest if the P0 spike worked). Docs: ARIADNE.md §5.4 rows, INDEX.ari entries. volrover3 picks it all up through libcvc. Retire the legacy libraries from volrover `master` once parity holds, together with #123's VolUtils leftovers | P2, P4, P7 |

P1, P3 and P5 share no files and need nothing from each other, so they can run as parallel
sessions. Separate items afterwards:

- a Segmentation (GenSeg) port, which becomes `volume_grid`'s `segment` request;
- SDF2D, the 2D medial axis and contour curation;
- TightCocone and PocketTunnel on the shared cocone core.

---

## 7. What is deliberately left behind

- **The old `src/Tiling` code:** its licence, the globals, the `exit()`s and the temp files.
  Its contour format survives as a reader.
- **All of VolumeGridRover's Qt and GL:**
  - `SliceCanvas`, the three slice renderers (ARB fragment program, paletted texture, SGI
    colour table), immediate mode, `GL_SELECT` picking;
  - the `QThread` workers that read widgets off the GUI thread;
  - the `QSettings` cache directory and the temp files in the current directory.
- **The memory-mapped readers** (`MappedVolumeFile`, `lfmap.c`), which are not built today.
  Region reads of one slice cover the same need, and a small cache of recent slices covers
  scrubbing.
- **`PolyTess`:** unused, with a non-commercial licence.
- **ContourTiler's tools** (Sandbox, Decimate, ecs, smfconvert, ContourNear) and its debug output
  files.
- **ContourFilter's smoothing singleton**, unless a dataset needs it.
- **EM clustering**, compiled out today for "serious memory problems".
- **Skeletonization's helix-specific strand window and its `#if 0` helix block**, unless they are
  wanted as an option; they fit better with a SecondaryStructures port.

---

## 8. Decisions for Joe

1. **CGAL's GPL packages.** Both CGAL ports use GPL-3+ packages: Skeletonization uses 3D
   Triangulations, and ContourTiler uses 2D Boolean operations, Arrangements, constrained
   Delaunay and the straight skeleton. libcvc already compiles CGAL's Spatial Searching (also
   GPL) into `nav/spatial.cpp` and `project_verts.h` whenever CGAL is found, so the native
   bundle already carries GPL CGAL code. Dropping FFTW to make the bundle LGPL-2.1 shows the
   licence matters. The options:
   - (a) build the ports into libcvc when CGAL is found, as today;
   - (b) put them in a separate optional library (say `cvc_cgal`), so the core stays LGPL. The
     existing two uses could move there later.

   **Recommended: (b).**
2. **ContourTiler's licence.** Its sources carry no licence header. Skeletonization and
   VolumeGridRover carry UT LGPL-2.1 headers. Confirm ContourTiler is UT CVC code under the same
   terms before vendoring it.
3. **The web.** CGAL is off on wasm today (`DISABLE_CGAL=ON`: GMP and MPFR have no wasm-mt
   build). The options:
   - ship skeletons and tiling native-only;
   - try the P0 spike. Skeletonization needs only EPICK plus 3D Delaunay, which should build
     without GMP. ContourTiler's exact boolean ops are much harder.

   **Recommended:** native-only, and run the spike for Skeletonization alone.
4. **Where the skeleton lives.** A request on `mesh_lab` needs both libigl and CGAL in the build.
   A small node type of its own does not. **Recommended:** `mesh_lab`. libigl is header-only, ON by
   default (`CVC_ENABLE_LIBIGL`, which turns itself off only when the packages are missing) and in
   the libcvc recipe, so a CGAL build without it is unusual.
5. **The VolumeGridRover layout.** Four views at once (recommended), or the legacy tabs. The
   `layout` arg can offer both.
6. **Segmentation.** Does GenSeg join this effort, or stay a separate item? **Recommended:**
   separate. It is 35.6k lines, and the seeds VolumeGridRover saves are enough to start.
7. **Names.** `volume_grid`, `contour_lab`, `volume_slice`, `cvc skeleton`, `cvc tile`.
