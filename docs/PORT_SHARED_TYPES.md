# The ports' shared types (P0d)

The data types the VolumeRover library ports share, built in phase P0d of
[VOLROVER-LIBRARY-PORTS-ROADMAP.md](roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) (§3, §4, §7):
`cvc::contour_stack`, the contours the tiler (P3b) and `volume_grid` (P10) share, and
`cvc::point_classes`, the seeds GenSeg (P5) grows from and the points `volume_grid` (P8) marks.
Each type has readers and writers registered on the app, parameter tables, DSL intrinsics and
pycvc bindings, on the pieces of [PORT_SHARED_PIECES.md](PORT_SHARED_PIECES.md).

## Contours (`cvc/geometry/contours.h`)

```cpp
cvc::contour_stack s;
s.z_spacing = 0.05;
s.add(3, "axon", {0, 0, 1, 0, 1, 1, 0, 1});      // slice 3, component "axon": x0 y0 x1 y1 ...
cvc::write_contours(app, s, "series.ser");       // the format by its extension
cvc::contour_read_params p;
p.include = {"ax.*"};
cvc::contour_stack t = cvc::read_contours(app, "series.ser", p);
```

- **Slices** are keyed by their z index. Slice k's plane is at `z_origin + k * z_spacing` in world
  units, unless the slice gives its own `z` (a Tiling series spaced unevenly).
- **Each slice** maps a component name to closed planar polygons: `x0 y0 x1 y1 ...` in world
  units, with the closing edge implied (the first vertex is not repeated).
- **Each component** (`contour_component`) has a name, a colour (r, g, b in [0, 1]) and a
  visibility, in the stack's order. `add()` and the readers keep an entry for every name a slice
  uses.
- `contour_problems(s)` lists what is wrong with a stack (too few vertices, a coordinate that
  isn't finite, a component the list lacks, a bad colour or spacing); `write_contours` refuses a
  stack with any.

### Formats

The readers and writers are each app's own: `contour_handlers(app)` is a
`cvc::file_handler_registry` that the app makes, with the built-in formats, the first time it is
asked (`app.slot<T>()`). A host adds a format to its app with `contour_handlers(app).insert(...)`,
and no other app sees it.

| Ext | Format | Read | Write | What the format keeps |
|---|---|---|---|---|
| `.ser` | Reconstruct: the series file and its section files beside it (`name.1`, `name.2`, ...) | yes | yes | names, colours (`border`), visibility (`hidden`), the slice spacing (`thickness`) |
| `.config` | The 1996 Tiling program: `PREFIX` / `SUFFIX` / `SLICE_RANGE` / `DETZ` and the numbered `.pts` files | yes | yes | one component, named after `PREFIX`; uneven slice planes |
| `.contour` | Tao Ju's ctr2suf: planes, vertices and edges with a material on each side | yes | yes | components as materials (`material_<m>`), orientation; no names or colours |
| `.pcd` | VolumeGridRover's point cloud: the vertex count, then `x y z` per vertex | no | yes | the vertices, no connectivity |
| `.cvcc` | libcvc's own, JSON | yes | yes | everything, bit for bit |

Where the port differs from the legacy readers and writers:

- **Reconstruct transforms.** Reconstruct stores the polynomial that takes series coordinates to
  a section's own, and a trace in the section's own, so a trace's world coordinates are that
  polynomial's inverse (as PyReconstruct reads them). The reader inverts it exactly for an affine
  transform and by Newton's method otherwise. ContourTiler's reader subtracted the constant term
  and applied the rest forwards, which is right only for a translation, and its `y` used the
  already transformed `x`. `apply_transforms = false` reads the coordinates as stored.
- **What Reconstruct traces are skipped.** A transform holding an `<Image>` holds that image's
  outline (`domain1`), not an object, and open traces (`closed="false"`) aren't polygons;
  `keep_open = true` keeps them. A repeated closing point is dropped. ContourFilter's smoothing
  and its hard-coded ignored names stay behind (§3).
- **Writing a Reconstruct series replaces it:** that series' section files which the stack lacks
  are removed. The series file written is a minimal `<Series>` element.
- **Tiling paths.** `PREFIX` is relative to the configuration's directory, not the working
  directory. The writer writes a file for every slice in the range, an empty one as the single
  point giving its plane, as the program needs.
- **ctr2suf and `.pcd`** were written one object a file by VolumeGridRover (with a `%1f` naming
  bug). Here one file holds the whole stack: in `.contour`, component k is material k + 1 on the
  side its polygon encloses and 0 on the other. A legacy file's `1 2` edges read as material 1.

### Reading parameters (`contour_read_params`)

Every reader applies the slice range, the component filters, `min_points` and `z_spacing`; the
last two are Reconstruct's.

| Key | Type | Default | Range | Description |
|---|---|---|---|---|
| `first` | int | `-1` | -1 .. | the first slice index read (-1: the first) |
| `last` | int | `-1` | -1 .. | the last slice index read (-1: the last) |
| `include` | list of string | `[]` | | component names to keep, as regular expressions (none: every one) |
| `exclude` | list of string | `[]` | | component names to drop, as regular expressions |
| `min_points` | int | `3` | 0 .. | polygons with fewer vertices are dropped (their slice is kept) |
| `z_spacing` | number | `0` | 0 .. | the slice spacing in world units, in place of the file's (0: the file's) |
| `apply_transforms` | bool | `1` | | Reconstruct: map traces through their section transforms |
| `keep_open` | bool | `0` | | Reconstruct: keep open traces as polygons |

The patterns are Perl-syntax regular expressions matched against the whole name, as the tiler's
`-c` and `-x` are. `contour_component` has a table too (`name`, `color`, `visible`), so a node
keeps each component's metadata as keys on the state tree (§1.5).

### DSL (`cvc/geometry/contour_intrinsics.h`)

| Intrinsic | Does |
|---|---|
| `(contours.read path [params])` | The stack in the file, as a data object (`cvc::contour_stack`); `params` is a dict read through `contour_read_params`' table |
| `(contours.info stack)` | A dict: `first`, `last`, `slices`, `polygons`, `vertices`, `z_spacing`, `z_origin`, and `components` (each `name`, `color`, `visible`, `polygons`) |
| `(contours.formats)` | The extensions `contours.read` reads, a list |
| `(params.defaults name)`, `(params.check name dict)` | With the tables `contours.read` and `contours.component` |

There is no `contours.write`: the DSL deals with stacks on the state tree, and writing a stack from
the tree to a file is left to the rest of libcvc (`write_contours`, pycvc, a node's request, the
CLI). A stack argument is a data object or a state path naming a node whose data channel holds one.
A path is the caller's, as a data reader's is (ARIADNE.md §9.4). They are small and synchronous, so
plain natives, not jobs.

`register_contour_intrinsics(env, ictx, app)` binds them in one environment (`cvc exec`, a pycvc
`Exec`, a test). `install_contour_intrinsics(app)` adds them to the app's own action-intrinsic
providers (key `cvc.contours`), so every program lane of that app's Ariadne documents gets them and
no other app's do.

### pycvc (`bindings/pycvc/pycvc_contours.i`)

```python
s = pycvc.contour_stack()
s.add(3, "axon", [0, 0, 1, 0, 1, 1])
pycvc.write_contours(app, s, "series.ser")
t = pycvc.read_contours(app, "series.ser", pycvc.contour_read_params(include=["ax.*"]))
t.indices(), t.component_names(), t.polygons(3, "axon"), t.components[0].color
```

Python gets copies: `s.components` is a list of copies (set one with `s.set_component(c)`), and
`s.polygons(index, name)` a list of coordinate lists. `read_contours` and `write_contours` let
other Python threads run meanwhile. Both parameter classes take keywords and have `validate()`,
`to_state` / `from_state` and `to_dict` / `from_dict` (pycvc_params.i).

### Tests

- `src/cvc/tests/contour_stack_test.cpp` (in `CVC_PORT_TESTS`, so the ports lane runs it under
  node, and in the sanitizer lanes): the model; T8 (both tables to the tree and back, defaults
  seeded, properties, bad values reported); T9 (every format round-trips, the legacy samples in
  `src/cvc/tests/data/contours` read, a corpus of truncated and garbage files raises naming the
  file); T6 (two apps keep their own handlers, two threads with different parameters read what
  each reads alone, A then B equals B then A); the intrinsics in an environment and from an
  Ariadne document through the app's own provider.
- `bindings/pycvc/test_pycvc_contours.py` (T11): building a stack, every format, keywords and the
  state round trip, errors, and another Python thread running during a read.

The Reconstruct reader parses XML with `src/cvc/utility/xml_lite`, a small reader that keeps no
state: boost::property_tree's XML parser keeps function-local statics, which the writable-data
scan would flag (§1.3). The name filters use boost::regex for the same reason.

## Point classes (`cvc/volume/point_classes.h`)

```cpp
cvc::point_classes pc;
pc.add("membrane", 12, 40, 7);              // a voxel, by its i j k indices
pc.add("background", 0, 0, 0);
cvc::write_point_classes(app, pc, "seeds.vgr");
cvc::point_class_read_params grid;           // .pts holds object coordinates
grid.dims = {64, 64, 32};
grid.bbox = {0, 0, 0, 1, 1, 0.5};
cvc::point_classes seeds = cvc::read_point_classes(app, "points.pts", grid);
```

- **Each class** has a name, a colour, a display radius (voxels), the variable and timestep of the
  volume it marks, and its points as voxel indices `i0 j0 k0 i1 j1 k1 ...`.
- **A class is known by its name, variable and timestep together,** as VolumeGridRover kept one
  list of classes per variable and timestep.
- `add()` creates a class with the next colour of a fixed palette (ColorBrewer's Set1, whole
  255ths, so `.vgr` keeps it exactly); the legacy drew random colours from `srand(time(NULL))`.
- `point_class_problems(pc, dims)` lists what is wrong (no name, a repeated class or point, a bad
  colour or radius, a negative index, and, given the volume's dims, a point outside it);
  `write_point_classes` refuses classes with any.

### Formats

`point_class_handlers(app)` is the app's own `file_handler_registry`, made with the built-in
formats the first time the app asks.

| Ext | Format | Read | Write | Notes |
|---|---|---|---|---|
| `.vgr` | VolumeGridRover's point classes (XML: `<pointclassdoc><pointclass name color variable timestep><point>i j k</point>`) | yes | yes | colours as `#rrggbb`; a `radius` attribute the legacy file didn't have (1 when absent) |
| `.pts` | VolumeGridRover's point list: per class, a count and that many `x y z` object coordinates | yes | no | classes `Class 0`, `Class 1`, ...; with the volume's dims and box, `int((x - xmin) / (xmax - xmin) * (xdim - 1))` clamped, as the legacy computed it; without, the nearest index |

The `.vgr` reader is new: Qt6 stubbed out VolumeGridRover's (its SAX content handler). A point or
class listed twice reads once.

### Parameters

`point_class` has a table for its metadata (`name`, `color`, `radius` in voxels, `variable`,
`timestep`; not its points, which go on the data channel), so `volume_grid` keeps
`<node>.grid.classes` as keys (§4). `point_class_read_params`:

| Key | Type | Default | Range | Description |
|---|---|---|---|---|
| `dims` | list of int | `[]` | 1 .. | the volume's dims, x y z (with bbox: `.pts` coordinates become voxels) |
| `bbox` | list of number | `[]` | | the volume's bounding box, xmin ymin zmin xmax ymax zmax |
| `variable` | int | `0` | 0 .. | the variable `.pts` classes mark |
| `timestep` | int | `0` | 0 .. | the timestep `.pts` classes mark |

`dims` and `bbox` go together. Given them, any reader's point outside the volume is an error.

### DSL (`cvc/volume/point_class_intrinsics.h`)

| Intrinsic | Does |
|---|---|
| `(point-classes.read path [params])` | The classes in the file, as a data object (`cvc::point_classes`) |
| `(point-classes.add pc class i j k [variable timestep])` | A new data object with the voxel added; `pc` is never changed, and nil starts from none |
| `(point-classes.remove pc class i j k [variable timestep])` | A new data object with the voxel removed |
| `(point-classes.points pc class [variable timestep])` | The class's points, a list of `(i j k)` lists |
| `(point-classes.info pc)` | A dict: `points` (the total) and `classes` (each `name`, `color`, `radius`, `variable`, `timestep`, `points`) |
| `(point-classes.formats)` | The extensions `point-classes.read` reads, a list |
| `(params.defaults name)`, `(params.check name dict)` | With the tables `point-classes.read` and `point-classes.class` |

An edit stores its result back: `(state-data-set "g.points" (point-classes.add "g.points" "seeds"
3 4 5))`. `register_point_class_intrinsics(env, ictx, app)` binds them in one environment;
`install_point_class_intrinsics(app)` adds them as the app's own keyed provider
(`cvc.point-classes`). As with contours there is no write: the DSL deals with point classes on the
state tree, and `write_point_classes`, pycvc or a node's request writes them to files.

### pycvc (`bindings/pycvc/pycvc_point_classes.i`)

```python
pc = pycvc.point_classes()
pc.add("membrane", 12, 40, 7)
pycvc.write_point_classes(app, pc, "seeds.vgr")
grid = pycvc.point_class_read_params(dims=[64, 64, 32], bbox=[0, 0, 0, 1, 1, 0.5])
pts = pycvc.read_point_classes(app, "points.pts", grid)
pc.points("membrane"), pc.points_array("membrane")   # lists, and a (n, 3) int32 numpy copy
```

`pc.classes` is a list of copies (set one with `pc.set_class(c)`). Reading and writing release
the GIL.

### Tests

- `src/cvc/tests/point_classes_test.cpp` (in `CVC_PORT_TESTS` and the sanitizer lanes): the model;
  T8 (both tables); T9 (`.vgr` round trip, the legacy samples in
  `src/cvc/tests/data/point_classes`, a bad-file corpus); T6 (two apps, two threads, order); the
  intrinsics, an edit never changing its input, and an Ariadne document marking a point through
  the app's own provider.
- `bindings/pycvc/test_pycvc_point_classes.py` (T11): marking, numpy points, files, keywords and
  the state round trip, errors, and another Python thread running during a read.
