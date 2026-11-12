# The ports' shared types (P0d)

The data types the VolumeRover library ports share, built in phase P0d of
[VOLROVER-LIBRARY-PORTS-ROADMAP.md](roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) (§3, §4, §7):
`cvc::contour_stack`, the contours the tiler (P3b) and `volume_grid` (P10) share. Each type has
readers and writers registered on the app, parameter tables, DSL intrinsics and pycvc bindings,
on the pieces of [PORT_SHARED_PIECES.md](PORT_SHARED_PIECES.md).

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
| `(contours.write stack path)` | Writes the stack; returns the path |
| `(contours.info stack)` | A dict: `first`, `last`, `slices`, `polygons`, `vertices`, `z_spacing`, `z_origin`, and `components` (each `name`, `color`, `visible`, `polygons`) |
| `(contours.formats)` | A dict of the extensions the app reads and writes |
| `(params.defaults name)`, `(params.check name dict)` | With the tables `contours.read` and `contours.component` |

A stack argument is a data object or a state path naming a node whose data channel holds one. A
path is the caller's, as a data reader's is (ARIADNE.md §9.4). They are small and synchronous, so
plain natives, not jobs.

`register_contour_intrinsics(env, ictx, app)` binds them in one environment (`cvc exec`, a pycvc
`Exec`, a test). `install_contour_intrinsics(app)` adds them to the app's own action-intrinsic
providers (key `cvc.contours`), so every program lane of that app's Ariadne documents gets them and
no other app's do. Which hosts install them is the host's choice: a document that can call
`contours.write` can write any file the process can.

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
