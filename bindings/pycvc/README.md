# pycvc — Python bindings for libcvc

SWIG bindings that expose libcvc to Python (Phase-13 modernization
deliverable). The goal is to **drive C++ from Python** — build `cvc`
geometry/volume data in Python and hand it to a C++ host (e.g. an embedded
interpreter inside VolumeRover3) — so numerical/research code never has to
be rewritten in C++.

`pycvc` is **general-purpose**: it knows nothing about any downstream
project. Domain code (e.g. a movement-bundle loader) lives in a separate
Python package that imports `pycvc`.

## Status

- **v0 (this): `Geometry`** — build meshes/polylines incrementally or in
  bulk, set per-vertex colors, compute normals, and read/write files
  (`.off`, CVC-raw, …) via libcvc's `geometry_file_io`.
- Planned: `Volume` (cvc::volume + SDF), `State` (cvc::state tree), and
  numpy fast-paths; then a `vr3` module (exposed by VolumeRover3's embedded
  interpreter) that turns a `pycvc.Geometry` into a live scene node.

## Design

`pycvc_geometry.h` is a SWIG-safe facade: it *forward-declares*
`cvc::geometry` behind a `shared_ptr`, so SWIG only ever parses
`std::string`/`std::vector`/primitive signatures — never libcvc's heavy
headers. Only the `.cpp` includes libcvc. C++ host apps reach the wrapped
`cvc::geometry` via `native()` (which is `%ignore`d on the Python side).

## Build

Consumes libcvc as an external SDK (`find_package(cvc CONFIG)`), exactly
like VolumeRover3 — so it builds against any installed or cvcpkg libcvc.

```bash
cmake -S bindings/pycvc -B build \
  -DCMAKE_PREFIX_PATH="<cvcpkg-deps-prefix>;<installed-libcvc-sdk>"
cmake --build build
ctest --test-dir build            # runs the smoke test
```

The importable module (`pycvc.py` + `_pycvc.so`) lands in `build/`. At
runtime, `libcvc.so` and its dependency prefix must be on the loader path
(`LD_LIBRARY_PATH`).

## Example

```python
import pycvc

g = pycvc.Geometry()
g.add_vertices([0,0,0, 1,0,0, 0,1,0])   # flat row-major xyz
g.add_triangle(0, 1, 2)
g.set_colors([1,0,0] * g.num_vertices())
g.compute_normals()
g.save("tri.off")
```

Requires SWIG ≥ 4.0 and Python 3 development headers.

## In-library data prep

The canonical training/twin prep ops are wrapped in-library, so a script never
round-trips through numpy for the whole grid just to reshape or normalize:

```python
v.normalize(0.0, 1.0)          # affine-remap the data range to [0,1] / [-1,1]
v.fill_value(0.0)              # clear before stamping obstacles
v.resample(64, 64, 64)        # resize in place to the network-input grid
patch = v.crop(x, y, z, 16, 16, 16)   # a NEW sub-grid; the source is untouched

n = g.get_normals()           # read compute_normals()'s result out (flat xyz)
g.set_normals([...])          # or stamp authored normals in
g.invert_normals(); g.reorient()
g.extents()                   # mesh AABB as (minx..maxz), like model.extents()
```

(Contract test: `test_pycvc_dataprep.py`.)

## World units & scene dimensions (digital twin)

Beyond geometry/volume/state, pycvc wraps the **world-unit base** so the
training/twin layer works in real metres, kilometres and miles — the Python
counterpart to the C++ `cvc::world_units` (full reference, including the Python
name-reshaping rules, in [`docs/WORLD_UNITS_API.md`](../../docs/WORLD_UNITS_API.md#python-pycvc)):

```python
import pycvc, pycvc_gl

app = pycvc.make_app()
app.world_units().set_regime_name("imperial")   # app-wide display regime
app.world_clock()                               # the app's simulation clock

wu = pycvc.world_units(1000.0)                   # 1 world unit == 1 km
wu.format_d(1000.0, "force")                     # -> value/unit measurement
wu.world_point_to_real(2000, 0, 0)              # -> (x, y, z, unit) coordinate

sg = pycvc_gl.SceneGraph(app)
node = sg.addGraphics("wing", geom)
node.local_to_world([0, 0, 0])                   # point through the transform chain
node.real_dimensions(app.world_units())          # "how big is this, really"
r.pick_world(x, y)                               # a clicked point in world space
```

The same wrapping covers the built-in reference nodes (`getGridNode()` /
`getAxisNode()`), lights (`addLight` → `LightNode`), the software raycaster
scene node (`VolRenNode`, `add_volren`; value types in `pycvc_volren.i`) and the
view-aligned slice renderer (`VolSliceNode`, `add_volslice`; value types in
`pycvc_volslice.i` — see [`docs/VOLSLICE_API.md`](../../docs/VOLSLICE_API.md#python-pycvc)).
The `cvc::state`-driven viewer/scene controllers are wrapped too, mirroring
`CameraController`: `StageLighting(sg)` (a cinematic key/fill/back/wash rig —
`apply_preset('three_point'|…)`, `setKey`/`setWash`, `apply()`, headless) and
`ScreenTextHud` (screen-space captions/status lines — `setText`, `setPosition`,
`get_color`; construct headless via `ScreenTextHud(app, path, None)`).
Contract tests: `test_pycvc_world_units.py`, `test_pycvc_gl_world.py`, and the
`extents_metres` case in `test_pycvc_model.py`.
