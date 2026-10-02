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

Requires SWIG ≥ 4.0 and Python 3 development headers. Build with the cvcpkg
`swig` (`cvcpkg install swig --prefix <p>`), never a distro one, and name it
explicitly; its binary reports a stale `-swiglib`, so pass the library too:

```bash
export SWIG_LIB=<p>/share/swig/4.4.1
cmake ... -DSWIG_EXECUTABLE=<p>/bin/swig -DSWIG_DIR="$SWIG_LIB" \
          -DCVC_PYCVC_REQUIRE_SWIG_VERSION=4.4   # any other SWIG: configure error
```

Without `-DSWIG_EXECUTABLE`, a `swig` in a `CMAKE_PREFIX_PATH` prefix is taken
(and its `share/swig/<ver>` used as `SWIG_DIR`) before CMake's FindSWIG searches
the system, where its versioned names (`swig4.0`) would otherwise win.

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

## Level of detail (cvc::simplify + cvc::lod)

The whole LOD family is wrapped directly (`pycvc_lod.i`, `LodGraphicsNode` in
`pycvc_gl.i`): QEM `simplify`, per-asset pyramids, the selection math, tile
partitioning, pooled tiled builds and the `scene.cvch5` store.

```python
pool = pycvc.thread_pool(3)                      # a pool the caller owns
mesh, res = pycvc.simplify(g, params)            # out-params come back as values
pyr = pycvc.build_mesh_pyramid(g, pool=pool)     # .rungs / .world_error_m / .rung(k)
tiles = pycvc.partition_parts([(name, geom), ...], 100.0, group_key=["_walls", "_roof"])
pyrs = pycvc.build_tiled_pyramids(tiles, pool=pool, on_tile=lambda i, tile, pyr: q.put(i))

w = pycvc.scene_writer(app)                      # in-memory scene.cvch5
w.write_mesh_pyramid("tile0", pyrs[0], "%016x" % tiles[0].content_hash)
blob = w.to_blob()                               # bytes
r = pycvc.scene_reader.open_verified(app, blob, hashlib.sha256(blob).hexdigest())

node = sg.add_lod("tile0"); node.setPyramid(r.read_mesh_pyramid("tile0"))
stats = sg.select_lod_stats(view.make_view_params(pycvc.preset_view("balanced")))
```

The long calls release the GIL (other Python threads keep running); Python
callbacks (`on_tile`, `group_key`, `LodGraphicsNode.setRungStyle`) reacquire it,
and an exception they raise comes back out of the call that started them.
Failures map to `ValueError` / `IndexError` / `OSError` (HDF5) / `RuntimeError`
(SHA-256 mismatch). `examples/lod_city.py` runs the whole pipeline on a
synthetic city -- progressive attach from a loader thread, bake to bytes,
verified reload, a camera fly-out printing `lod_stats` per pose (`--render DIR`
draws each pose offscreen). Contract tests: `test_pycvc_lod.py`,
`test_pycvc_gl_lod.py`.

## HUDs & UIs (Dear ImGui, no raw ImGui needed)

`ImGuiOverlay(view)` is the per-viewer Dear ImGui integration. Python draws a HUD
by setting a draw callback and calling the **state-bound `ui_*` widgets/panels**
(`pycvc_imgui.i`) inside it — their bodies run within cvcGL against the overlay's
own ImGui context, so a HUD never touches raw `ImGui::` (which would crash: the
extension has its own `GImGui`):

```python
ov = pycvc_gl.ImGuiOverlay(view)
ov.attachCamera(cam)

def draw():
    pycvc_gl.ui_slider_double(app, "Speed", "scene.viewers.left.camera.settings.move_speed", 1, 200)
    pycvc_gl.ui_combo(app, "Belief", "demo.swarm.belief", ["shared", "grouped", "private"])
    pycvc_gl.ui_scene_panel(sg)          # ready-made shadow/chrome panel
    pycvc_gl.ui_camera_menu_items(cam)

ov.setDrawCallback(draw)                  # re-invoked each frame; keep it cheap, don't raise
```

Every widget reads/writes `cvc::state`, so the same value is drivable from a
script, a config file or a replicated peer. Raw custom-widget ImGui (free-form
windows) is a separate, deferred design (a curated subset compiled inside cvcGL).
Contract test: `test_pycvc_gl_imgui.py`.
