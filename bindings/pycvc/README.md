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
