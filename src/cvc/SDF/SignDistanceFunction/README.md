# Signed Distance Function (SDF) Library

This directory hosts libcvc's SDF backend: it converts a triangulated
surface mesh into a regular grid of signed distances, where each grid
sample is the signed distance to the nearest point on the input
surface (negative inside, positive outside).

> The full library reference — algorithm details, performance
> numbers, and isosurface-roundtrip examples — lives in
> [docs/SDF_LIBRARY.md](../../../../docs/SDF_LIBRARY.md). This
> document covers the API exposed by the headers in this directory.

## What the library produces

Given:

- a triangle mesh as parallel vertex / triangle index arrays,
- a target grid resolution (a power of two between 16 and 1024),
- an axis-aligned bounding box for the grid in world coordinates,

the library emits an `N × N × N` array of `float` distances laid out
row-major, indexed as

```
sdf[k * N * N + j * N + i]
```

with world position
`(minx + i*span, miny + j*span, minz + k*span)`,
`span[a] = (max[a] - min[a]) / N`.

Sign convention:

- `> 0` — sample is outside the surface, value is the distance to the
  nearest point on the mesh.
- `= 0` — sample lies on the surface (within tolerance).
- `< 0` — sample is inside the surface; its absolute value is the
  distance to the nearest point on the mesh.

## How it works

1. **Bounding box / grid setup.** The user supplies world-space
   `mins` and `maxs` and the grid size; the library derives per-axis
   spans and integer cell coordinates.
2. **Octree of the mesh.** Each input triangle is inserted into an
   octree built over the grid. The octree lets distance queries from
   a grid sample skip empty regions of space.
3. **Boundary marking.** Cells that the surface intersects are
   flagged. The exact closest-point distance is computed for the
   eight corner samples of every boundary cell.
4. **Distance propagation.** A fast-marching-style sweep propagates
   distances outward from the boundary cells through the rest of
   the grid, taking the smaller of "current best" vs. "neighbor's
   best plus one cell" at each step until the field stabilizes.
5. **Sign assignment.** A flood pass uses the surface orientation
   (or its inverse, when `isNormalFlip` is set) to mark the inside
   as negative and the outside as positive.

The complexity is roughly `O(N³ + T log T)` where `T` is the
triangle count; the octree avoids the naive `O(N³ · T)` lookup.

## Public API

There are three entry points, all in `namespace SDFLibrary`. New
code should prefer the first; the others exist for fine-grained
control or for callers integrating against the legacy global API.

### 1. One-shot, thread-safe (recommended)

```cpp
#include "sdfLib.h"

std::unique_ptr<float[]> grid = SDFLibrary::computeSDF_MT(
    nverts, verts,            // float[3 * nverts], xyz interleaved
    ntris,  tris,             // int[3 * ntris], vertex indices
    size,                     // grid resolution N (power of two)
    /*isNormalFlip=*/0,
    mins, maxs);              // float[3] each
```

Returns an owning `unique_ptr` to `size³` distances, or `nullptr` on
failure. Each call uses a private `SDFContext` internally, so calls
from different threads do not contend.

### 2. Manual context (advanced)

When you want to reuse the octree across multiple grid evaluations,
inspect intermediate state, or integrate with a custom job system:

```cpp
#include "SDFContext.h"

auto ctx = SDFLibrary::createContext();
ctx->setParameters(size, /*isNormalFlip=*/0, mins, maxs);
if (!ctx->initSDF()) return /* error */;
ctx->readGeom(nverts, verts, ntris, tris);
ctx->adjustData();   // tighten bbox, build octree
ctx->compute();      // boundary + propagation + signs

float* values = ctx->getValues();         // borrowed pointer
auto   owned  = ctx->releaseValues();     // or take ownership
```

`SDFContext` is move-only and self-cleans on destruction. One
context per concurrent computation.

### 3. libcvc high-level API

If you already use libcvc's `cvc::geometry` and `cvc::volume` types,
prefer `cvc::sdf` from `<cvc/algorithm.h>`; it wraps the routines
above and returns a fully populated `cvc::volume`:

```cpp
#include <cvc/algorithm.h>

cvc::volume sdf_vol = cvc::sdf(ctx, mesh, dim, bbox);
```

See the libcvc geometry API documentation for the rest of the
options (algorithm selection, normal flipping).

## Threading

- `computeSDF_MT` and `SDFContext` are reentrant per instance.
  Spawn one context (or one `computeSDF_MT` call) per thread and
  the library will not share global state across them.
- A single `SDFContext` is *not* safe to use from multiple threads
  simultaneously; treat it like a `unique_ptr`.
- Memory ownership is expressed through `std::unique_ptr<float[]>`;
  no explicit `delete[]` is required when using the recommended
  API.

## Choosing a grid size

| Triangle count    | Suggested grid | Memory (`float`) |
|-------------------|----------------|------------------|
| ≤ 1 000           | 64³            | ~1 MiB           |
| 1 000 – 10 000    | 128³           | ~8 MiB           |
| 10 000 – 100 000  | 256³           | ~64 MiB          |
| > 100 000         | 512³           | ~512 MiB         |

The grid size must be a power of two (`16, 32, ..., 1024`); other
values fail at `setParameters`. Tighter bounding boxes give better
effective resolution, so it pays to compute the mesh AABB and add
only a small padding (5–10 %) before passing `mins`/`maxs`.

## Building and testing

The SDF backend is built whenever libcvc is configured with
`-DCVC_ENABLE_SDF=ON` (the default). The library's tests live with
the rest of the libcvc test suite:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build -R AlgorithmTest.SDF
```

## Citation

If the library is useful in published work, please cite:

> C. Bajaj, P. Djeu, V. Siddavanahalli, A. Thane,
> *Interactive Visual Exploration of Large Flexible Multi-component
> Molecular Complexes*, Proc. IEEE Visualization 2004,
> pp. 243–250.

## License

See the `LICENSE` file at the repository root.
