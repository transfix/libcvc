# volrover-goldens: the legacy libraries' outputs, for the ports' parity tests

The VolumeRover library ports (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) each start by
matching the legacy library's output on committed inputs: row T1 of §1.7. This directory makes
those outputs reproducibly:

1. `make_goldens.py` fetches transfix/volrover at the commit in `volrover.sha`. It is a shallow,
   sparse checkout of just the directories the libraries need.
2. It builds the legacy libraries with this directory's `CMakeLists.txt`. That build is never
   part of libcvc's.
3. It runs them on the committed inputs (`inputs/<library>/`).
4. It writes the goldens to `src/cvc/tests/data/volrover-goldens/<library>/`, where the ports'
   tests read them.

GPL code may be built here, and only here. The libraries that need CGAL (Skeletonization,
ContourTiler) build against a CGAL where CMake finds it (cvcpkg's, in CI). Nothing built here is linked into
libcvc, cvcGL or pycvc, and nothing here is installed or packaged.

```bash
python3 tools/volrover-goldens/make_goldens.py            # write the goldens
python3 tools/volrover-goldens/make_goldens.py --check    # make them again and compare
python3 tools/volrover-goldens/make_inputs.py             # rewrite the inputs (then commit them)
```

You need git, cmake, a C++ compiler and Python 3.8 or later. Ninja is used when it is on PATH.
`--work DIR` says where the checkout and the build go (default: the system temp directory).
`--volrover DIR` uses an existing checkout instead of fetching one.

CI's `volrover goldens` job runs `--check` on Linux whenever this directory or the goldens change.
When the check fails, it uploads the goldens it made.

## GenSeg

GenSeg is `src/Segmentation/GenSeg`, the seeded segmentation of §5. It is the first library here
because it has no CGAL.

- **What is built:** the legacy `segment.cpp` (the fast marching) and `inout.cpp` (its rawiv
  reader and writer), as they are at the pinned commit, with their warnings off.
  `genseg_driver.cpp` stands in for `general_segmentation.cpp`, the XML-RPC entry. It builds the
  seed lists the same way and calls `read_data` and `Segment` as that entry did.
- **What is shimmed:** `shim/` replaces three things the legacy files include:
  - VolMagick's `VolumeFileInfo` (the rawiv header, used only for the world coordinates in the
    border point cloud);
  - VolMagick's `big_endian()`;
  - the `XmlRpcValue` declaration in `genseg.h`.

  The real VolMagick needs volrover's whole core library to build, and GenSeg reads every voxel
  by hand anyway.
- **Inputs:** `inputs/genseg/<case>.raw` (float32, little-endian, x fastest) and `<case>.json`
  (dimensions, `tlow`, `thigh`, and seeds as `[class, x, y, z]`). `make_inputs.py` wrote them, and
  they are committed, so the goldens never depend on a machine's `exp()`. The script writes each
  input as the big-endian float rawiv file the legacy reader expects, on the unit grid. The legacy
  reader then normalizes the data to 0..255, so `tlow` and `thigh` are on that scale.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/genseg/`:
  - `<case>.labels.u8` holds the class of every voxel (uint8, x fastest; 255 means no class).
    It is read from the legacy `<prefix>_subunit_NN.rawiv` files: a voxel is in class NN where
    that file is nonzero. Every labelled voxel is at or above `tlow`, so it is above 0.
  - `<case>.indices.txt` is the legacy `<prefix>_indices.txt`: the voxels where two fronts met.
  - `<case>.json` records the case, the volrover commit, the platform it was made on, each
    class's voxel count, and the files' SHA-256.

| Case | Size | What it covers |
|---|---|---|
| `two_blobs` | 24³ | two touching blobs, one seed each |
| `three_classes_noisy` | 24³ | three classes on noisy data, so GenSeg's automatic seeds (its local maxima) join in |
| `two_seeds_one_class` | 20³ | a class with two seeds |

**Platforms:** the goldens were made on Windows with MSVC 2022. CI's check on ubuntu-24.04 with
GCC then made the same bytes for every case (2026-11-10, transfix/libcvc#1115).

The roadmap (§5) says the legacy GenSeg "doesn't work properly today". These goldens record what
it does, so the port can first match it voxel for voxel, before the port fixes it.

## Skeletonization

Skeletonization is `src/Skeletonization`, the skeleton of a point cloud (the roadmap's §2). It
needs CGAL, so this build needs CGAL, GMP, MPFR and Boost where CMake finds them: for example
cvcpkg's `cgal gmp mpfr boost` in a prefix on `CMAKE_PREFIX_PATH`, as CI's job installs them.
`--libs genseg` builds GenSeg alone.

- **What is built:** every legacy source but `PolyTess.cpp` (the ear-clipper nothing calls,
  whose notice forbids commercial use; `shim/Skeletonization/PolyTess.h` stands in for its header)
  and `main.cpp.unused`. They are compiled from copies in the build tree, patched at configure
  time. A patch that does not apply fails the configure. The patches:
  - **Five `CGAL_assertion()`s with side effects become plain calls:** `has_vertex()` in
    `medax.cpp` and `tcocone.cpp`, and `find_acceptor()` in `u1.cpp` and `u2.cpp`. These calls
    fill the indices the code then uses. A release build compiles the assertions out, which leaves
    those indices at -1 (or uninitialised), so the code reads and writes out of bounds. The port
    makes the same calls.
  - **`is_cospherical_pair()` returns false when one of the two cells is infinite,** as the port
    does. It used to test the infinite vertex's default-constructed (uninitialised) point.
  - **The helix strips `build_simple_skel()` returns get alpha 0.75 instead of 1,** so that
    `skeleton_driver.cpp` can tell them from the polyline segments, which share their colours.
- **What is shimmed:** `shim/cvcraw_geometry/Geometry.h`, the one piece of the legacy Geometry the
  library reads: its float vertex array.
- **Inputs:** `inputs/skeleton/<case>.pts` (a point a line, each coordinate the exact decimal of a
  float, so float and double parsers read the same value) and `<case>.json` (the parameters,
  legacy names, defaults when empty). `make_inputs.py --only skeleton` wrote them.
- **Goldens:** for each case, in `src/cvc/tests/data/volrover-goldens/skeleton/`:
  - `<case>.skel.txt`: what the legacy `skeletonize()` returned, a line per element: `L` polyline
    segments, `S` star segments, `H` helix strips and `P` sheet polygons
    (`skeleton_driver.cpp` gives the format);
  - `<case>.json`: the counts, the volrover commit, the platform and the SHA-256.

| Case | Points | What it covers |
|---|---|---|
| `torus` | 768 | a jittered torus: the legacy output is empty, because its core circle is a closed loop |
| `capsule` | 941 | a jittered capsule: one strand along the axis |
| `dumbbell` | 1,236 | two spheres joined by a rod: one strand through all three |
| `box_grid` | 154 | a box's surface on a lattice, exactly cospherical everywhere: strands, stars and two sheets |

`skeleton_test` (`MatchesTheLegacyLibrary`) compares the port with these goldens:
- **exactly (to 1e-9)** on the capsule, the torus and the box: every polyline and star segment,
  every sheet point and the sheets' area. The triangulation is CGAL's, cell for cell, even on the
  cospherical box. Closed loops are left out of the comparison, because the legacy code dropped
  them;
- **as a superset** on the dumbbell. Where U2's chains merge, which pieces are kept depends on the
  order the saddles are traced in. The legacy code used its cells' memory order (CGAL's allocation
  history); the port uses a geometric one. So the port has all 362 legacy segments, and 1 more.

**Platforms:** the skeleton goldens are the ones CI's check made on ubuntu-24.04 with GCC and
cvcpkg's CGAL (two runs made the same bytes), so `--check` passes there and only there. A Windows
build (MSVC 2022, CGAL 6.0.1) makes the same elements to 1e-9 on the capsule, the torus and the
box, but not the same bytes: the last digits differ, and so does the order of tied sheet polygons.
Its dumbbell differs more, because the cells' memory order is the allocator's: 361 segments, one
of them not in Linux's 362, and two of Linux's missing. The port contains both platforms'
dumbbell segments, and its test compares coordinates to 1e-9, so it passes against either set.
