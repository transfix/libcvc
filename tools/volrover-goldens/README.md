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
ContourTiler) will build it from the legacy tree's own copy. Nothing built here is linked into
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

The roadmap (§5) says the legacy GenSeg "doesn't work properly today". These goldens record what
it does, so the port can first match it voxel for voxel, before the port fixes it.
