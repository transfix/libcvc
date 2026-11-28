# texmol-goldens: TexMol's outputs, for cvc::mol's parity tests

Each TexMol library port is checked against its TexMol original
(docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §1 item 5), as the VolumeRover ports are against the
legacy VolumeRover (`tools/volrover-goldens`). This directory makes those outputs reproducibly:

1. `make_goldens.py` fetches transfix/TexMol at the commit in `texmol.sha`. It is a shallow, sparse
   checkout of just the directories the parser and GOABlur need.
2. It builds `blur_legacy` with this directory's `CMakeLists.txt`. That build is never part of
   libcvc's.
3. It runs it on PDB entry 1GRL (`examples/data/molecules/1grl.pdb`) for each case.
4. It writes the goldens to `src/cvc/tests/data/texmol-goldens/blur/`, where `blur_test` reads
   them.

TexMol's libraries are UT LGPL-2.1, as libcvc is. Nothing built here is linked into libcvc, cvcGL
or pycvc, and nothing here is installed or packaged.

```bash
python3 tools/texmol-goldens/make_goldens.py            # write the goldens
python3 tools/texmol-goldens/make_goldens.py --check    # make them again and compare
```

You need git, cmake, a C++ compiler and Python 3.8 or later. Ninja is used when it is on PATH.
`--work DIR` says where the checkout and the build go (default: the system temp directory).
`--texmol DIR` uses an existing checkout instead of fetching one.

CI's `texmol goldens` job runs `--check` on Linux whenever this directory, the goldens or 1GRL
change. When the check fails, it uploads the goldens it made.

## The parser and GOABlur

- **What is built:** TexMol's PDBParser (all of it), the pieces it includes (Utility, UsefulMath,
  ByteOrder, ColorManager, SimpleVolumeData) and Blurmaps' `GOABlur.cpp`, as they are at the
  pinned commit, with their warnings off and no floating-point contraction.
  `blur_driver.cpp` stands in for `BlurMapsDataManager::getBlurredVolume`, the resolution entry.
  It parses the file (`parserPDBtoGOA::parsePDBModels`), flattens it (`FlattenGOA`), sizes the grid
  as getBlurredVolume does (extent / resolution + 3 points), and calls
  `GOABlur::generateBlurmap` for the electron density with van der Waals radii and no colour.
- **What is shimmed:** `shim/` replaces two headers.
  - `Blurmaps/BlurMapsDataManager.h`: the real one pulls in most of TexMol, and GOABlur uses only
    its `getBoundingBox`. `CMakeLists.txt` copies that function's text, as it is, out of
    `BlurMapsDataManager.cpp` into the build tree.
  - `cvc/endians.h`, libcvc 3.x's, of which ByteOrder uses one function.
- **Cases:** `1grl_r1.5`, TexMol's default blobbiness (-2.3442) at 1.5 Å, and `1grl_coarse`, a
  low-resolution map (blobbiness -0.5 at 2 Å), the kind the skeleton demo uses.
- **Goldens:**
  - `1grl.atoms.txt`: the atoms TexMol's parser read, in its order (`x y z radius`, `%.9g`). It is
    the same for both cases.
  - `<case>.f32`: the density, float32 little-endian, x fastest.
  - `<case>.grid.txt`: the grid's dims, then its box's min and max.
  - `<case>.json`: the case, the TexMol commit, the platform and the files' SHA-256.
- **What `blur_test` checks:**
  - The atoms and radii are TexMol's, read with `unknown_radius` 1.0. TexMol gave an atom missing
    from its table its occupancy as its radius, and every 1GRL atom has occupancy 1.00.
  - The atoms come in a different order. TexMol's group tree keeps a sheet's strands together, so
    1GRL's residues 48-50 follow 38-40.
  - The grid and its box are TexMol's.
  - The densities match to 1e-5 relative. Most voxels match bit for bit; the rest are the same float
    sums taken in TexMol's order. `--check` uses the same tolerance, so another compiler's rounding
    passes.

## The parser's group tree and torsions

- **What is built:** `parser_legacy` (`parser_driver.cpp`), TexMol's PDBParser alone with the same
  supporting files, run on 1GRL. It prints the group tree `parsePDBModels` builds, depth first,
  one line a group (collection, model, chain, structure, residue), each residue with the phi and
  psi `finalizeParsing` stored, in radians.
- **Golden:** `src/cvc/tests/data/texmol-goldens/parser/1grl.tree.txt`, with `1grl.tree.json`
  (the TexMol commit, the platform and the files' SHA-256). `--check` compares every line exactly
  but the angles, which may differ by 1e-6.
- **What `molecule_groups_test` checks:**
  - `cvc::mol::build_group_tree` gives TexMol's tree line for line, and its walk TexMol's atom order
    (blur's `1grl.atoms.txt`);
  - `cvc::mol::torsions` gives TexMol's phi and psi to 2e-5 wherever TexMol's neighbours are the
    residue's real ones: 874 angles. TexMol took a residue's neighbours from its tree, which puts
    a sheet's strands together, so at 34 angles it measured to another strand; the test checks
    those are the angles to that neighbour. Where an atom is missing (1GRL's C-alpha-only
    stretch), TexMol wrote 0 and `torsions` gives NaN (126 angles).
