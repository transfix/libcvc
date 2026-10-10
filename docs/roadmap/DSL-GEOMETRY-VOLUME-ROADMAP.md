# DSL Geometry & Volume Processing Roadmap

**Status:** Active — planned, committed workstream (intended for implementation)  
**Date:** 2026-05-27 (committed to the repo 2026-10-10)

> **2026-10-10 — retained.** This roadmap predates the Ariadne backend pivot and had
> drifted to an untracked local file with no implementation; it is deliberately **kept**
> as an active workstream — the DSL geometry/volume *scripting* pipeline is wanted. Note
> it is distinct from the scene-side mesh→SDF *volume source* (`source: { sdf: … }`,
> PR #546), which is a UI/scene feature, not the `state_exec` scripting surface described
> here. The four phases below (dict/collection gaps → volume intrinsics → geometry
> intrinsics → pipeline/batch/distributed) remain the plan of record; they are queued
> after the current UI/scene/streaming work. Re-verify the "Current State" table against
> the code before starting — the DSL collections/dict surface has moved since 2026-05-27.

---

## Motivation

The state_exec DSL is designed to pipeline complex interactions
between parts of the codebase and make them reproducible.  A key
use case is geometry/volume processing pipelines — loading a volume,
extracting an isosurface, computing an SDF, tetrahedralizing, writing
results — all expressed as a DSL program that can be re-run, shared,
and distributed across a cluster.

Today the DSL has state-tree, messaging, scheduling, and a small
collections library, but **zero** geometry or volume intrinsics.
The `cvc` CLI already exposes 28+ subcommands for these operations.
The goal is to make the same surface available from the DSL so users
can script full pipelines.

---

## Current State

### What the DSL Has

| Area | Status |
|------|--------|
| Dicts (`dict`, `get-attr`, `set-attr`, `del-attr`) | 4 builtins |
| Dict stdlib (`dict-keys`, `dict-values`, `dict-merge`) | 3 functions |
| Lists (10 builtins, 9 stdlib) | Solid |
| Generators | 6 builtins |
| Strings (2 builtins, 12 stdlib) | Solid |
| Math (15 stdlib) | Solid |
| State tree (16 intrinsics) | Solid |
| State data objects (`state-data-get`/`state-data-set`) | Bridge exists |
| Geometry/volume intrinsics | **None** |
| Hash map type | **None** (dict is O(n) assoc list) |

### What the CVC CLI Exposes

| Category | CLI Commands |
|----------|-------------|
| File info | `info`, `stats` |
| Conversion | `copy`, `convert` |
| Geometry | `sdf`, `iso`, `tetrahedralize`, `hexahedralize`, `tetrahedralize2`, `layer-mesh` |
| Vol arithmetic | `add`, `subtract`, `scale`, `normalize`, `clip`, `negate`, `mask`, `downsample` |
| Vol transform | `rotate` |
| Analysis | `ssim` |
| Projection | `project`, `backproject` |
| Image I/O | `vol2img`, `img2vol`, `rgba-merge` |
| Test data | `bunny` |

### Underlying C++ Types

- **`cvc::geometry`** — surface/volume mesh with points, normals,
  colors, curvatures, tris, quads, tets, hexs.  Read/write via
  plugin I/O (OFF, CVC-RAW, etc.).
- **`cvc::volume`** — multi-dimensional voxel container with bounding
  box, interpolation, data types (UChar through Double).  Read/write
  via plugin I/O (RawIV, RawV, MRC, VTK, Spider, HDF5).
- **`cvc::volume_file_info`** — file metadata (dims, bbox, vars,
  timesteps, types).
- Both stored in the state tree via `state::data()` (boost::any).
- The DSL's `data_object` type wraps `boost::any` — the bridge
  mechanism exists but no domain intrinsics use it.

---

## Design

### Strategy: Opaque Data Objects + Intrinsics

Geometry and volume objects are too complex to represent as DSL
value_t variants.  Instead, they live as **opaque data objects**
in the state tree's `data()` slots, and DSL programs manipulate
them through intrinsics that operate on state paths:

```lisp
;; Load a volume from disk into the state tree
(vol-read "mydata" "/path/to/brain.rawiv")

;; Query properties
(set dims (vol-dims "mydata"))       ;; => (256 256 256)
(set bbox (vol-bbox "mydata"))       ;; => (0.0 0.0 0.0 1.0 1.0 1.0)

;; Volume arithmetic (result stored at new path)
(vol-normalize "mydata" "normalized")
(vol-scale "normalized" 2.0 "scaled")

;; Extract isosurface → geometry
(iso "scaled" 0.5 "surface" (dict "method" "fastcontouring"))

;; Compute SDF from geometry → new volume
(sdf "surface" "sdf-vol" (dict "dim" (list 128 128 128)))

;; Tetrahedralize
(tet "sdf-vol" 0.0 "tet-mesh")

;; Write results
(vol-write "sdf-vol" "/path/to/output.rawiv")
(geom-write "tet-mesh" "/path/to/output.off")
```

Dicts serve as option bags for operations that take many parameters.

### Hash Map Consideration

The current dict type (`vector<pair<string, value_t>>`) is an O(n)
ordered association list.  For typical DSL usage (option bags with
5–20 keys, small configuration dicts), this is fine.

A true hash map would be useful for:
- Volume metadata maps with many entries
- Large lookup tables in processing pipelines  
- Accumulator patterns (histograms, frequency counts)

**Recommendation:** Don't add a separate hash map type.  Instead,
optimize the existing dict implementation by switching from
`vector<pair>` to an **ordered map** (`std::map` or a
`robin_map` with insertion-order tracking) when the dict exceeds
a threshold (e.g. 32 entries).  This keeps the DSL type system
simple while gaining O(log n) or O(1) lookups for large dicts.
This can be done later as an optimization — the dict API stays
the same.

---

## Phase 1: Dict & Collection Gaps

Before adding geometry intrinsics that use dicts as option bags,
fill the usability gaps in the collection library.

### 1.1 Dict Usability

| Intrinsic | Signature | Behaviour |
|-----------|-----------|-----------|
| `has-key` | `(has-key dict key)` | Return `#t` if key exists |
| `get-attr-or` | `(get-attr-or dict key default)` | Return value or default |
| `dict?` | `(dict? val)` | Type predicate |
| `dict-entries` | `(dict-entries dict)` | List of `(key value)` pairs |
| `dict-from-pairs` | `(dict-from-pairs lst)` | Construct dict from list of pairs |
| `dict-update` | `(dict-update dict key fn)` | Update value at key with function |

### 1.2 Collection Completeness

| Function | Signature | Behaviour |
|----------|-----------|-----------|
| `find` | `(find lst pred)` | First element passing pred, or nil |
| `find-index` | `(find-index lst pred)` | Index of first match, or -1 |
| `any?` | `(any? lst pred)` | `#t` if any element passes |
| `all?` | `(all? lst pred)` | `#t` if all pass |
| `take` | `(take lst n)` | First n elements |
| `drop` | `(drop lst n)` | Skip first n |
| `list?` | `(list? val)` | Type predicate (if missing) |

### 1.3 Fix: collections.map/filter/reduce with closures

Currently `collections.map`, `filter`, `reduce` only work with
`native_fn` — calling with a DSL-defined closure throws.  These
need to dispatch through the evaluator to support closures.

---

## Phase 2: Volume Intrinsics

Map the CVC CLI volume commands to DSL intrinsics.  Every operation
reads from and writes to state tree data paths.

### 2.1 Volume I/O

| Intrinsic | Signature | CLI equiv |
|-----------|-----------|-----------|
| `vol-read` | `(vol-read state-path file-path)` | `cvc copy` |
| `vol-write` | `(vol-write state-path file-path)` | `cvc copy` |
| `vol-info` | `(vol-info file-path)` | `cvc info` |
| `vol-stats` | `(vol-stats state-path)` | `cvc stats` |

`vol-read` loads a volume file into `state(state-path).data()`.
`vol-info` returns a dict with dims, bbox, data type, num vars,
num timesteps — no loading required.

### 2.2 Volume Properties

| Intrinsic | Signature | Returns |
|-----------|-----------|---------|
| `vol-dims` | `(vol-dims path)` | `(xdim ydim zdim)` |
| `vol-bbox` | `(vol-bbox path)` | `(xmin ymin zmin xmax ymax zmax)` |
| `vol-type` | `(vol-type path)` | `"float"`, `"uchar"`, etc. |
| `vol-num-vars` | `(vol-num-vars path)` | integer |
| `vol-num-timesteps` | `(vol-num-timesteps path)` | integer |
| `vol-min` | `(vol-min path)` | float |
| `vol-max` | `(vol-max path)` | float |
| `vol-voxel` | `(vol-voxel path x y z)` | float (interpolated) |

### 2.3 Volume Arithmetic

| Intrinsic | Signature | CLI equiv |
|-----------|-----------|-----------|
| `vol-add` | `(vol-add a-path b-path out-path)` | `cvc add` |
| `vol-subtract` | `(vol-subtract a b out)` | `cvc subtract` |
| `vol-scale` | `(vol-scale in-path factor out-path)` | `cvc scale` |
| `vol-normalize` | `(vol-normalize in out)` | `cvc normalize` |
| `vol-clip` | `(vol-clip in min max out)` | `cvc clip` |
| `vol-negate` | `(vol-negate in out)` | `cvc negate` |
| `vol-mask` | `(vol-mask in mask-path out)` | `cvc mask` |
| `vol-downsample` | `(vol-downsample in factor out)` | `cvc downsample` |
| `vol-rotate` | `(vol-rotate in angle out)` | `cvc rotate` |

### 2.4 Volume Analysis & Projection

| Intrinsic | Signature | CLI equiv |
|-----------|-----------|-----------|
| `vol-ssim` | `(vol-ssim a b)` | `cvc ssim` → returns float |
| `vol-project` | `(vol-project in angle out)` | `cvc project` |
| `vol-backproject` | `(vol-backproject in angle out)` | `cvc backproject` |

### 2.5 Image Conversion

| Intrinsic | Signature | CLI equiv |
|-----------|-----------|-----------|
| `vol-to-slices` | `(vol-to-slices in dir fmt)` | `cvc vol2img` |
| `vol-from-slices` | `(vol-from-slices dir out)` | `cvc img2vol` |
| `vol-rgba-merge` | `(vol-rgba-merge r g b a out)` | `cvc rgba-merge` |

---

## Phase 3: Geometry Intrinsics

### 3.1 Geometry I/O

| Intrinsic | Signature |
|-----------|-----------|
| `geom-read` | `(geom-read state-path file-path)` |
| `geom-write` | `(geom-write state-path file-path)` |
| `geom-bunny` | `(geom-bunny state-path)` — built-in test mesh |

### 3.2 Geometry Properties

| Intrinsic | Returns |
|-----------|---------|
| `geom-num-verts` | integer |
| `geom-num-tris` | integer |
| `geom-num-quads` | integer |
| `geom-num-tets` | integer |
| `geom-num-hexs` | integer |
| `geom-type` | `"surface-tri"`, `"volume-tet"`, etc. |
| `geom-bounds` | `(xmin ymin zmin xmax ymax zmax)` |
| `geom-has-normals` | bool |
| `geom-has-colors` | bool |

### 3.3 Geometry Operations

| Intrinsic | Signature |
|-----------|-----------|
| `geom-compute-normals` | `(geom-compute-normals path)` — in-place |
| `geom-invert-normals` | `(geom-invert-normals path)` — in-place |
| `geom-smooth` | `(geom-smooth path iterations)` — in-place |
| `geom-quality-improve` | `(geom-quality-improve path)` — in-place |
| `geom-merge` | `(geom-merge a-path b-path out-path)` |

### 3.4 Geometry ↔ Volume Conversion

These are the core scientific computing operations:

| Intrinsic | Signature | C++ |
|-----------|-----------|-----|
| `sdf` | `(sdf geom-path vol-path opts-dict)` | `cvc::sdf()` |
| `iso` | `(iso vol-path isovalue geom-path opts-dict)` | `cvc::iso()` |
| `tet` | `(tet vol-path isovalue geom-path opts-dict)` | `cvc::tetrahedralize()` |
| `hex` | `(hex vol-path isovalue geom-path opts-dict)` | `cvc::hexahedralize()` |
| `layer-mesh` | `(layer-mesh vol outer inner geom opts)` | `cvc::tetrahedralize2()` |

Options dicts allow passing algorithm parameters:

```lisp
(iso "brain-vol" 128.0 "brain-surf"
  (dict "method" "fastcontouring"
        "improve" #t
        "compute-normals" #t))

(sdf "brain-surf" "brain-sdf"
  (dict "dim" (list 256 256 256)
        "flip-normals" #f))
```

---

## Phase 4: Pipeline Patterns & Replay

### 4.1 Pipeline Example

A complete volume → mesh processing pipeline in the DSL:

```lisp
(defun process-brain (input-file output-dir)
  (begin
    ;; Load
    (vol-read "raw" input-file)
    (print (str "Loaded: " (vol-dims "raw")))

    ;; Pre-process
    (vol-normalize "raw" "norm")
    (vol-clip "norm" 0.1 0.9 "clipped")

    ;; Extract isosurface
    (iso "clipped" 0.5 "surface"
      (dict "method" "fastcontouring"
            "improve" #t
            "compute-normals" #t))
    (print (str "Surface: " (geom-num-tris "surface") " tris"))

    ;; Tetrahedralize for FEM
    (tet "clipped" 0.5 "tet-mesh"
      (dict "improve" #t))
    (print (str "Tet mesh: " (geom-num-tets "tet-mesh") " tets"))

    ;; Write results
    (geom-write "surface" (str output-dir "/brain-surface.off"))
    (geom-write "tet-mesh" (str output-dir "/brain-tet.off"))

    ;; Return summary
    (dict "tris" (geom-num-tris "surface")
          "tets" (geom-num-tets "tet-mesh"))))

(process-brain "/data/brain.rawiv" "/output")
```

### 4.2 Batch Processing with Generators

```lisp
(defun process-all (file-list output-dir)
  (for f file-list
    (begin
      (print (str "Processing: " f))
      (process-brain f output-dir))))
```

### 4.3 Distributed Pipeline

```lisp
;; Coordinator splits work across nodes
(defun distribute-pipeline (files)
  (for f files
    (begin
      (state-set (str "jobs." f ".status") "pending")
      (state-set (str "jobs." f ".input") f)
      (msg-send "work-queue" f))))

;; Worker process
(defun worker ()
  (while #t
    (set job (msg-recv "work-queue"))
    (state-set (str "jobs." job ".status") "running")
    (process-brain
      (state-get (str "jobs." job ".input"))
      "/output")
    (state-set (str "jobs." job ".status") "done")))
```

---

## Implementation Order

| Phase | Scope | Depends on |
|-------|-------|------------|
| **1** | Dict gaps (`has-key`, `dict?`, `get-attr-or`, etc.) + closure fix for map/filter/reduce | — |
| **2** | Volume intrinsics (I/O, properties, arithmetic) | Phase 1 (dicts for options) |
| **3** | Geometry intrinsics (I/O, properties, SDF/iso/tet) | Phase 2 (volumes exist) |
| **4** | Pipeline patterns, batch, distributed examples | Phase 2+3 |

Each phase is independently testable and shippable.

---

## Open Questions

1. **In-place vs copy semantics.**  Should `vol-normalize` modify
   in-place or always produce a new data object at a different path?
   The design above uses explicit input/output paths (copy semantics)
   for clarity and reproducibility.  In-place variants could be added
   as `vol-normalize!` if needed.

2. **Error handling.**  File not found, unsupported format, CGAL
   crashes — these need to be surfaced as DSL-level errors, not
   C++ exceptions that crash the scheduler.  Each intrinsic should
   catch exceptions and return nil or raise a DSL error.

3. **Large data and memory.**  A 512³ float volume is 512 MB.
   Multiple copies in the state tree can exhaust memory quickly.
   The `memory-limit` resource policy should account for data
   objects, and intrinsics should support streaming/chunked
   processing where possible.

4. **CUDA acceleration.**  Several volume operations have CUDA
   paths.  The DSL intrinsics should transparently use GPU when
   available, controlled by a config flag or dict option.

5. **Subvolume / ROI extraction.**  Useful for processing regions
   of interest without loading full volumes.  Could add
   `vol-subvol` intrinsic wrapping `volume::sub_volume()`.

6. **VTK integration.**  The geometry/volume I/O supports VTK
   format.  Should we expose VTK pipeline operations through the
   DSL, or keep it at the cvc::geometry/volume abstraction level?
   Recommendation: stay at the cvc level for now.
