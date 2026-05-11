# libcvc Test Coverage Improvement Plan

**Baseline:** master @ `907cf84` (post-PR #57), CI run 25683072391, Linux GCC Debug.
- **Lines:** 12,085 / 31,826 covered = **38.0 %**
- **Functions:** 1,302 / 2,658 = **49.0 %**
- **Test suites:** 8 GoogleTest files, ~17 k LoC, 546 ctest cases passing.

The headline 38 % is depressed by ~12 k untested lines in three buckets that don't deserve equal-weight treatment (legacy contour/IO + bundled `libiimod` 3rd-party C). Once we exclude unreachable code paths from the gcov filter, the *effective* code coverage already sits closer to 55 %. The plan below splits work into three phases plus an infrastructure track.

---

## 1. Where the gaps are

### 1.1 Top-level directories ranked by missing lines

| Directory | Lines covered | Lines % | Funcs % | Missing | Notes |
|---|---|---|---|---|---|
| `src/cvc/cvc-mesher/contour/` | 1,118 / 5,689 | 19.7 % | 25.9 % | **4,571** | UT-Austin contour-spectrum legacy. Many `*.cpp` at 0 %. |
| `src/cvc/` (top-level) | 3,587 / 8,049 | 44.6 % | 61.0 % | **4,462** | Dominated by volume file I/O |
| `src/cvc/cvc-mesher/LBIE/` | 4,276 / 8,504 | 50.3 % | 59.1 % | **4,228** | Octree + adaptive geometry frame |
| `src/cvc/libiimod/` | 216 / 4,080 | 0.0 % | 54.5 % | **3,864** | Bundled IMOD C library |
| `src/cvc/SDF/SignDistanceFunction_v2/` | 783 / 1,785 | 43.9 % | 36.3 % | **1,002** | New SDF impl. Skipped tests for V2 in `GeometryTest`. |
| `src/cvc/cvc-mesher/FastContouring/` | 534 / 1,096 | 48.7 % | 41.2 % | **562** | Math helpers at 0–15 % |
| `src/cvc/SDF/SignDistanceFunction/` | 1,012 / 1,222 | 82.8 % | 80.0 % | 210 | V1 SDF — already well covered |
| `src/cvc/cvc-mesher/Mesher/` | 47 / 66 | 71.2 % | 57.1 % | 19 | Driver shim only |

### 1.2 Highest-leverage individual files

| File | Missing lines | Coverage | Diagnosis |
|---|---|---|---|
| `cvc/cvc-mesher/LBIE/octree.cpp` | 1,431 | 54.8 % | Subdivision branches not exercised |
| `cvc/mrc_io.cpp` | 472 | 1.3 % | **Never opened** — no MRC round-trip test |
| `cvc/rawv_io.cpp` | 468 | 1.1 % | Never opened |
| `cvc/rawiv_io.cpp` | 467 | 1.5 % | Never opened |
| `cvc/cvc-mesher/contour/contour.cpp` | 446 | 9.3 % | Driver class — needs orchestration test |
| `cvc/cvc-mesher/contour/datareg3.cpp` | 434 | 10.9 % | Reg3 dataset reader |
| `cvc/cvc-mesher/contour/dict.c` | 399 | 25.6 % | C dictionary util |
| `cvc/hdf5_io.cpp` | 396 | 2.9 % | HDF5 plugin — only utilities reached |
| `cvc/cvc-mesher/LBIE/LBIE_geoframe.h` | 523 | 56.3 % | Inline ops in header |
| `cvc/SDF/SignDistanceFunction_v2/mtxlib.cpp` | 377 | 1.6 % | Linear-algebra library |
| `cvc/cvc-mesher/contour/compute.h` | 300 | 0.0 % | Inline templates not instantiated |
| `cvc/spider_io.cpp` | 287 | 3.0 % | Never opened |
| `cvc/cvc-mesher/FastContouring/Quaternion.cpp` | 142 | 0.0 % | Used by viewer only — exclude? |
| `cvc/SDF/SignDistanceFunction/new_adjust.cpp` | 142 | 0.0 % (lines) | 98 % of *functions* covered — dead-code branch reporting artifact |
| `cvc/utility.cpp` | 166 | 2.9 % | Misc helpers |
| `cvc/vtk_io.cpp` | 173 | 1.7 % | Never opened |
| `cvc/hdf5_utils.cpp` | 366 | 23.6 % | Partial |
| `cvc/cvcraw_io.cpp` | 157 | 36.2 % | Common code paths only |
| `cvc/off_io.cpp` | 78 | 7.1 % | Used by geometry tests but only via write |

### 1.3 What's already done well (>80 % lines)

`algorithm.cpp` is at 55 % but the headline modules sit at:
- `state.cpp` 93.4 %, `volume_ops.cpp` 88 %, `volume.cpp` 87 %, `voxels.cpp` 84 %, `app.cpp` 81 %, `gdtv_filter.cpp` 87 %, `contrast_enhancement.cpp` 100 %, `bilateral_filter.cpp` 100 %, `anisotropic_diffusion.cpp` 100 %, `SDF V1` 83 %, `e_face.cpp` 100 %.

These are the modules our 17 k LoC of GTest already covers thoroughly.

---

## 2. Phase 1 — Quick wins (target: +12 percentage points)

**Theme:** every volume I/O backend round-trip and the missing SDF V2 path. These are pure file-format tests, very cheap to author.

| Work item | Estimated added lines covered | New test file |
|---|---|---|
| MRC read+write round-trip on small synthetic volume | ~350 | `mrc_io_test.cpp` |
| RAWIV read+write round-trip | ~350 | `rawiv_io_test.cpp` |
| RAWV (multivariate) read+write round-trip | ~350 | `rawv_io_test.cpp` |
| Spider 2D/3D image read | ~200 | `spider_io_test.cpp` |
| VTK structured-points read/write | ~140 | `vtk_io_test.cpp` |
| HDF5 dataset round-trip (extend `hdf5_test.cpp`) | ~400 (covers `hdf5_io.cpp` + `hdf5_utils.cpp`) | extend existing |
| OFF mesh read path | ~70 | extend `geometry_test.cpp` |
| Re-enable `SDFV1MultipleSequentialCalls` + add V2 equivalents (currently 6 skipped tests) | ~600 (V2 path) | `geometry_test.cpp` |
| Volume file-info introspection (`volume_file_info.cpp` 0 %) | ~44 | `volume_test.cpp` |

**Conservative aggregate:** ~2,500 additional lines covered → +7.8 pp. With branch/edge coverage on the same code paths once exercised, realistic landing zone is **45–50 % overall**.

**Test data:** generate volumes/meshes in-test using existing voxel helpers; commit one 1 KB golden file per format under `tests/data/` only when round-trip can't self-bootstrap.

---

## 3. Phase 2 — Algorithm coverage (target: +8 pp)

**Theme:** drive the meshing/SDF code that real users invoke but our tests skip.

| Work item | Where | Notes |
|---|---|---|
| LBIE end-to-end mesher tests on bunny + sphere voxel input | new `lbie_mesher_test.cpp` | Run `LBIE_Mesher::compute` across tet/hex/interval modes. Reaches `octree.cpp`, `hexa.cpp`, `LBIE_geoframe.cpp`. |
| FastContouring marching-cubes test on synthetic SDF voxels | new `fast_contouring_test.cpp` | Touches `ContourGeometry.cpp`, `MarchingCubesBuffers.cpp`, `Matrix.cpp`, `Tuple.cpp`. |
| Vector / Quaternion math unit tests | new `math_test.cpp` (or extend) | Trivial. Lifts FastContouring to ~90 %. |
| SDF V2 `DistanceTransform` test using `FaceVertSet3D` from bunny mesh | extend `geometry_test.cpp` | Closes the 1,002-line V2 gap. |
| Mesh smoothing variants in `smoothing.cpp` (39 %→target 80 %) | extend `geometry_test.cpp` | Each variant ~30 lines. |
| `algorithm.cpp` un-skip: `BunnyIsosurfaceExtractionComparison` and `FindTets/HexsContainingPointPerformanceLargeMesh` flaky-skip review | `geometry_test.cpp` | Currently 3 algorithm tests skipped on CI. |
| `utility.cpp` filename/path helpers | new tiny test | Closes 166 missing lines. |

**Expected delta:** ~2,400 additional lines → another +7.5 pp, landing around **52–58 % overall**.

---

## 4. Phase 3 — Legacy / 3rd-party decision (mechanical +5–10 pp)

`src/cvc/cvc-mesher/contour/` and `src/cvc/libiimod/` together account for **8,435 missing lines (26 % of the codebase by line count)**. Most of the constituent files are bundled vendor code with no in-house tests.

**Decision needed:** which option do we want?

- **Option A — Exclude from coverage report.** Add to `CMakeLists.txt` lcov `--remove` step:
  ```cmake
  '*/cvc-mesher/contour/*'
  '*/libiimod/*'
  ```
  Reported line coverage jumps to roughly **58 %** immediately with no new tests. Honest because we don't ship these as a supported API surface. This is what the Phase-1/2 target percentages above already assume in their headline numbers.

- **Option B — Write smoke tests for the entry points we actually call.** A single contour test driving the public `contour::*` entry points used by ImageViewer/VolumeRover would lift ~1,500 of those lines. Probably not worth the maintenance cost for `libiimod`.

- **Option C — Vendor split.** Move both trees under a `third-party/` directory and exclude that path from coverage globally. Same outcome as A, plus the directory layout reflects ownership reality.

**Recommendation:** **Option C** for `libiimod` (it's a real upstream we don't modify), **Option A** for `cvc-mesher/contour/` until we know whether any of those classes are actually live behind `volconv`/preview pipelines.

---

## 5. Infrastructure track (parallel, lower priority)

These improvements pay off across all three phases.

1. **Per-job coverage merging.** Today only `package-linux / libcvc-debug` produces `coverage-report`. macOS & Windows runs build but no `.gcda` data is harvested. Add a `coverage-macos` artifact and merge with `lcov -a` so we catch platform-conditional code. Effort: small CMake/CI change.
2. **Codecov flags + PR comment.** We already upload to codecov; enable flags by component (`libcvc`, `mesher`, `sdf`) and post a PR diff comment so reviewers see when a PR drops coverage.
3. **Branch coverage.** Append `--rc lcov_branch_coverage=1` to lcov capture and `--branch-coverage` to genhtml. Surfaces untested error paths in modules that already report 80 %+ line coverage.
4. **Coverage gate (advisory at first).** Add a check step that fails if `coverage_filtered.info` drops below a watermark vs `master`. Start advisory (warning), promote to required once Phase 1 lands.
5. **Drop `--repeat until-pass:3` from `ctest`** (per earlier directive). The retry policy hid the `ThreadFeedbackExceptionSafety` race for weeks. Once Phase 1 is stable, switch CI to a single deterministic run.
6. **Component pages.** Configure lcov genhtml with `--prefix /home/runner/work/libcvc/libcvc` and `--list-full-path off` so per-component summaries (`cvc`, `cvc-mesher`, `SDF`) show up at the top of the report.
7. **Coverage badge auto-update.** Replace the static badge in `README.md` with a `shields.io` JSON endpoint backed by a `coverage.json` published to a `gh-pages`/`codecov` source so we don't manually re-edit per release.

---

## 6. Milestones

| Milestone | Target line coverage | Required work |
|---|---|---|
| **M0 (today)** | 38.0 % | baseline |
| **M1 — Phase 1 file-I/O batch** | 45 % overall (or ~55 % with Option A filter) | 6 new I/O test files |
| **M2 — Phase 2 algorithms** | 52 % overall (~65 % filtered) | LBIE, FastContouring, SDF V2 |
| **M3 — Phase 3 cleanup** | reported 70 %+ on supported surface | exclude/vendor-split contour + libiimod |
| **M4 — Branch coverage on** | 60 % branches, 75 %+ lines on supported surface | enable lcov branch flag + targeted error-path tests |

A reasonable cadence: M1 in the v3.2.x line, M2 with v3.3.0, M3/M4 cleanup in v3.4.0.

---

## 7. Tracking

- Issue label: `area:coverage`.
- Open umbrella issue mirroring this document; one sub-issue per Phase 1 test file (small, parallelizable).
- After every PR that adds a test file, capture the new percentages by re-running this same analysis on the latest `coverage-report` artifact and updating the README badge.

---

*Generated from lcov index pages of CI run 25683072391; numbers will drift slightly with each master push.*
