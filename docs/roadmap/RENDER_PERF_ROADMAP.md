# Render Performance Roadmap (cvcGL nav demos + StageLighting scenes)

Goal: keep the full building geometry and every agent's full mesh, but stop
paying to draw what the viewer can't distinguish from a pixel. Frame budget
target on a laptop iGPU with shadow map on: **≥ 45 fps at Austin south with
500 agents; ≥ 60 fps at 1000 agents once Phase 3 lands**.

Scope: `nav_city_swarm`, `nav_fog_ghost`, `bunny_shadow`, `lsystem_forest`
and the `lsystem_lab` successor (see the L-System Laboratory roadmap, which
lands the same `cvc::lod` and world-tiling infrastructure this document
consumes).

---

## Ordering rule — measured, not assumed

`lsystem_forest`, the closest analogue we have with real measurements, is
**CPU-bound in the main loop, not triangle-bound**. Baselines at 1280×800:

| condition | fps |
|---|---|
| shadows on, full window | 24.8 |
| shadows on, quarter-pixel window | 29.2 (+18%) |
| shadows **off**, full window | 21.7 (slower) |

Turning shadows off actually slowed it down; a quarter of the pixels bought
under 20%. The cost lives in per-frame CPU vertex re-pose and re-upload
(`lsystem_forest.cpp:1403-1408` at `WIND_STRIDE=2`, plus regenerated sea
fields) — the `WIND_STRIDE=2 / SEA_STRIDE=4 / CLOUD_STRIDE=8 / SHADOW_STRIDE=16`
constants are the loop already fighting it.

Therefore work is prioritised **animation → generation → update → draw
calls → triangles**, not the other way around. Rasterisation optimisations
(Phase 4 shadow budget, Phase 5 impostors) come after the CPU-side wins
(Phase 1 culling, Phase 2 distance LOD, Phase 3 instancing).

## Non-goals

- **Not** a networked LOD streaming layer. That's phases 32-37 of the
  volrover3 modernization roadmap (§22.1's `pyramid_builder`, `lod_index`,
  `CvcLod` gRPC service, W-TinyLFU eviction). We claim only
  `inc/cvc/lod/select.h` — the single-process selection math — but adopt
  its vocabulary (`desired_pixel_error`, priority = projected area / distance²)
  so the streaming layer slots in later.
- **Not** mesh decimation of authored `.glb` files as a first pass. For
  imported meshes (buildings.glb, Humvee.glb, Soldier.glb) the ladder is
  culling → LOD-swap → impostor; decimation is a bake-time tool
  (`quadric_decimate`, volrover3 §26.4.2) applied to the archetype library.
- **Not** GPU-instanced glyphs on wasm. `vtkOpenGLGlyph3DHelper::GlyphRender`
  gates the instanced path on `GLAD_GL_ARB_instanced_arrays` — a desktop ARB
  extension WebGL2 never advertises — so it degrades to one draw call per
  glyph in the browser. Native-only paths rot; Phase 3's instancing has to
  degrade cleanly to a merged-actor fallback.
- **Not** touching `libcvc/inc/cvc/spatial/`. Reserved by the volrover3
  roadmap §20.18.10 (`world_tree.h`, `kd_tree.h`, `bsp.h`, `octree.h`,
  `portal.h`, `coarse_summary.h`). Our culling structure is a flat 32×32
  uniform grid (128 m tiles) — see Phase 1.

## Baselines (2026-08)

**Buildings — Austin south bundle:**
- `buildings.glb`: **978,242 triangles, 11,272 source meshes**, merged to
  1 VTK actor.
- `buildings_flat.glb`: **195,704 triangles**, same footprints, flat corners.
  Shipped in the bundle — the `--lite` short-term escape hatch.

**Agents:** one template glyph mesh per agent, drawn N times via
`AgentGlyphs::pack`/`pack_z` (CPU-side pack, streamed via
`GeometryNode::updateVertices`).

**Shadow map:** 2048², repopulated every 3rd frame across the full scene
AABB (both main-cam AND directional-light frustum see the whole city).

**L-System Laboratory targets (from PR #249, small preset):**
| | small | standard | large |
|---|---|---|---|
| Authored plant instances | 60 000 | 480 000 | 1 200 000 |
| Drawn instances/frame | ~6 500 | ~22 000 | ~28 000 |
| GPU ray-cast volumes | 1 | 2 | 2 |
| Target fps @ 1280×800 shadows-on | ≥ 30 | ≥ 45 | ≥ 45 |

## Rules that carry across all phases

- **LOD may never alter simulation correctness.** volrover3 §22 (`:7734`):
  *"nothing in this section changes simulation correctness."* Enforced
  here: the material/nav export always runs at full fidelity from the
  analytic surface function, independent of any render rung. A world
  exported while the camera is at T4 must be byte-identical to one exported
  at T0. Unit test.
- **Per-leaf-per-frame LOD, not per-actor** (volrover3 §20.18.5). Coarse
  summaries are coherent across a leaf and avoid the popping that per-actor
  LOD switches cause.
- **No silent visual regressions.** Every phase is opt-in behind a runtime
  flag first; measured wins get promoted to the default, and only after a
  before/after screenshot comparison is committed to `docs/img/`.
- **State-tree keys are snake_case, rooted at `demo.lod.*` / `lab.lod.*`.**
  (volrover3 §20.13.7 says camelCase, §22.1.6 says snake_case for the same
  path — we picked snake_case to match the rest of the cvcGL state tree
  and flagged the inconsistency; the roadmap owner should fix §20.13.7.)

---

## Phase 1 — Frustum culling for the building mesh (fast, big win)

The merged buildings mesh is one 1M-tri actor. VTK renders the whole thing
on every pass (main + shadow), even when the camera looks at 10% of the city.

- Split the merged mesh into a coarse spatial grid at load time. Use a
  **flat 32×32 uniform grid** (128 m tiles on Austin's 4 km span, 64×64 =
  64 m tiles on the `large` preset). One `GeometryNode` per non-empty tile,
  addressed by Morton code for cache-friendly traversal.
- Per frame: 2-D rect test of the frustum's ground footprint against each
  tile rect, plus a per-tile `[z_min, z_max]` slab test. 1024 tiles at
  ~5 ns each is **~5 µs/frame** — deliberately not an octree; octrees are
  slower at this N and would squat the reserved `cvc/spatial/` namespace.
- Independent frustums for the main camera and the directional light's
  shadow camera, plus a small guard band on the shadow side.

**Selection API** (lands as part of the L-System Lab work, but shared):
```cpp
// inc/cvc/lod/select.h — pure math. No VTK, no I/O, no allocation on hot path.
struct select_params {
  double desired_pixel_error = 2.0;
  // ...hysteresis, projected-area priority, etc.
};
// Distance at which world_error_m first exceeds desired_pixel_error.
```
`desired_pixel_error` defaults per preset: 1.0 (pristine), 2.0 (default),
3.5 (aggressive) — 2.0 halves terrain triangles at an error nobody sees at
1280×800.

**Park unused vertices at the first used vertex's position, not the origin**,
so the bounding box stays tight. Loose bboxes make the directional shadow
map fit the wrong volume and make the frustum culler pessimistic.

Success criteria: 4-6× fewer tris rasterized on a chase-cam close-up shot in
Austin south. No visual difference at frame captures.

## Phase 2 — Distance LOD for agents (Humvees)

Every agent instance rasterizes the full glyph mesh even when it's a 2×2 px
speck on the horizon. On Austin's 3 km span this dominates the fragment work
for high-N runs.

- `AgentGlyphs` gains a template-set: `build_template_lod({near, mid, far})`,
  each with its own vertex/tri arrays baked into the same instanced layout.
- `pack_z` partitions the instance list by camera distance thresholds and
  streams three separate `updateVertices` batches per frame — one node per
  LOD. Threshold = angular-size-based (metric: mesh diameter / distance,
  compared against `desired_pixel_error` from Phase 1's selection API).
- `far` bucket collapses to a billboard quad with a screen-oriented
  agent-colour + heading arrow (see Phase 5 for the impostor path).

**Adapted from Weber & Penn 1995** *(Creation and Rendering of Realistic
Trees)*, §5 range degradation: *"With progressively increasing ranges, a
tree will re-interpret stem meshes as lines and leaf polygons as points…
A 100 000 facet tree geometry may be rendered at 2 kilometres as about 30
lines and 1000 points."* Re-interpret, do not convert. For imported meshes
(Humvee/Soldier), the pre-baked LOD tiers are the analogue.

Success criteria: 500-agent Austin swarm at 60 FPS with shadows on. The
chase cam still sees the full mesh at close range; a top-down orbit sees
mostly billboards.

## Phase 3 — Instanced draws via GLSL attribute buffers (native), merged-actor fallback (wasm)

`AgentGlyphs` currently packs a big per-frame vertex buffer for the whole
fleet — CPU-side pack cost scales with N. VTK 9.3+ has proper GLSL-side
instancing (`vtkOpenGLPolyDataMapper` w/ instanced glyph3D) on desktop GL.
Move the per-instance pose (translation + heading rotation + colour) into
a per-instance attribute buffer; the vertex buffer stays static.

**Wasm caveat:** `vtkOpenGLGlyph3DHelper::GlyphRender` gates the instanced
path on the desktop-only extension `GLAD_GL_ARB_instanced_arrays`, so on
WebGL2 it silently degrades to one draw call per glyph — worse than today's
merged-actor pack. The Phase 3 refactor must therefore fall back to the
current merged-actor path on wasm, guarded by a runtime capability check
rather than a compile-time `#ifdef` (so a single native binary works on any
GPU tier).

Complements Phase 2: each LOD is its own instanced draw call.

Success criteria: pack CPU cost O(N) → O(1) on native; measured
`updateVertices` throughput stays constant as N doubles from 500 → 1000.
Wasm builds keep parity with today (no regression).

## Phase 4 — Shadow-map budget scaling

- Shadow update interval already dynamic (`setShadowUpdateInterval(3)`); make
  it also scale with camera velocity (slower cam → less frequent updates).
- Shadow map resolution scales with viewport size: 4K viewport gets 2048²,
  1080p gets 1024². Currently hard-coded to 2048² regardless.
- Cull shadow-caster list to the light's frustum (independent of Phase 1's
  main-cam cull).
- **Wind is not a shadow-map input.** `lsystem_forest` re-poses vertices
  on the CPU (`WIND_STRIDE=2`) and re-uploads them; if the shadow pass reads
  the same buffers it re-rasterises the whole scene every re-pose. Wind
  moves to a **vertex shader** function of world position + time (see
  L-System Lab §4.7-4.9); CPU sway is hard-capped at **24 plants at every
  world size** — enough for the near tier, no more.

Success criteria: shadow pass is <20% of frame time at every viewport size,
without visible shadow flicker under normal camera motion.

## Phase 5 — Building & vehicle impostors (billboard atlas)

For the `far` LOD bucket, render an impostor: a screen-facing billboard whose
texture is a pre-baked orthographic projection of the mesh from N view
angles (e.g. 8 azimuths × 4 elevations = 32 views), packed into a texture
atlas. Cost per instance drops to 2 tris.

- Bake step: offline (`tools/bake_impostors`) — one atlas per template mesh,
  written next to the .glb.
- Runtime: the `far` LOD reads the atlas UV via (azimuth-index,
  elevation-index) computed from the camera-relative pose.
- Building impostors are per-tile (Phase 1's tile grid), baked from that
  tile's local mesh.
- Cross-fade between tiers via **hashed alpha testing** (Wyman & McGuire
  2017) — no ordered-blend requirement, works with the deferred shadow pass.

Ships with **[Décoret et al. 2003]** billboard-cloud construction as prior
art; **SpeedTree SDK LOD docs** for the shrink-and-grow leaf-card
transitions the vegetation tier reuses.

Success criteria: 5 km²+ scene renders at 60 FPS with 1000 agents visible;
close-up chase cam still shows the full mesh (LOD boundary is
distance-based and invisible in motion).

## Phase 6 — Volumetric rendering budget (sea, sky, cloud shadow)

`lsystem_forest`'s two ray-cast volumes (sea + sky) are on the frame budget.
`nav_city_swarm` currently has none but will gain water if the L-System Lab
work lands. Measured guidance from PR #249's WATER-RENDERING-ROADMAP:

- **Do not use vtkOpenGLProjectedTetrahedraMapper for homogeneous volumes.**
  It is 100% CPU-bound at ≈0.43 µs/tet/frame (`ms == cpu_ms` from 2.4k to
  998k tets), gets no distance relief (18.73 → 18.60 ms when shrunk to 6% of
  screen), and is 3.4× the surface-render cost at 12k tets, 32× at 101k,
  70× at 246k. For homogeneous water a surface **is not an approximation** —
  with constant τ, `exp(−τ·L)` depends only on chord length, which the
  boundary determines. Tets stay as an opt-in scientific tier for fields
  that live natively on unstructured meshes.
- Cloud volumes: stay 2-max per scene; scale sample count with viewport.
  Peak-piercing correctness (opaque terrain terminating the volume ray)
  needs `vtkGPUVolumeRayCastMapper` reading opaque depth, which
  `SceneGraph.cpp:1044-1074` wires but has never been exercised with 500 m
  of mountain inside a cloud slab — a **named risk with a screenshot test**.

## Phase 7 — Async streaming / on-demand tile load

Only relevant for scenes larger than any current bundle. Tile grid from
Phase 1 becomes on-demand: tiles outside a ring around the camera are
unloaded from VRAM (kept on disk / mmap), loaded back when the camera moves
close. Requires the LOD bake to be per-tile and independent (Phase 5).

Deferred until a scene exceeds ~10 M tris — Austin south (~1 M tris) does
not need this. Procedural scenes (L-System Lab) never need this either:
their world is regenerable from a ~200-byte seed, there is nothing to
stream and nothing to invalidate.

## Phase 8 — Wasm-specific: SharedArrayBuffer + worker sim

Not a rendering item, but the wasm demo's frame budget is halved by running
sim + render on one thread. Wire the `CVC_NAV_DEMO_SIM_WORKER` branch into
the pthread wasm build so the demo stays interactive at 500 agents.

Blocked on: the deploy target serving COOP/COEP (needed for SharedArrayBuffer).

---

## Instrumentation

Before any phase lands, `nav_city_swarm` / `nav_fog_ghost` / `lsystem_forest`
must print a per-second frame breakdown behind a `--profile` flag: tris
rasterized (main vs shadow), draw calls, pack CPU ms, VTK render ms,
ImGui ms. Regressions are visible without a full trace.

The L-System Lab has already speced this UI: rung-colour overlay (T0 green
→ T4 grey; A0 white → A3 magenta), tile grid with per-tile rung labels and
`zmin/zmax` boxes, widened-frustum cull-volume overlay (so "why did that
rebuild fire?" is answerable), `Force rung: Auto / T0 / …`, **`Freeze LOD
camera`** (fly away and inspect the selection as it *was* — the single most
useful LOD debug control that exists), budget bars with the binding
constraint highlighted. Nav demos should adopt the same overlay so the
diagnostic surface is identical across scenes.

## Short-term escape hatches (already available)

- `--lite` on `nav_city_swarm`: use `buildings_flat.glb` (195k tris) instead
  of `buildings.glb` (978k tris). Same footprints, flat corners. Ships in
  the current bundles. (PR #255)
- `--no-shadows`: skips the shadow pass entirely.
- Shadows checkbox in the demo's control panel: same at runtime.
- `WIND_STRIDE` / `SEA_STRIDE` / `CLOUD_STRIDE` / `SHADOW_STRIDE` on
  `lsystem_forest`: the loop already fighting per-frame CPU cost.

## References

- **[Weber & Penn 1995]** *Creation and Rendering of Realistic Trees*,
  SIGGRAPH '95 — range degradation (§5).
- **[Losasso & Hoppe 2004]** *Geometry clipmaps*; **[Strugar 2010]** CDLOD;
  **[Ulrich 2002]** chunked LOD — terrain LOD lineage.
- **[Décoret et al. 2003]** *Billboard clouds* — impostor construction.
- **[Wyman & McGuire 2017]** *Hashed alpha testing* — cross-fade mechanism.
- **[Harris & Lastra 2001]** *Real-Time Cloud Rendering* — billboard cloud
  fallback for the deck.
- **[Schneider & Vos 2015]** *Real-Time Volumetric Cloudscapes of Horizon
  Zero Dawn* — cited as the technique we deliberately do NOT ship (2-5 ms
  discrete GPU, 10-20 ms iGPU, against a ~30 fps two-volume ceiling).
- **SpeedTree SDK LOD docs** — shrink-and-grow leaf-card transitions.
- **volrover3 modernization roadmap** §20.13 (client LOD selection),
  §20.18 (spatial namespaces), §22.1 (streaming pyramid vocabulary),
  §26.4.2 (`quadric_decimate`).
- **L-System Laboratory roadmap** (`LSYSTEM-LABORATORY-ROADMAP.md`) — the
  primary consumer of the same `cvc::lod::select` API this document plans.
- **Water Rendering roadmap** (`WATER-RENDERING-ROADMAP.md`) — the
  volumetric-rendering measurements Phase 6 cites.
