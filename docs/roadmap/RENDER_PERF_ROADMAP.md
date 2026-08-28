# Render Performance Roadmap (cvcGL nav demos + StageLighting scenes)

Goal: keep the full building geometry and every agent's full mesh, but stop
paying to draw what the viewer can't distinguish from a pixel. Frame budget
target on a laptop iGPU + shadow map on: 60 FPS at Austin south with 500 agents.

Baseline (2026-08, `nav_city_swarm` on Austin south bundle):
- `buildings.glb`: 978,242 tris, 11,272 source meshes, merged to 1 VTK actor.
- `buildings_flat.glb`: 195,704 tris (already shipped in the bundle — the
  `--lite` short-term escape hatch).
- Humvees: 1 template glyph mesh, drawn N times via instanced pack/unpack.
- Shadow map: 2048², repopulated every 3rd frame across the full scene AABB.

Every phase below is opt-in behind a runtime flag first; measured wins get
promoted to the default. No phase silently changes rendering output at
runtime — visual regressions must be gated by a comparison screenshot in
`docs/img/`.

---

## Phase 1 — Frustum culling for the building mesh (fast, big win)

The merged buildings mesh is one 1M-tri actor. VTK renders the whole thing on
every pass (main + shadow), even when the camera looks at 10% of the city.

- Split the merged mesh into a coarse spatial grid (e.g. 8×8 tiles over
  bounds), one GeometryNode per tile.
- Per frame, project each tile's AABB against the camera frustum in a cheap
  CPU test (main-cam AND shadow-cam frustums); toggle actor visibility.
- Shadow pass sees only tiles within the light's shadow frustum + a small
  guard band, so a top-lit scene stops rasterizing the far side of the map.

Success criteria: 4-6× fewer tris rasterized on a chase-cam close-up shot in
Austin south. No visual difference at frame captures.

## Phase 2 — Distance LOD for agents (Humvees)

Every agent instance rasterizes the full glyph mesh even when it's a 2×2 px
speck on the horizon. On Austin's 3 km span this dominates the fragment work
for high-N runs.

- AgentGlyphs gains a template-set: `build_template_lod({near, mid, far})`,
  each with its own vertex/tri arrays baked into the same instanced layout.
- `pack_z` partitions the instance list by camera distance thresholds and
  streams three separate `updateVertices` batches per frame — one node per
  LOD. Threshold = angular-size-based (metric: mesh diameter / distance).
- `far` bucket collapses to a billboard quad with a screen-oriented
  agent-colour + heading arrow (see Phase 5 for the impostor path).

Success criteria: 500-agent Austin swarm at 60 FPS with shadows on. The chase
cam still sees the full mesh at close range; a top-down orbit sees mostly
billboards.

## Phase 3 — Instanced draws via VTK's glyph3D / poly-instancing

`AgentGlyphs` currently packs a big per-frame vertex buffer for the whole
fleet — CPU-side pack cost scales with N. VTK 9.3+ has proper
GLSL-side instancing (`vtkOpenGLPolyDataMapper` w/ instanced glyph3D). Move
the per-instance pose (translation + heading rotation + colour) into a
per-instance attribute buffer; the vertex buffer stays static.

Complements Phase 2: each LOD is its own instanced draw call.

Success criteria: pack CPU cost O(N) → O(1); measured `updateVertices`
throughput stays constant as N doubles from 500 → 1000.

## Phase 4 — Shadow-map budget scaling

- Shadow update interval already dynamic (`setShadowUpdateInterval(3)`); make
  it also scale with camera velocity (slower cam → less frequent updates).
- Shadow map resolution scales with viewport size: 4K viewport gets 2048²,
  1080p gets 1024². Currently hard-coded to 2048² regardless.
- Cull shadow-caster list to the light's frustum (independent of Phase 1's
  main-cam cull).

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

Success criteria: 5 km²+ scene renders at 60 FPS with 1000 agents visible;
close-up chase cam still shows the full mesh (LOD boundary is
distance-based and invisible in motion).

## Phase 6 — Async streaming / on-demand tile load

Only relevant for scenes larger than any current bundle. Tile grid from
Phase 1 becomes on-demand: tiles outside a ring around the camera are
unloaded from VRAM (kept on disk / mmap), loaded back when the camera moves
close. Requires the LOD bake to be per-tile and independent (Phase 5).

Deferred until a scene exceeds ~10 M tris — Austin south (~1 M tris) does
not need this.

## Phase 7 — Wasm-specific: SharedArrayBuffer + worker sim

Not a rendering item, but the wasm demo's frame budget is halved by running
sim + render on one thread. Wire the `CVC_NAV_DEMO_SIM_WORKER` branch into
the pthread wasm build so the demo stays interactive at 500 agents.

Blocked on: the deploy target serving COOP/COEP (needed for SharedArrayBuffer).

---

## Instrumentation

Before any phase lands, `nav_city_swarm` and `lsystem_forest` must print a
per-second frame breakdown behind a `--profile` flag: tris rasterized (main
vs shadow), draw calls, pack CPU ms, VTK render ms, ImGui ms. Regressions
are visible without a full trace.

## Short-term escape hatches (already available)

- `--lite` on `nav_city_swarm`: use `buildings_flat.glb` (195k tris) instead
  of `buildings.glb` (978k tris). Same footprints, flat corners. Ships in
  the current bundles.
- `--no-shadows`: skips the shadow pass entirely.
- Shadows checkbox in the demo's control panel: same at runtime.
