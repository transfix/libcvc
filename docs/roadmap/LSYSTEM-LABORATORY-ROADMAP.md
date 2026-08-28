# L-System Laboratory — Design & Roadmap

**Status:** **Revision 2** — design approved for implementation, revised after an adversarial review pass and a rebase. Target branch family `feat/lsystem-lab-*`, baselined on `origin/master` @ **`8b6f426`** (was `10b7904`; `a33851f`, `e97d06c` and `8b6f426` have since merged).
**Owner:** L-System Laboratory session.
**Document path:** `docs/roadmap/LSYSTEM-LABORATORY-ROADMAP.md`
**Last verified against the tree:** 2026-08-27, worktree `/home/joe/src/cvc/wt-libcvc-lsyslab`, rebased onto `8b6f426`.
**What changed in revision 2:** see §17 (Revision history). The archipelago is now specified rather than asserted (§4.3a), the indoor clearance scheme is derived from one number rather than three contradictory ones (§6b.1a), hard classes carry ρ = 0 (§7.2a), the export frame confronts σ-in-cells (§7.1a), and the collision map is rebuilt for the post-merge world (§14).

---

## 0. Executive summary

1. We are building **three new libcvc modules** — `cvc::lsys` (a real rewriting engine), `cvc::world` (terrain, surfaces, interiors, export) and `cvc::lod` (selection math) — plus **one new cvcGL example**, `lsystem_lab`, that is a thin client of them. `src/cvcGL/examples/lsystem_forest.cpp` is never touched.
2. The world is a **multi-island archipelago** at 4096 m (default) / 8192 m (large) / 2048 m (wasm) extent, with mountains to **1420 m** that pierce a cloud deck based at **700 m**. "Archipelago" is a *specified mechanism*, not an adjective: island seeding, per-island biome divergence, a **smooth-max** mask combination operator, channel/bridge widths and a **mandatory outdoor connectivity gate** are all in §4.3a and §7.8.
3. Vegetation, rocks and buildings are **instances of a small archetype library** (≈64 baked archetypes), never unique meshes. 1.2 M authored instances; ≈28 k drawn per frame.
4. The measured bottleneck of the predecessor demo is **per-frame CPU animation**, not triangles (24.8 fps shadows-on, 21.7 fps shadows-**off**, +18 % at 1/4 the pixels). Therefore the LOD ladder is ordered **animation → generation → update → draw calls → triangles**. The wind moves to a vertex shader; CPU sway is hard-capped at **24 plants at every world size**.
5. Terrain surface classes (dirt/gravel/mud/grass/tile/carpet/…) are the **authored truth**. `risk_raw` (f32 [0,1]) and `hard` (u8) are a **derived, versioned, table-lookup projection** onto the (now merged) GRL-SNAM material contract. We export the two contract *inputs* and nothing derived. **Hard classes carry ρ = 0.00**, because the consumer penalises hard separately and a ρ = 1.00 wall bleeds through the consumer's blur into the corridor the agent must walk down (§7.2a).
6. The class map is a **function**, not a stored image: one `raster(grid_spec)` call emits class / risk_raw / hard / occupancy / height together, so the material-vs-occupancy-vs-heightfield misalignment hazard is structurally unrepresentable.
7. **Indoor scenarios are first-class.** One rewriting engine serves plant L-systems and building grammars (two derivation modes, one parser, one terminal record); floor-plan growth is a sibling module, not a grammar. Navigability is a **hard gate** with repair → resample → loud reject, run on every generated interior, every time. Every clearance number — door width, corridor width, gate minimum — is **derived from one authored quantity, `agent_radius_m`**, and snapped to the 0.5 m lattice at raster time (§6b.1a).
8. The **Laboratory widget** is a seven-tab ImGui panel with a four-tier edit→regenerate loop: validate every keystroke, continuous re-interpretation ≤ 16 ms scoped to the focused specimen + 24 plants, debounced re-derivation at 180 ms, explicit build for anything expensive.
9. **All library tests live under `src/cvc/tests/`.** `CVC_BUILD_EXAMPLES` defaults `OFF` (`src/cvcGL/CMakeLists.txt:290`) and **no workflow YAML sets it** (`grep -rn CVC_BUILD_EXAMPLES .github/` → zero hits), so a test registered in `examples/` never runs in *PR* CI. Examples *are* compiled post-merge, for wasm only, by `deploy-pages.yml` (which calls `build-wasm-demo.sh`, and that script sets `-DCVC_BUILD_EXAMPLES=ON`) — but that build sets `-DCVC_BUILD_TESTS=OFF` and runs no `ctest`, so `nav_common_test` has still never executed anywhere.
10. Pre-existing lines edited across the whole plan: **four**, or **six** if the example is packaged and deployed (§14.2 re-derives this against the *current* files; the previous "four" was computed against pre-`#229` line numbers and omitted two list edits).
11. **PR 1 is visual.** `cvc-lsys` renders an interpreted specimen to a self-contained **SVG** (and an OFF mesh) with no GL, no VTK and no new dependency, so the first landable PR is something you open and look at — while still being a pure library PR that the 80 % coverage gate can measure (§13, §13.1).

**The pitch.** Build the rewriting engine, the world model and the LOD selection math as three real libcvc modules with no renderer dependency, so they can be tested against *published module counts from the botanical literature* rather than against themselves, and so GRL-SNAM gets a loadable, byte-verified training bundle three PRs in. Make the surface class map the authored truth and the contract rasters a derived projection whose **exported bytes have zero C++ coupling** to `cvc::nav::material` — `cvc::world` never links `cvc::nav`, the bundle is files. (The *Lab's preview overlays* are the one deliberate exception: now that `inc/cvc/nav/material.h` is merged and stable, the demo calls `material_build` / `witness_gate` **read-only** so that "what the consumer will see" is the consumer's actual arithmetic rather than a second, drifting copy of it. §14.4.) Accept that the archetype library *is* the design, and that the measured bottleneck is CPU animation, so the wind becomes a vertex-shader function of world position and the CPU cascade is capped at a constant 24 plants no matter how large the world grows. Unify plant and building grammars behind one parser and one terminal record, but leave floor-plan growth as a sibling, and treat navigability — **indoors and outdoors** — as a gate that fails loudly rather than a cost term that fails silently.

**Headline numbers.**

| | native `large` | native `default` | wasm |
|---|---|---|---|
| World extent | 8192 m | 4096 m | 2048 m |
| Islands | 4 | 3 | 1 |
| Max peak | 1420 m | 1420 m | 890 m |
| Cloud deck base / top | 700 / 1150 m | 700 / 1150 m | 620 / 950 m |
| Authored plant instances | 1 200 000 | 480 000 | 60 000 |
| Drawn instances/frame | ≈ 28 000 | ≈ 22 000 | ≈ 6 500 |
| Triangle budget | 2.5 M | 1.6 M | 700 k |
| VTK prop budget | ≤ 48 (assert < 63) | ≤ 48 | ≤ 24 |
| CPU-swayed plants | 24 | 24 | 8 |
| GPU ray-cast volumes | 2 | 2 | 1 |
| Target | ≥ 45 fps @ 1280×800, shadows on | ≥ 45 fps | ≥ 30 fps |
| Export window (default) | 513 × 513 @ 0.5 m = 256.0 m | same | same |
| Export lattice (indoor **and** outdoor) | 0.5 m/cell | same | same |
| Outdoor window policy (v1) | single-island | single-island | single-island |

---

## 1. Motivation & goals

### 1.1 Why

`lsystem_forest` proved that cvcGL can carry a procedural world: 32 trees, an analytic island, a volumetric sea and sky, and a cloud-shadow bake, at ~25 fps. It is also a dead end by construction — every parameter is a file-scope `constexpr`, the L-system is a hardcoded string table walked recursively with no intermediate word, there is no LOD, no material concept, no live editing, and the "route C" merge that saved its frame rate is precisely what makes per-object culling impossible.

Meanwhile GRL-SNAM has just acquired a **material-aware navigation** feature whose entire input is a pair of rasters on the occupancy grid. It needs worlds — many of them, reproducible, labelled, with varied terrain semantics and, now, **interiors**. Hand-authoring those is not a plan.

The Laboratory is the tool that closes that loop: an interactive procedural world designer whose output is a training corpus.

### 1.2 Goals

- **G1 — Reusable library.** A real libcvc module under `src/` + `inc/` with unit tests, consumable headlessly by GRL-SNAM world generation and later by volrover3 and pycvc. Subject to the 80 % line-coverage gate (`COVERAGE_MIN: '80'`, `.github/workflows/ci.yml:17`, enforced at `ci.yml:479-495` on the Linux Debug non-gRPC job).
- **G2 — Scale.** Multiple islands, mountains that pierce cloud, hundreds of thousands of generated plants/rocks/shrubs, at ≥ 45 fps natively.
- **G3 — Live editing.** In-app editing of the L-systems in play — trees, terrain, clouds, **and building grammars** — with a latency budget per class of edit.
- **G4 — Material marking.** Mark terrain regions as dirt / gravel / mud / grass / …, and interiors as concrete / tile / carpet / grating / …, through **one** registry, feeding the GRL-SNAM contract.
- **G5 — Indoor scenarios.** Generate buildings with floors, rooms, corridors, doorways and stairwells. **An unreachable room is a broken training environment**, so navigability is validated, not hoped for.
- **G5b — Outdoor solvability.** An archipelago's islands are, by construction, disconnected. **A disconnected episode is a broken training environment too**, so outdoor connectivity is gated with the same seriousness as indoor connectivity (§7.8), and the export window policy is an explicit, recorded decision rather than an accident of where the ROI rectangle landed.
- **G6 — Training data product.** Headless batch generation, deterministic seeding, a documented bundle format, and a curriculum knob.
- **G7 — Native-first, wasm-reduced.** Design the native ceiling honestly; ship a reduced wasm variant with a named knob table. Parity is not required.
- **G8 — No collision.** Land alongside PR #223 and PR #200 (the only open PRs as of the rebase) without a merge conflict, and on top of the now-merged `cvc::nav::material` (#230) and wasm (#229/#231) work.

### 1.3 Explicit non-goals

- **Not** a networked LOD streaming system. Roadmap §22.1's `pyramid_builder` / `lod_index` / `CvcLod` gRPC service / W-TinyLFU eviction is phases 32–37 of a *client-server* design. Our world is procedural and regenerable from a ~200-byte seed; there is nothing to stream and nothing to invalidate. We claim only `inc/cvc/lod/select.h`.
- **Not** a visibility subsystem. A separate concurrent design owns portals / PVS / BVH / occlusion. Our job is to **emit** the topology it needs (cells, portals, links) as first-class entities, in a form precise enough to consume without a conversation — §6b.6 is written as a **seam specification for that design**, including winding, units, the `opaque`-vs-`traversable` split and id stability. We ship only the trivial consumer (current cell + 2 portal hops) and we design none of the renderer side here.
- **Not** mesh decimation. LOD rungs come from re-deriving the grammar at a lower generation, which is coherent, memory-free and semantically meaningful. QEM is for *imported* meshes; we import none. Roadmap §26.4.2 can land `quadric_decimate` independently.
- **Not** GPU instancing. `vtkOpenGLGlyph3DHelper::GlyphRender` gates the instanced path on `GLAD_GL_ARB_instanced_arrays` — a desktop ARB extension string WebGL2/GLES3 never advertises — so it degrades to one draw call per glyph in the browser. A native-only second path is the one that rots.
- **Not** physics. *Arches*' authors state their rock piles are not physically stable; a visual fake is fine and physics is a different subsystem.
- **Not** real-world data import (GeoTIFF/OSM/LiDAR). The nav demos already have a bundle path for real Austin terrain; the Laboratory's job is *synthetic* worlds.
- **Not** RF propagation. The registry carries `penetration_db_per_m` / `reflection_loss_db` slots and exports them, zeroed, because one raster must be able to key two independent property tables. Computing propagation is out of scope.
- **Not** editing `src/cvcGL/examples/lsystem_forest.cpp` or `src/cvcGL/examples/README.md`. `lsystem_forest` stays as the fast smoke test and the performance control.

---

## 2. Relationship to the modernization roadmap

The roadmap of record is `/home/joe/src/cvc/cvc-engagement-docs/modernization/2026-08-11-volrover3-roadmap.md` (10 852 lines). It specifies **three independent LOD mechanisms**, and it is important not to conflate them.

| # | System | § | Phases | Selection axis | Our relationship |
|---|---|---|---|---|---|
| A | Client LOD + memory budget (`LodManager`) | §20.13 | 10.5 | memory/bandwidth budget per asset | **We implement the selection + budget math**, single-process, in `inc/cvc/lod/`. |
| B | On-demand LOD streaming pyramid | §22.1 | 32–37 | screen-space pixel error per tile | **Not implemented.** We adopt its *vocabulary* (`desired_pixel_error`, priority = projected area / distance²) so the future layer slots in. |
| C | Spatial-partition coarse summaries | §20.18.5/.8 | 39–44 | projected screen size of a partition leaf | **We adopt its anti-popping rule** (see below) at tile granularity. |

### 2.1 The rules we inherit and honour

**Per-leaf, not per-actor.** §20.18.8 (`:7164-7169`), verbatim:

> Each visible leaf either receives full-fidelity actor data from its owner or a coarse summary, per §20.18.5. The switch is decided **per-leaf per-frame based on screen size, *not* per-actor** — coarse summaries are coherent across the leaf and avoid the popping that per-actor LOD switches cause.

We take this at **scatter-cell granularity (32 m)** rather than tile granularity (128 m). A 128 m tile is too coarse a group to switch invisibly at close range; 32 m is coherent enough to avoid salt-and-pepper shimmer and small enough that a transition is one hedge's worth of geometry. This is a deliberate, stated refinement of the rule, not a violation of it.

**LOD may never alter simulation correctness.** §22 framing (`:7734-7742`): *"nothing in this section changes simulation correctness."* Enforced here by an absolute rule: **the material/nav export always runs at full fidelity, from the analytic surface function, independent of any render rung.** A world exported while the camera is at T4 is byte-identical to one exported at T0. There is a unit test.

**Time constants, never frame ratios.** §22.4.3 (`:8862-8868`): *"any smoothing constant must be expressed as a time constant (`1 - exp(-dt/tau)`), never as a per-frame ratio."* Every cross-fade, sway ramp and camera smoothing in this design is `1 - exp(-dt/tau)` off the world clock.

**Pick against the terrain, never the decimated actor.** §22.3.6 and the GRL-SNAM lab doc (`:716-719`). The material paint brush ray-marches against the **analytic** `heightfield::sample(x,y)`, never against a T3 chunk mesh whose cells are 16 m across. This is a correctness requirement, not a nicety: at T3 a stroke picked against the render mesh lands up to 8 m from the click.

**Ladder shape.** §20.13.4 (`:4998-5004`) gives `Geometry: full tris → 1/2 → 1/8 → 1/64 → stub`. Our vegetation ladder (§8) is a generation-truncation ladder with roughly those ratios, and we say so.

**Reserved namespaces.** `libcvc/inc/cvc/spatial/` is reserved by §20.18.10 for `world_tree.h`, `kd_tree.h`, `bsp.h`, `octree.h`, `portal.h`, `coarse_summary.h`. We do **not** touch it — our culling structure is a flat 32×32 uniform grid, which frustum-culls in ~5 µs at this N and would be *slower* as an octree.

### 2.2 A roadmap inconsistency we settle

§20.13.7 specifies `/clients/<self>/config/lod` in **camelCase** (`ramBudgetBytes`, `maxTrianglesVisible`); §22.1.6 specifies the **same path** in **snake_case** (`ram_budget_bytes`, `desired_pixel_error`). Everything else in cvcGL's state tree is snake_case (`viewers.main.camera.settings.move_speed`). **We adopt snake_case** and root our knobs at `lab.lod.*`. Recommendation to the roadmap owner: correct §20.13.7. Flagged in §15 as needing an ack.

### 2.3 Open questions from §28.1 that this design settles

- *"Should we ship named presets in addition to raw weight knobs?"* — **Yes.** `aggressive` / `balanced` / `pristine` populate `lab.lod.*`; advanced users override.
- *"meshoptimizer or libigl for decimation?"* — **Neither, for this workload.** Generation-truncation is strictly better for grammar-generated geometry. The question stays open for imported meshes.

---

## 3. Prior art & literature basis

### L-systems and plant modelling
- **[Prusinkiewicz & Lindenmayer 1990]** *The Algorithmic Beauty of Plants* (ABOP). Source of the turtle alphabet, the bracketed/parametric/stochastic/context-sensitive formalism, the polygon operator `{ . }`, the tropism model, and the specific figures we ship as recipes (Fig. 1.24, 1.25, 2.6, 2.7, 2.8, 5.5–5.12) with their published module counts. Note ABOP is **Y-up**; every tropism vector is converted to Z-up on load.
- **[Honda 1971]** Description of the form of trees by parameters of the tree-like body. Assumption 5 (branches lie in near-horizontal planes) is what the `$` / `@v` re-level operator implements; omitting it visibly breaks the monopodial models.
- **[Prusinkiewicz et al. 2003]** *Self-organising tree models for image synthesis*, SIGGRAPH 2003. Table 1's nine parameter rows (r1, r2, a1, a2, φ1, φ2, w0, q, e, min, n) are five of our six tree recipes. `e = 0.5` is da Vinci's law: `w_child = w·q^e` and `w·(1−q)^e` conserves cross-sectional area.
- **[Weber & Penn 1995]** *Creation and rendering of realistic trees*, SIGGRAPH '95. §5's **range degradation** is the core of our vegetation LOD: *"With progressively increasing ranges, a tree will re-interpret stem meshes as lines and leaf polygons as points… A 100 000 facet tree geometry may be rendered at 2 kilometres as about 30 lines and 1000 points."* Re-interpret, do not convert.
- **[Prusinkiewicz, James & Měch 1994]** *Synthetic topiary*, SIGGRAPH '94 (TOP94). Query modules `?P(x,y,z)`, the pruning predicate, and the α=90 / β=32 / γ=20 angle set.
- **[Měch & Prusinkiewicz 1996]** *Visual models of plants interacting with their environment* (ENV96). Open L-systems; cited as the production-grade version of our `?P`/`?H` sketch. Full bidirectional `?E` communication is out of scope.
- **[Prusinkiewicz, Karwowski & Lane 2007]** *The L+C plant modelling language*.

### Terrain, rocks, clouds
- **[Musgrave 1993/2003]** *Procedural Fractal Terrains*. Ridged multifractal, HeteroTerrain, HybridMultifractal. **Caveat carried forward:** the published `musgrave.c` `multifractal()` has a frequency-update bug — use RidgedMultifractal/HybridMultifractal.
- **[Quílez]** *Domain warping*. One level, distortion 0.30.
- **[Musgrave, Kolb & Mace 1989]** thermal/hydraulic erosion, as reimplemented and measured by **[Olsen 2004]** (`T = 4/N`, `c = 0.5`). Olsen's timings are 2004 Java on a P4; treat scaled figures as order-of-magnitude.
- **[Braun & Willett 2013]** O(n) implicit stream-power. **[Barnes, Lehman & Mulla 2014]** priority-flood depression filling. **[O'Callaghan & Mark 1984]** D8; **[Beven & Kirkby 1979]** TWI. **[Riihimäki et al. 2021]** measured D8's weakness as a moisture proxy (<50 % of FD8's explanatory power) — we use D8 for channels, FD8 for TWI.
- **[Prusinkiewicz & Hammel 1993]** *A Fractal Model of Mountains with Rivers*. Adopted for the **marking** grammar's context-sensitive subdivision + hash determinism idea; its three published open problems (constant-altitude river, asymmetric V-valleys, no tributaries) remain open, so drainage *structure* comes from stream-power/FD8 and the grammar only stamps what the flow field found.
- **[Peytavie et al. 2009]** *Arches*. Voronoi-fracture rock tiles, repose-angle stabilisation. Honest caveat from the authors: their piles are **not** physically stable.
- **[Dobashi et al. 2000]** *A simple, efficient method for realistic animation of clouds*. Cellular-automaton evolution at < 1 ms/step for 256×128×20.
- **[Harris & Lastra 2001]** *Real-Time Cloud Rendering*. Billboard/impostor clouds; the fallback for the deck.
- **[Schneider & Vos 2015]** *The Real-Time Volumetric Cloudscapes of Horizon Zero Dawn*. Cited as the technique we deliberately do **not** ship (2–5 ms discrete GPU, 10–20 ms iGPU, against a documented ~30 fps two-volume ceiling).

### Architecture and interiors
- **[Stiny & Gips 1972]**, **[Stiny 1980]** shape grammars.
- **[Wonka et al. 2003]** *Instant Architecture*, ACM TOG 22(3). Split grammars; the containment property. Verbatim on why: *"a significant problem with most design grammars that allow automatic derivation (most notably, L-systems) is that it is difficult to prevent objects from growing into each other… The split-grammar formalism presented in this paper has been created to deal with exactly this problem."*
- **[Müller et al. 2006]** *Procedural Modeling of Buildings*, ACM TOG 25(3). CGA shape; Algorithm 1 is our sequential-priority derivation mode; `[ ]` push/pop of a **scope** (frame + size vector), which is a strict superset of a turtle state.
- **[Lopes et al. 2010]** *A Constrained Growth Method for Procedural Floor Plan Generation*, GAME-ON 2010. **The spine of our interior generator.** < 100 ms simple, ~1 s complex.
- **[Merrell et al. 2010]** *Computer-Generated Residential Building Layouts*, SIGGRAPH Asia. Source of the stair-geometry dispatch and the upper-floor support constraint — and the cautionary datapoint that a *soft* accessibility term fails **1-in-20** (two-storey) and **1-in-5** (three-storey).
- **[Merrell 2021]** *Comparing Model Synthesis and Wave Function Collapse*: *"model synthesis can generate in a few seconds what WFC fails to generate in over 20 minutes."* Neither is on our critical path.
- **[Deitke et al. 2022]** *ProcTHOR*, NeurIPS (Outstanding Paper). 10 000 procedurally generated, **fully navigable** houses built on Lopes-style growth. The direct precedent that a hard navigability gate scales.
- **[Aichholzer et al. 1995]** straight skeleton (roofs). **[Fuchs, Kedem & Naylor 1980]** BSP. **[Mononen et al.]** Recast & Detour off-mesh connections. **[Jung et al. 2024]** MuNES multi-floor navigation.
- **[Alexander et al. 1977]** *A Pattern Language*; **[Hillier & Hanson 1989]** space syntax — for the door-near-corner and privacy-gradient heuristics.
- ICC *International Building Code* §1010.1.1 and stair provisions: 32 in clear door width, 44 in corridor, 11 in tread / 7 in riser.

### LOD and rendering
- **[Luebke et al. 2002]** *Level of Detail for 3D Graphics*.
- **[Garland & Heckbert 1997]** QEM; **[Hoppe 1996, 1997]** progressive meshes — cited as the approach we do not need.
- **[Losasso & Hoppe 2004]** geometry clipmaps; **[Strugar 2010]** CDLOD; **[Ulrich 2002]** chunked LOD. Our terrain is chunked-LOD with skirts, CDLOD morphing deferred to a later phase.
- **[Décoret et al. 2003]** billboard clouds; **[Wyman & McGuire 2017]** hashed alpha testing (the cross-fade mechanism); **SpeedTree SDK** LOD docs (shrink-and-grow on leaf cards).

### Editor UX
- **[Prusinkiewicz et al. 2004]** *Art and Science for Life*, Acta Hort. — L-studio's MDI-tab design, the gallery paradigm, Design/Execute panel modes, function-of-one-variable editors.
- **L-studio User's Guide** and **vlab environment docs** — explicit re-run vs opt-in *Continuous modeling*; prototype/extension variants as copy-on-write deltas; *Make extension* defaulting to **not** switching you to the child.
- **Houdini L-System SOP** — fractional generations with continuous interpolation; MMB to see the current string; the published warning that cost *"increases exponentially"*.
- **Blender Geometry Nodes Inspection** — socket inspection, node warnings, timings overlay, and the crucial discipline: *"Values are not logged during rendering, to improve performance"*; plus the **geometry randomization** audit mode.
- **[Eiserloh 2017]** *Math for Game Programmers: Noise-Based RNG*, GDC. Hash-not-generator; unordered access, reseeding, record/playback, lock-free parallelization.
- **[Compton & Mateas 2015]** *Casual Creators*, ICCC. "Chorus line", "mutant shopping", "no blank canvas", "only the perception of progress is necessary".
- **[Lai, Latham & Fol Leymarie 2020]** *Towards Friendly Mixed Initiative PCG: Three Pillars of Industry*, FDG. *"controllability is preferred than expressivity."*
- **[Victor 2012]** *Inventing on Principle* — immediate connection, resolved against exponential cost by giving *validation* immediacy always and *generation* immediacy when affordable.

---

## 4. World model

### 4.1 Coordinate frame and units

Z-up, right-handed, **metres**. Sea level is `z = 0`. The world is centred on the origin. Every length constant in the codebase is a metre unless suffixed `_px` or `_cells`.

This differs from `lsystem_forest`, whose "world units" were dimensionless and whose `HALF = 120.0` was simultaneously the terrain extent, the shadow stage radius and an implicit scale for `SUN_DEPTH_CAP = 150.0`. Every distance in `cvc::world` is a metre and every constant that bounds the world is a runtime parameter.

### 4.2 Presets

```cpp
// inc/cvc/world/preset.h
namespace cvc::world {
struct world_preset {
  const char* name;
  double  extent_m;          // full square side
  int     islands;
  double  tile_m;            // 128.0
  double  base_sample_m;     // terrain sample spacing at T0
  double  cloud_base_m, cloud_top_m;
  std::uint64_t plant_target; // authored instances
  int     max_props;          // VTK prop budget
  std::uint64_t max_tris;
  int     cpu_sway_budget;
  int     volume_budget;      // GPU ray-cast volumes

  static world_preset large();   // 8192 m, 4 islands, 1.2 M plants
  static world_preset standard();// 4096 m, 3 islands, 480 k
  static world_preset wasm();    // 2048 m, 1 island, 60 k
};
}
```

### 4.3 Layout — the `standard` preset

```
                      N (+y)
   -2048 m                                        +2048 m
      +---------------------------------------------+  +2048
      |                                             |
      |            .-~~-.                           |
      |          /  ANVIL \      <- peak  1420 m    |
      |         |  r=620m   |        pierces deck   |
      |          \.__  __./                         |
      |             ~~                              |
      |                          .-~~~-.            |
      |      .-~-.              / KESTREL\          |
      |     / TERN \           |  r=480m   |        |
      |    | r=390m |           \  980 m  /         |
      |     \  310 /             '-~~~-'            |
      |      '-~-'                                  |
      |                                             |
      |   ~~~~~~~~~~~ sea level z=0 ~~~~~~~~~~~~~   |
      +---------------------------------------------+  -2048
   -2048 m                                        +2048 m
                      S (-y)

   Vertical section through ANVIL:

   1420 m  ..............A..............   peak (rock, snow above 1180 m)
   1150 m  ---- cloud deck top ---------
    950 m       ,-~-, ,~-. ,-~-,           volumetric cumulus (VolumeNode)
    700 m  ---- cloud deck base --------
    520 m           /   \                  scree apron, bare rock
    240 m         /       \                shrub / conifer belt
     60 m       /           \              broadleaf / grass
      0 m  ~~~~/~~~~~~~~~~~~~\~~~~~~~~~~   sea
   -180 m     seabed
```

Island centres, radii and peaks are parameters in the `.lsys` header, not constants. `standard` ships:

| island | centre (m) | mask radius | peak | character |
|---|---|---|---|---|
| Anvil | (−520, +780) | 620 | 1420 m | high ridged massif, scree aprons, snow cap, dry lee flank |
| Kestrel | (+900, +140) | 480 | 980 m | rounded, forested, one river to a delta |
| Tern | (−980, −760) | 390 | 310 m | low, braided delta, mudflats, the research station |

`large` adds **Shoal** (islet, r = 160 m, peak 90 m) carrying a second building cluster and the outdoor/indoor seam demo.

### 4.3a The archipelago mechanism — seeding, combination, channels, biomes

> **Why this section exists.** Revision 1 promised a "multi-island archipelago" in §0 and then specified exactly one radial falloff with one centre and one radius. Multiple separate islands were an explicit requirement, and separate islands have consequences — an overlap operator, a sea-level/channel story, per-island character, and (above all) a **connectivity gate**, because two islands separated by deep water are an unsolvable episode. All of that is specified here.

#### 4.3a.1 The island record

```cpp
// inc/cvc/world/island.h
namespace cvc::world {

struct island_spec {
  char          name[24];
  double        cx, cy;            // centre, world metres
  double        r_mask;            // Wyvill falloff radius, m  (mask == 0 beyond)
  double        r_core;            // plateau radius where mask == 1 exactly; 0 => pure Wyvill
  double        peak_m;            // target summit height
  double        amp_m;             // ridged-multifractal amplitude for this island
  double        peak_sigma_frac;   // Gaussian sigma as a fraction of r_mask (default 0.28)
  double        freq_scale;        // per-island base-frequency multiplier (roughness character)
  std::uint32_t biome_id;          // index into the biome table (section 16.4)
  std::uint32_t grammar_set;       // which marking / vegetation / building grammars run here
  std::uint64_t island_seed;       // hash4(master, stream::terrain, island_index, 0)
};

struct archipelago_spec {
  std::vector<island_spec> islands;
  double        sea_level_m       = 0.0;    // by definition (section 4.1)
  double        shelf_m           = -9.0;
  double        smax_k_m          = 40.0;   // smooth-max blend width, metres
  double        separation_factor = 1.15;   // placement: min centre distance / (r_i + r_j)
  double        channel_min_m     = 40.0;   // min open water between non-overlapping islands
  double        bridge_min_m      = 12.0;   // min navigable width of a deliberate isthmus
  bool          force_bridges     = true;   // DECIDED: D9 option 2 (user, 2026-08-27)
};
}
```

#### 4.3a.2 Seeding and placement

Two modes, both deterministic.

**Authored (the default, and what the three shipped presets use).** The island table is written verbatim in the `.lsys` header. `standard` ships Anvil / Kestrel / Tern as tabulated in §4.3; `large` adds Shoal. This is the mode a scenario author uses, and it is the one every number in this document is computed against.

**Seeded (`--islands N --seed S`).** Placement is **Mitchell best-candidate with radius-aware spacing** — a Poisson-disk variant that tolerates unequal radii, evaluated through the hashed RNG so it is order-independent and reproducible:

```
for i in 0 .. N-1:
    r_i     = lerp(r_min, r_max, uni(master, terrain, island_id(i), 0))
    accepted = false
    for round in 0 .. 7:
        sep = separation_factor * pow(0.95, round)
        for attempt in 0 .. 63:
            c = uniform point in the inner square, side = extent - 2*(r_i + margin_m)
                (drawn as uni(master, terrain, island_id(i), 1 + 2*attempt + 64*round) x2)
            if for all accepted j:  |c - c_j| >= sep * (r_i + r_j):
                accept c; accepted = true; break
        if accepted: break
    if not accepted:
        HARD FAIL with the island index, the radii in play and the final `sep`.
        (A silent "N-1 islands" is exactly the kind of quiet degradation this
         design refuses everywhere else.)
```

`element` is `island_id(i) = morton(i, 0)`, **never** the loop counter reused across rounds, so adding an island does not reshuffle the ones already placed (§5.2's insertion-stability rule applies here too, and is tested).

`separation_factor` is the archipelago's shape knob:

| value | result |
|---|---|
| `≥ 1.0` | masks never overlap. Islands are genuinely separate; the sea between them is at `shelf_m`. **Default 1.15.** |
| `0.75 – 1.0` | masks overlap at their skirts. Smooth-max produces a **saddle**; whether that saddle breaks the surface is what §7.8 measures. |
| `< 0.75` | a single lobed landmass with bays. Legal, and how `warehouse_flats`-style single-island scenarios are authored. |

#### 4.3a.3 The combination operator — smooth-max, not sum

Revision 1's `h = Σ_i mask_i · (…)` is wrong for overlapping islands: summing two Wyvill lobes **double-counts** in the overlap and raises a ridge exactly on the join — the highest ground in the world ends up between the islands. A hard `max` is correct in amplitude but is C⁰-discontinuous along the equidistant locus, which reads as a crease and, worse, produces a zero-width slope discontinuity that the FD8 flow accumulation turns into a fake river.

We use the **polynomial smooth maximum**:

```cpp
// C1, exact-max outside the blend band, no transcendentals.
inline double smax(double a, double b, double k) noexcept {
  const double hgt = std::clamp(0.5 + 0.5 * (b - a) / k, 0.0, 1.0);
  return std::lerp(a, b, hgt) + k * hgt * (1.0 - hgt);
}
```

with `k = smax_k_m = 40.0` m. Properties that matter here, all unit-tested:

- **Exactness away from the seam.** When `|a − b| ≥ k`, `smax(a,b,k) == max(a,b)` *bit-for-bit*. So a well-separated archipelago (`separation_factor ≥ 1.15`) evaluates **identically** to the per-island formula, and none of §4.3's tabulated peaks move.
- **Bounded overshoot.** The blend adds at most `k/4 = 10 m` at the midpoint. A saddle can rise 10 m above the taller of the two contributions and no more, so a 40 m blend width can never manufacture a land bridge out of two islands whose skirts both sit at −20 m.
- **C¹.** No crease, so no phantom drainage channel.
- **Associative enough.** Folding left-to-right over the island list is order-dependent at the 4th decimal for three-way overlaps. We therefore **sort the island list by `(cx, cy, name)` once at load** and fold in that fixed order, so the result is a pure function of the set. Tested by shuffling the input table.

The revised height function:

```
h(x,y) = smax_fold_i( mask_i(x,y) * ( ridged_mf(warp(x,y), freq_scale_i) * amp_i
                                     + peak_gauss_i(x,y) ),  k = smax_k_m )
         + shelf(x,y)
         + delta.sample(x,y)
```

**Biome attribution is a partition, not a blend.** The island that owns a cell for *material and grammar* purposes is `argmax_i mask_i(x,y)` — the largest mask value, with ties (`|m_i − m_j| < 0.02`) broken by the sorted island index. Attribution is deliberately **not** smooth-maxed: a class raster must be crisp, because `risk_raw[i] == registry[klass[i]].rho` exactly is a load-bearing invariant (§7.3) and a blended biome would need a blended class, which is not representable in a `uint16` class map. Cells outside every mask are attributed to `void_unknown` / the shelf biome.

#### 4.3a.4 Sea level, channels and bridges

Sea level is `z = 0` by definition (§4.1). Everything else is derived and measured, not authored:

- A **channel** between islands `i` and `j` is the connected set of `h < 0` cells separating their land components. Its **width** is `2 · min over the channel's medial axis of the distance transform to land` — i.e. the narrowest open-water crossing, in metres. This falls straight out of the same EDT the nav gate already runs.
- A **land bridge** (isthmus) exists between `i` and `j` when their land components are 4-connected in the free set. Its **width** is the same statistic computed on land instead of water.
- `channel_min_m` (default **40 m**) is the minimum open-water width the generator accepts between two islands whose masks do not overlap. A narrower channel is visually indistinguishable from a bridge and behaves as neither; the generator widens it by depressing the saddle to `−1.5 m` over a 3-cell feather.
- `bridge_min_m` (default **12 m**) is the minimum navigable width of a *deliberate* bridge. A land bridge narrower than this, or narrower than `2·agent_radius_m + 1.0 m` (§6b.1a), is either widened or drowned, per the `force_bridges` policy.

The two knobs are enforced in one pass, immediately after the delta grid is applied and **before** any class rasterisation, so the class map and the connectivity gate never disagree about where the water is.

#### 4.3a.5 Per-island biomes and grammar divergence

A `biome` is the record that makes islands *different* rather than merely *separate*. Each island names one; the biome parameterises four things:

| what the biome overrides | mechanism |
|---|---|
| **Layer-0 classification predicates** (§7.4) | the biome supplies a predicate table that *replaces* the default; e.g. the braided-delta biome lowers the TWI mud threshold from 7.5 to 5.5 and raises the sand band from 3.0 m to 6.0 m |
| **Species mix** (§16.3 vegetation scatter) | per-biome `density_per_ha`, `altitude_band_m` and `slope_max_deg` overrides, merged over the global table |
| **Marking grammar set** (§6.6) | which of `river_network` / `trail_network` / `mudflat_region` run, and with what seeds |
| **Building grammar set** (§6.7) | which shells are eligible and how many settlements are placed |

Four biomes ship (full table in **§16.4**): `alpine_massif` (Anvil), `forested_rounded` (Kestrel), `braided_delta` (Tern), `barren_islet` (Shoal). The biome id is part of the island record, so it is part of the `.lsys` header, so it is part of the world hash — a biome edit is a Tier-2 change and is recorded in provenance.

**Why this is worth the complexity:** three islands with one biome produce three worlds that are statistically the same world, and a training corpus built from them has one material distribution wearing three hats. Per-island biomes are what make the class-fraction histogram in §9.4 vary between bundles, which is the entire point of a corpus.

### 4.4 Terrain

Terrain is **analytic and resolution-independent**: a pure function of world (x, y) plus a stored delta grid for erosion and authored edits.

```
h(x,y) = smax_fold_i( mask_i(x,y) * ( ridged_mf(warp(x,y), freq_scale_i) * amp_i
                                     + peak_gauss_i(x,y) ),   k = smax_k_m )
         + shelf(x,y)
         + delta.sample(x,y)          // erosion + authored, bilinear
```

- `smax_fold_i`: the C¹ polynomial smooth maximum of §4.3a.3, folded in the sorted island order, `k = 40 m`. **Not a sum** — summing double-counts in overlaps and raises the world's highest ridge exactly on the join between two islands.
- `ridged_mf`: Musgrave RidgedMultifractal, `H = 1.0, offset = 1.0, gain = 2.0, lacunarity = 2.0`, 9 octaves, base frequency 1/1400 m × the island's `freq_scale`, amplitude scaled per island.
- `warp`: one level of Quílez domain warp, distortion 0.30, frequency 1/2600 m.
- `mask_i`: Wyvill compact falloff `(1 − d²/r²)³`, clamped, with an optional `r_core` plateau where it is exactly 1.
- `peak_gauss_i`: a Gaussian bump reaching the island's stated peak height, σ = `peak_sigma_frac`·r (default 0.28).
- `shelf`: a global −9 m offset outside all masks, giving a continental shelf that keeps the sea volume shallow.

**Octave band-limiting is the terrain LOD.** Each chunk level `L` evaluates only octaves whose frequency is below the Nyquist of that level's sample spacing:

```cpp
int octaves_for(double sample_m) {
  // stop when wavelength < 2 * sample spacing
  return std::clamp(int(std::log2( (1.0/base_freq_m) / (2.0*sample_m) )), 1, 9);
}
```

This is the fix for the predecessor's most fundamental scale bug: `lsystem_forest`'s relief frequencies (`0.045, 0.041, 0.11, 0.097`) are absolute while its dome scales with `HALF`, so the two halves of the formula scale differently and the relief aliases catastrophically as the world grows. Band-limiting makes coarse chunks *provably* alias-free, and it is unit-tested (a coarse chunk's height spectrum has no energy above its Nyquist).

**The delta grid** is a single 2048×2048 f32 raster over the whole world (16 MB) holding erosion output + authored terrain edits. Erosion is a **Tier-3 explicit** operation scoped to a user-selected region, never the whole world:

1. Priority-flood depression fill [Barnes 2014] — 30–80 ms @ 1024².
2. D8 + FD8 flow accumulation — 20–50 ms.
3. Implicit stream-power, ~30 steps, `m = 0.45, n = 1` [Braun & Willett 2013] — ~0.1 s.
4. Thermal slumping, 60 iterations, `T = 4/N, c = 0.5` [Olsen 2004] — ~0.1 s.

The FD8 accumulation field (the *hydrology grid*, 8 m cells) is retained: it drives river-marking grammar tropism and the TWI-based `mud` classification predicate. **It is the one field that is not resolution-independent** — a 0.5 m export window bilinearly resamples an 8 m field. This is stated as an invariant and tested. It is acceptable because rivers *also* lay down explicit paint at their true world width, which *is* resolution-independent; the hydrology field only steers the walk.

### 4.5 Tiling

The world is a **flat 32×32 (large: 64×64) uniform grid of 128 m tiles**, addressed by Morton code. Deliberately not a tree: frustum culling is a 2D rect test of the frustum's ground footprint against tile rects, plus a per-tile `[z_min, z_max]` slab test — 1024 tiles at ~5 ns each is **5 µs/frame**. An octree is slower at this N and would squat `cvc/spatial/`.

Each tile owns: its chunk mesh at the current level, its scatter cells (16 of them, 32 m each), its instance ranges per archetype, and cached `z_min`/`z_max`.

### 4.6 Sea

- **Near field** (within 600 m of the camera): one `VolumeNode`, 72×72×20 grid over a camera-following slab from −40 m to +8 m. Wave field is four crested travelling waves, evaluated **once per column** (the predecessor recomputes the z-independent `seaSurface` inside the k loop, an 18× waste on its dominant cost).
- **Far field**: a single flat mesh at z = 0 with a fresnel-ish shader, out to the world edge. Coastline foam is a texture band driven by `|h|` on the terrain chunk, not a separate mesh.

### 4.7 Clouds

The predecessor's sky is a fixed 60×60×28 grid stretched over a fixed `SKY_HALF = 150`. At 100× extent its drift silently freezes (`shift` is in grid cells) and the whole sky tiles with period `2·SKY_HALF`. Both are fixed here:

- **Density is a hash-tiled world-space function**, not a bounded grid, so there is no visible period.
- **Drift is metres per second**, converted to cells at sample time.
- Cloud L-system splats are restricted to the Gaussian's **3σ bounding box** (the predecessor splats over all 100 800 voxels per `F` regardless of `puff` — a ~40× build-time waste).
- A **Dobashi cellular automaton** (128×128×40, bit-packed, < 1 ms/step) evolves coverage; the L-system supplies the initial structure and the anvil bias. Orographic coupling raises coverage over windward flanks, which is what visually sells "the peak pierces the deck".
- **Rendering:** one `VolumeNode` for the near deck (within ~2500 m), plus **8–12 depth-tested translucent shells** at 45 m spacing for the far deck [Harris & Lastra 2001]. The shells are the fallback if the single volume misses budget; they trivially depth-test against mountains at < 0.3 ms.

Peak-piercing correctness relies on the volumetric pass running after the opaque pass and `vtkGPUVolumeRayCastMapper` terminating against the opaque depth buffer. This chain exists (`SceneGraph.cpp:1044-1074`) but has never been exercised with 500 m of mountain inside a cloud slab, so it is a **named risk with a screenshot test** (§15 R2).

### 4.8 Sun

`SUN_DEPTH_CAP = 150.0` in the predecessor is an absolute world coordinate; once the eye is > 150 units along the sun direction the sun disappears. Replaced by a **camera-relative** billboard placement with no absolute cap, tested by sampling the sun's screen position from 20 camera poses across an 8 km world.

---

## 5. The L-system core — `cvc::lsys`

### 5.1 One engine, cut at the derivation-mode / opcode-family seam

**Position: one engine.** The shared core is large and identical — parser, symbol table, parametric modules, guard expressions, stochastic alternatives with per-derivation coherence, priority groups, the transform stack, seeded RNG discipline, the step budget, and the derivation-DAG recorder. The divergences are three small, orthogonal escape hatches:

1. **Derivation order** — an enum. Parallel (every module rewritten simultaneously for exactly `n` generations; parallelism is *semantic*, it models simultaneous growth) vs sequential-priority (pop a nonterminal, fire one applicable rule, repeat until none remain — [Müller 2006] Algorithm 1). ~40 lines of divergence over one `match_and_fire()`.
2. **Containment** — a policy object. `null_policy` for plants (branches may interpenetrate); `strict_policy` for buildings (Wonka Def. 4.6: split children fill exactly the parent volume; conversion children are contained in it). One virtual call.
3. **Context** — an interface. String-neighbour context (`A < B > C`, with bracket skipping and `#ignore`) vs spatial context (BVH occlusion queries, snap lines). Two implementations of a two-method interface.

The transform stack correspondence is not an analogy: `[`/`]` push/pop a **scope** (position + orthonormal frame + size vector), and a turtle state is a scope where `s_y` = step length and `s_x = s_z` = branch radius. Implementing the scope gives free branch-radius tapering out of `S()`.

**The decisive argument is authoring, not code reuse.** If they are siblings, they get two rule languages and a scenario author must learn both. Sharing a core means `archipelago.lsys` and `warehouse.lsys` are the same file format with a different `mode:` header, the same `#seed`, the same guard syntax, and the same debugger. That is the difference between a laboratory and two tools in a trenchcoat.

**Where we refuse to unify:** Lopes constrained growth is **not** a rewriting system and stays a sibling module (`cvc::world::floorplan`). It is a grid growth loop with a constraint solver. It consumes a storey polygon *from* the grammar and returns rooms + portals. Forcing it into grammar rules is the abstraction that would actually cost us.

**Kill trigger, stated up front:** if `derivation_mode`, `containment_policy` or `context_provider` grows past ~150 lines each, the unification was wrong and the honest move is to fork — preserving the parser, the RNG discipline and the `terminal` record, which are the parts worth keeping regardless.

### 5.2 Determinism — hashed RNG, not a stream

This is the single most important architectural decision in the module, and it is a direct repair of the predecessor.

`lsystem_forest.cpp:1101-1121` walks **one** `std::mt19937(20260817u)` sequentially for x, y, size, maturity, phase and sway inside the placement loop, and mixes raw `rng()` bit draws with `uniform_real_distribution`. Consequence: adding one tree, changing `MAX_TREES`, or hitting the `if (h < SEA_LEVEL + 1.5) continue;` skip **reshuffles every subsequent draw**. The world is a function of iteration order.

Replaced wholesale with [Eiserloh 2017]-style noise-based RNG:

```cpp
// inc/cvc/lsys/rng.h  — pure, integer-only, header-only, trivially testable.
namespace cvc::lsys {

enum class stream : std::uint32_t {
  placement = 0, species, maturity, size, phase, sway, rule_choice,
  param_jitter, surface, terrain, hydrology, building, floorplan, props,
  cloud, rock, _count
};

// 4-round mix; no state, no order dependency.
constexpr std::uint64_t hash4(std::uint64_t master, std::uint32_t strm,
                              std::uint64_t element, std::uint32_t draw) noexcept;

constexpr double  uni  (std::uint64_t master, stream s, std::uint64_t el, std::uint32_t d) noexcept; // [0,1)
constexpr double  uni  (std::uint64_t, stream, std::uint64_t, std::uint32_t, double lo, double hi) noexcept;
constexpr double  nrand(std::uint64_t, stream, std::uint64_t, std::uint32_t, double mu, double sigma) noexcept;
constexpr int     irand(std::uint64_t, stream, std::uint64_t, std::uint32_t, int lo, int hi) noexcept;

// Stable element ids. NEVER a loop counter.
constexpr std::uint64_t cell_id(std::int32_t gx, std::int32_t gy) noexcept;      // morton
constexpr std::uint64_t path_id(std::uint64_t parent, std::uint32_t child) noexcept; // derivation path
}
```

Rules, enforced by tests:

- `element` is a **stable identity**: for placement, the packed scatter-cell coordinate; for a specimen, the id derived from its cell; for a module, `path_id(parent_id, child_index)` — the derivation path, **never** the flat index into the output vector.
- `stream` is a named enum. Independent streams mean the sway seed provably cannot move a tree.
- `draw` is a small per-call ordinal so one element draws many independent values without an order dependency.
- Every random draw in `cvc::lsys` and `cvc::world` goes through this. There is no `std::mt19937` anywhere in either module. A grep test enforces it.

**The salt audit** (`--debug-salt=<stream-mask>`) perturbs only the `element` derivation for the named streams. It is both a debug mode and a **CI invariant**:

```
TEST(lsys_determinism, cosmetic_streams_do_not_touch_the_contract) {
  // Salting sway, phase, size, param_jitter, cloud must leave
  // class_map / risk_raw / hard / occupancy BIT-IDENTICAL.
}
```

This is the canary that keeps the discipline honest as the code grows, and it protects the artifact with the longest half-life — the training data.

### 5.3 Alphabet

Full table in §16.1. Summary:

| family | symbols |
|---|---|
| motion | `F(l)` `f(l)` `G(l)` `@Gs @Gc @Ge` |
| rotation | `+(a) -(a) &(a) ^(a) \(a) /(a) \|` `$` / `@v` (re-level) `@R(v)` |
| state | `!(w)` `;(mat)` `'(c)` `@Ts @Ti @Tf` |
| structure | `[` `]` `%` (cut) `~S(id,s)` |
| polygon | `{` `.` `}` `@#(contour)` `@!(sides)` |
| scope (buildings/rocks) | `T(x,y,z)` `S(x,y,z)` `R(ax,ay,az)` `Subdiv(axis,…)` `Repeat(axis,d)` `Comp(sel)` `I(asset)` |
| query | `?P(x,y,z)` `?H(hx,hy,hz)` `?S(h,slope,flow)` |
| **paint** | `P(mat,r)` `Pw(mat,w)` `Pb(mat,w,f)` `Stamp2D(mat,rect)` |
| portal | `Portal(kind,w,h)` `Link(kind,cost_up,cost_down)` |

The **paint** and **portal** families are the additions that make this a world generator rather than a plant generator, and they are what let one grammar produce both geometry and the nav/material rasters in one walk.

**Angles are degrees in the source and converted exactly once at parse.** The predecessor's `mRot(ang,…)` takes radians while `YROTATE = 10.0` and `TILT = 120.0` read as degrees, so `R` actually rolls 10 rad ≈ 213° and `T` tilts 120 rad ≈ 35.5°, and the current look is an accident of `mod 2π`. We do not inherit that landmine.

### 5.4 Type sketches

```cpp
// inc/cvc/lsys/module.h
namespace cvc::lsys {

using symbol_t = std::uint16_t;                  // interned
inline constexpr int max_params = 6;

struct module_t {
  symbol_t sym;
  std::uint8_t nparams;
  std::uint8_t level;        // derivation depth at which this module appeared
  std::uint32_t _pad;
  double p[max_params];
  std::uint64_t path;        // path_id chain — the stable element id
};

// A derived word. Flat, contiguous, no per-module allocation.
class word {
public:
  std::size_t size() const noexcept;
  const module_t& operator[](std::size_t) const noexcept;
  // Modules with level <= k, in order. This IS the LOD rung, when nested.
  void filter_level(int k, std::vector<module_t>& out) const;
  std::uint64_t content_hash() const noexcept;
  const std::array<std::uint32_t, 32>& level_counts() const noexcept;
private:
  std::vector<module_t> m_;
};
}
```

```cpp
// inc/cvc/lsys/grammar.h
namespace cvc::lsys {

enum class derivation_mode : std::uint8_t { parallel, sequential_priority };
enum class containment     : std::uint8_t { none, strict };
enum class asset_kind      : std::uint8_t { plant, terrain, cloud, rock, building };

struct production {
  symbol_t         pred;
  std::vector<symbol_t> left_ctx, right_ctx;   // empty == context-free
  expr             guard;                      // empty == always
  expr             probability;                // empty == 1.0
  std::uint8_t     priority = 0;
  std::vector<module_expr> successor;
  bool             deletes = false;            // successor is empty, or contains '%'
};

struct ruleset {
  std::string      name, cite, parent;         // parent == variant lineage
  asset_kind       kind;
  derivation_mode  mode;
  containment      contain;
  std::vector<symbol_t> ignore;                // #ignore
  std::vector<module_expr> axiom;
  std::vector<production>  prods;
  param_table      params;                     // named, ranged, unit-tagged
  curve_table      curves;                     // f(x) -> y, bound to a param
  int              preview_gen = 6, build_gen = 10;
  std::array<int,5> lod_gens{};                // per-rung generation counts
  std::uint64_t    grammar_hash = 0;           // sha256 of the verbatim block

  // Set by derive(). FALSE when any production deletes, uses '%', or is
  // context-sensitive -- because then depth n-1 is NOT a subword of depth n
  // and the filter_level() LOD trick is INVALID for this ruleset.
  bool             gen_nested = true;
};
}
```

> **This `gen_nested` flag is load-bearing.** The "derive once, filter by level" LOD story is only sound for monotone grammars. Context-sensitive productions and the cut symbol `%` can *delete* modules, so depth `n−1` is not always a subword of depth `n`. Several shipped recipes (the TOP94 hedge, the marking grammars with `?S` queries, anything using `%` to shed failed tendrils) are exactly those cases. `derive()` **detects** this at derivation time and sets `gen_nested = false`; those archetypes fall back to N independent cached derivations, one per rung. Cost is 5× derivation for that archetype only, which is fine because derivation is cached and off the hot path. The Lab shows a badge. This is machine-checked, not documented.

```cpp
// inc/cvc/lsys/derive.h
namespace cvc::lsys {

struct derive_options {
  std::uint64_t master_seed = 0;
  int   generations   = 6;
  double fractional   = 0.0;   // in [0,1); scales the final cohort (Houdini-style)
  std::size_t max_modules = 200'000;   // HARD budget. Truncation is REPORTED.
  std::uint32_t max_steps = 4'000'000;
  const context_provider* ctx = nullptr;
};

struct derive_result {
  word w;
  bool truncated = false;
  std::size_t modules_dropped = 0;
  std::uint32_t generations_reached = 0;
  double ms = 0.0;
};

derive_result derive(const ruleset&, const derive_options&);

// Resumable form for the wasm render loop and for cancellation.
class deriver {
public:
  deriver(const ruleset&, const derive_options&);
  bool step(double budget_ms);   // false == done
  void cancel() noexcept;
  const derive_result& result() const noexcept;
};
}
```

The `max_modules` budget is **surfaced in the UI**, and the estimated module count at generations 1..12 is plotted next to the generations slider *before* you drag it. `ImGuiSliderFlags_AlwaysClamp` is applied to cost knobs (generations, sides-per-order, plant density, tile size) and deliberately **not** to aesthetic knobs (letting a user Ctrl+click 3× the slider max is how they find the range you should have shipped).

### 5.5 Interpretation is separate from derivation

The predecessor's `expandTree()` fuses rewriting and interpretation into one recursive walk, which means *every* parameter change is a full re-derivation. Separating them is what makes the Laboratory's continuous tier exist:

```cpp
// inc/cvc/lsys/interp.h
namespace cvc::lsys {

struct terminal {
  enum kind_t : std::uint8_t {
    cylinder, cone, card, polygon, mesh_ref, splat, paint, portal, link, cell
  } kind;
  std::uint8_t  level;          // derivation level -> the LOD rung filter
  std::uint16_t surface_class;  // index into cvc::world::surface_registry
  std::uint8_t  nav_class;      // free | rough | blocked_wall | blocked_fall | door | portal
  double xf[16];                // row-major, world
  double a, b, c;               // kind-dependent: radius/len/sides, w/h, etc.
  std::uint64_t path;
};

// The ONLY output interface. Three implementations; the engine knows none of them.
class emitter {
public:
  virtual ~emitter() = default;
  virtual void begin(const ruleset&) {}
  virtual void on_terminal(const terminal&) = 0;
  virtual void end() {}
};

struct interp_options {
  double xf0[16];
  int    max_level = 32;        // the LOD rung
  int    sides_by_order[6] = {10, 8, 6, 4, 0, 0};
  const  height_probe* ground = nullptr;   // for ?S queries
};

void interpret(const word&, const ruleset&, const interp_options&, emitter&);
}
```

Three emitters ship: `stats_emitter` (counts, bbox, level histogram — **no geometry**, which is what makes 92 % coverage of the derivation and interpretation paths cheap), `raster_emitter` (paint/portal/cell terminals into `cvc::world` rasters), `mesh_emitter` (into `cvc::geometry`). `emitter` is a plain abstract class with no templates and no `std::function` — a SWIG director candidate from day one.

**API rule, adopted at commit one and enforced by review:** no templates in public signatures, no `boost::` in public signatures, no `std::function` parameters, every array crossing a module boundary is `(ptr, rows, cols)`. This costs nothing now and is the only path to in-process world generation from a training loop later.

### 5.6 Serialization — the `.lsys` format

Text, line-oriented, diffable, hand-editable, with the grammar kept **verbatim**. L-studio's directory-of-plain-text-files format is 30 years old and still the right answer; Gaea gating its XML behind Enterprise is the cautionary tale.

The file has a machine-owned structured header and a user-owned grammar block below `--- grammar`. The writer round-trips the header and **never re-serialises, re-indents or escapes the grammar bytes**, so comments and formatting survive save/load.

```
# cvc lsystem v1
name:   spiral_broadleaf
cite:   "Prusinkiewicz et al., SIGGRAPH 2003, Table 1 row (g)"
kind:   plant
mode:   parallel
contain: none
parent: ""                       # variant lineage (vlab prototype/extension)
master_seed: 20260827
streams:
  placement: 0
  species:   1
  maturity:  2
  size:      3
  phase:     4
  sway:      5
  rule_choice: 6
  param_jitter: 7
pinned: [ placement, sway ]      # padlocked against "Randomize unlocked"
params:
  - { id: r1,   label: "Contraction 1",  value: 0.80, min: 0.30, max: 0.99 }
  - { id: r2,   label: "Contraction 2",  value: 0.80, min: 0.30, max: 0.99 }
  - { id: a1,   label: "Branch angle 1", value: 30.0, min: 0.0,  max: 70.0, unit: deg }
  - { id: a2,   label: "Branch angle 2", value: -30.0, min: -70.0, max: 70.0, unit: deg }
  - { id: phi1, label: "Divergence 1",   value: 137.5, min: 0.0, max: 180.0, unit: deg }
  - { id: phi2, label: "Divergence 2",   value: 137.5, min: 0.0, max: 180.0, unit: deg }
  - { id: w0,   label: "Base width",     value: 0.30, min: 0.02, max: 1.20, unit: m }
  - { id: q,    label: "Width split",    value: 0.50, min: 0.05, max: 0.95 }
  - { id: e,    label: "Width exponent", value: 0.50, min: 0.00, max: 1.00 }
  - { id: wmin, label: "Min width",      value: 0.012, min: 0.002, max: 0.20, unit: m }
curves:
  - { id: taper, bind: seg_radius, pts: [[0,1.0],[0.5,0.62],[1.0,0.06]] }
tropism: { T: [0,0,-1], A: 0.0, E: 0.18, S: 0.02 }
sides_by_order: [10, 8, 6, 4, 0, 0]
preview_gen: 6
build_gen:   10
lod_gens:    [10, 8, 6, 0, 0]      # A0 A1 A2 A3(card) A4(none)
--- grammar
#ignore: + - / \ ! ;

w  : !(w0) F(1.0) A(1.0, w0)

// da Vinci's law: e = 0.5 conserves cross-sectional area across the split.
p1 : A(l,w) : w > wmin ->
       !(w) F(l)
       [ /(phi1) &(a1) A(l*r1, w*pow(q,     e)) ]
       [ /(phi2) &(a2) A(l*r2, w*pow(1.0-q, e)) ]

p2 : A(l,w) : w <= wmin -> [ @v &(35) ~S(leafcard, l*0.7) ]
```

A **variant** stores `parent:` plus only the changed keys, and (if changed) a whole replacement grammar block. Preset galleries become a directory tree and "what did I change?" is a one-line diff. This is vlab's prototype/extension mechanism, and it is simultaneously the A/B feature, the preset system and the versioning story.

`cvc::state` is a natural in-memory backing (and `state_change_journal.h` gives an undo log for free), but **the on-disk format is not a state dump**.

---

## 6. Asset recipe library

Shipped as `.lsys` files, installed to `${CMAKE_INSTALL_PREFIX}/share/cvc/lsys/` and *also* embedded as a string table in `inc/cvc/lsys/recipes.h` so a wasm build with no filesystem still has them. Every recipe carries a `cite:` line and is unit-tested for module count and determinism.

### 6.1 Trees (6)

Five share the SIG03 self-similar production shown in §5.6, differing only in the parameter row:

| id | name | r1 | r2 | a1 | a2 | φ1 | φ2 | w0 | q | e | wmin | n | character | cite |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| T1 | `spiral_broadleaf` | .80 | .80 | 30 | −30 | 137 | 137 | 30 | .50 | .50 | 0.0 | 10 | golden-angle spiral; the generic tree | [Prusinkiewicz 2003] Table 1 **g** |
| T2 | `monopodial_whip` | .92 | .37 | 0 | 60 | 180 | 0 | 2 | .50 | .00 | 0.5 | 15 | whip-like, single leader | ibid. **f** |
| T3 | `spire` | .95 | .75 | 5 | −30 | −90 | 90 | 40 | .60 | .45 | 25.0 | 12 | tall sparse spire; terminates on thinness | ibid. **h** |
| T4 | `weeping` | .55 | .95 | −5 | 30 | 137 | 137 | 5 | .40 | .00 | 5.0 | 12 | inverted, drooping | ibid. **i** |
| T5 | `understory_planar` | .50 | .85 | 25 | −15 | 180 | 0 | 20 | .45 | .50 | 0.5 | 9 | planar/alternating; espalier, understory | ibid. **c** |

| T6 | `poplar_columnar` | ABOP Fig. 2.6 with `a0 = 20, d = 77, r1 = 0.97` plus `tropism: {T:[0,0,1], A:0, E:0.45}`. Shallow departure angle then pulled vertical — that combination *is* the columnar trick. `n = 10` ⇒ **1023 `F`** ⇒ ~12.3 k tris at 12 sides. | [ABOP Fig. 2.6]; arbaro `lombardy_poplar.xml` |

The conifer workhorse, kept separate because it is the highest-count asset:

| T0 | `pine_monopodial` | ABOP Fig. 2.6 verbatim (Honda monopodial), `r1=0.9, r2=0.6, a0=45, a2=45, d=137.5, wr=0.707`, `n=10`. `$` (re-level) is **assumption 5 of [Honda 1971]** and is not optional. | [ABOP Fig. 2.6] |

### 6.2 Shrubs (5)

| id | name | grammar | params | cite |
|---|---|---|---|---|
| S1 | `bush_ternary` | ABOP Fig. 1.25 verbatim, ternary, with the 6-vertex polygon leaf `L → ['''^^{-f+f+f-\|-f+f+f}]` (4 tris after fan) | `d = 22.5`; ship at `n = 5` (243 apices) for dressing, `n = 7` (2187 apices, 3891 `F`, 2607 `L`) hero only | [ABOP Fig. 1.25] |
| S2 | `leeuwenberg_sumac` | `A → O[A][A]K` fleshed as `A(l,w) : l>0.06 → !(w) F(l) /(α) [+(β+nran) A(l·r, w·wr)] [−(γ+nran) A(l·r, w·wr)] ~Flower(0.6l)` | `α=90 β=32 γ=20 r=0.80 wr=0.707 n=9` | [Barthélémy & Caraglio] Fig. 16; angles [TOP94] |
| S3 | `champagnat_cane` | `A → O1[A]B`, `B → O2[B]B`, `O2 → P` | `tropism: {T:[0,0,-1], E:0.05, S:0.09}` — the arch *is* the model | ibid. Fig. 25 |
| S4 | `desert_scrub` | high tilt jitter (σ = 45°, the arbaro `CurveV 360/480` equivalent), strong upward twig attraction, low wide prune envelope | `AttractionUp 2.0` | arbaro `desert_bush.xml` |
| S5 | `hedge_clipped` | TOP94 L-system 6 verbatim, `prune(x,y,z)` bound to a CSG box | `α=90 β=32 γ=20`, clip cube edge = 12 internodes, `a=0 b=1 c=−5`, derivation 21. **`gen_nested = false`** (uses `%`). | [TOP94] §6 |

### 6.3 Ground cover and ferns (4)

| id | name | recipe | cost | cite |
|---|---|---|---|---|
| G1 | `tuft_meadow` | distichous 3-pass tuft, `NB=11, dv=137.5, aB=14°, NS=14, BL=0.09 m, TW=0.010 m`; `tropism {T:[0,0,-1], E:0.16, S:0.05}` | ~154 quads = **308 tris/tuft** | derived: [ABOP §4] divergence + §2 tropism; arbaro `shave-grass.xml` |
| G2 | `sedge_upright` | same with `aB=8, E=0.10, NS=18` | ~360 tris | ibid. |
| F1 | `frond_pinnate` | ABOP Fig. 5.11: `A(d):d>0→A(d−1)`, `A(0)→F(1)[+A(D)][−A(D)]F(1)A(0)`, `F(a)→F(a·R)`; **D=1, R=1.50, n=16** | ~140 tris | [ABOP] Table 5.2 case **b** |
| F2 | `frond_alternate` | ABOP Fig. 5.12 A/B alternating; **D=1, R=1.36, n=20** | ~180 tris | [ABOP] Table 5.3 case **a** |

ABOP's own warning ships in the tooltip: *"The model is sensitive to growth rate values — a change of 0.01 visibly alters proportions."* `R` gets a `%.3f` drag widget over `[1.10, 2.10]`, not a slider.

### 6.4 Rocks (3) — the proof of the one-engine decision

Rocks are **scope grammars** (`mode: sequential_priority`, `contain: strict`) running on the same parser, the same seeds and the same terminal record as the plants. That is the strongest practical argument for §5.1.

| id | name | sketch | cost | cite |
|---|---|---|---|---|
| K1 | `boulder_fracture` | `I("icosphere4") B(0)`; `B(k):k<3 → Subdiv(rand_axis,u,1−u){B(k+1) B(k+1)}` (p=.62) \| `Comp(faces){Chip(k)}` (.24) \| `I(hull)` (.14); leaves are noise-displaced convex hulls, 3-octave ridged | R0 ≈ 1280 tris, R1 ≈ 380, R2 ≈ 90 | [Peytavie 2009]; Musgrave displacement |
| K2 | `scree_field` | Poisson scatter of K1 at depth 1, density `f(slope, curvature, cliff_proximity)`, repose-angle stabilisation as a local brush pass; each leaf emits `P(scree, r)` | merged per tile | [Peytavie 2009] |
| K3 | `hoodoo` | vertical `Repeat(z,h)` of alternating wide/narrow scope slabs, capped | bake-only, native | implicit-3D-features literature, expressed as a scope grammar |

### 6.5 Clouds (2)

| id | name | grammar |
|---|---|---|
| C1 | `cumulus_anvil` | **The existing `lsystem_forest` cloud grammar, ported byte-identically** as a regression baseline: axiom `[A][+++++A][-----A][++++++++++A][----------A][+++++++++++++++A]`; `A→FF[+<B]^F[-<C]<F[+<C]vFA`; `B→F[+<F]F<[-<F]vB`; `C→^<F[+<F][-<F]^<FC`; turn 32°, depth 6, step0 8.1, step decay 0.9, puff0 8.8, puff decay 0.88. If the new engine reproduces this density field within float tolerance, the port is proven. **Note:** the climb symbols are `^`/`v`, not `^`/`&` — the predecessor's README drifted. |
| C2 | `stratocumulus_deck` | low turn (11°), high branching, shallow climb — a sheet rather than a tower. The deck the peaks pierce. |

### 6.6 Terrain and surface-marking (4)

| id | name | kind | recipe |
|---|---|---|---|
| TR1 | `archipelago` | composition, not a grammar | The §4.4 stack. Header carries the island table and the Layer-0 classification predicates. |
| TR2 | `river_network` | surface L-system | `R(k,w):k<7 → ;(gravel) Pw(gravel, 0.6w) ;(mud) Pw(mud, 1.6w) @Tflow(1.0) F(0.06w+8) [+(26+nran(0,9)) R(k+1, 0.62w)] −(11) R(k+1, 0.80w)`; `R(k,w):k≥7 → Pw(mud, 1.6w) %`. `@Tflow` is a tropism toward the negative gradient of the FD8 field, so rivers descend. **`gen_nested = false`.** |
| TR3 | `trail_network` | surface L-system | Same shape, `Pb(dirt, 1.4, 0.6)`, seeded at building doorways and coastline landings, biased downhill by a `?S` slope query. **The training-relevant one** — it creates low-risk corridors an agent can learn to prefer. |
| TR4 | `mudflat_region` | surface L-system | Polygon-contour marking: the turtle traces a closed ring with `{ . . }` and the interior is filled with `mud`. |

### 6.7 Buildings (3) — see §6b

| id | name | shell | interior | storeys |
|---|---|---|---|---|
| B1 | `warehouse` | CGA subset: `Subdiv` the lot, `Repeat` bays, `Comp(faces)` → façade with `Portal(door)` / `Portal(window)` | single open volume + 4 racking zones by Lopes growth | 1 |
| B2 | `office_3storey` | mass model + `Repeat(z, 3.2 m)` | pinned vertical core (U-stair 5.0 × 2.5 m + elevator), Lopes growth per storey, Merrell support gate | 3 |
| B3 | `bunker` | box | BSP room-and-corridor, depth 4 on a 64×64 grid | 1 |

---

## 6b. Indoor scenarios

Indoor is not a footnote. It changes the world model (2.5D), the asset library (shells, floor plans, props), the material palette (14 new classes) and the export (per-storey rasters + links).

### 6b.1 Walls occupy cells

The decision that silently breaks a nav grid is zero-thickness walls between cells. **Give the wall a cell.** At 0.5 m resolution a 0.25 m authored wall rounds up to exactly one cell, and the nav grid becomes a free byproduct of the same rasterisation that produces the mesh, through the same `raster_emitter`. The *mesh* keeps the authored 0.25 m thickness; the *raster* wall is one cell, always. Those are two different numbers on purpose and the code names them `wall_thickness_render_m` and `wall_cells` so nobody reconciles them by accident.

### 6b.1a Clearance, reconciled — one authored number, one lattice

> **The defect this repairs.** Revision 1 carried **three mutually unreconciled clearance numbers**: the gate demanded `min_path_width_cells = 3` (1.5 m at 0.5 m/cell), the architectural table specified `door_clear_width_m = 0.90` (IBC 32 in — **1.8 cells**, not representable, and *narrower* than the gate minimum) and `corridor_min_width_m = 1.12` (IBC 44 in — **2.24 cells**, also unrepresentable and also below the gate), and the consumer's hard-hazard margin is a fourth number again (1.0 m). Every interior the design generated would have been rejected by its own gate.

**The scheme.** One lattice at `grid_m = 0.5` for indoor *and* outdoor (this is what makes §6b.5's "mixed single-storey is literally one 2D grid" true, and it is the reason not to give interiors a finer grid). The architectural widths are **authoring-space values**; they are **snapped up** to whole cells at raster time; and every clearance number is **derived from one authored quantity**:

```
agent_radius_m                      # THE authored number. Default 0.35 (person-scale).

free_opening_m   = max(IBC_door_m,     2*agent_radius_m + 0.20)   # 0.20 m = shoulder slack
corridor_m       = max(IBC_corridor_m, 2*agent_radius_m + 0.40)
stair_width_m    = corridor_m

opening_cells    = ceil(free_opening_m / grid_m)
corridor_cells   = ceil(corridor_m     / grid_m)
gate.min_path_width_cells = opening_cells      # DERIVED, never authored independently
```

with `IBC_door_m = 0.813` (32 in clear) and `IBC_corridor_m = 1.118` (44 in). Note the previous table's 0.90 / 1.12 were already rounded-up IBC values; we keep the *code* honest by carrying the exact conversions and letting `max()` do the work.

**The arithmetic, at the two ends of the range.**

| | person-scale agent | Austin-scale vehicle |
|---|---|---|
| `agent_radius_m` | **0.35** | **3.00** (`rr = 0.15` normalized ÷ `scale = 0.05`) |
| `free_opening_m` | `max(0.813, 0.90) = ` **0.90** | `max(0.813, 6.20) = ` **6.20** |
| `opening_cells` = `ceil(·/0.5)` | **2** → rasterised **1.00 m** | **13** → rasterised **6.50 m** |
| `corridor_m` | `max(1.118, 1.10) = ` **1.118** | `max(1.118, 6.40) = ` **6.40** |
| `corridor_cells` | **3** → rasterised **1.50 m** | **13** → rasterised **6.50 m** |
| `gate.min_path_width_cells` | **2** | **13** |
| passes its own gate? | `min(2, 3) = 2 ≥ 2` ✓ | `min(13, 13) = 13 ≥ 13` ✓ |
| clears IBC? | 1.00 ≥ 0.813 ✓, 1.50 ≥ 1.118 ✓ | trivially ✓ |
| which recipe | `office_3storey`, `bunker` | `warehouse` (racking aisles) |

The two columns are not a coincidence — the `warehouse` recipe (B1) exists precisely because a 3 m-radius vehicle needs 6.5 m aisles, and the previous revision had no way to say that. **`agent_radius_m` is written into `scenario.json` and into the manifest**, so a bundle records the agent it was built for; loading a 0.35 m interior with a 3.0 m vehicle is a mismatch the adapter can and does detect.

**Worked example — a generated corridor and door passing the gate.** `warehouse` interior, `agent_radius_m = 0.35`, `grid_m = 0.5`. `#` = `wall_interior` (hard, occupied), `.` = `concrete_floor` (free).

```
Corridor cross-section (one raster row), corridor_cells = 3:

   col:    47    48    49    50    51
         [ # ][ . ][ . ][ . ][ # ]        3 free cells  =  1.50 m free width
                    ^ path
   gate step 4:  free width along the path = 3 cells  >=  min_path_width_cells (2)   PASS

Doorway in the corridor's end wall, opening_cells = 2:

   col:    47    48    49    50    51
         [ # ][ # ][ . ][ . ][ # ]        2 free cells  =  1.00 m free width
   gate step 4:  free width at the pinch = 2 cells  >=  2                            PASS
   flood fill from the entrance reaches the room beyond                              PASS
```

**The consumer-side consequence, stated rather than discovered.** At `cell_w = 0.5`, the one-sided EDT gives `phi_m` at the door's free cells = `1 cell × 0.5 = 0.50 m`. The consumer's `hard_margin_m` is **1.00 m** — both twins agree on that number at this cell size (§7.1a) — so `witness_gate` will report `feasible_count = 0` for any ray threaded through a doorway. **That is correct behaviour, not a bug.** The witness gate is a *soft-risk detour* activation witness; it never chooses the executed action, `lam_hard` is never gated, and A\*'s `hard_penalty = 25.0` is a finite surcharge rather than a prohibition. Doors remain traversable; what the gate declines to do is *certify a detour through one*. A consumer that wants indoor gating must set `gate.hard_margin_m ≤ 0.5` explicitly, and the manifest carries `gate_hard_margin_max_viable_m` so it can see the number without guessing (§7.5).

**Propagated everywhere.** §6b.3 step 4, §16.3's interior block, the `bridge_min_m` floor in §4.3a.4 and the nav-gate report's `min_corridor_width_m` all read these derived values from one place, `interior_spec::derive_clearances(agent_radius_m, grid_m)`. There is a unit test that the derivation is self-consistent — `opening_cells * grid_m >= free_opening_m` and `gate.min_path_width_cells <= min(opening_cells, corridor_cells)` — for `agent_radius_m` swept over `[0.15, 4.0]`.

### 6b.2 Generation pipeline

```
1. SHELL (grammar, sequential_priority + strict containment)
     lot polygon -> mass model -> Subdiv(Z, {3.2}*n) -> Storey polygons
                 -> Comp(faces) -> facade Repeat/Subdiv -> Portal(door|window)
2. CORE (pinned before growth)
     stair core + elevator shaft placed first; its cells are pre-occupied
     for every storey, so rooms grow AROUND it rather than into it.
3. FLOOR PLAN per storey  [Lopes et al. 2010]  -- a SIBLING module, not a grammar
     0.5 m grid -> weighted seeding from the room programme
                -> GrowRect / GrowLShape -> FillGaps
                -> connectivity + reachability repair
4. UPPER-FLOOR SUPPORT GATE  [Merrell 2010]
     unsupported = area(F_i - F_{i-1}) / area(F_i)
     REQUIRE unsupported == 0 for interior floors.
     Deliberate cantilevers are allowed but their edge is tagged BLOCKED_FALL.
5. STAIR GEOMETRY, dispatched on core footprint shape:
     narrow rectangular -> single straight run
     wide (2 flights)   -> U-shaped with a landing
     L-shaped           -> 90-degree turn with a quarter landing
     Commercial: tread >= 0.28 m, riser <= 0.18 m, width >= stair_width_m
     (== corridor_m, derived in 6b.1a; 1.118 m at agent_radius_m = 0.35).
     3.2 m storey => ~18 risers => ~5.0 m of run. Core budget 5.0 x 2.5 m min.
6. NAVIGABILITY GATE  (section 6b.3)   <-- runs EVERY time, unconditionally
7. CIRCULATION SKELETON -> hard keep-out
8. PROPS (greedy, Poisson-disc, < 1 ms/room) -- placed AFTER the gate, inside
   the keep-out mask, so decoration can NEVER invalidate navigability.
9. EMIT: geometry + per-storey occupancy/class rasters + cells + portals + links
```

Step 8's ordering is the whole trick. [Merrell 2011]'s interactive furniture layout and [Yu 2011]'s simulated annealing are both batch-cost and both unnecessary once circulation is a hard keep-out.

### 6b.3 Navigability validation — a gate, not a cost term

```cpp
// inc/cvc/world/cells.h
struct nav_gate_report {
  bool passed = false;
  int  components = 0;
  int  unreachable_cells = 0;
  std::vector<std::uint32_t> unreachable_rooms;
  std::vector<std::uint32_t> blocked_portals;
  std::vector<std::uint32_t> broken_links;
  double min_corridor_width_m = 0.0;
  int  repairs_applied = 0, resamples = 0;
  double largest_component_fraction = 0.0;
  std::string failure_reason;      // empty on pass
};

nav_gate_report validate(const cell_graph&, const std::uint8_t* occ,
                         int rows, int cols, double cell_w);
```

Algorithm, per storey:

```
1. Rasterise walls/doors at 0.5 m. Walls occupy cells.
2. Flood-fill the free set from the building entrance.
   FAIL if any room's floor set is outside the entrance's component.
3. For each (entrance, room-centroid) pair: cvc::nav::astar must return a path.
   FAIL if any returns empty.
4. Min free width along every solution path >= gate.min_path_width_cells,
   which is DERIVED from agent_radius_m by 6b.1a (2 cells = 1.00 m at the
   default 0.35 m agent; 13 cells = 6.50 m for the 3.0 m vehicle).
   It is never authored independently of the door width, which is how
   revision 1 ended up with a gate no generated interior could pass.
5. Multi-storey: every stair landing must be passable on BOTH layers, and the
   upper-floor slab opening must be BLOCKED_FALL everywhere except the landing.
   (Forget the second half and agents walk off the stairwell edge into a hole
   the grid calls floor.)
6. FAILURE POLICY, in order:
     a. REPAIR: carve the minimum-cost door on the shortest wall segment
        joining the two largest components. Re-validate. Up to 3 times.
     b. RESAMPLE: bump the storey's `element` id by one and regenerate.
        Up to 8 times.
     c. HARD FAIL: refuse to write the bundle, write the failing seed and
        the full nav_gate_report to provenance.json, and surface a red
        banner in the Lab.
```

Cost: EDT + flood fill + A\* over 64×64 = 4096 cells ≈ **< 1 ms**; a large 3-storey office ≈ 5–20 ms. It runs on every generated interior including in the live preview.

**A broken training environment must be loud, never silently shipped.** That is the reason for (c). [Merrell 2010]'s *soft* accessibility term fails 1-in-20 / 1-in-5; [ProcTHOR]'s 10 000 fully-navigable houses used a hard gate. We use the hard gate.

**Verification:** PR L6 generates **500 interiors** across all three building specs with 500 seeds and asserts 100 % pass after repair, and records the repair / resample / reject rate distribution in the test output so the caps are tuned from data rather than guessed.

### 6b.4 The 2D-grid vs 2.5D problem

`sim_world::config` has `rows/cols/min_x/min_y/max_x/max_y/scale` and **no z**. Interiors are inherently 2.5D. Three options:

| option | pros | cons |
|---|---|---|
| **A. Per-storey layered grids + explicit `link` list** | the existing 2D planner works **unmodified** on each layer; matches Recast off-mesh connections and [Jung 2024] MuNES; exports cleanly | the *consumer* must plan across layers — Python-side work outside our control |
| B. Unfolded plane (storeys side by side, stitched at stairs) | one grid, zero consumer changes | geometrically dishonest; cross-seam distances are wrong; agent sensing and any future RF break |
| C. Single floor only | trivially works today | rules out every multi-storey scenario |

**Recommendation: A as the format, C as the v1 export default.**

- The layered format is **defined, written and tested in PR L1** (the bundle-schema PR), so multi-storey worlds can be generated and inspected immediately.
- v1 **exports** `storeys: 1` by default. Single-storey scenarios are fully supported end to end with **zero** change to the concurrent session's contract.
- **This must be decided before PR L1 freezes the schema, not when PR L6 lands** — it changes the bundle, not the renderer. Flagged in §15 as needing a user decision.

`cells.json` (always written, even for one storey):

```json
{
  "schema": "cvcworld.cells/1",
  "frame": { "up": "+z", "units": "metres", "handedness": "right" },
  "storeys": [
    { "z": 0.0, "grid": "layer00/occupancy.npy",
      "cells": [ {"id": 0, "kind": "lobby", "label": "entry",
                  "bounds_xy": [10.0, 2.0, 22.5, 14.0],
                  "footprint": [[10.0,2.0],[22.5,2.0],[22.5,14.0],[10.0,14.0]],
                  "z_floor": 0.0, "z_ceiling": 3.0,
                  "portals": [0,1]} ],
      "portals": [ {"id": 0, "kind": "door", "a": 0, "b": 3,
                    "p0": [12.5, 4.0, 0.0], "p1": [13.5, 4.0, 0.0],
                    "height_m": 2.1, "width_m": 1.0,
                    "traversable": true, "opaque": false} ] },
    { "z": 3.2, "grid": "layer01/occupancy.npy", "cells": [...], "portals": [...] }
  ],
  "links": [
    { "kind": "stair", "from": {"storey":0,"r":41,"c":88}, "to": {"storey":1,"r":41,"c":88},
      "capability": ["legged","climbing"], "cost_up": 14.0, "cost_down": 9.0 },
    { "kind": "elevator", "from": {"storey":0,"r":44,"c":88}, "to": {"storey":1,"r":44,"c":88},
      "capability": ["any"], "cost_up": 30.0, "cost_down": 30.0 }
  ]
}
```

Note `portal::traversable` **and** `portal::opaque`, which are **two independent booleans and must stay independent**:

| record | `traversable` | `opaque` | why |
|---|---|---|---|
| open doorway | true | false | walk and see |
| closed steel door | false | true | neither |
| window / `glass_pane` | **false** | **false** | **a window is a visibility portal but not a nav portal** |
| curtained opening | true | true | walk but not see |

Conflating them is how a PVS system decides agents can walk through glass, and how a nav system decides a room is sealed when you can see straight through it. Revision 1 carried only `traversable`, which forced the visibility consumer to infer opacity from `kind` — a string it does not own.

### 6b.5 The indoor/outdoor seam

- A building sits on a **pad**: its footprint flattens the delta grid to the pad height. The pad edge is a ramp if the drop is < 0.4 m, otherwise a `void_fall` cell ring.
- Ground-floor doorway cells are `nav_class::door` and are marked in **both** the exterior and interior rasters.
- **v1 supports mixed indoor/outdoor scenarios when they are single-storey**, and this is nearly free: the terrain and the ground floor are coplanar on the same 0.5 m lattice, so it is *literally one 2D grid* — the interior is just a region whose classes happen to be `concrete_floor`/`wall_interior` instead of `grass`/`tree_trunk`. That is an honest, clean answer.
- `scenario.json` carries `"scene_kind": "outdoor" | "indoor" | "mixed"`.

### 6b.6 Emitted topology — the seam specification for the visibility design

Rooms are **cells** and doorways are **portals**, emitted as first-class generated entities. `cvc::world::cell_graph` (types in §6b.4) is written to `cells.json` and is drawable in the Lab (`Debug ▸ Show portals`) as coloured quads — which is also the fastest way to eyeball a broken interior.

> **This subsection is a contract, not a description.** A companion design owns cells / portals / PVS / BVH / occlusion culling and is being produced separately. Nothing in *this* document consumes the topology beyond a trivial draw. So the emission has to be specified precisely enough that the other design can be written against it without a conversation, and it is frozen here. **We design none of the renderer side.**

**Frame and units.** World metres, Z-up, right-handed, the same frame as `heightfield::sample(x,y)` and the same origin as the render scene. `cells.json` states this explicitly in its `frame` block so a consumer never infers it.

**Portal geometry.** A portal is a **planar convex quad**, given compactly as two floor-level endpoints plus a height:

```
p0, p1   : [x, y, z] world metres, the two ends of the portal's floor edge
height_m : extrusion along +z

quad = [ p0, p1, p1 + (0,0,height_m), p0 + (0,0,height_m) ]
```

**Winding is normative:** the quad is wound **counter-clockwise when viewed from cell `a`**, so its face normal `n = normalize((p1−p0) × (0,0,1))` points from `a` toward `b`. Every portal-flood implementation needs a consistent orientation, and every implementation that is not given one guesses. Portals are always planar and always vertical in v1; a future sloped portal (a stair opening viewed as a portal) would need a third point and is explicitly **not** emitted today.

**Cell geometry.** `bounds_xy` is an AABB, kept because it is a cheap reject. `footprint` is the **CCW polygon ring in world metres** and is the authoritative shape, because floor-plan growth produces L-shapes and an AABB for an L-shaped room over-claims by up to 60 % of its area — which in a portal flood means over-drawing an entire neighbouring wing. `z_floor` / `z_ceiling` bound the cell vertically. Cells are guaranteed **simply connected** (no holes) and **non-overlapping in plan within a storey**; both are asserted by `world_cells_test`.

**Identity and stability.** `cell::id` and `portal::id` are derived from the generator's `path_id` chain (§5.2), *not* from emission order. Regenerating a world after an unrelated edit — a different vegetation seed, a repainted mudflat — leaves every surviving cell and portal id unchanged. This is what lets an incremental PVS cache per id instead of recomputing per bundle, and it is tested by the insertion-stability suite (§12.2).

**What we emit.** Cells, portals (with `traversable` **and** `opaque`), cross-storey links with asymmetric cost and a capability list, and the per-storey occupancy raster the cells index into.

**What we deliberately do not emit, and will not add without a request from the visibility design:** no cell-to-cell visibility matrix, no PVS, no BVH, no occluder fusion, no anti-portals, no portal-to-portal sight-line precompute, no exterior cell decomposition (the outdoor world is culled by the flat 32×32 tile grid of §4.5/§8.8 and has no cell graph). The Lab ships **one** consumer: draw the current cell plus ≤ 2 portal hops, so interiors are not absurdly slow before the real runtime lands.

**Versioning.** `cells.json` carries `"schema": "cvcworld.cells/1"`. Adding a field is a minor change and readers must ignore unknown keys; removing or re-meaning one bumps the major and the adapter refuses to load.

---

## 7. Material system

### 7.1 The contract, verified against merged code

The material work is **no longer concurrent — it is in the tree.** `#230` merged as `8b6f426`. There are now **two** authoritative surfaces and they have different arities, which revision 1 conflated:

**The C++ surface** — `inc/cvc/nav/material.h:93-94`, read directly:

```cpp
material_planes material_build(const float *risk_raw, const std::uint8_t *hard,
                               int rows, int cols,
                               double cell_w, double scale, double sigma);
```

It takes `rows, cols` **and** `cell_w` directly. It does **not** take `bounds` or `center`.

**The Python surface** — `/home/joe/src/cvc/GRL-SNAM/grl_snam/material.py:224-234` and `:255-257`:

```python
class MaterialGrid:
    def __init__(self, risk, hard, bounds, center, scale, *, sigma=1.0, params=None):
        ...
        ny, nx = self.risk_raw.shape
        self.cell_w = (self.bounds[2] - self.bounds[0]) / (nx - 1)   # line 247
        ...
        planes = _native.material_build(self.risk_raw, self.hard,
                                        self.cell_w, self.scale, self.sigma)
```

The binding's arity is `(risk, hard, cell_w, scale, sigma)` because numpy carries the shape; the C++ arity is `(risk, hard, rows, cols, cell_w, scale, sigma)` because a raw pointer does not. **Both are true at their own layer.** Revision 1 asserted the Python arity as "the" signature and that was half right in a way that would have mis-specified any C++ caller.

**The consequence for the bundle: it must satisfy both.** A Python consumer needs `bounds` + `center` + the array shape; a C++ consumer needs `rows`, `cols`, `cell_w`. So the manifest carries **all of them**, plus `cell_w` *explicitly* — even though Python re-derives it — and the loader asserts the written `cell_w` equals `(max_x − min_x)/(cols − 1)` bit-for-bit (§12.3). Neither entry point ever has to guess, and a disagreement is loud.

Three further consequences, each a repair of a mistake every earlier draft made:

**(a) `cell_w = extent / (n − 1)`, not `extent / n`.** Bounds are corner-inclusive sample centres. `planner.far_pair_in_free_space`'s `to_world` uses `c/(nx-1)` and `r/(ny-1)`; `MaterialField._to_grid` uses `(pos_world[:,1]-mny)/(mxy-mny)*(ny-1)`. A window written as "512 × 512 spanning 256.0 m" is read back as `cell_w = 0.50098 m`. Because `phi_hard_m = sqrt(edt2(hard)) * cell_w` is in **world metres** and feeds `d_hat_sdf_m` and the gate's `hard_margin_m` with no rescaling, that is a systematic, silent mis-scaling of the hard-hazard barrier in every world ever generated.

> **Rule.** The default export window is **513 × 513 samples spanning exactly 256.0 m** (`cell_w = 256.0 / 512 = 0.5` exactly). `grid_spec` computes `cell_w` the consumer's way and a unit test reconstructs it and compares bit-for-bit.

**(b) Row 0 = `min_y`.** Confirmed: `pos[:,0] = (pos_world[:,1] - mny)/(mxy - mny)*(ny-1)`. The research BEV builder (`bev.py`) uses `r = floor((y_max - y)/res)` — the opposite. **A mirrored risk field still looks completely plausible and poisons every training run undetectably.** Defence in depth (§7.6).

**(c) Do not export consumer tuning constants *as configuration*.** The circulated brief states `lam_soft = 1.5, lam_hard = 2.0, k_sharp = 5.0, d_hat_sdf_m = 3.0`. The shipped Python defaults are `lam_soft = 0.5, lam_hard = 1.0, k_sharp = 1.25, d_hat_sdf_m = 12.0` (material.py `MaterialParams`), with an explicit comment that *"here lam_soft = 1.5 measurably LAUNCHES a vehicle off-world."* The brief was stale on day one. **The manifest carries grid facts, provenance hashes and raw statistics — and, new in revision 2, a clearly quarantined `consumer_frame_ref` block that is provenance and is never read back** (§7.1a, §7.5).

### 7.1a The export frame — confronting σ-in-cells

> **The defect this repairs.** Revision 1 picked 0.5 m/cell and moved on. But `sigma` is measured in **cells** (`material.h:168` "blur, in cells"; tap radius `int(4σ + 0.5)`, `material.cpp:159`), so the effective blur *in metres*, the effective hard margin (`2·cell_w` when the caller passes `≤ 0`) and the effective gate horizon (`horizon_m / cell_w`) **all move with the export resolution** — while the consumer's `lam_soft` / `lam_hard` / `d_hat_m` were behaviourally tuned in some other frame. Picking a cell size without saying which frame is a silent retune of somebody else's validated constants.

**What the consumer's assumed frame actually is.** `GateParams`' own docstring states it, and it is worth quoting because it is the whole argument:

> *"the source BEV was 0.5 m/cell, this sim's default story grid is ~2.1 m/cell"* … `horizon_m = 25.0` because *"25 m ≈ the source's 12 cells at the default 96-cell/±100 m story grid (the source's 6 m would be 3 cells here — myopic)"* … `hard_margin_m = None` ⇒ 2 grid cells, *"the source's margin (1.0 m at 0.5 m/cell)"*.

And `MaterialParams` states the force-constant frame: `lam_soft = 0.5` was validated at ≈ **2.1 m/cell** with **σ = 1 cell ⇒ a 2.1 m blur length**, because *"the normalized-frame gradients are ~an order hotter"* than the source's pixel frame.

**What changes at 0.5 m/cell.** `grad_r` is divided by `float32(cell_w · scale)` (`material.cpp:213`), so in the continuum limit it is *risk per metre ÷ scale* and is cell-size-independent. The coupling is entirely through the **blur length**: `risk = gaussian_blur(risk_raw, σ_cells)` smooths over `σ_cells · cell_w` metres, and the gradient magnitude at a material boundary is `O(1 / (σ_cells · cell_w))`. So:

| quantity | consumer's tuned frame | our export at 0.5 m/cell, σ = 1 cell | ratio |
|---|---|---|---|
| blur length | 2.1 m | 0.5 m | **0.24×** |
| ‖grad r~‖ at a class boundary | 1× | **4.2×** | 4.2× |
| effective `lam_soft` | 0.5 | **≈ 2.1** | 4.2× |

`lam_soft = 1.5` is documented to *launch a vehicle off-world*. **Exporting at 0.5 m/cell and telling the consumer "σ = 1.0" would ship every bundle in a regime the consumer's own code comments call catastrophic.** That is the concrete harm, and it is why this section exists.

**The fix: σ is recorded as a length, not a count.** The manifest's new `frame` block carries a physical blur length `sigma_m` and derives the cell count from it:

```
sigma_m                  = 2.0            # scene_kind == "outdoor"  (~= the tuned 2.1 m)
sigma_recommended_cells  = sigma_m / cell_w        # = 4.0 at cell_w = 0.5
```

For `scene_kind` `indoor` or `mixed`, `sigma_m = 0.5` (**σ = 1 cell**, the *source BEV* frame exactly), because a 2.0 m blur across a 1.5 m corridor destroys the feature it is supposed to smooth. The manifest then also carries `lam_soft_scale_hint = sigma_m / 2.1`, a **dimensionless ratio of recorded blur lengths** — 0.95 outdoors, 0.24 indoors. It is not a value of `lam_soft`; it is arithmetic the consumer would otherwise have to redo, and it goes stale only if the consumer changes its own blur length, which is recorded three lines above it.

**Why 0.5 m/cell and not the consumer's 2.1 m.** Three independent reasons, all checkable:

1. **It is the source frame.** The research BEV that every one of these formulas was ported from is 0.5 m/cell. Exporting there is exporting into the frame the method was invented in.
2. **The twins agree on the hard margin at exactly this cell size, and nowhere else.** The C++ default is `hard_margin_m = 1.0` (fixed metres, `material.h:114`). The Python default is `None ⇒ 2·cell_w`. Those are the same number **iff `cell_w = 0.5`**. At 2.1 m/cell the Python twin uses 4.2 m and the C++ twin uses 1.0 m, and a bundle would behave differently depending on which language loaded it.
3. **0.5 m is the coarsest lattice on which a door exists at all.** A 0.90 m opening is 0.43 cells at 2.1 m/cell. The entire indoor half of this design is unrepresentable in the consumer's story frame.

**What must be recorded so this is auditable.** The `frame` block carries `rows`, `cols`, `bounds`, `center`, `cell_w`, `cell_h`, `cell_w_formula`, `row_order`, `scale`, `sigma_m`, `sigma_recommended_cells`, `blur_bleed_radius_m` (= `4·σ_cells·cell_w`, §7.2a), `gate_horizon_recommended_cells`, `gate_hard_margin_max_viable_m` and `agent_radius_m`. All of those are **grid facts**. Separately, a `consumer_frame_ref` block records the three numbers the recommendations were *derived from* (`reference_cell_w_m: 2.1`, `reference_sigma_m: 2.1`, `source_bev_cell_w_m: 0.5`) plus a hash of the `MaterialParams`/`material_config` defaults as of the generating commit. **`world_bundle.py` must never read `consumer_frame_ref`** — that is asserted by a test, and it is what keeps §7.1(c)'s principle intact: the block is forensics, so a bundle can be diagnosed a year later, and it is never configuration, so a bundle can never ship stale tuning.

**One divergence the manifest exposes rather than hides.** The gate horizon is `horizon_cells` (integer **cells**, default **12**) in C++ and `horizon_m` (**metres**, default **25.0**) in Python. At our `cell_w = 0.5`, Python resolves to `round(25/0.5) = 50` cells while an unconfigured C++ caller uses **12 cells = 6.0 m** — which is precisely the *"myopic"* value GRL-SNAM's own docstring rejects. The manifest therefore carries `gate_horizon_recommended_cells = 50`, and §15.1 R17 names the divergence.

### 7.2 The registry — semantic classes are the authored truth

The user asked to mark terrain as dirt / gravel / mud / grass. That is a **discrete semantic class map**, strictly richer than the two rasters the contract consumes. So:

- **The class map is the authored truth.** It is persisted for provenance, visualization and future richer consumers (RF, sensor modelling, aesthetics).
- **`risk_raw` and `hard` are a derived, versioned projection** of it by table lookup.

```cpp
// inc/cvc/world/surface.h
namespace cvc::world {

enum class nav_class : std::uint8_t {
  free = 0, rough, blocked_wall, blocked_fall, door, portal
};
enum class tier : std::uint8_t { low, medium, high_soft, hard_hazard };

struct surface_class {
  std::uint16_t id;
  const char*   name;
  tier          t;
  float         rho;          // risk in [0,1] -> risk_raw
  bool          hard;         // -> hard raster
  nav_class     nav;
  float         albedo[3];
  float         roughness;
  float         veg_density;  // scatter multiplier
  // RF slots. Carried and exported ZEROED; populating them needs a spec review.
  float         penetration_db_per_m[3];
  float         reflection_loss_db;
};

class surface_registry {
public:
  static const surface_registry& builtin();               // the 32 shipped classes
  static surface_registry from_yaml(std::string_view, std::vector<diagnostic>*);
  const surface_class& operator[](std::uint16_t) const noexcept;
  std::uint16_t by_name(std::string_view) const;
  std::size_t size() const noexcept;
  std::string to_json() const;                            // -> registry.json
  std::uint64_t ontology_hash() const noexcept;
};
}
```

**Ontology variants ship**, following the research repo's shape: `merged_default`, `soft_vegetation` (grass → 0.15, bush → 0.30), `strict_water_mud` (mud/puddle → 0.95). Swapping one re-runs the projection only (~2 ms for 513²), never a re-derivation. **This is the highest-value idea in the material design**: one authored class map yields N risk fields, so material-semantics ablations cost nothing.

Full 32-class table in §16.2.

### 7.2a Hard classes carry ρ = 0.00, and what to do about blur bleed

> **The defect this repairs.** Revision 1 gave `wall_interior`, `glass_pane`, `void_fall`, `water_deep`, `cliff_rock`, `tree_trunk`, `boulder` and `fence_pole` **ρ = 1.00 *and*** flagged them `hard`. That double-counts, and — because `risk_raw` is **blurred** by the consumer — it actively poisons the free space next to every wall.

**Decision: every `hard` class carries `ρ = 0.00`.** The `hard` raster carries them, and nothing else needs to.

**Why the double-count is real.** The consumer penalises hard cells in *both* of the two places that act:

1. **The planner.** `A*` adds `hard_penalty = 25.0` on a hard cell, on top of `risk_weight = 10.0 × risk`. A hard cell at ρ = 1.0 therefore costs `25 + 10 = 35`, of which 10 is a second, independently-tuned copy of "do not go here".
2. **The force field.** `F_hard = −lam_hard · db · grad φ` with `db = −sigmoid(k_sharp·(d̂ − φ_m))`, and `φ_m` is built **from the `hard` plane alone** (`phi_m = sqrt(edt2_squared(hard)) · cell_w`). The barrier is already correctly signed, correctly scaled in metres, and ungated. It does not need `F_soft` helping.

**Why the blur bleed is worse than the double-count.** `risk = gaussian_blur(risk_raw, σ)` uses taps `exp(−0.5/σ²·k²)` over radius `int(4σ + 0.5)` with reflect padding (`material.cpp:155-200`). Take a corridor at the person-scale default: 3 free cells wide, `wall_interior` cells at each side, `σ = 1 cell` (the indoor recommendation, §7.1a). The free centre cell is 2 cells from each wall centre. A ρ = 1.0 half-plane at distance `d` cells contributes ≈ `Q(d/σ)` (the Gaussian upper tail):

| | ρ_hard = 1.00 | ρ_hard = 0.00 |
|---|---|---|
| contribution of each wall at 2 cells, σ = 1 | `Q(2) ≈ 0.023` each | 0 |
| corridor-centre `r~` | `0.06 + 2(0.023) ≈ **0.11**` | `**0.06**` (its own `concrete_floor` ρ) |
| the same at σ = 4 cells (the **outdoor** recommendation) | `Q(0.5) ≈ 0.309` each → `r~ ≈ **0.62**` | `**0.06**` |

The σ = 4 row is the alarming one, and it is not hypothetical — it is what happens the moment an interior appears inside an *outdoor* export window (`scene_kind: "mixed"` at 4 cells, which is exactly why §7.1a makes `mixed` use σ = 1). **A corridor reading `r~ ≈ 0.62` is riskier than `mud` (0.80) is far, and `F_soft = −lam_soft·grad r~` points the agent *out of the corridor it must walk down*.** That is not a subtle miscalibration; it is a sign error in effect.

**What is lost by ρ_hard = 0.** Nothing the consumer needs. "Keep a margin from walls" is exactly what `F_hard` provides, in metres, from `φ_m`, at the right steepness. The A\* surcharge is unchanged. The only thing that disappears is an uncalibrated soft halo, and that halo was the bug.

**Blur bleed at material boundaries, generally.** The consumer's blur is isotropic and **occlusion-unaware**: a ρ = 0.85 puddle outside a warehouse raises the risk of the floor inside it, because the wall between them contributes nothing to the smoothing. Four mitigations, in order of how much they buy:

1. **ρ_hard = 0.00** (above). This is the dominant case by a wide margin, because walls are the only thing that is both high-ρ-under-the-old-scheme and everywhere.
2. **Bounded boundary contrast.** No two classes may sit adjacent across a single hard cell with `|ρ_a − ρ_b| > max_boundary_contrast` (default **0.60**). The registry validator checks this against the adjacency `raster()` actually produces (not against the class table in the abstract), and **warns** — naming the offending class pair and a sample cell — in the Lab and in `cvc-worldgen inspect`. It warns rather than errors because a legitimate world can contain a puddle next to concrete.
3. **The bleed radius is published and drawn.** `blur_bleed_radius_m = 4·σ_cells·cell_w` goes in the manifest (2.0 m indoors, 8.0 m outdoors at the §7.1a recommendations) and the Surface tab draws it as a halo ring around the brush cursor, so an author *sees* that painting a puddle 1.5 m from a doorway will raise the doorway's risk before they commit the stroke.
4. **`scene_kind` selects σ** (§7.1a), so interiors are never blurred at the outdoor length.

**And an explicit non-mitigation.** We do **not** pre-blur, erode, or wall-mask `risk_raw` before export. The export is the RAW plane by contract (§7.5); any smoothing we applied would be invisible to the consumer, would compose with its own blur, and would break the "if their σ changes, no bundle is invalidated" property that is the entire reason the seam is a file.

### 7.3 The class map is a function, not an image

There is **no stored class raster**. One call evaluates the three-layer priority stack and emits every plane together:

```cpp
// inc/cvc/world/raster.h
namespace cvc::world {

struct grid_spec {
  int rows = 513, cols = 513;
  double min_x, min_y, max_x, max_y;     // corner-INCLUSIVE sample centres
  static constexpr const char* row_order = "min_y_first";   // compile-time constant

  double cell_w() const noexcept {       // the CONSUMER's formula, verbatim
    return (max_x - min_x) / double(cols - 1);
  }
  double cell_h() const noexcept { return (max_y - min_y) / double(rows - 1); }
  bool   square(double eps = 1e-9) const noexcept;
  void   check_congruent(const grid_spec& other) const;   // throws
};

struct raster_out {
  std::vector<std::uint16_t> klass;      // rows*cols, semantic class id
  std::vector<float>         risk_raw;   // rows*cols, [0,1]
  std::vector<std::uint8_t>  hard;       // rows*cols, 0/1
  std::vector<std::uint8_t>  occupancy;  // rows*cols, 0/1
  std::vector<float>         height;     // rows*cols, metres
  std::vector<std::uint8_t>  layer_owner;// rows*cols, which layer won (provenance)
};

// THE single entry point. Every plane comes from one grid_spec, so a
// material-vs-occupancy-vs-heightfield misalignment is unrepresentable.
void raster(const world_model&, const grid_spec&, raster_out&);
}
```

Invariants asserted by `raster()` and by unit tests:

- `klass.size() == risk_raw.size() == hard.size() == occupancy.size() == height.size() == rows*cols`.
- `risk_raw[i] == registry[klass[i]].rho` exactly, for every cell of a randomized world.
- `hard[i] == registry[klass[i]].hard` exactly.
- **`hard ⊆ occupancy`** — every hard cell is also occupied. (Soft mud is *not* occupied.)
- `grid_spec::row_order == "min_y_first"` and it is written into every manifest.

### 7.4 The three-layer priority stack

Highest priority wins. `layer_owner` records which one, so the Lab can answer "why is this cell mud?".

| layer | name | source | authority |
|---|---|---|---|
| 0 | **derived** | predicate table over `(h, slope, curvature, twi, flow, aspect)` | lowest |
| 1 | **grammar paint** | `P` / `Pw` / `Pb` / `Stamp2D` terminals from `raster_emitter` | middle |
| 2 | **authored** | ordered `paint_op` list from the brush | highest |

Layer 0's default predicate table (editable in the Surface tab, serialized in the `.lsys` header):

```
h < -0.5                              -> water_deep
h < 0.0                               -> water_shallow
abs(h) < 3.0                          -> sand
h > 1180                              -> snow
slope > 42                            -> cliff_rock
slope > 30                            -> scree
twi > 7.5 && slope < 6                -> mud
twi > 5.5 && slope < 10               -> tall_grass
h > 620                               -> bare_rock
default                               -> grass
```

Layer 2 is an **ordered op list**, not a baked raster:

```cpp
struct paint_op {
  enum shape_t : std::uint8_t { disc, capsule, rect, polyline, flood } shape;
  std::uint16_t klass;
  double  a[2], b[2];      // shape-dependent
  double  radius_m, feather_m;
  std::uint32_t order;     // strictly increasing
  std::uint64_t stamp_id;  // for the UI's "stamp #14" readout
};
```

Ops are serialized in the `.lsys` header. Delete, reorder and per-op inspection are all free; this is a structured, inspectable undo, strictly better than a linear ring for the paint case (which also gets a 64-op ring for stroke-level Ctrl+Z).

**One stroke appends exactly one `paint_op` on mouse-up**, never one per mouse-move sample.

> **Feather vs blur.** `risk_raw` is the RAW plane; the consumer applies its own separable blur on top, at the σ **this bundle recommends** — `sigma_recommended_cells`, which is 1.0 cell (0.5 m) for `indoor`/`mixed` and 4.0 cells (2.0 m) for `outdoor` (§7.1a). Brush feather is therefore *additional* smoothing. Default `feather_m = 0.0` so exported worlds are comparable to the research baseline; the Surface tab labels the slider "extra feather (on top of the consumer's σ = N cells)" with N filled in from the current `scene_kind`, and draws the `blur_bleed_radius_m` halo around the brush cursor so the bleed band is visible before the stroke lands.

### 7.5 Bundle format

```
world_a/
├── manifest.json
├── registry.json                 # the full 32-class ontology, incl. zeroed RF slots
├── provenance.json               # seeds, .lsys sha256s, gate reports, tool version
├── cells.json                    # cells + portals + links (always, even 1 storey)
├── world.json                    # island table, preset, extents (for re-generation)
└── layer00/
    ├── class.npy                 # uint16 (513,513)   <- the AUTHORED TRUTH
    ├── risk_raw.npy              # float32 (513,513)  <- CONTRACT INPUT 1
    ├── hard.npy                  # uint8   (513,513)  <- CONTRACT INPUT 2
    ├── occupancy.npy             # uint8   (513,513)
    ├── height.npy                # float32 (513,513)
    └── layer_owner.npy           # uint8   (513,513)  <- provenance, optional
```

We write **`risk_raw` and `hard` and nothing derived**. `risk`, `phi_hard_m`, `grad_rx/ry/px/py` and the `[1,6,H,W]` stack all live on the consumer's side (`MaterialGrid._derive()`, material.py:250-273). If their blur σ, EDT convention, gradient normalisation or channel order changes, **not one generated bundle is invalidated.** This is the strongest available decoupling and it is why the export is file-based rather than a direct call.

`manifest.json` — grid facts, provenance and statistics only:

```json
{
  "format": "cvcworld/2",
  "generated_utc": "2026-08-27T22:41:08Z",
  "tool": { "name": "cvc-worldgen", "version": "1.0.0", "libcvc": "8b6f426" },
  "scene_kind": "mixed",
  "storeys": 1,
  "grid": {
    "rows": 513, "cols": 513,
    "bounds": [-128.0, -128.0, 128.0, 128.0],
    "center": [1050.0, -1150.0],
    "cell_w": 0.5, "cell_h": 0.5,
    "cell_w_formula": "(max_x - min_x) / (cols - 1)",
    "row_order": "min_y_first",
    "row_order_note": "row 0 is min_y. The research BEV builder is max_y-first; flip exactly once, in the adapter.",
    "cpp_note": "cvc::nav::material_build takes (rows, cols, cell_w, scale, sigma) directly; MaterialGrid takes (bounds, center, scale) and re-derives cell_w. Both are satisfied by this block."
  },
  "frame": {
    "scale": 0.05,
    "agent_radius_m": 0.35,
    "sigma_m": 0.5,
    "sigma_recommended_cells": 1.0,
    "blur_bleed_radius_m": 2.0,
    "lam_soft_scale_hint": 0.24,
    "gate_horizon_recommended_cells": 50,
    "gate_hard_margin_max_viable_m": 0.5,
    "note": "sigma is expressed in CELLS by the consumer, so it is recorded here as a LENGTH and converted. See roadmap 7.1a. lam_soft_scale_hint is a dimensionless ratio of the two blur lengths recorded here, not a value of lam_soft."
  },
  "consumer_frame_ref": {
    "_warning": "PROVENANCE ONLY. Never read this block into a config. world_bundle.py is tested not to.",
    "reference_cell_w_m": 2.1,
    "reference_sigma_m": 2.1,
    "source_bev_cell_w_m": 0.5,
    "material_params_defaults_hash": "b3:41ae…",
    "material_config_defaults_hash": "b3:7c02…"
  },
  "meta": {
    "scale":  0.05,
    "center": [1050.0, -1150.0],
    "bounds": [-128.0, -128.0, 128.0, 128.0],
    "region": 128.0,
    "rr":     0.15,
    "d_hat":  0.35,
    "dt":     0.06,
    "vmax":   0.9,
    "nsub":   2
  },
  "endpoints": {
    "selection": "far_pair_in_free_space",
    "inflate_m": 6.0,
    "inflate_cells": 12,
    "start_world": [-98.4, -71.2],
    "goal_world":  [ 84.9,  93.6],
    "component_fraction": 0.94
  },
  "material": {
    "ontology": "merged_default",
    "ontology_hash": "b3:9f1c…",
    "hard_class_rho": 0.0,
    "max_boundary_contrast": 0.6,
    "note": "risk_raw and hard are the RAW contract inputs. All derived planes (risk, phi_m, gradients) belong to cvc::nav::material_build / grl_snam.material.MaterialGrid. Hard classes carry rho = 0 by design (roadmap 7.2a): the consumer already penalizes hard via the A* surcharge and the phi_m barrier, and a rho=1 wall bleeds through the blur into the corridor."
  },
  "stats": {
    "class_fractions": { "grass": 0.41, "dirt": 0.09, "mud": 0.06, "…": 0.0 },
    "risk_mean": 0.27, "risk_std": 0.22, "risk_max": 0.90,
    "hard_fraction": 0.062, "occupancy_fraction": 0.071,
    "observed_fraction": 1.0
  },
  "difficulty": {
    "risk_detour": 1.31,
    "hard_frontage": 0.18,
    "bucket": 3,
    "note": "risk_detour is an UNVALIDATED candidate label. Raw statistics above are sufficient to recompute a better one without regenerating."
  },
  "validation": {
    "passed": true, "components": 1, "repairs_applied": 0, "resamples": 0,
    "outdoor": { "policy": "single-island", "island": "Kestrel",
                 "largest_component_fraction": 0.994, "channels_crossed": 0 }
  }
}
```

**`meta` values are in the NORMALIZED frame, not metres.** This is a correction to revision 1, which wrote `rr: 6.0, d_hat: 12.0, vmax: 8.0, dt: 0.1, nsub: 1` — metre-flavoured numbers that `SdfNavigator` consumes in the same normalized frame as `VEHICLE_DEFAULTS["L"] = 0.035` (≈ 3 m wheelbase). A bundle written with `rr = 6.0` would give the vehicle a repulsion radius comparable to the entire half-region (`region · scale = 128 × 0.05 = 6.4`) and would be unusable. The canonical values, verified in `grl_snam/tools/austin.py:85-95` and `grl_snam/tools/material_demo.py:31-44`, are `scale = 0.05, rr = 0.15, d_hat = 0.35, dt = 0.06, nsub = 2, vmax = 0.9` — i.e. `rr/scale = 3.0 m` of vehicle radius and `d_hat/scale = 7.0 m` of geometry-SDF reach. Note also that `meta["d_hat"]` (**normalized**, the geometry barrier) and `MaterialParams.d_hat_sdf_m` (**12.0 metres**, the *material* barrier) are two different quantities; revision 1 copied the latter into the former.

The `meta` block is not optional. `SdfNavigator.__init__` (`grl_snam/nav.py:37-66`) reads `meta["scale"]`, `meta["center"]`, `meta["rr"]`, `meta["d_hat"]`, `meta["dt"]`, `meta["vmax"]`, `meta.get("nsub", 1)`; `FogScenario.__init__` reads `meta["dt"]` (`scenario.py:137`) and `meta["region"]`/`meta["center"]` (`scenario.py:233-235`); the corner-goal picker at `nav.py:437-440` reads `meta["scale"]`, `meta["center"]`, `meta["region"]`, `meta["bounds"]`. A bundle carrying only bounds + scale is `np.load`-able and **not** scenario-loadable, and that gap is the difference between "the test passes" and "a training run starts".

`endpoints` matters just as much. `planner.far_pair_in_free_space` draws both endpoints from the **largest inflated-free component**, and `austin.py:58-62` documents the trap verbatim: *"a cell can be free and still unreachable once the route is inflated, so a run drives most of the way and then reports no route."* The generator selects endpoints the same way and records the inflation it used.

### 7.6 Defence against silent mirroring

Four independent layers, because this failure is undetectable by eye:

1. `grid_spec::row_order` is a **compile-time constant**, not a runtime field.
2. It is written into every manifest and **asserted by the Python reader** at load time, not just tested once.
3. The flip happens **exactly once**, in `grl_snam/world_bundle.py`, with its own round-trip test.
4. Every debug overlay and window preview stamps a deliberately **asymmetric L-shaped fiducial** in the corner of the region. A mirrored field is instantly visible.

### 7.7 Rendering the materials

- **Vertex colours** on terrain chunks from `registry[klass].albedo`, sampled at chunk resolution.
- **Splat texture** per tile at rung-proportional resolution (T0: 512², T1: 256², T2: 128², T3/T4: vertex colour only), height-aware blend with `depth = 0.2`, triplanar with `p = 8` on slopes > 35°.
- The predecessor bakes cloud transmittance into the terrain albedo at `SHADOW_RES = 96` with 202 752 CPU `sampleSky` calls per bake, single-threaded, every 16 frames. Replaced by a **projected cloud-shadow texture** derived from the CA coverage field, updated at 5 Hz, sampled in the fragment shader — O(1) in world size.
- The **render** class raster is cached per tile at 2 m resolution. The **export** raster is always evaluated fresh at the requested `grid_spec`. They are allowed to disagree at sub-cell scale, deliberately, and a unit test asserts they agree at shared sample points. The render cache never leaves the renderer.

### 7.8 The outdoor connectivity gate — mandatory, and the archipelago is why

> **The defect this repairs.** Revision 1 gated interiors with real seriousness and gated the *outdoors* not at all. An archipelago makes that untenable: two islands separated by deep water are, to a ground agent, an **unsolvable episode**, and `water_deep` and `cliff_rock` are `hard`. A batch run that draws an ROI straddling a channel produces bundles that look perfect and cannot be finished.

The outdoor gate runs on **every exported window, every time**, exactly like the interior gate, and shares its EDT and flood-fill machinery.

```cpp
// inc/cvc/world/cells.h  (alongside nav_gate_report)
struct outdoor_gate_report {
  bool   passed = false;
  int    components = 0;
  double largest_component_fraction = 0.0;
  int    islands_in_window = 0;
  int    channels_crossed = 0;         // open-water separations inside the window
  double narrowest_channel_m = 0.0;
  double narrowest_bridge_m  = 0.0;
  int    bridges_forced = 0, resamples = 0;
  std::string policy;                  // "single-island" | "forced-bridges" | "amphibious"
  std::string failure_reason;          // empty on pass
};

outdoor_gate_report validate_outdoor(const raster_out&, const grid_spec&,
                                     const archipelago_spec&, double inflate_m);
```

Algorithm:

```
1. Rasterise the export window at the export grid_spec (one raster() call, so the
   gate can never disagree with the exported bytes about where the water is).
2. free = !occupancy && !hard.
3. Inflate the OBSTACLES by inflate_m (6.0 m, matching planner.far_pair_in_free_space
   exactly), because "a cell can be free and still unreachable once the route is
   inflated" -- austin.py:58-62, quoted verbatim in section 7.5.
4. Label 4-connected components of the inflated-free set (4-connectivity, matching
   the planner's neighbourhood, not 8).
5. Record component count, largest_component_fraction, and the per-pair channel /
   bridge widths of section 4.3a.4.
6. Apply the POLICY (decision D9). Endpoints are ALWAYS drawn from ONE component,
   so a written bundle is never internally unsolvable; the gate governs whether the
   WORLD is accepted and how much of the window is wasted.
7. FAILURE POLICY, mirroring the interior gate:
     a. REPAIR: under `forced-bridges`, raise the narrowest disqualifying saddle to
        +0.6 m over a 3-cell feather and paint it `sand`/`gravel`. Re-validate.
        Up to 3 times. Under `single-island`, instead SHRINK the window to the
        dominant island's mask union its shelf, and re-validate once.
     b. RESAMPLE: bump the window's element id (and, in seeded placement mode, the
        island-placement element) and regenerate. Up to 8 times.
     c. HARD FAIL: refuse to write the bundle, record the seed, the policy and the
        full outdoor_gate_report in provenance.json, and raise a red banner in the Lab.
```

Cost: one EDT + one labelling pass over 513² ≈ **3 ms** natively. It runs in the live preview too, and the Lab's World tab shows `components: 1 ✓` or `components: 3 — 62 % largest` in red.

**Acceptance thresholds** (defaults; all in `archipelago_spec`):

| policy | pass condition |
|---|---|
| `single-island` (**v1 default**) | `largest_component_fraction ≥ 0.98` **and** `channels_crossed == 0` |
| `forced-bridges` | `components == 1` after repair, **and** `narrowest_bridge_m ≥ max(bridge_min_m, 2·agent_radius_m + 1.0)` |
| `amphibious` | `components == 1` with `water_shallow` treated as free (requires the ontology change of D9 option 3) |

The policy is **recorded in the manifest** (`validation.outdoor.policy`) and in `provenance.json`, because a corpus mixing policies silently mixes two different task definitions. **Which policy v1 ships is a user decision — D9 in §15.3.**

---

## 8. Level of detail

### 8.1 The axis ordering, and why

Measured on prettyhatemachine at Release:

| config | fps |
|---|---|
| 1280×800, shadows ON, 32 trees | **24.8** |
| 1280×800, shadows OFF, 32 trees | **21.7** |
| 640×400, shadows ON, 32 trees | **29.2** |

Quartering the pixels buys 18 %. Disabling shadows makes it *worse*. **The demo is CPU-bound in the main loop**, not GPU-bound: the wind re-poses every tree's vertices into two merged CPU-side buffers and re-uploads them (`lsystem_forest.cpp:1403-1408`, at `WIND_STRIDE = 2`), and the sea field is regenerated. The stride hacks (`WIND_STRIDE=2, SEA_STRIDE=4, CLOUD_STRIDE=8, SHADOW_STRIDE=16`) are the loop already fighting this.

Worse, `GeometryNode::updateVertices` (verified at `src/cvcGL/GeometryNode.cpp:490-520`) **deep-copies its argument into the `runOnMainThread` lambda** (`runOnMainThread([this, xyz]() {` at :502) and then writes per point via `pts->SetPoint()` in a scalar loop (:511). For a 130 k-vertex merged buffer that is a 3.1 MB copy plus 130 k virtual calls, every update.

**Therefore the ladder is ordered:**

```
1. ANIMATION LOD    who sways at all, at what rate, on CPU or GPU
2. GENERATION LOD   derivation depth by distance
3. UPDATE LOD       what is static vs re-uploaded per frame
4. DRAW-CALL LOD    merging / prop count
5. TRIANGLE LOD     the thing everyone thinks LOD means
```

A design that only reduces triangles has missed the bottleneck.

### 8.2 Animation LOD — what happens to the wind at scale

**The wind moves to a vertex shader and becomes a function of world position and time. CPU sway is capped at a constant.**

| band | radius | count | mechanism | per-frame CPU |
|---|---|---|---|---|
| **A0** | ≤ 60 m | **≤ 24 plants** (8 wasm) | full CPU cascade, `updateVertices` on a small dedicated actor | ~0.3 ms |
| **A1** | ≤ 260 m | ~2 400 | **GPU sway**, static VBO, zero CPU | **0** |
| **A2** | ≤ 700 m | ~9 000 | GPU sway at reduced amplitude | **0** |
| **A3** | ≤ 1600 m | ~16 000 | camera-facing cards, GPU sway ×0.3 | **0** |
| **A4** | > 1600 m | — | canopy tint baked into the terrain splat | **0** |

The A0 count is **constant at every world size, by construction.** That is the single most important sentence in this section. `lsystem_forest` at 32 trees does more per-frame vertex work than this design does at 1.2 million plants.

**GPU sway mechanism.** A vertex-shader replacement at `//VTK::PositionVC::Impl` displaces by

```glsl
// injected at //VTK::PositionVC::Dec  -- see the caveat below
in vec2 swayAttr;          // x = phase, y = amplitude*stiffness
uniform float u_time;
uniform vec2  u_windDir;
uniform float u_windGain;

// at //VTK::PositionVC::Impl, before the MCVC transform:
float s = sin(u_time * 1.7 + swayAttr.x + vertexMC.z * 0.06);
vertexMC.xy += u_windDir * (s * swayAttr.y * u_windGain);
```

`cvc::gl::GeometryNode::addVertexShaderReplacement(anchor, code)` exists (`inc/cvc/gl/GeometryNode.h:94`) and is already used twice in `lsystem_forest.cpp` (:159, :362). The uniforms go through `vtkShaderProperty`, which cvcGL already reaches.

> **⚠ Verified VTK trap — must be handled, and it is silent if not.**
> The per-vertex sway attribute rides `cvc::geometry::uvs` (the only spare per-vertex channel; `GeometryNode` uploads it via `SetTCoords`, `GeometryNode.cpp:845-863`). But `vtkOpenGLPolyDataMapper::ReplaceShaderTCoord` **early-returns when `tcoordnames` is empty**, and `tcoordnames` is populated only by iterating *bound textures*. An untextured vegetation mesh therefore never gets `in vec2 tcoordMC;` declared, the shader compiles cleanly, and the sway silently does nothing.
> **Fix:** the vertex-shader replacement must inject its own declaration at `//VTK::PositionVC::Dec` (`in vec2 tcoordMC;` aliased to `swayAttr`), *or* a 1×1 dummy texture is bound to force VTK's declaration. PR L4 ships **both** the injection and a **wasm shader-compile smoke test** that asserts the program links and that a probe vertex actually moves. This is not optional and it is not obvious.
>
> **Companion invariant, documented in `mesh_emit.h`:** *"vegetation meshes reserve `uvs` for sway. A bark texture atlas on vegetation must use a second UV set (which cvcGL does not have today) or `tangents`."* If UVs are ever needed for vegetation albedo, `cvc::geometry::tangents` is the escape hatch, at the cost of an upload path that does not exist.

### 8.3 Generation LOD

For a `gen_nested == true` ruleset, one derivation at `build_gen` produces every rung as a `filter_level(k)` subword — coherent, free, and pop-free by construction, because rung `k−1`'s module set is a **prefix-closed subset** of rung `k`'s. This is [Weber & Penn 1995]'s range degradation: *re-interpret* the stored derivation, do not convert the geometry.

For `gen_nested == false` (see §5.4), the archetype falls back to N independent cached derivations. Cost is 5× derivation for that archetype only.

Additionally, `sides_by_order` degrades the cylinder tessellation per branch order — 16→4 sides on orders ≥ 2 typically removes 60–75 % of wood triangles for no silhouette change.

### 8.4 The per-class ladders

**Terrain** — chunked LOD, 128 m chunks, skirts (not morphing) in v1:

| rung | sample spacing | verts/chunk | tris/chunk | octaves | switch radius |
|---|---|---|---|---|---|
| T0 | 2 m | 65² = 4 225 | 8 192 | 9 | ≤ 220 m |
| T1 | 4 m | 33² = 1 089 | 2 048 | 8 | ≤ 480 m |
| T2 | 8 m | 17² = 289 | 512 | 7 | ≤ 1 000 m |
| T3 | 16 m | 9² = 81 | 128 | 6 | ≤ 2 200 m |
| T4 | 32 m | 5² = 25 | 32 | 5 | > 2 200 m |

Neighbour constraint `|level_self − level_neighbour| ≤ 1`, enforced by the selector and tested. T-junctions are hidden by **8 m vertical skirts**. CDLOD vertex morphing is deferred to a later phase because it needs a per-vertex morph weight and terrain's UV channel is spent on the splat texture.

**Vegetation** — `lod_gens` from the recipe header, e.g. `spiral_broadleaf` `[10, 8, 6, 0, 0]`:

| rung | derivation gen | wood tris | foliage | sway |
|---|---|---|---|---|
| A0 | 10 | ~12 300 | full cards | CPU |
| A1 | 8 | ~3 100 | cards | GPU |
| A2 | 6 | ~770 | cards, halved | GPU |
| A3 | — | 2 (billboard) | 8-azimuth impostor atlas | GPU ×0.3 |
| A4 | — | 0 | terrain splat tint | — |

**Foliage is triangles (camera-facing cards), never `GeometryRenderMode::LINES`.** The predecessor's needle stars are `LINES`; we do not inherit them. This is deliberate and load-bearing (§8.6): degenerate zero-length `GL_LINES` are not reliably discarded, and under `setRenderLinesAsTubes` a degenerate line expands to a visible blob. Cards also give the A3 impostor path for free.

**Rocks:** R0 (1 280 tris) ≤ 120 m, R1 (380) ≤ 400 m, R2 (90) ≤ 1 200 m, then folded into the terrain silhouette.

**Buildings:** B0 (full shell + façade detail + interior when the camera is inside a cell) ≤ 200 m, B1 (shell + window quads) ≤ 900 m, B2 (mass box, per-component hashed tint — the `occupancy_to_walls` look) beyond.

**Clouds:** one near `VolumeNode` + 8–12 far depth-tested shells. Fallback ladder if the volume misses budget: half-resolution raycast with upsample → shells only → soft billboards.

### 8.5 Selection math

```cpp
// inc/cvc/lod/select.h  -- pure math. No VTK, no I/O, no allocation in the hot path.
namespace cvc::lod {

struct view_params {
  double eye[3];
  double viewport_h_px       = 800.0;
  double tan_half_fov        = 0.41421356;   // 45 deg vertical
  double desired_pixel_error = 2.0;
  double hysteresis          = 0.15;         // 15% band on the switch-OUT radius
};

// screen_error_px = world_error * h / (2 * dist * tan(fov/2))
double screen_error_px(double world_error_m, double dist_m, const view_params&) noexcept;

// Distance at which world_error_m first exceeds desired_pixel_error.
double switch_radius_m(double world_error_m, const view_params&) noexcept;

// Hysteretic rung choice. `current` is last frame's rung; the switch-out radius
// is switch_in * (1 + hysteresis), so a camera hovering on a boundary cannot
// oscillate. Pure function of its arguments.
int select_rung(double dist_m, const double* world_error_m, int nrungs,
                int current, const view_params&) noexcept;

struct budget {
  std::uint32_t max_props = 48;
  std::uint64_t max_tris  = 2'500'000;
  std::uint64_t max_bytes = 700ull << 20;
};

struct candidate {
  std::uint32_t group_id;        // tile or scatter-cell morton id
  int           desired_rung;
  int           min_rung;
  double        projected_area;  // solid-angle proxy
  double        dist_m;
  const std::uint64_t* tris_per_rung;
  const std::uint64_t* bytes_per_rung;
};

struct plan {
  std::vector<int> rung;
  std::uint32_t props; std::uint64_t tris, bytes;
  int binding;                   // which budget bound: 0 none, 1 props, 2 tris, 3 bytes
};

// Greedy: everything starts at min_rung; sort by
//   score = projected_area / (1 + (dist/1000)^2)          [roadmap 22.1.6]
// and promote toward desired_rung until a budget binds. Deterministic,
// allocation-free after one reserve, and a pure function of (candidates, budget)
// -- the ideal unit test.
plan solve(std::vector<candidate>&, const budget&);

// Cross-fade weight as a TIME CONSTANT off world dt (roadmap 22.4.3),
// never as a per-frame ratio.
inline double fade_alpha(double elapsed_s, double tau_s) {
  return 1.0 - std::exp(-elapsed_s / tau_s);
}
}
```

Thresholds:

| knob | large | default | wasm | source |
|---|---|---|---|---|
| `desired_pixel_error` | 2.0 px | 2.0 px | 3.5 px | roadmap §22.1.6 default is 1.0; 2.0 halves terrain triangles at an error nobody sees at 1280×800 |
| `hysteresis` | 0.15 | 0.15 | 0.20 | wider on wasm because rung swaps cost more there |
| `fade_tau_s` | 0.12 s | 0.12 s | 0.18 s | ≈ 2× the 60 fps frame time. The roadmap's 250 ms is for *streaming* invalidation, a different event |
| `max_props` | **measured** (placeholder 48) | 48 | 24 | see §8.7 |
| `max_tris` | 2.5 M | 1.6 M | 700 k | under §20.13.7's 4 M client default, with headroom for the shadow re-render |
| `max_bytes` | 700 MB | 380 MB | 90 MB | wasm keeps `cvc::geometry` + VTK's copy + a 25 MB module under 2 GiB with 10× fragmentation margin |
| tile promotions/frame | 2 | 2 | 1 | UE5 World Partition discipline: a hard cap beats a soft budget |
| actor content swaps/frame | 4 | 4 | 1 | with fixed-capacity meshes a swap is a buffer overwrite, not a rebuild |
| A0 CPU-swayed plants | 24 | 24 | 8 | constant at every world size |

### 8.6 Batching — fixed-capacity merged actors

This is the mechanism that makes the whole thing work with **zero new cvcGL API**.

`GeometryNode::updateVertices` / `updateColors` require only that the **point count** match what `setGeometry` established — verified at `GeometryNode.cpp:504-513`, which logs *"point-count mismatch … ignoring; call setGeometry to change topology"* on a count mismatch and nothing else. **Capacity, not usage, is what must match.** So:

```cpp
// src/cvcGL/examples/lsyslab_render.h  (demo-side; promoted only if a second consumer appears)
namespace lsyslab {

// A merged actor whose TOPOLOGY IS FIXED FOREVER at its capacity. Unused
// triangles are DEGENERATE (i0 == i1 == i2) -- the rasterizer discards zero-area
// primitives, and vtkPolyDataNormals never runs again after the one setGeometry().
//
// TRIANGLES ONLY. Line-mode actors are packed densely with no padding, because
// zero-length GL_LINES are not reliably discarded and expand to a visible blob
// under setRenderLinesAsTubes. This is why vegetation foliage is cards, not
// needle stars.
class fixed_mesh {
public:
  void create(SceneGraph& sg, const std::string& name,
              std::size_t cap_verts, std::size_t cap_tris);

  void begin();                                       // reset the write cursor
  std::size_t push(const submesh&, const double xf[16]);
  void commit();   // pad the tail with degenerate tris at the FIRST USED vertex
                   // (keeps the bbox tight), then updateVertices + updateColors

  std::size_t used_verts() const noexcept, cap_verts() const noexcept;
private:
  std::shared_ptr<GeometryNode> node_;
  std::vector<double>        xyz_;   // capacity-sized, allocated ONCE
  std::vector<unsigned char> rgb_;
  std::size_t                cur_ = 0;
};
}
```

Two details that matter and were nearly wrong:

- **Park unused vertices at the first *used* vertex's position, not the origin**, so the bounding box stays tight. A loose bbox makes the directional shadow map fit the wrong volume and makes the frustum culler pessimistic.
- **Run `vtkPolyDataNormals` once at `create()`** on a fully-populated dummy fill, so the one-time normals pass sees real geometry rather than a sea of degenerates.

**Honest accounting of what `commit()` costs.** `updateVertices` deep-copies the buffer into the lambda and writes per-point. For a 130 k-vertex vegetation chunk that is ~3.1 MB + 130 k `SetPoint` calls ≈ 1.1 ms. **Therefore `commit()` is not a per-frame operation.** It runs only when a chunk's *content* changes — a rung promotion, a tile page-in, or a Tier-2 rebuild — capped at 4/frame. Per-frame vegetation motion is entirely GPU-side (§8.2), so the steady state does **zero** `commit()` calls. This is the difference between this design and the predecessor, and it is the whole point.

Capacity table (native `default`; wasm ≈ 40 %):

| actor | cap verts | cap tris | points MB | colors MB |
|---|---|---|---|---|
| `terrain_t0` | 42 000 | 80 000 | 1.01 | 0.13 |
| `terrain_t1` | 46 000 | 88 000 | 1.10 | 0.14 |
| `terrain_t2` | 34 000 | 62 000 | 0.82 | 0.10 |
| `terrain_t3_t4` | 38 000 | 66 000 | 0.91 | 0.11 |
| `veg_a0_cpu` | 26 000 | 44 000 | 0.62 | 0.08 |
| `veg_a1` | 210 000 | 370 000 | 5.04 | 0.63 |
| `veg_a2` | 150 000 | 260 000 | 3.60 | 0.45 |
| `veg_a3_cards` | 64 000 | 64 000 | 1.54 | 0.19 |
| `rock_r0` / `r1` / `r2` | 60 k / 44 k / 20 k | 100 k / 70 k / 30 k | 2.98 | 0.37 |
| `bldg_b0` / `b1` / `b2` | 44 k / 90 k / 12 k | 74 k / 120 k / 18 k | 3.50 | 0.44 |
| `interior` (current cell + 2 hops) | 40 000 | 68 000 | 0.96 | 0.12 |
| **total** | **~940 k** | **~1.5 M** | **~23 MB** | **~2.9 MB** |

**Net: the LOD system requires no changes to `inc/cvc/gl/GeometryNode.h`.** That removes the design's only dependency on PR #223's merge, and removes the need to register a new test in `src/cvcGL/CMakeLists.txt` near the line #223 owns.

An optional `GeometryNode::swapGeometry()` (swap the polydata, keep the actor and mapper) remains the *correct* library fix for variable-topology LOD and is flagged for whoever needs it later. We do not need it and do not ship it.

### 8.7 The prop budget is measured, not asserted

`max_props = 48` is a **placeholder**. The evidence behind the "63-actor cliff" is two confounded data points: `examples/README.md:189-193` records 64 actors → 2 actors giving ~17 → 29 fps, in a scene where every one of those 64 actors was *also* doing a per-frame wind vertex upload *and* being drawn in both the main pass and the shadow bake. That is a per-actor cost curve with three variables, not a prop-count threshold.

**PR L3 ships `src/cvcGL/test/cvcgl_prop_sweep.cpp`**, which sweeps fps against (a) prop count, (b) per-prop triangle count, (c) shadow-caster participation, (d) whether the prop receives a per-frame vertex upload, on the target machine, and **sets the default from the measurement.** The code must not inherit this document's 48 blindly. Registration goes at **EOF of `src/cvcGL/CMakeLists.txt` (after line 293, which is the file's real last line — `endif()` closing the `CVC_BUILD_EXAMPLES` guard)**. #223's hunks are at ~24 and ~281-287, so an EOF append is ≥ 6 lines clear of its last hunk and will not textually conflict with it — though see §14.2 on why "append at EOF" is not conflict-*proof* in general.

cvcGL tests *do* run in CI: **16** `add_test(NAME cvcgl_*)` entries in `src/cvcGL/CMakeLists.txt` (lines 169, 175, 182, 188, 195, 201, 207, 212, 218, 224, 231, 253, 259, 268, 275, 283), executed under Xvfb + llvmpipe, with a drift guard at `ci.yml:334-342` that errors if zero reach CTest. **The number is 16, not 18** — and `ci.yml`'s own comments are stale in *both* directions, saying "13" at `ci.yml:231` and "18" at `ci.yml:284`. Three cvcGL executables are built and deliberately not registered (`cvcgl_state_probe` :236, `cvcgl_transform_bench` :242, `cvcgl_renderer_bench` :248); `enable_testing()` is at :166. Whatever the count is when L3 lands, the number in this document is not the authority — `grep -c 'add_test(NAME cvcgl_' src/cvcGL/CMakeLists.txt` is.

### 8.8 Culling

A flat 32×32 (large: 64×64) uniform grid, frustum-culled as a 2D rect test plus a `[z_min, z_max]` slab test. ~5 µs/frame for 1024 tiles. Deliberately not a tree (§4.5).

Indoors, the cell+portal graph gives portal-flood culling — but that runtime belongs to the concurrent visibility effort. We emit the topology and stop, plus a trivial "current cell + 2 portal hops" consumer so interiors are not absurdly slow before that effort lands.

The renderer's per-prop `AllocatedRenderTime` budget is computed by VTK's default `vtkFrustumCoverageCuller` and thrown away today. We continue to throw it away, deliberately: two selectors disagreeing is worse than one. This is also why `vtkLODActor` / `vtkLODProp3D` are rejected (§1.3).

### 8.9 Popping mitigation, in priority order

1. **Switch per scatter cell (32 m), never per plant** — the roadmap's per-leaf rule, taken at a granularity that is actually invisible (§2.1).
2. **15 % hysteresis** on every boundary. Free.
3. **Hashed-alpha dithered cross-fade** [Wyman & McGuire 2017] over `fade_tau_s`, at `//VTK::Color::Impl` with `discard`. GLES3-safe (the fragment *colour* anchor, not the normal anchor — see §8.11).
4. **Terrain skirts** in v1; CDLOD morphing later.
5. **Impostor azimuth blending** — the A3 billboard blends the two nearest of 8 azimuths by barycentric weight, so rotating the camera does not step.

### 8.10 Streaming / async

There is no network streaming. The world is procedural, so "streaming" means *baking*, and the bake is local.

| | native | wasm single-threaded | wasm `-pthread` |
|---|---|---|---|
| driver | 2-thread job queue | `world::pump(4000 /*µs*/)` from the render loop + `emscripten_sleep(0)` | worker, `num_threads` **capped at 4** |
| order | **visible tiles → near ring → world** (World Machine's ordering) | same | same |
| coarse-to-fine | a tile's T4 mesh is produced first and shown, then refined | same | same |
| cancellation | `deriver::cancel()` + a generation counter per tile request | same | same |
| never | blank the viewport | same | same |

`std::thread` construction **aborts** in the single-threaded wasm build (`pthread_create` returns `ENOTSUP`). Existing `cvc::nav` code survives only because the same stub returns `emscripten_num_logical_cores() == 1`, so `parallel_for` takes its inline branch. Every `cvc::world` entry point therefore defaults `num_threads = 0` and **never passes an explicit count**, matching the `cvc::nav` discipline exactly. In the `-pthread` build, `PTHREAD_POOL_SIZE=4` < `navigator.hardwareConcurrency`, so the cap is mandatory.

### 8.11 GLES3 constraints inherited by every shader we write

- `//VTK::Normal::Impl`'s writable local is `normalVCVSOutput` on desktop and `normalizedNormalVCVSOutput` under Emscripten; assigning the former is an ESSL "can't modify an input" error that fails the *whole program* and makes the actor invisible. Reuse the existing `CVC_FS_NORMAL` macro pattern.
- No geometry shaders. No `GL_DOUBLE`. No global-scope `mat4 g = <uniform expr>;`.
- Wide lines and large points are silently 1 px — another reason foliage is cards.

### 8.12 LOD state

Snake_case (§2.2), rooted at `lab.lod.*`:

```
lab.lod.preset                = "balanced"      # aggressive | balanced | pristine
lab.lod.desired_pixel_error   = 2.0
lab.lod.hysteresis            = 0.15
lab.lod.fade_tau_s            = 0.12
lab.lod.max_props             = 48
lab.lod.max_triangles_visible = 2500000
lab.lod.tile_m                = 128.0
lab.lod.cpu_sway_budget       = 24
lab.lod.force_rung            = -1              # -1 = auto
lab.lod.freeze_camera         = false
```

LOD state is **excluded** from the `.lsys` file and from the export manifest. A world is `(master_seed, .lsys sha256 set)`; LOD cannot affect it. This satisfies the §22.5 determinism/replay requirement and §28.1's `includeLodTraces` question by construction.

---

## 9. The Laboratory widget

### 9.1 Verified platform constraints that shape the design

| fact | source | consequence |
|---|---|---|
| ImGui **1.92.9 core**, `IMGUI_HAS_DOCK` absent | `deps-live/include/imgui/imgui.h:32` | **No docking, no viewports.** Fixed windows + `BeginTabBar` + one hand-rolled splitter. Not a compromise — L-studio, the reference tool for this exact domain, shipped MDI **tabs**. |
| **Keyboard does not reach the browser build** | `inc/cvc/gl/ImGuiOverlay.h:56-57` | **The text grammar editor is native-only.** Designed in from day one. |
| `io.IniFilename = nullptr` | `ImGuiOverlay.cpp:379` | Persist layout ourselves via `SaveIniSettingsToMemory()` on `io.WantSaveIniSettings`, stored next to the `.lsys`. No `imgui.ini` litter. |
| no `misc/cpp/imgui_stdlib.h` in the packaged headers | `deps-live/include/imgui/` | Vendor the ~20-line `ImGuiInputTextFlags_CallbackResize` handler. Do **not** bump the imgui recipe for this. |
| `ImGuiColorTextEdit` is not in `libcvc-deps/recipes` | — | v1 uses `InputTextMultiline` + a bottom error strip. Per-line red-background markers are a later, real dependency decision. |
| overlay draws **inside** VTK's render pass | `ImGuiOverlay.cpp:211-232` | The Lab UI **appears in offscreen captures** — free panel screenshots for docs, and a real regression artifact. |
| touch: only 2+ fingers intercepted; panels auto-hide below a 700 px short side | `TouchGestures.cpp:42-52`, `ImGuiOverlay.cpp:183-197` | 1-finger passes through to VTK/ImGui, so painting works on mobile for free. |
| a second `SceneRenderer` over one `SceneGraph` is **broken** | `SceneRenderer.h:55-62` (measured mean luma 81.8 → 0.0) | The specimen preview needs its own mechanism — see §9.3. |

### 9.2 Layout

```
┌─ Menu bar ──────────────────────────────────────────────────────────────────────────┐
│ File(New · Open · Save · Fork Variant… · Export Scenario… · Batch…)  Model  View     │
│ Run(F5 Regenerate · ☑Continuous · Esc Cancel)                                       │
│ Debug(Salt element IDs · Freeze LOD · Tint by rung · Show portals)                   │
│ Mode: ( Author ⇄ Play )                                              Edit(Ctrl+Z/Y)  │
├───────────────┬──────────────────────────────────────┬──────────────────────────────┤
│ LIBRARY       │                                      │ [Grammar][Params][Surface]   │
│               │                                      │ [World][Seeds][LOD][Stats]   │
│ ▾ archipelago │        LIVE 3D VIEW (the world)      │                              │
│   ├ windy     │                                      │  ── active tab body ──       │
│   └ eroded    │   ┌──────────────┐                   │                              │
│ ▸ spiral_broa │   │  SPECIMEN    │ ← the focused     │                              │
│   ├ dense     │   │  PREVIEW     │   asset, always   │                              │
│   └ juvenile  │   │ (own scene,  │   at FULL gen     │                              │
│ ▸ pine_monop  │   │  offscreen)  │                   │                              │
│ ▸ boulder_fr  │   └──────────────┘                   │                              │
│ ▸ warehouse   │                                      │                              │
│ ▸ office_3f   │   ⌗ brush: [mud ▾]  r = 26 m         │                              │
│ ▸ cumulus_an  │                                      │                              │
│               │   HUD  47 fps · 1.58 M tri · 41/48   │                              │
│  (prototype/  │        props · 4 281 mod · ● T2      │                              │
│   extension   │                                      │                              │
│   tree)       │                                      │                              │
├───────────────┤                                      │                              │
│ GALLERY       │                                      │                              │
│ [▣][▣][▣][▣]  │                                      │                              │
│  pinned       │                                      │                              │
│  variants     │                                      │                              │
├───────────────┴──────────────────────────────────────┴──────────────────────────────┤
│ ⚠ line 7: unknown module 'Q'   │ derive ▁▂▅▂▁▄▁ 41 ms │ preview gen 6 / build 10     │
│ props ████████░ 41/48 · tris ██████░░░ 1.58/2.50 M · veg 903 k · terr 205 k          │
└─────────────────────────────────────────────────────────────────────────────────────┘
```

Left = L-studio's gallery-at-the-edge fused with vlab's prototype/extension tree. Centre = the world with an inset single-asset preview. Right = tabbed editors (L-studio's shipped MDI design, and what works without docking). Bottom = Blender's node warnings + timings overlay flattened into a status strip.

**Mode switch.** `Play` hides Grammar and the Seeds internals, leaving Library / Gallery / Params / Surface-brush / Export and one master dice. Same code path, half the surface. This is L-studio's Design/Execute split and it is the highest-leverage feature for "usable by a researcher who wants worlds, not grammars."

**It never opens empty.** The Lab boots on `archipelago` with a visible island and a valid export window. The first move is "accept or discard the prompt", never "face a blank canvas" [Compton & Mateas 2015].

### 9.3 The specimen preview — mechanism, resolved

The predecessor recon establishes that a second `SceneRenderer` over the *same* `SceneGraph` silently migrates every actor and leaves the first drawing black. That constraint is **per scene**, not global. So:

- The specimen lives in its **own `SceneGraph`** (`"lab_specimen"`) with its **own offscreen `SceneRenderer`** at 320 × 320. Two disjoint scenes, one renderer each — the documented constraint is satisfied.
- The preview is rendered **only when dirty**, never per frame: on a T1 drag it re-renders at up to 30 Hz; otherwise it is static.
- `SceneRenderer::frameRGB()` (`SceneRenderer.h:67-132`) returns the pixels; they are uploaded to a GL texture once per re-render and drawn with `ImGui::Image`.
- Cost: one 320² readback (~300 KB) plus one texture upload per dirty frame ≈ 0.4 ms. Acceptable at ≤ 30 Hz, invisible when static.

This uses no invented API. It is written down here because it is the load-bearing UX element and the obvious implementation is broken.

### 9.4 Tab by tab

**Grammar** (native: editable; wasm: read-only + `Reload from file` + the numeric rule-row editor)

- Rule list as an **array of rows** (axiom + one row per production), individually addressable, reorderable by drag-drop, individually seedable, each with `×` and `⧉`. This is Houdini's L-System SOP shape and it is what a free-text blob cannot give you.
- Below it, a raw `InputTextMultiline` over the verbatim block for people who prefer bytes.
- `▸ Derived string` — symbol count, module count, max depth, and the first 4 kB of the word with `…12 431 more` elision. This is how a user learns why their rule did nothing.
- `▸ Cost curve` — `PlotLines` of estimated module count at generations 1..12, **so the exponential is visible before you drag**. `Generations` uses `ImGuiSliderFlags_AlwaysClamp`. If a derivation truncates at `max_modules`, the banner says so and reports the drop count.
- `▸ Nesting` badge — green `gen_nested` or amber "non-monotone: per-rung derivation".
- Live parse on **every keystroke, no debounce**; regenerate on debounce and **never on a grammar that does not parse** (a half-typed rule is a syntax error 100 % of the time you are mid-word).

**Params** — only what the header's `params:` block exposes, in domain language with tight ranges (`Branch angle 1 [35.0]°`, never `theta [0..6.28]`). The slider range *is* the design statement. Curve editors for `f(age) → branch_angle` and `f(t) → seg_radius` as an `InvisibleButton` canvas with draggable control points; drag-drop a curve onto a parameter to bind it. `AlwaysClamp` on cost knobs only.

**Surface** — the tab that produces the training data:

```
Surface ───────────────────────────────────────────────────────
 Palette    outdoor ▸ [dirt][gravel][grass][tall_grass][sand]
                      [mud][puddle][scree][rubble][asphalt]
                      [concrete_ext][snow][water_sh][boulder]
            indoor  ▸ [concrete_fl][tile][lino][wood][carpet]
                      [grating][wet_floor][debris][wall_int]
                      [glass_pane][door_closed][void_fall]
 Brush      size ▬▬▬●▬▬ 26.0 m    shape ( ● )( ▭ )( ╱ )( ⬚flood )
            extra feather  0.0 m   (consumer blurs at σ=1.0 cells = 0.50 m;
                                    bleed halo 2.0 m drawn on the cursor)
            mode ( Paint )( Erase→derived )( Sample/eyedropper )
 Show       ( Composite ▾ ) derived · grammar · authored · composite
                            risk_raw · hard · occupancy · phi_hard
                            layer_owner · risk-detour path
 Layers     ☑ derived   ☑ grammar paint   ☑ authored
            click a cell → "class=mud (layer 2: paint, stamp #14)"
 ─ Classification rules (layer 0) ──────────────────────────────
   h < -0.5                     → water_deep        [×]
   abs(h) < 3.0                 → sand              [×]
   slope > 42                   → cliff_rock        [×]
   twi > 7.5 && slope < 6       → mud               [×]
   …                                    [+ add]  [Apply → rebuild]
 ─ Marking grammars (layer 1) ──────────────────────────────────
   ☑ river_network   seed 4211 🎲 [edit…]
   ☑ trail_network   seed 8890 🎲 [edit…]
   ☐ mudflat_region  seed  117 🎲 [edit…]
 ─ Ontology ────────────────────────────────────────────────────
   ( merged_default ▾ )  soft_vegetation · strict_water_mud · custom…
   class table: id · name · tier · ρ · hard · nav · swatch · veg
   [ editing ρ is Tier-1b: reprojects in ~2 ms, no re-derivation ]
 ─ Distribution ────────────────────────────────────────────────
   class fractions ▁▃█▂▁▄▁▁  vs RELLIS reference ▁▄▆▃▁▃▁▁
 ─ Contract preview (the exact bytes that will be written) ─────
   class     uint16  (513,513)   32 classes present
   risk_raw  float32 (513,513)   mean 0.27  σ 0.22  min 0.00 max 0.90
                                 (max 0.90 = water_shallow; hard classes are rho=0)
   hard      uint8   (513,513)   6.2 % set    hard ⊆ occupancy ✓
   cell_w    0.500000 m  (= 256.0 / 512)   row 0 = min_y ✓
   frame     sigma_m 0.50 -> sigma 1.0 cells   bleed 2.0 m   scene_kind mixed
             gate horizon 50 cells (25.0 m)    hard_margin viable <= 0.50 m
   outdoor   components 1   largest 99.4 %     policy single-island (Kestrel)
   [ preview risk_raw ] [ preview hard ] [ preview phi_m ] [ Validate ] [ Export… ]
```

Three details that matter:

1. **Picking goes through `heightfield::sample()`**, never the LOD mesh (§2.1).
2. **The contract preview shows the exact arrays that will be written** — shape, dtype, mean, σ, hard fraction, `cell_w` with its arithmetic spelled out, and the row-order check. A user marking mud sees immediately that `risk_raw` went 0.25 → 0.80 there and that `hard` did not change. This is the single best defence against a silently-wrong export.
3. **`Erase→derived`, not `Erase→void`.** Erasing reveals the procedural classification underneath, which is almost always what you meant.

The **class-fraction histogram against the RELLIS reference distribution** is what tells you your synthetic worlds have drifted off the real data — before a 200-world dataset is generated rather than after.

The **risk-detour path overlay** draws the planner's actual route over the current field. It closes the loop from a paint stroke to a training consequence in one click.

> **Trap, surfaced in the UI:** the overlay's route is planned with `cvc::nav::astar(..., cost)` and **`simplify` is skipped when a cost field is present** — the line-of-sight shortcut ignores cost and would straighten the detour right back through the mud. `navdemo::plan_route` always simplifies; the Lab's overlay must not. Without this, painting mud and seeing no detour reads as "the material feature is broken" when the actual cause is post-hoc path smoothing.

**World** — the island table (name, centre, radius, `r_core`, peak, `freq_scale`, biome) with add/remove/reseed per row and a `Place N islands` button driving the §4.3a.2 sampler, a live **connectivity readout** (`components: 1 ✓` / `components: 3 — 62 % largest`, red on fail) with per-pair channel and bridge widths, the window-policy selector, sea level, cloud base/top with a live **"peaks pierce by N m"** readout (red if ≤ 0), tile size, species mix table (species × density/ha × slope range × altitude range × min spacing), erosion controls with an `Apply to authored region` button and an estimated-seconds readout, settlement placement, and the export ROI rect picker with a live cell-count readout (`513 × 513 @ 0.5 m = 256.0 × 256.0 m`).

**Seeds**

```
master              20260827   🎲
─────────────────────────────────
placement                  0   🎲 🔒
species                    1   🎲
maturity                   2   🎲
size                       3   🎲
phase                      4   🎲
sway                       5   🎲 🔒
rule_choice                6   🎲
param_jitter               7   🎲
surface                    8   🎲 🔒
terrain                    9   🎲 🔒
building                  12   🎲
floorplan                 13   🎲
[ Randomize unlocked ]
 Debug ▸ Salt element IDs → last audit: class_map ✓ risk_raw ✓ hard ✓ occupancy ✓
```

🔒 excludes a stream from `Randomize unlocked`. Because overrides anchor to `path`/`element`, re-seeding an unrelated stream **cannot** orphan a hand edit. Where it genuinely can (the element no longer exists), the button names the count in a confirm dialog: *"Reseeding PLACEMENT will orphan 3 hand edits."* The salt-audit result is surfaced here, not only in CI — the answer must always be "none changed".

**LOD** — rung-colour overlay (T0 green → T4 grey; A0 white → A3 magenta), tile grid with per-tile rung labels and `zmin/zmax` boxes, the widened-frustum cull-volume overlay (so "why did that rebuild fire?" is answerable), `Force rung: Auto / T0 / …`, **`Freeze LOD camera`** (fly away and inspect the selection as it *was* — the single most useful LOD debug control that exists), budget bars with the binding constraint highlighted, and sliders for `desired_pixel_error` / `hysteresis` / `max_props` / `max_tris` (all Tier 0).

**Stats** — `PlotLines` ring buffers, 120 samples, with `overlay_text` showing the current value: `derive ms`, `interpret ms`, `mesh ms`, `commit ms`, `project ms`, `frame ms`. Counts: modules, terminals, instances by rung, resident tiles, archetype bytes, **module visits/s** (the metric that tells you whether Tier 1 is holding). A `last evaluated` staleness marker. **Collection is gated on the panel being open** — Blender explicitly disables value logging during render, for exactly this reason.

### 9.5 The edit → regenerate loop

Pull-based dirty propagation over `params → word → terminals → {mesh, raster} → {props, planes}`. Each stage has its own dirty bit; most edits do not dirty stage 2.

| tier | triggers | work | budget | default |
|---|---|---|---|---|
| **−1 Validate** | every keystroke | parse only | < 1 ms, no debounce | always on |
| **0 Free** | colours, materials, wind gain, camera, LOD overlays, force-rung, layer visibility, splat albedo | actor properties / uniforms only | same frame | always on |
| **1 Continuous** | branch angle, segment length, radius, taper curve, tropism, roll/tilt, cylinder sides, **fractional generations**, `max_branch_order`, brush radius | re-run `interpret()` over the **cached word**; `fixed_mesh::commit()` | **≤ 16 ms, scoped to the specimen + the ≤ 24 A0 plants only** | **on by default** — it is genuinely free at that scope |
| **1b Project** | ontology ρ / hard edits, ontology variant swap, stamp add/delete/reorder | re-run the class→raster projection + overlay texture | ≤ 5 ms for 513² | on |
| **2 Debounced** | grammar text, integer generations, per-stream seeds, species mix, classification rules, marking grammars, island params | re-`derive()` + re-mesh + re-merge. Old mesh stays on screen. Cancellable. Ordered **focused specimen → near ring → far ring** | 180 ms debounce; near ring target ≤ 400 ms; **world convergence ≤ 6 s at 2 promotions/frame**, or press `Burst` | on |
| **3 Explicit** | `build_gen`, full-world bake, erosion, impostor atlas rebake, export, batch | the lot | seconds, spinner, progress | button only |

**Tier-1 scope is a hard bound, not a hope.** [lod-first R10]'s arithmetic is the reason: a 13 k-module tree at 60 Hz is ~800 k module visits/s. That is probably fine for one specimen; it is emphatically not fine for the world. PR L5 ships a **latency assertion** (`interpret()` over a cached word for the specimen + 24 plants < 16 ms) and a **pre-committed degradation**: if it misses, Tier 1 caps at 30 Hz, which is still continuous perceptually.

**World convergence is stated, not implied.** At 2 tile promotions/frame a T2 edit touching a common archetype takes ~300 frames to propagate across 600 visible tiles. The status strip shows a convergence bar, and `Run ▸ Burst` temporarily raises the cap to 16/frame, trading a visible hitch for coherence — which is the right trade while you are deliberately iterating rather than flying.

**Never blank the viewport.** Double-buffer: the old contents stay bound until the new terminal list is complete, then one `commit()`.

**Never `execv` to restart.** wasm has no exec and it silently no-ops. Regeneration is always an in-place rebuild against the same `SceneGraph`, `SceneRenderer`, GL context and camera.

Cache key: `hash(grammar_hash, params, seeds, generations)` → cached word. Gallery A/B and undo are therefore instant. A **pin** on a variant survives cache eviction.

### 9.6 Undo/redo

- **Global history**, 128 entries, spanning grammar text, params, seeds, ontology edits and paint strokes.
- Paint strokes are one entry per stroke (mouse-up batching), and *also* live as an ordered, individually-deletable, drag-reorderable `paint_op` list in the Surface tab — which is a structured, inspectable superset of linear undo.
- Undo is a **cached-state restore**, not a re-derivation, so `Ctrl+Z` is instant for anything already in the content-hash cache; only a cold-miss costs a Tier-2.
- A dice roll is undoable. Reseeding a stream is undoable. This is not optional: a researcher exploring a parameter space will hit `Ctrl+Z` within five minutes.

### 9.7 Presets, save/load, A/B

- **Save** and **Fork Variant…** are distinct verbs. Fork writes a child with `parent:` and, by default, **keeps editing the parent** — vlab's *Make extension* without *Move to new object*. Experimentation never silently mutates the preset you started from.
- **Gallery** = pinned snapshots, all cached. Click swaps the viewport instantly; Shift-click places both specimens in the scene at a fixed offset for side-by-side. This *is* the A/B feature; nobody in this space ships a textual diff view, and neither do we.
- Layout persisted via `SaveIniSettingsToMemory()` next to the model.

### 9.8 Native vs wasm

| | native | wasm |
|---|---|---|
| Grammar text | full edit, live parse | **read-only** + `Reload from file` (`EM_ASYNC_JS` fetch) + numeric rule-row editor (sliders only) |
| Painting / sliders / dice / gallery | full | full — mouse and 1-finger touch |
| Specimen preview | full | full |
| Derivation | 2-worker pool | `deriver::step(4 ms)` from the render loop + `emscripten_sleep(0)` |
| Restart / apply | in-place | **identical** |
| Export | writes the bundle to disk | writes to MEMFS, then `EM_JS` triggers a browser zip download |
| Batch sampler | yes | disabled |
| Panels | open | auto-hidden behind the floating circle below a 700 px short side; `uiScale` 2.0 on touch |
| Scale | `--preset large` / `standard` | `--preset wasm` forced |
| Capture | `--capture orbit\|fly` | forced `none`; `offscreen` forced `false` (a `writePNG` into MEMFS looks exactly like a hang) |

---

## 10. Headless / batch world generation

### 10.1 CLI

Two tools, both under `option(CVC_BUILD_LSYS_TOOLS)` — see §12.4 for why their placement matters to the coverage gate.

```
cvc-lsys derive   <recipe.lsys> [--seed N] [--gen K] [--stats] [--dump-word]
cvc-lsys svg      <recipe.lsys> --out tree.svg [--gen K] [--proj front|side|top]
                                [--width 900] [--colour order|level|class]
cvc-lsys mesh     <recipe.lsys> --out tree.off [--gen K] [--rung R]
cvc-lsys validate <recipe.lsys>          # parse + gen_nested + module-count report

cvc-worldgen build   --world archipelago.lsys --seed N --out world_a/
                     [--window cx cy half] [--cell 0.5] [--scale 0.05]
                     [--agent-radius 0.35] [--scene-kind outdoor|indoor|mixed]
                     [--window-policy single-island|forced-bridges|amphibious]
                     [--ontology merged_default] [--storeys 1]
                     [--preset standard] [--preview]
cvc-worldgen sample  --world archipelago.lsys --seeds 1000..1400 --n 200
                     [--bucket 3] [--min-detour 1.25] [--require-gate-fires]
                     [--out dataset/] [--jobs 8]
cvc-worldgen inspect <bundle/>           # print manifest + validation + difficulty
```

### 10.2 Reproducibility

A world is exactly `(master_seed, the sha256 of every .lsys in play, the tool version, the preset name)`. `provenance.json` records all of it, plus every gate report and every resample. `cvc-worldgen build` with the same inputs produces **byte-identical** `.npy` files — asserted by a CI test that builds the same world twice and compares.

Determinism holds across thread counts because every random draw is `hash4(master, stream, element, draw)` and no algorithm depends on iteration order. `cvc::nav` kernels are already built without `-ffast-math` / `-ffp-contract=fast` for bit-identity; `cvc::world`'s numerical routines adopt the same discipline and the same TU-level flag guard.

### 10.3 Curriculum and difficulty

`cvc-worldgen sample` generates candidates, runs the nav gate and the difficulty metrics, and keeps only those matching the filter. It prints live accept / reject / repair counts and writes a dataset index.

```cpp
// inc/cvc/world/metrics.h
struct difficulty {
  double risk_detour;      // len(cost-aware route) / len(blind route), >= 1.0
  double hard_frontage;    // fraction of the blind route within 2 m of a hard cell
  double clearance_p10;    // 10th-percentile clearance along the cost-aware route
  double risk_integral;    // mean risk along the cost-aware route
  int    bucket;           // 0..4, from risk_detour thresholds
};
difficulty measure(const raster_out&, const grid_spec&,
                   const double start[2], const double goal[2]);
```

Endpoints come from the **largest inflated-free component** at `inflate_m = 6.0`, matching `planner.far_pair_in_free_space` exactly, and both the selection method and the inflation are recorded in the manifest (§7.5). A generator that draws endpoints from raw free space produces a curated dataset with a silent tail of episodes that fail in the last stretch — which reads as a policy failure, not a data bug.

> `risk_detour` is an **unvalidated candidate label**. It is cheap, principled and computed from machinery that already exists, but whether a curriculum sorted by it improves training is an empirical question this design cannot answer. The manifest therefore carries enough raw statistics (class fractions, risk mean/σ, hard fraction, clearance percentiles, route lengths both ways) that a better label can be computed **retroactively without regenerating a single world**. Adding a statistic later invalidates the corpus; enumerate them now.

### 10.4 Dataset layout

```
dataset/
├── index.csv          # bundle, seed, bucket, risk_detour, hard_frontage,
│                      # class fractions, gate result, repairs, resamples
├── rejected.csv       # seed, reason  (gate exhaustion, filter miss, …)
├── ontology.json      # the ontology used for the whole run
├── run.json           # CLI invocation, tool version, libcvc commit, wall time
└── worlds/
    ├── w0000/  (a bundle, §7.5)
    ├── w0001/
    └── …
```

### 10.5 The consumer adapter

`grl_snam/world_bundle.py` (in the **GRL-SNAM repo**, not libcvc) — ~120 lines:

```python
def load_bundle(path, *, storey=0, ontology=None):
    """Read a cvcworld/N bundle -> (MaterialGrid, maps, meta, occupancy, endpoints).

    Asserts manifest['grid']['row_order'] == 'min_y_first' and flips EXACTLY
    ONCE here if a future format ever says otherwise.  Reconstructs cell_w the
    consumer's way -- (max_x-min_x)/(cols-1) -- and compares to the written
    value bit-for-bit.  Raises on mismatch; a mirrored or mis-scaled field must
    never load silently.
    """
```

It returns a `MaterialGrid(risk_raw, hard, bounds, center, scale, sigma=manifest["frame"]["sigma_recommended_cells"])` and the `meta` dict a `FogScenario` / `SdfNavigator` actually needs — with `meta` values in the **normalized** frame (`rr = 0.15`, `d_hat = 0.35`, `vmax = 0.9`, `dt = 0.06`, `nsub = 2`), not metres. The seam is a **file format**, not an ABI — which is why a signature change in `cvc::nav::material` (and it has already changed once relative to the circulated brief) cannot break a single generated bundle.

Three things the adapter must do and is tested on:

1. Assert `manifest["grid"]["row_order"] == "min_y_first"`, and flip exactly once if a future format ever says otherwise.
2. Reconstruct `cell_w` the consumer's way and compare **bit-for-bit** to the written `manifest["grid"]["cell_w"]`.
3. **Never read `manifest["consumer_frame_ref"]`.** A grep test enforces it (§12.3). That block exists so a bundle can be *diagnosed*, not configured.

---

## 11. Performance budgets & targets

### 11.1 Per-frame, native `standard`, 1280×800, shadows on

| stage | budget | notes |
|---|---|---|
| `processUIEvents` + touch + camera | 0.4 ms | |
| LOD select + budget solve | 0.15 ms | 1024 tiles + ~600 scatter cells, pure math |
| frustum + slab cull | 0.005 ms | flat grid |
| `fixed_mesh::commit()` | **0 ms steady state**, ≤ 4.4 ms on a page-in frame | capped at 4 swaps/frame |
| CPU sway (A0, 24 plants) | 0.3 ms | constant at every world size |
| sea near-field regen | 0.6 ms every 4th frame | z-independent term hoisted out of the k loop |
| cloud CA step | < 1 ms every 4th frame | 128×128×40 bit-packed |
| cloud shadow projection | 0 ms | fragment-shader sample of the CA coverage texture |
| ImGui | 0.5 ms | gated stats collection |
| shadow bake | 3.1 ms every 3rd frame | 2048², one spot, stage tracks the camera on an 8 m snap lattice |
| draw (opaque, ≤ 48 props, 1.6 M tris) | 8–11 ms | |
| draw (1 volume) | 6–9 ms | the documented two-volume ceiling is ~30 fps |
| **total** | **≈ 21 ms → ~47 fps** | target ≥ 45 fps |

### 11.2 Generation budgets

| operation | native | wasm | tier |
|---|---|---|---|
| parse a `.lsys` | < 0.5 ms | < 1.5 ms | −1 |
| derive `spiral_broadleaf` n=10 | 3.1 ms | 9 ms | 2 |
| derive `bush_ternary` n=7 (3891 F + 2607 L) | 4.4 ms | 13 ms | 2 |
| interpret one specimen (cached word) | 0.9 ms | 2.7 ms | 1 |
| bake 64 archetypes × 3 rungs | 1.4 s | 4 s (time-sliced) | 3, startup |
| terrain chunk T0 (65², 9 octaves) | 1.9 ms | 5 ms | 2 |
| terrain chunk T4 (5², 5 octaves) | 0.02 ms | 0.05 ms | 2 |
| class raster 513² (3 layers) | 4 ms | 12 ms | 1b/3 |
| projection only (ontology swap) | 1.8 ms | 5 ms | 1b |
| nav gate, one 64×64 storey | 0.7 ms | 2 ms | always |
| nav gate, 3-storey office | 12 ms | 34 ms | always |
| Lopes floor plan, simple | 60 ms | 180 ms | 3 |
| Lopes floor plan, complex | 900 ms | 2.6 s | 3 |
| erosion, one 1024² region (all 4 passes) | 0.55 s | 1.6 s | 3 |
| `.npy` bundle write (6 planes @ 513²) | 22 ms | 60 ms (MEMFS) | 3 |
| **full `cvc-worldgen build`** | **≈ 2.4 s** | n/a | — |
| **`cvc-worldgen sample --n 200 --jobs 8`** | **≈ 70 s** | n/a | — |

### 11.3 Memory

| | native `large` | native `standard` | wasm |
|---|---|---|---|
| archetype library | 180 MB | 96 MB | 22 MB |
| resident chunk meshes (`cvc::geometry`, 24 B/pt + 24 B/col + 24 B/tri) | 210 MB | 118 MB | 34 MB |
| VTK copies (float pts, uchar colours, 64-bit cells) | 240 MB | 134 MB | 39 MB |
| delta grid + hydrology | 20 MB | 20 MB | 5 MB |
| volumes (sea + cloud) | 3 MB | 3 MB | 1.4 MB |
| splat textures | 96 MB | 42 MB | 8 MB |
| **total resident** | **≈ 750 MB** | **≈ 415 MB** | **≈ 110 MB** |
| wasm module (`.wasm`, ASYNCIFY whole-program) | — | — | ≤ 26 MB uncompressed / ≤ 7 MB gzip |
| wasm heap ceiling | — | — | **2 GiB hard** (`getHeapMax() = 2147483648`) |

wasm sits at ~110 MB against a 2 GiB ceiling with ~18× margin, which is the right amount of margin given allocations must be contiguous in a heap that reallocs on every growth. A startup assert checks the estimate.

---

## 12. Testing & verification strategy

### 12.1 The oracle problem, solved by the literature

Golden files prove an engine is self-consistent. **Published module counts prove it is correct.** `lsys_recipes_test` asserts:

| recipe | assertion | source |
|---|---|---|
| ABOP Fig. 2.6 (Honda monopodial) | 63 / 255 / 1023 / 4095 `F` at n = 6 / 8 / 10 / 12 | [ABOP] |
| ABOP Fig. 2.7 (sympodial) | 63 / 255 / 1023 / 4095 `F` | [ABOP] |
| ABOP Fig. 2.8 (ternary) | `2·3ⁿ − 1`: **1457** at n=6, **13 121** at n=8 | [ABOP] |
| ABOP Fig. 1.24 d/e | **4118** `F` at n=7 | [ABOP] |
| ABOP Fig. 1.25 (bush) | **3891 `F` + 2607 `L`** at n=7; 2187 apices | [ABOP] |
| ABOP Fig. 5.11 (fern frond) | shape family (D, R) reproduces Table 5.2's five cases | [ABOP] |
| TOP94 L-system 6 | derivation lengths 3/6/9/13/21/27 terminate; `gen_nested == false` | [TOP94] |

If the engine is wrong, these fail. They cannot rot the way a self-generated snapshot can.

### 12.2 Determinism tests (the highest-value long-term suite)

1. **Order independence** — permuting production order in the file produces an identical word.
2. **Insertion stability** — adding an unrelated production, or a tree below sea level that gets skipped, leaves every other element byte-identical.
3. **Stream isolation** — the salt audit: salting `sway`, `phase`, `size`, `param_jitter`, `cloud` leaves `class_map`, `risk_raw`, `hard`, `occupancy` **bit-identical**. Runs in CI on every PR.
4. **Thread independence** — `cvc-worldgen build --jobs 1` and `--jobs 8` produce byte-identical bundles.
5. **Rebuild determinism** — building the same world twice produces byte-identical `.npy` files.
6. **No `std::mt19937`** — a grep test over `src/cvc/lsys/` and `src/cvc/world/`.

### 12.3 Contract tests

- **`.npy` round-trip** against a checked-in numpy-written fixture: header bytes and payload compared exactly, for `uint16`, `float32` and `uint8`.
- **Row-order test** — stamp an asymmetric L in *world* coordinates; assert array row 0 corresponds to `min_y`.
- **`cell_w` test** — reconstruct `cell_w` the consumer's way, `(max_x − min_x)/(cols − 1)`, and compare bit-for-bit to the manifest value.
- **Projection exactness** — `risk_raw[i] == registry[klass[i]].rho` and `hard[i] == registry[klass[i]].hard` for every cell of a randomized world, for all 32 classes and all 3 ontology variants.
- **`hard ⊆ occupancy`** over a randomized world.
- **Congruence** — `check_congruent` throws on any mismatch; every plane shares one `grid_spec`.
- **Versioning** — every `paint_op` and every ontology edit bumps the version counter.
- **Cross-language CI step** — a Python step loads a generated bundle, asserts dtypes / shapes / ranges / row order / `cell_w`, constructs `MaterialGrid(risk_raw, hard, bounds, center, scale, sigma=manifest["frame"]["sigma_recommended_cells"])` and asserts **`field().field.shape == (1, 6, 513, 513)`**. Note the double `.field`: `MaterialGrid.field()` (`material.py:276`) returns a **`MaterialField` object**, and the tensor is its `.field` attribute (`material.py:322-326`). Revision 1 asserted `field().shape`, which raises `AttributeError` — a test that fails for the wrong reason is not a contract test. **A contract tested only from the producing side is not tested.**
- **Both-arities test** — the same bundle is loaded through the Python path (`bounds` + `center` → derived `cell_w`) and through a C++ path calling `cvc::nav::material_build(risk_raw, hard, rows, cols, cell_w, scale, sigma)` with the manifest's explicit `cell_w`, and the two `[1,6,H,W]` stacks are compared **bit-for-bit**. This is the test that proves the manifest satisfies both entry points (§7.1).
- **`consumer_frame_ref` is never read** — a test greps `world_bundle.py` for the key and fails if it appears outside a comment. The block is forensics; the moment it becomes configuration, bundles start shipping stale tuning (§7.1a).
- **Hard-class projection** — `registry[k].hard == true ⇒ registry[k].rho == 0.0` for every class in every ontology variant (§7.2a), asserted over the shipped registry and any YAML-loaded one.
- **A\* cost test** — a route across a painted mud strip is longer in cells but lower in cost than the blind route, **with `simplify` skipped**; and the same route with `simplify` enabled is asserted to be *wrong*, so the trap is documented by a test.

### 12.4 The 80 % coverage gate

`COVERAGE_MIN: '80'` (`.github/workflows/ci.yml:17`), aggregate **line** coverage, enforced at `ci.yml:479-495` under `if: matrix.kind == 'libcvc' && matrix.build_type == 'Debug' && matrix.enable_grpc != true` by parsing `lcov --summary`. The allowlist is `$GITHUB_WORKSPACE/src/*` + `$GITHUB_WORKSPACE/inc/*` (`ci.yml:414-417`); the exclusions are `*/test/*`, `*/tests/*`, `*_test.cpp`, `src/xmlrpc/*` and `*/geometry/cvc-mesher/contour/*` (`ci.yml:422-427`). The job forces `cvcgl=OFF` when coverage is on (`ci.yml:295-298`), so `src/cvcGL/*` never enters the denominator. **Linux only** — macOS (`ci.yml:571`) and Windows (`ci.yml:752`) explicitly have no gate.

Design decisions taken *for* the gate:

- `cvc::lsys`, `cvc::world` and `cvc::lod` are **strictly GL-free and I/O-light**. `stats_emitter` exercises every derivation and interpretation path with zero geometry. `select.h` and `expr.h` are pure functions.
- `bundle.cpp` and every other I/O leaf takes an **injected `write_fn`** so disk-full and permission-denied paths are simulable. This is a module-wide rule from the first PR, not a retrofit for one file.
- **The CLI `main()`s (`cvc-lsys`, `cvc-worldgen`) live in `src/cvc/tools/` behind `option(CVC_BUILD_LSYS_TOOLS)` and are added to the lcov exclusion list**, alongside the existing `src/xmlrpc` and mesher-contour-tree exclusions. Argument parsing, usage text and error-exit paths are the hardest lines in the change to cover and there is no reason to put them in the denominator. Adding one line to the lcov `--remove` set is a shared-file edit to `ci.yml` and is called out in §14.
- **Every PR ships its tests in the same PR.** Never a "tests later" PR. Each PR description carries a locally-measured `lcov` number for its own files, gate ≥ 88 %.
- **The current repo-wide baseline must be measured and recorded before PR L0 lands**, because the gate is an aggregate: "88 % on my files" is only sufficient if the existing number is known.

### 12.5 Where tests live — a repaired mistake

> **`CVC_BUILD_EXAMPLES` defaults `OFF` (`src/cvcGL/CMakeLists.txt:290`) and no workflow YAML sets it** — `grep -rn CVC_BUILD_EXAMPLES .github/` returns **zero hits**. A test registered in `src/cvcGL/examples/CMakeLists.txt` — including the existing `nav_common_test` at `:92-97` — **never builds and never runs in any PR job.** The repo has already been burned by this exact class of failure: the registration-side drift guard at `src/cvc/tests/CMakeLists.txt:1135` exists because *"nav_coef_train_test compiled green for weeks while ctest ran none of its cases."*
>
> **The one nuance, stated precisely because revision 1 over-claimed.** Examples *are* compiled in CI — for **wasm only, post-merge**. `deploy-pages.yml:51` runs `./src/cvcGL/examples/wasm/build-wasm-demo.sh --pthread`, and that script sets `-DCVC_BUILD_EXAMPLES=ON` (`build-wasm-demo.sh:64`). The same is true of the `cvcgl-examples` cvcpkg recipe (`build.sh:32`, `build.ps1:27`, `build-wasm.sh:39`) driven by `publish-cvcpkg.yml`. Neither is a PR gate: `deploy-pages.yml` triggers on **push to master**, a nightly cron, and dispatch. So a *native* build break in an example still lands silently on master and blocks no PR — the conclusion is unchanged, but "never compiled in CI" was imprecise and "OFF in every CI job" was wrong.
>
> **`nav_common_test` still never runs anywhere.** In the one workflow that builds examples, `build-wasm-demo.sh:50` sets `-DCVC_BUILD_TESTS=OFF`, so `GTest::gtest` is not a target, the `if(TARGET GTest::gtest)` guard at `:92` is false — and that workflow runs no `ctest` at all. The file is real (128 lines, 7 `TEST(` cases) and is dead weight. **Whether it would pass if enabled is UNKNOWN**, and that unknown is exactly what makes D5 (§15.3) a real decision rather than a formality.

Consequently:

| test kind | lives in | registered via | runs in PR CI? |
|---|---|---|---|
| `cvc::lsys` / `cvc::world` / `cvc::lod` unit tests | `src/cvc/tests/lsys_*.cpp`, `world_*.cpp`, `lod_*.cpp` | `include(lsys_tests.cmake)` — **one line** before the `TEST_TARGETS` drift guard at **1281** | **yes** |
| cvcGL-touching tests (prop sweep, shader-compile smoke) | `src/cvcGL/test/cvcgl_*.cpp` | append at **EOF of `src/cvcGL/CMakeLists.txt`** (after line 293) | **yes** — **16** `cvcgl_*` tests already run under Xvfb + llvmpipe, with a guard at `ci.yml:334-342` that errors if zero reach CTest |
| Lab UI *logic* tests | `src/cvc/tests/lab_dispatch_test.cpp` | same include | **yes** |
| example smoke (`lsystem_lab --offscreen --frames 8`) | `src/cvcGL/examples/` | append after line **97** (EOF) | **no** — flagged as D5 in §15.3 |

Because the include is textual in the same directory scope, new test targets land **inside** `BUILDSYSTEM_TARGETS` (the property the guard at `src/cvc/tests/CMakeLists.txt:1295` reads) and are swept by *both* existing drift guards — the build-side one at 1281 and the registration-side one at 1135. The new module inherits the repo's own protection against tests that compile green and never run.

### 12.6 UI logic is tested; ImGui is not

The four-tier dispatcher, the 180 ms debounce, the cancellation generation counter and the content-hash cache invalidation are **pure logic** and are exactly where a subtle bug hides (a Tier-2 cancel racing a Tier-1 drag; an edit that should be Tier 1 quietly becoming Tier 2). They are extracted behind a plain interface and unit-tested headlessly with no ImGui, in `src/cvc/tests/`.

What the widget PR adds on top: an **offscreen capture test** — because `ImGuiOverlay` draws inside VTK's render pass, `lsystem_lab --offscreen --capture orbit --png` produces a PNG *of the panels*, which is a real regression artifact and doubles as documentation screenshots.

### 12.7 Performance regression

- `cvcgl_prop_sweep` (§8.7) records an fps-vs-prop-count table on the target machine and writes it into `LSYSTEM_LAB.md`.
- Every render PR records measured fps at 1280×800 shadows-on next to the `lsystem_forest` baseline (24.8 fps), so the comparison is always available and always honest.
- Derivation and raster timings are asserted with generous ceilings (3× the §11.2 budget) so the tests catch order-of-magnitude regressions without being flaky.

### 12.8 Visual smoke

- **Depth-interaction capture test** — a 1420 m peak must occlude the cloud volume from a camera at 900 m. This is the test that retires §15 R2.
- **Sun visibility test** — sample the sun billboard's screen position from 20 camera poses across an 8 km world; it must be visible from all of them.
- **Crack test** — no `|Δlevel| > 1` between neighbouring terrain chunks, over 500 random camera poses.
- **Band-limiting test** — a coarse chunk's height spectrum has no energy above its Nyquist.
- **wasm shader-compile smoke** — the sway replacement links under GLES3 *and* a probe vertex actually moves (the `tcoordMC` trap, §8.2).

---

## 13. Implementation plan

Twelve PRs. Each independently mergeable, **each demoable**, each with its tests in the same PR.

### 13.1 Making PR 1 something you can look at

> **The defect this repairs.** The fixed decision is *"a design doc AND a buildable first PR"* for a **visual** laboratory. Revision 1's PR 1 (`L0a`) demoed as `ctest -R lsys green` and a byte-exact file round-trip. That is a good library PR and a bad first PR for this project: nobody can see it.

Three constraints make "PR 1 is the ImGui lab" impossible, and they are all real:

1. **The coverage gate is aggregate.** ~11 000 new lines under `src/` + `inc/` enter an 80 % line denominator. A first PR that is mostly a cvcGL example ships library code with no tests attached to it and drags the aggregate down immediately (§12.4).
2. **A cvcGL example is not built in PR CI at all** (§12.5). An examples-first PR 1 would deliver a demo that no job compiles, let alone runs.
3. **The example needs terrain to render**, and terrain needs the noise stack, the tile grid and the LOD selector — that is `L2`'s worth of work sitting under any pixel.

So PR 1 stays a library PR, and **the library is given a visual output that costs ~180 lines and adds no dependency**:

```
cvc-lsys svg  <recipe.lsys> --gen 10 --out pine.svg [--proj front|side|top]
                            [--width 900] [--colour order|level|class]
cvc-lsys mesh <recipe.lsys> --gen 10 --out pine.off
```

`cvc-lsys svg` runs `derive()` → `interpret()` into a **new fourth emitter, `svg_emitter`**, which orthographically projects each `terminal` (cylinders as tapered quads, cards as triangles, polygons as paths) and writes a self-contained SVG. It is:

- **pure text** — no ImageMagick, no PNG codec, no GL, no VTK, and therefore no new dependency and no platform variance;
- **openable in any browser**, so a PR reviewer clicks the file in the GitHub diff and sees the tree;
- **trivially testable** — the SVG's element count is a deterministic function of the module count, so `lsys_svg_test` asserts the *published* module counts of §12.1 twice over, once through `stats_emitter` and once through the rendered element count;
- **fully in the coverage denominator and cheap to cover**, unlike anything that touches a GL context.

`--colour class` colours by `terminal::surface_class`, which makes the *material* half of the design visible in PR 1 as well.

PR L1 gets the same treatment one level up: `cvc-worldgen build --preview` writes `class_preview.png`, `risk_preview.png` and `hard_preview.png` through `cvc::image` (`inc/cvc/image/image.h`, `write_image(img, path)` — already in `libcvc`, already linked, no new dependency), falling back to binary PPM when no PNG handler is registered. So **`cvc-worldgen build` produces something you look at, not only something you `np.load`.**

The first *interactive* deliverable is still L3, unchanged, and that is stated rather than glossed: PR 1 is visual, not interactive, and the reason is the coverage gate plus the test-placement rule, not an oversight.

### 13.2 The PR table

| # | PR | Scope | New files | Shared files | Size | Demoable outcome | Depends on |
|---|---|---|---|---|---|---|---|
| **L0** | `cvc::lsys` — the whole engine, **and PR 1 is visual** | hashed RNG, expression VM, symbol table, parser, `.lsys` reader/writer, diagnostics, both derivation modes, containment policy, context provider, `word`, `filter_level`, **`gen_nested` detection**, **four** emitters (`stats`, `svg`, `mesh`, `raster`-stub), `deriver` (resumable), `cvc-lsys` CLI with `svg` / `mesh` / `derive` / `validate` | `inc/cvc/lsys/{rng,expr,symbol,grammar,parse,io,module,derive,interp,scope,recipes,svg}.h` + srcs; `src/cvc/lsys/lsys.cmake`; `src/cvc/tools/cvc_lsys_main.cpp`; `src/cvc/tests/{lsys_rng,lsys_expr,lsys_parse,lsys_derive,lsys_interp,lsys_recipes,lsys_svg}_test.cpp`; `src/cvc/tests/lsys_tests.cmake`; `src/cvc/tests/data/recipes/*.lsys` | `src/cvc/CMakeLists.txt` **+1 line** before `add_library(cvc` at **841**; `src/cvc/tests/CMakeLists.txt` **+1 line** before the `TEST_TARGETS` drift guard at **1281** | ~2 900 + 2 000 test | **`cvc-lsys svg pine_monopodial.lsys --gen 10 --out pine.svg` — open it in the PR diff and look at the tree.** Plus `--out tree.off`, `ctest -R lsys` green, byte-exact `.lsys` round-trip incl. comments, and **all published module counts assert** | — |
| **L1** | `cvc::world` surfaces + bundle | 32-class registry + 3 ontology variants (**hard ⇒ ρ = 0**, §7.2a), `grid_spec`, three-layer `raster()`, `heightfield`, `.npy` writer, `bundle` with the `frame` + `consumer_frame_ref` blocks (§7.1a), `cells.json` schema `cvcworld.cells/1` incl. `opaque` + `footprint` (§6b.6), `cvc-worldgen build --preview`. **Completely headless.** | `inc/cvc/world/{units,grid,surface,raster,heightfield,bundle,cells,npy,preview}.h` + srcs; `src/cvc/world/world.cmake`; `world_{surface,raster,bundle,cells}_test.cpp`; `data/golden/tiny_bundle/`; python CI step | **none** (both includes exist) | ~2 300 + 1 600 test | **`cvc-worldgen build --preview` writes a loadable bundle *and* `class_preview.png` / `risk_preview.png`; the Python CI step constructs `MaterialGrid` and asserts `field().field.shape == (1,6,513,513)`; a C++ step calls `material_build` on the same bytes and compares bit-for-bit. This PR unblocks GRL-SNAM.** | L0 |
| **L2** | terrain, **the archipelago**, tiling, LOD math | ridged multifractal + warp + masks, **smooth-max island combination + seeded placement + biome table (§4.3a)**, octave band-limiting, erosion passes, hydrology, tile grid, **`validate_outdoor` (§7.8)**, `cvc::lod::select` | `inc/cvc/world/{terrain,island,archipelago,biome,tile,scatter}.h`, `inc/cvc/lod/select.h` + srcs; `world_{terrain,archipelago,outdoor_gate}_test.cpp`, `lod_select_test.cpp` | **none** | ~2 200 + 1 300 test | `cvc-worldgen build --preset large --preview` produces a **4-island** world you can look at as a PNG; smooth-max exactness, placement determinism, band-limit, hysteresis and **outdoor-gate** tests green | L1 |
| **L3** | `lsystem_lab` skeleton + terrain render + **the measurement** | the example, `fixed_mesh`, chunk actors, frustum cull, camera/HUD/touch, ImGui shell (World/LOD/Stats tabs), shadow stage tracking, `cvcgl_prop_sweep` | `src/cvcGL/examples/lsystem_lab.cpp`, `lsyslab_render.{h,cpp}`, `lsyslab_ui.{h,cpp}`, `LSYSTEM_LAB.md`; `src/cvcGL/test/cvcgl_prop_sweep.cpp` | `src/cvcGL/examples/CMakeLists.txt` **append after line 97 (EOF)**; `src/cvcGL/CMakeLists.txt` **append after line 293 (EOF)** | ~2 200 + 300 test | Fly a 4 km archipelago at ≥ 45 fps, rungs recolour, budget gauge live. **`max_props` default is set from the sweep, not from this doc.** | L2 |
| **L4** | vegetation at scale + **GPU sway** | archetype library, stateless-hash placement, merged chunk actors, impostor atlas, budget solver wired to the loop, the A0–A4 animation ladder, the `tcoordMC` declaration fix + wasm shader smoke test | `inc/cvc/world/{archetype,mesh_emit}.h` + srcs; `world_archetype_test.cpp`; `src/cvcGL/test/cvcgl_sway_shader.cpp` | `src/cvcGL/CMakeLists.txt` (same EOF block) | ~1 700 + 800 test | **480 k plants at ≥ 45 fps with shadows on, and a wind slider that moves every visible tree at zero CPU cost beyond 60 m** | L3 |
| **L5** | the Laboratory widget | all seven tabs, four-tier loop + Tier 1b, surface painting, undo/redo, seed padlocks + salt audit UI, gallery/fork, specimen preview, LOD debug, profiling, contract preview | `lsyslab_ui_*.cpp`; `src/cvc/tests/lab_dispatch_test.cpp` | **none** | ~2 400 + 400 test | Edit a production → forest regenerates in 180 ms without blanking; paint mud → `risk_raw` changes in the contract preview; export | L4 |
| **L6** | interiors | scope-grammar shells, Lopes growth, BSP fallback, cells/portals/links, **`derive_clearances(agent_radius_m, grid_m)` (§6b.1a)**, the nav gate with repair→resample→loud-reject, stair dispatch, prop placer with circulation keep-out, indoor materials end to end | `inc/cvc/world/{building,floorplan,clearance}.h` + srcs; `world_{cells,floorplan,navgate,clearance}_test.cpp` | **none** | ~2 200 + 1 400 test | Walk into a 3-storey office; every room reachable; **500-interior gate sweep asserts 100 % pass after repair**, at `agent_radius_m` ∈ {0.35, 3.0} | L5 |
| **L7** | clouds, sea, sun at scale | Dobashi CA, hash-tiled world-space cloud field, shell deck, projected cloud shadow, near/far sea, sun fixes | `inc/cvc/world/{cloud,sea,sky}.h` + srcs; `world_cloud_test.cpp`; `src/cvcGL/test/cvcgl_volume_depth.cpp` | `src/cvcGL/CMakeLists.txt` (same EOF block) | ~1 000 + 500 test | **Stand on a 1420 m summit above the deck; fly down through it.** Depth-interaction capture test retires R2 | L4 |
| **L8** | batch generation + curriculum | `cvc-worldgen sample`, difficulty metrics, endpoint selection matching `far_pair_in_free_space`, dataset index, `rejected.csv`, `--jobs` | `inc/cvc/world/metrics.h` + src; `src/cvc/tools/cvc_worldgen_main.cpp`; `world_metrics_test.cpp` | `ci.yml` **+1 line** to the lcov `--remove` set for `src/cvc/tools/*` | ~900 + 600 test | `cvc-worldgen sample --n 200 --bucket 3 --min-detour 1.25` produces a curated dataset with an index | L6 |
| **L9** | wasm reduced preset | `world_preset::wasm()`, read-only grammar path, time-sliced deriver, MEMFS export + browser download, gallery registration | `lsyslab_wasm.cpp`; a gallery thumbnail | `src/cvcGL/examples/CMakeLists.txt` **line 41** (`_wasm_demos`) — **now COLD; #229 merged**; `.github/workflows/deploy-pages.yml` **line 38** (`DEMOS:`) — required for the demo to actually deploy; `wasm/demos.json` (cold, optional) | ~500 | The Lab in a browser at `transfix.github.io/libcvc/lsystem_lab/` | L7 |
| **L10** | GRL-SNAM adapter (**different repo**) | `grl_snam/world_bundle.py` + `--bundle` on the training entry point | — | none in libcvc | ~140 + 100 test | A training run consumes a laboratory bundle end to end | L8 |
| **L11** | pycvc bindings | SWIG `cvc::lsys::derive`, `cvc::world::raster`, the export path | `bindings/pycvc/lsys.i`, `world.i` | `bindings/pycvc/*.i` — warm (the material work touched them, and it is now **merged**), so this is ordering discipline rather than a blocker | ~400 + 300 test | In-process world generation from a training loop | L8 |

**Ordering:** L0 → L1 → L2 → L3 → L4 → (L5 ∥ L7) → L6 → L8 → (L9 ∥ L10) → L11.

**L1 is the earliest point at which GRL-SNAM gets value.** L4 is the earliest point at which the "much larger scope" claim is visibly true. **L0 is the earliest point at which anyone sees a picture** (§13.1).

**PR L0 is the foundational PR** the fixed constraints require: no GL, no VTK, no nav coupling, two one-line CMake inserts, `ctest -R lsys` green, published module counts asserted, a byte-exact round-trip — **and an SVG of a tree in the diff.** It is larger than revision 1's split L0a/L0b (≈ 4 900 lines with tests, versus 2 100 + 2 800), and that is a deliberate trade: an `L0a` that parses `.lsys` but cannot derive anything has no visual output *available* to it, so the split guaranteed a first PR nobody could see. If review size becomes the binding constraint, the correct split is **L0a = engine + `stats_emitter` + `svg_emitter` + CLI** (still visual) and **L0b = `mesh_emitter` + `scope`/building modes**, not the parse/derive split.

---

## 14. Collision avoidance with concurrent work

### 14.1 What is live (re-verified 2026-08-27, post-rebase)

Baseline `origin/master` = **`8b6f426`**. `gh pr list --state open --repo transfix/libcvc` returns **two** PRs:

| PR | branch | opened | owns |
|---|---|---|---|
| **#223** | `fix/cvcgl-caster-truth-and-pan-state` | 2026-08-22 | `inc/cvc/gl/CameraController.h`, `src/cvcGL/CameraController.cpp`, `src/cvcGL/StageLighting.cpp`, `src/cvcGL/CMakeLists.txt` **~24 and ~281-287** |
| #200 | `fix/pin-windows-2022-in-release` | 2026-08-19 | `.github/workflows/release.yml` |

**Three things revision 1 treated as pending have merged and are gone:**

| commit | PR | what it means for us |
|---|---|---|
| `a33851f` | **#229** | wasm imagemagick + assimp + Austin preload. `src/cvcGL/examples/CMakeLists.txt` is now **97 lines**, and `_wasm_demos` (line 41) / `_cvcgl_example_bins` (line **82**, not 69) are **cold**. The "L9 waits for #229" hazard is **retired**. |
| `e97d06c` | **#231** | `build-wasm-demo.sh` matched to the recipe. |
| `8b6f426` | **#230** | `cvc::nav` material-aware navigation. `inc/cvc/nav/material.h` and `src/cvc/nav/material.cpp` are **in the tree**, stable, and tested. `bindings/pycvc/*.i` is no longer contended. §7.1 and §7.1a are written against the merged header, not a description of it. |

**#223 is the one live collision.** It owns hunks at `src/cvcGL/CMakeLists.txt` ~24 and ~281-287, and line 283 is `add_test(NAME cvcgl_shadow_caster_growth …)` — i.e. it genuinely owns the region immediately above our EOF append point at 293.

### 14.2 The shared-file matrix, re-derived against the current files

| file | touched by | nature | rationale |
|---|---|---|---|
| `src/cvc/CMakeLists.txt` | L0 **only** | **+1 line**: `include(lsys/lsys.cmake)` immediately before `add_library(cvc ${SOURCE_FILES} ${INCLUDE_FILES})` at **841** | The `.cmake` fragments do `list(APPEND SOURCE_FILES …)` / `INCLUDE_FILES`, so L1/L2/… grow the fragment, not the shared file. The nav block #230 edited is ~700 lines above. |
| `src/cvc/tests/CMakeLists.txt` | L0 **only** | **+1 line**: `include(lsys_tests.cmake)` immediately before `# ── TEST_TARGETS drift guard ──` at **1281** | `cvc_discover_tests` is defined at **1143** and both guards run after 1281, so `add_executable` + `target_link_libraries` + `list(APPEND TEST_TARGETS)` + `cvc_discover_tests` are all legal there — **and the new targets land inside `BUILDSYSTEM_TARGETS` (read at 1295), so they inherit both the never-built and the never-registered guards.** |
| `src/cvcGL/CMakeLists.txt` | L3, L4, L7 | **append after line 293 (EOF)** | Sources are `file(GLOB *.cpp)` at :35 and headers `install(DIRECTORY)` at :138, so new cvcGL source/header files need **no** CMake edit at all — only the new `add_executable`/`add_test` block does. #223's last hunk ends ~287, six lines clear. |
| `src/cvcGL/examples/CMakeLists.txt` | L3 | **append after line 97 (EOF)**: `add_executable(lsystem_lab lsystem_lab.cpp)` + `target_link_libraries(… cvcGL ${VTK_LIBRARIES} Boost::program_options)` + `vtk_module_autoinit(…)` | **There is no GLOB in `examples/`** — a new example is invisible without this block. It does not have to touch any existing list, and nothing else is queued behind it. (If it wants the nav helpers it also links `nav_demo_common`, defined at :25.) |
| `src/cvcGL/examples/CMakeLists.txt` **line 41** | **L9 only** | one name appended to `_wasm_demos` | **Now cold — #229 merged.** This single edit is all a wasm demo needs: `add_custom_target(wasm-demos DEPENDS ${_wasm_demos})` (:73) and `wasm/build-pages.py` auto-discovery follow, and the per-demo `target_link_options` loop (:52-70) applies automatically. |
| `src/cvcGL/examples/CMakeLists.txt` **line 82** | **L3 or L9**, only if D6 says "package" | one name appended to `_cvcgl_example_bins` | That single list feeds **both** `set_target_properties(… INSTALL_RPATH "$ORIGIN/../lib" …)` (:83-85) **and** `install(TARGETS … COMPONENT cvcgl-examples)` (:86-87). Omitting it means `lsystem_lab` builds but is neither RPATH-fixed nor installed. |
| `.github/workflows/deploy-pages.yml` **line 38** | **L9 only** | one name appended to `DEMOS:` | Used at `:67` and `:123` to assemble and verify the gallery. Without it the wasm Lab builds on catx-03 and is **not deployed**. Revision 1 missed this file entirely. |
| `.github/workflows/ci.yml` | L8 | **+1 line** to the lcov `--remove` set (`ci.yml:422-427`) for `src/cvc/tools/*` | Cold; the set already excludes `src/xmlrpc/*` and the mesher contour tree. |
| `src/cvcGL/examples/wasm/demos.json` | L9 | one string + one object | Cold; **optional** — `build-pages.py` discovers demos by scanning `bin/` for a `.js`+`.wasm` pair, so an unlisted demo still appears with defaults. |
| `bindings/pycvc/*.i` | **L11 only** | new `.i` files + registration | Warm rather than hot now that #230 has merged. Still strictly last, because SWIG interface files are the least pleasant thing in this repo to rebase. |
| `inc/cvc/gl/GeometryNode.h`, `src/cvcGL/GeometryNode.cpp` | **never** | — | The fixed-capacity mesh needs no cvcGL API change (§8.6). This is why. |
| `src/cvcGL/examples/lsystem_forest.cpp`, `examples/README.md` |**never** | — | Fixed user constraint. `lsystem_forest` stays as the fast smoke test and as the performance control for every measurement in this plan. |
| `src/cvc/nav/**`, `inc/cvc/nav/**` | **never** | — | We only ever **read** this namespace (§14.4). We add nothing to it, and `cvc::world` does not link it at all. |
| `docs/roadmap/**` | this document | new file | Cold. |

**The true shared-file cost.**

| edit | PR | required? |
|---|---|---|
| `src/cvc/CMakeLists.txt:841` +1 | L0 | yes |
| `src/cvc/tests/CMakeLists.txt:1281` +1 | L0 | yes |
| `.github/workflows/ci.yml:422-427` +1 | L8 | yes |
| `src/cvcGL/examples/CMakeLists.txt:41` (`_wasm_demos`) +1 name | L9 | yes, for a browser Lab |
| `src/cvcGL/examples/CMakeLists.txt:82` (`_cvcgl_example_bins`) +1 name | L3/L9 | **only if D6 = package** |
| `.github/workflows/deploy-pages.yml:38` (`DEMOS`) +1 name | L9 | **only if the wasm Lab is to be deployed** |

**So: four pre-existing lines if the Lab is native-only-and-unpackaged; six if it is packaged and deployed.** Revision 1 said "four" and got there partly by luck — it counted the `_wasm_demos` edit, missed `_cvcgl_example_bins` and `deploy-pages.yml`, and used pre-`#229` line numbers throughout.

Plus **two** append-at-EOF blocks (`src/cvcGL/CMakeLists.txt` after 293, `src/cvcGL/examples/CMakeLists.txt` after 97).

> **Correction: "append at EOF cannot textually conflict" is false.** Two branches appending at the same EOF is the *canonical* three-way-merge conflict — git has no context after the last line to disambiguate with. The accurate claim is narrower and still useful: **appending at EOF cannot conflict with an edit that is not itself at EOF**, so our blocks are safe against #223's ~24 / ~281-287 hunks and against everything else currently open. They *will* conflict with each other if L3, L4 and L7 are ever developed in parallel rather than sequenced, and the resolution ("keep both hunks, in either order") is mechanical but not free. Sequence them, and re-check at every rebase.

### 14.3 Protocol

1. **Work in `wt-libcvc-lsyslab`**, branched off `origin/master` @ **`8b6f426`**. One worktree per session; never share a checkout.
2. **Rebase early and often.** The two one-line includes are small and mechanically re-appliable if they ever do conflict. Re-run the line-number checks in §14.2 after every rebase — this document has now been wrong about them once.
3. **Never** register a test in `src/cvcGL/CMakeLists.txt` near line 281 (#223's zone).
4. **Sequence the EOF appends.** L3 → L4 → L7 in `src/cvcGL/CMakeLists.txt`; L3 → L9 in `src/cvcGL/examples/CMakeLists.txt`. See the correction above.
5. If a new shared-file need appears mid-implementation, it goes in the PR description's shared-file table *before* the code is written, not after.

### 14.4 Do we consume the C++ API, the file format, or both? — **Both, at different layers**

> **The question revision 1 answered too bluntly.** Its rule was *"never call into `cvc::nav::material`."* That was the right instinct against an **unmerged, moving** header. `inc/cvc/nav/material.h` is now merged, stable, documented and covered by `nav_material_test`, so the rule deserves re-deciding rather than inheriting.

**The decision, in two halves:**

**(a) The bundle contract stays a file format. `cvc::world` never links `cvc::nav`.** Unchanged, and for unchanged reasons: we write `risk_raw` and `hard` as `.npy` plus a manifest, and nothing derived. If `material_build`'s blur, EDT convention, gradient normalisation, channel order or signature changes, **not one generated bundle is invalidated**. That property is worth more than any convenience, it is the reason a corpus has a long half-life, and it is enforced structurally — `world.cmake` does not add `cvc/nav` to anything, and there is a link-time test that `libcvc`'s world objects reference no `cvc::nav::` symbol.

**(b) The Lab's preview overlays call `cvc::nav::material` directly, read-only.** `lsystem_lab` — the *demo*, not the library — calls `material_build(risk_raw, hard, rows, cols, cell_w, scale, sigma)` and `witness_gate(...)` to draw the Surface tab's `phi_hard` preview, the risk-detour route and the gate-fires indicator. Three reasons:

1. **A preview that is not the consumer's arithmetic is a lie.** The whole value of the "contract preview" (§9.4) is that it shows what the trainer will see. Approximating it would make it a decoration.
2. **The alternative is a second copy of a bit-identity function.** `material_build` exists specifically to be bit-identical to `MaterialGrid._derive` — pinned op order, `-ffp-contract=off`, a scipy-'reflect'-equivalent separable Gaussian. Reimplementing that inside `cvc::world` would create a second implementation whose entire specification is "match the first one", and it would drift.
3. **It costs nothing to link.** `cvcGL` already PUBLIC-links `cvc::cvc`, which is where `cvc::nav` lives — the `nav_demo_common` comment in `examples/CMakeLists.txt:22-24` says so explicitly. There is no new dependency edge; the demos already have it.

**The rule, stated so it can be enforced:**

```
cvc::world  (library, export path)  ->  MUST NOT reference cvc::nav.  Link-tested.
lsystem_lab (demo, preview only)    ->  MAY call cvc::nav::material READ-ONLY.
Nothing                              ->  writes to cvc::nav, or adds to that namespace.
```

And one test that makes the seam honest in both directions: **the Lab's in-memory preview planes and a `material_build` call on the *exported bytes* must agree bit-for-bit.** With (b) in place that test is nearly trivial — which is the point. Under revision 1's rule it was impossible to write at all.

---

## 15. Risks & open questions

### 15.1 Risks, ordered by probability × damage

**R1 — Silent mirroring or mis-scaling of the exported grid.**
*Probability: high if unguarded. Damage: severe and undetectable.* Row 0 = `min_y` here; the research BEV builder is `max_y`-first. And `cell_w = extent/(n−1)`, not `extent/n` — an off-by-one that systematically mis-scales `phi_hard_m` (world metres) against `d_hat_sdf_m` and `hard_margin_m` in every world ever generated.
*Defence:* four independent layers (§7.6) — compile-time `row_order`, manifest field, reader-side assert, asymmetric L fiducial — plus a `cell_w` reconstruction test that computes it the consumer's way and compares bit-for-bit. The default window is 513×513 spanning exactly 256.0 m so `cell_w` is exactly 0.5.

**R2 — Mountains may not correctly occlude the cloud volume.**
*Probability: medium. Damage: high (it is the headline visual).* The pass chain runs volumetric after opaque and `vtkGPUVolumeRayCastMapper` terminates against the opaque depth buffer, so this *should* work — but it has never been exercised with 500 m of mountain inside a cloud slab.
*Defence:* PR L7 ships a depth-interaction screenshot test and this is the item it retires. *Fallback ladder, each a node swap not a redesign:* half-resolution raycast with upsample → 8–12 depth-tested translucent shells (`< 0.3 ms`, trivially correct) → soft billboards. The fallback is cheap and known-good, so the risk is bounded.

**R3 — The prop budget is an extrapolation.**
*Probability: certain that 48 is wrong by some margin. Damage: medium.* The "63-actor cliff" is two confounded data points (§8.7).
*Defence:* `cvcgl_prop_sweep` in PR L3 measures fps against prop count × per-prop triangles × shadow participation × per-frame upload, on the target machine, and **sets the default from the measurement**. Merge granularity (which bands share an actor) is a data parameter, not a structural commitment, so if the real limit is 30 we merge A1+A2 and T0+T1 with no code change.

**R4 — GPU sway silently does nothing.**
*Probability: high without the fix; low with it. Damage: high (it is the entire answer to the measured bottleneck).* `vtkOpenGLPolyDataMapper::ReplaceShaderTCoord` early-returns when no texture is bound, so `in vec2 tcoordMC;` is never declared for an untextured vegetation mesh and the shader compiles cleanly with no sway.
*Defence:* the replacement injects its own declaration at `//VTK::PositionVC::Dec`; a 1×1 dummy texture is the belt-and-braces alternative; and `cvcgl_sway_shader` asserts the program links **and that a probe vertex actually moves**, on both desktop GL and GLES3.

**R5 — Tests that never run.**
*Probability: high in any design that forgets it. Damage: catastrophic and invisible.* `CVC_BUILD_EXAMPLES` defaults `OFF` and **no workflow sets it**; a test registered in `examples/` runs in no PR job. The one place examples *are* compiled — `deploy-pages.yml` → `build-wasm-demo.sh`, wasm-only, post-merge, on catx-03 — sets `-DCVC_BUILD_TESTS=OFF` and runs no `ctest`, so `nav_common_test` has still never executed anywhere. A **native** break in an example therefore lands silently on master and blocks no PR. (Revision 1's phrasing — "OFF in every CI job", "never compiled in CI" — was absolute and wrong in detail; the conclusion is unchanged.)
*Defence:* §12.5's placement table is a hard rule. All library tests go through the `src/cvc/tests/` include, which lands inside `BUILDSYSTEM_TARGETS` and inherits **both** of the repo's drift guards. cvcGL tests go after line 293 of `src/cvcGL/CMakeLists.txt`, where **16** `cvcgl_*` tests already run under Xvfb + llvmpipe with a guard at `ci.yml:334-342` that errors if zero reach CTest.

**R6 — The coverage gate.**
*Probability: medium. Damage: blocking.* ~11 000 new lines under `src/` + `inc/` enter an **aggregate** 80 % denominator.
*Defence:* the modules are GL-free and emitter-driven by design; `stats_emitter` exercises every derivation path with no geometry; `select.h` and `expr.h` are pure; I/O leaves take an injected `write_fn`; the CLI `main()`s are excluded via the lcov `--remove` set. Every PR ships its tests, gate ≥ 88 % on its own files. **The repo-wide baseline is measured and recorded before L0 lands.**
*Residual:* `floorplan.cpp`'s repair heuristics are branchy and are the hardest legitimate lines in the change.

**R7 — `gen_nested` is false for the recipes that matter most.**
*Probability: medium-high.* The cut symbol `%` and context-sensitive productions appear in exactly the terrain-marking grammars (`river_network`, `trail_network`) and the clipped hedge — i.e. the grammars that produce the training data.
*Defence:* detection is machine-checked at derivation time, the fallback (N cached per-rung derivations) is correct, and the cost is 5× derivation for that archetype only, off the hot path. The Lab shows a badge so it is never a surprise.
*Residual:* if it turns out to be the *common* case rather than the rare one, archetype bake time roughly triples (1.4 s → 4 s natively). Acceptable; time-sliced in wasm.

**R8 — The nav gate rejects too much.**
*Probability: medium. Damage: medium (batch throughput collapses).*
*Defence:* repair (3 attempts) then resample (8 attempts) then **loud hard-fail** with the seed recorded. The 500-interior CI sweep reports the repair/resample/reject distribution so the caps are tuned from data. Lopes growth + circulation keep-out is the design [ProcTHOR] validated at 10 000 environments, so the base rate should be low.

**R9 — World convergence after a Tier-2 edit feels broken.**
*Probability: medium.* At 2 promotions/frame a grammar edit takes ~300 frames to propagate across the visible world.
*Defence:* the status strip shows a convergence bar with an ETA, and `Run ▸ Burst` raises the cap to 16/frame while you are deliberately iterating. Stated as a number (§9.5), not left implicit.

**R10 — Erosion bake time balloons.**
*Probability: low, by design.* Erosion is Tier 3 and scoped to the authored region, never the world. The far field is analytic-only. The UI shows the tile count and estimated seconds before you press the button. Musgrave's own warning stands: *"budget more time for tuning than for implementation."*

**R11 — wasm is never actually verified *in a PR*.**
*Probability: certain if nobody plans for it. Damage: medium.* Nothing gates wasm in PR CI (`grep -i wasm .github/workflows/ci.yml` → zero hits). `deploy-pages.yml` runs **post-merge only** — `push` to master, a nightly cron, and dispatch — so a wasm break in a PR is invisible until it is on master.
*Correction to revision 1.* Revision 1 asserted that `deploy-pages.yml` "has never succeeded" because `/opt/cvc-wasm/emsdk/emsdk_env.sh` does not exist. **That diagnosis was drawn from the wrong machine.** `deploy-pages.yml:34` declares `runs-on: [self-hosted, Linux, X64]` — catx-03 — and the workflow's own header comment says that runner *"already holds the emsdk + wasm-mt deps prefix at `/opt/cvc-wasm/`"*, with `CVC_EMSDK_DIR: /opt/cvc-wasm/emsdk` and `CVC_WASM_DEPS: /opt/cvc-wasm/deps-wasm-mt` set at `:36-37`. The path exists there; it does not exist on this development box, which is what was actually checked. The claim is withdrawn. The COOP/COEP question about `--pthread` on `transfix.github.io` is **left open rather than asserted** — demos are currently live on gh-pages, so whatever the header situation is, it is not fatal.
*Defence:* PR L9 carries an explicit manual verification checklist (`build-wasm-demo.sh` + `serve.py`, Chrome + Firefox, clean console, measured fps, heap high-water via `performance.memory`, mouse-drivable panels, touch on a phone) with the numbers recorded in `LSYSTEM_LAB.md`, and states plainly that **wasm is verified locally-plus-post-merge, never by a PR gate**. Adding a wasm PR gate is out of scope and is flagged as D8.

**R12 — The 45 fps target at 480 k plants.**
*Probability: medium.* The animation LOD removes the *known* bottleneck, but there is a second one I cannot measure from here: **the shadow bake re-renders the whole opaque scene per casting light and is not LOD-aware.** `vtkShadowMapBakerPass` sees whatever is in the renderer, and cvcGL has no per-pass visibility mask.
*Defence:* the cheap workaround is to keep A1/A2/A3 vegetation in a **second, non-casting scene branch** — only A0 and terrain cast — which costs nothing but a scene-graph split. The expensive one is a real per-pass mask, which is new cvcGL surface we are not adding. **We take the cheap one and say so in `LSYSTEM_LAB.md`.**

**R13 — The unified engine leaks.**
*Probability: low-medium.* Three escape hatches around a shared core could become "two things with a framework tax".
*Defence:* the seam is measured — `derivation_mode` ~40 lines, `containment_policy` one virtual call, `context_provider` two implementations of a two-method interface. **Kill trigger stated up front (§5.1):** if any grows past ~150 lines, fork. Lopes growth is already a sibling, which is where the pressure would otherwise have gone.

**R14 — Determinism drift as the code grows.**
*Probability: medium over months.* Hashed RNG is only as good as the discipline around `element` ids.
*Defence:* the salt audit runs in CI on every PR and is the canary; plus order-independence, insertion-stability, thread-independence and no-`mt19937` tests. The audit result is also surfaced in the Seeds tab so an author sees it, not just a reviewer.

**R15 — `risk_detour` is the wrong curriculum label.**
*Probability: unknown — it is an explicit hypothesis.*
*Defence:* the manifest carries enough raw statistics to recompute a better label retroactively without regenerating a world. The label is marked as a candidate in the manifest itself.

**R16 — Merge races.**
*Probability: low, and lower than in revision 1.* #229/#230/#231 have merged; only #223 and #200 remain open, and neither touches a file we edit except `src/cvcGL/CMakeLists.txt` at ~24 / ~281-287, well clear of our EOF append. Four (or six) pre-existing lines, two EOF appends.
*Residual:* the EOF appends conflict **with each other** if L3/L4/L7 are developed in parallel (§14.2's correction). Sequence them.

**R17 — The C++ and Python material twins disagree on two gate defaults.**
*Probability: certain — it is already true. Damage: medium and silent.* `gate_params::horizon_cells` is **12 integer cells** in C++ (`material.h:112`) while `GateParams.horizon_m` is **25.0 metres** in Python (`material.py:159`), and `hard_margin_m` is a fixed **1.0 m** in C++ (`material.h:114`) while Python defaults to `None ⇒ 2·cell_w`. At our export `cell_w = 0.5` the margins coincide at 1.0 m — but the horizons do not: Python resolves to **50 cells (25 m)** and an unconfigured C++ caller uses **12 cells (6 m)**, which is exactly the value GRL-SNAM's own docstring calls *"myopic"*. A bundle would therefore gate differently depending on which language loaded it, and nothing would say so.
*Defence:* the manifest publishes `frame.gate_horizon_recommended_cells = 50` and `frame.gate_hard_margin_max_viable_m`, the adapter asserts the value it configures, and §7.1a states the divergence in the open. **This is not ours to fix** — it is a note for the `cvc::nav` owner, and it is the clearest single argument for recording the frame rather than assuming it.

**R18 — Blur bleed across walls survives ρ_hard = 0.**
*Probability: medium. Damage: low-medium.* ρ_hard = 0 removes the dominant case, but the consumer's blur is still isotropic and occlusion-unaware, so an outdoor puddle within `blur_bleed_radius_m` of a door still raises indoor risk. At the outdoor recommendation (σ = 4 cells) that radius is **8.0 m**.
*Defence:* `scene_kind` selects σ (indoor/mixed ⇒ 1 cell ⇒ 2.0 m), the bounded-contrast validator warns on the specific class pair, the radius is published and drawn as a halo around the brush. *Residual:* a genuinely mixed scene with a mudflat against a warehouse wall will have a slightly hot interior, and the honest answer is that this is a property of the consumer's blur, not of our export — which is why we do not silently pre-compensate for it.

**R19 — The archipelago's outdoor gate rejects too much, or the window policy is wrong.**
*Probability: medium.* A `single-island` policy (D9's recommendation) means the export window can never straddle a channel, which quietly caps the usable window at the smallest island's diameter — Tern is r = 390 m, so a 256 m window fits comfortably, but a 512 m window (decision D2) does **not** fit on Tern at all.
*Defence:* `validate_outdoor` measures and reports it, `cvc-worldgen sample` records rejections in `rejected.csv` with the reason, and the window/island pairing is chosen by the generator rather than the user. *Residual:* D2 (window size) and D9 (window policy) interact, and they must be decided **together**; that interaction is called out in both entries.

### 15.2 What might genuinely not work, stated plainly

- **The 45 fps figure is the number I am least sure of.** R12 names the specific unknown (the shadow bake) and the specific cheap mitigation. If both the shadow proxy and the animation LOD are insufficient, the honest answer is fewer visible instances, and the budget solver already makes that a one-slider change.
- **CDLOD over an analytic height function has never been done in this codebase.** We ship skirts instead, deliberately, and morphing is a later phase that needs a second UV set which does not exist.
- **The hedge and any future vine recipe need query modules fed from a spatial structure**, which means the interpreter runs interleaved with a BVH. That is real work and it is why those recipes sit in the later half of §6 rather than in L0's test set.
- **`fixed_mesh` capacity padding is triangle-only.** If a future asset genuinely needs line geometry, it gets a densely-packed non-padded actor and loses the capacity trick. Foliage-as-cards is what makes this a non-issue today.

### 15.3 Decisions that need the user

> These are marked because they change artifacts other people consume, or shared files, or someone else's contract. **This document deliberately does not pick.**

**D1 — Multi-storey nav export. NEEDS A DECISION BEFORE PR L1 MERGES.**
Does v1 GRL-SNAM consume per-storey layered grids plus `links.json` (option A), or layer 0 only (option C)?
This changes the **bundle schema**, not the renderer, so it must be settled while L1 is being written — not when L6 lands. Recommendation: **write the layered format now (cheap, tested), export `storeys: 1` by default, and have the trainer consume layer 0 in v1.** Mixed indoor/outdoor single-storey works today with zero contract change. If the answer is "yes, cross-floor episodes in v1", the Python side needs a layered planner and a link-cost model, which is outside both this design's and the material session's declared scope.

**D2 — Default export window size. INTERACTS WITH D9.**
This document uses **513 × 513 @ 0.5 m = 256.0 m**. If GRL-SNAM episodes routinely exceed ~250 m of travel, it should be **1025 × 1025 @ 0.5 m = 512.0 m** (4 MB/plane), and every batch-throughput number in §11.2 roughly quadruples. What is the typical episode extent?
**New in revision 2:** under D9's recommended `single-island` policy a 512 m window **does not fit on Tern** (r = 390 m ⇒ ~780 m of land at most, but only ~250 m of it above the mud line) and would be rejected by the outdoor gate. Choosing 512 m therefore also means either dropping small islands from the sampling pool or accepting D9 option 2. Decide D2 and D9 together.

**D3 — ρ values for the classes with no RELLIS/DFC counterpart.**
`gravel`, `sand`, `scree`, `snow`, `tall_grass` and the 8 non-hard indoor classes have defensible but **derived** risk values (§16.2). A domain review before the first large dataset is generated is cheap; afterwards it means either an in-place re-projection with a version bump (possible, since `class.npy` is retained) or invalidating a corpus. Who signs off?
*Revision 2 narrows this:* the 11 `hard` classes no longer need a ρ review at all, because §7.2a sets them to 0.00 on structural grounds. That removes the classes whose ρ was least defensible (what *is* the soft risk of a wall?) and leaves a smaller, more answerable question.

**D4 — Roadmap key casing.**
§20.13.7 is camelCase, §22.1.6 is snake_case, same state path. We adopt **snake_case** at `lab.lod.*` and recommend correcting §20.13.7. **Needs a roadmap-owner ack.**

**D5 — Should a PR CI job build `lsystem_lab` natively?**
`CVC_BUILD_EXAMPLES` defaults `OFF` and no workflow sets it, so the example is compiled in **no PR job** and a native build break lands silently on `master`. (It *is* compiled post-merge for wasm by `deploy-pages.yml`, which is a real but late signal.) Flipping `-DCVC_BUILD_EXAMPLES=ON` in the `package-linux` job would fix that — but it also switches on `nav_common_test` for the first time ever, and **whether that test passes is genuinely unknown**: it has never been built with `GTest::gtest` available and never run under `ctest` anywhere. It is also an edit to a shared workflow file that no PR in this plan budgets for. **Do we want the example gated, and who owns the `nav_common_test` fallout?**

**D6 — Packaging.**
Should `lsystem_lab` ship in the `cvcgl-examples` cvcpkg recipe?
*Revision 2 corrects the premise.* Revision 1 said *"`bunny_shadow` is precedent for not packaging."* **That is false.** `_cvcgl_example_bins` at `src/cvcGL/examples/CMakeLists.txt:82` reads `lsystem_forest bunny_shadow nav_city_swarm nav_fog_ghost nav_finale` — `bunny_shadow` is in it, and it is also in `_wasm_demos` at line 41. **Every example currently built is also packaged**, so the precedent runs the other way and *not* packaging `lsystem_lab` would make it the sole exception. Packaging costs one name on line 82 (which feeds both the RPATH properties and the `install(TARGETS … COMPONENT cvcgl-examples)`) plus a `cvc_revision` bump on the recipe — and a recipe fix without a revision bump silently no-ops on published catalogs, so the bump is mandatory, not optional. **Recommendation: package it.**

**D7 — Material event schedules.**
`MaterialGrid.stamp_risk` / `stamp_hard` bump a version counter documented as *"the scenario's cue to re-cost and replan"*, and `FogScenario` already has an `Event` timeline. No part of this design exports a material **event schedule** — only a static raster. Does v1 training need mud-onset-style dynamic events? If so, the bundle needs an `events.json` schema, and that is a schema decision that belongs in L1 alongside D1.

**D8 — A wasm PR gate.**
`deploy-pages.yml` is **not** broken (see §15.1 R11's correction — the emsdk diagnosis was drawn from the wrong machine), but it runs post-merge only, so wasm breaks reach `master` before anyone hears about them. Adding a wasm build to PR CI needs the self-hosted catx-03 runner, which PR jobs do not currently use. Out of scope here. Does someone want it?

---

> **The two decisions below are new in revision 2. Both change artifacts other people consume. This document recommends but does not pick.**

**D9 — Cross-island traversal policy. DECIDED 2026-08-27 (user): OPTION 2, `forced-bridges`, is the v1 DEFAULT.**

> **User decision, verbatim:** *"let the terrain be connected throughout."*
>
> The generated world is therefore **one connected landmass**: islands are joined by isthmuses ≥ `max(bridge_min_m, 2·agent_radius_m + 1.0)`, and `validate_outdoor` enforces `components == 1` over the whole world rather than per-window. The archipelago remains a **visual and biome** feature — distinct islands with distinct biomes — while being **topologically connected** for navigation, so every sampled (start, goal) pair is solvable by land.
>
> Consequences, propagated below: the §15.1 R19 window cap disappears (a window may straddle a channel because a bridge always exists), the **D2 × D9 interaction is dissolved** (a 512 m window is no longer blocked by a small island), and `single-island` survives only as an opt-in ablation knob for curricula that deliberately want island-isolated episodes. Option 3 (`amphibious`) stays deferred — it changes exported `hard` bytes.

An archipelago's islands are disconnected by construction, and `water_deep` is `hard`. So what *is* an episode?

| | option | what it means | cost | risk |
|---|---|---|---|---|
| **1** | **One island per episode** (`single-island`) | The export window is clipped to one island's mask ∪ its shelf. The gate requires `largest_component_fraction ≥ 0.98` and `channels_crossed == 0`. The archipelago is a **world-scale authoring and visual** feature; the *training window* is single-island. | Free. Already how `far_pair_in_free_space` behaves. | Caps the usable window at the smallest island's land diameter (§15.1 R19). A 512 m window would not fit on Tern. |
| **2** | **Forced land bridges** (`forced-bridges`) | The generator guarantees an isthmus ≥ `max(bridge_min_m, 2·agent_radius_m + 1.0)` between every pair of islands whose windows are co-exported, by raising the smooth-max saddle to +0.6 m and painting it `sand`/`gravel`. Multi-island episodes become solvable. | One repair pass, ~3 ms. | The bridges read as causeways. It also makes the archipelago *topologically* one island, which arguably defeats the point of having several. |
| **3** | **Amphibious ontology** (`amphibious`) | `water_shallow` stops being `hard` and becomes high-soft (ρ 0.90, `hard = false`); only `water_deep` stays hard. Fording is expensive but legal. | Free to implement. | **It changes exported `hard` bytes for every consumer**, and it asserts a vehicle capability (fording) that is a domain claim, not a rendering choice. Corpora built under different policies are not comparable. |

**DECIDED: option 2 as the v1 default** (`--window-policy forced-bridges`, `force_bridges = true`), with **option 1 demoted to an opt-in ablation knob** (`--window-policy single-island`) because it costs one clamp and is genuinely useful for a "cross the isthmus" curriculum tier, and **option 3 explicitly deferred** because it changes `hard` semantics — which is exactly the kind of change §7.5's file-format seam exists to make loud rather than silent. Whichever is chosen is recorded in `manifest.validation.outdoor.policy` and in `provenance.json`, so a corpus can never mix policies undetectably.

**D10 — The export frame: cell size, σ, and what the manifest records. NEEDS A DECISION BEFORE PR L1 FREEZES THE SCHEMA.**

`sigma` is measured in **cells**, so choosing a cell size silently retunes the consumer's behaviourally-validated force constants (§7.1a). At 0.5 m/cell with σ = 1 cell, the effective `lam_soft` is ≈ 4.2× its tuned value — past the 1.5 the consumer's own comment says *"measurably LAUNCHES a vehicle off-world."*

| | option | what it means | cost | risk |
|---|---|---|---|---|
| **A** | **0.5 m/cell fixed; σ recorded as a LENGTH; frame recorded in the manifest** | `cell_w = 0.5` (the source BEV frame). The manifest's `frame` block carries `sigma_m` (2.0 outdoor / 0.5 indoor+mixed), `sigma_recommended_cells`, `blur_bleed_radius_m`, `lam_soft_scale_hint`, `gate_horizon_recommended_cells`, `agent_radius_m`; a quarantined `consumer_frame_ref` block records the reference numbers for forensics and is **never read back** (tested). | ~30 lines of manifest + one adapter assert. | The consumer must actually read `sigma_recommended_cells` instead of hard-coding 1.0. |
| **B** | **Match the consumer's story grid (~2.1 m/cell)** | Zero retune risk outdoors — the exported frame *is* the tuned frame. | Free. | **Kills the indoor half of the design.** A 0.90 m door is 0.43 cells. It also breaks the Python/C++ `hard_margin` coincidence (Python 4.2 m vs C++ 1.0 m). |
| **C** | **Dual export: a 2.1 m/cell outdoor plane and a 0.5 m/cell indoor plane, congruent by construction** | Both frames, honestly. | Doubles bundle size, doubles gate work. | **Breaks the single-`grid_spec` invariant**, which is §0 item 6 and the structural reason a material/occupancy/heightfield misalignment is unrepresentable. That invariant is one of the two or three best properties of this design; spending it here would be a bad trade. |

**Recommendation: option A.** Three independent reasons for 0.5 m specifically: it is the frame the method was ported *from*; it is the **only** cell size at which the C++ twin's fixed `hard_margin_m = 1.0` and the Python twin's `2·cell_w` fallback agree; and it is the coarsest lattice on which a door exists at all. And **yes, record a consumer-constants snapshot** — but under `consumer_frame_ref`, labelled provenance, asserted-unread. §7.1(c)'s rule ("never export consumer tuning") was aimed at *configuration*, and the twins already disagree in two places (R17), so a bundle that records nothing cannot be diagnosed after the fact.

---

## 16. Appendix

### 16.1 Full turtle / scope alphabet

| symbol | params | family | meaning |
|---|---|---|---|
| `F` | `(l)` | motion | move forward `l` and emit a segment (cylinder or generalized-cylinder ring) |
| `f` | `(l)` | motion | move forward `l` without emitting |
| `G` | `(l)` | motion | draw forward but **not** a polygon vertex (ABOP framework leaves) |
| `@Gs` `@Gc(n)` `@Ge(n)` | | motion | begin / add control point / end a generalized cylinder |
| `+` `-` | `(a)` | rotation | turn left / right about the up axis by `a` degrees |
| `&` `^` | `(a)` | rotation | pitch down / up by `a` |
| `\` `/` | `(a)` | rotation | roll left / right by `a` |
| `\|` | | rotation | turn 180° |
| `$` / `@v` | | rotation | **re-level to world up** — Honda assumption 5; not optional for monopodial models |
| `@R` | `(vx,vy,vz)` | rotation | aim the heading absolutely (surface following) |
| `!` | `(w)` | state | set line width / branch radius |
| `;` | `(mat)` | state | set the current surface class (registry name or id) |
| `'` | `(c)` | state | set the current render colour index |
| `@Ts` `@Ti` `@Tf` | `(i,e)` | state | select / set intensity of / toggle tropism `i` with elasticity `e` |
| `[` `]` | | structure | push / pop the scope (frame + size vector) |
| `%` | | structure | **cut** — delete the remainder of this branch. **Sets `gen_nested = false`.** |
| `~S` | `(id,s)` | structure | instance a named sub-mesh (leaf card, flower, prop) at scale `s` |
| `{` `.` `}` | | polygon | begin polygon / emit vertex / end polygon |
| `@#` | `(contour)` | polygon | select the generalized-cylinder cross-section contour |
| `@!` | `(sides)` | polygon | set polygons around the cylinder — **the per-order LOD knob** |
| `T` | `(x,y,z)` | scope | translate the scope |
| `S` | `(x,y,z)` | scope | set the scope size vector |
| `R` | `(ax,ay,az)` | scope | rotate the scope |
| `Subdiv` | `(axis, s0,s1,…)` | scope | split the scope along `axis`; children fill it exactly (strict containment) |
| `Repeat` | `(axis, d)` | scope | tile the scope along `axis` at pitch `d` |
| `Comp` | `(sel)` | scope | decompose into components — `faces`, `front`, `side`, `top`, `edges` |
| `I` | `(asset)` | scope | instance a terminal asset into the current scope |
| `?P` | `(x,y,z)` | query | position query — filled by the interpreter (topiary pruning, surface following) |
| `?H` | `(hx,hy,hz)` | query | heading query |
| `?S` | `(h,slope,flow)` | query | terrain query: height, slope, FD8 flow accumulation |
| `P` | `(mat,r)` | paint | stamp a disc of surface class `mat`, radius `r` metres, at the turtle position |
| `Pw` | `(mat,w)` | paint | stamp a capsule of width `w` from the previous position to this one |
| `Pb` | `(mat,w,f)` | paint | as `Pw` with feather `f` metres |
| `Stamp2D` | `(mat,r0,r1,c0,c1)` | paint | stamp an axis-aligned rectangle (the mud-onset idiom) |
| `Portal` | `(kind,w,h)` | portal | emit a portal record — `door`, `doorway`, `window`, `opening`, `stair_head`, `stair_foot` |
| `Link` | `(kind,up,down)` | portal | emit a cross-storey link with asymmetric cost |
| `Cell` | `(kind,label)` | portal | open a cell record (`room`, `corridor`, `stairwell`, `lobby`, `courtyard`, `exterior`) |
| `nran` | `(mu,sigma)` | function | hashed normal draw on the current `(stream, path, draw)` |
| `ran` | `(hi)` | function | hashed uniform draw in `[0,hi)` |
| `biran` | `(n,p)` | function | hashed binomial draw |

Notes:
- `#ignore: + - F` and friends are **load-bearing** for context matching. Without `#ignore` the ABOP Fig. 1.31 systems do not work at all.
- Left context skips complete `[…]` groups; right context must be able to match *into* a branch.
- Context-sensitive productions **outrank** context-free ones with the same strict predecessor (ABOP p. 30). If the engine picks the first match, Hogeweg–Hesper systems and every signal model silently misbehave.
- Tropism silently kills `/` and `\` unless `@Tf` is used (a cpfg gotcha). Symptom: golden-angle phyllotaxis collapses to a plane the moment gravitropism is enabled.
- Fractional split counts need the accumulated-error scheme, not per-segment rounding [Weber & Penn 1995 §4.2].
- Once **any** production is stochastic, **all** productions with the same predecessor need probabilities. The parser enforces this and reports the offending predecessor by line.

### 16.2 Material registry — the 32 shipped classes

`ρ` → `risk_raw`; `hard` → the `hard` raster; `nav` → the exported occupancy semantics; `veg` → scatter density multiplier. RF columns exist and are exported **zeroed** pending D3.

> **Revision 2: every `hard` class carries ρ = 0.00.** Revision 1 gave the 11 hard classes ρ = 1.00 *and* the `hard` flag, which double-counts against the consumer's A\* `hard_penalty` and its `φ_m` barrier, and — because `risk_raw` is blurred — bleeds up to `blur_bleed_radius_m` into the free space next to every wall. See §7.2a for the arithmetic. The invariant `registry[k].hard ⇒ registry[k].rho == 0.0` is asserted for every ontology variant (§12.3). Consequently the maximum value in `risk_raw` under `merged_default` is **0.90** (`water_shallow`), not 1.00.

**Outdoor (18)**

| id | name | tier | ρ | hard | nav | albedo | veg |
|---|---|---|---|---|---|---|---|
| 0 | `void_unknown` | medium | 0.55 | no | free | 0.50 0.50 0.50 | 0.0 |
| 1 | `asphalt` | low | 0.05 | no | free | 0.16 0.16 0.17 | 0.0 |
| 2 | `concrete_ext` | low | 0.08 | no | free | 0.55 0.54 0.52 | 0.0 |
| 3 | `dirt` | low | 0.05 | no | free | 0.42 0.33 0.23 | 0.15 |
| 4 | `gravel` | low | 0.12 | no | rough | 0.48 0.46 0.43 | 0.05 |
| 5 | `sand` | medium | 0.18 | no | rough | 0.68 0.62 0.44 | 0.02 |
| 6 | `grass` | medium | 0.25 | no | free | 0.27 0.44 0.19 | 1.00 |
| 7 | `tall_grass` | medium | 0.35 | no | rough | 0.30 0.42 0.18 | 1.40 |
| 8 | `bush_cover` | medium | 0.45 | no | rough | 0.22 0.36 0.16 | 1.80 |
| 9 | `bare_rock` | medium | 0.30 | no | rough | 0.46 0.45 0.43 | 0.10 |
| 10 | `scree` | high_soft | 0.55 | no | rough | 0.44 0.42 0.40 | 0.06 |
| 11 | `snow` | medium | 0.40 | no | rough | 0.92 0.94 0.97 | 0.00 |
| 12 | `rubble` | high_soft | 0.75 | no | rough | 0.40 0.38 0.36 | 0.02 |
| 13 | `mud` | high_soft | 0.80 | no | rough | 0.30 0.24 0.17 | 0.30 |
| 14 | `puddle` | high_soft | 0.85 | no | rough | 0.24 0.28 0.30 | 0.00 |
| 15 | `water_shallow` | high_soft | 0.90 | no | rough | 0.16 0.30 0.36 | 0.00 |
| 16 | `water_deep` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.06 0.14 0.22 | 0.00 |
| 17 | `cliff_rock` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.38 0.37 0.35 | 0.00 |

**Outdoor obstacles (3)**

| id | name | tier | ρ | hard | nav | albedo | veg |
|---|---|---|---|---|---|---|---|
| 18 | `tree_trunk` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.28 0.20 0.13 | 0.00 |
| 19 | `boulder` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.42 0.41 0.39 | 0.00 |
| 20 | `fence_pole` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.35 0.33 0.30 | 0.00 |

**Indoor (12)**

| id | name | tier | ρ | hard | nav | albedo | veg |
|---|---|---|---|---|---|---|---|
| 21 | `concrete_floor` | low | 0.06 | no | free | 0.58 0.57 0.55 | 0.0 |
| 22 | `tile` | low | 0.05 | no | free | 0.78 0.78 0.76 | 0.0 |
| 23 | `linoleum` | low | 0.05 | no | free | 0.62 0.60 0.52 | 0.0 |
| 24 | `wood_floor` | low | 0.08 | no | free | 0.52 0.36 0.20 | 0.0 |
| 25 | `carpet` | medium | 0.15 | no | free | 0.34 0.30 0.32 | 0.0 |
| 26 | `metal_grating` | medium | 0.30 | no | rough | 0.44 0.45 0.47 | 0.0 |
| 27 | `wet_floor` | high_soft | 0.65 | no | rough | 0.50 0.52 0.55 | 0.0 |
| 28 | `debris_indoor` | high_soft | 0.70 | no | rough | 0.40 0.37 0.33 | 0.0 |
| 29 | `wall_interior` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.82 0.80 0.76 | 0.0 |
| 30 | `glass_pane` | hard_hazard | **0.00** | **yes** | blocked_wall | 0.62 0.72 0.78 | 0.0 |
| 31 | `void_fall` | hard_hazard | **0.00** | **yes** | blocked_fall | 0.05 0.05 0.06 | 0.0 |

Notes:
- **All 11 `hard` classes carry ρ = 0.00** (§7.2a). The `hard` raster carries them; the consumer's A\* surcharge and `φ_m` barrier act on that raster; adding max soft risk on top double-counts and, through the consumer's blur, raises the risk of the corridor the agent has to walk down. Maximum `risk_raw` under `merged_default` is therefore **0.90**.
- `glass_pane` is `hard` (an agent cannot walk through it) but its **portal record is `traversable: false, opaque: false`** — the visibility system must see through it while the nav system must not. Those are two independent booleans in the emitted schema (§6b.4).
- `void_fall` is `hard` **and** occupied, which is how a stair opening on the upper storey is expressed. Forgetting the second half is how agents walk off a stairwell edge into a hole the grid calls floor.
- Every `hard` class is also `occupancy`-set. `hard ⊆ occupancy` is asserted (§7.3).
- `door_open` and `door_closed` are **not** classes; they are portal records whose `traversable` / `opaque` flags and whose underlying floor class (`concrete_floor` / `wall_interior`) carry the semantics. One raster, no special cases.
- **Bounded boundary contrast** (§7.2a): no two classes may sit adjacent across a single hard cell with `|ρ_a − ρ_b| > 0.60`. The validator warns, naming the pair and a sample cell. `puddle` (0.85) beside `tile` (0.05) is a legitimate 0.80 and will warn; that is the intended behaviour, not a false positive.
- Ontology variants: `soft_vegetation` sets `grass → 0.15`, `tall_grass → 0.22`, `bush_cover → 0.30`. `strict_water_mud` sets `mud → 0.95`, `puddle → 0.95`, `water_shallow → 0.97`, `void_unknown → 0.60`. **Neither variant touches a `hard` class** — they cannot, since the ρ = 0 invariant is asserted for every variant.

### 16.3 Default parameter sets

**LOD (`lab.lod.*`)**

```
preset                = "balanced"     # aggressive | balanced | pristine
desired_pixel_error   = 2.0            # aggressive 3.5, pristine 1.0
hysteresis            = 0.15           # aggressive 0.20, pristine 0.10
fade_tau_s            = 0.12
max_props             = 48             # SET BY MEASUREMENT in PR L3
max_triangles_visible = 2500000
max_bytes             = 734003200
tile_m                = 128.0
scatter_cell_m        = 32.0
cpu_sway_budget       = 24
promotions_per_frame  = 2              # Burst: 16
commits_per_frame     = 4
terrain_switch_m      = [220, 480, 1000, 2200]
veg_switch_m          = [60, 260, 700, 1600]
rock_switch_m         = [120, 400, 1200]
bldg_switch_m         = [200, 900]
force_rung            = -1
freeze_camera         = false
```

**Terrain synthesis**

```
noise            = ridged_multifractal   # NOT musgrave.c multifractal() -- freq bug
H                = 1.0
offset           = 1.0
gain             = 2.0
lacunarity       = 2.0
octaves_max      = 9
base_frequency   = 1/1400 m
warp_distortion  = 0.30
warp_frequency   = 1/2600 m
island_mask      = wyvill (1 - d^2/r^2)^3   # per island; r_core plateau optional
island_combine   = smooth_max               # NOT sum; see section 4.3a.3
smax_k_m         = 40.0                     # blend width; exact max when |a-b| >= k
island_fold_order= sorted by (cx, cy, name) # so h() is a function of the SET
biome_attribution= argmax_i mask_i          # a PARTITION, never a blend
peak_sigma_frac  = 0.28
shelf_m          = -9.0
separation_factor = 1.15    # placement: min |c_i - c_j| / (r_i + r_j)
placement_attempts = 64     # per relaxation round
placement_rounds   = 8      # sep *= 0.95 per round, then HARD FAIL
channel_min_m    = 40.0     # min open water between non-overlapping islands
bridge_min_m     = 12.0     # min navigable isthmus width (also >= 2*agent_radius_m + 1.0)
force_bridges    = true     # DECIDED: D9 option 2 (user, 2026-08-27)
erosion.thermal_iters      = 60      # T = 4/N, c = 0.5   [Olsen 2004]
erosion.streampower_steps  = 30      # m = 0.45, n = 1    [Braun & Willett 2013]
erosion.droplets           = 0       # off by default; expensive, marginal
hydrology.cell_m           = 8.0
hydrology.flow             = FD8     # D8 for channels, FD8 for TWI
```

**Cloud**

```
grammar          = cumulus_anvil
turn_deg         = 32.0
depth            = 6
step0_m          = 8.1
step_decay       = 0.90
puff0_m          = 8.8
puff_decay       = 0.88
splat_sigma_cut  = 3.0               # bound the splat; the predecessor does NOT
ca_grid          = 128 x 128 x 40
ca_step_hz       = 15
drift_m_per_s    = 3.0               # METRES, not grid cells
morph_period_s   = 60.0
deck_shells      = 10
shell_spacing_m  = 45.0
orographic_gain  = 0.65
```

**Vegetation scatter**

```
poisson_min_spacing_m = { conifer: 4.2, broadleaf: 5.6, shrub: 1.8,
                          grass_tuft: 0.9, fern: 1.1, boulder: 3.0 }
slope_max_deg         = { conifer: 34, broadleaf: 28, shrub: 42,
                          grass_tuft: 30, fern: 26, boulder: 55 }
altitude_band_m       = { conifer: [40, 900], broadleaf: [2, 380],
                          shrub: [0, 1100], grass_tuft: [0, 620],
                          fern: [4, 260], boulder: [0, 1420] }
density_per_ha        = { conifer: 220, broadleaf: 140, shrub: 380,
                          grass_tuft: 2600, fern: 300, boulder: 40 }
class_multiplier      = registry[class].veg_density
```

**Interior generation**

All clearance numbers below are **derived**, not authored, from `agent_radius_m` + `grid_m` by `interior_spec::derive_clearances()` (§6b.1a). The two columns are the shipped presets.

```
grid_m                 = 0.5         # indoor AND outdoor -- one lattice
wall_thickness_render_m= 0.25        # MESH thickness
wall_cells             = 1           # RASTER thickness -- walls occupy cells
storey_height_m        = 3.2

agent_radius_m         = 0.35        # THE authored number   | 3.00 (vehicle preset)
  IBC_door_m           = 0.813       # 32 in clear           | 0.813
  IBC_corridor_m       = 1.118       # 44 in                 | 1.118
  free_opening_m       = 0.90        # max(IBC, 2r+0.20)     | 6.20
  corridor_m           = 1.118       # max(IBC, 2r+0.40)     | 6.40
  opening_cells        = 2  (1.00 m) # ceil(0.90/0.5)        | 13 (6.50 m)
  corridor_cells       = 3  (1.50 m) # ceil(1.118/0.5)       | 13 (6.50 m)
  gate.min_path_width_cells = 2      # == opening_cells      | 13

stair_tread_m          = 0.28        # IBC 11 in
stair_riser_m          = 0.18        # IBC 7 in
stair_width_m          = corridor_m  # 1.118 | 6.40
core_min_m             = [5.0, 2.5]  # U-stair; 5.5 x 1.5 for a straight run
elevator_cost_s        = 30.0
stair_cost_up_s        = 14.0
stair_cost_down_s      = 9.0
gate.repair_attempts   = 3
gate.resample_attempts = 8
gate.upper_floor_unsupported_max = 0.0
props.poisson_min_m    = 0.8
props.circulation_keepout = true     # computed BEFORE props; hard
```

**Outdoor gate**

```
policy              = "forced-bridges"  # | "single-island" | "amphibious"  (D9: DECIDED)
inflate_m           = 6.0               # matches planner.far_pair_in_free_space
connectivity        = 4                 # matches the planner's neighbourhood
min_largest_fraction = 0.98             # single-island pass condition
repair_attempts     = 3
resample_attempts   = 8
```

**Export**

```
rows, cols        = 513, 513
extent_m          = 256.0            # so cell_w == 256.0/512 == 0.5 EXACTLY
cell_w            = 0.5              # WRITTEN EXPLICITLY: the C++ material_build takes
                                     # cell_w directly; MaterialGrid re-derives it. Both
                                     # must be satisfied, and the loader asserts they agree.
cell_w_formula    = "(max_x - min_x) / (cols - 1)"
row_order         = "min_y_first"
scale             = 0.05             # normalized-frame scale, matches meta["scale"]
agent_radius_m    = 0.35             # recorded, so a bundle knows who it was built for

# sigma is a LENGTH here and a CELL COUNT at the consumer. See section 7.1a.
sigma_m                     = 0.5    # scene_kind indoor|mixed  (the source BEV frame)
                            = 2.0    # scene_kind outdoor       (~= the tuned 2.1 m)
sigma_recommended_cells     = sigma_m / cell_w      # 1.0 indoor|mixed, 4.0 outdoor
blur_bleed_radius_m         = 4 * sigma_recommended_cells * cell_w   # 2.0 | 8.0
lam_soft_scale_hint         = sigma_m / 2.1         # 0.24 | 0.95  (a RATIO, not a constant)
gate_horizon_recommended_cells = 50                 # == round(25.0 m / cell_w)
gate_hard_margin_max_viable_m  = 0.5                # phi_m at a 2-cell doorway centre

ontology          = "merged_default"
max_boundary_contrast = 0.60
storeys           = 1                # see decision D1
endpoint_select   = "far_pair_in_free_space"
endpoint_inflate_m = 6.0
brush_feather_m   = 0.0              # the consumer blurs; extra feather is additive
```

**Consumer constants — recorded in the manifest ONLY under `consumer_frame_ref`, as provenance, never read back**

Verified in `/home/joe/src/cvc/GRL-SNAM/grl_snam/material.py` (`MaterialParams`, `GateParams`) and `inc/cvc/nav/material.h` (`material_config`, `gate_params`) on 2026-08-27. Note these differ from the circulated brief, which was already stale — **and note the two twins differ from each other in two places** (§15.1 R17):

```
                       Python (MaterialParams)      C++ (material_config)
lam_soft             = 0.5                          0.5f
lam_hard             = 1.0                          1.0f
k_sharp              = 1.25   (1/m)                 1.25f
d_hat (material)     = 12.0   m  (d_hat_sdf_m)      12.0f  (d_hat_m)
sigma                = 1.0    CELLS                 1.0    CELLS
risk_weight          = 10.0   (A* surcharge/unit risk)     -- (planner-side)
hard_penalty         = 25.0   (finite: bias, not forbid)   -- (planner-side)

gate.primitive_count = 16                           16
gate.horizon         = horizon_m 25.0 m  <-- DIVERGENT -->  horizon_cells 12
gate.hard_margin_m   = None (=> 2*cell_w) <-- DIVERGENT --> 1.0 m fixed
gate.improvement_margin  = 0.05                     0.05
gate.material_trigger    = 0.45                     0.45
gate.progress_slack_cells= 0.5                      0.5

# Frame these were tuned in (GateParams / MaterialParams docstrings, verbatim):
#   source BEV        0.5 m/cell, sigma 1 cell  => 0.5 m blur
#   sim story grid   ~2.1 m/cell, sigma 1 cell  => 2.1 m blur   <-- lam_soft=0.5 validated HERE
#   "lam_soft = 1.5 measurably LAUNCHES a vehicle off-world"
```

**The rule, restated.** Grid facts and derived recommendations go in `manifest.frame` and *are* read by the loader. The block above goes in `manifest.consumer_frame_ref`, is labelled provenance, and is **asserted-unread** by a test over `world_bundle.py`. Force constants move; a bundle that treats them as configuration ships stale. A bundle that records them as forensics can be diagnosed a year later — which, given the twins already disagree, is not a hypothetical need.

### 16.4 The biome table

Each `island_spec` names one biome (§4.3a.5). A biome overrides four things; everything it does not override falls through to the global defaults in §16.3.

| | `alpine_massif` | `forested_rounded` | `braided_delta` | `barren_islet` |
|---|---|---|---|---|
| shipped on | Anvil | Kestrel | Tern | Shoal (`large`) |
| `freq_scale` | 1.35 (rough, fractured) | 0.85 (smooth, rounded) | 0.55 (very smooth) | 1.10 |
| `peak_sigma_frac` | 0.22 (sharp) | 0.34 (broad) | 0.42 (a swell) | 0.30 |
| **Layer-0 overrides** | `h > 980 → snow` (default 1180); `slope > 36 → cliff_rock` (default 42); `slope > 24 → scree` | default table | `twi > 5.5 && slope < 6 → mud` (default 7.5); `abs(h) < 6.0 → sand` (default 3.0) | `h > 40 → bare_rock`; no `snow` rule |
| **species overrides** | conifer `density_per_ha` 260, `altitude_band_m` [40, 900]; no broadleaf | broadleaf 220, conifer 140, fern 420 | `grass_tuft` 3400, fern 180; no conifer | `boulder` 90; all vegetation ×0.15 |
| **marking grammars** | `trail_network` only | `river_network` + `trail_network` | `river_network` + `mudflat_region` + `trail_network` | none |
| **building grammars** | none (or 1 × `bunker`) | 1 × `office_3storey` + 2 × `warehouse` | 1 × `warehouse` (the research station) | 1 × `warehouse` (the seam demo) |
| expected `hard_fraction` | high (cliff) ≈ 0.14 | low ≈ 0.04 | very low ≈ 0.02 | medium ≈ 0.08 |
| expected `risk_mean` | ≈ 0.31 (scree) | ≈ 0.22 | ≈ 0.38 (mud) | ≈ 0.19 |

The last two rows are what makes the biome table worth having: **four biomes produce four visibly different material distributions**, so a corpus sampled across islands has a spread rather than one distribution wearing four hats. The Lab's class-fraction histogram (§9.4) plots the current window against both the RELLIS reference *and* the expected biome profile, so an island that has drifted off its own character is visible before 200 worlds are generated from it.

---

### Closing: the five claims this design stands on

1. **The bottleneck is per-frame CPU animation, so the ladder is animation → generation → update → draw calls → triangles.** The wind becomes a vertex-shader function of world position and CPU sway is capped at 24 plants at *every* world size — which means a 480 000-plant world does strictly less per-frame vertex work than the 32-tree demo it succeeds.

2. **The archetype library is the design, not an optimization.** "Hundreds of thousands of plants" is only tractable as "hundreds of thousands of instances of ~64 archetypes". Once that is accepted, merging, impostors, memory and derivation caching all become easy simultaneously.

3. **The class map is a function, not an image, and `risk_raw`/`hard` are a derived, versioned, file-based projection of it.** One `raster(grid_spec)` call emits every plane from one grid, so the material-vs-occupancy-vs-heightfield misalignment that would silently poison every training run is structurally unrepresentable — and because we export only the two raw contract inputs across a file boundary, `cvc::nav::material` can change its blur, its EDT, its gradients or its signature without invalidating a single generated bundle. `cvc::world` does not link `cvc::nav`; only the Lab's *preview* calls it, read-only, so that "what the trainer will see" is the trainer's actual arithmetic.

4. **Navigability is a gate that fails loudly, not a cost term that fails quietly — indoors *and* outdoors.** Merrell's soft accessibility term fails 1-in-5 at three storeys; ProcTHOR's 10 000 navigable houses used a hard gate. So do we — repair, then resample, then refuse to write the bundle and record the seed. And because an archipelago's islands are disconnected by construction, the same discipline applies to the outdoor window: a channel is as unsolvable as a sealed room.

5. **Every number that has to agree with another number is derived from it, not typed next to it.** The indoor clearances come from one `agent_radius_m` (so a gate that no interior can pass is unrepresentable), the blur is recorded as a length and converted to cells at the consumer's resolution (so choosing a cell size cannot silently retune somebody else's validated constants), and the hard classes carry ρ = 0 (so the `hard` plane is the *only* place hardness is expressed). Revision 1 got each of those three wrong in the same way: it wrote down two true numbers that could not both be true at once.

Everything else is staging discipline: three real libcvc modules with literature-validated oracles, tests that actually run in CI, **twelve** PRs whose first one produces a picture, and **four** pre-existing lines edited (six if the Lab is packaged and deployed).

---

## 17. Revision history

### Revision 2 — 2026-08-27, rebased onto `8b6f426`

Revision 1 was written against `10b7904` and reviewed adversarially. This pass keeps its research, structure, tables, code sketches, citations and numbers, and repairs eight defects. Every claim below was re-verified against the tree at `8b6f426`, not against revision 1's description of it.

| # | defect | what changed |
|---|---|---|
| **D1** | "Multi-island archipelago" was asserted in §0 and never specified — one radial falloff, one centre, one radius. | New **§4.3a** specifies seeding (authored + a radius-aware Mitchell/Poisson sampler through the hashed RNG), the `island_spec` / `archipelago_spec` records, the **smooth-max** combination operator (with the argument for why *sum* double-counts and *max* creases), `argmax` biome attribution as a partition, sea level / `channel_min_m` / `bridge_min_m`, and per-island biome divergence. §4.4's height formula now folds smooth-max. New **§16.4** ships four biomes with their Layer-0, species, grammar and expected-statistic overrides. New **§7.8** adds the **mandatory outdoor connectivity gate** (`validate_outdoor`, 4-connectivity, 6.0 m inflation matching `far_pair_in_free_space`, repair → resample → loud reject). The traversal policy is a user decision — **D9** in §15.3 with three concrete options; recommendation: `single-island` for v1, `forced-bridges` as an authored knob, `amphibious` deferred because it changes exported `hard` bytes. |
| **D2** | Three unreconciled clearance numbers: gate 1.5 m, IBC door 0.90 m (1.8 cells), IBC corridor 1.12 m (2.24 cells). No generated interior could pass the design's own gate. | New **§6b.1a** derives every clearance from one authored `agent_radius_m` plus the 0.5 m lattice, snapping up at raster time. `gate.min_path_width_cells` is now **derived** (= `opening_cells`), so the gate can never demand more than a door provides. Arithmetic is shown at both ends of the range (0.35 m person → 2/3 cells; 3.0 m vehicle → 13/13 cells) with a worked raster of a corridor and door passing the gate. The consumer-side consequence — `phi_m = 0.50 m` at a doorway, below `hard_margin_m = 1.0 m`, so the witness gate never certifies a ray through a door — is stated as documented behaviour rather than left to be discovered. Propagated to §6b.3 step 4, §6b.2 step 5, §16.3 and the L6 PR row. |
| **D3** | Hard classes carried ρ = 1.00 *and* the `hard` flag — double-counting against the A\* surcharge and the `φ_m` barrier, and bleeding through the consumer's blur into the corridors the agent must traverse. | New **§7.2a**: **every `hard` class carries ρ = 0.00**, with the double-count argument and the blur arithmetic (a 3-cell corridor reads `r~ ≈ 0.62` at σ = 4 cells under the old scheme, versus 0.06 now). Blur bleed is addressed generally: bounded boundary contrast (0.60, validator warns), a published-and-drawn `blur_bleed_radius_m`, `scene_kind`-selected σ, and an explicit refusal to pre-blur `risk_raw`. §16.2's table, its notes, the §9.4 contract preview and the manifest statistics all updated (max `risk_raw` is now 0.90). New risk **R18**. |
| **D4** | 0.5 m/cell chosen without confronting that σ is in **cells**, so the export resolution silently retunes `lam_soft`. | New **§7.1a** quotes the consumer's own docstrings for its assumed frame (source BEV 0.5 m/cell; sim story grid ≈ 2.1 m/cell; `lam_soft = 0.5` validated there), shows that σ = 1 cell at 0.5 m/cell makes gradients **4.2× hotter** — an effective `lam_soft ≈ 2.1`, past the 1.5 the consumer's comment says *"LAUNCHES a vehicle off-world"* — and fixes it by recording σ as a **length**. The manifest gains a `frame` block (`sigma_m`, `sigma_recommended_cells`, `blur_bleed_radius_m`, `lam_soft_scale_hint`, `gate_horizon_recommended_cells`, `gate_hard_margin_max_viable_m`, `agent_radius_m`) and a quarantined, **asserted-unread** `consumer_frame_ref` provenance block. Three reasons for 0.5 m are given, including that it is the **only** cell size at which the C++ twin's fixed 1.0 m margin and the Python twin's `2·cell_w` agree. Recorded as user decision **D10** with options A/B/C; recommendation A. §7.1 now distinguishes the **C++ arity** (`rows, cols, cell_w, scale, sigma`) from the **Python binding arity**, which revision 1 conflated, and the manifest carries what *both* entry points need. New risk **R17** names the two live twin divergences (`horizon_cells` 12 vs `horizon_m` 25; `hard_margin_m` 1.0 vs `2·cell_w`). |
| **D5** | Factual errors and stale line numbers. | **18 → 16** `add_test(NAME cvcgl_*)` (all 16 line numbers listed; `ci.yml`'s own comments noted as stale in both directions). **`bunny_shadow` IS packaged** — it is in `_cvcgl_example_bins`, so D6's premise was backwards and the recommendation flips to "package it". `_cvcgl_example_bins` is at line **82**, not 69; `examples/CMakeLists.txt` EOF is **97**, not 84; `add_library(cvc` is at **841**, not 827; the tests drift guard is at **1281**, not 1270; `cvc_discover_tests` is at **1143**, not 1133; the ctest drift guard is `ci.yml:334-342`. **"Append at EOF cannot textually conflict" is corrected** — it is the canonical conflict; the accurate narrower claim is stated and the appends are sequenced. The **emsdk diagnosis is withdrawn**: `deploy-pages.yml:34` runs on catx-03, which holds `/opt/cvc-wasm/emsdk`; revision 1 checked this box. `field().shape` → **`field().field.shape`** (a `MaterialField` object has no `.shape`). `meta` values corrected from metres to the **normalized** frame (`rr` 6.0 → **0.15**, `d_hat` 12.0 → **0.35**, `vmax` 8.0 → **0.9**, `dt` 0.1 → **0.06**, `nsub` 1 → **2**) — revision 1 had copied `MaterialParams.d_hat_sdf_m` into `meta["d_hat"]`, which is a different quantity. |
| **D6** | PR 1 was not demoable, against a fixed decision that the deliverable is a design doc **and a buildable first PR** for a *visual* laboratory. | New **§13.1** restructures the head of the plan: `L0a`/`L0b` merge into a single **`L0`**, and the library gains an **`svg_emitter`** + `cvc-lsys svg` — pure text, no GL, no VTK, no new dependency, openable from the PR diff, and fully inside the coverage denominator. `L1` gains `cvc-worldgen build --preview` writing class/risk/hard PNGs through `cvc::image`. The three constraints that make an ImGui-first PR 1 impossible (aggregate coverage gate, examples not built in PR CI, terrain needed under any pixel) are stated rather than implied, and the correct fallback split is named if review size binds. |
| **D7** | Portal/Cell/Link records were emitted with no consumer and no precision. | **§6b.6** is rewritten as a **seam specification** for the separate visibility design: frame and units, portal quad construction with **normative winding** (CCW viewed from cell `a`), cell `footprint` polygons alongside AABBs (an L-shaped room's AABB over-claims by up to 60 %), **`opaque` added as a second boolean independent of `traversable`** (revision 1 forced the consumer to infer opacity from a string it does not own), `path_id`-derived **id stability** so an incremental PVS can cache, a `cvcworld.cells/1` schema version, and an explicit list of what we deliberately do **not** emit. §1.3's non-goal updated. §6b.4's JSON updated. |
| **D8** | The collision map assumed #229/#230 were open and material was concurrent. | **§14** rewritten. #229 (`a33851f`), #231 (`e97d06c`) and #230 (`8b6f426`) are merged; only **#223** and **#200** are open (`gh pr list`). The shared-file cost is **re-derived against the current files** and is **four**, or **six** if the Lab is packaged and deployed — the extra two being `_cvcgl_example_bins` (line 82) and, newly found, **`deploy-pages.yml:38`'s `DEMOS:` list**, without which a wasm Lab builds and is never deployed. New **§14.4** re-decides the "never call `cvc::nav::material`" rule now that the header is merged and stable: **both, at different layers** — the bundle contract stays file-based and `cvc::world` is link-tested not to reference `cvc::nav`, while `lsystem_lab`'s *preview* calls `material_build` / `witness_gate` read-only, because a preview that is not the consumer's arithmetic is a lie and the alternative is a second copy of a function whose entire specification is bit-identity. `cvcGL` already PUBLIC-links `cvc::cvc`, so this adds no dependency edge. |

**Also changed:** §0 items 2, 5, 7, 9, 10 and a new item 11; the headline table gains export-lattice and window-policy rows; G5b (outdoor solvability) added and G8 rewritten; §12.3 gains four new contract tests (both-arities bit-identity, `consumer_frame_ref`-unread, hard-class ρ, and the corrected `field().field.shape`); §12.4 gains the exact lcov allowlist/exclusion and the Linux-only scope; §12.5's table and its preamble rewritten with the precise `CVC_BUILD_EXAMPLES` story (including that `nav_common_test` has still never executed *anywhere*, and that whether it would pass is **unknown**); risks R5, R11 and R16 corrected, R17/R18/R19 added; decisions D2, D3, D5, D6, D8 revised and D9/D10 added; §10.1's CLI and §9.4's World and Surface tabs updated; §16.3's terrain, interior and export blocks rewritten and an outdoor-gate block added; §16.4 (biome table) added.

**Deliberately unchanged:** the literature basis and every citation, the L-system engine design and its `gen_nested` machinery, the hashed-RNG determinism discipline and the salt audit, the LOD ladder and its measured justification, the `fixed_mesh` capacity trick, the GPU-sway `tcoordMC` trap, the Laboratory's four-tier loop, the interior generation pipeline and its Merrell/ProcTHOR grounding, the performance budgets, and the file-format decoupling that is the reason a generated corpus outlives any consumer signature change.

### Revision 1 — 2026-08-27, baselined on `10b7904`

Initial design. Superseded in the eight areas above; retained everywhere else.
