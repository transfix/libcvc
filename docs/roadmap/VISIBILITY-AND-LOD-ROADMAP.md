# Visibility, Culling & LOD — Design & Roadmap

**Module:** `cvc::vis` (headless, in `inc/cvc/vis/` + `src/cvc/vis/`) plus a thin VTK-facing adapter in `cvcGL`.
**Status:** design, not yet implemented.
**Provenance markers used throughout:** **[M]** measured on the target machine (Release, 32-core CUDA box, GTX 1650, 1280×800, vsync **off**), **[D]** derived arithmetically from measured values, **[E]** estimated.

---

## 0. Executive summary

### What we build

One composable subsystem, `cvc::vis`, that turns a flat table of object bounds into a sorted visible set with an LOD already chosen per survivor. It is renderer-free (no VTK, no GL, no Qt, no Boost in the headers) so `cvc::nav`, pycvc, volrover3 and the wasm build can all consume it. `cvcGL` depends on `cvc::vis`; the dependency never reverses.

Inside it, three axes compose:

- **Acceleration structures** (`spatial_index`): a 2-D XZ quadtree with a per-node Y interval over static content, a flat CSR counting-sort grid over moving agents, a brute-force linear index that doubles as the query oracle.
- **Visibility stages** (`cull_stage`): layer mask → distance/size → frustum → small-feature → terrain-horizon occlusion, seeded by either an index traversal or a portal-graph traversal. **v1 ships exactly two occlusion mechanisms — the terrain horizon march outdoors and portal traversal indoors — and no third.** Convex occluder volumes were specified in an earlier revision and are **cut**; §3.6 states the verdict, quantifies what it costs, and names the measurement that could reverse it.
- **LOD** fused into the same traversal, because it consumes the projected screen radius the cull already computed.

Plus three things no engine we surveyed ships and that our use case specifically needs: a **brute-force oracle in the shipping library** (not the test tree), a **conservativeness contract enforced by asserts and a gate**, and a **provenance manifest** recorded next to every generated dataset.

### The headline numbers

| Regime | Visible actors **[D]** | Visible triangles **[D]** | CPU/frame **[D]** | GPU/frame **[D]** |
|---|---|---|---|---|
| Vista — whole 1 km island, 2 km out, 950 m up | 35 | 0.26 M | **2.81 ms (356 fps)** | 0.54 ms |
| Ground — inside the forest, horizon view | 100 | 16.5 M | **8.96 ms (112 fps)** | 2.98 ms |
| Ridge — 208 m peak between eye and target | ~55 **[E]** | ~6 M **[E]** | **~5.0 ms [E]** | ~1.4 ms |
| Indoor **I-S** — `bunker`, 32 × 32 m, 16 cells, corridor view | 16 | 0.34 M | **2.74 ms (365 fps)** | 0.55 ms |
| Indoor **I-L** — `office_3storey`, 60 × 40 m × 3, 150 cells, corridor view | 31 | 0.38 M | **3.64 ms (275 fps)** | 0.56 ms |
| Indoor **I-L** — same, portal stage disabled | 230 | 0.86 M | **12.75 ms (78 fps)** | 0.63 ms |
| **Seam** — standing in the I-L entry doorway, in two cells at once | 41 | 0.27 M | **4.23 ms (236 fps)** | 0.54 ms |

At **200,000 plants**, i.e. 6,250× the content of the current 32-tree demo, which today cannot be loaded at all (§1.4). The two indoor rows are the **two building presets the world generator actually ships** (Lab roadmap §6.7), inventoried in §1.4a — not the "400-room building" earlier revisions of this document costed, which no recipe generates and which is withdrawn. The **seam** row is the hybrid frame the design claims to be about — camera in a doorway, in two cells, paying for **both partitions at once** — and it is a budgeted regime with a classifier, a policy and a test (§10.4a), not a corner case. It is cheap; what was not cheap was the governor's behaviour on the way out of it.

### The single most important insight

The measured cost model on the target machine is **[M]**:

```
cpu_ms/frame ≈ 1.2 + 44.6 µs × visibleActors + 0.2 ns × visibleTris + 100 ns × cpuAnimatedVerts
gpu_ms/frame ≈ 0.5 +                            0.15 ns × visibleTris
```

Read the coefficients against each other and the whole question changes shape:

> **One visible VTK actor costs as much CPU as 223,000 triangles. One CPU-animated vertex costs as much as 500 static triangles. At 60 fps the budget is 346 actors, 77,000,000 triangles, or 154,700 animated vertices — pick one.**

So the user's framing — *"pare down the number of triangles being rendered"* — is, on this engine and this scene, **the wrong objective**. A design that halves the triangle count buys about 0.1 ms. Triangles are effectively free; **actors and animated vertices are not**. Everything below is organized around that inversion, and §1.3 states plainly the several places where the obvious answer (occlusion culling) is a measured net loss.

The second-order insight, which is what makes this a *pluggable* subsystem rather than one culler, is the **regime asymmetry**:

> The same L-System Laboratory world contains an open island vista where occlusion culling has a measured ceiling of 21.5% and costs more than it saves, and a building interior where portal traversal removes **86.5% of visible actors for about 11 µs** (§1.4a, §12.1). A fixed pipeline must be tuned for the regime it serves worst. **The strategy has to be selectable per scene and per region, and the switch has to be a tested, deterministic, recorded artifact.**

And the third insight, which is the one that most changes what we build versus the literature's default answer:

> The measured 91.4% / 96.9% occlusion cull rates are **per-plant, with per-plant bounding boxes**. The 44.6 µs/actor coefficient forces us to batch at 32–256 m cells. A 32 m cell whose AABB contains 24 m trees is almost never *fully* occluded by a 10 m trunk prism, and a conservative box test needs every fragment occluded. **At the granularity we can actually afford, the forest-occlusion win collapses to roughly 10% [E].** Occlusion survives only where the occluder is *larger than a cell*. In this world exactly two things are: **the heightfield**, and **an interior wall seen from inside**. That is why v1 ships a terrain min/max-mip horizon march and a portal traversal, **no software occlusion rasterizer, and — the change made in this revision — no convex occluder-volume stage either (§3.6)**. A building shell is *not* on that list: measured against a 32 m cell it is too short, and §3.6 works the arithmetic. What a building shell does buy is the *interior* content it hides from an outside viewer, and that is delivered by the portal seed (§3.5.6), not by an occluder test.

---

## 1. The problem, measured

### 1.1 A correction to the baseline before anything else

The brief's baseline (24.8 fps shadows on / 21.7 fps shadows off / 29.2 fps at quarter resolution) was taken against a **vsync ceiling**. An *empty* scene renders in 17.7 ms — identical at 320×200 and at 2560×1600, shadows on or off **[M]**:

```
EMPTY 320x200    55.58 fps | render 17.88 ms/f
EMPTY 1280x800   56.10 fps | render 17.74 ms/f
EMPTY 2560x1600  56.15 fps | render 17.72 ms/f
```

With `__GL_SYNC_TO_VBLANK=0 vblank_mode=0` the same empty scene runs at **640 fps**. That single fact explains both anomalies: shadows-off looked *slower* than shadows-on, and quartering the pixel count bought only +18%. Neither was a scene property.

Re-measured with vsync off **[M]**:

| config | fps | cpu_submit | gpu_wait |
|---|---|---|---|
| baseline, shadows ON | 46.8 | 11.72 ms | 0.33 ms |
| baseline, shadows OFF | 53.5 | 9.15 ms | 0.34 ms |

The brief's *conclusion* — CPU-bound — is correct, and is now proven rather than inferred: **`gpu_wait` is 0.42 ms out of a 22.8 ms frame, 1.8%**.

**This is a standing rule for the benchmark harness (§11): any run that cannot demonstrate >300 fps on an empty scene is refused, not reported.**

### 1.2 Where the time actually goes

`lsf_inst --offscreen --frames 300 --width 1280 --height 800`, vsync off, 3 reps **[M]**:

| phase | ms/frame | % |
|---|---|---|
| `cpu_submit` inside `vtkRenderWindow::Render` | **13.50** | **59%** |
| ↳ two ray-cast volume mappers (sea + sky) | 7.74 | 34% |
| ↳ forest wood + needle actors | 3.05 | 13% |
| ↳ shadow-map pass CPU (2048², every 3rd frame) | 2.57 | 11% |
| ↳ terrain | 0.63 | 2.8% |
| ↳ VTK fixed floor (nothing visible) | 1.38 | 6% |
| `seaField` CPU regeneration | **6.30** | **28%** |
| `ResetCameraClippingRange()` | 0.89 | 3.9% |
| cloud shadow + texture upload | 1.12 | 4.9% |
| `reposeTree` + `updateVertices` (as reported by the demo's own timers) | 0.76 | 3.3% |
| **`gpu_wait`** | **0.42** | **1.8%** |

Two of these are **not visibility problems and this design does not get to claim their milliseconds**:

- **`seaField` — 6.30 ms, 28% of the frame.** `seaField()` loops 18 z-slices × 56 × 56 and calls `seaSurface()` (4 waves + `pow`) and `terrainH()` (an `exp` + 4 trig) in the innermost loop, but both depend only on `(x,y)`. It is an **18× redundant recomputation**. Hoisting is one edit worth ~6 ms and has nothing to do with culling. `src/cvcGL/examples/lsystem_forest.cpp` is off-limits to this work; the new `lsystem_lab` example simply must not inherit the bug. See decision **D4** in §14.
- **The two ray-cast volumes — 7.74 ms, 34% of the frame.** `lsystem_lab` uses a surface sea mesh and a skybox. Whether `lsystem_forest` should is a separate question about a file we do not touch. See decision **D3**.

### 1.3 The animation cost is 4.4× what the demo's timers report

`repose_cpu` + `upload_verts` = 0.76 ms looks negligible. Turning wind off (volumes hidden, other CPU work off) reveals the rest **[M]**:

```
vols hidden, WIND ON   145.7 fps | cpu_submit 3.594 | camera 0.728
vols hidden, WIND OFF  326.7 fps | cpu_submit 1.688 | camera 0.063
```

`updateVertices` marks the points modified, which (i) forces a **full VBO/IBO rebuild inside `Render()` (+1.91 ms)** and (ii) invalidates the actor bounds so `ResetCameraClippingRange()` must rescan all 65,596 points (**+0.67 ms**).

```
true animation cost = 0.28 (repose) + 0.48 (upload) + 1.91 (VBO rebuild) + 0.67 (bounds rescan)
                    = 3.34 ms/frame   ~15% of the frame, 4.4x the visible timers
```

**Any design that budgets from the visible timers alone budgets wrong.** This is the single largest line item this subsystem can actually delete, and it is deleted by moving sway to the vertex shader (§7.2), not by culling.

### 1.4 Scene inventory and the ceiling

Current demo, exact **[M]**: 32 trees of 81 candidate sites; 722 modules, 3,288 segments, 2,614 leaf stars; terrain 18,050 tris; forest wood 65,760 tris; needles 23,526 lines; **7 props total**; per wind tick 65,596 vertices × Mat4 into two `double` buffers = 1.50 MB.

Measured cost model, from `actorbench` (real cvcGL `SceneGraph`/`GeometryNode`/`SceneRenderer`, offscreen, vsync off) **[M]**:

- constant 65,536 tris split across N actors: slope **44.9 µs per visible actor**, dead linear 128 → 2048 actors
- one actor, sweeping triangles: **2 M triangles cost the same as 1,024** — marginal 0.2 ns/tri CPU, 0.15 ns/tri GPU
- `setVisible(false)` fully recovers the per-actor cost; re-setting visibility on 1024 actors every frame costs 0.229 ms = **224 ns per `SetVisibility` call**
- animation: **84–118 ns per animated vertex**, cross-validated on the real demo at 512 trees (1.05 M animated verts → 106 ms/animated frame → **101 ns/vertex**)
- VRAM: **46.8 B/vertex** including indices → **~82 KB per full-detail tree**

Target scene: 1 km-radius island, 145 ha of land, 200,000 plants (1,380 stems/ha), mountains to 208 m, plus enterable buildings.

**The target scene is not slow in the current architecture. It is impossible [D]:**

| term | value | cost |
|---|---|---|
| VRAM as unique geometry | 200,000 × 82 KB | **16.4 GB** on a 4 GB card |
| CPU-animated vertices | 200,000 × ~2,050 | 410 M × 100 ns = **41 s/frame** |
| triangles | 411 M | 82 ms |

Merging (route C in the existing demo, 3,776 actors → 145, 1.6 → 21 fps) is *correct* up to about 650 plants — the measurement is unambiguous: **651 trees, 2.2 M triangles, in 2 merged actors, runs at 62 fps static [M]**. Merging then fails, and it fails on **memory**, not on draw cost, at roughly 44,700 trees on this GPU. That is the ceiling argument, and it is why instancing + LOD is what makes the scene *exist* while culling is what keeps it at 60 fps once it does.

### 1.4a Indoor scene inventory — derived from the generator, not assumed

§1.4 inventories the outdoor scene exactly. Until this revision the indoor regime had **no
inventory at all**, and §0 / §3.5.1 / §7.3 / §12.1 nevertheless published two-decimal indoor
budgets. Those budgets were costing a scene that did not exist. This subsection builds the
scene, from the L-System Laboratory roadmap's §6b interior generator, and every indoor number
downstream is re-derived from it.

**Provenance rule for this subsection:** **[L]** = specified by the Lab roadmap (§6.7, §6b,
§8.6, §11.2, §16.2, §16.3) and copied here; **[D]** = derived arithmetically from **[L]** plus
the §1.4 measured coefficients; **[E]** = an assumption *we* adopt because the Lab roadmap does
not specify it. Every **[E]** is named as a gap in the seam, not smuggled in as a fact.

#### What the generator actually emits **[L]**

| Fact | Value | Source |
|---|---|---|
| Building recipes | exactly **three**: `warehouse` (B1, 1 storey), `office_3storey` (B2, 3 storeys), `bunker` (B3, 1 storey) | Lab §6.7 |
| Lattice | `grid_m = 0.5`, indoor **and** outdoor, one lattice | Lab §6b.1a |
| Wall | `wall_thickness_render_m = 0.25` (mesh), `wall_cells = 1` (raster) | Lab §6b.1 |
| Storey pitch / clear height | 3.2 m / 3.0 m (`z_floor 0.0`, `z_ceiling 3.0` in the shipped `cells.json`) | Lab §6b.4, §16.3 |
| Door opening | `opening_cells = 2` → **1.00 m** clear, `height_m = 2.1` | Lab §6b.1a, §6b.4 |
| Corridor | `corridor_cells = 3` → **1.50 m** clear | Lab §6b.1a |
| Stair | tread 0.28, riser 0.18, ~18 risers/storey, ~5.0 m run, core ≥ 5.0 × 2.5 m + elevator | Lab §6b.2 step 5, §16.3 |
| `bunker` plan | BSP **depth 4** on a **64 × 64** grid → **16 leaves**, 1 storey | Lab §6.7 |
| Portal record | planar vertical quad, CCW from cell `a`, with **independent** `traversable` and `opaque` | Lab §6b.4, §6b.6 |
| Indoor material classes | ids 21–31 = **11 classes**, all albedo-only, `hard ⇒ ρ = 0` | Lab §16.2 |
| Prop placement | Poisson-disc, `props.poisson_min_m = 0.8`, inside the circulation keep-out, < 1 ms/room | Lab §6b.2 step 8, §16.3 |
| Interior batch capacity | `interior` merged actor = 40,000 verts / 68,000 tris (current cell + 2 hops) | Lab §8.6 |
| Nav-gate timing | 0.7 ms for one 64 × 64 storey; **12 ms** for the 3-storey office | Lab §11.2 |

**Three gaps in the seam, named.** The Lab roadmap does **not** specify (i) the `office_3storey`
footprint, (ii) a room programme or mean room area, or (iii) **props per room** — `max_props = 48`
is a VTK *actor* budget and is explicitly a placeholder to be measured in Lab PR L3, not a
per-room prop count. Nor does it give an indoor agent population. Each is filled below with a
stated **[E]** assumption and a derivation that can be replaced by one number when the generator
owner supplies it.

*(A fourth, minor inconsistency, recorded because a reader will hit it: the Lab's §6b preamble
says the interior work adds "14 new classes", its §16.2 heading says "Indoor (12)", and the table
under that heading lists ids 21–31, which is **11**. 11 is the count consistent with the stated
32-class total (18 + 3 + 11). We use 11.)*

#### Preset **I-S** — `bunker`, the small interior **[D]**

Footprint is fully determined: 64 × 64 cells × 0.5 m = **32.0 × 32.0 m = 1,024 m²**, one storey,
3.0 m clear.

| Term | Value | Derivation |
|---|---|---|
| Cells | **16** | BSP depth 4 = 2⁴ leaves **[L]**; the room-and-corridor carve makes 13 of them rooms and 3 of them corridor runs |
| Rooms / corridor cells | 13 / 3 | as above |
| Corridor run | **76 m** (3 segments), 114 m² at 1.50 m | corridor spine reaching 13 leaves across 32 m |
| Wall run | **320 m** = 128 perimeter + 192 interior | depth-4 axis-aligned BSP cut lengths on 32 m: 32 + 2·16 + 4·16 + 8·8 = 192; raster footprint 160 m² at `wall_cells = 1` |
| Mean room | **57.7 m²** | (1,024 − 114 corridor − 160 wall) / 13 |
| Portals | **20** = 13 room doors + 2 corridor junctions + 4 room-to-room + 1 exterior | one door per room is the gate's own reachability requirement (Lab §6b.3 step 2) |
| Portals per cell | **2.5** | 2 × 20 / 16 |
| Windows (`glass_pane`) | **0** | a bunker has none |
| Shell triangles per cell | **~600** | walls both faces + floor + ceiling + door jambs, per §5.5a |
| Props per room | **4.8** **[E]** | 1 per 12 m² — sparse industrial; see the prop note below |
| **Props, building** | **63** | 13 × 4.8 |
| Prop triangles | **37,800** | 63 × 600 tri (the §6.2 interior-prop L0) |
| Agents | **8** **[E]** | squad-scale interior scenario |
| **Building total triangles** | **53.4 k** | 13 × (600 + 4.8 × 600) + 3 × 600 + 8 × 800 |
| Registry classes present | **4** — `concrete_floor`, `metal_grating`, `debris_indoor`, `wall_interior` | Lab §16.2 |
| Shader programs | **2** — opaque + HUD; no glazing, so no transparent program | §7.5 |
| **CPU-animated vertices** | **0** | no sway indoors; agents are rigid L0 meshes (§6.2 gives the agent ladder no sway row) |

#### Preset **I-L** — `office_3storey`, the large interior

Footprint **[E]: 60 × 40 m = 2,400 m² per storey, 3 storeys.** The Lab roadmap gives no
footprint. This one is **calibrated against the only quantitative handle it does give** — its own
§11.2 nav-gate timings. The gate's dominant term is step 3, an A\* per (entrance, room-centroid)
pair, so cost ≈ *k · R · N* for *R* rooms over *N* cells. The `bunker` row pins
*k* = 0.7 ms / (16 × 4,096) = **10.7 ns/unit**. A 60 × 40 m plate is 120 × 80 = 9,600 cells and
carries 38 rooms (below):

```
k x R x N  =  10.7 ns x 38 x 9,600  =  3.90 ms / storey   ->   11.7 ms for three storeys
Lab SS11.2 publishes                                            12 ms
```

**Within 2.5 %.** The footprint is an assumption, but it is an assumption the generator's own
published timing corroborates, which is the strongest form available short of asking the owner.

> **This is also what retires the "400-room building."** A 400-room office at ~49 m²/room needs
> ~133 rooms on an ~80 × 80 m plate, i.e. 160 × 160 = 25,600 cells per storey. Run it through the
> same calibration: 10.7 ns × 133 × 25,600 = **36 ms per storey, 109 ms for three** — **9× the
> Lab's published 12 ms.** The 400-room building is not merely unsourced; it is *inconsistent with
> the generator's own cost model by an order of magnitude*. It is withdrawn, and every number that
> was derived from it is re-derived below.

| Term | Value | Derivation |
|---|---|---|
| Plate / grid | 2,400 m², 120 × 80 = 9,600 cells | **[E]** above |
| Circulation | **22 %** of the plate = 528 m² = **352 m** of 1.50 m corridor | **[E]**, ~10 corridor cells/storey |
| Vertical core | 12.5 m² U-stair + ~9 m² elevator = 2 cells | **[L]** `core_min_m = [5.0, 2.5]` |
| Mean room area | **49 m²** (≈ 6.5 × 7.5 m) | **[E]**, consistent with §5.5a |
| Rooms per storey | **38** | (2,400 × 0.78 − 22) / 49 |
| Cells per storey | **50** = 38 rooms + 10 corridor + 2 core | |
| **Cells, building** | **150** | 50 × 3 |
| **Rooms, building** | **114** | |
| Wall run per storey | **~600 m** = 200 perimeter + ~400 partition | 38 rooms at 6.5 × 7.5 m, shared walls counted once |
| Façade bays | 66 per storey | 200 m perimeter at a 3.0 m `Repeat` pitch **[L]** Lab §6b.2 step 1 |
| Windows per storey | **40** (`glass_pane`, `traversable:false`, `opaque:false`) | 60 % of bays glazed **[E]** |
| Portals per storey | **90** = 38 doors + 10 junctions + 2 core + 40 windows | storey 0 adds 2 exterior doors |
| Vertical portals | **2** `VERTICAL_OPEN` stair openings | §3.5.4 requirement 4; elevator shafts stay sealed |
| **Portals, building** | **272** | 92 + 90 + 90 |
| **Portals per cell** | **3.6** | 2 × 272 / 150 — close enough to §3.5.2's "~4 portals per room" that that half of the old sentence survives |
| Shell triangles per cell | **~600** | §5.5a |
| Props per room | **12** | 1 per 4 m² inside the circulation keep-out (§5.5a) |
| **Props, building** | **1,368** | 114 × 12 |
| Prop triangles | **820,800** | |
| Agents | **24** **[E]** (8/storey) | |
| **Building total triangles** | **~932 k** | 114 × 7,800 + 30 corridor × 600 + 6 core × 1,000 + glass + 24 × 800 |
| Registry classes present | **7** — `linoleum`, `carpet`, `tile`, `wood_floor`, `wall_interior`, `glass_pane`, `void_fall` | Lab §16.2 |
| Shader programs | **3** — opaque + transparent (glass) + HUD | §7.5 |
| **CPU-animated vertices** | **0** | |

#### The two assumptions that are ours, stated once and used everywhere

**Props per room.** The Lab gives the *mechanism* (Poisson-disc at `props.poisson_min_m = 0.8`,
after the gate, inside the circulation keep-out, < 1 ms/room) and **no density**. `max_props = 48`
is a VTK *actor* budget and is explicitly a placeholder to be measured in Lab PR L3 — it is not a
per-room prop count and must not be read as one. We adopt **1 prop per 4 m² of room floor** for
`office_3storey` (furnished; §5.5a uses the same figure) and **1 per 12 m²** for `bunker` (sparse,
industrial). A 0.8 m Poisson disc saturates at ≈ 1.7 sites/m², so both are far below saturation and
neither can conflict with the generator's own spacing rule.

**Because props ride inside their cell's prop batch (§5.5a), a 2× error in prop density moves the
indoor actor count by exactly zero** and the I-L portals-on frame by 0.009 ms. That is worth
stating plainly: the number we could not get from the generator is the number that does not
matter. What *would* matter is a prop policy that gave props their own actors; §5.5a forbids it.

**Indoor agent population.** Not specified anywhere. We adopt **8 (I-S)** and **24 (I-L)**, against
the 4,000 the outdoor budget carries. Agents are one instanced batch for the whole frame
(§3.5.6b), so this too is an actor-count no-op.

#### The visible set, per regime **[D]**

Actors follow the §5.5a identity — **one shell batch and one prop batch per reached cell**, plus a
**third, transparent batch in cells that carry glazing** (a glass pane cannot share a mapper with
opaque geometry; this is a refinement of §5.5a's "2 and not 1", not a contradiction of it). Portal
traversal from a corridor reaches 6–12 cells (§3.5.2); with portals disabled the seed is
frustum-only and knows nothing about walls or floor slabs.

| | **I-S** portals on | **I-S** portals off | **I-L** portals on | **I-L** portals off |
|---|---|---|---|---|
| Cells reached | **7** (3 corridor + 4 rooms) | **12** of 16 | **10** (3 corridor + 6 rooms + core) | **96** of 150 |
| — why | narrowed frusta through 4 doors | 63.1° hfov covers ~75 % of a 32 m box | §3.5.2's 6–12, mid-range | 36 on storey 1 (73 % of a 60 × 40 plate) **+ 60** on the storeys above and below, which a 42° vfov reaches beyond 8.3 m and no frustum or Y-slab test can reject |
| Shell batches | 7 | 12 | 10 | 96 |
| Prop batches (rooms only) | 4 | 10 | 6 | 73 |
| Glass batches | 0 | 0 | 6 | 24 |
| Agent batch (instanced) | 1 | 1 | 1 | 1 |
| Exterior actors through apertures | **3** (narrowed) | **35** (un-narrowed) | **3** (narrowed) | **35** (un-narrowed) |
| HUD / ImGui | 1 | 1 | 1 | 1 |
| **Visible actors** | **16** | **59** | **31** | **230** |
| Interior triangles | 18.1 k | 42.4 k | 54.4 k | 598 k |
| Exterior triangles through the aperture | 324 k | 260 k | 324 k | 260 k |
| **Visible triangles** | **343 k** | **303 k** | **379 k** | **859 k** |
| CPU-animated vertices | 0 | 0 | 0 | 0 |

The exterior term is one band-A quadtree leaf at L0 (141 plants × 2,055 tri = 290 k), one terrain
chunk (32 k) and sea + sky (2 k), per §5.5a; the un-narrowed column takes the §12.1 vista set.

Three results in that table were not visible before the inventory existed:

- **Most of an indoor frame is outdoors.** At I-L portals-on, 324 k of 379 k visible triangles —
  **85 %** — are the view *through* the doorway, and 3 of 31 actors carry them. The interior is
  the cheap part. If the indoor budget ever tightens, the lever is the aperture, not the rooms.
- **The exterior hand-off is most of the small preset's win.** I-S goes 59 → 16 actors, and **32 of
  the 43 removed are outdoor actors** that an un-narrowed frustum drags in through the entry door.
  The interior cull alone is 22 → 11. A 32 m building is not big enough to need portal *traversal*;
  what it needs is `frustum::narrowed_by` at the doorway.
- **The cross-storey leak is the large preset's dominant interior term.** 60 of I-L's 96
  frustum-only cells are on storeys the camera is not standing on, reached because the quadtree's
  Y interval is a slab test and not an occlusion test. Portals delete them exactly — the stair
  opening is the only `VERTICAL_OPEN` portal and its narrowed frustum is tiny. **The dominant
  indoor win is vertical, not lateral**, which is not what §3.5.1's "a wall occludes everything
  behind it" framing implies. It is a floor slab, and there is no outdoor analogue of it.

### 1.5 The occlusion profile — and where occlusion culling does NOT help

Software z-buffer study, exact terrain mesh with per-pixel hierarchical-Z bounding-box tests, target scene, 1280×800, 42° vfov **[M]**:

| viewpoint | frustum-culled | terrain occl. | + trunk/canopy | still visible | depth complexity |
|---|---|---|---|---|---|
| **vista** (2 km out, 950 m up) | 0% | **0%** | 21.5% | **78.5%** | 7.7 |
| **ground** (inside forest, on a slope) | 62.1% | 34.1% | 34.6% | **3.2%** | 41.4 |
| **ridge** (208 m peak between) | 50.2% | 49.5% | 49.5% | **0.3%** | 16.8 |
| **thicket** (dead flat, vegetation only) | 84.0% | 0% | 15.5% | **0.5%** | 21.1 |

Three findings that must survive into the design:

**(a) Porous foliage is worthless as an occluder at any opacity. The solid stem is everything.** Same flat-ground scene, canopy-only occluders **[M]**:

| canopy α | depth complexity | conservatively occluded |
|---|---|---|
| 0.2 | 18.8 | **0 of 31,927** |
| 0.4 | 18.4 | **0** |
| 0.7 | 18.3 | **0** |
| **1.0 (fully solid canopy)** | 18.3 | **0** |
| + solid trunk prism (r = 0.12·canopy, 0 → 0.40 h) | 21.1 | **96.9%** |

Zero, even with *solid* canopies at depth complexity 18. Canopies span 0.35h → h, leaving an open understory band; a conservative box test needs *every* fragment of a bound occluded, and one peephole defeats it. This refines the usual "thin trees aggregate into good occluders": they aggregate **only through their trunks**, and only if the proxy is opaque.

**(b) In the vista regime, occlusion is the wrong tool and small-feature culling is the right one.** Contribution histogram at vista, 200k plants, mean **8.83 visible pixels per plant [M]**:

```
contributing <   1 px :  21.5%          contributing <  64 px : 100.0%
contributing <   4 px :  41.6%          contributing < 256 px : 100.0%
contributing <  16 px :  79.8%
```

Occlusion's theoretical ceiling in that frame is 21.5%. **A 4-pixel small-feature threshold reaches 41.6% for about 0.5 ns per candidate, with no occluder set, no depth buffer, and no conservativeness risk.** Every plant in the frame is under 64 px; the correct answer at vista is not to cull them but to *stop drawing them as plants* (HLOD, §6.4).

**(c) The batch-granularity collapse.** This is the finding that most changes the design and it is a derivation, not a measurement:

> The 91.4% / 96.9% figures above are computed with **per-plant** bounding boxes. The 44.6 µs/actor coefficient forces batching at 32–256 m cells (§5). A 32 m cell's AABB is 32 × 32 × ~28 m; the occluders available in a forest are trunk prisms reaching 0.40 × 23.8 m ≈ 9.5 m. For a conservative test the *entire* cell box must be occluded, and the crowns clear every trunk. Extrapolating the per-plant sweep to cell-sized boxes gives an expected cull rate of **~10% [E]** for vegetation-on-vegetation occlusion at ground level.
>
> Occlusion at batch granularity survives exactly where the occluder is **larger than a cell**: terrain ridges (which supply 34.1 of 34.6 points at ground and 49.5 of 49.5 behind a ridge — i.e. essentially all of it) and interior walls seen from inside. **Building shells fail the same test that trunk prisms fail, for the same reason and by a smaller margin**: a 10.5 m office block only fully occludes a 32 m cell that is more than 3× further away than the block itself. The arithmetic is in §3.6, and it is why the convex occluder-volume stage is cut.

**This number is [E], it is load-bearing, and §11.4 schedules the measurement that settles it** (re-run the occlusion study with cell-sized AABBs at 32/64/128/256 m — a one-line change to the existing `occl.cpp` harness). If it comes back at 60% rather than 10%, a raster occluder becomes worth building; see decision **D9**.

### 1.6 Consequences, ranked

1. **Animated-vertex budget** — hard cap ~155,000 verts/frame (75 full-detail trees) at 60 fps. Wind must be a GPU vertex-shader term or an LOD level, never a CPU pass over the world.
2. **VRAM** — ~44,700 full-detail trees on this GPU. LOD and instancing are mandatory long before draw cost bites.
3. **Visible actor / batch count** — 346 at 60 fps. This is where culling cashes out, and it is why the batch boundary must be the spatial cell (§5).
4. **Triangles** — a 77 M/frame budget. **Do not design for this.**

---

## 2. How Unreal / Fortnite does it

### 2.1 The classic pipeline, ordered by cost

Epic documents the culling methods in increasing order of cost and applies them in that order [Epic Docs; Anagnostou 2017]:

1. **Distance culling** — per-actor min/max draw distance; `Cull Distance Volume` generalizes it as an array of (bounding-sphere diameter → cull distance) pairs, so small props die close and large ones survive far.
2. **View frustum culling** — always on. `SceneVisibility.cpp::FrustumCull` is a `ParallelFor` over `Scene->PrimitiveBounds` — **a flat array of bounds owned by the scene, not the actor hierarchy** — one task per 4096 primitives, 32 at a time, writing per-view visibility bit arrays.
3. **Precomputed visibility** — offline per-cell PVS. Off by default; opt-in.
4. **Dynamic occlusion** — hardware occlusion queries (default), HZB occlusion, or round-robin (VR).

Two structural lessons transfer directly: **cull from a flat SoA table, not from the scene graph**, and **order stages cheapest-first**.

### 2.2 Why Epic moved away from hardware occlusion queries

Three reasons, all in Epic's own material and all applicable to us [Epic Docs; Bittner et al. 2004; Mattausch et al. 2008]:

- **Latency.** Results are read back one frame later; two frames on mobile. The error is asymmetric in the wrong direction — a false *negative* means the object is simply missing, and latency scales with camera angular velocity, so it is worst exactly when the player is doing the thing games are about.
- **Cost.** Each query is a draw call plus a state change plus a readback. Epic mitigates by grouping up to 8 known-invisible props into one query — a mitigation that is itself evidence the primitive is expensive.
- **Granularity.** A query resolves one bit for one bounding box. It cannot cull part of a mesh and cannot express LOD.

### 2.3 Nanite, in one paragraph each

**Cluster DAG.** Leaf clusters are exactly 128 triangles. Offline: group adjacent clusters, lock shared boundary edges, merge, simplify to ~half, split back into 128-tri clusters, repeat. Merge-then-split makes it a DAG, not a tree. Error is a quadric metric in world-space units, forced **monotonic** parent-to-child, so there is exactly one transition on any root-to-leaf path [Karis et al. 2021].

**LOD as a local decision.** `Render iff ParentError > threshold && ClusterError <= threshold`. Because it is local, no traversal is needed and any acceleration structure works; Epic uses a BVH8 keyed on *ParentError*. The threshold is ~1 pixel, which is why Nanite needs no dithering and no fade.

**Two-phase occlusion.** Draw what was visible last frame, rebuild the HZB from that partial depth, re-test the rejects against the *fresh* HZB, draw the disoccluded remainder. Karis's framing: *"why try to reproject the depth buffer when we can instead reproject the geometry we rendered to the depth buffer?"* Result: *"Almost perfect occlusion culling. Conservative. Only falls apart under extreme visibility changes."*

**Numbers.** ~2.5 ms from GPUScene to a complete visibility buffer with *"nearly zero CPU time"*; **25,041,711 triangles rasterized, consistent across the whole demo regardless of scene complexity**. Work becomes proportional to screen pixels, not to scene complexity.

**What it does not solve.** Streaming, memory, and per-actor CPU cost. Which is why World Partition and HLOD remain mandatory, and why `Min Screen Radius` and distance culling being *unsupported* on Nanite is acceptable rather than a loss.

### 2.4 World Partition, HLOD, and the foliage story

- **HLOD layer types**: *Instancing* (replace static meshes with ISM components at their lowest LOD — Epic calls this ideal for tree imposters), *Merged Mesh*, *Simplified Mesh*. Critically, **HLOD actors live on their own runtime grid with their own cell size and loading range**: the far field streams and culls at coarser spatial granularity than the near field.
- **HISM** builds a static cluster tree over instance transforms so culling descends from a root and rejects whole branches — ~N tests become ~log N. It never culls "inside" a draw call; it culls *before* it and **compacts surviving instance IDs into a contiguous buffer**, then issues one instanced draw with that count.
- **Fortnite Chapter 4 trees**: Nanite, ~300–500k vertices each. **Opaque geometry beats masked cards** — *"we found it was usually faster to avoid masked materials and instead rely on increasing triangle counts of the mesh and keeping the materials opaque"*. **Wind is baked**: branch pivots and orientations encoded as texture pixels indexed by a custom UV, so the rasterizer does one position + one quaternion lookup with no dependent reads. **WPO Disable Distance** kills wind evaluation past an artist-set distance, applied to the shadow pass too, relative to the *main camera*.
- **Preserve Area**: when simplification can no longer reduce a leaf below one triangle it must delete leaves, thinning the canopy; Preserve Area redistributes the lost area by dilating the remaining boundary edges. We hit the same problem and solve it analytically (§6.3).

### 2.5 Virtual Shadow Maps — four architectural lessons, none of them the mechanism

We cannot build VSM on VTK 9.5 + OpenGL2. Four of its findings are architectural and cost nothing to adopt [Lauritzen & Olsson 2023]:

1. **Split static from dynamic** in the shadow cache; re-render only movers.
2. **Cull casters against visible receivers**, not against the light frustum — VSM's page marking, stripped of virtualization, is "only render shadow space that something on screen actually samples". Fitting the light's ortho to the *visible* geometry bounds is free and raises effective shadow resolution at the same time.
3. **Exclude cheap things and fake them.** Fortnite ships grass with **no shadow-map contribution at all**, using screen-space contact shadows.
4. **Be skeptical of shadow proxy meshes.** Epic built them, shipped them (300k+ tri main mesh, 60k+ tri shadow proxy), then concluded they *"make no significant difference to the performance of distant (or even mid-range) foliage"*.

### 2.6 What we can and cannot borrow

| Borrowable | Why it transfers |
|---|---|
| Cost-ordered cascade (distance → frustum → occlusion) | Costs nothing; `vtkRenderer` already runs a culler, just the wrong one (§8.2) |
| Cull from a flat SoA bounds table owned by the scene | Directly shapes `scene_view` (§4.2) |
| Size-bucketed max draw distance | One formula, `d_max(r) = k·r / p_min` (§3.2), replaces the whole authored table |
| HLOD merged/simplified proxies **on their own coarser grid** | This is what makes our vista cheap by construction (§6.4) |
| Instanced foliage with a hierarchical instance cull, survivors compacted | Exactly our quadtree + batch design (§5) |
| Continuous terrain LOD by mip morphing (height **and** XY offset) | Pure vertex-shader arithmetic, works on GLES3 |
| The four shadow lessons above | Free, and #2 is the fix for a real correctness trap (§8.3) |
| Dithered LOD transition | **Not borrowable** — requires TAA/TSR to resolve; VTK 9.5 gives us FXAA only, which is spatial and will render a dither as noise. We cross-fade instead (§6.5). |

| Not borrowable | Hard blocker |
|---|---|
| Nanite's software rasterizer | 64-bit image atomics (`InterlockedMax` on an R32G32_UINT UAV) — not portable on GL 4.3, absent from WebGPU |
| GPU-driven two-phase HZB with indirect draw | Compute + SSBO + `DrawIndirect`. **Structurally impossible in WebGL2.** And on VTK it means bypassing mappers entirely, at which point VTK is a window and a camera. See §3.2 for the measured reason this is also a *loss* at our triangle counts. |
| Virtual Shadow Maps | Compute + indirect + atomics + per-pixel page table |
| Nanite's runtime DAG cut | The offline half is reproducible; evaluating a per-object cut per frame on the CPU costs more than a discrete ladder saves |
| Precomputed Visibility | The bake. Our worlds are procedurally regenerated per training episode (§3.5.5) |
| Fortnite's scale (16 M instances, 100k actor files) | VTK's binding constraint is per-actor render-thread CPU cost. The remedy is the pre-Nanite remedy: fewer, larger, spatially-coherent batches |

---

## 3. Approach comparison

This is the pros-and-cons section the request asked for. It is deliberately candid about where each technique is a net loss.

### 3.1 Acceleration structures

Complexities for *n* objects, *C* cells, *d* depth.

| Structure | Build | Frustum query | Memory | Dynamic | Impl. difficulty | Best-fit scene |
|---|---|---|---|---|---|---|
| **Uniform grid (CSR)** | O(n+C+R) counting sort, fully parallel | O(cells in frustum) — proportional to frustum **volume** | `4(C+1) + 4R` B, flat | **O(1) move; full rebuild/frame is usually faster** | Easiest (~150 lines) | Many similarly-sized movers; neighbour queries |
| **Hierarchical grid** | O(n), one pass, level = ⌈log₂(diam/h)⌉ | Σ over levels; must test level ℓ *and all coarser* | flat, ~1 ref/object | O(1) | Medium-easy | Heterogeneous *sizes*, all dynamic |
| **Octree (classic)** | O(n·d) | O(visible nodes + log n) | 40–64 B/node | O(d) remove+reinsert | Medium | Volumetric, density-varying, static |
| **Loose octree (k=2)** | O(n), level←size, cell←center | 2–3× more nodes visited (loose bounds overlap) | as octree | **O(1)** | Medium | Heterogeneous sizes *and* densities, dynamic |
| **kd-tree** | O(n log n) with sorted events | O(log n), strict front-to-back, exact early-out | 8 B/node but **2–5× primitive duplication** | **Hostile** — a split plane is a global statement, no refit analogue | Hard to make fast | Static ray tracing (historical) |
| **BSP (polygon-aligned)** | O(n log n)–O(n²); geometry grows 1.5–4× | O(n) traversal; the *ordering* is the product | 24 B/node + inflated geometry | **None** | Hard | Convex decomposition; solid modelling; collision hulls |
| **BVH (AABB tree)** | binned SAH O(n log n) | **O(k + log n), k = visible** — proportional to visible *content* | 32 B × (2n−1), **no duplication** | Refit O(n) or O(d)/leaf; rebuild on SAH degradation | Medium | The general default, static or dynamic |
| **XZ quadtree + Y interval** | Morton sort + one sweep | O(visible nodes + log n) | 40 B/node | rebuild leaf run on edit | Medium-easy | **2.5-D outdoor worlds** |
| **Cells + portals** | emitted, or from a BSP | O(visible cells × portals/cell) | ~200 B/cell + 80 B/portal | topology static, **contents fully dynamic** | Medium (~400 lines) | Indoor architecture |
| **PVS (from-region bake)** | superlinear; hours historically, minutes now | **O(1)** bit-vector lookup | naive `leaves²/8`, RLE-compressed | **Static only** | Hard (the bake) | Frozen indoor content |

#### Candid notes on each

**Uniform grid / spatial hash.** Its superpower is O(1) update with a flat, vectorizable, allocation-free layout, and for n up to ~100k a **full rebuild every frame beats incremental** — it is a counting sort, it parallelizes, and it leaves memory perfectly ordered [Ericson 2005 §7.1]. Its two structural weaknesses are teapot-in-a-stadium (a dense cluster degenerates to a linear scan) and, decisively for us, **query cost proportional to frustum volume rather than to visible content** — fatal at a 2 km view distance. Cell size ≈ 1.5–2× the mean object diameter; too small and the (object, cell) reference count explodes, too large and every cell is a linear scan.

**Hierarchical grid.** Fixes the *size-distribution* weakness while keeping O(1) updates, by assigning each object to the single level whose cell size ≥ its diameter, so it spans ≤ 2 cells per axis. It does **not** fix density heterogeneity, and the query must test the object's level *and every coarser level*, never finer. Good broad-phase for heterogeneous dynamic scenes; still volume-proportional at long range.

**Octree.** The killer is **straddling**: an object is stored at the deepest node that *fully contains* it, so a 1 cm object sitting on the world origin crosses all three root split planes and lands at the **root**, where every query visits it. This is not a rare pathology — procedural placement clusters on round coordinates, grid lines and street centrelines, so boundary bunching is systematic. All three classic workarounds are bad: duplicate into every overlapping leaf (unbounded blow-up), split the object (impossible for instances), or push to the parent (that *is* the problem).

**Loose octree.** Ulrich's fix decouples *which level* (a function of size alone) from *which cell* (a function of position alone) by expanding node bounds by k=2 about the centre; an object whose centre is in a cell is entirely inside that cell's loose bounds, so **straddling is structurally impossible** and every object lives in exactly one node [Ulrich 2000]. Insert and move are O(1) integer arithmetic. The honest characterization: **a loose octree with k=2 is a hierarchical grid with a tree spine over it** — same size→level, position→cell rule; the spine adds hierarchical rejection, which is the real difference. Cost: overlapping node volumes mean 2–3× more nodes visited than a tight tree. And like any octree it wastes its Y axis on a 2.5-D world.

**kd-tree.** Superb static ray-tracing structure with 8-byte nodes and exact early ray termination, and the SAH's empty-space maximization is genuinely valuable there. But it is **space**-partitioning, so straddling primitives are referenced in both children (typical 2–5× blow-up, pathological cases far worse), and it has no refit analogue: you cannot grow a plane the way you can grow an AABB, and any local fix breaks the disjointness invariant traversal depends on. The production ray-tracing world migrated kd → BVH around 2006–2008 and hardware RT (DXR, Vulkan RT) standardized on BVH. **Do not start a new one.**

**BVH.** Object-partitioning: every primitive in exactly one leaf, node volumes allowed to overlap — and that overlap is precisely what buys dynamic-friendliness, because refit is a min/max union. Binned SAH [Wald 2007] gets within a few percent of a full sweep at a small constant factor. Refit degrades under incoherent motion, so you monitor SAH cost and rebuild past ~1.3× [Wald et al. 2007], or use fat AABBs with incremental insertion (the Box2D/Bullet dynamic AABB tree). Two-level BLAS/TLAS makes "static content + moving agents" a non-problem. **Its cull cost scales with visible content, not with frustum volume** — that plus O(1)-ish updates is why it is the correct default for anything hierarchical. Its one real weakness for us: it gives no natural **level ↔ cell-size ↔ LOD-band** binding, which is exactly the property our batching needs (§5.3).

**BSP — the verdict.** A polygon-aligned BSP gives an exact depth ordering of polygons from any viewpoint in O(n) with no per-frame sort and no depth buffer [Fuchs et al. 1980]. That was decisive for Doom's visplane renderer and Quake's span/edge-list renderer — and **the hardware z-buffer made its entire product free, per-pixel, and better** (it handles interpenetration). Four further disqualifiers for rendering: splitting inflates geometry 1.5–4× *and* imposes a per-frame, view-dependent, per-polygon draw order that is irreconcilable with large static index buffers and instancing; per-triangle CPU traversal is the wrong granularity when we want thousands of triangles per draw; it is fully static (Quake's `func_*` brush entities were excluded from the BSP precisely because nothing that moves can be in the tree); and the bakes do not fit an iterative content workflow.

> **Verdict on BSP: dead as a rendering-order structure, alive and correct as a *convex decomposition* tool — which is exactly what portals need.** BSP is indoor-scene technology because its three premises (the world decomposes into convex cells; occluders are large relative to view distance; geometry is static) are satisfied by architecture and violated by terrain. It appears in this design in exactly one place: the interior generator's floor-plan subdivision, whose leaves become our cells and whose doorway cuts become our portals (§3.5). It is also still the right tool for solid-leaf collision hulls, point-in-solid classification, and CSG [Naylor et al. 1990].

**Cells + portals.** Order-of-magnitude wins indoors that no frustum-only structure can approach, because frustum culling has no concept of "a wall is in the way". **No precomputation**, and — critically — only the *decomposition* is static; cell **contents** may be fully dynamic. Recursion depth is bounded naturally by the frustum narrowing to empty. Degenerates to frustum-only in open-plan spaces and outdoors.

**PVS.** The cheapest runtime visibility ever shipped: descend to the camera's leaf, index a bit vector, frustum-cull a small candidate set — O(1), independent of scene size. Disqualified here by workflow, not by cost (§3.5.5).

### 3.2 Visibility algorithms

| Algorithm | Removes | Latency | Conservative | Occluder fusion | Requires | Net **loss** when |
|---|---|---|---|---|---|---|
| Frustum (sphere/AABB, SoA SIMD) | CPU + GPU | 0 | yes (exact) | n/a | nothing | never at our scale — overhead is 3 orders below the actor cost it removes |
| Distance / size-bucketed max draw | CPU + GPU | 0 | approximate by choice | n/a | nothing | never; it is the cheapest stage in the pipeline |
| Small-feature (screen area) | CPU + GPU | 0 | **no** (approximate) | n/a | nothing | never in cost; it trades a declared, tunable pixel error |
| Terrain horizon march (min/max-mip) | CPU + GPU | 0 | **yes**, one-line proof | yes (one continuous field) | a heightfield | flat terrain; camera above the terrain |
| Convex occluder volumes | CPU + GPU | 0 | yes | **no fusion** | authored/emitted hulls | many medium occluders that only jointly occlude |
| Portal traversal | CPU + GPU | 0 | yes | yes (baked into topology) | cell/portal graph | outdoors; open-plan interiors |
| PVS lookup | CPU + GPU | 0 | yes | yes (offline) | a bake, static geometry | procedural content; outdoors (one cell = the world) |
| HW occlusion queries (naive) | **nothing** — net loss | ≥1 frame (2 mobile; **≥1 by spec in WebGL2**) | yes | yes | GL 1.5 | **essentially always** |
| CHC++ | CPU submit + GPU | 0–1 frames | yes | yes | queries + a BVH ≥5k nodes | little occlusion; small hierarchies |
| Software occlusion raster (MSOC) | **CPU** + GPU | 0 | yes (±1 px) | yes | SSE2+/AVX2 | few good occluders; low object counts; **our batch granularity (§1.5c)** |
| GPU two-phase HZB + indirect | GPU + draw calls | 0 | yes | yes | **compute + SSBO + indirect** | **below ~100 M triangles** — measured 2× *slower* at 10 M |
| Depth reprojection / coverage buffer | CPU + GPU | 1 frame | **no** | yes | GPU depth readback | fast camera or fast movers; WebGL (readback = pipeline flush) |
| Ray-traced / SDF visibility | CPU + GPU | 0 | **no** (point sampling / lossy field) | yes | RT hardware or an SDF build | as a hard cull, always; fine as an LOD *hint* |

#### The break-even is real, published, and repeatedly rediscovered

- Greene, Kass & Miller measured HZB at **17% slower at 15k polygons**, exactly break-even at 45k, faster above [Greene et al. 1993]. That is the earliest published break-even for occlusion culling and it still holds qualitatively.
- Bittner et al.'s hierarchical stop-and-wait on their City scene: **19.90 ms vs 19.79 ms for plain frustum culling** — a net loss. On the Power Plant walkthrough it goes slower than frustum culling wherever depth complexity is low [Bittner et al. 2004].
- CHC++ Figure 1 shows **CHC itself performing worse than view-frustum culling** in parts of the Power Plant walkthrough. NOHC and CHC++ exist to fix that [Mattausch et al. 2008].
- MSOC on Neu Rungholt: *"outperformed by frustum culling for some difficult camera positions"* [Hasselgren et al. 2016].
- **Kitware's own VTK WebGPU two-pass HZB culler: 0.5× (2× SLOWER) at 10 M triangles**, 1.5× at 155 M, and their stated conclusion is that it is *"only viable for large amounts of triangles (100M+)"* [Kitware WebGPU]. Our ground-regime visible budget is 16.5 M.
- AC Unity culled **20–40% of triangles** with GPU-side backface and cluster-bounds culling and got *"only small overall gain: <10% of geometry rendering"* [Haar & Aaltonen 2015] — because the pipeline was not triangle-bound. Neither is ours.

#### Why hardware occlusion queries are excluded outright

The failure is structural, not an implementation defect. The WebGL 2.0 spec is explicit and deliberate: *"A query's result must not be made available until control has returned to the user agent's main loop"*, added *"to prevent applications from relying on being able to issue a query and fetch its result in the same frame"*. So a query-based scheme is ≥1 frame stale by specification in the browser and by driver reality natively. And the saving is on the GPU — the processor measured at **1.8% utilization**. Conditional rendering is worse: it saves GPU work only; the draw call, the state changes, the mapper update and the traversal all still run.

CHC++ is genuinely excellent engineering — batching cuts state changes by up to two orders of magnitude, jittered `n_av` removes the query-alignment spikes, multiqueries cut invisible-node queries by an order of magnitude, and it beats frustum culling reliably where CHC does not [Mattausch et al. 2008]. It is aimed at a bottleneck we do not have, it wants ≥5,000 hierarchy nodes (we will have ~1,900), and it wants to *drive* the traversal and hand batches to the engine, which fights `vtkRenderer`'s pass structure hard. **Documented fallback, not shipped.**

#### Why software occlusion rasterization is the *right family* and still not what we ship

MSOC is the only family whose answer arrives **on the CPU, synchronously, this frame**, so it can suppress draw submission, mapper updates *and* animation — which is exactly what a CPU-bound renderer needs [Hasselgren et al. 2016]. It stores depth directly at tile granularity (32×8 rasterization tile, 8×4 storage tile holding `Zmax0`, `Zmax1` and a 32-bit coverage mask), ~10% of a depth buffer's memory, near-free clear, and a front-to-back traversal with a wall-clock budget knob that degrades gracefully. Apache-2.0.

Three reasons we do not ship it in v1:

1. **The batch-granularity collapse (§1.5c).** The 96.9% flat-forest figure is per-plant; at 32–256 m cells the expected rate is ~10% **[E]**, while the stage costs 1–4 ms unconditionally on an 8–9 ms frame.
2. **Inner-conservative occluder authoring is an unbounded correctness risk.** The authors themselves note the occluder mesh must be shrunk by roughly the area of one pixel, which *"depends on projection and can potentially be unbounded"*, and that this *"puts high requirements on the artists"*. That is a rule-3 (conservativeness) dependency that no unit test in `cvc::vis` can prove absent.
3. **Runtime SIMD dispatch (AVX-512/AVX/SSE4.1/SSE2) is a determinism hazard** for reproducible GRL-SNAM datasets.

**What we ship instead** is a specialization that matches the measured occluder distribution exactly: a **terrain min/max-mip horizon march**. Two pyramids over the heightfield — `min_mip` (conservative hit) and `max_mip` (fast skip). The conservativeness proof is one line: the real terrain height at any point is ≥ the min over any footprint containing it, so `min_mip > ray_height` implies the ray is blocked. **No epsilon, no shrink, no occluder authoring, no SIMD dispatch, identical native and wasm, ~300 lines.** And it captures 34.1 of the 34.6 measured occlusion points at ground and 49.5 of 49.5 behind a ridge, because in this world the one occluder that is larger than a cell *is* a heightfield.

A note on HZB for anyone who revisits this: **VTK's OpenGL2 backend uses conventional depth (0 near, 1 far), not reverse-Z.** A depth pyramid here must therefore be a **max** reduction with the test "occluded iff the object's nearest depth exceeds the texel's max". Nanite's published `min` reduction is correct only under reverse-Z; copying it verbatim would invert the test and cull everything visible.

### 3.3 The verdict per scene type and viewing regime

| Regime | Defining property **[M]** | Structure | Seed | Filter stages | Primary tool | Occlusion verdict |
|---|---|---|---|---|---|---|
| **Vista** — whole island, far/above | 78.5% of in-frustum is visible; depth complexity 7.7; mean 8.83 px/plant; **terrain occludes 0%** | XZ quadtree, coarse levels only | `index_seed` | layer → distance/size → frustum → small-feature | **HLOD proxies.** 200,000 plants become ~9 baked canopy shells; there is nothing left to cull | **OFF.** Ceiling 21.5%, all tree-on-tree; small-feature reaches 41.6% at 1/3 the cost |
| **Ground** — among vegetation | 3.2% visible; depth complexity 41.4; **34.1 of 34.6 occlusion points are terrain** | XZ quadtree, all levels + agent grid | `index_seed` | + `terrain_horizon` | **GPU sway + LOD bands.** The bottleneck is animated vertices, not visibility | **ON but modest.** Terrain-vs-cell only; vegetation-on-vegetation collapses at batch granularity |
| **Ridge / valley** — mountain between | 0.3% visible; **49.5 of 49.5 points are terrain** | as ground | `index_seed` | + `terrain_horizon` | **Terrain horizon march.** This is the regime it exists for | **ON, strongest outdoor case** |
| **Indoor** — rooms, corridors | **13.5% of frustum-only actors survive [D]** (§1.4a); walls are near-perfect occluders | cell/portal graph | `portal_seed` | frustum ∩ narrowed portal frusta | **Portal traversal.** 86.5% of visible actors removed for ~11 µs (§12.1) | **Replaced by portals** — exact, bake-free, and it also gives gameplay/AI visibility |
| **Hybrid** — island with enterable buildings | both, often in one frame | one quadtree, `cell_id = 0` = exterior | `portal_seed` composing `index_seed` | union of both | One pipeline; only the *seeding* differs (§3.5.6) | Portals indoors, terrain march outdoors, in the same frame |
| **Small scene** (< ~1,000 proxies) | everything is visible anyway | `linear_index` | brute force | frustum only | **Nothing.** 500 SIMD sphere tests is ~2 µs; any structure is complexity for negative return | OFF |

---

## 3.5 Indoor scenes — cells, portals, PVS, and BSP

Indoor is now a first-class requirement, and it changes the analysis more than any other single input, because it is the regime where the techniques that lose outdoors win by an order of magnitude.

### 3.5.1 Why indoor inverts the outdoor conclusion

| | Vista | Indoor |
|---|---|---|
| In-frustum objects that are visible | **78.5% [M]** | **13.5% [D]** — 31 of 230 actors, 10 of 96 cells (§1.4a) |
| Occluder quality | canopies occlude **0 of 31,927** at α=1.0 **[M]** | a wall occludes everything behind it, exactly |
| Occluder size vs batch size | trunk 9.5 m vs 32 m cell → conservative test fails | wall spans the whole cell boundary → test succeeds trivially |
| Value of occlusion culling | −0.5 to −4 ms (net loss) **[D]** | **12.75 → 3.64 ms [D]** (I-L, §12.1) |
| Right primary tool | HLOD proxies | portal traversal |

**This asymmetry is the organizing justification for a pluggable subsystem.** It is not architectural taste; it is a **3.5× swing** in the same world (down from the 6× an uninventoried indoor scene appeared to give — §12.1), and no parameter tuning reconciles it because the disagreement is about which *mechanism* does the work. 3.5× on the frame's dominant term, with the outdoor mechanism scoring a *net loss* in the same world, is still more than enough to carry the argument.

**And the mechanism is not the one this table implies.** §1.4a shows that 60 of I-L's 96 frustum-only cells are on the storeys *above and below* the camera, dragged in because the quadtree's Y interval is a slab test and not an occlusion test. The dominant indoor win is **vertical**, not lateral: portals delete a whole building's worth of other-storey content that no frustum test can touch. Lateral room-to-room culling is the smaller half.

### 3.5.2 The portal traversal algorithm, concretely

Luebke & Georges' formulation with screen-space AABB narrowing [Luebke & Georges 1995]:

```
seed(eye, near_box):
    S = { c : cell c's bounds, inflated by the near-plane radius, contains eye }
    if S is empty:  S = { EXTERIOR_CELL }          # outdoors
    return S

traverse(cell, frustum f, depth):
    if depth > MAX_DEPTH: return
    if visited[cell] already recorded a frustum that CONTAINS f: return
    record visited[cell] = union(visited[cell], f)

    if cell == EXTERIOR_CELL:
        emit index_seed(quadtree, f)               # <-- the hybrid hand-off, §3.5.6
    else:
        emit contents(cell)                        # a contiguous proxy run

    for portal p in cell.portals:
        if p.state == CLOSED:                 continue
        if p.flags & BLOCKS_SIGHT:            continue      # a solid door
        if dot(p.plane.n, eye) + p.plane.d < 0: continue     # facing away
        f2 = f.narrowed_by(p.quad, eye)                      # 2-D screen-AABB intersect
        if f2.empty():                        continue
        traverse(p.other(cell), f2, depth + 1)
```

Cost: **O(visible cells × portals per cell)**, no precomputation. Against the real inventory (§1.4a): `office_3storey` carries **150 cells and 272 portals, i.e. 3.6 portals per cell** — the "~4 portals per room" earlier revisions asserted is the one part of that sentence that survives contact with the generator. Camera in a corridor: **10 cells reached** (inside the 6–12 range), **43 portal clips**, at the §15.7 rate of 250 ns/portal → **~11 µs [D]**, not the ~5 µs previously estimated. `bunker` is smaller still: 16 cells, 20 portals, 2.5 portals/cell, 7 cells reached, ~18 clips, **~4.5 µs [D]**. The result is also a 4096-bit `active_cells` bitset (512 B), consumed by **`cell_mask_stage`** — a real, shipped filter stage (§4.1, §4.3, §4.4), not a hypothetical "later stage". It ANDs the bitset against `scene_view::cell_id` at 1.5 ns/candidate and is the *only* consumer of that column.

Two cases that read like special cases and are not, because both are resolved **before** the stage runs:

- a static proxy straddling a doorway is a member of two cells, which one `uint16_t` cannot express — the second and later memberships live in a side table (**§3.5.6a**);
- an agent that has walked into a room has had its `cell_id` **rewritten this frame** by the agent locator, because `cell_id` is a per-proxy column the *scene* owns and a mover's membership is not static data (**§3.5.6b**).

Neither is a branch in the traversal, but neither is free either, and the two subsections below cost them.

`frustum::narrowed_by(poly, eye)` is a first-class operation on the frustum type, not a special case buried in the stage. That is what lets the exterior hand-off be composition rather than a branch.

### 3.5.3 The BSP duality — floor-plan subdivision vs render ordering

These are two different uses of the same data structure and only one of them is alive.

| BSP as… | Status | Why |
|---|---|---|
| **render-order structure** | **Dead** (§3.1 verdict) | z-buffer made its product free; splitting fights batching; static; wrong granularity |
| **convex space decomposition** | **Alive and exactly what we want** | BSP leaves are convex *by construction* — that is the one guarantee portals need |
| **point location** (which cell is the camera in) | **Alive** | O(depth) descent, robust, no epsilon tuning |
| **solid/collision hull** | Alive (out of scope here) | Quake/Source lineage still ships it for point/AABB-in-solid |

### 3.5.4 How much the BSP synergy really buys — and what breaks it

If the interior generator builds floor plans by BSP subdivision, the rendering-side cell decomposition and portal graph do come out largely for free:

| Generator artefact | Visibility artefact | Free? |
|---|---|---|
| Leaf of the subdivision | a convex cell | **Yes, if split planes lie on wall planes** |
| Shared boundary with a cut opening | a portal quad | **Yes** — the generator already knows where it cut a door |
| Interior node planes | O(depth) camera point location | **Yes** |
| Leaf convexity | correctness of the screen-rect narrowing argument | **Yes** for an axis-aligned BSP |

**Do not overclaim it. Six things break it, and the generator must be told about all six:**

1. **Split planes chosen for area balance rather than for walls.** If the subdivider optimizes leaf-size balance, leaves do not correspond to rooms at all and the portal graph is garbage. **This is a hard contract requirement, not a preference.**
2. **Open-plan and merged rooms.** A great room spanning six BSP leaves is one *visual* cell; without a merge pass you get portals through thin air and the traversal explodes. The generator must run a merge pass (union leaves with no wall between them) and emit merged cells, keeping the BSP only for point location. Detect the residual case at load: if `Σ portal_area / cell_boundary_area > 0.4`, mark the cell `OPEN` and let the stage pass it through rather than pretend to cull.
3. **Non-convex rooms** (L-shapes, alcoves). Either sub-split into convex sub-cells, or accept a looser AABB cell — which is conservative and therefore safe, just less selective.
4. **Vertical connectivity.** A per-floor 2-D BSP has no concept of a stairwell or a mezzanine, and culling a visible mezzanine is a *correctness* bug, not a performance one. The generator must emit explicit `kind = VERTICAL_OPEN` portals for stair volumes and floor openings.
5. **Glass, grates, doorways without doors.** A transparent surface is not an occluder. Two independent flags are needed (`BLOCKS_SIGHT`, `BLOCKS_MOVEMENT`), not one.
6. **Objects straddling a doorway.** A proxy whose AABB spans two cells must be registered in both or it disappears from one side — a *correctness* bug, not a performance one. `scene_view::cell_id` is one `uint16_t` and structurally cannot say "both"; a per-cell overflow list also cannot, because the stage is indexed by proxy, not by cell. The mechanism that can, its sentinel values, its memory and its cost are specified in **§3.5.6a**.

Net assessment: **the synergy is real and worth taking — it converts a heavy voxelization-and-portal-extraction pipeline into a data hand-off — but it is contingent on requirement 1, which is a decision the generator owner has to make deliberately.** If it is not honoured, we fall back to AABB cells derived from the room list, which still works and is still conservative, just with looser narrowing.

### 3.5.5 What we ask the world generator to emit

A concurrent design effort owns the generator. This is the ask, verbatim (see decision **D5**):

```
cells[]        : { convex hull (≤ 12 planes) or AABB, floor_z, ceil_z, floor_index,
                   room_kind, flags(OPEN|SEALED) }
portals[]      : { cell_a, cell_b,                       # cell_b == 0 means exterior
                   convex quad (≤ 8 coplanar verts, CCW from cell_a), plane,
                   state u8 (0 closed .. 255 open),      # present even for permanent openings
                   flags(BLOCKS_SIGHT|BLOCKS_MOVEMENT|VERTICAL_OPEN|DYNAMIC) }
bsp[]          : optional interior nodes + planes, for O(depth) camera point location
proxy_cell[]   : per emitted static object, its PRIMARY cell id.
                 0 = exterior;  0xFFFF = UNPARTITIONED (always active).
                 NOTE the correction: an earlier revision wrote "0xFFFF = exterior",
                 which contradicts SS3.5.6 and scene_view.h. 0 is the exterior.
proxy_cell_extra[] : CSR side table for STRADDLERS only -- ofs[n_straddlers+1] + ids[].
                 Empty for a purely outdoor world. See SS3.5.6a.
cell_content[] : per cell, { l0_tri_count, prop_count, shell_tri_count }.
                 What lets the runtime decide sub-batching from the manifest
                 instead of walking the geometry (SS5.5a).
instances[]    : MORTON-SORTED on (x,z) over the world bounds  -- the runtime asserts this
lod_ladder[]   : per species, { derivation_depth, radius_scale, err_world (metres) },
                 err_world MONOTONIC across rungs
seed, generator_revision                                  -- copied into the run manifest
```

Two of these are non-obvious and load-bearing. **Morton order** is what makes a cell a contiguous index range, which is what makes a cell simultaneously a batch, a draw range and an HLOD unit. **`portal.state` present even for permanently-open doorways** means a later closeable door is a data change rather than a schema change.

**Two entries were withdrawn from this ask in the present revision.** An earlier draft additionally requested

```
occluders[]    : per building/wall, an INNER-CONSERVATIVE convex hull (≤ 10 halfspaces)
trunk_prisms[] : per plant above a height threshold, an opaque 8-triangle prism
```

Both existed to feed one consumer, `occluder_volume_stage`, and that stage is cut (§3.6). With it gone **nothing in `cvc::vis` reads either array**, so asking for them would be asking the generator to author an inner-conservative hull — the single unbounded correctness risk identified in §3.2 — for a stage that does not exist. This also closes a live contract mismatch rather than opening one: the Lab roadmap's §6b.6 is frozen and explicitly emits *no* occluder hulls, no occluder fusion and no exterior cell decomposition. The withdrawn ask is preserved verbatim above because it is exactly the input a future MSOC stage would need (**D9**), and re-requesting it is the first step of that work, not a schema change.

### 3.5.6 The hybrid case and the doorway seam

`cell_id == 0` means "the exterior". This is not two systems glued together; it is one pipeline where only the *seed* differs:

- **Purely outdoor world**: the portal graph is empty, `portal_seed` degenerates to `index_seed`, cost 0.
- **Indoors**: `portal_seed` traverses cells; the quadtree is never touched.
- **Inside looking out through a window or door**: traversal reaches `EXTERIOR_CELL` and hands the **narrowed** frustum to `index_seed`, so you pay for outdoor content only through the aperture. This is the single most valuable line in the hybrid design and it is why `frustum::narrowed_by` is a public operation.
- **Outside looking at a building**: `portal_seed` starts at cell 0, emits the outdoor index seed, and additionally *rejects* proxies whose `cell_id` names an interior cell not reached through a visible doorway. Standing outside, you do not pay for the furniture inside.
- **Straddling a doorway** — the case that produces flicker in naive implementations: the seed is **every cell whose bounds, inflated by the near-plane radius, contain the eye**, seeded with the un-narrowed frustum, and the results are unioned. A camera in a doorway is therefore in two cells and sees both. Conservative, costs one extra cell, and requires **no special-case code** — the seam is a property of the seeding rule, not a branch in the traversal.

Portal polygons are additionally inflated by one pixel-equivalent at the near plane, so a camera clipping through a door jamb cannot produce a hole.

### 3.5.6a One proxy, two partitions — what `cell_id` actually names

The document says the hybrid frame is the point, and a hybrid frame contains **two partitions of space that do not nest**: the quadtree leaves of §5.2 and the portal cells of §3.5.5. A room is not a quadtree leaf and a quadtree leaf is not a room. `scene_view::cell_id` is a single `uint16_t`, so it can name one of them, and the document has to say which.

**It names the portal cell, and only the portal cell. The quadtree leaf is never stored per proxy.**

That is not a tie-break; it falls out of the seed/stage split:

| partition | who consumes it | what it needs | per-proxy column? |
|---|---|---|---|
| quadtree leaf | the **seed** (`index_seed`) | to *emit* a contiguous Morton run of proxy ids | **no** — the run *is* the membership; a column would be a redundant copy of `first_proxy/proxy_count` |
| portal cell | a **stage** (`cell_mask_stage`) | to *test* an arbitrary candidate id | **yes** — a filter is indexed by proxy and has no run structure to lean on |

A seed adds and therefore owns index ranges; a stage filters and therefore needs a column. There is exactly one column because exactly one of the two partitions is consumed by a filter. This also means the outdoor path never touches `cell_id` at all: for a purely outdoor world the column may be `nullptr`, `cell_mask_stage` is not in the pipeline, and the whole mechanism costs zero bytes and zero nanoseconds.

**The three rejected alternatives**, so nobody re-litigates them:

- *A discriminator bit* (high bit = quadtree leaf, low 15 = id). The portal bitset is capped at **4,096 cells** because 512 B fits a UBO (§15.8); halving the space to 2,048 to encode a partition we established needs no column is a pure loss.
- *A second `leaf_id` column.* 2 B × 200,000 proxies = 400 KB of cache-resident SoA that no stage ever reads.
- *A tagged union.* Same cost as the discriminator bit plus a branch in the hottest 1.5 ns loop in the module.

**Membership, encoded.** Cells 1…4,095 are interior cells. Two values are reserved:

```cpp
inline constexpr std::uint16_t exterior_cell     = 0;       // outdoors
inline constexpr std::uint16_t unpartitioned_cell = 0xFFFF; // "always active"
```

`unpartitioned_cell` is the **conservative escape hatch** and it exists so that no code path ever has to guess: an object the generator did not classify, an agent between locator updates, a debug proxy injected by a test — all take `0xFFFF`, and `cell_mask_stage` passes them through unconditionally. A wrong-but-conservative id costs one actor; a wrong-and-aggressive id deletes visible geometry.

**Straddlers.** A proxy in two or more cells carries its primary cell in `cell_id` and the rest in a CSR side table, emitted once at load (§3.5.5) and never rebuilt:

```cpp
// inc/cvc/vis/scene_view.h  (added to the struct)
  const std::uint16_t *cell_id        = nullptr;  // PRIMARY cell. 0 == exterior,
                                                  // 0xFFFF == unpartitioned/always-active
  // Straddlers only (SS3.5.6a). Both null, or both non-null.
  const std::uint32_t *cell_extra_ofs = nullptr;  // count+1 CSR offsets, or null
  const std::uint16_t *cell_extra     = nullptr;  // extra cell ids, ascending
```

`cell_mask_stage` is then, in full:

```
keep(p) =  cell_id[p] == 0xFFFF
        || active_cells.test(cell_id[p])
        || (cell_extra_ofs && any(active_cells.test(cell_extra[i]))
                              for i in [cell_extra_ofs[p], cell_extra_ofs[p+1]) )
```

The third clause is skipped entirely when `cell_extra_ofs == nullptr`, and when it is not, `cell_extra_ofs[p] == cell_extra_ofs[p+1]` for every non-straddler, so the loop body runs only for the few dozen proxies that genuinely span a doorway (§1.4a: `office_3storey` has 114 rooms and 272 portals, of which 232 are doors and windows). Measured shape of the cost: 1.5 ns/candidate, the same as `layer_mask`, which is why `cell_mask_stage` is placed **immediately after `layer_mask` and before everything else** — it is the cheapest stage in the pipeline and, indoors, by far the most selective.

**Memory [D]:** `cell_id` 2 B × 200,000 = 400 KB; `cell_extra_ofs` 4 B × 200,001 = 800 KB, allocated **only** when the world has straddlers; `cell_extra` ≈ 2 B × a few hundred straddlers ≈ **under 1 KB**. The offsets array is the expensive half, which is why it is optional rather than always present.

**What this costs elsewhere:** `cull_result::cell_masked` already exists and is now actually produced. `vis_index_equivalence_test` is untouched — the quadtree lives on the seed side of the seam and swapping indices cannot change a cell mask. `vis_portal_test` gains the straddler case (§11.1). And `validate_manifest()` gains three checks: every `cell_id < 4096 || == 0xFFFF`, every `cell_extra` id likewise, and `cell_extra` ascending within each run.

### 3.5.6b Dynamic agents indoors — the locator, and its budget

The requirement is agents moving through static interiors, and §3.5.2 disposed of it in a clause. It does not survive contact with the type: **`cell_id` is a static per-proxy column the scene owns**, and nobody had said who rewrites it for a mover, how often, or at what price. Here is the answer.

**Who.** Not `cvc::vis`. The scene owns the table (`scene_view.h` says so in capitals), so the adapter owns the update: `CullAction` calls `vis::agent_locator::update()` immediately after `grid_index::rebuild()` and before `cull()`, writing into the mutable agent range of the `cell_id` column. `cvc::vis` supplies the locator; it never reaches out and reads a simulation.

**How often.** *Every frame, for every agent.* A stale cell id is exactly the failure mode §10.3 names — **latency breaks conservativeness** — except worse, because staleness here does not merely widen the set, it can name a cell that is no longer active and delete a visible agent. If the budget ever forces amortization (say a quarter of the agents per frame), the un-updated agents must fall back to `unpartitioned_cell`, **never** to their previous cell. Widening is legal; a stale exact answer is not.

**At what cost.** A naive per-agent `portal_graph::locate()` BSP descent is not free and is nowhere in §7.3's budget. Price it honestly. `bunker` is a depth-4 BSP (§1.4a) and an office core is deeper, say depth 8; each node is one plane dot product ≈ 4 ns scalar, so 32 ns/agent × 4,000 agents = **0.13 ms/frame [D]** — three times the entire existing agent line item. And the `office_3storey` plan comes from **Lopes growth, not a BSP** (Lab §6b.2 step 3), so the generator may ship no `bsp[]` at all for it; `locate()` then falls back to a linear scan over the storey's cells — 36 for I-L at ~6 ns each = **216 ns/agent**, seven times worse. The fallback existing is exactly why the fast path below is not an optimisation but the design. That is affordable but wasteful, and it is wasteful for a reason worth exploiting: **an agent moves 0.1 m per frame at 6 m/s and 60 fps, and it can only leave a cell through a portal.** The Lab's navigability gate (its §6b.3) *proves* the free space is portal-connected, and `BLOCKS_MOVEMENT` is exactly that guarantee in the schema. So:

```
locate_coherent(agent):
    c = prev_cell[agent]
    if c != 0xFFFF and inside(cells[c], p):        return c        # ~5 ns
    for q in cells[c].portals_with(!BLOCKS_MOVEMENT):              # <= 6 neighbours
        if inside(cells[q.other(c)], p):           return q.other(c)   # ~70 ns
    return locate(p)                               # full BSP descent, ~32 ns
```

`inside()` is an AABB reject (6 ns) then the convex hull's ≤ 12 planes, and it is a **3-D** test — cells carry `floor_z`/`ceil_z`, so an agent on a stairwell landing resolves by height without a special case, which is the same `VERTICAL_OPEN` data §3.5.4 item 4 already demands.

Miss rate **[E]**: at 0.1 m/frame across a ~6 m room an agent crosses a boundary roughly every 60 frames, i.e. **1.6%** of agent-frames take the neighbour path; the full descent runs only on spawn, teleport and scripted jumps, well under 0.01%.

| term | share | ns/agent | 4,000 agents |
|---|---|---|---|
| same cell (AABB + hull) | 98.4% | 5 | 0.020 ms |
| neighbour through a portal | 1.6% | 70 | 0.004 ms |
| full BSP descent | < 0.01% | 32 | ~0 |
| **agent cell locate, total [D]** | | **~6 ns** | **0.024 ms → budget 0.03 ms** |

Outdoors the same call returns `exterior_cell` on the first `inside()` test against the root and costs ~2 ns/agent, so the locator is not an indoor-only tax; it is 0.01 ms at vista and ground.

**Scale check.** 4,000 is the *outdoor* GRL-SNAM swarm §7.3 budgets, and 0.024 ms is therefore the worst case — every agent in the fleet indoors at once. §1.4a's actual interior populations are **8 (I-S) and 24 (I-L)**, at which the locator costs ~0.15 µs and disappears. The 4,000-agent figure is budgeted rather than the realistic one because the ceiling is what a budget is for, and because nothing in the design forbids driving the whole swarm through a warehouse.

**The 16 m `grid_index` is not this, and the two are orthogonal.** `grid_index` is a *broadphase* — it answers "which agents are in the frustum" by counting-sorting positions into a flat CSR grid, and it knows nothing about rooms. `cell_id` is a *mask* — it answers "is this agent in a cell the portal walk reached". Both run, in that order, and neither can substitute for the other: the grid cannot see a wall, and the mask cannot cull the agent standing in the same room but behind the camera. Confusing them is how one ends up believing a 16 m outdoor grid handles interiors.

**Agents do not cost actors per cell.** All visible agents are one instanced batch (or one merged dynamic actor on the D2 fallback), so agents contribute **exactly 1 actor** no matter how many cells they are spread across; the cell mask reduces *instances*, not actors, at the per-frame instance-buffer compaction the visible set already drives. This is why §5.5a's actor arithmetic charges one line for agents and not ten.

**One thing the generator must not do:** emit a mover as a batched static prop. The Lab places props after the navigability gate as immutable decoration, which is correct; anything intended to move must be emitted into the dynamic set at generation time, because a proxy inside a cell's merged shell/prop batch has no independent `cell_id` to rewrite.

### 3.5.7 Verdict: does v1 implement portals?

**Yes — portals ship in v1 (PR 8), and PVS never does.**

The defence:

| | Portals | PVS | BVH + occlusion only |
|---|---|---|---|
| Bake required | **none** | minutes-to-hours, re-run on every regeneration | none |
| Survives procedural per-episode worlds | **yes** | **no — disqualifying** | yes |
| Dynamic contents (agents, props, doors) | **yes** | no | yes |
| Indoor cull rate | **86.5% of visible actors [D]** (§12.1) | ~95% **[E]** | ~0% at batch granularity |
| Cost | **~11 µs [D]** | O(1) lookup | 1–4 ms if a rasterizer, ~0 if terrain-march (which is useless indoors) |
| Lines of code | ~400 | ~1,200 + a bake tool | 0 additional |
| Gives gameplay/AI visibility (which agent can see what) | **yes** | yes | no |

The alternative — "rely on BVH + occlusion" — fails on its own terms indoors: the terrain march is meaningless inside a building, and a convex-occluder-volume test does not fuse, so a room bounded by four separate wall hulls is not culled by any of them individually. **That admission is not confined to the indoor case, and §3.6 is where it is cashed out honestly: the same non-fusion, applied outdoors to a cluster of buildings, is one of the three reasons the occluder-volume stage does not ship at all.** A raster occluder would work but costs 1–4 ms to reproduce what an 11 µs graph walk gives exactly. **PVS is rejected on workflow, not on cost** — it is the cheapest runtime visibility ever shipped and we still cannot use it, because the bake is invalidated by the thing our world generator exists to do. If a fixed, hand-authored benchmark interior ever wants it, `pvs_lite` is a stage anyone can add (§10.1); it will never be the default and never on the training path.

The one honest caveat: **the indoor win is contingent on the generator emitting a usable cell/portal graph (§3.5.4).** PR 8 is therefore sequenced last among the functional PRs and is independent of everything before it — if the graph does not arrive, interiors render as ordinary quadtree content at **12.75 ms, which is 78 fps [D]** (§12.1) — clearing 60 fps but failing 120 fps, which is the honest statement of what PR 9 buys. Nothing else in the plan changes.

**Two corrections to the strength of that caveat, from §1.4a.** The fallback was previously quoted at ~15 ms / 66 fps, computed against a 400-room building the generator does not produce and cannot afford to gate (§12.1). At the real `office_3storey` inventory it clears 60 fps with 3.9 ms in hand. Conversely, at `bunker` scale the *interior* cull is nearly worthless (22 → 11 actors); 32 of the 43 actors portals remove there are **outdoor** actors cut by `frustum::narrowed_by` at the doorway. Both presets argue the same implementation priority: **§3.5.6's seam is the load-bearing half of the indoor design, not the interior traversal.**

---

## 3.6 Do buildings occlude exterior content? No — `occluder_volume_stage` is cut

> **Verdict: `occluder_volume_stage` does not ship in v1.** An earlier revision of this document listed it in seven places and gave it an algorithm in none of them. It is removed rather than written, and this section states the loss precisely instead of leaving a stage name to imply a capability.
>
> **What that costs, in one sentence:** *no building occludes any exterior content, ever; the terrain horizon march is the only occluder in the outdoor pipeline.* A tree standing behind a warehouse is drawn.

### 3.6.1 The three reasons, in the order that decides it

**1. The arithmetic. A building is not larger than a cell in the direction that matters.**

§1.5c established that batch-granularity occlusion works only when the occluder covers the *entire* projected AABB of a 32 m cell. Run that test on the Lab's actual building recipes (§6.7 of the Lab roadmap: `warehouse`, `office_3storey`, `bunker`).

Eye at `h_e = 1.7 m`. A batch cell at ground distance `D` with content top `z_top = 28 m` (mean tree 23.77 m plus in-cell terrain relief, the same figure §1.5c uses). A convex building hull of top height `H` at ground distance `d` on the sightline. Conservative culling requires *every* corner blocked, so in particular the top-far corner, whose sightline has height `h(d) = h_e + (z_top − h_e)·d/D`. Therefore:

```
    vertical:   d/D  <=  (H - 1.7) / (28 - 1.7)      # the binding constraint
    lateral:    d/D  <=  W / 45.3                    # 45.3 m = the 32 m cell's plan diagonal
```

| recipe | H | W | vertical limit `d/D` | lateral limit `d/D` | reads as |
|---|---|---|---|---|---|
| `office_3storey` (3 × 3.2 m + parapet) | 10.5 m | 25 m | **0.334** | 0.55 | culls only cells **> 3.0×** further out than itself |
| `warehouse` | 8.0 m | 30 m | **0.240** | 0.66 | culls only cells **> 4.2×** further out |
| `bunker` | 4.0 m | 32 m | **0.087** | 0.71 | culls only cells **> 11.5×** further out |

Vertical binds in every row, and it binds hard. This is the trunk-prism failure of §1.5a repeated at building scale: a 10.5 m occluder against a 28 m target box is 10.5/28 of the problem. **[D]**

**2. Non-fusion, confronted where it actually bites.** §3.5.7 admits that convex hulls do not fuse and confines the admission to interiors, where portals make it harmless. Outdoors it is not harmless, because the exterior case *is* the fusion case: a cluster of buildings occludes as a union of silhouettes, and a per-hull test can only claim what one hull covers alone. Quantified below.

**3. The input does not exist and the generator has already declined to produce it.** The stage consumes `occluders[]` — per-building inner-conservative convex hulls. The Lab roadmap's §6b.6 is explicitly a frozen contract and its "what we deliberately do not emit" list names occluder fusion, anti-portals and exterior cell decomposition; occluder hulls appear nowhere in what it emits. Keeping the stage means re-opening a frozen contract to request the one artefact §3.2 already identified as an **unbounded** correctness risk ("shrunk by roughly the area of one pixel… can potentially be unbounded… puts high requirements on the artists"). We would be trading a proof-free authored input for the win computed next.

### 3.6.2 The town, quantified — how much of the exterior win non-fusion destroys

Camera outdoors at eye height 1.7 m, looking into a town. Model it as rows of blocks `W = 25 m`, `H = 10.5 m`, gaps 15 m (per-row azimuthal fill 0.625), rows at `d = 40 / 80 / 120 / 160 m`, 63.1° hfov. For a target cell at distance `D`, only rows satisfying `d ≤ 0.334 D` can contribute at all.

- **Fused** (what MSOC, PVS or a portal graph would get): P(blocked) `= 1 − Π_rows (1 − 0.625)`.
- **Unfused** (what a per-hull convex test gets): the cell's angular width `45.3/D` must fit *inside one block's* angular width `W/d` within that row's angular period; contributing fraction per row `= (W − 45.3·d/D) / period`, and `P = 1 − Π_rows (1 − that)`.

| target `D` | qualifying rows | fused | single-hull | **fraction of the win lost** |
|---|---|---|---|---|
| 150 m | `d = 40` | 0.63 | 0.32 | **48%** |
| 250 m | `d = 40, 80` | 0.86 | 0.61 | **29%** |
| 400 m | `d = 40, 80, 120` | 0.95 | 0.79 | **17%** |

**[D]** from the model, **[E]** in that the town layout is assumed rather than generated.

Read the trend, because it is the opposite of reassuring: **the loss is worst at short range and shrinks with distance — i.e. it is worst exactly where a culled actor is a full-cost band-A/B actor, and mildest out past 350 m where the survivor was going to be an impostor or an HLOD sector proxy anyway.** Averaged over the actor cost at each range rather than over cells, **non-fusion removes roughly 40% of the available exterior building-occlusion win [E]**.

**And then the absolute number kills it independently of the fraction.** The Lab does not generate city blocks; `standard` puts a research station on Tern and `large` adds one building cluster on Shoal — call it 5 buildings at `d = 40…160 m`. Per building at `d = 60 m, W = 25 m`, the qualifying shadow wedge is 0.417 rad restricted to `D ≥ 180 m`, which lands in band B (140–350 m, ~128 m cells → ~1.3 cells) and band C (350–900 m, 512 m cells → ~0.55 cells). Five partly-overlapping wedges give **~6–8 candidate cells, of which the vertical rule leaves 4–6 [E]**:

```
    4-6 cells x 44.6 us/actor  =  0.18 - 0.27 ms saved,  on an 8.96 ms ground frame  =  ~2.5%
```

That is the whole exterior prize, before the stage's own cost, and it is inside the run-to-run spread the §11.3 harness is required to report.

### 3.6.3 The 0.9 µs, derived — and what the number was hiding

The earlier revision asserted "0.9 µs, conservative, no fusion" with no working. There is exactly one arithmetic path to 0.9. The standard Coorg–Teller formulation builds, per occluder, a shadow volume from the hull's silhouette edges w.r.t. the eye: a ≤ 10-halfspace hull has a 5–6 edge silhouette from a generic eye, so ~6 shadow planes, and an AABB-vs-6-planes p/n-vertex test is the **14 ns** §15.7 already lists for the frustum-AABB stage. Hence

```
    0.90 us / candidate  =  K x 14 ns   =>   K = 64 occluder hulls tested per candidate
```

So the printed figure silently assumed **K = 64, an undocumented constant**, with (a) no spatial pruning of the occluder set and (b) no accounting for the per-frame "best K occluders" selection — at §15.8's 6,000-triangle occluder budget a ≤ 10-halfspace hull is ~16 triangles, so ~375 candidate hulls, and sorting them by solid angle is ~10 µs/frame that appeared in no budget.

The honest total is therefore ~100 candidates × 0.9 µs + 10 µs ≈ **0.10 ms/frame [D]** — which is *cheap*. **Cost is not why this is cut.** It is cut because 0.10 ms of budget and ~300 lines and a new oracle buy 0.18–0.27 ms of a win that does not exist yet in the generator, that non-fusion has already taken ~40% off, and whose underlying cull rate is the **[E]** number question **Q2** was scheduled to measure in the first place. A stage whose predicted saving is within 2× of its own cost is not a stage; it is a rounding error with a test suite.

### 3.6.4 What survives the cut, what does not

| Case | v1 behaviour after the cut |
|---|---|
| Ridge or hillside between eye and content | **unchanged** — `terrain_horizon`, and §1.5 shows terrain supplies 34.1 of 34.6 occlusion points at ground and 49.5 of 49.5 behind a ridge |
| Standing outside a building, its furniture and interior props inside | **unchanged, and this is the case people actually mean.** `portal_seed` rejects proxies whose `cell_id` names an interior cell not reached through a visible doorway (§3.5.6). The building shell hides its own contents by *topology*, never by an occluder test |
| Standing inside, looking at another room | **unchanged** — portal traversal, 86.5% of visible actors, ~11 µs (§12.1) |
| Standing inside, looking out a window at the town | **unchanged** — the narrowed frustum hand-off to `index_seed` |
| **A tree, rock or vegetation cell standing behind a building, outdoors** | **LOST. It is drawn.** ~4–6 extra actors ≈ 0.18–0.27 ms in the ground regime near a building cluster **[E]** |
| **One building behind another, outdoors** | **LOST.** Bounded by the building count: `standard` has a research station, `large` one extra cluster. Worst case a handful of actors |

Both losses are bounded by the same thing — the generator emits a handful of buildings, not a city — and neither touches the vista, ridge, forest or indoor regimes on which every headline number in §0 rests. **No number in §0, §11.4 or §12 changes as a result of this cut**, because the earlier revision never attributed a millisecond to this stage in any of them; that absence is itself the tell.

### 3.6.5 The condition that reverses this

Not a hedge — a trigger with a number attached. **If question Q2 returns a batch-granularity cull rate ≥ 60% (vs the design's ~10% [E]), the correct response is MSOC, not convex hulls** (decision **D9**). MSOC fuses, needs no authored inner-conservative hull (it rasterizes the shell mesh the generator already emits), and §15.7 costs it at 11.9 ns/occluder-tri + 120 ns/test. The convex-hull stage occupies a niche where it is strictly dominated: it needs the same occluders-larger-than-a-cell precondition MSOC needs, *and* it cannot fuse, *and* it needs an input MSOC does not. Adding it back is not the cheap option; it is the option that costs the same and works less well.

Second, weaker trigger: **if the Lab's world model ever grows a dense urban preset** — contiguous street canyons rather than a research station — §3.6.2's fused column rises toward 1.0 and the absolute cell count rises with it. At that point re-run §3.6.2 with the real layout. It is still MSOC that gets built.

---

## 4. Architecture — the `cvc::vis` module

### 4.1 Module boundary and file manifest

Three layers, deliberately separated. Every engine surveyed that fused them had to un-fuse them later: OGRE 1.x made the whole `SceneManager` the plugin point (a ~3,460-line god class), and by Ogre 2.x had collapsed to one manager with terrain demoted from a `SceneManager` subclass to a *component*; Godot 3's rooms-and-portals culler had to patch renderer internals to be fast and was dropped rather than ported to Godot 4.

```
  layer (a)  culler / cull_pipeline    per-view PURE FUNCTION, conservative by contract
  layer (b)  spatial_index             MUTABLE state across frames, OPTIONAL, stage-owned
  layer (c)  query_ray / query_aabb    consumer-facing (picking, nav sensor cones, LOS)
```

The seam that matters is the *input*: a culler never sees a scene graph. It sees a flat SoA table the caller owns and maintains. That inversion is what Unreal (`Scene->PrimitiveBounds`), Frostbite (grid blocks) and Unity (`BatchCullingContext`) all independently arrived at, and what OGRE got wrong.

```
inc/cvc/vis/
  types.h            proxy_id, aabb, sphere, plane, frustum (AoS + SoA), cull_result, regime
  scene_view.h       the SoA proxy table -- the ONLY thing a culler ever sees
  view_params.h      one view: matrices, frustum, k_px, thresholds, frame, deterministic
  visible_set.h      sorted ids + parallel dist2 / screen_px / lod payload + world bounds
  cull_scratch.h     explicitly-defined per-thread arena + the frame payload columns
  cull_stage.h       the four-rule stage contract  (READ THIS FIRST)
  cull_seed.h        seed interface: produces the initial candidate list (index or portal)
  cull_pipeline.h    culler, cull_pipeline, stage_trace
  stages.h           layer_mask, cell_mask, distance_size, frustum, aabb_frustum,
                     small_feature,
                     terrain_horizon         (occluder_volume: CUT, SS3.6)
  spatial_index.h    abstract: rebuild / update / query_frustum / query_aabb / query_ray
  linear_index.h     no structure -- the query oracle, and correct below ~1,000 proxies
  quadtree_index.h   XZ quadtree with per-node Y interval        <- static content
  grid_index.h       flat CSR uniform grid, counting-sort rebuild <- moving agents
  terrain_field.h    heightfield + min-mip & max-mip pyramids + conservative march
  portal_graph.h     cells, portals, locate/locate_multi/locate_coherent, agent_locator,
                     portal_seed, validate_manifest      <- the ONLY per-proxy partition
  lod.h              lod_ladder, screen-radius bands, hysteresis, fade schedule
  hlod.h             sector proxy descriptors (merged shell / impostor / baked canopy)
  regime.h           metrics (incl. enclosure_n + aperture_fraction), classifier,
                     governor policy, per-regime demotion state, hysteresis, pinning
  registry.h         culler_traits + registry (hung off cvc::app), recommend()
  reference.h        reference_culler oracle, conservativeness_violations, describe_build
  manifest.h         provenance record written beside every generated dataset

src/cvc/vis/         one .cpp per header; frustum_simd.cpp is a REAL TU with a runtime
                     switch, never an #if-selected inline (coverage-gate reason, SS11.6)

inc/cvc/gl/ + src/cvcGL/            the VTK-facing adapter (thin, replaceable)
  ViewParams.h/.cpp      vtkRenderer/vtkCamera -> cvc::vis::view_params        ~80 lines
  VisCuller.h/.cpp       vtkCuller subclass reading a precomputed bitset       ~120
  CullAction.h/.cpp      cvc::gl Action(Kind::Custom) filling a scene_view     ~300
  BatchedScene.h/.cpp    cell -> batch actor, LOD swap, residency, fades       ~700
  SwayShader.h/.cpp      vertex-shader sway via addVertexShaderReplacement     ~200

src/cvcGL/examples/
  lsystem_lab.cpp        the NEW example (lsystem_forest.cpp is untouched)
  vis_bake.cpp           offline impostor/proxy baker -- keeps FiltersCore OUT of cvcGL
```

`src/cvcGL/CMakeLists.txt` uses `file(GLOB CVCGL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")` (verified, line 34), so **every cvcGL-side source file lands with no CMake edit at all** — which keeps us off line 24 (`CVCGL_VTK_COMPONENTS`) and lines 281–292 (the examples block), both hot for PR #223.

### 4.2 Core headers

```cpp
// inc/cvc/vis/types.h
#ifndef __CVC_VIS_TYPES_H__
#define __CVC_VIS_TYPES_H__
#include <cstdint>
namespace cvc { namespace vis {

using proxy_id = std::uint32_t;                 // dense index into a scene_view
inline constexpr proxy_id invalid_proxy = ~0u;

struct aabb   { float mn[3]; float mx[3]; };
struct sphere { float c[3];  float r;     };
struct plane  { float n[3];  float d;     };    // inside iff  n.x + d >= 0

// Plane order is OURS: L, R, B, T, NEAR, FAR.
// vtkCamera::GetFrustumPlanes emits L, R, B, T, FAR, NEAR -- the header warns
// it is "NOT near,far". from_vtk_planes() reorders. Getting this wrong silently
// inverts near/far clipping, which is why it lives in one named function.
struct frustum {
  plane p[6];
  float soa[8][4];        // [x0123, y0123, z0123, w0123, x4545, y4545, z4545, w4545]

  static frustum from_vtk_planes(const double planes24[24]);
  static frustum from_view_proj(const double m[16]);          // Gribb-Hartmann

  // Narrow by a convex portal polygon (screen-space AABB intersection).
  // Returns false if the result is empty. This is what makes the indoor/outdoor
  // hand-off composition rather than a special case (SS3.5.6).
  bool narrowed_by(const float poly[][3], int n, const float eye[3],
                   frustum& out) const;
};

enum class cull_result : std::uint8_t {
  visible = 0, layer_masked, cell_masked, distance_culled,
  frustum_culled, small_feature, terrain_occluded, volume_occluded, portal_unreached
};

// `seam` is NOT "unclassified". It is the state in which the frame pays for BOTH
// partitions at once -- camera in two cells, or in one cell with a large aperture on
// the exterior, or outside looking into a room through a visible doorway. It has a
// classifier (SS10.4), a governor policy of its own (SS10.4a), a budget column
// (SS12.1) and a test (SS11.1). A boolean `enclosure` cannot express it, which is
// why the metric is a COUNT.
enum class regime : std::uint8_t { unknown = 0, vista, ground, ridge, indoor, seam };
}}
#endif
```

```cpp
// inc/cvc/vis/scene_view.h
// Borrowed SoA arrays the CALLER owns. Deliberately NOT a scene graph.
//
// KEEPING THIS TABLE CURRENT IS THE SCENE'S JOB, NEVER THE CULLER'S.
struct scene_view {
  std::size_t count = 0;

  // Bounding spheres, SoA. REQUIRED -- every stage can fall back to these.
  const float *cx = nullptr, *cy = nullptr, *cz = nullptr, *radius = nullptr;

  // Optional tighter world AABBs, SoA. Stages check for null.
  const float *bmnx=nullptr,*bmny=nullptr,*bmnz=nullptr;
  const float *bmxx=nullptr,*bmxy=nullptr,*bmxz=nullptr;

  const std::uint32_t *layer_mask    = nullptr;  // null => 0xFFFFFFFF; maps 1:1 onto
                                                 // cvc::gl::VisibilityElement::mask
  const std::uint8_t  *culling_active= nullptr;  // null => all cullable
  // THE portal-cell column, and the only per-proxy partition (SS3.5.6a). A quadtree
  // leaf is NEVER stored here: leaf membership is the seed's contiguous Morton run.
  // 0 == exterior; 0xFFFF == unpartitioned/always-active; 1..4095 == interior cells.
  // May be null for a purely outdoor world, in which case cell_mask_stage is absent.
  const std::uint16_t *cell_id       = nullptr;
  // Straddlers only: a proxy in >= 2 cells. CSR, emitted once at load, never rebuilt.
  // Both null or both non-null. Agents are NOT straddlers; their cell_id is rewritten
  // per frame by vis::agent_locator, which the SCENE calls, never the culler (SS3.5.6b).
  const std::uint32_t *cell_extra_ofs= nullptr;  // count+1 offsets, or null
  const std::uint16_t *cell_extra    = nullptr;  // extra cell ids, ascending
  const std::uint8_t  *asset_class   = nullptr;  // indexes the lod_ladder table
  // NOT PRESENT in v1: there was an is_occluder column here, read only by the cut
  // occluder_volume_stage (SS3.6). A future MSOC stage would re-add it.

  // THE COST MODEL LIVES IN THE CONTRACT. These let a stage report its saving in
  // MILLISECONDS rather than in object counts, which is the difference between an
  // honest tightness number and a misleading one (SS11.3). Without them the
  // governor and the harness would need a renderer-side side-channel.
  const std::uint32_t *anim_verts    = nullptr;  // CPU-animated verts if drawn at LOD0
  const std::uint32_t *tri_count     = nullptr;  // triangles if drawn at LOD0

  const std::uint64_t *stable_id     = nullptr;  // null => the index IS the id
  std::uint64_t revision = 0;                    // indexes cache on this; stale asserts
};
```

```cpp
// inc/cvc/vis/view_params.h
struct view_params {
  double  view[16], proj[16];
  frustum world_frustum;
  float   eye[3], forward[3];

  // k = (H_px / 2) / tan(vfov / 2).  screen_px(size, d) = k * size / d.
  // 1280x800 @ 42 deg vfov  ->  k = 400 / tan(21 deg) = 1042 px.m/m.
  float k_px             = 1042.f;

  float min_screen_px    = 1.5f;    // small-feature threshold; 0 disables
  float lod_error_px     = 4.0f;    // tau: geometric-error LOD threshold
  float lod_hysteresis   = 0.08f;   // +-8% deadband on band edges
  float impostor_px      = 32.0f;   // representation change, not error-driven
  float max_distance     = 0.f;     // 0 disables the hard far cut

  std::uint32_t layer_mask = 0xFFFFFFFFu;

  // EXPLICIT. Nothing in cvc::vis reads a clock or a global frame counter.
  // Hysteresis, fades, and the governor all step from this.
  std::uint64_t frame = 0;

  // Forces scalar reference kernels, disables fades, and freezes the governor
  // at its manifest-pinned configuration. Set this for dataset generation.
  bool   deterministic = false;
  regime pinned_regime = regime::unknown;   // != unknown => classifier disabled
};
```

```cpp
// inc/cvc/vis/cull_scratch.h
// EXPLICITLY DEFINED, because three things depend on knowing exactly what it is:
// stage purity (rule 4), thread-count invariance, and where LOD payload lives.
//
//  * Owned by the caller, reused across frames, never allocated in the hot loop.
//  * One instance per worker thread. A stage NEVER shares scratch across threads.
//  * The payload columns are indexed by proxy_id, sized to scene_view::count,
//    and validity-stamped by `frame` so no clear pass is needed.
//  * Rule 4 (purity) applies to scratch as well as to `out`: after a run, the
//    scratch columns touched by the pipeline must be a pure function of the
//    inputs. vis_determinism_test hashes them, not just the visible_set.
struct cull_scratch {
  std::vector<proxy_id>     work_a, work_b;      // ping-pong candidate buffers
  std::vector<std::uint64_t> stamp;              // per-proxy validity stamp
  std::vector<float>        dist2, screen_px;    // written by distance/frustum stages
  std::vector<std::uint8_t> lod;                 // written by the LOD selector
  std::vector<std::uint32_t> view_mask;          // cull_multi: bit i == passed view i

  void bind(const scene_view&, std::uint64_t frame);   // resize + stamp, no clear
  std::uint64_t hash() const;                          // determinism test hook
};
```

```cpp
// inc/cvc/vis/cull_stage.h
// ---------------------------------------------------------------------------
// THE CONTRACT.  Asserted in debug builds after EVERY stage.
//
//   1. out is a subset of in            -- a filter stage may only REMOVE
//   2. out is sorted ascending          -- canonical form, always
//   3. out is a superset of (truly-visible AND in)          [conservative]
//   4. pure: same (scene_view, view_params, in) => byte-identical out AND
//      byte-identical touched scratch, for ANY thread count and ANY SIMD path
//
// (3) composes for free: an intersection of supersets of the visible set is a
// superset. A pipeline of conservative stages is therefore conservative with no
// additional proof obligation. That -- not architectural taste -- is why the
// unit of pluggability is a STAGE and not a whole culler.
//
// Taxonomy per [Cohen-Or et al. 2003]: conservative (superset) / exact / approximate
// (sampled) / aggressive (deliberately gives up conservativeness for tightness).
// LATENCY BREAKS CONSERVATIVENESS. Any stage whose answer is stale -- an occlusion
// query readback, a reprojected depth buffer, an async result -- MUST return
// conservative() == false. No stage may be quietly non-conservative because of timing.
// ---------------------------------------------------------------------------
class cull_stage {
public:
  virtual ~cull_stage() = default;
  virtual const char* name() const noexcept = 0;
  virtual float cost_hint() const noexcept { return 1.f; }   // ns per candidate
  virtual bool  conservative() const noexcept { return true; }
  virtual bool  wasm_capable() const noexcept { return true; }

  virtual void run(const scene_view&, const view_params&,
                   std::span<const proxy_id> in,
                   std::vector<proxy_id>&    out,
                   cull_scratch&             scratch) const = 0;

  // Multi-view. The DEFAULT implementation loops run() per view and ORs bits, so
  // a third-party stage gets multi-view for free and correctly. frustum_stage
  // overrides it with the SoA kernel that tests N frusta in one pass, which is
  // what makes main camera + shadow cascades + sensor cones cost ONE traversal.
  // `view_mask` is indexed by POSITION in `in`.
  virtual bool supports_multi_view() const noexcept { return false; }
  virtual void run_multi(const scene_view&, std::span<const view_params>,
                         std::span<const proxy_id> in,
                         std::span<std::uint32_t>  view_mask,
                         cull_scratch&) const;
};
```

```cpp
// inc/cvc/vis/cull_seed.h
// A SEED produces the initial candidate list; a STAGE narrows one. Splitting the
// two is what makes portal traversal expressible without violating rule 1 -- a
// portal walk EMITS the contents of reachable cells, which is an "add", and an
// add is illegal for a filter. Exactly one seed runs, first.
//
// A seed may compose other seeds: portal_seed hands the narrowed frustum to
// index_seed when the traversal reaches the exterior cell (SS3.5.6).
class cull_seed {
public:
  virtual ~cull_seed() = default;
  virtual const char* name() const noexcept = 0;
  virtual bool conservative() const noexcept { return true; }
  virtual void seed(const scene_view&, const view_params&,
                    std::vector<proxy_id>& out,          // sorted, deduped
                    cull_scratch&) const = 0;
};
```

```cpp
// inc/cvc/vis/cull_pipeline.h
class culler {
public:
  virtual ~culler() = default;
  virtual const char* name() const noexcept = 0;
  virtual void cull(const scene_view&, const view_params&, visible_set&) const = 0;

  // ONE pass over the data for N frusta, writing a per-proxy bitmask of which
  // views survived. This is Frostbite's `visibleViews`, and here it is ALSO the
  // shadow-correctness mechanism: a prop is only SetVisibility(0) when it fails
  // the camera view AND every shadow view (SS8.3).
  virtual void cull_multi(const scene_view&, std::span<const view_params>,
                          std::span<std::uint32_t> view_mask_per_proxy) const;
};

class cull_pipeline final : public culler {
public:
  cull_pipeline& set_seed(std::shared_ptr<cull_seed>);
  cull_pipeline& add(std::shared_ptr<cull_stage>);
  cull_pipeline& sort_by_cost();          // cheap-first, honours cost_hint()
  cull_pipeline& set_time_budget(double ms);   // wasm main-thread guard (SS9.3)

  void cull(const scene_view&, const view_params&, visible_set&) const override;

  // Per-stage in/out/ms => TIGHTNESS, for free. The survey literature notes this
  // metric is almost never reported [Cohen-Or et al. 2003]; it is the number that
  // says whether a stage earns its cost.
  struct stage_trace {
    const char* name; std::size_t in, out; double ms;
    double tightness;                     // out / oracle, populated in validation runs
  };
  std::span<const stage_trace> last_trace() const;
  bool conservative() const noexcept;     // AND over seed and stages
};
```

```cpp
// inc/cvc/vis/visible_set.h
// SORTED ASCENDING IS PART OF THE CONTRACT. It is what makes output comparable
// across strategies, thread counts, SIMD paths and runs -- i.e. what makes the
// subsystem testable and the GRL-SNAM training data reproducible. An unordered
// container here would silently destroy every property test in SS11.
class visible_set {
public:
  void clear(); void reserve(std::size_t);
  void push(proxy_id, float dist2, float screen_px, std::uint8_t lod);
  void finalize();                                  // sort by id, dedupe, mark sorted

  std::span<const proxy_id>     ids()       const;
  std::span<const float>        dist2()     const;
  std::span<const float>        screen_px() const;
  std::span<const std::uint8_t> lod()       const;  // already chosen; no revisit

  // Aggregates the pipeline computed anyway -- the governor's and harness's inputs.
  std::uint64_t visible_tris() const;
  std::uint64_t visible_anim_verts() const;
  // AABB of everything visible. Feeds the camera clipping range directly,
  // replacing vtkRenderer::ResetCameraClippingRange (measured 0.89 ms/frame).
  void world_bounds(double out[6]) const;

  bool contains(proxy_id) const;          // binary search -- POST-batch only, never
                                          // a per-object query on the hot path
  bool is_subset_of(const visible_set&) const;
  std::uint64_t hash() const;             // order-stable, golden-file friendly
};
```

```cpp
// inc/cvc/vis/reference.h
// SHIPS IN THE LIBRARY, not in the test tree. Consumers enable it as a debug
// check; the training pipeline uses it to CERTIFY a dataset. No engine surveyed
// -- OGRE, OSG, Unreal, Unity/Umbra, Godot -- offers this, and it is the single
// thing our use case most needs.
class reference_culler final : public culler {   // brute force, double, scalar,
public:                                          // no index, no threads, no SIMD
  const char* name() const noexcept override { return "reference"; }
  void cull(const scene_view&, const view_params&, visible_set&) const override;
};

// Ids the candidate culled but the oracle kept. Empty == conservative on this
// input. THIS FUNCTION IS THE CORRECTNESS CONTRACT, EXECUTABLE.
std::vector<proxy_id> conservativeness_violations(const visible_set& candidate,
                                                 const visible_set& oracle);

// Screen-space ground truth for occlusion stages, where a frustum oracle cannot
// help: render offscreen with unique-id colours, pass the id buffer. SAMPLED --
// it yields counterexamples, never proofs. (SS11.2)
visible_set visible_from_id_buffer(std::span<const std::uint32_t> id_pixels);

// Provenance for the run manifest: which SIMD path was actually taken, FP flags,
// library revision. Recorded beside every generated dataset.
struct build_info { const char* simd_path; bool fp_contract_off; const char* revision; };
build_info describe_build();
```

### 4.3 The pipeline, drawn

```
  WORLD GENERATOR (owned elsewhere -- SS3.5.5)
  emits: Morton-sorted instances | cells + portals | heightfield
         | per-species LOD ladder (derivation depth, radius_scale, err)
         (NO occluder hulls, NO trunk prisms -- withdrawn, SS3.5.5 / SS3.6)
        |
        v  once, at load
  +-------------------------------------------------------------------------+
  |  cvc::vis  (headless: no VTK, no GL, no Qt, no Boost)                    |
  |                                                                          |
  |  quadtree_index (static, XZ + Y interval)   terrain_field (min/max-mip)  |
  |  grid_index (agents, counting sort/frame)   portal_graph (cells+portals) |
  |  agent_locator (coherent per-agent cell id, ~6 ns/agent -- SS3.5.6b)     |
  +-------------------------------------------------------------------------+
        |
        |  per frame:  scene_view (SoA bounds)  +  view_params (frustum, k_px, frame)
        v
  +-------------------------------------------------------------------------+
  |  cull_pipeline                                                           |
  |                                                                          |
  |   SEED         [ portal_seed ] --reaches EXTERIOR--> [ index_seed ]      |
  |                       |  narrowed frustum                |              |
  |                       +----------------+-----------------+              |
  |                                        v                                 |
  |   FILTER  layer_mask -> cell_mask -> distance_size -> frustum(SIMD)      |
  |              1.5ns         1.5ns          3ns            6ns/view        |
  |                                     -> small_feature                     |
  |                                             4ns                          |
  |                                        |                                 |
  |                                        v   [regime governor may enable]  |
  |                          terrain_horizon (1.4us/candidate)               |
  |             THE ONLY OCCLUSION STAGE OUTDOORS -- SS3.6 cut the other one |
  |                                        |                                 |
  |                                        v                                 |
  |   LOD SELECT   (reuses the screen radius the frustum stage computed)     |
  +----------------------------------------|--------------------------------+
                                           v
                              visible_set  { sorted ids, dist2, screen_px, lod,
                                             visible_tris, anim_verts, world_bounds }
        |
        v
  +-------------------------------------------------------------------------+
  |  cvcGL adapter                                                           |
  |    CullAction    fills scene_view from the graph, caches the composed 4x4 |
  |    BatchedScene  LOD swap + cross-fade + residency; prop visibility bitset|
  |    VisCuller     vtkCuller: reads the bitset, truncates listLength.       |
  |                  Calls NO GetBounds(). VTK's default culler is REMOVED.   |
  |    SwayShader    per-instance sway phase; 2 uniforms/frame, 0 CPU verts   |
  +-------------------------------------------------------------------------+
        |
        v
     VTK renders: window, context, camera, depth, volumes, HUD, ImGui overlay
```

### 4.4 Usage

```cpp
// src/cvcGL/examples/lsystem_lab.cpp  (abridged -- this is the whole integration)
using namespace cvc;

gl::SceneGraph     scene;
gl::SceneRenderer  view(scene, 1280, 800);

// 1. Load or generate the world. The generator emits the manifest; the visibility
//    system never rediscovers structure from triangle soup.
vis::world_manifest w = lsyslab::generate(seed, lsyslab::preset::island_with_ruins);

vis::quadtree_index veg(w.instances(), {.root_extent_m = 4096.f, .leaf_size_m = 32.f});
vis::terrain_field  terrain = vis::terrain_field::from_heightfield(w.height(), 4096.0);
vis::portal_graph   portals = vis::portal_graph::load(w.cells(), w.portals());
vis::grid_index     agents(/*cell_m*/ 16.f);       // BROADPHASE: which agents are in view
vis::agent_locator  agentCells(portals);           // MASK: which cell each agent is in

// 2. Compose the pipeline. Advisory selection, LOGGED and PINNED in the manifest.
vis::registry& reg = vis::visibility_registry(app);
auto rec = reg.recommend(w.stats());
CVC_LOG("cvc::vis: %s (%s)", rec.culler.c_str(), rec.reason.c_str());

auto pipe = std::make_shared<vis::cull_pipeline>();
pipe->set_seed(std::make_shared<vis::portal_seed>(portals, veg))   // degenerates to
    ->add(std::make_shared<vis::layer_mask_stage>())               // index_seed if
    ->add(std::make_shared<vis::cell_mask_stage>(portals))         // no portals exist
    ->add(std::make_shared<vis::distance_size_stage>())            // no portals exist
    ->add(std::make_shared<vis::frustum_stage>())
    ->add(std::make_shared<vis::small_feature_stage>())
    ->add(std::make_shared<vis::terrain_horizon_stage>(terrain));
// That is the whole outdoor occlusion pipeline. There is deliberately no
// occluder_volume_stage: buildings do not occlude exterior content (SS3.6).

vis::governor gov(vis::governor_policy::defaults());   // may disable stages; SS10.4

// 3. Take VTK's default culler OFF. It charges an O(N) GetBounds() point scan per
//    prop per frame on animated meshes and, on a 2-prop scene, culls nothing.
view.renderer()->GetCullers()->RemoveAllItems();
auto visCuller = gl::VisCuller::New();
view.renderer()->AddCuller(visCuller);

gl::BatchedScene batches(scene, veg, w.lod_ladder(), gl::residency_budget::from_vram());
vis::visible_set vs;

// 4. Frame loop.
while (view.isOpen()) {
  agents.rebuild(sim.agent_view());                     // counting sort, ~35 us
  agentCells.update(sim.agent_view(), batches.mutableCellIds());  // ~24 us, SS3.5.6b
  vis::view_params vp = gl::makeViewParams(view, frameNo);
  vis::view_params shadow = gl::makeShadowViewParams(view, sun, vs.world_bounds());

  std::uint32_t view_mask[2 * kMaxProxies];
  gov.configure(*pipe, vp);                             // regime -> stage enable/disable
  pipe->cull_multi(batches.sceneView(), {vp, shadow}, view_mask);
  pipe->cull(batches.sceneView(), vp, vs);

  batches.apply(vs, view_mask);        // LOD swap + fade + residency + SetVisibility
  visCuller->setVisible(batches.propBitset());
  view.setClippingRangeFromBounds(vs.world_bounds());   // replaces ResetCameraClippingRange
  sway.setTime(clock.t());                              // 2 uniforms; no CPU vertex work
  view.render();
}
```

Nothing in that loop does per-plant work, re-poses a vertex, calls `updateVertices`, or calls `ResetCameraClippingRange`. That is the design in one screenful.

---

## 5. Resolving the batching-vs-culling tension

### 5.1 The tension, stated exactly

The existing demo merges trees into two actors because **1 visible actor ≡ 223,000 triangles**. But a merged actor is culled all-or-nothing, and its AABB spans the island, so nothing ever culls. Worse — and this is the part that is usually missed — **the entire measured CPU cost is per-*mesh*, not per-*prop***: culling 90% of trees at prop granularity saves *nothing*, because `reposeTree`, the by-value `std::vector<double>` copy in `updateVertices`, the scalar `SetPoint` loop, and the full VBO re-upload all still run over 100% of the buffer.

**Any culling design that only removes props is a no-op on this baseline.**

### 5.2 The resolution: three collapsed concepts, and one dissolved trade

```
   quadtree leaf  ==  one batch actor  ==  one LOD unit
                  ==  one cull decision  ==  one residency/paging unit
```

Two mechanisms make this work:

1. **The cull result gates the batch's *update*, not only its *draw*.** `BatchedScene::apply(visible_set)` decides which cells are built, evicted, LOD-swapped, faded and uploaded. A culled cell costs nothing at all — not even a mapper `Update()`. That is what converts culling into *CPU* savings rather than triangle savings.
2. **Instancing, not merging, where it is available.** Instancing decouples the draw-call unit from the culling unit entirely, so the tension **dissolves** rather than being traded: one draw per (species, cell, LOD) *and* per-instance LOD *and* per-cell culling *and* per-instance identity for GRL-SNAM labels. Merging permanently destroys the last three. See decision **D2** and the PR-0 spike (§13) — cvcGL has no `vtkGlyph3DMapper` usage today and the recon's glyph benchmark was invalid, so this is measured before it is committed to, with a fully-costed merged-actor fallback.

### 5.3 Deriving the cell size

Two opposing costs, and they are opposed in the *right* way once GPU sway removes the animated-vertex term:

- **Coarse cells** → fewer batches → less per-actor submit (44.6 µs each).
- **Coarse cells** → looser AABBs → boundary over-draw. Over-draw costs triangles (0.2 ns each, a 77 M/frame budget) and, in band A only, GPU-swayed vertices.

Frustum geometry at 1280×800 / 42° vfov: hfov = 2·atan(tan 21° × 1.6) = **63.1°**, i.e. **17.53%** of a disc; θ = 1.101 rad. Plant density = 200,000 / 1.45×10⁶ m² = **0.138 plants/m²**.

For an annular wedge between R₀ and R₁ at cell size S:

```
cells(S) = 0.1753 * pi * (R1^2 - R0^2) / S^2        interior
         + ( 2*(R1 - R0) + 1.101*(R1 + R0) ) / S    boundary
cost(S)  = 0.0446 ms * cells(S)  +  0.0002 ms/Mtri * cells(S) * plants_per_cell * tris_per_plant
```

Band A (R = 0–140 m, L0 at 2,055 tris/plant), swept **[D]**:

| S | interior | boundary | cells | plants drawn | M tris | actor ms | tri ms | **total** |
|---|---|---|---|---|---|---|---|---|
| 16 m | 42.2 | 27.1 | 69 | 2,436 | 5.01 | 3.08 | 1.00 | 4.08 |
| **32 m** | **10.5** | **13.6** | **24** | **3,384** | **6.95** | **1.07** | **1.39** | **2.46** |
| 64 m | 2.6 | 6.8 | 9 | 5,085 | 10.45 | 0.40 | 2.09 | **2.49** |
| 128 m | 0.7 | 3.4 | 4 | 9,040 | 18.57 | 0.18 | 3.71 | 3.89 |

**The optimum is broad and flat between 32 m and 64 m (2.46 vs 2.49 ms).** That flatness is the important result, not the minimum: it means the cell size is not delicately tuned and will not be invalidated by a ±30% change in the per-actor coefficient on other hardware.

We pick **32 m** for band A on two tie-breaks the cost function does not see: (a) LOD-band granularity — a 140 m band needs ≥4 rings so the transition does not read as a visible step, and 32 m gives 4.4 while 64 m gives 2.2; and (b) band A is the only band with GPU sway, and sway cost scales with *drawn* plants, so the 1.5× lower over-draw at 32 m matters on a GPU term we have not measured (open question **Q3**).

**The upper constraint is therefore LOD granularity, not the actor budget** — and a quadtree satisfies both at once by making cell size level-dependent, which is a property a uniform grid structurally cannot provide. **This, not tightness, is the decisive argument for quadtree over grid in this design.**

### 5.4 The shipped band table

| Band | Range | Quadtree level | Cell | Cells visible **[D]** | Plants drawn | Representation | M tris |
|---|---|---|---|---|---|---|---|
| A | 0–140 m | L7 | 32 m | 24 | 3,384 | instanced L0 (2,055 tri), **GPU sway** | 6.95 |
| B | 140–350 m | L6 | 64 m | 29 | 16,385 | instanced L1 (560 tri), no sway | 9.18 |
| C | 350–900 m | L4 | 256 m | 15 | 33,900 | 2-quad cross impostor (4 tri) | 0.14 |
| D | > 900 m | L3 | 512 m | 5 | — | **baked HLOD canopy shell** (~20k tri) | 0.10 |
| terrain | — | L5 | 128 m | 20 | — | mip-morph chunks | 0.08 |
| agents + misc | — | grid 16 m | — | 7 | — | instanced / HUD / sky / sea | 0.05 |
| **Total, ground** | | | | **100 actors** | | | **16.5 M** |

Vista, same world **[D]**: bands A–C are empty, so 9 HLOD shells + 20 terrain chunks + 6 misc = **35 actors, 0.26 M triangles**.

### 5.5 The rule, generalized

> **Merge or instance as much as you like *within* a spatial cell; never across cells. Choose the leaf size so the actor cost of the in-frustum cell count and the triangle cost of the boundary over-draw are within a factor of two of each other, then round *down* to the nearest power-of-two subdivision that gives ≥4 cells across the narrowest LOD band.**

`vis::recommend_leaf_size(scene_stats, measured_coefficients)` computes this and logs the derivation. Note that it consumes **measured** coefficients from `vis_bench --calibrate` (§11.3), not hard-coded constants, so the batch size self-tunes across the heterogeneous builder fleet.

### 5.5a The indoor rule — the cell is given, so the free variable inverts

Everything in §5.3–§5.5 derives a *leaf size* from an annular-wedge cost function at 0.138 plants/m². **None of it applies indoors, and §15.6's generalized `recommend_leaf_size` must never be called for an interior.** A portal cell is an architectural room emitted by a subdivider whose split planes are pinned to wall planes (§3.5.4 requirement 1); it is whatever size the floor plan says, and the runtime does not get a vote. §7.5 settles the indoor cost *coefficient* from the GL-state side. This subsection settles the indoor *actor count*, which is the quantity every number in §1.4a and §12.1 is multiplied by.

**The free variable inverts.** Outdoors we choose `S` and accept the cell count that follows. Indoors the cell count is given and we choose `k`, **the actors a cell contributes**.

#### The identity

```
   portal cell  ==  1 shell batch  +  1 prop batch (rooms only)
                 +  1 glass batch  (only if the cell is glazed)
                ==  1 LOD unit  ==  1 cull decision  ==  1 residency unit
```

Same shape as §5.2's outdoor identity, with `k` pinned at 1–3 by the argument below rather than swept from a cost curve. Corridors and the vertical core carry no props and contribute one actor; a glazed office contributes three.

#### Never more than that: a cell is 36–86× under the split threshold

Splitting a cell further buys nothing that culling has not already bought. A portal cell is convex, so the narrowed frustum either reaches it or does not — and, the structural difference from outdoors, **the boundary over-draw term is identically zero**, because a cell boundary is a *wall*, not an arbitrary line through a forest. What remains:

```
split further only if   c_actor  <  c_tri * E[triangles the narrowed frustum rejects]
at c_actor = 44.6 us and c_tri = 0.2 ns:      E[rejected]  >  223,000 tri
```

Against the §1.4a inventory — the first version of this document to have one:

| | mean interior triangles per cell | threshold | margin |
|---|---|---|---|
| **I-S** `bunker` | 18.1 k / 7 reached = **2,590** | 223,000 | **86×** |
| **I-L** `office_3storey` | 598 k / 96 cells = **6,230** | 223,000 | **36×** |

> **A generated room is one to two orders of magnitude too small to justify another actor.** The test does not merely fail; it fails by 36–86× on both shipped presets, which is why the rule is an absolute rather than a formula evaluated per room.

**A second, independent bound confirms it from the carrier side.** The Lab's interior batch capacity is **40,000 vertices / 68,000 triangles** (Lab §8.6). Every generated cell fits with 10× headroom, and the capacity itself sits at **30 % of the point where splitting would begin to pay** — so the shipped carrier could not express a profitably-splittable cell even if the generator produced one.

#### Why 2 (or 3) and not 1, and what that costs

The cost model, left alone, says **`k = 1`**: merge the shell and the props into one actor per cell and save an actor. We do not, and the reason is not visible to the cost function:

- the **shell** (floor, walls, ceiling, door reveals) is unique geometry, immutable for the life of the world, and takes `vtkMapper::SetStatic(1)`. It has no ladder below the building-shell rung and never pages.
- the **props** carry §6.2's 600 → 150 ladder, switch at 96/32 px, and page. Sharing one actor with the shell would force a **shell re-upload on every prop LOD swap** — reintroducing indoors precisely the hidden VBO rebuild that §7.1 item 3 exists to delete, at 1.91 ms.
- **glass** is a *program* change, not a geometry change, and is the second of the three programs §7.5.2 caps the frame at. It cannot share a mapper with opaque geometry.

**The price of that choice, stated rather than hidden:** at I-L portals-on, 6 of the 31 visible actors are prop batches that a `k = 1` rule would fold away — **0.27 ms of 3.64 ms, 7.4 %.** That is what pageable, LOD-switchable interior props cost, and it is the correct trade only because the alternative reintroduces a 1.91 ms per-swap rebuild. **`k = 1` remains the right rule for any interior whose props are as immutable as its shell**, and a generator that wants it need only mark the cell `PROPS_STATIC`; nothing else changes.

**This is also the reconciliation §7.5.2 needs.** Its rule reads "props never get their own actors" — that means **never per-prop actors**, not "never a prop batch". Both batches are opaque and vertex-coloured, so §7.5.2's conclusion is untouched: the frame still binds **three programs total**, not three per cell.

#### Never fewer: merging across cells, at 14:1 against

Merging `m` cells into one actor saves `(m−1) × 44.6 µs` and **destroys the cell mask for all of them**: a multi-cell actor must be drawn whenever *any* of its cells is active, so at group size `m` the traversal's output degenerates toward the frustum-only set. Priced on the binding case (§12.1, I-L):

| | saving | cost |
|---|---|---|
| merge the 16 opaque interior actors of the 10 reached cells into 1 (10 shell + 6 prop; glass cannot merge with opaque) | 15 × 44.6 µs = **0.67 ms** | the 12.75 → 3.64 ms portal win, i.e. **9.11 ms** |

**14:1 against.** §5.5's "never merge across cells" is not an aesthetic preference indoors; it is the single most profitable constraint in the indoor budget. It is also why the Lab's own placeholder consumer — *"draw the current cell plus ≤ 2 portal hops"* as one merged `interior` actor (Lab §8.6) — is a **Lab-side debug draw and not our runtime batching**: a cell-plus-two-hops actor is by construction un-maskable. The two designs are not in conflict; they batch for different purposes, and this runtime does not inherit the Lab's.

#### The exception, and why the generator already handles it

A cell whose L0 content exceeds `c_actor / c_tri` = **223,000 triangles** is over the line. Neither inventoried preset comes close; the one recipe that plausibly does is `warehouse` (B1), a single open volume of ~2,400 m² whose racking at §1.4a's fill fractions reaches ~245,000 triangles **[E]**. It is sub-batched **by the generator's own decomposition** — B1's four racking zones become four sub-cells joined by `OPEN` portals — and **never by a runtime grid laid over a room**. That distinction is load-bearing: a sub-cell is *still a cell*, so `cell_id` names it, §5.2's identity survives verbatim, and no second spatial partition is introduced to solve a problem one partition already solves (§3.5.6a).

#### The rule, generalized

> **Outdoors, choose the cell size and take the actor count that follows. Indoors, the architecture chooses the cell size and you choose the actor count: one shell batch per reached cell, one prop batch if it is a room, one glass batch if it is glazed — never fewer, because merging destroys the cull decision at 14:1, and never more, because a generated room is 36–86× under the split threshold and 3× under the carrier's own capacity. Sub-batch only a cell whose L0 content exceeds `c_actor / c_tri`, and sub-batch it by emitting sub-cells, never by imposing a second partition.**

`vis::recommend_leaf_size` covers the outdoor half. The indoor half is `vis::check_cell_batching(cell_content[], measured_coefficients)`, which runs inside `validate_manifest()` at load, is O(cells), and writes three margins into the provenance manifest (§10.5): the largest cell's L0 triangle count against 223,000, the mean cell's split margin, and the largest cell's vertex count against the 40,000-vertex carrier capacity. Those decide whether §12.1's indoor columns still hold for a world nobody has generated — and, per §7.5.3's tripwire, they are the first things to go stale the moment someone adds a texture indoors.

### 5.6 What batching costs, honestly

| | Merged actors (route C) | Cell-batched instancing |
|---|---|---|
| Draw calls at 200k plants | 2 — but **16.4 GB VRAM, impossible** | ~100 |
| CPU submit | 89 µs (2 actors) — if it fit | 4.46 ms (100 actors) |
| Cullable units | 2 | ~1,900 cells |
| Per-object LOD | impossible | free (same projection the cull computed) |
| Per-object occlusion | impossible | free |
| Per-object identity for GRL-SNAM labels | lost | preserved |
| Add or remove one plant | re-merge the whole buffer | one cell run re-emitted, microseconds |
| VRAM **[D]** | 16.4 GB | ~60 MB instanced / ~1.1 GB merged-with-LRU |

## 6. LOD integration

### 6.1 Culling and LOD are one traversal

LOD selection needs exactly the projected screen radius the culling stages already computed; computing it twice is pure waste, and every GPU-driven engine fuses them — RedLynx report the combined pass literally as *"Object culling + LOD: 0.28 ms"* [Haar & Aaltonen 2015]. So `visible_set` carries `dist2`, `screen_px` and a resolved `lod` per survivor, and the consumer never revisits the object.

**Order is fixed: cull first, then select LOD for the survivors.** Picking an LOD for something about to be discarded is wasted work.

The selector, in full:

```
d          = max(z_near, |eye - c| - r)          # bound-NEAREST distance, never centre
screen_px  = k_px * r / d
err_px(L)  = k_px * err_world(L) / d             # err_world from the ladder, metres
choose the smallest L with err_px(L) <= lod_error_px          (tau = 4 px)
hysteresis: promote at d < d_L * 0.92, demote at d > d_L * 1.08
representation change (mesh -> impostor) uses projected WIDTH < impostor_px (32 px),
  not the error metric -- a billboard has no meaningful geometric error
```

`err_world` is measured at bake time (one-sided Hausdorff against L0, in metres) and is forced **monotonic** across rungs, which guarantees exactly one transition per distance shell and stops a rung from being skipped or oscillating [Karis et al. 2021].

### 6.2 The ladder per asset class

| Class | L0 | L1 | L2 | L3 / HLOD | Switch driver |
|---|---|---|---|---|---|
| **Tree** (2,055 tri, h 23.77 m, w 10.5 m) | derivation depth *n*, **GPU sway** | depth *n−1*, ~560 tri, `radius_scale` 1.35× | 2-quad cross impostor, 4 tri, octahedral atlas | folded into the sector canopy shell, then into terrain albedo > 1.5 km | 140 m / 350 m (width < 32 px) / 900 m |
| **Shrub, grass tuft** (60–300 tri) | full | 1 card | — | folded into terrain albedo > 120 m | 40 m / 120 m |
| **Rock** (400 tri) | 400 | 120 (`vtkQuadricClustering`) | 30 | merged into sector proxy | 64 / 24 / 6 px |
| **Building shell** (12k tri) | full | 4:1 quadric decimation | silhouette shell, ~600 tri (a *rendered* rung, not an occluder — §3.6) | merged proxy > 1.2 km | 192 / 64 / 16 px |
| **Interior prop** (600 tri) | full | 150 | — | **none — cell-masked instead** | 96 / 32 px |
| **Terrain chunk** | 128² quads | mip-morph 64² | 32² | 16² + baked canopy albedo | 256 / 96 / 32 px |
| **Agent** (800 tri) | full | 200 | capsule 40 | **never** (dynamic, and identity matters) | 96 / 32 px |

Measured crossovers that anchor the tree row **[M]** (k = 1042, mean tree h = 23.77 m, w = 10.5 m, 2,055 tris, 30% silhouette fill):

| distance | visible area | **triangles per visible pixel** |
|---|---|---|
| 100 m | 8,130 px² | 0.25 |
| **199 m** | 2,053 px² | **1.00 ← full detail stops paying** |
| 342 m | 695 px² | 2.96 (tree width = 32 px) |
| 774 m | 136 px² | 15.1 |
| 1000 m | 81 px² | 25.3 |

We set the L0/L1 boundary at **140 m rather than 199 m**, i.e. at ~2 tris/px rather than 1, because band A is the only band paying GPU sway and the boundary term dominates its cell count (§5.3).

### 6.3 L-system iteration depth as the generation-time LOD knob

This is the strongest asset-pipeline decision in the design and it is specific to this world.

> **LOD_k of a depth-*n* plant is the depth-(*n−k*) plant the generator already knows how to make.**

Not an approximation of a mesh — a coarser member of the same generative family. Measured reduction per iteration, from the demo's own maturity classes **[M]**: m1 = 80, m2 = 720, m3 = 2,640, m4 = 7,600 triangles, i.e. **~2.9–3.7× per derivation step**.

| | Re-derive at depth *n−1* | Quadric decimation of the depth-*n* mesh |
|---|---|---|
| Silhouette | **exact** — the branching structure *is* the silhouette | thins branches; shreds twigs |
| Topology | **valid by construction** | `vtkQuadricClustering` "can drastically affect topology" |
| Determinism | **bit-identical from (seed, depth)** on every machine, forever | depends on decimator version and float ordering |
| Bake cost | ~0 — the generator is already running | 8–40 ms/plant × 200,000 |
| Disk | **none** — LOD_k is a pure function of (species, seed, maturity, depth) | 200k × 4 meshes |
| Wind skeleton | **falls out** — the L-system *is* the branch hierarchy | must be re-derived or lost |
| Attribute continuity | branch pivots, UVs, species tint all re-derived consistently | interpolated; `vtkIdType` arrays discarded on collapse |

**Area preservation is analytic here.** Dropping branch order *j* removes silhouette area, which is Epic's "Preserve Area" problem in exactly the form they hit it on Fortnite's trees. Their fix was a mesh-space heuristic (dilate open boundary edges); ours is arithmetic on the grammar's own parameters — scale the retained order's radius by `sqrt(N_n / N_{n-1})` ≈ **1.35× at L1**, which conserves projected area by construction. It is one multiply, stored in the ladder as `radius_scale`.

**Consequence for the decimator:** `vtkQuadricClustering` / `vtkQuadricDecimation` are still wanted, but only for **rocks, buildings and terrain proxies**, where there is no generative ladder and the geometry is planar/closed/manifold — exactly where QEM works well and exactly where L-systems do not apply. That work is offline, so it lives in a separate `vis_bake` executable with its own `find_package(VTK COMPONENTS FiltersCore)`. **`FiltersCore` is therefore never added to `CVCGL_VTK_COMPONENTS`** — which both avoids the hot line 24 (PR #223) and is independently the better architecture: an offline baker has no business in the runtime library's VTK closure, which every downstream consumer inherits through `find_dependency(VTK)` in the generated `cvcGLConfig.cmake`.

### 6.4 HLOD — why the vista is cheap by construction, not by culling

Per 512 m sector (quadtree L3), baked once at load:

- a **canopy shell**: the union of every plant's L2 impostor card in the tile, merged into one `vtkPolyData` and decimated to ~20k triangles, with vertex colours sampled from the species palette;
- the tile's terrain at L3 with the canopy colour **baked into its vertex colours**, so beyond ~1.5 km the shell is dropped entirely and vegetation is albedo plus a normal perturbation on the terrain material — **zero actors, zero triangles**.

Bake cost **[E]**: 9 tiles × ~36,000 plants × ~1.2 µs ≈ **390 ms at load**, or 1 tile per frame on a worker. Memory **[D]**: 9 × 20k tri × 46.8 B/vert ≈ **8.4 MB**.

Result at vista **[D]**: **9 HLOD actors + 20 terrain chunks + 6 misc = 35 actors, 0.26 M triangles → 2.81 ms CPU (356 fps)**. There is nothing left for a culler to remove, which is precisely the point and precisely why the vista regime's correct culling configuration is "almost none".

Structural lesson copied intact from World Partition: **the far field gets its own coarser grid.** Sector proxies stream and cull on a 512 m grid, not the 32 m near-field grid.

### 6.5 Popping mitigation

Dithered LOD transition — Unreal's answer, chosen there because it has *"essentially no overdraw, no depth sorting cost, no extra lighting cost, and no negative effect on occlusion culling"* — **is not available to us.** It requires TAA/TSR to resolve the noise pattern over several frames, and VTK 9.5's OpenGL2 backend gives us `vtkOpenGLFXAAPass` only, which is spatial and will render a dither as visible noise. Five mechanisms instead:

1. **Hysteresis, ±8%** on every band edge (`lod_hysteresis`). Promote at 0.92·d_L, demote at 1.08·d_L. This alone kills the dominant popping source — a stationary camera oscillating one cell forever at a boundary.
2. **Cell-coherent switching.** The LOD is chosen from the *cell* distance, so a whole cell transitions at once. A spatially coherent pop reads as a distance change; a speckled per-plant pop reads as a bug.
3. **4–6 frame alpha cross-fade** on the cells actually in transition, via `vtkProperty::SetOpacity` on the two batch actors. Bounded by the transition *rate*, not the cell count — typically ≤2 cells, ≈ **0.09 ms [D]**.
4. **Never promote from nothing.** A cell entering the frustum, or newly disoccluded, enters at its *parent's* LOD for 4 frames and then descends. So a fast camera turn shows a coarse version, never a hole, and an LOD pop can never compound with a disocclusion pop.
5. **Sway ramps in** over the outer 15% of band A, so a tree does not start twitching the instant it crosses 140 m.

`deterministic = true` **disables fades entirely** (instant switch), so headless and batch renders are frame-exact. Fading is an interactive-only nicety and must never be on the dataset path.

---

## 7. Bounding the CPU work

This is the section the design lives or dies on, because the measured frame is 98% CPU. The rule is: **name the CPU work each mechanism deletes, or do not claim the mechanism.**

### 7.1 What is eliminated, item by item

| # | Measured CPU work | ms **[M]** | Mechanism | After |
|---|---|---|---|---|
| 1 | `reposeTree` — per-plant Mat4 over every vertex | 0.28 | **GPU vertex-shader sway** (§7.2) | **0** |
| 2 | `updateVertices` — by-value `std::vector<double>` copy + scalar `SetPoint` loop with virtual dispatch | 0.48 | GPU sway: the polydata is never modified | **0** |
| 3 | **hidden VBO/IBO rebuild inside `Render()`** (triggered by `polyData->Modified()`) | **1.91** | GPU sway | **0** |
| 4 | **hidden O(N) bounds rescan** (`pts->Modified()` invalidates `GetBounds()`; two callers per frame) | **0.67** | GPU sway + `SetStatic(1)` + generator-supplied static bounds | **0** |
| 5 | `ResetCameraClippingRange()` → `ComputeVisiblePropBounds` → `GetBounds()` on every prop | 0.89 | `setClippingRangeFromBounds(vs.world_bounds())` — the culler already has it | **0.02** |
| 6 | default `vtkFrustumCoverageCuller`: `prop->GetBounds()` per prop per frame → O(N) point scan on animated meshes, **culling nothing** on a 2-prop scene | ~0.6 (inside `cpu_submit`) | `GetCullers()->RemoveAllItems()`; `VisCuller` reads a bitset and never calls `GetBounds()` | **0.005** |
| 7 | per-actor VTK submit | 44.6 µs × **all** actors | `visible_set` gates `SetVisibility(0/1)` at **224 ns/call**, which fully recovers the 44.6 µs | 44.6 µs × **visible** |
| 8 | `vtkOpenGLPolyDataMapper::RenderPiece` → `GetInputAlgorithm()->Update()` per actor per frame | in the floor | `vtkMapper::SetStatic(1)` on every batch mapper; batch polydata is immutable by construction | folded |
| 9 | `Shape::applyState` on every visible shape every frame: `SetUserMatrix` with a **freshly allocated `vtkMatrix4x4` every traversal** → new pointer → `Modified()` → `vtkProp3D::ComputeMatrix()` → prop MTime → **`vtkShadowMapBakerPass` full re-bake every frame** | 2.57 per bake, currently masked by `StridedShadowBaker` | `CullAction` caches the composed matrix per (shape, view) and `memcmp`s 128 bytes (~10 ns) before calling `SetUserMatrix`. Kills the MTime cascade at its source. | re-bake only on real motion |
| 10 | shadow bake over all props | 2.57 (strided /3) | light frustum fitted to *visible receiver* bounds; vegetation shadow-distance-culled at 400 m; grass excluded entirely | ~0.90 |

Items 1–4 together are **3.34 ms/frame at 32 trees, and they are the term that scales**: 512 trees measured 51.2 ms of animation alone, and 200,000 plants would be 41 s. Deleting them is worth more than every other line in the table combined, and it is ~200 lines of shader replacement.

### 7.2 GPU sway — the single largest win, and the mechanism exists today

Verified present in the tree: `cvc::gl::GeometryNode::addVertexShaderReplacement` / `disableCoordinateShiftScale` (`inc/cvc/gl/GeometryNode.h`), implemented via `vtkShaderProperty` (`src/cvcGL/GeometryNode.cpp`). One passthrough must be **added**: `vtkOpenGLPolyDataMapper::MapDataArrayToVertexAttribute`, to feed the per-vertex sway attributes.

Bake once, at cell build time, three per-vertex arrays into the batch polydata: `swayPivot` (vec3, the branch base), `swayPhase` (float, from the plant's seed), `swayStiff` (float, `(1−t)²` along the branch). Then:

```glsl
// injected at //VTK::PositionVC::Impl
uniform float u_time;   uniform vec3 u_wind;      // 2 uniforms; no per-frame CPU work
vec3  arm  = vertexMC.xyz - swayPivot;
float amp  = swayStiff * (0.6 + 0.4 * sin(u_time * 1.7 + swayPhase * 6.2831));
vec3  disp = u_wind * amp
           + cross(vec3(0, 0, 1), u_wind) * amp * 0.35
             * sin(u_time * 2.9 + swayPhase * 6.2831);
vec3  pos  = swayPivot + normalize(arm + disp) * length(arm);   // length-preserving
```

Per-frame CPU cost: **two `SetUniform` calls per swaying batch actor.**

Cull bounds are inflated **once, at bake**, by the maximum sway amplitude (`max(swayStiff) · |u_wind|_max`), so the AABB stays conservative without ever being re-measured. That also sidesteps a known bug: `GeometryNode::getBoundingBox()` returns `m_geometry->extents()` — the *bind-pose* box — which `updateVertices()` never touches, so it goes stale under animation. Culling must never call it; see §8.1 item 6.

This is the design's structural analogue of Fortnite's baked wind (a single position + quaternion lookup, no dependent texture reads) and of its **WPO Disable Distance** (sway evaluates in band A only, and is disabled in the shadow pass, relative to the main camera).

Works identically in wasm: GLES3 has vertex attributes and uniforms, and cvcGL already carries an `#ifdef __EMSCRIPTEN__` shader path for the GLES3 mapper — the replacement string must simply be authored twice.

### 7.3 Before / after cost model

**Before** (measured, 32 trees, 1280×800, vsync off) **[M]**: 22.8 ms → 43.8 fps, of which 13.50 `cpu_submit`, 6.30 `seaField`, 3.34 true animation, 0.89 clipping range, 0.42 GPU.

**Before, extrapolated to the target scene [D]**: **does not run.** 16.4 GB VRAM on a 4 GB card; 41 s/frame of animation. (Measured scaling corroborates the trend: 512 trees = 10.0 fps, 106 ms per animated frame, 101 ns/vertex.)

**After**, target scene = 200,000 plants, ground regime **[D]**:

| term | count | rate | ms |
|---|---|---|---|
| VTK fixed floor | — | — | 1.20 |
| agent grid rebuild (counting sort, 4,000 agents) | — | — | 0.04 |
| **agent cell locate** (coherent per-agent portal-cell id, 4,000 agents) **[D]** | 4,000 | ~6 ns/agent | **0.02** |
| `cell_mask` stage (~0 outdoors, where the column is null) | — | 1.5 ns | 0.00 |
| seed + filter stages (SIMD, 2 views via `cull_multi`) | ~1,900 nodes → 100 batches | §3.6 | 0.31 |
| `terrain_horizon` (8 corners × 100 batches) | — | 1.4 µs/candidate | 0.14 |
| `horizon_coverage` regime probe (32 rays, always on) | — | — | 0.01 |
| visible actors | 100 | 44.6 µs | **4.46** |
| visible triangles | 16.5 M | 0.2 ns | **3.30** |
| **CPU-animated vertices** | **0** | 100 ns | **0.00** |
| shadow pass (fitted, stride 3, vegetation distance-culled) | — | — | 0.90 |
| clipping range from `visible_set::world_bounds()` | — | — | 0.02 |
| sea surface + skybox (replacing 7.74 ms of ray-casting) | — | — | 0.40 |
| **CPU total** | | | **8.96 ms → 112 fps** |
| GPU (0.5 + 0.15 ns × 16.5 M) | | | 2.98 ms |

Same world, other regimes **[D]**:

| regime | actors | M tris | CPU ms | fps |
|---|---|---|---|---|
| vista | 35 | 0.26 | **2.81** | 356 |
| ground | 100 | 16.5 | **8.96** | 112 |
| ridge **[E]** | ~55 | ~6 | ~5.0 | ~200 |
| **seam** (I-L entry doorway; §10.4a) | 41 | 0.27 | **4.23** | 236 |
| indoor **I-S** (portals on) | 16 | 0.34 | **2.74** | 365 |
| indoor **I-L** (portals on) | 31 | 0.38 | **3.64** | 275 |
| indoor **I-L** (portals off) | 230 | 0.86 | **12.75** | 78 |

**The flat frame time from 32 trees to 200,000 plants — and from vista to ground — is the property the whole architecture exists to deliver.** And note what produced it: GPU sway removed 3.34 ms and made 41 s/frame impossible-become-zero; HLOD made the vista 35 actors; batching+frustum made the ground 100 actors. **Occlusion contributed 0.0 ms at vista, ~0.3 ms at ground, and everything indoors.** That ordering is the honest attribution and §11.3 makes it a mandatory benchmark column so no PR can claim one mechanism's win for another's.

### 7.4 VRAM

| resource, 200,000 plants | instanced **[D]** | merged-with-LRU fallback **[D]** |
|---|---|---|
| unique meshes (8 species × 4 rungs) | 3.9 MB | 3.9 MB |
| instance transforms (200k × 48 B) | 9.6 MB | n/a |
| resident merged cells | n/a | ~1,020 MB (L0 44-cell + L1 32-cell LRU) |
| impostor atlas (8 species × 8 views × 256² RGBA) | 25 MB | 25 MB |
| HLOD canopy shells (9 tiles × 20k tri) | 8.4 MB | 8.4 MB |
| terrain + min/max-mip pyramids | 23 + 11.2 MB | 34.2 MB |
| shadow atlas | 24 MB | 24 MB |
| **total** | **~105 MB** | **~1.11 GB** |
| against VTK baseline 276 MB, card 4 GB **[M]** | comfortable | fits, with headroom |
| against the naive 16.4 GB | **156×** | 15× |

Both fit. That is why the instancing spike (**D2**, PR 0) is a *quality* decision rather than a *feasibility* one, and why the plan does not stall on it.

---

### 7.5 Indoor cost structure — draw calls, state changes, and whether 44.6 µs still applies

§7.1–§7.4 bound the *outdoor* CPU work. The indoor regime has a different cost shape and until
this revision the design simply reused the outdoor coefficient without saying so. The reuse is
defensible, but only under a constraint that has to be written down, because it is a design
requirement and not a fact about the world.

#### 7.5.1 The problem with reusing 44.6 µs/actor

The **44.6 µs/actor** coefficient is **[M]** from `actorbench`: 65,536 triangles split across
*N* actors, dead linear from 128 to 2,048 actors — and **every one of those actors was
identical-material**. Same `vtkProperty`, same mapper configuration, therefore the same shader
program, therefore VTK's `vtkOpenGLPolyDataMapper::UpdateShaders` took its cached-program fast
path on all *N*. What the number measures is per-actor *traversal and submit*: matrix compose,
`GetBounds`, culler dispatch, mapper `Update()`, the VAO bind and the draw. It does **not**
contain a program rebind, a uniform-block rewrite for a new material, or a texture bind.

Interiors are exactly where that assumption is most likely to break, because a room is a
multi-material object in a way a vegetation cell is not: a floor class, a wall class, glass, and
whatever the props are made of. If each of those forced its own program, the indoor coefficient
would be `44.6 µs + n_bind × c_bind` and the whole indoor budget would move.

#### 7.5.2 Why it does not break — and the constraint that keeps it that way

Two properties of the shipped stack collapse the material axis before it reaches the driver:

1. **Indoor albedo is vertex colour, not material state.** The Lab's material registry (§16.2)
   gives every one of the 11 indoor classes an RGB albedo and nothing else — no roughness map, no
   texture, no per-class shader. The Lab's own batching carrier, `fixed_mesh` (Lab §8.6), packs
   `xyz_` **and** `rgb_` into one `GeometryNode` and calls `updateVertices` + `updateColors`. So a
   floor and the wall above it differ by *vertex data*, not by GL state. They belong in the same
   actor and the same draw call.
2. **The portal traversal already emits a cell as a contiguous proxy run** (§3.5.2,
   `emit contents(cell)`), which is the same batch identity the outdoor design uses
   (§5.2: leaf ≡ batch ≡ LOD unit ≡ cull unit).

That gives the **v1 indoor batching rule**, which is a *constraint on the generator and the
adapter*, not an observation:

>   **One batch actor per (reached cell, blend class), and no finer.** The shell batch carries the
> cell's floor, walls, ceiling and door reveals; the prop batch carries its static props; a third,
> transparent batch appears only in cells that carry glazing. **All albedo is vertex colour. No
> textures indoors in v1.** The shell/prop split is forced by paging and LOD, not by material
> (§5.5a); the shell/glass split is forced by blend state. Nothing else may split a cell.

Under that rule the number of distinct GL programs in an interior frame is **three, total, for
the whole frame** — opaque-lit, transparent-lit, and the unlit HUD — regardless of room count,
material count, or building size. It is not three per cell; it is three per frame. Shell and prop
batches share the opaque program, so the §5.5a split costs actors but **zero** program switches.

#### 7.5.3 The residual, costed **[E]**

Program rebinds still happen at the boundaries between those three classes. A VTK program switch
on this driver is a `glUseProgram` plus a full uniform-block upload, ≈ **3 µs [E]** (bracketed
below by the measured 224 ns `SetVisibility` call and above by the 44.6 µs whole-actor cost; it
wants measuring in `vis_bench --calibrate`, and §11.3 gains a column for it). So:

```
c_actor_indoor  =  44.6 us  +  (n_program_switches / n_actors) x c_bind
```

**If actors are submitted sorted by material class**, `n_program_switches` is 2 per frame — one
transition into transparent, one into HUD — for *any* interior:

| Preset | actors | switches | adder | effective µs/actor | error vs 44.6 |
|---|---|---|---|---|---|
| I-S, portals on | 16 | 1 (no glazing) | 3 µs / 16 | 44.79 | **+0.4 %** |
| I-L, portals on | 31 | 2 | 6 µs / 31 | 44.79 | **+0.4 %** |
| I-L, portals off | 230 | 2 | 6 µs / 230 | 44.63 | **+0.06 %** |

**If actors are submitted unsorted**, the worst case is a switch on every transition between an
opaque batch and a glass batch — up to 2·min(opaque, glass) switches. At I-L portals-on that is 12
switches, 36 µs, **+1.16 µs/actor (+2.6 %)**. Still inside the noise of the coefficient itself, but
free to avoid.

**Verdict: the 44.6 µs coefficient is reused unchanged for the indoor budgets in §12.1, and the
reuse is valid by construction rather than by luck** — because §7.5.2's batching rule holds the
material count at three programs per frame. The residual is ≤ 0.4 % and is inside the ±30 %
hardware variation the cell-size derivation was already shown to tolerate (§5.3). **If the v1 rule
is ever relaxed — per-room texture atlases, a PBR indoor material, per-prop `vtkProperty` — the
coefficient is no longer transferable and §12.1's indoor columns must be re-measured, not
re-derived.** That is the tripwire; it is stated here so nobody trips it silently.

#### 7.5.4 Sort order: material class, then nothing

`visible_set` gains one sort key ahead of the existing canonical ordering:

```
sort key = (material_class, cell_id, proxy_run_offset)
             ^ 0=opaque 1=transparent 2=hud     ^ already canonical (Morton / BFS order)
```

Three notes on what this deliberately does **not** do:

- **No front-to-back depth sort.** Its only product is early-Z rejection, which is a *GPU* saving,
  and the measured frame is **1.8 % GPU** (§1.1). At I-L portals-on the entire GPU cost is
  0.51 ms; a perfect early-Z would recover a fraction of a fraction of that, against a per-frame
  radix sort over the visible set on the term that is 98 % of the frame. **Not implemented.** What
  we get for free is that portal traversal is a BFS outward from the camera's cell, so cells
  already arrive in roughly near-to-far order; we take that and claim nothing for it.
- **No per-frame re-batching.** The sort reorders *submission*, never geometry. `fixed_mesh::commit()`
  is ~1.1 ms for a 130 k-vertex chunk (Lab §8.6) and must stay off the per-frame path indoors
  exactly as it is outdoors. Cell contents are immutable; only visibility flips, at 224 ns/call.
- **Determinism is preserved.** The key is a total order over integers with no float comparison
  and no pointer identity, so it satisfies §11.5 rule 1 unchanged.

#### 7.5.5 Draw calls, stated

| | I-S on | I-S off | I-L on | I-L off | Ground (outdoor, for scale) |
|---|---|---|---|---|---|
| Draw calls (== visible actors) | 16 | 59 | 31 | 230 | 100 |
| Distinct GL programs bound | 2 | 2 | 3 | 3 | 4 (adds sway) |
| Program switches / frame (sorted) | 1 | 1 | 2 | 2 | 3 |
| Texture binds | **0** | 0 | 0 | 0 | 2 (impostor atlas, terrain) |
| Uniform uploads / frame | 16 | 59 | 31 | 230 | 100 (one MVP block per actor) |
| **Interior triangles per interior draw call** | 1.4 k | 0.8 k | 2.5 k | 3.1 k | 165 k |

The last row is the one to read against §1.4's inversion. An interior draw call carries **50–100×
fewer triangles** than an outdoor one and costs exactly the same 44.6 µs. **Indoors, the per-actor
cost is even more completely the whole story than it is outdoors** — the interior triangle term is
0.011 ms, **0.3 % of the I-L portals-on frame**, and 85 % of even that frame's triangles arrive
through the doorway on 3 actors (§1.4a). Every indoor optimisation that is not "reduce the number of cells whose
contents get submitted" is noise, and portal traversal is precisely and only that optimisation.

---

## 8. What must be added to cvcGL / what bypasses VTK

**VTK is not forked, not bypassed, and not replaced.** It remains the rasterizer, the shadow-map baker, the volume ray-caster, the depth/state manager and the window/interactor owner. We add four small classes and four passthroughs, and we *remove* one default behaviour. Everything below either exists in the installed VTK 9.5.0 prefix or is explicitly marked **must be added**.

### 8.1 New

| # | Addition | Kind | ~LoC | Uses (all verified present) |
|---|---|---|---|---|
| 1 | `cvc::gl::makeViewParams(vtkRenderer*, frame)` → `cvc::vis::view_params` | free function | 80 | `vtkCamera::GetFrustumPlanes(aspect, double[24])`, `vtkRenderer::GetTiledAspectRatio()`, `vtkCamera::GetViewAngle/GetPosition`, `vtkRenderWindow::GetSize`. **cvcGL exposes no frustum accessor today — `CameraController` gives only `getPose(eye, focal, up)`.** |
| 2 | `cvc::gl::VisCuller : public vtkCuller` | new class | 120 | `vtkCuller` is `VTKRENDERINGCORE_EXPORT` with a public pure-virtual `Cull()`; `vtkStandardNewMacro`; `vtkRenderer::AddCuller/GetCullers`. In-VTK precedent: `vtkWebGPUComputeOcclusionCuller` documents exactly this recipe. |
| 3 | `cvc::gl::CullAction : public cvc::gl::Action` | new class | 300 | `Action::Kind::Custom` **already exists** (`inc/cvc/gl/traversal.h`); `BoundingBoxAction` is the template; `VisibilityElement{uint32 mask}` maps 1:1 onto `scene_view::layer_mask` |
| 4 | `cvc::gl::BatchedScene` | new class | 700 | `vtkPolyData`, `vtkActor`, `vtkPolyDataMapper::SetStatic`, `vtkProp::SetVisibility`, `vtkProperty::SetOpacity` |
| 5 | `cvc::gl::SwayShader` | new class | 200 | `GeometryNode::addVertexShaderReplacement` (exists) + passthrough #8 below |
| 6 | `GeometryNode::setStaticMesh(bool)` → `mapper->SetStatic(1)` + cached bounds | passthrough | <10 | `vtkMapper::SetStatic` |
| 7 | `GeometryNode::setWorldBounds(const double[6])` — generator-supplied, sway-inflated | passthrough | <10 | works around the stale bind-pose `getBoundingBox()` |
| 8 | `GeometryNode::mapDataArrayToVertexAttribute(...)` — **must be added** | passthrough | <10 | `vtkOpenGLPolyDataMapper::MapDataArrayToVertexAttribute` |
| 9 | `GeometryNode::updateVertices(std::vector<double>&&)` and a `std::span<const double>` overload — **must be added** | passthrough | 20 | kills the by-value `std::vector` copy captured into the lambda (1.5 MB/mesh/frame in the demo). Still needed for agents and deforming props. |
| 10 | `GraphicsNode::setCullVisible(bool)` → `prop->SetVisibility(0/1)` **and nothing else** — **must be added** | passthrough | <10 | see §8.2 |
| 11 | `SceneRenderer::setClippingRangeFromBounds(const double[6])` — **must be added** | passthrough | 25 | replaces `ResetCameraClippingRange()` |
| 12 | **Fix** `TraversalState::multiplyTransform` to mutate a reused `vtkMatrix4x4` rather than allocating a fresh one per traversal | bug fix | 15 | prerequisite, not optional — see §7.1 item 9 |

`VisCuller::Cull` is deliberately trivial, because **all the real work happens before the frame**:

```cpp
double VisCuller::Cull(vtkRenderer*, vtkProp** props, int& n, int& initialized) {
  int keep = 0; double total = 0.0;
  for (int i = 0; i < n; ++i) {
    auto it = m_propToProxy.find(props[i]);          // flat hash, built at attach time
    const bool vis = (it == m_propToProxy.end()) || m_bits.test(it->second);
    double t = vis ? 1.0 : 0.0;
    if (initialized) t *= props[i]->GetRenderTimeMultiplier();
    props[i]->SetRenderTimeMultiplier(t);
    if (t > 0.0) { props[keep++] = props[i]; total += t; }   // compact, then shrink
  }
  n = keep; initialized = 1;
  return total > 0.0 ? total : 1.0;
}
```

**No `GetBounds()` call anywhere.** That is the entire point, and it is why removing VTK's default culler first is mandatory rather than cosmetic.

### 8.2 Removed, and not used

| Thing | Why |
|---|---|
| **`vtkFrustumCoverageCuller`** (installed by `vtkRenderer`'s own constructor) | `Cull()` calls `prop->GetBounds()` **per prop per frame** → `vtkPolyDataMapper::GetBounds` → `UpdateInformation()` + `UpdatePiece()` → `ComputeBounds()` = an **O(N) point scan whenever the point array MTime advanced**. On the current demo that is two full-mesh scans per frame while culling nothing (2 props, both on screen). Also `MinimumCoverage` defaults to 0.0 so it does no coverage culling at all, and `SortingStyle != None` is an insertion sort the header itself calls "a simple bubble sort". **`GetCullers()->RemoveAllItems()` is step zero** — stacking on top of it silently re-imports the whole tax. |
| **`SceneNode::setVisible()` as a per-frame primitive** | It calls `AddViewProp`/`RemoveViewProp`, which (1) `ReleaseGraphicsResources` — **destroying the actor's VBO/IBO/VAO/shaders**, so re-showing pays a full re-upload and shader relink; (2) walks an O(n) `vtkCollection` twice (`HasViewProp` then `IndexOfFirstOccurence`); (3) bumps `GetViewProps()->GetMTime()`, which **forces a full shadow re-bake**; (4) recurses the entire subtree; (5) round-trips through a `cvc::state` `"visible"` key. **Never on the frame path.** `setCullVisible` (addition #10) calls `prop->SetVisibility` and stops. |
| **`vtkLODActor` / `vtkQuadricLODActor` / the `RenderingLOD` module** | LOD is selected by **measured wall clock** against `AllocatedRenderTime`, which comes from `SetDesiredUpdateRate` and defaults to 0.0001 → **10,000 s per renderer**, so in cvcGL's interactor-free loop the automatic LOD *never drops below full resolution*. Even configured, timing-driven selection is **non-deterministic** — fatal for headless GRL-SNAM generation. `RenderingLOD` is not linked. |
| **`vtkLODProp3D`** | Genuinely deterministic with `AutomaticLODSelectionOff()` + `SetSelectedLODID`, and already in a linked module — a real option, and the right one if we ever need per-LOD *properties*. Rejected narrowly here because it introduces a prop type `GeometryNode` (actor + mapper) does not model, complicating the state, shadow and texture paths, for no benefit over toggling `SetVisibility` on N sibling batch actors. |
| **`vtkHardwareSelector` on the frame path** | `CaptureBuffers()` loops 2 iterations × ~5 passes, each a **full `rwin->Render()` plus a framebuffer readback** — up to ~10 full scene renders per query. It also forces a black background, disables swap, and **disables glyph instancing**. Retained **offline only**, as the id-buffer ground-truth oracle (§11.2) and for impostor atlas baking, where it is genuinely the right tool. |
| **`vtkWebGPUComputeOcclusionCuller`** | The WebGPU module is **not built in this prefix** (0 headers, no `libvtkRenderingWebGPU*.so`); its own header warns it breaks under an OpenGL backend (Y-flip); and Kitware measure it at **2× slower at 10 M triangles**. Not an option, and at our triangle counts not desirable either. |
| **`FiltersCore` in `CVCGL_VTK_COMPONENTS`** | Decimation is offline work; it lives in the `vis_bake` example with its own `find_package`. Keeps us off the hot line 24 *and* keeps the runtime library's VTK closure minimal for every downstream consumer (§6.3). |

### 8.3 The shadow-correctness trap, and the fix

`vtkShadowMapBakerPass::Render` builds its **own** prop array by walking `r->GetViewProps()` and filtering on `GetVisibility()`, **explicitly ignoring the camera-culled list** — the source comment is *"We need all the visible props, including the one culled out by the camera, because they can cast shadows too."*

Two consequences, both first-class design constraints:

1. **A `vtkCuller` saves exactly zero in the shadow pass.** Only `SetVisibility(0)` reduces the bake.
2. **`SetVisibility(0)` driven by *camera* frustum culling would silently drop off-screen shadow casters**, and shadows would pop at the frustum edge.

**The fix is `cull_multi`.** The shadow view is a second frustum — the directional light's ortho, fitted to the bounds of the *visible receivers* rather than to the whole scene (VSM's page-marking idea stripped of virtualization; free, and it raises effective shadow resolution at the same time). A prop is `SetVisibility(0)` **only when its `view_mask` is zero across every view.** `VisCuller` then truncates the camera list independently. This is why `cull_multi` is on the base `culler` interface rather than bolted on later.

Additionally, and following Fortnite directly: **vegetation is shadow-*distance*-culled at 400 m** (view-direction-independent, therefore artefact-free), terrain and buildings never are, and **grass and small foliage are excluded from the shadow pass entirely**. cvcGL's existing `StridedShadowBaker` stays; note that its safety guard counts shadow-casting *lights*, not props, so prop visibility churn does not defeat the stride — which is what makes `SetVisibility` toggling affordable at all, and which is why §11.1 mandates a shadow-correctness regression test.

### 8.4 What genuinely bypasses VTK

Exactly one thing: **the vertex shader for swaying batches**, via `addVertexShaderReplacement`, which is a sanctioned VTK extension point (`vtkShaderProperty`), not a bypass. No custom `vtkOpenGLRenderPass`, no mapper replacement, no GPU-driven pipeline, no second shadow system, no material system of our own, no picking replacement. `vtkPropPicker`, `vtkProperty`, the render-pass pipeline and the volume path all continue to work unchanged on every prop in the scene.

That is a deliberate scope decision and it is defended by measurement: a GPU-driven pipeline would remove **GPU** work from a GPU measured at 1.8% utilization, would be structurally impossible in WebGL2, and — per Kitware's own numbers on VTK's own implementation — would be a *loss* below ~100 M triangles against our 16.5 M. See decision **D9**.

---

## 9. Native vs WebGL2

**Native-first is the decision; wasm follows and is explicitly reduced-scale, not parity-gated.** The good news is that the entire visibility and LOD design is CPU-side by construction, so `cvc::vis` compiles to wasm unchanged.

### 9.1 The honest capability matrix

Verified against `emsdk-cvc/upstream/emscripten/system/include/GLES3/gl3.h` and VTK's own GLES gates, not assumed.

| Capability | Native | wasm / WebGL2 | Evidence |
|---|---|---|---|
| All of `cvc::vis` (indices, stages, portals, terrain march, LOD, governor) | yes | **yes, identical code** | pure C++, no GL, no VTK, no intrinsics requirement |
| SIMD frustum kernel | AVX2 / SSE4.1 | `-msimd128`, **128-bit only** | Emscripten maps SSE intrinsics onto wasm SIMD; some SSE4.1 intrinsics are *emulated* and slow |
| GPU vertex-shader sway | yes | **yes** | GLES3 has attributes + uniforms; cvcGL already has a GLES3 shader path |
| `drawElementsInstanced` | yes | **yes, core** | VTK hard-defines `GLAD_GL_ARB_instanced_arrays 1` under `GL_ES_VERSION_3_0` in `vtk_glad.h.in` |
| **VTK's GPU instance cull + LOD** (`SetCullingAndLODOn`) | yes, needs `ARB_gpu_shader5` + `ARB_transform_feedback3` = **GL 4.0, not 4.3** | **NO — compile-time disabled.** `vtkOpenGLGlyph3DHelper.cxx` forces `culling = false` under `#else // disable culling on OpenGL ES`; `GetMaxNumberOfLOD()` returns **0** | the single most important correction in this section |
| Compute shaders / SSBO / indirect draw / image atomics | 4.3+ | **no** (WebGL2 = GLES 3.0; the `gl31.h`/`gl32.h` headers emscripten ships are not implemented by the WebGL2 backend) | GPU-driven culling is structurally impossible |
| Occlusion queries | yes | yes, **but ≥1 frame stale by specification** | WebGL 2.0 spec: *"A query's result must not be made available until control has returned to the user agent's main loop"* — added deliberately *"to prevent applications from relying on being able to issue a query and fetch its result in the same frame"* |
| `WEBGL_multi_draw` | n/a | extension, widely shipped | collapses N JS draw calls into one; counts come from **CPU arrays**, so it is not GPU-driven culling. Probe at runtime; not required. |
| Geometry shaders, buffer textures (`samplerBuffer`), `gl_DrawID` | yes / yes / yes | **no / no / no** | per-instance data must ride in instanced vertex attributes or a 2-D texture — design for that from the start |
| Depth readback for a CPU coverage buffer | possible | **never attempt** — `readPixels` on a depth attachment is a full pipeline flush | costs more than the culling saves |

### 9.2 The inversion nobody expects

Because VTK gives **zero** GPU instance culling and **zero** GPU instance LOD under GLES3, the CPU culler is **more** load-bearing in the browser than natively, not less. The usual "wasm gets the degraded path" story is backwards here: wasm is the configuration in which `cvc::vis` is the *only* culling and the *only* LOD selection in the system.

The second inversion: **indoors is the wasm sweet spot.** Portal traversal is pure CPU integer work costing ~11 µs and removing 86.5% of visible actors (§12.1), with no GPU feature whatsoever. The strategy that is unavailable on the web (GPU occlusion) is the one the open vista did not want; the strategy the web *can* run is the one interiors need. That is the multi-strategy argument in its cleanest form.

### 9.3 What degrades

| Component | wasm behaviour |
|---|---|
| frustum / distance / small-feature | kept, 128-bit SIMD, ~2× the native per-candidate cost **[E]** — **measure, do not assume** |
| `terrain_horizon` | **kept.** ~250 µs **[E]**. Pure array math; the best-value stage in the browser |
| `portal_seed` | **kept, unchanged.** The indoor story |
| `occluder_volume` | **does not exist on either platform** — cut from v1 (§3.6), so there is nothing to degrade |
| GPU per-instance cull + LOD | **absent.** CPU supplies both, mandatory |
| Occlusion queries | **never used**, native or web (§3.2) |
| Raster occlusion, if ever built | **native only**, disabled in the browser |
| Shadows | 1 cascade, 1024², stride 6 |
| Threading | single-thread by default; parallel-for only in the pthread build. **Hard 1.5 ms wall-clock budget on the whole visibility pass** (`cull_pipeline::set_time_budget`) — a browser main thread cannot absorb a 4 ms spike |

### 9.4 The reduced-scale variant

`lsystem_lab --profile=web`, shipped through the existing `_wasm_demos` mechanism in `src/cvcGL/examples/CMakeLists.txt`:

| parameter | native | web |
|---|---|---|
| island radius | 1000 m | 400 m |
| plants | 200,000 | **25,000** |
| species | 8 | 3 |
| band radii | 140 / 350 / 900 m | 70 / 175 / 450 m |
| HLOD switch | 900 m | 380 m |
| resolution | 1280×800 | 1024×640 |
| visible actors **[E]** | 100 | 48 |
| per-actor submit **[E]** | 44.6 µs | ~90 µs (JS ↔ wasm ↔ GL) |
| visibility pass **[E]** | 0.45 ms | 0.90 ms |
| **frame [E]** | 8.96 ms | **~22 ms (45 fps)** |
| target | 60 fps | **30 fps** |
| interiors | full | **full** — the wasm sweet spot |

The web profile is a `cvc::state` subtree (`vis/profile/*`), **not an `#ifdef`**, so a native run can reproduce the web configuration exactly for debugging — which matters because the web build is the one you cannot attach a profiler to.

---

## 10. Pluggability & the conservative-correctness contract

### 10.1 How a new strategy is added

| Seam | Interface | Difficulty | Third-party examples |
|---|---|---|---|
| **Stage** (the normal case) | `cull_stage` — 1 required virtual, in → out, no state | **easy, ~120 lines** | a fog-distance stage, a per-species draw-distance stage, `pvs_lite` (a bitset lookup), an SDF occlusion *hint*, an MSOC raster stage |
| **Seed** | `cull_seed` — produces the initial candidate list | easy-medium, ~200 lines | a streaming-tile seed, a "cells visible to agent N" seed for `cvc::nav` |
| **Index** | `spatial_index` — rebuild/update + 4 queries | medium, ~400 lines | a loose octree, a hierarchical grid, a binned-SAH BVH, an R-tree for the nav side |
| **Whole culler** | `culler` | rare | a PVS reader, a middleware shim |

**The stage is the intended unit, and that is a load-bearing choice.** Conservativeness composes for free (an intersection of supersets of the visible set is a superset), so a pipeline of conservative stages is conservative with *no additional proof obligation*. That property is why OGRE's `SceneManager`-as-plugin had to be abandoned and Godot 3's rooms-and-portals culler died as a core patch, and why ours is a leaf: **a specialist culler here never reaches into the renderer.**

### 10.2 Registration

```cpp
// inc/cvc/vis/registry.h
// OGRE's SceneManagerFactory + SceneManagerMetaData idea, retargeted at the
// CULLER rather than at a scene god-object.
struct culler_traits {
  const char*   name;
  bool conservative      = true;   // false == AGGRESSIVE; opt-in only, never default
  bool deterministic     = true;   // false == runtime SIMD dispatch, async readback, ...
  bool handles_dynamic   = true;   // false == needs a bake (a PVS)
  bool needs_preprocess  = false;
  bool needs_gpu         = false;
  bool wasm_capable      = true;
  std::uint32_t good_below_objects = 0, good_above_objects = 0;
  const char*   good_regimes = "vista,ground,ridge,indoor,seam";
};

// NOT a process singleton. Hung off cvc::app's existing data map (verified:
// inc/cvc/core/app.h declares `boost::any data(const std::string&)` and
// `void data(const std::string&, const boost::any&)`), under the key
// "cvc.vis.registry" -- matching the house rule SceneRenderer.h states
// explicitly ("not a singleton and not a global"). Two apps in one process get
// two registries, which is also what keeps tests isolated.
class registry {
public:
  void add(culler_traits, std::function<std::unique_ptr<culler>()>);
  void add_stage(const char* name, std::function<std::shared_ptr<cull_stage>()>);
  std::unique_ptr<culler> create(std::string_view) const;
  std::vector<culler_traits> list() const;
};
registry& visibility_registry(cvc::app&);

struct recommendation { std::string culler; std::string reason; };
// ADVISORY ONLY. cull() NEVER calls this. The caller asks, LOGS the reason, and
// PINS the answer in the run manifest. Silently swapping strategy mid-run in a
// data-generation build is how a dataset becomes unreproducible in a way nobody
// notices for a month.
recommendation recommend(const scene_stats&, const registry&);
```

Shipped recommendations:

| `scene_stats` | recommended pipeline |
|---|---|
| `object_count < 1000` | `linear_index` seed → `layer_mask` → `frustum`. **Brute force wins; any structure is negative return.** |
| open outdoor, `dynamic_fraction < 0.05`, `count > 10k` | `index_seed(quadtree)` → distance_size → frustum → small_feature |
| terrain relief > 50 m | ... → `terrain_horizon` |
| `portal_graph` non-empty | `portal_seed` instead of `index_seed`, **plus `cell_mask` immediately after `layer_mask`** (§3.5.6a) |
| `dynamic_fraction > 0.8`, `count < 20k` | **no culling at all** — one instanced actor, upload everything |
| `deterministic == true` | as pinned in the manifest; governor frozen |

### 10.3 The contract, and its four layers of enforcement

Taxonomy from [Cohen-Or et al. 2003], cited in `cull_stage.h`: **conservative** (⊇ visible; may never classify a visible object as occluded) / **exact** / **approximate** (sampled) / **aggressive** (deliberately gives up conservativeness for tightness).

1. **Debug asserts inside `cull_pipeline::cull`, after every stage**: `out ⊆ in`, `out` sorted ascending, `|out| ≤ |in|`. This turns a whole class of bug into an immediate local failure instead of a wrong frame six stages later.
2. **Trait propagation.** `cull_pipeline::conservative()` is the AND over the seed and every stage. `small_feature_stage` is the only shipped stage returning `false`, and it appears in the trace as `[approx]`.
3. **`conservativeness_violations(candidate, oracle)` is a shipped library function**, not a test helper. It *is* the contract, executable, and `vis_oracle_test` runs it over a seeded fuzz corpus.
4. **A hard gate on the dataset path.** `cvc::vis::manifest::require_conservative(true)` makes `cull()` throw if the active pipeline reports `conservative == false` or `deterministic == false`. **A training dataset cannot be generated by an aggressive or non-deterministic culler by accident.**

And the rule that catches the subtle case: **latency breaks conservativeness, so it must be declared.** Any stage whose answer is stale — an occlusion query readback, a reprojected depth buffer, an async result — must return `conservative() == false`, or be documented as conservative-with-hysteresis and take a bounds-padding parameter. No stage may be quietly non-conservative because of timing. This is the specific reason occlusion queries are not in the shipped set.

### 10.4 The governor — advisory, hysteretic, observable, freezable

The regime asymmetry (§3.3) is real enough that the right stage set genuinely changes mid-flight, so a closed-loop controller runs over metrics the pipeline already computes:

```
Metrics, EWMA over 30 frames, all from cull_pipeline::last_trace() and scene_view:
  visible_ratio     = |visible_set| / |candidates after frustum|
  horizon_coverage  = terrain_field::horizon_coverage(view, 32 rays)   ~6 us, always on
  enclosure_n       = portal_graph::locate_multi(eye, near_r, cells[4])  # the SS3.5.6
                      # SEED RULE, already computed. 0 = exterior only, 1 = one cell,
                      # >= 2 = the camera straddles a doorway. A BOOLEAN CANNOT SAY THIS.
  aperture_fraction = sum(screen area of EXTERIOR portals reached) / frustum screen area
                      # also already computed: frustum::narrowed_by yields the screen
                      # rect per portal, so both metrics cost ZERO additional us.
  interior_reached  = any interior cell reached while enclosure_n == 0
                      # standing outdoors, looking in through a visible door
  saved_est_ms      = cull_ratio(stage) * 0.0446 ms * stage_in
  cost_ms           = measured stage time from last_trace()

Policy, per optional stage:
  if saved_est < 1.3 * cost for 30 consecutive frames -> demote (halve the budget;
      at minimum, disable for 120 frames)
  if saved_est > 3.0 * cost                            -> promote
  every 120 frames while disabled                      -> RE-PROBE one frame,
      results discarded  <-- this is what stops the system getting stuck disabled
                             after the camera walks back indoors
  dwell time >= 45 frames before any regime transition takes effect,
      EXCEPT entry into `seam`, whose dwell is 0 (SS10.4a)
  demotion state is keyed BY REGIME, not global -- walking indoors and back out
      restores the outdoor regime's own record instead of inheriting the indoor one
  ANY regime transition -> immediate re-probe of every disabled stage on the next
      frame, discarding the 120-frame timer.  The 120-frame timer is the STEADY-STATE
      re-probe; a transition is an EVENT re-probe, and SS10.4a is why both are needed.
  in `seam`: the governor is FROZEN -- no demotions, no promotions, no re-probes
  hard cap: the occlusion stage may never exceed 0.40 ms; it is truncated if it does
```

Classification, in full — the outdoor branch is unchanged and the two indoor-facing regimes are now separated:

```
if enclosure_n >= 2                                  -> seam    # camera in two cells
elif enclosure_n == 1 and aperture_fraction >= 0.05  -> seam    # inside, big window
elif enclosure_n == 1                                -> indoor
elif interior_reached                                -> seam    # outside, looking in
else                                                 -> vista | ground | ridge,
                                                        by horizon_coverage as before
```

The 1.3× / 3.0× band prevents oscillation; the re-probe prevents the single most likely failure of a naive on/off switch. **`deterministic = true` freezes the governor at its manifest-pinned configuration** — an adaptive controller is state the manifest cannot otherwise pin.

Expected behaviour, from the measured profile: **vista → occlusion OFF** (ceiling 21.5%, and small-feature already got more than that for a third of the cost); **ground → terrain march ON, modest**; **ridge → ON, strongest outdoor case**; **indoor → portal seed + cell mask, terrain march irrelevant**; **seam → the union of the indoor and outdoor stage sets, with the governor frozen (§10.4a).**

### 10.4a The seam is a regime, not a corner case

`regime::seam` was declared in `types.h` and used nowhere. §3.5.6 defines the state precisely — the camera is in two cells and sees both — and §11.1's `vis_regime_test` already asserts conservativeness at *"the 12 straddle frames"*, so the test knew about a state the classifier could not name, the policy did not cover and the budget did not price. This subsection closes all four.

**Why a boolean cannot do it.** `enclosure = portal_graph::locate(eye) != EXTERIOR` collapses three genuinely different frames onto two values:

| the camera is… | `enclosure` | what the frame actually pays for |
|---|---|---|
| deep in a room | true | interior only — the terrain march is dead weight |
| in a doorway, in **two** cells | true | **both** cell sets *and* the un-narrowed exterior seed |
| in a lobby behind a glass curtain wall | true | mostly exterior, through a 40% aperture |
| outdoors, facing an open door 5 m away | **false** | exterior *plus* the room behind the door |

Rows 2–4 are the seam. Two of them report `enclosure = true` and one reports `false`, so no threshold on that boolean separates them — the metric has to be the **count** `enclosure_n` plus the **aperture fraction**, and both are byproducts the seed already computes (§10.4). **The seam classifier costs 0 µs.**

**The doorway dwell — the failure the 120-frame re-probe does not catch.** This is the concrete bug, and it is worth walking:

```
frame    0   camera indoors.  governor: saved_est < 1.3 x cost for terrain_horizon
             for 30 consecutive frames (correct -- it culls nothing inside a building)
frame   30   terrain_horizon DEMOTED -> disabled for 120 frames
frame   40   camera walks through the door
frame  52-64 SEAM: 12-24 frames (see the arithmetic below)
frame   64   camera is outdoors, on a ridge.  terrain_horizon is STILL DISABLED
frame  150   the 120-frame re-probe finally fires
```

Between frames 64 and 150 — **86 frames, 1.4 s at 60 fps** — the strongest outdoor occlusion stage is off in the regime where §11.4 measures it as worth **+3.08 ms**. The re-probe was the mechanism meant to prevent exactly this, and it is 5–10× too slow, because a doorway is not crossed on a 120-frame timescale:

| | |
|---|---|
| door leaf depth + jamb | 0.25 m (the Lab's `wall_thickness_render_m`) |
| the seed inflates cell bounds by the near-plane radius on both sides | +2 × ~0.3 m |
| seam path length | **~0.85 m** |
| walking speed | 1.4 m/s (agent-scale) … 2.5 m/s (brisk) |
| **seam duration** | **0.34 s … 0.19 s = 20 … 12 frames at 60 fps** |

12 frames is precisely §11.1's figure, which is the corroboration: the test's straddle window and this arithmetic agree, and both are an **order of magnitude shorter than the re-probe period**. A timer whose period exceeds the lifetime of the event it is meant to catch cannot catch it.

**Three fixes, and each is independently testable:**

1. **A regime transition is a re-probe trigger.** Any change of `regime` re-probes every disabled stage on the next frame and discards the timer. The 120-frame timer becomes the steady-state fallback for a camera that never changes regime, which is the only case it was ever good for.
2. **`seam` never disables anything.** The governor is frozen while `regime == seam`, so the disabled state cannot survive the doorway at all: the transition into the seam re-enables the outdoor stages, and the 45-frame exit dwell holds the union set long enough for the outdoor classifier to settle.
3. **Demotion state is per-regime.** Walking indoors and back out restores the outdoor regime's own demotion record. This makes a door round-trip *idempotent*, which is a one-line assertion rather than a timing argument.

**Asymmetric hysteresis, the one exception in the governor.** Entry into `seam` takes effect **immediately** (dwell 0); leaving it takes the normal **45 frames**. The asymmetry is deliberate and the justification is one sentence: the seam is the *conservative* state, so being late to enter it produces a wrong frame, while being early to leave it produces flicker — and the design's whole contract prices a wrong frame above a slow one.

**What the seam costs.** It is the only frame that pays for both partitions, and it is not a spike:

| term | value | why |
|---|---|---|
| exterior, **un-narrowed** | **35 actors, 260 k tri** | the §3.5.6 seed rule hands `index_seed` the un-narrowed frustum when the eye straddles, so the exterior is *not* discounted. §1.4a measures the same effect from the other side: with portals off, an entry door drags in exactly this set |
| interior | **4 actors**, 12.5 k tri | 2 cells (vestibule + glazed lobby) → 2 shell + 1 prop + 1 glass (§5.5a) |
| agents + HUD | 2 actors | |
| **visible actors / triangles** | **41 / 272 k** | |
| stages | outdoor set **+ `portal_seed` (~11 µs, §3.5.2) + `cell_mask` (~0.5 µs)** | the union, both live |
| **CPU [D]** | `1.20 + 1.829 + 0.054 + 0.05 + 0.35 + 0.75` = **4.23 ms → 236 fps** | the same line items as §12.1's indoor columns, and it sums |
| vs I-L portals-on (3.64 ms) | **1.16×** | the doorway costs +0.59 ms over standing inside |
| vs the same doorway with the building standing in the forest | the outdoor regime **+ 4 actors = +0.18 ms, 1.02×** | the seam is whichever outdoor regime you are in, plus two rooms |
| against §11.3 rule 8's 99th-percentile cap | **1.16× the median — well inside the 1.6× bound** | |

**That is the result worth stating plainly: the seam itself is cheap. The only thing that could have made a doorway spike was the governor, and fixes 1–3 remove it.** A design that had left `regime::seam` unimplemented would have shipped a 3.1 ms regression — 5× the seam's own +0.59 ms — that appears 1.4 seconds *after* the doorway, i.e. attributed to the wrong place by anyone profiling it.

### 10.5 Provenance is a feature, not debug output

`culler::name()` + `culler_traits` + `describe_build()` (SIMD path actually taken, FP flags, library revision) + `cull_pipeline::last_trace()` (per-stage in/out/ms ⇒ **tightness**) + the governor's decision log go into a manifest written beside every generated dataset. **No engine surveyed — OGRE, OSG, Unreal, Unity/Umbra, Godot — offers this**, and for reproducible training-data generation it is the single most valuable thing in the design.

---

## 11. Validation

### 11.1 Test targets

All appended to `TEST_TARGETS` in `src/cvc/tests/CMakeLists.txt` **and** registered with `cvc_discover_tests(<target>)`. Both are required: the directory has a configure-time drift guard (verified) that enumerates `BUILDSYSTEM_TARGETS`, fails on any executable not in `TEST_TARGETS` or in `TEST_TARGETS_INTENTIONALLY_EXCLUDED`, and whose own comment records the precedent — a target that *"sat in TEST_TARGETS, compiled green, and never ran: nothing called gtest_discover_tests() on it."* Putting `cvc::vis` tests in a separate directory would evade that guard; **we do not.**

| Target | Level | What it pins |
|---|---|---|
| `vis_frustum_test` | 0 | ~50 hand-checkable predicate cases: fully inside; fully outside each of 6 planes individually; straddling each; zero radius; empty AABB; 10⁶-magnitude coordinates; sphere containing the eye; **the `vtkCamera` L,R,B,T,FAR,NEAR reorder** |
| `vis_oracle_test` | 1 | **The highest-value test.** Seeded fuzz corpus (500 scenes × 20 views): `conservativeness_violations(strategy, reference).empty()`, and `|strategy| ≤ |reference| × 1.35`. For **exact** stages (frustum, distance, cell mask) assert set **equality**. On failure print the seed, the offending id, its bounds and the view matrix, so the case reproduces from the seed alone. |
| `vis_stage_contract_test` | 2 | rules 1–4 for every shipped stage and seed, plus a deliberately-broken mock stage that the asserts must catch |
| `vis_index_equivalence_test` | 2 | `linear_index` vs `quadtree_index` vs `grid_index` produce **identical** visible sets. This is the gate that keeps "swap the acceleration structure" a permanently safe operation. |
| `vis_determinism_test` | 2 | (a) scalar vs SIMD byte-identical; (b) 1 thread vs 8 byte-identical; (c) repeat-run `visible_set::hash()` **and `cull_scratch::hash()`** identical; (d) shuffle proxy insertion order, rebuild, re-cull → identical. **Written before the SIMD kernel.** |
| `vis_metamorphic_test` | 2 | monotonicity (widening the frustum / pushing the far plane / lowering `min_screen_px` may only **grow** the set); rigid-motion equivariance, counting near-boundary objects separately rather than fighting them |
| `vis_lod_test` | 2 | `err_world` monotonic across rungs; exactly one rung selected; hysteresis has no limit cycle over a 10,000-frame sawtooth dolly; `deterministic = true` produces no fades; viewport-size sweep so a wrong `k_px` cannot pass |
| `vis_portal_test` | 1 | 3-room + corridor graph: from room A, room C invisible; through the doorway, C visible; **camera in the doorway ⇒ both cells active**; closed portal blocks; **an outdoor manifest (0 portals) is a bit-exact no-op**; non-convex cell rejected; `OPEN` flag passes through; vertical portal honoured. Plus the three §3.5.6a/b cases: **a straddling proxy registered in cell A only is visible from B when `cell_extra` names B and invisible when it does not** (the executable form of §3.5.4 item 6); `cell_id == 0xFFFF` is never cell-masked under any `active_cells`; and a **1,000-frame random walk of 200 agents asserts `locate_coherent` ≡ `locate` on every agent-frame**, which is the only thing standing between the fast path and a silently deleted agent |
| `vis_terrain_march_test` | 1 | for random (eye, box) pairs, a claimed occlusion is re-checked against a **256-sample dense sweep**; a `paranoid` mode inflating test bounds by one texel is asserted enabled in dataset runs |
| `vis_golden_test` | 3 | 5 curated scenes × a fixed 300-frame camera path → a text golden file of `frame → sorted ids` plus a per-frame hash. Header records the seed, the culler name, the regime trace and `describe_build()`. Small, text, reviewable in a diff. **This is what the GRL-SNAM pipeline actually depends on not changing.** |
| `vis_regime_test` | 5 | a scripted path vista → descend → forest → behind ridge → doorway → **straddle** → interior → back out. Asserts: the classifier reaches the expected regime within 45 frames of the ground-truth boundary; **≤ 8 total switches**; conservative at *every* frame, especially the 12 straddle frames; no frame exceeds 1.6× the median; with `deterministic=true` and `pinned_regime` set, **exactly 0 switches**. **Four seam assertions (§10.4a), each pinning one of the four things `regime::seam` previously lacked:** (a) the classifier reports `regime::seam` on **every** straddle frame and on no non-straddle frame — the test already knew the state existed, and this is where it stops being implicit; (b) **no stage is in a demoted or disabled state while `regime == seam`**; (c) the **door round trip is idempotent** — the stage-enable set 1 frame after leaving the seam is identical to the set 1 frame before entering it, which is the executable form of per-regime demotion state and fails outright under the old 120-frame re-probe; (d) the seam frame's cost is **≤ 1.6× the run median**, so the budgeted +0.13 ms is asserted rather than assumed. |
| `cvcgl_cull_test` | integration | `VisCuller` truncates `listLength`; the default culler is gone; **`GetBounds()` is never called during `Cull`** (counted with a spy mapper) |
| `cvcgl_shadow_cull_test` | integration | renders with and without culling and **diffs the shadow buffer** — the regression test for §8.3 and for `StridedShadowBaker` under visibility churn |
| `cvcgl_batched_scene_test` | integration | LOD swap, LRU eviction, generation-budget starvation falls back to the **parent LOD, never a hole** |
| `cvcgl_occlusion_groundtruth_test` | 4 | id-buffer oracle (§11.2), at two resolutions. **Its only subject is `terrain_horizon`** — v1 has exactly one occlusion stage outdoors and one indoors, so there is no second occluder to cross-check (§3.6). One scene in its corpus is a **building cluster with vegetation behind it**, asserting `culled ∩ ground_truth == ∅` while the trees behind the buildings are *expected to survive*; that is the pinned, deliberate loss of the cut, and a future occluder stage must not break the first half of that assertion while fixing the second |

### 11.2 The oracles, and their limits

- **Frustum/distance oracle** — `reference_culler`, brute force, `double`, scalar, no index, no threads, no SIMD. Exact, and it *proves* conservativeness on the inputs tested.
- **Terrain-march oracle** — a 256-sample dense sweep along the same segment. Also a proof on the inputs tested, because the march's claim is a pointwise geometric fact.
- **Portal oracle** — exhaustive BFS to depth 8 over the cell graph. A proof for the graph as given (it does not validate the graph itself; `validate_manifest()` does that).
- **Occlusion ground truth** — render offscreen with unique-id colours via `SceneRenderer::frameRGB()`, collect every id occupying ≥1 pixel, assert `culled ∩ ground_truth == ∅`, at 1280×800 and 2560×1600 (ids visible only at the higher resolution are the interesting near-boundary cases). **This oracle is sampled: it yields counterexamples, never proofs.** The test comment must say so.

Note the deliberate design property: **a single global frustum oracle cannot validate a portal stage or an occlusion stage.** Independent per-stage oracles are what make the contract checkable rather than merely documented.

### 11.3 The benchmark harness, and how to prove a win honestly

`vis_bench` (a real cvcGL program, not a synthetic loop) plus `lsystem_lab --bench`. Eight rules, each of which exists because violating it produced a wrong conclusion during recon:

1. **vsync OFF, and enforced.** `__GL_SYNC_TO_VBLANK=0 vblank_mode=0`. The harness **refuses to report** if an empty scene runs under 300 fps or within 5% of a plausible refresh period. The entire original baseline was taken against a 55 fps ceiling (§1.1).
2. **Report milliseconds, never fps.** fps is a reciprocal and hides additive costs.
3. **Report `cpu_submit` and `gpu_wait` separately**, via a `glFinish`-bracketed probe.
4. **Always ship the culling-DISABLED column**, per regime, on the same scene and camera path.
5. **Ablate every mechanism separately.** The mandatory three-column sweep is: (a) merged actors + CPU sway; (b) batched + cull + LOD + **CPU** sway; (c) batched + cull + LOD + **GPU** sway. Column (b) exists specifically so no PR can claim GPU sway's 3.34 ms for culling, or culling's actor savings for LOD.
6. **Report `cull_ratio` and `tightness` (|PVS| / |ground truth|) alongside the time.** A fast culler that culls nothing is not a win, and a tight culler that costs 3 ms is not either.
7. **Assert image equivalence, not just speed.** Render the same frame with `reference_culler` and with the strategy under test; require a pixel diff below a fixed threshold. **A culling "win" that changes the image is a bug.**
8. **Report mean *and* 99th-percentile frame time.** Culling that improves the mean while adding spikes (LOD swaps, HLOD bakes, regime switches, cell generation) is a regression.

`vis_bench --calibrate` re-measures 44.6 µs/actor, 0.2 ns/tri and 100 ns/animated-vert on the host and stores them in `cvc::state` under `vis/calibration/*`, so the §5.3 cell-size formula self-tunes across the heterogeneous builder fleet rather than baking in one GTX 1650's numbers.

### 11.4 The mandatory loss rows

> **A benchmark report with no LOSS rows is treated as a broken harness, not as a good result.**

| Adversarial configuration | Why it should lose | Asserted bound |
|---|---|---|
| **Empty scene** | fixed per-frame cost, nothing to cull | added CPU ≤ **0.10 ms** |
| **300 objects, all visible** | below every break-even; `linear_index` + frustum should win | full pipeline within **1.2×** of brute force, and `recommend()` must return the brute-force config |
| **Vista, terrain/occlusion stages forced ON** | measured ceiling 21.5%, terrain occludes 0% | **the loss is published**; the governor must disable the stage within **30 frames** |
| **Flat thicket, canopy-only occluders** | reproduces the measured "0 of 31,927 at α = 1.0" | culled = 0, reported as a loss. It no longer guards a trunk-prism requirement (that ask is withdrawn, §3.5.5); it now guards the **[E]** in §1.5c, i.e. it is the row that would have to move before **Q2** could return a high number |
| **Camera outdoors facing a building cluster, vegetation behind it** | **no v1 stage can cull it** — §3.6 cut the only mechanism that could | culled = 0 on the occluded trees, **published as a loss row every run**, so the cut stays visible in the harness rather than only in this document |
| **Depth complexity 1.03** (flat plane, top-down) | occlusion has literally nothing to cull | culled = 0, cost ≤ 0.40 ms, reported |
| **Camera teleport every frame** | defeats every temporal assumption | **zero** conservativeness violations; 99th-percentile ≤ 1.6× median |
| **Doorway oscillation** — cross a door boundary every 20 frames for 600 frames (§10.4a) | it is the adversarial form of the regime hysteresis, and it is the exact period the old 120-frame re-probe could not see | ≤ **1 regime switch per crossing**; the terrain stage is **enabled on every outdoor frame** (the pre-fix design leaves it off for ~86 of every 120); **zero** conservativeness violations; 99th-percentile ≤ 1.6× median |
| **200 agents walking a 3-storey stair loop** | the per-agent cell locate is the only per-frame write into `scene_view::cell_id`, and a stale id deletes a *visible* agent | `locate_coherent` ≡ `locate` on every agent-frame; **zero** agents culled that the oracle keeps; locate cost reported, and reported as a loss if it exceeds the 6 ns/agent budget |
| **All 200k proxies dynamic** | upload becomes the bottleneck | harness reports the crossover object count |
| **1 giant proxy + 200k tiny** | size heterogeneity defeats uniform grids | quadtree query cost within **1.5×** of the homogeneous case |

Illustrative output shape (numbers are the design's predictions, to be replaced by measurement):

```
scene=island_1000m plants=200000 res=1280x800 vsync=off reps=5 frames=300 (30 warmup)

regime   configuration            cpu_ms  cull_ms  actors  anim_v  tight   net_ms  verdict
vista    full pipeline              2.81     0.05      35       0   1.00       -   -
vista    + terrain_horizon          3.30     0.54      35       0   1.00   -0.49   LOSS
vista    cull disabled              8.90     0.00     132       0     -     +6.09  WIN
ground   full pipeline              8.96     0.45     100       0   0.91       -   -
ground   - terrain_horizon          9.24     0.31     108       0   0.86   -0.28   -
ground   cull disabled             52.10     0.00    1104   5.1M*   0.09   +43.1   WIN
ground   (b) CPU sway, cull+LOD    64.30     0.45     100   1.22M   0.91       -   ablation
ridge    full pipeline              5.02     0.51      55       0   0.88       -   -
ridge    - terrain_horizon          8.10     0.31     100       0   0.55   +3.08   WIN
indoor-S portal seed                2.74     0.01      16       0   0.94       -   -
indoor-S index seed only            4.75     0.01      59       0   0.27   +2.01   WIN
indoor-L portal seed                3.64     0.03      31       0   0.92       -   -
indoor-L index seed only           12.75     0.02     230       0   0.13   +9.11   WIN
                                                            (* GPU-swayed => 0 CPU)
```

### 11.5 Determinism as a design constraint, not a test tactic

1. **Canonical sorted output.** Never an unordered container, never pointer order. Parallel culling writes into per-chunk output ranges at fixed offsets and compacts deterministically — never `push_back` under a lock.
2. **`-ffp-contract=off` as a per-source-file property**, mirroring the existing in-tree precedent verbatim (`set_source_files_properties(nav/material.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")`, with the `/fp:precise` MSVC branch). An FMA-contracted and a non-contracted dot product differ in the last bit and flip a boundary decision.
3. **SIMD dispatch is a determinism hazard.** Hence `view_params::deterministic` forcing scalar kernels, `culler_traits::deterministic`, and a fuzz test asserting the paths agree.
4. **No hidden time.** `view_params::frame` is explicit; hysteresis, fades and the governor all step from it. `cvc::world_clock` is threaded in where a wall clock is genuinely needed; nothing calls `now()`.
5. **Thread count is an input.** The culler takes an optional injected parallel-for and never spawns a pool — which also keeps it usable from the sim thread and from the non-pthread wasm build.
6. **Rule 4 covers scratch.** `cull_scratch::hash()` is compared, not just `visible_set::hash()`.

### 11.6 Coverage-gate practicalities

The CI gate is 80% line coverage, lcov over `src/*` and `inc/*` minus the test trees. Three consequences that shape the plan:

1. **Ship exactly two culler implementations first** — `reference_culler` and one `cull_pipeline` — over `linear_index` and `quadtree_index`. Every additional strategy without a consumer is dead code dragging the gate down, which is independently the right answer to the OGRE lesson.
2. **Compile the SIMD kernel as a real TU with a runtime switch**, never `#if`-selected inlines. An `#if` only ever measures the branch the CI machine's CPU took; a runtime switch lets one test exercise both, and both get counted.
3. Header-only templates count via inlining into the test TU, so an untested stage is a *visible* liability — which is the desired incentive.

---

## 12. Performance budgets & targets

All figures **[D]** derived from the measured coefficients unless marked **[E]**. Native = 1280×800 on the GTX 1650 reference box; wasm = 1024×640 in a browser.

### 12.1 Native, 200,000 plants

**Outdoor columns**, 200,000 plants:

| | Vista | Ground | Ridge **[E]** |
|---|---|---|---|
| visible actors | 35 | 100 | ~55 |
| visible triangles | 0.26 M | 16.5 M | ~6 M |
| CPU-animated vertices | 0 | 0 | 0 |
| VTK floor | 1.20 | 1.20 | 1.20 |
| actor submit | 1.56 | 4.46 | 2.45 |
| triangles | 0.05 | 3.30 | 1.20 |
| visibility pass | 0.05 | 0.45 | 0.51 |
| shadows | 0.55 | 0.90 | 0.85 |
| sim / agents / present **[E]** | 0.40 | 0.90 | 0.80 |
| **CPU total** | **2.81** | **8.96** | **~5.02** |
| **fps** | **356** | **112** | **~200** |
| GPU total | 0.54 | 2.98 | 1.40 |
| **headroom at 60 fps (16.6 ms)** | 13.8 ms | **7.6 ms** | 11.6 ms |
| **headroom at 120 fps (8.3 ms)** | 5.5 ms | **−0.6 ms** | 3.3 ms |

**Ground level is the binding regime and it clears 60 fps with 7.6 ms to spare, and misses 120 fps by 0.6 ms.** The two obvious levers if 120 fps becomes a requirement are shrinking band A from 140 m to 110 m (−0.8 ms of actor and triangle cost) or moving band B to a 128 m cell (−0.6 ms of actor cost, +0.9 ms of triangles — a net loss, so: band A).

> **A defect in the three columns above, recorded rather than quietly patched.** Their **CPU total** rows do not equal the sum of their own line items: vista sums to 3.81 against a published 2.81, ground to 11.21 against 8.96, ridge to 7.01 against 5.02. The indoor columns below **do** sum, because they were re-derived line by line for this revision. Reconciling the outdoor columns is a change to §0's headline numbers and to §7.3, which are outside this revision's remit; it is flagged here as open work and should be settled before any of them is quoted as a target. Nothing in the indoor derivation depends on them.

**Indoor columns**, from the §1.4a inventory and the §5.5a actor identity. Two presets, because "indoor" was previously one unsourced column and the two real generator presets behave differently enough that a single column hides the mechanism:

| | **I-S** `bunker` portals on | **I-S** portals off | **I-L** `office_3storey` portals on | **I-L** portals off |
|---|---|---|---|---|
| building | 32 × 32 m, 1 storey, 16 cells, 20 portals, 63 props | same | 60 × 40 m × 3, 150 cells, 272 portals, 1,368 props | same |
| cells reached | 7 of 16 | 12 of 16 | 10 of 150 | 96 of 150 |
| visible actors | **16** | **59** | **31** | **230** |
| visible triangles | 343 k | 303 k | 379 k | 859 k |
| — of which interior | 18.1 k | 42.4 k | 54.4 k | 598 k |
| CPU-animated vertices | 0 | 0 | 0 | 0 |
| VTK floor | 1.20 | 1.20 | 1.20 | 1.20 |
| actor submit (44.6 µs, §7.5) | 0.714 | 2.631 | 1.383 | 10.258 |
| triangles (0.2 ns) | 0.069 | 0.061 | 0.076 | 0.172 |
| visibility pass | 0.010 | 0.010 | 0.030 | 0.020 |
| shadows **[E]** | 0.15 | 0.25 | 0.20 | 0.35 |
| sim / agents / present **[E]** | 0.60 | 0.60 | 0.75 | 0.75 |
| **CPU total** | **2.74** | **4.75** | **3.64** | **12.75** |
| **fps** | **365** | **211** | **275** | **78** |
| GPU total (0.5 + 0.15 ns/tri) | 0.55 | 0.55 | 0.56 | 0.63 |
| **headroom at 60 fps (16.6 ms)** | 13.9 ms | 11.9 ms | 13.0 ms | **3.9 ms** |
| **headroom at 120 fps (8.3 ms)** | 5.6 ms | 3.6 ms | 4.7 ms | **fails** |

Every one of those columns sums to its own total. **I-L portals-on is the binding indoor case**, and the indoor win is **12.75 → 3.64 ms, 9.11 ms, a 3.5× speed-up**, removing **86.5 %** of visible actors (230 → 31) for ~11 µs of portal traversal. That remains the largest single-stage win anywhere in this design.

**What changed against the previously published indoor numbers, and why the old ones could not stand.**

| Published | Re-derived | Why |
|---|---|---|
| "400-room building" | **16 cells (I-S) / 150 cells, 114 rooms (I-L)** | The generator ships three recipes (Lab §6.7). `bunker` is BSP depth 4 on a 64×64 grid = **16 leaves**, fully determined. `office_3storey` has no published footprint, so §1.4a derives one from the Lab's own §11.2 nav-gate timing — and the **same calibration prices a 400-room building at 109 ms of gate time against the Lab's published 12 ms, a 9× contradiction.** The figure is withdrawn as unsupportable, not merely unsourced. |
| Indoor 25 actors | **31** (I-L) | 10 shell + 6 prop + 6 glass + 1 agent + 3 exterior-through-aperture + 1 HUD. The old 25 assumed 10 shell + 10 prop batches — but only 6 of the 10 reached cells are rooms; corridors and the stair core carry no props. |
| Indoor 0.5 M visible triangles | **379 k** (I-L) | Same shape, lower count: the old figure assumed 12 props in each of 10 reached cells. |
| Indoor 2.40 ms / 417 fps | **3.64 ms / 275 fps** (I-L) | Two independent corrections in the same direction. First, the old total **did not sum to its own rows** (they gave 3.40 ms, not 2.40). Second, the actor count rose 25 → 31. `bunker` lands at **2.74 ms / 365 fps**, so the old figure was roughly right for the *small* preset — not the one it was labelled with. |
| Indoor portals off ~300 actors | **230** (I-L) / 59 (I-S) | ~300 was 150 rooms × 2 batches in the 400-room building. The real frustum-only set is 96 cells of 150 — 36 on the camera's storey plus **60 on the storeys above and below**. |
| Indoor portals off ~15.15 ms / 66 fps | **12.75 ms / 78 fps** (I-L) | Direct consequence. The un-culled interior clears 60 fps with 3.9 ms in hand; it was never a 66 fps scene. |
| "portals remove ~92 % of visible actors" | **86.5 %** (I-L), 73 % (I-S) | (230 − 31) / 230. |
| "the indoor win is 15.2 → 2.4 ms", 6× | **12.75 → 3.64 ms**, **9.11 ms**, **3.5×** | Smaller than advertised and still decisively the best win in the design. |
| portal traversal "~5 µs" | **~11 µs** (I-L), ~4.5 µs (I-S) | 43 portal clips × 250 ns (§15.7). Against a 9.11 ms saving the cost is still four orders of magnitude out of the way. |

**Two consequences for sequencing, stated plainly rather than buried.**

1. §3.5.7's fallback — "if the graph does not arrive, interiors render at ~15 ms, which is still 66 fps" — is now **12.75 ms, 78 fps**. It clears 60 fps but fails 120 fps outright, and it is the *only* configuration anywhere in §12.1 that fails 120 fps other than the outdoor ground regime. **PR 9 is what makes interiors a 120 fps regime**; without it they are a 78 fps regime. That is a sharper and more defensible argument for the PR than the withdrawn one.
2. At `bunker` scale the interior traversal is nearly worthless (22 → 11 interior actors), and **32 of the 43 actors it removes are outdoor actors cut by `frustum::narrowed_by` at the doorway**. §3.5.6's seam, not the interior traversal, is the load-bearing half of the indoor design at small scale. Both presets therefore point at the same implementation priority.

**The seam column**, because the hybrid frame the design exists to serve was budgeted nowhere. Camera in the I-L entry doorway, in two cells at once (§3.5.6), classified `regime::seam` (§10.4a):

| | **Seam** — I-L entry doorway |
|---|---|
| exterior, **un-narrowed** (the seed rule, §3.5.6) | 35 actors, 260 k tri |
| interior, 2 cells | 2 shell + 1 prop + 1 glass = 4 actors, 12.5 k tri |
| agents + HUD | 2 actors |
| **visible actors** | **41** |
| **visible triangles** | **272 k** |
| CPU-animated vertices | 0 |
| VTK floor | 1.20 |
| actor submit (44.6 µs) | 1.829 |
| triangles (0.2 ns) | 0.054 |
| visibility pass (outdoor chain **+** portal seed **+** cell mask) | 0.05 |
| shadows **[E]** | 0.35 |
| sim / agents / present **[E]** | 0.75 |
| **CPU total** | **4.23** |
| **fps** | **236** |
| GPU total | 0.54 |
| **headroom at 60 fps / 120 fps** | 12.4 ms / 4.1 ms |

**The seam is the most expensive *indoor-adjacent* frame and it is not a spike: 1.16× the I-L portals-on median, and 1.02× the outdoor regime's if the same building stands in the forest.** It clears 120 fps. The thing that could have made a doorway expensive was never the geometry; it was the governor leaving the terrain stage disabled for ~86 frames after the camera stepped outside, which §10.4a costs at 3.1 ms and fixes. Budgeting the seam is what makes that failure visible as a number instead of as a mystery 1.4 s after the door.

### 12.2 wasm, 25,000 plants, reduced scale **[E]**

| | Vista | Ground | Indoor **I-S** | Indoor **I-L** |
|---|---|---|---|---|
| visible actors | 22 | 48 | **16** | **31** |
| per-actor submit | ~90 µs | ~90 µs | ~90 µs | ~90 µs |
| actor submit | 1.98 | 4.32 | 1.44 | 2.79 |
| visible triangles | — | — | 90 k | 125 k |
| triangles (0.4 ns/tri, 128-bit lane penalty) | 0.02 | 1.10 | 0.04 | 0.05 |
| visibility pass (SIMD128) | 0.10 | 0.90 | 0.03 | 0.06 |
| VTK floor + shadows + present | 2.60 | 3.20 | 2.40 | 2.40 |
| **CPU total** | **4.70** | **9.52** | **3.91** | **5.30** |
| **fps** | **~213** | **~105** | **~256** | **~189** |
| **hard budget** (`set_time_budget`) | 1.5 ms on the whole visibility pass | | | |
| **target** | | **30 fps** | | |

The indoor actor counts are the §1.4a native ones **unchanged** — a building's cell graph does not shrink with the reduced-scale profile, because it is generated from a floor plan and not from a plant target. What does shrink is the view *through* the doorway: the band-A leaf that carries 290 k of the native indoor frame's triangles drops to ~36 k at 1/8 plant density. So the wasm indoor frame is **actor-bound in exactly the same proportion as the native one**, which is the cleanest possible confirmation of §7.5.5's point.

The wasm targets are comfortable, and the reason is worth stating: the reduced-scale profile drops the plant count 8× while the per-actor coefficient only doubles, so the actor term — the dominant one — improves. **The wasm risk is not the frame budget; it is a main-thread spike**, which is what the hard time budget exists for.

### 12.3 Budget ceilings, restated

| Resource | 60 fps budget **[M]** | Ground regime usage **[D]** | Headroom | Indoor **I-L** usage **[D]** | Headroom |
|---|---|---|---|---|---|
| Visible actors | 346 | 100 | 3.5× | 31 (230 portals off) | **11.2×** (1.5×) |
| Visible triangles | 77,000,000 | 16,500,000 | 4.7× | 379,000 (859,000) | 203× (90×) |
| CPU-animated vertices | 154,700 | **0** | ∞ | **0** | ∞ |
| VRAM (4 GB card, 276 MB VTK baseline) | ~3.7 GB | 105 MB instanced / 1.11 GB merged | 35× / 3.3× | +43 MB **[D]** | — |

The indoor VRAM adder is the whole `office_3storey`: 932 k triangles × 46.8 B/vertex (§1.4a, §15.8) ≈ **43 MB**, resident, no paging — a building is 0.04 % of the card and 41 % of the size of one HLOD canopy shell set. **Indoors, the only budget line that is not comfortable by two orders of magnitude is the actor count**, and with portals disabled it is the only one inside 2×. That is the §7.5.5 point restated as a ceiling: every indoor mechanism that does not reduce submitted cells is measuring the wrong thing.

---

## 13. Implementation plan

**Shared-file discipline.** `src/cvc/CMakeLists.txt` and `src/cvc/tests/CMakeLists.txt` are owned by a concurrent session; every touch below is an **append at a verified anchor**, so a 3-way merge is trivial:

- headers → `INCLUDE_FILES`, the list closing at line **89**
- sources → `SOURCE_FILES`, the list closing at line **179**
- FP flags → immediately after the existing `nav/material.cpp` block at lines **189–192**
- tests → the `add_executable` region and the `TEST_TARGETS` list, plus a matching `cvc_discover_tests(<target>)` call (both are required; the configure-time drift guard is at lines ~1285–1315)

`src/cvcGL/CMakeLists.txt` is touched **exactly once** (PR 11, two example targets, in the block at lines 281–292), and **never** at line 24 — `FiltersCore` stays out of `CVCGL_VTK_COMPONENTS` by design (§6.3). All other cvcGL sources land through the existing `file(GLOB CVCGL_SOURCES ...)` at line 34 with no CMake edit at all.

`src/cvcGL/examples/lsystem_forest.cpp`, `src/cvcGL/examples/README.md`, and everything under `inc/cvc/nav/`, `src/cvc/nav/`, `bindings/pycvc/*.i` are **not touched by any PR below**.

| PR | Scope | New files | Shared files touched | ~LoC | Demoable outcome | Depends on |
|---|---|---|---|---|---|---|
| **0** | **Spike, NOT merged.** Measure `vtkGlyph3DMapper` through cvcGL's `SceneRenderer`: (a) per-instance and per-batch cost at 1k/10k/100k instances; (b) whether `addVertexShaderReplacement` reaches the glyph helper mapper. cvcGL has **zero** `Glyph3DMapper` usage today and the recon's own glyph harness produced invalid numbers (no GL context, an impossible 73,000 fps). **PR 6 branches on the result; the fallback is fully costed (§7.4).** | scratch only | none | 250 | a table of instanced vs merged cost, and a yes/no on shader reach | — |
| **1** | `cvc::vis` core + the oracle: `types`, `scene_view`, `view_params`, `visible_set`, `cull_scratch`, `reference`, `manifest`. Tests `vis_frustum_test`, `vis_oracle_test`. **The oracle exists before anything is validated against it.** | 7 headers + 6 .cpp + 2 tests | `src/cvc/CMakeLists.txt` (2 appends + 1 FP-flag block), `src/cvc/tests/CMakeLists.txt` (2 appends + 2 `cvc_discover_tests`) | 1,300 | `vis_bench --calibrate` prints this machine's cost coefficients; the oracle culls a 10⁵-proxy synthetic scene | — |
| **2** | Stages + pipeline + seeds: `cull_stage`, `cull_seed`, `cull_pipeline`, `layer_mask`, `distance_size`, `frustum` (scalar), `aabb_frustum`, `small_feature`; contract asserts; `last_trace`. Tests `vis_stage_contract_test`, `vis_metamorphic_test`. | 4 headers + 4 .cpp + 2 tests | 2 appends each | 1,100 | pipeline matches the oracle **exactly** on frustum + distance; the per-stage trace table prints | 1 |
| **3** | SIMD frustum: SoA kernel as a **real TU with a runtime switch**; `run_multi` override. `vis_determinism_test` **written first**. | 1 .cpp + 1 test | 2 appends + FP flags | 600 | 15k spheres: scalar 1.4 ms → SIMD 0.42 ms, **byte-identical** | 2 |
| **4** | Indices: `spatial_index`, `linear_index`, `quadtree_index` (Morton leaves, 3-state descent, plane-mask propagation), `grid_index` (CSR counting sort), `index_seed`. Test `vis_index_equivalence_test`. | 5 headers + 5 .cpp + 1 test | 2 appends each | 1,400 | 200k proxies → ~1,900 quadtree nodes in ~28 ms; identical visible sets across all three indices | 2 |
| **5** | **cvcGL adapter — the first measurable win, on the EXISTING scene.** `makeViewParams`, `VisCuller`, `setClippingRangeFromBounds`, `setStaticMesh`, `setWorldBounds`, `setCullVisible`, `updateVertices(&&)`, the `multiplyTransform` matrix-reuse fix, `CullAction`, `RenderAction::consume`. Tests `cvcgl_cull_test`, `cvcgl_shadow_cull_test`. | 8 files (all via the GLOB) | `src/cvcGL/{traversal,nodes,GeometryNode,SceneNode}.cpp`, `RenderView.h`; tests appended in `src/cvc/tests/CMakeLists.txt` | 900 | **~1.5 ms/frame recovered on the current 32-tree demo without editing it**: default culler removed, `ResetCameraClippingRange` gone, per-frame shadow re-bake gone | 4 |
| **6** | **GPU sway** — `mapDataArrayToVertexAttribute` passthrough, `SwayShader` (native + GLES3 strings), sway-inflated bounds at bake. Plus the batching carrier chosen by PR 0: `InstancedShape` (glyph) or `BatchedScene` merged-static actors. | 4 files (GLOB) | none | 900 | **the 3.34 ms/frame animation cost goes to zero**, and the 41 s/frame ceiling at 200k plants disappears | 5, 0 |
| **7** | LOD + HLOD: `lod.h`, `hlod.h`, band table, screen-error selection, hysteresis, cross-fade, residency; `vis_bake` impostor/proxy baker. Tests `vis_lod_test`, `cvcgl_batched_scene_test`. | 4 headers + 5 .cpp + 1 example + 2 tests | 2 appends each | 1,800 | **the vista at 35 actors / 2.81 ms**; ladder switches without visible popping, deterministically | 6 |
| **8** | Outdoor occlusion — **terrain only**: `terrain_field` (min/max-mip pyramids + conservative march), `terrain_horizon_stage`. Tests `vis_terrain_march_test`, `cvcgl_occlusion_groundtruth_test`. **`occluder_volume_stage` was in this row and is cut (§3.6)**; the scope, file count and LoC below are reduced accordingly, and the demoable outcome is unchanged because the ridge win was always the terrain march's. | 2 headers + 2 .cpp + 2 tests | 2 appends each | 600 | the ridge scene: 100 → ~55 actors, +3.1 ms | 7 |
| **9** | Indoor: `portal_graph`, `portal_seed`, **`cell_mask_stage`**, **`agent_locator` (`locate` / `locate_multi` / `locate_coherent`)**, `validate_manifest` (incl. the §3.5.6a id-range and `cell_extra` ordering checks and §5.5a's `check_cell_batching`), the doorway seam, the exterior hand-off. Test `vis_portal_test`. | 2 headers + 3 .cpp + 1 test | 2 appends each | 1,350 | `office_3storey` (§1.4a I-L): **12.75 ms → 3.64 ms**, on the same world, one seed swapped | 4 (independent of 6–8) |
| **10** | The governor + registry: `regime.h`, metrics (`enclosure_n`, `aperture_fraction`, both free byproducts of the seed), the **`seam` classifier and its frozen-governor policy**, **per-regime demotion state**, **transition-triggered re-probe** (§10.4a), hysteresis, pinning, `registry` on `cvc::app`, `recommend()`, provenance manifest. Test `vis_regime_test`. | 3 headers + 3 .cpp + 1 test | 2 appends each | 1,050 | the scripted path vista→ground→ridge→**seam**→indoor→back out with ≤8 switches, zero conservativeness violations, and an idempotent door round trip | 8, 9 |
| **11** | `lsystem_lab.cpp` (**new** example) + `vis_bench.cpp` + `--calibrate` + the full §11.3/§11.4 harness, including the loss rows. The example uses a surface sea and a skybox, and does not inherit the `seaField` redundancy. | 2 examples | `src/cvcGL/examples/CMakeLists.txt` (2 targets, in the 281–292 block) | 1,700 | **the deliverable**: island vista → walk into the forest → behind the ridge → enter a building, 60 fps throughout, with the honesty table | 10 |
| **12** | wasm: `-msimd128` kernel path, `--profile=web`, time budget, `_wasm_demos` entry, gh-pages card | — | `src/cvcGL/examples/CMakeLists.txt` (1 line) | 500 | 25k plants at ~30–45 fps in a browser, interiors included | 11 |

**Sequencing notes.**

- PRs 1–4 are pure `cvc::vis` with no renderer dependency and can land in parallel with any cvcGL work.
- **PR 5 is the earliest real win and it lands on the existing scene without editing it** — that is deliberate, so the substrate proves itself before anything large is built on it.
- **PR 6 is the largest win per line of code in the whole plan** and is the one to protect if schedule pressure arrives.
- PRs 8, 9 are independent of each other; 9 (indoor) is blocked only on the generator contract and can slip without affecting anything else.
- The occlusion measurement that decides whether a raster occluder is ever worth building (§1.5c, question **Q2**) should run **before PR 8**, because it may also re-scope PR 8 itself. **PR 8 has already been re-scoped once, in this revision** — from 900 LoC and two occlusion stages down to 600 LoC and one — and Q2 is the only thing that can put the second one back, as MSOC rather than as convex hulls (§3.6.5, **D9**).

---

## 14. Risks, open questions, and decisions needing the user

### 14.1 Risks

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| 1 | **`vtkGlyph3DMapper` through cvcGL is unmeasured.** cvcGL has zero usage today; the recon's harness never obtained a GL context and its numbers were discarded. Instancing carries the 105 MB VRAM story. | high | medium | **PR 0 is a non-merged spike** whose only job is to measure it. Fallback (merged static per-cell actors, ~1.11 GB with an LRU) is fully costed in §7.4 and still fits the 4 GB card. The design does not depend on which branch we take. |
| 2 | **Shader replacement may not reach the glyph helper mapper** (`vtkOpenGLGlyph3DHelper` is created internally per glyph; `addVertexShaderReplacement` goes through the actor's `vtkShaderProperty`). GPU sway is the largest single win. | medium | **high** | Same spike answers it. If it fails, the merged-actor path uses plain `vtkOpenGLPolyDataMapper`, where the replacement is a documented, in-tree-proven path. **GPU sway is non-negotiable; its carrier is negotiable.** |
| 3 | **`SetVisibility(0)` drops off-screen shadow casters.** `vtkShadowMapBakerPass` bypasses the culled array by design. | high | **high (correctness)** | `cull_multi` with the shadow view; hide only when `view_mask == 0` across all views (§8.3). Guarded by `cvcgl_shadow_cull_test`, which diffs the shadow buffer with and without culling. |
| 4 | **The batch-granularity occlusion estimate (~10%) is [E], not [M]**, and it now justifies not building **any** occlusion stage beyond the terrain march — not just the raster occluder but the cut convex-hull stage too (§3.6). It is the single largest unmeasured load-bearing number in the design. | medium | medium | Question **Q2** schedules the measurement (a one-line change to the existing `occl.cpp`) **before PR 8**. If it returns ≥60%, MSOC becomes a stage worth adding — and the stage seam exists precisely so that is an additive change. Note the *building* case is separately bounded by geometry rather than by Q2: §3.6.1's `d/D ≤ (H−1.7)/26.3` holds whatever Q2 says |
| 5 | **Terrain-march conservativeness** depends on `min_mip` being a true *minimum* reduction. A smoothed reduction would cull visible objects. | low | **high (correctness)** | The proof is one line and the test is a 256-sample dense sweep (`vis_terrain_march_test`), plus a `paranoid` mode inflating test bounds by one texel, asserted enabled in dataset runs. |
| 6 | **The portal graph is wrong** (non-convex cell, missing vertical portal, area-balanced BSP splits) ⇒ *visible* geometry culled. | medium | **high (correctness)** | `validate_manifest()` runs at load, checks convexity tolerance, portal planarity and cell/portal consistency, and **disables portal culling on a failing manifest rather than culling wrongly** — fail loud, degrade safe. Cells with `Σ portal_area / boundary_area > 0.4` are downgraded to `OPEN` (pass-through). |
| 7 | **On-demand cell generation stalls the frame** on a fast vista→ground descent (merged-actor path only; the instanced path has no paging at all). | medium | medium | 2 cells/frame on a worker; prefetch one band ahead along the camera velocity; **starvation falls back to the parent LOD, never a hole** (`cvcgl_batched_scene_test`). And **question Q4** demands the ms/cell figure before PR 7 sizes the budget. |
| 8 | **GPU vertex-shader sway cost is unmeasured.** Band A moves ~7 M swayed vertices/frame onto a GPU characterized only by a raster coefficient. | medium | medium | Question **Q3**. If it bites, band A shrinks from 140 m to 110 m (−40% swayed vertices) or sway amplitude ramps to zero earlier. Both are one-constant changes. |
| 9 | **SIMD divergence or FP contraction** breaks dataset reproducibility. | low | **high** | `-ffp-contract=off` as a source-file property following the in-tree `nav/material.cpp` precedent; `vis_determinism_test` fuzzes scalar-vs-SIMD and 1-vs-8 threads; `deterministic=true` forces scalar; `describe_build()` records the path taken into the manifest. |
| 10 | **Coverage-gate dilution** from a large new module. | medium | medium | Ship two culler implementations first; add strategies only with a consumer; SIMD as a runtime-switched TU so both paths are measured (§11.6). |
| 11 | **CMake merge conflict** with the concurrent nav session. | medium | low | Append-only at four verified anchors; the configure-time drift guard turns a botched resolution into a configure error rather than a silent skipped test; PR 1 rebases immediately before merge. |
| 12 | **PR 11's `src/cvcGL/CMakeLists.txt` edit conflicts with PR #223.** | low | low | One block, two example targets, at lines 281–292; land it alone and rebase. `FiltersCore` is never added, so line 24 is never touched. |
| 14 | **An agent's `cell_id` goes stale** — a teleport, a respawn, an amortized locate, or a mover the generator emitted as a static prop ⇒ a *visible* agent culled. This is the only per-frame write into `scene_view::cell_id` and therefore the only place the column can be wrong. | medium | **high (correctness)** | `unpartitioned_cell` (0xFFFF) is the mandated fallback for any un-updated agent, so the failure mode is one extra actor rather than a deleted one (§3.5.6b); a double miss falls through to the full BSP descent; `vis_portal_test`'s 1,000-frame walk asserts `locate_coherent ≡ locate`; §11.4 carries the stair-loop loss row. |
| 15 | **The v1 indoor batching rule is relaxed** — a per-room texture atlas, a PBR indoor material, a per-prop `vtkProperty` — and the 44.6 µs/actor coefficient silently stops transferring, invalidating every indoor column. | medium | medium | §7.5.3 states the tripwire; §5.5a's `check_cell_batching` reports the largest cell against both the 223,000-triangle split threshold and the 40,000-vertex carrier capacity into the provenance manifest, so the violation is a manifest diff rather than a mystery regression. |
| 13 | **`k_px` wrong under `ResetCamera`, tiled rendering or a resized window** ⇒ LOD and small-feature culling silently change the visible set — and for GRL-SNAM that silently changes the dataset. | medium | medium | `makeViewParams` reads `GetTiledAspectRatio()` and the actual window size every frame; `vis_lod_test` sweeps viewport sizes and asserts monotonicity. |

### 14.2 Open questions (to be answered by measurement, not by argument)

- **Q1 — What is the per-instance and per-batch cost of `vtkGlyph3DMapper` driven through cvcGL's `SceneRenderer`, and does `addVertexShaderReplacement` reach its helper mapper?** Answered by PR 0. Everything about VRAM residency and paging branches on it.
- **Q2 — What is the occlusion cull rate at 32 / 64 / 128 / 256 m *batch* granularity, as opposed to per-plant?** A one-line change to the existing `occl.cpp` (test cell-sized AABBs instead of per-plant boxes). It decides whether a raster occluder is ever worth building and may re-scope PR 8. The design's ~10% is **[E]**.
- **Q3 — What does GPU vertex-shader sway cost per vertex on this GPU?** The measured GPU model has only a *raster* coefficient (0.15 ns/tri). Band A moves ~7 M swayed vertices/frame; this is the one place the design could quietly recreate a bottleneck on the other processor.
- **Q4 — What is the wall-clock cost of building one 32 m cell** (141 plants, ~290k triangles of L-system evaluation) on a worker? Only relevant on the merged-actor branch of Q1, but if that branch is taken it sizes the paging budget and determines whether a vista→ground descent is a 0.4 s detail lag or a stall.
- **Q5 — Is L-system depth-(*n−1*) re-derivation genuinely a ~3× reduction with a stable silhouette for the *same seed*?** The 80/720/2,640/7,600 figures are a maturity *mix across different plants*, not one plant re-derived at reduced depth. §6.3 makes this the LOD generator and the reason QEM is rejected for vegetation.
- **Q6 — Does per-frame `SetVisibility` churn defeat `StridedShadowBaker`?** Its safety guard counts shadow-casting *lights*, not props, which is why the churn should be absorbed — but this is inferred, and `cvcgl_shadow_cull_test` must confirm it.
- **Q7 — What is the real per-actor submit coefficient in the wasm build** (JS ↔ wasm ↔ GL)? §12.2 assumes ~90 µs, i.e. 2× native, and the whole web actor budget derives from it. The web frame budget is tighter, so a 2× error is decisive there.

### 14.3 Decisions needing the user

> These are genuinely user-facing choices. The recommendation is stated, but the decision is not made here.

**D1 — Module target shape.** Compile `cvc::vis` into the existing `libcvc` target (appending to `INCLUDE_FILES` / `SOURCE_FILES`), or make it a separate `cvc_vis` library?
*Recommendation: append to `libcvc`.* A separate public target adds install rules, an export set, a `cvcConfig.cmake` entry, a cvcpkg recipe surface and a SWIG decision, for no technical gain — the headers are already dependency-free, which is what "reusable" actually requires. The cost is that `-ffp-contract=off` becomes a `set_source_files_properties` call rather than a target property, which the tree already does for `nav/material.cpp`.

**D2 — Instancing vs merged batching**, pending PR 0. Instanced: ~105 MB VRAM, no paging, per-instance identity preserved. Merged: ~1.11 GB with an LRU and a generation budget, needs the paging pipeline (risk #7), but uses only APIs cvcGL already exercises.
*Recommendation: instanced if PR 0 says the cost is competitive; merged otherwise.* Both fit the 4 GB card, so this is a quality decision, not a feasibility one — **but it changes ~700 lines in PR 6/7 and the user should know which way we are going before those land.**

**D3 — Does `lsystem_lab` keep the ray-cast sea and sky volumes?** They cost **7.74 ms, 34% of the current frame [M]**, and are VTK volume rendering, not visibility.
*Recommendation: no — surface sea + skybox, with the volume path retained behind a flag.* If the L-System Laboratory wants a genuinely volumetric sky for its own sake, say so, and every "after" number in §12 changes.

**D4 — Who fixes `seaField`'s 18× redundant recompute (6.30 ms, 28% of the frame [M])?** `lsystem_forest.cpp` is off-limits to this work, and the new example simply will not inherit the bug.
*Recommendation: a separate one-line PR by whoever owns that file.* Naming it here so this design cannot be credited with someone else's milliseconds.

**D5 — Can the world-generator effort commit to the §3.5.5 contract?** Now **three** load-bearing items: **Morton-sorted instances**, **BSP split planes on wall planes with a merge pass**, and — added in this revision — **`proxy_cell_extra[]`, the straddler side table (§3.5.6a)**. The third is small and is not optional: without it a prop or a door reveal spanning a doorway is registered in one cell and vanishes from the other side, which is a correctness bug the runtime cannot detect and cannot repair, because only the generator knows the second membership. `cell_content[]` is requested alongside it and *is* optional — without it `check_cell_batching` (§5.5a) degrades from a load-time check to a runtime surprise.
*Recommendation: ask now, and ask for less overall.* Without Morton order the batch/cell/draw-range identity breaks. Without wall-aligned splits the portal graph is garbage and PR 9's 3.5× indoor win evaporates. Without `proxy_cell_extra[]` the straddler case is silently wrong. **A previously-listed fourth item — opaque trunk-prism occluder proxies, plus the `occluders[]` building hulls — is withdrawn** (§3.5.5, §3.6). Both fed only the cut `occluder_volume_stage`, and the Lab roadmap's frozen §6b.6 declines to emit either, so the previous revision was asking the generator owner to break a frozen contract to author an unbounded-risk artefact for a stage with no design. Withdrawing it removes the one item of D5 the generator owner was most likely to refuse, which is a reason to expect a *yes* on the three that matter — and the one added in its place, a CSR side table the generator can emit from data it already has, is by far the cheapest of them.

**D6 — Is the wasm target 30 fps at 25,000 plants acceptable**, or should the browser demo aim higher (60 fps at ~12,000) or wider (30 fps at 50,000)?
*Recommendation: 30 fps at 25,000, interiors included.* The reduced-scale profile is a `cvc::state` subtree, so this is a runtime knob, not a rebuild.

**D7 — Shadow policy.** Vegetation shadow-distance-culled at 400 m; grass and small foliage excluded from the shadow pass entirely (Fortnite ships exactly this).
*Recommendation: accept.* It is worth ~1.7 ms and the artefact is a soft loss of distant vegetation shadows. Terrain and buildings are never distance-culled.

**D8 — Determinism scope.** The *visible set* is bit-reproducible under `deterministic=true`. The *rendered image* is not bit-identical across GPUs once sway runs in a vertex shader.
*Recommendation: accept — certify the visible set, not the pixels.* If GRL-SNAM training needs pixel-identical frames across machines, sway must be CPU-side in dataset runs (a `--frozen-wind` flag costing the 3.34 ms back on ≤ band-A cells only), and we need to know that now.

**D9 — Is *any* additional occlusion culler permanently out of scope, or gated?** Broadened in this revision: it used to ask only about MSOC, because convex occluder volumes were assumed to be shipping. They are not (§3.6), so this decision now covers every occluder mechanism other than the terrain march and portals.
*Recommendation: gated on Q2, and if the gate opens the answer is MSOC — never the convex-hull stage.* Terrain-march + portals is the v1 answer and, on the current evidence, likely the permanent one. If Q2 returns a high batch-granularity cull rate, MSOC is an additive `cull_stage` — which is exactly what the seam is for. **The convex-hull stage is strictly dominated and should not be revived**: it needs the same "occluder larger than a cell" precondition MSOC needs, it additionally cannot fuse (§3.6.2 costs that at ~40% of the win), and it additionally needs an authored inner-conservative hull that MSOC does not (§3.2's unbounded-shrink risk). **The user-facing question is therefore simply: do you accept that no building occludes exterior content in v1?** §3.6.4 bounds that at 4–6 actors ≈ 0.18–0.27 ms near a building cluster, and nothing at all in the vista, ridge, forest or indoor regimes.

**D10 — VRAM target.** The design fits the 4 GB GTX 1650 on both branches. Should it be sized for 8 GB or 24 GB machines instead, which would allow a larger band A and a richer L0?
*Recommendation: keep 4 GB as the floor* — it is the only CUDA box and therefore the box that has to run everything.

---

## 15. Appendix

### 15.1 Frustum plane extraction

From `vtkCamera` (the path we use):

```cpp
double p24[24];
cam->GetFrustumPlanes(ren->GetTiledAspectRatio(), p24);
// Order is L, R, B, T, FAR, NEAR -- the header explicitly warns it is
// "NOT near,far". Each plane is (A,B,C,D) with the inside half-space at
// A*x + B*y + C*z + D >= 0. frustum::from_vtk_planes() reorders to L,R,B,T,N,F.
```

From a row-major world→clip matrix `M = P·V` with rows `m0..m3` (Gribb–Hartmann), used by the headless tests where no `vtkCamera` exists:

```
L = m3 + m0     R = m3 - m0     B = m3 + m1
T = m3 - m1     N = m3 + m2     F = m3 - m2
normalize each by |(x, y, z)|
```

### 15.2 Sphere vs frustum, SoA

For plane *i*: `s_i = n_i · c + d_i`.
Outside ⟺ `s_i < −r` for any *i*. Fully inside ⟺ `s_i ≥ +r` for all *i*.

The SoA transpose packs planes 0–3 as `x0x1x2x3 / y / z / w` and duplicates planes 4,5 as `x4x5x4x5 / …`, so four dot products become three FMAs and **two spheres are tested per iteration**, accumulating a per-proxy bitmask of which frusta passed:

```
d0123   = soa[0]*cx + soa[1]*cy + soa[2]*cz + soa[3]        // 3 FMA
outside = movemask(d0123 + splat(r) < 0)
```

Frostbite measured this exact layout at **15,000 spheres in 1.0 ms single-job / 0.32 ms across four** on a 2.66 GHz i7 [Collin 2011]. `-ffp-contract=off` is mandatory on this TU: contracted and non-contracted results differ in the last bit and flip boundary decisions.

### 15.3 AABB vs frustum — p/n-vertex with plane masking

```cpp
// Returns OUTSIDE / INTERSECT / INSIDE. `mask` carries which planes the PARENT
// did not already fully satisfy; a fully-satisfied plane is cleared for the
// whole subtree. INSIDE => emit the entire subtree with ZERO further plane tests.
inline int aabb_vs_frustum(const aabb& b, const frustum& f, std::uint8_t& mask) {
  int result = INSIDE;
  for (int i = 0; i < 6; ++i) {
    if (!(mask & (1u << i))) continue;
    const plane& p = f.p[i];
    // p-vertex: the corner farthest along n. Chosen by sign bits, no branch.
    const float px = p.n[0] >= 0 ? b.mx[0] : b.mn[0];
    const float py = p.n[1] >= 0 ? b.mx[1] : b.mn[1];
    const float pz = p.n[2] >= 0 ? b.mx[2] : b.mn[2];
    if (p.n[0]*px + p.n[1]*py + p.n[2]*pz + p.d < 0) return OUTSIDE;
    // n-vertex: the opposite corner. If it is inside too, the plane is satisfied
    // for the whole subtree.
    const float nx = p.n[0] >= 0 ? b.mn[0] : b.mx[0];
    const float ny = p.n[1] >= 0 ? b.mn[1] : b.mx[1];
    const float nz = p.n[2] >= 0 ? b.mn[2] : b.mx[2];
    if (p.n[0]*nx + p.n[1]*ny + p.n[2]*nz + p.d < 0) result = INTERSECT;
    else mask &= ~(1u << i);
  }
  return result;
}
```

One dot product per plane instead of eight corner tests. Three optimizations in payoff order: **(1) the fully-inside short-circuit** — the single largest constant-factor win and the most commonly omitted; **(2) plane masking** propagated down the descent, roughly halving plane tests at depth; **(3) SoA/SIMD over sibling nodes**. The octant test [Assarsson & Möller 2000] is *not* implemented: with (1) and (2) already in place its marginal value does not justify the code.

Standard conservatism note: an AABB outside the frustum but not fully outside any single plane (a "frustum corner" false positive) is reported visible. Harmless — you draw a few extra objects — and we do not add the reverse test.

### 15.4 Screen-space size, error and area

With viewport height `H` px and vertical fov `θ`:

```
k          = (H / 2) / tan(θ / 2)                  # 800/2 / tan(21 deg) = 1042 px.m/m
d          = max(z_near, |eye - c| - r)            # bound-NEAREST, never centre
screen_px  = k * r / d                             # projected radius
area_px    = pi * screen_px^2 * fill               # fill ~ 0.30 for a conifer silhouette
err_px(L)  = k * err_world(L) / d                  # LOD L's world error, in pixels
```

**Distance culling and small-feature culling are the same test**, which is a real simplification over maintaining two mechanisms:

```
cull iff  2 * r * k / d  <  min_screen_px
     iff  d  >  d_max(r) = 2 * r * k / min_screen_px
```

That single line reproduces Unreal's entire Cull-Distance-Volume behaviour (bucket by bounding-sphere diameter → cull distance), computed rather than authored:

| bucket | world radius | `d_max` at `min_screen_px` = 1.5 **[D]** | culls on a 1 km island? |
|---|---|---|---|
| pebble / debris | 0.10 m | 139 m | yes, aggressively |
| grass tuft | 0.30 m | 417 m | yes |
| shrub / rock | 1.0 m | 1,389 m | yes, at the far shore |
| small tree | 4.0 m | 5.6 km | no |
| mean tree (r = 5.25 m) | 5.25 m | 7.3 km | **never** |

**Consequence, stated plainly:** small-feature and distance culling handle *clutter*. They do **not** cull trees on this island — a 24 m tree is still 11 px tall from 2.2 km. Trees are made cheap by the HLOD ladder (§6.4), not by these stages. Anyone who claims small-feature culling solves the vista has not done this arithmetic.

### 15.5 The terrain horizon march

```
occludes_aabb(eye, box):
    for each of the 8 corners c:
        if !occludes_segment(eye, c): return false     # CONSERVATIVE: all must block
    return true

occludes_segment(a, b):                                # max-mip skip / min-mip confirm
    t = 0;  level = L_top
    while t < 1:
        p = lerp(a, b, t);  foot = footprint(p, level)
        if   max_mip[level](foot) < p.z:  t += step(level); level = min(level+1, L_top)
        elif level > 0:                   level -= 1
        else:
            if min_mip[0](foot) > p.z: return true      # PROVEN blocked
            t += step(0)
    return false
```

**Conservativeness proof, in one line:** the real terrain height at any point is ≥ the minimum over any footprint containing it; therefore `min_mip > ray_height` implies real terrain is above the ray, i.e. the ray is blocked. There is no epsilon to tune, no inner-conservative mesh to author, and no resolution-dependent shrink — which is the entire reason this is preferred to a low-resolution software rasterizer (§3.2).

Cost **[E]**: ~64 steps per ray, 8 rays per candidate, ~100 candidates → ~51k array reads ≈ **140 µs** at 1280×800. Memory **[D]**: 1024² base + mip chain × 2 pyramids × 4 B = **11.2 MB**, built once at load in ~9 ms.

### 15.6 Cell-size derivation, generalized

```
cells(S)  = 0.1753 * pi * (R1^2 - R0^2) / S^2         # interior, for a 63.1 deg hfov wedge
          + ( 2*(R1 - R0) + 1.101*(R1 + R0) ) / S     # boundary

cost(S)   = c_actor * cells(S)
          + c_tri   * cells(S) * density * S^2 * tris_per_plant

constraint: S <= (R1 - R0) / 4                        # >= 4 cells across the LOD band
```

with `c_actor = 44.6 µs` and `c_tri = 0.2 ns` **[M]**, re-measured per host by `vis_bench --calibrate`. For a different *outdoor* libcvc scene — a molecular surface, a city block — the same formula with different inputs yields a different leaf size, which is what `vis::recommend_leaf_size` computes and logs.

**The indoor form, because the above does not apply to an interior and calling it there is a bug.** A portal cell's size is chosen by the floor plan, not by the runtime, so there is no `S` to solve for and the boundary over-draw term is identically zero (a cell boundary is a wall). The free variable is `k`, the actors per cell (§5.5a):

```
split a cell into k batches only if
    (k - 1) * c_actor  <  c_tri * E[triangles the narrowed frustum rejects]

    equivalently:  E[rejected]  >  (k - 1) * c_actor / c_tri  =  (k-1) * 223,000 tri

merging m cells into one batch:
    saves   (m - 1) * c_actor
    costs   the entire cell-mask win, because a multi-cell actor must be drawn
            whenever ANY of its cells is active
    => forbidden; SS5.5a prices the exchange on the shipped presets

carrier bound (independent):  one batch <= 40,000 verts / 68,000 tris   [Lab SS8.6]
```

`vis::check_cell_batching(cell_content[], measured_coefficients)` evaluates all three at load and writes the margins into the provenance manifest. **Both halves consume the same two measured coefficients, which is the point: one cost model, two geometries, two different free variables.**

### 15.7 Master comparison table

| Structure | Build | Query | Memory | Dynamic | Difficulty | Fusion | Our use |
|---|---|---|---|---|---|---|---|
| Uniform grid (CSR) | O(n+C+R) | volume-proportional | 4(C+1)+4R B | **O(1)** | easiest | n/a | **agents** |
| Hierarchical grid | O(n) | Σ levels | flat | O(1) | easy-med | n/a | rejected — no hierarchical rejection |
| Octree | O(n·d) | O(vis + log n) | 40–64 B/node | O(d) | medium | n/a | **rejected — straddling + 20:1 aspect** |
| Loose octree | O(n) | 2–3× node visits | 40–64 B/node | **O(1)** | medium | n/a | rejected — is an hgrid with a spine; wastes Y |
| kd-tree | O(n log n) | O(log n) exact | 8 B/node, 2–5× dup | **hostile** | hard | n/a | rejected — no refit; wrong granularity |
| BSP | O(n log n)–O(n²) | O(n) | 24 B/node + inflated geom | **none** | hard | n/a | **generation-time only** (convex cells) |
| BVH (binned SAH) | O(n log n) | **O(k + log n)** | 32 B × (2n−1) | refit | medium | n/a | future `spatial_index`; not v1 |
| **XZ quadtree + Y** | Morton + sweep | O(vis + log n) | 40 B/node | rebuild leaf run | med-easy | n/a | **static content** |
| **Cells + portals** | emitted | O(cells × portals) | ~200 B/cell | contents dynamic | medium | **yes** | **interiors** |
| PVS | superlinear bake | **O(1)** | leaves²/8 RLE | **static** | hard | yes | rejected — the bake |

| Algorithm | Cost/candidate | Conservative | Fusion | Latency | Our use |
|---|---|---|---|---|---|
| layer mask | 1.5 ns | yes | n/a | 0 | shipped |
| **cell mask** (`active_cells` AND `cell_id`) | **1.5 ns** | **exact** | n/a | 0 | **shipped** (§3.5.6a) |
| distance / size bucket | 3 ns | approximate by choice | n/a | 0 | shipped |
| frustum sphere (SoA) | 6 ns/view | exact | n/a | 0 | shipped |
| frustum AABB (p/n-vertex) | 14 ns | conservative | n/a | 0 | shipped |
| small feature | 4 ns | **no** (declared) | n/a | 0 | shipped |
| terrain horizon march | 1.4 µs | **yes**, proven | yes | 0 | **shipped** |
| convex occluder volume | 0.9 µs = **64 hulls × 14 ns [D]**, §3.6.3 | yes | **no** | 0 | **cut — §3.6.** Dominated by MSOC on every axis |
| portal traversal | 250 ns/portal | yes | yes | 0 | **shipped** |
| PVS lookup | O(1) | yes | yes (baked) | 0 | rejected |
| HW occlusion query | 1 state change + 1 box raster | yes | yes | **≥1 frame** | rejected |
| CHC++ | amortized ≪1 query/node | yes | yes | 0–1 frame | documented fallback |
| MSOC raster | 11.9 ns/occluder-tri + 120 ns/test | yes (±1 px) | yes | 0 | gated on Q2 |
| GPU two-phase HZB | ~0 CPU | yes | yes | 0 | rejected (impossible in wasm; 2× slower below 100 M tris) |
| depth reprojection | cheap | **no** | yes | 1 frame | rejected |
| SDF visibility | cheap | **no** | yes | 0 | future LOD *hint* only |

### 15.8 Constants, in one place

| Quantity | Value | Source |
|---|---|---|
| per visible actor | **44.6 µs** | [M] `actorbench`, linear 128–2048 actors |
| per visible triangle (CPU / GPU) | **0.2 ns / 0.15 ns** | [M] 2 M tris cost the same as 1,024 |
| per CPU-animated vertex | **100 ns** | [M] two independent harnesses, 84–118 ns |
| `SetVisibility` | **224 ns** | [M] |
| VRAM per full-detail tree | **82 KB** (46.8 B/vertex) | [M] |
| actor ≡ triangles | **1 : 223,000** | [D] |
| animated vertex ≡ triangles | **1 : 500** | [D] |
| 60 fps ceilings | 346 actors / 77 M tris / 154,700 anim verts | [D] |
| `k_px` @ 1280×800, 42° vfov | **1042 px·m/m** | [D], validated against measurement |
| hfov @ 1.6 aspect | **63.1°**, 17.53% of a disc | [D] |
| full detail stops paying | **199 m** (1 tri/px) | [M] |
| impostor crossover | **342 m** (width < 32 px) | [M] |
| band A / B / C / D | 140 / 350 / 900 m | [D] §6.2 |
| near cell / far cell | **32 m / 512 m** | [D] §5.3 |
| plants per 32 m cell | 141 | [D] at 0.138 plants/m² |
| LOD ratio per L-system iteration | **~3× ** (80/720/2,640/7,600) | [M] |
| `radius_scale` at L1 | **1.35×** = √(N_n / N_{n−1}) | [D] |
| `lod_error_px` τ / hysteresis | 4 px / ±8% | design |
| `min_screen_px` | 1.5 px | design (4 px reaches 41.6% at vista [M]) |
| canopy occlusion contribution | **exactly 0** at α ∈ {0.2, 0.4, 0.7, 1.0} | [M] |
| occluder budget | ~6,000 triangles | Frostbite ships ~6k; AC Unity 300 best occluders. **Unused in v1** — no stage consumes an occluder set (§3.6); kept as the sizing figure a future MSOC would inherit |
| building occlusion reach | `d/D ≤ (H − 1.7)/26.3` → **0.334 / 0.240 / 0.087** for office / warehouse / bunker | [D] §3.6.1 — why buildings do not occlude a 32 m cell |
| terrain pyramids | 1024² × 2 × mips = 11.2 MB | [D] |
| max cells (portal bitset) | 4,096 (512 B, fits a UBO) | design — **27× the largest generated interior** (§1.4a: 150 cells); kept because 512 B is free, but nothing the generator ships approaches it |
| indoor cells / portals, `bunker` | **16 / 20** (2.5 portals per cell) | [D] §1.4a from Lab §6.7 (BSP depth 4 on 64×64) |
| indoor cells / portals, `office_3storey` | **150 / 272** (3.6 portals per cell) | [D] §1.4a, plate calibrated to Lab §11.2's 12 ms nav gate |
| indoor rooms, `office_3storey` | **114** (38/storey, 49 m² mean) | [D] §1.4a. **The "400-room building" is withdrawn** — the same calibration prices it at 109 ms of gate time against a published 12 ms |
| interior shell / prop | **600 tri** each; 12 props per office room, 4.8 per bunker room **[E]** | §5.5a, §1.4a — the Lab specifies the Poisson mechanism and no density |
| indoor batch identity | **1 batch per (cell, blend class)** — 2 unglazed, 3 glazed | §5.5a, §1.4a |
| indoor programs bound per frame | **3 max** (opaque, transparent, HUD) — independent of building size | §7.5.2 |
| indoor texture binds | **0** in v1 (albedo is vertex colour) | §7.5.2 — the constraint that makes 44.6 µs transferable indoors |
| program rebind | **~3 µs [E]** — wants measuring in `vis_bench --calibrate` | §7.5.3 |
| `c_actor` indoor, sorted | **45.15 µs** worst case (I-S), +1.2 % over 44.6 | [D] §7.5.3 |
| portal traversal, corridor view | **~11 µs** (I-L, 43 clips), ~4.5 µs (I-S) | [D] §3.5.2 at 250 ns/portal |
| indoor cull rate | **86.5 %** of visible actors (230 → 31) | [D] §12.1 |
| CPU-animated vertices, indoor | **0** — no sway; agents are rigid L0 meshes | §1.4a |
| governor band / dwell / re-probe | 1.3× / 3.0× · 45 frames · 120 frames | design |
| `seam` entry / exit dwell | **0 / 45 frames** (the one asymmetry) | design §10.4a |
| re-probe on a regime transition | **next frame**, timer discarded | design §10.4a |
| doorway seam duration | **12–20 frames** (0.85 m at 1.4–2.5 m/s) — 6–10× shorter than the 120-frame re-probe | [D] §10.4a |
| actors per reached portal cell | **1 shell + 1 prop (rooms) + 1 glass (glazed)** | design §5.5a, §7.5.2 |
| indoor split threshold `c_actor / c_tri` | **223,000 tri** — 36–86× the mean generated cell | [D] §5.5a |
| interior batch carrier capacity | 40,000 verts / **68,000 tris** | [L] Lab §8.6 |
| `cell_id` sentinels | **0** = exterior, **0xFFFF** = unpartitioned/always-active | design §3.5.6a |
| agent cell locate, coherent | **~6 ns/agent** (98.4 % same-cell, 1.6 % neighbour) | [D] §3.5.6b |
| agent cell locate, naive BSP descent | 32 ns/agent (depth 8) — 216 ns with no `bsp[]` at all | [D] §3.5.6b |
| wasm visibility time budget | **1.5 ms**, hard | design |

### 15.9 Sources

**Measured, this project**
- Recon pass A — VTK 9.5.0 / cvcGL capability audit (headers from `deps-live/include/vtk-9.5`, implementations from `VTK-upstream` at tag `v9.5.0`).
- Recon pass B — instrumented `lsystem_forest` (`lsf_inst`), `actorbench`, and a software z-buffer occlusion study (`occl.cpp`). All timings vsync-off unless stated.
- In-tree verification for this document: `inc/cvc/core/app.h` (the `boost::any` data map), `src/cvc/CMakeLists.txt` (append anchors, the `nav/material.cpp` FP-flag precedent), `src/cvc/tests/CMakeLists.txt` (`cvc_discover_tests`, the `TEST_TARGETS` configure-time drift guard, `TEST_TARGETS_INTENTIONALLY_EXCLUDED`), `src/cvcGL/CMakeLists.txt` (`CVCGL_VTK_COMPONENTS` without `FiltersCore`; `file(GLOB CVCGL_SOURCES)`).

**Books**
- Akenine-Möller, Haines & Hoffman, *Real-Time Rendering*, 4th ed. (2018) — ch. 19 (spatial data structures, frustum culling, portal culling, occlusion culling), ch. 22 §22.2–22.3 (bounding-volume tests).
- Ericson, *Real-Time Collision Detection* (2005) — ch. 6 (BVH), ch. 7 (grids, hgrids, spatial hashing, trees).

**Papers**
- [Fuchs et al. 1980] Fuchs, Kedem & Naylor, "On Visible Surface Generation by A Priori Tree Structures", SIGGRAPH '80.
- [Naylor et al. 1990] Naylor, Amanatides & Thibault, "Merging BSP Trees Yields Polyhedral Set Operations", SIGGRAPH '90.
- [Teller & Séquin 1991] "Visibility Preprocessing for Interactive Walkthroughs", SIGGRAPH '91.
- [Greene et al. 1993] Greene, **Kass** & Miller, "Hierarchical Z-Buffer Visibility", SIGGRAPH '93. *(Frequently miscited as "Greene, Kay & Snyder".)*
- [Luebke & Georges 1995] "Portals and Mirrors: Simple, Fast Evaluation of Potentially Visible Sets", I3D '95.
- [Gottschalk et al. 1996] "OBBTree", SIGGRAPH '96. [Klosowski et al. 1998] k-DOPs, IEEE TVCG 4(1).
- [Assarsson & Möller 2000] "Optimized View Frustum Culling Algorithms for Bounding Boxes", JGT 5(1).
- [Ulrich 2000] "Loose Octrees", *Game Programming Gems* vol. 1, ch. 4.11.
- [Cohen-Or et al. 2003] Cohen-Or, Chrysanthou, Silva & Durand, "A Survey of Visibility for Walkthrough Applications", IEEE TVCG. **The conservative/exact/approximate/aggressive taxonomy and the tightness metric.**
- [Teschner et al. 2003] "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV.
- [Bittner et al. 2004] Bittner, Wimmer, Piringer & Purgathofer, "Coherent Hierarchical Culling", CGF 23(3).
- [Wald 2007] binned SAH. [Wald et al. 2007] "Ray Tracing Deformable Scenes using Dynamic BVHs", ACM TOG 26(1).
- [Mattausch et al. 2008] Mattausch, Bittner & Wimmer, "CHC++: Coherent Hierarchical Culling Revisited", CGF 27(2).
- [Karras & Aila 2013] "Fast Parallel Construction of High-Quality BVHs", HPG.
- [Hasselgren et al. 2016] Hasselgren, Andersson & Akenine-Möller, "Masked Software Occlusion Culling", HPG.

**Engine and vendor material**
- [Karis et al. 2021] Karis, Stubbe & Wihlidal, "A Deep Dive into Nanite Virtualized Geometry", SIGGRAPH 2021 Advances in Real-Time Rendering.
- [Wihlidal 2023] "Bringing Nanite to Fortnite Battle Royale in Chapter 4", Epic tech blog.
- [Lauritzen & Olsson 2023] "Virtual Shadow Maps in Fortnite Battle Royale Chapter 4", Epic tech blog.
- Epic Docs — Visibility and Occlusion Culling; Precomputed Visibility Volumes; World Partition and HLOD; Landscape Technical Guide; Instanced Static Mesh Component.
- [Haar & Aaltonen 2015] "GPU-Driven Rendering Pipelines", SIGGRAPH 2015 (AC Unity + Trials).
- [Collin 2011] "Culling the Battlefield: Data-Oriented Design in Practice", GDC 2011; and Hill & Collin, *GPU Pro 2*.
- [Anagnostou 2017] "How Unreal Renders a Frame", Interplay of Light.
- Kitware — "WebGPU Occlusion Culling in VTK" (**the 0.5× at 10 M / viable at 100 M+ measurement**); "Rendering Engine Improvements in VTK" (instancing 1–1.5k → 100k objects).
- OGRE forums and API docs (SceneManager plugin history, Ogre 2.x collapse); OpenSceneGraph `CullSettings`/`CullingSet`; Godot proposal #3920 (rooms-and-portals not ported to Godot 4); Unity `BatchRendererGroup` / Umbra.
- WebGL 2.0 Specification, query objects — the "must not be made available until control has returned to the user agent's main loop" clause; Emscripten SIMD porting guide.
