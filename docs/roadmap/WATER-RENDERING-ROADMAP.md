# Above-Sea-Level Water — Design & Roadmap

**Status:** design, revision 1. **Target:** `libcvc`, worktree `wt-libcvc-lsyslab`, rebased on `origin/master @ 8b6f426`.
**Companion:** `docs/roadmap/LSYSTEM-LABORATORY-ROADMAP.md` (the "Lab roadmap"). This document extends it; every rule it inherits is cited by section.
**All performance figures are measured on `prettyhatemachine`** (Xeon E5-2650 v2 @ 2.6 GHz, GTX 1650, NVIDIA 595.84, VTK 9.5.0, Release, offscreen on `:1`, `__GL_SYNC_TO_VBLANK=0`) unless explicitly marked PROJECTED or UNMEASURED.

---

## 0. Executive summary, and THE DIRECT ANSWER

### The question

> *"How would we efficiently handle bodies of water above sea level? Maybe we can use volumetric meshes? We support tet and hex meshes — if we have a nice renderer for those maybe they can be used for volumetric water pools, streams, etc. How feasible is this?"*

### The answer, in one paragraph

**Feasible to build: yes — cvcGL needs roughly 250 lines for a new `UnstructuredVolumeNode` and no build-system changes. Correct choice for water: no, and the margin is not close. Ship it anyway, as an opt-in scientific tier for one specific case that is real CVC identity work.**

Three measured facts decide it.

1. **`vtkProjectedTetrahedraMapper` costs ≈ 0.43 µs per tetrahedron per frame, and that cost is 100 % CPU.** Measured `ms == cpu_ms` at every mesh size from 2,400 to 998,784 tets. The renderer we are extending is *already CPU-bound*: quartering the pixels in `lsystem_forest` bought only +18 % (29.2 vs 24.8 fps). Projected Tetrahedra spends its entire budget on the one resource we have none of.
2. **The cost is completely insensitive to viewport and to camera distance.** 48,600 tets: 19.93 ms at 640×400 vs 19.16 ms at 1280×800. Same mesh shrunk to 6 % of its linear screen size: 18.73 ms vs 18.60 ms. There is no fill relief, no distance-LOD relief, and no culling relief. **A pond 400 m away costs exactly what it costs at your feet.** For an open-world fly-through this is disqualifying on its own.
3. **For homogeneous water a closed surface produces the *identical* image, not an approximation of it.** [Max 1995]: transmittance along a ray is `T = exp(−∫τ ds)`; if `τ` is constant then `∫τ ds = τ·L` and `L` — the chord length — is fully determined by the boundary surface. The tetrahedra contribute exactly zero information. This is arithmetic, not taste. And `vtkOpenGLProjectedTetrahedraMapper`'s own fragment shader is 27 lines long whose entire body is `opacity = 1.0 - exp(-fattenuation*fdepth)` — **PT computes Beer–Lambert too**. Its whole cost is spent deriving `fdepth` per tet on the CPU instead of analytically in a fragment shader.

Head-to-head, same water body, same camera, PT with its float framebuffer already disabled:

| hexes | tets | surface tris | PT ms | closed surface, as measured (2 draws) | **PT / surface** |
|---|---|---|---|---|---|
| 2,000 | 12,000 | 2,400 | 4.18 | 1.24 | **3.4×** |
| 16,928 | 101,568 | 11,408 | 40.35 | 1.26 | **31.9×** |
| 40,960 | 245,760 | 21,504 | 103.45 | 1.47 | **70.5×** |

The surface is **flat in body size** because it is fill-bound; PT is linear in cell count because it is CPU-bound. The gap widens without limit. A **million-triangle** surface costs 2.06 ms as measured — 4.9 % of the `lsystem_forest` control frame.

**These surface numbers were measured with the two-draw composite that §5.2 has since replaced with a single draw.** They are therefore an *upper bound* on what W1 actually costs, and every ratio in the table is a *lower bound* on PT's disadvantage. The verdict on tetrahedra is decided by the ratios, so it is unaffected by the mechanism change and is not reopened; the corrected single-draw budgets are in §7.1.

And PT gives up, structurally and untunably: **all lighting** (`vtkVolumeProperty::SetShade` and every knob in `inc/cvc/gl/VolumeNode.h:64-100` is silently ignored), **all shadows** (`vtkShadowMapBakerPass` bakes through `vtkOpaquePass` only, so the Lab's tree shadows fall *through* the lake), **clipping planes** (`vtkUnstructuredGridVolumeMapper` has no `SetClippingPlanes`), **correct sorting on non-convex domains** (`vtkVisibilitySort.h:11-15` states subclasses need not be correct and that cycles can make an ordering *not exist*; a lake with an inlet or a peninsula is non-convex by construction), and **geometry near the camera** (`vtkOpenGLProjectedTetrahedraMapper.cxx:764-775` culls with `||` not `&&`, so any tet with a single vertex behind the near plane vanishes entirely).

### The recommendation

**Water is a surface with a scalar column. Tetrahedra are a scientific-visualization capability that happens to also render water.**

| Tier | What | Where it runs | Cost |
|---|---|---|---|
| **W0** wet material | no geometry; class + albedo/roughness in the terrain splat | everywhere | 0 |
| **W1** surface + absorption | closed clipped lid, per-vertex baked column, **single opaque draw**, chromatic Beer–Lambert over a shader-evaluated bed | **default** for lakes, ponds, pools | 28 µs CPU/actor + 0.26 ms/Mpx fill |
| **W2** flow ribbon | W1 core + two-phase flow map | streams, rivers | same |
| **W3** fall sheet + spray | constant thickness × erosion α, gravity-scaled scroll, billboards | waterfalls | same + particles |
| **W4** bounded heterogeneous march | W1 shell + 32-step ray-march of a low-res 3-D σ texture between surface and bed | sediment plumes, thermoclines | ~0.3 ms fill |
| **W5** unstructured tets (PT) | `UnstructuredVolumeNode` | **opt-in, native only, capped, paused-camera** | 0.43 µs/tet, 100 % CPU |

Three code paths (surface core, bounded march, PT), three parameterisations of the first. **W4 is the tier that removes the last honest argument for tets in this product**: it gives genuinely heterogeneous absorption with *no sort, no connectivity, no k-buffer, no OIT*, composites correctly by construction (one ray, front-to-back, inside one fragment), runs in WebGL2 today, and degrades gracefully to pure Beer–Lambert when the field is uniform.

**What survives for W5, and it is exactly one case:** the field lives natively on a solver's tetrahedral or hexahedral mesh whose adaptive refinement *is* the information, and resampling it to a regular grid is the thing you cannot do. That is FEM/CFD volume visualization — boundary-layer meshes, LBIE output, `cvc tetrahedralize` / `hexahedralize` results, patient-specific hemodynamics, ocean models on unstructured cells [Morrical 2020]. It is real CVC identity work, `UnstructuredVolumeNode` is the first unstructured path cvcGL will have, and 20 fps at 100 k tets with a parked camera is a perfectly good number for it. **It is not the demo's water renderer, and the user should hear that plainly.**

### The measured scaling limit for W5, stated once

Against the Lab's own 21 ms frame budget (§11.1 of the Lab roadmap, target ≥ 45 fps), a 10 % water allocation is 2.1 ms:

| water budget | max tets (float FBO **off**) | equivalent |
|---|---|---|
| 5 % of 21 ms (1.05 ms) | ~2,400 | one 20×20×3 pond |
| **10 % of 21 ms (2.1 ms)** | **~4,800** | **one 28×28×3 pond — the whole scene's tet budget** |
| 10 % of the 40.32 ms control | ~9,000 | one 27×27×2 lake |
| 20 % of the 40.32 ms control | ~18,300 | two 15 m lakes |

**`max_tets = 4800` is the shipped default cap, one body, native only.** See §6 for why the fixed term is disputed and what must be re-measured before that number is trusted below ~10 k tets.

---

## 1. The measured evidence

### 1.1 The context: we are CPU-bound

Ground-truth control, `lsystem_forest`, 32 trees, Release:

| configuration | fps | ms/frame |
|---|---|---|
| 1280×800, shadows ON | 24.8 | 40.32 |
| 1280×800, shadows OFF | 21.7 | 46.08 |
| **640×400, shadows ON (¼ the pixels)** | **29.2** | **34.25** |

Quartering the pixels bought +18 %. Attributing the 6.07 ms delta to ¾ of the fragment work puts **full-resolution fragment cost at ≈ 8.1 ms and everything else at ≈ 32.2 ms**. There is roughly 32 ms of idle GPU per frame and essentially no idle CPU.

**The design rule that follows:** *any technique whose cost lands on the CPU is guilty until measured; any technique whose cost lands on fill is presumed affordable.* Every decision below is an application of that rule.

### 1.2 Projected Tetrahedra, measured

1280×800, moving camera, best-of-5, `vtkOpenGLProjectedTetrahedraMapper` (returned by `vtkProjectedTetrahedraMapper::New()` via the object factory, registered at `Rendering/VolumeOpenGL2/CMakeLists.txt:47`):

| tets | fps | ms/frame | cpu ms | sort ms | µs/tet |
|---|---|---|---|---|---|
| 9,600 | 192.4 | 5.196 | 4.955 | 0.662 | 0.54 |
| 34,656 | 80.0 | 12.496 | 12.484 | 2.486 | 0.36 |
| 98,304 | 25.5 | 39.245 | 39.196 | 7.515 | 0.40 |
| 198,744 | 11.9 | 84.149 | 82.777 | 16.545 | 0.42 |
| 998,784 | 2.4 | 423.998 | 423.744 | 86.961 | 0.42 |

`ms == cpu_ms` at every size. **The GPU is idle.** Fitted (tets ≥ 9,600): `ms = 3.9 + 0.000429·ntets`, of which sorting is `0.000086·ntets`.

**The visibility sort is only 15–20 % of the cost, and the architect's prior was wrong about the mechanism.** Eliminating the sort entirely — VTK's own TODO at `vtkOpenGLProjectedTetrahedraMapper.cxx:552-556` invites the caching (`vtkCellCenterDepthSort::InitTraversal` caches cell *centers* but unconditionally re-sorts every frame, `Rendering/Core/vtkCellCenterDepthSort.cxx:150-181`) — would leave 80 % on the table. The other 80 % is structural: `TransformPoints` projects every point on the CPU, the Shirley–Tuchman [Shirley & Tuchman 1990] classification with the Wylie 2002 segment variant runs per tet per frame, and the VBO is repacked and re-uploaded per sort chunk. **PT is a CPU algorithm that uses the GPU only as a rasterizer.** Do not spend a sprint caching the sort.

### 1.3 The float-framebuffer tax — the largest single finding, and a live confound

Every PT mapper owns a private float FBO and blits the **whole screen** in (colour+depth, `:485-524`) and out (colour, `:1075-1091`) per `Render`. `UseFloatingPointFrameBuffer` defaults to **true** (`:103`). Measured, N ponds of **48 tets each**, 1280×800:

| ponds | total tets | float FBO **ON** | float FBO **OFF** |
|---|---|---|---|
| 1 | 48 | 7.46 ms | 0.60 ms |
| 16 | 768 | 53.65 ms | 0.90 ms |
| 32 | 1,536 | **6960 ms** | 1.32 ms |

Sixteen 48-tet puddles cost more than the entire forest while drawing essentially nothing; 32 props is 655 MB of RGBA32F attachments on a 4 GB card already holding 799 MB, and it thrashes.

**Two mandatory consequences.** (a) `vtkOpenGLProjectedTetrahedraMapper::SafeDownCast(m)->SetUseFloatingPointFrameBufferOff()` is not an optimisation, it is a correctness requirement. (b) All water bodies go into **one** `vtkUnstructuredGrid` — PT needs no connectivity, and disjoint components are legal — so the per-prop tax is paid once.

**The confound, stated honestly.** The 3.9 ms fixed term in §1.2 was measured with the float FBO at its default (on). A second sweep with it off gives `≈ 0.44 µs/tet` and an intercept indistinguishable from zero (a two-endpoint fit even yields a physically impossible −0.81 ms, which is a fit artifact and must not be published as a cost). **Both models agree above ~50 k tets and disagree by up to 2× in the 1 k–12 k range — which is exactly the range any shippable W5 configuration occupies.** Section 13, D-W6: the sweep must be re-run with the FBO disabled before the `max_tets` default is trusted. Until then this document quotes W5 costs as a **range**.

### 1.4 What the surface costs

| measurement | result |
|---|---|
| marginal CPU per water actor, full water shader, 1280×800 | **28.2 µs/actor** (9-point least-squares, `ms == cpu_ms` at every N) — a second harness gives 5.1 µs; budget the conservative number, see §13 R2. *Measured on a translucent actor; the opaque actor W1 now uses pays the same per-prop traversal and skips the translucent sort, so 28.2 µs stays conservative.* |
| full water fragment stack (2 ripple octaves + derivative blend, chord, per-channel `exp`, 3-tap chromatic caustics, Schlick, sky, 2 glint lobes, foam) at **100 % screen coverage**, 1280×800 | **0.19–0.20 ms** (two independent harnesses agree) |
| the same at 1920×1200 full coverage | 0.46 ms → **0.20 ms/Mpx**, linear in pixels |
| triangle cost: 512 → **524,288** triangles, one actor, full coverage | **+0.20 ms total** (≈ 0.4 ns/triangle) |
| opaque capture pass, colour only (`vtkFramebufferPass`) | 0.11 ms (and 0.00 ms in a second harness) |
| opaque capture **with the mandatory depth restore** (`TextureDepthBlit`) | **0.27 ms** |
| hydrology bake, 512² (0.47 m cells), single-threaded | **56–60 ms** |

**The three conclusions that shape everything below:**

- Water is **actor-bound and fill-bound, never vertex-bound**. 1,024× the triangles costs +0.20 ms. Mesh finely, merge aggressively, **never decimate water for LOD**.
- The whole optical model is **0.20 ms/Mpx**, which is spent on the resource §1.1 proved we have spare.
- The only always-paid global cost anyone identified is the capture pass, and it is **0.27 ms**, not the 1.7 ms an unmeasured estimate suggested. **§5 removes even that from the default tier.**

---

## 2. Taxonomy — the bodies, and what each actually needs

| body | geometry | waves | thickness | absorption | notes |
|---|---|---|---|---|---|
| **sea** | **UNCHANGED**: near-field `VolumeNode` camera-following slab −40→+8 m, far-field flat mesh at z = 0 (Lab roadmap §4.6) | 4 crested travelling waves | n/a | volumetric, as today | out of scope; touched only by the ordering invariant in §5.6 |
| **pond** (< 20 m) | flat CDT lid + skirt | **none** | analytic | W1 | Carried by the Fresnel mirror and the tint. **Adding waves to a pond makes it worse.** This is the most common mistake in procedural water. |
| **lake** (> 100 m) | flat CDT lid + skirt, 2–3 m edges | 3 Gerstner, λ = 12/7/3 m, s = 0.10/0.06/0.04, Σs ≤ 1, **amplitude damped by baked `dist_shore`** | analytic | W1 | [Finch 2004]; ~20 ALU/vertex, analytic normals, zero CPU |
| **pool** | as pond | none | analytic | W1 | often `tannic_pond` optics |
| **stream / river** | swept ribbon on the bed, U across / V = arc length | **none** — orbital motion is not advective motion | analytic | W2 | two-phase flow map [Grimes 2011] with per-pixel noise jitter; without the jitter every pixel resets together and the river visibly pulses at 1/cycle Hz |
| **waterfall** | open vertical sheet ribbon | none | **constant × erosion α** | W3 | free-falling aerated water is not a Beer–Lambert medium and has air behind it. A constant uniform, never a thickness pass. |
| **spray / mist** | camera-facing billboards | — | — | soft fade | soft-depth fade needs the capture (W-T2); a constant fade until then |
| **seep** (< 100 m², < 0.15 m) | **none** | — | — | W0 | folded into the terrain splat as `mud` / `puddle` + a wetness multiplier |

**Two body types fall outside the "surface + column" model and are documented exceptions, not oversights:** the sea (already volumetric and staying that way) and the waterfall sheet (open, aerated, no bounded interior, no bed to measure against).

**On content.** The `lsystem_forest` control terrain is a smooth dome plus two sine octaves: verified **zero** depressions after priority-flood at 256²/512²/1024², and a maximum above-sea-level slope of **37°** — it cannot grow a lake or trip a 60° waterfall detector, and lake inventories measured on it were measured on augmented terrain. **The Lab's terrain is a different function**: 9-octave ridged multifractal with domain warp, smooth-max island folding, a peak Gaussian, a shelf, an erosion delta grid, and a `cliff_rock` class defined at slope > 42° (Lab roadmap §4.4, §7.4). It has depressions and it has cliffs by construction. **Water generation targets the Lab terrain, and `lsystem_forest` remains the untouched performance control** (Lab roadmap §1.3).

---

## 3. Generation from the heightfield

### 3.1 The pipeline, and what is already paid for

The Lab's erosion op (roadmap §4.4, Tier 3, per selected region) **already runs** priority-flood [Barnes 2014] and D8 + FD8 accumulation, then throws the intermediates away. Water's first and cheapest change is to **retain them**.

```
Lab erosion (EXISTS, ~0.55 s / 1024² region)   water-specific work (NEW)
  [1] priority-flood depression fill  ───────► RETAIN `filled` (w)
  [2] D8 + FD8 flow accumulation      ───────► RETAIN `accum`  (A)
  [3] implicit stream-power, m=.45 n=1        │
  [4] thermal slumping                        │
                                              ├─ [B] depression hierarchy (forest)
                                              ├─ [C] fill–spill–merge → level_z per body
                                              ├─ [D] area/depth filter → W0 vs real bodies
                                              ├─ [E] shoreline: marching squares on φ → rings → CDT
                                              ├─ [F] channels: threshold A → polylines, Strahler, Q, w/d/v
                                              ├─ [G] waterfalls: three detectors, unioned
                                              ├─ [H] carve (clamped inside lakes), re-run [1][2] ONCE
                                              └─ [I] class predicates + connectivity feed (§9)
```

Measured extraction cost, single-threaded: **60 ms at 512², 289 ms at 1024², 1.43 s at 2048².** At the Lab's 1024² erosion region that is **≈ +290 ms on a ~550 ms Tier-3 op — a 53 % increase on an operation that is already explicit and already user-initiated.** It is not a new subsystem; it is a subsystem that already exists, being asked to keep its output.

**Two hard ordering facts.** (i) Carving changes spill points — cutting a channel through a sill drains the lake behind it. Deterministic fix: **clamp the carved bed inside lake polygons to `≥ level_z − d_lake_bed`**, one pass, no iteration; assert `carve(carve(z,p),p) == carve(z,p)` [Génevaux 2013]'s `C` replace operator is idempotent, `min()` is not. (ii) **A lake is one node in the stream graph, not a flat region of it.** Depression-fill and then D8 draws a river straight across the flat lake surface along whatever path the flat-resolution code happened to pick. Collapse each lake's cell set to a single graph node, inlets = stream cells entering the shoreline, outlet = spill cell.

### 3.2 Resolution independence — the key architectural decision

The Lab's absolute rule (roadmap §2.1): *"the material/nav export always runs at full fidelity, from the analytic surface function, independent of any render rung."* Water honours it exactly:

> **`φ` is not a stored raster. `φ(x,y) = level_z − heightfield::sample(x,y)`, clipped to the shoreline polygon.**

Only *extraction* needs a raster. Once extracted, a water body is **vector data**: a scalar `level_z`, a set of shoreline rings, a centreline polyline with per-sample width/depth/speed. It re-evaluates exactly at any `grid_spec` — 0.5 m export, 2 m render cache, 8 m hydrology grid — with no resampling and no LOD coupling. This mirrors the roadmap's existing statement that rivers "lay down explicit paint at their true world width, which *is* resolution-independent."

**Consequence for the world-scale network:** priority-flood at 0.5 m over a 4096 m world is 8192² = 67 M cells ≈ 23 s. It is never run. Lake and channel *extraction* runs at the erosion region's resolution (1024², user-scoped); the world-wide channel skeleton uses the existing 8 m FD8 hydrology grid. Both produce vector entities.

### 3.3 Priority-Flood + ε — the determinism rules *are* the algorithm

```cpp
// Barnes, Lehman & Mulla 2014, improved variant (Alg. 3). O(m log m) float, O(n) integer.
struct pf_key {                    // TOTAL order. Not a strict weak order.
  std::int32_t z_mm; std::uint64_t seq; std::int32_t idx;
  bool operator<(const pf_key &o) const {          // std::priority_queue is a max-heap
    return z_mm != o.z_mm ? z_mm > o.z_mm : seq > o.seq;
  }
};
std::priority_queue<pf_key> open;   std::queue<std::int32_t> pit;   // plain-queue trick

// seed: every grid-edge cell, and every cell at or below sea_level connected to the edge
while (!open.empty() || !pit.empty()) {
  auto [z, i] = pit.empty() ? pop(open) : pop(pit);
  for (int k = 0; k < 8; ++k) {                    // FIXED rotation, ties by index k
    int j = nb(i, k);
    if (j < 0 || closed[j]) continue;
    closed[j] = 1;
    if (w[j] <= z) { w[j] = z + 1; pit.push(j); }  // +1 mm; O(1), no heap
    else            open.push({w[j], seq++, j});
  }
}
```

Four rules, in order of how often they bite:

1. **Total-order priority key `(elevation, monotonic insertion counter)`.** A bare min-heap on a float is a *strict weak order*: equal-elevation cells pop in whatever order that STL's sift produces, so **you get different lakes on different libstdc++ versions**. Barnes et al. state this explicitly for ε ≠ 0, watershed labelling and direct flow-direction determination. Highest-value single line in the module.
2. **Elevations as `int32` millimetres.** Makes priority-flood O(n) instead of O(n log n), makes every comparison exact, **kills the ε question outright** (a hand-picked float ε is a no-op if too small and turns the filled pit into a mesa if too large), and makes golden tests bit-reproducible across x86-64 / aarch64 / wasm. If float is retained, the ε must be `std::nextafterf(z, +INF)` with Barnes' `PitTop` guard.
3. **Fixed 8-neighbour rotation; `>` vs `>=` consistent across every code path.**
4. **Accumulation is a Kahn topological sweep, never recursion** — and its terminating assert is the cheapest cycle test that exists:

```cpp
for (std::size_t qi = 0; qi < q.size(); ++qi) {
  int r = recip[q[qi]];
  if (r >= 0) { A[r] += A[q[qi]]; if (--indeg[r] == 0) q.push_back(r); }
}
assert(q.size() == std::size_t(rows) * cols);   // fails iff routing produced a cycle
```

Float `+=` is not associative, so accumulation is serial with a fixed order, or fixed-point.

### 3.4 Depression hierarchy and Fill–Spill–Merge — the knob that makes this content

Plain priority-flood destroys the information we need: it fills every nested depression to the *highest* enclosing sill, so a chain of ponds becomes one implausible mega-lake. Build the depression hierarchy [Barnes 2020] — leaf depressions, meta-depressions, an ocean root, geolinks — then route a runoff volume through it and solve for the level in closed form with the **Lake-Level Equation** [Barnes 2021, Eq. 6]:

```
        Σ z_i·a_i  +  V_w
z_w =  ───────────────────      over the flooded cells c_1..c_N, sorted by ASCENDING z
             Σ a_i
```

found by walking the elevation-sorted cell list and stopping at the first `k` where the volume below `z_k` exceeds `V_w = runoff_depth_m × catchment_area`. Then `φ(c) = max(0, z_w − z(c))`.

Union-find **without union-by-rank** (rank relabels roots and destroys the tree) and **with** path compression; iterate the **sorted** outlet list, never the outlet hash map, keyed `(elevation, min_label, max_label)`.

**`runoff_depth_m` is the one authoring knob and it earns its place.** Deterministic sweep:

| runoff | result |
|---|---|
| → 0 | dry pans and playas; no lakes |
| 0.1 – 0.6 | small depressions full and spilling; **large ones partially filled with the surface *below* the sill** — real endorheic basins, seasonal lakes, salt flats |
| → ∞ | every depression at its sill; identical to naive filling (**measured**: converges at runoff ≥ 0.5 m on a test island) |

One float, fully deterministic, sweepable live, producing the variety every other terrain generator hand-places. It is also the reason a raw depression count in the hundreds becomes a lake count in the tens. It belongs in the Lab's World tab as a slider.

### 3.5 Shoreline extraction

Marching squares on `φ`, **restricted to the depression's DH label set** — the label is what stops the contour leaking into a neighbouring basin at the same elevation, which is the classic bug. Because `z_w` is a single scalar and `z` is a bilinear patch, the isoline is **exact** for the bilinear surface:

```
edge crossing:  t = φa / (φa − φb)                            # closed form, no search
saddle (cases 5, 10), asymptotic decider:
  φc = (φ00·φ11 − φ10·φ01) / (φ00 + φ11 − φ10 − φ01);  connect by sign(φc)
```

Segments therefore always chain into closed Jordan curves — which is what makes the invariants in §10 checkable on *random* heightfields. Chain → orient (outer CCW, island holes CW) → Douglas–Peucker at a fixed **world-metre** tolerance (0.25 × extraction cell; never screen-derived, or the mesh stops being a pure function) → constrained Delaunay with Steiner refinement to a 2–3 m target edge.

Measured: **1.2 ms at 512², 63 ms at 4096².** Free relative to everything else.

### 3.6 Channels

| quantity | relation | note |
|---|---|---|
| discharge | `Q = 0.42 · A^0.69` | **A in km², not m².** [Dunne & Leopold 1978] via [Génevaux 2013], which prints m²; at m² a 1 km² catchment reads 5800 m³/s. Pinned by test at `A = 1 → 0.42`, `100 → 10.0`, `10⁴ → 241 m³/s` (±1 %). |
| width | `w = a·Q^0.50`, a = 3.0 | ***Downstream*** Leopold–Maddock exponents (b, f, m) = (0.50, 0.40, 0.10). **Not** at-a-station (0.26, 0.40, 0.34). |
| depth | `d = c·Q^0.40`, c = 0.20 | |
| velocity | `v = k·Q^0.10`, k = 1/(a·c) = 1.667 | `b + f + m == 1` and `a·c·k == 1` are unit tests |
| composed | `w ∝ A^0.345` | **doubling river width takes 7.6× the catchment** |
| bed velocity | Manning `v = (1/n)·R^{2/3}·S^{1/2}`, R ≈ depth, n ≈ 0.035 | riffles fast, pools slow — reads as unmistakably correct |
| order | Horton–Strahler on the D8 tree | non-decreasing downstream (test) |

**Width is a polyline attribute, never a raster one.** At the Lab's 0.5 m export grid a channel 1.5 cells wide needs ≈ 376 km² of catchment; a 4 km world has ~16 km² total. Every stream is sub-cell at every usable resolution. Carry `w` on the polyline, render a ribbon, and carve a valley of `max(w, 2.5·cell_w)`.

Channel selection: `A > A_t` (practical band 100–2000 cells) for v1, with slope-area `A·S² > θ` [Montgomery & Dietrich 1992] as the better default — it puts channel heads lower on gentle slopes and higher on steep ones, producing variable drainage density that looks markedly better on mixed terrain. Centreline surface z is a **running min then smoothed**; skip it and water flows uphill, which is instantly visible.

### 3.7 Waterfall detection — three detectors, unioned

1. **Slope window on the polyline.** Over a window of `k·cell ≈ 2–3 cells`: `Δz > 2.0 m && Δz/Δs > fall_min_slope`. Default `fall_min_slope = tan 50° = 1.19` rather than the textbook 60°, because the Lab's cliff class begins at 42°. Merge adjacent detections; drop = summed Δz.
2. **Hanging junctions.** At each confluence, `bed_trib − bed_trunk > h_min`. Costs nothing (both beds are already computed), catches the glacial hanging-valley case, and **doubles as a correctness assert** — undetected, it is exactly what produces floating rivers.
3. **Lake spill points** whose downstream gradient exceeds the threshold. Free from the DH, with `Q` from FSM and a lip elevation of `z_w`.

Type from exactly two scalars — slope and flow — by nearest-seed lookup in a Voronoi over hand-picked exemplars [Emilien 2015]: contact types `stream → river` (↑flow), `rapid → block` (↑slope), `cascade`, `horsetail`; free-fall types `ribbon → plunge → cataract`, `ledge`. O(1), deterministic, artist-editable (move a seed, reclassify). **This is the only mechanism proposed anywhere that makes two waterfalls look different from each other**, and without it every fall in the world is the same scrolling sheet at a different width.

---

## 4. Representation — `cvc::world`

New headers in the namespace the Lab roadmap already claims (`inc/cvc/world/`, PR L1/L2): **`water.h`, `hydrology.h`, `water_mesh.h`** plus `src/cvc/world/{hydrology,depression,shoreline,channels,falls,water_mesh}.cpp`. **Pure CPU. No VTK. No GL. No `cvc::nav` link.** That last is not stylistic: the roadmap's file seam exists so that a change to the consumer's blur σ, EDT convention or channel order invalidates zero previously generated bundles. Water writes files; it does not call the consumer.

The coverage job builds with `CVC_BUILD_CVCGL=OFF`, so **anything load-bearing that lives in a shader is outside the 80 % gate forever.** Every correctness quantity therefore lives here.

```cpp
namespace cvc { namespace world {

// ── the medium. ONE struct, shared by every tier: W1's uniforms, W4's 3-D texture
// baseline, W5's vtkColorTransferFunction/vtkPiecewiseFunction, and W0's terrain
// tint all derive from it, so a body looks like itself whichever tier draws it.
struct water_optics {
  std::array<float,3> sigma { 0.55f, 0.16f, 0.11f };  // per-channel extinction, m^-1
  std::array<float,3> deep  { 0.075f, 0.294f, 0.271f}; // L_inf, linear RGB
  float f0 = 0.0204f;          // ((1-1.333)/(1+1.333))^2  [Schlick 1994]
  float turbidity = 0.0f;      // scales sigma, damps caustics
  float sigma_per_ntu = 1.8f;
  float ripple_metres = 0.03f; // world size of one ripple texel; 2-5 cm or it reads as a pool cover
  float glint_power = 480.0f;
  static water_optics clear_alpine();  // (0.42, 0.075, 0.045)
  static water_optics island_lake();   // (0.55, 0.16,  0.11 )
  static water_optics tannic_pond();   // (0.45, 0.80,  1.40 )  see note
  static water_optics silty_stream();  // (0.70, 0.45,  0.40 )
};
// NOTE on tannic: CDOM absorbs BLUE hardest, so sigma_B > sigma_G > sigma_R and the
// usual ordering INVERTS. That is why forest ponds are brown. Getting the inequality
// backwards yields murky teal and is an easy, invisible mistake.

enum class body_kind : std::uint8_t { seep, pond, lake, pool, stream, river, fall };
enum class render_tier : std::uint8_t { wet_only, surface, ribbon, sheet, march, tets };

struct ring { std::vector<std::array<double,2>> pts; bool outer = true; };

// ── lakes / ponds / pools ────────────────────────────────────────────────────
struct lake {
  std::uint32_t id = 0;  body_kind kind = body_kind::pond;
  float  level_z = 0.f;      // z_w. THE geometry: one double.
  float  spill_z = 0.f;      // sill.  INVARIANT: level_z <= spill_z
  float  max_depth = 0.f;  double area_m2 = 0, volume_m3 = 0;
  cvc::bounding_box bounds;
  std::uint32_t parent = kNone;  std::vector<std::uint32_t> children;  // DH forest
  std::vector<ring> shore;       // [0] outer CCW, rest island holes CW
  std::int64_t  spill_cell = -1;
  std::uint32_t outlet_channel = kNone;   // kNone == endorheic
  render_tier   tier = render_tier::surface;
  water_optics  optics;
};
// NOTE what is ABSENT: a per-lake depth raster. phi(x,y) = level_z - heightfield::sample(x,y),
// clipped to `shore`. Storing it would be 4 bytes x cells x bodies of redundancy AND
// would bind water to one resolution. See §3.2.

// ── streams / rivers ─────────────────────────────────────────────────────────
struct channel_sample {
  std::array<double,3> p;   // centreline; p[2] is the WATER SURFACE.
                            // INVARIANT: monotone non-increasing downstream.
  float bed_z = 0.f, arc_m = 0.f;
  float width = 0.f, depth = 0.f, speed = 0.f, discharge = 0.f;
  std::array<float,2> flow; // world-space 2-D flow dir; |flow| = speed
  std::uint8_t strahler = 1;
};
struct ford { float arc0 = 0, arc1 = 0, max_depth = 0; bool synthetic = false; };
struct channel {
  std::uint32_t id = 0;  body_kind kind = body_kind::stream;
  std::vector<channel_sample> samples;          // arc-ordered, downstream
  std::uint32_t from_lake = kNone, to_lake = kNone;   // lakes are NODES in this graph
  std::vector<ford> fords;                      // FIRST-CLASS, see §9.3
  float roughness_n = 0.035f;
  render_tier tier = render_tier::ribbon;
  water_optics optics;
};

// ── waterfalls — the one body outside the surface+column model ────────────────
struct fall {
  std::uint32_t id = 0, channel_id = kNone, basin_lake = kNone;
  std::array<double,3> lip{}, plunge{};
  float drop_m = 0, width_lip = 0, width_base = 0, discharge = 0;
  float spray_radius_m = 0;              // clamp(1.6*sqrt(drop)*Q^0.25, 2, 40)
  float sheet_thickness_m = 0.12f;       // A CONSTANT. Never a thickness pass. §5.5.
  enum class kind : std::uint8_t { ribbon, plunge, cataract, ledge, cascade, horsetail } type;
};

// ── the authoritative answer. Immutable after build(). ───────────────────────
class water_state {
public:
  const std::vector<lake>    &lakes()    const;
  const std::vector<channel> &channels() const;
  const std::vector<fall>    &falls()    const;

  // THE CONTRACT. Every tier and every consumer — render, nav, materials, audio,
  // physics — agrees on exactly these three, and none of them owns geometry.
  float surface_z(double x, double y) const;    // NaN if dry
  float depth    (double x, double y) const;    // phi; 0 if dry
  const water_optics &optics_at(double x, double y) const;

  std::uint64_t content_hash = 0;               // for golden tests
};

struct hydro_params {
  double runoff_depth_m   = 0.35;   // THE content knob, §3.4
  double min_lake_area_m2 = 100.0;  // below -> W0
  double min_lake_depth_m = 0.15;   // below -> W0
  double channel_threshold_cells = 400;
  double d_ford_m = 1.00;           // human wading limit
  double vd_safe_m2s = 0.50;        // depth x velocity, flood-engineering criterion
  int    ford_order = 2;            // Strahler <= this: depth clipped, fordable by design
  double fall_min_drop_m = 2.0, fall_min_slope = 1.19;   // tan 50 deg
};

water_state build_water(const heightfield &h,          // Lab roadmap §4.4
                        const std::vector<float> &filled,  // RETAINED from erosion [1]
                        const std::vector<float> &accum,   // RETAINED from erosion [2]
                        const grid_spec &extraction_grid,
                        const hydro_params &p);
}}  // namespace cvc::world
```

**Mesh companion** (`water_mesh.h`) — pure `cvc::geometry` producers, no GL. `cvc::geometry` already carries `tets_t` / `hexs_t` (`inc/cvc/geometry/geometry.h:84-85`, accessors `:182-188`) and per-vertex `functions_t`; **no change to `cvc::geometry` is required.**

```cpp
struct water_mesh_params {
  double lake_edge_m    = 2.5;   // CDT + Steiner target
  double skirt_drop_m   = 0.60;  // == the Lab's terrain skirt drop (§8.9). Not a coincidence.
  double simplify_tol_m = 0.60;  // WORLD metres. Never screen-derived.
  int    stream_cross_segs = 5;
};
// Vertex layout rides the arrays GeometryNode ALREADY uploads (normals, scalars/colour,
// TCoords). No vtkOpenGLPolyDataMapper::MapDataArrayToVertexAttribute — generic vertex
// attributes are the least-exercised corner of the GLES3 low-memory mapper path.
//   LAKE   : uv = (columnH_m/32, shoreDist_m/32)  colour = (waveDamp, lodFlag, spare)
//   STREAM : uv = (u across [-1,1], v = arc_m/8)  colour = (speed, depth, foam)
//   FALL   : uv = (u across, v = drop fraction)   colour = (aeration, 0, 0)
cvc::geometry build_lake_mesh  (const lake&,    const heightfield&, const water_mesh_params&);
cvc::geometry build_stream_mesh(const channel&, const heightfield&, const water_mesh_params&);
cvc::geometry build_fall_mesh  (const fall&,                        const water_mesh_params&);
cvc::geometry merge_water(std::span<const cvc::geometry>);   // -> ONE actor per pool slot

// W5 only. A basin is a swept PRISM COLUMN: unstructured in plan (the same CDT),
// structured in depth. Do NOT route this through LBIE tetrahedralize/hexahedralize --
// those are isosurface-driven octree meshers over a cvc::volume with no control over
// layer structure. Prism -> 3 tets, conforming: the diagonal of each quad side face
// passes through its SMALLEST GLOBAL VERTEX INDEX, which is consistent by construction
// because plan node ids are shared and the layer offset is a constant k*P.
cvc::geometry build_lake_tets(const lake&, const heightfield&, int nlayers, double edge_m);
cvc::geometry build_lake_hexs(const lake&, const heightfield&, int nlayers, double edge_m);
```

**`build_lake_mesh` and `build_lake_tets` are two views of one construction**, and `Σ tet volume == Σ prism volume == boundary-enclosed volume` is a unit test (§10). That makes "the renderer and the nav raster disagree about where the water is" structurally unrepresentable.

### 4.1 Why per-vertex `columnH` is baked and not depth-reconstructed

`columnH` (the vertical water column under a surface vertex) is baked at mesh-build time from the analytic heightfield and interpolated across the triangle. The alternative — reconstructing it per-fragment from a captured depth buffer — is used by most shipping water and **is not view-independent**, contrary to a widespread belief: for a fixed surface point, the ray through it strikes the bed at a *different* point as the camera moves, so `columnH` changes and the foam band breathes. A swimming shoreline is the single most recognisable tell that water is fake.

Baked `columnH` also makes the shoreline **immune to terrain LOD**: when a chunk switches rung and the bed moves by centimetres, the shoreline and foam do not move at all. And it removes the capture pass from the default tier entirely (§5.2).

**The split this document adopts:** baked `columnH` and `shoreDist` drive the shoreline, foam, alpha and wave damping; the depth buffer, *where a capture exists*, drives only screen-space refraction and the true-bed sample. Each is used where it is strong.

---

## 5. Rendering

### 5.1 What must be added to cvcGL — nothing invented, everything verified

| # | Addition | File | ~LoC | Tier | Verified anchor |
|---|---|---|---|---|---|
| 1 | `GeometryNode::setShaderUniform(name, float)` / `(name, const double v[3])` / `(name, const double m[16])` — passthrough to `vtkShaderProperty::GetFragmentCustomUniforms()->SetUniformf/3f/Matrix4x4` | `inc/cvc/gl/GeometryNode.h`, `src/cvcGL/GeometryNode.cpp` | 45 | **W1, required** | `vtkShaderProperty.h:86-87` `vtkGetObjectMacro(FragmentCustomUniforms, vtkUniforms)`; `vtkUniforms.h:85,89,91` |
| 1b | `GeometryNode::setVertexAttribute(attrName, arrayName)` — passthrough to `vtkPolyDataMapper::MapDataArrayToVertexAttribute`, so the baked `columnH`/`shoreDist` point arrays reach the vertex shader | `inc/cvc/gl/GeometryNode.h`, `src/cvcGL/GeometryNode.cpp` | 30 | **W1, required** | `vtkPolyDataMapper.h:173`; OpenGL impl `vtkOpenGLPolyDataMapper.h:143`; `RemoveVertexAttributeMapping` at `:157` |
| 1c | `GeometryNode::setNamedTexture(name, const cvc::image&)` — passthrough to `vtkProperty::SetTexture(name, tex)` so the terrain-albedo, caustic and cloud-shadow maps are reachable as same-named `sampler2D`s. Distinct from the existing single-texture `setTexture()` | `inc/cvc/gl/GeometryNode.h`, `src/cvcGL/GeometryNode.cpp` | 35 | **W1, required** | `vtkProperty.h:673` `SetTexture(const char*, vtkTexture*)`, `:740` `RemoveTexture`; existing texture plumbing at `src/cvcGL/GeometryNode.cpp:384` |
| 2 | `cvc::gl::WaterNode : public GraphicsNode` — prop is **one opaque `vtkActor`** over one `vtkPolyData` and one stock `vtkPolyDataMapper` (§5.2.2); `setWaterMesh()`, `setOptics()`, `setTier()` | new `inc/cvc/gl/WaterNode.h`, `src/cvcGL/WaterNode.cpp` | 240 | **W1, required** | `SceneNode::addToRenderer` is generic (`renderer->AddViewProp(getProp())`, `src/cvcGL/SceneNode.cpp:62-70`). **No `vtkPropAssembly`, no second actor, no mapper subclass, no render pass, no GL state manipulation** — the node only builds polydata, binds attributes/textures/uniforms and installs shader replacements |
| 2b | `StridedShadowBaker::Render` hides water actors around the bake — the opaque lid would otherwise cast a shadow ring outside every shoreline (§5.6). No new VTK surface; an edit inside a subclass cvcGL already owns | `src/cvcGL/SceneGraph.cpp` | 8 | **W1, required** | `vtkShadowMapBakerPass` has **no** exclusion key — `vtkShadowMapBakerPass.cxx:288-301` accepts every visible prop. The override already exists at `src/cvcGL/SceneGraph.cpp:916-947` |
| 3 | `SceneGraph::buildPassChain(bool shadows)` — always constructs the chain; only `{baker, shadowMap}` are conditional | `src/cvcGL/SceneGraph.cpp` | 25 | W-T2 prerequisite | **Today `setShadowsEnabled(false)` does `m_renderer->SetPass(nullptr)` at `:1034`, so with shadows off there is no chain to insert anything into.** The chain is built at `:1064-1068`. Shadows-off is a shipped, measured mode (21.7 fps). |
| 4 | `SceneGraph::addWater(name, const cvc::world::water_state&)` | `inc/cvc/gl/SceneGraph.h` | 60 | W1 | alongside the existing `addGraphics` overloads |
| 5 | `cvc::gl::SceneCapturePass : public vtkFramebufferPass` — **with a depth restore**, exposing `colorTexture()` / `depthTexture()` | new `inc/cvc/gl/SceneCapturePass.h`, `.cpp` | 180 | **W-T2, optional** | `vtkFramebufferPass::Render` blits `GL_COLOR_BUFFER_BIT` **only**; using it unmodified silently strips depth from the sea volume, the cloud slab and the overlay. Restore with `vtkOpenGLRenderWindow::TextureDepthBlit` (`vtkOpenGLRenderWindow.h:484,487,490`) — a shader-quad copy VTK added precisely because browsers restrict depth-format blits, so **one code path serves native and wasm.** |
| 6 | `cvc::gl::UnstructuredVolumeNode : public GraphicsNode` | new header + `.cpp` | 250 | **W5, optional** | §6 |

**Build system: no change.** `RenderingVolume`, `RenderingVolumeOpenGL2`, `RenderingOpenGL2` and `FiltersGeneral` are already in `CVCGL_VTK_COMPONENTS` (`src/cvcGL/CMakeLists.txt:24-32`), which covers `vtkFramebufferPass`, `vtkTextureObject`, `vtkProjectedTetrahedraMapper` and `vtkDataSetTriangleFilter`.

**Explicitly NOT added, each for a measured reason:** `vtkUnstructuredGridVolumeRayCastMapper` and `vtkUnstructuredGridVolumeZSweepMapper` (both pure CPU software renderers; ZSweep is single-threaded across all 4,322 of its lines, and both silently drop to 1/100 resolution under load via `ImageSampleDistance` auto-degrade); `vtkMultiBlockUnstructuredGridVolumeMapper` (a `std::vector` of PT mappers — it *multiplies* the per-prop tax of §1.3); `vtkOpenGLFluidMapper` (particle input, wrong shape — but it is the best in-tree reference for correct multi-pass FBO work, crib its setup); depth peeling (123 ms exact vs 5.5 ms weighted-blended on the same scene [McGuire & Bavoil 2013]; a volume needs hundreds of layers); any OIT scheme (WebGL2 has no ROV, no image load/store, no per-pixel linked lists); `vtkLODActor` / `vtkLODProp3D` (rejected by Lab roadmap §1.3, and would need `RenderingLOD` added to a file another PR owns); planar reflection (§12).

### 5.2 The default tier is **one opaque draw**: no blend trickery, no capture pass, no new render pass, no pass-chain surgery

#### 5.2.1 Why the two-draw composite was removed

An earlier revision of this document rendered the lid **twice** — draw A under `glBlendFunc(GL_ZERO, GL_SRC_COLOR)` to multiply the framebuffer by the chromatic transmittance `T`, draw B additively for the surface radiance. **That mechanism cannot be built on VTK 9.5 and has been deleted.** Four independent blockers, each verified against the shipped headers at `/home/joe/src/cvc/wt-volrover-perf/deps-live/include/vtk-9.5` (`vtkVersionMacros.h:` `VTK_VERSION "9.5.0"`):

| # | Blocker | Verification |
|---|---|---|
| 1 | **There is no per-actor, per-mapper or per-property blend-function API.** Nothing in VTK can ask for `(GL_ZERO, GL_SRC_COLOR)` on one actor. | `grep -rn "SetBlend"` over the whole 9.5 include tree matches only `vtkVolumeMapper.h`, `vtkUnstructuredGridVolumeMapper.h`, `vtkMultiBlockVolumeMapper.h`, `vtkMultiBlockUnstructuredGridVolumeMapper.h`, `vtkOpenGLSurfaceProbeVolumeMapper.h`, `vtkImageBlend.h`, `vtkImageSlabReslice.h` and two vendored `libharu` headers. **`vtkActor.h`, `vtkProperty.h`, `vtkMapper.h`, `vtkPolyDataMapper.h` and `vtkOpenGLPolyDataMapper.h` contain no blend API at all.** `vtkTexture::BlendingMode` (`vtkTexture.h:187-204`) is fixed-function *texture-environment* combining, not framebuffer blending, and is unrelated. |
| 2 | **Translucent actors do not write depth.** The intended A-then-B ordering never establishes itself. | `vtkOpenGLActor.cxx:88` — `ostate->vtkglDepthMask(GL_FALSE); // transparency with alpha blending`, restored to `GL_TRUE` after `mapper->Render` at `:98`. The opaque branch instead sets `GL_TRUE` explicitly at `:52-56`. There *is* a per-actor escape hatch — `vtkOpenGLActor::GLDepthMaskOverride()` (`vtkOpenGLActor.h:52`), an information key honoured at `vtkOpenGLActor.cxx:70-85` — so blocker 2 alone is repairable. It does not rescue blockers 1 or 3. |
| 3 | **The translucent path defaults to order-independent transparency**, which reorders and reweights fragments and destroys any two-draw composite. Disabling it is **renderer-global**. | `vtkRenderer.h:930-932` (`vtkSetMacro(UseOIT, bool)`), `:1168` `bool UseOIT = true;`. `SetUseOITOff()` is on `vtkRenderer`, so it silently changes every other translucent prop in the scene — the sea slab, the cloud slab, mist cards, captions. |
| 4 | **Behaviour is path-dependent on an unrelated user toggle.** | `src/cvcGL/SceneGraph.cpp:1059` installs an explicit `vtkTranslucentPass` when shadows are **on**; with shadows **off**, `:1034` does `m_renderer->SetPass(nullptr)` and the stock OIT path runs instead. So the same water would composite one way with shadows on and another way with shadows off. |

Blockers 1 and 3 are the fatal pair, and blocker 1 generalises into a constraint worth stating once, because it decides the whole section:

> **The framebuffer-attenuation theorem.** VTK's translucent blend is `(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)`. The only quantity a fragment can scale the destination by is the **scalar** `1 − a`. Chromatic Beer–Lambert needs a **per-channel** factor `T = exp(−σ∘L)` with `T.r ≠ T.g ≠ T.b`. Therefore **per-channel absorption of already-rendered scene colour is impossible without either blend-function control (absent, blocker 1) or a framebuffer read (a capture pass).**

There is no third option. So W1 must choose which of the two properties to keep:

- keep the **real rendered bed**, give up chromatic absorption → grey-scale water. Unacceptable; depth-tinting *is* the feature.
- keep **chromatic absorption**, give up reading the framebuffer → **evaluate the bed in the water's own fragment shader**. This is what W1 now does.

#### 5.2.2 The mechanism: one opaque actor, bed evaluated in-shader

**W1 is a single `vtkActor` with `GetProperty()->SetOpacity(1.0)`, drawn on the stock opaque path.** Every VTK facility it uses is a shipped public API:

| Need | API | Anchor |
|---|---|---|
| inject the water fragment stack | `vtkShaderProperty::AddFragmentShaderReplacement` / `AddVertexShaderReplacement` | `vtkShaderProperty.h:101-107`; already wrapped as `GeometryNode::addFragmentShaderReplacement` (`src/cvcGL/GeometryNode.cpp:599-607`) and **already shipping** in `src/cvcGL/examples/lsystem_forest.cpp:158-163` |
| per-vertex baked `columnH`, `shoreDist` | `MapDataArrayToVertexAttribute(attrName, arrayName, fieldAssoc, comp)` | `vtkPolyDataMapper.h:173`; OpenGL impl `vtkOpenGLPolyDataMapper.h:143` |
| optics, time, sun, camera scalars | `vtkShaderProperty::GetFragmentCustomUniforms()` → `vtkUniforms::SetUniformf/3f/Matrix4x4` | `vtkShaderProperty.h:86-87`; `vtkUniforms.h:84-113` |
| bed albedo / normal / caustic / cloud-shadow maps | `vtkProperty::SetTexture(const char* name, vtkTexture*)` — the named texture is reachable from the replaced fragment source as a `sampler2D` of the same name | `vtkProperty.h:673`, `:740` `RemoveTexture` |

The optical path length stays exactly as §4.1 derived it — analytic, from the baked column, no depth read:

```
pathLen = columnH / max(-ray.z, 0.08)
```

exact for a locally flat bed, and at a 2.5 m triangle the bed *is* locally flat. `P_bed = wPos + ray*pathLen` is analytic too. The single change is that the shader now also produces `C_bed` itself instead of inheriting it from the framebuffer:

```
C_bed = albedo(P_bed) ∘ ( sunColor · max(N_bed·L, 0) · shadowTerm(P_bed) + skyAmbient )
```

and the final, **opaque** fragment is the whole composite in one write:

```
C = C_bed ∘ T ∘ (1−F)  +  L∞ ∘ (1−T) ∘ (1−F)  +  R_sky·F  +  glint  +  foam
```

which is **algebraically identical to what the two-draw scheme was trying to produce**, with `C_bed` supplied by evaluation rather than by the blender. Per-channel absorption is fully preserved, because `T` is a `vec3` multiplied inside the shader where no blend hardware is involved.

`albedo(P_bed)` is the Lab's existing terrain splat, bound as a named texture — the same 256×256-per-tile albedo bake the Lab already produces for its terrain shading, so W1 adds a texture *binding*, not a texture *bake*. `shadowTerm` is the Lab's projected cloud-shadow texture, already required by the caustics term in §5.3.

#### 5.2.3 What this buys, immediately and structurally

- **No blend-function control needed** — blocker 1 does not apply to an opaque draw.
- **Depth is written normally.** `vtkOpenGLActor.cxx:52-56` takes the `opaque` branch and sets `vtkglDepthMask(GL_TRUE)` explicitly, so the sea and cloud volumes downstream are correctly occluded by a lake surface — the §5.6 guarantee is now *structural* rather than something a translucent draw had to be argued into.
- **OIT is irrelevant.** No global `SetUseOITOff()`, so no side-effects on the sea slab, cloud slab, mist cards or captions. **The open question about global OIT is closed, not deferred.**
- **No path-dependence on shadows.** Both `SceneGraph` configurations render opaque props identically, and the shader itself is bit-identical in both — see the ordering note below.
- **No depth-peeling guard, and no `vtkPropAssembly`.** One actor, one mapper, one polydata. Risk **R1** is deleted rather than mitigated.
- **Refraction, soft-depth mist and true-bed caustics remain exactly the W-T2 refinement they already were** (PR W7), and the capture pass's scope is unchanged. W-T2 now *additionally* upgrades `C_bed` from evaluated to real — see §5.2.5.

**Fill cost.** The single draw does all the work the pair used to do, plus one albedo fetch and one lighting evaluation for the bed, and minus one full rasterisation of the lid. Net **≈ 0.26 ms per fully-covered megapixel**, down from the two-draw 0.40. Geometry halves outright (one rasterisation, [M3]'s 0.4 ns/triangle). The revised budgets are in §7.1.

#### 5.2.4 The one ordering rule that replaces the old guard

The water fragment stack is injected at `//VTK::Light::Impl` with `ReplaceFirst = true` — which is what `GeometryNode::addFragmentShaderReplacement` already passes (`src/cvcGL/GeometryNode.cpp:603`). This matters and must not be changed casually:

`vtkOpenGLPolyDataMapper::BuildShaders` applies **user `ReplaceFirst` replacements before `ReplaceShaderValues`**, and the non-first ones after it. `vtkShadowMapPass` injects its shadow-factor code into `//VTK::Light::Dec` and `//VTK::Light::Impl` from inside `ReplaceShaderValues` (`vtkShadowMapPass.cxx:311-313`, re-emitting the anchor at `:370` and `:450`). Because water consumes the anchor **first**, it removes VTK's default lighting block *and* the shadow pass's injection together — so **the water program is byte-identical whether shadows are on or off.** That is the property the old design could not have, and it is why `cvcgl_water_shader` must assert the compiled source hash matches across both `SceneGraph` configurations.

The consequence is stated plainly, not hidden: **VTK's shadow map never darkens the water, and the water shader cannot sample it.** The bed's tree-shadow term is therefore *not* available in W1 (§5.2.5, con 2, and D-W8).

#### 5.2.5 What W1 gives up, honestly

1. **Submerged non-bed geometry is hidden.** An opaque lid occludes anything between the surface and the bed — a fish, a submerged boulder actor, a sunken prop. The bed itself (terrain) is fine, because it is what the shader evaluates. For the island fly-through the demo actually renders there is no such geometry, but this is a real restriction and it is the strongest argument for D-W8 option B.
2. **Tree and terrain shadows do not tint the bed through the water.** §5.2.4 explains why. The cloud-shadow term *is* available and is applied. This is a genuine regression against the two-draw design's intent, and it is the second argument for D-W8 option B.
3. **The bed is the analytic heightfield, not the rendered terrain.** Bed lighting is re-evaluated rather than reused, so a change to terrain shading must be mirrored in the water shader or the two will drift at the shoreline. Bounded by sharing one GLSL include, and asserted by a shoreline-continuity golden.
4. **Screen-space refraction and soft-depth mist** are absent, exactly as before — W-T2, PR W7.

All four are repaired by the same thing: the **W-T2 capture pass**, which supplies the real, lit, shadowed, cloud-shaded framebuffer colour and depth as textures. With a capture in hand the shader reads `C_bed` instead of evaluating it, and cons 1–3 disappear together. **The capture pass is not new work invented to patch this section** — it is addition #5, already specified, already budgeted at 180 LoC, and already scheduled as PR W7. The mechanism change simply gives it three more reasons to exist. See **D-W8** for whether it should be pulled forward into W1.

### 5.3 The W1 / W2 fragment stack

Vertex side (requires `GeometryNode::disableCoordinateShiftScale()` so `vertexMC` is world space — the same thing the existing terrain bump map relies on):

```glsl
//VTK::Normal::Dec      ->  "out vec3 wPos; out vec2 wAttr; out vec3 wCol;"
//VTK::PositionVC::Impl ->  "wPos = vertexMC.xyz;"      // + uv/colour passthrough
```

Fragment, injected at `//VTK::Light::Impl`. Note `addFragmentShaderReplacement` consumes the anchor, which removes VTK's default lighting block — intended, we own the final colour. **GLES3 constraint inherited from Lab roadmap §8.11: never assign `normalVCVSOutput` directly; use the existing `CVC_FS_NORMAL` macro pattern, or the whole program fails to link under Emscripten and the actor is invisible.**

```glsl
float columnH   = max(wAttr.x, 0.0);      // BAKED. view-independent. §4.1
float shoreM    = wAttr.y;
vec3  V   = normalize(uCamPosW - wPos), ray = -V;

// 1. normal: two octaves, DERIVATIVE blend. NEVER normalize(n0+n1) -- that flattens.
vec2 d0 = n0.xy / max(n0.z, 1e-3);
vec2 d1 = n1.xy / max(n1.z, 1e-3);
vec3 N  = normalize(vec3(d0 + d1, 1.0));
// (W2: each octave is the two-phase flow-map blend of §5.4 instead of a plain scroll)
// Flatten toward +Z by SHORE PROXIMITY and by DISTANCE. Shore-flattening stops ripples
// in 2 cm of water; distance-flattening is the single most effective anti-aliasing
// measure water has -- unflattened high-frequency normals at range shimmer, and that
// glitter-noise is what reads as "cheap".
N = normalize(mix(vec3(0,0,1), N,
      clamp(columnH/0.8, 0.0, 1.0) * clamp(40.0/max(length(uCamPosW-wPos),1.0), 0.0, 1.0)));

// 2. THE CHORD. Analytic. No depth read.
float pathLen = columnH / max(-ray.z, 0.08);
float sunPath = columnH / max(uSunDirW.z, 0.15);          // downward solar path
vec3  sig = uSigma * (1.0 + uSigmaPerNtu * uTurbidity);
vec3  T   = exp(-sig * (pathLen + sunPath));              // Beer-Lambert, per channel
                                                          // == PT's own optical model

// 3. Fresnel  [Schlick 1994], F0 = 0.0204 for eta = 1.333
float F = uF0 + (1.0 - uF0) * pow(1.0 - max(dot(N, V), 0.0), 5.0);

// 4. THE BED, evaluated -- NOT read from the framebuffer (§5.2.1 theorem).
// Analytic bed point; albedo from the Lab's terrain splat bound as a named sampler2D
// (vtkProperty::SetTexture, vtkProperty.h:673); bed normal from the same heightfield
// gradient the terrain shader uses, so the two agree across the shoreline.
vec3  pbed  = wPos + ray * pathLen;
vec3  Nbed  = bedNormal(pbed.xy);                          // shared GLSL include
vec3  Cbed  = texture(uTerrainAlbedo, pbed.xy * uSplatScale).rgb
            * (uSunColor * max(dot(Nbed, uSunDirW), 0.0) * cloudShadow(pbed.xy)
               + uSkyAmbient);
// caustics land on the same analytic bed point -- zero extra pass, chromatically
// split, depth-faded. Folded into the bed radiance, before absorption.
Cbed += causticRGB(pbed.xy * uCausticScale, uTime)
        * cloudShadow(pbed.xy) * exp(-sig.g * columnH) * uCausticK;

// 5. ONE OPAQUE COMPOSITE. Chromatic absorption is a vec3 multiply in-shader:
// no blend hardware is involved, so nothing here depends on VTK's blend func,
// on the depth mask, on OIT, or on whether shadows are enabled.
vec3 col = Cbed * T * (1.0 - F);                           // absorbed bed
col += uDeep * (1.0 - T) * (1.0 - F);                      // in-scattered body colour
col += skyProbe(reflect(-V, N)) * F;                       // gradient probe, §12
vec3 H = normalize(uSunDirW + V);                          // two lobes: a single tight
col += uSunColor * F * (pow(max(dot(N,H),0.0), uGlintPow)        // one crawls and
                      + 0.08*pow(max(dot(N,H),0.0), 20.0));      // flickers alone
// shore foam: animated noise THRESHOLDED against the depth ramp. Never the raw ramp,
// which is a contour line and swims.
float shoreR = 1.0 - clamp(columnH / uFoamWidth, 0.0, 1.0);
col = mix(col, uFoamColor,
          step(noise(wPos.xy*2.2 + uTime*0.10), pow(shoreR, 2.0)) * 0.65);
gl_FragData[0] = vec4(col, 1.0);                           // alpha 1.0: OPAQUE actor
```

**Shoreline feathering without alpha.** An opaque actor cannot fade out, and it does not need to: the composite *self-feathers*. As `columnH → 0`, `pathLen → 0` and `T → 1`, so the in-scattered term vanishes and `col → Cbed·(1−F) + R_sky·F`, i.e. the water converges to **the very terrain colour it is drawn over**. The transition is continuous by construction rather than by blending, which is strictly better: it is immune to draw order, to OIT, and to LOD.

The one term that does *not* self-cancel is Fresnel — at grazing angles `F` stays large and would leave a bright rim at the waterline — so `F`, the glint lobes and the sky probe are each scaled by the same `saturate(columnH/k)` ramp already used to flatten the normal. With that, the edge is invisible. Combined with the **geometric skirt** — the boundary ring extended outward and dropped 0.60 m below the terrain, the same drop the Lab's terrain chunks use — **there is no water silhouette against terrain at any LOD**.

This is why the shoreline-continuity golden of §5.2.5 con 3 is load-bearing: it is what detects the water shader's `Cbed` drifting away from the terrain shader's own output.

### 5.4 Streams (W2)

Same core, with the two-phase flow-map advection [Grimes 2011] replacing each normal octave's scroll:

```glsl
vec2  flow = texture(flowTex, uvFlow).rg * 2.0 - 1.0;   // low spatial frequency
float jit  = texture(noiseTex, uvNoise).r;              // PER-PIXEL. Load-bearing.
float t = uTime/uCycle + jit, p0 = fract(t), p1 = fract(t + 0.5);
vec3 n = mix(texture(nrm, uv*s + flow*p0).xyz,
             texture(nrm, uv*s + flow*p1).xyz, abs(0.5 - p0)/0.5);
```

Each layer's weight is exactly zero at the instant its UV offset resets, so the pop is invisible. **Without the per-pixel jitter every pixel resets simultaneously and the whole river pulses at 1/cycle Hz** — the single most recognisable flow-map artifact.

Flow direction is `−normalize(∇H)` from a Sobel, blended toward the channel direction inside the river mask; raw D8 quantises to 8 directions and looks blocky. Speed from Manning modulates foam intensity and chop amplitude. Lakes get `flow ≈ 0` plus a ~0.02 wind drift so still water is not dead.

Foam: speed `saturate((|flow| − v0)·k)` for riffles, plus convergence `saturate(−div(flow))` — foam piles where flow converges, against rocks and in eddies. Cheap and disproportionately convincing.

### 5.5 Waterfalls (W3)

Five stacked layers; **nothing here uses depth absorption**, because an aerated free-falling sheet is not a Beer–Lambert medium and has air behind it.

1. **Sheet ribbon** down the fall face, V running downward, two layers of one texture at different scroll rates.
2. **Gravity-correct scroll.** Water accelerates, `v = √(2gh)`:
   `uv.y -= uTime * speed * (0.3 + 0.7 * uv.y);`
   **This one line is the difference between a waterfall and a conveyor belt.** Highest-value change in the section.
3. **Lateral wobble and break-up.** `uv.x += sin(uv.y*f + t*s)*a` in fragment space; then erode alpha downward — `alpha *= smoothstep(thr, thr+0.2, noise(...))` with `thr = mix(0.15, 0.65, uv.y)` — so the sheet fragments into strands lower down.
4. **Crest.** Bend the ribbon over the lip with a short blend, thin it, add a tight whitewater band. The lip is where a fall reads as fake.
5. **Impact.** Billboards at the base (soft-depth faded once W-T2 lands; constant fade before), 2–3 large scrolling additive noise cards, and — **the cheapest and most effective element** — a **wetness decal** on the terrain multiplying albedo by ~0.6 and raising smoothness. It sells the impact more than the mist does. [Emilien 2015]'s overhang displacement (`+λu` at the lip, `−λu` at the pool border) produces the characteristic flipped-S undercut and is worth the bake-time cost.

**Never volumetrics for spray.** Mist cards are heavy overdraw; render them at half resolution if the budget bites.

### 5.6 Composition and ordering

- **Water vs opaque terrain:** free, and now genuinely trivial — water is an opaque prop in the same opaque pass, so the depth buffer resolves it with no ordering assumption whatsoever.
- **Water vs the sea volume and the cloud slab:** VTK does **not** depth-sort volumes against each other or against translucent props — `vtkRenderer.cxx:686-689` walks `PropArray` in insertion order. The opaque water draw writes depth unconditionally (`vtkOpenGLActor.cxx:52-56` sets `vtkglDepthMask(GL_TRUE)` on the opaque branch), so volumes rendered after water are correctly clipped. Registration order is **sea → water → clouds**, and it is asserted.
- **Water vs OIT and depth peeling:** no interaction at all. Water is not a translucent prop, so `vtkRenderer::UseOIT` (`vtkRenderer.h:1168`, default true) and the depth-peeling flags are free to keep their defaults. **No global renderer setting is changed by this design.**
- **The sea-boundary invariant.** The Lab's near-field sea slab spans −40 → +8 m. A body whose `level_z ≤ sea_level + wave_amp` is **merged into the sea and is not an above-sea-level body**; asserted at build. A river delta is a W2 ribbon whose alpha fades to zero over its last few metres inside the sea polygon.
- **Water on water** (a stream entering a lake, a fall into its pool) is order-dependent and blends wrong at grazing angles. Mitigation: fade the tributary's alpha to zero over its last few metres inside the receiving polygon — a per-vertex bake, free, and it works. It is still a fudge and §12 says so.
- **Shadows — and one new obligation the opaque draw creates.** Making water opaque puts it into the shadow bake, which the translucent design got for free. **`vtkShadowMapBakerPass` has no exclusion key**: `vtkShadowMapBakerPass.cxx:287-301` walks `renderer->GetViewProps()` and accepts *every* prop with `GetVisibility()` true. An unexcluded lid would cast a shadow through its own skirt onto the terrain just outside the waterline — a dark ring at every shore.
  The lever is already in the tree: cvcGL **already subclasses the baker** as `StridedShadowBaker` with a `Render(const vtkRenderState*)` override (`src/cvcGL/SceneGraph.cpp:916-947`). Water actors are hidden around the `Superclass::Render(s)` call and restored after — about 8 lines inside a class that exists, needing no new VTK surface. This is safe against that override's own caster-count guard, which counts *lights* (`countShadowCasters`, `:956`), not props. `cvcgl_water_depth` asserts a lake casts no shadow ring.
  Water is also never a shadow **receiver**, and this is now structural rather than chosen: consuming `//VTK::Light::Impl` with `ReplaceFirst = true` removes `vtkShadowMapPass`'s injection along with VTK's default lighting (§5.2.4). The benefit is that the water program is identical shadows-on and shadows-off; **the cost is that the bed's tree-shadow term is unavailable in W1**, which is §5.2.5 con 2 and the substance of D-W8. The cloud-shadow term *is* applied to the bed, so large-scale light variation still reads correctly across a lake.

---

## 6. The volumetric tier — `UnstructuredVolumeNode`

### 6.1 When tets are genuinely justified

**All four conditions must hold** [Max 1995; Silva 2005]:

1. `τ`, scattering albedo or emission **actually varies inside**, on scales the camera resolves.
2. That variation is **what the viewer is meant to read** — the audience is interrogating structure, not admiring water.
3. The field **lives natively on that unstructured mesh and must not be resampled** — the mesh's adaptive refinement *is* the information (boundary layers, adaptive ocean cells [Morrical 2020]). If the field would survive resampling to a 3-D texture, **use the 3-D texture (W4)**: trivially sorted, trivially marched, works everywhere.
4. The mesh is **static or slowly time-varying** — both PT and connectivity-based ray casting must redo preprocessing on any geometry or topology change [Silva 2005 §5].

For procedurally generated island water, (1) and (2) are content decisions we control and **(3) is never true — there is no solver mesh, there is a heightfield.** Condition (3) is the only one that could make tets right here, and it structurally cannot occur for water. It occurs constantly for FEM/CFD, which is why the node ships.

### 6.2 The node

```cpp
// inc/cvc/gl/UnstructuredVolumeNode.h
class UnstructuredVolumeNode : public GraphicsNode {
public:
  void setGeometry(const cvc::geometry &g);   // tets_t; hexs_t via §6.3
  void setTransferFunction(vtkColorTransferFunction*, vtkPiecewiseFunction*);
  void setScalarOpacityUnitDistance(double);  // the physical sigma length scale
  vtkProp *getProp() override;                // a vtkVolume; AddViewProp handles it
  // applyClipPlanes() MUST no-op: vtkUnstructuredGridVolumeMapper has no
  // SetClippingPlanes. Every shading knob in VolumeNode.h:64-100 is ALSO a no-op here.
private:
  vtkSmartPointer<vtkVolume> m_prop;
  vtkSmartPointer<vtkProjectedTetrahedraMapper> m_mapper;   // factory -> the GL subclass
  vtkSmartPointer<vtkUnstructuredGrid> m_grid;              // ONE grid, ALL bodies
};
```

Construction rules, every one of them measured or read from source:

```cpp
m_mapper = vtkProjectedTetrahedraMapper::New();
vtkOpenGLProjectedTetrahedraMapper::SafeDownCast(m_mapper)
    ->SetUseFloatingPointFrameBufferOff();     // MANDATORY. §1.3. 53.65 ms -> 0.90 ms.
m_mapper->SetInputData(m_grid);                // one merged grid: disjoint components legal
```

- **Hexes never go in raw.** `vtkOpenGLProjectedTetrahedraMapper.cxx:337-348` warns `"Encountered non-tetrahedra cell!"`, sets `GaveError` and skips the cell — a hex mesh renders *partially and wrongly* rather than failing loudly, which is a debugging trap. Route `hexs_t` through `vtkDataSetTriangleFilter` with `TetrahedraOnlyOn()`: **measured multiplier exactly 6.000×** (400 hexes → 2400 tets), point count unchanged. `build_lake_hexs` therefore exists for FEM export, not for PT.
- **Scalar capacity:** one component → colour TF + opacity TF (the normal path). **Multi-component *independent* is unimplemented** — `MapIndependentComponents` says verbatim that it punts and copies the first scalar. **Two components *dependent*** → component 0 to RGB, component 1 to opacity via separate TFs; **this is the useful one** (turbidity → colour, depth → opacity). Four dependent → raw per-vertex RGBA, bake anything. So: **two TF-mapped fields, or four raw channels.** That is PT's one genuine advantage over a surface, and it is the whole justification.
- **Never joins `vtkMultiVolume`** (that needs `vtkGPUVolumeRayCastMapper`); `enableMultiVolumeRendering` must be off when W5 is on.
- **Hybrid mode** (recommended when W5 is used for water at all): draw the tets for the *interior* with no depth write, and the W1 boundary surface for the *skin* (Fresnel, glint, ripples, foam). This is the only configuration in which PT produces something that reads as water rather than jelly. Cost is additive.
- **`CachedCellCenterDepthSort`** (~60 LoC, caches the permutation when the camera matrix and input MTime are unchanged) is worth writing **only** because it makes the *paused inspection* case free. It buys 15–20 % and changes no decision.

### 6.3 The measured scaling limit, and the boundary of the technique

| tets | props | PT ms (FBO off) | vs 21 ms Lab frame | vs 40.32 ms control |
|---|---|---|---|---|
| 2,400 | 1 | 1.06 – 4.9 | 5 – 23 % | 2.6 – 12 % |
| **4,800** | 1 | **2.1 – 6.0** | **10 – 29 %** | **5 – 15 %** |
| 12,000 | 1 | 4.5 – 9.1 | 21 – 43 % | 11 – 23 % |
| 48,600 | 1 | 17.6 – 21.5 | 84 – 102 % | 44 – 53 % |
| 198,744 | 1 | 84.1 | 400 % | 209 % |

(Range = the §1.3 confound: lower bound from the FBO-off sweep, upper bound from the as-measured fit `3.9 + 0.000429·n`.)

**Shipped defaults: `max_tets = 4800`, `max_tet_bodies = 1`, native only, `--water-tier=volume` opt-in, and the node refuses rather than degrades above the cap.**

**Where the technique stops, stated as an engineering boundary rather than a benchmark:**

- **At ~5 k tets** in a live 45 fps frame. Not because 5 k is special, but because 0.43 µs/tet × the frame you can afford says so.
- **At any camera motion near the water**, at any tet count, because of the `||` near-plane cull.
- **At any non-convex domain** where the crude cell-centre ordering is wrong, and at any domain with sorting cycles where a correct ordering *does not exist* [Comba 1999; Williams 1992].
- **At the wasm boundary, absolutely.** Feature-wise PT would run: the shaders are 28-line/27-line and ES3-clean, `glDrawRangeElements` is core ES 3.0, the wasm deps prefix already ships `libvtkRenderingVolumeOpenGL2-9.5.a`, and the depth blit already has a WebGL2 fallback. **The blocker is call volume.** PT issues one VBO upload + one IBO upload + one draw per sort partition, and random-pivot partitions average ~500 cells against the 1000 cap, so chunks ≈ 2·⌈N/1000⌉. Measured: 12,000 tets → 24 chunks (~95 WebGL calls/frame); **101,568 tets → 202 chunks, ~810 JS-boundary crossings and ~24 MB of buffer re-upload per frame.** Raw wasm compute is *not* the problem (the CPU slice measures only 1.10–1.36× native). **W5 is compiled out of the wasm build at the CMake level.**
- **Beyond a workstation GPU**, if you wanted the modern answer instead. Interactive unstructured DVR at scale in 2026 means hardware BVH traversal — [Morrical 2020] gets 36 M wedges from 2.49 to 37 fps on RTX RT cores, 2–3 orders over ParaView's PT path, at **6.5 GB peak GPU memory**. That is the price of admission and it is not our target platform.

### 6.4 W4 — the tier that replaces the tets' usual justification

> A closed watertight surface plus a **low-resolution 3-D texture** (or a depth-parameterised 1-D/2-D turbidity profile) sampled along the ray **between the front surface and the analytic bed**, ray-marched at 16–64 steps in the fragment shader.

Genuinely heterogeneous absorption and scattering with **no sort, no connectivity, no preprocessing, no k-buffer, no OIT**. It composites correctly by construction — one ray, front-to-back, within one fragment. It runs in WebGL2 today. It degrades to pure Beer–Lambert when the field is uniform. It occupies the entire useful design space between W1 and W5. PROJECTED cost: ~0.3 ms of fill for a 64³ field marched 32 steps over 25 % coverage; measure in PR W9.

**W4 survives the §5.2 mechanism change, but the texture binding is not the obvious one, and a reviewer was right to flag it.**

*What is closed:* `vtkProperty::SetTexture(name, vtkTexture*)` **cannot deliver a `sampler3D`.** `vtkOpenGLTexture::Load` only ever calls `CreateDepthFromRaw` (`vtkOpenGLTexture.cxx:306`) or `Create2DFromRaw` (`:317`); there is no 3-D branch anywhere in the class. So the addition 1c path of §5.2.2 is 2-D only, and a 3-D σ field cannot ride it.

*What is open, and is the route W4 takes:* `vtkTextureObject` **does** support 3-D — `Create3D` and `Create3DFromRaw` (`vtkTextureObject.h:265`, `:272`) — together with `Activate()` (`:149`) and `GetTextureUnit()` (`:135`). The missing piece is a hook that runs per-frame with the live shader program in hand so the texture can be activated and its sampler uniform pointed at the right unit. That hook exists and needs **no subclass**:

> `vtkOpenGLPolyDataMapper::UpdateShaders` fires `this->InvokeEvent(vtkCommand::UpdateShaderEvent, cellBO.Program)` (`vtkOpenGLPolyDataMapper.cxx:2907`), after all of VTK's own uniform setting. An observer on the mapper receives the `vtkShaderProgram*` as call data and may set anything it likes.

This is a supported, in-tree pattern, not a discovered trick: **`vtkOpenGLSkybox` uses exactly it** — `mapper->AddObserver(vtkCommand::UpdateShaderEvent, this, &vtkOpenGLSkybox::UpdateUniforms)` (`vtkOpenGLSkybox.cxx:87`), whose callback then calls `program->SetUniform3f(...)`, `SetUniformMatrix(...)` and friends (`:90-108`).

So W4 is: keep the stock `vtkPolyDataMapper` and the stock opaque actor of W1; add a small `WaterVolumeBinder : vtkCommand` that owns a `vtkTextureObject` built with `Create3DFromRaw`, and on each `UpdateShaderEvent` calls `tex->Activate()` then `program->SetUniformi("uSigmaField", tex->GetTextureUnit())`. The march itself is fragment code inside the same replacement block W1 already installs, bounded by the analytic `pathLen` W1 already computes.

**W4 therefore becomes ~70 LoC of C++ plus the march GLSL — no mapper subclass, no render pass, and no change to the W1 actor.** It remains a strict superset of W1, and the claim that "the 3-D texture route is closed on the polydata path" is true only of `vtkTexture`, not of `vtkTextureObject`. The negative verdict on tetrahedra (§1.2, §6.3) is untouched by any of this.

*wasm note:* WebGL2 has `TEXTURE_3D` in core, so the binder works under Emscripten unchanged; the field is dropped to 32³ there for heap reasons (§7.3).

---

## 7. Cost budgets

### 7.1 Per frame, native

Against **both** references: the Lab's own budget (roadmap §11.1: ≈ 21 ms total, target ≥ 45 fps) and the `lsystem_forest` control (40.32 ms, 24.8 fps).

| item | CPU ms | GPU/fill ms | basis |
|---|---|---|---|
| 3 merged water actors (lakes+ribbons / falls+mist / spare), **one draw each** | **0.085** | ~0 | 3 × 28.2 µs × 1 draw, measured per-actor tax |
| W1/W2 fragment stack at a realistic **25 %** coverage, 1280×800, single draw | ~0.01 | **0.067** | 0.256 Mpx × 0.26 ms/Mpx (§5.2.3) |
| triangles (whole world's water ≪ 100 k tris) | 0 | ~0.02 | 0.4 ns/tri, measured; halved — one rasterisation |
| wave animation | **0** | < 0.05 | Gerstner is a *vertex* shader; **there is no `updateVertices` for water, ever** |
| flow-map animation | **0** | 0 | one `uTime` uniform |
| W3 spray, ~200 soft billboards | ~0.02 | ~0.08 | PROJECTED |
| uniform updates | ~0.003 | — | |
| **W0–W3 total** | **≈ 0.12** | **≈ 0.22** | **0.34 ms** |
| | | | **= 1.6 % of the Lab's 21 ms · 0.8 % of the 40.32 ms control** |

Worst realistic case — camera at the water's edge, water covering **80 %** of a 1920×1200 frame: `0.085 + 1.84 Mpx × 0.26 = 0.56 ms`, still **2.7 %** of the Lab budget.

**The single-draw mechanism is cheaper than the two-draw one it replaces on every line** (0.34 ms vs 0.47 ms typical; 0.56 ms vs 0.91 ms worst case). It was adopted for correctness — the two-draw scheme cannot be built at all (§5.2.1) — but it happens also to be the faster design, because the bed evaluation it adds costs far less than the whole second rasterisation it removes.

Optional tiers, added on top:

| tier | CPU | GPU | total | % of 21 ms |
|---|---|---|---|---|
| W-T2 opaque capture (refraction, true-bed caustics, soft mist) | 0.05 | 0.22 | **0.27** | +1.3 % |
| W4 bounded march, one body | ~0 | ~0.30 | **0.30** PROJECTED | +1.4 % |
| **W5 tets, 4,800, one grid, FBO off** | **2.1 – 6.0** | ~0 | **2.1 – 6.0** | **+10 – 29 %** |

**Prop budget.** The Lab's `lab.lod.max_props = 48` is explicitly a placeholder to be set by `cvcgl_prop_sweep` in PR L3 (roadmap §8.7). Water claims **3 slots**, merged per LOD band, and its per-actor cost must be re-checked against that sweep's curve rather than against this document's 28.2 µs.

### 7.2 Generation

| operation | native | wasm | Lab tier |
|---|---|---|---|
| water extraction, 512² (0.47 m) | **60 ms** | ~72 ms | 3 |
| water extraction, 1024² erosion region | **289 ms** | ~350 ms | **3, folded into the existing 0.55 s erosion op** |
| shoreline + CDT + skirt + ribbon sweep, ~20 bodies | ~25 ms PROJECTED | ~70 ms | 3 |
| φ / v / class re-evaluation at a 513² export window (analytic, §3.2) | ~4 ms PROJECTED | ~12 ms | 1b |
| water mesh rebuild, one body | < 1 ms | < 3 ms | 2 |

Water adds ≈ 53 % to a Tier-3 erosion operation that is already explicit and user-initiated. It adds **nothing** to Tier 0/1/2.

### 7.3 wasm parity

The wasm build is single-threaded by policy (`CMakeLists.txt:101`) and heap-capped.

| capability | native | WebGL2 | note |
|---|---|---|---|
| shader replacements on an **opaque** actor | ✅ | ✅ | **already proven live** — the terrain bump map at `transfix.github.io/libcvc` runs this exact path, on an opaque actor |
| `vtkShaderProperty` custom uniforms | ✅ | ✅ | plain uniforms, no textures |
| `MapDataArrayToVertexAttribute` (baked `columnH`) | ✅ | ✅ | a plain `in` attribute; ES 3.0 core |
| named `sampler2D` via `vtkProperty::SetTexture` (terrain albedo for the bed) | ✅ | ✅ | 2-D only — see the W4 row and §6.4 |
| `vtkTextureObject::Create3DFromRaw` + `UpdateShaderEvent` binder (**W4 only**) | ✅ | ✅ | `TEXTURE_3D` is WebGL2 core; field dropped to **32³** under Emscripten for heap |
| `exp(vec3)`, `pow`, `reflect`, `dFdx/dFdy`, `fract`, constant-bound loops | ✅ | ✅ | ES 3.0 core |
| *(was: `blendFuncSeparate` + depth-mask toggles for the two-draw composite)* | — | — | **No longer required by any tier.** W1/W2/W3 touch no blend state and no depth mask, so the GLSL is byte-identical across backends modulo `CVC_FS_NORMAL`, and the portability question this row used to carry is gone (§5.2) |
| Gerstner in the vertex shader | 3 waves | 2 waves | ~20 ALU/vertex |
| flow maps, two-phase + jitter | ✅ | ✅ | |
| W-T2 capture | ✅ | ⚠️ | depth restore via `TextureDepthBlit` (one code path); colour at **half res**; never sample a texture bound to the current FBO (`INVALID_OPERATION`), and `copyTexImage2D` is forbidden for `DEPTH_COMPONENT` |
| caustics | 3-tap chromatic | 1-tap mono, nearest bodies only | |
| mist billboards | 24 | **8** | overdraw is the only genuinely reduced item |
| foam accumulation ping-pong | ✅ | cut | falls back to the noise-threshold band |
| **W5 tets** | ✅ capped | ❌ **compiled out** | §6.3 |
| planar reflection | ❌ | ❌ | §12 |

**The one trap, already documented in-tree:** `//VTK::Normal::Impl` writes `normalVCVSOutput` on desktop and `normalizedNormalVCVSOutput` under Emscripten; assigning the wrong one is an ESSL *"can't modify an input"* error that fails the whole program and makes the actor invisible. Use `CVC_FS_NORMAL` and register a `cvcgl_water_shader` compile-and-probe test on **both** backends, modelled on the Lab's `cvcgl_sway_shader`.

---

## 8. LOD, and interaction with terrain LOD

### 8.1 Water LOD is a shader-flag problem, not a geometry problem

[M3] settles it: 512 → 524,288 triangles costs +0.20 ms. **Re-meshing water for LOD buys nothing.** The tier is fixed at bake time; only a per-frame `uint32 lod_flags` uniform varies, so nothing can pop and nothing needs to cross-fade.

| rung | trigger | shader | mesh |
|---|---|---|---|
| **L0** | screen coverage > 2 % | 2 ripple octaves, caustics, refraction (T2), foam, both glint lobes, flow map | unchanged |
| **L1** | 0.4 – 2 % | 1 octave, mono caustics, one glint lobe | unchanged |
| **L2** | < 0.4 % | plane normal (fully flattened), Beer–Lambert + Fresnel only, refraction **off** | unchanged |
| **L3** | body screen bbox < 32 px, or > ~1.2 km | **no actor** — folded into the terrain tile's splat texture as `water_shallow` / `water_deep` albedo | **none** |

Bodies merge into **3 actors** by band. Cross-band transitions use the Lab's existing anti-popping rule: hashed-alpha dithered cross-fade [Wyman & McGuire 2017] at `//VTK::Color::Impl` (the *colour* anchor — the normal anchor is the GLES3 trap), over `lab.lod.fade_tau_s` expressed as a time constant `1 − exp(−dt/τ)` off the world clock, never a frame ratio.

**This is unaffected by — and in fact suits — the opaque draw of §5.2.** Hashed-alpha is a *stochastic discard*, not a blend: it exists precisely so that geometry can cross-fade while staying on the opaque path with depth writes intact. The L3 rung folds the actor away entirely, so no band transition ever needs true transparency.

**W5 has no LOD, and that is a first-class disqualifier, not a gap.** Measured: 48,600 tets cost 18.60 ms at full frame and 18.73 ms with the body shrunk to 6 % of its linear size. The only possible W5 "LOD" is swapping a prebuilt tet mesh, i.e. rebuilding the merged `vtkUnstructuredGrid` — ~4 ms of allocation at 37 k tets, which must be amortised or it is a hitch.

### 8.2 Terrain LOD — three rules that eliminate the crack class

1. **The shoreline is derived from the extraction grid and the analytic heightfield, never from the render mesh.** A terrain rung change cannot move a shoreline. (§3.2)
2. **`level_z` is one constant per body**, so no vertical seam can open and no T-junction can crack.
3. **The skirt.** The boundary ring is extended outward and dropped **0.60 m** — deliberately the same drop the Lab's terrain chunks use — and alpha does the visual work. A terrain LOD pop of up to 0.60 m in the shore band is invisible because **there is no water silhouette against terrain, ever.** The same skirt is the fix for the hard shoreline intersection line; one mechanism, two problems.

**One coupling, stated as a constraint on the other system:** the terrain LOD selector must clamp its vertical error to `skirt_drop_m` inside any water body's bbox. That is a one-line addition to `cvc::lod::select`, and it is the only place water touches terrain LOD.

4. **Baked `columnH` is immune** (§4.1): when a chunk switches rung and the bed moves by centimetres, the shoreline and foam band do not move at all.
5. **Actors are pooled and recycled** (`setWaterMesh` into an existing node), never added or removed, which keeps the shadow-baker caster set constant.

---

## 9. Material and navigation export

### 9.1 One grid, one registry, no new ids

The material grid **is** the hydrology evaluation grid, because §3.2 makes `φ` analytic — it is evaluated fresh at whatever `grid_spec` the export asks for (default 513 × 513 spanning exactly 256.0 m, `cell_w = 0.5`, row 0 = `min_y`). Nothing is resampled, so nothing can drift, and the roadmap's rule that "the export always runs at full fidelity from the analytic surface function, independent of any render rung" holds by construction.

**Water uses the 32 shipped classes and adds none.** The registry (Lab roadmap §16.2) already contains everything needed, and its ids are contractually never renumbered:

| condition (`φ` = depth m, `v` = speed m/s) | class | id | ρ | hard |
|---|---|---|---|---|
| `φ > d_ford` (1.0 m) | `water_deep` | **16** | **0.00** | **yes** |
| `0 < φ ≤ d_ford` **and** `φ·v ≥ 0.5 m²/s` | `water_deep` | **16** | **0.00** | **yes** |
| `0 < φ ≤ d_ford` **and** `φ·v < 0.5` and `φ > 0.35` | `water_shallow` | **15** | 0.90 | no |
| `0 < φ ≤ 0.35` (ford, puddle) | `puddle` | **14** | 0.85 | no |
| `−0.35 < φ ≤ 0`, slope < 4°, high deposition | `mud` | **13** | 0.80 | no |
| channel margin, high stream power `A^0.5·S` | `gravel` | **4** | 0.12 | no |
| shoreline beach band | `sand` | **5** | 0.18 | no |
| inside a fall's `spray_radius_m` | `bare_rock` | **9** | 0.30 | no |

**Every `hard` class carries ρ = 0.00**, per the roadmap's revision-2 invariant `registry[k].hard ⇒ registry[k].rho == 0.0`, asserted for every ontology variant. Giving a hard class ρ = 1.00 double-counts against the consumer's A\* `hard_penalty` and its `φ_m` barrier and — because `risk_raw` is blurred by the consumer — actively poisons the free space next to every hazard out to `blur_bleed_radius_m` (8.0 m outdoors). Maximum `risk_raw` under `merged_default` stays **0.90** (`water_shallow`).

**`φ·v`, not depth alone, is the fordability gate.** It is the flood-engineering pedestrian criterion (< 0.5 m²/s safe for adults, < 1.0 marginal) and it costs nothing, because `v` is already on the grid from Manning or `v = k·Q^0.10`. Depth alone would call a 40 cm mountain torrent fordable; it is not. **Swift shallow water therefore maps to `water_deep`** — the registry class name describes *unfordable water*, and reusing it avoids a schema bump. Whether a distinct `torrent` id is worth a version bump is **decision D-W4** (§13).

Spray does **not** get a class of its own: it is `bare_rock` plus a render-only wetness multiplier (albedo × 0.6, smoothness up). A wet rock is a rock.

### 9.2 Where water sits in the three-layer priority stack

The Lab's stack is `derived (0) → grammar paint (1) → authored (2)`, highest wins, with `layer_owner` recording which. Water **extends layer 0's predicate table** and does not add a layer:

```
# hydrology predicates evaluated FIRST within layer 0
phi > d_ford  ||  (phi > 0 && phi*v >= vd_safe)   -> water_deep
phi > 0.35                                        -> water_shallow
phi > 0.0                                         -> puddle
phi > -0.35 && slope < 4 && deposition > t        -> mud
# then the existing h/slope/twi table, unchanged:
h < -0.5    -> water_deep      # the SEA. unchanged.
h < 0.0     -> water_shallow
abs(h) < 3  -> sand
...
```

Grammar paint and authored strokes still override, so an author can paint a dry lakebed and be obeyed. `layer_owner` gains a `derived_hydrology` value so the Lab can answer "why is this cell water?".

### 9.3 The CONNECTED-TERRAIN invariant

**Fixed decision: the terrain is connected throughout — no disconnected islands, land bridges and isthmuses join everything, and ground navigation is always solvable.** Water must never sever it.

**The gate already exists and water feeds it rather than duplicating it.** `validate_outdoor(raster_out&, grid_spec&, archipelago_spec&, inflate_m)` in `inc/cvc/world/cells.h` (Lab roadmap §7.8) rasterises at the export `grid_spec`, computes `free = !occupancy && !hard`, inflates obstacles by 6.0 m to match `planner.far_pair_in_free_space` exactly, labels **4-connected** components, and applies a policy with a `repair → resample → loud hard-fail` ladder. Cost: one EDT + one labelling pass over 513² ≈ **3 ms**, and it runs on every exported window, every time.

Water's contribution is four mechanisms, in priority order:

**(1) Lakes cannot sever anything.** A lake is a closed depression; its shore is a Jordan curve in the interior; you can always walk around it. **Only channels can cut**, because a channel runs continuously from a divide to the sea. That makes the whole problem cheap — the gate only ever has to inspect channel reaches.

**(2) Fordability by construction.** Any reach with `strahler ≤ ford_order` (default 2) has its depth clipped so `φ ≤ d_ford` everywhere. This is physically honest — order-1/2 headwaters genuinely are ankle-deep — and it means the **majority of the network is traversable by design**, not by repair. Independently, `w = 3.0·Q^0.5` on a 4 km world with catchments of order 0.02 km² gives `Q ≈ 0.06 m³/s` and `w ≈ 0.73 m`: sub-cell. Streams here are largely incapable of severing anything.

**(3) Explicit fords at any remaining cut.** Deterministic, auditable, and it is a **terrain edit, not a raster lie** — so the render and the nav raster are re-derived from the same edited heightfield and agree by construction, and the ford is visibly a shallow gravel crossing:

```
comp = 4-connected components of the inflated-free set          # validate_outdoor
while |comp| > 1 and attempts < 3:
    for each adjacent pair separated by a channel:
        # total order, no ties: min (phi*v), then min phi, then min cell index
        c* = argmin over the shared boundary
        raise the BED over a 4 m band about c* until phi <= 0.35 m
        reclassify the band to puddle + gravel margins
        record channel::fords{arc0, arc1, synthetic = true}
    re-run validate_outdoor
```

This reuses the existing repair primitive verbatim — the roadmap's `forced-bridges` policy already raises "the narrowest disqualifying saddle to +0.6 m over a 3-cell feather and paints it sand/gravel". A ford is that primitive applied to a channel instead of a channel-free saddle.

**(4) Bridges where a ford is geologically implausible** — a drop > 1.5 m within the band, i.e. a gorge. Emit a `deadfall`/log span as a scene prop, `occupancy`-free and walkable, falling back to (3) if the span exceeds 12 m.

**Policy.** The fixed decision maps to the roadmap's **`single-island`** policy: pass requires `largest_component_fraction ≥ 0.98` **and** `channels_crossed == 0`. Failure after the ladder is a **loud hard-fail with the seed recorded**, never a silent pass. Synthetic fords are flagged so a designer can see what the repair did.

**Measured feasibility of the gate**, on a real test heightfield at 512² / 0.47 m, 4-connected traversability:

| runoff (m) | lake % | deep % | ford % | components | largest / traversable |
|---|---|---|---|---|---|
| 0.10 | 9.79 | 0.07 | 9.72 | **1** | **100.00 %** |
| 0.25 | 11.20 | 0.38 | 10.82 | **1** | **100.00 %** |
| 0.50 | 11.53 | 0.56 | 10.97 | 2 | **100.00 %** |
| ∞ | 11.53 | 0.56 | 10.97 | 2 | **100.00 %** |

Stress test with **fords disabled entirely** (`d_ford_m = 0`, all water hard): worst case **8 components, 99.89 % largest, 0.11 % orphaned across 7 small pockets.** The invariant is cheap to hold and the repair ladder is bounded. Pockets below `min_pocket_m2` that remain isolated are reclassified to `mud` rather than water — bounded by that measured 0.11 %.

### 9.4 The shoreline as a material boundary — three layers, not one

1. **Material classes are *bands of the signed field* `φ`, not per-cell raster predicates.** The boundaries are then analytic surfaces in a continuous field, stable under `level_z` perturbation, and they resample correctly at any grid. Shader-side, blend with `smoothstep` over a band of width `≈ cell_w·|∇z|`, so a gentle beach gets a wide mud band and a cliff shore a hard edge — automatically and for free.
2. **The hard traversability classification is scanline-filled from the extracted *polygon*** (nonzero winding), not thresholded from the field, so **the nav boundary and the rendered shoreline come from the same marching-squares output and cannot disagree.** That disagreement is precisely the bug class this avoids.
3. **Gate:** `polygon-rasterised water set == field-thresholded water set (±1 cell band)`.

### 9.5 Export

The bundle contract is **files, not a C++ call.** `cvc::world` never links `cvc::nav`; the consumer derives everything. Contract arrays keep their shape and semantics; water appends optional planes:

```
water_depth.npy   float32 (rows, cols)   phi in METRES, signed (negative = dry)
flow_speed.npy    float32 (rows, cols)   Manning v, m/s (0 on dry land)
ford.npy          uint8   (rows, cols)   1 = carved ford or bridge span
hydrology.json    lakes[{id, level_z, area, volume, rings}],
                  channels[{id, strahler, samples[], fords[]}],
                  falls[{id, drop, Q, type, spray_radius}]
manifest.json     "hydrology": { runoff_depth_m, d_ford_m, vd_safe_m2s, ford_order,
                                 extraction_cell_w, connectivity_policy,
                                 connectivity_components }
```

GRL-SNAM needs none of it — `risk_raw` and `hard` already carry the whole story — but a scientist asking why a route detoured wants `water_depth` and `flow_speed` in front of them.

---

## 10. Testing and determinism

The coverage job builds with `CVC_BUILD_CVCGL=OFF`, so **everything below lives in `src/cvc/world` and is inside the 80 % gate.** Library tests register through `src/cvc/tests/CMakeLists.txt`; cvcGL tests append at **EOF of `src/cvcGL/CMakeLists.txt`** (the count of `add_test(NAME cvcgl_` is the authority, not this document).

Because the pipeline is deterministic by construction and the shoreline chains into Jordan curves by proof, **every invariant below is checkable on randomised heightfields, not just goldens** — copy the shape of `r-barnes/Barnes2020-FillSpillMerge`'s property suite (97 % coverage, 214,990 assertions).

```
FLOW
  every cell has a flow direction after priority-flood
  following it reaches OCEAN in <= n steps
  indeg == 0 everywhere after accumulation             # cheapest cycle check that exists
  accum monotone non-decreasing downstream
LAKES
  Sum_{c in L} (z_w - z_c)*a_c == V_w   (+/- 1e-6 * V_w)        volume conservation
  level_z <= spill_z;  z_c <= level_z for every c in L
  runoff -> inf  =>  extents converge to naive fill    (MEASURED: converges at >= 0.5 m)
  runoff == 0    =>  no lakes                          (MEASURED)
  GRID CONVERGENCE: gate on total_lake_area, deep_fraction, largest_lake_area.
    DO NOT gate on lake COUNT -- refinement keeps finding sub-cell pits, so raw
    count is NOT convergent (measured 319 / 740 / 877 / 909 at 128/256/512/1024).
    A filtered count near the area threshold is a flake generator.
SHORELINE
  every segment has exactly one predecessor and one successor
  every ring closed; signed area of outer rings > 0, island rings < 0
  shoreline subset of the DH label set;  shoreline intersect (phi > 0) == empty
MESH
  every internal triangular face of tets() shared by exactly 2 tets   (conformity)
  Sum(tet volume) == Sum(prism volume) == volume enclosed by boundary()  (1e-9 rel)
  no negative Jacobians;  boundary() watertight (every edge used twice)
NETWORK
  forest, no cycles;  Strahler non-decreasing;  width non-decreasing downstream
  channel surface p[2] monotone NON-INCREASING downstream
  Q(A): 1 km^2 -> 0.42, 100 -> 10.0, 1e4 -> 241 m^3/s   (+/- 1%)     THE UNIT TRAP
  b + f + m == 1  and  a * c * k == 1                               continuity
CARVING
  carve(carve(z,p),p) == carve(z,p)                    C-operator idempotence
  z_carved(c) >= level_z(L) - d_bed for every c in lake L
NAV / MATERIAL
  validate_outdoor(...).components == 1 after the repair ladder,
    over the full matrix runoff in {0, .05, .1, .25, .5, 1.0, inf}
                     x d_ford in {0.0, 0.5, 1.0}                    HARD CI GATE
  polygon-rasterised water set == field-thresholded water set (+/- 1 cell)
  registry[k].hard => registry[k].rho == 0.0 for every ontology variant
  max(risk_raw) <= 0.90 under merged_default
TIER AGREEMENT (the substitutability contract)
  for every tier t in {W0..W5}, for 4096 sample points:
      make_water_node(state, body, t)->surface_z(x,y) == state.surface_z(x,y)   (1e-6)
DETERMINISM
  identical lakes across 3 STL builds (the total-order PQ)
  accum bit-identical across -j1 and -j8
  hierarchy identical over 100 shuffles of the outlet map
  an upstream terrain edit leaves downstream meander phases unchanged
     (hash(seed, stable_id, index), never a sequential PRNG stream)
RENDER (cvcGL, Xvfb + llvmpipe)
  cvcgl_water_shader   -- W1/W2/W3 compile and link on BOTH backends (the
                          CVC_FS_NORMAL trap), modelled on cvcgl_sway_shader
  cvcgl_water_depth    -- sea volume pixels are non-background with water present
                          (the depth-peeling / volume-ordering regression)
  cvcgl_water_budget   -- 3 actors x 1 draw, 25% coverage <= 0.45 ms, measured
```

---

## 11. Implementation plan

Slots after the Lab's `L2` (which ships terrain, erosion, hydrology retention and `validate_outdoor`) and `L3` (which ships `lsystem_lab` and `cvcgl_prop_sweep`). **`src/cvcGL/examples/lsystem_forest.cpp` is never touched** — it stays the fast smoke test and the performance control (Lab roadmap §1.3). All demos are `lsystem_lab` flags.

| PR | Title | Scope | New files | Shared files | Demoable outcome | Gate | Depends |
|---|---|---|---|---|---|---|---|
| **W1** | `cvc::world` hydrology core | retain `filled`/`accum`; total-order Priority-Flood+ε; Kahn accumulation; depression hierarchy; Fill–Spill–Merge + LLE; area/depth filter | `inc/cvc/world/{water,hydrology}.h` + srcs; `world_hydrology_test.cpp` | `src/cvc/world/world.cmake`, `src/cvc/tests/CMakeLists.txt` (+1 line each) | `cvc-worldgen build --hydro --runoff 0.25` prints the lake inventory and writes `phi_preview.png`; sweep the slider from dry pans to full lakes | all FLOW/LAKES invariants; ≤ 60 ms at 512²; ≥ 80 % lines | L2 |
| **W2** | shoreline + channels + falls | marching squares w/ asymptotic decider, DP simplify, CDT + Steiner; D8 channel extraction, Strahler, Q/w/d/v, Manning; three fall detectors + Emilien typing; carve with lake clamp | `src/cvc/world/{shoreline,channels,falls}.cpp`; `world_shoreline_test.cpp`, `world_channels_test.cpp` | none | `cvc-worldgen build --hydro --preview` draws lakes, a stream network and fall markers as a PNG | SHORELINE + NETWORK + CARVING invariants; Q unit trap pinned | W1 |
| **W3** | `water_mesh` + the `GeometryNode` passthroughs | lake lid + skirt, ribbon sweep, fall sheet, merge; the uniform passthrough (#1), the vertex-attribute passthrough (#1b) and the named-texture passthrough (#1c) — 110 lines total | `inc/cvc/world/water_mesh.h` + src; `world_water_mesh_test.cpp` | `inc/cvc/gl/GeometryNode.h`, `src/cvcGL/GeometryNode.cpp` (2 files, warm) | `cvc-worldgen mesh --out lake.off` — open it in VolRover3 | MESH invariants (watertight, conforming, volume agreement) | W2 |
| **W4** | **`WaterNode` + the W1 tier — the first "wow" frame** | **one opaque actor, one stock mapper, one polydata**; the W1 shader with the in-shader bed (§5.2.2); `SceneGraph::addWater`; the `StridedShadowBaker` water-hiding fix (#2b) | `inc/cvc/gl/WaterNode.h`, `src/cvcGL/WaterNode.cpp`; `src/cvcGL/test/cvcgl_water_shader.cpp`, `cvcgl_water_depth.cpp` | `inc/cvc/gl/SceneGraph.h`, `src/cvcGL/SceneGraph.cpp`; `src/cvcGL/CMakeLists.txt` **EOF append** | **`lsystem_lab --water`: depth-tinted lakes with foam shorelines and correct terrain occlusion, identical with shadows on and off** | ≤ 0.030 ms/actor; ≤ 0.28 ms/Mpx at 1 draw; scene fps loss ≤ 3 %; **shader source hash identical shadows-on vs shadows-off**; no shadow ring at the shoreline | W3, L3 |
| **W5** | **streams and falls** — W2/W3 tiers | two-phase flow map with per-pixel jitter; gravity-scaled scroll; erosion alpha; mist billboards; **wetness decal**; W0 fold into the terrain splat | `water_flow.glsl`, `waterfall.glsl` in `src/cvcGL/water_shader.cpp` | same EOF block | **`lsystem_lab --water --rivers`: a stream from a ridge, over a fall, into a lake, into the sea.** *The island's real hydrology is a radial drainage fan — this, not the lakes, is the shot the terrain actually produces.* | no reset pulse (per-pixel jitter golden); ≤ 0.10 ms for 6 falls | W4 |
| **W6** | material + nav export + **the connectivity gate** | layer-0 hydrology predicates; `φ·v` classification onto the shipped registry; ford carving; `.npy`/`hydrology.json` export; feed `validate_outdoor` | `src/cvc/world/water_materials.cpp`, `water_connect.cpp`; `world_water_nav_test.cpp` | none | `nav_river_ford` — a swarm routes around a lake and *through* a ford; the deep reach is refused | **`components == 1` over the full runoff × ford matrix**; polygon-vs-field ±1 cell; hard ⇒ ρ = 0 | W5 |
| **W7** | **W-T2 capture** — refraction, true-bed caustics, soft mist | `buildPassChain(bool shadows)` (the `SetPass(nullptr)` fix); `SceneCapturePass` with `TextureDepthBlit`; refraction with the mandatory rejection test and the `columnH` clamp | `inc/cvc/gl/SceneCapturePass.h`, `.cpp` | `src/cvcGL/SceneGraph.cpp` | shallow water refracts a submerged boulder; mist fades softly against the cliff | ≤ 0.30 ms at 1280×800; **shadows-OFF path still renders volumes** (the regression this PR could cause) | W5 |
| **W8** | wasm parity + LOD ladder | `lod_flags`, actor merging, L3 fold into the tile splat, extension probes with graceful degrade, half-res capture | `src/cvc/world/water_lod.cpp` | `src/cvcGL/examples/CMakeLists.txt`, `deploy-pages.yml` | lakes and a river live at `transfix.github.io/libcvc/lsystem_lab/` | shader compiles on both backends; wasm fps ≥ 0.9× the no-water wasm baseline | W7 |
| **W9** | **W4 bounded heterogeneous march** | 3-D σ field via `vtkTextureObject::Create3DFromRaw` bound by a ~70-line `WaterVolumeBinder : vtkCommand` on `vtkCommand::UpdateShaderEvent` (§6.4); 32-step march between surface and analytic bed; turbidity authoring | `water_march.glsl`, `water_volume_binder.cpp`; `world_turbidity_test.cpp` | none | a sediment plume in a pond, and a thermocline in a deep lake | ≤ 0.4 ms fill; degrades to W1 when the field is uniform; **no mapper subclass and no render pass** | W8 |
| **W10** | **W5 `UnstructuredVolumeNode`** — the scivis capability | PT node; **`SetUseFloatingPointFrameBufferOff()`**; one merged grid; 2-component dependent TF; `CachedCellCenterDepthSort`; `max_tets` cap that refuses; hybrid PT-interior + W1-skin; `applyClipPlanes()` no-op; **compiled out of wasm at the CMake level** | `inc/cvc/gl/UnstructuredVolumeNode.h`, `.cpp`; `src/cvcGL/test/cvcgl_unstructured_volume.cpp` | same EOF block | **`water_column`** — an FEM/CFD tet field with a transfer function, paused camera, ~100 k tets at ~20 fps. **The deliverable that outlives this demo.** | refuses > `max_tets`; **the §1.3 FBO-off re-measurement lands here (D-W6)** | W9 |

**Ordering:** W1 → W2 → W3 → W4 → W5 → W6 → W7 → W8 → (W9 ∥ W10).

W1–W6 is a complete, shippable water system: generation, rendering, nav. W7–W10 are quality and capability, each independently revertible. **W10 can slip indefinitely without blocking anything**, which is exactly the right relationship to have with the tier that answers the user's question in the negative.

**Note on demo ordering.** W4 lands lakes before W5 lands streams, because `WaterNode` and the single-draw composite must be proven on the simplest body. But the Lab's ridged-multifractal terrain produces a *radial drainage fan* far more readily than it produces a hero lake, so **W5, not W4, is the PR that produces the demo's real headline shot**, and a hero lake will need an authored basin primitive (see D-W1).

---

## 12. Approaches considered and rejected

| Approach | Why rejected | Evidence |
|---|---|---|
| **Tets as the default water renderer** | 0.43 µs/tet/frame, 100 % CPU, no viewport or distance relief; 3.4–70.5× the surface at equal content; no lighting, no shadows, no clipping planes, approximate-and-possibly-cyclic sorting on domains that are non-convex by construction, and whole tets vanish near the camera | §1.2, §1.4, §6.3 |
| **Hexes straight into PT** | Non-tet cells warn once, set `GaveError` and are **skipped**, so the volume renders partially and wrongly rather than failing loudly. `vtkDataSetTriangleFilter` + `TetrahedraOnlyOn()` converts at exactly **6.000×** cell count | §6.2 |
| **RayCast / ZSweep unstructured mappers** | Pure CPU software renderers with a `vtkRayCastImageDisplayHelper` blit; ZSweep is single-threaded across 4,322 lines; both silently drop to 1/100 resolution under load | §5.1 |
| **`vtkMultiBlockUnstructuredGridVolumeMapper`** | A `std::vector` of PT mappers that sorts *blocks*; every block pays the full per-prop tax — it multiplies the problem | §5.1 |
| **Caching PT's visibility sort as a performance strategy** | Sorting is 15–20 % of PT's cost. A perfect fix leaves PT 3–70× behind. Worth ~60 lines *only* to make the paused-inspection case free | §1.2 |
| **A true-thickness pass** (two targets, or additive signed depth [van der Laan 2009]) | For a lake or river over an opaque bed the analytic exit is *exactly* equivalent, so it is pure waste. Waterfalls use a constant `sheet_thickness_m` — one uniform instead of a render pass | §2, §5.5 |
| **The two-draw chromatic composite** — draw the lid twice, `(GL_ZERO, GL_SRC_COLOR)` then `(GL_ONE, GL_ONE)`, to get per-channel absorption from the fixed-function blender | **Cannot be built on VTK 9.5.** There is no per-actor/mapper/property blend-function API (`SetBlend*` appears on no actor, property or polydata-mapper header); translucent actors have depth writes forced off (`vtkOpenGLActor.cxx:88`); and the translucent path defaults to OIT, which reorders fragments and whose only off-switch is renderer-global (`vtkRenderer.h:930-932`, `:1168`). Behaviour would also flip between cvcGL's shadows-on and shadows-off pass chains (`src/cvcGL/SceneGraph.cpp:1034` vs `:1059`). Replaced by the single opaque draw of §5.2.2, which is both correct and cheaper | §5.2.1 |
| **Making the two-draw scheme work by force** — `GLDepthMaskOverride` + `SetUseOITOff()` + a `vtkPropAssembly` | Repairs two blockers of four. `vtkOpenGLActor::GLDepthMaskOverride()` (`vtkOpenGLActor.h:52`) genuinely fixes the depth mask per-actor, and `SetUseOITOff()` fixes ordering — but **globally**, changing every other translucent prop in the scene, and blocker 1 (no blend-function API) has no workaround short of a mapper subclass that manipulates raw GL state. That subclass would then own blend, depth and OIT interactions for the life of the project, to reach an image the single draw produces exactly, for less | §5.2.1 |
| **A `vtkOpenGLPolyDataMapper` subclass that sets GL state around the draw** | The only route to per-actor blend control, and cvcGL subclasses **no** mapper today (it subclasses a render *pass*, `StridedShadowBaker`, `src/cvcGL/SceneGraph.cpp:916`). It would couple water to `vtkOpenGLPolyDataMapper`'s protected internals across VTK upgrades, and — decisively — **it buys nothing**: the single-draw composite is algebraically identical (§5.2.2) and needs no GL state at all. Rejected as invasive *and* unnecessary, not merely invasive | §5.2.2 |
| **A water-only `vtkRenderPass` inserted into cvcGL's chain** | Avoids the global-OIT question, but requires the `buildPassChain` fix (addition #3) as a hard prerequisite in W4 rather than W7, adds a pass whose interaction with the sea and cloud volumes must then be argued (R7), and still does not give per-actor blend control — a pass sets state for everything it draws. Kept in reserve as the escape hatch if D-W8 option B is ever taken, since the capture pass lands in that chain anyway | §5.2.3, D-W8 |
| **Any OIT scheme** (per-pixel linked lists, HAVS k-buffer, adaptive/multi-layer alpha) | WebGL2 has no ROV/fragment interlock, no image load/store, no SSBOs, no fragment atomics; framebuffer feedback is `INVALID_OPERATION`. HAVS's own authors call the k-buffer "strictly speaking, unstable" | [Callahan 2005]; §5.1 |
| **Depth peeling** | 123 ms exact vs 5.5 ms weighted-blended on the same scene. A volume needs hundreds of layers | [Everitt 2001]; [McGuire & Bavoil 2013] |
| **FFT ocean** [Tessendorf 2004] | Phillips is a fetch-limited wind-sea spectrum; a pond has zero fetch and a stream is advective, not dispersive. 16 fragment passes per cascade on WebGL2 | §2 |
| **Planar reflection** | The single biggest quality jump available — a lake mirroring the treeline reads instantly as a lake — and the single biggest cost: a second full scene traversal, ≈ 145 actor draws ≈ **5.8 ms of CPU**, resolution-*independent*, on a renderer that is CPU-bound at 24.8 fps. **This is the largest visual concession in the design and a reviewer will notice it first.** See D-W3 for the baked-probe alternative | §1.1 |
| **SSR** | 0.3–1.5 ms and noisiest at grazing angles, which is exactly where water spends its screen area | §1.1 |
| **Underwater camera** | Doubles shader permutations (Fresnel flips, TIR at 48.6°, fog applies to everything, shoreline logic inverts) for a view an island fly-through never uses. Mitigation: clamp `pathLen` and fade over the last 0.3 m as the camera crosses | §2 |
| **Hydrology-first terrain inversion** [Génevaux 2013] — grow the network, then build terrain around it | It *replaces* the Lab's generator. And the decisive signal: **[Peytavie 2019], same authors, same lab, six years later, ran it backwards** — bare-earth heightfield in, extraction, then Génevaux-style amplification. Adopt the second half (Strahler, `Q = 0.42·A^0.69`, Rosgen typing, the `B`/`C` construction-tree operators, compact-support kernels); skip the grammar growth and the Voronoi watersheds | §3 |
| **Pipe-model / shallow-water erosion as the lake-surface source** [Mei 2007] | It is a Jacobi relaxation: propagating a level along a 9,833-cell low-gradient reach costs ~97 M time units vs 9,833 for the direct solver — **~10,000×** — and its `d` field is rain-transient, so the extracted shoreline boils and is not reproducible. Keep the sim's *deposition* field, which is the physically motivated source for the silt/gravel split; take the lake surface from FSM | [Barnes 2021] |
| **Inventing new `terrain_class` ids for water** | The shipped registry already has `water_deep` (16), `water_shallow` (15), `puddle` (14), `mud` (13), `gravel` (4), `sand` (5). Its ids are contractually never renumbered and the table is 0..31 with every slot used, so an "append" is a schema version bump | §9.1, D-W4 |
| **Calling `cvc::nav::material_build` directly from `cvc::world`** | Violates the seam that exists so a change to the consumer's blur σ, EDT convention or channel order invalidates zero previously generated bundles. Water writes files | §9.5 |
| **Storing a per-body depth raster** | `φ = level_z − heightfield::sample(x,y)` clipped to the polygon. Storing it is redundant *and* binds water to one resolution, breaking the roadmap's full-fidelity export rule | §3.2 |

### The honest cons of what is being recommended

1. **No heterogeneous interior until W9.** A sediment plume, a thermocline, an estuarine turbidity maximum — W1 cannot express any of it. W9 fixes it without tets.
2. **Water-through-water is wrong**, order-dependent, and the alpha-fade mitigation is a dodge, not a fix.
3. **Nothing between the surface and the bed is visible at all.** The W1 lid is opaque, so a submerged boulder, a fish or a sunken prop is hidden rather than merely un-attenuated — a stronger restriction than the translucent design would have had, and the price of keeping chromatic absorption without a framebuffer read (§5.2.1). The island fly-through has no such geometry; W-T2 removes the restriction outright. See D-W8.
4. **No planar reflection**, so a lake on a forested island reflects a sky gradient rather than the treeline. The largest concession, taken deliberately (§12 table, D-W3).
5. **Neither the water surface nor its bed receives the shadow map.** Consuming `//VTK::Light::Impl` first removes `vtkShadowMapPass`'s injection along with VTK's lighting (§5.2.4). The gain is a shader that is provably identical shadows-on and shadows-off; the loss is that a tree's shadow does not darken the bed seen through the water. The cloud-shadow term is applied, so large-scale variation still reads. This is the sharpest regression against the previous revision's intent and is the substance of D-W8.
6. **Generation is global and cannot be tiled.** One brush stroke can move a spill point kilometres away. Mitigated by the coarse-proxy-live / full-res-on-commit split the Lab already uses, plus the depression hierarchy's locality (an edit inside one leaf invalidates that leaf and its ancestors' sills, not the world).
7. **Six tiers is real maintenance surface.** Bounded by W1/W2/W3 sharing one fragment core and by the tier-agreement test, but a reviewer should push back hard if a seventh is ever proposed.
8. **Tiers cannot cross-fade**, which is why they are fixed at bake time and only `lod_flags` varies. If a user toggles scientific mode live, the body pops.
9. **The connectivity gate can hard-fail a seed.** By design — a silent pass is far worse — but world generation therefore has a rejection rate, and §10's matrix exists to keep it near the measured 0 %.
10. **W10 builds a renderer this document argues against using for water.** Justified only by the FEM/CFD reuse, and it is scheduled last so that judgement can be revisited before any effort is spent.

---

## 13. Risks, and decisions needing the user

### 13.1 Risks, ordered by probability × damage

| # | Risk | Mitigation |
|---|---|---|
| **R1** | ~~The two-draw ordering assumption.~~ **Deleted.** The mechanism it described could not be built on VTK 9.5 at all (§5.2.1) and has been replaced by a single opaque draw, which has no ordering assumption, no blend-state requirement, no depth-mask requirement and no interaction with OIT or depth peeling. **No global renderer setting is changed by the design.** | n/a — the risk is removed rather than mitigated. The residual is R8. |
| **R8** | **The water shader's bed and the terrain shader's surface can drift apart.** W1 evaluates `C_bed` from the terrain albedo splat and heightfield rather than reading the rendered terrain (§5.2.5 con 3), so a change to terrain shading that is not mirrored in the water shader shows up as a discontinuity at the waterline. | The two share one GLSL include for `bedNormal` and the albedo fetch. `cvcgl_water_shore` is a golden that samples a band straddling the shoreline and asserts continuity within tolerance; it fails loudly the first time terrain shading changes alone. Fully retired if D-W8 option B is taken, since the capture then supplies the real terrain colour. |
| **R2** | **The per-actor CPU tax is uncertain by 5.5×** (28.2 µs vs 5.1 µs from two harnesses). The merge-vs-split policy depends on it. | Budget the conservative 28.2 µs. `cvcgl_prop_sweep` (Lab PR L3) resolves it on the real machine and its curve, not this document's number, sets the policy. |
| **R3** | **W5's fixed cost is unresolved below ~10 k tets** (§1.3), which is exactly where any shippable configuration sits. | PR W10 re-runs the sweep with `SetUseFloatingPointFrameBufferOff()` before `max_tets` is trusted. Until then W5 costs are quoted as ranges and the cap is set from the pessimistic end. |
| **R4** | **The wasm GLES3 mapper has never been asked to compile this shader.** The `CVC_FS_NORMAL` trap fails the whole program silently and the actor is simply invisible. | `cvcgl_water_shader` compiles and links on **both** backends in PR W4, before any wasm work. |
| **R5** | **The Lab terrain's real lake inventory is unknown.** All published lake counts were measured on the `lsystem_forest` control (which has zero depressions) or on augmented terrain. | PR W1's demoable outcome is precisely this measurement: run the inventory on the actual Lab terrain across the runoff sweep and publish it before W4 designs a shot around it. |
| **R6** | **Water actors merge per LOD band, so VTK can only frustum-cull a whole band.** A single visible pond keeps its band's actor alive. | This is why the L3 rung must be a *fold* into the tile splat, not a cull. Bands are per-region, not global. |
| **R7** | **The capture pass (W7) interacts with the sea and cloud volumes.** `vtkFramebufferPass` blits colour only; getting the depth restore wrong makes the sea draw over the mountains, silently. | `SceneCapturePass` restores depth via `TextureDepthBlit`; `cvcgl_water_depth` renders sea + lake + mountain and asserts occlusion. W7 is explicitly gated on it. |

### 13.2 Decisions that need the user

**D-W1 — Where does a hero lake come from?** The generation pipeline is correct, but a *convincing* lake needs a basin the terrain function will actually produce. Options: (a) add a `basin` / `lake_seed` terrain primitive to the Lab's construction tree ([Génevaux 2013]'s `B`/`C` operators, ~1 day, composes with everything); (b) rely on the erosion pass's natural depressions and accept whatever appears; (c) author basins by hand in the Surface tab. **Recommendation: (a).** A one-primitive addition that lets an author place a lake is worth more than any renderer feature in this document, and (b) alone risks a demo with no lake in it.

**D-W2 — Which runoff default ships?** `runoff_depth_m` is the content knob (§3.4). **Recommendation: 0.35 m**, which on test terrain gives full small basins, partially-filled large ones and 100 % connectivity. But this is an aesthetic call about how wet the world looks, and it should be seen before it is fixed.

**D-W3 — Reflections: sky-only, or a baked per-lake probe?** No planar reflection is the largest visual concession (§12). A third option none of the analysis costed: **bake a per-lake reflection probe** (dual-paraboloid or small cubemap) at world-bake time, re-baked on the same cadence as the Lab's existing 256×256 tile albedo bake. The forest is static between growth commits and reflections are low-frequency, so sway is invisible at reflection scale. It would put the treeline in the water for near-zero per-frame cost. **Recommendation: ship sky-only in W4, prototype the baked probe as a W8 stretch, and only consider live planar reflection if `cvcgl_prop_sweep` shows more CPU headroom than the control suggests.**

**D-W4 — Does swift shallow water get its own class id?** §9.1 maps it onto `water_deep` (16) to avoid a schema bump, which is semantically slightly odd ("deep" naming an unfordable 40 cm torrent). The alternative is ids 32/33 (`torrent`, `wet_rock`) — a registry schema version bump, re-assertion of every ontology-variant invariant, and coordination with GRL-SNAM. **Recommendation: reuse id 16 for v1**, document the semantics as "unfordable water", and revisit only if a consumer needs to distinguish them.

**D-W5 — Which connectivity policy?** The fixed decision (connected terrain) maps to `single-island`. But water introduces a case the archipelago gate did not have: a *stream* that severs land on a single island. **Recommendation: `single-island` with the ford ladder of §9.3 enabled**, i.e. fords are a legitimate repair rather than a failure. The alternative — treat any severing stream as a hard reject — would raise the rejection rate for no navigational benefit, since a ford is genuinely traversable. Confirm that fording is an acceptable agent capability for the GRL-SNAM consumer; it is a domain claim, not a rendering choice, and it is the same question the roadmap's `amphibious` ontology raises.

**D-W6 — Is W10 worth building at all?** It is ~250 lines of cvcGL for a renderer this document recommends against using for water. Its justification is entirely the FEM/CFD reuse: the first unstructured path in cvcGL, usable for LBIE output, `cvc tetrahedralize` results, and any solver mesh that must not be resampled. **Recommendation: build it, last, and scope its demo as `water_column` (a real heterogeneous field with a transfer function and a parked camera) rather than as scenery.** If the FEM/CFD use case is not on the roadmap within a year, cut it — the honest answer to the original question does not require it to exist.

**D-W7 — Does the demo need the capture pass at all?** W7 buys screen-space refraction, caustics on the true depth-buffer bed, and soft mist, for 0.27 ms and a pass-chain change. W1's analytic path already gives absorption, caustics on the analytic bed, Fresnel, glint, ripples and foam without any of that. **Recommendation: ship W1–W6 first and look at it.** If refraction is not missed, W7 can be deferred indefinitely and the renderer stays simpler. *(D-W8 raises the stakes on this one: the capture now also buys the real shadowed bed and submerged geometry, so read the two together.)*

**D-W8 — Evaluate the bed in-shader (W1 as specified), or pull the capture pass forward into W1?** *This is the one genuine decision the §5.2 rework creates, and it is a quality-versus-simplicity call rather than a technical one — both options are verified buildable.*

The §5.2.1 theorem forces the choice: chromatic absorption of scene colour needs either blend-function control (does not exist in VTK) or a framebuffer read. So W1 either evaluates the bed or captures it.

- **Option A — evaluate the bed in-shader (what §5.2 now specifies).** One opaque actor on the stock path. No render pass, no `buildPassChain` change, no global renderer settings, no OIT question, no depth-peeling guard, no path-dependence on shadows. ~240 LoC for `WaterNode` + 110 for the three `GeometryNode` passthroughs + 8 for the shadow-baker fix. **Costs 0.34 ms typical.** Gives up: submerged non-bed geometry, tree shadows on the bed, and exact agreement with terrain shading (§5.2.5 cons 1–3; risk R8).
- **Option B — land `SceneCapturePass` in W1 instead of W7.** The shader reads the real, lit, shadowed, cloud-shaded framebuffer colour and depth, so `C_bed` is exact and cons 1–3 vanish together, along with R8 and the shoreline golden. Water can stay a single opaque draw — the capture supplies the bed, the blend hardware is still not involved — so none of the §5.2.1 blockers return. **Costs +180 LoC, +0.27 ms, the `buildPassChain` fix (addition #3) promoted from W7 into W4, and R7 (the sea/cloud depth-restore risk) moved forward with it.** It also delivers refraction and soft mist early, making D-W7 moot.

**Recommendation: A, and hold B in reserve for W7 exactly as scheduled.** Three reasons. First, A is the only option with *no* pass-chain change, and `SetPass(nullptr)` at `src/cvcGL/SceneGraph.cpp:1034` means the shadows-off path has no chain to insert into — that fix (addition #3) is real work with a real regression surface (volumes silently vanishing), and W4 should not be the PR that takes it. Second, the three things A gives up are invisible in the demo this roadmap is actually driving: an island fly-through with no submerged props, where the bed under any lake deep enough to tint is already too dark for a tree shadow to read. Third, B is not foreclosed by A — it is a strict refinement of the same single-draw shader, swapping an evaluated `C_bed` for a sampled one, so taking A now costs nothing if B lands later.

**Take B instead if** the Lab acquires submerged geometry (a boulder or wreck prop under a lake), or if the first `lsystem_lab --water` frame shows the shoreline discontinuity R8 warns about and the shared-include mitigation proves fragile. Both are visible within one PR of W4, which is early enough to change course cheaply.

---

## 14. Appendix

### 14.1 Optics parameter table

Pure-water absorption `a` (m⁻¹) [Pope & Fry 1997; Smith & Baker 1981]:

| λ | 650 nm (R) | 600 nm | 550 nm (G) | 500 nm | 450 nm (B) |
|---|---|---|---|---|---|
| a | 0.34 | 0.244 | 0.057 | 0.020 | 0.0092 |

**Red is attenuated ~35× faster than blue. That ratio is the whole effect.** Shipped presets, including scattering:

| preset | σ (R, G, B) m⁻¹ | `deep` (L∞) | note |
|---|---|---|---|
| `clear_alpine` | 0.42, 0.075, 0.045 | 0.051, 0.231, 0.290 (`#0d3b4a`) | |
| `island_lake` | 0.55, 0.16, 0.11 | 0.075, 0.294, 0.271 (`#134b45`) | default |
| `tannic_pond` | **0.45, 0.80, 1.40** | 0.165, 0.184, 0.110 (`#2a2f1c`) | **σ_B > σ_G > σ_R — the ordering INVERTS.** CDOM absorbs blue hardest; this is why forest ponds are brown. Getting the inequality backwards yields murky teal. |
| `silty_stream` | 0.70, 0.45, 0.40 | 0.231, 0.251, 0.204 (`#3b4034`) | |

Effective σ with turbidity: `σ_eff = σ · (1 + sigma_per_ntu · turbidity)`.
Fresnel: `F0 = ((1.000 − 1.333)/(1.000 + 1.333))² = 0.0204` [Schlick 1994]; error < 1 % average, ~3.6 % max at 85°.
Total internal reflection at `asin(1/1.333) = 48.6°` — only matters underwater, which is cut.

### 14.2 Waves — Gerstner, Z-up

Per wave: unit 2-D direction `d`, wavelength `L`, steepness `s ∈ [0,1]`. [Finch 2004]

```glsl
float k = 6.28318 / L;
float c = sqrt(9.81 / k);              // deep-water phase speed, omega = sqrt(g k)
float a = s / k;                       // amplitude is DERIVED, never a free parameter
float f = k * (dot(d, p.xy) - c * t);

P        += vec3(d.x*a*cos(f),  d.y*a*cos(f),  a*sin(f));            // Z-up
tangent  += vec3(-d.x*d.x*s*sin(f), -d.x*d.y*s*sin(f),  d.x*s*cos(f));
binormal += vec3(-d.x*d.y*s*sin(f), -d.y*d.y*s*sin(f),  d.y*s*cos(f));
// init tangent = (1,0,0), binormal = (0,1,0), then N = normalize(cross(tangent, binormal))
```

**Constraint: `Σ sᵢ ≤ 1` over all waves, or the surface self-intersects and loops above the crests.** Amplitude is additionally damped by `saturate(dist_shore / damp_dist)`, or waves poke through the bank. Shipped set: λ = 12 / 7 / 3 m, s = 0.10 / 0.06 / 0.04, directions spread ±40° about the wind. Cost ~5 ALU + one `sin`/`cos` pair per wave per vertex; **irrelevant at any sane vertex count, and analytic normals mean no CPU and no geometry pass.**

Shallow-water dispersion `ω = √(g·k·tanh(k·h))` is what makes waves slow, shorten and steepen approaching a beach. Overkill for a lake; skipped.

### 14.3 Parameter table

| parameter | default | note |
|---|---|---|
| `runoff_depth_m` | 0.35 | THE content knob (D-W2) |
| `min_lake_area_m2` | 100.0 | below → W0 |
| `min_lake_depth_m` | 0.15 | below → W0 |
| `channel_threshold_cells` | 400 | or slope-area `A·S² > θ` |
| `d_ford_m` | 1.00 | human wading limit |
| `vd_safe_m2s` | 0.50 | φ·v; < 0.5 safe, < 1.0 marginal |
| `ford_order` | 2 | Strahler ≤ this is fordable by construction |
| `fall_min_drop_m` / `fall_min_slope` | 2.0 / 1.19 | tan 50°, tuned for a 42° cliff class |
| `lake_edge_m` | 2.5 | CDT + Steiner target |
| `skirt_drop_m` | 0.60 | **== the terrain skirt drop** |
| `simplify_tol_m` | 0.60 | Douglas–Peucker, **world metres** |
| `foam_width_m` | 1.1 | |
| `glint_power` | 480 | plus a broad lobe at 20, intensity 0.08 |
| `ripple_metres` | 0.03 | one ripple texel ≈ 2–5 cm of world, or it reads as a pool cover |
| `refract_fade_dist_m` | 0.8 | no refraction in 2 cm of water |
| `spray_radius_m` | `clamp(1.6·√drop·Q^0.25, 2, 40)` | |
| `max_tets` / `max_tet_bodies` | 4800 / 1 | W5 cap; **native only**; see D-W6, R3 |
| `sheet_thickness_m` | 0.12 | waterfall constant; **never a thickness pass** |

### 14.4 Three things that will bite whoever implements this

1. **Using an eye-Z difference instead of the reconstructed vertical column for the shoreline** — the foam band slides as the camera moves and you will chase it for a day. Baked `columnH` (§4.1) prevents it structurally.
2. **Forgetting to clamp the refraction offset by depth** (W7) — the dry beach shows through the shallows in the wrong place.
3. **Injecting the water stack at `//VTK::Light::Impl` with `ReplaceFirst = false`.** `vtkOpenGLPolyDataMapper::BuildShaders` applies non-first replacements *after* `ReplaceShaderValues`, by which point VTK's lighting has already consumed the anchor and the substitution silently matches nothing — the water shader compiles, links, and renders VTK's default lit surface with none of your code in it. `GeometryNode::addFragmentShaderReplacement` passes `true` (`src/cvcGL/GeometryNode.cpp:603`); do not "fix" it.

---

## Sources

- [Barnes 2014] Barnes, Lehman & Mulla, *Priority-Flood: An Optimal Depression-Filling and Watershed-Labeling Algorithm for DEMs*, Computers & Geosciences 62:117–127. Also *An Efficient Assignment of Drainage Direction Over Flat Surfaces*, ibid. 62:128–135.
- [Barnes 2020] Barnes, Callaghan & Wickert, *Computing water flow through complex landscapes, Part 2: Finding hierarchies in depressions*, Earth Surf. Dynam. 8:431–445.
- [Barnes 2021] Barnes, Callaghan & Wickert, *… Part 3: Fill–Spill–Merge*, Earth Surf. Dynam. 9:105–121. (The Lake-Level Equation.)
- [Braun & Willett 2013] *A very efficient O(n), implicit and parallel method to solve the stream power equation.* Geomorphology 180-181:170–179.
- [Callahan 2005] Callahan, Ikits, Comba & Silva, *Hardware-Assisted Visibility Sorting for Unstructured Volume Rendering*, IEEE TVCG 11(3):285–295.
- [Comba 1999] Comba, Klosowski, Max, Mitchell, Silva & Williams, *Fast Polyhedral Cell Sorting*, Computer Graphics Forum 18(3).
- [Cordonnier 2016] Cordonnier et al., *Large Scale Terrain Generation from Tectonic Uplift and Fluvial Erosion*, CGF 35(2):165–175.
- [Dunne & Leopold 1978] *Water in Environmental Planning.* (The `Q = 0.42·A^0.69` discharge relation.)
- [Emilien 2015] Emilien, Poulin, Cani & Vimont, *Interactive Procedural Modelling of Coherent Waterfall Scenes*, CGF 34(6):22–35.
- [Everitt 2001] Everitt, *Interactive Order-Independent Transparency*, NVIDIA.
- [Finch 2004] Finch, *Effective Water Simulation from Physical Models*, GPU Gems 1, ch. 1.
- [Garrity 1990] Garrity, *Raytracing Irregular Volume Data*, Computer Graphics 24(5):35–40.
- [Génevaux 2013] Génevaux, Galin, Guérin, Peytavie & Beneš, *Terrain Generation Using Procedural Models Based on Hydrology*, ACM TOG 32(4):143.
- [Grimes 2011] Grimes, *Making and Using Non-Standard Textures*, Valve, GDC 2011. (Two-phase flow maps.)
- [Guardado 2004] Guardado & Sánchez-Crespo, *Rendering Water Caustics*, GPU Gems 1, ch. 2.
- [Jain 2024] Jain, Kerbl, Gain, Finley & Cordonnier, *FastFlow: GPU Acceleration of Flow and Depression Routing*, CGF 43:e15243.
- [Johanson 2004] Johanson, *Real-time Water Rendering: Introducing the Projected Grid Concept*, Lund University.
- [Lengyel 2007] Lengyel, *Oblique View Frustum Depth Projection and Clipping* / *Projection Matrix Tricks*, GDC 2007.
- [Leopold & Maddock 1953] *The Hydraulic Geometry of Stream Channels and Some Physiographic Implications*, USGS PP 252.
- [Max 1995] Max, *Optical Models for Direct Volume Rendering*, IEEE TVCG 1(2):99–108. **The `∫τ ds = τ·L` argument.**
- [McGuire & Bavoil 2013] *Weighted Blended Order-Independent Transparency*, JCGT 2(2).
- [Mei 2007] Mei, Decaudin & Hu, *Fast Hydraulic Erosion Simulation and Visualization on GPU*, Pacific Graphics.
- [Montgomery & Dietrich 1992] *Channel Initiation and the Problem of Landscape Scale*, Science 255:826.
- [Morrical 2020] Morrical, Wald, Usher & Pascucci, *Accelerating Unstructured Mesh Point Location with RT Cores*, IEEE TVCG 28(8):2852–2866.
- [Münstermann 2018] Münstermann, Krumpen, Klein & Peters, *Moment-Based Order-Independent Transparency*, PACMCGIT 1(1):7.
- [O'Callaghan & Mark 1984] *The extraction of drainage networks from digital elevation data*, CVGIP 28:323–344.
- [Peytavie 2019] Peytavie, Dupont, Guérin, Cortial, Beneš, Gain & Galin, *Procedural Riverscapes*, Pacific Graphics / CGF.
- [Planchon & Darboux 2002] *A fast, simple and versatile algorithm to fill the depressions of DEMs*, Catena 46:159–176.
- [Pope & Fry 1997] Pope & Fry, *Absorption spectrum (380–700 nm) of pure water*, Applied Optics 36(33):8710–8723.
- [Schlick 1994] Schlick, *An Inexpensive BRDF Model for Physically-based Rendering*, CGF 13(3):233–246.
- [Shirley & Tuchman 1990] *A Polygonal Approximation to Direct Scalar Volume Rendering*, Computer Graphics 24(5):63–70.
- [Silva 2005] Silva, Comba, Callahan & Bernardon, *A Survey of GPU-Based Volume Rendering of Unstructured Grids.*
- [Smith & Baker 1981] *Optical properties of the clearest natural waters*, Applied Optics 20(2):177–184.
- [Tarboton 1997] Tarboton, *A new method for the determination of flow directions and upslope areas in grid DEMs*, WRR 33(2):309–319.
- [Tessendorf 2004] Tessendorf, *Simulating Ocean Water*, SIGGRAPH course notes.
- [van der Laan 2009] van der Laan, Green & Sainz, *Screen Space Fluid Rendering with Curvature Flow*, I3D 2009:91–98.
- [Weiler 2003] Weiler, Kraus, Merz & Ertl, *Hardware-Based Ray Casting for Tetrahedral Meshes*, IEEE Vis 2003:333–340.
- [Williams 1992] Williams, *Visibility-ordering meshed polyhedra*, ACM TOG 11(2):103–126.
- [Wyman & McGuire 2017] *Hashed Alpha Testing*, I3D 2017.
- [Yuksel 2007] Yuksel, House & Keyser, *Wave Particles*, SIGGRAPH 2007.

**In-tree anchors used above:** `docs/roadmap/LSYSTEM-LABORATORY-ROADMAP.md` §1.3, §2.1, §4.4, §4.6, §7.1, §7.4, §7.8, §8.7, §8.9, §8.11, §11.1, §13.2, §16.2 · `src/cvcGL/SceneGraph.cpp:1029-1074` (the pass chain and the `SetPass(nullptr)` hole at `:1034`) · `src/cvcGL/SceneNode.cpp:62-70` (`AddViewProp`) · `inc/cvc/gl/GeometryNode.h:101-111,125,161,178` · `inc/cvc/gl/VolumeNode.h:64-100` (the shading knobs PT ignores) · `inc/cvc/geometry/geometry.h:84-85,182-188` (`tets_t`/`hexs_t`) · `inc/cvc/nav/material.h:93-94` and `inc/cvc/nav/sim_world.h:182` (the consumer surface — reached through files, not a call) · `src/cvcGL/CMakeLists.txt:24-32` (VTK components; no build change needed) · VTK 9.5.0 `Rendering/VolumeOpenGL2/vtkOpenGLProjectedTetrahedraMapper.cxx:103,337-348,485-524,552-556,764-775,1030-1068,1075-1091`, `Rendering/Core/vtkCellCenterDepthSort.cxx:150-181`, `Rendering/Core/vtkVisibilitySort.h:11-15`, `Rendering/OpenGL2/vtkFramebufferPass.cxx` (colour-only blit), `vtkOpenGLRenderWindow.h:484,487,490` (`TextureDepthBlit`), `vtkShaderProperty.h:86-87`, `vtkUniforms.h:84-101`.