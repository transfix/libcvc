# Physics-Aware Water, Wind and Weather — Roadmap

**Status:** proposal, revision 1 (2026-11-09). Nothing in it is built. It follows `OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md` (how the FFT ocean was ported), `WATER-RENDERING-ROADMAP.md` (inland water, W1–W10), `WORLD-GEOMETRY-ROADMAP.md` (G6 fields), `GENERATED-WORLD-ROADMAP.md`, `ARIADNE_WORLDS.md` and `WORLD-CLOCK-SYNC-ROADMAP.md`. Where this document and one of those disagree about what is built, the code wins. Where they disagree about a plan, raise it as a decision (§15).

**Goal.** Water, wind and weather become parts of the world model. The terrain generator decides where the sea, lakes, ponds and streams are. Waves react to every shore. Bodies float on, and push through, the same waves that are drawn. Boats leave wakes. Wind and storms change the water, the sky, the ground and the physics together. The same scene runs **locally** (arigl, arigl-web) and **hosted** (arisrv), and every client draws the same water.

**Where the work happens.** The work starts in `examples/cvcgl/lsystem_coast.cpp`, where the FFT ocean lives today. Each piece then moves into Ariadne node types that `examples/ariadne/documents/terrain_lab.ari` uses. The C++ `examples/cvcgl/terrain_lab.cpp` is being dropped; nothing here targets it. Its `water_field` / `water_transfer` path is used by nothing else (§1.5).

**Provenance markers.**
- **[C]** read from source, file:line on master `ad8b0758`.
- **[X]** read from ABYSSAL (`Token-Gremlin/natural-disasters`, MIT, last code commit `849ff7f4`, 2026-08-28).
- **[L]** from the literature (§17).
- **[P]** derived or projected here, not measured.
- **[E]** an estimate.

---

## 0. Direct answers

1. **Can the water respond to the shore?** Yes, in two layers.
   - The ambient FFT sea is periodic and has no edges, so it cannot truly reflect off anything. It can be damped by depth, and a shore layer can take over near the coast: shoaling, refracting, breaking, running up and reflecting along the shoreline's distance field (Phase A, §4). That layer is a set of formulas rather than a simulation, but every term in it is grounded in the physics.
   - True reflection of *disturbances* (wakes, splashes) comes from the interactive layer, which masks the land (Phase C).
2. **Can bodies affect the waves, and the waves the bodies?** Yes.
   - **Waves move bodies.** cvc::physics already has fluid fields that apply buoyancy, drag, damping and per-cell torque [C `inc/cvc/physics/field.h:27-62`]. What they lack is a moving surface: the water's top is the flat top of a volume's box [C `src/cvc/physics/field_step.cpp:551-560`]. Phase B adds a deterministic CPU wave model and a surface hook on fields.
   - **Bodies make waves.** Phase C adds a dispersive interactive wave layer whose sources are the bodies' changing immersion.
3. **A convincing boat** needs four things: B and C, forces applied at a point (Phase D), and a hull model that works per triangle against the wave surface.
4. **Wind** today sets one spectrum, and changing it rebuilds the spectrum at once. Phase E covers:
   - a swell spectrum separate from the wind sea;
   - fetch and lee (calm water downwind of land);
   - gusts;
   - the sea state lagging behind the wind;
   - one wind for the whole scene, physics included.
5. **Lakes and ponds** get the same machinery as the sea. The difference is only which wave content a body carries: swell plus wind sea, fetch-limited wind sea only, or ripples only. The world model has to produce *water bodies* (§3).
6. **Rain, dark clouds and storms** build on terrain_lab's `cloud_sky` (Phase F, §9). Today nothing in the tree draws rain, storms or wet ground [C].
7. **Into the world model, locally and on arisrv.**
   - In the world model:
     - `water_bodies` is generated with the world;
     - a water volume in the world's geometry becomes one physics field;
     - `water`, `weather` and `boat` Ariadne nodes draw and drive it.
   - **Local scenes** need nothing beyond the phases.
   - **arisrv scenes** depend on room work that is planned but not built: carrying a generated world as a descriptor that clients regenerate, ticking custom nodes in rooms, and simulating scene bodies on the server (§11).
8. **FEM and the mesher** are not the real-time wave solver, but they help in specific places (§13).
9. **Upstream ABYSSAL** has no water changes since our port, and it has no shoreline, buoyancy or object code. What it has that we don't is in §10. Every item there is placed in a phase or deferred with a reason.

---

## 1. What exists today

### 1.1 The FFT ocean (lsystem_coast)

- **`OceanFFT`** [C `src/cvcGL/ocean/OceanFFT.{h,cpp}`] is ABYSSAL's spectral core re-hosted on VTK: a JONSWAP h0 pass, time evolution, an 8-stage butterfly IFFT, and an assemble pass that writes displacement and the Jacobian. It uses fragment ping-pong only (no compute shaders) and runs on WebGL2.
- **Cascades.** The demo runs 3 cascades with tiles 251 / 83 / 29 m, wind factors 1.0 / 0.8 / 0.6 and weights 1.0 / 0.55 / 0.32 [C `lsystem_coast.cpp:1337-1344`].
- **Shaders.**
  - The vertex shader displaces everywhere [C `lsystem_coast.cpp:607-620`].
  - The fragment shader's shore effects are a depth tint and a fixed foam band from `coastH()`, a hand copy of `terrainH()` in GLSL [C `:627-646`, `:673-676`].
- **Clock and knobs.** Waves advance on `cvc::world_clock` [C `:1615`]. The knobs live at `coast.water.*`.
- **Port debts found while reading.** These matter for Phase B, where the CPU must reproduce what the GPU draws:
  - **Shared noise.** All three cascades draw the *same* noise: `hash01(x, y)` has no per-cascade seed [C `OceanFFT.cpp:180-186`, `:219-230`].
  - **Overlapping bands.** No cascade is band-limited. Each one is a full JONSWAP spectrum at its own tile size, so their sum counts some energy two or three times. ABYSSAL assigns each cascade a wavenumber band [X `OceanFFT.js`, cascade bands `bi = 2π/Li·4`].
  - **Simplified spreading.** Directional spreading is `cos² + 0.03` [C `OceanFFT.cpp:67`], not ABYSSAL's Donelan–Banner spreading.
  - **Instant respectralize.** A change of wind, fetch or depth rebuilds h0 in one frame [C `lsystem_coast.cpp:1641-1653`], so the sea pops.

### 1.2 cvc::physics fields

- **Shapes.** A dynamic sphere, box or convex hull is cut into about 8 cells along its longest side [C `field_step.cpp:30`, `:151-158`].
- **Per-cell forces.** Each step, each cell's world box is clipped to the field's box. The immersed part gets buoyancy, quadratic drag against the field's velocity, and damping. Each acts at the cell's centre, so torques follow [C `field_step.cpp:515-617`]. Drag is integrated implicitly [C `:603-606`].
- **Velocity.** Fields carry an optional velocity volume per axis, which can express currents and river flow [C `field.h:96-99`]. `field_load` reports force, torque and immersed volume per body [C `field.h:143-147`].
- **Missing: a moving surface.** The surface is the box top.
- **Done (O10): off-centre forces.** The public API had only `add_force` / `add_impulse` at the centre of mass; `world::add_force_at` and `add_torque` now add the off-centre force and the torque Phase D needs (§7 D1).
- **Determinism.** Runs are cross-platform deterministic by default, and `det::` math is used in field stepping [C `world.h:26-30`, `field_step.cpp:607`]. **This is the constraint the whole design is built around** (§2, rule R1).
- **Friction.** `world::set_material` is documented as "how rain on asphalt … reaches a vehicle's tires" [C `world.h:224`]. That is the hook weather uses (Phase F).

### 1.3 Pieces we can reuse

- **`cvc::gl::HeightFieldTexture`** [C `inc/cvc/gl/HeightFieldTexture.h`]:
  - an R32F grid read with `texelFetch` and bilinear filtering done by hand (WebGL2-safe);
  - a CPU `sample()` that is "the GPU's arithmetic, on the CPU";
  - row-band updates from any thread;
  - spliceable GLSL (`cvc_hf_height`).

  This is the template for every CPU/GPU-shared grid below.
- **`cvc::nav::edt2_squared`** and the `sdf_field` builder [C `inc/cvc/nav/grid_nav.h:59-80`]: an exact Euclidean distance transform, plus a signed distance field with unit normals. These give the shoreline distance field without new code.
- **`GeometryNode::setShaderTexture` / `setShaderUniform*`** [C `inc/cvc/gl/GeometryNode.h:199`]: binds live render-to-texture fields into a mesh shader every draw.
- **`cvc::fem`** [C `inc/cvc/geometry/fem.h`]: P1 stiffness and mass matrices, Poisson, heat, eigenmodes, and solves stepped a slice at a time (§13).

### 1.4 terrain_lab.ari's water

The water is drawn by the Ariadne `generated_world` node, not by lsystem_coast.

- **Level.** One global level: `water_level_m`, with sea level at 0 m, and "below it is water (streams, ponds, coast)" [C `inc/cvc/world/generated_world.h:100-103`].
- **Surfaces.** `make_water_rungs` builds flat surfaces at that level, one per terrain LOD rung, coloured by a Beer–Lambert `water_colour(depth)` [C `src/cvc/world/terrain_lod.cpp:230-244`]. Each wet chunk is a `LodGraphicsNode` under `<id>_water` that switches rung with its ground [C `src/cvcGL/ariadne/world_nodes.cpp:327-421`].
- **Shoreline.** `paint_water` paints a 2048² RGBA texture whose alpha is 0 where dry, so dry fragments are discarded [C `generated_world.cpp:2002`, `world_nodes.cpp:344-354`].
- **Shading.** `style_water` is stock VTK Phong (specular 0.6, power 80), opaque and static: no custom shader, no animation [C `world_nodes.cpp:208-222`].
- **No lakes at their own levels.** There is no hydrology: no lake at its own spill level, and no stream flow. `WATER-RENDERING-ROADMAP.md` W1 plans these (Priority-Flood, the depression hierarchy, Fill–Spill–Merge); none of it is built. Today a "pond" is a hollow that dips below the global level.

### 1.5 terrain_lab.ari's sky and clouds

The sky in terrain_lab.ari is a `cloud_sky` node named `sky` [C `terrain_lab.ari:154-167`], realized by `realize_cloud_sky` [C `src/cvcGL/ariadne/lsystem_nodes.cpp:945`, registered at `:1289`].

- **Deck.** `cvc::world::cloud_layer` [C `inc/cvc/world/cloud_layer.h:116`]:
  - It morphs 4 cloud maps of 60×60×28 over a periodic grid of cells.
  - Coverage is a Dobashi cellular automaton that keeps exactly `round(coverage × cells)` cells cloudy [C `cloud_layer.cpp:387-475`].
  - It has an orographic `forcing` hook [C `cloud_layer.h:157-163`].
- **Drawing.**
  - Up to 8 near `VolumeNode`s are handed to the cells nearest the camera [C `lsystem_nodes.cpp:451-452`, `:841`].
  - A far mesh of 10 shells ray-marches an R8 opacity texture out to 4 km [C `:503-617`, `:1067`].
- **Light.**
  - Each voxel carries its optical depth toward the sun, giving `exp(-lightK·tau)` through a grey-blue → white ramp [C `cloud_layer.cpp:206`, `lsystem_nodes.cpp:585-588`, `:812-819`].
  - VTK's own volume shading is off because it "reads as storm cloud" [C `lsystem_nodes.cpp:1032`].
  - Ground shade is an R8 mask spliced into every receiver's lighting sum, double-buffered and crossfaded [C `:469-493`, `:757`, `:1226-1241`].
- **Gaps.**
  - **The water is not a shade receiver**, so it gets no cloud shadow [C `terrain_lab.ari:157`].
  - **The cloud sun is separate from the light rig.** `cloud_sky` reads its own `sun_az` / `sun_el` [C `lsystem_nodes.cpp:964-965`]; `lighting.sun` drives `StageLighting::setSun` [C `StageLighting.cpp:338`]; terrain_lab repeats the numbers in both places [C `terrain_lab.ari:122`, `:163-164`].
  - **Few live keys.** The live keys are `sky.drift`, `.morph`, `.shadow`, `.evolve` and `.orographic` [C `lsystem_nodes.cpp:1082-1086`]. Coverage, base/top, sun, wind direction and darkness are props read once at realize.
- **Sky.** A VTK gradient background only. There is no atmosphere, fog, haze or time of day anywhere [C `room_view.cpp:204-215`].
- **Drift.** Clouds drift at 3 m/s × `sky.drift` along `wind_az` [C `lsystem_nodes.cpp:438`, `:991-993`].
- **Older copies.** lsystem_coast and lsystem_forest each carry a private copy of the older cloud code [C `lsystem_coast.cpp:759`, `:908`, `:1029`]. Phase F does not build on those.

---

## 2. Design rules

- **R1. Physics never reads the GPU.** Anything that moves a body comes from a deterministic, standard-library CPU model. The GPU draws that model *plus* detail too small to matter to a body. GPU readback is not deterministic, stalls the pipeline, and `vtkTextureObject::Download` is not a WebGL2 operation. This keeps `state_hash`, save/restore, replays and native-vs-wasm agreement intact.
- **R2. Replicate inputs, not fields.** A sea is its parameters, its seed and the world clock. A storm is a schedule. A wake is the bodies that made it. Clients rebuild all of it, so arisrv replicates body states and a handful of state keys, never height fields (§11).
- **R3. One spectrum, two evaluators.** h0 is built once per cascade on the CPU and uploaded; the GPU h0 pass is retired. The GPU evolves the whole spectrum for the picture. The CPU evolves the long-wave band for physics. Both see the same numbers by construction.
- **R4. Seabed, shores and water bodies come from the world model** (§3), as CPU/GPU-shared grids in the `HeightFieldTexture` mould. No GLSL copy of a terrain function (retire `coastH()`).
- **R5. One wind and one weather per scene**, read by the trees, clouds, water, spray, rain and physics alike.
- **R6. Every lever is a state key**, live where possible (today's `coast.water.*`, later the node's keys), so ImGui panels, Python and arisrv clients all drive it the same way.
- **R7. WebGL2 parity.** Fragment ping-pong only. Float32 textures read with `texelFetch` and filtered by hand, as `OceanFFT` and `HeightFieldTexture` already do. Every phase states its pass count and is measured in the browser before it joins a deployed demo.

---

## 3. Water bodies: the shared foundation (Phase 0)

Everything later (depth, shore distance, fetch, wake masks, physics fields, rain ripples) is keyed by **which water body a point is in and what its surface level is**. The ocean around the island is one body; each lake, pond and stream reach is another.

**W0.1 `water_bodies`: a world-model output.** It is produced by `cvc::world`, standard library only, and is a grid over the world extent holding, per texel:
- `level`: the body's still-water surface (m);
- `id`: the body;
- `depth`: level − ground, ≤ 0 on dry land.

Per body it holds:
- `kind`: sea / lake / pond / stream;
- area, bounding box, and the surface level;
- a `fetch_scale`: the body's largest open-water extent;
- for streams, a flow-velocity grid.

How it is built:
- **Today (single level):** label the 8-connected wet components of `ground < water_level_m`. Every component touching the world edge is part of the one `sea`: the inlets meet past the world, and a standard terrain_lab world has several. Enclosed components are `lake` or `pond` by area. The threshold is decision **D7**; a starting point is 1 ha [E].
- **After W1 (hydrology):** each depression's fill level comes from Priority-Flood / Fill–Spill–Merge, so a mountain lake sits at its own spill level above the sea. Streams come from the accumulation network, with a Manning velocity as the water roadmap describes.

W0 is the seam between that roadmap's hydrology and this one's dynamics. Nothing here needs W1 to start; everything gets better once W1 lands.

**W0.2 The shore grid.** Per texel: signed distance to the body's shore (offshore +), the unit shore normal, the local seabed slope, and the shore phase P (§4.3). It is built on the CPU:
1. Make the land mask per body from `depth ≤ 0`.
2. Take the exact Euclidean distance transform of the mask and of its complement. `cvc::world` may not include `cvc::nav` (a configure-time rule), so it has its own copy of `edt2_squared`'s Felzenszwalb–Huttenlocher transform.
3. Take central differences for the normal. Smooth over about one swell wavelength so the medial axis does not twist crests [P].

It is rebuilt on regeneration. It is CPU-sampleable for physics and uploaded as an RGBA32F texture beside the seabed `HeightFieldTexture`. The phase P depends on the swell's period, so it is built with the shore layer (O4), not with the grid.

**As built (O1).** `cvc/world/water_bodies.h`: `build_water_bodies` from ground samples, and `generated_world::water` built in a `world_builder` stage (`shore`) from the terrain mesh's samples, roads graded in, at most 1025 a side. Native cost: 4 ms on the 1.2 km world, 70–105 ms on the 5 km and 10 km worlds (a 1001² grid); not yet measured in wasm. terrain_lab's debug view is the world panel's Colour by → Water bodies, and the panel lists the bodies (`world.stats.water`).

**W0.3 Wave content per body:**

| kind | ambient waves | shore layer | wake layer | physics |
|---|---|---|---|---|
| sea | swell + wind sea, FFT cascades (§1.1 + E1) | swell breakers, runup, reflection | yes | field with the FFT surface |
| lake | wind sea only, fetch-limited per body (E2); no swell | small lapping waves from the wind sea | yes | field with a fetch-limited surface |
| pond | ripples only (the shortest cascade, gust-modulated) | none beyond lapping | **dominant**: objects and rain make most of the motion | flat field plus ripples |
| stream | flow-map advection (water roadmap W2) | none | yes, advected | field with **velocity** from the flow grid: logs drift downstream |

JONSWAP growth is physically valid for lakes. Wave height grows as the square root of fetch [L Hasselmann 1973], and a small fetch makes a small, short-period sea, which is right for a lake. A pond's fetch is too short to raise anything but ripples, which is why `WATER-RENDERING-ROADMAP.md` rejects an FFT *ocean* there. Here a pond keeps only the ripple cascade, and the interactive layer does the visible work.

**W0.4 Physics: one water field for every body.** Phase B's surface hook (§5.3) reads `level(x, y)` from `water_bodies`. A single `cvc::physics` fluid field then covers the sea and every lake at its own level, which spares the world a field per pond. Stream velocity feeds `field::velocity`.

**Exit.** `water_bodies` is built for terrain_lab worlds (1.2 / 5 / 10 km), and its labelled components are drawn in a debug view. Unit tests cover:
- an island (one sea body);
- a hollow below the level (one lake);
- two separate hollows (two lakes);
- a channel joining a hollow to the sea (one sea body);
- inlets at opposite edges (one sea body).

---

## 4. Phase A — waves that react to the shore

The goal is to fix "the waves pass through the geometry". The ambient FFT cascades fade out as the water shoals, and a shore layer takes over in the shallows. That layer is analytic, so the CPU evaluates exactly what the GPU draws (R1, R3).

**A1. The seabed from the world model.**
- The ocean shader samples depth from the `water_bodies` level and a `HeightFieldTexture` of the ground (R4), retiring `coastH()`.
- In lsystem_coast the ground is `terrainH` sampled onto a grid. In terrain_lab it is the generated world's heightfield.

**A2. Damp the FFT cascades with depth.**
- A wave feels the bottom at d < λ/2 [L]. Scale cascade c by `w_c(d) = smoothstep(0, λ_c/2, d)`, with λ_c the cascade's peak wavelength. The long swell fades first, furthest out; ripples persist almost to the waterline.
- Sideways displacement is also faded to 0 within about 2 m of the waterline, so no vertex is pushed across it.
- On land (d ≤ 0) the FFT contributes nothing.

**As built (O3, lsystem_coast).**
- **The textures.** The seabed is a `HeightFieldTexture` of `terrainH` at 1 m over the ocean, and the shore grid is `water_bodies` uploaded as an RGBA32F texture (distance, normal, slope). `coastH()` is gone; the fragment shader's depth reads the seabed texture.
- **The fade is capped.** Each cascade fades over `min(λ_c/2, coast.water.shore_fade)`, with the knob at 3 m by default, where λ_c is the wavelength its band's energy peaks at. The island stands on a shelf about 9 m deep that runs out to the horizon, so the uncapped rule would fade the ~60 m peak waves over 30 m of depth and calm the whole visible sea. In reality the shallows slow and steepen waves rather than flattening them, and carrying that energy in is A3's job. A larger knob gives the uncapped rule once O4 lands.
- **A limit replaces the 2 m sideways fade.** A vertex moves sideways at most 0.8 of its distance to the shore, which guarantees no vertex crosses the waterline.
- **The check.** `lsystem_coast --shore-check` evaluates the vertex shader's arithmetic on the CPU, over the read-back cascades, for every offshore vertex. At t = 1.3 s the old shader moved 60 vertices onto dry ground, and this one none; at t = 5 s, 76 and none.

**A3. The shore layer.** This is a set of wave trains, driven by the **swell** (Hs, Tp, direction; E1) on the sea and by the wind sea on lakes.
- **Local wavenumber.** The frequency ω = 2π/Tp is fixed. The wavenumber comes from the finite-depth dispersion relation ω² = g·k·tanh(k·d), solved per texel with Eckart's explicit form: k ≈ (ω²/g) / sqrt(tanh(ω²d/g)), within about 5 % [L Eckart 1952].
- **Phase map P(x).** Solve the eikonal equation |∇P| = k(x) from the waterline (P = 0) outward, by fast marching on the CPU into the shore grid (W0.2). Level sets of P are crests. Because k grows as the water shoals, crests bend to run parallel to the shore: this is refraction, from the physics rather than painted on [P]. It is recomputed when the swell period or the terrain changes; it is a millisecond-scale solve at 256²–2048² [E].
- **Surface.** η = Σᵢ Aᵢ(x) · f(P(x)·ωᵢ/ω + ωᵢ·t + φᵢ), with 2–3 trains at slightly different periods so waves arrive in sets.
- **Amplitude.**
  - *Shoaling:* Green's law, A ∝ d^(−1/4) [L].
  - *Exposure:* `max(0, −n·d̂_swell)^p`. Shores facing away from the swell are in its shadow.
  - *Hand-off:* a fade in from the deep, the complement of A2.
  - *Breaking:* H = 2A is capped at γ_b·d with γ_b ≈ 0.78 [L McCowan]. The excess becomes breaking intensity, which feeds foam.
- **Profile.** Gerstner, with steepness rising toward breaking, so the front face steepens as the wave shoals.
- **Runup and swash.**
  - On the beach (d < 0) each wave's front runs up to R ≈ ξ·H (Hunt, valid for ξ ≲ 2.5 [L]) and drains back.
  - Swash is drawn **in the terrain's fragment shader** as a thin, glossy, foam-laced film over the sand inside the runup front. It is not geometry; decision **D4**.
  - A **wetness** texture records the time each texel was last wet, and the sand darkens and gains specular as `exp(−age / T_dry)`. Rain (Phase F) writes into the same texture.
- **Reflection.**
  - The Iribarren number ξ = tan β / sqrt(H/L₀), with L₀ = gT²/2π, sorts shores: beaches have small ξ and absorb; cliffs have large ξ and reflect.
  - Reflection coefficient K_r ≈ min(1, 0.1·ξ²) [L Battjes 1974].
  - A reflected train η_r = K_r · A · f(−P(x) + ω·t + φ_r) runs back offshore and fades over a few wavelengths. In front of a cliff, incoming plus reflected gives standing waves (clapotis). This is what "bounces off the shore" looks like.
- **Foam.** The fixed shore band is replaced by breaking-driven foam plus the swash lace, and it feeds the persistent foam texture (E-port, §10).

**As built (O4a: the waves; the runup film and wetness are O4b).** `cvc/ocean/shore.h`, `cvc::ocean::shore_layer`. It takes plain planes, so `cvc::ocean` still depends on no other library.
- **Where the phase starts.** The phase marches in from the grid edge the swell enters by, as a plane wave, not out from the waterline. On a shelf that reaches the horizon, starting at the waterline would ring the island with crests across the whole sea. Started at the edge, crests stay straight offshore and turn toward the shore where it shoals (Snell's law holds to 2 % in the tests), and they wrap round an island.
- **The wavenumber** is the exact dispersion relation (Newton from Eckart).
- **Shoaling** is sqrt(c_g0/c_g), which is Green's law in shallow water.
- **The breaking cap** applies to a set's crest: √N trains summed stay under γ_b·d.
- **The reflecting slope** is the mean slope from the waterline (d over the shore distance).
- **Exposure** fades back to the open swell two local wavelengths out.
- **What carries the swell.** In lsystem_coast the swell (`coast.water.swell_*`) is the shore layer's alone, and the cascades draw only the wind sea, because the island's shelf is shallow everywhere. A coast with deep water offshore would hand the swell from a cascade of its own to the shore layer.
- **Size.** `wave_amp` scales the cascades only, so the breaking cap holds at any setting.
- **Measured** (native): the build is 80 ms for a 441² grid. At a 1.2 m, 10 s swell, 1861 cells break and the steep island (tan β ≈ 0.3 at the waterline, ξ ≈ 3.4) reflects along most of its shore. The frame cost is within run-to-run noise. The browser is not yet measured.

**As built (O4b: runup, swash film, wetness).**
- **The runup field.** The shore layer holds a `runup` field: Hunt's R = min(ξ, 2.5)·H₀, as shadowed as the swell. It is carried with the phase `swash_reach_m` (30 m) up the dry beach, ring by ring from the waterline.
- **The swash.** `shore_layer::swash(x, y, t, ground_z)` has each wave's front run up to R(1 + cos θ)/2 on the first train's phase and drain back. It returns the film under the front, the foam lace at its edge, and the wetness.
- **No wetness texture: a deviation.** The wetness is exp(−age/T_dry), the age being the time since the front last left the ground. That age follows from the phase, so the swash needs no wetness texture and no per-frame ping-pong pass (WebGL2-friendly), and the CPU knows exactly where it is wet. Rain's wetness (F3) will need a real texture, and the swash can feed it then.
- **Drawing (D4).** lsystem_coast draws the swash in the terrain's fragment shader, after VTK's texture:
  - wet sand darkens and gains a glossy sky sheen at grazing angles;
  - the film tints it with water, with foam at the front;
  - `coast.water.dry_time` (6 s) sets the drying.
- **Measured.** At the default 1.2 m swell, runup reaches 3.0 m on the windward beaches and about 1000 beach cells lie within its reach, with the lee dry. At 2.5 m, runup reaches 6.25 m.

**A4. Shading pieces.**
- **Cloud shade on the water.** Add the water to `cloud_sky`'s shade receivers. Today it is not one [C `terrain_lab.ari:157`]; this is a one-line win in terrain_lab.
- **Shading upgrades.** Port ABYSSAL's GGX lobe, light through backlit crests (subsurface scattering), and roughness that follows the pixel footprint (§10, item 11). Do this before canonizing, while the shader is in one file.

**Tests.**
- A shader compile-and-link smoke test on desktop and on the GLES3 path, as `cvcgl_sway_shader` does.
- CPU unit tests:
  - a straight shore gives plane waves whose wavelength matches the dispersion relation;
  - amplitude follows Green's law;
  - the breaking cap holds;
  - a cliff gives K_r → 1 and a 1:50 beach gives K_r ≈ 0;
  - the eikonal phase stays smooth across the medial axis.

**Exit.** In lsystem_coast:
- crests turn to meet the beach, break and run up;
- the lee shore is quiet;
- the cliffs throw back a reflected train;
- no displaced vertex crosses the waterline.

Frame cost is measured native and in the browser.

---

## 5. Phase B — waves move bodies

**B1. A standard-library ocean model.** It lives in a library that physics, cvcGL, pycvc and arisrv can all link; where exactly is decision **D1**. It holds:
- the spectrum: JONSWAP with the TMA shallow-water correction, Donelan–Banner spreading, and the swell spectrum (E1);
- h0 generation from a **per-cascade** seed, **band-limited** per cascade (this pays the §1.1 debts);
- a deterministic radix-2 FFT;
- the shore layer (A3), evaluated over the shared grids.

`OceanFFT` uploads the model's h0 instead of running its h0 pass (R3).

**As built (O2).** D1 is settled as (a): `cvc::ocean`, a module of the core library that depends on the standard library only, enforced by a configure-time check.
- **`cvc/ocean/spectrum.h`.**
  - What it is: ABYSSAL's spectrum, ported. JONSWAP with TMA, Donelan–Banner blended with cos² (the cos² rotated with the wind, D10), the fetch law clamped at a fully developed sea, and the DNV swell term.
  - Also: `significant_wave_height`.
- **`cvc/ocean/h0.h`.**
  - `make_cascades` gives bands at 8π/L, which partition the wavenumbers, with a seed per cascade.
  - `build_h0` writes h0 in the texel layout the time evolution reads. A cascade's `amplitude` carries its artistic weight, so every consumer sees the same numbers.
- **The GPU side.**
  - `OceanFFT::setH0` replaces its spectrum shader pass and its noise texture.
  - lsystem_coast builds its three cascades this way. The swell is off by default (`coast.water.swell_hs` 0), and new sliders control swell height and period.
- **Cross-platform h0 (done before O6).** h0 was bit-identical across runs but not across platforms, since `std::log`, `exp`, `sin` and `cos` come from each platform's libm. `cvc::physics`'s private `det_math` moved to `cvc::algo` as `cvc/algo/det_math.h`, gaining exp, log, pow, sinh, cosh, acos and hypot. pow carries log x as a double-double and multiplies it by y exactly, so it stays within 3 ULP even where y log x is large.
  - `cvc::ocean`'s spectrum, h0 and shore layer call it in place of libm, and are built without FMA contraction.
  - `ocean_spectrum_test` and `ocean_shore_test` pin digests of the spectrum, h0 and the shore layer (its planes, evaluate() and swash()) measured on Windows; CI checks them on Linux and macOS.
  - The shore layer's fast marching pops (value, cell) pairs, which every standard library orders alike.
  - Physics keeps calling `det::` through a namespace alias, so its state hashes are unchanged.
- **The FFT (O7a).** `cvc/ocean/fft.h`: radix-2, its twiddles from `det::`, built without FMA contraction (B2 below).

**B2. The physics band.** Only waves longer than about twice a body's length move it much [P].
- The CPU evolves the lowest M×M modes of the swell and wind-sea cascades each physics step: M = 64 to start [E], each a 64² complex FFT.
- It packs height, two horizontal displacements and three velocity components into three complex FFTs.
- Cost estimate: under 1 ms per step [E].
- The cascades that only ripple the picture never reach physics.
- Record the RMS of GPU − CPU height in the harness as the "physics band residual", and pick M against it.

**B3. Height at a world point.** Sideways displacement moves surface points. The height at world (x, y) needs the source point x₀ with x₀ + D_h(x₀) = x, found by fixed-point iteration x₀ ← x − D_h(x₀). Arc Blanc uses 4 iterations [L Algis 2025].

**B4. Water velocity.** The orbital velocity is the time derivative of the displacement, computed in the spectrum (iω times each coefficient). It decays with depth as e^{kz}, or as cosh(k(z+d)) / sinh(kd) in finite depth. Arc Blanc samples 8 log-spaced depths and interpolates [L]. We start with the cascade's peak wavenumber [P].

**As built (O7a).** O7 is split in three: **O7a** is the sea a body feels (below), **O7b** the CPU-vs-GPU residual harness that chooses M, and **O7c** the debris in lsystem_coast.
- **`cvc/ocean/sea_surface.h`, `sea_surface`.** What lsystem_coast's water vertex shader draws, on the CPU, without the waves too short to move a body:
  - the cascades, each faded with the still depth;
  - the sideways move, capped at 0.8 of the way to the shore;
  - the shore layer, and on the beach the swash;
  - `wave_amp` and `chop` as the shader's uniforms.

  It is deterministic, cross-platform and safe to read from several threads; the fields for one time are computed once and shared.
- **B2.** Each cascade keeps its lowest `band.modes`² wavenumbers (|m| < modes/2, a set closed under k → −k, so the packed fields are real). It evolves them as OceanFFT does and transforms them onto a `band.grid`² lattice: height + i w, D_x + i D_y and u + i v, three complex FFTs. The lattice is read by Catmull-Rom splines.
  - h0 is band-limited per cascade, so lsystem_coast's 251 m and 83 m cascades have no modes past |m| = 12, and 32 modes keep them whole (`OceanSea.TheLongCascadesFitInTheBand`). Only the 29 m cascade loses waves to the band, those shorter than 1.8 m at 32 modes.
  - Measured cost for three cascades, MSVC Release: 0.64 ms per step at 32 modes on a 64 lattice, 0.83 ms at 64/64 and 3.2 ms at 64/128. O7a shipped 32/64; O7b chose 64/64 against the residual (below).
- **B3.** The fixed point x₀ ← x − D(x₀) contracts only by |∂D/∂x₀| a step. On a steep test sea (chop 1.5, the whole spectrum down to 6 m waves) its worst miss after 4 steps was 58 cm. `source()` uses Broyden's method instead: the same first step, one evaluation of D a step, and a secant Jacobian after that. Its worst miss is 3 cm after 4 steps, 2·10⁻⁵ m after 6 and 6·10⁻⁸ m after 8. The default stays at the roadmap's 4.
- **B4.**
  - The cascades' velocity is the time derivative of their fields at x₀, decaying as e^{decay_k z} below the still level, decay_k being the cascade's peak wavenumber, in deep water.
  - The shore layer's velocity is new in `shore_layer::velocity`: each train's linear finite-depth orbital motion in the local depth, so its vertical velocity at the surface is dη/dt.
  - With `bores`, a breaking crest also carries the water near the surface shoreward at the phase speed, scaled by its breaking (the surf zone's roller). Linear waves alone carry nothing over a period, and the bore is what carries floating things in.
  - On the beach the swash flows up and down the slope at the front's rise over the slope (`swash_sample::rate`), capped at √(2gR).
- **`cvc/physics/ocean_surface.h`.** `ocean_surface` puts a `sea_surface` behind `fluid_surface`. A slab rides the open sea, the 251 m cascade under a 9 m/s wind, in step with the surface at its centre (correlation > 0.95), about as high and at half draft (`PhysicsOceanSurface.ABoxRidesTheSea`).
- **Pins** measured on Windows, checked by CI on Linux and macOS:
  - `OceanSea.IsTheSameOnEveryPlatform`: three cascades and a beach with its shore layer, as heights, velocities and a band;
  - `PhysicsOceanSurface.CrossPlatformGoldenHash`: the ocean scene's state hash.
  - The macOS arm64 lanes first ran them after the merge and found the test's own sample points contracted into FMAs. #1146 builds `ocean_sea_test` without contraction, as cvc::ocean is built.

**As built (O7b): the physics band residual, and M chosen.** `cvcgl_ocean_band` runs lsystem_coast's three cascades on the GPU (OceanFFT at 256², read back) and compares them with `sea_surface` on the CPU.
- **Two OceanFFT bugs it found first.**
  - **The scissor box.** The renderer leaves the scissor test on, boxed to the window, and OceanFFT's passes did not turn it off. In a window smaller than the FFT (the harness's 64-pixel one, or a small canvas) every pass was clipped to the window's corner and the rest of each texture stayed stale. The long cascades' modes sit near texel 128, so those cascades came back flat. OceanFFT now disables the scissor test for its passes and restores it afterwards.
  - **The Nyquist modes.** The Nyquist row and column (k = −N/2) have no −k partner on the grid, so their fields are not real. The packing, two real fields to one complex transform, leaked the slope field into the height there, scaled by k_x up to the Nyquist: a faint checkerboard in the short cascade. OceanFFT now drops them, as the CPU's band does.
- **The waves the band drops, on the GPU's own lattice** (height RMS of GPU − CPU over the RMS height, at t = 7.5 and 37.25 s). With every mode kept, the CPU's twin is the GPU to 0.0005 % (float arithmetic), so the CPU computes what the GPU draws.

  | cascade | height RMS | M = 8 | 16 | 32 | 64 | 128 | 256 |
  |---|---|---|---|---|---|---|---|
  | 251 m | 1.17 m | 75 % | 30 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % |
  | 83 m | 0.36 m | 97 % | 37 % | 0.00 % | 0.00 % | 0.00 % | 0.00 % |
  | 29 m | 0.14 m | 99 % | 42 % | 21 % | 10 % | 4.5 % | 0.00 % |
- **The drawn sum.** The three cascades are sampled where the shader samples them and compared with `sea_surface::height` over 211² points of the 251 m tile, at t = 37.25 s (RMS height 1.245 m).

  | band | per cascade, 251 / 83 / 29 m (splines) | drawn sum | drawn, as sampled before O7b |
  |---|---|---|---|
  | 32 on 64 | 0.37 / 0.42 / 20 % | 4.41 % (55 mm) | 8.72 % (109 mm) |
  | 32 on 128 | 0.00 / 0.00 / 20 % | 4.39 % (55 mm) | 8.71 % |
  | **64 on 64** | 0.37 / 0.42 / 11 % | **3.99 % (50 mm)** | 8.54 % |
  | 64 on 128 | 0.00 / 0.00 / 9.9 % | 3.93 % (49 mm) | 8.52 % |
  | every mode | 0 / 0 / 0 % | 3.78 % (47 mm) | 8.50 % (106 mm) |

  - **The floor.** The band with every mode still misses by 3.78 %. That is the floor set by the shader's own nearest-texel steps (the texels are 1 m on the 251 m tile; WebGL2 filters no float texture). No band can go below it.
  - **The shader's sampling, fixed.** The shader used to take the texel at or before each point, so its steps trailed the field by half a texel and doubled that floor, to 106 mm. lsystem_coast's `oDisp` now takes the texel nearest the point (half a texel over), and the shore check samples the same way. The picture moves by a fraction of a metre and still crosses no waterline (`--shore-check`: 0 vertices onto dry ground).
- **M chosen: 64 modes on a 64 lattice**, the roadmap's first estimate. It misses the drawn sea by 3.99 %, within 6 % of the floor. 32 modes miss by 4.41 %, 17 % over the floor, for 0.19 ms less per step. A 128 lattice gains 0.06 points for four times the cost. `band_params` defaults to 64/64, and the harness fails if the default misses by more than 1.1 times the floor.
- **For a body**, what the band drops is the 29 m cascade's waves shorter than 0.9 m (10 % of that cascade's height). That is well below a crate's length, and the drawn steps are a larger error than anything the band leaves out.

**B5. The field surface hook** in `cvc::physics::field` (sketch, decision **D2**):

```cpp
struct fluid_surface {                       // standard library only, deterministic
  virtual ~fluid_surface() = default;
  virtual double level(double x, double y) const = 0;           // still water (water_bodies)
  virtual double height(double x, double y, double t) const = 0; // level + waves + shore layer
  virtual vec3 velocity(const vec3 &p, double t) const = 0;      // orbital + current/stream
  virtual std::uint64_t digest() const = 0;                      // folded into state_hash
};
// field::surface: std::shared_ptr<const fluid_surface>; null keeps today's flat box top.
```

In `field_set::apply` a cell clips z to `min(box top, height(cell x, y, t))`, and drag uses `velocity` in place of the volume velocity. Nothing else in the field loop changes [C `field_step.cpp:551-597`].

- **The time base must be pinned.** The surface is a function of world-clock time. `world::follow` anchors at the clock's tick on its first call [C `world.h:321-327`], so `world::time()` is not clock time. The field needs the anchored clock time `t_anchor + (tick − tick_anchor)·dt`. Check this before writing B5.
- **Shared immersion code.** The cell-immersion code (shape and pose against the surface) is shared with Phase C's wake sources (C2), so physics and the renderer agree on what is underwater.

**As built (O6a).** D2 is settled as (a).
- **`cvc/physics/fluid_surface.h`.** The interface is the sketch's `level`, `height`, `velocity` and `digest`. Two surfaces come with it:
  - `still_surface`: a level, with an optional uniform current;
  - `wave_train_surface`: linear Airy trains with finite-depth dispersion and cosh/sinh decay, computed with `det::` so they are the same on every platform.

  The cvc::ocean-backed sea (the physics band and the shore layer) is O7's, as in §12.
- **`field::surface`** is allowed on a fluid only. In `field_set::apply` a cell's top clips to `min(box top, height(x, y, t))` over its clipped centre, and drag and damping use `velocity()` in place of the velocity volumes. Nothing else in the loop changed.
- **The time base is pinned.** `world::field_time()` is `t0 + (tick − tick0)·dt`:
  - `follow()`'s first call sets `t0` to the clock's `t()`, so the field time is clock time;
  - `set_field_time(t)` sets it explicitly;
  - before either, it is `time()`.
- **The surface is world state.** With a free surface, `save()` appends a section holding the anchor and each surface's digest, so `state_hash()` covers both, and `restore()` refuses a world whose surfaces differ. Without one, the format is unchanged, so every existing hash and the replay trace are untouched.
- **B6's tests** are in `physics_field_test` (`PhysicsFluidSurface.*`). The CPU-vs-GPU residual waits for O7's physics band, and the native-vs-wasm check rides on `physics_replay_test`'s lane.
  - A box at density 500 floats at half draft on calm water at two levels.
  - In a 0.2 m, 4 s swell it heaves in phase with η at its centre (correlation > 0.97), about as high, at the swell's period.
  - Runs with one surface hash alike, and a different amplitude or field time changes the hash.
  - A save restores the field time and refuses another surface.

**As built (O6b).** pycvc binds the fields, the surfaces and the time base (`pycvc_physics.i`; `docs/PHYSICS_API.md`, "Python: pycvc"):
- `physics_world.add_field` / `set_field` take a `pycvc.volume`, the field's values and a `surface`;
- `still_surface` and `wave_train_surface` are C++'s classes, shared with the field;
- `follow(clock)`, `field_time()` and `set_field_time()` are the time base;
- `field_load_on` and `physics_binding.publish_field` complete the field API.

The swell scene's state hash is pinned in `PhysicsFluidSurface.CrossPlatformGoldenHash` (`f0085b4212e1a8c0`). `test_pycvc_physics.py` builds the same scene through pycvc and must get that value. It also checks the surfaces against Airy's formulas, the field time against a clock, save / restore with a surface, and a tree-set damping that keeps the surface.

**B6. Tests.**
- A box at density 500 settles at half draft on calm water.
- In swell it heaves at the swell period, in phase with η at its centre.
- CPU vs GPU: read back `OceanFFT` displacement in the native harness [C `OceanFFT.h:readbackDisplacement`] and bound the residual.
- `state_hash` agrees across two runs, and between native and wasm.
- A surface's digest changes when a spectrum knob changes.

**Exit.** Crates and barrels dropped in lsystem_coast ride the swell, get carried by the swash, and settle on calm lake water at the lake's own level.

---

## 6. Phase C — bodies make waves

**C1. The solver.** The choice decides whether a boat's wake looks right. Deep-water waves are dispersive, so a moving body leaves a Kelvin wake whose half-angle is a fixed 19.47° = arcsin(1/3) at any speed [L]. A non-dispersive wave equation instead gives a Mach cone whose angle depends on speed.

| solver | dispersion | obstacles | per step at N = 256 | WebGL2 | verdict |
|---|---|---|---|---|---|
| explicit finite-difference wave equation (Gomez 2000; Arc Blanc's wakes) | none: one speed, CFL-limited | Dirichlet mask | 1 pass | yes | simplest; wrong Kelvin angle |
| iWave (Tessendorf 2004) | approximately deep water, through a 13×13 kernel | `h = 0` on masked cells | 1 pass, 169 taps | yes | proven in games; waves travel too slowly [L Schreck 2019] |
| **eWave** (Tessendorf 2014) | **exact** for any ω(k), including one finite depth per window | mask applied in real space between transforms | 1 forward + 1 inverse complex FFT ≈ 32 butterfly passes + 2 | yes, **reuses OceanFFT's butterflies** | **recommended** |
| wave packets / water surface wavelets (Jeschke 2017/2018) | exact, and also reflects the *ambient* waves | true boundaries | many passes (an amplitude per direction) | probably | the unified long-term answer; research port; code licence unclear |

**eWave update** [P, from linearized potential flow, consistent with L Tessendorf 2014]. The state is the surface height h and the surface velocity potential φ. Let κ = |k|·tanh(|k|·d) and ω = sqrt(g·κ). Then per mode:

```
ĥ' = ĥ·cos(ωΔt) + (κ/ω)·φ̂·sin(ωΔt)
φ̂' = φ̂·cos(ωΔt) − (g/ω)·ĥ·sin(ωΔt)
```

- The step is exact for any Δt, so there is no CFL limit.
- h and φ are real, so they pack into one complex field z = h + iφ. The update pass separates them through Hermitian symmetry, reading texels k and −k. That makes **one forward and one inverse FFT per step**. The forward transform is OceanFFT's butterfly with conjugated twiddles.
- In real space between steps:
  - `h, φ ← mask·(h, φ)`: 0 on land and obstacles, 1 in open water. Beaches ramp toward 0 across the swash zone so they absorb, while cliffs are a hard 0 so they reflect (K_r from A3).
  - A sponge band at the window edge absorbs outgoing waves and hides the transform's wrap-around.
  - Then sources are added.
- Damping is e^(−γ·k²·Δt) on high wavenumbers plus a small global rate.
- Finite depth: one ω(k) per window, using the window's mean depth, plus extra damping in the shallows [P]. Running a deep and a shallow solve and blending them by local depth doubles the cost; that is a later option.

**C2. Sources from bodies.**
- Each step, each immersed body's waterline footprint is rasterized into the window as columns of immersed depth ζ(x, y).
- The height source is Δh = −(ζ(t) − ζ(t−Δt)), splatted with a smooth kernel. This conserves volume: a dropped crate makes a ring, and a moving hull displaces water at the bow and releases it at the stern. With exact dispersion, bow waves, stern waves and the Kelvin pattern all follow without more code [L].
- **ζ is computed on the client** from the body's replicated pose, its shape, and the shared ocean model (R2), using the same immersion code as physics (B5). That keeps it working under arisrv, where a client has no physics world.

**C3. The window.**
- N = 256 at Δx = 0.5 m gives 128 m, which resolves the 16 m transverse wake of a 10 m hull at 5 m/s (λ = 2πv²/g) [P].
- It is centred on the camera or on the active boat (decision **D3**) and snapped to cells. A move shifts the texture offset, as Arc Blanc's index shift does, and the sponge hides the strip that enters.
- Fallback for WebGL2 budgets: 256² at 1 m.

**C4. Drawing.**
- The wake height is added in the water's vertex stage where there are vertices enough, and always in the fragment normal.
- Foam comes from the wake's curvature and vertical speed, accumulated into the persistent foam texture (§10, item 2), so a boat leaves a trail.
- A trail longer than the window is a later ribbon decal.

**C5. Bodies are not pushed by wakes**, by R1: the GPU layer is visual. An opt-in CPU eWave at 64², deterministic, inside the B1 library could add wake-on-body coupling later (decision **D6**).

**C6. Lakes and ponds.** The window masks against `water_bodies`, so a pond inside the window reflects off all its banks. Rain adds ripple sources (Phase F).

**Cost.** OceanFFT's 3 cascades are 57 passes per frame at 256² [from the port notes]. eWave adds about 34 more at 256², roughly +60 % of the FFT fill [E]. Not measured; Arc Blanc reports 0.34 ms of GPU time per solid for its finite-difference wakes on an RTX 2070S [L].

**Exit.** A crate dropped near the beach sends a ring that reflects off the sand. A moving box hull leaves a wake at about 19.5°. A rock in the water casts a wave shadow.

---

## 7. Phase D — boats

**D1. Forces at a point.**
- Add `world::add_force_at(id, force, point)` and `add_torque(id, torque)`: validate the values are finite, document them, and bind them in pycvc. The backend path exists [C `jolt_backend.cpp:953-954`].

**As built (O10).** `world::add_force_at(id, force, point)` and `world::add_torque(id, torque)` (`world.h`; `docs/PHYSICS_API.md`, "Bodies and time"):
- **Frames and units.** The force (N), the point (m) and the torque (N·m) are all world-frame; the point is in world coordinates like `body_state::position`, and the Jolt backend moves it to the engine's floating-origin offsets (`to_engine`). Off the centre of mass the force also torques the body by (point − com) × force.
- **Same as `add_force` otherwise.** Both act through the next step only and add up with each other and with `add_force`. They wake a sleeping dynamic body (Jolt's `AddForce(id, f, point)` / `AddTorque` with the default activation, as `add_force` uses), do nothing to a static or kinematic body, and throw `cvc::physics_error` for a non-finite value or an unknown id. The null backend checks the id and does nothing else. They are new `backend_impl` calls rather than a reuse of `add_field_load`, which never wakes a body. Nothing records or serialises world calls, so no log or codec changed.
- **pycvc** binds both on `physics_world` (`add_force_at(id, force, point)`, `add_torque(id, torque)`), and `test_pycvc_physics.py`'s `test_forces_at_a_point` checks them against r × F dt / I and their refusals.
- **Tests** are in `physics_world_test`. Measured on a free 125 kg, 1 × 0.5 × 0.25 m box (no gravity, no damping) 3 m from a floating origin at (1000, −500, 0):
  - `PhysicsJolt.AnOffCentreForceSpinsTheBody`: 100 N up at the +x end and 40 N sideways at the top give ω = (−0.015360, −0.045176, −0.0000005) rad/s after one 0.01 s step; r × F dt / I gives (−0.015360, −0.045176, 0). The small z term is the second collision sub-step seeing the slightly turned body's inertia: with `collision_steps = 1` it is 7·10⁻¹⁰, with 4 it is −6.5·10⁻⁷. The centre moves at ΣF dt / m.
  - `PhysicsJolt.AForceAtTheCentreOfMassIsAddForce`: 120 steps of pushes on a spinning body through `add_force_at` at its position give `add_force`'s state hash exactly.
  - `PhysicsJolt.ATorqueSpinsWithoutTranslating`: ω = T dt / I within 0.1 % on each axis, and after 100 more steps the linear velocity is exactly zero and the centre has not moved.
  - `PhysicsJolt.ForceAtAndTorqueWakeASleepingBody` and `PhysicsWorld.ForceAtAndTorqueAreChecked` (null and Jolt backends: non-finite values, a removed id and the null id throw; a static body and a kinematic body take them and do not move or gain velocity).
  - `PhysicsJolt.AForceAtAHullsOriginIsOffItsCentreOfMass`: `body_state::position` is the body's origin, not its centre of mass. A hull cube with its origin at a corner spins under a force at its position, and does not under the same force at its centroid. A hull model must apply its forces about the hull's centroid (or the vehicle chassis's `center_of_mass`), not about `position`.

**D2. The hull model.** Forces per triangle against the B surface [L Kerner; Algis 2025].
- Triangles are split at the waterline.
- Hydrostatic pressure is ρ·g·depth·area along the normal.
- Viscous resistance is ½·ρ·C_f(Re)·S·v².
- Pressure drag grows with (v·n)².
- Slamming acts on triangles entering the water.

It is a new outline kind in `field_set` that carries a hull triangle list, separate from the collision shape. It is standard library only, in a fixed order, so it stays deterministic. The 8-cells-per-side model stays for crates and debris.

**D3. Propulsion and control.**
- Thrust is applied at the propeller.
- The rudder's lift is ½·ρ·A·C_L(δ)·v², applied at the rudder.
- An outboard motor vectors its thrust.
- Throttle and rudder are actuators like `vehicle_controls`.

**D4. A boat plant.** `dyn_boat_plant` sits beside the 4-wheel and bike plants, takes a nav command through a driver model, is selectable by `plant_host`, and keeps the same energy ledger. This puts watercraft in GRL-SNAM's reach.

**D5. Drawing.**
- **No water inside the hull.** The water shader discards inside the hull's waterline footprint, passed as a small signed-distance texture or polygon uniform. Without this, the sea shows through the deck.
- Bow spray uses the spray particles (§10, item 5), spawned where the wake breaks.
- Propeller wash goes into the foam texture.
- The hull darkens where it is wet.

**D6. An Ariadne node,** `boat` (or `vessel`): a hull mesh, plant parameters, and a body in the scene's physics world.

**Exit.** A drivable boat in lsystem_coast that:
- pitches over the swell;
- leaves a Kelvin wake with a foam trail;
- throws spray at speed;
- rocks in the swell reflected off the cliffs.

---

## 8. Phase E — wind and sea state

**E1. Separate the swell from the wind sea** (port, [X] §10, item 1).
- **Swell** is set by Hs, Tp, direction, γ and a cutoff, and is narrow-banded. It comes from distant weather, and it is what breaks on the beach (A3).
- **Wind sea** is set by U₁₀, fetch and direction. It makes the local chop, whitecaps and spray.
- With the two separate, onshore and offshore wind finally look different. Offshore wind holds the breakers' faces up and blows spray back off the crests.

**E2. Fetch and lee.**
- For each water texel, march upwind across `water_bodies` until land, capped at the body's open fetch. Recompute when the wind direction changes, debounced.
- Scale the wind-sea cascades by `sqrt(F_local / F_spectrum)`, clamped. Height grows as the square root of fetch [L], so the island's lee goes calm and a small lake stays small.
- This is an amplitude scaling only; a shorter fetch also shortens the peak period, which this ignores [P]. It is probably the most visible wind effect on an island.

**E3. Gusts.**
- A low-frequency noise drifts with the wind and modulates the ripple cascade's weight and the shading roughness, giving cat's paws: dark, ruffled patches racing across calm water.
- It is the same field the trees sway in, and ABYSSAL's `gustiness` sets its strength (§10, item 4).

**E4. The sea lags the wind.**
- U₁₀ ramps toward its target with a time constant. Duration-limited growth takes hours; a compressed scale is a knob.
- h0 crossfades from the old spectrum to the new one over seconds, instead of today's single-frame respectralize.

**E5. One scene wind** (R5). `wind.speed`, `wind.dir` and `wind.gustiness`, plus the storm modulation (Phase F), drive:
- tree sway;
- `cloud_sky`'s drift, replacing its separate `wind_az` prop;
- both wave spectra;
- spray and rain slant;
- a `cvc::physics` **flow** field, so light floating bodies drift and sails become possible [C `field.h:83-87`].

**E6. Currents** (port, [X] §10, item 3).
- A slowly varying curl-noise flow advects the small cascades, which breaks up their tiling.
- The same flow, plus stream velocities from `water_bodies`, feeds `field::velocity`, so a drifting body moves the way the drawn water does.

---

## 9. Phase F — dark rain clouds, rain and storms

This phase builds on terrain_lab's `cloud_sky` (§1.5), not on the older demo clouds.

**F1. Make the sky live.**
- Promote `coverage`, `base` / `top`, the sun, `wind_az` and a new `darkness` from realize-time props to live keys.
- Coverage is already a target count for the cellular automaton (`rank_cells`), so it can change live without a rebuild.
- Read the sun from the light rig (`lighting.*`) instead of `sun_az` / `sun_el` props. The sun baked into the maps is rebuilt when it changes.

**F2. The weather model.** It is standard library only and deterministic: a function of the seed and world time (R2).
- **Weather state:** clear, building, storm, clearing. Transitions are scheduled or authored, in the style of ABYSSAL's Director (§10, item 7).
- **Storm cells:** each has a centre that moves with the steering wind, a radius, and an intensity that runs through growth, maturity and decay over a compressed 20–60 minutes.
- **What each cell drives, through hooks that exist:**
  - **Clouds:** `cloud_layer`'s `forcing` (the orographic hook) raises coverage and density in the cell's footprint.
  - **Darkness:** the cell's intensity raises it. A deeper deck has more optical depth toward the sun, so its base goes dark grey through the same `exp(−k·tau)` ramp [C `lsystem_nodes.cpp:585`]. That is the physical reason storm clouds are dark, and the ramp needs only a darker low end.
  - **Height:** a storm deck needs tops of several kilometres, against today's 300–520 m. Taller cloud maps for cumulonimbus are a variant of the existing 60×60×28 maps. The far shells already reach 4 km.

**F3. Rain.** Under each cell, a rain rate R(x, y, t) wherever the cloud is thick enough. It shows in five places:
- **Near the camera:** streak particles in a GPU ping-pong state texture, lit by the sun's transmittance through the deck, slanted by the wind, with density ∝ R. This is a port (§10, item 6).
- **At a distance:** rain shafts. The far shells draw a translucent curtain below the cloud base wherever R > 0, so a storm reads from kilometres away.
- **On the ground:** the wetness texture (A3), with ground darker and glossier, and puddles in hollows and on the `puddle` / `wet_floor` surface classes the ontology already has [C `surface.cpp:132`, `:155`].
- **On every water body:** animated raindrop rings in the water's normal (a ripple texture, density ∝ R). Ponds benefit most. Optional: rain as wake-layer sources inside the window.
- **In the light:** the sun is dimmed by the deck's transmittance (the shade mask already computes it), the ambient light drops, the gradient sky darkens toward the storm, and a **distance fog / haze** thickens in rain. Fog is new: none exists today (§1.5).

**F4. Wind in a storm.**
- A gust front runs ahead of each cell. Wind speed rises and veers (E5), the wind sea builds after its lag (E4), whitecaps and spray increase, and trees bend harder.
- The physics flow field gusts.

**F5. Lightning.** Port ABYSSAL's branching bolts and flash lighting (§10, item 7): a transient light and an in-cloud flash, scheduled by the weather model so every client sees the same strike at the same time.

**F6. Weather into physics.**
- Rain lowers road and ground friction through `world::set_material` [C `world.h:224`], so vehicles and nav feel the storm.
- Wet surface classes change traction.
- Gusts push light bodies.
- Later, once W1 exists: rain raises lake levels and makes streams run.

**F7. Replication.**
- The weather is a function of the seed and time, plus authored events under state keys (`weather.*`). Clients rebuild it; a server streams only the events (decision **D5**).

**WebGL2.**
- The deck textures stay R8 [C `cloud_layer.cpp:42-43`].
- Rain particle state needs one float render-target pair, which OceanFFT already needs.
- The two-volume frame-rate ceiling [C `LSYSTEM-LABORATORY-ROADMAP.md:145`] argues for drawing rain shafts in the existing shells, not in new volumes.

**Exit.** In terrain_lab:
- a storm cell builds upwind, darkens the sky and the ground, walks across the world raining (shafts in the distance, streaks up close, rings on the lakes, wet roads), and clears;
- the wind and the sea build and settle behind it;
- a vehicle's traction drops while the ground is wet.

---

## 10. ABYSSAL features to port

ABYSSAL's water code has not changed since our port. Its one later code commit, `849ff7f4`, makes the adaptive quality loop recover faster on slow hardware. It has **no** shoreline, terrain, buoyancy or object-interaction code; its comment about "buoyancy probes" refers to debug readbacks.

| # | feature | worth it? | phase | notes |
|---|---|---|---|---|
| 1 | swell / wind-sea split | yes, needed | E1 (A3 uses the swell) | §10.1 |
| 2 | persistent foam, bubbles, crest and spray channels | yes, needed | A3, C4 | §10.2 |
| 3 | curl-noise currents | yes | E6 (to physics) | §10.3 |
| 4 | gustiness | yes | E3 | §10.4 |
| 5 | wind-blown spray particles | yes | D5, F4 | §10.5 |
| 6 | rain | yes | F3 | §10.6 |
| 7 | storm clouds, lightning, weather director | yes, adapted to `cloud_sky` | F2, F5 | §10.7 |
| 8 | waterspout | later | F (after F5) | §10.8 |
| 9 | tsunami, rogue wave, whirlpool, hurricane | later | H (§12) | a tsunami needs A3's shore layer to run up the beach; hurricane ties to F2 |
| 10 | adaptive quality | yes, for all of cvcGL | G | §10.10 |
| 11 | GGX, light through crests, footprint roughness; projected grid with earth curvature; atmosphere lookup tables | shading yes (A4); projected grid for large worlds (§11); atmosphere later | A4 / §11 / later | §10.11 |

ABYSSAL as a whole needs WebGL2 plus `EXT_color_buffer_float`. Simulation state is RGBA32F; outputs, clouds and the env map are RGBA16F. At most 3 render targets are bound at once [X `App.js:45-49`].

### 10.1 Swell / wind-sea split [X `OceanFFT.js:94-171`, `:570-639`]

- **Two spectra per cascade.** Each cascade's h0 pass sums two spectra, `S = spectrumAt(S0) + spectrumAt(S1)`, clipped to the cascade's wavenumber band. The band edges are b₁ = 8π/389 and b₂ = 8π/41.3, from tiles of 4099 / 389 / 41.3 m.
- **Spectrum.** `spectrumAt = scale · JONSWAP(ω) · D(θ) · exp(−fade²k²) · |dω/dk| / k`, with the TMA correction.
- **Spreading.** D mixes cos²θ with Donelan–Banner cos^{2s}. The swell term adds 16·tanh(ω/ω_p)·swell² to s, which narrows it.
  - **Quirk not to copy:** the cos²θ part is not rotated by the wind angle [X `:116-145`].
- **Wind sea.** F = clamp(220 km, 400, 2.2e4·u²/g), α = 0.076·(u²/Fg)^0.22, ω_p = max(22·(g²/uF)^⅓, 0.30).
- **Swell.** By DNV-RP-C205: α_s = 5.061·Hs²/Tp⁴·(1 − 0.287·ln γ), with γ = 6. The cutoff of 14 is used as a fade length, so the swell lives in cascade 0.
- **Uniforms.** Four vec4s pack both spectra [X `:618-623`].
- **Port.** Add the second spectrum term to our h0, which becomes a CPU computation under R3, and band-limit the cascades (the §1.1 debts). No new textures.
  - ABYSSAL rebuilds every frame while gusting; we crossfade instead (E4).
  - Its Director sets Hs 1.5–11 m and Tp 10.5–14 s, which are useful presets for the sea.

### 10.2 Persistent foam and its channels [X `OceanFFT.js:261-343`, `:441-492`; `OceanMesh.js:571-613`, `:683-735`]

- **Targets.** The assemble pass is MRT3 (RGBA16F). The turbulence target reads the previous frame and keeps four channels: foam, bubbles, crest, spray.
- **Breaking test.**
  - `fold = smoothstep(bias, bias−0.3, J)`;
  - `steep = smoothstep(sb, sb+0.3, |∇Dy|)`, times a leeward factor and an above-mean-level factor;
  - `f = max(fold, steep)`.
- **Integration.**
  - `foam = max(prev·e^(−dt·decay) + f·mul·dt − 0.015·dt, 0)`;
  - bubbles decay more slowly;
  - spray latches at `e^(−6dt)`.
- **Per-cascade scale.** FOAM_SCALE is [0.55, 1, 0.22]. Weather drives bias, mul and decay from U/30. Whitecap coverage follows Monahan: W = min(3.84e-6·U^3.41, 0.16).
- **In the surface material.** The foam mask is eroded by a baked 2048² foam texture, stretched in the wind's frame into windrows. Bubbles brighten the water body.
- **Port.** Add a third attachment, and keep the previous turbulence by **ping-ponging two FBOs** (ABYSSAL copies, one extra pass per cascade).
  - ABYSSAL does not advect foam (it stays world-tiled). Our wake and shore foam need a *window-space* foam texture of their own, advected by the current (E6) and decaying the same way.

### 10.3 Currents [X `OceanSampleGLSL.js:31-41`, `:227-230`; `ProceduralTextures.js:170-185`]

- **Texture.** A 256² RGBA8 texture, baked once: the curl of tiled fBm, **normalized**, so RG holds a direction, not a speed.
- **Flow.** `currentAt(p, t)` is two drifting taps. `warpCoord = p + currentAt·26 m` is a domain warp of every cascade lookup, in both the vertex and fragment stages. It is not advection, so displacement and foam stay registered.
- **Port.** Trivial: one bake and two taps per lookup.
  - For physics (E6) the warp is not a velocity. The current that feeds `field::velocity` is a separate, physical speed field (m/s) that the warp's direction follows, so what drifts in physics matches what is drawn [P].

### 10.4 Gustiness [X `Weather.js:107-147`; `Precipitation.js:69`]

- `gust = 1 + g·(0.5·sin φ + 0.3·sin(2.37φ + 1.1) + 0.2·sin 5.1φ)`, with φ = 0.21·t.
- It multiplies the wind that feeds the FFT (a rebuild every frame), the micro-roughness, spray, rain and whitecaps. Rain drops each get their own phase.
- **Port.** CPU only. Ours drives E3's spatial gust field and the scene wind (E5). It does not rebuild h0 every frame: under R3 h0 is a CPU rebuild, so gusts reach the waves only through E4's lagged sea state and the ripple weight.

### 10.5 Spray particles [X `Precipitation.js:267-587`]

- **State.** A ping-pong MRT2 RGBA32F holds position/age and velocity/seed. The count runs from 77² to 387² by quality tier. One simulation pass, then one instanced quad draw whose vertex stage reads the state.
- **Living particles.** `V += ((wind − V)·drag − g·ŷ)·dt`. A particle dies below the sea surface or beyond 1.35·R (R = 420 m).
- **Spawning.** A dead particle tries 3 candidates a frame at radius R·r^1.55. It spawns where breaking (the turbulence spray channel, or an analytic crest) beats a random draw, with a velocity that is part wind and part an upward kick of (3.5 + 9·breaking) m/s.
- **Drawing.** Billboards stretched by speed, with forward scattering (0.55 + 1.9·μ⁶).
- **Two latent bugs not to copy:**
  - The simulation's uniforms lack `uCascadeGain`, so the FFT reads as flat and spray spawns only on the analytic events.
  - Turbulence is sampled with `texture()` on a mipmapped target. Use `textureLod(…, 0)`.
- **Port.** Spawn sources:
  - breaking crests (wind sea);
  - A3's breakers (surf spray);
  - **the boat's bow and the wake's breaking** (D5).

### 10.6 Rain [X `Precipitation.js:6-265`; `OceanMesh.js:526-543`]

- **Streaks.** Stateless instanced streaks, 9k–180k, in an 80 m box that follows the camera. y wraps at 4.2–9.4 m/s with gust drift.
- **Width.** True width is 6–16 mm, clamped to at least 1.15 px with alpha compensating, so streaks never alias away.
- **Wind curtains.** An fBm gate drifting at 9 m/s.
- **Light.** Each streak is lit like a lens by the env map, plus a sun glint and lightning.
- **On the sea.** Procedural impact rings in a 0.45 m hash grid perturb the water's normal. There are no splash particles.
- **Not coupled to clouds.** `uRain` is one global scalar, tied to the clouds only through presets. `uFogDensity` and PostFX's rain uniforms are dead code.
- **Port.** The easiest item: no render targets.
  - Ours is spatial: the streak density and the ring density follow F3's rain rate under the storm cells, rather than one global scalar.
  - Ours lights streaks from the gradient sky and the deck's transmittance until an env map exists.

### 10.7 Storm clouds, lightning and the director [X `Clouds.js`, `Lightning.js`, `Weather.js`, `Director.js`]

**Clouds.** ABYSSAL ray-marches a spherical shell.
- It uses a 1024² weather map: synoptic ridged bands, mesoscale cells, cloud type, and convective cores.
- A height profile runs from stratus through cumulus to cumulonimbus with an anvil (`smoothstep(0.58, 0.74, h)` aloft, spreading to 1.8·cov + 0.24).
- Its shapes are Perlin–Worley 128³ + 32³.
- **Dark storm bases** come from three terms [X `Clouds.js:496-506`]:
  - Beer extinction through a thick, dense deck;
  - split ambient, with the top lit by sky-up ×1.45 and the bottom by 0.5·side + 0.15·up;
  - two upward occlusion taps, `skyVis = exp(−above·σ·0.55)`, scaling ambient by `mix(0.16, 1, skyVis)`.

**Lightning.**
- CPU midpoint-displacement bolts: depth ≤ 5, fork probability 0.42, plus in-cloud crawlers.
- 1–3 return strokes, drawn as additive instanced segments.
- Sheet flashes at a Poisson rate.
- It publishes the two strongest bolts and an ambient flash, which the ocean, clouds, sky, rain and spray all read.

**Weather and Director.**
- Each value is damped toward its target with τ ≈ 0.5–4 s.
- The Director plays 11 scripted acts in a 352 s loop (e.g. calm U 3.4 → violent storm U 32, Hs 8, rain 1, sun at 12°).
- It is all preset tables, with no physical coupling.

**Port, adapted to `cloud_sky`; we do not port the renderer.** Our deck already exists, is R8-cheap, and is lit by optical depth (§1.5).
- **Dark bases.** Add an upward optical depth beside `cell_sun_tau` (a `cell_up_tau` built the same way as `build_sun`), and darken ambient by `mix(0.16, 1, exp(−0.55·σ·τ_up))` at each voxel. That is ABYSSAL's term on our grid. It also corrects the ramp the code turned off because it "reads as storm cloud" [C `lsystem_nodes.cpp:1032`]: under a thin deck it now stays light.
- **The weather map's ideas** (bands, cells, convective cores, type) feed `cloud_layer::forcing` and the storm cells (F2).
- **Lightning:** port it.
- **Weather values:** use the damped-target pattern, driven by our weather model, not by tables.

### 10.8 Waterspout [X `Waterspout.js`]

- One fragment pass ray-marched inside a box proxy, 32–96 steps, premultiplied and transparent.
- A hollow wall and a base spray cascade. The axis leans downwind with height. It grows over 4.5 s and lives 34 s.
- **Port** after F5, as a storm event. Pair it with a vortex (§10.9) that moves *with* the funnel; ABYSSAL lets the two separate.

### 10.9 Disaster modifiers [X `OceanSampleGLSL.js:43-313`; `Director.js:294-543`]

All of these are vertex-shader maths plus a CPU event manager, and every one has a CPU mirror (`eventHeight`).

| modifier | count | shape | notes |
|---|---|---|---|
| vortex | ×4 | Rankine profile, depression −w/(1 + 2.2x²), shear foam capped at 0.62 | swirls by **absolute t**: use time since spawn |
| soliton (tsunami) | ×2 | asymmetric sech², overhang push | tsunami: 34 m, 150 m wide, 105 m/s |
| rogue group | ×1 | Gaussian envelope 240 m, 3-mode carrier at λ 420 m | — |
| hurricane | ×1 | swell ring plus an eye that calms the FFT (×0.2) | — |

- The projected grid re-aims its rays at events so a wall stays tessellated.
- **Port (Phase H).** A coastal tsunami needs A3: shoaling amplification and runup over the beach. That is where the shore layer and FEM inundation (§13) meet.
- They are events in the weather model (F7) and fold into the B1 surface, so physics feels them too.

### 10.10 Adaptive quality [X `Quality.js:41-172`, commit `849ff7f4`]

- **Window.** 24 frames or 400 ms, whichever comes first.
- **Panic.** If the **median** frame time exceeds 4× target, drop round(log₂(med/target)) tiers in one move, with a 3 s cooldown.
- **Otherwise:**
  - a mean over 1.25× target lowers the render scale by 0.09, dropping a tier at the floor;
  - a mean under 0.68× target raises the scale by 0.045.
- It never upgrades a tier. `tools/quality-sim.mjs` is a pure-logic test.
- **Port (Phase G).** Pure CPU, scene-wide in cvcGL, driving the tiers for:
  - the cascade count and FFT size;
  - the wake window;
  - spray and rain counts;
  - the near cloud volumes.

  It would also help terrain_lab's large worlds on their own.

### 10.11 Shading, projected grid, atmosphere [X `OceanMesh.js`, `ShadingGLSL.js`]

- **GGX.** Trowbridge–Reitz with height-correlated Smith visibility. The sun's disc widens the lobe; Schlick Fresnel with F₀ = 0.02.
- **Footprint roughness.** Cox–Munk mean square slope `0.003 + 0.00512·U`, split across cascades by how much of each the pixel's footprint fails to resolve.
- **Light through crests (subsurface scattering).** `heightNorm·(L·−V)⁴·(0.5 − 0.5·L·N)³·3.4`, plus body scatter.
- **Env map and aerial perspective.** A 256×128 equirect env map rebuilt each frame from the sky lookup tables and cloud env; aerial perspective from a 32³ froxel atlas.
- **Projected grid.** Rays from a screen-space grid hit a curved sea, and rays past 68 km snap to the horizon.
- **Port.**
  - GGX, footprint roughness and crest scattering in A4 (shader only; reflections stay the gradient sky until an env map exists).
  - The projected grid for the open sea on large terrain_lab worlds (§11).
  - The atmosphere lookup tables are a separate, larger sky project: they would also give the fog that F3 wants. Deferred (decision **D8**).

---

## 11. Into the world model: terrain_lab.ari, local and arisrv

The end state: water, wind and weather are generated with the world, live in its geometry and its physics, render through Ariadne node types that terrain_lab.ari includes, and run the same locally and on arisrv.

### 11.1 Where they go in the world model

**Today.**
- `cvc::world::world_model` holds the terrain source, the surface registry and the props, but no water [C `inc/cvc/world/raster.h:66-89`]. `generated_world` is the de-facto world: params (with `water_level_m`), the model, the terrain, the LOD chunks with their water rungs [C `generated_world.h:173-241`].
- Water exists only as render outputs: `paint_water`, `make_water_rungs`, and `water_field` / `seaSurface` [C `generated_world.cpp:992`, `:2017-2046`].
- **No water body becomes a physics field anywhere.**
  - `generated_world::geometry()` adds terrain, buildings, roads, props and foliage, and no volume [C `generated_world.cpp:1219-1246`].
  - `SceneWorld` adds a world's geometry but keeps only `made.bodies`, so a field would be dropped [C `src/cvc/ariadne/scene_world.cpp:300-308`].

**Changes:**
1. **`water_bodies` (W0) is built by the world builder's `"water"` stage** [C `generated_world.h:262-265`] and stored on `generated_world`. It is deterministic from the seed, like the rest.
2. **The world's geometry gets a water volume.** `world_geometry::add_volume` already carries a `physics::field` with roles [C `inc/cvc/world/world_geometry.h:80-86`, `:114`]; this is the G6 seam `WORLD-GEOMETRY-ROADMAP.md` drew for water and wind.
   - `generated_world::geometry()` adds one `water` volume whose fluid field carries the `fluid_surface` (B5) over every body (W0.4).
   - It adds one `wind` flow volume (E5).
3. **`SceneWorld` keeps `made.fields`.** With the surface hook, the field does not need a `set_field` every tick: the field evaluates the surface at the step's time inside the step.
4. **The ocean model (D1) and the weather model (F2) are GL-free** and sit beside the world, so a headless `SceneWorld` floats bodies exactly as a drawn scene does. A test already pins a generated world's `state_hash` equal drawn and headless [C `ARIADNE_WORLDS.md:95`]; extend it to a crate floating in a lake.

### 11.2 Node types in terrain_lab.ari

The pattern follows the existing nodes [C `scene_realize.h:275-282`; `extensions.cpp:35-43`]:
- registered with `register_scene_node_type`;
- props read through `n.props`;
- state keys from `node_key` (honouring `keys:`), seeded with `read_or_seed` and read with `read_or` each tick [C `inc/cvc/ariadne/scene.h:363-382`, `bind.h:32-92`];
- the ground taken from `out.height_fields[ground]`;
- a tick pushed onto `out.custom_ticks`;
- a panel component with `args: {keys, title, id}`, as `sea_controls.ari` does for today's `wave_sea` [C `src/cvc/ariadne/components/sea_controls.ari`].

**`water`.** Named `water`, not `ocean`, because it draws the sea, lakes, ponds and streams.
- **Inputs:** `world: <generated_world id>`, or `ground:` for any height field.
- **Drawing:** it reads the world's `water_bodies`, and its shader takes over the `<id>_water` LOD rungs from `style_water` [C `world_nodes.cpp:208-222`].
- **It owns:**
  - the FFT cascades (`OceanFFT` moves into the cvcGL library; today it is compiled only into its test and lsystem_coast [C `src/cvcGL/CMakeLists.txt:536-539`, `examples/CMakeLists.txt:137`]);
  - the shore layer;
  - the wake window;
  - foam and spray.
- **Keys and panel:** `water.*`, with a `water_controls.ari` panel and an `INDEX.ari` entry.
- **It supersedes `wave_sea`** (the depth-volume sea, re-filled from `cvc::lsys::sea_field` [C `lsystem_nodes.cpp:338-386`]), which stays until the scenes that use it move over.

**`weather`.**
- It holds the weather model (F2) and publishes `wind.*` (E5) and `weather.*` (storm cells, rain rate, lightning).
- `cloud_sky` and `water` read those keys. `cloud_sky` also gets its forcing from the storm cells (F2), its live keys (F1) and its sun from `lighting.*`.

**`boat`** (D6): a hull mesh, plant parameters, and a body.

**Headless.** `SceneWorld::from_scene` skips any custom node's body with a warning [C `scene_world.cpp:639-644`]. Water and boats get GL-free glue modelled on `scene_generated_world.h`, so physics works without a renderer.

### 11.3 Local scenes (arigl, arigl-web)

- This path needs nothing new. `realize_physics` builds the scene's world from the colliders and volumes [C `scene_realize.cpp:1129-1173`], `tick_scene` steps it [C `:1484-1496`], and the `water` node's tick advances the FFT and sets uniforms.
- The browser budget is the gate (R7). terrain_lab measures 25–47 fps in Chromium before any water work [C `GENERATED-WORLD-ROADMAP.md:314-321`].

### 11.4 arisrv scenes

**Today.**
- A room's holon runs a 120 Hz world clock and Jolt for **avatars only**. The server document's `scene.physics` is parsed, not simulated [C `inc/cvc/ariadne/holon.h:51-53`, `server_document.h:46-49`].
- A room carries only plane, box, sphere and mesh nodes. A `generated_world` or `cloud_sky` node is "not carried by the room; left out" [C `src/cvc/ariadne/conference/room_scene.cpp:565-571`].
- `room_view::update` never ticks custom nodes [C `src/cvcGL/ariadne/room_view.cpp:134-145`].
- Client clocks are re-aligned to the server's tick: they slew within 60 ticks and seek past that [C `holon.h:85-95`, `:293-320`].
- Regenerating terrain on the client from a replicated descriptor (G5; `scene.world.*`, WD5) is planned, not built [C `GENERATED-WORLD-ROADMAP.md:232-239`, `ARIADNE_WORLDS.md:198`, `:607`].

**What water and weather add on top, by R2:**
- **Replicated.**
  - The `water` and `weather` nodes' props and keys, as a room section like `env.*`.
  - The weather's seed and authored events (F7).
  - The poses of floating bodies and boats.
  - The world clock is already synced.
- **Rebuilt on each client.**
  - The terrain and `water_bodies` (from the G5 descriptor).
  - h0 from the parameters and seed (R3).
  - The waves at clock time t.
  - The wake window, from the replicated poses (C2).

  Two clients therefore draw the same waves to within float32 rounding, and nearly the same wakes. A few ticks of clock skew is a few milliseconds of wave phase, which is invisible. A seek past 60 ticks makes a visible jump.

**Prerequisites, in order:**
1. **WD5 / G5:** the room carries a `generated_world` as a descriptor, and clients regenerate it.
2. **`room_view` ticks custom nodes** on the holon's clock.
3. **The server simulates scene bodies,** not only avatars, so floating objects and boats are authoritative.
   - Until then, ambient debris could run on every client in lockstep. The physics is cross-platform deterministic, so identical starts stay identical. That breaks the moment an avatar touches the debris.
4. **Generation parity.** `generate_world` calls the standard library's sin/cos/exp/pow at about 19 sites, and no test checks native against wasm [C `src/cvc/world/generated_world.cpp`].
   - A last-bit difference in heights can move a shoreline texel, which flips a wake mask or a shore phase.
   - That is cosmetic for drawing, since clients do not run authoritative physics.
   - It still argues for a native-vs-wasm parity test of `generate_world` and `water_bodies` before arisrv relies on regeneration.

### 11.5 Migration

- lsystem_coast stays the development bench through Phase B.
- After O15, the coast scene becomes an Ariadne document using `water`, `weather` and the island terrain, and lsystem_coast can retire.
- `wave_sea` retires once nothing includes it.
- The C++ `terrain_lab.cpp` is being dropped and is not a target.

---

## 12. Order of work

Each piece below is one PR with its own tests. The work starts in lsystem_coast and moves to terrain_lab.ari at canonization (§11).

| # | PR | phase | depends on | where | exit |
|---|---|---|---|---|---|
| O1 | `water_bodies` + shore grid (W0.1–W0.2), standard library only, single-level labelling | 0 | — | cvc::world | the four labelling tests; a debug view in terrain_lab |
| O2 | the ocean model library (D1): CPU h0 with per-cascade seeds, band-limited, with the swell term (§10.1). `OceanFFT` uploads h0 and its h0 pass is retired | B1, E1 | — | new library + cvcGL | the coast sea looks as before (minus the overlapping-band energy); h0 bit-equal across runs |
| O3 | seabed + shore textures in lsystem_coast; depth damping (A2); `coastH()` retired | A1–A2 | O1, O2 | demo | no vertex crosses the waterline |
| O4 | the shore layer (A3): eikonal phase, Green's law, breaking, runup film, wetness, reflection; its CPU evaluator | A3 | O3 | library + demo | §4 tests; cliffs reflect, beaches break |
| O5 | shading (A4, §10.11) and persistent foam (§10.2) | A4 | O2 | demo | measured native and in the browser |
| O6 | `fluid_surface` hook on `cvc::physics::field`, the time base pinned, pycvc bindings | B5 | O2, O4 | cvc::physics | the box-heave and determinism tests |
| O7 | debris in lsystem_coast; the CPU-vs-GPU residual harness; M chosen | B2–B6 | O6 | demo + test | crates ride the swell and the swash |
| O8 | forward FFT + an eWave window with masks and sponge (C1, C3) | C | O2 | cvcGL | a synthetic drop reflects off a straight wall |
| O9 | shared immersion code; body sources (C2); wake foam (C4) | C | O6, O8 | library + cvcGL | ring from a dropped crate; a box hull's wake at about 19.5° |
| O10 | `add_force_at` / `add_torque` (D1) | D | — | cvc::physics | can land any time |
| O11 | hull model, propulsion, rudder, water exclusion; a drivable boat in lsystem_coast | D2–D5 | O6, O9, O10 | physics + demo | the §7 exit |
| O12 | `dyn_boat_plant` + plant_host | D4 | O11 | cvc::physics | nav drives the boat |
| O13 | fetch/lee map, gust field, sea lag, currents (E2–E4, E6, §10.3–10.4) | E | O1, O2 | library + demo | the lee goes calm; no pop on a wind change |
| O14 | one scene wind (E5) for trees, `cloud_sky`, water, physics flow | E | O13 | Ariadne + physics | one key turns everything |
| O15 | canonize: the Ariadne water node in terrain_lab.ari (§11) | — | O1–O7 at least | Ariadne | terrain_lab's sea, lakes and ponds animate and float bodies |
| O16 | arisrv: replicate inputs, rebuild on clients (§11) | — | O15 | Ariadne / arisrv | two clients draw the same waves and wakes |
| O17 | live sky keys, the sun from the light rig, the water as a shade receiver (F1, A4) | F | — | Ariadne | **quick win, can land any time** |
| O18 | the weather model, storm cells, dark bases (F2, §10.7) | F | O17 | cvc::world + Ariadne | a storm builds, darkens and clears |
| O19 | rain: streaks, rings, wetness, shafts, fog (F3, §10.6) | F | O18, O4 (wetness) | cvcGL | the F exit, measured in the browser |
| O20 | storm wind, lightning, friction into physics (F4–F6) | F | O14, O18 | all | traction drops in the rain |
| O21 | adaptive quality (§10.10) | G | — | cvcGL | a slow GPU recovers in seconds |
| O22+ | disasters (§10.9), waterspout (§10.8), FEM seiches (§13), water surface wavelets (research) | H | A, B, F | — | scoped separately |

**Suggested start:** O1 → O2 → O3 → O4. Together they fix what is visible (the waves at the shore) and pay the port debts, and they build the grids everything else reads.
- O10, O17 and O21 are independent and small enough to land alongside.
- Canonize (O15) after Phase B, so the node's interfaces settle first: water bodies, the surface model and the wind keys.

---

## 13. FEM and the mesher

The finite-element tools are not the real-time wave solver.
- A depth-weighted P1 wave equation on a triangulated water area (M·ü + K_gd·u = 0) gets an irregular coastline exactly right: its natural boundary condition reflects.
- But it is non-dispersive (long waves only, so no Kelvin wake), and it needs sparse solves on the CPU, single-threaded in wasm.

Where they do help:
- **Seiches: lakes and harbours sloshing.** `cvc::fem::laplacian_eigenmodes` on a lake's water-surface mesh, with the stiffness weighted per element by g·depth, gives the basin's sloshing modes. Impacts and the wind can drive those modes as a modal sum: cheap per frame, with exact boundaries. This fits lakes in terrain_lab. It needs a depth-weighted `stiffness_matrix`, a small change [C `fem.h:88`].
- **Offline refraction and diffraction maps.** The mild-slope equation, a Helmholtz problem, solved per coast and swell, would give a static amplitude and direction map for A3. It needs an indefinite or complex solver that `cvc::fem` does not have. Research.
- **Tsunami runup and storm surge:** non-linear shallow water on an unstructured mesh. Research, and only if the disaster work (§12, H) wants inundation.
- **Distance fields and immersion, available now.** The shore field reuses `cvc::nav::edt2_squared` (W0.2). `cvc::sdf` gives a hull's signed distance for D5's water exclusion, and immersion for arbitrary hull meshes.

Tetrahedral meshes of water stay parked as W5, for the reasons in `OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md` §B.2.

---

## 14. Risks and evidence gaps

- **Frame budget.** None of this is measured. The FFT cascades (57 passes), eWave (about 34), the shore layer, rain particles and the deck are all on one GPU. terrain_lab measures 25–47 fps in Chromium before any of it [C `GENERATED-WORLD-ROADMAP.md:314-321`]. Every phase is measured before it is deployed (R7).
- **The eWave update** is derived here from linear potential flow. The paper's own text was not read (its PDF could not be extracted on this machine). Check it against the paper before C1 lands.
- **The physics band (B2).** Is M = 64 enough for a 10 m hull? It was chosen against the measured residual (O7b): 64 modes on a 64 lattice miss lsystem_coast's drawn sea by 4.0 % RMS, within 6 % of the floor the shader's own sampling sets. A 10 m hull feels waves longer than about 20 m, and the band keeps every wave down to 0.9 m.
- **The time base (B5).** Physics ticks and clock time must line up, or bodies bob out of phase with the waves drawn.
- **The single water level until W1.** Lakes perched above the sea do not exist yet; W0 is written to accept them when they do.
- **arisrv prerequisites are not built** (§11.4): a descriptor-carried world (WD5 / G5), custom-node ticks in rooms, and server simulation of scene bodies. Water in rooms waits on them. Native-vs-wasm parity of `generate_world` is untested.
- **Arc Blanc** is the closest prior art: an FFT ocean, per-triangle buoyancy and drag, and finite-difference wakes per hull. It runs its physics on the GPU, which R1 rules out for us, and its wakes are non-dispersive. Its timings are from the paper, not measured here.
- **The Water Surface Wavelets code licence** is unclear: the paper is CC BY-NC-SA, and a third-party page lists the code as MIT with mixed replication. Do not port it without checking.

---

## 15. Open decisions

| # | decision | options | recommendation |
|---|---|---|---|
| D1 | where the standard-library ocean model lives | (a) a new `cvc::ocean` library linked by physics, cvcGL, pycvc and arisrv; (b) inside `cvc::physics`; (c) inside `cvc::world` | **(a)**: it is shared by every consumer and belongs to none of them |
| D2 | the shape of the surface hook | (a) a virtual `fluid_surface`; (b) `std::function`s; (c) a sampled height grid passed per step | **(a)**: it carries a digest for `state_hash`, and lakes, streams and the sea are all one type |
| D3 | the wake window | (a) one camera-centred window; (b) one per active boat (Arc Blanc); (c) both | **(a)** to start; (b) when two boats must interact visibly |
| D4 | how swash is drawn | (a) a film in the terrain's fragment shader; (b) displaced geometry over the beach | **(a)**: no z-fighting with the sand, and it works on every LOD rung |
| D5 | how weather replicates | (a) a deterministic schedule from the seed plus authored events under `weather.*`; (b) the server streams the whole weather state | **(a)**, by R2 |
| D6 | can wakes push bodies? | (a) no, the wake layer is visual; (b) an opt-in deterministic CPU eWave | **(a)** now; revisit for boat-to-boat play |
| D7 | lake vs pond, and per-body defaults | area threshold, fetch, ripple strength | start at 1 ha [E]; tune in terrain_lab |
| D8 | atmosphere lookup tables (ABYSSAL's sky) | port, or keep the gradient sky plus simple fog | **defer**; F3's fog first |
| D9 | how terrain_lab draws the sea | (a) displace the per-chunk LOD rungs; (b) a projected near grid (§10.11) plus rungs for distance; (c) rungs for geometry, everything small in the normal | **(c)** to start, then (b). Rung 0 of `make_water_rungs` is the heightfield's own sample spacing [C `terrain_lod.cpp:244-300`], which coarsens as the world grows, so on large worlds only the long swell can be geometry |
| D10 | ABYSSAL's unrotated cos² spreading | copy it or rotate it with the wind | **rotate**: it is a bug |
| D11 | the order of work | start at O1–O4 (the visible shore fix) or at O6 (physics) | **O1–O4** |
| D12 | does water get its own node? | (a) a `water` node referencing a world or a ground; (b) folded into `generated_world` | **(a)**: lsystem scenes and future terrain sources get water too, and the panel and keys stay separate |
| D13 | floating bodies in arisrv rooms before the server simulates scene bodies | (a) wait for server simulation; (b) ambient debris in deterministic lockstep on every client | **(a)** for anything an avatar can touch; (b) is acceptable for scenery |

---

## 16. Licensing

ABYSSAL is MIT, © 2026 Davi (Token-Gremlin). Ported GLSL and logic keep the MIT notice in source headers and `THIRD_PARTY_NOTICES.md`, as `OceanFFT.h` already does. Literature methods (iWave, eWave, Kerner, Arc Blanc's formulas) are reimplemented from the papers, not copied from code.

---

## 17. References

- ABYSSAL — Token-Gremlin, *natural-disasters*, MIT. https://github.com/Token-Gremlin/natural-disasters
- Tessendorf, J. *Simulating Ocean Water* (SIGGRAPH course notes, 2001/2004).
- Tessendorf, J. *Interactive Water Surfaces* (iWave), Game Programming Gems 4, 2004. https://jtessen.people.clemson.edu/reports/papers_files/Interactive_Water_Surfaces.pdf
- Tessendorf, J. *eWave: Using an Exponential Solver on the iWave Problem*, 2014. https://jtessen.people.clemson.edu/reports/papers_files/ewavealgorithm.pdf
- Gomez, M. *Interactive Simulation of Water Surfaces*, Game Programming Gems, 2000.
- Jeschke, S., Wojtan, C. *Water Wave Packets*, ACM TOG (SIGGRAPH) 2017.
- Jeschke, S., Skřivan, T., Müller-Fischer, M., Chentanez, N., Macklin, M., Wojtan, C. *Water Surface Wavelets*, ACM TOG (SIGGRAPH) 2018. https://pub.ista.ac.at/group_wojtan/projects/2018_Jeschke_WaterSurfaceWavelets/WaterSurfaceWavelets.pdf — replicability notes: https://graphicsreplicability.github.io/replicability.graphics/papers/10.1145-3197517.3201336/index.html
- Schreck, C., Hafner, C., Wojtan, C. *Fundamental Solutions for Water Wave Animation*, ACM TOG (SIGGRAPH) 2019.
- Algis, D. et al. *Arc Blanc: a real time ocean simulation framework*, arXiv:2503.03326; JCGT 14(1), 2025. https://arxiv.org/abs/2503.03326 · https://jcgt.org/published/0014/01/05/
- Kerner, J. *Water interaction model for boats in video games*, Parts 1–2, Game Developer. https://gamedeveloper.com/programming/water-interaction-model-for-boats-in-video-games · https://www.gamedeveloper.com/programming/water-interaction-model-for-boats-in-video-games-part-2
- Horvath, C. J. *Empirical Directional Wave Spectra for Computer Graphics*, DigiPro 2015.
- Hasselmann, K. et al. *Measurements of wind-wave growth and swell decay during the Joint North Sea Wave Project (JONSWAP)*, 1973.
- DNV-RP-C205, *Environmental Conditions and Environmental Loads* (the JONSWAP α from Hs and Tp).
- Eckart, C. *The propagation of gravity waves from deep to shallow water*, 1952 (explicit finite-depth dispersion).
- Green, G. 1838 (shoaling amplitude, A ∝ d^(−1/4)); McCowan, J. 1894 (breaking index ≈ 0.78).
- Battjes, J. A. *Surf similarity*, 1974 (the Iribarren number; reflection ≈ 0.1·ξ²).
- Hunt, I. A. *Design of seawalls and breakwaters*, 1959 (runup R ≈ ξ·H).
- Monahan, E. C., O'Muircheartaigh, I. 1980 (whitecap coverage vs wind speed).
- Dobashi, Y. et al. *A simple, efficient method for realistic animation of clouds*, SIGGRAPH 2000 (the cellular automaton `cloud_layer` uses).
