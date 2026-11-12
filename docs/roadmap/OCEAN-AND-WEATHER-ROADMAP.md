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

**As built (O7c): debris in lsystem_coast.** Crates and barrels float in a `cvc::physics` world (Jolt) beside the drawn island.
- **The sea they feel** is the sea the water vertex shader draws. It is a fluid field whose free surface is `ocean_surface` over `sea_surface`, built from:
  - the h0 the GPU cascades were given (each cascade now keeps it);
  - the cascades' fades and decay wavenumbers;
  - `wave_amp` and `chop`;
  - the shore grid and the shore layer.

  It is rebuilt whenever a knob it reads changes. The field's damping, 10⁴ N s/m⁴, makes a metre-sized body follow the water within a tenth of a second, so the breakers' bores and the swash carry it.
- **The crater lake.** The island had one body of water, the sea, so a bowl with a raised rim is carved into the summit (`craterH`).
  - A flood fill finds it spills at 24.4 m, so the lake holds still water at 23.5 m.
  - `water_bodies` finds the lake at that level: 571 m², 4.1 m deep (a pond by its one-hectare rule).
  - It is a second fluid field, with a `still_surface` at its level.
  - It is drawn as a flat sheet with the sea's Fresnel shading, the crater's floor showing through the shallows. Trees keep out of the crater.
- **The debris.** Crates are 1.2 × 0.9 × 0.8 m at 250 kg/m³; barrels are 1 m long and 0.36 m in radius, at 200 kg/m³. Both are empty, with air inside the boards and staves.
  - Six are dropped off the windward (west) shore, in still water 1.5 to 5.5 m deep, and three on the lake.
  - Each body is drawn where the world has it by `physics_scene_binding`. The panel's Debris section drops them again.
  - The knobs are `coast.debris.enabled` and `coast.debris.source_cell`.
- **B3 on a lattice.** At first every field cell's height and velocity inverted the sideways move from scratch: 7.5 ms a physics step for six bodies. Two changes in `sea_surface` cut that:
  - **A cell's height and velocity share one source.** The field reads both at the cell's centre through a memo it owns (`fluid_surface::height_with` / `velocity_with`, `sea_surface::source_memo`), so the velocity needs no second inversion. That brings an exact height-and-velocity pair from about 5 to 3.0 µs. (#1159 kept the last source in a `thread_local`; it moved to the caller, since libcvc keeps no process-wide state.)
  - **`sea_params::source_cell_m`** finds the source only at the corners of a lattice, once per corner and time. It interpolates the move between the corners, then takes one Newton step whose matrix is the interpolated move's own gradient.
    - On a steep test sea, a 0.5 m lattice misses the converged source by 2.4 mm RMS (37 mm at most); the exact 4 steps miss by 11 mm (273 mm). So it is more accurate as well as cheaper: 1.75 µs a pair against 3.0.
    - `OceanSea.TheSourceLatticeKeepsTheSurface` measures that. `OceanSea.TheSourceLatticeIsTheSameOnEveryPlatform` pins it at `08f956c6889de4d2`.
    - A box rides it as it rides the exact sea, within 0.02 µm (`PhysicsOceanSurface.ABoxRidesTheLatticeSeaAsTheExactOne`).
    - An exact surface's digest and values are unchanged, so every existing pin holds.
- **The cost.** The debris step at 40 Hz, three of the app clock's 1/120 s quanta: 4.1–4.3 ms a step on the lattice and 6.3 ms exact (MSVC Release, 6 bodies at sea; the lake's sleep once still).
  - Interactive runs step them on a thread of their own, so the frame rate is the same with them as without.
  - A capture or `--debris-check` steps them in the frame, so each frame shows the world at the clock's time.
- **Measured.** `lsystem_coast --debris-check` runs 60 s at the default 1.2 m swell on the synthetic clock and gives the same numbers every run:
  - one crate is carried 11 m in and left 2.4 m up the beach by the swash;
  - a barrel is carried 8 m into the shallows, to the waterline;
  - one crate is carried in and then 10 m back out by the backwash;
  - the three further out ride the swell, and none sank;
  - the lake's three float at rest with their centres at 23.70 m, the lake's surface through each.

  At a 2.5 m swell, three of the six are carried 4 to 17 m in. `--look EX,EY,EZ,TX,TY,TZ` holds the camera for frames of the surf and the lake. `--shore-check` still finds no vertex on dry ground.
- **Not yet:**
  - the ring a dropped crate makes, and the wakes (O9);
  - the browser build (wasm-mt, which has Jolt) is not checked.

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

**Exit.** Crates and barrels dropped in lsystem_coast ride the swell, get carried by the swash, and settle on calm lake water at the lake's own level. Met by O7c ("As built (O7c)" above), on a crater lake carved into the summit for it.

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
  - `h, φ ← mask·(h, φ)`: 0 on land and obstacles, 1 in open water. Beaches ramp toward 0 across the swash zone so they absorb, while cliffs are a hard 0 so they reflect (K_r from A3). As built, a hard 0 is a no-flux wall by default (`walls = no_flux`, O8c, below): its cells hold the water's mirror image through a symmetric operator stepped symplectically, so a crest reflects as a crest and nothing can grow. `walls = zero` keeps O8a's Dirichlet wall, under which a crest reflects as a trough.
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

**Cost.** At 256² one OceanFFT cascade is 35 passes (2 time-evolution passes, 2 × 16 butterflies, 1 assembly), so its 3 cascades are 105 passes per frame [counted in OceanFFT.cpp; the port notes' 57 was wrong]. eWave adds 34 more (counted pass by pass in O8b; with no-flux walls a 1/30 s step is 71 at the default 1/60 s substep and 36 with a 1/30 s substep, O8c), about +32 % by pass count [E]. Measured in O8b (below): a window adds about 20–27 % to the three cascades' GPU time, since its passes write two channels to the cascades' four. Arc Blanc reports 0.34 ms of GPU time per solid for its finite-difference wakes on an RTX 2070S [L].

**As built (O8a: the CPU eWave).** O8 is split in two: **O8a** is eWave on the CPU in `cvc::ocean` (below), **O8b** the GPU window in cvcGL, which O8a's tests and digest can check.
- `cvc/ocean/ewave.h`, `cvc::ocean::ewave`: an n × n window (n a power of two) of cell dx, one depth, origin at cell (0, 0). State h and φ; `step(dt)` packs z = h + iφ, runs fft.h's forward transform, separates H and Φ at k and −k, applies C1's exact update and the damping e^(−(γk² + rate)·Δt), and transforms back. The per-mode coefficients (with the inverse's 1/n² folded in) are kept while Δt is unchanged.
- In real space each step damps h and φ by a rate per cell, applied as e^(−rate·Δt), so the damping per second does not depend on the step size. A beach value m of the mask (0 < m < 1) adds `beach_rate`·(1 − m) (default 10 /s), and the sponge adds a rate ramping as r² over the last `sponge_cells` cells to `sponge_rate` at the edge. Land (m = 0) is zeroed every step.
- Sources go straight into h between steps, times the mask: `add_height(x, y, amount, radius)` splats amount·(1 − r²/R²)³, and `add_heights` adds a whole field (C2's −(ζ(t) − ζ(t−Δt))). `at(x, y)` reads h bilinearly; `energy()` is ½ρg Σh² dA plus ½ρ Σκ|Φ|² dA / n².
- k = 0: the mean height stays. The exact update would drift the mean potential by −g·mean(h)·Δt a step, and the mask would turn that uniform drift into a step of φ at every shore, so the mean potential is held at 0. That is a choice of gauge; the heights are those of the exact update.
- Deterministic: det math, fft.h, built without FMA contraction (in the exact-math list). `digest()` hashes the parameters, the mask, h, φ and the time; a fixed scenario (mask with a beach ramp, sponge, both dampings, three drops, two step sizes) is pinned at `0xa4933455f6946eca` in `ocean_ewave_test`, measured on Windows; CI checks it on Linux and macOS.
- The update is derived here from linear potential flow, and the standing-mode test checks it against ω = √(g k tanh kd) computed in the test, not taken from the library. It has still not been checked against the eWave paper's own text (§14).

Measured in `ocean_ewave_test` (Windows, MSVC Release, one thread):
- **Exact for any Δt:** a standing cosine mode, at ωΔt from 1.65 to 4.0 rad over 400 steps, deep and 1.5 m deep (kd ≈ 1.6), misses h(t) = A cos ωt by at most 6.6e−14 m on A = 0.3 m, with ω computed independently in the test.
- **Energy:** without damping, mask or sponge, the drift is at most 7.9e−14 (relative) over 2000 steps while the energy moves between potential and kinetic.
- **Ring:** a Gaussian drop (σ = 0.5 m) gives a ring whose envelope moves at 1.019 m/s, against the 1.000 m/s group velocity of its dominant wavelength (2.57 m, k = √1.5/σ by stationary phase). The crest height spreads by 2.9 % over 16 rays.
- **Wall (O8's exit):** with mask 0 over a half-plane, nothing appears beyond it (exactly 0, by construction: the mask zeroes the half-plane every step). The reflection comes back as the *antisymmetric* image, best fitted about x = dx/2, to 18.8 % RMS. h = φ = 0 is a Dirichlet wall: a crest comes back as a trough, where a real vertical wall reflects a crest as a crest (no flux, Neumann). O8c (below) added no-flux walls, `walls = no_flux`, now the default; O8a's wall stays available as `walls = zero`, bit for bit.
- **Thin walls leak:** the mask is a lid on the surface with water under it, and the spectral operator couples through it. With open water behind a band, a 3 m (12-cell) band lets through 4.6 % of the incident amplitude (held below 6 %) and a 6 m band 2.8 %. Diagnostic runs, not in the test: behind a thick band most of what is left came round the periodic window through the sponge. Doubling the window (n = 512) takes a 6 m band from 2.8 % to 1.4 % and a 12 m band from 3.1 % to 1.0 %, while a 3 m band only goes from 4.6 % to 4.0 %. A steeper sponge (6 /s) does not help; a wider one (32 cells) helps a little.
- **Sponge:** 16 cells at 2 /s leaves 0.02 % of a ring's energy after 30 s (about 1.4 % of its amplitude).
- **Beach:** a band ramping the mask from 1 to 0.2 and back over 8 m leaves 68.95 % of a ring's energy after 10 s stepped at 30 Hz, 68.96 % at 60 Hz and 68.89 % at 10 Hz.
- **Finite depth:** in 1 m of water a long plane pulse travels at 3.108 m/s, against √(gd) = 3.132 m/s (−0.8 %).
- **Kelvin wake (Phase C's exit):** a Gaussian footprint (σ = 1 m, 5 cm deep, ramped in over 8 s) driven as C2's source at 2.5 and 3.0 m/s in deep water (512² at 0.5 m, Δt = 0.2 s). It travels 140 m, so the rows read 16–44 m behind it are steady wake, clear of the start's ring. The envelope's edge on a row is the outermost point where |h| exceeds a fraction of the row maximum; its fitted slope gives 19.97°, 19.70° and 19.56° at 30, 40 and 50 % for 2.5 m/s, and 20.38°, 19.38° and 18.79° for 3.0 m/s, against 19.47°. Each must be within 1.5°. Lower thresholds (10–20 %) reach into the Airy tail beyond the cusp and read 21–24°. The row maxima, at 22.6° and 20.2°, are reported, not checked. For this footprint the speeds are past the Rabaud–Moisy transition, and at 4.5 m/s the 30–50 % edge itself narrows to 17.7–18.9°. Wider footprints (σ = 1.5 or 2 m), for a lower Froude number, leave cusp waves too weak to measure. The two speeds take 7–9 s each with other builds sharing the machine.
- **Cost of one step:** 0.16–0.22 ms at 64² (C5's coupling size), 0.6–1.0 ms at 128² and 2.7–3.5 ms at 256², with other builds sharing the machine.

Not in O8a:
- the GPU window (O8b);
- moving the window with the camera or boat (C3's index shift);
- a mask built from `water_bodies` (C6);
- body sources (C2, O9);
- a Neumann wall (built in O8c: a symmetric operator on ghost cells, stepped symplectically; the default).

**As built (O8b: the GPU window).** `src/cvcGL/ocean/EWave.{h,cpp}`, class `EWave`: O8a's window on the GPU, in float, taking the same `cvc::ocean::ewave_params`. Like OceanFFT it lives beside the cvcGL tests, outside the library, and is compiled into what uses it (for now only `cvcgl_ewave`; drawing the wake in lsystem_coast is O9).
- **The forward FFT is OceanFFT's.** The butterfly LUT, the butterfly pass and the pass setup (scissor test off, as O7b learned) moved out of `OceanFFT.cpp` into `src/cvcGL/ocean/OceanButterfly.{h,cpp}`, which OceanFFT, EWave, lsystem_coast and the two ocean tests now compile. The forward transform conjugates the LUT's twiddle in the shader (a `uForward` uniform), which is exact, so both directions read one table. OceanFFT is unchanged in its output: `cvcgl_ocean_band` prints O7b's table to the digit.
- **The state** z = h + iφ is one RG32F texture, cell (i, j) at texel (i, j). A step is the forward FFT (16 passes at 256²), an update pass, the inverse FFT (16) and a real-space pass: **34 passes**, counted as they are rendered (OceanButterfly counts its own passes) and checked against 4 log₂ n + 2 in `cvcgl_ewave`. The update pass reads texels k and −k, separates H and Φ, applies C1's exact update and the damping e^(−(γk² + rate)·Δt) with the coefficients computed per texel from uniforms (so a new Δt costs nothing), and recombines. The real-space pass normalises by 1/n², applies the beach rate and the sponge as e^(−rate·Δt) and zeroes land, from an R32F mask texture. Three RG32F textures rotate so that the state is in the same texture after every step, for a renderer to sample.
- **Nyquist.** In this natural layout the Nyquist index n/2 is its own mirror, so its row and column pair like any other and the Hermitian packing is the CPU's exactly. OceanFFT's centred layout is the one that leaves the Nyquist row without a partner; nothing is dropped here.
- **Sources go in at once, as on the CPU,** not in the real-space pass as first planned. `add_height` and `add_heights` change h immediately and the next step moves them on; a source added in the step's last pass would reach the picture a step late and unevolved. So `addDrop(x, y, amount, radius)` (the same bump, computed in the shader) and `addHeights(field)` (an R32F source texture, C2's shape) are one pass each, times the mask. With a body source every step that is 35 passes.
- Also `setMask`, `setState`, `step(dt)`, `heightTexture()` (.r = h, .g = φ), `readbackHeight()`, `readbackState()`, `passesPerStep()`. No compute shaders; RG32F and R32F render targets are what WebGL2's `EXT_color_buffer_float` provides.

Measured in `cvcgl_ewave` (Windows, the offscreen context on the Intel Arc iGPU, other builds sharing the machine):
- **GPU against CPU.** One scenario on both at 256² and dx = 0.25 m: a drop 4 m from a half-plane wall, a beach band (the mask down to 0.3), the sponge, γ = 10⁻³ m²/s and 0.02 /s of damping, a whole-field source at step 20, 24 steps at 0.2 s and 24 at 0.1 s. The RMS of GPU − CPU over the RMS field stays at 0.75–2.3·10⁻⁵ for h and 0.14–1.3·10⁻⁵ for φ (held under 10⁻³). The drop as added agrees to 2·10⁻⁸.
- **The wall on the GPU.** O8a's wall test (σ = 0.5 m Gaussian 6 m from the wall, to t = 13 s), stepped at 0.25 s: exactly 0 beyond the wall, the reflection 61 % of the field before it, and the field the antisymmetric image to 21.7 % RMS (O8a's 18.8 % was stepped at 0.1 s; the mask acts once a step). The same run on the CPU at 0.25 s agrees with the GPU to 5.6·10⁻⁵ RMS.
- **Cost of one step at 256²:** 0.28–1.23 ms of GPU time (a timer query; 0.40–1.66 ms wall with the queue drained), against 0.35–3.3 ms for one OceanFFT cascade in the same runs. eWave's 34 RG passes cost about 0.75 of a cascade's 35 RGBA passes (0.64–0.80 in six runs, 0.38 in a seventh), so a window adds about 20–27 % to the three cascades' GPU time, under the +32 % its pass count alone would suggest (the Cost paragraph above; the +60 % first estimated there came from a wrong count of 57 passes for the cascades). The spread is the shared machine; the ratio is steadier than the times. Not yet measured in the browser.

Not in O8b: drawing the wake (C4, O9), body sources (C2, O9), the window following the camera or boat (C3's index shift), a mask from `water_bodies` (C6).

**As built (O8c: no-flux walls).** O8a's wall zeroed h and φ: a Dirichlet wall, like a string's fixed end, under which a crest comes back as a trough. A vertical wall or cliff is a no-flux wall, ∂φ/∂n = 0, about which h and φ are both even: a crest comes back as a crest, and the wall is an antinode. O8c builds that on the CPU and the GPU as a symmetric operator stepped by a symplectic integrator, so it conserves energy by construction, and makes it the default again (`walls = no_flux`).
- **The first build, and why it was replaced.** Ghost cells held the mirror image of the water, overwritten after every exact step. A straight wall reflected correctly, but overwriting the ghosts is an oblique projection: the water's update was not symmetric wherever the image was not an exact mirror or was cut off at the band. The review measured exponential growth in closed basins: a disc 31 times in 2000 s at Δt = 0.1 s, a wall two cells thick 10⁷² times at 0.25 s. Tapers, ghost damping, weight caps and folds were tried on that scheme and failed (commit 2adfd1f8 has them).
- **The scheme.** Linear water waves are Hamiltonian: H = ½(g h·h + φ·Kφ), h_t = Kφ, φ_t = −g h. The state lives on the water cells (mask above 0); wall cells hold h = φ = 0.
  - R takes the water to an extended field over the whole window. Water cells copy, each ghost cell takes its mirror image (the weighted sum of its bilinear corners), and every other wall cell is 0. Rᵀ folds a field back: each water cell takes its own value plus, for each ghost that reads it, that ghost's weight times the field at the ghost.
  - S = RᵀGR, where G is the periodic multiplier on the window (FFT, multiply, inverse FFT). S is symmetric positive semidefinite for any ghosts and weights. M, the lumped mass, is the diagonal of the row sums of RᵀR (`ewave_lumped_mass`). The water moves by h_t = M⁻¹Sφ, φ_t = −g h, which conserves H = ½(g h·M·h + φ·S·φ).
  - Where a straight wall is mirrored cell for cell (weights 1), M = 2 and M⁻¹S is exactly the even extension's operator: the exact Neumann wall. Elsewhere the wall is first order in where it reflects, but it is never unstable.
  - Störmer–Verlet in substeps of one fixed Δt_s per window: φ½ = φ − (gΔt_s/2)h, h′ = h + Δt_s M⁻¹Sφ½, φ′ = φ½ − (gΔt_s/2)h′. Δt_s is `ewave_params::substep` (default 1/60 s), or 1.8/ω_max if that is smaller, with ω_max taken at the window's corner wavenumber √2·π/Δx (`ewave_substep`; 0.136 s at 0.25 m cells).
  - **Why it is stable.** With nonnegative weights S ≤ max(G)·RᵀR ≤ max(G)·M, so every generalised eigenvalue of (S, M) is at most G's largest, and every mode is inside Störmer–Verlet's limit ωΔt_s < 2.
  - The substep conserves exactly the modified energy ½(g h·M·h − (gΔt_s/2)² h·S·h + φ·S·φ). That is a quadratic invariant of the linear map for any symmetric S and positive M, positive definite under the limit, so nothing can grow. H itself wobbles below it: a mode started from h alone keeps H within [(x cot x)², 1] of its start, where x = ωΔt_s/2. That is 0.99 at ωΔt_s = 0.2 and 0.51 at the limit, which only the shortest waves reach.
  - **The substep never follows the caller's Δt** (the second review). κ̃ (below) and the invariant both depend on Δt_s. The first version of this scheme took the fewest equal substeps of each step's Δt, so a Δt that changed between steps switched between maps that each keep a different ellipse, and that pumps energy in by parametric resonance. The review measured it: Δt alternating 0.136 and 0.101 s grew a disc 10¹⁰³-fold in 4000 steps; a frame Δt uniform in [1/60, 1/30] s at 0.05 m cells grew it 26-fold in 60000 steps, a steady exponential; zero walls (the exact step, exact for any Δt) decayed. So `step(dt)` now runs ⌊(pending + Δt)/Δt_s⌋ whole substeps (`ewave_substep_count`, to within 10⁻⁹ of a substep) and carries the rest (`pending()`, below a substep), and every substep is the same linear map whatever the Δts. The waves trail `time()` by less than a substep (no interpolation), a step shorter than what is left of a substep runs none, and the cost is set by Δt_s: two substeps a 1/30 s step at the default; `substep = 1/30` halves that for a 30 Hz loop. A window with no ghosts keeps taking each Δt whole.
  - **G is κ̃ = 4 sin²(ωΔt_s/2)/(gΔt_s²), not κ.** Störmer–Verlet's discrete frequency then satisfies cos(ω̃Δt_s) = 1 − gκ̃Δt_s²/2 = cos(ωΔt_s). So open water keeps eWave's exact dispersion, and a straight wall's modes keep their exact frequencies. κ̃ ≤ κ is still a nonnegative multiplier, so the stability argument holds unchanged. h is exact at each step for a mode started from h alone; φ's amplitude is x cot x of the exact (within 1 % for ωΔt_s < 0.35). With κ itself a ring's phase drifts, and the straight wall's reflection was 55 % off its even image at Δt = 0.1 s, against round-off with κ̃.
  - **Dissipation acts every substep, for Δt_s (so that it too is the same map whatever the Δts), and only removes energy.** `damping_k2` becomes h ← h − M⁻¹Rᵀ(1 − F)Rh with F = e^(−2γk²Δt_s): one more FFT pair a substep, only when γ > 0. That operator is self-adjoint under M with its eigenvalues in (0, 1], so it only takes potential energy, and it is the identity as F goes to 1. Twice the rate on h alone takes a mode's energy at the exact step's mean rate.
    - Its first form, h ← M⁻¹RᵀFRh, is the same beside a straight wall but is not the identity at F = 1. M⁻¹RᵀR averages each water cell a ghost reads with the corners round that ghost's mirror point, wherever the point falls between cells. Run once a substep, that smoothing took lsystem_coast's crater lake's ring faster than the zero walls had (below, and O9's numbers).
  - The beach and the sponge also act on h alone, at twice their rate. A diagonal factor under the diagonal M only takes potential energy; one on φ need not, since S is not diagonal. The global damping acts on both, as a uniform factor that scales H.
  - Without ghosts (no walls, `walls = zero`, `ghost_cells = 0`) the window keeps the exact per-mode step bit for bit.
- **Where the image is cut, and the band.** In this scheme the fold carries G's response at every ghost back into the water. If the extended field steps down to 0 where the image is cut, the water at the cut's mirror distance feels that step. Waves longer than that distance see the step more than the wall, and reflect partly as a trough.
  - Measured on the straight wall below (RMS against the even image), with the image cut at 16, 32, 64 and 96 cells: 62 %, 61 %, 38 % and 17 % untapered.
  - With the cut tapered over 8 cells: 23 %, 39 %, 26 % and 12 %. With no cut (the whole wall, 128 cells, mirrors into the window): 2.2 %.
  - So 2.2 % is the best case, where the whole image fits. Where it is cut inside the wall (a narrow channel, a small pond, land deeper than the water opposite is wide), the tapered numbers for the cut's distance are what to expect.
  - So the default band reaches as far as the image does: `ghost_cells = 4096`, every wall cell whose mirror point lands in water, clamped to n.
  - Near the cuts that remain (a mirror point out of the window or among walls, a ghost dropped by the readers' limit), the weights taper to 0 over `ewave_taper_cells` = 8 cells as sin². So do the weights near seams: neighbouring ghosts whose mirror points are more than 2.5 cells apart, as at the two faces of a thin wall or a wall's medial axis, and ghosts whose nearest water lies on opposite sides. The second kind is the second review's: in a wall one cell thick both faces are equally near every cell, the 2.5-cell test missed it, and the cell held the front water's image right beside the water behind, so the wall let 88 % through (zero walls: 41 %). As a seam its image tapers to almost 0, and it leaks as a zero wall does.
  - With a taper, M is the row sum of RᵀR, 1 + Σ τ·w. The simpler 1 + Σ w would make the operator in the taper zone (1 + τ²)/(1 + τ) of the right one, as much as 17 % off.
  - The band sets accuracy, never stability, and costs only `set_mask` time and fill and fold work.
  - **The search.** Each wall cell's nearest water cell comes from a distance transform (Felzenszwalb and Huttenlocher's lower envelope down each column of the rows' distances), O(n²) whatever the band; the first version scanned 2·band + 1 rows per wall cell, O(n³) at the unlimited default (10 s at 1024², the second review measured). Ties, which set the mirror point as a mean, are found exactly within 32 cells of the shore (every row they can be in) and, deeper, among the nearest cells of a cell's neighbours. That reproduces the first version's ghost table on every test basin; on a 256² wavy coast about 0.5 % of the ghosts differ, all 50 or more cells inland and tapered.
- **Ghost limits keep it symmetric.** A ghost dropped is a whole row of R dropped, never one corner from the fold but not the fill. The GPU gathers each water cell's readers, so no water cell may be read by more than `ewave_max_readers` = 12 ghosts. Where more would read it (a small convex pond compresses its image), whole ghosts are dropped, farthest from the shore first, inside `ewave_wall_ghosts`, so the CPU and the GPU agree. The 64² test basins need at most 12 and drop none.
- **The parameters.** `ewave_params::walls` is `no_flux` (the default again) or `zero` (O8a's wall). `ghost_cells` now runs 0..4096 (at most n is used). `substep` (0 < Δt_s ≤ 1 s, default 1/60) sets the symmetric step's substep. Under `zero` the digest leaves all three out, so O8a's pin `0xa4933455f6946eca` stands bit for bit. Under `no_flux` it hashes them and `pending()`. The no-flux pin is re-measured, `0xaefff2ae097ed743` since the `damping_k2` filter's second form, on Windows; CI is to confirm it on Linux and macOS.
- **The GPU** (`EWave`) runs the same scheme with the same ghosts, weights, mass and readers' limit, all from the CPU functions at `setMask`, and the same substep, counted on the host in double with `ewave_substep_count` so it carries the CPU's remainder exactly. A substep is 4 log₂ n + 3 passes, 35 at 256²:
  - a fill pass: water texels write φ½, ghost texels the weighted sum of their four corners' φ½, fetched one by one since WebGL2 does not filter float textures, and other walls 0;
  - the forward butterflies (16);
  - a pass multiplying by κ̃/n², computed per texel from uniforms;
  - the inverse butterflies (16);
  - a gather pass. Each water texel adds to its own value up to 12 readers' weight × value. The readers sit two to an RGBA32F texel in six textures, each as the ghost's cell as a float index (an integer below 2²⁴, exact, read back by truncation: the first version's int(cell + 0.5) rounded odd indices up past 2²³, at n = 4096) and its weight, in the CPU table's order. The pass multiplies by 1/M (an R32F texture), writes h′ and φ′, and applies the substep's beach, sponge and global damping. The reader textures take 96 n² bytes: 6 MB at 256², 400 MB at 2048², 1.6 GB at 4096².
  - With `damping_k2`, the same four stages once more on h every substep. Then one pass brings the state home to the same texture (none when no substep ran and it is already there). A step of s substeps is s·35 + 1 passes (s·70 + 1 with `damping_k2`): at 256² a 1/30 s step is 71 at the default substep and 36 with `substep = 1/30`, against 34 without walls.
  - All passes are WebGL2-portable fragment shaders with the scissor test off. The gather shader's reader textures are generated per slot, since GLSL ES 3.0 cannot index a sampler array by a loop variable.
  - A source is one pass, since the walls hold no state to refill.

Measured in `ocean_ewave_test` and `cvcgl_ewave` (Windows, MSVC Release, the Intel Arc iGPU, other builds sharing the machine):
- **The exact Neumann operator.** The setup: a window of 2M = 64 cells along x, water on 0..M − 1, wall on M..2M − 1 (the window's wrap closes the channel), uniform along y, every wall cell a ghost.
  - Each ghost is its exact mirror cell with weight 1, and M = 2 on the water.
  - M⁻¹S applied to cos(πm(x + Δx/2)/L)·cos(2πqy/(nΔx)) gives κ times it to within 2.9·10⁻¹⁵, built in the test from `ghosts()` and `mass()`.
  - Stepped by the library at ωΔt_s from 0.25 to 0.74 rad, in steps of one to six whole substeps, deep and 1.5–2 m deep, the mode matches Störmer–Verlet's exact discrete solution over 400 steps. That solution is h exact and φ at x cot x. h is within 4.1·10⁻¹⁴ m on A = 0.3 m, and φ within 2.7·10⁻¹³ m²/s.
- **Stable: the review's probe as a test** (`WallsAreStable`). Broadband noise in h on the water of seven closed basins at 64² with 0.25 m cells: a disc of water 12 m across, the disc with an island, the disc with a breakwater four cells thick, a straight channel 5 m wide, a 45° channel, a wall two cells thick across the window, and a 2-cell wall 8 m long in open water. No damping, beach or sponge; 20000 steps each at three substeps: ω_max Δt_s = 1.8 (the limit, one a step), 0.125 s (steps of 0.25 s) and 1/30 s.
  - The modified energy never moved more than 1.3·10⁻¹⁴ from its start.
  - energy() never rose above its start, and dipped to 0.85 at the limit, 0.87 at 0.25 s and 0.991 at 1/30 s (the wobble).
  - Its means over the first and last quarters agree within 0.002: no trend.
  - Zero walls never rose above their start either.
- **Stable under a Δt that changes every step** (`VariableStepsAreStable`, the second review's probe as a test). The disc, the straight channel and the 2-cell wall in open water, from broadband noise: Δt 0.136 and 0.101 s in turn (2000 steps, default substep); Δt uniform in [0.6, 1.0] of the limit with the substep at the limit (20000 steps, 0 or 1 substep each); a frame Δt uniform in [1/60, 1/30] s at 0.05 m cells (15000 steps).
  - The modified energy never moved more than 5.7·10⁻¹⁵ from its start, and energy() never rose above it (lowest 0.85, at the limit).
  - The same frame jitter with `damping_k2`, the global damping, the sponge and a beach on every shore cell: energy() never rose above its start and fell below 10⁻⁵⁰ of it.
- **A crest reflects as a crest.** The setup is O8a's wall test: a σ = 0.5 m drop 6 m from a half-plane wall, to t = 13 s at 0.1 s.
  - The whole wall mirrors into the window (32768 ghosts). Without the sponge, the field before the wall is the even image to 2.9·10⁻¹⁴ RMS.
  - With the sponge it is 2.2 %, the sponge's h-only form; the first build gave 6.8 %. At Δt = 0.25 s it is 2.2 % (two substeps of 0.125 s), and at 0.5 s 2.1 % (four).
  - At the shore column, regressed on the incident wave, the field is 1.90 times it, and it peaks at 1.86 times the incident crest. The reflection against the mirrored free wave has slope +1.001.
  - Zero walls still give the odd image to 18.8 %, slope −0.68.
- **Lossless** (`NoFluxWallsAreLossless`). A smooth ring (σ = 1.5 m) bounces for 300 s (3000 steps of 0.1 s) about a closed 32 m straight channel, a 45° staircase channel, a disc and a pond with an island.
  - The modified energy stays within 7.5·10⁻¹⁵ of its start.
  - energy() stays in 0.980–1 (straight), 0.985–1 (45°), 0.983–1 (disc) and 0.985–1 (island). That is the wobble, not a loss.
  - The first build lost 0.8 %, 2.7 % and 8.1 % in the straight, 45° and disc basins.
- **`damping_k2` takes the same share whatever the walls** (`DampingK2TakesTheSameShareWhateverTheWalls`). The setup is a disc of water 36 m across on a 128² window of 0.5 m cells, 3.5 m deep (the crater lake's size), with a dent dropped off its centre and `damping_k2` = 2·10⁻³ m²/s (lsystem_coast's), over 300 steps of 1/30 s.
  - Of the energy at +0.5 s, the zero walls keep 75.5 % at +10 s and the no-flux walls 75.7 %. Under the filter's first form the no-flux walls kept 49.3 %. With lsystem_coast's global damping as well (0.02 /s), both keep 52 %, against 34 % under the first form.
  - At `damping_k2` = 10⁻¹⁵ the heights match the undamped run to 1.5·10⁻¹³ of the largest. Under the first form they were 99 % off: its smoothing alone.
  - **What the water's own Σh² shows.** Without damping H stays at 1.000, but the water's Σh² (the energy without M, as lsystem_coast's check reads it) settles at 0.76–0.86 of its first second once the ring reaches the banks, where zero walls keep 1.01–1.02. The disc's coast compresses its image over most of the water, so the mean M there is 1.31 (1/1.31 = 0.76). This is a share moved, not a loss: the reading wanders in that range from 10 s to 60 s without a trend. Near the banks the water's amplitude is that much lower than water-only energy would make it, which is the cut image's error (above) seen as energy.
- **Thin walls leak.** Behind a band with open water beyond it, as a share of the incident amplitude, at 0.25 m cells: 40.4 % at one cell, 26.8 % at two, 15.9 % at four, 10.8 % at 3 m, 5.7 % at 6 m and 3.0 % at 12 m.
  - For comparison: zero walls 41.1 %, 27.2 %, 14.3 %, 4.6 %, 2.8 % and 3.1 %; the first build 13.6 %, 4.5 % and 3.3 % at 3, 6 and 12 m, and the symmetric step before the opposite-sides seam 87.6 % at one cell.
  - So a wall up to four cells thick, mostly seam, leaks within 12 % of a zero wall; one a few metres thick, about twice a zero wall, because the front face's image sits inside the wall, nearer the water behind.
  - Without the seam taper the two faces' images meet in the wall, the fold carries one into the other, and the leak was 63 %, 53 % and 339 % at 3, 6 and 12 m. With a 4-cell seam taper it was 17 %, 10 % and 10 %.
- **GPU against CPU.** O8b's scenario with its wall no-flux and a round island 2.5 m across beside it. It has 32768 ghosts, the island's with fractional weights and zeroed wall corners, plus `damping_k2`, both dampings, a beach band, the sponge, a whole-field source and a drop; at a substep of 0.1 s it runs 24 steps of 0.2 s, then 24 of a Δt jumping between 0.03 and 0.17 s (steps of no substep and of two).
  - The two agree to 1.9–9.1·10⁻⁶ RMS for h and 1.5–8.0·10⁻⁶ for φ, and to 1.1–9.1·10⁻⁶ within 3 m of the island; they take the same substeps and carry the same remainder, bit for bit.
  - Passes, counted as rendered: 141 a step of two substeps (2 × 70 + 1, with `damping_k2`); a step of none renders nothing, or one pass home after a source.
  - The wall test at Δt = 0.25 s gives on the GPU the even image to 2.10 % and a reflection slope of +1.001, 71 passes a step, and agrees with the CPU to 1.2·10⁻⁵ RMS. Under zero walls the GPU still gives O8b's numbers: 21.7 % against the odd image, slope −0.54, and 5.6·10⁻⁵ against the CPU.
  - Broadband noise in the disc with an island, 10000 steps on the GPU of a Δt that changes every step, never raised the water's energy above its start: 0.887–1.000 with Δt uniform in [0.6, 1.6] of the limit and the substep at the limit (0 to 2 substeps a step), 0.998–1.000 with a frame Δt in [1/60, 1/30] s at the default substep.
- **Cost.**
  - CPU at 256², with a wavy coast over half the window: a 1/30 s step (two substeps at the default) takes 5.2–6.3 ms with the whole image (22398 ghosts; two runs), against 2.9 ms without walls. `set_mask` takes 11–13 ms (4 ms with a band of 16), and on the same coast scaled up 43–56 ms at 512² and 0.21–0.25 s at 1024².
  - GPU at 256² and 1/30 s, on the shared machine: 0.60 ms of GPU time with the coast at the default substep (71 passes), 0.30 ms with a 1/30 s substep (36), 0.27 ms without walls (34) and 0.35 ms for one OceanFFT cascade. The walls cost about their passes.

Not in O8c: a sub-cell shore (a smooth signed distance would place a staircase's shore better than half a cell from the nearest water cell); thin walls that stay fully no-flux without leaking (the lid model couples through a thin wall, the seam taper trades reflection for leak, and a wall a few metres thick leaks about twice what a zero wall does); output between substeps (the waves trail `time()` by up to a substep; interpolating needs the previous state); a gauge for the symmetric step (where every wall cell is a ghost with weights summing to 1, S annihilates constants, and a mean height makes φ's uniform part drift linearly without bound; it never reaches h, but costs φ's precision, on the GPU in float over hours; removing it needs a reduction pass on the GPU); volume in a closed basin is conserved only where every wall cell is a ghost (elsewhere the cut couples the mean level weakly to the deep wall); a mass that is uniform where the image is cut or compressed (M varies over such water, and the water by the banks carries M times its h² of the energy, so in a disc 36 m across the water's own Σh² settles at 0.76–0.86 of its start while H stays).

**As built (O9: bodies make waves).** The shared immersion code in `cvc::physics`, bodies as sources of the GPU window, the window following the camera's target with a mask from the shore, and the wake drawn in lsystem_coast's sea and crater lake.
- **C2's immersion code: `cvc/physics/immersion.h`.** `immersion_body` is a body's outline as the water sees it: its box, sphere or convex hull, or its `hull_mesh` where it has one (`immersion_body::of(body_desc)`, as the field step pushes on the hull in place of the cells). A convex hull's faces are found once from its points (every supporting plane through three of them, in a fixed order). `immersion_water` is a fluid field's surface, field time and box (`immersion_water::of(field, t)`). `add_immersion(body, position, rotation, water, grid, zeta)` adds the body's columns of immersed depth zeta over an `immersion_grid` laid out as `ewave`'s cells and returns their volume.
  - A cell's zeta is the mean, over 4 × 4 sub-columns across the cell, of the length of each vertical line inside the body and below the water's top over the cell's point: `fluid_surface::height` at the field time, capped by the field box's top, inside the box. The water's top is read once per cell and the shape once per sub-column. Zeta is point-sampled: as an edge crosses a cell it moves in steps of 1/16 of a column (a sub-column is in or out), not smoothly, and it agrees with the field step to that sampling (exactly only where both resolve the body, as for the level box below).
  - A box, a sphere and a convex hull are cut exactly by the vertical line (slabs, a chord, half-spaces). A hull mesh's inside is read from its crossings, entering where a face looks down and leaving where one looks up, so it need not be convex; the crossings are measured from below its highest vertex, so unbounded water (no surface, an open box) covers it whole. A point on an edge two faces share counts once (exactly antisymmetric edge functions and the top-left rule), and faces folding over at the outline decide alike.
  - Standard library, IEEE arithmetic and sqrt only, in the physics exact-math list, so a client with no physics world (arisrv, from a replicated pose) gets the same bits. `PhysicsImmersion.IsTheSameOnEveryPlatform` pins four bodies' columns on a swell at `0xb582404bf028ed63`, measured on Windows (MSVC); CI checks Linux and macOS.
  - `field_step` was not changed to call it (optional in the plan): the cells and the hull's triangles stay as they were, so every golden hash and pin is untouched. They sample the same shape, pose and surface, so the two agree to the cells' resolution (below).
- **C2's sources: `src/cvcGL/ocean/WakeSource.{h,cpp}`.** Each step, each body's zeta at its pose, then dh = −(ζ(t) − ζ(t − Δt)) smoothed by [1 2 1]/4 along x and y (it keeps the volume and takes out edges shorter than two cells) for `EWave::addHeights`. The first step only records ζ. When the window moves, the last ζ moves with it, and cells that enter take their first ζ as their last, so a body under the entering strip makes no source as it appears.
- **C3's window moves: `EWave::shift(di, dj)`**, one pass: cell (i, j) takes what was at (i + di, j + dj), the strip that enters is flat, the origin moves by whole cells and the sponge hides the strip. `EWave::setDepth` sets the window's one depth. The CPU `ewave` has no shift yet.
- **In lsystem_coast.**
  - **Two windows.** The sea's is 256² cells of 0.5 m (128 m) with C1's damping (γ = 2·10⁻³ m²/s and 0.02 /s), a 16-cell sponge and the window's wet cells' mean depth. The crater lake's is 128² cells of 0.5 m over the lake and a 3 m margin, 3.8 m deep (its mean), with no sponge, since the banks enclose it. `coast.wake.enabled` 0 at start leaves both out, shaders and all.
  - **D3: the sea's window is centred on the camera's target** (the `--look` target, or the camera's focal point): the point the view is about, not the eye, which may be far above the water, and there is no boat yet to follow. It moves only when the target is 16 cells (8 m) from its centre, by whole cells, and gets its new place's mask and depth. Under the default orbit camera, whose target is the island's middle, the window lies mostly over land.
  - **C6's masks.** The sea's is 0 on land (`water_bodies`' shore distance) and in the water ramps over the last 3 m to the shore from 1 down to 0.5 at the waterline, so beaches absorb (10 /s × (1 − m)). Where the shore is steep, it is 1 up to the waterline and a hard 0 past it, so cliffs reflect. Steep is the larger of the shore layer's K_r and the ground's slope ramped from 0.25 to 0.6, giving m = beach + (1 − beach) · steep. The ramp is narrow on purpose. A source in it is scaled by the mask and damped at that rate: under a first 8 m ramp to 0.3, crates dropped 5 m off the sand left no ring at all. The lake's mask is 1 in the lake (`water_bodies` at its level: depth > 0 and the lake's body) and a hard 0 on all its banks.
  - **Sources.** Each frame, each body at the pose it is drawn at (`pose_tracker::sample_one` at the frame's alpha) is immersed in the water physics feels at that pose's time. The view lags the world by one step, so that is the world's field time a fraction alpha past its tick k − 1, not the frame's world time: cutting the drawn pose against the sea 25 ms later put a false, wave-following source under each body. A teleport (the check's drop, the panel's "drop them again") makes the sources only record the columns until the drawn poses are the ones after it, so a body leaving its place leaves no false ring there. On the sea that is a second `ocean_surface` built from the same inputs as the physics field's, so it has the same values but its own one-time cache: reads at the drawn time do not evict the physics step's, which runs on its own thread at its own times. On the lake it is the still level. The windows then step by the frame's world time.
  - **C4's drawing.** Both water shaders read the window's RG32F state, bilinear by hand and faded to 0 over its outer 8 cells.
    - The vertex adds the height through a [1 2 1]/4 tent over its grid's spacing, which passes nothing at the grid's Nyquist wavelength. The sea's grid is 2 m, so it draws only the wake's long waves; a plain bilinear height aliased the 2–3 m rings into bright zigzag facets. The lake's grid is 0.7 m.
    - The fragment adds the slope to the normal, from per-cell central differences interpolated bilinearly, so the shading is continuous.
    - Foam, until §10's persistent foam texture exists: `coast.wake.foam` × (0.35 max(−∇²h, 0) + 2 max(|∇φ| − 0.15, 0)), at most 1. ∇²h is a sharp crest; |∇φ| is the surface's speed, which for a deep-water wave equals its vertical speed |∂h/∂t|. It whitens a splash and the first ring, not a gentle ring.
    - The knobs are `coast.wake.enabled`, `.foam`, `.vertex`, `.beach_m` and `.beach_mask`, plus a Wake section in the panel.
  - The wake is visual (C5): the debris feel nothing of it. `--debris-check` gives O7c's numbers unchanged: a crate carried 11.3 m in and left 2.4 m up the beach, a barrel 7.9 m into the shallows, a crate 10.5 m back out, and the lake's three at rest at 23.70 m. `--shore-check` still moves no vertex onto dry ground (the wake moves vertices only vertically).

Measured in `physics_immersion_test` (Windows, MSVC Release):
- **Analytic columns.** A level 1.2 × 0.9 × 0.8 m box 0.25 m deep in still water gives each cell the draft times its share of sub-columns in the footprint, to 10⁻¹⁵ m, at 1, 4 and 16 sub-columns a side. Turned 0.7 rad about z, a column inside the footprint is the draft and one outside is 0; tilted 25°, the column through the centre is 0.8/cos 25°, cut at the water as it should be.
- **One shape three ways.** The convex hull of a box's corners (and two interior points: 6 faces found) and the box's hull mesh, split into 4 × 4 squares a face and sampled on its shared edges and corners, give the box's columns to 10⁻⁹ m at a turned and tilted pose. A sphere's columns are its chords. lsystem_coast's barrel hull has its 22 faces.
- **The field step's volume.** These are field_load_on's immersed volume after one step, against zeta's at the pose and field time of that step (4 sub-columns of cells 5 cm wide).

  | body | still water | the swell (0.2 m, 4 s) |
  |---|---|---|
  | level box | 0.192000 m³, 0.000 % | −0.001 % |
  | tilted box (0.35 and 0.2 rad) | −0.12 % | +0.12 % |
  | sphere (r 0.4 m) | +0.68 % | −0.008 % |
  | barrel (lsystem_coast's convex hull) | −2.5 % | +2.9 % |
  | tilted box hull (12 triangles) | −0.001 % | −0.31 % |

  The level box agrees exactly because both samplings resolve it: its faces fall on the field's cell boundaries and between zeta's sub-columns. On cells of 0.25 m with 4 sub-columns the same box reads 0.292969 m³ against 0.27 m³ (+8.5 %). Elsewhere the difference is the cells' resolution, 8 along the longest side, worst for the barrel's round section. Those are the body's own samples; the columns are exact.

Measured in `cvcgl_wake` (Windows, the offscreen context on the Intel Arc iGPU, other builds sharing the machine):
- **Sources.** Σdh·dx² matches −(V(t) − V(t − Δt)) to 1.1·10⁻¹⁶ m³ for a turned crate pushed down and let up, and a body that stays still makes none.
- **Shift.** After a shift of (7, −5) cells, each cell holds exactly what was at its place in the world and the entering strip is exactly flat. Stepped on 2 s, the shifted window matches an unshifted one to 3.0·10⁻⁴ of the RMS height away from the edges (float round-off through the transforms, and the sponge sitting elsewhere for each).
- **Kelvin wake (Phase C's exit).** A box footprint 2 × 1 m with a 5 cm draft (ramped in over 8 s) moves at constant speed across still water. It is immersed by `add_immersion` and fed to EWave each step through `WakeSource`, 512² at 0.5 m with Δt = 0.2 s over 140 m, read back and measured as `ocean_ewave_test` measures the CPU's.

  | speed | Froude number on 2 m | 30 % edge | 40 % edge | 50 % edge | row maxima |
  |---|---|---|---|---|---|
  | 2.5 m/s | 0.56 | 19.43° | 19.43° | 20.02° | 20.51° |
  | 3.0 m/s | 0.68 | 19.61° | 19.34° | 19.79° | 17.82° |

  The reference is arcsin(1/3) = 19.47°, and each edge must be within 2°. The box reads closer to it than O8a's Gaussian. Each run takes 1.0–1.6 s.
- **A rock's shadow.** A wavemaker line 40 m up-wave sends waves 4 m long at a disc 8 m across masked to 0. Over 20 s, 8–20 m behind it within 2 m of the axis, the RMS height is 0.0116 m, 23 % of the same run without the rock (0.0511 m). 12–16 m to the side it is 100 %.

Measured in lsystem_coast (`--debris-check --wake-check --look -128,-48,9,-103,-14,0`, the default 1.2 m swell and 9 m/s wind, synthetic clock at 30 fps; the same numbers every run). Re-measured under O8c's no-flux walls (the banks and the sea window's land), after the fix to O8c's `damping_k2` filter:
- **The lake keeps its waves.** At 20 s the check drops a crate from 2 m onto the lake's middle (9.5, 0.5), beside a twin lake window with absorbing banks (the mask ramping to 0.2 over the last 3 m) fed the same sources.
  - The drop itself adds about 150 J: the absorbing twin goes from 11 J at 20.03 s, the crate still falling, to 170 J at +1 s and 160 J at +2 s. (Before the teleport fix the crate vanishing from where it floated made a second ring the frame after the drop, while the crate was still 1.6 m up: the twin jumped from 15 J to 79 J by 20.03 s.)
  - Of the lake's energy 2 s after the drop, 48 % is left 10 s later and 34 % 20 s later. With the absorbing banks it is 13 % and 1 %.
    - Under O9's zero walls it was 55 % and 34 %. Under the no-flux walls with the `damping_k2` filter's first form it was 20 % at 10 s: that filter smoothed the water the banks' image reaches, once a substep (O8c).
    - The check reads the water's Σh², without O8c's mass M. Under no-flux banks that reads about 0.85 of the energy the walls keep once the ring reaches them (O8c, `DampingK2TakesTheSameShareWhateverTheWalls`), which accounts for 48 % against 55 %.
  - Within 1.5 m of the drop the splash peaks at 0.090 m RMS at +0.6 s. From +8 to +30 s the RMS height there is 0.0066 m with the banks reflecting and 0.0016 m with them absorbing: what comes back from the banks.
  - Before the drop, the first three bodies' splashes at 0.75 s fill the lake to 444 J at 2 s and decay to 182 J by 18 s, against 16 J with absorbing banks.
  - The check fails (exit code 5) when a window with bodies in it has no waves, when the drop adds less than 20 J to the absorbing twin by +2 s, or when the reflecting lake keeps no more than twice the twin's share at +12 s. With no `--look` the sea's window lies over land, no body is in it, and the check says so rather than failing.
- **The sea.** In the swell the six bodies at sea ride the long waves but not the short wind waves. Their columns change as those pass, so they scatter them continuously. The window's energy rises from 1.3 kJ at 2 s to 2.7–2.8 kJ from 12 s to 20 s, 3.6–4.5 kJ from 24 s to 44 s and 5.9–6.6 kJ from 46 s (under O9's zero walls 2.8–3.0, 4.0–4.6 and 6.1–6.7 kJ): about 0.9 cm RMS over the 128 m window at the end, with the largest heights of 0.09–0.41 m in the bodies' own footprints, where the wake cancels the crest they keep out. Read against the sea at the frame's time instead of the drawn poses' (before the review's fix), it reached about 8.3 kJ by 46 s.
- **Frames.** On a calm sea (swell 0, wind 2–3 m/s):
  - At 2.5 s the crate nearest the camera sits in a foamy splash ring, and the barrels have rings of their own. By 5 s the rings have spread several metres. With the narrow beach ramp, the two crates nearest the sand (in 1.5 and 2 m of water) have rings that run in to the sand. At 7 s their arcs cross between the crates and the waterline, where the shore foam band hides the line itself. Under the first 8 m ramp those crates had no rings.
  - On the lake (`--look 24,-20,31,10,1,23.5`), at 2.5 s each of the three bodies has its own ring. By 4.5 s the rings are about 5 m across and overlapping, and by 11 s the lake shows only faint crossing ripples. At 25 s the check's dropped crate has a ring of its own.
  - In the swell at 60 s, the bodies in the surf radiate rings that cross the near water. The rings are smooth after the tent filter and the continuous slope, but strong near the bodies.
- **Cost** (Windows, Intel Arc iGPU, a loaded 22-core machine):
  - The wake costs 5.0–5.8 ms of CPU a frame. Of that, 3.0–3.8 ms is the sources: immersion takes 2.4–3.3 ms, of which 1.9–2.3 ms is the sea's band computed at the frame's time and 0.5–0.9 ms the sea's heights under the bodies and their columns. The rest is the uploads and the passes.
  - Both windows take 1.8–2.0 ms of GPU time a frame (a timer query; 2.5–2.7 ms with the check's third window).
  - Offscreen frame loops of 300 frames with the physics on its own thread took 89 and 140 ms a frame with the wake and 81 and 75 ms without. The machine was shared and the frame rate swung from 7 to 17 fps within a run, so these say less than the wake's own times.

Not in O9:
- **The sea's band at the drawn time.** About 2 ms of CPU a frame (2.8 ms on a more loaded machine in the review's re-run). It could come off the render thread.
- **A hull's drawn Kelvin wake.** There is no boat in lsystem_coast yet (O11). The Kelvin wake is the test's, and the sea's window follows the camera's target, not a boat.
- **Wake-on-body coupling** (C5, D6).
- **Persistent foam** (§10.2); foam is the simple term above.
- **A shift for the CPU `ewave`.**
- **The browser.** Not measured.
- **`world::set_state` with zero velocity.** On Jolt it does not wake a sleeping body, though `world.h` says it activates it: `SetPositionRotationAndVelocity` activates only for a non-zero velocity. The check drops its crate at 0.5 m/s. The panel's "drop them again" may leave sleeping lake bodies where they are.

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

**As built (O11a: the hull model).** `cvc/physics/hull.h`; the step is in `field_step.cpp`.
- **How a body declares one.** `body_desc::hull` is an optional `std::shared_ptr<const hull_mesh>`: vertices in the body's frame and triangles wound counter-clockwise seen from outside, apart from the collision shape. `box_hull(half)` builds a box's 12. `add_body` refuses a hull on a body that is not dynamic, an index out of range, a vertex that is not finite and triangles that enclose no positive volume, and keeps its own copy of the mesh, so the cached normals and the digest always describe what is simulated. A hull replaces the cells in fluid fields only; media and flows still push on the cells, and bodies without a hull are untouched.
- **Per triangle, per fluid field.** The water's top is taken once per vertex: the free surface's height (`fluid_surface::height`), capped by the box's top, or the box's top without one. A vertex outside the box in x and y is dry. Each triangle is clipped to depth > 0, giving 3 or 4 corners. Over the submerged polygon's fan, depth is linear, so these are exact:
  - **Hydrostatic pressure.** ρ·g·∫depth dA along the inward normal, with its moment ∫depth·(x − com) dA = A/12·(Σdᵢ·Σxᵢ + Σdᵢxᵢ) per piece. That gives the centre of pressure, so a hull floats, trims and rights itself. The immersed volume is −∫depth·n_z dA (the divergence theorem).
  - **Viscous resistance.** ½ρ·C_f·S·|u_t|·u_t, with the ITTC-57 line C_f = 0.075/(log₁₀Re − 2)². Re = |u|·L/ν per triangle, L being the hull's length (its x extent unless given) and ν defaulting to sea water's 1.19·10⁻⁶ m²/s. Re is floored at 10⁵, where the line gives 0.0083.
  - **Pressure drag.** ½ρ·C_d·S·|u_n|·u_n·n on faces the water meets, C_d being the field's `drag_coefficient`. `suction_coefficient` (default 0) acts on faces it leaves.
  - **Slamming.** A face entering the water (u_n < 0) whose wetted area grew since the last step sets the newly wetted water moving. Its added-mass rate is `slam_coefficient`·ρ·l·ΔS/Δt times u_n·n, with l = ∛(hull volume) a mesh-independent patch size. At the default 0.5, a 1 m² flat bottom gets 370 kg of added mass, against von Kármán's 480 kg for a 1 m² disk. The term is linear in u_n and integrated implicitly with the rest, so however fast a face enters, it can at most bring that face's normal speed to the water's in one step. The last wetted areas are world state (below).
  - **Damping.** The field's `damping` c gives the cells' rate, c·V_sub, against the vertical part of u only. It is shared among the downward faces by the column of water over each, depth·S·|n_z|; upward faces under water take none, so a submerged hull is damped no more as it sinks deeper. Unlike the cells' damping, it does not slow surge (below), so towing resistance is the hull's own.
  - **Relative velocity.** u is the water's velocity at the submerged centroid (`fluid_surface::velocity`, or the velocity volumes) minus the hull's point velocity there.
- **Integration.** All velocity terms are c_i times a projection of u. As in the cell model, they are summed and scaled by (1 − e^−x)/x with x = Σc_i·Δt/m, so a step never carries the body past the water's velocity. The horizontal parts are scaled in the rate without the damping, which acts on z alone. The drag torque is also scaled by its rotational rate, Σc_i·|arm|²·Δt over a low estimate of the least moment of inertia, whichever scale is smaller. The estimate is half a solid box's least moment over the hull's bounding box, or over the collision shape where that is lower (a sphere's 0.4·m·r², halved), since the body's true inertia comes from its shape. It keeps the scale toward less damping while a convex-hull collision shape roughly fills its bounding box. The hydrostatic push is explicit, as the cells' buoyancy is. It is stable while √(ρ·g·A_wp/m)·Δt < 2, and a lighter hull in a denser fluid oscillates and gains energy in heave. At the default 100 Hz in water, that limit is a hull of more than about 0.25 kg per m² of waterplane.
- **State.** With any hull, `save()` appends a `CVCHULL1` section: each hull body's id, the hull's FNV digest (vertices, triangles, length, ν and both coefficients) and its triangles' last wetted areas. `state_hash()` covers all of it, and `restore()` refuses a world whose hulls differ. A world without hulls saves exactly as before: `PhysicsFluidSurface.CrossPlatformGoldenHash` (`f0085b4212e1a8c0`), `PhysicsOceanSurface.CrossPlatformGoldenHash` (`315e7d5f0aed8af0`) and `physics_replay_test` are unchanged. `snapshot()`, its JSON and `physics_codec` do not carry hulls. pycvc does not bind them yet.
- **Tests** (`physics_hull_test`, MSVC Release; each a 1 × 1 × 0.4 m box at density 500 unless named):
  - **Buoyancy.** The first step's load is Archimedes': ρ·g·V to 1e-8 N, with no torque.
  - **Centre of pressure.** Heeled 10° and 20° at rest, the first step's torque is the wall-sided righting moment ρ·g·V·sin θ·(GM + BM/2·tan²θ): 110.056835 against 110.056832 N·m, and 230.938272 against 230.938258 N·m, for both 12 and 432 triangles (within 1e-7; the engine keeps the rotation in floats).
  - **Floating.** Half draft is 0.20000 m with 12 triangles, with the faces split into 432, and on the cells.
  - **Righting.** Tilted 20°, the box is at 0.29° after 8 s. A 0.4 × 0.4 × 0.8 m box, whose centre of mass is above its metacentre, tips from 5° to 90°.
  - **Towing.** A 2 × 0.8 × 0.5 m hull meets 106.7 N at 1 m/s and 434.5 N at 2 m/s, a ratio of 4.07. Bow pressure drag (100 N·v²) and ITTC-57 friction on its 2.6 m² of bottom and sides predict 105.5 and 419.0 N. At 2 m/s the bow's drag trims it about 0.01 rad bow down, wetting about 4 % more bow.
  - **Dropping.** Dropped from 1 m, the box carries 2157.5 J above rest. Its energy never rises above that after the slam, and it is at rest at half draft after 9 s.
  - **Sinking.** At density 1200 the box sinks at 0.1984 m/s both 1.5 m and 3.5 m down, as the cells do (0.1985 m/s). The excess weight against c·V·v and the bottom's pressure drag gives 0.1915 m/s; the drag's implicit scale applies it about 4 % short.
  - **A light hull.** An 8 kg slab in water with C_d = 100 (x ≈ 4 in its first step, where an explicit step would reverse it threefold) goes from 8 m/s to 0.10 m/s in that step without reversing. Braking its bow pitches it, and the pitching rocks it about 3 cm/s to and fro in surge as it stops. A spin slows without reversing.
  - **Swell.** On the 0.2 m, 4 s `wave_train_surface` it heaves with the surface at its centre, at correlation 0.9995 and height ratio 1.03.
  - **State.** Two runs hash alike. The hull, a different `slam_coefficient` and the cells all change the hash. A save restores and steps alike, and a world with another hull, no hull, or a state without the hull is refused. `add_body` keeps its own copy of the mesh: scaling the caller's mesh and doubling its triangles after `add_body` changes neither the run nor the hash.
  - **Pinned.** `PhysicsHull.CrossPlatformGoldenHash` is the box hull and a ball on the swell after 300 steps: `2f71a1fe229afa75`, measured on Windows MSVC; to be confirmed by CI on Linux and macOS.
- **Cost.** These are whole steps on the swell, Jolt included, with a box and a ball, MSVC Release on a loaded 22-core machine. The box on its 192 cells takes 0.06–0.16 ms, the 12-triangle hull 0.03–0.09 ms, and the 432-triangle hull 0.09–0.24 ms; the ratios hold from run to run (the hull about 0.6 of the cells, the 432 triangles about 1.6). The hull reads the surface once per vertex, and the cells once per cell.

**D3. Propulsion and control.**
- Thrust is applied at the propeller.
- The rudder's lift is ½·ρ·A·C_L(δ)·v², applied at the rudder.
- An outboard motor vectors its thrust.
- Throttle and rudder are actuators like `vehicle_controls`.

**As built (O11b: boats).** `cvc/physics/boat.h`; the step is in `boat.cpp` (`boat_step.h`), called from `world::step` after the fields and vehicles and before the engine step. `docs/PHYSICS_API.md`, "Boats".
- **The API mirrors vehicles.** `world::add_boat(boat_desc, position, rotation)` returns a `boat_id`; there are `remove_boat`, `boat_count`, `set_boat_controls`, `boat_state` and `get_boat`. A `boat_desc` holds:
  - the hull, a `body_desc` with its collision shape and (usually) its `hull_mesh`, which must be dynamic;
  - a `propeller_desc`: position and thrust direction in the body frame, `max_thrust` ahead and `max_reverse_thrust` astern, `max_inflow_speed`, and `outboard_max_angle` (0 for a fixed shaft);
  - a `rudder_desc`: position, area, `lift_slope`, `stall_angle`, `drag_coefficient`, `cross_flow_drag` and `max_angle`;
  - `actuator_time_constant`, for both actuators.
  `boat_controls` is { throttle, rudder }, each in [−1, 1]; + rudder turns left. The hull is an ordinary body that the fluid fields float and resist (O11a). `remove_body` refuses it. Boats work on any backend, since they need only `add_force_at` (O10).
- **Every step, in boat id order, standard library and `det::` math** (`boat.cpp` is in the exact-math source list):
  - **Actuators.** Each moves as a first-order lag, a += (c − a)(1 − e^(−Δt/τ)).
  - **Propeller.** It pushes only while its point is under a fluid's free surface. `field_set::water_at` gives the first fluid in id order over the point: its surface height (or box top), density and water velocity. T = |throttle|·T_max·(1 − u_a/u_max), clamped to [0, |throttle|·T_max], where u_a is the propeller's velocity through the water along its push. An outboard swings the thrust about body z by rudder × `outboard_max_angle`.
  - **Rudder.** The flow is the water's velocity at the rudder less its point velocity v + ω × (p − com), taken in the body's horizontal plane. The angle of attack is measured from the deflected trailing edge, and wrapped so that a rudder works reversed astern. Lift is ½ρAV²·C_L normal to the flow, with C_L = `lift_slope`·α up to the stall; past it the blade is a flat plate, C_L = `cross_flow_drag`·sin 2α / 2. Drag is ½ρAV²·(`drag_coefficient` + `cross_flow_drag`·sin²α) along the flow.
  - Both forces go to the backend's `add_force_at` at their world points, and only when non-zero, so a still boat stays asleep. The centre of mass is the fields' (`field_set::center_of_mass`).
- **State.** With boats, `save()` appends a `CVCBOAT1` section after the hulls'. It holds each boat's id, its hull's body id, an FNV digest of the propeller, rudder and time constant, the controls, the actuators and the last step's outputs, so `state_hash()` covers them. `restore()` refuses a world whose boats, hull ids or descriptions differ. As with the vehicles' and hulls' sections, that check comes after the engine's bodies are restored, so a refused state leaves the boats as they were but not the bodies. A world without boats saves exactly as before: the `CrossPlatformGoldenHash` of `PhysicsJolt` (`70b2cc0bd34ea4be`), `PhysicsFluidSurface` (`f0085b4212e1a8c0`), `PhysicsOceanSurface` (`315e7d5f0aed8af0`) and `PhysicsHull` (`2f71a1fe229afa75`), and `physics_replay_test`, are unchanged. `snapshot()`, its JSON, `physics_codec` and pycvc do not carry boats yet.
- **Tests** (`physics_boat_test`, MSVC Release). The boat is a 4 × 1.5 × 0.6 m box hull of 900 kg (192 triangles), floating 0.15 m deep, with no body damping, in water with C_d = 1 and damping 10⁴. Its propeller and rudder sit 0.2 m under the waterline at the stern: 2000 N ahead, 1000 N astern, u_max = 8 m/s, a 0.12 m² rudder, τ = 0.5 s.
  - **Terminal speed.** Full throttle settles at 3.1915 m/s, where the inflow is 3.1914 m/s. Thrust is 1202.14 N (2000·(1 − 3.1914/8) exactly), against 1196.01 N of hull resistance and 6.111 N of rudder drag: an imbalance of 0.021 N, with mean acceleration 4·10⁻¹² m/s². The heading stays 0 within 10⁻³ rad.
  - **Astern.** Full astern reaches −1.806 m/s after 30 s, with thrust −774 N as the inflow from astern grows.
  - **Turning.** At full throttle, each steady turn is measured over one whole circle, with the yaw rate within 1 % of its mean and the circle's diameter 2U/r within 2 %:

    | rudder (deg) | yaw rate (rad/s) | drift (rad) | radius U/r (m) | measured diameter (m) |
    |---|---|---|---|---|
    | 6.9 | 0.01518 | 0.108 | 211.15 | 422.30 |
    | 15.5 | 0.05796 | 0.222 | 55.74 | 111.48 |
    | 30.9 | 0.17593 | 0.395 | 18.23 | 36.46 |

    The speed holds at 3.21–3.23 m/s. Opposite rudder gives the mirror turn within 10⁻³. The hull drifts into the turn until the rudder sees almost no angle of attack: 0.0017 rad at 6.9° and 0.0082 rad at 15.5°.
  - **Dry propeller.** With the propeller over the deck, the thrust is 0 and the boat does not move (3·10⁻¹⁶ m/s after 10 s).
  - **Outboard.** At throttle 0.2 and full steer from rest, an outboard with a 0.5 rad swing has turned the boat 0.692 rad after 3 s. The same boat with its rudder, and no slipstream over it, has turned 0.062 rad.
  - **Swell.** On a 0.3 m, 5 s train along x (39 m long, slope amplitude kA = 0.048), the bow's rise follows the surface's slope at the centre, at correlation 0.9998, with pitch from −0.047 to 0.050. The propeller stayed wet throughout.
  - **State.** Two runs hash alike, whether stepped 250 steps in one call or one step per call. The thrust rating and the controls change the hash. A save restores and steps alike, including `boat_state`'s outputs. A world with another boat, a world with the same hull as a plain body, and a state without the boat are refused.
  - **Null backend.** Here the refusals are checked, along with a refused boat leaving no hull behind, `remove_boat` taking its hull, the actuator lag (1 − e⁻¹ after one τ, to 10⁻¹²) and the rated thrust at rest.
  - **Pinned.** `PhysicsBoat.CrossPlatformGoldenHash` is the boat at throttle 0.8 and rudder 0.5 on a 0.2 m, 4 s swell after 300 steps: `bb8e1b22f58136b2`, measured on Windows MSVC; to be confirmed by CI on Linux and macOS.
- **For D4.** `dyn_boat_plant` can drive `set_boat_controls` from a nav command and read `boat_state`'s speed, heading and yaw rate. An energy ledger would add the thrust's power, T·u_a, and the rudder's drag work. Neither is built here.

**D4. A boat plant.** `dyn_boat_plant` sits beside the 4-wheel and bike plants, takes a nav command through a driver model, is selectable by `plant_host`, and keeps the same energy ledger. This puts watercraft in GRL-SNAM's reach.

**D5. Drawing.**
- **No water inside the hull.** The water shader discards inside the hull's waterline footprint, passed as a small signed-distance texture or polygon uniform. Without this, the sea shows through the deck.
- Bow spray uses the spray particles (§10, item 5), spawned where the wake breaks.
- Propeller wash goes into the foam texture.
- The hull darkens where it is wet.

**D6. An Ariadne node,** `boat` (or `vessel`): a hull mesh, plant parameters, and a body in the scene's physics world.

**As built (O11c: the boat in lsystem_coast).** D5's water exclusion and the drivable boat, in `examples/cvcgl/lsystem_coast.cpp` only; no library code changed.
- **The skiff.** A 5 m open skiff with an outboard. Its hull is a V bottom with a hard chine, 1.9 m in beam, defined as sections from the transom (x = −2.4 m) to the stem (x = 2.6 m) and interpolated to 21 stations 0.25 m apart.
  - **Physics.** The `hull_mesh` is those sections joined and closed by the transom, the stem and a flat top at the sheer: 378 triangles and 5.34 m³. The collision shape is the convex hull of its vertices. It weighs 700 kg (hull, motor and a driver) with no body damping, and floats with 0.70 m³ immersed.
  - **Drawing.** The drawn hull is the same sections, painted: white topsides, a blue stripe, dark red below. Above the sheer, which the water never reaches, it has a sand-coloured foredeck, white rails, a teak cockpit sole 0.12 m above the origin (about 0.25 m above the waterline at rest), a console and the outboard. The motor does not swing as it steers.
  - **Propulsion.** A `boat_desc` added with `world::add_boat` to the debris' world, after the debris, so their ids and `--debris-check` are unchanged. The outboard gives 2.5 kN of bollard pull, falling to 0 at 11 m/s of inflow, and swings 0.5 rad to steer. Its leg is a 0.05 m² rudder that swings with it. τ is 0.4 s.
  - **Start.** On the lee (east) side at (110, 30), heading south over 8–11 m of water. `coast.boat.enabled` 0 leaves it out (default 1), and `coast.boat.start_x`, `.start_y` and `.start_heading` place it.
- **The helm.** `coast.boat.throttle` and `coast.boat.rudder`, each in [−1, 1], go to `set_boat_controls` every frame.
  - Keys: I / K raise and lower the throttle (0.6 a second) while held. J / L hold full rudder left / right, and the rudder comes back to 0 when they are let go. C toggles the chase camera. WASD, Tab, F, H and the arrows still move the camera.
  - The panel's Boat section has the two sliders, the chase checkbox, "put it back" and a readout of speed, heading and thrust.
  - The frame loop publishes `coast.boat.speed`, `.heading`, `.x` and `.y`, so a script can drive it.
- **The chase camera** (`coast.boat.chase`) sits `chase_back` (14 m) behind and `chase_height` (5 m) above the still level, with `chase_side` across, along the boat's heading eased over 0.9 s. It looks 4 m ahead of the boat. While it is on, the sea's wake window follows the boat, not the camera's target (D3), centred 24 m astern so the wake is in it. It moves 16 cells at a time as before (96–100 moves in the 110 s check).
- **No water inside the hull (D5).** The sea's fragment shader discards a fragment whose surface point lies inside the hull. The test runs in the boat's frame, from its drawn pose each frame, against six of its sections as uniforms (twelve `vec3`s: the node takes no arrays). Across each section it uses the V from the keel to the chine, the straight topsides to the sheer, and the sheer's breadth for 0.5 m over it. Water under the keel, as the boat leaps a crest, is still drawn, and so is green water well over the deck. `coast.boat.keep_dry` 0 turns it off. In the 9 m/s sea at 4 s, from a low chase camera, the two renders differ in 11 123 pixels: with it off the sea fills the cockpit, and with it on the teak sole shows.
- **The wake.** Each frame the boat's drawn pose is immersed (`add_immersion` through its `hull_mesh`) into the sea's window beside the debris, as in O9. The trail's foam is O9's wake foam.

Measured by `--boat-check`: 3300 frames on the synthetic clock at 30 fps (110 s), the chase camera on. It runs on a moderate sea unless `--set` says otherwise: the default 1.2 m, 10 s swell and a 4 m/s wind sea at `wave_amp` 0.3, about 0.4 m Hs. Two runs gave the same numbers to the last digit.

The run:

| s | what the boat does |
|---|---|
| 0–3 | settles |
| 3–21 | full throttle south, holding its heading |
| 21–41 | 3.5 m/s (a Froude number of 0.5 on 5 m), set by a speed loop |
| 41 on | full rudder and full throttle through one circle, measured from 47 s |
| then | waypoints round the south of the island to the south-west shore (slope about 0.5, which the shore layer reflects), and north-north-west along it |

- **Speed.** 6.85 m/s (13.3 kn) through the water over the last 4 s of the lee run, and 9.14 m/s at most. In still water (wind 0.5 m/s, no swell) it settles at 7.93 m/s on 697 N of thrust, with 0.631 m³ immersed and 1.2° bow down.
- **Turning circle.** One circle took 16.6 s. Across the track it measured 33.7 by 29.3 m (mean 31.5 m). 2U/r gives 31.0 m (5.87 m/s over the ground, 0.378 rad/s).
- **Motion.** RMS about the means:

  | stretch | heave | pitch | roll | sea under it | swell incident / reflected |
  |---|---|---|---|---|---|
  | lee run, 9–21 s | 0.128 m | 1.52° | 3.07° | 0.122 m | 0.09 / 0.02 m |
  | along the south-west shore, 10.4 s from 99 s, 18.9 m off it at the nearest | 0.329 m | 1.55° | 3.43° | 0.334 m | 0.39 / 0.30 m |

  The incident and reflected amplitudes are the shore layer's means along the track. Along that shore the boat heaves on the reflected swell. A 156 m swell barely pitches a 5 m hull, so pitch and roll come from the wind sea and change little. The largest excursions from the means were 4.2° of pitch and 7.0° of roll.
- **The wake's angle.** From the sea's window read back every frame, as RMS heights on rows 8–44 m astern in the boat's frame:

  | | speed | Froude number on 5 m | row maxima | 30 % edge | 40 % edge | 50 % edge | rows' ends against the row's largest |
  |---|---|---|---|---|---|---|---|
  | slow, 31–41 s | 3.49 m/s | 0.50 | 20.40° | 38.6° | 35.4° | 32.7° | 19 % |
  | full throttle, 12–21 s | 7.59 m/s | 1.08 | 10.34° | 15.3° | 14.3° | 13.4° | 1 % |

  At Froude number 0.5 the line of the row maxima, the cusps, is 0.9° from Kelvin's 19.47°. The outer edges read wider there: the waves the boat scatters off the swell hold 19 % of the cusps' RMS at the rows' ends. At full throttle the wake narrows to 10–15°.

  The window has one depth, the mean water depth under it (`setDepth`): 8.4 m on the slow stretch and 7.0 m at full throttle, depth Froude numbers U/√(gh) of 0.39 and 0.92. Near 1, linear finite-depth theory changes the wake angle a lot, so the full-throttle reading could have been depth's doing. The run was repeated with the window forced deep (`--set coast.wake.depth=1000`, depth Froude numbers 0.04 and 0.08), with the boat's motion unchanged to the last digit. Full throttle then reads 10.68° for the row maxima and 15.2 / 14.7 / 13.7° for the edges, and 3.5 m/s reads 20.78° and 39.7 / 36.8 / 34.2°, each within 1.5° of the run at the window's own depth. So the narrowing comes from the boat's speed against its length, as ship wakes narrow above a Froude number of about 0.5 (Rabaud and Moisy 2013), not from the shallow window. `--boat-check` prints the window's depth and depth Froude number with each wake reading.
- **Frames** (the chase camera; a high one at 16 m up and 26 m back):
  - At 19 s, at speed, a narrow V of foam spreads from the stern, and there is bow foam at the shoulders.
  - At 39 s, at 3.5 m/s, the high camera shows the Kelvin pattern: transverse crests in foam chevrons inside the V.
  - At 55 s, in the turn, the boat heels into it, and its wake piles into a saturated white patch where the circle crosses itself.
  - At 102 s, along the south-west shore, the debris and the beach's surf are ahead.
- **The default sea is too rough for it.** At 9 m/s and `wave_amp` 1.2 the cascades make a sea of 1.5 m RMS (6 m Hs) off the lee shore; at 4 m/s it is 1.5 m Hs. In that sea the skiff heaves 2.1 m RMS and pitches and rolls 24–25° RMS, and makes no way after 12 s. The check fails as it should (exit code 6: no full circle).
- **Exit code 6** means the boat sank (its whole hull under water, or its origin 1 m under the water over it, for 2 s), never exceeded 1 m/s, or did not turn a full circle.
- **The other checks are unchanged.** `--debris-check --wake-check` (1800 frames) gives O7c's and O9's numbers: crates carried 11.3 m in (2.38 m up the beach) and 10.5 m back out, a barrel 7.9 m in, the lake's three at 23.70 m, 55 % and 34 % of the lake's energy left against 13 % and 1 %, and a 0.087 m splash. `--shore-check` still moves no vertex onto dry ground. `cvcgl_ewave`, `cvcgl_wake` and `physics_boat_test` pass.
- **Cost** (Windows, Intel Arc iGPU, a shared machine): 39–96 ms a frame over the check's runs. The wake took 3.8–6.7 ms of CPU a frame, of which immersion was 2.7–4.7 ms with the boat's 378 triangles beside the nine debris (O9: 2.4–3.3 ms), and 1.0–1.8 ms of GPU.

Not in O11c:
- **Spray** (§10 item 5): left out.
- **Propeller wash in a foam texture,** and the hull darkening where wet.
- **The motor drawn swinging.**
- **The sea's height for its wind.** The cascades make about 4 times JONSWAP's Hs (6 m at 9 m/s, 1.5 m at 4 m/s), which is why the check lowers `wave_amp`. Not changed here.
- **Wake-on-body coupling** (C5, D6): the boat feels none of its own or the debris' waves.
- **`dyn_boat_plant`** (D4) and the Ariadne node (D6).
- **The browser.** Not measured.

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
