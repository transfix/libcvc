# Generated worlds: one template, switchable generators, the conference's world

**Status:** plan, revision 3 (2026-11-02). G0 landed with this document; first passes of G2 (the
street model) and G3 (building placement) landed in revision 2; G1 (the terrain-source interface
and the generator switch) in revision 3; the rest are the plan.

**Goal.** One generated world that every app builds on: the terrain lab, the Ariadne conference's
rooms (arisrv + arigl, the cyprty browser client), and the next demo. It is a document template
(`examples/ariadne/documents/terrain_lab.ari`) on libcvc's fastest render path, with a world that
switches between generators at run time, and libcvc's water whatever the generator.

## G0 — what landed

- **`cvc::world::generate_world`** (`inc/cvc/world/generated_world.h`): terrain_lab's world, GL-free
  and deterministic. The relief and L-system prop scatter (`world_model::generate`), the props as
  merged meshes, the roads-first city (`generate_roads`, `layout_lots`, the Austin archetypes or,
  since G3, procedural buildings), the surface texture (`paint_surface`), the water below
  `water_level_m` (`water_field`), and the world as a `world_geometry` for physics and nav.
  `examples/cvcgl/terrain_lab.cpp` now draws it (the same frame as before the move).
- **The `generated_world` scene node type** (`src/cvcGL/ariadne/world_nodes.cpp`), in the cvcGL
  extension bundle, so arigl, the browser gallery and the conference client all have it. Its
  `world.*` keys are driven by the reusable `cvc://components/world_controls.ari`.
- **The fast render path, declarable.** demo3-csma (CVC-DBG's Austin convoy demo) is our fastest
  cvcGL scene. It is fast by discipline, not custom GLSL: few merged meshes, nothing re-uploaded, few
  shadow bakes. Every piece of that discipline an `.ari` could not say is now a scene key:
  - `shadows: { focus: camera | { follow: camera, min_radius, max_radius, per_distance, lead, lift,
    ground } | { center, radius } }`: `SceneGraph::setShadowFocus`, re-aimed by `tick_scene`;
  - `shadows: { softness }`: `setShadowSoftness`;
  - `casts_shadow: false` on any node: `GraphicsNode::setCastsShadow`;
  - `material: { opaque: true }`: `GeometryNode::setForceOpaque`, new (no per-texel alpha scan);
  - a rig's `wash:`: removes the downlight ring and its shadow-map samplers.
- **`terrain_lab.ari`**: the lab as a document on those keys, with demo3's soft afternoon look.
  `cloud_sky` takes `extent` / `base` / `top`, so its slab fits a world of any size (since G6 a
  deck of clouds over the whole world, below).

## G1 — a generator interface and the switch

**Landed (revision 3).** The terrain lab's world is three choices, the first two switchable by name
in the panel:

| choice | today | adds |
|---|---|---|
| terrain | `fbm` (`world::heightfield`: fBm + warp + island dome), `dem` (an elevation .npy) | an eroded, drainage-aware relief (G4) |
| streets | `districts` (`generate_roads`: tensor-field tracing, districts, kinds, graded; G2), `none` | A* routing, district modes |
| buildings | block-deep lots, core-first fill, procedural or archetype (G3) | archetype by area, sheds |

- **`cvc::world::terrain_source`** (`inc/cvc/world/terrain_source.h`): `sample`, `slope_deg`,
  `bounds()`, its datum (`sea_level_m`, `relief_m`: what the raster's height and slope classes
  scale by, which `raster.cpp` used to read from the heightfield's params), and optional masks
  (`buildable`, `channel`, `cliff`). `world::heightfield` implements it. **`raster_heightfield`**
  is a gridded relief (bilinear, clamped to its edge, optional mask planes) that carries any
  generator's DEM; `sample_terrain` rasterizes any source onto a grid, and `load_dem` reads an
  elevation .npy (`read_npy`: f4, f8 and the integer dtypes). `world_model` (`terrain()`),
  `raster()`, the scatter and the city take the interface; `world_params::terrain` hands one in.
- **Generators by name**: `generated_world_params::terrain` / `city` / `terrain_file`;
  `register_terrain_generator` and `register_city_generator` let a host add its own (a city
  generator returns the streets and an urbanity field; the lots and buildings along them are
  shared). An unknown name throws `std::invalid_argument` before any work. The `generated_world`
  node reads `terrain:`, `city:` and `terrain_file:`, seeds `world.terrain`, `world.city` and
  `world.terrain_file`, and publishes the registered names (`world.generators.terrain` / `.city`,
  one per line) for `world_controls.ari`'s Terrain and City combos. A generator that fails (a
  missing DEM) keeps the current world and puts the error in `world.status`.
- **The terrain as a geometry source**: `source: { terrain: { generator, size, resolution, seed,
  amp, file, preset } }` in any document (or `heightfield: { generator: ... }`, whose layers add on
  top) draws a generator's ground as a `cvc::height_field`, so `ground:`, bodies and band colours
  work on it as on any heightfield.
- **Water stays libcvc's**: everything under `water_level_m` is water, whatever made the relief.
  Sea level is 0 m: every generator lays its ground with 0 m as the shoreline, its floor
  `world_sea_share` (34/90) of `amp_m` below it (a DEM is stretched to `amp_m` and anchored there
  too; the island stands its dome in the sea), so the water level means the same thing on any
  relief, and a preset's lower relief no longer drowns the world under a water level set for a
  higher one (the urban preset's 10 m under the old 34 m: a world of sea). The raster classes the
  ground under it as water and the 0.6 m above it as sand, and the ground texture shades it from a
  light shore green-blue to deep blue over 8 m, since the water volume all but vanishes over the
  shallows. The roads' no-go mask is the water plus the terrain's channel and cliff masks; lots also
  stay on its buildable mask; district centres keep off it.
- **Fixed on the way** (bugs, not kept): the fBm seed was XORed with the world seed, which was the
  same number, so every seed stood on the same ground (the default document is now seed 5, whose
  two towns fill the 110 buildings asked; seed 7 builds 76 on its new ground). The city's
  buildings were never stamped into the material / occupancy raster (a nav consumer saw open
  ground under them); they are now concrete-wall cells, occupied and hard, at least one per
  building. The terrain mesh's cells read the raster's material half a cell off.
  `road_network::distance_to_road` scanned every segment when no road was within about 30 to 60 m,
  which on a large world (blocks half a kilometre across) is most queries:
  `distance_within(x, y, r)` bounds a query, `on_or_near_road` and the lots' block-depth probe use
  it, and `distance_to_road` widens its window instead of scanning. Results are unchanged (the same
  nearest, the same tie-break).
- **Large worlds, measured** (native, Release, this box; `CVC_TL_PROF=1 terrain_lab --half H`):
  the world half slider already goes to 10 km. Generating a 1.2 km world takes about 0.5 s; a 5 km
  world (half 2500) took 10 s, 8 of them in the city's proximity queries, and now takes about
  1.4 s (raster 0.5 s and terrain mesh 0.5 s for its 1001² samples, streets 0.13 s, city 0.13 s).
  The browser build is single-threaded, so expect a few times that there.
- Still to do: a converter from any generator's polylines (with shared junction vertices) to a
  `road_network`, for an imported street map; a service road kind.

## G2 — the street model

**First pass landed (revision 2).** `generate_roads` keeps its tensor-field tracing and gains, all
opt-in through `road_params` (the defaults are the classic network):

- **District centres** (`centres`): each seeds an arterial cross. `generate_world` places about one
  per 600 m of world side, on dry, gentle ground, scored by slope and kept apart.
- **A hierarchy** (`collector_width`, `road_segment::kind`): an arterial's branches are collectors
  a block apart, and the collectors and locals sprout the local streets.
- **An urbanity field** (`generate_roads`' `density`): local streets pack tighter and branch more
  often in a core (`sparse_spacing`, `sparse_branch`), so the outskirts thin out.
- **A grade limit** (`max_grade`): a segment that is too steep turns toward the contour in 15°
  steps either way, up to 75° (80° in the lab), and is dropped if no turn fits. Hill roads wind
  instead of climbing the fall line. The lab allows 35 % (San Francisco's steepest streets are
  about 31 %): its relief is hilly, with a median slope of about 33 % and only a tenth of the land
  under 13 %. Measured on the lab's default world of the time (seed 7, on the seed-blind ground
  G1 fixed): at 25 % and 6 m of lot relief its two districts grew 2.3 km of road and 31
  buildings; at 35 % and 10 m, 5.8 km and the 110 asked for.
- **Fading local grids** (`local_min_urbanity`): local streets stop where the urbanity drops off,
  while the arterials and collectors run on between the districts.
- **A continuation fix**: a road continues along whichever field direction it is already on.
  Before, a road on the minor direction (each centre's cross arm, every branch) turned onto the
  major direction after one step, which striped the classic network with parallel streets about a
  step apart.
- **A no-go mask** (`generate_roads`' `blocked`): the water. Streets are steered away from the
  shore or stop at it, instead of being cut off after tracing.
- **`prune_roads`**: dead-end stubs shorter than 30 m are trimmed, and node degrees recomputed.

**Roads shape the ground (landed).** A road laid on the ground as it was rose and fell with every
bump and tilted with every side slope: on the 1.2 km lab world the 95th-percentile tilt across a
road was 0.76 (37 degrees), the grade 0.61, and 51 of 4,568 road samples were under water, so a
Humvee bounced and rolled. `cvc::world::road_grade` gives each road node a height: the ground,
kept 0.8 m above the water, relaxed along the network so the grade changes gradually, held to a
12% climb (5% on a segment meeting a junction, so a crossing is level), with roads that pass close
without meeting held to agree. `graded_terrain` presses that surface into the ground: level across
the road and a mesh cell past each edge, then a cut or bank back to the ground; where several roads
reach a point the ground blends toward all of them, so it never steps. `world_builder`'s new
`grade` stage swaps it in for everything after the streets: the buildings, the road mesh, the trees
and rocks (re-read their ground), the water, the raster (a bank lifted out of the water is gravel),
and the terrain mesh physics and nav drive on. Measured on that mesh (`RoadsAreDriveable`, and the
1.2 / 5.3 km worlds): tilt across p95 0.043 / 0.010, grade p95 0.134 / 0.065, grade change per
2 m p95 0.057 / 0.023, nothing under water; cuts and fills stay within about 7 / 12 m. What is left
is the terrain mesh's own resolution: a 5 m street on 5 m cells.

Every street is two lanes of `world_lane_m` (the Humvee's 2.18 m plus 0.4 m either side): local
streets 6.0 m (were 5.0), collectors 7.0 m (6.5), arterials 9.0 m. The road mesh draws each run of
segments between junctions as one strip, its bends rounded and mitred (no gaps or overlaps at a
bend), ending on the rim of the junction's pad so no corner pokes out, one asphalt colour, and a
dark yellow centre stripe that stops short of the junctions.

Still to do: routing a long street over the relief by a grade-limited A* (where steering only
winds), service alleys in the cores, and district modes (industrial, rural) with their own spacing
and widths.

## G3 — building placement

**First pass landed (revision 2).** `layout_lots` gains, again opt-in through `site_params`:

- **Block depth**: a lot reaches back to the middle of its block (a ray cast to the next road
  behind), so the lots of two parallel streets back onto each other.
- **A ground check** (with a height sampler): no lot with more than `max_relief` of fall under it
  (10 m in the lab: a 30 m lot on a 33 % slope falls about that much), or below `min_ground` (the
  water). A kept lot records its ground range, and its building stands on the low side with its
  walls down into the slope.
- **Districts** (with the urbanity field): narrower setbacks and side gaps toward a core, down to
  shared walls; whole street sides built or left empty on sparse ground (`sparse_fill`); jittered
  frontages.

`generate_world` builds the city roads-first with or without a building library. Lots fill from
the cores out, a street side at a time, up to `buildings`. Without the Austin archetypes, each lot
gets procedural massing: a footprint filling the frontage, and a lognormal height that rises with
urbanity and a little with footprint area. The cores add a rare podium and tower; outside them,
low houses get gable roofs. The cores' lots get paved forecourts in the ground texture. Trees and
rocks keep off the roads and buildings.

Still to do: a second fill pass for the holes, back-yard sheds, and choosing the Austin archetypes
by footprint area and the lot's urbanity rather than only by fit. The town also depends on how
steep the world is: the relief keeps its amplitude as the world shrinks, so at half 400 m (90 m of
relief) only 7 buildings fit where half 600 m fits the 110 asked for. Flatter buildable ground
where the districts sit belongs to G4.

## G4 — terrain synthesis

Band-limited spectral fields (an FFT: a small pocketfft or kissfft recipe, or octave noise if exact
spectra are not needed), archetype macrostructure (uplift, folds, scarps, ravine networks), a few
erosion passes (priority-flood depression filling, D8 routing, contributing area, stream-power
incision), thermal relaxation, and cliff / channel / buildable masks. Every random draw goes
through `cvc::lsys` hashed streams (the configure-time determinism guard on `world/`). The output
is a `raster_heightfield` with masks.

**Our own code, no port.** The DBG project's scene generator showed what reads well: district
cores, a street hierarchy, grade-limited streets, lots backing onto each other, and core-first
fill. G2 and G3 take those ideas into libcvc's own generators, written here. No code and no
fitted constants are carried over, and G4 is written the same way.

## G5 — the conference's world

The conference is the world's second consumer. Peers must see the same world and walk on it.

- **A replicated world descriptor** in the room scene (`conf.rooms.<room>.scene.world`): the
  template URI or the generator, its params and seed, and a revision. `arisrv --scene world` seeds
  one. Clients generate locally; `generate_world` is deterministic, so every peer gets the same
  world. Only the descriptor crosses the wire.
- **`room_view` realizes it** through the `generated_world` node type. The client document keeps
  its own look (lights, shadows, background).
- **Avatar physics on the world**: a height-field `avatar_static` and the buildings' boxes, built
  from `generated_world::geometry()` on the relay too (the holon is server-authoritative).
  `pick_ground` samples the terrain, not a flat plane.
- **Shadows on the local avatar**: `shadows: { focus: camera }` already follows the view. A
  `follow: avatar` (the local avatar's position) is a small addition to the same key.

## G6 — hardening the template

- **Measure, then budget.** Frame time and shadow-bake cost of `terrain_lab.ari` against
  `terrain_lab` and demo3, natively and in the browser (wasm-mt). demo3's numbers to beat: about
  4 ms a native offscreen frame; a shadow bake costs 30–37 ms in Firefox, hence demo3's bake
  interval of 24 live, 2–4 with moving casters.
- **The Austin archetypes in the browser** for arigl. The C++ terrain_lab's wasm build preloads
  the bundle at `/bundle`; arigl's needs the same, or `building_library:` as a fetched URI.
- **Large worlds**: terrain tiles and building LOD (`VISIBILITY-AND-LOD-ROADMAP.md`) for worlds
  past a few km, where one 2049² ground mesh stops being cheap.
  - Landed: a rebuild no longer blocks the page. Generating in one call froze the browser on a
    10.5 km world (a 14.8 s stall on the deployed gallery, 2026-11-02), so `world_builder` and
    `surface_painter` step the world and its texture as a sliced task (`world.progress`,
    `world.phase`, Stop); the water's ground is sampled once with the world instead of every few
    frames, and arigl logs at info level, not debug. Still one call each, so still a hitch on
    single-threaded wasm: the scatter, the streets, the city, the roads, the props, and handing
    the new 2049² mesh to VTK.
  - Landed: tree and rock LOD. On the deployed terrain lab the props were one merged mesh,
    771,994 triangles of branch tubes for 403 trees (~1,900 each, a maturity-4 pine 7,600) plus
    149,481 needle lines, and the GPU's largest single cost (skipping it: 29 -> 51 fps, Intel Arc,
    2026-11-03). The props are now laid down a 128 m ground cell at a time (`prop_cells`), and the
    node draws each cell as a `LodGraphicsNode` ladder chosen every frame by `cvc::lod` screen
    error, on the section 8.4 vegetation bands (rung 1 from 260 m, rung 2 from 700 m, scaled by the
    quality preset). Every rung is the same full-maturity tree: rung 1 tessellates it coarser
    (`cvc::lsys::tree_detail`: three-sided tubes, none under 5 cm, each pine leaf a solid cone
    of its needles' shape, a broadleaf's leaf twigs one blob per fork; 36% of the triangles, no
    lines), rung 2 is its trunk and a crown over its full canopy (1.1%). The crowns take the full
    tree's area-weighted colour and cover what its limbs do side-on, so the look holds as cells
    switch. Still to do: cards past 1.6 km.
  - Landed: terrain LOD (section 8.4's chunks with skirts). `cvc::world::terrain_lod` cuts the
    terrain into chunks of 64 cells (128 on a world over 16 chunks across), each a ladder of
    strides 1, 2, 4, ... down to 4 cells a side, every rung the terrain's own samples (rung 0 is
    its mesh exactly) with its largest height error measured. The node chooses each chunk's rung
    by screen error and holds neighbours within one rung, which is what the skirts are deep
    enough for. The rungs follow the error, not fixed radii: flat ground coarsens at once, a
    channel's banks stay fine. The 2049^2 ground of an 11.2 km world, 8.4 M triangles drawn
    whole, is 0.26 M from its middle and 16 k from the overview (natively, balanced preset); a
    1.2 km world's 5 m relief stays at rung 0 in the balanced preset. The rungs hold 1.42x the
    terrain's vertices as well as the terrain.
  - Landed: the cloud deck and its shade (section 8.4's cloud ladder, a first part). `cloud_sky`
    was one fixed 60x60x28 slab (1.28 km across in the terrain lab) whose ground shadow was a
    96^2 bake of shade x band albedo that replaced the ground's texture, so the terrain lab turned
    it off, and an 11.2 km world had clouds over its middle 1.28 km only. Now:
    - `cvc::world::cloud_layer` (GL-free, `world_cloud_test`) scatters a few L-system cumulus
      maps over a periodic grid of cells: which cells hold a cloud (`coverage`), its offset in the
      cell, and a per-cell sequence of maps it morphs through. The period is wider than the
      drawn area, so each cloud has one copy drawn and every texture wraps; the wind is a
      translation of the whole pattern. Each map is scaled to its own peak (the grammar grows some
      variants nearly empty: the old slab spent half its morph on a faint one). Morphs are linear
      crossfades, so the shadow's optical depth is the crossfade of the maps' and follows the
      morph exactly. The shadow and the far slices integrate the volume's transfer-function
      opacity, not the raw density, so they show the cloud the volume shows.
    - The shade is drawn in the receivers' shaders: the ground (every LOD rung), the buildings,
      the roads, the props. A periodic mask is read at the fragment's base-plane point along the
      sun and scales the direct light only; the ground's texture is untouched. `GeometryNode` gained
      shared shader inputs (one set that many meshes read at draw time; the deck updates it each
      frame) and fragment replacements applied after VTK's own: a receiver rewrites the lighting
      sum VTK generates, the one place both mappers keep the direct terms apart (the WebGL2 mapper
      scopes them to its lighting block). Checked on that mapper natively
      (`CVCGL_LOWMEM_MAPPER=force`): it applies the replacement and compiles the deck's shaders.
    - Coverage: a pool of near `VolumeNode`s (6 in the terrain lab, at most 8) on the clouds nearest
      the camera, and one mesh of 10 translucent shells for the rest, sampling a coarse opacity
      texture as the optical depth along the view ray. A cloud passing between the two
      crossfades: the shells cut out the near volumes' squares by uniform, so a fade never
      re-composites. The deck follows the ground's extent live.
    - The terrain lab's fill, environment and ambient went from 0.3 / 0.5 / 0.32 to 0.15 / 0.2 /
      0.2: with more light from the sky than the sun, building and cloud shadows washed out.
    - Cost, measured natively (this box, 1280x720 offscreen): the sky's tick averages 0.4 ms a
      frame on both the 1.2 km and the 11.2 km world. The shade (512^2 at most) and far (256^2 x 10
      at most) composites are spread over 8 frames once a second; their uploads take about 0.4 ms
      and 1 ms, into double-buffered textures; walking the receivers for new meshes about 1.5 ms
      once a second. The old slab cost about the same: a 60x60x28 crossfade and a 96^2 bake every
      third frame. In the browser, expect 2-3x the CPU on single-threaded wasm (about 1 ms a frame,
      4 ms once a second) and, on the GPU, up to 6 small volume ray casts plus 10 shell layers'
      fill. Not yet measured in a browser.
  - Landed: the cloud deck's look, the five issues the first part left.
    - Clouds reach the horizon. The far shells are out of the bounds the camera fits and clips to
      (`GeometryNode::setUseBounds(false)`: VTK's UseBounds and the scene's own box), and reach
      `far_reach` (4 km by default) past the ground, where the pattern repeats. A shell vertex
      past the far plane is drawn on it: its clip-space depth is clamped to 0.999 w, which leaves
      where it lands on screen and the world point its fragments read unchanged. The near volumes
      still draw only within `horizon` of the ground and stay in the bounds, so the framing holds.
    - No banding seen edge-on. Each shell stands for the slab of deck around it: its fragment
      marches the view ray across that slab (6 samples, at most 24 slabs' depth) and sums the
      optical depth, so the slices join into one cloud from below, beside or above, and the last
      3 degrees fade into haze. (Tried and dropped: a column-mean slice every shell reads, which
      stacked identical footprints into striped pillars at middling angles.)
    - Near and far clouds look alike. VTK's volume shading reads as storm cloud here (dark,
      metallic), so the volumes stay unshaded and both take one colour ramp from the density:
      grey-blue thin, white dense, the shells a little lighter with height.
    - No straight-edged clouds. The grammar clamps its turtle at the grid's y edges (it wraps x), so
      a cloud that wanders there piled puffs against the face and the grid's window cut them off:
      a straight edge along the volume's box. Each map now fades out over the outer 40% of its
      radius (`world_cloud_test` checks nothing reaches the rim), which also keeps the clouds round.
    - `terrain_bump` and `bark` name the normal by the node's mapper
      (`GeometryNode::usesLowMemoryMapper`), not by `__EMSCRIPTEN__`, so `CVCGL_LOWMEM_MAPPER=force`
      natively draws the whole terrain lab (ground, shade, shells, volumes) without a shader error:
      the browser's shader path, as GLSL 150, can be checked here.
  - Landed: new meshes take the shade as they join. `SceneGraph::treeVersion()` moves whenever a
    node joins or leaves the tree (registered or removed, or a child added or removed anywhere in
    the scene) and holds still for moves, visibility and geometry swaps (`cvcgl_tree_version`). The
    deck walks its receivers on the frame after it moves, instead of once a second: an LOD rung or
    a regenerated world's part is shaded on its first frame, and a settled scene never walks
    (`cvcgl_ariadne_realize` adds a mesh under the ground and checks the next tick shades it).
  - The weather evolves, and it gathers on windward flanks (LSYSTEM-LABORATORY-ROADMAP section
    4.7):
    - A Dobashi-style cellular automaton in `cvc::world::cloud_layer` decides which cells hold a
      cloud. Each cell has one bit-packed column of 32 layers, with Dobashi's humidity, activation
      and cloud bits.
    - Activation runs up a whole column and, by chance, into the four neighbours. Clouds go extinct
      at random and humidity returns, so clouds grow in patches and dissipate.
    - The cells with the most cloud hold the deck's clouds, still exactly `coverage` of them.
    - It steps on the world clock, once every 5 s at rate 1, so a cloud lasts some 8 minutes. A
      step over 64 x 64 cells takes 0.9 ms, and the 11.2 km world's 21 x 21 takes a fraction of
      that.
    - A cloud fades in or out over 20 s. The volume, the far slices and the shade take the same
      weight, so nothing shades where no cloud is drawn.
    - Orographic coupling: the slope along the wind under each cell, averaged over a cloud's
      footprint, raises its humidity and activation on a windward flank and its extinction in the
      lee. The pattern drifts over the ground, so a cloud forms as it reaches a mountain and dies
      past the crest.
    - Over ridges, windward cells hold a cloud 99% of the time and lee cells 14%; without the
      coupling both are 60%.
    - Live keys `sky.evolve` and `sky.orographic` are on `sky_controls.ari`. The props are
      `evolve`, `orographic`, `wind_az` (the drift now follows it) and `fade`.
    - Every rule is deterministic and frame-rate free (`world_cloud_test`).
  - Measured in the browser, 2026-11-06: the gallery deploy of 38c6b071 (deploy-pages
    42290995729), in the desktop app's built-in Chromium 152 browser (WebGL2), canvas 1077 x 772 at
    device pixel ratio 1.25.
    - The deck's shaders compile as GLSL ES: no shader errors in the console.
    - Clouds reach past the world's edge.
    - 1.2 km world, default camera: 34-47 fps, median frame 17-25 ms, p95 42-58 ms.
    - 11.2 km world (World half 5600, regenerated, 2049^2 terrain), close-up camera: 25-29 fps,
      median frame 25 ms, p95 83-133 ms. On 2026-11-03, before the terrain LOD, the same world
      ran at 3.3 fps.
    - JS heap about 64 MB.
  - Still to do for the clouds:
    - The deck's own cost in the browser. The cloud nodes can't be hidden from the URL, and the
      document's Sky panel isn't reachable from the gallery's menus.
    - Firefox (Joe's to check), and an 11.2 km overview camera.
    - The deck is one flat slab, so a peak pierces it only where the ground rises past `base`. A
      lower deck over high relief (or cloud bases that follow the ground) would show orographic
      caps at the terrain lab's 90 m of relief.
- **Dynamic props, demo3's way**, for vehicles and avatars on the world: unregistered slot nodes,
  a pose gate, out of the bounds fit, non-casting. demo3 does this by hand. A libcvc node type
  would make it a scene key.
