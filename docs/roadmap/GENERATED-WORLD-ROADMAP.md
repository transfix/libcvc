# Generated worlds: one template, switchable generators, the conference's world

**Status:** plan, revision 2 (2026-11-02). G0 landed with this document; first passes of G2 (the
street model) and G3 (building placement) landed in revision 2; G1 and the rest are the plan.

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
  `cloud_sky` takes `extent` / `base` / `top`, so its slab fits a world of any size.

## G1 — a generator interface and the switch

The terrain lab's world is three choices, each switchable in the panel:

| choice | today | adds |
|---|---|---|
| terrain | `world::heightfield` (fBm + warp + island dome) | an eroded, drainage-aware relief (G4) |
| streets | `generate_roads`: tensor-field tracing, districts, kinds, graded (G2) | A* routing, district modes |
| buildings | block-deep lots, core-first fill, procedural or archetype (G3) | archetype by area, sheds |

- **A terrain source interface**: `sample(x, y)`, `slope_deg`, `bounds()`, and masks
  (buildable, channel, cliff). `world::heightfield` implements it. A new `raster_heightfield` (a
  gridded relief, bilinear) carries any generator's DEM. `world_model`, `raster()` and the scatter
  take the interface, not the concrete heightfield. They read `params().sea_level_m` / `amp`
  today (`raster.cpp`), and that moves into `world_params`.
- **Generator choice in `generated_world_params`**: `terrain_generator` and `city_generator`
  (enums, or registry names so a host can add one). They are new `world.*` keys, with combos in
  `world_controls.ari`.
- **Water stays libcvc's**: everything under `water_level_m` is water, whatever made the relief.
  That level's mask joins the excluded mask a generator's lots and roads avoid. A generator's own
  water (river channels) is dropped, or kept as land-cover only. A generator whose relief is
  relative (not referenced to 0) is anchored, so the water level means the same thing.
- **Roads carry a kind**: `road_segment::kind` (local / collector / arterial, each with its width)
  landed with G2; a service kind is still to come. A converter turns any generator's polylines with
  shared junction vertices into a `road_network`, so `extrude_roads`, `rasterize_roads` and
  `layout_lots` keep working.

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
  under 13 %. Measured on the lab's default world (seed 7): at 25 % and 6 m of lot relief its two
  districts grew 2.3 km of road and 31 buildings; at 35 % and 10 m, 5.8 km and the 110 asked for.
- **Fading local grids** (`local_min_urbanity`): local streets stop where the urbanity drops off,
  while the arterials and collectors run on between the districts.
- **A continuation fix**: a road continues along whichever field direction it is already on.
  Before, a road on the minor direction (each centre's cross arm, every branch) turned onto the
  major direction after one step, which striped the classic network with parallel streets about a
  step apart.
- **A no-go mask** (`generate_roads`' `blocked`): the water. Streets are steered away from the
  shore or stop at it, instead of being cut off after tracing.
- **`prune_roads`**: dead-end stubs shorter than 30 m are trimmed, and node degrees recomputed.

Still to do: routing a long street over the relief by a grade-limited A* (where steering only
winds), smoothing the traced polylines, service alleys in the cores, and district modes
(industrial, rural) with their own spacing and widths.

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
- **Dynamic props, demo3's way**, for vehicles and avatars on the world: unregistered slot nodes,
  a pose gate, out of the bounds fit, non-casting. demo3 does this by hand. A libcvc node type
  would make it a scene key.
