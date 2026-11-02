# Generated worlds: one template, switchable generators, the conference's world

**Status:** plan, revision 1 (2026-11-01). G0 landed with this document; G1 onward is the plan.

**Goal.** One generated world that every app builds on: the terrain lab, the Ariadne conference's
rooms (arisrv + arigl, the cyprty browser client), and the next demo. It is a document template
(`examples/ariadne/documents/terrain_lab.ari`) on libcvc's fastest render path, with a world that
switches between generators at run time, and libcvc's water whatever the generator.

## G0 — what landed

- **`cvc::world::generate_world`** (`inc/cvc/world/generated_world.h`): terrain_lab's world, GL-free
  and deterministic. The relief and L-system prop scatter (`world_model::generate`), the props as
  merged meshes, the roads-first city (`generate_roads`, `layout_lots`, the Austin archetypes) or
  box massing without a building library, the surface texture (`paint_surface`), the water below
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
| streets | `generate_roads` (tensor-field tracing) | a district / hierarchy street model (G2) |
| buildings | fixed-frontage lots + archetype by hash | frontage lots + calibrated heights (G3) |

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
- **Roads carry a kind**: `road_segment` gains arterial / collector / local / service, each with
  its width. A converter turns any generator's polylines with shared junction vertices into a
  `road_network`, so `extrude_roads`, `rasterize_roads` and `layout_lots` keep working.

## G2 — the street model

A district-structured network. District centres are placed by slope-weighted, separation-repelled
sampling, each with a mode (urban, suburban, industrial, rural, mixed). Arterials cross the
domain, collectors link each centre to them, and each district's own grid is clipped to the area
it owns. Urban cores get service alleys. A street runs straight where the grade allows and is
otherwise routed by a grade-limited A* over the relief, then smoothed. Crossings become shared
junction vertices (4-way and T). The output is a `road_network` with kinds (G1).

## G3 — building placement

Lots are cut along each side of each street, frontage-first, filled core-first. Each lot's depth
is capped at half the clear distance to the street behind. Setbacks and spacing shrink toward the
cores, giving party walls downtown. Lots end at corners, a half-shifted second pass fills the
holes, and back-yard sheds are added. A lot is rejected for slope, pad relief, the excluded mask
(water included), the road corridor and overlap. Footprint area, aspect and height come from
calibrated distributions: height grows with area, plus a heavy tail of towers. libcvc keeps
stamping the Austin archetypes, chosen by footprint area and aspect and scaled to the sampled
height, with an extruded box as the fallback.

## G4 — terrain synthesis

Band-limited spectral fields (an FFT: a small pocketfft or kissfft recipe, or octave noise if exact
spectra are not needed), archetype macrostructure (uplift, folds, scarps, ravine networks), a few
erosion passes (priority-flood depression filling, D8 routing, contributing area, stream-power
incision), thermal relaxation, and cliff / channel / buildable masks. Every random draw goes
through `cvc::lsys` hashed streams (the configure-time determinism guard on `world/`). The output
is a `raster_heightfield` with masks.

**Owner gate for G2–G4.** The street, lot and terrain models above are those of the DBG project's
scene generator: private, grant-funded, a single author, no licence file. libcvc is public (LGPL
2.1). Before code is ported, or its algorithms and calibrated constants are transcribed, its author
and the project's PI must agree in writing. The DBG engagement documents expect the generic terrain
generator to live in open libcvc core, but that is not a licence. Without the OK, G2–G4 are written
from the published literature, with no transcription.

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
