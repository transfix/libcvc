# Fleets and night: the swarm as world vehicles, and lights after dark

Status: plan (2026-11-18). Nothing here is built yet. Phase ids: **FL** (fleets) and **NL** (night
lighting). Neither is used by another roadmap.

## 0. The ask

Joe, 2026-11-18, on transfix/libcvc#1358 (nav_city_swarm and nav_city_drive as Ariadne documents):

- Rework it so a swarm is the world's vehicles, not a runtime of its own.
- Add city lights, vehicle headlights and proper nighttime.
- Converge with the scenes the world runtime runs, locally and on a server (ARIADNE_WORLDS.md: L0
  to L4, and WD3's vehicles on the relay).

## 1. Where things stand

**#1358's swarm is a second vehicle runtime.**
- `type: nav_swarm` runs `cvc::ariadne::SceneSwarm` ([scene_swarm.h](../../inc/cvc/ariadne/scene_swarm.h)).
  It has its own `sim_world`, its own `plant_host` physics world and its own `sim_thread`.
- `type: swarm_layer` nodes draw it as about ten merged meshes for 800 vehicles:
  - LOD rungs, welded and decimated, chosen by on-screen size;
  - flags, goals, trails, belief walls, minimap markers;
  - a follow node.
- It is fast. On the reference laptop the documents beat the C++ programs:
  - swarm: 35 to 45 fps, against 9 to 17;
  - drive: 37 to 55 fps, against 19 to 29.
- But it lives outside the world:
  - its vehicles are not in the scene's physics world;
  - they are not entities, so they take no intents;
  - the relay does not send them, and the gym cannot step them;
  - a light cannot be attached to one of them.

**World vehicles are the world's entities**
([world_vehicles.h](../../inc/cvc/ariadne/world_vehicles.h), ARIADNE_WORLDS.md §14).
- Each entity has an identity (name, team, two colours), a controller (`nav@1`, `none`,
  `external`) and intents. Its plant (`jolt`, `dynamic`, `dynamic4w`, `kinematic`,
  `dynamicboat`) runs in the scene's one physics world.
- They run in every runtime:
  - drawn in arigl;
  - headless in ari, arix and `AriApp`;
  - in the gym (`WorldGym`);
  - on the relay (WD3): a server document's world runs headless, and its vehicles reach clients as
    `conf.rooms.<room>.entities` plus pose-stream entries (`~e<id>`, at most 30 Hz each), played
    out by `entity_playout` and drawn as box looks by `room_view`.
- Since #1339 and #1340 they steer around one another and can sense under fog (`vehicles.sense`).

**What world vehicles lack for a fleet of hundreds:**

| Gap | Today |
|---|---|
| Drawing | One scene node per entity (`sync_vehicle_markers`), all rebuilt when the revision moves (each spawn). The C++ demos measured per-vehicle actors falling over at about 63. |
| State | `publish_entities` writes about 25 keys per entity per publish: about 20,000 writes for 800. |
| Belief | One navigator per plant, one belief: shared only. The demos' grouped and private planes are missing. |
| Spawning | One at a time through `vehicles.spawn.*`. No layout (one edge to the other), no count. |
| Goals | `nav@1` wanders, or `set_goal` per entity. No rally points, and no colour by team. |
| Policy | `nav@1`'s default is the **built-in** biased net (`policy_library.h`); trained weights only by URI and hash, entity by entity. The demos (and #1358) default to the **published trained** CoefMLP (`coef_mlp::default_weights_path`), and fall back to the built-in net silently: on the reference laptop both did, because the build's install prefix had no weights. |
| Navigator metric | A world vehicle's navigator plans with its own vehicle's size and limits. The demos plan with the trainer's vehicle (`city_scene`'s rr, d_hat, vmax at 0.05 units a metre), the metric the trained CoefMLP learned in. |
| Threading | The navigator ticks on the frame's thread inside `SceneWorld::follow`. The swarm's ran on a `sim_thread` worker. |
| Relay | Every entity goes to every viewer at 30 Hz, each with its own identity entry. The client draws the box, never the model. |

**Lights.** Free `kind: spot` lights exist, and each casts a shadow map (`LightNode`,
`StridedShadowBaker`). Limits today:
- One baker for every shadow light, with one resolution and one re-bake interval.
- `shadows: { focus }` re-aims every light's map at the camera's sphere. That is right for a sun,
  wrong for a lamp.
- No per-light shadow switch: only `kind: fill` (a 90° cone) casts none.
- No attenuation, so a lamp lights the whole city.
- No way in the DSL to attach a light to a node, although `LightNode` already follows its parent.
- Every shadow light is a sampler in every lit shader: 16 units in WebGL2 (`make_stage_rig`
  already dropped two spots for this).
- On the relay, WD1 publishes the document's lights and a rig's knobs (`scene.env.lights`).

## 2. Design

### 2.1 Fleets: many entities, one declaration (FL1)

A fleet is a named `world: vehicles: start:` entry (settled: the start list grows, no new
block). An entry with `fleet:` is laid out, coloured, grouped into teams and published as one:

```yaml
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [nav@1, none, external],
                lights: { headlights: { cone: 30, range: 60, intensity: 1.2 } } }   # NL4
    start:
      - fleet: city                 # a state name: the fleet's keys are vehicles.fleets.city.*
        archetype: humvee
        count: 800                  # up to 4096 for a fleet (256 for an unnamed entry)
        plant: kinematic            # jolt | dynamic[:preset] | dynamic4w[:preset] | kinematic;
                                    # a list cycles over the vehicles; per team: { 0: jolt, ... }
        controller: nav@1
        policy: trained             # trained | builtin | { uri, path, sha256 } | { data: <entry> }
        metric: policy              # trained (the trainer's vehicle) | vehicle (each plant's own)
                                    # | policy (follows the policy, the default)
        sense: fog                  # known | fog
        belief: grouped             # shared | grouped (max(2, n/64)) | private
        spawn: opposed              # opposed | scattered | points (world: spawns:)
        seed: 7
        rally: { count: 16, assign: latitude }   # teams; goals held at the team's rally point
        colors: hsv                 # each team's colour, the entity's identity colour
        keys: summary               # summary (a fleet's default) | all: per-entity keys
```

- **An entry without `fleet:` is what it was:** `count:` vehicles (at most 256) spawned one by
  one, named `name 1`, `name 2`, ..., each with its own free point and wandering goal. A fleet's
  vehicles are named `<fleet> 1`, `<fleet> 2`, ..., unless `name:` says otherwise.
- **Laid out together.** The fleet is placed as a whole once the world's grid exists, from its
  `seed:`: `opposed` starts in the west 30 % and goals in the east 30 % (as the demos'
  `scatter_free`), `scattered` both anywhere free, `points` the world's spawn points. Teams come
  from the goals' latitude (`assign: latitude`, SceneSwarm's `assign_targets`), each team's rally
  point the goal nearest its band's centre; every vehicle's goal is held at its team's point (no
  wander).
- **Each vehicle is still an entity.** It has an id, an identity (team = its rally point, its
  colour), intents, a body in the physics world, and the gym's observation. Everything that works
  on entities works on a fleet's.
- **The plant.** Any plant world vehicles run, in the scene's one physics world, so a Jolt fleet
  collides with the city's buildings and terrain (the data node's `body: {}`), unlike #1358, whose
  `plant_host` drove Jolt vehicles on a flat ground of its own.
  - `jolt` is the drive's default (60 Humvees at their real size and limits). `kinematic` is the
    swarm's: hundreds of physics vehicles do not keep up with the clock, as FL0 will measure.
  - A list cycles over the vehicles, and a map gives each team its plant, so one fleet can mix
    Humvees on Jolt with bicycles.
  - `vehicles.fleets.<name>.plant` switches it live, as `nav_plant.vehicle` does today: the
    vehicles keep their poses and start at rest on the new plant.
- **The policy: trained or not, and which weights.**
  - `trained` (the default) is the published trained CoefMLP, downloaded by default: the
    `grl-snam-weights` package (`cvcpkg://cvc/grl-snam-weights`, its
    `share/grl-snam-weights/coef_sdf.cvcnav`) through the data store, checked against the
    registry's sha256 and cached by hash, so later runs read the cache. It is looked for in this
    order (settled, D8):
    1. `$CVC_NAV_WEIGHTS`, when set: a local override;
    2. a copy installed in the prefix (`coef_mlp::default_weights_path`'s `share/cvc/nav`, or the
       package installed beside libcvc);
    3. the download, when the host's network allows `https://cvcpkg.org` (the document lists `net`
       as an optional requirement, as the city's package does);
    4. none found: the built-in net, said so (below).
  - `builtin` is `coef_mlp::default_biased()`, the constant (1, 3, 4) basin.
  - Other weights by URI: `policy: { uri, path, sha256 }`. The URI is another cvcpkg package
    (`cvcpkg://ORG/NAME[@VERSION]`), an https URL, or a file beside the document; `path` names the
    `.cvcnav` inside an archive or package; `sha256` pins it. `policy: { data: <entry> }` names a
    `data:` entry, so the document's own `data:` block can declare the weights. Each is fetched,
    checked and cached like the city.
  - The fleet's choice goes through `policy_library`, so weights named twice load once. A team or a
    single entity may name another, through the entity spec's `policy` / `policy_sha256`, which
    already exist. That is the nav lab's arms on one city.
  - `vehicles.fleets.<name>.policy` switches it live through `sim_world::set_policy`, with no
    rebuild.
  - **Without the weights, it falls back visibly** (settled, D8). The built-in net drives, and the
    run says so:
    - `stats.policy` names what drives: `trained (cvcpkg://cvc/grl-snam-weights@1.0.0+cvc.1,
      sha256:...)`, `builtin`, or `builtin: no trained weights (<why>: not installed, the network
      off, the download failed)`;
    - one warning goes to the host's log;
    - the swarm panel's status line shows it.

    While a download is under way the fleet drives on the built-in net, and it switches to the
    trained weights live (`set_policy`) once they arrive.
- **The metric** (settled, D9). `trained` plans with the trainer's vehicle, at the metric the
  weights learned in, as the demos do. `vehicle` plans with each plant's own size and limits
  (`nav_plant_options::plan_with_vehicle`), as world vehicles do today. The default (`policy`)
  follows the policy: `trained` with trained weights (and while they are on their way, so their
  arrival does not make the navigator again), `vehicle` with the built-in net. `metric:` overrides
  it (FL1c).
- **The navigator per fleet.** A fleet's entities share one `sim_world`, built with the fleet's
  belief mode (`sim_world::belief_mode`, which #1358's `SceneSwarm` already drives). Fog coverage
  and sensed walls come from the existing `WorldVehicles::belief`, extended per plane.
- **Rally points are keys.** `vehicles.fleets.<name>.rally.<t>.{x,y,color}`. Writing one calls
  `set_goal` for its team, which is how the minimap's drag already works.
  A change of `vehicles.fleets.<name>.restart.request` or `reseed.request` lays the fleet out
  again (the same seed, or the next).
- **Summary publishing.** `vehicles.fleets.<name>.*` holds count, placed, arrived, teams, seed and
  status (FL1a; tick, planes and bounds to come); `vehicles.fleets.names` lists the fleets.
  Per-entity keys are written only for `keys: all`, the entities `vehicles.watch` names (the
  followed or selected one), and entities with an intent. A view reads
  poses through a lock-free snapshot (`WorldVehicles::poses(fleet)`), never through state.
- **Threading (D3).** A fleet's navigator tick (every 0.06 s, six physics steps) can run on a
  worker while the physics steps, with its commands taking effect a tick later. That keeps the
  render thread free, as the swarm's `sim_thread` did. It is opt-in for drawn hosts only; the gym,
  ari test and the relay stay synchronous and bit-for-bit.
- **What moves out of #1358.**
  - `SceneSwarm`'s pieces (city rasterization, targets by latitude, colours, belief coverage) move
    into fleets.
  - `nav_swarm` and `SceneSwarm` go away.
  - `sim_thread::post` and data nodes' `tint: satellite` stay.
  - The city is the scene's world (`world: data` on the data node). The fleet takes its grid from
    `SceneWorld::grid` at the demos' resolution, with `cell:` derived from `grid:`.

### 2.2 Drawing entities at scale (FL2)

`swarm_layer` becomes **`entity_layer`**, a node that draws one part of a set of entities:

```yaml
- node: city_bodies
  type: entity_layer
  of: { fleet: city }          # or { archetype: humvee } | { team: 3 } | all
  layer: bodies                # bodies flags goals trails path belief dots targets follow headlights
  lod: { near_cap: 10, px: [24, 8, 3] }
```

- **Instanced by archetype.** One merged mesh per archetype per layer, with the LOD rungs from
  #1358 (welded decimation, size floors, culling). An empty rung or an unchanged slot is not
  re-sent.
- **Promotion.** At most K entities (the followed or selected one, then the nearest; `promote: K`)
  are drawn as full per-entity nodes by today's `vehicle_view`. Those get wheels posed by their
  plant, labels, and attached lights (NL4). An entity is demoted as it recedes, with hysteresis.
  Below a threshold (32 entities) every entity is promoted, so `world_lab.ari` looks as it does
  today.
- **The same view on a client of the relay.** The client's entity table (`room_entities` and
  `entity_playout`) feeds the same layers. A server-hosted fleet then draws like a local one, and
  its model looks come from the same cache (WD3's "a model look drawn on the client").

### 2.3 Lights after dark (NL1 to NL5)

**Per light (NL1).** The `lights:` entries gain:

```yaml
lights:
  - light: lamp_12
    kind: spot
    attach: post_12            # or the light is a child of a node in nodes:
    target: [0, 0, -1]         # relative to the attachment when attached
    cone: 55
    softness: 0.3              # the cone's edge (VTK's spot exponent)
    range: 25                  # metres to nothing (VTK attenuation)
    color: { kelvin: 2700 }    # or [r, g, b]
    shadows: auto              # true | false | auto (the budget decides)
    shadow_resolution: 1024
```

- `shadows: { focus }` applies to directional lights only. A spot's map covers its own cone.
- **Per-light baking.** A light re-bakes only when it moved or a caster inside its frustum moved.
  A street lamp's map is baked once; a headlight's every frame its vehicle moves.
- WD1's wire (`scene.env.lights`) carries the new keys, so a server world's lamps reach clients.

**A shadow budget (NL2).** `shadows: { max_lights: N, pick: nearest | brightest }`.
- Each frame the N most important `auto` lights get maps, with hysteresis; the rest light
  unshadowed.
- N is checked against the texture units left after a scene's own textures (16 in WebGL2;
  `CVCGL_TEXTURE_UNITS=16` tests it natively).

**Many lamps: a lamp field (NL3).** Hundreds of street lamps cannot each be a VTK light, because
every light is evaluated for every fragment.
- `lamps:` declares them as data:
  - a list;
  - a dataset layer (a city bundle's street lights, when one has them);
  - or generated along a world's roads, as `spawns.suggest` samples them today.
- Receivers get a shader injection, the way the cloud deck adds its shade:
  - lamp positions, colours and ranges in a texture, binned on a 2-D grid over the city;
  - each fragment loops over its cell's lamps only;
  - no shadows.
- The nearest few lamps are also real spot lights through the budget, so the lamps by the camera
  cast shadows.
- Lamp heads use an `emissive:` material.

**Headlights (NL4).** An archetype's `lights:` (headlights, tail lights) apply to every entity of
it.
- A promoted entity gets real spots attached to its chassis, shadowed within the budget.
- An instanced entity adds its headlights to the lamp field each frame, from the pose snapshot,
  unshadowed. Optionally it paints a light pool on the ground.
- On the relay the archetype carries its lights, so clients add them to the entities they draw.

**Night (NL5).** `scene: sky: { time: 22.5, moon: {...}, exposure: ... }`.
- The rig's sun becomes a sun or moon by the time of day, and ambient, environment and the
  background follow curves.
- The cloud deck, water and weather already read the key light, so they darken with it.
- The world clock can drive a day-night cycle.
- A server world publishes the time of day as a section with an activation tick (WD2's
  discipline), so every client agrees.
- The nav city documents get a Night switch.

**Optional (NL6).** Lit building windows at night (a shader on a city's buildings), light-pool
decals, bloom.

### 2.4 Convergence with local and server-hosted scenes

| | Local (arigl) | Headless (ari, gym) | Relay (arisrv --doc) |
|---|---|---|---|
| Fleet | entities in the scene world | the same, synchronous | the server document's world runs it |
| Identity | keys (summary) | keys / `WorldEntities` | **fleet section**: archetype, count, team colours, id range; per-entity overrides only (FL4) |
| Poses | snapshot to the entity view | `observe()` | **fleet pose packets**: batched, quantized, by area of interest (S7's spatial interest) (FL4) |
| Drawing | entity_layer, promotion | none | the client's entity view, the same layers |
| Lights | NL1 to NL5 | none (no GL) | lights, lamps and time of day as world sections |

At 30 Hz, 800 entities would be 24,000 pose entries a second per client. FL4 batches them per
fleet, quantizes them, and sends only what a viewer's area of interest holds.

## 3. Phases

| Phase | What | Needs | Size |
|---|---|---|---|
| **FL0** | **Done** (2026-11-19; §3.1). `AriadneWorldGym.FleetBench` (`CVC_FLEET_BENCH=1`) times kinematic fleets of 100, 400 and 800 (map known and fog) and Jolt fleets of 60, 120 and 240, headless. Two fixes from it: `WorldVehicles::entities()` found kinematic overlaps pair by pair (quadratic; now a grid, linear), and the navigators get the app's pool. D3 decided by the numbers: yes. | — | S |
| **FL1a** | **Done** (2026-11-19). Fleets as named `start:` entries: `fleet:`, up to 4096 vehicles, `spawn:` (opposed, scattered, points), `seed:`, `rally: { count, assign: latitude }` (teams, hsv colours, goals held), `colors:`, `keys:` (summary, all). Laid out once the grid exists (linear: a hashed separation check); rally keys move a team; `restart.request` and `reseed.request`; `vehicles.watch`; `WorldVehicles::fleets`, `rally`, `move_rally`, `restart_fleet`; `entity_info::fleet` and `rally`. Tests: `AFleetIsANamedStartEntry`, `AFleetIsLaidOutInTeamsAtRallyPoints`, `AFleetWithoutRallyPointsHoldsItsOwnGoals`; the bench's `[fleet doc]` rows (§3.1). Left for FL1: sense, belief and policy per fleet, plant lists, the metric, `poses(fleet)`, the gym's fleet. | FL0 | M |
| **FL1b** | **Done** (2026-11-20). A navigator per fleet (one per plant, `<plant>/<fleet>`): `sense:` (known, fog; the world's by default) and `belief:` (shared; grouped, max(2, n / 64) planes by team; private, one per vehicle), both switched by their keys, which make the navigator again with its vehicles at rest where they are; `.planes`. `policy:` `trained` (`$CVC_NAV_WEIGHTS`, an installed copy, else `cvcpkg://cvc/grl-snam-weights` through the data store), `builtin`, a `.cvcnav`, `{ uri, path, sha256 }` or `{ data: <entry> }`; `.policy` switches it live (`set_policy` / `reset_policy`), `.drives` says what drives, and the visible fallback (the built-in net, its reason in `.drives` and once in the host's log). `policy_library::get_bytes`. A fleet refuses `controller: pose`. Test: `AFleetHasANavigatorAndAPolicyOfItsOwn`. Left for FL1: plant lists and per-team plants, the metric (D9), `poses(fleet)` and the gym's fleet; D3 goes with ARIADNE_WORLDS §18 (below). | FL1a | M |
| **FL1c** | **Done** (2026-11-20). A fleet's `plant:` is one plant, a list its vehicles cycle over, or each team's (`{ 0: jolt }`; a team not named on the archetype's), all floating or none, each plant a navigator of the fleet's own; `.plant` switches them live (`fleet_plants::text`: `jolt, kinematic`, `kinematic, 0: jolt`), its vehicles at rest where they are, and a bad one is said in `vehicles.status`. The metric (D9): `metric:` `trained` (the trainer's vehicle, `training_scene`'s rr, d_hat and vmax at the hosts' tick, and the plants' `plan_with_vehicle` off), `vehicle`, or `policy` (the default: trained while weights drive or are on their way, so their arrival does not make the navigator again; vehicle on the built-in net, a fallback's included); `.metric` switches it, `.plans` says which, and a change makes the navigators again. `plants()` reports each fleet's own bicycle. Tests: `AFleetHasPlantsAndAMetric`, `AFleetIsANamedStartEntry`'s plants and metric. Left for FL1: `poses(fleet)`, the summary's tick and bounds, and the gym's fleet. | FL1b | M |
| **FL1** | **Fleets in `WorldVehicles`.** Named `world: vehicles: start:` entries (`fleet:`; count, archetype, controller, sense, belief per fleet navigator, spawn layout, seed, rally points as teams and colours, goals held), summary publishing, `poses(fleet)`, fleet restart and reseed. Its plant: any world plant, Jolt included, a list or per team, switched live. Its policy: `trained` (downloads `cvcpkg://cvc/grl-snam-weights` unless installed or overridden), `builtin`, or other weights by cvcpkg package, URL, file or `data:` entry, per team or entity, switched live; the visible fallback; the metric following the policy. The gym spawns a fleet. Tests: `ariadne_scene_world_test` fleets (build, step, rally move, restart, belief modes, a Jolt fleet against a building, a mixed fleet, each policy choice, the download and the visible missing-weights fallback); a gym fleet episode. | L2, #1339, #1340 | L |
| **FL2** | **The entity view.** `entity_layer` (bodies with LOD rungs, flags, goals, trails, path, belief, dots, targets, follow) over world vehicles; promotion of K entities to `vehicle_view` nodes; the threshold. #1358's `swarm_layer` code moves here. Test: `cvcgl_ariadne_entities`. | FL1 | M |
| **FL3** | **The nav city documents on fleets.** nav_city_swarm/drive declare `world: data` and a fleet with entity layers. `nav_swarm` and `SceneSwarm` are removed. The minimap drags rally keys; the chase follows an entity. Perf gate: at least #1358's numbers on the reference laptop (and twice the C++ programs, measured back to back). | FL2 | M |
| **FL4** | **Fleets on the relay.** A fleet identity section, batched quantized fleet pose packets, an area of interest for entities, and the client's entity view with model looks. `traffic.ari` becomes a fleet. Test: a relay room with 800 entities, bytes per client. | FL2, WD3 | L |
| **NL1** | **Lights in the DSL.** `shadows:`, `range`, `softness`, `kelvin`, `attach:` or lights as node children, per-light resolution; focus for directional lights only; per-light dirtiness. The world's wire carries them. Tests: a lamp and a headlight offscreen, 16 texture units. | — | M |
| **NL2** | **The shadow budget.** `max_lights`, `pick`, hysteresis, texture-unit accounting. | NL1 | S |
| **NL3** | **The lamp field.** `lamps:` (list, dataset layer, along roads); receivers' shader injection with a 2-D binned lamp texture; emissive lamp heads; the nearest promoted to real shadowed spots (NL2). | NL1, NL2 | L |
| **NL4** | **Headlights.** An archetype's `lights:`: real attached spots on promoted entities, lamp-field entries for instanced ones, optional ground pools; carried by the relay's archetype. | FL2, NL1 to NL3 | M |
| **NL5** | **Night.** `sky:` time of day, sun and moon, ambient and background curves, exposure; the cloud, water and weather read it; a day-night cycle on the world clock; the time of day as a server world section; the nav city documents' Night switch. | NL1 | M |
| NL6 | Lit windows, light-pool decals, bloom. | NL3, NL5 | M |

### 3.1 FL0's numbers

Headless, an open 400 m square, `nav@1` driving every vehicle, on the reference laptop. A tick is
0.06 s of world time: at the demos' 3.6x a tick must take under 16.7 ms.

| Fleet | Tick | Physics | Publishing keys | The rest | Speed it keeps |
|---|---|---|---|---|---|
| 100 kinematic | 3.7 ms | 0.3 | 1.2 | 2.2 | 16x |
| 400 kinematic | 15.8 ms | 1.4 | 9.2 | 5.3 | 3.8x |
| 800 kinematic | 32.2 ms | 2.8 | 15.2 | 14.3 | 1.9x |
| 800 kinematic, fog | 39.2 ms | 2.9 | 15.1 | 21.2 | 1.5x |
| 60 Jolt | 9.4 ms | 6.8 | 1.0 | 1.6 | 6.4x |
| 120 Jolt | 20.6 ms | 15.7 | 1.9 | 3.0 | 2.9x |
| 240 Jolt | 47.8 ms | 42.7 | 3.9 | 1.3 | 1.3x |

The rest is the navigator, the per-step vehicle hooks and the gym's own observation. Before the
overlap fix, 800 kinematic vehicles took 42 ms a tick and the rest was 26 ms. #1358's swarm held 800
under fog at 3.6x.

What follows:
- **Publishing is the largest cost of a kinematic fleet:** about 19 µs per vehicle per tick for its
  25 keys. FL1's summary publishing removes it, and brings 800 kinematic vehicles to about 17 ms
  a tick, near 3.6x.
- **Jolt costs about 0.11 to 0.18 ms per vehicle per tick,** in the physics step itself. About 60
  keep up at 3.6x, as the C++ drive's default assumed. More need a slower world, or fewer on Jolt
  in a mixed fleet.
- **D3: an asynchronous fleet is needed in drawn hosts.** At 3.6x a fleet ticks 60 times a wall
  second, so even 60 Jolt vehicles (9.4 ms a tick) take more than half of a core, and 800
  kinematic ones all of it. On the frame's thread that is the frame rate. FL1 steps a drawn host's
  scene world (its physics and its navigators) on a worker, the frame reading its poses lock-free,
  as #1358's swarm did with `sim_thread`. Headless hosts, the gym and the relay stay synchronous
  and bit-for-bit.
- Under fog, sensing adds about 7 ms at 800.

**FL1a's numbers** (the bench's `[fleet doc]` rows): a fleet declared in the document, 16 rally
points, an opposed layout, a tick's six steps through the runtime (`step_world`), same machine.

| Fleet | Load and layout | Tick, `keys: all` | Tick, `keys: summary` | One publish, all / summary |
|---|---|---|---|---|
| 800 kinematic | 48 ms | 31.4 ms (1.9x) | 11.4 ms (5.3x) | 14.6 / 0.12 ms |
| 2000 kinematic | 250 ms | 83.5 ms | 35.6 ms (1.7x) | 36.4 / 0.35 ms |

Summary publishing took 800 kinematic vehicles past the demos' 3.6x on the stepping thread, better
than the 17 ms FL0 expected. Each tick also ran faster beyond the publish itself (20 ms less where
the publish was 14.5 ms), since the state tree stays small.

**Order.** FL0, FL1, FL2, then FL3, which completes #1358's rework. NL1 and NL2 can run beside
FL1 and FL2. NL3 and NL4 come after FL2, and NL5 any time after NL1. FL4 goes with WD3's next
part.

## 4. Open decisions

- **D1. #1358.** Hold it (draft) and land FL1 to FL3 as its rework, or merge it now and replace
  `nav_swarm` in FL3. Either way, `sim_thread::post` and `tint: satellite` can land on their own
  first.
- **D2. Fleet identity on the wire.** A fleet section with per-entity overrides, against an
  identity entry per entity.
- **D3. Decided by FL0 (§3.1):** a drawn host steps its scene world (physics and navigators) on a
  worker, with the frame reading poses lock-free; headless hosts, the gym and the relay stay
  synchronous. **With ARIADNE_WORLDS §18 (one world, the holon is the world; OW1-OW7, 2026-11-19)**
  the worker is the local server-role holon's: a drawn host runs its world in a holon of its own
  (§18.2, "Local is the same"), and the frame reads that holon's poses. So D3 is built with OW7's
  local holon, not as a second stepping thread, and `poses(fleet)` feeds the holon's entity
  playout (OW5). Fleets need nothing else from §18: they live in `WorldVehicles`, which OW2 puts in
  the holon's world at 120 Hz (OW1's 7-quanta tick is `steps_per_tick` here), OW4's movers reach
  each fleet's navigator as foreign agents, and once OW3 lands a vehicle a participant drives is
  to count as keyed in summary publishing. FleetBench's numbers are re-measured at 120 Hz once
  OW1 lands.
- **D4. A fleet's per-entity keys by default:** `summary` or `all`.
- **D5. Where Austin's lamps come from:** the bundle (does it carry street lights?), generated along
  roads, or OSM later.
- **D6. Time of day:** a scene key a client may override, or the server's world section only.
- **D7. The shadow budget's default N:** desktop against WebGL2.
- **D8. Settled (Joe, 2026-11-18).** Without the trained weights the fleet falls back to the
  built-in net, visibly. `trained` downloads `cvcpkg://cvc/grl-snam-weights` by default, and a
  fleet can name another cvcpkg package or a URL instead.
- **D9. Settled (Joe, 2026-11-18).** The navigator's metric follows the policy: the trainer's
  vehicle with trained weights, each plant's own with the built-in net; `metric:` overrides.
