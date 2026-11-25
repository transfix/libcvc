# Ariadne worlds: the server's document defines the world, and every client runs it

> **Status: WD0 and WD1 BUILT (2026-11-07); the rest is planned.** The relay seeds each room from
> a world document, look included (§15). Q1 is settled: the light rig lives in the server (§16).
> The other decisions are open. One server holds many rooms, each its own world
> (§10); it says which conference features it offers, and stock components follow (§11); clients
> may already hold the world, or fetch it as they need it (§12); a private room is end-to-end
> encrypted, its world simulated by its members, and the relay only forwards it (§13). The same
> world also runs with no relay, as the lab and gym that agents and people drive locally (§14,
> phases L0–L4). It runs with no rendering too: a headless runtime runs the physics and the whole
> world, and a window only adds a view (§14, settled). Parent
> roadmap: [`ARIADNE_CONFERENCE_ROADMAP.md`](ARIADNE_CONFERENCE_ROADMAP.md) (§2 "Server is an `.ari`
> app too", §5g, §5h, §5i, §5k). The server side rides
> [`ARIADNE_SCRIPTING_HOSTS.md`](ARIADNE_SCRIPTING_HOSTS.md) §6 (S0–S7) and §8 (wasm guests).

## 0. The ask

The owner's request, 2026-11-07:

1. **The server side is an Ariadne document too.** It defines the world's geometry and assets, the
   default avatars, the controller types, whether physics is on, and the rest of the rules.
2. **A client copies the world from the server.** On connecting, it takes the world and its
   settings, then runs the physics ticks and its local controllers itself.
3. **One authority, many local simulations.** The holon on the server is authoritative for physics,
   geometry and where entities are. Each client runs its own copy of the server's simulation.
4. **The server tells the client how to interact with it.** Content on the server can change while
   people are in it, and every client's copy follows.
5. **Clients bring their own content.** Avatar assets and controller logic, once designed.

The goal is a persistent, large-scale, real-time world built on what exists: the `.ari` document
model, the replicated state tree, message passing, streams, and scheduled state execution.

**The lab ask, 2026-11-06,** folded in here because it is the same world without a relay: a strong
lab and gym, a digital world that agents and users interact with. Its pieces are components any
document imports, general beyond GRL-SNAM and `cvc::nav`:

- **A world module** that sets the world for vehicles and the navigator, in place of the Austin
  bundle or synthetic city blocks.
- **A vehicle selector.** It previews the vehicle alone in its own lit, shadowed view; shows its
  size, mass, power and top speed; holds its settings and whether it drives the default or a trained
  network; and spawns it, with a name, a team, a primary and secondary colour, and key/value
  metadata.
- **Spawn points and a spawn policy,** set on a minimap: which vehicle type or team uses which
  points, picked at random, round-robin, and so on.
- **A physics panel:** physics on or off, and stats on the active entities and forces.

**The delivery, rooms and privacy ask, 2026-11-07:**

- **Clients may already have the world.** A client may hold the same scene locally, have
  downloaded it before, or ship bundled with a set of assets. Worlds will grow large, with many
  assets, so a client should not have to download hundreds of megabytes at once. An application may
  be deployed with its client side and its server side each holding a copy of the world up front.
  A client without the world, its controllers or anything else must still be able to download all
  of it.
- **The server defines the conference features, and the client detects them** so it can join
  conferences. Conference widgets belong in the stock component set.
- **One server, many rooms,** each with its own scene, world, physics, actors and entities.
- **End-to-end encryption through the relay.** Users talk to each other so that the server cannot
  see what they say: it only relays. They can make a private room with a private conference and a
  private world holon of their own; the stock default world is enough for now. txwtf's encrypted
  messaging is the model.

**The headless ask, 2026-11-07:** a headless runtime must still run physics and support the world
with no local rendering. The relay, a gym worker, a test and a terminal host run the same world a
window draws (§14, "Headless first").

## 1. What exists

Checked against master 797ba4d9; the lab's rows, from "Vehicles and their navigator" down, against 19423da3.

| Part | Today | Where |
|---|---|---|
| World geometry | A world document (WD1). `arisrv` runs `cvc://worlds/default.ari` (a 20 m ground plane and a 1 m reference box), `--scene large` runs `cvc://worlds/large.ari` (a 100 m plane and 24 meshes, the built-in bunny), and a `--doc` whose document has a `scene:` is the world instead. The relay publishes it in one revision at start-up. The C++ twins `default_world()` / `large_world()` are the fallback when the file can't be found; a test pins each equal to its document. | `src/cvc/ariadne/worlds/`; `room_world_from_scene`, `write_room_world` in `src/cvc/ariadne/conference/room_scene.cpp`; `src/arisrv/main.cpp` |
| Wire format | Scalar leaves under `conf.rooms.<room>.scene.{epoch, at, rev, env.*, objects.<id>.*}`: a subset of the document's `Scene` / `SceneNode` / `SceneLight`. Meshes go by `source.uri`. A removed entry is deleted from the tree (deletes replicate since transfix/libcvc#1473); a reader takes only entries with `present = 1`, which a write sets last. The scene is a revisioned section (§3): `rev` is written last, beside its writer's `epoch`, and `at` is 0 (it applies on arrival). Each light, object and fit carries the epoch it was written in, so a reader skips what an earlier relay left behind. | `inc/cvc/ariadne/conference/room_scene.h` |
| Look | The world's (Q1, WD1). The world document declares the light rig and the shadows, and the relay publishes them through `scene.env.lights.*` and `scene.env.shadows.*`, which now carry a rig's tuning knobs (stage, key, fill, back, warmth, environment, ambient, wash, sun) and the shadows' re-bake interval. The conference document has no `scene:` block. | `default_room_rig`, `write_scene_env` in `room_scene.cpp`; `src/cvc/ariadne/worlds/default.ari` |
| Who writes the scene | Only the relay. A client may write only its own participant subtree (bound writes, default on). | `room_write_gate.h` |
| Client realization | `room_view` rebuilds the scene from the tree whenever the scene's stamp moves (a new `rev`, or a restarted relay's under a new `epoch`), with the same realizer a document's `scene:` uses. The holon's physics statics follow the same stamp. | `src/cvcGL/ariadne/room_view.cpp` (`sync_scene`) |
| Physics statics | Built from the scene objects: a plane becomes a 0.2 m slab, and boxes and spheres become themselves. Meshes are skipped, so the bunnies don't collide. | `room_statics` in `src/cvc/ariadne/holon_sync.cpp` |
| Avatar looks | A catalog the relay seeds: ten colours plus an automatic hue, and seven accessories. A participant picks from it; every client draws the pick the same way. | `default_avatar_catalog`, `seed_avatar_catalog` in `room_avatar.cpp` |
| Body | **The world's** (WD2): its archetype's, published in `avatars.archetypes`. Every avatar has the first archetype; the holon re-creates the bodies where they stand when it changes, and `room_view` draws the markers at its size. `kAvatarWidth` / `kAvatarHeight` remain as the default person's, which the accessory envelope and the catalog are designed on. | `world_rules.h`; `room_rules.h`; `avatar_world::set_body` |
| Controller | **The world's** (WD2): the first its archetype allows, from `controllers.*` (a kernel by name and version, and its params). This build runs one kernel, `walker@1`, and the relay refuses a world naming another. The avatar panel shows the controller's id, and arigl's walk keys ask for its `walk_speed`. | `drive_walker` in `avatar_physics.cpp`; `controller_kernel_known` in `world_rules.cpp` |
| Energy policy | **Replicated**, a section of the world's rules since WD2: `conf.rooms.<room>.avatars.policy.{energy.*, max_speed, allow_vertical}`, stamped `{epoch, at, rev}`. The world document sets the numbers; `--avatar-energy` and `--avatar-empty` override it. | `write_avatar_policy` in `room_rules.cpp`; `sync_rules` in `holon_sync.cpp` |
| Speed veto | **The world's** (WD2): the policy's `max_speed` and `allow_vertical`. Each step applies them under the policy in force at its tick. | `holon::veto` in `holon.cpp` |
| Gravity | **The world's** (WD2): its scene's `physics: gravity`, published in `physics.gravity`; the walker prices carrying a body by it (it was 9.81 compiled in). | `avatar_world::set_gravity` |
| Holon | One world clock at 120 Hz. The relay simulates every avatar in Jolt. A client simulates its own avatar too, predicts it, and snaps and replays past 5 cm. Peers are kinematic, played from a buffer. Intents go up; pose snapshots come down at 60, 30 or 20 Hz depending on round-trip time. | `inc/cvc/ariadne/holon.h`, `holon_sync.h`, `room_pose_relay.cpp`, `inc/cvc/core/world_clock.h` |
| Determinism | Jolt runs in cross-platform determinism mode. A golden state hash pins it on Windows, Linux, macOS, x86_64 and arm64, and wasm-mt (its own CI job). | `physics_world_test.cpp` (`exact_scene_hash`), `physics_parity_test.cpp`; ci.yml "physics replay (wasm-mt, node)" |
| Documents on the server | `arisrv --doc server.ari` (S0, transfix/libcvc#1046) runs one headless: DSL programs on the relay's loop once per room step, the holon's world clock at `sim.tick`, the room read-only at `room`, `server: publish:` namespaces sent to every client, and `requires: [relay]`. A `scene:` with nodes or lights is the room's world, which the relay publishes before `init:` runs (WD1). Its physics and bodies are kept on `ServerDocument::scene()` but not simulated yet, and the load warns so: §14's headless world runtime will simulate them as the holon's one physics world. | ARIADNE_SCRIPTING_HOSTS.md §6.1, §6.5.3; `inc/cvc/ariadne/server_document.h` |
| What a document can already say | Far more than the room tree carries: `scene:` with primitives, meshes with `fit:`, heightfields, light rigs and shadows, and the custom node types `generated_world` and `cloud_sky`. `scene.physics:` gives gravity, `fixed_dt`, materials and per-node bodies (dynamic, kinematic or static; box, sphere, hull, mesh or terrain). All of it parses in core libcvc, with no GL. | `docs/ARIADNE.md` (Scene, Physics); `src/cvc/ariadne/loader.cpp` |
| Persistence | None. The room lives in memory, and a restart re-seeds it. Only the device-key pins can go to a file. `distributed_state_config::blob_store_path` is not read. | `src/arisrv/main.cpp` (`--device-pins`); DISTRIBUTED_STATE_ROADMAP.md |
| Scale pieces | Built but unused by the conference: hash partitioning (`state_hash_partition`, `state_cluster_shard::partition()`) and delegation leases (`state_delegation_manager`). | `inc/cvc/state/` |
| Vehicles and their navigator | `cvc::nav::sim_world` runs agents on an occupancy raster. **L0's third part:** agents are added and removed at run time (`add_agent`, `remove_agent`), each with an id that is never reused; each agent has a policy (`set_policy`: nav@1's, a `.cvcnav` loaded once per content hash by `policy_library`, run as one batch per policy by `drive_step_policies`); and each has an identity (name, team, primary and secondary colours, metadata), bound to Python and published under `<prefix>.nav.agents.<id>.*`. With a plant set, the plant adds and drops the agents' vehicles (`vehicle_plant::add_vehicle`, with each chassis body entering and leaving the physics world). An agent can have a belief plane of its own (`agent_spec::own_plane`, `add_plane`, `remove_plane`). Agents come and go while nav_stats is armed too: an added one counts from its first step, a removed one keeps its finished record (`nav_stats_collector::add_agent`, `remove_agent`). | `inc/cvc/nav/sim_world.h`; `inc/cvc/nav/policy_library.h`; `src/cvc/nav/sim_world_binding.cpp` |
| Vehicle plants | `plant_host` puts each agent on a plant: nav's kinematic bicycle, a Jolt vehicle, the dynamic single track or the dynamic four-wheel model. Presets `vehicle_desc::humvee()` and `roboracer()` carry mass, wheelbase, track, engine and `max_speed`. | `inc/cvc/physics/plant_host.h`, `inc/cvc/physics/vehicle.h` |
| Generated worlds in a document | The `generated_world` node builds terrain, roads, buildings, trees and water and publishes its height field (`RealizedScene::height_fields`), so `ground:` stands on it. **L0's second part:** the node's params, `world.*` keys, requests, stats and colliders are core libcvc (`scene_generated_world.h`); its `body:` is its terrain and buildings, replaced when it regenerates; headless, the scene's world runs the node and `SceneWorld::generated_world(id)` hands over the whole world (raster, roads, buildings, water). | `inc/cvc/ariadne/scene_generated_world.h`; `src/cvcGL/ariadne/world_nodes.cpp` |
| World to navigator | **L0's last part, built:** core `cvc::nav::world_grid` makes any world (generated, a data bundle's city, a flat square) the navigator's grid: occupancy walled at the edge, the terrain's height, the signed distance, free points drawn the same on every platform, and a `sim_world` config. `SceneWorld::grid(node, cell, half)` gives a generated or data node's grid, made once per world; pycvc's `nav_world_grid` and `AriApp.grid`. The `pycvc_gl.cars` template still has its own Python copy until it moves onto the gym (L4). | `inc/cvc/nav/world_grid.h`; `inc/cvc/ariadne/scene_world.h`; `bindings/pycvc/pymod_gl/cars.py` |
| Physics with no rendering | **L0's first part, built:** a document's `scene.physics:` world and its geometry nodes' `body:`s are core libcvc's `cvc::ariadne::SceneWorld` (`scene_world.h`), built from the scene alone. A headless `AppRuntime` (`ari`, `ari test`, pycvc's `AriApp`, `arix`) runs it on the world clock; cvcGL builds the same world through it and binds its nodes, and a test pins the two worlds' state hashes equal. The `scene.physics` token. L0's second part runs a `generated_world` node's body and a data node's headless too, and a test pins a generated world's state hash equal drawn and not. Still renderer-only: another custom node's body. | `inc/cvc/ariadne/scene_world.h`; `src/cvcGL/ariadne/scene_realize.cpp` |
| Physics stats | `physics::world_binding` writes `tick`, `step_ms`, `bodies`, `active_bodies`, `contact_events` and a snapshot to its prefix. A document's scene world publishes `physics.stats.{tick, time, step_ms, bodies, active_bodies, contact_events, saturated}`, drawn or not (L0). | `inc/cvc/physics/world_binding.h`; `inc/cvc/ariadne/scene_world.h` |
| Step and restart | `sim_transport.ari`'s Step and Restart buttons raise `sim.step` and `sim.restart`. **Built in (L0):** every `AppRuntime` handles them. A step runs one more quantum on the scene's clock, and a restart puts the clock back to tick 0 and rebuilds the scene's world as the document built it (`SceneWorld::restart`, headless and drawn), which then steps as a fresh world; `<prefix>.sim.restarts` counts restarts. A host may replace either. | `src/cvc/ariadne/components/sim_transport.ari`; `inc/cvc/core/world_clock_binding.h` |
| Rooms | One room per relay: `arisrv --room <id>` (default `standup`). The tree is already shaped for more, `conf.rooms.<room>.*`, and the area-of-interest, avatar and scene paths all take a room id. | `src/arisrv/main.cpp`; `inc/cvc/ariadne/conference/` |
| Conference in documents | The stock components include `room_users.ari` (the Users window) and `chat.ari`. The rest of a conference (joining, media controls, tiles, the avatar panel, the 3D room) is in the example documents `conference_roster.ari`, `conference_media.ari`, `conference_live.ari` and in C++ widgets (`room_view`, `stream_view`). | `src/cvc/ariadne/components/`; `examples/ariadne/documents/` |
| Feature detection | A document's `requires:` names host features (`gl.scene`, `net`, `media.player`, ...) and the host says whether it has them. Nothing asks the *server* what it offers. | `src/cvc/ariadne/requires.cpp` |
| Asset cache | A document's `data:` block fetches cvcpkg archives checked against the registry's sha256 and keeps them in a local cache by hash (`CVC_DATA_CACHE`, else the user's cache directory). A browser has no cache: downloads are not kept. | `inc/cvc/ariadne/data.h` |
| Keys and encryption | Each device has an Ed25519 key (a 32-byte seed, OpenSSL) and the relay pins it. Links to the relay use TLS, so the relay reads everything it forwards. SFrame end-to-end media is planned (§6g of the conference roadmap); no end-to-end encryption is built. | `src/cvc/state/state_device_key.cpp`; `inc/cvc/ariadne/stream/stream_relay.h` |
| Views | A document's `viewport` widgets draw the scene from cameras of their own, with per-view overrides, and `scenes:` gives a viewport a scene of its own: a studio set for one vehicle costs no host code. **L0:** a click in an `input: pick` view gives a world point and the node it hit, under the view's `pick:` key and to its `on_pick:` (`SceneViews::pick`). | `docs/ARIADNE.md` §5.18; `src/cvcGL/ariadne/scene_views.cpp` |

**What that adds up to.** The data already flows the way the ask wants: the server holds the state,
and each client rebuilds its view and its physics from the tree. Until WD2 most of the *rules* were
compiled into both binaries: the body, the controller and the speed veto. Prediction was right
because the client and the relay ran the same code, not because the relay told the client how its
world works.

**The energy policy was the exception, and it was the model.** It was a revisioned section the
server wrote, with its revision written last, and both the relay's holon and every client read it
from the same leaves. WD2 generalized that pattern to the world's rules: the body, the controller,
the veto and gravity are sections now too, each with an activation tick (§3).

**Two loose ends, fixed by WD0:**

- The comment above the seeding in `src/arisrv/main.cpp` said the world is lit "under a sun"; no
  light was seeded. The world's rig lights it now (WD1).
- `--set` was applied before the scene was seeded, and seeding happened only at revision 0. A
  `--set` of `scene.rev` therefore stopped the default world from being seeded at all, and a
  `--set` of an object's leaves was overwritten by the seed or quietly added to it. `--set` now
  comes after the world and the room's policies (and before a server document's `init:`, so its
  `ariadne.limits.*` still bound the document), and the world is published whatever the
  revision.

## 2. Two documents: the world and the viewer

| Document | Runs on | Says | Today |
|---|---|---|---|
| **World document** | the relay: `arisrv --doc world.ari` (S0) | the scene, its look and its physics; avatar archetypes and the look catalog; the controllers on offer; the movement and energy policy; entities; room rules and server programs | C++ seeds and flags |
| **Viewer document** | each client | windows, HUD, camera, input bindings, the client's own preferences | the conference document, which also owns the look |

- **libcvc ships default worlds** as components: `cvc://worlds/default.ari` (today's 20 m room) and
  `cvc://worlds/large.ari` (today's `--scene large`). `--scene default|large` becomes a shorthand
  for those documents and is kept for compatibility.
- **A site ships its own world document** beside its viewer document, and the relay runs it.
- **The look is the world's** (Q1, settled 2026-11-07). The light rig and the shadows move from the
  viewer document into the world document. The relay publishes them through `scene.env.lights.*`
  and `scene.env.shadows.*`, which the wire already carries and `read_room_scene` already turns
  into the scene's lights, rigs included.
- **Clients never receive the world document's source.** The relay publishes what the document
  *derives*: the sections in §3. The document can hold room rules, server programs, and anything
  else the operator doesn't want sent to clients. Clients need the world, not the program that
  built it.

A world document, as proposed syntax. `scene:` is today's syntax, and so is the `world:` block's
`avatars: archetypes:`, `avatars: policy:` and `controllers:` since WD2 (`world_rules.h`; `catalog:`
is still a warning). The `conference.world` token and `room-rules-step` do not exist yet. S0's `relay` token holds in every
server document, and `conference.world` would build on it.

```yaml
# world.ari -- run by the relay: arisrv --doc world.ari
requires: [conference.world]

scene:
  lights:
    - {light: sun_rig, rig: three_point, stage: {center: [0, 0, 1], radius: 10}}
  shadows: {enabled: true, resolution: 2048}
  physics: {gravity: [0, 0, -9.80665]}
  nodes:
    - node: ground
      source: {plane: {size: 20}}
      body: static
    - node: ref
      source: {box: {size: [1, 1, 1]}}
      transform: {position: [0, 0, 0.5]}
      body: static
    - node: crate
      source: {box: {size: [0.6, 0.6, 0.6]}}
      transform: {position: [3, 0, 2]}
      body: {motion: dynamic, mass: 20}     # a server-dynamic entity (section 4)

world:
  avatars:
    archetypes:
      person:
        body: {mass: 70, width: 0.5, height: 1.7}
        controllers: [walker@1]
    catalog: {colors: auto, accessories: [cap, tophat, halo, backpack]}
    policy:
      max_speed: 4.0
      energy: {enabled: true, force_max: 840, power_max: 1200, reserve_max: 30000, empty: rest}
  controllers:
    walker@1: {kernel: walker, version: 1, params: {walk_speed: 2.0}}

on_tick: (room-rules-step)                  # server programs (S0's DSL; S1's Python on arisrvx)
```

## 3. Publish definitions, not just state

The relay turns the world document into **sections** in the room tree. Each section has its own
revision, written last, exactly as `avatars.policy` does today.

| Section | Path under `conf.rooms.<room>.` | Holds | Today |
|---|---|---|---|
| scene | `scene.*` | objects, lights, shadows, background | built (objects; lights unused) |
| world descriptor | `scene.world.*` | a generator, its params and seed (GENERATED-WORLD-ROADMAP.md G5) | planned there |
| physics | `physics.*` | on/off, gravity, `fixed_dt`, materials; bodies per scene object | built (gravity; WD2) |
| archetypes | `avatars.archetypes.<i>.*` | body, look envelope, allowed controllers | built (body, controllers; WD2) |
| catalog | `avatars.catalog.*` | colours, accessories | built |
| policy | `avatars.policy.*` | energy, `max_speed`, later per-controller limits | built (energy, `max_speed`, `allow_vertical`; WD2) |
| controllers | `controllers.<i>.*` | kernel or guest, version, params, content hash | built (native kernels; WD2) |
| entities | `entities.<id>.*` | §4 | new |
| spawns | `spawns.*` | spawn points (position, heading, tags) and the spawn policy (§14) | new |
| stats | `physics.stats.*` | bodies, active bodies, contacts, step time; per entity its energy and forces. Read-only, written by whoever simulates: the relay, or a local world | new |
| manifest | `world.*` | the world's name, version, and the revision of every section | new |

**Rules for the sections:**

- **Assets go by URI plus content hash.** Meshes, textures and controller modules are fetched over
  HTTP through the existing URI cache, never through the relay's state stream. The hash makes a
  cached copy safe to reuse and a changed asset impossible to mistake for the old one.
- **The relay hosts what it publishes.** An asset's URI points at the relay's own origin or a CDN it
  names. This matters once users bring assets (§6): peers never fetch from an address another user
  chose.
- **Changes that affect the simulation carry an activation tick** (Q4, built in WD2). When
  `physics.*`, `controllers.*`, an archetype or the policy changes, the section says which
  world-clock tick it takes effect on (`at`), and the relay switches on that tick. The holon keeps
  the rules by tick, and every step, the relay's, a client's prediction and its replay, runs under
  the rules in force at its tick. A client that learns of a change late re-runs its prediction from
  that tick; the relay's samples correct it until then, as its reconcile always has. A change while
  the room runs takes effect half a second ahead (`rules_activation_tick`), so clients that hear of
  it in time switch with the relay. Cosmetic sections (lights, looks) apply on arrival.
- **Each section carries its writer's epoch** (`section.h`), beside its revision. A restarted relay
  writes its sections again at revision 1, which a client that outlived the old relay already holds;
  the epoch tells them apart, and a new one replaces the old run's schedule. The room scene is
  stamped the same way, so such a client rebuilds its drawn room and its physics statics.
- **A client keeps what an old relay wrote.** A snapshot deletes nothing, and the new relay never
  removes an id it never knew, so a client that outlived a relay still holds that relay's
  objects and lights. Lists that a reader takes `count` entries of (the rules' sections) are
  immune. Entries keyed by id are not: the scene stamps each light, object and fit with the epoch
  it was written in, and a reader takes only the entries of the scene's own epoch (unstamped ones
  too: a writer from before epochs, or a `--set`). Any new keyed section needs the same.
- **The relay is still the only writer.** A section is written by the relay's own code: the world
  document's programs and the script thread, through S0's `server: publish:` rule (§6.5.3).

This is how "the server tells the client how to interact with it" works: the client's simulation is
built entirely from these sections, so when the world document changes a section, every client's
copy changes with it.

## 4. Entities: one table for everything that moves

Today two kinds of things exist: the scene's statics and the participants' avatars. A world needs
props, vehicles, doors and characters the server drives. Each **entity** has:

- an **archetype** (body, look, allowed controllers);
- a **controller**, or none;
- an **owner**: the server, or a participant;
- an **identity**: a name, a team, a primary and secondary colour, and key/value metadata. The
  colours drive the archetype's look; the team is a label that spawn rules filter on (§14); the
  metadata is the user's own and nothing reads it;
- a **simulation class**:

| Class | Who simulates it | Clients | Example |
|---|---|---|---|
| static | everyone, from the scene | build it once per revision | the ground, walls, a building |
| server-dynamic | the relay | interpolate its pose stream | a pushed crate, a door, an NPC |
| client-predicted | the relay, and its owner's client | the owner predicts and reconciles; others interpolate | your avatar, a car you drive, an object you hold |
| cosmetic | each client alone | never replicated | particles, idle animation |

- **Avatars become entities whose owner is a participant.** Today's split (my avatar predicted, my
  peers kinematic) is exactly this table with two rows filled in.
- **Entities live under `conf.rooms.<room>.entities.<id>.*`.** Server-dynamic and client-predicted
  poses ride the pose stream (`room_pose_relay`, with its interest tiers), not the tree.
- **Meshes get colliders.** The document's `body: {shape: hull | mesh}` already exists; `room_statics`
  gains the shapes it skips today.

## 5. Controllers: code the server ships, run the same on both sides

A controller is a pure step:

```text
(body state, intent, dt, params, contacts) -> forces and torques (or a wanted velocity)
```

- **The server's rules clamp whatever it returns.** The energy budget, the force limit and the speed
  veto are applied after the controller, by the holon. A controller can be wrong; it cannot cheat.
- **It runs on the relay for authority, and on the owner's client for prediction.** No other client
  runs it: they interpolate the result.

**Two kinds:**

1. **Native kernels, chosen by data.** A registry of kernels by `name@version`, compiled into libcvc:
   `walker@1` is today's `drive_walker`. The second is a vehicle on the Jolt vehicle plant
   (NAV-VEHICLE-PHYSICS-ROADMAP.md; §5k's Humvee); a flyer might follow. The world document picks
   kernels and sets their params.
   - **`nav@1` is the navigator** (§14): `sim_world`'s planner over the entity's plant. Its params
     name a policy by URI and content hash (a trained `.cvcnav`), or none for `default_biased`.
     Entities naming the same policy share one loaded copy, so a policy per entity costs nothing
     extra. It runs as one batch over every entity that uses it, as `sim_world` does today.
2. **Portable guests.** Logic the world, or later a user, ships as a module: a wasm guest (scripting
   §8) or a DSL program. The same module runs on the relay and on the owner's client.

**What makes prediction work:**

- **A determinism gate per kernel.** The physics world already has one: a golden state hash on every
  target, wasm included. Each controller kernel gets the same gate: a scripted run whose state hash
  is pinned and must never be re-baselined per platform.
- **Guests are metered the same on both sides.** That means the install-time gas transform (scripting
  §8.5, G2), not wasmtime's fuel, which exists only natively.
  Guests are built without relaxed SIMD, and NaNs are canonicalized. Guests run as scheduler citizens
  under the DSL's limits, as §8.5 requires of every guest.
- **Capability negotiation.** A client says which kernels and guest engines it has. If it lacks the
  one its avatar's controller needs, it doesn't predict: it interpolates its own avatar, which feels
  slower but stays correct.

**Python's place** is the relay's script thread (S1): NPC planners, room rules, services, anything
that decides intents at a few hertz and can land a tick or two late. It is not a per-tick controller
language: the browser has no Python host yet (W1), and a controller must run in the owner's client.

## 6. Bringing your own

Each tier keeps the rule in conference roadmap §5k: what a user brings, the server validates and
runs; it never trusts it.

| Tier | The user brings | The server | Phase |
|---|---|---|---|
| 1. Pick | a choice from the catalog | offers the catalog | built |
| 2. Look | a mesh or texture by hash | checks size, triangle count, texture pixels and the envelope; then hosts the asset itself (§3); the physics body stays the archetype's | WD6 |
| 3. Body | a mass and size | clamps them to the archetype's ranges; the energy budget is the same for everyone, so size and mass are trade-offs, not advantages | WD6 |
| 4. Controller | a guest module | runs it sandboxed and metered on the relay; the owner's client runs it for prediction; the holon clamps its output | WD7 |

- **Every tier is keyed on a principal** (S5): who brought it, so who can be refused or rate-limited.
- **Tier 4 needs a threat model first** (§5i D4). The output clamp bounds what a controller does to
  the world; the threat model covers what it costs the relay (gas, memory, time per tick) and what
  it can read.

## 7. Server programs and live changes

The world document's programs run on the relay. Through them the world changes while people are in
it.

- **Room rules and spawns in the DSL** (S0): `on_tick`, events, and verbs that spawn, move and remove
  entities, change sections, and answer `room-call`s. They run under the DSL's step and time caps.
- **Python on `arisrvx`** (S1): the script thread, for anything heavier. Its writes land on a later
  relay tick.
- **Editing the world live.** A changed world document reloads into new section revisions. Static
  edits apply on arrival; simulation edits take effect on their activation tick (§3).

## 8. Persistence

- **Separate the definition from the state.**
  - The **definition** is the world document and its assets: versioned, content-addressed, and
    never mutated in place.
  - The **state** is what happened in the world: placed entities, edits, server-dynamic poses at
    rest, and per-principal saved state such as a parked car.
- **The state is snapshotted and journaled.** A restart is the document, then the latest snapshot,
  then the journal since. `blob_store_path` is the obvious hook (DISTRIBUTED_STATE_ROADMAP.md).
- **Not everything persists.** Intents, live poses and media don't. Last-known positions might, per
  principal.

## 9. Scale

- **Cells.** A large world is split into cells, each a subtree with its own section revisions
  (`scene.cells.<i>_<j>`, conference roadmap §5h H3). A client pages cells in and out by interest,
  as the spatial interest filter (§5h.1) already filters poses and media.
- **Generated worlds chunk the same way.** `generated_world`'s terrain chunks and LOD tiers should
  line up with cells, so a cell's world descriptor, assets and statics come together.
- **One holon per region.** H3's subdelegation gives a region to a subordinate server under a
  delegation lease. H4 hands an entity's authority across a boundary. The partitioning and lease
  machinery exists and is unused.
- **Persistence per cell.** A cell's snapshot and journal move with it when its region moves.

## 10. Rooms: one server, many worlds

A relay serves one room today. A world server serves many, and each room is a world of its own.

- **The server's document declares its rooms.** Each room names a world document
  (`cvc://worlds/default.ari`, or the site's own) or defines its world inline. Several rooms can
  share one world definition; each still has its own copy of the state.
- **Each room has its own everything:** sections (§3), entities, policy, holon and world clock. A
  room's physics, controllers and rules never reach into another room.
- **A lobby section lists the rooms:** `conf.lobby.rooms.<id>.{title, world, features, capacity,
  occupancy, access}`, where access is open, by token, by invitation or private (§13). A client
  reads the lobby before it joins anything.
- **Rooms can be made at run time,** by the server's programs (§7), or by a participant where the
  server's rules allow it: a private room (§13) or a copy of a template world.
- **A client reads only the rooms it joined.** Bound reads keep other rooms' trees off its link,
  as bound writes already keep it to its own participant subtree.
- **Rooms spread over servers** the way cells do (§9): a room is a natural unit for a holon and for
  the partitioning and leases that exist and are unused.

## 11. What a server offers: conference features, detected

The server says what it offers; a client's stock components show what is there.

- **A features section per room, and one for the server:** `conf.features.*` and
  `conf.rooms.<room>.features.*`, revisioned like the others. Entries name a feature and its
  version and limits: `conference.chat`, `conference.media` (audio and video, with codecs and
  simulcast), `conference.screen`, `conference.avatars`, `conference.world` (a shared scene and
  holon), `conference.e2ee` (§13), `conference.rooms` (the lobby and making rooms).
- **The client learns it on connect.** Before it shows anything, it reads the server's features
  and the lobby. `requires:` gains server tokens beside the host's (`server.conference.media`, ...),
  so a document can say what it needs from a server as it does from its host.
- **The conference is stock components.** A document gets a conference by importing them and
  pointing them at a server: `cvc://conference/` with `lobby` (rooms, join, create), `room_users`
  and `chat` (moved there), `media_bar` (camera, microphone, screen, devices), `tiles` (the stream
  views), `avatar_panel`, `room_view` (the 3D room), `room_settings`, and `e2ee_status` (keys,
  verification, who is in a private room). Each hides itself when the server doesn't offer its
  feature.
- **The example conference documents shrink** to a layout over those components, and a
  site's viewer document is the same components in its own layout.

## 12. Getting the world to a client: bundled, cached or fetched

Worlds will be large. A client should be able to start without downloading all of one, and one that
already holds the world should download nothing.

- **The manifest lists every asset by hash, size and priority.** Priority says when an asset is
  needed: the statics' colliders first, then by cell (§9) and level of detail, coarse before fine.
  Controllers that are guests (§5) are assets too. A native kernel is not: the client has it or
  says it hasn't (§5's capability negotiation).
- **A client looks for an asset by hash, nearest first:**
  1. the application's own bundle: a client built or packaged with a world pack;
  2. the local content cache: what it fetched before, or a pack copied in ahead of time;
  3. a world pack the client was pointed at: a cvcpkg package, an archive from a URL, or a data
     directory on disk (below);
  4. the relay's origin, or the CDN it names (§3).
  The hash makes every local copy safe to use without asking the server.
- **A world pack** is a world document and its assets, versioned and content-addressed. The relay
  is deployed with one, and the same pack can be bundled with a client. When the server's world
  moves on, the client fetches only the assets whose hashes it lacks.
- **A pack can come from anywhere** (Q16, settled 2026-11-07). What makes it a pack is its index
  (the world document, every asset's hash and size, and where in the pack each one is), not how it
  arrived:
  - a cvcpkg package, checked against the registry's sha256, as a document's `data:` archives are
    today;
  - a generic archive downloaded from a URL (zip or tar), checked against a hash the manifest or
    the user gives;
  - a data directory on disk: a mounted share, a checkout, an unpacked archive, an application's
    install folder.
  Each asset is checked against its hash as it is read, so a pack from an untrusted place can't
  substitute content: it can only be missing things, which the client then fetches from the relay.
  A client is pointed at packs by its viewer document, a command-line flag or an environment
  variable, and may use several at once.
- **Fetch by need, not up front.** A client fetches the room it joins: the statics' colliders
  where it spawns first, then the cells near it, coarse level of detail before fine. Until an
  asset arrives the client draws a stand-in (its bounds, or a coarser level), and until a cell's
  colliders arrive it interpolates there rather than predicting. Later cells stream as the client
  moves.
- **Everything is fetchable.** A client with nothing still gets the whole world from the relay: the
  manifest names every asset, guest controller and texture. Bundles and packs only make that
  faster.
- **The browser needs a content cache:** the Cache API or OPFS, keyed by hash, so a page that
  loaded a world once doesn't fetch it again.
- **Live edits list changed hashes** in the section revision that carries them, so a client fetches
  only what changed.
- **As built (OW5k, a pack as a directory):** `cvc::ariadne::world_pack` (world_pack.h) is a
  directory with an index, `world-pack.index` at its root: one line per file,
  `<sha256> <size> <path>`, the path relative to the root. `write_world_pack_index` writes one for
  every file under a directory. A client is pointed at packs by `CVC_WORLD_PACKS` (directories,
  separated as PATH's are) or `world_replica::add_pack`. When the room's document URI does not
  give the relay's bytes here, the replica finds the document by the SHA-256 the room names
  (`room_world_source`), in the first pack that lists it and whose files all match its index
  (`world_pack::verify`: each file's size and SHA-256). It builds the world from there, and the
  document's assets resolve inside the pack. A pack with a file that differs from its index is
  refused, and the replica's status says which file (`AriadneOneWorld`). Still to come: packs from
  a cvcpkg package or an archive at a URL, a viewer document's and a command-line flag's packs,
  the relay deployed with a pack, the manifest's priorities, and fetching from the relay.

## 13. Private rooms: end-to-end encrypted, the relay only forwards

A private room's members talk, share media and run a world together; the relay forwards their
traffic and cannot read it. txwtf's encrypted messaging (`docs/ENCRYPTED_MESSAGING.md` there) is the
model: X3DH, the Double Ratchet, keys only in clients.

**What the relay can and can't see.**

- **It can't see content:** chat, media payloads, and the private room's world (sections, entity
  state, intents, poses) reach it as ciphertext.
- **It sees metadata,** as txwtf's server does: who is in the room, who sends, when and how much. It
  routes on a small clear envelope: room, sender, sequence, kind, and for media the RTP header
  SFrame leaves clear (enough for routing, simulcast layers and the active speaker).
- **It is not defended against** traffic analysis, a malicious member's client, or a weak backup
  passphrase.

**Keys.**

| Key | Type | Purpose | The relay holds |
|---|---|---|---|
| Device key | Ed25519 | today's device identity; signs the keys below | public only (pinned today) |
| Identity DH key | X25519 | X3DH | public only |
| Signed prekey | X25519, signed | X3DH, rotated | public only |
| One-time prekeys | X25519 | X3DH, used once, replenished | public only |
| Sender key | a chain key per member per room | encrypts what a member sends to the room | never |
| Backup | AEAD ciphertext, Argon2id from a passphrase | restoring keys on a new device | opaque ciphertext |

- **A key directory on the relay** holds the public keys (txwtf's E1). A user's devices group under
  their principal once S5 exists; before then a device is its own identity.
- **Verification is the users'.** The relay could hand out its own keys, so clients show a safety
  number (both devices' key fingerprints) to compare, and warn when a member's key changes.

**Sessions and room keys.**

- **Pairwise sessions:** X3DH to start, the Double Ratchet per message, between every two members.
  They carry direct messages and nothing else but keys.
- **Room traffic uses sender keys** (txwtf's group design): each member has a chain key for the room
  and sends it to every other member over their pairwise sessions; a member's messages are
  encrypted once under its chain and fanned out by the relay. Someone leaving makes everyone
  rotate. txwtf's per-message fan-out of a fresh key (`grp-dr-v1`) is right for chat but not for
  120 Hz intents and media frames; sender keys are.
- **Media is SFrame,** its keys derived from the sender key, as the conference roadmap's §6g plans.
- **Joining needs a member, not the relay.** An invitation is a room id and a secret passed out of
  band or over a pairwise session; a member admits the newcomer by sending it the sender keys. The
  relay can admit someone to the traffic, never to the content.
- **Larger rooms may want MLS** (RFC 9420): rekeying in log n instead of n. Sender keys first (Q13).

**The private world.**

- **The relay can't be its authority:** it can't see the poses. The members simulate it.
  - **Proposed first: one member's client is the authority** (the room's creator), running the
    holon that the relay runs for a public room. The others predict and reconcile against it, as
    they do against the relay. When it leaves, the next member takes over from its last snapshot.
  - **Later: lockstep.** Every member runs the same deterministic simulation from everyone's
    intents. Jolt's cross-platform determinism gate makes it possible; there is then no authority
    to trust or lose.
- **The world is the stock default world** (`cvc://worlds/default.ari`). Every client has it bundled
  (§12), so a private room fetches no assets and leaks nothing by fetching them.
- **A private world of one's own comes later,** with assets encrypted the way txwtf's media store
  does it: a content key per asset (XChaCha20-Poly1305), sealed to each member's key; the relay
  stores and serves only ciphertext.
- **What doesn't work in a private room:** anything the relay does by reading content. The speed
  veto and the energy policy are enforced by the authority member instead; area-of-interest
  filtering by position doesn't work unless members put a coarse cell id in the clear envelope
  (Q15); the relay keeps no world state and no history.

**Crypto library (Q12).** txwtf uses libsodium (X25519, Ed25519, XChaCha20-Poly1305, Argon2id) and
its sumo build in the browser. libcvc's device key uses OpenSSL, which has X25519, Ed25519, HKDF and
ChaCha20-Poly1305 but not XChaCha20, and Argon2 only from 3.2. Proposed: libsodium for the
end-to-end layer, through a cvcpkg recipe, on every target including wasm.

## 14. Local worlds: the lab and gym

The world document doesn't need a relay. Run in arigl, in the browser, from pycvc or headless, the
process runs the world document itself and is its own authority: it writes the sections of §3 into
its own state tree and simulates every entity. That is the lab: people and agents drive vehicles in
a world, change it, and measure what happens. With reset, step and observe from Python it is a gym.

- **One runtime, two places.** A C++ **world runtime** reads the sections and runs the entities:
  their plants, their controllers, physics, spawning. Locally it is the authority. On the relay
  (WD1 on) the same runtime is the holon's authority, and clients build theirs from the published
  sections. Nothing in the lab is a second implementation that the relay later replaces.
- **The lab comes first** because it needs no server: no S0, no prediction, no determinism gate.
  Each piece it builds is a piece WD2–WD5 need anyway.
- **The components are panels over sections.** Locally a panel writes the section. In a room the
  same panel sends a request that the world's rules check before the relay writes it (§6's tier 1).

**Headless first** (settled, owner, 2026-11-07). The world runtime needs no rendering. With no GL,
no window and no scene graph it still runs physics and the whole world: the physics world and its
bodies, the entities, their plants and controllers, spawning, the world clock, the sections and
`physics.stats.*`. Rendering is a view onto that world, never its owner. That holds on the relay, in
`ari` and `ari test`, in `pycvc.AriApp` and `Host(mode="headless")`, in `arix`, in a gym worker that
runs many worlds at once, and in wasm under node.

- **Scene realization splits in two.** Today it is all in cvcGL (§1, "Physics with no rendering").
  - **The world half moves to core libcvc.** `scene.physics:` and every node's `body:` become a
    `cvc::physics::world` with its bodies and materials. The shapes those bodies need (primitives,
    meshes for `hull` and `mesh` bodies, height fields) are built from data, not from VTK.
    `generated_world` produces its terrain, buildings, occupancy, roads and water in core, which
    the navigator's rasters and the plants' colliders need anyway.
  - **The view half stays in cvcGL.** It holds the SceneGraph nodes and binds them to the world's
    poses: `physics_scene_binding` reads from the core world instead of owning it.
  - So one document runs headless or in a window, and in a window it is the same world plus a view.
- **The loader runs a scene headless.** A headless host no longer drops `scene:`: its world half
  runs and its draw half is skipped.
  - A token says so. Proposed: `scene.physics`, meaning "the scene's physics and bodies run", which
    holds on every host, headless ones included. `gl.scene` keeps meaning "the scene is drawn".
  - A document that needs the world but no picture requires `scene.physics` and loads headless.
  - A computed `bind:` or a view-only node still needs `gl.scene`.
- **The same world, drawn or not.** The world steps on the world clock, never on frames.
  - A test pins it: one document's state hash after N ticks is the same headless and in
    `arigl --offscreen`.
  - The determinism gate (§5) runs headless.
- **On the relay** the headless runtime is the holon's world: one `cvc::physics::world` holds the
  document's bodies and the avatars, never two worlds side by side (Q9). S0's server-document
  runtime is the runtime this attaches to (ARIADNE_SCRIPTING_HOSTS.md §6.1).

**The four components,** each an `.ari` file a document imports (`cvc://` or `load:`, with
`link:` holes for the prefix and the viewport):

| Component | What it does | Sections | Built on |
|---|---|---|---|
| `world_source` | Picks the world: generated (today's world controls), a data bundle such as Austin, a flat square or a file. Regenerates it, with progress and a ready revision other panels wait on. | `scene.world.*` | `generated_world`; the world runtime turns the world into the navigator's rasters and the plants' colliders, buildings included |
| `vehicle_selector` | A list of the world's vehicle archetypes. The chosen one is drawn alone on a lit, shadowed studio set in its own viewport (a `scenes:` entry). Stats from the archetype's body: size, mass, power, top speed, wheelbase. Settings within the archetype's ranges. The policy: default or a trained network. Identity: name, team, two colours, metadata. A Spawn button. | `vehicles.*` (as built: the catalog, the selection, the spawn keys), `entities.*` | §4's identity, §5's `nav@1`, runtime spawning |
| `spawn_points` | A top-down minimap viewport: click to add a point, click one to remove or tag it (team, archetype). The policy: fixed, random, round-robin or nearest-free, per team or archetype. Points are checked against the world's occupancy and the water's depth for each archetype's medium (land or water); candidates are suggested along the roads, or over the water for a boat. | `spawns.*` | click-to-world picking; the road graph; the grid's water layer |
| `world_physics` | Physics on or off, speed, step and restart. A table of entities: bodies, active, contacts, step time; per vehicle its energy and forces. | `physics.*`, `physics.stats.*` | stats from the world runtime; built-in step and restart |

**Vehicles are archetypes.** The vehicle catalog is the world document's archetypes, beside
`person`: each names its plant (`kinematic`, `jolt`, `dynamic`, `dynamic4w`, `dynamicboat`), its
body (a `vehicle_desc` preset or its fields; a boat preset), its look (a mesh, or the generated
Humvee), and the controllers it allows. libcvc's default worlds ship the built-in vehicles; a site adds its own in
its own world document.

- **As built (L2).** The catalog is `world: vehicles: archetypes:`, a list of its own beside
  `avatars:`, so the holon's avatar (the first avatar archetype) is unchanged
  (`world_rules::vehicles`, [world_rules.h](../../inc/cvc/ariadne/world_rules.h)).
  - Plants (L2b, built 2026-11-14): every plant runs in the world's own `cvc::physics::world`,
    stepped once a step, on the clock. `jolt` is a Jolt vehicle (`nav_plant` with `step_world`
    off). `dynamic` and `dynamic4w` (`dyn_bike_plant`, `dyn_4w_plant`, with `step_world` off) take
    one substep around each of the world's steps (`before_world_step`, `after_world_step`), their
    chassis boxes bodies the world's contacts push back into the models. `kinematic` is nav's
    bicycle with a kinematic box that pushes what it meets. Each plant's entities have a
    navigator of their own on the grid; `physics.stats.vehicles` names each one's plant; cvcGL
    poses a box look's wheels where its plant has them. A spawn may name a plant of its own
    (`entity_spec::plant`), as a gym's agents do (L4).
  - Looks: a humvee is the vehicle-humvee package's `Humvee.glb` when it is installed (found under
    the cvcpkg prefixes, oriented and scaled to the chassis as the nav demos do it), else its
    chassis box in the entity's colours with a cabin and wheels; `look: { model: | file: |
    asset: }` names another mesh.
  - Controllers: `nav@1` (with a policy per entity, by `policy_library`), `none`, or `external`.
  - Intents (L2a, built 2026-11-14): any entity takes a wanted steer and acceleration that
    replaces or blends with its controller's command (`sim_world::override_command`, which takes
    a blend weight on a plant), an external planar force on its drive (`cvc::nav::ext_force`),
    or both. They come from the entity's keys (`entities.<id>.intent.*`, `.force.*`), a second
    table the world block names (`world: vehicles: intents:`, a relay's published namespace, so
    a server document's program steers them), a thread-safe `WorldVehicles::set_intent`, and
    Python (`pycvc.WorldEntities`). An `external` entity is driven by its intent alone, and holds
    still without one. An intent-driven run reaches the same state hash headless and drawn.
  - Poses (built 2026-11-19): an entity with the `pose` controller is placed from outside, by a
    simulation the world does not run, and no plant integrates it. Its chassis is a kinematic box
    in the world's physics world, moved to each pose at the next step, so it keeps its archetype's
    look and label, casts shadows, is picked, pushes what it meets and is a foreign agent the
    navigators steer around; it needs no grid. Its poses come from its keys
    (`entities.<id>.pose.{x,y,z,heading,pitch,roll,speed,steer,t,ref}` or `.matrix`), a packed
    value of many (`entities.poses`, a text record a line, so a relay carries it), a table the
    world block names (`world: vehicles: poses:`, whose entries naming an archetype spawn and
    remove the entities too), and a thread-safe `WorldVehicles::set_pose` / `set_poses` (pycvc
    `WorldEntities.set_pose`, `set_poses` from numpy). Timed poses are drawn between the last
    two, a tick behind; a jump is taken at once. The box look's wheels steer by the pose's steer
    and roll at its speed, and so do a parted look's (built 2026-11-22: `look: { chassis:,
    wheel:, rig: }`, a chassis mesh, one wheel's and a rig, each wheel drawn on its own where
    `WorldVehicles::wheels` has it; ARIADNE.md, "A model in parts"). Every plant's wheels roll
    too. A scripted track is followed exactly, and its state hash is pinned
    headless, drawn and under `arigl snapshot` (`physics.hash`: 1 publishes
    `physics.stats.hash`). A drawn world now spawns `world: vehicles: start:` too, as a headless
    one does.
  - The runtime is `WorldVehicles` ([world_vehicles.h](../../inc/cvc/ariadne/world_vehicles.h)),
    one per `SceneWorld`, with nothing process-wide: a relay that runs a world per room (RM1,
    WD3) runs one per room, and each room's panels name its keys by their root (`world_source`'s
    `world:`, `world_physics`' `physics:`, `vehicle_selector`'s `vehicles:` and `entities:`).
  - Boats and spawns by medium (L3b, built 2026-11-16). `plant: dynamicboat` (or
    `dynamicboat:<preset>`, `launch` or `jonboat`; `vehicle:` overrides `mass` and `max_speed`) is
    a boat on `cvc::physics::dyn_boat_plant`, its hull a body in the world's physics world afloat
    on the world's water: the field a `water` node puts there, or, where the world has none over
    the grid's water (a generated world's sea with no water node), a still field at the water's
    level that `WorldVehicles` adds. The boats' navigator plans on the inverse grid: water at least
    the deepest boat's draft and `keel_clearance` deep, land and shallows blocked.
    - The grid's water layer: `nav::world_grid::water_depth` (the level less the ground at each
      sample, `nav::set_water`) and `solid` (what blocks a sample other than a water surface), and
      `nav::medium_grid(g, medium, depth)`, a car's grid with water deeper than a ford blocked or
      a boat's with only deep enough water free. `SceneWorld::grid` sets the water from a water
      node over the node's ground (its level) or a generated world's bodies of water.
    - The medium: each archetype's comes from its plant (`vehicle_plant::medium()`, `land` but for
      `dyn_boat_plant`'s `water`), with a boat's draft (`dyn_boat_plant::draft`: its keel or its
      propeller, the deepest boat's for every boat, which share a navigator) and half its length
      (`WorldSpawns::set_needs`). A car starts where the water is no deeper than `ford_depth`; a
      boat where it is at least the draft and `keel_clearance` deep and open water reaches half its
      length and `shore_clearance` more round it (`world: spawns:` knobs, 0.3, 0.3 and 0 m).
    - The filter: `WorldSpawns::choose`, `add`, `points()` (`free`, and `medium`: land, water, both
      or none) and the free-point fallback all take each archetype's medium's grid
      (`WorldSpawns::ground_for`). An untagged point serves only the archetypes whose medium it is
      on; a tagged point off its archetype's medium serves nothing. `suggest` follows the tag's
      archetype: roads on land (out of ford-deep water), or `water_spawn_points` over the water deep
      and open enough, spread over each body of water with room. A placed start off its medium (a
      car in a lake, a boat ashore) moves to a free point of its medium, as a blocked start always
      has: a regenerated world can flood a start a document gave.
    - `dyn_boat_plant` checks the ground under a start too (a ray from the hull's top to its
      deepest point), not only the field's box, which knows nothing of the terrain in it.
    - Boats are drawn as hulls (transfix/libcvc#1316: `add_hull_node`, a floating plant's box or
      default look as its hull). Not yet: boats in pycvc.
  - **The vehicles a world starts with.** `world: vehicles: start:` lists them (archetype, count up
    to 256, name, team, controller, plant, policy, `at`/`heading`), spawned when the world's
    vehicles are made. A document's `init:` runs before its world exists, and a spawn request
    already set when the vehicles start is ignored (a reload would spawn again), so this is how a
    document populates its world; `vehicles.spawn.count` spawns several at one request.
  - **On the relay** (the hand-off, WD3's first part). A server document (`arisrv --doc`) runs its
    scene's world headless, as any runtime without a scene adapter does, on the room's steps
    (`ServerDocument`; its physics world is the document's, apart from the holon's). Its vehicles go
    to every client as the room's entities (`conference::room_entities`): who they are in the
    revisioned section `conf.rooms.<room>.entities` (name, archetype, plant, controller, team,
    colours, chassis size, wheels, the look with a file's name alone), where they are on the pose
    stream beside the participants under entity pids (`~e<id>`, at most 30 Hz each, in every
    keyframe). A client plays them out with `entity_playout` (a client holon without physics: the
    participants' peer playout, no bodies) and `room_view` draws each as its vehicle's box look.
    `cvc://worlds/traffic.ari` is the stock server world: twelve blocks and eighteen vehicles.
    Not yet: an area of interest for entities (every viewer gets every one), one physics world for
    the avatars and the vehicles, a model look drawn on the client (it draws the box), the
    controller catalog's prediction (WD4: a client interpolates server-dynamic entities, so none is
    needed to watch them).
  - **Server-simulated entities by state** (a step toward WD3's server-dynamic entities, built
    2026-11-19 with the `pose` controller). A server that simulates its vehicles itself (a server
    document's program, or arisrvx's Python over a simulation of its own) publishes a table of
    poses (`server: publish: [fleet]`): an entry per vehicle naming its archetype and identity,
    and a packed value of their poses each step. A client's own world document names the table
    (`world: vehicles: poses: /fleet`) and its archetypes; its world spawns a `pose` entity per
    entry and places each where the server says, timed poses drawn a tick behind, so stock
    clients (arigl, arigl-web) draw them with the world's looks, labels and shadows, in a world
    that also runs physics of its own. The relay replicates values, not an entry's removal, so a
    server takes an entity out by emptying its entry's archetype. The pose stream (above) stays
    the path for a server document's own world's vehicles in a conference room; this one needs
    no room.
  - Not yet: the default meeting worlds' built-in vehicles (a stock world named by `--scene` is
    published, not run; `traffic.ari` is the server world with vehicles, and `world_lab.ari`
    declares its own); the model from a document's data package (the Austin package carries a
    copy, which `pycvc_gl.cars` reads).

**The gym** is the world runtime driven from Python or headless: `reset(seed)`, `step(actions)`,
`observe()`, a reward the user writes, and an episode log. It absorbs the glue in `pycvc_gl.cars`
(rasters, colliders, plants), which then becomes a thin layer over it. Training runs many worlds
headless; the same document opens in arigl to watch one.

- **As built (L4).** `cvc::ariadne::WorldGym` ([world_gym.h](../../inc/cvc/ariadne/world_gym.h))
  and `pycvc.WorldGym`:
  - **One app per gym.** Each loads its document headless on an app of its own (its state tree,
    clock, physics world and vehicles), so a process runs many; it pauses the clock, so only
    `step()` steps the world.
  - **reset(seed).** The world rebuilt as the document built it, every draw seeded (starts,
    goals, the spawn policy: `WorldVehicles::reseed`, `WorldSpawns::reseed`), and the agents
    spawned (`count` of the first archetype with nav@1, or a list of specs).
  - **step(steer, accel).** One navigator tick. Each agent's command replaces its controller's
    for the tick (`WorldVehicles::command`); NaN leaves it to the controller, so nav@1 is the
    baseline in the same loop. Goals are held (`set_wander(false)`).
  - **observe().** Per agent, as flat arrays: pose, velocity, steering, goal, contacts (counted
    after every physics step, `WorldVehicles::after_step`), out of the world, ray ranges on the
    grid, body and envelope.
  - **The reward and the log.** A reward the user writes, in C++ or Python (the default is
    progress, less collisions, plus arrival); a row per agent per step, to CSV or numpy;
    `<prefix>.gym.*` keys.
  - **The end.** Terminated when every agent with a goal is at it or one leaves the world (off
    its grid, or through its ground); truncated at `max_steps`.
  - **`pycvc_gl.cars`.** `GymCars` gives the template's `CarsState` and `step` over a gym, with
    `world_from_gym` for its rollouts and rasters; `--gym DOC` runs the template on it.
  - **The example.** `examples/ariadne/python/world_gym.ari` and `world_gym.py`: world_lab's
    worlds headless (`scene.physics` alone), arigl draws the same document.
  - **Fixed on the way.** `SceneWorld::restart` gives the vehicles the new ground bodies (a
    generated world's colliders come back with new ids), so a restart no longer counts the
    terrain as an obstacle.
  - **Then (the L4 follow-up).** Every plant in a scene world (`jolt`, `kinematic`, `dynamic`,
    `dynamic4w`; `entity_spec::plant`, `vehicles.spawn.plant`), so `GymCars` runs the cars
    template's plants in a world document (`--vehicle kinematic`, `dynamic:roboracer`); a
    world's buildings are obstacles, not ground (`SceneWorldBody::ground`: the height-field
    colliders alone); and `regenerate` makes a generated world per episode from the reset's seed
    (`SceneWorld::regenerate_world`).
  - **Not yet.** Vehicles do not steer around each other: the world runtime turns on none of the
    navigator's separation, yield or gap keeping, and freezes its sensing, so vehicles meet only
    physically, on one plant or several; the cars template's own worlds (its data, generated and
    flat builders) stay its own, beside the gym's.
- **As built (L4's twin).** What a differentiable twin of the gym's world reads, so a trainer can
  backpropagate through a model of it while the gym (Jolt, the physics worlds) stays the judge, as
  P8 did for the Jolt vehicle:
  - `WorldGym::fields()`: the navigator's grid and signed distance with its gradient, the
    ground's height and grip, and the navigator's settings;
  - `WorldGym::plants()` (`WorldVehicles::plants`): each agent's plant as a `cvc::nav::diff`
    model in SI, with its parameters, limits and state. On the dynamic plants it is the plant's
    own model, with what the model took of the last command in each substep after the driver.
    On `dynamic4w`, away from a collision, the replay is bit for bit
    (`AriadneWorldGym.ThePlantsAreModelsATwinReplays`).
  - GRL-SNAM's `grl_snam.gym_twin` is the torch twin over these.
  - **Not yet:** a twin of the driver (`dyn_driver`), of contact on the four-wheel model and of
    its `grade_force`; per-cell grip; a C++ twin rollout over a gym.
- **The tick (2026-11-15).** A world with vehicles runs on the simulation hosts' clock (0.01 s
  quanta, physics at 100 Hz; WORLD-CLOCK-SYNC-ROADMAP, "One tick"), and its vehicles tick at
  libcvc's one simulation tick, 0.06 s (`cvc::kSimTick`; `WorldVehicles::set_tick`,
  `gym_options::tick`). It was 0.05 s on the scenes' 1/120 s quanta.

**What the runtime needs that isn't there** (§1):

- `sim_world` takes agents at run time: add and remove, with a stable entity id each, on a plant
  too, and with a belief plane of their own if they need one. *(Built, L0's third part.)*
- A policy per agent, as `nav@1`'s params, loaded once per hash. *(Built: `policy_library`.)*
- A name per agent, and the team bound to Python and to documents. *(Built: identity in pycvc and
  under `<prefix>.nav.agents.<id>.*`.)*
- `generated_world` gives the runtime its whole world (buildings, occupancy, road graph, water),
  not only the height field, and the world-to-navigator glue moves from Python to C++. *(Built:
  the colliders, L0's second part; the grid, `cvc::nav::world_grid` and `SceneWorld::grid`, its
  last. The roads, for `spawn_points` (L3), come as the world's road mesh: its asphalt
  items, sampled for points along them.)*
- The document's physics publishes `physics.stats.*`. *(Built, L0's first part.)*
- Built-in handlers for `sim.step` and `sim.restart`. *(Built: `AppRuntime::step_world` /
  `restart_world`, `SceneWorld::restart`.)*
- A viewport click gives a world point (a ray against the scene, or the ground's height field).
  *(Built: `input: pick`, a cell pick against what the view draws, the ground included.)*

## 15. Phases

Each phase leaves the conference working. **WD** = world document. (S, G, W, H and D are the
scripting-hosts, wasm-guest, ariglx-web, holon and DSL-citizen phases in the documents above; G5 is
GENERATED-WORLD-ROADMAP.md's.)

| Phase | What | Depends on | Size |
|---|---|---|---|
| WD0 | **Done** (2026-11-07). The loose ends: `--set` after seeding, or a guard so a `--set` can't suppress the default world; the "under a sun" comment. Built as `--set` after the world and the room's policies, before a server document's `init:` (§1) | — | S |
| WD1 | **Done** (2026-11-07). **The world document seeds the room.** `arisrv --doc world.ari` publishes the document's `scene:` (objects, lights, shadows), in one revision (`write_room_world`). `cvc://worlds/default.ari` and `large.ari` replace the C++ seeds, and `--scene` names them. The light rig and shadows move from the conference document to the world document (Q1), and the conference document drops its `scene:` block. As built: the wire gained a rig's knobs and the shadows' interval; `room_world_from_scene` leaves out, with a warning, what the wire can't hold (other node types and sources, children, textures, shaders), and publishes every body as a static until WD3; `--scene` together with a `--doc` that has a `scene:` is refused; the C++ twins are the fallback | S0 (met by transfix/libcvc#1046) | M |
| WD2 | **Done** (2026-11-08). **The rules come from the document.** The physics, archetype, policy and controller sections (§3), with the activation tick. The holon and `room_view` read them instead of `avatar_body{}`, `kAvatarWidth`, `max_speed = 4.0` and the panel's "walker" literal. As built: the revisioned-section writer in core (`section.h`: `{epoch, at, rev}`, the revision last); the `world:` block (`world_rules.h`), whose stock worlds' twin is `default_world_rules()`; the four sections (`room_rules.h`), lists indexed since `walker@1` is no state name; the holon's rules by tick (`holon::set_physics` and the rest), the veto applied at each step, a late client's re-run from the tick, and a body change re-creating the avatars where they stand; gravity from the scene's `physics:`; `fixed_dt` stays the room's 1/120 s (a document's other value is a warning). Every avatar has the first archetype until WD4 and WD6 let a participant pick. | WD1 | M |
| WD3 | **Started** (2026-11-17: the vehicles). **Entities.** The entities table (§4); server-dynamic props and NPC bodies from the document and from server programs, simulated on the relay by L0's headless runtime; their poses on the pose stream; colliders for meshes. As built so far: a server document's world runs on the relay, and its vehicles reach every client as the room's entities (an identity section and pose-stream entries, played out and drawn: §14, "On the relay"); and (2026-11-19) entities a server simulates itself reach each client's own world by state, as `pose` entities placed from a published table of poses (§14, "Server-simulated entities by state"). | WD2, L0 | L |
| WD4 | **A controller catalog.** Kernels by `name@version`, a determinism gate per kernel, capability negotiation with an interpolation fallback; the vehicle as the second kernel. A participant picks from the archetype's list (§5k phase 3). | WD2 | L |
| WD5 | **Generated worlds** through the world document: G5's descriptor, height-field statics on the relay. | WD1, G5 | M |
| WD6 | **User looks and bodies.** Assets by hash, checked and rehosted by the relay; bodies clamped to the archetype (§5k phase 4). | WD2, S5 | M |
| WD7 | **User controllers as guests**, on the relay and the owner's client, metered and clamped; the threat model first (§5k phase 5, §5i D4). | WD4, G2, G4, S5 | L |
| WD8 | **Persistence.** Snapshot and journal of the world's state; restart restores it; per-principal saved state. | WD3, S5 | M |
| WD9 | **Cells and many holons.** Cell sections and paging, then subdelegation and handoff. | WD8, H3, H4 | L |

Rooms, features, delivery and privacy. **RM** = rooms, **CF** = conference features, **AD** =
asset delivery, **E** = end-to-end encryption.

| Phase | What | Depends on | Size |
|---|---|---|---|
| RM1 | **Many rooms on one relay,** each its own world, sections, holon and clock; the lobby section; bound reads per room. | WD1 | M |
| RM2 | **Rooms made at run time,** by server programs and by participants where the rules allow. | RM1, S0 | S |
| CF1 | **The features sections** and server tokens in `requires:`. | RM1 | S |
| CF2 | **The stock conference components** under `cvc://conference/`; the example documents become layouts over them. | CF1 | M |
| AD1 | **Started** (2026-11-23, OW5k: a pack as a directory with its index; a client finds the relay's document in one by its hash). **Assets by hash** in the manifest, with size and priority; the lookup order (bundle, cache, pack, relay); world packs from a cvcpkg package, an archive at a URL or a directory, each read through its index. | WD1 | M |
| AD2 | **Fetch by need:** colliders first, cells near the client, coarse before fine; stand-ins; the browser's content cache. | AD1, WD9 for cells | M |
| E0 | **Keys:** X25519 identity and prekeys signed by the device key, the key directory on the relay, safety numbers, the libsodium recipe. | — | M |
| E1 | **Encrypted direct messages:** X3DH and the Double Ratchet; `chat.ari` gets txwtf's lock. | E0 | M |
| E2 | **Private rooms:** invitations, admission by a member, sender keys, encrypted chat and membership. | E1, RM2 | M |
| E3 | **SFrame media** in private rooms. | E2 | M |
| E4 | **The private world:** a member's client as the authority over the stock default world; handover when it leaves. | E2, WD2 | L |
| E5 | **Private worlds of one's own:** encrypted assets sealed to members; lockstep as an option; MLS for large rooms. | E4, AD1 | L |

The lab's phases (§14). **L** = local world. They need no relay, so they can start before S0;
each builds what a WD phase needs.

| Phase | What | Feeds | Size |
|---|---|---|---|
| L0 | **Done** (2026-11-09). **The world runtime, local and headless.** The world half of scene realization moves to core libcvc (§14, "Headless first"): the document's physics world and bodies, and `generated_world`'s terrain, buildings and occupancy, with no GL. The `scene.physics` token; a test that a document's state hash after N ticks is the same headless and in `arigl --offscreen`. It reads a document's world sections and runs entities: `sim_world` with agents added and removed at run time; `nav@1` with a policy per entity; identity on entities; the whole generated world handed to the runtime, with its rasters and colliders built in C++; `physics.stats.*`; built-in step and restart; click-to-world picking in a viewport. pycvc bindings for all of it. | WD2, WD3, WD4, WD5 | L |
| L1 | **Built** (2026-11-09). **`world_source` and `world_physics`,** the two panels on pieces that already exist. As built: a node's `world: <name>` names the scene's worlds (generated, data, terrain, flat), and the scene's `world: { source, cell, half }` picks one. `SceneWorld::bind_worlds` keeps the others out of the physics world, and their nodes follow `scene.world.on.<node>`, so they are not drawn. It publishes `scene.world.*`: source, sources, node, kind, ready, rev, status, and the grid of the world on. `physics.enabled` (Q11 as proposed: off pauses the world), plus `physics.stats.step_ms_history` and `.nodes`. Per-vehicle energy and forces wait for L2's archetypes. | WD5 | M |
| L2 | **Built** (2026-11-11). **Vehicle archetypes and `vehicle_selector`,** with the studio preview. As built: `world: vehicles: archetypes:` (plant, preset and overrides, look, ranges, controllers); `WorldVehicles`, the scene world's entities, spawned as Jolt vehicles into its own physics world and driven by `nav@1` (a policy each) or held still (`none`), placed again on a world switch and back at their starts on restart; the `vehicles.*` and `entities.*` keys, and `physics.stats.vehicles` (speed, power, energy, wheel load) in `world_physics`; cvcGL draws each on its chassis (the Humvee model when installed), and `type: vehicle_preview` draws the selected one on a studio scene. No demo until L3 (owner, 2026-11-11). | WD4 | M |
| L2a | **Built** (2026-11-14). **Entity intents.** A document, a script, a server document or a C++ / Python host steers any world entity: a steer and acceleration in place of its controller's command or blended with it, a planar force on its drive, and the `external` controller (its intent alone). Keys `entities.<id>.intent.*` and `.force.*`, read each navigator tick; `world: vehicles: intents:` for a published namespace; `WorldVehicles::set_intent` from any thread; `pycvc.WorldEntities`. | WD4 | S |
| L2b | **Built** (2026-11-14). **Every plant in a scene world.** `kinematic`, `dynamic[:preset]` and `dynamic4w[:preset]` beside `jolt`, all in the scene's one physics world on its clock: the dynamic plants step there (a substep around each world step, `step_world` off) with their chassis against its buildings; the kinematic bicycle's box pushes what it meets. `physics.stats.vehicles` for each, and each look drawn (a dynamic4w wheel by wheel). | WD4 | M |
| L3 | **Built** (2026-11-11). **`spawns.*` and `spawn_points`,** with the minimap and road suggestions. As built: `world: spawns:` (points with a heading, team and archetype tags and a world; a policy, fixed, random, round_robin or nearest_free, kept per team, archetype or all); `WorldSpawns`, one per `WorldVehicles`, keeps the points per world, checks them against the grid, picks a spawn's start by the policy (aside when a vehicle sits on it), and suggests points along a generated world's road mesh; a minimap is a `mode: map`, `input: pick` viewport whose picks add, remove or tag points; cvcGL draws them tagged `spawns`. The demo is `world_lab.ari`, with every panel. | WD3 | M |
| L3b | **Built** (2026-11-16). **Boats as world vehicles, and spawns by medium.** `plant: dynamicboat` (launch, jonboat) on the world's water (a water node's field, else a still one at the water's level), its navigator on the inverse grid; the grid's water layer (`nav::world_grid::water_depth`, `nav::medium_grid`); each archetype's medium from its plant, and `world: spawns:`' `ford_depth`, `keel_clearance` and `shore_clearance`, so cars never start in a lake and boats only in water deep and open enough (§14, "Boats and spawns by medium"). | WD3 | M |
| L4 | **Built** (2026-11-12). **The gym:** reset, step, observe, reward, episode logs, headless; `pycvc_gl.cars` moves onto it. As built: `WorldGym` (C++, and `pycvc.WorldGym`), one app per gym, the clock paused so only `step()` steps it; `reset(seed)` rebuilds the world and spawns the agents with every draw seeded; `step(steer, accel)` is a navigator tick (NaN: nav@1 drives); the observation as flat arrays (pose, goal, contacts per physics step, out of the world, ray ranges, envelope); a reward of the user's; a log per agent per step (CSV, numpy); `GymCars` and `--gym` put `pycvc_gl.cars`' controllers on it; `world_gym.ari` and `world_gym.py` are the example. | — | M |

The one-world phases, **OW0** to **OW7**, are in §18.

**The first cut is WD0 + WD1** for the relay, and **L0** for the lab, which can run beside it. E0
and E1 need neither and can start any time. WD1 needs S0, which transfix/libcvc#1046 builds with
world sections in mind:

- **One write path.** The document writes through holes in its chroot, and the relay's shard
  sends `conf` plus the published namespaces. The world sections under `conf.rooms.<room>.*` are
  already in that scope, so WD1 points the same mechanism at them.
- **Revisions by hand.** A section's fields, then its `rev` last, in one program arrive in that
  order; a test pins it. WD1's revisioned-section writer, in core for L0 to share, is sugar over
  that.
- **The scene is kept.** The document's parsed `scene:` stays on its runtime for the world runtime
  to attach to.
- **`--set`** is still applied before the seeds and the document; WD0 moves it last.

## 16. Open decisions

- **Q1. Where does the look live? Settled (owner, 2026-11-07): the rig lives in the server.** The
  world document declares the lights and shadows, and every client draws the world under them.
  Still open: whether a viewer may lower their cost (shadow resolution, LOD, effects off) or adjust
  for accessibility (brightness, contrast). It never changes geometry or physics.
- **Q2. Do clients ever see the world document?** Proposed: no; only the derived sections (§2).
  S0 builds it that way: the document's own state stays under `ui.docs.arisrv`, which the relay
  never sends.
- **Q3. Which guest format for controllers comes first?** Proposed: native kernels now; wasm guests
  for world- and user-supplied controllers once G2 lands. DSL programs are for rules, not per-tick
  controllers, until their cost at 120 Hz per entity is measured.
- **Q4. How does a simulation change take effect mid-motion?** **Settled 2026-11-08** (Joe: "Yes,
  as proposed"): at a relay-stamped activation tick, with late clients replaying (§3). Built in WD2.
- **Q5. Who hosts user assets?** Proposed: the relay or its CDN, after checking them (§3, §6).
- **Q6. Who owns placed entities and saved state?** Proposed: the S5 principal. Open: what a guest
  without an account keeps, and for how long.
- **Q7. What persists, and for how long?** The state in §8, and its retention.
- **Q8. What scale does the first persistent world need?** Users per room or cell, and the world's
  size. This decides when WD9 is needed.
- **Q9. One simulator or two? Settled (owner, 2026-11-19): one, and the holon is the world**
  (§18). The holon runs avatars in Jolt at 120 Hz; `sim_world` runs nav
  agents at its own tick and reaches Jolt through `plant_host`. Proposed before: the entities table is the
  one registry, `sim_world` becomes `nav@1`'s batch kernel over the entities using it, and both run
  on the world clock. L2 builds the local half of it: `WorldVehicles` is the registry, its
  `sim_world` is `nav@1`'s batch kernel over the entities that use it, and their vehicles are in
  the scene's one physics world, stepped on its clock.
- **Q10. What does a team mean?** Proposed: a label and colours that spawn rules filter on, and the
  yielding `sim_world` already does. Rules and scoring by team come later, as server programs (§7).
- **Q11. What does "physics off" mean?** Proposed: `physics.enabled = false` pauses the world.
  Dropping vehicles to the kinematic plant is a per-archetype choice, not this switch. L1 built it
  that way: off, the steps due are dropped and the world holds still while the clock runs on.
- **Q12. Which crypto library for the end-to-end layer?** Proposed: libsodium, as txwtf, on every
  target (§13).
- **Q13. Sender keys or MLS for room keys?** Proposed: sender keys first; MLS if rooms grow past a
  few dozen members.
- **Q14. Who is a private world's authority?** Proposed: one member's client, with handover;
  lockstep later.
- **Q15. May a private room leak a coarse position for area-of-interest filtering?** Proposed: no
  by default; a room may opt in.
- **Q16. What is a world pack? Settled (owner, 2026-11-07):** a world document and its assets
  with an index, from any source: a cvcpkg package, a generic archive from a URL, or a local data
  directory on disk (§12).
- **Settled, headless (owner, 2026-11-07):** a headless runtime still runs physics and the whole
  world without local rendering. The world half of scene realization is core libcvc; cvcGL adds the
  view (§14). Open within it: the token's name (proposed `scene.physics`).
- **Q17. How do loaded and generated worlds meet? Settled (owner, 2026-11-19):** a generated world
  is a bundle (§18.8): the city bundle's format extended, one dataset path into physics, the
  navigator and the renderer, clients sent the bytes (or generating from the seed and comparing
  hashes), and a save in the terrain lab.
- **Settled with C++ for the core (owner, 2026-11-07):** the world runtime is C++, behind a scene
  node and pycvc; the lab's panels are `.ari` components over it.

## 17. Where this plugs in

- [`ARIADNE_CONFERENCE_ROADMAP.md`](ARIADNE_CONFERENCE_ROADMAP.md): §2 (the server is an `.ari` app),
  §5g (the shared scene), §5h (the holon; H2–H4), §5i (the DSL as a holon citizen; D1–D4), §5k
  (look, body, controller, energy).
- [`ARIADNE_SCRIPTING_HOSTS.md`](ARIADNE_SCRIPTING_HOSTS.md): §6 (`arisrvx`, S0–S7), §6.5.3
  (server-published subtrees), §8 (wasm guests, G1–G5).
- [`GENERATED-WORLD-ROADMAP.md`](GENERATED-WORLD-ROADMAP.md): G5 (the conference's world).
- [`WORLD-GEOMETRY-ROADMAP.md`](WORLD-GEOMETRY-ROADMAP.md): one world representation for rendering,
  physics and navigation.
- [`NAV-VEHICLE-PHYSICS-ROADMAP.md`](NAV-VEHICLE-PHYSICS-ROADMAP.md): the vehicle a second kernel
  comes from.
- [`DISTRIBUTED_STATE_ROADMAP.md`](DISTRIBUTED_STATE_ROADMAP.md): persistence, partitioning,
  subdelegation.
- txwtf's `docs/ENCRYPTED_MESSAGING.md` and `docs/ENCRYPTED_MEDIA.md`: the end-to-end design §13
  follows (X3DH, Double Ratchet, sender keys, sealed content keys).
- [`../CARS.md`](../CARS.md): the multi-car controller template, whose world glue the gym (L4)
  takes over.

## 18. One world: the holon is the world

The owner's decision, 2026-11-19: "We need a single physics world. Everything happens in holons
and should step at 120 hz. The server document should describe the holon scene." Cars avoid people
and avatars, and they are also solid to each other, with real contact: a car pushes, and knocks an
avatar about by the physics. The demos run in the holon, not in a space beside it. Entities are
generic: a user, a navigator, or a trained navigator moves each one, so a vehicle can be a user's
avatar, an avatar can get into a vehicle and drive it, and, where the server allows, an avatar can
simply be a vehicle.

**What this replaces.** Today there are two simulations side by side:

- the holon's avatars, at 120 Hz, in a private Jolt world (`avatar_world`), with the room's
  statics built from the published scene (`room_statics`);
- the server document's world, `SceneWorld` with its `WorldVehicles`, in its own physics world on
  the simulation hosts' clock (0.01 s quanta, physics at 100 Hz; WORLD-CLOCK-SYNC-ROADMAP, "One
  tick").

A car and an avatar never meet: the car's pose reaches clients beside the avatars' on the pose
stream (WD3), and nothing collides them. Q9's proposal (`WorldVehicles` as the entity registry,
`sim_world` as `nav@1`'s batch kernel) stands; what changes is where it runs.

### 18.1 The clock: 120 Hz everywhere

- **One quantum, 1/120 s,** for every host and trainer: holons, scene worlds with or without
  vehicles, the gym, `pycvc_gl.cars` and the trainers. `kSimQuantum` becomes 1/120 s and
  `SceneWorld::use_vehicle_clock` stays only for the real-time catch-up. A host on the clock
  builds its plant world at the quantum (the cars template, the swarm); `world_config`'s own
  0.01 s default stays, where the determinism goldens and the trainers' 0.06 s run.
- **The navigator's tick is 7 quanta, 0.0583 s** (owner, 2026-11-19: "7 quanta"), the one default
  (`kSimTick`), still configurable. 1/120 s divides neither the old 0.06 s tick nor the 0.01 s
  quantum; 7 quanta is the nearest whole tick, 3% shorter than the 0.06 s the GRL-SNAM networks
  were trained at. The tick is held as a count of quanta, so no host ever rounds it.
- **Trained policies are checked at the new tick** before anything depends on them, and
  retrained at it if they lose ground. **Checked (OW1):** on Jolt RoboRacers through the city,
  the same 24 s at 0.06 s and at 7/120 s, 20 seeds of 48 agents, the shipped weights reach
  0.519 and 0.522 (+0.003, SE 0.014) and the untrained default 0.522 and 0.548 (+0.026, SE
  0.011); at 0.05 s, 0.515 and 0.501
  (`NavPlantConvergence.PoliciesHoldTheirReachAtTheSimulationTick`, `CVC_NAV_TICK_AB=1`). No
  retraining. The trainers' own defaults (`training_scene::dt`, the city story's 0.06 s, which
  GRL-SNAM's Python stories mirror) move with GRL-SNAM's hosts.
- **Not changed:** the 50 Hz drift-tooling protocol (`plant_log_scene`, `cvc plant-log`), which
  the plant fits were measured at.
- **120 Hz is the default, not a law:** a world may run at a rate of its own, and the relay's
  clients follow it (§18.9).

### 18.2 The world: the holon runs the document's scene

- **The holon owns the world.** It builds a `SceneWorld` from the server document's `scene:` (or
  the stock world `--scene` names) and steps it on its own clock, never the app's. Two holons in
  one process keep two worlds, as they keep two clocks.
- **Avatars are bodies in that world.** `avatar_world` keeps its controllers (the walker, the
  energy books, the veto) but loses its private `cvc::physics::world`: it drives bodies in the
  scene's world. Creation order stays statics first.
- **The scene's bodies are the statics.** `room_statics` goes: the document's own `body:`s are
  what avatars stand on, meshes, terrain and a generated world's buildings included, the same on
  every side.
- **Vehicles are in the same world.** `WorldVehicles` already puts every plant in the scene's
  world (L2b); now that world is the holon's, at 120 Hz.
- **As built (OW2):** `holon::attach_world(SceneWorld *)` on a server holon with physics, for a
  world on its quantum: the avatars move into the scene's physics world where they stood
  (`avatar_world` over a shared world it does not own: no stepping, statics, gravity or air of its
  own; `release`, `move_to` when the owner replaces its world), and the holon steps the world from
  its own step through `SceneWorld::follow`'s per-step hooks, each avatar driven just before each
  step. `AppRuntime::set_world_external` stops the document's runtime stepping it, and `arisrv`
  attaches a server document's world to the room's holon at start-up. A Humvee driven into a
  standing avatar carries it down the road and ends 0.67 m behind the same run without it
  (`AriadneOneWorld`). Clients still build their statics from the published scene (OW5).
- **Local is the same.** A lab, a gym or a demo with no relay runs a server-role holon in its own
  process: the same world, the same entities, the same code path as the relay. `WorldGym`,
  `HostCars` and the GRL-SNAM hosts move onto it.
- **As built (OW7, first part: a drawn host's world on a worker; the fleets roadmap's D3).**
  `cvc::ariadne::world_worker` attaches a scene's `SceneWorld` to a server-role holon of its own
  (physics on, at the world's rate, `holon_config_at`) and steps it on a thread: the holon's clock
  is lockstep, and the thread runs it one quantum at a time behind the host's clock, tick for
  tick, so the world takes the steps the host's own `follow()` would. The world is deferred
  (`SceneWorld::set_deferred`): a step writes no key, since a state subscriber runs on the thread
  that writes, and the host calls `publish_due()` (its syncs and telemetry) under the lock the
  steps take. The host holds that lock for its frame, and the worker yields to a waiting host
  between steps. A restart (the clock back to tick 0) anchors the world on the holon's clock again
  at once. A world behind its clock by the scene's `clock: { max_lag }` (0.25 s by default, at 1x
  and growing with the speed) holds the host's clock (`SceneClockDriver::hold`). On the frame's
  thread, as every single-threaded host (native or browser) steps it, the same setting is the
  catch-up one frame may run, so a document bounds its world's lag the same way on every host. arigl runs it with `--world-thread` (`AriRuntime::set_world_thread`);
  the frame publishes and draws from the poses the worker captured. Tests:
  `AriadneWorldWorker.StepsTheWorldAsTheHostsFollowWould` (24 kinematic and 4 Jolt vehicles,
  through a restart, the same poses bit for bit as the host's own follow, the per-entity keys
  written on the host's thread) and cvcGL's `test_world_worker_drawn` (bodies and their drawn
  poses as the frame's own follow leaves them). Measured on arigl with 200 kinematic vehicles at
  3.6x, alternating runs: 58.3 ms a frame against 66.4 on the frame's thread, the world at 3.6x
  both ways; the rest of the frame is drawing 200 vehicles (the fleets roadmap's FL2). Not yet:
  avatars and participants in that holon (relay-less arigl has none), the `scenes:` entries'
  worlds (they step on the frame's thread), the gym and the GRL-SNAM hosts on it, and the drawn
  host as the holon's client (OW5's entity playout). The fleets roadmap's entity view (FL2,
  `entity_view.h`) draws a world's vehicles at scale in both holons: a scene's own world, posed
  by the physics binding whether the frame or this worker steps it, and a relay's entities on a
  client, posed where its `entity_playout` shows them (`room_view`).

### 18.3 Contact

- **Every entity is a dynamic body of its own mass:** an avatar's 70 kg box, a car's chassis at
  its preset's mass. A car that drives into an avatar shoves it by Jolt's contact; an avatar that
  walks into a car is stopped by it (and a heavy enough push moves a light car).
- **The avatar stays upright.** The walker holds it upright and pulls its velocity back toward its
  intent within its force limit, so a knock is a shove and a slide, with the walker recovering.
  Tumbling and ragdolls are not in scope.
- **The server is the authority** for every contact. A client predicts its own entity against the
  others (§18.5) and is corrected by the server's poses.

### 18.4 Entities: one table

Every thing that moves is an entity in the holon (§4): avatars, vehicles, boats, props.

| Part | What it is |
|---|---|
| Archetype | The body and look: `person`, a vehicle preset, a boat. The world document's `world:` archetypes, avatars and vehicles in one list. |
| Controller | The kernel that turns an intent into forces: `walker@1`, a vehicle drive per plant (`kinematic`, `jolt`, `dynamic`, `dynamic4w`, `dynamicboat`). |
| Driver | Who supplies the intent: a participant, `nav@1` with the default policy, `nav@1` with a trained policy, a server program or script, or nothing. |
| Owner | The server, or a participant. |
| Identity | Name, team, colours, metadata (§4). |

- **An intent is the controller's.** The walker's is today's `avatar_intent` (a velocity and a
  yaw); a vehicle's is a steer and an acceleration (L2a's). The server's rules clamp either
  (§5).
- **A participant's avatar is the entity it owns and drives.** The participants' pose stream
  becomes the entities' pose stream; a participant pid names its entity.
- **As built so far (OW3, server side):** `holon::entities()` lists every mover (each participant's
  person, the world's entities) with its kind, controller, driver, owner and policy.
  `holon::set_driver(pid, entity)` puts a participant in a world vehicle: its person leaves the
  world (feet and reserve kept), its intent's drive axes (`avatar_intent::steer`, `throttle`,
  `brake`, on the wire as `intent.steer` / `.throttle` / `.brake`; §18.6) replace the vehicle's
  command, and it is where the vehicle is; out, the person stands beside it and the vehicle goes
  back to its controller. The room's entities carry `driver`. Not yet: avatars that are vehicles,
  a person driven by a navigator.
- **Navigators avoid everything they don't drive.** `nav@1` takes every other entity as a foreign
  agent (position, velocity, radius), avatars included, and passes on the right
  (`sim_world::set_foreign_agents`, `WorldVehicles::set_avoid`). **As built (OW4):**
  `WorldVehicles::set_movers`, which a holon running the world fills with its avatars before
  each step; a car on `nav@1` swings 3.1 m past a person standing in its lane and drives on,
  where without avoiding it carries them 34 m down the road (`AriadneOneWorld`).

### 18.5 Clients: the same world, predicted where it is yours

- **The replica builds the same world** from the scene it is sent (§12): the document's bodies,
  not only the statics the wire carries today.
- **Entities it does not drive are kinematic bodies** held where it shows them (the peers'
  playout), so its own entity predicts bumping into them and being pushed. `entity_playout` folds
  into the holon's peer playout.
- **The entity it drives is predicted** with its controller (the walker, or the vehicle's plant)
  and reconciled to the server's poses, as the avatar is today. A client without the kernel
  interpolates its own entity (§5).
- **As built (OW5a):** the room's vehicles are in the client's world. `holon::hold_entity(id,
  size, at)` holds one as a kinematic box of its chassis's size where the client shows it (its
  playout's pose, carried on along its velocity through the frame's steps; a replay holds it
  still), on `avatar_world`'s obstacles; `client_pump(..., entities)` holds every entity the
  `entity_playout` shows except the one the participant drives or is, and releases one the room no
  longer lists. arigl pumps the entities before the holon, so the walk it predicts meets the cars
  it draws: walking into a parked Humvee stops at its side, and a car at 3 m/s pushes the avatar
  along (`HolonPhysicsTest`), where before the prediction walked through and the relay's pose
  snapped it back.
- **As built (OW5b, the kinematic plant):** the vehicle a participant drives (or is) is predicted
  when it runs on the `kinematic` plant, as traffic.ari's trucks and rovers do. That plant is nav's
  bicycle integrated once a navigator tick on the tick's command, its chassis box moved a step at
  a time between where the ticks leave it, and nothing else moves a driven one, so a client that
  knows its numbers runs it step for step: `vehicle_kinematics.h`'s `kinematic_drive` (the
  wheelbase, lock, top speed, the acceleration it is held to, the rear axle, the steps per tick
  and substeps, the envelope) and the clock ticks the navigator ticks on
  (`WorldVehicles::kinematics`; `SceneWorld::follow` passes each step's clock tick to
  `before_step`), carried by the room's entities (`room_entity::drive`). The relay sets a driver's
  command after each step rather than once a frame, from the intent in force at that tick and the
  vehicle as the step left it, so the client's steps match it whatever the relay's frame rate.
  `holon::predict_vehicle` runs the model (`kinematic_vehicle_step`) as the walker is predicted,
  logged by tick, and reconciles to the vehicle's samples: within `reconcile_tol` in plan, nothing;
  else it starts again from the sample (`kinematic_vehicle_at`, which takes the box as moving
  straight through its tick, the pose stream carrying no angular velocity) and replays the inputs
  since. `client_pump` keeps the participant local and predicts the vehicle when its room entity
  carries the numbers, and arigl draws it at the predicted pose (`entity_playout::
  show_predicted`). The model stays within 0.05 mm of the relay's Humvee over 9 s of throttle, a
  turn, the brake and reverse (`AriadneOneWorld`); the holon predicts it with nothing to correct,
  and a 1.5 m push the client cannot know is corrected at the next sample (`HolonVehicleTest`).
- **As built (OW5c, the statics from the document):** the room names the document the relay runs
  (`room_world_source`: `conf.rooms.<room>.world_source.{uri, sha256}`, the URI it was named by and
  the SHA-256 of its bytes; arisrv writes it beside the room scene, none for a C++ twin). A client
  that resolves the same URI to a file with the same hash (a stock world it ships with, the first
  place section 12 looks) builds that document's world as the relay does (`world_replica`:
  `SceneWorld::from_scene`, its vehicles left to the room's entities, its keys under a prefix of its
  own), and its holon's statics are that world's: each static body as the world made it (a mesh's
  triangles, a terrain's field; `avatar_static::kind::body`) and the colliders a node makes at run
  time, as each becomes ready (a generated world's terrain, buildings and edge walls;
  `kind::geometry`, through `add_world_geometry`). Only what is in the world now counts (a scene's
  worlds are on one at a time). `client_pump(..., world)` takes them once the replica is ready,
  and the room scene's plane, boxes and spheres until then or when the document is another; arigl
  passes one and logs what it did. A client on a 1 m dome of terrain stands on it (z 0.99) where
  the room scene's statics put it on the slab below (z 0); on a generated world it stands at
  z 1.23 where the relay's ground is at 1.22 (`AriadneOneWorld`).
- **As built (OW5d, the dynamic plant):** the vehicle a participant drives (or is) on the `dynamic`
  plant is predicted too. That plant is dyn_bike_plant, the dynamic single-track model (tires,
  load transfer, actuator lag, the driver that turns nav's acceleration into a drive force), one
  model substep around each world step; its vehicle is its archetype's preset with five numbers
  (`vehicle_desc_of`), so the room's entities carry those, the navigator's hold on a command (the
  fleet's steering lock, the top speed and the braking a tick's acceleration is held to:
  `sim_world::plant_plan_of`) and the ticks it ticks on (`vehicle_drive.h`'s `dynamic_drive`,
  `WorldVehicles::dynamics`, `room_entity::dynamics`). A client builds the same plant with that
  vehicle alone and no world, one substep a physics step (`vehicle_dynamics.h`'s
  `dynamic_vehicle_model`), and steps it as the relay's world does, the box put where each substep
  starts and carried a step along at the model's velocity; `dyn_bike_plant::save` and `restore`
  keep and put back its state (the model's, the wheels', the driver's, the command), so the holon
  logs it by tick and replays from a sample as for the kinematic plant. The pose stream carries no
  yaw rate, steering angle or driver; a sample takes them from what was predicted for its tick
  (`dynamic_vehicle_model::at`). The client's model stays within 0.007 mm and 0.002 mrad of the relay's Humvee over 9 s of throttle, a turn, the brake and reverse (`AriadneOneWorld`); the holon predicts it with nothing to correct while it turns at up to 0.68 rad/s, and a 1.5 m push is corrected at the next sample, after which it agrees exactly (`HolonVehicleTest`).
- **As built (OW5e, the dynamic4w plant):** the four-wheel plant is predicted the same way.
  dyn_4w_plant is four contact patches, each corner's spring and damper, Ackermann steering and
  the same driver, its heights measured from the ground under its first vehicle's start. The
  room's entities carry the plant and that height with the rest (`dynamic_drive::plant`,
  `dynamic_drive::ground`), and the client's `dynamic_vehicle_model` builds whichever plant the
  vehicle runs on, the four-wheel one alone on flat ground at that height (`dyn_4w_plant::save`,
  `restore`, `box_pose`; a `dynamic_vehicle` holds either plant's state). The box rides at the
  model's height and leans with its pitch and roll, which the client draws. A driver's pose
  carries its heading alone, so a sample takes the suspension (heave, roll, pitch and their rates)
  from what was predicted for its tick, as it takes the yaw rate. The client's model stays within
  0.007 mm and 0.025 mrad of the relay's Humvee on dynamic4w over the same 9 s, its box within
  0.5 mm of the relay's height and 0.7 mrad of its chassis's attitude (`AriadneOneWorld`); the
  holon predicts it with nothing to correct, and corrects a 1.5 m push exactly (`HolonVehicleTest`).
- **Still to come:** an avatar is the relay's correction for a jolt or boat model, and a wall or
  another vehicle for a dynamic plant's model (it touches nothing; OW5o gives the jolt and boat
  models the room's vehicles). A sea the
  relay's storms change (OW5l predicts a boat on its document's sea, a weather node's keyframes
  not followed). For the statics: a document the
  client does not ship with, from a pack it was pointed at (OW5k: a directory), or fetched from
  the relay (section 12).
- **As built (OW5f, the relay's world followed):** the relay publishes what its document has on
  (`room_world_on`: `conf.rooms.<room>.world_on`, `publish_world_on`, written when it changes):
  the scene's world that is on, and the params each generated_world node of it (of the scene,
  without worlds) made its world from, its DEM file as its key names it. A client's replica of the
  document follows it: it turns on the same world (`SceneWorld::set_world`) and makes the same
  generated worlds (`SceneWorld::follow_generated`: the node's keys take the params, a new preset
  first so it does not quick-set the counts over them, and a new world is asked for), and builds
  its statics again from the world on. A client on the relay's slab stands on the dome once the
  relay switches to it (z 0.99), and after a Regenerate with another seed and preset its ground is
  the relay's new ground (z -3.963 where it was 8.803; `AriadneOneWorld`). The params no key holds
  (the cell, the terrain's and the raster's caps) stay the document's, which are the relay's;
  a DEM file the client cannot resolve makes its world from the default generators, as the relay
  would.
- **As built (OW5g, the world's ground under the prediction):** the relay's vehicles drive on the
  world on's ground: its terrain where its node makes one (a generated world's), else its first
  node's navigator grid between samples (`vehicle_ground_height`, now public). A client building the
  same document has both (`SceneWorld::vehicle_ground`: the same grid from the world block's cell
  and half, the same terrain), so its replica hands that ground to the holon
  (`world_replica::ground`, a revision more each time its grid or terrain changes; `client_pump`
  calls `holon::set_vehicle_ground`, which rebuilds each predicted model on it, its state kept),
  and the dynamic plants' models drive on it: the slope's grade in the driver and the model, the
  bike's box on the ground and leaning with it (`dyn_bike_plant::box_pose`), the four wheels on
  the ground under each (a sample at rest settles its suspension there). The four-wheel model
  measures its heights from the relay's datum (`dyn_4w_plant_options::reference`, opt-in, unset
  by default), so its heave is the relay's on any ground. Over a 3 m dome the client's model stays
  within 0.5 mm (bike) and 0.6 mm (four wheels) of the relay's vehicle over 9 s as it climbs
  0.7-0.8 m, where on flat ground it strays 3.2 and 4.6 m (`AriadneOneWorld`). The driver's hold
  on a slope is not in the pose stream: a vehicle first predicted while parked on a slope starts
  from a fresh driver, and the relay's samples correct the difference.
- **As built (OW5h, the relay's surfaces):** a static body's material is the document's
  (`physics: { materials: ... }`, a body's `material:`), and its id is the world's own, so a
  client's statics used to take the default surface. The replica's statics now carry each body's
  material by value (`avatar_static::material`, read from its world), and the client's avatar
  world adds each distinct one once and puts the body on it: an ice rink is ice on the client too
  (friction 0.05, restitution 0.1), a rubber track rubber, a plain slab the default
  (`AriadneOneWorld`). A generated world's items carry their own surface classes already. The
  rain's friction (a weather node's, which moves the surfaces' friction with the wetness) is the
  relay's alone: a client's surfaces stay dry.
- **As built (OW5i, the jolt plant):** the vehicle a participant drives on the `jolt` plant is
  predicted too. That plant is a Jolt vehicle in the relay's world (nav_plant): the engine's
  wheels, suspension, powertrain and driver, the navigator's command given to `world::drive`. The
  room's entities carry it as they carry the dynamic plants (`dynamic_drive::plant` `jolt`,
  `WorldVehicles::dynamics`). A client's `dynamic_vehicle_model` runs it in a Jolt world of its
  own, made as the relay's scene world is (its step, gravity and solver settings), with the
  holon's statics (the replica's, each body on its surface: OW5c, OW5h) and that vehicle alone
  (an `avatar_world` made from a `world_config`). The whole of that world, saved each step
  (`world::save`, whose Jolt state holds the vehicle's wheels, gearbox and driver), is the
  vehicle's state (`jolt_vehicle_saved`): the holon logs it by tick, and a sample restores what was
  predicted for its tick, puts the chassis at the sample's position, heading and horizontal
  velocity (its pitch, roll and spin the predicted ones), and replays from there. New statics
  rebuild the model's world, the vehicle placed again where it is. Found: the gearbox, clutch
  and engine are not in the pose stream, and they matter. The relay's Humvee pulls away about half
  a second after a fresh model would (the navigator's hold before the driver got in is left in its
  powertrain), so running the model open loop from a sample strays metres. With the relay's
  sample every 4 ticks, as the holon takes them, each sample finds the model within 3 mm and
  0.6 mrad of the relay's vehicle on flat ground, and 5 mm and 1.7 mrad climbing the 3 m dome;
  its ride height is within 3-5 mm, and its pitch and roll within 0.02 rad (a sample does not
  carry them; `AriadneOneWorld`). Two models on the same statics agree exactly, and a 1.5 m push
  is corrected at the next sample (`HolonVehicleTest`).
- **As built (OW5j, boats):** the boat a participant drives (`dynamicboat`) is predicted too. The
  relay runs it on `dyn_boat_plant`: its hull a body in the world that the water's field floats and
  drags, its driver turning the navigator's command into a throttle and a rudder every world step.
  The room's entities carry it as they carry the other plants (`WorldVehicles::dynamics`; the boat
  is its preset with its mass and top speed, `boat_desc_of`). A client's `dynamic_vehicle_model`
  runs the same plant in a Jolt world of its own made as for a jolt vehicle, with the holon's
  statics and their water. The water comes with the statics (`world_replica::water`,
  `holon::set_statics`): the replica's world's fluid fields (a water node's), else the still water
  the relay adds for its boats over the grid's lake (`boat_still_water`, which `WorldVehicles`
  now calls too). The boat's state is that world, saved each step, and the driver's memory, which
  the save does not hold (its speed and rudder trims, the acceleration it last measured, the
  command it holds: `dyn_boat_plant::remember` and `resume`). A boat goes on the water at its
  first sample over water deep enough for it (`dyn_boat_plant::attach`); until then, and without
  water, it is played out. With the relay's sample every 4 ticks, each finds the model within
  0.8 mm and 0.08 mrad of the relay's launch on a still lake, afloat within 0.03 mm of its height
  and 5 mrad of its pitch and roll (`AriadneOneWorld`). Two models on the same water agree
  exactly, and a 1.5 m push is corrected at the next sample (`HolonVehicleTest`). A sea's waves
  came with OW5l.
- **As built (OW5l, a sea's waves):** the relay's world reads its water's waves at the clock's
  time: `SceneWorld::follow` sets the world's field time to the clock's at its first call, as
  `world::follow` does, so the step ending on tick T reads the sea at T x fixed_dt however long the
  holon ran before the world was attached. (Before, the field time counted from the world's own
  first step, so a world attached late floated its bodies on other waves than the drawn sea, which
  is drawn at the clock's time.) A client's boat model sets its world's field time to the sample's
  tick x fixed_dt (`dynamic_vehicle_model::at`; the holon passes the server's tick), and each step
  adds fixed_dt, so it reads the same waves at the same tick. On a lake with a 14 m/s wind sea
  that heaves the relay's launch 0.71 m, each sample finds the model within 2.2 mm of it and
  6.4 mm of its height; a model reading the waves half a second off is 171 mm off its height
  (`AriadneOneWorld`). A model starts at rest, level, where the relay's hull may already be
  pitching on a wave (the driver's pose carries its heading alone), so its attitude is up to
  145 mrad off in the first second; the same waves bring it within 23 mrad of the hull's pitch and
  roll, and 1.4 mrad of its heading, from then on. The replica's water is its document's: a sea
  the relay's storms change on the clock's periods (a weather node) is not followed yet.
- **As built (OW5m, a vehicle's attitude in its pose):** the pose stream carries an entity's
  chassis's attitude and vertical speed when its chassis is a body in the relay's world (a jolt
  vehicle's, a boat's hull, a dynamic plant's box), for the room's entities the relay streams
  (`conference::entity_pose_subjects`, which every client draws them by) and for a driver's pose
  alike (`entity_ride`); before, a turn about z alone. So a client draws
  the room's vehicles pitched and rolled as they are, and a jolt or boat model takes its chassis's
  pitch, roll and vertical speed from each sample (`dynamic_vehicle_model::at`), its spin still the
  predicted one (the stream does not carry it). A sample with a heading alone, an older relay's,
  keeps the predicted pitch and roll (`HolonVehicleTest`). The driver's pose is within 4e-8 rad of
  the hull's attitude. On the waves of OW5l the model is within 7.9 mrad of the hull's pitch and
  roll from its first sample (145 mrad before) and 2.6 mrad from the second second, and within
  4.7 mm of its height; the jolt Humvee within 3.7 mrad (17.5 before) and 0.4 mm of its height
  (`AriadneOneWorld`). A model reading the waves half a second off now drifts only 18 mm from the
  hull's height between samples, since each puts it back. The driver's drive axes read the
  pose's heading whatever its tilt (holon's `heading_of`).
- **As built (OW5n, a vehicle's spin and steering in its pose):** the pose stream carries each
  entry's angular velocity (mrad/s) and a vehicle's steering angle (microradians), as differences
  like the rest (`PoseEntry.wx, wy, wz, steer`; `PoseSnapshot.carries`, whose kCarriesMotion bit a
  relay before OW5n leaves 0, so its zeros read as not known rather than not turning). The relay
  fills them from the chassis body (`entity_ride`) and the plant's steering (`entity_info::steer`,
  `holon::steer` for a driver), and a client's sample carries them (`pose_sample::steer`) through
  `holon::apply_authoritative` to `dynamic_vehicle_model::at`: the dynamic plants take the
  steering angle and the yaw rate, dynamic4w the roll and pitch rates too, a jolt or boat model
  its chassis's spin. A dynamic4w model also takes its suspension's pitch, roll and heave from the
  sample (the relay's box's attitude, OW5m; the heave from the box's height, `dyn_4w_plant::
  box_pose` inverted); its heave rate stays the predicted one, since the pose's vertical speed is
  the box's after the relay's step under gravity. A client that starts predicting a vehicle the
  relay is already driving starts it as the relay has it, not level, still and with its wheels
  straight: joining a four-wheel Humvee mid-turn through the pose stream, its model is within
  0.7 mrad of the chassis's pitch and roll and 0.3 mrad of its heading (94 to 98 and 10.5 before;
  24 to 25 and 10.4 with the attitude alone), and within 2 mm of its height in the stream's
  millimetres (`AriadneOneWorld`); a sample places the holon's model with the relay's steering and
  yaw rate (`HolonVehicleTest`). The jolt Humvee is now within 1.1 mrad of its chassis's attitude
  (3.7 before) and the boat on OW5l's waves within 3.6 mm of the hull's height (4.7 before); a
  boat model reading the waves half a second off drifts 8 mm between samples, a second off 15 mm.
- **As built (OW5o, the room's vehicles in the car a client drives):** a jolt or boat model's
  world holds a box for each of the room's vehicles the client shows (`dynamic_vehicle_model::
  hold_obstacles`, from the holon's held entities: `holon::held_obstacles`), so the car the client
  predicts meets them as the relay's meets those vehicles, instead of driving into them between
  samples. Before each step each box is put where its vehicle is to be at the step's end, on from
  where it was shown along its velocity, as the walker's world holds them. A vehicle the relay
  moves as it will (the kinematic plant's box) is a kinematic box; one the relay's world can push
  (a jolt vehicle, a dynamic plant's box) a body of its mass (`room_entity::dynamics`, passed to
  `holon::hold_entity`), put where it is shown before each step, so the car shoves it as the
  relay's does. The boxes are made with the model's world (`jolt_vehicle_world::obstacle_room`:
  one for each vehicle held and two more of each size and mass), since its saved states must keep
  the same bodies; a box no vehicle holds waits out of the way. When more come into view than it
  has room for, the holon makes the model again with room, the vehicle where it is, as a change of
  statics does (`holon::vehicle_rebuilds`). The relay's car at full throttle into a parked
  vehicle: the client's model, sampled every 4 ticks through the pose stream, is within 3.9 mm of
  it at each sample holding the parked vehicle's box, 50 mm without (a kinematic-plant vehicle);
  4 mm and 21 mm for a parked jolt Humvee the car shoves 4 m (`AriadneOneWorld`). The car a holon
  predicts stops at a held vehicle and drives through one it does not hold (`HolonVehicleTest`).
  A vehicle is held where it is shown, its playout's delay behind the relay; a parked one is
  where it is. Avatars are not held yet: they could be bodies of their 70 kg as the relay's are,
  less the walker that pulls one back toward its intent. The dynamic plants' models still meet
  nothing, though the relay's boxes for them are bodies whose collisions go back into their
  plants.

### 18.6 Driving a vehicle, and being one

- **Getting in is a verb.** A participant asks to drive an entity; the server checks the world's
  rule (which archetypes a person may drive, and whether the seat is free), then moves the
  participant's control to the vehicle. The person's body leaves the world while it drives.
- **Getting out** puts the person back at a clear point beside the vehicle, and the vehicle goes
  back to its driver before (a navigator, or nothing).
- **An avatar can be a vehicle** where the world allows it: its avatar archetypes may include
  vehicles, and a participant picks one when it joins (WD4, WD6).
- **As built (OW6c):** an avatar archetype's `vehicle:` names one of the world's vehicle
  archetypes (`world: avatars: archetypes: { person: {...}, rover: { vehicle: rover } }`). A
  participant picks an archetype beside its look, under the same choice subtree
  (`participants.<pid>.avatar.archetype`, beside `avatar.color` and `avatar.accessory`;
  ARIADNE_CONFERENCE_ROADMAP.md §5g, §5k phase 3), with the Avatar window's Body combo or
  `room-set-archetype`; until it picks, it has the first. The relay rules on it
  (`holon::set_archetype`, from `server_pump`), answers in the relay-owned `archetype_status`, and
  for a vehicle spawns it where the person stands and seats the participant in it for good: it
  drives it with its vehicle scheme, cannot get out or into another, and the vehicle goes when it
  leaves or picks a person again (it stands beside it then). The room's entities carry `avatar`,
  so a client offers no one else that vehicle and shows its owner no get-out prompt. A person's
  body is still the first person archetype's for everyone: bodies that differ by archetype are
  still to come. `traffic.ari` offers a rover body.
- **Mouse look is first person** (OW6c): walking under mouse look, arigl's camera sits at the
  walker's eyes (0.92 of its body's height) and its own avatar is hidden; W walks where you look.
  The Controls window's "Mouse look in first person" (the camera's `settings.first_person`, on by
  default) turns it off for the view over your shoulder. Driving keeps the chase camera.
- **Each thing driven has its own controls** (`cvc/ariadne/controls.h`). A control scheme per
  kind, each with its own bindings (keys and mouse buttons, and a gamepad's buttons, triggers and
  stick halves), so a car's keys are not a person's:

  | Scheme | Actions (default keys) | Gamepad |
  |---|---|---|
  | walk | forward W/Up, backward S/Down, strafe left A/Left, strafe right D/Right, jump Space, mouse look on/off V, get in E (held 0.4 s) | walk on the left stick (as fast as it is pushed), look around on the right, jump A, mouse look on/off the right stick pressed, get in X (held) |
  | car | throttle W/Up, reverse S/Down, brake Space, steer left A/Left, steer right D/Right (each ramps to the lock over 0.25 s), get out E (held) | steer on the left stick, throttle RT, brake LT, reverse LB, look around on the right stick, get out X (held) |
  | boat | the car's, with no brake | the car's, with reverse on LT |

  Getting in switches a participant to its vehicle's scheme (the archetype's `controls:`, car or
  boat, by its plant when it names none), getting out back to walking. arigl keeps the bindings
  where the Controls window changes them: walking's at `<viewer>.avatar.keys.<action>`, the
  vehicles' at `<viewer>.controls.<scheme>.<action>`, a gamepad's at `<viewer>.pad.<scheme>.<action>`.
  The Controls window lists each action's gamepad input beside its keys, with a Change of its own:
  press the new button, pull the trigger or push the stick (`KeyBinding::pad_path`,
  `Runtime::post_pad_press`; Escape keeps the old one).
- **As built (OW6b):** a binding's `pad` names a gamepad's inputs (buttons A, B, X, Y by position,
  LB, RB, the D-pad, the sticks pressed; triggers LT, RT; stick halves `LeftY-`, `LeftX+`, ...);
  each has a deadzone (0.15 of a stick, 0.05 of a trigger), past which the rest of its travel is
  0 to 1, and an action is its keys' value or its gamepad's, the larger (a key's steering ramps, a
  stick's does not). cvcGL's `SdlGamepad` reads the first gamepad SDL finds (one plugged in later
  within a second). Played on the gamepad, arigl faces where the camera looks, as under mouse
  look; the right stick turns the camera (150 degrees a second at a full push, the chase camera
  waiting while it does), and the get-in prompt names the gamepad's button.
- **The drive axes are shares of the vehicle's envelope,** so one scheme drives any vehicle and
  the server holds each to its own: `steer` in [-1, 1] of its lock (`max_steer`), `throttle` in
  [-1, 1] of its `max_accel` (negative is reverse), `brake` in [0, 1] of its `max_decel`
  (`holon::drive_command`). The brake stops the vehicle and never backs it up. Reverse brakes a
  car still rolling forward (faster than 0.5 m/s), then backs it up; a boat has no brake, and its
  reverse is the propeller astern.
- **The walker jumps** (`avatar_intent::jump`, on the wire `intent.jump`) while it is held and
  something is under its feet, at its controller's `jump_speed` (4 m/s by default: about 0.8 m);
  the reserve pays the jump's kinetic energy.
- **As built (OW6a):** the ask is a room leaf, `participants.<pid>.drive = "<seq>:<entity>"` (0
  gets out; `request_drive`, the `room-drive` verb). The relay's `server_pump` answers each ask
  once, in `drive_status` (`"<seq> ok"` or `"<seq> refused: <why>"`), and says what the
  participant drives in `driving`; both are the relay's (`room_write_gate`). The rule
  (`holon::get_in`): the vehicle's archetype is `drivable: true`, and the person stands within its
  `reach:` (3 m by default) of its chassis box. The room's entities carry `drivable`, `reach` and
  `controls`, so a client prompts for the nearest drivable vehicle in reach (`drive_target`, the
  one ahead first): "E  Drive humvee", a ring filling as E is held. While the relay says a client
  drives, its own pose (the vehicle's) is played out from the relay's samples, not predicted
  (OW5 predicts it), its marker is hidden, and arigl's camera swings round behind the moving
  vehicle.

### 18.7 Phases

| Phase | What | Size |
|---|---|---|
| OW0 | This section. | S |
| OW1 | **Built** (2026-11-19). **The clock.** 1/120 s quanta everywhere, the navigator's tick 7 quanta; the trained policies checked at it (§18.1: they hold). | M |
| OW2 | **Built** (2026-11-19, server side). **The holon runs the document's world.** `SceneWorld` on the holon's clock; avatars as bodies in it; `room_statics` replaced by the scene's bodies; vehicles beside the avatars, colliding. A test: a car shoves an avatar, an avatar stops a car. | L |
| OW3 | **Started** (2026-11-19: the table and drivers, server side). **Generic entities.** One table; controller, driver and owner per entity; controller-shaped intents; participants own entities; one pose stream. | L |
| OW4 | **Built** (2026-11-19). **Navigators avoid avatars.** Every entity a navigator doesn't drive is a foreign agent to it. | S |
| OW5 | **Started** (2026-11-21, OW5a: the room's vehicles as kinematic boxes in the client's world; OW5b: the vehicle it drives predicted, on the kinematic plant; OW5c: the statics from the world document; OW5d: the dynamic plant predicted; OW5e: the dynamic4w plant predicted; OW5f: the relay's world switch and generated worlds followed; OW5g: the prediction on the world's ground; OW5h: the relay's surfaces; OW5i: the jolt plant predicted; OW5j: boats predicted; OW5k: the relay's document from a world pack; OW5l: a sea's waves under a predicted boat; OW5m: a vehicle's attitude in its pose; OW5n: a vehicle's spin and steering in its pose; OW5o: the room's vehicles in the car a client drives). **Clients build the same world,** others kinematic at their playout, their own entity predicted by its controller. | L |
| OW6 | **Started** (2026-11-20, OW6a: keyboard and mouse; OW6b: gamepads; OW6c: avatars that are vehicles). **Getting in and out of vehicles,** and avatars that are vehicles. OW6a: control schemes per kind (walk, car, boat), the drive axes as shares, brake and reverse apart, the jump, the ask and the rule, the client's prompt and chase camera. OW6b: gamepads (sticks, triggers, buttons per scheme), and their bindings in the Controls window. OW6c: an avatar archetype may be a vehicle, picked beside the look; a first-person walk under mouse look. | M |
| OW7 | **Started** (2026-11-22, first part: a drawn host's world in a local server-role holon on a thread of its own, `world_worker`, arigl `--world-thread`; §18.2). **The demos in the holon:** grl-snam and grl-snam-dbg on a local server-role holon, then `arisrv` hosting the traffic world with clients walking among the cars. | M |
| OW8 | **Built** (2026-11-21, first part). **A world's rate:** the relay chooses it (a scene's `clock: hz`, `arisrv --sim-hz`) and publishes it, and the clients build their holons at it (§18.9). The policies hold at 60 Hz's 4/60 s tick. Not yet: a change while the relay runs. | S |

**Open:**

- The dynamic plants are libcvc's own C++, not Jolt, so their cross-platform determinism is not
  pinned; a client's prediction of a vehicle may drift and be corrected. A gate per kernel (§5)
  says how much.
- The cost of 120 Hz with many vehicles, measured on the relay before the stock worlds grow. A
  world that does not need 120 Hz can run at 60 or less (§18.9).
- An area of interest for entities: today every client gets every one.

### 18.8 World contents: a generated world is a bundle

The owner's decisions, 2026-11-19: a generated world is made as if it had been saved and loaded as
a bundle, so a loaded city (Austin) and a generated world (the terrain lab) are one kind of world
with one path into physics, the navigator and the renderer. The city bundle's format is extended,
not replaced. Clients get a world's bytes, and may instead generate it from its seed and compare
hashes. The terrain lab can save its world, in the browser too, also for import into other
software. Nav materials, water and the wall colliders are in the bundle.

**What it replaces.** Today the two kinds of world are different scene nodes:

- a city is a data node: `data-load ... city_bundle` decodes `terrain.json` and `buildings.glb`
  into a dataset (`world` as `cvc::world::world_geometry`, `terrain`, `buildings`, `bounds`,
  `satellite`), the node is `source: { data: city }`, and `SceneWorld` reads its body through the
  dataset;
- a generated world is a `generated_world` node: `GeneratedWorldRuntime` makes the world and its
  own colliders, a `water` node follows it through `world:`, and cvcGL draws it with its own
  realize (`world_nodes.cpp`), not the data node's.

Both reach Jolt as a `world_geometry` through `cvc::physics::add_world_geometry`, and
`cvc::nav::world_grid` takes either. What is doubled is the node kind, the collider feeder, the
renderer and the contents: a generated world also has roads, props, a painted ground, water and
nav surfaces, which a city bundle cannot carry.

**The bundle, version 2.** A version 1 bundle (`terrain.json`, `buildings.glb`, optional
`buildings_flat.glb` and `satellite.png`) still loads as it does. Version 2 adds a manifest and
files beside them:

| File | What |
|---|---|
| `world.json` | The manifest: `format: cvc.city_bundle`, `version: 2`; the surface table by name (ids in the files are indices into it); every item (name, file, roles, surface, placement); `edge_walls`; an optional `region` the world is cut to; the water (`level`); the generator that made it (name, version, params, seed); and the content hash. |
| `terrain.json` | As version 1. A large world's grid may be `grid_file: terrain.f32` instead of `grid` (little-endian float32, rows x cols, north first, as `grid`). |
| `terrain_surfaces.u16` | One surface per terrain cell (little-endian uint16, north first): the nav and contact materials. |
| `buildings.glb` | As version 1: world metres, z up, at terrain height. |
| `<item>.glb` | Every other mesh item (roads, props, foliage, needles, the props' coarser rungs `props_r1` and `props_r2`), one primitive per surface, the material named by the surface, per-vertex colour where the item has it. |
| `props.json` | The props' render cells: the cell size and, per cell, its bounds and its run of triangles (lines, for the needles) in each prop mesh, so a renderer draws a cell as a ladder of rungs. A cell's points are the ones its faces use. |
| `streets.json` | The street network the roads were extruded from (nodes, and segments with their widths and classes), and the city's counts (buildings, districts, road length). |
| `ground.png` | The painted ground texture (the material colours, 2048 a side, north at the top), where a city has `satellite.png`. |

- **Nav materials** are the terrain's cell surfaces and each item's face surfaces, read through
  the one surface table, so the renderer, the physics and the navigator read the same materials
  (`world_grid` keeps them instead of dropping them).
- **The water** is the level in the manifest. The loader builds the bodies of water and the
  water's field from the stored terrain at that level, as `generated_world` does now; the
  navigator's `set_water` and a scene `water` node read the same level, so there is one water
  knob, and deep water is a body to a car, not ground at mu 0.10.
- **The walls** are the manifest's `edge_walls` at the world's edge, which is the `region` when
  there is one: the grid, the colliders and the edge walls follow the same cut, made once.
- **Derived things are not stored:** the terrain's LOD chunks, the water's shore grid and the
  ground under the water are rebuilt from the stored terrain on load.
- **The hash** is SHA-256 over every file but `world.json`, in name order (each name and its
  bytes' SHA-256), so two worlds with the same contents hash the same whatever wrote them.
- **The meshes are written through assimp's glTF 2 exporter** (glb, to memory, so the browser
  writes a bundle as natively does), filling the model handler's `write`, which throws today: one
  primitive per surface, per-vertex colour, triangles and lines. Reading `buildings.glb` already
  goes through assimp, and the published wasm builds have it. The files open in other software
  (Blender, for one).

**As if loaded.** glb positions are 32-bit floats and a mesh's vertices come back in the order
its faces first use them, so the world physics runs is the bundle read back, not the generator's
doubles: written, read and written again, a world gives the same bytes and the same hash.

**One path in.** A generated world is a dataset of the same shape as a decoded bundle:
`data-generate` beside `data-load` fills `world`, `terrain`, `buildings`, `bounds`, the ground
texture and the rest, and the node is `source: { data: world }`. Regenerate bumps the dataset's
revision, which `SceneWorld` already follows for a data node. `generated_world` the node kind
becomes shorthand for a `data-generate` and its data node, so documents that use it keep working.
`SceneWorld`'s public surface (`world`, `follow`, `restart`, `generated_world(id)`, `grid`,
`vehicles`) does not change; `generated_world(id)` reads the dataset.

**One renderer.** cvcGL's generated-world renderer (terrain LOD, the props' cells and rungs, roads,
the painted ground, cloud shade) draws any world dataset, a city's included. A `source: { data }`
node whose dataset is a world makes the drawn world from it with `world_from_bundle` off the frame
thread and puts it in a piece at a time, under the node (world_nodes.h's DataWorldView); a reload
replaces the shown world once the new one is in. The ground is the bundle's `ground.png`, else a
city's `satellite.png` at its own width and height. The data node's plain draw (each item a mesh,
the aerial image draped) is left for a terrain that takes a live texture
(`material: { texture: { stream } }`) and for datasets that are not worlds.

**Getting it to a client.** The server sends the bundle's bytes, content-addressed and cached
(§12, AD1), and a client builds the same world from them, with no generator determinism to pin
across platforms. A client that has the generator named in the manifest may instead generate the
world from its params and seed, write it as a bundle in memory, and compare the hash with the
server's; on a mismatch it takes the bytes.

**Saving.** The terrain lab saves its world as a bundle: natively a directory or a zip, in the
browser a zip handed to the browser as a download. A save for other software adds the terrain as
a mesh (`terrain.glb`) and a 16-bit height map PNG.

| Phase | What | Size |
|---|---|---|
| GB1 | **The bundle, version 2:** the manifest, glb export through assimp, `write_city_bundle` from a `world_geometry` and a generated world's extras, `load_city_bundle` reading version 2 (items, cell surfaces, water level, region, edge walls, hash). Tests: a world written and read back has the same terrain, cell surfaces, items and face surfaces, and writes the same bytes again; two loads simulate to the same state hash. | M |
| GB2 | **One path in:** `data-generate`; a generated world as a dataset; `SceneWorld` reads only data nodes; `generated_world` as shorthand; the region and the water level on the data node. | L |
| GB3 | **One renderer** for any world dataset. Part 1: everything the renderer draws is in the bundle (the props' rungs and cells, `streets.json`, `ground.png`), and `world_from_bundle` makes the drawn world from any bundle, version 1 included, rebuilding the terrain chunks and the water. Part 2: a data node whose dataset is a world (Austin, a saved or made world) draws with the generated-world renderer: the terrain's LOD chunks on the bundle's ground (a non-square aerial image included), the props' cells, the buildings, roads and water, made in a sliced job and put in a piece at a time. | L |
| GB4 | **Save in the terrain lab:** a directory or zip natively, a zip download in the browser, the terrain mesh and height map for other software. | M |
| GB5 | **Clients:** a world's bytes by content address, cached; a seed and a hash compare as the shortcut. After OW2 and with WD5, AD1. | M |

### 18.9 A world's rate: the server chooses, the clients follow

The owner, 2026-11-21: "some worlds might not need 120hz and can do 60hz or lower if bandwidth and
compute are an issue." So a world's rate is a knob: the relay picks it, and its clients follow.

- **The knob.** A scene's `clock: { hz: <n> }`, a whole number of ticks a second from 10 to 240
  (`cvc::sim_rate_valid`; 0 or absent is 120). `arisrv --sim-hz <n>` overrides the document's.
  Everything that ticks takes it from there:
  - **the quantum,** 1/hz: the app's world clock (`SceneWorld::use_vehicle_clock` puts it there
    before the physics world is made), a physics step, the holon's clock;
  - **the navigator's tick,** the whole quanta nearest the 0.06 s the policies were trained at
    (`cvc::sim_tick_quanta`): 7 at 120 Hz (`kSimTick`, as before), 6 at 100, 4 at 60 (0.0667 s), 2
    at 30, one at 20 and below. A tick asked for (`WorldVehicles::set_tick`) still wins;
  - **the holon's windows,** which are counted in ticks and were sized for 120 Hz: the skew, the
    lead slack, the peers' playout spans, the intent lead, the catch-up. `holon_config_at` keeps
    each one's time at the new rate (at 60 Hz each is half the ticks), at least one tick.
- **The relay.** `arisrv` takes `--sim-hz`, else the document's (or the stock world's) `clock: hz`,
  else 120. It builds its holon at that rate and the server document's world on it (a load hook
  sets the scene's `hz`, so `attach_world` finds the world on the holon's quantum), logs
  `[arisrv] clock: 60 Hz (...)`, and publishes the rate.
- **The wire.** `conf.rooms.<room>.clock.hz`, a revisioned section (§3) the relay writes once at
  start-up with its epoch (`room_rules.h`: `write_room_clock`, `read_room_clock`). A value that is
  no rate reads as none, and a room with none (a relay from before this) runs at 120 Hz. A client
  never writes it (`room_write_gate`).
- **The clients.** A host builds its holon from `room_holon_config(app, room)`, at the room's rate.
  A holon cannot change its quantum in place (every tick it counts would mean another time), so a
  host whose holon is off the room's clock (`on_room_clock`: it was built before the section came,
  or the relay restarted at another rate) builds it again; the roster puts everyone back at the
  next pump and the stream's next entries bring their poses. `arigl` does this each frame
  (`follow_room_clock`), keeping its lead in seconds.
- **What it saves.** The relay's CPU, mostly: physics, the navigators and the holon's integration
  run per tick. Bandwidth much less, since poses already go out at 60, 30 or 20 Hz by each
  viewer's round trip (room_pose_relay) and the room steps at 60 Hz whatever the rate; a near
  entity's pose is capped at 30 Hz at any rate. Video has its own knobs (`--video-max`,
  `--viewer-kbps-max`, `--aoi`, `--aoi-video-max`).
- **The policies hold at 4/60 s** (2026-11-21), which is the tick at 60 Hz and at 30 Hz. That tick
  is 11% longer than the 0.06 s they were trained at; 7/120 s was 3% shorter. The run is the
  same as OW1's (§18.1): Jolt RoboRacers through the city for 24 s, 20 seeds of 48 agents
  (`NavPlantConvergence.PoliciesHoldTheirReachAtTheSimulationTick`, `CVC_NAV_TICK_AB=1`, which now
  has the 4/60 s arm). Reach at 0.06 s, then at 4/60 s:
  - the shipped weights: 0.519, then 0.512;
  - the untrained default: 0.522, then 0.517.

  Both are within the seeds' noise. No retraining.
- **Not yet:**
  - A rate change while a relay runs. It is a restart: the clients follow the new run's section.
    In place it would be a world restart under a new epoch (§6, #1076) that every holon rebuilds
    on.
  - The scene swarm (`scene_swarm`) runs its own plant host and tick at 1/120 s and needs the
    120 Hz clock: a scene with a swarm and a rate of its own is not supported yet.
