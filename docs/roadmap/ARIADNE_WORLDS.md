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
| Wire format | Scalar leaves under `conf.rooms.<room>.scene.{rev, env.*, objects.<id>.*}`: a subset of the document's `Scene` / `SceneNode` / `SceneLight`. Meshes go by `source.uri`. Removal is soft (`present = 0`); `rev` is written last. | `inc/cvc/ariadne/conference/room_scene.h` |
| Look | The world's (Q1, WD1). The world document declares the light rig and the shadows, and the relay publishes them through `scene.env.lights.*` and `scene.env.shadows.*`, which now carry a rig's tuning knobs (stage, key, fill, back, warmth, environment, ambient, wash) and the shadows' re-bake interval. The conference document has no `scene:` block. | `default_room_rig`, `write_scene_env` in `room_scene.cpp`; `src/cvc/ariadne/worlds/default.ari` |
| Who writes the scene | Only the relay. A client may write only its own participant subtree (bound writes, default on). | `room_write_gate.h` |
| Client realization | `room_view` rebuilds the scene from the tree whenever `scene.rev` moves, with the same realizer a document's `scene:` uses. | `src/cvcGL/ariadne/room_view.cpp` (`sync_scene`) |
| Physics statics | Built from the scene objects: a plane becomes a 0.2 m slab, and boxes and spheres become themselves. Meshes are skipped, so the bunnies don't collide. | `room_statics` in `src/cvc/ariadne/holon_sync.cpp` |
| Avatar looks | A catalog the relay seeds: ten colours plus an automatic hue, and seven accessories. A participant picks from it; every client draws the pick the same way. | `default_avatar_catalog`, `seed_avatar_catalog` in `room_avatar.cpp` |
| Body | Compiled in: `avatar_body{70 kg, 0.5 m, 1.7 m}`, duplicated as `kAvatarWidth` / `kAvatarHeight` for the accessory envelope. Not replicated. | `inc/cvc/ariadne/avatar_physics.h`; `room_avatar.h` |
| Controller | Compiled in: the walker is "the only controller so far". The avatar panel's "walker" is a string literal. | `drive_walker` in `avatar_physics.cpp`; `avatar_panel.cpp` |
| Energy policy | **Replicated.** The relay writes every field under `conf.rooms.<room>.avatars.policy.energy.*`, then bumps `policy.rev`, each time it starts. The relay's holon and every client's read it from the same leaves and predict under it. Only on/off and rest/stop are exposed as flags; the numbers are C++ defaults. | `write_avatar_policy` in `room_avatar.cpp`; `sync_energy_policy` in `holon_sync.cpp` |
| Speed veto | Compiled in: `room_holon_config()` sets `max_speed = 4.0`. | `holon_sync.cpp` |
| Holon | One world clock at 120 Hz. The relay simulates every avatar in Jolt. A client simulates its own avatar too, predicts it, and snaps and replays past 5 cm. Peers are kinematic, played from a buffer. Intents go up; pose snapshots come down at 60, 30 or 20 Hz depending on round-trip time. | `inc/cvc/ariadne/holon.h`, `holon_sync.h`, `room_pose_relay.cpp`, `inc/cvc/core/world_clock.h` |
| Determinism | Jolt runs in cross-platform determinism mode. A golden state hash pins it on Windows, Linux, macOS, x86_64 and arm64, and wasm-mt (its own CI job). | `physics_world_test.cpp` (`exact_scene_hash`), `physics_parity_test.cpp`; ci.yml "physics replay (wasm-mt, node)" |
| Documents on the server | `arisrv --doc server.ari` (S0, transfix/libcvc#1046) runs one headless: DSL programs on the relay's loop once per room step, the holon's world clock at `sim.tick`, the room read-only at `room`, `server: publish:` namespaces sent to every client, and `requires: [relay]`. A `scene:` with nodes or lights is the room's world, which the relay publishes before `init:` runs (WD1). Its physics and bodies are kept on `ServerDocument::scene()` but not simulated yet, and the load warns so: §14's headless world runtime will simulate them as the holon's one physics world. | ARIADNE_SCRIPTING_HOSTS.md §6.1, §6.5.3; `inc/cvc/ariadne/server_document.h` |
| What a document can already say | Far more than the room tree carries: `scene:` with primitives, meshes with `fit:`, heightfields, light rigs and shadows, and the custom node types `generated_world` and `cloud_sky`. `scene.physics:` gives gravity, `fixed_dt`, materials and per-node bodies (dynamic, kinematic or static; box, sphere, hull, mesh or terrain). All of it parses in core libcvc, with no GL. | `docs/ARIADNE.md` (Scene, Physics); `src/cvc/ariadne/loader.cpp` |
| Persistence | None. The room lives in memory, and a restart re-seeds it. Only the device-key pins can go to a file. `distributed_state_config::blob_store_path` is not read. | `src/arisrv/main.cpp` (`--device-pins`); DISTRIBUTED_STATE_ROADMAP.md |
| Scale pieces | Built but unused by the conference: hash partitioning (`state_hash_partition`, `state_cluster_shard::partition()`) and delegation leases (`state_delegation_manager`). | `inc/cvc/state/` |
| Vehicles and their navigator | `cvc::nav::sim_world` runs N agents on an occupancy raster with one `coef_mlp` policy for all of them. N is fixed when it is built: agents can't be added or removed. Agents have colours and, in C++ only, a team (`set_team`, used for yielding); they have no name. | `inc/cvc/nav/sim_world.h`; `src/cvc/nav/sim_world_binding.cpp` |
| Vehicle plants | `plant_host` puts each agent on a plant: nav's kinematic bicycle, a Jolt vehicle, the dynamic single track or the dynamic four-wheel model. Presets `vehicle_desc::humvee()` and `roboracer()` carry mass, wheelbase, track, engine and `max_speed`. | `inc/cvc/physics/plant_host.h`, `inc/cvc/physics/vehicle.h` |
| Generated worlds in a document | The `generated_world` node builds terrain, roads, buildings, trees and water, but publishes only its height field (`RealizedScene::height_fields`), so `ground:` and bodies stand on it. Buildings, occupancy, the road graph and water stay inside the node. | `src/cvcGL/ariadne/world_nodes.cpp` |
| World to navigator | Only in Python: the `pycvc_gl.cars` template rasterizes a city or generated world (`nav_world_rasterize`) into the navigator's planes and hands colliders to the plants. | `bindings/pycvc/pymod_gl/cars.py`; `docs/CARS.md` |
| Physics with no rendering | None for a document. Its `scene.physics:` world and its nodes' `body:` are built only by cvcGL's scene realizer (`realize_physics`), which binds bodies to SceneGraph nodes through `physics_scene_binding`; `generated_world` is a cvcGL node too. A headless runtime (`pycvc.AriApp` and `Host(mode="headless")`, `ari`, `ari test`, arisrv) has no scene: it drops `scene:` with a warning, and a document that requires `gl.scene` doesn't load. The physics world itself (`cvc::physics::world`, Jolt), the plants and the relay's holon are core libcvc and need no GL. | `src/cvcGL/ariadne/scene_realize.cpp`; `inc/cvc/physics/world.h`; `docs/ARIADNE.md` (the headless mode) |
| Physics stats | `physics::world_binding` writes `tick`, `step_ms`, `bodies`, `active_bodies`, `contact_events` and a snapshot to its prefix. A document's `scene.physics:` world is held directly by the realized scene, not through the binding, so it publishes none of them. | `inc/cvc/physics/world_binding.h`; `inc/cvc/gl/ariadne/scene_realize.h` |
| Step and restart | `sim_transport.ari`'s Step and Restart buttons raise `sim.step` and `sim.restart` for the host to handle. The clock binding can queue steps (`request_steps`); nothing restarts a world. | `src/cvc/ariadne/components/sim_transport.ari`; `inc/cvc/core/world_clock_binding.h` |
| Rooms | One room per relay: `arisrv --room <id>` (default `standup`). The tree is already shaped for more, `conf.rooms.<room>.*`, and the area-of-interest, avatar and scene paths all take a room id. | `src/arisrv/main.cpp`; `inc/cvc/ariadne/conference/` |
| Conference in documents | The stock components include `room_users.ari` (the Users window) and `chat.ari`. The rest of a conference (joining, media controls, tiles, the avatar panel, the 3D room) is in the example documents `conference_roster.ari`, `conference_media.ari`, `conference_live.ari` and in C++ widgets (`room_view`, `stream_view`). | `src/cvc/ariadne/components/`; `examples/ariadne/documents/` |
| Feature detection | A document's `requires:` names host features (`gl.scene`, `net`, `media.player`, ...) and the host says whether it has them. Nothing asks the *server* what it offers. | `src/cvc/ariadne/requires.cpp` |
| Asset cache | A document's `data:` block fetches cvcpkg archives checked against the registry's sha256 and keeps them in a local cache by hash (`CVC_DATA_CACHE`, else the user's cache directory). A browser has no cache: downloads are not kept. | `inc/cvc/ariadne/data.h` |
| Keys and encryption | Each device has an Ed25519 key (a 32-byte seed, OpenSSL) and the relay pins it. Links to the relay use TLS, so the relay reads everything it forwards. SFrame end-to-end media is planned (§6g of the conference roadmap); no end-to-end encryption is built. | `src/cvc/state/state_device_key.cpp`; `inc/cvc/ariadne/stream/stream_relay.h` |
| Views | A document's `viewport` widgets draw the scene from cameras of their own, with per-view overrides, and `scenes:` gives a viewport a scene of its own: a studio set for one vehicle costs no host code. A click in a view doesn't give a world point. | `docs/ARIADNE.md` §5.18; `src/cvcGL/ariadne/scene_views.cpp` |

**What that adds up to.** The data already flows the way the ask wants: the server holds the state,
and each client rebuilds its view and its physics from the tree. Most of the *rules*, though, are
compiled into both binaries: the body, the controller and the speed veto. Prediction is right
because the client and the relay run the same code, not because the relay told the client how its
world works.

**The energy policy is the exception, and it is the model.** It is a revisioned section the server
writes, with its revision written last, and both the relay's holon and every client read it from the
same leaves. This plan generalizes that one pattern to everything that defines the world.

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

A world document, as proposed syntax. `scene:` is today's syntax; the `world:` block, the
`conference.world` token and `room-rules-step` do not exist yet. S0's `relay` token holds in every
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
| physics | `physics.*` | on/off, gravity, `fixed_dt`, materials; bodies per scene object | new |
| archetypes | `avatars.archetypes.<id>.*` | body, look envelope, allowed controllers | new |
| catalog | `avatars.catalog.*` | colours, accessories | built |
| policy | `avatars.policy.*` | energy, `max_speed`, later per-controller limits | built (energy only) |
| controllers | `controllers.<id>.*` | kernel or guest, version, params, content hash | new |
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
- **Changes that affect the simulation carry an activation tick.** When `physics.*`, `controllers.*`,
  an archetype or the policy changes, the section says which world-clock tick it takes effect on,
  and the relay switches on that tick. A client that learns of it late snaps and replays from that
  tick, which its reconcile already does. Cosmetic sections (lights, looks) apply on arrival.
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
| `vehicle_selector` | A list of the world's vehicle archetypes. The chosen one is drawn alone on a lit, shadowed studio set in its own viewport (a `scenes:` entry). Stats from the archetype's body: size, mass, power, top speed, wheelbase. Settings within the archetype's ranges. The policy: default or a trained network. Identity: name, team, two colours, metadata. A Spawn button. | `avatars.archetypes.*`, `controllers.*`, `entities.*` | §4's identity, §5's `nav@1`, runtime spawning |
| `spawn_points` | A top-down minimap viewport: click to add a point, click one to remove or tag it (team, archetype). The policy: fixed, random, round-robin or nearest-free, per team or archetype. Points are checked against the world's occupancy; candidates are suggested along the roads. | `spawns.*` | click-to-world picking; the road graph |
| `world_physics` | Physics on or off, speed, step and restart. A table of entities: bodies, active, contacts, step time; per vehicle its energy and forces. | `physics.*`, `physics.stats.*` | stats from the world runtime; built-in step and restart |

**Vehicles are archetypes.** The vehicle catalog is the world document's archetypes, beside
`person`: each names its plant (`kinematic`, `jolt`, `dynamic`, `dynamic4w`), its body (a
`vehicle_desc` preset or its fields), its look (a mesh, or the generated Humvee), and the
controllers it allows. libcvc's default worlds ship the built-in vehicles; a site adds its own in
its own world document.

**The gym** is the world runtime driven from Python or headless: `reset(seed)`, `step(actions)`,
`observe()`, a reward the user writes, and an episode log. It absorbs the glue in `pycvc_gl.cars`
(rasters, colliders, plants), which then becomes a thin layer over it. Training runs many worlds
headless; the same document opens in arigl to watch one.

**What the runtime needs that isn't there** (§1):

- `sim_world` takes agents at run time: add and remove, with a stable entity id each.
- A policy per agent, as `nav@1`'s params, loaded once per hash.
- A name per agent, and the team bound to Python and to documents.
- `generated_world` gives the runtime its whole world (buildings, occupancy, road graph, water),
  not only the height field, and the world-to-navigator glue moves from Python to C++.
- The document's physics publishes `physics.stats.*`.
- Built-in handlers for `sim.step` and `sim.restart`.
- A viewport click gives a world point (a ray against the scene, or the ground's height field).

## 15. Phases

Each phase leaves the conference working. **WD** = world document. (S, G, W, H and D are the
scripting-hosts, wasm-guest, ariglx-web, holon and DSL-citizen phases in the documents above; G5 is
GENERATED-WORLD-ROADMAP.md's.)

| Phase | What | Depends on | Size |
|---|---|---|---|
| WD0 | **Done** (2026-11-07). The loose ends: `--set` after seeding, or a guard so a `--set` can't suppress the default world; the "under a sun" comment. Built as `--set` after the world and the room's policies, before a server document's `init:` (§1) | — | S |
| WD1 | **Done** (2026-11-07). **The world document seeds the room.** `arisrv --doc world.ari` publishes the document's `scene:` (objects, lights, shadows), in one revision (`write_room_world`). `cvc://worlds/default.ari` and `large.ari` replace the C++ seeds, and `--scene` names them. The light rig and shadows move from the conference document to the world document (Q1), and the conference document drops its `scene:` block. As built: the wire gained a rig's knobs and the shadows' interval; `room_world_from_scene` leaves out, with a warning, what the wire can't hold (other node types and sources, children, textures, shaders), and publishes every body as a static until WD3; `--scene` together with a `--doc` that has a `scene:` is refused; the C++ twins are the fallback | S0 (met by transfix/libcvc#1046) | M |
| WD2 | **The rules come from the document.** The physics, archetype, policy and controller sections (§3), with the activation tick. The holon and `room_view` read them instead of `avatar_body{}`, `kAvatarWidth`, `max_speed = 4.0` and the panel's "walker" literal. | WD1 | M |
| WD3 | **Entities.** The entities table (§4); server-dynamic props and NPC bodies from the document and from server programs, simulated on the relay by L0's headless runtime; their poses on the pose stream; colliders for meshes. | WD2, L0 | L |
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
| AD1 | **Assets by hash** in the manifest, with size and priority; the lookup order (bundle, cache, pack, relay); world packs from a cvcpkg package, an archive at a URL or a directory, each read through its index. | WD1 | M |
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
| L0 | **The world runtime, local and headless.** The world half of scene realization moves to core libcvc (§14, "Headless first"): the document's physics world and bodies, and `generated_world`'s terrain, buildings and occupancy, with no GL. The `scene.physics` token; a test that a document's state hash after N ticks is the same headless and in `arigl --offscreen`. It reads a document's world sections and runs entities: `sim_world` with agents added and removed at run time; `nav@1` with a policy per entity; identity on entities; the whole generated world handed to the runtime, with its rasters and colliders built in C++; `physics.stats.*`; built-in step and restart; click-to-world picking in a viewport. pycvc bindings for all of it. | WD2, WD3, WD4, WD5 | L |
| L1 | **`world_source` and `world_physics`,** the two panels on pieces that already exist. | WD5 | M |
| L2 | **Vehicle archetypes and `vehicle_selector`,** with the studio preview. | WD4 | M |
| L3 | **`spawns.*` and `spawn_points`,** with the minimap and road suggestions. | WD3 | M |
| L4 | **The gym:** reset, step, observe, reward, episode logs, headless; `pycvc_gl.cars` moves onto it. | — | M |

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
- **Q4. How does a simulation change take effect mid-motion?** Proposed: at a relay-stamped
  activation tick, with late clients replaying (§3).
- **Q5. Who hosts user assets?** Proposed: the relay or its CDN, after checking them (§3, §6).
- **Q6. Who owns placed entities and saved state?** Proposed: the S5 principal. Open: what a guest
  without an account keeps, and for how long.
- **Q7. What persists, and for how long?** The state in §8, and its retention.
- **Q8. What scale does the first persistent world need?** Users per room or cell, and the world's
  size. This decides when WD9 is needed.
- **Q9. One simulator or two?** The holon runs avatars in Jolt at 120 Hz; `sim_world` runs nav
  agents at its own tick and reaches Jolt through `plant_host`. Proposed: the entities table is the
  one registry, `sim_world` becomes `nav@1`'s batch kernel over the entities using it, and both run
  on the world clock.
- **Q10. What does a team mean?** Proposed: a label and colours that spawn rules filter on, and the
  yielding `sim_world` already does. Rules and scoring by team come later, as server programs (§7).
- **Q11. What does "physics off" mean?** Proposed: `physics.enabled = false` pauses the world.
  Dropping vehicles to the kinematic plant is a per-archetype choice, not this switch.
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
