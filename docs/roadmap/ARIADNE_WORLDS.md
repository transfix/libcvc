# Ariadne worlds: the server's document defines the world, and every client runs it

> **Status: PLANNED (2026-11-07).** Nothing here is built. Q1 is settled: the light rig lives in
> the server (§11). The other decisions are open. Parent
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

## 1. What exists

Checked against master 797ba4d9.

| Part | Today | Where |
|---|---|---|
| World geometry | C++. The relay seeds a 20 m ground plane and a 1 m reference box, or with `--scene large` a 100 m plane and 24 meshes (the built-in bunny). It seeds only while the scene's revision is 0. | `default_scene_objects`, `large_demo_scene_objects` in `src/cvc/ariadne/conference/room_scene.cpp`; `src/arisrv/main.cpp` (`--scene`) |
| Wire format | Scalar leaves under `conf.rooms.<room>.scene.{rev, env.*, objects.<id>.*}`: a subset of the document's `Scene` / `SceneNode` / `SceneLight`. Meshes go by `source.uri`. Removal is soft (`present = 0`); `rev` is written last. | `inc/cvc/ariadne/conference/room_scene.h` |
| Look | The wire carries lights and shadows (`scene.env.lights.*`, `scene.env.shadows.*`), but the relay seeds none. The client document declares the light rig and shadows, by an earlier choice, so the rig "lives here, not in the server". | `default_scene_env`; `examples/ariadne/documents/conference_roster.ari` |
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
| Documents on the server | None. arisrv is a fixed relay. S0 adds `arisrv --doc server.ari`. | ARIADNE_SCRIPTING_HOSTS.md §6.1 |
| What a document can already say | Far more than the room tree carries: `scene:` with primitives, meshes with `fit:`, heightfields, light rigs and shadows, and the custom node types `generated_world` and `cloud_sky`. `scene.physics:` gives gravity, `fixed_dt`, materials and per-node bodies (dynamic, kinematic or static; box, sphere, hull, mesh or terrain). All of it parses in core libcvc, with no GL. | `docs/ARIADNE.md` (Scene, Physics); `src/cvc/ariadne/loader.cpp` |
| Persistence | None. The room lives in memory, and a restart re-seeds it. Only the device-key pins can go to a file. `distributed_state_config::blob_store_path` is not read. | `src/arisrv/main.cpp` (`--device-pins`); DISTRIBUTED_STATE_ROADMAP.md |
| Scale pieces | Built but unused by the conference: hash partitioning (`state_hash_partition`, `state_cluster_shard::partition()`) and delegation leases (`state_delegation_manager`). | `inc/cvc/state/` |

**What that adds up to.** The data already flows the way the ask wants: the server holds the state,
and each client rebuilds its view and its physics from the tree. Most of the *rules*, though, are
compiled into both binaries: the body, the controller and the speed veto. Prediction is right
because the client and the relay run the same code, not because the relay told the client how its
world works.

**The energy policy is the exception, and it is the model.** It is a revisioned section the server
writes, with its revision written last, and both the relay's holon and every client read it from the
same leaves. This plan generalizes that one pattern to everything that defines the world.

**Two loose ends**, for WD0:

- The comment above the seeding in `src/arisrv/main.cpp` says the world is lit "under a sun"; no
  light is seeded.
- `--set` is applied before the scene is seeded, and seeding happens only at revision 0. A `--set`
  of `scene.rev` therefore stops the default world from being seeded at all, and a `--set` of an
  object's leaves is overwritten by the seed or quietly added to it.

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
`conference.world` token and `room-rules-step` do not exist yet.

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

## 10. Phases

Each phase leaves the conference working. **WD** = world document. (S, G, W, H and D are the
scripting-hosts, wasm-guest, ariglx-web, holon and DSL-citizen phases in the documents above; G5 is
GENERATED-WORLD-ROADMAP.md's.)

| Phase | What | Depends on | Size |
|---|---|---|---|
| WD0 | The loose ends: `--set` after seeding, or a guard so a `--set` can't suppress the default world; the "under a sun" comment | — | S |
| WD1 | **The world document seeds the room.** `arisrv --doc world.ari` publishes the document's `scene:` (objects, lights, shadows) through `write_scene_object` / `write_scene_env`. `cvc://worlds/default.ari` and `large.ari` replace the C++ seeds, and `--scene` names them. The light rig and shadows move from the conference document to the world document (Q1), and the conference document drops its `scene:` block. | S0 | M |
| WD2 | **The rules come from the document.** The physics, archetype, policy and controller sections (§3), with the activation tick. The holon and `room_view` read them instead of `avatar_body{}`, `kAvatarWidth`, `max_speed = 4.0` and the panel's "walker" literal. | WD1 | M |
| WD3 | **Entities.** The entities table (§4); server-dynamic props and NPC bodies from the document and from server programs; their poses on the pose stream; colliders for meshes. | WD2 | L |
| WD4 | **A controller catalog.** Kernels by `name@version`, a determinism gate per kernel, capability negotiation with an interpolation fallback; the vehicle as the second kernel. A participant picks from the archetype's list (§5k phase 3). | WD2 | L |
| WD5 | **Generated worlds** through the world document: G5's descriptor, height-field statics on the relay. | WD1, G5 | M |
| WD6 | **User looks and bodies.** Assets by hash, checked and rehosted by the relay; bodies clamped to the archetype (§5k phase 4). | WD2, S5 | M |
| WD7 | **User controllers as guests**, on the relay and the owner's client, metered and clamped; the threat model first (§5k phase 5, §5i D4). | WD4, G2, G4, S5 | L |
| WD8 | **Persistence.** Snapshot and journal of the world's state; restart restores it; per-principal saved state. | WD3, S5 | M |
| WD9 | **Cells and many holons.** Cell sections and paging, then subdelegation and handoff. | WD8, H3, H4 | L |

**The first cut is WD0 + WD1.** WD1 needs S0, which is next on the tracker anyway, for application
services. S0's `server: publish:` namespaces (§6.5.3) should be designed with world sections in
mind from the start.

## 11. Open decisions

- **Q1. Where does the look live? Settled (owner, 2026-11-07): the rig lives in the server.** The
  world document declares the lights and shadows, and every client draws the world under them.
  Still open: whether a viewer may lower their cost (shadow resolution, LOD, effects off) or adjust
  for accessibility (brightness, contrast). It never changes geometry or physics.
- **Q2. Do clients ever see the world document?** Proposed: no; only the derived sections (§2).
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

## 12. Where this plugs in

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
