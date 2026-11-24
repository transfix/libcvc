# Changing a running server's scene, and where things spawn

Status: proposal, 2026-11-24. Nothing here is built yet. It follows GB5 part 1 (transfix/libcvc#1527),
where an rcon admin's Regenerate remakes the server's generated world.

The owner, 2026-11-24: "when we change the terrain, where are the spawn points set for avatars and
vehicles? do we have a way to set spawn points? Im rethinking how we get the server to change
scenes, including loading pregenerated data like the austin bundle".

## 1. What there is today

**The scene is fixed when the server starts.**
- arisrv runs one document: `--doc PATH`, or a stock world with `--scene default|large`
  (`cvc://worlds/<name>.ari`).
- At startup it writes three things to the room, once (src/arisrv/main.cpp, around line 1729):
  - `conf.rooms.<room>.scene` (`write_room_world`): what every client draws, lights and shadows
    included;
  - `world_source` (`write_room_world_source`): the document's URI and SHA-256;
  - `rules` (`write_room_rules`): physics, archetypes, policy, controllers, and the vehicles'
    spawns.
- Nothing rewrites the source or the rules afterwards, and `ServerDocument` has no reload.

**What can change while it runs:**
- **Switching worlds:** the server can turn on a different one of the worlds the document
  declares. `world_on.world` carries the switch, and replicas follow it (OW5f).
- **Regenerating:** a generated world can be made again with new params. `world_on.generated`
  carries them (OW5f). GB5 part 1 lets an rcon admin ask for it (`server.regenerate`).

**What clients do:**
- `world_replica` holds a copy of the document's world for the physics. It loads the document
  when the client resolves the room's URI to a file with the same SHA-256: a stock world, or a
  world pack (OW5k, `CVC_WORLD_PACKS`). It rebuilds when `world_source`'s stamp moves.
- arigl draws the document it was started with, plus the room scene. It does not follow
  `world_source`.

**Austin** is a `data:` entry, `uri: cvcpkg://cy-pca/scene-austin-south-small`,
`path: share/cvc-scenes/austin_south`, `as: city_bundle`
(examples/ariadne/documents/austin_data.ari). Since GB3, a data node whose dataset is a world
draws with the generated-world renderer. A saved lab world (GB4) loads the same way.

**Vehicle spawns:**
- `world: spawns:` lists `spawn_point`s (x, y, heading, team, archetype, world) and a policy:
  `fixed`, `random`, `round_robin` or `nearest_free` (world_rules.h).
- They're part of `world_rules`, so the room carries them.
- `WorldSpawns` checks each point against the grid of the world that's on (free ground, or deep
  enough water for a boat), and its minimap tools add, remove, tag and suggest points.
- Those tools edit the document running where they are. There's no server call for spawns.
- The points are fixed x/y positions: a new terrain leaves them where they were. Whether the check
  re-runs after a Regenerate is unverified.

**Avatar spawns don't exist:**
- A joining avatar goes on a 4 m ring around the origin, at the next golden-angle step, at
  z = 0 (holon_sync.cpp, "Add any newcomer").
- One that falls below `respawn_below_z` (-10 m) goes back to where it joined (holon.cpp).
- Nothing puts it on the ground. On generated terrain, or Austin (its ground is far above 0), it
  probably starts inside or under the terrain. That's unverified, and SC1's first test should
  show it.

## 2. Goals

1. An rcon admin can change the room's scene on a running server:
   - to another document the server is allowed to load;
   - to a world bundle (Austin, a saved lab world) with no document of its own.
2. Every client follows: it draws and simulates the new scene. A client without the bytes gets
   them by content address and checks them.
3. Spawn points belong to the scene, for avatars as well as vehicles. A spawn lands on the ground
   of the world that's on, and an admin can edit the points at run time.

These carry over unchanged:
- **The rcon rule.** The owner, 2026-11-24: "the client elevates to admin permissions if they
  have the rcon password, so they can edit the server world". Every call here is an admin call
  in arisrv's `admin_call` list.
- **Content is checked by hash.** Nothing a client draws or simulates is taken on a name alone.

## 3. Design

### 3.1 Spawns are scene state (SC1, SC2)

**Avatars get spawn points.**
- `world: spawns:` points tagged `archetype: avatar` (or any avatar archetype id) are avatar
  spawns. An untagged point serves avatars too.
- They ride in `world_rules.spawns` like the vehicles' points, so the room already carries them.
- holon_sync's newcomer placement then uses them:
  - It picks a point by the policy and scope (a team or archetype draw, `nearest_free` against the
    other avatars).
  - It sets z to the ground of the world that's on at (x, y), plus the archetype's half height and
    a small clearance. The ground comes from the server's `SceneWorld`, a raycast or
    `vehicle_ground`.
  - With no avatar points, the golden-angle ring stays, but each spot is put on the ground the
    same way.

**Respawn uses the ground and the points.**
- An avatar respawns when it is `respawn_depth` (default 10 m) below the lowest ground of the
  world that's on, not below a fixed -10 m. Austin's ground never goes near -10.
- It respawns at a spawn point by the policy, not where it joined, because where it joined may
  be gone after a Regenerate.

**After a terrain change** (a Regenerate, a world switch, a scene load):
- Every point keeps its x and y, and its z comes from the new ground.
- `WorldSpawns` checks each point again and publishes the result: free, in water, inside a
  building. Points that no longer work show in its status, so an admin can move them or ask for
  suggestions (the roads suggester exists already).
- Spawns are not moved automatically, since the author chose where they are.

**An admin edits spawns at run time (SC2):**
- `server.spawns.set {points, policy, scope}` replaces the room's spawn rules.
- arisrv publishes the new rules section at an activation tick (`write_room_rules` with a new
  stamp), and every holon takes it at that tick, as rules changes already work.
- The minimap's spawn tools, while joined, send their edits as that call, the way GB5 routes the
  lab's Regenerate (`GeneratedWorldRelay`).

**Bundles carry their own spawns (SC6).** The version 2 manifest (`world.json`) gains an optional
`spawns` block in the same shape. A saved lab world writes the lab's points, and a bundle used as
a scene (3.2) brings them.

### 3.2 Loading a scene on a running server (SC3, SC4)

**The call:** `server.scene.load SOURCE [OPTIONS]`, an admin call.
- SOURCE is a document or a bundle.
- **Documents:**
  - a stock world, `cvc://worlds/<name>.ari`;
  - a path under one of the server's `--scene-root DIR`s;
  - a `cvcpkg://owner/package` with `path:`, through the server's `--net-allow`.
- **Bundles:** a `city_bundle` (Austin, a saved world), by the same three kinds of address, with
  `as: city_bundle`.
- **Allow-list:** the server loads nothing outside that list. An admin's call must not become a
  way to read the server's disk.
- **Listing:** `server.scene.list` answers what the roots and the stock worlds offer, for a
  picker.

**A bundle with no document gets a wrapper document.**
- `scene_for_bundle(uri, options)` writes a small, canonical document: `physics: {}`, one data
  node naming the bundle (`as: city_bundle`, its `body:`), the stock lights, and the bundle's
  spawns.
- The wrapper's text is a pure function of its inputs. A client makes the same text and gets the
  same SHA-256.
- `world_source` names the wrapper by its inputs (`kind: bundle`, the bundle's URI and its
  content hash, `city_bundle_hash`), so a client wraps exactly what the server ran.

**The server swaps at a tick.**
1. It loads the new scene in the background. Decoding Austin takes a while (the small package;
   the full city is 117 MB). The old scene keeps running meanwhile, and
   `conf.rooms.<room>.scene_load.{state, progress, error}` says how far it is.
2. Once the new `SceneWorld` is ready, the server picks an activation tick T and, at T:
   - replaces the `ServerDocument` and its world;
   - rewrites `scene`, `world_source`, `rules` (spawns included), `world_on` and the clock, all
     stamped for T;
   - removes the old world's vehicles and spawns the new world's `vehicles: start:`;
   - respawns every avatar at the new spawns (3.1).
3. A load that fails leaves the old scene running, says why in `scene_load.error`, and answers
   the call with the error.

**Clients follow the source.**
- `world_replica` already rebuilds when `world_source`'s stamp moves.
- The new piece is the drawn document. While joined, arigl (and any host that joins) treats
  `world_source` as its document: when the stamp moves, it loads the new document. That means the
  resolved file, a pack entry, or the wrapper it makes itself. It keeps its own windows and
  panels and drops the old scene's. A host started with `--follow-room off` keeps its own
  document, as today.

### 3.3 The bytes, by content address (SC5, which is GB5 part 2)

**What a client needs:** the document (by its SHA-256) and the datasets it names (a bundle by
`city_bundle_hash`). It looks, in order:
1. wherever the URI resolves here;
2. its world packs (OW5k);
3. its cvcpkg cache, or cvcpkg.org itself, when the URI is `cvcpkg://` and the host allows that
   network;
4. the relay.

**Fetching from the relay:**
- `server.fetch HASH [OFFSET LENGTH]` answers the bytes of a file of the scene it is running,
  never anything else, in chunks under the call result cap.
- This one is not an admin call: anyone joined may fetch the scene they're in.

**What the client keeps:**
- It checks what it got against the hash before using it.
- It keeps it in a content-addressed cache (`<cache>/<sha256>`), so a second join, or a return
  to a scene, costs nothing.

**`world_on` carries each generated world's dataset hash and revision.**
- A client that made its own world (GB5's follow) compares hashes. On a mismatch (another
  generator version, another platform's floating point), it fetches the server's bytes instead
  of trusting its own.
- This is the part that conflicts with transfix/libcvc#1521 (OW5q's `seas` and `weathers` in
  `room_world_on`); whichever lands second takes the conflict.

## 4. Phases

| Phase | What | Size |
|---|---|---|
| SC1 | **Avatar spawns:** points tagged for avatars in `world: spawns:`. Newcomers and respawns take them by the policy, put on the ground of the world that's on (the ring when there are none, also on the ground). Respawn is relative to the ground. Spawns are checked again after a terrain change. Tests: on a generated world and on Austin, a joining avatar stands on the ground, and a Regenerate under it re-places it. | M |
| SC2 | **`server.spawns.set`** (admin): spawns re-published at an activation tick. The minimap's tools send their edits to the server while joined. | S–M |
| SC3 | **`server.scene.load` for documents** (stock worlds, `--scene-root`, cvcpkg): allow-list, background load, swap at a tick, everything re-published, vehicles and avatars respawned. Joined hosts follow `world_source` (`--follow-room`). `server.scene.list`. | L |
| SC4 | **Bundles as scenes:** `scene_for_bundle` wrapper documents, `world_source` by bundle hash, Austin and saved lab worlds loadable. | M |
| SC5 | **Bytes by content address** (GB5 part 2): `server.fetch`, the cache, the pack and cvcpkg lookups, and the dataset hash and revision in `world_on`. | L |
| SC6 | **Spawns in the bundle manifest:** `world.json` `spawns`, written by the lab's Save. | S |

SC1 and SC2 don't depend on the rest and fix the problem visible today. SC3 is the core.
SC4 and SC5 can go in either order, though without SC5 only clients that already hold Austin
(cvcpkg cache, a pack) can follow it.

## 5. Open questions for the owner

1. **Where may the server load scenes from?** Proposed: the stock worlds, any `--scene-root`
   directory, and cvcpkg packages when the server runs with `--net-allow https://cvcpkg.org`.
2. **What happens to people on a scene change?** Proposed: every avatar respawns at the new
   spawns. Vehicles end, and the new scene's starting vehicles are made. Seats and drives end.
3. **May clients fetch large scenes from the relay?** Proposed: yes, in chunks, but cvcpkg and
   packs come first, and a relay may cap it (`--fetch-max-mb`). Austin's full city is 117 MB.
4. **Should a joined arigl follow the server's document by default?** Proposed: yes; a host that
   wants its own lab while joined passes `--follow-room off`.
