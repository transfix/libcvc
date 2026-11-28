# Changing a running server's scene, what a client inherits, and where things spawn

Status: proposal, 2026-11-24. The owner answered its questions on 2026-11-25 (section 5), and this
revision folds those answers in, along with a review of 3.4 against ARIADNE_SHELL by the session
working on the shell's mounts and DSL processes. Nothing here is built yet. It follows GB5 part 1
(transfix/libcvc#1527), where an rcon admin's Regenerate remakes the server's generated world.

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

**What a client can run from the server today:**
- A client can call the server's functions (`room-call`).
- It can run its own DSL program or WebAssembly guest on the server, sandboxed (`room-run`, S3
  and G4).
- The server can't hand a client an application, a component or a UI to run on the client's
  side. The shell (ARIADNE_SHELL.md) mounts documents in a host, runs `remote:` principals
  sandboxed, and addresses packages by hash. Its section 7 leaves "Relay and conference
  documents as applications" out of scope.

**Austin** is a `data:` entry, `uri: cvcpkg://cy-pca/scene-austin-south-small`,
`path: share/cvc-scenes/austin_south`, `as: city_bundle`
(examples/ariadne/documents/austin_data.ari). Since GB3, a data node whose dataset is a world
draws with the generated-world renderer. A saved lab world (GB4) loads the same way.

**Vehicle spawns:**
- `world: spawns:` lists `spawn_point`s (x, y, heading, team, archetype, world) and a policy:
  `fixed`, `random`, `round_robin` or `nearest_free` (world_rules.h).
- They're part of `world_rules`, which the relay reads from its document. The room does not
  carry them: its rules sections are physics, controllers, archetypes and policy (room_rules.h).
  The relay's world runtime uses them for its vehicles, and since SC1 its holon for avatars.
- `WorldSpawns` checks each point against the grid of the world that's on (free ground, or deep
  enough water for a boat), and its minimap tools add, remove, tag and suggest points (along
  the roads, or over water for a boat).
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

1. **An rcon admin can change the room's scene on a running server:**
   - pick from a list the server offers (stock worlds, its scene directories, packages);
   - or push an arbitrary new one onto the server.
   A scene is everything the server's world is made of:
   - the document's server-side logic;
   - the logic, UIs, components and applications clients run;
   - scene geometry, materials, and datasets such as Austin.
2. **Every client follows.**
   - A generated world, a client makes again from the same seeds and settings, which saves
     bandwidth. It only fetches the bytes when its hash comes out different.
   - Anything else, it downloads, checks against its hash, and caches.
3. **A joining client inherits what the server gives it** to interact with the server's world:
   the server's client-side applications, UIs and components, mounted beside the client's own.
   It gets whatever it doesn't already hold.
4. **Spawn points belong to the scene,** for avatars as well as vehicles and boats.
   - A world can come with preset spawns, like a Quake map.
   - The terrain generator makes good ones by itself: roads and dirt for vehicles and avatars,
     water for boats.
   - A spawn lands on the ground of the world that's on, and an admin can edit the points at
     run time.

These carry over unchanged:
- **The rcon rule.** The owner, 2026-11-24: "the client elevates to admin permissions if they
  have the rcon password, so they can edit the server world". Every call that changes the scene,
  its spawns or what clients inherit is an admin call in arisrv's `admin_call` list.
- **Content is checked by hash.** Nothing a client draws, simulates or runs is taken on a name
  alone.
- **The sandbox.** What a client inherits from a server, the scene document included, runs as
  that room's principal (`room:`, 3.4). It is default-deny, as the shell runs a `remote:` document
  (ARIADNE_SHELL 4.12), and it never carries the user's admin rights.

## 3. Design

### 3.1 Spawns are scene state (SC1, SC2, SC6)

**Avatars get spawn points.**
- `world: spawns:` points tagged `archetype: avatar` (or any avatar archetype id) are avatar
  spawns. An untagged point serves avatars too.
- They ride in `world_rules.spawns` like the vehicles' points. Spawning is the relay's alone,
  so arisrv hands them to its holon (`holon::set_avatar_spawns`); no client needs them.
- holon_sync's newcomer placement then uses them (`holon::spawn_for`):
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

**Preset spawns** are a world's own, like a Quake map: the document's `world: spawns:`, or a
bundle's (below). When a world has them, they're the ones used.

**The terrain generator makes spawns (SC6).**
- A generated world proposes spawn points as part of what it makes, with the same tags the
  minimap's suggester uses:
  - **vehicles and avatars:** points on its roads and dirt, away from buildings and water,
    spread out (a minimum spacing) and facing along the road;
  - **boats:** points over water deep enough for the archetype's draft, clear of the shore by
    half its length.
- They're deterministic from the seed and settings, like the rest of the world, so a client that
  makes the world itself gets the same points.
- They go in the world's bundle (`world.json` `spawns`, below). They're used when the document
  states none, and the panel can keep, edit or clear them.

**Bundles carry spawns (SC6).** The version 2 manifest (`world.json`) gains an optional `spawns`
block in the same shape. The lab's Save writes its points, and a bundle used as a scene (3.2)
brings them.

**After a terrain change** (a Regenerate, a world switch, a scene load):
- Preset points keep their x and y, and their z comes from the new ground.
- A generated world's own points are made again with it.
- `WorldSpawns` checks each point again and publishes the result: free, in water, inside a
  building. Points that no longer work show in its status, for an admin to move.

**Everyone respawns on a change.** The owner, 2026-11-25: "everyone respawns". On a world switch,
a scene load, or a Regenerate under people:
- every avatar is placed again at a spawn point;
- vehicles end, the new scene's `vehicles: start:` are made, and seats and drives end.

**An admin edits spawns at run time (SC2):**
- `server.spawns.set {points, policy, scope}` replaces the room's spawn rules.
- arisrv publishes the new rules section at an activation tick (`write_room_rules` with a new
  stamp), and every holon takes it at that tick, as rules changes already work.
- The minimap's spawn tools, while joined, send their edits as that call, the way GB5 routes the
  lab's Regenerate (`GeneratedWorldRelay`).

### 3.2 Loading or pushing a scene on a running server (SC3, SC4, SC8)

**From the server's list:** `server.scene.load SOURCE [OPTIONS]`, an admin call.
- SOURCE is a document or a bundle the server can already reach:
  - a stock world, `cvc://worlds/<name>.ari`;
  - a path under one of the server's `--scene-root DIR`s;
  - a `cvcpkg://owner/package` with `path:`, through the server's `--net-allow`.
- A bundle (Austin, a saved world) is given with `as: city_bundle`.
- The server reads nothing outside that list on a load. An admin's call must not become a way to
  read the server's disk; a push (below) is how arbitrary content arrives.
- `server.scene.list` answers what the list offers, for a picker.

**Pushed by an admin (SC8):** `server.scene.push`. An admin client uploads a scene package, and
the server runs it.
- **The package.** A world pack (OW5k): the files and a `world-pack.index` (SHA-256, size and
  path for each).
  - It holds the document (server-side logic, the scene, its worlds and spawns, the `clients:`
    block of 3.4).
  - It also holds what the document names: geometry, materials, textures, components, apps, and
    datasets such as a bundle.
- **The upload.**
  1. The admin sends the index first.
  2. The server answers which hashes it lacks; it already holds what an earlier scene or package
     shared.
  3. The admin sends those in chunks (`server.scene.put HASH OFFSET BYTES`).
  4. The server checks each file against the index, stores it content-addressed, and loads the
     document as for `server.scene.load`.
- **Persistence.** A pushed scene stays in the server's store and can be listed and loaded again
  until an admin removes it (`server.scene.remove`).
- **What it may do.** A pushed document's server-side logic runs as the room's document does
  today, so the pusher (an rcon admin) is trusted with the server, as `--doc` is.

**A bundle with no document gets a wrapper document (SC4).**
- `scene_for_bundle(uri, options)` writes a small, canonical document: `physics: {}`, one data
  node naming the bundle (`as: city_bundle`, its `body:`), the stock lights, and the bundle's
  spawns.
- The wrapper's text is a pure function of its inputs. A client makes the same text and gets the
  same SHA-256.
- `world_source` names the wrapper by its inputs (`kind: bundle`, the bundle's URI and its
  content hash, `city_bundle_hash`), so a client wraps exactly what the server ran.

**The server swaps at a tick.**
1. It loads the new scene in the background (decoding Austin takes a while). The old scene keeps
   running meanwhile, and `conf.rooms.<room>.scene_load.{state, progress, error}` says how far it
   is.
2. Once the new `SceneWorld` is ready, the server picks an activation tick T and, at T:
   - replaces the `ServerDocument` and its world;
   - rewrites `scene`, `world_source`, `rules` (spawns included), `world_on`, the clock, and what
     clients inherit (3.4), all stamped for T;
   - respawns everyone (3.1).
3. A load that fails leaves the old scene running, says why in `scene_load.error`, and answers
   the call with the error.

**Clients follow the source.** The owner, 2026-11-25: a joined client "should get whatever it
needs from the server that it doesnt have client side to interact with its holon".
- `world_replica` already rebuilds when `world_source`'s stamp moves.
- While joined, a host also shows the room's scene. It does not load it as its own outermost
  document: that would run the server's document as `local`, outside the sandbox, and end the
  client's own apps.
- Instead, the followed scene is a mount in the room's slot (3.4): the shell's scene of a mounted
  document (SH3), shown in the main view.
  - It runs no server-side logic on the client; that stays on the server, and the client sees
    its replicated state.
  - Its geometry, materials and datasets go through ARIADNE_SHELL 4.12's list of formats a
    sandboxed document may load. A bundle format not on that list (Austin's `city_bundle`) is
    added on purpose, and put in the fuzzing order.
  - When the stamp moves, the mount is replaced with the new document: the resolved file, a cached
    or fetched copy, or the wrapper the client makes itself. What it inherits changes with it.
- This is the default. A host started with `--follow-room off` shows the room scene only, as
  today.

### 3.3 The bytes, by content address (SC5, which is GB5 part 2)

**Generated worlds come first.** A client makes a generated world itself from `world_on`'s seeds
and settings (GB5 part 1 does this), which costs no bandwidth. `world_on` gains each generated
world's dataset hash and revision. A client compares its own hash and fetches the server's bytes
only on a mismatch: another generator version, another platform's floating point, a generator it
doesn't have.

**Everything else:** the document (by its SHA-256), the datasets it names (a bundle by
`city_bundle_hash`), and the files of what the client inherits (3.4). It looks, in order:
1. its content-addressed cache;
2. wherever the URI resolves here;
3. its world packs (OW5k);
4. cvcpkg (its cache, or cvcpkg.org when the host allows that network);
5. the relay.

**Fetching from the relay:**
- `server.fetch HASH [OFFSET LENGTH]` answers the bytes of any file of the scene the server is
  running, in chunks under the call result cap.
- This one is not an admin call: anyone joined may fetch the scene they're in.
- Large scenes are fine. The owner, 2026-11-25: "large scenes are fine, we have tons of bandwidth
  and when running on a LAN this isn't a problem". There's no size cap by default; a relay may
  set one.

**What the client keeps.** It checks every file against its hash before using it, and keeps it in
`<cache>/<sha256>`, so a second join, or a return to a scene, costs nothing.

**The conflict.** The hash and revision in `world_on` conflict with transfix/libcvc#1521 (OW5q's
`seas` and `weathers` in `room_world_on`); whichever lands second takes the conflict.

### 3.4 What a joining client inherits (SC7)

The owner, 2026-11-25: "The server has apps that the client runs basically. The client can run
it's own apps and UI but when the client connects it inherets what the server gives it to
interact with the server world".

This section was reviewed against ARIADNE_SHELL by the session working on the shell's mounts and
DSL processes; its six changes are folded in below.

**The server's document says what clients get. It asks; it never grants.**

```yaml
clients:
  apps:
    - { as: drive, uri: apps/drive_panel.ari, view: window }
    - { as: map, uri: apps/minimap.ari, args: { follow: me } }
  components: [components/vehicle_hud.ari]
```

- There is no `grants:` here. In ARIADNE_SHELL 4.5 the loading document grants and the mounted
  document asks. The client is the loading document, so a server-side grant would be the server
  granting itself.
- Each inherited app lists what it needs in its own `requires:`. The client's slot grants it, up
  to the slot's ceiling, and asks the user for anything beyond the defaults.
- `clients:` goes into the closed `.ari` schema (#1464).

**The room carries it.**
- `conf.rooms.<room>.clients` holds each entry's URI and SHA-256, its args and its view, stamped.
- The files themselves come by content address (3.3).

**A joined host mounts them in a slot of its own.**
- The slot is a second `mounts:` widget, `mounts: shell.room_apps`, with entries at
  `shell.room_apps.<as>`. It is not under `shell.apps`, whose every child the shell mounts as an
  app.
- The slot's `grants:`, `allow:` and budget defaults are where the client caps everything a server
  hands out.
- The client's join code writes the slot. It reconciles `conf.rooms.<room>.clients` into it: it
  checks each hash against the content store, applies the host's policy, and refuses what the
  policy refuses.
- The slot is never bound to the replicated room key, so a server can't write the client's mount
  entries.
- A host that isn't the shell gets the same slot from a built-in component, so there is one code
  path.
- The followed scene (3.2) is a mount in the same slot.

**The principal is `room:<server>/<room>`.**
- `relay:` would collide with the `relay:` block a document uses to be a server, which is what
  ARIADNE_SHELL section 7 means by "Relay and conference documents as applications".
- Like `remote:`, it is fixed at mount time from the connection:
  - `<server>` is the canonical origin the client dialed: scheme, host and port;
  - nothing a document says can name or raise it;
  - everything an app loads inherits it;
  - its relative paths resolve in the room's content-addressed set, and `cvc://` stock components
    are allowed.
- Two rooms on one server are two principals, which is right for storage. It gets no
  credentials, so none are shared between them.

**The room channel is scoped and never carries admin rights.**
- `room-call` from an inherited app acts as the joined user only.
- The admin calls (`admin_call`'s names) are refused for this principal, even while the user is
  elevated by rcon. Otherwise an app one admin pushed would act with another admin's elevation
  without that admin knowing.
- The rcon password and the elevation are never readable from the principal.
- Admin calls come only from the host's own admin UI (the lab's panel, an admin console).
- Its tokens reach only that room on that connection, never another room or server.

**What the principal may do: proposed defaults, for the owner to settle.**
- **Given by default,** like `remote:` in 4.12:
  - its own state, windows and scene;
  - the host verbs marked safe for remote documents;
  - its own storage, with a quota;
  - `extend.settings`;
  - a tighter budget (4.11);
  - for its own room only: `state.room.read`, `room-call` (not admin) and `room-run` within the
    server's read-lane allow list. These are the reason to join, so asking for them would only
    add a click.
- **Not given by default:**
  - the network: its channel is the room, and `net.origin` is by grant;
  - credentials, end-to-end keys and OIDC: never the client's, and none of its own unless granted;
  - local files: `file-pick` is by grant;
  - `python:` blocks and native extensions: they fail the mount.
- **Asked on first join, and rememberable per server** (like 4.13's "Always trust this
  publisher"):
  - `delegate.keys` and `delegate.pointer`, which a drive panel needs; 4.6's focus rules already
    limit them to the focused app;
  - `extend.menubar` and `extend.context_menu`;
  - `request.main_view`.
  A host setting can grant these without asking for servers the user dialed directly.
- **Never:** `access.settings` (the shell's own settings), and `request.open` unless the owner wants
  servers to be able to open documents.

**The client keeps its own.** Its own documents, apps and UI keep running beside the inherited
ones.

**Lifecycle.**
- When `clients` changes (a scene load, a push), the host applies it at the activation tick T: it
  unmounts what is gone and mounts what is new. An app whose hash changed reloads, keeping its
  resident as 4.2's caching does.
- Leaving the room ends every inherited mount and drops its runtime state. Its per-principal
  storage stays until the user clears it.

**Logic for the client side.**
- An inherited app's DSL runs on the client as its own processes, under its mount, so a server can
  give clients UIs and client-side behaviour as well as server-side logic.
- SH0's scope and `owner_root` already label these processes in `(ps)` and the Processes view.
- `room-run` programs are the server's processes, not the client's.

**The badge** reads "from <server>, room <room>" and is always pinned, since everything is
content-addressed. "Pushed by <admin>" can show once SC8 knows who pushed it.

**What SC7 needs from the shell:**
- SH1, mounts and budgets;
- SH3, scenes of mounted documents, for `view: window` and the followed scene;
- SH7, asks and grants, keys and pointer;
- SH11, credentials per principal; without it, an inherited app could find the client's
  credentials by name;
- SH12, the sandbox and the format list;
- the badge, from SH14.

SH13's https fetcher isn't needed, since the files come through the room (SC5); its manifest
checker, which reads OW5k's index fields, is shared.

## 4. Phases

| Phase | What | Size |
|---|---|---|
| SC1 | **Avatar spawns** (in progress: `holon::set_avatar_spawns`, `spawn_for`, `ground_at`, ground-relative respawn; respawning everyone on a world switch or a Regenerate is not done yet): points tagged for avatars in `world: spawns:`. Newcomers and respawns take them by the policy, put on the ground of the world that's on (the ring when there are none, also on the ground). Respawn is relative to the ground. Everyone respawns on a world change. Tests: on a generated world and on Austin, a joining avatar stands on the ground, and a Regenerate under it re-places it. | M |
| SC2 | **`server.spawns.set`** (admin): spawns re-published at an activation tick. The minimap's tools send their edits to the server while joined. | S–M |
| SC3 | **`server.scene.load` from the server's list** (stock worlds, `--scene-root`, cvcpkg): background load, swap at a tick, everything re-published, everyone respawned. Joined hosts follow the room's scene by default (`--follow-room off` to opt out). `server.scene.list`. | L |
| SC4 | **Bundles as scenes:** `scene_for_bundle` wrapper documents, `world_source` by bundle hash, Austin and saved lab worlds loadable. | M |
| SC5 | **Bytes by content address** (GB5 part 2): `server.fetch`, the cache, the pack and cvcpkg lookups, and the dataset hash and revision in `world_on` so a generated world is fetched only on a mismatch. | L |
| SC6 | **Spawns in bundles and from the generator:** `world.json` `spawns`, written by the lab's Save. The terrain generator proposes points on roads and dirt (vehicles, avatars) and over water (boats), deterministic from the seed. | M |
| SC7 | **What a client inherits:** the document's `clients:` block (in the closed schema; asks, no grants), `conf.rooms.<room>.clients`, a host-written `shell.room_apps` slot reconciled from it, the `room:<server>/<room>` principal with no admin calls, the followed scene as a mount in that slot, applied at tick T. Needs the shell's SH1, SH3, SH7, SH11, SH12 and SH14's badge. | L |
| SC8 | **`server.scene.push`:** an admin uploads a scene package (a world pack: document, server and client logic, apps, components, UIs, geometry, materials, datasets), sending only the hashes the server lacks, checked against the index, kept in the server's store, loaded as SC3 does. `server.scene.remove`. | L |

**Order.** SC1 and SC2 fix the problem visible today and stand alone. SC3 is the core. SC5
before SC8, since a pushed scene's clients need to fetch it. SC4 and SC6 can go anywhere after
SC3 and SC1. SC7 waits on the shell's mounts and sandbox.

## 5. Decisions (the owner, 2026-11-25), and what's still open

1. **Where scenes come from:** "a pre-set list is fine but ideally an admin can push an arbitrary
   new world onto the server and the clients can download it (or regenerate if it's generated to
   save bandwidth, they just need the seeds and settings to match the world on the server)". The
   server's document can hold logic for clients as well as for itself, and an admin can push
   "server/client logic, UIs or components or apps, scene geometry materials etc." That's
   SC3 and SC8 here, and 3.4 for client logic.
2. **People on a change:** "everyone respawns". Worlds can have preset spawns, like a Quake map,
   and the terrain generator could make good ones itself (roads or dirt for vehicles and avatars,
   water for boats). That's 3.1 and SC6.
3. **Large scenes from the relay:** yes, with no default cap (3.3).
4. **A joined client follows the server:** yes; "it should get whatever it needs from the server
   that it doesnt have client side to interact with its holon" (3.2, 3.4).

Still open:
- **The `room:` principal's defaults** (3.4): the review proposes what is given by default, what is
  asked on first join (keys and pointer, menus, the main view), and what is never given. For the
  owner to settle, including whether servers may open documents (`request.open`).
- **The pushed-scene store:** where it lives on the server, and when it is pruned.
- **A pushed scene after a restart:** does `--doc` become "the last scene pushed", or does a
  restart go back to `--doc`?
