# arisrv identity, admission and admin (ID1–ID4)

Status: ID1 (#1293), ID2 and ID3 are built, and so is ID4a (#1296) with arisrv's wiring of it;
ID4b and ID4c are the plan. It extends the conference roadmap's §5f (room token,
device id, device keys) and the scripting hosts' S5 (`set_authenticator`, principals, role gates),
S3 (client programs) and G4 (client WebAssembly guests, #1283). It sits beside the resident guests
plan (#1290, `ARIADNE_RESIDENT_GUESTS.md`), whose gas ledger keys on this plan's identity.

## 0. What is asked

1. **A stable identity for every client**, both connected and previously connected. By default it
   is a semi-anonymous id hash made from the client's hardware, OS user, and the public and local
   IP addresses it connects from.
2. **A log for each id**: its connections, with IP and geographic information where available,
   and which of them are active. The log is kept in the state tree, in an area private to the
   server.
3. **A username for each id.** A client can change its own, and any other self settings the
   server offers, through a room call.
4. **Basic authentication that works without OIDC.** There are no per-user passwords yet. There
   is an optional **guest password** a client may need to connect at all. There is also a
   Quake-style **rcon password**: a connection that submits it may use admin commands, and a new
   connection must submit it again.
5. **Client guest programs on the server**, WebAssembly and Python, allowed by the server's
   settings, with memory and compute limits for each client.
6. Identities may be ephemeral. Nothing has to survive a restart yet.

## 1. What exists

| Piece | Where | What it gives |
|---|---|---|
| Device id | `state_identity.h` `default_device_id`, `scoped_device_id`; `Hello.device_id` | A salted SHA-256 of the machine id, host name and OS user, scoped to the relay. A browser keeps a random one in `localStorage`. An identifier, not a credential. |
| Device key | `state_device_key.h`, `Hello.nonce` + `DeviceProof` | Ed25519 per relay, pinned on first use. A verified device gets `participants.<pid>.device` (`room_net.cpp` `write_link_devices`). |
| Room token | `--room-token-file`, `CVC_ARISRV_ROOM_TOKEN` | A shared secret in the bearer slot, checked before admission: gRPC `authorization: Bearer`, WebSocket `cvc-token.<t>`. `valid_bearer_token` allows 1–4096 characters of `[A-Za-z0-9._~-]`. |
| Authenticate hook | `state_transport::set_authenticator`, `@host.authenticate` | A principal `{id, roles}` per connection, from `{bearer, transport, peer}`. It keys bound writes, `ctx.principal`, call limits and `host.disconnect`. When it is set, it decides instead of the room token. |
| Role gates | `@export(roles=[...])` | A caller without one of the roles gets `forbidden`. |
| Private tree | arisrv `main.cpp`: the shard's interest is `conf`; `ServerDocument::scope` adds each `publish:` namespace | **Nothing else in the relay's tree is ever sent.** A root outside `conf` and every `publish:` namespace is private by construction. S6's read scope only narrows what is published. |
| Room calls | `call_step` in arisrv `main.cpp` | Handles `program` (S3) and `guest` (G4). Every `fn` goes to arisrvx's Python, or fails with `unavailable`. |
| Client caps | `ServerBlock::client_programs`, `client_guests` (`server_document.h`) | `{steps, time, memory_mb}` for every client alike, below `ariadne.limits.action_max_*`. A guest's memory is 16 MiB by default, and a module at most 1 MiB. |
| Participant name | `participants.<pid>.profile.name`, set by `room_join` (`room_intrinsics.cpp`) | Written by the client itself. The gate allows it, because it is in the client's own subtree. |

What is missing: a server-side record of who connected, a username the server owns, a password a
person can type, an admin layer, and caps that differ by client.

## 2. Design

### 2.1 The client id (ID1)

```
client_id = "c" + first 31 hex of SHA-256("cvc-client-id/1" | relay salt | "device" | device id)
          = "c" + first 31 hex of SHA-256("cvc-client-id/1" | relay salt | "address" | local address | public address)   with no device id
```

The leading `c` makes the id a state name (a letter, then letters and digits), so it keys a node.

- **device id**: the `Hello.device_id` the relay already takes. It covers the hardware, host name
  and OS user, and is already scoped to the relay. The device key's pin already ties one key to
  each device id, so the key isn't hashed in. The record's `verified` says whether it was proved.
- **The addresses are logged, not hashed in (D1).** Each connection's entry has both:
  - the **public address** the transport sees (`state_credential.peer`). Behind a reverse proxy
    that is the proxy's address. Trusting `X-Forwarded-For` needs a flag of its own, which is left
    for later.
  - the **local address**, from a new `Hello.local_address`: the client's own end of the
    connection (`getsockname`). A browser can't see it and sends "".

  A laptop on a new network keeps its id and its username.
- **A client that sends no device id** is hashed on its two addresses. That covers an old client,
  or a browser page with no storage.
- **relay salt**: random for each process. That fits "ephemeral for now", and means two relays,
  or one relay across a restart, cannot link ids. A later `--identity-salt-file` keeps it.
- Each input is length-prefixed, so no two inputs collide by concatenation.

The id is an **identifier, not a credential**, as the device id is. The record says whether the
device behind it proved its key (`verified`), and notes the device tag. An admin can then see "the
same device on another network" even when the ids differ.

### 2.2 The registry (ID1)

The relay keeps one `client_registry`, a caller-owned object made in arisrv's `main()` beside the
gate, so there is no singleton. Its records are mirrored under one private root, outside `conf`:

```
arisrv.clients.<client_id>.
    username            the server's name for it (2.3)
    device              its room-scoped device tag, "" if unverified
    verified            whether its device proved its key
    first_seen          last_seen         (UTC, ISO 8601)
    active              whether any of its connections is open
    admin               whether a connection of it holds rcon now (2.5)
    limits.*            its caps when they differ from the defaults (2.6)
    connections.n<seq>.
        pid  transport  public  local  opened  closed  active     (geo.{country, region, city}: ID4c)
```

- **Private, with a test.** The root is outside the shard's interest and every `publish:`
  namespace, so it is never sent. A test joins a client and checks that nothing under
  `arisrv.` reaches it, in the live stream, the bootstrap or a snapshot reply. `room_write_gate`
  already refuses a client's write there. The server document doesn't see it either, unless a
  later part gives it a read-only link, as S0 does for `room`.
- **Bounded.** A record keeps its last 32 connections. The registry keeps at most 4096 records and
  drops the inactive one that was seen longest ago. Each cap is a flag. A client that reconnects
  over and over adds connections to one record, not new records.
- **Active flags.** A connection's entry is written at its Hello and closed when the transport
  reports it gone (`set_on_connection_closed`, applied in the room step where `room_leave` runs).
  `active` on the record is "any connection active".
- **Geographic info.** Empty by default, and never a network call on the relay loop. arisrvx gets
  an optional `@host.geolocate(address)` hook that returns `{country, region, city}` on the script
  thread, so an operator can use their own GeoIP database. Bundling one is left out: GeoLite2
  needs a licence key, and DB-IP Lite needs attribution.
- **The seam for other parts:**
  - `client_registry::identity_key(pid)`: the client id behind a pid, for this relay's records.
  - **#1290's gas ledger must not key on the client id.** Its salt is new each run, and a client
    with no device id is hashed on its addresses. The ledger keys on the principal, else on the
    verified device tag (`record_of(pid)->device`).
  - `client_registry::is_admin(pid)`: whether the connection holding that pid holds rcon (2.5).
    #1290's D10 uses it beside `admin_role`.
  - `client_registry::limits(pid)`: that client's overrides (2.6), or nullopt.
- **In Python** (ID4's side): `host.clients` is a read-only view (`host.clients[id]`,
  `host.clients.active()`), and `ctx.client` is the caller's record.

### 2.3 The username and self settings (ID1)

- `participants.<pid>.profile.name` becomes **relay-owned**: it goes on `room_write_gate`'s list
  beside `pose`, `net.relay`, `device` and `streams.<kind>.ready`. The relay writes it from the
  record's `username`, so the name peers see is the server's.
- `room_join` asks for a name instead of writing one. On a relay (a host with a room-call
  client for the room) it calls `server.set` with the name and `unique`, and writes nothing
  itself. Without one, in a mesh, it writes `profile.name` as before.
  - The relay checks the name: 1–32 bytes of UTF-8, trimmed, with no control characters. `unique`
    makes it unique among the active clients by adding `-2`, `-3`, and so on.
  - A client that asks for none is `guest-<first 4 hex of its id>`.
  - A returning client id keeps its last username. If an active client took that name meanwhile,
    it gets a suffix.
  - A Hello can't carry the name, because a host joins the relay before its document runs
    `room-join`.
- **A client that predates `server.set`** writes `profile.name` itself: a packaged ari or arigl,
  which cypkg's and nicotina's CI and users run until a release ships this.
  - The gate refuses that write, as it refuses any write to a relay-owned key. It also hands the
    value to the relay (`room_write_gate::set_name_requests`), which grants it on the room loop as
    a rename with `unique`, under the same checks and rate limit.
  - Such a client keeps its name.
  - Without bound writes there is no gate, so the relay writes no names at all, and they stay the
    clients' own, as before.
- **Built-in calls.** These are reserved `fn` names that `call_step` answers itself, before the
  Python dispatch, in stock arisrv as in arisrvx. No change to `room_calls.*` and no new client
  native is needed. A Python `@export` under `server.` is refused at load.
  - `(room-call "server.whoami")` returns `{client_id, username, verified, admin, limits}`.
  - `(room-call "server.set" (dict "username" "ana"))` returns the same, after the change.
    - Only keys on a fixed list may be set. The list starts as `username`, plus `unique` (true:
      take `ana-2` instead of a refusal).
    - A name another active client has, in any case, is `invalid`.
    - A client may rename at most 4 times a minute (`rate_limited`).
  - Any other `server.*` name is `not_found`.

### 2.4 The guest password (ID2)

- `arisrv --guest-password-file PATH` or `CVC_ARISRV_GUEST_PASSWORD`, never on the command line.
  Any UTF-8 text of 1–256 bytes, which a person can type.
- **It rides the bearer slot, hashed.** The client sends
  `hex(SHA-256("cvc-guest-password/1\0" | password))` as its bearer. That fits
  `valid_bearer_token`'s character set over both transports, keeps the plaintext off the wire and
  out of proxy logs, and needs no transport change. The relay compares it, in constant time, with
  the same hash of its own password, before the connection is admitted. A client without it is
  sent nothing, and the refusal is counted.
  - The hash is password-equivalent: someone who sees it can replay it. Use `grpcs://` or `wss://`
    when that matters, as for the room token.
  - The relay's origin is not hashed in. A relay can't know which of its names (a host, an IP or a
    proxy) the client dialed, so the hashes would not match.
- **As built, the relay expects the hash where it would expect a room token.** The transports'
  existing check does the comparing, so neither transport changed. A relay takes a room token or a
  guest password, not both: arisrv refuses to start with both.
- **When an `@host.authenticate` hook is set, the hook decides,** as it does for the room token.
  A program that wants both OIDC and a guest password checks the password in its hook. Giving the
  hook a helper for that (`host.guest_password_ok(cred)`) belongs to ID4c.
- **Clients:**
  - ari and the x hosts with `--relay` take it from `CVC_ARIADNE_GUEST_PASSWORD` when no
    `CVC_ARIADNE_ROOM_TOKEN` is set.
  - A relay join's `password: env:NAME` or `password: file:PATH` sets it. As with `token:`, the
    password itself is never written in the document, and a block has one or the other.
  - arigl reads `CVC_ARIADNE_GUEST_PASSWORD`. A browser page takes it from the invite link's
    fragment (`#password=…`, which never reaches a server) when its config's `fragment_env` names
    it: `"password": "CVC_ARIADNE_GUEST_PASSWORD"`.
- **Not yet:** changing the guest password while the relay runs. The transports take their token
  when they start, so rotating it means restarting the relay.

### 2.5 rcon (ID3)

- `arisrv --rcon-password-file PATH` or `CVC_ARISRV_RCON_PASSWORD`. Without one, rcon is off and
  every admin call is `forbidden`.
- `(room-call "server.rcon" "<password>")` checks it in constant time. On success, **the
  connection holding the caller's pid** is elevated until it closes. The elevation is keyed by the
  pid and the connection, not by the client id or the principal. A new connection must send it
  again, as asked, and a takeover of the pid by another link drops it.
  `(room-call "server.rcon-logout")` drops it early.
- Hardening:
  - Failures are rate-limited by public address and by client id. After 3 failures, `server.rcon`
    answers `rate_limited` for 30 s, and the wait doubles after each further one.
  - Each failure goes into the record's log. The password is never written to the tree, the logs
    or a call's progress.
  - A connection without TLS, from an address that is not loopback, is refused (`forbidden`)
    unless `--rcon-plaintext` is given. A Quake server sends rcon in the clear. We don't have to.
- **Admin calls**, built in. Each is `forbidden` unless the caller's connection is elevated. ID
  is a client id, or the pid one of its connections holds.
  - `server.clients`: every record, active first, then by when each was last seen.
  - `server.client` ID: one record, with its limits, its rcon failures and its connection log.
  - `server.kick` ID: closes that client's open connections (`state_transport::
    disconnect_connection`, by the number each link report carries), and answers how many.
  - `server.ban` ID / `server.unban` ID: a banned client's open connections are closed, and so is
    each new one as soon as the relay's next pass sees it. Kept in memory.
  - `server.rename` ID NAME: with no rate limit, but still unique.
  - `server.limits` ID {key value ...}: a client's caps (2.6). A `nil` value puts one back to the
    document's.
    - The keys are S3's and G4's: `programs` and `guests` (true or false; false refuses the
      client that kind), `steps` and `memory_mb` (whole numbers), and `time` (seconds). 0 is the
      document's.
    - They're kept by client id, and mirrored under `limits.*`. ID4a (#1296) and arisrv's
      `main()` make them apply (2.6), from the client's next program or guest.
    - There's no `fuel`: gas is charged as steps. There's no `concurrency`: one program or guest at
      a time per client is by design, and #1290's residents get their own count. ID4c adds
      `python`.
  - Rotating the guest password at run time waits on the transports (2.4).
- The rate limit is `rcon_guard`, beside `client_registry` in `client_registry.h`. Its lock
  starts at 30 s and doubles, up to an hour. A right password clears it.
- **In arisrvx**, while a connection is elevated, its `ctx.principal["roles"]` include `"admin"`.
  A program's own `@export(clients=True, roles=["admin"])` is then an admin command too, so an
  operator adds commands without touching the relay.

### 2.6 Each client's guest limits (ID4, the arisrvx session)

- **The seam, as #1296 builds it:**
  - `using ClientLimits = ClientPrograms`: `{on, max_steps, max_time, max_memory}`.
  - `enum class ServerDocument::ClientKind { program, guest }`. Python guests add a third kind.
  - `ServerDocument::set_client_limits(std::function<ClientLimits(const std::string &pid,
    ClientKind kind, const ClientLimits &document)>)`. The callback returns the whole caps: it
    starts from `document` and changes what the client's record sets. `on = false` refuses the
    client that kind (`forbidden`), and 0 is the document's.
  - arisrv's `main()` sets it from `client_registry::limits(pid)`, on the room loop, where
    `run_client_*` asks at each start. A change applies to the client's next start, not to one
    already running. A flag set false turns `on` off; `steps`, `time` and `memory_mb` (as bytes)
    replace the document's caps where they aren't 0. ctest `arisrv_client_limits` has an admin
    refuse a client programs, allow them again, and lower their steps.
  - A client's caps replace the document's for that client, lower or higher, up to
    `ariadne.limits.action_max_*` (D2; `ServerDocument::kClientLimitsAboveDocument` is true). A
    guest's memory stays under `wasm_max_memory`, and the action lane's caps apply on top.
- **Python guests in python.wasm** (D3, D4).
  - CPython can't be sandboxed in the relay's process, because a client's Python could reach `os`,
    `ctypes` and the interpreter itself. A WebAssembly build of CPython on G4's engine gets the same
    memory and gas caps, chroot, gate and ledger as any guest.
  - **It isn't W5's Python.** W5 builds CPython for `wasm32-emscripten`, which needs Emscripten's
    JS glue. G4's wasmtime runs WASI modules, so this needs a new build.
  - **The module is the server's.** It is far over G4's 1 MiB module cap, and a client shouldn't
    choose the interpreter. The document opts in with
    `server: client_python: {module, steps, time, memory_mb}`. A client sends source as data:
    `(room-run-opts (dict "language" "python") "source" arg ...)`.
  - **It needs a new cvcpkg recipe**, CPython for `wasm32-wasip1` built with wasi-sdk, speaking
    the ari ABI:
    - it exports `memory`, `ari_alloc` and `run`;
    - a small `_ari` C module converts CBOR to and from Python objects, so values stay typed;
    - the stdlib is frozen into the module, since guests get no files;
    - the interpreter is snapshotted after `Py_Initialize` (Wizer-style), so each run starts
      warm.

    This is the biggest risk in the plan, so it comes first within ID4.
  - CPython needs about 64 MiB of memory, above G4's 16 MiB default. That is one reason per-client
    limits come first.
  - Compute caps for Python are in gas. CPython's eval loop costs many instructions per bytecode,
    so a DSL-sized step cap is far too small.

## 3. Parts

| Part | What | Session | Files |
|---|---|---|---|
| ID1 | The client id, the registry and its private root, the username, `server.whoami` / `server.set`, `Hello.local_address`, each link report's addresses and connection number | the identity session (ID1-ID3) | `client_registry.{h,cpp}` (new, `conference/`), `state_local_address.cpp` (new), `state_transport.proto` + both transports, `state_link_stats.h`, `room_write_gate.*`, `room_intrinsics.cpp`, arisrv `main.cpp` |
| ID2 | The guest password | the identity session | `state_identity.{h,cpp}` (`valid_guest_password`, `guest_password_bearer`), arisrv `main.cpp`, `relay_session.{h,cpp}` (`password:`, `CVC_ARIADNE_GUEST_PASSWORD`), arigl `main.cpp`, `arigl-web.html`'s config notes |
| ID3 | rcon and the admin calls | the identity session | `client_registry.{h,cpp}` (elevation, ban, limits, `rcon_guard`), `state_transport*` (`disconnect_connection`, a link report's `secure`), arisrv `main.cpp`'s `call_step` |
| ID4a | Per-client limits (#1296) | "arisrvx S1: Python engine" | `server_document.{h,cpp}` |
| ID4b | The python-guest module (cvcpkg recipe) | "arisrvx S1: Python engine" | the cvcpkg recipe, `src/ariadne_wasm/*` |
| ID4c | The Python route and `host.clients` / `ctx.client` / `@host.geolocate` / `host.guest_password_ok` | "arisrvx S1: Python engine" | `server_document.*`, `room_calls.*` if a `language` field is needed, `ariadne_server.py`, `python_engine.cpp`, `server_link.{h,cpp}` |

- arisrv `main.cpp` is the identity session's in every part. Whatever ID4 needs wired there, it
  wires: the `set_client_limits` callback, the call event's `client` beside `principal`,
  geolocate requests, and `ServerLink::set_guest_password_bearer`. Python never sees the expected
  bearer: `host.guest_password_ok` compares in C++.
- Both sessions add ctests to `src/arisrv/CMakeLists.txt`, each in a block of its own. Whoever
  lands second merges.
- **Order.** ID1 first, then ID2 and ID3; ID4a can go beside ID1. ID4 stacks on #1283, and
  everything rebases onto master once #1283 lands.

## 4. Decisions (Joe, 2026-11-16)

- **D1. The addresses go in the log, not the client id.** He said the same to both sessions: "if
  no device ID then lean on IP for identity hash. but add it to the log either way". In the hash,
  a laptop on a new Wi-Fi network, or a phone moving between Wi-Fi and cellular, would be a new
  client. It would lose its
  username and, under #1290, get a fresh gas grant to farm. The id is the device id, and the
  addresses are the fallback only for a client that sends none. A browser's random id can be
  farmed either way, by clearing its storage.
- **D2. A client's limits may go above the document's caps,** up to the ceiling
  (`ariadne.limits.action_max_*`). The document's caps are the default, and an admin can raise or
  lower one client's.
  - His first answer to the arisrvx session read the other way: "no, the server should have a
    higher default to accommodate the biggest clients. there should be separate client limits
    possible too".
  - Asked again, he confirmed it to that session: "Yes, up to ceiling", as he had told the identity
    session.
  - #1296 sets `kClientLimitsAboveDocument = true`. Python needs this, since CPython wants about
    64 MiB.
- **D3. Clients' Python runs in python.wasm** on G4's engine (2.6). It does not run as in-process
  CPython.
- **D4. Python in wasm replaces S4** (RestrictedPython in worker processes) as tier 3 of the
  scripting hosts' §6.3.
