# Ariadne resident guests: clients' code that stays on the server, and the gas that pays for it

> **Status: PLANNED (2026-11-15).** Nothing here is built. It builds on G4's first part,
> transfix/libcvc#1283 (clients' one-shot WebAssembly guests on `arisrvx`). Parent roadmap:
> [`ARIADNE_SCRIPTING_HOSTS.md`](ARIADNE_SCRIPTING_HOSTS.md) (§8.7 points here). The decisions in
> §11 are open.

## 0. The ask

The owner's request, server side first:

1. **Hooks.** A client's guest can hook the relay's events, as a document's guests already do.
2. **A resident guest.** A client starts a guest that stays. By default it lasts as long as the
   client is connected.
3. **Lifetimes past the client.** A guest can stay after its client leaves:
   - until the client reconnects later and closes it;
   - until an admin kills it;
   - or until a timeout runs out. The timeout can change, and a reconnect within it can reset it.
4. **A gas ledger.** Each client has a balance of gas on the server, kept by its hashed identity.
   Running guests spends it, including guests left running while the client is away. Balances
   refill at a set rate, and an identity can get a starting grant the first time it connects.
5. **Guests on clients** (from the same discussion), in native clients and in the browser, without
   needing wasm-mt's threads.

## 1. What exists

Checked against master f2593e7a, plus #1283.

| Part | Today | Where |
|---|---|---|
| One-shot client guests (G4, first part) | `(room-run module arg ...)` runs one export as the client's S3 program: chrooted to `participants.<pid>`, the read lane's builtins plus five gated writers, `steps`/`time`/`memory_mb` caps, a fresh instance per run. The module compiles on the app's pool and the relay holds the run until it's `ready()`; the last 16 modules stay compiled. | `ServerDocument::run_client_guest`, `wasm_engine.cpp` `ClientGuest` |
| Hooks for a document's guests (G1) | `on_tick`, `on:`, `watch`, `on_key`/`on_pointer`/`on_signal`, `msg_recv`, each a resident activation. One activation per instance at a time, under the lock `wasm:<stem>`. Residents restart with backoff (§8.7). | `WasmEngine::load_block`, ARIADNE_SCRIPTING_HOSTS.md §8.7 |
| Gas | The gas pass (`meter.h`) counts the same in every runtime. `ariadne.limits.wasm_gas_per_step` is 1000 gas a step. On wasmtime, fuel is the backstop at 16 units per unit of gas. | `meter.h`, `wasm_engine.cpp` |
| Principals (S5) | arisrvx's `authenticate` hook maps a credential to a principal (`id`, plus `roles` and the rest of its dict). The gate binds each pid to its principal, and only the same principal can take over a silent pid. `host.disconnect(id)`. Stock arisrv has no authenticator: its device id is an identifier, not a credential. | `state_inbound_gate.h`, §6.5.4 |
| Read scope (S6) | `state_transport::set_read_scope` per principal: what each client is sent. | §6.5.6 |
| Departure | A closed connection is queued, and the next room step's `room_leave` retracts the participant. A silent holder's pid passes, after 6 s, to the next claim of the same principal. | `src/arisrv/main.cpp`, the gate |
| The write gate | `room_write_gate`: only the writer's own `participants.<pid>`, minus `pose`, `net.relay` and `streams.<kind>.ready`; canonical paths of at most 16 segments and 512 bytes. | `room_write_gate.h` |
| Persistence | None. arisrv keeps nothing across a restart (`--relay-journal` is the replication journal, not storage), and §6.5.9 says durable work belongs in the application's database. | — |
| A budget of work | None. The per-principal limits count calls (`--call-concurrency`, `--call-rate`), not the work they do. | §6.5.4 |

So the pieces for running a client's code safely exist: the chroot, the gate, gas, caps and
hooks. What's missing is anything that **stays**: an instance kept between calls, an owner who
can leave and come back, and an account that pays for the time in between.

## 2. The model

A **resident** is a guest instance that the relay keeps.
- **Owner:** its owner is a principal, kept by hash. A resident lives in one room.
- **Firings:** its hooks fire activations on that one instance. Each firing is a capped run, like
  a G4 call, and the ledger debits its owner's balance for it.
- **Lease:** its lease says when it ends: with its owner's connection (the default), some seconds
  after its owner was last connected (a reconnect resets that), or only when it is closed.
- **Home:** it writes its own subtree, `conf.rooms.<room>.guests.<gid>`, under the same gate rule
  as a participant's. The room sees that subtree as it sees a participant's.

A resident is like a headless participant that its owner brought into the room. That makes room
apps possible: a referee, a scoreboard, a non-player character, a bot that keeps a shared
document tidy. Other clients interact with it the way they interact with any participant, through
state it watches and state it writes.

## 3. RG1: residents with hooks, bound to the connection

### Starting one

```lisp
(define referee
  (room-guest-start (referee-bytes)
    (dict "name"  "referee"
          "hooks" (dict "on_tick" (dict "export" "tick" "every" 6)   ; every 6th room step
                        "watch"   (dict "room.participants" "roster")
                        "msg"     (dict "cmd" "command")
                        "timer"   (dict "every" 60 "export" "minute")
                        "close"   "goodbye"))))
(room-guest-send referee "cmd" (dict "op" "reset"))   ; to its msg hook, on channel "cmd"
(room-guest-call referee "status")                    ; one export now, as room-run calls one
(room-guests)                                         ; mine in this room: id, name, state, lease, gas used
(room-guest-stop referee)
```

```yaml
server:
  client_guests: { steps: 5000 }           # G4's one-shot calls, as now
  residents:
    per_principal: 4                       # live residents per owner
    memory_mb: 16                          # each; and 64 MiB per owner in all
    firing: { steps: 2000, time: 0.005 }   # the caps for each firing
    tick_every: 6                          # the fastest on_tick allowed: every 6th room step (10 Hz)
```

- **On the wire.** These calls go on `room-call`'s path, as `room-run` does. `room-guest-start`
  returns the resident's id (`gid`), which the relay assigns. The id is a valid state name.
- **Setup.** The module compiles as G4's do: on the pool, held until it's `ready()`, from the same
  module cache. `_initialize`, then `ari_init` if the module exports it, run once at the start under
  the firing caps. A failure there fails the start.
- **The instance stays.** One instance serves all of a resident's firings, so its linear memory
  and globals are its state. Only one firing runs at a time, under the lock `guest:<gid>`. Other
  firings queue behind it, as a document guest's hooks wait under `wasm:<stem>`.

### Hooks

| Hook | Fires on | Its argument | Coalescing |
|---|---|---|---|
| `on_tick` | every `every`-th room step, never more often than `tick_every` allows | `{frame}` | one pending at most |
| `watch` | a change under a path the resident may read | `{path, value}` | the latest value for each path |
| `msg` | `(room-guest-send gid channel value)` from its owner | `{channel, value}` | a queue of at most 64; overflow drops the oldest and counts it |
| `timer` | `every` seconds, or once `after` seconds; it can rearm itself (`guest-wake-in`) | `{at}` | one pending at most |
| `owner` | its owner's connection comes or goes (RG3) | `{kind: join or leave, pid}` | in order |
| `close` | its end, unless an admin killed it or it ran out of gas | `{reason}` | once, under the firing caps |

A hook names an export, or a dict `{export, every, after}`. The set follows a `wasm:` block's
`hooks:` (§8.7):
- `on_tick` and `watch` are the same.
- `msg` is `msg_recv`, on channels of the resident's own.
- The input hooks are left out, because a relay has no keys or pointer.
- `timer`, `owner` and `close` are new.

Once a resident is started, its set of hooks is fixed. To get different hooks, a client starts
another resident.

### What it reaches

- **Functions.** It gets S3's client environment: the read lane's builtins and the gated writers.
  On top of that it gets four natives of its own: `guest-gas` (its owner's balance),
  `guest-lease` (its lease), `guest-wake-in` (arm its timer) and `guest-stop` (end itself). It
  can't reach `spawn`, the running natives, or anything that installs a hook.
- **Writes.** It writes only its own subtree, `conf.rooms.<room>.guests.<gid>`. The gate gets one
  rule for that: a resident is held to the participant rule, with its own subtree in place of
  `participants.<pid>`, minus `status`, which the relay owns (D1).
- **Reads.** It reads its own subtree, plus what its owner is sent, under the owner's S6 read
  scope, through a read-only `room` link (D2). So a resident never sees more of the room than its
  owner could.
- **Limits.**
  - Each firing runs under the `firing:` caps. Its memory is capped at `memory_mb`.
  - An owner may hold at most `per_principal` residents, using at most 64 MiB in all.
  - A resident's subtree is capped too, in bytes and in nodes, so a resident left running can't
    grow the room's state without limit.

### Scheduling

Residents run on the relay's drain, as a document's residents do. Their firings share one budget
for each room step, so a room full of residents can't stall the room loop: when the budget runs
out, the remaining firings wait for the next step, taken round robin by owner. The pump budget
already counts what slices charge (G1's fifth part), and this budget is a cap within it.

### Status

The relay writes `conf.rooms.<room>.guests.<gid>.status`, which only it may write:
- `name`;
- `state`: starting, running, lingering, starved or ended, with the reason when it ended;
- `owner`: its owner's pid while the owner is connected, and empty otherwise;
- `until`: when its lease ends, if it is set to end;
- `used`: the gas it has spent.

The status never carries the owner's principal id or balance.

### How it ends in RG1

A resident ends when its owner stops it, when it stops itself, when its owner's connection
leaves, or when it keeps failing. A resident that keeps failing restarts with the same backoff as
§8.7 (after 1, 2, 4 … 64 drains, one drain per room step) and ends after 8 such stops in a row. Each ending fires `close`.
Then the relay removes the resident's subtree, all but its final `status`, which it removes a
minute later.

## 4. RG2: the gas ledger

```yaml
server:
  ledger:
    grant: 100000000                     # gas the first time a principal is seen (100k steps)
    refill: { rate: 100000, max: 1000000000 }   # gas a second, up to max
    fee: 1000                            # gas for each firing, on top of its work
    rent: 0                              # gas for each MiB-second a resident holds memory
    store: residents.ledger              # beside the document; arisrvx --ledger overrides it
    flush: 5                             # seconds between writes to the store
    starved_max: 3600                    # seconds a resident may stay starved before it ends
```

- **The unit is gas.**
  - A WebAssembly firing costs the gas it used, plus `fee`. The fee covers the host's fixed cost
    for each firing, which a firing that does almost nothing still has. A DSL program costs its
    steps × `wasm_gas_per_step`, so both pay the same per step.
  - `rent` charges for memory held between firings. It is off by default (D8).
- **Accounts are kept by hash.** The ledger's key is SHA-256(salt ‖ principal id), with a salt the
  server keeps beside the store, so the store holds no raw principal ids. An admin who knows an id
  can look up its account, because the relay can hash the id again.
- **Grants and refill.**
  - An account the ledger hasn't seen before starts at `grant`.
  - Refill is a token bucket worked out when the balance is read:
    `balance = min(max, balance + rate × elapsed)`. So nothing has to tick for every account.
  - Either one can be 0. With both at 0, the operator credits accounts by hand.
- **Reserve, then settle.**
  - Before a firing starts, the ledger reserves the gas the firing's caps allow, or the whole
    balance if that is less. So a firing's real cap is the smaller of its caps and the balance,
    and the account can never go negative.
  - When the firing ends, the ledger debits what it used and returns the rest.
  - The same reservation covers `_initialize`, `ari_init` and `close`.
- **Who pays.** The owner pays, connected or not. While a ledger is set, G4's one-shot calls and
  S3's programs are charged the same way, so a client can't avoid the ledger by sending one-shots
  (D6).
- **Running out.** A resident whose owner can't pay for its next firing is **starved**. Its hooks
  don't fire, but their events still coalesce as above. It wakes once the refill covers a firing,
  and ends after `starved_max` (an hour by default) (D7).
- **Seeing a balance.** A client asks with `(room-gas)`: balance, refill rate and cap, and gas spent
  in the last hour. A resident asks with `guest-gas`. The balance isn't written anywhere in the
  replicated tree.
- **The store.** It is an append-only CBOR journal: grants, debits as one total for each flush,
  credits and lease records. It is compacted when the relay starts.
  - Debits are flushed every `flush` seconds and at a clean exit. A crash therefore loses at most
    that window's debits, which favours the user.
  - Credits and grants are written at once.
  - The ledger is an object the `ServerDocument` owns, with the store behind an interface.
    arisrvx can hand in its own store (an application's database) in place of the file. It is
    never process-wide state.
- **Without principals.** A relay with no authenticator has no accounts to keep: stock arisrv, or
  arisrvx with no `authenticate` hook. There, residents run under their firing caps alone, and
  are bound to the connection (D5).

## 5. RG3: leases past the connection, closing and killing

```lisp
(define referee
  (room-guest-start (referee-bytes)
    (dict "name" "referee" "hooks" (dict "on_tick" "tick") "lease" (dict "linger" 600))))
(room-guest-set referee (dict "lease" (dict "linger" "forever")))   ; until I close it
(room-guest-set referee (dict "lease" (dict "linger" 60 "ttl" 86400)))
```

```yaml
server:
  residents:
    linger: { default: 10, max: 86400, forever: true }   # seconds; forever: until it is closed
    ttl_max: 604800                                      # no resident lives longer than a week
    admin_role: admin                                    # this role may list and stop any resident
```

- **The lease.**
  - `linger` is how many seconds a resident stays after its owner's last connection leaves.
  - The default is a reconnect grace of 10 seconds, not 0, so a network blip doesn't end a
    resident (D3). `forever` keeps it until it is closed.
  - `ttl` is an optional limit from its start, whatever the owner does.
  - The server's `max` and `ttl_max` cap both.
- **The countdown.**
  - It starts when the last connection of the owner's principal leaves the room. The resident
    gets an `owner` event `{kind: leave}`, and its state becomes `lingering`.
  - If the same principal connects again before the countdown ends, it stops: the reset the owner
    asked for. The next departure starts it again from the full `linger` (D4).
- **Changing it.** The owner changes the lease with `room-guest-set`, within the server's limits.
  A resident can lower its own, but never raise it.
- **Reattaching.** An owner is recognized by principal, not by pid, so a reconnect under a new
  pid still owns its residents.
  - `(room-guests)` lists them again.
  - Each gets `{kind: join, pid}`, and its `status.owner` shows the new pid.
  - If two connections of the same principal are in the room, the resident's owner is connected
    while either one is.
- **Closing.** A resident ends when:
  - its owner calls `room-guest-stop`;
  - it calls `guest-stop`;
  - its lease or `ttl` ends;
  - it stays starved past `starved_max`;
  - an admin stops it;
  - the relay shuts down.

  Each ending records its reason in `status`, and in the store, so an owner who comes back after
  a day can see why the resident is gone.
- **Admins.**
  - A principal whose dict has `admin_role` among its `roles` (from the `authenticate` hook) may
    list every resident in a room, stop any of them, and read and credit any account
    (`room-guests-all`, `room-guest-stop` on another's id, `room-gas-credit`).
  - arisrvx's operator code gets the same through `host.guests()`, `host.guest_stop(gid, reason)`
    and `host.ledger`.
  - An admin's stop skips `close`.
- **A relay restart ends every resident** ("relay restarted"). Their lease records stay in the
  store, and accounts keep their balances. No instance survives a restart until RG5.
- **Lingering needs a principal.** Lingering past the grace and `forever` both need an
  authenticated principal (S5). A device id is no proof that the same client came back (D5).

## 6. RG4: DSL residents

S3's programs get the same leases, hooks, ledger and status:
`(room-guest-start "source" (dict "hooks" (dict "on_tick" "tick" ...)))`.
- The program's top level runs once at the start.
- Each hook names one of its `defun`s, which runs as a firing on the process's kept environment.
- It costs steps × `wasm_gas_per_step`.

This needs no wasmtime, so stock arisrv can run it. With no authenticator there, it is bound to
the connection (D5).

## 7. RG5: hibernation, and residents across a restart

Between firings, a resident has no stack. Its state is its linear memory, its globals and its
table. RG5 writes that state to disk once a resident has been idle for a while, frees the memory,
and wakes the resident on its next timer, watch or message. The same snapshot lets lingering and
`forever` residents survive a relay restart.

The module's bytes are kept by hash beside the snapshot. The snapshot is the relay's own data:
it is never accepted from a client, and never sent to one. With hibernation, `rent` can charge
less for disk than for memory.

## 8. RG6: guests on clients

**Who sends the module.** First, only the client's own relay: the server document publishes
modules for its clients to run, for example the client side of a resident's app. These are
trusted as much as the relay's documents are. Modules from peers come later, behind consent and
the receiving client's own caps.

**Native clients** have threads. The same `ClientGuest` runs there: it compiles on the app's pool,
waits for `ready()`, and wasmtime's fuel is the backstop. Native clients have no wasm-mt question.

**The browser** needs no threads either. What it needs is metering:

- **Compiling.** `WebAssembly.compile(bytes)` returns a Promise, and the browser compiles the
  module in the background on its own. That needs no SharedArrayBuffer, so no COOP/COEP headers
  and no wasm-mt.
  - `ClientGuest`'s browser branch starts the compile, and the Promise's `.then` sets a flag that
    `ready()` reads. The host doesn't unwind its stack to wait, so this needs no Asyncify in the
    host either. That is unlike the document load's `ASYNC_LOAD`.
  - Today that branch compiles synchronously, with `new WebAssembly.Module`. That works below
    Chrome's 8 MB limit, but it blocks the frame while it compiles.
- **Metering.** The page runs only guests that `cvc_ariadne_wasm_transform` metered and
  Asyncify-transformed at install, and a module that arrives at run time hasn't been through that.
  The sender's own transform can't be trusted: a forged module can declare `ari.gas` and never
  call it. So the page runs the gas pass itself. `meter.cpp` already builds under Emscripten, as
  `ari_wasm_meter` does in node.
- **Running.** Without Asyncify a guest can't pause, so a client guest runs to its end within one
  frame, under a small gas cap. It traps, instead of pausing, when its gas runs out. Host calls
  that don't wait work as they do today. `yield`, and calls that would wait, are refused. That
  fits a one-shot call or a hook's firing.
- **Writes.** A client guest writes the client's own tree, under its document's lane. Anything
  that replicates passes through the relay's gate as the client's own writes do.

## 9. RG7: the Worker backstop (G4's second part)

A Worker is a second thread that needs no SharedArrayBuffer, and `terminate()` stops a runaway
guest with no metering at all. But the guest's host calls become `postMessage` round trips. A
guest that calls the host must then pause until the answer comes, through JSPI or Asyncify in the
Worker. The other way is to block on SharedArrayBuffer and `Atomics.wait`, which brings back
COOP/COEP. So the Worker suits guests that compute and call the host rarely, and it is the hard
stop beneath RG6's metering.

## 10. Phases

| Phase | What | Needs | Size |
|---|---|---|---|
| RG1 | Residents bound to the connection: `room-guest-start`/`-stop`/`-send`/`-call`, `room-guests`; the hooks of §3; the instance kept across firings; the `guests.<gid>` subtree with its gate rule and status; the per-step budget; per-principal limits | G4's first part (#1283) | M |
| RG2 | The gas ledger: accounts by salted hash, grant, refill, fee, reserve and settle, starving, `(room-gas)`, the journal store behind an interface; G4's and S3's calls charged too | RG1, S5 | M |
| RG3 | Leases: `linger` (a 10 s grace by default), `forever`, `ttl`; the countdown and its reset; reattaching by principal; the `owner` hook; closing reasons; the admin role and arisrvx's `host.guests`/`host.ledger` | RG2 | M |
| RG4 | DSL residents on the same leases, hooks and ledger | RG1–RG3, S3 | S |
| RG5 | Hibernation of idle residents, and lingering residents across a relay restart | RG3 | L |
| RG6 | Guests on clients: sent by the relay's document; native on the pool; in the browser, compiled by Promise, metered in the page and run to their end | G2 | M |
| RG7 | The Worker backstop in the browser (G4's second part) | RG6 | M |

RG1 through RG3 are the owner's ask, in order. RG4 can run beside RG2. RG6 and RG7 don't depend
on the server phases.

## 11. Decisions (open)

| # | Question | Recommendation |
|---|---|---|
| D1 | Where a resident writes | **Its own `guests.<gid>` subtree only.** Writing its owner's `participants.<pid>` while that connection lives can come later, if an app needs it |
| D2 | What a resident reads | **Its own subtree plus what its owner is sent** (the owner's S6 read scope) |
| D3 | The default lifetime | **The owner's connection plus a 10 s reconnect grace**, not the connection alone, so a blip doesn't end it |
| D4 | What a reconnect resets | **The countdown from the owner's last departure stops on a reconnect**, and starts again from the full `linger`. An optional `ttl` from the start caps it whatever the owner does |
| D5 | Lifetimes past the grace without an authenticator | **Not allowed.** On a relay with no authenticator (stock arisrv, or arisrvx with no `authenticate` hook), residents are bound to the connection, with no ledger |
| D6 | Whether G4's one-shots and S3's programs pay too | **Yes, whenever a ledger is set** |
| D7 | Out of gas | **Starve, don't kill**, and end after `starved_max` (1 h) |
| D8 | The cost of idling | **`rent` 0 by default**, with caps on count and memory per owner. Hibernation (RG5) makes idling cheap later |
| D9 | Who gets grants | **Authenticated principals only**, plus a server-wide cap on the gas grants create each hour, against identity farming |
| D10 | Who is an admin | **A role from the `authenticate` hook (`admin_role`)**, plus arisrvx's operator code |
| D11 | Whether other clients may call a resident's exports (a resident as a service) | **Later.** Until then they interact through state it watches and state it writes |
| D12 | Where the ledger is kept | **A file journal in libcvc behind an interface**, which arisrvx can replace with the application's database |

## 12. Risks and checks

1. **Identity farming.** Grants, and refill on many accounts, can be farmed with cheap
   identities. In the browser, a device id is a random value in localStorage. So D9 needs real
   principals, and the hourly cap on grants is the backstop.
2. **The room loop.** Many residents share one relay's drain. Measure the per-step budget at 100
   and at 1000 residents, against the room step's 16 ms, before choosing its default.
3. **Memory while no one is connected.** A `forever` resident holds up to `memory_mb` with nobody
   in the room. The per-owner caps bound this, and RG5 fixes it.
4. **Calibration.** `fee`, `rent` and the example grants depend on what a step costs
   (ARIADNE_SCRIPTING_HOSTS.md open question 14). Measure first, then set the defaults.
5. **The store.** If the salt is lost, every account is lost with it, so it lives beside the
   journal and is backed up with it. A crash loses at most `flush` seconds of debits.
6. **Clocks.** Refill and leases persist across restarts, so the store keeps wall-clock times,
   and a clock that jumps forward refills accounts. Clamp each refill to `max`, and log jumps.
7. **Restart semantics.** Until RG5, a restart ends every resident. Owners see the reason, but a
   long-running app has to keep its own state in its subtree, or in its owner's application.
8. **Privacy.** Balances and principal ids stay out of the replicated tree. Admins see hashed
   accounts, and names only through the `authenticate` hook's dict.
