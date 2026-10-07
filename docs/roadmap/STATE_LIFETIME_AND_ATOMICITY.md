# cvc::state — Cross-Node Lifetime Safety & Multi-Node Atomicity

*Design plan for closing the cross-node use-after-free in `cvc::state` and giving
callers a general path to multi-node-atomic reads/writes — without every subsystem
reinventing its own mutex.*

Status: **design / not yet implemented.** This doc is for review before any code lands.
Every `file:line` below was verified against the tree at merge of PRs #481/#482
(`inc/cvc/state/state.h`, `src/cvc/state/state.cpp`, and the cited call sites).

---

## Table of Contents

- [1. Background: the gap](#1-background-the-gap)
- [2. Two distinct problems](#2-two-distinct-problems)
- [3. Phase 1 — Option A: owning handles (lifetime/UAF)](#3-phase-1--option-a-owning-handles-lifetimeuaf)
- [4. Phase 2 — atomicity on top of A](#4-phase-2--atomicity-on-top-of-a)
- [5. Phase 3 — Option C: deferred-reclamation safety net](#5-phase-3--option-c-deferred-reclamation-safety-net)
- [6. Options considered and rejected](#6-options-considered-and-rejected)
- [7. Sequencing, PRs, and test/CI plan](#7-sequencing-prs-and-testci-plan)

---

## 1. Background: the gap

`cvc::state` is a hierarchical shared-state tree. Each node owns a
`boost::mutex _mutex` (`state.h:743`) and locks it for every value/data/child
operation — so **per-node access is thread-safe**, and the header says so
("Written to be thread safe", `state.h:137`).

The hazard is *cross-node*. Children are owned by
`typedef std::map<std::string, boost::shared_ptr<state>> child_map _children`
(`state.h:152-153`), so a node dies exactly when the last `shared_ptr` to it drops.
But the two primary accessors hand back **non-owning** references:

- `state &operator()(const std::string &childname)` (`state.h:415`) → bare `state&`
- `state *findDescendant(const std::string &path)` (`state.h:606`) → bare `state*`

The owning `shared_ptr` lives only in the parent's `_children`. So a concurrent
`sweepExpired()` — the sole production node-freeing path (`_children.erase` at
`state.cpp:1365`, last-owner drop at `state.cpp:1378`) — can free a node while
another thread still holds the bare `state&`/`state*` → **use-after-free**.

This is not theoretical: it is the class of the intermittent SEGFAULT that a
concurrent cache test surfaced. Today the only mitigation is that individual
subsystems (e.g. the HTTP cache, `uri_http_cache.cpp:16-24`) wrap *all* structural
access to their subtree in a bespoke process-wide mutex. The goal here is a
**general** fix so callers don't each have to do that.

The concrete cross-thread hazard that actually crashes: a compute-pool worker
running `(fetch "state://…")` → `resolve()` → `state_resolve` (bare `findDescendant`
+ `value()`/`data()` at `uri_state.cpp:60,63,70,78`, over a bare `root` captured at
`:103`) racing a tree-wide `sweepExpired()` fired on the scheduler thread
(`intrinsics.cpp:214-218`) or the pycvc thread (`pycvc_state.cpp:81`).

## 2. Two distinct problems

Keep these separate — they need different mechanisms, and conflating them is the
main source of confusion:

1. **Lifetime / UAF** — "the node I'm touching won't be freed under me."
2. **Multi-node atomicity** — "I read/write a *set* of related nodes as a consistent
   unit." The cache entry is 7 metadata children + a body blob; `write_entry`
   commits them across 7 separate node locks with the body last as the signal
   (`uri_http_cache.cpp:308-323`), and `read_entry` relies on "body present ==
   committed" (`:271-289`). A reader can otherwise observe `status = new, body =
   old`: a **torn read**.

**No lifetime fix provides atomicity.** Keeping nodes alive (Option A or C) does
nothing to stop a writer mutating live nodes in place while a reader reads them.
Atomicity is a *third* thing (§4). The value of getting the lifetime fix right first
is that it is the **substrate** that makes the efficient (lock-free-read) atomicity
design safe.

## 3. Phase 1 — Option A: owning handles (lifetime/UAF)

**Goal:** a general, opt-in way to hold a node alive across concurrent structural
mutation, replacing the need for a bespoke per-subsystem mutex *for lifetime*.

### 3.1 Why A is low-risk here

Every live `state` is already `shared_ptr`-owned — verified: the only two births are
the root (`state.cpp:154`, `ptr.reset(new state(ctx))`, stored on the app data map)
and a child inside `operator()` (`state.cpp:778`); the ctor is `protected`
(`state.h:738`), the copy ctor is `private`/undefined (`state.h:774`), there are **no
subclasses**, and the ctor body uses raw `this` only (`state.cpp:95-96`). So adding
`enable_shared_from_this` cannot hit an unowned instance, and `shared_from_this()` is
never reachable mid-construction.

### 3.2 API

```cpp
// inc/cvc/state/state.h
// MUST be the boost variant: ownership is boost::shared_ptr (state.h:152, state.cpp:154),
// which only populates a boost::enable_shared_from_this weak backpointer. std:: would
// leave it empty and shared_from_this() would throw bad_weak_ptr. (Distinct from
// cvcGL's SceneNode, which is std::shared_ptr / std::enable_shared_from_this.)
class state : public boost::enable_shared_from_this<state> {
  // ...

  // Owning analogue of findDescendant (state.h:606): null state_ptr when any
  // segment is absent. The returned node is PINNED — a concurrent sweepExpired()
  // can only unlink it, never free it, while the pin is held.
  state_ptr findDescendantShared(const std::string &path);

  // Owning analogue of operator()(name) (state.h:415): create-or-get, returns the
  // pinning state_ptr rather than a bare state&.
  state_ptr sharedChild(const std::string &childname = std::string());
};

// Thin RAII sugar (header-only): a handle that reads like a node but pins it.
class state::handle {
  state_ptr _p;
public:
  handle() = default;
  explicit handle(state_ptr p) : _p(std::move(p)) {}
  state *operator->() const { return _p.get(); }
  state &operator*()  const { return *_p; }
  state *get()        const { return _p.get(); }
  explicit operator bool() const { return static_cast<bool>(_p); }
};
```

`findDescendantShared` mirrors `findDescendant` (`state.cpp:980-1000`) but copies the
map's `shared_ptr` at each hop instead of `.get()`, so the returned node is owned:

```cpp
state::state_ptr state::findDescendantShared(const std::string &path) {
  std::string normalized = normalize_state_path(path);
  if (normalized.empty()) return shared_from_this();      // the one spot needing esft
  std::vector<std::string> keys; /* split as in state.cpp:986 */
  state_ptr cur = shared_from_this();
  for (auto &k : keys) {
    boost::algorithm::trim(k);
    if (k.empty()) continue;
    boost::mutex::scoped_lock lock(cur->_mutex);
    auto it = cur->_children.find(k);
    if (it == cur->_children.end() || !it->second) return state_ptr();
    cur = it->second;                                     // copies the OWNING shared_ptr
  }
  return cur;
}
```

`sharedChild` is `operator()` (`state.cpp:739-785`) returning the local `state_ptr`
`child` (already minted at `:771,776,778`) instead of `*child`.

### 3.3 The ancestor fix (MANDATORY — a leaf-pin alone is not enough)

Adversarial review found that pinning only the *addressed* node is insufficient for
the flagship resolver, because the resolver walks the **`_parent` chain**, which the
leaf-pin does not keep alive:

- `state_resolve` computes `"state://" + eff->fullName() + …` on **every** resolve
  (`uri_state.cpp:64`; `state_store` at `:97`). `fullName()` → `parentName()`
  (`state.h:227-239`) dereferences `_parent` (a raw `const state*`, `state.h:747`),
  recursing up the chain.
- `effective_node`/`resolveLink` also call `fullName()` on the start and target
  nodes (`state.cpp:1016,1047`).

Failing scenario: worker pins leaf `s.item.k`, then a tree-wide sweep erases the
*intermediate* `s.item` from `s`'s map (`state.cpp:1365`) and drops it (`:1378`);
`k->_parent` now dangles and `k->fullName()` UAFs — the exact bug we set out to kill.

`_parent` cannot simply become a `shared_ptr` — that would create a parent↔child
cycle and leak the whole tree. Two acceptable fixes (choose per site):

- **(a) Pin the ancestor chain** in the resolver: have `findDescendantShared`
  optionally return the vector of hop `state_ptr`s, or keep the root pin plus each
  intermediate, so the whole path stays alive for the caller's use.
- **(b) Don't recompute `fullName()` after a possible concurrent sweep** — build the
  canonical string from the already-normalized *input* path (which the resolver
  already has) instead of walking `_parent`. Apply the same to `resolveLink`'s
  internal `fullName()` calls.

Preference: **(b) for the resolver** (cheaper, removes the walk entirely) plus the
internal `resolveLink`/`resolveRemote` walker returning a pinned target
(`link_resolution { state *target; state_ptr target_owned; … }`, populating
`target_owned` by switching the internal `root.findDescendant(target)` at
`state.cpp:1038,1104` to `findDescendantShared`). Document that a raw handle pins a
*node*, not its ancestor chain.

### 3.4 Blast radius / migration (~12 sites, incremental)

Adding the API is inert. Migration is opt-in per subsystem; the verified census
(41 production `findDescendant` callers, 82 total) narrows to:

- **Must migrate (P1):** `uri_state.cpp:60,63,92` (resolver/store on a pool worker) +
  the internal `resolveLink`/`resolveRemote` pinned walker (ships with the API).
- **Migrate if ever concurrent:** `state_distributed_admin.cpp:261,364` (bare `n`
  across `isLink()`/`linkTarget()`/`linkMode()`).
- **Defensive (Python thread):** `pycvc_state.cpp:38,51,75` (`:45` is nullptr-only,
  already safe).
- **Keeps its own lock regardless:** `uri_http_cache.cpp` — Option A removes only the
  *lifetime* reason for `cache_mutex`; it must keep it for atomicity until Phase 2.
- **Do not need it:** the ~16 `intrinsics.cpp` read sites, `scheduler.cpp`,
  `async_scheduler.cpp`, `state_value_codec.cpp`, `ariadne.cpp` — all on the single
  scheduler/frame thread whose only sweeper runs on that same thread (see the
  single-thread-per-root premise, §7); and all test callers.

No flag-day: land API → migrate `uri_state` (closes the SEGFAULT) → migrate the rest
opportunistically.

### 3.5 What Phase 1 explicitly does NOT do

- Not atomicity (§4).
- Not transparent for the ~9 un-migrated bare-pointer sites, nor for ancestor-chain
  walks beyond the specific resolver fix — that transparent net is Phase 3 (§5).
- `operator()`'s `state&` **cannot** be made transparently safe: a bare `state&`
  carries no refcount, and changing the return type to `state_ptr` would break the
  pervasive fluent-chaining idiom (`root("a.b").value("x")`, `e("status").value(…)`
  at `uri_http_cache.cpp:313-320`, `operator std::string()` at `state.h:419`,
  `operator ptree()` at `:457`) and all 82 call sites. A is an **added** accessor,
  not a transparent upgrade — set expectations accordingly.

### 3.6 Risks

- **esft variant** — must be `boost::` (§3.2). Real footgun; std would throw
  `bad_weak_ptr`.
- **`destroyed` timing *and thread*** — a held handle defers `~state`/`destroyed`
  past the sweep, and the last-ref drop now happens wherever the pin dies (e.g. on a
  pool worker), not necessarily the sweeper thread (`state.cpp:1378`). No production
  subscriber depends on synchronous `destroyed` today (only `state_expiry_test.cpp`
  subscribes, and it holds no handle, so it stays green), but this is a **contract
  change** to state, not just "timing" — call it out in the changelog.
- **Ancestor expiry** remains a residual hazard for any *other* code that walks
  `_parent` off a pinned deep node; §3.3 covers the resolver, Phase 3 covers it
  transparently. Reading a pinned node's own storage (value/data/children/`_mutex`)
  is always safe.
- **ABI** — adding a base class + weak_ptr member changes layout: recompile all
  consumers (whole tree + pycvc/cvcGL). Source-compatible (additive); SWIG unaffected
  (protected ctor, non-owning returns).

## 4. Phase 2 — atomicity on top of A

**Goal:** a general path to multi-node-atomic reads/writes, so a subsystem does not
need a coarse lock across a whole entry. A (Phase 1) is the enabler: when a writer
publishes a new version and unlinks the old one, an in-flight reader's pin keeps the
old snapshot alive — *without A, the swap would free it under the reader.*

Two shapes, pick per site:

### 4.1 Collapse the unit into one node's `data()` payload (best for the cache)

Store the whole entry — metadata + body — as a **single node's `data()` blob**
(a serialized struct). Then a read is one locked `data()` read and a write is one
locked `data()` write: **inherently atomic**, no multi-node problem, no torn reads.

For the HTTP cache specifically this is the clean win. `write_entry`
(`uri_http_cache.cpp:308-323`) becomes "serialize the struct, one `data()` set";
`read_entry` (`:271-289`) becomes "one `data()` get, deserialize". Combined with A's
pin for lifetime and the fact that **single-flight already serializes writers per
key** (`flight_mutex`), the cache's **read path needs no `cache_mutex` at all** — it
shrinks to what single-flight already provides. This is the concrete route to the
atomicity goal and it strictly improves on today's coarse lock (lock-free reads).

Cost: the entry's children are no longer individually addressable via
`state://…?children` — acceptable for the cache (internal nodes), but note it.

### 4.2 Atomic-publish / COW or a `state::transaction` primitive (genuine subtrees)

For places that genuinely need a *multi-node subtree* read as a unit and cannot
collapse to one blob:

- **Atomic-publish / COW:** the writer builds a fresh node/subtree off to the side,
  then publishes it by swapping one child `state_ptr` in the parent map under the
  parent's brief `_mutex`. A published node is treated as **immutable** (never
  mutated in place again); a reader grabs the current `state_ptr` under that brief
  lock (← A's owning accessor), releases, and reads the now-immutable node lock-free.
- **`state::transaction` / subtree snapshot:** a first-class primitive that reads or
  swaps a subtree atomically, if a declarative form is preferred over the manual
  publish discipline.

Candidate consumers beyond the cache (verified as multi-node-atomic-sensitive):
`state_value_codec` `__type__`-keyed subtrees (`state_value_codec.cpp:193-279`,
used by evaluator-state encode/decode for checkpoint/migrate) and `state_list`. These
should get either the primitive or an explicit "not concurrent yet" annotation so the
lifetime fix is not mistaken for making them concurrency-safe.

### 4.3 What Phase 2 delivers

- The cache drops `cache_mutex` (via §4.1); lock-free, torn-read-free reads.
- A documented, reusable atomicity pattern (§4.2) for the remaining multi-node sites.

## 5. Phase 3 — Option C: deferred-reclamation safety net

**Goal:** a *transparent* lifetime net for the bare-pointer sites we choose not to
migrate, and — crucially — for the **ancestor-chain-walk** hazard that a leaf-pin
does not cover (a swept ancestor lingers, so `_parent`/`fullName()` stays valid).
This is the "belt and suspenders" layer. It does **not** add atomicity.

### 5.1 The hard requirement: a quiescence / drain point

`sweepExpired()` is purely lazy/on-access (`uri_http_cache.cpp:329,479`,
`intrinsics.cpp:218,655`, recursion `state.cpp:1353`) and there is **no epoch / RCU /
hazard-pointer / quiescence machinery** in `cvc::core` today (`thread_pool::_epoch`
at `thread_pool.h:125` is a per-job wake counter, unrelated). The only ambient
per-frame drain (`AriRuntime`) lives in the cvcGL layer, absent from CLI / headless /
pycvc / non-Ariadne downstream apps.

So C requires designing a drain mechanism first. Options to evaluate:
- an **epoch/quiescent-state** scheme where sweep unlinks + parks nodes in a graveyard
  reclaimed once all reader threads have passed a quiescent point;
- **hazard pointers** per reader thread;
- a **scheduler-tick barrier** as the natural drain point for the DSL/scheduler world
  (but headless/CLI/non-Ariadne apps need their own).

### 5.2 Costs to resolve before C ships

- **`destroyed` contract** — deferral changes when/where `destroyed` fires; guard the
  existing synchronous expectation (`state_expiry_test.cpp:108-109` asserts
  `expiring_order==1, destroyed_order==2` right after `sweepExpired()`).
- **Memory retention** — swept ≤64 MiB cache bodies would linger past the cache's
  only documented bound (`uri_http_cache.cpp:12-13,321-322`, freed today at
  `state.cpp:1378`); the drain cadence + an LRU budget must bound this.
- **`evict_locked` assumption** — the cache force-expires then sweeps then immediately
  re-reads/writes the same key expecting the old node gone (`uri_http_cache.cpp:326-330`);
  confirm deferred reclamation preserves that "unlink is synchronous, free is deferred"
  semantics (it does, since unlink at `:1365` stays synchronous).

### 5.3 Decision gate

C is worth it **only after** a drain point is designed and Phases 1–2 prove out. It
covers the long tail + ancestor walks transparently, but it does not advance
atomicity, and the actual crashing blast radius (§3.4) is already closed by Phase 1.
Scope C as its own PR once the drain design is settled.

## 6. Options considered and rejected

- **Change `operator()`/`findDescendant` return types to `state_ptr` (transparent A).**
  Rejected: breaks fluent chaining and all 82 call sites (§3.5).
- **Option D — tree-wide `shared_mutex` for structure.** Rejected: it self-deadlocks
  against the load-bearing per-node-lock + **synchronous-signal-with-reentrancy**
  model — structural mutation fires `childChanged`/`expiring` synchronously
  (`state.cpp:1376-1377`) and subscribers re-enter the tree on the same thread,
  re-acquiring the non-recursive global lock. It also can't cover a caller's use of a
  returned bare pointer *after* the accessor returns without the caller holding the
  read lock across its use — i.e. "add your own lock", shared. Confirmed the
  emit-after-unlock discipline (`state.cpp:450-452`; "never `resolveLink()` while
  holding `_mutex`", `state.h:279-281`).
- **C as the first/only fix ("zero caller change").** Rejected as a starting point:
  its "zero change" claim is false (needs the §5.1 drain + RCU read-side brackets on
  every bare reader), and A closes the real crash with a ~12-site opt-in migration.
  C is retained as the Phase 3 transparent net, not the primary fix.

## 7. Sequencing, PRs, and test/CI plan

**PR 1 — Option A core + resolver fix (closes the SEGFAULT).**
`state : boost::enable_shared_from_this`, `findDescendantShared`, `sharedChild`,
`state::handle`, pinned `resolveLink`/`resolveRemote`; migrate `uri_state.cpp` with
the §3.3 ancestor fix. Touches `state.h`/`state.cpp` + `uri_state.cpp` (+ callers).

**PR 2 — remaining Phase-1 migrations.** `state_distributed_admin`, `pycvc_state`
(defensive), any other confirmed cross-thread site.

**PR 3 — Phase 2 atomicity.** Collapse the cache entry to a single-node payload (§4.1)
and drop `cache_mutex`; land the atomic-publish pattern or `state::transaction`
primitive (§4.2) and annotate/covert `state_value_codec`/`state_list`.

**PR 4 — Phase 3 Option C.** Only after the §5.1 drain design is agreed.

**Tests (new `state_lifetime_stress_test.cpp` + additions):**
- **Reproducer:** reader threads doing the bare `findDescendant`+`value()`/`data()`
  shape (`uri_state.cpp:60-78`) vs sweeper threads doing
  `expireAt(now-1s); sweepExpired()` (`intrinsics.cpp:214-218`). Assert TSan/ASan
  flags a UAF with the bare accessor and **zero** reports with `findDescendantShared`
  held across the read.
- **Ancestor case (guards the H1 hole):** expire an *intermediate ancestor* (not just
  the leaf) and drive a reader through `effective_node`/`fullName()` — must be clean
  after the §3.3 fix; without it, this is the case that still crashes.
- **Deferred-destroy semantics:** hold a handle to `a.gone`, `expireAt(past)`,
  `sweepExpired()`; assert `findDescendant("a.gone")==nullptr` immediately (unlink is
  synchronous, `state.cpp:1365`) but `destroyed` has not fired; drop the handle and
  assert it fires. Keep `state_expiry_test.cpp:94-110` green (no handle → synchronous).
- **Torn-read reproducer (Phase 2):** concurrent `write_entry`-shape writer vs reader;
  assert no torn read after the single-node-payload / atomic-publish change.

**CI / gotchas:**
- TSan in this repo needs ASLR off: run under `setarch -R` (repo gotcha). Run the
  stress tests under both TSan and ASan.
- Coverage gate is 80% (`CMakeLists.txt:210`, `CVC_ENABLE_COVERAGE`): the new
  accessor branches (hit / missing-segment-null / empty-path-`shared_from_this`) each
  need direct unit coverage.
- Binding-visible: none of the new API is wrapped, but rebuild pycvc/cvcGL for the ABI
  change and re-run their tests.

**Open questions for review:**
1. Ancestor fix — prefer (b) derive-canonical-from-input everywhere, or (a) offer a
   chain-pinning `findDescendantShared` overload for callers that must walk `_parent`?
2. Phase 2 — is collapsing the cache entry to one `data()` blob acceptable (loses
   per-field `state://…?children` addressability of cache internals)?
3. Phase 3 — which drain mechanism (epoch / hazard-pointer / scheduler-tick), given
   headless/CLI/non-Ariadne apps have no ambient frame loop?
