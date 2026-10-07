# Ariadne — the libcvc UI DSL

*How an `.ari` document is authored, composed, and run: the backend-neutral
Runtime, its two evaluation lanes, the per-frame contract, document structure,
`include`/`import`/`load` composition, and state + channel scoping.*

This is the authoring and architecture guide. The normative spec is
[`docs/roadmap/CVCGL-UI-DSL-ROADMAP.md`](roadmap/CVCGL-UI-DSL-ROADMAP.md); where a
number here differs from the roadmap, **the code is authoritative** and this guide
cites `file:line` so you can confirm it. Ariadne builds on `cvc::state` and the
`state_exec` DSL — read [`STATE_API.md`](STATE_API.md) and
[`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md) first if those are
unfamiliar; every binding, computed field, and handler in an `.ari` document is a
`state_exec` program over a `cvc::state` tree.

## Table of Contents

1. [The Runtime](#1-the-runtime)
2. [Two evaluation lanes](#2-two-evaluation-lanes)
3. [The per-frame (deferred-intent) contract](#3-the-per-frame-deferred-intent-contract)
4. [Backends](#4-backends)
5. [Documents and the `.ari` schema](#5-documents-and-the-ari-schema)
6. [Composition: include / import / load](#6-composition-include--import--load)
7. [State prefix and mount scoping](#7-state-prefix-and-mount-scoping)
8. [Channel scoping](#8-channel-scoping)
9. [Running a document](#9-running-a-document)
10. [Async and networking](#10-async-and-networking)
11. [Caveats — what is not yet wired](#11-caveats--what-is-not-yet-wired)

---

## 1. The Runtime

`cvc::ariadne::Runtime` ([`inc/cvc/ariadne/ariadne.h:41`](../inc/cvc/ariadne/ariadne.h))
is the **backend-neutral** host for a single `.ari` document. It is pure libcvc: it
has no ImGui, VTK, or terminal knowledge. It owns exactly three things:

- the retained widget tree (built by the loader, reconciled each frame);
- the single per-frame reconcile/commit boundary (§3); and
- every `cvc::state` binding the document performs.

A Runtime is constructed with an app and a **state prefix**:

```cpp
cvc::ariadne::Runtime rt(app, "ui.docs.myapp");
```

The prefix roots every relative bind the document makes (§7). A leading `/` on a
bind escapes to app-root-absolute; an empty prefix leaves binds app-root-relative.
The Runtime holds a `Backend*` that it does **not** own — the backend must outlive
the Runtime (`set_backend`, [`ariadne.cpp`](../src/cvc/ariadne/ariadne.cpp)). Widget
binds and scene-visibility binds share one resolver (`resolve_bind`, §7), so a widget
`bind:` and a scene `visible:` written against the same relative path deliberately
collide on one state key.

You rarely wire a Runtime by hand. Two things package it:

- **`ariadne_hello`** ([`src/cvcGL/examples/ariadne_hello.cpp`](../src/cvcGL/examples/ariadne_hello.cpp))
  — the native reference host: it constructs the Runtime, an `ImGuiBackend` over a
  VTK view, registers URI handlers, loads the document, and runs the SDL-driven frame
  loop. Read it as the canonical "how to host Ariadne" example.
- **`cvc::gl::ariadne::AriRuntime`**
  ([`inc/cvc/gl/ariadne/AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)) — bundles
  Runtime + `ImGuiBackend` + loader + scene realize + the frame loop behind one
  object. It is libpython-free (it takes `std::function`, never `PyObject*`), so
  native C++, static-wasm, and the `pycvc_gl` Python bindings all share it. This is
  what lets you drive a full `.ari` application from Python (§9).

---

## 2. Two evaluation lanes

Every executable field in a document runs in one of two lanes. **The lane is chosen
by the slot the program sits in, not by what the program does** — this is the central
safety property. You cannot make a "read" slot perform an action, or vice versa.

### 2.1 The reactive READ lane

Reactive slots — `visible_when`, `enabled_when` / `disabled_when`, computed `bind` /
`text`, `tooltip`, `options`, `repeat.count` — run in a long-lived stackless evaluator
(`class ReactiveEngine`, [`ariadne.cpp:385`](../src/cvc/ariadne/ariadne.cpp)) over a
**default-deny** environment:

- The value allowlist admits arithmetic, coercion, logic, and side-effect-free state
  *readers* only (`kAllowed`, [`ariadne.cpp:417`](../src/cvc/ariadne/ariadne.cpp)).
- The special-form gate admits only `if / begin / let / while / for / lambda / defun /
  set / return / quote / yield / break` (`kAllowedForms`,
  [`ariadne.cpp:433`](../src/cvc/ariadne/ariadne.cpp)).

Nothing that writes state, schedules, watches, sends messages, or does I/O is
reachable. An off-allowlist symbol does not error loudly — it **fails safe**: the slot
degrades to hidden / disabled / empty / last-good value and the frame records a
one-shot warning (`take_reactive_warnings`,
[`ariadne.cpp:1682`](../src/cvc/ariadne/ariadne.cpp)). state_exec is always built (there is
no build option to drop it), so the engine is always present.

**The read lane never parks.** It is bounded three ways
([`ariadne.cpp:603`](../src/cvc/ariadne/ariadne.cpp)):

| bound | constant | value |
|-------|----------|-------|
| per-eval step cap | `kMaxSteps` | `100000` |
| per-eval wall-time cap | `kMaxSeconds` | `5 ms` |
| per-frame aggregate | `kFrameBudgetSeconds` | `10 ms` |

The frame budget resets at the top of each frame, so *N* widgets cannot jointly stall a
frame — once the aggregate is spent, remaining predicates degrade fail-safe for that
frame only.

> These are the as-built numbers. Roadmap §7.4 still cites an older `CAP=50000` / 2 ms;
> trust the code.

### 2.2 The action / drain lane

Fire-once handlers (`on:` on a button, a menu item's program) and the resident
handlers (`on_tick` / `on_key` / `on_pointer`) run in the action lane, on the
**app-wide scheduler** (`app.exec_scheduler()`), **off** the render walk, inside
`Runtime::drain()` ([`ariadne.cpp:1642`](../src/cvc/ariadne/ariadne.cpp)).

A fire-once handler is *submitted* (`submit_action`,
[`ariadne.cpp:1375`](../src/cvc/ariadne/ariadne.cpp)) with the full default environment
plus host intrinsics, its writes chrooted to the action's mount prefix (§7), under
per-activation caps ([`ariadne.cpp:1382`](../src/cvc/ariadne/ariadne.cpp)):

| bound | constant | value |
|-------|----------|-------|
| step cap | `kActionMaxSteps` | `5000` |
| wall-time cap | `kActionMaxSeconds` | `0.1 s` |
| write budget | `kActionMaxBytes` | `256 KiB` |

A quick action completes within that drain. An action that `await`s, `sleep`s, or
`msg-recv`s **parks** and resumes on a later frame — this is what makes the async arc
(§10) possible without blocking the frame.

### 2.3 Resident handlers

`on_tick` / `on_key` / `on_pointer` each become **one** long-lived, owner-tagged
process, wrapped by the runtime as `(while t (let ((event (msg-recv "chan"))) body))`
(`ensure_resident`, [`ariadne.cpp:1458`](../src/cvc/ariadne/ariadne.cpp)):

- **tick** is woken once per drain (coalesced — you get one tick per frame, not a
  backlog).
- **key / pointer** are woken per event; the event is delivered as a dict you read with
  `(get-attr event "key")`, `(get-attr event "x")`, etc.

Residents have no total budget (they run for the life of the document), but each *step*
is wall-capped at `kResidentStepSeconds = 0.05`
([`ariadne.cpp:1497`](../src/cvc/ariadne/ariadne.cpp)). On teardown they are reaped as a
group via their owner tag.

---

## 3. The per-frame (deferred-intent) contract

`Impl::render` ([`ariadne.cpp:1324`](../src/cvc/ariadne/ariadne.cpp)) is a **pure
reader**. This is the load-bearing discipline of the whole system:

1. **At frame top**, any pending tree swap (from `set_root`) is applied *before any node
   is emitted* — this is the single reconcile/commit boundary. Budgets reset here.
2. **The walk emits widgets and reads state.** Handlers never run mid-walk: a clicked
   button or menu item does `enqueue(w.on)`
   ([`ariadne.cpp:1101`](../src/cvc/ariadne/ariadne.cpp)); a widget `on_click` /
   `on_hover` / `on_drag` does `enqueue_event`
   ([`ariadne.cpp:1278`](../src/cvc/ariadne/ariadne.cpp)). `enqueue` captures the
   current mount prefix, so a mounted fragment's handler writes its own subtree.
3. **`drain()` runs the queued intents on the *next* frame**, then pumps the scheduler
   and residents.

The consequence — the **one-frame-latency** rule — is that the render walk *cannot cause
a state change that it then reacts to within the same frame*. A click enqueued this
frame is executed next frame; its state writes are visible to the reactive lane the
frame after. This is what makes reactive predicates stable to evaluate: nothing they
read is mutating underneath them mid-walk.

> **Footgun.** With a host-pumped backend you drive frames via the view's render
> (which fires `Runtime::render()` through the backend's draw callback). **Never call
> `Runtime::render()` directly**, and always run `frame()` before `render()`
> ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)). Calling render directly
> bypasses the frame-top reconcile and the event pump.

---

## 4. Backends

`cvc::ariadne::Backend` ([`inc/cvc/ariadne/backend.h`](../inc/cvc/ariadne/backend.h))
is the surface abstraction. The division of labor is strict: **core owns the tree,
the reconcile, and all `cvc::state`; the Backend owns only the surface** — it draws the
value handed to it and reports the edit, and never touches `cvc::state` itself.

Each state-bound primitive returns `{changed, committed, value}`
([`backend.h`](../inc/cvc/ariadne/backend.h)). Core writes state **only on
`committed`**, so a slider drag is one write on release, not one write per frame.

A backend advertises `Capabilities{windows, menubar, mouse, keyboard, color, glsl,
view_embed, owns_loop}` ([`backend.h`](../inc/cvc/ariadne/backend.h)); the loader uses
these to decide widget substitution. `owns_loop` distinguishes a **host-pumped**
backend (the app runs the loop and calls in per frame) from a **framework-owned** one
(the backend runs its own loop).

Two backends ship:

- **`cvc::gl::ImGuiBackend`**
  ([`inc/cvc/gl/ariadne/ImGuiBackend.h`](../inc/cvc/gl/ariadne/ImGuiBackend.h)) — ImGui
  over a VTK view, the **only** place Ariadne touches ImGui or VTK. It is inert without
  `CVC_ENABLE_IMGUI`. Its caps are all-true except `owns_loop = false` (host-pumped).
  Its `install()` sets the VTK view's overlay draw callback to call `Runtime::render()`
  — that single line is the entire coupling between Ariadne and VTK's render pass, and
  it lives in the backend, not the core.
- **FTXUI / terminal backend**
  ([`inc/cvc/ariadne/ftxui_backend.h`](../inc/cvc/ariadne/ftxui_backend.h)) — the
  `owns_loop = true` counterpart that draws the same tree in a terminal. Present as the
  secondary backend.

Because the core is backend-neutral, the same document renders through either backend;
capability gaps (e.g. no `color`) drive graceful widget substitution rather than errors.

---

## 5. Documents and the `.ari` schema

An `.ari` file is a YAML document. `load_file` / `load_string` parse it via `load_node`
([`loader.cpp:1740`](../src/cvc/ariadne/loader.cpp)) into a `LoadResult`
([`inc/cvc/ariadne/loader.h`](../inc/cvc/ariadne/loader.h)):

- a `Widget root` (a Group) — the retained tree;
- a parsed `Scene`, `Meta`, `customs`, and `channels` + `lint` config;
- and **verbatim-captured** script strings: `init_script`, `on_tick_script`,
  `on_key_script`, `on_pointer_script`.

**The loader is state-free.** It captures structure and script *text*; it never touches
`cvc::state` and never runs the DSL. The host runs the scripts (§9). This separation is
why the same `LoadResult` can be driven by different hosts.

### 5.1 Top-level keys

The built-in top-level keys (`is_builtin_block`,
[`loader.cpp:97`](../src/cvc/ariadne/loader.cpp)) are:

```
meta  menubar  windows  overlays  root (alias: children)  scene  customs
init  on_tick  on_key  on_pointer  units  import  channels  lint
```

Any **other** top-level key is dispatched to a parser registered via
`register_ari_block`, or — if none is registered — **silently ignored**. This is
deliberate forward-compatibility: a document authored for a newer build still loads the
parts an older build understands.

### 5.2 `meta:` and the `min_libcvc` gate

`parse_meta` ([`loader.cpp:1455`](../src/cvc/ariadne/loader.cpp)) reads `name`,
`author`, `description`, `version`, and `min_libcvc`. The version gate is checked
**first**, before anything else in the document
([`loader.cpp:1755`](../src/cvc/ariadne/loader.cpp)): if the running libcvc is older
than `min_libcvc`, the load **fails with no tree** (major.minor.patch compare, ignoring
`-pre` / `+build` suffixes; `version_at_least`,
[`loader.cpp:73`](../src/cvc/ariadne/loader.cpp)). A missing `min_libcvc` only warns. A
fragment pulled in by `load:` carries its own `meta` and is gated independently.

```yaml
meta:
  name: convoy-console
  version: 1.4.0
  min_libcvc: 0.42.0      # refuse to load on an older runtime
```

### 5.3 `init:` and the resident handlers

`init`, `on_tick`, `on_key`, and `on_pointer` are scalar `state_exec` strings the loader
only *captures*. The host wires them:

- **`init_script`** runs **once** at load time, at the document prefix, via `run_init`.
  Because it runs first, an `init:` seed such as `(state-set "demo.n" 3)` wins over a
  widget's `def:` default for the same key.
- **`on_tick` / `on_key` / `on_pointer`** become the resident processes of §2.3, fed
  events through `Runtime::post_input`.

A small, complete document (mirroring `nav_compute.ari`):

```yaml
meta: { name: hello, version: 0.1.0 }
menubar:
  - { label: File, items: [ { label: Quit, on: "(quit)" } ] }
windows:
  - title: Controls
    children:
      - { slider: { label: "N", bind: "demo.n", min: 1, max: 100 } }
      - { text: { text: "(str \"N = \" (state-get \"demo.n\"))" } }   # computed, read lane
      - { button: { label: "Bump", on: "(state-set \"demo.n\" (+ (state-get \"demo.n\") 1))" } }
init: "(state-set \"demo.n\" 10)"
on_tick: "(state-set \"demo.frames\" (+ (state-get \"demo.frames\") 1))"
```

---

## 6. Composition: include / import / load

Three mechanisms compose documents. They differ in **isolation**, not in how they
fetch:

| mechanism | fetch | scope |
|-----------|-------|-------|
| `include:` | none — an in-document `units:` template | **shares** the enclosing chroot |
| `import:` | cross-file | **shares** scope (merges `units:` only) |
| `load:` | cross-file | **own** `includes.<as>` chroot (isolated module) |

### 6.1 `include:` — template instantiation

`expand_include` ([`loader.cpp:852`](../src/cvc/ariadne/loader.cpp)) instantiates a
`units:` template: it clones the unit (`YAML::Clone`) and does a single-pass `{name}`
token substitution from the `include:`'s `args:`. Units are collected before the body,
so instantiation is order-free. A name-keyed expansion-stack guard
([`loader.cpp:863`](../src/cvc/ariadne/loader.cpp)) refuses recursion and fan-out; an
unknown unit warns and renders empty. `include:` is the static counterpart of `repeat:`.

### 6.2 `import:` — shared-scope library merge

`process_imports` ([`loader.cpp:761`](../src/cvc/ariadne/loader.cpp)) resolves a URI (or
list) through the shared resolver relative to the document's `base_dir`, merges another
`.ari`'s `units:` into scope, and recurses into that library's own imports. It dedups
and cycle-guards on the resolved `canonical` identity
([`loader.cpp:789`](../src/cvc/ariadne/loader.cpp)), capped at `kMaxImportDepth = 32` /
`kMaxImports = 4096` ([`loader.cpp:758`](../src/cvc/ariadne/loader.cpp)). Local units
override imported ones. This is how a document composes a library of shipped panels.

### 6.3 `load:` — isolated sub-module mount

`expand_load` ([`loader.cpp:928`](../src/cvc/ariadne/loader.cpp)) mounts an external
fragment as an **isolated module** in a fresh sub-context (its `units:` are its own),
based at its own directory. Guards: a resolved-URI cycle set, a per-chain depth cap
`kMaxMountDepth = 32`, and a document-wide aggregate cap `kMaxMounts = 256`
([`loader.cpp:882`](../src/cvc/ariadne/loader.cpp)) that catches the `K^depth` fan-out a
depth cap alone would miss. The mount id is the explicit `as:`, else it is derived from
the URI basename, sanitized to a single state segment, with sibling duplicates
disambiguated `_2` / `_3`. A mounted module is sandboxed to its own state sub-prefix
(§7) and reaches the parent only through explicitly granted `link:` holes.

### 6.4 The URI resolver

All external references (`import:`, `load:`, scene `source:`, `fetch`) go through one
resolver ([`inc/cvc/ariadne/uri.h`](../inc/cvc/ariadne/uri.h)):
`parse_uri` → `resolve(uri, base)` → `{ok, content, canonical, error}`. The grammar is
`<scheme>://<path>[?query]`; a bare path is `file`. Schemes:

- **`file://`** — built-in, app-free.
- **`state://…?value | ?data | ?children`** — reads from the `cvc::state` tree
  ([`uri_state.cpp`](../src/cvc/ariadne/uri_state.cpp)).
- **`http(s)://`** — **opt-in**: a host must call `register_http_uri_handler()` (or the
  cached variant, §10). It is *not* registered by default — see the caveats (§11).
- **`cvc://<relpath>`** — a location-independent component path, searched along
  `$CVC_ARIADNE_PATH` → host-supplied extra dirs → the install datadir
  `<prefix>/share/libcvc/ariadne` → cwd (`register_cvc_uri_handler`). Use this for
  shipped components so a document does not hard-code install paths.

The `canonical` field is the identity used for cycle-guarding and for resolving
relative bases in nested composition.

---

## 7. State prefix and mount scoping

### 7.1 The one bind rule

Every relative reference in a document is resolved by `resolve_bind(prefix, bind)`
([`bind.h:29`](../inc/cvc/ariadne/bind.h)):

- `bind` empty → empty (no binding);
- `bind` starts with `/` → strip it, resolve **app-root-absolute** (the only escape out
  of the document's subtree);
- else, non-empty `prefix` → `prefix + SEPARATOR + bind`;
- else → `bind` as-is.

Widgets and scene binds use the same rule, which is why a widget `bind:` and a scene
`visible:` on the same relative path co-resolve to one key.

### 7.2 Two prefix conventions

- The roadmap-canonical convention roots a document at `ui.docs.<docId>` (its tree under
  `.tree.<id>.*`, a mount under `.includes.<as>.*`).
- `AriRuntime` instead roots at the **scene's** `getStatePrefix()` (e.g. `cvcgl`,
  [`AriRuntime.cpp`](../src/cvcGL/ariadne/AriRuntime.cpp)) so that widget binds and scene
  binds land in the same subtree and co-resolve.

Same rule, different prefix string — pick the one your host wants; nothing else changes.

### 7.3 Mount sub-prefix

`Widget::scope` ([`widget.h`](../inc/cvc/ariadne/widget.h)) is a relative segment. When
the runtime emits a container it computes `sub_prefix = resolve_bind(parent_prefix,
scope)`, pushes it (RAII `ScopeGuard`), and resolves the children against it
([`ariadne.cpp:961`](../src/cvc/ariadne/ariadne.cpp)). A mount's *own* reactive fields
(e.g. its `visible_when`) stay in the **parent** scope — they are evaluated before the
push. A repeated mount gets `includes.<as>.<i>` per instance.

### 7.4 Default-deny and `link:` holes

A mounted module is sandboxed to its sub-prefix. Access to the parent's state is granted
**per name** via a `link:` hole (`LinkHole`, [`widget.h`](../inc/cvc/ariadne/widget.h)):

- scalar `name: target` grants read-write;
- map `name: {to:, mode: ro|rw}` **fails closed** — only an exact `"rw"` mode is
  writable ([`loader.cpp:1038`](../src/cvc/ariadne/loader.cpp)); anything else is
  read-only. A hole whose name starts with `/` is rejected.

At reconcile, `wire_holes` ([`ariadne.cpp:931`](../src/cvc/ariadne/ariadne.cpp)) plants a
transparent link at `resolve_bind(sub_prefix, name)` pointing to
`resolve_bind(parent_prefix, target)`, with writability set from the mode. Reads see
through the link; a read-only hole keeps writes local to the module. Holes are re-planted
every reconcile, so a re-mount can never inherit a stale grant. A fragment may declare
the holes it expects with `needs:`; an ungranted expectation warns and the fragment falls
back to a local (fail-safe) binding. A per-mount `init:` runs once at the sub-prefix
**after** the holes are wired.

---

## 8. Channel scoping

Message channels (`msg-send` / `msg-recv` / `msg-pending`) are **chrooted per document,
exactly like state**. The rule is `resolve_channel_key(root_path, channel)`
([`intrinsics.cpp:884`](../src/cvc/state/state_exec/intrinsics.cpp)):

- channel contains `#` → used verbatim (these are runtime-internal tick/key/pointer
  channels, and are policy-exempt);
- channel starts with `/` → app-root-global escape (strip the `/`; permitted only for a
  channel the document declared `global: true`);
- `root_path` empty → identity (backward-compatible with un-prefixed hosts);
- else → `<root_path>.channels.<name>` (private-by-prefix).

Cross-scope sharing reuses the §7 hole mechanism: a mount's `channels:` grant lowers into
a transparent link under the reserved `channels.` subtree, and `resolve_channel` follows
the link's **target string** — a channel key is a redirect, so the target node need not
exist ([`intrinsics.cpp:672`](../src/cvc/state/state_exec/intrinsics.cpp)).

### 8.1 Authoring surface

```yaml
channels:
  - telemetry                      # bare name — private to this document
  - { channel: alerts, global: true }   # reachable app-root as /alerts
lint:
  channels: strict                 # strict | warn | off  (default: strict when a channels: block is present)
  quiet: false
```

Declaring a `channels:` block is the **gate** — channel enforcement runs only for
documents that declare one.

### 8.2 Two enforcement layers

Both key off `lint.channels`:

- **Load-time static lint** (`lint_channels`,
  [`loader.cpp:1684`](../src/cvc/ariadne/loader.cpp)) walks each program lane's AST for
  `(msg-* "literal")` calls, skips `#`-channels, and errors or warns on an undeclared
  channel. A `/`-prefixed reference must match a channel declared `global: true`.
- **Runtime backstop** (`Runtime::set_channel_policy`,
  [`ariadne.h:86`](../inc/cvc/ariadne/ariadne.h)) catches an undeclared *dynamic* channel
  name (one the static lint could not see), fail-safe, exempting `#`-channels and
  grant-linked channels. The host feeds the policy from the `LoadResult`.

> The runtime global allowlist must be fed from each declared channel's `global` flag,
> not its `shared` flag (`shared` is documentation/intent for what a mount *may* be
> granted; `global` is what actually lives at app-root). The reference host and
> `AriRuntime` both key on `global`.

---

## 9. Running a document

### 9.1 Native (C++)

The reference host is
[`ariadne_hello.cpp`](../src/cvcGL/examples/ariadne_hello.cpp). The shape is:

```cpp
cvc::ariadne::Runtime rt(app, prefix);
rt.set_backend(&backend);           // backend must outlive rt
backend.install(rt, view);          // sets the view's overlay draw callback → rt.render()

register_cvc_uri_handler(/*extra dirs*/);        // cvc:// components
auto lr = cvc::ariadne::load_file(path);
if (!lr.ok) throw std::runtime_error(lr.error);

run_init(app, prefix, lr.init_script, nullptr);  // init seeds win over widget defaults
rt.set_tick_program(lr.on_tick_script);
rt.set_key_program(lr.on_key_script);
rt.set_pointer_program(lr.on_pointer_script);
if (lr.has_channels_block)
  rt.set_channel_policy(declared, global, strict, quiet);  // `global` from c.global
// ... realize scene if lr.scene.any(), then run the frame loop:
//   processUIEvents → cam.update → rt.drain() → sync_scene_visibility → tick_scene
//   view.render()  (fires rt.render() via the overlay callback)
```

### 9.2 From Python (a full app via `AriRuntime`)

`AriRuntime` ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)) collapses all of the
above into one object, exposed through the `pycvc_gl` bindings. Python never sees SDL or
VTK directly — cvcGL wraps SDL as the internal input backend and VTK owns the window:

```python
import pycvc, pycvc_gl

app  = pycvc.App()
view = pycvc_gl.SceneRenderer(...)      # owns the GL window
rt   = pycvc_gl.AriRuntime(view.scene(), view.camera(), view.overlay())

rt.load("demo.ari")
# Register a Python function as a DSL verb (host intrinsic on the app-wide scheduler):
rt.register_verb("greet", lambda name: f"hello {name}")
rt.register_async_verb("fetch_rows", slow_query)   # offloaded to the compute pool

while not rt.should_close():
    rt.frame(dt)     # processUIEvents → cam.update → drain → visibility → tick
    rt.render()      # NEVER call the underlying rt.render() directly; go through view
```

`register_verb` / `register_async_verb` install exactly one `register_action_intrinsics`
provider onto the app-wide scheduler, so the verbs are reachable from the document's
action lane. `AriRuntime` also exposes `step`, `run`, `on`, `post_key`, `post_pointer`,
and `reload_if_changed`.

### 9.3 From Python (DSL only, via `pycvc.Exec`)

If you only need to run `state_exec` programs against an app's tree (no widgets, no
scene), `pycvc.Exec` ([`bindings/pycvc/pycvc_exec.h`](../bindings/pycvc/pycvc_exec.h))
is the lighter handle. It owns a **private** async scheduler and lets you register Python
callables as DSL functions (§10). One `Exec` is driven by one thread — use a separate
`Exec` per thread.

---

## 10. Async and networking

### 10.1 Registering Python work as DSL functions

`pycvc.Exec` offers three registration modes
([`pycvc_exec.h`](../bindings/pycvc/pycvc_exec.h)):

- **`register_fn(name, callable)`** — *synchronous*. The callable runs inline on the
  `run()` thread with the GIL held and blocks the DSL program until it returns. Use for
  fast pure functions.
- **`register_async_fn(name, callable)`** — offloads a **blocking** callable to the app's
  compute pool and **parks** the calling DSL program until it finishes, so the scheduler
  stays live. The callable runs on a pool worker with the GIL acquired for the call, so
  `urllib`/`requests` or a GIL-releasing numpy compute runs truly concurrently. On a
  raise the program resumes with a `{"__async_error__": <message>}` dict (a reserved key
  a normal dict result cannot collide with).
- **`register_async_coro(name, coro_fn)`** — registers an `async def`. Calling it
  schedules the coroutine on an asyncio loop the `Exec` owns and parks the DSL program.
  Unlike `register_async_fn`, the coroutine runs **on** the `run()` thread, cooperatively
  stepped one bounded slice per pump iteration, so it and the DSL march along together and
  its `await asyncio.sleep(...)` / `aiohttp` progress between DSL slices.

> **The coroutine must yield.** The slice is cooperative — asyncio cannot preempt a
> running step — so a coroutine that does CPU-bound work without awaiting holds the
> `run()` thread and wedges the pump. Put blocking or CPU-bound work in
> `register_async_fn` (the pool), not in a coroutine. Passing an `async def` to
> `register_fn` / `register_async_fn` is rejected and steered to `register_async_coro`.

The nesting rule for both async forms: `(name ...)` must be nested inside an enclosing
frame (e.g. `(begin (name ...))` or `(state-set "r" (name ...))`), never the whole
program, so the park has a frame to suspend.

```python
ex = pycvc.Exec(app)

async def poll(url):
    async with aiohttp.ClientSession() as s:
        async with s.get(url) as r:
            return await r.text()

ex.register_async_coro("poll", poll)
ex.run('(begin (state-set "page" (poll "http://localhost:8080/status")))')
```

### 10.2 The DSL network verbs

In an Ariadne document's action lane, the network verbs are (from
[`net_intrinsics.cpp`](../src/cvc/ariadne/net_intrinsics.cpp)):

- `http-get` / `http-get-async`, `http-request` / `http-request-async` (a general
  `METHOD` verb), and the scheme-agnostic `fetch` / `fetch-async`.
- Options are a dict: `query`, `headers`, `body`, and `form` (form is url-encoded with a
  default `Content-Type`). Query values are url-encoded for you; there are `url-encode` /
  `url-decode` core builtins if you need them directly.
- The reply is a dict: `{ok, status, body, url, headers, error}` (`body` is raw bytes).
- The `-async` verbs return a future you `await`; `launch_http` offloads the request onto
  the compute pool and parks the program on a private reply channel, so the action lane
  never blocks the frame.

See [`ariadne-async-await-arc`] in the roadmap for how `await` threads through the drain
lane. **Note (§11):** these verbs are currently registered only in the Ariadne action
lane, and no default host registers an `http(s)://` URI handler — see the caveats.

---

## 11. Caveats — what is not yet wired

State plainly so authors are not misled. Each item cites the code; the roadmap items
below (G17–G21 in the internal gap report) are tracked for reconciliation.

- **`http(s)://` is opt-in and no default host registers a handler.**
  `register_http_uri_handler` ([`uri_http.cpp`](../src/cvc/ariadne/uri_http.cpp)) and the
  cached variant are called only by unit tests today. Until a host installs one,
  `import:` / `load:` over http and `fetch("http://…")` fail with an unknown-scheme error.
  `AriRuntime` registers `cvc://` but not http — treat the http handler as a host
  responsibility.
- **The §13.9 HTTP cache is implemented and unit-tested but wired into nothing**
  (`register_cached_http_uri_handler`,
  [`uri_http_cache.cpp`](../src/cvc/ariadne/uri_http_cache.cpp)). Even once installed, the
  primary `http-get*` verbs go straight to `cvc::net::send` and bypass it; only
  `fetch`/`fetch-async` route through the resolver where the cache lives.
- **The network verbs are not reachable from `pycvc.Exec` or `AriRuntime`.** They live in
  the Ariadne Runtime action lane only; neither the Exec env nor `AriRuntime` calls
  `register_net_intrinsics`. A pycvc author registers a `register_async_fn` HTTP wrapper
  instead.
- **`fetch` / `source:` / `import:` cannot see `Content-Type`.** `UriResult` is
  `{ok, content, canonical, error}` ([`uri.h`](../inc/cvc/ariadne/uri.h)); the cache
  stores the content type but the resolver path drops it. Only the direct `http-get` /
  `http-request` verbs expose headers, and only as an unparsed list.
- **Assets still load by filename** — there are no in-memory / bytes-blob read overloads
  yet, so a scene `source:` over a non-`file` scheme spills to a temp file via
  `resolve_to_file`, and a remote `source:` blocks the first frame synchronously
  ([`scene_realize.cpp`](../src/cvcGL/ariadne/scene_realize.cpp)). The decoders support
  memory (stb, ImageMagick `Blob`, assimp `ReadFileFromMemory`); exposing that is a
  planned follow-up.
- **AriRuntime document-level `on_key` / `on_pointer` residents need a VTK event
  producer.** The `post_key` / `post_pointer` forwarding is wired, but nothing yet
  translates VTK interactor events into those calls, so stock-loop key/pointer residents
  do not fire until a host feeds them
  ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)).
- **`load:` `prefix:` / `reload:` and §12.2 sub-scene mounting are not yet parsed** at the
  loader; only sub-UI `load:` is landed ([`loader.cpp`](../src/cvc/ariadne/loader.cpp)).
- **No DSL-visible future from the pycvc async layer.** Both `register_async_fn` and
  `register_async_coro` self-park; there is no gather/join to launch *N* Python async
  calls concurrently from one process (the net verbs' `-async` futures do offer this).

---

*Cross-references:* [`STATE_API.md`](STATE_API.md),
[`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md),
[`APP_API.md`](APP_API.md), and the normative spec
[`roadmap/CVCGL-UI-DSL-ROADMAP.md`](roadmap/CVCGL-UI-DSL-ROADMAP.md).
