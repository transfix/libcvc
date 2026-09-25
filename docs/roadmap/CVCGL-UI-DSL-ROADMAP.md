# cvcGL UI DSL — Scoping Spec (v0.2, for iteration)

Status: **draft for discussion, no code committed.** Grounded in a full survey of
the ImGui infrastructure (`ImGuiOverlay`, `ImGuiBinding`, `SceneRenderer`,
`RenderView`, `Settings`, `StageLighting`), all 14 cvcGL examples, and the
`cvc::state_exec` subsystem — all on `origin/master` (`4fb4e1198`). The goal is a
YAML description of the nested ImGui widget tree — composable, reusable-with-args,
with expressions and actions expressed in `state_exec` — that can express the
existing demo UIs. This document is the thing we iterate on before writing a loader.

> **v0.3 in progress:** decisions Q1/Q2/Q4 are folded in (§4.3, §4.6, §4.7, §7), and a
> new **scene-graph section (§9)** is being drafted so the root VTK scene's
> assets/nodes/views live in the *same* DSL as widgets and actions — which is what makes
> the minimap (a widget embedding a second VTK view) fully declarable. §9 lands as a
> follow-up commit on this PR.

### Decisions locked (v0.2)

| # | Decision | Resolution |
|---|---|---|
| 1 | Loader host | **Both** — a C++ core in libcvc + a Python/pycvc_gl entry over the same tree |
| 2 | Non-state binding | **Promote all demo tunables to `cvc::state` paths**; `bind:` is uniformly a state path |
| 3 | Expression / action grammar | **`state_exec`** — expressions are `state_exec` programs; per-frame predicates run sync, actions run on the **async schedulable** executor; authorable as **s-expr *or* nested YAML** |
| 4 | Window-geometry persistence | **Out of scope for now** |
| 5 | Escape hatches | **`custom:` opaque host nodes** |
| 6 | wasm keyboard | **Not a constraint** — wasm keyboard support is landing in a sibling effort; text-entry widgets are allowed |

---

## 1. Reality that shapes the whole design

Facts about the existing code that the schema is built *around*, not against.

1. **The root is the VTK canvas, not an ImGui window.** `cvc::gl::SceneRenderer`
   owns the render window + interactor + named viewer; `ImGuiOverlay(SceneRenderer&)`
   attaches to it and draws *inside* VTK's `RenderEvent` framebuffer (so the UI
   shows up in offscreen/wasm captures too). The DSL "root" is the **overlay host**;
   its children are a menu bar, floating windows, corner overlays, and HUDs drawn
   *on top of* the scene. The root is always present; everything else is a child.

2. **ImGui is immediate mode — there is no retained widget tree.** `ImGuiOverlay`
   stores **one** `std::function` draw callback that VTK re-invokes once per
   rendered frame between `NewFrame()` and `Render()`. So the DSL is parsed **once**
   into a tree, and the loader installs **one closure that walks that tree every
   frame**, re-emitting `ImGui::*` / `cvc::gl::ui::*` calls. Consequences: every
   value is re-read each frame (nothing is "set once"); IDs derive from
   label+position, so a unit included twice needs an injected `PushID`; hidden ≠
   collapsed ≠ disabled (three attributes); the walker owns `Begin`/`End` pairing.

3. **State-path binding already exists — and everything binds to it now (decision #2).**
   `ui::SliderDouble/SliderInt/Checkbox/MenuItem/Combo/Text` bind to a `cvc::state`
   string path, auto two-way, with the right commit policy (continuous sliders
   commit one write on `IsItemDeactivatedAfterEdit`; checkboxes/combos immediately;
   combos store option **text**). A `{bind: <path>}` needs no compiled variable. We
   promote the demo-local locals (`uiAgents`, `uiFog`, `followAgent`, …) into state
   so this is uniform.

4. **`state_exec` is homoiconic — the AST *is* a public value type (decision #3).**
   `cvc::state_exec` is a sandboxed Lisp on `cvc::state`. The parser produces
   `value_t` (`value_t parse(const std::string&)`, `parser.h:37`), and **every
   evaluator entry point takes `const value_t&` directly** (`create_state(const
   value_t&, env)`, `run(state, max_steps)→value_t`). `value_t` is a public
   `std::variant` (`types.h:75`, `using value_t = value_tag`) with public
   constructors and `make_list`/`make_dict`. **Therefore a nested-YAML expression
   and an s-expr string compile to the byte-identical program with no text
   round-trip** — this is the whole reason decision #3's dual surface is clean.

5. **Actions must be deferred, and `state_exec` gives us the machinery for free.**
   The draw callback runs mid-render, so side-effectful work (rebuild world, restart
   sim, add/remove nodes, reframe camera) must not happen inline. Every demo
   hand-rolls a `want*`-flag intent buffer today. `state_exec` replaces that with a
   long-lived **scheduler** (`int execute(const value_t&, opts)→pid`; `run(max_steps,
   max_time)→map<pid,exit>`; `pause/resume/kill/sleep`, `scheduler.h`) plus a set of
   **UI-action intrinsics that queue intents** the host drains before `tick()`.

Two more constraints on the widget set:

- **Composites already exist as single-token "includes"** — `ui::SceneMenuItems`,
  `ui::ScenePanel`, `ui::StageLightingPanel`, `ui::CameraMenuItems`.
- **`CVC_ENABLE_IMGUI=OFF`** makes the overlay inert (`enabled()==false`); the whole
  draw walk is skipped and the scene still renders. The `state_exec` scheduler + state
  tree keep running headless (so offscreen/wasm capture scripts still drive actions).

> **Grounding (verified in headers):** `parser.h` `parse`/`parse_all`;
> `stackless_evaluator.h` / `async_stackless_evaluator.h` `create_state(value_t)` +
> `run(state, max_steps)`; `scheduler.h` `execute_options{max_steps,max_time}`,
> `execute(value_t)→pid`, `step()`, `run()`; `intrinsics.h`
> `register_intrinsics(env, intrinsics_context*)`; `builtins.h`
> `register_fn(env, name, native_fn)`; `types.h` `native_fn =
> std::function<value_tag(std::span<const value_tag>)>`.

---

## 2. The closed vocabulary (from the demos, not invented)

**Leaf widgets (21).** `text` (formatted / caption / disabled header), `separator`
(+ `separator_text`), `checkbox`, `slider_int`, `slider_float`, `drag_float`,
`input_int`, `combo` (enum-as-text), `radio_row`, `button`, `small_button`,
`menu_item` (toggle / fire-once / radio), `collapsing_header`, `tree_node`,
`color_edit3`, `invisible_button` *(C++ escape)*, `custom_drawlist` *(C++ escape)*.
Plus three **modifiers that are attributes, not children**: `tooltip`, `same_line`,
`disabled_when`.

**Window archetypes (7).** `main_menu_bar`; `control_panel` (title bar + collapse
=minimize + drag + resize, seeded top-left); `library_own_window_panel` (a
composite's own window + close-box); `dense_collapsing_panel` (sectioned,
height-capped); `corner_overlay` (no-decoration, pinned by pivot); `minimap_pip`
(transparent ImGui window hosting a 2nd VTK renderer); `vtk_text_hud`
(FpsHud/ScreenTextHud — VTK actors, not ImGui).

**Action kinds (9).** `toggle_bool`, `set_param`, `raise_window`, `switch_scene`,
`sim_control`, `camera_preset`, `reset_regenerate`, `custom_interaction`,
`load_bundle`.

**Reusable units already recurring** (prime include candidates): five are backed by
`ui::` C++ helpers (`scene_menu`, `scene_panel`, `stage_lighting_panel`,
`camera_menu`, plus `fps_hud`/`text_hud`/`touch_gestures`); the rest are demo-local
(`nav_control_panel`, `minimap_pip`, `sim_controls_menu`, `view_menu`,
`instance_ring_controls`, `apply_restart_footer`) — the ones we'd promote into
parameterized YAML units.

---

## 3. The schema — structure

### 3.1 Document / root

```yaml
ui: 0.2
viewer: main                # SceneRenderer name; the root overlay host
prefix: auto                # cvc::state prefix; auto = scene.getStatePrefix()
overlay:
  toggle_button: true       # built-in bottom-right show/hide circle
  ui_scale: auto            # auto = 1.0 mouse / 2.0 touch; else a discrete step

units: { ... }              # reusable definitions (see 3.7)

menubar:  [ ... ]           # top strip
windows:  [ ... ]           # floating, user-movable panels
overlays: [ ... ]           # corner overlays (pinned)
huds:     [ ... ]           # VTK text/fps overlays
```

`root` is **not** an ImGui window — it is the canvas. `menubar`/`windows`/
`overlays`/`huds` are the four child channels drawn on top of it.

### 3.2 Menu bar and menus

```yaml
menubar:
  - menu: Sim
    items:
      - menu_item: Paused
        bind: sim.paused                        # tick, two-way to a state bool
      - menu_item: Step one frame
        on: sim.step                            # deferred action (see §4)
        enabled_when: (= (state-get "sim.paused") "true")
      - separator
      - menu_item: Restart
        on: sim.restart
  - menu: Scene
    include: scene_menu
    args: { scene: main, panel_open: ui.scene, lighting_open: null }
  - menu: Camera
    include: camera_menu
    args: { cam: fly, move_speed_max: 400, move_speed_default: 40 }
```

`menu_item` + `bind:` = a tick bound two-way to a state bool. `menu_item` + `on:` =
a deferred action (§4). `group:` makes a mutually-exclusive radio set.

### 3.3 Windows and placement policy

```yaml
windows:
  - window: Swarm controls
    id: swarm.controls          # stable ImGui id (the `Title###id` trick)
    pos: [10, 30]               # default coordinate...
    size: [300, 0]              # ...height 0 = auto-fit
    placement: first_use_ever   # seed once, user drag then wins
    open_bind: ui.controls      # optional: state bool shows/hides + close-box
    children: [ ... ]
```

Title bar, collapse (**= minimize**), title-bar drag, and corner resize come **free**
from stock `ImGui::Begin`. `placement:` answers "default coordinate *or* system
policy": `first_use_ever` (seed then user wins), `always` (re-pin every frame — used
by corner overlays), `system` (defer to composite self-placement + the
short-side<700px auto-hide). Persistence across runs is out of scope (decision #4).

### 3.4 Corner overlays

```yaml
overlays:
  - overlay: legend
    corner: top_right           # top_left | top_right | bottom_left | bottom_right
    margin: 12
    children:
      - text: "fps %.0f"
        fmt: (str (float (state-get "hud.fps")))
```

Implicit flags (`NoDecoration|NoBackground|NoMove|NoNav|AlwaysAutoResize`) and pivot
derive from `corner:`, mirroring the built-in `floatingCircleToggle`
(`ImGuiOverlay.cpp` ~L178). **Bottom-right is taken by the toggle button** — a BR
overlay implies `overlay.toggle_button: false`.

### 3.5 Leaf widgets, binding, and modifiers

```yaml
children:
  - slider_int: agents
    bind: sim.count                       # bare state path: direct read/write
    lo: 32
    hi: 4000
  - combo: belief
    bind: sim.belief
    options: [shared, grouped, private]   # static list = literal
    visible_when: (= (state-get "sim.comm.mode") "expert")
  - checkbox: fog (sensing)
    bind: sim.fog
    tooltip: "toggle range-limited sensing"
  - text: status
    fmt: (str-concat (str (int (state-get "sim.count"))) " agents | " (state-get "sim.belief"))
  - slider_float: avoid strength
    bind: sim.sep_gain
    lo: 0.0
    hi: 5.0
    fmt: "%.1f x rr"
    disabled_when: (not (= (state-get "sim.separate") "true"))
  - button: Apply / Restart
    on: sim.restart
```

**Binding is one of two forms:**

- `bind: <state-path>` — the common case. A **bare path is a direct `cvc::state`
  read/write on the draw thread** (no evaluator — that would be pure overhead): the
  widget seeds from `node->value()` each frame and commits edits via the coalesced
  publisher. Leading `/` = absolute; else spliced onto the resolved `prefix` via the
  canonical helpers, never string-concat.
- `bind: <expression>` — a computed/multi-value read; a `state_exec` program (§4)
  evaluated read-only each frame.

`value: {id, default}` (a loader-owned value store) survives only for genuinely
transient UI with no state home; decision #2 makes it rare.

**Modifiers are attributes, not children** (they depend on ImGui submission order):
`tooltip:`, `same_line:` (bool or `{offset, spacing}`), `disabled_when: <expr>`
(BeginDisabled greying), `enabled_when: <expr>` (sugar for `disabled_when: (not …)`),
`visible_when: <expr>` (don't emit this frame), `start: collapsed|open`. Every
`*_when` / `fmt` / dynamic `options` value is a `state_exec` expression (§4).

### 3.6 Composite drop-ins (single token → whole surface)

```yaml
  - panel: scene            # → ui::ScenePanel
    scene: main
    open_bind: ui.scene
  - panel: stage_lighting   # → ui::StageLightingPanel
    rig: city
    open_bind: ui.lighting
  - menu: camera            # → ui::CameraMenuItems (inside a menubar)
    cam: fly
```

These expand to entire control surfaces with their internal binding handled in C++.

### 3.7 Reusable units: `include` (static) vs `repeat` (dynamic)

**`include` — static expansion at load, with args** (for fixed templates):

```yaml
units:
  nav_control_panel:
    params:
      - { name: title }
      - { name: count_label }
      - { name: count_range, default: [32, 4000] }
      - { name: count_default, default: 800 }
    body:
      - window: "{{title}}"
        id: nav.controls
        pos: [10, 30]
        size: [300, 0]
        children:
          - slider_int: "{{count_label}}"
            bind: sim.count
            lo: "{{count_range[0]}}"
            hi: "{{count_range[1]}}"
          # ...the rest, identical between swarm and drive...

windows:
  - include: nav_control_panel
    args: { title: "Convoy controls", count_label: vehicles,
            count_range: [6, 600], count_default: 60 }
```

Args substitute into labels **and** bind-path prefixes (`{{arg}}`). A **null arg
drops its guarded node**. Includes have a max recursion depth; each instance gets an
auto-`PushID` so two includes of the same unit don't collide.

**`repeat` — runtime per-frame repetition** (lists whose length is only known at
runtime), with a `state_exec` count expression and per-index `PushID`:

```yaml
  - repeat:
      count_bind: { $int: rig.light_count }   # a state_exec expr (nested-YAML form)
      as: i
      body:
        - checkbox: "{{rig.light_name[i]}}"
          bind: "rig.light[{{i}}].enabled"
        - small_button: solo
          on: { rig.solo: [ {$: i} ] }
          same_line: true
```

### 3.8 The escape hatch: `custom:`

The minimap drag / click-to-follow / drawlist dots and app-driven HUD text are
irreducibly imperative. They become opaque, host-implemented nodes registered via
`builtins::register_fn`; a `custom` **draw** leaf runs inside the walk (may touch
ImGui directly, it's on the draw thread), a `custom` **action** queues an intent
like any `on:`. Opaque handles ride as a first-class `data_object` value.

```yaml
  - custom: minimap
    args: { targets: 6 }
```

---

## 4. Expressions & actions = `state_exec`

This is the heart of v0.2. Every `*_when`, `fmt`, dynamic `options`, `repeat.count`,
computed `bind`, and every `on:`/`on_change:` action is a `state_exec` program over
the **same state tree the widgets bind to**.

### 4.1 One AST, two lanes

Selection is **by slot class, not by expression content** — all evaluators are
result-identical over the same `value_t` AST, so this is a driver choice.

- **Read-only / per-frame lane** — `bind`(expr), `fmt`, `visible_when`,
  `disabled_when`, `enabled_when`, `options`, `tooltip`, `repeat.count`. Evaluated
  **synchronously inside the draw walk** on one long-lived `stackless_evaluator`
  seeded with a **read-only environment** (builtins + state *read* intrinsics + pure
  stdlib; **no** effectful intrinsics bound — a stray write fails at load). Run with
  `run(state, max_steps=CAP)`; if `state.done==false`, **fail-safe per slot**
  (`visible/enabled→false`, `fmt→""`, `options→last-good`) and log once. A frame is
  never blocked.
- **Effectful / action lane** — `on:`, `on_change:`. Compiled to `value_t` at load;
  at fire time **submitted** to a long-lived scheduler (`execute(ast, opts)→pid`),
  never run inline; **stepped/drained off the callback** with a per-frame budget.
  Per-program `max_steps`/`max_time`/`max_memory` auto-kill a runaway handler at a
  step boundary.

### 4.2 Homoiconic dual surface (s-expr *or* nested YAML)

Because the AST is `value_t`, both surfaces build the identical program:

- **s-expr string** → `parse(str)`. Verbatim Lisp:
  `(> (float (state-get "sim.count")) 100)`.
- **nested YAML** → a `yaml_to_value()` builder emits `value_t` via the public
  ctors/helpers (the same recursion the state codec already does), sourced from a
  YAML node. No rendering to text.

Canonical nested-YAML shape (designed to kill the symbol-vs-string ambiguity):

| Nested YAML | s-expr | Meaning |
|---|---|---|
| `{<op>: [a, b]}` | `(<op> a b)` | call; the single key is the operator **symbol** |
| `{do: [s1, s2]}` | `(begin s1 s2)` | statement sequence |
| `{$: p}` | `(state-get "p")` | state read (**string**) |
| `{$int: p}` / `{$float: p}` / `{$bool: p}` | `(int (state-get "p"))` … | typed state read |
| `{set: [p, v]}` | `(state-set "p" (str v))` | state write |
| scalar | literal | int→int64, float→double, bool→bool, null→nil, string→**string literal** |
| `!sym foo` | `foo` | explicit symbol in argument position (rare) |

Equivalence is a unit-test invariant: `values_equal(parse(sexpr),
yaml_to_value(yaml))` for every documented pair.

### 4.3 Two caveats the schema must front (both real footguns)

- **State is string-typed on the flat channel.** `state-get` returns a **string** (or
  nil); `state-set` coerces to string. A promoted int tunable is stored as `"800"`, so
  a numeric predicate must wrap the read in `int`/`float`. **Resolution (Q1): every
  bind-path carries a declared `type:`** (`int`/`float`/`bool`/`string`/`enum`), so the
  widget, `fmt`, and predicate all agree and the loader inserts the right coercion; the
  YAML `{$int:}`/`{$float:}` sugar remains for ad-hoc reads. **Typed/structured
  tunables route through a state_object's typed `data()` channel** (the `state-data-set`
  / `state-data-get` intrinsics + `data_object` value) rather than the flat string
  channel — so a color, vector, or transfer-function tunable round-trips as a real typed
  blob instead of a stringified scalar.
- **Truthiness is Lisp-ish, not C.** Only `nil` and `#f` are falsy; `0`, `0.0`, `""`,
  and the empty list are **all truthy** (and `#f` parses to nil, not bool-false). So a
  bare `(state-get p)` as a bool test is almost always wrong — predicates must be
  explicit comparisons. The loader lints for this.

### 4.4 Host intrinsics — the UI action verbs

Registered on the executor (`register_intrinsics` for the built-in state ops;
`builtins::register_fn` for ours). **Every effectful intrinsic QUEUES an intent**
drained before `tick()` on the correct (sim/render) thread — none mutates GL/camera/
sim directly off-thread (the intrinsics context and state root are not locked; direct
off-thread mutation would corrupt state).

| Intrinsic | Effect |
|---|---|
| `(sim.pause)` / `(sim.resume)` / `(sim.toggle-pause)` | queue pause/resume |
| `(sim.step [n])` | queue n-frame advance (paused only) |
| `(sim.restart)` / `(sim.seed [n])` | queue rebuild / reseed-restart |
| `(scene.regenerate [seed])` | queue world rebuild (heavy; may `await`) |
| `(scene.set "<ref>")` | queue switch-scene / act-jump |
| `(window.raise "id")` / `(window.hide "id")` | queue raise/hide — or just `state-set` the `open_bind` (callback-safe) |
| `(camera.fit)` / `(camera.ortho [b])` / `(camera.chase [target])` | queue reframe / mode change on render thread |
| `(ui.scale <step>)` | queue discrete UI-scale change (also persists via UiSettings) |
| `state-get/-set/-exists/-children` | **existing** — the promoted-tunable bridge; `state-set` is the one effect safe from the callback |
| `state-watch/-unwatch` | **existing** — backs `on_change:` |
| `custom.<name>` | host-registered escape (action or read-only query) |

### 4.5 Event model — `on:` is polymorphic; the scheduler replaces `want*`-flags

`on:` dispatches by the **shape** of its value (decided at load):

- **Bare enumerated event** — a scalar naming a registered event (`on: sim.restart`).
  **Fast path**: maps straight to the host handler / directly enqueues the intent. No
  evaluator, no process, zero per-fire allocation. This preserves an audited, closed
  set of event names as the cheap, reviewable common case.
- **Expression / program** — an s-expr string (leading `(`) or a nested-YAML
  `call`/`{do:}` (`on: {do: [{sim.pause: []}, {set: [sim.speed, 1.5]}]}`). **General
  path**: compiled to `value_t`, submitted with `execute(ast, opts)→pid`, run later
  off the callback. Full grammar: `if`/`let`/loops/`(await …)`/any intrinsic.

The same intrinsic symbols back both paths (`sim.restart` is *both* the enumerated
name *and* the intrinsic), so `on: sim.restart` ≡ `on: (sim.restart)`. Authors start
with bare names and reach for programs only when they need logic.

**What replaces the hand-rolled `want*`-flag queue:** three shared pieces owned by the
UI runtime — (a) one long-lived scheduler owning all deferred action programs; (b) one
thread-safe intent queue; (c) the UI-action intrinsics that queue intents. Firing an
action only ever enqueues; the host calls one `drain()` before `tick()`. Simple flag
toggles that used to be `want*`-bools become plain `state-set` writes (callback-safe).
N ad-hoc protocols collapse to one scheduler + one queue + one drain.

**`on_change:`** is a reactive handler over a state path, backed by the scheduler's
watch mechanism (`state-watch`; polled per scheduler step — no `boost::signals2`, so
macOS-safe), same intent discipline.

```yaml
- custom: comm-reactor
  on_change:
    path: sim.comm.enabled
    do:
      - { if: [ {$bool: sim.comm.enabled}, { camera.chase: [] }, { camera.ortho: [true] } ] }
```

**Input events** (`on:key` — wasm keyboard incoming; `on:tick` per-frame) use the same
model: a handler program enqueues intents the tick drains. wasm is single-threaded, so
the handler runs inline in the input pump and the queue discipline keeps it safe.

### 4.6 Executor selection + per-frame lifecycle + degradation

- **Compile (load, once):** parse the document; compile each expression slot to
  `value_t` (s-expr via `parse()`, YAML via `yaml_to_value()`); classify the slot and
  validate against its environment (read-only slots reject effectful symbols at load);
  cache the `value_t` and pool a reusable `evaluator_state` for hot per-frame slots.
- **Per frame (draw callback):** the walk evaluates every read-only slot on the one
  long-lived sync `stackless_evaluator` via `run(state, CAP)`, fail-safe on
  CAP-exceeded. State reads hit `cvc::state` directly; widget writes go through
  `state-set` (coalesced publisher). No scheduler touched on the draw thread.
- **Action fire (during the walk):** never inline. Fast path enqueues an intent;
  general path calls `execute(ast, opts)→pid`.
- **Main loop / `tick()`:** (1) `host.drain()` performs queued imperative work on the
  owning thread **before** tick; (2) sim advances; (3) fresh state published; (4) the
  action scheduler drained with a per-frame budget (`run(max_steps=BUDGET)` or a
  bounded `while(has_runnable() && n++<BUDGET) step()`) so a long action spreads over
  frames. The scheduler is **long-lived across frames** (never rebuilt), so
  `evaluator_state`, sleep timers, watch last-values, and messages survive.
- **Which action driver (Q2 — async is the target):** the action lane is built on the
  **async schedulable executor** (`async_stackless_evaluator` + `async_scheduler`), and
  we invest to make it **fully first-class early** — even where that means *adding the
  missing functionality* today's `async_scheduler` lacks (sleep, inter-process
  messaging, state-configured settings, and true intra-step preemption; today it
  internally steps a sync stackless evaluator and yields only between scheduler steps).
  `await` is therefore available to actions from day one (e.g. a long
  `scene.regenerate` suspends instead of consuming drain budget). The sync scheduler
  stays as a fallback only for a build without the async/coroutine path; the two are
  result-identical over the same `value_t`, so this is a driver choice, not a semantic
  one. **Bringing the async executor to parity is a P0/P1 workstream, not a P4 nicety.**
- **Degradation:** `CVC_ENABLE_IMGUI=OFF` → draw walk skipped, no predicate/`fmt`
  evaluated, scene still renders; scheduler + state tree still run headless so
  non-UI-triggered actions keep working. Python host → identical `value_t`/scheduler
  model (both hosts run the *same* libcvc `state_exec`), covers the declarative subset,
  delegates `custom` draw leaves to C++ — no semantic drift.

### 4.7 Sandboxing — per-panel chroot (Q4)

Each panel / included unit runs its programs under an **`apply_chroot` to its own state
prefix**: a unit's expressions and actions see `some.tunable` resolved *relative to that
panel's subtree*, so a reusable unit can't accidentally read or clobber a sibling's
state, and two includes of the same unit are naturally isolated. **Chroots nest** — an
included unit inside a panel chroots again under the parent's prefix, matching the
include tree.

Because a chroot hides everything outside the panel's subtree, cross-panel actions get
an **explicit escape hatch**: a leading `/` (or a `{root: …}` form) addresses an
absolute state path, and the enumerated window/scene intrinsics (`window.raise "id"`,
`scene.set`, …) operate on document-global targets by design. So the default is
sandboxed and local; reaching another panel is possible but must be written explicitly,
never by accident. (Chroot granularity — one root per document vs strict per-panel — is
now settled as **per-panel, nestable, with the absolute-path escape**.)

---

## 5. Worked examples

### A — `bunny_shadow` (fully declarative)

```yaml
ui: 0.2
viewer: main
prefix: auto
menubar:
  - menu: Scene
    include: scene_menu
    args: { scene: main, panel_open: ui.scene, lighting_open: ui.lighting }
windows:
  - panel: scene
    scene: main
    open_bind: ui.scene
  - panel: stage_lighting
    rig: bunny
    open_bind: ui.lighting
```

~15 lines reproduce the Scene menu, Scene panel, and the full Stage-lighting panel —
all inside the two `ui::` composites.

### B — `nav_city_swarm` **and** `nav_city_drive` from one unit + 8 args

The two demos are byte-identical except **8 parameters**. Each becomes ~10 lines of
`args:` over one shared `nav_scene` unit:

```yaml
# nav_city_drive.ui.yaml
ui: 0.2
viewer: main
include: nav_scene
args:
  title: "Convoy controls"
  noun: vehicles
  count_label: vehicles
  count_range: [6, 600]
  count_default: 60
  targets: 6
  status_fmt: "%d vehicles · %d convoys · belief %s"
```

`nav_scene`'s body is the shared UI: `sim_controls_menu` + `view_menu` + `scene_menu`
+ `camera_menu`; the `nav_control_panel` window; a `minimap_pip` (`custom`) with
`targets`; a `text_hud` status line; and the two `panel:` composites.

### C — expressions & actions in action

```yaml
- menu: Sim
  items:
    - menu_item: Paused
      bind: sim.paused
    - menu_item: Step
      on: sim.step
      enabled_when: { "=": [ {$: sim.paused}, "true" ] }     # nested-YAML predicate
    - menu_item: Regenerate world
      on:                                                     # program on the async lane
        do:
          - { if: [ { "=": [ {$: sim.paused}, "true" ] },
                    { await: [ { scene.regenerate: [ {$int: sim.seed} ] } ] },
                    { sim.restart: [] } ] }
- button: Apply / Restart
  on: |                                                       # s-expr form, same result
    (begin (state-set "sim.speed" (str 1.5))
           (state-set "sim.paused" "false")
           (sim.restart))
```

---

## 6. Coverage — honest scope

~7 of 10 demos are fully declarative (`bunny_shadow`, `volren_bunny`,
`volslice_bunny`, `terrain_lab`, `lsystem_forest`, `lsystem_coast`). The 3 nav demos
(`nav_city_swarm/drive`, `nav_finale`, `nav_fog_ghost`) each need one `custom:` node
(the minimap's direct-manipulation canvas and app-driven HUD text). No demo is
unreachable. The Python host covers the declarative subset (no C++ `custom_drawlist`/
`invisible_button`); the C++ host covers everything.

---

## 7. Open questions

**Resolved (this round):**
- **Q1 — typing:** ✅ per-path declared `type:` + coercion at the state boundary;
  typed/structured tunables route through a state_object's `data()` typed channel
  (§4.3).
- **Q2 — action driver:** ✅ build on the **async schedulable executor** and bring it to
  full parity early (add the missing sleep/messaging/settings/preemption as needed);
  `await` available from day one. Sync scheduler is a build fallback only (§4.6).
- **Q4 — sandboxing:** ✅ per-panel `apply_chroot`, **nestable**, with an absolute-path
  (`/…`) + enumerated-intrinsic escape hatch for cross-panel communication (§4.7).

**Still open (state_exec-specific):**
1. **`on:tick`/`on:key` handlers** — re-submit per event (simple) vs one resident
   process per handler that `await`s the event (lower overhead; now viable since we're
   investing in async parity — ties to Q2's messaging/watch work).
2. **Intent transport** — a dedicated typed thread-safe queue vs writing request nodes
   into the state tree (zero new plumbing, but stringifies structured intents; the typed
   queue is cleaner for imperative ops that carry handles).
3. **Predicate env surface** — expose stdlib (string/math/collections) to authors by
   default, or keep the per-frame surface minimal to bound cost and review.
4. **Budgets** — concrete `CAP` (per-frame read-only) and `BUDGET` (per-tick action
   drain), profiled against the heaviest demo (terrain_lab panel).
5. **Error surfacing** — silent fail-safe + log-once (ship-safe) vs an in-UI error
   badge on the node (authoring aid).

---

## 8. If we build it — suggested phasing

- **P0 — declarative core + state binding:** document/root, menubar, windows, leaf
  widgets with `bind: <path>`, modifiers as literals, corner overlays, HUDs; promote
  the target demos' tunables to `cvc::state`. Ship `bunny_shadow`, `terrain_lab`,
  `lsystem_*`, `volren/volslice` from YAML. C++ loader in libcvc; `enabled()==false`
  no-op path.
- **P1 — state_exec read-only lane:** `yaml_to_value()` + `parse()` dual surface with
  the round-trip test; `visible_when`/`disabled_when`/`enabled_when`/`fmt`/`options`/
  `repeat.count` on the sync `stackless_evaluator` with `CAP` + fail-safe; the `{$int:}`
  sugar and predicate linter.
- **P2 — state_exec action lane:** the long-lived scheduler + intent queue + UI
  intrinsics; `on:` fast path (enumerated) and general path (program); `drain()` before
  `tick()`; `on_change:` via watches. Standardize the deferred-apply that demos
  hand-roll.
- **P3 — composition + escape hatches:** `units`/`include` (args, null-drop, PushID,
  recursion guard), `repeat`; `custom:` host nodes (minimap). Ship swarm/drive-from-
  one-unit.
- **P4 — Python/CLI entry** over the C++ loader; a `cvc`/`grl-snam`-style command to
  launch a `.ui.yaml` against a scene; `await`/async adapter if any action needs it.
```
