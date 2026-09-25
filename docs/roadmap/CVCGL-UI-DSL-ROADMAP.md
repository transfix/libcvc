# cvcGL UI DSL — Scoping Spec (v0.6, for iteration)

Status: **draft for discussion, no code committed.** Grounded in a full survey of
the ImGui infrastructure (`ImGuiOverlay`, `ImGuiBinding`, `SceneRenderer`,
`RenderView`, `Settings`, `StageLighting`), all 14 cvcGL examples, and the
`cvc::state_exec` subsystem — all on `origin/master` (`4fb4e1198`). The goal is a
YAML description of the nested ImGui widget tree — composable, reusable-with-args,
with expressions and actions expressed in `state_exec` — that can express the
existing demo UIs. This document is the thing we iterate on before writing a loader.

> **v0.4:** the scene graph (§9) now makes **`RenderView` + `VisibilityMask` first-class
> from the start** — each view renders a *masked and restyled subset* of one authored scene
> (§9.5), the loader compiling one ownership-tree declaration into both the state tree and a
> generated traversal graph (a bridge, adopted in stages C→B→A). New **§10 Transfer
> functions**: control nodes *or* a raw table, on the `data()` typed channel, 1D wired with
> the nD (2D/3D) shape reserved per the volrover3 roadmap, and a ColorTable2-derived
> `tf_editor` widget. (v0.3 folded in decisions Q1/Q2/Q4 at §4.3/§4.7/§4.8; the minimap is a
> `view_embed`, §9.6.)
>
> **v0.5** resolves the §9.5 details: overrides lower to **duplicated masked branches** (robust,
> no engine change), **last-wins** precedence with `mode: replace|merge`, **dynamic masks** via
> `state_exec` expressions, and **dirty-propagated** incremental regen.
>
> **v0.6** closes §7: `on:tick`/`on:key` = **resident awaiting processes** (§7.1), a **typed
> intent queue** (§7.2), stdlib-by-default with **reduced-env + per-handler `limits:`** sandboxing
> (§7.3, §7.7), **concrete tunable budgets** (§7.4), **dual error surfacing** (§7.5), and a new
> **`requires:` load-time capability preflight** (§7.6) so a UI that calls a missing intrinsic
> fails at load, not mid-handler. §7 also names the `state_exec` gaps the runtime must close
> (async resident-await port, the frame-aborting missing try/catch, the dead `max_memory` lever)
> — now sequenced in §8. Only the scene streamed-source contract stays open.

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

**Query intrinsics (reads, not effects).** A second, small family that **returns a
value instead of queuing an intent** — so, unlike the verbs above, they are registered in
**both** the read-only predicate env and the action env and are safe to call inline:

| Query intrinsic | Returns |
|---|---|
| `(pick.world "<view>" px py)` | world `(x y)` under a pixel in that view — the generalized `pip_world_at` unproject through the view's camera (needs a rendered frame) |
| `(pick.node "<view>" px py [radius])` | the **name** of the nearest scene node to that pixel (nil if none within `radius`) — the nearest-agent hit-test, generalized |
| `(scene.bounds ["<node>"])` | the `(minx miny minz maxx maxy maxz)` of a node/scene (feeds `camera.fit`/framing) |

`pick.*` take a **view name** (§9.5) so the same pixel resolves differently per viewport —
which is exactly what the minimap needs (a click in the top-down inset unprojects through
the inset's ortho camera, not the main perspective one). They are the first-class
mechanism (decision, §9.8) that makes the minimap's click-to-follow / target-drag ordinary
`on:` actions instead of a `custom` node.

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

### 4.6 Pointer & input events (widget-level `on_*` handlers)

Beyond `on:` (fire on activate), a node may carry pointer/input handlers:
`on_click`, `on_drag` (+ `on_drag_start`/`on_drag_end`), `on_hover`, `on_key`, `on_tick`.
These are the general mechanism the minimap (§9.6) uses; they exist for any node.

Each fires the same two-shape `on:` value (enumerated name or program). The trigger's
**payload is written into a per-handler event scope** the program reads by relative path,
rather than a global — so it composes with the chroot (§4.8):

- `event.x` / `event.y` — pointer position, in the node's own coordinate space (for a
  `view_embed`, that is the embedded **view's** pixel space, which is what `pick.*`
  wants; for a plain widget, its content-rect-local pixels).
- `event.dx` / `event.dy` — drag delta since the last frame (drag handlers only).
- `event.button` / `event.key` / `event.mods` — which button/key/modifiers.
- `event.dt` — seconds since last frame (`on_tick` only).

So the minimap's click-to-follow reads `(pick.node "overview" {$int: event.x} {$int:
event.y})` and target-drag reads `(pick.world "overview" …)` — no globals, no `custom`.
Handlers that only write state (`set`) run inline (callback-safe); handlers that call
effectful intrinsics enqueue as usual. **Open (ties to §7.1):** whether a hot `on_tick`
is re-submitted each frame or runs as one resident `await`-ing process — deferred with the
other `on:tick`/`on:key` cadence questions.

### 4.7 Executor selection + per-frame lifecycle + degradation

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

### 4.8 Sandboxing — per-panel chroot (Q4)

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

With the widget DSL alone, ~7 of 10 demos are fully declarative (`bunny_shadow`,
`volren_bunny`, `volslice_bunny`, `terrain_lab`, `lsystem_forest`, `lsystem_coast`) and
the 3 nav demos each need one `custom:` node for the minimap. **Adding the scene-graph
section (§9) closes that gap:** the minimap becomes a declarative `view_embed` over a
mirror viewport, so **all 10 demos are describable.** The only residual host hooks are
narrow and well-fenced — streamed per-frame geometry (`source: stream`) and fully custom
GLSL (`shaders:`). The Python host covers the declarative subset (no C++
`custom_drawlist`/`invisible_button`); the C++ host covers everything.

---

## 7. Open questions — resolved

Prior rounds resolved Q1 (typing, §4.3), Q2 (async driver, §4.7), Q4 (chroot, §4.8). The
five state_exec-specific questions are now closed, plus one new requirement (`requires:`, §7.6).
Each resolution is grounded in the real `state_exec` code — including the **gaps a UI runtime
must close** (flagged inline; they feed §8 phasing).

### 7.1 `on:tick` / `on:key` handlers — one resident awaiting process per handler ✅

Each `on:tick`/`on:key`/`on_*` handler is compiled once and submitted **once** as a single
long-lived process that parks on an event and is woken per fire — *not* re-submitted
(`execute(ast)→pid`) every frame/keystroke. This kills the per-fire allocation/re-parse on the
hottest paths and matches the "long-lived scheduler across frames" contract (§4.7). The body
compiles to an await-loop `(while t (let ((event (msg-recv "ui.ev.<node>.<kind>"))) …))`:
`msg-recv` suspends the process (`scheduler::receive_message` sets `recv_path`, status →
`waiting`; `select_process` skips it — it does **not** poll); the host wakes it by draining the
typed queue (§7.2) into `deliver_to_receivers(path, event)`, which patches the parked result and
marks the process `ready`. The per-fire payload (`event.x`/`event.dt`, §4.6) rides *as the
message value*, so the per-handler event scope falls out of the same mechanism.

> **Gap (P0/P1, not a nicety).** The resident-await pattern is proven on the **sync scheduler**
> (tests `ReceiveMessageSuspendsProcess`, `DeliverToReceiversWakesProcess`, `SleepAndWake`).
> The **async lane §4.7 targets does not have it**: `async_scheduler` lacks
> `receive_message`/`deliver_to_receivers`/`sleep`, and `intrinsics_context.sched` is hard-typed
> `scheduler*` with no shared base, so intrinsics can't even bind to the async lane. Bringing
> residents to the async executor *is* the sleep/messaging/preemption parity work already scoped
> P0/P1 in §4.7 (port `recv_path`/`inbox`/`deliver_to_receivers`; re-type `sched` to a base).
> Until then residents run on the sync fallback — result-identical over the same `value_t`.

### 7.2 Intent transport — a dedicated typed thread-safe queue ✅

A dedicated **typed, thread-safe intent queue**, not request-node writes into state. It carries
`{intent-kind, args (value_t / native handle), origin-pid, origin-path}` records, drained by
`host.drain()` **before** `tick()` (§4.7). Two reasons over state-node writes: (a) imperative
ops legitimately carry **live handles** (a `view*`, a picked node id, a GL resource) that a
`state-set` would have to stringify and reconstitute — lossy/unsafe; the typed queue moves them
by value/pointer; (b) it preserves the ordering/identity of effects that state coalescing would
collapse. Simple flag toggles stay plain `state-set` writes (§4.5). The same queue is the
**wake channel for §7.1** — draining a UI event calls `deliver_to_receivers` for that handler's
resident process — so one mechanism carries both host→handler events and handler→host effects.

### 7.3 Predicate / action env — stdlib by default, reduced env + limits as the safety valve ✅

Expose the **full stdlib (string/math/collections) by default** in *both* the read-only predicate
env and the action env; bound cost/abuse with per-program limits (§7.7) and a **reduced env**,
not a permanently thin surface. `create_state(expr, env)` uses a non-null `execute_options.env`
**directly** with no fallback to the builtins global, so a hand-built env is a true sandbox; a
locked handler gets an env cloned to only whitelisted names (`import_module` also takes a
`specific_fns` subset filter).

> **Caveat that shapes §7.6/§7.7:** special forms (`if`/`let`/`lambda`/`defun`/`eval`/`quote`)
> are name-dispatched **before** any env lookup, so they **cannot be revoked by env pruning**.
> Denying dynamic code (`eval`/`lambda`/`defun`) in a locked handler needs an evaluator-level
> denylist keyed on a "restricted" flag, not a thinner env.

### 7.4 Budgets — concrete but tunable, profiled against the heaviest demos ✅

Two distinct budgets (`scheduler::run(max_steps,max_time)` is the shared per-*tick* advance;
`process.max_*` are per-*handler* ceilings — see §7.7):

| Budget | Where | Provisional default | Tunable via |
|---|---|---|---|
| **CAP** — per-frame read-only walk | `run(state, CAP)` over all predicate/`fmt` slots | **50,000 steps/frame**, ~**2,000/slot**, soft **2 ms** | `state_exec.defaults.frame_cap_steps` |
| **BUDGET** — per-tick action drain | bounded `run(max_steps=BUDGET)` | **20,000 steps/tick**, soft **4 ms** | `state_exec.defaults.tick_budget_steps` |
| **Per-handler / activation** | `execute_options.max_*` | **5,000 steps**, **1 ms**, **256 KiB** *per activation* | `state_exec.schedulers.<id>.*` |

All numbers are **provisional pending profiling** against **`terrain_lab`** (predicate-dense),
**`lsystem_coast`**, **`nav_city_swarm`**. Tunability is already wired: `scheduler::load_settings()`
resolves `state_exec.schedulers.<id>.<key>` → `state_exec.defaults.<key>` → fallback, so budgets
are hot-adjustable from the state tree. Note `max_steps` counts evaluator **micro-steps**, so a
single native `map`/`reduce` over a big list is one un-preemptible step — a wall-time hazard to
watch in profiling that step-count alone won't bound.

### 7.5 Error surfacing — both silent fail-safe + log AND an in-UI badge ✅

Both, complementary:

- **Silent fail-safe + log (always on).** A limit breach already `kill_process(proc, reason)` →
  `status=killed`, `exit_error` (`max_steps_exceeded`, `time_limit_exceeded`, …); the tick keeps
  running for every other handler. **Required fix:** `execute_process_step` calls `evaluator_.step()`
  with **no try/catch**, so an undefined-symbol / runtime throw from one handler propagates out of
  `run()` and **aborts the whole frame** — wrap it and convert a throw into
  `kill_process(proc, "error: "+what)`. Log `exit_error` once per (handler, reason).
- **In-UI badge (authoring aid, flag-gated).** Behind `ui.debug.badges` / `CVC_UI_DEBUG`, the node
  owning a killed handler renders a small error badge reporting the reason, node/handler id, and
  (for `error:`) the thrown `what()` — read from a typed error record pushed onto the intent queue
  (§7.2, cross-thread-clean) rather than racing `get_process_info` on a just-killed pid. Off by
  default in shipped demos.

### 7.6 `requires:` — capability declaration (load-time preflight)

**Problem.** Symbols resolve at **eval time**, and an unbound one **throws** (`"undefined symbol:
…"`) — never nil. So a user-extended demo or custom UI that calls an intrinsic missing in *this*
build/host (or pruned out of *this* handler's reduced env) fails **mid-handler on first fire**,
deep in a frame (and, per §7.5, currently aborts the frame). `requires:` moves that to **load**,
fails fast, and names exactly what is missing.

**Contract.** A program / handler / unit may declare `requires: [name, …]`. At load — after the
handler's target env is assembled — the loader checks each name against the **exact env that
handler will run in** (read-only walk env for predicates, action env for `on:`/`on_*`, the
*reduced* env if sandboxed, §7.3): `ok(n) = target_env.lookup(n) != null || is_special_form(n)`.
Checking the *target* env (not the global builtins) also catches "exists in the build but pruned
out of this sandbox." Failures are collected into **one** precise error — name(s), where declared
(document / unit / node id), which env. Preflight is pure lookup: cheap, deterministic.

- **Auto-derive (default when `requires:` absent):** statically walk the compiled `value_t` tree
  for applied head symbols, then subtract to get *free* heads — exclude special forms, **stop at
  `(quote …)`**, subtract `lambda`/`defun`/`let`/`for`/`set` bound names and self-defined
  functions.
- **Explicit `requires:` (authoritative):** the scan is a **lower bound** — blind to
  macro-expanded heads and **indirectly-applied** names (a symbol handed to `apply`/`map`/`send`).
  An explicit list adds what the scan misses; **strict mode** warns when `scanned ⊄ (declared ∪ env)`.
- **Scope:** per-program (one slot), per-unit (an `include`/`repeat` unit, checked in its chroot
  env), and whole-document (a top-level gate: "this custom UI needs intrinsics your build lacks").

```yaml
ui: 0.2
requires: [ state-get, state-set, pick.world, scene.set ]   # whole-doc gate: fail load if build lacks any
windows:
  - custom: minimap_pip
    on_drag:
      requires: [ pick.world ]        # explicit
      do:
        - { set: [ sim.target, { pick.world: [ "overview", {$int: event.x}, {$int: event.y} ] } ] }
    on_hover:                         # no requires: → auto-derived {pick.node}; runs in a reduced read-only env
      do:
        - { set: [ ui.hover.node, { pick.node: [ "overview", {$int: event.x}, {$int: event.y} ] } ] }
```

### 7.7 Per-handler `limits:` & sandboxing

Each handler spawns with an `execute_options` carrying its own ceilings; `check_limits(proc)`
runs **after every micro-step** and, on first breach, `kill_process(proc, reason)` — leaving the
tick running for others (this **is** the §7.5 fail-safe).

| Field (0 = unlimited) | Meaning | Enforcement state |
|---|---|---|
| `max_steps` | evaluator micro-steps | **live** |
| `max_time` (sec) | wall-clock over running slices | **live** |
| `max_memory` (bytes) | per-pid state-write bytes | **DEAD lever — must wire first** |
| `max_messages` / `max_message_bytes` | `msg-send` accounting | live |

Three gaps a UI runtime must close: **(a)** `max_memory` never fires — `memory_tracker::record_write`
has zero production callers, so `state-set` intrinsics must call it before the budget means
anything; **(b)** `check_limits` is **lifetime-cumulative**, so a resident handler (§7.1) would
self-terminate after a few frames under a raw `max_steps` — capture `{steps0,time0}` at each wake
and bound the **per-activation** delta (keep the lifetime limit as a coarse backstop); **(c)** a
bare `scheduler::execute()` bypasses `resource_policy` (defaults/clamps apply only on the
`exec_coordinator::submit` path), so the runtime must submit through the policy or apply §7.4
defaults itself.

```yaml
- custom: expensive_overlay
  on_tick:
    limits: { max_steps: 5000, max_time: 0.001, max_memory: 262144 }   # per activation
    env: read_only               # reduced env: state-get + stdlib; NO state-set/spawn/kill
    requires: [ state-get ]
    do:
      - { set: [ ui.overlay.fps, { fmt: [ "%.1f", {$: sim.fps} ] } ] }
```

Reduced env + limits + `requires:` compose into a real sandbox for an untrusted/user-added
handler: the env is a **capability floor** (the handler literally can't name `state-set`/`spawn`);
a `restricted` flag adds the evaluator-level special-form denylist (§7.3) to deny `eval`/`lambda`/
`defun`; `max_*` are the **cost ceiling**; the try/catch wrap (§7.5) is **fault containment**; and
the kill's `exit_error` feeds both the log and the badge.

**Remaining sub-questions** (fine-grained, not blocking): kill-and-respawn vs deactivate-and-reset
for a resident that overruns its activation; whether `max_memory` wiring is in the first cut or
ships documented-but-inert; and channel-name uniqueness for `ui.ev.<node>.<kind>` across nested
chroots.

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
  sugar and predicate linter; the **`requires:` preflight** (§7.6) and per-handler
  **`limits:`** (§7.7); the **`execute_process_step` try/catch wrap** so one bad handler
  can't abort the frame (§7.5).
- **P2 — state_exec action lane:** the **typed thread-safe intent queue** (§7.2) + UI
  intrinsics; `on:` fast path (enumerated) and general path (program); `drain()` before
  `tick()`; `on_change:` via watches; **resident awaiting processes** for `on:tick`/`on:key`
  (§7.1) with the **per-activation limit baseline** (§7.7). Standardize the deferred-apply
  that demos hand-roll.
- **P2-prereq — `state_exec` parity (the gaps this review surfaced):** port
  `receive_message`/`deliver_to_receivers`/`sleep` onto `async_scheduler` and re-type
  `intrinsics_context.sched` to a shared base (unblocks residents on the async lane, §7.1);
  wire `memory_tracker::record_write` into the `state-set` intrinsics (activates `max_memory`,
  §7.7); the `restricted`-flag special-form denylist for sandboxed handlers (§7.3). These are
  libcvc `state_exec` changes the DSL depends on — sequence them with P1/P2.
- **P3 — composition + escape hatches:** `units`/`include` (args, null-drop, PushID,
  recursion guard), `repeat`; `custom:` host nodes (minimap). Ship swarm/drive-from-
  one-unit.
- **P4 — scene graph (§9):** the `scene:` block — nodes/sources/materials/lights/chrome
  bound to state; the ownership-tree loader. Ship the volume/lsystem/terrain demos'
  *scenes* from YAML, not just their panels.
- **P5 — views + RenderView bridge (§9.5):** `views:` over `ViewportManager`, camera
  modes/framing, `tags:`/`show:`/`hide:`. Stage **C first** (overlay-only `RenderView` per
  viewport — proves the wiring with zero ownership-tree change), then stage **B** (the
  ownership→traversal generator; prerequisite: unify `GeometryShape`/`GeometryNode` onto one
  `vtkPolyData`) and the per-view `override:` table. The `view_embed`/`minimap` widget with
  window↔region coupling + the `pick.world`/`pick.node` intrinsics — retiring the `custom`
  minimap.
- **P6 — transfer functions (§10):** the TF `data()`-channel model (control nodes / raw
  table), on-the-fly 1D LUT regen, wire `VolumeNode.handleStateChanged` to re-read its TF;
  reserve the nD `axes`/`primitives` shape. The `tf_editor` widget follows the ColorTable2
  port.
- **P7 — Python/CLI entry** over the C++ loader; a `cvc`/`grl-snam`-style command to
  launch a `.ui.yaml` against a scene. Stage **A** (volumes onto `Shape` subclasses) lands
  as the traversal path matures.

---

## 9. Scene graph in the DSL

Declaring the root VTK scene in the *same* document as the widgets — so one `.ui.yaml`
carries the scene's assets, nodes, lights, views, and the minimap, all bound to the same
state tree the widgets read. This is what makes the minimap a first-class widget instead
of a hand-built C++ escape.

### 9.1 Which scene model

cvcGL has two trees, and — the key move — **the DSL author writes only one, and the
loader compiles it into both.** They are joined on node identity (a stable `id` + `tags`).

- **Ownership tree** — `SceneGraph` + `GraphicsNode` subclasses; the live tree
  VolRover3/pycvc_gl render. **Every node is a `cvc::state_object`**, so a declared node
  *is* a state subtree and every settable prop is already a reactive state key — loading
  a node == writing its subtree; a widget/expression driving a prop == writing one key.
  **This is the only tree the author writes** (`SceneGraph(app, statePrefix="cvcgl")`;
  `addGraphics(name, geometry|volume|<empty>)`, `addLight(name)`; child path
  `<parent>.children.<name>`). It keeps state binding, pose publishing, textures/clip/
  shaders, the Python proxies, and lifetime exactly as they are.
- **Traversal tree** — the OpenInventor-shaped render path
  (`Separator`/`Transform`/`Material`/`DrawStyleNode`/`VisibilityMask`/`Shape`, an ordered
  `TraversalState`, and `RenderView`). This is what makes **per-view masked + restyled
  subsets** work: one shared graph, N `RenderView`s each with a 32-bit mask, and a `Shape`
  that keeps **one `vtkProp` per view**. It is **fully implemented and headless
  unit-tested** — but today it is a *parallel* model wired to nothing (only its own
  `.cpp`s and one test reference it; `SceneGraph`/`Viewport`/`ViewportManager`/pycvc_gl do
  not). So making `RenderView`/`VisibilityMask` **first-class from the start is a bridge
  problem, not a build**: the loader *generates* the traversal graph from the ownership
  tree at view-build time (§9.5.4), and each `Viewport` — which already owns a
  `vtkRenderer` — drives a `RenderView` over it. The author never hand-builds a
  `Separator`/`Shape` graph.

### 9.2 The `scene:` block

Top-level, alongside `menubar`/`windows`/`overlays`/`huds`; rooted at the SceneGraph
`prefix` (default `cvcgl`).

```yaml
scene:
  shadows: { enabled: true, resolution: 1024, interval: 1 }   # -> <prefix>.shadows
  chrome:  { grid: false, axis: false, bboxes: false }        # diagnostic chrome
  lights:
    - light: key
      kind: spot            # directional | spot | fill
      pos: [10, 8, 12]
      target: [0, 0, 0]
      cone: 32
      intensity: 1.0
      # or:  rig: stage     # a StageLighting preset rig (paired with stage_lighting_panel)
  nodes:
    - node: bunny
      type: geometry        # geometry | volume | volren | volslice | group | light
      source: { file: bunny.off }
      material: { color: [0.8,0.8,0.9], ambient: 0.2, diffuse: 0.8 }
      transform: { position: [0,0,0], rotation: [0,0,0], scale: 1 }
      visible: sim.show_bunny            # bind visibility to state (or an expression)
      children: [ ... ]                  # child path = <parent>.children.<name>
```

Prop names mirror the C++: `GraphicsNode` carries name + transform/pose
(`position`/`rotation`/`scale`/`matrix`, all state-bound) + bbox/label/clip/children;
each subclass adds its own props (§9.3). `visible:` — and any prop — may be a bare state
path or a `state_exec` expression, same binding rules as widgets (§3.5, §4).

### 9.3 Node kinds

| `type:` | C++ | Key props |
|---|---|---|
| `geometry` | GeometryNode | `source` (geometry); `texture` (image via UVs); `render_mode` {points,lines,tris,quads,tets,hexs}; `material` {color,opacity,ambient,diffuse,specular,specular_power,point_size,line_width}; `tubes`/`spheres`; `depth_offset`; `shaders:` (→ `custom`) |
| `volume` | VolumeNode | `source` (volume); `transfer_function` {color:[s,r,g,b,…], opacity:[s,a,…]}; shading/ambient/diffuse/sample_distance |
| `volren` | VolRenNode | one-or-many volumes via the cvc::volren raycaster + `render_settings` (CUDA/software) |
| `volslice` | VolSliceNode | one volume as view-aligned composited slices |
| `group` | empty GraphicsNode | a transform/clip parent for `children` |
| `light` | LightNode | parentable, state-bound spot/directional/fill (or use `scene.lights`) |
| — | GridNode / AxisNode | auto chrome, toggled via `scene.chrome` |

### 9.4 Sources

The closed set of `source:` kinds a node draws from:

| `source:` | Backed by | Params |
|---|---|---|
| `{file: path}` | `cvc::read_geometry` / `cvc::volume(app, path)` | path; format inferred |
| `{procedural: {gen, …}}` | lsys recipe / `world_model::generate` / navdemo helpers | `lsystem`, `terrain` (occupancy/heightmap), `ground`, `disc`, `pyramid`, `sdf`/`field` volume + their params |
| `{transfer_function: …}` | control-point table over a volume | color + opacity control points |
| `{texture: {image: path}}` | `setTexture(cvc::image)` | image path, sampled through the node's UVs |
| `{stream: {handler, …}}` | `updateVertices` per frame (AgentGlyphs) | **imperative** — a fixed-topology shape whose vertex buffer a host handler fills each frame |
| `{inline: …}` | in-memory `cvc::geometry`/`volume` | rare in YAML; prefer procedural |

Streamed/dynamic geometry (thousands of agents) stays a declared *shape* + a
host-registered data handler — the one scene piece that isn't purely declarative.

### 9.5 Views render a masked, modified subset of the one authored scene

One render window hosts **N layered viewports**, each its own camera — this is
`ViewportManager` (`SceneRenderer` is its single-viewport facade and exposes
`viewportManager()`; one GL context, mandatory under wasm). But a view is not just a
camera onto identical content — **each view renders its own masked and restyled subset of
the single authored scene**, via `RenderView` + `VisibilityMask`, which the DSL treats as
**core model from the start** (§9.1). The confirmed per-view gate in the traversal code is
exactly:

```
visible = drawStyle.style != Invisible && (accumulated_mask & view.visibilityMask()) != 0
```

The **root view is implicit**; each view sets its camera and its subset/overrides:

```yaml
views:
  - view: main
    camera: { mode: orbit, frame: bounds }       # orbit | fly | track | map
    show: all
  - view: overview
    region: [0.72, 0.0, 1.0, 0.28]               # normalized, VTK y-up; movable every frame
    layer: 1
    background: { color: [0.02,0.03,0.05], opaque: true }   # opaque => clears => solid inset
    input: false                                 # non-interactive inset; clicks fall through
    camera: { mode: map, frame: { top_down: bounds } }      # frameMap(cx,cy,halfH,halfW)
    show: [ sim, annotation ]                    # masked subset (§9.5.2)
    override:                                     # restyled subset (§9.5.3)
      terrain: { drawstyle: wireframe, material: { opacity: 0.25 } }
```

Camera modes map to `CameraController`: `orbit`/`fly` (3-D), `track` (follow a named node
— `setTrackTarget`), `map` (top-down parallel projection, drag pans / wheel zooms —
`frameMap`; the minimap's camera). Framing: `bounds` (`frameBounds`), `top_down: bounds`
(`frameMap` fit-rect), or explicit `eye`/`focal`/`up`. Each view's camera is state-rooted
at `<prefix>.viewers.<name>.camera`, so a widget or action drives it.

#### 9.5.1 Tagging nodes → mask bits

An author never writes raw bits. A node (or subtree) declares symbolic **`tags:`**; the
compiler assigns each distinct tag one bit of the 32-bit mask and wraps the tagged subtree
in a `VisibilityMask` node (which **AND-narrows** the accumulated mask as traversal
descends).

```yaml
scene:
  nodes:
    - node: terrain    # untagged → shown in EVERY view (visibility is opt-OUT)
      type: geometry
      source: { procedural: { terrain: {...} } }
    - node: agents
      type: geometry
      tags: [ sim ]
      source: { stream: { handler: agent_glyphs } }
    - node: rally_labels
      type: geometry
      tags: [ annotation ]
      source: { procedural: { disc: {...} } }
    - node: nav_field
      type: geometry
      tags: [ sim, debug ]   # OR of both bits on ONE VisibilityMask → shown wherever sim OR debug shows
```

Two compiler rules (the load-bearing footguns): **multiple tags on one subtree OR into a
single mask** (`bit(sim)|bit(debug)`); a *nested* second `VisibilityMask` is emitted only
for an explicit intersection ("only where **both** are on"), because nesting two distinct
bits AND-narrows to `0` and the node draws **nowhere**. And **untagged = shown everywhere**
— "main-only" content needs its own tag the overview omits. (The mask is 32-bit → ≤32 tag
channels; the compiler allocates bits and hard-errors on exhaustion, never silently
reuses.)

#### 9.5.2 A view selects its subset — `show:` / `hide:` / `mask:`

A `RenderView.mask` is the **OR of the bits it shows** (plus the always-on untagged bit):
`show: [A, B]` → `bit(A)|bit(B)`; `hide: [A]` → `~bit(A)`; `mask: 0x…` → raw escape hatch.
Keep this **distinct from a global hide**: a global hide compiles to `DrawStyleNode(Invisible)`
/ `Switch(None)` **in the graph** (gone from all views); a per-view hide compiles to the
**mask** (gone from that view only). Both are independent in the gate above.

**Dynamic masks (decision).** `show:`/`hide:` may also be a **`state_exec` expression** that
returns a list of tag names (or a mask int), re-evaluated **every frame** in the read-only lane
(§4.1) — so a view's subset can follow state (a "layers" checkbox, the sim mode, a debug
toggle) with no rebuild. Tag→bit assignment stays **static at load** (the 32-bit budget is
fixed); only the view's *active* mask is recomputed and re-OR'd each frame — a couple of ALU
ops, well within the per-frame cap. Both surfaces work:

```yaml
  - view: overview
    show: (if (= (state-get "ui.layer") "debug") (list "sim" "debug") (list "sim"))
    # nested-YAML form:  show: { if: [ { "=": [ {$: ui.layer}, "debug" ] }, ["sim","debug"], ["sim"] ] }
```

#### 9.5.3 The *modified* subset — per-view material / drawstyle / transform overrides

Masking hides; overriding **restyles the same geometry per view**. A view's `override:` block
is keyed by node id or tag:

```yaml
  - view: overview
    show: [ sim, annotation ]
    override:                    # the "modified subset"
      terrain: { drawstyle: wireframe, material: { opacity: 0.25 } }
      agents:  { mode: merge, material: { color: [1, 0.9, 0.2] }, transform: { scale: 2 } }
      annotation: { material: { color: [0.2, 0.8, 1.0] } }   # override by TAG hits every node carrying it
```

**Lowering — duplicated masked branches (decision).** Rather than depend on a per-view override
value in `Shape` (which the traversal code does **not** have today), an `override:` compiles to
a **duplicated branch**: the loader generates the overridden node into *that view's* traversal
as `Separator{ Material/DrawStyle/Transform(the override) + a Shape over the node's SHARED
`cvc::geometry` }`, and the original is omitted/masked from that view. This is the **most robust
option** — it uses only traversal primitives that already work (`Material`/`DrawStyleNode`/
`Transform` are implemented; the per-view override table is not), so it needs **no engine
change**. The geometry buffer stays shared (stage-B unify, §9.5.4), so the cost is one extra
lightweight prop per overridden node per view. The `override:` DSL surface is stable, so the
future per-`Shape` override-table can replace this lowering with **zero author-facing change**.
*Caveat:* prefer a separate `only_in:` node over overriding a **streamed** node — a duplicated
branch of a streamed node would need its own stream feed.

**Precedence & mode (decision).** When both a **tag** override and a **node-id** override match
one node, **last declared wins**. Each entry carries a **`mode:`** — default **`replace`** (the
override's `material`/`drawstyle`/`transform` replaces that whole element of the accumulated
`StateFrame`) or **`merge`** (only the fields present override; the rest inherit). So
`mode: merge, material: { opacity: 0.25 }` dims a node while keeping its authored color, whereas
the default `replace` swaps the entire material.

#### 9.5.4 Author once, render per-view — the bridge

The two trees have complementary jobs; the DSL compiles **one declaration into both**, keyed
on node identity:

| Tree | Role | Owns |
|---|---|---|
| **Ownership** (`SceneGraph`/`GraphicsNode`) | authoring, state, lifetime | `state_object` per node, pose publishing, textures/clip/shaders, Python proxies, world-bounds |
| **Traversal** (`Separator`/`Shape`/`VisibilityMask`) | rendering | per-view props, order-dependent state, masks + overrides |

The `scene:` block authors into the ownership tree (unchanged — where `state_object`, `data()`,
and pose publishing live). At view-build time the loader **generates a traversal graph per
view** from it: each `GraphicsNode` → `Separator{ Transform(its matrix) + Material/DrawStyle
(its state, with that view's `override:` folded in per §9.5.3) + a Shape over the node's SHARED
`cvc::geometry` }` wrapped in `VisibilityMask(OR of its tag bits)`. Each `Viewport` **owns a
`RenderView`** over its generated root and calls `renderView.render(root)` per frame instead of
`SceneGraph::setRenderer`; `ViewportManager` composites the N renderers unchanged. Per-view
generation is what lets overrides lower to inline `Material`/`DrawStyle` (no unimplemented
per-`Shape` override table); the shared `cvc::geometry` keeps geometry single-copy, and dynamic
`VisibilityMask` + `RenderView.mask` handle the *dynamic* subset (§9.5.2). None of this edits
the authored ownership graph.

**Incremental regen (decision).** The generator does **not** rebuild on every state change. A
node's edit (pose / material / geometry) **dirty-propagates** to its generated
`Transform`/`Material`/`Shape` in each view's traversal — the same targeted-update contract the
pose-publish path already uses — so a slider drag re-emits one `Transform`, not a graph rebuild.
A structural change (add/remove node, retag) rebuilds only the affected subtree.

**Staged adoption** (so it ships without the full unify): **(C, minimal now)** keep the main
scene on `SceneGraph::setRenderer` and give only overlay/annotation layers their own
`RenderView` per viewport — already delivers "annotations in the overview only" and proves the
`Viewport`-owns-`RenderView` wiring with zero ownership-tree change; **(B, target)** the
generate-from-ownership adapter above, whose one prerequisite is unifying
`GeometryShape.setGeometry` with `GeometryNode.updatePolyData` onto **one** `vtkPolyData` (today
they build duplicate polydata); **(A, later)** retire the one-renderer binding and port
`VolumeNode`/`VolRenNode` onto `Shape` subclasses. **The DSL surface — `tags`, `show`/`hide`,
`override` — is identical across all three stages**, so authors are insulated from which stage
the runtime is at.

### 9.6 The `minimap` widget = a view embedded in a window

The minimap is just a **`view_embed` of a `RenderView`** (§9.5) whose masked/restyled subset
and top-down camera are declared like any other view. The widget binds that view to an ImGui
window whose **content rect drives the view's `region` every frame** (`ViewportManager` already
supports a movable-region viewport; `addMirrorViewport` is the degenerate `show: all` + no
`override:` case):

```yaml
windows:
  - window: Minimap
    id: minimap
    placement: { corner: bottom_right }
    transparent: true                # WindowBg alpha 0; the view shows through
    children:
      - view_embed: overview         # references the view from §9.5
        # loader computes region = window content rect (normalized, y-up), calls
        # view.setRegion(...) before render (one-frame lag invisible), layers above main,
        # opaque background => solid inset.
        markers:                     # PiP-only overlay nodes, attached to THIS view only
          - node: rally_dots
            type: geometry
            source: { stream: { handler: rally_glyphs } }
            only_in: overview
        on_click:                    # click-to-follow, as an action — not custom
          do:
            - { camera.chase: [ { pick.node: [ overview, {$int: event.x}, {$int: event.y} ] } ] }
        on_drag:                     # drag a target dot -> retarget (event.* payload, §4.6)
          do:
            - { set: [ sim.target, { pick.world: [ overview, {$int: event.x}, {$int: event.y} ] } ] }
```

The **window↔region coupling, layering, opaque-clear inset, and non-interactive
fall-through** are exactly the mechanics the hand-rolled nav minimap implements — now
declared. The two genuinely imperative pieces become host query intrinsics:
`pick.world(view, px, py)` (unproject through the view's ortho camera → world x,y — the
`pip_world_at` inverse) and `pick.node(view, px, py)` (nearest-node hit-test). With
those, click-to-follow and target-drag are ordinary `on:` actions that write state.

### 9.7 What this retires

The nav demos' `custom: minimap` node **collapses to a declarative `view_embed` over a
masked/restyled `RenderView` + two `pick.*` intrinsics**. The only remaining non-declarative
scene pieces are (a) streamed per-frame geometry (`source: stream`) and (b) fully custom GLSL
(`shaders:` → `custom`) — both narrow, well-fenced host hooks, not whole hand-built UIs.
**With §9, all 10 demos are describable**, and the nav trio no longer needs a bespoke
C++ minimap.

### 9.8 Open questions (scene-specific)

**Resolved:**
- **`pick.world`/`pick.node`** → ✅ **first-class query intrinsics** (§4.4). Requires
  generalizing the nav demo's `pip_world_at` unproject to any view camera.
- **Scene model** → ✅ commit to the **ownership tree** as the sole authoring surface and
  **generate the traversal graph from it**, so `RenderView`/`VisibilityMask` are first-class
  from the start via the bridge (§9.5.4), adopted in stages C→B→A with an identical DSL
  surface throughout.
- **Transfer functions** → ✅ moved to its own section — see **§10** (control nodes / raw
  table / nD, on the `data()` channel, with a ColorTable2-derived editor widget).
- **Override lowering** → ✅ **duplicated masked branches** (§9.5.3) — the most robust option,
  works on existing traversal primitives with no engine change; the per-`Shape` override table
  is a later drop-in optimization behind the same `override:` surface.
- **Override precedence & mode** → ✅ **last declared wins**; each entry has `mode: replace`
  (default) or `mode: merge` (§9.5.3).
- **Dynamic masks** → ✅ `show:`/`hide:` accept a `state_exec` expression re-evaluated per frame;
  tag→bit assignment stays static (§9.5.2).
- **Incremental regen** → ✅ **dirty-propagate** node edits into the generated traversal (same
  contract as the pose-publish path); structural changes rebuild only the affected subtree
  (§9.5.4).

**Still open:**
1. **Streamed-source contract** — buffer ownership, per-frame push vs pull, which thread;
   ties to the sim tick. *(User revisiting.)*

---

## 10. Transfer functions

A transfer function is **structured, typed data**, not a stringified scalar — so it rides the
`state_object` **`data()` typed channel** (Q1, §4.3: `state-data-get`/`state-data-set`,
`data_object` value), the same channel a color or vector tunable uses. That gives the TF one
place to live that round-trips as a real typed blob, drives the renderer reactively, and is
what a TF editor widget binds to like any other widget. One TF model already serves every
volume renderer in libcvc: the canonical `cvc::volren::transfer_function` (a sorted vector of
`{value, r,g,b,a}` points, `sample()` piecewise-linear) is shared verbatim by `volren` and
`volslice`, and `VolumeNode` consumes the same two flat arrays into VTK's own
`vtkColorTransferFunction`/`vtkPiecewiseFunction`. Each renderer **bakes** the TF at its own
resolution (`volren` 1024, `volslice` 256, VTK interpolates its points).

### 10.1 Two authoring forms: control **nodes** OR a raw **table**

A node's `transfer_function:` is **either** a set of control nodes the loader interpolates into
a table, **or** a raw baked table supplied directly — both lower to the same per-consumer LUT.

- **Control nodes** (primary, editor-native — the ColorTable2 shape). Two independent,
  position-sorted lists, because color and opacity control points need not coincide:
  **color nodes** (`scalar → rgb`) and **alpha nodes** (`scalar → a`), plus optional
  **isocontour nodes** (`scalar → id`) — which do **not** feed the LUT; they drive live
  isosurface extraction, so they are a separate attachment kept out of the table path.
- **Raw table** — an explicit `table:` of N RGBA entries (e.g. 256) + a domain; the baked form.

**Regeneration contract:** the nodes are the source of truth, the table is a derived cache. On
**any** node change the loader marks the LUT dirty and **regenerates it on the fly** —
piecewise-linear, clamped to [0,1], exact first/last entries at the ends (ColorTable2's proven
boundary rule). 1D regenerates the whole LUT cheaply; higher-D rebakes only the dirty
primitive's bounding box. Reactive edge: `data()` write → dirty → rebake → re-render.

```yaml
# 1D TF, control-node form (the ColorTable2 lift)
nodes:
  - node: skull
    type: volren
    transfer_function:
      dims: 1                              # one axis; see §10.2
      domain: auto                         # follow the volume's [min,max]; or {min, max}
      resolution: 256                      # baked LUT size — a DEFAULT, not hardcoded (the legacy 256 was a bug)
      opacity_cubed: false                 # ColorTable2's v³ low-opacity trick, optional
      color:                               # scalar → rgb (endpoints auto-inserted at 0 and 1)
        - { at: 0.0, rgb: [0,0,0] }
        - { at: 0.5, rgb: [1,0,0] }
        - { at: 1.0, rgb: [0,1,0] }
      alpha:                               # scalar → a, independently placed
        - { at: 0.0, a: 0.0 }
        - { at: 0.25, a: 0.75 }
        - { at: 1.0, a: 1.0 }
      isocontours:                         # optional; drives isosurface extraction, not the LUT
        - { at: 0.6, id: bone }

# 1D TF, raw-table form — a directly supplied 256-entry LUT
transfer_function:
  dims: 1
  domain: { min: 0, max: 255 }
  table: { size: 256, rgba: [ 0,0,0,0,  1,0,0,0.2,  … ] }   # N × RGBA, resampled per consumer
```

Back-compat: the 1D control-node form lowers to the existing split state keys
(`…transfer_function.color` = CSV `value,r,g,b`; `…transfer_function.opacity` = CSV `value,a`;
`merge_ramps()` fuses them), so nothing reading those today breaks; the structured/nD payload
rides `data()` alongside. The legacy `gradient_ramp` (a 1D `|gradient|`→alpha multiplier) and
the separate `isosurfaces` list are declarable as sibling fields.

> **Caveat (tracked):** `volren`/`volslice` are two-way state-bound (state write → re-read →
> re-bake → re-render), but **`VolumeNode`'s TF is one-way today** — `handleStateChanged` does
> not re-read the TF, so a TF editor bound to a VTK `VolumeNode` won't drive it until
> `VolumeNode.handleStateChanged` is extended to consume the TF.

### 10.2 Dimensionality is a runtime property — `dims:` / `axes:` with room for nD

Dimensionality is **one field on one TF type**, not N types — adopting the unified model from
the **volrover3 modernization roadmap §11–§12** (`cvc-engagement-docs-status/modernization/
2026-08-11-volrover3-roadmap.md`; the `CVC-modernization*.md` trio only records the parity gap
and the `cvcQt` consolidation). A TF is a function of an ordered **tuple of axes**; `dims:` (or
the length of `axes:`) *is* the dimensionality — 1D today, 2D/3D supported, shape open for
nD/4D (a `Time` axis + keyframes).

```yaml
axes:
  - { kind: value,        field: density, domain: auto }                 # axis 0 = today's only axis
  - { kind: gradient_mag, field: "",      domain: { min: 0, max: 1 } }   # axis 1
  # kind ∈ { value, gradient_mag, gradient2, time, custom }; each axis its own domain
```

For 2D+ the point lists give way to **primitives** (brushes in the axis plane —
`tent`/`rectangle`/`gaussian`/`bezier`/`freeform`), the generalization of control nodes:

```yaml
# 2D TF sketch (Kniss value × gradient-magnitude), same node type, +1 axis
transfer_function:
  dims: 2
  axes:
    - { kind: value,        field: density, domain: auto }
    - { kind: gradient_mag, field: "",      domain: { min: 0, max: 1 } }
  resolution: [256, 256]                    # per-axis (128 typical for 3D/4D)
  primitives:
    - { kind: gaussian,  coords: [0.40, 0.20, 0.05, 0.08], rgba: [1, 0.8, 0, 0.8] }
    - { kind: rectangle, coords: [0.70, 0.90, 0.60, 1.00], rgba: [0.2, 0.4, 1, 0.4] }
```

The baked form generalizes cleanly (`baked_transfer_function` is already "flat array + domain";
an nD bake is "flat array of ∏(sizes) + per-axis [lo,hi] + lookup"). **Reality check:** the
*renderers* are 1D today (the only multi-axis precursor is `gradient_ramp`, a separable alpha
multiplier). So **2D/3D declarations will be authorable before the raycaster can consume them**
— the DSL intentionally leads the renderer here; 1D is fully wired, nD reserves the shape.

### 10.3 The editor widget is a first-class DSL widget (ColorTable2, coming)

The forthcoming editor (the **ColorTable2** Qt widget from VolumeRover2, not yet ported) is a
DSL **widget node** that **binds to a TF's `data()` channel** — the widget is a *view*, the TF
data is the *model*, edits flow model → LUT → renderer as an ordinary reactive edge (ColorTable2's
`changed()` becomes a DSL edge, like a slider's two-way bind). ColorTable2's model is exactly
the node model above (`color_node`/`opacity_node`/`isocontour_node` sets, normalized [0,1],
`getTable(size)` → baked RGBA). One widget family covers all dimensionalities by inspecting
`axes` (1D = the ColorTable2 port `cvcQt::ColorTable`; 2D/3D/4D = `cvcQt::TransferFunctionEditor`
— the two existing widgets reconciled as *views over one model*, per the roadmap).

```yaml
windows:
  - window: Transfer Function
    children:
      - tf_editor: skull_tf
        bind: cvcgl.nodes.skull.transfer_function   # the node's TF data() channel
        interactive_updates: true                   # emit-on-drag vs on-release (ColorTable2's flag)
        layers: [ histogram, contour_spectrum ]     # overlays supplied FROM OUTSIDE as generic
                                                     # 1D-function / 2D-geometry layers (no VolMagick
                                                     # knowledge baked in — per ColorTable2's own TODO)
```

Because the TF lives on `data()`, the same editor drives `volren`, `volslice`, and (once
`VolumeNode` re-reads its TF) the VTK path, and round-trips into the scene file. Persistence
targets the roadmap's JSON form (`axes` + `primitives` + optional keyframes + optional baked
LUT); a **load-only importer** (`load_vinay`) reads legacy ColorTable2 `.vinay` text presets.
