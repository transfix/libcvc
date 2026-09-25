# cvcGL UI DSL — Scoping Spec (v0.9, for iteration)

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
> **v0.9** resolves the **last open item** — the **streamed-source contract** (§9.9): build-once +
> pull-latest-snapshot + `updateVertices` on the render thread, a lock-free atomic-`shared_ptr` handoff
> that coalesces to latest (never a queue), fixed topology or async rebuild, and unification with
> `state://…?data` live sources — with the hard rule that a Python producer never runs on the render
> thread. **Every open question in the spec is now resolved.**
>
> **v0.8** adds a **uniform URI resource model** (§13) behind `load:`/`include:`/`source:` — schemes
> `file`/`http(s)`/`state`/custom via a pluggable registry, with `state://…?value|?data|?children` (live
> or snapshot) letting any of them point at a state node; a **pycvc API** (§14) to register Python
> callables as `state_exec` intrinsics (~already shipped as `Exec.register_fn`) and as URI handlers, with
> the GIL/thread/lifetime contract. It also fleshes out the layout engine (§3.0.3a), the `ui.docs`
> subtree lifecycle (§11.4), and the per-pid observability node (§7.8.6a). Two concrete blockers flagged:
> no temp-file helper and no `bytes` marshaling branch (both needed for binary URI handlers).
>
> **v0.7** unifies the widget model (§3.0 — *everything is a widget, a window is a widget with
> `frame:` chrome*, Qt `layout:`/`size:` semantics for a clean cvcQt port), pins down **where a
> loaded UI's data lives** in the state tree (§11, `ui.docs.<doc>.*`), **how handlers are scoped**
> (§7.8 — chroot is enforced by the `intrinsics_context`, `.`-separators, link-node cross-reach,
> no local uid ACL), and adds **`load:` sub-UI/sub-scene modularization** (§12). It also **fixes a
> path bug** (scene nodes are `<prefix>.graphics.root.children.<name>`, not `cvcgl.nodes.<name>`)
> and corrects the §4.8 `/`-escape to a loader construct.
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

**Window archetypes (7)** — as of v0.7 these are **not** a privileged vocabulary; they are all
just **widget `type:` values + `frame:` variants** under the one widget model (§3.0):
`main_menu_bar` (`type: menubar`); `control_panel` (a `group` + title/collapse/drag/resize
chrome, seeded top-left); `library_own_window_panel` (a composite + close-box);
`dense_collapsing_panel` (sectioned, height-capped); `corner_overlay` (`type: overlay`,
`frame.placement: always`, no background); `minimap_pip` (`type: view_embed`, §9.6);
`vtk_text_hud` (`type: hud` — VTK actors, not ImGui).

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

### 3.0 One widget model: everything is a widget; a window is a widget with chrome

**The Qt move (v0.7):** *everything is a widget, widgets embed widgets.* The earlier drafts
had four sibling buckets (`menubar`/`windows`/`overlays`/`huds`) and a separate "window
archetypes" vocabulary (§2). We collapse that to **one node type**. A "window" is not a kind
— it is any widget that carries `frame:` chrome and sits at a document root. This makes the
cvcQt ports (`ColorTable2`→`cvcQt::ColorTable`, the `.ui`-composed dialogs) drop in 1:1,
because a Qt `QWidget` tree *is* this tree.

**The base widget node** (one shape for windows and embedded children):

```yaml
- widget: transfer_editor        # `window:` is sugar: widget + a default frame: block
  type: group                    # container kind OR a leaf type (slider_int, combo, …)
  title: "Transfer Function"     # groupbox/window caption
  frame:                         # PRESENCE of this block ⇒ this widget is a floating window
    chrome: [title, move, resize, collapse, dock, close]   # omitted decorations → ImGui No* flags
    placement: first_use_ever    # first_use_ever | always | system  (§3.3)
    pos: [10, 30]
  layout:                        # how THIS widget arranges its children (§3.0.2)
    kind: vertical               # vertical | horizontal | grid | form | stack | free
    spacing: 6                   # Qt layout spacing (px)
    margins: [11, 11, 11, 11]    # Qt contentsMargins L,T,R,B
  size:                          # this widget's sizing within its PARENT's layout (§3.0.3)
    hint: [300, 0]               # QWidget::sizeHint; 0 = auto that axis
    min:  [200, 0]
    max:  [0, 0]                 # 0 = unbounded (QWIDGETSIZE_MAX)
    policy: [expanding, fixed]   # [horizontal, vertical] — Qt names 1:1
    stretch: 1                   # this item's stretch factor in the parent box/grid cell
  bind: ui.controls.open         # optional two-way state (open/checked/value per type)
  on: ...                        # actions (§4); on_click/on_drag/… (§4.6)
  visible_when: ...              # still an attribute, not a child (§3.5)
  children: [ ... ]              # ordered; each child is another widget node (recursion)
```

`window: X` is pure sugar for `widget: X` + a default `frame:`. There is exactly one schema;
the loader never branches on "is this a window."

**`type:` absorbs the old archetypes and buckets** — nothing is privileged:

| Old thing | Now |
|---|---|
| `main_menu_bar`, a `menu` | `type: menubar` / `type: menu` (containers laid out horizontally / as a popup) |
| `control_panel` / own-window panel / dense panel | `type: group` (or `panel`) + `frame:` chrome variants |
| `corner_overlay` | `type: overlay` (a widget with `frame.placement: always` + no background) |
| `minimap_pip` / `view_embed` (§9.6) | `type: view_embed` |
| `vtk_text_hud` (Fps/ScreenText) | `type: hud` (VTK actors, not ImGui — §3.0.4 caveat) |
| `tf_editor` (§10), `panel: scene`, `panel: stage_lighting` | `type: tf_editor` / `scene_panel` / … (composites) |
| the 21 leaf widgets | unchanged `type:` values (`slider_int`, `combo`, `button`, …) |

`include`/`repeat`/`custom` (§3.7–3.8) are **unchanged and purely additive**: an `include`
expands to a subtree of these nodes; `repeat` emits N sibling widgets per frame; a `custom`
node is a widget whose paint/`measure:` body is a host/`state_exec` closure (this is how a
hand-coded Qt widget like `ColorTable` — a `QFrame` overriding `sizeHint()` — ports).

#### 3.0.2 `layout:` — Qt layout classes, Qt semantics

A container owns exactly one layout that arranges its ordered `children:`. Kinds map 1:1 to
the classes the volrover `.ui` corpus actually uses (QGridLayout 248, QHBoxLayout 86,
QFormLayout 54, QVBoxLayout 38, QTabWidget 34):

| `layout.kind` | Qt class | ImGui realization |
|---|---|---|
| `vertical` | QVBoxLayout | default cursor flow, one child/line, inside `BeginChild` (clip/scroll/margins) |
| `horizontal` | QHBoxLayout | `SameLine()` between children, **or** a 1-row `BeginTable` when any child is `expanding` |
| `grid` | QGridLayout | `BeginTable(cols)`; children carry `at: [row,col]` + `span: [r,c]`; `col_stretch:` → column `WidthStretch` weights |
| `form` | QFormLayout | 2-col `BeginTable`; children are `{label:, field:}`; col0 `WidthFixed`, col1 `WidthStretch` |
| `stack` | QStackedLayout/Widget | build **only** the active child (`active:` bind); `type: tabs` → `BeginTabBar` |
| `free` | absolute | `SetCursorPos(child.pos)` per child; sizes from each child's `size.hint` |

A pure spacer is `- spacer: { orient: horizontal|vertical, policy: expanding }` (Qt's
`QSpacerItem` — the corpus's dominant "push the button row to the edge" idiom; `expanding` →
a `WidthStretch` column or a `GetContentRegionAvail`+`SetCursorPos` spring, `fixed` →
`Dummy`). `spacing:` → `PushStyleVar(ItemSpacing)`; `margins:` → `BeginChild` +
`PushStyleVar(WindowPadding)`.

#### 3.0.3 `size:` — Qt size policy, per axis

`size.policy: [<h>, <v>]` uses Qt names verbatim so a `.ui` `<sizepolicy>` copies straight
across: `fixed` (only `hint`), `preferred` (default; may shrink/grow), `expanding` (grabs
slack → `SetNextItemWidth(-FLT_MIN)` / a stretch column), `minimum` (hint is a floor),
`maximum` (hint is a ceiling), `minexpanding`, `ignored`. Sugar: `auto`==`preferred`,
`fill`==`expanding`. `hint`/`min`/`max` are Qt's `sizeHint`/`minimumSize`/`maximumSize`; a
`custom` widget supplies its own via a `measure:` handler `(lambda () (list w h))` — the
`ColorTable2` case.

**Two-pass sizing** (ImGui is single-pass immediate, so a plain linear cursor can't
distribute `expanding` slack among not-yet-drawn siblings): **(A, default) table-backed** —
express every multi-child layout as `BeginTable`, letting ImGui's table engine do its own
two-pass column measure (Qt-like stretch/fixed distribution for free, at the cost of exact
first-frame pixels); **(B) explicit two-pass** — a measure pass computes each node's
`sizeHint` bottom-up, then an arrange pass assigns rects — only for a `custom` widget that
must reproduce a Qt `sizeHint()` exactly. §3.0.3a is the concrete rule.

#### 3.0.3a Layout engine — the concrete rule

**(A) Table-backed (default for `horizontal`/`grid`/`form`/`stack` + spacers).** `ImGui::TableUpdateLayout`
already runs an internal two-pass over columns — pass 1 sums each column's *last-frame* auto-width +
stretch weight; pass 2 gives `WidthFixed` columns their measured width and splits the rest among
`WidthStretch` columns by weight. **That is Qt's fixed/stretch distribution for free, with no measure
code of ours.** Qt maps directly onto column weights:

| Qt | Table |
|---|---|
| `QHBox` with an `expanding` child | 1-row table; each child a column, `expanding`→`WidthStretch`, else `WidthFixed` |
| `QGrid` | `BeginTable(cols)`, `col_stretch:`→ per-column `WidthStretch` weight |
| `QForm` | 2-col table: col0 `WidthFixed` (labels), col1 `WidthStretch` (fields) |
| `QSpacerItem policy:expanding` | an empty `WidthStretch` column (pushes the row to the edge) |
| `stretch: N` on a child | that column's `StretchWeight = N` |

`QVBox` is **not** a table — plain cursor flow inside `BeginChild`; vertical slack is a trailing
stretch **spacer row** (a `Dummy` spring). The one honest cost of (A): a stretch column's width this
frame was measured *last* frame, so the **first frame (and the frame after a resize)** can be one
frame stale — a visible pop only when a layout appears cold; a sub-16ms transient otherwise.

**(B) Explicit two-pass** — measure `sizeHint` bottom-up, arrange top-down — used **only** when a node
is `custom` with a `measure:`/overridden `sizeHint` (the `ColorTable2` port, which computes its hint
eagerly and so *can* be arranged eagerly), or when `layout.exact: true` (a screenshot-golden / print /
one-shot-modal path with no second frame to settle).

**Nesting** is native: a container child emits its own `BeginTable` inside the parent's cell and
measures against `GetContentRegionAvail` there — the one rule the loader enforces is that a column
hosting a **stretch** child must be `WidthStretch`/`WidthFixed`, never `WidthAuto` (a stretch child in
an auto column has no finite region and collapses to content width). A mode-B `custom` widget composes
fine inside a mode-A table cell (the cell is a hard region boundary). `size.hint` rides the widget's
state node (`ui.docs.<doc>.tree.<id>`, §11.2), propagated one frame behind for (A).

#### 3.0.4 Root children unify — placement, not buckets

The four root lists collapse to one ordered `root:` of widgets; what used to pick a bucket is
now the widget's `type:` (`menubar`/`overlay`/`hud`) and `frame.placement:`. Two honest
caveats carry over: **HUD widgets are VTK actors** (`type: hud` → the `<prefix>.viewers.<v>.hud`
subtree and the VTK HUD API, *not* the ImGui walk); and `include`/`repeat`/`custom` are
unchanged. The VTK canvas stays the implicit root host (§1) — these draw *on top of* the scene.

The subsections below (3.1 document/root, 3.2 menus, 3.3 windows, 3.5 widgets, 3.6 composites)
are the **specifics of particular widget types** under this one model.

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
subtree**: a unit's expressions and actions see `some.tunable` resolved *relative to that
panel's subtree*, so a reusable unit can't accidentally read or clobber a sibling's
state, and two includes of the same unit are naturally isolated. **Chroots nest** — the
loader chroots an included/loaded unit's handlers to a deeper path (§7.8.7). The full
mechanics — that chroot is enforced by the per-program `intrinsics_context`, not the
scheduler — are in **§7.8**.

Because a chroot hides everything outside the panel's subtree, cross-panel reach is
**explicit**, two ways: (1) the enumerated `window.*`/`scene.*` intrinsics operate on
document-global targets by design; (2) a **leading `/`** on a bind (`/cvcgl…`) reaches
app-root-absolute state. **Correction (grounded):** `/` is **not** a state-path feature —
`state_exec`'s separator is `.` and the intrinsic layer strips leading separators, so a
`/`-path handed to `state-get` still resolves *inside* the chroot (`CannotAccessOutsideChroot`).
The loader therefore realizes a `/`-bind through a **second, un-chrooted resolver** (a `ctx`
at the app root) or a **transparent link node** (`state::linkTo`, the sanctioned cross-subtree
mechanism) — never by passing the string through the chrooted intrinsics (§7.8.2–7.8.3). So
the default is sandboxed and local; reaching outside is possible but must be written
explicitly, never by accident. (Chroot granularity is settled as **per-panel, nestable**.)

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

### 7.8 How a handler is scoped — chroot root, tree access, uid/gid, where it lives

Grounded in the real `state_exec` code, with one **correction** to the §4.8 model.

**7.8.1 The scoping seam is the `intrinsics_context`, not the scheduler.** `scheduler::execute()`
copies `execute_options.root_path` into `proc.root_path`, but that is a **passive record** (fork
inheritance + migration only); the scheduler **never calls `apply_chroot`**, and its single shared
`evaluator_` has no per-process root. **All chroot lives in a per-program `intrinsics_context`.**
Per handler the loader must: build a fresh `ctx`; `apply_chroot(ctx, tree_root, root_path)`
(re-bases `ctx.root` onto the subtree node, *creating* it); `register_intrinsics(env, &ctx)`
(binds every intrinsic as a closure capturing `&ctx`, which must outlive the eval); pass `env` as
`execute_options.env`. (This is what the chroot integration **test** does; `pycvc_exec.cpp` reuses
one full-tree ctx — a UI must follow the test, one ctx+env per handler.)

**7.8.2 Default access = its own `ui` subtree; no absolute escape at the intrinsic layer.** Wire
each unit's handlers with `root_path = "ui.docs.<doc>.panels.<panelId>"` (or `ui.docs.<doc>`). The
handler then gets the full state-op surface **chroot-gated to its own subtree**
(`state-get/-set/-exists/-children/-delete`, `state-data-get/-set`, `state-watch/-unwatch`, expiry,
messaging) — every path-taking op resolves through `ctx.root`. **There is no `/` escape at the
intrinsic layer:** `SEPARATOR` is `.`, leading separators are stripped, so a `/`-path still
resolves *inside* the chroot (`CannotAccessOutsideChroot`). §4.8's `/` is therefore a **loader
construct** (a second un-chrooted resolver, or a link node), not a state-path feature.

**7.8.3 Cross-subtree reach = link nodes.** `state::linkTo(target)` marks a node as a reference
resolved against the **app root**; `transparent` + `setLinkWritable(true)` route reads/writes
through. Place named holes inside the unit subtree (`ui.docs.d.panels.p.shared → app.shared`) so a
handler touches shared state through **auditable links** while everything else stays sandboxed.

**7.8.4 uid/gid gate nothing locally.** They ride on the process and copy to children, but the
local scheduler enforces **no path ACL** (guide §10 "Local Sovereignty"): the chroot is the
addressing boundary. uid/gid are identity for auditing + the cluster-consensus write boundary +
leader-gated cross-node admin; `resource_policy` (§7.7) bounds cost, not paths. **The UI sandbox is
chroot + reduced env + limits, not uid.**

**7.8.5 Two shared-state footguns.** *Messaging is not chroot-scoped* — `deliver_to_receivers` keys
on the raw path string, so §7.1 event channels must be namespaced per unit
(`ui.docs.<doc>.ev.<nodeId>.<kind>`) or a bare `"clicked"` crosses units. *`state-watch` has one
shared watch root* — mixing handlers of different chroots on one scheduler mis-resolves watched
paths (last-writer-wins); give each chroot its own scheduler, or pin one fixed watch root per doc.

**7.8.6 Where running handlers live — and the observability gap.** Handler bodies are `value_t`
ASTs held only in the scheduler's in-memory `processes_` map — **not in the state tree.** The
tree's only `state_exec` footprint is scheduler *config* (`state_exec.defaults.<key>`,
`state_exec.schedulers.<id>.<key>`). Status is API-only (`get_process_info`/`ps`), and **`exit_error`
is dropped** (`process_info` has no such field) — a UI sees `status==killed` but not why.
**Recommendation (badges §7.5) — a per-pid observability node.** Publish, **outside any unit chroot**,
one node per live process (all keys are `value()` strings — the channel `json()`/`save()`/replication
carry, *not* `data()` which wouldn't survive a viewer hop):

```
state_exec.schedulers.<id>.processes.<pid>.status           spawning|running|blocked|killed|terminated
state_exec.schedulers.<id>.processes.<pid>.exit_error       ""|max_steps_exceeded|time_limit_exceeded|error:<what()>
state_exec.schedulers.<id>.processes.<pid>.step_count       cumulative micro-steps
state_exec.schedulers.<id>.processes.<pid>.{uid, node, kind, last_activation}
```

`node` = the owning `ui.docs.<doc>.tree.<id>` (the loader knows it when it wires the handler); `kind` =
`on_tick|on_key|on_click|action|load-preflight`. **Four write points, nowhere else:** *spawn* (create,
`status=running`, stamp identity); *step* (bump `step_count`/`last_activation` **once per activation
slice**, not per micro-step — badge liveness, not instruction granularity); *kill* (the §7.5 wrap sets
`status=killed` **and** `exit_error=reason` in the same critical section — this is the write that closes
the dropped-`exit_error` gap, durable the instant the kill happens); *terminate* (`status=terminated`,
then `state-expire`). Lifecycle bounded by the process: expired at terminate, a killed node lingers a
few seconds so a once-per-error badge can read it, residents persist for the doc.

**Badge binding.** The node is under `state_exec.*`, not `ui.docs.*`, so a chrooted panel can't name it
(§7.8.2). The loader places a **read-only transparent link** inside the panel chroot
(`ui.docs.<doc>.panels.<p>.badge_src → state_exec…processes.<pid>`, `setLinkWritable(false)`); the badge
then `bind:`/`state-watch`es `badge_src.status`/`.exit_error` **like any widget** (§11.3) — live
re-resolve, so a kill flips the badge the next frame, and the panel can't scribble back. This is the
same **live** (link) vs **snapshot** (decode-once) distinction as `state://` sources (§13); a badge
wants live. *(The `STATE_EXEC_PORTING_PLAN` already stores handler closures as state subtrees —
`__signals__.handlers`, `__watches__` — so this is the sanctioned direction; it needs no new API beyond
writes the scheduler is positioned to make.)*

**7.8.7 One env, no runtime re-chroot.** `fork` inherits `root_path`/`uid`/`gid` + the parent's
`global_env`. There is **no runtime nested-chroot API** — a deeper scope is a fresh `apply_chroot`
to a deeper path *at wire time*. §4.8's "chroots nest" means the loader chroots an included/loaded
unit's handlers to the deeper `ui.docs.<doc>.includes.<id>` path (§12), not in-evaluator nesting.

---

## 8. If we build it — suggested phasing

- **P0 — declarative core + the unified widget model + state binding:** the one widget node
  (§3.0) with `frame:`/`layout:`/`size:`; the **table-backed layout engine** (Qt VBox/HBox/
  grid/form/stack → `BeginTable`/`SameLine`/`BeginChild`, size policies, spacers); leaf widgets
  with `bind: <path>`; modifiers; the `ui.docs.<doc>` state-tree layout (§11) and the three bind
  cases (relative / `/`-absolute-via-loader / scene-qualified). Promote the target demos' tunables
  to `cvc::state`. Ship `bunny_shadow`, `terrain_lab`, `lsystem_*`, `volren/volslice` from YAML.
  C++ loader in libcvc; `enabled()==false` no-op path.
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
- **P3 — composition + modularization + escape hatches:** `units`/`include` (args, null-drop,
  PushID, recursion guard), `repeat`; the per-handler chroot wiring (one `intrinsics_context` +
  `apply_chroot` + `register_intrinsics` per handler, §7.8) and namespaced `ev.*` channels; the
  **`uri_resolver` + scheme-handler registry** (§13: `file://`/`state://`, the `image_file_io`-style
  registry, the temp-file bridge, the per-pid observability node §7.8.6a); **`load:` URI-based sub-UI /
  sub-scene fragments** (§12, mount prefix + own chroot + cycle guard + hot-reload); `custom:` host
  nodes. Ship swarm/drive-from-one-unit.
- **P3b — pycvc + host handlers (§14):** expose `Exec.register_intrinsic` (already ~present) +
  `register_uri_handler`; factor the shared PyObject wrapper out; add the `py_to_value` **`bytes`
  branch** and a **temp-file helper** (both blockers for binary URI handlers); a Python `requests`
  `https` handler as the reference web-fetcher.
- **P4 — scene graph (§9):** the `scene:` block — nodes/sources/materials/lights/chrome
  bound to state; the ownership-tree loader; the **streamed-source contract** (§9.9: the
  build-once + atomic-`shared_ptr` latest-snapshot + `updateVertices`-on-render-thread pattern,
  `{stream:}` and the `state://…?data` live path, degenerate-collapse LOD, the wasm inline
  fallback). Ship the volume/lsystem/terrain *scenes* — and the nav *agent stream* — from YAML.
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
  `addGraphics(name, geometry|volume|<empty>)`, `addLight(name)`). The scene's node root is
  **`<prefix>.graphics.root`** (a `NullGraphicNode "root"`), and a node's real path is
  **`<prefix>.graphics.root.children.<name>`** (nested `…children.<name>.children.<child>`) —
  see §11 for the full state layout. It keeps state binding, pose publishing, textures/clip/
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
      children: [ ... ]                  # real path <prefix>.graphics.root.children.<name> (see §11)
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

A `source:` is either a **URI** (`file://` / `http(s)://` / `state://` / custom — resolved through §13,
so a mesh can come from disk, the web, or a live scene node) or one of the structured generators:

| `source:` | Backed by | Params |
|---|---|---|
| a **URI** (`source: file://…` / `state://…?data` / `pkg://…`) | §13 resolver → the existing reader (path/temp-file) or a bound `data_object` | any registered scheme; `state://…?data` binds **live** to a node |
| `{procedural: {gen, …}}` | lsys recipe / `world_model::generate` / navdemo helpers | `lsystem`, `terrain` (occupancy/heightmap), `ground`, `disc`, `pyramid`, `sdf`/`field` volume + their params |
| `{transfer_function: …}` | control-point table over a volume | color + opacity control points (§10) |
| `{texture: {image: <uri>}}` | `setTexture(cvc::image)` | image URI, sampled through the node's UVs |
| `{stream: {handler, …}}` | `updateVertices` per frame (AgentGlyphs) | **imperative** — a fixed-topology shape whose vertex buffer a host handler fills each frame |
| `{inline: …}` | in-memory `cvc::geometry`/`volume` | rare in YAML; prefer procedural |

Streamed/dynamic geometry (thousands of agents) stays a declared *shape* + a host-registered data handler
(registered through the same §13/§14 seam) — the one scene piece that isn't purely declarative, and the
subject of the still-open streamed-source contract (§9.8).

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

- **Streamed-source contract** → ✅ **resolved — see §9.9** (build-once + pull-latest-snapshot +
  `updateVertices` on the render thread; lock-free atomic-`shared_ptr` handoff, coalesce-to-latest;
  fixed topology or rebuild; unified with `state://…?data` live sources).

*All scene-graph open questions are now resolved.*

### 9.9 Streamed-source contract

Closes the last open scene item. A node's `source:` may be a URI (§13, incl. `state://<node>?data`
live), a procedural generator, or `{stream: {...}}`. This section defines `{stream:}` and unifies it
with the `state://…?data` live path so both share one consumer discipline.

**9.9.1 Model — build-once, stream-latest.** A streamed source drives a node whose **topology is
fixed for the binding's lifetime**. The mesh is built once (`setGeometry`); thereafter only *attribute*
arrays are overwritten per frame via the topology-preserving fast paths `GeometryNode::updateVertices`
(positions, `[x,y,z,…]` float64), `updateColors` (per-vertex `[r,g,b,…]` uint8), `updateNormals`
(`[nx,ny,nz,…]` float64). Each is a full-array overwrite of the existing `vtkPolyData` buffer +
`Modified()` + `requestRender()`. Positions are the only required channel; colors/normals are opt-in
and require the mesh to have been *built with* those arrays (else the call silently no-ops). This is
the nav demos' `sim_thread → AgentGlyphs → updateVertices` shape, standardized as scene config.

**9.9.2 The `{stream:}` schema.**

```yaml
source:
  stream:
    topology:                                     # fixed-topology declaration (one of)
      instanced: { template: <mesh-ref>, count: <N> }   # merged N×template
      # or: vertices: <point-count>                      # explicit flat count
    channels:
      positions: required                         # xyz float64, 3*point_count
      colors:    optional                         # rgb uint8; needs a per-vertex-colored mesh
      normals:   optional                         # nxyz float64 unit
    handler: <name>                               # a registered §14 handler (C++/Python)
    # OR (mutually exclusive):
    live: state://<node>?data                     # declarative pull-latest source (9.9.6)
    pacing: { hz: <sim-rate>, cap: <max-catchup-ticks> }
    observe: <bool|node-ref>
```

`point_count` is derived once (`count × template.verts`, or `vertices` verbatim) and frozen. Every
frame the source must produce exactly `3*point_count` position elements (+ matching colors/normals per
declared channel); **any other length logs at level 1 and no-ops** — it never resizes or corrupts.
`handler` (imperative host-computed buffer) and `live` (declarative coalesced) are mutually exclusive.

**9.9.3 Buffer ownership & per-frame cost.** GeometryNode owns the `vtkPolyData`; the producer owns a
reused allocation-free scratch buffer (the `AgentGlyphs::xyz_` model) and never mutates a buffer after
publishing it; the consumer **copies it in** — there is **no zero-copy vertex path**. Per
`updateVertices` for `n` points: (1) the input vector is captured by value into the `runOnMainThread`
lambda (~24n bytes), (2) copied again into the VTK array (double→float cast; `memcpy` for colors;
per-point `SetTuple3` for normals — no bulk fast path), (3) a **full-array GPU VBO re-upload** next
frame (MTime-driven; no partial/dirty-range upload). Budget O(n) CPU copy twice + O(n) upload per
streamed channel per frame. *Zero-copy exists only for textures* (`setTexture(zeroCopy=true)` aliases a
pinned RGBA8 buffer); there is no aliased/pinned path for points/colors/normals. The Python write path
(`updateVertices(PyObject*)`, a hand-written `%extend`) takes a C-contiguous float64 buffer-protocol
object (a numpy `.ravel()`) and **copies** it — the array need not outlive the call and is not pinned.

**9.9.4 Push vs pull + thread model (the load-bearing rule).** *Producer writes snapshots OFF the
render thread; the consumer PULLS the latest on the render thread and calls `updateVertices` there.*

- **VTK is render-thread-affine.** All three stream calls wrap the mutation in `runOnMainThread` —
  inline on the SceneGraph owner (render/main) thread, or marshaled onto its event queue (drained by
  `processEvents()`, weak_ptr-guarded) from any other thread. So a producer *may* call from any thread,
  but the mutation always lands on the owner thread, deferred to the next `processEvents()` drain.
- **Consumer (pull-latest):** on the render thread, the binding does one atomic load of the latest
  snapshot, **holds that one snapshot for the whole frame** (so body/label/minimap-dot/follow-cam all
  see one consistent frame), packs it into the reused buffer, and calls `updateVertices`.
- **Handoff = lock-free latest-pointer; coalesce-to-latest, never a queue.** The producer builds the
  whole frame into a fresh **immutable** snapshot, publishes it by one `std::atomic_store(&latest_, snap)`;
  the consumer reads by one `std::atomic_load` (the verified `cvc::nav::sim_thread` mechanism). No torn
  frames, no hot-path lock, and **automatic backpressure** — a slow consumer never observes intermediate
  snapshots (published-but-never-read, freed on refcount drop). **Latest-wins is the ONLY delivery
  semantics; never buffer a backlog for a renderer.** (A seqlock is an acceptable POD equivalent.) This
  is the `state_publisher` model (last-value-per-path, background writer, eventually-consistent reader)
  applied to geometry.

**9.9.5 Topology-fixed invariant — stream vs rebuild.** A change in instance/vertex **count is a
REBUILD** (`setGeometry` rebuilds points/cells/colors/tcoords + `ensureNormals`), built **async
off-thread while the old binding keeps streaming**, then hot-swapped in one frame with a size-match
guard (skip a stale-sized snapshot during the swap window). **Bounded per-frame-varying selection stays
a stream** via degenerate-collapse: keep topology fixed and collapse unused slots to a degenerate point
(zero-area, rasterizer-discarded; the `pack_lod` trick). Rule: structural change (count, cell set,
first-time color/normal array) ⇒ rebuild; pose/color/normal of fixed topology ⇒ stream; bounded
selection within capacity ⇒ stream + degenerate-collapse.

**9.9.6 Unification with §13/§14 — one consumer, two producers.**

- **`handler:` (imperative).** The named handler is registered through the §14 seam — a `std::function`
  wrapped from a C++ `native_fn` or a Python callable via `make_python_native_fn` (not a director). It
  produces a fresh buffer every frame in lockstep with the tick. Right when the producer already runs
  per-frame and every frame differs (AgentGlyphs).
- **`live: state://<node>?data` (declarative, coalesced, no handler code).** The §13 resolver decodes
  `node.data()` via the codec registry; the consumer re-reads on `dataChanged` and pulls the latest.
  Some producer writes the node; writes coalesce last-value-per-path on the `state_publisher` cadence,
  so geometry pulls one refresh per flush regardless of write rate. Right when data changes but not every
  frame, when multiple views want the same latest snapshot, or when write rate should be decoupled from
  frame rate. `&snapshot` freezes it to one read.

Both share the pull-latest-on-render-thread consumer (9.9.4). **GIL/thread rule for a Python producer
(both lanes):** pycvc releases no GIL and the scheduler is synchronous, so a Python producer **must not
produce on the render thread** (it would run inline with the GIL held and stall the whole frame) — it
produces a **snapshot on a worker thread** and hands it over (`updateVertices` copies + marshals to the
render thread, whose raw `vtkFloatArray` write needs no GIL). For the `live:` path the §14.3 rule holds:
a Python writer under a `dataChanged` watch may run on a state-writer thread, must `PyGILState_Ensure`
around every crossing, must be thread-safe, and holds its callable as a `shared_ptr<PyObject>` with a
GIL-safe DECREF deleter.

**9.9.7 Pacing / backpressure.** The producer owns a **fixed-dt clock decoupled from render rate**
(SimPacer: accumulate `wall_dt*speed`, emit whole ticks, **cap catch-up and zero the carry on a stall**
— drop time on a hitch, never burst / spiral). Display rate never changes sim speed. Coalesce-to-latest
(9.9.4) *is* the backpressure — faster sim ⇒ skipped snapshots freed; faster render ⇒ re-reads the same
snapshot harmlessly.

**9.9.8 Config/state + observability.** The stream config is a scene-node subtree (`source.stream.*`)
surfaced in `ui.docs` like any node config. With `observe:` set, the binding publishes `fps`,
`last_update`, packed byte count, and the pacer's `behind`/dropped-tick counters to a child telemetry
node (the §7.8.6a mechanism), coalesced — one node write per frame.

**9.9.9 wasm single-threaded fallback.** With no worker thread (wasm without SharedArrayBuffer — gate on
`__EMSCRIPTEN_PTHREADS__`, **never** `__EMSCRIPTEN__` alone), the runtime drops the worker + snapshot +
atomic handoff and **steps the source inline** each render frame into reused buffers on the render
thread, running the identical pack/`updateVertices` path (sim advance then couples to frame rate). The
contract defines the snapshot *shape* and latest-wins semantics abstractly; worker-atomic-handoff vs
inline-step is a runtime choice gated on real thread availability, transparent to the binding.

**9.9.10 Honest caveats.** Per-frame two-copy + full VBO re-upload is unavoidable on this path (no
dirty-range/partial upload, no pinned vertex buffer — zero-copy is texture-only); large `n` at high fps
is copy/upload-bound. Python-on-the-render-thread is **banned** for non-trivial producers. The mapper
sets no static/dynamic draw hint (a future GL draw-usage optimization is a `vtkPolyDataMapper` change,
not a DSL change). `updateColors`/`updateNormals` require the mesh to carry those arrays — declaring
those channels is a promise the initial `setGeometry` set them up, validated at bind time.

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
        bind: /cvcgl.graphics.root.children.skull.transfer_function   # the node's TF data() channel (§11.3)
        interactive_updates: true                   # emit-on-drag vs on-release (ColorTable2's flag)
        layers: [ histogram, contour_spectrum ]     # overlays supplied FROM OUTSIDE as generic
                                                     # 1D-function / 2D-geometry layers (no VolMagick
                                                     # knowledge baked in — per ColorTable2's own TODO)
```

Because the TF lives on `data()`, the same editor drives `volren`, `volslice`, and (once
`VolumeNode` re-reads its TF) the VTK path, and round-trips into the scene file. Persistence
targets the roadmap's JSON form (`axes` + `primitives` + optional keyframes + optional baked
LUT); a **load-only importer** (`load_vinay`) reads legacy ColorTable2 `.vinay` text presets.

---

## 11. Where a loaded UI lands in the state tree

There is **one per-app root state** (`cvc::state::instance(app&)`); there is **no `cvcgl` wrapper
above it and no app-id segment**, and `SEPARATOR` is `.` (dot). `cvcgl` (the default `SceneGraph`
prefix) is a *direct child* of the app root, and so are `state_exec` and the UI subtree. The app
root concretely holds:

```
cvcgl.graphics.root.children.<node>.{position,rotation,scale,matrix,show_bbox,children.*}   # scene nodes (CSV keys)
cvcgl.shadows.{enabled,resolution,interval}
cvcgl.lighting.{key_intensity,…,stage_x,…,ambient,show_gizmos,…}          # the StageLighting rig
cvcgl.viewers.<v>.{camera.*, ui.*, hud.*, layout.*}                        # per-viewer objects
cvcgl.active_viewport
state_exec.defaults.<key>      state_exec.schedulers.<id>.<key>            # scheduler config (only state_exec footprint)
```

(`cvcgl.viewers.<v>.ui` = `UiSettings` `visible/scale/touch_mode/panels_open/toggle_button`;
`.camera` = `CameraController` mode/pose/settings/keys/track; `.hud` = `FpsHud`; `.layout` =
`ViewportLayout`.) Node keys are flat CSV strings (`position`="x,y,z", `matrix`=16 CSV), published
through the scene's coalesced `state_publisher`.

### 11.1 The UI subtree — `ui.docs.<docId>`, a sibling of `cvcgl`

A loaded UI document deserializes into its **own top-level subtree**, *not* under
`cvcgl.viewers.<v>.ui` (already owned by `ImGuiOverlay`; and a UI routinely spans multiple viewers
and scenes, so it can't nest in one viewer):

```
ui.docs.<docId>.meta                  viewer:, prefix:, source, ui version
ui.docs.<docId>.tree.<id>.*           the widget tree — one state_object per widget (11.2)
ui.docs.<docId>.panels.<panelId>.*    per-panel transient state + the panel's chroot root for its handlers
ui.docs.<docId>.values.<id>           loader value: store for transient widgets with no state home (rare)
ui.docs.<docId>.ev.<nodeId>.<kind>    the msg-send/msg-recv channels for §7.1 resident handlers
ui.docs.<docId>.includes.<as>.*       a load:'d sub-UI's namespaced subtree + chroot (§12)
```

### 11.2 The widget tree mirrors the scene-node idiom

Each widget deserializes into `ui.docs.<doc>.tree.<id>` using the **same `state_object` convention
scene nodes use** — flat CSV keys + `.children.<name>` recursion — so a Qt widget port maps onto a
state subtree the same way a `GraphicsNode` does:

```
ui.docs.<doc>.tree.<id>.geometry      "x,y,w,h"
ui.docs.<doc>.tree.<id>.size_policy    "expanding,fixed"
ui.docs.<doc>.tree.<id>.layout         "vertical"
ui.docs.<doc>.tree.<id>.visible / .enabled / .collapsed
ui.docs.<doc>.tree.<id>.children.<name> …   (recursion, same spelling as GraphicsNode)
```

Spelling the container key `.children.<name>` lets one traversal/bind machinery walk both trees.
(Cross-run persistence stays out of scope, decision #4 — this is the *runtime* tree.)

### 11.3 How `bind: <path>` resolves — three cases

- **Bare relative** (`bind: some.tunable`) → against the widget's **panel/doc chroot root**
  (`ui.docs.<doc>.panels.<panelId>` or `ui.docs.<doc>`). The common case for promoted-local
  tunables (decision #2); two includes of a unit can't clobber each other (§7.8).
- **Leading `/`** (`bind: /cvcgl.graphics.root.children.bunny.show_bbox`) → **app-root-absolute**,
  the *only* way to reach scene-node state (scene nodes live outside any UI chroot). Realized at
  the **loader level** (a second un-chrooted resolver / a link node), not by the intrinsics (§7.8.2).
- **Scene-qualified** → splice onto the doc's `meta.prefix` via the canonical helpers, never concat:
  `<prefix>.graphics.root.children.<node>.<key>`, `<prefix>.shadows.<key>`, `<prefix>.lighting.<key>`,
  `<prefix>.viewers.<v>.camera.<key>`.

A **typed/structured** tunable (color, vector, TF) routes through the `data()` channel at the same
node path (§4.3, §10). A document declares its mount id + bindings in the header:

```yaml
ui: 0.3
doc: nav_city_drive        # → ui.docs.nav_city_drive.*  (mount prefix; default = source basename)
viewer: main               # → meta.viewer ; binds camera/hud/ui host to cvcgl.viewers.main
prefix: cvcgl              # → meta.prefix ; scene-qualified binds splice onto this
chroot: per_panel          # per_panel (default) | per_doc  (the §4.8/§7.8 granularity)
```

Two UIs over one scene get distinct `ui.docs.<docId>` subtrees and both bind (absolute) to the
same `cvcgl.*` scene keys, which two-way binding reconciles.

### 11.4 Lifecycle, coexistence, and runtime persistence

The state subtree **is** the document at runtime — there is no separate in-memory document object —
which is what makes every widget bindable/watchable and lets a second viewer or a `state_observer` see
the whole UI. `meta` is written once at load (`viewer`/`prefix`/`source` URI/`chroot`/`mounts` — the
allocated `<as> → resolved-URI + prefix` table for the cycle guard and hot-reload) and is read-only
after.

**Unload is the mirror of load and must be ordered:** (1) `kill_process` every resident handler in the
doc (so no tick writes into a subtree mid-teardown); (2) `state-unwatch` every watch rooted in it; (3)
`state-expire`/`state-delete` the whole `ui.docs.<docId>`. **Kill-then-delete, never delete-then-kill** —
handler bodies live only in the scheduler's in-memory `processes_` map and still hold `&ctx` into the
nodes, so deleting the subtree first orphans live processes onto freed state (the `intrinsics_context`
must outlive its handlers). A `load:`'d mount unloads the same way scoped to `includes.<as>.*` — hot-reload
(§12) is exactly a scoped unload + reload of one mount.

**Two docs over one scene** share nothing at the UI layer (distinct `ui.docs.<docId>` subtrees); they
meet only where both bind (absolute) the same `cvcgl.*` scene keys, reconciled by the coalesced
publisher. This is *why* the UI subtree is a top-level sibling and not nested under
`cvcgl.viewers.<v>.ui` — a doc routinely spans multiple viewers/scenes.

**Runtime widget geometry/layout/visible/collapsed persist in the tree, not across runs.** A user
drag/resize/collapse writes back to `ui.docs.<doc>.tree.<id>.{geometry,visible,collapsed}` (the two-way
edge): the walk reads those to seed `SetNextWindowPos/Size/Collapsed` when present, then writes the
post-interaction values back at end-of-frame, so the state node is the authority next frame.
`layout`/`size_policy` are authored keys a handler *may* rewrite to re-flow live. Cross-run persistence
stays out of scope (decision #4) — and a `save()` of `ui.docs.<doc>` would capture only the `value()`
channel (geometry/visible/collapsed persist; a widget's `data()`-channel typed model does not — the
§13 `?data` caveat), so durable layout would need an explicit codec, not a free `json()` dump.

---

## 12. Loading sub-UIs and sub-scene-graphs (modularization)

`include`/`repeat` (§3.7) template widgets **in-document**. `load:` is the **cross-file module**
primitive: it mounts an **external** fragment (a `.ui.yaml` sub-UI or a `.scene.yaml` sub-scene) **as
its own namespaced subtree with its own chroot**. As of v0.8 its argument is a **URI** resolved through
the shared `uri_resolver` (§13), so a fragment can come from a file, a web URL, or a state node — the
same resolver that backs `include:` and node `source:`.

### 12.1 `load:` a sub-UI fragment

```yaml
root:
  - widget: telemetry_dock
    type: group
    layout: { kind: vertical }
    children:
      - load: file://panels/rf_telemetry.ui.yaml   # or state://…  or https://…  (bare = file:, §13)
        as: rf                                # mount id under this parent
        args: { unit: alpha, max_range: 4000 }   # substituted before parse, like include args
        prefix: null                          # optional scene prefix for the fragment's scene-binds
        reload: on_change                     # off (default) | on_change | poll:<sec>   (hot-reload, §12.5)
```

- **Namespaced state.** The fragment's tree lands at `ui.docs.<doc>.includes.rf.*`, and its
  resident handlers (§7.1) are chrooted to that subtree via a fresh `apply_chroot` (§7.8.1); its
  `ev.*` channels are namespaced `ui.docs.<doc>.ev.rf.<node>.<kind>` (the scheduler queue keys on
  the raw string, §7.8.5). An auto-`PushID` on the mount guards ImGui id collisions.
- **Nested chroot** = the loader chrooting each level's handlers to the deeper path at wire time
  (no in-evaluator nesting, §7.8.7).
- **Binds** resolve against the mount prefix: a bare `bind:` → `ui.docs.<doc>.includes.rf`; `/` →
  app root (loader-realized); scene-qualified → the fragment's `prefix:` arg (default = host's
  `meta.prefix`). A **relative `load:`/`source:` inside the fragment** resolves against the URI it was
  loaded from (its `mount_base`, §13).

### 12.2 `load:` a sub-scene-graph

Because **every scene path is `<prefix>.`-relative**, giving a fragment its own `SceneGraph` prefix
namespaces the whole sub-scene for free:

```yaml
scene:
  nodes:
    - load: scenes/city_block.scene.yaml      # a scene fragment
      as: block_a
      prefix: cvcgl.block_a                    # SceneGraph(app, "cvcgl.block_a") — its whole subtree
      transform: { position: [500, 0, 0] }     # host-applied mount transform
```

The fragment's nodes become `cvcgl.block_a.graphics.root.children.*`; its lights/shadows/viewers are
under `cvcgl.block_a.*`. A PiP/minimap over a second scene (§9.6) is exactly a `view_embed` whose
camera is `cvcgl.block_a.viewers.<name>.camera`. Nothing enforces prefix uniqueness, so the loader
**allocates** the prefix (`<parent>.<as>`) and records it in `meta`.

### 12.3 `include:` vs `load:` vs `source:` — one resolver, different isolation

All three route through `uri_resolver` (§13) but differ in **isolation**, not in fetch:

| | routes through §13 | fetches | isolation |
|---|---|---|---|
| `include:` (§3.7) | no — an in-doc `units:` template | nothing external | **shares** the enclosing chroot; in-document template |
| `load:` (sub-UI) | **yes** | file/http/state fragment | **own** `ui.docs.<doc>.includes.<as>` chroot + `ev.*` + PushID |
| `load:` (sub-scene) | **yes** | a `.scene.yaml` fragment | **own** `<prefix>.*` `SceneGraph` subtree |
| node `source:` (§9.4) | **yes** | file/http/state/stream payload | **none** — the payload is *data* decoded into the owning node; no handlers |

`include` is a **template** (no fetch, shared scope); `load` is a **module mount** (fetch + isolation);
`source` is a **data reference** (fetch, no isolation — it yields a geometry/volume/image/value into an
existing node). `{stream:}` (§9.4) stays a special case but registers through the **same** handler seam.

### 12.4 Recursion / cycle guard

`load:` carries a **max mount depth** and a **visited-set cycle guard keyed on the fully-resolved URI**
(`file://…` canonicalized to an absolute path, `state://a?value` on its resolved node path, `https://…`
on the normalized URL) — `a → b → a` hard-errors with the mount chain. Keying on the *resolved* URI
(not the literal string) catches a `state:` indirection that loops back to a `file:` already on the
stack. Args are substituted before parse, so a fragment is `requires:`-preflighted (§7.6) **in its own
mount chroot env** — a fragment needing an intrinsic this build lacks fails at load, naming the mount.

### 12.5 Hot-reload

`reload: on_change` watches the source and, on change, does a **scoped unload + reload of that one
mount** (§11.4 teardown restricted to `includes.<as>`: kill the mount's handlers → unwatch →
`state-expire includes.<as>.*` → re-resolve the URI → re-mount); the rest of the doc is untouched. The
watch is scheme-appropriate — `file://` mtime/inotify, `state://` a `state-watch` on the target node
(`valueChanged`/`dataChanged`/`childChanged`), `https://` an ETag or `poll:<sec>`.

---

## 13. Resource loading — URIs, schemes, and the handler registry

One resolver backs every external reference: `load:`/`include:` fragments **and** node `source:` data.
Today these are three unrelated filename-only paths (`load()` is SWIG `%extend`s on volume/geometry/image
calling `read(filename)`; there is **no** URI/scheme layer in `state_exec`). §13 specifies the one layer
they share.

**Hard constraints (verified in-tree):** every reader is **filename-only** (`read_geometry(path)`,
`readVolumeFile(app&, vol, path)` — note the required `app&`, `image::load(path)`); **libcvc has no C++
HTTP client** (only XmlRpc++ and the state-replication transport); **no temp-file helper exists** under
`inc/cvc`; and a `cvc::state` node has **three channels** — `value()` (string; the only one `json()`/`save()`
persist), `data()` (a `boost::any`, in-memory only), and the child subtree — with link nodes for live
indirection.

### 13.1 URI grammar and schemes

```
<scheme>://<path>[?<query>]
bare/relative/path.ui.yaml      # no scheme → file:, resolved against the enclosing fragment's mount_base
```

| Scheme | Resolves to | Backed by |
|---|---|---|
| `file://` (+ bare/relative) | a filesystem path | in-process, always available |
| `state://<node.path>[?value\|?data\|?children][&snapshot]` | a node's value / typed `data()` / child subtree | in-process |
| `http(s)://` | fetched bytes (or a cached temp path) | **host-supplied** handler (Python `requests`; emscripten `fetch` under wasm) |
| *custom* (`pkg://`, `s3://`, `mem://`, …) | whatever the handler returns | registered by C++ or pycvc (§14) |

### 13.2 The scheme-handler registry (modeled on `image_file_io`)

A URI is claimed by scheme/pattern — registration-order priority + a `can_open` predicate, like
`image_file_io::can_read`, **not** the extension-keyed `geometry_file_io` map. A handler returns a
`resource` = one of `{bytes}`, `{local_path}` (a real file a reader can open), or `{value}` (a string or
a `value_t` tree), plus a `media_hint`.

```cpp
namespace cvc::state_exec {
  struct resource { enum class kind { bytes, local_path, value } k; /* bytes | path | value_t */ std::string media_hint; };
  struct resolve_context { cvc::app* app; cvc::state* root; std::string mount_base; int depth;
                           const std::set<std::string>* in_flight; };   // built from the intrinsics_context
  struct uri_resolver {
    static void register_handler(uri_handler::ptr);                     // append; first can_open wins
    static void register_scheme(std::string scheme, std::function<resource(std::string_view, const resolve_context&)>);
    static resource resolve(std::string_view uri, const resolve_context&);
    static void register_default_handlers(cvc::app&);                   // lazy, app-driven — dodges static-init deadlocks (cf. io_handlers.h)
  };
}
```

`resolve_context` is populated from the `intrinsics_context` the evaluator already carries (`root`,
`uid`, `root_path`, …) — no new plumbing — and `mount_base` threads the enclosing fragment's URI so a
relative `source:`/`load:` inside it joins RFC-3986-style (`pkg://austin/scene.ui.yaml` + `blocks.off` →
`pkg://austin/blocks.off`).

### 13.3 `state://` — a node's value, data, or subtree

```
state://scene.ui.panel             # ?value (default): node.value() — a string
state://scene.ui.panel?value       #   inline YAML/DSL text  OR  another URI to re-dispatch (indirection)
state://scene.geom.mesh?data       # node.data() — a boost::any; decode by type via state_codec_registry
state://scene.tf.ramp?children     # the child subtree read back as a value_t tree (decode_value)
```

- **`?value`** — the durable, human-editable channel: **inline** (parse the string as a fragment) or
  **indirection** (the string is itself a URI → re-dispatch; this is how `load:` unifies with every scheme).
- **`?data`** — a serialized geometry/volume/image on `node.data()`, decoded via `state_codec_registry`
  keyed on `type_name` (`cvc.geometry.v1`, `cvc.volume.v1`, …); large/remote blobs arrive
  content-addressed (`state_blob_store`) and hydrate lazily (`state_data_hydrator`).
- **`?children`** — a UI/TF fragment authored *as state* (node-by-node editable) rather than opaque text.

**Live vs snapshot** — the key knob. A `state://` reference is **live** by default: if the target is a
link node (`state::linkTo`, transparent) the resolver `resolveLink()`s and the consumer re-reads on
`valueChanged`/`dataChanged` — it *tracks* the object. `&snapshot` instead `decode_value`/codec-decodes
once and drops the reference (frozen, for reproducibility). **A `source:` at a node is the right choice
over `file://` precisely when the data changes at runtime** — the per-pid badge node (§7.8.6a) and the
streamed source are exactly this.

### 13.4 How each consumer uses one `resolve()`

- **`include:`/`load:`** — expect text (`value` string or `bytes` as UTF-8) → parse as a fragment.
- **node `source:`** — `local_path` → hand to the existing reader (`read_geometry`/`readVolumeFile(cx.app,…)`/
  `read_image`; `image_file_io` magic-sniff means an extensionless URI still resolves by content); `bytes`
  → **temp-file bridge** (write → read → delete; needs the missing temp helper — the zero-reader-change
  path); `value` holding a `data_object` that is an already-decoded `cvc::geometry`/`image` → bind directly,
  **no reader**. So `source: file://…` and `source: state://…?data` are interchangeable at the node — the
  node declares `kind`, the resolver yields path/bytes/object, dispatch is by kind.

### 13.5 Caching and honest caveats

`state://` results are **never cached** (live by construction); `file://` may cache the path but honors
mtime for the dev loop; `http(s)` handlers own their cache (content-addressed temp files). Provide
`invalidate(uri)`/`clear_cache()`.

- **`state://…?data` does not survive persistence or a process hop by itself** — `boost::any` is in-memory
  only; a persisted/streamed UI referencing `?data` **must** have a registered codec for its `type_name`,
  or fall back to `?value`/`?children` (which persist). The resolver surfaces *"no codec for `<type>`"*,
  never a silent empty `any`.
- **The temp-file bridge is a real dependency** — `kind::bytes` → reader is dead until a temp helper is
  added; until then only handlers that name a real cache path (`kind::local_path`) feed the readers.
- **`http(s)://` is never in-core** — a bare `https://` in a `.ui.yaml` fails "no handler for scheme https"
  on a host that didn't register one (§14).

---

## 14. pycvc — registering Python intrinsics and Python URI handlers

Two capabilities, **one mechanism**, so a demo or custom UI extends the DSL/resolver in Python exactly as
C++ does. pycvc is **SWIG** (`%module(directors="1") pycvc`), which decides the mechanism: a **director**
fits only a C++→Python *virtual override* (used today for exactly one class, `state_observer`); a **Python
callable stored as a `std::function`** (`native_fn` / a URI handler) needs a **manual PyObject-holding
wrapper** — which the repo already ships and tests as `make_python_native_fn`. Both features reuse it; do
not add a director. (The one refactor: lift the wrapper out of its anonymous namespace so `register_fn`
and `register_uri_handler` share one body.)

### 14.1 Python callable as a state_exec intrinsic — ~90% already shipped

`pycvc.Exec(app).register_fn(name, callable)` already wraps the callable via `make_python_native_fn` and
installs it with `builtins::register_fn(env, name, fn)`, callable from DSL source `(name arg…)` like any
builtin (verified in `test_pycvc_exec.py`). Expose it to the UI DSL as **`register_intrinsic(name,
callable)`** (wrapper identical). Context (pid/scheduler/root) isn't in the `native_fn(span)` signature;
the tested idiom is a Python fn **closing over the `app`** and calling `pycvc.state_get/set/…` — keep that
the default, add a `register_ctx_fn` (prepends a lightweight context arg0) only if a concrete intrinsic
needs scheduler control.

```python
import pycvc, requests
ex = pycvc.Exec(app)
ex.register_intrinsic("kpi", lambda rows, f: sum(r[f] for r in rows))   # DSL: (kpi (state-children "m") "value")
ex.register_uri_handler("https", lambda url: requests.get(url, timeout=10).content)   # bytes → node source
```

### 14.2 Python URI-scheme handler — new surface, same wrapper

Because libcvc can't fetch, `http(s)://` and custom remote schemes come from the host, and a Python
`requests` handler is the easy path. `Exec.register_uri_handler(scheme, callable)` (plus an **app-level**
variant, since node sources resolve outside any Exec) INCREFs the callable into a `shared_ptr<PyObject>`
with a GIL-safe DECREF deleter; the lambda does `PyGILState_Ensure` → build the URL `str` → `PyObject_CallObject`
→ convert the return (`str`→string payload, `bytes`→blob, `dict`/`list`→structured) → `Py_DECREF` →
`Release`; a null return is `PyErr_Fetch`'d and thrown → surfaces as a **load error**, not a crash.

> **Gap to fix before shipping binary handlers:** `py_to_value` has **no `bytes` branch** today — a
> Python `bytes` return falls through to `str(o)` (a `b'…'` repr), wrong for a handler returning binary. Add
> a `bytes` → `data_object`/blob branch.

### 14.3 The GIL / thread-affinity / lifetime contract (document on the API)

- **GIL:** every crossing into Python — the call, arg/return marshaling, and **every** refcount change
  including the final DECREF — holds the GIL via `PyGILState_Ensure`/`Release` (from-any-thread), never bare
  `Py_INCREF`/`DECREF`.
- **Thread affinity:** a handler/intrinsic may run on (1) the `Exec.run()` caller thread (the scheduler is
  **synchronous** and `pycvc.i` releases no GIL, so today a Python intrinsic runs **inline with the GIL
  already held** — `PyGILState_Ensure` is a safe recursive acquire, no marshaling needed); (2) a background
  worker if resolution moves off-thread (`async_scheduler`); (3) the **state-writer thread** when a node
  source refreshes under a `childChanged` watch (writer threads may hold no GIL — the `state_observer`
  director proves the pattern). `PyGILState_Ensure` is correct in all three; the callable must be
  thread-safe and must not assume the main interpreter thread.
- **Lifetime:** the `shared_ptr<PyObject>` is captured by the `std::function`, so the callable lives as long
  as the env/registry (Exec/doc/app) that holds it; the deleter re-acquires the GIL because teardown can come
  from a non-Python thread or interpreter shutdown.
- **Exception containment:** a Python exception is fetched, stringified, and thrown as `std::runtime_error`
  so it unwinds through the try/catch-free scheduler; `Exec.run()` reaps the pid on throw.
- **Re-entrancy:** a Python handler that re-enters `Exec.run()` on the **same** Exec is GIL-safe but the
  synchronous scheduler is **not** re-entrant — unsupported, or route to a fresh scheduler.

### 14.4 Symmetry

| Capability | C++ | pycvc |
|---|---|---|
| DSL intrinsic | `builtins::register_fn(env, name, native_fn)` | `Exec.register_intrinsic(name, callable)` |
| URI scheme handler | `uri_resolver::register_scheme(scheme, fn)` | `Exec/app.register_uri_handler(scheme, callable)` |
| C++→Python callback (existing) | virtual interface | `%feature("director")` (`state_observer`) |

Same registries, same marshaling, same GIL discipline — a capability added in either language is
indistinguishable to the DSL and to node sources. (A host that prefers to *subclass* a C++ reader/URI
registry in Python — override `can_open`/`read_bytes` on a `uri_io` class — is the director route instead;
rule of thumb: **`std::function` handlers = wrap-in-`native_fn`, no director; class-based reader Python
overrides = director-subclass**.)
