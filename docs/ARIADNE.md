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
12. [Golden-image tests](#12-golden-image-tests)

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

- **`ariadne_hello`** ([`examples/ariadne/hosts/ariadne_hello.cpp`](../examples/ariadne/hosts/ariadne_hello.cpp))
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
  *readers* only (`kAllowed`, [`ariadne.cpp:417`](../src/cvc/ariadne/ariadne.cpp)), plus
  `(has? "name")`, which asks what the host provides
  ([§5.7](#57-requires-and-has--what-a-document-needs-from-its-host)).
- The special-form gate admits only `if / begin / let / while / for / lambda / defun /
  set / return / quote / yield / break` (`kAllowedForms`,
  [`ariadne.cpp:433`](../src/cvc/ariadne/ariadne.cpp)).

Nothing that writes state, schedules, watches, sends messages, or does I/O is
reachable. That includes the atomic updates `state-incr`, `state-cas` and `state-swap`: they
write, so they are not on the allowlist (actions may use them). An off-allowlist symbol does not error loudly — it **fails safe**: the slot
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

| bound | source | value |
|-------|--------|-------|
| step cap | tree setting `ariadne.limits.action_max_steps` | `5000` by default; `0` = no limit |
| time cap | tree setting `ariadne.limits.action_max_time` | `0.1 s` by default; `0` = no limit |
| memory budget | tree setting `ariadne.limits.action_max_memory` | `4 MiB` by default; `0` = no limit |

The settings are read on every submission, from the app's tree: they are the host's, outside any
document's prefix, so a document can't raise its own. A missing or unparsable value keeps the
default.

The step and time caps cover the action's async calls (§10.1; developer guide, "Async
Functions"): a call's run time and CPU time count against the time cap, and its CPU time as steps
at `limits.async_steps_per_cpu_ms` (8000 per CPU millisecond by default). At that rate the
5000-step cap is 0.625 ms of a kernel's CPU, so a host whose documents call CPU-heavy async verbs
raises the caps. An action that runs out is killed, and the kernel sees `cancelled()`.

The memory budget covers the action's state-tree writes and its
measured share of the values it holds (developer guide §7), and it bounds each native call's
allocations: state_exec's per-step allocation limit takes what is left of `max_memory`
(developer guide §11, "Allocation limit"). A finished action's process is reaped from the app
scheduler when the drain sweeps it, and what it left in reference cycles (a function it defined,
say) is freed by the cycle collector.

A quick action completes within that drain. An action that `await`s, `sleep`s, or
`msg-recv`s **parks** and resumes on a later frame — this is what makes the async arc
(§10) possible without blocking the frame.

### 2.3 Resident handlers

`on_tick` / `on_key` / `on_pointer` / `on_signal` each become **one** long-lived, owner-tagged
process, wrapped by the runtime as `(while t (let ((event (msg-recv "chan"))) body))`
(`ensure_resident`, [`ariadne.cpp:1458`](../src/cvc/ariadne/ariadne.cpp)):

- **tick** is woken once per drain (coalesced — you get one tick per frame, not a
  backlog).
- **key / pointer** are woken per event; the event is delivered as a dict you read with
  `(get-attr event "key")`, `(get-attr event "x")`, etc.
- **signal** is woken per signal the host routes through `Runtime::post_signal`, with
  `event` = `{kind: "signal", signal: "SIGINT", source: "terminal"}`. Signal names are
  POSIX: a terminal's Ctrl-C is `SIGINT`, Ctrl-\ is `SIGQUIT`.

Residents have no total budget (they run for the life of the document), but each *step*
is wall-capped at `kResidentStepSeconds = 0.05`
([`ariadne.cpp:1497`](../src/cvc/ariadne/ariadne.cpp)). On teardown they are reaped as a
group via their owner tag.

### 2.4 Errors in actions and residents

Handlers raise and catch exceptions like any state_exec program, with `try`/`catch`/`finally`,
`raise` and `error` ([STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md)). A builtin's or a
host intrinsic's error is an exception of its class too. An allocation an action's memory budget
refuses is a `MemoryError`, for example, so a handler can catch it.

**What the drain reports.** An exception a handler doesn't catch ends its process `failed`. The
drain sweep warns once per message, through the warnings `take_reactive_warnings()` returns:

| What ended | Warning |
|---|---|
| an action, by an exception | `ari: on: action program failed: Class: message (at <where>)` |
| a resident, by an exception | `ari: resident [<channel>] failed: Class: message (at <where>)` |
| an action, by a kill (its step or time cap, a syntax error found while reading it) | `ari: on: action program did not complete: <reason>` |
| a resident, by a kill | `ari: resident [<channel>] stopped: <reason>` |

`<where>` is the innermost line of the exception's traceback. An `init:` program that fails
reports `failed: Class: message (at <where>)` to its caller.

**Keeping a resident alive.** A resident is one process for the life of the document. An
exception it doesn't catch ends it, and its channel goes unserved from then on. Catch inside the
body to keep it running:

```yaml
on_key: |
  (try (handle-key event)
    (catch (e) (state-set "ui.last_error" (get-attr e "message"))))
```

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
([`backend.h`](../inc/cvc/ariadne/backend.h)). Core writes state on `committed`, and a
slider's on every `changed` too, so whatever it drives follows the drag. A slider with
`commit: release` writes once, when the drag ends, for a key whose change is too costly to
redo every frame (the forest controls' structural sliders, which regrow the whole forest).

A backend advertises `Capabilities{windows, menubar, mouse, keyboard, color, glsl,
view_embed, owns_loop, terminal, bitmap, kitty, sixel}`
([`backend.h`](../inc/cvc/ariadne/backend.h)). A document's `requires:` and `(has? ...)`
read them through capability tokens ([§5.7](#57-requires-and-has--what-a-document-needs-from-its-host)).
`owns_loop` distinguishes a **host-pumped**
backend (the app runs the loop and calls in per frame) from a **framework-owned** one
(the backend runs its own loop).

Three backends ship:

- **`cvc::gl::ImGuiBackend`**
  ([`inc/cvc/gl/ariadne/ImGuiBackend.h`](../inc/cvc/gl/ariadne/ImGuiBackend.h)) — ImGui
  over a VTK view, the **only** place Ariadne touches ImGui or VTK. It is inert without
  `CVC_ENABLE_IMGUI`. Its caps are all-true except `owns_loop = false` (host-pumped).
  Its `install()` sets the VTK view's overlay draw callback to call `Runtime::render()`
  — that single line is the entire coupling between Ariadne and VTK's render pass, and
  it lives in the backend, not the core.
- **`cvc::ariadne::FtxuiBackend`**
  ([`inc/cvc/ariadne/ftxui_backend.h`](../inc/cvc/ariadne/ftxui_backend.h)) — the
  `owns_loop = true` counterpart that draws the same tree in a terminal, through FTXUI
  (optional: without FTXUI it is an inert stub and `available()` is false). It is
  immediate-mode over FTXUI's DOM, like ImGui: every walk rebuilds the Element tree, each
  leaf records its on-screen box, and the backend keeps only the interaction state (focus,
  pressed item, open menu or combo, in-progress text and slider edits, window placement).
  So the whole tree is interactive:
  - **keyboard** — Tab / Shift-Tab (or Up / Down) move focus; Enter / Space activate;
    Left / Right step a slider or cycle a combo; text fields edit in place; Esc reverts an
    edit or closes a popup; F10 opens the menu bar.
  - **mouse** — click to focus or activate, drag a slider, drag a window by its title
    bar, click `[x]` to close it, click a menu title to open it.
  - **windows** (`caps.windows = true`) — free, z-ordered, raised on focus, placed from
    `pos:`/`size:` or the persisted `tree.<id>.geometry`; a drag persists back (§11.4). A
    window stops at the screen's edge and scrolls what does not fit (§5.8).
  - **graphics** — `draw_image` paints a raster published with `set_image`, and
    `set_background` puts one behind everything. Where the terminal speaks a pixel protocol
    the picture is real pixels (see *Pixel graphics* below); everywhere else it is
    half-block cells (two pixels per cell, 24-bit colour; a luminance ramp on a colourless
    terminal).
  - **custom widgets** — `register_widget(type, fn)` draws a custom type with the
    backend's own primitives, as `ImGuiBackend::register_widget` does. `stream_view` is
    one: `register_stream_view_widget(backend, app)`
    ([`stream/stream_view.h`](../inc/cvc/ariadne/stream/stream_view.h)) shows a live
    video stream's frames through `set_image` / `draw_image`. It uses the same per-widget
    frame puller as the ImGui `stream_view`, which follows token switches and reopened
    streams. A compressed codec needs a decoder from the host; `arigl` adds cvcGL's JPEG
    one for `mjpeg`. `ari` and `arigl --terminal` both register it. With no live stream,
    it shows whatever still image is published under the stream's name (`--image`).

  Commit edges match the ImGui backend: discrete widgets commit on the edit, a
  mouse-dragged slider reports each move and commits on release (arrow-key steps commit at
  once), a text field commits on Enter or when focus leaves it. Authored px convert to cells by a nominal cell
  size (`Options::cell_px_w/h`, 8 × 16), and reported geometry converts back, so a layout
  persisted under one backend holds under the other. Grid seams drag: a row split's
  panes and a `resizable` table's columns resize with the mouse; a row split persists
  its pane sizes to `tree.<id>.tracks` and a resizable table its column widths to
  `tree.<id>.columns` (§11.4; *Grid sizes* in §7.1). What a terminal cannot do it
  degrades: no GLSL, no `view_embed`, no scene (`AppRuntime` drops a `scene:` with a
  warning). `render_to_string` + `feed(TermEvent)` drive it headless (that is how
  `ariadne_ftxui_test` tests it); `run()` owns a real terminal.

  **Pixel graphics** ([`term_graphics.h`](../inc/cvc/ariadne/term_graphics.h)).
  `Options::image_color` is `Auto`, `Kitty`, `Sixel`, `Iterm`, `Color` (half-blocks) or
  `Mono`.
  `Auto` takes the best the terminal has, else half-blocks:
  - **Kitty** (kitty, Ghostty, WezTerm, Konsole). Each image is sent once, out of band,
    after the frame that first draws it (`a=T,U=1,f=24,q=2`, chunked base64), and is
    shown through **Unicode placeholder** cells (U+10EEEE with row and column diacritics,
    in a 24-bit foreground that names the image). The placeholders are ordinary cells, so
    a window or menu drawn over an image hides exactly the cells it covers, and FTXUI's
    full repaint redraws them for free. An image is sent again only when its pixels
    change (a republished identical frame is not), and deleted (`a=d,d=I`) once no frame
    draws it. A changed picture goes to a second image id, and the placeholders switch to
    it at the next frame, once it has fully arrived. Re-sending under the id the cells
    show makes the terminal drop that image for the whole transfer, so a scene that
    changes every frame would never appear. A placement addresses 297 rows and columns,
    so a bigger image goes as tiles.
  - **Sixel** (xterm -ti vt340, foot, mlterm, WezTerm, Windows Terminal). A bitmap is
    drawn after each repaint, because the repaint covers it, inside synchronized-output
    marks (`CSI ? 2026`). It is clipped to the cells nothing covers: the image node draws
    half-blocks underneath, and a pass after the whole frame keeps the cells still showing
    them, sent as one bitmap per rectangle. Colours go to a fixed 6 × 7 × 6 cube (fewer
    where the terminal reports fewer registers) with an ordered dither.
  - **Iterm** (iTerm2's inline images: WezTerm, iTerm2, mintty, xterm.js's image addon).
    Like Sixel, but each rectangle is a JPEG in an OSC 1337 escape that names its cells
    (`width=` / `height=`, `doNotMoveCursor=1`), and the terminal scales it to them, so the
    cell size need not be known. `Auto` picks it, after Kitty and Sixel, when
    `$TERM_PROGRAM` names such a terminal (`gfx::iterm_terminal`: WezTerm, iTerm2, mintty)
    and stdout is a tty, off Windows. A page with xterm.js's image addon asks for it
    (`--graphics iterm`), as arigl_term's does.
  - **Detection.** `run()` asks the terminal before FTXUI takes it over, because FTXUI's
    input parser has no APC case and a late kitty reply would arrive as typed text: on
    `/dev/tty` in raw mode it sends a kitty query, `CSI 16 t` (the cell size in pixels)
    and DA1, whose reply ends the exchange. Sixel is DA1's feature 4 (FTXUI's own DA1
    exchange is merged in), and a Sixel terminal is asked its register count
    (XTSMGRAPHICS). The cell size comes from `TIOCGWINSZ` where the tty reports pixels,
    else `CSI 16 t`, and is re-read every frame, so a font-size change follows. A host
    that probed already, or a headless driver, calls `set_terminal_graphics`.
  - **Units.** `cell_px_w/h` (8 × 16) stays the layout unit, so persisted geometry stays
    portable. The real cell size only sets the raster: an image covering `c × r` cells is
    drawn from `c × cw` by `r × ch` pixels (`image_raster`), and its rows follow the real
    cell's shape. Kitty images over `max_image_pixels` (default 1 Mi) are sent scaled
    down, and the terminal scales them back up.
  - **Headless.** `render_to_string` lays out the placeholders or underlay, and
    `take_graphics()` returns the bytes `run()` would write after that frame
    (`release_graphics()`: the deletions it sends on exit).
  - **Limits.** Hosted mode (the tvision desktop) always draws half-blocks, because
    tvision owns the output. No probe runs on Windows or in the browser, so `Auto` there is
    half-blocks (`Kitty`, `Sixel` and `Iterm` can still be asked for). On Windows a console
    program reaches its terminal through ConPTY,
    which decides what gets through: Windows Terminal's (1.22 and later) passes Sixel on,
    and `arigl --terminal --graphics sixel --cell-size WxH` draws real pixels there. The
    ConPTY WezTerm 20240203 bundles, and the one in Windows itself, drop kitty's APC and
    Sixel's DCS; they forward an iTerm2 image as it comes but redraw cells later on their
    own, so a full-screen program's image lands at the wrong place and is painted over.
    So kitty graphics need a terminal on Linux or macOS. Inside tmux the probe
    gets no kitty reply, so it falls back too (tmux passthrough is not wired). A Sixel
    bitmap never covers the screen's last row (a terminal that moves the cursor below a
    bitmap would scroll), and where its last six-pixel band would reach a covered row it
    stops short, so up to five pixel rows show the half-blocks.
- **`cvc::ariadne::TvisionBackend`**
  ([`inc/cvc/ariadne/tvision_backend.h`](../inc/cvc/ariadne/tvision_backend.h)) — the
  native-desktop terminal backend (roadmap §16.5 row 5). Turbo Vision owns the terminal
  and draws the chrome itself: movable, resizable, overlapping windows with close and
  zoom boxes, a pull-down menu bar, a status line, and its own keys (F10, F6, Alt-X,
  Ctrl-F5). The windows' *contents* are the FTXUI backend's: a `TvisionBackend` is an
  `FtxuiBackend` in hosted mode (`Options::hosted`), which lays each window body out
  offscreen for tvision to draw and takes that window's input in its own cells. So both
  terminal backends share one widget engine, focus and commit rule, and the hosted mode
  is open to any host with its own chrome. Built behind `CVC_ARIADNE_TVISION` (the flag
  the roadmap keeps for tvision's Borland-derived licensing) when tvision, ncursesw and
  FTXUI are found; `ari run --backend tvision` runs it. In a browser build it needs no
  ncurses: cvcpkg's wasm tvision carries a browser console (see *In the browser* below).

Because the core is backend-neutral, the same document renders through any backend. A
document that needs something only some hosts have says so in `requires:`, or asks with
`(has? ...)` and degrades itself
([§5.7](#57-requires-and-has--what-a-document-needs-from-its-host)); the core does not
substitute widgets on its own.

---

## 5. Documents and the `.ari` schema

An `.ari` file is a YAML document. `load_file` / `load_string` parse it via `load_node`
([`loader.cpp:1740`](../src/cvc/ariadne/loader.cpp)) into a `LoadResult`
([`inc/cvc/ariadne/loader.h`](../inc/cvc/ariadne/loader.h)):

- a `Widget root` (a Group) — the retained tree;
- a parsed `Scene`, `Meta`, `customs`, and `channels` + `lint` config;
- and **verbatim-captured** script strings: `init_script`, `on_tick_script`,
  `on_key_script`, `on_pointer_script`, `on_signal_script`.

**The loader is state-free.** It captures structure and script *text*; it never touches
`cvc::state` and never runs the DSL. The host runs the scripts (§9). This separation is
why the same `LoadResult` can be driven by different hosts.

### 5.1 Top-level keys

The built-in top-level keys (`is_builtin_block`,
[`loader.cpp:97`](../src/cvc/ariadne/loader.cpp)) are:

```
meta  menubar  windows  overlays  root (alias: children)  scene  customs  requires
init  on_tick  on_key  on_pointer  on_signal  keys  units  import  channels  lint  args
windows_menu
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

`init`, `on_tick`, `on_key`, `on_pointer`, and `on_signal` are scalar `state_exec` strings
the loader only *captures*. The host wires them:

- **`init_script`** runs **once** at load time, at the document prefix, via `run_init`.
  Because it runs first, an `init:` seed such as `(state-set "demo.n" 3)` wins over a
  widget's `def:` default for the same key.
- **`on_tick` / `on_key` / `on_pointer`** become the resident processes of §2.3, fed
  events through `Runtime::post_input`.
- **`on_signal`** becomes the signal resident (§2.3), fed through `Runtime::post_signal`.
  Declaring it **takes the signal over**, like a POSIX handler: `post_signal` returns true
  and the host skips the default action, so the program decides what an interrupt means.
  With no `on_signal`, Ctrl-C closes the app. The terminal loop also force-quits on three
  Ctrl-Cs within two seconds, so a handler that never quits cannot trap the user. Signals
  sent to the process reach it too (`source: "os"`) when the host collects them —
  `FtxuiBackend::RunOptions` has the hooks, and `ari` wires SIGTERM, SIGHUP, SIGINT,
  SIGQUIT and SIGUSR1/2 to them.

  ```yaml
  on_signal: |
    (if (= (state-get "app.quit_armed") "1")
        (quit)                                   # a host verb: `ari` registers it
        (begin (state-set "app.quit_armed" "1")
               (state-set "app.status" "Ctrl-C again to quit")))
  ```
- **`keys:`** names the document's **rebindable** key actions
  ([`keys.h`](../inc/cvc/ariadne/keys.h)): each has a label, its default key or keys, and an
  `on:` (a program, or a host event). A binding's keys live in a state key, `keys.<id>`
  (seeded from the defaults), and `Runtime::post_input` runs its `on:` on a key down that
  matches one of them, in the next drain, as a button's runs. Key names compare whoever
  spells them (`canonical_key`: SDL's `Left Shift` and VTK's `Shift_L` are one key). An
  `on_key:` still sees every key, besides.

  ```yaml
  keys:
    next:
      label: Next item
      key: [Right, PageDown]
      on: (state-set "item" (+ (int (state-get "item")) 1))
  ```

  `AppRuntime` lists every binding, the document's and the host's
  (`set_host_keys`: `arigl` adds its camera's fly keys), in a **Keys** window it puts in the
  document: closed until `tree.ariadne_keys.open` is set (arigl's *Settings > Keys...*), a row
  per binding with its keys and a **Change** button. Change writes the binding's id to
  `tree.ariadne_keys.capture`; while the window is open the next key down becomes its key
  (Escape keeps the old one; the comma key is stored as `Comma`), and no handler sees that key.
  Closing the window drops a capture still waiting. In the terminal every key goes to the
  capture, Tab and Return included. A group with an `id`, like each of those rows, scopes its
  children's identities as a window does, so every row's Change is its own.

  Documents a host loads in turn share its prefix, so a new document's `keys:` defaults replace
  what an earlier document left at the same `keys.<id>`; a reload of the same file keeps the
  keys the user chose. A new document also starts its layout afresh
  (`Runtime::forget_layout`, `Backend::reset_layout`): each window's first frame takes its own
  `pos:` and `size:`, not the last document's window of the same id. An authored `pos:` is
  measured from the work area, below a main menu bar.

A small, complete document (mirroring `nav_compute.ari`). A widget is a map whose
**first key is its type** and whose value is its label (`slider_int: "N"`), with the
widget's fields as sibling keys; copy the shipped components (§6.5) for the exact keys
each type takes:

```yaml
meta: { name: hello, version: 0.1.0, min_libcvc: 3.4.0 }
menubar:
  - menu: File
    items:
      - menu_item: Quit
        on: quit                          # a bare name: a host Runtime::on handler
windows:
  - window: Controls
    id: controls
    children:
      - slider_int: "N"
        bind: demo.n
        lo: 1
        hi: 100
      - text: "N x 2"                     # renders "N x 2: <value>"
        bind: (* 2 (int (state-get "demo.n")))   # a computed bind, read lane
      - button: Bump
        on: (state-set "demo.n" (+ (int (state-get "demo.n")) 1))   # a program, action lane
init: (begin (state-set "demo.n" 10) (state-set "demo.frames" 0))
on_tick: (state-set "demo.frames" (+ (int (state-get "demo.frames")) 1))
```

### 5.4 `scene:` sources and custom node types

A `scene:` node's `type:` is a built-in (`geometry`, `volume`, `volren`, `volslice`,
`group`) or a custom type a host registered with `register_scene_node_type`
([`scene_realize.h`](../inc/cvc/gl/ariadne/scene_realize.h)). cvcGL bundles its custom
types behind one call, `register_cvcgl_extensions`
([`extensions.h`](../inc/cvc/gl/ariadne/extensions.h)), which both `ariadne_hello` and
`AriRuntime` (so pycvc hosts) make: `forest_trees`, `wave_sea`, `cloud_sky`, and — on a
libcvc built with `CVC_ENABLE_LIBIGL` — `mesh_lab` and `fe_lab`. Keys the built-in parser
does not know land in the node's `props`, which a custom realizer reads; most realizers
**seed** state keys from them (`forest.*` for `forest_trees`, the node's own subtree for
`mesh_lab` / `fe_lab`), so a document's prop is the starting value a bound component then
edits live. A document that cannot run without a custom type says so
in `customs:` (`- node: mesh_lab` with `required: true`), and a host that checks
(`verify_scene_customs`, as `ariadne_hello` does) refuses it up front on a build that
lacks the type.

**Primitives.** Besides `source: { plane: { size } }` and `source: { heightfield: {...} }`,
a geometry node can draw `source: { box: { size: [x, y, z] } }` (or `{ box: <edge> }`), a
box centred on the origin, and `source: { sphere: { radius } }` (or `{ sphere: <radius> }`).
A heightfield is the world's own `cvc::height_field`: the node draws its mesh (each cell split
south-west to north-east), a body on it collides with the same triangles, and a custom node
standing on it (`ground:` in `forest_trees`, `wave_sea`, `cloud_sky`) samples the triangle under
each point.

**Physics.** A scene-level `physics:` block declares a rigid-body world
(`cvc::physics::world`), and a geometry node's `body:` gives it a body there:

```yaml
scene:
  physics:                      # all optional; physics: {} takes the defaults
    gravity: [0, 0, -9.80665]
    fixed_dt: 0.0083333         # a whole number of the clock's quanta (1/120 s); 0 = one quantum
    backend: auto               # auto (Jolt when built) | jolt | null (bodies placed, never moved)
    materials: { ice: { friction: 0.05, restitution: 0.0 } }
  nodes:
    - node: terrain
      source: { heightfield: { size: 20, resolution: 41, layers: [{ kind: dome, amplitude: 1, radius: 6 }] } }
      body: static              # the motion alone; a heightfield is always static
    - node: crate
      source: { box: { size: [1, 1, 1] } }
      transform: { position: [2, -1.5, 4] }
      body:
        motion: dynamic         # dynamic (default) | kinematic | static
        shape: auto             # auto | box | sphere | hull | mesh | terrain
        mass: 10                # kg; 0 = density (default 1000 kg/m^3) x volume
        material: ice
        velocity: [0, 0, 0]     # world m/s at the start; angular_velocity in rad/s
        linear_damping: 0.05    # and angular_damping
```

- **Shapes.** `shape: auto` collides with what the node draws:
  - a heightfield as terrain;
  - a box or sphere primitive as itself;
  - a plane as a thin slab under it;
  - any other mesh as its convex hull (moving), or itself (static).

  `box` and `sphere` on another mesh fit its bounds, and the node's scale is baked into the shape.
- **Placement.** Each body starts at its node's realized world pose (a nested node's included).
- **Stepping.** `tick_scene` drives `app.world_clock()` as `clock: {}` would, then steps the world
  by the quanta due. It draws each moving body's node at the body's pose, between the last two
  steps, wherever the node sits in the hierarchy.
- **Terrain transforms.** A terrain follows its node's position but not its rotation or scale.
  Draw a ground you collide with unrotated and unscaled.
- **Reading physics.** `RealizedScene::physics` is the world, `RealizedScene::body(id)` a node's
  body, and `RealizedScene::height_fields` the heightfield nodes' fields.
- **Problems** (an unknown material, a body on a group, a fixed_dt between quanta) warn and never
  throw.

**Volume from a mesh.** `source: { sdf: { mesh: <uri>, dim: N, algorithm: v1 | v2 | igl } }`
computes an N³ signed distance field (negative inside) framed to the mesh, after the
node's `fit:`. `algorithm` picks the `cvc::sdf` implementation: `v2` (the default), `v1`,
or `igl` (exact AABB distance signed by the fast winding number, robust to holes); `igl`
needs a `CVC_ENABLE_LIBIGL` build and falls back to `v2` with a warning without one.

**`fit:`** stands a loaded mesh on the ground: `fit: { height: H, up: x | y | z, flip: true }`
turns the dataset's own up axis to +z (`up: y` is a +90° turn about x, `up: x` the cyclic
x→z), then with `flip` turns it upside down (a half turn about x, for a scan stored head
down), centres it in x and y, sits it on z = 0 and scales its largest extent to H.

**`type: mesh_lab`** ([`mesh_nodes.h`](../inc/cvc/gl/ariadne/mesh_nodes.h)) reads its
`source:`/`fit:`/`material:` like a `geometry` node and processes the mesh with
`cvc::mesh_ops`. Its keys live under the node's own state, `<node>.mesh_ops.*`:

| key | meaning | seeded from prop |
|---|---|---|
| `request` | `smooth`, `decimate`, `repair`, `orient`, `curvature`, `reset`, `load`, `reload` (§5.5); several separated by spaces run in order, as in `load smooth` | — |
| `busy`, `status`, `stats.vertices`, `stats.faces` | published | — |
| `stats.work_ms` | published: the wall time of the last run of commands, from the first to idle | — |
| `source.file` | the mesh `load` reads: a path or URI, a relative path resolved against the document's directory (`stanford.bunny` is built in) | `source: { file }` |
| `source.up`, `source.flip`, `source.height` | the `fit:` `load` applies: up axis `x`/`y`/`z`, upside down, height (0 = no fit) | `fit:` |
| `smooth.method` | `cotan` (implicit) / `uniform` / `taubin` | `smooth_method` |
| `smooth.iterations`, `smooth.lambda`, `smooth.fix_boundary` | `cvc::smooth_params` | `smooth_iterations`, `smooth_lambda`, `fix_boundary` |
| `decimate.target_faces` | triangle target for `cvc::simplify` | `target_faces` |
| `repair.weld`, `repair.remove_degenerate`, `repair.remove_unreferenced`, `repair.orient` | `cvc::repair_params` | `weld`, `remove_degenerate`, `remove_unreferenced`, `orient` |
| `color.mode` | `none`, `mean`, `gaussian`, `k1`, `k2`, `geodesic`, `modes` — followed live | `color_mode` |
| `color.colormap` | `viridis`, `magma`, `plasma`, `inferno`, `turbo`, `jet`, `parula`, `gray`: resets the field colours (below) to that ramp | `colormap` |
| `geodesic.source` | source vertex for `geodesic` (clamped to #V-1; -1 is the highest vertex) | `geodesic_source` |
| `modes.index` | the Laplace–Beltrami eigenfunction `modes` shows (`fem::laplacian_eigenmodes` on the surface; 1 is the first non-constant, up to 50) | `mode` |
| `color.min`, `color.max` | the colour range used: the field colours' explicit domain, else the values' (2-98% percentiles for curvature) | — |

**Field colours (both node types).** The values a node shows are coloured through a transfer
function at `<node>.field_colors` (a `cvc::tf::model` string, outside `mesh_ops.*` / `fe.*`), so
a `tf_editor` edits the ramp: mesh_tools and fe_controls each hold one (`show_opacity: false`,
`show_isocontours: false`). Only the colour ramp is used. The colormap key (`color.colormap`,
`view.colormap`) seeds it and resets it to that colormap's ramp whenever it changes, so a
document stepping through colormaps still works; an edit stands until the next change. An
explicit domain in the model is the range the values are coloured over (in fe_lab it wins over
`view.fixed_range`). Beside it the node publishes `field_colors.range` (the colours' span under
an `auto` domain) and `field_colors.histogram` (128 bins of the values over it; fe_lab's is the
whole solution in the field view, the shown elements' size or quality in the elements view): the
editor's axis and histogram. When an editor shows the contour spectrum or contour tree, the node
computes them on the values' own mesh (fe_lab's tetrahedra, mesh_lab's surface) in an overlay task
on the state_exec scheduler and publishes `field_colors.contour_spectrum` / `.contour_tree`; fe_lab's
elements view has neither (one value per element), which `field_colors.overlay_status` says. See
[`TRANSFER_FUNCTIONS.md`](TRANSFER_FUNCTIONS.md) §5-6.

`curvature` switches a non-curvature `color.mode` to `mean` (or recomputes the one shown).
A geodesic solver is cached per mesh version, so moving the source is two
back-substitutions; so are the eigenpairs (at least ten are computed), so stepping
`modes.index` through the first few reuses one eigensolve. `load` reads the file again only
when a `source.*` key changed since the last load; otherwise it goes back to the loaded mesh
(undoing a smooth or a decimate), and keeps those caches when that mesh is still the one
shown. `reload` always reads it. A failed load says why in `status` and keeps the mesh.

**`type: fe_lab`** reads a volume like a `volren` node (`source: { file }` or `{ sdf }`) and
meshes it with the in-tree LBIE mesher (`CVC_ENABLE_MESHER`): tetrahedra
(`cvc::tetrahedralize`, or `tetrahedralize_interval` for the shell between two isovalues) or
hexahedra (`cvc::hexahedralize`). It solves linear (P1) finite-element problems on the
tetrahedra with `cvc::fem`. It has two views:

- **field** (the default): the node shows the field on the boundary, translucent while its
  child `<id>_slice` shows the field on an axis-aligned cut. The boundary is the smooth surface
  the elements approximate (`view.boundary: input`): an `sdf` source's own mesh, or a volume's
  isosurface (`cvc::iso`), the field sampled onto it from the elements' boundary. With
  `view.boundary: mesh`, or for an interval mesh, it is the elements' own boundary.
- **elements**: the node shows the elements on one side of the cut plane (the slice axis and
  offset; all of them with the slice off). Each is kept whole and flat-coloured by its size
  (log2 of its edge length, one band per octree level) or its quality (a tetrahedron's minimum
  dihedral angle in degrees, a hexahedron's scaled Jacobian). The child `<id>_edges` outlines
  each exposed face, and `<id>_ghost` draws the whole outer surface at `view.ghost` opacity.

Keys under `<node>.fe.*`:

| key | meaning | seeded from prop |
|---|---|---|
| `request` | `load`, `reload`, `mesh`, `solve`, `stop` (`stop` interrupts a heat solve; any other job runs to the end and the status says so); several separated by spaces run in order | `auto_mesh: true` seeds `mesh` |
| `busy`, `status`, `progress` (0-100), `stats.vertices`, `field.min`, `field.max` | published | — |
| `stats.tets`, `stats.hexes` | published: the element count (the other is 0) | — |
| `stats.quality_min`, `stats.quality_mean` | published: minimum dihedral angle (tetrahedra, degrees) or scaled Jacobian (hexahedra) | — |
| `stats.work_ms` | published: the wall time of the last run of commands, from the first to idle | — |
| `stoppable` | published: 1 while a heat solve runs (the Stop button's `enabled_when`), else 0 | — |
| `last_mesh` | published: the region and bounds the last mesh job started with (`solid <isovalue>` or `interval <lo> <hi>`), `""` before the first | — |
| `source.type` | `sdf` (the signed distance field of a mesh) or `volume` (a volume file) | `source:` |
| `source.file` | the mesh or the volume: a path or URI, a relative path resolved against the document's directory (`stanford.bunny` is built in) | `source.sdf.mesh` / `source.file` |
| `source.dim` | `sdf`: the grid's N; `volume`: resampled to N³ (0 = as stored) | `source.sdf.dim` / `resample` |
| `source.algorithm` | `v1`, `v2` or `igl` for an `sdf` source | `source.sdf.algorithm` |
| `source.up`, `source.flip`, `source.height` | the `fit:` applied to the elements: up axis, upside down, height (0 = none) | `fit:` |
| `auto.mesh`, `auto.solve` | mesh after a load; solve after a mesh (or after a load that keeps the mesh) | `auto_mesh`, `auto_solve` |
| `mesh.region` | `solid` (one side of `mesh.isovalue`) or `interval` (the shell between `mesh.isovalue` and `mesh.isovalue2`, in either order, whatever `mesh.inside` says; tetrahedra only) | `region` (default `solid`); `interval: [a, b]` is shorthand for `region: interval`, `isovalue: a`, `isovalue2: b` (`interval: true`: the region alone), and an explicit `region` / `isovalue` / `isovalue2` beside it wins |
| `mesh.isovalue` | the bounding level, in the volume's units (an `sdf` source's are the mesh file's own) | `isovalue` |
| `mesh.isovalue2` | an interval's second bound | `isovalue2`; absent, a tenth of the value range from `isovalue` on the side `mesh.inside` fills (an SDF's inner skin), at most half way to that end of the range, else on the other side (for an `sdf` source, set when its field has been computed) |
| `mesh.inside` | `below` (an SDF: mesh where v < isovalue) or `above` (a density); a solid mesh's side | `inside` (default `below` for an `sdf` source, else `above`) |
| `mesh.elements` | `tet` or `hex` (hexahedra show in the elements view; a solve refuses them) | `elements` |
| `mesh.improve`, `mesh.improve_iterations` | `cvc::improvement_method` (0-5; 5 is optimization) and its passes | `improve`, `improve_iterations` |
| `problem` | `poisson` (-Δu = f, u = 0 on the boundary), `heat` (hot floor) or `modes` (free-vibration eigenmodes of the solid) | `problem` |
| `poisson.f` | the constant source | `poisson_f` |
| `heat.kappa`, `heat.dt`, `heat.steps` | backward-Euler heat; κ is relative: the diffusivity used is κ·extent² | `heat_kappa`, `heat_dt`, `heat_steps` |
| `modes.index` | the eigenmode `modes` solves for (1 is the first non-constant, up to 50) | `mode` |
| `view.mode` | `field` or `elements` | `view` |
| `view.boundary` | `input` (the smooth surface, when there is one) or `mesh` | `boundary` |
| `view.colormap`, `view.show_surface`, `view.show_slice` | display; `view.colormap` resets the field colours (above) to that ramp | `colormap`, `show_surface`, `show_slice` |
| `view.slice_axis`, `view.slice_offset` | 0/1/2 and -1..1 across the mesh bounds: the cut, in both views | `slice_axis`, `slice_offset` |
| `view.surface_opacity` | the boundary's opacity while a slice shows | `surface_opacity` |
| `view.color_by` | the elements' colour: `size` or `quality` | `color_by` |
| `view.cut_side` | the side of the cut the elements view keeps: 1 (+) or -1 (−) | `cut_side` |
| `view.ghost` | the outer surface's opacity in the elements view (0 = off) | `ghost` |
| `view.fixed_range`, `view.range_min`, `view.range_max` | colour over a fixed range instead of the data's (both views; an explicit field-colours domain wins) | `fixed_range`, `range_min`, `range_max` |

The heat problem holds the boundary vertices in the lowest 5% of the height at 1 and
insulates the rest; the body starts at 0. `auto.solve` solves after every mesh,
unless a request written during the mesh job is already waiting: that request runs instead
(a waiting re-mesh auto-solves in its turn). Eigenpairs are cached per mesh (at least four),
so stepping `modes.index` through the first few re-solves nothing.
Until the first mesh the node shows the volume's bounds as lines.

An interval mesh's boundary has two sheets, an outer and an inner one round the cavity, each
wound (and lit) out of the shell; Poisson holds u = 0 on both, and the heat problem's hot
floor is the lowest 5% of either (on a shell, the outer sheet's bottom cap). The interval is
inclusive — a voxel exactly on a bound is inside: such a bound is meshed half way to the next
value beyond it, so `[1, 3]` on integer labels meshes as `[0.5, 3.5]` — and a bound on or past an end of the volume's
range bounds nothing (`[0.5, 2]` on a 0/1 mask meshes `{v ≥ 0.5}`). An interval that
holds no voxel leaves the last mesh and says so in `status` ("error: no tets: the interval
[lo, hi] does not cut the volume (its values span [min, max])"). The node's
`<node>.transfer_function` holds the bounds as isocontours, so a `tf_editor` drives both (see
[TRANSFER_FUNCTIONS.md](TRANSFER_FUNCTIONS.md) §6, "Meshing an interval from two isocontours").

**Switching sources.** The node's `source:` and `fit:` seed the `source.*` keys. A volume file
is read when the scene is realized; for an `sdf` source only the mesh is, so the scene frames
its box at once, and the first tick computes the field on the lane (`status` says
`loading ...`; requests wait for it). A `load` request reads the keys again on the lane, but only
when one changed; then, with `auto.mesh`, it meshes when the source or a `mesh.*` key changed,
and otherwise solves (`auto.solve`). So a document can step one node through several datasets
and problems, re-reading, re-meshing and re-solving only what changed, as
[`fem_gallery.ari`](../examples/ariadne/documents/fem_gallery.ari) does. `reload` always reads
the source. `fit:` places an `sdf` source's elements after its SDF is taken in the mesh file's
own units, as it places a volume's: LBIE refines its octree by an error in the volume's value
units, so an SDF scaled to the fit height would refine a small model everywhere.

**Relative paths** in the `source.*` keys of both node types resolve against the directory of
the document that declared the node (`SceneNode::base_dir`); a path found only relative to the
working directory is read from there. A built-in node's `source: { file }` still resolves
against the working directory.

Both node types run their kernels on one `cvc::async_lane` per node — off the render
thread, or deferred onto it in a single-threaded wasm build — and apply results in their
per-frame tick. A job captures only its own hand-off struct, never the node's runtime
(which owns the lane), and teardown cancels it; tearing a scene down still waits for a
running tet-mesh or Poisson solve to finish.

### 5.5 Commands as a request key

A component cannot call a C++ object, but it can write a key the object reads. A
long-running command is therefore a **request key** the node owns: the button's `on:`
program writes the operation's name, and the node's tick consumes it (writes `""` back),
runs it, and publishes `busy` and `status`:

```yaml
- button: Smooth
  on: (state-set "graphics.root.children.mesh.mesh_ops.request" "smooth")
  disabled_when: (= (state-get "graphics.root.children.mesh.mesh_ops.busy") "1")
```

A request written while a job runs waits in the key until the job finishes (the latest
write wins), so a click is never lost and two jobs never overlap. Parameters stay
ordinary live keys the job copies when it starts. The FTXUI backend draws such a panel
read-only: its buttons do not fire yet
([`ftxui_backend.cpp`](../src/cvc/ariadne/ftxui_backend.cpp)), and it has no scene.

### 5.6 `args:` and `argv` — the document's command line

A runtime takes only its own general flags, before the document path. Everything after the
path belongs to the document, as with `python script.py args`:

```
ari run --fps 30 conf.ari --room standup --camera
```

The runtime publishes it under the document root before `init:` runs
([`doc_args.h`](../inc/cvc/ariadne/doc_args.h)). All of it is read-only:

- **`argv`**: the strings as given. Its data is a DSL list, `(state-data-get "argv")`; its
  value is the same strings joined by spaces, for display.
- **`args.<name>`**: one per argument declared in an `args:` block that was given or has a
  default. Its value is the canonical text, `(state-get "args.room")`; its data is the typed
  value (string, bool, int or float).
- **`args`**: the whole set as one DSL dict of name → typed value.

```yaml
args:
  room:   {type: string, default: lobby, help: Room to join}
  camera: {type: bool, help: Use the real camera}
  peers:  {type: int, default: 8}
  file:   {type: string, positional: true}
  verbose: bool                  # shorthand: just the type
init: |
  (state-set "status" (str-concat "joining " (state-get "args.room")))
```

**Parsing rules:**
- `--name value` and `--name=value` both work.
- A bool is `--flag` or `--no-flag` (`--flag=false` works too); a bare flag never consumes
  the next argument.
- Declared positionals are taken in order, and `--` ends the options.
- If an option is repeated, the last one wins.
- A negative number is a value, not an option.

The load fails on any of:
- an unknown option;
- a malformed value;
- a missing `required:` argument;
- an extra positional.

The runtime then prints the generated usage and exits 2. `--help` or `-h` prints that
usage and exits 0. A malformed `args:` block (an unknown type, a default that is not of
its type, a bool positional, a required positional after an optional one) fails the load
too, since nobody could pass arguments to it. Without an `args:` block nothing is checked,
`--help` included: the document gets `argv` and makes of it what it wants. `ari check`
validates the block but not the arguments, and declared arguments take their defaults.

**Who can see them.** Only the outermost document takes arguments. A `load:`-mounted
module's own `args:` is ignored with a warning. Programs are chrooted to their own
subtree (§7), so a module sees neither `argv` nor `args` unless its parent grants a
read-only hole (§7.4). A hole links one node:
- `link: {args: {to: args, mode: ro}}` hands the module the dict,
  `(get-attr (state-data-get "args") "room")`;
- `link: {room: {to: args.room, mode: ro}}` hands it one argument, `(state-get "room")`.

---

### 5.7 `requires:` and `has?` — what a document needs from its host

The same document can run under `arigl` (a GL window), `arigl --terminal`, `ari` (a terminal
with no GL at all), a browser page or a custom host. What each provides differs: `ari` has
no scene renderer and none of cvcGL's verbs, and only a kitty or Sixel terminal shows real
pixels. A document says what it needs in a top-level `requires:` list (roadmap §7.6, §16.3;
[`requires.h`](../inc/cvc/ariadne/requires.h)):

```yaml
requires:
  - gl.scene                              # fail the load unless a scene renderer is here
  - room-join                             # ... or unless programs can call (room-join ...)
  - {name: term.kitty, optional: true}    # only warn if this is missing
```

`AppRuntime::load` checks every entry before `init:` runs. If any required entry is
missing, the load fails with one error that names them all. A missing `optional: true`
entry warns. An entry is one of:

- a **capability token**, answered by the backend's `Capabilities` and the host:

  | token | holds when |
  |---|---|
  | `gl.scene` | a scene adapter realizes `scene:` (arigl, the cvcGL hosts) |
  | `gl.shaders` | GLSL runs (scene `shaders:`, `shader_canvas`) |
  | `gl.view` | a live render view can sit in a widget |
  | `ui.windows`, `ui.menubar`, `ui.color` | movable windows; a menu bar; coloured styling |
  | `input.mouse`, `input.keyboard` | pointer input; key and text input |
  | `raster.bitmap` | images show in real pixels (a GL window, or a kitty or Sixel terminal) |
  | `term`, `term.kitty`, `term.sixel` | a text terminal; one that speaks kitty graphics; one that draws Sixel |

- a **function** a program can call: a builtin, a stdlib function, a state intrinsic or a
  host verb (`Runtime::has_function`);
- a **widget type**, built in (`slider_int`, `button`, …) or registered
  (`register_widget_type`, a backend-native widget such as `stream_view`).

Whether or not `requires:` is there, the load also reads every program (`init:`, the
`on_*:` handlers and each widget's `on:` / `on_click:` / … programs). Any function a
program calls that the host does not provide gets a warning at load. Before, the program
stopped with "undefined symbol" the first time it ran. The scan skips special forms,
quoted data and names the program binds itself. It cannot see a name built at run time or
handed to `apply`, so it only warns; `requires:` is the authoritative list, and a name
listed there is not warned about again. `ari check doc.ari` shows both.

A document that can run without something asks for it at run time with `(has? "name")`,
which answers the same question for a token, a function or a widget type. It works in
programs and in reactive expressions:

```yaml
windows:
  - window: Scene
    visible_when: (has? "gl.scene")   # hidden in a plain terminal
    children:
      - button: Join call
        enabled_when: (has? "room-join")
        on: (room-join "lobby")
```

A host can ask the same thing in C++: `Runtime::has`, `has_function`, `program_names` and
`host_facts`.

### 5.8 Window size, placement and scrolling

A window needs neither `pos:` nor `size:`. Its size:

- A window with a `size:` (px or `"%"` of the view, per axis; `min:`/`max:` bound it) opens at
  that size, or at its persisted `tree.<id>.geometry` (§11.4) once it has one. The user may
  resize it, and that size is the one persisted.
- A window with no `size:` fits its content every frame, so it grows and shrinks with what it
  shows. It stops at the edge of the view, measured from where it stands, and never goes below an
  authored `min:` or past a `max:`. Its persisted geometry places it but does not size it.

Content that does not fit the window, whatever set its size, scrolls. A window gets a scroll bar
on each axis where it needs one, and never clips its content or shrinks to a stub.

Where it opens: at its persisted position, else its `pos:`, measured from the work area below a
main menu bar. A window with neither joins a column down the left edge of the view, in document
order, below the window before it and below any window that stands where it is. When the next one
would not fit below, it starts a new column to the right; a column never starts past the right
edge, so on a crowded view the last one overlaps instead. A window the user drags leaves the
column, and the windows after it close up. A window that opens later, such as one reopened from
the Windows menu (§5.9), takes its place in the column again. A new document
(`Runtime::forget_layout`) lays every window out afresh.

How each backend does it:

- **ImGui** gives a window with no size ImGui's always-auto-resize: it measures the window's
  widgets each frame, its labeled widgets at the width ImGui gives an auto-resizing window's items
  (16 font sizes), and a row-split pane as wide as its own widgets. A window with a size gets a
  horizontal scroll bar as well as ImGui's vertical one, and a long label shrinks its slider,
  input, combo or colour widget rather than run past the window's edge.
- **The terminal backends** lay a window with no size out at its content's size every frame,
  and cascade new windows instead of flowing them. A window that would run off the screen stops at
  the screen's edge, and a window whose content is larger than it is (or than its `size:`) gets a
  scroll bar. Its body scrolls to keep the focused widget in view; there is no wheel scrolling.

### 5.9 Closing and reopening windows: the Windows menu

A window's open state is `tree.<id>.open` (default open; `<id>` is its `id:`, else its title). A
window whose state is `0` is not drawn. A `closable: true` window has a close button that sets it
to `0`.

Every document gets a **Windows** menu to open such a window again, with no menu items to write:

- It holds a ticked item per window, bound to that window's `tree.<id>.open`. Unticking an item
  closes the window, closable or not; ticking it opens it.
- It lists only the windows no menu item of the document already toggles. A document with its own
  "Settings" or "View" menu bound to `tree.<id>.open` (`lsystem_forest.ari`,
  `conference_roster.ari`) keeps it, and the Windows menu lists only the windows that menu leaves
  out. When it leaves none out there is no Windows menu.
- A window made by `repeat:` is not listed, since there can be any number of them. The document
  opens those itself (`conference_roster.ari`'s camera windows open from the Users list).
- A host's menus (`AppRuntime::set_host_menus`) count as the document's: arigl's
  *Settings > Keys...* opens the Keys window, so arigl's Windows menu leaves it out. Under a host
  without one, the Windows menu is how the Keys window opens.
- It goes after the document's own menus (and a host's). When the menu bar already has a menu of
  the same title (`fem_gallery.ari`'s own Windows menu), the items go at the end of that menu,
  after a separator, instead of a second menu of that name. A document with no `menubar:` gets a
  menu bar holding just the Windows menu, but only when one of its windows is closable; otherwise
  nothing can be closed.

A document places or turns it off:

```yaml
windows_menu: false            # top level: no Windows menu (a string instead renames it)
menubar:
  - menu: View
    items:
      - windows_menu: Panels   # here, as a submenu titled Panels, instead of at the end
```

A bare `- windows_menu` entry places it under its default title. The menu is the core's (it calls
the backend's ordinary menu functions), so the ImGui menu bar, the FTXUI menu bar and the tvision
host's menu bar all show it, and its items toggle the same state the close buttons write. The open
state lives in the document's state tree like the geometry (§11.4): it holds for the run and
across a reload, but not across runs.

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

A unit may declare its parameters with defaults in an `args:` map beside its widget keys; the
include site's `args:` override them, and a `{token}` with neither stays literal. That is how a
shipped component binds a sensible node when included bare and any node when told:

```yaml
units:
  panel:
    args: { node: volume, title: Volume }   # declared defaults
    window: "{title}"
    id: "{node}_panel"
    children:
      - tf_editor: Transfer function
        bind: graphics.root.children.{node}.transfer_function
windows:
  - include: panel                # node=volume
  - include: panel
    args: { node: skull, title: Skull }
```

Any widget may carry an `id:` (not only windows): it names the instance a backend keys its
per-instance state on and that the core persists view state under (`tree.<id>.*`). An include
site may override the unit's `id:`, so two instances of one unit stay distinct.

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
  `$CVC_ARIADNE_PATH` → host-supplied extra dirs → `share/libcvc/ariadne` beside the file
  holding libcvc (`<lib>/..` or `<bin>/..`, so a relocated install such as a cvcpkg prefix
  finds its own library) → the compiled-in install datadir `<prefix>/share/libcvc/ariadne`
  → cwd (`register_cvc_uri_handler`). Use this for shipped components so a document does
  not hard-code install paths.

The `canonical` field is the identity used for cycle-guarding and for resolving
relative bases in nested composition.

### 6.5 The shipped component library

`src/cvc/ariadne/components/*.ari` (installed to `<prefix>/share/libcvc/ariadne/components`
and embedded in the wasm demos) are reusable windows a document imports by `cvc://` and
includes. Each binds state keys a C++ object owns, so it needs no host code;
[`INDEX.ari`](../src/cvc/ariadne/components/INDEX.ari) catalogs them:

| component | drives |
|---|---|
| `stage_lighting`, `scene_controls`, `camera_controls` | a `rig:` light, `shadows:`, the main camera |
| `display_toggles`, `sim_transport`, `raster_view` | `show.*` layer keys, `sim.*` + host verbs, a host-published image |
| `volren_controls`, `volslice_controls` | a `volren` / `volslice` node named `bunny_volume` |
| `forest_controls`, `sea_controls`, `sky_controls` | `forest_trees` / `wave_sea` / `cloud_sky` nodes |
| `mesh_tools` | a `mesh_lab` node named `mesh` (§5.4) |
| `fe_controls` | an `fe_lab` node named `domain` (§5.4) |
| `transfer_function`, `transfer_function_editor` | any node's `transfer_function` model (args `node`, `id`): the ColorTable2 editor with histogram / contour spectrum / contour tree layers — see [`TRANSFER_FUNCTIONS.md`](TRANSFER_FUNCTIONS.md) |

Most components that address one node use a fixed node id; copy the file and change the id to
drive another node. `transfer_function` instead takes the node as an include arg (§6.1).
`examples/ariadne/documents/mesh_lab.ari` uses the last two, and so does
`examples/ariadne/documents/fem_gallery.ari`, which steps one `fe_lab` and one `mesh_lab`
through the twenty items of [FEM_GALLERY.md](FEM_GALLERY.md). Run them from the repository
root, where the component path resolves:
`arigl run --component-path src/cvc/ariadne examples/ariadne/documents/fem_gallery.ari`.

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

**Grid sizes (§11.4).** A grid the user can resize keeps its sizes under its id, the way a
window keeps `tree.<id>.geometry`: a row split's pane heights in `tree.<id>.tracks`, a
resizable table's column widths in `tree.<id>.columns`. Both are comma-separated px, one
entry per track. The walk seeds the backend from the key before the grid opens (each
backend applies it once, so a live drag wins) and writes back what the backend reports after
it closes; the write is skipped when nothing changed. A `0` entry means "not set": that
track keeps its authored size. The backends differ in what they report:

- The FTXUI backend reports a track it holds at a size (authored px, seeded, or dragged) and
  `0` for one it lays out naturally, so a stretch pane or column is never pinned at the size
  one frame happened to give it. Seeding fixes a track at the persisted width.
- The ImGui backend reports every pane height and every column's drawn width
  (`WidthGiven`). Seeding gives a fixed column its width and a stretch column its weight,
  so stretch columns keep their persisted proportions at another table width.

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

Three runtimes run a document. They share their general flags
([`host_cli.h`](../inc/cvc/ariadne/host_cli.h)) and the rule that everything after the
document path is the document's (§5.6):

| Runtime | Links | Runs a document | Source |
|---|---|---|---|
| `ari` | libcvc only (FTXUI, optionally tvision) | in the terminal; a `scene:` is dropped | [`src/ari`](../src/ari/main.cpp) |
| `arigl` | libcvc + cvcGL (VTK, ImGui) | in a real window, or `--terminal` with its scene rendered behind the widgets | [`src/arigl`](../src/arigl/main.cpp) |
| `arisrv` | libcvc + gRPC + WebSocket | headless, as a server (the conference relay + WS↔gRPC bridge) | [`src/arisrv`](../src/arisrv/main.cpp) |

```
arigl run      [--terminal] [flags] doc.ari [document args...]
arigl snapshot --png out.png [flags] doc.ari [document args...]       # offscreen window frame
arigl snapshot --terminal [--ansi] [--png scene.png] doc.ari [...]    # a terminal frame as text
arigl check    [flags] doc.ari
```

`arigl` registers cvcGL's bundles before it loads a document
([`bundles.h`](../inc/cvc/gl/ariadne/bundles.h)): its node types and shader presets, the
stream and media verbs, and the `stream_view` widget (ImGui in a window, the terminal one
with `--terminal`). In both modes it routes OS signals to `on_signal:` and exits 128 + N
for one the document does not handle. A snapshot's frames run at `--fps` (20 by default),
so a live stream delivers into them; in a window the scene clock is fixed, so its frames
are reproducible. The cvcGL examples
(`ariadne_hello`, `ariadne_stream`, `ariadne_conference`, `ariadne_terminal`) still build;
`arigl` is meant to replace them once it does everything they do.

**A gallery of documents.** With `CVC_ARIGL_GALLERY` naming a catalog of demos (JSON,
[`src/arigl/gallery.h`](../src/arigl/gallery.h)), `arigl run`'s menu bar is its own and the same
for every document: **Demos**, **Document** (the document's own menus, as submenus) and
**Settings** (the scene's shadows, grid, origin axis, bounding boxes and extent labels, which
`arigl` without a gallery adds after the document's menus). A demo picked from Demos that is a
`.ari` document loads in place of the running one, in the same window; any other demo opens on
its own (its page in the browser, its program natively). That is how the examples' browser gallery runs every `.ari` demo in one app
(`<gallery>/arigl/?doc=<name>`). The catalogs come from `examples/web/demos.json`: the build tree
and the `libcvc-examples` install get one (`share/libcvc-examples/gallery.json`), and the browser
gallery writes one for what it deployed.

Two `AppRuntime` pieces make that possible for any host:

- `set_host_menus(menus)` puts the host's menus in every document it mounts, and gives a
  document without a menubar one. Their `menu_action` events go to the host's `on()` handlers.
  The document's own menus go where the list holds `AppRuntime::document_menus()`: given a
  label, inside one menu of that name, so the bar keeps its shape from document to document (the
  menu is greyed out for a document without menus); without one, in the bar as written, where a
  host menu named like one of them adds its items to it. With no `document_menus()`, the host's
  menus go first.
- `set_host_keys(keys)` adds the host's key bindings to every document's, and gives each
  document the Keys window (§5.3), even one with no keys of its own.
- `load()` clears the last document's scene first (`SceneAdapter::clear()`: its nodes, free
  lights, rigs, physics, and the background, shadows and chrome it set), whether it is a hot
  reload or a switch to another document. A document without `channels:` also drops the last
  one's channel policy.

### 9.1 Native (C++)

The reference host is
[`ariadne_hello.cpp`](../examples/ariadne/hosts/ariadne_hello.cpp). The shape is:

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

### 9.2 In a terminal (`ari`, `FtxuiBackend`)

`ari` ([`src/ari/main.cpp`](../src/ari/main.cpp)) runs a document in
the terminal with no GL, VTK, or ImGui:

```
ari run      [flags] doc.ari [document args...]   # take over the terminal
ari snapshot [flags] doc.ari [document args...]   # print a frame (--width W --height H --frames N --ansi)
ari check    [flags] doc.ari                      # load + warnings
```

Its flags (`--backend ftxui|tvision`, `--prefix P`, `--fps N`, `--no-mouse`,
`--component-path DIR`, and the snapshot ones) go **before** the document. They are the
general flags every runtime shares, parsed by
[`host_cli.h`](../inc/cvc/ariadne/host_cli.h). Whatever follows the document is the
document's (§5.6): `ari run doc.ari --help` prints the document's own usage. A general
flag written after the document goes to the document, and `ari` points that out.

Three of the general flags are for pictures. `--graphics auto|kitty|sixel|iterm|color|mono` sets
how images draw (see *Pixel graphics* in §4), and `--image NAME=FILE` publishes a picture
for the document's `image:` widgets with `src: NAME`
([`terminal_images.ari`](../examples/ariadne/terminal/terminal_images.ari) is an example).
With `--ansi` and `--graphics kitty`, `sixel` or `iterm`, `snapshot` also prints the images, after
clearing the screen, at the terminal's cell size (`--cell-size WxH` where it can't be asked).

`ari` is its own cvcpkg package (`cvcpkg install ari`, linux for now), built from
[`cvcpkg/recipes/ari`](../cvcpkg/recipes/ari/recipe.yaml) against the installed libcvc,
like `cvc-cli`. It is a thin host over `AppRuntime` with no `SceneAdapter`; it routes
`on: quit` and a `(quit)` verb to `request_close()`, and an OS signal the document does not
handle ends it with exit status 128 + the signal number. Embedding the same thing:

```cpp
cvc::ariadne::FtxuiBackend backend;          // outlives the AppRuntime
cvc::ariadne::AppRuntime rt(app, prefix);    // no SceneAdapter: a scene: is dropped
rt.set_backend(backend);
rt.on("quit", [&] { rt.request_close(); });
rt.load(path);
backend.run(rt);  // owns the terminal; drains every iteration; returns on quit / Ctrl-C
```

To route OS signals as well, install your handlers in `RunOptions::on_start` (it runs after
FTXUI has installed its own), restore them in `on_stop`, and return pending signal names
from `take_signal`. [`host_signals.h`](../inc/cvc/ariadne/host_signals.h) has the
`sigaction` traps `ari` uses (`install_signal_traps`, `restore_signal_traps`,
`take_trapped_signal`, `signal_number`). Signal handlers are process-wide, so libcvc never
installs them by itself: a host program opts in around its own loop.

Give the document its command line with `rt.set_document_args(args, "myhost run doc.ari")`
before `load()`. When the arguments don't match its `args:` block, `load()` throws
`document_args_error`, carrying the usage text and whether `--help` was asked for.

`run()` forwards keys and pointer events the UI did not use to the document's
`on_key` / `on_pointer`, so document-level input works here today.
[`terminal_tour.ari`](../examples/ariadne/terminal/terminal_tour.ari) exercises every
widget kind the backend draws.

**In the browser.** An Emscripten build of libcvc with `ftxui` and `yaml-cpp` in its wasm
deps prefix (`cvcpkg install ftxui yaml-cpp --platform wasm --arch wasm32 --link static`)
builds `ari` as `bin/ari.js` + `ari.wasm` + `ari.html`. The page puts an xterm.js terminal
on the module's stdin/stdout, which is all FTXUI's Emscripten screen uses. Serve `bin/`
over plain http and open `ari.html`; it is single-threaded and needs no COOP/COEP. A page
has no argv, so the build bakes in one document, `CVC_ARI_WASM_DOC` (default
`terminal_tour.ari`), with the component library at `/components`. The same `ari.js`
runs headless under node: `node ari.js snapshot --width 100 /terminal_tour.ari`. CI's `ari-wasm`
job does exactly that.

With `tvision` in the deps prefix too (`cvcpkg install tvision --platform wasm --arch wasm32
--link static`, cvc.4 on), the build also has `ari_tvision` (`ari_tvision.js` + `.wasm` +
`.html`, built by name): the same document on the tvision desktop, from the same page with
`--backend tvision` baked in. Upstream tvision needs ncurses and `select()`, neither of which a
page has, so cvcpkg's recipe patches in a browser console
(`recipes/tvision/emscripten-browser-console.patch` in cy-pca/cvcpkg): ANSI output on stdout
ending each frame with a NUL as FTXUI does, non-blocking stdin decoded by tvision's own
escape-sequence parser (keys, SGR mouse, bracketed paste), the size through the exported
`tvision_set_screen_size(cols, rows)`, which the page calls beside `_ftxui_on_resize`, and an
event wait that yields with `emscripten_sleep`. Both pages are in the gallery.

What differs from a real terminal:

- The loop yields with `emscripten_sleep`, so the build links `-sASYNCIFY`. A hidden
  tab throttles timers, which slows redraws and `on_tick`.
- Ctrl-C reaches the document's `on_signal:` as SIGINT, as in a terminal. Only keys
  the browser lets a page see get through: Ctrl-W / Ctrl-T / Ctrl-N belong to the
  browser. Ctrl-Z is ignored, because a page has no job control.
- Quitting (`on: quit`, `(quit)`, or Ctrl-C with no handler) ends the program, and
  the page offers to run it again.
- Startup pauses for about half a second. FTXUI waits up to 500 ms for the terminal
  to answer its capability queries, and xterm.js can only answer once the loop yields.

**With a 3D scene (cvcGL).** The terminal backends stay GL-free; cvcGL extends them.
[`cvc::gl::ariadne::TerminalScene`](../inc/cvc/gl/ariadne/TerminalScene.h) is a `SceneAdapter`
that realizes the document's `scene:` into an offscreen `SceneRenderer`, renders it every
frame at the size the backend draws the screen from (`FtxuiBackend::image_raster`: the
half-block grid, two pixels per cell, supersampled; or the cells' real pixels on a terminal
with Kitty or Sixel graphics), and hands the frame to the backend as its background
(`FtxuiBackend::set_background`), so widgets, menus and windows draw over the scene the way
ImGui draws over the GL canvas. With pixel graphics a drag maps cells to the frame's own
pixels, so a map pan keeps the grabbed point under the pointer. Passed as
`RunOptions::background_input`, its `handle_input` turns background drags into orbit,
middle-drags into pan and the wheel into zoom. `arigl --terminal` is the host
(`arigl run --terminal [--backend tvision] doc.ari`, or `arigl snapshot --terminal --ansi`
for one frame as text), as is the older example
[`ariadne_terminal`](../examples/ariadne/hosts/ariadne_terminal.cpp);
[`terminal_scene.ari`](../examples/ariadne/documents/terminal_scene.ari) is a small example. No display is needed: VTK picks X, EGL or OSMesa at run time
(`VTK_DEFAULT_OPENGL_WINDOW=vtkEGLRenderWindow` forces EGL).

In the browser the same host is `arigl_term` (`arigl_term.js` + `.wasm` + `.data` +
`arigl_term.html`, built by name in a wasm build with FTXUI): `arigl run --terminal` behind an
xterm.js page derived from `ari.html`. VTK renders the scene into a hidden `#canvas` in the page
(WebGL; the scene reaches the terminal only through stdout), and the page loads xterm.js's image
addon, so by default the scene is drawn as iTerm2 inline images at the cells' real pixels
(`?gfx=iterm`), or as half-blocks (`?gfx=color`, `?gfx=mono`). `?doc=documents/terminal_scene`
picks a document under `/ariadne`. The page writes the program's output once per animation
frame, the text and the images after it together. FTXUI repaints every cell each frame, which
deletes the image tiles under them, and the addon decodes each image asynchronously, so for a
moment the half-block underlay can show through: the picture flickers between its real pixels
and the half-block version, never to blank.

### 9.3 From Python (a full app via `AriRuntime`)

`AriRuntime` ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)) collapses all of the
above into one object, exposed through the `pycvc_gl` bindings. Python never sees SDL or
VTK directly — cvcGL's onscreen window is SDL's (VTK renders into it) and SDL is the internal
input backend:

```python
import pycvc, pycvc_gl

app  = pycvc.App()
view = pycvc_gl.SceneRenderer(...)      # owns the GL window
rt   = pycvc_gl.AriRuntime(view.scene(), view.camera(), view.overlay())

rt.load("demo.ari")
# Register a Python function as a DSL verb (host intrinsic on the app-wide scheduler):
rt.register_verb("greet", lambda name: f"hello {name}")
rt.register_async_verb("fetch_rows", slow_query)   # an async function: (fetch_rows) parks

while not rt.should_close():
    rt.frame(dt)     # processUIEvents → cam.update → drain → visibility → tick
    rt.render()      # NEVER call the underlying rt.render() directly; go through view
```

`register_verb` / `register_async_verb` install exactly one `register_action_intrinsics`
provider onto the app-wide scheduler, so the verbs are reachable from the document's
action lane. An async verb is an async function on the app's async host
([STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md) section 3): `(fetch_rows)` parks the
action while `slow_query` runs on the app's background task pool, then returns `str()` of its
result. If it raises, the call raises a `PythonError`, which a `try` can catch. The call is
charged to the action's caps (§2.2). `register_async_verb(name, done_channel, fn)` also posts
the result to `done_channel`, for documents that wait with `(msg-recv "done_channel")`. `AriRuntime` also exposes `step`, `run`, `on`, `post_key`, `post_pointer`,
and `reload_if_changed`.

### 9.4 A document set, installed or served

An app written as `.ari` documents ships them as a directory: `main.ari` and the files it
imports, relative to itself (`import: lib/feed.ari`, `load: ../shared/panel.ari`), plus any
`cvc://components/...` it takes from the shipped library (§6.5). The same directory runs in
both hosts without changes.

**Installed** (`share/<app>/`, from the app's cvcpkg recipe, which depends on the `arigl`
or `ari` package): `arigl run <prefix>/share/<app>/main.ari`, or in the terminal
`ari run <prefix>/share/<app>/main.ari` (FTXUI, or `--backend tvision`), from any working
directory. Both runtimes load through the same loader. Every relative reference resolves
against the directory of the file that makes it, and `cvc://` finds the component library
beside the libcvc in the same prefix (§6.4). A launcher only has to pass the path and its
flags. `ctest -R '(ari|arigl)_docset'` checks this with
[`src/cvc/tests/data/ariadne_docset`](../src/cvc/tests/data/ariadne_docset/app/main.ari).

**Served** (any site path, e.g. `/apps/<app>/`): put the `arigl-web` package's files (the
stock browser host) beside the documents and open `/apps/<app>/?src=main.ari`. The package's
`ari.html` runs the same documents in ari's terminal backend, in an xterm.js terminal:
`/apps/<app>/ari.html?src=main.ari`. Either runtime fetches the document and, before the app
starts, every file it names by a relative path
(`import:`, `load:`, `vinay:`, and the files those name), writing each at its URL path in
the browser's file system. Then it runs the local copy, so relative references resolve as
they do on disk ([`web_document.h`](../inc/cvc/ariadne/web_document.h),
[`doc_fetch.h`](../inc/cvc/ariadne/doc_fetch.h)). By default the
document must be on the page's own origin. The page's `ariadne.config.json` names the
document, any other origins it may come from, the host flags (the network flags `--net` and `--net-allow`, ...), and
the extra data files to fetch; both pages read the same file. The URL can only add document
arguments. A failed load shows what the runtime printed: an error page in arigl-web, the
terminal itself in `ari.html`. [CVCGL_WASM.md](CVCGL_WASM.md#arigl-web-the-stock-browser-host)
has the details, the sizes, and what wasm-mt needs.

A path the document hands a scene source or a data reader (`source: { file: ... }`) is
relative to the working directory, not to the document. The browser shell runs in the
document's directory. Natively the working directory is the caller's, so a launcher for
such a document `cd`s to its directory first.

### 9.5 From Python (DSL only, via `pycvc.Exec`)

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
- **`register_async_fn(name, callable)`** — an *async function*
  ([STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md) section 3): runs a **blocking**
  callable on the app's background task pool and **parks** the calling DSL program until it
  finishes, so the scheduler stays live. The callable runs on a pool thread with the GIL
  acquired for the call, so `urllib`/`requests` or a GIL-releasing numpy compute runs truly
  concurrently. It gets copies of its arguments. If it raises, the call raises a
  `PythonError` (its `python_type` names the Python exception), which a `try` can catch;
  uncaught, `Exec.run` re-raises the Python exception. `(name-async ...)` returns a future
  for `(await f [seconds])` or `(await-all (list f ...))`. The call is charged to the
  program's budget: its run time against `max_time`, its CPU time as steps.
  `register_async_fn(name, callable, True)` passes the call's `pycvc.async_call` first,
  before the arguments: `call.cancelled()` says whether to stop (the program was killed or
  ran out of budget), and `call.post(value)` sends progress to `"<name>.progress"`.
- **`register_async_coro(name, coro_fn)`** — registers an `async def`. Calling it
  schedules the coroutine on an asyncio loop the `Exec` owns and parks the DSL program.
  Unlike `register_async_fn`, the coroutine runs **on** the `run()` thread, cooperatively
  stepped one bounded slice per pump iteration, so it and the DSL march along together and
  its `await asyncio.sleep(...)` / `aiohttp` progress between DSL slices. A raise raises a
  `PythonError` at the call, as for `register_async_fn`.

> **The coroutine must yield.** The slice is cooperative — asyncio cannot preempt a
> running step — so a coroutine that does CPU-bound work without awaiting holds the
> `run()` thread and wedges the pump. Put blocking or CPU-bound work in
> `register_async_fn` (the pool), not in a coroutine. Passing an `async def` to
> `register_fn` / `register_async_fn` is rejected and steered to `register_async_coro`.

An async call parks at any depth, the whole program included: the call's frame waits on the
stack (the `await_call` phase) until its result is delivered.

```python
ex = pycvc.Exec(app)

async def poll(url):
    async with aiohttp.ClientSession() as s:
        async with s.get(url) as r:
            return await r.text()

ex.register_async_coro("poll", poll)
ex.run('(state-set "page" (poll "http://localhost:8080/status"))')
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
- They are async functions on the app's exec scheduler (`app::exec_async_host()`,
  [STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md) section 3): the request runs on the
  app's background task pool and the calling program parks at the call, at any depth, so the
  action lane never blocks the frame. The `-async` verbs return a future you `await`
  (optionally with a timeout in seconds) or pass to `await-all`.
- A failed send is still a reply with `ok` `#f` and `error` set, not an exception. A call is
  charged to its program's `max_time` while it waits, and a killed or reaped program's call is
  cancelled.

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
- **AriRuntime document-level `on_key` / `on_pointer` residents need a host to feed them.**
  The `post_key` / `post_pointer` forwarding is wired, and `arigl` feeds it from SDL3. On
  Windows, Linux (X11 and Wayland) and macOS cvcGL's onscreen window is SDL's: SDL makes the
  window and its OpenGL context, VTK renders into it, and the window's interactor turns SDL's
  events into VTK's (the camera and ImGui see them as before) and hands the same events to
  `cvc::gl::SdlInput::attach`, keys and pointer alike. `$CVCGL_SDL_WINDOW=0` keeps VTK's own
  window; the documents then get VTK's key events, renamed as SDL names them, and no pointer. In
  the browser VTK's canvas is still the window, and SDL listens on the page for the keys typed
  into it (a hidden canvas of SDL's own); SDL owning the canvas is the next step. arigl hands the
  document neither the keys typed into an ImGui field nor the clicks on a window. A host on
  AriRuntime's stock loop that does not attach an `SdlInput` gets no key or pointer residents
  ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h), [`SdlInput.h`](../inc/cvc/gl/SdlInput.h)).
- **OS signals reach `on_signal` only where a host routes them.** `ari` and `arigl` do, in
  both of `arigl`'s modes (§9). `AriRuntime::post_signal` is the seam for other GL hosts;
  the `ariadne_hello` example routes none, and an embedding host that skips the
  `RunOptions` hooks gets FTXUI's default for SIGTERM (restore the terminal, exit).
- **The libcvc terminal backends render no scene by themselves.** `ari` drops a `scene:`
  with a warning; cvcGL's `TerminalScene` (§9.2, `arigl --terminal`) is what draws one, so a
  scene needs a cvcGL build and a GL that works offscreen (X, EGL or OSMesa). On a Kitty terminal a
  scene that keeps moving sends a frame per change (up to `max_image_pixels`), and on a Sixel one
  every frame, so a remote session is better off with half-blocks (`--graphics color`).
- **`load:` `prefix:` / `reload:` and §12.2 sub-scene mounting are not yet parsed** at the
  loader; only sub-UI `load:` is landed ([`loader.cpp`](../src/cvc/ariadne/loader.cpp)).
- **No DSL-visible future from the pycvc async layer.** Both `register_async_fn` and
  `register_async_coro` self-park; there is no gather/join to launch *N* Python async
  calls concurrently from one process (the net verbs' `-async` futures do offer this).

---

## 12. Golden-image tests

`cvcgl_ari_golden` ([`cvcgl_ari_golden.cpp`](../src/cvcGL/test/cvcgl_ari_golden.cpp)) checks
what the ImGui/VTK backend actually draws (roadmap §16.6, phase 1). Every `.ari` in
[`src/cvcGL/test/golden/`](../src/cvcGL/test/golden/) is loaded through `AriRuntime`, rendered
offscreen at 960 x 720 with the overlay visible, and its frame is compared with the PNG of the
same name. The comparison code is
[`golden_util.h`](../src/cvcGL/test/golden_util.h), for reuse by other render tests.

A capture is made repeatable by:

- the UI font pinned to `golden/fonts/LiberationSans-Regular.ttf` (OFL; never a system font);
- UI scale 1, touch mode off, the show/hide button hidden, panels open;
- ImGui's frame delta pinned (`ImGuiOverlay::setFixedDeltaTime`);
- at least 4 frames at a fixed dt before the capture.

The test then renders one more frame and requires it to be identical, so a UI that is still
changing fails as unsettled rather than as a flaky mismatch.

To tell the UI from the rasterizer, each of the two frames is fingerprinted with what ImGui
handed GL: its draw lists (vertices, indices, commands, clip rects, texture ids) and every
texture's size and pixels. The settle line says whether the UI was the same both times, and
names the first differing pixel.

macOS CI's Apple Software Renderer sometimes draws the same UI two ways in a process's first
frames. It has only ever hit the first document. In 2026-10 it moved 8 pixels by 1 between
frames whose fingerprints were identical. So when the UI is identical and only pixels differ,
the later frame becomes the capture and the test renders another, up to 10 times, until two
frames agree exactly. The log says how many extra frames that took. A UI that changed fails at
once, as does a rasterizer that never settles.

**Comparing.** A pixel mismatches when a channel differs by more than 40. A document passes
while at most 0.002% of its pixels (13) mismatch. The goldens are made under Mesa llvmpipe, the
rasterizer of the Linux CI job (Xvfb + cvcpkg's `mesa`). These limits were measured:

- another Mesa version's llvmpipe and an NVIDIA GPU leave no pixel beyond 40;
- one step of a slider's value (64 to 65) leaves 144 pixels beyond 40, so it fails;
- a window moved down by one pixel leaves 12,167, so it fails.

A rasterizer that needs its own pictures gets platform goldens, `<name>.<variant>.png`. They are
used instead of `<name>.png` when present. The variant comes from `CVC_GOLDEN_VARIANT`, and
defaults to `macos` on Apple. Each comparison prints its largest channel difference and how many
pixels differ by more than 0 / 8 / 24 / 40, so a CI log shows a platform's spread even when it
passes.

Measured in CI (2026-10-20):

- Windows' llvmpipe (mesa-dist-win 26.1.7) is within 1 of the goldens everywhere.
- macOS CI renders with the Apple Software Renderer, which is deterministic but draws edges and
  shading a little differently. Against the llvmpipe goldens, windows, menus and grid had 3 to 6
  pixels beyond 40 (they pass), widgets had 40 and scene 467 (they fail).
- So `widgets.macos.png` and `scene.macos.png` are macOS goldens, and macOS compares those two
  against its own renderer.

To make or refresh a macOS golden, take `<name>.actual.png` from the failing macOS job's
`ari-golden-macOS-<lane>` artifact, look at it, and commit it as `<name>.macos.png`.

**On a failure** the test writes `<name>.actual.png` and `<name>.diff.png` to
`build/src/cvcGL/ari_golden/`. In the diff, mismatches are red (brighter = larger), differences
within the tolerance are dark blue, and the golden is a dim grey underneath. The log line gives
the count, the fraction and the bounding box of the mismatches. CI prints every document's
numbers in its "Ariadne golden-image numbers" step, pass or fail. When a job fails, CI uploads
that directory as the `ari-golden-<OS>-<lane>` artifact.

**Adding a test.**

1. Add `src/cvcGL/test/golden/<name>.ari`. Declare `meta.min_libcvc`. Fix every value with
   `init:` and place every window with `pos:`. A window with no `size:` fits its content
   (§5.8), which the pinned font keeps the same from run to run. The test requires a document to
   load without warnings.
2. If the picture needs input, such as an open menu or a tooltip, add a pointer script for
   `<name>` to `kCases` in the test: a move, press or release before a given frame, in ImGui
   pixels from the top left.
3. Write the golden under llvmpipe and look at it:

   ```bash
   CVC_UPDATE_GOLDEN=1 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe DISPLAY=:99 \
     build/bin/cvcgl_ari_golden src/cvcGL/test/golden build/src/cvcGL/ari_golden <name>
   ```

   Run it under cvcpkg's `xvfb` and `mesa`, as CI does (`cvcpkg install xvfb mesa --prefix P`,
   start `P/bin/Xvfb :99`, and put `P/lib` on `LD_LIBRARY_PATH` if VTK's libglvnd is not in
   `P`). Writing `<name>.png` from any other renderer is refused. With `CVC_GOLDEN_VARIANT` set,
   the run writes that platform's golden instead.
4. Commit the `.ari` and the `.png` together. After a deliberate rendering change, regenerate
   the affected goldens the same way, and review the new pictures in the PR. A document with a
   `<name>.macos.png` needs that one refreshed too, from the macOS CI artifact (above).

The test skips in a build without ImGui (`CVC_ENABLE_IMGUI`) or the YAML loader, without a GL
that rasterises, or where ImGui's GL backend cannot start on VTK's context (VTK's EGL fallback
with no X display). With `CVC_REQUIRE_RENDER=1`, as in the Linux CI job, each of those fails
instead. CI installs `imgui` from cvcpkg for this; before that, every CI build of cvcGL had the
ImGui overlay compiled as inert stubs.

**Gaps the goldens step around (2026-10-19):**

- `layout: {kind: horizontal}` is ignored on a window body, and the embedded schema rejects the
  `size: {hint, min, max}` map and `"50%"` strings that the loader accepts.

---

*Cross-references:* [`STATE_API.md`](STATE_API.md),
[`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md),
[`APP_API.md`](APP_API.md), and the normative spec
[`roadmap/CVCGL-UI-DSL-ROADMAP.md`](roadmap/CVCGL-UI-DSL-ROADMAP.md).
