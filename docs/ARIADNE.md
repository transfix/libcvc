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
13. [Testing a document: `quit`, `assert` and `ari test`](#13-testing-a-document-quit-assert-and-ari-test)

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
  ([§5.7](#57-requires-and-has--what-a-document-needs-from-its-host)). `json-parse` and
  `json-encode` are on it ([§10.4](#104-json-replies)); `str-concat` is not, so a computed
  label joins its parts with `string.join`.
- The special-form gate admits only `if / begin / let / while / for / lambda / defun /
  set / return / quote / yield / break / and / or` (`kAllowedForms`,
  [`ariadne.cpp:433`](../src/cvc/ariadne/ariadne.cpp)).

Nothing that writes state, schedules, watches, sends messages, or does I/O is
reachable. That includes the atomic updates `state-incr`, `state-cas` and `state-swap`: they
write, so they are not on the allowlist (actions may use them). Nor is `with-lock`
(docs/STATE_EXEC_LOCKS.md): a reactive slot that names it fails safe like any other
off-allowlist symbol. An off-allowlist symbol does not error loudly — it **fails safe**: the slot
degrades to hidden / disabled / empty / last-good value and the frame records a
one-shot warning (`take_reactive_warnings`,
[`ariadne.cpp:1682`](../src/cvc/ariadne/ariadne.cpp)). state_exec is always built (there is
no build option to drop it), so the engine is always present.

**Locks in the other lanes.** An action may use `with-lock`. One that names no `:timeout` gives up
after `ariadne.exec.lock_timeout` seconds (default 1.0, a tree setting beside the
`ariadne.limits.action_max_*` keys, read at each submission): a document never hangs on another's
lock. A resident's request has no such default: it waits as long as it takes. The `init:` lane
never waits for a lock: a busy request there is as `:timeout 0` (`TimeoutError`, or `:else` with
`:if-available`). An action's `(fork)` forks the action's process, and the child is reaped with
the document.

**Spawned children.** A child an action or a resident `spawn`s runs in the document's scope, as
its parent does: the intrinsics, the host's program-lane intrinsics and `(has? ...)`; its writes
and channel keys under its parent's prefix; the document's channel policy and resources; and its
parent's lock default, so an action's child gives up after `ariadne.exec.lock_timeout` too. It
keeps the document's owner, so it is reaped with the document. A child an `init:` script spawns
runs under the script's prefix with the same intrinsics and never waits for a lock either: it is
stepped on the host thread with the script, so a busy request is as `:timeout 0` there too.

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

Fire-once handlers (`on:` on a button, a menu item's program, or a bound widget's `on:`, run
after each committed write: a checkbox or menu tick, a combo or radio pick, a slider's drag end
or key step, a text field's Enter, a colour edit) and the resident
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
| an action, by an exception | `ari: on: action program failed: Class: message (at <where>) [<program>]` |
| a resident, by an exception | `ari: <lane> resident [<channel>] failed: Class: message (at <where>)` |
| an action, by a kill (its step or time cap, a syntax error found while reading it) | `ari: on: action program did not complete: <reason> [<program>]` |
| a resident, by a kill | `ari: <lane> resident [<channel>] stopped: <reason>` |

`<where>` is the innermost line of the exception's traceback. `<program>` is the action's text on
one line, cut at 80 characters, and `<lane>` is `on_tick:`, `on_key:`, `on_pointer:` or
`on_signal:`. An `init:` program that fails
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

A `progress:` widget is read-only: a bar showing the number at `bind` placed in [`lo`, `hi`]
(default 0..1; fe_controls uses `hi: 100` for `fe.progress`), with the text at `overlay` (a
state path or a computed expression) drawn over it, else the percent. ImGui draws a progress bar;
a backend with none gets a text value (`[########------------] 40%`).

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
    Left / Right step a slider or cycle a combo or radio; text fields edit in place; Esc
    reverts an edit or closes a popup; F10 opens the menu bar. An `autofocus:` widget has focus
    when it appears (§5.10).
  - **mouse** — click to focus or activate, pick a radio's option, drag a slider, drag a
    window by its title bar, click `[x]` to close it, click a menu title to open it.
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
windows_menu  tests  functions
```

`tests:` is read only by `ari test` (§13).

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

A choice of a few values is a `combo:` (a drop-down) or a `radio:` (every option shown, stacked;
`radio_row:` side by side). Both take `options:` (a list, or a computed expression), an
optional parallel `values:` (the key then stores `values[i]`, not the label), `def:` and an
optional `on:` run after each pick:

```yaml
      - radio_row: Samples
        bind: volren.supersample
        options: [Off, 2x, 4x]
        values: [0, 2, 4]
        on: (state-set "demo.dirty" 1)
```

A backend with no radio buttons draws a radio as a combo.

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

Loading, meshing and solving run as sliced tasks on the app's state_exec scheduler
([`cvc/core/sliced_task.h`](../inc/cvc/core/sliced_task.h)): a few milliseconds a frame on the
scheduler thread where there are no threads (single-threaded wasm), in chunks on the app's
background pool where there are. Each stage steps (the SDF of an `sdf` source, a `volume`
source's resample to `dim`, the LBIE meshing with its own resample to (2^n+1)^3, the boundary and
the smooth surface the field shows on, the solve's assembly, factorization and solves; the
elements view's cut-away and the field view's cut too), so the page stays live and `progress` /
`phase` move while a job runs. The mesh's boundary and a solve's boundary vertices step too
(`cvc::tet_boundary_stepper`). Only reading a file and a few whole-mesh passes (orienting the
tetrahedra, their quality, libigl's search trees, a solve's AMD ordering) are one step each. Something must pump the scheduler: every
Ariadne Runtime does. `CVC_SLICED_THREADING=cooperative` in the environment runs every such task
cooperatively on a native build, the way a single-threaded wasm page runs them.

Keys under `<node>.fe.*`:

| key | meaning | seeded from prop |
|---|---|---|
| `request` | `load`, `reload`, `mesh`, `solve`, `stop` (`stop` interrupts a heat solve; any other job runs to the end and the status says so); several separated by spaces run in order | `auto_mesh: true` seeds `mesh` |
| `busy`, `status`, `progress` (0-100), `stats.vertices`, `field.min`, `field.max` | published | — |
| `phase` | published while a job runs: its stage and percent (`x lines 40%`, `factoring 62%`), `""` when idle; fe_controls' progress bar shows it | — |
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
read-only hole (§7.4). A hole grants a node and everything under it:
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
  input, combo or colour widget rather than run past the window's edge. A
  `layout: {kind: horizontal}` row with no `col_widths` puts its cells side by side on one line,
  each at its natural width, as the terminal backends do. One with `col_widths` is an ImGui table
  with a column per entry. `borders` and `resizable` need that table, so a row using them needs
  `col_widths` as well; without it the table has one column.
- **The terminal backends** lay a window with no size out at its content's size every frame,
  and cascade new windows instead of flowing them. A window that would run off the screen stops at
  the screen's edge, and a window whose content is larger than it is (or than its `size:`) gets a
  scroll bar. Its body scrolls to keep the focused widget in view; there is no wheel scrolling.
  On the tvision desktop, tvision's frame holds the window, and the host refits that frame to the
  content in each axis the document did not size, until the user resizes the window with the
  frame's own corner; from then on it keeps the user's size.

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

### 5.10 Forms: `autofocus:` and `password:`

```yaml
windows:
  - window: Sign in
    visible_when: (= (state-get "auth.state") "out")
    children:
      - { input_text: User, bind: auth.user, autofocus: true }
      - { input_text: Password, bind: auth.pass, password: true }
      - { button: Sign in, on: auth.submit }
```

- **`autofocus: true`** on any widget gives keyboard focus to it, or to the first focusable
  widget inside it, when it appears: on the first frame it is drawn, and again each time it comes
  back after a frame without it (its window reopened, its `visible_when` true again, a `repeat:`
  instance added). It does not hold focus: the user tabs or clicks away as usual. With several in
  one frame, the last one drawn wins. The FTXUI and tvision backends move their focus to it; ImGui
  sets keyboard focus to the next widget (`SetKeyboardFocusHere`), so on an `input_text` it
  starts typing there. A field without it is focused only when the user moves to it.
- **`password: true`** on an `input_text` masks it on screen: ImGui draws its password field (a
  mask, and no copy or cut of the text), and the terminal backends show a `•` per character. It
  is masking only. The text is the bound state key's value like any field's, so it is as readable
  to scripts, `state://`, replication and the persisted tree as any other key; a document keeps a
  secret out of shared state by clearing the key once it has used it.

### 5.11 Pictures by URL: `image:` with a URI

An `image:` whose `src` (or the value at its `bind:`) is a URI shows that picture. Design and
the work still to come (video, audio, a card component):
[ARIADNE_MEDIA.md](roadmap/ARIADNE_MEDIA.md).

```yaml
windows:
  - window: Profile
    children:
      - image: avatar                        # the label: the caption a failure shows
        src: https://media.example/u/42.webp
        auth: example                        # optional: the host sends credential `example`
        size: [48, 48]                       # a box in px; one number is a width
        fit: cover                           # contain (default) | cover | fill
        shape: circle                        # rect (default) | circle
        status: profile.avatar_status        # optional: loading | ready | failed
      - type: group
        repeat: (state-get "feed.items")
        children:
          - image: ""
            bind: feed.items.{i}.picture     # a key, or a computed expression, naming a URL
            size: [480, 270]
```

- **What counts as a URI.** A value that starts with `http://`, `https://`, `file://`, `cvc://` or
  `state://`. Anything else is the name of an image the host published (`set_image`,
  `--image NAME=FILE`), as before. `src:` may be a computed expression too.
- **http(s)** goes through the host's network (§10.3). The host must turn it on and allow the
  picture's origin (`ari --net-allow https://media.example`). `auth: NAME` sends credential
  `NAME` as `Authorization: Bearer`, on the same terms as `http-get`'s `auth`: only to an origin
  the credential is bound to, never following a redirect. The token never reaches state, an
  error, a tooltip or a published name. Without the network, an http(s) picture fails ("the host
  has the network off").
- **`file://`** paths resolve against the document's directory (`file://img/logo.png`), or are
  absolute (`file:///usr/share/...`). `cvc://` and `state://` resolve as `import:` does (§6.4).
- **Formats:** PNG, JPEG, GIF (its first frame), BMP and, where libcvc has libwebp, WebP. Where it
  has ImageMagick, also TIFF, AVIF and HEIC. The format comes from the bytes, not the name or the
  Content-Type. A file over 64 MiB, or one wider or taller than 8192 px, is refused before it is
  decoded. A decoded picture is downscaled to 2048 px on its long side.
- **Off the frame.** The first frame that draws the widget starts the load, and the widget shows a
  neutral placeholder of the same box until the picture arrives, so nothing moves. The fetch, the
  decode and the fit run on the app's background pool. A failure shows a darker placeholder, says
  why in its tooltip, and is tried again after 30 s. Pictures are kept in memory per runtime, up to
  64 MiB, and dropped 30 s after the last frame that drew them. Two widgets that show the same URL
  share one fetch. `ari snapshot` and `arigl snapshot` wait for the pictures they draw, those
  whose URL a reply puts in state included: they draw again until no picture is still loading
  (at most 10 s, as for requests).
- **The box.** `size: [w, h]` is a box; `fit` says how the picture fills it: `contain` (all of it,
  transparent bars), `cover` (all of the box, the middle of the picture) or `fill` (stretched).
  `shape: circle` makes the outside of the inscribed circle transparent. `size: w` keeps the
  picture's own shape at width `w`, and its placeholder is 16:9 (1:1 for a circle) until it loads.
  The fitted picture is made at twice the box's size, where the source has the pixels, so it stays
  sharp on a HiDPI screen.
- **Every backend draws them**, since the core publishes the fitted picture as any other image:
  ImGui as a texture, and the terminal backends as Kitty, Sixel or iTerm pixels, or half-blocks
  (§4). Hosted tvision always draws half-blocks. In half-blocks, a transparent pixel leaves its
  half of the cell to the terminal's own background, so a round avatar has no corners on a light
  or a dark terminal. Kitty, Sixel and iTerm draw transparent pixels black.
- **The capability token `image.url`** (§5.7) holds in every host that does this, so a document
  for older hosts too gates its pictures with `(has? "image.url")` or
  `requires: [{name: image.url, optional: true}]` and draws its own placeholder elsewhere (an
  older host shows an unknown URI as text).
- **`status:`** names a state key the widget keeps `loading`, `ready` or `failed` in, with the
  reason under `<key>.error`. A document reads it to show something else when a picture is
  missing, and a test asserts on it.
- **In a browser build without threads** (ari.html, arigl-web) there is no pool to run them on, so
  each picture's fetch and decode run between two frames, one picture at a time. Until the
  pictures' requests use the non-blocking transport (#914), that frame waits for the request,
  as an `http-get` does there today.

### 5.12 Backgrounds: `background:` on a window or group

A window, an overlay or a group can have a picture behind its content: a profile's background,
a banner behind a name.

```yaml
windows:
  - window: Profile
    background: {src: https://media.example/bg/42.jpg, opacity: 0.4}
    children:
      - type: group
        id: banner
        size: [auto, 120]                    # a group's background area is at least this
        background:
          bind: profile.banner_url           # or src:; a URI (§5.11) or a published image name
          auth: example
          fit: cover                         # cover (default) | contain | fill
          opacity: 1.0                       # 0..1
          terminal: banner                   # behind (default) | banner | none
          banner_rows: 6
          status: profile.banner_status      # loading | ready | failed, as image:'s
        children:
          - image: avatar
            bind: profile.avatar_url
            size: [64, 64]
            shape: circle
          - text: ""
            bind: profile.name
```

`background: URL` is short for `background: {src: URL}`. The picture loads as an `image:`'s does
(§5.11), at its own shape, and each backend fits it to the area the content takes. Until it
has loaded, or when it failed, nothing is drawn behind the content.

- **ImGui** draws it behind the content: a window's fills the window's inner area, and a group's
  fills the group's area. That area is the full width available unless `size:` says otherwise,
  and at least `size:`'s height. `fit` decides which part of the picture shows, and `opacity` is
  how strongly it shows over the window's own background.
- **The terminal backends** (FTXUI, and tvision's window bodies):
  - **`behind`** draws half-blocks under the content, dimmed toward black by `opacity`. A cell
    the content writes text into keeps the picture's colour as its background, dimmed to at most
    40%, and the text is drawn in the terminal's own foreground unless the document coloured it.
    So text stays readable on any picture. Pixel protocols can't put text over pixels (Kitty's
    pictures are themselves cells, and Sixel and iTerm draw over the cells), so `behind` is
    half-blocks whatever `--graphics` says.
  - **`banner`** draws the picture as a row `banner_rows` tall above the content instead, at full
    strength, and in Kitty, Sixel or iTerm pixels where the terminal has them (from the second
    frame, once the row's width is known).
  - **`none`** draws nothing in a terminal.
  - A terminal with no colour (`--graphics mono`) draws no background.

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
(§7) and reaches the parent only through explicitly granted `link:` holes, and calls only the
functions its mount grants (§6.6).

### 6.4 The URI resolver

All external references (`import:`, `load:`, scene `source:`, `fetch`) go through one
resolver ([`inc/cvc/ariadne/uri.h`](../inc/cvc/ariadne/uri.h)):
`parse_uri` → `resolve(uri, base)` → `{ok, content, canonical, error}`. The grammar is
`<scheme>://<path>[?query]`; a bare path is `file`. Schemes:

- **`file://`** — built-in, app-free.
- **`state://…?value | ?data | ?children`** — reads from the `cvc::state` tree
  ([`uri_state.cpp`](../src/cvc/ariadne/uri_state.cpp)).
- **`http(s)://`** — **opt-in**: registered when the host turns the network on (`ari` /
  `arigl --net-allow ...`, §10.3), or by a host calling `register_http_uri_handler()` (or the
  cached variant). It is *not* registered by default — see the caveats (§11).
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

### 6.6 `functions:` — code a document shares with its programs and its modules

`defun` inside a program defines a function for that program alone, and the stdlib scope is
frozen. A document declares the functions all of its programs can call in a top-level
`functions:` block, and grants some of them to the modules it `load:`s.

```yaml
functions: |
  ; a scalar state_exec program of (defun name (params) body...) forms, nothing else
  (defun semver-part (v i) (int (nth (string.split v ".") i)))
  (defun semver-major (v) (semver-part v 0))
  (defun fetch-text (url)
    (set r (http-get url))
    (bytes.decode (get-attr r "body")))
init: (state-set "major" (str (semver-major "3.7.5")))
root:
  - button: Refresh
    on: (state-set "latest" (fetch-text "https://example.org/version"))
  - load: badge.ari
    as: badge
    link: {server: {to: args.server, mode: ro}}
    functions: [semver-major]          # a list of names, one name, or "*" for all of them
```

- **Declaring.** The block is split into its top-level forms. Each must be
  `(defun name (params) body...)`; anything else (a `set`, a bare atom, a form that does not
  parse) is skipped with a warning. When a name is defined twice, the later definition stands,
  with a warning.
- **Who can call them.** Every program in the document's scope: `init:`, every `on:` action,
  the residents (`on_tick:`, `on_key:`, `on_pointer:`, `on_signal:`), key bindings, `tests:`
  (§13), and the functions themselves. The read lane (`bind:`, `visible_when:` and other reactive
  expressions) has its own reduced environment and does not get them.
- **`import:`.** A library's `functions:` merge into the importer's, as its `units:` do. When two
  imports define the same name, the later one stands, with a warning. The document's own
  definition replaces an imported one silently. A library of functions with an `ari test`
  document that imports it is a unit test (§13).
- **Granting.** A `load:` entry's `functions:` names the functions its module may call. They come
  from the parent's scope: its own, its imports', and the ones its own mount granted, so a grant
  can pass down nested mounts. A granted function brings along the parent's functions it calls,
  transitively, and the module can call those too. A name the parent does not define is skipped
  with a warning. A module whose mount grants nothing gets none: it is isolated by default, as
  its state is.
- **A module's own.** A loaded fragment may have its own `functions:` and `import:`s. They are
  visible inside the module only, never to its parent. One that replaces a granted function of
  the same name warns.
- **Code, not authority.** A function runs in the program that calls it. In a module it is
  chrooted to the module's sub-prefix (§7.3), so `(state-set "out" ...)` writes
  `includes.<as>.out`, and it reaches the parent's state only through the module's `link:` holes.
  Granting a function never grants state.
- **How they run.** The functions are a prelude, their sources one per line, that the runtime
  runs ahead of each program in the scope (`Widget::functions`, a `FunctionScope` on the root and
  on every mount wrapper). Each program defines them in its own process at the start, at about one
  step per function against the action's step budget (§7.4 limits), so no closure or environment
  is shared between processes. A resident defines them once, before its loop.
- **Checked at load.** The free-head preflight (§5.7) knows each scope's functions. A program that
  calls a function its scope lacks, such as a module calling one its mount did not grant, gets
  the same "this host does not provide" warning as a missing host verb. The function bodies are
  checked too, as `functions:`.

`LoadResult::functions` lists the document's functions, and `LoadResult::root.functions` holds the
same as a `FunctionScope`. A mount's are on its wrapper widget.

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
resizable table's column widths in `tree.<id>.columns`. A grid's id is its `id:`, else its
`label:`, else one the walk makes from where the grid sits in the document (`~0_2_1`, the child
index at each level), so two unnamed grids in one window never share a backend table or a key.
That id moves when the document's structure above the grid changes, so give a grid whose sizes
should outlast such an edit an `id:`. Both are comma-separated px, one
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

At reconcile, `wire_holes` ([`ariadne.cpp`](../src/cvc/ariadne/ariadne.cpp)) plants a
transparent link at `resolve_bind(sub_prefix, name)` pointing to
`resolve_bind(parent_prefix, target)`, with writability set from the mode. Reads see
through the link; a read-only hole keeps writes local to the module. Holes are torn down
(`clearLink`) and re-planted every reconcile, so a re-mount can never inherit a stale grant.

A hole grants the target's whole subtree, not only the node it names. With
`link: {tree: search.res}`, the module's `bind: tree.packages.0.name`,
`(state-get "tree.packages.0.name")`, `(state-children "tree.packages")` and
`(state-data-get "tree.meta")` all reach the parent's `search.res.…`, including keys the
parent writes after the mount. Paths are resolved by `cvc::state::findThroughLinks` (reads)
and `childThroughLinks` (writes) ([`state.h`](../inc/cvc/state/state.h)), which follow a
transparent link on every hop of the path; Ariadne binds (`bind.h`) and the state_exec
`state-*` natives use them. The mode covers the whole subtree:

- `rw`: a write under the hole (`state-set`, `state-data-set`, `state-from-json`, the
  atomics, a widget commit) lands on the target's descendant, created when absent.
- `ro`: a write under the hole lands on the hole's own local child and never touches the
  target. Reads keep seeing the target, so the local copy is shadowed. The atomics
  (`state-cas`, `state-swap`, `state-incr`) refuse instead (`read_only_error`), since their
  write would not land on the node a read sees.

Default-deny still holds. Paths have no parent step, so a hole reaches only its target and
what is under it; a transparent link the parent keeps inside that subtree is followed (it is
part of what the parent granted), an opaque one is not. `state-delete`, the expiry natives
and `state-watch` still act on the physical path and do not reach through a hole. A fragment may declare
the holes it expects with `needs:`; an ungranted expectation warns and the fragment falls
back to a local (fail-safe) binding. A per-mount `init:` runs once at the sub-prefix
**after** the holes are wired, with the module's functions (§6.6). A module's functions come
only from its own `functions:` and the mount's `functions:` grant, never from its parent
implicitly.

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
ari test     [flags] doc.ari...                   # headless test run (§13)
```

Its flags (`--backend ftxui|tvision`, `--prefix P`, `--fps N`, `--no-mouse`,
`--component-path DIR`, and the snapshot ones) go **before** the document. They are the
general flags every runtime shares, parsed by
[`host_cli.h`](../inc/cvc/ariadne/host_cli.h). Whatever follows the document is the
document's (§5.6): `ari run doc.ari --help` prints the document's own usage. A general
flag written after the document goes to the document, and `ari` points that out.

Three of the general flags are for pictures. `--graphics auto|kitty|sixel|iterm|color|mono` sets
how images draw (see *Pixel graphics* in §4), and `--image NAME=FILE` publishes a picture
for the document's `image:` widgets with `src: NAME` (an `image:` can also name a URL itself,
§5.11)
([`terminal_images.ari`](../examples/ariadne/terminal/terminal_images.ari) is an example).
With `--ansi` and `--graphics kitty`, `sixel` or `iterm`, `snapshot` also prints the images, after
clearing the screen, at the terminal's cell size (`--cell-size WxH` where it can't be asked).

`ari` is its own cvcpkg package (`cvcpkg install ari`, linux for now), built from
[`cvcpkg/recipes/ari`](../cvcpkg/recipes/ari/recipe.yaml) against the installed libcvc,
like `cvc-cli`. It is a thin host over `AppRuntime` with no `SceneAdapter`. Its
`register_quit()` routes `on: quit` and the `(quit [code])` verb to `request_close(code)`, and
`run`, `snapshot` and `check` exit with `exit_code()` (§13). An OS signal the document does not
handle ends it with exit status 128 + the signal number. Embedding the same thing:

```cpp
cvc::ariadne::FtxuiBackend backend;          // outlives the AppRuntime
cvc::ariadne::AppRuntime rt(app, prefix);    // no SceneAdapter: a scene: is dropped
rt.set_backend(backend);
rt.register_quit();                          // on: quit and (quit [code])
rt.load(path);
backend.run(rt);  // owns the terminal; drains every iteration; returns on quit / Ctrl-C
return rt.exit_code();
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
  `url-decode` core builtins if you need them directly. `auth`, `capture_auth` and credential
  values (`(dict "credential" NAME)` in a form or headers dict) use the host's credentials
  (§10.3).
- The reply is a dict with every key present:

  | key | what |
  |---|---|
  | `ok` | the transport worked (a reply came back), whatever its status |
  | `status` | the HTTP status, `0` when no reply came |
  | `body` | the body, as bytes |
  | `url` | the URL the reply came from |
  | `headers` | the reply's header lines (`"Name: value"`), in order |
  | `header_map` | the same as a dict, lowercased name → value (repeats joined with `", "`) |
  | `content_type` | the `Content-Type` header, `""` when there is none |
  | `captured` | whether `capture_auth` kept a credential from this reply |
  | `error` | why the request failed, or why `capture_auth` kept nothing; `""` otherwise |

- They are async functions on the app's exec scheduler (`app::exec_async_host()`,
  [STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md) section 3): the request runs on the
  app's background task pool and the calling program parks at the call, at any depth, so the
  action lane never blocks the frame. The `-async` verbs return a future you `await`
  (optionally with a timeout in seconds) or pass to `await-all`.
- A failed send is still a reply with `ok` `#f` and `error` set, not an exception; so is a
  request the host's policy refuses. A killed or reaped program's call is cancelled.
- They are I/O calls (`async_options::io`, STATE_EXEC_EXCEPTIONS.md 3.8): a call is charged
  one step, not the request's wall time or the transport's CPU. One HTTPS request takes a
  100-300 ms round trip and about 16 ms of CPU (the TLS handshake and the CA store, some
  130,000 steps), while an action's default budget is 5000 steps and 0.1 s
  (`ariadne.limits.*`, §2), so charged like other calls no action could make one. The
  transport's own timeouts (10 s to connect, 30 s in all) and its 64 MiB reply cap bound a
  request instead.

### 10.3 The host's network: the allowlist and credentials

A document reaches the network only when its host turns it on. `ari` and `arigl` take the same
flags before the document path ([`host_cli.h`](../inc/cvc/ariadne/host_cli.h)), with the
same meaning in every mode: `ari` with either terminal backend (FTXUI, or `--backend tvision`),
`arigl` in a window, `arigl --terminal`, and both in a browser. So a launcher script gives a
document the network without any code of its own, and the same document runs in a terminal
(over SSH too: the credential store is then the remote user's) or a window:

```sh
ari run --net-allow https://tx.wtf --credential-store nicotina share/nicotina/main.ari
ari run --backend tvision --net-allow https://tx.wtf --credential-store nicotina share/nicotina/main.ari
arigl run --net-allow https://tx.wtf --credential-store nicotina share/nicotina/main.ari
```

The flags hold for `run`, `snapshot`, `check` and `info` alike. With the network on, `ari
snapshot` and `arigl snapshot` (in a window or with `--terminal`) keep running after their
frames until the document is idle, no request in flight and no program step waiting (a program
parsing one reply before it sends the next is not idle), for three steps running, at most 10 s.
So one frame shows what a document that signs in and fetches brought back, and CI can render it
against a local test server. `ari snapshot` prints the programs' warnings after the frame, as
`arigl snapshot` does.

| flag | what |
|---|---|
| `--net` | turn the network on (off by default); in a browser the page's own origin is allowed |
| `--net-allow ORIGIN` | allow requests to `ORIGIN`, `scheme://host[:port]` (repeatable; implies `--net`) |
| `--net-policy FILE` | read `allow` and `credential` lines from `FILE` (repeatable; implies `--net`) |
| `--credential-store S` | keep credentials across runs in `S` (implies `--net`; not in a browser) |
| `--credential-env N=V` | credential `N` is the token in environment variable `V`, for this run (repeatable; a policy `credential` line must bind `N`; not in a browser) |

Without them a document has no `http-get` or `http-request`, and an `http(s)://` URI does not
resolve. With them the host ([`net_host.h`](../inc/cvc/ariadne/net_host.h), `enable_net`):

- **checks every request against the allowlist.** An origin is a scheme, a host and a port.
  A request whose URL's origin is not on the list is not made: the reply has `ok` `#f` and an
  `error` naming the origin. Redirects are followed by the host, not the transport, and each
  hop is checked the same way; a hop to another origin drops `Authorization`, `Cookie` and
  `Proxy-Authorization`. URLs are parsed strictly: only `http` and `https`, no user info, no
  backslash, space or control byte, and an ASCII host (punycode for an international name).
  In a browser the browser follows redirects, so only the first URL is checked there; CORS
  and the browser's own rule of dropping `Authorization` on a cross-origin hop bound the
  rest.
- **registers the verbs and the `http(s)://` URI handler** with that allowlist, so
  `import:` of an https URL and `(fetch "https://...")` are checked too.
- **keeps a credential store** ([`credentials.h`](../inc/cvc/net/credentials.h)) of bearer
  tokens by name. A document never holds one:
  - `(http-get url (dict "auth" "txwtf"))` has the host add `Authorization: Bearer <token>`
    for credential `txwtf` as the request is sent, if the credential is bound to the URL's
    origin. Otherwise the request is not sent and the reply's `error` says
    `no credential 'txwtf' for <origin> (sign in first)`. Any copy of the token the reply
    echoes is replaced with `[redacted]`. `auth` and an `Authorization` header together are
    an error.
  - `(http-request "POST" url (dict "form" ... "capture_auth" (dict "name" "txwtf" "field"
    "token")))` has the host take the JSON string at `field` (object keys joined by `.`;
    `token` by default) from a 2xx reply, keep it as credential `txwtf`, and put
    `"[redacted]"` in its place in the body, every other byte as it was. `captured` says
    whether it was kept. `capture_auth` also takes a list of such dicts, one credential each,
    for a reply that carries several tokens (an access and a refresh token). Each is blanked
    wherever the reply carries it. A dict with `"optional" #t` names a field the reply may lack:
    a refresh reply need not rotate the refresh token.
  - A **credential value**, `(dict "credential" NAME)`, as a value in the `form` or `headers`
    dict, has the host put credential `NAME`'s token there as the request is sent: percent-encoded
    in a form, as is in a header (`X-Api-Key`, say). It goes only to an origin the credential is
    bound to, like `auth`, and echoes are blanked. A `query` refuses one, since URLs are logged.
    So a document refreshes its session (RFC 6749 section 6) without seeing either token:

    ```lisp
    (http-request "POST" "https://registry.example/v1/auth/token"
      (dict "form" (dict "grant_type" "refresh_token"
                         "refresh_token" (dict "credential" "svc.refresh"))
            "capture_auth" (list (dict "name" "svc" "field" "access_token")
                                 (dict "name" "svc.refresh" "field" "refresh_token"
                                       "optional" #t))))
    ```
  - A request with any of these options does not follow redirects.
  - `(net-auth-has? NAME [URL])` asks whether the host has the credential (for URL's origin),
    `(net-auth-forget NAME)` drops it (from the file too) and `(net-auth-clear)` drops them
    all. `(net-allowed? URL)` asks whether a request to URL may be made.
  - A credential is bound to the origin it was captured from, unless the policy binds its
    name to a list of origins (below). A token is a bearer token as RFC 6750 writes one,
    16 to 4096 characters, so it can never split a header line.
  - So a token reaches neither the document's variables nor the state tree (which a state
    transport replicates), and no error message or log line quotes one.
- **takes tokens it was handed with `--credential-env NAME=VAR`** (an API token, a CI secret):
  the token in environment variable `VAR` becomes credential `NAME` for this run. A policy
  `credential NAME ORIGIN...` line must bind it, so the launcher, not the document, decides where
  it may go. It is held in memory, ahead of any token the store file has for `NAME`, and never
  written; a capture of `NAME` replaces it. An unset or empty variable supplies nothing, so a
  launcher can always pass the flag; a value that is no bearer token stops the host (the error
  never quotes it). A browser has no environment, so it refuses the flag.
- **keeps credentials across runs with `--credential-store`.** A plain name (`nicotina`) is a
  file of that name in the user's config directory: `$XDG_CONFIG_HOME/cvc/credentials` or
  `~/.config/cvc/credentials`, `%LOCALAPPDATA%\cvc\credentials` on Windows. Anything else
  is a path. The file holds one `<name> <origin> <token>` line each, written owner-only (0600
  in a 0700 directory) and replaced whole on each change; the host refuses to read one that
  another user owns or that others can read. In a browser credentials live in memory only.

A policy file holds one directive a line, `#` starting a comment:

```text
allow https://tx.wtf
allow https://media.tx.wtf
credential txwtf https://tx.wtf https://api.tx.wtf   # where credential txwtf may go
```

Every origin a `credential` line names must also be allowed. A document says it needs the
network with the capability tokens `net` (the host turned it on), `net.auth` (it keeps
credentials) and `net.auth.persist` (it keeps them across runs), in `requires:` or with
`(has? "net")` (§5.7). `ari info` and `arigl info` show the allowlist and where credentials
are kept.

A host of its own turns the network on with `AppRuntime::enable_net(NetHostConfig)` or
`AriRuntime::enable_net`, before `load()`; the runtime turns it off again when it is
destroyed. `register_net_intrinsics(app)` alone still registers unchecked verbs with no
credentials, for an embedding program that vouches for its documents.

### 10.4 JSON replies

A REST reply's body is text. Three verbs turn it into widgets
([STATE_EXEC_DEVELOPER_GUIDE.md](STATE_EXEC_DEVELOPER_GUIDE.md) §2, "JSON", has the details):

- `(json-parse text)` reads JSON into values: an object is a dict in document order, an array
  a list, a number without a fraction or exponent an int, `null` nil. Malformed text raises a
  `ValueError` with `line` and `col`, which a `try` can catch.
- `(json-encode value [indent])` writes values as JSON, for a request body.
- `(state-set-tree path value)` writes a parsed value as the subtree at `path`: a dict's keys
  and a list's indices name child nodes, and a scalar is a node's value. A list's node holds
  its length, so `repeat:` can count it. It replaces the subtree, removing what the value
  doesn't name, and it checks the whole value before it writes anything.

```yaml
init: |
  (state-set-tree "feed" (json-parse (get-attr (http-get "https://example.org/feed") "body")))
windows:
  - window: Feed
    children:
      - type: group
        repeat: (state-get "feed.items")     # the list's length
        children:
          - text: ""
            bind: feed.items.{i}.text         # each element's field
```

The request needs the host to allow its origin ([§10.3](#103-the-hosts-network-the-allowlist-and-credentials),
`ari --net-allow https://example.org`).
[`examples/ariadne/documents/json_feed.ari`](../examples/ariadne/documents/json_feed.ari) does
this with a JSON string in place of the request, so it runs anywhere, `ari` included. Its last
post has accented letters, CJK and an emoji: in both terminal backends a wide glyph takes two
columns and the window's edge stays straight. `ari_run_json_feed_ftxui` and
`ari_run_json_feed_tvision` check that on a real terminal: they run `ari run` in a tmux pane
(Linux, where tmux is installed) and compare the window's rows. `ariadne_ftxui_test`'s
`JsonFeed*` and `WideGlyphs*` tests check `ari snapshot` and the tvision backend's hosted layout;
until they did, `ari snapshot`'s plain text put a space after each wide glyph. All
three are data jobs: a large reply is parsed and written a slice per step, so it doesn't stall
a frame. `json-parse` and `json-encode` are pure, so reactive expressions may use them too.

---

### 10.5 WebSockets

The `ws-*` verbs keep a WebSocket client open across ticks, for a server that pushes changes
(a live feed) instead of being polled. Natively they use libcurl's WebSocket support (built with
`ws` and `wss`, as the deps' libcurl is); in the browser build, the browser's own `WebSocket`
(link with `-lwebsocket.js`, which the `cvc` target adds for Emscripten executables). A host
without either has no `ws-*` verbs, so a document asks `(has? "ws-open")` and polls otherwise.

| verb | what |
|---|---|
| `(ws-open URL [OPTS])`, `ws-open-async` | connect to a `ws://` or `wss://` URL; reply `{ok handle url error}` |
| `(ws-send H TEXT)` | send a text message; `#t`, or an error when `H` is closed |
| `(ws-poll H)` | the next message already in, or `nil`; never waits |
| `(ws-recv H [SECONDS])`, `ws-recv-async` | the next message, waiting up to `SECONDS` (30); `nil` when none came |
| `(ws-open? H)` | whether `H` is still open |
| `(ws-close H [CODE])` | close `H` (code 1000) and forget it; `#t` if it was open, never an error |

- `H` is a handle string (`"ws:3"`), so a program can keep it in state and use it on later
  ticks. A host keeps at most 64 open.
- A message is a dict `{type data code}`: `type` is `"text"` (`data` a string, ready for
  `json-parse`), `"binary"` (`data` bytes), `"close"` (`code` the peer's close code, 1005 when it
  sent none and 1006 when the connection dropped; `data` its reason) or `"error"`. A close or
  an error is the last message; `ws-open?` is `#f` from then on. Fragmented messages arrive
  whole; one longer than 16 MiB is an error and closes the connection.
- `OPTS` takes `query`, `headers`, `auth` and credential values as the HTTP verbs do (§10.2,
  §10.3), plus `timeout`, the seconds to connect (15). A browser's `WebSocket` can't send
  request headers, so there `headers` and `auth` are refused; a server that wants a token takes
  it in a message instead.
- The host's policy (§10.3) checks the URL as its `http(s)` twin: `--net-allow
  https://example.org` allows `wss://example.org/...`.
- `ws-open` and `ws-recv` are async I/O calls like the HTTP verbs: the connect, or the wait,
  runs on the background pool and the program parks. `ws-poll` is the one to call from an
  `on_tick` that also does other work: it takes what has arrived and returns.

## 11. Caveats — what is not yet wired

State plainly so authors are not misled. Each item cites the code; the roadmap items
below (G17–G21 in the internal gap report) are tracked for reconciliation.

- **`http(s)://` resolves only where the host turned the network on** (§10.3). `ari` and
  `arigl` register the handler, checked against their allowlist, when given `--net` or its
  kin; otherwise `import:` / `load:` over http and `fetch("http://…")` fail with an
  unknown-scheme error.
- **The §13.9 HTTP cache is implemented and unit-tested but wired into nothing**
  (`register_cached_http_uri_handler`,
  [`uri_http_cache.cpp`](../src/cvc/ariadne/uri_http_cache.cpp)), and it does not check a
  `NetPolicy`. Even once installed, the primary `http-get*` verbs go straight to
  `cvc::net::send` and bypass it; only `fetch`/`fetch-async` route through the resolver where
  the cache lives.
- **The network verbs are not reachable from `pycvc.Exec`.** They live in the Ariadne
  Runtime action lane; `AriRuntime` and `AppRuntime` reach them through `enable_net`. A pycvc
  author registers a `register_async_fn` HTTP wrapper instead.
- **`fetch` / `source:` / `import:` cannot see `Content-Type`.** `UriResult` is
  `{ok, content, canonical, error}` ([`uri.h`](../inc/cvc/ariadne/uri.h)); the cache
  stores the content type but the resolver path drops it. The `http-get` / `http-request`
  verbs give `content_type` and `header_map` (§10.2).
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

- The embedded schema rejects the `size: {hint, min, max}` map and `"50%"` strings that the
  loader accepts. (A `layout: {kind: horizontal}` row with no `col_widths` drew as one column in
  ImGui until 2026-10-30; it now draws its cells side by side, §5.8.)

---

*Cross-references:* [`STATE_API.md`](STATE_API.md),
[`STATE_EXEC_DEVELOPER_GUIDE.md`](STATE_EXEC_DEVELOPER_GUIDE.md),
[`APP_API.md`](APP_API.md), and the normative spec
[`roadmap/CVCGL-UI-DSL-ROADMAP.md`](roadmap/CVCGL-UI-DSL-ROADMAP.md).

---

## 13. Testing a document: `quit`, `assert` and `ari test`

Three pieces let a document test itself, and a CI job run those tests with no terminal.

**`(quit [code])`.** A host verb every stock host registers (`AppRuntime::register_quit()`:
`ari`, `arigl` in a window and with `--terminal`). It asks the host to close, with `code` (an
integer 0..255, default 0) as its exit status. `ari run`, `snapshot` and `check` and `arigl`'s
commands exit with that code. The first request's code stands, a later `(quit)` does not change
it, and the program that called it keeps running to its end. A code that is not an integer is a
`TypeError`, and one outside 0..255 a `ValueError`; neither closes anything. A menu item's
`on: quit` is the same request with code 0. `(quit 3)` in `init:` makes `ari check` exit 3,
since `check` runs `init:`. `snapshot` stops at the frame in which the document quit.

**`(assert cond [message [data]])`.** A state_exec builtin, in every lane that has the builtins
(not the read lane's reduced environment). It returns nil when `cond` is truthy and otherwise
raises an `AssertionError` (a subclass of `Exception`, not of `RuntimeError`) with the message
`assertion failed` or `assertion failed: <message>`, and `data` as its data. `try` catches it
like any exception. Uncaught, its report ends with the `(assert ...)` form that failed, from the
exception's traceback:

```
AssertionError: assertion failed: one and one (at (assert (= (+ 1 1) 3) "one and one"))
```

**The `tests:` block.** A map of test name to program, run by `ari test` and ignored by every
other host:

```yaml
meta: {min_libcvc: 3.7.5}
init: |
  (state-set "query" "zlib")
root:
  - text: (state-get "query")
tests:
  reads_init: |
    (assert (= (state-get "query") "zlib") "init: ran first")
  fetches: |                            # a test may wait: (await), (sleep), a request
    (set r (http-get "http://127.0.0.1:8080/ping"))
    (assert (= (get-attr r "status") 200) (get-attr r "error"))
```

Each test runs as an `on:` action program would, chrooted to the document's prefix, with the
document's host verbs, functions (§6.6) and limits. The tests run one at a time, in document order, starting
after `init:` and the first frame, and each starts once the one before has ended. A test fails
when it raises an exception it doesn't catch, or is killed (a step or time cap, a syntax error).
The rest still run. The loader's free-head check (§5.7) covers test programs too.

**`ari test [flags] doc.ari...`.** Runs each document in turn, headless: a fresh app and state
tree per document, laid out each frame by the FTXUI backend at `--width` x `--height` (100 x 30)
as `snapshot` does, but never drawn. There is no input. A document runs until the first of:

- it calls `(quit code)` (an `on: quit` cannot fire with no input);
- its `tests:` have all ended;
- `--frames N` frames have run (default: no limit);
- `--timeout S` seconds have passed (default 30), which is a failure.

A document with no `tests:` therefore ends with `(quit code)`, from `init:`, `on_tick:` or a
program it starts, or runs a fixed number of frames with `--frames`.

A document **fails** when:

- its load fails, or warns (an unknown widget, no `meta.min_libcvc`, a failed `init:`, a free
  head the host lacks, ...);
- a program raises an exception nothing catches (an `assert` included) or is killed: an `on:`
  action, a resident (`on_tick:`, ...), or a read-lane expression that fails. The run stops at
  the first such warning;
- a test fails;
- it quits with a non-zero code, or quits before all its tests have ended;
- it times out.

The network is as `snapshot`'s (§10.3): off unless `--net`, `--net-allow`, `--net-policy` or
`--credential-store` turns it on. A test that makes a request waits for its reply as any program
does. A document run for `--frames N` with the network on waits after its frames, as `snapshot`
does, until its requests have come back (at most 10 s), and a warning in that time fails it too.
`--prefix`, `--component-path` and `--image` work as for `snapshot`. A document has no command
line of its own under `ari test`: an `args:` block's arguments take their defaults. Flags go
before the first document.

The output is one line per document, then what failed, one line each, and a summary:

```
ok   tests/semver.ari (3 tests, 0.02 s)
FAIL tests/search.ari (4 tests, 0.11 s)
     test 'bad' failed: AssertionError: assertion failed: one and one (at (assert (= (+ 1 1) 3) "one and one"))
     test 'raises' failed: RuntimeError: boom (at (error "boom"))
FAIL tests/tick.ari (0.01 s)
     on_tick: resident [ari#2:ui.docs.ari#tick] failed: AssertionError: assertion failed (at (assert (= 1 2)))
ari test: 3 documents, 2 failed
```

Each failure line names its program: `test '<name>'`, `init:`, `on_tick:` (or another
resident's lane), or for an `on:` action its text in brackets at the end (§2.4). `--tap` prints
TAP version 13 instead: the plan, `ok N - doc # detail` or `not ok N - doc # detail`, and each
failure as a `# ` line, with no summary. A document's own `(print ...)` output goes to stdout
between the lines. `ari test` exits 0 when every document passed, 1 when any failed, and 2 on a
usage error.

Interactive behaviour, keys and both terminal backends are tested by driving `ari run` in tmux,
as `src/ari/tests/run_in_tmux_test.sh` (transfix/libcvc#883) does. Scripted input for `snapshot` and `test`, and file
descriptors as state channels, are on the agenda (transfix/libcvc#894).

