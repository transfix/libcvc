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

What steps each kind of program is fixed in code; no setting or flag changes it:

- the read lane: a long-lived stackless evaluator (`ReactiveEngine`) during the render walk, never
  a scheduler (§2.1);
- actions, residents and the children they spawn: the app-wide `async_scheduler`,
  `app.exec_scheduler()`, shared by every document the app hosts and pumped by `Runtime::drain()`
  (§2.2);
- `init:`: a private synchronous scheduler, run to completion on the loading thread (at most 10
  million steps and 5 s), always `round_robin`, locking on the app's lock manager without ever
  waiting ("Locks in the other lanes" below).

The developer guide's "Which Scheduler Runs a Program"
([STATE_EXEC_DEVELOPER_GUIDE.md](STATE_EXEC_DEVELOPER_GUIDE.md#which-scheduler-runs-a-program))
has the same table for every host, with the settings that apply and when they are read.

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
- `(time-now)`, the wall clock in seconds since the Unix epoch (a float), is on it too, so a
  computed text can say how long ago a timestamp was. It reads the clock each frame it is drawn.
  `(time-elapsed)` is different: it is the program's own budget clock, not the time of day.

  ```yaml
  - text: ""
    bind: (let ((s (- (time-now) (float (state-get "post.created_at")))))
            (if (< s 3600) "just now" (string.join (list (str (int (/ s 3600))) "h ago") "")))
  ```
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
locks on the app's lock manager (`app::lock_manager()`), the one the residents and actions use, so
an init script's request meets their holds: its scheduler's lock home attaches when the script
starts and detaches when it ends. The lane never waits for a lock (docs/STATE_EXEC_LOCKS.md 5.5,
LW-9): the host thread that runs the script also steps the residents, so a holder could not
release meanwhile, and a busy request there is as `:timeout 0` (`TimeoutError`, or `:else` with
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
| per-frame aggregate | `kFrameBudgetSeconds` | `30 ms` |

The frame budget resets at the top of each frame, so *N* widgets cannot jointly stall a
frame. Once the aggregate is spent, the rest are skipped for that frame only:

- Each skipped expression keeps the value it had last frame: a `visible_when`, an `enabled_when`,
  a computed `bind:` or text, a `tooltip`, combo `options` or a `repeat:` count.
- One that never ran yet takes its fail-safe value instead: hidden, disabled, empty or 0.
- A skip isn't the document's fault, so it is never a reactive warning (`take_reactive_warnings`,
  the web pages' `ariadneWarning`). A real 10-post feed decoding pictures in wasm can spend the
  budget on its first frames.
- `Runtime::reactive_skip_count()` counts the skips, for a host that wants to log them.

**The read lane keeps its results.** An expression that ran is kept with what it read: the state
paths its readers named (`state-get`, `state-exists`, `state-children`, `state-data-get`,
`state-json`) and the `(has? …)` answers it got. The next frame reuses it without running it,
until one of these happens:

- something is written to a path it read, or to an ancestor or a descendant of one (so a
  `state-set-tree` that replaces a subtree counts);
- a name it asked `has?` about answers otherwise (each name is asked once a frame).

What it builds on:

- The writes are seen through the state tree's own change signal, from any thread: an action, a
  host, a network reply.
- A write that leaves a value as it was signals nothing, so it changes nothing.
- A frame that wrote nothing runs no expression, and one that wrote a post's like count runs the
  expressions that read it.
- Results are kept per scope and per expression text, so each `{i}` row of a `repeat:` is its
  own.
- An expression that read `(time-now)`, an expiry (`state-is-expired`, `state-has-expiry`) or a
  node through a link is never kept: it runs every frame, as before.
- A kept result isn't run, so the frame budget never skips it. Only what runs counts against the
  30 ms.
- A result no frame used for 300 frames is dropped.
- `Runtime::reactive_eval_count()` counts the expressions that ran, for a host or a test.

> These are the as-built numbers. Roadmap §7.4 still cites an older `CAP=50000` / 2 ms;
> trust the code.

### 2.2 The action / drain lane

Fire-once handlers (`on:` on a button, a menu item's program, or a bound widget's `on:`, run
after each committed write: a checkbox or menu tick, a combo or radio pick, a slider's drag end
or key step, a text field's Enter, a colour edit) and the resident
handlers (`on_tick` / `on_key` / `on_pointer`) run in the action lane, on the
**app-wide scheduler** (`app.exec_scheduler()`), **off** the render walk, inside
`Runtime::drain()` ([`ariadne.cpp:1642`](../src/cvc/ariadne/ariadne.cpp)).

That scheduler's policy is the app tree's `state_exec.policy` (`round_robin`, `priority` or
`priority_rr`), read once, when `app.exec_scheduler()` is first built, and fixed from then on; an
unknown value is logged and `round_robin` used. Ariadne sets none, so its default stays
`round_robin`: a program's `set-priority` is stored and reported but changes no pick (developer
guide, "Priorities").

`ari` and `arigl` build this scheduler while they set up, before any document loads, and have no
option that sets `state_exec.*`. A document cannot set those keys either, since its programs'
writes are chrooted under its own prefix. So under `ari` the policy is `round_robin`, and the
app's lock manager is `single` unless pycvc is loaded in the same process (developer guide,
"Single and shared mode"). A host that embeds Ariadne sets them on the app tree before its first
`AppRuntime::register_quit`, `register_verb` or `enable_net`, each of which builds the scheduler.

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

Residents have no total budget (they run for the life of the document), but each *step* is
time-capped, so that one runaway step cannot hang the host's drain. The cap is the tree setting
`ariadne.limits.resident_max_step_time`, in seconds (`0` = no cap). It is read as each resident
starts, and a missing or unparsable value keeps the default. The default is `0.05 s`, or `1 s` in a
wasm build. The deadline is checked inside long builtin loops (`collect`, a generator, a big
`state-data-get`), not by a plain body. It is a `se::time_budget`: its first half is charged on the
steady clock, and its second half on the thread's CPU clock, with each gap between two checks
counted up to a sixteenth of that half. So a native step that the OS deschedules must still run for
half the cap itself before it is killed. Single-threaded wasm has no thread CPU clock: there,
`thread_cpu_seconds` is the steady clock, so on a loaded machine a step that checks often (a big
`collect`, decoding a large reply) is charged for its waits too, up to a sixteenth of the half per
check. That is why its default is higher. On teardown, residents are reaped as a group by their owner tag.

**A resident that stops is restarted.** A killed one (its step cap, or a syntax error read inside
it) or a failed one (an exception it didn't catch) is reported once per message (§2.4). Then it
is started again on a later drain, with a fresh loop; what it wrote to state stays:

- the first time, at once;
- while it keeps stopping within 100 drains of starting, after 1, 2, 4 … up to 64 drains;
- after 8 such stops in a row, never again, with the warning `ari: <lane> resident [<channel>]
  stopped 8 times in a row soon after starting: not restarted again`. Setting its program again,
  which a reload does, starts it afresh.

This holds for every lane: `on_tick`, each module's `on_tick` (whether or not the mount is
drawn), `on_key`, `on_pointer` and `on_signal`. A dead input handler would leave the document
deaf to keys. A dead signal handler would swallow every Ctrl-C, since declaring one takes the
signal over (§5.3). A signal that arrives while its handler waits out a restart delay starts it
at once. A key or pointer event that arrives in that wait is dropped, as it is for a document
with no handler.

A `load:`ed module's own `on_tick:` is a tick resident too, one per mount (§6.3): chrooted to the
mount's sub-prefix, with the mount's functions (§6.6), on the channel
`<owner>#tick@<sub-prefix>`, and woken with the document's each drain. It starts the first frame
the mount is drawn, right after the module's `init:`, so a module in a tab that is not open starts
when the tab opens. From then on it runs whether or not the mount is drawn, until a reconcile
drops the mount (§7.4) or the document goes. A module has no input lanes of its own (§6.3).

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
one line, cut at 80 characters, and `<lane>` is `on_tick:`, `on_key:`, `on_pointer:`,
`on_signal:`, or `on_tick: of mount '<sub-prefix>'` for a module's tick. An `init:` program that fails
reports `failed: Class: message (at <where>)` to its caller.

**Where a host shows them, while the document runs.** Each warning reaches the host as it is
produced, not only when the document exits. `AppRuntime::set_warning_listener` hears each one at the
drain that collects it, and `take_warnings()` still returns them all for a summary at exit. The
stock hosts hand them to a `LiveWarnings` ([`host_warnings.h`](../inc/cvc/ariadne/host_warnings.h)),
which sends each one to:

| Where | When |
|---|---|
| the bottom of the screen, wrapped, for 8 s, with how many there have been so far | `ari run` and `arigl run --terminal` with the FTXUI backend, or the status line on the tvision desktop |
| a log file, one line each, with the time: `2026-10-30 18:31:02 ari: warning: on_tick: resident [...] stopped: ...` | `--log FILE`, or the environment variable `ARIADNE_LOG=FILE`. Appended, and flushed per line. |
| stderr, as `ari: warning: ...` | when stderr is not the terminal: `ari run doc.ari 2>warnings.txt`, a pipe, or the browser page. On a terminal, a full-screen UI owns it, so they wait for the exit. |
| the browser page: `Module.ariadneWarning(text)` ([`web_document.h`](../inc/cvc/ariadne/web_document.h)) | ari.html shows the latest in its header. arigl-web shows it in the canvas's corner for 8 s. Both keep them in `window.ariadneWeb.warnings`. |

When `ari run` exits it lists every warning of the run on stderr. When they already went to stderr
as they came, it prints only how many there were. arigl's window prints each one to stderr at the
frame it comes, as it always has. A message is reported once per run (`warn_once`). So a resident
that stops the same way each time shows one `stopped:` warning, and then the give-up warning if it
gets that far. `snapshot`, `check` and `test` print their warnings when they finish, as before.

To watch the warnings of a running document from another terminal:

```sh
ari run --log /tmp/ari.log app.ari     # or: ARIADNE_LOG=/tmp/ari.log ari run app.ari
tail -f /tmp/ari.log                   # in another terminal
```

**Keeping a resident running.** An exception a resident doesn't catch ends its process. The
runtime restarts it (§2.3), but the activation that raised is lost, and so is any request it had
in flight. One that raises every time is given up after 8 quick restarts. Catch inside the body to
keep it running:

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

A `plot:` widget is read-only too: a line graph of the comma-separated numbers at `bind` (a state
path or a computed expression), oldest first, between `lo` and `hi`. Leave them out (or give
`lo` >= `hi`) and the range is fitted to the numbers. The text at `overlay` is its caption, and
`height` its height in pixels (default 60). The host keeps the series (the conference's Avatar
window plots the history `avatar_panel` publishes). ImGui draws a line plot that reads out the
value under the mouse; a backend with none gets a text sparkline of the last 32 numbers with the
newest and the range (`[ _.-=+*#] 12.3 (1..30)`).

A `spinner:` widget says something is under way when there is no amount to show (a request, a
search). The host turns it by its own frame clock (`Runtime::frame_clock`), not by state, so it keeps
moving while the document's programs are parked: a program waiting on `http-get` still has a
turning spinner beside it, in every backend and in the single-threaded browser pages (§10.2).
Show it while the work runs with `visible_when`:

```yaml
- spinner: Signing in...
  visible_when: (= (state-get "ui.busy") "1")
- spinner:                                # the label from state, or a computed expression
  bind: (string.join (list "Loading" (state-get "feed.name")) " ")
- spinner: Working
  glyphs: ascii                           # | / - \ rather than braille dots
```

- The label is the widget's key, or `bind:` (a state path, or a computed expression, read-lane
  rules: `string.join`, not `str-concat`) when it has one; a bare `- spinner` has none.
- Disabled (`enabled_when` falsy, or `disabled_when` truthy), it stands still, greyed where the
  backend greys.
- The terminal backends (FTXUI, tvision) draw ten braille dots a second (`⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏`), then
  the label. `glyphs: ascii` turns `| / - \` instead, as they do by themselves on the Linux console
  and on a `dumb` or `vt*` terminal, which have no braille (`Backend::unicode_glyphs`). ImGui draws
  a turning arc, a text line high, one turn a second, so it needs no glyph from the font (the browser
  build's is ASCII).
- A snapshot reads frame `f` at `f / fps` (`ari snapshot`: `--fps`, default 20; arigl's fixed
  clock: 1/30 s a frame), so `--frames 3` shows the spinner one step on from `--frames 1`.

A backend advertises `Capabilities{windows, menubar, mouse, keyboard, color, glsl,
view_embed, owns_loop, terminal, bitmap, kitty, sixel, item_groups, clipboard, fullscreen,
fullscreen_view}`
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
    Its `size:` is a width (`size: 320`, the height following each frame's aspect) or a box
    (`size: [320, 240]`). In a box each frame is fitted inside it with its shape kept, and
    ImGui fills the spare space with black bars, so a tile stays one size whatever a camera
    sends: a portrait phone shows whole, not squashed. A camera captured with `jpeg:#t`
    keeps its own aspect within the capture's `w` x `h`, a budget turned to the camera's
    orientation (a portrait phone at 320x240 sends 240x320).

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
  zoom boxes, a pull-down menu bar, a status line, and its own keys (F10 for the menu bar, F6
  for the next window, Alt-X to quit, Ctrl-F5 to move or resize the window from the keyboard:
  the arrows move it, Shift and the arrows resize it, Return keeps it, Escape puts it back).
  The windows' *contents* are the FTXUI backend's: a `TvisionBackend` is an
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
[`loader.cpp:100`](../src/cvc/ariadne/loader.cpp)) are:

```
meta  menubar  windows  overlays  root (alias: children)  scene  customs  requires
init  on_tick  on_key  on_pointer  on_signal  keys  units  import  channels  lint  args
windows_menu  tests  functions  data  assets  scenes
```

`tests:` is read only by `ari test` (§13). `assets:` and `scenes:` are §5.18's shared meshes and
scenes of their own.

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

  `AppRuntime` lists every binding, the document's and the host's (`set_host_keys`), in a
  **Controls** window it puts in the document: closed until `tree.ariadne_keys.open` is set
  (arigl's *Settings > Controls...*), a row per binding with its keys and a **Change** button.
  Every key a host acts on is such a binding, in the state tree beside what reads it. arigl's:

  | binding | state key (under the scene prefix) | default |
  |---|---|---|
  | the camera's move keys, Orbit / fly, Free the mouse, Reset the view | `viewers.main.camera.keys.<action>` | W A S D (the arrows too), Space, Left Ctrl, Left Shift, Tab, Escape, H (Home too) |
  | the avatar's walk keys (conference mode) | `viewers.main.avatar.keys.<action>` | W or Up, S or Down, A or Left, D or Right |

  A host's pointer and camera settings (`set_host_mouse`, a `MouseControls`) go above the keys,
  bound where its camera reads them: arigl's are `viewers.main.camera.settings.mouse_look`
  (*Drag to look* or *Mouse look*), `primary_drag_steers` (in a room: *Drag the floor to face
  it*), `mouse_sensitivity`, `invert_pitch` and `invert_yaw`, and *Camera distance*, the
  camera's `orbit.distance` (the wheel changes it too). Under mouse look a click on the scene
  captures the pointer and the mouse turns the view with no button held, until *Free the mouse*
  (Escape; in a browser, Escape always ends its pointer lock). In a browser the page takes its
  pointer lock inside that press itself (the cursor goes, and the view keeps turning past the
  screen's edge), asking for the mouse's raw motion where the browser has it: a press on a
  window is the UI's, judged by where the press lands against the windows the overlay drew
  last frame (`CameraController::setUiRegions`), so a quick move onto a window and a click
  there never takes the pointer. Fly (Tab) is a spectator camera: the mouse turns the view with
  the pointer captured, W A S D move along it, Space and Left Ctrl up and down, Left Shift
  faster, at speeds per second of wall time (arigl's camera moves by the time between frames,
  not its nominal 1 / 120 s step). In a browser the toggle key's own keydown takes the pointer
  lock, where the browser grants it; until it holds, a drag turns the view and a bare move of the
  cursor does not, and after Escape a click on the scene takes it again. In Orbit (and Map) the
  same keys and the arrows slide the view across the ground, the same share of the view a second
  at any zoom, and Space / Left Ctrl zoom out / in (`settings.orbit_keys`; off in a room, where
  they walk your avatar). H or Home resets the view to the one the scene opened with, framing
  what the scene draws now (`CameraController::resetView`; a write to the camera's `home` key
  does it from a button). The wheel's zoom glides
  to where its notches put it (`settings.zoom_tau`, 0.15 s; 0 zooms at once on each notch). In drag to look, a click on the floor turns your avatar to
  face it and a drag on the floor keeps it facing the point under the pointer, a mouse's or a
  finger's (`CameraController::setPrimaryDragSteers`); the right button, or two fingers, turns
  the view. In conference mode under mouse look the avatar faces where the camera aims,
  standing or walking, and walks that way. Either way it turns every frame on your screen, and
  the relay hears each turn (at most 20 intents a second, a key at once) and shows it to
  everyone else. The camera looks past the avatar's
  right shoulder at a point 4 m ahead of it (at most a third of the camera's distance), so the
  crosshair marks where you aim, downrange. In either look
  the ground is the camera's floor (`orbit.floor.*`): looking up slides the eye in along its line
  of sight rather than under the ground, and an eye that comes inside the avatar hides it.
  `$CVC_ARIGL_LOOK=mouse` (the browser page's `?look=mouse`) starts in mouse look. A touch
  screen keeps dragging. In a browser arigl draws a frame per display refresh (it yields to the
  page until the next animation frame; `$CVC_ARIGL_YIELD=timeout`, the page's `?yield=timeout`,
  yields on a 0 ms timer instead, which judders on a fast screen).

  A row's Change writes the binding's id to
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
`AriRuntime` (so pycvc hosts) make: `forest_trees`, `wave_sea`, `cloud_sky`,
`generated_world` (terrain_lab's world, [`world_nodes.h`](../inc/cvc/gl/ariadne/world_nodes.h)),
and — on a libcvc built with `CVC_ENABLE_LIBIGL` — `mesh_lab` and `fe_lab`. Keys the built-in parser
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

**Textures.** `material: { texture: <uri> }` drapes an image over a geometry node through its
mesh's UVs. UV (0, 0) is the image's bottom-left corner. A `source: { data }` city's terrain wears
its bundle's `satellite.png` this way, north up (§5.17).

**Volumes and shadows.** A volume node (`volume`, `volren`, `volslice`) casts nothing into the
scene's shadow maps (`shadows:`): its picture is no surface the maps can hold. A `volren` node's
raycaster has
shadows of its own instead: with `volren: { shadows: true }` (or
`{ shadows: { enabled: true, strength: 0.8 } }`) its `lights:` cast through the volume onto itself.
The Volumetric shadows box in `volren_controls` turns them on and off. `volren_bunny.ari` and
`transfer_function.ari` have them on.

`material: { uv: { rotate, flip_u, flip_v, scale, offset } }` orients the image in texture space,
for an image that sits otherwise. The steps apply in this order:
1. `flip_u` and `flip_v` mirror it.
2. `rotate` turns it 0, 90, 180 or 270 degrees counter-clockwise on the surface.
3. `scale` and `offset` (a number, or `[u, v]`) map the result: `uv * scale + offset`.

A textured node casts its shadow by its shape. Its image's alpha still cuts a cut-out's shadow,
but its colours do not.

**Clouds.** `cloud_sky` draws a deck of drifting, slowly changing L-system cumulus over its
`ground:` node's extent (followed live): the clouds nearest the camera as volumes (`near:`), the
rest as one coarse volume drawn by a `VolSliceNode` (the `volslice` renderer): planes facing the
camera, cut to the deck and blended back to front, spaced wider with distance (at most 64), with the
clouds' own lighting as its sample shader (docs/VOLSLICE_API.md, "Distance slicing").
The near volumes go to the clouds nearest the camera that it can see (the deck's pattern repeats
out to `far_reach`, so past the ground's edge they are the copies beside the camera), and they draw
after the far ones, farthest first, so a nearer cloud always covers a farther one. The weather
evolves on the world clock: a cellular automaton over the deck's cells (`cvc::world::cloud_layer`)
grows clouds in patches and dissipates them, always holding the `coverage` share of the cells. A
cloud fades in or out over `fade` seconds (20), and its volume, far slices and shade fade together.
Orographic coupling reads the ground's heights. The cells over a
windward flank (the slope along `wind_az`, the heading the wind blows toward, 90 = +x) gather cloud,
and the cells in the lee lose theirs, so a cloud forms as the wind brings it to a mountain and dies
past the crest. `orographic` (default 1; 0 turns the coupling off) sets how strongly. Their shade falls on the ground and on the nodes in `receivers:` (and their
children, LOD rungs included) through those meshes' shaders: it dims the sun's light and leaves
their textures and colours alone. The clouds are lit by the same sun: each volume's colour is the
sunlight left after its own cloud's depth toward the sun, and each far cloud's after the whole
deck's, so the tops are white and the undersides and down-sun flanks grey. Far clouds also shade one
another; the near volumes shade only themselves. `light_k` sets how strongly (optical depth per
metre of opacity; by default twice the volumes' own, about 0.3 on terrain_lab's deck; 0 lights them
evenly). Its other keys: `extent` (a cloud's half-width), `base` / `top`,
`coverage`, `spacing`, `seed`, `variants`, `sun_az` / `sun_el` (the key light's, which the shade
is cast along and the clouds are lit from), `shade_k` / `shade_floor`, `horizon` (how far past the ground the volumes draw
clouds), `far_reach` (how far the far clouds reach toward the horizon; out of the camera's bounds),
`evolve` (the weather's speed: 1 is a step every 5 s, a cloud lasting some 8 minutes; 0 holds the
clouds) and `shadow` (0 starts the shade off). Live keys under the node's root (`sky.` unless
`keys:`): `drift`, `morph`, `evolve`, `orographic` and `shadow`, all on `sky_controls.ari`.

**Generated terrain.** `source: { terrain: { generator, size, resolution, seed, amp, file,
preset } }` draws a `cvc::world` terrain generator's ground as such a heightfield: `fbm` (the
analytic relief the terrain lab stands on; the default), `dem` (an elevation `.npy` at `file`, a URI
or a path relative to the document, stretched to `amp`), or one a host registered
(`cvc::world::register_terrain_generator`). Sea level is 0 m: every generator lays its ground with
0 m as the shoreline, its floor `world_sea_share` (34/90) of the relief below it (the island preset
stands its dome in the sea), so `water_level` raises or lowers the sea from there and a lower or
higher relief never floods or drains the world. The size defaults to 1200 and the resolution to one
vertex per 5 m (at most 2049). A `heightfield:` source takes the same `generator:` keys, its
layers adding on top. The `generated_world` node type uses the same generators (its `terrain:`,
`city:` and `terrain_file:` props; `world_controls.ari` switches them live). Its first world is
made while the document loads. An edit to a `world.*` key then waits (`world.pending` is 1) until
Regenerate (`world.request: regenerate`) or Randomize, so several settings change in one rebuild;
`world.auto` (the node's `auto_regenerate`) rebuilds after every edit instead. Every rebuild and
every repaint (`world.color`, at once) is a sliced task on the app's scheduler
([`sliced_task.h`](../inc/cvc/core/sliced_task.h), as `fe_lab`'s jobs are): `cvc::world::world_builder`
and `surface_painter` step it a few rows at a time, on the background pool natively and a few
milliseconds a frame on single-threaded wasm, so the old world stays on screen until the new one is
swapped in. `world.busy`, `world.progress` (0–100) and `world.phase` (the stage and its percent)
follow it, and `world.request: stop` ends it, keeping the world shown. Its trees and rocks draw
with level of detail: a `LodGraphicsNode` ladder per 128 m ground cell (under `<id>_props`,
`<id>_foliage` and `<id>_needles`), chosen each frame by screen error. Every rung is the same
full-maturity tree, coarser further off: rung 1 from 260 m (three-sided tubes, none under 5 cm, the
leaves as solids), rung 2 from 700 m (a trunk and a crown with the tree's colour and coverage), the
radii scaled by the quality preset (the wasm build coarsens sooner). The ground does too: the node
draws nothing itself but holds the texture, and its children are a ladder per chunk of 64 terrain
cells (doubled until at most 16 chunks span the world). Rung k keeps every 2^k-th sample, down to 4
cells a chunk, so every rung is the terrain's own surface, points, normals and UVs, with fewer
triangles. A chunk coarsens once its largest height error falls under the preset's pixel budget,
and stays within one rung of each neighbour; skirts hung from the edges two chunks share hide the
seam. The chunks share the node's one texture and take its `casts_shadow`, `shader:` and
`material:` keys. Physics, nav and `ground:` read the full terrain. The roads shape that terrain
rather than lie on it (`cvc::world::road_grade`): each is level across, climbs at most 12% (5% into
a junction), changes grade gradually and stays above the water, keeping to a gentle hill and cutting
through a steep one, with the ground cut or banked to it, so a vehicle drives them without
bouncing; every street is two Humvee lanes wide, drawn as smooth strips with a dark yellow centre
stripe. The water is a surface per terrain chunk with any
under it (`<id>_water`), level at the water line on its chunk's LOD rung, coloured by the depth
under it (`cvc::world::water_colour`: the bed absorbed red first, so the shallows read sandy, then
green, then deep blue) from one texture over the world (`cvc::world::paint_water`) that is clear
wherever the full terrain mesh is dry, so the water ends at the mesh's own shoreline at every rung;
the ground's texture paints the water's colour where the mesh is under it, so a coarse rung's
ground that rises over a lake still reads as lake. `water: 0` leaves it out.

**The fast render path.** A big static world draws fast when it is a few merged meshes and its
shadow maps are re-baked rarely and spent where the camera looks. These keys say so
(`examples/ariadne/documents/terrain_lab.ari` uses all of them but `casts_shadow: false`: its
ground casts, so its ridges shade its valleys):

```yaml
scene:
  shadows:
    enabled: true
    resolution: 4096
    update_interval: 4
    softness: 2                 # the edge filter's radius in map texels (0: one hard tap)
    focus: camera               # or { follow: camera, min_radius: 150, max_radius: 600,
                                #      per_distance: 3, lead: 0.5, lift: 20, ground: <node> }
                                # or { center: [x, y, z], radius: r }: a fixed sphere
  lights:
    - light: sun
      rig: three_point
      sun: true                 # the key a directional sun over the whole world, not a spot whose
                                #   cone ends in a circle on a large ground (aimed by the focus)
      back: 0
      wash: 0                   # or { intensity, count, height }; 0 drops the downlight ring
  nodes:
    - node: ground
      casts_shadow: false       # receives shadows, casts none, never forces a re-bake (no
                                #   self-shading: a valley is not shaded by its ridge)
      material: { opaque: true }  # drawn opaque: no per-texel alpha scan of its texture
```

A camera focus spends the maps on a sphere `lead` radii ahead of the focal point, of radius
`min_radius + per_distance × (eye-to-focal distance)` clamped to `max_radius`, `lift` over the
`ground` node's heightfield; `tick_scene` re-aims it every frame, and the maps re-bake once it has
moved a fiftieth of its radius. Past it the scene is lit unshadowed.

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
- **Placement.** Each body starts at its node's world pose, composed from the scene's transforms
  (a nested node's included).
- **Stepping.** The world runs on `app.world_clock()`, driven as `clock: {}` would drive it, and
  steps by the quanta due. With a renderer, `tick_scene` does that and draws each moving body's
  node at the body's pose, between the last two steps, wherever the node sits in the hierarchy.
- **Headless.** The world is core libcvc's (`cvc::ariadne::SceneWorld`,
  [`scene_world.h`](../inc/cvc/ariadne/scene_world.h)), so it runs with no renderer too. An
  `AppRuntime` with no `SceneAdapter` (`ari`, `ari test`, pycvc's headless `AriApp`, `arix`)
  builds the world from the scene alone and steps it on the clock in `drain()`, on real time; a
  paused clock with step requests, or lockstep, makes that deterministic. Nothing is drawn, and a
  scene with no world still loads with "scene ignored". The same document reaches the same world
  state drawn or not, bit for bit (`cvcgl_ariadne_physics`). A geometry node's body, a
  `generated_world` node's and a data node's run headless; another custom node's needs a
  renderer. `AppRuntime::scene_world()` is the world it runs.
- **Step and restart.** Every `AppRuntime` handles `sim.step` and `sim.restart`, the bare-name
  events `sim_transport.ari`'s Step and Restart buttons raise; a host's own `on("sim.step", ...)`
  replaces them.
  - `sim.step` (`AppRuntime::step_world(n)`) adds one to the scene clock's step key
    (`<prefix>.sim.step` by default), and the clock runs that many more quanta, paused or not.
  - `sim.restart` (`AppRuntime::restart_world()`) puts `app.world_clock()` back to tick 0 and
    rebuilds the scene's world as the document built it (`SceneWorld::restart`, headless or
    drawn): the same world object, with every body back where its node put it and at rest unless
    it declared a velocity. A generated world keeps the world it has now, and the drawn bodies are
    bound to their nodes again. Stepping a restarted world gives the same state as stepping a world
    loaded fresh (`ariadne_scene_world_test`, `cvcgl_ariadne_physics`). It then adds one to
    `<prefix>.sim.restarts`, so a program or a host can reset state of its own.
  - pycvc's `AriApp` has `step_world(n)` and `restart_world()`.
- **A generated world's body.** `body: {}` on a `generated_world` node is the world's colliders:
  its terrain and its buildings, each a static body (`shape:` and `motion:` do not apply). The
  world is made in core libcvc ([`scene_generated_world.h`](../inc/cvc/ariadne/scene_generated_world.h)),
  so headless the scene's world runs the node itself: it seeds the `world.*` keys, makes the
  world as the document says, publishes `world.stats.*`, and makes a new world at once on
  Regenerate (or on an edit while `world.auto` is on). Drawn, cvcGL's node makes it as a sliced
  task and hands over its colliders. Either way a new world's colliders replace the old ones.
  `SceneWorld::generated_world(id)` is the world a headless host made: its raster, roads,
  buildings and water.
- **A data node's body.** `body: {}` on a `source: { data }` node collides with its dataset's
  world, terrain or geometry once the dataset is ready, and again each time it changes. The
  scene's world takes them, headless or drawn, and it needs the scene's `physics:` block.
- **Made colliders stay put.** A generated world's and a dataset's colliders sit where they were
  made: the node's transform does not move them (it warns).
- **Telemetry.** Each step publishes `physics.stats.{tick, time, step_ms, bodies, active_bodies,
  contact_events, saturated}` under the document's prefix, drawn or not (a `scenes:` entry's
  under `scenes.<id>.physics.stats`), and also:
  - `step_ms_history`: the last 120 step times, comma-separated, which a `plot:` draws;
  - `nodes`: a line per node with bodies, giving its bodies and the ones awake.
- **Physics on and off.** `physics.enabled` (seeded 1) turns the stepping off and on. While it is
  off, the world holds still and the clock runs on, so the steps due are dropped. Turned back on,
  the world carries on from where it stopped without catching up. `world_physics.ari` is the
  panel for it: physics on, pause, speed, step and restart, and the telemetry.
- **Worlds.** A scene can have several worlds and switch between them. `world: <name>` on a node
  makes it part of the world called `<name>`, such as a `generated_world` node, a data node (a
  city bundle), a terrain or a flat ground. One world is on at a time, and the others' nodes are
  out of the physics world and not drawn:

  ```yaml
  scene:
    physics: {}
    world: { source: generated, cell: 2 }   # the world on at the start; the grid's cell (m)
    nodes:
      - { node: land, type: generated_world, world: generated, body: {} }
      - { node: city, source: { data: city }, world: data, body: {} }
      - { node: square, source: { box: { size: [200, 200, 1] } }, world: flat, body: static }
  ```

  - **Switching.** `<prefix>.scene.world.source` names the world on. It starts as the block's
    `source:`, else the first node's world. Writing another name switches at the next step,
    headless or drawn (`SceneWorld::bind_worlds` / `set_world`).
  - **What the runtime publishes.** Under `scene.world`:
    - `sources`: the worlds' names, one per line;
    - `node` and `kind`: the world's nodes, and whether it is generated, data, terrain or flat;
    - `ready`: 1 once the world has its world (a generated world made, a dataset loaded);
    - `rev`: goes up each time the world on changes, on a switch, a Regenerate or a reload;
    - `status`: a line saying where it stands;
    - `on.<node>`: 1 for each node of the world on, 0 for the others'.
  - **Drawing.** A world node with no `visible:` of its own follows `scene.world.on.<node>`, so
    only the world on is drawn.
  - **The grid.** With `cell:` the runtime also publishes the navigator's grid of the world on
    (`scene.world.grid.{rows, cols, cell, occupied, summary}`, `SceneWorld::grid`):
    - a generated or data world's grid comes from its world;
    - a terrain's from its height field;
    - a flat ground's is an open walled grid over its footprint.
  - **The panel.** `world_source.ari` picks the world, and shows its settings and readiness:
    - for a generated world, `world_controls`' widgets;
    - for a data world, its loading progress, plus loading another bundle into the same dataset.
    Its `world:` arg is the key root (default `scene.world`), so it can drive another world
    runtime's keys by their absolute path, as `world_physics`' `physics:` arg can.
  - **Restart.** `sim.restart` rebuilds only the world on.
- **Vehicles.** The document's top-level `world:` block lists the vehicles it offers
  ([`world_rules.h`](../inc/cvc/ariadne/world_rules.h)), and the scene's world runtime spawns
  them as Jolt vehicles into the scene's own physics world, headless or drawn
  ([`world_vehicles.h`](../inc/cvc/ariadne/world_vehicles.h)). They collide with the scene's
  bodies, step on the world clock with them, and drive on the world that is on:

  ```yaml
  world:
    vehicles:
      archetypes:
        humvee:
          vehicle: humvee                                    # a preset: humvee, roboracer
          ranges: { mass: [2000, 3500], max_speed: [5, 20] } # what a spawn may set
          controllers: [nav@1, none]                         # the first is a spawn's default
        racer:
          vehicle: { preset: roboracer, max_speed: 6 }       # a preset with overrides
          look: { file: racer.glb, scale: 1, yaw: 90, z: 0 } # default: the chassis box
  ```

  - **Archetypes.** Each names its plant (`jolt`; `kinematic`, `dynamic` and `dynamic4w` are
    accepted but not run in a scene world yet), its body (a preset, with `mass`, `max_speed`,
    `max_accel`, `max_decel` or `max_steer` over it), its look, the ranges a spawn's settings are
    held to, and its controllers: `nav@1`, the navigator, or `none`.
  - **Looks.** With no `look:`, a humvee is the real Humvee, the vehicle-humvee package's
    `Humvee.glb`, when it is installed (`find_vehicle_model`: beside `$CVC_NAV_BUNDLE`, else in
    `share/cvc-scenes/shared` or `share/cvc-scenes/*/` under `$CVC_PREFIX`, `$CVCPKG_PREFIX`,
    `$CONDA_PREFIX` or the program's prefix), with its texture. It is oriented and scaled to the
    chassis as the nav demos do it, and stands on the ground under it. Anything else, or a humvee
    without the package, is its chassis box in the entity's two colours, with a cabin and wheels.
    `look: { model: humvee }` names a model, `file:` a mesh beside the document and `asset:` a
    document asset; `fit: true` orients and scales a file as a model is; `scale:`, `yaw:` and
    `z:` place it.
  - **Entities.** A spawn (`WorldVehicles::spawn`, or the keys below) is an entity with an id,
    an identity (a name, a team, two colours, key/value metadata), a controller and its
    settings. `nav@1` drives it from goal to goal across the world's grid (the scene's
    `world: { cell }`, 2 m by default), with the default policy or a trained `.cvcnav` (loaded
    once per runtime per content hash). `none` parks it where it starts. A switch to another
    world, a Regenerate or a reload places every entity again on the new ground; `sim.restart`
    puts each back where it started, at rest.
  - **What the runtime publishes.** Under `vehicles`: the catalog (`archetypes.names`, and for each
    archetype its size, mass, power, top speed, wheelbase, plant, controllers, look and ranges),
    `selected` and `selection.*` (the selected archetype as a spawn would make it now), and
    `status`. Under `entities`: `ids`, `count`, `rev`, `summary`, and for each entity its
    identity, pose (`x`, `y`, `z`, `yaw`), `speed`, `power_kw`, `energy_wh`, `wh_per_km`,
    `distance`, `wheel_load`, `aero_force` and `gear`. `physics.stats.vehicles` has a line per
    vehicle.
  - **Spawning from keys.** Set `vehicles.selected` and `vehicles.spawn.*` (`name`, `team`,
    `color`, `color2` as `r,g,b`, `metadata` as `key=value, ...`, `controller`, `policy`,
    `policy_sha256`, `mass` or `mass_t` (a fraction of the range), `max_speed` or `max_speed_t`,
    and `at` as `x,y[,heading in degrees]`; empty: a free point), then change
    `vehicles.spawn.request`. The result is in `vehicles.spawn.status`. `vehicles.remove.id`
    with `remove.request` removes one, and `clear.request` removes them all.
  - **Drawing.** cvcGL draws each entity on its chassis, posed between steps as the other moving
    bodies are. A `type: vehicle_preview` node (props `vehicles:`, the catalog's root, and
    `spin:` in degrees a second) draws the selected archetype in the spawn's colours.
  - **The panel.** `vehicle_selector.ari`: the vehicle, its stats, a preview viewport onto a
    `scenes:` entry holding a `vehicle_preview` node (its header shows one), the settings, the
    controller and policy, the identity, Spawn, and the spawned vehicles with Remove and Clear.
  - **One runtime per world.** Each `SceneWorld` has its own physics world, vehicles, policies
    and keys, with nothing process-wide, so a host that runs several worlds runs one each.
- **Terrain transforms.** A terrain follows its node's position but not its rotation or scale.
  Draw a ground you collide with unrotated and unscaled.
- **Reading physics.** `RealizedScene::physics` is the world (`RealizedScene::world` the
  `SceneWorld` holding it), `RealizedScene::body(id)` a node's body (the first, for a node with
  colliders; `world->bodies_of(id)` has them all), and `RealizedScene::height_fields` the
  heightfield nodes' fields.
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
a `tf_editor` edits the ramp and the opacity curve: mesh_tools and fe_controls each hold one
(`show_isocontours: false`). The values draw see-through where the opacity curve is below 1 (times
fe_lab's surface opacity); the isocontours are not used. The colormap key (`color.colormap`,
`view.colormap`) seeds it and resets it to that colormap's ramp, opaque, whenever it changes, so a
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
| `request` | `load`, `reload`, `mesh`, `solve`, `stop` (`stop` ends the running job at its next step: a heat solve keeps its field so far, a load, mesh or other solve keeps nothing and the node keeps what it had); several separated by spaces run in order, so `stop load` drops the running job and loads | `auto_mesh: true` seeds `mesh` |
| `busy`, `status`, `progress` (0-100), `stats.vertices`, `field.min`, `field.max` | published | — |
| `phase` | published while a job runs: its stage and percent (`x lines 40%`, `factoring 62%`), `""` when idle; fe_controls' progress bar shows it | — |
| `stats.tets`, `stats.hexes` | published: the element count (the other is 0) | — |
| `stats.quality_min`, `stats.quality_mean` | published: minimum dihedral angle (tetrahedra, degrees) or scaled Jacobian (hexahedra) | — |
| `stats.work_ms` | published: the wall time of the last run of commands, from the first to idle | — |
| `stoppable` | published: 1 while a job runs (the Stop button's `enabled_when`), else 0 | — |
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
  | `gl.scene` | a scene adapter realizes `scene:` (arigl in a window or `--terminal`, the cvcGL hosts) |
  | `gl.scene.shaders` | the scene runs the document's GLSL (a node's `shader:` block); arigl in a window or `--terminal` |
  | `scene.physics` | a scene's `physics:` and its nodes' `body:`s run, drawn or not: every host, headless ones too, in a build with Jolt. A document that needs the world but no picture lists this, not `gl.scene` |
  | `gl.ui.shaders` | the UI draws GLSL itself (`shader_canvas`): a GL window, never a terminal |
  | `gl.shaders` | both `gl.scene.shaders` and `gl.ui.shaders` |
  | `gl.view` | a live render view can sit in a widget |
  | `ui.windows`, `ui.menubar`, `ui.color` | movable windows; a menu bar; coloured styling |
  | `ui.fullscreen` | `(fullscreen ON)` makes the window cover the screen: a GL window, or the canvas in a browser; never a terminal (§10.7) |
  | `ui.maximize` | a window's title bar has a maximize button that fills it to the screen under the menu bar, and back (`tree.<id>.maximized`, §5.9); ImGui and FTXUI |
  | `input.mouse`, `input.keyboard` | pointer input; key and text input |
  | `ui.group_click` | a group with `on_click:` is one item over its whole area, and `hover: highlight` lights it (§5.19); ImGui, FTXUI and tvision |
  | `raster.bitmap` | images show in real pixels (a GL window, or a kitty or Sixel terminal) |
  | `input.on_change` | `message_input:` takes `live: true` and `on_change:` (the draft as it is typed), and a draft the document writes while it is edited (§5.15) |
  | `term`, `term.kitty`, `term.sixel` | a text terminal; one that speaks kitty graphics; one that draws Sixel |
  | `relay` | the document is a relay's server document, run by `arisrv --doc` (§9.7) |
  | `image.fullscreen` | `fullscreen: true` on `image:` and `video:` shows the picture alone, filling the window or the terminal (§5.13) |
  | `image.badge` | `badge:` on `image:` and `video:` shows a label over the picture, dimmed under it, or on the line under it in a terminal (§5.11) |

  GLSL runs in two places, and they are separate facts. The scene's shaders run in the
  scene adapter's renderer (`SceneAdapter::runs_shaders`), the UI's in the backend
  (`Capabilities::glsl`). `arigl --terminal` has the first without the second: it renders
  the scene offscreen through GL and draws the UI as text cells. So a document whose only
  GLSL is in `shader:` blocks requires `gl.scene.shaders` and runs there. One with a
  `shader_canvas` requires `gl.ui.shaders`, or lists it as optional and checks
  `(has? "gl.ui.shaders")`. `gl.shaders` asks for both, so it fails on `--terminal`.
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

A `shader:` block is an implicit `gl.scene.shaders` the same way. When the host has a scene
but its renderer does not run the document's GLSL, the load warns once, naming the shaded
nodes; they draw unshaded. Listing `gl.scene.shaders` makes that an error, or (optional)
one "optional requirement" warning instead. A host with no scene at all already warns that
the scene is ignored, so it adds nothing. (`shader_canvas` would be an implicit
`gl.ui.shaders`, once that widget exists.)

Every document under `examples/ariadne/` declares a `requires:` block, and
`AriadneComponents.ShippedExamplesDeclareWhatTheyNeed` keeps it so. A document with a
`scene:` lists `gl.scene`, so `ari` refuses it with one error where it used to drop the scene
with a warning. One with a `shader:` block also lists `gl.scene.shaders` (`lsystem_forest.ari`,
as optional: its bump and bark presets are detail). Write `requires:` the same way in a new example: list what the document cannot
work without, and mark the rest `optional: true` (as `terminal_images.ari` does for
`raster.bitmap`, since half-block cells still show the picture).

A document that can run without something asks for it at run time with `(has? "name")`,
which answers the same question for a token, a function or a widget type. It works in
programs and in reactive expressions, a `load:`ed module's included (its `init:` too: a module's
host is the document's host). The load-time scan does not follow the guard: a call behind
`(has? "name")` still warns, until `requires:` lists the name as `optional: true`. So the two go
together: the entry says the document can do without it, and the guard is how it does.

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

A `load:`ed module (§6.3) may have its own `requires:`, kept on its mount wrapper
(`Widget::requirements`), and `AppRuntime::load` checks it with the document's:

- A required entry the host lacks fails the whole load. The error names the module:
  `... requires: room-join (module 'includes.call')`.
- A missing optional entry warns, also naming the module.
- An entry covers the calls of the programs in that module and the modules it mounts. The
  document's own entries cover every program, its modules' included. A module's entries never
  cover its parent's calls: what a module needs is its own business, and a parent that calls the
  same verb lists it too.

### 5.8 Window size, placement and scrolling

A window needs neither `pos:` nor `size:`. Its size:

- A window with a `size:` (px or `"%"` of the view, per axis; `min:`/`max:` bound it) opens at
  that size, or at its persisted `tree.<id>.geometry` (§11.4) once it has one. The user may
  resize it, and that size is the one persisted.
- A window with no `size:` fits its content every frame, so it grows and shrinks with what it
  shows. It stops at the edge of the view, measured from where it stands, and never goes below an
  authored `min:` or past a `max:`. Its persisted geometry places it but does not size it.
- A window that closes and opens again in the same run (§5.9, or its `visible_when` turning true
  again) is sized as on its first opening: by its `size:`, else by what it shows now, rows it
  gained while closed included. Only a size the user gave it holds.

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

- **ImGui** measures in logical pixels at the UI size (§5.14): a document's `px` (a window's
  `pos:` and `size:`, a column's width, a picture's or a plot's size, the area a group's
  `background:` takes) is multiplied by the UI size
  and the display's pixel ratio, as the UI's text and widgets are, and the geometry it persists
  is divided back, so a layout looks the same at any UI size and on a phone's dense screen. A
  window opens inside the view: no larger than the work area, and moved in from its `pos:` (or
  its persisted place) until all of it shows, so a layout made for a laptop opens on a phone in
  reach. In a browser the view is the canvas from the first frame, as the page has laid it out.
  When the view gets smaller later (a browser window made smaller, a phone turned on its side,
  fullscreen left), a window that showed in full moves back in the same way, no larger than the
  work area. A window moved in, where it opened or since, goes back toward its place and size as
  the view grows again, as far as the view allows, until it is there (not one that opened in a
  phone's list of titles, §5.14: its place is the list's). One the user moves or resizes
  meanwhile stays where they put it, and so does one the user left partly outside the view.
  ImGui gives a window with no size ImGui's always-auto-resize: it measures the window's widgets
  each frame, its labeled widgets at the width ImGui gives an auto-resizing window's items (16
  font sizes), and a row-split pane as wide as its own widgets. A window with a size gets a
  horizontal scroll bar as well as ImGui's vertical one, and a long label shrinks its slider,
  input, combo or colour widget rather than run past the window's edge. A
  `layout: {kind: horizontal}` row with no `col_widths` puts its cells side by side on one line,
  each at its natural width, as the terminal backends do. One with `col_widths` is an ImGui table
  with a column per entry. `borders` and `resizable` need that table, so a row using them needs
  `col_widths` as well; without it the table has one column.
- **The terminal backends** lay a window with no size out at its content's size every frame,
  and cascade new windows instead of flowing them. A window that would run off the screen stops at
  the screen's edge, and a window whose content is larger than it is (or than its `size:`) gets a
  scroll bar. Its body scrolls to keep the focused widget in view. The mouse wheel over it scrolls
  it too, 3 rows a notch, from where it shows, and stops with its last row at the bottom (a feed in
  `ari`). A `text_view` under the pointer takes the wheel first. After a turn of the wheel the
  window stays where the wheel left it: a click there focuses a widget without jumping to it.
  Tab and the arrows move the focus and the window follows it again. The wheel over a window
  that fits, or over the background, is left to the host and the document (`on_pointer:`), as
  before. On the tvision desktop the wheel over a window's body scrolls it the same way.
  `ariadne_ftxui_test`'s `TheWheelScrolls*` tests turn it.
  On the tvision desktop, tvision's frame holds the window, and the host sizes that frame on every
  walk, in each axis: the document's size where it gave one, else the content's. That stops when
  the user resizes the window (the frame's corner, Ctrl-F5 then Shift and the arrows, or the zoom
  box): from then on it keeps the user's size, through a close and a reopen too. Nothing else
  counts as the user's: not a size the host gave the window to fit it, nor tvision scaling the
  windows when the terminal is resized, nor a move. Only the user's size is reported back to
  `tree.<id>.geometry`; for any other window the host reports its place alone.
  `ari_run_reopen_refit_ftxui` and `_tvision` check this in a tmux pane (Linux):
  [`src/ari/tests/reopen_refit.ari`](../src/ari/tests/reopen_refit.ari)'s windows gain rows while
  closed and must show them when they open again, and on tvision a window resized with Ctrl-F5
  must come back at that size.

### 5.9 Closing and reopening windows: the Windows menu

A window's open state is `tree.<id>.open` (default open; `<id>` is its `id:`, else its title). A
window whose state is `0` is not drawn. A `closable: true` window has a close button that sets it
to `0`. One that opens again is sized as §5.8 says: by its `size:` or its content, unless the user
resized it.

**Maximizing.** A window's title bar also has a maximize button, beside the collapse arrow and the
close X.
- **The button:** pressed, it makes the window fill the screen under the main menu bar. Pressed
  again, the window goes back to the place and size it had.
- **The state** is `tree.<id>.maximized`, `1` or `0`. A document may write it to maximize a window
  or put it back.
- **Geometry:** while a window is maximized its `tree.<id>.geometry` isn't written, so it keeps
  where the window goes back to, across a restart too.
- **Opting out:** `maximizable: false` on a window gives it no button; the default is `true`.
- **By backend:**
  - ImGui: a square, two squares while maximized. The window can't be moved or resized while
    maximized.
  - FTXUI: `[↑]` beside the `[x]`, `[↕]` to go back (`[^]` / `[=]` without those glyphs). The
    window is drawn from the top-left as big as the terminal.
  - The tvision desktop has its own zoom box (and Ctrl-F5) instead, which this key doesn't follow.
- **The token** `ui.maximize` (§5.7) says the backend has the button.

Every document gets a **Windows** menu to open such a window again, with no menu items to write:

- It holds a ticked item per window, bound to that window's `tree.<id>.open`. Unticking an item
  closes the window, closable or not; ticking it opens it.
- It lists only the windows no menu item of the document already toggles. A document with its own
  "Settings" or "View" menu bound to `tree.<id>.open` (`lsystem_forest.ari`,
  `conference_roster.ari`) keeps it, and the Windows menu lists only the windows that menu leaves
  out. When it leaves none out there is no Windows menu.
- A window made by `repeat:` is listed once per instance, under the title that instance shows
  (its `bind:`, else its label with `{i}` filled in), after the other windows. The list follows the
  repeat's count each frame. Opening one from the menu also brings it to the front.
  `conference_roster.ari` sets `windows_menu: View`, so its per-peer camera windows are listed by
  name at the end of its View menu.
- A host's menus (`AppRuntime::set_host_menus`) count as the document's: arigl's
  *Settings > Controls...* opens the Controls window, so arigl's Windows menu leaves it out. Under
  a host without one, the Windows menu is how the Controls window opens.
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
the work still to come: [ARIADNE_MEDIA.md](roadmap/ARIADNE_MEDIA.md).

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
- **Formats:** PNG, JPEG, GIF, BMP and, where libcvc has libwebp, WebP. Where it
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
- **Filling the width.** `size: "N%"` makes the picture that share of the width it has on its
  line: `"100%"` is as wide as its window, or its cell of a grid, and grows and shrinks with it,
  the picture's own shape setting the height. `size: ["N%", h]` is a box that wide and `h` tall.
  A small picture is scaled up to the width; a large one keeps its pixels up to the cache's 2048.
  `max_height: h` keeps a picture whose height follows its shape (a phone's portrait video, say)
  no taller than `h`: it is drawn narrower instead. `video:` takes both too (§5.13).
  - ImGui measures the line where the picture is drawn. A window with no `size:` is as wide as its
    other content makes it, and at least the widget's own width (256 px for `image:`, 480 for
    `video:`), so a window holding only the picture still has room for it. While a window is
    resized the picture is fitted again each 64 px, and scaled in between.
  - The terminal backends lay a window out after its widgets are drawn, so they can't measure
    the line: there the picture is the widget's own width.
  - The capability token `image.fill` (§5.7) says a host does this. An older host reads
    `size: "100%"` as no size and draws the widget at its own width.
- **Every backend draws them**, since the core publishes the fitted picture as any other image:
  ImGui as a texture, and the terminal backends as Kitty, Sixel or iTerm pixels, or half-blocks
  (§4). Hosted tvision always draws half-blocks. In half-blocks, a transparent pixel leaves its
  half of the cell to the terminal's own background, so a round avatar has no corners on a light
  or a dark terminal. Kitty, Sixel and iTerm draw transparent pixels black.
- **Animated GIF and WebP pictures move**, every frame fitted to the box like a still, on a
  loop the widgets showing one picture share. A terminal gets at most 8 new frames a second.
  An animated WebP needs libwebp's demux library (cvcpkg's libwebp has it). An APNG, or an
  animation over 512 frames or 128 MiB decoded, shows its first frame.
- **A video URL** (`.mp4`, `.webm`, `.mov` or `.m4v`, a query or fragment after it allowed, the
  rule web clients use for avatars) in an `image:` plays as a decorative video (§5.13): looping,
  silent, fitted and shaped like the picture would be. Without a player it is the placeholder,
  and `status:` says `unsupported` (or `remote`, `failed`) where a picture would say `ready`.
- **`fullscreen: true`**: a click on the picture shows it alone, filling the window (or the
  terminal), until Esc (§5.13).
- **`badge:`** is a label shown over the picture, with the picture dimmed under it. A media
  grid shows "+3" this way on its last tile, for the pictures it leaves out:

  ```yaml
  - image: ""
    repeat: (state-get "post.tiles")   # the tiles shown, 4 at most
    bind: post.media.{i}.url
    size: [120, 120]
    fit: cover
    badge: (if (= "{i}" "3") (state-get "post.more") "")   # "+3" on the last, or ""
  ```

  - It is literal text, or an expression (`(...)`) giving it, which can use `{i}` in a
    `repeat:`. An empty text shows nothing, and the picture isn't dimmed.
  - It isn't `overlay:`, which is a widget type: on a host before this one, an `image:` with an
    `overlay:` key is read as an overlay window.
  - The picture stays the item, so its `on_click:`, hover and tooltip are its own.
  - ImGui draws the text centred on the picture, a third of its height (within one and four
    lines of text, and narrower than the picture). A click on the label is a click on the
    picture.
  - A terminal (FTXUI, tvision) can't draw over a picture's cells, so it puts the text in bold on
    the line under the picture, centred to its width. That line isn't part of the picture's
    click, but a clickable group around the picture (§5.19) takes a click on it.
  - It isn't drawn in the fullscreen view (§5.13), where the picture is shown alone.
  - `(has? "image.badge")` (§5.7) says the backend shows it. Where it doesn't (an older host),
    a document can put a "+3 more" caption under the grid instead.
- **The capability token `image.url`** (§5.7) holds in every host that does this, so a document
  for older hosts too gates its pictures with `(has? "image.url")` or
  `requires: [{name: image.url, optional: true}]` and draws its own placeholder elsewhere (an
  older host shows an unknown URI as text).
- **`status:`** names a state key the widget keeps `loading`, `ready` or `failed` in, with the
  reason under `<key>.error`. A document reads it to show something else when a picture is
  missing, and a test asserts on it.
- **In a browser build without threads** (ari.html, arigl-web) a picture's http(s) request is
  started, not sent in place (§10.2), so the frame loop goes on while it is out. The reply is
  decoded when the browser hands it over, between frames. A `file://`, `cvc://` or `state://`
  picture is read and decoded between two frames, one at a time.
- **Pictures in a line of text: `emoji:`.** A `text:` with `emoji:` names a state key that holds
  a map of shortcode names to picture URIs (a post's custom emoji, as a social site's API gives
  them) or to text. Each `:name:` in the text that the map names as a picture is drawn as that
  picture, one line tall at its own shape, in the line where the backend can put a picture there:

  ```yaml
  - text: ""
    bind: feed.items.{i}.line          # "Shipped it :party: at 14:50:47"
    emoji: feed.items.{i}.emoji        # {party: "https://media.example/e/12.gif"}
  ```

  - A shortcode is found as txwtf, Slack and Mastodon find one: 1 to 64 of `A-Z a-z 0-9 _ + -`
    between two colons that aren't glued to a letter, digit or `_` outside them. So the `:50:` in
    `14:50:47` stays part of the time, while `:a::b:` is two. The map's names are looked up as
    written, then in lower case.
  - A name whose value isn't a URI is replaced by that value, as text, in every backend: a
    unicode emoji's character (`100: "💯"`), which the font draws. A name the map doesn't have
    stays as it is. A terminal backend can't put a picture in a line of text, so there a picture's
    `:name:` stays text too.
  - The pictures load as an `image:`'s do (the host's network and its allowlist, the cache), and
    a picture still loading is a placeholder. An animated GIF or WebP moves in the line, on the
    loop every widget showing it shares. A video URL (as `image:` takes one, §5.11) plays in the
    line as a decorative video (§5.13): silent, looping, by itself, where the host has a player.
    Hovering a picture shows its `:name:`. `{i}` in `emoji:` is substituted in a `repeat:`.
  - The capability token `text.emoji` says a host does this.

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

`background: URL` is short for `background: {src: URL}`. A video URL (as `image:` takes it,
§5.11) plays as a silent looping video behind the content, where the host has a player; an
animated GIF or WebP moves. The picture loads as an `image:`'s does
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

### 5.13 Video and audio: `video:` and `audio:`

```yaml
- video: clip                           # the label: the caption its messages use
  src: https://media.example/p/9.webm   # a URI (§5.11), or a computed expression giving one
  auth: example                         # optional: the host's credential, as image: takes it
  poster: https://media.example/p/9.jpg # optional: shown until the first frame
  size: [480, 270]                      # the picture's box; fit: as image: takes it
  bind: feed.items.{i}.player           # the player's state keys (below)
  autoplay: false
  loop: false                           # where the Loop box starts
  muted: false
- audio: voice note                     # the same, with no picture
  src: https://media.example/p/10.opus
  bind: feed.items.{i}.voice
```

- **`controls: false`** makes a decorative picture: no controls, and `autoplay`, `loop` and
  `muted` default to true. A muted one opens no sound device, so it plays in an SSH session too
  (it makes no sound there). `shape: circle` and `fit:` apply to its frames, and `status:` (as
  `image:` takes it) holds its state. A video URL in `image:` or `background:` is one of these.
- **The controls** are drawn with the widgets every backend has: Play / Pause, a seek slider
  with `m:ss / m:ss` as its value, Mute and Loop. "Open" is added where the media can't play here.
  They look and work the same in ImGui, FTXUI and tvision.
- **Loop** starts checked when `loop: true`. Unchecking it lets the player stop at the end from
  then on, and checking it makes it start again there. A new `src:` for the same widget (a
  repeat's row showing another post) starts as the box is. With `controls: false` there is no box,
  and `loop:` (true by default there) is how it plays.
- **The picture.** The newest frame is published as an image and drawn like one: an ImGui
  texture, or Kitty, Sixel, iTerm or half-block cells. A terminal gets at most 8 frames a
  second. Before the first frame the poster shows, else a placeholder of the box.
  `size: "100%"` and `max_height:` (§5.11) make it as wide as its line, at the video's own shape:
  `size: "100%"` with `max_height: 480` is a feed's video, as wide as the post, a portrait one
  narrower.
- **`autoplay: true`** starts it once it can, and from then on, until the user presses Play or
  Pause (or the document writes `playing`), it plays while its picture shows: scrolled out of its
  window's view, or its window closed, it pauses, and back in view it plays on. So a feed's
  videos play as they scroll by, not all at once. A terminal shows the whole window, so there it
  plays while its window is open.
- **The state keys** live under `bind:`, or `ariadne.media.players.p<n>` without one:

  | key | |
  |---|---|
  | `state` | `idle`, `loading`, `ready`, `playing`, `paused`, `ended`, `failed`, `remote` or `unsupported` |
  | `playing` | `1` / `0`; write it to play or pause |
  | `position`, `duration` | seconds (position at most 4 times a second while playing) |
  | `seek` | write seconds to seek; it is emptied once done |
  | `muted` | `1` / `0`; write it to mute |
  | `loop` | `1` / `0`, the Loop box; written from the start, and a document writes it to change it |
  | `error` | why it failed, or why it plays elsewhere |

- **A host plays them only if it has a player.**
  - `ari-media` (§9.2) and `arigl` have cvcMedia's FFmpeg + SDL3 player. FFmpeg decodes,
    frames are scaled to the box, and the sound plays through SDL3's audio subsystem, which is
    the clock (the wall clock without sound). The source is fetched whole first (the host's
    network policy and `auth:`, as for `image:`, at most 64 MiB) and read from memory.
  - FFmpeg is cvcpkg's `ffmpeg-lgpl`, linked shared, so the LGPL's terms are kept.
  - The lean `ari`, or any build without FFmpeg, has no player. Its media widgets say
    `unsupported` and offer "Open".
  - **The browser pages** (ari.html, arigl-web) play through the browser's own `<video>` and
    `<audio>`. FFmpeg is never compiled to wasm.
    - Each player is a hidden element. It plays the sound itself, and a video's new frames are
      drawn into a canvas the size of the box and copied into the widget's picture. That is one
      copy per frame shown, and it works the same in both pages, under windows and menus.
    - The policy and credentials are decided in C++ before the element sees anything:
      - an http(s) URL with no `auth:` is given to the element (it streams and seeks with range
        requests), once the host allows its origin;
      - one with `auth:` is fetched with the bearer token alone (no cookies, no redirect) into a
        blob;
      - `file://`, `cvc://` and `state://` sources are read from the page's files into a blob.
        A page lists its media in the config's `preload` to have them there.
    - A picture from another origin needs CORS to be read back. Without it the sound still
      plays and the error says so.
    - A browser plays sound only after the user has clicked or typed on the page. A muted
      `autoplay` starts by itself.
    - A site serving the pages under a Content-Security-Policy adds `media-src 'self' blob:`
      (and the media's origin when it is another). Without it the element falls back to
      `default-src`, and Chrome refuses media from another origin with "Media load rejected by
      URL safety check", which the player's `error` shows.
- **Over SSH** the sound would come out of the remote machine's speakers. So in an SSH session
  (`SSH_CONNECTION`, `SSH_CLIENT` or `SSH_TTY` set), players say `remote` and nothing is opened.
  - "Open" runs `(open-url URL)` where the host has it, else it warns with the URL.
  - An `auth:` source opened in a browser carries no token, so the browser needs its own
    sign-in. The tooltip says so.
  - `--media-play always` plays anyway, and `--media-play never` never plays.
- **Capability tokens** (§5.7): `media` means the media plays here, and `media.player` means
  the host has a player at all. `video.decorative` means `controls: false` and video URLs in
  `image:` and `background:` are understood, `image.animated` means GIF and WebP animate,
  `image.fill` means `size: "N%"` and `max_height:` are understood, `image.fullscreen` means
  `fullscreen: true` is (below), and `image.badge` means `badge:` is (§5.11). A video's
  `badge:` is drawn over its frames, above the controls.
- A player whose widget a frame did not draw (its window closed, its item scrolled out of a
  `repeat:`) is paused at once and closed 5 s later. `ari snapshot` waits while a player is
  still loading, as it waits for pictures.

**Fullscreen.** `fullscreen: true` on a `video:` adds a Full screen button to its controls, and on
an `image:` makes a click on the picture do the same. The picture is then shown alone, in place of
the whole document: no windows, no menu bar, a black background, the picture as large as fits with
its shape kept, and a video's controls along the bottom with Exit full screen in place of Full
screen.

```yaml
- image: ""
  bind: feed.items.{i}.media.0.url
  size: ["100%", 0]
  fullscreen: true                      # a click shows it alone, filling the window
- video: clip
  src: https://media.example/p/9.webm
  bind: post.player
  fullscreen: true                      # a Full screen button; post.player.fullscreen says 1 or 0
```

- **Leaving.** Esc, Exit full screen, or a click on the picture closes the view, and the document
  is drawn as it was. A new document (or a reload) closes it too.
- **The window.** In a GL window (`arigl`) the OS window goes fullscreen with the view, borderless
  at the desktop's mode, and goes back when the view closes. In a browser page the canvas goes
  fullscreen (`requestFullscreen`). The browser allows that only within the click's user
  activation, which still holds the frame after the click. The browser's own Esc ends its
  fullscreen without the page seeing a key, so the view closes when the page leaves fullscreen.
- **A terminal** (FTXUI) has no window to resize, so the view fills the terminal. Tab reaches a
  picture with `fullscreen: true`, and Enter shows it. The tvision desktop owns its screen, so
  there `fullscreen: true` changes nothing: the picture is drawn as without it, and a video has no
  Full screen button.
- **The player** is the same one, so a video keeps playing from where it was. Its frames are
  decoded for the view's size while it shows (`Player::set_frame_box`), and for the widget's own
  box again after it.
- **`<bind>.fullscreen`** (a `video:` with `bind:`) is `1` while the view shows it and `0`
  otherwise. Write `1` to open the view and `0` to close it.
- A click on a fullscreen picture inside a clickable group (§5.19) is the picture's, not the
  group's.
- Tokens (§5.7): `image.fullscreen` holds where the view opens (ImGui, FTXUI), and
  `ui.fullscreen` where the window can cover the screen as well (ImGui in a window or a browser).
  `(fullscreen ON)` (§10.7) does the window alone, for the whole document.

### 5.14 On a phone: UI size, hiding windows, the system keyboard

A document needs nothing of its own to work on a phone: the ImGui host (arigl, cyprty) does
this for every one.

- **UI size.** The UI is sized in logical pixels (CSS pixels in a browser) times the UI size,
  times the display's pixel ratio. A phone's canvas is about three device pixels to the CSS
  pixel, so without the ratio its UI would come out a third of its size. The UI size starts at
  1 (100%), on a touch device as with a mouse (touch mode widens the hit areas instead), and is
  the state key
  `<scene>.viewers.<viewer>.ui.scale` (arigl's: `ui.docs.ari.viewers.main.ui.scale`). The
  Controls window offers it as **UI size** (75% to 200%), under *Display* above the keys, for any
  host that calls `AppRuntime::set_host_ui_size`. A new size takes effect at the next frame, and
  the open windows grow or shrink with it. It holds for the run.
- **Hiding windows.** On a view whose short side is under 500 logical pixels the panels of a
  document with a `scene:` start hidden, and a round button in the bottom-right corner shows and
  hides them all (the menu bar too), so the scene is never buried. A document with no scene (a
  feed, a form) is all panels, so they start shown (`ImGuiOverlay::setPhoneHidesPanels`, which
  arigl's `AriRuntime::load` sets). The button stays above every window, even one dragged over
  its corner. A window collapses to its title bar with the arrow at the title's left (or a double
  tap on the title), closes with its X when it is `closable:`, and comes back from the View or
  Windows menu (§5.9). On a touch screen a window moves by its title bar only, so a finger that
  slips off a widget does not carry its window away.
- **A list of windows.** A document laid out for a wider screen would have its windows moved
  into the phone's view all onto the same spot, one over another. So on a phone (a view under
  500 CSS pixels on its short side, either way up) a window whose `pos:` or `size:` does not fit
  opens collapsed instead: its title bar goes under those of the others that did, from the top of
  the view, and the panels show a list of titles over the scene. The arrow (or a double tap)
  opens one: it unfolds from its title, as wide as the view and no taller than the room below,
  over the titles after it. Windows that fit where they were put, and those with no `pos:` (the
  flow, §5.8), open as anywhere else.
- **Labels.** A slider's, combo's or field's label stays in view: in a window that fits its
  widgets, a widget that would push its label past the view's edge is narrowed to leave it room.
- **Scrolling with a finger.** On a touch screen a finger drags a window's content along, its
  scroll bar untouched, and a finger that lifts while moving leaves the content gliding at its
  speed until friction stops it or it meets an end. This works in every window, child pane,
  `text_view:` and drop-down list that scrolls, and a press on content that is still gliding
  stops it (that press is not a tap). A drag scrolls when it starts on empty space, text, or a
  widget worked by a tap (a button, a checkbox, a selectable, a text field not yet being edited):
  past the drag threshold the widget's press is let go and it does not fire. A drag keeps its
  old meaning on a slider or a drag widget, a field being edited, a title or menu bar, a resize
  grip or border, a scroll bar, a table's column border, a scene view (§5.18), the transfer
  function editor, and any widget whose document handles `on_drag:`, `on_drag_start:` or
  `on_drag_end:` (§4.6). The window scrolled is the innermost one under the finger that scrolls
  along the drag's main axis, as the mouse wheel's is.

  The glide's speed falls as `v0 * exp(-friction * t)`, so a fling covers `v0 / friction`
  before it stops. Both are state keys beside the UI size, and a document reaches them under
  its own prefix in arigl (`viewers.main.ui.*`):

  | key | default | |
  |---|---|---|
  | `<scene>.viewers.<viewer>.ui.touch_scroll` | `1` | `0`: a finger's drag scrolls nothing (the scroll bars still do) |
  | `<scene>.viewers.<viewer>.ui.scroll_friction` | `2` | per second: 2 is iOS's normal deceleration, 10 its fast one; higher stops sooner, `0` glides to the end |

  ```yaml
  - slider_float: Scroll friction
    bind: viewers.main.ui.scroll_friction   # arigl: under the document's own prefix
    lo: 0.5
    hi: 20
  - button: Short flings
    on: (state-set "viewers.main.ui.scroll_friction" 8)
  ```

  On a touch screen arigl's Controls window offers it as **Scroll friction** under *Display*,
  for any host that sets `UiSizeControl::scroll_friction`. A mouse has the wheel and the scroll
  bars, so none of this applies to one.
- **The system keyboard.** A browser raises a phone's keyboard only for a focused text field,
  which the canvas is not. When a finger (or a pen) activates an `input_text`, the page focuses a
  hidden `<input>` placed at the field's caret, so the keyboard comes up and the browser keeps
  the field in view above it; what the keyboard types goes to the field, autocorrect and
  composed words included, and the field's deactivation puts the keyboard away. iOS raises its
  keyboard only for a focus made while the finger lifts, which is when the page makes it; a field
  that becomes active with no finger down (an `autofocus:` one) raises it on Android, and on iOS
  at the first tap on it. While the hidden input has the keys the canvas does not, so typing never
  reaches `on_key` or walks an avatar. A mouse click on a field types through the canvas as
  before. A native build has no phone yet; SDL3's `SDL_StartTextInput` is how one would raise the
  keyboard on iOS and Android.

### 5.15 Text buffers and message fields: `text_view:` and `message_input:`

Two widgets for a log and the field under it (a chat window, a console):

```yaml
- text_view: "##log"
  bind: chat.view.text          # the text, lines split on \n (a state path or a computed expression)
  height: 240                   # px (rows of 16 px in a terminal); 0 or left out: see below
  follow: true                  # stay at the bottom while lines arrive, if it is there (default)
  wrap: true                    # wrap long lines to the view's width (default)
  on_top: (chat-ui-more "chat" (state-get "room.token"))   # runs when the view reaches its top
- message_input: "##say"
  bind: chat.draft              # the draft: kept when the field loses focus, cleared when sent
  hint: Write a message         # shown while it is empty and not focused
  multiline: true               # Shift+Enter adds a line (default); false: one line only
  max_lines: 4                  # it grows to this many lines, then scrolls (default 4)
  max_length: 2000              # bytes (default 0: no cap)
  send: Send                    # a button beside it that sends it (ImGui; default: none)
  on: (chat-ui-send "chat" (state-get "room.token") (get-attr event "text"))
```

**`text_view:`** is read-only. Its text is a state path, or a computed expression (read-lane
rules, §2.1). The backend keeps its scroll position under the bound path, so a view that comes
back (a window reopened) keeps its place.

- **Following.** At the bottom, new lines keep it at the bottom. Scrolled up, it stays where it
  is.
- **Older lines.** When lines are added above (an older page), it moves by their height, so the
  lines in view stay put. Any other change of text (another channel) starts at the bottom.
- **`on_top:`** runs when the user scrolls the view to its top, once each time it gets there, and
  again when the wheel turns up there (ImGui) or the user presses Up against it (terminal). It
  also runs once for each text that does not fill the view, so a log with only a few lines asks
  for more until it fills or there is nothing older. An empty text does not run it. A disabled
  view runs nothing.
- **ImGui.** A bordered child window. Only the lines in view are drawn: their wrapped layout is
  kept and redone only when the text or the width changes, so a long log costs what is on
  screen. `height: 0` fills the window less what the window draws after the view (measured the
  frame before), which is how a log sits above its message field in a sized window. In a window
  that fits its content, it is 12 lines tall. Right-click copies the text.
- **Terminal backends** (FTXUI, and tvision, which draws FTXUI's widgets). The lines are
  word-wrapped to the width the view was laid out at last frame. It is `height` px tall in rows
  of `cell_px_h`, or 8 rows. With focus, Up and Down, PageUp and PageDown, and Home and End scroll
  it, and the wheel scrolls it from under the pointer. A gutter shows ▲ or ▼ (`^` or `v` without
  Unicode) where there is more.
- **A backend without one** draws the last lines as text lines (`Backend::text_view`'s default).

**`message_input:`** sends what is typed to its `on:` program.

- **Enter** sends it. The program reads it as `(get-attr event "text")`, and the draft at
  `bind:` is cleared. A message of blanks only is not sent. The field keeps the keyboard focus, so
  the next message can be typed at once. **`send:`** puts a button with that label on the field's
  right (ImGui), which sends it as Enter does; a terminal backend has Enter only.
- **Shift+Enter** adds a line (`multiline: true`, the default). In ImGui Ctrl+Enter does too. In
  a terminal, Shift+Enter usually reaches the program as a plain Enter, so Alt+Enter and Ctrl+J
  add a line there. Shift+Enter works where the terminal reports it (the kitty keyboard
  protocol, xterm's modifyOtherKeys).
- **Losing focus** keeps the draft at `bind:`. A draft written to `bind:` by a program shows in
  the field while it is not being edited.
- **ImGui** draws one line that grows with its wrapped text up to `max_lines`, then scrolls. The
  hint is drawn over it while it is empty. A bare-name `on:` (a host handler) gets no `event`;
  give it a program.
- **As it is typed: `live:` and `on_change:`.** `live: true` writes the draft to `bind:` at every
  change, not only when focus leaves. `on_change:` (it implies `live`) runs each time the draft
  changes, with `(get-attr event "text")` the draft, and once with `""` when a message is sent.
  Keys pressed between two drains run it once, with the draft as it is then.

  ```yaml
  - message_input: "##compose"
    bind: compose.draft
    max_length: 4096
    on_change: (compose-complete (get-attr event "text"))   # e.g. a #tag or @name completion
    on: (post-send (get-attr event "text"))
  - text: ""
    bind: (string.join (list (str (string.length (state-get "compose.draft"))) "4096") " / ")
  ```

  A draft the document writes to `bind:` while the field is being edited is taken by the field,
  with the cursor at its end. So `on_change:` can complete a word by writing the whole draft
  back. A key typed in the frame before that write lands is replaced by it. Without `live:` or
  `on_change:`, a field being edited keeps its own text, as before. `(has? "input.on_change")`
  holds where libcvc has this. `ariadne_ftxui_test`'s `MessageFieldLiveDraftAndACompletion` and
  `cvcgl_imgui_chat_widgets` type into one.

### 5.16 Conference chat: the chat verbs and the chat component

In a conference, the relay (arisrv) keeps the chat. Each client is pushed what it may read:

- the room's messages, to everyone in the room;
- a direct message, only to the connections holding its two participants' pids.

A client asks for an older page when its log reaches the top. The state layout, the pagination
and the permissions are in [ARIADNE_CHAT.md](roadmap/ARIADNE_CHAT.md). `register_room_intrinsics`
adds the verbs to the action lanes. `ROOM` is the `room-join` handle (or its token), and
`CHANNEL` is `""` for the room or a peer's pid:

| verb | returns |
|---|---|
| `(chat-post ROOM TEXT [TO])` | `{ok}`, or `{ok #f error}`: TEXT is empty or longer than 2000 bytes once cleaned, or there is no relay to send it to. TO `""` posts to the room, a pid is a direct message. The relay may still refuse it (its rate limit, TO not in the room): `chat-info`'s `error` says so. |
| `(chat-info ROOM [CHANNEL])` | `{available rev count oldest newest head first more loading error}`. `rev` changes whenever the messages held change; `oldest`..`newest` is the contiguous run held (`count` of them); `head` is the next seq and `first` the oldest the relay keeps; `more` says older ones are kept; `loading` says a page is on its way. It also asks for the newest page the first time a channel is read, and again when a push was lost. |
| `(chat-messages ROOM [CHANNEL [AFTER]])` | the run, oldest first, as `{seq from name text t clock mine}` (`clock` is the local "HH:MM", `mine` is #t for your own), or with AFTER only those after that seq. |
| `(chat-more ROOM [CHANNEL])` | `{ok loading more}`: asks for the page before the oldest message held. One request per channel is in flight; one unanswered for 3 s is sent again. |
| `(chat-conversations ROOM)` | the direct-message conversations: `{peer name head present}`. |
| `(chat-rev ROOM)` | a number that changes with every push the relay makes to you. |

The shipped component `chat.ari` (§6.5) builds a chat window from these verbs, in the DSL:
- a channel picker, the room and each conversation, with what came since you last looked;
- a `text_view` that pages back when scrolled to the top;
- a `message_input`.

`room_users.ari` is the conference's users list. Its message button opens a conversation in the
chat window. `conference_roster.ari` (the document cyprty runs) is built from both:

```yaml
import: [cvc://components/room_users.ari, cvc://components/chat.ari]
windows:
  - include: room_users
    args: { id: users, rows: peer, chat: chat }
  - include: chat
    args: { id: chat, room: room.token }
on_tick: |
  (begin
    ...                                     ; join, media, avatar
    (room-users-step "peer" (state-get "conf.room") "chat")
    (chat-ui-step "chat" (state-get "room.token")))
```

`src/ari/tests/chat_component.ari` tests the components' logic under `ari test`. It stands in for
the room verbs with functions. `src/arigl/tests/chat_driver.ari` and `chat_two_clients.sh` run
scripted arigl clients through a local arisrv: one posts, a late joiner pages back through
everything, the two exchange direct messages, and a third checks it was pushed none of them.

### 5.17 `data:`: datasets in the state tree

A document names its datasets by URI. The host loads each one off the frame thread into one
shared place in the state tree, and the scene draws from there. The design and its later phases
are in [ARIADNE_DATA_ROADMAP.md](roadmap/ARIADNE_DATA_ROADMAP.md); the code is
[`data.h`](../inc/cvc/ariadne/data.h).

```yaml
data:
  city:                                             # the document's data.city
    uri: cvcpkg://cy-pca/scene-austin-south-small   # where it comes from
    path: share/cvc-scenes/austin_south             # the part of the package to keep
    as: city_bundle                                 # how to decode it
  notes: notes.json                                 # a bare URI: decoded by its extension
  big: { uri: https://example.org/big.tar.gz, load: lazy }

scene:
  physics: {}
  nodes:
    - node: city
      source: { data: city }          # the terrain and the buildings, drawn
      body: static                    # and in the physics world
```

**Sources.** A source can be any of these:
- a file or a directory, relative to the document;
- an `https://` URL;
- `cvcpkg://ORG/NAME[@VERSION]`, a cvcpkg data package;
- `state://PATH?data`, where the host registered the `state` scheme (`register_state_uri_handler`);
- any scheme a host registered (§6.4).

`https` and `cvcpkg` need the host's network: `--net-allow https://cvcpkg.org` (or
`AriRuntime.enable_net_allow([...])` in Python). A document that loads from the network lists
`net` in `requires:`.

**cvcpkg packages.** For a `cvcpkg://` source the host:
1. asks the registry (`CVCPKG_URL`, else `https://cvcpkg.org`) for the package's newest
   noarch build, or for `@VERSION`;
2. downloads its archive;
3. checks the archive against the registry's size and sha256;
4. keeps it in a cache keyed by that hash (`CVC_DATA_CACHE`, else the user's cache directory).

The next run reads the cache.

**Archives.** An archive is recognized by its bytes, whatever its name: tar (plain, gzip or zstd)
and zip (stored or deflate). It is unpacked in memory, and only its regular files are kept.
- `path:` keeps one directory of the archive, stripped from the paths, or names one file.
- A path that is absolute or climbs out with `..` fails the load.
- So does going past `sys.data.max_entries` files (default 100,000) or `sys.data.max_bytes`
  bytes (default 2 GiB).

**Where it lands.** The store is the `data` subtree at the app root, one node per request, so the
same request from two documents or a host is loaded once. The document's `data.<name>` is a
transparent link to that node, so `bind: data.city.status` and `(state-get "data.city.rev")` read
through it:

| Under `data.city` | |
|---|---|
| `status` | `idle`, `fetching`, `unpacking`, `decoding`, `ready` or `error`; the node's value is the same |
| `progress`, `error` | 0..1 while it loads; why it failed |
| `rev` | 1, 2, ... each time a load completes: what a watcher polls |
| `uri`, `canonical`, `sha256`, `size`, `as` | the request, what it resolved to (`cvcpkg://cy-pca/scene-austin-south-small@1.0.0+cvc.2`), the archive's hash and size, the decoder |
| `files`, `files.<i>` | the file count; each file's path, with its bytes on the node's `data()` |

**Decoders (`as:`).** Each decoder writes its outputs next to those keys. The default is `files`
for an archive or a directory, and otherwise one picked by the file's extension:

| `as:` | Writes |
|---|---|
| `files` | the files only |
| `bytes`, `text` | `content` (the bytes on `data()`), `text` (the value) |
| `json` | `json`: objects as children (a `.` in a key becomes `_`), arrays as `0`, `1`, ... with the count as the value |
| `mesh` | `geometry`: a `cvc::geometry`, from any format `read_geometry` reads |
| `volume` | `volume`: a `cvc::volume` |
| `heightfield` | `terrain` (a `cvc::height_field`) and `bounds.*`, from a `terrain.json` |
| `city_bundle` | `world` (a `world_geometry`), `terrain`, `buildings` (a `cvc::geometry`), `buildings_error`, `bounds.min_x` ... `max_y`, `satellite` (`satellite.png` decoded: the image on `data()`, its file path as the value) |

A host adds its own with `register_data_decoder`.

**Scene nodes.** A `source: { data: NAME }` node is an empty group at load. Once its dataset is
ready, the scene draws into it the first of the dataset's `world`, `geometry`, `terrain` or
`volume` that it has (`item:` picks one). It draws again whenever the dataset's `rev` changes, or
the document's link points at another dataset. What it drew before goes, and so do its bodies:
- a world's items become the node's children, its terrain becomes the node's height field (for
  `ground:`) and wears the dataset's `satellite` image (north at the image's top), and with
  `body:` its items are static bodies in the scene's `physics:` world;
- a mesh or a height field becomes one mesh node;
- a volume becomes a volume node.

`frame: true` (or `first`) frames the main view's camera on what the node drew, the first time it
draws, and `frame: always` does so each time. It writes the bounds to the camera's `frame` request
(`<prefix>.viewers.main.camera.frame`, "min_x min_y min_z max_x max_y max_z"), which any program
or host can write as well.

**Document logic.**
- `(data-load NAME URI [AS [PATH]])` points `data.NAME` at a new request and starts it. The
  nodes bound to `NAME` keep drawing the old dataset until the new one is ready.
- `(data-reload NAME)` loads it again.
- `(data-status NAME)` returns its status.
- `(data-files NAME)` lists its file paths.
- `(data-file NAME PATH)` returns one file's bytes, or nil.

**Hosts.** In C++, use `AppRuntime::data_store()`. In Python:
- `pycvc.data_load` and `data_wait` start and wait for a load;
- `data_files` and `data_file` read the files;
- `data_world`, `data_height_field` and `data_geometry` return the objects the scene draws, which
  share their arrays.

So a host reads `"<prefix>.data.city"` and never opens the bundle a second time.

### 5.18 Views of the scene: `type: viewport` and node `tags:`

A `viewport` widget shows the document's scene inside one of its windows, from a camera of its
own. It can show the whole scene, or only some of it, chosen by the scene nodes' `tags:`
(roadmap §9.5). An overview map, a second angle on the same scene, or a debug view of one layer
needs no host code. The `scene_view` component (§6.5) is the widget in a window of its own.

The demos in `examples/ariadne/documents/`: `scene_views.ari` (an overview map and a view whose
layers a menu picks), `viewer_grid.ari` (four cameras on one scene), `viewer_looks.ari` (one scene
drawn four ways), `viewer_scenes.ari` (three scenes from one set of assets) and
`viewer_groups.ari` (two tagged groups of one scene).

```yaml
scene:
  nodes:
    - node: city                  # untagged: drawn in every view
      type: geometry
      source: { file: city.glb }
    - node: agents
      type: geometry
      tags: [sim]
      source: { stream: { handler: agent_glyphs } }
    - node: nav_field
      type: geometry
      tags: [sim, debug]          # in a view that shows sim OR debug
      source: { file: field.vtk }
windows:
  - window: Overview
    id: overview
    pos: [16, 16]
    size: [380, 330]
    children:
      - type: viewport
        view: overview            # its camera: <prefix>.viewers.overview.camera
        show: [sim]               # the untagged nodes, and those tagged sim
        mode: map                 # orbit (the default) | fly | map | track
        width: 360                # logical px; the window's width when unset
        height: 270               # 3/4 of the width when unset
```

| Key | Meaning |
|---|---|
| `view` | The view's name. Its camera is `<prefix>.viewers.<view>.camera`, so `camera_settings` (`args: { viewer: <view> }`) and actions drive it as they do the main one. Defaults to the widget's `id`. |
| `show` | Tags, as a list or one string (`show: "sim debug"`). The view draws the untagged nodes and the nodes that carry one of these tags. When unset, it draws every tag. |
| `hide` | Tags the view leaves out, whatever `show` says. |
| `bind` | A state key whose value is the tag list to show (space- or comma-separated), read every frame. Use it for a layers menu: a widget writes the key, and the view follows. |
| `mode` | Its own camera's mode: `orbit` (the default), `fly`, `map` (top-down, framed on what the view shows), or `track` with `track: <node id>`. (`camera: map` and the other three mode names still mean the mode.) |
| `camera` | Draw from a named camera instead of its own: one the scene declares (`scene: cameras:`, below), or `main`, the main view's. |
| `scene` | The scene it shows: a `scenes:` entry's name (below). Unset (or `""`): the document's `scene:`. |
| `shadows` | `true` (the default) draws the scene's shadows from the maps the scene's own renderer bakes. `own` bakes maps of its own from what this view draws, so a node it leaves out casts nothing there and a node the main view leaves out still casts; it costs one more bake. `false` draws none. |
| `chrome` | `false` leaves out the scene's grid, axes and bounding box. Defaults to `true`. |
| `clear_color` | `[r, g, b]` in 0..1, what the view clears to (one colour, never the scene's gradient). Defaults to the main view's background. (`background:` on a widget is the §5.12 backdrop key.) |
| `azimuth`, `elevation`, `zoom` | Where its own orbit camera starts once the view is framed: the orbit's azimuth and elevation in degrees, and a zoom factor on the framed distance (2 is twice as close). |
| `override` | Per-view restyling, and moving, turning or scaling, below. |
| `caption` | A label in the view's top-left corner. |
| `input` | `false` makes the view ignore the mouse. Otherwise a drag orbits its camera (the middle or right button pans), the wheel zooms, and a double-click puts the camera back where it started. `pick` makes the left button pick instead of orbiting (below); the middle and right buttons still pan, and the wheel still zooms. The view is outlined while the pointer is over it. |
| `pick` | With `input: pick`, the state key a pick writes under (bound like `bind:`). Defaults to `viewers.<view>.pick`. |
| `on_pick` | With `input: pick`, an action run on each pick (a program, or a bare name for a host handler), with an `event` dict of the same fields. |

**Picking.** In an `input: pick` view, the left button picks what the view draws: a press, each
move while it is held, and the release. Each pick tests the scene's nodes as this view draws them
(an override's restyled copy counts as its node; the grid, axes and bounds never do). It writes,
under the `pick:` key:

- `x`, `y`, `z`: the world point it hit, empty for a miss;
- `hit`: `1` or `0`;
- `node`: the id of the node it hit;
- `button` (`1`), `mods` (`shift`, `ctrl`, `alt`, `super`, comma-separated), `phase` (`down`,
  `drag` or `up`) and `view`;
- `seq` last: one more on each pick, so a watcher sees two identical clicks.

`on_pick:` then runs with those fields in `event`. `(get-attr event "node")` is the node, for
example, so a tool such as a spawn-point editor is a program. A host can ask for the same hit test
at any point with `SceneViews::pick(view, x, y, hit)`. A terrain, a generated world and a body are
nodes like any other, so a click on the ground gives a point on it. (`cvcgl_ariadne_pick`.)

**Tags.** A node's `tags:` hold for its whole subtree, so a tagged child under a tagged parent is
drawn only where both are allowed: the parent's tags take the child out of a view that rules them
out. An untagged node is in every view, so content meant for the main view only needs a tag of its
own that the other views leave out (`hide: [main_only]`). The main view always draws every node. A
node hidden with `visible:` is gone from every view, and so is everything under it: its children,
the ones a `source: { data }` node draws when its dataset arrives included, keep their own flags
but draw only while every ancestor is visible. A `generated_world` node's meshes and water beside
it follow it too.

**Overrides.** `override:` restyles nodes in one view only. Each entry names a node id, a tag, or
`"*"` (everything), and sets any of `drawstyle` (`surface`, `wireframe`, `points`), `opacity`,
`color`, `line_width` and `point_size` (`material: { color, opacity }` spells the same fields).
What an entry does not set stays as the node has it. An entry covers the named node's subtree, and
the entries apply in order, each adding to the ones before it field by field:

```yaml
- type: viewport
  view: xray
  override:
    "*": { opacity: 0.5 }                                  # everything half see-through,
    buildings: { opacity: 0.15 }                           # the buildings more so,
    sim: { color: [1.0, 0.9, 0.15], opacity: 1.0 }         # and the agents solid yellow
```

An entry with `mode: replace` starts over from the node's own look: the entries before it no longer
apply to the nodes it covers (`"*"`'s included), and the ones after it add to it as usual.

An entry's `transform: { position, rotation, scale }` moves, turns and scales what it covers in
that view, spelled as a node's `transform:` is (rotation in degrees; `scale` one number or three).
It applies in the frame of the node the entry names, so `pack: { transform: { scale: 2 } }` draws
the group `pack` twice its size about its own origin, children and all. A tag entry applies in each
tagged node's frame, and `"*"` in world space. Transforms from several entries compose in order.
The main view and every other view keep the node where it is.

```yaml
- type: viewport
  view: exploded
  override:
    "*": { opacity: 0.5 }
    roof: { transform: { position: [0, 0, 6] } }                  # lifted off the house
    agents: { transform: { scale: 3 } }                           # each node tagged agents, 3x
    ground: { mode: replace, material: { color: [0.2, 0.2, 0.2] } }  # opaque: "*" no longer applies
```

A restyled actor is drawn in that view by a twin: a second `vtkActor` with its own `vtkProperty`
over the node's own mapper. The geometry, the shader replacements and the texture are shared, not
copied. The twin follows the node's transform, visibility and material every frame, so a node that
moves, changes colour or swaps LOD rung keeps its override.

**Volumes in views.** A `type: volume` node (VTK's GPU raycaster) draws in every view as it is, and
takes `opacity` (every point of its opacity function scaled) and `transform`. A restyled one is a
second `vtkVolume` with its own property and its own mapper, which uploads the volume a second
time: a GPU volume mapper keeps one set of transfer-function tables, for the property it last drew.
A `volslice` node's slices face the camera they were swept for. A view from a camera of its own
draws slices of its own, swept for its camera, over the node's shaders, textures and transfer
function (`VolSliceNode::viewProp`); a `camera: main` view draws the node's own. A `volren` node's
picture is raycast for the main view and composited into its pixels, so it is in the main view only.
`examples/ariadne/documents/viewer_looks.ari` draws one scene four ways: wireframe, X-ray, a plan of
one layer, and a clay model. `viewer_grid.ari` is four independent cameras on one scene.

**Cameras.** A view's camera is its own unless it names one. `scene: cameras:` declares named
cameras, each at `<prefix>.viewers.<id>.camera`, with the same `mode`, `track`, `azimuth`,
`elevation` and `zoom` keys. Every viewport with `camera: <id>` draws from that camera, so a drag,
the wheel or a double-click in any of them moves all of them, whatever each one shows or restyles.
`camera: main` follows the main view's camera, and moves it.

```yaml
scene:
  cameras:
    - camera: plaza
      azimuth: -60
      elevation: 30
      zoom: 1.3
windows:
  - window: Looks
    children:
      - { type: viewport, view: wire, camera: plaza, override: { "*": { drawstyle: wireframe } } }
      - { type: viewport, view: xray, camera: plaza, override: { buildings: { opacity: 0.2 } } }
```

A named camera is framed on the whole scene the first time a view shows it, and its near and far
planes span the whole scene. Named cameras and views share the `viewers.*` names, so give them
different ones.

**Shadows.** When the scene casts shadows (`shadows: { enabled: true }`), every view draws them. A
view does not bake shadow maps of its own: it reads the ones the main view bakes
(`SceneGraph::makeShadowReceiverPass`). They are in light space, so they hold from any camera. A
view that leaves part of the scene out can bake its own instead (`shadows: own`,
`SceneGraph::makeShadowPasses`). A twin a `transform:` moves casts where the node is in shared maps,
and where the twin is in its view's own (`shadows: own`).

**Assets.** `assets:` names meshes, each a node's `source:` and `fit:`, that any node in any of the
document's scenes draws with `source: { asset: <id> }`. Each asset is read and fitted once. Every
node on it shares its geometry's buffers (`cvc::geometry` is copy-on-write), and the VTK arrays
built from them too: the asset's first node builds its points, cells, normals and texture
coordinates, and the others draw from the same arrays (`GeometryNode::shareArraysFrom`), so VTK
uploads them to the GPU once. A node that writes into its points or normals in place
(`updateVertices`, `updateNormals`) first gets arrays of its own; the others keep the shared ones.
Each node keeps its own transform, material, colours, tags and VTK actor.

```yaml
assets:
  bunny: { source: { file: stanford.bunny }, fit: { height: 4, up: y } }
  pillar: { source: { box: { size: [1.2, 1.2, 6] } } }
scene:
  nodes:
    - { node: statue, type: geometry, source: { asset: bunny } }
    - { node: post, type: geometry, source: { asset: pillar }, transform: { position: [7, 7, 3] } }
```

**Scenes.** `scenes:` declares scenes of their own beside the document's `scene:`. Each entry takes
the keys `scene:` does: nodes, lights, shadows, background, chrome, cameras. A viewport shows one
with `scene: <id>`, from its own camera or a named camera that scene declares. Each scene is a
SceneGraph of its own, under state `<prefix>.scenes.<id>`, so its nodes are at
`scenes.<id>.graphics.root.children.<node>`. Its nodes' `visible:` binds and the widgets share the
document's prefix. It gets a viewport of its own (`scene.<id>`), never shown, which holds its
lights, background and shadow chain; the views of it mirror that viewport. While its shadows are on
and one of its views shows, that viewport draws one pixel in a corner of the view, under it, so its
maps are baked. While none shows, nothing bakes them. On the first frame one shows again, they are
baked for whatever moved meanwhile, whatever the scene's `update_interval`, so a view never draws
stale maps. A document may have only `scenes:`, and then the main view is empty.
`examples/ariadne/documents/viewer_scenes.ari` has three scenes (day, night, a studio) built from
the same assets.

**Groups in one scene.** Where the "scenes" should share the lights and the shadows, make them
tagged groups of one scene. `main_view: { show: [...], hide: [...] }` in `scene:` picks the groups
the main view draws, as a viewport's `show:` / `hide:` do, and the main camera is framed on them.
The props of the groups the main view leaves out come out of the main renderer for the length of
its draw, so the views, which mirror it, still have them. They cast no shadow in the main view's
maps either, so a view that shows them uses `shadows: own`.
`examples/ariadne/documents/viewer_groups.ari` shows a courtyard in the main view and a gallery in
a viewport.

**How it draws.** Each view is a mirror viewport of the main one
(`ViewportManager::addMirrorViewport`). It draws the same props, the same geometry, from its own
camera, less the props of the nodes it leaves out and with its twins in place of the props it restyles
(`Viewport::setMirrorFilter`). The widget
reserves its rect in the window and cuts that rect out of the window's background
(`ImGuiBackend::cut_window_background`); the view is drawn there. So the view moves and scrolls with
its window, is clipped where the window clips it, and is hidden on any frame its widget is not
drawn: a closed or collapsed window, another tab. Every AriRuntime host (arigl, pycvc_gl) has the
widget. A terminal backend shows its placeholder. A new document removes the last one's views.

Limits:
- A `volren` node is in the main view only. A `volslice` node takes `transform` but no restyling;
  a `type: volume` node takes `opacity` and `transform`.
- The view lies under the whole UI. Another window behind the viewport's window still draws over
  the view.
- A view's shadows come from the main view's maps, unless it bakes its own. A node a view leaves
  out still casts its shadow there, and with `shadows: { focus: camera }` the maps follow the main
  camera, not the view's.
- A view's first frame is opaque: the window's background is cut from the frame after the widget
  first draws.

### 5.19 Clickable groups: `on_click:` on a group, `hover: highlight`

A group with an `on_click:`, `on_hover:` or `on_hover_end:`, or with `hover: highlight`, is one
item over its whole area. A feed's card opens its post wherever it is clicked:

```yaml
- type: group
  repeat: (state-get "feed.items")
  hover: highlight                          # lit while the pointer is over it
  on_click: (state-set "view.post" "{i}")
  on_hover_end: (state-set "feed.peek" "")
  tooltip: Open the post
  children:
    - text: ""
      bind: feed.items.{i}.text
    - image: ""
      bind: feed.items.{i}.media.0.url
      on_click: (state-set "view.picture" "{i}")   # the picture's own: not the card's
    - button: Like
      on: (state-set "feed.items.{i}.liked" "1")  # the button's too
```

- A click on the group's text, pictures or empty space is the group's. A widget inside it that
  takes clicks itself keeps them: a button, a checkbox, a field, or a widget with an `on_click:` of
  its own. The innermost one wins, so a clickable group inside another keeps its clicks.
- `on_hover:` runs every frame the pointer is over any part of the group, the widgets in it
  included. `on_hover_end:` runs once when the pointer leaves, or when the widget stops being drawn.
  It has no `event`. `on_hover_end:` works on any widget, not only a group.
- A `tooltip:` shows over the whole group. In the `event` of `on_click:` and `on_hover:`, `x` and
  `y` are measured from the group's top-left.
- **ImGui** (arigl, the browser page): a click is a tap. It must be released over the group, and the
  pointer must not have moved further than ImGui's drag threshold, so a finger scrolling a feed
  opens nothing. The group's area is as wide as the room it has (a card's whole row), however
  narrow its content. `hover: highlight` fills that area under the content with the header-hover
  colour, or the header-active colour while it is pressed.
- **Terminals** (FTXUI, tvision): a group with `on_click:` takes keyboard focus. Tab reaches it
  before the widgets inside it, and Enter or Space fires it. A left click fires it and focuses it.
  It has a one-column gutter on its left:
  - `▌` and a grey background while it has focus;
  - `│` while the pointer is over it, with `hover: highlight`;
  - blank otherwise.

  On a terminal with no block glyphs (the Linux console, `dumb`, `vt*`) the marks are `>` and
  `|`. A monochrome terminal gets no grey. The gutter column is always there, so focus never shifts
  the layout. A focused group scrolls into view like any focused widget.
- `(has? "ui.group_click")` holds where the backend does all this (ImGui, FTXUI, tvision). On
  another backend, a group's `on_click:` is still its last widget's.
- In a `repeat:`, `{i}` reaches the group's handlers (§6.5). `ariadne_ftxui_test`'s
  `GroupClick*` and `GroupHover*` tests drive a repeated card list by mouse and keyboard, and
  `cvcgl_imgui_group_click` taps, drags and hovers one through ImGui's own input.

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

Args are strings. A unit takes whole widget lists through **slots**: a `- slot: NAME` item in
any of its widget lists marks where they go. The include site's `slots: {NAME: [...]}` puts its
widgets there, in place of the item; a single widget map works too, and a slot left unfilled is
dropped. The include site's widgets are its own: the unit's args are not substituted into them,
though a `repeat:` around the include still substitutes `{i}`.

```yaml
units:
  framed:
    args: { title: Untitled }
    type: group
    children:
      - text: "{title}"
      - slot: body
      - slot: footer
windows:
  - window: W
    children:
      - include: framed
        args: { title: Hello }
        slots:
          body: [ { text: first }, { button: Go, on: go } ]
```

### 6.2 `import:` — shared-scope library merge

`process_imports` ([`loader.cpp:761`](../src/cvc/ariadne/loader.cpp)) resolves a URI (or
list) through the shared resolver relative to the document's `base_dir`, merges another
`.ari`'s `units:` into scope, and recurses into that library's own imports. It dedups
and cycle-guards on the resolved `canonical` identity
([`loader.cpp:789`](../src/cvc/ariadne/loader.cpp)), capped at `kMaxImportDepth = 32` /
`kMaxImports = 4096` ([`loader.cpp:758`](../src/cvc/ariadne/loader.cpp)). Local units
override imported ones. This is how a document composes a library of shipped panels.

An entry may be `{uri: URI, optional: true}`: a library the host may not have (a component
newer than it), whose absence is no warning, so `ari test` does not fail on it. A document's
own unit marked `fallback: true` stands in for a unit of that name only where no import defines
it, so a document adopts a shipped component and still runs on hosts without it:

```yaml
import: [{uri: cvc://components/card.ari, optional: true}]
units:
  card: {fallback: true, type: group, children: [...]}   # used where card.ari is not there
```

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

A module's own programs are its `init:`, its `on_tick:` and its widgets' actions. What they need
from the host goes in the module's own `requires:` (§5.7). The loader
captures `init:` and `on_tick:` onto the mount wrapper (`Widget::init_script`,
`Widget::tick_script`), and the runtime runs both at the sub-prefix with the mount's functions:
`init:` once, `on_tick:` as the mount's own tick resident (§2.3). So a module can own a loop, a
request poller say, and be tested on its own with `ari test` (§13).

`on_key:`, `on_pointer:` and `on_signal:` stay the outermost document's. A fragment that declares
one loads with a warning, and the handler is ignored. Raw input and signals belong to the
document, not to whatever it mounts: a module that saw every key could read what is typed into
the parent's fields, and two modules could not both decide what Ctrl-C does. A module reacts to its
own widgets. When it needs more, the parent handles the event and passes on what the module needs
through a `link:` hole, which the module's `on_tick:` can read.

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
includes. The `libcvc` cvcpkg bundles and the release tarballs carry the directory from 3.7.10.
Earlier native bundles left it out, so with those a host needs `CVC_ARIADNE_PATH` pointing at a
copy (a libcvc checkout's `src/cvc/ariadne`). Each binds state keys a C++ object owns, so it needs no host code;
[`INDEX.ari`](../src/cvc/ariadne/components/INDEX.ari) catalogs them:

| component | drives |
|---|---|
| `viewer_settings` | a viewer (arg `viewer`, default `main`): the frame-rate readout, UI size, grid / origin axis / scene bounds, camera and shadows in one window |
| `stage_lighting`, `scene_controls`, `camera_controls` | a `rig:` light, `shadows:`, a viewer's camera (arg `viewer`) |
| `display_toggles`, `sim_transport`, `raster_view` | `show.*` layer keys, `sim.*` + host verbs, a host-published image (arg `keys`, default `raster`) |
| `volren_controls`, `volslice_controls` | a `volren` / `volslice` node (arg `node`, default `bunny_volume`) |
| `forest_controls`, `sea_controls`, `sky_controls` | a `forest_trees` / `wave_sea` / `cloud_sky` node (arg `keys`, below) |
| `world_controls` | a `generated_world` node (§5.4; arg `keys`) |
| `mesh_tools` | a `mesh_lab` node (§5.4; arg `node`, default `mesh`) |
| `fe_controls` | an `fe_lab` node (§5.4; arg `node`, default `domain`) |
| `scene_view` | a window holding a view of the scene (the `viewport` widget, §5.18), from a camera of its own or a named one (args `view`, `scene`, `show`, `hide`, `mode`, `camera`, `chrome`, `shadows`, `caption`, `width`, `height`, `title`, `id`) |
| `card` | a card in a list: avatar / title / subtitle / timestamp header, body, `media` and `actions` slots; its args are state keys, so a `repeat:` includes one per item (not a window: embed it) |
| `transfer_function`, `transfer_function_editor` | any node's `transfer_function` model (args `node`, `id`): the ColorTable2 editor with histogram / contour spectrum / contour tree layers — see [`TRANSFER_FUNCTIONS.md`](TRANSFER_FUNCTIONS.md) |
| `chat`, `chat_panel` | a room's chat and direct messages through the chat verbs (args `id`, `title`, `room`); its functions come with the import: call `(chat-ui-step ID ROOM)` from `on_tick` (§5.16) |
| `room_users`, `room_users_compact` | everyone in a room from `room-roster`, with local cam / mic / volume and a message button (args `id`, `title`, `rows`, `chat`, `self`); call `(room-users-step ROWS ROOM CHAT)` from `on_tick` |

Each window component also takes `title` and `id` args, and every arg has a default, so a bare
`include:` works. A document with two nodes of a kind includes the panel twice, each with its
own `node` (or `keys`) and `id`; the window's geometry persists under `tree.<id>`. The forest,
sea, sky and world realizers keep their live keys scene-wide (`forest.*`, `sea.*`, `sky.*`,
`world.*`). A node's `keys:` prop moves them under a root of its own, and the panel's `keys` arg
names the same root:

```yaml
import: cvc://components/forest_controls.ari
windows:
  - include: forest_controls                           # forest.*: the south wood
  - include: forest_controls
    args: { keys: north, title: North wood, id: north_forest }
scene:
  nodes:
    - { node: south, type: forest_trees, count: 40 }
    - { node: north, type: forest_trees, count: 80, keys: north }   # north.count, north.seed, ...
```

`camera_controls` and `scene_controls` also ship their widgets without the window
(`camera_settings`, `shadow_settings`); `viewer_settings` is built from them.

The `card` is a generic social or inbox card (§5.11 pictures, slots in §6.1):

```yaml
import: cvc://components/card.ari
windows:
  - window: Feed
    children:
      - type: group
        repeat: (state-get "feed.items")
        children:
          - include: card
            args: {id: post_{i}, avatar: feed.items.{i}.author.avatar,
                   title: feed.items.{i}.author.name, subtitle: feed.items.{i}.author.handle,
                   time: feed.items.{i}.when, body: feed.items.{i}.text}
            slots:
              media:
                - image: ""
                  bind: feed.items.{i}.media.0.url
                  visible_when: (!= (state-get "feed.items.{i}.media.0.url") "")
                  size: [440, 248]
                  fit: cover
              actions:
                - { button: Reply, on: (state-set "compose.reply_to" "{i}") }
```

Inside a `repeat:`, `{i}` is the instance's 0-based index. It is filled in wherever an instance
names state or shows text: `id`, `label`, `bind`, `on`, the pointer handlers (`on_click`,
`on_hover`, `on_drag`, `on_drag_start`, `on_drag_end`), a `text_view`'s `on_top`, a
`message_input`'s `hint` and `send`, the `*_when` predicates, `tooltip`, `badge`, `options`,
`src`, `status`, `poster`, `emoji`, a `background:`'s `src`, `bind` and `status`, and a `load:`
mount's `link:` holes. A nested `repeat:` keeps its own `{i}`: the outer index does not reach
into it.

Slots `note` (a row above the header), `extra` (lines under the byline, in the header's text
column) and `body` (widgets after the body text) take more of a post's parts. A part whose key
is empty is not drawn. The avatar is a circle `avatar_size` px across (40),
loading through `avatar_auth`'s credential where its URL needs one. A one-column grid with an
outer border frames the card: an ImGui table, a box in a terminal.

Most components that address one node use a fixed node id; copy the file and change the id to
drive another node. `transfer_function` instead takes the node as an include arg (§6.1). A
component may also carry `functions:` (§6.6), which the import brings with its units: `chat` and
`room_users` keep their windows current that way, from the document's `on_tick`.
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

- `rw`: a write under the hole (`state-set`, `state-set-tree`, `state-data-set`,
  `state-from-json`, the atomics, a widget commit) lands on the target's descendant, created
  when absent.
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
**after** the holes are wired, with `(has? ...)` as the document's own `init:` has it and
with the module's functions (§6.6). The mount's `on_tick:`
resident starts right after it (§2.3). A reconcile keeps that resident running when the new tree
has the same mount at the same sub-prefix with the same program (its body and its functions), as
a reload of an unchanged document does, and stops it otherwise. A repeated mount's resident runs
per instance and restarts at a reconcile. An instance stops when its outermost `repeat:` count
drops below it (a nested repeat's count is not watched: its instances stop at the next
reconcile). A module's functions come
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
| `arisrv` | libcvc + gRPC + WebSocket | headless, as a server (the conference relay + WS↔gRPC bridge); with `--doc`, a server document on the relay (§9.7) | [`src/arisrv`](../src/arisrv/main.cpp) |
| `arix`, `ariglx` | `ari`, `arigl` + an embedded CPython (pycvc, pycvc_gl, numpy) | as `ari` and `arigl`, plus a document's `python:` block and a Python program in place of the document (§9.6) | [`src/ariadne_python`](../src/ariadne_python/python_engine.h) |
| `arisrvx` | `arisrv` + an embedded CPython | as `arisrv`, plus the operator's Python program beside the relay, serving ASGI apps on a script thread of its own (§9.8) | [`src/arisrv`](../src/arisrv/main.cpp), [`src/ariadne_python`](../src/ariadne_python/server_link.h) |

Each is its own cvcpkg package, built from its recipe in [`cvcpkg/recipes`](../cvcpkg/recipes)
against the installed libcvc: `cvcpkg install ari`, `cvcpkg install arigl`, and for a room's
server `cvcpkg install arisrv` (linux, macOS and windows). arisrv's transports, its Jolt avatars
and its libjpeg-turbo video rungs are libcvc's, so it needs a libcvc built with them: every
libcvc after 3.7.8 is. Its planes are gRPC, with WebSocket on top for browser clients, so it needs
`CVC_ENABLE_GRPC`. A libcvc build without gRPC leaves arisrv out and says so, and arisrv's
standalone configure refuses an SDK without gRPC. For a build of your own, gRPC comes from cvcpkg:
`cvcpkg install grpc protobuf --prefix <p>`, with `<p>` on `CMAKE_PREFIX_PATH`.

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
  document the Controls window (§5.3), even one with no keys of its own. `set_host_mouse(m)`
  adds the host camera's pointer settings to that window, above the keys.
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
`--component-path DIR`, `--log FILE` for the run's warnings as they happen (§2.4), and the snapshot
ones) go **before** the document. They are the
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

**`ari-media`** is the same program with sound: cvcMedia, libcvc's GL-free media library
([`src/cvcMedia`](../src/cvcMedia/CMakeLists.txt)), is linked in. It brings up SDL3's audio
subsystem alone, with no window, display or SDL video, and still no GL, VTK or ImGui, so it
plays sound in a terminal; `ari-media info` lists the playback devices. The lean `ari` links
libcvc only, with no SDL. `ari-media` is its own package
([`cvcpkg/recipes/ari-media`](../cvcpkg/recipes/ari-media/recipe.yaml), depending on `sdl3`),
a separate binary name, so the two never install the same file and a launcher can prefer
`ari-media` where it is installed. In a libcvc build it is built with cvcGL or with
`-DCVC_BUILD_ARI_MEDIA=ON`. cvcMedia holds the SDL3 audio output that used to be cvcGL's
(`cvc::media::playback`; cvcGL links cvcMedia and keeps the `cvc::gl::playback` names), and
its FFmpeg player plays the `video:` and `audio:` widgets (§5.13). The `ari-media` package also
depends on `ffmpeg-lgpl`, linked shared.

`ari` is its own cvcpkg package (`cvcpkg install ari`, linux for now), built from
[`cvcpkg/recipes/ari`](../cvcpkg/recipes/ari/recipe.yaml) against the installed libcvc,
like `cvc-cli`. It is a thin host over `AppRuntime` with no `SceneAdapter`. Its
`register_quit()` routes `on: quit` and the `(quit [code])` verb to `request_close(code)`, and
`run`, `snapshot` and `check` exit with `exit_code()` (§13). An OS signal the document does not
handle ends it with exit status 128 + the signal number. Embedding the same thing:

```cpp
cvc::ariadne::FtxuiBackend backend;          // outlives the AppRuntime
cvc::ariadne::AppRuntime rt(app, prefix);    // no SceneAdapter: a scene: is not drawn (its world runs)
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

### 9.3 Writing a Python host

A Python program can be a document's host: it opens the document, the document calls into it,
and the two pass data both ways. `pycvc_gl.ariadne.Host`
([`pymod_gl/ariadne.py`](../bindings/pycvc/pymod_gl/ariadne.py)) is that host; the programs in
[`examples/ariadne/python`](../examples/ariadne/python) are the place to start:

| Example | The C++ host it mirrors | What it shows |
|---|---|---|
| [`hello.py`](../examples/ariadne/python/hello.py) | `ariadne_hello` | a button calls a Python function, another raises an event Python handles; Python writes the document every frame |
| [`data_feed.py`](../examples/ariadne/python/data_feed.py) | `ariadne_stream` | Python pushes two plotted series, a table, a numpy picture and a live video stream, and reads the document's controls |
| [`form.py`](../examples/ariadne/python/form.py) | `nav_compute` | a two-way form: a function returns a dict the document writes with `state-set-tree`, an async function runs a Monte Carlo while the UI keeps drawing, a watch recomputes a preview |
| [`nav.py`](../examples/ariadne/python/nav.py) | `nav_convoy`, `nav_city_drive` | a Python loop drives Humvees on a `cvc::nav` sim_world (pycvc) through Austin, or a block city when its bundle is not installed: true-scale models, routes planned for their turning radius, Jolt physics or nav's kinematic bicycle (the Plant combo), a chase or overview camera, readouts and a map picture |

Each takes `--offscreen` (GL with no window), `--headless` (no GL), `--frames N` and
`--png PATH`; off a window it presses its own buttons and prints what the document shows.

For a controller of your own driving N Ackermann cars, start from `pycvc_gl.cars`: a template
(`python -m pycvc_gl.cars --copy DIR` writes `cars.py` and `cars.ari`) whose document loads Austin
through its `data:` block, picks the plant with `nav_plant.vehicle`, binds the navigator's settings,
and runs headless too. [CARS.md](CARS.md) describes it.
`nav.ari` has a `scene:`, so it requires `gl.scene` (§5.7) and `nav.py` has no headless run.

`nav.py` finds Austin with `pycvc_gl.scenes.find_scene()` (`$CVC_NAV_BUNDLE`, else
`<prefix>/share/cvc-scenes/austin_south`: `cvcpkg install scene-austin-south-small
vehicle-humvee`) and the Humvee with `find_vehicle_model()`. `pycvc_gl.vehicle.VehicleModel`
makes a glTF vehicle +X forward, +Z up and true to scale (as the C++ demos'
`load_vehicle_template` does), and `seat_transform` / `body_transform` place it from a heading or
a physics body's pose. Its planning grid is `pycvc.nav_world_rasterize(city.world(), rows, cols,
...)`, cvc::nav's rasterizer (no VTK), which also gives the terrain's height at each sample
(`pycvc.nav_world_grid(world, cell)` is the same grid walled at its edge, with its signed distance
and free points; `AriApp.grid(node, cell)` gives it for a scene's generated or data node): the
navigator gets the grade (`nav_sim_world_set_terrain` with the Humvee's
`nav_plant.terrain_config_of`), and the routes keep off the slopes `nav_terrain_build` marks
infeasible for it. The Jolt plant drives on flat ground with the buildings as boxes, as the C++
city demos do; `--relief` puts it on the city's own terrain and building mesh
(`pycvc.nav_plant(..., ground=field)` starts each vehicle on the ground under it). There the grade
term halves the rollovers but does not end them: the limit it applies is the climb, and a Humvee
cornering on a 10-25 degree side slope at 5-9 m/s can still roll (4 of 16 in 96 s of Austin).

Its `scene:` block is `terrain_lab.ari`'s soft afternoon look (§5.4): a pale sky gradient, a
low warm `three_point` sun with a cool fill, and shadows that follow the camera with soft edges
(`shadows.softness`, the shadow filter's radius in texels). The city's materials are matte (low
specular), set from Python on the nodes it adds.

```python
from pycvc_gl.ariadne import Host

host = Host(mode="window")       # "window", "offscreen" (GL, no window) or "headless" (no GL)

@host.function("greet")          # (greet NAME) in any of the document's programs
def greet(name):
    return f"Hello, {name}!"     # numbers, str, list, dict, None (and numpy) go back

@host.async_function("solve")    # (solve ...) parks its program; this runs on a pool thread
def solve(a, b):
    return heavy(a, b)

@host.on("reset")                # a button or menu item with `on: reset`
def reset():
    host.set("count", 0)

@host.on_frame                   # every frame, after the document's programs, before the draw
def tick(dt):
    host.set_series("signal", samples)   # a plot: draws it

host.load("doc.ari")             # register first: requires: is checked here
host.run()                       # until the window closes or the document quits
host.close()
```

**Keys.** A key is relative to the document's state prefix, as a `bind:` is (§7): `host.set("a.b",
1)` writes what `bind: a.b` shows. A key starting with `/` is app-root absolute. The prefix is
the host's `name` (`Host(name=...)`, `"ari"` by default).

**Python to the document.**

| Call | What the document sees |
|---|---|
| `host.set(key, value)` (also `host[key] = value`) | the value's text: an int, a float (shortest round trip), `"1"`/`"0"` for a bool, `""` for None |
| `host.set_tree(key, value)` | a subtree, as `(state-set-tree)` writes one: dict keys and list indices name children, a list's node holds its length |
| `host.set_table(key, rows, columns=None)` | rows at `key.<i>.<column>` and the row count at `key`, for `repeat: (state-get "key")` (column names at `key_columns`) |
| `host.set_series(key, values, limit=None)` | the comma-separated numbers a `plot:` draws |
| `host.publish_image(name, array)` | the picture `image:` widgets with `src: name` show: (H, W), (H, W, 3) or (H, W, 4), uint8 or float in [0, 1], row 0 at the top |
| `host.open_stream(id, w, h)` | a live video stream at `streams.<id>` for a `stream_view` widget or a `type: stream` node; `stream.publish(frame)` sends one |
| `host.run_program(script, done=None)` | a program queued as one of its actions (`done(how)` gets `""` or why it did not finish) |
| `host.emit(event)` | the event a button raises: its `on()` handler runs in the next frame |

**The document to Python.** `host.on(event)` handles a bare `on: event`.
`host.function(name)` and `host.async_function(name)` make a Python callable a function the
document's programs call with arguments (`pycvc.host_function` builds it; values convert as
`pycvc.Exec`'s do, §10.1). A synchronous function runs inline in the frame. An async one is an
async function on the app's async host
([STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md) section 3): its program parks while it
runs on the app's background task pool, `(name-async ...)` returns a future,
`with_call=True` passes the call first (`call.cancelled()`, `call.post(value)`), and
`done_channel` also posts its value there. Their exceptions are the document's: the call raises
a `PythonError` (its `python_type` names the Python exception), which a `try` catches.

**Reading back.** `host.get(key, default)`, `get_float`, `get_int`, `get_bool` (which reads
`"1"`/`"true"`/`"yes"`...), `get_series` (a numpy array), `get_tree` (the inverse of `set_tree`),
`host.watch(key)` (a callback with the new value after a frame that changed it; `tree=True`
for a subtree), `host.warnings`, and the frame: `host.snapshot(path)` (PNG) and
`host.frame_rgb()` (an (H, W, 3) array).

**The scene.** In a GL mode `host.scene` is the document's `SceneGraph`, so Python can add its
own nodes beside the ones the `scene:` block realizes: `host.lab` is a `pycvc_gl.lab.Lab` over
it (meshes, markers, paths, fields), and `host.frame_bounds(lo, hi)` aims the camera.
`nav.py` draws its agents and tracks that way.

**Modes.** `"window"` needs a display. `"offscreen"` renders into an offscreen window: a capture,
or CI on a software GL (VTK picks GLX, EGL or OSMesa at run time; the ImGui overlay needs GLX, so
CI runs it under Xvfb, see `pycvc-linux` in ci.yml). Its scene and UI clocks step by each frame's
`dt`, so its frames are reproducible. A view whose shorter side is under 700 logical pixels
starts with the windows hidden, as on a phone (§5.14), so captures use 1024 x 768 or more. `"headless"` has no GL at all: a `pycvc.AriApp`, an
`AppRuntime` with no backend and no scene. The document's `init:`, residents, actions, host calls
and state all run, but nothing is drawn, so a computed `bind:` is never evaluated, a `scene:` is
dropped with a warning, and a document that `requires: gl.scene` does not load. Tests and CI use
it.

**Threads and the GIL.** Handlers, functions, `on_frame` hooks and `watch` callbacks run on the
thread that calls `step()`/`run()`; an async function runs on a pool thread. An async function
may compute and write state, but the scene and the pictures (`publish_image`, `host.lab`,
`host.scene`) belong to the frame's thread: hand its result to an `on_frame` hook. A frame releases the
GIL while C++ runs (so do `load()`, `wait_idle()` and the destructors of the runtime and the
app), and each Python call takes it back, so an async function or another Python thread runs
while the frame does. A callback may read and write state and queue programs, but not run a
frame: `step()`, `run()`, `frame()` or `render()` from inside one raises `RuntimeError`
(`AriRuntime` refuses it too), and `load()` from inside one waits for the frame to end.
`close()` waits for the async functions still running, so a long one should watch
`call.cancelled()`.

**Errors.** An exception in an `on()` handler, an `on_frame` hook or a `watch` callback is raised
from the `step()`/`run()` that ran it. A function's is the document's (above); one it does not
catch stops that program with a warning, logged through `logging` (`pycvc_gl.ariadne`), and with
`Host(strict=True)` raised from `step()` as an `AriadneError` whose cause is the Python
exception: what a test wants.

**The caps.** Ariadne caps each action at 0.1 s and 5000 steps (§2.2), sized for a document's
own code, and a Python function's run time counts against the time cap (an async one's CPU time
as steps too). A Python host raises them: `Host` sets `ariadne.limits.action_max_time` to 30 s
and `action_max_steps` to 0 (no limit) unless `limits=` says otherwise (`limits={}` keeps
Ariadne's). An async call parks its program, so a long one costs the UI nothing.

**Underneath.** `Host` composes the bindings, which a program can use directly:
`pycvc_gl.AriRuntime(view, cam, ui)` ([`AriRuntime.h`](../inc/cvc/gl/ariadne/AriRuntime.h)) in a
GL mode and `pycvc.AriApp(app, prefix)` headless, both with `load`, `frame`, `render`, `step`,
`run`, `on`, `register_host_function(pycvc.host_function(name, fn, is_async=False,
with_call=False, done_channel=""))`, `register_quit`, `run_program`, `take_warnings`,
`wait_idle`, `has_function`, `prefix`; `AriRuntime` adds `publish_image`, `enable_streaming`,
`set_fixed_clock`, `post_key`, `post_pointer` and `reload_if_changed`. The older
`register_verb(name, fn)` (no arguments, no value) and `register_async_verb(name, fn)` (no
arguments, `str()` of the value) still work. In C++ the same functions are
`AppRuntime::register_function(name, native_fn)` and `register_async_function(name, marshal,
done_channel)`.

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

### 9.6 The scripting hosts: `arix` and `ariglx`

`arix` and `ariglx` are `ari` and `arigl` with an embedded CPython
([`src/ariadne_python`](../src/ariadne_python/python_engine.h); the plan for the rest, the browser
and `arisrv`, is [roadmap/ARIADNE_SCRIPTING_HOSTS.md](roadmap/ARIADNE_SCRIPTING_HOSTS.md)). They
take the same commands and flags, and add:

- a document's `python:` block, run at load;
- a Python program in place of the document: `ariglx run app.py`;
- `<host> python [args]`, the interpreter as a plain `python3`;
- `--python-path DIR` (repeatable): DIR on the interpreter's `sys.path`, as `PYTHONPATH` puts it.

The interpreter is the prefix's when the host is installed beside a `python312` (with what is in
its site-packages: pycvc, pycvc_gl, numpy, ...), else the one the host was built against.
`PYTHONHOME` and `PYTHONPATH` work as for `python3`. There is one interpreter per process: numpy
and the SWIG modules are not sub-interpreter safe.

Each is a cvcpkg package of its own (Linux), so the stock hosts stay Python-free:
`cvcpkg install ariglx` brings `python312`, `numpy-cp312`, `pycvc-cp312` and `pycvc-gl-cp312` into
the same prefix, and `ariglx` finds them there. `arix` brings pycvc but not pycvc_gl; install
`pycvc-gl-cp312` too for a Python program, whose `Host` is pycvc_gl's.

**A document's `python:` block.**

```yaml
requires: [python, python.numpy]
python:
  path: [py]              # sys.path entries, relative to the document (its directory is always one)
  modules: [sim]          # imported at load
  export:                 # functions to export by name
    - sim.step_world
    - {name: sim.reset, as: reset-world, async: true}
  source: |
    import numpy as np

    @export
    def histogram(values, bins=16):
        counts, edges = np.histogram(values, bins)
        return {"counts": counts.tolist(), "edges": edges.tolist()}
init: (state-set-tree "hist" (histogram (list 1 2 2 3)))
```

- The block is its source alone (`python: |`), or a map of `source:`, `modules:`, `path:` and
  `export:`.
- **When.** It runs on every load (a reload too), after the document parses and before its
  `requires:` is checked and `init:` runs, in a module of its own, fresh each load. A reload also
  reloads the modules it imports from the document's own directories.
- **Exports.** They become the document's host functions, as `Host.function` makes them (§9.3):
  `(histogram xs)` passes the call's arguments as Python values and returns the function's value.
  - `@export` marks a function in the source, where `export` is already defined. A module marks its
    own with `from _arihost import export` (or `pycvc_gl.ariadne.export`, which also imports
    under plain `python`).
  - `export:` names functions too; `as:` gives one another name, and `async: true` makes it an
    async function (§10.1).
  - Exporting goes through pycvc, so pycvc must be importable.
- **Errors.** A Python exception in a call is a `PythonError` the document can catch
  ([STATE_EXEC_EXCEPTIONS.md](STATE_EXEC_EXCEPTIONS.md)). One at load fails the load, with the
  traceback from the document's code.
- **Caps.** As `Host` does, the host raises the per-action caps (§2.2) to 30 s and no step cap, so a
  Python call can take its time, unless the app's tree already sets them.
- **Tokens.** `python` holds where an embedded Python runs the block, and `python.<module>` where
  `<module>` is importable (`python.numpy`): list them in `requires:`, or ask with
  `(has? "python.numpy")`. A stock host fails a document that requires `python` with an error that
  names the scripting hosts, and ignores a `python:` block with a warning.
- **Trust.** Natively the block runs with the user's authority, like any program they start (it
  can `import os`), so run a document with Python only when you would run the program.

**A Python program in place of the document.** `ariglx run app.py [args]` (or `snapshot`, `check`;
`arix` too) runs `app.py` as `__main__`, with `sys.argv` = `[app.py, args...]` and its directory
first on `sys.path`. Its `pycvc_gl.ariadne.Host()` adopts the host (mode `"embedded"`, the default
inside a scripting host) instead of opening a window of its own: the same window, runtime and
state, with the rest of `Host`'s API (§9.3).

```python
from pycvc_gl.ariadne import Host

host = Host()  # this ariglx's window and document runtime


@host.function("twice")
def twice(x):
    return 2 * x


host.load("app.ari")
host.run()  # the host's own loop: run's window, snapshot's frames
```

- `host.run()` runs the host's loop, once, and returns the document's exit status. A program that
  loads a document and returns has the host run it then. The host exits with the program's status,
  or the document's when the program's is 0.
- `on_frame` hooks, `watch` callbacks and `on` handlers run in each of the host's frames, whichever
  loop drives them. An exception in one is printed and closes the document with status 1.
- `pycvc_gl.ariadne.current_host()` is that Host (None under plain `python`), and the same program
  runs under plain `python`, where `Host()` opens a window.
- In `arix` the Host has no GL (its mode is `"headless"`).

**`<host> python`.** The embedded interpreter as `python3`, with the host's packages:
`ariglx python -m pytest`, or `ariglx python -m grl_snam.ariadne live austin-freedrive`. That
interpreter has no document attached, so `Host()` there opens a window of its own.

**In the browser: `ariglx-web`.** The `ariglx-web` package is arigl-web's page (§9.4) with
`ariglx` in it, built to single-threaded WebAssembly
([`cvcpkg/recipes/ariglx-web`](../cvcpkg/recipes/ariglx-web/recipe.yaml)). Serve
`share/ariglx-web/` beside the documents as for arigl-web, with the same `ariadne.config.json`.
The page fetches `python312.zip` with the module and puts it on `sys.path`. The zip holds the
stdlib and the pure-Python packages.

- **What runs.** A document's `python:` block runs at load as it does natively, with pycvc,
  pycvc_gl and numpy (`?src=main.ari`).
- **A program as the entry point.** `?src=app.py` runs a Python program in place of the document.
  Only a document's own imports are fetched ahead, so list the files the program loads in the
  config's `preload`.
- **`host.run()` returns 0 at once.** The page's frames run between yields to the browser, and
  Python is never on the stack at a yield. The page runs the document once the program returns,
  so a program reports from `on_frame` or `watch`, not after `run()`.
- **The packages are fixed when the page is built.** A browser can't load an extension module at
  run time, so every C extension is linked into the module: numpy now. A `.py` beside the
  document works as it does natively.
- **`xml.parsers.expat` and `hashlib`'s SHA-2 work** with python312 3.12.10+cvc.23 or later. The
  wasm bundles before it lack their objects, and the page refuses to build against one. `hashlib`
  has no OpenSSL backend: it offers CPython's own hashes.

**A document's `wasm:` block: WebAssembly guests.** An `arix` or `ariglx` built where wasmtime's
C API is found (the cvcpkg `wasmtime` package; [`src/ariadne_wasm`](../src/ariadne_wasm/wasm_engine.h))
also runs a compiled plugin, a guest, from any language that compiles to WebAssembly. Natively
only, so far: the browser and the BSDs come later (roadmap/ARIADNE_SCRIPTING_HOSTS.md §8, G2–G3).

```yaml
requires: [wasm]
wasm:
  module: scorer.wasm            # beside the document; a .wat file is compiled from text
  export: [score_route, {name: rank, as: rank-routes}]
  hooks: {on_tick: tick, on: {replan: replan}, watch: {route: route_changed},
          on_key: key, on_pointer: pointer, on_signal: signal, msg_recv: {jobs: take_job}}
  imports: [state.read, call]    # narrows what the guest's calls reach
  limits: {memory_mb: 32}
```

- **Exports.** Each is a function of the document: `(score_route r)` runs the guest's export as the
  calling program's own work, a slice of fuel per step of its process, charged to it, as
  `collections.map` runs its function. `(score_route-async r)` returns a future, and the export
  runs as an activation of its own on the app's scheduler. `init:`, which can't wait, runs the call
  to its end there instead.
- **`ari_init`.** A guest that exports it has it run at every load, before the document's
  `init:`, under the `init:` lane's caps (10 million steps, 5 s). It may call the host, as an
  export may. A failure fails the load.
- **Hooks.** Each runs an export with the event as its one argument, the dict the DSL's own hook
  reads:
  - `on_tick` at the end of every frame, one at a time;
  - `on: {event: export}` when a button or menu item fires `event`;
  - `watch: {path: export}` when the value under the document's prefix changes;
  - `on_key`, `on_pointer` and `on_signal` with the `event` an `on_key:`, `on_pointer:` or
    `on_signal:` program gets (§2.3), beside the document's own program if it has one. Every
    event comes, in order, none coalesced. A guest's `on_signal` takes the signal over as
    `on_signal:` does, so the host doesn't close;
  - `msg_recv: {channel: export}` with `{channel, value}`, for each message on the channel, as
    `(msg-recv channel)` in a program of the document would take it (its channel policy too).

  A key, pointer, signal or message hook runs one event at a time. Events that come meanwhile
  wait their turn, and a burst is taken in the frame it came in. An event whose call fails is
  warned of once per message, and the next event still runs.
- **What a guest can reach.** Its `call` import calls any function a program of the document can
  call, in the process the guest runs in (the calling program's, or a hook's or a future's
  activation), under the document's prefix and locks: the state functions, the host's functions,
  the document's `functions:`. `imports:` narrows that to the groups named (`state.read`, `state.write`,
  `msg`, `net`, `call` for the rest); a call outside them fails with a `PermissionError`.
  `state.write` holds every state function that writes: `state-set`, `state-set-tree`,
  `state-data-set`, `state-from-json`, `state-delete`, the atomics (`state-cas`, `state-swap`,
  `state-incr`) and the expiry setters.
- **State, quickly.** The state imports (`state_get`, `state_exists`, `state_children`,
  `state_data_get`, `state_set`, `state_data_set`, `state_delete`, `state_swap`, `state_incr`,
  `state_cas`) call the same functions `call` reaches, under the same prefix, `link:` holes and
  `imports:`, but inside the guest's slice: a step each, where a `call` ends the slice for the
  activation to make it. A guest that reads and writes state in a loop pays for the loop, not for
  a round trip per value.
- **Host objects by handle.** A volume, a mesh or a height field a program passes (from
  `state-data-get`, or a `data:` block's node) reaches the guest as a handle, not its bytes. The
  guest asks for its sizes (`data_info`) and copies only the region it reads (`volume_read`,
  `volume_sample`, `mesh_read`, `heightfield_read`) into its own memory, at a step per 256 bytes.
  A handle it hands back, as a result or a call's argument, is the object again. `imports:`
  without `data.read` refuses these calls.
- **Writing volumes.** `volume_write` copies a box from the guest's memory into a volume, and
  `volume_new` makes a new one, zeroed, from `dims`, `voxel` and `bounds` (data_info's map). The
  guest writes its own copy: the first write to a volume anyone else holds (the state tree, a
  renderer) copies it, and the handle names the copy from then on, so nothing else changes until
  the guest stores the handle with `state_data_set`. A new volume counts against a call's
  allocation limit, as a native's allocation does. `imports:` without `data.write` refuses
  both.
- **Limits.** A guest's work is charged as DSL steps (`ariadne.limits.wasm_gas_per_step`, 1000
  units of fuel a step) to the process it runs in, the calling program's or the activation's, so
  that process's caps (the action caps, §2.2) bound it. The guests share `ariadne.limits.wasm_frame_budget` (0.008 s) of each frame, and its memory
  is at most `ariadne.limits.wasm_max_memory` (256 MiB), which `limits: {memory_mb}` may lower.
- **One call at a time.** An instance runs one call at a time. The call holds the instance's
  lock, `wasm:<module's stem>` under the document's prefix (`lock-query` shows it), and the others
  wait for it as a program waits in `with-lock` (§2.1, the other lanes' locks). So an action gives up after
  `ariadne.exec.lock_timeout` (1 s) with a `TimeoutError`, while a resident waits. A call back
  into the guest from inside one of its own host calls, through a document function say, would
  wait on itself, so it raises a `RecursionError` there instead.
- **Errors.** A trap, or the guest's `fail`, is a `WasmError` the document can catch; its `trap`
  says what happened. A trap with the guest's memory at its limit is a `MemoryError`.
- **The ABI** a guest is written to (values as CBOR in its memory, `ari_alloc`, the `call`,
  `reply`, `yield` and `fail` imports, the state imports, and the data calls on handles) is in
  [`wasm_engine.h`](../src/ariadne_wasm/wasm_engine.h).

### 9.7 A server document: `arisrv --doc`

`arisrv --doc server.ari` runs a document on the relay
([`server_document.h`](../inc/cvc/ariadne/server_document.h); the plan is
[roadmap/ARIADNE_SCRIPTING_HOSTS.md](roadmap/ARIADNE_SCRIPTING_HOSTS.md) §6.1 and §6.5.3). It is a
headless `AppRuntime`, with no backend and no scene, over the relay's own app tree: room rules,
bots and checks, written in the DSL and held to its caps and its chroot.

```yaml
# server.ari, run by the relay: arisrv --doc server.ari
requires: [relay]              # a relay's server document; elsewhere the load fails
server:
  publish: [board]             # every client is sent it; only this document writes it
init: (state-set "board.status" "up")
on_tick: |
  (state-set "board.present" (length (state-children "room.participants")))
  (state-set "board.rev" (state-get "sim.tick"))   # a live view watches this one key
```

- **When it runs.** In this order:
  1. arisrv reads the `server:` block and chooses the room's world (below), before anything else
     that could fail late.
  2. It binds its ports, as it always has.
  3. It publishes the world, in one revision, seeds the avatar catalog, and publishes the
     world's rules (below).
  4. It applies `--set`.
  5. It loads the document, which runs its `init:`.

  `--set` comes after the world and the policies, so it overrides them (the worlds plan's WD0),
  and before the document, so its `ariadne.limits.*` bound `init:` too. A client that connects
  before the document has loaded still gets `init:`'s writes, live.
- **Every room step** (60 a second), `on_tick:` and the other residents run, ahead of the avatar
  holon. What they write goes out at the end of that step, which §6.5 of the roadmap calls "the
  next relay tick".
- **The clock.** `on_tick:` runs once per room step: it is the room rules' cadence, not a
  per-tick controller.
  - Before each step the relay writes its world clock (the avatar holon's) to the document's
    `sim.tick` and `sim.time`. These are the keys a scene's `clock:` publishes in a window (§9).
  - `sim.tick` counts 120 a second, so it moves by about 2 a step, and never back. A change that
    takes effect "at world tick N" means the same here as in the holon.
- **The relay's loop.** The programs run on it, under the DSL's caps (`ariadne.limits.*`, which
  arisrv's `--set` reaches, since it is applied before the document loads). A slow `on_tick:`
  delays everyone's poses, so keep it short.
- **Its own state.** The document binds under `ui.docs.arisrv`, which the relay never sends. No
  client gets the document or its own state.
- **`requires: [relay]`** holds only in a server document. On any other host, a document that
  requires it fails to load, and the error says it runs on the relay (`arisrv --doc`).
- **`server: publish:`** names the namespaces the document publishes.
  - A name is a dotted path of identifiers. It may not be `conf`, `ui`, `ariadne`,
    `state_exec` or `room`, or under one of them, and none may be inside another or overlap the
    document's own prefix.
  - Each is a writable hole of the same name in the document's chroot (§7.4): `board` there is
    the tree's `board`. `state-set`, `state-set-tree` and the atomics write through it.
    `state-delete` and `state-watch` act on the physical path (§7.4), so to take entries out,
    rewrite their parent with `state-set-tree`.
  - Every client is sent the published namespaces, live and in a joiner's snapshot. Clients'
    interest takes only `conf` for now, so they drop them until a document subscribes (the
    roadmap's `relay: subscribe:`, S2).
  - Only the server writes them. Bound writes refuse a client's write there, as anywhere outside
    its own participant, so a document that publishes needs `--bind-writes on`: arisrv refuses to
    start without it.
  - Writes reach clients in the order a program makes them. So a live view can watch one key:
    write a section's fields, then its `rev`, and a client holds those fields by the time it
    sees `rev` move.
- **`room`** is a read-only hole on the relay's room, `conf.rooms.<room>`:
  `(state-get "room.topic")`, `(state-children "room.participants")`. A write under it stays in
  the document's own subtree.
- **What the relay sends** is the room tree and the published namespaces, with or without a
  document. Anything else in its tree, such as a `--set` key outside `conf`, stays on the relay.
- **Stopping.** `(quit [code])` stops the relay with that exit status. A document that fails to
  load stops it with 1. One arisrv cannot read, or a bad `server:` block, stops it with 2 before
  it listens.
- **A `scene:` with nodes or lights is the room's world** ([roadmap/ARIADNE_WORLDS.md](roadmap/ARIADNE_WORLDS.md)
  WD1). The relay publishes it into `conf.rooms.<room>.scene` before `init:` runs, and every
  client builds the room from that, look included: the light rig, its tuning knobs and the shadows
  are the world's, not the client document's.
  - **What the room carries:** the background, the chrome, the shadows (resolution and
    `update_interval`), every light and rig, and each top-level geometry node with a plane, box or
    sphere source or a mesh by URI (with its `fit:`), its transform and its plain material.
  - **What it leaves out**, with a warning at start-up: other node types and sources (a
    heightfield, a volume, a custom node), children, textures, shaders, a `fit:` with `up: x` or
    `flip`, and a `visible:` bind.
  - **Bodies and `physics:`.** A body is published as a static, since the holon builds its statics
    from the published objects; a server-dynamic one waits for WD3, with a warning. The parse,
    `physics:` and bodies included, is kept on the runtime (`ServerDocument::scene()`). This
    runtime is the one the world runtime will run the world in, headless, as the holon's one
    physics world (§14 of the same roadmap). Until then the load warns that the physics is not
    simulated.
  - **Without one**, the room's world is the stock world `--scene` names: `cvc://worlds/default.ari`
    (the 20 m room) or `cvc://worlds/large.ari`. `--scene` and a `--doc` with a `scene:` both
    naming it stop arisrv with 2. A stock world opens in arigl too (`arigl
    cvc://worlds/default.ari`), to see the room without a relay.
- **A `world:` block sets the world's rules** ([roadmap/ARIADNE_WORLDS.md](roadmap/ARIADNE_WORLDS.md)
  WD2; [`world_rules.h`](../inc/cvc/ariadne/world_rules.h)): what every avatar moves under. The relay
  publishes them as four sections of the room (`physics`, `avatars.archetypes`, `avatars.policy`,
  `controllers`; [`room_rules.h`](../inc/cvc/ariadne/conference/room_rules.h)), and every holon, the
  relay's and each client's, runs under them.

  ```yaml
  scene:
    physics: { gravity: [0, 0, -9.80665] }   # the world's gravity
  world:
    avatars:
      archetypes:                  # every avatar has the first, for now
        person: { body: { mass: 70, width: 0.5, height: 1.7 }, controllers: [walker@1] }
      policy:
        max_speed: 4.0             # the veto: the fastest an intent may ask for (m/s)
        allow_vertical: false
        energy: { enabled: true, force_max: 840, power_max: 1200, power_rest: 300,
                  reserve_max: 30000, transport_cost: 0.2, empty: rest, recover_fraction: 0.25 }
    controllers:                   # walker@1 is the one kernel this build runs
      walker@1: { kernel: walker, version: 1, params: { walk_speed: 2.0 } }
  ```

  - **Each key left out keeps the default**, which is the block above. Archetypes, when given,
    replace the default list; controllers join it, or replace one by id.
  - **What cannot run stops arisrv with 2**: a controller kernel this build does not have, an
    archetype allowing a controller nobody defines, a body that is not positive, a limit that is
    negative. A key it does not know is a warning, and so is `catalog:` (the relay seeds its own),
    a `fixed_dt` other than the room's 1/120 s, and `materials:`.
  - **Where they come from.** The `--doc`'s, when it has a `world:` block or is the world;
    otherwise the stock world's, whose `world:` block states the defaults. `--avatar-energy on|off`
    and `--avatar-empty rest|stop` override the energy policy.
  - **When they take effect.** At start-up, from the start. A section the relay rewrites while the
    room runs names a world tick a little ahead, and every holon switches on it; a client that hears
    of it late re-runs its prediction from that tick.
- **Other hosts** ignore a `server:` block, with a warning, and a `world:` block silently.
- **Not yet:** a reload when the file changes, the network (`--net`), `on_signal:` for arisrv's
  own SIGTERM, and a `python:` block in the server document itself (arisrvx runs the operator's
  Python beside the relay instead, §9.8).

### 9.8 Python beside the relay: `arisrvx`

`arisrvx` is arisrv with an embedded CPython 3.12 ([`server_link.h`](../src/ariadne_python/server_link.h),
[`ariadne_server.py`](../src/ariadne_python/ariadne_server.py); the plan is
[roadmap/ARIADNE_SCRIPTING_HOSTS.md](roadmap/ARIADNE_SCRIPTING_HOSTS.md) §6.5). `arisrvx --python
main.py [relay flags]` runs the operator's program next to the relay, for a service with an HTTP
API for programs and live state for Ariadne clients:

```python
# main.py, run as: arisrvx --python main.py --doc server.ari [relay flags]
from fastapi import FastAPI
from pycvc.ariadne import Host, serve_asgi

app = FastAPI()
host = Host()                       # the relay's tree and the script thread's loop

@app.post("/jobs/{name}")
async def start(name: str):
    host.set_tree(f"svc.jobs.{name}", {"state": "queued"})   # svc is the document's publish:
    return {"ok": True}

serve_asgi(app, host="127.0.0.1", port=8780)
host.run()                          # until SIGTERM; the app's shutdown, then the relay
```

- **The script thread.** The program runs on a thread of its own with one asyncio loop
  (`host.loop`), which serves its ASGI apps (uvicorn) and its tasks. The relay never runs Python,
  never takes the GIL and never waits on the program: a sync handler holds up only the loop.
- **Writes.** `host.set`, `set_tree` and `set_series` (as `pycvc_gl.ariadne.Host`'s) take paths
  from the tree's root and are queued. The relay applies them on its next room step (60 Hz), ahead
  of the server document's programs, coalesced per path and in the order of each path's last write
  (a section's fields, then its `rev`). `--script-write-budget N` caps a step at N bytes (65536 by
  default; 0 for none); the rest wait for the steps after, in order, and one write larger than the
  cap still lands. `set_tree` writes as `(state-set-tree)` does: only what changed, and the
  children it doesn't name are removed. Clients receive what the server document publishes
  (`server: publish:`, §9.7), which needs `--bind-writes on`.
- **Reads.** `host.get`, `get_tree` and the rest read the relay's tree as it is, so a write of the
  program's own shows a step later.
- **Startup.** The program starts before anything listens. The relay listens once its ASGI apps have
  started (their lifespan startup and their bind); one that fails stops arisrvx with 1 and its
  traceback, before it listens. A program that returns without `host.run()` has its loop run by the
  host. `--http-listen ADDR:PORT` serves the program's `app` for a program that doesn't call
  `serve_asgi`.
- **Stopping.** On SIGTERM or SIGINT the relay keeps running while the program shuts down: each
  ASGI server stops accepting, finishes its requests within `--shutdown-timeout` (10 s), and runs
  its app's shutdown; the loop's other tasks are cancelled. Its last writes go out, and then the
  relay stops. `host.stop(code)`, or an ASGI server that ends by itself, stops the relay too, with
  that exit status. The host owns signals: uvicorn installs none.
- **Logging.** Python's `logging`, uvicorn's included, goes to stderr as `[arisrvx] <logger>:
  <level>: <message>`, unless the program sets up its own.
- **`arisrvx check [main.py] [relay flags]`** runs the program without serving, then loads the
  server document (its `python:` block, its `requires:`, which take `python.fastapi` and the like,
  §9.6, and its `init:`). It reports a missing module, an `--http-listen` with no `app`, and a port
  that is taken (the relay's and the apps'), and exits 1 on any of them, 2 on a bad flag. It also
  names the functions clients may call.
- **`arisrvx python [args]`** is the embedded interpreter as a plain python3.
- **Under plain `python`** the same program runs: `pycvc.ariadne` ships with pycvc, `Host` keeps a
  tree of its own, writes land at once, and `run()` takes SIGINT and SIGTERM itself. arisrvx has
  the module built in; it needs pycvc only for the functions below, which convert DSL values.
- **Packages.** `cvcpkg install arisrvx fastapi-cp312 uvicorn-cp312` (Linux), and `pycvc-cp312` for
  `room-call` and a server document's `python:` block. Stock arisrv refuses the Python flags and
  names arisrvx.

#### 9.8.1 Functions clients call: `room-call`

The program exports functions; a client calls one and waits for its value (roadmap §6.5.5;
[`room_calls.h`](../inc/cvc/ariadne/conference/room_calls.h)):

```python
from pycvc.ariadne import CallError, Host, export

host = Host()

@export(clients=True, roles=["staff"], timeout=600)
async def instance_create(ctx, project: str, spec: dict):
    job = await service.create(ctx.principal, project, spec, key=ctx.idempotency_key)
    async for step in job.steps():
        ctx.progress(step.fraction, step.message)
    if job.failed:
        raise CallError("conflict", job.reason, detail={"job": job.id})
    return job.result

host.run()
```

```lisp
(let ((r (room-call "instance_create" (dict "project" "acme" "spec" spec))))
  (state-set "created" (get-attr r "id")))
```

- **The call.** `(room-call "fn" arg ...)` parks the program until the value comes back;
  `(room-call-async ...)` returns a future for `(await f)`; `(room-call-opts (dict "key" K
  "timeout" S) "fn" arg ...)` sets the call's idempotency key and its timeout. One dict argument
  is the function's keyword arguments. The call rides the caller's own participant path, to the
  relay alone, and the reply goes only to the connection that holds the caller's pid. So it takes
  bound writes (`--bind-writes on`, the default).
- **Values keep their types.** Arguments and values are DSL values on both sides. They are sent as
  typed CBOR ([`value_cbor.h`](../inc/cvc/state/state_exec/value_cbor.h), symbols as themselves),
  so nil, booleans, ints, floats (`2.0` stays one), strings, bytes, symbols, lists and dicts
  arrive as they were sent. In arisrvx, pycvc converts them, as for every pycvc value: the
  function gets `int`, `float`, `str`, `bytes`, `pycvc.symbol`, `list` and `dict`, and what it
  returns goes back the same way. A value that doesn't cross (a function, a host object) fails the
  call where it is sent, as a `TypeError`.
- **`ctx`**, passed first to a function exported with `clients=True`:
  - `principal`, `{"id": <the caller's pid>, "roles": []}` until S5's `authenticate` hook says who
    it is;
  - `call_id`;
  - `idempotency_key`, the caller's or the call id;
  - `deadline`, a `time.time()`;
  - `progress(fraction, message)`, which lands in the caller's tree at `calls.<fn>.progress` and
    `calls.<fn>.message`, and is posted to its future.
- **Gates and errors.** `roles=` needs one of them, else `forbidden` (a `PermissionError`), and the
  function never runs. `raise CallError(code, message, detail=...)` reaches the caller with those as
  fields (a `RuntimeError`; `try` catches it). Any other exception is `internal`; its traceback is
  logged on the relay, never sent. A call past its `timeout` (the export's, or the caller's if
  shorter; 30 s when neither says) is cancelled: `deadline_exceeded`, a `TimeoutError`. A value
  over `--call-result-max` (1 MiB encoded; 0 = none) is `too_large`. A name no client function has
  is `not_found`. A relay with no Python (stock arisrv), or no pycvc beside it, answers
  `unavailable`.
- **Where they run.** An `async def` runs on the loop; a plain function runs inline on it, and one
  that runs longer than `--script-slow-ms` (50 ms; 0 = never) is logged by name, since it held up
  every other caller. `is_async=True` runs a plain function with `asyncio.to_thread` instead.

#### 9.8.2 A server document's `python:` block

A server document's `python:` block (§9.6's form) runs on the script thread too: arisrvx starts the
thread for it even without `--python`. Its exports (`@export`, `export:`), and the program's, are
the document's functions. A call from one of its programs parks the program and runs the function
on the script thread, and `init:` waits for it in place. Values cross as above, typed, and a raise
is a `PythonError` a `try` catches. A reload runs the block again.

#### 9.8.3 Joining a relay: `relay:` and `--relay`

`ari` and the `x` hosts join a relay as a client, from a document's `relay:` block or the host's
`--relay URL` ([`relay_session.h`](../inc/cvc/ariadne/conference/relay_session.h)):

```yaml
relay:
  url: grpcs://ops.example.com:7700   # grpc://, grpcs:// (TLS), or ws:// (plain)
  room: ops                           # the relay's --room (default standup)
  token: env:OPS_ROOM_TOKEN           # env:NAME, file:PATH, or credential:NAME / oidc:NAME
  ca: certs/ops-ca.pem                # grpcs://: the CA (default: gRPC's roots)
  subscribe: [cyprov]                 # the published namespaces to mirror
```

- **With a token** (`env:NAME`, `file:PATH`), the host joins at the document's load, before its
  `init:` runs, and the load fails if it can't. `--relay URL`, `--relay-room NAME` and
  `--relay-ca FILE` set those over the block (whose `subscribe:` stays); with `--relay` the token
  is `CVC_ARIADNE_ROOM_TOKEN`, never the command line.
- **With a credential** (`credential:NAME`, or `oidc:NAME` for the credential an
  `(oidc-sign-in ...)` keeps, §10.5; `--relay-credential NAME` on the command line), the token is
  read from the host's credential store at every join.
  - The credential must be bound to the relay's origin by a policy line, and that origin allowed:
    `https://host:port` for `grpcs://`, `http://host:port` for `grpc://` and `ws://`, as in
    `credential ops-sso https://ops.example.com:7700`.
  - The join waits until the credential holds a token (a sign-in the document starts), and
    happens from the host's frames. Until then `room-call` answers `unavailable`.
- **Reconnects.** Once joined, a connection the relay closes (a restart, `host.disconnect`, a
  network drop) is dialed again after 1 s, then twice as long each time up to 30 s.
  - Each dial reads the token again. A credential the relay refused is renewed first, which an
    OIDC sign-in does with its refresh token.
  - Meanwhile `room-call` answers `unavailable`.
  - The host keeps its participant id, so its bound writes are its own again.
- **Where it stands.** The session writes `relay.status` (`joined`, `dialing`, `waiting for
  credential NAME (…)`, `lost; dialing again (…)`) and `relay.joins` (1, then 2 after a
  reconnect, …) under the document's root, for a status line or a test to wait on.
- A subscribed namespace lands at its own path and is linked read-only under the document's root,
  so `cyprov.projects.acme` binds as any of the document's state.
- **TLS for native clients.** `arisrv --tls-cert PATH --tls-key PATH` serves both gRPC planes over
  TLS, and clients dial `grpcs://`. The WebSocket listener stays plain: a reverse proxy terminates
  TLS for browsers, and the native client has no `wss://` yet.
- A relay that turns the host away (a wrong token, or its authenticate hook, §9.8.4) fails a
  token join's load at once: "refused this host (unauthenticated…)". A refused credential, or a
  refused reconnect, is dialed again after the backoff, renewed first. A principal the service
  revoked stays out, and its calls stay `unavailable`.

#### 9.8.4 Who joins: `host.authenticate`

The program can decide who joins, in place of the room token (roadmap §6.5.4):

```python
@host.authenticate
async def auth(cred):                 # cred.bearer, cred.transport ("grpc" | "ws"), cred.peer
    user = await verify(cred.bearer)  # a JWT check, an IdP's userinfo, a table
    if not user:
        return None                   # refused
    return {"id": user.sub, "name": user.name, "roles": ["staff"] if user.staff else []}
```

- **The bearer.** It is gRPC's `authorization: Bearer` header, or the WebSocket's `cvc-token.<t>`
  subprotocol, as for a room token. A client presents it with `relay: token:` or
  `CVC_ARIADNE_ROOM_TOKEN`; it may be up to 4096 characters, an OIDC access token for one.
- **The verdict.** It is a principal, a dict with a str `id` (and whatever else the service keeps:
  `roles`, `name`), or None.
  - The hook runs on the program's loop, before the connection is admitted.
  - A None, a raise, or no verdict in `--auth-timeout` seconds (5) refuses the connection before
    it is sent anything (`unauthenticated`).
  - Declare the hook before the program serves.
- **What the principal keys.**
  - The connection's bound writes: its participant can be taken over only by a connection of the
    same principal.
  - Its calls' `ctx.principal`, with the role gates checked against its `roles`.
  - Their limits: `--call-concurrency` (8) calls in flight and `--call-rate` (no limit) calls a
    second per principal. Past either, a call is answered `rate_limited`.
- **`host.disconnect(principal_id)`** closes that principal's connections on the relay's next step.
  Their participants leave as any closed connection's do. Re-checking long-lived connections is the
  service's job: a task that calls it.

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
- **In a browser build without threads** (the single-threaded wasm pages: `ari.html`, arigl's
  page) there is no background pool to wait on: its jobs run on the page's main thread, between
  frames. There the verbs start the request instead of sending it
  (`cvc::net::HttpClient::start`, a browser fetch) and return to the frame loop at once. The
  calling program parks as it does natively, frames keep drawing and taking input, and the
  fetch's callback posts the reply to the scheduler, which resumes the program on a later
  frame. So a spinner (or a document's own progress bar between `(await f 0.05)` slices)
  animates while a request is out. The contract is the same as natively: the reply dict, the
  host's policy, `auth` / `capture_auth` and credential values (the token goes in and comes out
  in C++ as the reply arrives), a 30 s timeout (the reply's `error` says it timed out), the 64
  MiB cap (enforced as the body arrives), and a killed or reaped program aborts its fetch.
  `fetch` of an `http(s)` URI is started the same way, through the http handler's non-blocking
  form (`register_uri_async_handler`, `resolve_async` in [`uri.h`](../inc/cvc/ariadne/uri.h)); a
  host that registers its own `http` handler in its place gets the blocking call for it. Two
  things still wait in place in that build: a call where the program can't park (a generator's
  body, a recursive evaluator), which runs the request inline as natively; and the loads the
  host makes itself, outside any program: an `https` `import:`/`load:` while the document loads,
  and a scene's `source:` mesh or texture URL when the scene is realized
  (`resolve_to_file`). Those use the blocking `cvc::net::send()`, which yields to the browser
  under Asyncify but holds the frame that called it. (The browser pages fetch a document's
  relative imports themselves before it loads.) In builds with threads (native, wasm-mt) the
  request runs on the background pool as before.

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
parsing one reply before it sends the next is not idle), for three steps running, at most 10 s
(`--settle S` sets another limit, for a slow server or a slow build).
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
| `--oidc-issuer ISSUER` | documents may sign in (§10.5) with `ISSUER` only; allows its origin (repeatable; implies `--net`) |
| `--oidc-client I=ID` | sign in to issuer `I` as client `ID`, the launcher's registration (repeatable; pins `I` too) |
| `--oidc-redirect-uri U` | the redirect URI a browser page registers (§10.5; default: the page's own URL) |
| `--no-browser` | never open the user's browser: sign-in shows a device code, `open-url` shows the URL |

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

### 10.5 Signing in: OAuth 2.0 and OpenID Connect

A document signs its user in with any OpenID Connect provider (Keycloak, Authentik, Google, a
txwtf instance, ...) from two public facts: the provider's **issuer** URL and the **client id** of
a public client registered with it. The host ([`oidc_host.h`](../inc/cvc/ariadne/oidc_host.h))
reads the provider's discovery document (`/.well-known/openid-configuration`), runs the whole flow
and keeps every token in its credential store (§10.3) under a name the document chose. The
document then names that credential on its requests, as it does any other:

```lisp
(oidc-sign-in (dict "issuer" "https://id.example/realms/demo" "client_id" "my-app"
                    "scopes" (list "openid" "profile" "email") "credential" "me"))
;; ... later, once oidc.me.phase is "signed_in":
(http-get "https://api.example/v1/me" (dict "auth" "me"))
```

Nothing in libcvc is specific to a provider. The verbs exist wherever the network is on with an
HTTP backend (every `ari` and `arigl`, in a terminal, a window and a browser page):

| verb | what |
|---|---|
| `(oidc-sign-in OPTS)` | start a sign-in, or resume one kept from before; returns `"started"` at once (or `"in_progress"`) |
| `(oidc-cancel NAME)` | stop a sign-in in progress; `#t` if there was one |
| `(oidc-sign-out NAME [OPTS])` | forget the tokens; revoke them where the provider can (`"sent"` or `"no"`) |
| `(oidc-status NAME)` | the sign-in's status as a dict: the keys of the table below, `claims` a dict |
| `(open-url URL)` | open an http(s) URL the host allows in the user's browser (§10.6) |

`oidc-sign-in`'s options (an unknown one is an error):

| option | what |
|---|---|
| `issuer` | the issuer URL, exactly as the provider's discovery document writes it (required) |
| `client_id` | the public client (required unless the launcher gives one with `--oidc-client`, which wins) |
| `credential` | the credential to keep the sign-in under (required; 1-56 of `A-Z a-z 0-9 . _ -`) |
| `scopes` | a list of scopes or one space-separated string; default `openid` (`openid` asks for an ID token, and so for the claims) |
| `params` | a dict of extra authorization parameters: `prompt`, `login_hint`, `acr_values`, `audience`, ... (never one the host sets: `state`, `nonce`, `redirect_uri`, `code_challenge`, ...) |
| `flow` | `auto` (default), `loopback`, `device` or `redirect` |
| `interactive` | `#f`: only resume a kept sign-in (or complete a browser page's return); start nothing new. Default `#t` |
| `status` | the state path the progress goes to, under the document's root; default `oidc.<credential>` |
| `client_secret` | for a provider that gives installed apps a non-confidential "secret" (Google's desktop clients); never a confidential client's |
| `port`, `redirect_path` | the loopback flow's port (default 0: any free one) and path (default `/callback`) |
| `redirect_uri` | the redirect flow's redirect URI (default: the page's own URL; `--oidc-redirect-uri` wins) |
| `timeout` | seconds to wait for the browser's redirect (default 300) |

**Progress** goes to the state tree, so a widget shows it with a `bind:` or a `visible_when:`,
and to the channel `oidc#<credential>`, which gets the final status dict when a sign-in ends
(signed in, error, cancelled, signed out), so a program can wait with
`(msg-recv "oidc#me")`. None of it is secret:

| key under `oidc.<credential>` | what |
|---|---|
| `phase` | `signed_out`, `discovering`, `waiting_for_browser`, `redirecting`, `device_code_shown`, `exchanging`, `refreshing`, `signed_in`, `error`, `cancelled` |
| `signed_in` | `true` or `false` |
| `flow` | `loopback`, `device`, `redirect`, or `stored` (a sign-in kept from before) |
| `issuer` | the issuer |
| `error`, `error_description` | a short code (the provider's OAuth error, or `state_mismatch`, `invalid_token_reply`, `unsupported_flow`, `discovery_failed`, `timeout`, `network_error`, `session_expired`, ...) and a sentence for the user |
| `user_code`, `verification_uri`, `verification_uri_complete`, `code_expires_at`, `interval` | the device flow's code and where to enter it |
| `browser_url`, `browser_opened` | the authorization URL, and whether a browser was opened at it (show the URL when not) |
| `expires_at`, `has_refresh` | when the access token expires (Unix seconds, `0` unknown), whether there is a refresh token |
| `claims.sub`, `.iss`, `.name`, `.email`, `.preferred_username`, `.exp` | the signed-in user, from the ID token |

**The flows.** `flow: auto` picks per host; a flow the host or the provider cannot run fails with
`unsupported_flow` and says why (a provider without `device_authorization_endpoint` cannot run the
device flow). All of them check the provider's reply against the request: the authorization code
flows send PKCE S256 (RFC 7636), a random `state` and a `nonce`, check the `state` (and `iss`,
RFC 9207, when the provider sends it) of the reply, and check the ID token.

| host | auto picks | how |
|---|---|---|
| `ari` (FTXUI, tvision) or `arigl --terminal` on a machine with a desktop, `arigl` in a window | `loopback` (RFC 8252) | listens on `127.0.0.1` on a free port, opens the system browser at the authorization endpoint, takes the one redirect back and shows the tab "you can close this tab" |
| the same over SSH, with no display, or `--no-browser` | `device` (RFC 8628) | shows `user_code` and `verification_uri` (the document shows them; `verification_uri_complete` carries the code in the URL) and polls the token endpoint every `interval` seconds, `slow_down` adding 5; `access_denied` and `expired_token` end it |
| the arigl-web page and the `ari.html` (xterm.js) page | `redirect` | navigates the page to the authorization endpoint; the PKCE verifier, `state` and `nonce` wait in `sessionStorage` (this tab's, this origin's) meanwhile. When the provider sends the page back, the document's `oidc-sign-in` (run again as the page loads, `interactive` `#f` will do) exchanges the code, removes the record and takes `code` and `state` out of the address bar with `history.replaceState`. A page can also run `device` |

With no browser and no device flow at the provider, `auto` still runs `loopback` and shows the
authorization URL (`browser_url`) for the user to open on the same machine. A host that cannot open
a browser prints the URL on stderr, except while a full-screen terminal UI owns the screen, where
the document shows it.

**Capability tokens** (`requires:`, `has?`, §5.7): `oidc` (the verbs are here), `oidc.loopback`
(native hosts), `oidc.device` (every host), `oidc.redirect` (a browser page) and `browser` (a
browser can be opened here: not over SSH, not with `--no-browser`).

**Tokens** never reach the document. The access token is credential `NAME`, bound to the issuer's
origin and to any origins a policy `credential NAME ...` line names (so an API on another origin
gets it when the launcher says so). The refresh token is `NAME.refresh`, bound to the token
endpoint's origin, and the ID token `NAME.id`. The host:

- refreshes the access token (RFC 6749 6) before it expires, while the document is idle and before a
  request that names it, and once more when a request is answered 401, then sends the request again;
- with `--credential-store`, keeps the sign-in across runs: the next `oidc-sign-in` (`interactive`
  `#f` is enough) refreshes it and reports `flow` `stored`, or, when the provider has ended the
  session (`invalid_grant`), forgets it and reports `signed_out` with `error` `session_expired`;
- on `(oidc-sign-out NAME)` forgets all three and revokes the refresh token (RFC 7009) where the
  provider has a `revocation_endpoint` (`"revoke" #f` skips it). `"end_session" #t` also ends the
  provider's own session in the browser (RP-initiated logout at `end_session_endpoint`, with
  `post_logout_redirect_uri` when given).

The ID token's `iss`, `aud` (and `azp`), `exp`, `iat` and `nonce` are checked (OIDC Core 3.1.3.7);
a refreshed one must name the same `sub`. Its signature is not checked against the provider's
JWKS: the host takes ID tokens only from the token endpoint, over TLS, which 3.1.3.7 (6) allows in
place of the signature, and libcvc has no RSA/EC verification in every build (OpenSSL is optional,
and absent from the browser builds). An unsigned token (`alg` `none`) is refused.

**Which issuers.** By default a document may sign in with any issuer whose origin the host allows
(§10.3), and the provider's endpoints must share the issuer's origin. A launcher pins it, gives
the client id, or lets endpoints live elsewhere with flags or policy lines:

```sh
ari run --oidc-issuer https://id.example/realms/demo --oidc-client https://id.example/realms/demo=my-app \
    --credential-store demo app.ari
```

```text
oidc https://id.example/realms/demo                      # sign in with this issuer only
oidc https://accounts.google.com https://oauth2.googleapis.com   # its endpoints are also there
oidc-client https://accounts.google.com 1234.apps.googleusercontent.com
credential me https://api.example                        # the access token may go to the API too
```

Naming an issuer in either form allows its origin (and the endpoint origins listed) and pins
sign-in to the issuers named. An endpoint that is not https (http only for a loopback issuer, as a
local provider and the tests use), or is at an origin the policy does not list for its issuer,
fails the discovery: the host never contacts it. In a browser page the same flags go in the page
config's `flags` (`ariadne.config.json`):

```json
{"flags": ["--net", "--oidc-issuer", "https://id.example/realms/demo",
           "--oidc-client", "https://id.example/realms/demo=my-app"]}
```

**What a provider must allow** for a public client:

- the authorization code grant with PKCE S256 and no client secret (a "public" client); the
  device grant (`urn:ietf:params:oauth:grant-type:device_code`) for terminals over SSH;
- for the loopback flow, the redirect URI `http://127.0.0.1/callback` with any port (RFC 8252
  7.3), or a fixed `port` the document asks for;
- for a browser page, the page's URL as a redirect URI, exactly (with its query, e.g.
  `?src=app.ari`, or pin the page's document with `src_param: false` so it has none), and CORS for
  the page's origin on the discovery document and the token, device authorization and revocation
  endpoints (the page calls them with `fetch`), and the page's own Content-Security-Policy must list
  the provider's origin in `connect-src` (the arigl-web and `ari.html` pages run under a strict
  one);
- refresh tokens if sessions should outlive the access token (some providers want the
  `offline_access` scope, or a parameter such as Google's `access_type=offline` in `params`);
- a `post_logout_redirect_uri` registered, if a document uses one.

**What stays where.** The authorization URL holds no secret (the PKCE verifier never leaves the
host; a leaked `state` cannot complete a sign-in without it), so it may go to the state tree. A
browser page keeps its tokens in memory, like every credential there (§10.3): a reload signs in
again, which the provider's own session cookie usually makes a redirect there and back. The
loopback listener binds `127.0.0.1` only, takes one redirect, answers anything else 404, and
closes.

[`examples/ariadne/documents/oidc_sign_in.ari`](../examples/ariadne/documents/oidc_sign_in.ari)
signs in to a provider given on its command line and shows the claims:

```sh
ari run --oidc-issuer https://id.example/realms/demo examples/ariadne/documents/oidc_sign_in.ari \
    --issuer https://id.example/realms/demo --client my-app --api https://api.example/v1/me
```

The tests run every flow against a stub provider: `net_oidc_test` (the protocol pieces),
`ariadne_runtime_test` `AriadneOidc.*` (the flows, a redirect included, refresh, sign-out, a forged
`state` or `nonce`, the policy), `ari_net_test` `AriOidc.*` and `ari_oidc_hosts` (`ari` with FTXUI
and tvision and `arigl --terminal` in tmux, `arigl` in a window: no token in any screen, stdout or
stderr), and the wasm ari under node (ci.yml: the redirect flow there and back, the device code).
[`src/ari/tests/stub_oidc_provider.py`](../src/ari/tests/stub_oidc_provider.py) is a stub provider
to try a document against by hand.

### 10.6 Opening the user's browser: `open-url`

`(open-url URL)` opens an http(s) URL in the user's web browser
([`browser.h`](../inc/cvc/net/browser.h)): natively with `$BROWSER` when it is set, else
`xdg-open`, `open` or `ShellExecute`, its output thrown away; in a browser page with
`window.open`. It returns `#t` when a browser was asked to open it and `#f` where there is none
(over SSH, no display, `--no-browser`): the host then prints the URL, outside a full-screen terminal
UI, and the document should show it. The URL's origin must be one the host allows (§10.3), since a
page opened in a browser carries whatever its URL says; anything else, and a scheme other than
http(s), raises. One URL a second at most. It needs the network on, like the other verbs here.

### 10.7 The clipboard and the window: `clipboard-set` and `fullscreen`

Two verbs a document's programs call on the host's surface, for a "Copy link" button or a full
screen view. The backend answers them, so they work the same in every host built on it.

```yaml
- button: Copy link
  visible_when: (has? "clipboard-set")
  on: (clipboard-set (state-get "post.url"))
- button: Full screen
  visible_when: (has? "ui.fullscreen")
  on: (fullscreen (not (fullscreen)))
```

- **`(clipboard-set TEXT)`** puts TEXT on the system clipboard and returns `#t`, or `#f` when the
  host refused. `(has? "clipboard-set")` says whether the host has a clipboard at all. The verb is
  only bound where it does. The backends:
  - **ImGui** (`arigl`): `cvc::gl::clipboard::set`, which is SDL's clipboard (Wayland included),
    or the platform's under VTK's own window.
  - **The arigl-web page**: `navigator.clipboard.writeText`. The browser writes only within a user
    activation: the click (or key) that ran the action has one, since an action runs a frame
    after the click and the activation lasts longer than that. Safari is the strictest. The write
    is asynchronous, so `#t` means it was asked for.
  - **FTXUI** (`ari`): an OSC 52 escape (`ESC ] 52 ; c ; base64 BEL`), written after the drain
    that ran it. The terminal does the copy, so over SSH it lands on the user's own clipboard.
    Most terminals take it: kitty, WezTerm, foot, Alacritty, iTerm2, Windows Terminal, and
    xterm with `allowWindowOps`. tmux needs `set -g set-clipboard on`. A terminal that ignores it
    says nothing, so `#t` means the escape was sent. More than 1 MiB of text is refused (`#f`).
    `FtxuiBackend::take_clipboard()` hands the escapes to a headless driver.
  - **The ari.html page** (xterm.js): the page answers the escape with
    `navigator.clipboard.writeText`, as arigl-web does.
  - **tvision** (`ari run --backend tvision`): tvision's `TClipboard`. On Windows that is the
    system clipboard. On Unix it uses `wl-copy`, `xsel` or `xclip` when there is a display, else an
    OSC 52 escape.
- **`(fullscreen ON)`** makes the window cover the screen (`ON` truthy) or not, and returns `#t`
  when the host did, or asked the browser to. **`(fullscreen)`** says whether it covers the screen
  now. In a GL window it is the OS window, borderless at the desktop's mode. In the browser it is
  the canvas, and only within a click's user activation, as above. A terminal has no such window:
  both forms return `#f` there, and `(has? "ui.fullscreen")` is false. The verb is bound
  everywhere, so a program can call it without asking first. The whole document stays as it is,
  only larger. For one picture or video shown alone, use `fullscreen: true` on it (§5.13).
- `ariadne_ftxui_test`'s `ClipboardSetIsAnOsc52Escape`, `cvcgl_imgui_fullscreen` and
  `MediaPlayer.AFullscreenVideoIsShownAloneAndComesBack` check them.

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
| `(ws-send H MESSAGE)` | send a text message: a string as it is, a dict or list as JSON; `#t`, or an error when `H` is closed |
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
  §10.3), plus `timeout`, the seconds to connect (15), `protocols` and `auth_protocol` (below).
  A browser's `WebSocket` can't send request headers, so there `headers` and `auth` are
  refused; a server that wants a token takes it as a subprotocol (`auth_protocol`) or in a
  message (`ws-send`), and either way the document never holds it.
- `protocols` is a list of subprotocols to offer (`Sec-WebSocket-Protocol`), each an HTTP token
  (letters, digits and ``!#$%&'*+-.^_`|~``). The server must pick one of those offered, or the
  connect fails: browsers require that, and the native client checks it too.
- `auth_protocol`, `(dict "credential" NAME ["prefix" P] ["encoding" "raw"|"base64url"])`,
  offers the host's token for `NAME` as one more subprotocol, `P` followed by the token. This is
  the one handshake credential a browser can send. A token holding `/` or `=` (token68 may)
  can't be a subprotocol as it is, so use `"encoding" "base64url"` (unpadded, RFC 4648 §5),
  as Kubernetes does:

  ```lisp
  (ws-open "wss://k8s.example/api/v1/watch"
           (dict "protocols" (list "v4.channel.k8s.io")
                 "auth_protocol" (dict "credential" "kube"
                                       "prefix" "base64url.bearer.authorization.k8s.io."
                                       "encoding" "base64url")))
  ```

  The server should pick the real subprotocol, not the credential one. The token is fetched
  for the URL's origin, as for `auth`, and is never in the reply or an error. No credential
  ever goes in the URL: a query refuses one.
- `ws-send` sends a dict or a list as JSON, compact (`json-encode`'s rules: a value JSON lacks
  is an error). A credential value in it, `(dict "credential" NAME)` at any depth, is sent as
  the token the host keeps for `NAME`, a JSON string, so a document signs in to a live feed the
  one way a browser allows, in a message:

  ```lisp
  (set r (ws-open "wss://tx.wtf/ws/events"))
  (ws-send (get-attr r "handle") (dict "type" "hello" "token" (dict "credential" "txwtf")))
  ;; the server gets {"type":"hello","token":"<the host's txwtf token>"}
  ```

  The token is fetched for the connection's origin (its URL's `http(s)` twin, as for the
  policy), so a credential bound to other origins (§10.3) is not sent, and is put in on the
  host just before the message goes; no value, state node, error or log line holds it. A name
  with no token there is an error and sends nothing; so is a credential value on a host that
  keeps no credentials. As with HTTP replies, a token a connection has sent (in its handshake
  or a message) is blanked to `[redacted]` in what `ws-recv` and `ws-poll` return, should the
  server echo it (in its raw spelling, or in base64url as an `auth_protocol` sends it). A string
  is sent as it is: a document can't put a token in one.
- The host's policy (§10.3) checks the URL as its `http(s)` twin: `--net-allow
  https://example.org` allows `wss://example.org/...`.
- `ws-open` and `ws-recv` are async I/O calls like the HTTP verbs: the connect, or the wait,
  runs on the background pool and the program parks. `ws-poll` is the one to call from an
  `on_tick` that also does other work: it takes what has arrived and returns.

## 11. Caveats — what is not yet wired

State plainly so authors are not misled. Each item cites the code; the roadmap items
below (G17–G21 in the internal gap report) are tracked for reconciliation.

- **The scripting hosts are partial** (§9.6,
  [roadmap](roadmap/ARIADNE_SCRIPTING_HOSTS.md)). Only the top document's `python:` block runs: a
  `load:`ed module's is not run yet. A Python program runs in a window, offscreen or in `arix`, not
  under `ariglx --terminal`, and its embedded `Host.load()` from inside a frame raises (load before
  `run()`). `ariglx-web` (the browser) is single-threaded only, with numpy as its one C-extension
  package, and its `host.run()` returns at once. There is no `arisrvx` yet. The `arix` and `ariglx`
  packages are Linux-only, where `pycvc-cp312` and `pycvc-gl-cp312` are; elsewhere build them
  in-tree with the bindings (`-DCVC_BUILD_PYCVC=ON`).

- **Sign-in checks an ID token's claims, not its signature** (§10.5): the token comes straight from
  the token endpoint over TLS. A sign-in kept in a `--credential-store` file is picked up only when
  the document calls `oidc-sign-in` again (with `"interactive" #f`); until then its access token is
  sent but not refreshed.
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
  Windows, Linux (X11 and Wayland), macOS and in the browser cvcGL's onscreen window is SDL's:
  SDL makes the window and its OpenGL context (in the browser the page's canvas, filling its
  container, and WebGL2), VTK renders into it, and the window's interactor turns SDL's events
  into VTK's (the camera and ImGui see them as before) and hands the same events to
  `cvc::gl::SdlInput::attach`, keys and pointer alike. `$CVCGL_SDL_WINDOW=0` (in the browser,
  the page's `ENV`) keeps VTK's own window; the documents then get VTK's key events, renamed as
  SDL names them, and no pointer, and in the browser SDL listens on the page for the keys typed
  into VTK's canvas (a hidden canvas of SDL's own). arigl hands the
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
document's host verbs, functions (§6.6) and limits, save time. A running app gives a program 0.1 s
(`ariadne.limits.action_max_time`). Under `ari test` every program may run as long as the run's
`--timeout`, so a Debug build or a slow CI runner doesn't kill one that is merely slow, while the
step cap still stops a runaway. `--action-time S` sets it: `0.1` holds a document to what an app
allows, and `0` lifts it. The tests run one at a time, in document order, starting
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
does, until its requests have come back (at most 10 s, or `--settle S`), and a warning in that
time fails it too.
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

