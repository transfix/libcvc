# The Ariadne document format

This is the normative specification of an Ariadne document, a `.ari` file: what a document may
contain, and what a reader does with it. It has two halves.

- **Structure** is the JSON Schema
  [`src/cvc/ariadne/schema/ari.v1.schema.json`](../src/cvc/ariadne/schema/ari.v1.schema.json):
  which keys a document may have, and what shape each value takes.
- **Meaning** is this file: when each part is read, what it does, and what a reader reports.

[ARIADNE.md](ARIADNE.md) is the guide. It explains, shows examples and names the code; it is
informative. Where it and this file disagree, this file is right and the guide has a defect.

**Status: describes libcvc 4.0.10 as of transfix/libcvc#1609.** It covers the document as a whole:
its text (§2), its version gate (§3), loading (§4), starting (§5), frames (§6), programs and state
paths (§7), and every top-level block (§8). The scene's node semantics, the widget types and
the DSL's built-ins are not written here yet (§10); for those the guide is the reference. Where
libcvc does not do what this file says, §9 lists the difference. `ariadne_schema_test` checks
that §8 has one entry for each top-level key the schema allows, and none for any other.

## Contents

1. [Conformance](#1-conformance)
2. [Document text](#2-document-text)
3. [Versions: `meta.min_libcvc`](#3-versions-metamin_libcvc)
4. [Loading](#4-loading)
5. [Starting](#5-starting)
6. [Frames](#6-frames)
7. [Programs and state paths](#7-programs-and-state-paths)
8. [Top-level blocks](#8-top-level-blocks)
9. [Where libcvc 4.0.10 differs](#9-where-libcvc-4010-differs)
10. [Not yet specified](#10-not-yet-specified)

## 1. Conformance

The key words MUST, MUST NOT, REQUIRED, SHALL, SHALL NOT, SHOULD, SHOULD NOT, RECOMMENDED, MAY
and OPTIONAL are to be read as described in BCP 14 ([RFC 2119], [RFC 8174]) when, and only when,
they appear in capitals.

[RFC 2119]: https://www.rfc-editor.org/rfc/rfc2119
[RFC 8174]: https://www.rfc-editor.org/rfc/rfc8174

There are two conformance classes.

- **A conforming document** is valid against the schema and keeps every rule this file gives a
  document.
- **A conforming reader** loads, starts and runs every conforming document as this file says,
  and reports what this file says to report. libcvc's loader and runtime (`cvc::ariadne`) are
  one reader; the hosts built on them (`ari`, cvcGL's `arigl`, the scripting hosts, the relay's
  server documents) are readers through them.

The schema decides which keys and value shapes are allowed. This file decides what they mean and
when they act. If this file and the schema disagree about structure, the schema is right and this
file has a defect.

Two words name what a reader does with a problem.

- **Warn:** add a diagnostic to the load's warnings (or, at run time, to the frame's warnings and
  the notification feed), and go on. A host decides how strict to be; `ari test` fails a document
  that warns.
- **Refuse:** the load fails. The reader returns an error, applies nothing of the document, and,
  when it is reloading, keeps the document it was running.

## 2. Document text

### 2.1 YAML

A document is a [YAML 1.2](https://yaml.org/spec/1.2.2/) file, read by yaml-cpp.

- A document MUST be UTF-8. A reader MAY accept a byte-order mark before it.
- A file MUST hold one YAML document. A reader reads the first and SHOULD warn about any after it,
  even one that does not parse.
- Anchors and aliases MAY be used. An alias stands for a copy of its anchor's node. An alias with
  no anchor is a YAML error.
- Merge keys are not part of the format: `<<` is an ordinary key, so the schema reports it as an
  unknown key in every closed map.
- Tags are not part of the format. A reader ignores them.
- A map MUST NOT repeat a key. A reader SHOULD warn about a repeated key, and reads the first of
  them wherever it reads the map: by looking a key up, by going through its keys, or in the
  schema check.
- A YAML error in the document refuses the load. A YAML error in a library it imports warns and
  the import is skipped. A YAML error in a fragment it `load:`s warns and the mount is an empty
  group.

### 2.2 Scalars

A scalar is text, which each key reads as it needs. Quoting changes only what YAML itself needs
quoted: `"1"` and `1` are the same value, and so are `"true"` and `true`.

- **A flag** (`$defs/flag`) is a boolean: `true`/`false`, `yes`/`no`, `on`/`off` or `y`/`n` in
  any case, or `1`/`0`.
- **A number** is a scalar that is wholly a finite number.
- **A program** (`$defs/program`) is a string holding a state_exec program, whose syntax is
  [STATE_EXEC_GRAMMAR.md](STATE_EXEC_GRAMMAR.md), or a bare name that a host handles (§7.5).

### 2.3 The root, and the three uses of a file

The root of a file is a map, which is a document, or a sequence, which is a list of widgets and
means the same as a document holding only `root:` with that list. A reader warns about a scalar
root and loads nothing from it.

The same file shape serves three uses, and each reads a different part of it.

- **Run as the document.** Every block in §8 applies.
- **Imported as a library** (`import:`, §8). Only its `import:`, `units:` and `functions:` are
  read; a reader MUST ignore its other blocks.
- **Mounted as a fragment** (`load:` or a `mounts:` entry, §4.4). Its widgets and its `init:`,
  `state:`, `on_tick:`, `requires:`, `needs:`, `import:`, `units:`, `functions:` and `meta:` are
  read. A reader warns about its `on_key:`, `on_pointer:`, `on_signal:`, `context_menu:` and
  `args:` and ignores them. Its other blocks are ignored.

### 2.4 Extension keys

A key that starts with `x-` is an extension key. It MAY appear in the document map and in every
widget map. A reader MUST ignore it. Tools MAY use extension keys for their own data.

## 3. Versions: `meta.min_libcvc`

`meta.min_libcvc` is the oldest libcvc the document needs. A document SHOULD declare it, and the
schema requires it whenever `meta:` is present. A reader warns when a document declares none.

A version is compared as up to three numeric components, `major.minor.patch`.

- Text from the first `-` or `+` on is ignored, so `4.1.0-rc1` and `4.1.0+abc` are `4.1.0`.
- A `v` or `V` and spaces are skipped. Any other character ends the version.
- A missing or empty component is 0, so `4` is `4.0.0` and `latest` is `0.0.0`.

A reader MUST refuse a document whose `min_libcvc` is newer than the reader, before it checks or
reads anything else in it. A document too new for the reader may use keys the reader's schema
does not know, so it is refused, not checked. Each fragment carries its own `meta:` and is gated
the same way when it is mounted, on its own: a `load:` fragment too new for the reader warns and
is an empty group, and a `mounts:` entry is refused when it opens. Libraries are not gated.

In this version of the format `min_libcvc` selects no behaviour. It is only the gate: no key
means something different because of it.

## 4. Loading

### 4.1 The order of loading

A reader loads a document in these steps. A refused step ends the load.

1. **Read and parse** the file (§2.1). A YAML error refuses.
2. **The version gate** (§3). A document too new refuses.
3. **The schema check** of the document (§4.2).
4. **`customs:`.** A required custom type or block the reader lacks refuses; an optional one warns.
5. **`channels:`, `lint:` and `args:`.** A malformed `args:` refuses.
6. **Composition** (§4.4): the document's imports, recursively, then its own `units:` and
   `functions:`.
7. **The widget tree** (§8, UI): `menubar:`, then `windows:`, then `overlays:`, then `root:` (or
   `children:`), then the widgets the settings make (`windows_menu:`, `clock:`,
   `notifications:`). Each `include:` and `load:` expands where it stands. Then the document's
   `context_menu:`.
8. **The widget nesting check** (§4.5). Past the limit refuses.
9. **The scene:** `prototypes:`, `scene:`, `data:`, `assets:`, `scenes:`. Then the scene nesting
   check (§4.5). Past the limit refuses.
10. **The programs** are taken: `init:`, `state:`, `on_tick:`, `on_key:`, `on_pointer:`,
    `on_signal:`, `keys:`, `tests:`. None runs yet.
11. **The channel lint** (§8, `channels:`). A violation under `strict` refuses.
12. **Host blocks** (§4.6): each registered block's parser runs on its value.

The order the top-level keys are written in does not matter, with three exceptions:

- registered host blocks are handed to their parsers in the order they are written;
- the order of a widget list is the order of its widgets, and of the mount ids their `load:`s
  get (§4.4);
- where two definitions clash and the later one wins (imports, functions), "later" is document
  order.

### 4.2 The schema check

A reader that has a JSON Schema validator checks the document against the schema right after the
version gate. It also checks each library when it imports it, each fragment when it mounts it
(after the mount's `args:` are substituted), and each `include:` expansion, as a widget, after
its args are substituted and its slots filled. A reader without a validator checks nothing, and
says so when asked to validate.

- Each problem warns, with the file, line and column it is at, the context (`include <unit>`
  for an expansion) and the JSON pointer of the value. A schema problem never refuses a load.
- The check goes at most 128 levels of YAML deep (`kMaxSchemaDepth`), counting the document, or
  the `include:` expansion, as level 1. A file deeper than that is not checked; instead it gets
  one problem at its first value past the limit.
- The allowed top-level keys are the schema's, plus each block a host has registered (§4.6) and
  each block the document declares in `customs:`.

### 4.3 What refuses, and what only warns

A load is refused for:

- a YAML error in the document (§2.1);
- a `min_libcvc` newer than the reader (§3);
- a required `customs:` entry the reader lacks;
- a malformed `args:`;
- a widget tree or scene past the nesting limit (§4.5);
- a channel violation under `lint: {channels: strict}`;
- at start, a required `requires:` entry the host lacks (§5.1), or bad arguments when the host
  validates them.

Everything else this file calls a problem warns: schema problems, unknown widget keys, an
unknown unit, an import or fragment that cannot be read, a cycle, a limit on imports or mounts.

### 4.4 Composition

`include:`, `import:`, `load:`, `functions:` and `mounts:` are the five ways a document uses other
text. The guide's §6 explains each with examples.

**`include:`** instantiates a unit (a widget template, from `units:`) where it stands.

- The unit is copied, and each `{name}` token in its values is replaced by the site's arg or else
  the unit's declared default. Tokens are replaced in values only, never in keys, in one pass;
  args are scalars; a token no arg names stays as written.
- Slots are filled after args are substituted.
- The expansion shares the scope it stands in: its binds resolve as the site's do.
- A unit that includes itself, directly or not, warns and is not expanded again. An unknown unit
  warns and is an empty group.

**`import:`** merges libraries' `units:` and `functions:` into the document.

- A library's URI resolves against the importing file's directory. A library's own imports are
  merged before its units.
- When two libraries define the same name, the later one wins and the reader warns. The
  document's own unit wins over every library's, without a warning, unless it is marked
  `fallback:` (a flag): then it is used only where no library defines its name.
- A library imported twice, or in a cycle, is read once; the repeat is skipped without a warning.
- Imports nest at most 32 deep, and at most 4096 libraries are read. Past either warns.

**`load:`** mounts a fragment (§2.3): an isolated module.

- The fragment is parsed on its own: its own units, its own base directory, nothing imported by
  the document.
- Its mount id is its `as:`, or else its URI's file name, with every character outside
  `[A-Za-z0-9_]` replaced. A second mount with the same id at the same level gets `_2`, a third
  `_3`, and the reader warns. The mount's scope is `includes.<id>`, under the scope it stands in
  (§7.2).
- A fragment that mounts itself, directly or not, warns and is an empty group. Mounts nest at
  most 32 deep, and a document mounts at most 256 fragments. Past either warns, and the mount is
  an empty group.
- The mount's own keys (`as`, `name`, `args`, `functions`, `link`, `channels`) are evaluated in
  the scope it stands in. Its `args:` are substituted into the fragment before it is parsed, as an
  `include:`'s are.

**`functions:`** are `(defun …)` forms every program of the document can call. Imported
functions come first, then the document's own. A document's own function replaces an imported
one of the same name, without a warning unless the one it replaces was granted to it by a mount,
which warns. A form that is not a `defun` warns and is skipped.

**`mounts:`** are slots a document opens at run time. The loader reads only the slot; each entry
is loaded when it opens, as a `load:` fragment is, with its own version gate (§3) and
`requires:` check (§5.1). An entry that is refused is not mounted; the document goes on. A
document has at most 256 mounts open, nested at most 32 deep.

**URIs.** A reference with `://` names its scheme; anything else is a file path. A relative path
resolves against the referring file's directory (or, for a document read from a string, the
working directory). `file` reads regular files only, up to 16 MiB. `cvc://` searches
`CVC_ARIADNE_PATH`, then the host's extra directories, the install's, the compiled-in data
directory, then the working directory. A scheme no handler serves is an error for that
reference: the import, mount or picture it names is skipped with a warning.

### 4.5 How deep a document nests

A widget tree MUST NOT nest deeper than 256 levels (`kMaxNesting`). Each widget is a level, and so
is each `include:` and each `load:` on the way to it. A scene MUST NOT nest deeper than 256
levels either, where each node and each `use:` of a prototype is a level. A reader refuses a
document past either limit, and the error names the file, line and column of the first widget or
node past it and the `include:`s, `load:`s or `use:`s on the way. The guide's §6.7 gives the
stack each level costs.

### 4.6 Host blocks

A host MAY register a parser for a top-level key the loader does not read itself
(`register_ari_block`). A registered parser runs at the end of the document's load, on the
block's value, and may add data and warnings to the load. Registering one of the loader's own
blocks does nothing. Host blocks are read from the document only, never from a library or a
fragment.

A top-level key that is neither the loader's nor registered is ignored. The schema check warns
about it, unless the document declares it in `customs:` as a block. A reader SHOULD also warn
about a script or network block (`python:`, `wasm:`, `server:`, `relay:`) that no host in the
process handles, since the document then runs without its code.

## 5. Starting

### 5.1 The order of starting

When a host runs a loaded document, it does these steps in order.

1. **`requires:`** (§8). A required entry the host lacks refuses, naming every missing entry, so
   nothing of the new document is applied. An optional entry it lacks warns. A required entry of
   a `load:` fragment refuses the whole document; a required entry of a `mounts:` entry refuses
   only that entry.
2. **`args:`** are parsed from the host's command line and published read-only: the list at
   `argv`, and each argument at `args.<name>`. A host that validates arguments refuses bad ones.
3. **`data:`** entries are linked at `<prefix>.data.<name>`.
4. **`keys:`**: each binding's keys are written at `keys.<id>`. A new document overwrites them; a
   reload of the same document writes only the ones that are unset, so the user's rebindings
   survive it.
5. **`state:`** seeds are written (§8).
6. **`init:`** runs, after the document's `functions:`, as one program (§7.5). If it fails, the
   reader warns and the document goes on.
7. The resident handlers (`on_tick:`, `on_key:`, `on_pointer:`, `on_signal:`) are started, and
   the channel policy (`channels:`) is installed.
8. The scene is realized, on a host that draws scenes. A host that does not warns that the scene
   is ignored.
9. The widget tree is handed to the next frame.

Every step after the first sees the document's `args:`, so a seed, a data entry or `init:` may use
them.

### 5.2 Reload

A host MAY reload a document when its files change. A reload runs §4 and §5 again. If the new
load is refused, the host keeps running the old document. On a reload:

- `state:` is written and `init:` runs again;
- the top-level resident handlers are stopped and started again;
- the state tree is not cleared, so values the old document wrote stay unless the new one writes
  them;
- a `mounts:` entry whose files changed is reloaded on its own, without reloading the document.

A `load:` module's `state:` and `init:` run once per mount, the first time the mount is drawn
(§6), and again only after a Restart.

### 5.3 Restart

Restart applies to an application, a document mounted as one (the guide's §5.25). It:

1. stops the application's resident handlers and its running actions;
2. removes its state subtree and clears its notifications.

The next frame then plants its holes again, writes its `state:`, runs its `init:` and starts its
`on_tick:`, as at its first mount.

## 6. Frames

A reader runs a document in frames. Each frame has two parts: the drain, where programs run, and
the walk, where the widget tree is drawn.

**The deferred-intent rule.** Drawing a widget never runs a handler. When a widget fires (a click,
an edit, a key binding), its handler is queued, and it runs in the next drain, before the next
walk. A program therefore never runs in the middle of drawing, and the walk sees the state the
drain left. There are three exceptions, all writes the walk makes itself:

- a widget's committed edit writes its `bind:`;
- a widget's `def:` seeds its `bind:` when the key is unset;
- a `load:` module's holes, `state:` and `init:` are planted, written and run when the module is
  first drawn (§5.2).

**In the drain:**

- queued handlers are submitted, in the order they were queued;
- `on_tick:` is posted one tick, unless one is already waiting, so ticks never pile up;
- key and pointer events are delivered to `on_key:` and `on_pointer:` in the order they happened,
  none merged or dropped;
- the scheduler then runs every program, under the frame's budget (§7.5).

**In the walk:** a pending new tree (from a reload) replaces the old one first, and the old one's
holes are torn down. Then `mounts:` are reconciled with their slots, and the tree is drawn.
Read-lane expressions (§7.5) are evaluated as their widgets are drawn.

## 7. Programs and state paths

### 7.1 The bind rule

Every state path a document writes outside a program (a `bind:`, a `state:` key, a `data:` name, a
hole's target) resolves against the scope it is written in:

- a path with a leading `/` is from the application's root, with the `/` removed;
- otherwise it is `<scope>.<path>`, or the path as written when the scope is empty;
- an empty path stays empty.

### 7.2 Scopes

- **The document's scope** is a prefix the host chooses. Hosts that run several documents use
  `ui.docs.<id>`; a document that drives a scene uses the scene's prefix. A document MUST NOT
  depend on the prefix's value.
- **A mount's scope** is its `includes.<id>` resolved against the scope the mount stands in
  (§4.4). The mount's own keys are evaluated in the outer scope; everything in the fragment, in
  the mount's.
- **A repeated mount** (`repeat:`) gets `.<i>` after its scope, so each instance has its own.

### 7.3 Confinement and holes

A program is confined to its scope. Its `state-*` paths are relative to the scope, and it has no
way to name anything outside it: no leading `/`, no `..`.

A mounted module sees nothing of its parent's state unless the mount grants it a hole with
`link:`.

- `name: target` grants `name`, inside the module's scope, as a read-write view of `target`,
  which resolves in the scope the mount stands in.
- `name: {to: target, mode: ro|rw}` grants it read-only unless the mode is `rw`. A missing mode
  is read-only. Any mode but `ro` or `rw` warns and is read-only. Modes compare without
  regard to case.
- A hole name with a leading `/` would escape the module's scope: it warns and is skipped.
- A read through a hole reads the target; a write through a read-write hole writes the target,
  and a write through a read-only hole stays inside the module.
- Holes are planted when the mount is first drawn and torn down when its tree is replaced.

A mount's `channels:` does for message channels what `link:` does for state: it names the
channels outside its own that the module may use. A mount's `functions:` names the document's
functions the module may call.

### 7.4 Isolation

A host MAY run each document in its own scheduler group, with its own budget, so that one busy
document does not starve another (the guide's §2.5). A document stays in the shared group when it
has no prefix, uses sign-in (`oidc`), or has a read-write hole whose target lies outside its
prefix.

### 7.5 The two lanes

Programs run in one of two lanes.

**The read lane** evaluates the expressions a widget reads as it is drawn: `bind:` expressions,
`visible_when:`, `enabled_when:`, `disabled_when:`, `tooltip:` when it starts with `(`,
`options:`, `repeat:` and `color:`.

- A read-lane expression MUST NOT write state, send a message, or wait. Only the read-lane
  built-ins (and `has?`) are available, and only these special forms: `if`, `begin`, `and`,
  `or`, `let`, `while`, `for`, `lambda`, `defun`, `set`, `return`, `quote`, `yield`, `break`.
- Each expression runs under a budget: by default 100,000 steps and 5 ms, and all of a frame's
  together 30 ms.
- An expression that fails makes its widget fail safe: a failed `visible_when:` hides, a failed
  `enabled_when:` disables, and a failed value keeps its last good value. The reader warns once
  per expression. An expression skipped because the frame's budget ran out keeps its last good
  value without a warning.

**The action lane** runs everything else: widget handlers (`on:`, `on_click:` and the other
`on_*` keys), key bindings, `init:`, the resident handlers and `tests:`.

- An action has the full environment. Its writes are confined to its scope (§7.3), and a pointer
  handler gets an `event` dict.
- Each action runs under a budget: by default 5,000 steps, 0.1 s and 4 MiB. `init:` gets
  10,000,000 steps and 5 s. A host MAY change these.
- An action MAY wait (on a message, a timer, a network reply); it then resumes in a later drain.
- An action that is a bare name, not a program, is handed to the host's handler of that name. A
  name the host does not handle does nothing.
- An action that fails, or runs out of budget, ends; the reader warns once.

**Resident handlers** are the document's `on_tick:`, `on_key:`, `on_pointer:` and `on_signal:`.
Each is one long-lived program that waits for an event, binds it to `event`, runs its body, and
waits again.

- `on_tick:` gets the string `"tick"` once per frame.
- `on_key:` and `on_pointer:` get a dict with `kind`, `key`, `mods`, `repeat`, `x`, `y`, `dx`,
  `dy`, `button` and `clicks`.
- `on_signal:` gets `{kind: "signal", signal, source}`. A document with `on_signal:` handles the
  signal: the host does not apply its default (quitting, for Ctrl-C).
- Each step of a resident runs under a step budget (by default 0.05 s; 1 s in the browser), with
  no total cap.
- A resident that fails or is killed warns and is started again: at once, then after 1, 2, 4 …
  up to 64 frames. One that stops 8 times within 100 frames of starting is given up, with a
  warning. One that cannot start at all is given up at once.

## 8. Top-level blocks

Each top-level key the schema allows has an entry here. **Schema** names its structural half;
**When** says in which step (§4.1, §5.1, §6) it acts; **Guide** points at the explanation.

### 8.1 The document

#### `meta:`

**Schema:** `$defs/document/properties/meta`. **When:** load step 2.

Provenance and the version gate. `name` names the document: as an application (the guide's
§5.25) and in process views, and the host writes it to `ariadne.name`. `author`, `description`
and `version` are provenance only; a reader MUST NOT change behaviour by them. `min_libcvc` is
the gate (§3). A fragment's `meta:` gates the fragment. **Guide:** §5.2.

#### `customs:`

**Schema:** `$defs/document/properties/customs`. **When:** load step 4.

The custom types and blocks the document uses: each entry names one `widget:`, `node:` or
`block:`, with `required:`. A required widget or block the reader lacks refuses the load; an
optional one warns. A declared block is allowed by the schema check (§4.2). A node type is checked
by the host that draws scenes. A custom widget type the reader cannot draw is drawn as a
placeholder. **Guide:** §5.1, §5.4.

#### `requires:`

**Schema:** `$defs/document/properties/requires`. **When:** start step 1.

What the document needs from its host: capability tokens, functions and widget types, each
required or optional. See §5.1 for what a missing entry does. A program asks the same questions
at run time with `(has? …)`. **Guide:** §5.7.

#### `args:`

**Schema:** `$defs/document/properties/args`. **When:** parsed in load step 5, published in start
step 2.

The document's command line: `name: type`, or `{type, default, help, required, positional}`. The
arguments are read-only state: the list at `argv`, each at `args.<name>`. A malformed block
refuses the load. A fragment's `args:` warns and is ignored; a mount passes values to a fragment
with its own `args:` key instead (§4.4). **Guide:** §5.6.

#### `lint:`

**Schema:** `$defs/document/properties/lint`. **When:** load step 5, applied in step 11.

How strictly the document's channel use is checked: `channels: strict` (the default), `warn` or
`off`, and `quiet`. Any other value warns and means `strict`. It applies only when the document
has `channels:`. **Guide:** §8.

#### `channels:`

**Schema:** `$defs/document/properties/channels`. **When:** load steps 5 and 11; start step 7.

The message channels the document's programs use: each a name, or `{channel, shared, global}`.

- At load, every program of the document (`init:`, the residents, `tests:`, `functions:`, key
  bindings and widget handlers) is checked for a channel it does not declare. Under `strict` a
  use refuses the load; under `warn` it warns; under `off` it warns unless `quiet`.
- At run time, a program's `msg-send`, `msg-recv` and `msg-pending` on an undeclared channel
  fail. Channels starting with `#` (the runtime's own), declared globals and channels a mount
  grants are always allowed.

A document with no `channels:` is not checked. **Guide:** §8.

### 8.2 Composition

#### `import:`

**Schema:** `$defs/document/properties/import`. **When:** load step 6.

Libraries whose `units:` and `functions:` the document uses: each a URI, or a map with `uri:` and
`optional:`, a flag. A library that cannot be found warns, unless it is optional; one that does
not parse warns either way. **Guide:** §6.2.

#### `units:`

**Schema:** `$defs/document/properties/units`. **When:** load step 6; expanded in step 7.

Widget templates, by name, that `include:` instantiates (§4.4). A unit may declare `args:` with
defaults, and `fallback:`, a flag, to yield to a library's unit of the same name (§4.4). A unit
is checked as a widget when it is included, after its args are substituted. Units have no effect
until included. **Guide:** §6.1, §6.2.

#### `functions:`

**Schema:** `$defs/program`. **When:** load step 6; run as the prelude of every program.

`(defun name (params) body…)` forms that every program of the document can call (§4.4). A mount
MAY grant its functions to the module it mounts; grants carry down through nested mounts.
**Guide:** §6.6.

### 8.3 State and programs

#### `data:`

**Schema:** `$defs/document/properties/data`. **When:** load step 9; linked in start step 3.

Datasets in the state tree: `name: URI`, or `{uri | generate, path, as, params, load}`. Each is
linked at `<prefix>.data.<name>` from a store the host shares between documents. A `uri:` MAY be
`{arg: <name>}`, taken from the document's arguments. A lazy entry loads when it is first asked
for. A fragment's `data:` is ignored. **Guide:** §5.17.

#### `keys:`

**Schema:** `$defs/document/properties/keys`. **When:** load step 10; seeded in start step 4.

Rebindable key actions: `id: {label, key, on}`. The keys bound to each action are state, at
`keys.<id>`, so the user may rebind them; the host adds a Controls window for that. A key-down
that matches queues the action's `on:` for the next drain (§6). A fragment's `keys:` is ignored.
**Guide:** §5.3.

#### `state:`

**Schema:** `$defs/state_tree`. **When:** load step 10; written in start step 5, and for a
fragment when its mount is first drawn.

Starting values for state keys: a map of paths to literal values, written under the document's
scope (§7.1) just before `init:` runs, on every load and reload.

- A map's keys are path segments joined by dots. A key MAY hold dots itself. A top-level key
  starting with `/` is from the application's root; no other key may start with `/`.
- A scalar is written as written (§2.2): `paused: 0` writes `0`, `ready: true` writes `true`.
- A list of scalars is written as its items joined by commas.
- A key with no value writes `""`.
- A key that is not a path (an empty segment, `..`, a leading or trailing dot) warns and is
  left out. A list holding a map warns and is left out. A key given twice warns, and the later
  value is used.

Since `init:` runs after it, `init:` reads the seeds and may override them, and a seed wins over
a widget's `def:`, which seeds only an unset key. A fragment's `state:` is written at its mount's
scope, once per mount (§5.2). **Guide:** §5.3.

#### `init:`

**Schema:** `$defs/program`. **When:** start step 6, and for a fragment when its mount is first
drawn.

A program that runs once per start, after `state:`, with the document's `functions:` before it.
It runs in the action lane with `init:`'s budget (§7.5). If it fails, the reader warns and the
document goes on. A fragment's `init:` runs in its mount's scope, once per mount and again after a
Restart (§5.2, §5.3). **Guide:** §5.3.

#### `on_tick:`

**Schema:** `$defs/program`. **When:** started in start step 7; one tick each frame (§6).

The tick resident (§7.5). A fragment's `on_tick:` runs in its mount's scope, while the mount is
drawn; it keeps running across a reload when its scope and program are unchanged. **Guide:** §2.3.

#### `on_key:`

**Schema:** `$defs/program`. **When:** started in start step 7; each key event (§6).

The key resident (§7.5). A fragment's warns and is ignored. **Guide:** §2.3.

#### `on_pointer:`

**Schema:** `$defs/program`. **When:** started in start step 7; each pointer event (§6).

The pointer resident (§7.5). A fragment's warns and is ignored. **Guide:** §2.3.

#### `on_signal:`

**Schema:** `$defs/program`. **When:** started in start step 7; each signal the host receives.

The signal resident (§7.5). Its presence means the document handles signals itself. A fragment's
warns and is ignored. **Guide:** §2.3.

#### `tests:`

**Schema:** `$defs/document/properties/tests`. **When:** load step 10; run only by `ari test`.

`name: program`. A reader that is not testing ignores them, apart from checking them for the
channels and host functions they use. Under `ari test`, after the first frame, each test runs as
an action, one after another. `(quit [code])` ends the run with a code from 0 to 255, and the
first code stands; `(assert …)` fails the test. A document fails its test run on a load warning
or error, on any warning in a frame, on a failed test, on a non-zero quit or a quit before every
test has ended, or on a timeout (30 s unless the host says otherwise). **Guide:** §13.

### 8.4 The user interface

The widget tree's root is one group. Its children are, in this order: the menu bar
(`menubar:`), the items of `windows:`, the items of `overlays:`, the items of `root:` (or of
`children:`), and the widgets the settings make (`windows_menu:`, `clock:`, `notifications:`).
Each frame draws, in order: a menu bar the host adds (only when the document has no `menubar:` and
has a closable window or a mounted application), the root, the document's `context_menu:`, and
the host's own windows (notifications, budgets, file pickers).

#### `menubar:`

**Schema:** `$defs/widget_list`. **When:** load step 7.

The main menu bar's menus. It becomes one menu-bar widget whose children are the list.
`menu_collapse:` applies to it. **Guide:** §5.9, §5.23.

#### `menu_collapse:`

**Schema:** `$defs/collapse`. **When:** load step 7; decided each frame.

Whether the menu bar folds its menus into one: `auto` (when they do not fit), `never` or
`always`; a flag means `auto` (true) or `never` (false). Without `menubar:` it does nothing.
**Guide:** §5.23.

#### `windows:`

**Schema:** `$defs/widget_list`. **When:** load step 7.

A list of widgets, normally windows. Nothing makes an item a window but its own type. A window
with no `pos:` is placed in a column, in document order; which window is in front is the
backend's, and a click, the Windows menu or `tree.<id>.raise` raises one. A window's identity is
its `id:`, else its title; its open state, geometry and maximized state are kept at
`tree.<id>.*`. **Guide:** §5.8, §5.9.

#### `overlays:`

**Schema:** `$defs/widget_list`. **When:** load step 7.

A list of widgets drawn after `windows:`, normally windows over the scene. It is read exactly as
`windows:` is. **Guide:** §5.8.

#### `root:`

**Schema:** `$defs/widget_list`. **When:** load step 7.

Top-level widgets outside any window. They are siblings of the windows in the root group; a
backend draws them in its main area. **Guide:** §4, §5.1.

#### `children:`

**Schema:** `$defs/widget_list`. **When:** load step 7.

The same as `root:`, which it stands for. A document SHOULD NOT have both; when it does, a reader
warns, uses `root:` and ignores `children:`. **Guide:** §5.1.

#### `context_menu:`

**Schema:** `$defs/context_menu`. **When:** load step 7; drawn each frame after the tree.

The main view's own context menu: the menu a right-click on the document's background opens. A
fragment's top-level one warns and is ignored; a window or group within it may have its own.
**Guide:** §5.30, §5.28.

#### `windows_menu:`

**Schema:** `$defs/document/properties/windows_menu`. **When:** load step 7; planned again at
each tree swap.

The built-in Windows menu: `false` turns it off, and a string renames it. **Guide:** §5.9.

#### `clock:`

**Schema:** `$defs/flag`. **When:** load step 7.

Whether the tray shows a clock, at the menu bar's right end. (A scene's `clock:` is a different
thing: the scene's time.) **Guide:** §5.22.

#### `notifications:`

**Schema:** `$defs/document/properties/notifications`. **When:** load step 7.

Whether the tray shows the bell for the host's notification feed: `true`, `false`, or `always`
(shown even when the feed is empty). **Guide:** §5.24.

### 8.5 The scene

These blocks are read in load step 9 and realized in start step 8, by a host that draws scenes.
The semantics of the scene's nodes are not yet written here (§10).

#### `scene:`

**Schema:** `$defs/scene`. **When:** load step 9; start step 8.

The document's main scene: its nodes, lights, cameras, animations, environment, labels, main
view, shadows, chrome, background, clock, physics, world and wind. A viewport with no `scene:`
draws it. A fragment's `scene:` is ignored. **Guide:** §5.4, §5.18.

#### `scenes:`

**Schema:** `$defs/document/properties/scenes`. **When:** load step 9; start step 8.

More scenes, each with the keys of `scene:` and a name. A viewport's `scene: <name>` draws one.
Each lives under `<prefix>.scenes.<name>`. An entry with no name warns and is dropped. A document
MAY have `scenes:` and no `scene:`; its main view is then empty. **Guide:** §5.18.

#### `assets:`

**Schema:** `$defs/document/properties/assets`. **When:** load step 9.

Meshes loaded once and shared by every scene: `id: {source, fit, …}`, used by a node's
`source: {asset: id}`. Paths resolve against the document's directory. **Guide:** §5.18.

#### `prototypes:`

**Schema:** `$defs/document/properties/prototypes`. **When:** load step 9.

Node trees, by name, that a node's `use:` stands for, with `{name}` args. Each `use:` is a level
toward the scene's nesting limit (§4.5). The schema checks a prototype's `args:` only; its body is
checked where it is used. **Guide:** §5.18.

### 8.6 Host blocks

These keys belong to hosts (§4.6). The schema knows them so that a document using them is valid;
each means something only to the host that reads it.

#### `needs:`

**Schema:** `$defs/strings`. **When:** when the fragment is mounted.

A fragment's own: the `link:` holes it expects its mount to grant. In a document run on its own
it has no effect. **Guide:** §7.4.

#### `catalog:`

**Schema:** `$defs/document/properties/catalog`. **When:** never, by the loader.

Tooling data, such as the component library's index (`INDEX.ari`). It is not a widget tree, and
a reader ignores it. **Guide:** §6.5.

#### `python:`

**Schema:** `$defs/document/properties/python`. **When:** load step 12, by a scripting host.

Python for a scripting host (`arix`, `ariglx`): the code as a string, or a map. A reader with no
Python host warns that the document runs without it. **Guide:** §9.6.

#### `wasm:`

**Schema:** `$defs/document/properties/wasm`. **When:** load step 12, by a scripting host.

A WebAssembly module for a scripting host: its path, or a map with `module:`. A reader with no
WebAssembly host warns. **Guide:** §9.6.

#### `server:`

**Schema:** `$defs/document/properties/server`. **When:** load step 12, by the relay.

What the relay that runs the document (`arisrv --doc`) provides; a bare `server:` asks for
nothing. A reader that is not a relay warns. **Guide:** §9.7.

#### `relay:`

**Schema:** `$defs/document/properties/relay`. **When:** load step 12, by a host that joins a
relay.

The relay to join: its URL, or a map with `url:`. A reader that does not join relays warns.
**Guide:** §9.8.3.

#### `world:`

**Schema:** `$defs/document/properties/world`. **When:** load step 12, by the world runtime.

The world's rules (vehicles, spawns, …), read by the world runtime; a bare `world:` sets none.
**Guide:** §9.7.

## 9. Where libcvc 4.0.10 differs

None known: libcvc keeps every rule above. A difference found later is listed here, as a defect in
libcvc to be fixed in the code, until it is fixed.

## 10. Not yet specified

These are normative only through the schema, for now; the guide describes their behaviour.

- **The scene:** node types and their keys; transforms (the guide's §5.4: rotation is degrees
  about x, then y, then z, `R = Rz Ry Rx`); models and their node trees; materials (`material.pbr`)
  and the environment; picking (`on_pick:`); authored viewpoints; prototypes and their expansion;
  keyframe and imported animation; physics; the world. These are the scene-format gaps 1 to 8 the
  libcvc workstream closed in November 2026, and are the next part of this file to write.
- **The widget types:** each of the 35 built-in types' keys and behaviour, the shared widget
  keys, `repeat:` and instance keys, and what each backend draws.
- **The DSL:** the built-ins a program can call and what each does. Its syntax is
  [STATE_EXEC_GRAMMAR.md](STATE_EXEC_GRAMMAR.md).
