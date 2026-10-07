# State Directory Layout Refactor — move state code to `src/cvc/state` + `inc/cvc/state`

Status: **PROPOSED** (supersedes nothing — the distributed-state *runtime* work is
tracked in `DISTRIBUTED_STATE_ROADMAP.md`; this is a pure layout refactor of where the
state code lives).

## Purpose

Today the state API and its runtime are the largest subsystem in libcvc, but its code is
buried inside `core/` — 48 `.cpp` files and 57 headers interleaved with the genuinely
core pieces (app, threading, text, world clock). The ariadne work established the
per-API-subsystem layout we want for every major API surface:

```
src/cvc/ariadne/      ariadne.cpp  app_runtime.cpp  loader.cpp  uri_state.cpp  …
  components/         .ari component library (installed by CMake)
  stream/             frame_pool.cpp  stream_channel.cpp  producer_thread.cpp  …
inc/cvc/ariadne/      ariadne.h  app_runtime.h  bind.h  widget.h  …
  stream/             frame.h  stream.h  …
```

Each API owns `src/cvc/<api>/` + `inc/cvc/<api>/`, compiles into the shared `cvc`
target via **explicit** file lists (no globs — `src/cvc/CMakeLists.txt` L1155/1183),
and is consumed through the `cvc/<api>/…` include namespace. State should get the same
treatment: everything `state_*` lives under `{src,inc}/cvc/state/`, leaving `core/`
for the actual core.

## Current layout (the mess)

`src/cvc/CMakeLists.txt` lists the state pieces inside the `core/` groups:

- **Sources** (L179+ of the `SOURCE_FILES` block): 31 top-level `core/state_*.cpp`
  (+ `core/distributed_state_session.cpp`) and the `core/state_exec/*.cpp` subtree
  (16 files: evaluator, async evaluator, scheduler, builtins, stdlib, …) — **48 total**.
- **Headers** (L25–84 of the `INCLUDE_FILES` block): 36 top-level
  `inc/cvc/core/state_*.h` (+ `distributed_state_session.h`) and 18 of the 20
  `inc/cvc/core/state_exec/` headers — `generator.h` and `utf8.h` are **actively
  included** (tests + `parser.cpp`/`builtins.cpp`) but missing from the CMake list.
- **Consumers**: 246 in-tree files include `<cvc/core/state…>` or
  `<cvc/core/state_exec/…>` (by directory: `src/cvc/tests` 83, `src/cvc/core` 35,
  `inc/cvc/core` 22, `src/cvcGL/test` 21, `state_exec` 32, `src/cvcGL` 14,
  `src/cvc/ariadne` 9, `bindings/pycvc` 3, plus one each in cvc-cli, xmlrpc,
  volslice, volren, lod, gl, ariadne/stream).

Notable consumers on the other side of the library boundary: the ariadne runtime
(`state_io`, `uri_state`, `app_runtime`), the cvcGL viewer, `src/cvc-cli`, and the
SWIG binding layer (`bindings/pycvc/pycvc.i`, `pycvc_state.cpp`, `pycvc_exec.cpp`
— 8 state includes total).

## Target layout

```
src/cvc/state/
  state.cpp  state_list.cpp  state_replica.cpp  state_change_journal.cpp  …  (31 + distributed_state_session.cpp)
  state_exec/   types.cpp  parser.cpp  evaluator.cpp  async_scheduler.cpp  builtins.cpp  stdlib.cpp  …  (16)
inc/cvc/state/
  state.h  state_list.h  state_replica.h  …  (36 + distributed_state_session.h)
  state_exec/  types.h  parser.h  evaluator.h  …  (20)
```

- Everything with the `state_*` prefix moves; `state_exec/` becomes a subdirectory of
  `state/` (same pattern as `ariadne/stream/`), so its includes read
  `cvc/state/state_exec/…`.
- **Stays in `core/`**: `app.{h,cpp}`, `async_lane`, `async_task`, `text.cpp`,
  `thread_pool.cpp`, `world_clock.cpp`, `world_units.cpp`, `exception.h`,
  `namespace.h`, `config.h.cmake` (and the build-generated `inc/cvc/core/config.h`,
  installed at L1694 — core build config, not state).
- **Not moved despite the name**: `inc/cvc/volren/state_settings.h` +
  `src/cvc/volren/state_settings.cpp` — volume-renderer state settings, a volren
  concern, not the state system.
- Ariadne's own state-flavoured files (`ariadne/state_io`, `uri_state`) **stay in
  ariadne**: they are the state system's *consumer* (document I/O + URI state
  transport), which is exactly why they sit in the API that uses them.

## The mechanical work

1. **Move** (one commit, `git mv` so history/blame survives):
   `src/cvc/core/state*` and `src/cvc/core/state_exec/` → `src/cvc/state/`;
   same for `inc/cvc/core/`. Update the two CMake file lists in
   `src/cvc/CMakeLists.txt` (`INCLUDE_FILES` L25–84, `SOURCE_FILES` L179+): path
   prefixes only, same 66 entries (plus adding the missing `state_exec/generator.h`
   + `utf8.h` to `INCLUDE_FILES` — list hygiene, no build change).
2. **Rename the includes**: `<cvc/core/state…>` → `<cvc/state/state…>` and
   `<cvc/core/state_exec/X>` → `<cvc/state/state_exec/X>` in the ~246 in-tree files
   (scriptable: the include paths are unique strings; run the full test suite after).
3. **Install**: nothing to change — headers ship via the bulk
   `install(DIRECTORY inc/cvc/ …)` (L1676), which picks up the new layout
   automatically. The `cvc` target's exported include dir is `inc/`, so consumers
   get `<cvc/state/state.h>` with no CMake change on their side.
4. **SWIG/pycvc**: update the 8 includes in `bindings/pycvc/{pycvc.i, pycvc_state.cpp,
   pycvc_exec.cpp}`; the pycvc recipe regenerates the wrappers, so the pycvc builds
   (cp311–cp313) must be rebuilt after this — no `%include` *name* changes, only
   paths, so no binding-semantics churn.
5. **Docs**: update path references in `docs/STATE_API.md`,
   `docs/STATE_EXEC_DEVELOPER_GUIDE.md`, `docs/STATE_EXEC_ROADMAP.md`,
   `docs/STATE_EXEC_PORTING_PLAN.md`, `docs/STATE_LIFETIME_AND_ATOMICITY.md`,
   `docs/STATE_BINARY_STREAMING.md` and `docs/roadmap/DISTRIBUTED_STATE_ROADMAP.md`
   (the `release.yml:459` comment is historical — leave it).
6. **Compatibility (recommended)**: a plain rename is a **source**-break for the
   out-of-tree consumers cvcpkg publishes (pycvc users who embed headers, apps that
   include `<cvc/core/state.h>` directly). There is **no ABI break** — symbol names
   are unchanged; `nav_abi_smoke` continues to guard header/lib drift. Recommended:
   leave thin forwarding headers in `inc/cvc/core/` for one release cycle:

   ```cpp
   // inc/cvc/core/state.h (temporary, removed next minor)
   #pragma once
   #include <cvc/state/state.h>
   ```

   (…×37 top-level + the `state_exec/` forwarding dir, ~20 more). Remove the shims in
   the follow-up PR; update `docs/STATE_API.md` to call the new path canonical.
   If the team prefers a hard cutover, drop this step and note the source break in
   the release notes — the in-tree + pycvc surface is fully covered by steps 2–4.

## Phases

| PR | content | gate |
|----|---------|------|
| 1 | `git mv` + CMake list edits + include rename + pycvc includes + docs + (optional) shims | full `ctest` (incl. the ctest-drift guard), in-tree `CVC_BUILD_CVCGL=ON CVC_BUILD_EXAMPLES=ON` build, cvcpkg recipe builds (`libcvc`, `cvcgl`, `cvcgl-examples`, `pycvc-cp313`), wasm CI job, `nav_abi_smoke` pass |
| 2 | (next minor) drop the shim headers if step 6 was taken; cvcpkg `libcvc` `cvc_revision` bump + republish | release pipeline |

## Risks / notes

- **Review noise**: a 246-file include rename is mechanical but huge in diff size —
  keep it a single dedicated PR with no logic changes so it's trivially reviewable
  (and bisectable).
- **State-exec porting**: `STATE_EXEC_PORTING_PLAN.md` tracks ongoing work *inside*
  `state_exec/`; the directory move is orthogonal (it lands under
  `state/state_exec/`, and the porting doc's file references are updated in step 5).
- **No option churn**: `CVC_STATE_EXEC` is gone (state_exec always built, see
  `docs/ARIADNE.md` L94 and the cvcgl-examples recipe comment) — there is no build
  switch to carry across the move.
- **wasm**: the browser demos statically embed the cvc core; the move changes no
  symbols, so the wasm gallery builds are unaffected (verify via CI, not by hand).
