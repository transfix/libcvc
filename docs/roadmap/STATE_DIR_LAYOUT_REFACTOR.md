# State Directory Layout Refactor — move state code to `src/cvc/state` + `inc/cvc/state`

Status: **PHASE 1 IMPLEMENTED** — the move, include rename, docs and compat shims have
landed (see [Phase 1 implementation notes](#phase-1-implementation-notes)); Phase 2 (drop
the shims) is pending. Supersedes nothing — the distributed-state *runtime* work is
tracked in `DISTRIBUTED_STATE_ROADMAP.md`; this is a pure layout refactor of where the
state code lives.

## Purpose

Before Phase 1 the state API and its runtime — the largest subsystem in libcvc — were
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
target via **explicit** file lists (no globs — see the ariadne `list(APPEND SOURCE_FILES …)`
lines in `src/cvc/CMakeLists.txt`),
and is consumed through the `cvc/<api>/…` include namespace. State should get the same
treatment: everything `state_*` lives under `{src,inc}/cvc/state/`, leaving `core/`
for the actual core.

## Layout before Phase 1 (the mess)

Kept as written when the plan was made; the paths below are the *old* ones.


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
  installed by its own `install(FILES …/config.h …)` rule — core build config, not state).
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
   `src/cvc/CMakeLists.txt` (the `INCLUDE_FILES` and `SOURCE_FILES` blocks): path
   prefixes only, same 66 entries (plus adding the missing `state_exec/generator.h`
   + `utf8.h` to `INCLUDE_FILES` — list hygiene, no build change).
2. **Rename the includes**: `<cvc/core/state…>` → `<cvc/state/state…>` and
   `<cvc/core/state_exec/X>` → `<cvc/state/state_exec/X>` in the ~246 in-tree files
   (scriptable: the include paths are unique strings; run the full test suite after).
3. **Install**: nothing to change — headers ship via the bulk
   `install(DIRECTORY inc/cvc/ …)` rule, which picks up the new layout
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
   install thin forwarding headers at `include/cvc/core/` for one release cycle.
   *(As implemented they live in `compat/inc/cvc/core/`, not `inc/cvc/core/` — see the
   implementation notes for why.)*

   ```cpp
   // compat/inc/cvc/core/state.h, installed as include/cvc/core/state.h
   // (temporary, removed next minor)
   #pragma once
   #include <cvc/state/state.h>
   ```

   (…×37 top-level + the `state_exec/` forwarding dir, ~20 more). Remove the shims in
   the follow-up PR. (`docs/STATE_API.md` already names the new path canonical as of
   Phase 1.)
   If the team prefers a hard cutover, drop this step and note the source break in
   the release notes — the in-tree + pycvc surface is fully covered by steps 2–4.

## Phases

| PR | content | gate |
|----|---------|------|
| 1 | `git mv` + CMake list edits + include rename + pycvc includes + docs + (optional) shims | full `ctest` (incl. the ctest-drift guard), in-tree `CVC_BUILD_CVCGL=ON CVC_BUILD_EXAMPLES=ON` build, cvcpkg recipe builds (`libcvc`, `cvcgl`, `cvcgl-examples`, `pycvc-cp313`), wasm CI job, `nav_abi_smoke` pass |
| 2 | (next minor) drop the shims: delete `compat/`, the `compat/inc` `install()` rule in `src/cvc/CMakeLists.txt` and `state_compat_headers_test` (+ its `src/cvc/tests/CMakeLists.txt` entries); drop the "old paths still compile" note in `docs/STATE_API.md`; cvcpkg `libcvc` `cvc_revision` bump + republish | release pipeline |

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

## Phase 1 implementation notes

What PR 1 actually did, and where it departs from the plan above:

- **Move**: 105 files, all `git mv` — 32 + 16 `.cpp` into `src/cvc/state/`
  and `src/cvc/state/state_exec/`, 37 + 20 headers into `inc/cvc/state/` and
  `inc/cvc/state/state_exec/`. 84 of them also carry the include rewrite below, so git
  sees them as renames at 94–100% similarity (pure renames for the other 21). `core/` keeps
  only non-state files: the "stays" list above, their headers, and `types.h`.
- **CMake**: besides the two file lists, three other path spellings had to move with
  them — the `list(REMOVE_ITEM SOURCE_FILES …)` lines for `state_transport_ipc.cpp`
  (`WIN32`) and `state_memory_manager.cpp` / `state_eviction_store.cpp`
  (`CVC_STATE_MEMORY_MANAGER=OFF`), and the gRPC block's `state_transport_grpc.cpp`.
  `REMOVE_ITEM` is a silent no-op on a non-matching path, so a missed one would only show
  up as a Windows link failure or a feature flag that stopped working. `generator.h` and
  `utf8.h` were added to `INCLUDE_FILES`.
- **Include rename**: 776 substitutions across 255 files — the 246 C++/SWIG consumers
  counted above, plus `src/cvc/CMakeLists.txt` and 8 docs. `.clang-format` sorts and
  regroups includes, so 55 include blocks were re-sorted to keep the format check green
  (`<cvc/state/…>` now sorts after `<cvc/gl/…>`, `<cvc/lod/…>`, etc.), and five files'
  trailing `// …` comments on include lines were realigned (the one-character-longer path
  breaks clang-format's comment alignment against unchanged neighbours). Note that CI's
  `git clang-format` diffs with `diff-index`, which does no rename detection, so the
  format check treats every moved file as new and checks it in full.
- **Shims live in `compat/inc/cvc/core/`, not `inc/cvc/core/`.** PRs here are squash-merged.
  With shims at the old source paths, the squashed commit would show `inc/cvc/core/state.h`
  as *modified* (872 lines → a 5-line forwarder) and `inc/cvc/state/state.h` as *added*. Git
  only pairs renames from deleted paths, so `git log --follow` and blame would lose the
  history of all 57 headers. Keeping the old paths deleted keeps the move a true rename. A
  separate `install(DIRECTORY compat/inc/cvc/ …)` rule ships the shims to the same installed
  paths (`include/cvc/core/state*.h`, `include/cvc/core/state_exec/`), so installed
  consumers see no difference. Two side effects: `compat/inc` is not on libcvc's include
  path, so in-tree code *cannot* regress to the old spellings (Phase 2 can't be broken by a
  stray new include). And a project that consumes libcvc via
  `add_subdirectory`/`FetchContent` rather than an install does not get the shims — those
  builds see the new paths only.
- **Shim test**: `state_compat_headers_test` puts `compat/inc` on its own include path and
  includes all 57 shims through their old spellings, so a shim pointing at a moved or
  renamed header fails CI instead of a consumer's build.
- **Docs**: also updated `USAGE.md`, `docs/ARIADNE.md` and
  `docs/roadmap/UNICODE_SUPPORT_ROADMAP.md` (path references the plan didn't list), and
  fixed pre-existing stale paths in `STATE_EXEC_ROADMAP.md` (`inc/cvc/state_list.h` and
  similar, from before `core/` existed). `docs/STATE_API.md` now names `cvc/state/` as
  canonical and points here. The same pre-`core/` stale paths were fixed in
  `STATE_EXEC_PORTING_PLAN.md` (inline header references and the §5 File Layout tree,
  now rooted at `inc/cvc/state/` and `src/cvc/state/`). `README.md`'s Project Structure
  tree gained a `state/` entry. The `release.yml` comment is left alone, as planned.
