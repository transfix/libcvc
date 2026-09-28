# libcvc Unicode Support Roadmap — a Unicode-clean core, end to end

> **Goal:** make `libcvc` (and the apps built on it — VolumeRover, pycvc,
> the cvcGL demos) **fully Unicode-capable**: non-ASCII file paths, dataset
> names, log text and UI strings must round-trip correctly on every platform,
> without the char-mode workarounds the tree currently relies on. This is
> **essential** — the tools are used internationally and on data whose paths and
> names are routinely non-ASCII — but it is **future work**, tracked here, not a
> commitment for the current cycle.

## Why this is needed

libcvc and its consumers are **char-mode throughout**. On POSIX that is usually
fine (paths are UTF-8 byte strings). On **Windows it silently loses data**: the
narrow (`char`) Win32 API interprets `std::string` paths in the process ANSI
code page, so a dataset under a path with any non-ASCII character (an accented
name, CJK, Cyrillic, …) fails to open or is mangled — even though the file
exists and the wide (`wchar_t`) API would open it. For a scientific/medical
visualization tool this is a correctness bug, not a nicety.

The char-mode assumption is baked in as a **deliberate workaround**, most
visibly in logging:

- **`CVC/log4cplus_compat.h`** (VolumeRover) `#undef`s `UNICODE`/`_UNICODE`
  around the log4cplus includes, *specifically* to force the char API and match
  a narrow log4cplus build. Qt already defines `UNICODE` on Windows, so this is
  fighting the platform.
- libcvc's own log4cplus use (`cvc::app`, under `USING_LOG4CPLUS_DEFAULT`) and
  all consumers pass **`char` string literals** to `Logger::getInstance(...)`
  and stream **`std::string`** messages — none of it is wrapped in
  `LOG4CPLUS_TEXT`, so it only compiles against a narrow log4cplus.

As of **cy-pca/cvcpkg#82** the cvcpkg `log4cplus` bundle ships **both** the
narrow (`log4cplus.lib`) and Unicode (`log4cplusU.lib`) variants, so the
*dependency* no longer forces the choice — the remaining work is in **our**
code.

## What "complete Unicode support" means here

Four boundaries, in rough priority order:

1. **Filesystem paths (highest impact).** Stop passing `std::string` paths to
   the OS on Windows. Route every path through `std::filesystem::path`
   (constructed from UTF-8, opened via the wide API the STL uses under the hood)
   or an explicit UTF-8↔UTF-16 shim at the OS boundary. Audit: `fopen`/
   `std::ifstream(std::string)`/`CreateFile`/ImageMagick/HDF5/assimp/VTK file
   entry points — anything that takes a path.
2. **Logging.** Use the Unicode log4cplus variant (now available): remove the
   `log4cplus_compat.h` `UNICODE` undef, wrap logger names and literal messages
   in `LOG4CPLUS_TEXT(...)`, and provide a portable `char*`→`tstring` helper for
   `FUNCTION_LOGGER` (`BOOST_CURRENT_FUNCTION`). Or introduce a thin
   `cvc::log` façade that is char-based in the API and converts once at the
   log4cplus boundary, so consumers never see `tstring`. **The façade is
   probably the cheaper, lower-churn path** — it localizes the wide-char contact
   to one file instead of every `LOG4CPLUS_*` call site.
3. **String storage & conversion policy.** Pick one internal representation —
   **UTF-8 `std::string` everywhere, convert only at OS/UI edges** is the
   recommended policy (matches POSIX, matches Qt's `QString::fromUtf8`, avoids a
   `wchar_t` sprawl) — and document it so new code follows it.
4. **UI / bindings.** Qt is already Unicode; the boundary is libcvc's `char`
   APIs handed to/from Qt (`QString::fromUtf8`/`toUtf8`) and the pycvc `str`
   surface (Python 3 `str` is Unicode — ensure the SWIG typemaps carry UTF-8,
   not the ANSI code page).

## Phased plan (future)

- **P0 — Policy + audit.** Write down the "UTF-8 internally, convert at the
  edge" rule (a short `docs/UNICODE.md`). Grep the tree for the path and logging
  entry points above; enumerate the offenders. No behavior change.
- **P1 — Paths.** Fix the file-open boundary so non-ASCII paths work on Windows.
  Add a regression test that opens a dataset under a non-ASCII path on all
  platforms (skips only if the FS can't represent it). This is the change that
  actually fixes user-visible breakage.
- **P2 — Logging.** Either the `cvc::log` façade (preferred) or the
  `LOG4CPLUS_TEXT` sweep + drop the `log4cplus_compat.h` undef, linking the
  Unicode log4cplus. Keep the narrow bundle available for anything not yet
  converted.
- **P3 — Bindings/UI edges + docs.** Confirm pycvc/SWIG carries UTF-8; document
  the Qt boundary conventions; remove the now-dead char-mode workarounds.

## Status

**DEFERRED — documented, not scheduled.** Nothing here blocks current work; the
cvcpkg dual-variant log4cplus (cy-pca/cvcpkg#82) removes the only *external*
obstacle, so P2 can start whenever P0/P1 land. Owner-gated on when it becomes a
cycle priority.

## Related

- `CVC/log4cplus_compat.h` (VolumeRover) — the char-mode workaround to retire.
- cy-pca/cvcpkg#82 — dual (narrow + Unicode) log4cplus bundle.
- `docs/WORLD_UNITS_API.md`, `docs/STATE_API.md` — precedent for a documented,
  fail-loud, cross-cutting invariant.
