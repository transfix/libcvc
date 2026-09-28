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

---

# state_exec (the Ariadne DSL) — Unicode support

> The four boundaries above are about libcvc's **host** code (paths, logging,
> Qt/pycvc edges). `state_exec` — the small Lisp-like DSL that drives Ariadne
> (`cvc::state_exec`: parser → evaluators → string/collections/math stdlib →
> the multiprocess wire codec → the cvc::state snapshot codec) — is a **separate
> surface** the section above never mentions, and it needs its own treatment.
> Everything in Ariadne that carries text (widget labels, bound state keys and
> values, `on_*` handler script bodies, `msg-send`/`msg-recv` payloads, HTTP
> request/response bodies) flows through `state_exec` strings, so "full Unicode
> support everywhere there is a string" is largely a `state_exec` question.

## What already works (byte-transparent)

`state_exec` stores every string as a raw `std::string` byte buffer
(`value_tag` holds `std::string`; state nodes store `_value`/`_name` as
`std::string`). Because UTF-8 is self-synchronizing and `std::string` is
byte-transparent, **any operation that only stores, copies, concatenates, or
substring-searches a UTF-8 string is already correct today** and needs no
change:

- **Storage & round-trip.** A `"café"` / emoji / CJK literal survives
  parse → `value_t` → `to_string` → the cvc::state snapshot codec
  (`state_value_codec.cpp`) → the multiprocess wire encoder
  (`exec_coordinator.cpp` `jkv`, emits raw UTF-8 as valid JSON) and back,
  byte-for-byte. `print` emits string bytes unchanged.
- **State keys/paths.** `state-set`/`get`, `state-data-set`/`get` round-trip
  arbitrary UTF-8 in both keys and values. Path splitting on the `.` SEPARATOR
  is byte-safe (no UTF-8 multibyte sequence contains `0x2E`); `std::map` key
  lookup is bytewise, so a UTF-8 key node is found and enumerated correctly. No
  `state_exec`-side validation rejects non-ASCII keys.
- **Byte-sequence string ops.** `str-concat`, `string.join`,
  `string.replace`, `string.contains`, `string.starts-with`/`ends-with`,
  `string.split` **with a non-empty delimiter**, and `=`/`!=` whole-string
  compare are all whole-byte-sequence operations and are UTF-8-safe on
  well-formed input.

This byte-transparent storage **already satisfies the libcvc-wide policy above**
("UTF-8 internally, convert only at edges"). The gaps are entirely in code that
ascribes *character semantics* to a byte, plus the absence of a first-class
binary type.

## The two tracks

Full Unicode support for `state_exec` splits cleanly into two independent
tracks, mirroring Python 3's `str`/`bytes` split:

- **Track A — make `string` Unicode-correct.** `string` is *text*: a sequence
  of Unicode codepoints stored as UTF-8. Every operation that counts, indexes,
  slices, folds case, or lexes must operate on codepoints, not bytes.
- **Track B — add a `bytes` type.** A `std::string` is doing double duty today:
  it is both the text type and the *de facto* binary buffer (HTTP octet-stream
  bodies, Python `bytes` marshalled in, binary URI payloads). Once `string`
  becomes codepoint-semantic, binary data must **not** ride in a `string` — text
  operations (case-fold, codepoint length, `\u` decode, NFC) are meaningless and
  corrupting on raw octets. `bytes` gives binary data a home so it stops
  masquerading as text.

### Track A — `string` Unicode correctness (gap ledger)

All line refs are current as of this writing; treat them as anchors, not
guarantees.

| # | Severity | Gap | Where |
|---|----------|-----|-------|
| A1 | breaks-utf8 | `string.char-at` byte-indexes and returns a single **byte** — a multibyte codepoint yields an invalid partial byte; bounds-checks against byte length | `stdlib.cpp:179-186` |
| A2 | breaks-utf8 | `string.substring` uses **byte** pos/len — a boundary inside a multibyte sequence emits invalid UTF-8 | `stdlib.cpp:162-177` |
| A3 | breaks-utf8 | `string.split ""` (empty delim) splits into single **bytes**, shredding every multibyte codepoint | `stdlib.cpp:70-72` |
| A4 | breaks-utf8 | `string.upper`/`lower` call `::toupper`/`::tolower` per **byte** — ASCII-only folding, and passing a high (signed) byte to `<cctype>` is UB | `stdlib.cpp:126-138` |
| A5 | wrong-result | `string.length` returns **byte** count, not codepoint count (`"café"` → 5) | `stdlib.cpp:188-192` |
| A6 | wrong-result | `builtin_length` returns **byte** size for string args (same off-by) | `builtins.cpp` (length) |
| A7 | rejects-valid | Lexer identifier class (`is_symbol_char`) is ASCII-alnum only — a symbol/var/fn name with a byte ≥0x80 is silently truncated at the first high byte | `parser.cpp:169-196` |
| A8 | rejects-valid | String literals support only `\n \t \r \\ \"` — no `\uXXXX`/`\u{…}`/`\xNN`; a codepoint can only be entered as raw UTF-8 bytes in source | `parser.cpp:135-155` |
| A9 | wrong-result | Wire JSON **decoder** (`xstr`) drops `\uXXXX` escapes — corrupts data from a standard JSON producer; encoder `jkv` also under-escapes control bytes <0x20 | `exec_coordinator.cpp:96-134` |
| A10 | breaks-utf8 | `to_string` 64 KiB truncation cuts on a byte boundary, can leave an invalid trailing byte before the U+2026 ellipsis | `types.cpp:116-122,152-155,170-172` |
| A11 | wrong-result | Token classification uses locale-dependent `std::isalnum`/`isspace` on raw bytes | `parser.cpp` (lexer) |
| A12 | latent | `cvc::state::isValidStateName`/`sanitizeStateName` are ASCII-only + signed-char UB; **not** on the state_exec key path today, but any per-field key validation wired to them would reject valid UTF-8 | `state.cpp:1485-1547` |
| A13 | cosmetic | `string.trim` (and lexer whitespace) recognize only ASCII ` \t\n\r`, not Unicode whitespace (NBSP U+00A0, U+2028, U+3000…) | `stdlib.cpp:116-124` |
| A14 | cosmetic/policy | No NFC/NFD normalization anywhere — canonically-equivalent composed vs decomposed encodings are distinct keys/unequal strings. Recommended policy: **leave keys as raw bytes and document it**; normalize to NFC only at the DSL boundary if canonical-equivalence matching is ever required | `state.cpp`, byte-wise map keys |

Note that the display-oriented builtins advertised in `intrinsics.h`
(`println`/`format`/`error`/`to-string`/`gensym`) are **declared but not yet
implemented** — only `print` is registered. When `format` lands, its
field-width/alignment must be built on **codepoint (ideally East-Asian display)
width from the start**, never on `std::string::size()`, or every row with
multibyte text misaligns.

The whole track rests on **one shared UTF-8 codepoint helper**
(decode/iterate/length/index→byte-offset). Add it once (a small hand-rolled
decoder, or vendor a header-only `utfcpp`; **no ICU dependency needed for A1–A11
and A13** — ICU/a case table is only required for real (non-ASCII) case folding
in A4 and for NFC in A14). Everything else is byte-arithmetic replaced by
codepoint-arithmetic against that one helper.

### Track B — add a `bytes` type (Python-3-style)

Today `value_t` is a 12-alternative variant
(`monostate,bool,int64,double,string,symbol,list,closure,dict,native_fn,data_object,generator`)
with **no binary type**. `data_object` is a *typed handle into `cvc::state`*,
not a raw octet buffer, so it does not fill this role. The drivers for a real
`bytes` primitive:

- **HTTP octet-stream bodies** (roadmap §13.10 handler surface): a binary
  request/response body (PNG, protobuf, gzip) must not be forced through a
  text `string` that later gets codepoint-length'd, case-folded, `\u`-decoded,
  or NFC-normalized.
- **`py_to_value` (P3b Python bridge):** Python `bytes` has no faithful landing
  type today; it coerces to `string`, which then invites text operations on
  binary.
- **Binary URI handlers, state-data payloads, `msg-send`/`msg-recv` payloads:**
  same — anywhere raw octets travel.
- **Codec round-trip:** `string.encode`/`bytes.decode` need somewhere for the
  intermediate octet buffer to land.

Implementation sketch:

- **New variant alternative.** A distinct wrapper type — you **cannot** add a
  second bare `std::string` alternative (the variant would be ambiguous), so
  introduce e.g. `struct bytes_value { std::string data; };` (or
  `std::vector<std::byte>`) and add it to `value_tag::variant_type`. Update
  `type_name()`, `to_string` (print as `b"…"` with non-printable octets hex-
  escaped, **never** as raw text), `values_equal` (byte compare), and truthiness.
- **Literal syntax.** `b"…"` string-like literals with `\xNN` (and raw bytes),
  and/or a `(bytes …)`/hex constructor. Parser addition parallel to A8's escape
  work but byte-valued.
- **`bytes` builtins, byte-semantic by definition.** `bytes.length` (byte
  count — correct, unlike `string.length`), `bytes.slice`/`bytes-at` (byte
  offsets), `bytes.concat`, `bytes.=`. These are the operations `string`
  *should not* have.
- **Codecs (the bridge between the tracks).** `string.encode s "utf-8"` →
  `bytes`; `bytes.decode b "utf-8"` → `string` (with a well-defined error or
  replacement policy on invalid input). This is the *only* sanctioned crossing
  between text and binary.
- **Type introspection.** `type-of` / `intrinsics` report `"bytes"`; ensure the
  wire codec and state-data codec marshal `bytes` distinctly (base64 or a length-
  prefixed binary field) rather than as a JSON string, so a round-trip preserves
  the type, not just the bytes.

Track B is **additive** — it introduces a new type and new builtins without
changing existing `string` behavior, so it can land independently of Track A.
But the two are complementary: Track A makes `string` safe to treat as text;
Track B ensures binary data is never *forced* to be text in the first place.

## Phased plan (state_exec)

Namespaced `SE-*` to distinguish from the libcvc-wide `P0–P3` above.

- **SE-0 — Codepoint helper + test corpus + policy note. ✅ LANDED.** The shared
  UTF-8 helper is `inc/cvc/core/state_exec/utf8.h` (`decode`/`count`/`byte_offset`/
  `encode`/`is_valid`, header-only, no ICU, lenient decode that always progresses on
  malformed input). `state_exec_unicode_test.cpp` unit-tests it (every byte-length
  class + malformed/overlong/surrogate/truncated cases) and pins the transparent
  paths with a `"café"` / 4-byte-emoji / CJK corpus: parse→value→to_string and the
  cvc::state snapshot codec round-trip **pass today** (wire `jkv`/`xstr` are covered
  by the existing codec tests). The "text vs bytes" policy is this section, restated
  in the helper's header. **No behavior change** — the helper is added and tested in
  isolation, unwired, so SE-1 is a pure byte→codepoint swap in the string builtins.
- **SE-1 — Track A string builtins.** Fix A1–A6 (char-at, substring, split-"",
  upper/lower, length ×2) against the helper. Add failing-then-passing tests
  that pin codepoint semantics. Highest user-visible impact.
- **SE-2 — Track A lexer + escapes + wire codec.** A7 (UTF-8 identifiers, or an
  explicit documented ASCII-only decision), A8 (`\u`/`\x` escapes), A9 (wire
  `\u` decode + control-byte escaping), A11 (unsigned-char/locale-free
  classification), A10 (truncate on codepoint boundary). A13 optional.
- **SE-3 — Track B `bytes` type.** New variant alternative, literal syntax,
  `bytes` builtins, `encode`/`decode` codecs, `type-of`, wire/state-data
  marshalling. Then route the binary drivers (HTTP octet-stream, `py_to_value`
  bytes branch, binary URI handlers) to `bytes` instead of `string`.
- **SE-4 — Normalization policy + latent validators.** Decide and document A14
  (recommended: raw-bytes keys, NFC only at the boundary if needed). Fix the
  A12 signed-char UB regardless, and gate on UTF-8-aware rules if those
  validators are ever wired to DSL keys.

## Status (state_exec)

**DEFERRED — documented, not scheduled**, consistent with the libcvc-wide work
above. The transparent storage/copy/concat/search paths and the wire/state
codecs are **already Unicode-clean**, so nothing here blocks Ariadne today; the
gap is purely in character-*semantic* operations and the missing binary type.
SE-0 is a safe, no-behavior-change first step (helper + tests + policy) that can
land whenever this becomes a priority. **Zero Unicode test coverage exists
today** — no `state_exec` test exercises a multibyte value — so SE-0's corpus is
the prerequisite for trusting any later change.

## Related

- `CVC/log4cplus_compat.h` (VolumeRover) — the char-mode workaround to retire.
- cy-pca/cvcpkg#82 — dual (narrow + Unicode) log4cplus bundle.
- `docs/WORLD_UNITS_API.md`, `docs/STATE_API.md` — precedent for a documented,
  fail-loud, cross-cutting invariant.
- `src/cvc/core/state_exec/{parser,stdlib,builtins,types,exec_coordinator}.cpp`
  — the state_exec surfaces enumerated in the gap ledger above.
- `docs/roadmap/CVCGL-UI-DSL-ROADMAP.md` — the Ariadne DSL roadmap this
  state_exec section backs (all Ariadne text flows through state_exec strings).
