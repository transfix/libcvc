# The writable-data scan

libcvc keeps no process-wide state: everything lives under a `cvc::app` or in an object the
caller owns ([VOLROVER-LIBRARY-PORTS-ROADMAP.md](../../docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md)
§1.3). This scan checks that from the compiled objects, so it catches what a grep misses: a static
inside a macro, a vendored template, a function-local cache.

For every object file of the scanned CMake targets, `scan.py` lists the writable data symbols:

| kind | what it is |
|---|---|
| `data` | an initialised mutable object (`.data`, `.data.rel`, `.data.rel.local`) |
| `bss` | a zero-initialised or dynamically initialised one (`.bss`), function-local statics included |
| `tls` | a `thread_local` (`.tdata`, `.tbss`) |
| `guard` | a static's guard variable (`_ZGV…`), or a TU's `__tls_guard` |
| `common` | a C tentative definition (`*COM*`) |

Read-only data is left out: `.rodata`, and `.data.rel.ro` (relocated by the loader, then
read-only: vtables, typeinfo, `const` pointer tables). So is what no libcvc source defines, and so
no source change can remove; the summary counts each kind:

- what the toolchain writes into every TU: libstdc++'s `std::__ioinit`, coverage counters,
  `DW.ref.__gxx_personality_v0`, and nvcc's fatbinary registration stubs (`TOOLCHAIN` in
  `scan.py`);
- the tables a code generator writes into its own output (`GENERATED`): SWIG's module runtime in
  the `*PYTHON_wrap.cxx` wrappers (its `_swigt__` / `_swigc__` type and cast tables, `swig::`
  templates, NumPy's `PyArray_API`), and protoc's message tables in `*.pb.cc`. pycvc's own code
  that SWIG copies into a wrapper is still scanned;
- third-party libraries' own statics, which their headers and templates put into our objects
  (`THIRD_PARTY`): Boost's, VTK's per-file Schwarz counters (`vtkDebugLeaksManagerInstance`,
  `vtksys::SystemToolsManagerInstance` …), libigl's, Eigen's, pocketfft's and the standard
  library's. Vendored code built into libcvc (SDF, cvc-mesher …) is libcvc's and is scanned.

The scan fails on

- a symbol that is not on [allowlist.txt](allowlist.txt): new process-wide state;
- an allowlist line this build no longer has: the object moved onto the app, so delete the line.

The allowlist is libcvc's own objects still to move, and it only shrinks, to empty. **Never add a
line**, not even one a CI artifact suggests: move the object onto the app, or into an object the
caller owns. CI's `clang-format (changed lines)` job runs [ratchet.py](ratchet.py), which fails a
change whose allowlist has a line its base's does not (a moved or renamed object counts as new).
Each move comes with tests that make several `cvc::app`s, in both orders and on threads, and show
that nothing one does reaches another.

## The allowlist

One object per line:

```
<target> <source> <kind> <symbol>
cvc src/cvc/core/app.cpp bss cvc::app::some_registry
cvc src/cvc/core/app.cpp guard guard variable for cvc::detail::table()::t
cvc <header> bss boost::asio::detail::execution_context_service_base<...>::id
```

- `<target>` is the CMake target whose object holds it.
- `<source>` is the file it is defined in, from the object's path in the build tree. An inline
  function's static, a template's static member and their guards have vague linkage: any object
  that includes the header may carry them, so their source is `<header>` and they are matched
  anywhere in the target.
- `<symbol>` is the demangled name, to the end of the line.
- Lines starting with `#` are comments. The file is generated, sorted by target and source.

## Where it runs

On the Linux CI lanes, where the objects are ELF and GNU `nm` and `c++filt` read them:

| Lane | Targets |
|---|---|
| `package-linux / libcvc-release-grpc` (the shipped configuration: gRPC, WebSocket and CUDA on) | `cvc` (with the vendored code built into it: cvc-mesher, SDF, LBIE …), `cvc_igl`, `cvc_physics_jolt`, `cvcGL`, `cvcMedia`, `cvc-cli` |
| `pycvc-linux / cp312` | `pycvc`, `pycvc_gl` |

Each lane checks only its own targets' lines, so each line is exact for one configuration. Each
uploads the allowlist its build needs as an artifact (`singleton-scan-libcvc`,
`singleton-scan-pycvc`) and writes the per-subsystem table to the job summary.

## Running it

After a Linux build:

```bash
python3 tools/singleton_scan/scan.py --build-dir build \
  --targets cvc,cvc_igl,cvc_physics_jolt,cvcGL,cvcMedia,cvc-cli
```

`--write-allowlist tools/singleton_scan/allowlist.txt` rewrites the allowlist for the scanned
targets (other targets' lines are kept) and exits 0. Without a Linux box, the CI artifact holds
the same list. Use either only to delete the lines of objects you moved: a line that appears is
new state, and `ratchet.py` fails the change.

`python3 tools/singleton_scan/ratchet.py <base allowlist> tools/singleton_scan/allowlist.txt`
runs the ratchet locally (`git show origin/master:tools/singleton_scan/allowlist.txt` gives the
base).

Its tests use canned `nm` output, plus one end-to-end case that compiles `fixture.cpp` and scans
the object. That case needs a compiler that writes ELF, GNU `nm` and `c++filt`
(`SCAN_TEST_CXX`, `SCAN_TEST_NM`, `SCAN_TEST_CXXFILT`; on Windows, VS's
`clang++ --target=x86_64-linux-gnu` and MinGW's binutils work):

```bash
python3 tools/singleton_scan/test_scan.py
```
