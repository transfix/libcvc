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
read-only: vtables, typeinfo, `const` pointer tables). So are a few symbols the toolchain writes
into every TU, which no source change can remove: libstdc++'s `std::__ioinit`, coverage counters,
`DW.ref.__gxx_personality_v0`, and nvcc's fatbinary registration stubs (`TOOLCHAIN` in `scan.py`).

The scan fails on

- a symbol that is not on [allowlist.txt](allowlist.txt): new process-wide state;
- an allowlist line this build no longer has: the object moved onto the app, so delete the line.

So the allowlist only shrinks. It starts as the state §1.3's audit found, and P0a removes lines
subsystem by subsystem until only §9 item 5's process resources are left (signal dispositions,
SDL's event queue, curl / ImageMagick / Jolt / HDF5 / gRPC global initialisation, CUDA's
per-device state).

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

A new line needs a reason in review: why the object cannot live under the app.

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
targets (other targets' lines are kept) and exits 0. Without a Linux box, take the allowlist from
the CI artifact instead. Review the diff before committing it: a line that appears is new state.

Its tests use canned `nm` output, plus one end-to-end case that compiles `fixture.cpp` and scans
the object. That case needs a compiler that writes ELF, GNU `nm` and `c++filt`
(`SCAN_TEST_CXX`, `SCAN_TEST_NM`, `SCAN_TEST_CXXFILT`; on Windows, VS's
`clang++ --target=x86_64-linux-gnu` and MinGW's binutils work):

```bash
python3 tools/singleton_scan/test_scan.py
```
