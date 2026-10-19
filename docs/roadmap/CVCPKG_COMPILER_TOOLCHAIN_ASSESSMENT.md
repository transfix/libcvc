# Assessment: compilers from cvcpkg for libcvc builds

**Status:** assessment only; no workflow changes. 2026-10-12.
**Base:** origin/master `790760f6`; cvcpkg CLI 2.3.2; cvcpkg recipes at `d0662c09`.

Question: should every libcvc build switch from the runner-image compilers
(GCC on ubuntu, Xcode clang on macos, MSVC on windows-2022) to the LLVM/clang
toolchain cvcpkg publishes? Follow-ups: can cvcpkg provide the Microsoft Build
Tools, should it ship GCC for Linux and the BSDs, and can cvcpkg provision a
compiler on demand on Windows?

## Recommendation

No, not all at once. Go platform by platform, and remember that "the compiler"
has two halves:

1. **The compiler binary.** cvcpkg's `llvm`/`llvm18` bundles provide this.
2. **The runtime/sysroot.** This is the C++ standard library, the CRT and libc
   headers/startfiles, and the platform SDK. None of the cvcpkg LLVM bundles
   build any of it: no libc++, no compiler-rt, no libomp.

Moving to cvcpkg clang therefore changes where `clang` comes from. libstdc++
(Linux), the macOS SDK, and MSVC's STL/CRT plus the Windows SDK still come from
the runner image, and those are the parts that set the ABI.

| Platform | Recommendation | Why |
|---|---|---|
| **macOS arm64** | **Yes, first.** Pilot in ci.yml once two cvcpkg packaging bugs are fixed. | It is the only clean swap: there is no CUDA, and libc++ comes from the SDK either way. It also pins the compiler, which today follows Xcode (AppleClang 21 on macos-latest). Blocked by the bundle's undeclared `libzstd`. Pair it with cvcpkg `openmp`, because **macOS libcvc builds without OpenMP today**. |
| **Linux x86_64** | **Not clang.** Adopt cvcpkg's planned `gcc-toolchain` (GCC 14 + glibc 2.28 sysroot) when it lands, together with CUDA 12.8 or 12.9. | cvcpkg clang still compiles against the runner's GCC install (libstdc++ headers, crtbegin, libgcc), so the ABI-defining parts stay outside cvcpkg. cvcpkg's own roadmap rejects clang for exactly this reason (`docs/roadmap/native-toolchain-spec.md` §1). Clang 18 would also need CUDA ≥ 12.6, and clang 22 needs CUDA 13.4. |
| **Windows x86_64** | **Not now.** Pin the MSVC toolset, and build cvcpkg's Build Tools provisioning first (see the follow-up section). | clang-cl still needs MSVC's STL/CRT and the Windows SDK, and NVIDIA lists only MSVC as nvcc's Windows host compiler, so MSVC stays mandatory. cvcpkg's only Windows clang (llvm18) is rejected by the runner's STL: VS 17.14 requires Clang ≥ 19. |

No cvcpkg dependency needs rebuilding for clang on Linux (verified below) or
macOS (same libc++). The one runtime gap is OpenMP. Clang needs LLVM's libomp,
which cvcpkg has (`openmp` 21.1.8) for Linux and macOS but has not published for
Windows.

### Baseline today

From ci.yml run 38171168600 on master `c8f64ad9`; configure logs via `gh run view --log`:

| Lane | Compiler | CUDA | OpenMP |
|---|---|---|---|
| package-linux (ubuntu-latest) | GNU 13.3.0 | 12.5.82, host GNU 13.3.0 | libgomp, OpenMP 4.5 |
| package-macos (macos-latest) | AppleClang 21.0.0 | off | **"OpenMP not found - disabling OpenMP support"** |
| package-windows (windows-2022) | MSVC 19.44.35229 (VS 17.14), SDK 10.0.26100 | 12.5.82, host MSVC 19.44 | `-openmp:llvm` → libomp140 (not redistributable) |

The fleet builds the cvcpkg deps for Linux with Ubuntu 22.04's **GCC 11.4.0**,
per `.comment` in libboost_filesystem, libhdf5 and libcvc 3.5.0. The Linux lane
therefore already mixes GCC 11 deps with GCC 13 libcvc. It works because newer
libstdc++ is backward compatible.

## 1. Published clang, relocatability, sysroot

| | linux x86_64 | macos arm64 | windows x86_64 |
|---|---|---|---|
| Published with clang | `llvm` 22.1.8+cvc.3, `llvm18` 18.1.8+cvc.3 (`llvm20` has no clang) | `llvm` 22.1.8+cvc.3 only | `llvm18` 18.1.8+cvc.3 only (`llvm`/`llvm20` list windows but have no build) |
| Relocatable | Yes. RUNPATH `$ORIGIN:$ORIGIN/../lib` on clang, libLLVM and libclang-cpp; no builder paths | Yes. `@rpath/…` install names, LC_RPATH `@loader_path`, `@loader_path/../lib`; minos 13.0, sdk 26.5 | Yes. Static `clang-cl.exe`; `zlib1.dll` comes from the cvcpkg zlib bundle |
| Undeclared host deps | System `libzstd.so.1` (and `libtinfo.so.6` for 18); GLIBC_2.34 / GLIBCXX_3.4.30 floor. Present on ubuntu runners | **`@rpath/libzstd.1.5.7.dylib`: not shipped and not declared**, so it very likely fails to load from a clean prefix | VC++ runtime (`MSVCP140`, `VCRUNTIME140`, `_1`) |
| Resource headers (`lib/clang/N/include`) | Yes, both | Yes | Yes |
| Sysroot / C++ stdlib | None. Uses the system GCC install (`Selected GCC installation: /usr/lib/gcc/x86_64-linux-gnu/11`) and glibc | None. Needs the Xcode/CLT SDK (`/usr/lib/libc++.1.dylib`) | None. Needs MSVC STL/CRT and the Windows SDK (a VS install) |
| compiler-rt (builtins, sanitizers, profile) | No | No | No |
| Default linker | bundled `ld.lld` (`CLANG_DEFAULT_LINKER=lld`) | bundled `ld64.lld` | `link.exe` unless `-fuse-ld=lld` |

The earlier cmake problem, where bundles carried builder-absolute paths, does
**not** affect the LLVM bundles. Their problems are undeclared dependencies.

## 2. ABI: linking the existing cvcpkg deps

- **Linux: compatible, verified.**
  - clang 18 and clang 22 compiled a probe that:
    - returns a `std::string` from GCC-built libcvc 3.5.0;
    - builds `boost::filesystem::path` objects;
    - catches a `boost::program_options` exception thrown by GCC-compiled code;
    - calls HDF5 1.14.4.
  - It ran correctly. Both compilers use the same Itanium ABI and libstdc++.
  - **Nothing needs rebuilding.**
- **macOS: compatible.** Apple clang and upstream clang both link the SDK's
  libc++ (`clang-22` itself loads `/usr/lib/libc++.1.dylib`). Nothing needs
  rebuilding. Not run on a Mac.
- **Windows: compatible in principle, blocked in practice.**
  - clang-cl targets the MSVC ABI, so the MSVC-built deps link. mingw-w64 is a
    different ABI (catalog key `windows-gnu`) and is not an option for libcvc.
  - Blockers:
    - **llvm18 is too old for the runner's STL.** VS 17.14's `yvals_core.h`
      contains `#if __clang_major__ < 19 … STL1000 "expected Clang 19.0.0 or
      newer"`, and MSVC 14.50 has the same check.
    - **There is no Windows `openmp` build** to replace libomp140.
    - **There are no compiler-rt builtins.**
    - **lld-link can't consume `/GL` objects.**
- **OpenMP is the only gap, on every platform.**
  - Clang's codegen targets libomp.
  - Without `omp.h`/libomp, `find_package(OpenMP)` fails and libcvc's
    CMakeLists quietly sets `CVC_ENABLE_OPENMP OFF`.
  - `-fopenmp=libgomp` makes clang drop `-fopenmp` from cc1, so the pragmas are
    ignored.
  - With cvcpkg `openmp` it works (4 threads, correct reduction), and the binary
    NEEDs `libomp.so` instead of `libgomp.so.1`.

## 3. CUDA host compiler

Supported host compilers on Linux, from NVIDIA's installation guides; Windows
lists MSVC only:

| CUDA | GCC | Clang | Notes |
|---|---|---|---|
| 12.5 (current) | ≤ 13.2 | ≤ 17 | No cvcpkg clang qualifies (there is no `llvm17`) |
| 12.6 | ≤ 13.2 | ≤ 18 | Minimum for cvcpkg `llvm18` |
| 12.8 / 12.9 | ≤ 14 | ≤ 19 | Minimum for cvcpkg's planned GCC 14. Keeps sm_50/60/70 (deprecated warnings on 12.9). libcvc's CMakeLists adds sm_100/120 (Blackwell) automatically at ≥ 12.8 |
| 13.0 | ≤ 15 | ≤ 20 | **Drops sm_50–72.** libcvc ships SASS only for `50 60 70 75 80 90`, so this removes Maxwell, Pascal and Volta support and needs a CMakeLists change |
| 13.4 | ≤ 16 | ≤ 22 | Minimum for cvcpkg `llvm` (22). VS 2026 (MSVC 195x) is supported from 13.2 |

nvcc enforces these limits with `#error` checks in `crt/host_config.h`. The flag
`-allow-unsupported-compiler` bypasses them, but it is unsupported and should
not be used for releases.

**Any compiler change on Linux implies a CUDA bump.** 12.8 or 12.9 is the sweet
spot:
- It accepts GCC 14 and clang 19.
- It keeps every architecture libcvc ships today.
- It adds Blackwell.

On Windows, nvcc's host compiler stays `cl.exe` even if C++ moves to clang-cl.

Sources: the [12.5](https://docs.nvidia.com/cuda/archive/12.5.1/cuda-installation-guide-linux/index.html),
[12.6](https://docs.nvidia.com/cuda/archive/12.6.3/cuda-installation-guide-linux/index.html),
[12.8](https://docs.nvidia.com/cuda/archive/12.8.1/cuda-installation-guide-linux/index.html),
[13.0](https://docs.nvidia.com/cuda/archive/13.0.0/cuda-installation-guide-linux/index.html) and
[current](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) guides, and the
[13.0 release notes](https://docs.nvidia.com/cuda/archive/13.0.0/cuda-toolkit-release-notes/index.html).
I did not inspect the actual `host_config.h` thresholds; no CUDA toolkit was
available locally.

## 4. Shipped artifacts and consumers

Consumers are **not** required to use the same compiler. On each platform the
shipped libraries keep the platform's standard C++ ABI and standard library.
The changes would be:

- **OpenMP runtime (Linux).**
  - A clang-built `libcvc.so` NEEDs cvcpkg's `libomp.so`, which the SDK would
    bundle, instead of the system `libgomp.so.1`. That drops libgomp from
    release.yml's list of host requirements.
  - However, the exported `cvcConfig.cmake` calls `find_dependency(OpenMP)` and
    links `OpenMP::OpenMP_CXX` (from `cvcTargets.cmake`). The consumer's own
    compiler resolves that. A GCC consumer gets libgomp, so the process loads
    two OpenMP runtimes, with possible oversubscription.
- **Exported flags.**
  - Today's GCC-built SDK exports `INTERFACE_COMPILE_OPTIONS "-frounding-math"`.
    It comes from a GCC-only branch in `CMake/SetupCGAL.cmake`.
  - A clang build would not export it, so consumers would see that change.
- **libstdc++ floor (Linux, clang).** The floor is set by whichever GCC install
  clang picks, which is the newest one found. Pin it with `--gcc-install-dir`,
  or the floor follows the runner image.
- **Windows static flavor.**
  - clang-cl objects can reference compiler-rt builtins, such as 128-bit
    division helpers, which a consumer's `link.exe` can't resolve. cvcpkg ships
    no compiler-rt.
  - The same applies on every platform if LTO is ever turned on: bitcode objects
    in shipped `.a`/`.lib` files tie consumers to the same LLVM.
- **macOS:** no visible change. The deployment target still comes from
  `MACOSX_DEPLOYMENT_TARGET`, and libc++ comes from the OS.

## 5. Effort, risk, path

Effort figures are rough.

| Option | Effort | Risk | Gain |
|---|---|---|---|
| macOS → cvcpkg `llvm` 22 | cvcpkg: declare zstd (or `LLVM_ENABLE_ZSTD=OFF`), republish. libcvc: about 1–2 days (CC/CXX, `-isysroot $(xcrun --show-sdk-path)`, decide ld64.lld vs `-fuse-ld=ld`, add `openmp`) | Low to medium: ld64.lld maturity; no compiler-rt (`@available` needs `__isPlatformVersionAtLeast`) | Compiler pinned, independent of Xcode drift; OpenMP finally on |
| Linux → cvcpkg clang | About 2–3 days: compiler, `CMAKE_CUDA_HOST_COMPILER`, `--gcc-install-dir`, openmp dep and SDK bundling, closure checks, CUDA ≥ 12.6 | Medium | **Low.** libstdc++, crt and libgcc still come from the runner |
| Linux → cvcpkg `gcc-toolchain` | cvcpkg: weeks (recipe, env wiring, catalog rebuild; spec exists). libcvc: about 1–2 days plus the CUDA 12.8/12.9 bump | Medium: the libstdc++ transition window, shipping libstdc++ in the SDK | **High.** Compiler, libstdc++ and glibc floor all come from cvcpkg |
| Windows → clang-cl | 1–2 weeks across both repos. cvcpkg: llvm ≥ 19 for Windows (a 5h build), openmp for Windows, compiler-rt. libcvc: VS generator → Ninja + vcvars env, nvcc host stays cl | High | **None for provenance.** MSVC is still required |

**Recommended sequence:**

1. **libcvc, independent of compiler choice:**
   - Add `openmp` (macos) to `cvcpkg/recipes/libcvc` and bundle `libomp.dylib`
     in the macOS SDK, so macOS stops building serially.
   - Plan the CUDA 12.5 → 12.8 or 12.9 bump.
2. **cvcpkg fixes** (listed below), especially the llvm zstd dependency and
   publishing Windows `openmp`.
3. **macOS:** a non-blocking ci.yml lane on cvcpkg `llvm` 22. Switch release.yml
   once it is green.
4. **Linux:** switch when cvcpkg's `gcc-toolchain` lands, with CUDA ≥ 12.8.
5. **Windows:** pin the toolset (component `VC.14.44.17.14.x86.x64` /
   `-vcvars_ver=14.44`). Do the Build Tools provisioning below. Revisit
   clang-cl only if there is a reason other than provenance.

## Follow-up: cvcpkg as the compiler provider

### A Microsoft Build Tools recipe

**Repackaging is out.**
- The VS 2022 Build Tools license forbids sharing or publishing the software.
  The VS 2026 terms forbid distributing "the software or any of its code".
- The Windows SDK forbids publishing it "for others to copy".
- Only the listed REDIST runtime files may be shipped.
- conda-forge states it is "not allowed to redistribute the MSVC compiler".

So cl.exe, the STL and the SDK cannot go in a cvcpkg archive. Two shapes work,
and both download from Microsoft on the user's machine after the user accepts
the license:

1. **`msvc-build-tools` provisioning recipe.** This is the supported route and
   the one I recommend first.
   - It fetches the official `vs_BuildTools.exe` and verifies its Authenticode
     signature. A sha256 won't do, because the bootstrapper is updated in place.
   - It runs `--quiet --wait --norestart --nocache --add
     Microsoft.VisualStudio.Workload.VCTools --add
     Microsoft.VisualStudio.Component.VC.14.44.17.14.x86.x64 --add
     Microsoft.VisualStudio.Component.Windows11SDK.26100`.
   - It handles exit codes 3010 (reboot), 740 (elevation required) and 5007
     (blocked).
   - **Admin is required**, and the install is system-wide and registered with
     vswhere. That means cvcpkg's existing `Import-CvcMsvcEnv`
     (`recipes/_common/env-windows.ps1:57-108`) finds it with no changes.
   - The cvcpkg prefix keeps only a marker recording the install path and
     toolset.
   - Precedents: Chocolatey `visualstudio2022buildtools`, winget
     `Microsoft.VisualStudio.2022.BuildTools`, and rustup-init offering a VS
     install.
2. **`msvc-portable` (opt-in).** This is the approach of portable-msvc.py and
   xwin.
   - It reads Microsoft's VS channel manifest and downloads the pinned MSVC and
     SDK packages straight into a cvcpkg prefix.
   - It needs no admin, is relocatable, and pins the toolset exactly.
   - Microsoft doesn't support it, it doesn't register with vswhere, and it has
     **no MSBuild**. That means Ninja only, while libcvc's Windows lane uses the
     VS generator today.
   - nvcc needs the vcvars environment set up for it.
   - The same mechanism in "CRT + SDK only" mode is what clang-cl would need.

**Licence caveats** (have counsel confirm; I'm not giving legal advice):
- Use of Build Tools requires a Visual Studio license, except under the "Open
  Source Dependencies" right. That right covers compiling third-party
  OSI-licensed code, which is what cvcpkg recipe builds are.
- cvcpkg must show the license and require an explicit yes from the user, or an
  explicit flag in CI. It must never accept on the user's behalf.

**What cvcpkg needs first:**
- Install today only downloads and extracts (`installer.py:370-396`). It needs
  an install-time provisioning action that can request elevation.
- A license gate. `license.eula`, `redistributable: false` and BYO are still
  roadmap items (`docs/roadmap/config-management.md`).
- Toolset identity recorded in bundle manifests. No compiler ID is stamped
  today, and the `abi:` block isn't enforced.

### GCC for Linux and the BSDs

- **Linux: yes.** It is the piece that makes "everything from cvcpkg" true,
  because it owns libstdc++ and a glibc sysroot as well as the compiler.
  - It is already specified: `docs/roadmap/native-toolchain-spec.md` calls for
    GCC 14.2, a glibc 2.28 sysroot and binutils, acquired by extracting
    manylinux gcc-toolset-14 first and building with crosstool-NG later.
  - GPL with the runtime exception allows redistribution.
  - Open items:
    - The glibc floor: the spec says 2.28, while `glibc.py` defaults to 2.35.
    - There is no GLIBCXX floor gate.
    - libstdc++ must ship in the runtime closure, because GCC 14 binaries need a
      newer libstdc++ than older hosts have.
    - The catalog needs one rebuild.
  - It would also give clang a pinned `--gcc-install-dir` if clang is ever
    wanted.
- **BSDs: no.**
  - FreeBSD's and OpenBSD's base compiler is clang with libc++. A GCC/libstdc++
    would add a second C++ ABI that is incompatible with base and ports
    libraries.
  - NetBSD's base compiler is already GCC.
  - The base compiler is versioned with the OS release. The builders pin the
    releases (FreeBSD 14.4, NetBSD 10.1, OpenBSD 7.7), which already pins the
    compiler.
  - OpenBSD's ld.so has no `$ORIGIN`, so a relocatable toolchain bundle doesn't
    work there anyway.
  - Better: record the compiler ID and version in bundle manifests.

### Windows: provision a compiler when a build needs one

The goal is feasible as a **consent-gated provisioning step**. The flow:

1. `cvcpkg build X`, or `install --fallback-to-source`, needs a native compile.
2. A pre-flight check uses `vswhere` to look for a VCTools instance with the
   pinned toolset.
3. If none is found, cvcpkg prompts: "This build needs MSVC 14.44. Install
   Visual Studio Build Tools (admin, several GB, Microsoft license <url>)?"
4. On yes, it runs the provisioning recipe above (or `msvc-portable` when there
   is no admin) and continues the build.
5. CI passes an explicit accept flag.

Prerequisites in cvcpkg:
- **A compiler pre-flight.** None exists. `cvcpkg doctor`
  (`cli/_doctor.py:84-101`) only looks for `cl` on PATH, so it reports a false
  FAIL on machines with Build Tools outside a developer prompt.
- **A full fallback-to-source path.** `--fallback-to-source` builds a single
  recipe with no dependency closure and no host_tools (`installer.py:399-450`).
- The provisioning and license-gate mechanics above.
- An `msvc` builder capability, for routing.

cvcpkg's own clang can't be the bootstrap compiler on a fresh Windows machine.
`clang-cl.exe` imports MSVCP140/VCRUNTIME140. That runtime is redistributable,
so cvcpkg could ship it the way `mingw-w64-runtime` is shipped. But clang-cl
still needs MSVC's headers and libraries. `mingw-w64-gcc` (WinLibs GCC 16.1,
already in the catalog) can cover C/Fortran or `windows-gnu` recipes, but not
MSVC-ABI C++ like libcvc.

## cvcpkg issues found along the way

1. **`llvm` macOS:** `lld` and `libLLVM.dylib` load `@rpath/libzstd.1.5.7.dylib`.
   The bundle neither ships it nor declares it (only zlib is declared). cvcpkg
   does publish `zstd` 1.5.7 for macOS.
2. **`llvm`/`llvm18` Linux:** libLLVM NEEDs the system `libzstd.so.1` (and
   `libtinfo.so.6` for 18), which is undeclared. It works on Ubuntu only because
   these are base packages.
3. **`llvm` macOS archive:** every file is stored about 4 times (17,324 tar
   entries, 4,116 unique names). The Windows zip is clean; I didn't check Linux.
4. **`llvm` packaging:** the recipe's `package.files` has no `lib/clang/`, but
   the published 22.1.8+cvc.3 bundles contain `lib/clang/22/include`. The
   llvm18 recipe comment saying the top-level recipe omits them is stale.
5. **`llvm` and `llvm20`** list windows, but no Windows build is published.
   llvm18 for Windows is too old for the VS 17.14 STL.
6. **No compiler-rt in any LLVM bundle:** no sanitizers, profiling or builtins.
7. **`openmp` for Windows** (cvc.2 in the recipe repo) is not published.
8. **`cvcpkg doctor` on Windows** ignores vswhere.
9. **Windows bundles are ZIPs**, but `cvcpkg download` saves them with a
   `.tar.zst` name.

## Evidence

Commands were run in a scratch prefix on Ubuntu 22.04 (GCC 11.4); nothing was
installed system-wide. The macOS and Windows bundles were inspected with the
Linux `llvm-objdump`/`llvm-readobj` from the llvm18 bundle.

```text
$ cvcpkg search llvm --platform {linux,macos,windows}
llvm   22.1.8+cvc.3  linux x86_64  shared release 221.2 MB
llvm18 18.1.8+cvc.3  linux x86_64  shared release 208.8 MB
llvm   22.1.8+cvc.3  macos arm64   shared release 345.0 MB
llvm20 20.1.8+cvc.1  macos arm64   shared release 185.2 MB   # no clang
llvm18 18.1.8+cvc.3  windows x86_64 shared release 852.6 MB  # only Windows LLVM

$ readelf -d p-llvm18/bin/clang-18 | grep -E 'NEEDED|RUNPATH'
 NEEDED libclang-cpp.so.18.1, libLLVM.so.18.1, libstdc++.so.6, libc.so.6
 RUNPATH [$ORIGIN:$ORIGIN/../lib]
$ ldd p-llvm18/bin/clang-18 | grep -E 'zstd|tinfo'
 libzstd.so.1 => /lib/x86_64-linux-gnu/libzstd.so.1
 libtinfo.so.6 => /lib/x86_64-linux-gnu/libtinfo.so.6
$ objdump -T ... | max symbol versions
 clang-18: GLIBC_2.34 GLIBCXX_3.4.29   libLLVM.so.18.1: GLIBC_2.34 GLIBCXX_3.4.30
 builder: GCC: (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0
 (llvm 22: same RUNPATH and floors; libLLVM.so.22.1 also NEEDs libzstd.so.1)

$ p-llvm18/bin/clang++ -v hello.cpp
 Selected GCC installation: /usr/lib/gcc/x86_64-linux-gnu/11
 -internal-isystem /usr/lib/gcc/x86_64-linux-gnu/11/../../../../include/c++/11
 ".../p-llvm18/bin/ld.lld" ... /usr/lib/gcc/x86_64-linux-gnu/11/crtbeginS.o ...
   -lstdc++ -lm -lgcc_s -lgcc -lc ...

$ clang++ (18 and 22) abi.cpp -lcvc -lboost_filesystem -lboost_program_options -lhdf5
  # against a local libcvc 3.5.0 cvcpkg prefix (all GCC 11.4-built)
boost path: /tmp/x/y.h5 stem=y
caught boost exception thrown by GCC code: unrecognised option '--bogus'
hdf5 1.14.4
libcvc ip: '…' len=12

$ clang++ -fopenmp omp.cpp                     -> fatal error: 'omp.h' file not found
$ clang++ -fopenmp=libgomp -### omp.cpp | grep -c '"-fopenmp"'   -> 0
$ cvcpkg install openmp; clang++ -fopenmp -I.../include omp.cpp -L.../lib
threads=4 sum=499999500000            NEEDED libomp.so

$ llvm-objdump --macho --dylibs-used / --private-headers  (llvm 22 macOS)
 clang-22:     @rpath/libclang-cpp.dylib @rpath/libLLVM.dylib /usr/lib/libc++.1.dylib
               LC_RPATH @loader_path, @loader_path/../lib   minos 13.0 sdk 26.5
 lld:          @rpath/libz.1.3.1.dylib @rpath/libzstd.1.5.7.dylib @rpath/libLLVM.dylib
 libLLVM.dylib: /usr/lib/libedit.3.dylib @rpath/libzstd.1.5.7.dylib ...
$ tar -tf llvm-22.1.8+cvc.3-macos-arm64-release-shared.tar.zst | grep -c libzstd   -> 0
  17324 entries / 4116 unique

$ llvm-readobj --coff-imports bin/clang-cl.exe   (llvm18 Windows)
 ADVAPI32 KERNEL32 MSVCP140 VCRUNTIME140 VCRUNTIME140_1 api-ms-win-crt-* zlib1.dll ...
 lib/clang/18/include present (230 files); no lib/clang/18/lib (no compiler-rt)

$ curl raw.githubusercontent.com/microsoft/STL/vs-2022-17.14/stl/inc/yvals_core.h
 #if __clang_major__ < 19 ... STL1000 "Unexpected compiler version, expected Clang 19.0.0 or newer."
 (vs-2022-17.12: Clang 17; msvc-build-tools-14.50: Clang 19; main: Clang 22, CUDA 13.4)

$ gh run view 38171168600 --job <macos> --log | grep OpenMP
 -- OpenMP not found - disabling OpenMP support     (AppleClang 21.0.0.21000101)

$ grep cvcTargets.cmake (3.5.0 Linux SDK)
 INTERFACE_COMPILE_OPTIONS "-frounding-math"
 INTERFACE_LINK_LIBRARIES "...;OpenMP::OpenMP_CXX;..."
```

Not verified (no hardware or toolkit available here):
- clang on an actual macOS or Windows runner.
- nvcc paired with any clang.
- Which GCC install clang selects on ubuntu-24.04 images.
