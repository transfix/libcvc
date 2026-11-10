# Vendored third-party code

Header-only libraries copied into libcvc. They are private: only libcvc's `.cpp` files (and its
tests and `tools/cgal_ab`) include them, so no installed header depends on them. Formatting is
off here (`.clang-format`). Each one's notice is in `THIRD_PARTY_NOTICES.md`.

| Library | Version | Licence | Source | Used by |
|---|---|---|---|---|
| nanoflann | 2.0.1 | BSD-2-Clause (`nanoflann/COPYING`) | https://github.com/jlblancoc/nanoflann, `include/nanoflann.hpp` at tag `2.0.1` | `nav/spatial.cpp` (fixed-radius neighbours), `geometry/project_verts.cpp` (nearest reference vertices) |
| CDT | 2.0.1 | MPL-2.0 (`CDT/LICENSE`); its `predicates.h` BSD-3-Clause (`CDT/LICENSE-predicates.txt`), its `portable_nth_element.hpp` Apache-2.0 WITH LLVM-exception (`CDT/LICENSE-LLVM.txt`) | https://github.com/artem-ogre/CDT, `CDT/include/` at tag `2.0.1` (tarball sha256 `5e3c4571...367d`), two files changed (below) | 2D constrained Delaunay for the VolumeRover ports (`cvc::planar`, P3a); `tests/permissive_libs_test.cpp`, `tools/cgal_ab` |
| Shewchuk's predicates | May 18, 1996 | public domain | https://www.cs.cmu.edu/~quake/robust.html, `predicates.c` (sha256 `f8662c3f...2029`), turned into `shewchuk/predicates.hpp` by `shewchuk/gen_predicates.py` | exact orientation and in-circle / in-sphere tests for the ports (`cvc::tet_complex`, P1; `cvc::planar`, P3a); the same tests and tools |

Qhull and Clipper2, the ports' other two permissive libraries, are compiled libraries and come
from cvcpkg packages instead (`CVC_WITH_QHULL`, `CVC_WITH_CLIPPER2` in `src/cvc/CMakeLists.txt`).
The build-tree interface target `cvc_permissive_geometry` puts this directory on the include
path (`#include "CDT/CDT.h"`, `#include "shewchuk/predicates.hpp"`) together with those two.

## What is changed from upstream, and why

libcvc holds no mutable process-wide state (all state lives under a `cvc::app` or in objects the
caller owns; VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.3), and that holds for vendored code too.

- **nanoflann:** unchanged.
- **CDT**, two files:
  - `Triangulation.hpp`: `static double log2_constant` in `detail::log2_bc` becomes
    `static const double`. It is never written, and is compiled only before C++11.
  - `predicates.h` (William Lenthe's predicates): `Epsilon<T>()`, `ExpansionBase<T>::Splitter`
    and the `Constants<T>` error bounds were `static const` values with dynamic initialisers
    (`std::exp2`, a function call), so each object file got writable copies and guard variables.
    They are initialised from constant expressions now (`numeric_limits<T>::epsilon() / 2`,
    `1ULL << (digits / 2 + 1)`), with the same values.
- **Shewchuk's predicates:** generated, not copied (`python shewchuk/gen_predicates.py
  <predicates.c> shewchuk/predicates.hpp`; Python 3, standard library only). The header's
  prologue lists the changes: `exactinit()`'s globals become `constexpr` constants (so there is
  no initialisation call and nothing mutable), only the four adaptive predicates and what they
  call are kept, floating-point contraction is switched off around them, and `insphereexact`'s
  ~200 KB of arrays live in one heap block instead of on the stack (a wasm thread's stack is
  64 KB unless the program sets `-sSTACK_SIZE`). The adaptive stages still use up to ~25 KB of
  stack.

## Checking for mutable state

Check again whenever a library here is updated, and for the packaged ones when their version
changes.

1. **The sources.** No namespace-scope or static variable that is not `const`:

   ```bash
   grep -rnE "^\s*(static|thread_local)\b[^(]*(=|;)" src/cvc/third_party \
     --include=*.h --include=*.hpp | grep -vE "static (constexpr|const)\b|static_assert|static_cast"
   ```

   It prints nothing. (CDT's `const static` index constants and Lenthe's `static const`
   members are constants.)

2. **The object files**, which also catch what a grep misses (a static in a macro or a template,
   a dynamic initialiser). List the symbols in writable sections of a translation unit that uses
   each library, with `nm` on Linux, or `llvm-nm` on MSVC's COFF objects:

   ```bash
   llvm-nm -C permissive_libs_test.obj | grep -E " [dDbB] "
   ```

   Checked 2026-11-09 with MSVC 19.44 (`/O2`):
   - `shewchuk/predicates.hpp`: nothing.
   - `CDT/CDT.h` (a `Triangulation<double>` built and used): only the compiler's RTTI
     descriptors for CDT's exception types. Before the `predicates.h` change it also had
     `Epsilon<double>()`'s static and guard, `Splitter` and nine `Constants<double>` members.
   - Clipper2 (the package's headers and `Clipper2.lib`): `const` objects MSVC places in `.data`
     (`InvalidPoint64`, `InvalidPointD`, `InvalidRect64`, `InvalidRectD`, `invalid_rect`, whose
     constructors are not `constexpr`), and the five error-message pointers
     (`Clipper2Lib::precision_error` etc., `static const char *` whose pointers are not
     `const`). None is ever written; they are what the object scan's allowlist needs for a
     translation unit that includes `clipper2/clipper.h`.
   - Qhull (`qhullstatic_r.lib`): one table, `mergetypes` (`const char *mergetypes[]` in
     `merge_r.c`: merge-type names for trace output, never written). Everything else lives in
     the caller's `qhT`; `qh_clock`'s function-local static is compiled only with
     `qh_CLOCKtype 2`, and the default is 1.
