# Vendored third-party code

libcvc takes third-party code from cvcpkg packages, not by vendoring it: the VolumeRover ports'
libraries (Qhull, Clipper2, CDT, Shewchuk's predicates) are the `qhull`, `clipper2`, `cdt` and
`shewchuk-predicates` packages (`CVC_WITH_*` in `src/cvc/CMakeLists.txt`). What is left here
predates that rule, and moves to a package once one is published.

| Library | Version | Licence | Source | Used by |
|---|---|---|---|---|
| nanoflann | 2.0.1 | BSD-2-Clause (`nanoflann/COPYING`) | https://github.com/jlblancoc/nanoflann, `include/nanoflann.hpp` at tag `2.0.1` | `nav/spatial.cpp` (fixed-radius neighbours), `geometry/project_verts.cpp` (nearest reference vertices) |

nanoflann is copied unchanged. It is private: only libcvc's `.cpp` files include it, so no
installed header depends on it. Formatting is off here (`.clang-format`). cvcpkg's `nanoflann`
2.0.1 recipe (cy-pca/cvcpkg#597) packages the same header; libcvc switches to it once it is
published.

nanoflann holds no mutable global or static state, so it fits libcvc's rule that all state lives
under a `cvc::app` or in objects the caller owns. Check that again when updating it:

```bash
grep -n "^\s*static [^c(]\|thread_local" src/cvc/third_party/nanoflann/nanoflann.hpp \
  | grep -v "static constexpr\|static inline\|static_assert\|static_cast\|static const\b"
```
