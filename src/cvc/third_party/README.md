# Vendored third-party code

Header-only libraries copied into libcvc unchanged. They are private: only libcvc's `.cpp` files
include them, so no installed header depends on them. Formatting is off here (`.clang-format`).

| Library | Version | Licence | Source | Used by |
|---|---|---|---|---|
| nanoflann | 2.0.1 | BSD-2-Clause (`nanoflann/COPYING`) | https://github.com/jlblancoc/nanoflann, `include/nanoflann.hpp` at tag `2.0.1` | `nav/spatial.cpp` (fixed-radius neighbours), `geometry/project_verts.cpp` (nearest reference vertices) |

nanoflann holds no mutable global or static state, so it fits libcvc's rule that all state lives
under a `cvc::app` or in objects the caller owns. Check that again when updating it:

```bash
grep -n "^\s*static [^c(]\|thread_local" src/cvc/third_party/nanoflann/nanoflann.hpp \
  | grep -v "static constexpr\|static inline\|static_assert\|static_cast\|static const\b"
```
