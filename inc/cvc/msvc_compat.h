// libcvc: MSVC force-include compatibility shim.
//
// The Universal CRT's <stdio.h> exposes legacy POSIX spellings of a few
// functions via a textual macro remap (e.g. `#define snprintf _snprintf`)
// when the internal macro _CRT_INTERNAL_NONSTDC_NAMES is non-zero. On
// modern Windows SDKs (10.0.26100+) that macro is re-asserted by
// corecrt.h unconditionally, so attempting to turn the remap off via a
// command-line -D_CRT_INTERNAL_NONSTDC_NAMES=0 both fails to take effect
// *and* triggers warning C4005 (macro redefinition) which cascades into
// C2220 in targets compiled with /WX (e.g. vendored googletest).
//
// The remap textually rewrites `std::snprintf(...)` into
// `std::_snprintf(...)`, which is a compile error because `_snprintf`
// is not in namespace std. Boost.Core (type_name.hpp) and Boost.Asio
// (config.ipp) both trigger this when consumed on Windows.
//
// Work around it here by pulling in <cstdio> once and then removing the
// offending macro definition if it was introduced. This header is
// force-included via `/FI` on MSVC from the top-level CMakeLists.txt so
// that every translation unit sees the fix before any third-party
// header is parsed.
#ifndef CVC_MSVC_COMPAT_H
#define CVC_MSVC_COMPAT_H

#if defined(_MSC_VER)
#  include <cstdio>
#  ifdef snprintf
#    undef snprintf
#  endif
#  ifdef vsnprintf
#    undef vsnprintf
#  endif
#endif

#endif  // CVC_MSVC_COMPAT_H
