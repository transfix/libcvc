"""Turn Shewchuk's predicates.c (May 18, 1996, public domain) into libcvc's reentrant header.

usage: python -I gen_predicates.py <predicates.c> <out predicates.hpp>

Only the four adaptive predicates and the routines they call are kept. The bodies are copied
verbatim; the mechanical changes are listed in the header's prologue.
"""
import re
import sys

src_path, out_path = sys.argv[1], sys.argv[2]
lines = open(src_path, encoding="latin-1").read().split("\n")

# The routines kept, in dependency order. Pointer parameters that are written are listed here;
# every other pointer parameter becomes const.
FUNCS = [
    ("fast_expansion_sum_zeroelim", {"h"}),
    ("scale_expansion_zeroelim", {"h"}),
    ("estimate", set()),
    ("orient2dadapt", set()),
    ("orient2d", set()),
    ("orient3dadapt", set()),
    ("orient3d", set()),
    ("incircleadapt", set()),
    ("incircle", set()),
    ("insphereexact", set()),
    ("insphereadapt", set()),
    ("insphere", set()),
]
PUBLIC = ["orient2d", "orient3d", "incircle", "insphere"]


def code_subst(s):
    if s.lstrip().startswith("/*"):
        return s  # comments stay as upstream wrote them
    s = re.sub(r"\bINEXACT REAL\b", "double", s)
    s = re.sub(r"\bINEXACT\b\s*", "", s)
    s = re.sub(r"\bREAL\b", "double", s)
    return s


def find_line(pred, start=0):
    for i in range(start, len(lines)):
        if pred(lines[i]):
            return i
    raise SystemExit("not found")


# Macros: from `#define Absolute(a)` through the end of `Two_Square`.
m_begin = find_line(lambda l: l.startswith("#define Absolute(a)"))
m_end = find_line(lambda l: l.startswith("#define Two_Square("), m_begin)
while lines[m_end].rstrip().endswith("\\"):
    m_end += 1
macro_lines = lines[m_begin:m_end + 1]
# Drop the commented-out alternative `fabs` definition.
macro_lines = [l for l in macro_lines if not l.startswith("/* #define")]
macro_names = []
for l in macro_lines:
    mm = re.match(r"#define (\w+)", l)
    if mm:
        macro_names.append(mm.group(1))

out = []


def emit_function(name, written):
    i = find_line(lambda l: re.match(r"^(REAL|int) %s\(" % name, l) is not None)
    sig = lines[i]
    ret = "double" if sig.startswith("REAL") else "int"
    params = [p.strip() for p in sig[sig.index("(") + 1:sig.index(")")].split(",")]
    decls = {}
    j = i + 1
    while lines[j].strip() != "{":
        d = lines[j].split("/*")[0].strip().rstrip(";")
        if d:
            ty, nm = d.rsplit(" ", 1)
            ptr = nm.startswith("*")
            nm = nm.lstrip("*")
            ty = code_subst(ty)
            if ptr:
                decls[nm] = ("%s *%s" % (ty, nm)) if nm in written else ("const %s *%s" % (ty, nm))
            else:
                decls[nm] = "%s %s" % (ty, nm)
        j += 1
    trailing = sig[sig.index(")") + 1:].strip()
    out.append("inline %s %s(%s)%s" % (ret, name, ", ".join(decls[p] for p in params),
                                         (" " + trailing) if trailing else ""))
    k = j
    body = []
    while lines[k] != "}":
        body.append(code_subst(lines[k]))
        k += 1
    if name in HEAP_SCRATCH:
        body = heap_scratch(name, body)
    out.extend(body)
    out.append("}")
    out.append("")


# insphereexact's arrays add up to about 24,900 doubles (~200 KB), more than a thread's stack on
# some targets (Emscripten's default is 64 KB, and an overflow there is silent). The arrays of 96
# or more elements move to one heap block; references keep their names and array types, so the
# body below is unchanged. It is the last-resort exact stage, so the allocation is rare.
HEAP_SCRATCH = {"insphereexact"}
HEAP_MIN = 96


def heap_scratch(name, body):
    moved = []
    kept = []
    for l in body:
        mm = re.match(r"^(\s*)double (.*);\s*$", l)
        if mm and "[" in mm.group(2):
            small = []
            for item in mm.group(2).split(","):
                item = item.strip()
                am = re.match(r"(\w+)\[(\d+)\]$", item)
                if am and int(am.group(2)) >= HEAP_MIN:
                    moved.append((am.group(1), int(am.group(2))))
                else:
                    small.append(item)
            if small:
                kept.append("%sdouble %s;" % (mm.group(1), ", ".join(small)))
            continue
        kept.append(l)
    total = sum(n for _, n in moved)
    decl = ["  /* libcvc: the large arrays live in one heap block (%d doubles), not on the stack. */"
            % total,
            "  struct scratch_t { double %s; };" % ", ".join("%s[%d]" % m for m in moved),
            "  const std::unique_ptr<scratch_t> scratch(new scratch_t);"]
    decl += ["  double (&%s)[%d] = scratch->%s;" % (nm, n, nm) for nm, n in moved]
    # The block goes after the opening brace and the declarations it replaced.
    assert kept[0].strip() == "{"
    return [kept[0]] + decl + kept[1:]


prologue = r'''// Shewchuk's adaptive robust geometric predicates, as a reentrant C++ header.
//
// Upstream: predicates.c, "Routines for Arbitrary Precision Floating-point Arithmetic and Fast
// Robust Geometric Predicates", Jonathan Richard Shewchuk, May 18, 1996, placed in the public
// domain. https://www.cs.cmu.edu/~quake/robust.html,
// https://www.cs.cmu.edu/afs/cs/project/quake/public/code/predicates.c
// (sha256 f8662c3f407d1c1c5dcd4dd49ea8b8ddd801a71827d65734206ddc3741792029).
//
// GENERATED from that file by the script described in README.md; do not edit by hand.
//
// What differs from upstream, all mechanical:
// - exactinit() is gone. Its globals (epsilon, splitter and the twelve error bounds) are
//   constexpr constants with the values exactinit() computes for IEEE-754 binary64 with
//   round-to-nearest-even: epsilon = 2^-53, splitter = 2^27 + 1. So there is no mutable state at
//   all and no initialisation call: every routine keeps its state on its own stack, and the
//   predicates are safe to call from any number of threads (libcvc's no-singletons rule).
// - Only the four adaptive predicates (orient2d, orient3d, incircle, insphere) and the routines
//   they call are kept. Their bodies are upstream's, verbatim, with REAL spelled double and
//   INEXACT (empty upstream) dropped.
// - K&R definitions became inline ANSI ones in namespace shewchuk::detail; input arrays are
//   const. The public names are shewchuk::orient2d etc., plus the typed wrappers at the end.
// - insphereexact's large arrays (about 200 KB of them) live in one heap block instead of on the
//   stack, which a wasm thread (64 KB by default) would silently overflow. The adaptive stages
//   keep theirs on the stack: at most about 25 KB, in incircleadapt and insphereadapt.
// - Floating-point contraction is switched off around the code (clang, GCC): fusing a*b+c into
//   one FMA would break the error-free transformations (Two_Product and friends). The macros
//   are #undef'd at the end, so nothing leaks to the includer.
//
// Signs, as upstream documents them:
// - orient2d(a, b, c) > 0 when a, b, c are counterclockwise (CGAL::orientation: LEFT_TURN).
// - orient3d(a, b, c, d) > 0 when d lies below the plane of a, b, c, "below" being the side from
//   which a, b, c appear clockwise. That is the opposite sign to CGAL::orientation(a, b, c, d).
// - incircle(a, b, c, d) > 0 when d is inside the circle through a, b, c, for a, b, c
//   counterclockwise; the sign flips when they are clockwise.
// - insphere(a, b, c, d, e) > 0 when e is inside the sphere through a, b, c, d, for
//   orient3d(a, b, c, d) > 0; the sign flips when that orientation is negative.
// Each returns an approximation of the determinant whose sign is exact.

/*****************************************************************************/
/*                                                                           */
/*  Routines for Arbitrary Precision Floating-point Arithmetic               */
/*  and Fast Robust Geometric Predicates                                     */
/*  (predicates.c)                                                           */
/*                                                                           */
/*  May 18, 1996                                                             */
/*                                                                           */
/*  Placed in the public domain by                                           */
/*  Jonathan Richard Shewchuk                                                */
/*  School of Computer Science                                               */
/*  Carnegie Mellon University                                               */
/*  5000 Forbes Avenue                                                       */
/*  Pittsburgh, Pennsylvania  15213-3891                                     */
/*  jrs@cs.cmu.edu                                                           */
/*                                                                           */
/*****************************************************************************/

#ifndef CVC_THIRD_PARTY_SHEWCHUK_PREDICATES_HPP
#define CVC_THIRD_PARTY_SHEWCHUK_PREDICATES_HPP

#if defined(__GNUC__)
#pragma GCC system_header
#endif

#include <array>
#include <limits>
#include <memory>

static_assert(std::numeric_limits<double>::is_iec559 && std::numeric_limits<double>::digits == 53,
              "Shewchuk's predicates need IEEE-754 binary64 doubles");
#if defined(__FLT_EVAL_METHOD__) && (__FLT_EVAL_METHOD__ == 2 || __FLT_EVAL_METHOD__ < 0)
#error "Shewchuk's predicates need double arithmetic without extended precision (no x87)"
#endif

#if defined(__clang__)
#pragma float_control(push)
#pragma clang fp contract(off)
#elif defined(__GNUC__)
#pragma GCC push_options
#pragma GCC optimize("fp-contract=off")
#endif
#if defined(_MSC_VER)
#pragma warning(push, 0)
#endif

namespace shewchuk {
namespace detail {

/* The constants exactinit() computed, for binary64 with round-to-nearest-even. */
constexpr double epsilon = 1.0 / 9007199254740992.0; /* 2^-53 */
constexpr double splitter = 134217729.0;             /* 2^27 + 1 */
constexpr double resulterrbound = (3.0 + 8.0 * epsilon) * epsilon;
constexpr double ccwerrboundA = (3.0 + 16.0 * epsilon) * epsilon;
constexpr double ccwerrboundB = (2.0 + 12.0 * epsilon) * epsilon;
constexpr double ccwerrboundC = (9.0 + 64.0 * epsilon) * epsilon * epsilon;
constexpr double o3derrboundA = (7.0 + 56.0 * epsilon) * epsilon;
constexpr double o3derrboundB = (3.0 + 28.0 * epsilon) * epsilon;
constexpr double o3derrboundC = (26.0 + 288.0 * epsilon) * epsilon * epsilon;
constexpr double iccerrboundA = (10.0 + 96.0 * epsilon) * epsilon;
constexpr double iccerrboundB = (4.0 + 48.0 * epsilon) * epsilon;
constexpr double iccerrboundC = (44.0 + 576.0 * epsilon) * epsilon * epsilon;
constexpr double isperrboundA = (16.0 + 224.0 * epsilon) * epsilon;
constexpr double isperrboundB = (5.0 + 72.0 * epsilon) * epsilon;
constexpr double isperrboundC = (71.0 + 1408.0 * epsilon) * epsilon * epsilon;
'''

out.append(prologue)
out.extend(code_subst(l) for l in macro_lines)
out.append("")
for name, written in FUNCS:
    emit_function(name, written)

out.append("} // namespace detail")
out.append("")
out.append(r'''// The public predicates, and typed wrappers over them.
using detail::orient2d;
using detail::orient3d;
using detail::incircle;
using detail::insphere;

using point2 = std::array<double, 2>;
using point3 = std::array<double, 3>;

inline double orient2d(const point2 &a, const point2 &b, const point2 &c) {
  return detail::orient2d(a.data(), b.data(), c.data());
}
inline double orient3d(const point3 &a, const point3 &b, const point3 &c, const point3 &d) {
  return detail::orient3d(a.data(), b.data(), c.data(), d.data());
}
inline double incircle(const point2 &a, const point2 &b, const point2 &c, const point2 &d) {
  return detail::incircle(a.data(), b.data(), c.data(), d.data());
}
inline double insphere(const point3 &a, const point3 &b, const point3 &c, const point3 &d,
                       const point3 &e) {
  return detail::insphere(a.data(), b.data(), c.data(), d.data(), e.data());
}

// -1, 0 or +1: the exact sign of a predicate's result.
inline int sign(double v) { return (v > 0.0) - (v < 0.0); }

} // namespace shewchuk
''')
for nm in macro_names:
    out.append("#undef %s" % nm)
out.append("")
out.append(r'''#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#if defined(__clang__)
#pragma float_control(pop)
#elif defined(__GNUC__)
#pragma GCC pop_options
#endif

#endif // CVC_THIRD_PARTY_SHEWCHUK_PREDICATES_HPP''')
out.append("")

with open(out_path, "w", encoding="utf-8", newline="\n") as fh:
    fh.write("\n".join(out))
print("wrote", out_path, "macros:", len(macro_names))
