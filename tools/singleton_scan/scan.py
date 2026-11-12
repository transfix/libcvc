#!/usr/bin/env python3
"""The writable-data scan of libcvc's object files.

VOLROVER-LIBRARY-PORTS-ROADMAP.md section 1.3 says all of libcvc's state lives under a
``cvc::app`` or in objects the caller owns. This scan is how that is checked: for every object
file of the scanned CMake targets it lists the writable data symbols (``nm``'s ``.data`` and
``.bss``, thread-locals, function-local statics and their guard variables) and fails on

- any symbol that is not on the allowlist (a new process-wide object), and
- any allowlist entry this build no longer has (the object moved onto the app: delete the line,
  so the allowlist only ever shrinks to what is left).

It reads ELF objects through GNU ``nm`` and ``c++filt``, so it runs on the Linux CI lanes. See
README.md next to it for the allowlist's format and how to regenerate it.
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import dataclasses
import os
import re
import subprocess
import sys
from typing import Callable, Iterable, Sequence

HEADER = "<header>"
"""The source of a symbol with vague linkage (an inline function's static, a template's static
member, their guards). Any object that includes the header may define it, so it is keyed by
target and symbol, not by object."""

KINDS = ("data", "bss", "tls", "guard", "common")

# Toolchain artifacts: written per translation unit by the compiler or the standard library,
# never by a line of libcvc, so no source change can remove them. Matched on demangled names.
TOOLCHAIN = (
    (re.compile(r"^std::__ioinit$"), "libstdc++'s per-TU <iostream> initialiser (GCC < 13)"),
    (re.compile(r"^(__gcov|__llvm_gcov|__llvm_prf|__profc_|__profd_)"), "coverage counters"),
    (re.compile(r"^DW\.ref\."), "the exception-handling personality routine's address"),
    (
        re.compile(r"^(__cuda|__nv_|__fatDeviceText)"),
        "nvcc's host-side fatbinary registration stubs",
    ),
)

# Generated code: the tables a code generator writes into its own output, which no libcvc source
# defines. (source pattern, symbol pattern, reason); matched on the object's source and the
# demangled name.
GENERATED = (
    (
        re.compile(r"PYTHON_wrap\.cxx$"),
        re.compile(
            r"^(_swigt__|_swigc__|swig_|Swig|SWIG_|swig::|interpreter_counter$"
            r"|PyInit__\w+::(SWIG_module|SwigSlots)$|PyArray_API$|PyArray_RUNTIME_VERSION$)"
        ),
        "SWIG's generated module runtime (type and cast tables) and NumPy's C-API table",
    ),
    (re.compile(r"\.pb\.cc$"), re.compile(r""), "protoc's generated message tables"),
)

# Third-party libraries' own statics: defined by their headers or templates in our objects, so no
# libcvc source change can remove them short of dropping the library. Matched on demangled names.
# Everything else, vendored code included, is libcvc's and must move onto the app.
THIRD_PARTY = (
    (re.compile(r"^boost::"), "Boost's statics (its templates and headers, instantiated here)"),
    (
        re.compile(r"^(vtk\w*(Manager|Cleanup)Instance|vtksys::SystemToolsManagerInstance)$"),
        "VTK's per-file header objects (its Schwarz counters)",
    ),
    (re.compile(r"^igl::"), "libigl's statics (its colormaps and thread pool)"),
    (re.compile(r"^Eigen::"), "Eigen's cache-size table"),
    (re.compile(r"^pocketfft::"), "pocketfft's thread pool"),
    (re.compile(r"^std::"), "the standard library's statics"),
)

_GUARD = "guard variable for "

# One line of `nm -f sysv`: name|value|class|type|size|line|section
_SYSV_FIELDS = 7
_SYMBOLS_FROM = re.compile(r"^Symbols from (.*):$")

# Classes with vague linkage: u (GNU unique), V/v (weak object), W/w (weak).
_VAGUE = frozenset("uVvWw")


@dataclasses.dataclass(frozen=True)
class Symbol:
    """One defined symbol of one object, as nm printed it."""

    obj: str
    name: str
    cls: str
    typ: str
    section: str


@dataclasses.dataclass(frozen=True)
class Finding:
    """One writable data symbol, keyed as the allowlist keys it."""

    target: str
    source: str
    kind: str
    symbol: str

    @property
    def key(self) -> tuple[str, str, str]:
        return (self.target, self.source, self.symbol)

    def line(self) -> str:
        return f"{self.target} {self.source} {self.kind} {self.symbol}"


@dataclasses.dataclass(frozen=True)
class Entry:
    """One allowlist line."""

    target: str
    source: str
    kind: str
    symbol: str
    lineno: int

    @property
    def key(self) -> tuple[str, str, str]:
        return (self.target, self.source, self.symbol)

    def line(self) -> str:
        return f"{self.target} {self.source} {self.kind} {self.symbol}"


# ---------------------------------------------------------------------------------------------
# Classification


def section_kind(section: str, cls: str) -> str | None:
    """The kind of writable storage a section holds, or None if it is not writable data."""
    if cls in ("C", "c") or section == "*COM*":
        return "common"
    s = section
    if s.startswith((".tbss", ".tdata", ".gnu.linkonce.tb.", ".gnu.linkonce.td.")):
        return "tls"
    if s.startswith((".bss", ".sbss", ".lbss", ".gnu.linkonce.b.", ".gnu.linkonce.sb.")):
        return "bss"
    # RELRO: written by the dynamic loader's relocations, read-only before main() runs.
    if s.startswith(".data.rel.ro"):
        return None
    if s.startswith((".data", ".sdata", ".ldata", ".gnu.linkonce.d.", ".gnu.linkonce.s.")):
        return "data"
    return None


def symbol_kind(sym: Symbol) -> str | None:
    """The allowlist kind of a symbol, or None if it is not writable data."""
    kind = section_kind(sym.section, sym.cls)
    if kind is None:
        return None
    if sym.typ in ("FUNC", "SECTION", "FILE"):
        return None
    # _ZGV: a static's guard; __tls_guard: the guard of a TU's dynamically initialised
    # thread-locals.
    if sym.name.startswith("_ZGV") or sym.name == "__tls_guard":
        return "guard"
    return kind


def left_out_reason(source: str, demangled: str) -> str | None:
    """Why a symbol is not libcvc's to move (TOOLCHAIN, GENERATED, THIRD_PARTY), or None.
    `source` is the object's own source file; a guard is judged by the object it guards."""
    name = demangled[len(_GUARD) :] if demangled.startswith(_GUARD) else demangled
    for pattern, reason in TOOLCHAIN:
        if pattern.search(name):
            return reason
    for src_pattern, pattern, reason in GENERATED:
        if src_pattern.search(source) and pattern.search(name):
            return reason
    for pattern, reason in THIRD_PARTY:
        if pattern.search(name):
            return reason
    return None


def subsystem(target: str, source: str) -> str:
    """The audit's subsystem for a source path, for the summary table."""
    if source == HEADER:
        return f"{target} (headers)"
    p = source.split("/")
    if len(p) >= 4 and p[0] == "src" and p[1] == "cvc":
        sub = p[2]
        if sub == "state" and p[3] == "state_exec":
            return "state_exec"
        if sub == "geometry" and p[3] == "cvc-mesher":
            return "cvc-mesher (vendored)"
        if sub == "geometry" and p[3] == "SDF":
            return "SDF (vendored)"
        if sub == "xmlrpc":
            return "xmlrpc"
        return sub
    if len(p) >= 2 and p[0] == "src":
        return {
            "cvcGL": "cvcGL",
            "cvcMedia": "cvcMedia",
            "cvc-cli": "cli",
            "xmlrpc": "xmlrpc (vendored)",
        }.get(p[1], p[1])
    if p[0] == "bindings":
        return "pycvc"
    return p[0]


# ---------------------------------------------------------------------------------------------
# Objects


def find_objects(build_dir: str, targets: Iterable[str]) -> dict[str, list[tuple[str, str]]]:
    """Each target's object files under a CMake build tree, as (object path, source path).

    An object of target T lives at ``<build>/<dir>/CMakeFiles/T.dir/<rel>.o``. Its source is
    ``<dir>/<rel>``, with CMake's ``__`` for ``..`` undone: the build tree mirrors the source
    tree, so that is the path in the repository (a generated source gets its build-tree path).
    """
    wanted = {f"{t}.dir": t for t in targets}
    found: dict[str, list[tuple[str, str]]] = {t: [] for t in targets}
    build_dir = os.path.abspath(build_dir)
    for root, dirs, _files in os.walk(build_dir):
        if os.path.basename(root) != "CMakeFiles":
            continue
        for d in list(dirs):
            target = wanted.get(d)
            if target is None:
                continue
            target_dir = os.path.join(root, d)
            src_dir = os.path.relpath(os.path.dirname(root), build_dir)
            for troot, _tdirs, tfiles in os.walk(target_dir):
                for f in tfiles:
                    if not f.endswith(".o"):
                        continue
                    obj = os.path.join(troot, f)
                    found[target].append((obj, object_source(obj, target_dir, src_dir)))
    for objs in found.values():
        objs.sort()
    return found


def object_source(obj: str, target_dir: str, src_dir: str) -> str:
    rel = os.path.relpath(obj, target_dir).replace(os.sep, "/")
    if rel.endswith(".o"):
        rel = rel[:-2]
    parts = [".." if s == "__" else s for s in rel.split("/")]
    base = [] if src_dir in ("", ".") else src_dir.replace(os.sep, "/").split("/")
    return os.path.normpath("/".join(base + parts)).replace(os.sep, "/")


# ---------------------------------------------------------------------------------------------
# nm and c++filt


def parse_sysv(text: str, default_obj: str = "") -> list[Symbol]:
    """Parse ``nm -f sysv`` output, one or many objects."""
    out: list[Symbol] = []
    obj = default_obj
    for raw in text.splitlines():
        line = raw.rstrip()
        m = _SYMBOLS_FROM.match(line)
        if m:
            obj = m.group(1)
            continue
        if "|" not in line:
            continue
        fields = [f.strip() for f in line.split("|")]
        if len(fields) < _SYSV_FIELDS:
            continue
        name, _value, cls, typ, _size, _line, section = fields[:_SYSV_FIELDS]
        if not name:
            continue
        out.append(Symbol(obj=obj, name=name, cls=cls, typ=typ, section=section))
    return out


def run_nm(nm: str, objs: Sequence[str]) -> str:
    proc = subprocess.run(
        [nm, "-f", "sysv", "--defined-only", "--", *objs],
        check=False,
        capture_output=True,
        text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"{nm} failed ({proc.returncode}): {proc.stderr.strip()}")
    if len(objs) == 1 and "Symbols from" not in proc.stdout:
        return f"Symbols from {objs[0]}:\n{proc.stdout}"
    return proc.stdout


def demangle(cxxfilt: str, names: Sequence[str]) -> dict[str, str]:
    if not names:
        return {}
    proc = subprocess.run(
        [cxxfilt],
        input="\n".join(names) + "\n",
        check=True,
        capture_output=True,
        text=True,
    )
    lines = proc.stdout.splitlines()
    if len(lines) != len(names):
        raise RuntimeError(f"{cxxfilt} returned {len(lines)} names for {len(names)}")
    return dict(zip(names, lines))


def scan_symbols(
    nm: str,
    objs: Sequence[str],
    runner: Callable[[str, Sequence[str]], str] = run_nm,
    chunk: int = 64,
) -> list[Symbol]:
    chunks = [objs[i : i + chunk] for i in range(0, len(objs), chunk)]
    out: list[Symbol] = []
    workers = max(1, min(len(chunks), os.cpu_count() or 1))
    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as pool:
        for text in pool.map(lambda c: runner(nm, c), chunks):
            out.extend(parse_sysv(text))
    return out


# ---------------------------------------------------------------------------------------------
# Findings


def collect(
    objects: dict[str, list[tuple[str, str]]],
    symbols: Iterable[Symbol],
    names: dict[str, str],
) -> tuple[set[Finding], collections.Counter]:
    """Writable data symbols as findings, and how many were left out, by reason."""
    where = {
        os.path.normcase(os.path.abspath(o)): (t, s) for t, objs in objects.items() for o, s in objs
    }
    findings: set[Finding] = set()
    skipped: collections.Counter = collections.Counter()
    for sym in symbols:
        kind = symbol_kind(sym)
        if kind is None:
            continue
        owner = where.get(os.path.normcase(os.path.abspath(sym.obj)))
        if owner is None:
            continue
        target, source = owner
        demangled = names.get(sym.name, sym.name)
        reason = left_out_reason(source, demangled)
        if reason is not None:
            skipped[reason] += 1
            continue
        if sym.cls in _VAGUE:
            source = HEADER
        findings.add(Finding(target=target, source=source, kind=kind, symbol=demangled))
    return findings, skipped


# ---------------------------------------------------------------------------------------------
# Allowlist


def parse_allowlist(text: str) -> tuple[list[Entry], list[str]]:
    """Entries, and errors for lines that are not ``target source kind symbol``."""
    entries: list[Entry] = []
    errors: list[str] = []
    seen: dict[tuple[str, str, str], int] = {}
    for n, raw in enumerate(text.splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split(None, 3)
        if len(parts) != 4:
            errors.append(f"line {n}: expected 'target source kind symbol': {raw!r}")
            continue
        target, source, kind, symbol = parts
        if kind not in KINDS:
            errors.append(f"line {n}: kind {kind!r} is not one of {', '.join(KINDS)}")
            continue
        e = Entry(target=target, source=source, kind=kind, symbol=symbol.strip(), lineno=n)
        if e.key in seen:
            errors.append(f"line {n}: repeats line {seen[e.key]}")
            continue
        seen[e.key] = n
        entries.append(e)
    return entries, errors


ALLOWLIST_HEADER = """\
# The writable-data scan's allowlist (tools/singleton_scan/README.md).
#
# One process-wide object of libcvc's own per line: <target> <source> <kind> <symbol>. <source>
# is the file the object is defined in, or <header> for an inline function's or a template's
# static, which any object including its header may carry. <kind> is data, bss, tls, guard or
# common. Third-party libraries' statics are left out (scan.py's THIRD_PARTY) and only counted.
#
# libcvc keeps no process-wide state: it lives on cvc::app or in an object the caller owns. Each
# line here is one still to move, and the list only shrinks to empty. Never add a line: CI's
# format-check job fails a change that does (ratchet.py). The scan fails on a line this build no
# longer has, so delete a line when its object moves.
"""


def render_allowlist(lines_by_target: dict[str, list[str]]) -> str:
    out = [ALLOWLIST_HEADER]
    for target in sorted(lines_by_target):
        lines = sorted(set(lines_by_target[target]), key=_sort_key)
        if not lines:
            continue
        out.append(f"\n# --- {target} ({len(lines)})\n")
        out.extend(f"{line}\n" for line in lines)
    return "".join(out)


def _sort_key(line: str) -> tuple[int, str, str]:
    target, source, _kind, symbol = line.split(None, 3)
    return (1 if source == HEADER else 0, source, symbol)


# ---------------------------------------------------------------------------------------------
# Report


def summary_table(findings: Iterable[Finding]) -> str:
    rows: dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
    for f in findings:
        rows[subsystem(f.target, f.source)][f.kind] += 1
    lines = [
        "| Subsystem | objects | guards | thread-locals |",
        "|---|---|---|---|",
    ]
    total: collections.Counter = collections.Counter()
    for name in sorted(rows, key=lambda r: (-sum(rows[r].values()), r)):
        c = rows[name]
        objects = c["data"] + c["bss"] + c["common"]
        lines.append(f"| {name} | {objects} | {c['guard']} | {c['tls']} |")
        total.update(c)
    objects = total["data"] + total["bss"] + total["common"]
    lines.append(f"| **total** | **{objects}** | **{total['guard']}** | **{total['tls']}** |")
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------


def main(
    argv: Sequence[str] | None = None,
    runner: Callable[[str, Sequence[str]], str] = run_nm,
    demangler: Callable[[str, Sequence[str]], dict[str, str]] = demangle,
) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build-dir", required=True, help="the CMake build tree")
    ap.add_argument(
        "--targets",
        required=True,
        help="comma-separated CMake targets whose objects are scanned; each must have objects",
    )
    ap.add_argument(
        "--allowlist",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "allowlist.txt"),
    )
    ap.add_argument("--nm", default="nm")
    ap.add_argument("--cxxfilt", default="c++filt")
    ap.add_argument(
        "--write-allowlist",
        metavar="PATH",
        help="write the allowlist this build needs (other targets' lines kept) and exit 0",
    )
    ap.add_argument(
        "--emit",
        metavar="PATH",
        help="also write the allowlist this build needs here (for CI artifacts), and still check",
    )
    ap.add_argument("--report", metavar="PATH", help="also write the summary as Markdown here")
    args = ap.parse_args(argv)

    targets = [t for t in (s.strip() for s in args.targets.split(",")) if t]
    objects = find_objects(args.build_dir, targets)
    missing = [t for t in targets if not objects[t]]
    if missing:
        print(
            f"error: no objects for {', '.join(missing)} under {args.build_dir}"
            " (renamed, or not built in this lane?)",
            file=sys.stderr,
        )
        return 2

    all_objs = [o for objs in objects.values() for o, _ in objs]
    symbols = scan_symbols(args.nm, all_objs, runner=runner)
    candidates = sorted({s.name for s in symbols if symbol_kind(s) is not None})
    names = demangler(args.cxxfilt, candidates)
    findings, skipped = collect(objects, symbols, names)

    allow_text = ""
    if os.path.exists(args.allowlist):
        with open(args.allowlist, encoding="utf-8") as fh:
            allow_text = fh.read()
    entries, errors = parse_allowlist(allow_text)
    if errors:
        for e in errors:
            print(f"{args.allowlist}: {e}", file=sys.stderr)
        return 2

    scanned = set(targets)
    allowed = {e.key: e for e in entries if e.target in scanned}
    found = {f.key: f for f in findings}
    new = sorted((found[k] for k in found.keys() - allowed.keys()), key=lambda f: f.line())
    stale = sorted((allowed[k] for k in allowed.keys() - found.keys()), key=lambda e: e.lineno)

    counts = ", ".join(f"{t} {len(objects[t])}" for t in targets)
    report = [
        f"## Writable-data scan: {len(all_objs)} objects ({counts})\n\n",
        summary_table(findings),
        "\n",
        f"{len(findings)} writable symbols, {len(allowed)} allowlist entries for these targets.\n",
    ]
    if skipped:
        report.append("\nLeft out (toolchain artifacts, generated code, third-party libraries):\n")
        report.extend(f"- {n} x {reason}\n" for reason, n in sorted(skipped.items()))
    print("".join(report))
    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as fh:
            fh.write("".join(report))

    for path in (args.emit, args.write_allowlist):
        if not path:
            continue
        by_target: dict[str, list[str]] = collections.defaultdict(list)
        for e in entries:
            if e.target not in scanned:
                by_target[e.target].append(e.line())
        for f in findings:
            by_target[f.target].append(f.line())
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(render_allowlist(by_target))
        print(f"wrote {path}")
    if args.write_allowlist:
        return 0

    if new:
        print(f"\n{len(new)} writable symbols are not on the allowlist ({args.allowlist}):")
        for f in new:
            print(f"  + {f.line()}")
        print(
            "\nEach is process-wide state, which libcvc does not keep. Put it on the app, or in an"
            " object the caller owns (VOLROVER-LIBRARY-PORTS-ROADMAP.md 1.3). Do not add it to the"
            " allowlist: the list only shrinks, and CI fails a change that adds a line."
        )
    if stale:
        print(f"\n{len(stale)} allowlist entries this build no longer has; delete them:")
        for e in stale:
            print(f"  - line {e.lineno}: {e.line()}")
    if new or stale:
        return 1
    print("\nok: every writable symbol is on the allowlist, and every entry is still needed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
