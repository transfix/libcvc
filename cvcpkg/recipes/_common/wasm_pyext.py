#!/usr/bin/env python3
"""recipes/_common/wasm_pyext.py -- one namespace per Python C extension built for wasm.

Natively each extension is a shared library loaded RTLD_LOCAL: its PyInit function and its
internals are private to it. Linked statically into one wasm module, all extensions share one
namespace, so `PyInit_<leaf>` collides across packages (`a._common` and `b._common` both define
`PyInit__common`) and copies of one helper library collide too. numpy's extensions each carry
npymath and the random distributions, and two of them define seven of those functions with
different signatures, so a link that merges them binds calls to a trapping stub.

An extension built by emcc without SIDE_MODULE is one relocatable wasm object whose symbol table
lives in the `linking` custom section (WebAssembly tool-conventions, Linking.md, version 2).
This tool rewrites that table:

* `PyInit_<leaf>` becomes `PyInit__x_<n1><part1><n2><part2>...`, each part of the dotted module
  name with its length in front, as C++ mangling does: `numpy._core._simd` becomes
  `PyInit__x_5numpy5_core5_simd`. Distinct module names give distinct symbols.
* Every other defined global symbol becomes local (private to the object), with three kinds
  left global:
  - weak C++ RTTI (`_ZTI*` type_info objects, `_ZTS*` names). Under the Itanium ABI these must be
    one object program-wide: libc++abi matches a catch clause by comparing type_info pointers, so
    a private copy would stop an extension catching what libc++ throws (std::bad_alloc from
    operator new). Their duplicates are expected, and `check` accepts them;
  - what the object asks to keep (WASM_SYM_EXPORTED, WASM_SYM_NO_STRIP), reported;
  - nothing else.
* A COMDAT group with a symbol made local gets a per-extension prefix, so C++ inline and template
  code stays private. A group with none (RTTI's) keeps its name, so the linker merges it as usual.
* Function names in the `name` section get the same prefix, so a combined `wasm-ld -r` link and
  stack traces stay unambiguous.

References inside the object go by symbol index, so they survive. Undefined symbols (what the
extension takes from libpython and libc) are untouched. llvm-objcopy can't do this: it refuses
every symbol option on wasm objects. Common symbols (binding 3) are refused: they don't occur with
emsdk's clang, which rejects -fcommon for wasm.

A package's extensions are listed in `share/python-wasm/<name>.inittab`, one tab-separated line
per extension: the dotted module name, its renamed init symbol, and the object's path relative to
the prefix that holds the manifest. A host link registers each with
`PyImport_AppendInittab(dotted, symbol)` and links the objects.

Subcommands (stdlib only; run it with the build's native Python):

  object SRC DST DOTTED       localize one extension object
  package PREFIX PACKAGE      localize a package's extensions in place and write its manifest
          [--site-packages DIR] [--manifest-name NAME] [--out DIR] [--keep-tests]
                              --out writes localized copies, and their manifest, under DIR
                              instead (a prefix overlay), leaving PREFIX untouched
  check MANIFEST...           fail unless the manifests' extensions can link together: unique
                              module names, and no global defined by two extensions (weak RTTI
                              aside)
  registrar --function NAME --cpp OUT --objects OUT MANIFEST...
                              write the C++ registrar (extern "C" int NAME(void), 0 = success)
                              and the list of objects to link, one per line
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path

MANIFEST_DIR = "share/python-wasm"
MANIFEST_HEADER = (
    "# python-wasm inittab v1: dotted-module<TAB>init-symbol<TAB>object (relative to the prefix)"
)

# tool-conventions/Linking.md
_LINKING_VERSION = 2
_WASM_COMDAT_INFO, _WASM_SYMBOL_TABLE = 7, 8
_FUNCTION, _DATA, _GLOBAL, _SECTION, _TAG, _TABLE = 0, 1, 2, 3, 4, 5
_BINDING_MASK = 0x3
_BINDING_GLOBAL, _BINDING_WEAK, _BINDING_LOCAL, _BINDING_COMMON = 0, 1, 2, 3
_UNDEFINED = 0x10
_EXPORTED = 0x20
_EXPLICIT_NAME = 0x40
_NO_STRIP = 0x80
_KEEP = _EXPORTED | _NO_STRIP
# COMDAT member kinds, and the symbol kind each names by index
_COMDAT_KINDS = {0: _DATA, 1: _FUNCTION, 2: _GLOBAL, 3: _TAG, 4: _TABLE}

_NAME_FUNCTIONS = 1  # the `name` section's function-names subsection
_RTTI_PREFIXES = (b"_ZTI", b"_ZTS")

_DOTTED = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(\.[A-Za-z_][A-Za-z0-9_]*)*$")
_EXT_SUFFIX = re.compile(r"(\.cpython-[^.]*|\.abi3)?\.so$")


class LocalizeError(Exception):
    """The input is not an extension object (or manifest) this tool can handle."""


@dataclass
class Report:
    dotted: str
    init_symbol: str
    localized: int = 0
    comdats: int = 0
    names: int = 0
    kept_rtti: int = 0
    kept_global: list | None = None
    already: bool = False


@dataclass
class Symbol:
    kind: int
    flags: int
    name: str | None

    @property
    def defined(self) -> bool:
        return not (self.flags & _UNDEFINED)

    @property
    def binding(self) -> int:
        return self.flags & _BINDING_MASK

    @property
    def local(self) -> bool:
        return self.binding == _BINDING_LOCAL

    @property
    def unifiable(self) -> bool:
        """Weak C++ RTTI: copies in several objects are meant to become one."""
        return (
            self.binding == _BINDING_WEAK
            and self.name is not None
            and self.name.encode().startswith(_RTTI_PREFIXES)
        )


def init_symbol_for(dotted: str) -> str:
    """The renamed init symbol for a dotted module name: each part with its length in front."""
    if not _DOTTED.match(dotted):
        raise LocalizeError(f"not a dotted Python module name: {dotted!r}")
    return "PyInit__x_" + "".join(f"{len(part)}{part}" for part in dotted.split("."))


# ---- binary reading ---------------------------------------------------------------------------


def _byte(buf: bytes, i: int) -> tuple:
    if i >= len(buf):
        raise LocalizeError("truncated table")
    return buf[i], i + 1


def _uleb(buf: bytes, i: int) -> tuple:
    result = shift = 0
    while True:
        if i >= len(buf):
            raise LocalizeError("truncated LEB128")
        byte = buf[i]
        i += 1
        result |= (byte & 0x7F) << shift
        shift += 7
        if byte < 0x80:
            return result, i
        if shift > 63:
            raise LocalizeError("LEB128 too long")


def _enc(value: int) -> bytes:
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        out.append(byte | (0x80 if value else 0))
        if not value:
            return bytes(out)


def _name(raw: bytes) -> bytes:
    return _enc(len(raw)) + raw


def _read_name(buf: bytes, i: int) -> tuple:
    n, i = _uleb(buf, i)
    if i + n > len(buf):
        raise LocalizeError("truncated name")
    return bytes(buf[i : i + n]), i + n


def _sections(data: bytes):
    """(id, body) for each section of a wasm module."""
    if data[:4] != b"\0asm":
        raise LocalizeError("not a WebAssembly module")
    if data[4:8] != b"\x01\x00\x00\x00":
        raise LocalizeError("unsupported WebAssembly binary version")
    i = 8
    while i < len(data):
        sid = data[i]
        size, j = _uleb(data, i + 1)
        body = data[j : j + size]
        if len(body) != size:
            raise LocalizeError("truncated section")
        i = j + size
        yield sid, body


def _subsections(body: bytes, i: int):
    """(id, payload) for each subsection of a custom section's payload, from offset i."""
    while i < len(body):
        sub = body[i]
        size, j = _uleb(body, i + 1)
        payload = body[j : j + size]
        if len(payload) != size:
            raise LocalizeError("truncated subsection")
        i = j + size
        yield sub, payload


def _parse_symtab(payload: bytes) -> list:
    """[(kind, flags, index_or_None, name_or_None, data_tail_or_None)] in table order."""
    count, i = _uleb(payload, 0)
    out = []
    for _ in range(count):
        kind, i = _byte(payload, i)
        flags, i = _uleb(payload, i)
        defined = not (flags & _UNDEFINED)
        index = name = tail = None
        if kind in (_FUNCTION, _GLOBAL, _TAG, _TABLE):
            index, i = _uleb(payload, i)
            if defined or (flags & _EXPLICIT_NAME):
                name, i = _read_name(payload, i)
        elif kind == _DATA:
            name, i = _read_name(payload, i)
            if defined and (flags & _BINDING_MASK) == _BINDING_COMMON:
                raise LocalizeError(f"common symbol {name.decode('utf-8', 'replace')}: unsupported")
            if defined:
                segment, i = _uleb(payload, i)
                offset, i = _uleb(payload, i)
                size, i = _uleb(payload, i)
                tail = (segment, offset, size)
        elif kind == _SECTION:
            index, i = _uleb(payload, i)
        else:
            raise LocalizeError(f"unknown symbol kind {kind}")
        out.append((kind, flags, index, name, tail))
    if i != len(payload):
        raise LocalizeError("symbol table has trailing bytes")
    return out


def _encode_symtab(entries: list) -> bytes:
    out = bytearray(_enc(len(entries)))
    for kind, flags, index, name, tail in entries:
        out += bytes([kind]) + _enc(flags)
        if kind in (_FUNCTION, _GLOBAL, _TAG, _TABLE):
            out += _enc(index)
            if name is not None:
                out += _name(name)
        elif kind == _DATA:
            out += _name(name)
            if tail is not None:
                out += b"".join(_enc(v) for v in tail)
        else:
            out += _enc(index)
    return bytes(out)


def _linking_body(data: bytes):
    for sid, body in _sections(data):
        if sid == 0:
            name, k = _read_name(body, 0)
            if name == b"linking":
                return body, k
    raise LocalizeError("no linking section: not a relocatable object")


def symbols(data: bytes) -> list:
    """The symbol table of a relocatable wasm object."""
    body, k = _linking_body(data)
    _, k = _uleb(body, k)
    for sub, payload in _subsections(body, k):
        if sub == _WASM_SYMBOL_TABLE:
            return [
                Symbol(kind, flags, None if name is None else name.decode("utf-8", "replace"))
                for kind, flags, _, name, _ in _parse_symtab(payload)
            ]
    raise LocalizeError("linking section has no symbol table")


def defined_globals(data: bytes, clashing_only: bool = False) -> list:
    """The names an object defines globally. With `clashing_only`, leave out weak RTTI, whose
    copies are meant to become one; what is left would clash with another object's."""
    return [
        s.name
        for s in symbols(data)
        if s.defined
        and not s.local
        and s.name
        and s.kind != _SECTION
        and not (clashing_only and s.unifiable)
    ]


# ---- the rewrite ------------------------------------------------------------------------------


def _rewrite_symtab(payload: bytes, leaf_init: bytes, new_init: bytes, rep: Report) -> tuple:
    """The new symbol table, and {symbol kind: indices of what it made local} for the COMDATs."""
    entries = _parse_symtab(payload)
    renamed = 0
    made_local = {kind: set() for kind in _COMDAT_KINDS.values()}
    out = []
    for kind, flags, index, name, tail in entries:
        binding = flags & _BINDING_MASK
        if kind != _SECTION and not (flags & _UNDEFINED) and binding != _BINDING_LOCAL:
            if kind != _DATA and name == leaf_init:
                name = new_init
                flags &= ~_BINDING_MASK  # a strong global
                renamed += 1
            elif binding == _BINDING_WEAK and name.startswith(_RTTI_PREFIXES):
                rep.kept_rtti += 1
            elif flags & _KEEP:
                rep.kept_global = (rep.kept_global or []) + [name.decode("utf-8", "replace")]
            else:
                flags = (flags & ~_BINDING_MASK) | _BINDING_LOCAL
                made_local[kind].add(tail[0] if kind == _DATA else index)
                rep.localized += 1
        out.append((kind, flags, index, name, tail))
    if renamed != 1:
        what = "no" if renamed == 0 else str(renamed)
        raise LocalizeError(f"{what} global function {leaf_init.decode()} in the symbol table")
    return _encode_symtab(out), made_local


def _rewrite_comdats(payload: bytes, tag: bytes, made_local: dict, rep: Report) -> bytes:
    count, i = _uleb(payload, 0)
    out = bytearray(_enc(count))
    for _ in range(count):
        name, i = _read_name(payload, i)
        flags, i = _uleb(payload, i)
        nsyms, i = _uleb(payload, i)
        members = []
        for _ in range(nsyms):
            kind, i = _uleb(payload, i)
            index, i = _uleb(payload, i)
            members.append((kind, index))
        private = any(
            index in made_local.get(_COMDAT_KINDS.get(kind, -1), ()) for kind, index in members
        )
        if private:
            name = tag + name
            rep.comdats += 1
        out += _name(name) + _enc(flags) + _enc(nsyms)
        out += b"".join(_enc(kind) + _enc(index) for kind, index in members)
    if i != len(payload):
        raise LocalizeError("COMDAT table has trailing bytes")
    return bytes(out)


def _rewrite_linking(body: bytes, start: int, leaf_init, new_init, tag, rep) -> bytes:
    version, i = _uleb(body, start)
    if version != _LINKING_VERSION:
        raise LocalizeError(f"linking section version {version}, expected {_LINKING_VERSION}")
    subs = list(_subsections(body, i))
    made_local = None
    for n, (sub, payload) in enumerate(subs):
        if sub == _WASM_SYMBOL_TABLE:
            new, made_local = _rewrite_symtab(payload, leaf_init, new_init, rep)
            subs[n] = (sub, new)
    if made_local is None:
        raise LocalizeError("linking section has no symbol table")
    out = bytearray(body[:start]) + _enc(version)
    for sub, payload in subs:
        if sub == _WASM_COMDAT_INFO:
            payload = _rewrite_comdats(payload, tag, made_local, rep)
        out += bytes([sub]) + _enc(len(payload)) + payload
    return bytes(out)


def _rewrite_names(body: bytes, start: int, tag: bytes, rep: Report) -> bytes:
    """Prefix the `name` section's function names; other subsections are copied as they are."""
    out = bytearray(body[:start])
    count_names = 0
    for sub, payload in _subsections(body, start):
        if sub == _NAME_FUNCTIONS:
            count, k = _uleb(payload, 0)
            new = bytearray(_enc(count))
            for _ in range(count):
                index, k = _uleb(payload, k)
                name, k = _read_name(payload, k)
                new += _enc(index) + _name(tag + name)
                count_names += 1
            if k != len(payload):
                raise LocalizeError("function names have trailing bytes")
            payload = bytes(new)
        out += bytes([sub]) + _enc(len(payload)) + payload
    rep.names += count_names
    return bytes(out)


def localize_bytes(data: bytes, dotted: str) -> tuple:
    """Rewrite one extension object given as bytes; returns (new bytes, Report)."""
    new_init = init_symbol_for(dotted)
    leaf_init = ("PyInit_" + dotted.rsplit(".", 1)[-1]).encode()
    tag = (dotted + "::").encode()
    rep = Report(dotted=dotted, init_symbol=new_init)
    out = bytearray(data[:8])
    linked = False
    for sid, body in _sections(data):
        if sid == 0:
            name, k = _read_name(body, 0)
            if name in (b"dylink.0", b"dylink"):
                raise LocalizeError(
                    "a side module (dylink): localize the relocatable build instead"
                )
            if name == b"linking":
                body = _rewrite_linking(body, k, leaf_init, new_init.encode(), tag, rep)
                linked = True
            elif name == b"name":
                try:
                    body = _rewrite_names(body, k, tag, rep)
                except LocalizeError:
                    pass  # debug names only: an unreadable one is kept as it is
        out += bytes([sid]) + _enc(len(body)) + body
    if not linked:
        raise LocalizeError("no linking section: not a relocatable object")
    return bytes(out), rep


def localize_object(src: Path, dst: Path, dotted: str) -> Report:
    """Localize one object. An object already localized for `dotted` is copied as it is."""
    data = Path(src).read_bytes()
    symbol = init_symbol_for(dotted)
    if symbol in defined_globals(data):
        new, rep = data, Report(dotted=dotted, init_symbol=symbol, already=True)
    else:
        new, rep = localize_bytes(data, dotted)
    dst = Path(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_name(dst.name + ".tmp")
    tmp.write_bytes(new)
    os.replace(tmp, dst)
    return rep


# ---- packages and manifests -------------------------------------------------------------------


def find_site_packages(prefix: Path) -> Path:
    for pattern in ("lib/python3*/site-packages", "Lib/site-packages"):
        for candidate in sorted(Path(prefix).glob(pattern)):
            if candidate.is_dir():
                return candidate
    raise LocalizeError(f"no site-packages under {prefix}")


def extension_objects(site: Path, package: str, keep_tests: bool = False) -> list:
    """(dotted, path) for each extension of `package` under `site`, in a stable order."""
    found = []
    root = Path(site, *package.split("."))
    if root.is_dir():
        for path in sorted(root.rglob("*.so")):
            rel = path.relative_to(site)
            leaf = _EXT_SUFFIX.sub("", rel.name)
            if not keep_tests and (leaf.endswith("_tests") or "tests" in rel.parent.parts):
                continue
            found.append((".".join(list(rel.parent.parts) + [leaf]), path))
    parent = root.parent
    single = sorted(parent.glob(root.name + ".cpython-*.so"))
    single += [parent / (root.name + ".abi3.so"), parent / (root.name + ".so")]
    for path in single:
        if path.is_file():
            found.append((package, path))
    return found


def localize_package(
    prefix,
    package: str,
    site=None,
    keep_tests: bool = False,
    manifest_name: str | None = None,
    out=None,
) -> tuple:
    """Localize every extension of `package` and write its manifest; returns (manifest, reports).

    In place by default. With `out`, the localized copies go under `out` at the same paths
    relative to it as the originals have to `prefix`, and the manifest goes there too."""
    prefix = Path(prefix)
    site = Path(site) if site else find_site_packages(prefix)
    dest_root = Path(out) if out else prefix
    objects = extension_objects(site, package, keep_tests)
    if not objects:
        raise LocalizeError(f"no extension objects for {package!r} under {site}")
    reports, lines, seen = [], [MANIFEST_HEADER], {}
    for dotted, path in objects:
        symbol = init_symbol_for(dotted)
        if symbol in seen:
            raise LocalizeError(f"{dotted} and {seen[symbol]} map to one init symbol {symbol}")
        seen[symbol] = dotted
        resolved = path.resolve()
        if not resolved.is_relative_to(prefix.resolve()):
            raise LocalizeError(f"{path} is not under the prefix {prefix}")
        rel = resolved.relative_to(prefix.resolve())
        reports.append(localize_object(path, dest_root / rel, dotted))
        lines.append(f"{dotted}\t{symbol}\t{rel.as_posix()}")
    manifest = dest_root / MANIFEST_DIR / f"{manifest_name or package}.inittab"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    with open(manifest, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return manifest, reports


def read_manifest(path) -> list:
    """[(dotted, symbol, absolute object path)] for each line of a manifest."""
    path = Path(path)
    prefix = path.resolve().parent.parent.parent  # <prefix>/share/python-wasm/<name>.inittab
    entries = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        parts = line.split("\t")
        if len(parts) != 3 or not all(parts):
            raise LocalizeError(f"{path}: bad manifest line {line!r}")
        dotted, symbol, rel = parts
        if init_symbol_for(dotted) != symbol:
            raise LocalizeError(f"{path}: {dotted} should map to {init_symbol_for(dotted)}")
        entries.append((dotted, symbol, prefix / rel))
    return entries


def check(manifests) -> list:
    """Problems that would stop the manifests' extensions linking together; [] when none.

    A global defined by two extensions is a problem even when weak (an inline function kept
    global because it is exported or `used`): the two packages' copies may differ, and the
    linker would keep one. Weak RTTI is the exception; its copies are meant to become one."""
    problems, modules, owners = [], {}, {}
    for manifest in manifests:
        for dotted, symbol, obj in read_manifest(manifest):
            if dotted in modules:
                problems.append(f"{dotted}: listed by {modules[dotted]} and {manifest}")
                continue
            modules[dotted] = manifest
            if not obj.is_file():
                problems.append(f"{dotted}: missing object {obj}")
                continue
            data = obj.read_bytes()
            if symbol not in defined_globals(data):
                problems.append(f"{dotted}: {obj.name} doesn't define {symbol} (not localized?)")
            for name in defined_globals(data, clashing_only=True):
                if name in owners:
                    problems.append(f"{name}: defined by {owners[name]} and {dotted}")
                else:
                    owners[name] = dotted
    return problems


def write_registrar(manifests, function: str, cpp, objects) -> int:
    """Write the registrar source and the object list; returns how many extensions."""
    entries = [e for m in manifests for e in read_manifest(m)]
    lines = ["// Generated by wasm_pyext.py registrar: do not edit.", "#include <Python.h>", ""]
    lines.append('extern "C" {')
    lines += [f"PyObject *{symbol}(void);" for _, symbol, _ in entries]
    lines.append(f"int {function}(void) {{")
    for dotted, symbol, _ in entries:
        lines.append(f'  if (PyImport_AppendInittab("{dotted}", {symbol}) != 0) return 1;')
    lines += ["  return 0;", "}", '} // extern "C"', ""]
    Path(cpp).write_text("\n".join(lines), encoding="utf-8", newline="\n")
    Path(objects).write_text(
        "".join(f"{obj.as_posix()}\n" for _, _, obj in entries), encoding="utf-8", newline="\n"
    )
    return len(entries)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="wasm_pyext.py", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    one = sub.add_parser("object", help="localize one extension object")
    one.add_argument("src")
    one.add_argument("dst")
    one.add_argument("dotted")
    pkg = sub.add_parser("package", help="localize a package's extensions; write its manifest")
    pkg.add_argument("prefix")
    pkg.add_argument("package")
    pkg.add_argument("--site-packages")
    pkg.add_argument("--manifest-name")
    pkg.add_argument("--out")
    pkg.add_argument("--keep-tests", action="store_true")
    chk = sub.add_parser("check", help="fail unless the manifests' extensions link together")
    chk.add_argument("manifests", nargs="+")
    reg = sub.add_parser("registrar", help="write the C++ registrar and the object list")
    reg.add_argument("--function", required=True)
    reg.add_argument("--cpp", required=True)
    reg.add_argument("--objects", required=True)
    reg.add_argument("manifests", nargs="+")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "check":
            problems = check(args.manifests)
            for p in problems:
                print(f"wasm_pyext: {p}", file=sys.stderr)
            return 1 if problems else 0
        if args.cmd == "registrar":
            n = write_registrar(args.manifests, args.function, args.cpp, args.objects)
            print(f"wasm_pyext: registrar for {n} extensions -> {args.cpp}")
            return 0
        if args.cmd == "object":
            reports = [localize_object(Path(args.src), Path(args.dst), args.dotted)]
        else:
            manifest, reports = localize_package(
                args.prefix,
                args.package,
                args.site_packages,
                args.keep_tests,
                args.manifest_name,
                args.out,
            )
            print(f"wasm_pyext: {manifest}")
    except (LocalizeError, OSError) as e:
        print(f"wasm_pyext: {e}", file=sys.stderr)
        return 1
    for r in reports:
        if r.already:
            print(f"{r.dotted}\t{r.init_symbol}\talready localized")
            continue
        kept = f"; kept global: {', '.join(r.kept_global)}" if r.kept_global else ""
        print(
            f"{r.dotted}\t{r.init_symbol}\t{r.localized} localized, {r.comdats} comdats, "
            f"{r.kept_rtti} RTTI kept{kept}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
