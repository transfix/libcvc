#!/usr/bin/env python3
"""make_python_zip.py -- the pure-Python side of the browser's scripting host, as one zip.

ariglx-web (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §5.1) links every C extension in, so what
Python still reads from files is pure Python: the stdlib, pycvc.py, the pycvc_gl package and the
.py files of the packages in the deps prefix. The page fetches this zip and writes it to
/py/lib/python312.zip, which CPython puts on sys.path at start, so all of it imports through
zipimport.

    make_python_zip.py --out python312.zip --stdlib <prefix>/lib/python3.12
        [--site-packages <prefix>/lib/python3.12/site-packages] [--exclude NAME]...
        [--add FILE_OR_PACKAGE_DIR]... [--merge-zip ZIP]...

Left out: the stdlib's tests and the parts a browser can't run (tkinter, idlelib, turtledemo,
ensurepip, venv, lib2to3, the config-* build directory), every __pycache__, test and tests
directory, and anything that isn't Python source or package data (objects, archives, headers,
C sources, shared libraries, bytecode). The entries are stored, not deflated, in sorted order
with a fixed time: zipimport reads a stored zip without zlib, the server compresses it on the
wire, and the same inputs give the same bytes.
"""

import argparse
import sys
import zipfile
from pathlib import Path

# Top-level stdlib entries a browser host has no use for, or that only build or test CPython.
STDLIB_SKIP = {
    "site-packages", "test", "idlelib", "tkinter", "turtledemo", "ensurepip", "venv", "lib2to3",
    "lib-dynload", "turtle.py", "__phello__", "__hello__.py",
}
DIR_SKIP = {"__pycache__", "test", "tests", "idle_test"}
SUFFIX_SKIP = {
    ".pyc", ".pyo", ".o", ".a", ".so", ".dll", ".pyd", ".lib", ".exe", ".h", ".c", ".cpp",
    ".pxd", ".pyx", ".pyi", ".wasm", ".js",
}
FIXED_TIME = (1980, 1, 1, 0, 0, 0)


def wanted(rel: Path) -> bool:
    if any(part in DIR_SKIP for part in rel.parts[:-1]):
        return False
    return rel.suffix.lower() not in SUFFIX_SKIP


def add_tree(entries: dict, root: Path, arc_prefix: str, skip_top=frozenset()) -> None:
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(root)
        if rel.parts[0] in skip_top or rel.parts[0].startswith("config-3"):
            continue
        if wanted(rel):
            entries[f"{arc_prefix}{rel.as_posix()}"] = path.read_bytes()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--stdlib", required=True, help="<prefix>/lib/python3.12")
    ap.add_argument("--site-packages", help="its packages (.py and package data only)")
    ap.add_argument("--exclude", action="append", default=[],
                    help="a site-packages entry to leave out (repeatable)")
    ap.add_argument("--add", action="append", default=[],
                    help="a module file or package directory, at the zip's root")
    ap.add_argument("--merge-zip", action="append", default=[],
                    help="another zip whose entries go in at the root (vtk-python's _vtk.zip)")
    args = ap.parse_args(argv)

    entries: dict[str, bytes] = {}
    stdlib = Path(args.stdlib)
    if not (stdlib / "os.py").is_file():
        print(f"make_python_zip: {stdlib} is not a Python stdlib (no os.py)", file=sys.stderr)
        return 1
    add_tree(entries, stdlib, "", skip_top=frozenset(STDLIB_SKIP))
    if args.site_packages:
        site = Path(args.site_packages)
        for entry in sorted(site.iterdir()):
            if entry.name in args.exclude or entry.name in DIR_SKIP:
                continue
            if entry.is_dir():
                add_tree(entries, entry, f"{entry.name}/")
            elif wanted(Path(entry.name)) and entry.suffix != ".zip":
                entries[entry.name] = entry.read_bytes()
    for add in args.add:
        p = Path(add)
        if p.is_dir():
            add_tree(entries, p, f"{p.name}/")
        elif p.is_file():
            entries[p.name] = p.read_bytes()
        else:
            print(f"make_python_zip: {p} does not exist", file=sys.stderr)
            return 1
    for merge in args.merge_zip:
        with zipfile.ZipFile(merge) as z:
            for info in z.infolist():
                if not info.is_dir() and wanted(Path(info.filename)):
                    entries[info.filename] = z.read(info)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(out.suffix + ".tmp")
    with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_STORED) as z:
        for name in sorted(entries):
            info = zipfile.ZipInfo(name, date_time=FIXED_TIME)
            info.external_attr = 0o644 << 16
            z.writestr(info, entries[name])
    tmp.replace(out)
    total = sum(len(b) for b in entries.values())
    print(f"make_python_zip: {out.name}: {len(entries)} files, {total / 1048576:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main())
