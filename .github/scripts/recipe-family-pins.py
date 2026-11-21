#!/usr/bin/env python3
"""Check (or write) that every dependency on one of libcvc's own packages pins this release.

Every package built from this tree (ari*, arisrv*, arigl*, pycvc*, cvcgl*, cvc-cli*, ...) links
the libcvc, cvcgl and pycvc of its own release, and the C++ ABI changes between patch releases,
not only between majors:

* arisrvx 3.7.13 took libcvc-cuda 4.0.0 and did not load (libcvc.so.3 => not found);
* while libcvc 4.0.1 was half-published (2026-11-19), cvcpkg paired pycvc-gl-cp313 4.0.1 with
  cvcgl 4.0.0 (`_pycvc_gl.so`: undefined symbol cvc::gl::RibbonNode::replaceFrom) and
  pycvc-cp311 4.0.0 with libcvc 4.0.1 (segfaults, garbage energies) in CVC-Lab/GRL-SNAM's CI.

`cvcpkg install` takes the newest provider a dependency's range allows, so a range wider than
one release lets the resolver mix releases whenever one is partly on the catalog. So every
`depends:` entry, build or runtime, that names a recipe under cvcpkg/recipes/ or a name one of
them provides (libcvc-cuda provides libcvc) must carry `version: "==<X.Y.Z>"`, where X.Y.Z is
CMakeLists.txt's project VERSION (which every recipe's upstream_version already equals: the
recipe-version CI job checks that).

cvcpkg reads `==X.Y.Z` with no `+cvc.N` as "this upstream version, any revision"
(cvcpkg/semver.py), so a `+cvc.N` rebuild of either side still satisfies it. Its resolver
backtracks, so while a release is half-published a consumer gets the newest release whose
packages are all on the catalog for its platform, instead of a mix.

Usage:
  recipe-family-pins.py [--write] [repo-root]      (needs PyYAML)

Without --write it checks and exits 1 on a missing or stale pin. With --write it rewrites each
such dependency's `version:` line in place (inserting one under `- name:` where there is none),
leaving every other line, comment and the key order alone. bump-version.py runs it on a release
bump.
"""

import re
import sys
from pathlib import Path

import yaml

RECIPES = "cvcpkg/recipes"


def project_version(root):
    """CMakeLists.txt's VERSION (X.Y.Z), read as the recipe-version check reads it."""
    for line in (root / "CMakeLists.txt").read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*VERSION\s+(\d+\.\d+\.\d+)\b", line)
        if m:
            return m.group(1)
    return None


def load_recipes(root):
    recipes = {}
    for path in sorted((root / RECIPES).glob("*/recipe.yaml")):
        with open(path, encoding="utf-8") as f:
            recipes[path.parent.name] = yaml.safe_load(f)
    return recipes


def family_names(recipes):
    """Recipe names under cvcpkg/recipes/ plus every name one of them provides."""
    ours = set(recipes)
    for recipe in recipes.values():
        ours |= set(recipe.get("provides") or [])
    return ours


def _dep_name(dep):
    name = dep if isinstance(dep, str) else dep.get("name", "")
    return name.rsplit("/", 1)[-1]  # "org/name" names the same package


def check(root, want):
    recipes = load_recipes(root)
    ours = family_names(recipes)
    bad = 0
    pinned = 0
    for name, recipe in sorted(recipes.items()):
        path = f"{RECIPES}/{name}/recipe.yaml"
        depends = recipe.get("depends") or {}
        for kind in ("build", "runtime"):
            for dep in depends.get(kind) or []:
                dep_name = _dep_name(dep)
                if dep_name not in ours:
                    continue
                got = None if isinstance(dep, str) else dep.get("version")
                if got != want:
                    print(f"::error file={path}::{kind} dependency {dep_name} pins "
                          f"{got!r}, not version: \"{want}\"")
                    bad += 1
                else:
                    pinned += 1
    return bad, pinned


_DEPENDS_RE = re.compile(r"^depends:\s*(#.*)?$")
_KEY_RE = re.compile(r"^(\s+)([A-Za-z_][\w-]*):")
_NAME_RE = re.compile(r"^(\s*)- name:\s*['\"]?([^'\"#\s]+)['\"]?\s*(#.*)?$")
_VERSION_RE = re.compile(r"^(\s+)version:.*$")


def write_pins(path, ours, want):
    """Rewrite *path*'s pins on *ours* to *want*; return how many lines changed."""
    with open(path, encoding="utf-8", newline="") as f:  # newline="": keep CRLF checkouts CRLF
        text = f.read()
    newline = "\r\n" if "\r\n" in text else "\n"
    lines = text.split(newline)
    out = []
    changed = 0
    in_depends = False
    kind_indent = None  # indentation of depends:'s own keys (build:, runtime:, host_tools:)
    in_kind = False
    i = 0
    while i < len(lines):
        line = lines[i]
        out.append(line)
        i += 1
        if line and not line[0].isspace() and not line.startswith("#"):
            in_depends = bool(_DEPENDS_RE.match(line))
            kind_indent = None
            in_kind = False
            continue
        if not in_depends:
            continue
        m = _KEY_RE.match(line)
        if m and (kind_indent is None or m.group(1) == kind_indent):
            kind_indent = m.group(1)
            in_kind = m.group(2) in ("build", "runtime")
            continue
        if not in_kind:
            continue
        m = _NAME_RE.match(line)
        if not m or m.group(2).rsplit("/", 1)[-1] not in ours:
            continue
        indent = m.group(1) + "  "
        pin = f'{indent}version: "{want}"'
        if i < len(lines) and _VERSION_RE.match(lines[i]) and \
                _VERSION_RE.match(lines[i]).group(1) == indent:
            if lines[i] != pin:
                changed += 1
            out.append(pin)
            i += 1
        else:
            out.append(pin)
            changed += 1
    if changed:
        path.write_text(newline.join(out), encoding="utf-8", newline="")
    return changed


def main(argv):
    args = [a for a in argv if a != "--write"]
    write = len(args) != len(argv)
    root = Path(args[0]) if args else Path(__file__).resolve().parents[2]
    version = project_version(root)
    if version is None:
        print("::error::could not read VERSION from CMakeLists.txt")
        return 1
    want = f"=={version}"

    if write:
        ours = family_names(load_recipes(root))
        total = 0
        for path in sorted((root / RECIPES).glob("*/recipe.yaml")):
            n = write_pins(path, ours, want)
            if n:
                print(f"{path.relative_to(root).as_posix()}: {n} pin(s) -> {want}")
            total += n
        print(f"wrote {total} pin(s)")

    bad, pinned = check(root, want)
    if bad:
        print(f"::error::{bad} dependencies on libcvc's own packages do not pin {want}. Each "
              f"`- name:` of one of them under depends: needs `version: \"{want}\"`, "
              f"CMakeLists.txt's VERSION, so cvcpkg never installs it beside another release's "
              f"libcvc, cvcgl or pycvc. `python3 .github/scripts/recipe-family-pins.py --write` "
              f"writes them (.github/scripts/recipe-family-pins.py says why).")
        return 1
    print(f"all {pinned} dependencies on libcvc's own packages pin {want}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
