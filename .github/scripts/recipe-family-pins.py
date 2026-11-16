#!/usr/bin/env python3
"""Fail when a dependency on one of libcvc's own packages does not pin libcvc's major version.

Every package built from this tree (ari*, arisrv*, arigl*, pycvc*, cvcgl*, cvc-cli*, ...) links
the libcvc of its own release, and libcvc's SONAME carries the major version (libcvc.so.4, with
libcvcGL.so.4 beside it). `cvcpkg install` resolves an unversioned dependency to the newest
provider, so a package from one major can land beside the next major's libcvc and not load:
arisrvx 3.7.13 took libcvc-cuda 4.0.0 (libcvc.so.3 => not found) once 4.0.0 reached the catalog.

So every `depends:` entry, build or runtime, that names a recipe under cvcpkg/recipes/ or a name
one of them provides (libcvc-cuda provides libcvc) must carry `version: "^<major>.0"`, where
<major> is the major of CMakeLists.txt's project VERSION. cvcpkg's resolver tests the range
against whichever recipe provides the name and ignores +cvc.N, so ^4.0 takes any 4.x and nothing
else. A major bump moves these pins along with upstream_version; a minor one leaves them alone.

Usage: recipe-family-pins.py [repo-root]   (needs PyYAML)
"""

import re
import sys
from pathlib import Path

import yaml

RECIPES = "cvcpkg/recipes"


def project_major(root):
    """The major of CMakeLists.txt's VERSION, read as the recipe-version check reads it."""
    for line in (root / "CMakeLists.txt").read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*VERSION\s+(\d+)\.", line)
        if m:
            return int(m.group(1))
    return None


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2]
    major = project_major(root)
    if major is None:
        print("::error::could not read VERSION from CMakeLists.txt")
        return 1
    want = f"^{major}.0"

    recipes = {}
    for path in sorted((root / RECIPES).glob("*/recipe.yaml")):
        with open(path, encoding="utf-8") as f:
            recipes[path.parent.name] = yaml.safe_load(f)
    ours = set(recipes)
    for recipe in recipes.values():
        ours |= set(recipe.get("provides") or [])

    bad = 0
    pinned = 0
    for name, recipe in sorted(recipes.items()):
        path = f"{RECIPES}/{name}/recipe.yaml"
        depends = recipe.get("depends") or {}
        for kind in ("build", "runtime"):
            for dep in depends.get(kind) or []:
                dep_name = dep if isinstance(dep, str) else dep.get("name", "")
                dep_name = dep_name.rsplit("/", 1)[-1]  # "org/name" names the same package
                if dep_name not in ours:
                    continue
                got = None if isinstance(dep, str) else dep.get("version")
                if got != want:
                    print(f"::error file={path}::{kind} dependency {dep_name} pins "
                          f"{got!r}, not version: \"{want}\"")
                    bad += 1
                else:
                    pinned += 1

    if bad:
        print(f"::error::{bad} dependencies on libcvc's own packages do not pin {want}. Each "
              f"`- name:` of one of them under depends: needs `version: \"{want}\"`, the major "
              f"of CMakeLists.txt's VERSION, so cvcpkg never installs it beside another major's "
              f"libcvc (.github/scripts/recipe-family-pins.py says why).")
        return 1
    print(f"all {pinned} dependencies on libcvc's own packages pin {want}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
