#!/usr/bin/env python3
"""Bump libcvc to a new release: CMakeLists.txt, every cvcpkg recipe, and the family pins.

A release bump (transfix/libcvc#1299 for 4.0.1) moves three things together, and CI checks each:

* CMakeLists.txt's project VERSION, the source of truth;
* every cvcpkg/recipes/*/recipe.yaml's upstream_version, with cvc_revision back to 1 (it counts
  revisions OF an upstream version), which the recipe-version job checks;
* every dependency on one of libcvc's own packages, pinned `version: "==<VERSION>"`, which the
  same job checks with recipe-family-pins.py. The pins are why a half-published release cannot
  pair one release's bindings with another's libraries (recipe-family-pins.py says how).

Usage:
  python3 .github/scripts/bump-version.py X.Y.Z [repo-root]      (needs PyYAML)

It edits the files in place and changes nothing else; review `git diff`, commit, and open the
bump PR. Tag vX.Y.Z on the bump's merge commit once it is on master.
"""

import importlib.util
import re
import sys
from pathlib import Path

RECIPES = "cvcpkg/recipes"
HERE = Path(__file__).resolve().parent


def _pins_module():
    spec = importlib.util.spec_from_file_location("recipe_family_pins",
                                                  HERE / "recipe-family-pins.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read(path):
    with open(path, encoding="utf-8", newline="") as f:  # newline="": keep CRLF checkouts CRLF
        return f.read()


def _sub_file(path, pattern, repl, count=0, flags=re.MULTILINE):
    """re.sub over *path*'s text, keeping its line endings; return the number of changes."""
    text = _read(path)
    new, n = re.subn(pattern, repl, text, count=count, flags=flags)
    if new != text:
        path.write_text(new, encoding="utf-8", newline="")
    return n if new != text else 0


def main(argv):
    if not argv or not re.fullmatch(r"\d+\.\d+\.\d+", argv[0]):
        print(__doc__)
        return 2
    new = argv[0]
    root = Path(argv[1]) if len(argv) > 1 else HERE.parents[1]
    pins = _pins_module()
    old = pins.project_version(root)
    if old is None:
        print("could not read VERSION from CMakeLists.txt")
        return 1
    if old == new:
        print(f"CMakeLists.txt is already {new}; checking the recipes and pins only")

    # The first `VERSION X.Y.Z` line is project()'s (the recipe-version job reads the same one).
    _sub_file(root / "CMakeLists.txt", r"^(\s*VERSION\s+)\d+\.\d+\.\d+\b", rf"\g<1>{new}",
              count=1)

    bumped = 0
    for path in sorted((root / RECIPES).glob("*/recipe.yaml")):
        text = _read(path)
        m = re.search(r'^  upstream_version: "([^"]+)"', text, flags=re.MULTILINE)
        if not m or m.group(1) == new:
            continue
        prev = m.group(1)
        _sub_file(path, r'^(  upstream_version: )"[^"]+"', rf'\g<1>"{new}"', count=1)
        _sub_file(path, r"^(  cvc_revision: )\d+", r"\g<1>1", count=1)
        # The "Reset to 1 for the X upstream version" note, where it names the outgoing one.
        _sub_file(path, rf"(Reset to 1 for the ){re.escape(prev)}( upstream version)",
                  rf"\g<1>{new}\g<2>")
        bumped += 1
    print(f"CMakeLists.txt: {old} -> {new}; {bumped} recipe(s) moved to {new}, cvc_revision 1")

    # Pins last: they follow CMakeLists.txt's VERSION, which is now the new one.
    return pins.main(["--write", str(root)])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
