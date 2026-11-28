#!/usr/bin/env python3
"""Copy cvcpkg/recipes with each vendored recipe's source pinned to a tarball.

libcvc's recipes build from this checkout (``source: {type: vendored, path: ../../..}``). A
cvcpkg.org builder has no checkout, and ``cvcpkg recipe push`` does not bundle a tree that holds
the recipes directory, so a pushed vendored recipe cannot build on the fleet. For the fleet's
columns (the BSDs, Haiku) publish-cvcpkg.yml pushes this copy instead, whose source is the
release tag's GitHub tarball and its sha256:

    pin-recipe-source.py --recipes cvcpkg/recipes --out pinned \\
        --url https://github.com/transfix/libcvc/archive/refs/tags/v4.0.10.tar.gz \\
        --sha256 <hex> [--revision libcvc=2 --revision ari=1 ...] \\
        [--revisions-json '${{ toJSON(needs.compute-revisions.outputs) }}']

Only the ``source:`` block and, with --revision, ``cvc_revision`` change; comments and the rest of
each recipe are kept as written. A recipe whose source is not vendored is copied unchanged.
"""

import argparse
import json
import re
import shutil
import sys
from pathlib import Path

SOURCE_BLOCK = re.compile(r"^source:\n(?:[ \t]+.*\n|[ \t]*\n)*", re.M)
VENDORED = re.compile(r"^\s+type:\s*vendored\s*$", re.M)
REVISION = re.compile(r"^(  cvc_revision:)\s*\d+(.*)$", re.M)

# compute-revisions' output keys that are not the recipe name with '-' for '_'.
OUTPUT_KEY_RECIPES = {"pycvc_cuda": "pycvc-cp311-cuda", "pycvc_gl_cuda": "pycvc-gl-cp311-cuda"}


def pin(text: str, url: str, sha256: str) -> str:
    """*text* with a vendored ``source:`` block replaced by the tarball, else unchanged."""
    m = SOURCE_BLOCK.search(text)
    if m is None or not VENDORED.search(m.group(0)):
        return text
    block = (
        "source:\n"
        "  # Pinned by .github/scripts/pin-recipe-source.py for the cvcpkg.org fleet: the release\n"
        "  # tag's tarball. In the repository this recipe builds from the checkout (vendored).\n"
        "  type: tarball\n"
        f"  url: {url}\n"
        f'  sha256: "{sha256}"\n'
        "  strip_components: 1\n"
        "\n"
    )
    return text[: m.start()] + block + text[m.end() :]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--recipes", required=True, type=Path, help="the recipes directory to copy")
    ap.add_argument("--out", required=True, type=Path, help="where the pinned copy goes (must not exist)")
    ap.add_argument("--url", required=True, help="the tarball every vendored recipe builds from")
    ap.add_argument("--sha256", required=True, help="the tarball's sha256 (hex)")
    ap.add_argument(
        "--revision",
        action="append",
        default=[],
        metavar="NAME=N",
        help="set recipe NAME's cvc_revision to N (repeatable)",
    )
    ap.add_argument(
        "--revisions-json",
        metavar="JSON",
        help="publish-cvcpkg.yml compute-revisions' outputs ({key: N}), each key a recipe name "
        "with '_' for '-' (pycvc_cuda and pycvc_gl_cuda are the cp311 CUDA columns)",
    )
    args = ap.parse_args()

    if not re.fullmatch(r"[0-9a-f]{64}", args.sha256):
        print(f"pin-recipe-source: --sha256 is not 64 hex digits: {args.sha256!r}", file=sys.stderr)
        return 2
    revisions = {}
    for item in args.revision:
        name, sep, value = item.partition("=")
        if not sep or not value.isdigit():
            print(f"pin-recipe-source: --revision wants NAME=N, got {item!r}", file=sys.stderr)
            return 2
        revisions[name] = int(value)
    if args.revisions_json:
        for key, value in json.loads(args.revisions_json).items():
            if not str(value).isdigit():
                print(f"pin-recipe-source: revision {key}={value!r} is not a number", file=sys.stderr)
                return 2
            revisions.setdefault(OUTPUT_KEY_RECIPES.get(key, key.replace("_", "-")), int(value))

    shutil.copytree(args.recipes, args.out)
    pinned = []
    for recipe_yaml in sorted(args.out.glob("*/recipe.yaml")):
        name = recipe_yaml.parent.name
        text = recipe_yaml.read_text(encoding="utf-8")
        new = pin(text, args.url, args.sha256)
        if name in revisions:
            new, n = REVISION.subn(rf"\g<1> {revisions[name]}\g<2>", new, count=1)
            if n != 1:
                print(f"pin-recipe-source: {name}: no cvc_revision line to set", file=sys.stderr)
                return 1
        if new != text:
            recipe_yaml.write_text(new, encoding="utf-8", newline="\n")
            pinned.append(name if name not in revisions else f"{name} (+cvc.{revisions[name]})")
    missing = sorted(set(revisions) - {p.parent.name for p in args.out.glob("*/recipe.yaml")})
    if missing:
        print(f"pin-recipe-source: no such recipe: {', '.join(missing)}", file=sys.stderr)
        return 1
    print(f"pinned {len(pinned)} recipe(s) to {args.url}: {', '.join(pinned)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
