#!/usr/bin/env python3
"""Pin the family edges of libcvc packages ALREADY on the catalog, as the recipes now pin new ones.

The recipes pin every dependency on one of libcvc's own packages to their own release
(`version: "==X.Y.Z"`, see recipe-family-pins.py), so a bundle published from them is only ever
installed beside its own release's libcvc, cvcgl and pycvc. Rows published before that record
those edges with no version, and cvcpkg's resolver falls back to them whenever the newest release
is only partly on the catalog: with libcvc 4.0.2 published and pycvc 4.0.2 not yet, an install of
pycvc gets pycvc 4.0.1 (no pin) beside libcvc 4.0.2, the mix that broke CVC-Lab/GRL-SNAM's CI
during 4.0.1's publish. So the pins protect a release only once the rows it can fall back to
carry pins too.

For every catalog row of a libcvc package (a recipe under cvcpkg/recipes/ or a name one provides)
whose required_deps name one of them without a version, this pins that edge to the row's own
upstream version (`==<X.Y.Z>`, any +cvc.N), which is the release it was built with. It uses the
registry's admin endpoint `POST /v1/admin/packages/{name}/{version}/backfill-deps`, one variant per
call, with `expect_required_deps` set to what the row records now, so a row anything else changed
in the meantime is left alone. Rows that already pin are skipped, so a rerun is a no-op.

Without --apply it only reads the public catalog and prints what it would change, and which rows
would then have no same-release peer on their platform (they stop resolving, rather than resolving
to a mix). --apply needs an admin token in CVCPKG_TOKEN and changes published rows, so it is run by
hand, once, by the registry's owner.

Usage:
  backfill-family-pins.py [--apply] [--server URL] [--org cvc] [--only-major N] [repo-root]
"""

import argparse
import json
import os
import re
import sys
import urllib.parse
import urllib.request
from collections import Counter
from pathlib import Path

import yaml

RECIPES = "cvcpkg/recipes"


def family(root):
    names = set()
    for path in sorted((root / RECIPES).glob("*/recipe.yaml")):
        with open(path, encoding="utf-8") as f:
            recipe = yaml.safe_load(f)
        names.add(path.parent.name)
        names |= set(recipe.get("provides") or [])
    return names


def upstream(version):
    return version.split("+", 1)[0]


def fetch_catalog(server):
    req = urllib.request.Request(f"{server}/v1/catalog", headers={"Accept": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as resp:
        return json.load(resp)


def plan(catalog, ours, org, only_major):
    """[(row, new_deps)] for every row of ours with an unpinned edge on ours."""
    out = []
    for b in catalog.get("bundles", []):
        if b["name"] not in ours or (b.get("org") or "") != org or b.get("yanked"):
            continue
        if only_major is not None and not upstream(b["version"]).startswith(f"{only_major}."):
            continue
        deps = b.get("required_deps") or []
        if not any(d.get("name") in ours and not d.get("version") for d in deps):
            continue
        pin = f"=={upstream(b['version'])}"
        new = []
        for d in deps:
            d = dict(d)
            if d.get("name") in ours and not d.get("version"):
                d["version"] = pin
            new.append(d)
        out.append((b, new))
    return out


def stranded(catalog, changes, ours):
    """Rows that resolve today but whose pinned peers are absent for their variant.

    A peer of SOME release exists for the variant (so the row resolves now, to a mix) but not
    one of the row's own release, so once pinned the resolver passes over it. Rows with no peer
    of any release for their variant do not resolve today either and are not listed.
    """
    have = set()
    have_any = set()
    for b in catalog.get("bundles", []):
        if b.get("yanked"):
            continue
        key = (b.get("platform"), b.get("arch"), b.get("build_type"), b.get("link"))
        for n in [b["name"], *(b.get("provides") or [])]:
            have.add((n, upstream(b["version"])) + key)
            have_any.add((n,) + key)
    out = []
    for b, new in changes:
        key = (b.get("platform"), b.get("arch"), b.get("build_type"), b.get("link"))
        pinned = [d for d in new
                  if d.get("name") in ours and d.get("version", "").startswith("==")]
        if any((d["name"],) + key not in have_any for d in pinned):
            continue  # unresolvable today already
        missing = [d["name"] for d in pinned if (d["name"], d["version"][2:]) + key not in have]
        if missing:
            out.append((b, missing))
    return out


def apply(server, token, org, b, new):
    query = urllib.parse.urlencode({"platform": b["platform"], "arch": b["arch"],
                                    "build_type": b["build_type"], "link": b["link"],
                                    "org": org})
    url = (f"{server}/v1/admin/packages/{urllib.parse.quote(b['name'], safe='')}/"
           f"{urllib.parse.quote(b['version'], safe='')}/backfill-deps?{query}")
    body = json.dumps({"required_deps": new,
                       "expect_required_deps": b.get("required_deps") or []}).encode()
    req = urllib.request.Request(url, data=body, method="POST",
                                 headers={"Authorization": f"Bearer {token}",
                                          "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=60) as resp:
        return json.load(resp)


def label(b):
    return (f"{b['name']}=={b['version']} "
            f"({b.get('platform')}/{b.get('arch')}/{b.get('build_type')}/{b.get('link')})")


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--apply", action="store_true", help="write the pins (admin CVCPKG_TOKEN)")
    ap.add_argument("--server", default=os.environ.get("CVCPKG_SERVER", "https://cvcpkg.org"))
    ap.add_argument("--org", default=os.environ.get("CVCPKG_ORG", "cvc"))
    ap.add_argument("--only-major", type=int, default=None,
                    help="limit to rows of one major (e.g. 4)")
    ap.add_argument("--verbose", action="store_true", help="list every row it would change")
    ap.add_argument("root", nargs="?", default=str(Path(__file__).resolve().parents[2]))
    args = ap.parse_args(argv)
    server = args.server.rstrip("/")

    ours = family(Path(args.root))
    catalog = fetch_catalog(server)
    changes = plan(catalog, ours, args.org, args.only_major)
    by_minor = Counter(re.sub(r"^(\d+\.\d+).*", r"\1", b["version"]) for b, _ in changes)
    print(f"{len(changes)} row(s) of {len(ours)} libcvc package names record a family edge "
          f"with no version; by release line: "
          + ", ".join(f"{k}: {v}" for k, v in sorted(
              by_minor.items(), key=lambda kv: [int(x) for x in kv[0].split(".")])))
    if args.verbose:
        for b, new in changes:
            pins = ", ".join(f"{d['name']} {d['version']}" for d in new if d.get("name") in ours)
            print(f"  {label(b)}: {pins}")
    lost = stranded(catalog, changes, ours)
    print(f"{len(lost)} of them resolve today only beside another release's packages (no "
          f"same-release peer for their variant), so once pinned the resolver passes over them:")
    for b, missing in lost[:50]:
        print(f"  {label(b)}: no {', '.join(missing)} {upstream(b['version'])}")
    if len(lost) > 50:
        print(f"  ... and {len(lost) - 50} more")

    if not args.apply:
        print("dry run: nothing written (--apply writes the pins with an admin CVCPKG_TOKEN)")
        return 0
    token = os.environ.get("CVCPKG_TOKEN", "")
    if not token:
        print("--apply needs an admin token in CVCPKG_TOKEN")
        return 1
    done = 0
    failed = 0
    for b, new in changes:
        try:
            result = apply(server, token, args.org, b, new)
        except Exception as exc:  # noqa: BLE001 - report every row, then fail
            print(f"FAILED {label(b)}: {exc}")
            failed += 1
            continue
        done += 1
        if args.verbose:
            print(f"  {label(b)}: {result}")
    print(f"pinned {done} row(s), {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
