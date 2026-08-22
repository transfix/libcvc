#!/usr/bin/env python3
"""Assemble the servable cvcGL wasm demo gallery from a built bin/ directory.

The binary build (build-wasm-demo.sh) drops <demo>.js/.wasm into build-wasm*/bin.
This turns that flat output into a gallery tree:

    <out>/index.html              # the gallery, one card per demo
    <out>/<demo>/index.html       # each demo's host page (from templates/demo.html.in)
    <out>/<demo>/<demo>.{js,wasm} # the binaries
    <out>/img/, <out>/fonts/      # assets copied from --assets (if given)

Demos are DISCOVERED by scanning bin/ for a matching <demo>.js + <demo>.wasm pair,
so a newly-added example target shows up with no edits here. demos.json supplies
rich per-demo text and gallery order; anything not listed still appears with
name-based defaults. Templates are plain files with {{TOKEN}} placeholders, so a
downstream site (e.g. a branded host) can point --templates at its own theme and
reuse the same discovery/layout.
"""
import argparse
import json
import math
import shutil
import sys
import urllib.parse
from pathlib import Path

HERE = Path(__file__).resolve().parent


def placeholder_thumb(demo: str) -> str:
    """A neutral inline-SVG data URI for demos with no bundled thumbnail."""
    svg = (
        "<svg xmlns='http://www.w3.org/2000/svg' width='320' height='180'>"
        "<rect width='100%' height='100%' fill='#0d1117'/>"
        "<text x='50%' y='50%' fill='#5aa9e6' font-family='monospace' font-size='18' "
        "text-anchor='middle' dominant-baseline='middle'>" + demo + "</text></svg>"
    )
    return "data:image/svg+xml," + urllib.parse.quote(svg)


def render(template: str, **subs) -> str:
    out = template
    for key, val in subs.items():
        out = out.replace("{{" + key + "}}", str(val))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bin", required=True, type=Path, help="built output dir (has <demo>.js/.wasm)")
    ap.add_argument("--out", required=True, type=Path, help="gallery output dir (created)")
    ap.add_argument("--templates", type=Path, default=HERE / "templates",
                    help="dir with demo.html.in, gallery.html.in, card.html.in")
    ap.add_argument("--manifest", type=Path, default=HERE / "demos.json")
    ap.add_argument("--assets", type=Path, default=None,
                    help="optional dir whose contents (img/, fonts/, …) are copied into <out>")
    args = ap.parse_args()

    if not args.bin.is_dir():
        sys.exit(f"build-pages: --bin {args.bin} is not a directory")

    manifest = {"order": [], "demos": {}}
    if args.manifest.is_file():
        manifest = json.loads(args.manifest.read_text())
    meta = manifest.get("demos", {})

    demo_tpl = (args.templates / "demo.html.in").read_text()
    gallery_tpl = (args.templates / "gallery.html.in").read_text()
    card_tpl = (args.templates / "card.html.in").read_text()

    # discover: a demo is any <name>.wasm with a sibling <name>.js
    discovered = sorted(
        p.stem for p in args.bin.glob("*.wasm") if (args.bin / (p.stem + ".js")).is_file()
    )
    if not discovered:
        sys.exit(f"build-pages: no <demo>.wasm+.js pairs found in {args.bin}")

    # order: manifest order first (kept only if actually built), then any extras
    ordered = [d for d in manifest.get("order", []) if d in discovered]
    ordered += [d for d in discovered if d not in ordered]

    args.out.mkdir(parents=True, exist_ok=True)
    if args.assets and args.assets.is_dir():
        for item in args.assets.iterdir():
            dst = args.out / item.name
            if item.is_dir():
                shutil.copytree(item, dst, dirs_exist_ok=True)
            else:
                shutil.copy2(item, dst)

    cards = []
    for demo in ordered:
        info = meta.get(demo, {})
        used_default = demo not in meta

        # per-demo page
        wasm = args.bin / (demo + ".wasm")
        wasm_mb = max(1, math.ceil(wasm.stat().st_size / 1048576))
        ddir = args.out / demo
        ddir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(wasm, ddir / (demo + ".wasm"))
        shutil.copy2(args.bin / (demo + ".js"), ddir / (demo + ".js"))
        (ddir / "index.html").write_text(render(
            demo_tpl,
            DEMO=demo,
            DESC=info.get("desc", "a cvcGL WebAssembly example"),
            WASM_MB=wasm_mb,
        ))

        # gallery card
        thumb = info.get("thumb")
        if not (thumb and (args.out / thumb).is_file()):
            thumb = placeholder_thumb(demo)
        cards.append(render(
            card_tpl,
            DEMO=demo,
            TITLE=info.get("title", demo),
            BLURB=info.get("blurb", "A cvcGL scene-graph example, cross-compiled to WebAssembly."),
            THUMB=thumb,
        ))
        print(f"  {demo:<20} {wasm_mb:>3} MB  {'(defaults)' if used_default else 'manifest'}")

    (args.out / "index.html").write_text(render(gallery_tpl, CARDS="\n".join(cards)))
    print(f"gallery: {len(ordered)} demo(s) -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
