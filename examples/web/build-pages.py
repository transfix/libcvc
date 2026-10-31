#!/usr/bin/env python3
"""Assemble the servable cvcGL wasm demo gallery from a built bin/ directory.

The binary build (build-wasm-demo.sh) drops <demo>.js/.wasm into build-wasm*/bin.
This turns that flat output into a gallery tree:

    <out>/index.html              # the gallery, one card per demo
    <out>/<demo>/index.html       # each demo's host page (from templates/demo.html.in)
    <out>/<demo>/<demo>.{js,wasm} # the binaries
    <out>/img/, <out>/fonts/      # assets copied from --assets (if given)
    <out>/gallery.json            # the deployed demos, for arigl's Demos menu
    <out>/gallery.js              # the same, for every page's demo picker and the C++ demos' menu

Demos are DISCOVERED by scanning bin/ for a matching <demo>.js + <demo>.wasm pair,
so a newly-added example target shows up with no edits here. demos.json supplies
rich per-demo text and gallery order; anything not listed still appears with
name-based defaults. Templates are plain files with {{TOKEN}} placeholders, so a
downstream site (e.g. a branded host) can point --templates at its own theme and
reuse the same discovery/layout.

A demos.json entry with "app" and "doc" is a DOCUMENT demo. It has no binary of its own: it is
that app's page opened on that document (<app>/?doc=<doc>), so arigl -- the gallery's Ariadne app
-- runs every .ari demo and switches between them in place. It is listed when its app was built,
and its own directory redirects there. An app's "boot" ({args, env, files, doc}) is its command
line ({doc} becomes /ariadne/documents/<?doc= or boot.doc>.ari), its environment, and files
fetched into its file system before main (path -> URL). "card": false keeps a demo off the index.
arigl_term (arigl --terminal behind xterm.js, its own page) takes a doc under /ariadne instead
("documents/terminal_scene"); in arigl's Demos menu its documents are pages to open, not loaded.
"""
import argparse
import json
import hashlib
import math
import shutil
import sys
import urllib.parse
from pathlib import Path

HERE = Path(__file__).resolve().parent

# Appended to gallery.js: fill a page's <select id="demos"> and go where it is pointed.
PICKER_JS = r"""
(function () {
  function fill() {
    var sel = document.getElementById('demos');
    if (!sel) return;
    var here = location.pathname.replace(/\/index\.html$/, '').replace(/\/$/, '').split('/').pop();
    var doc = new URLSearchParams(location.search).get('doc');
    window.CVC_GALLERY.forEach(function (d) {
      var o = document.createElement('option');
      o.value = d.href;
      o.textContent = d.title;
      o.title = d.desc;
      var m = /\.\.\/([^/]+)\/(?:\?doc=(.*))?$/.exec(d.href);
      if (m && m[1] === here && (m[2] ? decodeURIComponent(m[2]) === doc : !doc)) o.selected = true;
      sel.appendChild(o);
    });
    sel.addEventListener('change', function () { location.href = sel.value; });
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', fill);
  else fill();
})();
"""


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

    def app_of(demo):
        """The app a document demo runs in, or None for a demo with its own binary."""
        info = meta.get(demo, {})
        return info.get("app") if info.get("doc") else None

    listed = set(discovered) | {d for d in meta if app_of(d) in discovered}

    # order: manifest order first (kept only if actually built), then any extras
    ordered = [d for d in manifest.get("order", []) if d in listed]
    ordered += [d for d in discovered if d not in ordered]

    def href(demo):
        """Where a demo is, from a page one directory down (every page is <out>/<demo>/)."""
        app = app_of(demo)
        if app:
            return f"../{app}/?doc={urllib.parse.quote(meta[demo]['doc'])}"
        return f"../{demo}/"

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
        ddir = args.out / demo
        ddir.mkdir(parents=True, exist_ok=True)

        if app_of(demo):
            # A document demo: its own address opens its app on its document.
            target = href(demo)
            (ddir / "index.html").write_text(
                "<!DOCTYPE html>\n<meta charset=\"utf-8\">\n"
                f"<meta http-equiv=\"refresh\" content=\"0; url={target}\">\n"
                f"<link rel=\"canonical\" href=\"{target}\">\n"
                f"<a href=\"{target}\">{demo}</a>\n")
            size = "doc"
        else:
            # per-demo page
            wasm = args.bin / (demo + ".wasm")
            wasm_mb = max(1, math.ceil(wasm.stat().st_size / 1048576))
            asset_v = hashlib.sha256(wasm.read_bytes()).hexdigest()[:12]
            shutil.copy2(wasm, ddir / (demo + ".wasm"))
            shutil.copy2(args.bin / (demo + ".js"), ddir / (demo + ".js"))
            # The emscripten --preload-file package (bundle + weights) lands as <demo>.data
            # next to the js/wasm; the loader fetches it, so it MUST ship or the demo 404s.
            data = args.bin / (demo + ".data")
            if data.is_file():
                shutil.copy2(data, ddir / (demo + ".data"))
            # A target that builds its own host page next to its binaries (<demo>.html, e.g. ari's
            # xterm.js terminal, which has no canvas) keeps it; the rest get the canvas template.
            own_page = args.bin / (demo + ".html")
            if own_page.is_file():
                shutil.copy2(own_page, ddir / "index.html")
                # ...with the files it loads beside it (ari's: its loader, style sheet and xterm/),
                # which <demo>.page-files lists, one path relative to the binaries per line.
                listing = args.bin / (demo + ".page-files")
                if listing.is_file():
                    for rel in listing.read_text().split():
                        src = args.bin / rel
                        if src.is_dir():
                            shutil.copytree(src, ddir / rel, dirs_exist_ok=True)
                        else:
                            shutil.copy2(src, ddir / rel)
            else:
                (ddir / "index.html").write_text(render(
                    demo_tpl,
                    DEMO=demo,
                    DESC=info.get("desc", "a cvcGL WebAssembly example"),
                    WASM_MB=wasm_mb,
                    ASSET_V=asset_v,
                    BOOT=json.dumps(info.get("boot")),
                ))
            size = f"{wasm_mb} MB"

        # gallery card
        if info.get("card", True):
            thumb = info.get("thumb")
            if not (thumb and (args.out / thumb).is_file()):
                thumb = placeholder_thumb(demo)
            cards.append(render(
                card_tpl,
                DEMO=demo,
                HREF=href(demo)[len("../"):],  # from the index, one directory up
                TITLE=info.get("title", demo),
                BLURB=info.get("blurb", "A cvcGL scene-graph example, cross-compiled to WebAssembly."),
                THUMB=thumb,
            ))
        print(f"  {demo:<20} {size:>6}  {'(defaults)' if used_default else 'manifest'}")

    # The deployed demos, for the menus: arigl reads gallery.json (a document demo is a .ari it
    # loads in place, at the path its build embeds it under), the pages and C++ demos gallery.js.
    entries = []
    for demo in ordered:
        info = meta.get(demo, {})
        if not info.get("card", True):
            continue  # an app's own page (arigl's): it is in the menus as its documents
        e = {"slug": demo, "title": info.get("title", demo), "desc": info.get("desc", ""),
             "href": href(demo)}
        if app_of(demo) == "arigl":  # arigl loads it in place; another app's is a page to open
            e["doc"] = f"/ariadne/documents/{info['doc']}.ari"
        entries.append(e)
    (args.out / "gallery.json").write_text(json.dumps({"demos": entries}, indent=1) + "\n")
    (args.out / "gallery.js").write_text(
        "// GENERATED by build-pages.py: the deployed demos, for each page's #demos picker and for a\n"
        "// demo's own Demos menu (window.CVC_GALLERY).\n"
        f"window.CVC_GALLERY = {json.dumps(entries)};\n" + PICKER_JS)

    (args.out / "index.html").write_text(render(gallery_tpl, CARDS="\n".join(cards)))
    print(f"gallery: {len(ordered)} demo(s) -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
