#!/usr/bin/env python3
"""Static server for the cvcGL wasm examples.

The -pthread build uses SharedArrayBuffer, which browsers only enable on
cross-origin-isolated pages — that requires these response headers, which
plain `python3 -m http.server` cannot send:

    Cross-Origin-Opener-Policy:   same-origin
    Cross-Origin-Embedder-Policy: require-corp

The headers are harmless for the single-threaded build, so this server is
safe to use for either variant.

Profiling (all off by default; without them the server is exactly the plain
isolated one above):

  --glsync[=PATH]   insert <script src="/glsync.js"></script> before the first
                    <script> of every served index.html, so the Firefox WebGL
                    sync census (src/cvcGL/wasm/devtools/glsync.js, see GLSYNC.md
                    there) wraps WebGL before anything else does. It does
                    nothing unless the page URL has ?glsync. /glsync.js is
                    served from PATH whatever -d says; the default is the copy
                    in the source tree (or devtools/ next to an installed copy
                    of this script). It is read on every request: edit, reload.
  --prof-param NAME also inject window.__glsyncProfParam = "NAME", so the census
                    adds ?NAME to the URL (for an app that prints PROF lines
                    only with a URL switch, e.g. "prof"). Needs --glsync.
  --js-profiling    send Document-Policy: js-profiling, which Chromium requires
                    before it exposes the JS Self-Profiling API (it samples wasm
                    frames too).

Usage: serve.py [-d DIRECTORY] [--glsync[=PATH]] [--prof-param NAME]
                [--js-profiling] [PORT]
"""

import argparse
import functools
import io
import os
import re
import sys
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

HERE = Path(__file__).resolve().parent
# glsync.js: in the source tree (src/cvcGL/examples/wasm -> src/cvcGL/wasm/devtools), or in a
# devtools/ directory next to an installed copy of this script.
GLSYNC_CANDIDATES = (HERE.parent.parent / "wasm" / "devtools" / "glsync.js",
                     HERE / "devtools" / "glsync.js")
GLSYNC_URL = "/glsync.js"
GLSYNC_TAG = b'<script src="/glsync.js"></script>'
FIRST_SCRIPT = re.compile(rb"<script\b", re.IGNORECASE)
PROF_PARAM = re.compile(r"^[A-Za-z0-9_-]+$")


def default_glsync():
    """The first glsync.js candidate that exists, else None."""
    return next((p for p in GLSYNC_CANDIDATES if p.is_file()), None)


def inject_glsync(html: bytes, prof_param=None) -> bytes:
    """Insert the census tag (after the optional __glsyncProfParam script) before the first
    <script>, or at the end; idempotent."""
    if GLSYNC_TAG in html:
        return html
    tags = GLSYNC_TAG + b"\n"
    if prof_param:
        tags = b'<script>window.__glsyncProfParam = "' + prof_param.encode("ascii") + b'";</script>\n' + tags
    m = FIRST_SCRIPT.search(html)
    if m is None:
        return html + tags
    return html[: m.start()] + tags + html[m.start():]


class IsolatedHandler(SimpleHTTPRequestHandler):
    # Set per server by make_handler(); the class defaults are the plain isolated server.
    glsync_js = None      # Path to serve at /glsync.js and inject into index.html, or None
    prof_param = None     # name for window.__glsyncProfParam, or None
    js_profiling = False  # send Document-Policy: js-profiling

    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        if self.js_profiling:
            self.send_header("Document-Policy", "js-profiling")
        # wasm/js must never be served stale while iterating on builds
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def send_head(self):
        if self.glsync_js is None:
            return super().send_head()
        url_path = unquote(urlsplit(self.path).path)
        if url_path == GLSYNC_URL:
            try:
                body = Path(self.glsync_js).read_bytes()
            except OSError:
                self.send_error(HTTPStatus.NOT_FOUND, "glsync.js not found")
                return None
            return self._send_bytes(body, "text/javascript; charset=utf-8")
        fs_path = self.translate_path(self.path)
        if os.path.isdir(fs_path):
            if not url_path.endswith("/"):
                return super().send_head()  # the usual 301 to the trailing-slash URL
            fs_path = os.path.join(fs_path, "index.html")
        elif os.path.basename(fs_path) != "index.html":
            return super().send_head()
        if not os.path.isfile(fs_path):
            return super().send_head()  # index.htm, directory listing or 404, as before
        try:
            html = Path(fs_path).read_bytes()
        except OSError:
            self.send_error(HTTPStatus.NOT_FOUND, "File not found")
            return None
        return self._send_bytes(inject_glsync(html, self.prof_param), "text/html; charset=utf-8")

    def _send_bytes(self, body: bytes, content_type: str):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        return io.BytesIO(body)


def make_handler(directory, glsync_js=None, prof_param=None, js_profiling=False,
                 base=IsolatedHandler):
    """A handler class serving `directory` with the given profiling options."""
    if prof_param is not None and not PROF_PARAM.match(prof_param):
        raise ValueError(f"--prof-param must match {PROF_PARAM.pattern}: {prof_param!r}")
    if prof_param is not None and glsync_js is None:
        raise ValueError("--prof-param needs --glsync")
    cls = type("Handler", (base,), {
        "glsync_js": None if glsync_js is None else Path(glsync_js),
        "prof_param": prof_param,
        "js_profiling": bool(js_profiling),
    })
    return functools.partial(cls, directory=str(directory))


def parse_args(argv=None):
    """Parse the command line: (namespace, glsync Path or None). Exits on a bad combination."""
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--directory", default=".", help="directory to serve")
    ap.add_argument("--glsync", nargs="?", const="", default=None, metavar="PATH",
                    help="inject the WebGL sync census (default: the source-tree glsync.js)")
    ap.add_argument("--prof-param", default=None, metavar="NAME",
                    help="have the census add ?NAME to the page URL (needs --glsync)")
    ap.add_argument("--js-profiling", action="store_true",
                    help="send Document-Policy: js-profiling (Chromium JS Self-Profiling API)")
    ap.add_argument("port", nargs="?", type=int, default=None, help="port (default 8811)")
    args = ap.parse_args(argv)

    # `--glsync 8822`: argparse hands the port to --glsync's optional PATH. A bare number that is
    # not a file is the port.
    if args.glsync and args.glsync.isdigit() and not Path(args.glsync).exists():
        if args.port is not None:
            ap.error(f"--glsync {args.glsync}: no such file (use --glsync=PATH)")
        args.port, args.glsync = int(args.glsync), ""
    if args.port is None:
        args.port = 8811

    glsync = None
    if args.glsync is not None:
        glsync = Path(args.glsync) if args.glsync else default_glsync()
        if glsync is None or not glsync.is_file():
            ap.error("--glsync: glsync.js not found (looked at "
                     + (str(glsync) if args.glsync else ", ".join(map(str, GLSYNC_CANDIDATES)))
                     + "); pass --glsync=PATH")
    if args.prof_param is not None and glsync is None:
        ap.error("--prof-param needs --glsync")
    if args.prof_param is not None and not PROF_PARAM.match(args.prof_param):
        ap.error(f"--prof-param must match {PROF_PARAM.pattern}: {args.prof_param!r}")
    return args, glsync


def main(argv=None):
    args, glsync = parse_args(argv)
    handler = make_handler(args.directory, glsync, args.prof_param, args.js_profiling)
    extras = []
    if glsync is not None:
        extras.append(f"?glsync census from {glsync}")
    if args.prof_param:
        extras.append(f"census adds ?{args.prof_param}")
    if args.js_profiling:
        extras.append("js-profiling")
    with ThreadingHTTPServer(("", args.port), handler) as httpd:
        print(f"Serving {args.directory} at http://localhost:{args.port} "
              "(cross-origin isolated" + "".join("; " + e for e in extras) + ")")
        sys.stdout.flush()
        httpd.serve_forever()


if __name__ == "__main__":
    main()
