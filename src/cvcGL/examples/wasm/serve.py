#!/usr/bin/env python3
"""Static server for the cvcGL wasm examples.

The -pthread build uses SharedArrayBuffer, which browsers only enable on
cross-origin-isolated pages — that requires these response headers, which
plain `python3 -m http.server` cannot send:

    Cross-Origin-Opener-Policy:   same-origin
    Cross-Origin-Embedder-Policy: require-corp

The headers are harmless for the single-threaded build, so this server is
safe to use for either variant.

Usage: serve.py [-d DIRECTORY] [PORT]
"""

import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


class IsolatedHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        # wasm/js must never be served stale while iterating on builds
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("-d", "--directory", default=".", help="directory to serve")
    ap.add_argument("port", nargs="?", type=int, default=8811)
    args = ap.parse_args()

    handler = functools.partial(IsolatedHandler, directory=args.directory)
    with ThreadingHTTPServer(("", args.port), handler) as httpd:
        print(f"Serving {args.directory} at http://localhost:{args.port} "
              "(cross-origin isolated)")
        httpd.serve_forever()


if __name__ == "__main__":
    main()
