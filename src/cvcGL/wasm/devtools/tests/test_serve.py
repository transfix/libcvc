#!/usr/bin/env python3
"""Tests for src/cvcGL/examples/wasm/serve.py: the opt-in glsync.js injection, --prof-param,
--js-profiling, and the unchanged isolation headers (and nothing else) without them.

Runs the handler in-process on ephemeral 127.0.0.1 ports (never a port a developer uses), against
a temporary gallery-shaped directory.
    python3 src/cvcGL/wasm/devtools/tests/test_serve.py
"""
import http.client
import importlib.util
import io
import tempfile
import threading
import unittest
from contextlib import redirect_stderr
from http.server import ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEVTOOLS = HERE.parent
SERVE_PY = HERE.parents[2] / "examples" / "wasm" / "serve.py"
spec = importlib.util.spec_from_file_location("cvcgl_serve", SERVE_PY)
serve = importlib.util.module_from_spec(spec)
spec.loader.exec_module(serve)
TAG = serve.GLSYNC_TAG
GLSYNC = DEVTOOLS / "glsync.js"
PAGE = (b"<!doctype html><html><head><style>x</style></head><body><canvas></canvas>"
        b"<script>var Module = {};</script><script src=\"demo.js\"></script></body></html>")


class QuietHandler(serve.IsolatedHandler):
    def log_message(self, *args):
        pass


class Server:
    def __init__(self, directory, **opts):
        handler = serve.make_handler(directory, base=QuietHandler, **opts)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.port = self.httpd.server_address[1]
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def get(self, path, method="GET"):
        c = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        c.request(method, path)
        r = c.getresponse()
        body = r.read()
        c.close()
        return r, body

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


def make_gallery(root: Path):
    (root / "demo").mkdir()
    (root / "demo" / "index.html").write_bytes(PAGE)
    (root / "demo" / "demo.js").write_bytes(b"// the module\n")
    (root / "index.html").write_bytes(b"<html><body><a href=demo/>demo</a><script>x()</script></body></html>")
    (root / "nodir").mkdir()


class InjectTest(unittest.TestCase):
    def test_before_first_script_case_insensitive(self):
        html = b"<html><head><style>x</style></head><body><SCRIPT>a()</SCRIPT><script src=b.js></script>"
        out = serve.inject_glsync(html)
        self.assertEqual(out.count(TAG), 1)
        self.assertLess(out.index(TAG), out.index(b"<SCRIPT>a()"))

    def test_idempotent_and_no_script(self):
        once = serve.inject_glsync(b"<p>no scripts</p>")
        self.assertTrue(once.endswith(TAG + b"\n"))
        self.assertEqual(serve.inject_glsync(once), once)

    def test_not_fooled_by_scripts_prefix(self):
        out = serve.inject_glsync(b"<scripts-like></scripts-like><script>x</script>")
        self.assertLess(out.index(TAG), out.index(b"<script>x"))
        self.assertGreater(out.index(TAG), out.index(b"</scripts-like>"))

    def test_prof_param_goes_first(self):
        out = serve.inject_glsync(b"<body><script>go()</script>", "prof")
        self.assertEqual(out, b'<body><script>window.__glsyncProfParam = "prof";</script>\n'
                              + TAG + b"\n<script>go()</script>")
        self.assertEqual(serve.inject_glsync(out, "prof"), out)


class NoFlagsTest(unittest.TestCase):
    """No flags: exactly the plain isolated server -- no injection, no Document-Policy."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        make_gallery(Path(cls.tmp.name))
        cls.srv = Server(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):
        cls.srv.close()
        cls.tmp.cleanup()

    def test_page_untouched_and_headers(self):
        for path in ("/demo/", "/demo/index.html", "/demo/?glsync"):
            r, body = self.srv.get(path)
            self.assertEqual(r.status, 200, path)
            self.assertEqual(body, PAGE, path)
            self.assertEqual(r.getheader("Cross-Origin-Opener-Policy"), "same-origin")
            self.assertEqual(r.getheader("Cross-Origin-Embedder-Policy"), "require-corp")
            self.assertEqual(r.getheader("Cache-Control"), "no-cache")
            self.assertIsNone(r.getheader("Document-Policy"), path)

    def test_no_census_url(self):
        r, _ = self.srv.get("/glsync.js")
        self.assertEqual(r.status, 404)


class GlsyncTest(unittest.TestCase):
    """--glsync=PATH --js-profiling."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        make_gallery(Path(cls.tmp.name))
        cls.srv = Server(cls.tmp.name, glsync_js=GLSYNC, js_profiling=True)

    @classmethod
    def tearDownClass(cls):
        cls.srv.close()
        cls.tmp.cleanup()

    def assert_headers(self, r):
        self.assertEqual(r.getheader("Cross-Origin-Opener-Policy"), "same-origin")
        self.assertEqual(r.getheader("Cross-Origin-Embedder-Policy"), "require-corp")
        self.assertEqual(r.getheader("Document-Policy"), "js-profiling")
        self.assertEqual(r.getheader("Cache-Control"), "no-cache")

    def test_demo_page_injected_before_first_script(self):
        for path in ("/demo/", "/demo/index.html", "/demo/?glsync=seq"):
            r, body = self.srv.get(path)
            self.assertEqual(r.status, 200, path)
            self.assert_headers(r)
            self.assertEqual(int(r.getheader("Content-Length")), len(PAGE) + len(TAG) + 1)
            self.assertTrue(r.getheader("Content-Type").startswith("text/html"))
            self.assertEqual(body.count(TAG), 1)
            self.assertEqual(body.index(TAG), body.lower().index(b"<script"), "not the first <script")
            self.assertEqual(body.replace(TAG + b"\n", b"", 1), PAGE, "page otherwise changed")
            self.assertNotIn(b"__glsyncProfParam", body)

    def test_head_request_matches_get(self):
        r, body = self.srv.get("/demo/", method="HEAD")
        self.assertEqual(r.status, 200)
        self.assertEqual(body, b"")
        self.assertEqual(int(r.getheader("Content-Length")), len(PAGE) + len(TAG) + 1)

    def test_gallery_index_injected_too(self):
        r, body = self.srv.get("/")
        self.assertEqual(r.status, 200)
        self.assertEqual(body.count(TAG), 1)

    def test_glsync_js_served_from_path(self):
        r, body = self.srv.get("/glsync.js")
        self.assertEqual(r.status, 200)
        self.assertTrue(r.getheader("Content-Type").startswith("text/javascript"))
        self.assertEqual(body, GLSYNC.read_bytes())
        self.assert_headers(r)

    def test_other_files_untouched(self):
        r, body = self.srv.get("/demo/demo.js")
        self.assertEqual(r.status, 200)
        self.assertEqual(body, b"// the module\n")
        self.assert_headers(r)

    def test_dir_without_slash_redirects(self):
        r, _ = self.srv.get("/demo")
        self.assertEqual(r.status, 301)
        self.assertEqual(r.getheader("Location"), "/demo/")

    def test_missing_index_404_and_listing(self):
        r, _ = self.srv.get("/nope/index.html")
        self.assertEqual(r.status, 404)
        r, body = self.srv.get("/nodir/")
        self.assertEqual(r.status, 200)  # directory listing, as before
        self.assertNotIn(TAG, body)


class ProfParamTest(unittest.TestCase):
    def test_prof_param_injected_ahead_of_the_census(self):
        with tempfile.TemporaryDirectory() as d:
            make_gallery(Path(d))
            srv = Server(d, glsync_js=GLSYNC, prof_param="prof")
            try:
                r, body = srv.get("/demo/")
                pre = b'<script>window.__glsyncProfParam = "prof";</script>\n'
                self.assertEqual(body.count(pre), 1)
                self.assertLess(body.index(pre), body.index(TAG))
                self.assertEqual(body.index(pre), body.lower().index(b"<script"))
                self.assertIsNone(r.getheader("Document-Policy"))  # only with --js-profiling
            finally:
                srv.close()

    def test_bad_combinations(self):
        with self.assertRaises(ValueError):
            serve.make_handler(".", glsync_js=GLSYNC, prof_param="a b")
        with self.assertRaises(ValueError):
            serve.make_handler(".", prof_param="prof")  # needs --glsync


class CliTest(unittest.TestCase):
    def parse(self, *argv):
        return serve.parse_args(list(argv))

    def fails(self, *argv):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            serve.parse_args(list(argv))

    def test_defaults_are_off(self):
        args, glsync = self.parse()
        self.assertIsNone(glsync)
        self.assertIsNone(args.prof_param)
        self.assertFalse(args.js_profiling)
        self.assertEqual(args.port, 8811)

    def test_bare_glsync_finds_the_source_tree_copy(self):
        args, glsync = self.parse("--glsync", "-d", "/tmp")
        self.assertEqual(glsync.resolve(), GLSYNC.resolve())
        self.assertEqual(serve.default_glsync().resolve(), GLSYNC.resolve())

    def test_glsync_path_and_port(self):
        args, glsync = self.parse(f"--glsync={GLSYNC}", "--prof-param", "prof", "--js-profiling", "8831")
        self.assertEqual(glsync, GLSYNC)
        self.assertEqual((args.prof_param, args.js_profiling, args.port), ("prof", True, 8831))

    def test_glsync_then_port_is_the_port(self):
        args, glsync = self.parse("--glsync", "8822")
        self.assertEqual(args.port, 8822)
        self.assertEqual(glsync.resolve(), GLSYNC.resolve())

    def test_errors(self):
        self.fails("--prof-param", "prof")               # needs --glsync
        self.fails("--glsync", "--prof-param", "a;b")    # not a URL parameter name
        self.fails("--glsync=/nonexistent/glsync.js")
        self.fails("--glsync", "8822", "9000")           # 8822 is no file


if __name__ == "__main__":
    unittest.main(verbosity=2)
