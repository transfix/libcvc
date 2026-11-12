#!/usr/bin/env python3
"""Tests for scan.py: the parser, the classification, the allowlist matching, and one end-to-end
run on a real ELF object compiled from fixture.cpp.

    python3 tools/singleton_scan/test_scan.py

The end-to-end case needs a C++ compiler that writes ELF and GNU nm and c++filt. It reads
SCAN_TEST_CXX (default "c++"), SCAN_TEST_NM ("nm") and SCAN_TEST_CXXFILT ("c++filt"), and skips
when they are missing, unless SCAN_TEST_REQUIRE_E2E=1 (as CI sets it).
"""

from __future__ import annotations

import io
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ratchet  # noqa: E402
import scan  # noqa: E402

SYSV = """\

Symbols from /b/src/cvc/CMakeFiles/cvc.dir/core/app.cpp.o:

Name                  Value           Class        Type         Size             Line  Section

_ZN3cvc8g_handlersE |0000000000000000|   B  |            OBJECT|0000000000000030|     |.bss
_ZZN3cvc3app4initEvE4seen|0000000000000040|   b  |            OBJECT|0000000000000001|     |.bss
_ZGVZN3cvc3app4initEvE4seen|0000000000000048|   b  |            OBJECT|0000000000000008|     |.bss
_ZN3cvc3app4initEv  |0000000000000000|   T  |              FUNC|0000000000000010|     |.text
_ZTVN3cvc3appE      |0000000000000000|   V  |            OBJECT|0000000000000020|     |.data.rel.ro._ZTVN3cvc3appE
_ZStL8__ioinit      |0000000000000050|   b  |            OBJECT|0000000000000001|     |.bss
_ZN3cvc7k_tableE    |0000000000000000|   R  |            OBJECT|0000000000000010|     |.rodata

Symbols from /b/src/cvc/CMakeFiles/cvc.dir/state/state.cpp.o:

Name                  Value           Class        Type         Size             Line  Section

_ZZN3cvc6detail5cacheEvE1c|0000000000000000|   u  |            OBJECT|0000000000000008|     |.bss._ZZN3cvc6detail5cacheEvE1c
_ZN3cvc5tl_idE      |0000000000000000|   B  |               TLS|0000000000000008|     |.tbss
_ZN3cvc6g_nameE     |0000000000000000|   D  |            OBJECT|0000000000000008|     |.data.rel.local
"""

NAMES = {
    "_ZN3cvc8g_handlersE": "cvc::g_handlers",
    "_ZZN3cvc3app4initEvE4seen": "cvc::app::init()::seen",
    "_ZGVZN3cvc3app4initEvE4seen": "guard variable for cvc::app::init()::seen",
    "_ZStL8__ioinit": "std::__ioinit",
    "_ZZN3cvc6detail5cacheEvE1c": "cvc::detail::cache()::c",
    "_ZN3cvc5tl_idE": "cvc::tl_id",
    "_ZN3cvc6g_nameE": "cvc::g_name",
}

OBJECTS = {
    "cvc": [
        ("/b/src/cvc/CMakeFiles/cvc.dir/core/app.cpp.o", "src/cvc/core/app.cpp"),
        ("/b/src/cvc/CMakeFiles/cvc.dir/state/state.cpp.o", "src/cvc/state/state.cpp"),
    ]
}

EXPECTED = {
    ("cvc", "src/cvc/core/app.cpp", "bss", "cvc::g_handlers"),
    ("cvc", "src/cvc/core/app.cpp", "bss", "cvc::app::init()::seen"),
    ("cvc", "src/cvc/core/app.cpp", "guard", "guard variable for cvc::app::init()::seen"),
    ("cvc", scan.HEADER, "bss", "cvc::detail::cache()::c"),
    ("cvc", "src/cvc/state/state.cpp", "tls", "cvc::tl_id"),
    ("cvc", "src/cvc/state/state.cpp", "data", "cvc::g_name"),
}


class Classification(unittest.TestCase):
    def test_sections(self):
        k = scan.section_kind
        self.assertEqual(k(".bss", "b"), "bss")
        self.assertEqual(k(".bss._ZZ3foovE1x", "u"), "bss")
        self.assertEqual(k(".data", "D"), "data")
        self.assertEqual(k(".data.rel.local", "d"), "data")
        self.assertEqual(k(".tbss", "B"), "tls")
        self.assertEqual(k(".tdata.x", "D"), "tls")
        self.assertEqual(k("*COM*", "C"), "common")
        self.assertIsNone(k(".data.rel.ro", "d"))
        self.assertIsNone(k(".data.rel.ro.local", "d"))
        self.assertIsNone(k(".rodata", "R"))
        self.assertIsNone(k(".text", "T"))

    def test_left_out(self):
        r = scan.left_out_reason
        wrap = "bindings/pycvc/CMakeFiles/pycvc.dir/pycvcPYTHON_wrap.cxx"
        self.assertIsNotNone(r(wrap, "_swigt__p_cvc__geometry"))
        self.assertIsNotNone(
            r(wrap, "guard variable for swig::traits_info<cvc::image>::type_info()::info")
        )
        # pycvc's own code in the wrapper stays, and SWIG's names count only in a wrapper.
        self.assertIsNone(r(wrap, "pycvc::kSimWorldCapsule"))
        self.assertIsNone(r("src/cvc/core/app.cpp", "_swigt__p_cvc__geometry"))
        self.assertIsNotNone(
            r("src/cvc/proto/state_transport.pb.cc", "cvc::transport::v1::Frame::_table_")
        )
        self.assertIsNotNone(r("src/cvc/core/app.cpp", "std::__ioinit"))
        self.assertIsNone(r("src/cvc/core/app.cpp", "cvc::g_handlers"))

    def test_third_party_left_out(self):
        r = scan.left_out_reason
        for src, name in (
            (scan.HEADER, "boost::asio::detail::service_base<x>::id"),
            (scan.HEADER, "guard variable for boost::date_time::month_str_to_ushort()::month_map"),
            ("src/cvcGL/GridNode.cpp", "vtkDebugLeaksManagerInstance"),
            ("src/cvcGL/GridNode.cpp", "vtkObjectFactoryRegistryCleanupInstance"),
            ("src/cvcGL/GridNode.cpp", "vtkCommonInformationKeyManagerInstance"),
            ("src/cvcGL/ImGuiOverlay.cpp", "vtkTimerLogCleanupInstance"),
            ("src/cvcGL/GridNode.cpp", "vtksys::SystemToolsManagerInstance"),
            (scan.HEADER, "igl::viridis_cm"),
            (scan.HEADER, "Eigen::internal::manage_caching_sizes()::m_cacheSizes"),
            (scan.HEADER, "pocketfft::detail::threading::max_threads"),
            (scan.HEADER, "std::__n4861::coroutine_handle<x>::_S_fr"),
        ):
            self.assertIsNotNone(r(src, name), name)
        # libcvc's own, vendored code and VTK-looking names of ours included, stay on the list.
        for src, name in (
            ("src/cvc/geometry/SDF/SignDistanceFunction/head.cpp", "SDFLibrary::size"),
            ("src/cvcGL/GridNode.cpp", "cvc::gl::(anonymous namespace)::vtkCache"),
            ("src/cvcGL/GridNode.cpp", "vtkMyOwnThing"),
            (scan.HEADER, "cvc::state_exec::nil_value"),
        ):
            self.assertIsNone(r(src, name), name)

    def test_subsystems(self):
        s = scan.subsystem
        self.assertEqual(s("cvc", "src/cvc/core/app.cpp"), "core")
        self.assertEqual(s("cvc", "src/cvc/state/state_exec/parser.cpp"), "state_exec")
        self.assertEqual(s("cvc", "src/cvc/state/state.cpp"), "state")
        self.assertEqual(
            s("cvc", "src/cvc/geometry/cvc-mesher/LBIE/octree.cpp"), "cvc-mesher (vendored)"
        )
        self.assertEqual(s("cvcGL", "src/cvcGL/Scene.cpp"), "cvcGL")
        self.assertEqual(s("cvc-cli", "src/cvc-cli/main.cpp"), "cli")
        self.assertEqual(s("pycvc", "bindings/pycvc/pycvc_context.cpp"), "pycvc")
        self.assertEqual(s("cvc", scan.HEADER), "cvc (headers)")

    def test_object_source(self):
        self.assertEqual(
            scan.object_source(
                "/b/src/cvc/CMakeFiles/cvc.dir/core/app.cpp.o",
                "/b/src/cvc/CMakeFiles/cvc.dir",
                "src/cvc",
            ),
            "src/cvc/core/app.cpp",
        )
        # CMake writes "__" for "..".
        self.assertEqual(
            scan.object_source(
                "/b/src/cvcGL/CMakeFiles/cvcGL.dir/__/cvc/third/x.cpp.o",
                "/b/src/cvcGL/CMakeFiles/cvcGL.dir",
                "src/cvcGL",
            ),
            "src/cvc/third/x.cpp",
        )


class Collect(unittest.TestCase):
    def test_findings(self):
        symbols = scan.parse_sysv(SYSV)
        findings, skipped = scan.collect(OBJECTS, symbols, NAMES)
        self.assertEqual({(f.target, f.source, f.kind, f.symbol) for f in findings}, EXPECTED)
        self.assertEqual(sum(skipped.values()), 1)

    def test_single_object_output_gets_its_header(self):
        body = SYSV.split("Symbols from", 2)[1].split("\n", 1)[1]
        symbols = scan.parse_sysv(body, default_obj="/x.o")
        self.assertTrue(symbols)
        self.assertTrue(all(s.obj == "/x.o" for s in symbols))


class Allowlist(unittest.TestCase):
    def test_parse(self):
        entries, errors = scan.parse_allowlist(
            "# comment\n"
            "\n"
            "cvc src/cvc/core/app.cpp bss cvc::g_handlers\n"
            "cvc <header> guard guard variable for std::foo<int, char>::x\n"
            "cvc src/cvc/core/app.cpp heap oops\n"
            "cvc src/cvc/core/app.cpp bss cvc::g_handlers\n"
            "too short\n"
        )
        self.assertEqual(len(entries), 2)
        self.assertEqual(entries[1].symbol, "guard variable for std::foo<int, char>::x")
        self.assertEqual(len(errors), 3)

    def test_round_trip(self):
        lines = {"cvc": [f"{t} {s} {k} {y}" for t, s, k, y in EXPECTED]}
        text = scan.render_allowlist(lines)
        entries, errors = scan.parse_allowlist(text)
        self.assertEqual(errors, [])
        self.assertEqual({(e.target, e.source, e.kind, e.symbol) for e in entries}, EXPECTED)
        # Objects with a file sort before <header> ones.
        self.assertEqual(entries[-1].source, scan.HEADER)


class Ratchet(unittest.TestCase):
    BASE = (
        "# --- cvc (2)\n"
        "cvc src/cvc/core/app.cpp bss cvc::g_handlers\n"
        "cvc src/cvc/state/state.cpp tls cvc::tl_id\n"
    )

    def test_deleting_lines_passes(self):
        new, problems = ratchet.added(self.BASE, "cvc src/cvc/state/state.cpp tls cvc::tl_id\n")
        self.assertEqual((new, problems), ([], []))
        self.assertEqual(ratchet.added(self.BASE, ""), ([], []))

    def test_an_added_line_fails(self):
        head = self.BASE + "cvc src/cvc/state/state.cpp data cvc::g_name\n"
        new, problems = ratchet.added(self.BASE, head)
        self.assertEqual(problems, [])
        self.assertEqual([e.symbol for e in new], ["cvc::g_name"])

    def test_a_moved_object_counts_as_added(self):
        # The same symbol in another file is a new line, as is a renamed one.
        head = "cvc src/cvc/core/other.cpp bss cvc::g_handlers\n"
        new, _ = ratchet.added(self.BASE, head)
        self.assertEqual([e.source for e in new], ["src/cvc/core/other.cpp"])

    def test_main(self):
        with tempfile.TemporaryDirectory() as tmp:
            base, head = os.path.join(tmp, "base.txt"), os.path.join(tmp, "head.txt")
            with open(base, "w", encoding="utf-8") as fh:
                fh.write(self.BASE)
            with open(head, "w", encoding="utf-8") as fh:
                fh.write(self.BASE.splitlines(True)[1])
            out = io.StringIO()
            with redirect_stdout(out):
                self.assertEqual(ratchet.main([base, head]), 0)
            with open(head, "a", encoding="utf-8") as fh:
                fh.write("cvc src/cvc/x.cpp bss cvc::x\n")
            with redirect_stdout(out):
                self.assertEqual(ratchet.main([base, head]), 1)
            self.assertIn("+ line 2: cvc src/cvc/x.cpp bss cvc::x", out.getvalue())
            with open(head, "a", encoding="utf-8") as fh:
                fh.write("not a line\n")
            with redirect_stdout(out), redirect_stderr(io.StringIO()):
                self.assertEqual(ratchet.main([base, head]), 2)


def _fake_runner(_nm, objs):
    return SYSV


def _fake_demangler(_cxxfilt, names):
    return {n: NAMES.get(n, n) for n in names}


class Main(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.build = os.path.join(self.tmp, "b")
        for _obj, src in OBJECTS["cvc"]:
            rel = src[len("src/cvc/") :]
            path = os.path.join(self.build, "src", "cvc", "CMakeFiles", "cvc.dir", rel + ".o")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            open(path, "wb").close()
        # The canned output names /b/...; point it at this tree.
        global SYSV
        self._sysv = SYSV
        SYSV = SYSV.replace("/b/", self.build.replace(os.sep, "/") + "/")
        self.allow = os.path.join(self.tmp, "allowlist.txt")

    def tearDown(self):
        global SYSV
        SYSV = self._sysv
        shutil.rmtree(self.tmp)

    def run_main(self, *extra):
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(out):
            rc = scan.main(
                ["--build-dir", self.build, "--targets", "cvc", "--allowlist", self.allow, *extra],
                runner=_fake_runner,
                demangler=_fake_demangler,
            )
        return rc, out.getvalue()

    def test_write_then_pass_then_new_then_stale(self):
        # Nothing allowed: every finding is new.
        rc, out = self.run_main()
        self.assertEqual(rc, 1, out)
        self.assertIn("+ cvc src/cvc/core/app.cpp bss cvc::g_handlers", out)

        # The written allowlist makes the scan pass.
        rc, out = self.run_main("--write-allowlist", self.allow)
        self.assertEqual(rc, 0, out)
        rc, out = self.run_main()
        self.assertEqual(rc, 0, out)

        # Other targets' lines are not this lane's to check.
        with open(self.allow, "a", encoding="utf-8") as fh:
            fh.write("cvcGL src/cvcGL/Scene.cpp bss cvc::gl_cache\n")
        rc, out = self.run_main()
        self.assertEqual(rc, 0, out)

        # An entry this build no longer has fails, so the list only shrinks.
        with open(self.allow, "a", encoding="utf-8") as fh:
            fh.write("cvc src/cvc/core/app.cpp bss cvc::moved_to_app\n")
        rc, out = self.run_main()
        self.assertEqual(rc, 1, out)
        self.assertIn("- line", out)
        self.assertIn("cvc::moved_to_app", out)

        # Regenerating drops it and keeps the other target's line.
        rc, out = self.run_main("--write-allowlist", self.allow)
        self.assertEqual(rc, 0, out)
        with open(self.allow, encoding="utf-8") as fh:
            text = fh.read()
        self.assertNotIn("moved_to_app", text)
        self.assertIn("cvc::gl_cache", text)

    def test_a_target_without_objects_is_an_error(self):
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(out):
            rc = scan.main(
                ["--build-dir", self.build, "--targets", "cvc,cvcGL", "--allowlist", self.allow],
                runner=_fake_runner,
                demangler=_fake_demangler,
            )
        self.assertEqual(rc, 2)
        self.assertIn("no objects for cvcGL", out.getvalue())


class EndToEnd(unittest.TestCase):
    """fixture.cpp through a real compiler, nm and c++filt."""

    def setUp(self):
        self.cxx = shlex.split(os.environ.get("SCAN_TEST_CXX", "c++"))
        self.nm = os.environ.get("SCAN_TEST_NM", "nm")
        self.cxxfilt = os.environ.get("SCAN_TEST_CXXFILT", "c++filt")
        missing = [t for t in (self.cxx[0], self.nm, self.cxxfilt) if shutil.which(t) is None]
        if missing:
            if os.environ.get("SCAN_TEST_REQUIRE_E2E") == "1":
                self.fail(f"not found: {', '.join(missing)}")
            self.skipTest(f"not found: {', '.join(missing)}")
        self.tmp = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_fixture(self):
        here = os.path.dirname(os.path.abspath(__file__))
        build = os.path.join(self.tmp, "build")
        obj = os.path.join(build, "src", "cvc", "CMakeFiles", "cvc.dir", "core", "fixture.cpp.o")
        os.makedirs(os.path.dirname(obj))
        cmd = [*self.cxx, "-std=c++17", "-O2", "-fPIC", "-c", os.path.join(here, "fixture.cpp")]
        subprocess.run([*cmd, "-o", obj], check=True)
        allow = os.path.join(self.tmp, "allowlist.txt")
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(out):
            rc = scan.main(
                ["--build-dir", build, "--targets", "cvc", "--allowlist", allow]
                + ["--nm", self.nm, "--cxxfilt", self.cxxfilt, "--write-allowlist", allow]
            )
        self.assertEqual(rc, 0, out.getvalue())
        with open(allow, encoding="utf-8") as fh:
            entries, errors = scan.parse_allowlist(fh.read())
        self.assertEqual(errors, [])
        got = {(e.source, e.kind, e.symbol) for e in entries}
        src = "src/cvc/core/fixture.cpp"
        must = {
            (src, "data", "g_counter"),
            (src, "bss", "s_internal"),
            (src, "bss", "(anonymous namespace)::anon_count"),
            (src, "bss", "fx::g_widget"),
            (src, "bss", "registry::count"),
            (src, "bss", "local_static()::n"),
            (src, "guard", "guard variable for local_static()::n"),
            (scan.HEADER, "bss", "header_static()::h"),
            (scan.HEADER, "guard", "guard variable for header_static()::h"),
            (scan.HEADER, "data", "tpl<int>::v"),
            (src, "tls", "tl_scratch"),
            (src, "tls", "tl_widget"),
            (src, "data", "g_names"),
        }
        self.assertEqual(must - got, set(), f"missing; found {sorted(got)}")
        symbols = {s for _src, _k, s in got}
        for constant in ("k_table", "k_names"):
            self.assertNotIn(constant, symbols)
        # And the allowlist it wrote passes.
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(out):
            rc = scan.main(
                ["--build-dir", build, "--targets", "cvc", "--allowlist", allow]
                + ["--nm", self.nm, "--cxxfilt", self.cxxfilt]
            )
        self.assertEqual(rc, 0, out.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
