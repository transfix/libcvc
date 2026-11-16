// The Python engine (src/ariadne_python/python_engine.h) under one AppRuntime, with no host around
// it, for what only a live reload shows: a document whose python: block has console: true, then,
// in the same runtime, one with no python: block. The engine's unload puts back the console
// natives that refuse, so no cell runs on the first load's consoles, and they leave linecache.
// And two documents on one app, each with its own runtime: each one's cells and exports are its
// own. Each case boots an interpreter of its own, so each is a ctest entry of its own.

#include "python_engine.h"

#include <cvc/ariadne/app_runtime.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace ari = cvc::ariadne;
namespace fs = std::filesystem;

namespace {

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

fs::path test_file(const std::string &name) {
  const fs::path dir = fs::temp_directory_path() / "arix_engine_test";
  fs::create_directories(dir);
  return dir / name;
}

std::string write_doc(const std::string &name, const std::string &text) {
  const fs::path p = test_file(name + ".ari");
  std::ofstream(p) << text;
  return p.string();
}

// Run `program` to its end: how it ended ("" when it finished).
std::string run(ari::AppRuntime &rt, const std::string &program) {
  bool ended = false;
  std::string how = "did not end";
  rt.run_program(program, [&](const std::string &h) {
    ended = true;
    how = h;
  });
  for (int i = 0; i < 500 && !ended; ++i)
    rt.drain();
  return how;
}

std::string joined(const std::vector<std::string> &v) {
  std::string out;
  for (const std::string &s : v)
    out += s + "\n";
  return out;
}

} // namespace

TEST(ArixEngine, AReloadIntoADocumentWithNoPythonBlockTakesTheConsoleAway) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  ari::python::Engine python({"arix", {}});
  ASSERT_TRUE(python.ok()) << python.error();
  cvc::app app;
  ari::AppRuntime rt(app, "ui.t");
  ari::python::HostHandles handles;
  handles.app = &app;
  handles.runtime = &rt;
  handles.mode = "headless";
  const ari::python::Attachment lent = python.attach(handles);

  std::vector<std::string> warns = rt.load(write_doc("with_console", R"(meta: {min_libcvc: 3.7.13}
requires: [python]
python:
  console: true
  source: x = 41
root:
  - text: a console
)"));
  EXPECT_TRUE(warns.empty()) << joined(warns);
  EXPECT_EQ(run(rt, R"((state-set "r" (get-attr (python-console "c" "x + 1") "output")))"), "");
  EXPECT_EQ(val(app, "ui.t.r"), "42\n") << "a cell, in the block's namespace";

  warns =
      rt.load(write_doc("no_python", "meta: {min_libcvc: 3.7.13}\nroot:\n  - text: no python\n"));
  EXPECT_TRUE(warns.empty()) << joined(warns);
  EXPECT_EQ(run(rt, R"((begin
    (state-set "r" (try (python-console "c" "x") (catch (PermissionError e) "refused")))
    (state-set "k" (try (python-console-clear "c") (catch (PermissionError e) "refused")))))"),
            "");
  EXPECT_EQ(val(app, "ui.t.r"), "refused") << "no cell runs on the first load's consoles";
  EXPECT_EQ(val(app, "ui.t.k"), "refused");

  // The block of a third load sees linecache before its own consoles have a cell: the first
  // load's cells went when the reload let go of its consoles.
  warns = rt.load(write_doc("console_again", R"(meta: {min_libcvc: 3.7.13}
requires: [python]
python:
  console: true
  source: |
    import linecache
    left = [f for f in linecache.cache if f.startswith("<console ")]
root:
  - text: a console again
)"));
  EXPECT_TRUE(warns.empty()) << joined(warns);
  EXPECT_EQ(run(rt, R"((state-set "r" (get-attr (python-console "c" "left") "output")))"), "");
  EXPECT_EQ(val(app, "ui.t.r"), "[]\n") << "the first load's cells left linecache";
}

// Two documents on one app, each in a runtime of its own that the engine is attached to, and both
// loaded before either runs. The same names are each document's own: the console natives run
// cells in the document's own namespace, and a function its block exports (with pycvc, which
// export: needs) is its own.
TEST(ArixEngine, TwoDocumentsOnOneAppEachRunTheirOwnPython) {
  if (!ari::AppRuntime::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  std::vector<std::string> path;
#ifdef ARIX_ENGINE_TEST_PYCVC
  path = {ARIX_ENGINE_TEST_PYCVC_PKG, ARIX_ENGINE_TEST_PYCVC};
#endif
  ari::python::Engine python({"arix", path});
  ASSERT_TRUE(python.ok()) << python.error();
  const fs::path probe = test_file("probe_pycvc.py");
  std::ofstream(probe) << "import sys\ntry:\n    import pycvc\nexcept ImportError:\n"
                          "    sys.exit(3)\n";
  const bool pycvc = python.run_script(probe.string(), {}) == 0;
  const auto doc = [pycvc](int n) {
    return "meta: {min_libcvc: 3.7.13}\n"
           "requires: [python]\n"
           "python:\n"
           "  console: true\n"
           "  source: |\n"
           "    n = " +
           std::to_string(n) +
           "\n"
           "    def which():\n"
           "        return n\n" +
           std::string(pycvc ? "  export: [which]\n" : "") + "root:\n  - text: python\n";
  };

  cvc::app app;
  ari::AppRuntime first(app, "ui.p1");
  ari::AppRuntime second(app, "ui.p2");
  ari::python::HostHandles handles;
  handles.app = &app;
  handles.mode = "headless";
  handles.runtime = &first;
  const ari::python::Attachment lent_first = python.attach(handles);
  std::vector<std::string> warns = first.load(write_doc("first", doc(1)));
  EXPECT_TRUE(warns.empty()) << joined(warns);
  handles.runtime = &second;
  const ari::python::Attachment lent_second = python.attach(handles);
  warns = second.load(write_doc("second", doc(2)));
  EXPECT_TRUE(warns.empty()) << joined(warns);

  const char cell[] = R"((state-set "r" (get-attr (python-console "c" "n") "output")))";
  EXPECT_EQ(run(first, cell), "");
  EXPECT_EQ(run(second, cell), "");
  EXPECT_EQ(val(app, "ui.p1.r"), "1\n") << "the first document's cell runs in its own namespace";
  EXPECT_EQ(val(app, "ui.p2.r"), "2\n");
  if (!pycvc)
    GTEST_SKIP() << "the export: half needs pycvc, which this interpreter can't import";
  EXPECT_EQ(run(first, R"((state-set "w" (which)))"), "");
  EXPECT_EQ(run(second, R"((state-set "w" (which)))"), "");
  EXPECT_EQ(val(app, "ui.p1.w"), "1") << "the first document's export is its own";
  EXPECT_EQ(val(app, "ui.p2.w"), "2");
}
