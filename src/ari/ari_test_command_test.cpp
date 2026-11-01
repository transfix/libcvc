// ari's testing features end to end (docs/ARIADNE.md "Testing a document"): (quit code) as the
// exit status of check and snapshot, and `ari test` over passing, failing, quitting, timing-out
// and networked documents, in its line and TAP forms; and `ari run` reporting a warning while the
// document runs. The ari binary is run as a launcher would.

#include "../cvc/tests/local_http_server.h"
#include "ari_run.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <string>

namespace fs = std::filesystem;
using ari_test::ari;
using ari_test::AriRun;
using ari_test::quoted;
using ari_test::TempDir;
using cvc_test::LocalHttpServer;

namespace {

// A document with `extra` top-level blocks after its meta and root.
std::string doc(const std::string &extra) {
  return "meta: {min_libcvc: 3.7.0}\nroot:\n  - text: hi\n" + extra;
}

bool have_terminal_ari() {
  const AriRun r = ari("info");
  return r.status == 0 && r.output.find("terminal backend (FTXUI)      yes") != std::string::npos &&
         r.output.find(".ari loader (yaml-cpp)        yes") != std::string::npos;
}

#define REQUIRE_ARI()                                                                              \
  do {                                                                                             \
    if (!have_terminal_ari())                                                                      \
      GTEST_SKIP() << "ari lacks FTXUI or yaml-cpp";                                               \
  } while (0)

bool has(const AriRun &r, const std::string &text) {
  return r.output.find(text) != std::string::npos;
}

} // namespace

TEST(AriQuit, CheckAndSnapshotExitWithTheCode) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path three = dir.write("three.ari", doc("init: (quit 3)\n"));
  const fs::path zero = dir.write("zero.ari", doc("init: (quit)\n"));
  for (const char *cmd : {"check", "snapshot"}) {
    SCOPED_TRACE(cmd);
    EXPECT_EQ(ari(std::string(cmd) + " " + quoted(three)).status, 3);
    EXPECT_EQ(ari(std::string(cmd) + " " + quoted(zero)).status, 0);
  }
  // A quit from a later frame ends the snapshot on that frame, with its code.
  const fs::path later =
      dir.write("later.ari", doc("init: (state-set \"n\" 0)\n"
                                 "on_tick: |\n"
                                 "  (state-set \"n\" (+ 1 (int (state-get \"n\"))))\n"
                                 "  (if (= (int (state-get \"n\")) 2) (quit 5))\n"));
  EXPECT_EQ(ari("snapshot --frames 50 " + quoted(later)).status, 5);
  EXPECT_EQ(ari("info --json").status, 0); // info registers (quit) too and still exits 0
}

TEST(AriTest, PassingDocumentsAndTheSummary) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path a = dir.write(
      "a.ari", doc("init: (state-set \"x\" \"2\")\n"
                   "tests:\n"
                   "  reads_init: (assert (= (state-get \"x\") \"2\") \"init ran first\")\n"
                   "  catches: |\n"
                   "    (assert (= (try (assert false \"m\") (catch (AssertionError e) 1)) 1))\n"
                   "  waits: (sleep 0.02)\n"));
  const fs::path b = dir.write("b.ari", doc("on_tick: (quit 0)\n"));
  const AriRun r = ari("test " + quoted(a) + " " + quoted(b));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_TRUE(has(r, "ok   " + a.string() + " (3 tests, ")) << r.output;
  EXPECT_TRUE(has(r, "ok   " + b.string() + " (")) << r.output;
  EXPECT_TRUE(has(r, "ari test: 2 documents, 0 failed")) << r.output;
}

TEST(AriTest, EachWayToFail) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path asserts =
      dir.write("asserts.ari", doc("tests:\n"
                                   "  good: (assert true)\n"
                                   "  bad: (assert (= (+ 1 1) 3) \"one and one\")\n"
                                   "  raises: (error \"boom\")\n"
                                   "  after: (assert true)\n"));
  const AriRun r = ari("test " + quoted(asserts));
  EXPECT_EQ(r.status, 1) << r.output;
  EXPECT_TRUE(has(r, "FAIL " + asserts.string() + " (4 tests, ")) << r.output; // all four ran
  EXPECT_TRUE(has(r, "test 'bad' failed: AssertionError: assertion failed: one and one "
                     "(at (assert (= (+ 1 1) 3) \"one and one\"))"))
      << r.output;
  EXPECT_TRUE(has(r, "test 'raises' failed: RuntimeError: boom")) << r.output;
  EXPECT_TRUE(has(r, "ari test: 1 document, 1 failed")) << r.output;

  struct Case {
    const char *name;
    std::string text;
    const char *says;
  };
  const Case cases[] = {
      {"quits", doc("init: (quit 4)\n"), "(quit 4)"},
      {"tick", doc("on_tick: (assert (= 1 2) \"tick\")\n"),
       "on_tick: resident [ari#"}, // ... failed: AssertionError: assertion failed: tick
      {"init", doc("init: (assert false \"seed\")\n"),
       "load warning: init: script failed: AssertionError: assertion failed: seed"},
      {"warns", "root:\n  - text: hi\non_tick: (quit 0)\n", "load warning: no meta.min_libcvc"},
      {"broken", "root: [ unclosed\n", "load failed: YAML parse error"},
      {"quit_early", doc("tests:\n  one: (quit 0)\n  two: (assert true)\n"),
       "quit with 1 of 2 tests not finished"},
  };
  for (const Case &c : cases) {
    SCOPED_TRACE(c.name);
    const fs::path p = dir.write(std::string(c.name) + ".ari", c.text);
    const AriRun f = ari("test " + quoted(p));
    EXPECT_EQ(f.status, 1) << f.output;
    EXPECT_TRUE(has(f, "FAIL " + p.string())) << f.output;
    EXPECT_TRUE(has(f, c.says)) << f.output;
  }
  const AriRun missing = ari("test " + quoted(dir.path / "nope.ari"));
  EXPECT_EQ(missing.status, 1) << missing.output;
  EXPECT_TRUE(has(missing, "load failed: cannot open")) << missing.output;
}

TEST(AriTest, FramesAndTimeout) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path idle = dir.write("idle.ari", doc(""));
  const AriRun timed = ari("test --timeout 0.5 " + quoted(idle));
  EXPECT_EQ(timed.status, 1) << timed.output;
  EXPECT_TRUE(has(timed, "timed out after 0.5 s (a document with no tests: block ends with (quit "
                         "code), or pass --frames N)"))
      << timed.output;
  const AriRun framed = ari("test --frames 3 " + quoted(idle));
  EXPECT_EQ(framed.status, 0) << framed.output;
  const fs::path stuck =
      dir.write("stuck.ari", doc("tests:\n  waits: (set v (msg-recv \"never\"))\n"));
  const AriRun hung = ari("test --timeout 1 " + quoted(stuck));
  EXPECT_EQ(hung.status, 1) << hung.output;
  EXPECT_TRUE(has(hung, "timed out after 1 s in test 'waits'")) << hung.output;
}

TEST(AriTest, TapAndUsage) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path good = dir.write("good.ari", doc("tests:\n  t: (assert true)\n"));
  const fs::path bad = dir.write("bad.ari", doc("tests:\n  t: (assert false \"no\")\n"));
  const AriRun r = ari("test --tap " + quoted(good) + " " + quoted(bad));
  EXPECT_EQ(r.status, 1) << r.output;
  EXPECT_TRUE(has(r, "TAP version 13\n1..2\n")) << r.output;
  EXPECT_TRUE(has(r, "ok 1 - " + good.string() + " # 1 test, ")) << r.output;
  EXPECT_TRUE(has(r, "not ok 2 - " + bad.string() + " # 1 test, ")) << r.output;
  EXPECT_TRUE(has(r, "# test 't' failed: AssertionError: assertion failed: no")) << r.output;
  EXPECT_FALSE(has(r, "ari test: ")) << r.output; // no summary line in TAP

  EXPECT_EQ(ari("test " + quoted(good) + " --tap").status, 2); // flags go first
  EXPECT_EQ(ari("check --tap " + quoted(good)).status, 2);
  EXPECT_EQ(ari("snapshot --timeout 3 " + quoted(good)).status, 2);
  EXPECT_TRUE(has(ari("--help"), "ari test [flags]"));
}

// The network is as snapshot's: off without the flags; with them, a test's request is waited for.
TEST(AriTest, NetworkRulesAreSnapshots) {
  REQUIRE_ARI();
  if (!has(ari("info"), "HTTP (cvc::net)               yes"))
    GTEST_SKIP() << "ari has no HTTP backend";
  LocalHttpServer api([](const LocalHttpServer::Request &req) {
    return req.target == "/ping" ? LocalHttpServer::Reply{200, {}, "pong"}
                                 : LocalHttpServer::Reply{404, {}, ""};
  });
  TempDir dir;
  const fs::path p = dir.write(
      "net.ari", doc("requires: [net]\n"
                     "tests:\n"
                     "  ping: |\n"
                     "    (set r (http-get \"" +
                     api.url("/ping") +
                     "\"))\n"
                     "    (assert (= (get-attr r \"status\") 200) (get-attr r \"error\"))\n"
                     "    (assert (= (bytes.decode (get-attr r \"body\")) \"pong\"))\n"));
  const AriRun off = ari("test " + quoted(p));
  EXPECT_EQ(off.status, 1) << off.output;
  EXPECT_TRUE(has(off, "does not provide what the document requires: net")) << off.output;
  const AriRun on = ari("test --net-allow " + api.origin() + " " + quoted(p));
  EXPECT_EQ(on.status, 0) << on.output;
  EXPECT_TRUE(has(on, "ok   " + p.string() + " (1 test, ")) << on.output;
}

// §6.6 functions under ari test: a library's functions imported and tested directly, and a module
// that calls what its mount granted.
TEST(AriTest, FunctionsImportedAndGranted) {
  REQUIRE_ARI();
  TempDir dir;
  dir.write("semver.ari", "functions: |\n"
                          "  (defun semver-part (v i) (int (nth (string.split v \".\") i)))\n"
                          "  (defun semver-major (v) (semver-part v 0))\n");
  dir.write("badge.ari", "init: (state-set \"major\" (str (semver-major (state-get \"v\"))))\n"
                         "root:\n  - text: badge\n");
  const std::string mount = "  - load: badge.ari\n    as: badge\n";
  const fs::path p = dir.write("semver_test.ari",
                               "meta: {min_libcvc: 3.7.0}\n"
                               "import: semver.ari\n"
                               "init: (state-set \"includes.badge.v\" \"3.7.5\")\n"
                               "root:\n" +
                                   mount + "    functions: [semver-major]\n" +
                                   "tests:\n"
                                   "  major: (assert (= (semver-major \"3.7.5\") 3))\n"
                                   "  part: (assert (= (semver-part \"3.7.5\" 2) 5))\n"
                                   "  module: |\n"
                                   "    (assert (= (state-get \"includes.badge.major\") \"3\")\n"
                                   "            (state-get \"includes.badge.major\"))\n");
  const AriRun r = ari("test " + quoted(p));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_TRUE(has(r, "ok   " + p.string() + " (3 tests, ")) << r.output;

  // Without the grant the module's init: cannot call it, and the load says so.
  const fs::path q = dir.write("ungranted.ari", "meta: {min_libcvc: 3.7.0}\n"
                                                "import: semver.ari\n"
                                                "on_tick: (quit 0)\n"
                                                "root:\n" +
                                                    mount);
  const AriRun u = ari("test " + quoted(q));
  EXPECT_EQ(u.status, 1) << u.output;
  EXPECT_TRUE(has(u, "calls (semver-major ...), which this host does not provide")) << u.output;
}

// §6.3 a module's on_tick: under ari test: it runs at the mount, so a document testing the module
// sees it tick, and a failing one fails the document, named by its mount.
TEST(AriTest, AModulesOnTickRuns) {
  REQUIRE_ARI();
  TempDir dir;
  dir.write("poller.ari", "init: (state-set \"n\" 0)\n"
                          "on_tick: (state-set \"n\" (+ 1 (int (state-get \"n\"))))\n"
                          "root:\n  - text: poller\n");
  const fs::path p = dir.write("poller_test.ari",
                               "meta: {min_libcvc: 3.7.0}\n"
                               "root:\n  - load: poller.ari\n    as: p\n"
                               "tests:\n"
                               "  ticks: |\n"
                               "    (while (< (int (state-get \"includes.p.n\")) 3) (sleep 0.01))\n"
                               "    (assert (not (state-exists \"n\")) \"the parent's own n\")\n");
  const AriRun r = ari("test " + quoted(p));
  EXPECT_EQ(r.status, 0) << r.output;
  EXPECT_TRUE(has(r, "ok   " + p.string() + " (1 test, ")) << r.output;

  dir.write("broken.ari", "on_tick: (assert (= 1 2))\nroot:\n  - text: broken\n");
  const fs::path q = dir.write("broken_test.ari", "meta: {min_libcvc: 3.7.0}\n"
                                                  "root:\n  - load: broken.ari\n    as: b\n");
  const AriRun f = ari("test --frames 50 " + quoted(q));
  EXPECT_EQ(f.status, 1) << f.output;
  EXPECT_TRUE(has(f, "on_tick: of mount 'ui.docs.")) << f.output;
  EXPECT_TRUE(has(f, ".includes.b' resident")) << f.output;
}

#ifndef _WIN32
// docs/ARIADNE.md §2.4: `ari run` reports a warning while the document runs. Its on_tick: fails
// once (and is restarted); the document never quits. While ari is still running the warning is
// already in --log FILE and on stderr (a file here, not the terminal), and FTXUI's screen shows it
// on the bottom line. A SIGTERM ends the run; the exit summary then counts it.
TEST(AriRun, WarningsWhileTheDocumentRuns) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path d = dir.write("w.ari", doc("init: (state-set \"n\" 0)\n"
                                            "on_tick: |\n"
                                            "  (state-set \"n\" (+ 1 (int (state-get \"n\"))))\n"
                                            "  (if (= (int (state-get \"n\")) 3)\n"
                                            "      (raise (ValueError \"boom\")))\n"));
  const fs::path log = dir.path / "w.log", out = dir.path / "out.txt", err = dir.path / "err.txt";
  // Start ari, wait (at most 20 s) for the warning in the log, then see that ari is still running.
  const std::string script =
      "\"" + std::string(std::getenv("ARI_BINARY")) + "\" run --log " + quoted(log) + " " +
      quoted(d) + " </dev/null >" + quoted(out) + " 2>" + quoted(err) +
      " & pid=$!; i=0; until grep -q 'ValueError: boom' " + quoted(log) +
      " 2>/dev/null || [ $i -ge 400 ]; do i=$((i+1)); sleep 0.05; done; "
      "kill -0 $pid && echo still-running; kill -TERM $pid; wait $pid; echo \"exit $?\"";
  const std::string sh = "sh -c '" + [&] {
    std::string q;
    for (char c : script)
      q += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return q;
  }() + "'";
  FILE *fp = popen(sh.c_str(), "r");
  ASSERT_NE(fp, nullptr);
  std::string said;
  char buf[256];
  while (std::fgets(buf, sizeof buf, fp))
    said += buf;
  pclose(fp);
  const auto slurp = [](const fs::path &p) {
    std::ifstream in(p, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  };
  const std::string logged = slurp(log), stderr_text = slurp(err), screen = slurp(out);
  EXPECT_NE(said.find("still-running"), std::string::npos) << said << stderr_text;
  EXPECT_NE(said.find("exit 143"), std::string::npos) << said; // 128 + SIGTERM
  const std::string line = "ari: warning: on_tick: resident [";
  EXPECT_NE(logged.find(line), std::string::npos) << logged;
  EXPECT_NE(logged.find("failed: ValueError: boom"), std::string::npos) << logged;
  EXPECT_NE(stderr_text.find(line), std::string::npos) << stderr_text;
  EXPECT_NE(stderr_text.find("ari: 1 warning while the document ran (above)"), std::string::npos)
      << stderr_text;
  EXPECT_NE(screen.find(line), std::string::npos); // the notice line
}
#endif
