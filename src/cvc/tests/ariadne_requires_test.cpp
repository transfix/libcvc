// requires: and has? (roadmap §7.6, §16.3): what a document needs from its host. The free-head scan
// that derives what a program calls, the capability tokens, Runtime::has / has_function, (has? ...)
// inside programs, and AppRuntime::load's preflight (a missing requirement fails the load, an
// optional one warns, an unprovided call warns).

#include <algorithm>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_info.h>
#include <cvc/ariadne/requires.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/types.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::ariadne;
namespace fs = std::filesystem;

namespace {

#define REQUIRE_YAML()                                                                             \
  do {                                                                                             \
    if (!AppRuntime::have_yaml())                                                                  \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

fs::path write_doc(const std::string &name, const std::string &text) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_requires_test";
  fs::create_directories(dir);
  const fs::path p = dir / (name + ".ari");
  std::ofstream(p) << text;
  return p;
}

bool any_contains(const std::vector<std::string> &v, const std::string &needle) {
  for (const std::string &s : v)
    if (s.find(needle) != std::string::npos)
      return true;
  return false;
}

// A backend that reports chosen capabilities. FtxuiBackend (real or the no-FTXUI stub) supplies the
// rest of the interface; nothing here draws.
class CapsBackend : public FtxuiBackend {
public:
  explicit CapsBackend(Capabilities c) : c_(c) {}
  Capabilities capabilities() const override { return c_; }

private:
  Capabilities c_;
};

} // namespace

// ---- free_heads ---------------------------------------------------------------------------------

TEST(AriadneRequires, FreeHeadsAreTheCallsAProgramMakes) {
  EXPECT_EQ(free_heads("(gl-bind-stream \"cam\" (state-get \"x\"))"),
            (std::vector<std::string>{"gl-bind-stream", "state-get"}));
  EXPECT_EQ(free_heads("(room-join \"a\") (room-join \"b\")"),
            (std::vector<std::string>{"room-join"})); // each name once, every form scanned
}

TEST(AriadneRequires, FreeHeadsSkipSpecialFormsQuotesAndBoundNames) {
  // Special forms are not calls; quoted data is never called.
  EXPECT_EQ(free_heads("(if (> 1 0) (begin (quote (not-a-call 1))) 0)"),
            (std::vector<std::string>{">"}));
  // lambda / let / for bind names; a call through one is not a host function.
  EXPECT_EQ(free_heads("(let ((f (lambda (g) (g 1)))) (f (lambda (x) x)))"),
            (std::vector<std::string>{}));
  EXPECT_EQ(free_heads("(for item (list 1 2) (item))"), (std::vector<std::string>{"list"}));
  // defun / set / defclass define names the program then calls, even before the definition.
  EXPECT_EQ(free_heads("(helper 1) (defun helper (x) (* x 2))"), (std::vector<std::string>{"*"}));
  EXPECT_EQ(free_heads("(set h (lambda () 1)) (h)"), (std::vector<std::string>{}));
  EXPECT_EQ(free_heads("(defclass Box (init (self v) (set-attr self \"v\" v))) (Box 1)"),
            (std::vector<std::string>{"set-attr"}));
  // catch binds its variable; catch / finally are not calls.
  EXPECT_EQ(free_heads("(try (risky) (catch (NameError e) (e)) (finally (cleanup)))"),
            (std::vector<std::string>{"risky", "cleanup"}));
}

TEST(AriadneRequires, FreeHeadsOfANonProgramAreEmpty) {
  EXPECT_TRUE(free_heads("").empty());
  EXPECT_TRUE(free_heads("quit").empty());        // a bare event name, not a program
  EXPECT_TRUE(free_heads("(unbalanced").empty()); // its parse error surfaces when it runs
}

// ---- capability tokens --------------------------------------------------------------------------

TEST(AriadneRequires, CapabilityTokensReadTheHostFacts) {
  EXPECT_TRUE(is_capability_token("gl.scene"));
  EXPECT_TRUE(is_capability_token("raster.bitmap"));
  EXPECT_FALSE(is_capability_token("state-get"));
  for (const CapabilityToken &t : capability_tokens())
    EXPECT_TRUE(is_capability_token(t.name)) << t.name;

  HostFacts f;
  EXPECT_FALSE(capability_holds("gl.scene", f));
  f.scene = true;
  f.caps.terminal = true;
  f.caps.kitty = true;
  EXPECT_TRUE(capability_holds("gl.scene", f));
  EXPECT_TRUE(capability_holds("term", f));
  EXPECT_TRUE(capability_holds("term.kitty", f));
  EXPECT_FALSE(capability_holds("term.sixel", f));
  EXPECT_FALSE(capability_holds("gl.shaders", f));
  EXPECT_FALSE(capability_holds("no.such.token", f));
}

// ---- Runtime::has / has_function
// -----------------------------------------------------------------

TEST(AriadneRequires, RuntimeKnowsWhatProgramsCanCall) {
  cvc::app app;
  Runtime rt(app, "ui.req");
  EXPECT_TRUE(rt.has_function("state-get"));    // a state intrinsic
  EXPECT_TRUE(rt.has_function("string.split")); // the stdlib
  EXPECT_TRUE(rt.has_function("if"));           // a special form
  EXPECT_TRUE(rt.has_function("has?"));
  EXPECT_FALSE(rt.has_function("gl-bind-stream-that-does-not-exist"));
  const std::vector<std::string> names = rt.program_names();
  EXPECT_TRUE(std::find(names.begin(), names.end(), "state-set") != names.end());

  // A provider registered later is seen without a refresh: the registry's generation moved.
  register_action_intrinsics(
      [](std::shared_ptr<cvc::state_exec::environment> env, cvc::state_exec::intrinsics_context &) {
        cvc::state_exec::builtins::register_fn(
            env, "req-test-verb",
            [](std::span<const cvc::state_exec::value_t>) { return cvc::state_exec::value_t(); });
      });
  EXPECT_TRUE(rt.has_function("req-test-verb"));
  clear_action_intrinsics();
  EXPECT_FALSE(rt.has_function("req-test-verb"));
}

TEST(AriadneRequires, HasAnswersTokensFunctionsAndWidgetTypes) {
  cvc::app app;
  Runtime rt(app, "ui.req");
  Capabilities c;
  c.terminal = true;
  CapsBackend be(c);
  rt.set_backend(&be);
  EXPECT_TRUE(rt.has("term"));
  EXPECT_FALSE(rt.has("gl.shaders"));
  EXPECT_FALSE(rt.has("gl.scene"));
  rt.set_scene_capable(true);
  EXPECT_TRUE(rt.has("gl.scene"));
  EXPECT_TRUE(rt.has("state-get"));
  EXPECT_TRUE(rt.has("slider_int")); // a built-in widget type
  EXPECT_FALSE(rt.has("no-such-thing"));
}

TEST(AriadneRequires, TheTerminalBackendIsATerminal) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  FtxuiBackend be;
  const Capabilities c = be.capabilities();
  EXPECT_TRUE(c.terminal);
  EXPECT_FALSE(c.glsl);
  EXPECT_FALSE(c.bitmap); // half-blocks until a probe (or the host) says kitty / Sixel
  TermGraphics g;
  g.kitty = true;
  be.set_terminal_graphics(g);
  EXPECT_TRUE(be.capabilities().kitty);
  EXPECT_TRUE(be.capabilities().bitmap); // Auto image mode resolves to kitty pixels
}

// ---- AppRuntime::load preflight
// ------------------------------------------------------------------

TEST(AriadneRequires, AMissingRequirementFailsTheLoadNamingEveryOne) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.req"); // no SceneAdapter: no gl.scene
  const fs::path doc = write_doc("missing", "requires: [gl.scene, state-get, frobnicate]\n"
                                            "windows:\n"
                                            "  - window: W\n"
                                            "    children:\n"
                                            "      - text: hi\n");
  try {
    art.load(doc.string());
    FAIL() << "the load should have failed";
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("gl.scene"), std::string::npos) << what;
    EXPECT_NE(what.find("frobnicate"), std::string::npos) << what;
    EXPECT_EQ(what.find("state-get"), std::string::npos) << what; // that one is provided
  }
}

TEST(AriadneRequires, AnOptionalRequirementOnlyWarns) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.req");
  const fs::path doc = write_doc("optional", "requires:\n"
                                             "  - state-get\n"
                                             "  - {name: gl.shaders, optional: true}\n"
                                             "windows:\n"
                                             "  - window: W\n"
                                             "    children:\n"
                                             "      - text: hi\n");
  std::vector<std::string> warns;
  ASSERT_NO_THROW(warns = art.load(doc.string()));
  EXPECT_TRUE(any_contains(warns, "optional requirement gl.shaders")) << warns.size();
}

TEST(AriadneRequires, ACallTheHostLacksWarnsAtLoad) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.req");
  art.register_verb("quit", [] {});
  const fs::path doc = write_doc("calls", "init: (state-set \"n\" \"1\")\n"
                                          "windows:\n"
                                          "  - window: W\n"
                                          "    children:\n"
                                          "      - button: Join\n"
                                          "        on: (room-join \"lobby\")\n"
                                          "      - button: Quit\n"
                                          "        on: (quit)\n");
  const std::vector<std::string> warns = art.load(doc.string());
  EXPECT_TRUE(any_contains(warns, "calls (room-join ...)")) << (warns.empty() ? "" : warns[0]);
  EXPECT_FALSE(any_contains(warns, "(quit ...)"));      // a registered verb
  EXPECT_FALSE(any_contains(warns, "(state-set ...)")); // an intrinsic
}

TEST(AriadneRequires, AProgramCanAskWithHas) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.req");
  const fs::path doc =
      write_doc("has", "init: |\n"
                       "  (begin\n"
                       "    (state-set \"scene\" (if (has? \"gl.scene\") \"yes\" \"no\"))\n"
                       "    (state-set \"fn\" (if (has? \"state-get\") \"yes\" \"no\"))\n"
                       "    (state-set \"none\" (if (has? 'no-such-fn) \"yes\" \"no\")))\n"
                       "windows:\n"
                       "  - window: W\n"
                       "    children:\n"
                       "      - text: hi\n");
  ASSERT_NO_THROW(art.load(doc.string()));
  EXPECT_EQ(val(app, "ui.req.scene"), "no"); // no SceneAdapter here
  EXPECT_EQ(val(app, "ui.req.fn"), "yes");
  EXPECT_EQ(val(app, "ui.req.none"), "no");
}

TEST(AriadneRequires, AReactiveExpressionCanAskWithHas) {
  REQUIRE_YAML();
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.req"); // no SceneAdapter: (has? "gl.scene") is false
  art.set_backend(be);
  const fs::path doc = write_doc("reactive", "windows:\n"
                                             "  - window: W\n"
                                             "    children:\n"
                                             "      - text: scene-panel\n"
                                             "        visible_when: (has? \"gl.scene\")\n"
                                             "      - text: terminal-note\n"
                                             "        visible_when: (has? \"term\")\n");
  ASSERT_NO_THROW(art.load(doc.string()));
  art.drain();
  const std::string screen = be.render_to_string(art.runtime(), 60, 10);
  EXPECT_EQ(screen.find("scene-panel"), std::string::npos) << screen;
  EXPECT_NE(screen.find("terminal-note"), std::string::npos) << screen;
  EXPECT_TRUE(art.take_warnings().empty());
}

// ---- the host report (ari info / arigl info) ----------------------------------------------------

TEST(AriadneRequires, TheHostReportListsCapabilitiesAndFunctions) {
  cvc::app app;
  Runtime rt(app, "ui.req");
  Capabilities c;
  c.terminal = true;
  c.kitty = true;
  CapsBackend be(c);
  rt.set_backend(&be);
  HostInfo info = gather_host_info(rt, "ari");
  info.backend = "ftxui";
  EXPECT_EQ(info.runtime, "ari");
  EXPECT_FALSE(info.libcvc_version.empty());
  EXPECT_TRUE(info.graphics.kitty);
  EXPECT_EQ(info.capabilities.size(), capability_tokens().size());
  EXPECT_TRUE(std::find(info.functions.begin(), info.functions.end(), "state-get") !=
              info.functions.end());

  const std::string text = format_host_info(info, /*json=*/false, /*functions=*/false);
  EXPECT_NE(text.find("ari (libcvc "), std::string::npos) << text;
  EXPECT_NE(text.find("term.kitty      yes"), std::string::npos) << text;
  EXPECT_NE(text.find("gl.scene        no"), std::string::npos) << text;
  EXPECT_NE(text.find("functions programs can call: "), std::string::npos) << text;
  EXPECT_EQ(text.find("  state-get\n"), std::string::npos); // listed only with functions=true
  EXPECT_NE(format_host_info(info, false, true).find("  state-get\n"), std::string::npos);

  const std::string json = format_host_info(info, /*json=*/true, /*functions=*/false);
  EXPECT_NE(json.find("\"runtime\": \"ari\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"term.kitty\": true"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gl.scene\": false"), std::string::npos) << json;
  EXPECT_NE(json.find("\"state-get\""), std::string::npos) << json;
}
