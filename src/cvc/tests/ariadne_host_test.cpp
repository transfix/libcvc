// What a runtime host (ari, arigl, arisrv) relies on: the shared command line (host_cli.h), a
// document's own arguments (doc_args.h: the argv list, the args: block, their read-only state and
// the mount grant), and AppRuntime's host verbs and init: diagnostics.

#include <chrono>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/doc_args.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/host_cli.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace se = cvc::state_exec;
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

// The DSL value parked on a node's data channel (what state-data-set / publish_document_args put).
se::value_t data_of(cvc::app &app, const std::string &path) {
  boost::any d = cvc::state::instance(app)(path).data();
  if (const se::value_t *v = boost::any_cast<se::value_t>(&d))
    return *v;
  return se::value_t();
}

// A fresh directory per test, holding the documents it writes.
fs::path test_dir(const std::string &name) {
  const fs::path dir = fs::temp_directory_path() / ("ariadne_host_test_" + name);
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

fs::path write(const fs::path &dir, const std::string &file, const std::string &text) {
  const fs::path p = dir / file;
  std::ofstream(p) << text;
  return p;
}

const char *kConfDoc = "meta:\n"
                       "  name: conf\n"
                       "  description: Join a call.\n"
                       "  min_libcvc: 3.4.0\n"
                       "args:\n"
                       "  room:   {type: string, default: lobby, help: Room to join}\n"
                       "  camera: {type: bool, help: Use the real camera}\n"
                       "  peers:  {type: int, default: 8}\n"
                       "  scale:  float\n"
                       "  file:   {type: string, positional: true}\n"
                       "init: |\n"
                       "  (state-set \"got.room\" (state-get \"args.room\"))\n"
                       "  (state-set \"got.argc\" (str (length (state-data-get \"argv\"))))\n"
                       "root:\n"
                       "  - text: hello\n";

std::vector<ArgSpec> conf_spec() {
  LoadResult lr = load_string(kConfDoc);
  EXPECT_TRUE(lr.ok) << lr.error;
  return lr.args;
}

std::string value_of(const DocumentArgs &a, const std::string &name) {
  for (const auto &kv : a.values)
    if (kv.first == name)
      return kv.second;
  return "<absent>";
}

} // namespace

// ── the shared command line ─────────────────────────────────────────────────────────────────────

TEST(AriadneHostCli, RuntimeFlagsComeBeforeTheDocumentAndTheRestIsItsOwn) {
  const char *argv[] = {"ari", "run",     "--fps", "30", "--component-path",
                        "/c",  "doc.ari", "--fps", "5",  "x"};
  const HostOptions o = parse_host_args(10, argv);
  EXPECT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.command, "run");
  EXPECT_EQ(o.fps, 30); // the one before the document
  ASSERT_EQ(o.component_paths.size(), 1u);
  EXPECT_EQ(o.doc, "doc.ari");
  EXPECT_EQ(o.doc_args, (std::vector<std::string>{"--fps", "5", "x"}));
  EXPECT_NE(misplaced_flag_note("ari", o.doc_args).find("'--fps'"), std::string::npos);
}

TEST(AriadneHostCli, TimeoutIsSecondsAboveZero) {
  const char *ok[] = {"ari", "test", "--timeout", "2.5", "a.ari", "b.ari"};
  HostOptions o = parse_host_args(6, ok, {"--tap"});
  EXPECT_TRUE(o.error.empty()) << o.error;
  EXPECT_DOUBLE_EQ(o.timeout, 2.5);
  EXPECT_EQ(o.doc, "a.ari");
  EXPECT_EQ(o.doc_args, (std::vector<std::string>{"b.ari"})); // ari test reads them as documents
  for (const char *bad : {"0", "-1", "soon", "2s"}) {
    const char *argv[] = {"ari", "test", "--timeout", bad, "a.ari"};
    EXPECT_NE(parse_host_args(5, argv).error.find("--timeout needs a number of seconds"),
              std::string::npos)
        << bad;
  }
}

TEST(AriadneHostCli, InfoTakesNoDocument) {
  const char *info[] = {"ari", "info", "--json"};
  const HostOptions o = parse_host_args(3, info, {"--json"});
  EXPECT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.command, "info");
  EXPECT_TRUE(o.doc.empty());
  const char *run[] = {"ari", "run"};
  EXPECT_EQ(parse_host_args(2, run).error, "no document given"); // the others still need one
}

TEST(AriadneHostCli, ExtraFlagsEqualsFormsAndErrors) {
  const char *a1[] = {"arigl", "snapshot", "--terminal", "--width=80", "--ansi", "d.ari"};
  HostOptions o = parse_host_args(6, a1, {"--terminal"});
  EXPECT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.extra_flags, (std::vector<std::string>{"--terminal"}));
  EXPECT_EQ(o.width, 80);
  EXPECT_TRUE(o.ansi);

  const char *a2[] = {"ari", "run", "--room", "x", "d.ari"};
  o = parse_host_args(5, a2);
  EXPECT_NE(o.error.find("unknown option '--room'"), std::string::npos) << o.error;

  const char *a3[] = {"ari", "run", "--fps", "fast", "d.ari"};
  EXPECT_FALSE(parse_host_args(5, a3).error.empty());

  const char *a4[] = {"ari", "run"};
  EXPECT_FALSE(parse_host_args(2, a4).error.empty()); // no document

  const char *a5[] = {"ari", "run", "--", "-odd.ari", "--x"};
  o = parse_host_args(5, a5);
  EXPECT_EQ(o.doc, "-odd.ari");
  EXPECT_EQ(o.doc_args, (std::vector<std::string>{"--x"}));

  const char *a6[] = {"ari", "--help"};
  EXPECT_TRUE(parse_host_args(2, a6).help);
}

TEST(AriadneHostCli, NetworkFlagsTurnItOnAndCheckTheirOrigins) {
  const char *none[] = {"ari", "run", "d.ari"};
  EXPECT_FALSE(parse_host_args(3, none).net.enabled) << "off by default";

  const char *flags[] = {"ari",
                         "run",
                         "--net-allow",
                         "https://tx.wtf",
                         "--net-allow=http://127.0.0.1:8080",
                         "--net-policy",
                         "nicotina.policy",
                         "--credential-store",
                         "nicotina",
                         "main.ari",
                         "--net-allow",
                         "https://evil.example"};
  HostOptions o = parse_host_args(12, flags);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.net.enabled);
  EXPECT_EQ(o.net.allow, (std::vector<std::string>{"https://tx.wtf", "http://127.0.0.1:8080"}));
  EXPECT_EQ(o.net.policy_files, (std::vector<std::string>{"nicotina.policy"}));
  EXPECT_EQ(o.net.credential_store, "nicotina");
  EXPECT_EQ(o.doc, "main.ari");
  EXPECT_EQ(o.doc_args, (std::vector<std::string>{"--net-allow", "https://evil.example"}))
      << "after the document they are the document's";
  EXPECT_NE(misplaced_flag_note("ari", o.doc_args).find("'--net-allow'"), std::string::npos);

  const char *bare[] = {"arigl", "run", "--net", "d.ari"};
  o = parse_host_args(4, bare);
  EXPECT_TRUE(o.net.enabled);
  EXPECT_TRUE(o.net.allow.empty());
  const char *store_only[] = {"ari", "run", "--credential-store", "/tmp/c", "d.ari"};
  EXPECT_TRUE(parse_host_args(5, store_only).net.enabled) << "a credential store implies --net";

  const char *url[] = {"ari", "run", "--net-allow", "https://tx.wtf/api", "d.ari"};
  EXPECT_NE(parse_host_args(5, url).error.find("--net-allow: 'https://tx.wtf/api' is a URL"),
            std::string::npos);
  const char *creds[] = {"ari", "run", "--net-allow", "https://me:pw@tx.wtf", "d.ari"};
  EXPECT_NE(parse_host_args(5, creds).error.find("user info"), std::string::npos);
  const char *missing[] = {"ari", "run", "--net-allow"};
  EXPECT_NE(parse_host_args(3, missing).error.find("--net-allow needs a value"), std::string::npos)
      << parse_host_args(3, missing).error;
  const char *valued[] = {"ari", "run", "--net=yes", "d.ari"};
  EXPECT_NE(parse_host_args(4, valued).error.find("--net takes no value"), std::string::npos)
      << parse_host_args(4, valued).error;
  const char *guess[] = {"ari", "run", "--net-al", "https://tx.wtf", "d.ari"};
  EXPECT_NE(parse_host_args(5, guess).error.find("unknown option '--net-al'"), std::string::npos)
      << "no guessing at an abbreviated flag: " << parse_host_args(5, guess).error;
  EXPECT_NE(host_flags_help().find("--net-allow ORIGIN"), std::string::npos);

  const char *env[] = {
      "ari",    "run",  "--credential-env", "cvcpkg=CVCPKG_TOKEN", "--credential-env",
      "ci=X_1", "d.ari"};
  o = parse_host_args(7, env);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.net.enabled) << "--credential-env implies --net";
  ASSERT_EQ(o.net.credential_env.size(), 2u);
  EXPECT_EQ(o.net.credential_env[0],
            (std::pair<std::string, std::string>{"cvcpkg", "CVCPKG_TOKEN"}));
  EXPECT_EQ(o.net.credential_env[1].second, "X_1");
  for (const char *bad :
       {"CVCPKG_TOKEN", "cvcpkg=", "=CVCPKG_TOKEN", "bad name=V", "c=1X", "c=X-Y"}) {
    const char *argv[] = {"ari", "run", "--credential-env", bad, "d.ari"};
    EXPECT_NE(parse_host_args(5, argv).error.find("--credential-env is NAME=VAR"),
              std::string::npos)
        << bad;
  }
}

// ── parsing a document's arguments ──────────────────────────────────────────────────────────────

TEST(AriadneHostCli, GraphicsImageAndCellSizeFlags) {
  const char *a1[] = {"ari",           "snapshot",           "--graphics",  "kitty", "--image",
                      "photo=a=b.png", "--image=logo=l.png", "--cell-size", "9x17",  "doc.ari"};
  HostOptions o = parse_host_args(10, a1);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.graphics, "kitty");
  ASSERT_EQ(o.images.size(), 2u);
  EXPECT_EQ(o.images[0].first, "photo");
  EXPECT_EQ(o.images[0].second, "a=b.png") << "the name ends at the first '='";
  EXPECT_EQ(o.images[1].first, "logo");
  EXPECT_EQ(o.cell_w, 9);
  EXPECT_EQ(o.cell_h, 17);
  EXPECT_EQ(o.doc, "doc.ari");

  const char *iterm[] = {"ari", "run", "--graphics", "iterm", "d.ari"};
  EXPECT_EQ(parse_host_args(5, iterm).graphics, "iterm");
  const char *bad_mode[] = {"ari", "run", "--graphics", "blocks", "d.ari"};
  EXPECT_NE(parse_host_args(5, bad_mode).error.find("--graphics"), std::string::npos);
  const char *bad_image[] = {"ari", "run", "--image", "photo.png", "d.ari"};
  EXPECT_NE(parse_host_args(5, bad_image).error.find("NAME=FILE"), std::string::npos);
  const char *bad_cell[] = {"ari", "snapshot", "--cell-size", "9", "d.ari"};
  EXPECT_NE(parse_host_args(5, bad_cell).error.find("WxH"), std::string::npos);
  const char *zero_cell[] = {"ari", "snapshot", "--cell-size", "0x16", "d.ari"};
  EXPECT_FALSE(parse_host_args(5, zero_cell).error.empty());
  // After the document they are the document's, and ari says so.
  const char *after[] = {"ari", "run", "d.ari", "--graphics", "sixel"};
  o = parse_host_args(5, after);
  EXPECT_TRUE(o.graphics.empty());
  EXPECT_NE(misplaced_flag_note("ari", o.doc_args).find("'--graphics'"), std::string::npos);
}

TEST(AriadneDocArgs, ParsesOptionsBoolsAndPositionals) {
  const DocumentArgs a = parse_document_args(
      conf_spec(), {"--room", "standup", "--camera", "--peers=3", "--scale", "0.5", "notes.txt"});
  EXPECT_TRUE(a.errors.empty()) << a.errors.front();
  EXPECT_FALSE(a.help);
  EXPECT_EQ(value_of(a, "room"), "standup");
  EXPECT_EQ(value_of(a, "camera"), "true");
  EXPECT_EQ(value_of(a, "peers"), "3");
  EXPECT_EQ(value_of(a, "scale"), "0.5");
  EXPECT_EQ(value_of(a, "file"), "notes.txt");
}

TEST(AriadneDocArgs, DefaultsNegationRepeatsAndDoubleDash) {
  const std::vector<ArgSpec> spec = conf_spec();
  DocumentArgs a = parse_document_args(spec, {});
  EXPECT_TRUE(a.errors.empty());
  EXPECT_EQ(value_of(a, "room"), "lobby"); // defaults fill in
  EXPECT_EQ(value_of(a, "peers"), "8");
  EXPECT_EQ(value_of(a, "camera"), "<absent>"); // no default, not given: absent
  a = parse_document_args(spec, {"--camera", "--no-camera", "--room", "a", "--room", "b"});
  EXPECT_EQ(value_of(a, "camera"), "false");
  EXPECT_EQ(value_of(a, "room"), "b"); // the last one wins
  a = parse_document_args(spec, {"--", "--not-an-option"});
  EXPECT_TRUE(a.errors.empty());
  EXPECT_EQ(value_of(a, "file"), "--not-an-option");
  a = parse_document_args(spec, {"--peers", "-3"}); // a negative number is a value
  EXPECT_EQ(value_of(a, "peers"), "-3");
  a = parse_document_args(spec, {"-h"});
  EXPECT_TRUE(a.help);
}

TEST(AriadneDocArgs, ReportsEveryProblem) {
  const DocumentArgs a =
      parse_document_args(conf_spec(), {"--bogus", "--peers", "many", "--camera=maybe", "one.txt",
                                        "two.txt", "--room"});
  ASSERT_EQ(a.errors.size(), 5u);
  EXPECT_NE(a.errors[0].find("unknown option '--bogus'"), std::string::npos);
  EXPECT_NE(a.errors[1].find("'many' is not an int"), std::string::npos);
  EXPECT_NE(a.errors[2].find("'maybe' is not a bool"), std::string::npos);
  EXPECT_NE(a.errors[3].find("unexpected argument 'two.txt'"), std::string::npos);
  EXPECT_NE(a.errors[4].find("--room needs a string value"), std::string::npos);

  ArgSpec need;
  need.name = "token";
  need.required = true;
  EXPECT_EQ(parse_document_args({need}, {}).errors,
            (std::vector<std::string>{"missing required --token"}));
}

TEST(AriadneDocArgs, UsageListsEveryArgument) {
  const std::string u = format_document_usage("ari run conf.ari", "Join a call.", conf_spec());
  EXPECT_NE(u.find("usage: ari run conf.ari [options] [file]"), std::string::npos) << u;
  EXPECT_NE(u.find("Join a call."), std::string::npos) << u;
  EXPECT_NE(u.find("--room <string>"), std::string::npos) << u;
  EXPECT_NE(u.find("Room to join (default: lobby)"), std::string::npos) << u;
  EXPECT_NE(u.find("--camera "), std::string::npos) << u; // a bool takes no value
  EXPECT_NE(u.find("-h, --help"), std::string::npos) << u;
}

TEST(AriadneDocArgs, LoaderRejectsAMalformedSpec) {
  REQUIRE_YAML();
  const auto fails = [](const std::string &args, const std::string &want) {
    LoadResult lr = load_string("meta: {min_libcvc: 3.4.0}\nargs:\n" + args + "root: []\n");
    EXPECT_FALSE(lr.ok) << args;
    EXPECT_NE(lr.error.find(want), std::string::npos) << lr.error;
  };
  fails("  n: {type: number}\n", "unknown type 'number'");
  fails("  n: {type: int, default: lots}\n", "default 'lots' is not an int");
  fails("  help: bool\n", "not a valid argument name");
  fails("  9lives: bool\n", "not a valid argument name");
  fails("  f: {type: bool, positional: true}\n", "cannot be a bool");
  fails("  a: {positional: true}\n  b: {positional: true, required: true}\n",
        "a required positional cannot follow an optional one");
  LoadResult ok = load_string("meta: {min_libcvc: 3.4.0}\nargs:\n  v: bool\nroot: []\n");
  ASSERT_TRUE(ok.ok) << ok.error;
  EXPECT_TRUE(ok.has_args_block);
  ASSERT_EQ(ok.args.size(), 1u);
  EXPECT_EQ(ok.args[0].type, ArgSpec::Type::Bool);
}

// ── the arguments on the tree ───────────────────────────────────────────────────────────────────

TEST(AriadneDocArgs, AppRuntimePublishesThemBeforeInitRuns) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("publish"), "conf.ari", kConfDoc);
  cvc::app app;
  FtxuiBackend be; // an inert stub without FTXUI; this test never draws
  AppRuntime art(app, "ui.docs.conf");
  art.set_backend(be);
  art.set_document_args({"--room", "standup", "--peers", "3", "--camera"});
  const std::vector<std::string> warns = art.load(doc.string());
  EXPECT_TRUE(warns.empty()) << warns.front();
  EXPECT_TRUE(art.has_args_block());
  // init: ran after the arguments were on the tree.
  EXPECT_EQ(val(app, "ui.docs.conf.got.room"), "standup");
  EXPECT_EQ(val(app, "ui.docs.conf.got.argc"), "5");
  // argv: the strings as given, as a DSL list.
  const se::value_t argv = data_of(app, "ui.docs.conf.argv");
  const auto *list = std::get_if<se::list_ptr>(&argv.v);
  ASSERT_TRUE(list && *list);
  ASSERT_EQ((*list)->size(), 5u);
  EXPECT_EQ(std::get<std::string>((*list)->at(1).v), "standup");
  EXPECT_EQ(val(app, "ui.docs.conf.argv"), "--room standup --peers 3 --camera");
  // args.<name>: canonical text plus the typed value.
  EXPECT_EQ(val(app, "ui.docs.conf.args.peers"), "3");
  EXPECT_EQ(std::get<int64_t>(data_of(app, "ui.docs.conf.args.peers").v), 3);
  EXPECT_TRUE(std::get<bool>(data_of(app, "ui.docs.conf.args.camera").v));
  EXPECT_EQ(cvc::state::instance(app).findDescendant("ui.docs.conf.args.scale"), nullptr);
  // args: the whole set as one dict.
  const se::value_t all = data_of(app, "ui.docs.conf.args");
  const auto *dict = std::get_if<se::dict_ptr>(&all.v);
  ASSERT_TRUE(dict && *dict);
  EXPECT_EQ((*dict)->size(), 3u); // room, camera, peers (scale and file were not given)
}

TEST(AriadneDocArgs, ADocumentCannotRewriteItsArguments) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("readonly"), "ro.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "args:\n  room: {default: lobby}\n"
                             "init: |\n"
                             "  (state-set \"args.room\" \"hijacked\")\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (state-set \"argv\" \"rewritten\")\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.ro");
  art.set_backend(be);
  art.set_document_args({"--room", "standup"});
  const std::vector<std::string> warns = art.load(doc.string());
  for (int i = 0; i < 3; ++i)
    art.drain();
  EXPECT_EQ(val(app, "ui.docs.ro.args.room"), "standup");
  EXPECT_EQ(val(app, "ui.docs.ro.argv"), "--room standup");
  // The refused write in init: is reported, not silent.
  bool reported = false;
  for (const std::string &w : warns)
    reported = reported || w.find("init:") != std::string::npos;
  EXPECT_TRUE(reported);
}

TEST(AriadneDocArgs, HelpAndBadArgumentsStopTheLoadWithUsage) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("usage"), "conf.ari", kConfDoc);
  cvc::app app;
  FtxuiBackend be;
  {
    AppRuntime art(app, "ui.docs.a");
    art.set_backend(be);
    art.set_document_args({"--help"}, "ari run conf.ari");
    try {
      art.load(doc.string());
      FAIL() << "--help must stop the load";
    } catch (const document_args_error &e) {
      EXPECT_TRUE(e.help);
      EXPECT_NE(e.usage.find("usage: ari run conf.ari"), std::string::npos) << e.usage;
    }
    EXPECT_EQ(cvc::state::instance(app).findDescendant("ui.docs.a.got"), nullptr); // no init:
  }
  {
    AppRuntime art(app, "ui.docs.b");
    art.set_backend(be);
    art.set_document_args({"--bogus"});
    try {
      art.load(doc.string());
      FAIL() << "an unknown option must stop the load";
    } catch (const document_args_error &e) {
      EXPECT_FALSE(e.help);
      EXPECT_NE(std::string(e.what()).find("unknown option '--bogus'"), std::string::npos);
    }
  }
  {
    AppRuntime art(app, "ui.docs.c"); // a check: arguments are not validated, defaults apply
    art.set_backend(be);
    art.set_document_args({"--bogus"}, "", /*validate=*/false);
    EXPECT_NO_THROW(art.load(doc.string()));
    EXPECT_EQ(val(app, "ui.docs.c.args.room"), "lobby");
  }
}

TEST(AriadneDocArgs, WithoutAnArgsBlockTheDocumentGetsRawArgv) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("raw"), "raw.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "init: |\n"
                             "  (state-set \"first\" (car (state-data-get \"argv\")))\n"
                             "root:\n  - text: hi\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.raw");
  art.set_backend(be);
  art.set_document_args({"--anything", "goes"});
  EXPECT_NO_THROW(art.load(doc.string()));
  EXPECT_FALSE(art.has_args_block());
  EXPECT_EQ(val(app, "ui.docs.raw.first"), "--anything");
  EXPECT_EQ(cvc::state::instance(app).findDescendant("ui.docs.raw.args"), nullptr);
}

TEST(AriadneDocArgs, AModuleSeesThemOnlyThroughAReadOnlyGrant) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("mount");
  const fs::path frag =
      write(dir, "peek.ari",
            "init: |\n"
            "  (state-set \"seen\" (if (state-exists \"args\")\n"
            "                        (get-attr (state-data-get \"args\") \"room\") \"none\"))\n"
            "  (state-set \"seen_argv\" (if (state-exists \"argv\") \"yes\" \"no\"))\n"
            "root:\n  - text: peek\n");
  const fs::path doc = write(dir, "parent.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "args:\n  room: {default: lobby}\n"
                             "root:\n"
                             "  - load: " +
                                 frag.string() +
                                 "\n"
                                 "    as: granted\n"
                                 "    link:\n"
                                 "      args: {to: args, mode: ro}\n"
                                 "  - load: " +
                                 frag.string() +
                                 "\n"
                                 "    as: denied\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.p");
  art.set_backend(be);
  art.set_document_args({"--room", "standup"});
  art.load(doc.string());
  art.runtime().render(); // the reconcile mounts both modules, wires the hole, runs their init:
  EXPECT_EQ(val(app, "ui.docs.p.includes.granted.seen"), "standup");
  EXPECT_EQ(val(app, "ui.docs.p.includes.denied.seen"), "none");
  EXPECT_EQ(val(app, "ui.docs.p.includes.granted.seen_argv"), "no"); // granted args, not argv
  EXPECT_EQ(val(app, "ui.docs.p.includes.denied.seen_argv"), "no");
}

// A hole on an interior node grants the subtree under it: a module's programs read the parent's
// parsed reply through it, write it only through a read-write grant, and reach nothing else.
TEST(AriadneLinkHoles, ASubtreeGrantReachesItsDescendantsOnly) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("subtree");
  const fs::path frag = write(
      dir, "view.ari",
      "init: |\n"
      "  (state-set \"first\" (state-get \"tree.packages.0.name\"))\n"
      "  (state-set \"count\" (length (state-children \"tree.packages\")))\n"
      "  (state-set \"meta\" (if (state-exists \"tree.meta\")\n"
      "                        (get-attr (state-data-get \"tree.meta\") \"total\") \"none\"))\n"
      "  (state-set \"peek\" (if (state-exists \"search.secret\") \"leak\" \"none\"))\n"
      "  (state-set \"tree.packages.0.seen\" \"yes\")\n"
      "root:\n  - text: view\n");
  const fs::path doc = write(dir, "parent.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "init: |\n"
                             "  (state-set \"search.res.packages.0.name\" \"zstd\")\n"
                             "  (state-set \"search.res.packages.1.name\" \"lz4\")\n"
                             "  (state-data-set \"search.res.meta\" (dict \"total\" 2))\n"
                             "  (state-set \"search.secret\" \"s3cret\")\n"
                             "root:\n"
                             "  - load: " +
                                 frag.string() +
                                 "\n"
                                 "    as: ro\n"
                                 "    link:\n"
                                 "      tree: {to: search.res, mode: ro}\n"
                                 "  - load: " +
                                 frag.string() +
                                 "\n"
                                 "    as: rw\n"
                                 "    link:\n"
                                 "      tree: search.res\n"
                                 "  - load: " +
                                 frag.string() +
                                 "\n"
                                 "    as: denied\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.p");
  art.set_backend(be);
  art.load(doc.string());
  art.runtime().render(); // the reconcile mounts the modules, wires the holes, runs their init:
  for (const char *as : {"ro", "rw"}) {
    const std::string at = std::string("ui.docs.p.includes.") + as + ".";
    EXPECT_EQ(val(app, at + "first"), "zstd") << as;
    EXPECT_EQ(val(app, at + "count"), "4") << as; // packages.0, .0.name, .1, .1.name
    EXPECT_EQ(val(app, at + "meta"), "2") << as;
    EXPECT_EQ(val(app, at + "peek"), "none") << as; // the parent outside the target stays out
  }
  // Only the read-write grant wrote the parent; the read-only one's write stayed in the module.
  EXPECT_EQ(val(app, "ui.docs.p.search.res.packages.0.seen"), "yes");
  EXPECT_EQ(val(app, "ui.docs.p.includes.ro.tree.packages.0.seen"), "yes");
  EXPECT_EQ(val(app, "ui.docs.p.includes.rw.tree.packages.0.seen"), "");
  EXPECT_EQ(val(app, "ui.docs.p.includes.denied.first"), "");
  EXPECT_EQ(val(app, "ui.docs.p.includes.denied.count"), "0");
}

// ── host verbs and init: diagnostics ────────────────────────────────────────────────────────────

TEST(AriadneHostVerbs, EveryRegisteredVerbIsBound) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("verbs"), "v.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (first-verb)\n"
                             "  (second-verb)\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.v");
  art.set_backend(be);
  int first = 0, second = 0;
  art.register_verb("first-verb", [&] { ++first; });
  art.register_verb("second-verb", [&] { ++second; }); // registered after the first one
  art.load(doc.string());
  for (int i = 0; i < 4; ++i)
    art.drain();
  EXPECT_GT(first, 0);
  EXPECT_GT(second, 0);
}

TEST(AriadneHostVerbs, AGoneRuntimesVerbsAreNotBound) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("gone"), "g.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (stale-verb)\n");
  cvc::app app;
  FtxuiBackend be;
  int stale = 0;
  {
    AppRuntime old(app, "ui.docs.old");
    old.register_verb("stale-verb", [&] { ++stale; });
  } // its provider stays in the process-wide registry, but must bind nothing now
  AppRuntime art(app, "ui.docs.g");
  art.set_backend(be);
  art.load(doc.string());
  for (int i = 0; i < 4; ++i)
    art.drain();
  EXPECT_EQ(stale, 0);
}

// Drain `art` until `pred` holds, or about 5 s pass.
template <class Pred> bool drain_until(AppRuntime &art, Pred pred) {
  for (int i = 0; i < 2500 && !pred(); ++i) {
    art.drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

// ── (quit [code]), run_program and tests: (ari test) ───────────────────────────────────────────

// Mount a lone button with `on` as its on:, and press it.
void press_button(AppRuntime &art, FtxuiBackend &be, const std::string &on) {
  art.set_root(group({button("Go", on)}));
  be.render_to_string(art.runtime(), 80, 24);
  be.feed(art.runtime(), TermEvent::key_event("Tab"));
  be.render_to_string(art.runtime(), 80, 24);
  be.feed(art.runtime(), TermEvent::key_event("Return"));
}

// A document that runs `tick` on its first tick only.
std::string once_on_tick(const std::string &tick) {
  return "meta: {min_libcvc: 3.4.0}\n"
         "root:\n  - text: hi\n"
         "init: (state-set \"t.done\" \"\")\n"
         "on_tick: |\n"
         "  (if (= (state-get \"t.done\") \"\") (begin (state-set \"t.done\" \"1\") " +
         tick + "))\n";
}

TEST(AriadneQuit, QuitSetsTheExitCodeAndTheFirstOneStands) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("quit_code"), "q.ari", once_on_tick("(quit 3) (quit 4)"));
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.q");
  art.set_backend(be);
  art.register_quit();
  EXPECT_TRUE(art.runtime().has_function("quit"));
  art.load(doc.string());
  EXPECT_EQ(art.exit_code(), 0);
  ASSERT_TRUE(drain_until(art, [&] { return art.should_close(); }));
  EXPECT_EQ(art.exit_code(), 3);
  EXPECT_TRUE(art.take_warnings().empty());
}

TEST(AriadneQuit, QuitFromInitAndWithNoCode) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("quit_init");
  {
    cvc::app app;
    FtxuiBackend be;
    AppRuntime art(app, "ui.docs.qi");
    art.set_backend(be);
    art.register_quit();
    art.load(write(dir, "i.ari",
                   "meta: {min_libcvc: 3.4.0}\nroot:\n  - text: hi\n"
                   "init: (quit 7)\n")
                 .string());
    EXPECT_TRUE(art.should_close()); // init: runs in load()
    EXPECT_EQ(art.exit_code(), 7);
  }
  {
    cvc::app app;
    FtxuiBackend be;
    AppRuntime art(app, "ui.docs.qn");
    art.set_backend(be);
    art.register_quit();
    art.load(write(dir, "n.ari", once_on_tick("(quit)")).string());
    ASSERT_TRUE(drain_until(art, [&] { return art.should_close(); }));
    EXPECT_EQ(art.exit_code(), 0);
  }
}

TEST(AriadneQuit, ABadCodeRaisesAndNothingCloses) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("quit_bad");
  const std::pair<const char *, const char *> cases[] = {
      {"(quit 256)", "ValueError: quit: the exit code is 0..255, not 256"},
      {"(quit -1)", "ValueError: quit: the exit code is 0..255, not -1"},
      {"(quit \"x\")", "TypeError: quit: the exit code is an integer"},
      {"(quit 1 2)", "TypeError: quit: expected an optional exit code"},
  };
  for (const auto &[call, want] : cases) {
    SCOPED_TRACE(call);
    cvc::app app;
    FtxuiBackend be;
    AppRuntime art(app, "ui.docs.qb");
    art.set_backend(be);
    art.register_quit();
    art.load(write(dir, "b.ari", once_on_tick(call)).string());
    std::vector<std::string> warnings;
    ASSERT_TRUE(drain_until(art, [&] {
      for (std::string &w : art.take_warnings())
        warnings.push_back(std::move(w));
      return !warnings.empty();
    }));
    EXPECT_NE(warnings.front().find(want), std::string::npos) << warnings.front();
    EXPECT_FALSE(art.should_close());
  }
}

// `on: quit` (a menu item's event) still closes, with code 0.
TEST(AriadneQuit, TheQuitEventCloses) {
  REQUIRE_YAML();
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.qe");
  art.set_backend(be);
  art.register_quit();
  press_button(art, be, "quit"); // a menu item's on: quit is the same event
  art.drain();
  EXPECT_TRUE(art.should_close());
  EXPECT_EQ(art.exit_code(), 0);
}

// run_program reports how the program ended to its callback, and never as a warning.
TEST(AriadneRunProgram, DoneSaysHowItEnded) {
  REQUIRE_YAML();
  const fs::path doc =
      write(test_dir("run_program"), "r.ari", "meta: {min_libcvc: 3.4.0}\nroot:\n  - text: hi\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.rp");
  art.set_backend(be);
  art.load(doc.string());
  art.drain();
  std::vector<std::string> how;
  const auto run = [&](const std::string &script) {
    const std::size_t before = how.size();
    art.runtime().run_program(script, [&](const std::string &h) { how.push_back(h); });
    EXPECT_TRUE(drain_until(art, [&] { return how.size() > before; })) << script;
  };
  run("(state-set \"x\" 5)");
  run("(assert (= (state-get \"x\") 6) \"x is six\")");
  run("(sleep 0.01) (error \"late\")"); // parks first: it ends on a later drain
  run("(this is not closed");
  ASSERT_EQ(how.size(), 4u);
  EXPECT_EQ(how[0], "");
  EXPECT_EQ(val(app, "ui.docs.rp.x"), "5"); // chrooted to the document, as an on: is
  EXPECT_NE(how[1].find("failed: AssertionError: assertion failed: x is six (at (assert"),
            std::string::npos)
      << how[1];
  EXPECT_NE(how[2].find("failed: RuntimeError: late"), std::string::npos) << how[2];
  EXPECT_FALSE(how[3].empty());
  EXPECT_TRUE(art.take_warnings().empty());
}

// An on: action's failure names the program it came from.
TEST(AriadneRunProgram, AnActionsFailureNamesItsProgram) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("action_names"), "a.ari", once_on_tick("(set y 1)"));
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.an");
  art.set_backend(be);
  art.load(doc.string());
  press_button(art, be, "(assert (= 1 2)\n  \"one is two\")");
  std::vector<std::string> warnings;
  ASSERT_TRUE(drain_until(art, [&] {
    for (std::string &w : art.take_warnings())
      warnings.push_back(std::move(w));
    return !warnings.empty();
  }));
  EXPECT_NE(warnings.front().find("action program failed: AssertionError: assertion failed: one "
                                  "is two"),
            std::string::npos)
      << warnings.front();
  EXPECT_NE(warnings.front().find("[(assert (= 1 2) \"one is two\")]"), std::string::npos)
      << warnings.front();
}

TEST(AriadneTests, AppRuntimeKeepsTheTestsBlock) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("tests_block"), "t.ari",
                             "meta: {min_libcvc: 3.4.0}\nroot:\n  - text: hi\n"
                             "tests:\n  one: (assert true)\n  two: (nope)\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.tb");
  art.set_backend(be);
  const std::vector<std::string> warnings = art.load(doc.string());
  ASSERT_EQ(art.tests().size(), 2u);
  EXPECT_EQ(art.tests()[1].first, "two");
  bool named = false; // the free-head preflight covers a test's program too
  for (const std::string &w : warnings)
    named = named || w.find("test 'two' calls (nope ...)") != std::string::npos;
  EXPECT_TRUE(named);
}

// register_async_verb makes an async function (docs/STATE_EXEC_EXCEPTIONS.md 3): (name) parks the
// program while the work runs on the app's background task pool, and returns its string.
TEST(AriadneHostVerbs, AnAsyncVerbParksAndReturnsItsValue) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("async"), "a.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (state-set \"got\" (slow-verb))\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.a");
  art.set_backend(be);
  art.register_async_verb("slow-verb", [] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return std::string("done");
  });
  art.load(doc.string());
  EXPECT_TRUE(drain_until(art, [&] { return val(app, "ui.docs.a.got") == "done"; }))
      << val(app, "ui.docs.a.got");
}

// With a done channel the result is also posted there, so a document that waits with msg-recv
// (the old pattern) keeps working.
TEST(AriadneHostVerbs, AnAsyncVerbStillPostsToItsDoneChannel) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("asyncchan"), "c.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (begin (old-verb) (state-set \"chan\" (msg-recv \"old.done\")))\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.c");
  art.set_backend(be);
  art.register_async_verb("old-verb", "old.done", [] { return std::string("posted"); });
  art.load(doc.string());
  EXPECT_TRUE(drain_until(art, [&] { return val(app, "ui.docs.c.chan") == "posted"; }))
      << val(app, "ui.docs.c.chan");
}

// The work's exception raises at the call, where the program can catch it. It used to be lost, and
// the channel got "".
TEST(AriadneHostVerbs, AnAsyncVerbsErrorRaisesAtTheCall) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("asyncerr"), "e.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "root:\n  - text: hi\n"
                             "on_tick: |\n"
                             "  (state-set \"err\" (try (bad-verb) (catch (e) (list (get-attr e "
                             "\"origin\") (get-attr e \"message\")))))\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.e");
  art.set_backend(be);
  art.register_async_verb("bad-verb",
                          []() -> std::string { throw std::runtime_error("the verb failed"); });
  art.load(doc.string());
  EXPECT_TRUE(drain_until(art, [&] { return !val(app, "ui.docs.e.err").empty(); }));
  EXPECT_NE(val(app, "ui.docs.e.err").find("async"), std::string::npos)
      << val(app, "ui.docs.e.err");
  EXPECT_NE(val(app, "ui.docs.e.err").find("the verb failed"), std::string::npos)
      << val(app, "ui.docs.e.err");
}

TEST(AriadneHostVerbs, AFailingInitIsAWarning) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("init"), "i.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "init: |\n"
                             "  (no-such-function 1)\n"
                             "root:\n  - text: hi\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.i");
  art.set_backend(be);
  const std::vector<std::string> warns = art.load(doc.string());
  ASSERT_FALSE(warns.empty());
  EXPECT_NE(warns.front().find("init:"), std::string::npos) << warns.front();
}

// ── §6.6 functions: a document's, and the ones a mount grants its module ─────────────────────────

namespace {
// Press the only button on screen (Tab to it, Enter).
void press_only_button(AppRuntime &art, FtxuiBackend &be) {
  be.render_to_string(art.runtime(), 80, 24);
  be.feed(art.runtime(), TermEvent::key_event("Tab"));
  be.render_to_string(art.runtime(), 80, 24);
  be.feed(art.runtime(), TermEvent::key_event("Return"));
}
} // namespace

TEST(AriadneFunctions, EveryLaneOfTheDocumentCallsItsFunctions) {
  REQUIRE_YAML();
  const fs::path doc = write(test_dir("fn_lanes"), "f.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "functions: |\n"
                             "  (defun tag (s) (str-concat \"<\" s \">\"))\n"
                             "  (defun mark (key s) (state-set key (tag s)))\n"
                             "init: (mark \"t.init\" \"init\")\n"
                             "on_tick: (mark \"t.tick\" \"tick\")\n"
                             "root:\n"
                             "  - button: Go\n"
                             "    on: (mark \"t.click\" \"click\")\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.fl");
  art.set_backend(be);
  const std::vector<std::string> warnings = art.load(doc.string());
  EXPECT_TRUE(warnings.empty()) << warnings.front(); // the preflight knows tag and mark
  EXPECT_EQ(val(app, "ui.docs.fl.t.init"), "<init>");
  ASSERT_TRUE(drain_until(art, [&] { return val(app, "ui.docs.fl.t.tick") == "<tick>"; }));
  press_only_button(art, be);
  ASSERT_TRUE(drain_until(art, [&] { return val(app, "ui.docs.fl.t.click") == "<click>"; }));
  std::string how = "pending";
  art.runtime().run_program("(assert (= (tag \"t\") \"<t>\"))",
                            [&](const std::string &h) { how = h; });
  ASSERT_TRUE(drain_until(art, [&] { return how != "pending"; }));
  EXPECT_EQ(how, "");
  EXPECT_TRUE(art.take_warnings().empty());
}

// A granted function runs in the module: its writes land in the module's own subtree (code is
// granted, not the parent's state), and a module granted nothing cannot call it.
TEST(AriadneFunctions, AModuleCallsWhatItsMountGrants) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("fn_grant");
  write(dir, "mod.ari",
        "init: (record \"init\")\n"
        "root:\n"
        "  - button: Go\n"
        "    on: (record \"click\")\n");
  const fs::path doc = write(dir, "p.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "functions: |\n"
                             "  (defun stamp (s) (str-concat s \"!\"))\n"
                             "  (defun record (s) (state-set \"out\" (stamp s)))\n"
                             "root:\n"
                             "  - load: mod.ari\n"
                             "    as: m\n"
                             "    functions: [record]\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.fg");
  art.set_backend(be);
  const std::vector<std::string> warnings = art.load(doc.string());
  EXPECT_TRUE(warnings.empty()) << warnings.front();
  be.render_to_string(art.runtime(), 80, 24); // mounts the module, runs its init:
  EXPECT_EQ(val(app, "ui.docs.fg.includes.m.out"), "init!");
  EXPECT_EQ(val(app, "ui.docs.fg.out"), ""); // not the parent's
  press_only_button(art, be);
  ASSERT_TRUE(drain_until(art, [&] { return val(app, "ui.docs.fg.includes.m.out") == "click!"; }));
  EXPECT_TRUE(art.take_warnings().empty());
}

TEST(AriadneFunctions, AModuleGrantedNothingCannotCall) {
  REQUIRE_YAML();
  const fs::path dir = test_dir("fn_deny");
  write(dir, "mod.ari", "root:\n  - button: Go\n    on: (record \"click\")\n");
  const fs::path doc = write(dir, "p.ari",
                             "meta: {min_libcvc: 3.4.0}\n"
                             "functions: |\n  (defun record (s) (state-set \"out\" s))\n"
                             "root:\n  - load: mod.ari\n    as: m\n");
  cvc::app app;
  FtxuiBackend be;
  AppRuntime art(app, "ui.docs.fd");
  art.set_backend(be);
  bool told = false; // the preflight names the program in the module
  for (const std::string &w : art.load(doc.string()))
    told = told || w.find("widget 'Go' calls (record ...)") != std::string::npos;
  EXPECT_TRUE(told);
  press_only_button(art, be);
  std::vector<std::string> warnings;
  ASSERT_TRUE(drain_until(art, [&] {
    for (std::string &w : art.take_warnings())
      warnings.push_back(std::move(w));
    return !warnings.empty();
  }));
  EXPECT_NE(warnings.front().find("NameError"), std::string::npos) << warnings.front();
  EXPECT_EQ(val(app, "ui.docs.fd.includes.m.out"), "");
}
