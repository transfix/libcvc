// The network, its credentials, the end-to-end keys and the OIDC sign-ins per principal
// (docs/roadmap/ARIADNE_SHELL.md 4.12, SH11): a program's verbs reach what the principal of the
// mount it runs in may, so a document mounted beside nicotina cannot post as the reader. Each
// mount is a scope with a principal recorded for it (principal.h), as the shell's Runtime records
// one when it opens a mounts: entry; each site is a localhost server, its own origin.

#include "local_http_server.h"

#include <chrono>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/e2ee_intrinsics.h>
#include <cvc/ariadne/media.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/net_intrinsics.h>
#include <cvc/ariadne/principal.h>
#include <cvc/core/app.h>
#include <cvc/net/credentials.h>
#include <cvc/net/e2ee.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <cvc/net/ws_client.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;
using cvc::ariadne::AppRuntime;
using cvc::ariadne::enable_net;
using cvc::ariadne::LaneProcess;
using cvc::ariadne::NetHostConfig;
using cvc::ariadne::ScopePrincipal;
using cvc::ariadne::set_scope_principal;
using cvc_test::LocalHttpServer;

const std::string kReaderToken = "cmVhZGVyLXNlc3Npb24tdG9rZW4tZm9yLW5pY290aW5h";
const std::string kStrangerToken = "c3RyYW5nZXItc2Vzc2lvbi10b2tlbi1vZi1pdHMtb3du";

// A TX.WTF-shaped site that hands out `token`: POST /user/login answers {token}; GET /user/me and
// POST /posts want it as the bearer; /echo answers with the Authorization it was sent.
LocalHttpServer::Handler site(const std::string &token) {
  return [token](const LocalHttpServer::Request &req) -> LocalHttpServer::Reply {
    const bool authed = req.header("Authorization") == "Bearer " + token;
    if (req.target == "/user/login" && req.method == "POST")
      return {200, {"Content-Type: application/json"}, "{\"token\": \"" + token + "\"}"};
    if (req.target == "/user/me")
      return authed ? LocalHttpServer::Reply{200, {}, "{\"name\": \"joe\"}"}
                    : LocalHttpServer::Reply{401, {}, "{}"};
    if (req.target == "/posts" && req.method == "POST")
      return authed ? LocalHttpServer::Reply{201, {}, "{\"id\": 7}"}
                    : LocalHttpServer::Reply{401, {}, "{}"};
    if (req.target == "/echo")
      return {200, {}, "sent " + req.header("Authorization")};
    return {404, {}, ""};
  };
}

NetHostConfig allow(std::initializer_list<std::string> origins) {
  NetHostConfig c;
  c.enabled = true;
  c.allow = origins;
  return c;
}

ScopePrincipal as(const std::string &principal, std::vector<std::string> granted = {}) {
  ScopePrincipal p;
  p.principal = principal;
  p.net_origins = std::move(granted);
  return p;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// Run `program` as a program of the mount at `scope` and drain until it ends: "" or why not.
std::string run_in(AppRuntime &art, const std::string &scope, const std::string &program) {
  bool ended = false;
  std::string how = "unset";
  LaneProcess p;
  p.program = program;
  p.name = "sh11";
  p.scope = scope;
  p.done = [&](const std::string &h) {
    ended = true;
    how = h;
  };
  art.runtime().start_process(std::move(p));
  const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  while (!ended && std::chrono::steady_clock::now() < until) {
    art.drain();
    if (!ended)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return ended ? how : "timed out";
}

// The program that signs in to `api` and keeps the token as credential txwtf.
std::string login(const LocalHttpServer &api) {
  return "(begin (set r (http-request \"POST\" \"" + api.url("/user/login") +
         "\" (dict \"form\" (dict \"username\" \"joe\" \"password\" \"pw\")"
         " \"capture_auth\" (dict \"name\" \"txwtf\" \"field\" \"token\"))))"
         " (state-set \"login.captured\" (get-attr r \"captured\"))"
         " (state-set \"login.error\" (get-attr r \"error\")))";
}

// The status (or the error) of a GET of `url` sent with credential txwtf, kept under `key`.
std::string get_with_txwtf(const std::string &url, const std::string &key) {
  return "(begin (set r (try (http-get \"" + url +
         "\" (dict \"auth\" \"txwtf\")) (catch (e) (dict \"status\" -1 \"error\" (get-attr e "
         "\"message\")))))"
         " (state-set \"" +
         key + ".status\" (get-attr r \"status\")) (state-set \"" + key +
         ".error\" (get-attr r \"error\")))";
}

// The Authorization headers a server was sent, in order.
std::vector<std::string> auths(const LocalHttpServer &s) {
  std::vector<std::string> out;
  for (const LocalHttpServer::Request &r : s.requests())
    out.push_back(r.header("Authorization"));
  return out;
}

struct TempDir {
  fs::path path;
  TempDir() {
    std::random_device rd;
    path = fs::temp_directory_path() /
           ("cvc-sh11-test-" + std::to_string(rd()) + std::to_string(rd()));
    fs::create_directories(path);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
};

std::string read_file(const fs::path &p) {
  std::ifstream in(p, std::ios::binary);
  std::ostringstream s;
  s << in.rdbuf();
  return s.str();
}

#define NEEDS_HTTP()                                                                               \
  if (!cvc::net::have_http_backend())                                                              \
  GTEST_SKIP() << "libcvc built without an HTTP backend"

} // namespace

// --- who a scope runs as -------------------------------------------------------------------------

TEST(AriadnePrincipal, TheInnermostMountDecidesAndALocalOneCannotRaiseIt) {
  cvc::app app;
  using cvc::ariadne::scope_principal;
  EXPECT_EQ(scope_principal(app, "shell").principal, "local") << "nothing recorded";
  ASSERT_EQ(set_scope_principal(app, "shell.includes.feed", as("local")), "");
  ASSERT_EQ(
      set_scope_principal(app, "shell.includes.orrery", as("remote:HTTPS://Apps.Example.org:443/")),
      "");
  ASSERT_EQ(set_scope_principal(app, "shell.includes.orrery.includes.moon", as("local")), "");
  ASSERT_EQ(
      set_scope_principal(app, "shell.includes.orrery.includes.ads", as("remote:https://ads.test")),
      "");
  EXPECT_EQ(scope_principal(app, "shell.includes.feed.list.3").principal, "local");
  EXPECT_EQ(scope_principal(app, "shell.includes.orrery").principal,
            "remote:https://apps.example.org")
      << "written as the store keys it";
  EXPECT_EQ(scope_principal(app, "shell.includes.orrery.includes.moon.x").principal,
            "remote:https://apps.example.org")
      << "what a remote document mounts cannot raise it";
  EXPECT_EQ(scope_principal(app, "shell.includes.orrery.includes.ads").principal,
            "remote:https://ads.test");
  EXPECT_EQ(scope_principal(app, "shell.includes.orreryx").principal, "local")
      << "a sibling whose name starts the same is not under it";
  cvc::ariadne::clear_scope_principal(app, "shell.includes.orrery");
  EXPECT_EQ(scope_principal(app, "shell.includes.orrery.y").principal, "local");

  // Refused: not a principal, a sandboxed one with a network of its own.
  EXPECT_NE(set_scope_principal(app, "shell.includes.bad", as("nicotina")), "");
  EXPECT_NE(set_scope_principal(app, "shell.includes.bad", as("remote:https://x.test/path")), "");
  ScopePrincipal own = as("file:/home/joe/apps");
  own.credential_store = "mine";
  EXPECT_NE(set_scope_principal(app, "shell.includes.bad", own).find("sandboxed"),
            std::string::npos);
  EXPECT_EQ(scope_principal(app, "shell.includes.bad").principal, "local") << "nothing recorded";

  // The table is the app's: another app has none of it.
  cvc::app other;
  EXPECT_EQ(scope_principal(other, "shell.includes.feed.list.3").principal, "local");
  EXPECT_TRUE(cvc::ariadne::scope_principals(other).empty());
}

// --- credentials ---------------------------------------------------------------------------------

// The SH11 test of 4.12: a credential captured by one mount is not found from another, even where
// the other may reach the origin it was captured from; the other's own, of the same name, is its.
TEST(AriadneNetPrincipal, ACredentialCapturedByOneMountIsNotFoundFromAnother) {
  NEEDS_HTTP();
  LocalHttpServer api(site(kReaderToken)), own(site(kStrangerToken));
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({api.origin()})), "");
  const std::string feed = "shell.includes.feed", stranger = "shell.includes.stranger";
  // The stranger may reach the reader's site too, so only the credential's principal stops it.
  ASSERT_EQ(set_scope_principal(app, stranger, as("remote:" + own.origin(), {api.origin()})), "");
  AppRuntime art(app, "shell");

  ASSERT_EQ(run_in(art, feed, login(api)), "");
  EXPECT_EQ(val(app, feed + ".login.captured"), "true") << val(app, feed + ".login.error");

  // The stranger names it: not found, for any origin, and nothing goes out with it.
  ASSERT_EQ(run_in(art, stranger,
                   "(begin (state-set \"has\" (net-auth-has? \"txwtf\"))"
                   " (state-set \"has_api\" (net-auth-has? \"txwtf\" \"" +
                       api.url("/") + "\")))"),
            "");
  EXPECT_EQ(val(app, stranger + ".has"), "false");
  EXPECT_EQ(val(app, stranger + ".has_api"), "false");
  ASSERT_EQ(run_in(art, stranger, get_with_txwtf(api.url("/posts"), "steal")), "");
  EXPECT_NE(val(app, stranger + ".steal.error").find("no credential 'txwtf'"), std::string::npos)
      << val(app, stranger + ".steal.error");
  for (const std::string &a : auths(api))
    EXPECT_NE(a, "Bearer " + kReaderToken) << "only the sign-in reached the reader's site";

  // Its own sign-in at its own origin keeps a txwtf of its own.
  ASSERT_EQ(run_in(art, stranger, login(own)), "");
  EXPECT_EQ(val(app, stranger + ".login.captured"), "true") << val(app, stranger + ".login.error");
  ASSERT_EQ(run_in(art, stranger, get_with_txwtf(own.url("/user/me"), "me")), "");
  EXPECT_EQ(val(app, stranger + ".me.status"), "200");
  // ... bound to where it came from: still nothing for the reader's site.
  ASSERT_EQ(run_in(art, stranger, get_with_txwtf(api.url("/posts"), "steal2")), "");
  EXPECT_NE(val(app, stranger + ".steal2.error").find("no credential 'txwtf'"), std::string::npos);

  // The reader's still works, and is the reader's token; the stranger's clear leaves it.
  ASSERT_EQ(run_in(art, stranger, "(net-auth-clear)"), "");
  ASSERT_EQ(run_in(art, feed, get_with_txwtf(api.url("/user/me"), "again")), "");
  EXPECT_EQ(val(app, feed + ".again.status"), "200");
  EXPECT_EQ(auths(api).back(), "Bearer " + kReaderToken);
  for (const std::string &a : auths(own))
    EXPECT_NE(a, "Bearer " + kReaderToken) << "the reader's token never reached the stranger";
  // The launcher's view, the reader's: its txwtf, none of the stranger's.
  const auto launcher = cvc::ariadne::net_host_access(app);
  ASSERT_TRUE(launcher && launcher->credentials);
  EXPECT_EQ(launcher->credentials->names(), std::vector<std::string>{"txwtf"});
}

// The credential file keeps each principal's lines, and the next run gives each its own back.
TEST(AriadneNetPrincipal, TheCredentialFileKeepsEachPrincipalsOwn) {
  NEEDS_HTTP();
  LocalHttpServer api(site(kReaderToken)), own(site(kStrangerToken));
  TempDir dir;
  const fs::path file = dir.path / "credentials";
  NetHostConfig config = allow({api.origin()});
  config.credential_store = file.string();
  const std::string feed = "shell.includes.feed", stranger = "shell.includes.stranger";
  const std::string remote = "remote:" + own.origin();
  {
    cvc::app app;
    ASSERT_EQ(enable_net(app, config), "");
    ASSERT_EQ(set_scope_principal(app, stranger, as(remote)), "");
    AppRuntime art(app, "shell");
    ASSERT_EQ(run_in(art, feed, login(api)), "");
    ASSERT_EQ(run_in(art, stranger, login(own)), "");
  }
  const std::string text = read_file(file);
  EXPECT_NE(text.find("local txwtf " + api.origin() + " " + kReaderToken + "\n"), std::string::npos)
      << text;
  EXPECT_NE(text.find(remote + " txwtf " + own.origin() + " " + kStrangerToken + "\n"),
            std::string::npos)
      << text;
  {
    cvc::app app;
    ASSERT_EQ(enable_net(app, config), "");
    ASSERT_EQ(set_scope_principal(app, stranger, as(remote)), "");
    AppRuntime art(app, "shell");
    ASSERT_EQ(
        run_in(art, feed, "(state-set \"has\" (net-auth-has? \"txwtf\" \"" + api.url("/") + "\"))"),
        "");
    EXPECT_EQ(val(app, feed + ".has"), "true");
    ASSERT_EQ(run_in(art, stranger, get_with_txwtf(own.url("/user/me"), "me")), "");
    EXPECT_EQ(val(app, stranger + ".me.status"), "200") << "its own, back from the file";
    ASSERT_EQ(run_in(art, stranger,
                     "(state-set \"has\" (net-auth-has? \"txwtf\" \"" + api.url("/") + "\"))"),
              "");
    EXPECT_EQ(val(app, stranger + ".has"), "false");
  }
}

// --- the allowlist -------------------------------------------------------------------------------

// The SH11 test of 4.12: each mount's allowlist holds. local: the launcher's; remote: its own
// origin and what it was granted; file: only what it was granted.
TEST(AriadneNetPrincipal, EachMountsAllowlistHolds) {
  NEEDS_HTTP();
  LocalHttpServer launcher_site(site(kReaderToken)), own(site(kStrangerToken)),
      granted(site(kStrangerToken)), for_file(site(kStrangerToken));
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({launcher_site.origin()})), "");
  const std::string local = "shell.includes.feed", remote = "shell.includes.orrery",
                    file = "shell.includes.notes", bare = "shell.includes.bare";
  ASSERT_EQ(set_scope_principal(app, remote, as("remote:" + own.origin(), {granted.origin()})), "");
  ASSERT_EQ(set_scope_principal(app, file, as("file:/home/joe/notes", {for_file.origin()})), "");
  ASSERT_EQ(set_scope_principal(app, bare, as("file:/home/joe/bare")), "");
  AppRuntime art(app, "shell");

  struct Case {
    const LocalHttpServer *site;
    const char *name;
  };
  const std::vector<Case> sites = {
      {&launcher_site, "launcher"}, {&own, "own"}, {&granted, "granted"}, {&for_file, "for_file"}};
  const auto probe = [&](const std::string &scope) {
    std::string prog = "(begin";
    for (const Case &c : sites) {
      const std::string url = c.site->url("/echo");
      prog += " (state-set \"allowed." + std::string(c.name) + "\" (net-allowed? \"" + url + "\"))";
      prog += " (set r (try (http-get \"" + url +
              "\") (catch (e) (dict \"status\" -1))))"
              " (state-set \"status." +
              c.name + "\" (get-attr r \"status\"))";
    }
    prog += ")";
    EXPECT_EQ(run_in(art, scope, prog), "") << scope;
  };
  for (const std::string &scope : {local, remote, file, bare})
    probe(scope);
  const auto reached = [&](const std::string &scope, const char *name) {
    return val(app, scope + ".status." + name) == "200";
  };
  const auto allowed = [&](const std::string &scope, const char *name) {
    return val(app, scope + ".allowed." + name) == "true";
  };
  // The launcher's allowlist, for the local document.
  EXPECT_TRUE(reached(local, "launcher") && allowed(local, "launcher"));
  for (const char *n : {"own", "granted", "for_file"})
    EXPECT_FALSE(reached(local, n) || allowed(local, n)) << n;
  // The remote one's own origin and its grant, not the launcher's.
  EXPECT_TRUE(reached(remote, "own") && allowed(remote, "own"));
  EXPECT_TRUE(reached(remote, "granted") && allowed(remote, "granted"));
  EXPECT_FALSE(reached(remote, "launcher") || allowed(remote, "launcher"));
  EXPECT_FALSE(reached(remote, "for_file") || allowed(remote, "for_file"));
  // The file: one, its grant only; one with none reaches nothing.
  EXPECT_TRUE(reached(file, "for_file") && allowed(file, "for_file"));
  for (const char *n : {"launcher", "own", "granted"})
    EXPECT_FALSE(reached(file, n) || allowed(file, n)) << n;
  for (const char *n : {"launcher", "own", "granted", "for_file"})
    EXPECT_FALSE(reached(bare, n) || allowed(bare, n)) << n;
  // Each site saw only the requests its principals may make.
  EXPECT_EQ(launcher_site.requests().size(), 1u);
  EXPECT_EQ(own.requests().size(), 1u);
  EXPECT_EQ(granted.requests().size(), 1u);
  EXPECT_EQ(for_file.requests().size(), 1u);
}

// (fetch URL) goes through the app's http(s) handler, on the launcher's allowlist: a mounted
// document's own allowlist holds first.
TEST(AriadneNetPrincipal, FetchHoldsToTheMountsAllowlistToo) {
  NEEDS_HTTP();
  LocalHttpServer launcher_site(site(kReaderToken)), own(site(kStrangerToken));
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({launcher_site.origin(), own.origin()})), "");
  const std::string remote = "shell.includes.orrery";
  ASSERT_EQ(set_scope_principal(app, remote, as("remote:" + own.origin())), "");
  AppRuntime art(app, "shell");
  const auto fetch = [&](const std::string &scope, const LocalHttpServer &s, const char *key) {
    EXPECT_EQ(run_in(art, scope,
                     "(begin (set r (fetch \"" + s.url("/echo") + "\"))" + " (state-set \"" + key +
                         ".ok\" (get-attr r \"ok\"))" + " (state-set \"" + key +
                         ".error\" (get-attr r \"error\")))"),
              "");
  };
  fetch(remote, own, "own");
  fetch(remote, launcher_site, "other");
  fetch("shell", launcher_site, "launcher");
  EXPECT_EQ(val(app, remote + ".own.ok"), "true") << val(app, remote + ".own.error");
  EXPECT_EQ(val(app, remote + ".other.ok"), "false");
  EXPECT_NE(val(app, remote + ".other.error").find("fetch:"), std::string::npos);
  EXPECT_EQ(val(app, "shell.launcher.ok"), "true");
  EXPECT_EQ(launcher_site.requests().size(), 1u) << "the refused fetch was never sent";
}

// --- a local application's own policy and store (nicotina in the shell) --------------------------

// nicotina in the shell: shell.apps.nicotina.net_policy and .credential_store give it its own
// fleet policy and credential file, as bin/nicotina's --net-policy and --credential-store do; it
// signs in and posts against a local stand-in for its site. The launcher reaches none of that site,
// and another local application finds none of nicotina's credentials.
TEST(AriadneNetPrincipal, NicotinaWithItsOwnPolicyAndStoreSignsInAndPosts) {
  NEEDS_HTTP();
  LocalHttpServer txwtf(site(kReaderToken)), elsewhere(site(kStrangerToken));
  TempDir dir;
  const fs::path policy = dir.path / "fleet.policy";
  std::ofstream(policy) << "# the stand-in fleet\nallow " << txwtf.origin() << "\ncredential txwtf "
                        << txwtf.origin() << "\n";
  const fs::path store = dir.path / "nicotina" / "credentials";
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({elsewhere.origin()})), ""); // the shell's own, in memory
  ScopePrincipal nicotina;
  nicotina.net_policy = policy.string();
  nicotina.credential_store = store.string();
  const std::string feed = "shell.includes.nicotina", other = "shell.includes.counter";
  ASSERT_EQ(set_scope_principal(app, feed, nicotina), "");
  AppRuntime art(app, "shell");

  ASSERT_EQ(run_in(art, feed, login(txwtf)), "");
  EXPECT_EQ(val(app, feed + ".login.captured"), "true") << val(app, feed + ".login.error");
  ASSERT_EQ(run_in(art, feed,
                   "(begin (set r (http-request \"POST\" \"" + txwtf.url("/posts") +
                       "\" (dict \"auth\" \"txwtf\" \"body\" \"{\\\"text\\\": \\\"hi\\\"}\")))"
                       " (state-set \"post.status\" (get-attr r \"status\")))"),
            "");
  EXPECT_EQ(val(app, feed + ".post.status"), "201") << "it posts as the reader";
  EXPECT_EQ(auths(txwtf).back(), "Bearer " + kReaderToken);
  // Its file, not the launcher's store, holds the token.
  EXPECT_NE(read_file(store).find("local txwtf " + txwtf.origin() + " " + kReaderToken),
            std::string::npos);
  const auto launcher = cvc::ariadne::net_host_access(app);
  ASSERT_TRUE(launcher && launcher->credentials);
  EXPECT_FALSE(launcher->credentials->has("txwtf"));

  // Another local application: the launcher's network, none of nicotina's.
  ASSERT_EQ(run_in(art, other,
                   "(begin (state-set \"has\" (net-auth-has? \"txwtf\"))"
                   " (state-set \"allowed\" (net-allowed? \"" +
                       txwtf.url("/") + "\")))"),
            "");
  EXPECT_EQ(val(app, other + ".has"), "false");
  EXPECT_EQ(val(app, other + ".allowed"), "false");
  // ... and nicotina is held to its own policy: not the launcher's other site.
  ASSERT_EQ(
      run_in(art, feed, "(state-set \"allowed\" (net-allowed? \"" + elsewhere.url("/") + "\"))"),
      "");
  EXPECT_EQ(val(app, feed + ".allowed"), "false");
}

// A local application's own policy that does not load refuses its mount, rather than leaving its
// programs without a network; the same for a credential line binding an origin it does not allow.
TEST(AriadneNetPrincipal, AnOwnPolicyThatDoesNotLoadRefusesTheMount) {
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({"https://tx.wtf"})), "");
  TempDir dir;
  ScopePrincipal p;
  p.net_policy = (dir.path / "missing.policy").string();
  EXPECT_NE(set_scope_principal(app, "shell.includes.nicotina", p).find("net_policy"),
            std::string::npos);
  const fs::path bad = dir.path / "bad.policy";
  std::ofstream(bad) << "allow https://tx.wtf\ncredential txwtf https://elsewhere.test\n";
  p.net_policy = bad.string();
  EXPECT_NE(set_scope_principal(app, "shell.includes.nicotina", p).find("not allowed"),
            std::string::npos);
  EXPECT_TRUE(cvc::ariadne::scope_principals(app).empty()) << "nothing recorded";
}

// --- WebSockets, sign-ins, end-to-end keys, pictures ---------------------------------------------

namespace {

// A connection that is open until closed and keeps what it is sent.
class HeldWsConnection : public cvc::net::WsConnection {
public:
  std::vector<std::string> sent;
  bool is_open = true;
  std::optional<cvc::net::WsEvent> recv(double, const std::function<bool()> &) override {
    return std::nullopt;
  }
  bool send_text(const std::string &text, std::string *) override {
    sent.push_back(text);
    return is_open;
  }
  void close(int) override { is_open = false; }
  bool open() const override { return is_open; }
};

class HeldWsConnector : public cvc::net::WsConnector {
public:
  std::vector<std::shared_ptr<HeldWsConnection>> made;
  std::shared_ptr<cvc::net::WsConnection> connect(const std::string &, const cvc::net::WsOptions &,
                                                  std::string *) override {
    made.push_back(std::make_shared<HeldWsConnection>());
    return made.back();
  }
};

} // namespace

// A WebSocket one principal's program opened is not there for another's, by its handle.
TEST(AriadneNetPrincipal, AWebSocketIsItsOpenersAlone) {
  cvc::app app;
  auto ws = std::make_shared<HeldWsConnector>();
  NetHostConfig config = allow({"http://chat.test"});
  config.ws = ws;
  ASSERT_EQ(enable_net(app, config), "");
  const std::string feed = "shell.includes.feed", stranger = "shell.includes.stranger";
  ASSERT_EQ(set_scope_principal(app, stranger, as("remote:http://chat.test")), "");
  AppRuntime art(app, "shell");
  ASSERT_EQ(run_in(art, feed,
                   "(begin (set r (ws-open \"ws://chat.test/live\"))"
                   " (state-set \"h\" (get-attr r \"handle\")) (ws-send (get-attr r \"handle\") "
                   "\"mine\"))"),
            "");
  const std::string handle = val(app, feed + ".h");
  ASSERT_FALSE(handle.empty());
  ASSERT_EQ(run_in(art, stranger,
                   "(begin (state-set \"open\" (ws-open? \"" + handle +
                       "\"))"
                       " (state-set \"send\" (try (ws-send \"" +
                       handle +
                       "\" \"forged\") (catch (e) (get-attr e \"message\"))))"
                       " (state-set \"close\" (ws-close \"" +
                       handle + "\")))"),
            "");
  EXPECT_EQ(val(app, stranger + ".open"), "false");
  EXPECT_NE(val(app, stranger + ".send").find("no WebSocket"), std::string::npos)
      << val(app, stranger + ".send");
  EXPECT_EQ(val(app, stranger + ".close"), "false");
  ASSERT_EQ(ws->made.size(), 1u);
  EXPECT_EQ(ws->made[0]->sent, std::vector<std::string>{"mine"});
  EXPECT_TRUE(ws->made[0]->is_open) << "the stranger could not close it";
  ASSERT_EQ(run_in(art, feed, "(state-set \"open\" (ws-open? \"" + handle + "\"))"), "");
  EXPECT_EQ(val(app, feed + ".open"), "true");
}

// A sign-in is kept under its principal: another principal's (oidc-status NAME) knows nothing of
// the reader's stored sign-in of that name.
TEST(AriadneNetPrincipal, ASignInIsItsPrincipalsAlone) {
  NEEDS_HTTP();
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({"https://id.example"})), "");
  const auto launcher = cvc::ariadne::net_host_access(app);
  ASSERT_TRUE(launcher && launcher->credentials);
  ASSERT_TRUE(launcher->credentials->capture("svc", *cvc::net::parse_origin("https://id.example"),
                                             kReaderToken));
  const std::string feed = "shell.includes.feed", stranger = "shell.includes.stranger";
  ASSERT_EQ(set_scope_principal(app, stranger, as("remote:https://id.example")), "");
  AppRuntime art(app, "shell");
  const char *status = "(state-set \"phase\" (get-attr (oidc-status \"svc\") \"phase\"))";
  ASSERT_EQ(run_in(art, feed, status), "");
  ASSERT_EQ(run_in(art, stranger, status), "");
  EXPECT_EQ(val(app, feed + ".phase"), "stored");
  EXPECT_NE(val(app, stranger + ".phase"), "stored");
  // Its sign-out forgets its own, not the reader's.
  ASSERT_EQ(run_in(art, stranger, "(oidc-sign-out \"svc\" (dict \"revoke\" #f))"), "");
  EXPECT_TRUE(launcher->credentials->has("svc"));
}

// End-to-end keys: a sandboxed principal gets a store of its own, never the launcher's keys.
TEST(AriadneNetPrincipal, EndToEndKeysAreAPrincipalsOwn) {
  if (!cvc::net::e2ee_available())
    GTEST_SKIP() << "libcvc built without libsodium";
  cvc::app app;
  auto store = std::make_shared<cvc::net::E2eeStore>(cvc::net::E2eeStore::Options{});
  ASSERT_TRUE(store->open());
  cvc::ariadne::register_e2ee_intrinsics(app, store);
  const std::string feed = "shell.includes.feed", stranger = "shell.includes.stranger";
  ASSERT_EQ(set_scope_principal(app, stranger, as("remote:https://apps.example.org")), "");
  AppRuntime art(app, "shell");
  ASSERT_EQ(
      run_in(art, feed, "(state-set \"fp\" (get-attr (e2ee-create \"me\" 1) \"fingerprint\"))"),
      "");
  const std::string fp = val(app, feed + ".fp");
  ASSERT_FALSE(fp.empty());
  ASSERT_EQ(run_in(art, stranger,
                   "(begin (state-set \"seen\" (if (e2ee-identity \"me\") \"some\" \"nil\"))"
                   " (state-set \"persistent\" (get-attr (e2ee-status) \"persistent\"))"
                   " (state-set \"fp\" (get-attr (e2ee-create \"me\" 1) \"fingerprint\"))"
                   " (state-set \"forgot\" (e2ee-forget \"me\")))"),
            "");
  EXPECT_EQ(val(app, stranger + ".seen"), "nil") << "the reader's identity is not its";
  EXPECT_EQ(val(app, stranger + ".persistent"), "false");
  EXPECT_NE(val(app, stranger + ".fp"), fp);
  EXPECT_EQ(val(app, stranger + ".forgot"), "true");
  ASSERT_TRUE(store->has_identity("me")) << "its forget dropped its own";
  ASSERT_EQ(
      run_in(art, feed, "(state-set \"fp2\" (get-attr (e2ee-identity \"me\") \"fingerprint\"))"),
      "");
  EXPECT_EQ(val(app, feed + ".fp2"), fp);
}

// A picture a mounted document's widget names with auth: is fetched with its principal's
// credential and on its allowlist (media::fetch_bytes with the widget's scope).
TEST(AriadneNetPrincipal, APicturesCredentialIsItsWidgetsPrincipals) {
  NEEDS_HTTP();
  LocalHttpServer api(site(kReaderToken));
  cvc::app app;
  ASSERT_EQ(enable_net(app, allow({api.origin()})), "");
  const auto launcher = cvc::ariadne::net_host_access(app);
  ASSERT_TRUE(
      launcher->credentials->capture("txwtf", *cvc::net::parse_origin(api.origin()), kReaderToken));
  const std::string stranger = "shell.includes.stranger";
  ASSERT_EQ(
      set_scope_principal(app, stranger, as("remote:https://apps.example.org", {api.origin()})),
      "");
  using cvc::ariadne::media::fetch_bytes;
  const auto mine = fetch_bytes(app, api.url("/user/me"), "txwtf", "", "shell.includes.feed");
  EXPECT_TRUE(mine.ok) << mine.error;
  const auto theirs = fetch_bytes(app, api.url("/user/me"), "txwtf", "", stranger + ".card.3");
  EXPECT_FALSE(theirs.ok);
  EXPECT_NE(theirs.error.find("no credential"), std::string::npos) << theirs.error;
  EXPECT_EQ(theirs.error.find(kReaderToken), std::string::npos);
  ASSERT_EQ(set_scope_principal(app, stranger, as("remote:https://apps.example.org")), "");
  const auto refused = fetch_bytes(app, api.url("/echo"), "", "", stranger);
  EXPECT_FALSE(refused.ok) << "off its allowlist, though the launcher's has it";
  std::size_t sent_with_token = 0;
  for (const std::string &a : auths(api))
    sent_with_token += a == "Bearer " + kReaderToken;
  EXPECT_EQ(sent_with_token, 1u) << "only the reader's own fetch carried the token";
}
