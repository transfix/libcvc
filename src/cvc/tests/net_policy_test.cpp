// cvc::net's network policy and credential store (docs/ARIADNE.md §10.3), none of it needing the
// external network:
//   - origins and the policy file: what parses, what is refused, and the errors;
//   - send(req, policy): a refused origin is never dispatched, and redirects are followed here,
//     each hop checked (a fake transport for the rules, two localhost servers end to end);
//   - CredentialStore: binding, capture, forget and clear, and its file (owner-only, round trip);
//   - take_json_string / redact: the capture's JSON scan and the blanking.

#include "local_http_server.h"

#include <cstdlib>
#include <cvc/net/credentials.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <sys/stat.h>
#endif

using cvc::net::CredentialStore;
using cvc::net::HttpRequest;
using cvc::net::HttpResponse;
using cvc::net::NetPolicy;
using cvc::net::Origin;
using cvc_test::LocalHttpServer;

namespace fs = std::filesystem;

namespace {

Origin origin(const std::string &spec) {
  std::string why;
  const std::optional<Origin> o = cvc::net::parse_origin_spec(spec, &why);
  EXPECT_TRUE(o.has_value()) << spec << ": " << why;
  return o.value_or(Origin{});
}

// A fake transport: answers from a script, keyed on the URL, and records what it was sent.
class ScriptedClient : public cvc::net::HttpClient {
public:
  std::vector<std::pair<std::string, HttpResponse>> script;
  std::vector<HttpRequest> sent;
  HttpResponse send(const HttpRequest &req) override {
    sent.push_back(req);
    for (const auto &[url, r] : script)
      if (url == req.url) {
        HttpResponse out = r;
        out.canonical_url = req.url;
        return out;
      }
    HttpResponse r;
    r.ok = true;
    r.status = 404;
    r.canonical_url = req.url;
    return r;
  }
};

struct ClientGuard {
  ~ClientGuard() { cvc::net::set_http_client(nullptr); }
};

HttpResponse redirect(long status, const std::string &location) {
  HttpResponse r;
  r.ok = true;
  r.status = status;
  r.headers = {"Location: " + location};
  return r;
}

HttpResponse ok_body(const std::string &body) {
  HttpResponse r;
  r.ok = true;
  r.status = 200;
  r.body = body;
  return r;
}

bool has_header(const std::vector<std::string> &lines, const std::string &prefix) {
  for (const std::string &l : lines)
    if (l.rfind(prefix, 0) == 0)
      return true;
  return false;
}

// A fresh directory under the system temp dir, removed on scope-out.
struct TempDir {
  fs::path path;
  TempDir() {
    std::random_device rd;
    path =
        fs::temp_directory_path() / ("cvc-net-test-" + std::to_string(rd()) + std::to_string(rd()));
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

const std::string kToken = "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJ0ZXN0In0.c2lnbmF0dXJl";

} // namespace

// --- origins and the policy file ---------------------------------------------------------------

TEST(NetOrigin, ParsesSchemeHostAndPortWithDefaults) {
  const std::optional<Origin> a = cvc::net::parse_origin("HTTPS://TX.wtf/user/login?x=1#f");
  ASSERT_TRUE(a);
  EXPECT_EQ(a->scheme, "https");
  EXPECT_EQ(a->host, "tx.wtf");
  EXPECT_EQ(a->port, 443);
  EXPECT_EQ(a->str(), "https://tx.wtf");
  EXPECT_EQ(*cvc::net::parse_origin("https://tx.wtf:443"), *a) << "an explicit default port";
  const std::optional<Origin> b = cvc::net::parse_origin("http://127.0.0.1:8080");
  ASSERT_TRUE(b);
  EXPECT_EQ(b->port, 8080);
  EXPECT_EQ(b->str(), "http://127.0.0.1:8080");
  const std::optional<Origin> v6 = cvc::net::parse_origin("http://[::1]:9000/x");
  ASSERT_TRUE(v6);
  EXPECT_EQ(v6->host, "[::1]");
  EXPECT_NE(*cvc::net::parse_origin("http://tx.wtf"), *a) << "the scheme is part of the origin";
}

TEST(NetOrigin, RefusesWhatCouldBeReadTwoWays) {
  std::string why;
  for (const char *bad :
       {"ftp://tx.wtf/", "file:///etc/passwd", "tx.wtf", "https://user:pw@tx.wtf/",
        "https://allowed.example@evil.example/", "https://evil.example\\@allowed.example/",
        "https://tx.wtf /x", "https://tx.wtf:0/", "https://tx.wtf:70000/", "https://tx.wtf:8a/",
        "https:///x", "https://t_x.wtf/", "https://xn--bcher-kva.example\x01/", "https://[::1/"}) {
    why.clear();
    EXPECT_FALSE(cvc::net::parse_origin(bad, &why)) << bad;
    EXPECT_FALSE(why.empty()) << bad;
  }
  EXPECT_TRUE(cvc::net::parse_origin("https://xn--bcher-kva.example/")) << "punycode is fine";
}

TEST(NetOrigin, AnOriginSpecIsNotAUrl) {
  EXPECT_TRUE(cvc::net::parse_origin_spec("https://tx.wtf"));
  EXPECT_TRUE(cvc::net::parse_origin_spec("https://tx.wtf/")) << "a trailing slash is allowed";
  std::string why;
  EXPECT_FALSE(cvc::net::parse_origin_spec("https://tx.wtf/api", &why));
  EXPECT_NE(why.find("not an origin"), std::string::npos) << why;
}

TEST(NetPolicyFile, AllowAndCredentialLines) {
  NetPolicy p;
  std::string why;
  ASSERT_TRUE(cvc::net::parse_net_policy("# nicotina\n"
                                         "allow https://tx.wtf   # the API\n"
                                         "\n"
                                         "allow http://127.0.0.1:8080\n"
                                         "credential txwtf https://tx.wtf http://127.0.0.1:8080\n",
                                         p, &why))
      << why;
  ASSERT_EQ(p.allowed().size(), 2u);
  EXPECT_TRUE(p.allows_url("https://tx.wtf/user/login"));
  EXPECT_TRUE(p.allows_url("http://127.0.0.1:8080/x"));
  EXPECT_FALSE(p.allows_url("http://127.0.0.1:8081/x"));
  EXPECT_FALSE(p.allows_url("http://tx.wtf/"));
  ASSERT_EQ(p.credential_bindings().count("txwtf"), 1u);
  EXPECT_EQ(p.credential_bindings().at("txwtf").size(), 2u);

  why.clear();
  EXPECT_FALSE(p.allows_url("https://evil.example/", &why));
  EXPECT_NE(why.find("https://evil.example is not an origin this host allows"), std::string::npos)
      << why;
}

TEST(NetPolicyFile, ErrorsNameTheLine) {
  struct Case {
    const char *text, *expect;
  } cases[] = {
      {"allow https://tx.wtf\nalow https://x.example\n", "policy:2: unknown directive 'alow'"},
      {"allow\n", "policy:1: allow takes one origin"},
      {"allow https://a.example https://b.example\n", "policy:1: allow takes one origin"},
      {"credential txwtf\n", "policy:1: credential takes a name and at least one origin"},
      {"allow https://tx.wtf/api\n", "policy:1: 'https://tx.wtf/api' is a URL"},
  };
  for (const Case &c : cases) {
    NetPolicy p;
    std::string why;
    EXPECT_FALSE(cvc::net::parse_net_policy(c.text, p, &why, "policy")) << c.text;
    EXPECT_NE(why.find(c.expect), std::string::npos) << why;
  }
  NetPolicy p;
  std::string why;
  EXPECT_FALSE(cvc::net::load_net_policy_file("/nonexistent/net.policy", p, &why));
  EXPECT_NE(why.find("cannot read"), std::string::npos) << why;
}

TEST(NetPolicyFile, LoadsAFile) {
  TempDir dir;
  const fs::path file = dir.path / "net.policy";
  std::ofstream(file) << "allow https://tx.wtf\n";
  NetPolicy p;
  std::string why;
  ASSERT_TRUE(cvc::net::load_net_policy_file(file.string(), p, &why)) << why;
  EXPECT_TRUE(p.allows(origin("https://tx.wtf")));
}

TEST(NetPolicy, DefaultAllowsNothing) {
  NetPolicy p;
  EXPECT_FALSE(p.allows_url("https://tx.wtf/"));
  EXPECT_FALSE(p.allows_url("http://127.0.0.1/"));
}

// --- send(req, policy) -------------------------------------------------------------------------

TEST(NetCheckedSend, RefusedOriginIsNeverDispatched) {
  ClientGuard guard;
  auto *fake = new ScriptedClient;
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  NetPolicy p;
  p.allow(origin("https://tx.wtf"));
  HttpRequest req;
  req.url = "https://evil.example/steal";
  const HttpResponse r = cvc::net::send(req, p);
  EXPECT_FALSE(r.ok);
  EXPECT_EQ(r.status, 0);
  EXPECT_NE(r.error.find("refused"), std::string::npos) << r.error;
  EXPECT_NE(r.error.find("https://evil.example"), std::string::npos) << r.error;
  EXPECT_TRUE(fake->sent.empty());

  req.url = "https://tx.wtf/ok";
  fake->script = {{"https://tx.wtf/ok", ok_body("fine")}};
  const HttpResponse ok = cvc::net::send(req, p);
  EXPECT_TRUE(ok.ok);
  EXPECT_EQ(ok.body, "fine");
  ASSERT_EQ(fake->sent.size(), 1u);
  EXPECT_FALSE(fake->sent[0].follow_redirects) << "the transport never follows on its own";
}

TEST(NetCheckedSend, RedirectsAreCheckedHopByHop) {
  ClientGuard guard;
  auto *fake = new ScriptedClient;
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  NetPolicy p;
  p.allow(origin("https://tx.wtf"));
  p.allow(origin("https://media.tx.wtf"));
  fake->script = {
      {"https://tx.wtf/a", redirect(302, "/b")},                         // same origin
      {"https://tx.wtf/b", redirect(301, "https://media.tx.wtf/c?x=1")}, // allowed origin
      {"https://media.tx.wtf/c?x=1", ok_body("arrived")},
      {"https://tx.wtf/out", redirect(302, "https://evil.example/")}, // not allowed
      {"https://tx.wtf/rel/x", redirect(307, "y")},                   // relative path
      {"https://tx.wtf/rel/y", ok_body("relative")},
      {"https://tx.wtf/loop", redirect(302, "/loop")},
  };
  HttpRequest req;
  req.url = "https://tx.wtf/a";
  req.headers = {"Authorization: Bearer secret-for-tx", "X-Keep: 1"};
  HttpResponse r = cvc::net::send(req, p);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.body, "arrived");
  ASSERT_EQ(fake->sent.size(), 3u);
  EXPECT_TRUE(has_header(fake->sent[1].headers, "Authorization:")) << "same origin keeps it";
  EXPECT_FALSE(has_header(fake->sent[2].headers, "Authorization:")) << "another origin drops it";
  EXPECT_TRUE(has_header(fake->sent[2].headers, "X-Keep:"));

  fake->sent.clear();
  req.url = "https://tx.wtf/out";
  r = cvc::net::send(req, p);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("https://evil.example is not an origin this host allows"),
            std::string::npos)
      << r.error;
  EXPECT_EQ(fake->sent.size(), 1u) << "the disallowed hop is never requested";

  req.url = "https://tx.wtf/rel/x";
  r = cvc::net::send(req, p);
  EXPECT_EQ(r.body, "relative");

  req.url = "https://tx.wtf/loop";
  req.max_redirects = 3;
  r = cvc::net::send(req, p);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("more than 3 redirects"), std::string::npos) << r.error;

  fake->sent.clear();
  req.url = "https://tx.wtf/out";
  req.follow_redirects = false; // the caller does not follow: the 3xx itself is the reply
  r = cvc::net::send(req, p);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.status, 302);
  EXPECT_EQ(fake->sent.size(), 1u);
}

TEST(NetCheckedSend, SeeOtherAndPostRedirectsBecomeBodylessGets) {
  ClientGuard guard;
  auto *fake = new ScriptedClient;
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));
  NetPolicy p;
  p.allow(origin("https://tx.wtf"));
  fake->script = {{"https://tx.wtf/form", redirect(303, "/done")},
                  {"https://tx.wtf/keep", redirect(307, "/done")},
                  {"https://tx.wtf/done", ok_body("done")}};
  HttpRequest req;
  req.url = "https://tx.wtf/form";
  req.method = "POST";
  req.body = "a=1";
  req.headers = {"Content-Type: application/x-www-form-urlencoded"};
  ASSERT_TRUE(cvc::net::send(req, p).ok);
  ASSERT_EQ(fake->sent.size(), 2u);
  EXPECT_EQ(fake->sent[1].method, "GET");
  EXPECT_TRUE(fake->sent[1].body.empty());
  EXPECT_FALSE(has_header(fake->sent[1].headers, "Content-Type:"));

  fake->sent.clear();
  req.url = "https://tx.wtf/keep";
  ASSERT_TRUE(cvc::net::send(req, p).ok);
  ASSERT_EQ(fake->sent.size(), 2u);
  EXPECT_EQ(fake->sent[1].method, "POST") << "307 keeps the method";
  EXPECT_EQ(fake->sent[1].body, "a=1");
}

// End to end over the real backend: server A redirects to server B, another origin (another
// port). Allowed, the redirect is followed; not allowed, B never sees a request.
TEST(NetCheckedSend, LocalServersRedirectAcrossOrigins) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "built without an HTTP backend";
  LocalHttpServer b([](const LocalHttpServer::Request &) {
    return LocalHttpServer::Reply{200, {"Content-Type: text/plain"}, "from-b"};
  });
  const std::string to_b = b.url("/landing");
  LocalHttpServer a([&](const LocalHttpServer::Request &req) {
    if (req.target == "/go")
      return LocalHttpServer::Reply{302, {"Location: " + to_b}, ""};
    return LocalHttpServer::Reply{200, {}, "from-a"};
  });
  HttpRequest req;
  req.url = a.url("/go");
  req.headers = {"Authorization: Bearer for-a-only"};

  NetPolicy only_a;
  only_a.allow(origin(a.origin()));
  HttpResponse r = cvc::net::send(req, only_a);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find(b.origin() + " is not an origin this host allows"), std::string::npos)
      << r.error;
  EXPECT_TRUE(b.requests().empty()) << "the refused hop must not be requested";

  NetPolicy both = only_a;
  both.allow(origin(b.origin()));
  r = cvc::net::send(req, both);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.status, 200);
  EXPECT_EQ(r.body, "from-b");
  EXPECT_EQ(cvc::net::response_header(r.headers, "content-type"), "text/plain");
  ASSERT_EQ(b.requests().size(), 1u);
  EXPECT_EQ(b.requests()[0].header("Authorization"), "") << "dropped on the cross-origin hop";
  ASSERT_EQ(a.requests().size(), 2u);
  EXPECT_EQ(a.requests()[0].header("Authorization"), "Bearer for-a-only");

  NetPolicy none;
  req.url = a.url("/plain");
  EXPECT_FALSE(cvc::net::send(req, none).ok);
  EXPECT_EQ(a.requests().size(), 2u) << "a default policy reaches nothing";
}

// --- the credential store ----------------------------------------------------------------------

TEST(NetCredentials, AnUnboundCredentialGoesOnlyWhereItCameFrom) {
  CredentialStore store;
  const Origin tx = origin("https://tx.wtf"), other = origin("https://other.example");
  std::string why;
  ASSERT_TRUE(store.capture("txwtf", tx, kToken, &why)) << why;
  EXPECT_TRUE(why.empty());
  EXPECT_EQ(store.token_for("txwtf", tx), kToken);
  EXPECT_FALSE(store.token_for("txwtf", other));
  EXPECT_FALSE(store.token_for("nope", tx));
  EXPECT_TRUE(store.has("txwtf"));
  EXPECT_TRUE(store.has("txwtf", tx));
  EXPECT_FALSE(store.has("txwtf", other));
  // The same name from another origin is its own credential, bound there.
  const std::string other_token = "b3RoZXItb3JpZ2luLXRva2Vu";
  ASSERT_TRUE(store.capture("txwtf", other, other_token));
  EXPECT_EQ(store.token_for("txwtf", tx), kToken);
  EXPECT_EQ(store.token_for("txwtf", other), other_token);
  EXPECT_EQ(store.names(), (std::vector<std::string>{"txwtf"}));
}

TEST(NetCredentials, ABoundCredentialGoesToItsOriginsOnly) {
  CredentialStore store;
  const Origin tx = origin("https://tx.wtf"), api = origin("https://api.tx.wtf"),
               evil = origin("https://evil.example");
  store.bind("txwtf", {tx, api});
  std::string why;
  EXPECT_FALSE(store.capture("txwtf", evil, kToken, &why));
  EXPECT_NE(why.find("not bound to https://evil.example"), std::string::npos) << why;
  EXPECT_EQ(why.find(kToken), std::string::npos) << "an error never quotes the token";
  EXPECT_FALSE(store.has("txwtf"));

  ASSERT_TRUE(store.capture("txwtf", tx, kToken));
  EXPECT_EQ(store.token_for("txwtf", tx), kToken);
  EXPECT_EQ(store.token_for("txwtf", api), kToken) << "bound to both";
  EXPECT_FALSE(store.token_for("txwtf", evil));
  // A new capture from another bound origin replaces it: a bound name holds one token.
  const std::string next = "bmV4dC10b2tlbi1mcm9tLWFwaQ";
  ASSERT_TRUE(store.capture("txwtf", api, next));
  EXPECT_EQ(store.token_for("txwtf", tx), next);
}

TEST(NetCredentials, RefusesMalformedNamesAndTokens) {
  CredentialStore store;
  const Origin tx = origin("https://tx.wtf");
  std::string why;
  EXPECT_FALSE(store.capture("bad name", tx, kToken, &why));
  EXPECT_FALSE(store.capture("", tx, kToken, &why));
  EXPECT_FALSE(store.capture(std::string(65, 'a'), tx, kToken, &why));
  for (const std::string &bad :
       {std::string("short"), kToken + "\r\nX-Injected: 1", std::string("has space in it ok"),
        std::string(5000, 'a'), std::string("=================")}) {
    why.clear();
    EXPECT_FALSE(store.capture("txwtf", tx, bad, &why)) << bad;
    EXPECT_EQ(why.find(bad), std::string::npos) << "the error must not quote it";
  }
  EXPECT_TRUE(CredentialStore::valid_token("YWJjZGVmZ2hpamtsbW5vcA=="));
  EXPECT_TRUE(CredentialStore::valid_token(kToken));
  EXPECT_FALSE(store.has("txwtf"));
}

TEST(NetCredentials, ForgetAndClear) {
  CredentialStore store;
  const Origin tx = origin("https://tx.wtf");
  ASSERT_TRUE(store.capture("txwtf", tx, kToken));
  ASSERT_TRUE(store.capture("other", tx, "YW5vdGhlci10b2tlbi12YWx1ZQ"));
  EXPECT_TRUE(store.forget("txwtf"));
  EXPECT_FALSE(store.forget("txwtf")) << "nothing left to forget";
  EXPECT_FALSE(store.token_for("txwtf", tx));
  EXPECT_TRUE(store.has("other"));
  store.clear();
  EXPECT_TRUE(store.names().empty());
}

TEST(NetCredentials, PersistsOwnerOnlyAndRoundTrips) {
  TempDir dir;
  const fs::path file = dir.path / "cvc" / "credentials" / "nicotina";
  const Origin tx = origin("https://tx.wtf");
  {
    CredentialStore store(file.string());
    ASSERT_TRUE(store.load()) << "a missing file holds nothing";
    std::string why;
    ASSERT_TRUE(store.capture("txwtf", tx, kToken, &why)) << why;
    EXPECT_TRUE(why.empty()) << why;
  }
  ASSERT_TRUE(fs::exists(file));
  const std::string text = read_file(file);
  EXPECT_NE(text.find("txwtf https://tx.wtf " + kToken + "\n"), std::string::npos) << text;
#if !defined(_WIN32)
  struct stat st {};
  ASSERT_EQ(::stat(file.c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0600u);
  ASSERT_EQ(::stat(file.parent_path().c_str(), &st), 0);
  EXPECT_EQ(st.st_mode & 0777, 0700u) << "the directory it made is the user's alone";
#endif
  {
    CredentialStore again(file.string());
    std::string why;
    ASSERT_TRUE(again.load(&why)) << why;
    EXPECT_EQ(again.token_for("txwtf", tx), kToken);
    EXPECT_TRUE(again.forget("txwtf", &why)) << why;
  }
  EXPECT_EQ(read_file(file).find(kToken), std::string::npos) << "forget rewrites the file";
  CredentialStore empty(file.string());
  ASSERT_TRUE(empty.load());
  EXPECT_FALSE(empty.has("txwtf"));
}

#if !defined(_WIN32)
TEST(NetCredentials, RefusesAFileOthersCanRead) {
  TempDir dir;
  const fs::path file = dir.path / "creds";
  std::ofstream(file) << "txwtf https://tx.wtf " << kToken << "\n";
  ::chmod(file.c_str(), 0644);
  CredentialStore store(file.string());
  std::string why;
  EXPECT_FALSE(store.load(&why));
  EXPECT_NE(why.find("open to other users"), std::string::npos) << why;
  EXPECT_FALSE(store.has("txwtf"));
  ::chmod(file.c_str(), 0600);
  EXPECT_TRUE(store.load(&why)) << why;
  EXPECT_TRUE(store.has("txwtf"));
}
#endif

TEST(NetCredentials, AMalformedFileIsRefusedWithoutQuotingIt) {
  TempDir dir;
  const fs::path file = dir.path / "creds";
  {
    std::ofstream out(file);
    out << "# header\ntxwtf https://tx.wtf " << kToken << " trailing\n";
  }
  fs::permissions(file, fs::perms::owner_read | fs::perms::owner_write);
  CredentialStore store(file.string());
  std::string why;
  EXPECT_FALSE(store.load(&why));
  EXPECT_NE(why.find("line 2"), std::string::npos) << why;
  EXPECT_EQ(why.find(kToken), std::string::npos) << why;
}

TEST(NetCredentials, AStoreNameLivesInTheConfigDir) {
#if defined(_WIN32)
  GTEST_SKIP() << "the POSIX config dir";
#else
  const char *old = std::getenv("XDG_CONFIG_HOME");
  const std::string saved = old ? old : "";
  ::setenv("XDG_CONFIG_HOME", "/tmp/xdg-test", 1);
  EXPECT_EQ(CredentialStore::resolve_path("nicotina"), "/tmp/xdg-test/cvc/credentials/nicotina");
  EXPECT_EQ(CredentialStore::resolve_path("./nicotina"), "./nicotina") << "a path is a path";
  EXPECT_EQ(CredentialStore::resolve_path("/var/x/creds"), "/var/x/creds");
  if (old)
    ::setenv("XDG_CONFIG_HOME", saved.c_str(), 1);
  else
    ::unsetenv("XDG_CONFIG_HOME");
#endif
}

// --- the capture's JSON scan and the blanking --------------------------------------------------

TEST(NetCapture, TakesTheFieldAndLeavesEveryOtherByte) {
  std::string body =
      "{\"user\": {\"name\": \"joe\", \"token\": \"not-this\"}, \"expires\": 1730000000,"
      " \"token\": \"" +
      kToken + "\", \"session_uuid\": \"abc\"}";
  const std::string expect =
      "{\"user\": {\"name\": \"joe\", \"token\": \"not-this\"}, \"expires\": "
      "1730000000, \"token\": \"[redacted]\", \"session_uuid\": \"abc\"}";
  std::string token;
  ASSERT_TRUE(cvc::net::take_json_string(body, "token", token));
  EXPECT_EQ(token, kToken);
  EXPECT_EQ(body, expect);

  std::string nested = "{\"data\": {\"auth\": {\"token\": \"x\\u0041\\/y\"}}}";
  ASSERT_TRUE(cvc::net::take_json_string(nested, "data.auth.token", token));
  EXPECT_EQ(token, "xA/y") << "escapes decoded";
  EXPECT_EQ(nested, "{\"data\": {\"auth\": {\"token\": \"[redacted]\"}}}");
}

TEST(NetCapture, NoStringNoChange) {
  for (const std::string &text : {std::string("{\"token\": 42}"), std::string("{\"other\": \"x\"}"),
                                  std::string("[\"token\"]"), std::string("not json"),
                                  std::string("{\"token\": \"unterminated}"), std::string(""),
                                  std::string("{\"a\": [1, {\"b\": 2}], \"token\" 3}")}) {
    std::string body = text, token;
    EXPECT_FALSE(cvc::net::take_json_string(body, "token", token)) << text;
    EXPECT_EQ(body, text);
  }
  std::string body = "{\"a\": [1, {\"b\": \"c\"}, null, true], \"token\": \"t\"}", token;
  EXPECT_TRUE(cvc::net::take_json_string(body, "token", token)) << "skips arrays and objects";
  EXPECT_EQ(token, "t");
}

TEST(NetCapture, RedactBlanksEveryCopy) {
  std::string text = "a " + kToken + " b " + kToken;
  EXPECT_EQ(cvc::net::redact(text, kToken), 2u);
  EXPECT_EQ(text, "a [redacted] b [redacted]");
  EXPECT_EQ(cvc::net::redact(text, ""), 0u);
}
