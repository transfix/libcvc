// cvc::net::oidc — the OAuth 2.0 / OpenID Connect client's protocol pieces (docs/ARIADNE.md §10.5):
// issuers and discovery (fail closed), PKCE, the authorization request and reply, token and device
// replies, ID token checks, the policy's oidc lines, the loopback redirect listener and the browser
// seam. The flows themselves are tested through the Ariadne host (ariadne_runtime_test,
// AriadneOidc.*) and the ari binary (ari_net_test, AriOidc.*).

#include <atomic>
#include <chrono>
#include <cvc/core/app.h>
#include <cvc/net/browser.h>
#include <cvc/net/http_client.h>
#include <cvc/net/loopback_redirect.h>
#include <cvc/net/net_policy.h>
#include <cvc/net/oidc.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace o = cvc::net::oidc;
using cvc::net::Origin;

namespace {

std::string discovery(const std::string &issuer, const std::string &extra = "") {
  return "{\"issuer\":\"" + issuer + "\",\"authorization_endpoint\":\"" + issuer +
         "/authorize\",\"token_endpoint\":\"" + issuer + "/token\"" + extra + "}";
}

std::string jwt(const std::string &claims, const std::string &header = "{\"alg\":\"RS256\"}") {
  return o::base64url_encode(header) + "." + o::base64url_encode(claims) + ".c2ln";
}

} // namespace

TEST(NetOidc, IssuersAreHttpsOrLoopbackWithNoQuery) {
  EXPECT_EQ(o::check_issuer("https://id.example/realms/demo"), "");
  EXPECT_EQ(o::check_issuer("http://127.0.0.1:8080"), "");
  EXPECT_EQ(o::check_issuer("http://localhost:9000/x"), "");
  EXPECT_EQ(o::check_issuer("http://[::1]:9000"), "");
  EXPECT_NE(o::check_issuer("http://id.example"), "");
  EXPECT_NE(o::check_issuer("https://id.example/?tenant=a"), "");
  EXPECT_NE(o::check_issuer("https://id.example/#x"), "");
  EXPECT_NE(o::check_issuer("https://user@id.example"), "");
  EXPECT_NE(o::check_issuer("ftp://id.example"), "");
  EXPECT_TRUE(o::is_loopback_host("127.0.0.1"));
  EXPECT_TRUE(o::is_loopback_host("127.10.0.3"));
  EXPECT_FALSE(o::is_loopback_host("127.0.0.1.evil.example"));
  EXPECT_FALSE(o::is_loopback_host("128.0.0.1"));
  EXPECT_EQ(o::discovery_url("https://id.example/realms/demo/"),
            "https://id.example/realms/demo/.well-known/openid-configuration");
  EXPECT_EQ(o::discovery_url("https://id.example"),
            "https://id.example/.well-known/openid-configuration");
}

TEST(NetOidc, DiscoveryIsCheckedAndFailsClosed) {
  const std::string iss = "https://id.example/realms/demo";
  std::string why;
  std::optional<o::Provider> p = o::parse_discovery(
      discovery(iss, ",\"device_authorization_endpoint\":\"" + iss +
                         "/device\",\"code_challenge_methods_supported\":[\"plain\",\"S256\"],"
                         "\"authorization_response_iss_parameter_supported\":true"),
      iss, {}, &why);
  ASSERT_TRUE(p) << why;
  EXPECT_EQ(p->token_endpoint, iss + "/token");
  EXPECT_EQ(p->device_authorization_endpoint, iss + "/device");
  EXPECT_TRUE(p->iss_parameter);

  // The issuer must match exactly; a trailing slash is another issuer, and the error says which.
  EXPECT_FALSE(o::parse_discovery(discovery(iss + "/"), iss, {}, &why));
  EXPECT_NE(why.find("must match exactly"), std::string::npos) << why;
  EXPECT_FALSE(o::parse_discovery("not json", iss, {}, &why));
  EXPECT_FALSE(o::parse_discovery("[1]", iss, {}, &why));
  EXPECT_FALSE(o::parse_discovery("{\"issuer\":\"" + iss + "\"}", iss, {}, &why));
  EXPECT_NE(why.find("token_endpoint"), std::string::npos) << why;
  // No http endpoint for an https issuer.
  EXPECT_FALSE(o::parse_discovery("{\"issuer\":\"" + iss +
                                      "\",\"token_endpoint\":\"http://id.example/token\"}",
                                  iss, {}, &why));
  EXPECT_NE(why.find("not https"), std::string::npos) << why;
  // An endpoint at another origin, unless the policy lists it for this issuer.
  const std::string foreign =
      "{\"issuer\":\"" + iss + "\",\"token_endpoint\":\"https://oauth2.example/token\"}";
  EXPECT_FALSE(o::parse_discovery(foreign, iss, {}, &why));
  EXPECT_NE(why.find("not the issuer's origin"), std::string::npos) << why;
  EXPECT_TRUE(
      o::parse_discovery(foreign, iss, {*cvc::net::parse_origin_spec("https://oauth2.example")}));
  // PKCE S256 may not be ruled out.
  EXPECT_FALSE(o::parse_discovery(
      discovery(iss, ",\"code_challenge_methods_supported\":[\"plain\"]"), iss, {}, &why));
  EXPECT_NE(why.find("S256"), std::string::npos) << why;
  // A loopback issuer (tests, a local provider) may use http on loopback only.
  const std::string local = "http://127.0.0.1:4444";
  EXPECT_TRUE(o::parse_discovery(discovery(local), local, {}));
}

TEST(NetOidc, PkceS256MatchesRfc7636AppendixB) {
  EXPECT_EQ(o::s256_challenge("dBjftJeZ4CVP-mB92K27uhbUJU1p1r_wW1gFWFOEjXk"),
            "E9Melhoa2OwvFrEMTJguCHaoeK1t8URWbuGJSstw-cM");
  const std::string a = o::random_token(32), b = o::random_token(32);
  EXPECT_EQ(a.size(), 43u);
  EXPECT_NE(a, b);
  EXPECT_EQ(a.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_"),
            std::string::npos);
  EXPECT_EQ(o::base64url_decode(o::base64url_encode("any\x01\xff bytes")).value(),
            "any\x01\xff bytes");
}

TEST(NetOidc, TheAuthorizationRequestCarriesPkceStateAndNonce) {
  o::Provider p;
  p.issuer = "https://id.example";
  p.authorization_endpoint = "https://id.example/auth?tenant=a";
  o::AuthorizationRequest r{
      "cli ent", "http://127.0.0.1:5000/callback",        "openid email", "st", "nn",
      "ch",      {{"prompt", "login"}, {"state", "evil"}}};
  const std::string url = o::authorization_url(p, r);
  EXPECT_EQ(
      url.rfind("https://id.example/auth?tenant=a&response_type=code&client_id=cli%20ent&", 0), 0u)
      << url;
  for (const char *want :
       {"redirect_uri=http%3A%2F%2F127.0.0.1%3A5000%2Fcallback", "scope=openid%20email", "state=st",
        "nonce=nn", "code_challenge=ch", "code_challenge_method=S256", "prompt=login"})
    EXPECT_NE(url.find(want), std::string::npos) << want << " in " << url;
  EXPECT_EQ(url.find("state=evil"), std::string::npos) << "a caller cannot set the host's own";
  EXPECT_FALSE(o::caller_param("code_verifier"));
  EXPECT_FALSE(o::caller_param("redirect_uri"));
  EXPECT_TRUE(o::caller_param("login_hint"));
}

TEST(NetOidc, TheRedirectIsCheckedForStateIssAndErrors) {
  o::Provider p;
  p.issuer = "https://id.example";
  auto r = o::parse_authorization_response("code=abc&state=s1&iss=https%3A%2F%2Fid.example");
  EXPECT_EQ(r.code, "abc");
  EXPECT_EQ(o::check_authorization_response(r, p, "s1"), "");
  EXPECT_NE(o::check_authorization_response(r, p, "s2").find("state"), std::string::npos);
  r = o::parse_authorization_response("code=abc&state=s1&iss=https%3A%2F%2Fevil.example");
  EXPECT_NE(o::check_authorization_response(r, p, "s1").find("iss"), std::string::npos);
  r = o::parse_authorization_response("code=abc&state=s1"); // no iss: fine unless promised
  EXPECT_EQ(o::check_authorization_response(r, p, "s1"), "");
  p.iss_parameter = true;
  EXPECT_NE(o::check_authorization_response(r, p, "s1"), "");
  p.iss_parameter = false;
  r = o::parse_authorization_response("error=access_denied&error_description=No+way&state=s1");
  EXPECT_NE(o::check_authorization_response(r, p, "s1").find("access_denied: No way"),
            std::string::npos);
}

TEST(NetOidc, TokenAndDeviceReplies) {
  o::TokenReply t = o::parse_token_reply(
      200,
      "{\"access_token\":\"AAAAAAAAAAAAAAAAAAAA\",\"token_type\":\"bearer\",\"expires_in\":\"60\","
      "\"refresh_token\":\"RRRRRRRRRRRRRRRRRRRR\"}");
  EXPECT_TRUE(t.ok);
  EXPECT_EQ(*t.expires_in, 60);
  t = o::parse_token_reply(400, "{\"error\":\"authorization_pending\"}");
  EXPECT_FALSE(t.ok);
  EXPECT_EQ(t.error, "authorization_pending");
  t = o::parse_token_reply(200,
                           "{\"access_token\":\"AAAAAAAAAAAAAAAAAAAA\",\"token_type\":\"DPoP\"}");
  EXPECT_FALSE(t.ok);
  EXPECT_TRUE(t.access_token.empty()) << "a refused reply's tokens are wiped";
  EXPECT_FALSE(o::parse_token_reply(502, "<html>").ok);

  std::string why;
  auto d = o::parse_device_authorization(
      200,
      "{\"device_code\":\"dc\",\"user_code\":\"ABCD-EFGH\",\"verification_uri\":"
      "\"https://id.example/device\",\"expires_in\":600}",
      &why);
  ASSERT_TRUE(d) << why;
  EXPECT_EQ(d->interval, 5) << "RFC 8628's default";
  EXPECT_FALSE(o::parse_device_authorization(
      200,
      "{\"device_code\":\"dc\",\"user_code\":\"x\",\"verification_uri\":\"http://evil.example\","
      "\"expires_in\":600}",
      &why));
  EXPECT_FALSE(o::parse_device_authorization(400, "{\"error\":\"unauthorized_client\"}", &why));
  EXPECT_NE(why.find("unauthorized_client"), std::string::npos);
}

TEST(NetOidc, IdTokenChecks) {
  const long now = 1'800'000'000;
  o::IdTokenExpect e;
  e.issuer = "https://id.example";
  e.client_id = "app";
  e.nonce = "n1";
  e.now = now;
  const auto claims = [&](const std::string &extra) {
    return "{\"iss\":\"https://id.example\",\"sub\":\"u1\",\"exp\":" + std::to_string(now + 300) +
           ",\"iat\":" + std::to_string(now) + extra + "}";
  };
  const auto check = [&](const std::string &token) {
    std::string why;
    const auto c = o::decode_id_token(token, &why);
    return c ? o::check_id_token(*c, e) : "decode: " + why;
  };
  EXPECT_EQ(check(jwt(claims(",\"aud\":\"app\",\"nonce\":\"n1\",\"name\":\"Ada\""))), "");
  EXPECT_EQ(check(jwt(claims(",\"aud\":[\"app\"],\"nonce\":\"n1\""))), "");
  EXPECT_NE(check(jwt(claims(",\"aud\":\"other\",\"nonce\":\"n1\""))).find("aud"),
            std::string::npos);
  EXPECT_NE(check(jwt(claims(",\"aud\":[\"app\",\"api\"],\"nonce\":\"n1\""))).find("azp"),
            std::string::npos);
  EXPECT_EQ(check(jwt(claims(",\"aud\":[\"app\",\"api\"],\"azp\":\"app\",\"nonce\":\"n1\""))), "");
  EXPECT_NE(check(jwt(claims(",\"aud\":\"app\",\"nonce\":\"replayed\""))).find("nonce"),
            std::string::npos);
  EXPECT_NE(check(jwt(claims(",\"aud\":\"app\""))).find("nonce"), std::string::npos);
  EXPECT_NE(
      check(jwt("{\"iss\":\"https://evil.example\",\"sub\":\"u\",\"aud\":\"app\"}")).find("iss"),
      std::string::npos);
  EXPECT_NE(check(jwt("{\"iss\":\"https://id.example\",\"sub\":\"u\",\"aud\":\"app\",\"nonce\":"
                      "\"n1\",\"exp\":" +
                      std::to_string(now - 600) + ",\"iat\":" + std::to_string(now - 900) + "}"))
                .find("expired"),
            std::string::npos);
  EXPECT_NE(check(jwt("{\"iss\":\"https://id.example\",\"sub\":\"u\",\"aud\":\"app\",\"nonce\":"
                      "\"n1\",\"exp\":" +
                      std::to_string(now + 9000) + ",\"iat\":" + std::to_string(now + 3600) + "}"))
                .find("iat"),
            std::string::npos);
  EXPECT_NE(check(jwt(claims(",\"aud\":\"app\",\"nonce\":\"n1\""), "{\"alg\":\"none\"}"))
                .find("unsigned"),
            std::string::npos);
  EXPECT_NE(check("not.a-jwt").find("decode"), std::string::npos);
  e.nonce.reset(); // a refresh: no nonce sent, the same subject
  e.sub = "u1";
  EXPECT_EQ(check(jwt(claims(",\"aud\":\"app\""))), "");
  e.sub = "someone-else";
  EXPECT_NE(check(jwt(claims(",\"aud\":\"app\""))).find("subject"), std::string::npos);
}

TEST(NetOidc, PolicyOidcLinesPinIssuersAndAllowTheirOrigins) {
  cvc::net::NetPolicy p;
  std::string why;
  ASSERT_TRUE(cvc::net::parse_net_policy("oidc https://id.example/realms/demo\n"
                                         "oidc https://accounts.example https://oauth2.example\n"
                                         "oidc-client https://accounts.example 123.apps.example\n",
                                         p, &why))
      << why;
  EXPECT_TRUE(p.allows(*cvc::net::parse_origin_spec("https://id.example")));
  EXPECT_TRUE(p.allows(*cvc::net::parse_origin_spec("https://oauth2.example")));
  ASSERT_EQ(p.oidc_issuers().size(), 2u);
  const cvc::net::OidcIssuer *g = p.oidc_issuer("https://accounts.example");
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->client_id, "123.apps.example");
  ASSERT_EQ(g->endpoint_origins.size(), 1u);
  EXPECT_EQ(p.oidc_issuer("https://id.example/realms/demo/"), nullptr) << "an exact match";
  cvc::net::NetPolicy bad;
  EXPECT_FALSE(cvc::net::parse_net_policy("oidc http://id.example\n", bad, &why));
  EXPECT_NE(why.find("not https"), std::string::npos) << why;
  EXPECT_FALSE(cvc::net::parse_net_policy("oidc-client https://id.example\n", bad, &why));
  EXPECT_FALSE(cvc::net::parse_net_policy("oidc-client https://id.example a b\n", bad, &why));
}

#if !defined(__EMSCRIPTEN__)
TEST(NetOidc, TheLoopbackListenerTakesExactlyOneRedirect) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "no HTTP backend to play the browser";
  cvc::net::LoopbackRedirect l;
  std::string why;
  ASSERT_TRUE(l.listen(0, "/callback", &why)) << why;
  const std::string uri = l.redirect_uri();
  EXPECT_EQ(uri.rfind("http://127.0.0.1:", 0), 0u) << uri;
  EXPECT_EQ(uri.substr(uri.size() - 9), "/callback");
  const std::string base = uri.substr(0, uri.size() - 9);
  cvc::app app;
  const auto get = [&app](const std::string &url) {
    cvc::net::HttpRequest r;
    r.url = url;
    r.timeout_secs = 5;
    return cvc::net::send(app, r);
  };
  // Another path (a favicon) gets a 404 and is not the redirect.
  std::atomic<bool> favicon_done{false};
  std::thread favicon([&] {
    const cvc::net::HttpResponse r = get(base + "/favicon.ico");
    EXPECT_EQ(r.status, 404) << r.error;
    favicon_done = true;
  });
  std::optional<std::string> q;
  std::thread browser([&] {
    const cvc::net::HttpResponse r = get(uri + "?code=c1&state=s1");
    EXPECT_EQ(r.status, 200) << r.error;
    EXPECT_NE(r.body.find("close this tab"), std::string::npos) << r.body;
    EXPECT_NE(r.body.find("&lt;b&gt;"), std::string::npos) << "the message is escaped";
  });
  for (int i = 0; i < 2000 && (!q || !favicon_done); ++i) {
    if (std::optional<std::string> got = l.poll())
      q = got;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_TRUE(q);
  EXPECT_EQ(*q, "code=c1&state=s1");
  favicon.join();
  l.respond(true, "<b>signed in</b>");
  browser.join();
  EXPECT_FALSE(l.poll()) << "one redirect only";
  EXPECT_EQ(l.redirect_uri(), "") << "it stopped listening";
  EXPECT_FALSE(l.listen(0, "callback", &why));
}
#endif

TEST(NetOidc, OpenUrlTakesOnlyHttpUrlsAndHonoursNoBrowser) {
  cvc::app app;
  std::vector<std::string> opened;
  cvc::net::set_url_opener(app, [&](const std::string &u) {
    opened.push_back(u);
    return true;
  });
  std::string why;
  EXPECT_EQ(cvc::net::open_url(app, "https://id.example/a?b=c", &why),
            cvc::net::OpenUrlResult::opened);
  EXPECT_EQ(cvc::net::open_url(app, "javascript:alert(1)", &why), cvc::net::OpenUrlResult::failed);
  EXPECT_EQ(cvc::net::open_url(app, "file:///etc/passwd", &why), cvc::net::OpenUrlResult::failed);
  EXPECT_EQ(cvc::net::open_url(app, "https://id.example/a b", &why),
            cvc::net::OpenUrlResult::failed);
  EXPECT_EQ(opened.size(), 1u);
  cvc::net::set_browser_disabled(app, true);
  EXPECT_FALSE(cvc::net::browser_available(app));
  EXPECT_EQ(cvc::net::open_url(app, "https://id.example/", &why),
            cvc::net::OpenUrlResult::no_browser);
  cvc::net::set_browser_disabled(app, false);
  cvc::net::set_url_opener(app, nullptr);
  std::string noticed;
  cvc::net::set_url_notice(app, [&](const std::string &u) { noticed = u; });
  cvc::net::url_notice(app, "https://id.example/device");
  EXPECT_EQ(noticed, "https://id.example/device");
  cvc::net::set_url_notice(app, nullptr);
}

// Each app's opener, notice, page hooks and --no-browser are its own: set on one, the other never
// sees them, made in either order.
TEST(NetBrowser, TwoAppsKeepTheirOwnOpenerNoticeHooksAndNoBrowser) {
  for (int order = 0; order < 2; ++order) {
    cvc::app a, b;
    cvc::app &first = order == 0 ? a : b;
    cvc::app &second = order == 0 ? b : a;
    std::vector<std::string> first_opened, second_opened, first_noticed, second_noticed;
    cvc::net::set_url_opener(first, [&](const std::string &u) {
      first_opened.push_back(u);
      return true;
    });
    cvc::net::set_url_opener(second, [&](const std::string &u) {
      second_opened.push_back(u);
      return true;
    });
    cvc::net::set_url_notice(first, [&](const std::string &u) { first_noticed.push_back(u); });
    cvc::net::set_url_notice(second, [&](const std::string &u) { second_noticed.push_back(u); });
    auto hooks = std::make_shared<cvc::net::page::Hooks>();
    hooks->url = [] { return std::string("https://page.example/first"); };
    cvc::net::page::set_hooks(first, hooks);

    EXPECT_EQ(cvc::net::open_url(first, "https://one.example/"), cvc::net::OpenUrlResult::opened);
    EXPECT_EQ(cvc::net::open_url(second, "https://two.example/"), cvc::net::OpenUrlResult::opened);
    cvc::net::url_notice(first, "https://one.example/n");
    cvc::net::url_notice(second, "https://two.example/n");
    EXPECT_EQ(first_opened, std::vector<std::string>{"https://one.example/"});
    EXPECT_EQ(second_opened, std::vector<std::string>{"https://two.example/"});
    EXPECT_EQ(first_noticed, std::vector<std::string>{"https://one.example/n"});
    EXPECT_EQ(second_noticed, std::vector<std::string>{"https://two.example/n"});
#if !defined(__EMSCRIPTEN__)
    EXPECT_TRUE(cvc::net::page::present(first));
    EXPECT_FALSE(cvc::net::page::present(second));
    EXPECT_EQ(cvc::net::page::url(first), "https://page.example/first");
    EXPECT_EQ(cvc::net::page::url(second), "");
#endif

    cvc::net::set_browser_disabled(first, true); // --no-browser on one app only
    EXPECT_FALSE(cvc::net::browser_available(first));
    EXPECT_TRUE(cvc::net::browser_available(second));
    EXPECT_EQ(cvc::net::open_url(first, "https://one.example/again"),
              cvc::net::OpenUrlResult::no_browser);
    EXPECT_EQ(cvc::net::open_url(second, "https://two.example/again"),
              cvc::net::OpenUrlResult::opened);
    EXPECT_EQ(first_opened.size(), 1u);
    EXPECT_EQ(second_opened.size(), 2u);
  }
}

// Apps on threads at once: each one's opener hears only its own URLs.
TEST(NetBrowser, AppsOnThreadsAtOnceOpenWithTheirOwnOpeners) {
  constexpr int kApps = 4;
  std::atomic<int> wrong{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &wrong] {
      cvc::app app;
      const std::string mine = "https://app" + std::to_string(i) + ".example/";
      cvc::net::set_url_opener(app, [&](const std::string &u) {
        if (u.rfind(mine, 0) != 0)
          ++wrong;
        return true;
      });
      for (int k = 0; k < 50; ++k)
        if (cvc::net::open_url(app, mine + std::to_string(k)) != cvc::net::OpenUrlResult::opened)
          ++wrong;
    });
  for (std::thread &t : threads)
    t.join();
  EXPECT_EQ(wrong.load(), 0);
}

// A later app, made once another is gone, starts with no opener, no hooks and the browser on.
TEST(NetBrowser, LaterAppStartsClean) {
  {
    cvc::app app;
    cvc::net::set_url_opener(app, [](const std::string &) { return true; });
    cvc::net::page::set_hooks(app, std::make_shared<cvc::net::page::Hooks>());
    cvc::net::set_browser_disabled(app, true);
  }
  cvc::app later;
#if !defined(__EMSCRIPTEN__)
  EXPECT_FALSE(cvc::net::page::present(later));
#endif
  std::string noticed;
  cvc::net::set_url_notice(later, [&](const std::string &u) { noticed = u; });
  cvc::net::url_notice(later, "https://later.example/");
  EXPECT_EQ(noticed, "https://later.example/");
  // Given an opener of its own, it may open URLs: the earlier app's --no-browser did not carry
  // over.
  cvc::net::set_url_opener(later, [](const std::string &) { return true; });
  EXPECT_TRUE(cvc::net::browser_available(later));
}
