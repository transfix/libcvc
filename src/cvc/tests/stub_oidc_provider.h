// A stub OpenID Provider for tests, on LocalHttpServer: discovery, an authorization endpoint that
// approves at once (a 302 back to the redirect URI), a token endpoint (authorization code with PKCE
// S256, refresh with rotation, the device grant), device authorization, revocation, an
// end-session endpoint and a protected /api/me. Its issuer is its own origin
// (http://127.0.0.1:PORT). It records every token it issues, so a test can check that none reached
// the document's state or a host's output. ID tokens carry a placeholder signature: the client
// takes them only from the token endpoint (OIDC Core 3.1.3.7 (6)).
#ifndef CVC_TESTS_STUB_OIDC_PROVIDER_H
#define CVC_TESTS_STUB_OIDC_PROVIDER_H

#include "local_http_server.h"

#include <atomic>
#include <ctime>
#include <cvc/net/http_client.h>
#include <cvc/net/oidc.h>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace cvc_test {

struct StubOidcOptions {
  bool device = true;                 // advertise a device_authorization_endpoint
  bool revocation = true;             // advertise a revocation_endpoint
  bool rotate_refresh = true;         // a refresh issues a new refresh token
  int expires_in = 3600;              // the access token's lifetime
  bool tamper_state = false;          // the redirect carries another state (a forged reply)
  bool tamper_nonce = false;          // the ID token carries another nonce (a replay)
  bool wrong_issuer = false;          // the discovery document names another issuer
  std::string foreign_token_endpoint; // a token_endpoint at another origin ("" = its own)
  int device_pending_polls = 0; // polls answered authorization_pending before an approval counts
  bool slow_down_once = false;  // the first pending poll says slow_down
};

class StubOidcProvider {
public:
  using Options = StubOidcOptions;

  explicit StubOidcProvider(Options o = {})
      : opt_(std::move(o)),
        server_([this](const LocalHttpServer::Request &r) { return serve(r); }) {}

  std::string issuer() const { return server_.origin(); }
  std::string origin() const { return server_.origin(); }
  std::string url(const std::string &path) const { return server_.url(path); }
  std::vector<LocalHttpServer::Request> requests() const { return server_.requests(); }
  const std::string &client_id() const { return client_id_; }

  // Every token it has issued (access, refresh, ID), the authorization codes and device codes.
  std::vector<std::string> secrets() const {
    std::lock_guard<std::mutex> lock(m_);
    return secrets_;
  }
  // Approve (or deny) the device code shown as `user_code`.
  void approve(const std::string &user_code, bool allow = true) {
    std::lock_guard<std::mutex> lock(m_);
    for (auto &[code, d] : devices_)
      if (d.user_code == user_code)
        d.decision = allow ? 1 : -1;
  }
  // The user code of the last device authorization, "" when none.
  std::string last_user_code() const {
    std::lock_guard<std::mutex> lock(m_);
    return last_user_code_;
  }
  // Make every refresh token issued so far invalid (the provider ended the session).
  void end_sessions() {
    std::lock_guard<std::mutex> lock(m_);
    refresh_.clear();
  }
  // Make every access token issued so far invalid (they expired early): /api/me answers 401.
  void expire_access_tokens() {
    std::lock_guard<std::mutex> lock(m_);
    access_.clear();
  }
  int count(const std::string &path_prefix) const {
    int n = 0;
    for (const auto &r : server_.requests())
      if (r.target.rfind(path_prefix, 0) == 0)
        ++n;
    return n;
  }
  std::vector<std::string> revoked() const {
    std::lock_guard<std::mutex> lock(m_);
    return revoked_;
  }

private:
  struct Grant {
    std::string client_id, redirect_uri, challenge, nonce, scope;
  };
  struct Device {
    std::string user_code, scope;
    int decision = 0; // 0 pending, 1 approved, -1 denied
    int polls = 0;
  };
  using Reply = LocalHttpServer::Reply;

  static Reply json(int status, const std::string &body) {
    return {status, {"Content-Type: application/json", "Cache-Control: no-store"}, body};
  }
  static Reply oauth_error(const std::string &code, int status = 400) {
    return json(status, "{\"error\": \"" + code + "\"}");
  }
  static std::string param(const cvc::net::oidc::Params &ps, const std::string &k) {
    for (const auto &[n, v] : ps)
      if (n == k)
        return v;
    return std::string();
  }

  std::string mint(const char *prefix) { // m_ held
    std::string t = std::string(prefix) + cvc::net::oidc::random_token(24);
    secrets_.push_back(t);
    return t;
  }

  std::string id_token(const std::string &nonce, const std::string &client) { // m_ held
    namespace o = cvc::net::oidc;
    const long now = static_cast<long>(std::time(nullptr));
    std::string claims = "{\"iss\":\"" + issuer() + "\",\"sub\":\"user-42\",\"aud\":\"" + client +
                         "\",\"exp\":" + std::to_string(now + 600) +
                         ",\"iat\":" + std::to_string(now) +
                         ",\"name\":\"Ada Lovelace\",\"email\":\"ada@example.test\","
                         "\"preferred_username\":\"ada\"";
    if (!nonce.empty())
      claims += ",\"nonce\":\"" + (opt_.tamper_nonce ? std::string("not-the-nonce") : nonce) + "\"";
    claims += "}";
    const std::string jwt = o::base64url_encode("{\"alg\":\"RS256\",\"kid\":\"stub\"}") + "." +
                            o::base64url_encode(claims) + "." +
                            o::base64url_encode("placeholder-signature-" + o::random_token(8));
    secrets_.push_back(jwt);
    return jwt;
  }

  Reply tokens(const std::string &client, const std::string &nonce, bool with_id) { // m_ held
    const std::string access = mint("oidcAT_");
    access_[access] = true;
    std::string body = "{\"access_token\":\"" + access +
                       "\",\"token_type\":\"Bearer\","
                       "\"expires_in\":" +
                       std::to_string(opt_.expires_in);
    const std::string refresh = mint("oidcRT_");
    refresh_[refresh] = client;
    body += ",\"refresh_token\":\"" + refresh + "\"";
    if (with_id)
      body += ",\"id_token\":\"" + id_token(nonce, client) + "\"";
    return json(200, body + "}");
  }

  Reply serve(const LocalHttpServer::Request &r) {
    namespace o = cvc::net::oidc;
    const std::size_t q = r.target.find('?');
    const std::string path = r.target.substr(0, q);
    const o::Params query = o::parse_query(q == std::string::npos ? "" : r.target.substr(q + 1));
    const o::Params form = o::parse_query(r.body);
    std::lock_guard<std::mutex> lock(m_);
    if (path == "/.well-known/openid-configuration") {
      const std::string iss = opt_.wrong_issuer ? issuer() + "/elsewhere" : issuer();
      std::string doc =
          "{\"issuer\":\"" + iss + "\",\"authorization_endpoint\":\"" + url("/authorize") +
          "\",\"token_endpoint\":\"" +
          (opt_.foreign_token_endpoint.empty() ? url("/token") : opt_.foreign_token_endpoint) +
          "\",\"end_session_endpoint\":\"" + url("/logout") + "\",\"jwks_uri\":\"" + url("/jwks") +
          "\",\"code_challenge_methods_supported\":[\"S256\"],"
          "\"response_types_supported\":[\"code\"]";
      if (opt_.device)
        doc += ",\"device_authorization_endpoint\":\"" + url("/device") + "\"";
      if (opt_.revocation)
        doc += ",\"revocation_endpoint\":\"" + url("/revoke") + "\"";
      return json(200, doc + "}");
    }
    if (path == "/authorize") {
      const std::string redirect = param(query, "redirect_uri");
      if (param(query, "response_type") != "code" || param(query, "client_id") != client_id_ ||
          param(query, "code_challenge_method") != "S256" ||
          param(query, "code_challenge").empty() || param(query, "state").empty() ||
          redirect.empty())
        return {400, {}, "bad authorization request"};
      const std::string code = mint("oidcAC_");
      grants_[code] = Grant{client_id_, redirect, param(query, "code_challenge"),
                            param(query, "nonce"), param(query, "scope")};
      const std::string state = opt_.tamper_state ? "forged-state" : param(query, "state");
      const std::string location =
          redirect + (redirect.find('?') == std::string::npos ? "?" : "&") +
          o::form_encode({{"code", code}, {"state", state}, {"iss", issuer()}});
      return {302, {"Location: " + location}, ""};
    }
    if (path == "/token" && r.method == "POST") {
      const std::string grant = param(form, "grant_type");
      if (grant == "authorization_code") {
        const auto it = grants_.find(param(form, "code"));
        if (it == grants_.end())
          return oauth_error("invalid_grant");
        const Grant g = it->second;
        grants_.erase(it); // one use
        if (param(form, "client_id") != g.client_id ||
            param(form, "redirect_uri") != g.redirect_uri ||
            o::s256_challenge(param(form, "code_verifier")) != g.challenge)
          return oauth_error("invalid_grant");
        return tokens(g.client_id, g.nonce,
                      (" " + g.scope + " ").find(" openid ") != std::string::npos);
      }
      if (grant == "refresh_token") {
        const auto it = refresh_.find(param(form, "refresh_token"));
        if (it == refresh_.end() || it->second != param(form, "client_id"))
          return oauth_error("invalid_grant");
        const std::string client = it->second;
        if (!opt_.rotate_refresh) {
          const std::string access = mint("oidcAT_");
          access_[access] = true;
          return json(200, "{\"access_token\":\"" + access +
                               "\",\"token_type\":\"Bearer\","
                               "\"expires_in\":" +
                               std::to_string(opt_.expires_in) + "}");
        }
        refresh_.erase(it);
        return tokens(client, std::string(), true);
      }
      if (grant == "urn:ietf:params:oauth:grant-type:device_code") {
        const auto it = devices_.find(param(form, "device_code"));
        if (it == devices_.end())
          return oauth_error("invalid_grant");
        Device &d = it->second;
        ++d.polls;
        if (d.decision < 0)
          return oauth_error("access_denied");
        if (d.decision == 0 || d.polls <= opt_.device_pending_polls) {
          if (opt_.slow_down_once && d.polls == 1)
            return oauth_error("slow_down");
          return oauth_error("authorization_pending");
        }
        const std::string scope = d.scope;
        devices_.erase(it);
        return tokens(client_id_, std::string(),
                      (" " + scope + " ").find(" openid ") != std::string::npos);
      }
      return oauth_error("unsupported_grant_type");
    }
    if (path == "/device" && r.method == "POST" && opt_.device) {
      if (param(form, "client_id") != client_id_)
        return oauth_error("invalid_client", 401);
      const std::string device_code = mint("oidcDC_");
      const std::string user_code = "WDJB-" + std::to_string(1000 + devices_.size());
      devices_[device_code] = Device{user_code, param(form, "scope")};
      last_user_code_ = user_code;
      return json(200, "{\"device_code\":\"" + device_code + "\",\"user_code\":\"" + user_code +
                           "\",\"verification_uri\":\"" + url("/activate") +
                           "\",\"verification_uri_complete\":\"" +
                           url("/activate?user_code=" + user_code) +
                           "\",\"expires_in\":600,\"interval\":1}");
    }
    if (path == "/revoke" && r.method == "POST") {
      const std::string token = param(form, "token");
      revoked_.push_back(token);
      refresh_.erase(token);
      access_.erase(token);
      return {200, {}, ""};
    }
    if (path == "/logout")
      return {200, {"Content-Type: text/html"}, "<p>signed out</p>"};
    if (path == "/api/me") {
      const std::string auth = r.header("Authorization");
      const std::string token = auth.rfind("Bearer ", 0) == 0 ? auth.substr(7) : std::string();
      if (access_.count(token))
        return json(200, "{\"sub\":\"user-42\",\"plan\":\"pro\"}");
      return {401, {"WWW-Authenticate: Bearer error=\"invalid_token\""}, "{}"};
    }
    return {404, {}, ""};
  }

  Options opt_;
  std::string client_id_ = "ariadne-test-client";
  mutable std::mutex m_;
  std::vector<std::string> secrets_;
  std::map<std::string, Grant> grants_;
  std::map<std::string, std::string> refresh_; // refresh token -> client
  std::map<std::string, bool> access_;
  std::map<std::string, Device> devices_;
  std::vector<std::string> revoked_;
  std::string last_user_code_;
  LocalHttpServer server_; // last: it stops (joins) before the maps above go
};

// A stand-in for the user's browser: GET the authorization URL from the provider without following
// its redirect, then GET where it points (the loopback listener), on a thread of its own, so the
// host's frame loop keeps polling the listener meanwhile. What the listener answered is kept.
class FakeBrowser {
public:
  ~FakeBrowser() {
    for (std::thread &t : threads_)
      if (t.joinable())
        t.join();
  }
  bool open(const std::string &url) {
    std::lock_guard<std::mutex> lock(m_);
    opened_.push_back(url);
    threads_.emplace_back([this, url] {
      cvc::net::HttpRequest a;
      a.url = url;
      a.follow_redirects = false;
      const cvc::net::HttpResponse r = cvc::net::send(a);
      const std::string location = cvc::net::response_header(r.headers, "Location");
      if (location.empty())
        return;
      {
        std::lock_guard<std::mutex> l(m_);
        locations_.push_back(location);
      }
      if (!follow_)
        return;
      cvc::net::HttpRequest b;
      b.url = location;
      b.follow_redirects = false;
      const cvc::net::HttpResponse page = cvc::net::send(b);
      std::lock_guard<std::mutex> l(m_);
      pages_.push_back(std::to_string(page.status) + " " + page.body);
    });
    return true;
  }
  void follow(bool f) { follow_ = f; }
  std::vector<std::string> opened() const {
    std::lock_guard<std::mutex> lock(m_);
    return opened_;
  }
  std::vector<std::string> locations() const {
    std::lock_guard<std::mutex> lock(m_);
    return locations_;
  }
  std::vector<std::string> pages() const {
    std::lock_guard<std::mutex> lock(m_);
    return pages_;
  }

private:
  mutable std::mutex m_;
  std::vector<std::thread> threads_;
  std::vector<std::string> opened_, locations_, pages_;
  std::atomic<bool> follow_{true};
};

} // namespace cvc_test

#endif // CVC_TESTS_STUB_OIDC_PROVIDER_H
