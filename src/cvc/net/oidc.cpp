// cvc::net::oidc — the OAuth 2.0 / OpenID Connect protocol pieces. See inc/cvc/net/oidc.h.

#if defined(_WIN32) && !defined(_CRT_RAND_S)
#define _CRT_RAND_S // rand_s, before anything includes <stdlib.h>
#endif

#include "json_lite.h"

#include <algorithm>
#include <cvc/net/oidc.h>
#include <cvc/state/state_blob_store.h> // cvc::sha256_hex
#include <stdexcept>

#if defined(_WIN32)
#include <stdlib.h>
#elif defined(__APPLE__) || defined(__EMSCRIPTEN__)
#include <sys/random.h> // getentropy (Emscripten: crypto.getRandomValues)
#include <unistd.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace cvc {
namespace net {
namespace oidc {

namespace {

bool fail(std::string *error, std::string message) {
  if (error)
    *error = std::move(message);
  return false;
}

bool is_unreserved(unsigned char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
         c == '.' || c == '_' || c == '~';
}

std::string percent_encode(const std::string &s) {
  static const char *hex = "0123456789ABCDEF";
  std::string out;
  out.reserve(s.size());
  for (unsigned char c : s) {
    if (is_unreserved(c)) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('%');
      out.push_back(hex[c >> 4]);
      out.push_back(hex[c & 15]);
    }
  }
  return out;
}

int hex_digit(char c) {
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return -1;
}

std::string percent_decode(const std::string &s) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '+') {
      out.push_back(' ');
    } else if (s[i] == '%' && i + 2 < s.size() && hex_digit(s[i + 1]) >= 0 &&
               hex_digit(s[i + 2]) >= 0) {
      out.push_back(static_cast<char>(hex_digit(s[i + 1]) * 16 + hex_digit(s[i + 2])));
      i += 2;
    } else {
      out.push_back(s[i]);
    }
  }
  return out;
}

// Check one endpoint URL of `issuer`'s provider. "" or why not.
std::string check_endpoint(const char *name, const std::string &url, const Origin &issuer,
                           const std::vector<Origin> &extra) {
  std::string why;
  const std::optional<Origin> o = parse_origin(url, &why);
  if (!o)
    return std::string(name) + " " + why;
  if (url.find('#') != std::string::npos)
    return std::string(name) + " has a fragment";
  if (o->scheme != "https" &&
      !(issuer.scheme == "http" && is_loopback_host(issuer.host) && is_loopback_host(o->host)))
    return std::string(name) + " " + o->str() + " is not https";
  if (*o != issuer && std::find(extra.begin(), extra.end(), *o) == extra.end())
    return std::string(name) + " is at " + o->str() + ", not the issuer's origin " + issuer.str() +
           " (a host policy 'oidc ISSUER " + o->str() + "' line allows it)";
  return std::string();
}

void wipe_string(std::string &s) {
  std::fill(s.begin(), s.end(), '\0');
  s.clear();
}

bool fill_random(unsigned char *out, std::size_t n) {
#if defined(_WIN32)
  for (std::size_t i = 0; i < n;) {
    unsigned int r = 0;
    if (rand_s(&r) != 0)
      return false;
    for (int k = 0; k < 4 && i < n; ++k, ++i)
      out[i] = static_cast<unsigned char>(r >> (8 * k));
  }
  return true;
#elif defined(__APPLE__) || defined(__EMSCRIPTEN__)
  while (n > 0) {
    const std::size_t chunk = n < 256 ? n : 256; // getentropy's limit
    if (getentropy(out, chunk) != 0)
      return false;
    out += chunk;
    n -= chunk;
  }
  return true;
#else
  // /dev/urandom: every Linux and BSD, whatever its libc's age (getentropy is glibc 2.25+).
  const int fd = ::open("/dev/urandom", O_RDONLY | O_CLOEXEC);
  if (fd < 0)
    return false;
  while (n > 0) {
    const ssize_t got = ::read(fd, out, n);
    if (got <= 0) {
      ::close(fd);
      return false;
    }
    out += got;
    n -= static_cast<std::size_t>(got);
  }
  ::close(fd);
  return true;
#endif
}

} // namespace

bool is_loopback_host(const std::string &host) {
  if (host == "localhost" || host == "[::1]")
    return true;
  // 127.0.0.0/8, written as a dotted quad.
  int parts = 0, digits = 0, value = 0, first = -1;
  for (char c : host + ".") {
    if (c == '.') {
      if (digits == 0 || value > 255)
        return false;
      if (parts == 0)
        first = value;
      ++parts;
      digits = value = 0;
    } else if (c >= '0' && c <= '9') {
      value = value * 10 + (c - '0');
      if (++digits > 3)
        return false;
    } else {
      return false;
    }
  }
  return parts == 4 && first == 127;
}

std::string check_issuer(const std::string &issuer) {
  std::string why;
  const std::optional<Origin> o = parse_origin(issuer, &why);
  if (!o)
    return "the issuer " + why;
  if (issuer.find_first_of("?#") != std::string::npos)
    return "the issuer '" + issuer + "' has a query or a fragment";
  if (o->scheme != "https" && !is_loopback_host(o->host))
    return "the issuer '" + issuer + "' is not https (http is only for a loopback issuer)";
  return std::string();
}

std::string discovery_url(const std::string &issuer) {
  std::string base = issuer;
  while (!base.empty() && base.back() == '/' && base.find("://") + 3 < base.size())
    base.pop_back();
  return base + "/.well-known/openid-configuration";
}

std::optional<Provider> parse_discovery(const std::string &body, const std::string &issuer,
                                        const std::vector<Origin> &endpoint_origins,
                                        std::string *error) {
  const auto bad = [&](std::string why) {
    fail(error, "the discovery document of " + issuer + ": " + std::move(why));
    return std::nullopt;
  };
  const std::optional<Origin> io = parse_origin(issuer);
  if (!io)
    return bad("the issuer does not parse");
  std::string why;
  const std::optional<json_lite::Value> doc = json_lite::parse(body, &why);
  if (!doc || !doc->is_object())
    return bad(doc ? "it is not a JSON object" : "it is not JSON (" + why + ")");
  const std::optional<std::string> said = doc->string_at("issuer");
  if (!said)
    return bad("it names no issuer");
  if (*said != issuer)
    return bad("it is for issuer '" + printable(*said) +
               "', not this one (an issuer must match exactly; use '" + printable(*said) +
               "' if that is the provider's)");
  Provider p;
  p.issuer = issuer;
  struct Field {
    const char *name;
    std::string *into;
    bool required;
  } fields[] = {
      {"authorization_endpoint", &p.authorization_endpoint, false},
      {"token_endpoint", &p.token_endpoint, true},
      {"device_authorization_endpoint", &p.device_authorization_endpoint, false},
      {"revocation_endpoint", &p.revocation_endpoint, false},
      {"end_session_endpoint", &p.end_session_endpoint, false},
      {"userinfo_endpoint", &p.userinfo_endpoint, false},
  };
  for (const Field &f : fields) {
    const std::optional<std::string> url = doc->string_at(f.name);
    if (!url || url->empty()) {
      if (f.required)
        return bad(std::string("it has no ") + f.name);
      continue;
    }
    if (const std::string e = check_endpoint(f.name, *url, *io, endpoint_origins); !e.empty())
      return bad(e);
    *f.into = *url;
  }
  p.code_challenge_methods = doc->strings_at("code_challenge_methods_supported");
  if (!p.code_challenge_methods.empty() &&
      std::find(p.code_challenge_methods.begin(), p.code_challenge_methods.end(), "S256") ==
          p.code_challenge_methods.end())
    return bad("its code_challenge_methods_supported leaves out S256 (PKCE S256 is required)");
  if (const json_lite::Value *v = doc->get("authorization_response_iss_parameter_supported"))
    p.iss_parameter = v->kind == json_lite::Value::Kind::boolean && v->b;
  return p;
}

std::string random_token(std::size_t bytes) {
  std::string raw(bytes, '\0');
  if (!fill_random(reinterpret_cast<unsigned char *>(raw.data()), raw.size()))
    throw std::runtime_error("the system has no random bytes to give (getentropy failed)");
  std::string out = base64url_encode(raw);
  wipe_string(raw);
  return out;
}

std::string s256_challenge(const std::string &verifier) {
  const std::string hex =
      cvc::sha256_hex(reinterpret_cast<const unsigned char *>(verifier.data()), verifier.size());
  std::string digest;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2)
    digest.push_back(static_cast<char>(hex_digit(hex[i]) * 16 + hex_digit(hex[i + 1])));
  return base64url_encode(digest);
}

std::string base64url_encode(const std::string &bytes) {
  static const char *abc = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  std::size_t i = 0;
  for (; i + 2 < bytes.size(); i += 3) {
    const unsigned v = (static_cast<unsigned char>(bytes[i]) << 16) |
                       (static_cast<unsigned char>(bytes[i + 1]) << 8) |
                       static_cast<unsigned char>(bytes[i + 2]);
    out.push_back(abc[(v >> 18) & 63]);
    out.push_back(abc[(v >> 12) & 63]);
    out.push_back(abc[(v >> 6) & 63]);
    out.push_back(abc[v & 63]);
  }
  if (i + 1 == bytes.size()) {
    const unsigned v = static_cast<unsigned char>(bytes[i]) << 16;
    out.push_back(abc[(v >> 18) & 63]);
    out.push_back(abc[(v >> 12) & 63]);
  } else if (i + 2 == bytes.size()) {
    const unsigned v = (static_cast<unsigned char>(bytes[i]) << 16) |
                       (static_cast<unsigned char>(bytes[i + 1]) << 8);
    out.push_back(abc[(v >> 18) & 63]);
    out.push_back(abc[(v >> 12) & 63]);
    out.push_back(abc[(v >> 6) & 63]);
  }
  return out;
}

std::optional<std::string> base64url_decode(const std::string &text) {
  std::string out;
  unsigned acc = 0;
  int bits = 0;
  for (char c : text) {
    int v;
    if (c >= 'A' && c <= 'Z')
      v = c - 'A';
    else if (c >= 'a' && c <= 'z')
      v = c - 'a' + 26;
    else if (c >= '0' && c <= '9')
      v = c - '0' + 52;
    else if (c == '-' || c == '+')
      v = 62;
    else if (c == '_' || c == '/')
      v = 63;
    else if (c == '=')
      break;
    else
      return std::nullopt;
    acc = (acc << 6) | static_cast<unsigned>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<char>((acc >> bits) & 0xFF));
    }
  }
  return out;
}

std::string form_encode(const Params &fields) {
  std::string out;
  for (const auto &[k, v] : fields) {
    if (!out.empty())
      out.push_back('&');
    out += percent_encode(k) + "=" + percent_encode(v);
  }
  return out;
}

Params parse_query(const std::string &query) {
  Params out;
  std::size_t start = (!query.empty() && query[0] == '?') ? 1 : 0;
  while (start <= query.size()) {
    std::size_t end = query.find('&', start);
    if (end == std::string::npos)
      end = query.size();
    const std::string pair = query.substr(start, end - start);
    if (!pair.empty()) {
      const std::size_t eq = pair.find('=');
      out.emplace_back(percent_decode(pair.substr(0, eq)),
                       eq == std::string::npos ? std::string()
                                               : percent_decode(pair.substr(eq + 1)));
    }
    start = end + 1;
  }
  return out;
}

bool caller_param(const std::string &name) {
  static const char *kOwn[] = {
      "response_type", "client_id",      "redirect_uri",          "scope",         "state",
      "nonce",         "code_challenge", "code_challenge_method", "client_secret", "code",
      "code_verifier", "grant_type",     "device_code",           "refresh_token", "request",
      "request_uri",   "response_mode"};
  if (name.empty())
    return false;
  for (const char *own : kOwn)
    if (name == own)
      return false;
  return true;
}

std::string authorization_url(const Provider &p, const AuthorizationRequest &r) {
  Params q = {{"response_type", "code"},
              {"client_id", r.client_id},
              {"redirect_uri", r.redirect_uri},
              {"scope", r.scope},
              {"state", r.state}};
  if (!r.nonce.empty())
    q.emplace_back("nonce", r.nonce);
  q.emplace_back("code_challenge", r.code_challenge);
  q.emplace_back("code_challenge_method", "S256");
  for (const auto &kv : r.extra)
    if (caller_param(kv.first))
      q.push_back(kv);
  std::string url = p.authorization_endpoint;
  if (const std::size_t hash = url.find('#'); hash != std::string::npos)
    url.resize(hash);
  url.push_back(url.find('?') == std::string::npos ? '?' : '&');
  return url + form_encode(q);
}

AuthorizationResponse parse_authorization_response(const std::string &query) {
  AuthorizationResponse r;
  for (const auto &[k, v] : parse_query(query)) {
    if (k == "code")
      r.code = v;
    else if (k == "state")
      r.state = v;
    else if (k == "iss") {
      r.iss = v;
      r.has_iss = true;
    } else if (k == "error")
      r.error = v;
    else if (k == "error_description")
      r.error_description = v;
  }
  return r;
}

std::string check_authorization_response(const AuthorizationResponse &r, const Provider &p,
                                         const std::string &state) {
  if (state.empty() || r.state != state)
    return "the reply's state does not match the request's (a stale or forged redirect)";
  if (r.has_iss ? r.iss != p.issuer : p.iss_parameter)
    return "the reply's iss is not the issuer (RFC 9207: a mix-up between providers)";
  if (!r.error.empty())
    return "the provider said " + printable(r.error) +
           (r.error_description.empty() ? "" : ": " + printable(r.error_description));
  if (r.code.empty())
    return "the reply carries no code";
  return std::string();
}

void TokenReply::wipe() {
  wipe_string(access_token);
  wipe_string(refresh_token);
  wipe_string(id_token);
}

TokenReply parse_token_reply(long status, const std::string &body) {
  TokenReply t;
  std::string why;
  const std::optional<json_lite::Value> doc = json_lite::parse(body, &why);
  if (!doc || !doc->is_object()) {
    t.error = "invalid_reply";
    t.error_description =
        "the token endpoint answered " + std::to_string(status) + " with no JSON object";
    return t;
  }
  if (status < 200 || status >= 300 || doc->get("error")) {
    t.error = doc->string_at("error").value_or("invalid_reply");
    t.error = printable(t.error, 64);
    t.error_description = printable(doc->string_at("error_description").value_or(""));
    if (t.error == "invalid_reply")
      t.error_description = "the token endpoint answered " + std::to_string(status);
    return t;
  }
  t.access_token = doc->string_at("access_token").value_or("");
  t.token_type = doc->string_at("token_type").value_or("");
  t.refresh_token = doc->string_at("refresh_token").value_or("");
  t.id_token = doc->string_at("id_token").value_or("");
  t.scope = doc->string_at("scope").value_or("");
  t.expires_in = doc->int_at("expires_in");
  std::string type = t.token_type;
  std::transform(type.begin(), type.end(), type.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (t.access_token.empty()) {
    t.error = "invalid_reply";
    t.error_description = "the token endpoint's reply has no access_token";
  } else if (type != "bearer") {
    t.error = "invalid_reply";
    t.error_description =
        "the token endpoint gave a '" + printable(t.token_type, 32) + "' token, not a Bearer token";
  } else {
    t.ok = true;
  }
  if (!t.ok)
    t.wipe();
  return t;
}

std::optional<DeviceAuthorization> parse_device_authorization(long status, const std::string &body,
                                                              std::string *error) {
  std::string why;
  const std::optional<json_lite::Value> doc = json_lite::parse(body, &why);
  if (!doc || !doc->is_object()) {
    fail(error, "the device authorization endpoint answered " + std::to_string(status) +
                    " with no JSON object");
    return std::nullopt;
  }
  if (status < 200 || status >= 300 || doc->get("error")) {
    const std::string code = printable(doc->string_at("error").value_or("an error"), 64);
    const std::string desc = printable(doc->string_at("error_description").value_or(""));
    fail(error,
         "the device authorization endpoint said " + code + (desc.empty() ? "" : ": " + desc));
    return std::nullopt;
  }
  DeviceAuthorization d;
  d.device_code = doc->string_at("device_code").value_or("");
  d.user_code = doc->string_at("user_code").value_or("");
  d.verification_uri = doc->string_at("verification_uri").value_or("");
  if (d.verification_uri.empty()) // a draft of RFC 8628, and Google, wrote verification_url
    d.verification_uri = doc->string_at("verification_url").value_or("");
  d.verification_uri_complete = doc->string_at("verification_uri_complete").value_or("");
  d.expires_in = doc->int_at("expires_in").value_or(0);
  d.interval = std::max<std::int64_t>(1, doc->int_at("interval").value_or(5));
  const auto web = [](const std::string &url) {
    const std::optional<Origin> o = parse_origin(url);
    return o && (o->scheme == "https" || is_loopback_host(o->host));
  };
  if (d.device_code.empty() || d.user_code.empty() || d.expires_in <= 0) {
    fail(error, "the device authorization reply lacks a device_code, user_code or expires_in");
    return std::nullopt;
  }
  if (!web(d.verification_uri)) {
    fail(error, "the device authorization reply's verification_uri is not an https URL");
    return std::nullopt;
  }
  if (!d.verification_uri_complete.empty() && !web(d.verification_uri_complete))
    d.verification_uri_complete.clear();
  d.user_code = printable(d.user_code, 64);
  return d;
}

std::optional<IdTokenClaims> decode_id_token(const std::string &jwt, std::string *error) {
  const std::size_t a = jwt.find('.');
  const std::size_t b = a == std::string::npos ? a : jwt.find('.', a + 1);
  if (b == std::string::npos || jwt.find('.', b + 1) != std::string::npos) {
    fail(error, "the ID token is not a JWT (three dot-separated parts)");
    return std::nullopt;
  }
  const std::optional<std::string> header = base64url_decode(jwt.substr(0, a));
  const std::optional<std::string> payload = base64url_decode(jwt.substr(a + 1, b - a - 1));
  std::string why;
  const std::optional<json_lite::Value> h = header ? json_lite::parse(*header, &why) : std::nullopt;
  const std::optional<json_lite::Value> p =
      payload ? json_lite::parse(*payload, &why) : std::nullopt;
  if (!h || !h->is_object() || !p || !p->is_object()) {
    fail(error, "the ID token's header or claims are not JSON objects");
    return std::nullopt;
  }
  IdTokenClaims c;
  c.alg = h->string_at("alg").value_or("");
  if (c.alg.empty() || c.alg == "none") {
    fail(error,
         "the ID token is unsigned (alg " + (c.alg.empty() ? std::string("missing") : c.alg) + ")");
    return std::nullopt;
  }
  c.iss = p->string_at("iss").value_or("");
  c.sub = p->string_at("sub").value_or("");
  c.azp = p->string_at("azp").value_or("");
  c.aud = p->strings_at("aud");
  c.exp = p->int_at("exp").value_or(0);
  c.iat = p->int_at("iat").value_or(0);
  if (const std::optional<std::string> n = p->string_at("nonce")) {
    c.nonce = *n;
    c.has_nonce = true;
  }
  c.name = p->string_at("name").value_or("");
  c.email = p->string_at("email").value_or("");
  c.preferred_username = p->string_at("preferred_username").value_or("");
  return c;
}

std::string check_id_token(const IdTokenClaims &c, const IdTokenExpect &e) {
  if (c.iss != e.issuer)
    return "the ID token's iss is not the issuer";
  if (c.sub.empty())
    return "the ID token names no subject";
  if (std::find(c.aud.begin(), c.aud.end(), e.client_id) == c.aud.end())
    return "the ID token is not for this client (aud)";
  if ((c.aud.size() > 1 || !c.azp.empty()) && c.azp != e.client_id)
    return "the ID token's azp is not this client";
  if (c.exp <= 0 || c.exp + e.leeway < e.now)
    return "the ID token has expired";
  if (c.iat <= 0 || c.iat > e.now + e.leeway)
    return "the ID token's iat is missing or in the future";
  if (e.nonce && (!c.has_nonce || c.nonce != *e.nonce))
    return "the ID token's nonce is not the one sent (a replayed token)";
  if (!e.sub.empty() && c.sub != e.sub)
    return "the refreshed ID token names another subject";
  return std::string();
}

std::string printable(const std::string &s, std::size_t max) {
  std::string out;
  for (unsigned char c : s) {
    if (out.size() >= max)
      break;
    out.push_back(c >= 0x20 && c < 0x7F ? static_cast<char>(c) : '?');
  }
  return out;
}

} // namespace oidc
} // namespace net
} // namespace cvc
