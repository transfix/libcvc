#ifndef CVC_NET_OIDC_H
#define CVC_NET_OIDC_H

// cvc::net::oidc — the protocol pieces of an OAuth 2.0 / OpenID Connect public client
// (docs/ARIADNE.md §10.5). Nothing here sends a request or keeps a token: these build requests and
// read replies, and the host (cvc/ariadne/oidc_host.h) drives the flows with them, sending through
// cvc::net::send(req, policy) and keeping tokens in a CredentialStore.
//
// Generic by design: any provider that publishes an OpenID Provider discovery document
// (/.well-known/openid-configuration) and accepts public clients (no client secret, or the
// non-confidential one some providers give installed apps). The client:
//   - reads the discovery document and refuses it unless its issuer is the one asked for, its
//     endpoints are https (http only for a loopback issuer, as tests and local providers use) and
//     at the issuer's origin or an origin the host's policy lists for that issuer, and it does not
//     rule out PKCE S256;
//   - uses the authorization code grant with PKCE S256 (RFC 7636), a random state and a nonce,
//     and the device authorization grant (RFC 8628) where there is no browser;
//   - checks an ID token's iss, aud (and azp), exp, iat and nonce (OIDC Core 3.1.3.7). Its
//     signature is not checked: the client takes ID tokens only from the token endpoint, over
//     TLS, which 3.1.3.7 (6) allows in place of the signature.

#include <cstddef>
#include <cstdint>
#include <cvc/net/net_policy.h>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace net {
namespace oidc {

using Params = std::vector<std::pair<std::string, std::string>>;

// --- issuers and discovery -----------------------------------------------------------------------

// A loopback host: 127.0.0.0/8, [::1] or localhost.
bool is_loopback_host(const std::string &host);

// Whether `issuer` can be an issuer: an https URL with no user info, query or fragment, or an
// http one on a loopback host. "" when it can, else why not.
std::string check_issuer(const std::string &issuer);

// The discovery document's URL (OIDC Discovery 4): the issuer without a trailing '/', then
// "/.well-known/openid-configuration".
std::string discovery_url(const std::string &issuer);

// What the provider's discovery document says, once checked.
struct Provider {
  std::string issuer;
  std::string authorization_endpoint; // "" when the provider has none (a device-only one)
  std::string token_endpoint;
  std::string device_authorization_endpoint;       // "" when it does not do RFC 8628
  std::string revocation_endpoint;                 // RFC 7009, or ""
  std::string end_session_endpoint;                // RP-initiated logout, or ""
  std::string userinfo_endpoint;                   // or ""
  std::vector<std::string> code_challenge_methods; // as advertised (often absent)
  bool iss_parameter = false; // authorization_response_iss_parameter_supported (RFC 9207)
};

// Read a discovery document `body` for `issuer`. `endpoint_origins` are the origins besides the
// issuer's that its endpoints may be at (the host's policy). Fails closed: nullopt, with `error`
// saying why, unless the body is a JSON object whose issuer is exactly `issuer`, which has a
// token_endpoint, whose endpoints are https (http only when the issuer is a loopback http URL)
// and each at the issuer's origin or one of `endpoint_origins`, and which, if it lists
// code_challenge_methods_supported, lists S256.
std::optional<Provider> parse_discovery(const std::string &body, const std::string &issuer,
                                        const std::vector<Origin> &endpoint_origins,
                                        std::string *error = nullptr);

// --- the authorization request -------------------------------------------------------------------

// `bytes` bytes from the system's CSPRNG as base64url text, no padding (32 bytes: 43 characters).
// Throws std::runtime_error if the system has none to give.
std::string random_token(std::size_t bytes = 32);
// PKCE S256 (RFC 7636 4.2): BASE64URL(SHA-256(verifier)).
std::string s256_challenge(const std::string &verifier);

std::string base64url_encode(const std::string &bytes);
std::optional<std::string> base64url_decode(const std::string &text);
// application/x-www-form-urlencoded, in order.
std::string form_encode(const Params &fields);
// A query string's name=value pairs, percent-decoded ('+' as a space), in order.
Params parse_query(const std::string &query);

// The parameters of an authorization request (OIDC Core 3.1.2.1, RFC 7636).
struct AuthorizationRequest {
  std::string client_id;
  std::string redirect_uri;
  std::string scope; // space-separated
  std::string state;
  std::string nonce; // "" sends none (a plain OAuth 2.0 request)
  std::string code_challenge;
  Params extra; // the caller's own (prompt, login_hint, ...); none may be one of the above
};

// Whether a caller may set parameter `name` itself (not one the client sets: response_type,
// client_id, redirect_uri, scope, state, nonce, code_challenge, code_challenge_method,
// client_secret, code, code_verifier, grant_type, device_code, refresh_token, request,
// request_uri).
bool caller_param(const std::string &name);

// The authorization endpoint's URL carrying `r` in its query (after any query it already has).
std::string authorization_url(const Provider &p, const AuthorizationRequest &r);

// What the provider sent back to the redirect URI.
struct AuthorizationResponse {
  std::string code, state, iss, error, error_description;
  bool has_iss = false;
};
AuthorizationResponse parse_authorization_response(const std::string &query);
// Check a response against the request: its state is `state`, it carries no error, it has a code,
// and its iss (RFC 9207), when present or when `p` says it always is, is p.issuer. "" or why not.
std::string check_authorization_response(const AuthorizationResponse &r, const Provider &p,
                                         const std::string &state);

// --- the token endpoint and the device grant ----------------------------------------------------

// A token endpoint reply: RFC 6749 5.1 (ok) or 5.2 (error), and the device grant's errors
// (RFC 8628 3.5: authorization_pending, slow_down, access_denied, expired_token).
struct TokenReply {
  bool ok = false; // a 2xx JSON reply with a Bearer access token
  std::string access_token, token_type, refresh_token, id_token, scope;
  std::optional<std::int64_t> expires_in;
  std::string error;             // the provider's error code, or "invalid_reply"
  std::string error_description; // shown to the user (printable() of the provider's text)
  void wipe();                   // overwrite the tokens
};
TokenReply parse_token_reply(long status, const std::string &body);

// A device authorization reply (RFC 8628 3.2).
struct DeviceAuthorization {
  std::string device_code, user_code, verification_uri, verification_uri_complete;
  std::int64_t expires_in = 0;
  std::int64_t interval = 5; // seconds between polls (5 when the reply gives none)
};
// nullopt, with `error` saying why, unless the reply is a 2xx JSON object with a device_code, a
// user_code, an https (or loopback http) verification_uri and an expires_in.
std::optional<DeviceAuthorization> parse_device_authorization(long status, const std::string &body,
                                                              std::string *error = nullptr);

// --- ID tokens ----------------------------------------------------------------------------------

struct IdTokenClaims {
  std::string iss, sub, azp, nonce, name, email, preferred_username, alg;
  std::vector<std::string> aud;
  std::int64_t exp = 0, iat = 0;
  bool has_nonce = false;
};
// Decode a JWT's header and claims (JWS compact form). nullopt, with `error`, when it is not one,
// or its alg is "none".
std::optional<IdTokenClaims> decode_id_token(const std::string &jwt, std::string *error = nullptr);

struct IdTokenExpect {
  std::string issuer;
  std::string client_id;
  std::optional<std::string> nonce; // the nonce the request sent; nullopt: none was sent
  std::string sub;                  // a refresh's token must name the same subject ("" = any)
  std::int64_t now = 0;             // seconds since the epoch
  std::int64_t leeway = 120;        // clock skew allowed, seconds
};
// OIDC Core 3.1.3.7 (2-5, 9, 10, 11) and 12.2: iss is the issuer, aud holds the client (and azp,
// present when aud holds others, is the client), exp is not past, iat is not in the future, and
// nonce is the one sent. "" or why not.
std::string check_id_token(const IdTokenClaims &c, const IdTokenExpect &e);

// A provider's text made safe to show: printable ASCII (anything else as '?'), at most `max` bytes.
std::string printable(const std::string &s, std::size_t max = 200);

} // namespace oidc
} // namespace net
} // namespace cvc

#endif // CVC_NET_OIDC_H
