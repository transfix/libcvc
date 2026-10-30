#ifndef CVC_ARIADNE_OIDC_HOST_H
#define CVC_ARIADNE_OIDC_HOST_H

// Ariadne — OAuth 2.0 / OpenID Connect sign-in for documents (docs/ARIADNE.md §10.4).
//
// A document signs in with any standards-compliant provider (Keycloak, Authentik, Google, ...)
// from the provider's issuer URL and a public client's id, and then names the credential on its
// requests with auth:, as it does any other (net_host.h). The host runs the whole flow and keeps
// every token: the access token, a refresh token and the ID token live in the host's credential
// store under names the document chose, never in a program variable, the state tree, a log line
// or an error message. What the document sees is the flow's progress and the signed-in user's
// non-secret claims, written to the state tree under a path it chose and handed back by
// (oidc-status NAME).
//
// enable_net (net_host.h) turns it on with the network; the verbs need nothing else:
//
//   (oidc-sign-in OPTS)       start (or resume) a sign-in; returns at once
//   (oidc-cancel NAME)        stop one in progress
//   (oidc-sign-out NAME [OPTS]) forget the tokens, revoking them (and ending the provider's
//                             session) where the provider supports it
//   (oidc-status NAME)        the sign-in's progress and claims, as a dict
//   (open-url URL)            open an http(s) URL the host's policy allows in the user's browser
//
// The host picks the flow (OPTS "flow" "auto", or loopback, redirect or device):
//   - loopback (RFC 8252): a native host with a browser listens on 127.0.0.1, opens the
//     authorization endpoint in the system browser and takes the one redirect back;
//   - redirect: a browser build navigates the page to the authorization endpoint and completes the
//     sign-in when the provider sends it back (the PKCE verifier, state and nonce wait in
//     sessionStorage meanwhile);
//   - device (RFC 8628): a terminal with no browser (over SSH) shows a code and a URL to visit on
//     any other device, and polls the token endpoint.
// The authorization code flows use PKCE S256, a random state and a nonce. Tokens are refreshed
// before they expire and after a 401, when the provider issued a refresh token.
//
// Runtime::drain advances the flows (pump_oidc), so a sign-in progresses with the host's frames
// and never blocks one: its requests run on the app's background pool (inline in a browser build
// without threads, as the http verbs do).

#include <atomic>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace net {
class NetPolicy;
class CredentialStore;
} // namespace net
namespace ariadne {

// What the host's flags say about sign-in (the policy's oidc lines say the rest).
struct OidcHostConfig {
  // --oidc-issuer ISSUER: documents may sign in with these issuers only (repeatable).
  std::vector<std::string> issuers;
  // --oidc-client ISSUER=CLIENT_ID: the client the launcher registered with ISSUER (repeatable).
  std::vector<std::pair<std::string, std::string>> clients;
  // --oidc-redirect-uri URI: the redirect URI a browser build's page registers ("" = the page's
  // own URL, without the reply's parameters).
  std::string redirect_uri;
  // --no-browser: never open a browser (open-url and the loopback flow show the URL; auto picks
  // the device flow).
  bool no_browser = false;
};

// Which flows this host can run, for the capability tokens oidc.loopback, oidc.device and
// oidc.redirect (requires.h).
struct OidcFlows {
  bool loopback = false; // a native host (it can listen on 127.0.0.1)
  bool device = false;   // any host
  bool redirect = false; // a browser build
};
OidcFlows oidc_flows();

// Turn sign-in on for documents run in `app`, with the host's policy and credential store
// (enable_net calls it). Registers the verbs above as action intrinsics.
void enable_oidc(cvc::app &app, std::shared_ptr<const cvc::net::NetPolicy> policy,
                 std::shared_ptr<cvc::net::CredentialStore> store, const OidcHostConfig &config,
                 std::shared_ptr<const std::atomic<bool>> live);
// Turn it off: every flow in progress stops (a loopback listener closes), and the store forgets
// the host's renewers. Tokens kept in a file stay there.
void disable_oidc();

// Advance the sign-ins in progress: a request's reply, a redirect that arrived, a poll that is due,
// a token about to expire. Runtime::drain calls it every frame, on the host's thread.
void pump_oidc();
// Whether a sign-in has a request in flight (AppRuntime::idle: a snapshot waits for it). A flow
// waiting on the user (a browser, a device code) is not busy.
bool oidc_busy();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_OIDC_HOST_H
