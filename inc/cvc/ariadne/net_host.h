#ifndef CVC_ARIADNE_NET_HOST_H
#define CVC_ARIADNE_NET_HOST_H

// Ariadne — a host turning the network on for its documents (docs/ARIADNE.md §10.3).
//
// A document reaches the network only when its host says so. ari and arigl do it from their
// command line (host_cli.h): --net, --net-allow ORIGIN, --net-policy FILE and --credential-store
// SPEC. Without any of them a document has no http verbs and no http(s) URIs. With them,
// enable_net:
//   - builds the allowlist: the --net-allow origins, the policy files' allow lines and, in a
//     browser, the page's own origin. Every request, and every redirect it follows, must go to one;
//   - builds the host's credential store (cvc/net/credentials.h): in memory, or in the
//     --credential-store file natively, its names bound as the policy files' credential lines say,
//     holding for this run the tokens --credential-env reads from the environment;
//   - registers the http verbs (net_intrinsics.h) and the http(s) URI handler (uri_http.h) with
//     both, so (http-get ...), (http-request ...), (fetch "https://...") and an import: of an
//     https URL are all checked;
//   - makes the capability tokens net, net.auth and net.auth.persist hold (requires.h), so a
//     document's requires: fails clearly where its launcher did not turn the network on;
//   - turns on sign-in with OAuth 2.0 / OpenID Connect providers (oidc_host.h): the oidc-* verbs
//     and open-url, the --oidc-issuer and --oidc-client pins, and the tokens oidc, oidc.loopback,
//     oidc.device, oidc.redirect and browser.
//
// The registrations are process-wide, like the action-intrinsic providers they add to.

#include <cvc/ariadne/oidc_host.h> // OidcHostConfig
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

// What the host's flags asked for.
struct NetHostConfig {
  bool enabled = false;                  // --net, or implied by any of the others
  std::vector<std::string> allow;        // --net-allow ORIGIN, in order
  std::vector<std::string> policy_files; // --net-policy FILE, in order
  std::string credential_store;          // --credential-store SPEC ("" = in memory)
  // --credential-env NAME=VAR, in order: credential NAME is the token in environment variable VAR
  // (a policy credential line binds NAME); held for the run, never written.
  std::vector<std::pair<std::string, std::string>> credential_env;
  // --oidc-issuer, --oidc-client, --oidc-redirect-uri, --no-browser (oidc_host.h).
  OidcHostConfig oidc;
};

// Turn the network on for documents run in `app`, as `config` says. Call once at setup, before
// load(); a config that is not enabled does nothing. Returns "" or what is wrong (an origin that
// does not parse, an unreadable policy or credential file, a credential bound to an origin not
// allowed, a credential store or --credential-env in a browser, a --credential-env name no policy
// line binds or whose variable holds no bearer token); nothing is turned on then. A
// --credential-env variable that is unset or empty supplies nothing and is no error, so a launcher
// can always pass the flag. `app` must outlive the network being on: call disable_net() before it
// dies.
std::string enable_net(cvc::app &app, const NetHostConfig &config);

// Turn it off again: programs built afterwards get no http verbs, a call to one bound earlier
// raises an error, the http(s) URI handlers are unregistered and the capability tokens stop
// holding. Credentials kept in a file stay there.
void disable_net();

// What enable_net turned on, for (has? "net"), `ari info` and the like.
struct NetHostStatus {
  bool enabled = false;             // the network is on (the token net)
  bool auth = false;                // a credential store is there (net.auth)
  bool persist = false;             // it keeps credentials in a file (net.auth.persist)
  std::vector<std::string> allowed; // the allowlist, as origins
  std::string credential_store;     // the file, or ""
  std::vector<std::string> env_credentials; // names --credential-env supplied a token for
  bool oidc = false;                        // sign-in with an OpenID provider (oidc_host.h)
  OidcFlows oidc_flows;                     // ... and its flows here
  std::vector<std::string> oidc_issuers;    // the issuers sign-in is pinned to (empty: any allowed)
  bool browser = false;                     // open-url can open a browser here
};
NetHostStatus net_host_status();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_NET_HOST_H
