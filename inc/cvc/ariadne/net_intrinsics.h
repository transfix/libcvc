#ifndef CVC_ARIADNE_NET_INTRINSICS_H
#define CVC_ARIADNE_NET_INTRINSICS_H

// Ariadne — HTTP verbs for the state_exec program lanes (roadmap §13.8; docs/ARIADNE.md §10.2).
//
//   (http-get URL [OPTS])            a GET
//   (http-request METHOD URL [OPTS]) any method
//   (fetch URI [BASE])               any registered scheme (file/state/cvc/http)
//
// Each is an async function: the call parks the program while the request runs on the app's
// background task pool and resumes it with the reply; its "-async" form returns a future for
// (await f). In a browser build without threads (cvc::net::http_starts_async) the request is
// started instead, a browser fetch, and the fetch's callback posts the reply, so the frame loop
// goes on meanwhile (state_exec's async_work, docs/STATE_EXEC_EXCEPTIONS.md 3.12). The http verbs'
// reply is a dict with every key present:
//
//   ok            the transport worked (a reply came back), whatever its status
//   status        the HTTP status (0 when no reply came)
//   body          the body, as bytes
//   url           the URL the reply came from
//   headers       the reply's header lines ("Name: value"), in order
//   header_map    the same as a dict: lowercased name -> value (repeats joined with ", ")
//   content_type  the Content-Type header ("" when there is none)
//   captured      whether capture_auth: kept a credential from this reply
//   error         why the request failed, or why capture_auth: kept nothing ("" otherwise)
//
// OPTS is a dict: query (dict, appended to the URL), headers (dict, or a list of "Name: value"
// lines), body (string or bytes) or form (dict, sent x-www-form-urlencoded). A host with a
// credential store (NetAccess::credentials) also takes:
//
//   auth          "NAME": the host adds "Authorization: Bearer <token>" for credential NAME as the
//                 request is sent, if NAME is bound to the URL's origin. The program never holds
//                 the token, and any copy of it the reply echoes is blanked.
//   capture_auth  (dict "name" NAME ["field" "token"] ["optional" #f]), or a list of them: from a
//                 2xx JSON reply, the host keeps the string at each FIELD (object keys joined by
//                 '.') as credential NAME, captured from the URL's origin, and the reply's body has
//                 "[redacted]" in each one's place. A missing optional field is no error (a
//                 refresh reply need not rotate the refresh token).
//   a credential value, (dict "credential" NAME), as a form or headers dict value: the host puts
//                 credential NAME's token there as the request is sent (percent-encoded in a
//                 form), on the same terms as auth, and blanks echoes of it. A query refuses one:
//                 a URL is logged. So an RFC 6749 section 6 refresh,
//                 (http-request "POST" url (dict "form" (dict "grant_type" "refresh_token"
//                   "refresh_token" (dict "credential" "svc.refresh")) "capture_auth" (list ...))),
//                 never shows the program either token.
//
// A request with any of them does not follow redirects. With a credential store, programs also get
// (net-auth-has? NAME [URL]), (net-auth-forget NAME) and (net-auth-clear); every program gets
// (net-allowed? URL).
//
// Use the verbs in a program lane (on:, on_tick:, on_key:, on_pointer:): not in init:, where
// nothing resumes a parked call, nor in the reactive read lane, which forbids parking. The http
// verbs are absent without a compiled cvc::net backend; fetch is always there.

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace cvc {
class app;
namespace net {
class NetPolicy;       // cvc/net/net_policy.h
class CredentialStore; // cvc/net/credentials.h
} // namespace net
namespace ariadne {

// What the verbs may reach and which credentials they may use.
struct NetAccess {
  // The origins requests may go to, redirects included (cvc::net::send(req, policy)). Null checks
  // nothing: the embedding program vouches for its documents.
  std::shared_ptr<const cvc::net::NetPolicy> policy;
  // The host's credentials. Null: a request with auth: or capture_auth: fails, and the net-auth
  // verbs are not there.
  std::shared_ptr<cvc::net::CredentialStore> credentials;
  // Turns the verbs off when cleared (disable_net): programs built afterwards do not get them,
  // and a call to one bound earlier raises an error. Null: always on.
  std::shared_ptr<const std::atomic<bool>> live;
};

// Register the verbs against `app` (its background task pool and exec scheduler). Appends to the
// process-global action-intrinsic providers, so call it once at setup, before load_*; tear down
// with cvc::ariadne::clear_action_intrinsics() (or clear `access.live`) before `app` dies, since
// the provider holds `app` by reference. A host that hands documents the network uses
// enable_net (net_host.h), which builds the NetAccess from its flags.
void register_net_intrinsics(cvc::app &app, NetAccess access);
// register_net_intrinsics(app, {}): unchecked, and no credentials.
void register_net_intrinsics(cvc::app &app);

// A GET the host makes for a document, not a program: an image: widget's picture (media.h). It
// keeps the verbs' terms: `access.policy` checks the URL, and `auth` ("" = none) names a
// credential the host puts in as the Authorization header, only for an origin the credential is
// bound to, never following a redirect, with any echo of the token blanked. Blocking; thread-safe.
// `ok` only for a 2xx reply; otherwise `error` says why ("http 404", the policy's refusal, "no
// credential ..."), and never quotes a token.
struct HostGet {
  bool ok = false;
  long status = 0;
  std::string body;
  std::string content_type;
  std::string error;
};
HostGet host_get(const std::string &url, const std::string &auth, const NetAccess &access);

// host_get without blocking where the transport starts requests (cvc::net::http_starts_async: a
// browser build without threads): `done` gets the result later, from the browser's event loop on
// this thread. Elsewhere it is host_get, and `done` is called before this returns.
void host_get_start(const std::string &url, const std::string &auth, const NetAccess &access,
                    std::function<void(HostGet)> done);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_NET_INTRINSICS_H
