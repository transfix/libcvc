#ifndef CVC_ARIADNE_URI_HTTP_H
#define CVC_ARIADNE_URI_HTTP_H

// Ariadne — the built-in http:// / https:// URI handler (roadmap §13.6 read + §13.10 write).
// OPTIONAL: compiled against libcurl when libcvc is built with it (CVC_ARIADNE_HAVE_HTTP);
// otherwise register_http_uri_handler() is a no-op and the scheme stays unresolved unless a host
// registers its own. Like state://, it is opt-in: a host CALLS register_http_uri_handler() during
// setup (before load_*) — Ariadne never touches the network on its own. Reads are bounded (connect
// + total timeout, a response-size cap) and restricted to the http/https protocols (redirects too),
// so a fragment URL cannot be redirected into file:// or another local scheme.
//
// A GET (resolve) and a PUT/POST (store) both run through the SAME easy-handle setup, and both
// consult an optional host-supplied options provider for auth headers and the HTTP method — see
// HttpRequestOptions / set_http_options_provider below.

#include <functional>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// Per-request HTTP options a HOST supplies out-of-band. Deliberately NOT sourced from the .ari
// document: a document is on-disk / shared / sandboxed and must never carry a bearer token — it
// only names a URL. The host injects credentials here, keyed on the URL, so a token minted for host
// A is never sent to host B. Consulted by BOTH the GET reader (resolve) and the PUT/POST writer
// (store).
struct HttpRequestOptions {
  // Full header lines, sent verbatim — e.g. "Authorization: Bearer <token>",
  // "Content-Type: application/json". To SUPPRESS a header libcurl adds by default, give the name
  // with nothing after the colon ("Expect:"); to send one with an empty value, use "X:;" —
  // libcurl's own convention. Order is preserved. A store with no Content-Type here defaults to
  // application/octet-stream (so a server does not form-parse opaque bytes); pass your own to
  // override.
  //
  // REDIRECT SAFETY: when this list is non-empty (a credentialed request), the handler does NOT
  // follow HTTP redirects and reports a 3xx as an error — because libcurl re-sends custom headers
  // (an "X-Api-Key", say) verbatim to the redirect target and only strips Authorization/Cookie/
  // Proxy-Authorization on a cross-origin hop. That keeps a token minted for the URL's host from
  // being carried to another origin the (untrusted) .ari document could steer the redirect to.
  // Point the URL at the final resource; do not rely on a redirect to reach it under auth.
  std::vector<std::string> headers;
  // Override the HTTP method. Empty = the operation default: GET for a read (resolve), PUT for a
  // write (store). Set "POST" to POST a store body (an RPC-style endpoint), or "PATCH"/"DELETE"/…;
  // on a read, a non-empty non-"GET" value is sent as the request method (a bodyless custom verb).
  std::string method;
};

// Given the request URL and whether this call is a write (store) or a read (resolve), return the
// options (auth headers, method) to apply. Invoked on EVERY http(s) resolve/store while a provider
// is installed, just before the transfer. May be called concurrently from multiple threads, so it
// must be reentrant; it runs inside resolve()/store()'s try/catch barrier, so a throw is contained
// (turned into an error result) rather than propagated.
using HttpOptionsProvider =
    std::function<HttpRequestOptions(const std::string &url, bool for_write)>;

// Install (or, with a default-constructed / null std::function, clear) the process-global HTTP
// options provider — the way a host attaches an Authorization header per host. Thread-safe; the
// http handlers copy the provider out under a lock before each call, so setting/clearing it never
// races a transfer in flight. A no-op in a build without libcurl (the scheme can make no requests
// anyway).
void set_http_options_provider(HttpOptionsProvider provider);

// Register the built-in "http" and "https" URI handlers — BOTH read (GET, via register_uri_handler)
// and write (PUT/POST, via register_uri_store_handler). A no-op when this build has no libcurl.
// Process-global and thread-safe; register once, before load_*/store. Replaces any existing
// handlers for those schemes (a host that wants different behaviour registers its own AFTER this,
// or does not call this). Tear down with unregister_uri_handler / unregister_uri_store_handler on
// "http"/"https".
void register_http_uri_handler();

// Whether this build has the libcurl-backed http handler compiled in (CVC_ARIADNE_HAVE_HTTP).
bool have_http_uri_handler();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_HTTP_H
