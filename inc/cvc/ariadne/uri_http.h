#ifndef CVC_ARIADNE_URI_HTTP_H
#define CVC_ARIADNE_URI_HTTP_H

// Ariadne — the built-in http:// / https:// URI handler (roadmap §13.6). OPTIONAL: compiled
// against libcurl when libcvc is built with it (CVC_ARIADNE_HAVE_HTTP); otherwise
// register_http_uri_handler() is a no-op and the scheme stays unresolved unless a host registers
// its own. Like state://, it is opt-in: a host CALLS register_http_uri_handler() during setup
// (before load_*) — Ariadne never fetches the network on its own. The fetch is bounded (connect +
// total timeout, a response-size cap) and restricted to the http/https protocols (redirects too),
// so a fragment URL cannot be redirected into file:// or another local scheme.

namespace cvc {
namespace ariadne {

// Register the built-in "http" and "https" URI handlers. A no-op when this build has no libcurl.
// Process-global and thread-safe; register once, before load_*. Replaces any existing handler for
// those schemes (a host that wants different behaviour registers its own via register_uri_handler
// AFTER this, or simply does not call this).
void register_http_uri_handler();

// Whether this build has the libcurl-backed http handler compiled in (CVC_ARIADNE_HAVE_HTTP).
bool have_http_uri_handler();

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_HTTP_H
