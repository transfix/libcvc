#ifndef CVC_ARIADNE_URI_HTTP_CACHE_H
#define CVC_ARIADNE_URI_HTTP_CACHE_H

// Ariadne — an app-wide caching http(s):// URI handler (roadmap §13.9). It replaces the plain
// http(s) READ handler with one backed by the cvc::app's state tree: repeated import:/load:/source:
// of the same URL is served from a cache under `sys.net.http_cache.entries.<key>` instead of
// re-fetching. Freshness (Cache-Control max-age / Expires) serves with NO network; a stale entry
// does a conditional GET (If-None-Match / If-Modified-Since) — a 304 revalidates cheaply, a 200
// replaces. Retention (>= freshness) rides the state tree's own node-expiry (expireAt +
// sweepExpired, swept on access), and concurrent requests for the same URL coalesce
// (single-flight).
//
// It builds on the plain handler's transport (cvc::net) and policy (uri_http.cpp): the auth-header
// provider, the response cap, and the redirect-safety + >=400 mapping are shared. A CREDENTIALED
// request (the host provider returns headers) BYPASSES the cache entirely — a process-global cache
// must not serve one principal's authorized body to another; full Vary/`private` handling is later
// work. The WRITE side stays the plain PUT/POST store, but a successful store INVALIDATES the
// cached entry for that URL.
//
// Like state://, the cached handler uses the app's state tree; it is registered on that app (its
// handlers are the app's, and go with it). A host calls it during setup (before load_*). A no-op
// (leaves the scheme unregistered) when no cvc::net backend is compiled.

namespace cvc {
class app;
namespace ariadne {

// Register the caching http/https READ handler and an invalidating PUT/POST WRITE handler against
// `app`'s root state (cvc::state::instance(app)). Replaces any existing http/https handlers. No-op
// if no HTTP backend is compiled (cvc::net::have_http_backend() == false).
void register_cached_http_uri_handler(cvc::app &app);

// Tear down `app`'s cached http/https read + write handlers (leaves the schemes unregistered; a
// host that wants the plain handler back calls register_http_uri_handler(app)). The cached data
// under sys.net.http_cache stays in the state tree.
void unregister_cached_http_uri_handler(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_HTTP_CACHE_H
