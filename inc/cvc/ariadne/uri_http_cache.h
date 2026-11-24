#ifndef CVC_ARIADNE_URI_HTTP_CACHE_H
#define CVC_ARIADNE_URI_HTTP_CACHE_H

// Ariadne — an app-wide caching http(s):// URI handler (roadmap §13.9). It replaces the plain
// http(s) READ handler with one backed by the cvc::app's state tree: repeated import:/load:/source:
// of the same URL is served from a cache under `sys.net.http_cache.entries.<key>` instead of
// re-fetching. Freshness (Cache-Control max-age) serves with NO network; a stale entry does a
// conditional GET (If-None-Match / If-Modified-Since) — a 304 revalidates cheaply, a 200 replaces.
// Retention (>= freshness) rides the state tree's own node-expiry (expireAt + sweepExpired, swept
// on access), the total size is held to a budget by dropping the least recently used entries, and
// concurrent requests for the same URL coalesce (single-flight).
//
// It builds on the plain handler's transport (cvc::net) and policy (uri_http.cpp): the auth-header
// provider, the response cap, the NetPolicy check, and the redirect-safety + >=400 mapping are
// shared. A CREDENTIALED request (the host provider returns headers, or the URL carries userinfo)
// BYPASSES the cache entirely — the cache is the app's, and must not serve one principal's
// authorized body to another; full Vary/`private` handling is later work. The WRITE side stays the
// plain PUT/POST store, but a successful store INVALIDATES the cached entry for that URL.
//
// Like state://, the cached handler uses the app's state tree; it is registered on that app (its
// handlers are the app's, and go with it). enable_net (net_host.h) registers it in place of the
// plain handler, so a host that turns the network on has it. A no-op (leaves the scheme
// unregistered) when no cvc::net backend is compiled.
//
// The state it keeps, all under `sys.net.http_cache` (app-root keys: a widget binds them with a
// leading '/', components/http_cache.ari is the settings window):
//   policy.enabled          1 (default): reads use the cache; 0: every read goes to the network and
//                           nothing is stored (what is there stays, for when it is turned on again)
//   policy.max_bytes        the total budget, bytes (256 MiB; 48 MiB in a browser). Past it, the
//                           least recently used entries are dropped
//   policy.max_entry_bytes  the largest body kept (64 MiB, which is also the response cap); a
//                           larger one is served but not kept
//   policy.max_mib          policy.max_bytes in MiB, and policy.max_entry_mib
//                           policy.max_entry_bytes in MiB, for a slider; writing either form
//                           updates the other
//   policy.freshness_secs   how long a response with no Cache-Control max-age and no validator is
//                           fresh (60)
//   policy.retention_secs   how long an entry is kept at least, past its freshness (3600)
//   stats.entries, .bytes   what the cache holds now
//   stats.hits              reads served with no network (fresh, or a coalesced follower)
//   stats.misses            reads whose body came from the network (or failed)
//   stats.revalidated       stale reads a 304 answered, served from the cache
//   stats.evicted           entries dropped for the budget or their retention
//   stats.fill, stats.used  bytes over policy.max_bytes (0..1), and the same as text
//   clear                   write any non-empty value: the cache empties, and the key is reset
//                           to ""
// Registration seeds every policy key that has no value, so a host may set its own before it. A
// change to a policy key takes effect at once: the budget is enforced as it is lowered, and every
// read reads the policy. Retention applies to entries stored after it changes.

#include <cstdint>
#include <cvc/ariadne/uri.h>
#include <cvc/net/http_client.h>
#include <memory>
#include <optional>
#include <string>

namespace cvc {
class app;
namespace net {
class NetPolicy; // cvc/net/net_policy.h
} // namespace net
namespace ariadne {

// Register the caching http/https READ handler (and its non-blocking form, register_uri_async_
// handler, for a browser build without threads) and an invalidating PUT/POST WRITE handler against
// `app`'s root state (cvc::state::instance(app)). Replaces any existing http/https handlers. Every
// request they make is checked against `policy`, as register_http_uri_handler(app, policy)'s are,
// and a cached body is served only for a URL `policy` allows (an entry kept under an earlier
// policy is not served past a stricter one). Null `policy`: unchecked. No-op if no HTTP backend is
// compiled (cvc::net::have_http_backend() == false).
void register_cached_http_uri_handler(cvc::app &app,
                                      std::shared_ptr<const cvc::net::NetPolicy> policy);
// register_cached_http_uri_handler(app, nullptr): unchecked.
void register_cached_http_uri_handler(cvc::app &app);

// Tear down `app`'s cached http/https read + write handlers (leaves the schemes unregistered; a
// host that wants the plain handler back calls register_http_uri_handler(app)). The cached data
// under sys.net.http_cache stays in the state tree. disable_net calls this.
void unregister_cached_http_uri_handler(cvc::app &app);

// Whether `app` has the cache: register_cached_http_uri_handler was called and not undone since.
bool has_http_cache(cvc::app &app);

// A host-side GET of `url` (an image: widget's picture, a native video's download) through `app`'s
// cache, with no credentials, checked against the policy the cache was registered with: what
// cvc::net::send would reply, a fresh or revalidated entry as a 200 with its Content-Type.
// Blocking; thread-safe. A policy-refused URL is answered without a request; a URL with userinfo,
// or the cache turned off (policy.enabled 0), goes to the network uncached. nullopt when `app` has
// no cache (the caller makes its request another way).
std::optional<cvc::net::HttpResponse>
http_cache_get(cvc::app &app, const std::string &url,
               const UriCancelled &cancelled = UriCancelled());

// http_cache_get without blocking where `app`'s transport starts requests (cvc::net::start): `done`
// gets the reply later, from the browser's event loop on this thread, or before this returns (a
// fresh entry, a refusal, a transport that sends in place). false, and `done` is not called, when
// `app` has no cache.
bool http_cache_start(cvc::app &app, const std::string &url, cvc::net::HttpDone done);

// Empty `app`'s cache now (what writing sys.net.http_cache.clear does; AppRuntime's bare action
// http_cache.clear, components/http_cache.ari's Clear button, calls it). The counters keep
// counting. Works whether or not the handler is registered.
void clear_http_cache(cvc::app &app);

// What `app`'s cache holds and has done (sys.net.http_cache.stats.* in the state tree).
struct HttpCacheStats {
  std::uint64_t entries = 0;
  std::uint64_t bytes = 0;
  std::uint64_t hits = 0;
  std::uint64_t misses = 0;
  std::uint64_t revalidated = 0;
  std::uint64_t evicted = 0;
};
HttpCacheStats http_cache_stats(cvc::app &app);

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_HTTP_CACHE_H
