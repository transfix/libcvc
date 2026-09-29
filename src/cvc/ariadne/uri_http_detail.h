#ifndef CVC_ARIADNE_URI_HTTP_DETAIL_H
#define CVC_ARIADNE_URI_HTTP_DETAIL_H

// Ariadne — internal shared pieces of the http(s):// handler, used by BOTH the plain handler
// (uri_http.cpp) and the §13.9 caching handler (uri_http_cache.cpp) so the request-building,
// response-mapping, and store policy live in ONE place. NOT a shipped header (lives under src/, not
// inc/): it is included only by those two sibling TUs. The public API stays in uri_http.h.

#include <cstddef>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/net/http_client.h>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {
namespace detail {

// A remote fragment/library is DSL text or a modest asset; a response past this is refused rather
// than buffered whole (an availability guard). Shared by the read + store + cache paths.
constexpr std::size_t kMaxHttpBytes = 64u * 1024u * 1024u; // 64 MiB

// Consult the host-installed HttpOptionsProvider (auth headers + method override) for `url`.
// Returns a default-constructed options (no headers, empty method) when no provider is installed.
// Thread-safe: the provider is copied under a lock before the call (set/clear never races).
HttpRequestOptions consult_http_provider(const std::string &url, bool for_write);

// Case-insensitive test for a header whose name (text before the first ':') equals `name`. Also
// matches libcurl's suppression form (`Name:` with an empty value).
bool http_header_present(const std::vector<std::string> &lines, const std::string &name);

// Map a cvc::net READ response to a UriResult with Ariadne's policy: a transport failure, an HTTP
// status >= 400, and (when not following) a 3xx are all errors; otherwise the body + effective URL.
// `follow` is whether redirects were followed (governs the 3xx-is-an-error rule) and is used only
// for the error text. Shared so the plain fetch and the cache's miss/replace paths map identically.
UriResult map_http_fetch_result(const std::string &raw_url, bool follow,
                                const cvc::net::HttpResponse &r);

// The UNCACHED store (roadmap §13.10): PUT (default) / POST/… the bytes to the URL via cvc::net,
// consulting the provider for method + auth headers, defaulting a neutral Content-Type, never
// following a redirect. This is the plain http_store body; the cache's write wrapper delegates here
// and then invalidates the entry.
StoreResult http_store_uncached(const Uri &u, const std::string &content, const std::string &base);

} // namespace detail
} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_URI_HTTP_DETAIL_H
