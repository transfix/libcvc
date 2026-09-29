// Ariadne — the http:// / https:// URI handler (roadmap §13.6 read + §13.10 write). See uri_http.h.
//
// A THIN ADAPTER over cvc::net (inc/cvc/net/http_client.h). The transport — libcurl on native,
// Emscripten fetch on wasm — lives behind cvc::net, so this handler works identically on both
// targets. The Ariadne-specific policy the transport must not bake in — the host auth-header/method
// provider, the response-size cap, the redirect-safety rules, the store Content-Type default, and
// the HttpResponse -> UriResult/StoreResult mapping — lives in the `detail` helpers here
// (uri_http_detail.h) so the §13.9 caching handler (uri_http_cache.cpp) reuses exactly the same
// policy. The handler is opt-in: a host must call register_http_uri_handler(), which registers only
// when a real net backend is compiled — otherwise the scheme is left unresolved, as before.

#include "uri_http_detail.h"

#include <cctype>
#include <cstddef>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/net/http_client.h>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// A store body of unknown media type gets a neutral Content-Type so a strict server does not
// form-parse or 415 it (a host that knows better passes its own via the provider).
constexpr char kDefaultStoreContentType[] = "Content-Type: application/octet-stream";

// The host-supplied options provider (auth headers + method), guarded by its own mutex. Copied out
// under the lock before each call so set/clear never races a transfer in flight (uri_http.h).
std::mutex &provider_mutex() {
  static std::mutex m;
  return m;
}
HttpOptionsProvider &provider_cell() {
  static HttpOptionsProvider p;
  return p;
}

} // namespace

namespace detail {

HttpRequestOptions consult_http_provider(const std::string &url, bool for_write) {
  HttpOptionsProvider p;
  {
    std::lock_guard<std::mutex> lock(provider_mutex());
    p = provider_cell();
  }
  if (p)
    return p(url, for_write);
  return HttpRequestOptions{};
}

bool http_header_present(const std::vector<std::string> &lines, const std::string &name) {
  for (const std::string &line : lines) {
    if (line.find(':') != name.size())
      continue; // name-length must match exactly (":" right after the name)
    bool eq = true;
    for (std::size_t i = 0; i < name.size(); ++i)
      if (std::tolower(static_cast<unsigned char>(line[i])) !=
          std::tolower(static_cast<unsigned char>(name[i]))) {
        eq = false;
        break;
      }
    if (eq)
      return true;
  }
  return false;
}

UriResult map_http_fetch_result(const std::string &raw_url, bool follow,
                                const cvc::net::HttpResponse &r) {
  if (!r.ok)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + raw_url + "' failed: " + r.error};
  // A 304 reaching the plain mapper is unexpected (only the cache sends conditional requests, and
  // it serves the cached body on its own path); a bodyless 304 must never surface as an empty-body
  // OK.
  if (r.status == 304)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + raw_url +
                "' returned 304 Not Modified with no cached entry to revalidate"};
  // The facade does not fail a >= 400 (so a cache can see a 304); the resolver treats it as an
  // error.
  if (r.status >= 400)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + raw_url + "' failed: HTTP status " + std::to_string(r.status)};
  // Not following (credentialed): a 3xx would otherwise be returned as OK with the redirect PAGE as
  // the body. Surface it instead of silently handing back the wrong bytes.
  if (!follow && r.status >= 300 && r.status < 400)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + raw_url + "' returned redirect status " +
                std::to_string(r.status) + " to '" + r.canonical_url +
                "'; a credentialed read does not follow redirects (the auth header would cross an "
                "origin). Resolve the target directly."};
  return {true, r.body, r.canonical_url, std::string()};
}

StoreResult http_store_uncached(const Uri &u, const std::string &content,
                                const std::string & /*base*/) {
  const HttpRequestOptions opts = consult_http_provider(u.raw, /*for_write=*/true);
  const std::string method = opts.method.empty() ? std::string("PUT") : opts.method;

  cvc::net::HttpRequest req;
  req.url = u.raw;
  req.method = method;
  req.body = content;
  req.max_bytes = kMaxHttpBytes;
  // NEVER follow a redirect on a write: 301/302/303 silently drop the body and 307/308 re-send it —
  // to a host the untrusted .ari document chose. A 3xx on a store is an error the host must
  // resolve.
  req.follow_redirects = false;
  // Default a neutral Content-Type when the provider supplied none (its own wins).
  std::vector<std::string> header_lines = opts.headers;
  if (!http_header_present(header_lines, "Content-Type"))
    header_lines.push_back(kDefaultStoreContentType);
  req.headers = std::move(header_lines);

  const cvc::net::HttpResponse r = cvc::net::send(req);
  if (!r.ok)
    return {false, std::string(), "ari: http " + method + " to '" + u.raw + "' failed: " + r.error};
  if (r.status >= 400)
    return {false, std::string(),
            "ari: http " + method + " to '" + u.raw + "' failed: HTTP status " +
                std::to_string(r.status)};
  if (r.status >= 300 && r.status < 400)
    return {false, std::string(),
            "ari: http " + method + " to '" + u.raw + "' returned redirect status " +
                std::to_string(r.status) + " to '" + r.canonical_url +
                "'; redirects are not followed on a write (the body would be dropped or re-sent to "
                "another origin). Store to the resolved location directly."};
  return {true, r.canonical_url, std::string()};
}

} // namespace detail

namespace {

UriResult http_fetch(const Uri &u, const std::string & /*base*/) {
  const HttpRequestOptions opts = detail::consult_http_provider(u.raw, /*for_write=*/false);
  cvc::net::HttpRequest req;
  req.url = u.raw;
  req.max_bytes = detail::kMaxHttpBytes;
  req.headers = opts.headers;
  // Follow a redirect only when uncredentialed (nothing to leak across an origin the untrusted .ari
  // document chose); a credentialed read stops at the redirect instead of carrying the token
  // onward.
  const bool follow = opts.headers.empty();
  req.follow_redirects = follow;
  // A method override on a read is a bodyless custom verb (default stays GET).
  if (!opts.method.empty() && opts.method != "GET")
    req.method = opts.method;
  return detail::map_http_fetch_result(u.raw, follow, cvc::net::send(req));
}

StoreResult http_store(const Uri &u, const std::string &content, const std::string &base) {
  return detail::http_store_uncached(u, content, base);
}

} // namespace

void set_http_options_provider(HttpOptionsProvider provider) {
  std::lock_guard<std::mutex> lock(provider_mutex());
  provider_cell() = std::move(provider);
}

void register_http_uri_handler() {
  if (!cvc::net::have_http_backend())
    return; // no transport compiled — leave the scheme unresolved (as the old libcurl-less stub
            // did)
  register_uri_handler("http", http_fetch);
  register_uri_handler("https", http_fetch);
  register_uri_store_handler("http", http_store);
  register_uri_store_handler("https", http_store);
}

bool have_http_uri_handler() { return cvc::net::have_http_backend(); }

} // namespace ariadne
} // namespace cvc
