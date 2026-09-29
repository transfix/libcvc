// Ariadne — the http:// / https:// URI handler (roadmap §13.6 read + §13.10 write). See uri_http.h.
//
// A THIN ADAPTER over cvc::net (inc/cvc/net/http_client.h). The transport — libcurl on native,
// Emscripten fetch on wasm — lives behind cvc::net, so this handler works identically on both
// targets. What stays HERE is the Ariadne-specific policy the transport must not bake in:
//   - the host auth-header/method provider (credentials come from the host, never the .ari doc),
//   - the response-size cap,
//   - the redirect-safety rules (a credentialed read does not follow; a write never follows),
//   - the store Content-Type default,
//   - mapping a cvc::net::HttpResponse (which reports a >= 400 as ok=true + status) to a
//     UriResult/StoreResult, re-imposing ">= 400 is an error" to preserve resolve()/store() output.
// It is opt-in: a host must call register_http_uri_handler(), which registers only when a real net
// backend is compiled — otherwise the scheme is left unresolved, exactly as before.

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

// A remote fragment/library is DSL text or a modest asset; a response past this is refused rather
// than buffered whole (an availability guard, mirroring the file handler's cap). Also bounds the
// (discarded) response body of a store.
constexpr std::size_t kMaxHttpBytes = 64u * 1024u * 1024u; // 64 MiB

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
HttpOptionsProvider current_provider() {
  std::lock_guard<std::mutex> lock(provider_mutex());
  return provider_cell();
}

// True if `lines` already carries a header whose name (the text before the first ':') equals `name`
// case-insensitively. Also matches libcurl's suppression form (`Name:` with an empty value), so an
// explicit suppression by the provider is respected rather than overridden by a default.
bool has_header(const std::vector<std::string> &lines, const std::string &name) {
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

UriResult http_fetch(const Uri &u, const std::string & /*base*/) {
  // Consult the provider FIRST. No provider -> default GET, no extra headers. A throwing provider
  // (host code) unwinds to resolve()'s barrier.
  HttpRequestOptions opts;
  if (const HttpOptionsProvider p = current_provider())
    opts = p(u.raw, /*for_write=*/false);

  cvc::net::HttpRequest req;
  req.url = u.raw;
  req.max_bytes = kMaxHttpBytes;
  req.headers = opts.headers;
  // libcurl re-sends custom headers (X-Api-Key, …) VERBATIM on every redirect hop and only strips
  // Authorization/Cookie/Proxy-Authorization on a cross-origin redirect. So follow a redirect only
  // when NO provider headers are attached (nothing to leak across an origin the untrusted .ari
  // document chose); a credentialed read stops at the redirect instead of carrying the token
  // onward.
  const bool follow = opts.headers.empty();
  req.follow_redirects = follow;
  // A method override on a read is a bodyless custom verb (default stays GET).
  if (!opts.method.empty() && opts.method != "GET")
    req.method = opts.method;

  const cvc::net::HttpResponse r = cvc::net::send(req);
  if (!r.ok)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' failed: " + r.error};
  // The facade does not fail a >= 400 (so a cache can see a 304); the resolver treats it as an
  // error.
  if (r.status >= 400)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' failed: HTTP status " + std::to_string(r.status)};
  // Not following (credentialed): a 3xx would otherwise be returned as OK with the redirect PAGE as
  // the body. Surface it instead of silently handing back the wrong bytes.
  if (!follow && r.status >= 300 && r.status < 400)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' returned redirect status " +
                std::to_string(r.status) + " to '" + r.canonical_url +
                "'; a credentialed read does not follow redirects (the auth header would cross an "
                "origin). Resolve the target directly."};
  return {true, r.body, r.canonical_url, std::string()};
}

// §13.10 the write analogue: PUT (default) or POST/… the `content` bytes to the URL. Method + auth
// headers come from the same provider as the reader (for_write = true).
StoreResult http_store(const Uri &u, const std::string &content, const std::string & /*base*/) {
  HttpRequestOptions opts;
  if (const HttpOptionsProvider p = current_provider())
    opts = p(u.raw, /*for_write=*/true);
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
  // Default a neutral Content-Type when the provider supplied none (its own wins) — otherwise a
  // server might form-parse opaque store bytes.
  std::vector<std::string> header_lines = opts.headers;
  if (!has_header(header_lines, "Content-Type"))
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
