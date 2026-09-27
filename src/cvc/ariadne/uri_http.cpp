// Ariadne — the http:// / https:// URI handler (roadmap §13.6). See uri_http.h. Compiled against
// libcurl only when CVC_ARIADNE_HAVE_HTTP is defined (the CMake `find_package(CURL)` path);
// otherwise the handler is an inert stub. Lives in its own TU so the pure resolver (uri.cpp) and
// the rest of Ariadne carry no networking dependency — http is the opt-in add-on that pulls curl.

#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <string>

#ifdef CVC_ARIADNE_HAVE_HTTP
#include <cstddef>
#include <curl/curl.h>
#include <mutex>
#endif

namespace cvc {
namespace ariadne {

#ifdef CVC_ARIADNE_HAVE_HTTP

namespace {

// A remote fragment/library is DSL text or a modest asset; a response past this is refused rather
// than buffered whole (an availability guard, mirroring the file handler's cap).
constexpr std::size_t kMaxHttpBytes = 64u * 1024u * 1024u; // 64 MiB

// libcurl's global init is not thread-safe on its first call; do it exactly once before any easy
// handle. (curl_global_cleanup is intentionally not called — process-lifetime, like other libcvc
// process-global singletons.)
void ensure_curl_global() {
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

// Accumulate the response body, refusing to grow past the cap (returning < the offered size makes
// libcurl abort the transfer with CURLE_WRITE_ERROR).
std::size_t write_cb(char *ptr, std::size_t size, std::size_t nmemb, void *userdata) {
  auto *out = static_cast<std::string *>(userdata);
  const std::size_t n = size * nmemb;
  if (out->size() + n > kMaxHttpBytes)
    return 0;
  out->append(ptr, n);
  return n;
}

UriResult http_fetch(const Uri &u, const std::string & /*base*/) {
  ensure_curl_global();
  CURL *h = curl_easy_init();
  if (!h)
    return {false, std::string(), std::string(), "ari: http: could not init libcurl"};
  std::string body;
  curl_easy_setopt(h, CURLOPT_URL, u.raw.c_str());
  curl_easy_setopt(h, CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(h, CURLOPT_WRITEDATA, &body);
  curl_easy_setopt(h, CURLOPT_FOLLOWLOCATION, 1L);
  curl_easy_setopt(h, CURLOPT_MAXREDIRS, 5L);
  curl_easy_setopt(h, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(h, CURLOPT_TIMEOUT, 30L);
  curl_easy_setopt(h, CURLOPT_FAILONERROR, 1L); // an HTTP >= 400 status is a fetch failure
  curl_easy_setopt(h, CURLOPT_NOSIGNAL,
                   1L); // thread-safe: no SIGALRM-based timeouts. NOTE: with
                        // NOSIGNAL the DNS-phase timeout needs an async
                        // resolver — the curl recipe must build with the
                        // threaded resolver or c-ares (CURL_VERSION_ASYNCHDNS),
                        // else a stalled DNS lookup ignores CONNECTTIMEOUT.
  curl_easy_setopt(h, CURLOPT_USERAGENT, "cvc-ariadne/1");
  // Restrict to http/https for BOTH the request and any redirect target, so a URL cannot be
  // redirected into file:// / gopher:// / etc. (an SSRF/local-file exfil guard).
  // CURLOPT_PROTOCOLS(_STR) are enum values, not macros — key the choice on the libcurl version
  // (the _STR forms arrived in 7.85.0; the LONG bitmask forms are deprecated there).
#if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(h, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(h, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
  curl_easy_setopt(h, CURLOPT_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
  curl_easy_setopt(h, CURLOPT_REDIR_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
#endif
  const CURLcode rc = curl_easy_perform(h);
  char *eff = nullptr;
  curl_easy_getinfo(h, CURLINFO_EFFECTIVE_URL, &eff); // the post-redirect URL = the canonical id
  const std::string canonical = eff ? std::string(eff) : u.raw;
  curl_easy_cleanup(h);
  if (rc != CURLE_OK)
    return {false, std::string(), std::string(),
            std::string("ari: http fetch of '") + u.raw + "' failed: " + curl_easy_strerror(rc)};
  return {true, std::move(body), canonical, std::string()};
}

} // namespace

void register_http_uri_handler() {
  register_uri_handler("http", http_fetch);
  register_uri_handler("https", http_fetch);
}

bool have_http_uri_handler() { return true; }

#else // no libcurl

void register_http_uri_handler() { /* built without libcurl — the scheme stays unresolved */ }

bool have_http_uri_handler() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
