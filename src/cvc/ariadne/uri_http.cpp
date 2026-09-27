// Ariadne — the http:// / https:// URI handler (roadmap §13.6 read + §13.10 write). See uri_http.h.
// Compiled against libcurl only when CVC_ARIADNE_HAVE_HTTP is defined (the CMake
// `find_package(CURL)` path); otherwise the handler is an inert stub. Lives in its own TU so the
// pure resolver (uri.cpp) and the rest of Ariadne carry no networking dependency — http is the
// opt-in add-on that pulls curl.

#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <string>

#ifdef CVC_ARIADNE_HAVE_HTTP
#include <cctype>
#include <cstddef>
#include <curl/curl.h>
#include <memory>
#include <mutex>
#include <vector>
#endif

namespace cvc {
namespace ariadne {

#ifdef CVC_ARIADNE_HAVE_HTTP

namespace {

// A remote fragment/library is DSL text or a modest asset; a response past this is refused rather
// than buffered whole (an availability guard, mirroring the file handler's cap). Also bounds the
// (discarded) response body of a store, so a chatty endpoint cannot OOM the writer.
constexpr std::size_t kMaxHttpBytes = 64u * 1024u * 1024u; // 64 MiB

// A store body of unknown media type: send a neutral Content-Type so a strict server does not
// form-parse or 415 it. (A COPYPOSTFIELDS handle is in POST mode and libcurl would otherwise inject
// `application/x-www-form-urlencoded`.) A host that knows better passes its own via the provider.
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

// libcurl's global init is not thread-safe on its first call; do it exactly once before any easy
// handle. (curl_global_cleanup is intentionally not called — process-lifetime, like other libcvc
// process-global singletons.)
void ensure_curl_global() {
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

// Accumulate a response body, refusing to grow past the cap (returning < the offered size makes
// libcurl abort the transfer with CURLE_WRITE_ERROR). Used by the reader for the fetched bytes and
// by the writer to bound-and-discard the server's reply.
std::size_t write_cb(char *ptr, std::size_t size, std::size_t nmemb, void *userdata) {
  auto *out = static_cast<std::string *>(userdata);
  const std::size_t n = size * nmemb;
  if (out->size() + n > kMaxHttpBytes)
    return 0;
  out->append(ptr, n);
  return n;
}

// RAII owners so the easy handle and the header list are freed on EVERY exit path — including a
// std::bad_alloc thrown while building the canonical-URL std::string between perform and cleanup
// (that unwinds through resolve()/store()'s catch(...), which would otherwise leak the
// handle+list).
using EasyHandle = std::unique_ptr<CURL, decltype(&curl_easy_cleanup)>;
using HeaderList = std::unique_ptr<struct curl_slist, decltype(&curl_slist_free_all)>;

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

// Build a libcurl header list into *out (nullptr for an empty list). FAIL-CLOSED: on an allocation
// failure it frees the partial list and returns false, so a caller aborts the request rather than
// silently sending it WITHOUT the auth header (curl_slist_append returns NULL without freeing the
// list it was given, and continuing would both leak and drop headers).
bool build_header_list(const std::vector<std::string> &lines, struct curl_slist **out) {
  struct curl_slist *list = nullptr;
  for (const std::string &line : lines) {
    struct curl_slist *next = curl_slist_append(list, line.c_str());
    if (!next) {
      curl_slist_free_all(list);
      *out = nullptr;
      return false;
    }
    list = next;
  }
  *out = list;
  return true;
}

// The setopts common to every request (read and write) EXCEPT redirect-following, which each caller
// sets by hand (a credentialed request must not auto-follow — see http_fetch/http_store): bounded
// timeouts, http/https-only for the request AND any redirect (an SSRF / local-file-exfil guard),
// thread-safe signal handling, a user-agent. Body/method/headers are layered on by the caller.
void set_common_opts(CURL *h) {
  curl_easy_setopt(h, CURLOPT_MAXREDIRS, 5L);
  curl_easy_setopt(h, CURLOPT_CONNECTTIMEOUT, 10L);
  curl_easy_setopt(h, CURLOPT_TIMEOUT, 30L);
  curl_easy_setopt(h, CURLOPT_FAILONERROR, 1L); // an HTTP >= 400 status is a failure
  curl_easy_setopt(h, CURLOPT_NOSIGNAL,
                   1L); // thread-safe: no SIGALRM-based timeouts. NOTE: with
                        // NOSIGNAL the DNS-phase timeout needs an async
                        // resolver — the curl recipe must build with the
                        // threaded resolver or c-ares (CURL_VERSION_ASYNCHDNS),
                        // else a stalled DNS lookup ignores CONNECTTIMEOUT.
  curl_easy_setopt(h, CURLOPT_USERAGENT, "cvc-ariadne/1");
  // CURLOPT_PROTOCOLS(_STR) are enum values, not macros — key the choice on the libcurl version
  // (the _STR forms arrived in 7.85.0; the LONG bitmask forms are deprecated there).
#if LIBCURL_VERSION_NUM >= 0x075500
  curl_easy_setopt(h, CURLOPT_PROTOCOLS_STR, "http,https");
  curl_easy_setopt(h, CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
  curl_easy_setopt(h, CURLOPT_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
  curl_easy_setopt(h, CURLOPT_REDIR_PROTOCOLS, static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
#endif
}

// The current response status (0 if unavailable).
long response_code(CURL *h) {
  long code = 0;
  curl_easy_getinfo(h, CURLINFO_RESPONSE_CODE, &code);
  return code;
}

UriResult http_fetch(const Uri &u, const std::string & /*base*/) {
  // Consult the provider FIRST — before curl_easy_init — so a throwing provider (host code) leaks
  // no handle; the throw unwinds to resolve()'s barrier. No provider -> default GET, no extra
  // headers.
  HttpRequestOptions opts;
  if (const HttpOptionsProvider p = current_provider())
    opts = p(u.raw, /*for_write=*/false);

  ensure_curl_global();
  EasyHandle h(curl_easy_init(), &curl_easy_cleanup);
  if (!h)
    return {false, std::string(), std::string(), "ari: http: could not init libcurl"};
  std::string body;
  curl_easy_setopt(h.get(), CURLOPT_URL, u.raw.c_str());
  curl_easy_setopt(h.get(), CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(h.get(), CURLOPT_WRITEDATA, &body);
  set_common_opts(h.get());
  // libcurl re-sends custom headers (X-Api-Key, X-Auth-Token, …) VERBATIM on every redirect hop and
  // only strips Authorization/Cookie/Proxy-Authorization on a cross-origin redirect. So follow a
  // redirect only when NO provider headers are attached (nothing to leak across an origin the
  // untrusted .ari document chose); a credentialed read stops at the redirect instead of carrying
  // the token onward. SSRF stays bounded either way by the http/https protocol restriction.
  const bool follow = opts.headers.empty();
  curl_easy_setopt(h.get(), CURLOPT_FOLLOWLOCATION, follow ? 1L : 0L);
  // A method override on a read is a bodyless custom verb (default stays GET).
  if (!opts.method.empty() && opts.method != "GET")
    curl_easy_setopt(h.get(), CURLOPT_CUSTOMREQUEST, opts.method.c_str());
  struct curl_slist *raw_hdrs = nullptr;
  if (!build_header_list(opts.headers, &raw_hdrs))
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' failed: out of memory building request headers"};
  HeaderList hdrs(raw_hdrs, &curl_slist_free_all);
  if (hdrs)
    curl_easy_setopt(h.get(), CURLOPT_HTTPHEADER, hdrs.get());

  const CURLcode rc = curl_easy_perform(h.get());
  const long status = response_code(h.get());
  char *eff = nullptr;
  curl_easy_getinfo(h.get(), CURLINFO_EFFECTIVE_URL, &eff); // post-redirect URL = the canonical id
  const std::string canonical = eff ? std::string(eff) : u.raw;
  if (rc != CURLE_OK)
    return {false, std::string(), std::string(),
            std::string("ari: http fetch of '") + u.raw + "' failed: " + curl_easy_strerror(rc)};
  // Not following (credentialed): a 3xx would otherwise be returned as OK with the redirect PAGE as
  // the body. Surface it instead of silently handing back the wrong bytes.
  if (!follow && status >= 300 && status < 400)
    return {false, std::string(), std::string(),
            "ari: http fetch of '" + u.raw + "' returned redirect status " +
                std::to_string(status) + " to '" + canonical +
                "'; a credentialed read does not follow redirects (the auth header would cross an "
                "origin). Resolve the target directly."};
  return {true, std::move(body), canonical, std::string()};
}

// §13.10 the write analogue: PUT (default) or POST/… the `content` bytes to the URL. The method and
// any auth headers come from the same provider as the reader (for_write = true). Response body is
// bounded and discarded — store reports only the canonical identity.
StoreResult http_store(const Uri &u, const std::string &content, const std::string & /*base*/) {
  HttpRequestOptions opts;
  if (const HttpOptionsProvider p = current_provider())
    opts = p(u.raw, /*for_write=*/true);
  const std::string method = opts.method.empty() ? std::string("PUT") : opts.method;

  ensure_curl_global();
  EasyHandle h(curl_easy_init(), &curl_easy_cleanup);
  if (!h)
    return {false, std::string(), "ari: http: could not init libcurl"};
  std::string sink; // the server's reply — bounded by write_cb, then discarded
  curl_easy_setopt(h.get(), CURLOPT_URL, u.raw.c_str());
  curl_easy_setopt(h.get(), CURLOPT_WRITEFUNCTION, write_cb);
  curl_easy_setopt(h.get(), CURLOPT_WRITEDATA, &sink);
  set_common_opts(h.get());
  // NEVER follow a redirect on a write: 301/302/303 silently drop the body (curl converts to a
  // bodyless GET) and 307/308 re-send the body — and the redirect target is a host the untrusted
  // .ari document chose. A 3xx on a store is therefore an error the host must resolve, not a
  // success.
  curl_easy_setopt(h.get(), CURLOPT_FOLLOWLOCATION, 0L);
  // Send the body with an explicit method. Order matters: set the size BEFORE COPYPOSTFIELDS so
  // libcurl copies exactly content.size() bytes (it would otherwise strlen() the buffer and
  // truncate an embedded NUL). COPYPOSTFIELDS makes libcurl copy the body (no lifetime worry) and
  // default the verb to POST; CUSTOMREQUEST then overrides the verb to PUT/POST/PATCH/… as chosen.
  curl_easy_setopt(h.get(), CURLOPT_POSTFIELDSIZE_LARGE, static_cast<curl_off_t>(content.size()));
  curl_easy_setopt(h.get(), CURLOPT_COPYPOSTFIELDS, content.data());
  curl_easy_setopt(h.get(), CURLOPT_CUSTOMREQUEST, method.c_str());
  // A COPYPOSTFIELDS handle is in POST mode, so libcurl injects a default
  // `Content-Type: application/x-www-form-urlencoded` unless the request already carries one —
  // wrong for opaque store bytes. Default a neutral type when the provider supplied none (its own
  // wins).
  std::vector<std::string> header_lines = opts.headers;
  if (!has_header(header_lines, "Content-Type"))
    header_lines.push_back(kDefaultStoreContentType);
  struct curl_slist *raw_hdrs = nullptr;
  if (!build_header_list(header_lines, &raw_hdrs))
    return {false, std::string(),
            "ari: http " + method + " to '" + u.raw +
                "' failed: out of memory building request headers"};
  HeaderList hdrs(raw_hdrs, &curl_slist_free_all);
  if (hdrs)
    curl_easy_setopt(h.get(), CURLOPT_HTTPHEADER, hdrs.get());

  const CURLcode rc = curl_easy_perform(h.get());
  const long status = response_code(h.get());
  char *eff = nullptr;
  curl_easy_getinfo(h.get(), CURLINFO_EFFECTIVE_URL, &eff);
  const std::string canonical = eff ? std::string(eff) : u.raw;
  if (rc != CURLE_OK)
    return {false, std::string(),
            std::string("ari: http ") + method + " to '" + u.raw +
                "' failed: " + curl_easy_strerror(rc)};
  if (status >= 300 && status < 400)
    return {false, std::string(),
            "ari: http " + method + " to '" + u.raw + "' returned redirect status " +
                std::to_string(status) + " to '" + canonical +
                "'; redirects are not followed on a write (the body would be dropped or re-sent to "
                "another origin). Store to the resolved location directly."};
  return {true, canonical, std::string()};
}

} // namespace

void set_http_options_provider(HttpOptionsProvider provider) {
  std::lock_guard<std::mutex> lock(provider_mutex());
  provider_cell() = std::move(provider);
}

void register_http_uri_handler() {
  register_uri_handler("http", http_fetch);
  register_uri_handler("https", http_fetch);
  register_uri_store_handler("http", http_store);
  register_uri_store_handler("https", http_store);
}

bool have_http_uri_handler() { return true; }

#else // no libcurl

void set_http_options_provider(HttpOptionsProvider /*provider*/) {
  // built without libcurl — no requests are made, so the provider is inert
}

void register_http_uri_handler() { /* built without libcurl — the scheme stays unresolved */ }

bool have_http_uri_handler() { return false; }

#endif

} // namespace ariadne
} // namespace cvc
