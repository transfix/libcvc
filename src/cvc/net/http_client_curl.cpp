// cvc::net — the libcurl transport (native builds). Compiled only when CVC_NET_HAVE_CURL is defined
// (the CMake find_package(CURL) path); otherwise this TU is empty and another backend supplies
// make_default_backend(). The curl setup here was lifted verbatim from ariadne/uri_http.cpp when
// the http handler was refactored onto cvc::net, with three deliberate changes for the facade
// contract:
//   - CURLOPT_FAILONERROR is NOT set, so a >= 400 (or a 304) comes back as ok=true with its status.
//     The caller judges status (a cache must SEE a 304; the Ariadne adapter re-imposes >= 400 as an
//     error).
//   - response headers + status are captured into HttpResponse (the cache reads the validators).
//   - the response-size cap and redirect-following are per-request (HttpRequest), not file-global.

#include <cvc/net/http_client.h>

#ifdef CVC_NET_HAVE_CURL

#include "http_backends.h"

#include <cstddef>
#include <curl/curl.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace cvc {
namespace net {
namespace {

// libcurl's global init is not thread-safe on its first call; do it exactly once before any easy
// handle. (curl_global_cleanup is intentionally not called — process-lifetime, like other libcvc
// process-global singletons.)
void ensure_curl_global() {
  static std::once_flag once;
  std::call_once(once, [] { curl_global_init(CURL_GLOBAL_DEFAULT); });
}

// The response accumulator plus its per-request cap (write_cb has no other way to see max_bytes).
struct WriteCtx {
  std::string *out;
  std::size_t cap;
};

// Accumulate the body, refusing to grow past the cap (returning < the offered size makes libcurl
// abort the transfer with CURLE_WRITE_ERROR — surfaced as a transport error).
std::size_t write_cb(char *ptr, std::size_t size, std::size_t nmemb, void *userdata) {
  auto *w = static_cast<WriteCtx *>(userdata);
  const std::size_t n = size * nmemb;
  if (w->out->size() + n > w->cap)
    return 0;
  w->out->append(ptr, n);
  return n;
}

// Capture response header lines. libcurl calls this once per header line per hop (including the
// "HTTP/…" status line and the terminating blank line). Clearing on each status line leaves only
// the FINAL response's headers after all redirects; the status/blank lines themselves are dropped,
// so the vector matches parse_response_header_blob's "Name: value" form.
std::size_t header_cb(char *buf, std::size_t size, std::size_t nitems, void *userdata) {
  auto *out = static_cast<std::vector<std::string> *>(userdata);
  const std::size_t n = size * nitems;
  std::string line(buf, n);
  while (!line.empty() && (line.back() == '\n' || line.back() == '\r'))
    line.pop_back();
  if (line.rfind("HTTP/", 0) == 0) {
    out->clear(); // a new (final) response begins — drop the earlier hop's headers
    return n;
  }
  if (!line.empty())
    out->push_back(std::move(line));
  return n;
}

// RAII owners so the easy handle and the header list are freed on EVERY exit path, including a
// std::bad_alloc thrown while building a std::string between perform and cleanup.
using EasyHandle = std::unique_ptr<CURL, decltype(&curl_easy_cleanup)>;
using HeaderList = std::unique_ptr<struct curl_slist, decltype(&curl_slist_free_all)>;

// Build a libcurl header list into *out (nullptr for an empty list). FAIL-CLOSED: on an allocation
// failure it frees the partial list and returns false, so the caller aborts rather than silently
// sending the request WITHOUT the auth header (curl_slist_append returns NULL without freeing the
// list it was given).
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

class CurlHttpClient : public HttpClient {
public:
  HttpResponse send(const HttpRequest &req) override {
    HttpResponse resp;
    resp.canonical_url = req.url;

    ensure_curl_global();
    EasyHandle h(curl_easy_init(), &curl_easy_cleanup);
    if (!h) {
      resp.error = "cvc::net: could not init libcurl";
      return resp;
    }

    WriteCtx wc{&resp.body, req.max_bytes};
    curl_easy_setopt(h.get(), CURLOPT_URL, req.url.c_str());
    curl_easy_setopt(h.get(), CURLOPT_WRITEFUNCTION, write_cb);
    curl_easy_setopt(h.get(), CURLOPT_WRITEDATA, &wc);
    curl_easy_setopt(h.get(), CURLOPT_HEADERFUNCTION, header_cb);
    curl_easy_setopt(h.get(), CURLOPT_HEADERDATA, &resp.headers);

    curl_easy_setopt(h.get(), CURLOPT_MAXREDIRS, req.max_redirects);
    curl_easy_setopt(h.get(), CURLOPT_CONNECTTIMEOUT, req.connect_timeout_secs);
    curl_easy_setopt(h.get(), CURLOPT_TIMEOUT, req.timeout_secs);
    // Thread-safe: no SIGALRM-based timeouts. NOTE: with NOSIGNAL the DNS-phase timeout needs an
    // async resolver — the curl recipe must build with the threaded resolver or c-ares
    // (CURL_VERSION_ASYNCHDNS), else a stalled DNS lookup ignores CONNECTTIMEOUT.
    curl_easy_setopt(h.get(), CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(h.get(), CURLOPT_USERAGENT, req.user_agent.c_str());
    // NO CURLOPT_FAILONERROR: the facade reports a >= 400 (and a 304) with the status set, and the
    // CALLER decides what is an error — a cache must be able to see a 304.
    // CURLOPT_PROTOCOLS(_STR) are enum values, not macros — key the choice on the libcurl version
    // (the _STR forms arrived in 7.85.0; the LONG bitmask forms are deprecated there). http/https
    // only, for the request AND any redirect — an SSRF / local-file-exfil guard.
#if LIBCURL_VERSION_NUM >= 0x075500
    curl_easy_setopt(h.get(), CURLOPT_PROTOCOLS_STR, "http,https");
    curl_easy_setopt(h.get(), CURLOPT_REDIR_PROTOCOLS_STR, "http,https");
#else
    curl_easy_setopt(h.get(), CURLOPT_PROTOCOLS,
                     static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
    curl_easy_setopt(h.get(), CURLOPT_REDIR_PROTOCOLS,
                     static_cast<long>(CURLPROTO_HTTP | CURLPROTO_HTTPS));
#endif
    curl_easy_setopt(h.get(), CURLOPT_FOLLOWLOCATION, req.follow_redirects ? 1L : 0L);

    // Body + method. Order matters: set the size BEFORE COPYPOSTFIELDS so libcurl copies exactly
    // body.size() bytes (it would otherwise strlen() the buffer and truncate an embedded NUL).
    // COPYPOSTFIELDS copies the body (no lifetime worry) and defaults the verb to POST;
    // CUSTOMREQUEST then overrides it to the requested method. A bodyless custom verb sets
    // CUSTOMREQUEST only; a plain GET sets neither.
    if (!req.body.empty()) {
      curl_easy_setopt(h.get(), CURLOPT_POSTFIELDSIZE_LARGE,
                       static_cast<curl_off_t>(req.body.size()));
      curl_easy_setopt(h.get(), CURLOPT_COPYPOSTFIELDS, req.body.data());
    }
    if (!req.method.empty())
      curl_easy_setopt(h.get(), CURLOPT_CUSTOMREQUEST, req.method.c_str());

    struct curl_slist *raw_hdrs = nullptr;
    if (!build_header_list(req.headers, &raw_hdrs)) {
      resp.error = "cvc::net: out of memory building request headers";
      return resp;
    }
    HeaderList hdrs(raw_hdrs, &curl_slist_free_all);
    if (hdrs)
      curl_easy_setopt(h.get(), CURLOPT_HTTPHEADER, hdrs.get());

    const CURLcode rc = curl_easy_perform(h.get());
    long status = 0;
    curl_easy_getinfo(h.get(), CURLINFO_RESPONSE_CODE, &status);
    resp.status = status;
    char *eff = nullptr;
    curl_easy_getinfo(h.get(), CURLINFO_EFFECTIVE_URL,
                      &eff); // post-redirect URL = the canonical id
    if (eff)
      resp.canonical_url = eff;
    if (rc != CURLE_OK) {
      resp.ok = false;
      resp.error = std::string("cvc::net: ") + curl_easy_strerror(rc);
      return resp;
    }
    resp.ok = true;
    return resp;
  }
};

} // namespace

namespace detail {
std::unique_ptr<HttpClient> make_default_backend() { return std::make_unique<CurlHttpClient>(); }
} // namespace detail

} // namespace net
} // namespace cvc

#endif // CVC_NET_HAVE_CURL
