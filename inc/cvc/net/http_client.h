#ifndef CVC_NET_HTTP_CLIENT_H
#define CVC_NET_HTTP_CLIENT_H

// cvc::net — a small, synchronous HTTP client facade (roadmap §13.x).
//
// One transport-neutral interface (HttpClient::send) with TWO interchangeable backends chosen at
// CONFIGURE time, never at the call site:
//   - libcurl            on native builds        (CVC_NET_HAVE_CURL)
//   - Emscripten fetch    on wasm/Emscripten builds (CVC_NET_HAVE_FETCH)
//   - a NullHttpClient    when neither is compiled  (send() reports "no backend")
// so a caller writes the SAME code on both targets and the right transport is linked. Ariadne's
// http(s):// URI handler (ariadne/uri_http.cpp) and the §13.9 HTTP cache are both thin adapters
// over this — that is why they work identically native and in the browser.
//
// SYNCHRONY: send() BLOCKS until the full (capped) response body is buffered or an error occurs,
// and returns by value. This matches Ariadne's synchronous resolve()/store() contract, so no caller
// grows an async path. On native, libcurl blocks naturally. On wasm the fetch backend issues an
// async emscripten_fetch and spins on emscripten_sleep(0) — which only returns synchronously when
// the EXECUTABLE is linked with -sASYNCIFY (the demo gallery) or runs off the main thread on a
// -pthread build; see src/cvc/net/http_client_fetch.cpp and the CMake note. This is a LINK
// requirement of the wasm executable, not a property of this interface.
//
// This header is deliberately dependency-light (only <cstddef>/<memory>/<string>/<vector>) and
// knows nothing about curl, Emscripten, or Ariadne — the dependency arrows point INTO cvc::net.

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace net {

// One HTTP request. `method` empty means the backend's default verb (GET). `headers` are verbatim
// "Name: value" lines (the libcurl slist / fetch convention preserved end to end) — a host injects
// auth here. `body` is empty for a GET; a non-empty body is sent with `method` (e.g. PUT/POST).
// `follow_redirects` is decided by the CALLER (a credentialed request should not auto-follow — see
// uri_http.cpp), never baked into the transport. `max_bytes` caps the buffered response so a chatty
// endpoint cannot OOM the process (the curl backend aborts the transfer past it; the fetch backend,
// which the browser has already loaded to memory, reports an error).
struct HttpRequest {
  std::string url;
  std::string method;                          // "" => GET
  std::vector<std::string> headers;            // verbatim "Name: value" request-header lines
  std::string body;                            // empty for a bodyless GET
  bool follow_redirects = true;                // caller policy; the transport just obeys it
  std::size_t max_bytes = 64u * 1024u * 1024u; // response-size cap (64 MiB)
  long connect_timeout_secs = 10;
  long timeout_secs = 30;
  long max_redirects = 5;
  std::string user_agent = "cvc-net/1";
};

// One HTTP response. `ok` means the TRANSPORT succeeded (a request was made and a reply received) —
// NOT that the HTTP status was 2xx. A caller inspects `status` itself: this facade never treats a
// 4xx/5xx (or a 304) as a transport error, precisely so the HTTP cache can act on a 304 and an
// adapter can map ">= 400" to failure on its own terms. `headers` are the FINAL response's header
// lines ("Name: value", status line and blank line dropped) — the validators (ETag/Last-Modified/
// Cache-Control) the cache reads. `canonical_url` is the effective post-redirect URL (best-effort
// on wasm, where the browser exposes no effective URL — it falls back to the request URL). On a
// transport failure `ok` is false and `error` is non-empty.
struct HttpResponse {
  bool ok = false;
  long status = 0; // HTTP status code; 0 if the transport never got a reply
  std::string body;
  std::string canonical_url;
  std::vector<std::string> headers; // final-response header lines "Name: value"
  std::string error;                // non-empty transport/setup error text when !ok
};

// The transport interface. One synchronous, non-throwing call: a network error is reported through
// HttpResponse.ok/error, never as an exception (send() runs inside resolve()/store()'s barrier, but
// callers must be able to rely on a value either way).
class HttpClient {
public:
  virtual ~HttpClient() = default;
  virtual HttpResponse send(const HttpRequest &req) = 0;
  // Whether this is a real transport (false only for the no-backend NullHttpClient). Lets a host
  // ask "was an HTTP backend compiled in?" without special-casing the null build. See
  // have_http_backend.
  virtual bool is_real() const { return true; }
};

// Send `req` through the process-global client (the compiled backend, or one a host injected via
// set_http_client). Thread-safe: the current client is captured under a lock so a concurrent
// set_http_client cannot pull it out from under an in-flight call. This is the primary entry — the
// http URI handler and the cache both call it.
HttpResponse send(const HttpRequest &req);

// Replace the process-global client — the seam a HOST or a TEST uses to inject a fake transport
// (canned responses, request capture) with no live network. Pass nullptr to restore the compiled
// default backend. Thread-safe; typically called once at setup, before concurrent send()s.
void set_http_client(std::unique_ptr<HttpClient> client);

// Whether a REAL HTTP backend is compiled into this build (curl or fetch) — i.e. whether the
// default client can actually make requests. Independent of any injected client. Ariadne's
// have_http_uri_handler() forwards to this.
bool have_http_backend();

// Parse an HTTP response-header BLOB (header lines joined by CRLF or LF, as Emscripten's
// emscripten_fetch_get_response_headers returns) into individual "Name: value" lines, dropping the
// "HTTP/… <status>" status line(s) and blank lines — matching the line form the curl backend
// produces. Exposed (and pure) so it is unit-testable on native, where the fetch backend never
// runs.
std::vector<std::string> parse_response_header_blob(const std::string &blob);

} // namespace net
} // namespace cvc

#endif // CVC_NET_HTTP_CLIENT_H
