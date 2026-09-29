// cvc::net — the transport-neutral dispatcher: the process-global current client, the injection
// seam, have_http_backend(), and the shared response-header parser. The actual transports live in
// http_client_curl.cpp (native) and http_client_fetch.cpp (wasm); this TU also supplies the
// NullHttpClient + make_default_backend() when NEITHER backend is compiled. See http_client.h.

#include "http_backends.h"

#include <cvc/net/http_client.h>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace net {

namespace {

// The current client and its guard. The client starts unset and is lazily initialised to the
// compiled default on first use, so no static-init-order dependency on the backend TU. A reader
// copies the shared_ptr out under the lock (current_client) — holding a ref-count — so a concurrent
// set_http_client that swaps the cell cannot destroy the client mid-send().
std::mutex &client_mutex() {
  static std::mutex m;
  return m;
}
std::shared_ptr<HttpClient> &client_cell() {
  static std::shared_ptr<HttpClient> c;
  return c;
}
std::shared_ptr<HttpClient> current_client() {
  std::lock_guard<std::mutex> lock(client_mutex());
  std::shared_ptr<HttpClient> &c = client_cell();
  if (!c)
    c = detail::make_default_backend();
  return c;
}

// Case-insensitive ASCII equality of a method token against a literal.
bool method_ieq(const std::string &m, const char *lit) {
  std::size_t i = 0;
  for (; i < m.size() && lit[i]; ++i) {
    char a = m[i];
    if (a >= 'a' && a <= 'z')
      a = static_cast<char>(a - 'a' + 'A');
    char b = lit[i];
    if (b >= 'a' && b <= 'z')
      b = static_cast<char>(b - 'a' + 'A');
    if (a != b)
      return false;
  }
  return i == m.size() && lit[i] == '\0';
}

// GET and HEAD must not carry a request body: XHR (the wasm backend) silently drops it, and libcurl
// mis-sends it — so we strip it centrally and BOTH targets behave identically.
bool method_forbids_body(const std::string &m) {
  return method_ieq(m, "GET") || method_ieq(m, "HEAD");
}

// A syntactically valid method: a non-empty RFC 7230 "token" (visible ASCII minus separators), and
// short enough for the fetch backend's fixed 32-byte requestMethod buffer (so it is never silently
// truncated on wasm). Rejecting an out-of-spec verb here is uniform across both transports.
bool method_is_valid(const std::string &m) {
  if (m.empty() || m.size() > 31)
    return false;
  for (unsigned char c : m) {
    const bool tchar =
        (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
        std::string("!#$%&'*+-.^_`|~").find(static_cast<char>(c)) != std::string::npos;
    if (!tchar)
      return false;
  }
  return true;
}

} // namespace

// The dispatcher NORMALIZES every request before it reaches a backend, so the curl (native) and
// fetch (wasm) transports see one identical, well-formed request — the "same code on both targets"
// contract this facade promises. Without this the two diverge: an empty method becomes GET on fetch
// but POST-with-body on curl; a GET/HEAD body is sent by curl but dropped by XHR; a >31-char verb
// is truncated only on wasm. We settle all three here, once, rather than in each backend.
HttpResponse send(const HttpRequest &in) {
  HttpRequest req = in; // a working copy we may normalize
  if (req.method.empty())
    req.method = "GET"; // the documented default, made explicit so both backends agree
  if (!method_is_valid(req.method)) {
    HttpResponse r;
    r.ok = false;
    r.status = 0;
    r.canonical_url = req.url;
    r.error = "cvc::net: invalid HTTP method";
    return r;
  }
  if (method_forbids_body(req.method))
    req.body.clear(); // GET/HEAD carry no body on either transport
  return current_client()->send(req);
}

void set_http_client(std::unique_ptr<HttpClient> client) {
  std::lock_guard<std::mutex> lock(client_mutex());
  if (client)
    client_cell() = std::shared_ptr<HttpClient>(std::move(client));
  else
    client_cell() = std::shared_ptr<HttpClient>(detail::make_default_backend()); // restore default
}

bool have_http_backend() {
  // Reflects the COMPILED capability, independent of any injected client: ask a fresh default
  // backend (the backend ctors are trivial). Null → false; curl/fetch → true.
  return detail::make_default_backend()->is_real();
}

std::vector<std::string> parse_response_header_blob(const std::string &blob) {
  std::vector<std::string> out;
  std::string line;
  auto flush = [&]() {
    // Drop the "HTTP/… <status>" status line(s) and blank lines; keep "Name: value" header lines.
    if (!line.empty() && line.rfind("HTTP/", 0) != 0)
      out.push_back(line);
    line.clear();
  };
  for (char ch : blob) {
    if (ch == '\n')
      flush();
    else if (ch != '\r')
      line.push_back(ch);
  }
  flush(); // a trailing header with no final newline
  return out;
}

// The no-backend build: the null transport plus the make_default_backend() the linker resolves when
// neither CVC_NET_HAVE_CURL nor CVC_NET_HAVE_FETCH is defined.
#if !defined(CVC_NET_HAVE_CURL) && !defined(CVC_NET_HAVE_FETCH)

namespace {
class NullHttpClient : public HttpClient {
public:
  bool is_real() const override { return false; }
  HttpResponse send(const HttpRequest &req) override {
    HttpResponse r;
    r.ok = false;
    r.status = 0;
    r.canonical_url = req.url;
    r.error = "cvc::net: no HTTP backend compiled in this build";
    return r;
  }
};
} // namespace

namespace detail {
std::unique_ptr<HttpClient> make_default_backend() { return std::make_unique<NullHttpClient>(); }
} // namespace detail

#endif // no backend

} // namespace net
} // namespace cvc
