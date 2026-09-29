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

} // namespace

HttpResponse send(const HttpRequest &req) { return current_client()->send(req); }

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
