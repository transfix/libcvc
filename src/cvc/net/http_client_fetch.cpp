// cvc::net — the Emscripten fetch transport (wasm builds). Compiled only when CVC_NET_HAVE_FETCH is
// defined (the CMake EMSCRIPTEN path); otherwise this TU is empty and the curl (or null) backend
// supplies make_default_backend().
//
// SYNCHRONY on the browser main thread: send() must BLOCK (Ariadne's resolve()/store() are
// synchronous), but a browser fetch is asynchronous. We issue an ASYNC emscripten_fetch (NO
// EMSCRIPTEN_FETCH_SYNCHRONOUS — that would need an off-main-thread pthread build) and spin on
// emscripten_sleep(0), which under -sASYNCIFY unwinds/rewinds the C++ stack to yield to the browser
// event loop until the transfer completes — exactly the primitive the demo run-loops already use to
// yield each frame. THEREFORE the wasm EXECUTABLE that links this backend MUST enable -sFETCH=1 and
// EITHER -sASYNCIFY (main thread — the demo gallery) OR -pthread and run resolve()/send() off the
// main thread. That is a link-time property of the executable, not of this interface; a wasm exe
// with neither cannot block on the network from the main thread.
//
// Two documented differences from the curl backend, inherent to browser fetch: requests are subject
// to CORS (a cross-origin .ari fragment must be CORS-enabled), and the effective post-redirect URL
// is not exposed (canonical_url falls back to the request URL).

#include <cvc/net/http_client.h>

#ifdef CVC_NET_HAVE_FETCH

#include "http_backends.h"

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <emscripten/emscripten.h>
#include <emscripten/fetch.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace net {
namespace {

// XHR readyState value for a completed transfer (success OR error — status distinguishes them).
constexpr unsigned short kReadyStateDone = 4;

class FetchHttpClient : public HttpClient {
public:
  HttpResponse send(const HttpRequest &req) override {
    HttpResponse resp;
    resp.canonical_url = req.url; // browser fetch exposes no effective URL; best-effort

    emscripten_fetch_attr_t attr;
    emscripten_fetch_attr_init(&attr);
    const std::string method = req.method.empty() ? std::string("GET") : req.method;
    std::snprintf(attr.requestMethod, sizeof(attr.requestMethod), "%s", method.c_str());
    // Buffer the whole body to memory; keep OUR cache authoritative rather than the browser HTTP
    // cache / IndexedDB (no EMSCRIPTEN_FETCH_PERSIST_FILE).
    attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY | EMSCRIPTEN_FETCH_REPLACE;
    attr.timeoutMSecs =
        req.timeout_secs > 0 ? static_cast<unsigned long>(req.timeout_secs) * 1000u : 0u;

    // Split each verbatim "Name: value" line into the alternating {name, value, …, NULL} array
    // Emscripten wants. `owned` keeps the split strings alive across the fetch + spin.
    std::vector<std::string> owned;
    owned.reserve(req.headers.size() * 2);
    for (const std::string &line : req.headers) {
      const std::size_t colon = line.find(':');
      if (colon == std::string::npos) {
        owned.push_back(line);
        owned.emplace_back();
        continue;
      }
      std::string value = line.substr(colon + 1);
      if (!value.empty() && value.front() == ' ')
        value.erase(0, 1); // the single space in "Name: value"
      owned.push_back(line.substr(0, colon));
      owned.push_back(std::move(value));
    }
    std::vector<const char *> header_ptrs;
    if (!owned.empty()) {
      header_ptrs.reserve(owned.size() + 1);
      for (const std::string &s : owned)
        header_ptrs.push_back(s.c_str());
      header_ptrs.push_back(nullptr);
      attr.requestHeaders = header_ptrs.data();
    }

    if (!req.body.empty()) {
      attr.requestData = req.body.data();
      attr.requestDataSize = req.body.size();
    }

    emscripten_fetch_t *f = emscripten_fetch(&attr, req.url.c_str());
    if (!f) {
      resp.error = "cvc::net: emscripten_fetch returned null";
      return resp;
    }
    // Block by yielding to the event loop until the transfer is DONE (see the file header re:
    // Asyncify / pthread). timeoutMSecs guards against a stuck transfer.
    while (f->readyState != kReadyStateDone)
      emscripten_sleep(0);

    resp.status = f->status;

    const std::size_t hlen = emscripten_fetch_get_response_headers_length(f);
    if (hlen > 0) {
      std::string blob(hlen + 1, '\0');
      emscripten_fetch_get_response_headers(f, &blob[0], blob.size());
      blob.resize(std::strlen(blob.c_str()));
      resp.headers = parse_response_header_blob(blob);
    }

    const std::size_t got = static_cast<std::size_t>(f->numBytes);
    if (got > req.max_bytes) {
      // The browser already loaded it all to memory, so we cannot abort early as curl does; refuse.
      emscripten_fetch_close(f);
      resp.ok = false;
      resp.error = "cvc::net: response exceeds max_bytes cap";
      return resp;
    }
    if (f->data && got > 0)
      resp.body.assign(f->data, got);

    const long status = f->status;
    emscripten_fetch_close(f);
    if (status == 0) {
      resp.ok = false;
      resp.error = "cvc::net: fetch failed (network or CORS)";
      return resp;
    }
    resp.ok = true;
    return resp;
  }
};

} // namespace

namespace detail {
std::unique_ptr<HttpClient> make_default_backend() { return std::make_unique<FetchHttpClient>(); }
} // namespace detail

} // namespace net
} // namespace cvc

#endif // CVC_NET_HAVE_FETCH
