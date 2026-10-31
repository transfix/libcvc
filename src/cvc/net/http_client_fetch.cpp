// cvc::net — the Emscripten fetch transport (wasm builds). Compiled only when CVC_NET_HAVE_FETCH is
// defined (the CMake EMSCRIPTEN path); otherwise this TU is empty and the curl (or null) backend
// supplies make_default_backend().
//
// TWO ENTRIES. start() is the one that never blocks: it issues an async emscripten_fetch and
// returns; the browser's event loop delivers the reply to the onsuccess/onerror callbacks, on the
// thread that started it, when the app yields to it (the run loops' emscripten_sleep, or the end of
// a main-loop callback). In a build without threads that is how Ariadne's network verbs reach the
// network, so a page keeps drawing while a request is out (starts_async()).
//
// send() must BLOCK (Ariadne's resolve()/store() are synchronous), but a browser fetch is
// asynchronous. It issues the same async fetch (NO EMSCRIPTEN_FETCH_SYNCHRONOUS — that would need
// an off-main-thread pthread build) and spins on emscripten_sleep(0), which under -sASYNCIFY
// unwinds/rewinds the C++ stack to yield to the browser event loop until the transfer completes —
// exactly the primitive the demo run-loops already use to yield each frame. THEREFORE the wasm
// EXECUTABLE that links this backend MUST enable -sFETCH=1 and EITHER -sASYNCIFY (main thread — the
// demo gallery) OR -pthread and run resolve()/send() off the main thread. That is a link-time
// property of the executable, not of this interface; a wasm exe with neither cannot block on the
// network from the main thread. Nothing draws while send() spins: the frame that called it is on
// the stack.
//
// Two documented differences from the curl backend, inherent to browser fetch: requests are subject
// to CORS (a cross-origin .ari fragment must be CORS-enabled), and the effective post-redirect URL
// is not exposed (canonical_url falls back to the request URL).

#include <cvc/net/http_client.h>

#ifdef CVC_NET_HAVE_FETCH

#include "http_backends.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <emscripten/emscripten.h>
#include <emscripten/fetch.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
namespace net {
namespace {

// XHR readyState value for a completed transfer (success OR error — status distinguishes them).
constexpr unsigned short kReadyStateDone = 4;

// The emscripten_fetch_attr_t for `req`, pointing into `owned`/`ptrs`, which must live until
// emscripten_fetch() returns (it copies the headers, and the body is read as the XHR is sent).
void fill_attr(emscripten_fetch_attr_t &attr, const HttpRequest &req,
               std::vector<std::string> &owned, std::vector<const char *> &ptrs) {
  emscripten_fetch_attr_init(&attr);
  const std::string method = req.method.empty() ? std::string("GET") : req.method;
  std::snprintf(attr.requestMethod, sizeof(attr.requestMethod), "%s", method.c_str());
  // Buffer the whole body to memory; keep OUR cache authoritative rather than the browser HTTP
  // cache / IndexedDB (no EMSCRIPTEN_FETCH_PERSIST_FILE).
  attr.attributes = EMSCRIPTEN_FETCH_LOAD_TO_MEMORY | EMSCRIPTEN_FETCH_REPLACE;
  // KNOWN DIVERGENCE (documented, not fixable here): emscripten_fetch_attr_t exposes no redirect
  // control, so req.follow_redirects and req.max_redirects cannot be honored on wasm — XHR always
  // follows redirects up to the browser's own (~20) limit. The curl backend enforces both. A
  // caller that depends on no-follow as a credential/SSRF guard must not rely on it in the
  // browser.
  attr.timeoutMSecs =
      req.timeout_secs > 0 ? static_cast<unsigned long>(req.timeout_secs) * 1000u : 0u;

  // Split each verbatim "Name: value" line into the alternating {name, value, …, NULL} array
  // Emscripten wants.
  owned.clear();
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
  ptrs.clear();
  if (!owned.empty()) {
    ptrs.reserve(owned.size() + 1);
    for (const std::string &s : owned)
      ptrs.push_back(s.c_str());
    ptrs.push_back(nullptr);
    attr.requestHeaders = ptrs.data();
  }

  if (!req.body.empty()) {
    attr.requestData = req.body.data();
    attr.requestDataSize = req.body.size();
  }
}

// Blank the request's copies of what it carried (a credential the host put in), as the caller's
// own copies are.
void wipe(std::vector<std::string> &owned) {
  for (std::string &s : owned)
    std::fill(s.begin(), s.end(), '\0');
  owned.clear();
}

// The reply of a DONE transfer `f` (its callbacks have run, or readyState is 4). `timeout_secs`
// tells a timeout from another failure (the browser reports both as status 0).
HttpResponse read_reply(emscripten_fetch_t *f, const HttpRequest &req, double elapsed_secs) {
  HttpResponse resp;
  resp.canonical_url = req.url; // browser fetch exposes no effective URL; best-effort
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
    resp.ok = false;
    resp.error = "cvc::net: response exceeds max_bytes cap";
    return resp;
  }
  if (f->data && got > 0)
    resp.body.assign(f->data, got);

  if (resp.status == 0) {
    resp.ok = false;
    resp.error = req.timeout_secs > 0 && elapsed_secs >= double(req.timeout_secs)
                     ? "cvc::net: timed out after " + std::to_string(req.timeout_secs) + " s"
                     : "cvc::net: fetch failed (network or CORS)";
    return resp;
  }
  resp.ok = true;
  return resp;
}

// A request start() issued. It keeps itself alive (self_) until the browser answers or it is
// aborted; the fetch's userData points at it. live_ is true until then: emscripten calls onerror in
// place both when the XHR cannot be sent (inside emscripten_fetch, before fetch_ is set) and when
// an in-flight fetch is closed (abort), and only the first answer counts. An aborted fetch's
// runtime keepalive is released by Fetch.js itself, as its XHR reports the abort.
class FetchTransfer : public HttpTransfer {
public:
  static std::shared_ptr<HttpTransfer> begin(const HttpRequest &req, HttpDone done) {
    auto t = std::make_shared<FetchTransfer>();
    t->req_ = req;
    t->done_ = std::move(done);
    std::vector<std::string> owned;
    std::vector<const char *> ptrs;
    emscripten_fetch_attr_t attr;
    fill_attr(attr, t->req_, owned, ptrs);
    attr.userData = t.get();
    attr.onsuccess = &FetchTransfer::on_done; // a 2xx
    attr.onerror = &FetchTransfer::on_done;   // any other status, a network error, a timeout
    attr.onprogress = &FetchTransfer::on_progress;
    t->started_ = emscripten_get_now();
    t->self_ = t;
    t->live_ = true;
    emscripten_fetch_t *f = emscripten_fetch(&attr, t->req_.url.c_str());
    wipe(owned);
    // Read as the XHR was sent; the reply needs neither.
    std::fill(t->req_.body.begin(), t->req_.body.end(), '\0');
    t->req_.body.clear();
    for (std::string &h : t->req_.headers)
      std::fill(h.begin(), h.end(), '\0');
    t->req_.headers.clear();
    if (!t->live_)
      return nullptr; // answered (and closed) in place
    if (!f) {
      t->live_ = false;
      HttpResponse r;
      r.canonical_url = req.url;
      r.error = "cvc::net: emscripten_fetch returned null";
      t->deliver(std::move(r));
      return nullptr;
    }
    t->fetch_ = f;
    return t;
  }

  void abort() override {
    std::shared_ptr<FetchTransfer> keep = std::move(self_); // released as this returns
    HttpDone dropped = std::move(done_);
    done_ = nullptr;
    if (!live_)
      return;
    live_ = false;
    if (emscripten_fetch_t *f = std::exchange(fetch_, nullptr))
      emscripten_fetch_close(f); // the browser aborts the XHR; its callbacks no longer run
  }

private:
  static void on_done(emscripten_fetch_t *f) {
    auto *t = static_cast<FetchTransfer *>(f->userData);
    if (!t || !t->live_)
      return;
    t->live_ = false;
    t->fetch_ = nullptr;
    HttpResponse r = read_reply(f, t->req_, (emscripten_get_now() - t->started_) / 1000.0);
    emscripten_fetch_close(f);
    t->deliver(std::move(r));
  }

  // The cap, enforced as the body arrives rather than once the browser holds all of it.
  static void on_progress(emscripten_fetch_t *f) {
    auto *t = static_cast<FetchTransfer *>(f->userData);
    if (!t || !t->live_ || t->fetch_ != f)
      return;
    const uint64_t seen = std::max<uint64_t>(f->dataOffset + f->numBytes, f->totalBytes);
    if (seen <= t->req_.max_bytes)
      return;
    HttpResponse r;
    r.canonical_url = t->req_.url;
    r.status = f->status;
    r.error = "cvc::net: response exceeds max_bytes cap";
    t->live_ = false;
    t->fetch_ = nullptr;
    emscripten_fetch_close(f);
    t->deliver(std::move(r));
  }

  void deliver(HttpResponse r) {
    std::shared_ptr<FetchTransfer> keep = std::move(self_); // alive through the callback
    HttpDone done = std::move(done_);
    done_ = nullptr;
    if (done)
      done(std::move(r));
  }

  HttpRequest req_;
  HttpDone done_;
  emscripten_fetch_t *fetch_ = nullptr;
  bool live_ = false;
  double started_ = 0.0; // emscripten_get_now(), ms
  std::shared_ptr<FetchTransfer> self_;
};

class FetchHttpClient : public HttpClient {
public:
  // XHR follows redirects itself (see KNOWN DIVERGENCE in fill_attr).
  bool follows_redirects_itself() const override { return true; }

#ifndef __EMSCRIPTEN_PTHREADS__
  // Without threads the frame loop and every request share the page's main thread: a request must
  // not hold it. (With -pthread a request blocks a worker of the app's background pool instead.)
  bool starts_async() const override { return true; }
#endif

  std::shared_ptr<HttpTransfer> start(const HttpRequest &req, HttpDone done) override {
    return FetchTransfer::begin(req, std::move(done));
  }

  HttpResponse send(const HttpRequest &req) override {
    std::vector<std::string> owned;
    std::vector<const char *> ptrs;
    emscripten_fetch_attr_t attr;
    fill_attr(attr, req, owned, ptrs);
    const double started = emscripten_get_now();
    emscripten_fetch_t *f = emscripten_fetch(&attr, req.url.c_str());
    wipe(owned);
    if (!f) {
      HttpResponse resp;
      resp.canonical_url = req.url;
      resp.error = "cvc::net: emscripten_fetch returned null";
      return resp;
    }
    // Block by yielding to the event loop until the transfer is DONE (see the file header re:
    // Asyncify / pthread). timeoutMSecs guards against a stuck transfer.
    while (f->readyState != kReadyStateDone)
      emscripten_sleep(0);
    HttpResponse resp = read_reply(f, req, (emscripten_get_now() - started) / 1000.0);
    emscripten_fetch_close(f);
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
