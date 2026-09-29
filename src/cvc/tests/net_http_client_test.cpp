// cvc::net HTTP facade tests. Three layers, none needing the external network:
//   - parse_response_header_blob: the pure splitter the wasm fetch backend feeds into HttpResponse
//     (native CI never runs the fetch backend, so this is its only direct coverage).
//   - the injection seam: set_http_client() swaps in a fake transport; send() dispatches to it;
//     nullptr restores the compiled default. This is the seam the §13.9 cache tests will use.
//   - the compiled backend end to end against a throwaway localhost server, asserting the NEW
//   status
//     + response-header capture (the http URI handler tests in ariadne_loader_test cover the
//     request side; here we prove the response side the cache depends on).

#include <atomic>
#include <boost/asio.hpp> // a throwaway localhost server for the live-backend test
#include <chrono>
#include <cvc/net/http_client.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using cvc::net::HttpRequest;
using cvc::net::HttpResponse;

// --- parse_response_header_blob ----------------------------------------------------------------

TEST(NetHeaderBlob, SplitsCrlfDropsStatusAndBlank) {
  const std::string blob = "HTTP/1.1 200 OK\r\n"
                           "Content-Type: text/plain\r\n"
                           "ETag: \"abc123\"\r\n"
                           "\r\n";
  const std::vector<std::string> lines = cvc::net::parse_response_header_blob(blob);
  ASSERT_EQ(lines.size(), 2u); // status line + blank dropped
  EXPECT_EQ(lines[0], "Content-Type: text/plain");
  EXPECT_EQ(lines[1], "ETag: \"abc123\"");
}

TEST(NetHeaderBlob, HandlesLfOnlyAndNoTrailingNewline) {
  const std::string blob =
      "HTTP/2 304\nETag: \"v9\"\nCache-Control: max-age=60"; // no final newline
  const std::vector<std::string> lines = cvc::net::parse_response_header_blob(blob);
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_EQ(lines[0], "ETag: \"v9\"");
  EXPECT_EQ(lines[1], "Cache-Control: max-age=60");
}

TEST(NetHeaderBlob, EmptyBlobIsEmpty) {
  EXPECT_TRUE(cvc::net::parse_response_header_blob("").empty());
  EXPECT_TRUE(cvc::net::parse_response_header_blob("HTTP/1.1 200 OK\r\n\r\n").empty());
}

// --- the injection seam ------------------------------------------------------------------------

namespace {
// A fake transport: records the last request and returns a canned response. Proves send()
// dispatches to an injected client and that status/headers/body round-trip through the facade
// unchanged.
class FakeClient : public cvc::net::HttpClient {
public:
  explicit FakeClient(HttpResponse canned) : canned_(std::move(canned)) {}
  HttpResponse send(const HttpRequest &req) override {
    last = req;
    ++calls;
    return canned_;
  }
  HttpRequest last;
  int calls = 0;

private:
  HttpResponse canned_;
};

// Restore the compiled default client on scope-out so one test's injection never leaks into
// another.
struct ClientGuard {
  ~ClientGuard() { cvc::net::set_http_client(nullptr); }
};
} // namespace

TEST(NetInjection, SendDispatchesToInjectedClientAndRoundTrips) {
  ClientGuard g;
  HttpResponse canned;
  canned.ok = true;
  canned.status = 200;
  canned.body = "canned-body";
  canned.headers = {"Content-Type: application/json"};
  canned.canonical_url = "http://example/final";
  auto *fake = new FakeClient(canned);
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));

  HttpRequest req;
  req.url = "http://example/x";
  req.method = "PUT";
  req.headers = {"Authorization: Bearer tok"};
  req.body = "sent";
  const HttpResponse r = cvc::net::send(req);

  EXPECT_EQ(fake->calls, 1);
  EXPECT_EQ(fake->last.url, "http://example/x");
  EXPECT_EQ(fake->last.method, "PUT");
  EXPECT_EQ(fake->last.body, "sent");
  ASSERT_EQ(fake->last.headers.size(), 1u);
  EXPECT_EQ(fake->last.headers[0], "Authorization: Bearer tok");
  // response round-trips untouched
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.status, 200);
  EXPECT_EQ(r.body, "canned-body");
  EXPECT_EQ(r.canonical_url, "http://example/final");
  ASSERT_EQ(r.headers.size(), 1u);
  EXPECT_EQ(r.headers[0], "Content-Type: application/json");
}

TEST(NetInjection, NullptrRestoresCompiledDefault) {
  {
    ClientGuard g;
    HttpResponse canned;
    canned.ok = true;
    canned.status = 222;
    cvc::net::set_http_client(std::make_unique<FakeClient>(canned));
    HttpRequest req;
    req.url = "http://example/y";
    EXPECT_EQ(cvc::net::send(req).status, 222); // the fake answers
  } // guard restores default
  // The default is back: without a live server a real backend errors and a null backend reports "no
  // backend" — either way it is NOT the fake's 222.
  HttpRequest req;
  req.url = "http://127.0.0.1:1/nope";
  EXPECT_NE(cvc::net::send(req).status, 222);
}

// The dispatcher normalizes every request so both backends see one canonical form
// (backend-agnostic, so an injected fake observes exactly what a real transport would).
TEST(NetNormalize, DefaultsMethodDropsGetHeadBodyAndRejectsInvalid) {
  ClientGuard g;
  HttpResponse canned;
  canned.ok = true;
  canned.status = 200;
  auto *fake = new FakeClient(canned);
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));

  { // an empty method becomes GET (the documented default), made explicit for both backends
    HttpRequest req;
    req.url = "http://x/";
    const HttpResponse r = cvc::net::send(req);
    EXPECT_TRUE(r.ok);
    EXPECT_EQ(fake->last.method, "GET");
  }
  { // a GET carrying a body: the body is stripped before dispatch (XHR would drop it anyway)
    HttpRequest req;
    req.url = "http://x/";
    req.method = "GET";
    req.body = "payload";
    cvc::net::send(req);
    EXPECT_TRUE(fake->last.body.empty()) << "GET body should be dropped";
  }
  { // HEAD (case-insensitive) likewise carries no body
    HttpRequest req;
    req.url = "http://x/";
    req.method = "head";
    req.body = "payload";
    cvc::net::send(req);
    EXPECT_TRUE(fake->last.body.empty()) << "HEAD body should be dropped";
  }
  { // a body-bearing verb keeps its body untouched
    HttpRequest req;
    req.url = "http://x/";
    req.method = "POST";
    req.body = "payload";
    cvc::net::send(req);
    EXPECT_EQ(fake->last.method, "POST");
    EXPECT_EQ(fake->last.body, "payload");
  }
  { // an out-of-token method is rejected at the facade — the backend is never reached
    const int before = fake->calls;
    HttpRequest req;
    req.url = "http://x/";
    req.method = "BAD VERB"; // contains a space
    const HttpResponse r = cvc::net::send(req);
    EXPECT_FALSE(r.ok);
    EXPECT_FALSE(r.error.empty());
    EXPECT_EQ(fake->calls, before) << "invalid method must short-circuit before dispatch";
  }
  { // a method too long for the wasm requestMethod buffer (32 bytes) is rejected uniformly
    HttpRequest req;
    req.url = "http://x/";
    req.method = std::string(40, 'A');
    EXPECT_FALSE(cvc::net::send(req).ok);
  }
}

// --- the compiled backend, end to end against localhost ----------------------------------------

namespace {
// Binds an OS-assigned port on 127.0.0.1, serves ONE connection a fixed 200 with two headers and a
// body, then closes. Enough to exercise the curl backend's status + response-header capture.
class OneShotServer {
public:
  OneShotServer()
      : acceptor_(io_,
                  boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0)) {
    port_ = acceptor_.local_endpoint().port();
    thread_ = std::thread([this] {
      boost::asio::ip::tcp::socket sock(io_);
      boost::system::error_code ec;
      acceptor_.accept(sock, ec);
      if (ec)
        return;
      char buf[4096];
      const std::size_t got = sock.read_some(boost::asio::buffer(buf), ec); // request line/headers
      const bool is_head = std::string(buf, ec ? 0 : got).rfind("HEAD ", 0) == 0;
      const std::string body = "hi-from-localhost";
      std::string reply = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\nX-Test: abc\r\n"
                          "Content-Length: " +
                          std::to_string(body.size()) + "\r\nConnection: close\r\n\r\n";
      // A standards-compliant HEAD reply carries the headers + Content-Length but NO body — exactly
      // the case that hangs a curl backend which set CUSTOMREQUEST=HEAD without CURLOPT_NOBODY.
      if (!is_head)
        reply += body;
      boost::asio::write(sock, boost::asio::buffer(reply), ec);
      sock.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    });
  }
  ~OneShotServer() {
    if (thread_.joinable())
      thread_.join();
  }
  unsigned short port() const { return port_; }

private:
  boost::asio::io_context io_;
  boost::asio::ip::tcp::acceptor acceptor_;
  unsigned short port_ = 0;
  std::thread thread_;
};

bool header_present(const std::vector<std::string> &lines, const std::string &needle) {
  for (const std::string &l : lines)
    if (l.find(needle) != std::string::npos)
      return true;
  return false;
}
} // namespace

TEST(NetCurlBackend, CapturesStatusAndResponseHeaders) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "built without an HTTP backend";
  OneShotServer server;
  HttpRequest req;
  req.url = "http://127.0.0.1:" + std::to_string(server.port()) + "/x";
  const HttpResponse r = cvc::net::send(req);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.status, 200);
  EXPECT_EQ(r.body, "hi-from-localhost");
  // the NEW capture: response headers surfaced as "Name: value" lines, status line dropped
  EXPECT_TRUE(header_present(r.headers, "X-Test: abc")) << "response headers not captured";
  EXPECT_TRUE(header_present(r.headers, "Content-Type: text/plain"));
  for (const std::string &l : r.headers)
    EXPECT_NE(l.rfind("HTTP/", 0), 0u) << "status line should be dropped: " << l;
}

TEST(NetCurlBackend, HeadRequestReturnsPromptlyWithNoBody) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "built without an HTTP backend";
  OneShotServer server;
  HttpRequest req;
  req.method = "HEAD";
  req.url = "http://127.0.0.1:" + std::to_string(server.port()) + "/x";
  req.timeout_secs = 5; // without CURLOPT_NOBODY curl would block on a body until this fires
  const auto start = std::chrono::steady_clock::now();
  const HttpResponse r = cvc::net::send(req);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.status, 200);
  EXPECT_TRUE(r.body.empty()) << "a HEAD response must not carry a body";
  EXPECT_TRUE(header_present(r.headers, "Content-Type: text/plain")) << "HEAD headers not captured";
  EXPECT_LT(elapsed, std::chrono::seconds(3)) << "HEAD hung — CURLOPT_NOBODY not set for HEAD";
}

TEST(NetCurlBackend, TransportFailureIsErrorNotThrow) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "built without an HTTP backend";
  HttpRequest req;
  req.url = "http://127.0.0.1:1/nope"; // nothing listens on port 1
  const HttpResponse r = cvc::net::send(req);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}
