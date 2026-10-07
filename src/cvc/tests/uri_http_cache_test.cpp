// §13.9 app-wide HTTP cache tests. All offline: the cvc::net transport is swapped for a scripted
// fake (PR1's set_http_client seam), so no test touches the network. Each test registers the
// caching handler against a fresh cvc::app and drives cvc::ariadne::resolve()/store().

#include <atomic>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <condition_variable>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/ariadne/uri_http_cache.h>
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // cvc::sha256_hex (to locate an entry node by key)
#include <deque>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace pt = boost::posix_time;
using cvc::net::HttpRequest;
using cvc::net::HttpResponse;

HttpResponse ok_resp(long status, std::vector<std::string> headers, std::string body,
                     std::string url = "http://ex/x") {
  HttpResponse r;
  r.ok = true;
  r.status = status;
  r.headers = std::move(headers);
  r.body = std::move(body);
  r.canonical_url = std::move(url);
  return r;
}

// A scripted transport: pops one canned response per call (the last one repeats), and records every
// request so a test can assert the conditional headers that were sent.
class ScriptedClient : public cvc::net::HttpClient {
public:
  std::deque<HttpResponse> responses;
  std::vector<HttpRequest> requests;
  int calls = 0;
  HttpResponse send(const HttpRequest &req) override {
    // ConcurrentDistinctKeysAreServedSafely drives DISTINCT keys, which do not coalesce in
    // single-flight, so every resolve becomes a leader and calls send() concurrently (the cache
    // correctly releases cache_mutex across the transport). A real HttpClient is thread-safe or
    // per-call independent; this fake mutates shared std::deque/std::vector/int, so it must lock or
    // it races — concurrent push_back reallocates the vector mid-access → UB (a Windows-CI
    // SEGFAULT).
    std::lock_guard<std::mutex> lk(m_);
    requests.push_back(req);
    ++calls;
    if (responses.empty()) {
      HttpResponse r;
      r.ok = false;
      r.error = "scripted: no more responses";
      return r;
    }
    HttpResponse r = responses.front();
    if (responses.size() > 1)
      responses.pop_front(); // keep the last so a surprise extra call still answers
    return r;
  }

private:
  std::mutex m_;
};

bool req_has_header(const HttpRequest &r, const std::string &needle) {
  for (const std::string &h : r.headers)
    if (h.find(needle) != std::string::npos)
      return true;
  return false;
}

// The cache key derivation, mirrored for tests that need to locate the entry node. The test URLs
// are already normalized (lowercase, no default port, no fragment), so normalization is identity
// here.
std::string key_of(const std::string &normalized_url) {
  return cvc::sha256_hex(reinterpret_cast<const unsigned char *>(normalized_url.data()),
                         normalized_url.size());
}

class HttpCacheTest : public ::testing::Test {
protected:
  cvc::app app;
  void SetUp() override {
    if (!cvc::ariadne::have_http_uri_handler())
      GTEST_SKIP() << "built without an HTTP backend";
    cvc::ariadne::register_cached_http_uri_handler(app);
  }
  void TearDown() override {
    cvc::ariadne::unregister_cached_http_uri_handler();
    cvc::net::set_http_client(nullptr);
    cvc::ariadne::set_http_options_provider(nullptr);
  }
  ScriptedClient *install() {
    auto *f = new ScriptedClient();
    cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(f));
    return f;
  }
  cvc::state &root() { return cvc::state::instance(app); }
  cvc::state *entry(const std::string &url) {
    return root().findDescendant("sys.net.http_cache.entries." + key_of(url));
  }
};

TEST_F(HttpCacheTest, FreshHitServesWithoutNetwork) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A"));
  auto r1 = cvc::ariadne::resolve("http://ex/x");
  ASSERT_TRUE(r1.ok) << r1.error;
  EXPECT_EQ(r1.content, "A");
  auto r2 = cvc::ariadne::resolve("http://ex/x");
  ASSERT_TRUE(r2.ok) << r2.error;
  EXPECT_EQ(r2.content, "A");
  EXPECT_EQ(fake->calls, 1) << "the second resolve must be served from the cache, no network";
}

TEST_F(HttpCacheTest, StaleRevalidation304ServesCachedBody) {
  auto *fake = install();
  // max-age=0 => always stale => the next resolve revalidates; the server answers 304.
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "A"));
  fake->responses.push_back(ok_resp(304, {}, ""));
  auto r1 = cvc::ariadne::resolve("http://ex/x");
  ASSERT_TRUE(r1.ok) << r1.error;
  EXPECT_EQ(r1.content, "A");
  auto r2 = cvc::ariadne::resolve("http://ex/x");
  ASSERT_TRUE(r2.ok) << r2.error;
  EXPECT_EQ(r2.content, "A") << "a 304 serves the cached body";
  EXPECT_EQ(fake->calls, 2);
  ASSERT_EQ(fake->requests.size(), 2u);
  EXPECT_TRUE(req_has_header(fake->requests[1], "If-None-Match: \"v1\""))
      << "the revalidation must send the stored ETag";
}

TEST_F(HttpCacheTest, StaleRevalidation200ReplacesBody) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "A"));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v2\""}, "B"));
  fake->responses.push_back(ok_resp(304, {}, ""));
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A"); // miss -> store
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "B"); // stale -> 200 replaces
  cvc::ariadne::resolve("http://ex/x");                         // stale -> revalidate with v2
  ASSERT_EQ(fake->requests.size(), 3u);
  EXPECT_TRUE(req_has_header(fake->requests[1], "If-None-Match: \"v1\""));
  EXPECT_TRUE(req_has_header(fake->requests[2], "If-None-Match: \"v2\""))
      << "the replaced entry must revalidate with the NEW ETag";
}

TEST_F(HttpCacheTest, NoStoreBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: no-store"}, "X"));
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "X");
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "X");
  EXPECT_EQ(fake->calls, 2) << "no-store must not cache, so every resolve hits the network";
  ASSERT_EQ(fake->requests.size(), 2u);
  EXPECT_FALSE(req_has_header(fake->requests[1], "If-None-Match"))
      << "a no-store re-fetch is a fresh miss, not a revalidation";
  EXPECT_EQ(entry("http://ex/x"), nullptr) << "nothing was stored";
}

TEST_F(HttpCacheTest, RetentionExpiryEvictsAndReFetches) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A"));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A2"));
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A"); // stored fresh
  cvc::state *e = entry("http://ex/x");
  ASSERT_NE(e, nullptr);
  // Force the node past its retention; the next resolve's access-time sweep must drop it.
  e->expireAt(pt::microsec_clock::universal_time() - pt::seconds(1));
  auto r = cvc::ariadne::resolve("http://ex/x");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "A2") << "the evicted entry is re-fetched, not served stale";
  EXPECT_EQ(fake->calls, 2);
}

TEST_F(HttpCacheTest, CredentialedRequestBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "S"));
  cvc::ariadne::set_http_options_provider([](const std::string &, bool) {
    cvc::ariadne::HttpRequestOptions o;
    o.headers.push_back("Authorization: Bearer tok");
    return o;
  });
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "S");
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "S");
  EXPECT_EQ(fake->calls, 2)
      << "a credentialed request must bypass the cache (no cross-principal reuse)";
  EXPECT_EQ(entry("http://ex/x"), nullptr) << "a credentialed response is never stored";
}

TEST_F(HttpCacheTest, UnsolicitedNotModifiedIsAnError) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(304, {}, "")); // a 304 to our unconditional (miss) GET
  auto r = cvc::ariadne::resolve("http://ex/x");
  EXPECT_FALSE(r.ok) << "a 304 with no cached entry must be an error, not an empty-body success";
  EXPECT_NE(r.error.find("304"), std::string::npos);
  EXPECT_EQ(entry("http://ex/x"), nullptr);
}

TEST_F(HttpCacheTest, Revalidation304NoStoreEvicts) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "A"));
  fake->responses.push_back(ok_resp(304, {"Cache-Control: no-store"}, ""));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A3"));
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A"); // stored (stale, ttl 0)
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content,
            "A"); // 304 no-store: served once, evicted
  EXPECT_EQ(entry("http://ex/x"), nullptr) << "a no-store revalidation must evict the entry";
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A3"); // gone -> re-fetched
  EXPECT_EQ(fake->calls, 3);
}

TEST_F(HttpCacheTest, UserinfoUrlBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "U"));
  EXPECT_EQ(cvc::ariadne::resolve("http://user:pass@ex/x").content, "U");
  EXPECT_EQ(cvc::ariadne::resolve("http://user:pass@ex/x").content, "U");
  EXPECT_EQ(fake->calls, 2) << "a userinfo URL is credential-bearing and must bypass the cache";
  ASSERT_EQ(fake->requests.size(), 2u);
  EXPECT_FALSE(req_has_header(fake->requests[1], "If-None-Match")) << "bypass is a fresh miss";
}

TEST_F(HttpCacheTest, AgeAnchorsFreshnessBeforeReceipt) {
  auto *fake = install();
  // max-age=100 but the response is already Age=100s old (a CDN hit): the fresh window is 0, so the
  // next resolve must revalidate rather than serve it as fresh.
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=100", "Age: 100"}, "A"));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=100", "Age: 0"}, "A2"));
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A");
  EXPECT_EQ(cvc::ariadne::resolve("http://ex/x").content, "A2")
      << "an Age==max-age response is already stale on arrival, so it must re-fetch";
  EXPECT_EQ(fake->calls, 2);
}

TEST_F(HttpCacheTest, ConcurrentDistinctKeysAreServedSafely) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "OK")); // repeats
  constexpr int kN = 8;
  std::vector<std::thread> ts;
  std::vector<std::string> bodies(kN);
  std::vector<char> oks(kN, 0);
  for (int i = 0; i < kN; ++i)
    ts.emplace_back([&, i] {
      auto r = cvc::ariadne::resolve("http://ex/" + std::to_string(i)); // distinct keys
      oks[i] = r.ok ? 1 : 0;
      bodies[i] = r.content;
    });
  for (auto &t : ts)
    t.join();
  // Distinct keys don't coalesce, so each is one transfer; the point is no crash/deadlock/UAF under
  // concurrent stores + sweeps of the shared entries subtree (cache_mutex serializes the tree
  // work).
  for (int i = 0; i < kN; ++i) {
    EXPECT_EQ(oks[i], 1) << "resolve " << i << " failed";
    EXPECT_EQ(bodies[i], "OK");
  }
  EXPECT_EQ(fake->calls, kN);
}

// A transport that blocks in send() until released, so a test can drive concurrent resolves.
class LatchClient : public cvc::net::HttpClient {
public:
  explicit LatchClient(HttpResponse r) : resp_(std::move(r)) {}
  std::atomic<int> calls{0};
  HttpResponse send(const HttpRequest &) override {
    {
      std::unique_lock<std::mutex> lk(m_);
      in_send_ = true;
      cv_.notify_all();
      cv_.wait(lk, [&] { return release_; });
    }
    ++calls;
    return resp_;
  }
  void wait_until_in_send() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&] { return in_send_; });
  }
  void release() {
    {
      std::lock_guard<std::mutex> lk(m_);
      release_ = true;
    }
    cv_.notify_all();
  }

private:
  std::mutex m_;
  std::condition_variable cv_;
  bool in_send_ = false;
  bool release_ = false;
  HttpResponse resp_;
};

TEST_F(HttpCacheTest, SingleFlightCoalescesConcurrentResolves) {
  auto *fake = new LatchClient(ok_resp(200, {"Cache-Control: max-age=300"}, "C"));
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(fake));

  // Start the leader and wait until it is inside send() — by then it has registered the in-flight
  // marker, so every follower launched now either waits on it or hits the (soon) fresh cache.
  std::string leader_body;
  std::thread leader([&] { leader_body = cvc::ariadne::resolve("http://ex/x").content; });
  fake->wait_until_in_send();

  constexpr int kFollowers = 5;
  std::vector<std::thread> followers;
  std::vector<std::string> bodies(kFollowers);
  for (int i = 0; i < kFollowers; ++i)
    followers.emplace_back([&, i] { bodies[i] = cvc::ariadne::resolve("http://ex/x").content; });

  fake->release();
  leader.join();
  for (auto &t : followers)
    t.join();

  EXPECT_EQ(fake->calls.load(), 1) << "concurrent resolves must coalesce to one transfer";
  EXPECT_EQ(leader_body, "C");
  for (int i = 0; i < kFollowers; ++i)
    EXPECT_EQ(bodies[i], "C") << "follower " << i << " got the wrong body";
}

} // namespace
