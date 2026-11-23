// §13.9 app-wide HTTP cache tests. All offline: the cvc::net transport is swapped for a scripted
// fake (PR1's set_http_client seam), so no test touches the network. Each test registers the
// caching handler against a fresh cvc::app and drives cvc::ariadne::resolve()/store().

#include <algorithm>
#include <atomic>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <condition_variable>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/uri_http.h>
#include <cvc/ariadne/uri_http_cache.h>
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <cvc/net/net_policy.h>
#include <cvc/state/state.h>
#include <cvc/state/state_blob_store.h> // cvc::sha256_hex (to locate an entry node by key)
#include <deque>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <optional>
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
  // The fake is the app's own client: it goes with the app.
  ScriptedClient *install() {
    auto *f = new ScriptedClient();
    cvc::net::set_http_client(app, std::unique_ptr<cvc::net::HttpClient>(f));
    return f;
  }
  cvc::state &root() { return cvc::state::instance(app); }
  cvc::state *entry(const std::string &url) {
    return root().findDescendant("sys.net.http_cache.entries." + key_of(url));
  }
  // A key under sys.net.http_cache, "" when it has none.
  std::string key(const std::string &rel) {
    cvc::state *n = root().findDescendant("sys.net.http_cache." + rel);
    return n ? n->value() : std::string();
  }
  void set_key(const std::string &rel, const std::string &v) {
    root()("sys.net.http_cache." + rel).value(v);
  }
  std::string stat(const std::string &name) { return key("stats." + name); }
};

TEST_F(HttpCacheTest, FreshHitServesWithoutNetwork) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A"));
  auto r1 = cvc::ariadne::resolve(app, "http://ex/x");
  ASSERT_TRUE(r1.ok) << r1.error;
  EXPECT_EQ(r1.content, "A");
  auto r2 = cvc::ariadne::resolve(app, "http://ex/x");
  ASSERT_TRUE(r2.ok) << r2.error;
  EXPECT_EQ(r2.content, "A");
  EXPECT_EQ(fake->calls, 1) << "the second resolve must be served from the cache, no network";
}

TEST_F(HttpCacheTest, StaleRevalidation304ServesCachedBody) {
  auto *fake = install();
  // max-age=0 => always stale => the next resolve revalidates; the server answers 304.
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "A"));
  fake->responses.push_back(ok_resp(304, {}, ""));
  auto r1 = cvc::ariadne::resolve(app, "http://ex/x");
  ASSERT_TRUE(r1.ok) << r1.error;
  EXPECT_EQ(r1.content, "A");
  auto r2 = cvc::ariadne::resolve(app, "http://ex/x");
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
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A"); // miss -> store
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "B"); // stale -> 200 replaces
  cvc::ariadne::resolve(app, "http://ex/x");                         // stale -> revalidate with v2
  ASSERT_EQ(fake->requests.size(), 3u);
  EXPECT_TRUE(req_has_header(fake->requests[1], "If-None-Match: \"v1\""));
  EXPECT_TRUE(req_has_header(fake->requests[2], "If-None-Match: \"v2\""))
      << "the replaced entry must revalidate with the NEW ETag";
}

TEST_F(HttpCacheTest, NoStoreBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: no-store"}, "X"));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "X");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "X");
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
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A"); // stored fresh
  cvc::state *e = entry("http://ex/x");
  ASSERT_NE(e, nullptr);
  // Force the node past its retention; the next resolve's access-time sweep must drop it.
  e->expireAt(pt::microsec_clock::universal_time() - pt::seconds(1));
  auto r = cvc::ariadne::resolve(app, "http://ex/x");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "A2") << "the evicted entry is re-fetched, not served stale";
  EXPECT_EQ(fake->calls, 2);
}

TEST_F(HttpCacheTest, CredentialedRequestBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "S"));
  cvc::ariadne::set_http_options_provider(app, [](const std::string &, bool) {
    cvc::ariadne::HttpRequestOptions o;
    o.headers.push_back("Authorization: Bearer tok");
    return o;
  });
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "S");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "S");
  EXPECT_EQ(fake->calls, 2)
      << "a credentialed request must bypass the cache (no cross-principal reuse)";
  EXPECT_EQ(entry("http://ex/x"), nullptr) << "a credentialed response is never stored";
}

TEST_F(HttpCacheTest, UnsolicitedNotModifiedIsAnError) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(304, {}, "")); // a 304 to our unconditional (miss) GET
  auto r = cvc::ariadne::resolve(app, "http://ex/x");
  EXPECT_FALSE(r.ok) << "a 304 with no cached entry must be an error, not an empty-body success";
  EXPECT_NE(r.error.find("304"), std::string::npos);
  EXPECT_EQ(entry("http://ex/x"), nullptr);
}

TEST_F(HttpCacheTest, Revalidation304NoStoreEvicts) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "A"));
  fake->responses.push_back(ok_resp(304, {"Cache-Control: no-store"}, ""));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A3"));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A"); // stored (stale, ttl 0)
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content,
            "A"); // 304 no-store: served once, evicted
  EXPECT_EQ(entry("http://ex/x"), nullptr) << "a no-store revalidation must evict the entry";
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A3"); // gone -> re-fetched
  EXPECT_EQ(fake->calls, 3);
}

TEST_F(HttpCacheTest, UserinfoUrlBypassesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "U"));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://user:pass@ex/x").content, "U");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://user:pass@ex/x").content, "U");
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
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "A2")
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
      auto r = cvc::ariadne::resolve(app, "http://ex/" + std::to_string(i)); // distinct keys
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
  cvc::net::set_http_client(app, std::unique_ptr<cvc::net::HttpClient>(fake));

  // Start the leader and wait until it is inside send() — by then it has registered the in-flight
  // marker, so every follower launched now either waits on it or hits the (soon) fresh cache.
  std::string leader_body;
  std::thread leader([&] { leader_body = cvc::ariadne::resolve(app, "http://ex/x").content; });
  fake->wait_until_in_send();

  constexpr int kFollowers = 5;
  std::vector<std::thread> followers;
  std::vector<std::string> bodies(kFollowers);
  for (int i = 0; i < kFollowers; ++i)
    followers.emplace_back(
        [&, i] { bodies[i] = cvc::ariadne::resolve(app, "http://ex/x").content; });

  fake->release();
  leader.join();
  for (auto &t : followers)
    t.join();

  EXPECT_EQ(fake->calls.load(), 1) << "concurrent resolves must coalesce to one transfer";
  EXPECT_EQ(leader_body, "C");
  for (int i = 0; i < kFollowers; ++i)
    EXPECT_EQ(bodies[i], "C") << "follower " << i << " got the wrong body";
}

// The late arrival: a resolve that read the cache (a miss) before the leader stored, but reached
// single-flight after the leader left, saw neither the in-flight marker nor, from its old read, the
// entry, and became a second leader. The new leader now re-reads the entry. An instant transport
// and a start gate make that window common; every round is one key and must be one transfer.
TEST_F(HttpCacheTest, LateArrivalAfterTheLeaderLeftIsServedFromTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "L")); // repeats
  constexpr int kRounds = 1000, kThreads = 8;
  int bad_bodies = 0;
  for (int round = 0; round < kRounds; ++round) {
    const std::string url = "http://ex/late/" + std::to_string(round);
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::vector<std::string> bodies(kThreads);
    std::vector<std::thread> ts;
    for (int i = 0; i < kThreads; ++i)
      ts.emplace_back([&, i] {
        ready.fetch_add(1);
        while (!go.load())
          std::this_thread::yield();
        bodies[i] = cvc::ariadne::resolve(app, url).content;
      });
    while (ready.load() < kThreads)
      std::this_thread::yield();
    go.store(true);
    for (auto &t : ts)
      t.join();
    for (const std::string &b : bodies)
      bad_bodies += b == "L" ? 0 : 1;
  }
  EXPECT_EQ(fake->calls, kRounds) << "a resolve of an already-cached key made a second transfer";
  EXPECT_EQ(bad_bodies, 0);
}

// --- the policy keys, the budget, the stats and clearing -----------------------------------------

constexpr std::uint64_t kMiB = 1024u * 1024u;

TEST_F(HttpCacheTest, RegistrationSeedsThePolicyAndTheStats) {
  EXPECT_EQ(key("policy.enabled"), "1");
#ifdef __EMSCRIPTEN__
  EXPECT_EQ(key("policy.max_bytes"), std::to_string(48 * kMiB));
  EXPECT_EQ(key("policy.max_mib"), "48");
#else
  EXPECT_EQ(key("policy.max_bytes"), std::to_string(256 * kMiB));
  EXPECT_EQ(key("policy.max_mib"), "256");
  EXPECT_EQ(stat("used"), "0 MiB of 256 MiB");
#endif
  EXPECT_EQ(key("policy.max_entry_bytes"), std::to_string(64 * kMiB));
  EXPECT_EQ(key("policy.max_entry_mib"), "64");
  EXPECT_EQ(key("policy.freshness_secs"), "60");
  EXPECT_EQ(key("policy.retention_secs"), "3600");
  for (const char *s : {"entries", "bytes", "hits", "misses", "revalidated", "evicted"})
    EXPECT_EQ(stat(s), "0") << s;
  EXPECT_EQ(stat("fill"), "0.0000");
  EXPECT_NE(root().findDescendant("sys.net.http_cache.clear"), nullptr);
  EXPECT_EQ(key("clear"), "");
  EXPECT_TRUE(cvc::ariadne::has_http_cache(app));
  EXPECT_TRUE(cvc::ariadne::has_uri_async_handler(app, "http"))
      << "the cached handler has a non-blocking form, as the plain one does";
  EXPECT_TRUE(cvc::ariadne::has_uri_async_handler(app, "https"));
}

TEST(HttpCachePolicy, AKeySetBeforeRegistrationIsKept) {
  if (!cvc::ariadne::have_http_uri_handler())
    GTEST_SKIP() << "built without an HTTP backend";
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("sys.net.http_cache.policy.max_bytes").value(std::to_string(3 * kMiB));
  root("sys.net.http_cache.policy.freshness_secs").value(std::string("5"));
  cvc::ariadne::register_cached_http_uri_handler(app);
  EXPECT_EQ(root("sys.net.http_cache.policy.max_bytes").value(), std::to_string(3 * kMiB));
  EXPECT_EQ(root("sys.net.http_cache.policy.max_mib").value(), "3");
  EXPECT_EQ(root("sys.net.http_cache.policy.freshness_secs").value(), "5");
  EXPECT_EQ(root("sys.net.http_cache.policy.enabled").value(), "1") << "the unset ones are seeded";
}

TEST_F(HttpCacheTest, StatsCountHitsMissesAndRevalidations) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A"));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"v1\""}, "B"));
  fake->responses.push_back(ok_resp(304, {}, ""));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/a").content, "A"); // miss
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/a").content, "A"); // hit
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/b").content, "B"); // miss
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/b").content, "B"); // revalidated (304)
  EXPECT_EQ(fake->calls, 3);
  const cvc::ariadne::HttpCacheStats s = cvc::ariadne::http_cache_stats(app);
  EXPECT_EQ(s.hits, 1u);
  EXPECT_EQ(s.misses, 2u);
  EXPECT_EQ(s.revalidated, 1u);
  EXPECT_EQ(s.entries, 2u);
  EXPECT_EQ(s.evicted, 0u);
  EXPECT_GT(s.bytes, 2u) << "each entry is its body and its metadata";
  // ... mirrored into the state tree for a widget.
  EXPECT_EQ(stat("hits"), "1");
  EXPECT_EQ(stat("misses"), "2");
  EXPECT_EQ(stat("revalidated"), "1");
  EXPECT_EQ(stat("entries"), "2");
  EXPECT_EQ(stat("bytes"), std::to_string(s.bytes));
  ASSERT_NE(entry("http://ex/a"), nullptr);
  ASSERT_NE(entry("http://ex/b"), nullptr);
  EXPECT_EQ(std::stoull(entry("http://ex/a")->value()) + std::stoull(entry("http://ex/b")->value()),
            s.bytes)
      << "an entry node's value is its size";
}

TEST_F(HttpCacheTest, TheBudgetDropsTheLeastRecentlyUsed) {
  auto *fake = install();
  const std::string body(1000, 'x');
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, body)); // repeats
  for (const char *u : {"http://ex/1", "http://ex/2", "http://ex/3"})
    ASSERT_TRUE(cvc::ariadne::resolve(app, u).ok);
  ASSERT_NE(entry("http://ex/1"), nullptr);
  const std::uint64_t size = std::stoull(entry("http://ex/1")->value());
  EXPECT_GT(size, body.size());
  set_key("policy.max_bytes", std::to_string(3 * size + size / 2)); // room for three
  EXPECT_EQ(cvc::ariadne::http_cache_stats(app).entries, 3u) << "three fit: nothing goes";
  EXPECT_TRUE(cvc::ariadne::resolve(app, "http://ex/1").ok); // a hit: 1 is the newest now
  EXPECT_EQ(fake->calls, 3);
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/4").ok); // a fourth: one must go
  EXPECT_EQ(fake->calls, 4);
  EXPECT_EQ(entry("http://ex/2"), nullptr) << "the least recently used entry goes first";
  EXPECT_NE(entry("http://ex/1"), nullptr) << "a hit made it recent";
  EXPECT_NE(entry("http://ex/3"), nullptr);
  EXPECT_NE(entry("http://ex/4"), nullptr) << "the one just stored stays";
  const cvc::ariadne::HttpCacheStats s = cvc::ariadne::http_cache_stats(app);
  EXPECT_EQ(s.entries, 3u);
  EXPECT_EQ(s.evicted, 1u);
  EXPECT_EQ(s.bytes, std::stoull(entry("http://ex/1")->value()) +
                         std::stoull(entry("http://ex/3")->value()) +
                         std::stoull(entry("http://ex/4")->value()));
  EXPECT_EQ(stat("evicted"), "1");
  EXPECT_EQ(stat("entries"), "3");
  // The evicted one is fetched again.
  EXPECT_TRUE(cvc::ariadne::resolve(app, "http://ex/2").ok);
  EXPECT_EQ(fake->calls, 5);
}

TEST_F(HttpCacheTest, LoweringTheBudgetEvictsAtOnce) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, std::string(500, 'y')));
  for (const char *u : {"http://ex/1", "http://ex/2", "http://ex/3"})
    ASSERT_TRUE(cvc::ariadne::resolve(app, u).ok);
  const std::uint64_t size = std::stoull(entry("http://ex/3")->value());
  set_key("policy.max_bytes", std::to_string(size)); // room for one
  EXPECT_EQ(entry("http://ex/1"), nullptr);
  EXPECT_EQ(entry("http://ex/2"), nullptr);
  EXPECT_NE(entry("http://ex/3"), nullptr) << "the most recently used stays";
  EXPECT_EQ(stat("entries"), "1");
  EXPECT_EQ(stat("evicted"), "2");
  EXPECT_EQ(stat("fill"), "1.0000");
  EXPECT_EQ(key("policy.max_mib"), "0") << "the MiB form follows (rounded)";
}

TEST_F(HttpCacheTest, TheMibKeysAndTheByteKeysFollowEachOther) {
  set_key("policy.max_mib", "3");
  EXPECT_EQ(key("policy.max_bytes"), std::to_string(3 * kMiB));
  set_key("policy.max_bytes", std::to_string(5 * kMiB));
  EXPECT_EQ(key("policy.max_mib"), "5");
  set_key("policy.max_entry_mib", "2");
  EXPECT_EQ(key("policy.max_entry_bytes"), std::to_string(2 * kMiB));
  set_key("policy.max_entry_bytes", std::to_string(kMiB + 1)); // not whole MiB: kept as written
  EXPECT_EQ(key("policy.max_entry_mib"), "1");
  EXPECT_EQ(key("policy.max_entry_bytes"), std::to_string(kMiB + 1));
#ifndef __EMSCRIPTEN__
  EXPECT_EQ(stat("used"), "0 MiB of 5 MiB") << "the text follows the budget";
#endif
}

TEST_F(HttpCacheTest, ABodyPastMaxEntryBytesIsServedButNotKept) {
  auto *fake = install();
  set_key("policy.max_entry_bytes", "10");
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "0123456789ABCDEF"));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/big").content, "0123456789ABCDEF");
  EXPECT_EQ(entry("http://ex/big"), nullptr);
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/big").content, "0123456789ABCDEF");
  EXPECT_EQ(fake->calls, 2);
  fake->responses.front() = ok_resp(200, {"Cache-Control: max-age=300"}, "small");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/small").content, "small");
  EXPECT_NE(entry("http://ex/small"), nullptr) << "one within the limit is kept";
}

TEST_F(HttpCacheTest, FreshnessSecsAppliesWithoutARestart) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {}, "N")); // no max-age, no validator; repeats
  set_key("policy.freshness_secs", "0");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "N");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "N");
  EXPECT_EQ(fake->calls, 2) << "fresh for 0 s: every read asks";
  set_key("policy.freshness_secs", "300");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "N"); // stored fresh for 300 s
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/x").content, "N"); // served
  EXPECT_EQ(fake->calls, 3);
}

TEST_F(HttpCacheTest, TurnedOffEveryReadIsARequestAndNothingIsKept) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A"));
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/a").ok);
  set_key("policy.enabled", "0");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/a").content, "A");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/new").content, "A");
  EXPECT_EQ(fake->calls, 3) << "off: each read is a request, fresh entry or not";
  EXPECT_EQ(entry("http://ex/new"), nullptr) << "off: nothing is kept";
  EXPECT_NE(entry("http://ex/a"), nullptr) << "what was kept stays";
  set_key("policy.enabled", "1");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/a").content, "A");
  EXPECT_EQ(fake->calls, 3) << "on again: the kept entry is served";
}

TEST_F(HttpCacheTest, WritingTheClearKeyEmptiesTheCache) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "C"));
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/1").ok);
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/2").ok);
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/2").ok); // a hit
  ASSERT_EQ(stat("entries"), "2");
  set_key("clear", "1");
  EXPECT_EQ(entry("http://ex/1"), nullptr);
  EXPECT_EQ(entry("http://ex/2"), nullptr);
  EXPECT_EQ(stat("entries"), "0");
  EXPECT_EQ(stat("bytes"), "0");
  EXPECT_EQ(key("clear"), "") << "the key goes back to empty, so the next write clears again";
  EXPECT_EQ(stat("hits"), "1") << "the counters keep counting";
  EXPECT_EQ(stat("evicted"), "0") << "a clear is not an eviction";
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/1").ok);
  EXPECT_EQ(fake->calls, 3) << "cleared: fetched again";
  set_key("clear", "yes");
  EXPECT_EQ(entry("http://ex/1"), nullptr) << "any value clears";
}

TEST_F(HttpCacheTest, ClearHttpCacheEmptiesIt) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "C"));
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/1").ok);
  cvc::ariadne::clear_http_cache(app);
  EXPECT_EQ(entry("http://ex/1"), nullptr);
  EXPECT_EQ(cvc::ariadne::http_cache_stats(app).entries, 0u);
  EXPECT_EQ(cvc::ariadne::http_cache_stats(app).bytes, 0u);
  EXPECT_EQ(stat("entries"), "0");
}

TEST_F(HttpCacheTest, RetentionExpiryCountsAsEvicted) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "A")); // repeats
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/x").ok);
  entry("http://ex/x")->expireAt(pt::microsec_clock::universal_time() - pt::seconds(1));
  ASSERT_TRUE(cvc::ariadne::resolve(app, "http://ex/y").ok); // the access-time sweep drops x
  EXPECT_EQ(entry("http://ex/x"), nullptr);
  EXPECT_EQ(stat("evicted"), "1");
  EXPECT_EQ(stat("entries"), "1");
}

// --- the NetPolicy ---------------------------------------------------------------------------

std::shared_ptr<const cvc::net::NetPolicy> allow_only(const std::string &origin) {
  auto p = std::make_shared<cvc::net::NetPolicy>();
  std::string why;
  const std::optional<cvc::net::Origin> o = cvc::net::parse_origin_spec(origin, &why);
  EXPECT_TRUE(o.has_value()) << why;
  if (o)
    p->allow(*o);
  return p;
}

TEST_F(HttpCacheTest, APolicyRefusesWithNoRequest) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "OK"));
  cvc::ariadne::register_cached_http_uri_handler(app, allow_only("http://ex.test"));
  const cvc::ariadne::UriResult denied = cvc::ariadne::resolve(app, "http://evil.test/x");
  EXPECT_FALSE(denied.ok);
  EXPECT_NE(denied.error.find("not an origin this host allows"), std::string::npos) << denied.error;
  EXPECT_EQ(fake->calls, 0) << "a refused URL is never requested";
  EXPECT_TRUE(cvc::ariadne::resolve(app, "http://ex.test/x").ok);
  EXPECT_EQ(fake->calls, 1);
  // The store side is checked too.
  EXPECT_FALSE(cvc::ariadne::store(app, "http://evil.test/y", "body").ok);
  EXPECT_EQ(fake->calls, 1);
}

TEST_F(HttpCacheTest, AFreshEntryIsNotServedPastAStricterPolicy) {
  auto *fake = install();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "KEPT"));
  ASSERT_EQ(cvc::ariadne::resolve(app, "http://ex.test/x").content, "KEPT"); // unchecked
  ASSERT_NE(entry("http://ex.test/x"), nullptr);
  cvc::ariadne::register_cached_http_uri_handler(app, allow_only("http://other.test"));
  const cvc::ariadne::UriResult r = cvc::ariadne::resolve(app, "http://ex.test/x");
  EXPECT_FALSE(r.ok) << "the kept body must not be served to a URL the policy refuses";
  EXPECT_EQ(fake->calls, 1);
  // The host-side GET (pictures, media) is held to the same policy.
  const std::optional<cvc::net::HttpResponse> g =
      cvc::ariadne::http_cache_get(app, "http://ex.test/x");
  ASSERT_TRUE(g.has_value());
  EXPECT_FALSE(g->ok);
  EXPECT_NE(g->error.find("not an origin this host allows"), std::string::npos) << g->error;
  EXPECT_EQ(fake->calls, 1);
}

TEST(HttpCacheNetHost, EnableNetRegistersTheCacheAndDisableNetRemovesIt) {
  if (!cvc::ariadne::have_http_uri_handler())
    GTEST_SKIP() << "built without an HTTP backend";
  cvc::app app;
  auto *fake = new ScriptedClient();
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=300"}, "N"));
  cvc::net::set_http_client(app, std::unique_ptr<cvc::net::HttpClient>(fake));
  cvc::ariadne::NetHostConfig cfg;
  cfg.enabled = true;
  cfg.allow = {"http://ex.test"};
  ASSERT_EQ(cvc::ariadne::enable_net(app, cfg), "");
  EXPECT_TRUE(cvc::ariadne::has_http_cache(app));
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex.test/a").content, "N");
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex.test/a").content, "N");
  EXPECT_EQ(fake->calls, 1) << "enable_net's handler is the cache";
  EXPECT_FALSE(cvc::ariadne::resolve(app, "http://evil.test/a").ok) << "on its allowlist";
  EXPECT_EQ(fake->calls, 1);
  cvc::ariadne::disable_net(app);
  EXPECT_FALSE(cvc::ariadne::has_http_cache(app));
  EXPECT_FALSE(cvc::ariadne::has_uri_handler(app, "http"));
  EXPECT_FALSE(cvc::ariadne::has_uri_handler(app, "https"));
  EXPECT_FALSE(cvc::ariadne::has_uri_async_handler(app, "http"));
  EXPECT_FALSE(cvc::ariadne::has_uri_store_handler(app, "http"));
  EXPECT_FALSE(cvc::ariadne::http_cache_get(app, "http://ex.test/a").has_value());

  cfg.http_cache = false; // the plain handler: every read is a request
  ASSERT_EQ(cvc::ariadne::enable_net(app, cfg), "");
  EXPECT_FALSE(cvc::ariadne::has_http_cache(app));
  EXPECT_TRUE(cvc::ariadne::resolve(app, "http://ex.test/b").ok);
  EXPECT_TRUE(cvc::ariadne::resolve(app, "http://ex.test/b").ok);
  EXPECT_EQ(fake->calls, 3);
}

// --- the host-side GET (pictures, media) -------------------------------------------------------

TEST_F(HttpCacheTest, HttpCacheGetServesAKeptEntryAsA200) {
  auto *fake = install();
  fake->responses.push_back(
      ok_resp(200, {"Content-Type: image/png", "Cache-Control: max-age=300"}, "PNG"));
  for (int i = 0; i < 2; ++i) {
    const std::optional<cvc::net::HttpResponse> r =
        cvc::ariadne::http_cache_get(app, "http://ex/p.png");
    ASSERT_TRUE(r.has_value());
    EXPECT_TRUE(r->ok) << r->error;
    EXPECT_EQ(r->status, 200);
    EXPECT_EQ(r->body, "PNG");
    EXPECT_EQ(cvc::net::response_header(r->headers, "Content-Type"), "image/png");
  }
  EXPECT_EQ(fake->calls, 1);
  EXPECT_EQ(cvc::ariadne::resolve(app, "http://ex/p.png").content, "PNG")
      << "a URI read and a host-side GET share the entry";
  EXPECT_EQ(fake->calls, 1);
  // A URL with userinfo is never kept.
  ASSERT_TRUE(cvc::ariadne::http_cache_get(app, "http://u:p@ex/p.png").has_value());
  ASSERT_TRUE(cvc::ariadne::http_cache_get(app, "http://u:p@ex/p.png").has_value());
  EXPECT_EQ(fake->calls, 3);
  cvc::ariadne::unregister_cached_http_uri_handler(app);
  EXPECT_FALSE(cvc::ariadne::http_cache_get(app, "http://ex/p.png").has_value())
      << "no cache: the caller makes its request another way";
}

// --- the non-blocking form (a browser build without threads) -----------------------------------

// A transport that starts requests and answers when the test says, on the test's thread, as the
// browser's event loop would.
class DeferredClient : public cvc::net::HttpClient {
public:
  struct Transfer : cvc::net::HttpTransfer {
    bool aborted = false;
    void abort() override { aborted = true; }
  };
  struct Pending {
    HttpRequest req;
    cvc::net::HttpDone done;
    std::shared_ptr<Transfer> transfer;
  };
  std::vector<Pending> pending;
  std::deque<HttpResponse> responses;
  int sends = 0;
  bool starts_async() const override { return true; }
  std::shared_ptr<cvc::net::HttpTransfer> start(const HttpRequest &req,
                                                cvc::net::HttpDone done) override {
    auto t = std::make_shared<Transfer>();
    pending.push_back({req, std::move(done), t});
    return t;
  }
  HttpResponse send(const HttpRequest &) override {
    ++sends;
    return HttpResponse{};
  }
  bool answer() {
    if (pending.empty() || responses.empty())
      return false;
    Pending p = std::move(pending.front());
    pending.erase(pending.begin());
    HttpResponse r = responses.front();
    responses.pop_front();
    if (!p.transfer->aborted)
      p.done(r);
    return true;
  }
};

TEST_F(HttpCacheTest, TheNonBlockingFormStartsTheRequestAndKeepsTheReply) {
  auto *fake = new DeferredClient();
  cvc::net::set_http_client(app, std::unique_ptr<cvc::net::HttpClient>(fake));
  fake->responses.push_back(ok_resp(200, {"Cache-Control: max-age=0", "ETag: \"d1\""}, "D"));
  fake->responses.push_back(ok_resp(304, {"Cache-Control: max-age=300"}, ""));
  std::vector<cvc::ariadne::UriResult> got;
  const auto done = [&](cvc::ariadne::UriResult r) { got.push_back(std::move(r)); };

  cvc::ariadne::resolve_async(app, "http://ex/d", "", done);
  ASSERT_EQ(fake->pending.size(), 1u) << "the request is started, not sent";
  EXPECT_TRUE(got.empty());
  ASSERT_TRUE(fake->answer());
  ASSERT_EQ(got.size(), 1u);
  EXPECT_TRUE(got[0].ok) << got[0].error;
  EXPECT_EQ(got[0].content, "D");
  EXPECT_NE(entry("http://ex/d"), nullptr) << "the reply is kept";

  cvc::ariadne::resolve_async(app, "http://ex/d", "", done); // stale: revalidated
  ASSERT_EQ(fake->pending.size(), 1u);
  EXPECT_NE(std::find(fake->pending[0].req.headers.begin(), fake->pending[0].req.headers.end(),
                      "If-None-Match: \"d1\""),
            fake->pending[0].req.headers.end());
  ASSERT_TRUE(fake->answer());
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(got[1].content, "D") << "a 304 serves the kept body";

  cvc::ariadne::resolve_async(app, "http://ex/d", "", done); // fresh now: answered at once
  EXPECT_TRUE(fake->pending.empty());
  ASSERT_EQ(got.size(), 3u);
  EXPECT_EQ(got[2].content, "D");
  EXPECT_EQ(fake->sends, 0) << "nothing blocked";
  EXPECT_EQ(stat("misses"), "1");
  EXPECT_EQ(stat("revalidated"), "1");
  EXPECT_EQ(stat("hits"), "1");

  // The host-side GET's non-blocking form shares it.
  std::vector<cvc::net::HttpResponse> replies;
  ASSERT_TRUE(cvc::ariadne::http_cache_start(
      app, "http://ex/d", [&](cvc::net::HttpResponse r) { replies.push_back(std::move(r)); }));
  ASSERT_EQ(replies.size(), 1u);
  EXPECT_EQ(replies[0].body, "D");
  EXPECT_TRUE(fake->pending.empty());
}

TEST_F(HttpCacheTest, TheNonBlockingFormIsHeldToThePolicy) {
  auto *fake = new DeferredClient();
  cvc::net::set_http_client(app, std::unique_ptr<cvc::net::HttpClient>(fake));
  cvc::ariadne::register_cached_http_uri_handler(app, allow_only("http://ex.test"));
  std::vector<cvc::ariadne::UriResult> got;
  cvc::ariadne::resolve_async(app, "http://evil.test/x", "",
                              [&](cvc::ariadne::UriResult r) { got.push_back(std::move(r)); });
  ASSERT_EQ(got.size(), 1u) << "refused before it starts";
  EXPECT_FALSE(got[0].ok);
  EXPECT_NE(got[0].error.find("not an origin this host allows"), std::string::npos) << got[0].error;
  EXPECT_TRUE(fake->pending.empty());
}

} // namespace
