// Unit tests for cvc::thread_pool — the persistent-worker fork-join pool.
// Concurrency correctness: every index runs exactly once, results are complete
// and race-free, nested fan-outs don't deadlock, concurrent orchestrator
// threads all complete, and workers are reused across many calls (the whole
// reason the pool exists).

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/thread_pool.h>
#include <gtest/gtest.h>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

using cvc::thread_pool;

TEST(ThreadPool, VisitsEachIndexExactlyOnce) {
  thread_pool pool;
  for (int n : {0, 1, 2, 3, 7, 19, 100, 1000, 100000}) {
    std::vector<int> hits(n > 0 ? n : 1, 0);
    pool.parallel_for(n, [&](int i) { hits[i]++; });
    for (int i = 0; i < n; ++i)
      ASSERT_EQ(hits[i], 1) << "n=" << n << " i=" << i;
  }
}

TEST(ThreadPool, ParallelSumMatchesClosedForm) {
  thread_pool pool;
  const int n = 2000000;
  std::atomic<long long> sum{0};
  pool.parallel_for(n, [&](int i) { sum.fetch_add(i, std::memory_order_relaxed); });
  EXPECT_EQ(sum.load(), static_cast<long long>(n) * (n - 1) / 2);
}

TEST(ThreadPool, NestedFanoutRunsInlineWithoutDeadlock) {
  thread_pool pool;
  std::atomic<int> total{0};
  pool.parallel_for(64, [&](int) {
    // A parallel_for on the SAME pool from within a task must not deadlock
    // waiting on workers that are busy with the outer job — it runs inline.
    pool.parallel_for(64, [&](int) { total.fetch_add(1, std::memory_order_relaxed); });
  });
  EXPECT_EQ(total.load(), 64 * 64);
}

TEST(ThreadPool, WorkersReusedAcrossManyFanouts) {
  thread_pool pool;
  std::atomic<long long> acc{0};
  for (int r = 0; r < 5000; ++r)
    pool.parallel_for(500, [&](int) { acc.fetch_add(1, std::memory_order_relaxed); });
  EXPECT_EQ(acc.load(), 5000LL * 500);
}

TEST(ThreadPool, SmallPoolAndMaxParStillCoverEveryIndex) {
  thread_pool one(1);
  std::vector<int> hits(200, 0);
  one.parallel_for(200, [&](int i) { hits[i]++; });
  for (int h : hits)
    ASSERT_EQ(h, 1);

  thread_pool pool;
  std::vector<int> hits2(300, 0);
  pool.parallel_for(300, [&](int i) { hits2[i]++; }, /*max_par=*/2);
  for (int h : hits2)
    ASSERT_EQ(h, 1);
}

TEST(ThreadPool, RunsWorkOnMultipleThreads) {
  if (std::thread::hardware_concurrency() <= 1)
    GTEST_SKIP() << "single hardware thread";
  thread_pool pool;
  // One task per participant, each of which BLOCKS briefly. Because the calling
  // thread is stuck inside its own task while it sleeps, it cannot race ahead and
  // drain the queue alone (the bug an earlier version of this test had with
  // trivial work) — the workers must pick up the rest, so more than one distinct
  // thread provably runs the work.
  const int tasks = static_cast<int>(pool.concurrency());
  std::mutex m;
  std::unordered_set<std::size_t> ids;
  pool.parallel_for(tasks, [&](int) {
    const std::size_t id = std::hash<std::thread::id>{}(std::this_thread::get_id());
    {
      std::lock_guard<std::mutex> lk(m);
      ids.insert(id);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  });
  EXPECT_GT(ids.size(), 1u) << "work did not spread beyond the calling thread";
}

TEST(ThreadPool, ConcurrentOrchestratorsAllComplete) {
  // Regression: parallel_for posts into a single shared job slot. Before the
  // orchestrator lock, two threads posting concurrently could overwrite each
  // other's _job/_epoch before a slow-to-wake worker enlisted in the first job
  // adopted it — its `remaining` count then never drained and the first caller
  // blocked in _done.wait forever (and a finishing caller's `_job = nullptr`
  // could likewise stomp a freshly posted job, stranding THAT caller). Four
  // orchestrators looping small fan-outs over a 2-worker pool hung within a few
  // iterations. Sizes cycle through 1..16 so inline (n == 1) and posted
  // fan-outs interleave, exercising the post/drain/clear transitions.
  thread_pool pool(2);
  constexpr int kOrchestrators = 4;
  constexpr int kIters = 250;
  std::atomic<long long> total{0};
  std::atomic<int> finished{0};
  std::atomic<bool> exact{true};

  std::vector<std::thread> orchestrators;
  orchestrators.reserve(kOrchestrators);
  for (int t = 0; t < kOrchestrators; ++t)
    orchestrators.emplace_back([&, t] {
      for (int r = 0; r < kIters; ++r) {
        const int n = 1 + ((t + r) % 16);
        std::vector<int> hits(n, 0);
        pool.parallel_for(n, [&](int i) {
          hits[i]++;
          total.fetch_add(1, std::memory_order_relaxed);
        });
        for (int i = 0; i < n; ++i)
          if (hits[i] != 1)
            exact.store(false, std::memory_order_relaxed);
      }
      finished.fetch_add(1, std::memory_order_release);
    });

  // Watchdog: pre-fix, stranded orchestrators block inside parallel_for with no
  // way to unwind, so joining them would hang the test runner (and gtest can't
  // fail a test whose threads never return). Poll with a generous deadline and
  // hard-exit with a diagnostic instead, so CI reports a failure, not a timeout.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (finished.load(std::memory_order_acquire) < kOrchestrators) {
    if (std::chrono::steady_clock::now() >= deadline) {
      std::fprintf(stderr,
                   "ThreadPool.ConcurrentOrchestratorsAllComplete watchdog: "
                   "%d/%d orchestrators finished after 60s (parallel_for hang), "
                   "total tasks run: %lld\n",
                   finished.load(), kOrchestrators, total.load());
      std::fflush(stderr);
      std::_Exit(EXIT_FAILURE);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (auto &th : orchestrators)
    th.join();

  EXPECT_TRUE(exact.load()) << "some index ran zero times or more than once";
  long long expected = 0;
  for (int t = 0; t < kOrchestrators; ++t)
    for (int r = 0; r < kIters; ++r)
      expected += 1 + ((t + r) % 16);
  EXPECT_EQ(total.load(), expected);
}

TEST(ThreadPool, EmptyRangeIsANoop) {
  thread_pool pool;
  int calls = 0;
  pool.parallel_for(0, [&](int) { ++calls; });
  pool.parallel_for(-5, [&](int) { ++calls; });
  EXPECT_EQ(calls, 0);
}
