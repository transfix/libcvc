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
#include <stdexcept>
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

// ─── Exception safety ────────────────────────────────────────────────────────
// A task that throws must NOT escape a worker thread (that would std::terminate) nor
// unwind the caller past still-running workers (use-after-free of the stack job). The
// fan-out joins fully, the first exception is re-thrown from parallel_for on the
// caller's thread, and the pool stays usable afterward.

TEST(ThreadPool, ThrowingTaskPropagatesToCallerAndPoolStaysUsable) {
  thread_pool pool;
  if (pool.concurrency() <= 1)
    GTEST_SKIP() << "need workers to exercise the fan-out throw path";
  // Each task blocks briefly so the caller cannot drain the whole range alone — the
  // throwing index is very likely claimed by a worker, exactly the case that used to
  // std::terminate. The throw must instead surface at the caller.
  std::atomic<int> ran{0};
  bool threw = false;
  try {
    pool.parallel_for(256, [&](int i) {
      ran.fetch_add(1, std::memory_order_relaxed);
      if (i == 123)
        throw std::runtime_error("boom");
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    });
  } catch (const std::runtime_error &e) {
    threw = true;
    EXPECT_STREQ(e.what(), "boom");
  }
  EXPECT_TRUE(threw) << "the task exception must propagate to parallel_for's caller";

  // remaining drained, _job cleared, t_active_pool restored -> the pool still works.
  std::vector<int> hits(500, 0);
  pool.parallel_for(500, [&](int i) { hits[i]++; });
  for (int h : hits)
    ASSERT_EQ(h, 1);
}

TEST(ThreadPool, EveryTaskThrowsSurfacesExactlyOneAndDoesNotCrash) {
  thread_pool pool;
  int caught = 0;
  try {
    pool.parallel_for(200, [&](int i) { throw i; }); // every index throws (an int)
  } catch (int) {
    caught = 1;
  } catch (...) {
    caught = -1;
  }
  EXPECT_EQ(caught, 1) << "exactly one exception, of the thrown type, should surface";
  std::atomic<int> acc{0};
  pool.parallel_for(1000, [&](int) { acc.fetch_add(1, std::memory_order_relaxed); });
  EXPECT_EQ(acc.load(), 1000);
}

TEST(ThreadPool, ThrowOnInlineAndWorkerlessPathsPropagatesAndRestoresState) {
  thread_pool pool;
  // n == 1 always runs inline on the caller; the throw must propagate AND restore
  // t_active_pool so the next fan-out is not stuck on the re-entrant path.
  EXPECT_THROW(pool.parallel_for(1, [&](int) { throw std::runtime_error("x"); }),
               std::runtime_error);
  std::atomic<int> acc{0};
  pool.parallel_for(300, [&](int) { acc.fetch_add(1, std::memory_order_relaxed); });
  EXPECT_EQ(acc.load(), 300);

  // A workerless pool runs every index inline on the caller.
  thread_pool solo(0);
  EXPECT_THROW(solo.parallel_for(50,
                                 [&](int i) {
                                   if (i == 10)
                                     throw std::runtime_error("y");
                                 }),
               std::runtime_error);
  std::atomic<int> acc2{0};
  solo.parallel_for(50, [&](int) { acc2.fetch_add(1, std::memory_order_relaxed); });
  EXPECT_EQ(acc2.load(), 50);
}

TEST(ThreadPool, NestedTaskThrowPropagatesThroughOuterFanout) {
  thread_pool pool;
  int caught = 0;
  try {
    pool.parallel_for(32, [&](int i) {
      // The inner fan-out runs inline (re-entrant). Its throw must escape the inner
      // parallel_for, be caught by the outer task's run_indices, latched, and re-thrown
      // to us from the outer parallel_for.
      pool.parallel_for(8, [&](int k) {
        if (i == 5 && k == 3)
          throw std::runtime_error("nested");
      });
    });
  } catch (const std::runtime_error &) {
    caught = 1;
  }
  EXPECT_EQ(caught, 1);
  std::atomic<int> acc{0};
  pool.parallel_for(200, [&](int) { acc.fetch_add(1, std::memory_order_relaxed); });
  EXPECT_EQ(acc.load(), 200);
}

// Concurrent fan-outs run side by side: a fan-out started while another is in flight begins at
// once (on its own caller, and on any free worker) instead of waiting for the first to drain.
// Each of two fan-outs holds its chunks until it has seen the other one running; when fan-outs
// serialized, the second caller blocked before running anything, so the first waited out the
// deadline.
TEST(ThreadPool, ConcurrentFanoutsOverlap) {
  thread_pool pool(4);
  std::atomic<bool> running[2] = {false, false};
  std::atomic<bool> saw_other[2] = {false, false};
  auto fanout = [&](int me) {
    pool.parallel_for(
        4,
        [&, me](int) {
          running[me].store(true);
          const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
          while (!running[1 - me].load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::yield();
          if (running[1 - me].load())
            saw_other[me].store(true);
        },
        2);
  };
  std::thread a(fanout, 0);
  std::thread b(fanout, 1);
  a.join();
  b.join();
  EXPECT_TRUE(saw_other[0].load());
  EXPECT_TRUE(saw_other[1].load());
}

// Every chunk of a fan-out runs under its own poster's fanout_context (state_exec charges a
// kernel's chunks to its call through it), also when several fan-outs share the workers, and a
// worker only joins a fan-out through run_chunks, at most as many as the fan-out's cap allows.
namespace {
struct counting_context final : cvc::fanout_context {
  std::atomic<int> shares{0};
  std::atomic<int> inside{0};
  std::atomic<int> most_inside{0};
  void run_chunks(const std::function<void()> &chunks) noexcept override {
    shares.fetch_add(1);
    const int now = inside.fetch_add(1) + 1;
    int seen = most_inside.load();
    while (now > seen && !most_inside.compare_exchange_weak(seen, now)) {
    }
    chunks();
    inside.fetch_sub(1);
  }
};
} // namespace

TEST(ThreadPool, ConcurrentFanoutsKeepTheirOwnContexts) {
  thread_pool pool(6);
  constexpr int kPosters = 3;
  constexpr int kIters = 200;
  counting_context ctx[kPosters];
  std::atomic<int> wrong{0};
  std::atomic<long long> ran[kPosters] = {0, 0, 0};
  std::vector<std::thread> posters;
  for (int p = 0; p < kPosters; ++p)
    posters.emplace_back([&, p] {
      cvc::fanout_context_scope scope(&ctx[p]);
      for (int r = 0; r < kIters; ++r)
        pool.parallel_for(
            32,
            [&, p](int) {
              if (cvc::current_fanout_context() != &ctx[p])
                wrong.fetch_add(1);
              ran[p].fetch_add(1);
            },
            3); // the caller and at most 2 workers
    });
  for (auto &t : posters)
    t.join();
  EXPECT_EQ(wrong.load(), 0);
  for (int p = 0; p < kPosters; ++p) {
    EXPECT_EQ(ran[p].load(), 32LL * kIters);
    // Worker shares only (the caller's own chunks count as its own work), never more than the
    // cap's 2 workers in one fan-out at a time (the poster runs its fan-outs one after another).
    EXPECT_LE(ctx[p].most_inside.load(), 2);
    EXPECT_LE(ctx[p].shares.load(), 2 * kIters);
  }
}
