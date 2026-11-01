// PR-9 (docs/STATE_EXEC_LOCKS.md 7.6, 13.6): cvc::thread_class, the OS thread and process
// classes. An executable of its own: a process class can't be undone unprivileged on Linux, so it
// must not leak into another suite's process.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cvc/core/async_lane.h>
#include <cvc/core/thread_class.h>
#include <cvc/core/thread_pool.h>
#include <future>
#include <gtest/gtest.h>
#include <thread>

#if defined(__linux__) && !defined(__EMSCRIPTEN__)
#include <cerrno>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace {

using cvc::thread_class;

#if defined(__linux__) && !defined(__EMSCRIPTEN__)
long tid() { return static_cast<long>(syscall(SYS_gettid)); }

// A thread's nice value, or -100 when it can't be read.
int nice_of(long t) {
  errno = 0;
  const int n = getpriority(PRIO_PROCESS, static_cast<id_t>(t));
  return n == -1 && errno != 0 ? -100 : n;
}

// What `fn` returns on one of `pool`'s workers (not the calling thread): a two-way fan-out whose
// two indices meet, so the worker takes one while the caller holds the other.
template <class F> auto on_worker(cvc::thread_pool &pool, F fn) {
  std::atomic<int> arrived{0};
  const long caller = tid();
  decltype(fn()) result{};
  pool.parallel_for(
      2,
      [&](int) {
        ++arrived;
        const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        while (arrived.load() < 2 && std::chrono::steady_clock::now() < until)
          std::this_thread::yield();
        if (tid() != caller)
          result = fn();
      },
      2);
  return result;
}

// Whether this process may raise a thread's nice value back (CAP_SYS_NICE, or an RLIMIT_NICE that
// allows it), tried on a scratch thread.
bool can_raise_nice() {
  bool raised = false;
  std::thread([&] {
    const int now = nice_of(tid());
    if (setpriority(PRIO_PROCESS, static_cast<id_t>(tid()), now + 1) == 0)
      raised = setpriority(PRIO_PROCESS, static_cast<id_t>(tid()), now) == 0;
  }).join();
  return raised;
}
#endif

} // namespace

TEST(Priority, PR09_ThreadAndProcessClasses) {
  // The thread class, on a thread of its own: reported, never thrown.
  bool background = false, utility = false, normal = false;
  std::thread([&] {
    background = cvc::set_current_thread_class(thread_class::background);
    utility = cvc::set_current_thread_class(thread_class::utility);
    normal = cvc::set_current_thread_class(thread_class::normal);
  }).join();
  std::printf("thread class: background %d, utility %d, normal %d\n", background, utility, normal);
#if defined(__EMSCRIPTEN__)
  EXPECT_FALSE(background);
  EXPECT_FALSE(utility);
  EXPECT_FALSE(normal);
  EXPECT_FALSE(cvc::set_process_class(thread_class::background));
  EXPECT_FALSE(cvc::set_process_class(thread_class::utility));
  EXPECT_FALSE(cvc::set_process_class(thread_class::normal));
#elif defined(_WIN32)
  EXPECT_TRUE(background);
  EXPECT_TRUE(utility);
  EXPECT_TRUE(normal);
  // The process class, and back to normal: every step taken.
  EXPECT_TRUE(cvc::set_process_class(thread_class::background));
  EXPECT_TRUE(cvc::set_process_class(thread_class::background)); // already in it
  EXPECT_TRUE(cvc::set_process_class(thread_class::utility));
  EXPECT_TRUE(cvc::set_process_class(thread_class::normal));
#elif defined(__APPLE__)
  EXPECT_TRUE(background);
  // A process has no utility class here; background (PRIO_DARWIN_BG) and back are reported.
  EXPECT_FALSE(cvc::set_process_class(thread_class::utility));
  const bool bg = cvc::set_process_class(thread_class::background);
  const bool back = cvc::set_process_class(thread_class::normal);
  std::printf("process class: background %d, normal %d\n", bg, back);
#elif defined(__linux__)
  // Demoting needs no privilege; raising back (5, then 0, from 10) does.
  const bool raise_ok = can_raise_nice();
  std::printf("this process %s raise a nice value back\n", raise_ok ? "can" : "cannot");
  EXPECT_TRUE(background);
  EXPECT_EQ(utility, raise_ok);
  EXPECT_EQ(normal, raise_ok);

  // The process class: per thread on Linux. Set from a thread that isn't the main one, it reaches
  // the main thread and the library's workers made before the call (the /proc/self/task walk),
  // and those made after it (their worker loops apply it as they start).
  const long main_tid = tid();
  if (nice_of(main_tid) > 10)
    GTEST_SKIP() << "the test runs below nice 10 already: background would be a raise";
  cvc::thread_pool before_pool(1);
  const long pool_tid = on_worker(before_pool, [] { return tid(); });
  ASSERT_GT(pool_tid, 0);
  ASSERT_NE(pool_tid, main_tid);
  cvc::async_lane lane("pr09");
  std::promise<long> lane_tid_p;
  ASSERT_TRUE(lane.submit([&] { lane_tid_p.set_value(tid()); }));
  const long lane_tid = lane_tid_p.get_future().get();
  bool applied = false;
  std::thread([&] { applied = cvc::set_process_class(thread_class::background); }).join();
  EXPECT_TRUE(applied);
  EXPECT_EQ(nice_of(main_tid), 10);
  EXPECT_EQ(nice_of(pool_tid), 10);
  EXPECT_EQ(nice_of(lane_tid), 10);
  cvc::thread_pool after_pool(1);
  EXPECT_EQ(on_worker(after_pool, [] { return nice_of(tid()); }), 10);
  std::promise<int> lane_nice_p;
  cvc::async_lane lane_after("pr09b");
  ASSERT_TRUE(lane_after.submit([&] { lane_nice_p.set_value(nice_of(tid())); }));
  EXPECT_EQ(lane_nice_p.get_future().get(), 10);
#else
  (void)background;
  (void)utility;
  (void)normal;
#endif
}
