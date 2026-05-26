/**
 * @file task.h
 * @brief Lazy single-shot coroutine return type with continuation support.
 *
 * Provides task<T>, a C++20 coroutine type that suspends immediately on
 * creation and resumes when awaited.  Supports both void and non-void
 * return types, exception propagation, and a trampoline-based
 * resumption loop for safe coroutine chaining.
 *
 * ## Trampoline pattern
 *
 * The C++20 symmetric-transfer idiom (await_suspend returning a
 * coroutine_handle) is designed to let the compiler turn chains of
 * co_await calls into tail calls, keeping the native stack depth at
 * O(1).  In practice, GCC and Clang only perform this optimisation
 * at -O1 and above.  In debug builds (-O0 / -Og) every symmetric
 * transfer becomes a regular call, so a deeply recursive evaluator
 * script such as fib(15) — which produces ~1 200 nested
 * eval_internal → apply_closure → eval_internal frames, each
 * involving an intermediate check_interrupted_async co_await —
 * generates 59 000+ native stack frames and immediately segfaults.
 *
 * The fix is a thread-local trampoline:
 *
 *   1. Instead of returning the next coroutine handle from
 *      await_suspend (symmetric transfer), every suspension point
 *      writes the handle into a thread-local slot and uses a void
 *      return from await_suspend to unconditionally suspend —
 *      which returns control to the nearest resume() call on the
 *      native stack.  (We avoid returning noop_coroutine() because
 *      MSVC has coroutine-frame bugs with that pattern.)
 *
 *   2. sync_wait() (and the sync_evaluate timeout loop in
 *      async_evaluator.cpp) drives a flat while-loop that reads
 *      the thread-local, clears it, and resumes the handle, one
 *      step at a time.  The native stack depth is always 1
 *      regardless of how deeply the script recurses.
 *
 *   3. The thread-local is saved/restored on entry/exit of
 *      sync_wait() so that nested synchronous evaluations (e.g.
 *      defclass method wrappers that call sync_wait inside a
 *      native_fn) compose correctly.
 *
 * This gives the same semantics as symmetric transfer — each
 * suspension still resumes exactly the right continuation — but
 * works reliably at every optimisation level.
 */
#ifndef CVC_STATE_EXEC_TASK_H
#define CVC_STATE_EXEC_TASK_H

#include <coroutine>
#include <exception>
#include <utility>
#include <variant>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Thread-local trampoline slot.
//
// Every co_await and final_suspend writes the next coroutine handle
// here instead of doing direct symmetric transfer.  The sync_wait()
// loop picks it up and resumes the handle in a flat loop, avoiding
// O(N) native stack growth for N-deep recursive evaluations.
//
// noop_coroutine() is the sentinel meaning "nothing to resume".
// ---------------------------------------------------------------------------
namespace detail {
inline std::coroutine_handle<> &trampoline_next() noexcept {
  static thread_local std::coroutine_handle<> h{std::noop_coroutine()};
  return h;
}
} // namespace detail

/// Lazy, single-shot coroutine return type with continuation support.
///
/// Usage:
///   task<int> compute() { co_return 42; }
///   task<void> run()    { int x = co_await compute(); }
///
/// The coroutine is lazy: it does not start until awaited or
/// explicitly resumed.  When awaited, it stores a continuation
/// (the awaiting coroutine) and resumes it on completion.
template <typename T = void> class task;

// ---------------------------------------------------------------------------
// task<T> — non-void specialization
// ---------------------------------------------------------------------------

template <typename T> class task {
public:
  struct promise_type {
    std::variant<std::monostate, T, std::exception_ptr> result_;
    std::coroutine_handle<> continuation_{std::noop_coroutine()};

    task get_return_object() noexcept {
      return task{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_always initial_suspend() noexcept { return {}; }

    // On final suspend, hand the continuation to the trampoline rather
    // than doing symmetric transfer (returning it directly), so that the
    // native call stack unwinds back to the sync_wait/trampoline loop.
    // We use void-returning await_suspend (unconditional suspend) instead
    // of returning noop_coroutine(), because MSVC's coroutine frame
    // management has bugs when await_suspend returns noop_coroutine().
    auto final_suspend() noexcept {
      struct final_awaiter {
        bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
        }
        void await_resume() noexcept {}
      };
      return final_awaiter{};
    }

    void return_value(T v) { result_.template emplace<1>(std::move(v)); }

    void unhandled_exception() { result_.template emplace<2>(std::current_exception()); }
  };

  using handle_type = std::coroutine_handle<promise_type>;

  task() noexcept = default;
  explicit task(handle_type h) noexcept : handle_(h) {}

  task(task &&o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
  task &operator=(task &&o) noexcept {
    if (this != &o) {
      if (handle_)
        handle_.destroy();
      handle_ = o.handle_;
      o.handle_ = nullptr;
    }
    return *this;
  }

  task(const task &) = delete;
  task &operator=(const task &) = delete;

  ~task() {
    if (handle_)
      handle_.destroy();
  }

  /// Run the coroutine to completion synchronously.
  ///
  /// Drives the trampoline loop: each iteration resumes exactly one
  /// coroutine; that coroutine's await_suspend / final_suspend writes
  /// the *next* handle into the thread-local slot and returns
  /// noop_coroutine, so resume() returns here immediately.  The loop
  /// keeps the native stack depth at 1 regardless of script recursion.
  ///
  /// The save/restore of the thread-local lets nested sync_wait calls
  /// (e.g. defclass method wrappers) work without corrupting the
  /// outer evaluation's trampoline state.
  T sync_wait() {
    auto saved = detail::trampoline_next(); // save for re-entrancy
    handle_.promise().continuation_ = std::noop_coroutine();
    detail::trampoline_next() = handle_; // seed the loop
    while (!handle_.done()) {
      auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
      if (next == std::noop_coroutine())
        next = handle_; // coroutine suspended via non-trampoline awaitable (e.g. suspend_point)
      next.resume();    // runs one coroutine step; sets trampoline_next
    }
    detail::trampoline_next() = saved; // restore for re-entrancy
    auto &r = handle_.promise().result_;
    if (r.index() == 2)
      std::rethrow_exception(std::get<2>(r));
    if (r.index() == 0)
      throw std::runtime_error("task::sync_wait: coroutine completed without returning a value");
    return std::move(std::get<1>(r));
  }

  /// Awaiter: suspends the caller, stores it as continuation,
  /// and schedules this (inner) coroutine via the trampoline.
  auto operator co_await() noexcept {
    struct awaiter {
      handle_type h;
      bool await_ready() noexcept { return false; }
      // Store caller as continuation so final_suspend chains back,
      // then park the inner handle in the trampoline slot and use
      // void return to unconditionally suspend (avoids MSVC bugs
      // with noop_coroutine return from await_suspend).
      void await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
      }
      T await_resume() {
        auto &r = h.promise().result_;
        if (r.index() == 2)
          std::rethrow_exception(std::get<2>(r));
        return std::move(std::get<1>(r));
      }
    };
    return awaiter{handle_};
  }

  bool done() const noexcept { return handle_ && handle_.done(); }
  handle_type handle() const noexcept { return handle_; }

private:
  handle_type handle_{nullptr};
};

// ---------------------------------------------------------------------------
// task<void> — void specialization
// ---------------------------------------------------------------------------

template <> class task<void> {
public:
  struct promise_type {
    std::exception_ptr exception_;
    std::coroutine_handle<> continuation_{std::noop_coroutine()};

    task get_return_object() noexcept {
      return task{std::coroutine_handle<promise_type>::from_promise(*this)};
    }

    std::suspend_always initial_suspend() noexcept { return {}; }

    // Same trampoline strategy as task<T>::final_suspend (see above).
    auto final_suspend() noexcept {
      struct final_awaiter {
        bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
        }
        void await_resume() noexcept {}
      };
      return final_awaiter{};
    }

    void return_void() noexcept {}

    void unhandled_exception() { exception_ = std::current_exception(); }
  };

  using handle_type = std::coroutine_handle<promise_type>;

  task() noexcept = default;
  explicit task(handle_type h) noexcept : handle_(h) {}

  task(task &&o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
  task &operator=(task &&o) noexcept {
    if (this != &o) {
      if (handle_)
        handle_.destroy();
      handle_ = o.handle_;
      o.handle_ = nullptr;
    }
    return *this;
  }

  task(const task &) = delete;
  task &operator=(const task &) = delete;

  ~task() {
    if (handle_)
      handle_.destroy();
  }

  /// Run the void coroutine to completion (see task<T>::sync_wait).
  void sync_wait() {
    auto saved = detail::trampoline_next();
    handle_.promise().continuation_ = std::noop_coroutine();
    detail::trampoline_next() = handle_;
    while (!handle_.done()) {
      auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
      if (next == std::noop_coroutine())
        next = handle_; // coroutine suspended via non-trampoline awaitable (e.g. suspend_point)
      next.resume();
    }
    detail::trampoline_next() = saved;
    if (handle_.promise().exception_)
      std::rethrow_exception(handle_.promise().exception_);
  }

  // Same trampoline-based co_await as task<T> (see above).
  auto operator co_await() noexcept {
    struct awaiter {
      handle_type h;
      bool await_ready() noexcept { return false; }
      void await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
      }
      void await_resume() {
        if (h.promise().exception_)
          std::rethrow_exception(h.promise().exception_);
      }
    };
    return awaiter{handle_};
  }

  bool done() const noexcept { return handle_ && handle_.done(); }
  handle_type handle() const noexcept { return handle_; }

private:
  handle_type handle_{nullptr};
};

/// A suspend point that yields control to the scheduler.
/// co_await suspend_point{} is equivalent to asyncio.sleep(0).
struct suspend_point {
  bool await_ready() noexcept { return false; }
  void await_suspend(std::coroutine_handle<>) noexcept {}
  void await_resume() noexcept {}
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_TASK_H
