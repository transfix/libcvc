/**
 * @file task.h
 * @brief Lazy single-shot coroutine return type with continuation support.
 *
 * Provides task<T>, a C++20 coroutine type that suspends immediately on
 * creation and resumes when awaited.  Supports both void and non-void
 * return types, exception propagation, and coroutine chaining.
 *
 * ## Coroutine chaining strategy
 *
 * Two resumption strategies are compiled depending on the compiler:
 *
 * ### GCC / Clang — thread-local trampoline
 *
 * GCC and Clang only optimize symmetric transfer (tail-calling the
 * handle returned by await_suspend) at -O1 and above.  In debug
 * builds (-O0 / -Og) every symmetric transfer becomes a regular
 * call, so a deeply recursive evaluator script such as fib(15) —
 * which produces ~1 200 nested eval_internal frames — generates
 * 59 000+ native stack frames and immediately segfaults.
 *
 * The fix is a thread-local trampoline: every suspension point
 * writes the next handle into a thread-local slot and returns void
 * (unconditional suspend), returning control to the nearest
 * resume() call.  sync_wait() drives a flat while-loop that reads
 * and clears the slot, keeping native stack depth at O(1).  The
 * slot is saved/restored for nested sync_wait re-entrancy.
 *
 * ### MSVC — symmetric transfer
 *
 * MSVC performs the coroutine-to-state-machine transform at the
 * frontend, so symmetric transfer is always a tail call regardless
 * of optimisation level.  The trampoline pattern is not needed and
 * triggers MSVC coroutine-frame bugs that manifest as
 * bad_variant_access.  On MSVC, await_suspend returns the next
 * handle directly (standard symmetric transfer) and sync_wait()
 * uses a simple resume() loop.  The trampoline slot is still
 * written (so that async_evaluator's manual timeout loop can
 * optionally read it) but sync_wait itself ignores it.
 */
#ifndef CVC_STATE_EXEC_TASK_H
#define CVC_STATE_EXEC_TASK_H

#include <coroutine>
#include <exception>
#include <utility>
#include <variant>

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Compiler-dependent resumption strategy (see file-level doc comment).
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#define CVC_COROUTINE_TRAMPOLINE 0
#else
#define CVC_COROUTINE_TRAMPOLINE 1
#endif

// ---------------------------------------------------------------------------
// Thread-local trampoline slot.
//
// On GCC/Clang: every co_await and final_suspend writes the next
// coroutine handle here instead of doing direct symmetric transfer.
// sync_wait() drives a flat while-loop, keeping native stack at O(1).
//
// On MSVC: still written by await_suspend / final_suspend (so
// async_evaluator's manual timeout loop can use it if needed), but
// sync_wait() ignores it and uses symmetric transfer instead.
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

    // On final suspend, store the continuation in the trampoline slot.
    // GCC/Clang: return void (unconditional suspend, trampoline resumes).
    // MSVC: return the continuation handle (symmetric transfer).
    auto final_suspend() noexcept {
      struct final_awaiter {
        bool await_ready() noexcept { return false; }
#if CVC_COROUTINE_TRAMPOLINE
        void await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
        }
#else
        std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
          return h.promise().continuation_;
        }
#endif
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
  /// GCC/Clang: drives the trampoline loop — each iteration resumes
  /// exactly one coroutine step, keeping native stack depth at 1.
  /// MSVC: uses direct resume() with symmetric transfer (always a
  /// tail call on MSVC regardless of optimisation level).
  T sync_wait() {
#if CVC_COROUTINE_TRAMPOLINE
    auto saved = detail::trampoline_next(); // save for re-entrancy
    handle_.promise().continuation_ = std::noop_coroutine();
    detail::trampoline_next() = handle_; // seed the loop
    while (!handle_.done()) {
      auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
      if (next == std::noop_coroutine())
        next = handle_; // non-trampoline awaitable (e.g. suspend_point)
      next.resume();
    }
    detail::trampoline_next() = saved; // restore for re-entrancy
#else
    handle_.promise().continuation_ = std::noop_coroutine();
    handle_.resume();
    while (!handle_.done())
      handle_.resume();
#endif
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
      // Store caller as continuation, write inner handle to trampoline.
      // GCC/Clang: void return (unconditional suspend, trampoline loop resumes).
      // MSVC: return inner handle (symmetric transfer, tail call).
#if CVC_COROUTINE_TRAMPOLINE
      void await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
      }
#else
      std::coroutine_handle<> await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
        return h;
      }
#endif
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

    // Same strategy as task<T>::final_suspend (see above).
    auto final_suspend() noexcept {
      struct final_awaiter {
        bool await_ready() noexcept { return false; }
#if CVC_COROUTINE_TRAMPOLINE
        void await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
        }
#else
        std::coroutine_handle<> await_suspend(std::coroutine_handle<promise_type> h) noexcept {
          detail::trampoline_next() = h.promise().continuation_;
          return h.promise().continuation_;
        }
#endif
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
#if CVC_COROUTINE_TRAMPOLINE
    auto saved = detail::trampoline_next();
    handle_.promise().continuation_ = std::noop_coroutine();
    detail::trampoline_next() = handle_;
    while (!handle_.done()) {
      auto next = std::exchange(detail::trampoline_next(), std::noop_coroutine());
      if (next == std::noop_coroutine())
        next = handle_;
      next.resume();
    }
    detail::trampoline_next() = saved;
#else
    handle_.promise().continuation_ = std::noop_coroutine();
    handle_.resume();
    while (!handle_.done())
      handle_.resume();
#endif
    if (handle_.promise().exception_)
      std::rethrow_exception(handle_.promise().exception_);
  }

  // Same compiler-conditional co_await as task<T> (see above).
  auto operator co_await() noexcept {
    struct awaiter {
      handle_type h;
      bool await_ready() noexcept { return false; }
#if CVC_COROUTINE_TRAMPOLINE
      void await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
      }
#else
      std::coroutine_handle<> await_suspend(std::coroutine_handle<> caller) noexcept {
        h.promise().continuation_ = caller;
        detail::trampoline_next() = h;
        return h;
      }
#endif
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
