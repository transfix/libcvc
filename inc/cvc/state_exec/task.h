/**
 * @file task.h
 * @brief Lazy single-shot coroutine return type with continuation support.
 *
 * Provides task<T>, a C++20 coroutine type that suspends immediately on
 * creation and resumes when awaited.  Supports both void and non-void
 * return types, exception propagation, and symmetric transfer for
 * efficient coroutine chaining.
 */
#ifndef CVC_STATE_EXEC_TASK_H
#define CVC_STATE_EXEC_TASK_H

#include <coroutine>
#include <exception>
#include <utility>
#include <variant>

namespace cvc::state_exec {

/// Lazy, single-shot coroutine return type with continuation support.
///
/// Usage:
///   task<int> compute() { co_return 42; }
///   task<void> run()    { int x = co_await compute(); }
///
/// The coroutine is lazy: it does not start until awaited or
/// explicitly resumed.  When awaited, it stores a continuation
/// (the awaiting coroutine) and resumes it on completion.
template <typename T = void>
class task;

// ---------------------------------------------------------------------------
// task<T> — non-void specialization
// ---------------------------------------------------------------------------

template <typename T>
class task {
public:
    struct promise_type {
        std::variant<std::monostate, T, std::exception_ptr> result_;
        std::coroutine_handle<> continuation_{std::noop_coroutine()};

        task get_return_object() noexcept {
            return task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }

        // On final suspend, resume the continuation (the awaiting coroutine)
        auto final_suspend() noexcept {
            struct final_awaiter {
                bool await_ready() noexcept { return false; }
                std::coroutine_handle<> await_suspend(
                    std::coroutine_handle<promise_type> h) noexcept {
                    return h.promise().continuation_;
                }
                void await_resume() noexcept {}
            };
            return final_awaiter{};
        }

        void return_value(T v) { result_.template emplace<1>(std::move(v)); }

        void unhandled_exception() {
            result_.template emplace<2>(std::current_exception());
        }
    };

    using handle_type = std::coroutine_handle<promise_type>;

    task() noexcept = default;
    explicit task(handle_type h) noexcept : handle_(h) {}

    task(task&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    task& operator=(task&& o) noexcept {
        if (this != &o) {
            if (handle_) handle_.destroy();
            handle_ = o.handle_;
            o.handle_ = nullptr;
        }
        return *this;
    }

    task(const task&) = delete;
    task& operator=(const task&) = delete;

    ~task() { if (handle_) handle_.destroy(); }

    /// Run the coroutine to completion synchronously.
    T sync_wait() {
        // Set continuation to noop so final_suspend returns to us
        handle_.promise().continuation_ = std::noop_coroutine();
        while (!handle_.done()) handle_.resume();
        auto& r = handle_.promise().result_;
        if (r.index() == 2)
            std::rethrow_exception(std::get<2>(r));
        return std::move(std::get<1>(r));
    }

    /// Awaiter: suspends the caller, stores it as continuation,
    /// and resumes this (inner) coroutine.
    auto operator co_await() noexcept {
        struct awaiter {
            handle_type h;
            bool await_ready() noexcept { return false; }
            std::coroutine_handle<> await_suspend(
                std::coroutine_handle<> caller) noexcept {
                h.promise().continuation_ = caller;
                return h; // symmetric transfer to inner
            }
            T await_resume() {
                auto& r = h.promise().result_;
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

template <>
class task<void> {
public:
    struct promise_type {
        std::exception_ptr exception_;
        std::coroutine_handle<> continuation_{std::noop_coroutine()};

        task get_return_object() noexcept {
            return task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always initial_suspend() noexcept { return {}; }

        auto final_suspend() noexcept {
            struct final_awaiter {
                bool await_ready() noexcept { return false; }
                std::coroutine_handle<> await_suspend(
                    std::coroutine_handle<promise_type> h) noexcept {
                    return h.promise().continuation_;
                }
                void await_resume() noexcept {}
            };
            return final_awaiter{};
        }

        void return_void() noexcept {}

        void unhandled_exception() {
            exception_ = std::current_exception();
        }
    };

    using handle_type = std::coroutine_handle<promise_type>;

    task() noexcept = default;
    explicit task(handle_type h) noexcept : handle_(h) {}

    task(task&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    task& operator=(task&& o) noexcept {
        if (this != &o) {
            if (handle_) handle_.destroy();
            handle_ = o.handle_;
            o.handle_ = nullptr;
        }
        return *this;
    }

    task(const task&) = delete;
    task& operator=(const task&) = delete;

    ~task() { if (handle_) handle_.destroy(); }

    void sync_wait() {
        handle_.promise().continuation_ = std::noop_coroutine();
        while (!handle_.done()) handle_.resume();
        if (handle_.promise().exception_)
            std::rethrow_exception(handle_.promise().exception_);
    }

    auto operator co_await() noexcept {
        struct awaiter {
            handle_type h;
            bool await_ready() noexcept { return false; }
            std::coroutine_handle<> await_suspend(
                std::coroutine_handle<> caller) noexcept {
                h.promise().continuation_ = caller;
                return h;
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
