/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::result_job<R>: any resumable stepper as a cvc::sliced_job that owns its result
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.9). A port's routine is a stepper beside
// its blocking function; the job wraps it, so the scheduler (schedule_sliced_job), a node, the
// DSL's intrinsics (cvc/core/job_intrinsics.h) and pycvc all run it the same way:
//
//   std::unique_ptr<cvc::result_job<cvc::contour_tree>> job =
//       cvc::make_result_job(cvc::contour_tree_builder(vol, opt)); // R from take()
//   while (!job->step(4096)) show(job->phase(), job->progress());
//   cvc::contour_tree t = job->take_result();                     // or result()
//
// The stepper. What it must have: bool step(std::size_t budget) (at most about `budget` units;
// budget 0 runs to the end; true once done) and double progress() const. What it may have, used
// when it does: const char *phase() const (a static string), std::string status() const, bool
// done() const, void cancel() (thread-safe, for a stop within a step), and R take() (else pass a
// taker: make_result_job<R>(stepper, [](S &s) { return s.field(); })). A failure is an exception
// its step throws; the job catches it.
//
// The job. It owns the stepper (and through it the stepper's shared, read-only input) and:
//   - step(budget) steps it, and once it is done takes the result and destroys the stepper, so the
//     stepper's working memory goes back as soon as the result is ready;
//   - cancel() (from any thread) makes the next step end the job with nothing, destroying the
//     stepper there; done() and cancelled() are true at once;
//   - a step that throws ends the job: failed(), error() and exception() say what, phase() is
//     "failed" and status() "failed: <message>" (sliced_job reports failures through status());
//   - phase() is the stepper's (or "running" for one with none), then "done", "cancelled" or
//     "failed"; progress() is the stepper's, clamped to [0, 1] and never falling, and 1 once done;
//     status() is the stepper's when it has one, else "<phase> 42%", and once done "done",
//     "cancelled" or the failure;
//   - result() / take_result() hand out the result once done, and throw otherwise (rethrowing the
//     stepper's own exception when it failed);
//   - work_ms(), max_step_ms() and steps() time the steps (steady clock), for stats.work_ms and
//     T7's slowest-step record.
//
// Threads. Like the stepper, the job is not thread-safe, but it may move between threads between
// steps (threaded mode does that). cancel() is the exception, and phase(), progress(), done(),
// cancelled(), failed(), work_ms(), max_step_ms() and steps() may be read from another thread
// while a step runs (a UI polling a job). status() and error() may not.

#ifndef CVC_CORE_RESULT_JOB_H
#define CVC_CORE_RESULT_JOB_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdio>
#include <cvc/core/sliced_task.h>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace cvc {

// What result_job needs of a stepper.
template <class S>
concept resumable_stepper = requires(S &s, const S &cs, std::size_t n) {
  { s.step(n) } -> std::convertible_to<bool>;
  { cs.progress() } -> std::convertible_to<double>;
};

// The error result()/take_result() throw when there is no result to hand out.
class result_job_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// What every result_job<R> reports, whatever R is: for code that handles any job (the node keys in
// cvc/core/job_keys.h, the DSL's generic forms).
class result_job_base : public sliced_job {
public:
  // The stepper threw: what, as text and as the exception.
  virtual bool failed() const = 0;
  virtual std::string error() const = 0;
  virtual std::exception_ptr exception() const = 0;
  // Whether result() would hand one out now.
  virtual bool has_result() const = 0;
  // Steady-clock time inside step(), the slowest step, and how many steps.
  virtual double work_ms() const = 0;
  virtual double max_step_ms() const = 0;
  virtual std::size_t steps() const = 0;
};

template <class R> class result_job : public result_job_base {
public:
  using result_type = R;
  ~result_job() override = default;

  // ---- sliced_job ----
  bool step(std::size_t budget) final {
    if (_finished.load(std::memory_order_acquire))
      return true;
    if (_cancel.load(std::memory_order_acquire)) {
      end_cancelled();
      return true;
    }
    const auto t0 = std::chrono::steady_clock::now();
    bool ready = false;
    try {
      ready = do_step(budget);
      if (_cancel.load(std::memory_order_acquire)) {
        end_cancelled();
      } else if (ready) {
        // The stepper's own summary once done, if it has a status(): read before take() (a
        // stepper may let go of its state there), so status() is not left at the last progress
        // note ("clean-up 87%").
        std::string s;
        if (do_status(s))
          _status = std::move(s);
        _result.emplace(do_take());
        do_release();
        _phase.store("done", std::memory_order_release);
        _progress.store(1.0, std::memory_order_release);
        _finished.store(true, std::memory_order_release);
      } else {
        publish_progress();
      }
    } catch (...) {
      _error = std::current_exception();
      _error_text = describe(_error);
      do_release();
      _phase.store("failed", std::memory_order_release);
      _failed.store(true, std::memory_order_release);
      _finished.store(true, std::memory_order_release);
    }
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    _work_ms.store(_work_ms.load(std::memory_order_relaxed) + ms, std::memory_order_relaxed);
    if (ms > _max_step_ms.load(std::memory_order_relaxed))
      _max_step_ms.store(ms, std::memory_order_relaxed);
    _steps.fetch_add(1, std::memory_order_relaxed);
    return _finished.load(std::memory_order_acquire);
  }
  bool done() const final {
    return _finished.load(std::memory_order_acquire) || _cancel.load(std::memory_order_acquire);
  }
  void cancel() final {
    if (!_cancel.exchange(true, std::memory_order_acq_rel))
      do_cancel();
  }
  bool cancelled() const final {
    return _cancel.load(std::memory_order_acquire) && !_failed.load(std::memory_order_acquire) &&
           !has_result_now();
  }
  const char *phase() const final {
    if (_cancel.load(std::memory_order_acquire) && !_finished.load(std::memory_order_acquire))
      return "cancelled";
    return _phase.load(std::memory_order_acquire);
  }
  double progress() const final { return _progress.load(std::memory_order_acquire); }
  std::string status() const final {
    if (_failed.load(std::memory_order_acquire))
      return "failed: " + _error_text;
    if (cancelled())
      return "cancelled";
    if (_finished.load(std::memory_order_acquire))
      return _status.empty() ? std::string("done") : _status;
    if (!_status.empty())
      return _status;
    return std::string(phase()) + " " + percent_text(progress());
  }

  // ---- the result ----
  bool failed() const final { return _failed.load(std::memory_order_acquire); }
  std::exception_ptr exception() const final { return _error; }
  std::string error() const final { return _error_text; }
  bool has_result() const final { return has_result_now(); }
  // The result, once done. Throws result_job_error while it runs, after a cancel and once taken,
  // and rethrows the stepper's exception after a failure.
  const R &result() const {
    check_result();
    return *_result;
  }
  // The result moved out (once).
  R take_result() {
    check_result();
    R r = std::move(*_result);
    _result.reset();
    _taken = true;
    return r;
  }

  // ---- timings ----
  double work_ms() const final { return _work_ms.load(std::memory_order_relaxed); }
  double max_step_ms() const final { return _max_step_ms.load(std::memory_order_relaxed); }
  std::size_t steps() const final { return _steps.load(std::memory_order_relaxed); }

protected:
  result_job() = default;
  // The stepper, as the derived class holds it.
  virtual bool do_step(std::size_t budget) = 0;
  virtual R do_take() = 0;
  virtual void do_release() = 0; // destroy the stepper
  virtual void do_cancel() {}    // the stepper's own cancel(), if it has one (any thread)
  virtual const char *do_phase() const = 0;
  virtual double do_progress() const = 0;
  virtual bool do_status(std::string &) const { return false; } // the stepper's own status()

private:
  static std::string percent_text(double p) {
    char buf[16];
    std::snprintf(buf, sizeof buf, "%d%%", static_cast<int>(std::floor(100.0 * p)));
    return buf;
  }
  static std::string describe(const std::exception_ptr &e) {
    try {
      std::rethrow_exception(e);
    } catch (const std::exception &x) {
      return x.what();
    } catch (...) {
      return "an unknown exception";
    }
  }
  bool has_result_now() const {
    return _finished.load(std::memory_order_acquire) && !_failed.load(std::memory_order_acquire) &&
           _result.has_value();
  }
  void check_result() const {
    if (_failed.load(std::memory_order_acquire) && _error)
      std::rethrow_exception(_error);
    if (_taken)
      throw result_job_error("result_job: the result was already taken");
    if (_cancel.load(std::memory_order_acquire) && !has_result_now())
      throw result_job_error("result_job: cancelled");
    if (!has_result_now())
      throw result_job_error("result_job: not done yet");
  }
  void end_cancelled() {
    do_release();
    _finished.store(true, std::memory_order_release);
    _phase.store("cancelled", std::memory_order_release);
  }
  void publish_progress() {
    const char *ph = do_phase();
    _phase.store(ph ? ph : "running", std::memory_order_release);
    double p = do_progress();
    if (!(p >= 0.0)) // NaN too
      p = 0.0;
    p = std::min(p, 1.0);
    _progress.store(std::max(p, _progress.load(std::memory_order_relaxed)),
                    std::memory_order_release);
    std::string s;
    if (do_status(s))
      _status = std::move(s);
  }

  std::atomic<bool> _cancel{false};
  std::atomic<bool> _finished{false};
  std::atomic<bool> _failed{false};
  std::atomic<const char *> _phase{"starting"};
  std::atomic<double> _progress{0.0};
  std::atomic<double> _work_ms{0.0};
  std::atomic<double> _max_step_ms{0.0};
  std::atomic<std::size_t> _steps{0};
  std::string _status; // the stepper's own status() after the last step, if it has one
  std::optional<R> _result;
  bool _taken = false;
  std::exception_ptr _error;
  std::string _error_text;
};

namespace result_job_detail {

template <class S>
concept has_phase = requires(const S &s) {
  { s.phase() } -> std::convertible_to<const char *>;
};
template <class S>
concept has_status = requires(const S &s) {
  { s.status() } -> std::convertible_to<std::string>;
};
template <class S>
concept has_cancel = requires(S &s) { s.cancel(); };
template <class S>
concept has_done = requires(const S &s) {
  { s.done() } -> std::convertible_to<bool>;
};
template <class S>
concept has_take = requires(S &s) { s.take(); };

template <class R, class S, class Take> class stepper_job final : public result_job<R> {
public:
  stepper_job(std::unique_ptr<S> s, Take take) : _s(std::move(s)), _take(std::move(take)) {
    if (!_s)
      throw std::invalid_argument("make_result_job: no stepper");
  }

private:
  bool do_step(std::size_t budget) override {
    const bool r = static_cast<bool>(_s->step(budget));
    if constexpr (has_done<S>)
      return r || static_cast<bool>(std::as_const(*_s).done());
    else
      return r;
  }
  R do_take() override { return _take(*_s); }
  void do_release() override {
    if constexpr (has_cancel<S>) {
      std::lock_guard<std::mutex> lk(_cancel_mu); // no cancel() of another thread's is under way
      _s.reset();
    } else {
      _s.reset();
    }
  }
  void do_cancel() override {
    if constexpr (has_cancel<S>) {
      // Any thread, while a step may run: the stepper's cancel() is its one thread-safe member.
      // The lock keeps the stepper from being destroyed meanwhile (do_release).
      std::lock_guard<std::mutex> lk(_cancel_mu);
      if (_s)
        _s->cancel();
    }
  }
  const char *do_phase() const override {
    if constexpr (has_phase<S>)
      return _s->phase();
    else
      return "running";
  }
  double do_progress() const override { return static_cast<double>(_s->progress()); }
  bool do_status(std::string &out) const override {
    if constexpr (has_status<S>) {
      out = _s->status();
      return true;
    } else {
      (void)out;
      return false;
    }
  }

  std::unique_ptr<S> _s;
  std::mutex _cancel_mu; // a stepper with cancel(): do_cancel against do_release
  Take _take;

public:
  ~stepper_job() override = default;
};

} // namespace result_job_detail

// The job of a stepper held by pointer (a stepper that cannot move, such as fem::stepped_solve,
// comes from a factory as a unique_ptr), with a taker that makes the result from the finished
// stepper.
template <class R, resumable_stepper S, class Take>
  requires std::is_invocable_r_v<R, Take &, S &>
std::unique_ptr<result_job<R>> make_result_job(std::unique_ptr<S> stepper, Take take) {
  return std::make_unique<result_job_detail::stepper_job<R, S, Take>>(std::move(stepper),
                                                                      std::move(take));
}

// The same, the result being the stepper's take().
template <resumable_stepper S>
  requires result_job_detail::has_take<S>
auto make_result_job(std::unique_ptr<S> stepper) {
  using R = std::decay_t<decltype(stepper->take())>;
  auto take = [](S &s) -> R { return s.take(); };
  return make_result_job<R>(std::move(stepper), take);
}

// The job of a stepper given by value (moved into the job).
template <resumable_stepper S>
  requires result_job_detail::has_take<S> && std::move_constructible<S>
auto make_result_job(S stepper) {
  return make_result_job(std::make_unique<S>(std::move(stepper)));
}

template <class R, resumable_stepper S, class Take>
  requires std::move_constructible<S> && std::is_invocable_r_v<R, Take &, S &>
std::unique_ptr<result_job<R>> make_result_job(S stepper, Take take) {
  return make_result_job<R>(std::make_unique<S>(std::move(stepper)), std::move(take));
}

} // namespace cvc

#endif // CVC_CORE_RESULT_JOB_H
