/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// How the vendored cocone and skeleton code is made resumable (VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.9): each of its long functions is a C++20 coroutine returning a `step_task`, and every loop
// whose length depends on the input -- over cells, facets, edges or vertices, and every walk or
// work stack -- yields once per iteration (SK_UNIT). So the legacy code keeps its shape, its
// locals live in the coroutine frame between steps, and a walk stops mid-way and resumes where it
// was. A function that calls another long one runs it with SK_RUN, which passes each of the
// callee's units up as one of its own.
//
//   step_task tang_umbrella(Triangulation &triang) {
//     for (FFI fit = ...; ...; ++fit) {
//       SK_UNIT;
//       ...
//     }
//   }
//
// The stepper resumes its top task once per unit, so step(budget) does at most `budget`
// iterations, each bounded by the local degree around a cell, edge or vertex. Destroying a task
// destroys its frame and every suspended callee's (a cancel frees everything at once). An
// exception escapes from resume() and is rethrown by the caller's SK_RUN, up to the stepper.

#ifndef CVC_GEOMETRY_SKELETON_STEPPING_H
#define CVC_GEOMETRY_SKELETON_STEPPING_H

#include <coroutine>
#include <exception>
#include <utility>

namespace cvc::cocone {

class step_task {
public:
  struct promise_type {
    std::exception_ptr error;
    step_task get_return_object() {
      return step_task(std::coroutine_handle<promise_type>::from_promise(*this));
    }
    std::suspend_always initial_suspend() noexcept { return {}; }
    std::suspend_always final_suspend() noexcept { return {}; }
    std::suspend_always yield_value(int) noexcept { return {}; }
    void return_void() noexcept {}
    void unhandled_exception() { error = std::current_exception(); }
  };

  step_task() = default;
  explicit step_task(std::coroutine_handle<promise_type> h) : _h(h) {}
  step_task(step_task &&o) noexcept : _h(std::exchange(o._h, {})) {}
  step_task &operator=(step_task &&o) noexcept {
    if (this != &o) {
      if (_h)
        _h.destroy();
      _h = std::exchange(o._h, {});
    }
    return *this;
  }
  step_task(const step_task &) = delete;
  step_task &operator=(const step_task &) = delete;
  ~step_task() {
    if (_h)
      _h.destroy();
  }

  // Runs to the next unit; true once the function has returned. Rethrows what it threw.
  bool resume() {
    if (!_h || _h.done())
      return true;
    _h.resume();
    if (_h.promise().error)
      std::rethrow_exception(std::exchange(_h.promise().error, nullptr));
    return _h.done();
  }
  bool done() const { return !_h || _h.done(); }
  explicit operator bool() const { return bool(_h); }

private:
  std::coroutine_handle<promise_type> _h;
};

} // namespace cvc::cocone

#include <vector>

namespace cvc::cocone {

// A stable bottom-up merge sort of v by `less`, a unit per 64 elements moved: a sort that a
// stepper can slice (std::sort would be one indivisible call).
template <class T, class Less> step_task sliced_sort(std::vector<T> &v, Less less) {
  const std::size_t n = v.size();
  if (n < 2)
    co_return;
  std::vector<T> buf;
  buf.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    buf.push_back(v[i]);
    if (i % 64 == 63)
      co_yield 0;
  }
  std::vector<T> *src = &v, *dst = &buf;
  std::size_t moved = 0;
  for (std::size_t width = 1; width < n; width *= 2) {
    for (std::size_t lo = 0; lo < n; lo += 2 * width) {
      const std::size_t mid = std::min(lo + width, n), hi = std::min(lo + 2 * width, n);
      std::size_t a = lo, b = mid, o = lo;
      while (a < mid || b < hi) {
        if (b >= hi || (a < mid && !less((*src)[b], (*src)[a])))
          (*dst)[o++] = (*src)[a++];
        else
          (*dst)[o++] = (*src)[b++];
        if (++moved % 64 == 0)
          co_yield 0;
      }
    }
    std::swap(src, dst);
  }
  if (src != &v)
    for (std::size_t i = 0; i < n; ++i) {
      v[i] = (*src)[i];
      if (i % 64 == 63)
        co_yield 0;
    }
}

} // namespace cvc::cocone

// One unit of work: the stepper may stop here and resume later.
#define SK_UNIT co_yield 0
// Run a long callee to its end, passing each of its units up.
#define SK_RUN(call)                                                                               \
  do {                                                                                             \
    ::cvc::cocone::step_task sk_sub_ = (call);                                                     \
    while (!sk_sub_.resume())                                                                      \
      co_yield 0;                                                                                  \
  } while (0)

#endif // CVC_GEOMETRY_SKELETON_STEPPING_H
