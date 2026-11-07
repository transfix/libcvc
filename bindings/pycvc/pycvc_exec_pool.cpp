// pycvc_exec_pool.cpp — pycvc.ExecPool (pycvc_exec_pool.h).
//
// Stage 1 interfaces: a stub. The constructor checks its arguments and builds (or finds) the app's
// pool, so `threads` and close() work; run_all and register_* raise RuntimeError. S1.12 is the
// pool of Execs.

// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec_pool.h"

#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace pycvc {

namespace {

// `o` as a whole number of at least `min`, or nullopt for None (or no argument). Anything else is
// a ValueError (std::invalid_argument).
std::optional<unsigned> optional_count(PyObject *o, long long min, const char *what) {
  if (o == nullptr || o == Py_None)
    return std::nullopt;
  const std::string bad = std::string("pycvc.ExecPool: ") + what + " must be None or an int, " +
                          std::to_string(min) + " or more";
  if (!PyLong_Check(o) || PyBool_Check(o))
    throw std::invalid_argument(bad);
  int overflow = 0;
  const long long v = PyLong_AsLongLongAndOverflow(o, &overflow);
  if (overflow != 0 || (v == -1 && PyErr_Occurred())) {
    PyErr_Clear();
    throw std::invalid_argument(bad);
  }
  if (v < min || v >= static_cast<long long>(std::numeric_limits<unsigned>::max()))
    throw std::invalid_argument(bad);
  return static_cast<unsigned>(v);
}

[[noreturn]] void not_built(const char *what) {
  throw std::logic_error(std::string("pycvc.ExecPool.") + what + ": not implemented yet");
}

} // namespace

struct ExecPool::ExecPoolImpl {
  std::shared_ptr<cvc::app> app; // keeps the app (and its pool) alive
  cvc::exec_pool *pool = nullptr;
  std::optional<unsigned> threads;
  std::optional<unsigned> size;
  std::string prefix;
  bool fresh_env = false;
  double idle_timeout = 60.0;
  bool closed = false;

  void check_open(const char *what) const {
    if (closed)
      throw std::runtime_error(std::string("pycvc.ExecPool.") + what + ": the pool is closed");
  }
};

ExecPool::ExecPool(const std::shared_ptr<cvc::app> &app, PyObject *threads, PyObject *size,
                   const std::string &prefix, bool fresh_env, double idle_timeout) {
  if (!app)
    throw std::runtime_error("pycvc.ExecPool: null app handle");
  if (!(idle_timeout >= 0.0)) // NaN too
    throw std::invalid_argument(
        "pycvc.ExecPool: idle_timeout must be a number of seconds, 0 or more");
  auto impl = std::make_shared<ExecPoolImpl>();
  impl->app = app;
  impl->threads = optional_count(threads, 0, "threads");
  impl->size = optional_count(size, 1, "size");
  impl->prefix = prefix;
  impl->fresh_env = fresh_env;
  impl->idle_timeout = idle_timeout;
  // The app's one pool (S1-Q5): the first caller that builds it sets its size.
  if (impl->threads) {
    cvc::pool_options opts;
    opts.threads = *impl->threads;
    impl->pool = &app->exec_pool(opts);
  } else {
    impl->pool = &app->exec_pool();
  }
  impl_ = std::move(impl);
}

ExecPool::~ExecPool() = default;

PyObject *ExecPool::run_all(PyObject *) {
  impl_->check_open("run_all");
  not_built("run_all");
}

void ExecPool::register_fn(const std::string &, PyObject *) {
  impl_->check_open("register_fn");
  not_built("register_fn");
}

void ExecPool::register_async_fn(const std::string &, PyObject *, bool) {
  impl_->check_open("register_async_fn");
  not_built("register_async_fn");
}

unsigned ExecPool::threads() const { return impl_->pool->threads(); }

void ExecPool::close() { impl_->closed = true; }

} // namespace pycvc
