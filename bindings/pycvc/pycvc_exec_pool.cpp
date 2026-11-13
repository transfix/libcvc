// pycvc_exec_pool.cpp — pycvc.ExecPool (pycvc_exec_pool.h): pooled Execs, each an Exec in group
// mode (pycvc_exec.cpp), run in fork-join batches on the app's exec_pool (S1.12, S1-Q5).

// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec_pool.h"

#include "pycvc_exec.h"

#include <condition_variable>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state_exec/async_scheduler.h> // kernel_runs_here
#include <cvc/state/state_exec/exec_group.h>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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

} // namespace

struct ExecPool::ExecPoolImpl {
  std::shared_ptr<cvc::app> app; // keeps the app (and its pool) alive
  cvc::exec_pool *pool = nullptr;
  bool fresh_env = false;
  // The pooled Execs; empty once closed. Destroyed with the GIL held (~Exec releases it while
  // its kernels finish).
  std::vector<std::unique_ptr<Exec>> execs;

  // A run_all batch or a registration in progress (`busy`, on thread `user`), and close()'s
  // request to the batch (`closing`). Under `m`, which a thread takes with the GIL held or
  // without it, but never waits for the GIL while it holds: the GIL comes first.
  std::mutex m;
  std::condition_variable idle_cv;
  bool closed = false;
  bool busy = false;
  std::thread::id user;
  std::atomic<bool> closing{false};

  // Claim the pool for `what` (run_all, register_*): it is open and nothing else uses it. The
  // GIL is held.
  void claim(const char *what) {
    std::lock_guard<std::mutex> lock(m);
    if (closed)
      throw std::runtime_error(std::string("pycvc.ExecPool.") + what + ": the pool is closed");
    if (busy)
      throw std::runtime_error(std::string("pycvc.ExecPool.") + what +
                               ": a run_all or registration is in progress");
    busy = true;
    user = std::this_thread::get_id();
  }
  void release() noexcept {
    {
      std::lock_guard<std::mutex> lock(m);
      busy = false;
      user = std::thread::id();
    }
    idle_cv.notify_all();
  }
  // Releases the claim on every exit.
  struct claim_guard {
    ExecPoolImpl *p;
    ~claim_guard() { p->release(); }
  };
};

ExecPool::ExecPool(const std::shared_ptr<cvc::app> &app, PyObject *threads, PyObject *size,
                   const std::string &prefix, bool fresh_env, double idle_timeout) {
  if (!app)
    throw std::runtime_error("pycvc.ExecPool: null app handle");
  if (!(idle_timeout >= 0.0)) // NaN too
    throw std::invalid_argument(
        "pycvc.ExecPool: idle_timeout must be a number of seconds, 0 or more");
  if (prefix.empty())
    throw std::invalid_argument("pycvc.ExecPool: prefix must not be empty");
  auto impl = std::make_shared<ExecPoolImpl>();
  impl->app = app;
  const std::optional<unsigned> want_threads = optional_count(threads, 0, "threads");
  const std::optional<unsigned> want_size = optional_count(size, 1, "size");
  impl->fresh_env = fresh_env;
  // The app's one pool (S1-Q5): the first caller that builds it sets its size.
  if (want_threads) {
    cvc::pool_options opts;
    opts.threads = *want_threads;
    impl->pool = &app->exec_pool(opts);
  } else {
    impl->pool = &app->exec_pool();
  }
  // `size` defaults to `threads` (td 4.12): the pool's real count, which a request the pool did
  // not apply (it was built already, or clamped) does not change; at least 1 (R1-mid S-m11).
  const unsigned n = want_size ? *want_size : std::max(1u, impl->pool->threads());
  impl->execs.reserve(n);
  for (unsigned i = 0; i < n; ++i)
    impl->execs.push_back(std::unique_ptr<Exec>(
        new Exec(Exec::pooled_tag{}, app, prefix + "." + std::to_string(i), idle_timeout)));
  impl_ = std::move(impl);
}

ExecPool::~ExecPool() = default;

PyObject *ExecPool::run_all(PyObject *programs) {
  ExecPoolImpl &p = *impl_;
  // The programs as C++ strings, before the batch (the GIL held).
  PyObject *seq =
      PySequence_Fast(programs, "pycvc.ExecPool.run_all: programs must be a list of str");
  if (seq == nullptr)
    return nullptr; // TypeError
  std::vector<std::string> sources;
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  sources.reserve(static_cast<std::size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject *item = PySequence_Fast_GET_ITEM(seq, i);
    Py_ssize_t len = 0;
    const char *s = PyUnicode_Check(item) ? PyUnicode_AsUTF8AndSize(item, &len) : nullptr;
    if (s == nullptr) {
      Py_DECREF(seq);
      if (!PyErr_Occurred())
        PyErr_Format(PyExc_TypeError, "pycvc.ExecPool.run_all: program %zd is not a str", i);
      return nullptr;
    }
    sources.emplace_back(s, static_cast<std::size_t>(len));
  }
  Py_DECREF(seq);
  p.claim("run_all");
  std::vector<Exec::batch_outcome> out;
  {
    ExecPoolImpl::claim_guard claimed{&p};
    std::vector<Exec *> execs;
    for (const std::unique_ptr<Exec> &e : p.execs)
      execs.push_back(e.get());
    // The whole batch with the GIL released: a registered Python function takes it for its call,
    // so such calls serialize on it, and pure-DSL programs run in parallel.
    PyThreadState *ts = PyEval_SaveThread();
    struct gil_back {
      PyThreadState *ts;
      ~gil_back() { PyEval_RestoreThread(ts); }
    } restore{ts};
    out = Exec::run_batch(*p.pool, execs, sources, p.fresh_env, p.closing);
  }
  // The first failing program's error, in program order, once every program has ended.
  for (Exec::batch_outcome &o : out)
    if (o.error)
      std::rethrow_exception(o.error);
  PyObject *list = PyList_New(static_cast<Py_ssize_t>(out.size()));
  if (list == nullptr)
    return nullptr;
  for (std::size_t i = 0; i < out.size(); ++i) {
    PyObject *s = PyUnicode_FromStringAndSize(out[i].result.data(),
                                              static_cast<Py_ssize_t>(out[i].result.size()));
    if (s == nullptr) {
      Py_DECREF(list);
      return nullptr;
    }
    PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), s);
  }
  return list;
}

void ExecPool::register_fn(const std::string &name, PyObject *callable) {
  ExecPoolImpl &p = *impl_;
  p.claim("register_fn");
  ExecPoolImpl::claim_guard claimed{&p};
  for (const std::unique_ptr<Exec> &e : p.execs)
    e->register_fn(name, callable);
}

void ExecPool::register_async_fn(const std::string &name, PyObject *callable, bool with_call) {
  ExecPoolImpl &p = *impl_;
  p.claim("register_async_fn");
  ExecPoolImpl::claim_guard claimed{&p};
  for (const std::unique_ptr<Exec> &e : p.execs)
    e->register_async_fn(name, callable, with_call);
}

unsigned ExecPool::threads() const { return impl_->pool->threads(); }

void ExecPool::close() {
  ExecPoolImpl &p = *impl_;
  std::vector<std::unique_ptr<Exec>> gone;
  {
    std::unique_lock<std::mutex> lock(p.m);
    if (p.closed)
      return;
    if (p.busy) {
      // Not from inside the batch, which could not end while close() waits for it: run_all's
      // caller, a worker pumping one of its groups (a registered function), or a kernel of one.
      bool inside = p.user == std::this_thread::get_id();
      for (const std::unique_ptr<Exec> &e : p.execs)
        inside = inside || e->group().pumping_here() || e->group().sched().kernel_runs_here();
      if (inside)
        throw std::runtime_error("pycvc.ExecPool.close: called from inside its run_all");
      // A batch on another thread: what it runs is killed, and close() waits for it with the GIL
      // released, the mutex unlocked before the GIL is taken back.
      p.closing.store(true, std::memory_order_release);
      struct gil_released_until_unlocked {
        std::unique_lock<std::mutex> &lock;
        PyThreadState *ts = PyEval_SaveThread();
        ~gil_released_until_unlocked() {
          if (lock.owns_lock())
            lock.unlock();
          PyEval_RestoreThread(ts);
        }
      };
      {
        gil_released_until_unlocked nogil{lock};
        p.idle_cv.wait(lock, [&] { return !p.busy; });
      }
      lock.lock();
      if (p.closed)
        return; // another close() finished it meanwhile
    }
    p.closed = true;
    gone = std::move(p.execs);
  }
  // The pooled Execs' teardowns, the GIL held: each kills what is left, waits for its kernels
  // with the GIL released, and destroys its group.
  gone.clear();
}

} // namespace pycvc
