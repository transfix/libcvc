// pycvc_exec.cpp — Exec implementation: run DSL programs in an app's context
// and bridge Python callables into the DSL as native_fn values.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec.h"

#include <stdexcept>

#ifdef CVC_STATE_EXEC

#include <chrono>
#include <condition_variable>
#include <cvc/core/app.h>
#include <cvc/core/async_task.h> // park_on_pool_task — offload an async Python fn to the pool
#include <cvc/core/state.h>
#include <cvc/core/state_exec/async_scheduler.h> // Exec drives a private async scheduler
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/types.h>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace pycvc {

namespace {
using namespace cvc::state_exec;

// ── value_t <-> Python marshaling ───────────────────────────────────────
// Convert a DSL value to a NEW Python reference. Scalars + list/dict cross;
// opaque DSL types (symbol/closure/native_fn/data/generator) degrade to their
// printed form so a Python fn still receives something sensible.
PyObject *value_to_py(const value_t &v) {
  if (v.is_nil())
    Py_RETURN_NONE;
  if (const auto *b = std::get_if<bool>(&v.v))
    return PyBool_FromLong(*b);
  if (const auto *i = std::get_if<int64_t>(&v.v))
    return PyLong_FromLongLong(static_cast<long long>(*i));
  if (const auto *d = std::get_if<double>(&v.v))
    return PyFloat_FromDouble(*d);
  if (const auto *s = std::get_if<std::string>(&v.v))
    return PyUnicode_FromStringAndSize(s->data(), static_cast<Py_ssize_t>(s->size()));
  if (const auto *l = std::get_if<list_ptr>(&v.v)) {
    const auto &vec = **l;
    PyObject *out = PyList_New(static_cast<Py_ssize_t>(vec.size()));
    for (Py_ssize_t k = 0; k < static_cast<Py_ssize_t>(vec.size()); ++k)
      PyList_SET_ITEM(out, k, value_to_py(vec[k])); // steals the new ref
    return out;
  }
  if (const auto *dd = std::get_if<dict_ptr>(&v.v)) {
    const auto &pairs = **dd;
    PyObject *out = PyDict_New();
    for (const auto &kv : pairs) {
      PyObject *pv = value_to_py(kv.second);
      PyDict_SetItemString(out, kv.first.c_str(), pv);
      Py_DECREF(pv);
    }
    return out;
  }
  // Fallback: the DSL's printed form.
  std::string s = to_string(v);
  return PyUnicode_FromStringAndSize(s.data(), static_cast<Py_ssize_t>(s.size()));
}

// Convert a Python object (borrowed) to a DSL value. bool BEFORE int (bool is a
// subclass of int in Python). dict order is preserved.
value_t py_to_value(PyObject *o) {
  if (o == nullptr || o == Py_None)
    return value_t(std::monostate{});
  if (PyBool_Check(o))
    return value_t(o == Py_True);
  if (PyLong_Check(o))
    return value_t(static_cast<int64_t>(PyLong_AsLongLong(o)));
  if (PyFloat_Check(o))
    return value_t(PyFloat_AsDouble(o));
  if (PyUnicode_Check(o)) {
    Py_ssize_t n = 0;
    const char *s = PyUnicode_AsUTF8AndSize(o, &n);
    return value_t(std::string(s ? s : "", s ? static_cast<size_t>(n) : 0));
  }
  if (PyList_Check(o) || PyTuple_Check(o)) {
    bool tup = PyTuple_Check(o);
    Py_ssize_t n = tup ? PyTuple_Size(o) : PyList_Size(o);
    std::vector<value_t> elems;
    elems.reserve(static_cast<size_t>(n));
    for (Py_ssize_t k = 0; k < n; ++k)
      elems.push_back(py_to_value(tup ? PyTuple_GetItem(o, k) : PyList_GetItem(o, k)));
    return value_t(std::make_shared<std::vector<value_t>>(std::move(elems)));
  }
  if (PyDict_Check(o)) {
    auto pairs = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
    PyObject *k = nullptr, *v = nullptr;
    Py_ssize_t pos = 0;
    while (PyDict_Next(o, &pos, &k, &v)) {
      PyObject *ks = PyObject_Str(k);
      Py_ssize_t n = 0;
      const char *s = ks ? PyUnicode_AsUTF8AndSize(ks, &n) : "";
      pairs->emplace_back(std::string(s ? s : "", s ? static_cast<size_t>(n) : 0), py_to_value(v));
      Py_XDECREF(ks);
    }
    return value_t(dict_ptr(std::move(pairs)));
  }
  // Anything else: its str().
  PyObject *s = PyObject_Str(o);
  Py_ssize_t n = 0;
  const char *cs = s ? PyUnicode_AsUTF8AndSize(s, &n) : "";
  value_t out(std::string(cs ? cs : "", cs ? static_cast<size_t>(n) : 0));
  Py_XDECREF(s);
  return out;
}

std::string fetch_py_error() {
  if (!PyErr_Occurred())
    return "unknown error";
  PyObject *type = nullptr, *val = nullptr, *tb = nullptr;
  PyErr_Fetch(&type, &val, &tb);
  PyErr_NormalizeException(&type, &val, &tb);
  std::string msg = "error";
  if (val) {
    PyObject *s = PyObject_Str(val);
    if (s) {
      const char *cs = PyUnicode_AsUTF8(s);
      if (cs)
        msg = cs;
      Py_DECREF(s);
    }
  }
  Py_XDECREF(type);
  Py_XDECREF(val);
  Py_XDECREF(tb);
  return msg;
}

// INCREF a callable and wrap it in a shared_ptr whose deleter DECREFs under the
// GIL, so it is owned for as long as the holder lives and torn down safely from
// any thread.
std::shared_ptr<PyObject> own_callable(PyObject *callable) {
  Py_INCREF(callable);
  return std::shared_ptr<PyObject>(callable, [](PyObject *p) {
    PyGILState_STATE g = PyGILState_Ensure();
    Py_DECREF(p);
    PyGILState_Release(g);
  });
}

// Call the held Python callable with `args`, ACQUIRING THE GIL for the duration.
// Safe from ANY thread (the scheduler thread for a sync fn, a pool worker for an
// async fn): PyGILState_Ensure creates/binds a thread state and blocks until the
// GIL is free, so the only rule is that whoever is idle-waiting for this worker
// must have released the GIL (run()'s pump does). Contains a Python exception as
// a C++ error carrying the message.
value_t call_python(const std::shared_ptr<PyObject> &holder, std::span<const value_t> args,
                    const std::string &name) {
  PyGILState_STATE gil = PyGILState_Ensure();
  PyObject *pyargs = PyTuple_New(static_cast<Py_ssize_t>(args.size()));
  for (Py_ssize_t i = 0; i < static_cast<Py_ssize_t>(args.size()); ++i)
    PyTuple_SET_ITEM(pyargs, i, value_to_py(args[i])); // steals the new ref
  PyObject *res = PyObject_CallObject(holder.get(), pyargs);
  Py_DECREF(pyargs);
  if (res == nullptr) {
    std::string msg = fetch_py_error();
    PyGILState_Release(gil);
    throw std::runtime_error("python DSL fn '" + name + "': " + msg);
  }
  value_t out = py_to_value(res);
  Py_DECREF(res);
  PyGILState_Release(gil);
  return out;
}

// Wrap a Python callable as a SYNCHRONOUS DSL native_fn: it runs inline on the
// scheduler thread (GIL held by run()).
native_fn make_python_native_fn(const std::string &name, PyObject *callable) {
  std::shared_ptr<PyObject> holder = own_callable(callable);
  return [name, holder](std::span<const value_t> args) -> value_t {
    return call_python(holder, args, name);
  };
}

std::string render_result(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (v.is_nil())
    return "";
  return to_string(v);
}

// Counts the async pool tasks a run() has in flight so no worker (which posts back to this Exec's
// private scheduler, and may touch Python) can outlive run()/the Exec. `begin()` is called on the
// scheduler thread at offload; `end()` runs on the pool worker the instant its Python body returns
// (an EndGuard, below), BEFORE the marshalled result is posted — so the count never depends on when
// the pool happens to destroy the completed task. wait_idle() blocks — WITH THE GIL RELEASED so a
// GIL-holding worker can finish — until the count reaches zero. wait_idle()'s caller must hold the
// GIL (run()/~Exec do).
struct Inflight {
  std::mutex m;
  std::condition_variable cv;
  int count = 0;
  void begin() {
    std::lock_guard<std::mutex> l(m);
    ++count;
  }
  void end() {
    {
      std::lock_guard<std::mutex> l(m);
      --count;
    }
    cv.notify_all();
  }
  int pending() {
    std::lock_guard<std::mutex> l(m);
    return count;
  }
  void wait_idle() {
    std::unique_lock<std::mutex> l(m);
    if (count == 0)
      return;
    Py_BEGIN_ALLOW_THREADS cv.wait(l, [this] { return count == 0; });
    Py_END_ALLOW_THREADS
  }
};

// RAII on the WORKER: end() the moment the work body returns or throws (before launch_pool_task
// marshals/posts), so the in-flight count is decoupled from the pool's task-object lifetime.
struct EndGuard {
  std::shared_ptr<Inflight> f;
  ~EndGuard() { f->end(); }
};
} // namespace

// Owns everything an evaluation needs. The intrinsics capture &ictx (a raw
// pointer), so ictx must outlive any run — holding it here (Exec-lifetime)
// satisfies that. Mirrors the C++ tests' make_exec_env().
struct Exec::ExecImpl {
  std::shared_ptr<cvc::app> app; // co-own so root/ictx stay valid for our lifetime
  cvc::state *root = nullptr;
  // A PRIVATE async_scheduler owned by this Exec (NOT the app-wide one): it supports park/await +
  // cross-thread delivery, and isolating it means a pycvc program never races an Ariadne Runtime
  // (or another Exec) driving a shared scheduler, and its finished processes die with the Exec. An
  // async fn offloaded to the compute pool posts back to THIS scheduler (register_async_fn passes
  // it), so the parked program resumes here.
  async_scheduler sched;
  intrinsics_context ictx;
  process_ptr host_proc;
  environment_ptr env;
  std::shared_ptr<Inflight> inflight = std::make_shared<Inflight>();
};

Exec::Exec(const std::shared_ptr<cvc::app> &app) : impl_(std::make_shared<ExecImpl>()) {
  if (!app)
    throw std::invalid_argument("pycvc.Exec: null app handle");
  impl_->app = app; // keep the app (and its compute pool) alive for as long as this Exec exists
  impl_->root = &cvc::state::instance(*app);
  // Warm the compute pool now, on this thread, before any worker touches it. The pool is the app's;
  // only the scheduler is private.
  app->computePool();
  impl_->sched.set_watch_root(impl_->root);

  impl_->host_proc = make_process();
  impl_->host_proc->pid = 0;
  impl_->host_proc->status = process_status::ready;

  impl_->ictx.sched = &impl_->sched;
  impl_->ictx.root = impl_->root;
  impl_->ictx.proc = impl_->host_proc;
  impl_->ictx.pid = 0;
  impl_->ictx.uid = "pycvc";
  impl_->ictx.cluster_id = "local";
  impl_->ictx.node_id = "local";

  impl_->env = builtins::make_default_environment();
  register_intrinsics(impl_->env, &impl_->ictx);
}

Exec::~Exec() {
  // No pool worker may outlive the private scheduler it posts to: quiesce before ExecImpl (and its
  // scheduler) tear down. run() already drains on every exit, so this is normally a no-op.
  if (impl_ && impl_->inflight)
    impl_->inflight->wait_idle();
}

void Exec::register_fn(const std::string &name, PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument("pycvc.Exec.register_fn: '" + name + "' is not callable");
  builtins::register_fn(impl_->env, name, make_python_native_fn(name, callable));
}

void Exec::register_async_fn(const std::string &name, PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument("pycvc.Exec.register_async_fn: '" + name + "' is not callable");
  std::shared_ptr<PyObject> holder = own_callable(callable);
  cvc::app *app = impl_->app.get();
  async_scheduler *sched = &impl_->sched; // this Exec's PRIVATE scheduler (the offload posts here)
  std::shared_ptr<Inflight> inflight = impl_->inflight;
  const std::string root = impl_->ictx.root_path; // "" (full-tree; the '#' channel is identity)
  // (name args...) marshals its args on the scheduler thread, offloads the Python call to a pool
  // worker via park_on_pool_task, and SELF-PARKS until the worker delivers the return value. The
  // args are COPIED into the worker closure (the span is only valid during this call). An
  // InflightToken rides ALONG with the worker closure so run()/~Exec can wait out the worker before
  // tearing down. A Python exception becomes a {"__async_error__": <message>} dict — a reserved key
  // that a plain dict result cannot collide with — so a waiter still resumes and can tell an error
  // from data.
  builtins::register_fn(
      impl_->env, name,
      [app, sched, root, holder, name, inflight](std::span<const value_t> args) -> value_t {
        auto argv = std::make_shared<std::vector<value_t>>(args.begin(), args.end());
        inflight
            ->begin(); // ++in-flight NOW (on the scheduler thread) so run() sees it before parking
        return cvc::park_on_pool_task(
            *app, *sched, root, name + ".reply",
            [holder, argv, name, inflight]() -> value_t {
              EndGuard guard{
                  inflight}; // --in-flight when this body returns/throws, before the post
              return call_python(holder, *argv, name);
            },
            [](const std::string &e) -> value_t {
              auto pairs = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
              pairs->emplace_back("__async_error__", value_t(e));
              return value_t(dict_ptr(std::move(pairs)));
            });
      });
}

std::string Exec::run(const std::string &src) {
  async_scheduler &sched = impl_->sched;
  Inflight &inflight = *impl_->inflight;
  execute_options opts;
  opts.name = "pycvc";
  opts.env = impl_->env;
  const int pid = sched.execute(src, opts); // may throw on parse error (nothing to clean up yet)

  // Drive the private scheduler to completion. A purely synchronous program finishes in the first
  // slice; a program that parks (await / an async fn) yields, and the pump waits for a compute-pool
  // worker to post back, then resumes it. Slices are bounded so a runaway never-parking loop can't
  // wedge the pump.
  static constexpr uint64_t kSliceSteps = 2000000;
  static constexpr double kSliceSeconds = 0.05;
  static constexpr double kIdleTimeoutSecs = 60.0; // parked with NO pending work this long -> bail
  using clock = std::chrono::steady_clock;
  auto idle_deadline = clock::now() + std::chrono::duration<double>(kIdleTimeoutSecs);
  std::optional<value_t> result;
  std::optional<std::string> error; // set -> throw AFTER the pool is quiesced
  try {
    while (true) {
      sched.drain_ingress(); // deliver any worker post_message FIRST, so a parked receiver readies
      sched.wake_awaiting(); // re-ready (await …)-parked processes (one frame per drain)
      const bool had_runnable = sched.has_runnable();
      // Release the GIL AROUND the slice: a synchronous Python native_fn stepped here re-acquires
      // it via call_python's PyGILState_Ensure, while an in-flight pool worker running a Python
      // async body is never starved of the GIL by a co-runnable process — the deadlock the review
      // found.
      Py_BEGIN_ALLOW_THREADS sched.sync_run(kSliceSteps, kSliceSeconds);
      Py_END_ALLOW_THREADS result = sched.get_result(pid);
      if (result)
        break; // the program terminated
      const std::optional<process_info> info = sched.get_process_info(pid);
      if (!info) {
        error = "pycvc.Exec.run: process vanished (pid " + std::to_string(pid) + ")";
        break;
      }
      if (info->status == process_status::killed) {
        // Surface the real failure reason (a thrown Python fn's message, or a resource-limit tag)
        // rather than a generic "killed" — the async scheduler contains a native_fn throw as a
        // kill.
        const std::optional<std::string> ex = sched.get_exit_error(pid);
        error = (ex && !ex->empty())
                    ? *ex
                    : ("pycvc.Exec.run: program killed (pid " + std::to_string(pid) + ")");
        break;
      }
      // Fast-path only on OUR OWN pid's runnability — has_runnable() scans siblings too, so gating
      // on it would spin (holding the GIL) while our program is actually parked awaiting a worker.
      if (info->status == process_status::ready || info->status == process_status::running) {
        idle_deadline = clock::now() + std::chrono::duration<double>(kIdleTimeoutSecs);
        continue; // our program has more to run — loop (the next slice releases the GIL again)
      }
      // Our pid is parked (waiting). A pool worker still in flight, or any sibling progress, means
      // a delivery is coming, so keep the idle clock fresh; only a park with NOTHING pending can
      // time out (e.g. a msg-recv on a channel nobody sends to) — safe to abandon then, no worker
      // to outlive us.
      if (inflight.pending() > 0 || sched.has_runnable() || had_runnable) {
        idle_deadline = clock::now() + std::chrono::duration<double>(kIdleTimeoutSecs);
      } else if (clock::now() > idle_deadline) {
        sched.kill(pid);
        error = "pycvc.Exec.run: parked with no pending work (pid " + std::to_string(pid) + ")";
        break;
      }
      // Idle-wait a beat with the GIL released so a worker can acquire it and deliver.
      Py_BEGIN_ALLOW_THREADS std::this_thread::sleep_for(std::chrono::milliseconds(1));
      Py_END_ALLOW_THREADS
    }
  } catch (...) {
    inflight.wait_idle(); // never leave a worker touching our (about-to-unwind) scheduler
    sched.kill(pid);
    sched.reap(pid);
    throw;
  }
  // No pool worker may outlive run(): quiesce (GIL released) BEFORE we reap the process or return
  // to Python, so a worker can never post to / be destroyed against a torn-down scheduler.
  inflight.wait_idle();
  sched.reap(pid); // drop the finished/killed process from the private scheduler (no leak)
  if (error)
    throw std::runtime_error(*error);
  if (!result.has_value())
    throw std::runtime_error("pycvc.Exec.run: program produced no result (pid " +
                             std::to_string(pid) + ")");
  return render_result(*result);
}

} // namespace pycvc

#else // !CVC_STATE_EXEC

namespace pycvc {

struct Exec::ExecImpl {};

Exec::Exec(const std::shared_ptr<cvc::app> &) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}
Exec::~Exec() = default;
void Exec::register_fn(const std::string &, PyObject *) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}
std::string Exec::run(const std::string &) {
  throw std::runtime_error("pycvc.Exec: this libcvc build was compiled without state_exec");
}

} // namespace pycvc

#endif // CVC_STATE_EXEC
