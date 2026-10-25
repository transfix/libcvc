// pycvc_exec.cpp — Exec implementation: run DSL programs in an app's context
// and bridge Python callables into the DSL as native_fn values.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec.h"

#include "pycvc_value.h" // the shared value_t <-> Python conversion

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>      // Exec's async functions run through its host
#include <cvc/state/state_exec/async_scheduler.h> // Exec drives a private async scheduler
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/types.h>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

namespace pycvc {

namespace {
using namespace cvc::state_exec;

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

// Best-effort: is `fn` an `async def` (a coroutine function)? Its code object carries the
// CO_COROUTINE flag. A plain async def is caught; a decorated/wrapped one may slip through. Used to
// steer an async def away from the sync/pool paths (where its un-awaited coroutine would marshal as
// a nonsense repr) and toward register_async_coro.
bool is_coroutine_function(PyObject *fn) {
  PyObject *code = PyObject_GetAttrString(fn, "__code__");
  if (code == nullptr) {
    PyErr_Clear();
    return false;
  }
  bool coro = false;
  if (PyObject *flags = PyObject_GetAttrString(code, "co_flags")) {
    coro = (PyLong_AsLong(flags) & 0x0080) != 0; // CO_COROUTINE
    Py_DECREF(flags);
  } else {
    PyErr_Clear();
  }
  Py_DECREF(code);
  return coro;
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

// The qualified name of an exception type: "module.QualName", or the bare name of a builtin.
std::string exception_type_name(PyObject *type) {
  std::string name;
  if (PyObject *qual = PyObject_GetAttrString(type, "__qualname__")) {
    if (const char *s = PyUnicode_AsUTF8(qual))
      name = s;
    Py_DECREF(qual);
  }
  if (PyObject *mod = PyObject_GetAttrString(type, "__module__")) {
    const char *m = PyUnicode_Check(mod) ? PyUnicode_AsUTF8(mod) : nullptr;
    if (m && std::string(m) != "builtins" && !name.empty())
      name = std::string(m) + "." + name;
    Py_DECREF(mod);
  }
  PyErr_Clear(); // a type without the attributes keeps what was found
  return name;
}

// The Python exception set now, taken (cleared) as the C++ error it becomes in the DSL: a
// python_exception (PythonError) holding it, its message prefixed with `what`.
python_exception take_py_exception(const std::string &what) {
  PyObject *type = nullptr, *val = nullptr, *tb = nullptr;
  PyErr_Fetch(&type, &val, &tb);
  PyErr_NormalizeException(&type, &val, &tb);
  if (val != nullptr && tb != nullptr)
    PyException_SetTraceback(val, tb);
  std::string msg = "error";
  if (val != nullptr) {
    if (PyObject *s = PyObject_Str(val)) {
      if (const char *cs = PyUnicode_AsUTF8(s))
        msg = cs;
      Py_DECREF(s);
    }
  }
  const std::string type_name = type != nullptr ? exception_type_name(type) : std::string();
  std::shared_ptr<PyObject> held = val != nullptr ? own_callable(val) : nullptr;
  PyErr_Clear();
  Py_XDECREF(type);
  Py_XDECREF(val);
  Py_XDECREF(tb);
  return python_exception(what + ": " + msg, type_name, std::move(held));
}

// Call the held Python callable with `args`, ACQUIRING THE GIL for the duration.
// Safe from ANY thread (the scheduler thread for a sync fn, a pool worker for an
// async fn): PyGILState_Ensure creates/binds a thread state and blocks until the
// GIL is free, so the only rule is that whoever is idle-waiting for this worker
// must have released the GIL (run()'s pump does). A Python exception becomes a
// python_exception (a PythonError in the DSL, docs/STATE_EXEC_EXCEPTIONS.md),
// its message prefixed with `what`. With `want_result` false the return value
// is ignored (an on_complete callback, whose exception stays a runtime_error
// thrown from the scheduler call) and nil returned. For an async function's kernel,
// `call` is its call: the wait for the GIL isn't charged as its run time (3.8).
value_t call_python(const std::shared_ptr<PyObject> &holder, std::span<const value_t> args,
                    const std::string &what, bool want_result = true, async_call *call = nullptr) {
  const auto wait_start = std::chrono::steady_clock::now();
  PyGILState_STATE gil = PyGILState_Ensure();
  if (call != nullptr)
    call->add_wait(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count());
  PyObject *pyargs = values_to_py_tuple(args); // nullptr (error set) if an arg can't convert
  PyObject *res = pyargs ? PyObject_CallObject(holder.get(), pyargs) : nullptr;
  Py_XDECREF(pyargs);
  if (res == nullptr) {
    if (!want_result) {
      std::string msg = fetch_py_error();
      PyGILState_Release(gil);
      throw std::runtime_error(what + ": " + msg);
    }
    python_exception error = take_py_exception(what);
    PyGILState_Release(gil);
    throw error;
  }
  value_t out;
  // strict: e.g. a set or non-str dict key fails
  const bool converted = !want_result || py_to_value(res, out);
  Py_DECREF(res);
  if (!converted) {
    std::string msg = fetch_py_error();
    PyGILState_Release(gil);
    throw std::runtime_error(what + " returned an unconvertible value: " + msg);
  }
  PyGILState_Release(gil);
  return out;
}

std::string render_result(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  if (v.is_nil())
    return "";
  return to_string(v);
}

// The C++ error a Python exception object `exc` (not the one set now) becomes in the DSL: a
// python_exception (PythonError) holding it, its message prefixed with `what`. The GIL is held.
std::exception_ptr python_error_of(PyObject *exc, const std::string &what) {
  std::string msg = "error";
  if (PyObject *s = PyObject_Str(exc)) {
    if (const char *cs = PyUnicode_AsUTF8(s))
      msg = cs;
    Py_DECREF(s);
  }
  PyErr_Clear();
  return std::make_exception_ptr(python_exception(
      what + ": " + msg, exception_type_name(reinterpret_cast<PyObject *>(Py_TYPE(exc))),
      own_callable(exc)));
}

// Settle an external call (a coroutine's) with what it raised and post its outcome.
void fail_external(async_host &host, const std::shared_ptr<async_call> &call,
                   std::exception_ptr error) {
  call->settle({}, std::move(error));
  host.finish(call);
}

// A coroutine an async coroutine function's call is waiting for: its asyncio.Task (owned) and
// the call it settles.
struct pending_coro {
  PyObject *task;
  std::shared_ptr<async_call> call;
};

// Step the Exec's asyncio loop for a BOUNDED wall-time slice, then settle every completed
// coroutine's call with its result (or its exception, which the call raises at its call site) and
// post it to the parked DSL process. Runs on the run() thread with the GIL held; a no-op when no
// coroutine is pending, so a coro-free Exec pays nothing. Each Task is DECREF'd when collected.
void drive_coros(PyObject *loop, std::vector<pending_coro> &pending, async_host &host,
                 double budget_secs) {
  if (loop == nullptr || pending.empty())
    return;
  // Step the loop until a call_later(budget) stops it: ready coroutine steps + timers/IO due within
  // the slice advance, then control returns to the DSL pump. Public asyncio API only. NOTE: the
  // slice is COOPERATIVE — asyncio cannot preempt a running coroutine step, so the budget is
  // honored only at the coroutine's await points. A coroutine that never awaits (CPU-bound work, an
  // accidental (while True) with no await) holds this thread inside run_forever and wedges the
  // whole pump — put blocking/CPU work in register_async_fn (the pool), not register_async_coro.
  bool step_failed = true;
  if (PyObject *stop = PyObject_GetAttrString(loop, "stop")) {
    PyObject *h = PyObject_CallMethod(loop, "call_later", "dO", budget_secs, stop);
    Py_DECREF(stop);
    if (h != nullptr) {
      Py_DECREF(h);
      // run_forever ONLY when a stop timer is armed — otherwise it would never return.
      if (PyObject *r = PyObject_CallMethod(loop, "run_forever", nullptr)) {
        Py_DECREF(r);
        step_failed = false;
      }
    }
  }
  if (step_failed) {
    // The loop cannot be stepped (call_later/run_forever/getattr failed): the pending coroutines
    // can never advance, so fail them all rather than clearing the error and spinning an unkillable
    // pump.
    const std::string msg = "python coro: event loop step failed: " + fetch_py_error();
    for (pending_coro &pc : pending) {
      fail_external(host, pc.call, std::make_exception_ptr(std::runtime_error(msg)));
      Py_DECREF(pc.task);
    }
    pending.clear();
    return;
  }
  PyErr_Clear(); // a stray (already-handled) error from stepping must not leak into the next call
  for (auto it = pending.begin(); it != pending.end();) {
    PyObject *task = it->task;
    PyObject *done_obj = PyObject_CallMethod(task, "done", nullptr);
    const bool done = done_obj != nullptr && PyObject_IsTrue(done_obj) == 1;
    Py_XDECREF(done_obj);
    if (!done) {
      ++it;
      continue;
    }
    const std::string what = "python coro fn '" + it->call->function_name() + "'";
    PyObject *exc = PyObject_CallMethod(task, "exception", nullptr); // None if it returned normally
    if (exc == nullptr) {
      fail_external(host, it->call, std::make_exception_ptr(take_py_exception(what)));
    } else if (exc == Py_None) {
      Py_DECREF(exc);
      PyObject *res = PyObject_CallMethod(task, "result", nullptr);
      value_t converted;
      if (res != nullptr && py_to_value(res, converted)) {
        it->call->settle(std::move(converted), nullptr);
        host.finish(it->call);
      } else {
        fail_external(host, it->call,
                      std::make_exception_ptr(std::runtime_error(
                          what + " returned an unconvertible value: " + fetch_py_error())));
      }
      Py_XDECREF(res);
    } else {
      fail_external(host, it->call, python_error_of(exc, what));
      Py_DECREF(exc);
    }
    Py_DECREF(task);
    it = pending.erase(it);
  }
}

// Wait until no kernel of `host` is queued or running, WITH THE GIL RELEASED so a kernel calling
// into Python can finish. The caller holds the GIL (run()/~Exec do).
void wait_idle(async_host &host) {
  if (host.in_flight() == 0)
    return;
  Py_BEGIN_ALLOW_THREADS host.wait_idle();
  Py_END_ALLOW_THREADS
}
} // namespace

// A SYNCHRONOUS DSL native_fn: it runs inline on the thread stepping the scheduler (Exec::run's,
// or the caller of a wrapped scheduler's run/step, which release the GIL; call_python retakes it).
native_fn python_native_fn(const std::string &who, const std::string &name, PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument(who + ": '" + name + "' is not callable");
  if (is_coroutine_function(callable))
    throw std::invalid_argument(who + ": '" + name +
                                "' is an async def — use Exec.register_async_coro");
  std::shared_ptr<PyObject> holder = own_callable(callable);
  const std::string what = "python DSL fn '" + name + "'";
  return [what, holder](std::span<const value_t> args) -> value_t {
    return call_python(holder, args, what);
  };
}

void python_on_complete::operator()(value_t result) const {
  call_python(callable, std::span<const value_t>(&result, 1), "python on_complete",
              /*want_result=*/false);
}

async_marshal python_async_marshal(const std::string &who, const std::string &name,
                                   PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument(who + ": '" + name + "' is not callable");
  if (is_coroutine_function(callable))
    throw std::invalid_argument(who + ": '" + name +
                                "' is an async def — use register_async_coro (register_async_fn is "
                                "for a blocking callable run on an executor thread)");
  std::shared_ptr<PyObject> holder = own_callable(callable);
  const std::string what = "python DSL fn '" + name + "'";
  // The arguments are private copies already (register_async_fn copies lists and dicts), so the
  // kernel keeps them; it converts them to Python on its own thread, with the GIL.
  return [holder, what](std::span<const value_t> args) -> async_kernel {
    return [holder, what,
            argv = std::vector<value_t>(args.begin(), args.end())](async_call &call) -> value_t {
      return call_python(holder, argv, what, true, &call);
    };
  };
}

python_on_complete make_python_on_complete(PyObject *callable) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument("pycvc: on_complete is not callable");
  return python_on_complete{own_callable(callable)};
}

std::shared_ptr<PyObject> python_ref(PyObject *o) { return own_callable(o); }

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
  // The async host of this Exec's async functions (docs/STATE_EXEC_EXCEPTIONS.md 3): kernels run
  // on the app's background task pool and post back to `sched`. After `sched`, so it goes first;
  // ~Exec shuts it down before that, with the GIL released.
  std::unique_ptr<cvc::state_exec::async_host> host;
  // Coroutine support (register_async_coro): a private asyncio loop (lazily created; owned) and the
  // coroutines in flight, which the pump steps + collects. Touched only on the run() thread (the
  // verb during sync_run + drive_coros after), so no lock needed.
  PyObject *loop = nullptr;
  std::vector<pending_coro> pending_coros;
  bool running = false; // re-entrancy guard: a registered fn/coro must not call back into run()
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
  // The same state_exec defaults as the app's exec scheduler, so a program behaves the same from
  // Python as from Ariadne: scripts are read inside the process (host_parser_defaults) and deep
  // data is handled a slice per step (host_data_defaults), and the app tree's
  // state_exec.defaults.parser.* / data.* keys override them for both.
  impl_->sched.set_parser_defaults(async_scheduler::host_parser_defaults());
  impl_->sched.set_data_defaults(async_scheduler::host_data_defaults());

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
  impl_->host = std::make_unique<cvc::state_exec::async_host>(impl_->sched, app.get());
}

Exec::~Exec() {
  // No kernel may outlive the private scheduler it posts to: cancel what is in flight and wait for
  // it, with the GIL released (a kernel may be calling into Python). run() already waits on every
  // exit, so this normally has nothing to wait for.
  if (impl_ && impl_->host) {
    Py_BEGIN_ALLOW_THREADS impl_->host->shutdown();
    Py_END_ALLOW_THREADS
  }
  // Cancel + drop any un-collected coroutine Tasks and close the asyncio loop (GIL held: ~Exec is
  // called from Python via SWIG).
  if (impl_ && impl_->loop) {
    PyGILState_STATE gil = PyGILState_Ensure();
    for (pending_coro &pc : impl_->pending_coros) {
      if (PyObject *r = PyObject_CallMethod(pc.task, "cancel", nullptr))
        Py_DECREF(r);
      Py_DECREF(pc.task);
    }
    impl_->pending_coros.clear();
    if (PyObject *r = PyObject_CallMethod(impl_->loop, "close", nullptr))
      Py_DECREF(r);
    Py_DECREF(impl_->loop);
    impl_->loop = nullptr;
    PyErr_Clear();
    PyGILState_Release(gil);
  }
}

void Exec::register_fn(const std::string &name, PyObject *callable) {
  builtins::register_fn(impl_->env, name,
                        python_native_fn("pycvc.Exec.register_fn", name, callable));
}

void Exec::register_async_fn(const std::string &name, PyObject *callable) {
  // An async function on this Exec's host (docs/STATE_EXEC_EXCEPTIONS.md 3): (name args...) parks
  // the program while the callable runs on the app's background task pool, then resumes it with
  // the value or raises the callable's exception at the call (a PythonError). (name-async ...)
  // returns a future.
  cvc::state_exec::register_async_fn(
      impl_->env, *impl_->host, name,
      python_async_marshal("pycvc.Exec.register_async_fn", name, callable));
}

cvc::state_exec::async_host &Exec::async_host() { return *impl_->host; }

void Exec::register_async_coro(const std::string &name, PyObject *coro_fn) {
  if (coro_fn == nullptr || !PyCallable_Check(coro_fn))
    throw std::invalid_argument("pycvc.Exec.register_async_coro: '" + name + "' is not callable");
  // Lazily create the asyncio loop this Exec drives (once, at setup, GIL held by the caller).
  if (impl_->loop == nullptr) {
    PyObject *asyncio = PyImport_ImportModule("asyncio");
    if (asyncio == nullptr)
      throw std::runtime_error("pycvc.Exec.register_async_coro: cannot import asyncio: " +
                               fetch_py_error());
    impl_->loop = PyObject_CallMethod(asyncio, "new_event_loop", nullptr);
    Py_DECREF(asyncio);
    if (impl_->loop == nullptr)
      throw std::runtime_error("pycvc.Exec.register_async_coro: new_event_loop failed: " +
                               fetch_py_error());
  }
  std::shared_ptr<PyObject> holder = own_callable(coro_fn);
  ExecImpl *impl = impl_.get();
  // (name args...) calls coro_fn(*args) -> a coroutine, schedules it as a Task on the loop, and
  // parks the program at the call (an external call of the Exec's host, which no executor runs).
  // run()'s pump steps the loop (drive_coros) and settles the call with the Task's result or its
  // exception, which raises at the call. Its arguments are copied first, as for any async function.
  const auto start = [impl, holder, name](std::span<const value_t> args, bool future) -> value_t {
    value_t result;
    const std::shared_ptr<async_call> call = impl->host->begin_external(name, future, result);
    const std::string what = "python coro fn '" + name + "'";
    // The verb runs during sync_run (GIL released by the pump) — re-acquire it to touch Python.
    PyGILState_STATE gil = PyGILState_Ensure();
    PyObject *pyargs = values_to_py_tuple(args); // nullptr (error set) on an unconvertible arg
    PyObject *coro = pyargs ? PyObject_CallObject(holder.get(), pyargs) : nullptr;
    Py_XDECREF(pyargs);
    PyObject *task = nullptr;
    if (coro != nullptr) {
      task = PyObject_CallMethod(impl->loop, "create_task", "O", coro);
      Py_DECREF(coro);
    }
    if (task == nullptr) {
      // Nothing to await: the call raises the error once the next drain delivers it.
      std::exception_ptr error = std::make_exception_ptr(take_py_exception(what));
      PyGILState_Release(gil);
      fail_external(*impl->host, call, std::move(error));
    } else {
      impl->pending_coros.push_back({task, call}); // owns the Task ref until collected
      PyGILState_Release(gil);
    }
    return result;
  };
  for (const bool future : {false, true})
    builtins::register_fn(impl_->env, future ? name + "-async" : name,
                          [start, future](std::span<const value_t> args) -> value_t {
                            value_t argv =
                                make_list(std::vector<value_t>(args.begin(), args.end()));
                            auto then = [start, future](value_t copied) -> value_t {
                              const std::vector<value_t> &items = *std::get<list_ptr>(copied.v);
                              return start(std::span<const value_t>(items), future);
                            };
                            if (std::none_of(args.begin(), args.end(),
                                             [](const value_t &a) { return is_compound(a); }))
                              return then(std::move(argv));
                            return run_data_job(make_copy_job(std::move(argv)), std::move(then));
                          });
}

std::string Exec::run(const std::string &src) {
  // One Exec is driven by ONE thread. A registered fn/coro that calls back into run() (or a second
  // thread) would corrupt the shared scheduler + asyncio loop, so reject re-entrancy up front.
  if (impl_->running)
    throw std::runtime_error("pycvc.Exec.run: re-entrant call — a registered fn/coro must not call "
                             "run() (use a separate Exec per thread)");
  impl_->running = true;
  // On EVERY exit: clear the running flag, and cancel+drop any coroutine Tasks still pending for
  // this run (e.g. an orphaned coroutine of a sibling process that never completed) so it can't
  // suppress the NEXT run()'s idle timeout. PyGILState_Ensure re-acquires the GIL on whatever path
  // we unwind.
  struct RunGuard {
    ExecImpl *impl;
    ~RunGuard() {
      if (!impl->pending_coros.empty()) {
        PyGILState_STATE gil = PyGILState_Ensure();
        for (pending_coro &pc : impl->pending_coros) {
          if (PyObject *r = PyObject_CallMethod(pc.task, "cancel", nullptr))
            Py_DECREF(r);
          Py_DECREF(pc.task);
        }
        impl->pending_coros.clear();
        PyErr_Clear();
        PyGILState_Release(gil);
      }
      impl->running = false;
    }
  } run_guard{impl_.get()};

  async_scheduler &sched = impl_->sched;
  cvc::state_exec::async_host &host = *impl_->host;
  execute_options opts;
  opts.name = "pycvc";
  opts.env = impl_->env;
  // A syntax error is read inside the process (parser.step_tokens) and surfaces below as the
  // killed process's exit error; with step_tokens = 0 on the tree, execute() throws it here
  // instead (nothing to clean up yet). Either way run() raises with the parse error's message.
  const int pid = sched.execute(src, opts);

  // Drive the private scheduler to completion. A purely synchronous program finishes in the first
  // slice; a program that parks (await / an async fn) yields, and the pump waits for a kernel to
  // post back, then resumes it. Slices are bounded so a runaway never-parking loop can't wedge the
  // pump. Each drain also charges the async calls still running to their processes (3.8).
  static constexpr uint64_t kSliceSteps = 2000000;
  static constexpr double kSliceSeconds = 0.05;
  static constexpr double kCoroSliceSeconds = 0.005; // asyncio-loop slice per pump iteration
  static constexpr double kIdleTimeoutSecs = 60.0; // parked with NO pending work this long -> bail
  using clock = std::chrono::steady_clock;
  auto idle_deadline = clock::now() + std::chrono::duration<double>(kIdleTimeoutSecs);
  std::optional<value_t> result;
  std::optional<std::string> error; // set -> throw AFTER the pool is quiesced
  bool killed = false;              // the scheduler killed the process (error has its exit error)
  std::exception_ptr failure; // the program failed: its dsl_exception, thrown after the quiesce
  try {
    while (true) {
      sched.drain_ingress(); // deliver kernels' results and posts FIRST, so a parked caller readies
      sched.wake_awaiting(); // re-ready (await …)-parked processes (one frame per drain)
      const bool had_runnable = sched.has_runnable();
      // Release the GIL AROUND the slice: a synchronous Python native_fn stepped here re-acquires
      // it via call_python's PyGILState_Ensure, while an in-flight pool worker running a Python
      // async body is never starved of the GIL by a co-runnable process — the deadlock the review
      // found.
      Py_BEGIN_ALLOW_THREADS sched.sync_run(kSliceSteps, kSliceSeconds);
      Py_END_ALLOW_THREADS
          // Coroutine leg: step the asyncio loop a bounded slice and deliver any completed
          // coroutine's result to its parked process (GIL held here). A no-op when no coroutine is
          // pending, so a coro-free Exec is unaffected.
          drive_coros(impl_->loop, impl_->pending_coros, host, kCoroSliceSeconds);
      result = sched.get_result(pid);
      if (result)
        break; // the program terminated
      const std::optional<process_info> info = sched.get_process_info(pid);
      if (!info) {
        error = "pycvc.Exec.run: process vanished (pid " + std::to_string(pid) + ")";
        break;
      }
      if (info->status == process_status::failed) {
        // An exception the program didn't catch (docs/STATE_EXEC_EXCEPTIONS.md 2.6): raised as
        // pycvc.dsl_error, or as the Python function's own exception that it was made from.
        try {
          sched.raise_exit_exception(pid);
        } catch (const dsl_exception &) {
          failure = std::current_exception();
        }
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
        killed = true;
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
      if (host.in_flight() > 0 || !impl_->pending_coros.empty() || sched.has_runnable() ||
          had_runnable) {
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
    sched.kill(pid); // cancels its calls in flight
    wait_idle(host); // never leave a kernel touching our (about-to-unwind) scheduler
    sched.reap(pid);
    throw;
  }
  // No kernel of this run may outlive it: wait (GIL released) BEFORE we reap the process or return
  // to Python, so a kernel can never post to / be destroyed against a torn-down scheduler.
  wait_idle(host);
  sched.reap(pid); // drop the finished/killed process from the private scheduler (no leak)
  if (killed) {
    // A script read inside its process (parser.step_tokens) reports a syntax error only as the
    // exit error's text. Re-parse it, so a syntax error raises pycvc.parse_error with its position
    // either way, as it does when the script is parsed at submit.
    (void)parse_all(src, parse_options_from_state(*impl_->root, {}, sched.parser_defaults()));
  }
  if (failure)
    std::rethrow_exception(failure);
  if (error)
    throw std::runtime_error(*error);
  if (!result.has_value())
    throw std::runtime_error("pycvc.Exec.run: program produced no result (pid " +
                             std::to_string(pid) + ")");
  return render_result(*result);
}

cvc::state_exec::parse_options Exec::parser_defaults() const {
  return impl_->sched.parser_defaults();
}

void Exec::set_parser_defaults(const cvc::state_exec::parse_options &defaults) {
  impl_->sched.set_parser_defaults(defaults);
}

cvc::state_exec::data_options Exec::data_defaults() const { return impl_->sched.data_defaults(); }

void Exec::set_data_defaults(const cvc::state_exec::data_options &defaults) {
  impl_->sched.set_data_defaults(defaults);
}

cvc::state_exec::async_scheduler &Exec::scheduler() { return impl_->sched; }

} // namespace pycvc
