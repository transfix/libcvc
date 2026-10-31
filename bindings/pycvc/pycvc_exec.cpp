// pycvc_exec.cpp — Exec implementation: run DSL programs in an app's context
// and bridge Python callables into the DSL as native_fn values.
// clang-format off
#include <Python.h>
// clang-format on

#include "pycvc_exec.h"

#include "pycvc_value.h" // the shared value_t <-> Python conversion

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>      // Exec's async functions run through its host
#include <cvc/state/state_exec/async_scheduler.h> // Exec drives a private async scheduler
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/types.h>
#include <deque>
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
// `call` is its call: the wait for the GIL isn't charged as its run time (3.8), and with
// `with_call` the callable gets its pycvc.async_call handle before the arguments.
value_t call_python(const std::shared_ptr<PyObject> &holder, std::span<const value_t> args,
                    const std::string &what, bool want_result = true, async_call *call = nullptr,
                    bool with_call = false) {
  const auto wait_start = std::chrono::steady_clock::now();
  PyGILState_STATE gil = PyGILState_Ensure();
  if (call != nullptr)
    call->add_wait(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count());
  PyObject *pyargs = values_to_py_tuple(args); // nullptr (error set) if an arg can't convert
  if (pyargs != nullptr && with_call && call != nullptr) {
    // (handle, *args)
    PyObject *handle = async_call_to_py(call->shared_from_this());
    const Py_ssize_t n = PyTuple_GET_SIZE(pyargs);
    PyObject *with = handle != nullptr ? PyTuple_New(n + 1) : nullptr;
    if (with != nullptr) {
      PyTuple_SET_ITEM(with, 0, handle);
      for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject *item = PyTuple_GET_ITEM(pyargs, i);
        Py_INCREF(item);
        PyTuple_SET_ITEM(with, i + 1, item);
      }
    } else {
      Py_XDECREF(handle);
    }
    Py_DECREF(pyargs);
    pyargs = with;
  }
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
// Returns whether the slice waited: whether asyncio blocked in its selector (until the stop timer,
// or I/O) for most of it. Where its clock is fine (Linux, macOS, Windows on Python 3.13+) it does.
// Where the clock is coarse (Windows before 3.13: ~15.6 ms), asyncio runs a timer due within that
// resolution at once, the stop timer included, and the slice returns without waiting.
bool drive_coros(PyObject *loop, std::vector<pending_coro> &pending, async_host &host,
                 double budget_secs) {
  if (loop == nullptr || pending.empty())
    return false;
  using clock = std::chrono::steady_clock;
  bool waited = false;
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
      const clock::time_point from = clock::now();
      if (PyObject *r = PyObject_CallMethod(loop, "run_forever", nullptr)) {
        Py_DECREF(r);
        step_failed = false;
        waited = clock::now() - from >= std::chrono::duration<double>(0.5 * budget_secs);
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
    return false;
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
  return waited;
}

// Wait until no kernel of `host` is queued or running, WITH THE GIL RELEASED so a kernel calling
// into Python can finish. The caller holds the GIL (run()/~Exec do).
void wait_idle(async_host &host) {
  if (host.in_flight() == 0)
    return;
  Py_BEGIN_ALLOW_THREADS host.wait_idle();
  Py_END_ALLOW_THREADS
}

// Py_BEGIN_ALLOW_THREADS / Py_END_ALLOW_THREADS as a scope: the GIL is released for the scope's
// life and taken back however it ends, a C++ exception included (a sibling process's on_complete
// raising out of a step), so run()'s catch path quiesces and reaps holding the GIL.
class gil_released {
public:
  gil_released() : ts_(PyEval_SaveThread()) {}
  ~gil_released() { PyEval_RestoreThread(ts_); }
  gil_released(const gil_released &) = delete;
  gil_released &operator=(const gil_released &) = delete;

private:
  PyThreadState *ts_;
};

// A priority call (Exec.set_priority / get_priority, docs/STATE_EXEC_LOCKS.md 7.4) as plain C++
// data: what it asks, and its answer. The thread that answers it never builds a Python object;
// the caller does, once it no longer holds the control mutex and holds the GIL again.
struct priority_query {
  int pid = -1;
  std::optional<int> set_to;         // set_priority's priority; a get_priority when empty
  std::optional<priority_info> info; // get_priority's answer
  bool applied = false;              // set_priority's answer
  std::string error;                 // no answer could be had: the call raises RuntimeError
  bool done = false;
};

// Answer `q` from `sched`. The caller holds the Exec's control mutex and may touch the scheduler:
// it is the active run()'s thread, between steps, or no run() is active.
void answer(async_scheduler &sched, priority_query &q) noexcept {
  try {
    if (q.set_to)
      q.applied = sched.set_priority(q.pid, *q.set_to); // a host's change: unchecked
    else
      q.info = sched.get_priority(q.pid);
  } catch (const std::exception &e) {
    q.error = e.what();
  } catch (...) {
    q.error = "pycvc.Exec: the priority call failed";
  }
  q.done = true;
}

// The Exec (its ExecImpl's address) whose run() this thread is in, the innermost; null when none.
thread_local const void *t_running_exec = nullptr;

// Step (1) of a priority call: from a thread running another Exec's run(), raise at once, with the
// GIL held and no mutex taken. A call that waited there would hold that run() up, and the other
// Exec's run() could be waiting for this one.
void check_not_in_other_run(const void *impl) {
  if (t_running_exec != nullptr && t_running_exec != impl)
    throw std::runtime_error("Exec.run is active on another thread");
}

// CVC_PYCVC_TEST_QUIESCE_DELAY_MS, for PR-8's test only: milliseconds run() sleeps, holding the
// GIL, between its quiesce and its reap's lock. 0 when unset or not a positive number.
int test_quiesce_delay_ms() {
  const char *v = std::getenv("CVC_PYCVC_TEST_QUIESCE_DELAY_MS");
  if (v == nullptr)
    return 0;
  char *end = nullptr;
  const long ms = std::strtol(v, &end, 10);
  return end != v && ms > 0 ? static_cast<int>(std::min(ms, 60000L)) : 0;
}

std::optional<scheduling_policy> policy_named(const std::string &name) {
  if (name == "round_robin")
    return scheduling_policy::round_robin;
  if (name == "priority")
    return scheduling_policy::priority;
  if (name == "priority_rr")
    return scheduling_policy::priority_rr;
  return std::nullopt;
}

// The policy an Exec's scheduler is built with (docs/STATE_EXEC_LOCKS.md 7.3, 7.4): None reads
// state_exec.policy under `root`, logging an unknown value to `app`'s log and using round_robin, as
// app::exec_scheduler() does; a pycvc.scheduling_policy member or its name is that policy. Anything
// else throws std::invalid_argument (ValueError). The GIL is held.
scheduling_policy exec_policy(PyObject *policy, cvc::state &root, cvc::app &app) {
  if (policy == nullptr || policy == Py_None) {
    bool unknown = false;
    const scheduling_policy p = policy_from_state(root, scheduling_policy::round_robin, &unknown);
    if (unknown) {
      const cvc::state *node = root.findDescendant("state_exec.policy");
      app.log(1, "pycvc.Exec: state_exec.policy: unknown value '" +
                     (node ? const_cast<cvc::state *>(node)->value() : std::string()) +
                     "' (round_robin, priority or priority_rr); using round_robin");
    }
    return p;
  }
  std::optional<scheduling_policy> chosen;
  std::string got;
  PyObject *type = scheduling_policy_type();
  const int is_member = type != nullptr ? PyObject_IsInstance(policy, type) : 0;
  if (is_member == 1) {
    // A member's name is its C++ name.
    if (PyObject *name = PyObject_GetAttrString(policy, "name")) {
      if (const char *s = PyUnicode_Check(name) ? PyUnicode_AsUTF8(name) : nullptr)
        chosen = policy_named(s);
      Py_DECREF(name);
    }
  } else if (PyUnicode_Check(policy)) {
    if (const char *s = PyUnicode_AsUTF8(policy)) {
      got = std::string(" '") + s + "'";
      chosen = policy_named(s);
    }
  } else {
    got = std::string(" (") + Py_TYPE(policy)->tp_name + ")";
  }
  PyErr_Clear(); // a lookup that failed above is the ValueError below
  if (!chosen)
    throw std::invalid_argument("pycvc.Exec: policy must be None, a pycvc.scheduling_policy or its "
                                "name (round_robin, priority, priority_rr), not" +
                                (got.empty() ? std::string(" that") : got));
  return *chosen;
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
                                   PyObject *callable, bool with_call) {
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
  return [holder, what, with_call](std::span<const value_t> args) -> async_kernel {
    return [holder, what, with_call,
            argv = std::vector<value_t>(args.begin(), args.end())](async_call &call) -> value_t {
      return call_python(holder, argv, what, true, &call, with_call);
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
  explicit ExecImpl(scheduling_policy policy) : sched(policy) {}

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
  // The builtins, the intrinsics and the registered functions; `env` extends it with the
  // programs' top-level definitions, which persist across run(). A spawned child extends `natives`
  // too, so it calls the registered functions but not the parent's definitions (td 4.15).
  environment_ptr natives;
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
  // Re-entrancy guard: a registered fn/coro (or a kernel, or another thread) must not call back
  // into run(). run() tests and sets it at entry, and its RunGuard clears it last, both under
  // control_mutex, so a second run() raises even while the first quiesces.
  bool running = false;
  // How long run() waits while its program is parked with nothing pending (Exec's idle_timeout),
  // and the test-only quiesce delay (CVC_PYCVC_TEST_QUIESCE_DELAY_MS, read at construction).
  double idle_timeout = 60.0;
  int test_quiesce_delay_ms = 0;

  // The priority calls (docs/STATE_EXEC_LOCKS.md 7.4). control_mutex stands in for the pump
  // token: the scheduler is touched only by the thread of the active run() or, with none active,
  // by a holder of the mutex and the GIL (the GIL is what Python's own calls on the scheduler,
  // Exec.scheduler()'s methods, hold). The GIL comes before the mutex: every thread takes the
  // mutex with the GIL held (run() at entry, for its quiesce and its reaps, in RunGuard; a
  // priority call), and no thread ever waits for the GIL while it holds the mutex. Nothing under
  // the mutex builds or frees a Python object (reap() frees its process after the unlock), and a
  // priority call that waits for run() releases the GIL under the mutex and takes it back only
  // once the mutex is unlocked (priority_call).
  std::mutex control_mutex;
  std::condition_variable control_cv;
  // A run() is active: set before its first step, cleared by its quiesce. Not `running`, which
  // stays set through the quiesce.
  bool accepting_queries = false;
  std::thread::id run_thread; // the active run()'s thread
  // The calls waiting for the active run() to answer them between steps.
  std::deque<std::shared_ptr<priority_query>> queries;

  // Answer every waiting call and wake its caller. control_mutex is held, on the active run()'s
  // thread between steps (the query a call posts runs this), or in the quiesce. Builds no PyObject.
  void answer_queries() {
    if (queries.empty())
      return;
    for (const std::shared_ptr<priority_query> &q : queries)
      answer(sched, *q);
    queries.clear();
    control_cv.notify_all();
  }

  // run()'s quiesce, on every exit path, after its last step and any kill and before it waits for
  // its kernels: no call is left waiting, and from now on a call acts directly, under the mutex.
  void quiesce() {
    std::lock_guard<std::mutex> lock(control_mutex);
    accepting_queries = false;
    answer_queries();
  }

  // run()'s reap of its program (both exit paths), under control_mutex, so a thread acting
  // directly during the quiesce never touches the process table while the reap erases from it.
  // The GIL is held, taken before the mutex. The process is freed after the unlock: its captured
  // values may hold the last reference to a Python object (an exception's traceback, and the
  // frames and locals in it), whose finalizer may release and retake the GIL, or call this Exec,
  // and under the mutex either would deadlock.
  void reap(int pid) {
    if (test_quiesce_delay_ms > 0) // PR-8: a woken caller reaches its GIL while run() holds it
      std::this_thread::sleep_for(std::chrono::milliseconds(test_quiesce_delay_ms));
    process_ptr gone; // freed when this returns: the GIL held, the mutex not
    std::lock_guard<std::mutex> lock(control_mutex);
    sched.reap(pid, &gone);
  }

  // Steps (2)-(5) of a priority call: answer `q`, directly or by the active run().
  void priority_call(priority_query &q);
};

void Exec::ExecImpl::priority_call(priority_query &q) {
  // (2) Take the control mutex with the GIL held, the order run() uses. Waiting for it so cannot
  // deadlock: no holder of the mutex waits for the GIL. Holding both, a direct answer excludes
  // run() and Python's own calls on the scheduler alike.
  std::unique_lock<std::mutex> lock(control_mutex);
  if (!accepting_queries || std::this_thread::get_id() == run_thread) {
    // (3) No run() is active, or this is its thread (a registered function, coroutine or
    // callback): act directly.
    answer(sched, q);
    return;
  }
  // (3) A run() pumps on another thread: post the call for it to answer between steps, and wait
  // with the GIL released. Its quiesce answers it too, if the run ends first.
  auto pending = std::make_shared<priority_query>(q);
  queries.push_back(pending);
  control_cmd cmd;
  cmd.what = control_cmd::kind::query;
  cmd.query = [this](scheduler_base &) {
    std::lock_guard<std::mutex> answering(control_mutex);
    answer_queries();
  };
  sched.post_control(q.pid, std::move(cmd));
  // The GIL is released under the mutex (taking it there is what may never happen), and on every
  // path, an exception included, the mutex is unlocked (4) before the GIL is taken back (5).
  struct gil_released_until_unlocked {
    std::unique_lock<std::mutex> &lock;
    PyThreadState *ts = PyEval_SaveThread();
    ~gil_released_until_unlocked() {
      if (lock.owns_lock())
        lock.unlock();
      PyEval_RestoreThread(ts);
    }
  } nogil{lock};
  control_cv.wait(lock, [&] { return pending->done; });
  q = *pending; // (4) the answer, copied out under the mutex
}

Exec::Exec(const std::shared_ptr<cvc::app> &app, PyObject *policy, double idle_timeout) {
  if (!app) // a RuntimeError, as it always was (a bad policy or idle_timeout is a ValueError)
    throw std::runtime_error("pycvc.Exec: null app handle");
  if (!(idle_timeout >= 0.0)) // NaN too
    throw std::invalid_argument("pycvc.Exec: idle_timeout must be a number of seconds, 0 or more");
  cvc::state &root = cvc::state::instance(*app);
  // The policy is fixed for the scheduler's life (docs/STATE_EXEC_LOCKS.md 7.3).
  impl_ = std::make_shared<ExecImpl>(exec_policy(policy, root, *app));
  impl_->app = app; // keep the app (and its compute pool) alive for as long as this Exec exists
  impl_->root = &root;
  impl_->idle_timeout = idle_timeout;
  impl_->test_quiesce_delay_ms = test_quiesce_delay_ms();
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

  impl_->natives = builtins::make_default_environment();
  register_intrinsics(impl_->natives, &impl_->ictx);
  impl_->env = environment::extend(impl_->natives);
  // On one scheduler the intrinsics act for the running process, so the Exec's own context serves
  // its children; they keep its owner and extend its natives.
  impl_->ictx.spawn_context = [natives = impl_->natives](const intrinsics_context &) {
    return spawn_setup{environment::extend(natives), nullptr};
  };
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
  builtins::register_fn(impl_->natives, name,
                        python_native_fn("pycvc.Exec.register_fn", name, callable));
}

void Exec::register_async_fn(const std::string &name, PyObject *callable, bool with_call) {
  // An async function on this Exec's host (docs/STATE_EXEC_EXCEPTIONS.md 3): (name args...) parks
  // the program while the callable runs on the app's background task pool, then resumes it with
  // the value or raises the callable's exception at the call (a PythonError). (name-async ...)
  // returns a future.
  cvc::state_exec::register_async_fn(
      impl_->natives, *impl_->host, name,
      python_async_marshal("pycvc.Exec.register_async_fn", name, callable, with_call));
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
    builtins::register_fn(impl_->natives, future ? name + "-async" : name,
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
  ExecImpl *const impl = impl_.get();
  // One Exec is driven by ONE thread. A registered fn/coro that calls back into run() (or a second
  // thread, or a kernel while this run() quiesces) would corrupt the shared scheduler + asyncio
  // loop, so reject re-entrancy up front. The test and set are under the control mutex, taken with
  // the GIL held (the one order allowed, docs/STATE_EXEC_LOCKS.md 7.4), and so is what makes this
  // run() the active one before its first step: from here another thread's priority call waits
  // for this run() to answer it between steps.
  {
    std::lock_guard<std::mutex> lock(impl->control_mutex);
    if (impl->running)
      throw std::runtime_error("pycvc.Exec.run: re-entrant call — a registered fn/coro must not "
                               "call run() (use a separate Exec per thread)");
    impl->running = true;
    impl->run_thread = std::this_thread::get_id();
    impl->accepting_queries = true;
  }
  const void *const outer_run = t_running_exec; // a run() of another Exec this one nests in
  t_running_exec = impl;
  // On EVERY exit: quiesce (if an exit below didn't), cancel+drop any coroutine Tasks still pending
  // for this run (e.g. an orphaned coroutine of a sibling process that never completed) so it can't
  // suppress the NEXT run()'s idle timeout, and clear the running flag, last. PyGILState_Ensure
  // re-acquires the GIL on whatever path we unwind.
  struct RunGuard {
    ExecImpl *impl;
    const void *outer_run;
    ~RunGuard() {
      // A run() that ended before its loop (execute() threw) quiesces here; the others did.
      impl->quiesce();
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
      t_running_exec = outer_run;
      // Last, under the control mutex: until here a second run() raises.
      std::lock_guard<std::mutex> lock(impl->control_mutex);
      impl->run_thread = std::thread::id();
      impl->running = false;
    }
  } run_guard{impl, outer_run};

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
  const double idle_timeout = impl->idle_timeout; // parked with NO pending work this long -> bail
  using clock = std::chrono::steady_clock;
  static constexpr auto kMaxIdleWait = std::chrono::milliseconds(50);
  // Saturating: an infinite idle_timeout never runs out.
  const auto fresh_idle_deadline = [idle_timeout] {
    return cvc::state_exec::detail::time_after(clock::now(), idle_timeout);
  };
  clock::time_point idle_deadline = fresh_idle_deadline();
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
      // found. A throw out of the slice (a sibling's on_complete) takes the GIL back first.
      {
        gil_released nogil;
        sched.sync_run_steps(kSliceSteps, kSliceSeconds); // reads get_result
      }
      // Coroutine leg: step the asyncio loop a bounded slice and deliver any completed
      // coroutine's result to its parked process (GIL held here). A no-op when no coroutine is
      // pending, so a coro-free Exec is unaffected.
      const bool coros_waited =
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
        idle_deadline = fresh_idle_deadline();
        continue; // our program has more to run — loop (the next slice releases the GIL again)
      }
      // Our pid is parked (waiting). A pool worker still in flight, or any sibling progress, means
      // a delivery is coming, so keep the idle clock fresh; only a park with NOTHING pending can
      // time out (e.g. a msg-recv on a channel nobody sends to) — safe to abandon then, no worker
      // to outlive us.
      if (host.in_flight() > 0 || !impl_->pending_coros.empty() || sched.has_runnable() ||
          had_runnable) {
        idle_deadline = fresh_idle_deadline();
      } else if (clock::now() > idle_deadline) {
        sched.kill(pid);
        error = "pycvc.Exec.run: parked with no pending work (pid " + std::to_string(pid) + ")";
        break;
      }
      // Idle-wait with the GIL released (so a worker can take it and deliver) until there is
      // work: a kernel's post or result, a sleeper's or timed wait's deadline, a call's timeout
      // or charge, or the idle deadline; never longer than kMaxIdleWait, a backstop for work that
      // arrives some other way. Coroutines on the asyncio loop are this loop's to drive: while one
      // is pending, a slice that waited (coros_waited) was this iteration's wait, and the loop
      // goes on at once (another wait here would hold asyncio off for as long). A slice that
      // returned at once gets a 5 ms beat instead, so the loop doesn't spin: the old 1 ms sleep
      // lasted a ~15.6 ms timer tick there. Processes parked until the next pump (await) make it
      // a 1 ms beat, as it was. These beats burn no CPU (wakeup).
      if (!impl_->pending_coros.empty() && coros_waited)
        continue;
      clock::time_point until =
          std::min(idle_deadline, clock::time_point(clock::now() + kMaxIdleWait));
      if (!impl_->pending_coros.empty())
        until = std::min(until, clock::now() + std::chrono::milliseconds(5));
      if (sched.awaiting_frame_count() > 0)
        until = std::min(until, clock::now() + std::chrono::milliseconds(1));
      gil_released nogil;
      sched.wait_for_work(until);
    }
  } catch (...) {
    sched.kill(pid); // cancels its calls in flight
    impl->quiesce(); // after the last step and the kill: no priority call is left waiting
    wait_idle(host); // never leave a kernel touching our (about-to-unwind) scheduler
    impl->reap(pid); // under the control mutex
    throw;
  }
  // The last step is done (and any kill): answer the priority calls still waiting, and let later
  // ones act directly, under the control mutex.
  impl->quiesce();
  // No kernel of this run may outlive it: wait (GIL released) BEFORE we reap the process or return
  // to Python, so a kernel can never post to / be destroyed against a torn-down scheduler.
  wait_idle(host);
  impl->reap(pid); // drop the finished/killed process from the private scheduler (no leak)
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

bool Exec::set_priority(int pid, int priority) {
  check_not_in_other_run(impl_.get()); // (1)
  priority_query q;
  q.pid = pid;
  q.set_to = priority;
  impl_->priority_call(q); // (2)-(5)
  if (!q.error.empty())
    throw std::runtime_error(q.error);
  return q.applied;
}

PyObject *Exec::get_priority(int pid) {
  check_not_in_other_run(impl_.get()); // (1)
  priority_query q;
  q.pid = pid;
  impl_->priority_call(q); // (2)-(5): the GIL is held again, the control mutex isn't
  if (!q.error.empty())
    throw std::runtime_error(q.error);
  if (!q.info)
    Py_RETURN_NONE;
  return priority_info_to_py(*q.info);
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
