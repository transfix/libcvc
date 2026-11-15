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
#include <cvc/core/exec_pool.h> // ExecPool's batches pump on the app's pool
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>      // Exec's async functions run through its host
#include <cvc/state/state_exec/async_scheduler.h> // Exec drives a private async scheduler
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/lock_home.h> // the Exec's lock home: blocking_sinks, progress
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/exec_group.h> // a pooled Exec's scheduler is a group's
#include <cvc/state/state_exec/heap.h>       // Exec::tracked_objects
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h> // the app's lock manager (Exec::lock_query)
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h> // kPumpedElsewhereError
#include <cvc/state/state_exec/stdlib.h>    // the Exec's random_stream
#include <cvc/state/state_exec/types.h>
#include <deque>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <variant>
#include <vector>

namespace pycvc {

namespace {
using namespace cvc::state_exec;
using cvc::state_exec::detail::lock_home;

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
                    bool with_call = false, exec_context *ctx = nullptr,
                    bool with_context = false) {
  const auto wait_start = std::chrono::steady_clock::now();
  PyGILState_STATE gil = PyGILState_Ensure();
  if (call != nullptr)
    call->add_wait(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count());
  PyObject *pyargs = values_to_py_tuple(args); // nullptr (error set) if an arg can't convert
  // The context handle given to a with_context callable: it holds `ctx` until the call returns.
  std::shared_ptr<call_context> context;
  if (pyargs != nullptr && ((with_call && call != nullptr) || with_context)) {
    // (handle, *args): the kernel's call, or the native call's context
    PyObject *handle = nullptr;
    if (with_call && call != nullptr) {
      handle = async_call_to_py(call->shared_from_this());
    } else {
      context = std::make_shared<call_context>(call_context{ctx});
      handle = call_context_to_py(context);
    }
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
  if (context)
    context->ctx = nullptr; // the call is over: a handle Python kept holds no context
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
// Kept in Python's own per-thread state (PyThreadState_GetDict) under this key, as an int, so a
// Python function a run() calls on this thread sees it too. The GIL is held at every access.
constexpr const char *kRunningExecKey = "pycvc.running_exec";

const void *running_exec() {
  PyObject *d = PyThreadState_GetDict();
  PyObject *v = d != nullptr ? PyDict_GetItemString(d, kRunningExecKey) : nullptr; // borrowed
  return v != nullptr ? PyLong_AsVoidPtr(v) : nullptr;
}

// Make `impl` (null: none) this thread's running Exec. Leaves a Python error that was pending as it
// was.
void set_running_exec(const void *impl) {
  PyObject *d = PyThreadState_GetDict();
  if (d == nullptr)
    return;
  PyObject *type = nullptr, *value = nullptr, *traceback = nullptr;
  PyErr_Fetch(&type, &value, &traceback);
  if (impl == nullptr) {
    if (PyDict_GetItemString(d, kRunningExecKey) != nullptr)
      PyDict_DelItemString(d, kRunningExecKey);
  } else if (PyObject *v = PyLong_FromVoidPtr(const_cast<void *>(impl))) {
    PyDict_SetItemString(d, kRunningExecKey, v);
    Py_DECREF(v);
  }
  PyErr_Clear();
  PyErr_Restore(type, value, traceback);
}

// Step (1) of a priority call: from a thread running another Exec's run(), raise at once, with the
// GIL held and no mutex taken. A call that waited there would hold that run() up, and the other
// Exec's run() could be waiting for this one.
void check_not_in_other_run(const void *impl) {
  const void *running = running_exec();
  if (running != nullptr && running != impl)
    throw std::runtime_error("Exec.run is active on another thread");
}

// The Exec calls that are the pump thread's (S1-Q4, td 4.12): they change its scheduler or the
// natives its programs share, or read what its steps change, so while another thread pumps the
// scheduler (its run(), or a raw call's) they raise instead of racing it.
void check_pump_thread(const async_scheduler &sched) {
  if (sched.pumped_elsewhere())
    throw std::runtime_error(kPumpedElsewhereError);
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
      // Read with the node held, as the schedulers read their settings (R0-mid).
      const std::optional<std::string> value =
          cvc::state_exec::detail::setting_value(root, "state_exec.policy");
      app.log(1, "pycvc.Exec: state_exec.policy: unknown value '" + value.value_or(std::string()) +
                     "' (round_robin, priority or priority_rr); using round_robin");
    }
    return p;
  }
  std::optional<scheduling_policy> chosen;
  std::string got;
  PyObject *type = scheduling_policy_type();
  const int is_member = type != nullptr ? PyObject_IsInstance(policy, type) : 0;
  Py_XDECREF(type);
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

// The absolute key of the lock a process waits for, from its waiting_on ("lock /<key>", design
// 5.10), or nullopt when it waits for none.
std::optional<std::string> lock_waited_for(const process_info &info) {
  static constexpr std::string_view tag = "lock /";
  if (!info.waiting_on || std::string_view(*info.waiting_on).substr(0, tag.size()) != tag)
    return std::nullopt;
  return info.waiting_on->substr(tag.size());
}

// run()'s idle rule for the lock waits of an Exec's processes (docs/STATE_EXEC_LOCKS.md 9.2: a
// lock wait of any of them counts, not only run()'s own): of the processes they ultimately wait
// for (blocking_sinks, protocol 4.13) that are on another home, the latest of their homes'
// progress within `idle_timeout` of `now`; nullopt when there is none. One search from all of
// `pids` at once, under one hold of the manager's mutex. Each sink comes with its home's progress,
// loaded under that mutex, so no other home is reached here. A sink on this home counts only
// through run()'s own test, never because this run() pumps it (LT-8b).
std::optional<std::chrono::steady_clock::time_point>
progress_waited_for(const lock_home &home, std::span<const int> pids, double idle_timeout,
                    std::chrono::steady_clock::time_point now) {
  using clock = std::chrono::steady_clock;
  std::optional<clock::time_point> latest;
  for (const auto &[sink, ticks] : home.blocking_sinks(pids)) {
    if (sink.home == home.id())
      continue;
    const clock::time_point progress{clock::duration(ticks)};
    if (std::chrono::duration<double>(now - progress).count() <= idle_timeout)
      latest = std::max(latest.value_or(progress), progress);
  }
  return latest;
}

// A lock wait to name in a kill's message (pid and absolute key): run()'s own (`root`), else one
// of the other lock waiters', or nullopt. Read only for a kill.
std::optional<std::pair<int, std::string>> lock_wait_named(const async_scheduler &sched,
                                                           const process_info &root,
                                                           std::span<const int> waiters) {
  if (std::optional<std::string> key = lock_waited_for(root))
    return std::make_pair(root.pid, std::move(*key));
  for (int pid : waiters)
    if (const std::optional<process_info> info = sched.get_process_info(pid))
      if (std::optional<std::string> key = lock_waited_for(*info))
        return std::make_pair(pid, std::move(*key));
  return std::nullopt;
}

// A lock-query answer as the DSL's (lock-query) builds it (lock_intrinsics.cpp), its keys shown
// under `root` (protocol 5.10). Plain C++ values: Exec::lock_query converts them once it holds the
// GIL again.
value_t lock_query_value(const lock_query_result &r, std::string_view root) {
  const auto mode = [](lock_mode m) {
    return value_t(std::string(m == lock_mode::exclusive ? "exclusive" : "shared"));
  };
  std::vector<value_t> held;
  for (const lock_info &i : r.held) {
    std::vector<value_t> pids;
    for (int pid : i.pids)
      pids.emplace_back(static_cast<int64_t>(pid));
    held.push_back(make_dict(
        {{"key", value_t(lock_home::show(i.key, root))},
         {"mode", mode(i.mode)},
         {"pids", make_list(std::move(pids))},
         {"count", value_t(static_cast<int64_t>(i.count))},
         {"generation", i.generation ? value_t(static_cast<int64_t>(*i.generation)) : nil_value()},
         {"abandoned", value_t(i.abandoned)}}));
  }
  std::vector<value_t> waiting;
  for (const lock_waiter_info &w : r.waiting) {
    // A string for one key, a list for several.
    value_t key;
    if (w.keys.size() == 1) {
      key = value_t(lock_home::show(w.keys[0], root));
    } else {
      std::vector<value_t> keys;
      for (const std::string &k : w.keys)
        keys.emplace_back(lock_home::show(k, root));
      key = make_list(std::move(keys));
    }
    waiting.push_back(
        make_dict({{"key", std::move(key)},
                   {"mode", mode(w.mode)},
                   {"pid", w.pid ? value_t(static_cast<int64_t>(*w.pid)) : nil_value()}}));
  }
  return make_dict(
      {{"held", make_list(std::move(held))}, {"waiting", make_list(std::move(waiting))}});
}
} // namespace

// A SYNCHRONOUS DSL native_fn: it runs inline on the thread stepping the scheduler (Exec::run's,
// or the caller of a wrapped scheduler's run/step, which release the GIL; call_python retakes it).
native_fn python_native_fn(const std::string &who, const std::string &name, PyObject *callable,
                           bool with_context) {
  if (callable == nullptr || !PyCallable_Check(callable))
    throw std::invalid_argument(who + ": '" + name + "' is not callable");
  if (is_coroutine_function(callable))
    throw std::invalid_argument(who + ": '" + name +
                                "' is an async def — use Exec.register_async_coro");
  std::shared_ptr<PyObject> holder = own_callable(callable);
  const std::string what = "python DSL fn '" + name + "'";
  return [what, holder, with_context](call_args args) -> value_t {
    return call_python(holder, args, what, true, nullptr, false, args.context(), with_context);
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
  // A plain Exec: a scheduler of its own, with its own registry and random stream.
  explicit ExecImpl(scheduling_policy policy)
      : registry(make_cycle_registry()), own_sched(std::in_place, policy), sched(*own_sched) {
    sched.set_cycle_registry(registry.get());
    sched.set_random_stream(&random);
  }
  // A pooled Exec (ExecPool): an exec_group's scheduler, which binds the group's registry and
  // stream.
  explicit ExecImpl(std::unique_ptr<exec_group> g) : group(std::move(g)), sched(group->sched()) {}
  // The spawn context holds `natives`: let it go with them, before a pooled Exec's group.
  ~ExecImpl() { ictx.spawn_context = nullptr; }

  std::shared_ptr<cvc::app> app; // co-own so root/ictx stay valid for our lifetime
  cvc::state *root = nullptr;
  // A plain Exec is its own group 0 (S1.12, td 4.7, 4.10): its scheduler binds this registry and
  // random stream itself whichever thread drives it. Before `own_sched`, `env` and `natives`, so
  // the registry is collected and freed after them; after `app`, whose tree what it frees may pin.
  // A pooled Exec's are its group's (null and unused here).
  cycle_registry_ptr registry;
  random_stream random;
  // A plain Exec's PRIVATE async_scheduler (NOT the app-wide one): it supports park/await +
  // cross-thread delivery, and isolating it means a pycvc program never races an Ariadne Runtime
  // (or another Exec) driving a shared scheduler, and its finished processes die with the Exec. An
  // async fn offloaded to the compute pool posts back to THIS scheduler (register_async_fn passes
  // it), so the parked program resumes here.
  std::optional<async_scheduler> own_sched;
  intrinsics_context ictx;
  // A pooled Exec's group (S1.12), which owns its scheduler, async host, registry and stream.
  // After `ictx` and before `natives` and `env`: the group's teardown kills and reaps its
  // processes while the context they point into is here, and its registry is collected once the
  // Exec's environments are gone.
  std::unique_ptr<exec_group> group;
  async_scheduler &sched; // *own_sched, or the group's
  process_ptr host_proc;
  // The builtins, the intrinsics and the registered functions; `env` extends it with the
  // programs' top-level definitions, which persist across run(). A spawned child extends `natives`
  // too, so it calls the registered functions but not the parent's definitions (td 4.15).
  environment_ptr natives;
  environment_ptr env;
  // The async host of this Exec's async functions (docs/STATE_EXEC_EXCEPTIONS.md 3): kernels run
  // on the app's background task pool and post back to `sched`. A plain Exec's own (after
  // `own_sched`, so it goes first), or its group's; ~Exec shuts it down first, with the GIL
  // released.
  std::unique_ptr<cvc::state_exec::async_host> own_host;
  cvc::state_exec::async_host *host = nullptr;
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
  // Exec.scheduler()'s methods, hold). The scheduler's pump owner backs it (S1-Q4): run() holds
  // it until it returns, a direct call with no run() active while it holds the mutex, and Python's
  // own calls on the scheduler raise while another thread holds it. The GIL comes before the
  // mutex: every thread
  // takes the mutex with the GIL held (run() at entry, for its quiesce and its reaps, in RunGuard;
  // a priority call), and no thread ever waits for the GIL while it holds the mutex. Nothing under
  // the mutex runs Python or frees a Python object (reap() cancels its program's async calls
  // before it and frees the process after it), and a
  // priority call that waits for run() releases the GIL under the mutex and takes it back only
  // once the mutex is unlocked (priority_call).
  std::mutex control_mutex;
  std::condition_variable control_cv;
  // A run() is active: set before its first step, cleared by its quiesce. Not `running`, which
  // stays set through the quiesce.
  bool accepting_queries = false;
  std::thread::id run_thread; // the active run()'s thread
  // The active run()'s hold of the scheduler's pump owner (S1-Q4): taken with `running` and
  // released with it, as run() returns, both under control_mutex and on run()'s thread. Through
  // the quiesce, its wait for its kernels and its reap, a raw call from another thread raises as
  // it does during the run (R1-mid C-m3).
  std::optional<async_scheduler::pump_owner_scope> run_pump_owner;
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
  // run() keeps the pump owner until it returns (run_pump_owner).
  void quiesce() {
    std::lock_guard<std::mutex> lock(control_mutex);
    accepting_queries = false;
    answer_queries();
  }

  // run()'s reap of its program (both exit paths), under control_mutex, so a thread acting
  // directly during the quiesce never touches the process table while the reap erases from it.
  // The GIL is held, taken before the mutex. Nothing that can run Python happens under the
  // mutex, where a finalizer or a slot that released and retook the GIL, or called this Exec,
  // would deadlock: the program's async calls are cancelled first (their charge reads the step
  // rate, which a scheduler with an id publishes to the tree, where a state_observer's
  // on_changed runs), and the process is freed after the unlock (its captured values may hold the
  // last reference to a Python object: an exception's traceback, and the frames and locals in it).
  void reap(int pid) {
    if (test_quiesce_delay_ms > 0) // PR-8: a woken caller reaches its GIL while run() holds it
      std::this_thread::sleep_for(std::chrono::milliseconds(test_quiesce_delay_ms));
    sched.cancel_async_calls_of(pid);
    process_ptr gone; // freed when this returns: the GIL held, the mutex not
    std::lock_guard<std::mutex> lock(control_mutex);
    sched.reap(pid, &gone);
  }

  // Steps (2)-(5) of a priority call: answer `q`, directly or by the active run().
  void priority_call(priority_query &q);

  // The programs' context over `root`, chrooted at `root_path` (none when empty) before the
  // intrinsics are registered into `base`, which becomes `natives`; `h` is the async host.
  void init_context(const std::string &root_path, environment_ptr base,
                    cvc::state_exec::async_host *h) {
    host_proc = make_process();
    host_proc->pid = 0;
    host_proc->status = process_status::ready;
    ictx.sched = &sched;
    ictx.root = root;
    if (!root_path.empty())
      apply_chroot(ictx, *root, root_path);
    ictx.proc = host_proc;
    ictx.pid = 0;
    ictx.uid = "pycvc";
    ictx.cluster_id = "local";
    ictx.node_id = "local";
    natives = std::move(base);
    register_intrinsics(natives, &ictx);
    env = environment::extend(natives);
    // On one scheduler the intrinsics act for the running process, so the Exec's own context
    // serves its children; they keep its owner and extend its natives.
    ictx.spawn_context = [natives = natives](const intrinsics_context &) {
      return spawn_setup{environment::extend(natives), nullptr};
    };
    host = h;
    ictx.host = h; // its group's host (td 4.9, B30); spawned children copy it
  }
};

void Exec::ExecImpl::priority_call(priority_query &q) {
  // (2) Take the control mutex with the GIL held, the order run() uses. Waiting for it so cannot
  // deadlock: no holder of the mutex waits for the GIL. Holding both, a direct answer excludes
  // run() and Python's own calls on the scheduler alike.
  std::unique_lock<std::mutex> lock(control_mutex);
  if (!accepting_queries || std::this_thread::get_id() == run_thread) {
    // (3) No run() is active, or it has quiesced, or this is its thread (a registered function,
    // coroutine or callback): act directly. With no run() active the call is the pump owner
    // (S1-Q4), so it raises while another thread pumps the scheduler through a raw call. A
    // quiesced run() still holds the owner (R1-mid C-m3), and from its quiesce on its thread
    // touches the scheduler only under this mutex (the reap's erase), apart from reap()'s
    // cancel_async_calls_of, which touches only its ended program's calls; raw calls from other
    // threads raise on the owner. So a call during the quiesce acts under the mutex alone.
    std::optional<async_scheduler::pump_owner_scope> owner;
    if (!running)
      owner.emplace(sched);
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
  // Its locks are the app's (docs/STATE_EXEC_LOCKS.md 8.4, 9.2): one namespace for every Exec and
  // the app's exec scheduler, the home attached now. An Exec may run() on any Python thread, so
  // its home attaches as a threaded host's: cross-thread wherever the build has threads, which a
  // single-mode manager refuses, naming state_exec.locks.threading (8.3). Without threads (the
  // static Emscripten pycvc) auto made the manager single, and it takes that attach (9.5). An app a
  // host made hears of the threaded host here, before its manager is made, if it isn't yet.
  cvc::state_exec::detail::note_threaded_host(*app);
  impl_->sched.set_lock_manager(app->lock_manager(), cvc::state_exec::threaded_host_attach());
  impl_->own_host = std::make_unique<cvc::state_exec::async_host>(impl_->sched, app.get());
  impl_->init_context("", builtins::make_default_environment(), impl_->own_host.get());
}

Exec::Exec(pooled_tag, const std::shared_ptr<cvc::app> &app, const std::string &name,
           double idle_timeout) {
  cvc::state &root = cvc::state::instance(*app);
  // The group builds its scheduler as the app's is built (the policy, the watch root, the parser
  // and data defaults, the lock attach in the manager's own mode), with the app pool's pid source
  // and its own registry and random stream (S1.5).
  cvc::state_exec::detail::note_threaded_host(*app); // before the group makes the lock manager
  exec_group::options opts;
  opts.name = name;
  opts.app = app.get();
  impl_ = std::make_shared<ExecImpl>(std::make_unique<exec_group>(std::move(opts)));
  impl_->app = app;
  impl_->root = &root;
  impl_->idle_timeout = idle_timeout;
  app->computePool();
  // Chrooted at its name (td 4.8: an unchrooted Exec locks from the app root), over the frozen
  // builtins rather than a default environment of its own (S1.3).
  impl_->init_context(name, environment::extend(frozen_builtins(*app)), &impl_->group->host());
}

exec_group &Exec::group() const { return *impl_->group; }

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

void Exec::register_fn(const std::string &name, PyObject *callable, bool with_context) {
  check_pump_thread(impl_->sched);
  builtins::register_fn(impl_->natives, name,
                        python_native_fn("pycvc.Exec.register_fn", name, callable, with_context));
}

void Exec::register_async_fn(const std::string &name, PyObject *callable, bool with_call) {
  // An async function on this Exec's host (docs/STATE_EXEC_EXCEPTIONS.md 3): (name args...) parks
  // the program while the callable runs on the app's background task pool, then resumes it with
  // the value or raises the callable's exception at the call (a PythonError). (name-async ...)
  // returns a future.
  check_pump_thread(impl_->sched);
  cvc::state_exec::register_async_fn(
      impl_->natives, *impl_->host, name,
      python_async_marshal("pycvc.Exec.register_async_fn", name, callable, with_call));
}

cvc::state_exec::async_host &Exec::async_host() { return *impl_->host; }

void Exec::register_async_coro(const std::string &name, PyObject *coro_fn) {
  check_pump_thread(impl_->sched);
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
  const auto start = [impl, holder, name](call_args args, bool future) -> value_t {
    value_t result;
    const std::shared_ptr<async_call> call =
        impl->host->begin_external(args.context(), name, future, result);
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
                          [start, future](call_args args) -> value_t {
                            value_t argv =
                                make_list(std::vector<value_t>(args.begin(), args.end()));
                            exec_context *const ctx = args.context();
                            auto then = [start, future, ctx](value_t copied) -> value_t {
                              const std::vector<value_t> &items = *std::get<list_ptr>(copied.v);
                              return start(call_args(items, ctx), future);
                            };
                            if (std::none_of(args.begin(), args.end(),
                                             [](const value_t &a) { return is_compound(a); }))
                              return then(std::move(argv));
                            return run_data_job(
                                ctx,
                                make_copy_job(std::move(argv), copy_mode::share, "async arguments"),
                                std::move(then));
                          });
}

std::string Exec::run(const std::string &src) {
  ExecImpl *const impl = impl_.get();
  // One Exec is driven by ONE thread. A registered fn/coro that calls back into run() (or a second
  // thread, or a kernel while this run() quiesces) would corrupt the shared scheduler + asyncio
  // loop, so reject re-entrancy up front. The test and set are under the control mutex, taken with
  // the GIL held (the one order allowed, docs/STATE_EXEC_LOCKS.md 7.4), and so is what makes this
  // run() the active one before its first step: from here another thread's priority call waits
  // for this run() to answer it between steps. This thread is the scheduler's pump owner from
  // here until run() returns (S1-Q4); while another thread pumps it through a raw call, run()
  // raises.
  {
    std::lock_guard<std::mutex> lock(impl->control_mutex);
    if (impl->running)
      throw std::runtime_error("pycvc.Exec.run: re-entrant call — a registered fn/coro must not "
                               "call run() (use a separate Exec per thread)");
    impl->run_pump_owner.emplace(impl->sched);
    impl->running = true;
    impl->run_thread = std::this_thread::get_id();
    impl->accepting_queries = true;
  }
  const void *const outer_run = running_exec(); // a run() of another Exec this one nests in
  set_running_exec(impl);
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
      {
        // The thread's Python state, which needs the GIL (whatever path this unwinds on).
        PyGILState_STATE gil = PyGILState_Ensure();
        set_running_exec(outer_run);
        PyGILState_Release(gil);
      }
      // Last, under the control mutex: until here a second run() raises, and the pump owner is
      // this thread's.
      std::lock_guard<std::mutex> lock(impl->control_mutex);
      impl->run_pump_owner.reset();
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
  std::vector<int> lock_waiters; // the idle rule's lock-waiting pids, reused across passes
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
      // to outlive us. Work pending here is this home's progress too (docs/STATE_EXEC_LOCKS.md
      // 8.4), which a waiter on another Exec behind one of our processes reads. A lock wait of
      // any of our processes is pending only while another home it waits for makes progress
      // (9.2), as of that progress: the idle window runs from it, so a wait behind a home that
      // stopped is killed about idle_timeout after its last progress. It stores none, so two
      // Execs whose waiters wait on each other's parked holders can't keep each other alive.
      // A post in the lock port (a grant made on another thread after this pass's drain) is work
      // pending too: the next pass applies it. The Exec's lock home is read where it is used, never
      // kept across the loop.
      lock_home &home = *sched.locks();
      if (host.in_flight() > 0 || !impl_->pending_coros.empty() || sched.has_runnable() ||
          had_runnable || home.port_pending()) {
        const clock::time_point now = clock::now();
        idle_deadline = cvc::state_exec::detail::time_after(now, idle_timeout);
        home.store_progress(now);
      } else {
        // The lock rule, only while a process of ours waits for a lock: one search from all of
        // them (lock_waiters, reused across passes).
        lock_waiters.clear();
        if (sched.lock_waiting_count() > 0) {
          sched.lock_waiting_pids(lock_waiters);
          if (const std::optional<clock::time_point> progress =
                  progress_waited_for(home, lock_waiters, idle_timeout, clock::now()))
            idle_deadline = std::max(idle_deadline,
                                     cvc::state_exec::detail::time_after(*progress, idle_timeout));
        }
        if (clock::now() > idle_deadline) {
          const std::optional<std::pair<int, std::string>> named =
              lock_wait_named(sched, *info, lock_waiters);
          sched.kill(pid);
          error = "pycvc.Exec.run: parked with no pending work (pid " + std::to_string(pid);
          if (named) // ", waiting for lock k" for our own wait, else ", pid 2 waiting for lock k"
            *error +=
                (named->first == pid ? std::string(",") : ", pid " + std::to_string(named->first)) +
                " waiting for lock " + lock_home::show(named->second, impl->ictx.root_path);
          *error += ")";
          break;
        }
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

PyObject *Exec::lock_query(const std::string &prefix) {
  // (lock-query prefix)'s reading of the prefix: "" (or blanks) is every key under the Exec's
  // root, anything else a key as a program writes it (app-global with a leading '/').
  ExecImpl *const impl = impl_.get();
  const intrinsics_context &ctx = impl->ictx;
  std::string abs = ctx.root_path;
  if (prefix.find_first_not_of(" \t\r\n\f\v") != std::string::npos)
    abs = cvc::state_exec::detail::resolve_lock_key(ctx, prefix, "Exec.lock_query");
  // Straight to the manager, from any thread (docs/STATE_EXEC_LOCKS.md 7.4): it never waits for
  // run() or the control mutex, only for the manager's mutex, which nothing holds while it waits
  // for the GIL. The home is the viewer (only this Exec's pids show, owner ""): its manager and id
  // are copied while the GIL is held, and the home itself is never touched without it. The GIL is
  // released for the query and taken back once it has returned, and only then are Python objects
  // made.
  const lock_home &home = *impl->sched.locks();
  const std::shared_ptr<lock_manager> manager = home.manager_ptr();
  const cvc::state_exec::detail::home_id viewer = home.id();
  lock_query_result r;
  {
    gil_released nogil;
    r = manager->table().query(abs, viewer, "");
  }
  return value_to_py(lock_query_value(r, ctx.root_path));
}

cvc::state_exec::parse_options Exec::parser_defaults() const {
  return impl_->sched.parser_defaults();
}

void Exec::set_parser_defaults(const cvc::state_exec::parse_options &defaults) {
  check_pump_thread(impl_->sched);
  impl_->sched.set_parser_defaults(defaults);
}

cvc::state_exec::data_options Exec::data_defaults() const { return impl_->sched.data_defaults(); }

void Exec::set_data_defaults(const cvc::state_exec::data_options &defaults) {
  check_pump_thread(impl_->sched);
  impl_->sched.set_data_defaults(defaults);
}

cvc::state_exec::async_scheduler &Exec::scheduler() { return impl_->sched; }

// The Exec's own registry (S1.12), whichever thread ran its programs. Its steps change it, so this
// is the pump thread's too.
std::size_t Exec::tracked_objects() const {
  check_pump_thread(impl_->sched);
  return cvc::state_exec::tracked_objects(impl_->group ? *impl_->group->registry()
                                                       : *impl_->registry);
}

cvc::state_exec::cycle_stats Exec::collect_cycles() {
  // As the pump owner (S1-Q4) for the whole collection: it raises while another thread pumps,
  // and no run() starts meanwhile, with the GIL released while it walks.
  async_scheduler::pump_owner_scope owner(impl_->sched);
  cycle_registry &r = impl_->group ? *impl_->group->registry() : *impl_->registry;
  gil_released nogil;
  return cvc::state_exec::collect_cycles(r);
}

std::vector<Exec::batch_outcome> Exec::run_batch(cvc::exec_pool &pool, std::span<Exec *const> execs,
                                                 std::span<const std::string> programs,
                                                 bool fresh_env, const std::atomic<bool> &closing) {
  using clock = std::chrono::steady_clock;
  // A group's slice per round, as run()'s.
  static constexpr uint64_t kSliceSteps = 2000000;
  static constexpr double kSliceSeconds = 0.05;
  // With nothing runnable, the wait on one parked group, rotating, so that a completion posted to
  // any group is seen within about this long (S1.12; Stage 2's eventcount replaces it). It is
  // also the frame beat, run()'s 1 ms: a process awaiting the next frame gets one per round.
  static constexpr auto kIdleBeat = std::chrono::milliseconds(1);
  std::vector<batch_outcome> out(programs.size());
  struct slot {
    ExecImpl *impl;
    std::deque<std::size_t> queue; // its programs not started yet, in order
    std::size_t prog = 0;          // the program running
    int pid = -1;                  // its process; -1 while none runs
    bool had_runnable = false;     // something was runnable after the round's drain and wake
    clock::time_point idle_deadline{};
  };
  std::vector<slot> slots;
  for (Exec *e : execs)
    slots.push_back({e->impl_.get(), {}});
  if (slots.empty())
    return out;
  for (std::size_t i = 0; i < programs.size(); ++i)
    slots[i % slots.size()].queue.push_back(i);
  const auto fail = [&](std::size_t prog, const std::string &what) {
    out[prog].error = std::make_exception_ptr(std::runtime_error(what));
  };
  const auto pid_text = [](int pid) { return " (pid " + std::to_string(pid) + ")"; };
  static constexpr const char *kClosed = "pycvc.ExecPool.run_all: the pool was closed";
  // The program ended: its calls cancelled, its process reaped and freed here, the GIL released
  // (what it holds of Python is freed under the GIL its owner takes).
  const auto reap = [](slot &s) {
    s.impl->sched.cancel_async_calls_of(s.pid);
    process_ptr gone;
    s.impl->sched.reap(s.pid, &gone);
    s.pid = -1;
  };
  std::vector<exec_group *> groups;
  std::vector<int> lock_waiters;
  std::size_t rotate = 0;
  // After a round, run()'s tests on slot `s`'s program: ended (its outcome set, its process
  // reaped), runnable (true), or parked, killed past its idle limit.
  const auto settle = [&](slot &s) -> bool {
    async_scheduler &sched = s.impl->sched;
    const int pid = s.pid;
    if (std::optional<value_t> result = sched.get_result(pid)) {
      out[s.prog].result = render_result(*result);
      reap(s);
      return false;
    }
    const std::optional<process_info> info = sched.get_process_info(pid);
    if (!info) {
      fail(s.prog, "pycvc.ExecPool.run_all: process vanished" + pid_text(pid));
      s.pid = -1;
      return false;
    }
    if (info->status == process_status::failed) {
      // Its uncaught exception, as run() raises it.
      try {
        sched.raise_exit_exception(pid);
        fail(s.prog, "pycvc.ExecPool.run_all: program produced no result" + pid_text(pid));
      } catch (const dsl_exception &) {
        out[s.prog].error = std::current_exception();
      }
      reap(s);
      return false;
    }
    if (info->status == process_status::killed) {
      const std::optional<std::string> ex = sched.get_exit_error(pid);
      const std::string error =
          ex && !ex->empty() ? *ex : "pycvc.ExecPool.run_all: program killed" + pid_text(pid);
      reap(s);
      // A syntax error read inside the process is only its exit error's text: re-parsed, it
      // raises pycvc.parse_error with its position, as run() does.
      try {
        (void)parse_all(programs[s.prog],
                        parse_options_from_state(*s.impl->root, {}, sched.parser_defaults()));
        fail(s.prog, error);
      } catch (...) {
        out[s.prog].error = std::current_exception();
      }
      return false;
    }
    if (closing.load(std::memory_order_acquire)) {
      // ExecPool.close() from another thread: what still runs is killed.
      sched.kill(pid);
      fail(s.prog, kClosed);
      reap(s);
      return false;
    }
    if (info->status == process_status::ready || info->status == process_status::running) {
      s.idle_deadline = cvc::state_exec::detail::time_after(clock::now(), s.impl->idle_timeout);
      return true;
    }
    // Parked: run()'s idle rule (docs/STATE_EXEC_LOCKS.md 8.4, 9.2).
    lock_home &home = *sched.locks();
    const double idle_timeout = s.impl->idle_timeout;
    if (s.impl->host->in_flight() > 0 || sched.has_runnable() || s.had_runnable ||
        home.port_pending()) {
      const clock::time_point now = clock::now();
      s.idle_deadline = cvc::state_exec::detail::time_after(now, idle_timeout);
      home.store_progress(now);
      return false;
    }
    lock_waiters.clear();
    if (sched.lock_waiting_count() > 0) {
      sched.lock_waiting_pids(lock_waiters);
      if (const std::optional<clock::time_point> progress =
              progress_waited_for(home, lock_waiters, idle_timeout, clock::now()))
        s.idle_deadline =
            std::max(s.idle_deadline, cvc::state_exec::detail::time_after(*progress, idle_timeout));
    }
    if (clock::now() > s.idle_deadline) {
      const std::optional<std::pair<int, std::string>> named =
          lock_wait_named(sched, *info, lock_waiters);
      sched.kill(pid);
      std::string error =
          "pycvc.ExecPool.run_all: parked with no pending work (pid " + std::to_string(pid);
      if (named)
        error +=
            (named->first == pid ? std::string(",") : ", pid " + std::to_string(named->first)) +
            " waiting for lock " + lock_home::show(named->second, s.impl->ictx.root_path);
      fail(s.prog, error + ")");
      reap(s);
    }
    return false;
  };
  try {
    for (;;) {
      // Each pooled Exec with no program running starts its next one. One that cannot start (a
      // syntax error, with parser.step_tokens 0 on the tree) has its error at once.
      for (slot &s : slots) {
        while (s.pid < 0 && !s.queue.empty()) {
          s.prog = s.queue.front();
          s.queue.pop_front();
          if (closing.load(std::memory_order_acquire)) {
            fail(s.prog, kClosed);
            continue;
          }
          try {
            execute_options opts;
            opts.name = "pycvc";
            opts.env = fresh_env ? environment::extend(s.impl->natives) : s.impl->env;
            s.pid = s.impl->group->execute(programs[s.prog], opts);
            s.idle_deadline =
                cvc::state_exec::detail::time_after(clock::now(), s.impl->idle_timeout);
          } catch (...) {
            out[s.prog].error = std::current_exception();
          }
        }
      }
      groups.clear();
      for (slot &s : slots)
        if (s.pid >= 0) {
          // The round's drain and wake first, here between pumps, then the runnable test, as
          // run() orders them: a process they ready (a frame-yield's await, a kernel's result) is
          // pending work for the idle rule (R1-mid S-M2). The pump's own drain and wake then find
          // only what arrives meanwhile.
          async_scheduler &sched = s.impl->sched;
          sched.drain_ingress();
          sched.wake_awaiting();
          s.had_runnable = sched.has_runnable();
          groups.push_back(s.impl->group.get());
        }
      if (groups.empty())
        break;
      // One fork-join round: each group pumped once, on the pool's workers and this thread.
      pool.pump_all(groups, pump_budget{kSliceSteps, kSliceSeconds}, clock::time_point::max());
      bool runnable = false;
      for (slot &s : slots)
        if (s.pid >= 0)
          runnable = settle(s) || runnable;
      if (runnable)
        continue;
      // Pump again at once only for what a pump would do now: a runnable process, or
      // (wait_for_work's checks) pending ingress, a lock post, a timer due or a call to charge. A
      // process waiting for the next frame ((await v)) waits for it, at most kIdleBeat, as run()
      // makes it; wants_pump() counts it, and pumping it at once would spin a core (R1, N2).
      bool work = false;
      clock::time_point until = clock::now() + kIdleBeat;
      std::size_t parked = 0;
      for (slot &s : slots)
        if (s.pid >= 0) {
          async_scheduler &sched = s.impl->sched;
          work = work || sched.has_runnable() || sched.wait_for_work(clock::time_point::min());
          until = std::min(until, s.idle_deadline);
          ++parked;
        }
      if (work || parked == 0)
        continue;
      // Nothing to pump yet: wait on one parked group, the next each time.
      std::size_t k = rotate++ % parked;
      for (slot &s : slots)
        if (s.pid >= 0 && k-- == 0) {
          s.impl->sched.wait_for_work(until);
          break;
        }
    }
  } catch (...) {
    // As run()'s catch: what runs is killed, no kernel outlives the batch, and the processes are
    // reaped.
    for (slot &s : slots)
      if (s.pid >= 0)
        try {
          s.impl->sched.kill(s.pid);
        } catch (...) {
        }
    for (Exec *e : execs)
      e->impl_->host->wait_idle();
    for (slot &s : slots)
      if (s.pid >= 0)
        try {
          reap(s);
        } catch (...) {
        }
    throw;
  }
  // No kernel of the batch outlives it.
  for (Exec *e : execs)
    e->impl_->host->wait_idle();
  return out;
}

} // namespace pycvc
