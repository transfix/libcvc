// pycvc_exec.h — run state_exec DSL programs in an app's context, and register
// Python callables as DSL functions.
//
// Exec is a HANDLE bound to an explicit app (no module-global): it owns the DSL
// environment + scheduler over that app's state root, so registered Python
// functions and run() programs all see the same shared tree. A Python function
// registered here becomes callable from DSL source like any builtin.
#pragma once

#include <cvc/state/state_exec/data_job.h> // data_options
#include <cvc/state/state_exec/parser.h>   // parse_options
#include <memory>
#include <string>

namespace cvc {
class app;
namespace state_exec {
class async_host;
class async_scheduler;
} // namespace state_exec
} // namespace cvc

// Forward-declare PyObject so this SWIG-visible header needs no <Python.h>.
struct _object;
typedef _object PyObject;

namespace pycvc {

class Exec {
public:
  // Build a DSL environment + scheduler over `app`'s state root.
  explicit Exec(const std::shared_ptr<cvc::app> &app);
  ~Exec();

  // Register a Python callable as a DSL function named `name`. DSL programs run
  // later can call it; args arrive as DSL values converted to Python
  // (int/float/bool/str/None/list/dict) and the return value converts back. A
  // Python exception inside it is contained and surfaced as a run() error, not
  // a crash. The callable is kept alive for this Exec's lifetime.
  //
  // SYNCHRONOUS: the callable runs inline on the thread that calls run(), with
  // the GIL held, and blocks the DSL program (and run()) until it returns. Use
  // register_async_fn for slow work that should not stall the scheduler.
  void register_fn(const std::string &name, PyObject *callable);

  // Register a Python callable as an ASYNC DSL function named `name`, on this
  // Exec's async host (async_host(); docs/STATE_EXEC_EXCEPTIONS.md 3). Calling
  // (name args...) in a DSL program runs the callable on the app's background
  // task pool and PARKS the calling program until it finishes, so the scheduler
  // stays live — other DSL processes (and other Python threads) run meanwhile.
  // The call works anywhere, the whole program included. The program resumes
  // with the callable's return value (converted back to a DSL value); if the
  // callable raises, the call raises a PythonError (`python_type` names the
  // Python exception's type), which a (try ...) can catch; uncaught, run()
  // re-raises the Python exception. (name-async args...) returns a future for
  // (await f [seconds]) / (await-all (list ...)).
  //
  // The callable runs on a pool thread with the GIL acquired for the call, so a
  // blocking request (urllib/requests) or a GIL-releasing compute (numpy) runs
  // truly concurrently; its wait for the GIL isn't charged to the program. It
  // gets copies of its arguments. It must NOT call back into this Exec (run() /
  // register*) or touch the DSL/state tree — it receives its args and returns a
  // value, nothing more.
  void register_async_fn(const std::string &name, PyObject *callable);

  // Register a Python COROUTINE function (an `async def`) as an async DSL function
  // named `name`. Calling (name args...) calls coro_fn(*args) to get a coroutine,
  // schedules it on an asyncio event loop this Exec owns, and PARKS the calling
  // program until it completes. Unlike register_async_fn (which offloads a blocking
  // callable to a pool WORKER), the coroutine runs ON the run() thread, cooperatively
  // stepped one slice per pump iteration — so it and the DSL march along together, and
  // its `await asyncio.sleep`/`aiohttp`/... progress between DSL slices. The program
  // resumes with the coroutine's return value; a raise raises a PythonError at the
  // call, as for register_async_fn. (name-async args...) returns a future.
  //
  // The coroutine MUST YIELD (use await). The slice is COOPERATIVE — asyncio cannot
  // preempt a running step — so a coroutine that does CPU-bound work without awaiting
  // (or an accidental non-awaiting loop) holds the run() thread and wedges the pump:
  // put blocking/CPU work in register_async_fn (the pool), not here. And, as with any
  // awaited async work, a coroutine whose await NEVER resolves leaves run() waiting
  // (the same as a never-returning register_async_fn callable).
  void register_async_coro(const std::string &name, PyObject *coro_fn);

  // This Exec's async host: register more async functions on it with
  // pycvc.builtins.register_async_fn(env, host, ...). In Python it keeps this Exec alive.
  cvc::state_exec::async_host &async_host();

  // Execute a DSL program in this app's context; returns the rendered result
  // (strings raw, nil as "", others via the DSL's printed form). Throws on a
  // parse/eval error (surfaced as a Python exception). Drives this Exec's PRIVATE
  // async scheduler to completion, so a program that awaits an async fn parks and
  // resumes here; the GIL is released while idle-waiting so pool workers run, and
  // run() does not return until every worker it launched has finished.
  //
  // NOT thread-safe: one Exec is driven by ONE thread. Do not call run() (or
  // register*) concurrently on the same Exec; use a separate Exec per thread
  // (each owns its own scheduler, so they do not interfere).
  std::string run(const std::string &src);

  // The parser settings this Exec's scheduler parses programs with when the app tree's
  // state_exec.defaults.parser.* keys don't say otherwise; the same calls as
  // async_scheduler::parser_defaults() / set_parser_defaults(). They start as
  // async_scheduler::host_parser_defaults(), as the app's exec scheduler's do.
  cvc::state_exec::parse_options parser_defaults() const;
  void set_parser_defaults(const cvc::state_exec::parse_options &defaults);

  // The deep-data settings, likewise: async_scheduler::data_defaults() / set_data_defaults(),
  // starting as async_scheduler::host_data_defaults().
  cvc::state_exec::data_options data_defaults() const;
  void set_data_defaults(const cvc::state_exec::data_options &defaults);

  // This Exec's private scheduler, the one run() drives. A program submitted to it directly
  // (scheduler().execute(...)) runs alongside run()'s programs, and run() steps it too. In Python
  // the returned scheduler keeps this Exec alive.
  cvc::state_exec::async_scheduler &scheduler();

private:
  struct ExecImpl;
  std::shared_ptr<ExecImpl> impl_;
};

} // namespace pycvc
