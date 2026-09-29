// pycvc_exec.h — run state_exec DSL programs in an app's context, and register
// Python callables as DSL functions.
//
// Exec is a HANDLE bound to an explicit app (no module-global): it owns the DSL
// environment + scheduler over that app's state root, so registered Python
// functions and run() programs all see the same shared tree. A Python function
// registered here becomes callable from DSL source like any builtin.
//
// Guarded by CVC_STATE_EXEC (a PUBLIC compile def on the cvc target); when the
// build lacks state_exec, the ctor throws.
#pragma once

#include <memory>
#include <string>

namespace cvc {
class app;
}

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

  // Register a Python callable as an ASYNC DSL function named `name`. Calling
  // (name args...) in a DSL program OFFLOADS the callable to the app's compute
  // pool and PARKS the calling program until it finishes, so the scheduler stays
  // live — other DSL processes (and other Python threads) run meanwhile. The
  // program resumes with the callable's return value (converted back to a DSL
  // value); if the callable raises, it resumes with a {"__async_error__":
  // <message>} dict — a reserved key that a normal dict result cannot collide
  // with, so a program can tell an error from data.
  //
  // The callable runs on a pool WORKER thread with the GIL acquired for the call,
  // so a blocking request (urllib/requests) or a GIL-releasing compute (numpy)
  // runs truly concurrently. It must NOT call back into this Exec (run() /
  // register*) or touch the DSL/state tree — it receives its args and returns a
  // value, nothing more. `(name ...)` must be nested (not the whole program) so
  // the park has an enclosing frame — e.g. (begin (name ...)) or
  // (state-set "r" (name ...)).
  void register_async_fn(const std::string &name, PyObject *callable);

  // Register a Python COROUTINE function (an `async def`) as an async DSL function
  // named `name`. Calling (name args...) calls coro_fn(*args) to get a coroutine,
  // schedules it on an asyncio event loop this Exec owns, and PARKS the calling
  // program until it completes. Unlike register_async_fn (which offloads a blocking
  // callable to a pool WORKER), the coroutine runs ON the run() thread, cooperatively
  // stepped one slice per pump iteration — so it and the DSL march along together, and
  // its `await asyncio.sleep`/`aiohttp`/... progress between DSL slices. The program
  // resumes with the coroutine's return value; a raise resumes it with a
  // {"__async_error__": <message>} dict (do not return a dict using that reserved key).
  //
  // The coroutine MUST YIELD (use await). The slice is COOPERATIVE — asyncio cannot
  // preempt a running step — so a coroutine that does CPU-bound work without awaiting
  // (or an accidental non-awaiting loop) holds the run() thread and wedges the pump:
  // put blocking/CPU work in register_async_fn (the pool), not here. And, as with any
  // awaited async work, a coroutine whose await NEVER resolves leaves run() waiting
  // (the same as a never-returning register_async_fn callable). Same nesting rule as
  // register_async_fn.
  void register_async_coro(const std::string &name, PyObject *coro_fn);

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

private:
  struct ExecImpl;
  std::shared_ptr<ExecImpl> impl_;
};

} // namespace pycvc
