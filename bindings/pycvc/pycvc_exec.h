// pycvc_exec.h — run state_exec DSL programs in an app's context, and register
// Python callables as DSL functions.
//
// Exec is a HANDLE bound to an explicit app (no module-global): it owns the DSL
// environment + scheduler over that app's state root, so registered Python
// functions and run() programs all see the same shared tree. A Python function
// registered here becomes callable from DSL source like any builtin.
#pragma once

#include <cstddef>
#include <cvc/state/state_exec/data_job.h> // data_options
#include <cvc/state/state_exec/heap.h>     // cycle_stats
#include <cvc/state/state_exec/parser.h>   // parse_options
#include <memory>
#include <string>
#ifndef SWIG
#include <atomic>
#include <exception>
#include <span>
#include <vector>
#endif

namespace cvc {
class app;
class exec_pool;
namespace state_exec {
class async_host;
class async_scheduler;
class exec_group;
} // namespace state_exec
} // namespace cvc

// Forward-declare PyObject so this SWIG-visible header needs no <Python.h>.
struct _object;
typedef _object PyObject;

namespace pycvc {

class ExecPool;

class Exec {
public:
  // Build a DSL environment + scheduler over `app`'s state root. Python takes keywords:
  // Exec(app, policy=None, idle_timeout=60.0).
  //
  // `policy` is the scheduler's scheduling policy, fixed for its life (docs/STATE_EXEC_LOCKS.md
  // 7.3): None reads the app tree's state_exec.policy (round_robin when it is unset; an unknown
  // value is logged and round_robin used, as app::exec_scheduler() does); otherwise a
  // pycvc.scheduling_policy member or its name ("round_robin", "priority", "priority_rr").
  // `idle_timeout` is how long, in seconds, run() waits while its program is parked with nothing
  // pending before it kills it. A lock wait is pending while the process it waits for is another
  // Exec's (or scheduler's) that made progress within idle_timeout (docs/STATE_EXEC_LOCKS.md
  // 9.2). Anything else, or a negative or NaN idle_timeout, raises ValueError
  // (std::invalid_argument).
  //
  // Its with-lock keys are the app's (app::lock_manager()): every Exec of an app, and the app's
  // exec scheduler, share one lock namespace. With state_exec.locks.threading set to single it
  // raises RuntimeError naming that setting, since an Exec may run() on any Python thread.
  explicit Exec(const std::shared_ptr<cvc::app> &app, PyObject *policy = nullptr,
                double idle_timeout = 60.0);
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
  //
  // With with_context, the callable gets the call's pycvc.call_context first,
  // before the arguments: what pycvc.eval_alloc_check, run_data_job,
  // call_function and count_data_nodes take to act in the call (a lambda called
  // through call_function runs as the program's own frames).
  void register_fn(const std::string &name, PyObject *callable, bool with_context = false);

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
  // value, nothing more. With `with_call`, it gets the call's pycvc.async_call
  // handle first, before the arguments: call.cancelled() says whether to stop,
  // and call.post(value) sends progress to "<name>.progress".
  void register_async_fn(const std::string &name, PyObject *callable, bool with_call = false);

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
  // (each owns its own scheduler; their with-lock keys are the app's, shared). A
  // second run() of the same Exec, from any thread, raises RuntimeError until the
  // first has returned. set_priority, get_priority and lock_query are the
  // exceptions: any thread may call them.
  //
  // The pump owner (S1-Q4): run() is its scheduler's pump owner until it
  // returns. Meanwhile register_fn, register_async_fn, register_async_coro,
  // set_parser_defaults, set_data_defaults, tracked_objects, collect_cycles and
  // scheduler()'s calls (all but stop, is_running and post_message) raise RuntimeError
  // ("scheduler: another thread is pumping it") on any other thread, a kernel's
  // included; from run()'s own thread (a registered function) they work. A
  // run() started while another thread pumps the scheduler through one of its
  // raw calls raises the same.
  std::string run(const std::string &src);

  // A process's priorities (docs/STATE_EXEC_LOCKS.md 7.4). set_priority(pid, p)
  // changes its base priority, unchecked, as a host does, and returns whether the
  // pid was found; get_priority(pid) returns {"base", "effective", "floor",
  // "policy"} (policy a pycvc.scheduling_policy) or None for a pid this Exec's
  // scheduler doesn't have (a reaped one included). Under round_robin a change is
  // stored and reported and changes no pick.
  //
  // THREADS: any thread may call them. While run() pumps on another thread they
  // are answered by run() between its steps, the caller waiting with the GIL
  // released; with no run() active, or from inside run() (a registered function,
  // coroutine or callback), they act at once. A call waiting as run() ends is
  // answered by run() before it returns. From a thread running ANOTHER Exec's
  // run() they raise RuntimeError("Exec.run is active on another thread"), so two
  // Execs whose functions call each other can't wait for each other.
  bool set_priority(int pid, int priority);
  PyObject *get_priority(int pid);

  // The app's locks under `prefix`, as (lock-query prefix) in this Exec's programs returns them
  // (docs/STATE_EXEC_LOCKS.md 4.1, 7.4): {"held": [{"key", "mode", "pids", "count", "generation",
  // "abandoned"}], "waiting": [{"key", "mode", "pid"}]}. "" is every key. Counts include every
  // Exec's holds; pids are shown only for this Exec's processes (its owner "" ones), since pids
  // repeat across Execs.
  //
  // THREADS: any thread may call it, and it never waits for run(): it reads the app's lock
  // manager directly, under the manager's own mutex, with the GIL released.
  PyObject *lock_query(const std::string &prefix = "");

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

  // Objects this Exec's cycle registry tracks (docs/STATE_EXEC_THREADED.md 4.7): its programs'
  // captured scopes and mutated containers, alive after the last collection or tracked since,
  // whichever thread ran them. The pump thread's, as register_fn is (run(), above).
  std::size_t tracked_objects() const;

  // Collect this Exec's cycle registry now, with the GIL released, and return what it did. Its
  // runs collect it as they go (every so many steps, and as they end, throttled); this frees its
  // garbage at once. pycvc.collect_cycles() collects the calling thread's own registry, which an
  // Exec's programs never use. The pump thread's, as register_fn is; from inside a step (a
  // registered function) it does nothing.
  cvc::state_exec::cycle_stats collect_cycles();

private:
  struct ExecImpl;
  std::shared_ptr<ExecImpl> impl_;

#ifndef SWIG
  // ExecPool's pooled Execs (S1.12, S1-Q5), never exposed to Python.
  friend class ExecPool;
  struct pooled_tag {};
  // Pooled Exec `name`, an Exec in group mode: its scheduler, async host, cycle registry and
  // random stream are an exec_group's (the app pool's pid source, the app's lock manager), its
  // natives extend the frozen builtins (td 4.2), and its context is chrooted at `name`.
  Exec(pooled_tag, const std::shared_ptr<cvc::app> &app, const std::string &name,
       double idle_timeout);
  // A pooled Exec's group.
  cvc::state_exec::exec_group &group() const;
  // A program's outcome in a batch: its result as run() renders it, or what run() would raise.
  struct batch_outcome {
    std::string result;
    std::exception_ptr error;
  };
  // ExecPool::run_all's batch, called with the GIL released: program i runs on
  // execs[i % execs.size()], each pooled Exec's programs one at a time and in order, in its env
  // (a fresh extend of its natives with `fresh_env`). It pumps their groups on `pool` in fork-join
  // rounds until every program has ended or passed its idle limit, then waits for their kernels.
  // Once `closing` is set, what runs is killed and what has not started is not started.
  static std::vector<batch_outcome> run_batch(cvc::exec_pool &pool, std::span<Exec *const> execs,
                                              std::span<const std::string> programs, bool fresh_env,
                                              const std::atomic<bool> &closing);
#endif
};

} // namespace pycvc
