// script_engine.h — the scripting engines' seam: what a language engine (Python in arix, ariglx and
// arisrvx; later Lua and WebAssembly guests) gets from a document and from the runtime that runs it
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3.1 and §8.3–8.7).
//
// An engine is registered once per process (register_script_engine), as its interpreter is booted
// once. That makes its language a block key (`python:`) the loader keeps, and its tokens hold:
// `python`, and `python.<module>` for a module has_module() finds. A runtime then runs the blocks
// of the engines it was given (AppRuntime::use_script_engine): at every load, after the parse and
// before requires: is checked and init: runs, it hands each block to its engine's load_block()
// with the document's ScriptLane.
//
// The lane is everything an engine reaches the document through, and it is the program lane's own,
// so an engine gets what a DSL program of the document gets and no more (§8.6):
//   - functions: one registry. An export is a host function (AppRuntime::register_function), which
//     every program of the document can call and which requires: and the load's scan see. Python's
//     exports reach the same registry through pycvc.host_function.
//   - hooks (§8.7): an on: event, every frame, a value that changes. Each delivery is an
//     activation, run under the lane the DSL would run it in: an event as an action, a frame or a
//     change as a resident, one at a time.
//   - activations (§8.5): an engine's long work runs as processes on app.exec_scheduler(), built
//     like a sliced task (cvc/core/sliced_task.h). The process is a small DSL loop around a native
//     that steps the activation one slice at a time:
//
//       (let ((k (script-step)))
//         (while (not (= k "done"))
//           ...                 ; "yield": (await #f); "await": (await f); "call": (apply f args)
//           (set k (script-step))))
//       (script-finish)
//
//     It is one of the document's programs (Runtime::start_process): its owner tag, so it is reaped
//     with the document; writes chrooted to the document's prefix; the document's channel policy
//     and lock manager; and the lane's caps from ariadne.limits.* (an action's, or a resident's
//     per-step cap). A slice charges the work it did as DSL steps, so the step cap is the
//     activation's gas cap. The activations of one language share a time budget per frame,
//     ariadne.limits.<language>_frame_budget (0.008 s): once it is spent, the rest yield to the
//     next frame. A function the activation calls (call()) is called by the loop, in the process,
//     so it is exactly what a DSL program there could call, with the same chroot, link: holes and
//     locks, and a call that waits parks the process as it would park a program.
//
// Everything here runs on the thread that drains the runtime, except has_module(), which any
// thread may ask.
#ifndef CVC_ARIADNE_SCRIPT_ENGINE_H
#define CVC_ARIADNE_SCRIPT_ENGINE_H

#include <cstdint>
#include <cvc/ariadne/ariadne.h> // ProgramLane
#include <cvc/ariadne/value.h>
#include <cvc/state/state_exec/async_call.h> // async_marshal
#include <cvc/state/state_exec/types.h>      // value_t, native_fn
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {

class AppRuntime;
class ScriptLane;

// A document's block in an engine's language, as the load found it.
struct ScriptBlock {
  std::string language; // its key, and the engine's: "python"
  Value content;        // as written: a bare string, or a map
  std::string document; // the document's path, as load() was given it
  std::string dir;      // its directory, absolute: where the block's relative paths resolve
  std::string prefix;   // the state prefix the document's programs run under
  std::string module;   // "" for the top document; a load:ed module's name once module blocks run
};

// A language engine.
class ScriptEngine {
public:
  virtual ~ScriptEngine() = default;
  // Its language: the block's key, the requires: token, and the head of `<language>.<module>`.
  virtual std::string language() const = 0;
  // `<language>.<module>`: whether `module` is there to use (Python: importable). Any thread.
  virtual bool has_module(const std::string &module) const {
    (void)module;
    return false;
  }
  // Run a document's block (at every load, a reload too): register what it exports and its hooks
  // through `lane`, which stays valid while the runtime lives. Throw to fail the load; push onto
  // `warnings` to warn.
  virtual void load_block(const ScriptBlock &block, ScriptLane &lane,
                          std::vector<std::string> &warnings) = 0;
  // The document `lane` ran is going: another load is about to run the blocks again, or the
  // runtime is going. Let go of what load_block made for it. The lane's hooks are dropped for
  // the engine; its activations still running go on unless it stops them.
  virtual void unload(ScriptLane &lane) { (void)lane; }
};

// Register `engine` for this process, by its language, replacing any engine of that language.
// A host does it once its interpreter is up, and unregisters before the interpreter goes.
void register_script_engine(std::shared_ptr<ScriptEngine> engine);
// Unregister `language`'s engine; its tokens stop holding. Runtimes that use it keep it.
void unregister_script_engine(const std::string &language);
// The engine registered for `language`, or null.
std::shared_ptr<ScriptEngine> find_script_engine(const std::string &language);

// ── activations ─────────────────────────────────────────────────────────────────────────────────

// One slice of an activation: what the slice may spend, what came back from what the last slice
// asked for, and how this one ends.
class ScriptSlice {
public:
  // The work the slice may do, in DSL steps: ariadne.limits.<language>_slice_steps (64), or the
  // activation's own.
  std::uint64_t budget() const { return budget_; }

  // What the last slice's call() or await() came back with: its value, or the exception it raised
  // (exception() is the exception value, error() "Class: message"). False on the first slice and
  // after a yield.
  bool has_reply() const { return replied_; }
  const cvc::state_exec::value_t &reply() const { return reply_; }
  bool failed() const { return !error_.empty(); }
  const std::string &error() const { return error_; }
  const cvc::state_exec::value_t &exception() const { return exception_; }

  // The work this slice did, in DSL steps: charged to the process, so the lane's step cap bounds
  // the activation. Adds up over a slice.
  void charge(std::uint64_t steps) { charged_ += steps; }

  // How the slice ends, at most one of these; with none, the activation has more to do and the
  // next slice runs at once (in the same drain, budgets allowing).
  //   finish: the activation is done, with `result`.
  //   yield_frame: give the frame back; the next slice runs in the next frame.
  //   await: park until `future` (an async call's, or a channel future) settles; the next slice
  //     gets its value or its exception.
  //   call: call `function` (any name the lane holds: a native, a host function, a document
  //     function) with `args`, as a program would; the next slice gets its value or its exception.
  //     Narrowed by ActivationOptions::may_call: a refused name comes back as a PermissionError.
  void finish(cvc::state_exec::value_t result = cvc::state_exec::value_t());
  void yield_frame();
  void await(cvc::state_exec::value_t future);
  void call(std::string function, std::vector<cvc::state_exec::value_t> args);

private:
  friend struct ActivationRunner;
  friend class SlicedCall;
  enum class Next { more, finish, yield, await, call };
  std::uint64_t budget_ = 0;
  bool replied_ = false;
  cvc::state_exec::value_t reply_;
  std::string error_;
  cvc::state_exec::value_t exception_;
  std::uint64_t charged_ = 0;
  Next next_ = Next::more;
  cvc::state_exec::value_t out_; // finish's result, or await's future
  std::string function_;
  std::vector<cvc::state_exec::value_t> args_;
};

// An activation (§8.5): one export call, one hook delivery, one guest's init. The core steps it,
// one slice per step() of its process.
class ScriptActivation {
public:
  virtual ~ScriptActivation() = default;
  // Do one slice of the work, within slice.budget(), and say how it ended (ScriptSlice). A throw
  // raises in the process: a se::dsl_error with its class (a built-in one, as PythonError is),
  // anything else as state_exec maps a C++ exception. Nothing catches it there, so the activation
  // fails with it.
  virtual void step(ScriptSlice &slice) = 0;
  // Its process ended before it finished: killed (a cap, stop(), the document going) or failed.
  // No step() follows.
  virtual void cancel() {}
};

// How an activation ended, for ActivationOptions::on_done.
struct ActivationOutcome {
  bool ok = false;                // it finished
  cvc::state_exec::value_t value; // what it finished with
  // When not ok, why: "failed: Class: message (at ...)", or "did not complete: max_steps_exceeded".
  std::string error;
  // The exception a step raised, when one did (else nil: a cap or a kill ended it).
  cvc::state_exec::value_t exception;
};

struct ActivationOptions {
  std::string name; // as (ps) shows it: "<language>/<subject>", say "wasm/scorer:score_route"
  ProgramLane lane = ProgramLane::action; // whose caps; a hook's kind decides it for a hook
  std::string scope;                      // its chroot: "" = the document's prefix
  // Caps of its own, which only ever lower the lane's: 0 leaves the lane's.
  std::uint64_t max_steps = 0;
  double max_time = 0.0;
  std::uint64_t max_memory = 0;
  int priority = 0;
  std::uint64_t slice_steps = 0; // 0: ariadne.limits.<language>_slice_steps
  // The names call() may reach (§8.6's imports:): null reaches all the lane holds.
  std::function<bool(const std::string &function)> may_call;
  // Once it has ended, on the drain's thread.
  std::function<void(const ActivationOutcome &)> on_done;
};

// What a hook makes of one event: the activation to deliver it to, or null to let it pass.
using ScriptHook =
    std::function<std::shared_ptr<ScriptActivation>(const cvc::state_exec::value_t &event)>;

// What an exported function makes of one call: the activation that answers it, given the call's
// arguments (private copies), or null to answer nil.
using ScriptCall = std::function<std::shared_ptr<ScriptActivation>(
    std::span<const cvc::state_exec::value_t> args)>;

// ── the lane ────────────────────────────────────────────────────────────────────────────────────

// A runtime's document as one engine reaches it: AppRuntime::use_script_engine makes one for each
// engine it runs, and it lives as long as the runtime.
class ScriptLane {
public:
  ScriptLane(AppRuntime &runtime, std::string language);
  ~ScriptLane();
  ScriptLane(const ScriptLane &) = delete;
  ScriptLane &operator=(const ScriptLane &) = delete;

  cvc::app &app() const;
  AppRuntime &runtime() const { return runtime_; }
  const std::string &language() const { return language_; }
  const std::string &prefix() const; // the document's state prefix

  // Functions (§8.2: one registry): (name a b ...) in any of the document's programs calls `fn`
  // (AppRuntime::register_function / register_async_function).
  void export_function(const std::string &name, cvc::state_exec::native_fn fn);
  void export_async_function(const std::string &name, cvc::state_exec::async_marshal marshal,
                             const std::string &done_channel = std::string());
  // A function whose every call is an activation (§8.5), `make`'s, run three ways:
  //   (name a b ...) from a program whose host parks (an action, a resident: on group 0, or on
  //     the document's isolated group) runs it as the program's own sliced native call (a data job,
  //     data_job.h): a slice per data step of the calling process, charged to it, so its caps are
  //     the activation's. The slice's call()s and await()s are the program's own, made in its
  //     process (its chroot, its functions), and a yield_frame() parks it for a frame. It returns
  //     what the activation finished with, or raises what a step threw, with its own class.
  //     options' caps, lane, scope and priority don't apply: the program's do. A call it can't run
  //     there (none of the program's steps is running) raises RuntimeError.
  //   (name-async ...) returns a future: the activation runs as an action of the document, in a
  //     process of its own (start), under options; a cap or a kill raises RuntimeError at the
  //     await. When the calling process goes first, the activation stops at its next slice.
  //   Where the caller can't park (init:, which runs on a scheduler of its own), the call runs the
  //     activation to its end there (run), and raises RuntimeError where that would re-enter the
  //     app's scheduler.
  void export_activation(const std::string &name, ScriptCall make, ActivationOptions options = {});

  // Start an activation now: the next pump runs its first slice (the drain's, when called before
  // the drain pumps: from an on: handler, or a hook). Returns its pid; throws when it can't start.
  int start(std::shared_ptr<ScriptActivation> activation, ActivationOptions options);
  // Run an activation to its end now: start it, then pump the document's scheduler (its isolated
  // group's, else the app's; Runtime::pump) until it ends, a frame per pump. For work that can't
  // wait for the frame loop: a guest's ari_init at load, a call from init:. Under options.lane's
  // caps (ProgramLane::init's for a load's work), and at most `max_seconds` of wall time, past
  // which it is stopped. Returns how it ended (options.on_done hears it too). Throws RuntimeError
  // from inside a step of either scheduler, or a frame's pump, which it would have to re-enter.
  ActivationOutcome run(std::shared_ptr<ScriptActivation> activation, ActivationOptions options,
                        double max_seconds = 30.0);
  // Stop one: its process is killed, and its on_done hears "did not complete: killed".
  void stop(int pid);
  // Whether its process is still running.
  bool running(int pid) const;

  // Hooks (§8.7), until the engine's next unload. Every delivery runs the activation its hook
  // makes, with `options` (the hook's kind sets options.lane):
  //   on_event: `event` fired (a button's or a menu item's on:), as an action, every time, with
  //     {"event": event}. It is the runtime's handler of that event (AppRuntime::on).
  //   on_frame: at the end of every drain, as a resident, with {"frame": exec_frame()}: one in
  //     flight, so a slow one coalesces the frames it spans.
  //   on_watch: `path` (under the document's prefix, as a program's state-watch is) changed, as a
  //     resident, with {"path": path, "value": its text}: read at the end of each drain, one in
  //     flight, and the next delivers the value as it is then.
  void on_event(const std::string &event, ScriptHook hook, ActivationOptions options = {});
  void on_frame(ScriptHook hook, ActivationOptions options = {});
  void on_watch(const std::string &path, ScriptHook hook, ActivationOptions options = {});

  // ── for AppRuntime ──
  // Drop the hooks (the engine unloads).
  void drop_hooks();
  // The end of a drain: the frame and watch hooks' deliveries.
  void frame();

private:
  struct Hook;
  void fire(const std::string &event);
  // Start the activation `hook` makes of `event`; a failure to is the runtime's warning.
  void deliver(Hook &hook, const cvc::state_exec::value_t &event);
  AppRuntime &runtime_;
  std::string language_;
  std::vector<std::shared_ptr<Hook>> hooks_;
  std::vector<std::string> handled_; // the events it has an AppRuntime::on handler for
  // Shared with those handlers (installed once per event, never removed): they deliver through it
  // while the lane lives, and do nothing after.
  std::shared_ptr<ScriptLane *> self_;
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_SCRIPT_ENGINE_H
