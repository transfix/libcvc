// script_engine.cpp — see the header: the engine registry, the lanes, their hooks, and
// activations run as processes of the document's program lane (Runtime::start_process).
//
// Ownership, as a sliced task's (sliced_task.cpp): an activation's record (the activation, its
// slice, what it asked for) is held by the natives of its process's environment, through an
// anchor. When the process ends, for any reason, the environment lets go of the anchor, and the
// anchor of the activation, cancelling one that never finished. The process's done (the Runtime's
// sweep) holds the record too, for the result it hands on_done.

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/loader.h> // register_ari_block: an engine's language is a block key
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h> // in_pump_all: run() never re-enters a frame's pump
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h> // async_work: an exported call parks on its activation
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h> // a call from a program is its sliced work
#include <cvc/state/state_exec/detail/lock_home.h> // lock_ops_of: the caller's with-lock chroot
#include <cvc/state/state_exec/evaluator.h>        // step_kind
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/exec_group.h> // an isolated document's group: its scheduler
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <deque>
#include <map>
#include <mutex>
#include <set>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>

namespace cvc {
namespace ariadne {

namespace se = cvc::state_exec;

namespace {

using steady = std::chrono::steady_clock;

// ── settings ────────────────────────────────────────────────────────────────────────────────────

// The unsigned integer at `key`; `fallback` when it is missing or unparsable.
std::uint64_t count_setting(cvc::state &root, const std::string &key, std::uint64_t fallback) {
  cvc::state *node = root.findDescendant(key);
  if (!node)
    return fallback;
  const std::string v = node->value();
  std::uint64_t n = 0;
  const auto [end, ec] = std::from_chars(v.data(), v.data() + v.size(), n);
  return !v.empty() && ec == std::errc() && end == v.data() + v.size() ? n : fallback;
}

// The seconds at `key` (>= 0); `fallback` when they are missing or unparsable.
double seconds_setting(cvc::state &root, const std::string &key, double fallback) {
  cvc::state *node = root.findDescendant(key);
  if (!node)
    return fallback;
  const std::string v = node->value();
  char *end = nullptr;
  const double s = v.empty() ? -1.0 : std::strtod(v.c_str(), &end);
  return end == v.c_str() + v.size() && std::isfinite(s) && s >= 0.0 ? s : fallback;
}

constexpr std::uint64_t kSliceSteps = 64; // ariadne.limits.<language>_slice_steps
constexpr double kFrameBudget = 0.008;    // ariadne.limits.<language>_frame_budget, seconds

// ── the frame budget ────────────────────────────────────────────────────────────────────────────

// The time one language's activations have taken in the frame they are in. Per app, in its data
// map (as the app's other registries are), so it dies with the app; read and written only on the
// thread that pumps the scheduler.
struct frame_budget {
  std::uint64_t frame = UINT64_MAX;
  double spent = 0.0; // seconds
};

std::shared_ptr<frame_budget> budget_of(cvc::app &app, const std::string &language) {
  using budgets = std::map<std::string, std::shared_ptr<frame_budget>>;
  static const char kKey[] = "__ariadne_script_frame_budgets";
  const cvc::mutex_ptr guard = app.mutex(kKey); // check-then-create, as for the app's state root
  boost::mutex::scoped_lock lock(*guard);
  std::shared_ptr<budgets> all;
  try {
    all = app.data<std::shared_ptr<budgets>>(kKey);
  } catch (const std::exception &) {
    // not yet present
  }
  if (!all) {
    all = std::make_shared<budgets>();
    app.data(kKey, all);
  }
  std::shared_ptr<frame_budget> &b = (*all)[language];
  if (!b)
    b = std::make_shared<frame_budget>();
  return b;
}

// The process's loop (the header's). A reply is (ok value): #t and the value of the call or the
// await, or #f and the exception it raised.
const char kLoop[] = "(let ((k (script-step)))\n"
                     "  (while (not (= k \"done\"))\n"
                     "    (if (= k \"yield\")\n"
                     "        (await #f)\n"
                     "        (if (= k \"call\")\n"
                     "            (script-reply (try (list #t (apply (script-callee) "
                     "(script-arguments)))\n"
                     "                            (catch (e) (list #f e))))\n"
                     "            (if (= k \"await\")\n"
                     "                (script-reply (try (list #t (await (script-awaited)))\n"
                     "                                (catch (e) (list #f e)))))))\n"
                     "    (set k (script-step))))\n";

// The process's program: the loop, inside (with-lock (script-lock) ...) for an activation with a
// lock (ActivationOptions::lock).
std::string program(bool locked) {
  return locked ? std::string("(with-lock (script-lock)\n") + kLoop + ")\n(script-finish)\n"
                : std::string(kLoop) + "(script-finish)\n";
}

// The absolute key of `lock`, as ActivationOptions has it, for the document at `prefix`.
std::string lock_path(const std::string &lock, const std::string &prefix) {
  if (!lock.empty() && lock.front() == '/')
    return lock.substr(1);
  return prefix.empty() ? lock : prefix + "." + lock;
}

// `abs`, an absolute key, as with-lock takes it in a process chrooted at `root`: relative when it
// lies under the root, else from the app's root.
std::string written_key(const std::string &abs, const std::string &root) {
  if (root.empty())
    return abs;
  if (abs.size() > root.size() && abs.compare(0, root.size(), root) == 0 && abs[root.size()] == '.')
    return abs.substr(root.size() + 1);
  return "/" + abs;
}

} // namespace

// What an activation's natives close over (the header's ownership note).
struct ActivationRunner {
  cvc::app *app = nullptr;
  std::shared_ptr<ScriptActivation> activation; // let go of once its process ends
  std::uint64_t slice_steps = kSliceSteps;
  double frame_seconds = kFrameBudget; // 0: no frame budget
  std::shared_ptr<frame_budget> budget;
  std::function<bool(const std::string &)> may_call;
  std::string lock; // the lock's key as the process writes it, for (script-lock)
  // The process's environment and intrinsics context, set as it starts: both live as long as the
  // process, and the natives run only while it does.
  se::environment *env = nullptr;
  se::intrinsics_context *ictx = nullptr;
  ScriptSlice slice;             // carries a reply from one step to the next
  se::value_t callee, arguments; // a call() asked for
  se::value_t awaited;           // an await() asked for
  bool finished = false;
  se::value_t result;
  se::value_t failure; // the exception a step threw, for on_done

  // ── a slice's bookkeeping, shared with SlicedCall ──

  // Whether a language's activations have had their time in this frame (`b`, reset per frame).
  static bool frame_spent(cvc::app &app, frame_budget *b, double frame_seconds) {
    if (frame_seconds <= 0.0 || !b)
      return false;
    const std::uint64_t frame = exec_frame(app);
    if (b->frame != frame) {
      b->frame = frame;
      b->spent = 0.0;
    }
    return b->spent >= frame_seconds;
  }

  // A slice's start: its budget, what its calls reach, and nothing asked for yet (the reply, if
  // any, is this slice's).
  static void open(ScriptSlice &s, std::uint64_t budget, se::environment *env,
                   const std::function<bool(const std::string &)> &may_call) {
    s.env_ = env;
    s.may_call_ = may_call ? &may_call : nullptr;
    s.budget_ = budget;
    s.charged_ = 0;
    s.next_ = ScriptSlice::Next::more;
    s.out_ = se::value_t();
    s.function_.clear();
    s.args_.clear();
  }

  // Whether the slice finished its activation, and taking that back: a queue's delivery goes on
  // to its next event (QueueDelivery).
  static bool finishing(const ScriptSlice &s) { return s.next_ == ScriptSlice::Next::finish; }
  static se::value_t go_on(ScriptSlice &s) {
    s.next_ = ScriptSlice::Next::more;
    return std::exchange(s.out_, se::value_t());
  }

  // A slice's end: the reply it had is spent.
  static void close(ScriptSlice &s) {
    s.replied_ = false;
    s.reply_ = se::value_t();
    s.error_.clear();
    s.exception_ = se::value_t();
  }

  // A reply the next slice gets without the loop's help: a call that was refused, or that names
  // nothing.
  static void refuse(ScriptSlice &s, const char *cls, const std::string &message) {
    s.replied_ = true;
    s.reply_ = se::value_t();
    s.exception_ = se::make_exception(se::exception_class(cls), message);
    s.error_ = se::exception_summary(s.exception_);
  }

  // A reply (ok value) for the next slice: #t and the value of the call or the await, or #f and
  // the exception it raised.
  static void reply(ScriptSlice &s, const se::value_t &r) {
    const auto *l = std::get_if<se::list_ptr>(&r.v);
    if (!l || !*l || (*l)->size() != 2)
      return;
    s.replied_ = true;
    if ((**l)[0].is_truthy()) {
      s.reply_ = (**l)[1];
      s.error_.clear();
      s.exception_ = se::value_t();
    } else {
      s.reply_ = se::value_t();
      s.exception_ = (**l)[1];
      s.error_ = se::exception_summary(s.exception_);
      if (s.error_.empty())
        s.error_ = "Exception";
    }
  }

  // A pure export's activation (ScriptLane::export_pure), run to its end inside the read lane's
  // eval that called it: a slice at a time, each slice's steps charged to that eval, and the
  // eval's deadline checked between slices. What a slice asks for is answered at once, with no
  // loop to make it: a call by call_now() in the read lane's environment `env` (anything else
  // there is refused), a yield by going on, an await by refusing it.
  static se::value_t run_pure(ScriptActivation &a, se::environment &env, std::uint64_t slice_steps,
                              const std::string &name) {
    ScriptSlice s;
    s.pure_ = true;
    const std::function<bool(const std::string &)> all; // the environment is the narrowing
    try {
      for (;;) {
        se::eval_deadline_check(name.c_str());
        open(s, slice_steps, &env, all);
        a.step(s);
        if (s.charged_ > 0)
          if (se::detail::data_step_scope *data = se::detail::data_step_scope::current())
            data->stats().add_steps(se::step_kind::data, s.charged_);
        close(s);
        switch (s.next_) {
        case ScriptSlice::Next::finish:
          return std::move(s.out_);
        case ScriptSlice::Next::await:
          refuse(s, "RuntimeError",
                 name + ": a pure function can't wait (the read lane runs it to its end)");
          break;
        case ScriptSlice::Next::call:
          if (!s.call_now(s.function_, s.args_))
            refuse(s, "PermissionError",
                   s.function_ + ": not a function of the read lane (a pure function reaches the "
                                 "state readers and the pure functions only)");
          break;
        case ScriptSlice::Next::yield: // no frame to give back: on at once
        case ScriptSlice::Next::more:
          break;
        }
      }
    } catch (...) {
      a.cancel(); // the eval failed, timed out or ran out of steps around it
      throw;
    }
  }

  // (script-step): run a slice, and say what the loop does next.
  se::value_t step() {
    if (finished || !activation)
      return se::value_t(std::string("done"));
    if (frame_spent(*app, budget.get(), frame_seconds))
      return se::value_t(std::string("yield")); // the reply, if any, waits for the next frame
    ScriptSlice &s = slice;
    open(s, slice_steps, env, may_call);
    const steady::time_point t0 = steady::now();
    const auto settle = [&] {
      if (budget)
        budget->spent += std::chrono::duration<double>(steady::now() - t0).count();
      if (s.charged_ > 0 && ictx && ictx->sched)
        if (const se::process_ptr &proc = ictx->sched->current_process())
          proc->state.stats.add_steps(se::step_kind::data, s.charged_);
      close(s);
    };
    try {
      activation->step(s);
    } catch (const std::exception &e) {
      settle();
      failure = se::exception_from(e);
      throw;
    } catch (...) {
      settle();
      throw;
    }
    settle();
    switch (s.next_) {
    case ScriptSlice::Next::finish:
      finished = true;
      result = std::move(s.out_);
      return se::value_t(std::string("done"));
    case ScriptSlice::Next::yield:
      return se::value_t(std::string("yield"));
    case ScriptSlice::Next::await:
      awaited = std::move(s.out_);
      return se::value_t(std::string("await"));
    case ScriptSlice::Next::call: {
      if (may_call && !may_call(s.function_)) {
        refuse(s, "PermissionError",
               s.function_ + ": not among the functions this activation may call (its imports)");
        return se::value_t(std::string("more"));
      }
      const se::value_t *fn = env ? env->lookup(s.function_) : nullptr;
      if (!fn) {
        refuse(s, "NameError", s.function_ + ": no such function here");
        return se::value_t(std::string("more"));
      }
      callee = *fn;
      arguments = se::make_list(std::move(s.args_));
      return se::value_t(std::string("call"));
    }
    case ScriptSlice::Next::more:
      break;
    }
    return se::value_t(std::string("more"));
  }
};

namespace {

// Held by the natives: when the environment lets go of it, the process has ended.
struct runner_anchor {
  std::shared_ptr<ActivationRunner> run;
  ~runner_anchor() {
    try {
      if (run && run->activation) {
        std::shared_ptr<ScriptActivation> a = std::move(run->activation);
        if (!run->finished)
          a->cancel();
      }
    } catch (...) {
    }
  }
};

// The activation's natives, in its process's own scope.
void bind_runner(const se::environment_ptr &env, const std::shared_ptr<runner_anchor> &anchor) {
  using args_t = std::span<const se::value_t>;
  se::builtins::register_fn(env, "script-step",
                            [anchor](args_t) -> se::value_t { return anchor->run->step(); });
  // (script-reply (ok value)): what the call or the await came back with, for the next slice.
  se::builtins::register_fn(env, "script-reply", [anchor](args_t a) -> se::value_t {
    if (!a.empty())
      ActivationRunner::reply(anchor->run->slice, a[0]);
    return se::value_t();
  });
  se::builtins::register_fn(env, "script-callee",
                            [anchor](args_t) -> se::value_t { return anchor->run->callee; });
  se::builtins::register_fn(env, "script-arguments",
                            [anchor](args_t) -> se::value_t { return anchor->run->arguments; });
  se::builtins::register_fn(env, "script-awaited",
                            [anchor](args_t) -> se::value_t { return anchor->run->awaited; });
  se::builtins::register_fn(env, "script-finish", [anchor](args_t) -> se::value_t {
    return se::value_t(anchor->run->finished);
  });
  se::builtins::register_fn(env, "script-lock",
                            [anchor](args_t) -> se::value_t { return anchor->run->lock; });
}

} // namespace

// ── the registry ────────────────────────────────────────────────────────────────────────────────

namespace {
// An app's script engines, by language.
struct engine_table {
  static constexpr const char *app_slot = "cvc.ariadne.script-engines";
  std::mutex m;
  std::map<std::string, std::shared_ptr<ScriptEngine>> engines;
};
} // namespace

void register_script_engine(cvc::app &app, std::shared_ptr<ScriptEngine> engine) {
  if (!engine)
    return;
  const std::string language = engine->language();
  if (language.empty())
    return;
  {
    engine_table &t = app.slot<engine_table>();
    std::lock_guard<std::mutex> lock(t.m);
    t.engines[language] = std::move(engine);
  }
  // Its blocks are kept for the runtimes that run them (AppRuntime::use_script_engine).
  register_ari_block(app, language, [language](const Value &content, LoadResult &out) {
    out.extras.emplace_back(language, content);
  });
}

void unregister_script_engine(cvc::app &app, const std::string &language) {
  std::shared_ptr<ScriptEngine> gone; // let go of outside the lock
  engine_table &t = app.slot<engine_table>();
  std::lock_guard<std::mutex> lock(t.m);
  auto it = t.engines.find(language);
  if (it == t.engines.end())
    return;
  gone = std::move(it->second);
  t.engines.erase(it);
}

std::shared_ptr<ScriptEngine> find_script_engine(cvc::app &app, const std::string &language) {
  engine_table &t = app.slot<engine_table>();
  std::lock_guard<std::mutex> lock(t.m);
  auto it = t.engines.find(language);
  return it == t.engines.end() ? nullptr : it->second;
}

// ── ScriptSlice ─────────────────────────────────────────────────────────────────────────────────

void ScriptSlice::finish(se::value_t result) {
  next_ = Next::finish;
  out_ = std::move(result);
}

void ScriptSlice::yield_frame() { next_ = Next::yield; }

void ScriptSlice::await(se::value_t future) {
  next_ = Next::await;
  out_ = std::move(future);
}

void ScriptSlice::call(std::string function, std::vector<se::value_t> args) {
  next_ = Next::call;
  function_ = std::move(function);
  args_ = std::move(args);
}

bool ScriptSlice::call_now(const std::string &function, std::span<const se::value_t> args) {
  if (!env_ || (may_call_ && !(*may_call_)(function)))
    return false;
  const se::value_t *fn = env_->lookup(function);
  const se::native_fn *native = fn ? std::get_if<se::native_fn>(&fn->v) : nullptr;
  if (!native)
    return false;
  // The call is this slice's, not its step's: what it would hand the evaluator to finish (a
  // data job) runs to its end here instead, so the step's own call isn't completed with it.
  struct no_defer {
    se::detail::data_step_scope *scope = se::detail::data_step_scope::current();
    bool was = scope && scope->defer_allowed();
    no_defer() {
      if (scope)
        scope->allow_defer(false);
    }
    ~no_defer() {
      if (scope)
        scope->allow_defer(was);
    }
  };
  se::value_t value;
  try {
    const no_defer here;
    const se::detail::native_nesting level(function.c_str());
    value = (*native)(args);
  } catch (const std::exception &e) {
    if (se::is_uncatchable(e))
      throw;
    replied_ = true;
    reply_ = se::value_t();
    exception_ = se::exception_from(e);
    error_ = se::exception_summary(exception_);
    if (error_.empty())
      error_ = "Exception";
    return true;
  }
  replied_ = true;
  reply_ = std::move(value);
  error_.clear();
  exception_ = se::value_t();
  return true;
}

// ── ScriptLane ──────────────────────────────────────────────────────────────────────────────────

namespace {

// A key, pointer, signal or message hook's events not yet delivered, and the failures it has
// reported (each once).
struct HookQueue {
  std::deque<se::value_t> events;
  std::set<std::string> reported;
};

using OutcomeFn = std::function<void(const ActivationOutcome &)>;

// The delivery of a queued hook's events (ScriptLane::on_key ...): one resident takes them in
// order, each to the activation the hook makes of it, stepped in this one process, and ends once
// the queue is empty. `done` hears how each event's activation ended; one that throws is over and
// the next event goes on, as an action's failure leaves the next action alone.
class QueueDelivery : public ScriptActivation {
public:
  QueueDelivery(ScriptHook make, std::shared_ptr<HookQueue> queue, OutcomeFn done)
      : make_(std::move(make)), queue_(std::move(queue)), done_(std::move(done)) {}

  void step(ScriptSlice &s) override {
    while (!inner_) {
      if (queue_->events.empty()) {
        s.finish();
        return;
      }
      const se::value_t event = std::move(queue_->events.front());
      queue_->events.pop_front();
      try {
        inner_ = make_ ? make_(event) : nullptr; // null lets the event pass
      } catch (const std::exception &e) {
        ended(false, se::value_t(), std::string("did not start: ") + e.what(), se::value_t());
      }
    }
    try {
      inner_->step(s);
    } catch (const std::exception &e) {
      inner_.reset();
      ActivationRunner::go_on(s);
      const se::value_t ex = se::exception_from(e);
      ended(false, se::value_t(), "failed: " + se::exception_summary(ex), ex);
      if (queue_->events.empty())
        s.finish();
      return;
    }
    if (!ActivationRunner::finishing(s))
      return;
    inner_.reset();
    ended(true, ActivationRunner::go_on(s), std::string(), se::value_t());
    if (queue_->events.empty())
      s.finish(); // nothing waiting: saves a slice
  }

  void cancel() override {
    if (inner_)
      inner_->cancel();
  }

private:
  void ended(bool ok, se::value_t value, std::string error, se::value_t exception) {
    if (!done_)
      return;
    ActivationOutcome o;
    o.ok = ok;
    o.value = std::move(value);
    o.error = std::move(error);
    o.exception = std::move(exception);
    done_(o);
  }

  ScriptHook make_;
  std::shared_ptr<HookQueue> queue_;
  OutcomeFn done_;
  std::shared_ptr<ScriptActivation> inner_; // the event being delivered
};

// A message hook's receiver: it waits on the channel as a program's (msg-recv channel) does, in a
// process of the lane, and queues each message for the hook's delivery, which `take` starts when
// none runs. It holds no lock, so nothing waits for it while it waits.
class MessageReceiver : public ScriptActivation {
public:
  MessageReceiver(std::string channel, std::shared_ptr<HookQueue> queue, std::function<void()> take)
      : channel_(std::move(channel)), queue_(std::move(queue)), take_(std::move(take)) {}

  void step(ScriptSlice &s) override {
    if (s.has_reply()) {
      if (s.failed()) { // the channel refused (its policy), say: the receiver stops with it
        if (!s.exception().is_nil())
          throw se::dsl_exception(s.exception());
        throw se::dsl_error("RuntimeError", s.error());
      }
      queue_->events.push_back(
          se::make_dict({{"channel", se::value_t(channel_)}, {"value", s.reply()}}));
      take_();
    }
    s.call("msg-recv", {se::value_t(channel_)});
  }

private:
  std::string channel_;
  std::shared_ptr<HookQueue> queue_;
  std::function<void()> take_;
};

// A failure a queued hook reports itself (it has no on_done), at most this many different ones.
constexpr std::size_t kMaxReported = 16;
// A message hook's receiver is restarted as the Runtime restarts a resident (ariadne.cpp).
constexpr std::uint64_t kQuickStop = 100;
constexpr int kMaxQuickStops = 8;
constexpr std::uint64_t kRestartMaxDelay = 64;

} // namespace

struct ScriptLane::Hook {
  enum class Kind { event, frame, watch, key, pointer, signal, message };
  Kind kind = Kind::event;
  // The event, the watched node's path from the app's root, the kind of input ("key", "pointer",
  // "signal"), or the channel.
  std::string key;
  std::string path; // a watch hook's path as it was given, for its events
  ScriptHook make;
  ActivationOptions options;
  int pid = -1;     // the delivery in flight (a frame, watch or queued hook's)
  std::string last; // a watch hook's: the value it last delivered, or saw when it was set
  std::shared_ptr<HookQueue> queue; // a key, pointer, signal or message hook's
  // A message hook's receiver, and its restarts: the frame it started on, its stops soon after
  // starting in a row, the frame before which it isn't started again, and whether it never is.
  int receiver = -1;
  std::uint64_t started = 0;
  int quick_stops = 0;
  std::uint64_t retry_at = 0;
  bool given_up = false;
  bool tapped() const { return kind == Kind::key || kind == Kind::pointer || kind == Kind::signal; }
};

ScriptLane::ScriptLane(AppRuntime &runtime, std::string language)
    : runtime_(runtime), language_(std::move(language)),
      self_(std::make_shared<ScriptLane *>(this)) {}

ScriptLane::~ScriptLane() { *self_ = nullptr; }

cvc::app &ScriptLane::app() const { return runtime_.app(); }

const std::string &ScriptLane::prefix() const { return runtime_.prefix(); }

void ScriptLane::export_function(const std::string &name, se::native_fn fn) {
  runtime_.register_function(name, std::move(fn));
}

void ScriptLane::export_async_function(const std::string &name, se::async_marshal marshal,
                                       const std::string &done_channel) {
  runtime_.register_async_function(name, std::move(marshal), done_channel);
}

int ScriptLane::start(std::shared_ptr<ScriptActivation> activation, ActivationOptions options) {
  if (!activation)
    throw std::invalid_argument("ScriptLane::start: no activation");
  cvc::state &root = cvc::state::instance(app());
  auto run = std::make_shared<ActivationRunner>();
  run->app = &app();
  run->activation = std::move(activation);
  run->slice_steps =
      options.slice_steps
          ? options.slice_steps
          : count_setting(root, "ariadne.limits." + language_ + "_slice_steps", kSliceSteps);
  run->frame_seconds =
      seconds_setting(root, "ariadne.limits." + language_ + "_frame_budget", kFrameBudget);
  run->budget = budget_of(app(), language_);
  run->may_call = std::move(options.may_call);
  const std::string lock = options.lock.empty() ? std::string() : lock_path(options.lock, prefix());

  LaneProcess p;
  p.program = program(!lock.empty());
  p.name = options.name.empty() ? language_ + "/activation" : std::move(options.name);
  p.lane = options.lane;
  p.scope = std::move(options.scope);
  p.max_steps = options.max_steps;
  p.max_time = options.max_time;
  p.max_memory = options.max_memory;
  p.priority = options.priority;
  p.extend = [run, lock](const se::environment_ptr &env, se::intrinsics_context &ictx) {
    run->env = env.get();
    run->ictx = &ictx;
    if (!lock.empty())
      run->lock = written_key(lock, ictx.root_path);
    auto anchor = std::make_shared<runner_anchor>();
    anchor->run = run;
    bind_runner(env, anchor);
  };
  if (options.on_done) // without it, a failure is the runtime's warning (Runtime::start_process)
    p.done = [run, done = std::move(options.on_done)](const std::string &how) {
      ActivationOutcome o;
      o.ok = how.empty();
      o.value = run->result;
      o.error = how;
      if (!o.ok)
        o.exception = run->failure;
      done(o);
    };
  return runtime_.runtime().start_process(std::move(p));
}

namespace {

// An exported call's activation, which stops at its next slice once the call is cancelled (the
// calling process killed or reaped, the host going): a flag, since the cancel may come from any
// thread and the scheduler's own calls may not.
class CallActivation : public ScriptActivation {
public:
  CallActivation(std::shared_ptr<ScriptActivation> inner, std::shared_ptr<std::atomic<bool>> stop)
      : inner_(std::move(inner)), stop_(std::move(stop)) {}
  void step(ScriptSlice &slice) override {
    if (stop_->load(std::memory_order_acquire))
      throw se::dsl_error("RuntimeError", "the call was cancelled");
    inner_->step(slice);
  }
  void cancel() override { inner_->cancel(); }

private:
  std::shared_ptr<ScriptActivation> inner_;
  std::shared_ptr<std::atomic<bool>> stop_;
};

// What a sliced call hands the evaluator for each call or await its activation asks for: k is
// "call" (apply f to the list a) or "await" (await f; #f is the next frame), and the value is the
// reply (ok value), as the activation loop's (kProgram).
const char kSlicedAsk[] = "(try (list #t (if (= k \"call\") (apply f a) (await f)))\n"
                          "     (catch (e) (list #f e)))";

} // namespace

// Where the document's processes, its activations among them, run: its isolated group's scheduler,
// else the app's (group 0). current_group decides nothing and moves nothing.
static se::async_scheduler &document_scheduler(AppRuntime &runtime) {
  se::exec_group *const group = runtime.runtime().current_group();
  return group ? group->sched() : runtime.app().exec_scheduler();
}

// An exported call from a program whose host parks (export_activation; group 0's, or an isolated
// document's group's): the program's own deep-data work (data_job.h), one slice of the activation
// per data step of its process, each slice's steps charged to that process, so the program's caps
// bound it. A call or an await a slice asks for is the program's, made in its process by a closure
// around a try (kSlicedAsk): its value or its exception comes back to the next slice instead of
// unwinding the job. When the job goes before its activation finished (a throw, a cap, a kill), the
// activation is cancelled.
class SlicedCall : public se::data_job {
public:
  SlicedCall(cvc::app &app, std::shared_ptr<ScriptActivation> activation, std::string name,
             std::uint64_t slice_steps, double frame_seconds, std::shared_ptr<frame_budget> budget,
             std::function<bool(const std::string &)> may_call, se::environment_ptr env)
      : app_(app), activation_(std::move(activation)), name_(std::move(name)),
        slice_steps_(slice_steps), frame_seconds_(frame_seconds), budget_(std::move(budget)),
        may_call_(std::move(may_call)), env_(std::move(env)) {
    auto ask = std::make_shared<se::closure>();
    ask->params = {se::symbol{"k"}, se::symbol{"f"}, se::symbol{"a"}};
    ask->body = {se::parse(kSlicedAsk)};
    ask->env_snapshot = env_;
    ask_ = se::value_t(std::move(ask));
  }

  ~SlicedCall() override {
    try {
      if (activation_ && !finished_)
        activation_->cancel();
    } catch (...) {
    }
  }

  bool step(std::size_t) override {
    if (finished_)
      return true;
    if (ActivationRunner::frame_spent(app_, budget_.get(), frame_seconds_)) {
      ask("await", se::value_t(false), se::value_t(), false); // the reply, if any, waits too
      return false;
    }
    ScriptSlice &s = slice_;
    ActivationRunner::open(s, slice_steps_, env_.get(), may_call_);
    const steady::time_point t0 = steady::now();
    const auto settle = [&] {
      if (budget_)
        budget_->spent += std::chrono::duration<double>(steady::now() - t0).count();
      if (s.charged_ > 0)
        if (se::detail::data_step_scope *data = se::detail::data_step_scope::current())
          data->stats().add_steps(se::step_kind::data, s.charged_);
      ActivationRunner::close(s);
    };
    try {
      activation_->step(s);
    } catch (...) {
      settle();
      throw;
    }
    settle();
    switch (s.next_) {
    case ScriptSlice::Next::finish:
      finished_ = true;
      result_ = std::move(s.out_);
      return true;
    case ScriptSlice::Next::yield:
      ask("await", se::value_t(false), se::value_t(), false);
      return false;
    case ScriptSlice::Next::await:
      ask("await", std::move(s.out_), se::value_t(), true);
      return false;
    case ScriptSlice::Next::call: {
      if (may_call_ && !may_call_(s.function_)) {
        ActivationRunner::refuse(s, "PermissionError",
                                 s.function_ +
                                     ": not among the functions this activation may call (its "
                                     "imports)");
        return false;
      }
      const se::value_t *fn = env_ ? env_->lookup(s.function_) : nullptr;
      if (!fn) {
        ActivationRunner::refuse(s, "NameError", s.function_ + ": no such function here");
        return false;
      }
      ask("call", *fn, se::make_list(std::move(s.args_)), true);
      return false;
    }
    case ScriptSlice::Next::more:
      break;
    }
    return false;
  }

  se::value_t result() override { return std::move(result_); }

  void call_returned(se::value_t value) override {
    if (wants_reply_)
      ActivationRunner::reply(slice_, value);
  }

private:
  // Stop for the evaluator to make a call of kSlicedAsk: (k f a). A yield's value is no reply.
  void ask(const char *k, se::value_t f, se::value_t a, bool wants_reply) {
    asked_ = {se::value_t(std::string(k)), std::move(f), std::move(a)};
    wants_reply_ = wants_reply;
    call_ = call_request{&ask_, asked_, name_.c_str()};
  }

  cvc::app &app_;
  std::shared_ptr<ScriptActivation> activation_;
  std::string name_;
  std::uint64_t slice_steps_;
  double frame_seconds_;
  std::shared_ptr<frame_budget> budget_;
  std::function<bool(const std::string &)> may_call_;
  se::environment_ptr env_; // the calling process's: what its calls reach
  se::value_t ask_;         // kSlicedAsk, as a closure over env_
  std::vector<se::value_t> asked_;
  bool wants_reply_ = false;
  ScriptSlice slice_;
  bool finished_ = false;
  se::value_t result_;
};

namespace {

// A sliced call under its activation's lock (`abs`, absolute): (with-lock key (run)), a call the
// calling process makes in its environment `env` (call_function), the job starting in the body
// once the lock is held. The key is written as that process's with-lock takes it.
se::value_t locked_call(std::unique_ptr<SlicedCall> job, const std::string &abs,
                        const se::environment_ptr &env, const std::string &name) {
  const se::value_t *with = env ? env->lookup("with-lock") : nullptr;
  const se::detail::lock_ops *ops = with ? se::detail::lock_ops_of(*with) : nullptr;
  if (!ops || !ops->ctx)
    throw se::dsl_error("RuntimeError", name + ": called where with-lock isn't bound, so it "
                                               "can't wait for what its activation runs on");
  const se::value_t body = se::parse("(with-lock key (run))");
  auto held = std::make_shared<std::unique_ptr<SlicedCall>>(std::move(job));
  se::native_fn run = [held](std::span<const se::value_t>) -> se::value_t {
    if (!*held)
      throw se::dsl_error("RuntimeError", "the call has already run");
    return se::run_data_job(std::move(*held));
  };
  auto c = std::make_shared<se::closure>();
  c->params = {se::symbol{"key"}, se::symbol{"run"}};
  c->body = {body};
  c->env_snapshot = env;
  const se::value_t args[] = {se::value_t(written_key(abs, ops->ctx->root_path)),
                              se::value_t(std::move(run))};
  return se::call_function(se::value_t(std::move(c)), args, "with-lock");
}

} // namespace

void ScriptLane::export_activation(const std::string &name, ScriptCall make,
                                   ActivationOptions options) {
  cvc::app *app = &this->app();
  const std::string language = language_;
  options.lane = ProgramLane::action;
  if (options.name.empty())
    options.name = language + "/" + name;
  // A call that waits for its activation (a program parks on it).
  se::async_work_marshal parked = [self = self_, app, name, make,
                                   options](std::span<const se::value_t> args) {
    se::async_work work; // no kernel: a call that can't park raises RuntimeError
    work.start = [self, app, name, make, options,
                  copied = std::vector<se::value_t>(args.begin(), args.end())](
                     const std::shared_ptr<se::async_call> &call) {
      ScriptLane *lane = *self;
      if (!lane)
        throw std::runtime_error(name + ": its document has gone");
      // The host the call was made on, the calling program's (its group's in an isolated
      // document, S1.9): the call is that host's, so its outcome goes back through it. Its
      // finisher posts nothing once that host has shut down (a move retired the group).
      se::async_host *const made_on = call->starting_host();
      const se::async_host::finisher finisher =
          (made_on ? *made_on : app->exec_async_host()).make_finisher();
      std::shared_ptr<ScriptActivation> made = make(copied);
      if (!made) {
        if (call->settle(se::value_t(), nullptr))
          finisher.finish(call);
        return;
      }
      auto stop = std::make_shared<std::atomic<bool>>(false);
      ActivationOptions o = options;
      o.on_done = [call, finisher, name](const ActivationOutcome &out) {
        if (call->cancel_requested())
          return;
        std::exception_ptr error;
        if (!out.ok)
          error = !out.exception.is_nil()
                      ? std::make_exception_ptr(se::dsl_exception(out.exception))
                      : std::make_exception_ptr(std::runtime_error(name + ": " + out.error));
        if (call->settle(out.ok ? out.value : se::value_t(), error))
          finisher.finish(call);
      };
      lane->start(std::make_shared<CallActivation>(std::move(made), stop), std::move(o));
      call->on_cancel([stop] { stop->store(true, std::memory_order_release); });
    };
    return work;
  };
  // A call from where the program can't park (init:, whose host runs its calls inline, S1-Q2):
  // the activation, run to its end in it.
  se::native_fn blocking = [self = self_, name, make,
                            options](std::span<const se::value_t> args) -> se::value_t {
    ScriptLane *lane = *self;
    if (!lane)
      throw std::runtime_error(name + ": its document has gone");
    std::shared_ptr<ScriptActivation> made =
        make(std::vector<se::value_t>(args.begin(), args.end()));
    if (!made)
      return se::value_t();
    const ActivationOutcome out = lane->run(std::move(made), options);
    if (out.ok)
      return out.value;
    if (!out.exception.is_nil())
      throw se::dsl_exception(out.exception);
    throw std::runtime_error(name + ": " + out.error);
  };
  // A call from a program whose host parks (group 0's, or an isolated document's group's): the
  // activation as the program's own sliced work.
  se::native_fn sliced = [self = self_, app, language, name, make, may_call = options.may_call,
                          slices = options.slice_steps,
                          lock = options.lock](std::span<const se::value_t> args) -> se::value_t {
    ScriptLane *lane = *self;
    if (!lane)
      throw std::runtime_error(name + ": its document has gone");
    se::detail::data_step_scope *data = se::detail::data_step_scope::current();
    // The calling program's process: on the group this thread is pumping (an isolated document's,
    // S1.9), else on the app's scheduler (group 0).
    se::exec_group *const group = se::exec_group::current();
    const se::process_ptr &proc =
        (group ? group->sched() : app->exec_scheduler()).current_process();
    if (!data || !data->defer_allowed() || !proc)
      throw se::dsl_error("RuntimeError",
                          name + ": called where no program's step is running, so it can't run "
                                 "as that program's work");
    std::shared_ptr<ScriptActivation> made =
        make(std::vector<se::value_t>(args.begin(), args.end()));
    if (!made)
      return se::value_t();
    cvc::state &root = cvc::state::instance(*app);
    const std::uint64_t slice_steps =
        slices ? slices
               : count_setting(root, "ariadne.limits." + language + "_slice_steps", kSliceSteps);
    const double frame_seconds =
        seconds_setting(root, "ariadne.limits." + language + "_frame_budget", kFrameBudget);
    auto job =
        std::make_unique<SlicedCall>(*app, std::move(made), name, slice_steps, frame_seconds,
                                     budget_of(*app, language), may_call, proc->state.global_env);
    if (lock.empty())
      return se::run_data_job(std::move(job));
    return locked_call(std::move(job), lock_path(lock, lane->prefix()), proc->state.global_env,
                       name);
  };
  runtime_.register_async_work_function(name, std::move(parked), std::move(blocking),
                                        std::move(sliced));
}

void ScriptLane::export_pure(const std::string &name, ScriptCall make, ActivationOptions options) {
  cvc::app *app = &this->app();
  const std::string language = language_;
  const std::string label = options.name.empty() ? language + "/" + name : options.name;
  runtime_.runtime().register_read_function(
      name,
      [self = self_, app, language, label, make, slices = options.slice_steps](
          std::span<const se::value_t> args, se::environment &env) -> se::value_t {
        if (!*self)
          throw std::runtime_error(label + ": its document has gone");
        std::shared_ptr<ScriptActivation> made = make(args);
        if (!made)
          return se::value_t();
        const std::uint64_t slice_steps =
            slices ? slices
                   : count_setting(cvc::state::instance(*app),
                                   "ariadne.limits." + language + "_slice_steps", kSliceSteps);
        return ActivationRunner::run_pure(*made, env, slice_steps, label);
      });
}

ActivationOutcome ScriptLane::run(std::shared_ptr<ScriptActivation> activation,
                                  ActivationOptions options, double max_seconds) {
  // Its pumps (Runtime::pump) would re-enter one this thread is inside: a step of the document's
  // scheduler or of group 0's, or a frame's pump_all (exec_pump_frame, td 4.12).
  const cvc::exec_pool *const pool = app().exec_pool_probe();
  if (document_scheduler(runtime_).current_pid() >= 0 ||
      app().exec_scheduler().current_pid() >= 0 || (pool && pool->in_pump_all()))
    throw se::dsl_error("RuntimeError",
                        "a " + language_ +
                            " activation can't be run to its end from inside a step of the "
                            "app's scheduler: call it where the program can wait for it");
  // Shared with on_done, which the Runtime's sweep calls: it may come after a stop, below.
  struct Ended {
    bool done = false;
    ActivationOutcome out;
  };
  auto ended = std::make_shared<Ended>();
  std::function<void(const ActivationOutcome &)> user = std::move(options.on_done);
  options.on_done = [ended, user](const ActivationOutcome &o) {
    ended->done = true;
    ended->out = o;
    if (user)
      user(o);
  };
  const int pid = start(std::move(activation), std::move(options));
  const steady::time_point give_up =
      steady::now() + std::chrono::duration_cast<steady::duration>(
                          std::chrono::duration<double>(max_seconds > 0 ? max_seconds : 30.0));
  while (!ended->done) {
    runtime_.runtime().pump();
    if (ended->done)
      break;
    if (steady::now() >= give_up) {
      stop(pid);
      runtime_.runtime().pump(); // the sweep reports the kill
      if (!ended->done) {
        ended->out.ok = false;
        ended->out.error = "did not complete: it ran past " + std::to_string(max_seconds) + " s";
      }
      break;
    }
    // Where it runs, now that start placed the document: its group's scheduler, else group 0's.
    if (running(pid) && document_scheduler(runtime_).get_stats().ready == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // parked: on a pool, a timer
  }
  return ended->out;
}

void ScriptLane::stop(int pid) {
  if (pid >= 0)
    document_scheduler(runtime_).kill(pid);
}

bool ScriptLane::running(int pid) const {
  if (pid < 0)
    return false;
  const auto info = document_scheduler(runtime_).get_process_info(pid);
  return info && !se::is_finished(info->status);
}

void ScriptLane::on_event(const std::string &event, ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::event;
  h->key = event;
  h->make = std::move(hook);
  h->options = std::move(options);
  h->options.lane = ProgramLane::action;
  hooks_.push_back(std::move(h));
  if (std::find(handled_.begin(), handled_.end(), event) != handled_.end())
    return;
  handled_.push_back(event);
  runtime_.on(event, [self = self_, event] {
    if (ScriptLane *lane = *self)
      lane->fire(event);
  });
}

void ScriptLane::on_frame(ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::frame;
  h->make = std::move(hook);
  h->options = std::move(options);
  h->options.lane = ProgramLane::resident;
  hooks_.push_back(std::move(h));
}

void ScriptLane::on_watch(const std::string &path, ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::watch;
  // Under the document's prefix, as a chrooted program's state-watch resolves it; a leading '/'
  // is from the app's root.
  if (!path.empty() && path[0] == '/')
    h->key = path.substr(1);
  else
    h->key = prefix().empty() ? path : prefix() + "." + path;
  h->path = path;
  h->make = std::move(hook);
  h->options = std::move(options);
  h->options.lane = ProgramLane::resident;
  if (cvc::state *node = cvc::state::instance(app()).findDescendant(h->key))
    h->last = node->value();
  hooks_.push_back(std::move(h));
}

void ScriptLane::on_key(ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::key;
  h->key = "key";
  h->make = std::move(hook);
  h->options = std::move(options);
  add_queued(std::move(h));
}

void ScriptLane::on_pointer(ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::pointer;
  h->key = "pointer";
  h->make = std::move(hook);
  h->options = std::move(options);
  add_queued(std::move(h));
}

void ScriptLane::on_signal(ScriptHook hook, ActivationOptions options) {
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::signal;
  h->key = "signal";
  h->make = std::move(hook);
  h->options = std::move(options);
  add_queued(std::move(h));
}

void ScriptLane::on_message(const std::string &channel, ScriptHook hook,
                            ActivationOptions options) {
  if (channel.empty())
    throw std::invalid_argument("ScriptLane::on_message: no channel");
  auto h = std::make_shared<Hook>();
  h->kind = Hook::Kind::message;
  h->key = channel;
  h->make = std::move(hook);
  h->options = std::move(options);
  Hook &added = *h;
  add_queued(std::move(h));
  receive(added);
}

void ScriptLane::add_queued(std::shared_ptr<Hook> hook) {
  hook->options.lane = ProgramLane::resident;
  hook->queue = std::make_shared<HookQueue>();
  const bool tapped = hook->tapped();
  hooks_.push_back(std::move(hook));
  if (tapped)
    runtime_.script_hooks_changed();
}

void ScriptLane::take_queue(Hook &h) {
  if (!h.queue || h.queue->events.empty() || running(h.pid))
    return;
  ActivationOptions o = h.options; // each delivery its own copy
  OutcomeFn done = std::move(o.on_done);
  // The delivery reports each event through `done`; the activation itself must not report again.
  // A moved-from std::function is not guaranteed empty: libc++ leaves a small callable in place.
  o.on_done = nullptr;
  if (!done) // a failed event's activation is the runtime's warning, once per message
    done = [self = self_, queue = h.queue,
            label = o.name.empty() ? language_ + " hook (" + h.key + ")"
                                   : o.name](const ActivationOutcome &out) {
      ScriptLane *lane = *self;
      if (out.ok || !lane || queue->reported.size() >= kMaxReported ||
          !queue->reported.insert(out.error).second)
        return;
      lane->runtime().add_warning(
          "ari: " + label + " " + out.error +
          (queue->reported.size() == kMaxReported ? " (its other failures go unreported)" : ""));
    };
  try {
    h.pid = start(std::make_shared<QueueDelivery>(h.make, h.queue, std::move(done)), std::move(o));
  } catch (const std::exception &e) {
    h.queue->events.clear(); // nothing would take them
    runtime_.add_warning("ari: " + language_ + " hook (" + h.key + ") did not start: " + e.what());
  }
}

void ScriptLane::receive(Hook &h) {
  if (h.given_up || running(h.receiver))
    return;
  if (h.receiver >= 0) { // it stopped (the runtime has warned why): when to start it again
    h.receiver = -1;
    h.quick_stops = frames_ - h.started < kQuickStop ? h.quick_stops + 1 : 1;
    if (h.quick_stops > kMaxQuickStops) {
      h.given_up = true;
      runtime_.add_warning("ari: " + language_ + " hook (" + h.key + ") stopped " +
                           std::to_string(kMaxQuickStops) +
                           " times in a row soon after starting: not restarted again");
      return;
    }
    // At once after its first stop, then 1, 2, 4 ... frames later.
    h.retry_at = frames_ + (h.quick_stops <= 1 ? 0
                                               : std::min(kRestartMaxDelay,
                                                          std::uint64_t{1} << (h.quick_stops - 2)));
  }
  if (frames_ < h.retry_at)
    return;
  ActivationOptions o;
  o.name = language_ + "/msg-recv:" + h.key;
  o.lane = ProgramLane::resident;
  o.scope = h.options.scope;
  o.priority = h.options.priority;
  std::weak_ptr<Hook> weak;
  for (const std::shared_ptr<Hook> &held : hooks_)
    if (held.get() == &h)
      weak = held;
  auto take = [self = self_, weak] {
    ScriptLane *lane = *self;
    if (std::shared_ptr<Hook> hook = weak.lock(); lane && hook)
      lane->take_queue(*hook);
  };
  try {
    h.receiver =
        start(std::make_shared<MessageReceiver>(h.key, h.queue, std::move(take)), std::move(o));
    h.started = frames_;
  } catch (const std::exception &e) {
    h.given_up = true;
    runtime_.add_warning("ari: " + language_ + " hook (" + h.key + ") did not start: " + e.what());
  }
}

unsigned ScriptLane::event_kinds() const {
  unsigned kinds = 0;
  for (const std::shared_ptr<Hook> &h : hooks_)
    kinds |= h->kind == Hook::Kind::key       ? Runtime::tap_key
             : h->kind == Hook::Kind::pointer ? Runtime::tap_pointer
             : h->kind == Hook::Kind::signal  ? Runtime::tap_signal
                                              : 0u;
  return kinds;
}

void ScriptLane::post_event(const std::string &kind, const se::value_t &event) {
  const std::vector<std::shared_ptr<Hook>> hooks = hooks_; // a delivery may add or drop hooks
  for (const std::shared_ptr<Hook> &h : hooks)
    if (h->tapped() && h->key == kind) {
      h->queue->events.push_back(event);
      take_queue(*h);
    }
}

void ScriptLane::drop_hooks() {
  bool tapped = false;
  for (const std::shared_ptr<Hook> &h : hooks_) {
    if (h->queue) // its delivery ends after the event it is on
      h->queue->events.clear();
    if (h->kind == Hook::Kind::message)
      stop(h->receiver);
    tapped = tapped || h->tapped();
  }
  hooks_.clear();
  if (tapped)
    runtime_.script_hooks_changed();
}

void ScriptLane::fire(const std::string &event) {
  const std::vector<std::shared_ptr<Hook>> hooks = hooks_; // a delivery may add or drop hooks
  for (const std::shared_ptr<Hook> &h : hooks)
    if (h->kind == Hook::Kind::event && h->key == event)
      deliver(*h, se::make_dict({{"event", se::value_t(event)}}));
}

void ScriptLane::frame() {
  ++frames_;
  if (hooks_.empty())
    return;
  const std::vector<std::shared_ptr<Hook>> hooks = hooks_;
  for (const std::shared_ptr<Hook> &h : hooks) {
    if (h->queue) {
      if (h->kind == Hook::Kind::message)
        receive(*h);
      take_queue(*h); // what came as the last delivery was ending
      continue;
    }
    if (h->kind == Hook::Kind::event || running(h->pid))
      continue; // one in flight: it coalesces what happens meanwhile
    if (h->kind == Hook::Kind::frame) {
      deliver(*h, se::make_dict(
                      {{"frame", se::value_t(static_cast<std::int64_t>(exec_frame(app())))}}));
      continue;
    }
    cvc::state *node = cvc::state::instance(app()).findDescendant(h->key);
    const std::string now = node ? node->value() : std::string();
    if (now == h->last)
      continue;
    h->last = now;
    deliver(*h, se::make_dict({{"path", se::value_t(h->path)}, {"value", se::value_t(now)}}));
  }
}

void ScriptLane::deliver(Hook &hook, const se::value_t &event) {
  try {
    std::shared_ptr<ScriptActivation> a = hook.make ? hook.make(event) : nullptr;
    if (!a)
      return;
    ActivationOptions o = hook.options; // each delivery its own copy (on_done, may_call)
    hook.pid = start(std::move(a), std::move(o));
  } catch (const std::exception &e) {
    runtime_.add_warning("ari: " + language_ + " hook" +
                         (hook.key.empty() ? std::string() : " (" + hook.key + ")") +
                         " did not start: " + e.what());
  }
}

} // namespace ariadne
} // namespace cvc
