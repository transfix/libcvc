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
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h> // async_work: an exported call parks on its activation
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h> // a call from a program is its sliced work
#include <cvc/state/state_exec/evaluator.h> // step_kind
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <map>
#include <mutex>
#include <span>
#include <stdexcept>
#include <thread>
#include <utility>

namespace cvc {
namespace ariadne {

namespace se = cvc::state_exec;

namespace {

using steady = std::chrono::steady_clock;

// ── the registry ────────────────────────────────────────────────────────────────────────────────

std::mutex &engines_mutex() {
  static std::mutex m;
  return m;
}
std::map<std::string, std::shared_ptr<ScriptEngine>> &engines() {
  static std::map<std::string, std::shared_ptr<ScriptEngine>> e;
  return e;
}

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

// The process's program (the header's loop). A reply is (ok value): #t and the value of the call
// or the await, or #f and the exception it raised.
const char kProgram[] =
    "(let ((k (script-step)))\n"
    "  (while (not (= k \"done\"))\n"
    "    (if (= k \"yield\")\n"
    "        (await #f)\n"
    "        (if (= k \"call\")\n"
    "            (script-reply (try (list #t (apply (script-callee) (script-arguments)))\n"
    "                            (catch (e) (list #f e))))\n"
    "            (if (= k \"await\")\n"
    "                (script-reply (try (list #t (await (script-awaited)))\n"
    "                                (catch (e) (list #f e)))))))\n"
    "    (set k (script-step))))\n"
    "(script-finish)\n";

} // namespace

// What an activation's natives close over (the header's ownership note).
struct ActivationRunner {
  cvc::app *app = nullptr;
  std::shared_ptr<ScriptActivation> activation; // let go of once its process ends
  std::uint64_t slice_steps = kSliceSteps;
  double frame_seconds = kFrameBudget; // 0: no frame budget
  std::shared_ptr<frame_budget> budget;
  std::function<bool(const std::string &)> may_call;
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

  // A slice's start: its budget, and nothing asked for yet (the reply, if any, is this slice's).
  static void open(ScriptSlice &s, std::uint64_t budget) {
    s.budget_ = budget;
    s.charged_ = 0;
    s.next_ = ScriptSlice::Next::more;
    s.out_ = se::value_t();
    s.function_.clear();
    s.args_.clear();
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

  // (script-step): run a slice, and say what the loop does next.
  se::value_t step() {
    if (finished || !activation)
      return se::value_t(std::string("done"));
    if (frame_spent(*app, budget.get(), frame_seconds))
      return se::value_t(std::string("yield")); // the reply, if any, waits for the next frame
    ScriptSlice &s = slice;
    open(s, slice_steps);
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
}

} // namespace

// ── the registry ────────────────────────────────────────────────────────────────────────────────

void register_script_engine(std::shared_ptr<ScriptEngine> engine) {
  if (!engine)
    return;
  const std::string language = engine->language();
  if (language.empty())
    return;
  {
    std::lock_guard<std::mutex> lock(engines_mutex());
    engines()[language] = std::move(engine);
  }
  // Its blocks are kept for the runtimes that run them (AppRuntime::use_script_engine).
  register_ari_block(language, [language](const Value &content, LoadResult &out) {
    out.extras.emplace_back(language, content);
  });
}

void unregister_script_engine(const std::string &language) {
  std::shared_ptr<ScriptEngine> gone; // let go of outside the lock
  std::lock_guard<std::mutex> lock(engines_mutex());
  auto it = engines().find(language);
  if (it == engines().end())
    return;
  gone = std::move(it->second);
  engines().erase(it);
}

std::shared_ptr<ScriptEngine> find_script_engine(const std::string &language) {
  std::lock_guard<std::mutex> lock(engines_mutex());
  auto it = engines().find(language);
  return it == engines().end() ? nullptr : it->second;
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

// ── ScriptLane ──────────────────────────────────────────────────────────────────────────────────

struct ScriptLane::Hook {
  enum class Kind { event, frame, watch };
  Kind kind = Kind::event;
  std::string key;  // the event, or the watched node's path from the app's root
  std::string path; // a watch hook's path as it was given, for its events
  ScriptHook make;
  ActivationOptions options;
  int pid = -1;     // the delivery in flight (a frame or watch hook's)
  std::string last; // a watch hook's: the value it last delivered, or saw when it was set
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

  LaneProcess p;
  p.program = kProgram;
  p.name = options.name.empty() ? language_ + "/activation" : std::move(options.name);
  p.lane = options.lane;
  p.scope = std::move(options.scope);
  p.max_steps = options.max_steps;
  p.max_time = options.max_time;
  p.max_memory = options.max_memory;
  p.priority = options.priority;
  p.extend = [run](const se::environment_ptr &env, se::intrinsics_context &ictx) {
    run->env = env.get();
    run->ictx = &ictx;
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

// An exported call from a program on the app's scheduler (export_activation): the program's own
// deep-data work (data_job.h), one slice of the activation per data step of its process, each
// slice's steps charged to that process, so the program's caps bound it. A call or an await a
// slice asks for is the program's, made in its process by a closure around a try (kSlicedAsk):
// its value or its exception comes back to the next slice instead of unwinding the job. When the
// job goes before its activation finished (a throw, a cap, a kill), the activation is cancelled.
class SlicedCall : public se::data_job {
public:
  SlicedCall(cvc::app &app, std::shared_ptr<ScriptActivation> activation, std::string name,
             std::uint64_t slice_steps, double frame_seconds, std::shared_ptr<frame_budget> budget,
             std::function<bool(const std::string &)> may_call, se::environment_ptr env)
      : app_(app), activation_(std::move(activation)), name_(std::move(name)),
        slice_steps_(slice_steps), frame_seconds_(frame_seconds), budget_(std::move(budget)),
        may_call_(std::move(may_call)), env_(std::move(env)) {
    static const se::value_t body = se::parse(kSlicedAsk);
    auto ask = std::make_shared<se::closure>();
    ask->params = {se::symbol{"k"}, se::symbol{"f"}, se::symbol{"a"}};
    ask->body = {body};
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
    ActivationRunner::open(s, slice_steps_);
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
    se::async_work work;
    work.start = [self, app, name, make, options,
                  copied = std::vector<se::value_t>(args.begin(), args.end())](
                     const std::shared_ptr<se::async_call> &call) {
      ScriptLane *lane = *self;
      if (!lane)
        throw std::runtime_error(name + ": its document has gone");
      se::async_host &host = app->exec_async_host();
      std::shared_ptr<ScriptActivation> made = make(copied);
      if (!made) {
        if (call->settle(se::value_t(), nullptr))
          host.finish(call);
        return;
      }
      auto stop = std::make_shared<std::atomic<bool>>(false);
      ActivationOptions o = options;
      o.on_done = [call, &host, name](const ActivationOutcome &out) {
        if (call->cancel_requested())
          return;
        std::exception_ptr error;
        if (!out.ok)
          error = !out.exception.is_nil()
                      ? std::make_exception_ptr(se::dsl_exception(out.exception))
                      : std::make_exception_ptr(std::runtime_error(name + ": " + out.error));
        if (call->settle(out.ok ? out.value : se::value_t(), error))
          host.finish(call);
      };
      lane->start(std::make_shared<CallActivation>(std::move(made), stop), std::move(o));
      call->on_cancel([stop] { stop->store(true, std::memory_order_release); });
    };
    return work;
  };
  // A call from where the program can't park (init:): the activation, run to its end in it.
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
  // A call from a program on the app's scheduler: the activation as the program's own sliced work.
  se::native_fn sliced =
      [self = self_, app, language, name, make, may_call = options.may_call,
       slices = options.slice_steps](std::span<const se::value_t> args) -> se::value_t {
    ScriptLane *lane = *self;
    if (!lane)
      throw std::runtime_error(name + ": its document has gone");
    se::detail::data_step_scope *data = se::detail::data_step_scope::current();
    const se::process_ptr &proc = app->exec_scheduler().current_process();
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
    return se::run_data_job(std::make_unique<SlicedCall>(*app, std::move(made), name, slice_steps,
                                                         frame_seconds, budget_of(*app, language),
                                                         may_call, proc->state.global_env));
  };
  runtime_.register_async_work_function(name, std::move(parked), std::move(blocking),
                                        std::move(sliced));
}

ActivationOutcome ScriptLane::run(std::shared_ptr<ScriptActivation> activation,
                                  ActivationOptions options, double max_seconds) {
  if (app().exec_scheduler().current_pid() >= 0)
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
    if (running(pid) && app().exec_scheduler().get_stats().ready == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // parked: on a pool, a timer
  }
  return ended->out;
}

void ScriptLane::stop(int pid) {
  if (pid >= 0)
    app().exec_scheduler().kill(pid);
}

bool ScriptLane::running(int pid) const {
  if (pid < 0)
    return false;
  const auto info = app().exec_scheduler().get_process_info(pid);
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

void ScriptLane::drop_hooks() { hooks_.clear(); }

void ScriptLane::fire(const std::string &event) {
  const std::vector<std::shared_ptr<Hook>> hooks = hooks_; // a delivery may add or drop hooks
  for (const std::shared_ptr<Hook> &h : hooks)
    if (h->kind == Hook::Kind::event && h->key == event)
      deliver(*h, se::make_dict({{"event", se::value_t(event)}}));
}

void ScriptLane::frame() {
  if (hooks_.empty())
    return;
  const std::vector<std::shared_ptr<Hook>> hooks = hooks_;
  for (const std::shared_ptr<Hook> &h : hooks) {
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
