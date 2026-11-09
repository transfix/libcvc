// script_engine.cpp — see the header: the engine registry, the lanes, their hooks, and
// activations run as processes of the document's program lane (Runtime::start_process).
//
// Ownership, as a sliced task's (sliced_task.cpp): an activation's record (the activation, its
// slice, what it asked for) is held by the natives of its process's environment, through an
// anchor. When the process ends, for any reason, the environment lets go of the anchor, and the
// anchor of the activation, cancelling one that never finished. The process's done (the Runtime's
// sweep) holds the record too, for the result it hands on_done.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/loader.h> // register_ari_block: an engine's language is a block key
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h> // step_kind
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/exec_group.h> // an isolated document's group: its scheduler
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <map>
#include <mutex>
#include <span>
#include <stdexcept>
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

  // A reply the next slice gets without the loop's help: a call that was refused, or that names
  // nothing.
  void refuse(const char *cls, const std::string &message) {
    slice.replied_ = true;
    slice.reply_ = se::value_t();
    slice.exception_ = se::make_exception(se::exception_class(cls), message);
    slice.error_ = se::exception_summary(slice.exception_);
  }

  // (script-step): run a slice, and say what the loop does next.
  se::value_t step() {
    if (finished || !activation)
      return se::value_t(std::string("done"));
    if (frame_seconds > 0.0) { // this language's activations have had their time this frame
      const std::uint64_t frame = exec_frame(*app);
      if (budget->frame != frame) {
        budget->frame = frame;
        budget->spent = 0.0;
      }
      if (budget->spent >= frame_seconds)
        return se::value_t(std::string("yield")); // the reply, if any, waits for the next frame
    }
    ScriptSlice &s = slice;
    s.budget_ = slice_steps;
    s.charged_ = 0;
    s.next_ = ScriptSlice::Next::more;
    s.out_ = se::value_t();
    s.function_.clear();
    s.args_.clear();
    const steady::time_point t0 = steady::now();
    const auto settle = [&] {
      if (budget)
        budget->spent += std::chrono::duration<double>(steady::now() - t0).count();
      if (s.charged_ > 0 && ictx && ictx->sched)
        if (const se::process_ptr &proc = ictx->sched->current_process())
          proc->state.stats.add_steps(se::step_kind::data, s.charged_);
      s.replied_ = false;
      s.reply_ = se::value_t();
      s.error_.clear();
      s.exception_ = se::value_t();
    };
    try {
      activation->step(s);
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
        refuse("PermissionError", s.function_ + ": not among the functions this activation may "
                                                "call (its imports)");
        return se::value_t(std::string("more"));
      }
      const se::value_t *fn = env ? env->lookup(s.function_) : nullptr;
      if (!fn) {
        refuse("NameError", s.function_ + ": no such function here");
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

  // (script-reply (ok value)): what the call or the await came back with, for the next slice.
  void reply(const se::value_t &r) {
    const auto *l = std::get_if<se::list_ptr>(&r.v);
    if (!l || !*l || (*l)->size() != 2)
      return;
    slice.replied_ = true;
    if ((**l)[0].is_truthy()) {
      slice.reply_ = (**l)[1];
      slice.error_.clear();
      slice.exception_ = se::value_t();
    } else {
      slice.reply_ = se::value_t();
      slice.exception_ = (**l)[1];
      slice.error_ = se::exception_summary(slice.exception_);
      if (slice.error_.empty())
        slice.error_ = "Exception";
    }
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
  se::builtins::register_fn(env, "script-reply", [anchor](args_t a) -> se::value_t {
    if (!a.empty())
      anchor->run->reply(a[0]);
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
      done(o);
    };
  return runtime_.runtime().start_process(std::move(p));
}

// Where the document's processes, its activations among them, run: its isolated group's scheduler,
// else the app's (group 0). current_group decides nothing and moves nothing.
static se::async_scheduler &document_scheduler(AppRuntime &runtime) {
  se::exec_group *const group = runtime.runtime().current_group();
  return group ? group->sched() : runtime.app().exec_scheduler();
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
