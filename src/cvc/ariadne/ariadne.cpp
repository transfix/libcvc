// Ariadne core walker + runtime (cvc::ariadne). Pure libcvc: NO ImGui/VTK.
//
// The walk drives a pluggable Backend (backend.h): the core owns the retained
// tree, the reconcile boundary, and ALL cvc::state binding; the backend only
// draws value-in / edit-out. For each bound widget the core reads (and seeds) the
// state value, hands it to a backend primitive, and writes back ONLY when the
// backend reports the edit committed — so a slider drag costs one state write on
// release, not one per frame (writes fan out to observers / replicated peers).

#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#ifdef CVC_STATE_EXEC
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/evaluator.h> // evaluation_timeout / evaluation_interrupted
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/parser.h> // parse, parse_error
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h> // §4 read-lane predicate evaluator
#endif

#include <algorithm>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// read_or_seed<T> / write<T> / resolve_bind now live in cvc/ariadne/bind.h so the
// widget walk here and the scene binder (§9) share ONE definition and can't drift.
// They are used unqualified below (same cvc::ariadne namespace).

// Read a state value as a plain string with no seeding (read-only Text view).
std::string read_string(cvc::app &ctx, const std::string &path) {
  try {
    return cvc::state::instance(ctx)(path).value();
  } catch (const std::exception &) {
    return "<unset>";
  }
}

// §4 read-lane: is this bind an s-EXPRESSION (a computed value) rather than a state path?
// State paths are dotted identifiers and never begin with '('; an s-expr always does. So the
// first non-space character decides — no ambiguity, and no new field/key needed.
bool is_expr(const std::string &bind) {
  for (char c : bind) {
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
      continue;
    return c == '(';
  }
  return false;
}

// Replace every occurrence of `token` in `s` with `rep`.
std::string replace_all(std::string s, const std::string &token, const std::string &rep) {
  if (token.empty())
    return s;
  for (std::size_t pos = s.find(token); pos != std::string::npos;
       pos = s.find(token, pos + rep.size()))
    s.replace(pos, token.size(), rep);
  return s;
}

// §3 repeat: deep-copy a template widget, substituting the loop-index token `{i}` with the
// 0-based `index` in every field an instance uses to address distinct state or to display
// (id/label/bind/on/predicates/tooltip/options). Recurses into children. (Single-level: a
// nested `repeat` shares the same `{i}` token and is not independently indexed — a follow-up.)
Widget substitute_index(const Widget &w, int index) {
  const std::string idx = std::to_string(index);
  static const std::string kTok = "{i}";
  Widget out = w; // copies every field
  out.id = replace_all(out.id, kTok, idx);
  out.label = replace_all(out.label, kTok, idx);
  out.bind = replace_all(out.bind, kTok, idx);
  out.on = replace_all(out.on, kTok, idx);
  out.visible_when = replace_all(out.visible_when, kTok, idx);
  out.enabled_when = replace_all(out.enabled_when, kTok, idx);
  out.disabled_when = replace_all(out.disabled_when, kTok, idx);
  out.tooltip = replace_all(out.tooltip, kTok, idx);
  out.options_expr = replace_all(out.options_expr, kTok, idx);
  out.children.clear();
  out.children.reserve(w.children.size());
  for (const Widget &c : w.children)
    out.children.push_back(substitute_index(c, index));
  return out;
}

} // namespace

// --- custom widget type registry (§ extensibility) --------------------------
namespace {
std::mutex &widget_mutex() {
  static std::mutex m;
  return m;
}
std::unordered_map<std::string, WidgetEmitFn> &widget_registry() {
  static std::unordered_map<std::string, WidgetEmitFn> r;
  return r;
}
WidgetEmitFn lookup_widget(const std::string &type) {
  std::lock_guard<std::mutex> lock(widget_mutex());
  auto it = widget_registry().find(type);
  return it != widget_registry().end() ? it->second : WidgetEmitFn{};
}
} // namespace

void register_widget_type(const std::string &type, WidgetEmitFn emit) {
  if (!emit || type.empty())
    return;
  std::lock_guard<std::mutex> lock(widget_mutex());
  widget_registry()[type] = std::move(emit);
}

bool has_widget_type(const std::string &type) {
  std::lock_guard<std::mutex> lock(widget_mutex());
  return widget_registry().find(type) != widget_registry().end();
}

// --- the init: block runner (state_exec, gated by CVC_STATE_EXEC) ------------

bool have_state_exec() {
#ifdef CVC_STATE_EXEC
  return true;
#else
  return false;
#endif
}

bool run_init(cvc::app &app, const std::string &prefix, const std::string &script,
              std::vector<std::string> *errors) {
  const auto err = [&](const std::string &m) {
    if (errors)
      errors->push_back(m);
  };
  if (script.empty())
    return true; // nothing to run
#ifndef CVC_STATE_EXEC
  (void)app;
  (void)prefix;
  err("ari: init: script present but this libcvc was built without state_exec "
      "(CVC_STATE_EXEC=OFF) — the script did not run");
  return false;
#else
  namespace se = cvc::state_exec;
  try {
    // All of these must outlive execute()+run(): the intrinsics capture &ictx by
    // pointer. A synchronous run-to-completion in this one scope satisfies that.
    se::scheduler sched;
    se::memory_tracker tracker;
    auto proc = se::make_process();
    proc->pid = 1;
    proc->status = se::process_status::ready;
    se::intrinsics_context ictx;
    cvc::state &root = cvc::state::instance(app);
    ictx.sched = &sched;
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    se::apply_chroot(ictx, root, prefix); // scope writes under the document prefix
    auto env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    // Bound the init so a looping or blocking script (e.g. an accidental infinite
    // loop, or a (msg-recv ...) with no sender) can never hang the app at load: cap
    // the process (check_limits kills a runner at the cap) AND the run loop (breaks out
    // even when the sole process is blocked, which check_limits can't catch). A
    // load-time seed/compute should finish well within these.
    static constexpr uint64_t kInitMaxSteps = 10'000'000;
    static constexpr double kInitMaxSeconds = 5.0;
    se::execute_options opts;
    opts.env = env;
    opts.max_steps = kInitMaxSteps;
    opts.max_time = kInitMaxSeconds;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    sched.run(kInitMaxSteps, kInitMaxSeconds);   // bounded run to completion
    if (auto info = sched.get_process_info(pid)) {
      // Success is ONLY a normal finish. Any other terminal/non-terminal state — killed
      // (runtime error / resource limit) or still ready/running/waiting after the cap
      // (it blocked or looped past the budget) — is a reported failure, not silent.
      if (info->status != se::process_status::terminated) {
        err("ari: init: script did not complete normally (a runtime error, a resource "
            "limit, or it blocked/looped past the init budget)");
        return false;
      }
    }
    return true;
  } catch (const std::exception &e) {
    err(std::string("ari: init: script failed: ") + e.what());
    return false;
  }
#endif
}

// --- §4 read-lane: the per-frame reactive predicate evaluator ----------------
#ifdef CVC_STATE_EXEC
namespace {
namespace se = cvc::state_exec;

// Evaluates a widget's read-lane expressions (visible_when, …) each frame on ONE
// long-lived stackless evaluator over a DEFAULT-DENY, SCALAR-ONLY environment. Only scalar
// arithmetic/comparison/coercion/logic and the side-effect-free scalar/bool state readers
// are bound; every writer, scheduler op, watch, and I/O is excluded — as is anything that
// could do unbounded work in one native step (see the ctor for why: the caps are checked
// only BETWEEN steps, so compound builders/walkers and `apply` are barred, not just the
// obvious looping builtins). A predicate that reaches off the allowlist hits an unbound
// symbol and fails fail-safe. Special forms (if/begin/while/for/let/lambda/…) and the
// true/false/nil literals are parser/evaluator-intrinsic, so an empty base still runs them.
//
// Each distinct expression is parsed ONCE and cached. Every eval is bounded three ways:
// a per-eval step cap, a per-eval wall-time cap, and a per-FRAME aggregate wall-time
// budget (so many reactive widgets cannot together stall a frame). It never throws out.
//
// state-get keys are prefix-relative (the context is chroot'd to the document prefix),
// so `(state-get "demo.n")` reads the SAME key a widget `bind: demo.n` resolves to.
class ReactiveEngine {
public:
  ReactiveEngine(cvc::app &app, const std::string &prefix) {
    // The intrinsics capture &ctx_ by pointer; sched_/tracker_/proc_ back it and share
    // this object's lifetime. No scheduler op is ever exposed, so the scheduler stays
    // idle — it exists only to give the readers a well-formed context (mirrors run_init).
    proc_ = se::make_process();
    proc_->pid = 1;
    proc_->status = se::process_status::ready;
    cvc::state &root = cvc::state::instance(app);
    ctx_.sched = &sched_;
    ctx_.tracker = &tracker_;
    ctx_.proc = proc_;
    ctx_.pid = 1;
    se::apply_chroot(ctx_, root, prefix);

    // DEFAULT-DENY on two axes — builtins (via the env) AND special forms (via the evaluator
    // gate) — but now leaning on the hardened MECHANISMS rather than a bare scalar allowlist:
    //   * values_equal / to_string are memoized + bounded, so `=`/`!=`/`str` over a shared or
    //     cyclic structure cost its PHYSICAL size, never an exponential unfolding;
    //   * run() arms a per-thread deadline that nested evaluators inherit, so a loop that
    //     lives inside one native step (a method body, a generator drive) aborts at the budget.
    // With those in place a predicate can safely use compound values and structural equality.
    // What stays OUT: the SIZE-DOUBLING materializers (str-concat, append, slice, set-*), the
    // internal-loop / IO / capability builtins (generator/next/range/collect, print, apply,
    // send, every writer/scheduler/watch). And via the special-form gate below, the code-gen
    // and object-graph forms (defmacro/eval/root/defclass/super) — usable in the full-env
    // init:/action lanes, just not in a per-frame predicate.
    env_ = std::make_shared<se::environment>();
    se::environment_ptr full = se::builtins::make_default_environment();
    se::register_intrinsics(full, &ctx_);
    static const char *const kAllowed[] = {
        // arithmetic / comparison / coercion / type predicates / logic
        "+", "-", "*", "/", "%", "<", ">", "<=", ">=", "=", "!=", "int", "float", "str", "is-int",
        "is-float", "is-string", "is-null", "is-list", "type-of", "not", "and", "or",
        // bounded compound construction (by-reference; not the doubling materializers) + access
        "list", "cons", "car", "cdr", "nth", "length", "dict", "get-attr",
        // side-effect-free state readers (state-data-get deep-copies; no aliasing, no callables)
        "state-get", "state-exists", "state-children", "state-data-get", "state-root-path",
        "state-has-expiry", "state-is-expired"};
    for (const char *name : kAllowed)
      if (const se::value_t *v = full->lookup(name))
        env_->set(name, *v);
    ev_ = std::make_unique<se::stackless_evaluator>(env_);
    // Special-form gate: allow control flow, binding, functions, literals and loops (all
    // step-capped or bounded); DENY the code-generation / object-graph forms. Static so the
    // set is shared across every ReactiveEngine.
    static const std::shared_ptr<const std::set<std::string>> kAllowedForms =
        std::make_shared<const std::set<std::string>>(
            std::set<std::string>{"if", "begin", "let", "while", "for", "lambda", "defun", "set",
                                  "return", "quote", "yield", "break"});
    ev_->restrict_special_forms(kAllowedForms);
  }

  // Reset the per-FRAME aggregate budget — call once at the top of each render frame,
  // before the walk. Once the frame's cumulative predicate-eval time is spent, the
  // remaining predicates this frame skip evaluation and degrade fail-safe (hidden) with a
  // one-time warning, so N reactive widgets cannot together stall the frame.
  void begin_frame() { frame_spent_ = 0.0; }

  struct Outcome {
    bool value;        // predicate result, or the caller's `dflt` on any failure
    std::string error; // empty on success; a diagnostic otherwise
  };
  struct StringOutcome {
    std::string value; // display string, or the caller's `dflt` on any failure
    std::string error; // empty on success; a diagnostic otherwise
  };

  // Evaluate `src` as a boolean predicate (visible_when / enabled_when / …). Fail-safe: a
  // parse error, a runtime error, or a step/time-cap overrun returns {dflt, <why>}.
  Outcome eval_bool(const std::string &src, bool dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error};
    return {o.value.is_truthy(), std::string()};
  }

  // Evaluate `src` and coerce the result to a display string (computed text / fmt / …).
  // Same bounding + fail-safe as eval_bool; on any failure returns {dflt, <why>}.
  StringOutcome eval_string(const std::string &src, const std::string &dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error};
    return {display_string(o.value), std::string()};
  }

  struct StringListOutcome {
    std::vector<std::string> value; // empty on failure or a non-list/nil result
    std::string error;              // empty on success
  };

  // Evaluate `src` and coerce a LIST result to display strings (computed combo options). A
  // non-list, non-nil result becomes a single-element list; nil / failure yields empty. The
  // list length is capped (a combo with thousands of options is pathological) — extras are
  // dropped. Same bounding + fail-safe as the others.
  StringListOutcome eval_string_list(const std::string &src) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {{}, o.error};
    std::vector<std::string> out;
    if (auto *l = std::get_if<se::list_ptr>(&o.value.v)) {
      if (*l)
        for (const auto &e : **l) {
          if (out.size() >= kMaxOptions)
            break;
          out.push_back(display_string(e));
        }
    } else if (!o.value.is_nil()) {
      out.push_back(display_string(o.value));
    }
    return {std::move(out), std::string()};
  }

  struct IntOutcome {
    int64_t value;     // the caller's `dflt` on any failure
    std::string error; // empty on success
  };

  // Evaluate `src` and coerce the result to an integer (a repeat count). int as-is, double
  // truncated, a numeric string parsed; anything else is an error. Bounded + fail-safe.
  IntOutcome eval_int(const std::string &src, int64_t dflt) {
    const RawOutcome o = eval_raw(src);
    if (!o.error.empty())
      return {dflt, o.error};
    if (auto *i = std::get_if<int64_t>(&o.value.v))
      return {*i, std::string()};
    if (auto *d = std::get_if<double>(&o.value.v))
      return {static_cast<int64_t>(*d), std::string()};
    if (auto *s = std::get_if<std::string>(&o.value.v)) {
      try {
        return {static_cast<int64_t>(std::stoll(*s)), std::string()};
      } catch (const std::exception &) {
        return {dflt, "ari: reactive expr \"" + src + "\" did not yield an integer"};
      }
    }
    return {dflt, "ari: reactive expr \"" + src + "\" did not yield a number"};
  }

private:
  struct RawOutcome {
    se::value_t value; // valid only when error is empty
    std::string error; // empty on success; a diagnostic otherwise (contains src + cause)
  };

  // The shared evaluate path for every read-lane expression: compile-cached, bounded three
  // ways (per-eval step cap + per-eval time cap + per-frame aggregate), never throws out.
  RawOutcome eval_raw(const std::string &src) {
    const se::value_t *expr = compile(src);
    if (!expr)
      return {{}, "ari: reactive expr parse error in \"" + src + "\""};
    const double remaining = kFrameBudgetSeconds - frame_spent_;
    if (remaining <= 0.0)
      return {{}, "ari: reactive expr \"" + src + "\" skipped — per-frame budget exhausted"};
    try {
      se::evaluator_state st = ev_->create_state(*expr);
      const auto t0 = std::chrono::steady_clock::now();
      se::value_t r = ev_->run(st, kMaxSteps, std::min(kMaxSeconds, remaining));
      frame_spent_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (!st.done) // step cap: run() returns nil with done==false
        return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (step cap)"};
      return {std::move(r), std::string()};
    } catch (const se::evaluation_timeout &) {
      frame_spent_ += std::min(kMaxSeconds, remaining); // run() threw before the post-run charge
      return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (time cap)"};
    } catch (const se::evaluation_interrupted &) {
      return {{}, "ari: reactive expr \"" + src + "\" exceeded the eval budget (interrupted)"};
    } catch (const std::exception &e) {
      return {{}, "ari: reactive expr \"" + src + "\" failed at eval (" + e.what() + ")"};
    }
  }

  // Coerce a result value to a display string: a string is shown RAW (no quotes), a bool as
  // true/false, nil as empty; anything else via the bounded to_string (numbers → digits, a
  // list/dict → its readable, length-capped form).
  static std::string display_string(const se::value_t &v) {
    if (auto *s = std::get_if<std::string>(&v.v))
      return *s;
    if (auto *b = std::get_if<bool>(&v.v))
      return *b ? "true" : "false";
    if (v.is_nil())
      return std::string();
    return se::to_string(v);
  }

  // Parse once; cache the AST. A parse failure caches std::nullopt so a broken predicate
  // is not re-parsed every frame. Returns nullptr on a (cached) parse failure.
  const se::value_t *compile(const std::string &src) {
    auto it = compiled_.find(src);
    if (it != compiled_.end())
      return it->second ? &*it->second : nullptr;
    try {
      se::value_t v = se::parse(src);
      auto ins = compiled_.emplace(src, std::move(v));
      return &*ins.first->second;
    } catch (const se::parse_error &) {
      compiled_.emplace(src, std::nullopt);
      return nullptr;
    }
  }

  static constexpr uint64_t kMaxSteps = 100'000;       // per-eval step cap (a predicate is tiny)
  static constexpr double kMaxSeconds = 0.005;         // per-eval wall-time cap (5 ms)
  static constexpr double kFrameBudgetSeconds = 0.010; // aggregate across all predicates/frame
  static constexpr std::size_t kMaxOptions = 1024;     // cap on computed combo options

  double frame_spent_ = 0.0; // wall-time spent on predicate evals in the current frame
  se::scheduler sched_;
  se::memory_tracker tracker_;
  std::shared_ptr<se::process> proc_;
  se::intrinsics_context ctx_;
  se::environment_ptr env_;
  std::unique_ptr<se::stackless_evaluator> ev_;
  std::unordered_map<std::string, std::optional<se::value_t>> compiled_;
};
} // namespace
#endif // CVC_STATE_EXEC

struct Runtime::Impl {
  cvc::app &app;
  Backend *backend = nullptr;
  std::string prefix;

  Widget root;
  Widget pending;
  bool has_pending = false;

  std::unordered_map<std::string, std::function<void()>> handlers;
  std::vector<std::string> queued_events;

  // §4 read-lane: the reactive predicate evaluator (lazily built on first use so a UI
  // with no reactive fields pays nothing), plus de-duplicated diagnostics surfaced by
  // take_reactive_warnings(). The warnings live regardless of state_exec (the OFF path
  // also warns once). reactive_warned keeps the dedup set across drains.
#ifdef CVC_STATE_EXEC
  std::unique_ptr<ReactiveEngine> reactive;
#endif
  std::vector<std::string> reactive_warnings;
  std::set<std::string> reactive_warned;

  Impl(cvc::app &a, std::string p) : app(a), prefix(std::move(p)) {}

  // Resolve a widget bind path to an absolute cvc::state path. Delegates to the
  // shared rule (bind.h) so widget binds and scene `visible:` binds collide on the
  // same key for the same relative path.
  std::string resolve(const std::string &bind) const { return resolve_bind(prefix, bind); }

  void enqueue(const std::string &event) {
    if (!event.empty())
      queued_events.push_back(event);
  }

  void emit(const Widget &w);      // repeat + visibility gates (§4), then emit_node
  void emit_node(const Widget &w); // draw the widget assuming it is visible
  // §3 repetition: call `f(instance, i)` for each instance of `w` — once with i==-1 for a
  // plain widget, or N times (i=0..N-1, `{i}` substituted) for a `repeat` template. The
  // one place the 1-or-N expansion lives, shared by emit() and the grid layout.
  void each_instance(const Widget &w, const std::function<void(const Widget &, int)> &f);
  void emit_children(const Widget &w);
  void emit_container(const Widget &w); // lay children out per w.layout (§3.0.3b)
  void render();

  // §4 read-lane: is `w` shown this frame? True when it has no visible_when; otherwise
  // the predicate's result (fail-safe HIDDEN on a state_exec build, fail-safe SHOWN on a
  // build without state_exec — hiding every reactive widget would gut a minimal build).
  bool visible(const Widget &w);
  // §4 read-lane: is `w` disabled (greyed, non-interactive) this frame? True when
  // enabled_when is falsy OR disabled_when is truthy. Fail-safe DISABLED on a broken
  // predicate; a build without state_exec leaves it enabled (+ warns once).
  bool disabled(const Widget &w);
  // §4 read-lane: evaluate a computed-value expression `expr` to a display string.
  // Fail-safe EMPTY string on a broken predicate (§4.1 fmt→""); warns once.
  std::string eval_text(const std::string &expr);
  // §4 read-lane: evaluate a computed combo-options expression to a list of strings.
  // Fail-safe EMPTY list on a broken expr; warns once.
  std::vector<std::string> eval_options(const std::string &expr);
  // §3/§4: evaluate a repeat-count expression to a non-negative, capped int. Fail-safe 0.
  int eval_count(const std::string &expr);
  void warn_once(const std::string &msg) {
    if (reactive_warned.insert(msg).second)
      reactive_warnings.push_back(msg);
  }
};

bool Runtime::Impl::visible(const Widget &w) {
  if (w.visible_when.empty())
    return true;
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  const ReactiveEngine::Outcome o = reactive->eval_bool(w.visible_when, /*dflt=*/false);
  if (!o.error.empty())
    warn_once(o.error);
  return o.value;
#else
  warn_once("ari: visible_when on '" + (w.label.empty() ? w.id : w.label) +
            "' ignored — this libcvc was built without state_exec (CVC_STATE_EXEC=OFF)");
  return true;
#endif
}

bool Runtime::Impl::disabled(const Widget &w) {
  if (w.enabled_when.empty() && w.disabled_when.empty())
    return false;
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  // enabled_when falsy -> disabled (fail-safe dflt=false: a broken predicate disables).
  if (!w.enabled_when.empty()) {
    const ReactiveEngine::Outcome o = reactive->eval_bool(w.enabled_when, /*dflt=*/false);
    if (!o.error.empty())
      warn_once(o.error);
    if (!o.value)
      return true;
  }
  // disabled_when truthy -> disabled (fail-safe dflt=true: a broken predicate disables).
  if (!w.disabled_when.empty()) {
    const ReactiveEngine::Outcome o = reactive->eval_bool(w.disabled_when, /*dflt=*/true);
    if (!o.error.empty())
      warn_once(o.error);
    if (o.value)
      return true;
  }
  return false;
#else
  warn_once("ari: enabled_when/disabled_when on '" + (w.label.empty() ? w.id : w.label) +
            "' ignored — this libcvc was built without state_exec (CVC_STATE_EXEC=OFF)");
  return false; // can't evaluate -> leave it functional
#endif
}

std::string Runtime::Impl::eval_text(const std::string &expr) {
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  const ReactiveEngine::StringOutcome o = reactive->eval_string(expr, /*dflt=*/std::string());
  if (!o.error.empty())
    warn_once(o.error);
  return o.value;
#else
  warn_once("ari: computed text expr ignored — this libcvc was built without state_exec "
            "(CVC_STATE_EXEC=OFF)");
  return std::string();
#endif
}

std::vector<std::string> Runtime::Impl::eval_options(const std::string &expr) {
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  const ReactiveEngine::StringListOutcome o = reactive->eval_string_list(expr);
  if (!o.error.empty())
    warn_once(o.error);
  return o.value;
#else
  warn_once("ari: computed options ignored — this libcvc was built without state_exec "
            "(CVC_STATE_EXEC=OFF)");
  return {};
#endif
}

int Runtime::Impl::eval_count(const std::string &expr) {
  constexpr int64_t kMaxRepeat = 4096; // a UI with thousands of repeated widgets is pathological
#ifdef CVC_STATE_EXEC
  if (!reactive)
    reactive = std::make_unique<ReactiveEngine>(app, prefix);
  const ReactiveEngine::IntOutcome o = reactive->eval_int(expr, /*dflt=*/0);
  if (!o.error.empty())
    warn_once(o.error);
  int64_t n = o.value;
  if (n < 0)
    n = 0;
  if (n > kMaxRepeat) {
    warn_once("ari: repeat count " + std::to_string(n) + " exceeds the cap (" +
              std::to_string(kMaxRepeat) + ") — clamped");
    n = kMaxRepeat;
  }
  return static_cast<int>(n);
#else
  (void)kMaxRepeat;
  warn_once("ari: repeat ignored — this libcvc was built without state_exec (CVC_STATE_EXEC=OFF)");
  return 0;
#endif
}

void Runtime::Impl::emit_children(const Widget &w) {
  for (const Widget &c : w.children)
    emit(c);
}

// Lay a container's children out per its layout (§3.0.3b): a Grid/Horizontal
// layout flows them into the backend's sized tracks; anything else is a plain
// vertical stack. Used for both a Window's body and a Group.
void Runtime::Impl::emit_container(const Widget &w) {
  Backend &b = *backend;
  if (w.layout.kind == LayoutKind::Grid || w.layout.kind == LayoutKind::Horizontal) {
    const std::string &id = w.id.empty() ? w.label : w.id;
    if (b.begin_grid(w.layout, id.c_str())) {
      for (const Widget &c : w.children) {
        // Expand a `repeat` child into per-instance cells; a plain child is one instance.
        // Decide visibility BEFORE advancing the cell, so a hidden child/instance consumes
        // no cell (otherwise it would leave an empty cell and shift the following siblings).
        each_instance(c, [&](const Widget &inst, int i) {
          if (!visible(inst))
            return;
          b.grid_next_cell();
          if (i >= 0)
            b.push_id(std::to_string(i).c_str());
          emit_node(inst);
          if (i >= 0)
            b.pop_id();
        });
      }
      b.end_grid();
    }
  } else {
    emit_children(w);
  }
}

void Runtime::Impl::each_instance(const Widget &w,
                                  const std::function<void(const Widget &, int)> &f) {
  if (w.repeat.empty()) {
    f(w, -1); // a plain widget: one instance, no index
    return;
  }
  const int n = eval_count(w.repeat); // capped, fail-safe 0
  for (int i = 0; i < n; ++i) {
    Widget inst = substitute_index(w, i);
    inst.repeat.clear(); // the instance renders once
    f(inst, i);
  }
}

void Runtime::Impl::emit(const Widget &w) {
  each_instance(w, [this](const Widget &inst, int i) {
    if (i >= 0)
      backend->push_id(std::to_string(i).c_str()); // per-instance identity
    if (visible(inst))                             // §4: a falsy visible_when hides it + subtree
      emit_node(inst);
    if (i >= 0)
      backend->pop_id();
  });
}

void Runtime::Impl::emit_node(const Widget &w) {
  Backend &b = *backend;
  // §4 read-lane: wrap the widget (and its subtree, emitted within the cases) in the
  // backend's disabled scope when a reactive enabled_when/disabled_when says so.
  const bool dis = disabled(w);
  if (dis)
    b.begin_disabled();
  const char *label = w.label.c_str();
  switch (w.kind) {
  case Kind::Menubar:
    if (b.begin_main_menu_bar()) {
      emit_children(w);
      b.end_main_menu_bar();
    }
    break;

  case Kind::Menu:
    if (b.begin_menu(label)) {
      emit_children(w);
      b.end_menu();
    }
    break;

  case Kind::MenuItemToggle: {
    const std::string path = resolve(w.bind);
    const bool cur = read_or_seed<int>(app, path, w.bdef ? 1 : 0) != 0;
    const BoolEdit e = b.menu_item_toggle(label, cur);
    if (e.committed)
      write<int>(app, path, e.value ? 1 : 0);
    break;
  }

  case Kind::MenuItemAction:
    if (b.menu_item_action(label))
      enqueue(w.on);
    break;

  case Kind::Window: {
    // The stable identity is the id (defaults to the label); the backend keeps a
    // window's geometry keyed on it. Begin/End pair unconditionally.
    const std::string &id = w.id.empty() ? w.label : w.id;
    const bool visible = b.begin_window(w.label.c_str(), id.c_str(), w.size, w.frame_border);
    if (visible) {
      b.push_id(id.c_str());
      emit_container(w); // §3.0.3b: the window body honours w.layout
      b.pop_id();
    }
    b.end_window();
    break;
  }

  case Kind::Group:
    emit_container(w); // §3.0.3b: a group honours w.layout (grid / vertical)
    break;

  case Kind::Text:
    if (w.literal_text)
      b.text_line(label);
    else if (is_expr(w.bind))
      // §4 read-lane (homoiconic): a `bind` that is an s-expression (starts with '(') is a
      // COMPUTED value re-evaluated each frame, not a state path. State paths never start
      // with '(', so this is unambiguous. Read-only, bounded, fail-safe empty.
      b.text_value(label, eval_text(w.bind));
    else
      b.text_value(label, read_string(app, resolve(w.bind)));
    break;

  case Kind::Separator:
    b.separator();
    break;

  case Kind::Checkbox: {
    const std::string path = resolve(w.bind);
    const bool cur = read_or_seed<int>(app, path, w.bdef ? 1 : 0) != 0;
    const BoolEdit e = b.checkbox(label, cur);
    if (e.committed)
      write<int>(app, path, e.value ? 1 : 0);
    break;
  }

  case Kind::SliderInt: {
    const std::string path = resolve(w.bind);
    const int cur = read_or_seed<int>(app, path, w.idef);
    const IntEdit e = b.slider_int(label, path.c_str(), cur, w.ilo, w.ihi);
    if (e.committed)
      write<int>(app, path, e.value);
    break;
  }

  case Kind::SliderFloat: {
    const std::string path = resolve(w.bind);
    const double cur = read_or_seed<double>(app, path, w.def);
    const DoubleEdit e = b.slider_double(label, path.c_str(), cur, w.lo, w.hi, w.fmt.c_str());
    if (e.committed)
      write<double>(app, path, e.value);
    break;
  }

  case Kind::Combo: {
    // §4 read-lane: options are either a static list or a computed expression re-evaluated
    // each frame. A local holds the computed list so its lifetime spans the emit.
    std::vector<std::string> computed;
    if (!w.options_expr.empty())
      computed = eval_options(w.options_expr);
    const std::vector<std::string> &opts = w.options_expr.empty() ? w.options : computed;
    if (opts.empty())
      break;
    const std::string path = resolve(w.bind);
    const std::string fallback = w.sdef.empty() ? opts.front() : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, fallback);
    int idx = 0;
    for (std::size_t i = 0; i < opts.size(); ++i)
      if (opts[i] == cur) {
        idx = static_cast<int>(i);
        break;
      }
    const IndexEdit e = b.combo(label, idx, opts);
    if (e.changed && e.index >= 0 && e.index < static_cast<int>(opts.size()))
      write<std::string>(app, path, opts[e.index]); // store the TEXT, not an index
    break;
  }

  case Kind::Button:
    if (b.button(label))
      enqueue(w.on);
    break;

  case Kind::Custom: {
    // Three ways a custom widget renders, in priority order:
    //  1. a COMPOSITIONAL fn (register_widget_type) — composes built-ins through a
    //     context that reuses the core's binding; works on every backend;
    //  2. else the BACKEND's novel-primitive escape (custom_widget) — the core reads
    //     the bound value, hands it in, and writes back on the committed edge;
    //  3. else a placeholder (neither knows this type).
    if (const WidgetEmitFn fn = lookup_widget(w.custom_type)) {
      WidgetEmitContext ctx;
      ctx.emit = [this](const Widget &child) { emit(child); };
      ctx.read = [this](const std::string &bind) { return read_string(app, resolve(bind)); };
      ctx.fire = [this](const std::string &event) { enqueue(event); };
      fn(w, ctx);
      break;
    }
    const std::string path = w.bind.empty() ? std::string() : resolve(w.bind);
    const std::string cur = w.bind.empty() ? std::string() : read_string(app, path);
    const CustomEdit e = b.custom_widget(w.custom_type.c_str(), cur, w);
    if (!e.handled)
      b.text_line(("[" + w.custom_type + "?]").c_str()); // neither composed nor backend-drawn
    else if (e.committed && !w.bind.empty())
      write<std::string>(app, path, e.value);
    break;
  }
  }
  // §4 read-lane: attach a hover tooltip to the widget just drawn (the backend's last item).
  // Literal text, or a computed expression (starts with '('); empty result = no tooltip.
  if (!w.tooltip.empty()) {
    const std::string tip = is_expr(w.tooltip) ? eval_text(w.tooltip) : w.tooltip;
    if (!tip.empty())
      b.set_tooltip(tip.c_str());
  }
  if (dis)
    b.end_disabled();
}

void Runtime::Impl::render() {
  if (!backend)
    return;
  // Reconcile commit boundary (roadmap §11.5.1): a queued tree swap is applied
  // here, at the top of the frame, before any node is emitted — never mid-walk.
  if (has_pending) {
    root = std::move(pending);
    pending = Widget{};
    has_pending = false;
  }
#ifdef CVC_STATE_EXEC
  if (reactive)
    reactive->begin_frame(); // §4: reset the per-frame reactive eval budget
#endif
  backend->begin_frame();
  emit(root);
  backend->end_frame();
}

// --------------------------------------------------------------------------

Runtime::Runtime(cvc::app &app, std::string prefix) : m_(new Impl(app, std::move(prefix))) {}

Runtime::~Runtime() = default;

void Runtime::set_backend(Backend *backend) { m_->backend = backend; }

void Runtime::set_root(Widget root) {
  m_->pending = std::move(root);
  m_->has_pending = true;
}

void Runtime::on(std::string event, std::function<void()> handler) {
  m_->handlers[std::move(event)] = std::move(handler);
}

void Runtime::render() { m_->render(); }

void Runtime::drain() {
  // Run the queued action events on the host thread, then clear. Never called
  // from inside render() (the deferred intent-buffer discipline, §7.2/§4.7).
  std::vector<std::string> events;
  events.swap(m_->queued_events);
  for (const std::string &e : events) {
    auto it = m_->handlers.find(e);
    if (it != m_->handlers.end() && it->second)
      it->second();
  }
}

std::vector<std::string> Runtime::take_reactive_warnings() {
  std::vector<std::string> out;
  out.swap(m_->reactive_warnings); // reactive_warned stays -> still deduped across drains
  return out;
}

} // namespace ariadne
} // namespace cvc
