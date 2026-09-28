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
#include <cvc/ariadne/input.h> // §4.6 InputEvent — Runtime::post_input feeds on_key/on_pointer
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#ifdef CVC_STATE_EXEC
#include <cvc/core/state_exec/async_scheduler.h> // §4.7 app-wide action scheduler (exec_scheduler)
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/evaluator.h> // evaluation_timeout / evaluation_interrupted
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/parser.h> // parse, parse_error
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h> // §4 read-lane predicate evaluator
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
namespace ariadne {

namespace {

// read_or_seed<T> / write<T> / resolve_bind now live in cvc/ariadne/bind.h so the
// widget walk here and the scene binder (§9) share ONE definition and can't drift.
// They are used unqualified below (same cvc::ariadne namespace).

// Read a state value as a plain string with no seeding (read-only Text view). Follows a
// transparent-link hole to its target (§12), so a bound Text in a mounted module shows the
// granted parent value, matching every other read path.
std::string read_string(cvc::app &ctx, const std::string &path) {
  try {
    return read_effective(cvc::state::instance(ctx)(path)).value();
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

// §G7 colour: parse an "r,g,b" state string into rgb[3] (comma- or space-separated). Returns false
// (leaving `out` at the caller's default) unless exactly three numbers parse.
bool parse_rgb3(const std::string &s, float out[3]) {
  std::string t = s;
  for (char &c : t)
    if (c == ',')
      c = ' ';
  std::istringstream is(t);
  float a = 0, b = 0, c = 0;
  if (is >> a >> b >> c) {
    out[0] = a;
    out[1] = b;
    out[2] = c;
    return true;
  }
  return false;
}
std::string format_rgb3(const float rgb[3]) {
  std::ostringstream os;
  os << rgb[0] << "," << rgb[1] << "," << rgb[2];
  return os.str();
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
  // §12: a repeated MOUNT (Widget::scope set) gets a per-instance sub-prefix — append the loop
  // index so each instance's fragment binds to its OWN state subtree (includes.<as>.<i>), rather
  // than all N instances sharing one, which would violate repeat's per-instance-state contract.
  if (!out.scope.empty())
    out.scope += "." + idx;
  // §12: and its parent-scope holes are {i}-substituted too, so an instance can grant a
  // per-instance target (link: { slot: fleet.{i} }) — mirroring the bind/on substitution above.
  for (LinkHole &h : out.links) {
    h.name = replace_all(h.name, kTok, idx);
    h.target = replace_all(h.target, kTok, idx);
  }
  out.children.clear();
  out.children.reserve(w.children.size());
  for (const Widget &c : w.children)
    // A nested `repeat` template is left UNtouched so its own expansion owns its `{i}` (rather
    // than the outer index clobbering the inner template's bindings). Single-level still: the
    // inner cannot also reference the outer index — the loader warns on a nested repeat.
    out.children.push_back(c.repeat.empty() ? substitute_index(c, index) : c);
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

// Host-contributed program-lane intrinsics registry (§ nav verbs et al.). Kept OUTSIDE the
// CVC_STATE_EXEC guard so register/clear are always callable (the providers are simply never
// applied in a build with no program lanes). ActionIntrinsicProvider is state_exec-typed via
// forward-decls.
namespace {
std::mutex &action_intrinsics_mutex() {
  static std::mutex m;
  return m;
}
std::vector<ActionIntrinsicProvider> &action_intrinsics_registry() {
  static std::vector<ActionIntrinsicProvider> v;
  return v;
}
std::vector<ActionIntrinsicProvider> action_intrinsics_snapshot() {
  std::lock_guard<std::mutex> lock(action_intrinsics_mutex());
  return action_intrinsics_registry();
}
} // namespace

void register_action_intrinsics(ActionIntrinsicProvider provider) {
  if (!provider)
    return;
  std::lock_guard<std::mutex> lock(action_intrinsics_mutex());
  action_intrinsics_registry().push_back(std::move(provider));
}
void clear_action_intrinsics() {
  std::lock_guard<std::mutex> lock(action_intrinsics_mutex());
  action_intrinsics_registry().clear();
}

#ifdef CVC_STATE_EXEC
namespace {
// Run `script` as a FULL-ENV state_exec program chrooted to `prefix`, bounded by (max_steps,
// max_seconds, max_bytes). Returns "" on a clean, normal finish, else a human diagnostic (no
// leading "ari:" — the caller frames it). Shared by the load-time `init:` lane (run_init) and the
// per-interaction ACTION lane (a program `on:`, drained by Runtime::drain) — same trust model
// (see the ReactiveEngine note on the full-env init:/action lanes): the full intrinsics env with
// writes confined to the document prefix, resource-capped, and it never throws.
std::string run_scoped_program(cvc::app &app, const std::string &prefix, const std::string &script,
                               uint64_t max_steps, double max_seconds, uint64_t max_bytes) {
  namespace se = cvc::state_exec;
  try {
    // All of these must outlive execute()+run(): the intrinsics capture &ictx by pointer. A
    // synchronous run-to-completion in this one scope satisfies that.
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
    se::apply_chroot(ictx, root, prefix); // scope writes under the document/mount prefix
    auto env = se::builtins::make_default_environment();
    se::register_intrinsics(env, &ictx);
    // Let a host bind extra program-lane intrinsics (nav verbs, …) into this env — AFTER the
    // standard ones, so it can add or override. Snapshot the registry so a provider is free to
    // register/throw without holding the lock.
    for (const ActionIntrinsicProvider &p : action_intrinsics_snapshot())
      if (p)
        p(env, ictx);
    // Bound the run so a looping or blocking script (an accidental infinite loop, or a
    // (msg-recv ...) with no sender) can never hang: cap the process (check_limits kills a runner
    // at the cap) AND the run loop (breaks out even when the sole process is blocked, which
    // check_limits can't catch).
    se::execute_options opts;
    opts.env = env;
    opts.max_steps = max_steps;
    opts.max_time = max_seconds;
    opts.max_memory = max_bytes;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    sched.run(max_steps, max_seconds);           // bounded run to completion
    if (auto info = sched.get_process_info(pid)) {
      // Success is ONLY a normal finish. Any other terminal/non-terminal state — killed
      // (runtime error / resource limit) or still ready/running/waiting after the cap
      // (it blocked or looped past the budget) — is a reported failure, not silent.
      if (info->status != se::process_status::terminated)
        return "did not complete normally (a runtime error, a resource limit, or it "
               "blocked/looped past the budget)";
    }
    return std::string();
  } catch (const std::exception &e) {
    return std::string("failed: ") + e.what();
  }
}
} // namespace
#endif // CVC_STATE_EXEC

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
  // A load-time seed/compute is generous — well above the per-tick action budget (§7.4).
  static constexpr uint64_t kInitMaxSteps = 10'000'000;
  static constexpr double kInitMaxSeconds = 5.0;
  const std::string e = run_scoped_program(app, prefix, script, kInitMaxSteps, kInitMaxSeconds, 0);
  if (!e.empty()) {
    err("ari: init: script " + e);
    return false;
  }
  return true;
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
    root_path_ = prefix;

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

  // §12: re-chroot the read-lane to `path` so a mounted fragment's predicates read state
  // relative to its own sub-prefix, exactly as its widget binds resolve there. The evaluator,
  // env, and compile cache are unchanged — only ctx_.root moves (apply_chroot repoints it,
  // creating the subtree if absent). A no-op when the scope is unchanged (the common case).
  void set_root_path(cvc::app &app, const std::string &path) {
    if (path == root_path_)
      return;
    se::apply_chroot(ctx_, cvc::state::instance(app), path);
    root_path_ = path;
  }

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
    if (auto *d = std::get_if<double>(&o.value.v)) {
      // Casting a non-finite or out-of-range double to int64 is UB — guard it (a repeat
      // count that computes to inf/nan/huge is a broken expr -> fail-safe dflt).
      if (!std::isfinite(*d) || *d < -9.2e18 || *d > 9.2e18)
        return {dflt, "ari: reactive expr \"" + src + "\" did not yield an in-range integer"};
      return {static_cast<int64_t>(*d), std::string()};
    }
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
  std::string root_path_;    // §12: the scope the ctx_ is currently chrooted to (set_root_path)
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
  // A queued action fired this frame (§7.2/§4.7 deferred intent buffer). `event` is EITHER a bare
  // event name (looked up in `handlers`, the host C++ seam) OR — when it starts with '(' — a
  // state_exec PROGRAM (the action lane), run at drain against `prefix` (the mount scope captured
  // at enqueue time, so a mounted fragment's program writes its own subtree, like its binds).
  struct QueuedAction {
    std::string event;
    std::string prefix;
  };
  std::vector<QueuedAction> queued_events;

  // §4 read-lane: the reactive predicate evaluator (lazily built on first use so a UI
  // with no reactive fields pays nothing), plus de-duplicated diagnostics surfaced by
  // take_reactive_warnings(). The warnings live regardless of state_exec (the OFF path
  // also warns once). reactive_warned keeps the dedup set across drains.
#ifdef CVC_STATE_EXEC
  std::unique_ptr<ReactiveEngine> reactive;
#endif
  std::vector<std::string> reactive_warnings;
  std::set<std::string> reactive_warned;
  int frame_instances = 0; // §3: repeat instances emitted this frame (bounds many repeats)

  // §4.7 action lane: a unique owner tag for this Runtime's processes on the app-wide
  // scheduler (app.exec_scheduler()), so teardown reaps exactly this document's process
  // group via kill_owner. Monotonic so two Runtimes on the same prefix never collide.
  static std::atomic<uint64_t> owner_seq;
  std::string owner_ = "ari#" + std::to_string(owner_seq.fetch_add(1)) + ":" + prefix;

  Impl(cvc::app &a, std::string p) : app(a), prefix(std::move(p)) {}
  ~Impl();

#ifdef CVC_STATE_EXEC
  // A submitted `on:` program's persistent context. The intrinsics capture &ictx by
  // pointer, so — unlike the load-time init: lane's one-scope run — an action that
  // suspends (await/sleep/msg-recv) needs its context kept alive across frames until the
  // process terminates or is reaped. Field order matters for teardown: ictx (which points
  // into tracker/proc) is declared last so it destructs first.
  struct ActionContext {
    cvc::state_exec::memory_tracker tracker;
    cvc::state_exec::process_ptr proc;
    cvc::state_exec::environment_ptr env;
    cvc::state_exec::intrinsics_context ictx;
  };
  std::unordered_map<int, std::unique_ptr<ActionContext>> live_actions_;
  bool used_scheduler_ = false; // did we ever submit to the app scheduler? (gate teardown reap)

  // Submit a program `on:` as a process on the app-wide scheduler (owner-tagged, chrooted
  // to `prefix`), keeping its ActionContext alive in live_actions_. Returns immediately.
  void submit_action(const std::string &action_prefix, const std::string &script);
  // Pump the app scheduler a bounded slice, then sweep finished actions out of
  // live_actions_ (reporting a killed one; a still-parked one persists to a later frame).
  void pump_and_sweep_actions();

  // §7.1 resident on:tick handler. tick_script_ is the author's per-frame body (empty = none).
  // ensure_tick_resident() submits ONE long-lived process (idempotent) that loops
  // (while t (begin (msg-recv "<owner>#tick") <body>)) — parking between frames; post_tick()
  // wakes it once per drain. Reaped by ~Impl's kill_owner (owner-tagged) or set_tick_program.
  std::string tick_script_;    // §7.1 on:tick body (woken every frame, coalesced)
  std::string key_script_;     // §4.6 document-level on_key body (keyboard events, via post_input)
  std::string pointer_script_; // §4.6 document-level on_pointer body (mouse events, via post_input)
  std::unique_ptr<ActionContext> tick_resident_, key_resident_, pointer_resident_;
  int tick_resident_pid_ = -1, key_resident_pid_ = -1, pointer_resident_pid_ = -1;
  std::string tick_channel() const { return owner_ + "#tick"; }
  std::string key_channel() const { return owner_ + "#key"; }
  std::string pointer_channel() const { return owner_ + "#pointer"; }
  // Submit ONE long-lived resident wrapping `body` on `channel`, storing its context/pid in the
  // slots (idempotent — no-op if body is empty or a resident is already running). The wrapper is
  // (while t (let ((event (msg-recv "<channel>"))) <body>)): msg-recv parks the process between
  // deliveries and binds each delivered value as `event`, so an on_key/on_pointer body reads
  // (get-attr event "key") etc. (on:tick's body simply ignores `event`).
  void ensure_resident(const std::string &channel, const std::string &body,
                       std::unique_ptr<ActionContext> &ctx_slot, int &pid_slot);
  void ensure_tick_resident();   // submit the tick resident (woken by the coalesced post_tick)
  void post_tick();              // wake the tick resident once this drain
  void ensure_input_residents(); // submit the key + pointer residents (woken by post_input)
  // Deliver one input event: serialize it to a dict and post_message it (NO coalesce — a burst in
  // one frame must all arrive) onto the key channel (keyboard) or pointer channel (mouse).
  void deliver_input(const InputEvent &ev);
#endif

  // §12: the active mount scope during emit. Empty stack = the document prefix; each entry is
  // a composed sub-prefix pushed when the walk enters a mounted subtree (Widget::scope) and
  // popped on exit. current_prefix() is what binds and read-lane reads resolve against.
  std::vector<std::string> scope_stack;
  const std::string &current_prefix() const {
    return scope_stack.empty() ? prefix : scope_stack.back();
  }

  // §12 load: sub-prefixes whose parent-scope holes (link:) have been planted, so we wire each
  // mount's link nodes ONCE (idempotent, but findDescendant+linkTo per hole per frame is waste).
  // Cleared on a reconcile (the swapped-in tree may mount different fragments).
  std::set<std::string> wired_mounts;
  // §12 load: the state paths of every hole planted for the CURRENT tree. On a reconcile these
  // are torn down (clearLink) BEFORE the new tree re-wires — otherwise a fragment re-mounted at
  // the same sub-prefix (same `as`) would inherit the previous tree's grants it was never given,
  // defeating default-deny (a hole node persists in cvc::state; clearing wired_mounts alone only
  // re-plants, never removes).
  std::vector<std::string> planted_holes;
  // Plant the transparent link-node holes for a mount: for each, a node at <sub_prefix>.<name>
  // links to `target` resolved against the mount's PARENT scope (leading '/' = app root).
  void wire_holes(const std::string &parent_prefix, const std::string &sub_prefix,
                  const std::vector<LinkHole> &links);

  // §12 load: sub-prefixes whose mounted fragment's init: has already run. Init is a ONE-SHOT
  // seed, so unlike wired_mounts this is NOT cleared on a reconcile — the fragment's init runs
  // once when the mount first appears, never re-running when the tree is rebuilt.
  std::set<std::string> ran_inits;
  // Run a mounted fragment's init: once at its sub-prefix (chrooted there by run_init); any error
  // is surfaced as a one-time runtime warning rather than thrown into the frame.
  void run_mount_init(const std::string &sub_prefix, const std::string &script);

  // Resolve a widget bind path to an absolute cvc::state path. Delegates to the
  // shared rule (bind.h) so widget binds and scene `visible:` binds collide on the
  // same key for the same relative path — against the active mount scope (§12).
  std::string resolve(const std::string &bind) const {
    return resolve_bind(current_prefix(), bind);
  }

#ifdef CVC_STATE_EXEC
  // Lazily build the read-lane engine (a UI with no reactive fields pays nothing), then point
  // it at the active mount scope so a mounted fragment's predicates read its own sub-prefix,
  // matching where its binds resolve. Every reactive helper goes through here.
  ReactiveEngine &ensure_reactive() {
    if (!reactive)
      reactive = std::make_unique<ReactiveEngine>(app, prefix);
    reactive->set_root_path(app, current_prefix());
    return *reactive;
  }
#endif

  void enqueue(const std::string &event) {
    if (!event.empty())
      queued_events.push_back({event, current_prefix()}); // capture the mount scope for a program
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
  // Like eval_text, but for free-text fields (tooltip): if `expr` does NOT evaluate cleanly it
  // is almost certainly a literal that merely starts with '(' — return it VERBATIM and do NOT
  // warn. A genuinely computed value is returned; a broken one degrades to its own text.
  std::string eval_text_or_literal(const std::string &expr);
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
  const ReactiveEngine::Outcome o = ensure_reactive().eval_bool(w.visible_when, /*dflt=*/false);
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
  ReactiveEngine &re = ensure_reactive();
  // enabled_when falsy -> disabled (fail-safe dflt=false: a broken predicate disables).
  if (!w.enabled_when.empty()) {
    const ReactiveEngine::Outcome o = re.eval_bool(w.enabled_when, /*dflt=*/false);
    if (!o.error.empty())
      warn_once(o.error);
    if (!o.value)
      return true;
  }
  // disabled_when truthy -> disabled (fail-safe dflt=true: a broken predicate disables).
  if (!w.disabled_when.empty()) {
    const ReactiveEngine::Outcome o = re.eval_bool(w.disabled_when, /*dflt=*/true);
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
  const ReactiveEngine::StringOutcome o =
      ensure_reactive().eval_string(expr, /*dflt=*/std::string());
  if (!o.error.empty())
    warn_once(o.error);
  return o.value;
#else
  warn_once("ari: computed text expr ignored — this libcvc was built without state_exec "
            "(CVC_STATE_EXEC=OFF)");
  return std::string();
#endif
}

std::string Runtime::Impl::eval_text_or_literal(const std::string &expr) {
#ifdef CVC_STATE_EXEC
  const ReactiveEngine::StringOutcome o =
      ensure_reactive().eval_string(expr, /*dflt=*/std::string());
  // Success -> the computed value; failure -> the literal verbatim (no warning: a tooltip that
  // starts with '(' is far more likely plain text than a broken expression).
  return o.error.empty() ? o.value : expr;
#else
  return expr; // no state_exec -> show the literal
#endif
}

std::vector<std::string> Runtime::Impl::eval_options(const std::string &expr) {
#ifdef CVC_STATE_EXEC
  const ReactiveEngine::StringListOutcome o = ensure_reactive().eval_string_list(expr);
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
  const ReactiveEngine::IntOutcome o = ensure_reactive().eval_int(expr, /*dflt=*/0);
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
void Runtime::Impl::wire_holes(const std::string &parent_prefix, const std::string &sub_prefix,
                               const std::vector<LinkHole> &links) {
  cvc::state &root = cvc::state::instance(app);
  for (const LinkHole &h : links) {
    // The hole node the module sees inside its chroot; the target it grants, resolved against the
    // mount's PARENT scope. A transparent link so a read sees through (bind.h read_effective /
    // state-get read_through_link); setLinkWritable gates the write (rw -> target, ro -> local).
    const std::string hole_path = resolve_bind(sub_prefix, h.name);
    const std::string target = resolve_bind(parent_prefix, h.target);
    // Record the path BEFORE linking, so even if linkTo throws mid-way the node is still torn
    // down on the next reconcile (clearLink is idempotent on a non-link node).
    planted_holes.push_back(hole_path);
    try {
      cvc::state &node = root(hole_path);
      node.linkTo(target, cvc::state::link_mode::transparent);
      node.setLinkWritable(h.writable);
    } catch (const std::exception &) {
      // Never throw into a frame; a hole that cannot be wired simply isn't (the module then
      // reads/writes its own local node — sandboxed, no parent reach).
    }
  }
}

void Runtime::Impl::run_mount_init(const std::string &sub_prefix, const std::string &script) {
  std::vector<std::string> errs;
  run_init(app, sub_prefix, script, &errs);
  for (const std::string &e : errs)
    warn_once("ari: mount init at '" + sub_prefix + "': " + e);
}

void Runtime::Impl::emit_container(const Widget &w) {
  Backend &b = *backend;
  // §12: a mount point (Widget::scope) scopes its CHILDREN — the loaded fragment — to a deeper
  // sub-prefix; the mount widget's own reactive fields already evaluated in the parent scope
  // (in emit() before we got here). Push the composed scope for the children walk, pop after
  // (RAII, so a backend throw cannot leave the stack unbalanced).
  const bool scoped = !w.scope.empty();
  if (scoped) {
    const std::string parent_prefix = current_prefix(); // before the push
    const std::string sub_prefix = resolve_bind(parent_prefix, w.scope);
    // Plant the mount's parent-scope holes once, BEFORE emitting the fragment that reads them.
    if (!w.links.empty() && wired_mounts.insert(sub_prefix).second)
      wire_holes(parent_prefix, sub_prefix, w.links);
    // Run the fragment's init: once (after its holes exist, so init may seed through them). Kept
    // in ran_inits, NOT cleared on reconcile — a one-shot seed, never re-run per frame/rebuild.
    if (!w.init_script.empty() && ran_inits.insert(sub_prefix).second)
      run_mount_init(sub_prefix, w.init_script);
    scope_stack.push_back(sub_prefix);
  }
  struct ScopeGuard {
    std::vector<std::string> &stack;
    bool active;
    ~ScopeGuard() {
      if (active)
        stack.pop_back();
    }
  } guard{scope_stack, scoped};

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
  // Per-FRAME instance budget: each repeat is capped individually, but many repeats could
  // still compound into an unbounded per-frame walk (substitute_index deep-copy + emit),
  // which the eval-time budget doesn't see. Bound the total across the whole frame.
  constexpr int kMaxFrameInstances = 16384;
  for (int i = 0; i < n; ++i) {
    if (frame_instances >= kMaxFrameInstances) {
      warn_once("ari: too many repeated widget instances this frame (cap " +
                std::to_string(kMaxFrameInstances) + ") — remaining instances skipped");
      break;
    }
    ++frame_instances;
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
    const bool cur =
        read_bool_or_seed(app, path, w.bdef ? 1 : 0) != 0; // tolerant of "true"/"false"
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
    const bool cur =
        read_bool_or_seed(app, path, w.bdef ? 1 : 0) != 0; // tolerant of "true"/"false"
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
    // A `values:` list (validated equal-length at load, static options only) maps each displayed
    // option to the value STORED in the key (e.g. an int); otherwise the key holds the option TEXT.
    const bool mapped =
        !w.values.empty() && w.options_expr.empty() && w.values.size() == opts.size();
    const std::vector<std::string> &store = mapped ? w.values : opts;
    const std::string path = resolve(w.bind);
    const std::string fallback = w.sdef.empty() ? store.front() : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, fallback);
    int idx = 0;
    for (std::size_t i = 0; i < store.size(); ++i)
      if (store[i] == cur) { // match the STORED value (or text), display opts[idx]
        idx = static_cast<int>(i);
        break;
      }
    const IndexEdit e = b.combo(label, idx, opts); // always DISPLAY the labels
    if (e.changed && e.index >= 0 && e.index < static_cast<int>(store.size()))
      write<std::string>(app, path, store[e.index]); // store the mapped value (or the text)
    break;
  }

  case Kind::Color: {
    const std::string path = resolve(w.bind);
    const std::string dflt = w.sdef.empty() ? std::string("1,1,1") : w.sdef;
    const std::string cur = read_or_seed<std::string>(app, path, dflt);
    float rgb[3] = {1.0f, 1.0f, 1.0f};
    parse_rgb3(cur, rgb); // leaves the 1,1,1 default on a parse failure
    const ColorEdit e = b.color(label, rgb);
    if (!e.drawn) {
      b.text_value(label, cur); // a backend with no colour widget -> show the "r,g,b" value
      break;
    }
    if (e.committed)
      write<std::string>(app, path, format_rgb3(e.rgb)); // store "r,g,b"
    break;
  }

  case Kind::Image: {
    // The image name: a bound key's value (a combo can switch rasters) else the static src.
    const std::string name =
        w.bind.empty() ? w.src : read_or<std::string>(app, resolve(w.bind), w.src);
    const float width = static_cast<float>(w.img_size > 0 ? w.img_size : 256.0);
    if (!b.draw_image(name.c_str(), width)) // a backend with no image path -> show the name as text
      b.text_value(w.label.empty() ? "image" : w.label.c_str(),
                   name.empty() ? std::string("(no image)") : name);
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
  // §4.6 widget-level on_click: if this widget carries an on_click and the backend reports the item
  // just drawn was clicked, enqueue it — a fire-once action drained + submitted like a Button's on:
  // (a program on: runs through state_exec; a bare name routes to a host handler). Sourced from the
  // backend's per-item click (ImGui::IsItemClicked), so it works wherever ImGui gets input (native
  // VTK-interactor AND wasm) — no raw hit-test. Defaults off on a backend without per-item input.
  // Inside the disabled scope (below), so a disabled widget never fires (IsItemClicked is false).
  if (!w.on_click.empty() && b.item_clicked())
    enqueue(w.on_click);
  // §4 read-lane: attach a hover tooltip to the widget just drawn (the backend's last item).
  // A tooltip is free human text, so — unlike a bind (a dotted state path never begins with
  // '(') — one that merely LOOKS like an expression ("(optional) …", "(beta)") must not be
  // mistaken for one. So: try to evaluate an expr-looking tooltip, but if it does NOT evaluate
  // cleanly, fall back to showing it verbatim (no spurious warning). A genuine computed tooltip
  // still works; a literal that happens to start with '(' is shown as-is.
  if (!w.tooltip.empty()) {
    std::string tip = w.tooltip;
    if (is_expr(w.tooltip))
      tip = eval_text_or_literal(w.tooltip);
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
    // §12: tear down the OLD tree's parent-scope holes before the new tree re-wires its own, so a
    // fragment re-mounted at the same sub-prefix cannot inherit a grant the new manifest omits
    // (clearLink turns the hole back into a plain, sandboxed local node). Re-wiring happens as
    // this same frame walks the new tree, so there is no window where a stale grant is live.
    cvc::state &root_state = cvc::state::instance(app);
    for (const std::string &hole : planted_holes) {
      try {
        if (cvc::state *n = root_state.findDescendant(hole))
          n->clearLink();
      } catch (const std::exception &) {
      }
    }
    planted_holes.clear();
    wired_mounts.clear();
  }
#ifdef CVC_STATE_EXEC
  if (reactive)
    reactive->begin_frame(); // §4: reset the per-frame reactive eval budget
#endif
  frame_instances = 0; // §3: reset the per-frame repeat-expansion budget
  scope_stack.clear(); // §12: start every frame at the document scope (RAII keeps it balanced;
                       // this is belt-and-suspenders so one bad frame can't leak into the next)
  backend->begin_frame();
  emit(root);
  backend->end_frame();
}

// --------------------------------------------------------------------------

std::atomic<uint64_t> Runtime::Impl::owner_seq{0};

Runtime::Impl::~Impl() {
#ifdef CVC_STATE_EXEC
  // Reap this document's whole process group so a parked (await/sleep/msg-recv) action never
  // lingers on the shared app scheduler after its Runtime is gone. Only touch the scheduler if
  // we ever used it (exec_scheduler() lazily builds it — don't force it at teardown otherwise).
  if (used_scheduler_)
    app.exec_scheduler().kill_owner(owner_);
#endif
}

#ifdef CVC_STATE_EXEC
void Runtime::Impl::submit_action(const std::string &action_prefix, const std::string &script) {
  namespace se = cvc::state_exec;
  // §7.4 per-activation caps (same as the old synchronous action lane): the STEP cap is the
  // deterministic guard on an action's real work; wall-time is a generous anti-hang backstop;
  // memory the §7.4 cap. Full env, writes chrooted to the action's mount prefix.
  static constexpr uint64_t kActionMaxSteps = 5000;
  static constexpr double kActionMaxSeconds = 0.1;
  static constexpr uint64_t kActionMaxBytes = 262144;
  try {
    auto &sched = app.exec_scheduler();
    used_scheduler_ = true;
    auto ac = std::make_unique<ActionContext>();
    ac->proc = se::make_process(); // placeholder for ictx.proc; the RUNNING process is the
    ac->proc->status = se::process_status::ready; // scheduler's own, reached via current_process()
    cvc::state &root = cvc::state::instance(app);
    ac->ictx.sched = &sched;
    ac->ictx.tracker = &ac->tracker;
    ac->ictx.proc = ac->proc;
    se::apply_chroot(ac->ictx, root, action_prefix); // confine writes to the document/mount subtree
    ac->env = se::builtins::make_default_environment();
    se::register_intrinsics(ac->env, &ac->ictx);
    // Host program-lane intrinsics (nav verbs, …), AFTER the standard ones (add or override).
    for (const ActionIntrinsicProvider &p : action_intrinsics_snapshot())
      if (p)
        p(ac->env, ac->ictx);
    se::execute_options opts;
    opts.env = ac->env;
    opts.owner = owner_; // so ~Impl's kill_owner reaps exactly this document's processes
    opts.max_steps = kActionMaxSteps;
    opts.max_time = kActionMaxSeconds;
    opts.max_memory = kActionMaxBytes;
    const int pid = sched.execute(script, opts); // throws se::parse_error on a syntax error
    live_actions_[pid] = std::move(ac);          // keep the context alive until the process ends
  } catch (const std::exception &e) {
    warn_once(std::string("ari: on: action program failed: ") + e.what() + " [" + script + "]");
  }
}

void Runtime::Impl::pump_and_sweep_actions() {
  namespace se = cvc::state_exec;
  auto &sched = app.exec_scheduler();
  // Apply any cross-thread post_message() deliveries FIRST, so a delivery that readies a parked
  // (msg-recv …) receiver is seen by sync_run's has_runnable() gate — otherwise, with everything
  // parked, sync_run would skip stepping and never drain the ingress itself.
  sched.drain_ingress();
  // Frame boundary: re-ready any process parked by (await …) on a PRIOR drain. Called here (once
  // per drain), NOT inside step()/sync_run, so an await crosses exactly one frame.
  sched.wake_awaiting();
  // Pump a bounded slice: a quick (non-suspending) action completes within THIS drain (so a
  // fire-then-check interaction still sees its effect immediately), while an action that parks
  // on await/sleep/msg-recv yields and resumes on a later frame. Global step + wall-time
  // backstops bound the pump; per-process step caps kill runaways.
  static constexpr uint64_t kPumpMaxSteps = 200000;
  static constexpr double kPumpMaxSeconds = 0.1;
  sched.sync_run(kPumpMaxSteps, kPumpMaxSeconds);
  // Sweep: drop finished actions, report a killed one, keep a still-parked one for a later frame.
  for (auto it = live_actions_.begin(); it != live_actions_.end();) {
    auto info = sched.get_process_info(it->first);
    if (!info || info->status == se::process_status::terminated) {
      it = live_actions_.erase(it); // normal completion (or already reaped from the process map)
    } else if (info->status == se::process_status::killed) {
      warn_once("ari: on: action program did not complete (a runtime error or resource limit)");
      it = live_actions_.erase(it);
    } else {
      ++it; // ready/running/waiting — still in flight (e.g. suspended on await/msg-recv/sleep)
    }
  }
}

void Runtime::Impl::ensure_resident(const std::string &channel, const std::string &body,
                                    std::unique_ptr<ActionContext> &ctx_slot, int &pid_slot) {
  if (body.empty() || pid_slot >= 0)
    return; // nothing to run, or already submitted (residents submit ONCE, not per frame)
  namespace se = cvc::state_exec;
  try {
    auto &sched = app.exec_scheduler();
    used_scheduler_ = true;
    auto ac = std::make_unique<ActionContext>();
    ac->proc = se::make_process();
    ac->proc->status = se::process_status::ready;
    cvc::state &root = cvc::state::instance(app);
    ac->ictx.sched = &sched;
    ac->ictx.tracker = &ac->tracker;
    ac->ictx.proc = ac->proc;
    se::apply_chroot(ac->ictx, root, prefix); // resident writes confined to the document prefix
    ac->env = se::builtins::make_default_environment();
    se::register_intrinsics(ac->env, &ac->ictx);
    for (const ActionIntrinsicProvider &p : action_intrinsics_snapshot())
      if (p)
        p(ac->env, ac->ictx);
    // Park-then-run loop on `channel`: msg-recv parks the process between deliveries and BINDS each
    // delivered value as `event`, so an on_key/on_pointer body reads (get-attr event "key");
    // on:tick simply ignores `event`. The body runs once per delivery, then loops back to park. No
    // total step cap (a resident loops forever; the per-frame pump budget bounds each activation,
    // and the msg-recv guarantees a park every iteration).
    const std::string wrapped =
        "(while t (let ((event (msg-recv \"" + channel + "\"))) " + body + "))";
    se::execute_options opts;
    opts.env = ac->env;
    opts.owner = owner_; // reaped by ~Impl's kill_owner(owner_) on teardown
    opts.max_steps = 0;  // unlimited total — see above
    opts.max_time = 0.0;
    pid_slot = sched.execute(wrapped, opts);
    ctx_slot = std::move(ac);
  } catch (const std::exception &e) {
    warn_once("ari: resident failed to start [" + channel + "]: " + e.what());
  }
}

void Runtime::Impl::ensure_tick_resident() {
  ensure_resident(tick_channel(), tick_script_, tick_resident_, tick_resident_pid_);
}

void Runtime::Impl::post_tick() {
  if (tick_resident_pid_ < 0)
    return; // no tick resident registered
  // Wake the tick resident for one activation this frame. Post only when nothing is queued on the
  // channel, so a resident that can't keep up doesn't accrue a backlog of ticks (coalesce).
  auto &sched = app.exec_scheduler();
  if (sched.pending_message_count(tick_channel()) == 0)
    sched.post_message(tick_channel(), cvc::state_exec::value_t(std::string("tick")));
}

void Runtime::Impl::ensure_input_residents() {
  ensure_resident(key_channel(), key_script_, key_resident_, key_resident_pid_);
  ensure_resident(pointer_channel(), pointer_script_, pointer_resident_, pointer_resident_pid_);
}

void Runtime::Impl::deliver_input(const InputEvent &ev) {
  namespace se = cvc::state_exec;
  // Keyboard events feed the on_key resident, mouse events the on_pointer resident. Drop the event
  // if that handler is not registered (no point queueing events nothing will drain).
  const bool keyboard = ev.is_keyboard();
  if ((keyboard ? key_resident_pid_ : pointer_resident_pid_) < 0)
    return;
  // A dict with EVERY field always populated: get-attr throws on a missing key, so an on_* body can
  // read any field of any event kind (unused fields are 0/""). These names are the .ari event
  // contract — (get-attr event "kind"/"key"/"mods"/"x"/"y"/"dx"/"dy"/"button"/"clicks"/"repeat").
  std::vector<std::pair<std::string, se::value_t>> f;
  f.emplace_back("kind", se::value_t(std::string(ev.kind_name())));
  f.emplace_back("key", se::value_t(ev.key));
  f.emplace_back("mods", se::value_t(static_cast<int64_t>(ev.mods)));
  f.emplace_back("repeat", se::value_t(ev.repeat));
  f.emplace_back("x", se::value_t(static_cast<double>(ev.x)));
  f.emplace_back("y", se::value_t(static_cast<double>(ev.y)));
  f.emplace_back("dx", se::value_t(static_cast<double>(ev.dx)));
  f.emplace_back("dy", se::value_t(static_cast<double>(ev.dy)));
  f.emplace_back("button", se::value_t(static_cast<int64_t>(ev.button)));
  f.emplace_back("clicks", se::value_t(static_cast<int64_t>(ev.clicks)));
  // NO coalesce (unlike post_tick): every event must arrive. A burst in one frame FIFO-queues on
  // the channel and the resident drains them one per loop iteration during the pump.
  app.exec_scheduler().post_message(keyboard ? key_channel() : pointer_channel(),
                                    se::make_dict(std::move(f)));
}
#endif // CVC_STATE_EXEC

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

void Runtime::set_tick_program(std::string script) {
#ifdef CVC_STATE_EXEC
  // Replacing the program kills any running resident so the new body starts fresh next drain.
  if (m_->tick_resident_pid_ >= 0) {
    m_->app.exec_scheduler().kill(m_->tick_resident_pid_);
    m_->tick_resident_.reset();
    m_->tick_resident_pid_ = -1;
  }
  m_->tick_script_ = std::move(script);
#else
  (void)script; // residents need state_exec — no-op otherwise
#endif
}

void Runtime::set_key_program(std::string script) {
#ifdef CVC_STATE_EXEC
  if (m_->key_resident_pid_ >= 0) {
    m_->app.exec_scheduler().kill(m_->key_resident_pid_);
    m_->key_resident_.reset();
    m_->key_resident_pid_ = -1;
  }
  m_->key_script_ = std::move(script);
#else
  (void)script;
#endif
}

void Runtime::set_pointer_program(std::string script) {
#ifdef CVC_STATE_EXEC
  if (m_->pointer_resident_pid_ >= 0) {
    m_->app.exec_scheduler().kill(m_->pointer_resident_pid_);
    m_->pointer_resident_.reset();
    m_->pointer_resident_pid_ = -1;
  }
  m_->pointer_script_ = std::move(script);
#else
  (void)script;
#endif
}

void Runtime::post_input(const InputEvent &ev) {
#ifdef CVC_STATE_EXEC
  // Submit the input residents on first use so the delivered event is consumed, not dropped, then
  // deliver it. The pump in drain() steps the resident; feed input BEFORE drain() for same-frame
  // delivery (see the header contract). A no-op without state_exec.
  m_->ensure_input_residents();
  m_->deliver_input(ev);
#else
  (void)ev;
#endif
}

void Runtime::render() { m_->render(); }

void Runtime::drain() {
  // Run the queued action events on the host thread, then clear. Never called
  // from inside render() (the deferred intent-buffer discipline, §7.2/§4.7).
  std::vector<Impl::QueuedAction> events;
  events.swap(m_->queued_events);
  for (const Impl::QueuedAction &a : events) {
    // A program action (`on:` starting with '(', like a computed `bind:`/`tooltip:`) runs through
    // state_exec — the north-star lane: a flag toggle or reset is pure .ari, no C++ handler.
    if (is_expr(a.event)) { // a program on: (whitespace-tolerant, exactly like a computed bind:)
#ifdef CVC_STATE_EXEC
      // SUBMIT it to the app-wide scheduler (owner-tagged, chrooted to the action's mount prefix)
      // and return — no run-to-completion inside drain(). The per-frame pump below advances it; a
      // quick action still finishes this drain, while an (await …)/(msg-recv …)/(sleep …) parks
      // and resumes on a later frame instead of blocking the UI thread (§4.7).
      m_->submit_action(a.prefix, a.event);
#else
      m_->warn_once("ari: on: a program action needs state_exec (CVC_STATE_EXEC=OFF); it did not "
                    "run [" +
                    a.event + "]");
#endif
      continue;
    }
    auto it = m_->handlers.find(a.event); // a bare event name -> the host C++ handler seam
    if (it != m_->handlers.end() && it->second)
      it->second();
  }
#ifdef CVC_STATE_EXEC
  // §7.1 resident on:tick: submit the resident once (idempotent), then post it this frame's tick.
  m_->ensure_tick_resident();
  m_->post_tick();
  // §4.6 input residents (on_key/on_pointer): ensure they exist (idempotent; post_input also does,
  // but a host that set a program yet fed no input this frame still gets its resident submitted).
  m_->ensure_input_residents();
  // Advance submitted actions + the resident — and anything still-parked from earlier frames — a
  // bounded slice, then sweep the finished ones. Runs once used_scheduler_ latches.
  if (m_->used_scheduler_)
    m_->pump_and_sweep_actions();
#endif
}

std::vector<std::string> Runtime::take_reactive_warnings() {
  std::vector<std::string> out;
  out.swap(m_->reactive_warnings); // reactive_warned stays -> still deduped across drains
  return out;
}

} // namespace ariadne
} // namespace cvc
