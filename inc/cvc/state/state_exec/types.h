/**
 * @file types.h
 * @brief Core value types and environment model for the state_exec DSL.
 *
 * Defines value_t (a tagged union over nil, bool, int64, double, string,
 * symbol, list, dict, lambda, native_fn, error), along with environment_t
 * (lexically-scoped variable bindings) and execute_options (per-process
 * resource limits).
 */
#ifndef CVC_STATE_EXEC_TYPES_H
#define CVC_STATE_EXEC_TYPES_H

#include <boost/any.hpp>
#include <chrono>
#include <compare>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace cvc::state_exec {

// Forward declarations
struct closure;
struct environment;
struct generator;

using closure_ptr = std::shared_ptr<closure>;
using environment_ptr = std::shared_ptr<environment>;

/// Symbol name in the DSL.
struct symbol {
  std::string name;
  auto operator<=>(const symbol &) const = default;
  bool operator==(const symbol &o) const { return name == o.name; }
};

/// Typed data object wrapping boost::any from state::data().
struct data_object {
  boost::any payload;
  std::string type_name;
  bool is_type(const std::string &t) const { return type_name == t; }
};
using data_object_ptr = std::shared_ptr<data_object>;
using generator_ptr = std::shared_ptr<generator>;

// Ordered map preserving insertion order for dicts
using ordered_map = std::vector<std::pair<std::string, struct value_tag>>;

// Forward-declare value_t (variant needs complete types for some alternatives;
// we use shared_ptr to break the recursion)
struct value_tag;
using list_ptr = std::shared_ptr<std::vector<value_tag>>;
using dict_ptr = std::shared_ptr<std::vector<std::pair<std::string, value_tag>>>;
using native_fn = std::function<value_tag(std::span<const value_tag>)>;

/// A raw byte buffer — the binary counterpart of the (text) `string` type (SE-3 of the Unicode
/// roadmap). A distinct wrapper (NOT a second std::string alternative — the variant would be
/// ambiguous) so the variant discriminates text from binary: `string` is UTF-8 TEXT with codepoint
/// semantics (length/index/slice by codepoint, see SE-1), whereas `bytes` is opaque OCTETS with
/// byte semantics and no case/normalization/codepoint meaning. Backed by std::string purely as a
/// byte container. Binary data (HTTP octet-stream bodies, Python `bytes`, binary payloads) belongs
/// here, never in a `string`; `string.encode`/`bytes.decode` are the only sanctioned crossings.
struct bytes_value {
  std::string data;
  bool operator==(const bytes_value &o) const { return data == o.data; }
};

namespace detail {
/// Drop one owner of a nested value's storage (a list, dict, closure, generator or
/// environment) without recursing once per nesting level. Freeing a 100000-deep list through
/// plain shared_ptr destructors takes a native stack frame per level and overflows the stack.
/// Here the first release on a thread runs a loop: owners dropped while it frees something
/// (that thing's elements, bindings, enclosing scope) are queued and freed one at a time by the
/// loop, so the stack stays a few frames deep whatever the depth.
void release_nested(std::shared_ptr<void> owner) noexcept;

/// Drop `owner` through release_nested when it is the last one; dropping any other owner only
/// decrements a count. (Should another thread drop the last but one meanwhile, this drop frees
/// the storage inline, and its elements start releases of their own: a few frames, still.)
template <class T> void drop_owner(std::shared_ptr<T> &owner) noexcept {
  if (owner && owner.use_count() == 1)
    release_nested(std::move(owner));
}
} // namespace detail

/// The DSL value variant.  All runtime values are represented as value_t.
///
/// Variants:
///   monostate  → nil
///   bool       → #t / #f
///   int64_t    → integer
///   double     → float
///   string     → string literal
///   symbol     → symbol name
///   list_ptr   → ordered list
///   closure_ptr → lambda / defun closure
///   dict_ptr   → ordered dictionary
///   native_fn  → C++ callable (built-in or stdlib bridge)
///   data_object_ptr → typed data from state::data()
///   generator_ptr → lazy sequence with yield support
///   bytes_value → raw octet buffer (binary; the counterpart of the text `string`)
struct value_tag {
  using variant_type =
      std::variant<std::monostate, bool, int64_t, double, std::string, symbol, list_ptr,
                   closure_ptr, dict_ptr, native_fn, data_object_ptr, generator_ptr, bytes_value>;

  variant_type v;

  value_tag() : v(std::monostate{}) {}

  // Implicit constructors for convenience
  value_tag(std::monostate) : v(std::monostate{}) {}
  value_tag(bool b) : v(b) {}
  value_tag(int64_t i) : v(i) {}
  value_tag(int i) : v(static_cast<int64_t>(i)) {}
  value_tag(double d) : v(d) {}
  value_tag(const std::string &s) : v(s) {}
  value_tag(std::string &&s) : v(std::move(s)) {}
  value_tag(const char *s) : v(std::string(s)) {}
  value_tag(symbol s) : v(std::move(s)) {}
  value_tag(list_ptr l) : v(std::move(l)) {}
  value_tag(closure_ptr c) : v(std::move(c)) {}
  value_tag(dict_ptr d) : v(std::move(d)) {}
  value_tag(native_fn f) : v(std::move(f)) {}
  value_tag(data_object_ptr d) : v(std::move(d)) {}
  value_tag(generator_ptr g) : v(std::move(g)) {}
  value_tag(bytes_value b) : v(std::move(b)) {}

  value_tag(const value_tag &) = default;
  value_tag(value_tag &&) = default;
  value_tag &operator=(const value_tag &) = default;
  value_tag &operator=(value_tag &&) = default;

  /// Nested storage is freed through detail::drop_owner, never recursively.
  ~value_tag() {
    if (auto *l = std::get_if<list_ptr>(&v))
      detail::drop_owner(*l);
    else if (auto *d = std::get_if<dict_ptr>(&v))
      detail::drop_owner(*d);
    else if (auto *c = std::get_if<closure_ptr>(&v))
      detail::drop_owner(*c);
    else if (auto *g = std::get_if<generator_ptr>(&v))
      detail::drop_owner(*g);
  }

  /// Check if the value is nil (monostate).
  bool is_nil() const { return std::holds_alternative<std::monostate>(v); }

  /// Truthiness: nil and false are falsy, everything else is truthy.
  bool is_truthy() const {
    if (is_nil())
      return false;
    if (auto *b = std::get_if<bool>(&v))
      return *b;
    return true;
  }

  /// Type name string for error messages / introspection.
  std::string type_name() const;
};

using value_t = value_tag;

/// Closure: a lambda body + captured environment.
struct closure {
  std::vector<symbol> params;
  bool variadic = false; // &rest parameter
  std::vector<value_t> body;
  environment_ptr env_snapshot;
};

/// Lexically-scoped environment chain.
struct environment {
  std::unordered_map<std::string, value_t> bindings;
  std::shared_ptr<environment> outer;
  /// A frozen scope is shared read-only, possibly across threads (the stdlib scope that every
  /// default environment links to). set() refuses it, and set_existing() never writes into it: a
  /// name bound there is shadowed in the outermost unfrozen scope instead.
  bool frozen = false;

  /// Drops `outer` through detail::drop_owner, so a long scope chain is not freed recursively.
  ~environment();

  /// Look up a name, walking the scope chain.
  value_t *lookup(const std::string &name);
  const value_t *lookup(const std::string &name) const;

  /// Set a binding in this scope. Throws std::logic_error when this scope is frozen.
  void set(const std::string &name, value_t val);

  /// Set a binding in the nearest enclosing unfrozen scope that has it, or in the outermost
  /// unfrozen scope if none has it. Throws std::logic_error when called on a frozen scope.
  void set_existing(const std::string &name, value_t val);

  /// Create a new child scope.
  static environment_ptr extend(environment_ptr parent);
};

// ---- Convenience helpers ---------------------------------------------------

/// Make a list value from a vector of values.
inline value_t make_list(std::vector<value_t> elems) {
  return value_t(std::make_shared<std::vector<value_t>>(std::move(elems)));
}

/// Make an empty list.
inline value_t make_list() { return value_t(std::make_shared<std::vector<value_t>>()); }

/// Make a dict value from key-value pairs.
inline value_t make_dict(std::vector<std::pair<std::string, value_t>> entries = {}) {
  return value_t(
      std::make_shared<std::vector<std::pair<std::string, value_t>>>(std::move(entries)));
}

/// Make a bytes value from a raw byte buffer (SE-3). The std::string is a byte container, NOT text.
inline value_t make_bytes(std::string data) { return value_t(bytes_value{std::move(data)}); }

/// The nil value.
inline const value_t nil_value{};

/// Boolean true value.
inline const value_t true_value{true};

/// Boolean false value.
inline const value_t false_value{false};

/// Convert a value_t to a human-readable string representation. Bounded and cycle-safe:
/// output is length-capped (a shared/cyclic structure is truncated with an ellipsis rather
/// than expanding exponentially), and the walk is iterative, so depth alone neither truncates
/// it nor overflows the stack.
std::string to_string(const value_t &val);

/// Check structural equality of two values. Memoized on pointer-pairs, so a shared or
/// cyclic structure is compared in time proportional to its PHYSICAL size (distinct node
/// pairs), never its exponential logical unfolding. Iterative, so any depth compares; it polls
/// the evaluation deadline and throws std::runtime_error once that is spent.
bool values_equal(const value_t &a, const value_t &b);

// -- time budgets -------------------------------------------------------------
//
// The step caps bound a program's work deterministically; the time budgets -- a process's
// max_time and max_step_time, a scheduler run's max_time, and the evaluation deadline below --
// bound what a step cap cannot see, a slow native step. They are charged with time_budget, so
// that time the thread spends not running (the OS or a hypervisor running something else: a
// loaded machine, a CI runner running tests in parallel) cannot use them up: a program that
// does next to no work is never cut off because its thread waited for a core.

/// Seconds of CPU time the calling thread has used: time it ran, not time it waited to run
/// (though in a virtual machine whose kernel does not account steal time, time the hypervisor
/// takes from the virtual CPU the thread is on still counts). CLOCK_THREAD_CPUTIME_ID where
/// POSIX has it (Linux, macOS), a system call, so it is read sparingly. Elsewhere it is the
/// steady clock -- on Windows too, whose per-thread CPU times advance only at the ~15.6 ms
/// scheduler tick, too coarse for millisecond budgets.
double thread_cpu_seconds();

/// A time budget charged to work the OS may deschedule. Its first half is charged on the
/// steady clock, the rest on the running thread's CPU clock (thread_cpu_seconds) -- but never
/// more of it than the steady clock measured between the work's checks (its steps, or a
/// polling loop's iterations) with each gap between two checks counted up to a sixteenth of
/// it. So a stall between two checks, however long (the OS running other threads, or a
/// hypervisor whose stolen time the CPU clock counts), can use up the first half but at most a
/// sixteenth of the second: the budget runs out only once its owner has itself run for half
/// of it, while an owner that runs straight through runs out on the steady clock's schedule,
/// as with a plain deadline. The CPU clock is read only once the first half is spent, so a
/// budget nowhere near running out costs a steady-clock read per check, as a plain deadline
/// does.
///
/// Charge it over intervals of work, start() to stop(): one (a scheduler run, checked with
/// spent() at each step) or many (a frame's predicate evaluations, each stop() given the steps
/// it took), each interval on one thread.
class time_budget {
public:
  using clock = std::chrono::steady_clock;

  /// A budget of `seconds`; zero or less is spent from the start.
  explicit time_budget(double seconds);

  void start();
  /// End the interval. Since its last check (spent(), or start()) it took `checks` steps,
  /// whose gaps the budget did not see: they count as that many gaps.
  void stop(std::uint64_t checks = 1);
  /// A check: whether the budget has run out, by `now` during an interval (counting as a
  /// check), or as of the last stop() between intervals.
  bool spent(clock::time_point now = clock::now());
  /// Seconds left as of the last stop(): the first half's unspent steady-clock time plus the
  /// second half, then the second half's unspent CPU time; 0 once spent.
  double remaining() const;

private:
  void gap(clock::time_point now, std::uint64_t gaps = 1); // the second half: a check at `now`
  void charge(double cpu_now); // the second half: the CPU time since c0_, capped by the gaps

  double half_;                 // half the budget, seconds (0: spent from the start)
  double wall_ = 0.0;           // the first half: steady-clock seconds finished intervals used
  double cpu_ = 0.0;            // the second half: CPU seconds charged so far
  bool on_cpu_ = false;         // the first half is spent; charging the CPU clock
  bool running_ = false;        // in an interval
  clock::time_point t0_{};      // the interval's start
  clock::time_point last_{};    // the interval's last check
  double c0_ = 0.0;             // the thread's CPU time when cpu_ was last charged
  double gaps_ = 0.0;           // the capped steady-clock gaps between checks since then
  clock::time_point next_ = {}; // during an interval: spent() reads no CPU clock before this
};

// -- cooperative evaluation deadline ----------------------------------------
//
// A per-thread time budget for the current synchronous evaluation. It lets a single
// long-running native step — an uncapped nested evaluator (a defclass method body), a big
// generator/collect/range loop, a deep structural walk — abort AT the time budget instead of
// running to completion, closing the gap that the evaluator's between-steps caps cannot see.
// A nested evaluator INHERITS the deadline automatically (it lives on the thread), and a
// nested guard only ever TIGHTENS it: every armed guard's budget is checked.

/// RAII: arm a time_budget of `seconds` from now for the current thread (no-op if nullopt);
/// the outer ones stay armed. Disarms it on destruction. run() arms one.
class eval_deadline_guard {
public:
  explicit eval_deadline_guard(std::optional<double> seconds);
  ~eval_deadline_guard();
  eval_deadline_guard(const eval_deadline_guard &) = delete;
  eval_deadline_guard &operator=(const eval_deadline_guard &) = delete;

private:
  friend bool eval_deadline_expired();
  std::optional<time_budget> budget_;
  eval_deadline_guard *outer_ = nullptr; // the next armed guard out
};

/// True iff a deadline is armed on this thread and its budget is spent; each call is a check
/// of every armed budget. Cheap (one steady-clock read until a budget is half spent);
/// long-running builtins call this in their loops so they too honour the budget.
bool eval_deadline_expired();

// -- per-call allocation limit ----------------------------------------------
//
// A native call builds its whole result inside one evaluator step, where the per-process
// memory limit (checked between steps, and counting only state-tree writes) cannot see it:
// (collections.range 1000000000) would allocate tens of GB in one call. Every builtin or
// stdlib function whose result can be larger than all of its arguments estimates the result's
// size first and calls eval_alloc_check() before allocating.
//
// The schedulers arm a guard around each step with the tighter of the tree setting
// limits.max_alloc_bytes (state_exec.schedulers.<id>, then state_exec.defaults) and what is
// left of the process's max_memory. With no guard armed (a bare evaluator, a direct native
// call) the limit is kDefaultMaxAllocBytes.

/// The limit when no guard is armed, and the default of limits.max_alloc_bytes: 16 MiB.
inline constexpr std::size_t kDefaultMaxAllocBytes = std::size_t{16} << 20;

/// RAII: bound what a single native call may allocate on this thread while the guard lives
/// (std::nullopt: no bound). A guard nested inside another only tightens it. The schedulers
/// arm one per step.
class eval_alloc_guard {
public:
  explicit eval_alloc_guard(std::optional<std::size_t> max_bytes);
  ~eval_alloc_guard();
  eval_alloc_guard(const eval_alloc_guard &) = delete;
  eval_alloc_guard &operator=(const eval_alloc_guard &) = delete;

private:
  friend std::optional<std::size_t> eval_alloc_limit();
  std::optional<std::size_t> limit_; // the effective limit: this guard's, tightened by outer ones
  eval_alloc_guard *outer_ = nullptr;
};

/// The allocation limit in force on this thread (std::nullopt: unbounded).
std::optional<std::size_t> eval_alloc_limit();

/// Throw std::runtime_error naming `who` when `bytes` is over eval_alloc_limit().
void eval_alloc_check(std::size_t bytes, const char *who);

/// `count * each`, saturating at SIZE_MAX, for size estimates that must not wrap.
inline std::size_t alloc_bytes(std::size_t count, std::size_t each) {
  return each != 0 && count > SIZE_MAX / each ? SIZE_MAX : count * each;
}

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_TYPES_H
