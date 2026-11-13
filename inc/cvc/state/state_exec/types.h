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
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

namespace cvc::state_exec {

// Forward declarations
struct closure;
struct environment;
struct generator;
class call_args;
class exec_context;

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
/// A native function: called with its arguments and the context of the call (call_args). One
/// declared to take std::span<const value_t> is one too: it gets the arguments alone.
using native_fn = std::function<value_tag(call_args)>;

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
/// What a nested value's storage is, so freeing it can take its parts first (release_nested).
enum class owner_kind : std::uint8_t { list, dict, closure, generator, env };

template <class T> constexpr owner_kind owner_kind_of() noexcept {
  if constexpr (std::is_same_v<T, std::vector<value_tag>>)
    return owner_kind::list;
  else if constexpr (std::is_same_v<T, std::vector<std::pair<std::string, value_tag>>>)
    return owner_kind::dict;
  else if constexpr (std::is_same_v<T, closure>)
    return owner_kind::closure;
  else if constexpr (std::is_same_v<T, generator>)
    return owner_kind::generator;
  else
    return owner_kind::env;
}

/// A last owner a release has yet to free, and what it costs as deep-data work: 1, plus a list's
/// or dict's elements.
struct released_owner {
  std::shared_ptr<void> owner;
  owner_kind kind = owner_kind::list;
  std::size_t nodes = 1;
};

/// Free `owner`, the last owner of a nested value's storage (a list, dict, closure, generator or
/// environment), without recursing once per nesting level. Freeing a 100000-deep list through
/// plain shared_ptr destructors takes a native stack frame per level and overflows the stack.
/// Here the release runs a loop over a queue of its own: before it frees something, it moves the
/// parts that have no other owner (a list's or dict's nested values, a closure's body and
/// captured scope, a scope's bindings and enclosing scope) out of it into the queue, so freeing
/// the thing itself frees no nesting, and the stack stays a few frames deep whatever the depth.
/// Nothing per thread: a release is the frame that runs it. (A generator is freed whole: what it
/// holds starts releases of its own, a few frames each.)
void release_nested(released_owner owner) noexcept;

/// Drop `owner` through release_nested when it is the last one; dropping any other owner only
/// decrements a count. (Should another thread drop the last but one meanwhile, this drop frees
/// the storage inline, and its parts start releases of their own: a few frames, still.)
template <class T> void drop_owner(std::shared_ptr<T> &owner) noexcept {
  if (!owner || owner.use_count() != 1)
    return;
  std::size_t nodes = 1;
  if constexpr (std::is_same_v<T, std::vector<value_tag>> ||
                std::is_same_v<T, std::vector<std::pair<std::string, value_tag>>>)
    nodes += owner->size();
  release_nested(released_owner{std::move(owner), owner_kind_of<T>(), nodes});
}

/// Last owners a slicing process step dropped (data.step_nodes > 0) and not freed yet. The
/// evaluator hands it what its step drops where a big value goes (a binding set, a frame popped:
/// data_step_scope::release), and the step frees them a slice at a time: freeing a huge value
/// then counts against the process's budget and spreads over its steps instead of stalling one.
/// (A single owner is freed whole: a flat list of a million numbers is one free.)
class release_backlog {
public:
  release_backlog() = default;
  ~release_backlog(); // frees what is left, iteratively like any release
  release_backlog(const release_backlog &) = delete;
  release_backlog &operator=(const release_backlog &) = delete;

  bool empty() const { return owners_.empty(); }

  /// Take `v`'s storage when `v` holds its last owner (a list, dict, closure or generator), to
  /// free later; anything else is dropped here.
  void add(value_tag &&v) noexcept;
  /// Take `owner` when it is the last one, to free later; any other owner is dropped here.
  template <class T> void add(std::shared_ptr<T> owner) noexcept {
    if (!owner || owner.use_count() != 1)
      return;
    std::size_t nodes = 1;
    if constexpr (std::is_same_v<T, std::vector<value_tag>> ||
                  std::is_same_v<T, std::vector<std::pair<std::string, value_tag>>>)
      nodes += owner->size();
    push(released_owner{std::move(owner), owner_kind_of<T>(), nodes});
  }

  /// Free owners until their nodes reach `budget` (0: until none are left). The parts of what it
  /// frees that have no other owner join the backlog. Returns the nodes freed.
  std::size_t drain(std::size_t budget);

private:
  void push(released_owner owner) noexcept;
  std::vector<released_owner> owners_;
};
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

/// A native function's arguments, and the context it is called in (exec_context): what the
/// evaluator, a scheduler's step or a data job running it has in force, which the natives that
/// need it (a deadline, an allocation limit, a data step, a cycle registry, a random stream)
/// pass on to what they call. Null when no state_exec work called it (a host calling a native
/// directly): then nothing is in force. It converts to and from std::span<const value_t>, so a
/// native function declared to take the span gets the arguments alone, and a span calls a native
/// with no context.
class call_args {
public:
  call_args(std::span<const value_t> args, exec_context *ctx = nullptr) noexcept
      : args_(args), ctx_(ctx) {}
  call_args(const std::vector<value_t> &args, exec_context *ctx = nullptr) noexcept
      : args_(args), ctx_(ctx) {}

  operator std::span<const value_t>() const noexcept { return args_; }
  std::span<const value_t> span() const noexcept { return args_; }
  /// The context of the call, or null.
  exec_context *context() const noexcept { return ctx_; }

  std::size_t size() const noexcept { return args_.size(); }
  bool empty() const noexcept { return args_.empty(); }
  const value_t &operator[](std::size_t i) const noexcept { return args_[i]; }
  const value_t *data() const noexcept { return args_.data(); }
  auto begin() const noexcept { return args_.begin(); }
  auto end() const noexcept { return args_.end(); }
  const value_t &front() const noexcept { return args_.front(); }
  const value_t &back() const noexcept { return args_.back(); }
  std::span<const value_t> subspan(std::size_t offset,
                                   std::size_t count = std::dynamic_extent) const noexcept {
    return args_.subspan(offset, count);
  }

private:
  std::span<const value_t> args_;
  exec_context *ctx_;
};

/// Closure: a lambda body + captured environment.
struct closure {
  std::vector<symbol> params;
  bool variadic = false; // &rest parameter
  std::vector<value_t> body;
  environment_ptr env_snapshot;
  /// `body` as one expression when it has several forms, the list (begin body...), so that each
  /// call needn't build it again: made when the stackless evaluator makes the closure, and shared
  /// by the closures one lambda form makes. Only a cache: null when nobody made it (a one-form
  /// body, a closure built elsewhere), and ignored once it no longer matches `body`. Not
  /// serialized, printed or compared.
  list_ptr body_expr;
};

namespace detail {
/// A serial for a new scope, from the one count every scope's serial is drawn from (a scope cache
/// takes a run of them at a time, exec_context::scopes), so no other scope has it until 2^32 more
/// have been made.
std::uint32_t new_scope_serial() noexcept;

/// Whether the bucket formula environment lookups use (types.cpp: a mask for a power-of-two count,
/// else a remainder) matches this standard library's unordered_map::bucket(). The lookups take it
/// as known for MSVC's, libstdc++'s and libc++'s maps (and use find() with any other); a test
/// checks it.
bool bucket_formula_holds();

/// Which scope an environment is, to the evaluators' call records (eval_frame), which can't go by
/// its address alone: a scope made where a dropped one was (environment::extend() hands out the
/// block the last dropped scope left) has the same address. A scope gets a new serial when it is
/// made, copied or assigned, and when extend() makes it in a dropped scope's block.
struct scope_serial {
  std::uint32_t value = new_scope_serial();

  scope_serial() = default;
  scope_serial(const scope_serial &) noexcept : value(new_scope_serial()) {}
  scope_serial &operator=(const scope_serial &) noexcept {
    value = new_scope_serial();
    return *this;
  }
};

/// Whether a scope is in a block extend() made (environment::from_extend). Never copied: a copy
/// of a scope is a scope of its own, wherever it is.
struct block_mark {
  bool value = false;

  block_mark() = default;
  block_mark(const block_mark &) noexcept {}
  block_mark &operator=(const block_mark &) noexcept { return *this; }
};
} // namespace detail

/// Lexically-scoped environment chain.
struct environment {
  std::unordered_map<std::string, value_t> bindings;
  std::shared_ptr<environment> outer;
  /// A frozen scope is shared read-only, possibly across threads (the stdlib scope that every
  /// default environment links to). set() refuses it, and set_existing() never writes into it: a
  /// name bound there is shadowed in the outermost unfrozen scope instead.
  bool frozen = false;
  /// extend() made it in a block of its own, which a scope cache can keep (detail::release_scope).
  detail::block_mark from_extend;
  /// Which scope this is (detail::scope_serial), for the evaluators: not part of what the scope
  /// means, so not serialized or compared. (It fits in the padding after `frozen`: a scope is no
  /// larger for it.)
  detail::scope_serial serial;

  /// Drops `outer` through detail::drop_owner, so a long scope chain is not freed recursively.
  ~environment();

  /// Look up a name, walking the scope chain.
  value_t *lookup(const std::string &name);
  const value_t *lookup(const std::string &name) const;

  /// Set a binding in this scope. Throws std::logic_error when this scope is frozen.
  void set(const std::string &name, value_t val);
  /// The same as the evaluators set it: a new name's binding node from `ctx`'s scope cache, and
  /// the value it replaces handed to `ctx`'s data step (detail::data_step_scope::release).
  void set(const std::string &name, value_t val, exec_context &ctx);

  /// Set a binding in the nearest enclosing unfrozen scope that has it, or in the outermost
  /// unfrozen scope if none has it. Throws std::logic_error when called on a frozen scope.
  void set_existing(const std::string &name, value_t val);
  /// The same as the evaluators set it (see set(name, val, ctx)).
  void set_existing(const std::string &name, value_t val, exec_context &ctx);

  /// Create a new child scope, in a block of its own.
  static environment_ptr extend(environment_ptr parent);
  /// The same as the evaluators make their call scopes: in a block `ctx`'s scope cache kept, if
  /// it has one, with a serial from its run.
  static environment_ptr extend(environment_ptr parent, exec_context &ctx);
};

class eval_deadline_guard;
class eval_alloc_guard;
struct cycle_registry; // heap.h
struct random_stream;  // stdlib.h
class exec_group;      // exec_group.h
namespace detail {
class data_step_scope; // data_job.h
struct await_slot;     // stackless_evaluator.cpp
struct scope_cache;    // types.cpp
struct frame_vectors;  // stackless_evaluator.cpp

/// Drop `scope`, a scope the evaluator running in `ctx` drops (a frame's, popped): when this is
/// its last owner and extend() made it, its block and binding nodes go to `ctx`'s scope cache for
/// the next extend(). Any other scope is dropped as usual.
void release_scope(environment_ptr &scope, exec_context &ctx) noexcept;
} // namespace detail

/// What state_exec work in progress has in force, handed down explicitly from whoever runs it (a
/// scheduler's step, an evaluator's run, an async kernel, a group's pump) to what it calls (the
/// evaluators' steps, data jobs, and native functions, through call_args::context()). Never
/// kept per thread: two apps, groups or evaluators running on one thread, nested or in turn, each
/// see only their own. A context is used by one run at a time; it moves between threads with its
/// owner (a group pumped by any worker of a pool).
///
/// Its fields are set by whoever arms them, and restored by the same (the guards and scopes
/// below and in data_job.h, heap.h, stdlib.h and stackless_evaluator.h take the context).
class exec_context {
public:
  exec_context();
  ~exec_context();
  exec_context(const exec_context &) = delete;
  exec_context &operator=(const exec_context &) = delete;

  eval_deadline_guard *deadlines = nullptr; ///< the innermost deadline armed (eval_deadline_guard)
  eval_alloc_guard *alloc = nullptr;        ///< the innermost allocation guard (eval_alloc_guard)
  bool in_async_kernel = false;             ///< an async function's kernel runs in it
  /// Where captured scopes and changed lists and dicts are tracked (heap.h): its owner's, or a
  /// group's. Null: nowhere.
  cycle_registry *registry = nullptr;
  random_stream *random = nullptr;         ///< math.random's stream (null: its registry's own)
  exec_group *group = nullptr;             ///< the group a pump of it runs (exec_group::current)
  detail::data_step_scope *data = nullptr; ///< the stackless evaluator step's data side
  std::size_t native_depth = 0;            ///< levels of native nesting (detail::native_nesting)
  detail::await_slot *await = nullptr;     ///< the native call that may park (can_await)
  int step_depth = 0;                      ///< stackless evaluator steps nested in it
  bool park_forbidden = false;             ///< parking is forbidden (park_forbidden_scope)

  /// The lowest address of the native stack of the thread running it now, found on that thread
  /// the first time it is asked there (0: unknown, so no limit). Not read from the OS on every
  /// check: only when the context runs on a thread it didn't before.
  std::uintptr_t stack_low();

  /// Its scope cache: blocks and binding nodes of the scopes environment::extend(parent, ctx)
  /// made that were dropped, and a run of scope serials. Made at the first use, bounded, and freed
  /// with the context.
  detail::scope_cache &scopes();
  /// The stackless evaluator's frame vectors (stackless_evaluator.cpp), made at the first use.
  detail::frame_vectors &vectors();

private:
  std::thread::id stack_thread_;
  std::uintptr_t stack_low_ = 0;
  std::unique_ptr<detail::scope_cache, void (*)(detail::scope_cache *)> scopes_;
  std::unique_ptr<detail::frame_vectors, void (*)(detail::frame_vectors *)> vectors_;
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

/// The nil value: a new one each call (value_t isn't a literal type, so it can't be a constant,
/// and a shared object would be process-wide state).
inline value_t nil_value() noexcept { return value_t(); }

/// Boolean true value.
inline value_t true_value() noexcept { return value_t(true); }

/// Boolean false value.
inline value_t false_value() noexcept { return value_t(false); }

/// Convert a value_t to a human-readable string representation. Bounded and cycle-safe:
/// output is length-capped (a shared/cyclic structure is truncated with an ellipsis rather
/// than expanding exponentially), and the walk is iterative, so depth alone neither truncates
/// it nor overflows the stack.
std::string to_string(const value_t &val);

/// The shortest text that reads back as `d`: std::to_chars's form where the library has it for
/// floating point, else detail::format_number_shortest_fallback's identical text. How to_string
/// prints a float, and how state-set and state-incr store one: 1234567.5 as "1234567.5", 0.1 + 0.2
/// as "0.30000000000000004", a whole float as an integer's digits (2.0 as "2").
std::string format_number_shortest(double d);

/// Check structural equality of two values. Memoized on pointer-pairs, so a shared or
/// cyclic structure is compared in time proportional to its PHYSICAL size (distinct node
/// pairs), never its exponential logical unfolding. Iterative, so any depth compares; it polls
/// the evaluation deadline of `ctx` (null: none) and throws evaluation_timeout once that is spent.
bool values_equal(const value_t &a, const value_t &b, exec_context *ctx = nullptr);

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
/// takes from the virtual CPU the thread is on still counts). A system call, so it is read
/// sparingly:
/// - Linux and macOS: CLOCK_THREAD_CPUTIME_ID.
/// - Windows: QueryThreadCycleTime, the thread's cycles on the time-stamp counter, converted at
///   the counter's rate, measured once (the first call takes about 5 ms). Where the counter's
///   rate isn't constant (no invariant TSC) or there is none (ARM64), GetThreadTimes, in 100 ns
///   units that advance at the ~15.6 ms scheduler tick.
/// - Elsewhere: the steady clock.
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
  /// start() at `now`, a steady_clock reading the caller already has.
  void start(clock::time_point now);
  /// End the interval. Since its last check (spent(), or start()) it took `checks` steps,
  /// whose gaps the budget did not see: they count as that many gaps.
  void stop(std::uint64_t checks = 1);
  /// A check: whether the budget has run out, by `now` during an interval (counting as a
  /// check), or as of the last stop() between intervals.
  bool spent(clock::time_point now = clock::now());
  /// Seconds left as of the last stop(): the first half's unspent steady-clock time plus the
  /// second half, then the second half's unspent CPU time; 0 once spent.
  double remaining() const;
  /// Between intervals (after stop()): charge `seconds` its owner spent waiting for work, with
  /// nothing to run, as time it ran: on the steady clock in the first half, the rest as CPU time
  /// in the second. An owner that would otherwise have polled through the wait (a scheduler
  /// run whose processes all sleep) spends its budget as the polling would have.
  void charge_wait(double seconds);

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
// A time budget for the evaluation running in a context (exec_context). It lets a single
// long-running native step — an uncapped nested evaluator (a defclass method body), a big
// generator/collect/range loop, a deep structural walk — abort AT the time budget instead of
// running to completion, closing the gap that the evaluator's between-steps caps cannot see.
// A nested evaluator INHERITS the deadline (it runs in the same context), and a nested guard only
// ever TIGHTENS it: every armed guard's budget is checked.

/// RAII: arm a time_budget of `seconds` from now in `ctx` (no-op if nullopt); the outer ones stay
/// armed. Disarms it on destruction. run() arms one.
class eval_deadline_guard {
public:
  eval_deadline_guard(exec_context &ctx, std::optional<double> seconds);
  /// The same, armed from `now`, a steady_clock reading the caller already has (a scheduler's
  /// step start).
  eval_deadline_guard(exec_context &ctx, std::optional<double> seconds,
                      std::chrono::steady_clock::time_point now);
  ~eval_deadline_guard();
  eval_deadline_guard(const eval_deadline_guard &) = delete;
  eval_deadline_guard &operator=(const eval_deadline_guard &) = delete;
  /// Whether this guard's own budget is spent (a check, like eval_deadline_expired()). Tells a
  /// scheduler's step deadline from one armed outside it, both of which throw evaluation_timeout.
  bool expired();

private:
  friend bool eval_deadline_expired(exec_context *ctx);
  exec_context *ctx_ = nullptr;
  std::optional<time_budget> budget_;
  eval_deadline_guard *outer_ = nullptr; // the next armed guard out
};

/// Exception thrown when evaluation is interrupted.
class evaluation_interrupted : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// Exception thrown when evaluation times out: by the evaluator between steps, or by a long
/// loop inside one native step (eval_deadline_check). The schedulers kill a process whose step
/// throws it with time_limit_exceeded, as when the time runs out between steps.
class evaluation_timeout : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

/// True iff a deadline is armed in `ctx` (null: none) and its budget is spent; each call is a
/// check of every armed budget. Cheap (one steady-clock read until a budget is half spent).
bool eval_deadline_expired(exec_context *ctx);

/// Throws evaluation_timeout ("<what>: evaluation exceeded timeout") when
/// eval_deadline_expired(ctx). Long-running builtins and data jobs call this in their loops, so
/// the budget cuts them the same way it cuts a program.
void eval_deadline_check(exec_context *ctx, const char *what);

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
// left of the process's max_memory. With no guard armed in the call's context (a bare evaluator,
// a direct native call) the limit is kDefaultMaxAllocBytes.

/// The limit when no guard is armed, and the default of limits.max_alloc_bytes: 16 MiB.
inline constexpr std::size_t kDefaultMaxAllocBytes = std::size_t{16} << 20;

/// RAII: bound what a single native call may allocate in `ctx` while the guard lives
/// (std::nullopt: no bound). A guard nested inside another only tightens it. The schedulers
/// arm one per step.
class eval_alloc_guard {
public:
  eval_alloc_guard(exec_context &ctx, std::optional<std::size_t> max_bytes);
  ~eval_alloc_guard();
  eval_alloc_guard(const eval_alloc_guard &) = delete;
  eval_alloc_guard &operator=(const eval_alloc_guard &) = delete;

  /// Bytes natives allocated while this guard was the innermost one: every eval_alloc_check()
  /// that passed, and every eval_alloc_note(). A scheduler adds them to the process's heap
  /// accounting (heap.h), which uses them to decide when to re-measure.
  std::size_t allocated() const { return allocated_; }

private:
  friend std::optional<std::size_t> eval_alloc_limit(const exec_context *);
  friend void eval_alloc_check(exec_context *, std::size_t, const char *);
  friend void eval_alloc_note(exec_context *, std::size_t);
  exec_context *ctx_ = nullptr;
  std::optional<std::size_t> limit_; // the effective limit: this guard's, tightened by outer ones
  eval_alloc_guard *outer_ = nullptr;
  std::size_t allocated_ = 0;
};

/// The allocation limit in force in `ctx` (std::nullopt: unbounded; null `ctx`, or none armed:
/// kDefaultMaxAllocBytes).
std::optional<std::size_t> eval_alloc_limit(const exec_context *ctx);

/// Throw std::runtime_error naming `who` when `bytes` is over eval_alloc_limit(ctx); otherwise
/// report the bytes as allocated (eval_alloc_guard::allocated()).
void eval_alloc_check(exec_context *ctx, std::size_t bytes, const char *who);

/// Report `bytes` a native allocated without a limit check (a copy no bigger than an argument,
/// such as cons or slice), for heap accounting only.
void eval_alloc_note(exec_context *ctx, std::size_t bytes);

// -- async kernels (docs/STATE_EXEC_EXCEPTIONS.md 3.7) ------------------------
//
// An async function's kernel runs off the scheduler's timeline, so it must not touch the
// evaluator, a scheduler or a DSL value the program holds. The host marks the context an inline
// kernel runs in (the step's), and the evaluator refuses to step in a context so marked; it marks
// its scheduler while a kernel of it runs on a thread (scheduler_base::kernel_thread_scope: its
// executor's, its fan-out chunks', the step's for an inline kernel), and the scheduler's entry
// points refuse a call from such a thread, throwing std::logic_error. (The checks are always on:
// NDEBUG can't tell a debug build here, the mesher defines it for every build type.) A kernel may
// run an evaluator of its own: that touches nothing the program holds. The state tree isn't
// checked: Ariadne's (fetch "state://...") resolves on its kernel, by design, pinning the nodes it
// reads.

/// Whether an async function's kernel runs in `ctx` now (null: no).
bool in_async_kernel(const exec_context *ctx);

namespace detail {
/// RAII: mark `ctx` as running a kernel (`running`) or not, while alive.
class async_kernel_scope {
public:
  async_kernel_scope(exec_context &ctx, bool running);
  ~async_kernel_scope();
  async_kernel_scope(const async_kernel_scope &) = delete;
  async_kernel_scope &operator=(const async_kernel_scope &) = delete;

private:
  exec_context &ctx_;
  bool outer_;
};

/// Throw std::logic_error naming `what` when a kernel runs in `ctx`.
void check_not_in_async_kernel(const exec_context &ctx, const char *what);
} // namespace detail

/// `count * each`, saturating at SIZE_MAX, for size estimates that must not wrap.
inline std::size_t alloc_bytes(std::size_t count, std::size_t each) {
  return each != 0 && count > SIZE_MAX / each ? SIZE_MAX : count * each;
}

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_TYPES_H
