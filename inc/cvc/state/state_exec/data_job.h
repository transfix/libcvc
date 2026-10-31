/**
 * @file data_job.h
 * @brief Deep-data work (comparing, printing, copying, expanding nested values) in slices.
 *
 * A walk over a nested value costs one unit per node it visits. A data_job keeps its walk on the
 * heap, so it can stop after any number of nodes and go on later. A scheduler runs a process's
 * deep-data work a slice at a time (data.step_nodes on the state tree), as it reads a script a
 * slice at a time (parser.step_tokens): the work counts against the process's max_steps and
 * max_time, and other processes run between the slices.
 */
#ifndef CVC_STATE_EXEC_DATA_JOB_H
#define CVC_STATE_EXEC_DATA_JOB_H

#include <cstddef>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

struct evaluation_stats;

/// Byte work (scanning, copying) counts one node per this many bytes, so a slice of a string
/// function costs about what a slice of a walk does.
inline constexpr std::size_t kDataNodeBytes = 256;

/// A walk over a nested value, done a slice at a time. A job holds what it walks (the shared
/// lists and dicts, by owning pointer) and reads them by index, checking bounds at every node,
/// so a value changed between two slices (by another process) cannot leave it dangling; the
/// result then reflects each part as it was when visited.
class data_job {
public:
  virtual ~data_job() = default;

  /// Visit at most `nodes` more nodes; 0 visits all that remain. True once the walk is done.
  /// A job that applies a function (collections.map and friends) also stops when it needs a
  /// call made: step() returns false with a call pending (take_call()). Whoever runs the job
  /// makes the call, passes its value to call_returned() and steps the job again.
  virtual bool step(std::size_t nodes) = 0;

  /// The walk's result, once step() has returned true. Call it once.
  virtual value_t result() = 0;

  /// Nodes visited so far.
  std::size_t nodes() const { return nodes_; }

  /// A function application a job waits for. `fn` and `args` belong to the job and stay valid
  /// until it is stepped again.
  struct call_request {
    const value_t *fn;
    std::span<const value_t> args;
    const char *who; // the stdlib function making the call, for errors
  };

  /// The call the job stopped for, if any (taking it clears it).
  std::optional<call_request> take_call() { return std::exchange(call_, std::nullopt); }
  const call_request *pending_call() const { return call_ ? &*call_ : nullptr; }

  /// The value of the call the job stopped for.
  virtual void call_returned(value_t value) { (void)value; }

  /// Called once the job outlives the native call that made it: copy what it only borrowed
  /// from that call's arguments.
  virtual void own_inputs() {}

  /// True for a job whose result is the value of the one call it makes, unchanged (a call
  /// call_function() made a job). When that call leaves work of its own, the evaluator lets the
  /// work take the job's place, so a chain of such calls runs in constant space.
  virtual bool forwards_call() const { return false; }

  /// For measuring the heap of a process whose work waits on the job (heap.h), which happens
  /// between its slices: `visit` is called with each value the job owns a copy of (what it was
  /// given, what it has built so far). A job that only borrows its caller's arguments, or whose
  /// results are held to the allocation limit as they grow, need not report them.
  virtual void held_values(const std::function<void(const value_t &)> &visit) const { (void)visit; }

  /// The heap bytes the job takes itself, its object and its own buffers, besides what the
  /// values it reports hold. A job left to run after its call reports these as allocated
  /// (eval_alloc_note), so a heap measurement comes due as such jobs pile up.
  virtual std::size_t held_bytes() const { return 0; }

protected:
  std::size_t nodes_ = 0;
  std::optional<call_request> call_;
};

/// Whether walking `v` visits any nodes: it is a non-empty list or dict.
bool is_compound(const value_t &v);

/// values_equal(a, b), as a bool value.
std::unique_ptr<data_job> make_equal_job(value_t a, value_t b);

/// to_string(v), as a string value.
std::unique_ptr<data_job> make_print_job(value_t v);

/// `parts` printed one after another with `sep` between them, as a string value: a string as its
/// raw text, anything else as to_string() prints it. The same string may be passed many times, so
/// the result can outgrow every part: each append is checked against the allocation limit
/// (eval_alloc_check), naming `who` (a string literal: the builtin) when it is over.
std::unique_ptr<data_job> make_concat_job(std::vector<value_t> parts, std::string sep,
                                          const char *who);

/// A private copy of v's lists and dicts, keeping their sharing (a node reached twice is copied
/// once; a cycle is copied as a cycle). Anything else is shared, not copied.
std::unique_ptr<data_job> make_copy_job(value_t v);

/// A macro template with its parameters replaced: each symbol named in `subst` becomes its
/// value; lists are copied, everything else is kept.
std::unique_ptr<data_job> make_substitute_job(value_t tmpl,
                                              std::unordered_map<std::string, value_t> subst);

/// Run a job to completion and return its result. The calls it makes are made here: a native
/// function's own deep-data work then runs here too, and a DSL function (a closure) throws,
/// since only an evaluator can run one.
value_t run_to_completion(data_job &job);

/// Deep-data settings ("Deep data on the executor" in docs/STATE_EXEC_GRAMMAR.md). Hosts keep them
/// on the state tree under `data.*`; data_options_from_state() reads them from there.
struct data_options {
  /// How many nodes of deep-data work a scheduler step may do. 0 does all of it in the step that
  /// needs it. N > 0 does up to N there; what is left runs N per step in the steps after it
  /// (the process waits, and those steps count against its max_steps and max_time). Freeing
  /// counts too: up to N owners a step dropped are freed at its end, the rest in steps after.
  /// State-tree key: `data.step_nodes`.
  std::size_t step_nodes = 0;
};

/// Resolve data settings from a state tree, per key, like parse_options_from_state():
/// `state_exec.schedulers.<scheduler_id>.data.<key>`, then `state_exec.defaults.data.<key>`, then
/// `base`. A value that is missing, empty or malformed is skipped.
data_options data_options_from_state(cvc::state &root, const std::string &scheduler_id = {},
                                     const data_options &base = {});

/// Do a native function's deep-data work. Inside a stackless evaluator step, called from the
/// native function that step is applying, `job` does one slice here (all of it when the step
/// does not slice, data.step_nodes = 0); if that finishes it, `then(result)` is returned. If not
/// (the slice ran out, or the job needs a function called), the job and `then` pass to the
/// evaluator: it runs a slice per step, makes the job's calls (a DSL function as ordinary
/// frames of the same process) and completes the call with then's value once the job is done;
/// what this returns is then ignored, so the native function should return it at once.
/// A native function the job calls is called here only from the step's own native call
/// (data_step_scope::calls_here); deeper, the evaluator calls it. Anywhere else the job runs to
/// completion here (see run_to_completion). Without `then`, the job's result is the call's.
value_t run_data_job(std::unique_ptr<data_job> job, std::function<value_t(value_t)> then = {});

/// Apply `fn`, a native function or a DSL closure, to `args` from inside a native function.
/// Inside a stackless evaluator step the call is the evaluator's, as a one-call job
/// (run_data_job): a closure's body becomes the process's own frames, and a native function is
/// called by the evaluator at its own level, in a data step, unless this is the step's own native
/// call, which calls it here (data_step_scope::calls_here). The native function should return
/// what this returns at once. Anywhere else a native function is called here, one level of
/// native nesting (detail::native_nesting), and a closure throws. `who` names the caller in
/// errors.
value_t call_function(const value_t &fn, std::span<const value_t> args, const char *who);

/// As call_function, with `then` applied to the call's value: for a native function that has
/// more to do after the call, such as a class constructor returning the new instance once init
/// has run. The call is a one-call job either way (run_data_job), so `then` runs after the
/// function finishes, however the evaluator ran it. `who` must outlive the call (a literal).
value_t call_function(const value_t &fn, std::span<const value_t> args, const char *who,
                      std::function<value_t(value_t)> then);

/// How native calls may nest on one thread's native stack. Inside an evaluator step they barely
/// do: when a native function (apply, send, a class constructor running init, an inherited init
/// running its superclass's, a job) calls a native function, the call is made here only from the
/// native function the step applies, and any call deeper than that is the evaluator's, made in a
/// data step (data_step_scope::calls_here). So a self-containing argument list, a class whose init
/// is its own constructor, or a long chain of inits is a sequence of steps, stopped by max_steps
/// and max_time like a loop. What still nests is a level: that one call, a generator stepping its
/// own evaluator, and a native function calling another from C++ with no evaluator step running
/// (or one that cannot wait for it, see run_to_completion). A level is refused (it throws) past
/// kMaxNativeNesting levels, or when less than kNativeStackReserve bytes of the thread's native
/// stack are left (where the platform says where its stack ends). Levels cost from about 0.5 KiB
/// (apply) to a few KiB (a generator's evaluator) in a release build.
constexpr std::size_t kMaxNativeNesting = 256;

/// The native stack a level leaves for the error it may throw. MSVC runs a catch block before
/// unwinding the frames above it, so on Windows whatever catches the error (a scheduler killing
/// the process, say) runs at the depth it was thrown from and needs this room: a throw with 32 KiB
/// left crashed in a test, 64 KiB did not, and this doubles that for heavier catch blocks.
/// Elsewhere a throw unwinds first and needs little.
#ifdef _WIN32
constexpr std::size_t kNativeStackReserve = 128 * 1024;
#else
constexpr std::size_t kNativeStackReserve = 32 * 1024;
#endif

/// Advance `gen` as a job: each step of a DSL generator's own evaluator (or each value of a
/// native one) is a node, so a generator body runs a slice per step of the process advancing
/// it, counted against its budget, instead of to its next yield inside one step. The result is
/// a one-element list holding the value yielded, or an empty list once the generator is
/// exhausted. With `collect`, the job runs the generator to exhaustion and its result is the
/// list of everything it yielded.
std::unique_ptr<data_job> make_generator_job(generator_ptr gen, bool collect = false);

/// How deep json-parse, json-encode and state-set-tree let lists and dicts nest. JSON comes from
/// outside, so a document can't make a value deeper than this.
inline constexpr std::size_t kJsonMaxDepth = 512;

/// The most spaces json-encode indents a level by, as JSON.stringify.
inline constexpr std::size_t kJsonMaxIndent = 10;

/// The value of the JSON text `text`, as json-parse reads it ("JSON" in
/// docs/STATE_EXEC_DEVELOPER_GUIDE.md §2): an object is a dict in document order, an array a
/// list, null nil. A number without a fraction or exponent is an int when it fits in 64 bits;
/// any other is a float. A malformed text is a ValueError with its `line`, `col` and `offset`.
/// A node per value, and per kDataNodeBytes of text.
std::unique_ptr<data_job> make_json_parse_job(std::string text);

/// `v` as JSON text, as json-encode writes it: compact, or with each element on its own line,
/// indented `indent` spaces a level (at most kJsonMaxIndent). Dicts keep their order. Anything
/// JSON can't hold is an error: a TypeError for a type JSON lacks, a ValueError for a non-finite
/// float, a string that isn't UTF-8 or a list or dict that contains itself.
std::unique_ptr<data_job> make_json_encode_job(value_t v, std::size_t indent = 0);

/// Count `nodes` units of deep-data work done in the current evaluator step (a walk run to
/// completion counts itself; the release backlog counts what it frees).
void count_data_nodes(std::size_t nodes);

namespace detail {

/// RAII: one level of native nesting on this thread (see kNativeStackReserve). Throws, naming
/// `who`, when the stack is too close to its end for another.
class native_nesting {
public:
  explicit native_nesting(const char *who);
  ~native_nesting();
  native_nesting(const native_nesting &) = delete;
  native_nesting &operator=(const native_nesting &) = delete;

  /// Levels this thread is nested now.
  static std::size_t depth();
};

/// A native call's deep-data job left to run after the call returned (see run_data_job).
struct deferred_data_job {
  std::unique_ptr<data_job> job;
  std::function<value_t(value_t)> then;
  /// Set when the job waits on work a native function it called left: that runs first, and
  /// its result is the call's value.
  std::unique_ptr<deferred_data_job> waits_on;
};

/// The data side of one stackless evaluator step, armed for the step's duration (nested evaluator
/// steps arm their own and restore this one). It counts the step's deep-data nodes into `stats`
/// and, when the step slices, routes the owners it drops into `backlog` (see release_backlog).
class data_step_scope {
public:
  data_step_scope(std::size_t step_nodes, evaluation_stats &stats, release_backlog *backlog);
  ~data_step_scope();
  data_step_scope(const data_step_scope &) = delete;
  data_step_scope &operator=(const data_step_scope &) = delete;

  /// The slice: how many nodes the step may visit (0 = no slicing).
  std::size_t step_nodes() const { return step_nodes_; }
  /// Nodes counted in this step so far.
  std::size_t nodes() const { return nodes_; }
  /// The stats of the state being stepped.
  evaluation_stats &stats() const { return stats_; }
  /// What is left of the slice (never 0 while slicing, so a job always advances).
  std::size_t remaining() const;

  /// Let run_data_job() defer a job while `allowed` (around the native call being applied).
  void allow_defer(bool allowed) { may_defer_ = allowed; }
  bool defer_allowed() const { return may_defer_; }
  /// The job a native call deferred, if any.
  std::unique_ptr<deferred_data_job> take_deferred() { return std::move(deferred_); }

  /// Whether a native function that the native call being applied wants called (call_function,
  /// a job's call) is called right here, on the native stack: only from that native call itself,
  /// one level down, and while the slice has room. A call from deeper goes to the evaluator,
  /// which makes it at its own level, so native calls nest at most one level in a step.
  bool calls_here() const;

  /// Free backlog owners with what is left of the slice (all of them when not slicing).
  void drain_backlog();

  /// The scope armed on this thread, if any.
  static data_step_scope *current();

private:
  friend value_t cvc::state_exec::run_data_job(std::unique_ptr<data_job>,
                                               std::function<value_t(value_t)>);
  friend value_t cvc::state_exec::call_function(const value_t &, std::span<const value_t>,
                                                const char *);
  friend void cvc::state_exec::count_data_nodes(std::size_t);

  std::size_t step_nodes_;
  evaluation_stats &stats_;
  release_backlog *backlog_;
  std::size_t native_depth_; // native_nesting::depth() when the step began
  std::size_t nodes_ = 0;
  bool may_defer_ = false;
  std::unique_ptr<deferred_data_job> deferred_;
  data_step_scope *outer_;
  backlog_scope routing_;
};

} // namespace detail

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_DATA_JOB_H
