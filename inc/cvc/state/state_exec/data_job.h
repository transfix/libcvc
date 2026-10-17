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
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
}

namespace cvc::state_exec {

struct evaluation_stats;

/// A walk over a nested value, done a slice at a time. A job holds what it walks (the shared
/// lists and dicts, by owning pointer) and reads them by index, checking bounds at every node,
/// so a value changed between two slices (by another process) cannot leave it dangling; the
/// result then reflects each part as it was when visited.
class data_job {
public:
  virtual ~data_job() = default;

  /// Visit at most `nodes` more nodes; 0 visits all that remain. True once the walk is done.
  virtual bool step(std::size_t nodes) = 0;

  /// The walk's result, once step() has returned true. Call it once.
  virtual value_t result() = 0;

  /// Nodes visited so far.
  std::size_t nodes() const { return nodes_; }

protected:
  std::size_t nodes_ = 0;
};

/// Whether walking `v` visits any nodes: it is a non-empty list or dict.
bool is_compound(const value_t &v);

/// values_equal(a, b), as a bool value.
std::unique_ptr<data_job> make_equal_job(value_t a, value_t b);

/// to_string(v), as a string value.
std::unique_ptr<data_job> make_print_job(value_t v);

/// `parts` printed one after another with `sep` between them, as a string value: a string as its
/// raw text, anything else as to_string() prints it.
std::unique_ptr<data_job> make_concat_job(std::vector<value_t> parts, std::string sep);

/// A private copy of v's lists and dicts, keeping their sharing (a node reached twice is copied
/// once; a cycle is copied as a cycle). Anything else is shared, not copied.
std::unique_ptr<data_job> make_copy_job(value_t v);

/// A macro template with its parameters replaced: each symbol named in `subst` becomes its
/// value; lists are copied, everything else is kept.
std::unique_ptr<data_job> make_substitute_job(value_t tmpl,
                                              std::unordered_map<std::string, value_t> subst);

/// Run a job to completion and return its result.
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

/// Do a native function's deep-data work. Inside a stackless evaluator step that slices
/// (data.step_nodes > 0), called from the native function that step is applying, `job` does one
/// slice here; if that finishes it, `then(result)` is returned. If not, the job and `then` pass
/// to the evaluator, which runs a slice per step and completes the call with then's value once
/// the job is done; what this returns is then ignored, so the native function should return it
/// at once. Anywhere else the job runs to completion here. Without `then`, the job's result is
/// the call's.
value_t run_data_job(std::unique_ptr<data_job> job, std::function<value_t(value_t)> then = {});

/// Count `nodes` units of deep-data work done in the current evaluator step (a walk run to
/// completion counts itself; the release backlog counts what it frees).
void count_data_nodes(std::size_t nodes);

namespace detail {

/// A native call's deep-data job left to run after the call returned (see run_data_job).
struct deferred_data_job {
  std::unique_ptr<data_job> job;
  std::function<value_t(value_t)> then;
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
  /// What is left of the slice (never 0 while slicing, so a job always advances).
  std::size_t remaining() const;

  /// Let run_data_job() defer a job while `allowed` (around the native call being applied).
  void allow_defer(bool allowed) { may_defer_ = allowed; }
  /// The job a native call deferred, if any.
  std::unique_ptr<deferred_data_job> take_deferred() { return std::move(deferred_); }

  /// Free backlog owners with what is left of the slice (all of them when not slicing).
  void drain_backlog();

  /// The scope armed on this thread, if any.
  static data_step_scope *current();

private:
  friend value_t cvc::state_exec::run_data_job(std::unique_ptr<data_job>,
                                               std::function<value_t(value_t)>);
  friend void cvc::state_exec::count_data_nodes(std::size_t);

  std::size_t step_nodes_;
  evaluation_stats &stats_;
  release_backlog *backlog_;
  std::size_t nodes_ = 0;
  bool may_defer_ = false;
  std::unique_ptr<deferred_data_job> deferred_;
  data_step_scope *outer_;
  backlog_scope routing_;
};

} // namespace detail

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_DATA_JOB_H
