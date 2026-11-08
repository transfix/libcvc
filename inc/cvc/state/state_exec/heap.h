/**
 * @file heap.h
 * @brief A process's live DSL heap: its share for max_memory, and collecting reference cycles.
 *
 * Values are reference counted and may be shared between processes: a broadcast message is one
 * dict held by every receiver, and a forked child shares its parent's lists. A process is charged
 * its share of each node it can reach, size x (its references / the node's owner count), and a
 * node passes its share on to the nodes under it. A node's shares over all its holders add up to
 * its size, so a value held by k processes costs each of them about 1/k of it.
 *
 * Reference counting cannot free a cycle: a closure bound in the environment it captured (every
 * defun), or a list appended to itself. The collector here finds the cycles nothing outside them
 * holds, as CPython's does: it counts the references the tracked objects make to each other, and
 * whatever has more owners than that is held from outside, as is everything it reaches. The rest
 * is garbage, and clearing it lets it free.
 */
#ifndef CVC_STATE_EXEC_HEAP_H
#define CVC_STATE_EXEC_HEAP_H

#include <cstddef>
#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <memory>
#include <string>

namespace cvc {
class state;
}

namespace cvc::state_exec {

class data_job;
struct process;

// ---------------------------------------------------------------------------
// Measuring a process's heap
// ---------------------------------------------------------------------------

/// Heap settings. Hosts keep them on the state tree under `limits.*`, like
/// limits.max_alloc_bytes.
struct heap_options {
  /// Re-measure a process that has a memory budget at least every this many steps, besides the
  /// measurements its allocations call for (0 = only those). State-tree key:
  /// `limits.heap_check_steps`.
  std::size_t check_steps = 4096;
};

/// Resolve heap settings from a state tree, per key: `state_exec.schedulers.<id>.limits.<key>`,
/// then `state_exec.defaults.limits.<key>`, then `base`. A missing or malformed value is skipped.
heap_options heap_options_from_state(cvc::state &root, const std::string &scheduler_id = {},
                                     const heap_options &base = {});

/// A data job that measures `proc`'s heap share; its result is the share in bytes, as an int.
/// It holds what it measures, so the process (and others) may run between its slices.
std::unique_ptr<data_job> make_heap_share_job(const process &proc);

/// `proc`'s heap share in bytes, measured now in one go.
std::size_t measure_heap_share(const process &proc);

/// The bytes a re-measure waits for: natives' reported allocations since the last measurement
/// trigger one once they reach this (64 KiB, or half the last measurement if that is more).
std::size_t heap_remeasure_bytes(std::size_t last_measured);

/// The heap accounting a scheduler does after each step of `proc`: take a measurement that just
/// finished (proc.heap_bytes), add `allocated` (the step guard's eval_alloc_guard::allocated()),
/// and start the next measurement as the process's data work when one is due
/// (process::heap_measure_due), on top of any deep-data work in flight. The measurement counts
/// that work too, with what its jobs hold (data_job::held_values). `state_bytes` is the memory
/// tracker's count for the process. Nothing is measured for a process without a memory budget.
void heap_after_step(process &proc, std::size_t allocated, uint64_t state_bytes);

// ---------------------------------------------------------------------------
// Reference cycles
// ---------------------------------------------------------------------------

// Tracked objects live in a cycle registry: the one bound on the calling thread
// (cycle_registry_scope), else the thread's own. A scheduler given a registry
// (set_cycle_registry) binds it itself on every entry that can run DSL or track, so whichever
// thread drives it, its objects go to its registry (docs/STATE_EXEC_THREADED.md 4.7). Bare
// evaluators and bare schedulers use the thread's own.

/// A registry of tracked objects, owned by a group (an exec_group, the app's scheduler, a pycvc
/// Exec) and set on its scheduler. Opaque.
struct cycle_registry;

/// Collects the registry under its own binding, then frees it: what its objects still hold is
/// garbage by then, since its owner destroys it after its scheduler and environments. What a host
/// still holds joins the caller's registry (the bound one, else the thread's own).
struct cycle_registry_deleter {
  void operator()(cycle_registry *r) const noexcept;
};
using cycle_registry_ptr = std::unique_ptr<cycle_registry, cycle_registry_deleter>;

/// A new, empty registry with an id of its own (bound_cycle_registry_id()).
cycle_registry_ptr make_cycle_registry();

/// RAII: bind `r` on the calling thread until the scope ends (null: none, so the thread's own
/// registry is used), then restore the binding it found. One store and one restore.
class cycle_registry_scope {
public:
  explicit cycle_registry_scope(cycle_registry *r) noexcept;
  ~cycle_registry_scope();
  cycle_registry_scope(const cycle_registry_scope &) = delete;
  cycle_registry_scope &operator=(const cycle_registry_scope &) = delete;

private:
  cycle_registry *outer_;
};

/// Track an environment that a closure, a generator or a class method captured: it may close a
/// cycle (the closure bound in it, or in something it holds). It is tracked in the bound
/// registry, else this thread's.
void track_captured_env(const environment_ptr &env);

/// Track a list or dict that was changed in place (append, set-nth, set-attr): only a change in
/// place can make a container hold itself.
void track_mutated(const list_ptr &list);
void track_mutated(const dict_ptr &dict);

/// What a collection did.
struct cycle_stats {
  std::size_t candidates = 0; ///< tracked objects still alive when it started
  std::size_t examined = 0;   ///< objects reachable from them
  std::size_t freed = 0;      ///< objects found to be garbage and cleared
};

/// Find the reference cycles among the tracked objects of the bound registry, else this
/// thread's, that nothing outside them holds, and clear them so they free. Does nothing (and
/// returns zeros) while an evaluator step is running on this thread: call it between steps. The
/// schedulers call it when one is due.
cycle_stats collect_cycles();
/// The same for `r`, bound while it runs.
cycle_stats collect_cycles(cycle_registry &r);

/// Whether the bound registry, else this thread's, has tracked enough since its last collection
/// for one to be due: as many new objects as survived the last one, at least an eighth of the
/// objects it walked (so a walk costs a bounded amount per newly tracked object), and at least
/// 256.
bool cycle_collection_due();
/// The same for `r`.
bool cycle_collection_due(const cycle_registry &r);
/// Whether `r` should be collected now that its scheduler has gone idle (S1.1): a process ended
/// and something was tracked since the last collection, with no time throttle, since an idle
/// group takes no step that would collect it later.
bool cycle_collection_idle_due(const cycle_registry &r);

/// Note that a process ended, in the bound registry, else this thread's: what it leaked is
/// garbage now, so a collection is due as soon as anything has been tracked since the last one
/// (at most once a second).
void note_process_exit();
/// The same for `r`: a scheduler names its own registry, so a host's kill(pid) between pumps notes
/// the group's.
void note_process_exit(cycle_registry &r);

/// Objects the bound registry, else this thread's, tracks: those alive after its last collection
/// plus those tracked since.
std::size_t tracked_objects();
/// The same for `r`.
std::size_t tracked_objects(const cycle_registry &r);

/// The id of the registry bound on the calling thread, or 0 when none is (the thread's own is
/// used). Ids are monotonic, never reused by a later registry at the same address.
std::uint64_t bound_cycle_registry_id() noexcept;

/// The calling thread's own registry is collected when the thread ends (S1-Q3). A thread opts in
/// once: the pool's workers do, and test threads may. Python threads and the main thread never
/// should: their thread-locals go after Py_Finalize, or under the Windows loader lock.
void collect_cycles_at_thread_exit();

/// Tests: whether a process exit is noted in `r` since its last collection.
bool exit_pending_for_test(const cycle_registry &r);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_HEAP_H
