/**
 * @file detail/priority.h
 * @brief Priority rules shared by the schedulers and the intrinsics (docs/STATE_EXEC_LOCKS.md
 *        7.2, 8.1): who may change whose priority, the floor, the spawn clamp, and the weights.
 *
 * Internal. Built in phase L2 (locks plan L2.1).
 */
#ifndef CVC_STATE_EXEC_DETAIL_PRIORITY_H
#define CVC_STATE_EXEC_DETAIL_PRIORITY_H

#include <cvc/state/state_exec/scheduler_base.h>
#include <functional>

namespace cvc::state_exec {

struct process;

namespace detail {

/// Design 7.2: whether `caller` may set `target`'s base priority to `p`. `unknown` when the
/// caller may not see the target (it ended, another owner, or another uid that isn't a
/// descendant by parent_pid within 64 hops); lowering is always allowed; raising only down to
/// max(target's floor, caller's floor), else `refused`; the caller itself only to its own floor.
/// `find` looks a pid up among the scheduler's live processes.
priority_change check_priority_change(const process &caller, const process &target, int p,
                                      const std::function<const process *(int)> &find);

/// The visibility half of check_priority_change, for get-priority. A process always sees itself.
bool visible_to(const process &caller, const process &target,
                const std::function<const process *(int)> &find);

/// A process's floor: its priority_floor, or its priority where it has none.
int floor_of(const process &p);

/// The priority a spawn by `caller` asking for `requested` gets: under a priority policy, or with
/// an explicit floor on the caller, nothing better than the caller's floor. `clamped` is set when
/// the request was clamped.
int clamp_spawn_priority(const process &caller, int requested, scheduling_policy policy,
                         bool *clamped);

/// The weight of a nice value (the CFS table, design 8.1; first used by groups, phase L4).
int weight_of(int nice);

} // namespace detail
} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_DETAIL_PRIORITY_H
