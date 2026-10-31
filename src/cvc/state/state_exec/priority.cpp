// Priority rules shared by the schedulers and the intrinsics (docs/STATE_EXEC_LOCKS.md 7.2, 8.1):
// who may change whose priority, the floor, the spawn clamp, and the CFS weights.

#include <algorithm>
#include <cvc/state/state_exec/detail/priority.h>
#include <cvc/state/state_exec/process.h>

namespace cvc::state_exec::detail {

namespace {

// Design 7.2: descent by parent_pid is followed for at most this many hops.
constexpr int kMaxDescentHops = 64;

// Whether `target` descends from `caller` by parent_pid, within kMaxDescentHops. The walk goes
// through live ancestors only (`find`), so a process whose parent ended no longer descends from
// its grandparent, as an orphan doesn't on Unix.
bool descends_from(const process &caller, const process &target,
                   const std::function<const process *(int)> &find) {
  int pid = target.parent_pid;
  for (int hops = 0; hops < kMaxDescentHops && pid >= 0; ++hops) {
    if (pid == caller.pid)
      return true;
    const process *up = find(pid);
    if (!up)
      return false;
    pid = up->parent_pid;
  }
  return false;
}

} // namespace

bool visible_to(const process &caller, const process &target,
                const std::function<const process *(int)> &find) {
  if (caller.pid == target.pid)
    return true;
  if (is_finished(target.status) || target.owner != caller.owner)
    return false;
  return target.uid == caller.uid || descends_from(caller, target, find);
}

priority_change check_priority_change(const process &caller, const process &target, int p,
                                      const std::function<const process *(int)> &find) {
  if (!visible_to(caller, target, find))
    return priority_change::unknown;
  // Lowering (a higher number) is always allowed; raising only down to the worse of the two
  // floors, the target's and the caller's. For the caller itself both are its own.
  if (p >= target.priority)
    return priority_change::applied;
  return p >= std::max(floor_of(target), floor_of(caller)) ? priority_change::applied
                                                           : priority_change::refused;
}

int floor_of(const process &p) { return p.priority_floor.value_or(p.priority); }

int clamp_spawn_priority(const process &caller, int requested, scheduling_policy policy,
                         bool *clamped) {
  if (clamped)
    *clamped = false;
  // Under round_robin a spawn's priority is stored unchecked, as before priorities could change,
  // unless a host gave the caller its floor (design 7.2).
  if (policy == scheduling_policy::round_robin && !caller.priority_floor_explicit)
    return requested;
  const int floor = floor_of(caller);
  if (requested >= floor)
    return requested;
  if (clamped)
    *clamped = true;
  return floor;
}

int weight_of(int nice) {
  // Linux's sched_prio_to_weight: 1024 at nice 0, each step about 1.25x (design 7.5, 8.1).
  static constexpr int kWeights[40] = {
      /* -20 */ 88761, 71755, 56483, 46273, 36291,
      /* -15 */ 29154, 23254, 18705, 14949, 11916,
      /* -10 */ 9548,  7620,  6100,  4904,  3906,
      /*  -5 */ 3121,  2501,  1991,  1586,  1277,
      /*   0 */ 1024,  820,   655,   526,   423,
      /*   5 */ 335,   272,   215,   172,   137,
      /*  10 */ 110,   87,    70,    56,    45,
      /*  15 */ 36,    29,    23,    18,    15,
  };
  return kWeights[std::clamp(nice, -20, 19) + 20];
}

} // namespace cvc::state_exec::detail
