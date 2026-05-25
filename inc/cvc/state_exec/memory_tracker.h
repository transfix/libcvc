/**
 * @file memory_tracker.h
 * @brief Per-process memory accounting for state tree objects.
 *
 * Tracks which process owns each value written to the state tree and
 * accumulates byte counts per PID.  The scheduler checks current_bytes()
 * against a process's max_memory at each step boundary to enforce
 * memory limits.
 */
#ifndef CVC_STATE_EXEC_MEMORY_TRACKER_H
#define CVC_STATE_EXEC_MEMORY_TRACKER_H

#include <cstddef>
#include <cvc/state_exec/types.h>
#include <unordered_map>

namespace cvc::state_exec {

/// Per-process memory accounting for state tree objects.
///
/// Tracks which process "owns" each value written to the state tree and
/// accumulates byte counts per PID.  The scheduler checks current_bytes()
/// against a process's max_memory at each step boundary.
class memory_tracker {
public:
  /// Record that process `pid` wrote `bytes` to a logical node identified
  /// by `id`.  If the node was already owned, the old owner is credited
  /// back and the new owner is charged.
  void record_write(int pid, uintptr_t id, std::size_t bytes);

  /// Record deletion of node `id`.  Credits bytes back to the owner.
  void record_delete(uintptr_t id);

  /// Query live bytes owned by process.
  std::size_t current_bytes(int pid) const;

  /// High-water mark for process.
  std::size_t peak_bytes(int pid) const;

  /// Release all ownership for a terminated process (reassign to PID 0).
  void release_ownership(int pid);

  /// After forking, copy ownership entries from parent to child for a set
  /// of node IDs.
  void fork_ownership(int parent_pid, int child_pid, const std::vector<uintptr_t> &child_node_ids);

  /// Reset all tracking data.
  void clear();

private:
  struct ownership_entry {
    int pid = 0;
    std::size_t bytes = 0;
  };

  std::unordered_map<uintptr_t, ownership_entry> ownership_;
  std::unordered_map<int, std::size_t> current_;
  std::unordered_map<int, std::size_t> peak_;

  void add_bytes(int pid, std::size_t bytes);
  void sub_bytes(int pid, std::size_t bytes);
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_MEMORY_TRACKER_H
