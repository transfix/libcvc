/**
 * @file resource_policy.h
 * @brief Cluster-level resource policy for process limits.
 *
 * Defines constraints on per-process resource allocations (time, steps,
 * memory) and the policy for handling out-of-range requests (clamp or
 * reject).  Applied by the coordinator/scheduler at each execute() call.
 */
#ifndef CVC_STATE_EXEC_RESOURCE_POLICY_H
#define CVC_STATE_EXEC_RESOURCE_POLICY_H

#include <cstdint>
#include <string>

namespace cvc::state_exec {

/// Cluster-level resource policy for process limits.
///
/// Applied to each execute() call on the scheduling node.  Controls how
/// per-process limits are constrained and what happens when requests
/// fall outside the configured range.
struct resource_policy {
  // Per-process time limits (seconds)
  double max_time_min = 0.0;     // floor (0 = no minimum)
  double max_time_max = 0.0;     // ceiling (0 = no maximum)
  double max_time_default = 0.0; // applied when not specified (0 = unlimited)

  // Per-process memory limits (bytes)
  uint64_t max_memory_min = 0;
  uint64_t max_memory_max = 0;
  uint64_t max_memory_default = 0;

  // Per-process message limits
  uint64_t max_messages_min = 0;
  uint64_t max_messages_max = 0;
  uint64_t max_messages_default = 0;

  // Per-process message byte limits
  uint64_t max_message_bytes_min = 0;
  uint64_t max_message_bytes_max = 0;
  uint64_t max_message_bytes_default = 0;

  // Per-process step limits
  uint64_t max_steps_min = 0;
  uint64_t max_steps_max = 0;
  uint64_t max_steps_default = 0;

  // Cluster-wide caps
  int max_processes = 0;         // 0 = unlimited
  uint64_t max_total_memory = 0; // aggregate bytes, 0 = unlimited

  /// Enforcement mode.
  enum class mode { strict, clamp, warn };
  mode enforce = mode::clamp;
};

/// Limits requested for a single process.
struct process_limits {
  uint64_t max_steps = 0;
  double max_time = 0.0;
  uint64_t max_memory = 0;
  uint64_t max_messages = 0;
  uint64_t max_message_bytes = 0;
};

/// Validate and possibly adjust limits against a policy.
///
/// In strict mode, throws std::runtime_error if limits are out of range.
/// In clamp mode, silently adjusts limits to [min, max].
/// In warn mode, returns adjusted limits (caller should log).
///
/// Returns the (possibly adjusted) limits.
process_limits validate_limits(const resource_policy &policy, const process_limits &requested);

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_RESOURCE_POLICY_H
