/*
  Copyright 2026 The University of Texas at Austin

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

/**
 * @file exec_coordinator.h
 * @brief Distributed execution coordinator for multi-node scheduling.
 *
 * Manages leader election (bully algorithm), cross-node process submission,
 * live process migration, cluster-wide observation (ps/stats), and
 * remote administration (pause/resume/kill/policy broadcast).  Communicates
 * via cvc::state_message_bus or cvc::state_cluster_shard.
 */
#ifndef CVC_STATE_EXEC_EXEC_COORDINATOR_H
#define CVC_STATE_EXEC_EXEC_COORDINATOR_H

#include <atomic>
#include <chrono>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/resource_policy.h>
#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/state_value_codec.h>
#include <cvc/state_exec/types.h>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cvc {
class state;
class state_cluster_shard;
class state_cluster_membership;
class state_message;
class state_message_bus;
struct membership_event;
} // namespace cvc

namespace cvc::state_exec {

// ---------------------------------------------------------------------------
// Content types for state_exec OOB messages
// ---------------------------------------------------------------------------
inline constexpr const char *MIME_EXEC_ELECTION = "application/x-state-exec-election";
inline constexpr const char *MIME_EXEC_HEARTBEAT = "application/x-state-exec-heartbeat";
inline constexpr const char *MIME_EXEC_SUBMIT = "application/x-state-exec-submit";
inline constexpr const char *MIME_EXEC_MIGRATE = "application/x-state-exec-migrate";
inline constexpr const char *MIME_EXEC_CONTROL = "application/x-state-exec-control";
inline constexpr const char *MIME_EXEC_STATUS = "application/x-state-exec-status";
inline constexpr const char *MIME_EXEC_POLICY = "application/x-state-exec-policy";

// ---------------------------------------------------------------------------
// Message path prefixes
// ---------------------------------------------------------------------------
inline constexpr const char *PATH_EXEC_PREFIX = "__state_exec";

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

struct submit_result {
  int pid = -1;
  bool accepted = false;
  std::string error;
  std::string node_id;
};

struct migrate_result {
  bool success = false;
  int new_pid = -1;
  std::string target_node;
  std::string error;
};

struct cluster_process_info {
  process_info info;
  std::string node_id;
};

struct cluster_exec_stats {
  scheduler_stats local;
  std::unordered_map<std::string, scheduler_stats> per_node;
  int total_processes = 0;
  int total_running = 0;
};

struct coordinator_stats {
  uint64_t elections_initiated = 0;
  uint64_t elections_won = 0;
  uint64_t submissions_received = 0;
  uint64_t submissions_accepted = 0;
  uint64_t submissions_rejected = 0;
  uint64_t migrations_initiated = 0;
  uint64_t migrations_completed = 0;
  uint64_t migrations_failed = 0;
  uint64_t admin_commands = 0;
  uint64_t heartbeats_sent = 0;
  uint64_t heartbeats_received = 0;
  uint64_t status_broadcasts = 0;
};

// ---------------------------------------------------------------------------
// exec_coordinator — distributed scheduling coordinator
//
// Implements Steps 28-32 of the state_exec porting plan:
//   28. Leader election, heartbeat, handoff via state_message_bus
//   29. Process submission via OOB messages with policy validation
//   30. Process migration (pause → serialize → send → resume)
//   31. Cross-cluster observation (ps-all, cluster_stats)
//   32. Admin controls via message bus
//
// The coordinator runs on every node. Exactly one node per cluster
// is elected leader (the "scheduling node"). Clients may submit
// processes to any node; non-leaders forward to the current leader.
//
// Election uses a bully-style protocol over OOB messages:
//   - Triggered on leader failure (peer_dead/peer_evicted)
//   - Higher node_id wins ties; timestamp breaks further ties
//   - Timeout-based convergence (no distributed consensus required)
// ---------------------------------------------------------------------------
class exec_coordinator {
public:
  struct config {
    std::chrono::milliseconds heartbeat_interval{2000};
    std::chrono::milliseconds election_timeout{3000};
    int election_priority = 0; // higher = more likely leader
  };

  exec_coordinator();
  ~exec_coordinator();

  exec_coordinator(const exec_coordinator &) = delete;
  exec_coordinator &operator=(const exec_coordinator &) = delete;

  // -- Wiring (call before start()) --
  void attach_scheduler(scheduler *sched);
  void attach_shard(cvc::state_cluster_shard *shard);
  void attach_membership(cvc::state_cluster_membership *membership);
  void attach_message_bus(cvc::state_message_bus *bus);
  void set_resource_policy(const resource_policy &policy);
  void set_config(const config &cfg);
  void set_node_id(const std::string &node_id);
  void set_cluster_id(const std::string &cluster_id);

  // -- Lifecycle --
  void start();
  void stop();
  bool is_running() const noexcept { return running_; }

  // -- Election (Step 28) --
  bool is_leader() const noexcept { return is_leader_; }
  std::string leader_node_id() const;
  void request_election();

  // -- Process submission (Step 29) --
  submit_result submit(const std::string &script, const execute_options &opts = {});
  submit_result submit(const char *script, const execute_options &opts = {}) {
    return submit(std::string(script), opts);
  }
  submit_result submit(const value_t &expr, const execute_options &opts = {});

  // -- Process migration (Step 30) --
  migrate_result migrate(int pid, const std::string &target_node_id);

  // -- Cross-cluster observation (Step 31) --
  std::vector<cluster_process_info> ps_all() const;
  cluster_exec_stats cluster_stats() const;

  // -- Admin controls (Step 32) --
  bool admin_pause(int pid, const std::string &node_id = {});
  bool admin_resume(int pid, const std::string &node_id = {});
  bool admin_kill(int pid, const std::string &node_id = {});
  bool admin_set_policy(const resource_policy &policy, const std::string &target_cluster = {});
  bool admin_handoff(const std::string &target_node_id);

  // -- Stats --
  coordinator_stats stats() const;

  // -- Message handling (public for testing) --
  void on_message(const cvc::state_message &msg);
  void on_membership_event(int kind, const std::string &node_id);

  // -- Heartbeat (public for testing) --
  void emit_heartbeat();
  void emit_status_broadcast();

  // -- Serialization helpers --
  static std::string serialize_scheduler_stats(const scheduler_stats &s);
  static bool deserialize_scheduler_stats(const std::string &json, scheduler_stats &out);
  static std::string serialize_process_info(const process_info &pi);
  static bool deserialize_process_info(const std::string &json, process_info &out);
  static std::string serialize_process_list(const std::vector<process_info> &procs);
  static bool deserialize_process_list(const std::string &json, std::vector<process_info> &out);
  static std::string serialize_resource_policy(const resource_policy &p);
  static bool deserialize_resource_policy(const std::string &json, resource_policy &out);

private:
  // Election protocol
  void start_election();
  void declare_victory();
  void accept_leader(const std::string &node_id);

  // Message dispatch
  void handle_election(const cvc::state_message &msg);
  void handle_heartbeat(const cvc::state_message &msg);
  void handle_submit(const cvc::state_message &msg);
  void handle_migrate(const cvc::state_message &msg);
  void handle_control(const cvc::state_message &msg);
  void handle_status(const cvc::state_message &msg);
  void handle_policy(const cvc::state_message &msg);

  // Submit helpers
  submit_result submit_local(const std::string &script, const execute_options &opts);
  submit_result submit_local(const value_t &expr, const execute_options &opts);
  void send_submit_response(const std::string &reply_path, const submit_result &result);

  // Migration helpers
  std::string serialize_process_for_migration(int pid);
  int ingest_migrated_process(const std::string &data);

  // Message sending
  void send_message(const std::string &path, const std::string &content_type,
                    const std::string &payload);

  // Configuration
  config cfg_;
  std::string node_id_;
  std::string cluster_id_;

  // Attached components (non-owning)
  scheduler *sched_ = nullptr;
  cvc::state_cluster_shard *shard_ = nullptr;
  cvc::state_cluster_membership *membership_ = nullptr;
  cvc::state_message_bus *bus_ = nullptr;
  resource_policy policy_;

  // Runtime state
  std::atomic<bool> running_{false};
  std::atomic<bool> is_leader_{false};
  std::atomic<bool> election_in_progress_{false};
  mutable std::mutex leader_mu_;
  std::string leader_node_id_;
  std::chrono::steady_clock::time_point election_start_time_;

  // Subscription tracking
  uint64_t bus_sub_id_ = 0;
  size_t membership_cb_id_ = 0;

  // Remote state cache (from heartbeats/status broadcasts)
  mutable std::mutex remote_mu_;
  std::unordered_map<std::string, scheduler_stats> remote_stats_;
  std::unordered_map<std::string, std::vector<process_info>> remote_procs_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> remote_last_seen_;

  // Counters
  mutable std::mutex stats_mu_;
  coordinator_stats stats_;
};

} // namespace cvc::state_exec

#endif // CVC_STATE_EXEC_EXEC_COORDINATOR_H
