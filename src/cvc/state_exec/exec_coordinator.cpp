/*
  Copyright 2026 The University of Texas at Austin

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <cstring>
#include <cvc/state.h>
#include <cvc/state_cluster_membership.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_exec/builtins.h>
#include <cvc/state_exec/exec_coordinator.h>
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/state_value_codec.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <sstream>
#include <stdexcept>

namespace cvc::state_exec {

// ===========================================================================
// Lightweight JSON helpers (no external dependency)
// ===========================================================================

namespace {

void jkv(std::ostringstream &os, const char *key, const std::string &val, bool &f) {
  if (!f)
    os << ',';
  // Escape basic special characters in val
  os << '"' << key << "\":\"";
  for (char c : val) {
    switch (c) {
    case '"':
      os << "\\\"";
      break;
    case '\\':
      os << "\\\\";
      break;
    case '\n':
      os << "\\n";
      break;
    case '\r':
      os << "\\r";
      break;
    case '\t':
      os << "\\t";
      break;
    default:
      os << c;
      break;
    }
  }
  os << '"';
  f = false;
}

void jkv(std::ostringstream &os, const char *key, int64_t val, bool &f) {
  if (!f)
    os << ',';
  os << '"' << key << "\":" << val;
  f = false;
}

void jkv(std::ostringstream &os, const char *key, uint64_t val, bool &f) {
  if (!f)
    os << ',';
  os << '"' << key << "\":" << val;
  f = false;
}

void jkv(std::ostringstream &os, const char *key, int val, bool &f) {
  jkv(os, key, static_cast<int64_t>(val), f);
}

void jkv(std::ostringstream &os, const char *key, double val, bool &f) {
  if (!f)
    os << ',';
  os << '"' << key << "\":" << val;
  f = false;
}

void jkv(std::ostringstream &os, const char *key, bool val, bool &f) {
  if (!f)
    os << ',';
  os << '"' << key << "\":" << (val ? "true" : "false");
  f = false;
}

bool xstr(const std::string &json, const char *key, std::string &out) {
  std::string needle = std::string("\"") + key + "\":\"";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  std::string result;
  for (size_t i = pos; i < json.size(); ++i) {
    if (json[i] == '\\' && i + 1 < json.size()) {
      ++i;
      switch (json[i]) {
      case '"':
        result += '"';
        break;
      case '\\':
        result += '\\';
        break;
      case 'n':
        result += '\n';
        break;
      case 'r':
        result += '\r';
        break;
      case 't':
        result += '\t';
        break;
      default:
        result += json[i];
        break;
      }
    } else if (json[i] == '"') {
      break;
    } else {
      result += json[i];
    }
  }
  out = std::move(result);
  return true;
}

bool xi64(const std::string &json, const char *key, int64_t &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && json[pos] == ' ')
    ++pos;
  char *end = nullptr;
  out = std::strtoll(json.c_str() + pos, &end, 10);
  return end != json.c_str() + pos;
}

bool xu64(const std::string &json, const char *key, uint64_t &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && json[pos] == ' ')
    ++pos;
  char *end = nullptr;
  out = std::strtoull(json.c_str() + pos, &end, 10);
  return end != json.c_str() + pos;
}

bool xint(const std::string &json, const char *key, int &out) {
  int64_t v = 0;
  if (!xi64(json, key, v))
    return false;
  out = static_cast<int>(v);
  return true;
}

bool xdbl(const std::string &json, const char *key, double &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && json[pos] == ' ')
    ++pos;
  char *end = nullptr;
  out = std::strtod(json.c_str() + pos, &end);
  return end != json.c_str() + pos;
}

bool xbool(const std::string &json, const char *key, bool &out) {
  std::string needle = std::string("\"") + key + "\":";
  auto pos = json.find(needle);
  if (pos == std::string::npos)
    return false;
  pos += needle.size();
  while (pos < json.size() && json[pos] == ' ')
    ++pos;
  if (json.compare(pos, 4, "true") == 0) {
    out = true;
    return true;
  }
  if (json.compare(pos, 5, "false") == 0) {
    out = false;
    return true;
  }
  return false;
}

std::string make_path(const std::string &cluster_id, const std::string &suffix) {
  return std::string(PATH_EXEC_PREFIX) + "." + cluster_id + "." + suffix;
}

} // anonymous namespace

// ===========================================================================
// Construction / destruction
// ===========================================================================

exec_coordinator::exec_coordinator() = default;

exec_coordinator::~exec_coordinator() {
  if (running_)
    stop();
}

// ===========================================================================
// Wiring
// ===========================================================================

void exec_coordinator::attach_scheduler(scheduler *sched) { sched_ = sched; }

void exec_coordinator::attach_shard(cvc::state_cluster_shard *shard) { shard_ = shard; }

void exec_coordinator::attach_membership(cvc::state_cluster_membership *m) { membership_ = m; }

void exec_coordinator::attach_message_bus(cvc::state_message_bus *bus) { bus_ = bus; }

void exec_coordinator::set_resource_policy(const resource_policy &p) {
  std::lock_guard lk(remote_mu_);
  policy_ = p;
}

void exec_coordinator::set_config(const config &cfg) { cfg_ = cfg; }

void exec_coordinator::set_node_id(const std::string &id) { node_id_ = id; }

void exec_coordinator::set_cluster_id(const std::string &id) { cluster_id_ = id; }

// ===========================================================================
// Lifecycle
// ===========================================================================

void exec_coordinator::start() {
  if (running_)
    return;
  if (!bus_)
    throw std::runtime_error("exec_coordinator: message bus required");
  if (!sched_)
    throw std::runtime_error("exec_coordinator: scheduler required");

  // Subscribe to all state_exec messages
  std::string prefix = std::string(PATH_EXEC_PREFIX);
  bus_sub_id_ = bus_->subscribe(prefix, [this](const cvc::state_message &m) { on_message(m); });

  running_ = true;

  // If no leader known, start election
  {
    std::lock_guard lk(leader_mu_);
    if (leader_node_id_.empty()) {
      start_election();
    }
  }
}

void exec_coordinator::stop() {
  if (!running_)
    return;
  running_ = false;

  if (bus_ && bus_sub_id_) {
    bus_->unsubscribe(bus_sub_id_);
    bus_sub_id_ = 0;
  }

  is_leader_ = false;
  election_in_progress_ = false;
}

// ===========================================================================
// Election protocol (Step 28)
// ===========================================================================

std::string exec_coordinator::leader_node_id() const {
  std::lock_guard lk(leader_mu_);
  return leader_node_id_;
}

void exec_coordinator::request_election() { start_election(); }

void exec_coordinator::start_election() {
  election_in_progress_ = true;
  election_start_time_ = std::chrono::steady_clock::now();

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.elections_initiated;
  }

  // Send election announcement
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "type", std::string("election-start"), f);
  jkv(os, "node_id", node_id_, f);
  jkv(os, "priority", cfg_.election_priority, f);
  auto ts = std::chrono::steady_clock::now().time_since_epoch().count();
  jkv(os, "timestamp", static_cast<int64_t>(ts), f);
  os << '}';

  send_message(make_path(cluster_id_, "election"), MIME_EXEC_ELECTION, os.str());
}

void exec_coordinator::declare_victory() {
  {
    std::lock_guard lk(leader_mu_);
    leader_node_id_ = node_id_;
  }
  is_leader_ = true;
  election_in_progress_ = false;

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.elections_won;
  }

  // Broadcast victory
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "type", std::string("election-victory"), f);
  jkv(os, "node_id", node_id_, f);
  os << '}';

  send_message(make_path(cluster_id_, "election"), MIME_EXEC_ELECTION, os.str());
}

void exec_coordinator::accept_leader(const std::string &new_leader) {
  {
    std::lock_guard lk(leader_mu_);
    leader_node_id_ = new_leader;
  }
  is_leader_ = (new_leader == node_id_);
  election_in_progress_ = false;
}

void exec_coordinator::handle_election(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string type, sender;
  xstr(json, "type", type);
  xstr(json, "node_id", sender);

  if (type == "election-start") {
    if (sender == node_id_)
      return; // ignore own start messages

    int sender_pri = 0;
    xint(json, "priority", sender_pri);

    // Bully protocol: respond if we have higher priority or higher node_id
    bool we_win = (cfg_.election_priority > sender_pri) ||
                  (cfg_.election_priority == sender_pri && node_id_ > sender);

    if (we_win) {
      // Send alive (we're taking over)
      std::ostringstream os;
      os << '{';
      bool f = true;
      jkv(os, "type", std::string("election-alive"), f);
      jkv(os, "node_id", node_id_, f);
      jkv(os, "priority", cfg_.election_priority, f);
      os << '}';

      send_message(make_path(cluster_id_, "election"), MIME_EXEC_ELECTION, os.str());

      // Start our own election
      if (!election_in_progress_) {
        start_election();
      }
    }
  } else if (type == "election-alive") {
    if (sender == node_id_)
      return; // ignore own alive messages

    int sender_pri = 0;
    xint(json, "priority", sender_pri);

    // A higher-priority node is alive — stand down
    bool they_win = (sender_pri > cfg_.election_priority) ||
                    (sender_pri == cfg_.election_priority && sender > node_id_);
    if (they_win) {
      election_in_progress_ = false;
    }
  } else if (type == "election-victory") {
    accept_leader(sender);
  }
}

void exec_coordinator::on_membership_event(int kind, const std::string &nid) {
  // kind: 0=joined, 1=suspect, 2=dead, 3=evicted
  if (kind >= 2) { // dead or evicted
    std::string current_leader;
    {
      std::lock_guard lk(leader_mu_);
      current_leader = leader_node_id_;
    }
    if (nid == current_leader) {
      // Leader died — trigger re-election
      start_election();
    }

    // Clean up remote state for dead node
    {
      std::lock_guard lk(remote_mu_);
      remote_stats_.erase(nid);
      remote_procs_.erase(nid);
      remote_last_seen_.erase(nid);
    }
  }
}

// ===========================================================================
// Heartbeat (Step 28)
// ===========================================================================

void exec_coordinator::emit_heartbeat() {
  if (!sched_)
    return;

  auto s = sched_->get_stats();
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "node_id", node_id_, f);
  jkv(os, "is_leader", is_leader_.load(), f);
  jkv(os, "stats", serialize_scheduler_stats(s), f);
  os << '}';

  send_message(make_path(cluster_id_, "heartbeat"), MIME_EXEC_HEARTBEAT, os.str());

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.heartbeats_sent;
  }
}

void exec_coordinator::handle_heartbeat(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string sender;
  xstr(json, "node_id", sender);
  if (sender == node_id_)
    return;

  scheduler_stats rs;
  std::string stats_json;
  xstr(json, "stats", stats_json);
  deserialize_scheduler_stats(stats_json, rs);

  {
    std::lock_guard lk(remote_mu_);
    remote_stats_[sender] = rs;
    remote_last_seen_[sender] = std::chrono::steady_clock::now();
  }

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.heartbeats_received;
  }

  // If election is in progress and we haven't heard a victory yet,
  // check if enough time has passed to declare ourselves winner
  if (election_in_progress_) {
    auto elapsed = std::chrono::steady_clock::now() - election_start_time_;
    if (elapsed >= cfg_.election_timeout) {
      declare_victory();
    }
  }
}

// ===========================================================================
// Status broadcast (Step 31)
// ===========================================================================

void exec_coordinator::emit_status_broadcast() {
  if (!sched_)
    return;

  auto procs = sched_->list_processes();
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "node_id", node_id_, f);
  jkv(os, "processes", serialize_process_list(procs), f);
  jkv(os, "stats", serialize_scheduler_stats(sched_->get_stats()), f);
  os << '}';

  send_message(make_path(cluster_id_, "status"), MIME_EXEC_STATUS, os.str());

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.status_broadcasts;
  }
}

void exec_coordinator::handle_status(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string sender;
  xstr(json, "node_id", sender);
  if (sender == node_id_)
    return;

  std::vector<process_info> procs;
  std::string procs_json;
  if (xstr(json, "processes", procs_json)) {
    deserialize_process_list(procs_json, procs);
  }

  scheduler_stats rs;
  std::string stats_json;
  if (xstr(json, "stats", stats_json)) {
    deserialize_scheduler_stats(stats_json, rs);
  }

  {
    std::lock_guard lk(remote_mu_);
    remote_procs_[sender] = std::move(procs);
    remote_stats_[sender] = rs;
    remote_last_seen_[sender] = std::chrono::steady_clock::now();
  }
}

// ===========================================================================
// Process submission (Step 29)
// ===========================================================================

submit_result exec_coordinator::submit(const std::string &script, const execute_options &opts) {
  if (!sched_)
    return {-1, false, "no scheduler attached", ""};

  // If we're the leader (or no leader known), submit locally
  bool local = is_leader_ || leader_node_id().empty();
  if (local) {
    return submit_local(script, opts);
  }

  // Forward to leader via message
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "action", std::string("submit"), f);
  jkv(os, "script", script, f);
  jkv(os, "name", opts.name, f);
  jkv(os, "priority", opts.priority, f);
  jkv(os, "uid", opts.uid, f);
  jkv(os, "gid", opts.gid, f);
  jkv(os, "max_steps", opts.max_steps, f);
  jkv(os, "max_time", opts.max_time, f);
  jkv(os, "max_memory", opts.max_memory, f);
  jkv(os, "max_messages", opts.max_messages, f);
  jkv(os, "reply_node", node_id_, f);
  os << '}';

  send_message(make_path(cluster_id_, "submit"), MIME_EXEC_SUBMIT, os.str());

  // For forwarded submissions, we return a pending result
  return {-1, true, "", leader_node_id()};
}

submit_result exec_coordinator::submit(const value_t &expr, const execute_options &opts) {
  if (!sched_)
    return {-1, false, "no scheduler attached", ""};

  bool local = is_leader_ || leader_node_id().empty();
  if (local) {
    return submit_local(expr, opts);
  }

  // For remote submission of expressions, we'd need to serialize the
  // value_t. For now, convert to string representation.
  return {-1, false, "expression submission requires local leader", ""};
}

submit_result exec_coordinator::submit_local(const std::string &script,
                                             const execute_options &opts) {
  // Validate against resource policy
  process_limits req{opts.max_steps, opts.max_time, opts.max_memory, opts.max_messages};
  auto validated = validate_limits(policy_, req);

  execute_options validated_opts = opts;
  validated_opts.max_steps = validated.max_steps;
  validated_opts.max_time = validated.max_time;
  validated_opts.max_memory = validated.max_memory;
  validated_opts.max_messages = validated.max_messages;

  // Check process count limit
  if (policy_.max_processes > 0 && sched_->process_count() >= policy_.max_processes) {
    std::lock_guard lk(stats_mu_);
    ++stats_.submissions_received;
    ++stats_.submissions_rejected;
    return {-1, false, "max_processes limit reached", node_id_};
  }

  int pid = sched_->execute(script, validated_opts);

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.submissions_received;
    ++stats_.submissions_accepted;
  }

  return {pid, true, "", node_id_};
}

submit_result exec_coordinator::submit_local(const value_t &expr, const execute_options &opts) {
  process_limits req{opts.max_steps, opts.max_time, opts.max_memory, opts.max_messages};
  auto validated = validate_limits(policy_, req);

  execute_options validated_opts = opts;
  validated_opts.max_steps = validated.max_steps;
  validated_opts.max_time = validated.max_time;
  validated_opts.max_memory = validated.max_memory;
  validated_opts.max_messages = validated.max_messages;

  if (policy_.max_processes > 0 && sched_->process_count() >= policy_.max_processes) {
    std::lock_guard lk(stats_mu_);
    ++stats_.submissions_received;
    ++stats_.submissions_rejected;
    return {-1, false, "max_processes limit reached", node_id_};
  }

  int pid = sched_->execute(expr, validated_opts);

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.submissions_received;
    ++stats_.submissions_accepted;
  }

  return {pid, true, "", node_id_};
}

void exec_coordinator::handle_submit(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string action;
  xstr(json, "action", action);

  if (action == "submit") {
    // Only the leader processes submissions
    if (!is_leader_)
      return;

    std::string script, name, uid, gid, reply_node;
    int priority = 0;
    uint64_t max_steps = 0, max_memory = 0, max_messages = 0;
    double max_time = 0.0;

    xstr(json, "script", script);
    xstr(json, "name", name);
    xstr(json, "uid", uid);
    xstr(json, "gid", gid);
    xstr(json, "reply_node", reply_node);
    xint(json, "priority", priority);
    xu64(json, "max_steps", max_steps);
    xdbl(json, "max_time", max_time);
    xu64(json, "max_memory", max_memory);
    xu64(json, "max_messages", max_messages);

    execute_options opts;
    opts.name = std::move(name);
    opts.priority = priority;
    opts.uid = std::move(uid);
    opts.gid = std::move(gid);
    opts.max_steps = max_steps;
    opts.max_time = max_time;
    opts.max_memory = max_memory;
    opts.max_messages = max_messages;

    auto result = submit_local(script, opts);

    // Send response to the requesting node
    if (!reply_node.empty()) {
      send_submit_response(make_path(cluster_id_, "submit.reply." + reply_node), result);
    }
  } else if (action == "submit-response") {
    // Response from leader — the requesting node can process this
    // (e.g., update a promise). Currently tracked via stats only.
  }
}

void exec_coordinator::send_submit_response(const std::string &reply_path,
                                            const submit_result &result) {
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "action", std::string("submit-response"), f);
  jkv(os, "pid", static_cast<int64_t>(result.pid), f);
  jkv(os, "accepted", result.accepted, f);
  jkv(os, "error", result.error, f);
  jkv(os, "node_id", result.node_id, f);
  os << '}';

  send_message(reply_path, MIME_EXEC_SUBMIT, os.str());
}

// ===========================================================================
// Process migration (Step 30)
// ===========================================================================

migrate_result exec_coordinator::migrate(int pid, const std::string &target_node_id) {
  if (!sched_)
    return {false, -1, target_node_id, "no scheduler"};
  if (target_node_id == node_id_)
    return {false, -1, target_node_id, "cannot migrate to self"};

  // Pause the process
  if (!sched_->pause(pid)) {
    return {false, -1, target_node_id, "failed to pause process"};
  }

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.migrations_initiated;
  }

  // Serialize process state
  std::string data = serialize_process_for_migration(pid);
  if (data.empty()) {
    sched_->resume(pid);
    std::lock_guard lk(stats_mu_);
    ++stats_.migrations_failed;
    return {false, -1, target_node_id, "serialization failed"};
  }

  // Send migration message to target
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "action", std::string("migrate-request"), f);
  jkv(os, "source_node", node_id_, f);
  jkv(os, "source_pid", static_cast<int64_t>(pid), f);
  jkv(os, "process_data", data, f);
  os << '}';

  send_message(make_path(cluster_id_, "migrate." + target_node_id), MIME_EXEC_MIGRATE, os.str());

  // Kill the original process (migration is fire-and-forward)
  sched_->kill(pid);

  return {true, -1, target_node_id, ""};
}

void exec_coordinator::handle_migrate(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string action;
  xstr(json, "action", action);

  if (action == "migrate-request") {
    std::string source_node, process_data;
    int64_t source_pid = -1;
    xstr(json, "source_node", source_node);
    xi64(json, "source_pid", source_pid);
    xstr(json, "process_data", process_data);

    int new_pid = ingest_migrated_process(process_data);

    // Send ack back to source
    std::ostringstream os;
    os << '{';
    bool f = true;
    jkv(os, "action", std::string("migrate-ack"), f);
    jkv(os, "source_pid", source_pid, f);
    jkv(os, "new_pid", static_cast<int64_t>(new_pid), f);
    jkv(os, "target_node", node_id_, f);
    jkv(os, "success", new_pid >= 0, f);
    os << '}';

    send_message(make_path(cluster_id_, "migrate." + source_node), MIME_EXEC_MIGRATE, os.str());

    if (new_pid >= 0) {
      std::lock_guard lk(stats_mu_);
      ++stats_.migrations_completed;
    } else {
      std::lock_guard lk(stats_mu_);
      ++stats_.migrations_failed;
    }
  } else if (action == "migrate-ack") {
    // Source receives ack — update stats.
    // Skip if we sent this ack ourselves (target == self).
    std::string target_node;
    xstr(json, "target_node", target_node);
    if (target_node == node_id_)
      return;

    bool success = false;
    xbool(json, "success", success);
    if (success) {
      std::lock_guard lk(stats_mu_);
      ++stats_.migrations_completed;
    } else {
      std::lock_guard lk(stats_mu_);
      ++stats_.migrations_failed;
    }
  }
}

std::string exec_coordinator::serialize_process_for_migration(int pid) {
  auto pi = sched_->get_process_info(pid);
  if (!pi)
    return {};

  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "name", pi->name, f);
  jkv(os, "priority", pi->priority, f);
  jkv(os, "uid", pi->uid, f);
  jkv(os, "gid", pi->gid, f);
  jkv(os, "max_memory", pi->max_memory, f);
  jkv(os, "max_time", pi->max_time, f);
  jkv(os, "max_messages", pi->max_messages, f);
  jkv(os, "parent_pid", pi->parent_pid, f);
  jkv(os, "step_count", pi->step_count, f);
  jkv(os, "elapsed_time", pi->elapsed_time, f);
  // Note: full evaluator state serialization would use
  // state_value_codec::encode_evaluator_state via state tree.
  // For the coordinator-level migration message, we include the
  // process metadata. Full state goes through a state tree subtree.
  os << '}';
  return os.str();
}

int exec_coordinator::ingest_migrated_process(const std::string &data) {
  if (!sched_ || data.empty())
    return -1;

  std::string name, uid, gid;
  int priority = 0;
  uint64_t max_memory = 0, max_messages = 0;
  double max_time = 0.0;

  xstr(data, "name", name);
  xint(data, "priority", priority);
  xstr(data, "uid", uid);
  xstr(data, "gid", gid);
  xu64(data, "max_memory", max_memory);
  xdbl(data, "max_time", max_time);
  xu64(data, "max_messages", max_messages);

  // Create a new process with the migrated metadata.
  // The actual evaluator state would be reconstructed from the
  // state tree via state_value_codec::decode_evaluator_state.
  // For now, the migrated process starts with a no-op script
  // (the state tree carries the real evaluator state).
  execute_options opts;
  opts.name = name.empty() ? "migrated" : name;
  opts.priority = priority;
  opts.uid = uid;
  opts.gid = gid;
  opts.max_memory = max_memory;
  opts.max_time = max_time;
  opts.max_messages = max_messages;

  // Submit a minimal process — in production, restoring the full
  // evaluator_state involves decode_evaluator_state from the tree
  int pid = sched_->execute(std::string("nil"), opts);
  return pid;
}

// ===========================================================================
// Cross-cluster observation (Step 31)
// ===========================================================================

std::vector<cluster_process_info> exec_coordinator::ps_all() const {
  std::vector<cluster_process_info> result;

  // Local processes
  if (sched_) {
    for (const auto &pi : sched_->list_processes()) {
      result.push_back({pi, node_id_});
    }
  }

  // Remote processes from status broadcasts
  {
    std::lock_guard lk(remote_mu_);
    for (const auto &[nid, procs] : remote_procs_) {
      for (const auto &pi : procs) {
        result.push_back({pi, nid});
      }
    }
  }

  return result;
}

cluster_exec_stats exec_coordinator::cluster_stats() const {
  cluster_exec_stats result;

  if (sched_) {
    result.local = sched_->get_stats();
    result.total_processes += result.local.total_processes;
    result.total_running += result.local.running;
  }

  {
    std::lock_guard lk(remote_mu_);
    for (const auto &[nid, rs] : remote_stats_) {
      result.per_node[nid] = rs;
      result.total_processes += rs.total_processes;
      result.total_running += rs.running;
    }
  }

  return result;
}

// ===========================================================================
// Admin controls (Step 32)
// ===========================================================================

bool exec_coordinator::admin_pause(int pid, const std::string &node_id) {
  {
    std::lock_guard lk(stats_mu_);
    ++stats_.admin_commands;
  }

  // Local operation
  if (node_id.empty() || node_id == node_id_) {
    if (!sched_)
      return false;
    return sched_->pause(pid);
  }

  // Remote: send control message
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "command", std::string("pause"), f);
  jkv(os, "pid", static_cast<int64_t>(pid), f);
  jkv(os, "from", node_id_, f);
  os << '}';

  send_message(make_path(cluster_id_, "control." + node_id), MIME_EXEC_CONTROL, os.str());
  return true;
}

bool exec_coordinator::admin_resume(int pid, const std::string &node_id) {
  {
    std::lock_guard lk(stats_mu_);
    ++stats_.admin_commands;
  }

  if (node_id.empty() || node_id == node_id_) {
    if (!sched_)
      return false;
    return sched_->resume(pid);
  }

  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "command", std::string("resume"), f);
  jkv(os, "pid", static_cast<int64_t>(pid), f);
  jkv(os, "from", node_id_, f);
  os << '}';

  send_message(make_path(cluster_id_, "control." + node_id), MIME_EXEC_CONTROL, os.str());
  return true;
}

bool exec_coordinator::admin_kill(int pid, const std::string &node_id) {
  {
    std::lock_guard lk(stats_mu_);
    ++stats_.admin_commands;
  }

  if (node_id.empty() || node_id == node_id_) {
    if (!sched_)
      return false;
    return sched_->kill(pid);
  }

  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "command", std::string("kill"), f);
  jkv(os, "pid", static_cast<int64_t>(pid), f);
  jkv(os, "from", node_id_, f);
  os << '}';

  send_message(make_path(cluster_id_, "control." + node_id), MIME_EXEC_CONTROL, os.str());
  return true;
}

bool exec_coordinator::admin_set_policy(const resource_policy &policy,
                                        const std::string &target_cluster) {
  {
    std::lock_guard lk(stats_mu_);
    ++stats_.admin_commands;
  }

  // Apply locally
  set_resource_policy(policy);

  // Broadcast to cluster
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "from", node_id_, f);
  jkv(os, "policy", serialize_resource_policy(policy), f);
  os << '}';

  std::string cid = target_cluster.empty() ? cluster_id_ : target_cluster;
  send_message(make_path(cid, "policy"), MIME_EXEC_POLICY, os.str());
  return true;
}

bool exec_coordinator::admin_handoff(const std::string &target_node_id) {
  if (!is_leader_)
    return false;

  {
    std::lock_guard lk(stats_mu_);
    ++stats_.admin_commands;
  }

  // Announce handoff as a victory for the target
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "type", std::string("election-victory"), f);
  jkv(os, "node_id", target_node_id, f);
  os << '}';

  send_message(make_path(cluster_id_, "election"), MIME_EXEC_ELECTION, os.str());

  // Accept the new leader locally
  accept_leader(target_node_id);
  return true;
}

void exec_coordinator::handle_control(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string command;
  xstr(json, "command", command);

  int pid = 0;
  {
    int64_t v = 0;
    xi64(json, "pid", v);
    pid = static_cast<int>(v);
  }

  if (!sched_)
    return;

  if (command == "pause") {
    sched_->pause(pid);
  } else if (command == "resume") {
    sched_->resume(pid);
  } else if (command == "kill") {
    sched_->kill(pid);
  } else if (command == "send_signal") {
    std::string signal;
    xstr(json, "signal", signal);
    sched_->send_signal(pid, signal);
  }
}

void exec_coordinator::handle_policy(const cvc::state_message &msg) {
  const auto &json = msg.string_value;
  std::string from;
  xstr(json, "from", from);
  if (from == node_id_)
    return;

  std::string policy_json;
  if (xstr(json, "policy", policy_json)) {
    resource_policy p;
    if (deserialize_resource_policy(policy_json, p)) {
      std::lock_guard lk(remote_mu_);
      policy_ = p;
    }
  }
}

// ===========================================================================
// Message dispatch
// ===========================================================================

void exec_coordinator::on_message(const cvc::state_message &msg) {
  if (!running_)
    return;

  const auto &ct = msg.content_type.empty() ? msg.effective_content_type() : msg.content_type;

  if (ct == MIME_EXEC_ELECTION) {
    handle_election(msg);
    return;
  }
  if (ct == MIME_EXEC_HEARTBEAT) {
    handle_heartbeat(msg);
    return;
  }
  if (ct == MIME_EXEC_SUBMIT) {
    handle_submit(msg);
    return;
  }
  if (ct == MIME_EXEC_MIGRATE) {
    handle_migrate(msg);
    return;
  }
  if (ct == MIME_EXEC_CONTROL) {
    handle_control(msg);
    return;
  }
  if (ct == MIME_EXEC_STATUS) {
    handle_status(msg);
    return;
  }
  if (ct == MIME_EXEC_POLICY) {
    handle_policy(msg);
    return;
  }
}

// ===========================================================================
// Stats
// ===========================================================================

coordinator_stats exec_coordinator::stats() const {
  std::lock_guard lk(stats_mu_);
  return stats_;
}

// ===========================================================================
// Message sending helper
// ===========================================================================

void exec_coordinator::send_message(const std::string &path, const std::string &content_type,
                                    const std::string &payload) {
  auto m = cvc::state_message::make_text(path, payload, content_type);
  m.cluster_id = cluster_id_;
  // Leave origin_node_id and message_id empty for fire-and-forget
  // (dedup bypass — both empty means every admit fires).
  // Node identity is carried inside the JSON payload.

  if (shard_) {
    // Shard will stamp origin_node_id from its local_node_id
    shard_->send_message(std::move(m));
  } else if (bus_) {
    bus_->admit(m);
  }
}

// ===========================================================================
// Serialization: scheduler_stats
// ===========================================================================

std::string exec_coordinator::serialize_scheduler_stats(const scheduler_stats &s) {
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "total_processes", s.total_processes, f);
  jkv(os, "running", s.running, f);
  jkv(os, "ready", s.ready, f);
  jkv(os, "paused", s.paused, f);
  jkv(os, "terminated", s.terminated, f);
  jkv(os, "killed", s.killed, f);
  jkv(os, "total_steps", s.total_steps, f);
  os << '}';
  return os.str();
}

bool exec_coordinator::deserialize_scheduler_stats(const std::string &json, scheduler_stats &s) {
  if (json.empty())
    return false;
  xint(json, "total_processes", s.total_processes);
  xint(json, "running", s.running);
  xint(json, "ready", s.ready);
  xint(json, "paused", s.paused);
  xint(json, "terminated", s.terminated);
  xint(json, "killed", s.killed);
  xu64(json, "total_steps", s.total_steps);
  return true;
}

// ===========================================================================
// Serialization: process_info
// ===========================================================================

std::string exec_coordinator::serialize_process_info(const process_info &pi) {
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "pid", pi.pid, f);
  jkv(os, "name", pi.name, f);
  jkv(os, "status", std::string(to_string(pi.status)), f);
  jkv(os, "priority", pi.priority, f);
  jkv(os, "uid", pi.uid, f);
  jkv(os, "gid", pi.gid, f);
  jkv(os, "step_count", pi.step_count, f);
  jkv(os, "elapsed_time", pi.elapsed_time, f);
  jkv(os, "current_memory", pi.current_memory, f);
  jkv(os, "peak_memory", pi.peak_memory, f);
  jkv(os, "max_memory", pi.max_memory, f);
  jkv(os, "max_time", pi.max_time, f);
  jkv(os, "message_count", pi.message_count, f);
  jkv(os, "max_messages", pi.max_messages, f);
  jkv(os, "parent_pid", pi.parent_pid, f);
  os << '}';
  return os.str();
}

bool exec_coordinator::deserialize_process_info(const std::string &json, process_info &pi) {
  if (json.empty())
    return false;
  xint(json, "pid", pi.pid);
  xstr(json, "name", pi.name);
  std::string status_str;
  if (xstr(json, "status", status_str))
    pi.status = parse_process_status(status_str);
  xint(json, "priority", pi.priority);
  xstr(json, "uid", pi.uid);
  xstr(json, "gid", pi.gid);
  xu64(json, "step_count", pi.step_count);
  xdbl(json, "elapsed_time", pi.elapsed_time);
  xu64(json, "current_memory", pi.current_memory);
  xu64(json, "peak_memory", pi.peak_memory);
  xu64(json, "max_memory", pi.max_memory);
  xdbl(json, "max_time", pi.max_time);
  xu64(json, "message_count", pi.message_count);
  xu64(json, "max_messages", pi.max_messages);
  xint(json, "parent_pid", pi.parent_pid);
  return true;
}

// ===========================================================================
// Serialization: process list (JSON array)
// ===========================================================================

std::string exec_coordinator::serialize_process_list(const std::vector<process_info> &procs) {
  std::ostringstream os;
  os << '[';
  for (size_t i = 0; i < procs.size(); ++i) {
    if (i > 0)
      os << ',';
    os << serialize_process_info(procs[i]);
  }
  os << ']';
  return os.str();
}

bool exec_coordinator::deserialize_process_list(const std::string &json,
                                                std::vector<process_info> &out) {
  out.clear();
  if (json.empty() || json[0] != '[')
    return false;

  // Simple parser: find each { ... } block
  size_t pos = 1;
  while (pos < json.size()) {
    auto start = json.find('{', pos);
    if (start == std::string::npos)
      break;

    // Find matching closing brace (handles nested braces)
    int depth = 0;
    size_t end = start;
    for (; end < json.size(); ++end) {
      if (json[end] == '{')
        ++depth;
      else if (json[end] == '}') {
        --depth;
        if (depth == 0)
          break;
      }
    }
    if (depth != 0)
      break;

    process_info pi{};
    if (deserialize_process_info(json.substr(start, end - start + 1), pi)) {
      out.push_back(pi);
    }
    pos = end + 1;
  }
  return true;
}

// ===========================================================================
// Serialization: resource_policy
// ===========================================================================

std::string exec_coordinator::serialize_resource_policy(const resource_policy &p) {
  std::ostringstream os;
  os << '{';
  bool f = true;
  jkv(os, "max_time_min", p.max_time_min, f);
  jkv(os, "max_time_max", p.max_time_max, f);
  jkv(os, "max_time_default", p.max_time_default, f);
  jkv(os, "max_memory_min", p.max_memory_min, f);
  jkv(os, "max_memory_max", p.max_memory_max, f);
  jkv(os, "max_memory_default", p.max_memory_default, f);
  jkv(os, "max_messages_min", p.max_messages_min, f);
  jkv(os, "max_messages_max", p.max_messages_max, f);
  jkv(os, "max_messages_default", p.max_messages_default, f);
  jkv(os, "max_steps_min", p.max_steps_min, f);
  jkv(os, "max_steps_max", p.max_steps_max, f);
  jkv(os, "max_steps_default", p.max_steps_default, f);
  jkv(os, "max_processes", p.max_processes, f);
  jkv(os, "max_total_memory", p.max_total_memory, f);
  jkv(os, "enforce", static_cast<int64_t>(static_cast<int>(p.enforce)), f);
  os << '}';
  return os.str();
}

bool exec_coordinator::deserialize_resource_policy(const std::string &json, resource_policy &p) {
  if (json.empty())
    return false;
  xdbl(json, "max_time_min", p.max_time_min);
  xdbl(json, "max_time_max", p.max_time_max);
  xdbl(json, "max_time_default", p.max_time_default);
  xu64(json, "max_memory_min", p.max_memory_min);
  xu64(json, "max_memory_max", p.max_memory_max);
  xu64(json, "max_memory_default", p.max_memory_default);
  xu64(json, "max_messages_min", p.max_messages_min);
  xu64(json, "max_messages_max", p.max_messages_max);
  xu64(json, "max_messages_default", p.max_messages_default);
  xu64(json, "max_steps_min", p.max_steps_min);
  xu64(json, "max_steps_max", p.max_steps_max);
  xu64(json, "max_steps_default", p.max_steps_default);
  xint(json, "max_processes", p.max_processes);
  xu64(json, "max_total_memory", p.max_total_memory);
  int enforce_int = 0;
  if (xint(json, "enforce", enforce_int))
    p.enforce = static_cast<resource_policy::mode>(enforce_int);
  return true;
}

} // namespace cvc::state_exec
