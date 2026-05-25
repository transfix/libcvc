#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cvc/state.h>
#include <cvc/state_exec/builtins.h>
#include <cvc/state_exec/intrinsics.h>
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/scheduler.h>
#include <stdexcept>
#include <string>

namespace cvc::state_exec {

namespace {

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

void expect_exact(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() != n)
    throw std::runtime_error(std::string(name) + ": expected " + std::to_string(n) +
                             " argument(s), got " + std::to_string(args.size()));
}

void expect_min(std::span<const value_t> args, size_t n, const char *name) {
  if (args.size() < n)
    throw std::runtime_error(std::string(name) + ": expected at least " + std::to_string(n) +
                             " argument(s), got " + std::to_string(args.size()));
}

const std::string &as_string(const value_t &v, const char *name) {
  if (auto *s = std::get_if<std::string>(&v.v))
    return *s;
  throw std::runtime_error(std::string(name) + ": expected string, got " + v.type_name());
}

int64_t as_int(const value_t &v, const char *name) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return *i;
  throw std::runtime_error(std::string(name) + ": expected integer, got " + v.type_name());
}

double as_number(const value_t &v, const char *name) {
  if (auto *i = std::get_if<int64_t>(&v.v))
    return static_cast<double>(*i);
  if (auto *d = std::get_if<double>(&v.v))
    return *d;
  throw std::runtime_error(std::string(name) + ": expected number, got " + v.type_name());
}

void require_root(const intrinsics_context *ctx, const char *name) {
  if (!ctx->root)
    throw std::runtime_error(std::string(name) + ": no state root bound");
}

void require_sched(const intrinsics_context *ctx, const char *name) {
  if (!ctx->sched)
    throw std::runtime_error(std::string(name) + ": no scheduler bound");
}

// ---------------------------------------------------------------------------
// State tree intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_state_get(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-get");
  require_root(ctx, "state-get");
  auto &path = as_string(args[0], "state-get");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return nil_value;
  return value_t(node->value());
}

value_t intrinsic_state_set(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-set");
  require_root(ctx, "state-set");
  auto &path = as_string(args[0], "state-set");
  auto &val = as_string(args[1], "state-set");
  // operator() creates child nodes as needed
  (*ctx->root)(path).value(val);
  return nil_value;
}

value_t intrinsic_state_children(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-children");
  require_root(ctx, "state-children");
  auto &path = as_string(args[0], "state-children");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return make_list();
  auto names = node->children();
  std::vector<value_t> result;
  result.reserve(names.size());
  for (auto &n : names)
    result.emplace_back(n);
  return make_list(std::move(result));
}

value_t intrinsic_state_exists(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-exists");
  require_root(ctx, "state-exists");
  auto &path = as_string(args[0], "state-exists");
  return value_t(ctx->root->findDescendant(path) != nullptr);
}

value_t intrinsic_state_delete(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-delete");
  require_root(ctx, "state-delete");
  auto &path = as_string(args[0], "state-delete");
  auto *node = ctx->root->findDescendant(path);
  if (node) {
    // Use expiry for immediate deletion
    node->expireAt(boost::posix_time::microsec_clock::universal_time());
    ctx->root->sweepExpired();
  }
  return nil_value;
}

value_t intrinsic_state_data_get(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-data-get");
  require_root(ctx, "state-data-get");
  auto &path = as_string(args[0], "state-data-get");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return nil_value;
  auto d = node->data();
  if (d.empty())
    return nil_value;
  // Wrap the boost::any in a data_object
  auto obj = std::make_shared<data_object>();
  obj->payload = d;
  obj->type_name = d.type().name();
  return value_t(std::move(obj));
}

value_t intrinsic_state_data_set(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-data-set");
  require_root(ctx, "state-data-set");
  auto &path = as_string(args[0], "state-data-set");
  // Store the value_t as boost::any
  (*ctx->root)(path).data(boost::any(args[1]));
  return nil_value;
}

value_t intrinsic_state_root_path(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "state-root-path");
  return value_t(ctx->root_path);
}

value_t intrinsic_state_watch(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-watch");
  require_root(ctx, "state-watch");
  auto &path = as_string(args[0], "state-watch");
  // args[1] is the handler expression (lambda or quoted form)
  value_t handler = args[1];

  // Navigate to the node (create if needed so we can watch it)
  auto *node = &(*ctx->root)(path);

  // Allocate a watch ID
  int watch_id = ctx->proc->next_watch_id++;

  // Store the handler on the scheduler's process (not ctx->proc which may
  // be a separate copy).  This also avoids capturing value_t in the signal
  // lambda, which can cause ABI issues with boost::signals2 on some
  // platforms (e.g. macOS).
  scheduler *sched = ctx->sched;
  int pid = ctx->pid;
  if (sched) {
    sched->register_watch_handler(pid, watch_id, handler);
  } else {
    ctx->proc->watch_handlers[watch_id] = handler;
  }

  // Connect to the node's valueChanged signal.
  // The lambda captures only POD/simple types — no value_t.
  std::string watched_path = path;
  boost::signals2::connection conn;
  if (sched) {
    conn = node->valueChanged.connect([sched, pid, watch_id, watched_path]() {
      sched->queue_watch_event(pid, {watch_id, watched_path, std::string()});
    });
  } else {
    // No scheduler (unit-test path): push directly onto process
    auto proc_weak = std::weak_ptr<process>(ctx->proc);
    conn = node->valueChanged.connect([proc_weak, watch_id, watched_path]() {
      auto proc = proc_weak.lock();
      if (!proc)
        return;
      proc->pending_watch_events.push_back({watch_id, watched_path, std::string()});
    });
  }

  // Store the connection for later disconnection
  auto conn_shared = std::make_shared<boost::signals2::connection>(std::move(conn));
  ctx->watches[watch_id] = {path, handler, [conn_shared]() { conn_shared->disconnect(); }};

  return value_t(static_cast<int64_t>(watch_id));
}

value_t intrinsic_state_unwatch(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-unwatch");
  int64_t watch_id = as_int(args[0], "state-unwatch");
  auto it = ctx->watches.find(static_cast<int>(watch_id));
  if (it == ctx->watches.end())
    return value_t(false);
  it->second.disconnect();
  ctx->watches.erase(it);
  // Remove handler from the scheduler's process (or ctx->proc if no scheduler)
  if (ctx->sched)
    ctx->sched->unregister_watch_handler(ctx->pid, static_cast<int>(watch_id));
  else
    ctx->proc->watch_handlers.erase(static_cast<int>(watch_id));
  return value_t(true);
}

// ---------------------------------------------------------------------------
// Scheduler intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_spawn(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_min(args, 1, "spawn");
  require_sched(ctx, "spawn");
  auto &script = as_string(args[0], "spawn");
  execute_options opts;
  if (args.size() > 1)
    opts.name = as_string(args[1], "spawn");
  if (args.size() > 2)
    opts.priority = static_cast<int>(as_int(args[2], "spawn"));
  opts.uid = ctx->uid;
  int pid = ctx->sched->execute(script, opts);
  return value_t(static_cast<int64_t>(pid));
}

value_t intrinsic_fork(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "fork");
  require_sched(ctx, "fork");
  int child = ctx->sched->fork(ctx->pid);
  return value_t(static_cast<int64_t>(child));
}

value_t intrinsic_self_pid(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "self-pid");
  return value_t(static_cast<int64_t>(ctx->pid));
}

value_t intrinsic_self_uid(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "self-uid");
  return value_t(ctx->uid);
}

value_t intrinsic_kill(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "kill");
  require_sched(ctx, "kill");
  int pid = static_cast<int>(as_int(args[0], "kill"));
  return value_t(ctx->sched->kill(pid));
}

value_t intrinsic_pause(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "pause");
  require_sched(ctx, "pause");
  int pid = static_cast<int>(as_int(args[0], "pause"));
  return value_t(ctx->sched->pause(pid));
}

value_t intrinsic_resume(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "resume");
  require_sched(ctx, "resume");
  int pid = static_cast<int>(as_int(args[0], "resume"));
  return value_t(ctx->sched->resume(pid));
}

value_t intrinsic_ps(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "ps");
  require_sched(ctx, "ps");
  auto procs = ctx->sched->list_processes();
  std::vector<value_t> result;
  result.reserve(procs.size());
  for (auto &pi : procs) {
    // Each process → dict with key fields
    std::vector<std::pair<std::string, value_t>> entries;
    entries.emplace_back("pid", value_t(static_cast<int64_t>(pi.pid)));
    entries.emplace_back("name", value_t(pi.name));
    entries.emplace_back(
        "status", value_t(std::string(pi.status == process_status::ready        ? "ready"
                                      : pi.status == process_status::running    ? "running"
                                      : pi.status == process_status::paused     ? "paused"
                                      : pi.status == process_status::waiting    ? "waiting"
                                      : pi.status == process_status::terminated ? "terminated"
                                      : pi.status == process_status::killed     ? "killed"
                                                                                : "unknown")));
    entries.emplace_back("priority", value_t(static_cast<int64_t>(pi.priority)));
    entries.emplace_back("uid", value_t(pi.uid));
    entries.emplace_back("steps", value_t(static_cast<int64_t>(pi.step_count)));
    result.push_back(make_dict(std::move(entries)));
  }
  return make_list(std::move(result));
}

value_t intrinsic_inspect(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "inspect");
  require_sched(ctx, "inspect");
  int pid = static_cast<int>(as_int(args[0], "inspect"));
  auto info = ctx->sched->get_process_info(pid);
  if (!info)
    return nil_value;
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back("pid", value_t(static_cast<int64_t>(info->pid)));
  entries.emplace_back("name", value_t(info->name));
  entries.emplace_back(
      "status", value_t(std::string(info->status == process_status::ready        ? "ready"
                                    : info->status == process_status::running    ? "running"
                                    : info->status == process_status::paused     ? "paused"
                                    : info->status == process_status::waiting    ? "waiting"
                                    : info->status == process_status::terminated ? "terminated"
                                    : info->status == process_status::killed     ? "killed"
                                                                                 : "unknown")));
  entries.emplace_back("priority", value_t(static_cast<int64_t>(info->priority)));
  entries.emplace_back("uid", value_t(info->uid));
  entries.emplace_back("gid", value_t(info->gid));
  entries.emplace_back("steps", value_t(static_cast<int64_t>(info->step_count)));
  entries.emplace_back("elapsed_time", value_t(info->elapsed_time));
  entries.emplace_back("memory", value_t(static_cast<int64_t>(info->current_memory)));
  entries.emplace_back("peak_memory", value_t(static_cast<int64_t>(info->peak_memory)));
  entries.emplace_back("max_memory", value_t(static_cast<int64_t>(info->max_memory)));
  entries.emplace_back("max_time", value_t(info->max_time));
  entries.emplace_back("messages", value_t(static_cast<int64_t>(info->message_count)));
  entries.emplace_back("max_messages", value_t(static_cast<int64_t>(info->max_messages)));
  entries.emplace_back("message_bytes", value_t(static_cast<int64_t>(info->message_bytes)));
  entries.emplace_back("max_message_bytes", value_t(static_cast<int64_t>(info->max_message_bytes)));
  entries.emplace_back("parent_pid", value_t(static_cast<int64_t>(info->parent_pid)));
  return make_dict(std::move(entries));
}

// ---------------------------------------------------------------------------
// Resource intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_memory_usage(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "memory-usage");
  if (!ctx->tracker)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->tracker->current_bytes(ctx->pid)));
}

value_t intrinsic_memory_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "memory-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_memory));
}

value_t intrinsic_time_elapsed(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "time-elapsed");
  if (!ctx->proc)
    return value_t(0.0);
  return value_t(ctx->proc->elapsed_time());
}

value_t intrinsic_time_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "time-limit");
  if (!ctx->proc)
    return value_t(0.0);
  return value_t(static_cast<double>(ctx->proc->max_time));
}

value_t intrinsic_message_count(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-count");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->message_count));
}

value_t intrinsic_message_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_messages));
}

value_t intrinsic_message_bytes(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-bytes");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->message_bytes));
}

value_t intrinsic_message_bytes_limit(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "message-bytes-limit");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->max_message_bytes));
}

value_t intrinsic_step_count(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "step-count");
  if (!ctx->proc)
    return value_t(int64_t(0));
  return value_t(static_cast<int64_t>(ctx->proc->step_count()));
}

// ---------------------------------------------------------------------------
// System identity intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_cluster_id(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "cluster-id");
  return value_t(ctx->cluster_id);
}

value_t intrinsic_node_id(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "node-id");
  return value_t(ctx->node_id);
}

value_t intrinsic_scheduler_stats(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 0, "scheduler-stats");
  require_sched(ctx, "scheduler-stats");
  auto s = ctx->sched->get_stats();
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back("total", value_t(static_cast<int64_t>(s.total_processes)));
  entries.emplace_back("running", value_t(static_cast<int64_t>(s.running)));
  entries.emplace_back("ready", value_t(static_cast<int64_t>(s.ready)));
  entries.emplace_back("paused", value_t(static_cast<int64_t>(s.paused)));
  entries.emplace_back("terminated", value_t(static_cast<int64_t>(s.terminated)));
  entries.emplace_back("killed", value_t(static_cast<int64_t>(s.killed)));
  entries.emplace_back("steps", value_t(static_cast<int64_t>(s.total_steps)));
  return make_dict(std::move(entries));
}

// ---------------------------------------------------------------------------
// Expiry intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_state_expire(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-expire");
  require_root(ctx, "state-expire");
  auto &path = as_string(args[0], "state-expire");
  double secs = as_number(args[1], "state-expire");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-expire: path not found: " + path);
  node->expireAfter(boost::posix_time::milliseconds(static_cast<int64_t>(secs * 1000.0)));
  return nil_value;
}

value_t intrinsic_state_expire_at(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 2, "state-expire-at");
  require_root(ctx, "state-expire-at");
  auto &path = as_string(args[0], "state-expire-at");
  auto &iso = as_string(args[1], "state-expire-at");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-expire-at: path not found: " + path);
  auto pt = boost::posix_time::time_from_string(iso);
  node->expireAt(pt);
  return nil_value;
}

value_t intrinsic_state_has_expiry(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-has-expiry");
  require_root(ctx, "state-has-expiry");
  auto &path = as_string(args[0], "state-has-expiry");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return false_value;
  return value_t(node->hasExpiry());
}

value_t intrinsic_state_is_expired(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-is-expired");
  require_root(ctx, "state-is-expired");
  auto &path = as_string(args[0], "state-is-expired");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    return false_value;
  return value_t(node->isExpired());
}

value_t intrinsic_state_clear_expiry(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_exact(args, 1, "state-clear-expiry");
  require_root(ctx, "state-clear-expiry");
  auto &path = as_string(args[0], "state-clear-expiry");
  auto *node = ctx->root->findDescendant(path);
  if (!node)
    throw std::runtime_error("state-clear-expiry: path not found: " + path);
  node->clearExpiry();
  return nil_value;
}

value_t intrinsic_state_sweep(intrinsics_context *ctx, std::span<const value_t> args) {
  if (args.size() > 1)
    throw std::runtime_error("state-sweep-expired: expected 0-1 argument(s), got " +
                             std::to_string(args.size()));
  require_root(ctx, "state-sweep-expired");
  cvc::state *target = ctx->root;
  if (args.size() == 1) {
    auto &path = as_string(args[0], "state-sweep-expired");
    target = ctx->root->findDescendant(path);
    if (!target)
      return value_t(int64_t(0));
  }
  return value_t(static_cast<int64_t>(target->sweepExpired()));
}

// ---------------------------------------------------------------------------
// Messaging intrinsics
// ---------------------------------------------------------------------------

value_t intrinsic_msg_send(intrinsics_context *ctx, std::span<const value_t> args) {
  expect_min(args, 2, "msg-send");
  require_root(ctx, "msg-send");
  auto &path = as_string(args[0], "msg-send");
  auto &payload = as_string(args[1], "msg-send");
  std::string content_type = "text/plain";
  if (args.size() > 2)
    content_type = as_string(args[2], "msg-send");
  auto *node = ctx->root->findDescendant(path);
  if (!node) {
    // Create the node so we can send to it
    node = &(*ctx->root)(path);
  }
  auto result = node->sendMessage(payload, content_type);
  if (ctx->proc) {
    ctx->proc->message_count++;
    ctx->proc->message_bytes += payload.size();
  }
  std::vector<std::pair<std::string, value_t>> entries;
  entries.emplace_back(
      "status",
      value_t(std::string(result.status == cvc::state::send_message_result::status_kind::delivered
                              ? "delivered"
                              : "error")));
  entries.emplace_back("path", value_t(result.resolved_path));
  return make_dict(std::move(entries));
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void register_intrinsics(environment_ptr env, intrinsics_context *ctx) {
  auto reg = [&](const std::string &name, auto fn) {
    builtins::register_fn(env, name,
                          [ctx, fn](std::span<const value_t> args) { return fn(ctx, args); });
  };

  // State tree
  reg("state-get", intrinsic_state_get);
  reg("state-set", intrinsic_state_set);
  reg("state-children", intrinsic_state_children);
  reg("state-exists", intrinsic_state_exists);
  reg("state-delete", intrinsic_state_delete);
  reg("state-data-get", intrinsic_state_data_get);
  reg("state-data-set", intrinsic_state_data_set);
  reg("state-root-path", intrinsic_state_root_path);
  reg("state-watch", intrinsic_state_watch);
  reg("state-unwatch", intrinsic_state_unwatch);

  // Scheduler
  reg("spawn", intrinsic_spawn);
  reg("fork", intrinsic_fork);
  reg("self-pid", intrinsic_self_pid);
  reg("self-uid", intrinsic_self_uid);
  reg("kill", intrinsic_kill);
  reg("pause", intrinsic_pause);
  reg("resume", intrinsic_resume);
  reg("ps", intrinsic_ps);
  reg("inspect", intrinsic_inspect);

  // Resource queries
  reg("memory-usage", intrinsic_memory_usage);
  reg("memory-limit", intrinsic_memory_limit);
  reg("time-elapsed", intrinsic_time_elapsed);
  reg("time-limit", intrinsic_time_limit);
  reg("message-count", intrinsic_message_count);
  reg("message-limit", intrinsic_message_limit);
  reg("message-bytes", intrinsic_message_bytes);
  reg("message-bytes-limit", intrinsic_message_bytes_limit);
  reg("step-count", intrinsic_step_count);

  // System identity
  reg("cluster-id", intrinsic_cluster_id);
  reg("node-id", intrinsic_node_id);
  reg("scheduler-stats", intrinsic_scheduler_stats);

  // Expiry
  reg("state-expire", intrinsic_state_expire);
  reg("state-expire-at", intrinsic_state_expire_at);
  reg("state-has-expiry", intrinsic_state_has_expiry);
  reg("state-is-expired", intrinsic_state_is_expired);
  reg("state-clear-expiry", intrinsic_state_clear_expiry);
  reg("state-sweep-expired", intrinsic_state_sweep);

  // Messaging
  reg("msg-send", intrinsic_msg_send);
}

void apply_chroot(intrinsics_context &ctx, cvc::state &tree_root, const std::string &root_path) {
  if (root_path.empty()) {
    ctx.root = &tree_root;
    ctx.root_path.clear();
    return;
  }
  // Navigate or create the subtree node
  ctx.root = &tree_root(root_path);
  ctx.root_path = root_path;
}

} // namespace cvc::state_exec
