// The lock natives (docs/STATE_EXEC_LOCKS.md 4.1): the with-lock binding, locks-held, lock-query
// and %lock-wait, and the resolution of a lock key as a program writes it.

#include <cctype>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/stackless_evaluator.h>

namespace cvc::state_exec::detail {

namespace {

thread_local const lock_enter_token *t_lock_enter = nullptr;

const char *mode_name(lock_mode m) { return m == lock_mode::exclusive ? "exclusive" : "shared"; }

std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front())))
    s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back())))
    s.remove_suffix(1);
  return s;
}

// A key's segments, trimmed, empties dropped, joined by '.' (state.cpp's path rule).
std::string normalize(std::string_view w, const char *who) {
  std::string out;
  while (!w.empty()) {
    const std::size_t dot = w.find('.');
    const std::string_view seg = trim(w.substr(0, dot));
    w = dot == std::string_view::npos ? std::string_view() : w.substr(dot + 1);
    if (seg.empty())
      continue;
    if (seg.find('#') != std::string_view::npos)
      throw dsl_error("ValueError",
                      std::string(who) + ": a lock key has no '#': '" + std::string(seg) + "'");
    if (!out.empty())
      out += '.';
    out += seg;
  }
  return out;
}

// A '/' key is app-global: allowed exactly where a '/' channel is (gap G-15). A policy that
// enforces allows only a declared global or a key under one.
void check_global(const intrinsics_context &ctx, const std::string &key, const char *who) {
  if (!ctx.channels || !ctx.channels->enforce)
    return;
  for (const std::string &g : ctx.channels->global) {
    const std::string gn = normalize(g, who);
    if (!gn.empty() && key_prefix(gn, key))
      return;
  }
  throw dsl_error("PermissionError",
                  std::string(who) + ": the app-global key '/" + key +
                      "' is not under a declared global (declare it in the channels: block)",
                  "dsl", {{"key", value_t("/" + key)}});
}

const std::string &key_text(const value_t &v, const char *who) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  throw dsl_error("ValueError",
                  std::string(who) + ": a lock key is a string, got " + v.type_name());
}

// shown(keys) (protocol 5.10): a string for one key, a list for several.
template <class Keys, class Text>
value_t shown_list(const Keys &keys, Text text, std::string_view root) {
  if (keys.size() == 1)
    return value_t(lock_home::show(text(keys[0]), root));
  std::vector<value_t> out;
  for (const auto &k : keys)
    out.emplace_back(lock_home::show(text(k), root));
  return make_list(std::move(out));
}

process *stepping(const intrinsics_context *ctx) {
  if (!ctx || !ctx->sched || ctx->sched->current_pid() < 0)
    return nullptr;
  return ctx->sched->current_process().get();
}

[[noreturn]] void lock_error(const std::string &reason, const std::string &message) {
  throw dsl_error("LockError", message, "dsl", {{"reason", value_t(reason)}});
}

value_t make_lock_wait_handle() {
  auto obj = std::make_shared<data_object>();
  obj->type_name = kLockWaitType;
  return value_t(std::move(obj));
}

} // namespace

const lock_ops *lock_ops_of(const value_t &v) {
  const auto *obj = std::get_if<data_object_ptr>(&v.v);
  if (!obj || !*obj || (*obj)->type_name != kLockOpsType)
    return nullptr;
  return boost::any_cast<lock_ops>(&(*obj)->payload);
}

value_t make_lock_ops(intrinsics_context *ctx, std::shared_ptr<void> keep) {
  auto obj = std::make_shared<data_object>();
  obj->type_name = kLockOpsType;
  obj->payload = lock_ops{ctx, std::move(keep)};
  return value_t(std::move(obj));
}

const lock_enter_token *lock_enter_token_get() { return t_lock_enter; }

lock_enter_scope::lock_enter_scope(const lock_enter_token *token) : outer_(t_lock_enter) {
  t_lock_enter = token;
}

lock_enter_scope::~lock_enter_scope() { t_lock_enter = outer_; }

std::string resolve_lock_key(const intrinsics_context &ctx, std::string_view written,
                             const char *who) {
  std::string_view w = trim(written);
  const bool global = !w.empty() && w.front() == '/';
  if (global)
    w.remove_prefix(1);
  std::string key = normalize(w, who);
  if (key.empty())
    throw dsl_error("ValueError",
                    std::string(who) + ": an empty lock key ('" + std::string(written) + "')");
  if (global) {
    check_global(ctx, key, who);
    return key;
  }
  if (ctx.root_path.empty())
    return key;
  std::string abs = ctx.root_path;
  abs += '.';
  abs += key;
  return abs;
}

void build_request(home_request &req, lock_home &home, const intrinsics_context &ctx,
                   const value_t &key, lock_mode mode) {
  req.mode = mode;
  req.keys.clear();
  const std::uint32_t scope = home.scope_id(ctx.root_path);
  const auto add = [&](const value_t &k) {
    home_request::key rk;
    rk.scope = scope;
    rk.written = key_text(k, "with-lock");
    rk.node = home.lookup(scope, rk.written);
    if (!rk.node)
      rk.absolute = resolve_lock_key(ctx, rk.written, "with-lock");
    req.keys.push_back(std::move(rk));
  };
  if (const auto *l = std::get_if<list_ptr>(&key.v)) {
    if (!*l || (*l)->empty())
      throw dsl_error("ValueError", "with-lock: an empty list of keys");
    for (const value_t &k : **l)
      add(k);
  } else {
    add(key);
  }
  canonicalize(req);
}

value_t lock_wait_call(const lock_ops &ops, std::span<const value_t> args) {
  if (!can_await())
    lock_error("cannot wait here", "with-lock: cannot wait here");
  const lock_enter_token *token = lock_enter_token_get();
  process *p = stepping(ops.ctx);
  if (!token || !token->state || !p || token->pid != p->pid)
    lock_error("no form", "%lock-wait: only with-lock waits for a lock");
  if (args.size() != 3)
    lock_error("no form", "%lock-wait: expected the keys, the mode and the timeout");
  const lock_mode mode =
      std::get_if<std::string>(&args[1].v) && std::get<std::string>(args[1].v) == "shared"
          ? lock_mode::shared
          : lock_mode::exclusive;
  std::optional<double> timeout;
  if (const auto *i = std::get_if<int64_t>(&args[2].v))
    timeout = static_cast<double>(*i);
  else if (const auto *d = std::get_if<double>(&args[2].v))
    timeout = *d;
  lock_home &home = *ops.ctx->sched->locks();
  home.begin_request();
  home_request req;
  build_request(req, home, *ops.ctx, args[0], mode); // as written: the restoring chroot applies
  const std::optional<frame_outcome> o = home.wait(*p, req, timeout, ops.ctx->root_path);
  if (!o) {
    request_await(
        {.channel = std::string(), .function = "with-lock", .handle = make_lock_wait_handle()});
    return nil_value;
  }
  // Granted or failed at once: the lock_enter frame keeps the outcome, and its next step acts on
  // it (protocol 5.2).
  eval_frame &f = token->state->stack[token->frame];
  f.extra_vals[static_cast<std::size_t>(lock_frame_slot::wait_or_outcome)] = outcome_value(*o);
  return nil_value;
}

void register_lock_intrinsics(environment_ptr env, intrinsics_context *ctx) {
  env->set("with-lock", make_lock_ops(ctx));

  builtins::register_fn(env, "locks-held", [ctx](std::span<const value_t> args) -> value_t {
    if (!args.empty())
      throw dsl_error("TypeError", "locks-held: expected no arguments");
    const process *p = stepping(ctx);
    std::vector<value_t> out;
    if (p && p->locks)
      for (const mhold &h : p->locks->holds)
        out.push_back(make_dict(
            {{"key", shown_list(
                         h.keys, [](const lock_node *n) { return std::string_view(n->key); },
                         ctx->root_path)},
             {"mode", value_t(std::string(mode_name(h.mode)))},
             {"count", value_t(static_cast<int64_t>(h.count))},
             {"generation", value_t(static_cast<int64_t>(h.gen))}}));
    return make_list(std::move(out));
  });

  builtins::register_fn(env, "lock-query", [ctx](std::span<const value_t> args) -> value_t {
    if (args.size() > 1)
      throw dsl_error("TypeError", "lock-query: expected at most a prefix");
    if (!ctx || !ctx->sched)
      throw std::runtime_error("lock-query: no scheduler bound");
    std::string abs = ctx->root_path;
    if (!args.empty() && !args[0].is_nil()) {
      const std::string &written = key_text(args[0], "lock-query");
      if (!trim(written).empty())
        abs = resolve_lock_key(*ctx, written, "lock-query");
    }
    lock_home &home = *ctx->sched->locks();
    const process *p = stepping(ctx);
    const lock_query_result r =
        home.manager().query(abs, &home, p ? std::string_view(p->owner) : std::string_view());
    std::vector<value_t> held;
    for (const lock_info &i : r.held) {
      std::vector<value_t> pids;
      for (int pid : i.pids)
        pids.emplace_back(static_cast<int64_t>(pid));
      held.push_back(make_dict(
          {{"key", value_t(lock_home::show(i.key, ctx->root_path))},
           {"mode", value_t(std::string(mode_name(i.mode)))},
           {"pids", make_list(std::move(pids))},
           {"count", value_t(static_cast<int64_t>(i.count))},
           {"generation", i.generation ? value_t(static_cast<int64_t>(*i.generation)) : nil_value},
           {"abandoned", value_t(i.abandoned)}}));
    }
    std::vector<value_t> waiting;
    for (const lock_waiter_info &w : r.waiting)
      waiting.push_back(
          make_dict({{"key", shown_list(
                                 w.keys, [](const std::string &k) { return std::string_view(k); },
                                 ctx->root_path)},
                     {"mode", value_t(std::string(mode_name(w.mode)))},
                     {"pid", w.pid ? value_t(static_cast<int64_t>(*w.pid)) : nil_value}}));
    return make_dict(
        {{"held", make_list(std::move(held))}, {"waiting", make_list(std::move(waiting))}});
  });

  // Called by a program directly, with no lock form's token, it refuses (LW-2).
  builtins::register_fn(env, "%lock-wait", [ctx](std::span<const value_t> args) -> value_t {
    return lock_wait_call(lock_ops{ctx, nullptr}, args);
  });
}

} // namespace cvc::state_exec::detail
