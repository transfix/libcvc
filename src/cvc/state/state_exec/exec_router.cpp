// Stage 2's router (docs/STATE_EXEC_THREADED.md 4.6; S2.md S2b, S2-Q10): group handles, channel
// homes by registered prefix, and the host's post (exec_router.h has the rules).
#include <algorithm>
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/exec_router.h>
#include <mutex>
#include <shared_mutex>
#include <utility>

namespace cvc::state_exec {

namespace detail {
// What a group_ref points at. A post holds `mu` shared while it pushes into the scheduler's
// ingress, and close() takes it exclusively, so once close() returns no post touches the scheduler.
struct route_target {
  mutable std::shared_mutex mu;
  async_scheduler *sched = nullptr; // null once closed
  exec_group *group = nullptr;      // null for group 0
  bool group0 = false;
  std::uint64_t id = 0;
};
} // namespace detail

// --- group_ref ---

group_ref group_ref::make(async_scheduler &sched, exec_group *group) {
  group_ref r;
  r.t_ = std::make_shared<detail::route_target>();
  r.t_->sched = &sched;
  r.t_->group = group;
  r.t_->group0 = group == nullptr;
  r.t_->id = group ? group->pool_link().seq : 0;
  return r;
}

void group_ref::close() const noexcept {
  if (!t_)
    return;
  std::unique_lock<std::shared_mutex> lock(t_->mu);
  t_->sched = nullptr;
}

bool group_ref::closed() const noexcept {
  if (!t_)
    return true;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched == nullptr;
}

bool group_ref::is_group0() const noexcept { return t_ && t_->group0; }

exec_group *group_ref::group() const noexcept { return t_ ? t_->group : nullptr; }

std::uint64_t group_ref::id() const noexcept { return t_ ? t_->id : 0; }

bool group_ref::post(detail::route_entry e) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_route(std::move(e));
}

bool group_ref::post_message(const std::string &key, const value_t &value) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_message(key, value);
}

bool group_ref::post_control(int pid, control_cmd cmd) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  return t_->sched && t_->sched->post_control(pid, std::move(cmd));
}

bool group_ref::visit(const std::function<void(async_scheduler &)> &fn) const {
  if (!t_)
    return false;
  std::shared_lock<std::shared_mutex> lock(t_->mu);
  if (!t_->sched)
    return false;
  fn(*t_->sched);
  return true;
}

// --- exec_router ---

struct exec_router::impl {
  mutable std::shared_mutex mu;
  struct registration {
    std::string prefix;
    group_ref group;
  };
  std::vector<registration> regs; // every prefix but group 0's "", in registration order
  group_ref group0;               // the default home, once group 0 registered ""
  std::atomic<std::uint64_t> epoch{0};
  std::atomic<int> others{0};          // registrations of groups other than group 0
  std::atomic<long> remote_waiters{0}; // note_remote_waiters
  mutable std::atomic<std::uint64_t> consulted{0};
};

exec_router::exec_router(cvc::app *app) : app_(app), impl_(std::make_unique<impl>()) {}

exec_router::~exec_router() = default;

void exec_router::register_prefix(const group_ref &group, std::string prefix) {
  if (!group)
    return;
  impl &m = *impl_;
  {
    std::unique_lock<std::shared_mutex> lock(m.mu);
    if (prefix.empty() && group.is_group0()) {
      m.group0 = group;
    } else {
      m.regs.push_back({std::move(prefix), group});
      if (!group.is_group0())
        m.others.fetch_add(1, std::memory_order_relaxed);
    }
    m.epoch.fetch_add(1, std::memory_order_release);
  }
}

void exec_router::unregister(const group_ref &group) {
  if (!group)
    return;
  impl &m = *impl_;
  std::unique_lock<std::shared_mutex> lock(m.mu);
  const auto end = std::remove_if(m.regs.begin(), m.regs.end(),
                                  [&](const impl::registration &r) { return r.group == group; });
  const auto removed = static_cast<int>(m.regs.end() - end);
  m.regs.erase(end, m.regs.end());
  if (!group.is_group0())
    m.others.fetch_sub(removed, std::memory_order_relaxed);
  if (m.group0 == group)
    m.group0 = group_ref();
  m.epoch.fetch_add(1, std::memory_order_release);
}

group_ref exec_router::home_of(std::string_view key) const {
  impl &m = *impl_;
  m.consulted.fetch_add(1, std::memory_order_relaxed);
  for (int attempt = 0; attempt < 2; ++attempt) {
    {
      std::shared_lock<std::shared_mutex> lock(m.mu);
      const impl::registration *best = nullptr;
      for (const impl::registration &r : m.regs) {
        const std::string &p = r.prefix;
        const bool covers = key.size() >= p.size() && key.compare(0, p.size(), p) == 0 &&
                            (key.size() == p.size() || p.empty() || key[p.size()] == '.');
        if (covers && (!best || p.size() > best->prefix.size()))
          best = &r;
      }
      if (best)
        return best->group;
      if (m.group0 || !app_)
        return m.group0;
    }
    // Group 0 is not built yet: build it (it registers "" here), outside the router's lock.
    if (attempt == 0)
      app_->exec_scheduler();
  }
  std::shared_lock<std::shared_mutex> lock(m.mu);
  return m.group0;
}

bool exec_router::active() const noexcept {
  return impl_->others.load(std::memory_order_acquire) > 0 ||
         impl_->remote_waiters.load(std::memory_order_acquire) > 0;
}

std::uint64_t exec_router::epoch() const noexcept {
  return impl_->epoch.load(std::memory_order_acquire);
}

std::uint64_t exec_router::consulted() const noexcept {
  return impl_->consulted.load(std::memory_order_relaxed);
}

void exec_router::note_remote_waiters(int delta) noexcept {
  impl_->remote_waiters.fetch_add(delta, std::memory_order_acq_rel);
}

// S2b.0 stub (R builds it in S2b.4): group 0's post, as a host's post to app::exec_scheduler()
// today.
bool exec_router::post_message(const std::string &key, const value_t &value) {
  if (!app_)
    return false;
  return app_->exec_scheduler().post_message(key, value);
}

} // namespace cvc::state_exec
