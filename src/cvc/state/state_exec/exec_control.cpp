// Stage 2's process control across groups (docs/STATE_EXEC_THREADED.md 4.5; S2.md S2b,
// S2-Q7..S2-Q9): the pid directory, the owner rule's cross-group half, and published info
// (exec_router.h has the rules).
#include <algorithm>
#include <cassert>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/detail/priority.h> // floor_of
#include <cvc/state/state_exec/exec_router.h>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <utility>

namespace cvc::state_exec {

// --- exec_directory (S2b.2; td 4.5: 64 shards, each a mutex and a map) ---

namespace {
constexpr std::size_t kDirectoryShards = 64;
std::size_t shard_of(int pid) noexcept {
  return static_cast<std::size_t>(static_cast<unsigned>(pid)) % kDirectoryShards;
}
} // namespace

struct exec_directory::shard {
  mutable std::mutex mu;
  std::unordered_map<int, directory_entry> entries;
};

exec_directory::exec_directory() : shards_(std::make_unique<shard[]>(kDirectoryShards)) {}

exec_directory::~exec_directory() = default;

void exec_directory::add(int pid, const group_ref &group, const std::string &owner) {
  shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  // Pids are unique in an app and an entry goes at its reap, so one already here is a live pid
  // issued twice (a pid source below a live pid, which set_pid_source refuses).
  assert(s.entries.find(pid) == s.entries.end() && "exec_directory: add replaces a live entry");
  s.entries.insert_or_assign(pid, directory_entry{pid, group, owner});
}

void exec_directory::remove(int pid) noexcept {
  shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  s.entries.erase(pid);
}

std::optional<directory_entry> exec_directory::find(int pid) const {
  const shard &s = shards_[shard_of(pid)];
  std::lock_guard<std::mutex> lock(s.mu);
  const auto it = s.entries.find(pid);
  if (it == s.entries.end())
    return std::nullopt;
  return it->second;
}

std::vector<group_ref> exec_directory::groups_of(const std::string &owner) const {
  // Each group once, in group id order (creation order; group 0 first), whatever order the
  // shards' maps hold their pids in: pool.kill_owner posts in that order.
  std::vector<group_ref> out;
  for (std::size_t i = 0; i < kDirectoryShards; ++i) {
    const shard &s = shards_[i];
    std::lock_guard<std::mutex> lock(s.mu);
    for (const auto &[pid, e] : s.entries)
      if (e.owner == owner && e.group && std::find(out.begin(), out.end(), e.group) == out.end())
        out.push_back(e.group);
  }
  std::stable_sort(out.begin(), out.end(),
                   [](const group_ref &a, const group_ref &b) { return a.id() < b.id(); });
  return out;
}

std::size_t exec_directory::size() const {
  std::size_t n = 0;
  for (std::size_t i = 0; i < kDirectoryShards; ++i) {
    std::lock_guard<std::mutex> lock(shards_[i].mu);
    n += shards_[i].entries.size();
  }
  return n;
}

// --- The router's control entry points (S2b.5) and info (S2b.3) ---

bool exec_router::post_control(int pid, control_cmd cmd) {
  (void)pid;
  (void)cmd;
  return false;
}

std::optional<published_process> exec_router::info(int pid) const {
  (void)pid;
  return std::nullopt;
}

// --- async_scheduler: the owner rule (S2b.5) ---

bool async_scheduler::control_as(const std::string &caller_owner, int pid, control_cmd::kind what) {
  return scheduler_base::control_as(caller_owner, pid, what);
}

// --- async_scheduler: published info (S2b.3; S2-Q9) ---

void async_scheduler::publish_info() {}

async_scheduler::published_summary async_scheduler::published() const noexcept { return {}; }

std::optional<published_process> async_scheduler::published_info(int pid) const {
  (void)pid;
  return std::nullopt;
}

std::vector<published_process> async_scheduler::published_rows() const { return {}; }

} // namespace cvc::state_exec
