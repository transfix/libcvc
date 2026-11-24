// Stage 2's process control across groups (docs/STATE_EXEC_THREADED.md 4.5; S2.md S2b,
// S2-Q7..S2-Q9): the pid directory, the owner rule's cross-group half, and published info
// (exec_router.h has the rules). S2b.0 stubs: the directory finds nothing, nothing is published,
// control is local.
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_router.h>

namespace cvc::state_exec {

// --- exec_directory (S2b.2) ---

struct exec_directory::shard {};

exec_directory::exec_directory() = default;
exec_directory::~exec_directory() = default;

void exec_directory::add(int pid, const group_ref &group, const std::string &owner) {
  (void)pid;
  (void)group;
  (void)owner;
}

void exec_directory::remove(int pid) noexcept { (void)pid; }

std::optional<directory_entry> exec_directory::find(int pid) const {
  (void)pid;
  return std::nullopt;
}

std::vector<group_ref> exec_directory::groups_of(const std::string &owner) const {
  (void)owner;
  return {};
}

std::size_t exec_directory::size() const { return 0; }

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

// --- async_scheduler: the owner rule (S2b.5) and published info (S2b.3) ---

bool async_scheduler::control_as(const std::string &caller_owner, int pid, control_cmd::kind what) {
  return scheduler_base::control_as(caller_owner, pid, what);
}

void async_scheduler::publish_info() {}

async_scheduler::published_summary async_scheduler::published() const noexcept { return {}; }

std::optional<published_process> async_scheduler::published_info(int pid) const {
  (void)pid;
  return std::nullopt;
}

std::vector<published_process> async_scheduler::published_rows() const { return {}; }

} // namespace cvc::state_exec
