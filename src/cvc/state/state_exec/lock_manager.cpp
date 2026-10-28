// The state_exec lock manager (docs/STATE_EXEC_LOCKS_PROTOCOL.md sections 2-4).
//
// Interface commit of phase L1: the operations are declared (detail/lock_table.h) and stubbed;
// L1.2 implements them.

#include <cstdlib>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <stdexcept>

namespace cvc::state_exec {

namespace detail {

namespace {
std::atomic<const lock_test_hooks *> g_lock_test_hooks{nullptr};

[[noreturn]] void not_built() { throw std::logic_error("lock_table: not built yet (phase L1.2)"); }
} // namespace

void set_lock_test_hooks(const lock_test_hooks *hooks) {
  g_lock_test_hooks.store(hooks, std::memory_order_release);
}
const lock_test_hooks *lock_test_hooks_get() {
  return g_lock_test_hooks.load(std::memory_order_acquire);
}

struct lock_table::impl {};

lock_table::lock_table(options opts) : opts_(opts), impl_(std::make_unique<impl>()) {}
lock_table::~lock_table() = default;

home_id lock_table::attach_home(bool, std::shared_ptr<lock_port>,
                                std::shared_ptr<std::atomic<std::int64_t>>, lock_notes &) {
  not_built();
}
std::optional<grant> lock_table::try_acquire(lock_pid, rec_info, lock_request &,
                                             const cache_delta &) {
  not_built();
}
block_result lock_table::block(lock_pid, rec_info, lock_request &, const cache_delta &,
                               lock_notes &) {
  not_built();
}
lock_outcome lock_table::resume(lock_pid, std::uint64_t) { not_built(); }
void lock_table::start_body(lock_pid, std::uint64_t) { not_built(); }
std::optional<ticket_state> lock_table::cancel(lock_pid, std::uint64_t, lock_notes &) {
  not_built();
}
void lock_table::release(lock_pid, std::uint64_t, lock_notes &) { not_built(); }
void lock_table::end(lock_pid, bool, lock_notes &) { not_built(); }
void lock_table::detach_home(home_id, lock_notes &) { not_built(); }
void lock_table::set_base(lock_pid, int, lock_notes &) { not_built(); }
int lock_table::read_inh(lock_pid) { not_built(); }
lock_query_result lock_table::query(std::string_view, std::optional<home_id>,
                                    std::string_view) const {
  not_built();
}
void lock_table::ref_nodes(home_id, std::span<key_ref>) { not_built(); }
void lock_table::unref_nodes(home_id, std::span<lock_node *const>) { not_built(); }
std::span<lock_node *const> lock_table::hold_keys(std::uint64_t) const { not_built(); }
lock_stats lock_table::stats() const { return {}; }
void lock_table::check() const {}
bool lock_table::empty() const { return true; }
std::string lock_table::canonical() const { not_built(); }

} // namespace detail

namespace {
bool check_forced_by_environment() {
  static const bool forced = [] {
    const char *v = std::getenv("CVC_STATE_EXEC_CHECK");
    return v && *v && *v != '0';
  }();
  return forced;
}
} // namespace

lock_manager::lock_manager() : lock_manager(options{}) {}

lock_manager::lock_manager(options opts) : opts_(opts) {
  if (opts_.threading != lock_threading::single)
    throw std::invalid_argument("lock_manager: shared mode is phase L3");
  detail::lock_table::options t;
  t.inheritance = opts_.inheritance;
  t.self_check = opts_.self_check || check_forced_by_environment();
  t.coalesce_relevels = opts_.coalesce_relevels;
  table_ = std::make_unique<detail::lock_table>(t);
}

lock_manager::~lock_manager() = default;

lock_query_result lock_manager::query(std::string_view abs_prefix, const detail::lock_home *,
                                      std::string_view owner) const {
  // The viewer's home id comes from lock_home (L1.5); until then no pid is shown.
  return table_->query(abs_prefix, std::nullopt, owner);
}

lock_stats lock_manager::stats() const { return table_->stats(); }
lock_threading lock_manager::threading() const { return opts_.threading; }
const lock_manager::options &lock_manager::opts() const { return opts_; }
void lock_manager::check() const { table_->check(); }

} // namespace cvc::state_exec
