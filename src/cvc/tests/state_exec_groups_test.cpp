// Threaded execution, Stage 1 (docs/STATE_EXEC_THREADED.md 7, 8.2): execution groups and the
// app's exec_pool, the Groups suite (S1.md section 4: the GR, FF, RX, RB, AS, BF, OC, DT, GL, PO
// and PA cases). The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and require each
// case's line. No case name contains "Stress": run.sh filters those out.

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/exec_pool.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exec_group.h>
#include <cvc/state/state_exec/heap.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::state_exec;

// Bare evaluators and bare schedulers track into the main thread's own registry, and the cases are
// too short for a collection to be due: once every test is done, one collection leaves
// LeakSanitizer only real leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

namespace {

// Helpers shared by the cases go here.

} // namespace
