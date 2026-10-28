/// @file state_exec_lock_table_test.cpp
/// @brief Directed cases of the lock manager's table on one home, no scheduler (suite LockTable;
///        locks plan L1.2). Compiled into state_exec_locks_model_test beside the model check.

#include <cvc/state/state_exec/detail/lock_table.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

TEST(LockTable, Constructs) {
  detail::lock_table t(detail::lock_table::options{});
  EXPECT_TRUE(t.empty());
}
