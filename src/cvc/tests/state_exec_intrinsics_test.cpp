/// @file state_exec_intrinsics_test.cpp
/// @brief Tests for Phase 5: Intrinsics, Resource Policy, Stdlib.

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/evaluator.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/resource_policy.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/stdlib.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::state_exec;

// ===========================================================================
// IntrinsicsContextTest — verify intrinsics_context defaults
// ===========================================================================

class IntrinsicsContextTest : public ::testing::Test {};

TEST_F(IntrinsicsContextTest, DefaultValues) {
  intrinsics_context ctx;
  EXPECT_EQ(ctx.sched, nullptr);
  EXPECT_EQ(ctx.root, nullptr);
  EXPECT_EQ(ctx.tracker, nullptr);
  EXPECT_EQ(ctx.proc, nullptr);
  EXPECT_EQ(ctx.pid, -1);
  EXPECT_TRUE(ctx.uid.empty());
  EXPECT_TRUE(ctx.cluster_id.empty());
  EXPECT_TRUE(ctx.node_id.empty());
}

// ===========================================================================
// StateTreeIntrinsicsTest — state-get, state-set, state-exists, etc.
// ===========================================================================

class StateTreeIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::ready;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "test-user";
    ictx.cluster_id = "cluster-1";
    ictx.node_id = "node-1";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    EXPECT_NE(fn, nullptr) << "Not a function: " << name;
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StateTreeIntrinsicsTest, StateSetAndGet) {
  call("state-set", {std::string("test.key"), std::string("hello")});
  auto result = call("state-get", {std::string("test.key")});
  ASSERT_TRUE(std::holds_alternative<std::string>(result.v));
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StateTreeIntrinsicsTest, StateGetNonexistent) {
  auto result = call("state-get", {std::string("nonexistent.path")});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(StateTreeIntrinsicsTest, StateExists) {
  call("state-set", {std::string("exists.test"), std::string("val")});
  auto yes = call("state-exists", {std::string("exists.test")});
  auto no = call("state-exists", {std::string("nope.path")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StateTreeIntrinsicsTest, StateChildren) {
  call("state-set", {std::string("parent.child1"), std::string("a")});
  call("state-set", {std::string("parent.child2"), std::string("b")});
  auto result = call("state-children", {std::string("parent")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_GE((*lst)->size(), 2u);
}

TEST_F(StateTreeIntrinsicsTest, StateChildrenNonexistent) {
  auto result = call("state-children", {std::string("no.such.path")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_TRUE((*lst)->empty());
}

TEST_F(StateTreeIntrinsicsTest, StateDelete) {
  call("state-set", {std::string("del.target"), std::string("bye")});
  EXPECT_TRUE(std::get<bool>(call("state-exists", {std::string("del.target")}).v));
  call("state-delete", {std::string("del.target")});
  EXPECT_FALSE(std::get<bool>(call("state-exists", {std::string("del.target")}).v));
}

TEST_F(StateTreeIntrinsicsTest, StateDeleteNonexistent) {
  // Should not throw
  EXPECT_NO_THROW(call("state-delete", {std::string("nothing.here")}));
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetSet) {
  call("state-data-set", {std::string("data.node"), std::string("data-val")});
  auto result = call("state-data-get", {std::string("data.node")});
  EXPECT_FALSE(result.is_nil());
  auto *obj = std::get_if<data_object_ptr>(&result.v);
  ASSERT_NE(obj, nullptr);
}

TEST_F(StateTreeIntrinsicsTest, StateDataGetNonexistent) {
  auto result = call("state-data-get", {std::string("no.data")});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(StateTreeIntrinsicsTest, StateGetWrongArgCount) {
  EXPECT_THROW(call("state-get", {}), std::runtime_error);
  EXPECT_THROW(call("state-get", {std::string("a"), std::string("b")}), std::runtime_error);
}

TEST_F(StateTreeIntrinsicsTest, StateSetWrongArgCount) {
  EXPECT_THROW(call("state-set", {std::string("a")}), std::runtime_error);
}

// ===========================================================================
// SchedulerIntrinsicsTest
// ===========================================================================

class SchedulerIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "admin";
    ictx.cluster_id = "c1";
    ictx.node_id = "n1";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args = {}) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    EXPECT_NE(fn, nullptr) << "Not a function: " << name;
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(SchedulerIntrinsicsTest, SelfPid) {
  auto result = call("self-pid");
  EXPECT_EQ(std::get<int64_t>(result.v), 1);
}

TEST_F(SchedulerIntrinsicsTest, SelfUid) {
  auto result = call("self-uid");
  EXPECT_EQ(std::get<std::string>(result.v), "admin");
}

TEST_F(SchedulerIntrinsicsTest, ClusterId) {
  auto result = call("cluster-id");
  EXPECT_EQ(std::get<std::string>(result.v), "c1");
}

TEST_F(SchedulerIntrinsicsTest, NodeId) {
  auto result = call("node-id");
  EXPECT_EQ(std::get<std::string>(result.v), "n1");
}

TEST_F(SchedulerIntrinsicsTest, SpawnReturnsValidPid) {
  auto result = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(result.v);
  EXPECT_GT(pid, 0);
}

TEST_F(SchedulerIntrinsicsTest, SpawnWithName) {
  auto result = call("spawn", {std::string("(+ 1 2)"), std::string("adder")});
  auto pid = std::get<int64_t>(result.v);
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "adder");
}

TEST_F(SchedulerIntrinsicsTest, KillProcess) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto result = call("kill", {pid});
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(SchedulerIntrinsicsTest, KillNonexistent) {
  auto result = call("kill", {int64_t(9999)});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(SchedulerIntrinsicsTest, PauseAndResume) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto pause_result = call("pause", {pid});
  EXPECT_TRUE(std::get<bool>(pause_result.v));
  auto resume_result = call("resume", {pid});
  EXPECT_TRUE(std::get<bool>(resume_result.v));
}

TEST_F(SchedulerIntrinsicsTest, PsList) {
  call("spawn", {std::string("(+ 1 2)")});
  auto result = call("ps");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_GE((*lst)->size(), 1u);
}

TEST_F(SchedulerIntrinsicsTest, InspectProcess) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)"), std::string("inspectable")});
  auto pid = std::get<int64_t>(pid_val.v);
  auto result = call("inspect", {pid});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  // Verify has expected keys
  bool found_pid = false, found_name = false;
  for (auto &[k, v] : **d) {
    if (k == "pid") {
      EXPECT_EQ(std::get<int64_t>(v.v), pid);
      found_pid = true;
    }
    if (k == "name") {
      EXPECT_EQ(std::get<std::string>(v.v), "inspectable");
      found_name = true;
    }
  }
  EXPECT_TRUE(found_pid);
  EXPECT_TRUE(found_name);
}

TEST_F(SchedulerIntrinsicsTest, InspectNonexistent) {
  auto result = call("inspect", {int64_t(9999)});
  EXPECT_TRUE(result.is_nil());
}

TEST_F(SchedulerIntrinsicsTest, SchedulerStats) {
  call("spawn", {std::string("(+ 1 2)")});
  auto result = call("scheduler-stats");
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  bool found_total = false;
  for (auto &[k, v] : **d) {
    if (k == "total") {
      EXPECT_GE(std::get<int64_t>(v.v), int64_t(1));
      found_total = true;
    }
  }
  EXPECT_TRUE(found_total);
}

// ===========================================================================
// ResourceIntrinsicsTest
// ===========================================================================

class ResourceIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    proc->max_memory = 1024 * 1024;
    proc->max_time = 60;
    proc->max_messages = 100;
    proc->message_count = 5;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args = {}) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(ResourceIntrinsicsTest, MemoryUsage) {
  auto result = call("memory-usage");
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ResourceIntrinsicsTest, MemoryLimit) {
  auto result = call("memory-limit");
  EXPECT_EQ(std::get<int64_t>(result.v), 1024 * 1024);
}

TEST_F(ResourceIntrinsicsTest, TimeElapsed) {
  auto result = call("time-elapsed");
  EXPECT_GE(std::get<double>(result.v), 0.0);
}

TEST_F(ResourceIntrinsicsTest, TimeLimit) {
  auto result = call("time-limit");
  EXPECT_EQ(std::get<double>(result.v), 60.0);
}

TEST_F(ResourceIntrinsicsTest, MessageCount) {
  auto result = call("message-count");
  EXPECT_EQ(std::get<int64_t>(result.v), 5);
}

TEST_F(ResourceIntrinsicsTest, MessageLimit) {
  auto result = call("message-limit");
  EXPECT_EQ(std::get<int64_t>(result.v), 100);
}

TEST_F(ResourceIntrinsicsTest, StepCount) {
  auto result = call("step-count");
  EXPECT_GE(std::get<int64_t>(result.v), 0);
}

TEST_F(ResourceIntrinsicsTest, NullProcess) {
  // With no process bound, resource intrinsics return 0
  intrinsics_context ctx2;
  ctx2.sched = &sched;
  auto env2 = builtins::make_default_environment();
  register_intrinsics(env2, &ctx2);
  auto *fn_val = env2->lookup("memory-limit");
  auto *fn = std::get_if<native_fn>(&fn_val->v);
  auto result = (*fn)(std::span<const value_t>{});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

// ===========================================================================
// ExpiryIntrinsicsTest
// ===========================================================================

class ExpiryIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.sched = &sched;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(ExpiryIntrinsicsTest, HasExpiryFalseByDefault) {
  call("state-set", {std::string("exp.node"), std::string("val")});
  auto result = call("state-has-expiry", {std::string("exp.node")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, SetExpiry) {
  call("state-set", {std::string("exp.node2"), std::string("val")});
  call("state-expire", {std::string("exp.node2"), 3600.0}); // 1hr
  auto result = call("state-has-expiry", {std::string("exp.node2")});
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, ClearExpiry) {
  call("state-set", {std::string("exp.clear"), std::string("val")});
  call("state-expire", {std::string("exp.clear"), 100.0});
  EXPECT_TRUE(std::get<bool>(call("state-has-expiry", {std::string("exp.clear")}).v));
  call("state-clear-expiry", {std::string("exp.clear")});
  EXPECT_FALSE(std::get<bool>(call("state-has-expiry", {std::string("exp.clear")}).v));
}

TEST_F(ExpiryIntrinsicsTest, IsNotExpiredByDefault) {
  call("state-set", {std::string("exp.fresh"), std::string("val")});
  auto result = call("state-is-expired", {std::string("exp.fresh")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, IsNotExpiredForFutureExpiry) {
  call("state-set", {std::string("exp.future"), std::string("val")});
  call("state-expire", {std::string("exp.future"), 3600.0});
  auto result = call("state-is-expired", {std::string("exp.future")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ExpiryIntrinsicsTest, ExpireNonexistentThrows) {
  EXPECT_THROW(call("state-expire", {std::string("no.path"), 100.0}), std::runtime_error);
}

TEST_F(ExpiryIntrinsicsTest, SweepExpiredRoot) {
  auto result = call("state-sweep-expired", {});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ExpiryIntrinsicsTest, SweepNonexistent) {
  auto result = call("state-sweep-expired", {std::string("no.path")});
  EXPECT_EQ(std::get<int64_t>(result.v), 0);
}

TEST_F(ExpiryIntrinsicsTest, HasExpiryNonexistent) {
  auto result = call("state-has-expiry", {std::string("no.path")});
  EXPECT_FALSE(std::get<bool>(result.v));
}

// ===========================================================================
// MsgSendIntrinsicsTest
// ===========================================================================

class MsgSendIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    proc->message_count = 0;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.proc = proc;
    ictx.pid = 1;

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(MsgSendIntrinsicsTest, SendMessage) {
  auto result = call("msg-send", {std::string("msg.target"), std::string("hello")});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
  EXPECT_EQ(proc->message_count, 1u);
}

TEST_F(MsgSendIntrinsicsTest, SendWithContentType) {
  auto result = call("msg-send", {std::string("msg.typed"), std::string("{\"a\":1}"),
                                  std::string("application/json")});
  auto *d = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(d, nullptr);
}

TEST_F(MsgSendIntrinsicsTest, SendIncreasesCount) {
  call("msg-send", {std::string("msg.a"), std::string("1")});
  call("msg-send", {std::string("msg.b"), std::string("2")});
  call("msg-send", {std::string("msg.c"), std::string("3")});
  EXPECT_EQ(proc->message_count, 3u);
}

TEST_F(MsgSendIntrinsicsTest, WrongArgCount) {
  EXPECT_THROW(call("msg-send", {std::string("only-path")}), std::runtime_error);
}

// ===========================================================================
// RegisterIntrinsicsTest — verify all expected names exist
// ===========================================================================

class RegisterIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.pid = 1;
    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }
};

TEST_F(RegisterIntrinsicsTest, AllStateIntrinsicsRegistered) {
  for (auto &name : {"state-get", "state-set", "state-children", "state-exists", "state-delete",
                     "state-data-get", "state-data-set"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllSchedulerIntrinsicsRegistered) {
  for (auto &name :
       {"spawn", "fork", "self-pid", "self-uid", "kill", "pause", "resume", "ps", "inspect"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllResourceIntrinsicsRegistered) {
  for (auto &name : {"memory-usage", "memory-limit", "time-elapsed", "time-limit", "message-count",
                     "message-limit", "step-count"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllSystemIntrinsicsRegistered) {
  for (auto &name : {"cluster-id", "node-id", "scheduler-stats"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllExpiryIntrinsicsRegistered) {
  for (auto &name : {"state-expire", "state-expire-at", "state-has-expiry", "state-is-expired",
                     "state-clear-expiry", "state-sweep-expired"}) {
    EXPECT_NE(env->lookup(name), nullptr) << "Missing intrinsic: " << name;
  }
}

TEST_F(RegisterIntrinsicsTest, AllMsgIntrinsicsRegistered) {
  EXPECT_NE(env->lookup("msg-send"), nullptr);
}

// ===========================================================================
// ResourcePolicyTest — validate_limits()
// ===========================================================================

class ResourcePolicyTest : public ::testing::Test {};

TEST_F(ResourcePolicyTest, NoConstraints) {
  resource_policy policy;
  process_limits req{1000, 60.0, 1024, 50};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 1000u);
  EXPECT_EQ(result.max_time, 60.0);
  EXPECT_EQ(result.max_memory, 1024u);
  EXPECT_EQ(result.max_messages, 50u);
}

TEST_F(ResourcePolicyTest, DefaultsApplied) {
  resource_policy policy;
  policy.max_steps_default = 500;
  policy.max_time_default = 30.0;
  policy.max_memory_default = 2048;
  policy.max_messages_default = 20;

  process_limits req{};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 500u);
  EXPECT_EQ(result.max_time, 30.0);
  EXPECT_EQ(result.max_memory, 2048u);
  EXPECT_EQ(result.max_messages, 20u);
}

TEST_F(ResourcePolicyTest, DefaultsNotOverride) {
  resource_policy policy;
  policy.max_steps_default = 500;
  process_limits req{1000, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 1000u);
}

TEST_F(ResourcePolicyTest, ClampAboveMax) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{200, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
}

TEST_F(ResourcePolicyTest, ClampBelowMin) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{10, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 50u);
}

TEST_F(ResourcePolicyTest, StrictAboveMaxThrows) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{200, 0, 0, 0};
  EXPECT_THROW(validate_limits(policy, req), std::runtime_error);
}

TEST_F(ResourcePolicyTest, StrictBelowMinThrows) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{10, 0, 0, 0};
  EXPECT_THROW(validate_limits(policy, req), std::runtime_error);
}

TEST_F(ResourcePolicyTest, StrictWithinRange) {
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.max_steps_max = 200;
  policy.enforce = resource_policy::mode::strict;
  process_limits req{100, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
}

TEST_F(ResourcePolicyTest, WarnModeClampsLikeClamp) {
  resource_policy policy;
  policy.max_time_max = 30.0;
  policy.enforce = resource_policy::mode::warn;
  process_limits req{0, 60.0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_time, 30.0);
}

TEST_F(ResourcePolicyTest, AllFieldsClamped) {
  resource_policy policy;
  policy.max_steps_max = 100;
  policy.max_time_max = 10.0;
  policy.max_memory_max = 500;
  policy.max_messages_max = 20;
  policy.enforce = resource_policy::mode::clamp;

  process_limits req{999, 99.0, 999, 999};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 100u);
  EXPECT_EQ(result.max_time, 10.0);
  EXPECT_EQ(result.max_memory, 500u);
  EXPECT_EQ(result.max_messages, 20u);
}

TEST_F(ResourcePolicyTest, ZeroValueNotClamped) {
  // A value of 0 means "unlimited" and should not be clamped
  resource_policy policy;
  policy.max_steps_min = 50;
  policy.max_steps_max = 200;
  policy.enforce = resource_policy::mode::clamp;
  process_limits req{0, 0, 0, 0};
  auto result = validate_limits(policy, req);
  EXPECT_EQ(result.max_steps, 0u);
}

// ===========================================================================
// StdlibRegistryTest — module registration and lookup
// ===========================================================================

class StdlibRegistryTest : public ::testing::Test {
protected:
  stdlib_registry registry;
};

TEST_F(StdlibRegistryTest, BuiltinModulesExist) {
  auto mods = registry.list_modules();
  EXPECT_GE(mods.size(), 3u);
  auto has = [&](const std::string &m) {
    return std::find(mods.begin(), mods.end(), m) != mods.end();
  };
  EXPECT_TRUE(has("string"));
  EXPECT_TRUE(has("math"));
  EXPECT_TRUE(has("collections"));
}

TEST_F(StdlibRegistryTest, StringModuleFunctions) {
  auto fns = registry.list_functions("string");
  EXPECT_GE(fns.size(), 10u);
  auto has = [&](const std::string &f) {
    return std::find(fns.begin(), fns.end(), f) != fns.end();
  };
  EXPECT_TRUE(has("string.split"));
  EXPECT_TRUE(has("string.join"));
  EXPECT_TRUE(has("string.replace"));
  EXPECT_TRUE(has("string.trim"));
  EXPECT_TRUE(has("string.upper"));
  EXPECT_TRUE(has("string.lower"));
  EXPECT_TRUE(has("string.starts-with"));
  EXPECT_TRUE(has("string.ends-with"));
  EXPECT_TRUE(has("string.contains"));
  EXPECT_TRUE(has("string.substring"));
  EXPECT_TRUE(has("string.char-at"));
  EXPECT_TRUE(has("string.length"));
}

TEST_F(StdlibRegistryTest, MathModuleFunctions) {
  auto fns = registry.list_functions("math");
  EXPECT_GE(fns.size(), 14u);
}

TEST_F(StdlibRegistryTest, CollectionsModuleFunctions) {
  auto fns = registry.list_functions("collections");
  EXPECT_GE(fns.size(), 10u);
}

TEST_F(StdlibRegistryTest, ListEmpty) {
  auto fns = registry.list_functions("nonexistent");
  EXPECT_TRUE(fns.empty());
}

TEST_F(StdlibRegistryTest, ImportAll) {
  auto env = builtins::make_default_environment();
  registry.import_module("string", env);
  EXPECT_NE(env->lookup("string.split"), nullptr);
  EXPECT_NE(env->lookup("string.join"), nullptr);
  EXPECT_NE(env->lookup("string.upper"), nullptr);
}

TEST_F(StdlibRegistryTest, ImportSpecific) {
  auto env = builtins::make_default_environment();
  registry.import_module("math", env, {"math.sqrt", "math.abs"});
  EXPECT_NE(env->lookup("math.sqrt"), nullptr);
  EXPECT_NE(env->lookup("math.abs"), nullptr);
  // Not imported
  EXPECT_EQ(env->lookup("math.sin"), nullptr);
}

TEST_F(StdlibRegistryTest, ImportUnknownModuleThrows) {
  auto env = builtins::make_default_environment();
  EXPECT_THROW(registry.import_module("bogus", env), std::runtime_error);
}

TEST_F(StdlibRegistryTest, ImportUnknownFunctionThrows) {
  auto env = builtins::make_default_environment();
  EXPECT_THROW(registry.import_module("string", env, {"no.such.fn"}), std::runtime_error);
}

TEST_F(StdlibRegistryTest, LookupQualified) {
  auto *fn = registry.lookup_qualified("string.split");
  EXPECT_NE(fn, nullptr);
}

TEST_F(StdlibRegistryTest, LookupQualifiedNotFound) {
  auto *fn = registry.lookup_qualified("nonexistent.fn");
  EXPECT_EQ(fn, nullptr);
}

TEST_F(StdlibRegistryTest, RegisterCustomFunction) {
  registry.register_function("custom", "custom.noop",
                             [](std::span<const value_t>) -> value_t { return nil_value; });
  auto mods = registry.list_modules();
  auto has = std::find(mods.begin(), mods.end(), "custom") != mods.end();
  EXPECT_TRUE(has);
  auto fns = registry.list_functions("custom");
  EXPECT_EQ(fns.size(), 1u);
}

// ===========================================================================
// StdlibStringTest — individual string functions
// ===========================================================================

class StdlibStringTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("string", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibStringTest, Split) {
  auto result = call("string.split", {std::string("a,b,c"), std::string(",")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  EXPECT_EQ(std::get<std::string>((**lst)[0].v), "a");
  EXPECT_EQ(std::get<std::string>((**lst)[1].v), "b");
  EXPECT_EQ(std::get<std::string>((**lst)[2].v), "c");
}

TEST_F(StdlibStringTest, SplitDefault) {
  auto result = call("string.split", {std::string("a b c")});
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
}

TEST_F(StdlibStringTest, Join) {
  auto lst = make_list({std::string("x"), std::string("y"), std::string("z")});
  auto result = call("string.join", {lst, std::string("-")});
  EXPECT_EQ(std::get<std::string>(result.v), "x-y-z");
}

TEST_F(StdlibStringTest, Replace) {
  auto result = call("string.replace",
                     {std::string("hello world"), std::string("world"), std::string("there")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello there");
}

TEST_F(StdlibStringTest, Trim) {
  auto result = call("string.trim", {std::string("  hello  ")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StdlibStringTest, TrimAllWhitespace) {
  auto result = call("string.trim", {std::string("   ")});
  EXPECT_EQ(std::get<std::string>(result.v), "");
}

TEST_F(StdlibStringTest, Upper) {
  auto result = call("string.upper", {std::string("hello")});
  EXPECT_EQ(std::get<std::string>(result.v), "HELLO");
}

TEST_F(StdlibStringTest, Lower) {
  auto result = call("string.lower", {std::string("HELLO")});
  EXPECT_EQ(std::get<std::string>(result.v), "hello");
}

TEST_F(StdlibStringTest, StartsWith) {
  auto yes = call("string.starts-with", {std::string("hello"), std::string("he")});
  auto no = call("string.starts-with", {std::string("hello"), std::string("lo")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, EndsWith) {
  auto yes = call("string.ends-with", {std::string("hello"), std::string("lo")});
  auto no = call("string.ends-with", {std::string("hello"), std::string("he")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, Contains) {
  auto yes = call("string.contains", {std::string("hello world"), std::string("lo w")});
  auto no = call("string.contains", {std::string("hello"), std::string("xyz")});
  EXPECT_TRUE(std::get<bool>(yes.v));
  EXPECT_FALSE(std::get<bool>(no.v));
}

TEST_F(StdlibStringTest, Substring) {
  auto result = call("string.substring", {std::string("hello"), int64_t(1), int64_t(3)});
  EXPECT_EQ(std::get<std::string>(result.v), "ell");
}

TEST_F(StdlibStringTest, SubstringNoLen) {
  auto result = call("string.substring", {std::string("hello"), int64_t(2)});
  EXPECT_EQ(std::get<std::string>(result.v), "llo");
}

TEST_F(StdlibStringTest, CharAt) {
  auto result = call("string.char-at", {std::string("hello"), int64_t(1)});
  EXPECT_EQ(std::get<std::string>(result.v), "e");
}

TEST_F(StdlibStringTest, CharAtOutOfRange) {
  EXPECT_THROW(call("string.char-at", {std::string("hi"), int64_t(5)}), std::runtime_error);
}

TEST_F(StdlibStringTest, Length) {
  auto result = call("string.length", {std::string("hello")});
  EXPECT_EQ(std::get<int64_t>(result.v), 5);
}

// ===========================================================================
// StdlibMathTest — individual math functions
// ===========================================================================

class StdlibMathTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("math", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibMathTest, Sqrt) {
  auto result = call("math.sqrt", {16.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 4.0);
}

TEST_F(StdlibMathTest, Abs) {
  auto d = call("math.abs", {-3.5});
  EXPECT_DOUBLE_EQ(std::get<double>(d.v), 3.5);
  auto i = call("math.abs", {int64_t(-7)});
  EXPECT_EQ(std::get<int64_t>(i.v), 7);
}

TEST_F(StdlibMathTest, Floor) {
  auto result = call("math.floor", {3.7});
  EXPECT_EQ(std::get<int64_t>(result.v), 3);
}

TEST_F(StdlibMathTest, Ceil) {
  auto result = call("math.ceil", {3.2});
  EXPECT_EQ(std::get<int64_t>(result.v), 4);
}

TEST_F(StdlibMathTest, Round) {
  auto result = call("math.round", {3.5});
  EXPECT_EQ(std::get<int64_t>(result.v), 4);
}

TEST_F(StdlibMathTest, Pow) {
  auto result = call("math.pow", {2.0, 10.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 1024.0);
}

TEST_F(StdlibMathTest, Log) {
  auto result = call("math.log", {1.0});
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 0.0);
}

TEST_F(StdlibMathTest, Trig) {
  auto s = call("math.sin", {0.0});
  auto c = call("math.cos", {0.0});
  auto t = call("math.tan", {0.0});
  EXPECT_NEAR(std::get<double>(s.v), 0.0, 1e-10);
  EXPECT_NEAR(std::get<double>(c.v), 1.0, 1e-10);
  EXPECT_NEAR(std::get<double>(t.v), 0.0, 1e-10);
}

TEST_F(StdlibMathTest, MinMax) {
  auto mn = call("math.min", {3.0, 1.0, 2.0});
  auto mx = call("math.max", {3.0, 1.0, 2.0});
  EXPECT_DOUBLE_EQ(std::get<double>(mn.v), 1.0);
  EXPECT_DOUBLE_EQ(std::get<double>(mx.v), 3.0);
}

TEST_F(StdlibMathTest, Clamp) {
  auto below = call("math.clamp", {-1.0, 0.0, 10.0});
  auto above = call("math.clamp", {15.0, 0.0, 10.0});
  auto inside = call("math.clamp", {5.0, 0.0, 10.0});
  EXPECT_DOUBLE_EQ(std::get<double>(below.v), 0.0);
  EXPECT_DOUBLE_EQ(std::get<double>(above.v), 10.0);
  EXPECT_DOUBLE_EQ(std::get<double>(inside.v), 5.0);
}

TEST_F(StdlibMathTest, Pi) {
  auto result = call("math.pi", {});
  EXPECT_NEAR(std::get<double>(result.v), 3.14159265, 1e-6);
}

TEST_F(StdlibMathTest, E) {
  auto result = call("math.e", {});
  EXPECT_NEAR(std::get<double>(result.v), 2.71828182, 1e-6);
}

TEST_F(StdlibMathTest, Random) {
  auto result = call("math.random", {});
  double val = std::get<double>(result.v);
  EXPECT_GE(val, 0.0);
  EXPECT_LT(val, 1.0);
}

// ===========================================================================
// StdlibCollectionsTest — individual collections functions
// ===========================================================================

class StdlibCollectionsTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("collections", env);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(StdlibCollectionsTest, MapWithNative) {
  native_fn double_it = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) * 2);
  };
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.map", {lst, value_t(double_it)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 4);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 6);
}

TEST_F(StdlibCollectionsTest, FilterWithNative) {
  native_fn is_positive = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) > 0);
  };
  auto lst = make_list({int64_t(-1), int64_t(2), int64_t(-3), int64_t(4)});
  auto result = call("collections.filter", {lst, value_t(is_positive)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 2u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 4);
}

TEST_F(StdlibCollectionsTest, ReduceSum) {
  native_fn add = [](std::span<const value_t> args) -> value_t {
    return value_t(std::get<int64_t>(args[0].v) + std::get<int64_t>(args[1].v));
  };
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.reduce", {lst, value_t(add), int64_t(0)});
  EXPECT_EQ(std::get<int64_t>(result.v), 6);
}

TEST_F(StdlibCollectionsTest, Zip) {
  auto a = make_list({int64_t(1), int64_t(2)});
  auto b = make_list({std::string("a"), std::string("b")});
  auto result = call("collections.zip", {a, b});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, Flatten) {
  auto inner = make_list({int64_t(2), int64_t(3)});
  auto lst = make_list({int64_t(1), inner, int64_t(4)});
  auto result = call("collections.flatten", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 4u);
}

TEST_F(StdlibCollectionsTest, Sort) {
  auto lst = make_list({int64_t(3), int64_t(1), int64_t(2)});
  auto result = call("collections.sort", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 1);
  EXPECT_EQ(std::get<int64_t>((**r)[1].v), 2);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 3);
}

TEST_F(StdlibCollectionsTest, SortStrings) {
  auto lst = make_list({std::string("c"), std::string("a"), std::string("b")});
  auto result = call("collections.sort", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(std::get<std::string>((**r)[0].v), "a");
}

TEST_F(StdlibCollectionsTest, Reverse) {
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(3)});
  auto result = call("collections.reverse", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 3);
  EXPECT_EQ(std::get<int64_t>((**r)[2].v), 1);
}

TEST_F(StdlibCollectionsTest, Range) {
  auto result = call("collections.range", {int64_t(5)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 5u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**r)[4].v), 4);
}

TEST_F(StdlibCollectionsTest, RangeStartStop) {
  auto result = call("collections.range", {int64_t(2), int64_t(5)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**r)[0].v), 2);
}

TEST_F(StdlibCollectionsTest, RangeStep) {
  auto result = call("collections.range", {int64_t(0), int64_t(10), int64_t(3)});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  ASSERT_EQ((*r)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**r)[3].v), 9);
}

TEST_F(StdlibCollectionsTest, RangeZeroStepThrows) {
  EXPECT_THROW(call("collections.range", {int64_t(0), int64_t(5), int64_t(0)}), std::runtime_error);
}

TEST_F(StdlibCollectionsTest, Unique) {
  auto lst = make_list({int64_t(1), int64_t(2), int64_t(1), int64_t(3), int64_t(2)});
  auto result = call("collections.unique", {lst});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 3u);
}

TEST_F(StdlibCollectionsTest, DictKeys) {
  auto d = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto result = call("collections.dict-keys", {d});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, DictValues) {
  auto d = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto result = call("collections.dict-values", {d});
  auto *r = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ((*r)->size(), 2u);
}

TEST_F(StdlibCollectionsTest, DictMerge) {
  auto d1 = make_dict({{"a", int64_t(1)}, {"b", int64_t(2)}});
  auto d2 = make_dict({{"b", int64_t(99)}, {"c", int64_t(3)}});
  auto result = call("collections.dict-merge", {d1, d2});
  auto *r = std::get_if<dict_ptr>(&result.v);
  ASSERT_NE(r, nullptr);
  // b should be updated to 99, c should be added
  EXPECT_EQ((*r)->size(), 3u);
  bool found_b = false, found_c = false;
  for (auto &[k, v] : **r) {
    if (k == "b") {
      EXPECT_EQ(std::get<int64_t>(v.v), 99);
      found_b = true;
    }
    if (k == "c") {
      EXPECT_EQ(std::get<int64_t>(v.v), 3);
      found_c = true;
    }
  }
  EXPECT_TRUE(found_b);
  EXPECT_TRUE(found_c);
}

TEST_F(StdlibCollectionsTest, DictKeysNotDict) {
  EXPECT_THROW(call("collections.dict-keys", {int64_t(42)}), std::runtime_error);
}

// ===========================================================================
// IntrinsicsViaEvaluatorTest — test intrinsics end-to-end through DSL eval
// ===========================================================================

class IntrinsicsViaEvaluatorTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 42;
    proc->status = process_status::running;
    proc->max_memory = 1024;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 42;
    ictx.uid = "eval-user";
    ictx.cluster_id = "test-cluster";
    ictx.node_id = "test-node";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t eval(const std::string &code) {
    evaluator e(env);
    return e.evaluate_script(code);
  }
};

TEST_F(IntrinsicsViaEvaluatorTest, SelfPidViaDSL) {
  auto result = eval("(self-pid)");
  EXPECT_EQ(std::get<int64_t>(result.v), 42);
}

TEST_F(IntrinsicsViaEvaluatorTest, SelfUidViaDSL) {
  auto result = eval("(self-uid)");
  EXPECT_EQ(std::get<std::string>(result.v), "eval-user");
}

TEST_F(IntrinsicsViaEvaluatorTest, ClusterIdViaDSL) {
  auto result = eval("(cluster-id)");
  EXPECT_EQ(std::get<std::string>(result.v), "test-cluster");
}

TEST_F(IntrinsicsViaEvaluatorTest, StateSetGetViaDSL) {
  eval("(state-set \"dsl.key\" \"dsl-value\")");
  auto result = eval("(state-get \"dsl.key\")");
  EXPECT_EQ(std::get<std::string>(result.v), "dsl-value");
}

TEST_F(IntrinsicsViaEvaluatorTest, StateExistsViaDSL) {
  eval("(state-set \"dsl.exists\" \"yes\")");
  auto result = eval("(state-exists \"dsl.exists\")");
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(IntrinsicsViaEvaluatorTest, MemoryLimitViaDSL) {
  auto result = eval("(memory-limit)");
  EXPECT_EQ(std::get<int64_t>(result.v), 1024);
}

TEST_F(IntrinsicsViaEvaluatorTest, SpawnViaDSL) {
  auto result = eval("(spawn \"(+ 1 2)\")");
  EXPECT_GT(std::get<int64_t>(result.v), 0);
}

// ===========================================================================
// StdlibViaEvaluatorTest — test stdlib through DSL
// ===========================================================================

class StdlibViaEvaluatorTest : public ::testing::Test {
protected:
  stdlib_registry registry;
  environment_ptr env;

  void SetUp() override {
    env = builtins::make_default_environment();
    registry.import_module("string", env);
    registry.import_module("math", env);
    registry.import_module("collections", env);
  }

  value_t eval(const std::string &code) {
    evaluator e(env);
    return e.evaluate_script(code);
  }
};

TEST_F(StdlibViaEvaluatorTest, StringUpperViaDSL) {
  auto result = eval("(string.upper \"hello\")");
  EXPECT_EQ(std::get<std::string>(result.v), "HELLO");
}

TEST_F(StdlibViaEvaluatorTest, MathSqrtViaDSL) {
  auto result = eval("(math.sqrt 25)");
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 5.0);
}

TEST_F(StdlibViaEvaluatorTest, MathPiViaDSL) {
  auto result = eval("(math.pi)");
  EXPECT_NEAR(std::get<double>(result.v), 3.14159265, 1e-6);
}

TEST_F(StdlibViaEvaluatorTest, StringSplitAndLength) {
  auto result = eval("(string.length \"hello world\")");
  EXPECT_EQ(std::get<int64_t>(result.v), 11);
}

TEST_F(StdlibViaEvaluatorTest, CollectionsRangeViaDSL) {
  auto result = eval("(collections.range 3)");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  EXPECT_EQ((*lst)->size(), 3u);
}
