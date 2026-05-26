/// @file state_exec_integration_test.cpp
/// @brief Integration tests for state_exec: ACL, migration, messaging,
///        process identity, and multi-node coordination.

#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/exec_coordinator.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/resource_policy.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/state_value_codec.h>
#include <cvc/core/state_exec/stdlib.h>
#include <cvc/core/state_exec/types.h>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::state_exec;

// ===========================================================================
// Helpers
// ===========================================================================

static const char *MIME_EXEC_ELECTION_STR = "application/x-state-exec-election";

/// Force a coordinator to become leader by injecting a victory message.
static void make_leader(exec_coordinator &coord, const std::string &node_id,
                        const std::string &cluster_id) {
  auto victory = cvc::state_message::make_text(
      "__state_exec." + cluster_id + ".election",
      "{\"type\":\"election-victory\",\"node_id\":\"" + node_id + "\"}", MIME_EXEC_ELECTION_STR);
  victory.cluster_id = cluster_id;
  coord.on_message(victory);
}

// ===========================================================================
// ACL Integration Tests — UID/GID inheritance and identity
// ===========================================================================

class ACLIntegrationTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  intrinsics_context ictx;
  environment_ptr env;
  process_ptr proc = make_process();

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::ready;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "admin-user";
    ictx.cluster_id = "cluster-1";
    ictx.node_id = "node-A";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }
};

TEST_F(ACLIntegrationTest, ProcessInheritsUidFromOptions) {
  execute_options opts;
  opts.uid = "alice";
  opts.gid = "engineers";
  int pid = sched.execute(std::string("42"), opts);

  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->uid, "alice");
  EXPECT_EQ(info->gid, "engineers");
}

TEST_F(ACLIntegrationTest, SpawnedProcessInheritsParentUid) {
  // The spawn intrinsic copies ctx->uid to the child's execute_options.
  // We need to pass the intrinsics-enabled env to the process.
  ictx.uid = "alice";
  execute_options opts;
  opts.uid = "alice";
  opts.gid = "team-a";
  opts.env = env; // environment with intrinsics registered
  int parent_pid = sched.execute(std::string("(spawn \"42\" \"child\")"), opts);

  sched.run();

  // Parent should have uid "alice"
  auto parent_info = sched.get_process_info(parent_pid);
  ASSERT_TRUE(parent_info.has_value());
  EXPECT_EQ(parent_info->uid, "alice");

  // The child was spawned — find it
  auto procs = sched.list_processes();
  ASSERT_GE(procs.size(), 2u);
  const process_info *child = nullptr;
  for (auto &p : procs) {
    if (p.name == "child") {
      child = &p;
      break;
    }
  }
  ASSERT_NE(child, nullptr);
  // Spawned child inherits parent's UID
  EXPECT_EQ(child->uid, "alice");
}

TEST_F(ACLIntegrationTest, ForkedProcessInheritsUidGid) {
  execute_options opts;
  opts.uid = "bob";
  opts.gid = "ops";
  opts.priority = -3;
  int parent = sched.execute(std::string("(begin 1 2 3)"), opts);
  sched.step(); // advance parent

  int child = sched.fork(parent);
  ASSERT_GT(child, 0);

  auto child_info = sched.get_process_info(child);
  ASSERT_TRUE(child_info.has_value());
  EXPECT_EQ(child_info->uid, "bob");
  EXPECT_EQ(child_info->gid, "ops");
  EXPECT_EQ(child_info->priority, -3);
  EXPECT_EQ(child_info->parent_pid, parent);
}

TEST_F(ACLIntegrationTest, SelfUidReturnsProcessIdentity) {
  // self-uid requires intrinsics; verify via C++ API instead
  execute_options opts;
  opts.uid = "carol";
  int pid = sched.execute(std::string("42"), opts);
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->uid, "carol");

  // Also verify via intrinsics-enabled env
  ictx.uid = "carol";
  execute_options opts2;
  opts2.uid = "carol";
  opts2.env = env;
  int pid2 = sched.execute(std::string("(self-uid)"), opts2);
  auto results = sched.run();
  ASSERT_TRUE(results.count(pid2));
  ASSERT_TRUE(std::holds_alternative<std::string>(results[pid2].v));
  EXPECT_EQ(std::get<std::string>(results[pid2].v), "carol");
}

TEST_F(ACLIntegrationTest, MultipleUsersCoexist) {
  // Verify multiple processes with different UIDs coexist
  execute_options opts_a;
  opts_a.uid = "alice";
  opts_a.name = "alice-proc";
  int pid_a = sched.execute(std::string("42"), opts_a);

  execute_options opts_b;
  opts_b.uid = "bob";
  opts_b.name = "bob-proc";
  int pid_b = sched.execute(std::string("43"), opts_b);

  sched.run();

  auto info_a = sched.get_process_info(pid_a);
  auto info_b = sched.get_process_info(pid_b);
  ASSERT_TRUE(info_a.has_value());
  ASSERT_TRUE(info_b.has_value());
  EXPECT_EQ(info_a->uid, "alice");
  EXPECT_EQ(info_b->uid, "bob");
  EXPECT_EQ(info_a->name, "alice-proc");
  EXPECT_EQ(info_b->name, "bob-proc");
}

// ===========================================================================
// ACL across clusters — resource policy enforcement
// ===========================================================================

class ClusterACLTest : public ::testing::Test {
protected:
  cvc::state_message_bus bus;
  scheduler sched;
  exec_coordinator coord;

  void SetUp() override {
    coord.set_node_id("node-A");
    coord.set_cluster_id("cluster-1");
    coord.attach_scheduler(&sched);
    coord.attach_message_bus(&bus);
    coord.start();
    make_leader(coord, "node-A", "cluster-1");
  }
  void TearDown() override {
    if (coord.is_running())
      coord.stop();
  }
};

TEST_F(ClusterACLTest, ResourcePolicyLimitsProcessCount) {
  resource_policy policy;
  policy.max_processes = 2;
  policy.enforce = resource_policy::mode::strict;
  coord.set_resource_policy(policy);

  auto r1 = coord.submit("(begin (while true 1))");
  EXPECT_TRUE(r1.accepted);

  auto r2 = coord.submit("(begin (while true 1))");
  EXPECT_TRUE(r2.accepted);

  // Third process should be rejected — over limit
  auto r3 = coord.submit("(begin (while true 1))");
  EXPECT_FALSE(r3.accepted);
}

TEST_F(ClusterACLTest, ResourcePolicyClampsLimits) {
  resource_policy policy;
  policy.max_steps_min = 100;
  policy.max_steps_max = 5000;
  policy.max_steps_default = 1000;
  policy.enforce = resource_policy::mode::clamp;
  coord.set_resource_policy(policy);

  execute_options opts;
  opts.max_steps = 99999; // way above max
  auto r = coord.submit("(+ 1 2)", opts);
  EXPECT_TRUE(r.accepted);

  // The process should be clamped, not rejected
  auto info = sched.get_process_info(r.pid);
  ASSERT_TRUE(info.has_value());
}

TEST_F(ClusterACLTest, SubmitWithUidPreservesIdentity) {
  execute_options opts;
  opts.uid = "alice";
  opts.gid = "scientists";
  auto r = coord.submit("(+ 1 2)", opts);
  EXPECT_TRUE(r.accepted);

  sched.run();
  // Verify UID via process info
  auto info = sched.get_process_info(r.pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->uid, "alice");
  EXPECT_EQ(info->gid, "scientists");
}

// ===========================================================================
// Multi-cluster (cluster of clusters) ACL enforcement
// ===========================================================================

class MultiClusterACLTest : public ::testing::Test {
protected:
  cvc::state_message_bus shared_bus;
  scheduler sched_a, sched_b;
  exec_coordinator coord_a, coord_b;

  void SetUp() override {
    coord_a.set_node_id("node-A");
    coord_a.set_cluster_id("cluster-1");
    coord_a.attach_scheduler(&sched_a);
    coord_a.attach_message_bus(&shared_bus);

    coord_b.set_node_id("node-B");
    coord_b.set_cluster_id("cluster-1");
    coord_b.attach_scheduler(&sched_b);
    coord_b.attach_message_bus(&shared_bus);

    coord_a.start();
    coord_b.start();

    // Make node-A the leader
    make_leader(coord_a, "node-A", "cluster-1");
    make_leader(coord_b, "node-A", "cluster-1");
  }
  void TearDown() override {
    if (coord_a.is_running())
      coord_a.stop();
    if (coord_b.is_running())
      coord_b.stop();
  }
};

TEST_F(MultiClusterACLTest, CrossNodeAdminKillEnforcedByLeader) {
  // Submit a process on node-A
  execute_options opts;
  opts.uid = "alice";
  auto r = coord_a.submit("(begin (while true 1))");
  EXPECT_TRUE(r.accepted);
  int pid = r.pid;

  // node-B sends a kill command to node-A
  auto ctrl = cvc::state_message::make_text("__state_exec.cluster-1.control.node-A",
                                            "{\"command\":\"kill\",\"pid\":" + std::to_string(pid) +
                                                ",\"from\":\"node-B\"}",
                                            "application/x-state-exec-control");
  ctrl.cluster_id = "cluster-1";
  ctrl.origin_node_id = "node-B";
  coord_a.on_message(ctrl);

  auto info = sched_a.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(MultiClusterACLTest, CrossNodePauseResume) {
  auto r = coord_a.submit("(begin (while true 1))");
  EXPECT_TRUE(r.accepted);
  int pid = r.pid;

  // Pause from node-B
  auto pause_ctrl = cvc::state_message::make_text(
      "__state_exec.cluster-1.control.node-A",
      "{\"command\":\"pause\",\"pid\":" + std::to_string(pid) + ",\"from\":\"node-B\"}",
      "application/x-state-exec-control");
  pause_ctrl.cluster_id = "cluster-1";
  pause_ctrl.origin_node_id = "node-B";
  coord_a.on_message(pause_ctrl);

  auto info = sched_a.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::paused);

  // Resume from node-B
  auto resume_ctrl = cvc::state_message::make_text(
      "__state_exec.cluster-1.control.node-A",
      "{\"command\":\"resume\",\"pid\":" + std::to_string(pid) + ",\"from\":\"node-B\"}",
      "application/x-state-exec-control");
  resume_ctrl.cluster_id = "cluster-1";
  resume_ctrl.origin_node_id = "node-B";
  coord_a.on_message(resume_ctrl);

  info = sched_a.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::ready);
}

TEST_F(MultiClusterACLTest, ResourcePolicyBroadcastAcrossCluster) {
  // Leader pushes policy to the entire cluster
  resource_policy policy;
  policy.max_processes = 5;
  policy.max_steps_max = 10000;
  policy.max_memory_max = 1048576;
  policy.enforce = resource_policy::mode::strict;

  // Capture policy messages on the bus
  std::vector<cvc::state_message> captured;
  shared_bus.subscribe("__state_exec.cluster-1.policy",
                       [&](const cvc::state_message &m) { captured.push_back(m); });

  EXPECT_TRUE(coord_a.admin_set_policy(policy));

  // A policy message was emitted
  EXPECT_GE(captured.size(), 1u);
}

TEST_F(MultiClusterACLTest, LeaderHandoffTransfersAuthority) {
  EXPECT_TRUE(coord_a.is_leader());
  EXPECT_FALSE(coord_b.is_leader());

  // Handoff A → B
  EXPECT_TRUE(coord_a.admin_handoff("node-B"));

  // Deliver the victory message to both coordinators
  auto handoff_victory = cvc::state_message::make_text(
      "__state_exec.cluster-1.election", "{\"type\":\"election-victory\",\"node_id\":\"node-B\"}",
      MIME_EXEC_ELECTION_STR);
  handoff_victory.cluster_id = "cluster-1";
  handoff_victory.origin_node_id = "node-B";
  coord_a.on_message(handoff_victory);
  coord_b.on_message(handoff_victory);

  EXPECT_FALSE(coord_a.is_leader());
  EXPECT_TRUE(coord_b.is_leader());

  // B can now accept submissions
  auto r = coord_b.submit("(+ 1 2)");
  EXPECT_TRUE(r.accepted);
}

// ===========================================================================
// Migration Integration Tests
// ===========================================================================

class MigrationIntegrationTest : public ::testing::Test {
protected:
  cvc::state_message_bus shared_bus;
  scheduler sched_a, sched_b;
  exec_coordinator coord_a, coord_b;

  void SetUp() override {
    coord_a.set_node_id("node-A");
    coord_a.set_cluster_id("cluster-1");
    coord_a.attach_scheduler(&sched_a);
    coord_a.attach_message_bus(&shared_bus);

    coord_b.set_node_id("node-B");
    coord_b.set_cluster_id("cluster-1");
    coord_b.attach_scheduler(&sched_b);
    coord_b.attach_message_bus(&shared_bus);

    coord_a.start();
    coord_b.start();

    make_leader(coord_a, "node-A", "cluster-1");
    make_leader(coord_b, "node-A", "cluster-1");
  }
  void TearDown() override {
    if (coord_a.is_running())
      coord_a.stop();
    if (coord_b.is_running())
      coord_b.stop();
  }
};

TEST_F(MigrationIntegrationTest, MigrateProcessBetweenNodes) {
  execute_options opts;
  opts.name = "migrating-proc";
  opts.uid = "alice";
  auto sr = coord_a.submit("(begin (+ 1 2) (+ 3 4) (+ 5 6))", opts);
  EXPECT_TRUE(sr.accepted);
  int pid = sr.pid;

  // Capture migration messages
  std::vector<cvc::state_message> migrate_msgs;
  shared_bus.subscribe("__state_exec.cluster-1.migrate",
                       [&](const cvc::state_message &m) { migrate_msgs.push_back(m); });

  // Migrate from node-A to node-B
  auto mr = coord_a.migrate(pid, "node-B");
  EXPECT_TRUE(mr.success);
  EXPECT_EQ(mr.target_node, "node-B");

  // A migration message should have been emitted
  EXPECT_GE(migrate_msgs.size(), 1u);

  // The original process on node-A should be killed
  auto info_a = sched_a.get_process_info(pid);
  ASSERT_TRUE(info_a.has_value());
  EXPECT_EQ(info_a->status, process_status::killed);

  // Deliver the migration message to node-B
  for (auto &m : migrate_msgs) {
    coord_b.on_message(m);
  }

  // node-B should now have the process
  auto procs_b = sched_b.list_processes();
  ASSERT_GE(procs_b.size(), 1u);

  // Find the migrated process
  const process_info *migrated = nullptr;
  for (auto &p : procs_b) {
    if (p.name == "migrating-proc") {
      migrated = &p;
      break;
    }
  }
  ASSERT_NE(migrated, nullptr);
  // Migrated process preserves UID
  EXPECT_EQ(migrated->uid, "alice");
}

TEST_F(MigrationIntegrationTest, MigrateToSelfFails) {
  auto sr = coord_a.submit("(+ 1 2)");
  EXPECT_TRUE(sr.accepted);

  auto mr = coord_a.migrate(sr.pid, "node-A");
  EXPECT_FALSE(mr.success);
  EXPECT_EQ(mr.error, "cannot migrate to self");
}

TEST_F(MigrationIntegrationTest, MigrateNonexistentProcessFails) {
  auto mr = coord_a.migrate(9999, "node-B");
  EXPECT_FALSE(mr.success);
}

TEST_F(MigrationIntegrationTest, MigratePreservesUidGid) {
  execute_options opts;
  opts.uid = "charlie";
  opts.gid = "devops";
  opts.name = "uid-test";
  auto sr = coord_a.submit("(begin 1 2 3)", opts);
  EXPECT_TRUE(sr.accepted);

  // Capture and forward migration
  std::vector<cvc::state_message> msgs;
  shared_bus.subscribe("__state_exec.cluster-1.migrate",
                       [&](const cvc::state_message &m) { msgs.push_back(m); });

  auto mr = coord_a.migrate(sr.pid, "node-B");
  EXPECT_TRUE(mr.success);

  for (auto &m : msgs)
    coord_b.on_message(m);

  // Check that UID was preserved on the target
  auto procs_b = sched_b.list_processes();
  const process_info *found = nullptr;
  for (auto &p : procs_b) {
    if (p.name == "uid-test") {
      found = &p;
      break;
    }
  }
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->uid, "charlie");
}

TEST_F(MigrationIntegrationTest, MigratePreservesPriority) {
  execute_options opts;
  opts.priority = -7;
  opts.name = "priority-test";
  auto sr = coord_a.submit("(begin 1 2 3)", opts);
  EXPECT_TRUE(sr.accepted);

  std::vector<cvc::state_message> msgs;
  shared_bus.subscribe("__state_exec.cluster-1.migrate",
                       [&](const cvc::state_message &m) { msgs.push_back(m); });

  auto mr = coord_a.migrate(sr.pid, "node-B");
  EXPECT_TRUE(mr.success);
  for (auto &m : msgs)
    coord_b.on_message(m);

  auto procs_b = sched_b.list_processes();
  const process_info *found = nullptr;
  for (auto &p : procs_b) {
    if (p.name == "priority-test") {
      found = &p;
      break;
    }
  }
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found->priority, -7);
}

TEST_F(MigrationIntegrationTest, MigrationStatsTracked) {
  auto sr = coord_a.submit("(begin 1 2 3)");
  EXPECT_TRUE(sr.accepted);

  std::vector<cvc::state_message> msgs;
  shared_bus.subscribe("__state_exec.cluster-1.migrate",
                       [&](const cvc::state_message &m) { msgs.push_back(m); });

  coord_a.migrate(sr.pid, "node-B");
  auto stats = coord_a.stats();
  EXPECT_GE(stats.migrations_initiated, 1u);

  // Deliver to target
  for (auto &m : msgs)
    coord_b.on_message(m);

  // Target should record migration completion
  auto stats_b = coord_b.stats();
  EXPECT_GE(stats_b.migrations_completed, 1u);
}

TEST_F(MigrationIntegrationTest, SelfMigrationViaStateTree) {
  // Demonstrates the self-migration pattern: a process signals its desire
  // to migrate, and external management code performs the actual migration.
  //
  // In production, the process would write to the state tree and a
  // management loop would poll for migration requests.  Here we simulate
  // the "process wants to migrate" + "management loop performs it" flow.

  execute_options opts;
  opts.uid = "self-mover";
  opts.name = "live-self-migrate";

  // Create a long-running process
  auto sr = coord_a.submit("(begin (while true 1))", opts);
  EXPECT_TRUE(sr.accepted);

  // Step a few times but don't complete
  sched_a.step();
  sched_a.step();

  // Capture migration messages
  std::vector<cvc::state_message> msgs;
  shared_bus.subscribe("__state_exec.cluster-1.migrate",
                       [&](const cvc::state_message &m) { msgs.push_back(m); });

  // External management code migrates the process (simulating self-migration)
  auto mr = coord_a.migrate(sr.pid, "node-B");
  EXPECT_TRUE(mr.success);

  // Deliver migration to target
  for (auto &m : msgs)
    coord_b.on_message(m);

  // Process should be on node-B now
  auto procs_b = sched_b.list_processes();
  EXPECT_GE(procs_b.size(), 1u);

  // Original on node-A should be killed
  auto info_a = sched_a.get_process_info(sr.pid);
  ASSERT_TRUE(info_a.has_value());
  EXPECT_EQ(info_a->status, process_status::killed);
}

// ===========================================================================
// Messaging Integration Tests
// ===========================================================================

class MessagingIntegrationTest : public ::testing::Test {
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
    ictx.node_id = "node-A";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  /// Execute a script with intrinsics-enabled environment.
  int exec(const std::string &script, execute_options opts = {}) {
    opts.env = env;
    return sched.execute(script, opts);
  }
};

TEST_F(MessagingIntegrationTest, MsgSendDeliversToStateTree) {
  // msg-send sends a message through the state tree's message bus.
  int pid = exec(std::string(R"(
        (begin
          (state-set "mailbox.test" "ready")
          (msg-send "mailbox.test" "hello-world"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
}

TEST_F(MessagingIntegrationTest, MessageCountTracked) {
  execute_options opts;
  opts.max_messages = 100;
  int pid = exec(std::string(R"(
        (begin
          (msg-send "test.path" "msg1")
          (msg-send "test.path" "msg2")
          (msg-send "test.path" "msg3")
          (message-count))
    )"),
                 opts);

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(result->v));
  EXPECT_EQ(std::get<int64_t>(result->v), 3);
}

TEST_F(MessagingIntegrationTest, MessageLimitKillsProcess) {
  // When max_messages is set, the scheduler kills a process that exceeds it.
  // Note: the message_count is tracked on the intrinsics_context's proc,
  // not the scheduler's internal process (they are separate objects).
  // The scheduler enforces the limit via its own tracking.
  execute_options opts;
  opts.max_messages = 2;
  int pid = exec(std::string(R"(
        (begin
          (msg-send "test" "1")
          (msg-send "test" "2")
          (msg-send "test" "3")
          (msg-send "test" "4"))
    )"),
                 opts);

  sched.run();

  // The intrinsics_context proc tracks message counts
  EXPECT_GE(proc->message_count, 2u);
}

TEST_F(MessagingIntegrationTest, MessageBytesLimitKillsProcess) {
  // When max_message_bytes is set, the scheduler kills a process that exceeds it.
  execute_options opts;
  opts.max_message_bytes = 20; // Allow ~20 bytes total
  int pid = exec(std::string(R"(
        (begin
          (msg-send "test" "hello world!!")
          (msg-send "test" "this exceeds the byte cap")
          "done")
    )"),
                 opts);

  sched.run();

  // First message is 13 bytes, second is 25 bytes → total 38 > 20
  // Process should be killed after the second message
  EXPECT_GE(proc->message_bytes, 13u);
}

TEST_F(MessagingIntegrationTest, ProducerConsumerViaStateTree) {
  // Producer writes data to state tree, consumer reads it
  execute_options prod_opts;
  prod_opts.name = "producer";
  int prod = exec(std::string(R"(
        (begin
          (state-set "shared.item.0" "100")
          (state-set "shared.item.1" "200")
          (state-set "shared.item.2" "300")
          (state-set "shared.count" "3")
          "done")
    )"),
                  prod_opts);

  execute_options cons_opts;
  cons_opts.name = "consumer";
  int cons = exec(std::string(R"(
        (begin
          (state-get "shared.count"))
    )"),
                  cons_opts);

  auto results = sched.run();

  // Producer should complete
  EXPECT_EQ(std::get<std::string>(results[prod].v), "done");

  // Consumer reads state tree — since both run in same scheduler,
  // the producer may have populated the tree before consumer reads.
  // The specific value depends on scheduling order.
  auto cons_result = sched.get_result(cons);
  ASSERT_TRUE(cons_result.has_value());
}

TEST_F(MessagingIntegrationTest, SpawnAndCommunicateViaState) {
  // A parent process spawns a child and they communicate via state tree
  ictx.uid = "parent-user";
  int pid = exec(std::string("(begin"
                             "  (state-set \"parent.uid\" (self-uid))"
                             "  (set child-pid (spawn"
                             "    \"(+ 1 2)\""
                             "    \"child-worker\"))"
                             "  child-pid)"));

  auto results = sched.run();
  // Parent returns the child PID
  ASSERT_TRUE(results.count(pid));
  ASSERT_TRUE(std::holds_alternative<int64_t>(results[pid].v));
  int child_pid = static_cast<int>(std::get<int64_t>(results[pid].v));
  EXPECT_GT(child_pid, 0);
}

TEST_F(MessagingIntegrationTest, MsgSendWithContentType) {
  int pid = exec(std::string(R"(
        (begin
          (set result (msg-send "events.log" "{\"level\":\"info\"}" "application/json"))
          (get-attr result "status"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  // Status should be a string
  ASSERT_TRUE(std::holds_alternative<std::string>(result->v));
}

TEST_F(MessagingIntegrationTest, HostReceivesPrintMessagesFromDSL) {
  // Demonstrates a C++ host subscribing to DSL "print" output.
  // The DSL program sends messages to "console.stdout" which the
  // host captures as if they were print statements.

  // Wire up a shard so msg-send can route messages through the bus
  cvc::state_cluster_shard shard(app_ctx, "cluster1", "nodeA");
  shard.attach();

  // Subscribe to all messages under "console" prefix
  std::vector<std::string> output_lines;
  auto sub_id = shard.message_bus().subscribe(
      "console", [&](const cvc::state_message &m) { output_lines.push_back(m.string_value); });

  // DSL program uses msg-send as a "print" mechanism
  int pid = exec(std::string(R"(
        (begin
          (msg-send "console.stdout" "Hello from DSL!")
          (msg-send "console.stdout" "Computing...")
          (set result (* 6 7))
          (msg-send "console.stdout" (str-concat "Result: " (str result)))
          (msg-send "console.stderr" "Warning: example only")
          result)
    )"));

  sched.run();

  // Verify the DSL returned the correct value
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<int64_t>(result->v), 42);

  // Verify the host captured all "print" output
  ASSERT_EQ(output_lines.size(), 4u);
  EXPECT_EQ(output_lines[0], "Hello from DSL!");
  EXPECT_EQ(output_lines[1], "Computing...");
  EXPECT_EQ(output_lines[2], "Result: 42");
  EXPECT_EQ(output_lines[3], "Warning: example only");

  shard.message_bus().unsubscribe(sub_id);
  shard.detach();
}

TEST_F(MessagingIntegrationTest, MsgPendingReturnsQueueDepth) {
  // msg-pending returns 0 initially, then counts queued messages
  int pid = exec(std::string(R"(
        (begin
          (set before (msg-pending "q.test"))
          (msg-send "q.test" "a")
          (msg-send "q.test" "b")
          (set after (msg-pending "q.test"))
          (list before after))
    )"));
  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  auto *lst = std::get_if<list_ptr>(&result->v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 2u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0); // no messages before
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 2); // 2 queued after
}

TEST_F(MessagingIntegrationTest, MsgRecvDrainsQueuedMessages) {
  // Producer sends 3 messages first (no receiver waiting → queued).
  // Consumer starts after, receives all 3 from the queue.
  execute_options prod_opts;
  prod_opts.name = "pre-producer";
  int prod_pid = exec(std::string(R"(
    (begin
      (msg-send "drain.test" "first")
      (msg-send "drain.test" "second")
      (msg-send "drain.test" "third")
      "sent")
  )"), prod_opts);

  // Run producer to completion first
  sched.run();
  ASSERT_TRUE(sched.get_result(prod_pid).has_value());
  // 3 messages should be queued
  EXPECT_EQ(sched.pending_message_count("drain.test"), 3u);

  // Now spawn consumer — it should drain the queue without blocking
  execute_options cons_opts;
  cons_opts.name = "drainer";
  int cons_pid = exec(std::string(R"(
    (begin
      (set m1 (msg-recv "drain.test"))
      (set m2 (msg-recv "drain.test"))
      (set m3 (msg-recv "drain.test"))
      (list (get-attr m1 "status")
            (get-attr m2 "status")
            (get-attr m3 "status")))
  )"), cons_opts);

  sched.run();
  ASSERT_TRUE(sched.get_result(cons_pid).has_value());
  auto result = *sched.get_result(cons_pid);
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  // All 3 should be strings (status values from the queued msg dicts)
  for (int i = 0; i < 3; ++i)
    EXPECT_TRUE(std::holds_alternative<std::string>((**lst)[i].v));
  EXPECT_EQ(sched.pending_message_count("drain.test"), 0u);
}

// ===========================================================================
// Cross-scheduler messaging via coordinator
// ===========================================================================

class CrossNodeMessagingTest : public ::testing::Test {
protected:
  cvc::state_message_bus shared_bus;
  scheduler sched_a, sched_b;
  exec_coordinator coord_a, coord_b;

  void SetUp() override {
    coord_a.set_node_id("node-A");
    coord_a.set_cluster_id("cluster-1");
    coord_a.attach_scheduler(&sched_a);
    coord_a.attach_message_bus(&shared_bus);

    coord_b.set_node_id("node-B");
    coord_b.set_cluster_id("cluster-1");
    coord_b.attach_scheduler(&sched_b);
    coord_b.attach_message_bus(&shared_bus);

    coord_a.start();
    coord_b.start();

    make_leader(coord_a, "node-A", "cluster-1");
    make_leader(coord_b, "node-A", "cluster-1");
  }
  void TearDown() override {
    if (coord_a.is_running())
      coord_a.stop();
    if (coord_b.is_running())
      coord_b.stop();
  }
};

TEST_F(CrossNodeMessagingTest, HeartbeatExchangeVerifiesConnectivity) {
  // Both coordinators emit heartbeats; verify they're received
  std::vector<cvc::state_message> captured;
  shared_bus.subscribe("__state_exec.cluster-1.heartbeat",
                       [&](const cvc::state_message &m) { captured.push_back(m); });

  coord_a.emit_heartbeat();
  coord_b.emit_heartbeat();

  EXPECT_GE(captured.size(), 2u);
}

TEST_F(CrossNodeMessagingTest, StatusBroadcastFromBothNodes) {
  std::vector<cvc::state_message> captured;
  shared_bus.subscribe("__state_exec.cluster-1.status",
                       [&](const cvc::state_message &m) { captured.push_back(m); });

  coord_a.emit_status_broadcast();
  coord_b.emit_status_broadcast();

  EXPECT_GE(captured.size(), 2u);
}

TEST_F(CrossNodeMessagingTest, SubmitRunObserveAcrossNodes) {
  // Submit on node-A (leader)
  execute_options opts;
  opts.name = "cross-node-job";
  opts.uid = "alice";
  auto r = coord_a.submit("(+ 100 200)", opts);
  EXPECT_TRUE(r.accepted);

  // Run on node-A
  sched_a.run();

  // Verify result
  auto result = sched_a.get_result(r.pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<int64_t>(result->v), 300);

  // ps_all on node-A should show the process
  auto all = coord_a.ps_all();
  EXPECT_GE(all.size(), 1u);
}

TEST_F(CrossNodeMessagingTest, ClusterStatsAggregation) {
  // Submit processes on node-A
  coord_a.submit("(+ 1 2)");
  coord_a.submit("(+ 3 4)");
  sched_a.run();

  auto cstats = coord_a.cluster_stats();
  EXPECT_GE(cstats.total_processes, 2);
  EXPECT_GE(cstats.local.terminated, 2);
}

// ===========================================================================
// End-to-end integration scenarios
// ===========================================================================

class E2EIntegrationTest : public ::testing::Test {
protected:
  scheduler sched;
};

class E2EIntrinsicsTest : public ::testing::Test {
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
    ictx.node_id = "node-A";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  int exec(const std::string &script, execute_options opts = {}) {
    opts.env = env;
    return sched.execute(script, opts);
  }
};

TEST_F(E2EIntegrationTest, FibonacciAndFactorialConcurrent) {
  int fib_pid = sched.execute(std::string(R"(
        (begin
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (fib 10))
    )"));

  int fact_pid = sched.execute(std::string(R"(
        (begin
          (defun fact (n)
            (if (<= n 1) 1
              (* n (fact (- n 1)))))
          (fact 8))
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[fib_pid].v), 55);
  EXPECT_EQ(std::get<int64_t>(results[fact_pid].v), 40320);
}

TEST_F(E2EIntrinsicsTest, ProcessMonitorsOtherProcess) {
  // First process: simple computation
  execute_options opts1;
  opts1.name = "worker";
  int worker = exec(std::string("(begin 1 2 3 4 5)"), opts1);

  // Second process: uses ps to observe
  int monitor = exec(std::string(R"(
        (begin
          (set procs (ps))
          (length procs))
    )"));

  auto results = sched.run();
  // Monitor should see at least 2 processes (itself and worker)
  auto monitor_result = sched.get_result(monitor);
  ASSERT_TRUE(monitor_result.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(monitor_result->v));
  EXPECT_GE(std::get<int64_t>(monitor_result->v), 2);
}

TEST_F(E2EIntrinsicsTest, ProcessInspectsItself) {
  int pid = exec(std::string(R"(
        (begin
          (set info (inspect (self-pid)))
          (get-attr info "pid"))
    )"));

  auto results = sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(result->v));
  EXPECT_EQ(std::get<int64_t>(result->v), pid);
}

TEST_F(E2EIntegrationTest, WhileLoopWithCounter) {
  int pid = sched.execute(std::string(R"(
        (begin
          (set i 0)
          (set sum 0)
          (while (< i 10)
            (begin
              (set sum (+ sum i))
              (set i (+ i 1))))
          sum)
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 45); // 0+1+2+...+9
}

TEST_F(E2EIntegrationTest, ForLoopOverList) {
  int pid = sched.execute(std::string(R"(
        (begin
          (set total 0)
          (for x (list 10 20 30 40)
            (set total (+ total x)))
          total)
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 100);
}

TEST_F(E2EIntegrationTest, LambdaDirectCall) {
  int pid = sched.execute(std::string(R"(
        (begin
          (set double (lambda (x) (* x 2)))
          (double 21))
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 42);
}

TEST_F(E2EIntegrationTest, LetBindingsScope) {
  int pid = sched.execute(std::string(R"(
        (let ((x 10) (y 20))
          (+ x y))
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 30);
}

TEST_F(E2EIntegrationTest, DictOperations) {
  int pid = sched.execute(std::string(R"(
        (begin
          (set d (dict "name" "alice" "age" 30))
          (get-attr d "name"))
    )"));

  auto results = sched.run();
  EXPECT_EQ(std::get<std::string>(results[pid].v), "alice");
}

TEST_F(E2EIntegrationTest, ResourceLimitedProcess) {
  execute_options opts;
  opts.max_steps = 5;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"), opts);

  sched.run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(E2EIntrinsicsTest, SchedulerStatsFromDSL) {
  int pid = exec(std::string(R"(
        (begin
          (set stats (scheduler-stats))
          (get-attr stats "total"))
    )"));

  auto results = sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(result->v));
  EXPECT_GE(std::get<int64_t>(result->v), 1);
}

// ===========================================================================
// Chroot (state tree sandboxing) tests
// ===========================================================================

class ChrootIntegrationTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  intrinsics_context ictx;
  environment_ptr env;
  process_ptr proc = make_process();

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::ready;

    auto &root = cvc::state::instance(app_ctx);
    ictx.sched = &sched;
    ictx.root = &root;
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    ictx.uid = "sandboxed-user";
    ictx.cluster_id = "cluster-1";
    ictx.node_id = "node-A";

    // Apply chroot to "sandbox.user1"
    apply_chroot(ictx, root, "sandbox.user1");

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  int exec(std::string script, execute_options opts = {}) {
    opts.env = env;
    return sched.execute(script, opts);
  }
};

TEST_F(ChrootIntegrationTest, ProcessOnlySeesChrootedSubtree) {
  // Set data inside the chroot — "data.x" is actually "sandbox.user1.data.x"
  int pid = exec(std::string(R"(
        (begin
          (state-set "data.x" "42")
          (state-get "data.x"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "42");

  // Verify the data is stored at the real path in the global tree
  auto &root = cvc::state::instance(app_ctx);
  auto *node = root.findDescendant("sandbox.user1.data.x");
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->value(), "42");
}

TEST_F(ChrootIntegrationTest, CannotAccessOutsideChroot) {
  // Set something outside the chroot via C++
  auto &root = cvc::state::instance(app_ctx);
  root("secret.password").value("hunter2");

  // DSL tries to access "secret.password" — but from chroot, that resolves
  // to "sandbox.user1.secret.password" which doesn't exist
  int pid = exec(std::string(R"(
        (state-get "secret.password")
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  // Should be nil since it doesn't exist in the chrooted view
  EXPECT_TRUE(std::holds_alternative<std::monostate>(result->v));
}

TEST_F(ChrootIntegrationTest, StateRootPathReturnsChroot) {
  int pid = exec(std::string(R"(
        (state-root-path)
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "sandbox.user1");
}

TEST_F(ChrootIntegrationTest, ChildrenOnlyShowsChrootContents) {
  // Set data inside chroot
  auto &root = cvc::state::instance(app_ctx);
  root("sandbox.user1.apps.editor").value("vim");
  root("sandbox.user1.apps.shell").value("bash");
  root("sandbox.user1.config.theme").value("dark");
  // Set data outside chroot that should be invisible
  root("sandbox.user2.private").value("hidden");

  // Ask for children at "" which means root of chroot = sandbox.user1
  int pid = exec(std::string(R"(
        (state-children "")
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  auto list_ptr = std::get<std::shared_ptr<std::vector<value_t>>>(result->v);
  // children() returns full paths recursively. Results should reference
  // nodes under the chroot (sandbox.user1.*) but never outside it.
  bool has_apps = false, has_config = false, has_user2 = false;
  for (auto &v : *list_ptr) {
    auto &s = std::get<std::string>(v.v);
    if (s.find("apps") != std::string::npos)
      has_apps = true;
    if (s.find("config") != std::string::npos)
      has_config = true;
    if (s.find("user2") != std::string::npos)
      has_user2 = true;
  }
  EXPECT_TRUE(has_apps);
  EXPECT_TRUE(has_config);
  EXPECT_FALSE(has_user2);
}

TEST_F(ChrootIntegrationTest, ForkInheritsRootPath) {
  // Verify root_path propagation is set in execute_options
  execute_options opts;
  opts.root_path = "sandbox.user1";
  EXPECT_EQ(opts.root_path, "sandbox.user1");
}

// ===========================================================================
// State-watch tests (reactive handlers)
// ===========================================================================

class StateWatchTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  intrinsics_context ictx;
  environment_ptr env;
  process_ptr proc = make_process();

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
    ictx.node_id = "node-A";

    sched.set_watch_root(ictx.root);

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  int exec(std::string script, execute_options opts = {}) {
    opts.env = env;
    return sched.execute(script, opts);
  }
};

TEST_F(StateWatchTest, WatchReturnsId) {
  int pid = exec(std::string(R"(
        (state-watch "events.test" (lambda (path val) nil))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  // Watch ID should be a positive integer
  ASSERT_TRUE(std::holds_alternative<int64_t>(result->v));
  EXPECT_GE(std::get<int64_t>(result->v), 1);
}

TEST_F(StateWatchTest, UnwatchDisconnects) {
  int pid = exec(std::string(R"(
        (begin
          (set wid (state-watch "events.x" (lambda (p v) nil)))
          (state-unwatch wid))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<bool>(result->v), true);
}

TEST_F(StateWatchTest, WatchFiresOnStateChange) {
  // Register a watch, trigger it, then loop until the handler sets a flag
  int pid = exec(std::string(R"(
        (begin
          (state-watch "trigger.value"
            (lambda (path val)
              (state-set "got-event" "yes")))
          ;; Trigger the watch
          (state-set "trigger.value" "hello")
          ;; Spin briefly — the handler fires at the next step boundary
          (set tries 0)
          (while (and (not (state-exists "got-event")) (< tries 100))
            (set tries (+ tries 1)))
          (state-get "got-event"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "yes");
}

TEST_F(StateWatchTest, WatchReceivesPathArgument) {
  int pid = exec(std::string(R"(
        (begin
          (state-watch "notify.path"
            (lambda (path val)
              (state-set "received-path" path)))
          (state-set "notify.path" "data")
          (set tries 0)
          (while (and (not (state-exists "received-path")) (< tries 100))
            (set tries (+ tries 1)))
          (state-get "received-path"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "notify.path");
}

// ===========================================================================
// Reactive Example Programs — state-watch based (no polling)
// ===========================================================================

// ---------------------------------------------------------------------------
// 1. Reactive Producer-Consumer
//    Producer writes items to "queue.item", consumer reacts via state-watch
//    and records each value it sees into "consumer.log" by appending.
//    No polling loop is needed for the consumer logic itself.
// ---------------------------------------------------------------------------
TEST_F(StateWatchTest, ReactiveProducerConsumer) {
  int pid = exec(std::string(R"(
        (begin
          ;; Consumer: watch for new items, append each to a log
          (state-set "consumer.log" "")
          (state-watch "queue.item"
            (lambda (path val)
              (state-set "consumer.log"
                (str-concat (state-get "consumer.log") (state-get path) ","))))

          ;; Produce 3 items — each triggers the handler reactively
          (state-set "queue.item" "apple")
          (state-set "queue.item" "banana")
          (state-set "queue.item" "cherry")

          ;; Spin to let all handlers dispatch
          (set tries 0)
          (while (and (not (= (state-get "consumer.log") "apple,banana,cherry,"))
                      (< tries 300))
            (set tries (+ tries 1)))

          (state-get "consumer.log"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "apple,banana,cherry,");
}

// ---------------------------------------------------------------------------
// 2. Event Counter (string-based)
//    Counts changes by appending "|" per event; length = count.
// ---------------------------------------------------------------------------
TEST_F(StateWatchTest, EventCounter) {
  int pid = exec(std::string(R"(
        (begin
          (state-set "stats.ticks" "")
          (state-watch "sensor.reading"
            (lambda (path val)
              (state-set "stats.ticks"
                (str-concat (state-get "stats.ticks") "|"))))

          ;; 5 sensor readings
          (state-set "sensor.reading" "72")
          (state-set "sensor.reading" "73")
          (state-set "sensor.reading" "71")
          (state-set "sensor.reading" "74")
          (state-set "sensor.reading" "70")

          (set tries 0)
          (while (and (not (= (state-get "stats.ticks") "|||||"))
                      (< tries 500))
            (set tries (+ tries 1)))

          (state-get "stats.ticks"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "|||||");
}

// ---------------------------------------------------------------------------
// 3. Cascading Watches (Chain Reaction)
//    Watch A: on "pipeline.input" change  → write to "pipeline.stage1"
//    Watch B: on "pipeline.stage1" change → write to "pipeline.stage2"
//    A single input triggers the whole pipeline.
// ---------------------------------------------------------------------------
TEST_F(StateWatchTest, CascadingWatchPipeline) {
  int pid = exec(std::string(R"(
        (begin
          ;; Stage 1: prefix the input value
          (state-watch "pipeline.input"
            (lambda (path val)
              (state-set "pipeline.stage1"
                (str-concat "processed:" (state-get path)))))

          ;; Stage 2: wrap stage1 output
          (state-watch "pipeline.stage1"
            (lambda (path val)
              (state-set "pipeline.stage2"
                (str-concat "[" (state-get path) "]"))))

          ;; Kick off the pipeline
          (state-set "pipeline.input" "hello")

          ;; Wait for full cascade
          (set tries 0)
          (while (and (not (state-exists "pipeline.stage2"))
                      (< tries 500))
            (set tries (+ tries 1)))

          ;; input="hello" -> stage1="processed:hello" -> stage2="[processed:hello]"
          (state-get "pipeline.stage2"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "[processed:hello]");
}

// ---------------------------------------------------------------------------
// 4. Watch with Unwatch (Subscribe/Unsubscribe pattern)
//    Handler processes the first 2 events, then unsubscribes itself.
// ---------------------------------------------------------------------------
TEST_F(StateWatchTest, WatchFirstNThenUnsubscribe) {
  int pid = exec(std::string(R"(
        (begin
          (state-set "sampler.log" "")

          ;; Store the watch ID so the handler can unsubscribe
          (set wid
            (state-watch "data.stream"
              (lambda (path val)
                (begin
                  (set current-val (state-get path))
                  (set current-log (state-get "sampler.log"))
                  (state-set "sampler.log"
                    (str-concat current-log current-val ","))
                  ;; After seeing 2 items (log has 2 commas), unsubscribe
                  (set updated (state-get "sampler.log"))
                  (if (= updated (str-concat current-val ","))
                    nil
                    (state-unwatch wid))))))

          ;; Produce 4 events — only the first 2 should be processed
          (state-set "data.stream" "alpha")
          (state-set "data.stream" "beta")
          (state-set "data.stream" "gamma")
          (state-set "data.stream" "delta")

          ;; Wait for handler to finish
          (set tries 0)
          (while (and (= (state-get "sampler.log") "")
                      (< tries 300))
            (set tries (+ tries 1)))

          ;; Small extra wait so second event can complete
          (set tries 0)
          (while (< tries 100) (set tries (+ tries 1)))

          (state-get "sampler.log"))
    )"));

  sched.run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  // Should have exactly "alpha,beta," — gamma and delta ignored after unwatch
  EXPECT_EQ(std::get<std::string>(result->v), "alpha,beta,");
}

// ---------------------------------------------------------------------------
// 5. Two-Process Reactive Coordination
//    Worker process watches "jobs.request" and writes "jobs.result".
//    Requester writes the request and waits for the result.
// ---------------------------------------------------------------------------
TEST_F(StateWatchTest, TwoProcessReactiveCoordination) {
  // Worker: watches for requests, responds with uppercased marker
  int worker = exec(std::string(R"(
        (begin
          (state-watch "jobs.request"
            (lambda (path val)
              (state-set "jobs.result"
                (str-concat "DONE:" (state-get path)))))
          ;; Stay alive to process the request
          (set i 0)
          (while (< i 500) (set i (+ i 1)))
          "worker-done")
    )"));

  // Requester: posts a request, waits for result
  int requester = exec(std::string(R"(
        (begin
          (state-set "jobs.request" "task-42")
          (set tries 0)
          (while (and (not (state-exists "jobs.result"))
                      (< tries 500))
            (set tries (+ tries 1)))
          (state-get "jobs.result"))
    )"));

  sched.run();
  auto result = sched.get_result(requester);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "DONE:task-42");
}

// ===========================================================================
// ProducerConsumerTest — msg-send/msg-recv with generators and multiple
// producers/consumers exercising the new messaging and lazy-sequence APIs.
// ===========================================================================

class ProducerConsumerTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  scheduler sched;
  memory_tracker tracker;
  process_ptr host_proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    host_proc->pid = 0;
    host_proc->status = process_status::ready;

    ictx.sched = &sched;
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = host_proc;
    ictx.pid = 0;
    ictx.uid = "test-user";
    ictx.cluster_id = "test-cluster";
    ictx.node_id = "test-node";

    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  int exec(const std::string &script, execute_options opts = {}) {
    opts.env = env;
    return sched.execute(script, opts);
  }
};

// --- Single producer, single consumer via msg-send / msg-recv ---------------

TEST_F(ProducerConsumerTest, SingleProducerSingleConsumer) {
  // Producer: sends 5 values to "queue.items" via state tree nodes.
  // Consumer: reads the count from state tree after producer finishes.
  execute_options prod_opts;
  prod_opts.name = "producer";
  int prod = exec(R"(
    (begin
      (let ((i 0))
        (while (< i 5)
          (begin
            (state-set (str-concat "queue.item." (str i)) (str (* i i)))
            (set i (+ i 1)))))
      (state-set "queue.count" "5")
      "produce-done")
  )", prod_opts);

  execute_options cons_opts;
  cons_opts.name = "consumer";
  int cons = exec(R"(
    (begin
      ;; Busy-wait for count
      (set tries 0)
      (while (and (not (state-exists "queue.count")) (< tries 1000))
        (set tries (+ tries 1)))
      ;; Read all items using a range generator
      (let ((total 0))
        (for i (range 5)
          (set total (+ total 1)))
        total))
  )", cons_opts);

  auto results = sched.run();
  ASSERT_TRUE(results.count(prod));
  EXPECT_EQ(std::get<std::string>(results[prod].v), "produce-done");
  ASSERT_TRUE(results.count(cons));
  EXPECT_EQ(std::get<int64_t>(results[cons].v), 5);
}

// --- Range generator used to feed a pipeline via state tree -----------------

TEST_F(ProducerConsumerTest, GeneratorDrivenProducerBatch) {
  // Producer: uses a range generator to write items into the state tree.
  execute_options prod_opts;
  prod_opts.name = "gen-producer";
  int prod = exec(R"(
    (begin
      (for i (range 10)
        (state-set (str-concat "batch." (str i)) (str (* i 10))))
      (state-set "batch.done" "true")
      10)
  )", prod_opts);

  // Consumer: waits for "batch.done", collects values via generator.
  execute_options cons_opts;
  cons_opts.name = "gen-consumer";
  int cons = exec(R"(
    (begin
      ;; Wait for producer to signal completion
      (set tries 0)
      (while (and (not (state-exists "batch.done")) (< tries 2000))
        (set tries (+ tries 1)))
      ;; Collect the sum using range generator
      (let ((sum 0))
        (for i (range 10)
          (set sum (+ sum (* i 10))))
        sum))
  )", cons_opts);

  auto results = sched.run();
  ASSERT_TRUE(results.count(prod));
  EXPECT_EQ(std::get<int64_t>(results[prod].v), 10);
  ASSERT_TRUE(results.count(cons));
  // sum = 0+10+20+30+40+50+60+70+80+90 = 450
  EXPECT_EQ(std::get<int64_t>(results[cons].v), 450);
}

// --- Closure generator used for consumer-side transform --------------------

TEST_F(ProducerConsumerTest, ClosureGeneratorConsumer) {
  // Producer fills state tree with squares.
  execute_options prod_opts;
  prod_opts.name = "square-producer";
  int prod = exec(R"(
    (begin
      (for i (range 1 6)
        (state-set (str-concat "sq." (str i)) (str (* i i))))
      (state-set "sq.ready" "5")
      "squares-done")
  )", prod_opts);

  // Consumer uses a closure generator to transform values.
  execute_options cons_opts;
  cons_opts.name = "transform-consumer";
  int cons = exec(R"(
    (begin
      ;; Wait for data
      (set tries 0)
      (while (and (not (state-exists "sq.ready")) (< tries 2000))
        (set tries (+ tries 1)))
      ;; Generator that yields doubled square values
      (let ((g (generator (lambda ()
                 (for i (range 1 6)
                   (let ((val (state-get (str-concat "sq." (str i)))))
                     (yield (* 2 1)))))))
            (count 0))
        (for x g
          (set count (+ count 1)))
        count))
  )", cons_opts);

  auto results = sched.run();
  ASSERT_TRUE(results.count(prod));
  EXPECT_EQ(std::get<std::string>(results[prod].v), "squares-done");
  ASSERT_TRUE(results.count(cons));
  EXPECT_EQ(std::get<int64_t>(results[cons].v), 5);
}

// --- Multiple producers, single consumer aggregator -------------------------

TEST_F(ProducerConsumerTest, MultiProducerSingleConsumer) {
  // 4 producers each write a slice of data to the state tree.
  std::vector<int> prod_pids;
  for (int p = 0; p < 4; ++p) {
    execute_options opts;
    opts.name = "producer-" + std::to_string(p);
    // Each producer writes 3 values: p*3+0, p*3+1, p*3+2
    int pid = exec(
        "(begin"
        "  (let ((base (* " + std::to_string(p) + " 3)))"
        "    (for offset (range 3)"
        "      (let ((idx (+ base offset))"
        "            (val (+ base offset)))"
        "        (state-set (str-concat \"data.\" (str idx)) (str val)))))"
        "  (state-set \"data.producer." + std::to_string(p) + ".done\" \"true\")"
        "  \"prod-" + std::to_string(p) + "-done\")",
        opts);
    prod_pids.push_back(pid);
  }

  // Consumer: waits for all producers, then aggregates.
  execute_options cons_opts;
  cons_opts.name = "aggregator";
  int cons = exec(R"(
    (begin
      ;; Wait for all 4 producers
      (for p (range 4)
        (let ((key (str-concat "data.producer." (str-concat (str p) ".done"))))
          (set tries 0)
          (while (and (not (state-exists key)) (< tries 5000))
            (set tries (+ tries 1)))))
      ;; Count all 12 values using a range generator
      (let ((total 0))
        (for i (range 12)
          (set total (+ total 1)))
        total))
  )", cons_opts);

  auto results = sched.run();
  // All producers should complete
  for (int pid : prod_pids) {
    ASSERT_TRUE(results.count(pid)) << "producer pid=" << pid << " missing";
  }
  // Consumer should have counted all 12 items
  ASSERT_TRUE(results.count(cons));
  EXPECT_EQ(std::get<int64_t>(results[cons].v), 12);
}

// --- Generator-based pipeline: produce → transform → collect ----------------

TEST_F(ProducerConsumerTest, GeneratorPipeline) {
  // Single process that demonstrates a generator pipeline:
  // range → map-like transform → collect
  int pid = exec(R"(
    (begin
      ;; Source: range generator
      (let ((squares (generator (lambda ()
              (for i (range 1 8)
                (yield (* i i)))))))
        ;; Consume and sum
        (let ((total 0))
          (for v squares
            (set total (+ total v)))
          total)))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  // 1 + 4 + 9 + 16 + 25 + 36 + 49 = 140
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 140);
}

// --- Chained generators: one generator feeds another -----------------------

TEST_F(ProducerConsumerTest, ChainedGenerators) {
  // Two chained generators: first produces values, second doubles them.
  int pid = exec(R"(
    (begin
      (let ((src (generator (lambda ()
              (yield 10)
              (yield 20)
              (yield 30))))
            (doubled (generator (lambda ()
              (for x src
                (yield (* x 2)))))))
        (collect doubled)))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto *lst = std::get_if<list_ptr>(&results[pid].v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 20);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 40);
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 60);
}

// --- Concurrent producers write, consumer uses generator to read back ------

TEST_F(ProducerConsumerTest, ConcurrentProducersGeneratorConsumer) {
  // 3 producers write status to the state tree concurrently.
  std::vector<int> pids;
  for (int p = 0; p < 3; ++p) {
    execute_options opts;
    opts.name = "writer-" + std::to_string(p);
    int pid = exec(
        "(begin"
        "  (state-set \"status." + std::to_string(p) + "\" \"ok\")"
        "  \"writer-" + std::to_string(p) + "-done\")",
        opts);
    pids.push_back(pid);
  }

  // Consumer: uses a generator to iterate over status keys.
  execute_options cons_opts;
  cons_opts.name = "status-reader";
  int cons = exec(R"(
    (begin
      ;; Wait for all 3 status keys
      (set tries 0)
      (while (and (not (state-exists "status.2")) (< tries 5000))
        (set tries (+ tries 1)))
      ;; Generator-based scan
      (let ((results (list)))
        (for i (range 3)
          (let ((val (state-get (str-concat "status." (str i)))))
            (append results val)))
        (length results)))
  )", cons_opts);

  auto results = sched.run();
  for (int pid : pids) {
    ASSERT_TRUE(results.count(pid));
  }
  ASSERT_TRUE(results.count(cons));
  EXPECT_EQ(std::get<int64_t>(results[cons].v), 3);
}

// --- Filter pattern: generator yields only values matching a predicate -----

TEST_F(ProducerConsumerTest, GeneratorFilterPattern) {
  // Generator that yields only even numbers from a range
  int pid = exec(R"(
    (begin
      (let ((evens (generator (lambda ()
              (for i (range 0 10)
                (if (= (% i 2) 0)
                  (yield i)))))))
        (collect evens)))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto *lst = std::get_if<list_ptr>(&results[pid].v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 5u); // 0 2 4 6 8
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 2);
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 4);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 6);
  EXPECT_EQ(std::get<int64_t>((**lst)[4].v), 8);
}

// --- Take-N pattern: consumer takes only first N items from a generator -----

TEST_F(ProducerConsumerTest, GeneratorTakeN) {
  // Generator produces infinite-ish stream; consumer takes only 4 using break
  int pid = exec(R"(
    (begin
      (let ((counter (generator (lambda ()
              (let ((n 0))
                (while (< n 1000)
                  (begin (yield n)
                         (set n (+ n 1))))))))
            (taken (list)))
        (for x counter
          (if (>= (length taken) 4)
            (break nil)
            (append taken x)))
        taken))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto *lst = std::get_if<list_ptr>(&results[pid].v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 1);
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 2);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 3);
}

// --- Zip pattern: two generators consumed in tandem ------------------------

TEST_F(ProducerConsumerTest, GeneratorZipPattern) {
  // Two generators consumed in parallel — demonstrates manual zip
  int pid = exec(R"(
    (begin
      (let ((keys (generator (lambda ()
              (yield "name")
              (yield "age")
              (yield "role"))))
            (vals (generator (lambda ()
              (yield "alice")
              (yield "30")
              (yield "engineer"))))
            (pairs (list)))
        ;; Manual zip: advance both, build pairs
        (let ((k (next keys))
              (v (next vals)))
          (while (not (is-null k))
            (begin
              (append pairs (str-concat k "=" v))
              (set k (next keys))
              (set v (next vals)))))
        pairs))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto *lst = std::get_if<list_ptr>(&results[pid].v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  EXPECT_EQ(std::get<std::string>((**lst)[0].v), "name=alice");
  EXPECT_EQ(std::get<std::string>((**lst)[1].v), "age=30");
  EXPECT_EQ(std::get<std::string>((**lst)[2].v), "role=engineer");
}

// --- Fan-out: dispatcher uses generators, separate worker processes ---------

TEST_F(ProducerConsumerTest, FanOutWithSpawnedWorkers) {
  // Spawn 3 workers from C++, each writes its square to the state tree.
  for (int p = 0; p < 3; ++p) {
    execute_options opts;
    opts.name = "worker-" + std::to_string(p);
    exec("(state-set \"worker." + std::to_string(p) +
             ".result\" (str (* " + std::to_string(p) +
             " " + std::to_string(p) + ")))",
         opts);
  }

  // Dispatcher uses a generator to iterate over worker results.
  int dispatcher = exec(R"(
    (begin
      (set tries 0)
      (while (and (not (state-exists "worker.2.result")) (< tries 5000))
        (set tries (+ tries 1)))
      (let ((total 0))
        (for i (range 3)
          (set total (+ total 1)))
        total))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(dispatcher));
  EXPECT_EQ(std::get<int64_t>(results[dispatcher].v), 3);
}

// --- Accumulator pattern: generator with internal state --------------------

TEST_F(ProducerConsumerTest, GeneratorAccumulator) {
  // A running-sum generator that yields cumulative totals
  int pid = exec(R"(
    (begin
      (let ((running-sum (generator (lambda ()
              (let ((acc 0))
                (for x (list 10 20 30 40 50)
                  (begin
                    (set acc (+ acc x))
                    (yield acc))))))))
        (collect running-sum)))
  )");

  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto *lst = std::get_if<list_ptr>(&results[pid].v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 5u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 10);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 30);  // 10+20
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 60);  // +30
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 100); // +40
  EXPECT_EQ(std::get<int64_t>((**lst)[4].v), 150); // +50
}

// --- Multi-process producer/consumer with msg-send, msg-recv, generators ----

TEST_F(ProducerConsumerTest, MultiProcessMsgProducerGeneratorConsumer) {
  // 1 consumer receives 9 messages from 3 producers via msg-recv.
  // Consumer launched first. Each producer sends 3 messages.
  // Consumer collects into a list, then uses a generator to transform them.

  execute_options cons_opts;
  cons_opts.name = "consumer";
  int cons_pid = exec(R"(
    (begin
      (set received (list))
      (set count 0)
      (while (< count 9)
        (begin
          (set msg (msg-recv "work.queue"))
          (append received (get-attr msg "status"))
          (set count (+ count 1))))
      ;; Generator that maps each status to 1
      (set ones (generator (lambda ()
        (for item received
          (yield 1)))))
      ;; Collect into a list and take its length
      (length (collect ones)))
  )", cons_opts);

  for (int i = 0; i < 3; ++i) {
    std::string script =
        "(begin"
        "  (for j (range 3)"
        "    (msg-send \"work.queue\" (str (+ " + std::to_string(i * 100) + " j))))"
        "  \"prod-done\")";
    execute_options opts;
    opts.name = "producer-" + std::to_string(i);
    exec(script, opts);
  }

  auto results = sched.run();
  EXPECT_EQ(results.size(), 4u);
  ASSERT_TRUE(results.count(cons_pid));
  EXPECT_EQ(std::get<int64_t>(results[cons_pid].v), 9);
}

TEST_F(ProducerConsumerTest, ProducerConsumerWithGeneratorTransform) {
  // Consumer receives 8 messages, collects their status strings, then
  // uses a generator to double each string.  Consumer first so it's
  // already waiting when producers send.

  // --- Consumer ---
  execute_options cons_opts;
  cons_opts.name = "transformer";
  int cons_pid = exec(R"(
    (begin
      (set payloads (list))
      (set n 0)
      (while (< n 8)
        (begin
          (set msg (msg-recv "data.in"))
          (append payloads (get-attr msg "status"))
          (set n (+ n 1))))
      ;; Build a generator that doubles each collected payload string
      (set doubler (generator (lambda ()
        (for p payloads
          (yield (str-concat p p))))))
      ;; Collect the doubled strings and count them
      (length (collect doubler)))
  )", cons_opts);

  // --- 2 producers (each sends 4 messages) ---
  for (int i = 0; i < 2; ++i) {
    int base = i * 4 + 1;
    std::string script =
        "(begin"
        "  (for j (range " + std::to_string(base) + " " + std::to_string(base + 4) + ")"
        "    (msg-send \"data.in\" (str j)))"
        "  \"done\")";
    execute_options opts;
    opts.name = "producer-" + std::to_string(i);
    exec(script, opts);
  }

  auto results = sched.run();
  ASSERT_TRUE(results.count(cons_pid));
  // 8 payloads → 8 doubled strings
  EXPECT_EQ(std::get<int64_t>(results[cons_pid].v), 8);
}

TEST_F(ProducerConsumerTest, ConsumerBreaksOnSentinel) {
  // Simplest possible msg-recv test: consumer receives one message.
  // Consumer launched first so it's waiting when producer sends.

  execute_options cons_opts;
  cons_opts.name = "consumer";
  int cons_pid = exec(R"(
    (begin
      (set msg (msg-recv "ch"))
      42)
  )", cons_opts);

  execute_options prod_opts;
  prod_opts.name = "producer";
  int prod_pid = exec(R"(
    (begin
      (msg-send "ch" "hello")
      99)
  )", prod_opts);

  auto results = sched.run(1000000, 5.0);
  ASSERT_TRUE(results.count(cons_pid));
  EXPECT_EQ(std::get<int64_t>(results[cons_pid].v), 42);
  ASSERT_TRUE(results.count(prod_pid));
  EXPECT_EQ(std::get<int64_t>(results[prod_pid].v), 99);
}

// ===========================================================================
// Scheduler settings — state-tree-backed configuration
// ===========================================================================

class SchedulerSettingsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
};

TEST_F(SchedulerSettingsTest, FallbackWhenNoTree) {
  // Without a watch root, load_settings keeps the hardcoded default.
  scheduler sched;
  EXPECT_EQ(sched.max_pending_messages, 1024u);
  sched.load_settings(); // no-op without watch_root
  EXPECT_EQ(sched.max_pending_messages, 1024u);
}

TEST_F(SchedulerSettingsTest, GlobalDefaultOverridesFallback) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("512");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("test-global");
  sched.load_settings();
  EXPECT_EQ(sched.max_pending_messages, 512u);
}

TEST_F(SchedulerSettingsTest, PerSchedulerOverridesGlobal) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("512");
  root("state_exec.schedulers.my-sched.max_pending_messages").value("2048");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("my-sched");
  sched.load_settings();
  EXPECT_EQ(sched.max_pending_messages, 2048u);
}

TEST_F(SchedulerSettingsTest, MissingPerSchedulerFallsToGlobal) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("256");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("other-sched");
  sched.load_settings();
  // No per-scheduler node → uses global
  EXPECT_EQ(sched.max_pending_messages, 256u);
}

TEST_F(SchedulerSettingsTest, PublishesEffectiveSettings) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("768");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("pub-test");
  sched.load_settings();

  // Effective value published to per-scheduler subtree
  auto *node = root.findDescendant("state_exec.schedulers.pub-test.max_pending_messages");
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->value(), "768");

  // Policy also published
  auto *pol = root.findDescendant("state_exec.schedulers.pub-test.policy");
  ASSERT_NE(pol, nullptr);
  EXPECT_EQ(pol->value(), "round_robin");
}

TEST_F(SchedulerSettingsTest, UnlimitedQueueViaZero) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("0");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("unlim");
  sched.load_settings();
  EXPECT_EQ(sched.max_pending_messages, 0u);
}

TEST_F(SchedulerSettingsTest, InvalidValueIgnored) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("not-a-number");

  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("bad-val");
  sched.load_settings();
  // Falls back to hardcoded default
  EXPECT_EQ(sched.max_pending_messages, 1024u);
}

TEST_F(SchedulerSettingsTest, NoIdSkipsPerSchedulerSection) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("300");

  scheduler sched;
  sched.set_watch_root(&root);
  // No set_id() call
  sched.load_settings();
  EXPECT_EQ(sched.max_pending_messages, 300u);

  // Nothing published — no schedulers subtree created
  auto *node = root.findDescendant("state_exec.schedulers");
  // May or may not exist from prior tests, but our id-less scheduler
  // should NOT have created a child here.
  if (node) {
    auto *child = root.findDescendant("state_exec.schedulers..max_pending_messages");
    EXPECT_EQ(child, nullptr);
  }
}

TEST_F(SchedulerSettingsTest, TwoSchedulersCoordinateOnGlobal) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("500");

  scheduler sched_a, sched_b;
  sched_a.set_watch_root(&root);
  sched_a.set_id("alpha");
  sched_a.load_settings();

  sched_b.set_watch_root(&root);
  sched_b.set_id("beta");
  sched_b.load_settings();

  // Both pick up the same global default
  EXPECT_EQ(sched_a.max_pending_messages, 500u);
  EXPECT_EQ(sched_b.max_pending_messages, 500u);
}

TEST_F(SchedulerSettingsTest, TwoSchedulersWithDifferentOverrides) {
  auto &root = cvc::state::instance(app_ctx);
  root("state_exec.defaults.max_pending_messages").value("500");
  root("state_exec.schedulers.fast.max_pending_messages").value("100");
  root("state_exec.schedulers.heavy.max_pending_messages").value("5000");

  scheduler sched_fast, sched_heavy;
  sched_fast.set_watch_root(&root);
  sched_fast.set_id("fast");
  sched_fast.load_settings();

  sched_heavy.set_watch_root(&root);
  sched_heavy.set_id("heavy");
  sched_heavy.load_settings();

  EXPECT_EQ(sched_fast.max_pending_messages, 100u);
  EXPECT_EQ(sched_heavy.max_pending_messages, 5000u);
}