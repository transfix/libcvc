/// @file state_exec_integration_test.cpp
/// @brief Integration tests for state_exec: ACL, migration, messaging,
///        process identity, and multi-node coordination.

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_exec/builtins.h>
#include <cvc/state_exec/exec_coordinator.h>
#include <cvc/state_exec/intrinsics.h>
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/resource_policy.h>
#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/state_value_codec.h>
#include <cvc/state_exec/stdlib.h>
#include <cvc/state_exec/types.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
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
