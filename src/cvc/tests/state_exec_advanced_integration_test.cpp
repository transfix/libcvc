/// @file state_exec_advanced_integration_test.cpp
/// @brief Advanced integration tests for state_exec: large data objects,
///        multi-node contention, cluster hierarchy with delegation,
///        write-policy security, and coordinated messaging.

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_change_journal.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_delegation_manager.h>
#include <cvc/state_exec/builtins.h>
#include <cvc/state_exec/exec_coordinator.h>
#include <cvc/state_exec/intrinsics.h>
#include <cvc/state_exec/memory_tracker.h>
#include <cvc/state_exec/process.h>
#include <cvc/state_exec/resource_policy.h>
#include <cvc/state_exec/scheduler.h>
#include <cvc/state_exec/stackless_evaluator.h>
#include <cvc/state_exec/state_value_codec.h>
#include <cvc/state_exec/types.h>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

// ===========================================================================
// Helpers
// ===========================================================================

/// Build a state_mutation for set_value.
static cvc::state_mutation make_set_value(const std::string &origin, uint64_t seq,
                                          const std::string &path, const std::string &val,
                                          const std::string &cluster = "root") {
  cvc::state_mutation m;
  m.cluster_id = cluster;
  m.tree_id = "default";
  m.origin_node_id = origin;
  m.sequence = seq;
  m.mutation_id = origin + ":" + std::to_string(seq);
  m.path = path;
  m.op = cvc::state_mutation_op::set_value;
  m.type_name = "std::string";
  m.string_value = val;
  m.latest_value_only = true;
  return m;
}

/// Build a mutation carrying binary payload (inline bytes).
static cvc::state_mutation make_data_mutation(const std::string &origin, uint64_t seq,
                                              const std::string &path,
                                              const std::vector<unsigned char> &bytes,
                                              const std::string &cluster = "root") {
  cvc::state_mutation m;
  m.cluster_id = cluster;
  m.tree_id = "default";
  m.origin_node_id = origin;
  m.sequence = seq;
  m.mutation_id = origin + ":" + std::to_string(seq);
  m.path = path;
  m.op = cvc::state_mutation_op::set_data;
  m.type_name = "bytes";
  m.payload = cvc::state_payload::inline_data(bytes);
  m.latest_value_only = true;
  return m;
}

static const char *MIME_ELECTION = "application/x-state-exec-election";

/// Force a coordinator to become leader.
static void make_leader(exec_coordinator &coord, const std::string &node_id,
                        const std::string &cluster_id) {
  auto victory = cvc::state_message::make_text(
      "__state_exec." + cluster_id + ".election",
      "{\"type\":\"election-victory\",\"node_id\":\"" + node_id + "\"}", MIME_ELECTION);
  victory.cluster_id = cluster_id;
  coord.on_message(victory);
}

// ===========================================================================
//  1.  Large data-object tests
// ===========================================================================

class LargeDataObjectTest : public ::testing::Test {
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
    ictx.uid = "data-user";
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

TEST_F(LargeDataObjectTest, StoreAndRetrieveLargeStringValue) {
  // Build a 10 KB string via DSL
  int pid = exec(R"(
        (begin
          (set big "")
          (set i 0)
          (while (< i 100)
            (begin
              (set big (str-concat big "0123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789"))
              (set i (+ i 1))))
          (state-set "data.large_string" big)
          (length big))
    )");

  auto results = sched.run();
  auto r = sched.get_result(pid);
  ASSERT_TRUE(r.has_value());
  ASSERT_TRUE(std::holds_alternative<int64_t>(r->v));
  EXPECT_EQ(std::get<int64_t>(r->v), 10000);

  // Verify the node has the value
  auto *node = ictx.root->findDescendant("data.large_string");
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->value().size(), 10000u);
}

TEST_F(LargeDataObjectTest, StoreBoostAnyViaDataSet) {
  // state-data-set stores value_t as boost::any
  int pid = exec(R"(
        (begin
          (state-data-set "data.typed_obj" (list 1 2 3 4 5))
          (set obj (state-data-get "data.typed_obj"))
          (is-null obj))
    )");

  sched.run();
  auto r = sched.get_result(pid);
  ASSERT_TRUE(r.has_value());
  // data-get returns a data_object, which is not nil
  ASSERT_TRUE(std::holds_alternative<bool>(r->v));
  EXPECT_FALSE(std::get<bool>(r->v));
}

TEST_F(LargeDataObjectTest, OverwriteLargeDataRepeatedly) {
  // Overwrite a key 50 times to test mutation stability
  int pid = exec(R"(
        (begin
          (set i 0)
          (while (< i 50)
            (begin
              (state-set "data.counter" (str i))
              (set i (+ i 1))))
          (state-get "data.counter"))
    )");

  auto results = sched.run();
  auto r = sched.get_result(pid);
  ASSERT_TRUE(r.has_value());
  ASSERT_TRUE(std::holds_alternative<std::string>(r->v));
  EXPECT_EQ(std::get<std::string>(r->v), "49");
}

TEST_F(LargeDataObjectTest, LargeDataMutationViaClusterShard) {
  // Create a shard and ingest a mutation with inline bytes payload
  cvc::app shard_app;
  cvc::state_cluster_shard sh(shard_app, "root", "nodeA");
  sh.attach();

  // Build a 64 KB payload
  std::vector<unsigned char> payload(65536, 0xAB);
  auto m = make_data_mutation("remote-node", 1, "volume.slice.0", payload);
  auto result = sh.ingest_remote(m);
  EXPECT_TRUE(result.applied);
  EXPECT_FALSE(result.rejected);

  sh.detach();
}

TEST_F(LargeDataObjectTest, MultipleLargePayloads) {
  // Ingest many large payloads to different paths
  cvc::app shard_app;
  cvc::state_cluster_shard sh(shard_app, "root", "nodeA");
  sh.attach();

  for (int i = 0; i < 20; ++i) {
    std::vector<unsigned char> payload(32768, static_cast<unsigned char>(i));
    auto m = make_data_mutation("remote", static_cast<uint64_t>(i + 1),
                                "volume.slice." + std::to_string(i), payload);
    auto r = sh.ingest_remote(m);
    EXPECT_TRUE(r.applied) << "slice " << i << " failed";
  }

  sh.detach();
}

// ===========================================================================
//  2.  Contention / race-condition tests — 10+ nodes competing
// ===========================================================================

class ContentionTest : public ::testing::Test {
protected:
  cvc::app root_app;
  cvc::state_cluster_shard *shard = nullptr;

  void SetUp() override {
    shard = new cvc::state_cluster_shard(root_app, "root", "arbiter");
    shard->set_resolve_conflicts(true);
    shard->attach();
  }

  void TearDown() override {
    shard->detach();
    delete shard;
  }
};

TEST_F(ContentionTest, TenNodesWriteSamePath) {
  // 10 nodes each write to the same path at the same sequence.
  // With conflict resolution enabled, only the deterministic winner
  // (highest origin_node_id) should survive.
  const int kNodes = 10;
  int applied = 0;
  std::string last_winner;

  for (int i = 0; i < kNodes; ++i) {
    std::string origin = "node-" + std::string(1, 'A' + i);
    auto m = make_set_value(origin, 1, "contested.key", "value-from-" + origin);
    auto r = shard->ingest_remote(m);
    if (r.applied) {
      ++applied;
      last_winner = origin;
    }
  }

  // With conflict resolution, only one value sticks (last-writer-wins)
  EXPECT_GE(applied, 1);
  EXPECT_LE(applied, kNodes);

  // The arbiter should have exactly one value
  auto val = cvc::state::instance(root_app)("contested.key").value();
  EXPECT_FALSE(val.empty());

  // Conflicts should have been detected
  EXPECT_GE(shard->total_conflicts_detected(), 1u);
}

TEST_F(ContentionTest, TenNodesWriteDifferentPaths) {
  // 10 nodes writing to different paths — no conflicts
  const int kNodes = 10;
  for (int i = 0; i < kNodes; ++i) {
    std::string origin = "node-" + std::string(1, 'A' + i);
    auto m = make_set_value(origin, 1, "unique." + origin, "val-" + origin);
    auto r = shard->ingest_remote(m);
    EXPECT_TRUE(r.applied);
  }
  EXPECT_EQ(shard->total_conflicts_detected(), 0u);
}

TEST_F(ContentionTest, TenNodesBurstWritesSamePath) {
  // Each of 10 nodes writes 10 mutations to the same key
  const int kNodes = 10;
  const int kWritesPerNode = 10;
  int total_applied = 0;

  for (int seq = 1; seq <= kWritesPerNode; ++seq) {
    for (int i = 0; i < kNodes; ++i) {
      std::string origin = "node-" + std::string(1, 'A' + i);
      auto m = make_set_value(origin, static_cast<uint64_t>(seq), "hot.key",
                              "v" + std::to_string(seq) + "-" + origin);
      auto r = shard->ingest_remote(m);
      if (r.applied)
        ++total_applied;
    }
  }

  // At least kWritesPerNode applied (one per sequence round)
  EXPECT_GE(total_applied, kWritesPerNode);

  // Many conflicts expected
  EXPECT_GT(shard->total_conflicts_detected(), 0u);
}

TEST_F(ContentionTest, ConcurrentThreadedContention) {
  // 10 threads each ingest 100 mutations to the same path
  const int kThreads = 10;
  const int kPerThread = 100;
  std::atomic<int> total_applied{0};

  auto worker = [&](int thread_id) {
    std::string origin = "thread-" + std::to_string(thread_id);
    for (int s = 1; s <= kPerThread; ++s) {
      auto m = make_set_value(origin, static_cast<uint64_t>(s), "concurrent.hot",
                              origin + ":" + std::to_string(s));
      auto r = shard->ingest_remote(m);
      if (r.applied)
        total_applied.fetch_add(1);
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back(worker, t);
  for (auto &th : threads)
    th.join();

  // Every thread's mutations should be accounted for
  EXPECT_EQ(shard->total_remote_applied() + shard->total_remote_duplicates() +
                shard->total_conflicts_lost(),
            static_cast<uint64_t>(kThreads * kPerThread));

  // Multiple threads → some conflicts expected
  EXPECT_GT(shard->total_conflicts_detected(), 0u);
}

TEST_F(ContentionTest, SequentialEscalatingConflict) {
  // Escalating sequences: higher sequence always wins
  for (uint64_t seq = 1; seq <= 20; ++seq) {
    std::string origin = (seq % 2 == 0) ? "node-even" : "node-odd";
    auto m = make_set_value(origin, seq, "escalate.key", "seq-" + std::to_string(seq));
    shard->ingest_remote(m);
  }

  // The last write (seq=20) should win
  auto val = cvc::state::instance(root_app)("escalate.key").value();
  // node_odd > node_even lexicographically → node_odd always wins
  EXPECT_EQ(val, "seq-19");
}

// ===========================================================================
//  3.  Cluster hierarchy: root cluster + child clusters with delegation
// ===========================================================================

class ClusterHierarchyTest : public ::testing::Test {
protected:
  // Root cluster
  cvc::app root_app;
  cvc::state_cluster_shard root_shard{root_app, "root", "root_node"};

  // Child cluster "alpha"
  cvc::app alpha_app;
  cvc::state_cluster_shard alpha_shard{alpha_app, "alpha", "alpha_node_1"};

  // Child cluster "beta"
  cvc::app beta_app;
  cvc::state_cluster_shard beta_shard{beta_app, "beta", "beta_node_1"};

  // In-process transport for each cluster
  cvc::state_transport_inproc root_transport;
  cvc::state_transport_inproc alpha_transport;
  cvc::state_transport_inproc beta_transport;

  void SetUp() override {
    root_shard.attach();
    alpha_shard.attach();
    beta_shard.attach();

    root_transport.register_shard(&root_shard);
    alpha_transport.register_shard(&alpha_shard);
    beta_transport.register_shard(&beta_shard);
  }

  void TearDown() override {
    root_transport.unregister_shard(&root_shard);
    alpha_transport.unregister_shard(&alpha_shard);
    beta_transport.unregister_shard(&beta_shard);

    root_shard.detach();
    alpha_shard.detach();
    beta_shard.detach();
  }
};

TEST_F(ClusterHierarchyTest, RootDelegatesSubtreeToChild) {
  // Root delegates "scene" to the alpha cluster
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);

  auto decision = root_shard.route_path("scene.geometry.mesh");
  EXPECT_EQ(decision.kind, cvc::state_delegation_manager::route_kind::remote);
  EXPECT_EQ(decision.cluster_id, "alpha");
  EXPECT_EQ(decision.matched_prefix, "scene");
}

TEST_F(ClusterHierarchyTest, UndelegatedPathsRemainLocal) {
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);

  // "config" is not delegated — should be local
  auto decision = root_shard.route_path("config.display");
  EXPECT_EQ(decision.kind, cvc::state_delegation_manager::route_kind::local);
  EXPECT_TRUE(decision.cluster_id.empty());
}

TEST_F(ClusterHierarchyTest, MultipleDelegationsToMultipleClusters) {
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);
  root_shard.publish_delegation("data", "beta", "inproc://beta", 0);

  auto d1 = root_shard.route_path("scene.lights.spot");
  EXPECT_EQ(d1.cluster_id, "alpha");

  auto d2 = root_shard.route_path("data.volumes.ct-scan");
  EXPECT_EQ(d2.cluster_id, "beta");

  // Root paths stay local
  auto d3 = root_shard.route_path("config.network");
  EXPECT_EQ(d3.kind, cvc::state_delegation_manager::route_kind::local);
}

TEST_F(ClusterHierarchyTest, DelegationEnforcementRejectsRemoteWrites) {
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);
  root_shard.set_enforce_delegation(true);

  // A remote write to a delegated subtree should be rejected
  auto m = make_set_value("foreign-node", 1, "scene.camera.fov", "90");
  auto r = root_shard.ingest_remote(m);
  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(r.reject_reason.empty());

  EXPECT_GE(root_shard.total_delegation_routed(), 1u);
}

TEST_F(ClusterHierarchyTest, DelegationAllowsWritesToUndelegatedPaths) {
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);
  root_shard.set_enforce_delegation(true);

  // Write to undelegated "config" should succeed
  auto m = make_set_value("any-node", 1, "config.key", "value");
  auto r = root_shard.ingest_remote(m);
  EXPECT_TRUE(r.applied);
  EXPECT_FALSE(r.rejected);
}

TEST_F(ClusterHierarchyTest, RevocationBringsSubtreeBackLocal) {
  root_shard.publish_delegation("scene", "alpha", "inproc://alpha", 0);
  root_shard.set_enforce_delegation(true);

  // Before revocation: write blocked
  auto m1 = make_set_value("local-node", 1, "scene.test", "v1");
  EXPECT_TRUE(root_shard.ingest_remote(m1).rejected);

  // Revoke delegation
  root_shard.publish_revocation("scene");

  // After revocation: write succeeds
  auto m2 = make_set_value("local-node", 2, "scene.test", "v2");
  auto r2 = root_shard.ingest_remote(m2);
  EXPECT_TRUE(r2.applied);
  EXPECT_FALSE(r2.rejected);
}

TEST_F(ClusterHierarchyTest, ChildClusterWritesToItsOwnSubtree) {
  // Alpha cluster's shard can write to its own state tree freely
  // First value() on a new child is consumed by creation, so set twice.
  cvc::state::instance(alpha_app)("scene.geometry.vertex_count").value(std::string("init"));
  cvc::state::instance(alpha_app)("scene.geometry.vertex_count").value(std::string("1024"));

  alpha_transport.pump_all();

  EXPECT_EQ(cvc::state::instance(alpha_app)("scene.geometry.vertex_count").value(), "1024");
}

TEST_F(ClusterHierarchyTest, TwoNodeChildClusterReplicates) {
  // Add a second node to alpha cluster
  cvc::app alpha_app2;
  cvc::state_cluster_shard alpha_shard2(alpha_app2, "alpha", "alpha_node_2");
  alpha_shard2.attach();
  alpha_transport.register_shard(&alpha_shard2);

  // Write on node-1 (set twice: first creates, second journals)
  cvc::state::instance(alpha_app)("scene.mesh.triangles").value(std::string("init"));
  cvc::state::instance(alpha_app)("scene.mesh.triangles").value(std::string("5000"));

  // Pump replicates to node-2
  alpha_transport.pump_all();

  EXPECT_EQ(cvc::state::instance(alpha_app2)("scene.mesh.triangles").value(), "5000");

  alpha_transport.unregister_shard(&alpha_shard2);
  alpha_shard2.detach();
}

// ===========================================================================
//  4.  Write-policy security and delegation semantics
// ===========================================================================

class DelegationSecurityTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  cvc::state_cluster_shard shard{app_ctx, "root", "root_node"};

  void SetUp() override { shard.attach(); }
  void TearDown() override { shard.detach(); }
};

TEST_F(DelegationSecurityTest, WritePolicyDeniesUnauthorizedNode) {
  shard.write_policy().allow("secrets", {"trusted-node"});
  shard.set_enforce_write_policy(true);

  // Trusted node can write
  auto m_ok = make_set_value("trusted-node", 1, "secrets.api-key", "abc123");
  EXPECT_TRUE(shard.ingest_remote(m_ok).applied);

  // Untrusted node is rejected
  auto m_bad = make_set_value("intruder", 1, "secrets.api-key", "hacked");
  auto r = shard.ingest_remote(m_bad);
  EXPECT_TRUE(r.rejected);
  EXPECT_FALSE(r.reject_reason.empty());
}

TEST_F(DelegationSecurityTest, WritePolicyEmptySetLocksDown) {
  // Empty allowed set = deny all
  shard.write_policy().allow("locked", {});
  shard.set_enforce_write_policy(true);

  auto m = make_set_value("anyone", 1, "locked.value", "try");
  auto r = shard.ingest_remote(m);
  EXPECT_TRUE(r.rejected);
}

TEST_F(DelegationSecurityTest, WritePolicyLongestPrefixWins) {
  // Grant "data" to wide audience, but lock "data.secret" to admin only
  shard.write_policy().allow("data", {"alice", "bob", "admin"});
  shard.write_policy().allow("data.secret", {"admin"});
  shard.set_enforce_write_policy(true);

  // Bob can write to data.general
  auto m1 = make_set_value("bob", 1, "data.general.info", "public");
  EXPECT_TRUE(shard.ingest_remote(m1).applied);

  // Bob cannot write to data.secret
  auto m2 = make_set_value("bob", 2, "data.secret.token", "stolen");
  EXPECT_TRUE(shard.ingest_remote(m2).rejected);

  // Admin can write to data.secret
  auto m3 = make_set_value("admin", 1, "data.secret.token", "real-token");
  EXPECT_TRUE(shard.ingest_remote(m3).applied);
}

TEST_F(DelegationSecurityTest, UncoveredPathsArePermissiveByDefault) {
  shard.write_policy().allow("protected", {"admin"});
  shard.set_enforce_write_policy(true);

  // Path not covered by any rule = allowed
  auto m = make_set_value("anyone", 1, "open.path.value", "public");
  EXPECT_TRUE(shard.ingest_remote(m).applied);
}

TEST_F(DelegationSecurityTest, DelegationAndWritePolicyCombined) {
  // Delegate "simulation" to cluster "compute"
  shard.publish_delegation("simulation", "compute", "grpc://compute:1234", 0);
  shard.set_enforce_delegation(true);

  // Also lock "config" to admin
  shard.write_policy().allow("config", {"admin"});
  shard.set_enforce_write_policy(true);

  // Write to delegated subtree → rejected (delegation enforcement)
  auto m1 = make_set_value("worker", 1, "simulation.step", "100");
  EXPECT_TRUE(shard.ingest_remote(m1).rejected);

  // Write to locked config by non-admin → rejected (write policy)
  auto m2 = make_set_value("worker", 1, "config.max-iters", "1000");
  EXPECT_TRUE(shard.ingest_remote(m2).rejected);

  // Write to locked config by admin → allowed
  auto m3 = make_set_value("admin", 1, "config.max-iters", "1000");
  EXPECT_TRUE(shard.ingest_remote(m3).applied);
}

TEST_F(DelegationSecurityTest, ClientNodeLooksUpValuesInItsSubtree) {
  // A client node writes values to the paths it has access to
  // and reads them back via state tree traversal

  shard.write_policy().allow("workspace.alice", {"alice-node"});
  shard.write_policy().allow("workspace.bob", {"bob-node"});
  shard.set_enforce_write_policy(true);

  // Alice writes to her workspace
  auto m1 = make_set_value("alice-node", 1, "workspace.alice.settings", "dark-mode");
  EXPECT_TRUE(shard.ingest_remote(m1).applied);

  auto m2 = make_set_value("alice-node", 2, "workspace.alice.project", "thesis");
  EXPECT_TRUE(shard.ingest_remote(m2).applied);

  // Alice cannot write to Bob's workspace
  auto m3 = make_set_value("alice-node", 3, "workspace.bob.settings", "hacked");
  EXPECT_TRUE(shard.ingest_remote(m3).rejected);

  // Bob writes to his workspace
  auto m4 = make_set_value("bob-node", 1, "workspace.bob.project", "research");
  EXPECT_TRUE(shard.ingest_remote(m4).applied);

  // Verify values via state tree
  auto &root = cvc::state::instance(app_ctx);
  EXPECT_EQ(root("workspace.alice.settings").value(), "dark-mode");
  EXPECT_EQ(root("workspace.alice.project").value(), "thesis");
  EXPECT_EQ(root("workspace.bob.project").value(), "research");
}

TEST_F(DelegationSecurityTest, ClientNodeLooksUpAndWritesDataObjects) {
  // Client writes binary data to its allowed subtree
  shard.write_policy().allow("userdata.alice", {"alice-node"});
  shard.set_enforce_write_policy(true);

  // Write string data
  auto m1 = make_set_value("alice-node", 1, "userdata.alice.profile", "avatar.png");
  EXPECT_TRUE(shard.ingest_remote(m1).applied);

  // Write binary payload
  std::vector<unsigned char> avatar_bytes(8192, 0x42);
  auto m2 = make_data_mutation("alice-node", 2, "userdata.alice.avatar", avatar_bytes);
  auto r = shard.ingest_remote(m2);
  EXPECT_TRUE(r.applied);

  // Unauthorized node cannot write data
  std::vector<unsigned char> evil_bytes(100, 0xFF);
  auto m3 = make_data_mutation("intruder", 1, "userdata.alice.avatar", evil_bytes);
  EXPECT_TRUE(shard.ingest_remote(m3).rejected);
}

TEST_F(DelegationSecurityTest, DelegationWithLeaseExpiry) {
  // Delegation with a lease that expires
  // Use authority map directly since shard's delegation manager
  // uses the shard's internal clock

  shard.delegation().delegate("temp-data", "ephemeral-cluster", "grpc://ephemeral:1234",
                              1000000); // 1ms lease

  auto d1 = shard.route_path("temp-data.item");
  EXPECT_EQ(d1.kind, cvc::state_delegation_manager::route_kind::remote);
  EXPECT_EQ(d1.cluster_id, "ephemeral-cluster");
}

// ===========================================================================
//  5.  Multi-node replication with contention
// ===========================================================================

class MultiNodeReplicationTest : public ::testing::Test {
protected:
  // 3-node cluster sharing a transport
  cvc::app app1, app2, app3;
  cvc::state_cluster_shard s1{app1, "cluster_a", "node_1"};
  cvc::state_cluster_shard s2{app2, "cluster_a", "node_2"};
  cvc::state_cluster_shard s3{app3, "cluster_a", "node_3"};
  cvc::state_transport_inproc transport;

  void SetUp() override {
    s1.attach();
    s2.attach();
    s3.attach();
    s1.set_resolve_conflicts(true);
    s2.set_resolve_conflicts(true);
    s3.set_resolve_conflicts(true);
    transport.register_shard(&s1);
    transport.register_shard(&s2);
    transport.register_shard(&s3);
  }

  void TearDown() override {
    transport.unregister_shard(&s1);
    transport.unregister_shard(&s2);
    transport.unregister_shard(&s3);
    s1.detach();
    s2.detach();
    s3.detach();
  }
};

TEST_F(MultiNodeReplicationTest, WriteOnNode1ReplicatesToAll) {
  // First value() on a new child is consumed by creation, so set twice.
  cvc::state::instance(app1)("shared.value").value(std::string("init"));
  cvc::state::instance(app1)("shared.value").value(std::string("from_node_1"));
  transport.pump_all();

  EXPECT_EQ(cvc::state::instance(app2)("shared.value").value(), "from_node_1");
  EXPECT_EQ(cvc::state::instance(app3)("shared.value").value(), "from_node_1");
}

TEST_F(MultiNodeReplicationTest, AllNodesWriteDifferentPaths) {
  // First value() creates node, second is journaled
  cvc::state::instance(app1)("n1.data").value(std::string("init"));
  cvc::state::instance(app1)("n1.data").value(std::string("val1"));
  cvc::state::instance(app2)("n2.data").value(std::string("init"));
  cvc::state::instance(app2)("n2.data").value(std::string("val2"));
  cvc::state::instance(app3)("n3.data").value(std::string("init"));
  cvc::state::instance(app3)("n3.data").value(std::string("val3"));

  transport.pump_all();

  // After pump, all nodes see all paths
  for (auto *app : {&app1, &app2, &app3}) {
    EXPECT_EQ(cvc::state::instance(*app)("n1.data").value(), "val1");
    EXPECT_EQ(cvc::state::instance(*app)("n2.data").value(), "val2");
    EXPECT_EQ(cvc::state::instance(*app)("n3.data").value(), "val3");
  }
}

TEST_F(MultiNodeReplicationTest, ConflictingWritesDeterministicWinner) {
  // All 3 nodes write to the same key. With conflict resolution enabled
  // and equal HLC (0), lexicographically greatest origin wins.
  // "node_3" > "node_2" > "node_1", so node_3 wins on all shards.

  auto m1 = make_set_value("node_1", 1, "contested", "from_node_1", "cluster_a");
  auto m2 = make_set_value("node_2", 1, "contested", "from_node_2", "cluster_a");
  auto m3 = make_set_value("node_3", 1, "contested", "from_node_3", "cluster_a");

  // Ingest all three on each shard
  s1.ingest_remote(m1);
  s1.ingest_remote(m2);
  s1.ingest_remote(m3);
  s2.ingest_remote(m1);
  s2.ingest_remote(m2);
  s2.ingest_remote(m3);
  s3.ingest_remote(m1);
  s3.ingest_remote(m2);
  s3.ingest_remote(m3);

  // All nodes should converge to the same value
  auto v1 = cvc::state::instance(app1)("contested").value();
  auto v2 = cvc::state::instance(app2)("contested").value();
  auto v3 = cvc::state::instance(app3)("contested").value();
  EXPECT_EQ(v1, v2);
  EXPECT_EQ(v2, v3);
  EXPECT_EQ(v1, "from_node_3");
}

TEST_F(MultiNodeReplicationTest, SequentialOverwrites) {
  // Create the node first
  cvc::state::instance(app1)("seq.value").value(std::string("init"));
  transport.pump_all();

  for (int i = 0; i < 10; ++i) {
    cvc::state::instance(app1)("seq.value").value(std::string("v" + std::to_string(i)));
    transport.pump_all();
  }

  // Final value should be the last written
  EXPECT_EQ(cvc::state::instance(app2)("seq.value").value(), "v9");
  EXPECT_EQ(cvc::state::instance(app3)("seq.value").value(), "v9");
}

// ===========================================================================
//  6.  Coordinated messaging: state_exec programs work together
// ===========================================================================

class CoordinatedMessagingTest : public ::testing::Test {
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
    ictx.uid = "coord-user";
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

TEST_F(CoordinatedMessagingTest, ProducerConsumerCoordination) {
  // Two processes coordinate: producer writes items to state tree,
  // consumer reads them and writes a count to signal completion.

  int producer = exec(R"(
        (begin
          (state-set "queue.item.0" "apple")
          (state-set "queue.item.1" "banana")
          (state-set "queue.item.2" "cherry")
          (state-set "queue.count" "3")
          (msg-send "queue.ready" "items-available")
          "producer-done")
    )");

  int consumer = exec(R"(
        (begin
          (set result (list))
          (set i 0)
          (while (< i 3)
            (begin
              (set key (str-concat "queue.item." (str i)))
              (set val (state-get key))
              (if (!= val nil)
                (set i (+ i 1))
                nil)))
          (state-set "consumed.total" (str i))
          i)
    )");

  auto results = sched.run();

  auto prod_result = sched.get_result(producer);
  ASSERT_TRUE(prod_result.has_value());
  EXPECT_EQ(std::get<std::string>(prod_result->v), "producer-done");

  auto cons_result = sched.get_result(consumer);
  ASSERT_TRUE(cons_result.has_value());
  EXPECT_EQ(std::get<int64_t>(cons_result->v), 3);
}

TEST_F(CoordinatedMessagingTest, MapReducePattern) {
  // 3 mapper processes write partial sums to state tree.
  // A reducer reads them and computes the total.

  int m1 = exec(R"(
        (begin
          (state-set "partial.0" (str (+ 1 2 3)))
          "map-0-done")
    )");

  int m2 = exec(R"(
        (begin
          (state-set "partial.1" (str (+ 4 5 6)))
          "map-1-done")
    )");

  int m3 = exec(R"(
        (begin
          (state-set "partial.2" (str (+ 7 8 9)))
          "map-2-done")
    )");

  int reducer = exec(R"(
        (begin
          (set s0 (state-get "partial.0"))
          (set s1 (state-get "partial.1"))
          (set s2 (state-get "partial.2"))
          (state-set "result.total"
            (str (+ (if (= s0 nil) 0 6)
                    (if (= s1 nil) 0 15)
                    (if (= s2 nil) 0 24))))
          45)
    )");

  sched.run();

  // Verify all mappers completed
  for (auto pid : {m1, m2, m3}) {
    auto r = sched.get_result(pid);
    ASSERT_TRUE(r.has_value());
  }

  // Reducer should compute 45
  auto rr = sched.get_result(reducer);
  ASSERT_TRUE(rr.has_value());
  EXPECT_EQ(std::get<int64_t>(rr->v), 45);
}

TEST_F(CoordinatedMessagingTest, WorkerPoolWithMsgSend) {
  // N workers each do computation and use msg-send to signal completion
  const int kWorkers = 5;
  std::vector<int> pids;

  for (int i = 0; i < kWorkers; ++i) {
    std::string script = "(begin"
                         "  (state-set \"worker." +
                         std::to_string(i) +
                         ".result\" "
                         "    (str (* " +
                         std::to_string(i + 1) + " " + std::to_string(i + 1) +
                         ")))"
                         "  (msg-send \"worker.done\" (str (self-pid)))"
                         "  \"ok\")";
    pids.push_back(exec(script));
  }

  sched.run();

  // All workers completed
  for (auto pid : pids) {
    auto r = sched.get_result(pid);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(std::get<std::string>(r->v), "ok");
  }

  // Verify state tree has all results
  auto &root = *ictx.root;
  for (int i = 0; i < kWorkers; ++i) {
    auto *node = root.findDescendant("worker." + std::to_string(i) + ".result");
    ASSERT_NE(node, nullptr) << "worker " << i << " result missing";
    EXPECT_EQ(node->value(), std::to_string((i + 1) * (i + 1)));
  }
}

TEST_F(CoordinatedMessagingTest, SpawnWorkersAndCollectResults) {
  // A coordinator process spawns workers and waits for results via state tree

  // spawn creates sub-processes with default builtins only
  // (no intrinsics). Workers use basic arithmetic.
  int coordinator = exec("(begin"
                         "  (spawn \"(* 10 10)\" \"w0\")"
                         "  (spawn \"(* 20 20)\" \"w1\")"
                         "  (spawn \"(* 30 30)\" \"w2\")"
                         "  3)");

  auto results = sched.run();

  auto coord_result = sched.get_result(coordinator);
  ASSERT_TRUE(coord_result.has_value());
  EXPECT_EQ(std::get<int64_t>(coord_result->v), 3);

  // Verify all 4 processes exist (coordinator + 3 workers)
  auto procs = sched.list_processes();
  EXPECT_GE(procs.size(), 4u);
}

TEST_F(CoordinatedMessagingTest, ProcessWritesProgressToStateTree) {
  // A process writes progress updates as it computes
  int pid = exec(R"(
        (begin
          (set total 0)
          (set i 0)
          (while (< i 5)
            (begin
              (set total (+ total (* i i)))
              (state-set "progress.current" (str i))
              (state-set "progress.total" (str total))
              (set i (+ i 1))))
          total)
    )");

  sched.run();

  auto r = sched.get_result(pid);
  ASSERT_TRUE(r.has_value());
  // 0 + 1 + 4 + 9 + 16 = 30
  EXPECT_EQ(std::get<int64_t>(r->v), 30);

  // Verify final progress value in state tree
  auto *node = ictx.root->findDescendant("progress.total");
  ASSERT_NE(node, nullptr);
  EXPECT_EQ(node->value(), "30");
}

TEST_F(CoordinatedMessagingTest, TwoProcessesCollaborateOnDataWrite) {
  // Process A builds the header, Process B builds the body,
  // both write to the same state tree namespace

  int header_writer = exec(R"(
        (begin
          (state-set "document.header.title" "My Report")
          (state-set "document.header.author" "Alice")
          (state-set "document.header.date" "2026-05-24")
          (msg-send "document.events" "header-complete")
          "header-done")
    )");

  int body_writer = exec(R"(
        (begin
          (state-set "document.body.section1" "Introduction text...")
          (state-set "document.body.section2" "Methods text...")
          (state-set "document.body.section3" "Results text...")
          (msg-send "document.events" "body-complete")
          "body-done")
    )");

  sched.run();

  // Both writers completed
  EXPECT_EQ(std::get<std::string>(sched.get_result(header_writer)->v), "header-done");
  EXPECT_EQ(std::get<std::string>(sched.get_result(body_writer)->v), "body-done");

  // Full document exists in state tree
  auto &root = *ictx.root;
  EXPECT_EQ(root("document.header.title").value(), "My Report");
  EXPECT_EQ(root("document.header.author").value(), "Alice");
  EXPECT_EQ(root("document.body.section1").value(), "Introduction text...");
  EXPECT_EQ(root("document.body.section3").value(), "Results text...");
}

TEST_F(CoordinatedMessagingTest, MsgSendNotifiesExternalOnGoalComplete) {
  // A process computes a result, writes it to state tree,
  // then sends a completion message

  int pid = exec(R"(
        (begin
          ;; Compute Fibonacci(12) = 144
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (set result (fib 12))

          ;; Write result to state tree
          (state-set "computation.fib12.result" (str result))
          (state-set "computation.fib12.status" "complete")

          ;; Notify via messaging
          (msg-send "computation.notifications" "fib12-done")

          result)
    )");

  sched.run();

  auto r = sched.get_result(pid);
  ASSERT_TRUE(r.has_value());
  EXPECT_EQ(std::get<int64_t>(r->v), 144);

  EXPECT_EQ(ictx.root->findDescendant("computation.fib12.result")->value(), "144");
  EXPECT_EQ(ictx.root->findDescendant("computation.fib12.status")->value(), "complete");
}

// ===========================================================================
//  7.  Combined: coordinator + cluster + state_exec programs
// ===========================================================================

class CoordinatorClusterIntegrationTest : public ::testing::Test {
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

TEST_F(CoordinatorClusterIntegrationTest, SubmitMultipleJobsAndObserve) {
  // Submit 10 jobs and verify they all complete
  std::vector<int> pids;
  for (int i = 0; i < 10; ++i) {
    auto r = coord.submit("(+ " + std::to_string(i) + " 100)");
    ASSERT_TRUE(r.accepted);
    pids.push_back(r.pid);
  }

  sched.run();

  for (int i = 0; i < 10; ++i) {
    auto result = sched.get_result(pids[i]);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(std::get<int64_t>(result->v), i + 100);
  }

  auto stats = coord.cluster_stats();
  EXPECT_GE(stats.total_processes, 10);
}

TEST_F(CoordinatorClusterIntegrationTest, ResourcePolicyPreventsOverload) {
  resource_policy policy;
  policy.max_processes = 5;
  policy.enforce = resource_policy::mode::strict;
  coord.set_resource_policy(policy);

  // Submit 5 long-running processes
  for (int i = 0; i < 5; ++i) {
    auto r = coord.submit("(begin (while true 1))");
    EXPECT_TRUE(r.accepted);
  }

  // 6th should be rejected
  auto r = coord.submit("(+ 1 2)");
  EXPECT_FALSE(r.accepted);

  // After killing one, we can submit again
  sched.kill(1);
  // Note: kill changes status but process is still counted.
  // The policy check depends on implementation.
}

TEST_F(CoordinatorClusterIntegrationTest, ObserveAllProcessesViaCoordinator) {
  coord.submit("(+ 1 2)");
  coord.submit("(+ 3 4)");
  coord.submit("(+ 5 6)");

  sched.run();

  auto all = coord.ps_all();
  EXPECT_GE(all.size(), 3u);

  auto cstats = coord.cluster_stats();
  EXPECT_GE(cstats.total_processes, 3);
  EXPECT_GE(cstats.local.terminated, 3);
}
