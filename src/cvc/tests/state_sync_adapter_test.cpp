/*
  Copyright 2025 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License version
  2.1 as published by the Free Software Foundation.
*/

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_sync_adapter.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

namespace {

bool env_flag(const char *name) {
  const char *v = std::getenv(name);
  return v != nullptr && std::string(v) == "1";
}

// Helper to get a fresh-ish state subtree. cvc::state is a per-app
// singleton; tests use unique root paths to avoid cross-test pollution.
std::string unique_root(const std::string &tag) {
  static std::atomic<int> counter{0};
  int n = counter.fetch_add(1);
  return std::string("test_sync_") + tag + "_" + std::to_string(n);
}

} // namespace

TEST(StateSyncAdapterTest, AttachIsIdempotent) {
  cvc::app a;
  std::string root = unique_root("idem");
  cvc::state::instance(a)(root + ".child").value("seed");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  EXPECT_FALSE(adapter.is_attached());
  adapter.attach();
  EXPECT_TRUE(adapter.is_attached());
  std::size_t paths1 = adapter.observed_paths();
  adapter.attach();
  EXPECT_EQ(paths1, adapter.observed_paths());
}

TEST(StateSyncAdapterTest, LocalChangeIsJournaled) {
  cvc::app a;
  std::string root = unique_root("local");
  cvc::state::instance(a)(root + ".x").value("0");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  cvc::state::instance(a)(root + ".x").value("42");

  EXPECT_GE(adapter.local_mutation_count(), 1u);
  EXPECT_GE(adapter.journal().size(), 1u);
  auto snap = adapter.journal().snapshot();
  bool found = false;
  for (const auto &m : snap) {
    if (m.path == root + ".x" && m.string_value == "42") {
      found = true;
      EXPECT_EQ(m.op, cvc::state_mutation_op::set_value);
      EXPECT_FALSE(m.mutation_id.empty());
      EXPECT_GT(m.sequence, 0u);
    }
  }
  EXPECT_TRUE(found);
}

TEST(StateSyncAdapterTest, NewChildIsObservedLazily) {
  cvc::app a;
  std::string root = unique_root("lazy");
  cvc::state::instance(a)(root).touch();

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();
  std::size_t before = adapter.observed_paths();

  // Create a brand-new child after attach.
  cvc::state::instance(a)(root + ".freshChild").value("hello");

  EXPECT_GT(adapter.observed_paths(), before);
  // Note: the FIRST value() on a fresh child fires valueChanged
  // *before* childChanged reaches the adapter, so that initial set
  // is lost. A subsequent set on the now-observed child must land
  // in the journal.
  cvc::state::instance(a)(root + ".freshChild").value("hello2");
  EXPECT_GE(adapter.local_mutation_count(), 1u);
}

TEST(StateSyncAdapterTest, RemoteApplyDoesNotFeedback) {
  cvc::app a;
  std::string root = unique_root("remote");
  cvc::state::instance(a)(root + ".y").value("0");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  cvc::state_mutation m;
  m.path = root + ".y";
  m.op = cvc::state_mutation_op::set_value;
  m.string_value = "remote-value";
  EXPECT_TRUE(adapter.apply_remote(m));

  // Confirm the local tree reflects the remote change.
  EXPECT_EQ(cvc::state::instance(a)(root + ".y").value(), "remote-value");
  // And confirm the remote change did NOT loop back into the
  // journal as a local mutation.
  EXPECT_EQ(adapter.local_mutation_count(), 0u);
  EXPECT_EQ(adapter.remote_mutation_count(), 1u);
}

TEST(StateSyncAdapterTest, SuppressionScopeSilencesLocalChanges) {
  cvc::app a;
  std::string root = unique_root("suppress");
  cvc::state::instance(a)(root + ".z").value("0");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  {
    cvc::state_sync_adapter::suppression_scope guard(adapter);
    cvc::state::instance(a)(root + ".z").value("silent");
  }
  EXPECT_EQ(adapter.local_mutation_count(), 0u);

  cvc::state::instance(a)(root + ".z").value("loud");
  EXPECT_GE(adapter.local_mutation_count(), 1u);
}

TEST(StateSyncAdapterTest, OnLocalMutationCallbackFires) {
  cvc::app a;
  std::string root = unique_root("cb");
  cvc::state::instance(a)(root + ".q").value("0");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  std::atomic<int> fired{0};
  std::string last_path;
  adapter.set_on_local_mutation([&](const cvc::state_mutation &m) {
    fired.fetch_add(1);
    last_path = m.path;
  });
  adapter.attach();

  cvc::state::instance(a)(root + ".q").value("99");
  EXPECT_GE(fired.load(), 1);
  EXPECT_EQ(last_path, root + ".q");
}

TEST(StateSyncAdapterTest, OnRemoteAppliedCallbackFires) {
  cvc::app a;
  std::string root = unique_root("rcb");
  cvc::state::instance(a)(root).touch();

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  std::atomic<int> fired{0};
  adapter.set_on_remote_applied([&](const cvc::state_mutation &) { fired.fetch_add(1); });
  adapter.attach();

  cvc::state_mutation m;
  m.path = root + ".w";
  m.op = cvc::state_mutation_op::set_value;
  m.string_value = "ok";
  EXPECT_TRUE(adapter.apply_remote(m));
  EXPECT_EQ(fired.load(), 1);
}

TEST(StateSyncAdapterTest, DetachStopsJournaling) {
  cvc::app a;
  std::string root = unique_root("detach");
  cvc::state::instance(a)(root + ".v").value("0");

  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();
  cvc::state::instance(a)(root + ".v").value("1");
  std::uint64_t before = adapter.local_mutation_count();
  EXPECT_GE(before, 1u);

  adapter.detach();
  EXPECT_FALSE(adapter.is_attached());
  cvc::state::instance(a)(root + ".v").value("2");
  EXPECT_EQ(adapter.local_mutation_count(), before);
}

TEST(StateSyncAdapterStressTest, OptionalHighChurnStress) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run adapter "
                    "stress tests";
  }
  cvc::app a;
  std::string root = unique_root("stress");
  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  const int kThreads = 4;
  const int kPerThread = 5000;
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&, t]() {
      for (int i = 0; i < kPerThread; ++i) {
        std::string path = root + ".w" + std::to_string(t) + "." + std::to_string(i % 32);
        cvc::state::instance(a)(path).value(std::to_string(i));
      }
    });
  }
  for (auto &th : threads)
    th.join();

  EXPECT_GT(adapter.local_mutation_count(), 0u);
}

TEST(StateSyncAdapterPerformanceTest, OptionalLocalChangeThroughputSmoke) {
  if (!env_flag("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run adapter "
                    "performance smoke tests";
  }
  cvc::app a;
  std::string root = unique_root("perf");
  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  const int kIters = 20000;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < kIters; ++i) {
    cvc::state::instance(a)(root + ".k" + std::to_string(i % 64)).value(std::to_string(i));
  }
  auto elapsed = std::chrono::steady_clock::now() - start;
  double secs = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
  double rate = kIters / secs;
  std::cerr << "[adapter perf] " << kIters << " local sets in " << secs << "s (" << rate << "/s)\n";
  EXPECT_GE(adapter.local_mutation_count(), static_cast<std::uint64_t>(kIters - 64));
  EXPECT_LT(secs, 10.0);
}
