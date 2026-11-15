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
#include <cvc/state/state.h>
#include <cvc/state/state_sync_adapter.h>
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
  // The FIRST value() on a fresh child fires valueChanged before childChanged reaches the
  // adapter, so the adapter journals the child's content when it first sees it: the initial set
  // replicates too. A subsequent set on the now-observed child lands in the journal as usual.
  const auto values_at = [&](const std::string &path) {
    std::vector<std::string> out;
    for (const auto &m : adapter.journal().snapshot())
      if (m.path == path && m.op == cvc::state_mutation_op::set_value)
        out.push_back(m.string_value);
    return out;
  };
  EXPECT_EQ(values_at(root + ".freshChild"), (std::vector<std::string>{"hello"}));
  cvc::state::instance(a)(root + ".freshChild").value("hello2");
  EXPECT_EQ(values_at(root + ".freshChild"), (std::vector<std::string>{"hello", "hello2"}));
  EXPECT_EQ(adapter.local_mutation_count(), 2u);
}

// What is first seen without content is not journaled: a node made on the way to another, or a
// touch (set_replicated's touch-then-write, whose write is then journaled once).
TEST(StateSyncAdapterTest, FirstSightJournalsOnlyContent) {
  cvc::app a;
  std::string root = unique_root("sight");
  cvc::state::instance(a)(root).touch();
  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  cvc::state::instance(a)(root + ".a.b.c").value("deep");
  cvc::state &t = cvc::state::instance(a)(root + ".touched");
  t.touch();
  t.value("after touch");
  std::vector<std::string> paths;
  for (const auto &m : adapter.journal().snapshot())
    if (m.op == cvc::state_mutation_op::set_value)
      paths.push_back(m.path + "=" + m.string_value);
  EXPECT_EQ(paths, (std::vector<std::string>{root + ".a.b.c=deep", root + ".touched=after touch"}));
}

// A remote apply that makes a node is not journaled at first sight either (no echo).
TEST(StateSyncAdapterTest, RemoteApplyOfAFreshNodeDoesNotFeedback) {
  cvc::app a;
  std::string root = unique_root("remotefresh");
  cvc::state::instance(a)(root).touch();
  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  cvc::state_mutation m;
  m.path = root + ".fresh.leaf";
  m.op = cvc::state_mutation_op::set_value;
  m.string_value = "from a peer";
  EXPECT_TRUE(adapter.apply_remote(m));
  EXPECT_EQ(cvc::state::instance(a)(root + ".fresh.leaf").value(), "from a peer");
  EXPECT_EQ(adapter.local_mutation_count(), 0u);
  EXPECT_EQ(adapter.journal().size(), 0u);
}

// A remote delete (a relay's read-scope retraction) takes the subtree out, never making a missing
// one, and journals nothing; the observers go with it, so a node made at that path later is
// observed afresh and its writes replicate.
TEST(StateSyncAdapterTest, ARemoteDeleteTakesTheSubtreeOut) {
  cvc::app a;
  std::string root = unique_root("remotedelete");
  cvc::state::instance(a)(root).touch();
  cvc::state_sync_adapter adapter(a, root, "nodeA");
  adapter.attach();

  cvc::state_mutation set;
  set.path = root + ".gone.leaf"; // "gone" is made on the way, with no observer of its own
  set.op = cvc::state_mutation_op::set_value;
  set.string_value = "v";
  ASSERT_TRUE(adapter.apply_remote(set));

  cvc::state_mutation del;
  del.path = root + ".gone";
  del.op = cvc::state_mutation_op::delete_subtree;
  EXPECT_TRUE(adapter.apply_remote(del));
  cvc::state &top = cvc::state::instance(a);
  EXPECT_EQ(top.findDescendant(root + ".gone"), nullptr) << "not made again by its parent's slot";
  EXPECT_EQ(top.findDescendant(root + ".gone.leaf"), nullptr);
  del.path = root + ".never";
  EXPECT_TRUE(adapter.apply_remote(del));
  EXPECT_EQ(top.findDescendant(root + ".never"), nullptr) << "a missing node is not made";
  EXPECT_EQ(adapter.journal().size(), 0u);

  cvc::state::instance(a)(root + ".gone.leaf").value("local");
  bool journaled = false;
  for (const auto &m : adapter.journal().snapshot())
    journaled = journaled || (m.path == root + ".gone.leaf" && m.string_value == "local");
  EXPECT_TRUE(journaled) << "a node made there later is observed again";
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

// The suppressed threads are each app's own: a scope on one app's adapter silences that app's
// adapters (all of them), never another app's, whichever app is made first.
TEST(StateSyncAdapterTest, SuppressionIsPerApp) {
  for (int order = 0; order < 2; ++order) {
    cvc::app a, b;
    cvc::app &quiet = order == 0 ? a : b;
    cvc::app &loud = order == 0 ? b : a;
    cvc::state::instance(quiet)("s.v").value("0");
    cvc::state::instance(loud)("s.v").value("0");
    cvc::state_sync_adapter quietAdapter(quiet, "s", "q"), quietOther(quiet, "s", "q2");
    cvc::state_sync_adapter loudAdapter(loud, "s", "l");
    quietAdapter.attach();
    quietOther.attach();
    loudAdapter.attach();
    {
      cvc::state_sync_adapter::suppression_scope guard(quietAdapter);
      cvc::state::instance(quiet)("s.v").value("silent");
      cvc::state::instance(loud)("s.v").value("heard");
    }
    EXPECT_EQ(quietAdapter.local_mutation_count(), 0u);
    EXPECT_EQ(quietOther.local_mutation_count(), 0u) << "the app's other adapter journaled";
    EXPECT_EQ(loudAdapter.local_mutation_count(), 1u) << "another app's scope silenced this app";
  }
}

// A remote apply on one app does not hide a local change to another app made on the same thread
// (here by a slot the apply sets off): that change is the other app's own.
TEST(StateSyncAdapterTest, RemoteApplyOnOneAppDoesNotHideAnothersChange) {
  cvc::app a, b;
  cvc::state::instance(a)("s.v").value("0");
  cvc::state::instance(b)("s.v").value("0");
  cvc::state_sync_adapter adapterA(a, "s", "A"), adapterB(b, "s", "B");
  adapterA.attach();
  adapterB.attach();
  boost::signals2::scoped_connection mirror = cvc::state::instance(b)("s.v").valueChanged.connect(
      [&a, &b]() { cvc::state::instance(a)("s.v").value(cvc::state::instance(b)("s.v").value()); });

  cvc::state_mutation m;
  m.path = "s.v";
  m.op = cvc::state_mutation_op::set_value;
  m.string_value = "from a peer";
  EXPECT_TRUE(adapterB.apply_remote(m));
  EXPECT_EQ(adapterB.local_mutation_count(), 0u) << "the apply echoed";
  EXPECT_EQ(cvc::state::instance(a)("s.v").value(), "from a peer");
  EXPECT_EQ(adapterA.local_mutation_count(), 1u) << "app a's own change was not journaled";
}

TEST(StateSyncAdapterTest, SuppressionAppsOnThreadsAtOnce) {
  constexpr int kApps = 4;
  std::vector<std::uint64_t> counts(kApps, 99);
  std::vector<std::thread> threads;
  for (int i = 0; i < kApps; ++i)
    threads.emplace_back([i, &counts] {
      cvc::app app;
      cvc::state::instance(app)("s.v").value("0");
      cvc::state_sync_adapter adapter(app, "s", "n" + std::to_string(i));
      adapter.attach();
      for (int k = 0; k < 50; ++k) {
        if (k % 2 == 0) {
          cvc::state_sync_adapter::suppression_scope guard(adapter);
          cvc::state::instance(app)("s.v").value(std::to_string(k));
        } else {
          cvc::state::instance(app)("s.v").value(std::to_string(k));
        }
      }
      counts[i] = adapter.local_mutation_count();
    });
  for (std::thread &t : threads)
    t.join();
  for (int i = 0; i < kApps; ++i)
    EXPECT_EQ(counts[i], 25u) << "app " << i;
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
