#include <chrono>
#include <cstdlib>
#include <cvc/core/state_subscription_router.h>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace cvc;

namespace {

std::set<state_subscription_id>
subscription_ids(const std::vector<state_subscription> &subscriptions) {
  std::set<state_subscription_id> ids;
  for (const state_subscription &subscription : subscriptions) {
    ids.insert(subscription.id);
  }
  return ids;
}

bool opt_in_enabled(const char *name) {
  const char *value = std::getenv(name);
  return value != nullptr && std::string(value) == "1";
}

} // namespace

TEST(StateSubscriptionRouterTest, RootSubscriptionMatchesEveryPath) {
  state_subscription_router router;
  state_subscription_id root_id = router.subscribe("");

  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera")),
            std::set<state_subscription_id>({root_id}));
  EXPECT_EQ(subscription_ids(router.subscriptions_for("simulation.volume.brick")),
            std::set<state_subscription_id>({root_id}));
}

TEST(StateSubscriptionRouterTest, PrefixSubscriptionMatchesDescendantsOnPathBoundary) {
  state_subscription_router router;
  state_subscription_id scene_id = router.subscribe("scene.camera");
  state_subscription_id volume_id = router.subscribe("simulation.volume");

  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera")),
            std::set<state_subscription_id>({scene_id}));
  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera.position.x")),
            std::set<state_subscription_id>({scene_id}));
  EXPECT_TRUE(router.subscriptions_for("scene.camera2.position.x").empty());
  EXPECT_EQ(subscription_ids(router.subscriptions_for("simulation.volume.brick.0")),
            std::set<state_subscription_id>({volume_id}));
}

TEST(StateSubscriptionRouterTest, ExactOnlySubscriptionDoesNotMatchDescendants) {
  state_subscription_router router;
  state_subscription_id exact_id = router.subscribe("scene.camera", false);

  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera")),
            std::set<state_subscription_id>({exact_id}));
  EXPECT_TRUE(router.subscriptions_for("scene.camera.position").empty());
}

TEST(StateSubscriptionRouterTest, MultipleSubscriptionsCanMatchOnePath) {
  state_subscription_router router;
  state_subscription_id root_id = router.subscribe("");
  state_subscription_id scene_id = router.subscribe("scene");
  state_subscription_id camera_id = router.subscribe("scene.camera");

  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera.position.x")),
            std::set<state_subscription_id>({root_id, scene_id, camera_id}));
}

TEST(StateSubscriptionRouterTest, UnsubscribeRemovesOnlyRequestedSubscription) {
  state_subscription_router router;
  state_subscription_id scene_id = router.subscribe("scene");
  state_subscription_id camera_id = router.subscribe("scene.camera");

  EXPECT_TRUE(router.unsubscribe(scene_id));
  EXPECT_FALSE(router.unsubscribe(scene_id));

  EXPECT_EQ(router.size(), 1u);
  EXPECT_EQ(subscription_ids(router.subscriptions_for("scene.camera.position.x")),
            std::set<state_subscription_id>({camera_id}));
}

TEST(StateSubscriptionRouterTest, NormalizesLeadingAndTrailingSeparators) {
  state_subscription_router router;
  state_subscription_id camera_id = router.subscribe(".scene.camera.");

  std::vector<state_subscription> matches = router.subscriptions_for(".scene.camera.position.");

  ASSERT_EQ(matches.size(), 1u);
  EXPECT_EQ(matches[0].id, camera_id);
  EXPECT_EQ(matches[0].path_prefix, "scene.camera");
}

TEST(StateSubscriptionRouterTest, ClearRemovesSubscriptionsButKeepsIdMonotonic) {
  state_subscription_router router;
  router.subscribe("scene");
  router.subscribe("simulation");

  router.clear();
  state_subscription_id next_id = router.subscribe("analysis");

  EXPECT_EQ(router.size(), 1u);
  EXPECT_EQ(next_id, 3u);
}

TEST(StateSubscriptionRouterTest, ConcurrentSubscribeAndLookupIsSafe) {
  state_subscription_router router;
  const int thread_count = 6;
  const int subscriptions_per_thread = 200;
  std::vector<std::thread> threads;

  for (int thread_index = 0; thread_index < thread_count; ++thread_index) {
    threads.emplace_back([&router, thread_index]() {
      for (int subscription_index = 0; subscription_index < subscriptions_per_thread;
           ++subscription_index) {
        router.subscribe("scene." + std::to_string(thread_index) + "." +
                         std::to_string(subscription_index));
        router.subscriptions_for("scene." + std::to_string(thread_index) + "." +
                                 std::to_string(subscription_index) + ".value");
      }
    });
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  EXPECT_EQ(router.size(), static_cast<std::size_t>(thread_count * subscriptions_per_thread));
}

TEST(StateSubscriptionRouterStressTest, OptionalLargePrefixSetStress) {
  if (!opt_in_enabled("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run router stress tests";
  }

  state_subscription_router router;
  const int subscription_count = 20000;
  for (int subscription_index = 0; subscription_index < subscription_count; ++subscription_index) {
    router.subscribe("tree.branch." + std::to_string(subscription_index));
  }

  EXPECT_EQ(router.size(), static_cast<std::size_t>(subscription_count));
  EXPECT_EQ(router.subscriptions_for("tree.branch.19999.leaf").size(), 1u);
  EXPECT_TRUE(router.subscriptions_for("tree.branch.20000.leaf").empty());
}

TEST(StateSubscriptionRouterPerformanceTest, OptionalLookupThroughputSmoke) {
  if (!opt_in_enabled("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run router performance smoke tests";
  }

  state_subscription_router router;
  const int subscription_count = 5000;
  const int lookup_count = 20000;
  for (int subscription_index = 0; subscription_index < subscription_count; ++subscription_index) {
    router.subscribe("tree.branch." + std::to_string(subscription_index));
  }

  auto start_time = std::chrono::steady_clock::now();
  std::size_t total_matches = 0;
  for (int lookup_index = 0; lookup_index < lookup_count; ++lookup_index) {
    total_matches +=
        router
            .subscriptions_for("tree.branch." + std::to_string(lookup_index % subscription_count) +
                               ".leaf")
            .size();
  }
  auto end_time = std::chrono::steady_clock::now();

  double elapsed_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
  double lookups_per_second = static_cast<double>(lookup_count) / elapsed_seconds;
  RecordProperty("lookups_per_second", lookups_per_second);

  EXPECT_EQ(total_matches, static_cast<std::size_t>(lookup_count));
}
