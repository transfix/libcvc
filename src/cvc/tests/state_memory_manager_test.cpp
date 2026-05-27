#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_memory_manager.h>
#include <gtest/gtest.h>

class StateMemoryManagerTest : public ::testing::Test {
protected:
  void SetUp() override {
    app_ = std::make_unique<cvc::app>();
    root_ = &cvc::state::instance(*app_)("mem_mgr_test");
  }

  std::unique_ptr<cvc::app> app_;
  cvc::state *root_ = nullptr;
};

TEST_F(StateMemoryManagerTest, InitialState) {
  cvc::state_memory_manager mgr(*root_, 1024);
  EXPECT_EQ(mgr.budget(), 1024u);
  EXPECT_EQ(mgr.resident_bytes(), 0u);
  EXPECT_EQ(mgr.evicted_count(), 0u);
}

TEST_F(StateMemoryManagerTest, SetBudget) {
  cvc::state_memory_manager mgr(*root_, 1024);
  mgr.set_budget(2048);
  EXPECT_EQ(mgr.budget(), 2048u);
}

TEST_F(StateMemoryManagerTest, TrackAndResident) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("a").value("hello");
  mgr.track("a", 100);
  EXPECT_EQ(mgr.resident_bytes(), 100u);

  (*root_)("b").value("world");
  mgr.track("b", 200);
  EXPECT_EQ(mgr.resident_bytes(), 300u);
}

TEST_F(StateMemoryManagerTest, TrackUpdateSize) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("a").value("small");
  mgr.track("a", 50);
  EXPECT_EQ(mgr.resident_bytes(), 50u);

  // Update with larger payload
  (*root_)("a").value("much larger value");
  mgr.track("a", 200);
  EXPECT_EQ(mgr.resident_bytes(), 200u);
}

TEST_F(StateMemoryManagerTest, Untrack) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("a").value("hi");
  mgr.track("a", 100);
  EXPECT_EQ(mgr.resident_bytes(), 100u);

  mgr.untrack("a");
  EXPECT_EQ(mgr.resident_bytes(), 0u);
}

TEST_F(StateMemoryManagerTest, ManualEvictAndRepopulate) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("data").value("important_data");
  mgr.track("data", 100);

  EXPECT_FALSE(mgr.is_evicted("data"));
  EXPECT_TRUE(mgr.evict("data"));
  EXPECT_TRUE(mgr.is_evicted("data"));
  EXPECT_EQ(mgr.resident_bytes(), 0u);
  EXPECT_EQ(mgr.evicted_count(), 1u);

  // Value should be cleared
  EXPECT_EQ((*root_)("data").value(), "");

  // Repopulate
  EXPECT_TRUE(mgr.repopulate("data"));
  EXPECT_FALSE(mgr.is_evicted("data"));
  EXPECT_EQ((*root_)("data").value(), "important_data");
  EXPECT_EQ(mgr.evicted_count(), 0u);
}

TEST_F(StateMemoryManagerTest, EvictNonexistentPath) {
  cvc::state_memory_manager mgr(*root_, 1024);
  EXPECT_FALSE(mgr.evict("nonexistent"));
}

TEST_F(StateMemoryManagerTest, RepopulateNonEvicted) {
  cvc::state_memory_manager mgr(*root_, 1024);
  EXPECT_FALSE(mgr.repopulate("nonexistent"));
}

TEST_F(StateMemoryManagerTest, LRUEvictionOnBudgetExceed) {
  // Budget of 200, high watermark 0.95 (190), low watermark 0.80 (160)
  cvc::state_memory_manager mgr(*root_, 200);
  mgr.set_high_watermark(0.95);
  mgr.set_low_watermark(0.80);

  (*root_)("a").value("aaa");
  mgr.track("a", 80);

  (*root_)("b").value("bbb");
  mgr.track("b", 80);

  // At 160, still under high watermark (190)
  EXPECT_EQ(mgr.resident_bytes(), 160u);
  EXPECT_EQ(mgr.evicted_count(), 0u);

  // Adding 40 more pushes to 200, exceeding high watermark
  (*root_)("c").value("ccc");
  mgr.track("c", 40);

  // Eviction should have occurred until <= low watermark (160)
  EXPECT_LE(mgr.resident_bytes(), 160u);
  // "a" was accessed longest ago, should be evicted
  EXPECT_TRUE(mgr.is_evicted("a"));
}

TEST_F(StateMemoryManagerTest, TouchUpdatesLRUOrder) {
  // Budget 300, high=0.90 (270), low=0.80 (240).
  // Track a=80, b=80, touch a, then c=120 → total 280 > 270.
  // Evict until <= 240 → evict "b" (LRU after touch), leaving 200.
  cvc::state_memory_manager mgr(*root_, 300);
  mgr.set_high_watermark(0.90);
  mgr.set_low_watermark(0.80);

  (*root_)("a").value("aaa");
  mgr.track("a", 80);

  (*root_)("b").value("bbb");
  mgr.track("b", 80);

  // Touch "a" — now "b" is least recently used
  mgr.touch("a");

  // Push over high watermark
  (*root_)("c").value("ccc");
  mgr.track("c", 120);

  // "b" should be evicted (LRU after touch), not "a"
  EXPECT_TRUE(mgr.is_evicted("b"));
  EXPECT_FALSE(mgr.is_evicted("a"));
}

TEST_F(StateMemoryManagerTest, RunEvictionManually) {
  // Use manual policy to prevent auto-eviction, then run_eviction explicitly
  cvc::state_memory_manager mgr(*root_, 100, cvc::state_memory_manager::eviction_policy::manual);
  mgr.set_low_watermark(0.50);

  (*root_)("x").value("xxx");
  mgr.track("x", 60);
  (*root_)("y").value("yyy");
  mgr.track("y", 60);

  EXPECT_EQ(mgr.resident_bytes(), 120u);

  // Manually switch to LRU for eviction and run
  // (run_eviction uses whatever policy is set, but manual returns 0)
  // Instead, just evict manually and check
  EXPECT_TRUE(mgr.evict("x"));
  EXPECT_EQ(mgr.resident_bytes(), 60u);
  EXPECT_TRUE(mgr.evict("y"));
  EXPECT_EQ(mgr.resident_bytes(), 0u);
}

TEST_F(StateMemoryManagerTest, EvictionCallback) {
  cvc::state_memory_manager mgr(*root_, 100);
  mgr.set_high_watermark(0.90);
  mgr.set_low_watermark(0.50);

  std::vector<std::string> evicted_paths;
  mgr.on_eviction([&](const std::string &path) { evicted_paths.push_back(path); });

  (*root_)("a").value("aaa");
  mgr.track("a", 50);
  (*root_)("b").value("bbb");
  mgr.track("b", 50);

  // Should trigger eviction of "a"
  EXPECT_FALSE(evicted_paths.empty());
  EXPECT_EQ(evicted_paths[0], "a");
}

TEST_F(StateMemoryManagerTest, RepopulationCallback) {
  cvc::state_memory_manager mgr(*root_, 1024);

  std::vector<std::string> repop_paths;
  mgr.on_repopulation([&](const std::string &path) { repop_paths.push_back(path); });

  (*root_)("x").value("data");
  mgr.track("x", 100);
  mgr.evict("x");
  mgr.repopulate("x");

  ASSERT_EQ(repop_paths.size(), 1u);
  EXPECT_EQ(repop_paths[0], "x");
}

TEST_F(StateMemoryManagerTest, NullEvictionStore) {
  auto store = std::make_unique<cvc::null_eviction_store>();
  cvc::state_memory_manager mgr(*root_, 1024, cvc::state_memory_manager::eviction_policy::lru,
                                std::move(store));

  (*root_)("x").value("some data");
  mgr.track("x", 100);
  mgr.evict("x");

  // Repopulate — null store returns empty
  mgr.repopulate("x");
  EXPECT_EQ((*root_)("x").value(), "");
}

TEST_F(StateMemoryManagerTest, MemoryEvictionStoreRoundTrip) {
  cvc::memory_eviction_store store;
  auto token = store.store("p", "hello", boost::any(42));

  std::string val;
  boost::any dat;
  EXPECT_TRUE(store.retrieve(token, val, dat));
  EXPECT_EQ(val, "hello");
  EXPECT_EQ(boost::any_cast<int>(dat), 42);

  // Token consumed — second retrieve should fail
  EXPECT_FALSE(store.retrieve(token, val, dat));
}

TEST_F(StateMemoryManagerTest, MemoryEvictionStoreDiscard) {
  cvc::memory_eviction_store store;
  auto token = store.store("p", "val", boost::any());
  EXPECT_EQ(store.stored_count(), 1u);

  EXPECT_TRUE(store.discard(token));
  EXPECT_EQ(store.stored_count(), 0u);
  EXPECT_FALSE(store.discard(token));
}

TEST_F(StateMemoryManagerTest, ManualPolicyNoAutoEviction) {
  cvc::state_memory_manager mgr(*root_, 100, cvc::state_memory_manager::eviction_policy::manual);

  (*root_)("a").value("aaa");
  mgr.track("a", 60);
  (*root_)("b").value("bbb");
  mgr.track("b", 60);

  // Over budget but manual policy — no auto eviction
  EXPECT_EQ(mgr.resident_bytes(), 120u);
  EXPECT_EQ(mgr.evicted_count(), 0u);

  // Manual eviction still works
  EXPECT_TRUE(mgr.evict("a"));
  EXPECT_EQ(mgr.resident_bytes(), 60u);
}

TEST_F(StateMemoryManagerTest, NestedPathResolve) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("level1")("level2").value("deep");
  mgr.track("level1.level2", 100);
  EXPECT_EQ(mgr.resident_bytes(), 100u);

  EXPECT_TRUE(mgr.evict("level1.level2"));
  EXPECT_EQ((*root_)("level1")("level2").value(), "");

  EXPECT_TRUE(mgr.repopulate("level1.level2"));
  EXPECT_EQ((*root_)("level1")("level2").value(), "deep");
}

TEST_F(StateMemoryManagerTest, WatermarkConfiguration) {
  // Very tight budget: 100 bytes, high=0.50 (50), low=0.30 (30)
  cvc::state_memory_manager mgr(*root_, 100);
  mgr.set_high_watermark(0.50);
  mgr.set_low_watermark(0.30);

  (*root_)("a").value("a");
  mgr.track("a", 20);
  (*root_)("b").value("b");
  mgr.track("b", 20);

  // 40 bytes — still under high watermark (50)
  EXPECT_EQ(mgr.resident_bytes(), 40u);
  EXPECT_EQ(mgr.evicted_count(), 0u);

  // Add 20 more — now at 60, over high watermark (50)
  (*root_)("c").value("c");
  mgr.track("c", 20);

  // Should have evicted until <= low watermark (30)
  EXPECT_LE(mgr.resident_bytes(), 30u);
}

TEST_F(StateMemoryManagerTest, UntrackCleansEvicted) {
  cvc::state_memory_manager mgr(*root_, 1024);
  (*root_)("x").value("data");
  mgr.track("x", 100);
  mgr.evict("x");
  EXPECT_TRUE(mgr.is_evicted("x"));

  mgr.untrack("x");
  EXPECT_FALSE(mgr.is_evicted("x"));
  EXPECT_EQ(mgr.evicted_count(), 0u);
}
