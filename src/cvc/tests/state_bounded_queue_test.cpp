// SPDX-License-Identifier: LGPL-2.1
// Tests for cvc::state_bounded_queue (Phase 6 backpressure /
// bounded-queue policy on transports).

#include <atomic>
#include <chrono>
#include <cvc/core/state_bounded_queue.h>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vector>

using cvc::state_bounded_queue;
using policy_t = state_bounded_queue<int>::overflow_policy;

TEST(StateBoundedQueue, ZeroCapacityNormalizedToOne) {
  state_bounded_queue<int> q(0, policy_t::drop_newest);
  EXPECT_EQ(q.capacity(), 1u);
}

TEST(StateBoundedQueue, BasicFifoPushPop) {
  state_bounded_queue<int> q(8, policy_t::drop_newest);
  for (int i = 0; i < 5; ++i)
    EXPECT_TRUE(q.push(i));
  EXPECT_EQ(q.size(), 5u);

  int v = -1;
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(q.try_pop(v));
    EXPECT_EQ(v, i);
  }
  EXPECT_FALSE(q.try_pop(v));
  EXPECT_EQ(q.total_admitted(), 5u);
  EXPECT_EQ(q.total_popped(), 5u);
}

TEST(StateBoundedQueue, DropNewestRejectsOverflow) {
  state_bounded_queue<int> q(3, policy_t::drop_newest);
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_TRUE(q.push(3));
  EXPECT_FALSE(q.push(4));
  EXPECT_FALSE(q.push(5));
  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.total_dropped_newest(), 2u);
  EXPECT_EQ(q.total_dropped_oldest(), 0u);

  int v = 0;
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 2);
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 3);
}

TEST(StateBoundedQueue, DropOldestEvictsFront) {
  state_bounded_queue<int> q(3, policy_t::drop_oldest);
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_TRUE(q.push(3));
  EXPECT_TRUE(q.push(4)); // evicts 1
  EXPECT_TRUE(q.push(5)); // evicts 2
  EXPECT_EQ(q.size(), 3u);
  EXPECT_EQ(q.total_dropped_oldest(), 2u);
  EXPECT_EQ(q.total_dropped_newest(), 0u);

  int v = 0;
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 3);
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 4);
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 5);
}

TEST(StateBoundedQueue, BlockPolicyTimesOutWhenFull) {
  state_bounded_queue<int> q(2, policy_t::block);
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_FALSE(q.push_for(3, std::chrono::milliseconds(20)));
  EXPECT_EQ(q.total_blocked_timeouts(), 1u);
  EXPECT_EQ(q.size(), 2u);
}

TEST(StateBoundedQueue, BlockPolicyUnblocksOnPop) {
  state_bounded_queue<int> q(2, policy_t::block);
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));

  std::atomic<bool> done{false};
  std::thread producer([&]() {
    EXPECT_TRUE(q.push_for(3, std::chrono::seconds(2)));
    done = true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(done);

  int v = 0;
  EXPECT_TRUE(q.try_pop(v));
  EXPECT_EQ(v, 1);

  producer.join();
  EXPECT_TRUE(done);
  EXPECT_EQ(q.size(), 2u);
  EXPECT_EQ(q.total_blocked_timeouts(), 0u);
}

TEST(StateBoundedQueue, PopBlocksUntilPushed) {
  state_bounded_queue<int> q(4, policy_t::drop_newest);
  std::atomic<bool> popped{false};
  int got = -1;
  std::thread consumer([&]() {
    EXPECT_TRUE(q.pop(got));
    popped = true;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(popped);
  EXPECT_TRUE(q.push(42));
  consumer.join();
  EXPECT_TRUE(popped);
  EXPECT_EQ(got, 42);
}

TEST(StateBoundedQueue, CloseUnblocksPopAndRejectsPush) {
  state_bounded_queue<int> q(2, policy_t::block);
  std::atomic<bool> done{false};
  std::thread consumer([&]() {
    int v = 0;
    bool ok = q.pop(v);
    EXPECT_FALSE(ok);
    done = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(done);
  q.close();
  consumer.join();
  EXPECT_TRUE(done);
  EXPECT_TRUE(q.closed());

  EXPECT_FALSE(q.push(7));
  EXPECT_EQ(q.total_rejected_closed(), 1u);
}

TEST(StateBoundedQueue, CloseUnblocksBlockedProducer) {
  state_bounded_queue<int> q(1, policy_t::block);
  EXPECT_TRUE(q.push(1));

  std::atomic<bool> producer_returned{false};
  std::thread producer([&]() {
    bool ok = q.push(2);
    EXPECT_FALSE(ok);
    producer_returned = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  EXPECT_FALSE(producer_returned);
  q.close();
  producer.join();
  EXPECT_TRUE(producer_returned);
  EXPECT_GE(q.total_rejected_closed(), 1u);
}

TEST(StateBoundedQueue, ConcurrentProducerConsumerDropNewest) {
  state_bounded_queue<int> q(64, policy_t::drop_newest);
  constexpr int N = 5000;

  std::atomic<int> pops{0};
  std::thread consumer([&]() {
    int v = 0;
    while (q.pop(v))
      pops.fetch_add(1, std::memory_order_relaxed);
  });

  for (int i = 0; i < N; ++i)
    q.push(i);
  q.close();
  consumer.join();

  // Every popped + dropped item must equal N (no duplication, no
  // loss outside the policy).
  EXPECT_EQ(pops.load() + (long long)q.total_dropped_newest(), (long long)N);
  EXPECT_EQ((long long)q.total_admitted() + (long long)q.total_dropped_newest(), (long long)N);
}

TEST(StateBoundedQueue, ConcurrentProducerConsumerDropOldest) {
  state_bounded_queue<int> q(64, policy_t::drop_oldest);
  constexpr int N = 5000;

  std::atomic<int> pops{0};
  std::thread consumer([&]() {
    int v = 0;
    while (q.pop(v))
      pops.fetch_add(1, std::memory_order_relaxed);
  });

  for (int i = 0; i < N; ++i)
    q.push(i);
  q.close();
  consumer.join();

  // drop_oldest never rejects, so admitted == N.
  EXPECT_EQ((long long)q.total_admitted(), (long long)N);
  EXPECT_EQ(pops.load() + (long long)q.total_dropped_oldest(), (long long)N);
}

TEST(StateBoundedQueue, MoveOnlyValueType) {
  using uq_policy = state_bounded_queue<std::unique_ptr<int>>::overflow_policy;
  state_bounded_queue<std::unique_ptr<int>> q(2, uq_policy::drop_newest);
  EXPECT_TRUE(q.push(std::make_unique<int>(7)));
  EXPECT_TRUE(q.push(std::make_unique<int>(8)));
  EXPECT_FALSE(q.push(std::make_unique<int>(9)));
  std::unique_ptr<int> out;
  EXPECT_TRUE(q.try_pop(out));
  ASSERT_NE(out, nullptr);
  EXPECT_EQ(*out, 7);
}
