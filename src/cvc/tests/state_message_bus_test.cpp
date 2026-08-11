// SPDX-License-Identifier: LGPL-2.1
// Tests for cvc::state_message_bus (Phase 4 OOB messaging).

#include <atomic>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <gtest/gtest.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using cvc::state_message;
using cvc::state_message_bus;

namespace {

state_message make_msg(const std::string &origin, const std::string &id, const std::string &path,
                       const std::string &str = {}) {
  state_message m;
  m.origin_node_id = origin;
  m.message_id = id;
  m.path = path;
  m.string_value = str;
  m.content_type = "text/plain";
  return m;
}

} // namespace

TEST(StateMessageBus, PrefixMatches) {
  EXPECT_TRUE(state_message_bus::prefix_matches("", "anything"));
  EXPECT_TRUE(state_message_bus::prefix_matches("", ""));
  EXPECT_TRUE(state_message_bus::prefix_matches("p", "p"));
  EXPECT_TRUE(state_message_bus::prefix_matches("p", "p.x"));
  EXPECT_TRUE(state_message_bus::prefix_matches("p", "p.x.y"));
  EXPECT_FALSE(state_message_bus::prefix_matches("p", "px"));
  EXPECT_FALSE(state_message_bus::prefix_matches("p", "q"));
  EXPECT_FALSE(state_message_bus::prefix_matches("p.x", "p"));
}

TEST(StateMessageBus, DedupSameOriginAndId) {
  state_message_bus bus;
  std::atomic<int> hits{0};
  bus.subscribe("", [&](const state_message &) { hits.fetch_add(1); });

  auto m = make_msg("A", "1", "p.x", "hello");
  EXPECT_TRUE(bus.admit(m));
  EXPECT_FALSE(bus.admit(m));
  EXPECT_FALSE(bus.admit(m));

  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(bus.total_admitted(), 1u);
  EXPECT_EQ(bus.total_duplicates(), 2u);
  EXPECT_EQ(bus.total_dispatched(), 1u);
}

TEST(StateMessageBus, MultipleSubscribersFireOncePerAdmit) {
  state_message_bus bus;
  std::atomic<int> a{0}, b{0}, c{0};
  bus.subscribe("", [&](const state_message &) { a.fetch_add(1); });
  bus.subscribe("p", [&](const state_message &) { b.fetch_add(1); });
  bus.subscribe("p.x", [&](const state_message &) { c.fetch_add(1); });

  EXPECT_TRUE(bus.admit(make_msg("A", "1", "p.x")));
  EXPECT_TRUE(bus.admit(make_msg("A", "2", "p.y")));
  EXPECT_TRUE(bus.admit(make_msg("A", "3", "q")));

  EXPECT_EQ(a.load(), 3);
  EXPECT_EQ(b.load(), 2); // p.x and p.y
  EXPECT_EQ(c.load(), 1); // p.x only
  EXPECT_EQ(bus.total_dispatched(), 6u);
}

TEST(StateMessageBus, UnsubscribeStopsDelivery) {
  state_message_bus bus;
  std::atomic<int> hits{0};
  auto id = bus.subscribe("", [&](const state_message &) { hits.fetch_add(1); });

  EXPECT_TRUE(bus.admit(make_msg("A", "1", "x")));
  EXPECT_EQ(hits.load(), 1);

  EXPECT_TRUE(bus.unsubscribe(id));
  EXPECT_FALSE(bus.unsubscribe(id));

  EXPECT_TRUE(bus.admit(make_msg("A", "2", "x")));
  EXPECT_EQ(hits.load(), 1);
  EXPECT_EQ(bus.subscriber_count(), 0u);
}

TEST(StateMessageBus, EmptyKeyBypassesDedup) {
  state_message_bus bus;
  std::atomic<int> hits{0};
  bus.subscribe("", [&](const state_message &) { hits.fetch_add(1); });

  // Empty origin and message_id => sentinel: every admit fires.
  state_message m;
  m.path = "x";
  EXPECT_TRUE(bus.admit(m));
  EXPECT_TRUE(bus.admit(m));
  EXPECT_TRUE(bus.admit(m));
  EXPECT_EQ(hits.load(), 3);
  EXPECT_EQ(bus.total_duplicates(), 0u);
}

TEST(StateMessageBus, DedupCapacityEviction) {
  state_message_bus bus;
  bus.set_dedup_capacity(2);

  EXPECT_TRUE(bus.admit(make_msg("A", "1", "x")));
  EXPECT_TRUE(bus.admit(make_msg("A", "2", "x")));
  EXPECT_TRUE(bus.admit(make_msg("A", "3", "x"))); // evicts (A,1)

  // (A,1) was evicted; should be admitted again.
  EXPECT_TRUE(bus.admit(make_msg("A", "1", "x")));
  // (A,3) is still tracked.
  EXPECT_FALSE(bus.admit(make_msg("A", "3", "x")));

  EXPECT_LE(bus.dedup_size(), 2u);
}

TEST(StateMessageBus, NoteDroppedBumpsCounter) {
  state_message_bus bus;
  EXPECT_EQ(bus.total_dropped(), 0u);
  bus.note_dropped();
  bus.note_dropped();
  bus.note_dropped();
  EXPECT_EQ(bus.total_dropped(), 3u);
}

TEST(StateMessageBus, DistinctOriginsAreNotDeduped) {
  state_message_bus bus;
  std::atomic<int> hits{0};
  bus.subscribe("", [&](const state_message &) { hits.fetch_add(1); });

  EXPECT_TRUE(bus.admit(make_msg("A", "1", "x")));
  EXPECT_TRUE(bus.admit(make_msg("B", "1", "x")));
  EXPECT_TRUE(bus.admit(make_msg("C", "1", "x")));
  EXPECT_FALSE(bus.admit(make_msg("A", "1", "x")));
  EXPECT_EQ(hits.load(), 3);
}

// The same logical stream can reach a bus over more than one path --
// two transport reader threads carrying one peer's messages, say.
// Dedup picks the right winner for each message, but if dispatch
// happened after the lock were dropped the winners could reach
// subscribers in the wrong order: thread A wins m2 and thread B wins
// m3, then B's callback runs first. Both threads feed the same
// ordered sequence here, so subscribers must see exactly that order.
TEST(StateMessageBus, ConcurrentAdmitOfOneStreamDispatchesInOrder) {
  const int kMessages = 400;
  const int kRounds = 20;

  for (int round = 0; round < kRounds; ++round) {
    state_message_bus bus;
    std::mutex mu;
    std::vector<int> seen;
    bus.subscribe("s", [&](const state_message &m) {
      std::lock_guard<std::mutex> lk(mu);
      seen.push_back(std::stoi(m.string_value));
    });

    std::atomic<bool> go{false};
    auto feed = [&]() {
      while (!go.load(std::memory_order_acquire)) {
      }
      for (int i = 0; i < kMessages; ++i)
        bus.admit(make_msg("A", "m" + std::to_string(i), "s.chan", std::to_string(i)));
    };
    std::thread t1(feed), t2(feed);
    go.store(true, std::memory_order_release);
    t1.join();
    t2.join();

    std::lock_guard<std::mutex> lk(mu);
    ASSERT_EQ(seen.size(), static_cast<std::size_t>(kMessages))
        << "each message should be dispatched exactly once (round " << round << ")";
    for (int i = 0; i < kMessages; ++i) {
      ASSERT_EQ(seen[i], i) << "subscriber saw the stream out of order at index " << i << " (round "
                            << round << ")";
    }
  }
}

// A subscriber is allowed to call back into the bus on the
// dispatching thread. admit() holds its lock across dispatch, so that
// lock has to be recursive or this deadlocks.
TEST(StateMessageBus, SubscriberMayReenterTheBus) {
  state_message_bus bus;
  std::atomic<int> outer{0};
  std::atomic<int> inner{0};

  bus.subscribe("in", [&](const state_message &) { inner.fetch_add(1); });
  bus.subscribe("out", [&](const state_message &) {
    outer.fetch_add(1);
    // Re-entrant admit, plus a read and a subscription change.
    bus.admit(make_msg("A", "nested", "in.chan", "x"));
    (void)bus.total_admitted();
    auto id = bus.subscribe("never", [](const state_message &) {});
    bus.unsubscribe(id);
  });

  EXPECT_TRUE(bus.admit(make_msg("A", "outer", "out.chan", "y")));
  EXPECT_EQ(outer.load(), 1);
  EXPECT_EQ(inner.load(), 1);
}
