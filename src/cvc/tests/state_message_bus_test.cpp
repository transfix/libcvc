// SPDX-License-Identifier: LGPL-2.1
// Tests for cvc::state_message_bus (Phase 4 OOB messaging).

#include <atomic>
#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>
#include <gtest/gtest.h>
#include <string>
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
