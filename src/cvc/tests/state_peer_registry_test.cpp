/*
  Copyright 2026 The University of Texas at Austin
  Phase 5 — state_peer_registry tests.
*/

#include <atomic>
#include <cvc/state/state_peer_registry.h>
#include <gtest/gtest.h>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using cvc::state_peer_registry;

namespace {

// Denies every path under one prefix. The prefix is owned by the filter, so evaluating a filter
// that was already freed reads freed memory (which a sanitizer build reports).
class deny_prefix_filter final : public cvc::state_delivery_filter {
public:
  explicit deny_prefix_filter(std::string prefix) : _prefix(std::move(prefix)) {}
  bool deliver(const std::string &path) const noexcept override {
    return !state_peer_registry::prefix_matches(_prefix, path);
  }

private:
  std::string _prefix;
};

std::shared_ptr<const cvc::state_delivery_filter> deny(const std::string &prefix) {
  return std::make_shared<deny_prefix_filter>(prefix);
}

} // namespace

TEST(StatePeerRegistry, PrefixMatchesEmptyMatchesAll) {
  EXPECT_TRUE(state_peer_registry::prefix_matches("", "anything"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("", ""));
}

TEST(StatePeerRegistry, PrefixMatchesExactAndDotBoundary) {
  EXPECT_TRUE(state_peer_registry::prefix_matches("a", "a"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("a", "a.b"));
  EXPECT_TRUE(state_peer_registry::prefix_matches("a.b", "a.b.c"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a", "ab"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a.b", "a.bc"));
  EXPECT_FALSE(state_peer_registry::prefix_matches("a.b", "a"));
}

TEST(StatePeerRegistry, AnyPrefixMatchesEmptyListIsMatchAll) {
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({}, "x.y"));
}

TEST(StatePeerRegistry, AnyPrefixMatchesNonEmptyFilters) {
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({"a", "x"}, "a.b"));
  EXPECT_TRUE(state_peer_registry::any_prefix_matches({"a", "x"}, "x"));
  EXPECT_FALSE(state_peer_registry::any_prefix_matches({"a", "x"}, "y"));
}

TEST(StatePeerRegistry, AddRemoveHas) {
  state_peer_registry r;
  EXPECT_FALSE(r.has_peer("n1"));
  r.add_peer("n1", "c", "addr");
  EXPECT_TRUE(r.has_peer("n1"));
  EXPECT_EQ(1u, r.size());
  // re-add overwrites; size stays 1.
  r.add_peer("n1", "c", "addr");
  EXPECT_EQ(1u, r.size());
  EXPECT_TRUE(r.remove_peer("n1"));
  EXPECT_FALSE(r.remove_peer("n1"));
  EXPECT_FALSE(r.has_peer("n1"));
  EXPECT_EQ(0u, r.size());
}

TEST(StatePeerRegistry, ShouldDeliverUnknownPeerIsTrue) {
  state_peer_registry r;
  EXPECT_TRUE(r.should_deliver("unknown_node", "any.path"));
}

TEST(StatePeerRegistry, ShouldDeliverEmptySubscriptionsIsMatchAll) {
  state_peer_registry r;
  r.add_peer("n1", "c", "");
  EXPECT_TRUE(r.should_deliver("n1", "a.b"));
  EXPECT_TRUE(r.should_deliver("n1", ""));
}

TEST(StatePeerRegistry, ShouldDeliverFiltersByPrefix) {
  state_peer_registry r;
  r.add_peer("n1", "c", "");
  EXPECT_TRUE(r.set_subscriptions("n1", {"alpha", "beta.gamma"}));
  EXPECT_TRUE(r.should_deliver("n1", "alpha"));
  EXPECT_TRUE(r.should_deliver("n1", "alpha.x.y"));
  EXPECT_TRUE(r.should_deliver("n1", "beta.gamma.z"));
  EXPECT_FALSE(r.should_deliver("n1", "beta"));
  EXPECT_FALSE(r.should_deliver("n1", "alphax"));
  EXPECT_FALSE(r.should_deliver("n1", "gamma"));
}

TEST(StatePeerRegistry, CountersAndSnapshot) {
  state_peer_registry r;
  r.add_peer("n1", "c", "addr1");
  r.note_mutation_delivered("n1");
  r.note_mutation_delivered("n1");
  r.note_message_delivered("n1");
  r.note_delivery_filtered("n1");
  r.note_seen("n1", 12345);
  auto snap = r.snapshot();
  ASSERT_EQ(1u, snap.size());
  EXPECT_EQ("n1", snap[0].node_id);
  EXPECT_EQ(2u, snap[0].mutations_delivered);
  EXPECT_EQ(1u, snap[0].messages_delivered);
  EXPECT_EQ(1u, snap[0].deliveries_filtered);
  EXPECT_EQ(12345u, snap[0].last_seen_ns);
}

TEST(StatePeerRegistry, ClearWipesAll) {
  state_peer_registry r;
  r.add_peer("a", "c", "");
  r.add_peer("b", "c", "");
  r.clear();
  EXPECT_EQ(0u, r.size());
  EXPECT_FALSE(r.has_peer("a"));
}

// ---- Delivery filter ----

TEST(StatePeerRegistry, FilterDeniesAfterPrefixesAllow) {
  state_peer_registry r;
  r.add_peer("n1", "c", "", {"conf"});
  ASSERT_TRUE(r.set_filter("n1", deny("conf.rooms.r.participants.q.pose")));

  // Both rules must pass: the prefix admits all of conf, the filter then takes q's pose back out.
  EXPECT_TRUE(r.should_deliver("n1", "conf.rooms.r.participants.q.presence.joined"));
  EXPECT_FALSE(r.should_deliver("n1", "conf.rooms.r.participants.q.pose"));
  EXPECT_FALSE(r.should_deliver("n1", "conf.rooms.r.participants.q.pose.x"));
  EXPECT_TRUE(r.should_deliver("n1", "conf.rooms.r.participants.q.posex"));
  // A path the prefixes refuse stays refused although the filter would pass it.
  EXPECT_FALSE(r.should_deliver("n1", "ui.docs.x"));

  // A filter also denies on a match-all peer (no prefixes).
  r.add_peer("n2", "c", "");
  ASSERT_TRUE(r.set_filter("n2", deny("ui")));
  EXPECT_FALSE(r.should_deliver("n2", "ui.docs.x"));
  EXPECT_TRUE(r.should_deliver("n2", "conf.x"));

  // Clearing the filter leaves the prefixes in force.
  ASSERT_TRUE(r.set_filter("n1", nullptr));
  EXPECT_TRUE(r.should_deliver("n1", "conf.rooms.r.participants.q.pose.x"));
  EXPECT_FALSE(r.should_deliver("n1", "ui.docs.x"));
}

TEST(StatePeerRegistry, UnknownPeerStillDelivers) {
  state_peer_registry r;
  r.add_peer("n1", "c", "", {"conf"});
  ASSERT_TRUE(r.set_filter("n1", deny("conf")));

  // Another peer's rules never leak onto an unregistered id.
  EXPECT_TRUE(r.should_deliver("n2", "conf.x"));
  EXPECT_TRUE(r.should_deliver("n2", "ui.docs.x"));
  const state_peer_registry::delivery_view v = r.view_of("n2");
  EXPECT_FALSE(v.known);
  EXPECT_FALSE(v.filtered());
  EXPECT_TRUE(v.subscriptions.empty());
  EXPECT_EQ(v.filter, nullptr);
  EXPECT_TRUE(v.deliver("conf.x"));
  EXPECT_TRUE(v.deliver(""));
}

TEST(StatePeerRegistry, SetFilterOnUnknownPeerFails) {
  state_peer_registry r;
  EXPECT_FALSE(r.set_filter("ghost", deny("a")));
  EXPECT_FALSE(r.set_filter("ghost", nullptr));
  // The failed call registers nothing and leaves nothing behind for a later add_peer.
  EXPECT_FALSE(r.has_peer("ghost"));
  EXPECT_EQ(0u, r.size());
  EXPECT_TRUE(r.should_deliver("ghost", "a.x"));
  r.add_peer("ghost", "c", "");
  EXPECT_TRUE(r.should_deliver("ghost", "a.x"));
  EXPECT_EQ(r.view_of("ghost").filter, nullptr);
}

TEST(StatePeerRegistry, AddPeerAgainKeepsFilter) {
  state_peer_registry r;
  r.add_peer("n1", "c", "addr1", {"a"});
  const auto f = deny("a.secret");
  ASSERT_TRUE(r.set_filter("n1", f));

  // A re-add (a reconnect re-announcing the peer) replaces the prefixes, cluster and endpoint...
  r.add_peer("n1", "c2", "addr2", {"a", "b"});
  EXPECT_TRUE(r.should_deliver("n1", "b.x"));
  // ...and keeps the filter.
  EXPECT_FALSE(r.should_deliver("n1", "a.secret.k"));
  EXPECT_TRUE(r.should_deliver("n1", "a.pub"));

  auto snap = r.snapshot();
  ASSERT_EQ(1u, snap.size());
  EXPECT_EQ("c2", snap[0].cluster_id);
  EXPECT_EQ("addr2", snap[0].endpoint);
  EXPECT_EQ(f, snap[0].filter);
  EXPECT_EQ(f, r.view_of("n1").filter);
}

TEST(StatePeerRegistry, RemovePeerDropsFilter) {
  state_peer_registry r;
  r.add_peer("n1", "c", "", {"a"});
  std::weak_ptr<const cvc::state_delivery_filter> watch;
  {
    auto f = deny("a.secret");
    watch = f;
    ASSERT_TRUE(r.set_filter("n1", std::move(f)));
  }
  EXPECT_FALSE(r.should_deliver("n1", "a.secret"));

  EXPECT_TRUE(r.remove_peer("n1"));
  EXPECT_TRUE(watch.expired()); // the registry held the last reference
  EXPECT_TRUE(r.should_deliver("n1", "a.secret"));

  // Coming back under the same id starts without a filter.
  r.add_peer("n1", "c", "", {"a"});
  EXPECT_TRUE(r.should_deliver("n1", "a.secret"));
  EXPECT_EQ(r.view_of("n1").filter, nullptr);

  // Replacing and clearing release the old filter too.
  ASSERT_TRUE(r.set_filter("n1", deny("a.x")));
  std::weak_ptr<const cvc::state_delivery_filter> replaced = r.view_of("n1").filter;
  ASSERT_TRUE(r.set_filter("n1", deny("a.y")));
  EXPECT_TRUE(replaced.expired());
  std::weak_ptr<const cvc::state_delivery_filter> cleared = r.view_of("n1").filter;
  r.clear();
  EXPECT_TRUE(cleared.expired());
}

TEST(StatePeerRegistry, ViewOfMatchesShouldDeliver) {
  state_peer_registry r;
  r.add_peer("open", "c", "");
  r.add_peer("subs", "c", "", {"conf", "ui.docs.a"});
  r.add_peer("filt", "c", "");
  ASSERT_TRUE(r.set_filter("filt", deny("conf.x")));
  r.add_peer("both", "c", "", {"conf"});
  ASSERT_TRUE(r.set_filter("both", deny("conf.rooms.r.participants.p1")));

  struct expected_view {
    const char *id;
    bool known;
    bool filtered;
  };
  const expected_view peers[] = {{"open", true, false},
                                 {"subs", true, true},
                                 {"filt", true, true},
                                 {"both", true, true},
                                 {"unknown", false, false}};

  // Paths are a stem near the rules' boundaries plus a random tail, so every rule both passes and
  // refuses some of them.
  const char *stems[] = {"",
                         "conf",
                         "conf.x",
                         "conf.xy",
                         "conf.rooms.r.participants.p1",
                         "conf.rooms.r.participants.p10",
                         "conf.rooms.r.participants.p2",
                         "ui",
                         "ui.docs.a",
                         "ui.docs.ab",
                         "scene"};
  const char *tails[] = {"pose", "x", "joined", "matrix", "p1"};
  std::mt19937 rng(20261024u);
  std::uniform_int_distribution<int> pick_stem(0, static_cast<int>(std::size(stems)) - 1);
  std::uniform_int_distribution<int> pick_tail(0, static_cast<int>(std::size(tails)) - 1);
  std::uniform_int_distribution<int> tail_len(0, 3);
  std::vector<std::string> paths;
  for (int i = 0; i < 2000; ++i) {
    std::string p = stems[pick_stem(rng)];
    for (int k = tail_len(rng); k > 0; --k)
      p += (p.empty() ? "" : ".") + std::string(tails[pick_tail(rng)]);
    paths.push_back(std::move(p));
  }

  for (const expected_view &e : peers) {
    const state_peer_registry::delivery_view v = r.view_of(e.id);
    EXPECT_EQ(v.known, e.known) << e.id;
    EXPECT_EQ(v.filtered(), e.filtered) << e.id;
    std::size_t delivered = 0;
    for (const std::string &p : paths) {
      const bool want = r.should_deliver(e.id, p);
      ASSERT_EQ(v.deliver(p), want) << e.id << " '" << p << "'";
      delivered += want ? 1 : 0;
    }
    if (e.filtered) {
      EXPECT_GT(delivered, 0u) << e.id;
      EXPECT_LT(delivered, paths.size()) << e.id;
    } else {
      EXPECT_EQ(delivered, paths.size()) << e.id;
    }
  }
}

TEST(StatePeerRegistry, ConcurrentSetFilterAndShouldDeliver) {
  // One thread swaps the peer's filter as fast as it can while another asks for deliveries.
  // Paths the swap cannot affect must answer the same every time, and a filter replaced
  // mid-evaluation must stay alive until the reader is done with it.
  constexpr int kIterations = 100000;
  state_peer_registry r;
  r.add_peer("n1", "c", "", {"conf"});

  std::atomic<bool> go{false};
  std::atomic<int> wrong{0};
  std::thread writer([&]() {
    while (!go.load(std::memory_order_acquire)) {
    }
    // Odd iterations install a fresh deny filter, even ones clear it; the last one is odd.
    for (int i = 0; i < kIterations; ++i)
      r.set_filter("n1", (i % 2 == 1) ? deny("conf.secret") : nullptr);
  });
  std::thread reader([&]() {
    while (!go.load(std::memory_order_acquire)) {
    }
    for (int i = 0; i < kIterations; ++i) {
      bool pub;
      bool outside;
      if (i % 2 == 0) {
        pub = r.should_deliver("n1", "conf.pub.x");
        outside = r.should_deliver("n1", "ui.docs.x");
        (void)r.should_deliver("n1", "conf.secret.x"); // either answer; it must not crash
      } else {
        const state_peer_registry::delivery_view v = r.view_of("n1");
        pub = v.deliver("conf.pub.x");
        outside = v.deliver("ui.docs.x");
        (void)v.deliver("conf.secret.x");
      }
      if (!pub || outside)
        wrong.fetch_add(1, std::memory_order_relaxed);
    }
  });
  go.store(true, std::memory_order_release);
  writer.join();
  reader.join();

  EXPECT_EQ(wrong.load(), 0);
  EXPECT_FALSE(r.should_deliver("n1", "conf.secret.x"));
  EXPECT_TRUE(r.should_deliver("n1", "conf.pub.x"));
}
