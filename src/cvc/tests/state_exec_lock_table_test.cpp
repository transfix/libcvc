/// @file state_exec_lock_table_test.cpp
/// @brief Directed cases of the lock manager's table, no scheduler (suite LockTable; locks plan
///        L1.2), and of its shared mode, several homes and ports (suites LockShared, LockHomes;
///        L3). Compiled into state_exec_locks_model_test beside the model check.
///
/// Each LockTable case drives detail::lock_table as a lock home would (protocol section 4), on a
/// table built with options::self_check on, and calls check() after every operation. The case
/// names follow the design's test list (docs/STATE_EXEC_LOCKS.md section 13) where a case is the
/// manager-level shape of one of its tests. The LockHomes cases drive real lock homes over a
/// minimal fake scheduler (lock_sched_ops); the scheduler-driven cross-thread cases (LT-*) are
/// state_exec_locks_test's.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <deque>
#include <functional>
#include <gtest/gtest.h>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc::state_exec;
using detail::lock_table;
using detail::ticket_state;

namespace {

constexpr lock_mode S = lock_mode::shared;
constexpr lock_mode X = lock_mode::exclusive;

using keys_t = std::initializer_list<std::string_view>;

/// A table with one attached home and helpers that call check() after each operation.
struct world {
  lock_table t;
  detail::home_id h = 0;
  detail::lock_notes notes; // the last operation's

  /// `prio`: the home applies priorities (a scheduler with a priority policy, phase L2).
  explicit world(std::uint32_t mutations = 0, bool self_check = true, bool prio = false)
      : t(options(mutations, self_check)) {
    detail::lock_notes n;
    h = t.attach_home(prio, nullptr, nullptr, n);
    EXPECT_TRUE(n.empty());
    checked();
  }

  static lock_table::options options(std::uint32_t mutations, bool self_check) {
    lock_table::options o;
    o.self_check = self_check;
    o.mutations = mutations;
    return o;
  }

  /// check(), except in the mutation cases that expect a violation (they call it themselves).
  bool auto_check = true;
  void checked() {
    if (auto_check)
      t.check();
  }

  detail::lock_pid pid(int p) const { return {h, p}; }

  static detail::lock_request request(lock_mode m, keys_t keys) {
    detail::lock_request r;
    r.mode = m;
    for (std::string_view k : keys)
      r.keys.push_back(detail::key_ref{nullptr, k, false});
    return r;
  }

  std::optional<detail::grant> try_acquire(int p, lock_mode m, keys_t keys,
                                           std::string_view owner = "o", int base = 0) {
    auto r = request(m, keys);
    auto g = t.try_acquire(pid(p), {owner, base}, r, {});
    checked();
    return g;
  }

  /// A try_acquire that must be granted: the hold's id.
  std::uint64_t acquire(int p, lock_mode m, keys_t keys, std::string_view owner = "o",
                        int base = 0) {
    auto g = try_acquire(p, m, keys, owner, base);
    EXPECT_TRUE(g.has_value()) << "pid " << p;
    return g ? g->hold : 0;
  }

  detail::block_result block(int p, lock_mode m, keys_t keys, std::string_view owner = "o",
                             int base = 0) {
    notes.clear();
    auto r = request(m, keys);
    auto b = t.block(pid(p), {owner, base}, r, {}, notes);
    checked();
    return b;
  }

  /// A busy request that queues, as %lock-wait does after a busy try_acquire: its ticket.
  std::uint64_t wait(int p, lock_mode m, keys_t keys, std::string_view owner = "o", int base = 0) {
    EXPECT_FALSE(try_acquire(p, m, keys, owner, base).has_value()) << "pid " << p;
    auto b = block(p, m, keys, owner, base);
    EXPECT_EQ(b.state, ticket_state::queued) << "pid " << p;
    return b.tid;
  }

  void set_base(int p, int base) {
    notes.clear();
    t.set_base(pid(p), base, notes);
    checked();
  }

  /// A record's inherited priority, read as a check reads it (no side effect).
  int inh(int p) const {
    auto r = t.rec_of(pid(p));
    return r ? r->inh : detail::lock_inf;
  }

  /// The last operation's notifications as text: wake(PID,TID) and relevel(PID), in order.
  std::vector<std::string> posts() const {
    std::vector<std::string> v;
    for (const auto &n : notes) {
      EXPECT_EQ(n.home, h);
      v.push_back(n.post.what == detail::lock_post::kind::wake
                      ? "wake(" + std::to_string(n.post.pid) + "," + std::to_string(n.post.tid) +
                            ")"
                      : "relevel(" + std::to_string(n.post.pid) + ")");
    }
    return v;
  }

  detail::lock_outcome resume(int p, std::uint64_t tid) {
    auto o = t.resume(pid(p), tid);
    checked();
    return o;
  }

  /// Collect a granted ticket and begin its body (protocol 4.7, 4.8): the hold's id.
  std::uint64_t enter(int p, std::uint64_t tid) {
    auto o = resume(p, tid);
    EXPECT_EQ(o.what, detail::lock_outcome::kind::granted) << "pid " << p;
    t.start_body(pid(p), o.g.hold);
    checked();
    return o.g.hold;
  }

  void release(int p, std::uint64_t hold) {
    notes.clear();
    t.release(pid(p), hold, notes);
    checked();
  }

  std::optional<ticket_state> cancel(int p, std::uint64_t tid) {
    notes.clear();
    auto s = t.cancel(pid(p), tid, notes);
    checked();
    return s;
  }

  void end(int p, bool abandon) {
    notes.clear();
    t.end(pid(p), abandon, notes);
    checked();
  }

  /// The last operation's notifications, as (pid, tid) wakes.
  std::vector<std::pair<int, std::uint64_t>> wakes() const {
    std::vector<std::pair<int, std::uint64_t>> v;
    for (const auto &n : notes) {
      EXPECT_EQ(n.home, h);
      EXPECT_EQ(n.post.what, detail::lock_post::kind::wake);
      v.emplace_back(n.post.pid, n.post.tid);
    }
    return v;
  }

  ticket_state state(std::uint64_t tid) const {
    auto v = t.ticket_of(tid);
    EXPECT_TRUE(v.has_value()) << "ticket " << tid;
    return v ? v->state : ticket_state::failed;
  }
};

using wake_list = std::vector<std::pair<int, std::uint64_t>>;

std::vector<std::pair<std::optional<int>, std::string>>
edges(const std::vector<detail::deadlock_edge> &cycle) {
  std::vector<std::pair<std::optional<int>, std::string>> v;
  for (const auto &e : cycle)
    v.emplace_back(e.pid, e.key);
  return v;
}

std::string violation_of(const lock_table &t) {
  try {
    t.check();
  } catch (const detail::lock_violation &v) {
    return v.invariant();
  }
  return "";
}

/// The keys of a key-cache reference request.
std::vector<detail::key_ref> refs_of(keys_t keys) {
  std::vector<detail::key_ref> v;
  for (std::string_view k : keys)
    v.push_back(detail::key_ref{nullptr, k, false});
  return v;
}

} // namespace

TEST(LockTable, Constructs) {
  lock_table t(lock_table::options{});
  EXPECT_TRUE(t.empty());
  EXPECT_EQ(t.stats().trie_nodes, 1u);
  t.check();
}

// -- 4.4 ------------------------------------------------------------------------------------------

TEST(LockTable, AttachHomeOneHomeBeforeL3) {
  lock_table t(world::options(0, true));
  detail::lock_notes n;
  EXPECT_EQ(t.attach_home(false, nullptr, nullptr, n), 0u);
  EXPECT_THROW(t.attach_home(false, nullptr, nullptr, n), std::logic_error);
  t.check();
  t.detach_home(0, n);
  t.check();
  EXPECT_THROW(t.detach_home(0, n), detail::lock_violation);
  EXPECT_EQ(t.attach_home(false, nullptr, nullptr, n), 1u); // ids are never reused
  EXPECT_TRUE(n.empty());
  EXPECT_NE(t.canonical().find("homes\nH1 prio=0\nrecs\n"), std::string::npos);
  // prio is taken as given (phase L2), with inheritance on or off; one_home off takes more homes.
  lock_table::options o = world::options(0, true);
  o.one_home = false;
  lock_table t2(o);
  EXPECT_EQ(t2.attach_home(true, nullptr, nullptr, n), 0u);
  EXPECT_EQ(t2.attach_home(false, nullptr, nullptr, n), 1u);
  EXPECT_TRUE(n.empty());
  EXPECT_NE(t2.canonical().find("homes\nH0 prio=1\nH1 prio=0\nrecs\n"), std::string::npos);
  t2.check();
  o.inheritance = false;
  lock_table t3(o);
  EXPECT_EQ(t3.attach_home(true, nullptr, nullptr, n), 0u);
  EXPECT_NE(t3.canonical().find("H0 prio=1"), std::string::npos);
  t3.check();
}

// -- 3.3, 4.3, 4.12, 4.13: inheritance (phase L2) -------------------------------------------------

TEST(LockTable, InheritanceBoostsTheHolderAndWithdrawsIt) {
  // PI-1's shape: a better waiter boosts the holder; its timeout withdraws the boost
  world w(0, true, true);
  w.acquire(1, X, {"a"}, "o", 5);
  const std::uint64_t t2 = w.wait(2, X, {"a"}, "o", -5);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)"})); // the waiter's own is unchanged
  EXPECT_EQ(w.inh(1), -5);
  EXPECT_EQ(w.inh(2), detail::lock_inf);
  EXPECT_EQ(w.t.read_inh(w.pid(1)), -5); // the apply (5.9), which clears the coalescing flag
  EXPECT_EQ(w.cancel(2, t2), ticket_state::cancelled);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)"}));
  EXPECT_EQ(w.inh(1), detail::lock_inf);
  EXPECT_EQ(w.t.stats().relevels, 2u);
  EXPECT_EQ(w.t.stats().relevels_coalesced, 0u);
  EXPECT_EQ(w.resume(2, t2).what, detail::lock_outcome::kind::timeout);
  EXPECT_EQ(w.t.read_inh(w.pid(2)), detail::lock_inf); // no record: INF
}

TEST(LockTable, InheritanceCoalescesUntilTheApply) {
  // 4.3: at most one relevel per record is pending; read_inh clears the flag (G-39)
  world w(0, true, true);
  w.acquire(1, X, {"a"}, "o", 5);
  const std::uint64_t t2 = w.wait(2, X, {"a"}, "o", -5);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)"}));
  w.cancel(2, t2); // not applied yet: coalesced
  EXPECT_TRUE(w.posts().empty());
  EXPECT_EQ(w.t.stats().relevels_coalesced, 1u);
  EXPECT_EQ(w.t.read_inh(w.pid(1)), detail::lock_inf); // the pending one reads the current value
  w.resume(2, t2);
  w.wait(2, X, {"a"}, "o", -5);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)"})); // the flag was cleared
  EXPECT_EQ(w.t.stats().relevels, 2u);
}

TEST(LockTable, InheritanceSetBase) {
  // 4.12: a waiter's base change re-levels the holder it boosts; the holder's own base changes
  // nothing it inherits (PI-4)
  world w(0, true, true);
  w.acquire(1, X, {"a"}, "o", 5);
  w.wait(2, X, {"a"}, "o", 0);
  EXPECT_EQ(w.inh(1), 0);
  w.t.read_inh(w.pid(1));
  w.set_base(2, -3);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)"}));
  EXPECT_EQ(w.inh(1), -3);
  EXPECT_EQ(w.t.rec_of(w.pid(2))->base, -3);
  w.t.read_inh(w.pid(1));
  w.set_base(1, 7);
  EXPECT_TRUE(w.posts().empty());
  EXPECT_EQ(w.inh(1), -3);
  w.set_base(3, -9); // no record: nothing
  EXPECT_TRUE(w.posts().empty());
  EXPECT_FALSE(w.t.rec_of(w.pid(3)).has_value());
}

TEST(LockTable, InheritanceThroughAWaiterEdgeAndAfterTheWakes) {
  // PI-9's shape: W queues behind V's earlier ticket, V behind H's hold; H's release grants V,
  // whose wake comes before the relevels, which are in pid order
  world w(0, true, true);
  const std::uint64_t h = w.acquire(1, X, {"a.b"}, "o", 10);
  const std::uint64_t tv = w.wait(2, X, {"a"}, "o", 10);
  w.t.read_inh(w.pid(1));
  w.wait(3, S, {"a.c"}, "o", -10);
  EXPECT_EQ(w.posts(), (std::vector<std::string>{"relevel(1)", "relevel(2)"}));
  EXPECT_EQ(w.inh(1), -10);
  EXPECT_EQ(w.inh(2), -10);
  w.t.read_inh(w.pid(1));
  w.t.read_inh(w.pid(2));
  w.release(1, h);
  EXPECT_EQ(w.posts(),
            (std::vector<std::string>{"wake(2," + std::to_string(tv) + ")", "relevel(1)"}));
  EXPECT_EQ(w.inh(2), -10); // W now waits on V's hold
  EXPECT_FALSE(w.t.rec_of(w.pid(1)).has_value());
}

TEST(LockTable, InheritanceChainCappedAt8Edges) {
  // anc8 (3.2, 3.3): p1 (-5) boosts p2..p9 through 1..8 edges, not p10 (9); p10 inherits 3 from
  // p2..p9; p1's end withdraws its share down the whole chain at once (PI-8)
  world w(0, true, true);
  for (int i = 10; i >= 1; --i)
    w.acquire(i, X, {"k" + std::to_string(i)}, "o", i == 1 ? -5 : 3);
  for (int i = 9; i >= 1; --i) {
    const std::string k = "k" + std::to_string(i + 1);
    w.wait(i, X, {std::string_view(k)}, "o", i == 1 ? -5 : 3);
  }
  for (int i = 2; i <= 9; ++i)
    EXPECT_EQ(w.inh(i), -5) << "p" << i;
  EXPECT_EQ(w.inh(10), 3);
  EXPECT_EQ(w.inh(1), detail::lock_inf);
  for (int i = 1; i <= 10; ++i)
    w.t.read_inh(w.pid(i));
  w.end(1, true);
  std::vector<std::string> want{"relevel(2)", "relevel(3)", "relevel(4)", "relevel(5)",
                                "relevel(6)", "relevel(7)", "relevel(8)", "relevel(9)"};
  EXPECT_EQ(w.posts(), want); // p10's stays 3
  EXPECT_EQ(w.inh(2), detail::lock_inf);
  for (int i = 3; i <= 10; ++i)
    EXPECT_EQ(w.inh(i), 3) << "p" << i;
}

TEST(LockTable, InheritancePassesThroughARoundRobinProcess) {
  // 3.3: a process on a round-robin home lends nothing but passes on what it inherits, and gets
  // no relevel; when the last priority home detaches every inh goes back to INF, posting nothing
  lock_table::options o = world::options(0, true);
  o.one_home = false;
  lock_table t(o);
  detail::lock_notes n;
  const detail::home_id hp = t.attach_home(true, nullptr, nullptr, n);
  const detail::home_id hr = t.attach_home(false, nullptr, nullptr, n);
  auto req = [](lock_mode m, std::string_view k) { return world::request(m, {k}); };
  auto r1 = req(X, "a");
  ASSERT_TRUE(t.try_acquire({hp, 1}, {"o", 5}, r1, {}));
  auto r2 = req(X, "b");
  ASSERT_TRUE(t.try_acquire({hr, 2}, {"o", 0}, r2, {}));
  auto r3 = req(X, "a");
  EXPECT_EQ(t.block({hr, 2}, {"o", 0}, r3, {}, n).state, ticket_state::queued);
  EXPECT_TRUE(n.empty()); // the round-robin waiter lends nothing
  auto r4 = req(X, "b");
  n.clear();
  EXPECT_EQ(t.block({hp, 3}, {"o", -7}, r4, {}, n).state, ticket_state::queued);
  ASSERT_EQ(n.size(), 1u); // H (hp,1) through V (hr,2), and no relevel to the round-robin home
  EXPECT_EQ(n[0].home, hp);
  EXPECT_EQ(n[0].post.pid, 1);
  EXPECT_EQ(t.rec_of({hp, 1})->inh, -7);
  EXPECT_EQ(t.rec_of({hr, 2})->inh, -7);
  t.check();
  n.clear();
  t.detach_home(hp, n); // ends (hp,1) and (hp,3): V is granted a, and lending becomes false
  ASSERT_EQ(n.size(), 1u);
  EXPECT_EQ(n[0].post.what, detail::lock_post::kind::wake);
  EXPECT_EQ(t.rec_of({hr, 2})->inh, detail::lock_inf);
  t.check();
}

TEST(LockTable, InheritanceOff) {
  // options::inheritance false (the lock.inherit bench's off points): nothing lends
  lock_table::options o = world::options(0, true);
  o.inheritance = false;
  lock_table t(o);
  detail::lock_notes n;
  const detail::home_id h = t.attach_home(true, nullptr, nullptr, n);
  auto r1 = world::request(X, {"a"});
  ASSERT_TRUE(t.try_acquire({h, 1}, {"o", 5}, r1, {}));
  auto r2 = world::request(X, {"a"});
  n.clear();
  EXPECT_EQ(t.block({h, 2}, {"o", -5}, r2, {}, n).state, ticket_state::queued);
  EXPECT_TRUE(n.empty());
  t.set_base({h, 2}, -9, n);
  EXPECT_TRUE(n.empty());
  EXPECT_EQ(t.rec_of({h, 1})->inh, detail::lock_inf);
  EXPECT_EQ(t.stats().relevels, 0u);
  t.check();
}

// -- 4.5, 4.10: the fast path ---------------------------------------------------------------------

TEST(LockTable, UncontendedAcquireRelease) {
  world w;
  auto g = w.try_acquire(1, X, {"a.b"});
  ASSERT_TRUE(g);
  EXPECT_EQ(g->hold, 1u);
  EXPECT_EQ(g->gen, 1u);
  EXPECT_FALSE(g->abandoned);
  EXPECT_EQ(w.t.stats().holds, 1u);
  EXPECT_EQ(w.t.stats().trie_nodes, 3u); // root, a, a.b
  auto s = w.try_acquire(2, S, {"a.c"});
  ASSERT_TRUE(s);
  EXPECT_EQ(s->gen, 1u); // an S grant's generation is the last X one (I15)
  w.release(1, g->hold);
  EXPECT_TRUE(w.notes.empty());
  w.release(2, s->hold);
  EXPECT_TRUE(w.t.empty());
  EXPECT_EQ(w.t.stats().trie_nodes, 1u); // every node freed (2.4)
  EXPECT_FALSE(w.t.rec_of(w.pid(1)).has_value());
  auto g2 = w.try_acquire(1, X, {"a.b"});
  ASSERT_TRUE(g2);
  EXPECT_EQ(g2->hold, 3u);
  EXPECT_EQ(g2->gen, 2u);
  w.release(1, g2->hold);
  const lock_stats st = w.t.stats();
  EXPECT_EQ(st.grants, 3u);
  EXPECT_EQ(st.fast_grants, 3u);
  EXPECT_EQ(st.waits, 0u);
}

TEST(LockTable, HoldKeysInCanonicalOrder) {
  world w;
  const std::uint64_t id = w.acquire(1, X, {"a", "b.c", "b.d"});
  auto keys = w.t.hold_keys(id);
  ASSERT_EQ(keys.size(), 3u);
  EXPECT_EQ(keys[0]->key, "a");
  EXPECT_EQ(keys[1]->key, "b.c");
  EXPECT_EQ(keys[2]->key, "b.d");
  EXPECT_EQ(keys[1]->parent->key, "b");
  EXPECT_EQ(keys[1]->depth, 2u);
  w.release(1, id);
  EXPECT_TRUE(w.t.hold_keys(id).empty());
}

TEST(LockTable, ReadsForChecks) {
  world w;
  const std::uint64_t a = w.acquire(1, X, {"a"});
  const std::uint64_t b = w.acquire(1, S, {"b"});
  const std::uint64_t tid = w.block(2, X, {"a"}, "q", 7).tid;
  auto r1 = w.t.rec_of(w.pid(1));
  ASSERT_TRUE(r1);
  EXPECT_EQ(r1->owner, "o");
  EXPECT_EQ(r1->holds, (std::vector<std::uint64_t>{a, b}));
  EXPECT_FALSE(r1->ticket.has_value());
  EXPECT_EQ(r1->inh, detail::lock_inf);
  auto r2 = w.t.rec_of(w.pid(2));
  ASSERT_TRUE(r2);
  EXPECT_EQ(r2->owner, "q");
  EXPECT_EQ(r2->base, 7);
  EXPECT_TRUE(r2->holds.empty());
  EXPECT_EQ(r2->ticket, tid);
  auto tv = w.t.ticket_of(tid);
  ASSERT_TRUE(tv);
  EXPECT_EQ(tv->pid, w.pid(2));
  EXPECT_EQ(tv->state, ticket_state::queued);
  EXPECT_FALSE(tv->hold.has_value());
  auto hv = w.t.hold_of(a);
  ASSERT_TRUE(hv);
  EXPECT_EQ(hv->pid, w.pid(1));
  EXPECT_EQ(hv->mode, X);
  EXPECT_EQ(hv->gen, 1u);
  EXPECT_TRUE(hv->body_started);
  EXPECT_EQ(w.t.hold_of(b)->mode, S);
  w.release(1, a);
  EXPECT_EQ(w.wakes(), (wake_list{{2, tid}}));
  tv = w.t.ticket_of(tid);
  ASSERT_TRUE(tv);
  EXPECT_EQ(tv->state, ticket_state::granted);
  ASSERT_TRUE(tv->hold.has_value());
  EXPECT_FALSE(w.t.hold_of(*tv->hold)->body_started); // granted, not started
  EXPECT_EQ(w.t.rec_of(w.pid(2))->holds, (std::vector<std::uint64_t>{*tv->hold}));
  EXPECT_FALSE(w.t.hold_of(a).has_value());
  const std::uint64_t h2 = w.enter(2, tid);
  EXPECT_FALSE(w.t.ticket_of(tid).has_value());
  EXPECT_TRUE(w.t.hold_of(h2)->body_started);
  EXPECT_EQ(w.t.read_inh(w.pid(2)), detail::lock_inf);
  EXPECT_EQ(w.t.read_inh(w.pid(9)), detail::lock_inf);
  w.notes.clear();
  w.t.set_base(w.pid(2), -3, w.notes);
  EXPECT_TRUE(w.notes.empty()); // phase L1: nothing to relevel
  EXPECT_EQ(w.t.rec_of(w.pid(2))->base, -3);
  w.t.check();
  // home_cache_refs
  auto ks = refs_of({"c", "c", "d"});
  w.t.ref_nodes(w.h, ks);
  w.t.check();
  EXPECT_EQ(ks[0].node, ks[1].node);
  EXPECT_EQ(w.t.home_cache_refs(w.h), 3u);
  EXPECT_EQ(w.t.home_cache_refs(w.h, ks[0].node), 2u);
  EXPECT_EQ(w.t.home_cache_refs(w.h, ks[2].node), 1u);
  EXPECT_EQ(w.t.home_cache_refs(w.h, w.t.hold_keys(h2)[0]), 0u);
  EXPECT_EQ(w.t.home_cache_refs(w.h + 1), 0u);
}

// -- 4.6, 4.7, 4.8 --------------------------------------------------------------------------------

TEST(LockTable, BlockGrantsAtOnceWhenGrantable) {
  world w;
  // The state changed between a busy try_acquire and block: block grants (protocol 4.6).
  auto b = w.block(1, X, {"z"});
  EXPECT_EQ(b.tid, 1u);
  EXPECT_EQ(b.state, ticket_state::granted);
  EXPECT_TRUE(w.notes.empty());
  auto o = w.resume(1, b.tid);
  ASSERT_EQ(o.what, detail::lock_outcome::kind::granted);
  EXPECT_EQ(o.g.hold, 1u);
  EXPECT_EQ(o.g.gen, 1u);
  EXPECT_FALSE(w.t.hold_of(o.g.hold)->body_started);
  w.t.start_body(w.pid(1), o.g.hold);
  w.t.check();
  EXPECT_TRUE(w.t.hold_of(o.g.hold)->body_started);
  w.release(1, o.g.hold);
  EXPECT_TRUE(w.t.empty());
  EXPECT_EQ(w.t.stats().grants, 1u);
  EXPECT_EQ(w.t.stats().fast_grants, 0u);
}

TEST(LockTable, OwnerChecks) {
  world w;
  const std::uint64_t a = w.acquire(1, X, {"a"});
  const std::uint64_t tid = w.wait(2, X, {"a"});
  w.release(3, a); // not 3's: nothing (I12)
  EXPECT_TRUE(w.t.hold_of(a).has_value());
  EXPECT_FALSE(w.cancel(3, tid).has_value());
  EXPECT_FALSE(w.cancel(2, 99).has_value());
  EXPECT_EQ(w.state(tid), ticket_state::queued);
  EXPECT_THROW(w.t.start_body(w.pid(3), a), detail::lock_violation);
  EXPECT_THROW(w.t.resume(w.pid(3), tid), detail::lock_violation);
  // A process with a ticket makes no other request (I14).
  auto r = world::request(X, {"b"});
  detail::lock_notes n;
  EXPECT_THROW(w.t.try_acquire(w.pid(2), {"o", 0}, r, {}), detail::lock_violation);
  EXPECT_THROW(w.t.block(w.pid(2), {"o", 0}, r, {}, n), detail::lock_violation);
  w.t.check();
  // resume of a queued ticket changes nothing
  EXPECT_EQ(w.resume(2, tid).what, detail::lock_outcome::kind::queued);
  EXPECT_EQ(w.state(tid), ticket_state::queued);
}

TEST(LockTable, SelfCheckRefusesNonCanonicalRequests) {
  world w;
  auto bad = [&](lock_mode m, keys_t keys) {
    auto r = world::request(m, keys);
    try {
      w.t.try_acquire(w.pid(1), {"o", 0}, r, {});
    } catch (const detail::lock_violation &v) {
      return v.invariant();
    }
    return std::string();
  };
  EXPECT_EQ(bad(X, {"b", "a"}), "1");
  EXPECT_EQ(bad(X, {"a", "a.b"}), "1");
  EXPECT_EQ(bad(X, {"a", "a"}), "1");
  EXPECT_EQ(bad(X, {"a#b"}), "1");
  EXPECT_EQ(bad(X, {"a..b"}), "1");
  EXPECT_EQ(bad(X, {""}), "1");
  EXPECT_EQ(bad(X, {}), "1");
  EXPECT_EQ(bad(X, {"a.b", "a-c"}), ""); // '.' orders before every other character
  EXPECT_FALSE(w.t.empty());
}

// -- LK: grant rule, FIFO, conversion
// ---------------------------------------------------------------

TEST(LockTable, LK08_ConflictOnSegments) {
  world w;
  w.acquire(1, X, {"a.b"});
  EXPECT_FALSE(w.try_acquire(2, S, {"a"}));
  EXPECT_FALSE(w.try_acquire(3, X, {"a"}));
  EXPECT_FALSE(w.try_acquire(4, S, {"a.b.c"}));
  EXPECT_TRUE(w.try_acquire(5, X, {"a.c"}));
  EXPECT_TRUE(w.try_acquire(6, X, {"a.bc"}));
  // A queued X on a against a later X a.d, which no hold blocks: it waits behind it (FIFO).
  w.wait(3, X, {"a"});
  EXPECT_FALSE(w.try_acquire(7, X, {"a.d"}));
  EXPECT_EQ(w.block(7, X, {"a.d"}).state, ticket_state::queued);
  EXPECT_EQ(w.t.stats().queued, 2u);
  EXPECT_TRUE(w.try_acquire(8, X, {"b"})); // no overlap: passes freely
}

TEST(LockTable, LK04_WriterBlocksLaterReaders) {
  world w;
  w.acquire(1, S, {"k"});
  w.acquire(2, S, {"k"});
  const std::uint64_t tw = w.wait(3, X, {"k"});
  EXPECT_FALSE(w.try_acquire(4, S, {"k"})); // never barges past the queued writer
  const std::uint64_t tr = w.block(4, S, {"k"}).tid;
  w.end(1, false);
  EXPECT_TRUE(w.notes.empty());
  w.end(2, false);
  EXPECT_EQ(w.wakes(), (wake_list{{3, tw}}));
  EXPECT_EQ(w.state(tr), ticket_state::queued);
}

TEST(LockTable, LK03_WritersInArrivalOrder) {
  world w;
  std::uint64_t hold = w.acquire(0, X, {"k"});
  std::vector<std::uint64_t> tids;
  for (int p = 1; p <= 5; ++p)
    tids.push_back(w.wait(p, X, {"k"}));
  for (int p = 0; p < 5; ++p) {
    w.release(p, hold);
    ASSERT_EQ(w.wakes(), (wake_list{{p + 1, tids[std::size_t(p)]}}));
    hold = w.enter(p + 1, tids[std::size_t(p)]);
  }
  w.release(5, hold);
  EXPECT_TRUE(w.t.empty());
}

TEST(LockTable, LK19_MultiKeyWaitsHoldingNothing) {
  world w;
  const std::uint64_t qh = w.acquire(1, X, {"a.y"});     // Q
  const std::uint64_t tp = w.wait(2, X, {"a.x", "a.y"}); // P
  auto q = w.t.query("", w.h, "o");
  ASSERT_EQ(q.held.size(), 1u); // while P waits, a.x is not held
  EXPECT_EQ(q.held[0].key, "a.y");
  const std::uint64_t tr = w.wait(3, X, {"a"}); // R
  EXPECT_EQ(w.t.stats().deadlocks, 0u);
  w.release(1, qh);
  EXPECT_EQ(w.wakes(), (wake_list{{2, tp}})); // P, not R
  EXPECT_EQ(w.state(tr), ticket_state::queued);
  const std::uint64_t ph = w.enter(2, tp);
  auto keys = w.t.hold_keys(ph);
  ASSERT_EQ(keys.size(), 2u); // both keys in one grant
  EXPECT_EQ(keys[0]->key, "a.x");
  EXPECT_EQ(keys[1]->key, "a.y");
  w.release(2, ph);
  EXPECT_EQ(w.wakes(), (wake_list{{3, tr}}));
}

TEST(LockTable, LK20_NestedSiblingPassesQueuedParentWriter) {
  world w;
  w.acquire(1, X, {"a.x"});     // P inside (with-lock "a.x" ...)
  w.wait(2, X, {"a"});          // an X on a queued
  for (int i = 0; i < 3; ++i) { // plain, :if-available, :timeout 0: each a try_acquire
    auto g = w.try_acquire(1, X, {"a.y"});
    ASSERT_TRUE(g) << i;
    w.release(1, g->hold);
    EXPECT_TRUE(w.notes.empty());
  }
  EXPECT_EQ(w.t.stats().deadlocks, 0u);
}

TEST(LockTable, LK21_GrantAllInOrder) {
  world w;
  const std::uint64_t x = w.acquire(1, X, {"a"});
  const std::uint64_t t1 = w.wait(2, S, {"a"});
  const std::uint64_t t2 = w.wait(3, S, {"a"});
  const std::uint64_t t3 = w.wait(4, X, {"a"});
  const std::uint64_t t4 = w.wait(5, S, {"a"});
  w.release(1, x);
  EXPECT_EQ(w.wakes(), (wake_list{{2, t1}, {3, t2}})); // the two first S together, in seq order
  EXPECT_EQ(w.state(t3), ticket_state::queued);
  EXPECT_EQ(w.state(t4), ticket_state::queued);
  const std::uint64_t h1 = w.enter(2, t1);
  const std::uint64_t h2 = w.enter(3, t2);
  w.release(2, h1);
  EXPECT_TRUE(w.notes.empty());
  w.release(3, h2);
  EXPECT_EQ(w.wakes(), (wake_list{{4, t3}})); // the X after both
  const std::uint64_t h3 = w.enter(4, t3);
  EXPECT_EQ(w.state(t4), ticket_state::queued);
  w.release(4, h3);
  EXPECT_EQ(w.wakes(), (wake_list{{5, t4}})); // the last S after the X
}

TEST(LockTable, LK22_WriterTimeoutLetsReadersIn) {
  world w;
  w.acquire(1, S, {"a.b"});                       // H
  const std::uint64_t tw = w.wait(2, X, {"a.b"}); // W
  const std::uint64_t tr = w.wait(3, S, {"a"});   // R, behind W
  EXPECT_EQ(w.cancel(2, tw), ticket_state::cancelled);
  EXPECT_EQ(w.wakes(), (wake_list{{3, tr}})); // R granted at once
  EXPECT_EQ(w.state(tw), ticket_state::cancelled);
  EXPECT_EQ(w.resume(2, tw).what, detail::lock_outcome::kind::timeout);
  EXPECT_FALSE(w.t.rec_of(w.pid(2)).has_value()); // W held nothing: its record went
  // A cancel of a granted ticket returns its state: the waiter adopts the grant (4.9).
  EXPECT_EQ(w.cancel(3, tr), ticket_state::granted);
  w.enter(3, tr);
  EXPECT_EQ(w.t.stats().timeouts, 1u);
}

TEST(LockTable, LK25_SharedWaiterBehindTimedOutWriter) {
  world w;
  w.acquire(1, S, {"k"});
  const std::uint64_t tw = w.wait(2, X, {"k"});
  const std::uint64_t tr = w.wait(3, S, {"k"});
  w.cancel(2, tw);
  EXPECT_EQ(w.wakes(), (wake_list{{3, tr}}));
}

TEST(LockTable, LK26_BroadenAfterOtherReleases) {
  world w;
  w.acquire(1, X, {"a.b"});                          // P
  const std::uint64_t rh = w.acquire(2, S, {"a.c"}); // R
  const std::uint64_t tp = w.wait(1, X, {"a"});
  w.release(2, rh);
  EXPECT_EQ(w.wakes(), (wake_list{{1, tp}}));
  w.enter(1, tp);
}

TEST(LockTable, LK27_ConversionRuleNested) {
  world w;
  const std::uint64_t p1 = w.acquire(1, S, {"a.x"}); // P
  const std::uint64_t tq = w.wait(2, X, {"a"});      // Q
  auto g = w.try_acquire(1, X, {"a.y"});
  ASSERT_TRUE(g); // granted at once
  w.release(1, g->hold);
  EXPECT_TRUE(w.notes.empty());
  w.release(1, p1);
  EXPECT_EQ(w.wakes(), (wake_list{{2, tq}})); // Q after P releases both
}

TEST(LockTable, LK28_ConversionRuleBroaden) {
  world w;
  const std::uint64_t p1 = w.acquire(1, X, {"a.b"}); // P
  const std::uint64_t tq = w.wait(2, X, {"a"});      // Q
  auto g = w.try_acquire(1, X, {"a"});
  ASSERT_TRUE(g);
  EXPECT_EQ(g->gen, 2u);
  w.release(1, g->hold);
  EXPECT_TRUE(w.notes.empty());
  w.release(1, p1);
  EXPECT_EQ(w.wakes(), (wake_list{{2, tq}}));
}

TEST(LockTable, LK32_NonConflictingPassFreely) {
  world w;
  w.acquire(1, X, {"a"});                         // H
  const std::uint64_t tw = w.wait(2, S, {"a.b"}); // W
  EXPECT_TRUE(w.try_acquire(3, X, {"b"}));        // R
  EXPECT_TRUE(w.try_acquire(3, X, {"ab"}));
  EXPECT_EQ(w.state(tw), ticket_state::queued);
}

// -- LD: deadlock ---------------------------------------------------------------------------------

TEST(LockTable, LD01_TwoInOppositeOrder) {
  world w;
  w.acquire(1, X, {"a"});
  const std::uint64_t b2 = w.acquire(2, X, {"b"});
  const std::uint64_t t1 = w.wait(1, X, {"b"});
  auto b = w.block(2, X, {"a"});
  EXPECT_EQ(b.state, ticket_state::failed); // the closer is the victim
  EXPECT_TRUE(w.notes.empty());
  EXPECT_EQ(w.t.stats().deadlocks, 1u);
  EXPECT_EQ(w.t.stats().queued, 1u);
  auto o = w.resume(2, b.tid);
  ASSERT_EQ(o.what, detail::lock_outcome::kind::deadlock);
  using E = std::vector<std::pair<std::optional<int>, std::string>>;
  EXPECT_EQ(edges(o.cycle), (E{{2, "a"}, {1, "b"}}));
  w.release(2, b2); // the victim's finally
  EXPECT_EQ(w.wakes(), (wake_list{{1, t1}}));
}

TEST(LockTable, LD02_CycleOfEightAndThroughShared) {
  using E = std::vector<std::pair<std::optional<int>, std::string>>;
  for (bool other_owner : {false, true}) {
    world w;
    const std::string keys[] = {"k1", "k2", "k3", "k4", "k5", "k6", "k7", "k8"};
    auto owner = [&](int p) { return other_owner && p == 3 ? "z" : "o"; };
    for (int p = 1; p <= 8; ++p)
      w.acquire(p, X, {keys[p - 1]}, owner(p));
    for (int p = 1; p <= 7; ++p)
      w.wait(p, X, {keys[p]}, owner(p));
    auto b = w.block(8, X, {"k1"});
    ASSERT_EQ(b.state, ticket_state::failed);
    auto o = w.resume(8, b.tid);
    E want{{8, "k1"}};
    for (int p = 1; p <= 7; ++p)
      want.emplace_back(other_owner && p == 3 ? std::nullopt : std::optional<int>(p), keys[p]);
    EXPECT_EQ(edges(o.cycle), want) << other_owner;
  }
  // Through an S lock with several holders.
  world w;
  w.acquire(1, S, {"a"});
  w.acquire(2, S, {"a"});
  w.acquire(3, X, {"b"});
  w.wait(1, X, {"b"});
  auto b = w.block(3, X, {"a"});
  ASSERT_EQ(b.state, ticket_state::failed);
  EXPECT_EQ(edges(w.resume(3, b.tid).cycle), (E{{3, "a"}, {1, "b"}}));
}

TEST(LockTable, LD03_TwoBroadeners) {
  world w;
  const std::uint64_t ph = w.acquire(1, S, {"a.b"});
  const std::uint64_t qh = w.acquire(2, S, {"a.c"});
  const std::uint64_t tp = w.wait(1, X, {"a"});
  auto b = w.block(2, X, {"a"});
  ASSERT_EQ(b.state, ticket_state::failed);
  EXPECT_EQ(w.resume(2, b.tid).what, detail::lock_outcome::kind::deadlock);
  w.release(2, qh); // the victim unwinds
  EXPECT_EQ(w.wakes(), (wake_list{{1, tp}}));
  w.release(1, w.enter(1, tp));
  w.release(1, ph);
  EXPECT_TRUE(w.t.empty());
}

TEST(LockTable, LD04_LongChainNoCycle) {
  world w;
  std::vector<std::string> keys;
  for (int i = 0; i <= 32; ++i)
    keys.push_back("c" + std::to_string(i));
  for (int p = 0; p <= 32; ++p)
    w.acquire(p, X, {keys[std::size_t(p)]});
  for (int p = 0; p < 32; ++p)
    EXPECT_EQ(w.block(p, X, {keys[std::size_t(p) + 1]}).state, ticket_state::queued) << p;
  EXPECT_EQ(w.t.stats().deadlocks, 0u);
  EXPECT_EQ(w.t.stats().queued, 32u);
  for (int p = 32; p >= 0; --p)
    w.end(p, false);
  EXPECT_TRUE(w.t.empty());
}

TEST(LockTable, LD07_ConversionRuleSharedParent) {
  world w;
  const std::uint64_t q1 = w.acquire(1, S, {"a.b.c"}); // Q
  const std::uint64_t tp = w.wait(2, X, {"a.b.c"});    // P
  auto g = w.try_acquire(1, S, {"a"});
  ASSERT_TRUE(g); // the conversion rule
  w.release(1, g->hold);
  EXPECT_TRUE(w.notes.empty());
  w.release(1, q1);
  EXPECT_EQ(w.wakes(), (wake_list{{2, tp}}));
}

TEST(LockTable, LD09_CycleThroughWaiterEdge) {
  world w;
  w.acquire(1, X, {"a.b"}); // H
  w.wait(2, S, {"a"});      // V
  w.acquire(3, X, {"z"});   // W
  w.wait(3, X, {"a.c"});    // W waits behind V
  auto b = w.block(1, X, {"z"});
  ASSERT_EQ(b.state, ticket_state::failed);
  using E = std::vector<std::pair<std::optional<int>, std::string>>;
  EXPECT_EQ(edges(w.resume(1, b.tid).cycle), (E{{1, "z"}, {3, "a.c"}, {2, "a"}}));
}

TEST(LockTable, LD10_PairwiseConversionLimit) {
  world w;
  w.acquire(1, S, {"a.x"}); // P
  w.wait(2, X, {"a"});      // R
  w.wait(3, S, {"a.q"});    // Q
  auto b = w.block(1, X, {"a.q.w"});
  ASSERT_EQ(b.state, ticket_state::failed);
  using E = std::vector<std::pair<std::optional<int>, std::string>>;
  EXPECT_EQ(edges(w.resume(1, b.tid).cycle), (E{{1, "a.q.w"}, {3, "a.q"}, {2, "a"}}));
}

// -- 4.11: ends and detaches
// ------------------------------------------------------------------------

TEST(LockTable, EveryEndEmptiesTheTable) {
  world w;
  w.acquire(1, X, {"a.b"});
  w.acquire(1, S, {"c", "d.e"});
  w.acquire(2, S, {"c"});
  const std::uint64_t t3 = w.wait(3, X, {"a"});
  const std::uint64_t t4 = w.wait(4, X, {"c", "f"});
  const std::uint64_t t5 = w.wait(5, X, {"d"});
  w.end(1, false);
  EXPECT_EQ(w.wakes(), (wake_list{{3, t3}, {5, t5}})); // X c,f still waits for 2's S c
  w.end(2, false);
  EXPECT_EQ(w.wakes(), (wake_list{{4, t4}}));
  w.end(3, true); // granted, never started: released clean
  w.end(4, false);
  w.end(5, false);
  w.end(6, false); // no record: nothing
  EXPECT_TRUE(w.t.empty());
  EXPECT_EQ(w.t.stats().trie_nodes, 1u);
  EXPECT_EQ(w.t.stats().ends, 5u);
  EXPECT_NE(w.t.canonical().find("recs\nholds\ntickets\nqueue=[]\nabandoned=[]\n"),
            std::string::npos);
}

TEST(LockTable, DetachHomeEndsEveryProcess) {
  world w;
  w.acquire(1, X, {"a"}); // started X: abandoned by the detach
  w.acquire(2, S, {"b"}); // S: clean
  w.wait(3, X, {"a"});    // queued
  const std::uint64_t c = w.acquire(5, X, {"c"});
  const std::uint64_t t4 = w.wait(4, X, {"c"});
  w.release(5, c);
  EXPECT_EQ(w.state(t4), ticket_state::granted); // granted, not resumed: clean
  auto ks = refs_of({"r"});
  w.t.ref_nodes(w.h, ks);
  w.notes.clear();
  w.t.detach_home(w.h, w.notes);
  w.t.check();
  EXPECT_TRUE(w.notes.empty());
  EXPECT_TRUE(w.t.empty());
  EXPECT_EQ(w.t.stats().cache_refs, 0u);
  EXPECT_EQ(w.t.stats().trie_nodes, 2u); // the root and a's flag
  EXPECT_NE(w.t.canonical().find("abandoned=[a]\n"), std::string::npos);
  EXPECT_EQ(w.t.stats().ends, 4u);
  // A new home's first X grant on a reports it.
  detail::lock_notes n;
  const detail::home_id h2 = w.t.attach_home(false, nullptr, nullptr, n);
  auto r = world::request(X, {"a"});
  auto g = w.t.try_acquire({h2, 1}, {"o", 0}, r, {});
  ASSERT_TRUE(g);
  EXPECT_TRUE(g->abandoned);
  w.t.check();
}

// -- I11: abandoned flags
// ---------------------------------------------------------------------------

TEST(LockTable, AbandonedReportedOnceToTheNextXGrant) {
  world w;
  w.acquire(1, X, {"a.b"});
  w.end(1, true); // killed inside its body
  EXPECT_EQ(w.t.stats().abandoned, 1u);
  EXPECT_EQ(w.t.stats().trie_nodes, 3u); // root, a, a.b's flag
  auto parent = w.try_acquire(2, X, {"a"});
  ASSERT_TRUE(parent);
  EXPECT_FALSE(parent->abandoned); // an X on the parent does not take it
  w.release(2, parent->hold);
  auto s = w.try_acquire(3, S, {"a.b"});
  ASSERT_TRUE(s);
  EXPECT_FALSE(s->abandoned); // nor does an S grant
  auto q = w.t.query("a", w.h, "o");
  ASSERT_EQ(q.held.size(), 1u);
  EXPECT_TRUE(q.held[0].abandoned);
  w.release(3, s->hold);
  auto x = w.try_acquire(4, X, {"a.b"});
  ASSERT_TRUE(x);
  EXPECT_TRUE(x->abandoned); // once
  w.release(4, x->hold);
  auto x2 = w.try_acquire(4, X, {"a.b"});
  ASSERT_TRUE(x2);
  EXPECT_FALSE(x2->abandoned);
  w.release(4, x2->hold);
  EXPECT_EQ(w.t.stats().trie_nodes, 1u);
  // An S holder killed, and an X holder ended without abandon, leave no flag.
  w.acquire(5, S, {"c"});
  w.end(5, true);
  w.acquire(6, X, {"d"});
  w.end(6, false);
  EXPECT_NE(w.t.canonical().find("abandoned=[]\n"), std::string::npos);
  EXPECT_EQ(w.t.stats().abandoned, 1u);
}

TEST(LockTable, AbandonedFlagGoesBackFromAnUnstartedGrant) {
  world w;
  w.acquire(1, X, {"d"});
  const std::uint64_t t2 = w.wait(2, X, {"d"});
  w.end(1, true); // the flag is set, then taken by 2's grant
  EXPECT_EQ(w.wakes(), (wake_list{{2, t2}}));
  EXPECT_NE(w.t.canonical().find("abandoned=[]\n"), std::string::npos);
  EXPECT_NE(w.t.canonical().find("taken=[d]"), std::string::npos);
  w.end(2, true); // killed before its body started: released clean, the flag goes back
  EXPECT_NE(w.t.canonical().find("abandoned=[d]\n"), std::string::npos);
  EXPECT_EQ(w.t.stats().abandoned, 1u);
  // The same grant collected and started reports it.
  w.acquire(3, X, {"e"});
  const std::uint64_t t4 = w.wait(4, X, {"d", "e"});
  w.end(3, false);
  EXPECT_EQ(w.wakes(), (wake_list{{4, t4}}));
  auto o = w.resume(4, t4);
  ASSERT_EQ(o.what, detail::lock_outcome::kind::granted);
  EXPECT_TRUE(o.g.abandoned);
  w.t.start_body(w.pid(4), o.g.hold);
  w.t.check();
  w.release(4, o.g.hold);
  auto x = w.try_acquire(5, X, {"d"});
  ASSERT_TRUE(x);
  EXPECT_FALSE(x->abandoned);
}

// -- 4.15: key-cache references
// ---------------------------------------------------------------------

TEST(LockTable, CacheReferences) {
  world w;
  auto ks = refs_of({"a.b", "c"});
  w.t.ref_nodes(w.h, ks);
  w.t.check();
  ASSERT_TRUE(ks[0].node && ks[1].node);
  EXPECT_EQ(ks[0].node->key, "a.b");
  EXPECT_EQ(ks[0].node->refs, 1u);
  EXPECT_EQ(w.t.stats().cache_refs, 2u);
  EXPECT_EQ(w.t.stats().trie_nodes, 4u); // root, a, a.b, c
  {
    // A hit: the request is its node; the hold keeps nothing new alive.
    detail::lock_request r;
    r.mode = X;
    r.keys.push_back(detail::key_ref{ks[0].node, {}, false});
    auto g = w.t.try_acquire(w.pid(1), {"o", 0}, r, {});
    w.t.check();
    ASSERT_TRUE(g);
    EXPECT_EQ(w.t.hold_keys(g->hold)[0], ks[0].node);
    w.release(1, g->hold);
    EXPECT_EQ(w.t.stats().trie_nodes, 4u); // kept by the reference
  }
  // A miss the cache keeps: ref on the request's key.
  detail::lock_request r;
  r.mode = X;
  r.keys.push_back(detail::key_ref{nullptr, "d", true});
  auto g = w.t.try_acquire(w.pid(1), {"o", 0}, r, {});
  w.t.check();
  ASSERT_TRUE(g);
  ASSERT_TRUE(r.keys[0].node);
  EXPECT_EQ(r.keys[0].node->refs, 1u);
  EXPECT_EQ(w.t.stats().cache_refs, 3u);
  // A busy request changes only references (I13).
  const std::string before = w.t.canonical();
  detail::lock_request busy;
  busy.mode = S;
  busy.keys.push_back(detail::key_ref{nullptr, "d", true});
  busy.keys.push_back(detail::key_ref{nullptr, "e", false});
  EXPECT_FALSE(w.t.try_acquire(w.pid(2), {"o", 0}, busy, {}));
  w.t.check();
  EXPECT_EQ(busy.keys[0].node, r.keys[0].node);
  EXPECT_EQ(busy.keys[0].node->refs, 2u);
  EXPECT_EQ(w.t.stats().cache_refs, 4u);
  EXPECT_EQ(w.t.stats().trie_nodes, 5u); // no node for the busy, unreferenced e
  const std::string after = w.t.canonical();
  EXPECT_EQ(before.substr(0, before.find("cache_refs")), after.substr(0, after.find("cache_refs")));
  EXPECT_NE(after.find("H0 {a.b:1,c:1,d:2}"), std::string::npos);
  // An eviction riding a request: c's last reference goes, and its node with it.
  detail::cache_delta delta;
  delta.drop.push_back(ks[1].node);
  detail::lock_request f = world::request(X, {"f"});
  ASSERT_TRUE(w.t.try_acquire(w.pid(3), {"o", 0}, f, delta));
  w.t.check();
  EXPECT_EQ(w.t.stats().cache_refs, 3u);
  EXPECT_EQ(w.t.query("c", w.h, "o").held.size(), 0u);
  EXPECT_EQ(w.t.canonical().find("c:1"), std::string::npos);
  // unref_nodes frees a.b, and a with it.
  const std::size_t nodes = w.t.stats().trie_nodes;
  detail::lock_node *const drop[] = {ks[0].node};
  w.t.unref_nodes(w.h, drop);
  w.t.check();
  EXPECT_EQ(w.t.stats().trie_nodes, nodes - 2);
  EXPECT_THROW(w.t.unref_nodes(w.h, drop), detail::lock_violation); // no reference left
  // detach_home drops the rest; every node goes with its last count.
  w.release(1, g->hold);
  w.end(3, false);
  w.notes.clear();
  w.t.detach_home(w.h, w.notes);
  w.t.check();
  EXPECT_EQ(w.t.stats().cache_refs, 0u);
  EXPECT_EQ(w.t.stats().trie_nodes, 1u);
  EXPECT_EQ(w.t.home_cache_refs(w.h), 0u);
  EXPECT_TRUE(w.t.empty());
  auto late = refs_of({"g"});
  EXPECT_THROW(w.t.ref_nodes(w.h, late), detail::lock_violation); // H is detached
}

// -- 4.13
// -------------------------------------------------------------------------------------------

TEST(LockTable, Query) {
  world w;
  w.acquire(1, X, {"a.b"}, "o");
  w.acquire(2, S, {"c"}, "q");
  w.acquire(3, S, {"c"}, "o");
  w.wait(4, X, {"c"}, "o");
  w.wait(5, S, {"a"}, "q");
  w.wait(6, X, {"b", "c.d"}, "o");
  auto q = w.t.query("", w.h, "o");
  ASSERT_EQ(q.held.size(), 2u);
  EXPECT_EQ(q.held[0].key, "a.b");
  EXPECT_EQ(q.held[0].mode, X);
  EXPECT_EQ(q.held[0].pids, (std::vector<int>{1}));
  EXPECT_EQ(q.held[0].count, 1u);
  EXPECT_EQ(q.held[0].generation, 1u);
  EXPECT_FALSE(q.held[0].abandoned);
  EXPECT_EQ(q.held[1].key, "c");
  EXPECT_EQ(q.held[1].mode, S);
  EXPECT_EQ(q.held[1].pids, (std::vector<int>{3})); // 2 is another owner's: counted, not shown
  EXPECT_EQ(q.held[1].count, 2u);
  EXPECT_FALSE(q.held[1].generation.has_value());
  ASSERT_EQ(q.waiting.size(), 3u); // in seq order
  EXPECT_EQ(q.waiting[0].keys, (std::vector<std::string>{"c"}));
  EXPECT_EQ(q.waiting[0].mode, X);
  EXPECT_EQ(q.waiting[0].pid, 4);
  EXPECT_EQ(q.waiting[1].keys, (std::vector<std::string>{"a"}));
  EXPECT_FALSE(q.waiting[1].pid.has_value());
  EXPECT_EQ(q.waiting[2].keys, (std::vector<std::string>{"b", "c.d"}));
  EXPECT_EQ(q.waiting[2].pid, 6);
  // Under a prefix: keys under it only.
  auto qa = w.t.query("a", w.h, "o");
  ASSERT_EQ(qa.held.size(), 1u);
  EXPECT_EQ(qa.held[0].key, "a.b");
  ASSERT_EQ(qa.waiting.size(), 1u);
  EXPECT_EQ(qa.waiting[0].keys[0], "a");
  auto qab = w.t.query("a.b", w.h, "o");
  EXPECT_EQ(qab.held.size(), 1u);
  EXPECT_TRUE(qab.waiting.empty()); // a is not under a.b
  auto qc = w.t.query("c.d", w.h, "o");
  EXPECT_TRUE(qc.held.empty());
  ASSERT_EQ(qc.waiting.size(), 1u);
  EXPECT_EQ(qc.waiting[0].pid, 6);
  EXPECT_TRUE(w.t.query("zz", w.h, "o").held.empty());
  // Another owner, another home, no viewer: no pid.
  auto qq = w.t.query("", w.h, "q");
  EXPECT_EQ(qq.held[1].pids, (std::vector<int>{2}));
  EXPECT_EQ(qq.waiting[1].pid, 5);
  auto other = w.t.query("", w.h + 1, "o");
  EXPECT_TRUE(other.held[0].pids.empty());
  EXPECT_FALSE(other.waiting[0].pid.has_value());
  auto none = w.t.query("", std::nullopt, "o");
  EXPECT_TRUE(none.held[1].pids.empty());
  EXPECT_EQ(none.held[1].count, 2u);
}

TEST(LockTable, QueryGenerationIsTheLatestXGrant) {
  for (bool first : {false, true}) {
    world w(first ? lock_table::query_first_gen : 0);
    w.acquire(1, X, {"a.b"});      // gen 1
    w.acquire(1, X, {"a.b", "b"}); // nested: its own hold never blocks it; gen 2
    auto q = w.t.query("a.b", w.h, "o");
    ASSERT_EQ(q.held.size(), 1u);
    EXPECT_EQ(q.held[0].count, 2u);
    EXPECT_EQ(q.held[0].pids, (std::vector<int>{1, 1}));
    EXPECT_EQ(q.held[0].generation, first ? 1u : 2u);
  }
}

// -- canonical() ----------------------------------------------------------------------------------

TEST(LockTable, CanonicalOfAKnownState) {
  world w;
  w.acquire(1, X, {"a.b"}, "o");
  w.block(2, S, {"a"}, "q", 3);
  auto ks = refs_of({"c"});
  w.t.ref_nodes(w.h, ks);
  EXPECT_EQ(w.t.canonical(), "homes\n"
                             "H0 prio=0\n"
                             "recs\n"
                             "(0,1) owner=\"o\" base=0 holds=[1] ticket=- inh=INF\n"
                             "(0,2) owner=\"q\" base=3 holds=[] ticket=1 inh=INF\n"
                             "holds\n"
                             "1 pid=(0,1) keys=[a.b] mode=X gen=1 started=1 taken=[]\n"
                             "tickets\n"
                             "1 pid=(0,2) keys=[a] mode=S state=queued hold=- error=-\n"
                             "queue=[1]\n"
                             "abandoned=[]\n"
                             "next seq=1 hold=1 gen=1\n"
                             "cache_refs\n"
                             "H0 {c:1}\n"
                             "counters grants=1\n");
  // A failed ticket, a granted one, an abandoned flag.
  world v;
  v.acquire(1, X, {"a"});
  v.acquire(2, X, {"b"});
  v.wait(1, X, {"b"});
  v.block(2, X, {"a"}); // tid 2: DeadlockError
  v.acquire(3, X, {"k"});
  v.end(3, true); // k flagged
  v.acquire(4, S, {"m.n", "x"});
  v.block(5, X, {"k"}); // tid 3: granted at once, taking k's flag
  EXPECT_EQ(v.t.canonical(), "homes\n"
                             "H0 prio=0\n"
                             "recs\n"
                             "(0,1) owner=\"o\" base=0 holds=[1] ticket=1 inh=INF\n"
                             "(0,2) owner=\"o\" base=0 holds=[2] ticket=2 inh=INF\n"
                             "(0,4) owner=\"o\" base=0 holds=[4] ticket=- inh=INF\n"
                             "(0,5) owner=\"o\" base=0 holds=[5] ticket=3 inh=INF\n"
                             "holds\n"
                             "1 pid=(0,1) keys=[a] mode=X gen=1 started=1 taken=[]\n"
                             "2 pid=(0,2) keys=[b] mode=X gen=2 started=1 taken=[]\n"
                             "4 pid=(0,4) keys=[m.n,x] mode=S gen=3 started=1 taken=[]\n"
                             "5 pid=(0,5) keys=[k] mode=X gen=4 started=0 taken=[k]\n"
                             "tickets\n"
                             "1 pid=(0,1) keys=[b] mode=X state=queued hold=- error=-\n"
                             "2 pid=(0,2) keys=[a] mode=X state=failed hold=- "
                             "error=DeadlockError[(2:a),(1:b)]\n"
                             "3 pid=(0,5) keys=[k] mode=X state=granted hold=5 error=-\n"
                             "queue=[1]\n"
                             "abandoned=[]\n"
                             "next seq=3 hold=5 gen=4\n"
                             "cache_refs\n"
                             "H0 {}\n"
                             "counters grants=5\n");
}

// -- stats
// ------------------------------------------------------------------------------------------

TEST(LockTable, Stats) {
  world w;
  const std::uint64_t a = w.acquire(1, X, {"a"});
  const std::uint64_t t2 = w.wait(2, X, {"a"});
  const std::uint64_t t3 = w.wait(3, X, {"a"});
  w.cancel(3, t3);
  w.acquire(4, X, {"b"});
  w.wait(4, X, {"a"});
  w.block(1, X, {"b"}); // deadlock: 1 -> 4 -> 1
  w.end(1, true);       // abandons a; grants 2
  w.t.note_leaked_at_exit();
  const lock_stats s = w.t.stats();
  EXPECT_EQ(s.holds, 2u);
  EXPECT_EQ(s.queued, 1u);
  EXPECT_EQ(s.grants, 3u);
  EXPECT_EQ(s.fast_grants, 2u);
  EXPECT_EQ(s.waits, 3u);
  EXPECT_EQ(s.timeouts, 1u);
  EXPECT_EQ(s.deadlocks, 1u);
  EXPECT_EQ(s.abandoned, 1u);
  EXPECT_EQ(s.leaked_at_exit, 1u);
  EXPECT_EQ(s.ends, 1u);
  EXPECT_EQ(s.trie_nodes, 3u);
  EXPECT_EQ(s.cache_refs, 0u);
  EXPECT_EQ(s.relevels, 0u);
  EXPECT_EQ(s.posts, 0u);
  (void)a;
  (void)t2;
}

// -- mutations
// ----------------------------------------------------------------------------------------

TEST(LockTable, MutationNoExemption) {
  world w(lock_table::no_exemption);
  w.acquire(1, S, {"a.x"});
  w.wait(2, X, {"a"});
  EXPECT_FALSE(w.try_acquire(1, X, {"a.y"})); // LK-27 waits behind Q...
  auto b = w.block(1, X, {"a.y"});
  EXPECT_EQ(b.state, ticket_state::failed); // ...and that closes a cycle
  world normal;
  normal.acquire(1, S, {"a.x"});
  normal.wait(2, X, {"a"});
  EXPECT_TRUE(normal.try_acquire(1, X, {"a.y"}));
}

TEST(LockTable, MutationCancelReexaminesHoldsOnly) {
  world w(lock_table::cancel_reexamines_holds_only, false);
  w.auto_check = false;
  w.acquire(1, S, {"k"});
  const std::uint64_t tw = w.wait(2, X, {"k"});
  const std::uint64_t tr = w.wait(3, S, {"k"});
  w.cancel(2, tw);
  EXPECT_TRUE(w.notes.empty());
  EXPECT_EQ(w.state(tr), ticket_state::queued); // LK-25's reader left queued while grantable
  EXPECT_EQ(violation_of(w.t), "I6");
}

TEST(LockTable, MutationKeepDetachedRefs) {
  world w(lock_table::keep_detached_refs, false);
  w.auto_check = false;
  auto ks = refs_of({"a"});
  w.t.ref_nodes(w.h, ks);
  w.t.detach_home(w.h, w.notes);
  EXPECT_EQ(violation_of(w.t), "I9");
  EXPECT_EQ(w.t.stats().trie_nodes, 2u); // the node outlives its home
  EXPECT_NE(w.t.canonical().find("cache_refs\nH0 {a:1}\n"), std::string::npos);
  world normal;
  auto nks = refs_of({"a"});
  normal.t.ref_nodes(normal.h, nks);
  normal.t.detach_home(normal.h, normal.notes);
  EXPECT_EQ(violation_of(normal.t), "");
  EXPECT_EQ(normal.t.stats().trie_nodes, 1u);
}

TEST(LockTable, MutationUnrefPopsEntry) {
  world w(lock_table::unref_pops_entry, false);
  w.auto_check = false;
  auto ks = refs_of({"a"});
  w.t.ref_nodes(w.h, ks);
  detail::lock_node *const drop[] = {ks[0].node};
  w.t.unref_nodes(w.h, drop);
  EXPECT_EQ(violation_of(w.t), "I9"); // the attached home lost its entry
  EXPECT_EQ(w.t.canonical().find("H0 {"), std::string::npos);
  auto again = refs_of({"a"});
  EXPECT_THROW(w.t.ref_nodes(w.h, again), detail::lock_violation);
  world normal;
  auto nks = refs_of({"a"});
  normal.t.ref_nodes(normal.h, nks);
  normal.t.unref_nodes(normal.h, std::span<detail::lock_node *const>(&nks[0].node, 1));
  EXPECT_NE(normal.t.canonical().find("cache_refs\nH0 {}\n"), std::string::npos);
}

TEST(LockTable, MutationCancelMarksFailed) {
  world w(lock_table::cancel_marks_failed);
  w.acquire(1, X, {"k"});
  const std::uint64_t tid = w.wait(2, X, {"k"});
  EXPECT_EQ(w.cancel(2, tid), ticket_state::cancelled); // the result is the protocol's
  EXPECT_EQ(w.state(tid), ticket_state::failed);
  EXPECT_NE(w.t.canonical().find("state=failed hold=- error=TimeoutError\n"), std::string::npos);
  EXPECT_EQ(w.resume(2, tid).what, detail::lock_outcome::kind::timeout);
}

TEST(LockTable, MutationPlantDivergence) {
  auto run = [](world &w) {
    const std::uint64_t a = w.acquire(1, X, {"a"});
    const std::uint64_t t2 = w.wait(2, X, {"a"});
    const std::uint64_t t3 = w.wait(3, X, {"a"});
    w.release(1, a); // grants 2: the planted release
    const std::uint64_t h2 = w.enter(2, t2);
    w.release(2, h2); // grants 3: counted again
    w.release(3, w.enter(3, t3));
    return w.t.stats().grants;
  };
  world planted(lock_table::plant_divergence);
  world normal;
  EXPECT_EQ(run(normal), 3u);
  EXPECT_EQ(run(planted), 2u);
  const std::string a = normal.t.canonical(), b = planted.t.canonical();
  EXPECT_NE(a, b);
  EXPECT_EQ(a.substr(0, a.find("counters")), b.substr(0, b.find("counters")));
}

TEST(LockTable, MutationCancelSkipsRelevel) {
  world w(lock_table::cancel_skips_relevel, false, true);
  w.auto_check = false;
  w.acquire(1, X, {"a"}, "o", 5);
  const std::uint64_t t2 = w.wait(2, X, {"a"}, "o", -5);
  w.cancel(2, t2);
  EXPECT_TRUE(w.posts().empty());
  EXPECT_EQ(w.inh(1), -5);            // PI-3's boost kept after its source timed out
  EXPECT_EQ(violation_of(w.t), "I8"); // check()'s own search finds it
}

TEST(LockTable, MutationCoalesceNoClear) {
  world w(lock_table::coalesce_no_clear, true, true);
  w.acquire(1, X, {"a"}, "o", 5);
  const std::uint64_t t2 = w.wait(2, X, {"a"}, "o", -5);
  EXPECT_EQ(w.t.read_inh(w.pid(1)), -5); // the apply leaves the flag set
  w.cancel(2, t2);
  EXPECT_TRUE(w.posts().empty()); // so the withdrawal is coalesced away, for ever
  EXPECT_EQ(w.t.stats().relevels_coalesced, 1u);
  EXPECT_EQ(w.inh(1), detail::lock_inf); // the manager is right; a home's copy stays -5 (the
                                         // model check's I8 quiescent clause finds it)
}

TEST(LockTable, MutationImplNoteAfterOnly) {
  // Q holds X a; P (10) queues; W (-5) queues, then times out: W has no edge after the cancel,
  // so a set A built from the graph after only misses P and Q, whose inh changes
  world w(lock_table::impl_note_after_only, true, true);
  w.acquire(1, X, {"a"}, "o", 0);
  w.wait(2, X, {"a"}, "o", 10);
  const std::uint64_t t3 = w.wait(3, X, {"a"}, "o", -5);
  EXPECT_EQ(w.inh(2), -5);
  try {
    w.t.cancel(w.pid(3), t3, w.notes);
    ADD_FAILURE() << "no violation";
  } catch (const detail::lock_violation &v) {
    EXPECT_EQ(v.invariant(), "impl-note") << v.what();
  }
  world normal(0, true, true);
  normal.acquire(1, X, {"a"}, "o", 0);
  normal.wait(2, X, {"a"}, "o", 10);
  const std::uint64_t n3 = normal.wait(3, X, {"a"}, "o", -5);
  normal.t.read_inh(normal.pid(1));
  normal.t.read_inh(normal.pid(2));
  normal.cancel(3, n3);
  EXPECT_EQ(normal.posts(), (std::vector<std::string>{"relevel(1)", "relevel(2)"}));
  EXPECT_EQ(normal.inh(1), 10);
  EXPECT_EQ(normal.inh(2), detail::lock_inf);
}

// == L3: shared mode, several homes ===============================================================

namespace {

std::shared_ptr<lock_manager> make_manager(lock_threading mode) {
  lock_manager::options o;
  o.threading = mode;
  o.self_check = true;
  return std::make_shared<lock_manager>(o);
}

const char *mode_name(lock_threading m) {
  return m == lock_threading::shared ? "shared" : "single";
}

/// A scheduler as its lock home sees it (lock_sched_ops), over a process table of its own: what
/// the home-level cases need and no more.
struct fake_ops : detail::lock_sched_ops {
  detail::process_table procs;
  std::vector<int> parks, readies;
  int throw_ready_for = -1; // ready_lock_waiter throws for this pid (an injected failure)

  bool park_lock(process &p) override {
    parks.push_back(p.pid);
    procs.set_status(p, process_status::waiting);
    return true;
  }
  void ready_lock_waiter(process &p) override {
    if (p.pid == throw_ready_for)
      throw std::runtime_error("ready_lock_waiter: injected");
    readies.push_back(p.pid);
    procs.set_status(p, process_status::ready);
  }
  void relevel(process &p) override { procs.update(p); }
  std::size_t lock_expired_count() const override { return procs.lock_expired_count(); }
  void take_lock_expired(std::vector<process *> &out) override { procs.take_lock_expired(out); }
  process *find_process(int pid) override { return procs.find(pid); }
  void for_each_process(const std::function<void(process &)> &fn) override {
    for (const auto &[pid, p] : procs.all())
      fn(*p);
  }
};

/// One scheduler's side of a home-level case: its processes and its home, which goes first, as
/// a scheduler's does (protocol 5.7).
struct sched_side {
  fake_ops ops;
  std::deque<std::string> written; // the request keys as written, alive as long as the side
  std::unique_ptr<detail::lock_home> home;

  explicit sched_side(std::shared_ptr<lock_manager> mgr, lock_attach attach = {},
                      std::shared_ptr<detail::wakeup> wake = nullptr)
      : home(std::make_unique<detail::lock_home>(
            std::move(mgr), ops, scheduling_policy::round_robin, std::move(wake), attach)) {}
  ~sched_side() { home.reset(); }
  sched_side(const sched_side &) = delete;
  sched_side &operator=(const sched_side &) = delete;

  process &add(int pid) {
    process_ptr p = make_process();
    p->pid = pid;
    p->status = process_status::ready;
    return *ops.procs.insert(std::move(p));
  }

  /// A request as build_request makes one: each key found in the home's cache or passed absolute.
  detail::home_request request(lock_mode m, keys_t keys) {
    home->begin_request();
    detail::home_request r;
    r.mode = m;
    for (std::string_view k : keys) {
      const std::string &w = written.emplace_back(k);
      detail::home_request::key hk;
      hk.written = w;
      hk.node = home->lookup(0, w);
      if (!hk.node)
        hk.absolute = w;
      r.keys.push_back(std::move(hk));
    }
    detail::canonicalize(r);
    return r;
  }
  /// P takes KEYS on the fast path (5.1): the hold, or nullopt when busy.
  std::optional<std::uint64_t> take(process &p, lock_mode m, keys_t keys) {
    detail::home_request r = request(m, keys);
    const detail::enter_result e = home->try_enter(p, r, "");
    if (e.what == detail::enter_result::kind::busy)
      return std::nullopt;
    return e.hold;
  }
  /// P (stepping) waits for KEYS (5.2): true when it is queued and parked.
  bool wait(process &p, lock_mode m, keys_t keys) {
    ops.procs.set_status(p, process_status::running);
    detail::home_request r = request(m, keys);
    return !home->wait(p, r, std::nullopt, "").has_value();
  }
  /// The hold P was granted last (a wake's collect appends it to the mirror).
  static std::uint64_t last_hold(const process &p) {
    return p.locks && !p.locks->holds.empty() ? p.locks->holds.back().id : 0;
  }
  void release(process &p, std::uint64_t hold) { home->release_frame(p, hold, false); }
};

/// The manager's check() and every given side's check_quiescent(), as failures.
void check_all(const lock_manager &mgr, std::initializer_list<const sched_side *> sides) {
  try {
    mgr.check();
    for (const sched_side *s : sides)
      if (s)
        s->home->check_quiescent();
  } catch (const detail::lock_violation &v) {
    ADD_FAILURE() << v.what();
  }
}

/// Whether two canonical key lists overlap (protocol 1).
bool keys_overlap(const std::vector<std::string> &a, const std::vector<std::string> &b) {
  for (const std::string &x : a)
    for (const std::string &y : b)
      if (detail::key_prefix(x, y) || detail::key_prefix(y, x))
        return true;
  return false;
}

/// A table request over `keys`, which it views: they must outlive it.
detail::lock_request request_of(lock_mode m, const std::vector<std::string> &keys) {
  detail::lock_request r;
  r.mode = m;
  for (const std::string &k : keys)
    r.keys.push_back(detail::key_ref{nullptr, k, false});
  return r;
}

} // namespace

// -- L3.1: shared mode, the attach rules, several homes -------------------------------------------

TEST(LockShared, SharedModeConstructs) {
  const std::shared_ptr<lock_manager> m = make_manager(lock_threading::shared);
  EXPECT_EQ(m->threading(), lock_threading::shared);
  EXPECT_TRUE(m->table().opts().shared);
  EXPECT_FALSE(m->table().opts().one_home);
  EXPECT_TRUE(m->table().empty());
  m->check();
  lock_manager single;
  EXPECT_EQ(single.threading(), lock_threading::single);
  EXPECT_FALSE(single.table().opts().shared);
  EXPECT_FALSE(single.table().opts().one_home);
}

TEST(LockShared, CrossThreadAttachNeedsASharedManager) {
  // 4.4's L3 rule (design 8.3): a home that may be pumped on another thread is refused by a
  // single manager, naming the setting; a shared manager takes it. On the table, and through a
  // home (the schedulers' set_lock_manager makes one).
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const bool shared = mode == lock_threading::shared;
    lock_table::options o;
    o.self_check = true;
    o.one_home = false;
    o.shared = shared;
    lock_table t(o);
    detail::lock_notes n;
    EXPECT_EQ(t.attach_home(false, nullptr, nullptr, n), 0u); // a same-thread home: accepted
    if (shared) {
      EXPECT_EQ(t.attach_home(false, nullptr, nullptr, n, lock_attach{true}), 1u);
    } else {
      try {
        t.attach_home(false, nullptr, nullptr, n, lock_attach{true});
        ADD_FAILURE() << "a cross-thread home attached to a single table";
      } catch (const std::logic_error &e) {
        EXPECT_NE(std::string(e.what()).find("state_exec.locks.threading"), std::string::npos)
            << e.what();
        // and when that mode is chosen (R3b F6)
        EXPECT_NE(std::string(e.what()).find("fixed when the app first makes its manager"),
                  std::string::npos)
            << e.what();
      }
      EXPECT_NE(t.canonical().find("homes\nH0 prio=0\nrecs\n"), std::string::npos); // no H1
    }
    t.check();

    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    fake_ops ops;
    if (shared) {
      detail::lock_home h(mgr, ops, scheduling_policy::round_robin, nullptr, lock_attach{true});
      EXPECT_EQ(h.id(), 0u);
      check_all(*mgr, {});
    } else {
      try {
        detail::lock_home h(mgr, ops, scheduling_policy::round_robin, nullptr, lock_attach{true});
        ADD_FAILURE() << "a cross-thread home attached to a single manager";
      } catch (const std::logic_error &e) {
        EXPECT_NE(std::string(e.what()).find("state_exec.locks.threading"), std::string::npos)
            << e.what();
      }
      EXPECT_NE(mgr->table().canonical().find("homes\nrecs\n"), std::string::npos); // none
      detail::lock_home h(mgr, ops, scheduling_policy::round_robin); // same thread: accepted
      EXPECT_EQ(h.id(), 0u);                                         // the refused home took no id
    }
    EXPECT_TRUE(mgr->table().empty());
    mgr->check();
  }
}

TEST(LockHomes, SeveralHomesAttachAndDetach) {
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    auto c = std::make_unique<sched_side>(mgr);
    EXPECT_EQ(a->home->id(), 0u);
    EXPECT_EQ(b->home->id(), 1u);
    EXPECT_EQ(c->home->id(), 2u);
    process &pa = a->add(1);
    process &pb = b->add(1); // pids repeat across homes: (1,1) is not (0,1)
    process &pc = c->add(2);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    EXPECT_FALSE(b->take(pb, X, {"k"})); // A's hold blocks B's process
    ASSERT_TRUE(b->wait(pb, X, {"k"}));
    EXPECT_EQ(b->ops.parks, (std::vector<int>{1}));
    const std::optional<std::uint64_t> hc = c->take(pc, S, {"j"});
    ASSERT_TRUE(hc);
    check_all(*mgr, {a.get(), b.get(), c.get()});
    // A's release grants B's waiter on A's thread; the wake goes to B's port (4.14), and B
    // applies it when it drains (5.9).
    a->release(pa, *ha);
    EXPECT_FALSE(a->home->port_pending());
    EXPECT_TRUE(b->home->port_pending());
    EXPECT_TRUE(b->ops.readies.empty());
    b->home->drain_port();
    EXPECT_FALSE(b->home->port_pending());
    EXPECT_EQ(b->ops.readies, (std::vector<int>{1}));
    const std::uint64_t hb = sched_side::last_hold(pb);
    EXPECT_NE(hb, 0u);
    EXPECT_EQ(mgr->stats().holds, 2u);
    check_all(*mgr, {a.get(), b.get(), c.get()});
    // B is destroyed holding k: its detach ends its process; the others keep theirs.
    b.reset();
    EXPECT_EQ(mgr->stats().holds, 1u);
    check_all(*mgr, {a.get(), c.get()});
    // k is free again for A.
    const std::optional<std::uint64_t> ha2 = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha2);
    a->release(pa, *ha2);
    c->release(pc, *hc);
    // A home attached later gets a new id; ids are never reused.
    auto d = std::make_unique<sched_side>(mgr);
    EXPECT_EQ(d->home->id(), 3u);
    check_all(*mgr, {a.get(), c.get(), d.get()});
    a.reset();
    c.reset();
    d.reset();
    EXPECT_TRUE(mgr->table().empty());
    EXPECT_EQ(mgr->stats().cache_refs, 0u);
    EXPECT_EQ(mgr->stats().trie_nodes, 1u);
    EXPECT_NE(mgr->table().canonical().find("homes\nrecs\n"), std::string::npos);
    mgr->check();
  }
}

TEST(LockShared, TableDrivenFromSeveralThreads) {
  // One shared table, a home per thread, each thread driving its own pids: contended keys,
  // nested requests that may close a cycle, waits resolved by other threads' releases, and
  // timeouts. self_check runs check() inside every operation, under M.mu. As homes do, a thread
  // hands each operation's wakes for another home to that home (here a flag, not a port), and a
  // waiter reads its ticket when woken.
  constexpr int kThreads = 4;
  constexpr int kOps = 1500;
  lock_table::options o;
  o.self_check = true;
  o.one_home = false;
  o.shared = true;
  lock_table t(o);
  std::vector<detail::home_id> homes;
  {
    detail::lock_notes n;
    for (int i = 0; i < kThreads; ++i)
      homes.push_back(t.attach_home(false, nullptr, nullptr, n, lock_attach{true}));
    EXPECT_TRUE(n.empty());
  }
  const std::vector<std::vector<std::string>> requests = {
      {"a"}, {"a.b"},      {"a.c"},        {"b"},      {"b.x"},
      {"c"}, {"a.b", "c"}, {"a.c", "b.x"}, {"b", "d"}, {"d"}};
  std::atomic<int> started{0}, hot_queued{0};
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> waited{0}, deadlocked{0}, timed_out{0};
  std::vector<std::string> errors(kThreads);
  std::vector<std::atomic<bool>> woken(kThreads); // by home id: a wake was handed to it

  const auto driver = [&](int i) {
    const detail::home_id h = homes[i];
    std::mt19937 rng(0x5eed + i);
    detail::lock_notes notes;
    const auto fail = [&](const std::string &what) {
      if (errors[i].empty())
        errors[i] = what;
      stop = true;
    };
    // 4.14 for this test: every note is a wake (no home applies priorities), handed to its home.
    const auto deliver = [&] {
      for (const detail::lock_note &n : notes)
        woken[n.home].store(true);
      notes.clear();
    };
    // Wait for ticket `tid` of `p` until it is granted or failed, reading it when woken, or
    // cancel it after `patience` idle turns (a timeout: cancelled, or granted already and
    // adopted). The outcome, collected.
    const auto await = [&](detail::lock_pid p, std::uint64_t tid,
                           int patience) -> detail::lock_outcome {
      const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(60);
      for (int idle = 0;; ++idle) {
        if (idle == 0 || woken[h].exchange(false)) {
          const auto v = t.ticket_of(tid);
          if (!v) {
            fail("ticket " + std::to_string(tid) + " vanished");
            return {};
          }
          if (v->state != ticket_state::queued)
            break;
        }
        if (idle == patience) {
          t.cancel(p, tid, notes);
          deliver();
          ++timed_out;
          break;
        }
        if (stop || std::chrono::steady_clock::now() > give_up) {
          fail("a wait never ended");
          return {};
        }
        std::this_thread::yield();
      }
      return t.resume(p, tid);
    };
    // Acquire for p as a home would: the fast path, else block and wait. The hold, or 0.
    const auto acquire = [&](detail::lock_pid p, lock_mode m, const std::vector<std::string> &ks,
                             int patience, bool hot = false) -> std::uint64_t {
      detail::lock_request r = request_of(m, ks);
      if (const auto g = t.try_acquire(p, {"o", 0}, r, {}))
        return g->hold;
      r = request_of(m, ks);
      const detail::block_result b = t.block(p, {"o", 0}, r, {}, notes);
      deliver();
      if (b.state == ticket_state::queued) {
        ++waited;
        if (hot)
          ++hot_queued;
      }
      const detail::lock_outcome out = await(p, b.tid, patience);
      if (out.what == detail::lock_outcome::kind::deadlock)
        ++deadlocked;
      if (out.what != detail::lock_outcome::kind::granted)
        return 0;
      t.start_body(p, out.g.hold);
      return out.g.hold;
    };
    const auto release = [&](detail::lock_pid p, std::uint64_t hold) {
      t.release(p, hold, notes);
      deliver();
    };

    ++started;
    while (started < kThreads)
      std::this_thread::yield();
    try {
      // A first round on one hot key: whoever is granted holds it until every other thread has
      // queued behind it, so every run has real waits resolved by a release.
      {
        const detail::lock_pid p{h, 1};
        const std::vector<std::string> hot{"hot"}; // the request's keys view these strings
        detail::lock_request r = request_of(X, hot);
        if (const auto g = t.try_acquire(p, {"o", 0}, r, {})) {
          const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(60);
          while (hot_queued < kThreads - 1 && !stop) {
            if (std::chrono::steady_clock::now() > give_up)
              fail("the hot key's waiters never queued");
            std::this_thread::yield();
          }
          release(p, g->hold);
        } else if (const std::uint64_t hold = acquire(p, X, hot, -1, true)) {
          release(p, hold);
        } else {
          fail("a hot-key wait was not granted");
        }
      }
      for (int op = 0; op < kOps && !stop; ++op) {
        const detail::lock_pid p{h, 1 + op % 3};
        const std::vector<std::string> &outer = requests[rng() % requests.size()];
        const lock_mode mo = rng() % 3 == 0 ? S : X;
        const int patience = rng() % 8 == 0 ? int(rng() % 64) : -1;
        const std::uint64_t ho = acquire(p, mo, outer, patience);
        if (ho && rng() % 2 == 0) {
          // A nested request on keys apart from the outer ones: it may close a cycle with
          // another thread's process (DeadlockError), or wait while holding.
          std::vector<std::string> inner;
          for (int tries = 0; tries < 8 && inner.empty(); ++tries) {
            const std::vector<std::string> &c = requests[rng() % requests.size()];
            if (!keys_overlap(c, outer))
              inner = c;
          }
          if (!inner.empty())
            if (const std::uint64_t hi = acquire(p, rng() % 3 == 0 ? S : X, inner, patience))
              release(p, hi);
        }
        if (ho)
          release(p, ho);
        if (op % 97 == 0) {
          t.check();
          (void)t.query("", h, "o");
          (void)t.stats();
        }
      }
    } catch (const std::exception &e) {
      fail(e.what());
    }
  };

  std::vector<std::thread> threads;
  for (int i = 0; i < kThreads; ++i)
    threads.emplace_back(driver, i);
  for (std::thread &th : threads)
    th.join();
  for (int i = 0; i < kThreads; ++i)
    EXPECT_TRUE(errors[i].empty()) << "thread " << i << ": " << errors[i];
  std::printf("LockShared: %llu waits, %llu deadlocks, %llu timeouts over %d threads\n",
              (unsigned long long)waited.load(), (unsigned long long)deadlocked.load(),
              (unsigned long long)timed_out.load(), kThreads);
  EXPECT_GE(waited.load(), std::uint64_t(kThreads - 1)); // the hot round's, at least
  EXPECT_TRUE(t.empty()) << t.canonical();
  t.check();
  detail::lock_notes n;
  for (detail::home_id h : homes)
    t.detach_home(h, n);
  EXPECT_TRUE(n.empty());
  EXPECT_EQ(t.stats().trie_nodes, 1u);
  EXPECT_NE(t.canonical().find("homes\nrecs\n"), std::string::npos);
  t.check();
}

// -- L3.2: the port, posts, and a home's destruction ----------------------------------------------

namespace {

/// Counts the notes deliver hands to another home's port (lock_test_hooks::cross_home_note), from
/// any thread, while alive.
std::atomic<std::uint64_t> g_cross_notes{0};
void count_cross_note(const detail::lock_note &) {
  g_cross_notes.fetch_add(1, std::memory_order_relaxed);
}
struct note_counter {
  detail::lock_test_hooks hooks;
  note_counter() {
    g_cross_notes = 0;
    hooks.cross_home_note = count_cross_note;
    detail::set_lock_test_hooks(&hooks);
  }
  ~note_counter() { detail::set_lock_test_hooks(nullptr); }
  note_counter(const note_counter &) = delete;
  note_counter &operator=(const note_counter &) = delete;
  std::uint64_t count() const { return g_cross_notes.load(); }
};

detail::lock_post wake_post(int pid, std::uint64_t tid) {
  return detail::lock_post{detail::lock_post::kind::wake, pid, tid};
}
detail::lock_post relevel_post(int pid) {
  return detail::lock_post{detail::lock_post::kind::relevel, pid, 0};
}
std::string text(const detail::lock_post &p) {
  return p.what == detail::lock_post::kind::wake
             ? "wake(" + std::to_string(p.pid) + "," + std::to_string(p.tid) + ")"
             : "relevel(" + std::to_string(p.pid) + ")";
}
std::vector<std::string> texts(const std::vector<detail::lock_post> &v) {
  std::vector<std::string> out;
  for (const detail::lock_post &p : v)
    out.push_back(text(p));
  return out;
}

/// What one round of PostsRacingACloseNeverLand saw.
struct race_result {
  std::uint64_t attempted = 0, posts = 0, dropped = 0, drained = 0, discarded = 0;
  bool landed_after_close = false;
};

/// Posters on `kPosters` threads, each through a home of its own, deliver notes for home B's
/// process while B's thread drains some and then destroys B (protocol 5.7). Every post lands
/// before the close or is dropped.
race_result post_close_race(std::uint64_t round) {
  constexpr int kPosters = 3;
  race_result r;
  const std::shared_ptr<lock_manager> mgr = make_manager(lock_threading::shared);
  note_counter notes_seen;
  auto b = std::make_unique<sched_side>(mgr, lock_attach{true}, std::make_shared<detail::wakeup>());
  const std::shared_ptr<detail::lock_port> port = b->home->port();
  detail::lock_notes one;
  one.push_back(
      detail::lock_note{b->home->id(), port, round % 2 ? wake_post(1, round) : relevel_post(1)});
  std::vector<std::unique_ptr<sched_side>> posters;
  for (int i = 0; i < kPosters; ++i)
    posters.push_back(std::make_unique<sched_side>(mgr, lock_attach{true}));
  std::atomic<int> ready{0};
  std::vector<std::thread> threads;
  for (int i = 0; i < kPosters; ++i)
    threads.emplace_back([&, i] {
      ++ready;
      // Post until this thread has seen the port closed and posted a few times more, so every
      // round has posts on both sides of the close; bounded in case the close never comes. A
      // yield after each post keeps three posters from starving the closing thread.
      const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(60);
      int after = 0;
      for (int k = 1; after < 8; ++k) {
        posters[i]->home->deliver(one);
        if (port->closed())
          ++after;
        if (k % 1024 == 0 && std::chrono::steady_clock::now() > give_up)
          break;
        std::this_thread::yield();
      }
    });
  while (ready < kPosters)
    std::this_thread::yield();
  // B's thread: wait for a post (a bounded spin), drain what has landed so far, then go.
  std::vector<detail::lock_post> got;
  for (int spin = 0; spin < 100000 && !port->pending(); ++spin)
    std::this_thread::yield();
  port->drain(got);
  r.drained = got.size();
  b.reset();
  for (std::thread &t : threads)
    t.join();
  port->drain(got); // nothing may have landed after the close
  r.landed_after_close = !got.empty() || port->pending();
  r.attempted = notes_seen.count();
  const lock_stats s = mgr->stats();
  r.posts = s.posts;
  r.dropped = s.posts_dropped;
  r.discarded = s.posts_discarded;
  posters.clear();
  EXPECT_TRUE(mgr->table().empty());
  return r;
}

/// Runs post_close_race `rounds` times and checks each round's counts.
void run_post_close_races(int rounds) {
  std::uint64_t attempted = 0, dropped = 0, discarded = 0;
  for (int round = 0; round < rounds; ++round) {
    const race_result r = post_close_race(std::uint64_t(round));
    attempted += r.attempted;
    dropped += r.dropped;
    discarded += r.discarded;
    EXPECT_FALSE(r.landed_after_close) << "round " << round;
    EXPECT_EQ(r.posts + r.dropped, r.attempted) << "round " << round;
    EXPECT_EQ(r.posts, r.drained + r.discarded) << "round " << round;
    EXPECT_GE(r.dropped, 1u) << "round " << round; // every poster posted after the close
    if (::testing::Test::HasFailure())
      break;
  }
  std::printf("LockHomes: %d rounds, %llu notes, %llu dropped at the close, %llu discarded\n",
              rounds, (unsigned long long)attempted, (unsigned long long)dropped,
              (unsigned long long)discarded);
}

} // namespace

TEST(LockPort, PostCloseDrain) {
  const auto wake = std::make_shared<detail::wakeup>();
  detail::lock_port port(wake);
  std::vector<detail::lock_post> out{relevel_post(99)}; // drain clears what it is given
  EXPECT_FALSE(port.pending());
  EXPECT_FALSE(port.closed());
  port.drain(out);
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(port.post(wake_post(1, 10)));
  EXPECT_TRUE(port.pending());
  // The post notified the shared wakeup: a wait begun after it ends at once (a one-sided
  // ceiling; the time printed).
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(wake->wait_until(t0 + std::chrono::seconds(10)));
  std::printf(
      "LockPort: the notified wait took %.3f ms\n",
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  EXPECT_TRUE(port.post(relevel_post(2)));
  EXPECT_TRUE(port.post(wake_post(3, 11)));
  port.drain(out); // FIFO
  EXPECT_EQ(texts(out), (std::vector<std::string>{"wake(1,10)", "relevel(2)", "wake(3,11)"}));
  EXPECT_FALSE(port.pending());
  port.drain(out);
  EXPECT_TRUE(out.empty());
  EXPECT_TRUE(port.post(relevel_post(4)));
  port.close();
  EXPECT_TRUE(port.closed());
  EXPECT_TRUE(port.pending()); // the close keeps what landed before it
  EXPECT_FALSE(port.post(wake_post(5, 12)));
  EXPECT_FALSE(port.post(relevel_post(6)));
  port.drain(out);
  EXPECT_EQ(texts(out), (std::vector<std::string>{"relevel(4)"})); // nothing after the close
  EXPECT_FALSE(port.pending());
  EXPECT_FALSE(port.post(relevel_post(7)));
  EXPECT_FALSE(port.pending());
  // A port with no wakeup takes posts all the same.
  detail::lock_port bare(nullptr);
  EXPECT_TRUE(bare.post(relevel_post(1)));
  EXPECT_TRUE(bare.pending());
}

TEST(LockHomes, CrossHomeNoteLandsInItsPort) {
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    note_counter notes_seen;
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb = b->add(7);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb, X, {"k"}));
    // A's release grants B's waiter; its wake is a note for B, posted to B's port.
    a->release(pa, *ha);
    EXPECT_EQ(notes_seen.count(), 1u);
    EXPECT_EQ(mgr->stats().posts, 1u);
    EXPECT_EQ(mgr->stats().posts_dropped, 0u);
    EXPECT_TRUE(b->home->port_pending());
    // A note handed to deliver by hand goes the same way, behind the first.
    detail::lock_notes hand;
    hand.push_back(detail::lock_note{b->home->id(), b->home->port(), relevel_post(7)});
    a->home->deliver(hand);
    EXPECT_EQ(notes_seen.count(), 2u);
    EXPECT_EQ(mgr->stats().posts, 2u);
    // A note for this home is applied at once, never posted or counted.
    detail::lock_notes own;
    own.push_back(detail::lock_note{a->home->id(), a->home->port(), relevel_post(1)});
    a->home->deliver(own);
    EXPECT_EQ(notes_seen.count(), 2u);
    EXPECT_FALSE(a->home->port_pending());
    // B applies both in order: the wake readies its waiter, the relevel re-reads (no change).
    b->home->drain_port();
    EXPECT_EQ(b->ops.readies, (std::vector<int>{7}));
    const std::uint64_t hb = sched_side::last_hold(pb);
    ASSERT_NE(hb, 0u);
    check_all(*mgr, {a.get(), b.get()});
    // A note captured for a home that is then destroyed finds its port closed: dropped.
    const std::shared_ptr<detail::lock_port> b_port = b->home->port();
    const detail::home_id b_id = b->home->id();
    b.reset();
    EXPECT_TRUE(b_port->closed());
    detail::lock_notes late;
    late.push_back(detail::lock_note{b_id, b_port, wake_post(7, 1)});
    a->home->deliver(late);
    EXPECT_EQ(notes_seen.count(), 3u);
    const lock_stats s = mgr->stats();
    EXPECT_EQ(s.posts, 2u);
    EXPECT_EQ(s.posts_dropped, 1u);
    EXPECT_EQ(s.posts_discarded, 0u); // B drained both before it went
    EXPECT_EQ(s.holds, 0u);           // B's detach released its grant
    a.reset();
    EXPECT_TRUE(mgr->table().empty());
    mgr->check();
  }
}

TEST(LockHomes, DestroyedHomeDiscardsItsPosts) {
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb = b->add(2);
    process &pb2 = b->add(3);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb, X, {"k"}));
    const std::optional<std::uint64_t> hb2 = b->take(pb2, S, {"j"});
    ASSERT_TRUE(hb2);
    a->release(pa, *ha); // grants pb: a wake in B's port
    detail::lock_notes hand;
    hand.push_back(detail::lock_note{b->home->id(), b->home->port(), relevel_post(2)});
    a->home->deliver(hand);
    EXPECT_EQ(mgr->stats().posts, 2u);
    EXPECT_EQ(mgr->stats().holds, 2u); // pb's grant, not yet collected, and pb2's S j
    // B goes with both posts unapplied (5.7): closed, detached (pb's grant and pb2's hold
    // released), its posts discarded and counted, its processes' lock fields cleared.
    const std::shared_ptr<detail::lock_port> b_port = b->home->port();
    b->home.reset();
    EXPECT_TRUE(b_port->closed());
    EXPECT_FALSE(b_port->pending());
    const lock_stats s = mgr->stats();
    EXPECT_EQ(s.posts_discarded, 2u);
    EXPECT_EQ(s.posts_dropped, 0u);
    EXPECT_EQ(s.holds, 0u);
    EXPECT_TRUE(b->ops.readies.empty()); // the wake was never applied
    for (const process *p : {&pb, &pb2}) {
      EXPECT_FALSE(p->lock_waiting);
      EXPECT_TRUE(!p->locks || (p->locks->holds.empty() && !p->locks->ticket));
    }
    // k is A's to take again.
    const std::optional<std::uint64_t> ha2 = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha2);
    a->release(pa, *ha2);
    check_all(*mgr, {a.get()});
    b.reset();
    a.reset();
    EXPECT_TRUE(mgr->table().empty());
    mgr->check();
  }
}

TEST(LockHomes, PostsRacingACloseNeverLand) {
  // Protocol 2.3: post and close exclude each other. Over many rounds of posters on other
  // threads racing a home's destruction, every note handed to the port either landed before the
  // close (and was drained, or discarded at the close) or was dropped; none landed after.
  run_post_close_races(200);
}

// -- L3.3: blocking_sinks (protocol 4.13) ---------------------------------------------------------

namespace {

using sinks_t = std::vector<std::pair<detail::lock_pid, std::int64_t>>;

std::vector<int> sink_pids(const sinks_t &v) {
  std::vector<int> out;
  for (const auto &[p, progress] : v)
    out.push_back(p.pid);
  return out;
}

} // namespace

TEST(LockTable, BlockingSinksFollowTheChain) {
  world w;
  w.acquire(3, X, {"c"});
  w.acquire(2, X, {"b"});
  w.wait(2, X, {"c"}); // 2 -> 3
  w.wait(1, X, {"b"}); // 1 -> 2
  // 1's one sink is 3, with 0 for its progress: this home was attached with no cell.
  EXPECT_EQ(w.t.blocking_sinks(w.pid(1)), (sinks_t{{w.pid(3), 0}}));
  EXPECT_EQ(sink_pids(w.t.blocking_sinks(w.pid(2))), (std::vector<int>{3}));
  EXPECT_TRUE(w.t.blocking_sinks(w.pid(3)).empty()); // holds, waits for nothing
  EXPECT_TRUE(w.t.blocking_sinks(w.pid(9)).empty()); // no record at all
  // Through a waiter edge too: 4 waits behind 2's hold and 1's earlier ticket on b.
  w.wait(4, X, {"b"});
  EXPECT_EQ(sink_pids(w.t.blocking_sinks(w.pid(4))), (std::vector<int>{3}));
  for (int p : {1, 2, 3, 4})
    w.end(p, false);
  EXPECT_TRUE(w.t.empty());
}

TEST(LockTable, BlockingSinksBothSharedHolders) {
  world w;
  w.acquire(1, S, {"k"});
  w.acquire(2, S, {"k.x"});
  w.acquire(5, S, {"other"});
  w.wait(3, X, {"k"});
  EXPECT_EQ(sink_pids(w.t.blocking_sinks(w.pid(3))), (std::vector<int>{1, 2}));
  EXPECT_TRUE(w.t.blocking_sinks(w.pid(1)).empty());
  for (int p : {1, 2, 3, 5})
    w.end(p, false);
  EXPECT_TRUE(w.t.empty());
}

TEST(LockTable, BlockingSinksCarryTheirHomesProgress) {
  lock_table::options o;
  o.self_check = true;
  o.one_home = false;
  lock_table t(o);
  const auto cell0 = std::make_shared<std::atomic<std::int64_t>>(100);
  const auto cell1 = std::make_shared<std::atomic<std::int64_t>>(200);
  detail::lock_notes n;
  const detail::home_id h0 = t.attach_home(false, nullptr, cell0, n);
  const detail::home_id h1 = t.attach_home(false, nullptr, cell1, n);
  const detail::home_id h2 = t.attach_home(false, nullptr, nullptr, n); // no cell
  const auto take = [&](detail::lock_pid p, std::vector<std::string> ks) {
    detail::lock_request r = request_of(X, ks);
    EXPECT_TRUE(t.try_acquire(p, {"o", 0}, r, {}));
  };
  const auto queue = [&](detail::lock_pid p, std::vector<std::string> ks) {
    detail::lock_request r = request_of(X, ks);
    EXPECT_EQ(t.block(p, {"o", 0}, r, {}, n).state, ticket_state::queued);
  };
  take({h1, 5}, {"a"});
  take({h2, 6}, {"b"});
  take({h0, 2}, {"c"});
  queue({h0, 1}, {"a", "b"});
  EXPECT_EQ(t.blocking_sinks({h0, 1}), (sinks_t{{{h1, 5}, 200}, {{h2, 6}, 0}}));
  cell1->store(250); // loaded at each call
  EXPECT_EQ(t.blocking_sinks({h0, 1}), (sinks_t{{{h1, 5}, 250}, {{h2, 6}, 0}}));
  // A sink on the waiter's own home carries its own home's cell.
  queue({h1, 7}, {"c"});
  EXPECT_EQ(t.blocking_sinks({h1, 7}), (sinks_t{{{h0, 2}, 100}}));
  t.check();
  for (detail::home_id h : {h0, h1, h2})
    t.detach_home(h, n);
  EXPECT_TRUE(t.empty());
  t.check();
}

TEST(LockHomes, BlockingSinksReadTheHomesProgressCell) {
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb = b->add(1);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb, X, {"k"}));
    // The scheduler's per-step store (design 8.4) is what B's waiter sees for A's holder.
    a->home->store_progress(detail::lock_clock::time_point(detail::lock_clock::duration(12345)));
    EXPECT_EQ(b->home->blocking_sinks(pb.pid), (sinks_t{{{a->home->id(), 1}, 12345}}));
    a->home->store_progress(detail::lock_clock::time_point(detail::lock_clock::duration(12346)));
    EXPECT_EQ(b->home->blocking_sinks(pb.pid), (sinks_t{{{a->home->id(), 1}, 12346}}));
    EXPECT_TRUE(a->home->blocking_sinks(pa.pid).empty());
    a->release(pa, *ha);
    EXPECT_TRUE(b->home->blocking_sinks(pb.pid).empty()); // granted: it waits for nothing
    b->home->drain_port();
    b->release(pb, sched_side::last_hold(pb));
    check_all(*mgr, {a.get(), b.get()});
    a.reset();
    b.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

// -- L3.7: CVC_EXEC_FUZZ --------------------------------------------------------------------------

namespace {

/// The fuzz seed while alive (0: off); off again after.
struct fuzz_scope {
  explicit fuzz_scope(std::uint64_t seed) { detail::set_lock_fuzz_seed(seed); }
  ~fuzz_scope() { detail::set_lock_fuzz_seed(0); }
  fuzz_scope(const fuzz_scope &) = delete;
  fuzz_scope &operator=(const fuzz_scope &) = delete;
};

/// A few operations through every protocol point on one thread: an operation with notes and its
/// delivery, posts, a drain of two posts, and a home's destruction.
void pass_every_point() {
  const std::shared_ptr<lock_manager> mgr = make_manager(lock_threading::shared);
  auto a = std::make_unique<sched_side>(mgr);
  auto b = std::make_unique<sched_side>(mgr);
  process &pa = a->add(1);
  process &pb = b->add(2);
  const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
  ASSERT_TRUE(ha);
  ASSERT_TRUE(b->wait(pb, X, {"k"}));
  a->release(pa, *ha); // after_op, post_enter, post_exit
  detail::lock_notes hand;
  hand.push_back(detail::lock_note{b->home->id(), b->home->port(), relevel_post(2)});
  a->home->deliver(hand);
  b->home->drain_port(); // drain_between
  EXPECT_EQ(b->ops.readies, (std::vector<int>{2}));
  check_all(*mgr, {a.get(), b.get()});
  b.reset(); // close_then_detach
  a.reset();
  EXPECT_TRUE(mgr->table().empty());
  EXPECT_EQ(mgr->stats().posts, 2u);
}

} // namespace

TEST(LockFuzz, OffByDefault) {
  const char *env = std::getenv("CVC_EXEC_FUZZ");
  if (!env || !*env)
    EXPECT_EQ(detail::lock_fuzz_seed(), 0u);
  {
    fuzz_scope off(0);
    const std::uint64_t hits = detail::lock_fuzz_hits();
    pass_every_point();
    EXPECT_EQ(detail::lock_fuzz_hits(), hits); // off: no point fuzzed
  }
  {
    fuzz_scope on(42);
    EXPECT_EQ(detail::lock_fuzz_seed(), 42u);
    const std::uint64_t hits = detail::lock_fuzz_hits();
    pass_every_point();
    // after_op (the release), post_enter and post_exit (two posts), drain_between (one), and
    // close_then_detach (two homes): 8 at least
    EXPECT_GE(detail::lock_fuzz_hits() - hits, 8u);
  }
  EXPECT_EQ(detail::lock_fuzz_seed(), 0u);
}

TEST(LockFuzz, PostsRacingACloseNeverLandUnderFuzz) {
  fuzz_scope on(0xC0FFEE);
  const std::uint64_t hits = detail::lock_fuzz_hits();
  run_post_close_races(100);
  EXPECT_GT(detail::lock_fuzz_hits(), hits);
}

// -- the manager mutex's waits (lock.cross_thread) ------------------------------------------------

TEST(LockShared, MutexWaitsCounted) {
  // A single table has no mutex: nothing to count.
  {
    world w;
    const std::uint64_t h1 = w.acquire(1, X, {"a"});
    w.wait(2, X, {"a"});
    w.release(1, h1);
    w.end(2, false);
    EXPECT_EQ(w.t.stats().mutex_waits, 0u);
    EXPECT_EQ(w.t.stats().mutex_wait_ns, 0u);
  }
  lock_manager single;
  EXPECT_EQ(single.stats().mutex_waits, 0u);
  // Two threads on one shared table, each on its own key: they contend only for M.mu, and some
  // operation finds it held sooner or later. A bounded loop, no timing.
  lock_table::options o;
  o.one_home = false;
  o.shared = true;
  lock_table t(o);
  detail::home_id homes[2];
  {
    detail::lock_notes n;
    for (detail::home_id &h : homes)
      h = t.attach_home(false, nullptr, nullptr, n, lock_attach{true});
  }
  std::atomic<bool> seen{false};
  std::atomic<int> started{0};
  const std::vector<std::string> keys[2] = {{"k0"}, {"k1"}};
  const auto driver = [&](int i) {
    ++started;
    while (started < 2)
      std::this_thread::yield();
    detail::lock_notes notes;
    for (int k = 1; k <= 2000000 && !seen; ++k) {
      detail::lock_request r = request_of(X, keys[i]);
      const auto g = t.try_acquire({homes[i], 1}, {"o", 0}, r, {});
      if (g)
        t.release({homes[i], 1}, g->hold, notes);
      notes.clear();
      if (k % 256 == 0 && t.stats().mutex_waits > 0)
        seen = true;
    }
  };
  std::thread a(driver, 0), b(driver, 1);
  a.join();
  b.join();
  const lock_stats s = t.stats();
  std::printf("LockShared: %llu mutex waits, %llu ns blocked\n", (unsigned long long)s.mutex_waits,
              (unsigned long long)s.mutex_wait_ns);
  EXPECT_GT(s.mutex_waits, 0u);
  EXPECT_TRUE(t.empty());
  t.check();
}

// -- review fixes: detached homes (R3b F1) --------------------------------------------------------

TEST(LockTable, DetachedHomesLeaveNoEntry) {
  // A manager outlives its homes (every pycvc Exec, every Ariadne init script attaches one), so a
  // detached home leaves nothing behind: its entry goes with it, and no operation walks the homes
  // that came before. Ids are still never reused (protocol 2.1).
  lock_table::options o;
  o.self_check = true;
  o.one_home = false;
  lock_table t(o);
  detail::lock_notes n;
  const detail::home_id first = t.attach_home(false, nullptr, nullptr, n); // lives throughout
  const std::vector<std::string> w{"w"}, k{"k"};
  detail::lock_request rw = request_of(X, w);
  const auto hw = t.try_acquire({first, 1}, {"o", 0}, rw, {});
  ASSERT_TRUE(hw);
  for (int i = 1; i <= 10000; ++i) {
    const detail::home_id h = t.attach_home(i % 3 == 0, nullptr, nullptr, n);
    ASSERT_EQ(h, detail::home_id(i)); // never reused
    if (i % 500 == 0) {
      // Now and then the home goes holding a key, waiting, and with a cache reference.
      detail::lock_request rk = request_of(S, k);
      ASSERT_TRUE(t.try_acquire({h, 1}, {"o", 0}, rk, {}));
      detail::lock_request rq = request_of(X, w);
      ASSERT_EQ(t.block({h, 2}, {"o", -1}, rq, {}, n).state, ticket_state::queued);
      std::vector<detail::key_ref> refs{detail::key_ref{nullptr, "c", false}};
      t.ref_nodes(h, refs);
    }
    t.detach_home(h, n);
    ASSERT_EQ(t.home_entries(), 1u) << "after home " << h;
  }
  t.release({first, 1}, hw->hold, n);
  EXPECT_TRUE(t.empty());
  EXPECT_EQ(t.stats().cache_refs, 0u);
  EXPECT_EQ(t.stats().trie_nodes, 1u);
  EXPECT_NE(t.canonical().find("homes\nH0 prio=0\nrecs\n"), std::string::npos);
  t.check();
  EXPECT_EQ(t.attach_home(false, nullptr, nullptr, n), 10001u); // the next id is still fresh
  EXPECT_EQ(t.home_entries(), 2u);
  t.check();
}

TEST(LockTable, LendingFollowsTheAttachedPrioHomes) {
  // lending (protocol 2.1) holds exactly while a home that applies priorities is attached, through
  // attaches and detaches in any order.
  lock_table::options o;
  o.self_check = true;
  o.one_home = false;
  lock_table t(o);
  detail::lock_notes n;
  const std::vector<std::string> a{"a"};
  const detail::home_id rr = t.attach_home(false, nullptr, nullptr, n);
  detail::lock_request r = request_of(X, a);
  ASSERT_TRUE(t.try_acquire({rr, 1}, {"o", 5}, r, {})); // the holder, base 5
  const auto waiter = [&](detail::home_id h, int base) {
    detail::lock_request q = request_of(X, a);
    EXPECT_EQ(t.block({h, 1}, {"o", base}, q, {}, n).state, ticket_state::queued);
  };
  const auto inh = [&] { return t.rec_of({rr, 1})->inh; };
  const detail::home_id p1 = t.attach_home(true, nullptr, nullptr, n);
  waiter(p1, -5);
  EXPECT_EQ(inh(), -5); // P1's waiter lends to the holder
  t.detach_home(p1, n);
  EXPECT_EQ(inh(), detail::lock_inf); // no lending home left
  const detail::home_id p2 = t.attach_home(true, nullptr, nullptr, n);
  const detail::home_id p3 = t.attach_home(true, nullptr, nullptr, n);
  waiter(p2, -3);
  EXPECT_EQ(inh(), -3);
  t.detach_home(p3, n);
  EXPECT_EQ(inh(), -3); // P2 still lends
  // A round-robin home's waiter lends nothing.
  waiter(t.attach_home(false, nullptr, nullptr, n), -9);
  EXPECT_EQ(inh(), -3);
  t.detach_home(p2, n);
  EXPECT_EQ(inh(), detail::lock_inf);
  t.check();
}

// -- review fixes: the host's attach (R3b F4) -----------------------------------------------------

TEST(LockShared, ThreadedHostAttach) {
  // What pycvc's Exec and the wasm smoke attach with: cross-thread where threads exist (a shared
  // manager takes it, a single one refuses it), a same-thread attach on a build without them.
  static_assert(threaded_host_attach().cross_thread == threads_compiled_in);
  fake_ops ops;
  const std::shared_ptr<lock_manager> shared = make_manager(lock_threading::shared);
  {
    detail::lock_home h{shared, ops, scheduling_policy::round_robin, nullptr,
                        threaded_host_attach()};
    EXPECT_EQ(h.id(), 0u);
  }
  const std::shared_ptr<lock_manager> single = make_manager(lock_threading::single);
  if constexpr (threads_compiled_in) {
    EXPECT_THROW((detail::lock_home{single, ops, scheduling_policy::round_robin, nullptr,
                                    threaded_host_attach()}),
                 std::logic_error);
  } else {
    detail::lock_home h{single, ops, scheduling_policy::round_robin, nullptr,
                        threaded_host_attach()};
    EXPECT_EQ(h.id(), 0u);
  }
  EXPECT_TRUE(shared->table().empty());
  EXPECT_TRUE(single->table().empty());
}

// -- review fixes: a grant the mirror never took (R3a F2) -----------------------------------------

TEST(LockHomes, UnwoundWaitGrantedByAnotherHomeIsReleased) {
  // 5.4's defensive path: a lock_enter frame unwinds with its ticket still set (an exception in
  // the step that queued it), and another home granted the ticket meanwhile. The cancel finds it
  // granted; the grant never reached the mirror, so the unwind gives it back.
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb = b->add(2);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb, X, {"k"}));
    a->release(pa, *ha); // grants pb's ticket on A's thread; the wake waits in B's port
    ASSERT_TRUE(b->home->port_pending());
    ASSERT_EQ(mgr->stats().holds, 1u);
    b->home->release_frame(pb, std::nullopt, true); // the unwind, before B drains
    EXPECT_EQ(mgr->stats().holds, 0u);
    EXPECT_FALSE(pb.lock_waiting);
    EXPECT_TRUE(pb.locks->holds.empty());
    EXPECT_FALSE(pb.locks->ticket);
    b->home->drain_port(); // the wake is stale now: ignored
    EXPECT_TRUE(b->ops.readies.empty());
    const std::optional<std::uint64_t> ha2 = a->take(pa, X, {"k"}); // k is free
    ASSERT_TRUE(ha2);
    a->release(pa, *ha2);
    check_all(*mgr, {a.get(), b.get()});
    a.reset();
    b.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

TEST(LockHomes, ProcessEndedEndsARecordTheMirrorMissed) {
  // A process that has reached the manager is ended there when it ends, whatever its mirror
  // says, so a record the mirror missed (a grant whose collect failed to record it) can't outlive
  // the process and keep its keys.
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    const std::optional<std::uint64_t> h = a->take(pa, X, {"k"});
    ASSERT_TRUE(h);
    a->release(pa, *h);
    // A hold of pa's that its mirror doesn't have.
    const std::vector<std::string> k{"k"};
    detail::lock_request r = request_of(X, k);
    ASSERT_TRUE(mgr->table().try_acquire({a->home->id(), pa.pid}, {"o", 0}, r, {}));
    ASSERT_TRUE(pa.locks && pa.locks->holds.empty() && !pa.locks->ticket);
    a->ops.procs.set_status(pa, process_status::terminated);
    a->home->process_ended(pa, false);
    EXPECT_EQ(mgr->stats().holds, 0u);
    EXPECT_EQ(mgr->stats().leaked_at_exit, 0u); // the mirror held nothing
    mgr->check();
    a.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

// -- review fixes: a throw while delivering (R3a) -------------------------------------------------

TEST(LockHomes, DeliverFinishesAfterAThrow) {
  // An exception applying one note (here from the scheduler's ready_lock_waiter) doesn't drop the
  // notes after it: a dropped wake would leave its granted waiter parked for ever.
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    process &p0 = a->add(1);
    process &p1 = a->add(2);
    process &p2 = a->add(3);
    const std::optional<std::uint64_t> h0 = a->take(p0, X, {"k1", "k2"});
    ASSERT_TRUE(h0);
    ASSERT_TRUE(a->wait(p1, X, {"k1"}));
    ASSERT_TRUE(a->wait(p2, X, {"k2"}));
    a->ops.throw_ready_for = 2;
    a->release(p0, *h0); // two wakes, this home's: p1's apply throws, p2's still runs
    EXPECT_EQ(a->ops.readies, (std::vector<int>{3}));
    EXPECT_FALSE(p2.lock_waiting);
    EXPECT_NE(sched_side::last_hold(p2), 0u);
    a->ops.throw_ready_for = -1;
    a->release(p1, sched_side::last_hold(p1));
    a->release(p2, sched_side::last_hold(p2));
    EXPECT_EQ(mgr->stats().holds, 0u);
    a.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

TEST(LockHomes, DrainFinishesAfterAThrow) {
  // drain_port applies every post it took, then rethrows the first error.
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb1 = b->add(2);
    process &pb2 = b->add(3);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k1", "k2"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb1, X, {"k1"}));
    ASSERT_TRUE(b->wait(pb2, X, {"k2"}));
    a->release(pa, *ha); // two wakes in B's port
    b->ops.throw_ready_for = 2;
    EXPECT_THROW(b->home->drain_port(), std::runtime_error);
    EXPECT_EQ(b->ops.readies, (std::vector<int>{3}));
    EXPECT_FALSE(pb2.lock_waiting);
    EXPECT_FALSE(b->home->port_pending());
    b->ops.throw_ready_for = -1;
    b->release(pb1, sched_side::last_hold(pb1));
    b->release(pb2, sched_side::last_hold(pb2));
    EXPECT_EQ(mgr->stats().holds, 0u);
    a.reset();
    b.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

// -- second round: blocking_sinks from several waiters --------------------------------------------

namespace {

/// What blocking_sinks(from) must be: the union of the single-pid results, less the sources.
sinks_t union_of_singles(const lock_table &t, const std::vector<detail::lock_pid> &from) {
  std::map<std::uint64_t, std::pair<detail::lock_pid, std::int64_t>> u;
  for (detail::lock_pid p : from)
    for (const auto &s : t.blocking_sinks(p))
      u.emplace(s.first.packed(), s);
  for (detail::lock_pid p : from)
    u.erase(p.packed());
  sinks_t out;
  for (const auto &kv : u)
    out.push_back(kv.second);
  return out;
}

/// blocking_sinks(from) on `t`: equal to union_of_singles, and its pids.
std::vector<int> multi_sinks(const lock_table &t, const std::vector<detail::lock_pid> &from) {
  const sinks_t multi =
      t.blocking_sinks(std::span<const detail::lock_pid>(from.data(), from.size()));
  EXPECT_EQ(multi, union_of_singles(t, from));
  return sink_pids(multi);
}

} // namespace

TEST(LockTable, BlockingSinksFromSeveralWaiters) {
  {
    // A chain: 1 waits for 2, which waits for 3.
    world w;
    w.acquire(3, X, {"c"});
    w.acquire(2, X, {"b"});
    w.wait(2, X, {"c"});
    w.wait(1, X, {"b"});
    EXPECT_EQ(multi_sinks(w.t, {w.pid(1), w.pid(2)}), (std::vector<int>{3}));
    EXPECT_EQ(multi_sinks(w.t, {w.pid(1)}), (std::vector<int>{3}));
    EXPECT_EQ(multi_sinks(w.t, {w.pid(2), w.pid(1), w.pid(2)}), (std::vector<int>{3}));
    EXPECT_TRUE(multi_sinks(w.t, {}).empty());
  }
  {
    // Two shared holders, two writers waiting.
    world w;
    w.acquire(1, S, {"k"});
    w.acquire(2, S, {"k.x"});
    w.wait(3, X, {"k"});
    w.wait(4, X, {"k"});
    EXPECT_EQ(multi_sinks(w.t, {w.pid(3), w.pid(4)}), (std::vector<int>{1, 2}));
  }
  {
    // A long FIFO queue on one key: every waiter waits for the holder and the waiters before it;
    // the one sink is the holder. (The reference, one search per waiter, is the cubic cost the
    // single search removes: 100 waiters keep it quick under the sanitizers.)
    world w(0, false); // no self_check: each check() would be a full recount
    w.acquire(1, X, {"k"});
    std::vector<detail::lock_pid> from;
    for (int p = 2; p <= 101; ++p) {
      w.wait(p, X, {"k"});
      from.push_back(w.pid(p));
    }
    EXPECT_EQ(multi_sinks(w.t, from), (std::vector<int>{1}));
  }
  {
    // Sources that wait for each other: 3 waits for 2, which waits for 1.
    world w;
    w.acquire(1, X, {"a"});
    w.acquire(2, X, {"b"});
    w.wait(2, X, {"a"});
    w.wait(3, X, {"b"});
    EXPECT_EQ(multi_sinks(w.t, {w.pid(3), w.pid(2)}), (std::vector<int>{1}));
    // A source with no edge out is a sink of the others, but never in the result.
    EXPECT_TRUE(multi_sinks(w.t, {w.pid(1), w.pid(2), w.pid(3)}).empty());
  }
}

TEST(LockHomes, BlockingSinksOfSeveralWaitersOfAHome) {
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb1 = b->add(1);
    process &pb2 = b->add(2);
    const std::optional<std::uint64_t> ha = a->take(pa, X, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb1, X, {"k"}));
    ASSERT_TRUE(b->wait(pb2, S, {"k"}));
    a->home->store_progress(detail::lock_clock::time_point(detail::lock_clock::duration(777)));
    const std::vector<int> pids{pb1.pid, pb2.pid};
    EXPECT_EQ(b->home->blocking_sinks(std::span<const int>(pids.data(), pids.size())),
              (sinks_t{{{a->home->id(), 1}, 777}}));
    a->release(pa, *ha);
    b->home->drain_port();
    b->release(pb1, sched_side::last_hold(pb1));
    b->home->drain_port();
    b->release(pb2, sched_side::last_hold(pb2));
    check_all(*mgr, {a.get(), b.get()});
    a.reset();
    b.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}

// -- second round: home ids never wrap ------------------------------------------------------------

TEST(LockTable, HomeIdsNeverWrap) {
  // Ids are never reused (protocol 2.1), and a home_id has 32 bits: past the last one, attach_home
  // refuses rather than wrapping onto 0, which a live home may hold (and the entries' binary
  // search assumes ascending ids). options::first_home starts the count near the end.
  constexpr detail::home_id last = std::numeric_limits<detail::home_id>::max();
  lock_table::options o;
  o.self_check = true;
  o.one_home = false;
  o.first_home = last - 1;
  lock_table t(o);
  detail::lock_notes n;
  const detail::home_id a = t.attach_home(false, nullptr, nullptr, n);
  const detail::home_id b = t.attach_home(true, nullptr, nullptr, n);
  EXPECT_EQ(a, last - 1);
  EXPECT_EQ(b, last);
  try {
    t.attach_home(false, nullptr, nullptr, n);
    ADD_FAILURE() << "an attach past the last home id";
  } catch (const std::logic_error &e) {
    EXPECT_NE(std::string(e.what()).find("home id"), std::string::npos) << e.what();
  }
  t.detach_home(a, n); // a freed id is never handed out again
  EXPECT_THROW(t.attach_home(false, nullptr, nullptr, n), std::logic_error);
  EXPECT_EQ(t.home_entries(), 1u);
  t.check();
  // The last id works as any other: a process of it locks and releases.
  const std::vector<std::string> k{"k"};
  detail::lock_request r = request_of(X, k);
  const auto g = t.try_acquire({b, 7}, {"o", 0}, r, {});
  ASSERT_TRUE(g);
  EXPECT_TRUE(t.rec_of({b, 7}).has_value());
  t.release({b, 7}, g->hold, n);
  t.detach_home(b, n);
  EXPECT_TRUE(t.empty());
  EXPECT_EQ(t.home_entries(), 0u);
  t.check();
}

// -- second round: the unwound wait's notes in order ----------------------------------------------

namespace {

/// What the cross_home_note hook saw of the unwinding process's ticket when the cancel's note
/// went out.
struct cancel_probe {
  const lock_table *table = nullptr;
  std::uint64_t tid = 0;
  int notes = 0;
  bool ticket_there = false;
};
cancel_probe g_cancel_probe;
void probe_cancel_note(const detail::lock_note &) {
  ++g_cancel_probe.notes;
  g_cancel_probe.ticket_there = g_cancel_probe.table->ticket_of(g_cancel_probe.tid).has_value();
}

} // namespace

TEST(LockHomes, UnwoundWaitDeliversTheCancelsNotesFirst) {
  // 5.4's defensive path delivers the cancel's notes (the wakes of the waiters the cancel lets
  // in) right after the cancel, before its resume and any release, so neither can lose them.
  for (const lock_threading mode : {lock_threading::single, lock_threading::shared}) {
    SCOPED_TRACE(mode_name(mode));
    const std::shared_ptr<lock_manager> mgr = make_manager(mode);
    auto a = std::make_unique<sched_side>(mgr);
    auto b = std::make_unique<sched_side>(mgr);
    auto c = std::make_unique<sched_side>(mgr);
    process &pa = a->add(1);
    process &pb = b->add(2);
    process &pc = c->add(3);
    const std::optional<std::uint64_t> ha = a->take(pa, S, {"k"});
    ASSERT_TRUE(ha);
    ASSERT_TRUE(b->wait(pb, X, {"k"})); // behind A's S hold
    ASSERT_TRUE(c->wait(pc, S, {"k"})); // behind B's earlier X ticket (FIFO)
    g_cancel_probe = cancel_probe{&mgr->table(), *pb.locks->ticket};
    detail::lock_test_hooks hooks;
    hooks.cross_home_note = probe_cancel_note;
    detail::set_lock_test_hooks(&hooks);
    b->home->release_frame(pb, std::nullopt, true); // the cancel lets C's waiter in
    detail::set_lock_test_hooks(nullptr);
    EXPECT_EQ(g_cancel_probe.notes, 1);
    EXPECT_TRUE(g_cancel_probe.ticket_there); // not resumed yet when the wake went out
    EXPECT_FALSE(pb.locks->ticket);
    c->home->drain_port();
    EXPECT_EQ(c->ops.readies, (std::vector<int>{3}));
    c->release(pc, sched_side::last_hold(pc));
    a->release(pa, *ha);
    check_all(*mgr, {a.get(), b.get(), c.get()});
    a.reset();
    b.reset();
    c.reset();
    EXPECT_TRUE(mgr->table().empty());
  }
}
