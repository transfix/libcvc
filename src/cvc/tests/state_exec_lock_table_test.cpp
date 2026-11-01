/// @file state_exec_lock_table_test.cpp
/// @brief Directed cases of the lock manager's table on one home, no scheduler (suite LockTable;
///        locks plan L1.2). Compiled into state_exec_locks_model_test beside the model check.
///
/// Each case drives detail::lock_table as a lock home would (protocol section 4), on a table built
/// with options::self_check on, and calls check() after every operation. The case names follow
/// the design's test list (docs/STATE_EXEC_LOCKS.md section 13) where a case is the manager-level
/// shape of one of its tests.

#include <cvc/state/state_exec/detail/lock_table.h>
#include <gtest/gtest.h>
#include <initializer_list>
#include <string>
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

// -- 3.3, 4.3, 4.12, 4.13: inheritance (phase L2) ------------------------------------------------

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
