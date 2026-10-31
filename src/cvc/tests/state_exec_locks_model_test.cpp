/// @file state_exec_locks_model_test.cpp
/// @brief The locks model check, LT-5 (docs/STATE_EXEC_LOCKS_PROTOCOL.md section 7; locks plan
///        L1.3): a spec oracle, a world and an explorer over the real detail::lock_table.
///
/// Written from the protocol and tools/locks_model/ (the reviewed Python model), never from the
/// table's code. The parts, each a port of a Python file:
///
///   1. spec_manager (lockmodel.py's Manager): protocol 2-4 literally, over sets and maps (no
///      trie), with lock_table's mutation bits, its own check() of section 6's manager clauses,
///      and canonical() in lock_table.h's format.
///   2. lockstep: the oracle and the real lock_table side by side. Every operation runs on both,
///      as a lock home calls the table (key_ref and cache_delta from the home's key cache); the
///      results, the notification lists and the canonical dumps must be equal, then both check()s
///      run. The real side can be switched off (CVC_LOCKS_MODEL_REAL=0): the oracle alone.
///   3. world (world.py): homes, processes, frames, key caches, ports and posts, the per-action
///      micro-operations of protocol 5, and the home-side and transition invariants.
///   4. explore() (explore.py): 0-1 BFS over fingerprints of the world's state, each state kept as
///      (parent, action) and rebuilt by replaying its path into a fresh world; a fair drain at
///      the bound; the shortest trace per violation.
///   5. Random runs (random_runs.py on one home), scenarios.py's scenarios on one home (the
///      stale-wake ones on two), the L1 and L2 mutations, and the replay of the Python model's
///      traces (tools/locks_model/export_traces.py, src/cvc/tests/data/locks_model_traces/).
///
/// The configurations: 'l1' and 'l1x' (phase L1, one round_robin home), 'l2' and 'noinh1' (phase
/// L2, one priority_rr home with set-priority, inheritance on and off), and explore.py's 'core'
/// on the oracle alone. Sizes come from CVC_LOCKS_MODEL_BOUND, CVC_LOCKS_MODEL_STEPS and
/// CVC_LOCKS_SANITIZER; see tools/locks_model/README.md.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
// windows.h first
#include <psapi.h>
#else
#include <sys/resource.h>
#endif

using namespace cvc::state_exec;

namespace {

namespace dt = cvc::state_exec::detail;
using dt::home_id;
using dt::lock_pid;
using dt::lock_violation;
using dt::ticket_state;

constexpr int INF = dt::lock_inf;
constexpr std::uint64_t SEQ_INF = UINT64_MAX;
constexpr lock_mode S = lock_mode::shared;
constexpr lock_mode X = lock_mode::exclusive;

// =============================================================================================
// 0. Basics: violations, keys, formatting
// =============================================================================================

/// Every finding is a lock_violation named as the Python model names it ("I6", "4.13",
/// "lockstep", "require", "model", ...), so a test can require exactly the one it expects.
[[noreturn]] void fail(const std::string &name, const std::string &detail) {
  throw lock_violation(name, detail);
}

/// The real table is still the interface commit's stub (it throws "not built yet").
struct not_built : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// A scenario's own assertion failed (scenarios.py's `assert`).
struct scenario_failure : std::runtime_error {
  using std::runtime_error::runtime_error;
};

#define MODEL_ASSERT(cond, msg)                                                                    \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::ostringstream os_;                                                                      \
      os_ << "line " << __LINE__ << ": " << #cond << ": " << msg;                                  \
      throw scenario_failure(os_.str());                                                           \
    }                                                                                              \
  } while (0)

bool is_stub(const std::exception &e) { return std::strstr(e.what(), "not built yet") != nullptr; }

struct key_cmp {
  bool operator()(const std::string &a, const std::string &b) const { return dt::key_less(a, b); }
};
using key_set = std::set<std::string, key_cmp>;
using key_list =
    std::vector<std::string>; // canonical: sorted by key_less, no key a prefix of another

bool prefix(const std::string &a, const std::string &b) { return dt::key_prefix(a, b); }
bool overlap(const std::string &a, const std::string &b) { return prefix(a, b) || prefix(b, a); }
bool compat(lock_mode m, lock_mode n) { return m == S && n == S; }
bool conflicts(const key_list &ak, lock_mode am, const key_list &bk, lock_mode bm) {
  if (am == S && bm == S)
    return false;
  for (const auto &a : ak)
    for (const auto &b : bk)
      if (overlap(a, b))
        return true;
  return false;
}
bool under(const key_list &hkeys, const std::string &k) {
  for (const auto &hk : hkeys)
    if (prefix(hk, k))
      return true;
  return false;
}
bool contains(const key_list &ks, const std::string &k) {
  return std::find(ks.begin(), ks.end(), k) != ks.end();
}

/// Protocol 1's canonical request: sorted by segment tuples, every key with an earlier prefix
/// (equal or ancestor) dropped.
key_list canonical_keys(key_list keys) {
  std::sort(keys.begin(), keys.end(), key_cmp{});
  key_list out;
  for (const auto &k : keys) {
    bool covered = false;
    for (const auto &o : out)
      covered = covered || prefix(o, k);
    if (!covered)
      out.push_back(k);
  }
  return out;
}

key_list split(const std::string &s, char sep) {
  key_list out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

char mode_char(lock_mode m) { return m == S ? 'S' : 'X'; }
std::string pid_str(lock_pid p) {
  return "(" + std::to_string(p.home) + "," + std::to_string(p.pid) + ")";
}
std::string inh_str(int v) { return v == INF ? "INF" : std::to_string(v); }
const char *state_name(ticket_state s) {
  switch (s) {
  case ticket_state::queued:
    return "queued";
  case ticket_state::granted:
    return "granted";
  case ticket_state::cancelled:
    return "cancelled";
  case ticket_state::failed:
    return "failed";
  }
  return "?";
}
template <class C> std::string join_u64(const C &c) {
  std::string o = "[";
  bool first = true;
  for (auto v : c) {
    if (!first)
      o += ",";
    first = false;
    o += std::to_string(v);
  }
  return o + "]";
}
template <class C> std::string keys_br(const C &keys) {
  std::string o = "[";
  bool first = true;
  for (const auto &k : keys) {
    if (!first)
      o += ",";
    first = false;
    o += k;
  }
  return o + "]";
}
/// A key list as one token: "a.b,b", "-" when empty (the trace format).
template <class C> std::string keys_tok(const C &keys) {
  std::string o;
  for (const auto &k : keys) {
    if (!o.empty())
      o += ",";
    o += k;
  }
  return o.empty() ? "-" : o;
}
key_list parse_keys_tok(const std::string &t) {
  if (t == "-")
    return {};
  return split(t, ',');
}
std::string req_str(const key_list &keys, lock_mode mode) {
  return std::string(1, mode_char(mode)) + "{" + keys_tok(keys) + "}";
}

// =============================================================================================
// 1. The spec oracle: lockmodel.py's Manager, protocol 2-4
// =============================================================================================

/// One edge of a DeadlockError's cycle (4.6), with its source process (the Python model keeps it
/// for its ghost check of the cycle; the table's deadlock_edge has only pid and key).
struct spec_edge {
  std::optional<int> pid;
  std::string key; // empty: no key overlaps (the ghost check reports it, I7)
  lock_pid from;
};

/// A ticket's error (2.1): none, TimeoutError (only under cancel_marks_failed) or DeadlockError.
struct spec_error {
  enum kind_t : std::uint8_t { none, timeout, deadlock } kind = none;
  std::vector<spec_edge> cycle;
};

struct spec_rec {
  std::string owner;
  int base = 0;
  std::set<std::uint64_t> holds;
  std::optional<std::uint64_t> ticket;
  int inh = INF;
  bool rl = false; // 4.3 coalescing: a relevel for this process is in a port
};

struct spec_hold {
  lock_pid pid;
  key_list keys;
  lock_mode mode = X;
  std::uint64_t gen = 0;
  bool started = false;
  key_set taken;
};

struct spec_ticket {
  lock_pid pid;
  key_list keys;
  lock_mode mode = X;
  std::uint64_t seq = 0;
  ticket_state state = ticket_state::queued;
  std::optional<std::uint64_t> hold;
  spec_error error;
};

struct spec_note {
  home_id home = 0;
  dt::lock_post post;
};
using spec_notes = std::vector<spec_note>;

/// resume's outcome (4.7), with the cycle's sources.
struct spec_outcome {
  dt::lock_outcome::kind what = dt::lock_outcome::kind::queued;
  dt::grant g;
  std::vector<spec_edge> cycle;
};

using graph_t = std::map<lock_pid, std::set<lock_pid>>;

class spec_manager {
public:
  using mutation = dt::lock_table::mutation;
  struct options {
    bool inheritance = true;
    bool coalesce = false;
    bool one_home = true;
    std::uint32_t mutations = 0;
  };
  struct home_rec {
    bool prio = false;
  };

  explicit spec_manager(options o) : opts(o) {}

  options opts;
  std::map<home_id, home_rec> homes;
  std::map<lock_pid, spec_rec> recs;
  std::map<std::uint64_t, spec_hold> holds;
  std::map<std::uint64_t, spec_ticket> tickets;
  std::set<std::uint64_t> queue;
  key_set abandoned;
  std::uint64_t next_seq = 0, next_hold = 0, next_gen = 0;
  std::uint64_t grants = 0; // the counters line of canonical(): every grant
  std::uint64_t relevels = 0, relevels_coalesced = 0; // lock_stats' (not in canonical())
  std::map<home_id, std::map<std::string, int, key_cmp>> cache_refs;
  home_id next_home = 0; // attach_home hands out 0, 1, ... (lock_table's home ids)
  // ghosts (lockmodel.py): abandonment reports not yet seen, the last X generation, the largest
  key_set g_pending_report;
  std::uint64_t g_last_x_gen = 0, g_max_gen = 0;

  bool has(std::uint32_t m) const { return (opts.mutations & m) != 0; }

  // -- 2.1 derived -----------------------------------------------------------------------------
  void require(bool c, const std::string &what) const {
    if (!c)
      fail("require", what);
  }
  bool applies(lock_pid P) const {
    auto it = homes.find(P.home);
    return it != homes.end() && it->second.prio;
  }
  bool lends(lock_pid P) const { return opts.inheritance && applies(P); }
  bool lending() const {
    for (const auto &kv : homes)
      if (opts.inheritance && kv.second.prio)
        return true;
    return false;
  }

  // -- 3.1 blocking ------------------------------------------------------------------------------
  std::vector<std::uint64_t> blk_hold(lock_pid P, const key_list &keys, lock_mode mode) const {
    std::vector<std::uint64_t> out;
    for (const auto &[i, h] : holds)
      if (h.pid != P && conflicts(keys, mode, h.keys, h.mode))
        out.push_back(i);
    return out;
  }
  bool exempt(lock_pid P, const spec_ticket &U) const {
    if (has(mutation::no_exemption))
      return false;
    for (const auto &[i, h] : holds)
      if (h.pid == P && conflicts(U.keys, U.mode, h.keys, h.mode))
        return true;
    return false;
  }
  std::vector<std::uint64_t> blk_wait(lock_pid P, const key_list &keys, lock_mode mode,
                                      std::uint64_t s) const {
    std::vector<std::uint64_t> out;
    for (auto t : queue) {
      const auto &U = tickets.at(t);
      if (U.seq < s && U.pid != P && conflicts(keys, mode, U.keys, U.mode) && !exempt(P, U))
        out.push_back(t);
    }
    return out;
  }
  bool grantable(lock_pid P, const key_list &keys, lock_mode mode, std::uint64_t s) const {
    return blk_hold(P, keys, mode).empty() && blk_wait(P, keys, mode, s).empty();
  }
  bool grantable_t(const spec_ticket &T) const { return grantable(T.pid, T.keys, T.mode, T.seq); }

  // -- 3.2 the wait-for graph
  // ----------------------------------------------------------------------
  std::set<lock_pid> succ_of(lock_pid P) const {
    auto r = recs.find(P);
    if (r == recs.end() || !r->second.ticket)
      return {};
    const auto &T = tickets.at(*r->second.ticket);
    if (T.state != ticket_state::queued)
      return {};
    std::set<lock_pid> out;
    for (auto i : blk_hold(P, T.keys, T.mode))
      out.insert(holds.at(i).pid);
    for (auto t : blk_wait(P, T.keys, T.mode, T.seq))
      out.insert(tickets.at(t).pid);
    return out;
  }
  graph_t graph() const {
    graph_t g;
    for (const auto &[P, r] : recs)
      if (r.ticket && tickets.at(*r.ticket).state == ticket_state::queued)
        g[P] = succ_of(P);
    return g;
  }
  static std::set<lock_pid> reach_in(const graph_t &g, const std::set<lock_pid> &s0) {
    std::set<lock_pid> seen = s0;
    std::vector<lock_pid> stack(s0.begin(), s0.end());
    while (!stack.empty()) {
      auto a = stack.back();
      stack.pop_back();
      auto it = g.find(a);
      if (it == g.end())
        continue;
      for (auto b : it->second)
        if (seen.insert(b).second)
          stack.push_back(b);
    }
    return seen;
  }
  /// A cycle of g, if any (I7).
  static std::optional<std::vector<lock_pid>> find_cycle(const graph_t &g) {
    std::map<lock_pid, int> col; // 0 white, 1 grey, 2 black
    std::vector<lock_pid> path;
    std::optional<std::vector<lock_pid>> found;
    std::function<bool(lock_pid)> dfs = [&](lock_pid a) -> bool {
      col[a] = 1;
      path.push_back(a);
      auto it = g.find(a);
      if (it != g.end())
        for (auto b : it->second) {
          int c = col.count(b) ? col[b] : 0;
          if (c == 1) {
            auto at = std::find(path.begin(), path.end(), b);
            found = std::vector<lock_pid>(at, path.end());
            found->push_back(b);
            return true;
          }
          if (c == 0 && dfs(b))
            return true;
        }
      col[a] = 2;
      path.pop_back();
      return false;
    };
    for (const auto &kv : g)
      if (!col.count(kv.first) && dfs(kv.first))
        return found;
    return std::nullopt;
  }

  /// Every process within `depth` edges downstream of s0 in g, and s0 (lockmodel.py's
  /// reach_depth).
  static std::set<lock_pid> reach_depth(const graph_t &g, const std::set<lock_pid> &s0, int depth) {
    std::set<lock_pid> seen = s0;
    std::vector<lock_pid> frontier(s0.begin(), s0.end());
    for (int d = 0; d < depth; ++d) {
      std::vector<lock_pid> nxt;
      for (auto a : frontier) {
        auto it = g.find(a);
        if (it == g.end())
          continue;
        for (auto b : it->second)
          if (seen.insert(b).second)
            nxt.push_back(b);
      }
      frontier = std::move(nxt);
    }
    return seen;
  }

  // -- 3.3 inheritance ---------------------------------------------------------------------------
  using pred_t = std::map<lock_pid, std::set<lock_pid>>;
  static pred_t pred_of(const graph_t &g) {
    pred_t pred;
    for (const auto &[a, bs] : g)
      for (auto b : bs)
        pred[b].insert(a);
    return pred;
  }
  std::set<lock_pid> anc8(lock_pid x, const pred_t &pred) const {
    std::set<lock_pid> seen;
    std::vector<lock_pid> frontier{x};
    for (int d = 0; d < 8; ++d) {
      std::vector<lock_pid> nxt;
      for (auto a : frontier) {
        auto it = pred.find(a);
        if (it == pred.end())
          continue;
        for (auto y : it->second)
          if (!seen.count(y) && y != x) {
            seen.insert(y);
            nxt.push_back(y);
          }
      }
      frontier = std::move(nxt);
    }
    return seen;
  }
  int inh_formula(lock_pid x, const pred_t &pred, bool lend) const {
    if (!lend)
      return INF;
    int best = INF;
    for (auto y : anc8(x, pred))
      if (lends(y))
        best = std::min(best, recs.at(y).base);
    return best;
  }
  /// 3.3 by brute force, for check(): every path backwards from each process, 1 to 8 edges.
  std::map<lock_pid, int> inh_reference(const graph_t &g, bool lend) const {
    std::map<lock_pid, std::vector<lock_pid>> pred;
    for (const auto &[a, bs] : g)
      for (auto b : bs)
        pred[b].push_back(a);
    std::map<lock_pid, int> out;
    for (const auto &[x, r] : recs) {
      int best = INF;
      if (lend) {
        std::set<lock_pid> anc;
        std::vector<std::pair<lock_pid, int>> stack{{x, 0}};
        while (!stack.empty()) {
          auto [n, d] = stack.back();
          stack.pop_back();
          if (d == 8)
            continue;
          auto it = pred.find(n);
          if (it == pred.end())
            continue;
          for (auto y : it->second) {
            if (y != x)
              anc.insert(y);
            stack.push_back({y, d + 1});
          }
        }
        for (auto y : anc)
          if (opts.inheritance && homes.count(y.home) && homes.at(y.home).prio)
            best = std::min(best, recs.at(y).base);
      }
      out[x] = best;
    }
    return out;
  }

  // -- 4.15 key-cache references
  // ---------------------------------------------------------------------
  void ref(home_id H, const std::string &k) {
    require(homes.count(H) && cache_refs.count(H),
            "ref: H" + std::to_string(H) + " is not attached");
    ++cache_refs[H][k];
  }
  void unref(home_id H, const std::string &k) {
    auto d = cache_refs.find(H);
    require(d != cache_refs.end() && d->second.count(k) && d->second[k] > 0,
            "unref: no reference of H" + std::to_string(H) + " on " + k);
    if (--d->second[k] == 0)
      d->second.erase(k);
    if (d->second.empty() && has(mutation::unref_pops_entry))
      cache_refs.erase(d); // the rev-4 reading: I9 must catch it
  }
  void apply_refs(home_id H, const key_list &add, const key_list &drop) {
    for (const auto &k : add)
      ref(H, k);
    for (const auto &k : drop)
      unref(H, k);
  }

  // -- 4.1 helpers
  // -------------------------------------------------------------------------------------
  void ensure_rec(lock_pid P, const std::string &owner, int base) {
    if (!recs.count(P)) {
      spec_rec r;
      r.owner = owner;
      r.base = base;
      recs.emplace(P, std::move(r));
    }
  }
  std::uint64_t new_hold(lock_pid P, const key_list &keys, lock_mode mode, bool started,
                         std::uint64_t s) {
    // ghost I1 and I5 at the moment of the grant (gap G-14)
    if (!blk_hold(P, keys, mode).empty())
      fail("I1", "hold created for " + pid_str(P) + " " + req_str(keys, mode) +
                     " while another process holds a conflicting hold");
    for (auto t : queue) {
      const auto &U = tickets.at(t);
      if (U.seq < s && U.pid != P && conflicts(keys, mode, U.keys, U.mode) && !exempt(P, U))
        fail("I5", "hold for " + pid_str(P) + " " + req_str(keys, mode) + " (seq " +
                       (s == SEQ_INF ? std::string("INF") : std::to_string(s)) +
                       ") granted ahead of earlier conflicting waiter " + pid_str(U.pid) + " seq " +
                       std::to_string(U.seq));
    }
    std::uint64_t hid = ++next_hold;
    std::uint64_t gen = 0;
    key_set taken;
    if (mode == X) {
      gen = ++next_gen;
      for (const auto &k : keys)
        if (abandoned.count(k))
          taken.insert(k);
      for (const auto &k : taken)
        abandoned.erase(k);
    } else {
      gen = next_gen;
    }
    // ghost I15
    if (mode == X) {
      if (gen <= g_max_gen)
        fail("I15", "X gen " + std::to_string(gen) + " not greater than earlier " +
                        std::to_string(g_max_gen));
      g_last_x_gen = gen;
    } else if (gen != g_last_x_gen) {
      fail("I15",
           "S gen " + std::to_string(gen) + " != last X gen " + std::to_string(g_last_x_gen));
    }
    g_max_gen = std::max(g_max_gen, gen);
    spec_hold h;
    h.pid = P;
    h.keys = keys;
    h.mode = mode;
    h.gen = gen;
    h.started = started;
    h.taken = std::move(taken);
    auto &ref_h = holds[hid] = std::move(h);
    recs.at(P).holds.insert(hid);
    ++grants;
    if (started)
      g_body_start(ref_h);
    return hid;
  }
  void g_body_start(const spec_hold &h) {
    for (const auto &k : h.taken) {
      if (!g_pending_report.count(k))
        fail("I11", "abandoned report for " + k + " seen twice");
      g_pending_report.erase(k);
    }
    if (h.mode == X)
      for (const auto &k : h.keys)
        if (abandoned.count(k))
          fail("I11", "X body on " + k + " started while its abandoned flag is still set");
  }
  key_list drop_hold(std::uint64_t hid, bool abandon) {
    spec_hold h = holds.at(hid);
    if (!h.started) {
      for (const auto &k : h.taken)
        abandoned.insert(k);
    } else if (abandon && h.mode == X) {
      for (const auto &k : h.keys) {
        abandoned.insert(k);
        g_pending_report.insert(k);
      }
    }
    recs.at(h.pid).holds.erase(hid);
    holds.erase(hid);
    return h.keys;
  }
  void maybe_drop_rec(lock_pid P) {
    auto r = recs.find(P);
    if (r != recs.end() && r->second.holds.empty() && !r->second.ticket)
      recs.erase(r);
  }

  // -- operation framing
  // ---------------------------------------------------------------------------------
  void begin() {
    N_.clear();
    start_inh_.clear();
    for (const auto &[P, r] : recs)
      start_inh_[P] = r.inh;
    seeds_.clear();
    lending_before_ = lending();
    // the graph before, for 4.3's implementation note (lockmodel.py takes it in every _begin; it
    // is read only when lending is true before and after)
    start_graph_.clear();
    if (lending_before_)
      start_graph_ = graph();
  }

  // -- 4.2 / 4.3
  // ------------------------------------------------------------------------------------------
  std::set<lock_pid> re_examine(const key_set &region) {
    std::set<lock_pid> G;
    std::vector<std::uint64_t> snapshot(queue.begin(), queue.end());
    for (auto t : snapshot) {
      auto &T = tickets.at(t);
      if (T.state != ticket_state::queued)
        continue;
      if (has(mutation::cancel_reexamines_holds_only) && region_is_ticket_)
        continue;
      bool ov = false;
      for (const auto &k : T.keys)
        for (const auto &r : region)
          ov = ov || overlap(k, r);
      if (!ov)
        continue;
      if (grantable_t(T)) {
        queue.erase(t);
        T.hold = new_hold(T.pid, T.keys, T.mode, false, T.seq);
        T.state = ticket_state::granted;
        N_.push_back({T.pid.home, dt::lock_post{dt::lock_post::kind::wake, T.pid.pid, T.seq}});
        G.insert(T.pid);
      }
    }
    return G;
  }
  void finish(const key_set &region, const std::set<lock_pid> &seeds,
              const std::set<lock_pid> &skip) {
    auto G = re_examine(region);
    std::set<lock_pid> all = seeds;
    all.insert(G.begin(), G.end());
    for (auto P : all)
      maybe_drop_rec(P);
    seeds_.insert(G.begin(), G.end()); // 4.3's note: the processes granted are seeds too
    relevel_all(skip);
  }
  /// 4.3's reference loop, then lockmodel.py's check of the implementation note against it.
  void relevel_all(const std::set<lock_pid> &skip) {
    if (in_cancel_ && has(mutation::cancel_skips_relevel))
      return; // 'no-relevel-cancel'
    bool lend = lending();
    graph_t g;
    pred_t pred;
    if (lend) {
      g = graph();
      pred = pred_of(g);
    }
    std::set<lock_pid> xs;
    for (const auto &kv : start_inh_)
      xs.insert(kv.first);
    for (const auto &kv : recs)
      xs.insert(kv.first);
    std::vector<lock_pid> changed, posted;
    for (auto x : xs) {
      auto si = start_inh_.find(x);
      int old = si == start_inh_.end() ? INF : si->second;
      auto r = recs.find(x);
      int nw = r != recs.end() ? inh_formula(x, pred, lend) : INF;
      if (r != recs.end())
        r->second.inh = nw;
      if (nw == old)
        continue;
      changed.push_back(x);
      if (!skip.count(x) && applies(x)) { // left to right, as 4.3 says
        if (opts.coalesce && r != recs.end() && r->second.rl) {
          ++relevels_coalesced;
          continue;
        }
        if (opts.coalesce && r != recs.end())
          r->second.rl = true;
        N_.push_back({x.home, dt::lock_post{dt::lock_post::kind::relevel, x.pid, 0}});
        posted.push_back(x);
        ++relevels;
      }
    }
    impl_note_check(changed, posted, g, lend);
  }
  /// 4.3's implementation note, its four lending cases, against the reference loop
  /// (lockmodel.py's _impl_note_check).
  void impl_note_check(const std::vector<lock_pid> &changed, const std::vector<lock_pid> &posted,
                       const graph_t &g_after, bool lend) const {
    auto list = [](const std::vector<lock_pid> &v) {
      std::string s;
      for (auto p : v)
        s += (s.empty() ? "" : " ") + pid_str(p);
      return "[" + s + "]";
    };
    if (!lending_before_ && !lend) {
      std::vector<lock_pid> bad = changed;
      for (const auto &[P, r] : recs)
        if (r.inh != INF)
          bad.push_back(P);
      if (!bad.empty())
        fail("impl-note", "lending false before and after, but the reference changed or holds a "
                          "non-INF inh for " +
                              list(bad));
      return;
    }
    if (lending_before_ && !lend) {
      std::vector<lock_pid> bad;
      for (const auto &[P, r] : recs)
        if (r.inh != INF)
          bad.push_back(P);
      if (!bad.empty() || !posted.empty())
        fail("impl-note", "lending true before, false after: reference inh non-INF for " +
                              list(bad) + ", posts " + list(posted) +
                              "; the note sets INF and posts nothing");
      return;
    }
    if (!lending_before_)
      return; // it runs the reference loop over every rec
    std::set<lock_pid> A = reach_depth(g_after, seeds_, 8);
    if (!has(mutation::impl_note_after_only)) {
      auto before = reach_depth(start_graph_, seeds_, 8);
      A.insert(before.begin(), before.end());
    }
    std::vector<lock_pid> missing;
    for (auto x : changed)
      if (!A.count(x))
        missing.push_back(x);
    if (!missing.empty())
      fail("impl-note", "inh of " + list(missing) + " changed but they are not in A (seeds " +
                            list(std::vector<lock_pid>(seeds_.begin(), seeds_.end())) + ")");
  }

  // -- 4.4
  // -----------------------------------------------------------------------------------------------
  home_id attach_home(bool prio, spec_notes &out) {
    begin();
    require(!(opts.one_home && !homes.empty()),
            "attach_home: before L3 a manager accepts one home");
    home_id H = next_home++;
    homes[H] = home_rec{prio};
    cache_refs[H]; // empty; detach_home deletes it
    relevel_all({});
    out = N_;
    return H;
  }

  // -- 4.5
  // -----------------------------------------------------------------------------------------------
  std::optional<dt::grant> try_acquire(lock_pid P, const std::string &owner, int base,
                                       const key_list &keys, lock_mode mode, const key_list &add,
                                       const key_list &drop) {
    begin();
    auto r = recs.find(P);
    require(r == recs.end() || !r->second.ticket, "try_acquire: P has a ticket");
    apply_refs(P.home, add, drop);
    if (!grantable(P, keys, mode, SEQ_INF))
      return std::nullopt;
    ensure_rec(P, owner, base);
    auto hid = new_hold(P, keys, mode, true, SEQ_INF);
    const auto &h = holds.at(hid);
    return dt::grant{hid, h.gen, !h.taken.empty()};
  }

  // -- 4.6
  // -----------------------------------------------------------------------------------------------
  dt::block_result block(lock_pid P, const std::string &owner, int base, const key_list &keys,
                         lock_mode mode, const key_list &add, const key_list &drop,
                         spec_notes &out) {
    begin();
    out.clear();
    auto r = recs.find(P);
    require(r == recs.end() || !r->second.ticket, "block: P has a ticket");
    apply_refs(P.home, add, drop);
    ensure_rec(P, owner, base);
    std::uint64_t s = ++next_seq;
    if (grantable(P, keys, mode, SEQ_INF)) {
      auto hid = new_hold(P, keys, mode, false, s);
      spec_ticket T;
      T.pid = P;
      T.keys = keys;
      T.mode = mode;
      T.seq = s;
      T.state = ticket_state::granted;
      T.hold = hid;
      tickets[s] = std::move(T);
      recs.at(P).ticket = s;
      return {s, ticket_state::granted};
    }
    std::set<lock_pid> targets;
    for (auto i : blk_hold(P, keys, mode))
      targets.insert(holds.at(i).pid);
    for (auto t : blk_wait(P, keys, mode, SEQ_INF))
      targets.insert(tickets.at(t).pid);
    if (reach_in(graph(), targets).count(P)) {
      auto cycle = deadlock_cycle(P, keys, mode, targets);
      g_check_deadlock(P, keys, mode, s, cycle);
      spec_ticket T;
      T.pid = P;
      T.keys = keys;
      T.mode = mode;
      T.seq = s;
      T.state = ticket_state::failed;
      T.error.kind = spec_error::deadlock;
      T.error.cycle = std::move(cycle);
      tickets[s] = std::move(T);
      recs.at(P).ticket = s;
      return {s, ticket_state::failed};
    }
    spec_ticket T;
    T.pid = P;
    T.keys = keys;
    T.mode = mode;
    T.seq = s;
    T.state = ticket_state::queued;
    tickets[s] = std::move(T);
    recs.at(P).ticket = s;
    queue.insert(s);
    seeds_ = {P};
    relevel_all({});
    out = N_;
    return {s, ticket_state::queued};
  }

  std::vector<spec_edge> deadlock_cycle(lock_pid P, const key_list &keys, lock_mode mode,
                                        const std::set<lock_pid> &targets) const {
    std::set<lock_pid> visited;
    std::optional<std::vector<lock_pid>> found;
    std::function<bool(lock_pid, std::vector<lock_pid> &)> dfs =
        [&](lock_pid a, std::vector<lock_pid> &path) -> bool {
      if (visited.count(a))
        return false;
      visited.insert(a);
      path.push_back(a);
      for (auto b : succ_of(a)) { // ascending (home, pid)
        if (b == P) {
          found = path;
          return true;
        }
        if (dfs(b, path))
          return true;
      }
      path.pop_back();
      return false;
    };
    for (auto t : targets) {
      std::vector<lock_pid> path;
      if (dfs(t, path))
        break;
    }
    if (!found)
      fail("I7", "deadlock_cycle found no path back to " + pid_str(P));
    std::vector<lock_pid> nodes{P};
    nodes.insert(nodes.end(), found->begin(), found->end());
    const std::string &owner = recs.at(P).owner;
    std::vector<spec_edge> cyc;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      lock_pid a = nodes[i], b = nodes[(i + 1) % nodes.size()];
      key_list akeys;
      lock_mode amode = mode;
      std::uint64_t aseq = SEQ_INF;
      if (a == P) {
        akeys = keys;
      } else {
        const auto &T = tickets.at(*recs.at(a).ticket);
        akeys = T.keys;
        amode = T.mode;
        aseq = T.seq;
      }
      std::vector<const key_list *> bl;
      for (auto h : blk_hold(a, akeys, amode))
        if (holds.at(h).pid == b)
          bl.push_back(&holds.at(h).keys);
      for (auto t : blk_wait(a, akeys, amode, aseq))
        if (tickets.at(t).pid == b)
          bl.push_back(&tickets.at(t).keys);
      std::string key;
      for (const auto &k : akeys) {
        bool hit = false;
        for (const auto *bk : bl)
          for (const auto &x : *bk)
            hit = hit || overlap(k, x);
        if (hit) {
          key = k;
          break;
        }
      }
      bool same = a.home == P.home && recs.at(a).owner == owner;
      cyc.push_back({same ? std::optional<int>(a.pid) : std::nullopt, key, a});
    }
    return cyc;
  }

  /// Every DeadlockError corresponds to a real cycle at that moment (the model's ghost check).
  void g_check_deadlock(lock_pid P, const key_list &keys, lock_mode mode, std::uint64_t s,
                        const std::vector<spec_edge> &cycle) {
    spec_ticket T;
    T.pid = P;
    T.keys = keys;
    T.mode = mode;
    T.seq = s;
    tickets[s] = T;
    auto old = recs.at(P).ticket;
    recs.at(P).ticket = s;
    queue.insert(s);
    auto restore = [&] {
      queue.erase(s);
      tickets.erase(s);
      recs.at(P).ticket = old;
    };
    try {
      auto g = graph();
      auto gp = g.count(P) ? g.at(P) : std::set<lock_pid>{};
      if (!reach_in(g, gp).count(P))
        fail("I7", "DeadlockError for " + pid_str(P) + " but no cycle through it");
      if (cycle.empty() || cycle[0].from != P)
        fail("I7", "cycle does not start with the requester");
      for (std::size_t i = 0; i < cycle.size(); ++i) {
        lock_pid a = cycle[i].from, b = cycle[(i + 1) % cycle.size()].from;
        if (!g.count(a) || !g.at(a).count(b))
          fail("I7", "cycle edge " + pid_str(a) + " -> " + pid_str(b) + " does not exist");
        if (cycle[i].key.empty())
          fail("I7", "cycle edge " + pid_str(a) + " -> " + pid_str(b) + " has no key");
      }
    } catch (...) {
      restore();
      throw;
    }
    restore();
  }

  // -- 4.7
  // ------------------------------------------------------------------------------------------------
  spec_outcome resume(lock_pid P, std::uint64_t tid) {
    begin();
    auto T = tickets.find(tid);
    auto r = recs.find(P);
    require(T != tickets.end() && T->second.pid == P && r != recs.end() && r->second.ticket &&
                *r->second.ticket == tid,
            "resume: not P's ticket");
    spec_outcome out;
    if (T->second.state == ticket_state::queued)
      return out;
    if (T->second.state == ticket_state::granted) {
      const auto &h = holds.at(*T->second.hold);
      out.what = dt::lock_outcome::kind::granted;
      out.g = dt::grant{*T->second.hold, h.gen, !h.taken.empty()};
    } else if (T->second.state == ticket_state::cancelled) {
      out.what = dt::lock_outcome::kind::timeout;
    } else if (T->second.error.kind == spec_error::timeout) {
      out.what = dt::lock_outcome::kind::timeout;
    } else {
      out.what = dt::lock_outcome::kind::deadlock;
      out.cycle = T->second.error.cycle;
    }
    tickets.erase(T);
    r->second.ticket.reset();
    maybe_drop_rec(P);
    return out;
  }

  // -- 4.8
  // ------------------------------------------------------------------------------------------------
  void start_body(lock_pid P, std::uint64_t hid) {
    begin();
    auto h = holds.find(hid);
    require(h != holds.end() && h->second.pid == P, "start_body: not P's hold");
    h->second.started = true;
    g_body_start(h->second);
  }

  // -- 4.9
  // ------------------------------------------------------------------------------------------------
  std::optional<ticket_state> cancel(lock_pid P, std::uint64_t tid, spec_notes &out) {
    begin();
    out.clear();
    auto T = tickets.find(tid);
    if (T == tickets.end() || T->second.pid != P)
      return std::nullopt;
    if (T->second.state != ticket_state::queued)
      return T->second.state;
    queue.erase(tid);
    T->second.state = ticket_state::cancelled;
    if (has(mutation::cancel_marks_failed)) {
      T->second.state = ticket_state::failed;
      T->second.error.kind = spec_error::timeout;
    }
    key_set region(T->second.keys.begin(), T->second.keys.end());
    seeds_ = {P};
    region_is_ticket_ = true;
    in_cancel_ = true;
    try {
      finish(region, {P}, {});
    } catch (...) {
      region_is_ticket_ = false;
      in_cancel_ = false;
      throw;
    }
    region_is_ticket_ = false;
    in_cancel_ = false;
    out = N_;
    return ticket_state::cancelled;
  }

  // -- 4.10
  // -----------------------------------------------------------------------------------------------
  void release(lock_pid P, std::uint64_t hid, spec_notes &out) {
    begin();
    out.clear();
    auto h = holds.find(hid);
    if (h == holds.end() || h->second.pid != P)
      return;
    auto keys = drop_hold(hid, false);
    seeds_ = {P};
    finish(key_set(keys.begin(), keys.end()), {P}, {});
    out = N_;
  }

  // -- 4.11
  // -----------------------------------------------------------------------------------------------
  key_set end_locked(lock_pid P, bool abandon) {
    auto r = recs.find(P);
    if (r == recs.end())
      return {};
    key_set region;
    if (r->second.ticket) {
      auto &T = tickets.at(*r->second.ticket);
      if (T.state == ticket_state::queued) {
        queue.erase(T.seq);
        region.insert(T.keys.begin(), T.keys.end());
      }
      tickets.erase(*r->second.ticket);
      r->second.ticket.reset();
    }
    std::set<std::uint64_t> hs = r->second.holds;
    for (auto hid : hs) {
      auto keys = drop_hold(hid, abandon);
      region.insert(keys.begin(), keys.end());
    }
    recs.erase(P);
    return region;
  }
  void end(lock_pid P, bool abandon, spec_notes &out) {
    begin();
    seeds_ = {P};
    auto region = end_locked(P, abandon);
    finish(region, {}, {P});
    out = N_;
  }
  void detach_home(home_id H, spec_notes &out) {
    begin();
    require(homes.count(H) > 0, "detach_home: H" + std::to_string(H) + " is not attached");
    if (!has(mutation::keep_detached_refs)) {
      auto it = cache_refs.find(H);
      if (it != cache_refs.end()) {
        auto copy = it->second;
        for (const auto &[k, c] : copy)
          for (int i = 0; i < c; ++i)
            unref(H, k);
      }
      cache_refs.erase(H);
    }
    key_set region;
    std::set<lock_pid> ended;
    std::vector<lock_pid> ps;
    for (const auto &kv : recs)
      ps.push_back(kv.first);
    for (auto P : ps)
      if (P.home == H) {
        auto rg = end_locked(P, true);
        region.insert(rg.begin(), rg.end());
        ended.insert(P);
      }
    homes.erase(H);
    seeds_ = ended;
    finish(region, {}, ended);
    for (const auto &n : N_)
      if (n.home == H)
        fail("detach", "a notification of detach_home targets the detached home");
    out = N_;
  }

  // -- 4.12
  // -----------------------------------------------------------------------------------------------
  void set_base(lock_pid P, int base, spec_notes &out) {
    begin();
    out.clear();
    auto r = recs.find(P);
    if (r == recs.end())
      return;
    r->second.base = base;
    seeds_ = {P};
    relevel_all({});
    out = N_;
  }

  // -- 4.13
  // -----------------------------------------------------------------------------------------------
  int read_inh(lock_pid P) {
    auto r = recs.find(P);
    if (r == recs.end())
      return INF;
    if (!has(mutation::coalesce_no_clear))
      r->second.rl = false; // 4.3 coalescing: the apply clears the flag under M.mu
    return r->second.inh;
  }
  int peek_inh(lock_pid P) const {
    auto r = recs.find(P);
    return r == recs.end() ? INF : r->second.inh;
  }
  lock_query_result query(const std::string &pre, std::optional<home_id> vh,
                          const std::string &vowner) const {
    auto visible = [&](lock_pid Q) {
      if (!vh || Q.home != *vh)
        return false;
      auto r = recs.find(Q);
      return r != recs.end() && r->second.owner == vowner;
    };
    key_set keys;
    for (const auto &[i, h] : holds)
      for (const auto &k : h.keys)
        if (prefix(pre, k))
          keys.insert(k);
    lock_query_result res;
    for (const auto &k : keys) {
      lock_info e;
      e.key = k;
      std::vector<const spec_hold *> hs;
      for (const auto &[i, h] : holds)
        if (contains(h.keys, k))
          hs.push_back(&h);
      bool anyx = false;
      for (const auto *h : hs)
        if (h->mode == X) {
          if (!anyx)
            e.generation = h->gen;
          else
            e.generation = has(mutation::query_first_gen) ? std::min(*e.generation, h->gen)
                                                          : std::max(*e.generation, h->gen);
          anyx = true;
        }
      e.mode = anyx ? X : S;
      for (const auto *h : hs)
        if (visible(h->pid))
          e.pids.push_back(h->pid.pid);
      e.count = hs.size();
      e.abandoned = abandoned.count(k) > 0;
      res.held.push_back(std::move(e));
    }
    for (auto t : queue) {
      const auto &T = tickets.at(t);
      bool any = false;
      for (const auto &k : T.keys)
        any = any || prefix(pre, k);
      if (!any)
        continue;
      lock_waiter_info w;
      w.keys = T.keys;
      w.mode = T.mode;
      if (visible(T.pid))
        w.pid = T.pid.pid;
      res.waiting.push_back(std::move(w));
    }
    return res;
  }

  void ref_nodes(home_id H, const key_list &keys) {
    begin();
    for (const auto &k : keys)
      ref(H, k);
  }
  void unref_nodes(home_id H, const key_list &keys) {
    begin();
    for (const auto &k : keys)
      unref(H, k);
  }

  // -- section 6: the manager's clauses
  // ------------------------------------------------------------------
  void check() const {
    // I1
    for (auto a = holds.begin(); a != holds.end(); ++a)
      for (auto b = std::next(a); b != holds.end(); ++b)
        if (a->second.pid != b->second.pid &&
            conflicts(a->second.keys, a->second.mode, b->second.keys, b->second.mode))
          fail("I1", "holds " + std::to_string(a->first) + " and " + std::to_string(b->first) +
                         " of " + pid_str(a->second.pid) + " and " + pid_str(b->second.pid) +
                         " conflict");
    // I2
    std::set<std::uint64_t> q;
    for (const auto &[t, T] : tickets)
      if (T.state == ticket_state::queued)
        q.insert(t);
    if (q != queue)
      fail("I2", "queue " + join_u64(queue) + " != queued tickets " + join_u64(q));
    std::map<std::uint64_t, lock_pid> owned;
    for (const auto &[P, r] : recs)
      if (r.ticket) {
        if (owned.count(*r.ticket))
          fail("I2", "ticket owned twice");
        owned[*r.ticket] = P;
      }
    std::set<std::uint64_t> tk, ok;
    for (const auto &kv : tickets)
      tk.insert(kv.first);
    for (const auto &kv : owned)
      ok.insert(kv.first);
    if (tk != ok)
      fail("I2", "tickets " + join_u64(tk) + " are not exactly the recs' tickets " + join_u64(ok));
    for (const auto &[t, T] : tickets) {
      if (owned.at(t) != T.pid)
        fail("I2", "ticket " + std::to_string(t) + " pid mismatch");
      if (T.state == ticket_state::granted) {
        auto h = T.hold ? holds.find(*T.hold) : holds.end();
        if (h == holds.end() || h->second.pid != T.pid || h->second.started)
          fail("I2",
               "granted ticket " + std::to_string(t) + " has no unstarted hold of its process");
      }
    }
    // I3
    for (const auto &[P, r] : recs) {
      if (r.holds.empty() && !r.ticket)
        fail("I3", "rec " + pid_str(P) + " with nothing");
      std::set<std::uint64_t> mine;
      for (const auto &[i, h] : holds)
        if (h.pid == P)
          mine.insert(i);
      if (mine != r.holds)
        fail("I3", "rec " + pid_str(P) + " holds mismatch");
    }
    for (const auto &[i, h] : holds)
      if (!recs.count(h.pid))
        fail("I3", "hold " + std::to_string(i) + " without rec");
    // I9: M.cache_refs has an entry exactly for each attached home; I4: counts positive
    for (const auto &[H, d] : cache_refs) {
      if (!homes.count(H))
        fail("I9", "key-cache references of detached home H" + std::to_string(H) +
                       " kept: " + keys_br([&] {
                         key_list ks;
                         for (const auto &kv : d)
                           ks.push_back(kv.first);
                         return ks;
                       }()));
      for (const auto &kv : d)
        if (kv.second <= 0)
          fail("I4", "non-positive cache reference count for H" + std::to_string(H));
    }
    for (const auto &kv : homes)
      if (!cache_refs.count(kv.first))
        fail("I9", "attached home H" + std::to_string(kv.first) + " has no key-cache entry");
    // 4.13: every process with a rec is on an attached home
    for (const auto &kv : recs)
      if (!homes.count(kv.first.home))
        fail("4.13", "rec " + pid_str(kv.first) + " of a home that is not attached");
    // (no trie in the oracle: the table's own check() asserts its trie, I4)
    // I6 first half
    for (auto t : queue)
      if (grantable_t(tickets.at(t)))
        fail("I6", "queued ticket " + std::to_string(t) + " of " + pid_str(tickets.at(t).pid) +
                       " is grantable after the operation");
    // I7
    auto g = graph();
    if (auto cyc = find_cycle(g)) {
      std::string s;
      for (auto p : *cyc)
        s += pid_str(p) + " ";
      fail("I7", "wait-for cycle " + s);
    }
    // I8 manager clause, against an independent enumeration of paths of 1..8 edges
    auto ref_inh = inh_reference(g, lending());
    for (const auto &[P, r] : recs)
      if (r.inh != ref_inh.at(P))
        fail("I8", "rec " + pid_str(P) + " inh " + inh_str(r.inh) + " != transitive definition " +
                       inh_str(ref_inh.at(P)));
    // I11
    key_set seen = abandoned;
    for (const auto &[i, h] : holds) {
      for (const auto &k : h.taken)
        if (seen.count(k))
          fail("I11", "abandoned flags not disjoint");
      seen.insert(h.taken.begin(), h.taken.end());
    }
    key_set expect = abandoned;
    for (const auto &[i, h] : holds)
      if (!h.started)
        expect.insert(h.taken.begin(), h.taken.end());
    if (expect != g_pending_report)
      fail("I11", "reports not conserved: pending " + keys_br(g_pending_report) +
                      ", flags+unstarted taken " + keys_br(expect));
    for (const auto &[i, h] : holds)
      if (h.mode == S && !h.taken.empty())
        fail("I11", "S hold took a flag");
  }

  /// The mathematical state in lock_table.h's canonical() format.
  std::string canonical() const {
    std::string o;
    o.reserve(1024);
    o += "homes\n";
    for (const auto &[h, v] : homes)
      o += "H" + std::to_string(h) + " prio=" + (v.prio ? "1" : "0") + "\n";
    o += "recs\n";
    for (const auto &[P, r] : recs)
      o += pid_str(P) + " owner=\"" + r.owner + "\" base=" + std::to_string(r.base) +
           " holds=" + join_u64(r.holds) +
           " ticket=" + (r.ticket ? std::to_string(*r.ticket) : std::string("-")) +
           " inh=" + inh_str(r.inh) + "\n";
    o += "holds\n";
    for (const auto &[i, h] : holds)
      o += std::to_string(i) + " pid=" + pid_str(h.pid) + " keys=" + keys_br(h.keys) +
           " mode=" + mode_char(h.mode) + " gen=" + std::to_string(h.gen) +
           " started=" + (h.started ? "1" : "0") + " taken=" + keys_br(h.taken) + "\n";
    o += "tickets\n";
    for (const auto &[t, T] : tickets)
      o += std::to_string(t) + " pid=" + pid_str(T.pid) + " keys=" + keys_br(T.keys) +
           " mode=" + mode_char(T.mode) + " state=" + state_name(T.state) +
           " hold=" + (T.hold ? std::to_string(*T.hold) : std::string("-")) +
           " error=" + error_str(T.error) + "\n";
    o += "queue=" + join_u64(queue) + "\n";
    o += "abandoned=" + keys_br(abandoned) + "\n";
    o += "next seq=" + std::to_string(next_seq) + " hold=" + std::to_string(next_hold) +
         " gen=" + std::to_string(next_gen) + "\n";
    o += "cache_refs\n";
    for (const auto &[h, d] : cache_refs) {
      o += "H" + std::to_string(h) + " {";
      bool first = true;
      for (const auto &[k, c] : d) {
        if (!first)
          o += ",";
        first = false;
        o += k + ":" + std::to_string(c);
      }
      o += "}\n";
    }
    o += "counters grants=" + std::to_string(grants) + "\n";
    return o;
  }
  static std::string error_str(const spec_error &e) {
    if (e.kind == spec_error::none)
      return "-";
    if (e.kind == spec_error::timeout)
      return "TimeoutError";
    std::string o = "DeadlockError[";
    for (std::size_t i = 0; i < e.cycle.size(); ++i) {
      if (i)
        o += ",";
      o += "(" + (e.cycle[i].pid ? std::to_string(*e.cycle[i].pid) : std::string("nil")) + ":" +
           e.cycle[i].key + ")";
    }
    return o + "]";
  }

  /// 2.4's trie, counted: the root and every prefix of a key with a hold, a queued ticket, an
  /// abandoned flag or a key-cache reference (what I9 leaves once every process has ended).
  std::size_t expected_trie_nodes() const {
    std::set<std::string> nodes;
    auto add = [&](const std::string &k) {
      for (std::size_t i = 0; i <= k.size(); ++i)
        if (i == k.size() || k[i] == '.')
          nodes.insert(k.substr(0, i));
    };
    for (const auto &[i, h] : holds)
      for (const auto &k : h.keys)
        add(k);
    for (auto t : queue)
      for (const auto &k : tickets.at(t).keys)
        add(k);
    for (const auto &k : abandoned)
      add(k);
    for (const auto &[H, d] : cache_refs)
      for (const auto &kv : d)
        add(kv.first);
    return nodes.size() + 1; // the root
  }
  std::size_t total_cache_refs() const {
    std::size_t n = 0;
    for (const auto &[H, d] : cache_refs)
      for (const auto &kv : d)
        n += std::size_t(kv.second);
    return n;
  }

private:
  spec_notes N_;
  std::map<lock_pid, int> start_inh_;
  bool region_is_ticket_ = false;
  bool in_cancel_ = false; // cancel_skips_relevel's test
  // 4.3's implementation note: the operation's seeds (its process, the processes it ended, and
  // the processes granted), lending and the wait-for graph when it started
  std::set<lock_pid> seeds_;
  bool lending_before_ = false;
  graph_t start_graph_;
};

// =============================================================================================
// 2. Results in one text form: the lockstep comparison and the trace files share it
// =============================================================================================

std::string fmt_grant(const std::optional<dt::grant> &g) {
  if (!g)
    return "busy";
  return "granted " + std::to_string(g->hold) + " " + std::to_string(g->gen) + " " +
         (g->abandoned ? "1" : "0");
}
std::string fmt_block(const dt::block_result &b) {
  return std::to_string(b.tid) + " " + state_name(b.state);
}
std::string fmt_cycle(const std::vector<dt::deadlock_edge> &c) {
  std::string o;
  for (const auto &e : c) {
    if (!o.empty())
      o += ",";
    o += (e.pid ? std::to_string(*e.pid) : std::string("nil")) + ":" + e.key;
  }
  return o;
}
dt::lock_outcome to_outcome(const spec_outcome &s) {
  dt::lock_outcome o;
  o.what = s.what;
  o.g = s.g;
  for (const auto &e : s.cycle)
    o.cycle.push_back(dt::deadlock_edge{e.pid, e.key});
  return o;
}
std::string fmt_outcome(const dt::lock_outcome &o) {
  switch (o.what) {
  case dt::lock_outcome::kind::queued:
    return "queued";
  case dt::lock_outcome::kind::granted:
    return fmt_grant(o.g);
  case dt::lock_outcome::kind::timeout:
    return "timeout";
  case dt::lock_outcome::kind::deadlock:
    return "deadlock " + fmt_cycle(o.cycle);
  }
  return "?";
}
std::string fmt_cancel(const std::optional<ticket_state> &s) {
  return s ? std::string(state_name(*s)) : std::string("none");
}
std::string fmt_post(home_id h, const dt::lock_post &p) {
  if (p.what == dt::lock_post::kind::wake)
    return "wake(" + std::to_string(h) + "," + std::to_string(p.pid) + "," + std::to_string(p.tid) +
           ")";
  return "relevel(" + std::to_string(h) + "," + std::to_string(p.pid) + ")";
}
std::string fmt_notes(const spec_notes &N) {
  std::string o;
  for (const auto &n : N) {
    if (!o.empty())
      o += " ";
    o += fmt_post(n.home, n.post);
  }
  return o;
}
std::string fmt_notes(const dt::lock_notes &N) {
  std::string o;
  for (const auto &n : N) {
    if (!o.empty())
      o += " ";
    o += fmt_post(n.home, n.post);
  }
  return o;
}
/// lock-query's result; `normalized` sorts held by key and each pid list (the protocol fixes the
/// waiting order, seq, but not those two orders), which is how the table is compared.
std::string fmt_query(lock_query_result r, bool normalized) {
  if (normalized) {
    std::sort(r.held.begin(), r.held.end(),
              [](const lock_info &a, const lock_info &b) { return dt::key_less(a.key, b.key); });
    for (auto &e : r.held)
      std::sort(e.pids.begin(), e.pids.end());
  }
  std::string o = "held";
  for (const auto &e : r.held) {
    std::string pids;
    for (int p : e.pids)
      pids += (pids.empty() ? "" : ",") + std::to_string(p);
    o += " " + e.key + "/" + mode_char(e.mode) + "/" + (pids.empty() ? "-" : pids) + "/" +
         std::to_string(e.count) + "/" +
         (e.generation ? std::to_string(*e.generation) : std::string("-")) + "/" +
         (e.abandoned ? "1" : "0");
  }
  o += " waiting";
  for (const auto &w : r.waiting)
    o += " " + keys_tok(w.keys) + "/" + mode_char(w.mode) + "/" +
         (w.pid ? std::to_string(*w.pid) : std::string("-"));
  return o;
}

// =============================================================================================
// 3. lockstep: the oracle and the real table, operation by operation
// =============================================================================================

class lockstep {
public:
  struct options {
    bool real = true;
    bool inheritance = true;
    bool coalesce = false;
    bool one_home = true;
    std::uint32_t mutations = 0;      // both sides (lockmodel.py's single Manager takes them)
    std::uint32_t real_mutations = 0; // the real table only (plant_divergence)
    std::uint32_t spec_mutations = 0; // the oracle only
  };

  explicit lockstep(const options &o)
      : opts(o), spec(spec_manager::options{o.inheritance, o.coalesce, o.one_home,
                                            o.mutations | o.spec_mutations}) {
    if (o.real) {
      dt::lock_table::options t;
      t.inheritance = o.inheritance;
      t.self_check = true; // also the grant-time I5 and I15 assertions (gap G-14)
      t.coalesce_relevels = o.coalesce;
      t.one_home = o.one_home;
      t.mutations = o.mutations | o.real_mutations;
      real = std::make_unique<dt::lock_table>(t);
    }
  }
  /// A world with no real table is copied (the explorer's clone mode); one with the real table is
  /// never copied: its states are rebuilt by replay.
  lockstep(const lockstep &o)
      : opts(o.opts), spec(o.spec), checked(o.checked), canon_(o.canon_), dirty_(o.dirty_) {
    if (o.real)
      throw std::logic_error("lockstep: a world over the real table is replayed, never copied");
  }
  lockstep(lockstep &&) = default;
  lockstep &operator=(const lockstep &) = delete;
  lockstep &operator=(lockstep &&) = delete;

  options opts;
  spec_manager spec;
  std::unique_ptr<dt::lock_table> real;
  /// false while a known path is replayed: results and notes are still compared, but not the
  /// canonical dumps, and no check() runs.
  bool checked = true;

  // -- operations ---------------------------------------------------------------------------------
  home_id attach_home(bool prio, spec_notes &N) {
    const std::string desc = std::string("attach_home prio=") + (prio ? "1" : "0");
    home_id h = 0;
    try {
      h = spec.attach_home(prio, N);
    } catch (const lock_violation &v) {
      if (v.invariant() == "require" && real) {
        bool refused = false; // protocol 4.4: the table refuses too, with a std::logic_error
        try {
          dt::lock_notes rn;
          real->attach_home(prio, nullptr, nullptr, rn);
        } catch (const std::logic_error &e) {
          if (is_stub(e))
            throw not_built(e.what());
          refused = true;
        }
        if (!refused)
          fail("lockstep", desc + ": the oracle refuses (" + v.what() + ") but the table attached");
      }
      throw;
    }
    if (real) {
      dt::lock_notes rn;
      home_id rh = real_call(desc, [&] { return real->attach_home(prio, nullptr, nullptr, rn); });
      if (rh != h)
        mismatch(desc, "home id", std::to_string(h), std::to_string(rh));
      compare_notes(desc, N, rn);
    }
    after(desc);
    return h;
  }

  std::optional<dt::grant> try_acquire(lock_pid P, const std::string &owner, int base,
                                       const key_list &keys, lock_mode mode, const key_list &add,
                                       const key_list &drop) {
    const std::string desc =
        "try_acquire " + pid_str(P) + " " + req_str(keys, mode) + refs_desc(add, drop);
    auto r = spec_call(
        desc, [&] { return spec.try_acquire(P, owner, base, keys, mode, add, drop); },
        [&] {
          auto rq = build(P.home, keys, mode, add, drop);
          real->try_acquire(P, dt::rec_info{owner, base}, rq.req, rq.delta);
        });
    if (real) {
      auto rq = build(P.home, keys, mode, add, drop);
      auto rr = real_call(
          desc, [&] { return real->try_acquire(P, dt::rec_info{owner, base}, rq.req, rq.delta); });
      settle_refs(desc, P.home, keys, add, drop, rq);
      if (fmt_grant(r) != fmt_grant(rr))
        mismatch(desc, "result", fmt_grant(r), fmt_grant(rr));
    }
    after(desc);
    return r;
  }

  dt::block_result block(lock_pid P, const std::string &owner, int base, const key_list &keys,
                         lock_mode mode, const key_list &add, const key_list &drop, spec_notes &N) {
    const std::string desc =
        "block " + pid_str(P) + " " + req_str(keys, mode) + refs_desc(add, drop);
    auto r = spec_call(
        desc, [&] { return spec.block(P, owner, base, keys, mode, add, drop, N); },
        [&] {
          auto rq = build(P.home, keys, mode, add, drop);
          dt::lock_notes rn;
          real->block(P, dt::rec_info{owner, base}, rq.req, rq.delta, rn);
        });
    if (real) {
      auto rq = build(P.home, keys, mode, add, drop);
      dt::lock_notes rn;
      auto rr = real_call(
          desc, [&] { return real->block(P, dt::rec_info{owner, base}, rq.req, rq.delta, rn); });
      settle_refs(desc, P.home, keys, add, drop, rq);
      if (fmt_block(r) != fmt_block(rr))
        mismatch(desc, "result", fmt_block(r), fmt_block(rr));
      compare_notes(desc, N, rn);
    }
    after(desc);
    return r;
  }

  spec_outcome resume(lock_pid P, std::uint64_t tid) {
    const std::string desc = "resume " + pid_str(P) + " ticket " + std::to_string(tid);
    auto r = spec_call(desc, [&] { return spec.resume(P, tid); }, [&] { real->resume(P, tid); });
    if (real) {
      auto rr = real_call(desc, [&] { return real->resume(P, tid); });
      auto a = fmt_outcome(to_outcome(r)), b = fmt_outcome(rr);
      if (a != b)
        mismatch(desc, "result", a, b);
    }
    after(desc);
    return r;
  }

  void start_body(lock_pid P, std::uint64_t hid) {
    const std::string desc = "start_body " + pid_str(P) + " hold " + std::to_string(hid);
    spec_call(
        desc,
        [&] {
          spec.start_body(P, hid);
          return 0;
        },
        [&] { real->start_body(P, hid); });
    if (real)
      real_call(desc, [&] { real->start_body(P, hid); });
    after(desc);
  }

  std::optional<ticket_state> cancel(lock_pid P, std::uint64_t tid, spec_notes &N) {
    const std::string desc = "cancel " + pid_str(P) + " ticket " + std::to_string(tid);
    auto r = spec_call(desc, [&] { return spec.cancel(P, tid, N); });
    if (real) {
      dt::lock_notes rn;
      auto rr = real_call(desc, [&] { return real->cancel(P, tid, rn); });
      if (fmt_cancel(r) != fmt_cancel(rr))
        mismatch(desc, "result", fmt_cancel(r), fmt_cancel(rr));
      compare_notes(desc, N, rn);
    }
    after(desc);
    return r;
  }

  void release(lock_pid P, std::uint64_t hid, spec_notes &N) {
    const std::string desc = "release " + pid_str(P) + " hold " + std::to_string(hid);
    spec_call(desc, [&] {
      spec.release(P, hid, N);
      return 0;
    });
    if (real) {
      dt::lock_notes rn;
      real_call(desc, [&] { real->release(P, hid, rn); });
      compare_notes(desc, N, rn);
    }
    after(desc);
  }

  void end(lock_pid P, bool abandon, spec_notes &N) {
    const std::string desc = "end " + pid_str(P) + (abandon ? " abandon" : " clean");
    spec_call(desc, [&] {
      spec.end(P, abandon, N);
      return 0;
    });
    if (real) {
      dt::lock_notes rn;
      real_call(desc, [&] { real->end(P, abandon, rn); });
      compare_notes(desc, N, rn);
    }
    after(desc);
  }

  void detach_home(home_id H, spec_notes &N) {
    const std::string desc = "detach_home H" + std::to_string(H);
    spec_call(
        desc,
        [&] {
          spec.detach_home(H, N);
          return 0;
        },
        [&] {
          dt::lock_notes rn;
          real->detach_home(H, rn);
        });
    if (real) {
      dt::lock_notes rn;
      real_call(desc, [&] { real->detach_home(H, rn); });
      nodes_.erase(H); // its references went with it (kept under keep_detached_refs, unused)
      compare_notes(desc, N, rn);
    }
    after(desc);
  }

  void set_base(lock_pid P, int base, spec_notes &N) {
    const std::string desc = "set_base " + pid_str(P) + " " + std::to_string(base);
    spec_call(desc, [&] {
      spec.set_base(P, base, N);
      return 0;
    });
    if (real) {
      dt::lock_notes rn;
      real_call(desc, [&] { real->set_base(P, base, rn); });
      compare_notes(desc, N, rn);
    }
    after(desc);
  }

  int read_inh(lock_pid P) {
    const std::string desc = "read_inh " + pid_str(P);
    int v = spec.read_inh(P);
    if (real) {
      int rv = real_call(desc, [&] { return real->read_inh(P); });
      if (v != rv)
        mismatch(desc, "result", inh_str(v), inh_str(rv));
    }
    after(desc);
    return v;
  }

  lock_query_result query(const std::string &pre, std::optional<home_id> vh,
                          const std::string &vowner) {
    auto r = spec.query(pre, vh, vowner);
    if (real && checked) {
      const std::string desc = "query '" + pre + "' viewer " +
                               (vh ? "H" + std::to_string(*vh) : std::string("none")) + "/" +
                               vowner;
      auto rr = real_call(desc, [&] { return real->query(pre, vh, vowner); });
      auto a = fmt_query(r, true), b = fmt_query(rr, true);
      if (a != b)
        mismatch(desc, "result", a, b);
    }
    return r;
  }

  void ref_nodes(home_id H, const key_list &keys) {
    const std::string desc = "ref_nodes H" + std::to_string(H) + " " + keys_br(keys);
    spec_call(
        desc,
        [&] {
          spec.ref_nodes(H, keys);
          return 0;
        },
        [&] {
          std::vector<dt::key_ref> refs;
          for (const auto &k : keys)
            refs.push_back(dt::key_ref{nullptr, k, true});
          real->ref_nodes(H, std::span<dt::key_ref>(refs));
        });
    if (real) {
      std::vector<dt::key_ref> refs;
      for (const auto &k : keys) {
        dt::key_ref r;
        r.key = k;
        r.ref = true;
        refs.push_back(r);
      }
      real_call(desc, [&] { real->ref_nodes(H, std::span<dt::key_ref>(refs)); });
      auto &m = nodes_[H];
      for (std::size_t i = 0; i < keys.size(); ++i) {
        if (!refs[i].node)
          fail("lockstep", desc + ": the table did not fill the node of " + keys[i]);
        auto &e = m[keys[i]];
        e.node = refs[i].node;
        ++e.count;
      }
    }
    after(desc);
  }

  void unref_nodes(home_id H, const key_list &keys) {
    const std::string desc = "unref_nodes H" + std::to_string(H) + " " + keys_br(keys);
    spec_call(
        desc,
        [&] {
          spec.unref_nodes(H, keys);
          return 0;
        },
        [&] {
          // the node of each key as the home knows it; a key the home holds no reference on
          // passes another home's node of it, or null (the table refuses both, 4.15)
          std::vector<dt::lock_node *> ns;
          for (const auto &k : keys)
            ns.push_back(any_node(H, k));
          real->unref_nodes(H, std::span<dt::lock_node *const>(ns));
        });
    if (real) {
      auto &m = nodes_[H];
      std::vector<dt::lock_node *> ns;
      for (const auto &k : keys) {
        auto it = m.find(k);
        if (it == m.end() || !it->second.node)
          fail("model", desc + ": the home holds no reference on " + k);
        ns.push_back(it->second.node);
      }
      real_call(desc, [&] { real->unref_nodes(H, std::span<dt::lock_node *const>(ns)); });
      for (const auto &k : keys)
        unref_local(m, k);
    }
    after(desc);
  }

  // -- reads for the world's checks
  // -----------------------------------------------------------------
  /// The oracle's canonical dump (computed when the state changed since the last one).
  const std::string &canonical() {
    if (dirty_) {
      canon_ = spec.canonical();
      dirty_ = false;
    }
    return canon_;
  }
  /// Every field but the key-cache references (I12 and I13 compare it, as lock_key() does).
  std::string lock_key() {
    std::string c = canonical();
    auto a = c.find("cache_refs\n");
    auto b = c.find("counters ");
    if (a != std::string::npos && b != std::string::npos && b > a)
      c.erase(a, b - a);
    return c;
  }
  /// The world's mirror check against the table's hold_keys.
  void check_hold_keys(std::uint64_t id, const key_list &keys) {
    if (!real || !checked)
      return;
    const std::string desc = "hold_keys " + std::to_string(id);
    auto span = real_call(desc, [&] { return real->hold_keys(id); });
    key_list got;
    for (auto *n : span)
      got.push_back(n->key);
    if (got != keys)
      mismatch(desc, "keys", keys_br(keys), keys_br(got));
  }
  /// I4's home side on the table: home h holds one reference on the node of each entry of its
  /// cache and none other (home_cache_refs).
  void check_home_cache(home_id h, const key_list &cache) {
    if (!real || !checked)
      return;
    const std::string desc = "home_cache_refs H" + std::to_string(h);
    auto m = nodes_.find(h);
    std::size_t want = 0;
    for (const auto &k : cache) {
      if (m == nodes_.end() || !m->second.count(k) || !m->second.at(k).node)
        continue; // a key the home caches with no reference: the oracle's I4 reports it
      ++want;
      auto n = real_call(desc, [&] { return real->home_cache_refs(h, m->second.at(k).node); });
      if (n != 1)
        fail("I4", "[table] H" + std::to_string(h) + " holds " + std::to_string(n) +
                       " references on " + k + ", which its cache has once");
    }
    auto total = real_call(desc, [&] { return real->home_cache_refs(h); });
    if (total != want)
      fail("I4", "[table] H" + std::to_string(h) + " holds " + std::to_string(total) +
                     " key-cache references, its cache " + std::to_string(want));
  }
  /// I9 on the table once every process has ended: empty(), and the trie holds only the nodes of
  /// abandoned flags and key-cache references and their ancestors (protocol 2.4).
  void check_empty_table() {
    if (!real || !checked)
      return;
    if (!real_call("empty", [&] { return real->empty(); }))
      fail("I9", "[table] every process ended (or every home detached) but empty() is false");
    auto st = real_call("stats", [&] { return real->stats(); });
    auto want = spec.expected_trie_nodes();
    if (st.trie_nodes != want)
      fail("I9", "[table] " + std::to_string(st.trie_nodes) + " trie nodes left, expected " +
                     std::to_string(want) +
                     " (the root and the flagged or cached keys with their ancestors)");
  }
  /// After a check run outside an operation (the world's first check).
  void check_now() {
    if (!checked)
      return;
    wrap_spec("check", [&] { spec.check(); });
    if (real)
      real_call("check", [&] { real->check(); });
  }

private:
  struct node_ref {
    dt::lock_node *node = nullptr;
    int count = 0;
  };
  /// The real side's view of each home's key-cache references: the node of each key the home
  /// holds a reference on (the home's cache entries), so a hit passes its node as lock_home does.
  std::map<home_id, std::map<std::string, node_ref>> nodes_;
  std::string canon_;
  bool dirty_ = true;

  static std::string refs_desc(const key_list &add, const key_list &drop) {
    if (add.empty() && drop.empty())
      return "";
    return " refs +" + keys_br(add) + " -" + keys_br(drop);
  }

  template <class F> static void wrap_spec(const std::string &desc, F &&f) {
    try {
      f();
    } catch (const lock_violation &v) {
      throw lock_violation(v.invariant(), "[oracle] " + desc + ": " + v.what());
    }
  }
  template <class F> auto spec_call(const std::string &desc, F &&f) -> decltype(f()) {
    try {
      return f();
    } catch (const lock_violation &v) {
      if (v.invariant() == "require")
        throw; // a refusal the caller expects (the traces' 'refused')
      throw lock_violation(v.invariant(), "[oracle] " + desc + ": " + v.what());
    }
  }
  /// spec_call, and when the oracle refuses (a 'require' of the protocol), the same operation on
  /// the table (`real_op`), which must refuse too: with a lock_violation naming the clause whose
  /// precondition fails, or 4.4's std::logic_error. The oracle's refusal is then rethrown.
  template <class F, class G>
  auto spec_call(const std::string &desc, F &&f, G &&real_op) -> decltype(f()) {
    try {
      return f();
    } catch (const lock_violation &v) {
      if (v.invariant() != "require")
        throw lock_violation(v.invariant(), "[oracle] " + desc + ": " + v.what());
      if (real) {
        bool refused = false;
        try {
          real_op();
        } catch (const lock_violation &) {
          refused = true;
        } catch (const std::logic_error &e) {
          if (is_stub(e))
            throw not_built(e.what());
          refused = true;
        }
        if (!refused)
          fail("lockstep",
               desc + ": the oracle refuses (" + v.what() + ") but the table accepted it");
        if (checked) // both refused at the same point, so their states are still equal
          compare_canonical(desc + " (refused)", spec.canonical(), real->canonical());
      }
      dirty_ = true;
      throw;
    }
  }
  template <class F> auto real_call(const std::string &desc, F &&f) -> decltype(f()) {
    try {
      return f();
    } catch (const lock_violation &v) {
      throw lock_violation(v.invariant(), "[table] " + desc + ": " + v.what());
    } catch (const std::logic_error &e) {
      if (is_stub(e))
        throw not_built(e.what());
      fail("lockstep", desc + ": the table threw std::logic_error: " + e.what());
    } catch (const std::exception &e) {
      fail("lockstep", desc + ": the table threw: " + std::string(e.what()));
    }
  }

  [[noreturn]] static void mismatch(const std::string &desc, const std::string &what,
                                    const std::string &oracle, const std::string &table) {
    fail("lockstep", desc + ": " + what + " differs\n  oracle: " + oracle + "\n  table:  " + table);
  }
  static void compare_notes(const std::string &desc, const spec_notes &a, const dt::lock_notes &b) {
    auto x = fmt_notes(a), y = fmt_notes(b);
    if (x != y)
      mismatch(desc, "notification list", "[" + x + "]", "[" + y + "]");
  }
  static void compare_canonical(const std::string &desc, const std::string &a,
                                const std::string &b) {
    if (a == b)
      return;
    auto la = split(a, '\n'), lb = split(b, '\n');
    // one trailing newline more or less is not a difference of state
    while (!la.empty() && la.back().empty())
      la.pop_back();
    while (!lb.empty() && lb.back().empty())
      lb.pop_back();
    if (la == lb)
      return;
    std::size_t i = 0;
    while (i < la.size() && i < lb.size() && la[i] == lb[i])
      ++i;
    fail("lockstep",
         "after " + desc + ": the canonical dumps differ at line " + std::to_string(i + 1) +
             "\n  oracle: " + (i < la.size() ? la[i] : "<end>") + "\n  table:  " +
             (i < lb.size() ? lb[i] : "<end>") + "\n--- oracle\n" + a + "--- table\n" + b);
  }

  void after(const std::string &desc) {
    dirty_ = true;
    if (!checked)
      return;
    const std::string &c = canonical();
    if (real) {
      auto rc = real_call(desc, [&] { return real->canonical(); });
      compare_canonical(desc, c, rc);
    }
    wrap_spec("check after " + desc, [&] { spec.check(); });
    if (real) {
      real_call("check after " + desc, [&] { real->check(); });
      auto st = real_call(desc, [&] { return real->stats(); });
      auto chk = [&](const char *name, std::uint64_t got, std::uint64_t want) {
        if (got != want)
          mismatch(desc, std::string("stats().") + name, std::to_string(want), std::to_string(got));
      };
      chk("holds", st.holds, spec.holds.size());
      chk("queued", st.queued, spec.queue.size());
      chk("grants", st.grants, spec.grants);
      chk("cache_refs", st.cache_refs, spec.total_cache_refs());
      chk("relevels", st.relevels, spec.relevels);
      chk("relevels_coalesced", st.relevels_coalesced, spec.relevels_coalesced);
    }
  }

  struct real_request {
    dt::lock_request req;
    dt::cache_delta delta;
    key_list deferred; // evictions of the request's own keys (see build)
  };
  /// The request as lock_home builds it: a key the home holds a reference on passes its node, a
  /// miss passes its key with ref (4.15's refs.add), another key passes its key. An eviction
  /// passes the evicted entry's node (refs.drop). The implementation pins a request's own
  /// entries, so refs.drop never names a key of the request (lock_table.h's cache_delta); the
  /// Python model's FIFO cache does not, and a trace from it may evict a key of the request.
  /// Such an eviction is applied right after the operation, by unref_nodes: the references end
  /// the same, and the canonical dumps are compared after both calls.
  real_request build(home_id H, const key_list &keys, lock_mode mode, const key_list &add,
                     const key_list &drop) {
    real_request rq;
    rq.req.mode = mode;
    auto &m = nodes_[H];
    for (const auto &k : keys) {
      dt::key_ref r;
      auto it = m.find(k);
      if (contains(add, k)) {
        r.key = k;
        r.ref = true;
      } else if (it != m.end() && it->second.node) {
        r.node = it->second.node;
      } else {
        r.key = k;
      }
      rq.req.keys.push_back(r);
    }
    for (const auto &k : drop) {
      if (contains(keys, k)) {
        rq.deferred.push_back(k);
        continue;
      }
      auto it = m.find(k);
      if (it == m.end() || !it->second.node)
        fail("model", "an eviction of " + k + " that the home holds no reference on");
      rq.delta.drop.push_back(it->second.node);
    }
    return rq;
  }
  void settle_refs(const std::string &desc, home_id H, const key_list &keys, const key_list &add,
                   const key_list &drop, real_request &rq) {
    auto &m = nodes_[H];
    for (std::size_t i = 0; i < keys.size(); ++i)
      if (contains(add, keys[i])) {
        auto *n = rq.req.keys[i].node;
        if (!n)
          fail("lockstep", desc + ": the table did not fill the node of " + keys[i] +
                               ", on which it took a reference");
        auto &e = m[keys[i]];
        e.node = n;
        ++e.count;
      }
    for (const auto &k : drop)
      if (!contains(keys, k))
        unref_local(m, k);
    for (const auto &k : rq.deferred) {
      auto it = m.find(k);
      if (it == m.end() || !it->second.node)
        fail("model", "an eviction of " + k + " that the home holds no reference on");
      dt::lock_node *const arr[1] = {it->second.node};
      real_call(desc + " (its eviction of " + k + ")",
                [&] { real->unref_nodes(H, std::span<dt::lock_node *const>(arr, 1)); });
      unref_local(m, k);
    }
  }
  /// The node of key k that home H holds a reference on, else one another home does, else null.
  dt::lock_node *any_node(home_id H, const std::string &k) const {
    auto h = nodes_.find(H);
    if (h != nodes_.end())
      if (auto it = h->second.find(k); it != h->second.end() && it->second.node)
        return it->second.node;
    for (const auto &[other, m] : nodes_)
      if (auto it = m.find(k); it != m.end() && it->second.node)
        return it->second.node;
    return nullptr;
  }
  static void unref_local(std::map<std::string, node_ref> &m, const std::string &k) {
    auto it = m.find(k);
    if (it == m.end())
      return;
    if (--it->second.count <= 0)
      m.erase(it);
  }
};

// =============================================================================================
// 4. The world: world.py's homes, processes, frames, key caches, ports and posts (protocol 5)
// =============================================================================================

enum class rkind : std::uint8_t { wait, wait_dl, ifavail, t0 };
const char *rkind_name(rkind k) {
  switch (k) {
  case rkind::wait:
    return "wait";
  case rkind::wait_dl:
    return "wait_dl";
  case rkind::ifavail:
    return "ifavail";
  case rkind::t0:
    return "t0";
  }
  return "?";
}

/// The world's own mutations (scenarios.py's names in parentheses).
enum world_mutation : std::uint32_t {
  wake_carries_outcome = 1u << 0,    // ('wake-no-tid-check') a wake resumes whatever waits now
  cache_miss_unreferenced = 1u << 1, // ('cache-miss-unreferenced') a miss cached with no reference
  no_set_base = 1u << 2,             // ('no-set-base') set-priority skips the home's set_base
};

struct proc_cfg {
  int hid = 0;
  int pid = 0;
  std::string owner;
  int base = 0;
};

/// world.py's Config, without the parts of other phases (late homes, split posts, group
/// teardown, scripted processes).
struct config {
  std::string name;
  std::vector<bool> homes; // prio of each home; its id is its index (attach order)
  std::vector<proc_cfg> procs;
  std::vector<std::pair<key_list, lock_mode>> requests;
  std::vector<rkind> kinds{rkind::wait};
  int max_depth = 2;
  int max_hdepth = 1;
  std::vector<int> prio_values;
  int signals = 0;
  std::vector<bool> kills{true};
  bool destroy = false;
  bool inheritance = true;
  std::uint32_t mutations = 0;       // the manager's, on both sides
  std::uint32_t real_mutations = 0;  // the real table's only
  std::uint32_t spec_mutations = 0;  // the oracle's only
  std::uint32_t world_mutations = 0; // world_mutation bits
  int prio_changes = 0;
  int req_per_proc = 2;
  bool coalesce = false;
  bool foreign = false;
  std::vector<int> cache{0}; // the key-cache capacity, or a choice made by the first action
  bool cache_choice = false;
  bool cache_clear = false;
  bool one_home = false;
  /// The implementation's pinning (protocol 4.15, design 8.4): the probe looks up every key of the
  /// request it builds before it makes any miss, so a miss never evicts a key of the request, an
  /// earlier or a later one, and a key that finds every entry pinned passes uncached. false:
  /// world.py's FIFO, which may evict a key of the same request (the Python state counts).
  bool pin = true;
  bool real = true;
  /// Set when the table refused this configuration's homes (one_home off, a second home): the
  /// world then runs the oracle alone.
  bool real_dropped = false;
};

enum class ak : std::uint8_t {
  cachecap,
  hstep,
  post,
  destroy,
  cclear,
  req,
  enter_step,
  finish,
  raise,
  hraise,
  signal,
  hend,
  prio,
  kill,
  deadline,
  frelease,
  fcancel
};
/// One action: a = pid or home (or the capacity), b = request index, value or abandon, c = the
/// request kind or a foreign id.
struct action {
  ak k = ak::hstep;
  std::int8_t a = 0;
  std::int16_t b = 0;
  std::uint32_t c = 0;
};
static_assert(sizeof(action) == 8);
action act(ak k, int a = 0, int b = 0, std::uint64_t c = 0) {
  return action{k, static_cast<std::int8_t>(a), static_cast<std::int16_t>(b),
                static_cast<std::uint32_t>(c)};
}
bool costs_event(ak k) { return k != ak::hstep && k != ak::post && k != ak::cachecap; }

struct post_t {
  bool wake = true;
  int pid = 0;
  std::uint64_t tid = 0;
};
std::string post_str(const post_t &p) {
  return p.wake ? "wake(p" + std::to_string(p.pid) + ",t" + std::to_string(p.tid) + ")"
                : "relevel(p" + std::to_string(p.pid) + ")";
}

enum class pk : std::uint8_t {
  block,
  dlv,
  after_block,
  wake_self,
  unwind,
  exit_handler,
  end_clear,
  destroy_clear,
  defensive_resume,
  detach
};
struct pending_op {
  pk k = pk::block;
  int pid = 0;
  int tgt = 0;
  post_t post;
  std::uint64_t tid = 0;
  int st = -1; // a ticket_state, or -1 for none
};
bool local_op(pk k) {
  return k == pk::after_block || k == pk::end_clear || k == pk::exit_handler ||
         k == pk::destroy_clear;
}
std::string pending_str(const pending_op &o) {
  switch (o.k) {
  case pk::block:
    return "block p" + std::to_string(o.pid);
  case pk::dlv:
    return "dlv to H" + std::to_string(o.tgt) + " " + post_str(o.post);
  case pk::after_block:
    return "after_block p" + std::to_string(o.pid) + " t" + std::to_string(o.tid) + " " +
           (o.st < 0 ? "-" : state_name(ticket_state(o.st)));
  case pk::wake_self:
    return "wake_self p" + std::to_string(o.pid) + " t" + std::to_string(o.tid) + " " +
           (o.st < 0 ? "none" : state_name(ticket_state(o.st)));
  case pk::unwind:
    return "unwind p" + std::to_string(o.pid);
  case pk::exit_handler:
    return "exit_handler p" + std::to_string(o.pid);
  case pk::end_clear:
    return "end_clear p" + std::to_string(o.pid);
  case pk::destroy_clear:
    return "destroy_clear";
  case pk::defensive_resume:
    return "defensive_resume p" + std::to_string(o.pid) + " t" + std::to_string(o.tid);
  case pk::detach:
    return "detach";
  }
  return "?";
}

struct outcome_t {
  enum kind_t : std::uint8_t { none, granted, timeout, deadlock } k = none;
  std::uint64_t hold = 0;
  std::vector<spec_edge> cycle;
};
std::string outcome_str(const outcome_t &o) {
  switch (o.k) {
  case outcome_t::none:
    return "-";
  case outcome_t::granted:
    return "g" + std::to_string(o.hold);
  case outcome_t::timeout:
    return "T";
  case outcome_t::deadlock: {
    std::string s = "D";
    for (const auto &e : o.cycle)
      s += "(" + (e.pid ? std::to_string(*e.pid) : std::string("nil")) + ":" + e.key + ":" +
           pid_str(e.from) + ")";
    return s;
  }
  }
  return "?";
}

/// lock_enter {outcome} or lock_body {hold} (2.2), with the request and whether it was made in a
/// handler.
struct frame {
  bool enter = false;
  std::optional<std::uint64_t> hold;
  key_list keys;
  lock_mode mode = X;
  outcome_t outcome;
  bool deadline = false;
  bool ih = false;
  std::optional<std::uint64_t> tid;
};

struct mhold {
  std::uint64_t id = 0;
  key_list keys;
  lock_mode mode = X;
  int count = 1;
  std::uint64_t gen = 0;
  bool abandoned = false;
  bool in_handler = false;
};

enum class pstatus : std::uint8_t { run, parked, ended };

struct proc {
  int hid = 0;
  int pid = 0;
  std::string owner;
  pstatus status = pstatus::run;
  int priority = 0;
  int inherited = INF;
  std::vector<mhold> locks;
  std::optional<std::uint64_t> lock_ticket;
  bool lock_waiting = false;
  bool lock_deadline = false;
  bool in_handler = false;
  std::optional<std::size_t> hbase;
  std::vector<frame> frames;
  int req_left = 0, sig_left = 0, prio_left = 0;
  int pc = 0, hpc = 0;
  lock_pid P() const { return lock_pid{home_id(hid), pid}; }
};

struct home_t {
  int hid = 0;
  bool prio = false;
  std::deque<post_t> port;
  bool closed = false;
  std::deque<pending_op> pending;
  bool destroyed = false;
  bool attached = true;
  key_list cache; // FIFO, at most the capacity
};

/// Whether the real table accepts `n` homes with one_home off (the stale-wake scenarios need two
/// homes; the L1 table may refuse a second home whatever the option). Probed once.
bool table_accepts_homes(std::size_t n) {
  static std::map<std::size_t, bool> memo;
  auto it = memo.find(n);
  if (it != memo.end())
    return it->second;
  bool ok = true;
  try {
    dt::lock_table::options o;
    o.one_home = false;
    dt::lock_table t(o);
    dt::lock_notes N;
    for (std::size_t i = 0; i < n; ++i)
      t.attach_home(false, nullptr, nullptr, N);
  } catch (const std::logic_error &e) {
    if (is_stub(e))
      throw not_built(e.what());
    ok = false;
  }
  memo[n] = ok;
  return ok;
}

class world {
public:
  explicit world(config c) : cfg(std::move(c)), M(lockstep_options(cfg)) {
    for (std::size_t i = 0; i < cfg.homes.size(); ++i) {
      home_t h;
      h.hid = int(i);
      h.prio = cfg.homes[i];
      homes.push_back(h);
      spec_notes N;
      home_id id = M.attach_home(cfg.homes[i], N);
      if (id != home_id(i))
        fail("model", "attach_home gave H" + std::to_string(id) + " to the home of index " +
                          std::to_string(i));
    }
    for (const auto &pc : cfg.procs) {
      proc p;
      p.hid = pc.hid;
      p.pid = pc.pid;
      p.owner = pc.owner;
      p.priority = pc.base;
      p.req_left = cfg.req_per_proc;
      p.sig_left = cfg.signals;
      p.prio_left = cfg.prio_changes;
      procs.emplace(pc.pid, std::move(p));
    }
    if (!cfg.cache_choice)
      cache_cap = cfg.cache.at(0);
  }

  config cfg;
  lockstep M;
  std::vector<home_t> homes;
  std::map<int, proc> procs;
  std::vector<std::pair<std::string, std::string>> soft;
  int events = 0;
  std::optional<int> cache_cap;
  bool need_mcheck = true;

  bool wmut(std::uint32_t m) const { return (cfg.world_mutations & m) != 0; }
  bool checking() const { return M.checked; }

  /// The request index of (keys, mode), appended when new (scenarios.py's req_index).
  int req_index(const key_list &keys, lock_mode mode) {
    auto ck = canonical_keys(keys);
    for (std::size_t i = 0; i < cfg.requests.size(); ++i)
      if (canonical_keys(cfg.requests[i].first) == ck && cfg.requests[i].second == mode)
        return int(i);
    cfg.requests.push_back({ck, mode});
    return int(cfg.requests.size() - 1);
  }

  // -- enabled actions (world.py's enabled)
  // --------------------------------------------------------
  bool idle(int hid) const {
    const auto &h = homes.at(std::size_t(hid));
    return !h.destroyed && h.pending.empty() && h.attached;
  }

  std::vector<action> enabled() const {
    std::vector<action> acts;
    if (!cache_cap) {
      for (int c : cfg.cache)
        acts.push_back(act(ak::cachecap, c));
      return acts;
    }
    for (const auto &h : homes) {
      if (h.destroyed)
        continue;
      if (!h.pending.empty()) {
        acts.push_back(act(ak::hstep, h.hid));
        continue;
      }
      if (!h.port.empty())
        acts.push_back(act(ak::post, h.hid));
      if (cfg.destroy)
        acts.push_back(act(ak::destroy, h.hid));
      if (cfg.cache_clear && !h.cache.empty())
        acts.push_back(act(ak::cclear, h.hid));
    }
    for (const auto &[pid, p] : procs) {
      if (p.status == pstatus::ended || !idle(p.hid))
        continue;
      if (cfg.foreign)
        foreign_acts(p, acts);
      if (p.prio_left > 0)
        for (int v : cfg.prio_values)
          if (v != p.priority)
            acts.push_back(act(ak::prio, pid, v));
      for (bool ab : cfg.kills)
        acts.push_back(act(ak::kill, pid, ab ? 1 : 0));
      if (p.status == pstatus::parked) {
        if (p.lock_deadline)
          acts.push_back(act(ak::deadline, pid));
        continue;
      }
      std::size_t base = p.in_handler ? *p.hbase : 0;
      const frame *top = p.frames.size() > base ? &p.frames.back() : nullptr;
      if (top && top->enter) {
        acts.push_back(act(ak::enter_step, pid));
        acts.push_back(act(ak::raise, pid));
      } else {
        int depth = int(p.frames.size() - base);
        int lim = p.in_handler ? cfg.max_hdepth : cfg.max_depth;
        if (p.req_left > 0 && depth < lim)
          for (std::size_t ri = 0; ri < cfg.requests.size(); ++ri)
            for (auto kind : cfg.kinds)
              acts.push_back(act(ak::req, pid, int(ri), std::uint64_t(kind)));
        if (top) {
          acts.push_back(act(ak::finish, pid));
          acts.push_back(act(ak::raise, pid));
        }
      }
      if (!p.in_handler && p.sig_left > 0)
        acts.push_back(act(ak::signal, pid));
      if (p.in_handler) {
        if (p.frames.size() == *p.hbase)
          acts.push_back(act(ak::hend, pid));
        acts.push_back(act(ak::hraise, pid));
      }
    }
    return acts;
  }

  void foreign_acts(const proc &p, std::vector<action> &out) const {
    for (const auto &[i, h] : M.spec.holds)
      if (h.pid != p.P())
        out.push_back(act(ak::frelease, p.pid, 0, i));
    for (const auto &[t, T] : M.spec.tickets)
      if (T.pid != p.P())
        out.push_back(act(ak::fcancel, p.pid, 0, t));
    out.push_back(act(ak::frelease, p.pid, 0, M.spec.next_hold + 1));
    out.push_back(act(ak::fcancel, p.pid, 0, M.spec.next_seq + 1));
  }

  std::string describe(const action &a) const {
    switch (a.k) {
    case ak::req: {
      const auto &r = cfg.requests.at(std::size_t(a.b));
      return "req p" + std::to_string(a.a) + " " + req_str(r.first, r.second) + " " +
             rkind_name(rkind(a.c));
    }
    case ak::cachecap:
      return "key caches of " + std::to_string(a.a) + " entries";
    case ak::cclear:
      return "H" + std::to_string(a.a) + " clears its key cache " +
             keys_br(homes.at(std::size_t(a.a)).cache);
    case ak::hstep:
      return "H" + std::to_string(a.a) + ": " +
             (homes.at(std::size_t(a.a)).pending.empty()
                  ? std::string("<nothing>")
                  : pending_str(homes.at(std::size_t(a.a)).pending.front()));
    case ak::post:
      return "H" + std::to_string(a.a) + " applies post " +
             (homes.at(std::size_t(a.a)).port.empty()
                  ? std::string("<none>")
                  : post_str(homes.at(std::size_t(a.a)).port.front()));
    case ak::destroy:
      return "destroy H" + std::to_string(a.a);
    case ak::enter_step:
      return "enter_step p" + std::to_string(a.a);
    case ak::finish:
      return "finish p" + std::to_string(a.a);
    case ak::raise:
      return "raise p" + std::to_string(a.a);
    case ak::hraise:
      return "hraise p" + std::to_string(a.a);
    case ak::signal:
      return "signal p" + std::to_string(a.a);
    case ak::hend:
      return "hend p" + std::to_string(a.a);
    case ak::prio:
      return "prio p" + std::to_string(a.a) + " " + std::to_string(a.b);
    case ak::kill:
      return std::string(a.b ? "kill p" : "terminate p") + std::to_string(a.a);
    case ak::deadline:
      return "deadline p" + std::to_string(a.a);
    case ak::frelease:
      return "foreign p" + std::to_string(a.a) + " release hold " + std::to_string(a.c);
    case ak::fcancel:
      return "foreign p" + std::to_string(a.a) + " cancel ticket " + std::to_string(a.c);
    }
    return "?";
  }

  // -- the step relation
  // ---------------------------------------------------------------------------
  void do_(const action &a) {
    if (costs_event(a.k))
      ++events;
    switch (a.k) {
    case ak::cachecap:
      cache_cap = a.a;
      return;
    case ak::hstep:
      a_hstep(a.a);
      return;
    case ak::post:
      a_post(a.a);
      return;
    case ak::destroy:
      a_destroy(a.a);
      return;
    case ak::cclear:
      a_cclear(a.a);
      return;
    case ak::req:
      a_req(a.a, a.b, rkind(a.c));
      return;
    case ak::enter_step:
      a_enter_step(a.a);
      return;
    case ak::finish:
    case ak::raise:
      a_finish(a.a);
      return;
    case ak::hraise:
      a_hraise(a.a);
      return;
    case ak::signal:
      a_signal(a.a);
      return;
    case ak::hend:
      a_hend(a.a);
      return;
    case ak::prio:
      a_prio(a.a, a.b);
      return;
    case ak::kill:
      a_kill(a.a, a.b != 0);
      return;
    case ak::deadline:
      a_deadline(a.a);
      return;
    case ak::frelease:
    case ak::fcancel:
      a_foreign(a.a, a.k == ak::frelease, a.c);
      return;
    }
  }

  // -- the state, for the fingerprint
  // ----------------------------------------------------------------
  /// The world's state: the manager's canonical dump, then every field world.py's key() has.
  std::string dump() {
    std::string o = M.canonical();
    if (cfg.coalesce) {
      // 4.3's coalescing flags: in World.key() (Rec.key()), not in the canonical dump of 2.1
      o += "rl[";
      for (const auto &[P, r] : M.spec.recs)
        if (r.rl)
          o += pid_str(P);
      o += "]\n";
    }
    o += "world cap=" + (cache_cap ? std::to_string(*cache_cap) : std::string("?")) + "\n";
    for (const auto &h : homes) {
      o += "H" + std::to_string(h.hid) + " p" + (h.prio ? "1" : "0") + " c" +
           (h.closed ? "1" : "0") + " d" + (h.destroyed ? "1" : "0") + " a" +
           (h.attached ? "1" : "0") + " port[";
      for (const auto &p : h.port)
        o += post_str(p) + ";";
      o += "] cache" + keys_br(h.cache) + " pending[";
      for (const auto &op : h.pending)
        o += pending_str(op) + ";";
      o += "]\n";
    }
    for (const auto &[pid, p] : procs) {
      o += "p" + std::to_string(pid) + " h" + std::to_string(p.hid) + " " +
           (p.status == pstatus::run      ? "run"
            : p.status == pstatus::parked ? "parked"
                                          : "ended") +
           " pr" + std::to_string(p.priority) + " in" + inh_str(p.inherited) + " L[";
      for (const auto &h : p.locks)
        o += std::to_string(h.id) + ":" + keys_tok(h.keys) + ":" + mode_char(h.mode) + ":" +
             std::to_string(h.count) + ":" + std::to_string(h.gen) + ":" +
             (h.abandoned ? "1" : "0") + (h.in_handler ? "1" : "0") + ";";
      o += "] t" + (p.lock_ticket ? std::to_string(*p.lock_ticket) : std::string("-")) + " w" +
           (p.lock_waiting ? "1" : "0") + " dl" + (p.lock_deadline ? "1" : "0") + " ih" +
           (p.in_handler ? "1" : "0") + " hb" +
           (p.hbase ? std::to_string(*p.hbase) : std::string("-")) + " F[";
      for (const auto &f : p.frames)
        o += std::string(f.enter ? "E" : "B") + ":" +
             (f.hold ? std::to_string(*f.hold) : std::string("-")) + ":" + keys_tok(f.keys) + ":" +
             mode_char(f.mode) + ":" + outcome_str(f.outcome) + ":" + (f.deadline ? "1" : "0") +
             (f.ih ? "1" : "0") + ":" + (f.tid ? std::to_string(*f.tid) : std::string("-")) + ";";
      o += "] r" + std::to_string(p.req_left) + " s" + std::to_string(p.sig_left) + " q" +
           std::to_string(p.prio_left) + " pc" + std::to_string(p.pc) + " hpc" +
           std::to_string(p.hpc) + "\n";
    }
    return o;
  }

  // -- invariants (world.py's check)
  // -------------------------------------------------------------------
  void check() {
    auto &sp = M.spec;
    if (need_mcheck) {
      M.check_now();
      need_mcheck = false;
    }
    // I10: no hold or ticket outlives its process
    for (const auto &[i, h] : sp.holds) {
      auto p = procs.find(h.pid.pid);
      if (p == procs.end() || p->second.status == pstatus::ended)
        fail("I10", "hold " + std::to_string(i) + " of ended process " + pid_str(h.pid));
    }
    for (const auto &[t, T] : sp.tickets) {
      auto p = procs.find(T.pid.pid);
      if (p == procs.end() || p->second.status == pstatus::ended)
        fail("I10", "ticket " + std::to_string(t) + " of ended process " + pid_str(T.pid));
    }
    for (const auto &home : homes) {
      home_id hid = home_id(home.hid);
      if (!sp.homes.count(hid)) {
        if (sp.cache_refs.count(hid))
          fail("I9", "detached home H" + std::to_string(hid) + " still has key-cache references");
      } else {
        // I4 home side: the cache holds exactly the keys of its entry, one reference each
        auto d = sp.cache_refs.find(hid);
        key_list want;
        bool ones = true;
        if (d != sp.cache_refs.end())
          for (const auto &[k, c] : d->second) {
            want.push_back(k);
            ones = ones && c == 1;
          }
        key_list have = home.cache;
        std::sort(have.begin(), have.end(), key_cmp{});
        if (d == sp.cache_refs.end() || have != want || !ones)
          fail("I4", "H" + std::to_string(hid) + " key cache " + keys_br(home.cache) +
                         " != its references " + keys_br(want));
        M.check_home_cache(hid, home.cache);
      }
      if (home.destroyed || !home.pending.empty())
        continue;
      for (auto &[pid, p] : procs)
        if (p.hid == home.hid)
          check_proc(p);
    }
    // I8 base clause: rec.base == p.priority
    for (const auto &[P, r] : sp.recs) {
      auto p = procs.find(P.pid);
      if (p != procs.end() && p->second.hid == int(P.home) && r.base != p->second.priority)
        fail("I8", "rec " + pid_str(P) + " base " + std::to_string(r.base) + " != p.priority " +
                       std::to_string(p->second.priority));
    }
    // I9: once end has been called for every process, or every home is detached
    bool all_ended = true;
    for (const auto &kv : procs)
      all_ended = all_ended && kv.second.status == pstatus::ended;
    if (sp.homes.empty() || all_ended)
      check_empty();
    if (quiescent())
      check_quiescent();
  }

  bool quiescent() const {
    for (const auto &h : homes)
      if (!h.destroyed && (!h.pending.empty() || !h.port.empty()))
        return false;
    return true;
  }

  void check_proc(proc &p) {
    auto &sp = M.spec;
    lock_pid P = p.P();
    auto rec = sp.recs.find(P);
    if (p.status == pstatus::ended) {
      if (rec != sp.recs.end())
        fail("I3", "ended process " + pid_str(P) + " has a rec");
      return;
    }
    bool has = !p.locks.empty() || p.lock_ticket.has_value();
    if (has != (rec != sp.recs.end()))
      fail("I3", "process " + pid_str(P) + ": mirror/ticket " + (has ? "yes" : "no") + " but rec " +
                     (rec != sp.recs.end() ? "yes" : "no"));
    if (rec == sp.recs.end()) {
      for (const auto &f : p.frames)
        if (!f.enter && f.hold)
          fail("mirror", pid_str(P) + " has a lock_body frame with a hold but no rec");
    } else {
      std::set<std::uint64_t> exp = rec->second.holds;
      if (rec->second.ticket) {
        const auto &T = sp.tickets.at(*rec->second.ticket);
        if (T.state == ticket_state::granted)
          exp.erase(*T.hold);
      }
      std::set<std::uint64_t> mine;
      for (const auto &h : p.locks)
        mine.insert(h.id);
      if (mine != exp)
        fail("I3", "process " + pid_str(P) + " mirror " + join_u64(mine) +
                       " != rec holds less granted " + join_u64(exp));
      if (rec->second.ticket) {
        std::uint64_t t = *rec->second.ticket;
        if (p.lock_ticket != t || !p.lock_waiting)
          fail("I2", "ticket " + std::to_string(t) + " of " + pid_str(P) +
                         " exists but the process is not parked on it");
        const auto &T = sp.tickets.at(t);
        if (T.state == ticket_state::granted && !wake_in_flight(p.hid, p.pid, T.seq))
          fail("I2", "granted ticket " + std::to_string(T.seq) + " of " + pid_str(P) +
                         " with no wake in flight (lost wakeup)");
        if (T.state != ticket_state::queued && T.state != ticket_state::granted)
          fail("I2", "ticket " + std::to_string(T.seq) + " of " + pid_str(P) + " in state " +
                         state_name(T.state) + " between actions");
      }
    }
    if (p.lock_ticket &&
        (rec == sp.recs.end() || !rec->second.ticket || *rec->second.ticket != *p.lock_ticket))
      fail("I2", pid_str(P) + " parked on ticket " + std::to_string(*p.lock_ticket) +
                     " the manager does not have");
    if ((p.status == pstatus::parked) != p.lock_waiting)
      fail("model", "status/lock_waiting mismatch for " + pid_str(P));
    // mirror counts against frames
    for (const auto &h : p.locks) {
      int n = 0;
      for (const auto &f : p.frames) {
        if (!f.enter && f.hold && *f.hold == h.id)
          ++n;
        if (f.enter && f.outcome.k == outcome_t::granted && f.outcome.hold == h.id) {
          ++n;
          if (h.count != 1)
            fail("I2", "lock_enter frame granted " + std::to_string(h.id) + " with count " +
                           std::to_string(h.count));
          auto mh = sp.holds.find(h.id);
          if (mh == sp.holds.end() || mh->second.started)
            fail("I2", "lock_enter frame granted " + std::to_string(h.id) +
                           " whose hold started or is gone");
        }
      }
      if (n != h.count)
        fail("mirror", pid_str(P) + " hold " + std::to_string(h.id) + " count " +
                           std::to_string(h.count) + " but " + std::to_string(n) +
                           " frames refer to it (leak)");
      auto mh = sp.holds.find(h.id);
      if (mh == sp.holds.end() || mh->second.keys != h.keys || mh->second.mode != h.mode ||
          mh->second.pid != P)
        fail("mirror",
             pid_str(P) + " mirror hold " + std::to_string(h.id) + " does not match the manager");
      M.check_hold_keys(h.id, h.keys);
    }
    for (const auto &f : p.frames)
      if (!f.enter && f.hold) {
        bool found = false;
        for (const auto &h : p.locks)
          found = found || h.id == *f.hold;
        if (!found)
          fail("mirror", pid_str(P) + " frame refers to hold " + std::to_string(*f.hold) +
                             " not in its mirror");
      }
    if (p.lock_waiting) {
      const frame *f = p.frames.empty() ? nullptr : &p.frames.back();
      if (!f || !f->enter || f->outcome.k != outcome_t::none)
        fail("model", "parked " + pid_str(P) + " without a pending lock_enter frame");
    }
  }

  bool wake_in_flight(int hid, int pid, std::uint64_t tid) const {
    const auto &h = homes.at(std::size_t(hid));
    for (const auto &p : h.port)
      if (p.wake && p.pid == pid && p.tid == tid)
        return true;
    for (const auto &oh : homes)
      for (const auto &op : oh.pending)
        if (op.k == pk::dlv && op.tgt == hid && op.post.wake && op.post.pid == pid &&
            op.post.tid == tid)
          return true;
    return false;
  }

  void check_quiescent() {
    auto &sp = M.spec;
    for (const auto &[pid, p] : procs) {
      if (p.status == pstatus::ended)
        continue;
      lock_pid P = p.P();
      if (p.lock_waiting) {
        auto T = sp.tickets.find(*p.lock_ticket);
        if (T == sp.tickets.end() || T->second.state != ticket_state::queued)
          fail("I6", "quiescent, " + pid_str(P) + " parked on a non-queued ticket " +
                         std::to_string(*p.lock_ticket) + " (lost wakeup)");
      }
      // I8 quiescent clause (a pure read: the checker must not clear the coalescing flag)
      if (sp.applies(P)) {
        int v = sp.peek_inh(P);
        if (p.inherited != v)
          fail("I8", "quiescent, " + pid_str(P) + " inherited_priority " + inh_str(p.inherited) +
                         " != read_inh " + inh_str(v));
      } else if (p.inherited != INF) {
        fail("I8", "quiescent, " + pid_str(P) +
                       " on a home without applies has inherited_priority " + inh_str(p.inherited));
      }
    }
    // stuck: every sink a waiter ultimately waits for can run (4.13's blocking_sinks, whose
    // progress cells are L3's, is this set)
    auto g = sp.graph();
    for (const auto &kv : g) {
      for (auto Q : spec_manager::reach_in(g, {kv.first})) {
        auto it = g.find(Q);
        if ((it != g.end() && !it->second.empty()) || Q == kv.first)
          continue;
        const auto &q = procs.at(Q.pid);
        const auto &hq = homes.at(Q.home);
        if (q.status != pstatus::run || hq.destroyed || !hq.attached || !sp.homes.count(Q.home))
          fail("stuck", "quiescent: waiter " + pid_str(kv.first) + " leads to sink " + pid_str(Q) +
                            " which cannot run");
      }
    }
    check_query();
  }

  void check_empty() {
    auto &sp = M.spec;
    if (!sp.recs.empty() || !sp.holds.empty() || !sp.tickets.empty() || !sp.queue.empty())
      fail("I9", "every process ended (or every home detached) but the manager is not empty");
    M.check_empty_table();
  }

  void check_query() {
    auto &sp = M.spec;
    std::set<std::pair<int, std::string>> viewers;
    for (const auto &[pid, p] : procs)
      if (sp.homes.count(home_id(p.hid)))
        viewers.insert({p.hid, p.owner});
    for (const auto &[H, owner] : viewers) {
      auto r = M.query("", home_id(H), owner);
      for (const auto &e : r.held) {
        std::vector<const spec_hold *> hs;
        for (const auto &[i, h] : sp.holds)
          if (contains(h.keys, e.key))
            hs.push_back(&h);
        if (hs.empty() || e.count != hs.size())
          fail("4.13", "lock-query count " + std::to_string(e.count) + " for " + e.key + ", " +
                           std::to_string(hs.size()) + " holds");
        // the largest generation among the X holds with the key (I15), else nil
        std::optional<std::uint64_t> mx;
        for (const auto *h : hs)
          if (h->mode == X)
            mx = mx ? std::max(*mx, h->gen) : h->gen;
        if (e.mode != (mx ? X : S) || e.generation != mx)
          fail("4.13", "lock-query of " + e.key + " shows mode " + mode_char(e.mode) +
                           " generation " +
                           (e.generation ? std::to_string(*e.generation) : std::string("nil")) +
                           "; the latest X grant on it has generation " +
                           (mx ? std::to_string(*mx) : std::string("nil")));
        for (int pid : e.pids) {
          lock_pid Q{home_id(H), pid};
          bool holder = false;
          for (const auto *h : hs)
            holder = holder || h->pid == Q;
          if (!holder || sp.recs.at(Q).owner != owner)
            fail("4.13", "lock-query by H" + std::to_string(H) + "/" + owner + " shows pid " +
                             std::to_string(pid) + " for " + e.key +
                             ", not a holder of its home and owner");
        }
      }
      for (const auto &w : r.waiting)
        if (w.pid) {
          lock_pid Q{home_id(H), *w.pid};
          auto rq = sp.recs.find(Q);
          const spec_ticket *T = nullptr;
          if (rq != sp.recs.end() && rq->second.ticket)
            T = &sp.tickets.at(*rq->second.ticket);
          if (!T || T->state != ticket_state::queued || T->keys != w.keys ||
              rq->second.owner != owner)
            fail("4.13", "lock-query by H" + std::to_string(H) + "/" + owner +
                             " shows waiting pid " + std::to_string(*w.pid) +
                             " that is not its own");
        }
    }
  }

  // -- the fair drain (world.py's drain)
  // -------------------------------------------------------------------
  void drain(int limit = 10000) {
    int steps = 0;
    while (steps < limit) {
      ++steps;
      std::optional<action> a;
      for (const auto &h : homes)
        if (!h.destroyed && !h.pending.empty()) {
          a = act(ak::hstep, h.hid);
          break;
        }
      if (!a)
        for (const auto &h : homes)
          if (!h.destroyed && !h.port.empty()) {
            a = act(ak::post, h.hid);
            break;
          }
      if (!a)
        for (const auto &[pid, p] : procs) {
          if (p.status != pstatus::run)
            continue;
          std::size_t base = p.in_handler ? *p.hbase : 0;
          if (p.frames.size() > base) {
            a = act(p.frames.back().enter ? ak::enter_step : ak::finish, pid);
            break;
          }
          if (p.in_handler) {
            a = act(ak::hend, pid);
            break;
          }
        }
      if (!a)
        break;
      do_(*a);
      check();
    }
    for (const auto &[pid, p] : procs)
      if (p.status == pstatus::parked)
        fail("liveness", "after every body finished, " + pid_str(p.P()) +
                             " is still parked on ticket " +
                             (p.lock_ticket ? std::to_string(*p.lock_ticket) : std::string("-")));
    std::vector<int> pids;
    for (const auto &kv : procs)
      pids.push_back(kv.first);
    for (int pid : pids) {
      auto &p = procs.at(pid);
      if (p.status != pstatus::ended) {
        do_(act(ak::kill, pid, 0));
        check();
        while (!homes.at(std::size_t(p.hid)).pending.empty()) {
          do_(act(ak::hstep, p.hid));
          check();
        }
      }
    }
    while (!quiescent())
      for (const auto &h : homes)
        if (!h.destroyed && (!h.pending.empty() || !h.port.empty())) {
          do_(act(h.pending.empty() ? ak::post : ak::hstep, h.hid));
          check();
        }
    check();
    auto &sp = M.spec;
    if (!sp.recs.empty() || !sp.holds.empty() || !sp.tickets.empty() || !sp.queue.empty())
      fail("I9", "after every process ended the manager is not empty");
  }

private:
  static lockstep::options lockstep_options(config &c) {
    if (c.real && c.homes.size() > 1 && !table_accepts_homes(c.homes.size())) {
      c.real = false;
      c.real_dropped = true;
    }
    lockstep::options o;
    o.real = c.real;
    o.inheritance = c.inheritance;
    o.coalesce = c.coalesce;
    o.one_home = c.one_home;
    o.mutations = c.mutations;
    o.real_mutations = c.real_mutations;
    o.spec_mutations = c.spec_mutations;
    return o;
  }

  // -- mcall: a manager operation, then the transition checks (I2, I10)
  // -----------------------------------
  struct before_t {
    std::map<std::uint64_t, ticket_state> tickets;
    std::set<std::uint64_t> holds;
  };
  before_t snap() const {
    before_t b;
    if (!checking())
      return b;
    for (const auto &[t, T] : M.spec.tickets)
      b.tickets[t] = T.state;
    for (const auto &kv : M.spec.holds)
      b.holds.insert(kv.first);
    return b;
  }
  void transitions(const char *op, const before_t &b) {
    if (!checking())
      return;
    auto &sp = M.spec;
    auto is = [&](const char *a) { return std::strcmp(op, a) == 0; };
    for (const auto &[t, st] : b.tickets) {
      auto T = sp.tickets.find(t);
      if (T == sp.tickets.end()) {
        if (!is("resume") && !is("end") && !is("detach_home"))
          fail("I2", "ticket " + std::to_string(t) + " deleted by " + op);
      } else if (T->second.state != st &&
                 !(st == ticket_state::queued && (T->second.state == ticket_state::granted ||
                                                  T->second.state == ticket_state::cancelled))) {
        fail("I2", "ticket " + std::to_string(t) + " moved " + state_name(st) + " -> " +
                       state_name(T->second.state) + " in " + op);
      }
    }
    for (const auto &[t, T] : sp.tickets)
      if (!b.tickets.count(t) &&
          (!is("block") || !(T.state == ticket_state::queued || T.state == ticket_state::granted ||
                             T.state == ticket_state::failed)))
        fail("I2", "ticket " + std::to_string(t) + " created " + state_name(T.state) + " by " + op);
    for (auto i : b.holds)
      if (!sp.holds.count(i) && !is("release") && !is("end") && !is("detach_home"))
        fail("I10", "hold " + std::to_string(i) + " dropped by " + op);
  }

  std::vector<pending_op> dlv_ops(const spec_notes &N) const {
    std::vector<pending_op> ops;
    for (const auto &n : N) {
      pending_op o;
      o.k = pk::dlv;
      o.tgt = int(n.home);
      o.post = post_t{n.post.what == dt::lock_post::kind::wake, n.post.pid, n.post.tid};
      ops.push_back(o);
    }
    return ops;
  }
  void push_front(int hid, const std::vector<pending_op> &ops) {
    auto &q = homes.at(std::size_t(hid)).pending;
    q.insert(q.begin(), ops.begin(), ops.end());
  }
  void push_back(int hid, const pending_op &op) {
    homes.at(std::size_t(hid)).pending.push_back(op);
  }

  /// 4.15: the home probes its key cache; a miss adds an entry (and its reference) and an entry
  /// over capacity is evicted (dropping its reference), in the operation's own critical section.
  struct plan_t {
    key_list cache, add, drop;
  };
  plan_t cache_plan(int hid, const key_list &keys) const {
    plan_t pl;
    int cap = cache_cap.value_or(0);
    if (!cap)
      return pl;
    pl.cache = homes.at(std::size_t(hid)).cache;
    if (cfg.pin) {
      // as lock_home builds a request: every key is looked up first, each hit pinned; then each
      // miss takes a free entry or evicts the oldest unpinned one, and its new entry is pinned
      key_list pinned;
      for (const auto &k : keys)
        if (contains(pl.cache, k))
          pinned.push_back(k);
      for (const auto &k : keys) {
        if (contains(pinned, k))
          continue; // a hit
        if (int(pl.cache.size()) < cap) {
          pl.cache.push_back(k);
          pl.add.push_back(k);
          pinned.push_back(k);
          continue;
        }
        auto victim = std::find_if(pl.cache.begin(), pl.cache.end(),
                                   [&](const std::string &e) { return !contains(pinned, e); });
        if (victim == pl.cache.end())
          continue; // every entry pinned: the key passes uncached
        pl.drop.push_back(*victim);
        pl.cache.erase(victim);
        pl.cache.push_back(k);
        pl.add.push_back(k);
        pinned.push_back(k);
      }
    } else {
      for (const auto &k : keys) {
        if (contains(pl.cache, k))
          continue;
        pl.cache.push_back(k);
        pl.add.push_back(k);
        while (int(pl.cache.size()) > cap) {
          pl.drop.push_back(pl.cache.front());
          pl.cache.erase(pl.cache.begin());
        }
      }
    }
    if (wmut(cache_miss_unreferenced)) {
      // the home caches a miss without its reference: I4 (home side)
      pl.add.clear();
      key_list d;
      auto e = M.spec.cache_refs.find(home_id(hid));
      for (const auto &k : pl.drop)
        if (e != M.spec.cache_refs.end() && e->second.count(k))
          d.push_back(k);
      pl.drop = d;
    }
    return pl;
  }

  // -- home steps
  // -----------------------------------------------------------------------------------------
  void a_hstep(int hid) {
    auto &h = homes.at(std::size_t(hid));
    pending_op op = h.pending.front();
    h.pending.pop_front();
    run_op(hid, op);
    // private micro-operations commute with every other home's action: run them at once
    while (!homes.at(std::size_t(hid)).destroyed && !homes.at(std::size_t(hid)).pending.empty() &&
           local_op(homes.at(std::size_t(hid)).pending.front().k)) {
      pending_op o2 = homes.at(std::size_t(hid)).pending.front();
      homes.at(std::size_t(hid)).pending.pop_front();
      run_op(hid, o2);
    }
  }

  void run_op(int hid, const pending_op &op) {
    switch (op.k) {
    case pk::block:
      op_block(hid, op.pid);
      return;
    case pk::dlv:
      op_dlv(hid, op.tgt, op.post);
      return;
    case pk::after_block:
      op_after_block(op.pid, op.tid, ticket_state(op.st));
      return;
    case pk::wake_self:
      op_wake_self(hid, op.pid, op.tid, op.st);
      return;
    case pk::unwind: {
      auto &p = procs.at(op.pid);
      frame f = p.frames.back();
      p.frames.pop_back();
      release_lock_frame(hid, p, f);
      return;
    }
    case pk::exit_handler: {
      auto &p = procs.at(op.pid);
      if (!p.hbase || p.frames.size() != *p.hbase)
        fail("model", "exit handler with handler frames");
      p.in_handler = false;
      p.hbase.reset();
      return;
    }
    case pk::end_clear:
      end_clear(procs.at(op.pid));
      return;
    case pk::destroy_clear: {
      auto &h = homes.at(std::size_t(hid));
      h.port.clear();
      h.cache.clear();
      for (auto &[pid, p] : procs)
        if (p.hid == hid) {
          p.status = pstatus::ended;
          end_clear(p);
        }
      h.destroyed = true;
      return;
    }
    case pk::defensive_resume: {
      auto &p = procs.at(op.pid);
      auto b = snap();
      M.resume(p.P(), op.tid);
      transitions("resume", b);
      p.lock_ticket.reset();
      p.lock_waiting = false;
      p.lock_deadline = false;
      return;
    }
    case pk::detach:
      op_detach(hid);
      return;
    }
  }

  static void end_clear(proc &p) {
    p.locks.clear();
    p.lock_ticket.reset();
    p.lock_waiting = false;
    p.lock_deadline = false;
    p.inherited = INF;
    p.frames.clear();
    p.in_handler = false;
    p.hbase.reset();
  }

  void op_dlv(int hid, int tgt, const post_t &post) {
    // 4.14: to the caller's own home, applied at once; else port.post (one critical section)
    if (tgt == hid) {
      apply(hid, post);
      return;
    }
    auto &h = homes.at(std::size_t(tgt));
    if (h.closed)
      return; // dropped: a closed port refuses posts (2.3)
    if (h.destroyed)
      fail("port-race", "H" + std::to_string(hid) + " posts to H" + std::to_string(tgt) +
                            " after its scheduler was destroyed with the port open");
    h.port.push_back(post);
  }

  void op_block(int hid, int pid) {
    auto &p = procs.at(pid);
    if (p.frames.empty() || !p.frames.back().enter || p.frames.back().outcome.k != outcome_t::none)
      fail("model", "block without a pending lock_enter frame");
    frame &f = p.frames.back();
    auto pl = cache_plan(hid, f.keys);
    spec_notes N;
    auto b = snap();
    auto r = M.block(p.P(), p.owner, p.priority, f.keys, f.mode, pl.add, pl.drop, N);
    transitions("block", b);
    homes.at(std::size_t(hid)).cache = pl.cache;
    p.frames.back().tid = r.tid;
    auto ops = dlv_ops(N);
    pending_op ab;
    ab.k = pk::after_block;
    ab.pid = pid;
    ab.tid = r.tid;
    ab.st = int(r.state);
    ops.push_back(ab);
    push_front(hid, ops);
  }

  void op_after_block(int pid, std::uint64_t tid, ticket_state st) {
    auto &p = procs.at(pid);
    if (st == ticket_state::queued) {
      p.lock_ticket = tid;
      p.lock_waiting = true;
      p.lock_deadline = p.frames.back().deadline;
      p.status = pstatus::parked;
    } else {
      collect(p, p.frames.back(), tid);
    }
  }

  void op_wake_self(int hid, int pid, std::uint64_t tid, int st) {
    auto &p = procs.at(pid);
    apply_wake(hid, pid, tid);
    const frame &f = p.frames.back();
    // a timeout that finds a grant adopts it
    if (st == int(ticket_state::granted) && f.outcome.k != outcome_t::granted)
      fail("adopt", "cancel returned granted but the waiter did not adopt the grant");
    if (st == int(ticket_state::cancelled) && f.outcome.k != outcome_t::timeout)
      fail("adopt", "cancel cancelled but the outcome is " + outcome_str(f.outcome));
    if (st < 0)
      fail("adopt", "cancel found no ticket for a waiting process");
    if (p.lock_waiting)
      fail("adopt", "process still parked after its deadline fired");
  }

  void op_detach(int hid) {
    // 5.7: detach_home's precondition, asserted outside the operation
    if (!homes.at(std::size_t(hid)).closed)
      fail("5.7", "detach_home of H" + std::to_string(hid) + " called with its port open");
    spec_notes N;
    auto b = snap();
    M.detach_home(home_id(hid), N);
    transitions("detach_home", b);
    auto ops = dlv_ops(N);
    pending_op dc;
    dc.k = pk::destroy_clear;
    ops.push_back(dc);
    push_front(hid, ops);
  }

  // -- 5.9 posts
  // --------------------------------------------------------------------------------------------
  void a_post(int hid) {
    auto &h = homes.at(std::size_t(hid));
    post_t post = h.port.front();
    h.port.pop_front();
    apply(hid, post);
  }

  proc *proc_of_home(int hid, int pid) {
    auto it = procs.find(pid);
    if (it == procs.end() || it->second.hid != hid || it->second.status == pstatus::ended)
      return nullptr;
    return &it->second;
  }

  void apply(int hid, const post_t &post) {
    if (post.wake) {
      apply_wake(hid, post.pid, post.tid);
      return;
    }
    proc *p = proc_of_home(hid, post.pid);
    if (!p)
      return;
    auto b = snap();
    int v = M.read_inh(p->P());
    transitions("read_inh", b);
    if (p->inherited != v)
      p->inherited = v;
  }

  void apply_wake(int hid, int pid, std::uint64_t tid) {
    proc *p = proc_of_home(hid, pid);
    if (!p)
      return;
    if (wmut(wake_carries_outcome)) {
      if (!p->lock_waiting)
        return;
      if (p->lock_ticket != tid) {
        // the wake carries an outcome: it resumes whatever the process waits on now
        p->lock_ticket.reset();
        p->lock_waiting = false;
        p->lock_deadline = false;
        p->status = pstatus::run;
        return;
      }
    }
    if (p->lock_ticket != tid || !p->lock_waiting)
      return; // stale
    frame &f = p->frames.back();
    if (!f.enter || f.tid != tid)
      fail("model", "parked process without its lock_enter frame on top");
    if (!collect(*p, f, tid))
      return;
    p->lock_ticket.reset();
    p->lock_waiting = false;
    p->lock_deadline = false;
    p->status = pstatus::run;
  }

  bool collect(proc &p, frame &f, std::uint64_t tid) {
    auto b = snap();
    auto out = M.resume(p.P(), tid);
    transitions("resume", b);
    if (out.what == dt::lock_outcome::kind::queued)
      return false;
    if (out.what == dt::lock_outcome::kind::granted) {
      const auto &h = M.spec.holds.at(out.g.hold);
      if (h.keys != f.keys || h.mode != f.mode)
        fail("collect", "granted hold " + std::to_string(out.g.hold) + " " +
                            req_str(h.keys, h.mode) + " does not match the frame's request " +
                            req_str(f.keys, f.mode));
      p.locks.push_back(
          mhold{out.g.hold, f.keys, f.mode, 1, out.g.gen, out.g.abandoned, p.in_handler});
      f.outcome = outcome_t{outcome_t::granted, out.g.hold, {}};
    } else if (out.what == dt::lock_outcome::kind::timeout) {
      f.outcome = outcome_t{outcome_t::timeout, 0, {}};
    } else {
      f.outcome = outcome_t{outcome_t::deadlock, 0, out.cycle};
    }
    return true;
  }

  // -- 5.4 release_lock_frame
  // ----------------------------------------------------------------------------
  void release_lock_frame(int hid, proc &p, const frame &f) {
    std::optional<std::uint64_t> rid;
    if (!f.enter)
      rid = f.hold;
    else if (f.outcome.k == outcome_t::granted)
      rid = f.outcome.hold;
    std::vector<pending_op> ops;
    if (rid) {
      auto h = std::find_if(p.locks.begin(), p.locks.end(),
                            [&](const mhold &m) { return m.id == *rid; });
      if (h == p.locks.end())
        return;
      if (--h->count == 0) {
        std::uint64_t id = h->id;
        p.locks.erase(h);
        spec_notes N;
        auto b = snap();
        M.release(p.P(), id, N);
        transitions("release", b);
        ops = dlv_ops(N);
      }
    }
    if (f.enter && p.lock_ticket) {
      soft.push_back({"defensive-path", "release_lock_frame withdrew a ticket still pending"});
      std::uint64_t tid = *p.lock_ticket;
      spec_notes N2;
      auto b = snap();
      M.cancel(p.P(), tid, N2);
      transitions("cancel", b);
      auto more = dlv_ops(N2);
      ops.insert(ops.end(), more.begin(), more.end());
      pending_op dr;
      dr.k = pk::defensive_resume;
      dr.pid = p.pid;
      dr.tid = tid;
      ops.push_back(dr);
    }
    push_front(hid, ops);
  }

  // -- process actions
  // ------------------------------------------------------------------------------------
  void a_req(int pid, int ri, rkind kind) {
    auto &p = procs.at(pid);
    --p.req_left;
    if (p.in_handler)
      ++p.hpc;
    else
      ++p.pc;
    key_list keys = canonical_keys(cfg.requests.at(std::size_t(ri)).first);
    lock_mode mode = cfg.requests.at(std::size_t(ri)).second;
    int hid = p.hid;
    bool in_h = p.in_handler;
    std::vector<const mhold *> view;
    for (const auto &h : p.locks)
      if (!in_h || h.in_handler)
        view.push_back(&h);
    // 1. the handler rule
    if (in_h)
      for (const auto &k : keys)
        for (const auto &h : p.locks) {
          if (h.in_handler)
            continue;
          bool ov = false;
          for (const auto &hk : h.keys)
            ov = ov || overlap(hk, k);
          if ((under(h.keys, k) && (h.mode == X || mode == S)) || (ov && !compat(h.mode, mode)))
            return; // LockError("handler"): nothing changes
        }
    // 2. upgrade
    if (mode == X)
      for (const auto &k : keys) {
        bool s_under = false, x_under = false;
        for (const auto *h : view) {
          s_under = s_under || (h->mode == S && under(h->keys, k));
          x_under = x_under || (h->mode == X && under(h->keys, k));
        }
        if (s_under && !x_under)
          return; // LockError("upgrade")
      }
    // 3. covered
    bool covered = true;
    for (const auto &k : keys) {
      bool c = false;
      for (const auto *h : view)
        c = c || (under(h->keys, k) && (h->mode == X || mode == S));
      covered = covered && c;
    }
    if (covered) {
      mhold *c = nullptr;
      for (auto &h : p.locks)
        if ((!in_h || h.in_handler) && under(h.keys, keys[0]) && (h.mode == X || mode == S)) {
          c = &h;
          break;
        }
      if (checking())
        g_covered(p, keys, mode);
      ++c->count;
      frame f;
      f.hold = c->id;
      f.keys = keys;
      f.mode = mode;
      f.ih = in_h;
      p.frames.push_back(f);
      return;
    }
    // 4. the fast path
    std::string k_before = checking() ? M.lock_key() : std::string();
    auto pl = cache_plan(hid, keys);
    auto b = snap();
    auto r = M.try_acquire(p.P(), p.owner, p.priority, keys, mode, pl.add, pl.drop);
    transitions("try_acquire", b);
    homes.at(std::size_t(hid)).cache = pl.cache;
    if (!r && checking() && M.lock_key() != k_before)
      fail("I13", "a busy try_acquire changed the manager");
    if (r) {
      p.locks.push_back(mhold{r->hold, keys, mode, 1, r->gen, r->abandoned, in_h});
      frame f;
      f.hold = r->hold;
      f.keys = keys;
      f.mode = mode;
      f.ih = in_h;
      p.frames.push_back(f);
      return;
    }
    // 5. busy
    if (kind == rkind::ifavail) {
      frame f;
      f.keys = keys;
      f.mode = mode;
      f.ih = in_h;
      p.frames.push_back(f);
      return;
    }
    if (kind == rkind::t0)
      return; // TimeoutError
    frame f;
    f.enter = true;
    f.keys = keys;
    f.mode = mode;
    f.deadline = kind == rkind::wait_dl;
    f.ih = in_h;
    p.frames.push_back(f);
    pending_op op;
    op.k = pk::block;
    op.pid = pid;
    push_back(hid, op);
  }

  void g_covered(const proc &p, const key_list &keys, lock_mode mode) const {
    const auto &sp = M.spec;
    if (!sp.blk_hold(p.P(), keys, mode).empty())
      fail("covered", "covered request " + req_str(keys, mode) + " of " + pid_str(p.P()) +
                          " conflicts with another process's hold");
    for (auto t : sp.queue) {
      const auto &U = sp.tickets.at(t);
      if (U.pid != p.P() && conflicts(keys, mode, U.keys, U.mode) && !sp.exempt(p.P(), U))
        fail("covered", "covered request " + req_str(keys, mode) + " of " + pid_str(p.P()) +
                            " passes earlier non-exempt waiter " + pid_str(U.pid));
    }
  }

  void a_enter_step(int pid) {
    auto &p = procs.at(pid);
    frame f = p.frames.back();
    if (f.outcome.k == outcome_t::none)
      fail("I2", "lock_enter step with no outcome (woken without a grant)");
    if (f.outcome.k == outcome_t::granted) {
      auto b = snap();
      M.start_body(p.P(), f.outcome.hold);
      transitions("start_body", b);
      frame body;
      body.hold = f.outcome.hold;
      body.keys = f.keys;
      body.mode = f.mode;
      body.ih = f.ih;
      p.frames.back() = body;
    } else {
      p.frames.pop_back();
      release_lock_frame(p.hid, p, f);
    }
  }

  void a_finish(int pid) {
    auto &p = procs.at(pid);
    pending_op op;
    op.k = pk::unwind;
    op.pid = pid;
    push_back(p.hid, op);
    a_hstep(p.hid);
  }

  void a_hraise(int pid) {
    auto &p = procs.at(pid);
    std::size_t n = p.frames.size() - *p.hbase;
    pending_op uw;
    uw.k = pk::unwind;
    uw.pid = pid;
    for (std::size_t i = 0; i < n; ++i)
      push_back(p.hid, uw);
    pending_op eh;
    eh.k = pk::exit_handler;
    eh.pid = pid;
    push_back(p.hid, eh);
    if (*p.hbase > 0)
      push_back(p.hid, uw);
    a_hstep(p.hid);
  }

  void a_signal(int pid) {
    auto &p = procs.at(pid);
    --p.sig_left;
    p.in_handler = true;
    p.hbase = p.frames.size();
    p.hpc = 0;
  }

  void a_hend(int pid) {
    auto &p = procs.at(pid);
    p.in_handler = false;
    p.hbase.reset();
  }

  void a_prio(int pid, int v) {
    auto &p = procs.at(pid);
    --p.prio_left;
    p.priority = v;
    if ((!p.locks.empty() || p.lock_ticket) && !wmut(no_set_base)) {
      spec_notes N;
      auto b = snap();
      M.set_base(p.P(), v, N);
      transitions("set_base", b);
      push_front(p.hid, dlv_ops(N));
    }
  }

  void a_kill(int pid, bool abandon) {
    auto &p = procs.at(pid);
    p.status = pstatus::ended;
    if (p.locks.empty() && !p.lock_ticket) {
      end_clear(p);
      return;
    }
    spec_notes N;
    auto b = snap();
    M.end(p.P(), abandon, N);
    transitions("end", b);
    auto ops = dlv_ops(N);
    pending_op ec;
    ec.k = pk::end_clear;
    ec.pid = pid;
    ops.push_back(ec);
    push_front(p.hid, ops);
  }

  void a_deadline(int pid) {
    auto &p = procs.at(pid);
    std::uint64_t tid = *p.lock_ticket;
    spec_notes N;
    auto b = snap();
    auto st = M.cancel(p.P(), tid, N);
    transitions("cancel", b);
    auto ops = dlv_ops(N);
    pending_op ws;
    ws.k = pk::wake_self;
    ws.pid = pid;
    ws.tid = tid;
    ws.st = st ? int(*st) : -1;
    ops.push_back(ws);
    push_front(p.hid, ops);
  }

  void a_cclear(int hid) {
    // 4.15: the host clears the home's key cache: unref_nodes over every entry; the home's entry
    // in M.cache_refs stays, empty (I9)
    auto &h = homes.at(std::size_t(hid));
    key_list keys = h.cache;
    h.cache.clear();
    auto b = snap();
    M.unref_nodes(home_id(hid), keys);
    transitions("unref_nodes", b);
  }

  void a_foreign(int pid, bool release, std::uint64_t ident) {
    // I12: a frame of a restored copy (or any stale id) names another process's hold or ticket;
    // the owner check must leave the manager exactly as it was
    auto &p = procs.at(pid);
    std::string before = checking() ? M.lock_key() : std::string();
    spec_notes N;
    std::optional<ticket_state> r;
    auto b = snap();
    if (release) {
      M.release(p.P(), ident, N);
      transitions("release", b);
    } else {
      r = M.cancel(p.P(), ident, N);
      transitions("cancel", b);
    }
    if (checking() && (!N.empty() || r || M.lock_key() != before))
      fail("I12", std::string(release ? "release" : "cancel") + " by " + pid_str(p.P()) + " of " +
                      std::to_string(ident) + " (not its own) changed the manager");
  }

  void a_destroy(int hid) {
    // 5.7: ~lock_home first closes the port; detach_home is a later action of the same thread
    homes.at(std::size_t(hid)).closed = true;
    pending_op d;
    d.k = pk::detach;
    push_front(hid, {d});
  }
};

// =============================================================================================
// 5. The explorer: explore.py's 0-1 BFS
// =============================================================================================

struct fp128 {
  std::uint64_t lo = 0, hi = 0;
  bool operator==(const fp128 &o) const { return lo == o.lo && hi == o.hi; }
};
struct fp_hash {
  std::size_t operator()(const fp128 &f) const {
    return std::size_t(f.lo ^ (f.hi * 0x9E3779B97F4A7C15ull));
  }
};
inline std::uint64_t rotl64(std::uint64_t x, int r) { return (x << r) | (x >> (64 - r)); }
inline std::uint64_t fmix64(std::uint64_t k) {
  k ^= k >> 33;
  k *= 0xff51afd7ed558ccdull;
  k ^= k >> 33;
  k *= 0xc4ceb9fe1a85ec53ull;
  k ^= k >> 33;
  return k;
}
/// A 128-bit fingerprint of a state's dump (MurmurHash3 x64_128's mixing, the tail zero-padded).
fp128 fingerprint(const std::string &s) {
  const std::uint64_t c1 = 0x87c37b91114253d5ull, c2 = 0x4cf5ad432745937full;
  std::uint64_t h1 = 0x5bd1e995u, h2 = 0x1b873593u;
  const char *p = s.data();
  std::size_t n = s.size(), i = 0;
  auto mix = [&](std::uint64_t k1, std::uint64_t k2) {
    k1 *= c1;
    k1 = rotl64(k1, 31);
    k1 *= c2;
    h1 ^= k1;
    h1 = rotl64(h1, 27);
    h1 += h2;
    h1 = h1 * 5 + 0x52dce729;
    k2 *= c2;
    k2 = rotl64(k2, 33);
    k2 *= c1;
    h2 ^= k2;
    h2 = rotl64(h2, 31);
    h2 += h1;
    h2 = h2 * 5 + 0x38495ab5;
  };
  for (; i + 16 <= n; i += 16) {
    std::uint64_t k1, k2;
    std::memcpy(&k1, p + i, 8);
    std::memcpy(&k2, p + i + 8, 8);
    mix(k1, k2);
  }
  unsigned char tail[16] = {};
  std::memcpy(tail, p + i, n - i);
  std::uint64_t k1, k2;
  std::memcpy(&k1, tail, 8);
  std::memcpy(&k2, tail + 8, 8);
  mix(k1, k2);
  h1 ^= n;
  h2 ^= n;
  h1 += h2;
  h2 += h1;
  h1 = fmix64(h1);
  h2 = fmix64(h2);
  h1 += h2;
  h2 += h1;
  return fp128{h1, h2};
}

std::size_t peak_rss_mb() {
#if defined(_WIN32)
  PROCESS_MEMORY_COUNTERS pmc{};
  if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc)))
    return std::size_t(pmc.PeakWorkingSetSize / (1024 * 1024));
  return 0;
#else
  struct rusage ru {};
  getrusage(RUSAGE_SELF, &ru);
#if defined(__APPLE__)
  return std::size_t(ru.ru_maxrss / (1024 * 1024));
#else
  return std::size_t(ru.ru_maxrss / 1024);
#endif
#endif
}

struct found_t {
  std::string detail;
  std::vector<action> trace;
  int cost = 0;
  bool drain = false;
};

struct explore_options {
  int bound = 2;
  std::size_t max_states = 100000000;
  bool drain = true;
  bool force_replay = false;      // replay even without the real table (to measure its cost)
  bool stop_on_violation = false; // stop at the first violation (the BFS goes by cost)
};

struct explore_result {
  std::size_t states = 0, expanded = 0, transitions = 0;
  double seconds = 0;
  bool complete = true;
  std::map<std::string, found_t> violations, soft;
  std::string summary() const {
    std::ostringstream os;
    os << states << " states, " << expanded << " expanded, " << transitions << " transitions, "
       << seconds << " s" << (complete ? "" : " (incomplete)");
    return os.str();
  }
};

/// A violation's trace in words: the actions replayed into a fresh world of the oracle alone.
std::vector<std::string> render(config cfg, const found_t &f) {
  cfg.real = false;
  std::vector<std::string> out;
  try {
    world w(cfg);
    w.M.checked = false;
    for (const auto &a : f.trace) {
      out.push_back(w.describe(a));
      w.do_(a);
    }
  } catch (const std::exception &e) {
    out.push_back(std::string("  (the oracle alone stops here: ") + e.what() + ")");
  }
  if (f.drain)
    out.push_back("<fair drain: finish every body, apply every post>");
  out.push_back("  -> " + f.detail);
  return out;
}

std::string report(const config &cfg, const std::map<std::string, found_t> &table) {
  std::string o;
  for (const auto &[name, f] : table) {
    o += "violation " + name + " (" + std::to_string(f.cost) + " events):\n";
    for (const auto &line : render(cfg, f))
      o += "  " + line + "\n";
  }
  return o;
}

explore_result explore(const config &cfg, const explore_options &eo) {
  explore_result res;
  auto t0 = std::chrono::steady_clock::now();
  const bool replay = cfg.real || eo.force_replay;
  struct srec {
    std::uint32_t parent;
    action act;
    std::uint8_t cost;
  };
  std::vector<srec> st;
  std::unordered_map<fp128, std::uint32_t, fp_hash> index;
  std::deque<std::pair<std::uint32_t, std::uint8_t>> dq;

  auto path_of = [&](std::uint32_t s) {
    std::vector<action> path;
    while (st[s].parent != UINT32_MAX) {
      path.push_back(st[s].act);
      s = st[s].parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  };
  auto note = [&](std::map<std::string, found_t> &table, const std::string &name,
                  const std::string &detail, std::vector<action> trace, int cost, bool drain) {
    auto it = table.find(name);
    if (it == table.end() || cost < it->second.cost)
      table[name] = found_t{detail, std::move(trace), cost, drain};
  };
  auto rebuild = [&](const std::vector<action> &path) {
    world w(cfg);
    w.M.checked = false;
    for (const auto &a : path)
      w.do_(a);
    w.M.checked = true;
    w.need_mcheck = false;
    return w;
  };

  {
    world w0(cfg);
    w0.check();
    index.emplace(fingerprint(w0.dump()), 0u);
  }
  st.push_back({UINT32_MAX, action{}, 0});
  dq.push_back({0u, std::uint8_t(0)});

  while (!dq.empty()) {
    auto [s, cost] = dq.front();
    dq.pop_front();
    if (st[s].cost < cost)
      continue;
    if (++res.expanded > eo.max_states) {
      res.complete = false;
      break;
    }
    auto path = path_of(s);
    std::optional<world> base;
    try {
      base.emplace(rebuild(path));
    } catch (const not_built &) {
      throw;
    } catch (const lock_violation &v) {
      note(res.violations, v.invariant() + "(replay)", v.what(), path, cost, false);
      continue;
    }
    auto acts = base->enabled();
    if (eo.drain && cost == eo.bound) {
      try {
        if (replay) {
          world d = rebuild(path);
          d.drain();
        } else {
          world d = *base;
          d.drain();
        }
      } catch (const not_built &) {
        throw;
      } catch (const lock_violation &v) {
        note(res.violations, v.invariant() + "(drain)", v.what(), path, cost, true);
      } catch (const std::exception &e) {
        note(res.violations, "crash(drain)", e.what(), path, cost, true);
      }
    }
    std::vector<std::size_t> todo;
    for (std::size_t i = 0; i < acts.size(); ++i)
      if (cost + (costs_event(acts[i].k) ? 1 : 0) <= eo.bound)
        todo.push_back(i);
    for (std::size_t j = 0; j < todo.size(); ++j) {
      const action &a = acts[todo[j]];
      int nc = cost + (costs_event(a.k) ? 1 : 0);
      ++res.transitions;
      std::vector<action> trace = path;
      trace.push_back(a);
      std::optional<world> w2;
      try {
        if (!replay)
          w2.emplace(*base);
        else if (j + 1 == todo.size())
          w2.emplace(std::move(*base));
        else
          w2.emplace(rebuild(path));
        w2->do_(a);
        w2->check();
      } catch (const not_built &) {
        throw;
      } catch (const lock_violation &v) {
        note(res.violations, v.invariant(), v.what(), std::move(trace), nc, false);
        if (eo.stop_on_violation)
          break;
        continue;
      } catch (const std::exception &e) {
        note(res.violations, "crash", e.what(), std::move(trace), nc, false);
        if (eo.stop_on_violation)
          break;
        continue;
      }
      for (const auto &[sn, sd] : w2->soft)
        note(res.soft, sn, sd, trace, nc, false);
      auto fp = fingerprint(w2->dump());
      auto it = index.find(fp);
      if (it != index.end() && st[it->second].cost <= nc)
        continue;
      std::uint32_t id;
      if (it != index.end()) {
        id = it->second;
        st[id] = {s, a, std::uint8_t(nc)};
      } else {
        id = std::uint32_t(st.size());
        st.push_back({s, a, std::uint8_t(nc)});
        index.emplace(fp, id);
      }
      if (nc == cost)
        dq.push_front({id, std::uint8_t(nc)});
      else
        dq.push_back({id, std::uint8_t(nc)});
    }
    if (eo.stop_on_violation && !res.violations.empty()) {
      res.complete = false;
      break;
    }
  }
  res.states = index.size();
  res.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return res;
}

// =============================================================================================
// 6. Configurations, sizes, the real-table switch
// =============================================================================================

/// The real-table side: on by default; CVC_LOCKS_MODEL_REAL=0 runs the oracle alone (to develop
/// and validate the checker while the table is the interface commit's stub).
bool real_flag() {
  static const bool v = [] {
    const char *e = std::getenv("CVC_LOCKS_MODEL_REAL");
    return !(e && std::string(e) == "0");
  }();
  return v;
}
bool table_built() {
  static const bool v = [] {
    try {
      dt::lock_table t(dt::lock_table::options{});
      dt::lock_notes N;
      t.attach_home(false, nullptr, nullptr, N);
      return true;
    } catch (const std::logic_error &e) {
      return !is_stub(e);
    }
  }();
  return v;
}
bool use_real() { return real_flag() && table_built(); }
const char *const kStubSkip =
    "detail::lock_table is not built yet (it throws its \"not built yet\" logic_error, locks plan "
    "L1.2); CVC_LOCKS_MODEL_REAL=0 runs this test on the spec oracle alone";

#define LT5_NEEDS_TABLE()                                                                          \
  do {                                                                                             \
    if (real_flag() && !table_built())                                                             \
      GTEST_SKIP() << kStubSkip;                                                                   \
  } while (0)

int env_int(const char *name, int dflt) {
  const char *e = std::getenv(name);
  if (!e || !*e)
    return dflt;
  return std::atoi(e);
}
bool sanitizer_lane() {
  const char *e = std::getenv("CVC_LOCKS_SANITIZER");
  return e && std::string(e) == "1";
}
#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif
/// The regular sizes (locks plan L1 section 3, "Sizes, by lane"), measured in L1.3: see
/// tools/locks_model/README.md.
int regular_bound() {
  return env_int("CVC_LOCKS_MODEL_BOUND", sanitizer_lane() ? 2 : kRelease ? 3 : 2);
}
int regular_steps() {
  return env_int("CVC_LOCKS_MODEL_STEPS", sanitizer_lane() ? 2000 : kRelease ? 20000 : 5000);
}
int stress_bound() { return env_int("CVC_LOCKS_MODEL_BOUND", 4); }
int stress_steps() { return env_int("CVC_LOCKS_MODEL_STEPS", 200000); }

const std::string kA = "a", kAB = "a.b", kAC = "a.c", kB = "b";

/// explore.py's 'l1' exactly: one round_robin home of three processes, the requests X a, S a.b,
/// X a.c and X {a.b, b}, nesting depth 2, waits with deadlines and :if-available, kills and
/// terminations, the home's destruction, key caches of 0-3 entries (chosen by a first action
/// that costs nothing) and a host's cache clear.
config l1_config() {
  config c;
  c.name = "l1";
  c.homes = {false};
  c.procs = {{0, 1, "o1", 0}, {0, 2, "o1", 1}, {0, 3, "o2", -1}};
  c.requests = {{{kA}, X}, {{kAB}, S}, {{kAC}, X}, {{kAB, kB}, X}};
  c.kinds = {rkind::wait_dl, rkind::ifavail};
  c.kills = {true, false};
  c.destroy = true;
  c.req_per_proc = 2;
  c.cache = {0, 1, 2, 3};
  c.cache_choice = true;
  c.cache_clear = true;
  c.one_home = true;
  c.real = use_real();
  return c;
}

/// 'l1' with what locks plan L1 section 3 (LT-5 item 4) adds to it: one signal per process whose
/// handler may lock (nesting depth 1) and raise, and foreign release and cancel calls (I12).
config l1x_config() {
  config c = l1_config();
  c.name = "l1x";
  c.signals = 1;
  c.foreign = true;
  return c;
}

/// explore.py's 'l2' (protocol 7, phase L2): one priority_rr home of three processes of bases 0,
/// 2 and -2, the requests of 'l1', nesting depth 2, waits with deadlines, kills, the home's
/// destruction, one set-priority per process over -2..2, and key caches of 0-3 entries. Relevels
/// coalesce, the production setting (G-39); on one home every relevel is applied right after its
/// operation, so coalescing changes no notification list, and explore.py's states are the same
/// with it on or off.
config l2_config() {
  config c;
  c.name = "l2";
  c.homes = {true};
  c.procs = {{0, 1, "o1", 0}, {0, 2, "o1", 2}, {0, 3, "o2", -2}};
  c.requests = {{{kA}, X}, {{kAB}, S}, {{kAC}, X}, {{kAB, kB}, X}};
  c.kinds = {rkind::wait_dl};
  c.kills = {true};
  c.prio_values = {-2, -1, 0, 1, 2};
  c.prio_changes = 1;
  c.destroy = true;
  c.req_per_proc = 2;
  c.cache = {0, 1, 2, 3};
  c.cache_choice = true;
  c.coalesce = true;
  c.one_home = true;
  c.real = use_real();
  return c;
}

/// 'l2' with inheritance off (lock_manager::options::inheritance, the lock.inherit bench's off
/// points, design 6.2): nothing lends, every inh stays INF and no relevel is sent, while bases
/// change. explore.py's 'l2' with inheritance=False has its states.
config noinh1_config() {
  config c = l2_config();
  c.name = "noinh1";
  c.inheritance = false;
  return c;
}

/// explore.py's 'core' (L3's: two homes, one of them priority_rr), for the measurement test's
/// cross-check of the parts 'l1' never runs: posts between homes, inheritance and relevels.
config core_config() {
  config c;
  c.name = "core";
  c.homes = {true, false};
  c.procs = {{0, 1, "o1", 0}, {0, 2, "o1", 1}, {1, 3, "o2", -1}};
  c.requests = {{{kA}, X}, {{kAB}, S},     {{kAB}, X}, {{kAC}, X},
                {{kB}, X}, {{kAB, kB}, X}, {{kA}, S}};
  c.kinds = {rkind::wait_dl};
  c.kills = {true};
  c.req_per_proc = 2;
  c.pin = false;
  c.real = false;
  return c;
}

// =============================================================================================
// 7. Scenarios (scenarios.py), on one home unless they need posts between two homes
// =============================================================================================

/// Where mk puts a scenario's manager mutation: on both sides, as lockmodel.py's single Manager
/// takes it; on the real table alone, the oracle unmutated beside it; or on the oracle alone,
/// with no table (the explorer's oracle-only mode). In lockstep the side that throws first stops
/// the step before the other runs, so only a side run alone shows that it reports the violation
/// itself.
enum class mutation_side { both, table, oracle };
thread_local mutation_side g_mutation_side = mutation_side::both;

/// scenarios.py's mk: every request kind, nesting 3 (2 in a handler), two signals, kills and
/// terminations, priorities, five requests per process.
world mk(std::vector<bool> homes, std::vector<proc_cfg> procs, std::uint32_t mut = 0,
         std::uint32_t wmut = 0, int cache = 0, bool cache_clear = false, bool coalesce = false) {
  config c;
  c.name = "scenario";
  c.homes = std::move(homes);
  c.procs = std::move(procs);
  c.kinds = {rkind::wait, rkind::wait_dl, rkind::ifavail, rkind::t0};
  c.max_depth = 3;
  c.max_hdepth = 2;
  c.signals = 2;
  c.kills = {true, false};
  c.destroy = true;
  c.prio_values = {-10, -2, 0, 2, 5, 10};
  (g_mutation_side == mutation_side::both    ? c.mutations
   : g_mutation_side == mutation_side::table ? c.real_mutations
                                             : c.spec_mutations) = mut;
  c.world_mutations = wmut;
  c.prio_changes = 3;
  c.req_per_proc = 5;
  c.cache = {cache};
  c.cache_clear = cache_clear;
  c.coalesce = coalesce;
  c.one_home = c.homes.size() == 1;
  c.real = use_real() && g_mutation_side != mutation_side::oracle;
  return world(std::move(c));
}

action R(world &w, int pid, key_list keys, lock_mode mode, rkind kind = rkind::wait) {
  return act(ak::req, pid, w.req_index(keys, mode), std::uint64_t(kind));
}

void settle(world &w) {
  for (;;) {
    bool did = false;
    for (const auto &h : w.homes)
      if (!h.destroyed && !h.pending.empty()) {
        w.do_(act(ak::hstep, h.hid));
        w.check();
        did = true;
        break;
      }
    if (did)
      continue;
    for (const auto &h : w.homes)
      if (!h.destroyed && !h.port.empty()) {
        w.do_(act(ak::post, h.hid));
        w.check();
        did = true;
        break;
      }
    if (!did)
      return;
  }
}

void run(world &w, std::initializer_list<action> script) {
  for (const auto &a : script) {
    w.do_(a);
    w.check();
    settle(w);
  }
}
void step(world &w, const action &a) {
  w.do_(a);
  w.check();
}

const frame &top(world &w, int pid) { return w.procs.at(pid).frames.back(); }

std::string scenario_lk27(std::uint32_t mut = 0) {
  // P holds S a.x; Q's X on a queues; P nests X a.y: granted at once (the conversion rule)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}}, mut);
  run(w, {R(w, 1, {"a.x"}, S), R(w, 2, {"a"}, X), R(w, 1, {"a.y"}, X)});
  auto &p1 = w.procs.at(1);
  if (mut & dt::lock_table::no_exemption) {
    const frame &f = p1.frames.back();
    MODEL_ASSERT(f.enter && f.outcome.k == outcome_t::deadlock, outcome_str(f.outcome));
    return "DeadlockError as expected: " + outcome_str(f.outcome);
  }
  MODEL_ASSERT(p1.status == pstatus::run && p1.locks.size() == 2, "");
  MODEL_ASSERT(!p1.frames.back().enter, "");
  run(w, {act(ak::finish, 1), act(ak::finish, 1)});
  MODEL_ASSERT(w.procs.at(2).status == pstatus::run, "");
  run(w, {act(ak::enter_step, 2)});
  MODEL_ASSERT(!top(w, 2).enter, "");
  w.drain();
  return "ok";
}

std::string scenario_lk25(std::uint32_t mut = 0) {
  // H holds S k; W queues X k; R queues S k; W times out: R granted at once
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "o", 0}}, mut);
  run(w, {R(w, 1, {"k"}, S), R(w, 2, {"k"}, X, rkind::wait_dl), R(w, 3, {"k"}, S)});
  MODEL_ASSERT(w.procs.at(3).status == pstatus::parked, "");
  run(w, {act(ak::deadline, 2)});
  MODEL_ASSERT(w.procs.at(3).status == pstatus::run, "");
  MODEL_ASSERT(top(w, 2).outcome.k == outcome_t::timeout, outcome_str(top(w, 2).outcome));
  w.drain();
  return "ok";
}

std::string scenario_lk21() {
  auto w =
      mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "o", 0}, {0, 4, "o", 0}, {0, 5, "o", 0}});
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, S), R(w, 3, {"a"}, S), R(w, 4, {"a"}, X),
          R(w, 5, {"a"}, S)});
  run(w, {act(ak::finish, 1)});
  std::vector<pstatus> st;
  for (int i = 2; i <= 5; ++i)
    st.push_back(w.procs.at(i).status);
  MODEL_ASSERT(
      (st == std::vector<pstatus>{pstatus::run, pstatus::run, pstatus::parked, pstatus::parked}),
      "");
  w.drain();
  return "ok";
}

std::string scenario_ld10() {
  // P holds S a.x; R queues X a; Q queues S a.q; P requests X a.q.w: DeadlockError (P, Q, R)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "o", 0}});
  run(w, {R(w, 1, {"a.x"}, S), R(w, 2, {"a"}, X), R(w, 3, {"a.q"}, S), R(w, 1, {"a.q.w"}, X)});
  const frame &f = top(w, 1);
  MODEL_ASSERT(f.outcome.k == outcome_t::deadlock, outcome_str(f.outcome));
  std::string r = outcome_str(f.outcome);
  w.drain();
  return "DeadlockError " + r;
}

std::string scenario_ld9() {
  // H holds X a.b; V queues S a; W holds X z and queues X a.c; H requests X z: DeadlockError
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "o", 0}});
  run(w, {R(w, 1, {"a.b"}, X), R(w, 2, {"a"}, S), R(w, 3, {"z"}, X), R(w, 3, {"a.c"}, X),
          R(w, 1, {"z"}, X)});
  const frame &f = top(w, 1);
  MODEL_ASSERT(f.outcome.k == outcome_t::deadlock, outcome_str(f.outcome));
  std::string r = outcome_str(f.outcome);
  w.drain();
  return "DeadlockError " + r;
}

std::string scenario_lx4() {
  // grant, then a signal handler that raises before the lock_enter step (LX-4)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "o", 0}});
  run(w, {R(w, 1, {"k"}, X), R(w, 2, {"k"}, X)});
  run(w, {act(ak::finish, 1)});
  run(w, {act(ak::signal, 2), R(w, 2, {"k"}, S)}); // in the handler: refused ("handler")
  MODEL_ASSERT(w.procs.at(2).frames.size() == 1, "");
  run(w, {act(ak::hraise, 2)});
  MODEL_ASSERT(w.procs.at(2).frames.empty() && w.M.spec.holds.empty(), "");
  w.drain();
  return "ok";
}

/// Two homes, posts between them (LX-10, LX-11): one_home off on both sides.
world mk2(std::vector<proc_cfg> procs, std::uint32_t wmut = 0) {
  return mk({false, false}, std::move(procs), 0, wmut);
}
std::string two_home_result(const world &w) {
  return w.cfg.real_dropped ? "ok, on the oracle alone: the table refuses a second home even with "
                              "one_home off"
                            : "ok";
}

std::string scenario_lx10() {
  // grant to a waiter on another home, then the waiter killed before the wake is applied
  auto w = mk2({{0, 1, "o", 0}, {1, 2, "o", 0}, {0, 3, "o", 0}});
  run(w, {R(w, 1, {"k"}, X), R(w, 2, {"k"}, X), R(w, 3, {"k"}, X)});
  step(w, act(ak::finish, 1));
  while (!w.homes[0].pending.empty())
    step(w, act(ak::hstep, 0));
  step(w, act(ak::kill, 2, 1));
  settle(w);
  MODEL_ASSERT(w.procs.at(3).status == pstatus::run, "");
  w.drain();
  return two_home_result(w);
}

std::string scenario_lx11(std::uint32_t wmut = 0) {
  // a granted wake posted to another home; the waiter's timeout adopts it; it then queues anew;
  // the stale wake must not resume the new request
  auto w = mk2({{0, 1, "o", 0}, {1, 2, "o", 0}, {0, 3, "o", 0}}, wmut);
  run(w, {R(w, 1, {"k"}, X), R(w, 2, {"k"}, X, rkind::wait_dl)});
  step(w, act(ak::finish, 1)); // p1 releases on H0: the wake to p2 goes to H1's port
  while (!w.homes[0].pending.empty())
    step(w, act(ak::hstep, 0));
  MODEL_ASSERT(!w.homes[1].port.empty(), "wake should be in flight");
  step(w, act(ak::deadline, 2));
  while (!w.homes[1].pending.empty())
    step(w, act(ak::hstep, 1));
  MODEL_ASSERT(top(w, 2).outcome.k == outcome_t::granted, "");
  step(w, act(ak::enter_step, 2));
  step(w, R(w, 3, {"m"}, X));
  step(w, R(w, 2, {"m"}, X)); // nested: p2 waits for m behind p3
  while (!w.homes[1].pending.empty())
    step(w, act(ak::hstep, 1));
  MODEL_ASSERT(w.procs.at(2).status == pstatus::parked, "");
  step(w, act(ak::post, 1)); // the stale wake
  settle(w);
  MODEL_ASSERT(w.procs.at(2).status == pstatus::parked, "stale wake resumed a newer request");
  w.drain();
  return two_home_result(w);
}

std::size_t refs_of(world &w, home_id h, const std::string &k) {
  auto e = w.M.spec.cache_refs.find(h);
  if (e == w.M.spec.cache_refs.end() || !e->second.count(k))
    return 0;
  return std::size_t(e->second.at(k));
}

std::string scenario_cache_detach(std::uint32_t mut = 0, std::uint32_t wmut = 0) {
  // a home's key-cache references keep its keys while it is attached, and detach_home drops them
  // (2.4, 4.11): the manager is then empty, its trie only the root (I9)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}}, mut, wmut, 2);
  run(w, {R(w, 1, {"a.b"}, X), act(ak::finish, 1), R(w, 2, {"b"}, S), act(ak::finish, 2)});
  MODEL_ASSERT(refs_of(w, 0, "a.b") == 1 && refs_of(w, 0, "b") == 1, "");
  run(w, {act(ak::destroy, 0)});
  MODEL_ASSERT(w.M.spec.cache_refs.empty() && w.M.spec.homes.empty(), "");
  w.drain();
  return "ok";
}

std::string scenario_cache_clear(std::uint32_t mut = 0) {
  // attach makes an empty entry; a host's clear (unref_nodes) empties it and the entry stays until
  // detach_home deletes it (4.4, 4.15, I9)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}}, mut, 0, 2, true);
  MODEL_ASSERT(w.M.spec.cache_refs.size() == 1 && w.M.spec.cache_refs.at(0).empty(), "");
  run(w, {R(w, 1, {"a.b"}, X), act(ak::finish, 1), R(w, 1, {"b"}, S), act(ak::finish, 1)});
  MODEL_ASSERT(refs_of(w, 0, "a.b") == 1 && refs_of(w, 0, "b") == 1, "");
  run(w, {act(ak::cclear, 0)});
  MODEL_ASSERT(w.M.spec.cache_refs.size() == 1 && w.M.spec.cache_refs.at(0).empty(), "");
  MODEL_ASSERT(w.M.spec.expected_trie_nodes() == 1, "");
  run(w, {R(w, 1, {"a"}, X), act(ak::finish, 1)}); // the entry takes references again
  MODEL_ASSERT(refs_of(w, 0, "a") == 1 && w.M.spec.cache_refs.at(0).size() == 1, "");
  run(w, {act(ak::destroy, 0)});
  MODEL_ASSERT(w.M.spec.cache_refs.empty(), "");
  w.drain();
  return "ok";
}

std::string scenario_cycle_other_owner_pid() {
  // 4.6: a cycle through processes of another owner shows their pids as nil (scenarios.py's
  // cycle_other_home_pid, on one home with the other processes of another owner)
  auto w = mk({false}, {{0, 1, "p", 0}, {0, 2, "o", 0}, {0, 3, "p", 0}});
  run(w, {R(w, 1, {"a"}, X), R(w, 3, {"c"}, X), R(w, 2, {"b"}, X), R(w, 1, {"b"}, X),
          R(w, 3, {"a"}, X), R(w, 2, {"c"}, X)});
  const frame &f = top(w, 2);
  MODEL_ASSERT(f.outcome.k == outcome_t::deadlock, outcome_str(f.outcome));
  std::vector<std::optional<int>> pids;
  for (const auto &e : f.outcome.cycle)
    pids.push_back(e.pid);
  MODEL_ASSERT(pids.size() == 3 && pids[0] == 2 && !pids[1] && !pids[2], outcome_str(f.outcome));
  std::string r = outcome_str(f.outcome);
  w.drain();
  return "DeadlockError " + r;
}

std::string scenario_query_visibility(std::uint32_t mut = 0) {
  // 4.13: lock-query counts every hold and shows pids only for the viewer's home and owner; one
  // process may hold two X holds with the same key (X a.b, then a nested X {a.b, b}), and the
  // generation shown is the latest X grant's (the world checks it at quiescence)
  auto w = mk({false}, {{0, 1, "o", 0}, {0, 2, "o", 0}, {0, 3, "p", 0}}, mut);
  run(w, {R(w, 1, {"a.b"}, X), R(w, 1, {"a.b", "b"}, X), R(w, 2, {"a"}, X), R(w, 3, {"b"}, S)});
  auto q1 = w.M.query("", 0, "o");
  auto q3 = w.M.query("", 0, "p");
  const lock_info *ab = nullptr;
  for (const auto &e : q1.held)
    if (e.key == "a.b")
      ab = &e;
  MODEL_ASSERT(ab && ab->count == 2 && ab->pids == (std::vector<int>{1, 1}) && ab->generation == 2u,
               fmt_query(q1, false));
  MODEL_ASSERT(q1.waiting.size() == 2 && q1.waiting[0].pid == 2 && !q1.waiting[1].pid,
               fmt_query(q1, false));
  MODEL_ASSERT(q3.waiting.size() == 2 && !q3.waiting[0].pid && q3.waiting[1].pid == 3,
               fmt_query(q3, false));
  for (const auto &e : q3.held)
    MODEL_ASSERT(e.pids.empty(), fmt_query(q3, false));
  w.drain();
  return "ok: two X holds of p1 with a.b (generations 1 and 2)";
}

std::string scenario_l1_one_home() {
  // protocol 7, L1: one home, prio false: no inheritance, every inh INF, no posts
  auto w = mk({false}, {{0, 1, "o", 5}, {0, 2, "o", -5}, {0, 3, "o", 0}}, 0, 0, 3);
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X), R(w, 3, {"a.b"}, S)});
  for (const auto &kv : w.M.spec.recs)
    MODEL_ASSERT(kv.second.inh == INF, "");
  MODEL_ASSERT(w.procs.at(1).inherited == INF, "");
  run(w, {act(ak::finish, 1)});
  MODEL_ASSERT(w.procs.at(2).status == pstatus::run && w.procs.at(3).status == pstatus::parked, "");
  w.drain();
  return "ok";
}

std::string scenario_set_base() {
  // I8's base clause: a holder's set-priority reaches its rec (5.8; L1: the base only)
  auto w = mk({false}, {{0, 1, "o", 5}, {0, 2, "o", 0}});
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X), act(ak::prio, 1, -2)});
  MODEL_ASSERT(w.M.spec.recs.at(lock_pid{0, 1}).base == -2, "");
  w.drain();
  return "ok";
}

std::string scenario_one_home_refusal() {
  // 4.4: before L3 a manager accepts one home; attaching a second is refused (by the oracle with
  // a 'require', by the table with a std::logic_error)
  lockstep::options o;
  o.real = use_real();
  lockstep M(o);
  spec_notes N;
  MODEL_ASSERT(M.attach_home(false, N) == 0, "");
  MODEL_ASSERT(M.try_acquire(lock_pid{0, 1}, "o", 0, {"a"}, X, {}, {}).has_value(), "");
  try {
    M.attach_home(false, N);
  } catch (const lock_violation &v) {
    MODEL_ASSERT(v.invariant() == "require", v.what());
    return std::string("refused: ") + v.what();
  }
  throw scenario_failure("a second home was attached to a one-home manager");
}

std::string scenario_ref_nodes() {
  // 4.15: ref_nodes requires an attached home; unref_nodes requires a reference. Each refusal is
  // the oracle's 'require', and the lockstep requires the table to refuse the same call (its
  // 4.15 precondition)
  lockstep::options o;
  o.real = use_real();
  lockstep M(o);
  spec_notes N;
  M.attach_home(false, N);
  M.ref_nodes(0, {"a.b"});
  MODEL_ASSERT(M.spec.cache_refs.at(0).at("a.b") == 1 && M.spec.expected_trie_nodes() == 3, "");
  M.unref_nodes(0, {"a.b"});
  MODEL_ASSERT(M.spec.cache_refs.at(0).empty() && M.spec.expected_trie_nodes() == 1, "");
  for (int i = 0; i < 2; ++i) {
    try {
      if (i == 0)
        M.ref_nodes(2, {"a"});
      else
        M.unref_nodes(0, {"a"});
    } catch (const lock_violation &v) {
      MODEL_ASSERT(v.invariant() == "require", v.what());
      continue;
    }
    throw scenario_failure(i == 0 ? "ref_nodes of a home that is not attached was accepted"
                                  : "unref_nodes of a key with no reference was accepted");
  }
  M.check_now();
  MODEL_ASSERT(M.spec.cache_refs.at(0).empty() && M.spec.expected_trie_nodes() == 1, "");
  return "ok";
}

// -- phase L2: scenarios.py's inheritance scenarios, their homes merged into one priority_rr home
// (protocol 7: on one home every relevel is applied right after its operation, 4.14) -----------

int inh_of(world &w, int pid) { return w.procs.at(pid).inherited; }

std::string scenario_l2_one_home() {
  // protocol 7, L2: one home under priority_rr: a waiter boosts the holder, applied at once on the
  // same home (4.14), and a set-priority re-levels it
  auto w = mk({true}, {{0, 1, "o", 5}, {0, 2, "o", -5}}, 0, 0, 1);
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X)});
  MODEL_ASSERT(inh_of(w, 1) == -5 && w.homes[0].port.empty(), inh_str(inh_of(w, 1)));
  run(w, {act(ak::prio, 2, 2)});
  MODEL_ASSERT(inh_of(w, 1) == 2, inh_str(inh_of(w, 1)));
  w.drain();
  return "ok";
}

std::string scenario_pi9() {
  // PI-9: H (10) holds X a.b; V (10) queues X a; W (-10) queues S a.c: H and V boosted to -10, H
  // through W's waiter edge to V; V granted within H's section, W after V's
  auto w = mk({true}, {{0, 1, "o", 10}, {0, 2, "o", 10}, {0, 3, "o", -10}});
  run(w, {R(w, 1, {"a.b"}, X), R(w, 2, {"a"}, X), R(w, 3, {"a.c"}, S)});
  MODEL_ASSERT(inh_of(w, 1) == -10 && inh_of(w, 2) == -10,
               inh_str(inh_of(w, 1)) + " " + inh_str(inh_of(w, 2)));
  run(w, {act(ak::finish, 1)});
  MODEL_ASSERT(w.procs.at(2).status == pstatus::run && w.procs.at(3).status == pstatus::parked, "");
  w.drain();
  return "ok";
}

std::string scenario_pi11(std::uint32_t mut = 0) {
  // PI-11: P (5) holds X a.b, R (5) holds S a.c; P queues X a; W (-10) queues X a, then times
  // out: P and R boosted to -10, then both back to 5
  auto w = mk({true}, {{0, 1, "o", 5}, {0, 2, "o", 5}, {0, 3, "o", -10}}, mut);
  run(w, {R(w, 1, {"a.b"}, X), R(w, 2, {"a.c"}, S), R(w, 1, {"a"}, X),
          R(w, 3, {"a"}, X, rkind::wait_dl)});
  MODEL_ASSERT(inh_of(w, 1) == -10 && inh_of(w, 2) == -10,
               inh_str(inh_of(w, 1)) + " " + inh_str(inh_of(w, 2)));
  run(w, {act(ak::deadline, 3)});
  const auto eff = [&](int pid) { return std::min(w.procs.at(pid).priority, inh_of(w, pid)); };
  MODEL_ASSERT(eff(1) == 5 && eff(2) == 5, inh_str(inh_of(w, 1)) + " " + inh_str(inh_of(w, 2)));
  w.drain();
  return "ok";
}

std::string scenario_chain_cap8() {
  // p_i holds X k_i and nests X k_(i+1); p10 holds k10. p1 (base -5) reaches p2..p9 (1..8 edges),
  // not p10 (9 edges), which inherits 3 from p2..p9; p1's kill withdraws its share down the chain
  const int n = 10;
  std::vector<proc_cfg> procs;
  for (int i = 1; i <= n; ++i)
    procs.push_back({0, i, "o", i == 1 ? -5 : 3});
  auto w = mk({true}, procs);
  for (int i = n; i >= 1; --i)
    run(w, {R(w, i, {"k" + std::to_string(i)}, X)});
  for (int i = n - 1; i >= 1; --i)
    run(w, {R(w, i, {"k" + std::to_string(i + 1)}, X)});
  auto inh = [&] {
    std::string s;
    for (int i = 1; i <= n; ++i)
      s += (i > 1 ? " " : "") + inh_str(inh_of(w, i));
    return s;
  };
  for (int i = 2; i <= 9; ++i)
    MODEL_ASSERT(inh_of(w, i) == -5, inh());
  MODEL_ASSERT(inh_of(w, 1) == INF && inh_of(w, 10) == 3, inh());
  run(w, {act(ak::kill, 1, 1)});
  MODEL_ASSERT(inh_of(w, 2) == INF, inh());
  for (int i = 3; i <= n; ++i)
    MODEL_ASSERT(inh_of(w, i) == 3, inh());
  std::string r = inh();
  w.drain();
  return "ok " + r;
}

std::string scenario_coalesce(std::uint32_t mut = 0) {
  // P1 (5) holds X a; P2 (-1) queues X a: P1 boosted by a relevel; P2 times out: P1's boost must
  // go, through a second relevel that coalescing must not swallow
  auto w = mk({true}, {{0, 1, "o", 5}, {0, 2, "o", -1}}, mut, 0, 0, false, true);
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X, rkind::wait_dl)});
  MODEL_ASSERT(inh_of(w, 1) == -1, inh_str(inh_of(w, 1)));
  run(w, {act(ak::deadline, 2)});
  MODEL_ASSERT(inh_of(w, 1) == INF, inh_str(inh_of(w, 1)));
  w.drain();
  return "ok";
}

std::string scenario_impl_note(std::uint32_t mut = 0) {
  // Q holds X a; P (10) queues X a; W (-5) queues X a, then times out: P's inh changes, and P is
  // downstream of the seed W only in the graph before the cancel
  auto w = mk({true}, {{0, 1, "o", 0}, {0, 2, "o", 10}, {0, 3, "o", -5}}, mut);
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X), R(w, 3, {"a"}, X, rkind::wait_dl)});
  MODEL_ASSERT(inh_of(w, 2) == -5, inh_str(inh_of(w, 2)));
  run(w, {act(ak::deadline, 3)});
  MODEL_ASSERT(inh_of(w, 2) == INF, inh_str(inh_of(w, 2)));
  w.drain();
  return "ok";
}

std::string scenario_set_base_l2(std::uint32_t wmut = 0) {
  // I8's base clause on a priority_rr home: a holder's set-priority reaches its rec (5.8), and the
  // waiter's boost of it stays
  auto w = mk({true}, {{0, 1, "o", 5}, {0, 2, "o", 0}}, 0, wmut);
  run(w, {R(w, 1, {"a"}, X), R(w, 2, {"a"}, X), act(ak::prio, 1, -2)});
  MODEL_ASSERT(w.M.spec.recs.at(lock_pid{0, 1}).base == -2, "");
  MODEL_ASSERT(inh_of(w, 1) == 0, inh_str(inh_of(w, 1)));
  w.drain();
  return "ok";
}

// =============================================================================================
// 8. Random runs (random_runs.py on one home: of prio false, phase L1, or prio true, phase L2)
// =============================================================================================

const std::vector<std::string> kRandomKeys = {"a", "a.b", "a.c", "a.b.d", "b", "b.e", "c"};

std::vector<std::pair<key_list, lock_mode>> all_requests() {
  std::set<std::pair<key_list, int>> seen;
  std::vector<std::pair<key_list, lock_mode>> out;
  const auto &K = kRandomKeys;
  std::vector<key_list> combos;
  for (std::size_t i = 0; i < K.size(); ++i) {
    combos.push_back({K[i]});
    for (std::size_t j = i + 1; j < K.size(); ++j) {
      combos.push_back({K[i], K[j]});
      for (std::size_t k = j + 1; k < K.size(); ++k)
        combos.push_back({K[i], K[j], K[k]});
    }
  }
  for (const auto &c : combos) {
    auto ck = canonical_keys(c);
    if (ck.size() != c.size())
      continue;
    for (auto m : {S, X})
      if (seen.insert({ck, int(m)}).second)
        out.push_back({ck, m});
  }
  return out;
}

double weight(ak k) {
  switch (k) {
  case ak::hstep:
  case ak::post:
  case ak::enter_step:
    return 6;
  case ak::req:
    return 4;
  case ak::finish:
    return 3;
  case ak::raise:
  case ak::hraise:
    return 0.4;
  case ak::signal:
    return 0.25;
  case ak::hend:
    return 2;
  case ak::prio:
    return 0.3;
  case ak::kill:
    return 0.06;
  case ak::deadline:
    return 0.6;
  case ak::destroy:
    return 0.01;
  case ak::frelease:
  case ak::fcancel:
    return 0.15;
  case ak::cclear:
    return 0.05;
  case ak::cachecap:
    return 1;
  }
  return 1;
}

config random_config(std::mt19937_64 &rnd, std::uint64_t seed, bool prio) {
  auto randint = [&](int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rnd); };
  auto random01 = [&] { return std::uniform_real_distribution<double>(0, 1)(rnd); };
  int variant = int(seed % 3);
  config c;
  c.name = "random seed " + std::to_string(seed);
  c.homes = {prio};
  int np = variant < 2 ? randint(3, 8) : randint(10, 14);
  for (int i = 0; i < np; ++i)
    c.procs.push_back({0, i + 1, randint(0, 1) ? "o2" : "o1", randint(-2, 2)});
  c.inheritance = random01() >= 0.15;
  c.cache = {randint(0, 3)};
  c.requests = all_requests();
  c.kinds = {rkind::wait, rkind::wait_dl, rkind::wait_dl, rkind::ifavail, rkind::t0};
  c.max_depth = 3;
  c.max_hdepth = 2;
  c.signals = 1000000000;
  c.kills = {true, false};
  c.destroy = true;
  if (prio)
    c.prio_values = {-2, -1, 0, 1, 2}; // set-priority only under a priority policy (L1 has none)
  c.prio_changes = 1000000000;
  c.req_per_proc = 1000000000;
  c.coalesce = (seed / 3) % 2 == 1;
  c.foreign = true;
  c.cache_clear = true;
  c.one_home = true;
  c.real = use_real();
  return c;
}

action pick(std::mt19937_64 &rnd, const std::vector<action> &acts, std::size_t nreq) {
  std::vector<double> ws;
  for (const auto &a : acts) {
    double w = weight(a.k);
    if (a.k == ak::req)
      w /= double(nreq) * 5.0;
    else if (a.k == ak::prio)
      w /= 4.0;
    else if (a.k == ak::kill)
      w /= 2.0;
    else if (a.k == ak::frelease || a.k == ak::fcancel)
      w /= 6.0;
    ws.push_back(w);
  }
  std::discrete_distribution<std::size_t> d(ws.begin(), ws.end());
  return acts[d(rnd)];
}

struct random_failure {
  std::uint64_t seed = 0;
  std::string name, detail;
  std::vector<std::string> trace;
};

/// Episodes of up to 400 random steps from seeds seed0, seed0 + 1000003, ..., each checked after
/// every step and ended by the fair drain, until `total` steps ran. The first failure stops it.
/// `prio`: the one home applies priorities (phase L2) or not (phase L1).
std::optional<random_failure> random_runs(std::uint64_t seed0, long total, bool prio,
                                          long *steps_out, long *episodes_out) {
  long steps = 0, eps = 0;
  std::uint64_t seed = seed0;
  while (steps < total) {
    std::mt19937_64 rnd(seed);
    config cfg = random_config(rnd, seed, prio);
    std::vector<action> trace;
    bool draining = false;
    try {
      world w(cfg);
      w.check();
      for (int n = 0; n < 400 && steps < total; ++n) {
        auto acts = w.enabled();
        if (acts.empty())
          break;
        auto a = pick(rnd, acts, cfg.requests.size());
        trace.push_back(a);
        w.do_(a);
        w.check();
        ++steps;
      }
      draining = true;
      w.drain();
    } catch (const lock_violation &v) {
      random_failure f;
      f.seed = seed;
      f.name = v.invariant();
      f.detail = v.what();
      found_t ft;
      ft.detail = v.what();
      ft.trace = trace;
      ft.drain = draining;
      f.trace = render(cfg, ft);
      *steps_out = steps;
      *episodes_out = eps;
      return f;
    }
    ++eps;
    seed += 1000003;
  }
  *steps_out = steps;
  *episodes_out = eps;
  return std::nullopt;
}

// =============================================================================================
// 9. The Python model's traces (tools/locks_model/export_traces.py)
// =============================================================================================

std::filesystem::path traces_dir() {
  if (const char *e = std::getenv("CVC_LOCKS_MODEL_TRACES"))
    return e;
  return std::filesystem::path(__FILE__).parent_path() / "data" / "locks_model_traces";
}

std::uint32_t python_mutation(const std::string &name) {
  using m = dt::lock_table::mutation;
  if (name == "no-exempt")
    return m::no_exemption;
  if (name == "region-holds-only")
    return m::cancel_reexamines_holds_only;
  if (name == "no-unref-on-detach")
    return m::keep_detached_refs;
  if (name == "unref-pops-entry")
    return m::unref_pops_entry;
  if (name == "cancel-marks-failed")
    return m::cancel_marks_failed;
  if (name == "query-first-gen")
    return m::query_first_gen;
  if (name == "no-relevel-cancel")
    return m::cancel_skips_relevel;
  if (name == "coalesce-no-clear")
    return m::coalesce_no_clear;
  if (name == "impl-note-after-only")
    return m::impl_note_after_only;
  fail("trace", "unknown mutation " + name);
}

/// Replays one trace file; returns the number of operations replayed.
std::size_t replay_trace(const std::filesystem::path &file, bool real) {
  std::ifstream in(file, std::ios::binary);
  if (!in)
    fail("trace", "cannot open " + file.string());
  std::optional<lockstep> M;
  std::string line;
  int lineno = 0;
  std::size_t ops = 0;
  while (std::getline(in, line)) {
    ++lineno;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty() || line[0] == '#')
      continue;
    const std::string where = file.filename().string() + ":" + std::to_string(lineno) + ": ";
    auto tok = split(line, ' ');
    if (tok[0] == "config") {
      lockstep::options o;
      o.real = real;
      for (std::size_t i = 1; i < tok.size(); ++i) {
        auto kv = split(tok[i], '=');
        if (kv.size() != 2)
          fail("trace", where + "bad config token " + tok[i]);
        if (kv[0] == "inheritance")
          o.inheritance = kv[1] == "1";
        else if (kv[0] == "coalesce")
          o.coalesce = kv[1] == "1";
        else if (kv[0] == "one_home")
          o.one_home = kv[1] == "1";
        else if (kv[0] == "mutations" && kv[1] != "-")
          for (const auto &n : split(kv[1], ','))
            o.mutations |= python_mutation(n);
      }
      M.emplace(o);
      continue;
    }
    if (!M)
      fail("trace", where + "an operation before the config line");
    auto arrow = std::find(tok.begin(), tok.end(), "->");
    auto bar = std::find(arrow, tok.end(), "|");
    if (arrow == tok.end() || bar == tok.end())
      fail("trace", where + "no '->' or '|'");
    std::vector<std::string> args(tok.begin() + 1, arrow);
    std::string want_result, want_notes;
    for (auto it = arrow + 1; it != bar; ++it)
      want_result += (want_result.empty() ? "" : " ") + *it;
    for (auto it = bar + 1; it != tok.end(); ++it)
      want_notes += (want_notes.empty() ? "" : " ") + *it;
    const std::string &op = tok[0];
    auto need = [&](std::size_t n) {
      if (args.size() != n)
        fail("trace", where + op + " takes " + std::to_string(n) + " arguments");
    };
    auto I = [&](std::size_t i) { return std::stoll(args.at(i)); };
    auto Pid = [&](std::size_t i) { return lock_pid{home_id(I(i)), std::int32_t(I(i + 1))}; };
    auto Mode = [&](std::size_t i) { return args.at(i) == "S" ? S : X; };
    std::string got_result, got_notes;
    spec_notes N;
    try {
      if (op == "attach_home") {
        need(1);
        got_result = std::to_string(M->attach_home(I(0) != 0, N));
      } else if (op == "try_acquire" || op == "block") {
        need(8);
        auto keys = parse_keys_tok(args[5]), add = parse_keys_tok(args[6]),
             drop = parse_keys_tok(args[7]);
        if (op == "try_acquire")
          got_result =
              fmt_grant(M->try_acquire(Pid(0), args[2], int(I(3)), keys, Mode(4), add, drop));
        else
          got_result = fmt_block(M->block(Pid(0), args[2], int(I(3)), keys, Mode(4), add, drop, N));
      } else if (op == "resume") {
        need(3);
        got_result = fmt_outcome(to_outcome(M->resume(Pid(0), std::uint64_t(I(2)))));
      } else if (op == "start_body") {
        need(3);
        M->start_body(Pid(0), std::uint64_t(I(2)));
        got_result = "ok";
      } else if (op == "cancel") {
        need(3);
        got_result = fmt_cancel(M->cancel(Pid(0), std::uint64_t(I(2)), N));
      } else if (op == "release") {
        need(3);
        M->release(Pid(0), std::uint64_t(I(2)), N);
        got_result = "ok";
      } else if (op == "end") {
        need(3);
        M->end(Pid(0), I(2) != 0, N);
        got_result = "ok";
      } else if (op == "detach_home") {
        need(1);
        M->detach_home(home_id(I(0)), N);
        got_result = "ok";
      } else if (op == "set_base") {
        need(3);
        M->set_base(Pid(0), int(I(2)), N);
        got_result = "ok";
      } else if (op == "read_inh") {
        need(2);
        got_result = inh_str(M->read_inh(Pid(0)));
      } else if (op == "query") {
        need(3);
        got_result =
            fmt_query(M->query(args[0] == "-" ? "" : args[0], home_id(I(1)), args[2]), false);
      } else if (op == "ref_nodes" || op == "unref_nodes") {
        need(2);
        if (op == "ref_nodes")
          M->ref_nodes(home_id(I(0)), parse_keys_tok(args[1]));
        else
          M->unref_nodes(home_id(I(0)), parse_keys_tok(args[1]));
        got_result = "ok";
      } else {
        fail("trace", where + "unknown operation " + op);
      }
    } catch (const lock_violation &v) {
      if (v.invariant() != "require")
        throw lock_violation(v.invariant(), where + v.what());
      got_result = "refused";
      N.clear();
    }
    got_notes = fmt_notes(N);
    if (got_result != want_result)
      fail("trace",
           where + op + ": result '" + got_result + "', the Python model's '" + want_result + "'");
    if (got_notes != want_notes)
      fail("trace", where + op + ": notifications '" + got_notes + "', the Python model's '" +
                        want_notes + "'");
    ++ops;
  }
  return ops;
}

} // namespace

// =============================================================================================
// 10. The tests
// =============================================================================================

namespace {

/// Runs a scenario; any violation or assertion is a failure with its message.
void expect_ok(const std::function<std::string()> &fn) {
  try {
    std::string r = fn();
    std::printf("  %s\n", r.c_str());
  } catch (const not_built &e) {
    GTEST_SKIP() << kStubSkip << " (" << e.what() << ")";
  } catch (const lock_violation &v) {
    ADD_FAILURE() << "violation " << v.invariant() << ": " << v.what();
  } catch (const scenario_failure &e) {
    ADD_FAILURE() << "scenario assertion: " << e.what();
  } catch (const std::exception &e) {
    ADD_FAILURE() << "crash: " << e.what();
  }
}

/// A mutation run: the violation it names and nothing else (scenarios.py 450-451: an assertion,
/// a different violation or none at all is unexpected).
void expect_violation(const std::string &want, const std::function<std::string()> &fn) {
  try {
    std::string r = fn();
    ADD_FAILURE() << "no violation (wanted " << want << "): " << r;
  } catch (const not_built &e) {
    GTEST_SKIP() << kStubSkip << " (" << e.what() << ")";
  } catch (const lock_violation &v) {
    EXPECT_EQ(v.invariant(), want) << v.what();
    std::printf("  caught as expected (%s): %s\n", want.c_str(), v.what());
  } catch (const scenario_failure &e) {
    ADD_FAILURE() << "scenario assertion (wanted " << want << "): " << e.what();
  } catch (const std::exception &e) {
    ADD_FAILURE() << "crash (wanted " << want << "): " << e.what();
  }
}

void print_result(const char *what, const config &cfg, int bound, const explore_result &r) {
  std::printf("  %s: %s at bound %d: %s, peak %zu MB%s\n", what, cfg.name.c_str(), bound,
              r.summary().c_str(), peak_rss_mb(),
              cfg.real ? " (with the real table)" : " (oracle alone)");
  for (const auto &[name, f] : r.soft)
    std::printf("  soft finding %s: %s\n", name.c_str(), f.detail.c_str());
}

void run_bounded(config cfg, int bound) {
  explore_options eo;
  eo.bound = bound;
  explore_result r;
  try {
    r = explore(cfg, eo);
  } catch (const not_built &e) {
    GTEST_SKIP() << kStubSkip << " (" << e.what() << ")";
  }
  print_result("explored", cfg, bound, r);
  EXPECT_TRUE(r.complete);
  EXPECT_TRUE(r.violations.empty()) << report(cfg, r.violations);
}

void run_random(long total, bool prio) {
  long steps = 0, eps = 0;
  auto t0 = std::chrono::steady_clock::now();
  std::optional<random_failure> f;
  try {
    f = random_runs(prio ? 11 : 7, total, prio, &steps, &eps);
  } catch (const not_built &e) {
    GTEST_SKIP() << kStubSkip << " (" << e.what() << ")";
  }
  double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("  random runs: %ld steps in %ld episodes, %.1f s%s\n", steps, eps, secs,
              use_real() ? " (with the real table)" : " (oracle alone)");
  if (f) {
    std::string t;
    for (const auto &l : f->trace)
      t += "  " + l + "\n";
    ADD_FAILURE() << "violation " << f->name << " in the episode of seed " << f->seed << ":\n" << t;
  }
}

} // namespace

// -- bounded exploration
// ----------------------------------------------------------------------------

TEST(LocksModel, L1Bounded) {
  LT5_NEEDS_TABLE();
  run_bounded(l1x_config(), regular_bound());
}

TEST(LocksModelStress, L1Bound4) {
  LT5_NEEDS_TABLE();
  run_bounded(l1x_config(), stress_bound());
}

TEST(LocksModel, L2Bounded) {
  LT5_NEEDS_TABLE();
  run_bounded(l2_config(), regular_bound());
}

TEST(LocksModel, NoInheritance) {
  LT5_NEEDS_TABLE();
  run_bounded(noinh1_config(), regular_bound());
}

TEST(LocksModelStress, L2Bound4) {
  LT5_NEEDS_TABLE();
  run_bounded(l2_config(), stress_bound());
}

namespace {

/// explore.py's counts (revision 5b of the model) of distinct states and of transitions. 'l1x' is
/// explore.py's 'l1' with signals=1 and foreign=True (tools/locks_model/README.md, "The C++ model
/// check"); 'core' is explore.py's own, two homes with posts between them and inheritance; 'l2' is
/// explore.py's (the same with coalesce=True, as here); 'noinh1' is explore.py's 'l2' with
/// inheritance=False. explore.py deduplicates states on hash(World.key()), and CPython's
/// hash(-1) == hash(-2), so two states that differ only in a priority of -1 against -2 collide
/// and one of them is never explored: the counts of 'l2' and 'noinh1', whose set-priority values
/// include both, are explore.py's run with states deduplicated on World.key() itself
/// (tools/locks_model/README.md, "The C++ model check"). The others are the same either way.
struct python_count {
  const char *config;
  int bound;
  std::size_t states, transitions;
};
constexpr python_count kPythonCounts[] = {
    {"l1", 1, 73, 132},      {"l1", 2, 1633, 2412},     {"l1", 3, 26834, 43572},
    {"l1x", 1, 85, 168},     {"l1x", 2, 1921, 3552},    {"l1x", 3, 34266, 68514},
    {"core", 1, 25, 24},     {"core", 2, 776, 871},     {"core", 3, 15746, 23201},
    {"l2", 1, 121, 120},     {"l2", 2, 2587, 3624},     {"l2", 3, 42075, 63798},
    {"noinh1", 1, 121, 120}, {"noinh1", 2, 2323, 3360}, {"noinh1", 3, 33297, 55020},
};

/// A configuration by name: l1, l1x, l2, noinh1 or core.
config config_named(const std::string &name) {
  if (name == "core")
    return core_config();
  if (name == "l1x")
    return l1x_config();
  if (name == "l2")
    return l2_config();
  if (name == "noinh1")
    return noinh1_config();
  if (name == "l1")
    return l1_config();
  fail("model", "no configuration named " + name);
}

/// The world and the oracle against the Python model: on the oracle alone and with explore.py's
/// FIFO cache (no pinning), each configuration must have exactly the states and transitions
/// explore.py counts, whether each state is copied or rebuilt by replaying its path (the way the
/// explorer runs over the real table).
void expect_python_counts(int min_bound, int max_bound, bool replay_too) {
  for (const auto &pc : kPythonCounts) {
    if (pc.bound < min_bound || pc.bound > max_bound)
      continue;
    config cfg = config_named(pc.config);
    cfg.pin = false;
    cfg.real = false;
    for (bool replay : {false, true}) {
      if (replay && !replay_too)
        continue;
      explore_options eo;
      eo.bound = pc.bound;
      eo.force_replay = replay;
      auto r = explore(cfg, eo);
      print_result(replay ? "explore.py's cache, replayed" : "explore.py's cache", cfg, pc.bound,
                   r);
      EXPECT_EQ(r.states, pc.states) << "explore.py " << pc.config << " " << pc.bound;
      EXPECT_EQ(r.transitions, pc.transitions) << "explore.py " << pc.config << " " << pc.bound;
      EXPECT_TRUE(r.violations.empty()) << report(cfg, r.violations);
    }
  }
}

} // namespace

TEST(LocksModel, PortMatchesPythonStateCounts) { expect_python_counts(1, 2, true); }

TEST(LocksModelStress, PortMatchesPythonStateCountsBound3) { expect_python_counts(3, 3, false); }

/// The explorer's cost, for fixing the sizes (locks plan L1.3: states, seconds and peak memory on
/// l1 at bounds 3, 4 and 5; L2.3 the same on l2). Disabled: run it with
/// --gtest_also_run_disabled_tests, one configuration per process so the peak is that run's, e.g.
/// CVC_LOCKS_MODEL_MEASURE=l1x:4. The form is CONFIG:BOUND[:replay]: CONFIG is l1, l1x, l2 or
/// noinh1, the same with 'py' appended for explore.py's cache (their states are explore.py's for
/// 'l1', 'l1' with signals=1 and foreign=True, 'l2', and 'l2' with inheritance=False), or core
/// (explore.py's, on the oracle alone); ':replay' rebuilds every state by replay even on the
/// oracle alone, which is what the explorer does over the real table. CVC_LOCKS_MODEL_REAL=0
/// measures the oracle alone.
TEST(LocksModelStress, DISABLED_Measure) {
  LT5_NEEDS_TABLE();
  const char *e = std::getenv("CVC_LOCKS_MODEL_MEASURE");
  for (const auto &item : split(e && *e ? e : "l1:3,l1x:3", ',')) {
    auto f = split(item, ':');
    ASSERT_GE(f.size(), 2u) << item;
    const bool py = f[0].size() > 2 && f[0].compare(f[0].size() - 2, 2, "py") == 0;
    config cfg = config_named(py ? f[0].substr(0, f[0].size() - 2) : f[0]);
    if (py) {
      cfg.pin = false;
      cfg.name += " (explore.py's cache)";
    }
    explore_options eo;
    eo.bound = std::atoi(f[1].c_str());
    eo.force_replay = f.size() > 2 && f[2] == "replay";
    auto r = explore(cfg, eo);
    print_result(eo.force_replay ? "measured, replayed" : "measured", cfg, eo.bound, r);
    EXPECT_TRUE(r.violations.empty()) << report(cfg, r.violations);
  }
}

// -- random runs
// ------------------------------------------------------------------------------------

TEST(LocksModel, RandomL1) {
  LT5_NEEDS_TABLE();
  run_random(regular_steps(), false);
}

TEST(LocksModelStress, RandomL1) {
  LT5_NEEDS_TABLE();
  run_random(stress_steps(), false);
}

TEST(LocksModel, RandomL2) {
  LT5_NEEDS_TABLE();
  run_random(regular_steps(), true);
}

TEST(LocksModelStress, RandomL2) {
  LT5_NEEDS_TABLE();
  run_random(stress_steps(), true);
}

// -- the Python model's traces
// ---------------------------------------------------------------------------

TEST(LocksModel, PythonTraces) {
  auto dir = traces_dir();
  ASSERT_TRUE(std::filesystem::is_directory(dir))
      << dir.string() << " is missing (tools/locks_model/export_traces.py writes it)";
  std::vector<std::filesystem::path> files;
  for (const auto &e : std::filesystem::directory_iterator(dir))
    if (e.path().extension() == ".txt")
      files.push_back(e.path());
  std::sort(files.begin(), files.end());
  ASSERT_FALSE(files.empty());
  const bool real = use_real();
  if (real_flag() && !table_built())
    std::printf("  %s: replaying into the oracle alone\n", kStubSkip);
  std::size_t ops = 0;
  for (const auto &f : files) {
    try {
      ops += replay_trace(f, real);
    } catch (const lock_violation &v) {
      ADD_FAILURE() << v.what();
    }
  }
  std::printf("  %zu files, %zu operations replayed%s\n", files.size(), ops,
              real ? " into the oracle and the table" : " into the oracle");
}

// -- the lockstep check is checked too
// ---------------------------------------------------------------------

TEST(LocksModel, LockstepCatchesPlantedDivergence) {
  if (!real_flag())
    GTEST_SKIP() << "needs the real table (CVC_LOCKS_MODEL_REAL=0)";
  LT5_NEEDS_TABLE();
  config cfg = l1_config();
  cfg.real = true;
  cfg.real_mutations = dt::lock_table::plant_divergence;
  explore_options eo;
  eo.bound = 3;
  eo.drain = false;
  // the whole of l1 at bound 3 in an optimized build; elsewhere up to the first violation, which
  // the BFS finds among the first transitions of 3 events (a release that grants)
  eo.stop_on_violation = !kRelease || sanitizer_lane();
  auto r = explore(cfg, eo);
  print_result("planted divergence", cfg, 3, r);
  ASSERT_EQ(r.violations.size(), 1u) << report(cfg, r.violations);
  ASSERT_TRUE(r.violations.count("lockstep")) << report(cfg, r.violations);
  const auto &d = r.violations.at("lockstep").detail;
  EXPECT_NE(d.find("release"), std::string::npos) << d;
  EXPECT_NE(d.find("counters grants="), std::string::npos) << d;
  std::printf("%s", report(cfg, r.violations).c_str());
}

// -- the directed scenarios
// -----------------------------------------------------------------------------------

TEST(LocksModel, LK27ConversionRule) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lk27(); });
}
TEST(LocksModel, LK25CancelLetsReaderIn) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lk25(); });
}
TEST(LocksModel, LK21Fifo) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lk21(); });
}
TEST(LocksModel, LD10ChainDeadlock) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_ld10(); });
}
TEST(LocksModel, LD9Deadlock) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_ld9(); });
}
TEST(LocksModel, LX4HandlerRaisesBeforeBody) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lx4(); });
}
TEST(LocksModel, LX10KilledBeforeWake) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lx10(); });
}
TEST(LocksModel, LX11StaleWake) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_lx11(); });
}
TEST(LocksModel, CacheDetach) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_cache_detach(); });
}
TEST(LocksModel, CacheClear) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_cache_clear(); });
}
TEST(LocksModel, CycleOtherOwnerPid) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_cycle_other_owner_pid(); });
}
TEST(LocksModel, QueryVisibility) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_query_visibility(); });
}
TEST(LocksModel, L1OneHome) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_l1_one_home(); });
}
TEST(LocksModel, SetBase) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_set_base(); });
}
TEST(LocksModel, OneHomeRefusal) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_one_home_refusal(); });
}
TEST(LocksModel, RefNodes) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_ref_nodes(); });
}

// -- the inheritance scenarios (phase L2, one priority_rr home)
// ------------------------------------------

TEST(LocksModel, L2OneHome) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_l2_one_home(); });
}
TEST(LocksModel, PI09BoostThroughWaiterEdge) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_pi9(); });
}
TEST(LocksModel, PI11MultiHolderBoostAndWithdraw) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_pi11(); });
}
TEST(LocksModel, PIChainCappedAt8) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_chain_cap8(); });
}
TEST(LocksModel, PICoalesce) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_coalesce(); });
}
TEST(LocksModel, PIImplNote) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_impl_note(); });
}
TEST(LocksModel, SetBaseL2) {
  LT5_NEEDS_TABLE();
  expect_ok([] { return scenario_set_base_l2(); });
}

// -- the mutations (locks plan L1 section 3, LT-5 item 5)
// ------------------------------------------------------

TEST(LocksModel, MutationNoExemption) {
  LT5_NEEDS_TABLE();
  // LK-27's nested request raises DeadlockError: the scenario's own outcome, no violation
  expect_ok([] {
    auto r = scenario_lk27(dt::lock_table::no_exemption);
    MODEL_ASSERT(r.rfind("DeadlockError as expected", 0) == 0, r);
    return r;
  });
}
TEST(LocksModel, MutationCancelReexaminesHoldsOnly) {
  LT5_NEEDS_TABLE();
  expect_violation("I6",
                   [] { return scenario_lk25(dt::lock_table::cancel_reexamines_holds_only); });
}
TEST(LocksModel, MutationKeepDetachedRefs) {
  LT5_NEEDS_TABLE();
  expect_violation("I9", [] { return scenario_cache_detach(dt::lock_table::keep_detached_refs); });
}
TEST(LocksModel, MutationUnrefPopsEntry) {
  LT5_NEEDS_TABLE();
  expect_violation("I9", [] { return scenario_cache_clear(dt::lock_table::unref_pops_entry); });
}
TEST(LocksModel, MutationCancelMarksFailed) {
  LT5_NEEDS_TABLE();
  expect_violation("I2", [] { return scenario_lk25(dt::lock_table::cancel_marks_failed); });
}
TEST(LocksModel, MutationQueryFirstGen) {
  LT5_NEEDS_TABLE();
  expect_violation("4.13",
                   [] { return scenario_query_visibility(dt::lock_table::query_first_gen); });
}
TEST(LocksModel, MutationWakeCarriesOutcome) {
  LT5_NEEDS_TABLE();
  expect_violation("I2", [] { return scenario_lx11(wake_carries_outcome); });
}
TEST(LocksModel, MutationCacheMissUnreferenced) {
  LT5_NEEDS_TABLE();
  expect_violation("I4", [] { return scenario_cache_detach(0, cache_miss_unreferenced); });
}

// The L2 mutations (locks plan L2.3; protocol 7): each on the table and the oracle together, or
// in the world, and each must report its own violation.

TEST(LocksModel, Mutation_CancelSkipsRelevel) {
  LT5_NEEDS_TABLE();
  // PI-11's timeout keeps the boost of P and R (I8)
  expect_violation("I8", [] { return scenario_pi11(dt::lock_table::cancel_skips_relevel); });
}
TEST(LocksModel, Mutation_CoalesceNoClear) {
  LT5_NEEDS_TABLE();
  // the flag is never cleared, so the relevel that would withdraw P1's boost is coalesced away and
  // the boost is kept for ever (I8's quiescent clause)
  expect_violation("I8", [] { return scenario_coalesce(dt::lock_table::coalesce_no_clear); });
}
TEST(LocksModel, Mutation_ImplNoteAfterOnly) {
  LT5_NEEDS_TABLE();
  // the set A without the graph before misses P and Q, whose inh the timeout changes
  expect_violation("impl-note",
                   [] { return scenario_impl_note(dt::lock_table::impl_note_after_only); });
}
// Each manager mutation of L2 on one side alone, each side required to report it (R2a: in
// lockstep one side's violation stops the step before the other runs): `want_table` from the
// table beside an unmutated oracle, tagged [table], or, for a mutation the table's own check()
// cannot see, as the two sides' notifications differing ("lockstep"); `want_oracle` from the
// oracle alone, where no table runs to report anything.
void expect_violation_on_each_side(const std::string &want_table, const std::string &want_oracle,
                                   const std::function<std::string()> &scenario) {
  if (!real_flag())
    GTEST_SKIP() << "needs the real table (CVC_LOCKS_MODEL_REAL=0)";
  for (const mutation_side side : {mutation_side::table, mutation_side::oracle}) {
    const bool table = side == mutation_side::table;
    const std::string &want = table ? want_table : want_oracle;
    const std::string who = table ? "the table" : "the oracle";
    const std::string tag = !table ? "" : want == "lockstep" ? "lockstep:" : "[table]";
    SCOPED_TRACE(who + " alone");
    g_mutation_side = side;
    try {
      const std::string r = scenario();
      ADD_FAILURE() << "no violation (wanted " << want << " from " << who << "): " << r;
    } catch (const lock_violation &v) {
      EXPECT_EQ(v.invariant(), want) << v.what();
      EXPECT_NE(std::string(v.what()).find(tag), std::string::npos) << v.what();
      std::printf("  %s alone, caught as expected (%s): %s\n", who.c_str(), want.c_str(), v.what());
    } catch (const std::exception &e) {
      ADD_FAILURE() << who << " alone (wanted " << want << "): " << e.what();
    }
    g_mutation_side = mutation_side::both;
  }
}

TEST(LocksModel, Mutation_CancelSkipsRelevel_EachSide) {
  LT5_NEEDS_TABLE();
  expect_violation_on_each_side("I8", "I8",
                                [] { return scenario_pi11(dt::lock_table::cancel_skips_relevel); });
}
TEST(LocksModel, Mutation_CoalesceNoClear_EachSide) {
  LT5_NEEDS_TABLE();
  // On the table the second relevel is coalesced away while every inh stays right: only the
  // notifications show it, against the oracle's
  expect_violation_on_each_side(
      "lockstep", "I8", [] { return scenario_coalesce(dt::lock_table::coalesce_no_clear); });
}
TEST(LocksModel, Mutation_ImplNoteAfterOnly_EachSide) {
  LT5_NEEDS_TABLE();
  expect_violation_on_each_side("impl-note", "impl-note", [] {
    return scenario_impl_note(dt::lock_table::impl_note_after_only);
  });
}
TEST(LocksModel, Mutation_NoSetBase) {
  LT5_NEEDS_TABLE();
  // set-priority skips the home's set_base: the rec's base is stale (I8's base clause)
  expect_violation("I8", [] { return scenario_set_base_l2(no_set_base); });
}
