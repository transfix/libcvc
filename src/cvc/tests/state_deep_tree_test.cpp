// cvc::state on deep trees. A write builds the node's full path (fullName()) and signals every
// ancestor (childChanged, the root's slots first); listing descendants (children()) walks the
// subtree; destroying a node destroys its subtree; json()/save() and their loaders, traverse()
// and reset() walk it too. None of that may recurse once per tree level, or a deep tree
// overflows the native stack. Each body runs on a thread with a small fixed stack (kStackKiB,
// wasm's 64 KiB), so per-level recursion fails here on every platform.
//
// For probing, CVC_STATE_DEEP_DEPTH overrides the depth and CVC_STATE_DEEP_STACK_KIB the stack
// (0 runs the body on the calling thread). CVC_STATE_BENCH=1 runs the write benchmark.

#include <algorithm>
#include <boost/chrono.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_distributed_admin.h>
#include <exception>
#include <filesystem>
#include <functional>
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr std::size_t kDepth = 100000;
constexpr std::size_t kStackKiB = 64;

std::size_t env_size(const char *name, std::size_t fallback) {
  const char *s = std::getenv(name);
  return s && *s ? static_cast<std::size_t>(std::strtoull(s, nullptr, 10)) : fallback;
}

std::size_t depth() { return env_size("CVC_STATE_DEEP_DEPTH", kDepth); }

// Run `body` on a thread with a kStackKiB stack and rethrow what it throws.
void on_small_stack(const std::function<void()> &body) {
  const std::size_t kib = env_size("CVC_STATE_DEEP_STACK_KIB", kStackKiB);
  if (kib == 0) {
    body();
    return;
  }
  std::exception_ptr error;
  boost::thread::attributes attrs;
  attrs.set_stack_size(kib * 1024);
  boost::thread t(attrs, [&] {
    try {
      body();
    } catch (...) {
      error = std::current_exception();
    }
  });
  t.join();
  if (error)
    std::rethrow_exception(error);
}

// A chain of `n` nodes named "n" under `top`, built one level per call; returns the deepest.
cvc::state &chain(cvc::state &top, std::size_t n) {
  cvc::state *node = &top;
  for (std::size_t i = 0; i < n; ++i)
    node = &(*node)("n");
  return *node;
}

std::string repeat_path(const std::string &prefix, std::size_t n) {
  std::string out = prefix;
  out.reserve(prefix.size() + 2 * n);
  for (std::size_t i = 0; i < n; ++i)
    out += ".n";
  return out;
}

class StateDeepTree : public ::testing::Test {
protected:
  cvc::app ctx;
  cvc::state &root = cvc::state::instance(ctx);
};

} // namespace

TEST_F(StateDeepTree, WriteDeepInTheTree) {
  const std::size_t n = depth();
  cvc::state &top = root("deep");
  cvc::state &leaf = chain(top, n);
  std::vector<std::string> at_root, at_top, at_parent;
  auto c1 = root.childChanged.connect([&](const std::string &p) { at_root.push_back(p); });
  auto c2 = top.childChanged.connect([&](const std::string &p) { at_top.push_back(p); });
  auto c3 =
      const_cast<cvc::state *>(leaf.parent())->childChanged.connect([&](const std::string &p) {
        at_parent.push_back(p);
      });
  std::string full;
  on_small_stack([&] {
    leaf.value("x");
    full = leaf.fullName();
  });
  const std::string expected = repeat_path("deep", n);
  EXPECT_EQ(full, expected);
  ASSERT_EQ(at_root.size(), 1u);
  EXPECT_EQ(at_root[0], expected);
  ASSERT_EQ(at_top.size(), 1u);
  EXPECT_EQ(at_top[0], expected.substr(std::string("deep.").size()));
  ASSERT_EQ(at_parent.size(), 1u);
  EXPECT_EQ(at_parent[0], "n");
  c1.disconnect();
  c2.disconnect();
  c3.disconnect();
  on_small_stack([&] { root("deep").reset(false); }); // destroys the chain
  EXPECT_EQ(root("deep").findDescendant("n"), nullptr);
}

TEST_F(StateDeepTree, DestroyDeepSubtree) {
  cvc::state &top = root("gone");
  chain(top, depth()).value("x");
  on_small_stack([&] { top.reset(false); });
  EXPECT_EQ(top.findDescendant("n"), nullptr);
  // The app's teardown destroys a deep tree too.
  chain(root("left_for_teardown"), depth());
}

TEST_F(StateDeepTree, ListDeepDescendants) {
  // Every descendant's full name: quadratic in depth by nature, so a shorter chain.
  const std::size_t n = std::min<std::size_t>(depth(), 3000);
  cvc::state &top = root("listed");
  chain(top, n);
  std::vector<std::string> names;
  on_small_stack([&] { names = top.children(); });
  ASSERT_EQ(names.size(), n);
  EXPECT_EQ(names.front(), "listed.n");
  EXPECT_EQ(names.back(), repeat_path("listed", n));
}

TEST_F(StateDeepTree, SignalsReachAncestorsRootFirst) {
  // One write signals the written node and each ancestor, the ROOT's slots first and the
  // parent's last, each with the path relative to it.
  cvc::state &leaf = root("o.p.q.r");
  std::vector<std::string> seen;
  auto connect = [&](cvc::state &s, const char *tag) {
    return s.childChanged.connect(
        [&seen, tag](const std::string &p) { seen.push_back(std::string(tag) + ":" + p); });
  };
  auto c1 = connect(root, "root");
  auto c2 = connect(root("o"), "o");
  auto c3 = connect(root("o.p"), "p");
  auto c4 = connect(root("o.p.q"), "q");
  auto c5 = leaf.valueChanged.connect([&] { seen.push_back("r:value"); });
  leaf.value("v");
  EXPECT_EQ(seen, (std::vector<std::string>{"r:value", "root:o.p.q.r", "o:p.q.r", "p:q.r", "q:r"}));
  for (auto *c : {&c1, &c2, &c3, &c4, &c5})
    c->disconnect();
}

TEST_F(StateDeepTree, NamesAndListsAreUnchanged) {
  EXPECT_EQ(root.fullName(), "");
  EXPECT_EQ(root("a").fullName(), "a");
  EXPECT_EQ(root("a.b.c").fullName(), "a.b.c");
  EXPECT_EQ(root("a.b.c").parentName(), "a.b");
  EXPECT_EQ(root("a").parentName(), "");
  root("a.b.d").value("1");
  root("a.e").value("2");
  EXPECT_EQ(root("a").children(),
            (std::vector<std::string>{"a.b", "a.b.c", "a.b.d", "a.e"})); // pre-order, map order
  EXPECT_EQ(root("a").children(".*d"), (std::vector<std::string>{"a.b.d"}));
  EXPECT_EQ(root("a").numChildren(), 2u);
}

TEST_F(StateDeepTree, PinnedDescendantKeepsItsSubtree) {
  chain(root("pinned"), 50).value("leaf");
  cvc::state::state_ptr mid = root("pinned").findDescendantShared(repeat_path("n", 9));
  ASSERT_TRUE(mid);
  root("pinned").reset(false); // drops the tree's ownership; `mid` still owns its part
  EXPECT_EQ(root("pinned").findDescendant("n"), nullptr);
  cvc::state *leaf = mid->findDescendant(repeat_path("n", 39));
  ASSERT_NE(leaf, nullptr);
  EXPECT_EQ(leaf->value(), "leaf");
}

// ---- links -------------------------------------------------------------------

namespace {
// `n` link nodes links.l0 -> links.l1 -> ... -> links.l<n-1> -> links.end (a value), or, with
// `cyclic`, l<n-1> -> l0.
void link_chain(cvc::state &root, std::size_t n, bool cyclic) {
  for (std::size_t i = 0; i < n; ++i) {
    const std::string next = i + 1 < n ? "links.l" + std::to_string(i + 1)
                                       : (cyclic ? std::string("links.l0") : "links.end");
    root("links.l" + std::to_string(i)).linkTo(next);
  }
  if (!cyclic)
    root("links.end").value("end");
}
} // namespace

TEST_F(StateDeepTree, LongLinkChainResolvesAndIsChecked) {
  // Following a chain of links (resolveLink, its hop budget raised to cover it) and finding the
  // cycles in the link graph (state_distributed_admin::link_cycles) walk link by link.
  const std::size_t n = std::min<std::size_t>(depth(), 20000);
  link_chain(root, n, false);
  cvc::state::link_resolution r;
  cvc::state_distributed_admin::link_cycles_result cycles;
  on_small_stack([&] {
    r = root("links.l0").resolveLink(n + 1);
    cycles = cvc::state_distributed_admin::link_cycles(root);
  });
  EXPECT_EQ(r.kind, cvc::state::link_resolution_kind::resolved);
  ASSERT_NE(r.target, nullptr);
  EXPECT_EQ(r.target->value(), "end");
  EXPECT_EQ(r.hops, n);
  EXPECT_EQ(cycles.link_nodes_scanned, n);
  EXPECT_TRUE(cycles.cycles.empty());
}

TEST_F(StateDeepTree, LongLinkCycleIsFound) {
  const std::size_t n = std::min<std::size_t>(depth(), 20000);
  link_chain(root, n, true);
  cvc::state_distributed_admin::link_cycles_result cycles;
  on_small_stack([&] { cycles = cvc::state_distributed_admin::link_cycles(root); });
  ASSERT_EQ(cycles.cycles.size(), 1u);
  EXPECT_EQ(cycles.cycles[0].size(), n);
  EXPECT_EQ(cycles.cycles[0].front(), "links.l0");
}

TEST_F(StateDeepTree, WriteThroughALinkSignalsTheTargetsAncestors) {
  // A writable transparent link routes a write to its target, deep in the tree: the target's
  // ancestors are signaled (the root's slots first), the link's are not.
  const std::size_t n = std::min<std::size_t>(depth(), 20000);
  cvc::state &target = chain(root("far"), n);
  cvc::state &link = root("near.alias");
  link.linkTo(target.fullName(), cvc::state::link_mode::transparent);
  link.setLinkWritable(true);
  std::vector<std::string> at_root, at_near;
  auto c1 = root.childChanged.connect([&](const std::string &p) { at_root.push_back(p); });
  auto c2 = root("near").childChanged.connect([&](const std::string &p) { at_near.push_back(p); });
  on_small_stack([&] { link.value("through"); });
  EXPECT_EQ(target.value(), "through");
  EXPECT_EQ(link.resolvedValue(), "through");
  ASSERT_EQ(at_root.size(), 1u);
  EXPECT_EQ(at_root[0], repeat_path("far", n));
  EXPECT_TRUE(at_near.empty());
  c1.disconnect();
  c2.disconnect();
}

// ---- json / save / restore -----------------------------------------------------

namespace {
// The values down the chain of "n" nodes under `top`, top first. Creates nothing.
std::vector<std::string> chain_values(cvc::state &top) {
  std::vector<std::string> out;
  for (cvc::state *node = &top; node; node = node->findDescendant("n"))
    out.push_back(node->value());
  return out;
}

// The entries of a json() document, in order, as (key, value). An entry that nests fails.
std::vector<std::pair<std::string, std::string>> json_entries(const std::string &j) {
  std::stringstream ss(j);
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(ss, pt);
  std::vector<std::pair<std::string, std::string>> out;
  for (const boost::property_tree::ptree::value_type &kv : pt) {
    EXPECT_TRUE(kv.second.empty()) << kv.first << " nests";
    out.emplace_back(kv.first, kv.second.data());
  }
  return out;
}
} // namespace

TEST_F(StateDeepTree, JsonRoundTripsADeepTree) {
  // Every node is keyed by its full path, so the document is quadratic in depth, and loading it
  // resolves each key from the root: a shorter chain. json() and save() write it, json(j) and
  // restore() read it into a fresh app.
  const std::size_t n = std::min<std::size_t>(depth(), 1000);
  cvc::state *node = &root("saved");
  node->value("0");
  for (std::size_t i = 1; i <= n; ++i) {
    node = &(*node)("n");
    node->value(std::to_string(i));
  }
  const std::vector<std::string> expected = chain_values(root("saved"));
  ASSERT_EQ(expected.size(), n + 1);
  const std::string file =
      (std::filesystem::temp_directory_path() / "state_deep_tree_save.json").string();
  std::string j;
  on_small_stack([&] {
    j = root("saved").json();
    root("saved").save(file);
  });
  cvc::app from_json, from_file;
  cvc::state &r1 = cvc::state::instance(from_json);
  cvc::state &r2 = cvc::state::instance(from_file);
  on_small_stack([&] {
    r1.json(j);
    r2.restore(file);
  });
  std::error_code ec;
  std::filesystem::remove(file, ec);
  EXPECT_EQ(chain_values(r1("saved")), expected);
  EXPECT_EQ(chain_values(r2("saved")), expected);
}

TEST_F(StateDeepTree, JsonIsOneLevelOfFullPaths) {
  // One entry per node, keyed by its full path, in pre-order (children in name order): the
  // node's value, or "" for a node with neither a value nor children. Those are the leaves of
  // the nested document json() used to write, which still loads.
  root("j").value("1");
  root("j.b.c").value("3");
  root("j.b.e").value("5");
  root("j.d");
  const std::vector<std::pair<std::string, std::string>> expected{
      {"j", "1"}, {"j.b.c", "3"}, {"j.b.e", "5"}, {"j.d", ""}};
  EXPECT_EQ(json_entries(root("j").json()), expected);
  boost::property_tree::ptree pt = root("j").ptree();
  ASSERT_EQ(pt.size(), expected.size());
  for (const boost::property_tree::ptree::value_type &kv : pt)
    EXPECT_TRUE(kv.second.empty()) << kv.first << " nests";

  const std::string nested =
      R"({"j": "1", "j.b": {"j.b.c": {"j.b.c": "3"}, "j.b.e": {"j.b.e": "5"}}, "j.d": ""})";
  cvc::app other;
  cvc::state &r = cvc::state::instance(other);
  r.json(nested);
  EXPECT_EQ(r("j").value(), "1");
  EXPECT_EQ(r("j.b.c").value(), "3");
  EXPECT_EQ(r("j.b.e").value(), "5");
  // The loader skips empty values, so j.d is not recreated.
  EXPECT_EQ(json_entries(r("j").json()), (std::vector<std::pair<std::string, std::string>>{
                                             {"j", "1"}, {"j.b.c", "3"}, {"j.b.e", "5"}}));
}

// ---- traverse ------------------------------------------------------------------

TEST_F(StateDeepTree, TraverseVisitsEachNodeOnce) {
  // Pre-order, children in name order, each node once. A node's traverseEnter fires before func
  // sees it and its traverseExit after everything under it. With a pattern: the node traverse()
  // was called on, then the descendants whose full names match; the walk still goes under the
  // ones that do not.
  const std::vector<std::string> paths{"t", "t.b", "t.b.c", "t.b.d", "t.e"};
  std::vector<std::string> seen;
  std::vector<boost::signals2::connection> conns;
  for (const std::string &p : paths) {
    const std::string tag = p.substr(p.rfind('.') + 1);
    conns.push_back(root(p).traverseEnter.connect([&seen, tag] { seen.push_back("+" + tag); }));
    conns.push_back(root(p).traverseExit.connect([&seen, tag] { seen.push_back("-" + tag); }));
  }
  const auto record = [&seen](std::string name) { seen.push_back(name); };
  root("t").traverse(record);
  EXPECT_EQ(seen, (std::vector<std::string>{"+t", "t", "+b", "t.b", "+c", "t.b.c", "-c", "+d",
                                            "t.b.d", "-d", "-b", "+e", "t.e", "-e", "-t"}));
  seen.clear();
  root("t").traverse(record, ".*[ce]");
  EXPECT_EQ(seen,
            (std::vector<std::string>{"+t", "t", "+c", "t.b.c", "-c", "+e", "t.e", "-e", "-t"}));
  for (boost::signals2::connection &c : conns)
    c.disconnect();
}

TEST_F(StateDeepTree, TraverseADeepTree) {
  // func gets every node's full name, quadratic in depth by nature: a shorter chain.
  const std::size_t n = std::min<std::size_t>(depth(), 3000);
  cvc::state &top = root("walked");
  chain(top, n);
  std::size_t visits = 0, enters = 0, exits = 0;
  std::string last;
  auto c1 = top.traverseEnter.connect([&] { ++enters; });
  auto c2 = top.traverseExit.connect([&] { ++exits; });
  on_small_stack([&] {
    top.traverse([&](std::string name) {
      ++visits;
      last = std::move(name);
    });
  });
  EXPECT_EQ(visits, n + 1);
  EXPECT_EQ(last, repeat_path("walked", n));
  EXPECT_EQ(enters, 1u);
  EXPECT_EQ(exits, 1u);
  c1.disconnect();
  c2.disconnect();
}

// ---- reset ---------------------------------------------------------------------

TEST_F(StateDeepTree, ResetADeepTree) {
  cvc::state &top = root("cleared");
  top.value("top");
  cvc::state &leaf = chain(top, depth());
  leaf.value("leaf");
  on_small_stack([&] { top.reset(true, false); });
  EXPECT_EQ(top.value(), "");
  EXPECT_EQ(leaf.value(), "");
  EXPECT_FALSE(leaf.initialized());
  EXPECT_FALSE(leaf.parent()->initialized());
  // reset(true) keeps every node.
  EXPECT_EQ(top.findDescendant(repeat_path("n", depth() - 1)), &leaf);
}

TEST_F(StateDeepTree, ResetADeepTreeSignalsLeafFirst) {
  // Each touch signals every ancestor, quadratic in depth: a shorter chain.
  const std::size_t n = std::min<std::size_t>(depth(), 3000);
  cvc::state &top = root("touched");
  chain(top, n).value("leaf");
  std::vector<std::string> heard;
  auto c = root.childChanged.connect([&](const std::string &p) { heard.push_back(p); });
  on_small_stack([&] { top.reset(); });
  c.disconnect();
  ASSERT_EQ(heard.size(), n + 1);
  EXPECT_EQ(heard.front(), repeat_path("touched", n));
  EXPECT_EQ(heard.back(), "touched");
}

TEST_F(StateDeepTree, ResetClearsTopDownAndTouchesBottomUp) {
  // reset(true) clears a node before the nodes under it and touches it after them, a subtree at
  // a time: when c (the first leaf) is touched, its later sibling d is not cleared yet.
  const std::vector<std::string> paths{"r", "r.b", "r.b.c", "r.b.d", "r.e"};
  for (const std::string &p : paths)
    root(p).value(p);
  std::vector<std::string> seen;
  std::vector<boost::signals2::connection> conns;
  for (const std::string &p : paths)
    conns.push_back(root(p).valueChanged.connect([&seen, p] { seen.push_back(p); }));
  conns.push_back(
      root.childChanged.connect([&](const std::string &p) { seen.push_back("^" + p); }));
  cvc::state &d = root("r.b.d");
  std::string d_at_c;
  conns.push_back(root("r.b.c").valueChanged.connect([&] { d_at_c = d.value(); }));
  root("r").reset();
  EXPECT_EQ(seen, (std::vector<std::string>{"r.b.c", "^r.b.c", "r.b.d", "^r.b.d", "r.b", "^r.b",
                                            "r.e", "^r.e", "r", "^r"}));
  EXPECT_EQ(d_at_c, "r.b.d");
  for (boost::signals2::connection &c : conns)
    c.disconnect();
}

TEST_F(StateDeepTree, ResetSignalsWithoutTheAncestorsLocks) {
  // A slot that finds its node from the root (as the sync adapter's do) or reads an ancestor
  // takes the ancestors' locks, so reset() must not hold them while it signals. Nor while
  // reset(false) destroys the subtree and fires its destroyed() slots.
  root("w.b.c").value("c");
  root("w").value("w");
  std::string at_c = "unset";
  auto c1 = root("w.b.c").valueChanged.connect(
      [&] { at_c = root("w.b.c").value() + "|" + root("w.b").value() + "|" + root("w").value(); });
  root("z.k").value("k");
  root("z").value("z");
  std::string at_destroy = "unset";
  auto c2 = root("z.k").destroyed.connect([&] { at_destroy = root("z").value(); });
  boost::thread t([&] {
    root("w").reset();
    root("z").reset(false);
  });
  if (!t.try_join_for(boost::chrono::seconds(60))) {
    // Deadlocked on a lock it holds itself; nothing can unblock it.
    std::fprintf(stderr, "reset() deadlocked: it signals while holding an ancestor's lock\n");
    std::fflush(stderr);
    std::_Exit(1);
  }
  EXPECT_EQ(at_c, "||"); // its ancestors were cleared first
  EXPECT_EQ(at_destroy, "");
  c1.disconnect();
  c2.disconnect();
}

TEST_F(StateDeepTree, WriteBench) {
  if (env_size("CVC_STATE_BENCH", 0) != 1) {
    SUCCEED();
    return;
  }
  // A write 30 levels deep with a listener on the root (as the sync adapter and pycvc's watch
  // have): fullName(), and the signal to every ancestor.
  cvc::state &leaf = chain(root("bench"), 30);
  std::size_t heard = 0;
  auto c = root.childChanged.connect([&](const std::string &) { ++heard; });
  constexpr int N = 200000;
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < N; ++i)
    leaf.value(std::to_string(i & 1));
  auto t1 = std::chrono::steady_clock::now();
  const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
  std::printf("BENCH state_write_depth30 N=%d wall_ns_total=%lld wall_ns_per_op=%lld\n", N,
              static_cast<long long>(ns), static_cast<long long>(ns / N));
  EXPECT_EQ(heard, static_cast<std::size_t>(N));
  c.disconnect();
}
