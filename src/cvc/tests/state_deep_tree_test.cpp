// cvc::state on deep trees. A write builds the node's full path (fullName()) and signals every
// ancestor (childChanged, the root's slots first); listing descendants (children()) walks the
// subtree; destroying a node destroys its subtree. None of that may recurse once per tree
// level, or a deep tree overflows the native stack. Each body runs on a thread with a small
// fixed stack (kStackKiB, wasm's 64 KiB), so per-level recursion fails here on every platform.
//
// For probing, CVC_STATE_DEEP_DEPTH overrides the depth and CVC_STATE_DEEP_STACK_KIB the stack
// (0 runs the body on the calling thread). CVC_STATE_BENCH=1 runs the write benchmark.

#include <algorithm>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_distributed_admin.h>
#include <exception>
#include <functional>
#include <gtest/gtest.h>
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
