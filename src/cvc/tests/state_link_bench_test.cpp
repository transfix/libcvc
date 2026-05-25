/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Phase 8 benchmark: 1M-path tree with 10k links (including a few
// cycles). Asserts that resolver and subscription latency stay
// bounded. Gated on CVC_DISTRIBUTED_STATE_BENCH=1 so normal CI is
// not affected; always-on correctness companions run unconditionally.
//
// Output format (when enabled):
//   BENCH <name> N=<count> wall_ns_total=<n> wall_ns_per_op=<n>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_distributed_admin.h>
#include <cvc/core/state_sync_adapter.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <vector>

namespace {

bool bench_enabled() {
  const char *v = std::getenv("CVC_DISTRIBUTED_STATE_BENCH");
  return v && std::string(v) == "1";
}

void report(const char *name, std::size_t n, std::uint64_t wall_ns) {
  std::uint64_t per = (n == 0) ? 0u : wall_ns / n;
  std::printf("BENCH %s N=%zu wall_ns_total=%llu wall_ns_per_op=%llu\n", name, n,
              static_cast<unsigned long long>(wall_ns), static_cast<unsigned long long>(per));
}

// Build a flat tree of `count` leaf paths under randomised 3-level
// prefixes: root8/mid5/leaf_N. Returns the generated leaf paths.
std::vector<std::string> build_flat_tree(cvc::state &root, std::size_t count,
                                         std::uint32_t seed = 42) {
  static const char *roots[] = {"vol", "geom", "mesh", "cam", "scene", "ui", "fx", "net"};
  static const char *mids[] = {"a", "b", "c", "d", "e"};
  std::mt19937 rng(seed);
  std::vector<std::string> paths;
  paths.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::string p = roots[rng() % 8];
    p += '.';
    p += mids[rng() % 5];
    p += ".leaf";
    p += std::to_string(i);
    paths.push_back(p);
    root(p); // materialise the node
  }
  return paths;
}

// Install `link_count` transparent links in the tree, pointing at
// randomly chosen target paths. Injects `cycle_count` 2-link cycles
// at the end (A→B, B→A). Returns the link source paths.
std::vector<std::string> install_links(cvc::state &root, const std::vector<std::string> &targets,
                                       std::size_t link_count, std::size_t cycle_count,
                                       std::uint32_t seed = 99) {
  std::mt19937 rng(seed);
  std::vector<std::string> link_paths;
  link_paths.reserve(link_count);

  std::size_t normal_links = link_count > cycle_count * 2 ? link_count - cycle_count * 2 : 0;

  // Normal transparent links.
  for (std::size_t i = 0; i < normal_links; ++i) {
    std::string lp = "link." + std::to_string(i);
    std::string tp = targets[rng() % targets.size()];
    root(lp).linkTo(tp, cvc::state::link_mode::transparent);
    link_paths.push_back(std::move(lp));
  }

  // Cycle pairs.
  for (std::size_t c = 0; c < cycle_count; ++c) {
    std::string a = "cycle." + std::to_string(c) + ".a";
    std::string b = "cycle." + std::to_string(c) + ".b";
    root(a).linkTo(b, cvc::state::link_mode::transparent);
    root(b).linkTo(a, cvc::state::link_mode::transparent);
    link_paths.push_back(a);
    link_paths.push_back(b);
  }

  return link_paths;
}

} // namespace

// =====================================================================
// Opt-in benchmarks (CVC_DISTRIBUTED_STATE_BENCH=1)
// =====================================================================

// -- resolveLink over 10k links in a 1M-path tree --------------------
TEST(StateLinkBenchmark, ResolveLinkLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 1'000'000;
  constexpr std::size_t kLinks = 10'000;
  constexpr std::size_t kCycles = 50;

  auto paths = build_flat_tree(root, kPaths);
  auto links = install_links(root, paths, kLinks, kCycles);

  // Warm up the link resolver once.
  for (auto &lp : links)
    (void)root(lp).resolveLink();

  auto t0 = std::chrono::steady_clock::now();
  std::size_t resolved = 0, cycles = 0;
  for (auto &lp : links) {
    auto r = root(lp).resolveLink();
    if (r.kind == cvc::state::link_resolution_kind::resolved)
      ++resolved;
    else if (r.kind == cvc::state::link_resolution_kind::cycle_detected)
      ++cycles;
  }
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("resolve_link_10k_in_1M_tree", links.size(), wall_ns);

  EXPECT_EQ(resolved + cycles, links.size());
  EXPECT_EQ(cycles, kCycles * 2); // both halves of each cycle pair
  // Sanity: per-op well under 500 µs even in a debug build.
  EXPECT_LT(wall_ns / links.size(), 500'000u) << "resolveLink per-op > 500 µs; possible regression";
}

// -- resolvedValue over 10k links in a 1M-path tree ------------------
TEST(StateLinkBenchmark, ResolvedValueLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 1'000'000;
  constexpr std::size_t kLinks = 10'000;
  constexpr std::size_t kCycles = 50;

  auto paths = build_flat_tree(root, kPaths);
  auto links = install_links(root, paths, kLinks, kCycles);

  // Set a value on every target so resolved reads are meaningful.
  for (std::size_t i = 0; i < kPaths; ++i)
    root(paths[i]).value(std::string("v") + std::to_string(i));

  auto t0 = std::chrono::steady_clock::now();
  std::size_t non_empty = 0;
  for (auto &lp : links) {
    std::string v = root(lp).resolvedValue();
    if (!v.empty())
      ++non_empty;
  }
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("resolved_value_10k_in_1M_tree", links.size(), wall_ns);

  // Every link should yield a value: normal links resolve to a
  // populated target; cycle links fall back to their own value
  // (which is empty, but that still counts as a string return).
  EXPECT_EQ(non_empty + kCycles * 2, links.size());
  EXPECT_LT(wall_ns / links.size(), 500'000u)
      << "resolvedValue per-op > 500 µs; possible regression";
}

// -- resolveRemote over 10k links in a 1M-path tree ------------------
TEST(StateLinkBenchmark, ResolveRemoteLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node-1");
  shard.attach();
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 1'000'000;
  constexpr std::size_t kLinks = 10'000;
  constexpr std::size_t kCycles = 50;

  auto paths = build_flat_tree(root, kPaths);
  auto links = install_links(root, paths, kLinks, kCycles);

  auto t0 = std::chrono::steady_clock::now();
  std::size_t local = 0, cycles = 0;
  for (auto &lp : links) {
    auto r = root(lp).resolveRemote();
    if (r.kind == cvc::state::remote_resolution_kind::resolved_local)
      ++local;
    else if (r.kind == cvc::state::remote_resolution_kind::cycle_detected)
      ++cycles;
  }
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("resolve_remote_10k_in_1M_tree", links.size(), wall_ns);

  EXPECT_EQ(local + cycles, links.size());
  EXPECT_EQ(cycles, kCycles * 2);
  EXPECT_LT(wall_ns / links.size(), 500'000u)
      << "resolveRemote per-op > 500 µs; possible regression";
}

// -- transparent_link_index over 5k links in a 100k-path tree --------
// Uses a smaller tree than the resolve benchmarks because the index
// walker visits every node in the tree, not just links.
TEST(StateLinkBenchmark, TransparentLinkIndexLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 100'000;
  constexpr std::size_t kLinks = 5'000;
  constexpr std::size_t kCycles = 25;

  auto paths = build_flat_tree(root, kPaths);
  install_links(root, paths, kLinks, kCycles);

  auto t0 = std::chrono::steady_clock::now();
  auto idx = cvc::state_distributed_admin::transparent_link_index(root);
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("transparent_link_index_100k_tree_5k_links", 1u, wall_ns);

  EXPECT_EQ(idx.links.size(), kLinks);
  // Full-tree scan of 100k nodes: < 30 s even in a debug build.
  EXPECT_LT(wall_ns, 30'000'000'000ULL) << "transparent_link_index > 30 s";
}

// -- transparent_link_aliases sampling 10 paths in a 100k tree -------
TEST(StateLinkBenchmark, TransparentLinkAliasesLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 100'000;
  constexpr std::size_t kLinks = 5'000;
  constexpr std::size_t kCycles = 25;
  constexpr std::size_t kSamples = 10;

  auto paths = build_flat_tree(root, kPaths);
  install_links(root, paths, kLinks, kCycles);

  std::mt19937 rng(77);
  std::vector<std::string> samples;
  samples.reserve(kSamples);
  for (std::size_t i = 0; i < kSamples; ++i)
    samples.push_back(paths[rng() % paths.size()]);

  auto t0 = std::chrono::steady_clock::now();
  std::size_t total_aliases = 0;
  for (auto &p : samples) {
    auto aliases = cvc::state_distributed_admin::transparent_link_aliases(root, p);
    total_aliases += aliases.size();
  }
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("transparent_link_aliases_10_samples_in_100k_tree", kSamples, wall_ns);

  // Each call walks the whole link index; 10 samples × full
  // tree scan. Allow up to 120 s in a debug build.
  EXPECT_LT(wall_ns, 120'000'000'000ULL) << "alias expansion > 120 s";
  EXPECT_GE(total_aliases, 0u);
}

// -- subscriptions_for_path with adapter over 10 paths in 100k tree --
TEST(StateLinkBenchmark, SubscriptionsForPathLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  cvc::state_cluster_shard shard(a, "alpha", "node-1");
  shard.attach();
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 100'000;
  constexpr std::size_t kLinks = 5'000;
  constexpr std::size_t kCycles = 25;
  constexpr std::size_t kSamples = 10;

  auto paths = build_flat_tree(root, kPaths);
  install_links(root, paths, kLinks, kCycles);

  // Install some subscriptions on a few root prefixes.
  auto &router = shard.router();
  router.subscribe("vol");
  router.subscribe("geom");
  router.subscribe("mesh");
  router.subscribe("link");

  std::mt19937 rng(55);
  std::vector<std::string> samples;
  samples.reserve(kSamples);
  for (std::size_t i = 0; i < kSamples; ++i)
    samples.push_back(paths[rng() % paths.size()]);

  auto t0 = std::chrono::steady_clock::now();
  std::size_t total_subs = 0;
  for (auto &p : samples) {
    auto subs = shard.adapter().subscriptions_for_path(p);
    total_subs += subs.size();
  }
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("subscriptions_for_path_10_samples_in_100k_tree", kSamples, wall_ns);

  EXPECT_GT(total_subs, 0u);
  // 10 samples × full tree scan: < 120 s in a debug build.
  EXPECT_LT(wall_ns, 120'000'000'000ULL) << "subscriptions_for_path > 120 s";
}

// -- link_cycles over 5k links in a 100k-path tree --------------------
TEST(StateLinkBenchmark, LinkCyclesLargeTree) {
  if (!bench_enabled()) {
    SUCCEED() << "set CVC_DISTRIBUTED_STATE_BENCH=1 to run";
    return;
  }

  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 100'000;
  constexpr std::size_t kLinks = 5'000;
  constexpr std::size_t kCycles = 25;

  auto paths = build_flat_tree(root, kPaths);
  install_links(root, paths, kLinks, kCycles);

  auto t0 = std::chrono::steady_clock::now();
  auto result = cvc::state_distributed_admin::link_cycles(root);
  auto t1 = std::chrono::steady_clock::now();

  auto wall_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
  report("link_cycles_100k_tree_5k_links", 1u, wall_ns);

  EXPECT_EQ(result.cycles.size(), kCycles);
  EXPECT_GE(result.link_nodes_scanned, kLinks);
  // Full-tree scan: < 30 s even in a debug build.
  EXPECT_LT(wall_ns, 30'000'000'000ULL) << "link_cycles > 30 s";
}

// =====================================================================
// Always-on correctness companions (run in normal CI)
// =====================================================================

// Small-scale correctness: verify the benchmark helpers produce the
// expected tree shape and link topology.
TEST(StateLinkBenchCorrectness, BuildAndInstallLinks) {
  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 100;
  constexpr std::size_t kLinks = 20;
  constexpr std::size_t kCycles = 3;

  auto paths = build_flat_tree(root, kPaths);
  EXPECT_EQ(paths.size(), kPaths);
  // All paths should be reachable.
  for (auto &p : paths)
    EXPECT_NE(root.findDescendant(p), nullptr) << "missing: " << p;

  auto links = install_links(root, paths, kLinks, kCycles);
  EXPECT_EQ(links.size(), kLinks);

  // Every link node should exist and be a link.
  for (auto &lp : links) {
    auto *n = root.findDescendant(lp);
    ASSERT_NE(n, nullptr) << "missing link node: " << lp;
    EXPECT_TRUE(n->isLink()) << lp;
    EXPECT_EQ(n->linkMode(), cvc::state::link_mode::transparent) << lp;
  }

  // Cycle pairs should form detectable cycles.
  std::size_t cycle_count = 0;
  for (auto &lp : links) {
    auto r = root(lp).resolveLink();
    if (r.kind == cvc::state::link_resolution_kind::cycle_detected)
      ++cycle_count;
  }
  EXPECT_EQ(cycle_count, kCycles * 2);
}

TEST(StateLinkBenchCorrectness, ResolvedValueOnSmallTree) {
  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 50;
  constexpr std::size_t kLinks = 10;
  constexpr std::size_t kCycles = 2;

  auto paths = build_flat_tree(root, kPaths);
  auto links = install_links(root, paths, kLinks, kCycles);

  for (auto &p : paths)
    root(p).value(std::string("val_") + p);

  // Non-cycle links should resolve to the target's value.
  std::size_t correct = 0;
  for (auto &lp : links) {
    auto r = root(lp).resolveLink();
    if (r.kind == cvc::state::link_resolution_kind::resolved && r.target != nullptr) {
      std::string rv = root(lp).resolvedValue();
      EXPECT_EQ(rv, r.target->value()) << lp;
      ++correct;
    }
  }
  EXPECT_GT(correct, 0u);
}

TEST(StateLinkBenchCorrectness, LinkCyclesOnSmallTree) {
  cvc::app a;
  auto &root = cvc::state::instance(a);
  constexpr std::size_t kPaths = 50;
  constexpr std::size_t kLinks = 10;
  constexpr std::size_t kCycles = 2;

  build_flat_tree(root, kPaths);
  // Can't pass paths to install_links targets without them, but
  // build_flat_tree already materialised the nodes.
  auto paths = build_flat_tree(root, kPaths, 123); // different seed for variety
  install_links(root, paths, kLinks, kCycles);

  auto result = cvc::state_distributed_admin::link_cycles(root);
  EXPECT_EQ(result.cycles.size(), kCycles);
  for (auto &cycle : result.cycles) {
    EXPECT_GE(cycle.size(), 2u);
  }
}
