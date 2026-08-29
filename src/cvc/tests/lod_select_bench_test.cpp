/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Micro-benchmark for cvc/lod/select.h -- the per-frame cost of the LOD
// selection math. Gated on CVC_LOD_BENCH=1 so a bare `ctest` skips it (the
// numbers are meaningless on a shared CI runner and the header's claims are
// locked by lod_select_test.cpp regardless); run it on the target box with
//
//   CVC_LOD_BENCH=1 ctest --test-dir build -R LodSelectBench -V
//
// It measures the two things the header promises are cheap:
//   * select_rung() -- called once per visible group per frame;
//   * solver::solve() reused across frames -- the header claims "no allocation
//     on the hot path after one reserve", so the reused solver must not get
//     slower than a fresh solve() that reallocates every call.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/lod/select.h>
#include <gtest/gtest.h>
#include <string>
#include <vector>

using namespace cvc::lod;

namespace {

bool bench_enabled() {
  const char *v = std::getenv("CVC_LOD_BENCH");
  return v && std::string(v) == "1";
}

// A 32x32 tile grid over a 4 km span -- the Austin/Lab near-field grid
// (RENDER_PERF phase 1). Each tile carries the quartering 5-rung ladder every
// section 8.4 class has. Deterministic: distances fan out from a fixed lattice,
// no RNG (which the workflow/runtime forbids anyway).
struct scene {
  static constexpr int kGrid = 32;
  static constexpr int kTiles = kGrid * kGrid;
  std::uint64_t tris[5] = {8192, 2048, 512, 128, 32};
  std::uint64_t bytes[5];
  std::vector<candidate> cands;

  scene() {
    for (int i = 0; i < 5; ++i)
      bytes[i] = tris[i] * 48;
    cands.reserve(kTiles);
    const double tile_m = 4000.0 / kGrid;
    for (int gy = 0; gy < kGrid; ++gy) {
      for (int gx = 0; gx < kGrid; ++gx) {
        const double cx = (gx + 0.5) * tile_m;
        const double cy = (gy + 0.5) * tile_m;
        const double dist = std::sqrt(cx * cx + cy * cy); // camera at a corner
        candidate c;
        c.group_id = static_cast<std::uint32_t>(gy * kGrid + gx);
        c.nrungs = 5;
        c.desired_rung = 0;
        c.min_rung = 4;
        c.projected_area = 4.0e6 / (dist * dist + 1.0);
        c.dist_m = dist;
        c.tris_per_rung = tris;
        c.bytes_per_rung = bytes;
        cands.push_back(c);
      }
    }
  }
};

double ns_per(std::chrono::steady_clock::duration d, std::uint64_t ops) {
  const double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
  return ops ? ns / static_cast<double>(ops) : 0.0;
}

} // namespace

TEST(LodSelectBench, SelectRungThroughput) {
  if (!bench_enabled())
    GTEST_SKIP() << "Set CVC_LOD_BENCH=1 to enable";

  const double ladder[5] = {2.0, 4.0, 8.0, 16.0, 32.0};
  const view_params vp = preset_view(quality_preset::balanced);

  constexpr std::uint64_t N = 20'000'000;
  int current = -1;
  std::uint64_t checksum = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (std::uint64_t i = 0; i < N; ++i) {
    // Sweep the distance so the branch predictor cannot memoize one answer.
    const double d = 1.0 + static_cast<double>(i % 4000);
    current = select_rung(d, ladder, 5, current, vp);
    checksum += static_cast<std::uint64_t>(current);
  }
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_GT(checksum, 0u); // keep the loop from being optimized away
  std::printf("[lod bench] select_rung: %.2f ns/call, %.1f M calls/s\n", ns_per(dt, N),
              1e3 / ns_per(dt, N));
  RecordProperty("select_rung_ns", std::to_string(ns_per(dt, N)));
}

TEST(LodSelectBench, SolveReusedIsNotSlowerThanFresh) {
  if (!bench_enabled())
    GTEST_SKIP() << "Set CVC_LOD_BENCH=1 to enable";

  scene s;
  const budget b = preset_budget(budget_profile::desktop_default);
  constexpr std::uint64_t frames = 4000; // ~1 min of frames

  // Fresh: solve() allocates a new plan (and internal scratch) every call.
  std::uint64_t sink = 0;
  const auto f0 = std::chrono::steady_clock::now();
  for (std::uint64_t i = 0; i < frames; ++i) {
    const plan p = solve(s.cands, b);
    sink += p.tris;
  }
  const auto fresh = std::chrono::steady_clock::now() - f0;

  // Reused: one solver, one plan, reserved once -- the steady-state per-frame
  // path. The header's claim is that this allocates nothing after the reserve.
  solver slv;
  slv.reserve(s.cands.size());
  plan p;
  p.rung.reserve(s.cands.size());
  const auto r0 = std::chrono::steady_clock::now();
  for (std::uint64_t i = 0; i < frames; ++i) {
    slv.solve(s.cands, b, p);
    sink += p.tris;
  }
  const auto reused = std::chrono::steady_clock::now() - r0;

  EXPECT_GT(sink, 0u);
  const double fresh_ns = ns_per(fresh, frames);
  const double reused_ns = ns_per(reused, frames);
  std::printf("[lod bench] solve() %d tiles: fresh %.1f us/frame, reused %.1f us/frame (%.2fx)\n",
              scene::kTiles, fresh_ns / 1e3, reused_ns / 1e3, fresh_ns / reused_ns);
  RecordProperty("solve_fresh_us", std::to_string(fresh_ns / 1e3));
  RecordProperty("solve_reused_us", std::to_string(reused_ns / 1e3));

  // Reuse must not be materially slower than the allocating path. A generous
  // 1.5x bound so this is a real regression tripwire, not a timing-noise flake.
  EXPECT_LT(reused_ns, fresh_ns * 1.5)
      << "reused solver (" << reused_ns / 1e3 << " us) lost to fresh solve (" << fresh_ns / 1e3
      << " us) -- the hot path is allocating";
}
