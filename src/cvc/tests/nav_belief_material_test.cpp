/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// nav_belief_material_test: belief (observed) material (belief_material.h) and the
// sense_batch material-range mask (grid_nav.h). docs/roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md.
//
// - The kernels against brute-force references: the per-cell select, the gate surface's
//   OR and its clearance chain, and the range disc against a full numpy-linspace scan.
// - sense_batch: a null material mask changes nothing; at the default range the mask
//   grows exactly as ever_seen; a shorter range is the field of view of a cast at that
//   range (square cells), and material_changed flags exactly the planes that grew.
// - Every cell seen gives back the truth rasters, so material_build is the oracle's.
// - NavBeliefMaterial.Golden: the BIT golden GRL-SNAM's numpy twin is held to
//   (src/cvc/tests/data/belief_material_golden.json). CVC_BELIEF_MATERIAL_GOLDEN_WRITE=1
//   rewrites it; otherwise this build must reproduce every hash and gate output exactly.

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/nav/belief_material.h>
#include <cvc/nav/belief_occupancy.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/material.h>
#include <cvc/state/state_blob_store.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::nav;

namespace {

// ── a small scene ─────────────────────────────────────────────────────────────
// 32 x 32 corner samples on [0, 31]^2 (1 m cells). Two buildings are in the sense
// truth (they occlude). The hard material is a pond and a canopy outside the sense
// truth (seen across) plus the first building; a risk-0.6 patch sits east.
constexpr int kR = 32, kC = 32;
constexpr double kMin = 0.0, kMax = 31.0;

struct Scene {
  std::vector<std::uint8_t> truth, hard;
  std::vector<float> risk;
  Scene() : truth(kR * kC, 0), hard(kR * kC, 0), risk(kR * kC, 0.05f) {
    auto rect = [](std::vector<std::uint8_t> &m, int r0, int r1, int c0, int c1) {
      for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
          m[r * kC + c] = 1;
    };
    auto rrect = [this](int r0, int r1, int c0, int c1, float v) {
      for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c)
          risk[r * kC + c] = v;
    };
    rect(truth, 10, 15, 12, 17); // building A
    rect(truth, 20, 25, 4, 8);   // building B
    rect(hard, 3, 7, 20, 27);    // pond
    rect(hard, 26, 29, 18, 22);  // canopy
    rect(hard, 10, 15, 12, 17);  // building A is hard too
    rrect(14, 22, 20, 28, 0.6f); // the risky patch
    rrect(3, 7, 20, 27, 1.0f);
    rrect(26, 29, 18, 22, 1.0f);
    rrect(10, 15, 12, 17, 1.0f);
    rrect(20, 25, 4, 8, 1.0f);
  }
};

// Three agents on two planes (agents 0 and 1 share plane 0), three sense ticks of
// scripted poses (no drive, so nothing FLOAT enters the golden), and fixed goals.
constexpr int kN = 3, kM = 2, kTicks = 3;
const int kMap[kN] = {0, 0, 1};
const double kPose[kTicks][kN][3] = {
    {{4.3, 4.7, 0.1}, {24.5, 21.5, -1.45}, {9.6, 27.1, -0.7}},
    {{8.1, 6.2, 0.4}, {24.3, 20.8, -1.52}, {12.4, 24.3, -1.1}},
    {{11.9, 7.4, 0.9}, {24.1, 20.1, -1.61}, {15.8, 21.6, -1.3}},
};
// Agent 1 stands in the risky patch with its goal straight across it, so the gate can fire.
const double kGoal[kN][2] = {{27.5, 20.5}, {24.0, 1.5}, {28.0, 27.0}};
constexpr double kRange = 12.0, kFov = 6.283185307179586;
constexpr int kRays = 90;
constexpr double kLocc = 2.2, kLfree = -1.4, kLclamp = 8.0;
constexpr double kPthresh = 0.5, kBand = 0.15;
constexpr double kCellW = 1.0, kScale = 0.0625, kSigma = 1.0;

gate_params golden_gate() {
  gate_params g;
  g.primitive_count = 16;
  g.horizon_cells = 8;
  g.hard_margin_m = 2.0;
  g.improvement_margin = 0.05;
  g.material_trigger = 0.45;
  g.progress_slack_cells = 0.5;
  return g;
}

struct Case {
  const char *name;
  belief_material_params p;
};

std::vector<Case> golden_cases() {
  std::vector<Case> cs;
  belief_material_params a; // the defaults
  cs.push_back({"defaults", a});
  belief_material_params b;
  b.material_range_m = 7.0;
  cs.push_back({"material_range_7", b});
  belief_material_params c;
  c.rule = seen_rule::disc;
  cs.push_back({"disc", c});
  belief_material_params d;
  d.unknown_risk = 0.5f;
  d.unknown_hard = 1;
  d.unknown_gate_hard = false;
  cs.push_back({"pessimistic_ungated_unknown", d});
  return cs;
}

template <class T> std::string hash_of(const std::vector<T> &v, std::size_t off, std::size_t n) {
  return cvc::sha256_hex(reinterpret_cast<const unsigned char *>(v.data() + off), n * sizeof(T));
}

std::string num(double x) {
  if (!std::isfinite(x)) // JSON has no inf/nan: quote them
    return std::isnan(x) ? "\"nan\"" : (x > 0 ? "\"inf\"" : "\"-inf\"");
  char b[40];
  std::snprintf(b, sizeof b, "%.17g", x);
  return b;
}

// A golden double: a number, or "inf" / "-inf" / "nan" for the non-finite ones.
bool same_double(const std::string &golden, double now) {
  if (golden == "nan")
    return std::isnan(now);
  if (golden == "inf")
    return now == std::numeric_limits<double>::infinity();
  if (golden == "-inf")
    return now == -std::numeric_limits<double>::infinity();
  return std::isfinite(now) && std::stod(golden) == now;
}

// One plane's belief-material state at one tick.
struct PlaneOut {
  std::vector<std::uint8_t> mask, occ, hard_b, gate_hard;
  std::vector<float> risk_b, clear_m, planes6; // planes6 is [6, H, W]
};

struct TickOut {
  PlaneOut pl[kM];
  std::uint8_t active[kN];
  double nominal[kN], best[kN];
  std::int32_t count[kN];
};

// Run one case through every tick: sense, the material mask, the planning raster,
// the belief inputs, material_build, the gate surface and the batched gate.
std::vector<TickOut> run_case(const Scene &sc, const belief_material_params &p) {
  const int hw = kR * kC;
  std::vector<float> lo(kM * hw, 0.0f);
  std::vector<std::uint8_t> lv(kM * hw, 0), es(kM * hw, 0), ms(kM * hw, 0);
  std::vector<std::int32_t> ver(kM, 0), flips(kN, 0);
  std::vector<std::uint8_t> mchg(kM, 0);
  std::vector<double> rng(kN, kRange), fov(kN, kFov), mrange(kN, p.material_range_m);
  std::vector<std::int32_t> nr(kN, kRays), amap(kMap, kMap + kN);
  std::vector<TickOut> out;
  for (int t = 0; t < kTicks; ++t) {
    std::vector<double> pos(2 * kN), head(kN);
    for (int i = 0; i < kN; ++i) {
      pos[2 * i] = kPose[t][i][0];
      pos[2 * i + 1] = kPose[t][i][1];
      head[i] = kPose[t][i][2];
    }
    sense_agents ag;
    ag.pos = pos.data();
    ag.heading = head.data();
    ag.range_m = rng.data();
    ag.fov_rad = fov.data();
    ag.n_rays = nr.data();
    ag.agent_map = amap.data();
    ag.n = kN;
    ag.material_range_m = mrange.data();
    belief_planes bp{lo.data(), lv.data(), es.data(), ver.data(), kM};
    if (p.rule == seen_rule::raycast) {
      bp.material_seen = ms.data();
      bp.material_changed = mchg.data();
    }
    sense_batch(sc.truth.data(), kR, kC, kMin, kMin, kMax, kMax, ag, nullptr, 0, nullptr, 0, bp,
                kLocc, kLfree, kLclamp, flips.data(), 1);
    if (p.rule == seen_rule::disc) {
      const double R = p.material_range_m > 0.0 ? p.material_range_m : kRange;
      for (int i = 0; i < kN; ++i)
        seen_disc(pos[2 * i], pos[2 * i + 1], R, kR, kC, kMin, kMin, kMax, kMax,
                  ms.data() + static_cast<std::size_t>(kMap[i]) * hw);
    }
    TickOut to{};
    std::vector<float> risk_all(kM * hw);
    std::vector<std::uint8_t> gh_all(kM * hw);
    std::vector<float> cl_all(kM * hw);
    for (int m = 0; m < kM; ++m) {
      PlaneOut &po = to.pl[m];
      po.mask.assign(ms.begin() + m * hw, ms.begin() + (m + 1) * hw);
      po.occ.assign(hw, 0);
      to_occupancy(lo.data() + m * hw, kR, kC, unknown_policy::optimistic, kPthresh, kBand,
                   po.occ.data());
      po.risk_b.assign(hw, 0.0f);
      po.hard_b.assign(hw, 0);
      belief_material_inputs(sc.risk.data(), sc.hard.data(), po.mask.data(), kR, kC, p,
                             po.risk_b.data(), po.hard_b.data());
      const material_planes mp =
          material_build(po.risk_b.data(), po.hard_b.data(), kR, kC, kCellW, kScale, kSigma);
      po.planes6 = mp.stacked();
      po.gate_hard.assign(hw, 0);
      po.clear_m.assign(hw, 0.0f);
      belief_gate_surface(po.hard_b.data(), po.occ.data(), po.mask.data(), kR, kC, kCellW, p,
                          po.gate_hard.data(), po.clear_m.data());
      std::copy(mp.risk.begin(), mp.risk.end(), risk_all.begin() + m * hw);
      std::copy(po.gate_hard.begin(), po.gate_hard.end(), gh_all.begin() + m * hw);
      std::copy(po.clear_m.begin(), po.clear_m.end(), cl_all.begin() + m * hw);
    }
    // Gate positions are continuous cell coordinates (row, col).
    std::vector<double> prc(2 * kN), grc(2 * kN);
    const double s = (kC - 1) / (kMax - kMin);
    for (int i = 0; i < kN; ++i) {
      prc[2 * i] = (pos[2 * i + 1] - kMin) * s;
      prc[2 * i + 1] = (pos[2 * i] - kMin) * s;
      grc[2 * i] = (kGoal[i][1] - kMin) * s;
      grc[2 * i + 1] = (kGoal[i][0] - kMin) * s;
    }
    witness_gate_batch(risk_all.data(), gh_all.data(), cl_all.data(), kR, kC, prc.data(),
                       grc.data(), kN, golden_gate(), to.active, to.nominal, to.best, to.count, 1,
                       kMap);
    out.push_back(std::move(to));
  }
  return out;
}

const char *kPlaneNames[6] = {"risk", "phi_m", "grad_rx", "grad_ry", "grad_px", "grad_py"};

// The per-raster hashes of one plane at one tick, in a fixed key order.
std::vector<std::pair<std::string, std::string>> plane_hashes(const PlaneOut &po) {
  const std::size_t hw = static_cast<std::size_t>(kR) * kC;
  std::vector<std::pair<std::string, std::string>> h;
  h.emplace_back("material_seen", hash_of(po.mask, 0, hw));
  h.emplace_back("occ", hash_of(po.occ, 0, hw));
  h.emplace_back("risk_b", hash_of(po.risk_b, 0, hw));
  h.emplace_back("hard_b", hash_of(po.hard_b, 0, hw));
  for (int ch = 0; ch < 6; ++ch)
    h.emplace_back(kPlaneNames[ch], hash_of(po.planes6, ch * hw, hw));
  h.emplace_back("gate_hard", hash_of(po.gate_hard, 0, hw));
  h.emplace_back("clear_m", hash_of(po.clear_m, 0, hw));
  return h;
}

template <class T> std::string arr(const T *v, std::size_t n) {
  std::string s = "[";
  for (std::size_t k = 0; k < n; ++k) {
    if (k)
      s += ",";
    s += num(static_cast<double>(v[k]));
  }
  return s + "]";
}

} // namespace

// ── the kernels ──────────────────────────────────────────────────────────────

TEST(NavBeliefMaterial, InputsSelectSeenAndPinUnseen) {
  const int R = 5, C = 7, hw = R * C;
  std::mt19937 rnd(7);
  std::vector<float> risk(hw);
  std::vector<std::uint8_t> hard(hw), seen(hw);
  for (int k = 0; k < hw; ++k) {
    risk[k] = static_cast<float>(rnd() % 1000) / 1000.0f;
    hard[k] = static_cast<std::uint8_t>(rnd() % 3); // 0, 1, 2: nonzero is hard
    seen[k] = static_cast<std::uint8_t>(rnd() % 2);
  }
  belief_material_params p;
  p.unknown_risk = 0.25f;
  p.unknown_hard = 1;
  std::vector<float> rb(hw);
  std::vector<std::uint8_t> hb(hw);
  belief_material_inputs(risk.data(), hard.data(), seen.data(), R, C, p, rb.data(), hb.data());
  for (int k = 0; k < hw; ++k) {
    if (seen[k]) {
      EXPECT_EQ(rb[k], risk[k]);
      EXPECT_EQ(hb[k], hard[k] ? 1 : 0);
    } else {
      EXPECT_EQ(rb[k], 0.25f);
      EXPECT_EQ(hb[k], 1);
    }
  }
}

TEST(NavBeliefMaterial, ParamsAreValidated) {
  std::vector<float> r(4, 0.0f), ro(4);
  std::vector<std::uint8_t> h(4, 0), s(4, 1), ho(4);
  auto run = [&](const belief_material_params &p) {
    belief_material_inputs(r.data(), h.data(), s.data(), 2, 2, p, ro.data(), ho.data());
  };
  belief_material_params p;
  EXPECT_NO_THROW(run(p));
  p.unknown_risk = 1.5f;
  EXPECT_THROW(run(p), std::invalid_argument);
  p.unknown_risk = std::nanf("");
  EXPECT_THROW(run(p), std::invalid_argument);
  p = belief_material_params{};
  p.unknown_hard = 2;
  EXPECT_THROW(run(p), std::invalid_argument);
  p = belief_material_params{};
  p.material_range_m = -1.0;
  EXPECT_THROW(run(p), std::invalid_argument);
}

TEST(NavBeliefMaterial, GateSurfaceIsTheOrAndTheClearanceChain) {
  const int R = 9, C = 11, hw = R * C;
  const double cw = 0.75;
  std::mt19937 rnd(11);
  std::vector<std::uint8_t> hb(hw), occ(hw), seen(hw);
  for (int k = 0; k < hw; ++k) {
    hb[k] = (rnd() % 9) == 0;
    occ[k] = (rnd() % 7) == 0;
    seen[k] = (rnd() % 4) != 0;
  }
  for (bool ugh : {true, false}) {
    belief_material_params p;
    p.unknown_gate_hard = ugh;
    std::vector<std::uint8_t> gh(hw);
    std::vector<float> cl(hw);
    belief_gate_surface(hb.data(), occ.data(), seen.data(), R, C, cw, p, gh.data(), cl.data());
    std::vector<std::uint8_t> want(hw);
    for (int k = 0; k < hw; ++k)
      want[k] = (hb[k] || occ[k] || (ugh && !seen[k])) ? 1 : 0;
    EXPECT_EQ(gh, want);
    const std::vector<double> d2 = edt2_squared(want.data(), R, C);
    for (int k = 0; k < hw; ++k)
      EXPECT_EQ(cl[k], static_cast<float>(std::sqrt(d2[k]) * cw)) << k;
  }
  // A null occupancy is no occupancy.
  belief_material_params p;
  std::vector<std::uint8_t> g1(hw), g2(hw), zero(hw, 0);
  std::vector<float> c1(hw), c2(hw);
  belief_gate_surface(hb.data(), nullptr, seen.data(), R, C, cw, p, g1.data(), c1.data());
  belief_gate_surface(hb.data(), zero.data(), seen.data(), R, C, cw, p, g2.data(), c2.data());
  EXPECT_EQ(g1, g2);
  EXPECT_EQ(c1, c2);
}

TEST(NavBeliefMaterial, DiscMatchesAFullLinspaceScan) {
  const int R = 23, C = 31;
  const double mnx = -3.5, mny = 2.25, mxx = 41.0, mxy = 19.0;
  std::mt19937 rnd(5);
  for (int trial = 0; trial < 40; ++trial) {
    const double px = mnx - 4.0 + (mxx - mnx + 8.0) * (rnd() % 10000) / 10000.0;
    const double py = mny - 4.0 + (mxy - mny + 8.0) * (rnd() % 10000) / 10000.0;
    const double rad = (rnd() % 1500) / 100.0;
    std::vector<std::uint8_t> got(R * C, 0), want(R * C, 0);
    const bool changed = seen_disc(px, py, rad, R, C, mnx, mny, mxx, mxy, got.data());
    // The numpy form: xs = np.linspace(mnx, mxx, C); (xs - px)**2 + (ys - py)**2 <= r*r.
    const double sx = (mxx - mnx) / (C - 1), sy = (mxy - mny) / (R - 1);
    bool any = false;
    for (int r = 0; r < R; ++r)
      for (int c = 0; c < C; ++c) {
        const double x = c == C - 1 ? mxx : c * sx + mnx;
        const double y = r == R - 1 ? mxy : r * sy + mny;
        const double dx = x - px, dy = y - py;
        if (dx * dx + dy * dy <= rad * rad) {
          want[r * C + c] = 1;
          any = true;
        }
      }
    EXPECT_EQ(got, want) << "trial " << trial;
    EXPECT_EQ(changed, any);
    // Marking the same disc again changes nothing.
    EXPECT_FALSE(seen_disc(px, py, rad, R, C, mnx, mny, mxx, mxy, got.data()));
  }
  std::vector<std::uint8_t> s(R * C, 0);
  EXPECT_FALSE(seen_disc(1.0, 1.0, -1.0, R, C, mnx, mny, mxx, mxy, s.data()));
  EXPECT_FALSE(seen_disc(std::nan(""), 1.0, 5.0, R, C, mnx, mny, mxx, mxy, s.data()));
}

TEST(NavBeliefMaterial, EverySeenGivesBackTheTruthPlanes) {
  const Scene sc;
  const int hw = kR * kC;
  std::vector<std::uint8_t> all(hw, 1), hb(hw);
  std::vector<float> rb(hw);
  belief_material_params p;
  p.unknown_risk = 0.9f; // never used: every cell is seen
  p.unknown_hard = 1;
  belief_material_inputs(sc.risk.data(), sc.hard.data(), all.data(), kR, kC, p, rb.data(),
                         hb.data());
  const material_planes a = material_build(rb.data(), hb.data(), kR, kC, kCellW, kScale, kSigma);
  const material_planes b =
      material_build(sc.risk.data(), sc.hard.data(), kR, kC, kCellW, kScale, kSigma);
  EXPECT_EQ(a.stacked(), b.stacked());
}

// ── the sense_batch material-range mask ─────────────────────────────────────────

namespace {
struct SenseRun {
  std::vector<float> lo;
  std::vector<std::uint8_t> lv, es, ms, mchg, fovc;
  std::vector<std::int32_t> ver, flips;
};

SenseRun sense_once(const Scene &sc, const double *pos, const double *head, const double *rng,
                    const double *mrange, bool want_mask, int planes = kM,
                    const int *amap_in = kMap) {
  const int hw = kR * kC;
  SenseRun s{std::vector<float>(planes * hw, 0.0f),     std::vector<std::uint8_t>(planes * hw, 0),
             std::vector<std::uint8_t>(planes * hw, 0), std::vector<std::uint8_t>(planes * hw, 0),
             std::vector<std::uint8_t>(planes, 7),      std::vector<std::uint8_t>(planes, 7),
             std::vector<std::int32_t>(planes, 0),      std::vector<std::int32_t>(kN, 0)};
  std::vector<double> fov(kN, kFov);
  std::vector<std::int32_t> nr(kN, kRays), amap(amap_in, amap_in + kN);
  sense_agents ag;
  ag.pos = pos;
  ag.heading = head;
  ag.range_m = rng;
  ag.fov_rad = fov.data();
  ag.n_rays = nr.data();
  ag.agent_map = amap.data();
  ag.n = kN;
  ag.material_range_m = mrange;
  belief_planes bp{s.lo.data(), s.lv.data(), s.es.data(), s.ver.data(), planes};
  bp.fov_changed = s.fovc.data();
  if (want_mask) {
    bp.material_seen = s.ms.data();
    bp.material_changed = s.mchg.data();
  }
  sense_batch(sc.truth.data(), kR, kC, kMin, kMin, kMax, kMax, ag, nullptr, 0, nullptr, 0, bp,
              kLocc, kLfree, kLclamp, s.flips.data(), 2);
  return s;
}

void tick_pose(int t, std::vector<double> &pos, std::vector<double> &head) {
  pos.resize(2 * kN);
  head.resize(kN);
  for (int i = 0; i < kN; ++i) {
    pos[2 * i] = kPose[t][i][0];
    pos[2 * i + 1] = kPose[t][i][1];
    head[i] = kPose[t][i][2];
  }
}
} // namespace

TEST(NavBeliefMaterial, SenseMaskOffChangesNothing) {
  const Scene sc;
  std::vector<double> pos, head, rng(kN, kRange);
  tick_pose(0, pos, head);
  const SenseRun a = sense_once(sc, pos.data(), head.data(), rng.data(), nullptr, false);
  const SenseRun b = sense_once(sc, pos.data(), head.data(), rng.data(), nullptr, true);
  EXPECT_EQ(a.lo, b.lo);
  EXPECT_EQ(a.lv, b.lv);
  EXPECT_EQ(a.es, b.es);
  EXPECT_EQ(a.ver, b.ver);
  EXPECT_EQ(a.flips, b.flips);
  EXPECT_EQ(a.fovc, b.fovc);
  EXPECT_EQ(a.mchg, std::vector<std::uint8_t>(kM, 7)); // untouched when not asked
}

TEST(NavBeliefMaterial, SenseMaskAtTheSenseRangeIsEverSeen) {
  const Scene sc;
  std::vector<double> pos, head, rng(kN, kRange), big(kN, 1e9);
  tick_pose(1, pos, head);
  for (const double *mr :
       {static_cast<const double *>(nullptr), static_cast<const double *>(big.data())}) {
    const SenseRun s = sense_once(sc, pos.data(), head.data(), rng.data(), mr, true);
    EXPECT_EQ(s.ms, s.es);
    EXPECT_EQ(s.mchg, std::vector<std::uint8_t>(kM, 1));
  }
}

TEST(NavBeliefMaterial, ShorterMaterialRangeIsACastAtThatRange) {
  // On square cells a ray's cells within the shorter range are exactly the cells a cast
  // at that range touches (the march is the same; only the cutoff moves).
  const Scene sc;
  for (int t = 0; t < kTicks; ++t)
    for (double rm : {3.0, 5.5, 7.0, 9.25}) {
      std::vector<double> pos, head, rng(kN, kRange), mr(kN, rm);
      tick_pose(t, pos, head);
      const SenseRun s = sense_once(sc, pos.data(), head.data(), rng.data(), mr.data(), true);
      const SenseRun ref = sense_once(sc, pos.data(), head.data(), mr.data(), nullptr, false);
      EXPECT_EQ(s.ms, ref.es) << "tick " << t << " range " << rm;
    }
}

TEST(NavBeliefMaterial, MaterialChangedFlagsOnlyPlanesThatGrew) {
  const Scene sc;
  const int hw = kR * kC;
  std::vector<double> pos, head, rng(kN, kRange);
  tick_pose(2, pos, head);
  // All agents on plane 0 of two: plane 1 never grows.
  const int amap0[kN] = {0, 0, 0};
  std::vector<float> lo(kM * hw, 0.0f);
  std::vector<std::uint8_t> lv(kM * hw, 0), es(kM * hw, 0), ms(kM * hw, 0), mchg(kM, 9);
  std::vector<std::int32_t> ver(kM, 0), flips(kN, 0), nr(kN, kRays), amap(amap0, amap0 + kN);
  std::vector<double> fov(kN, kFov);
  sense_agents ag;
  ag.pos = pos.data();
  ag.heading = head.data();
  ag.range_m = rng.data();
  ag.fov_rad = fov.data();
  ag.n_rays = nr.data();
  ag.agent_map = amap.data();
  ag.n = kN;
  belief_planes bp{lo.data(), lv.data(), es.data(), ver.data(), kM};
  bp.material_seen = ms.data();
  bp.material_changed = mchg.data();
  sense_batch(sc.truth.data(), kR, kC, kMin, kMin, kMax, kMax, ag, nullptr, 0, nullptr, 0, bp,
              kLocc, kLfree, kLclamp, flips.data(), 1);
  EXPECT_EQ(mchg[0], 1);
  EXPECT_EQ(mchg[1], 0);
  // The same poses again add nothing.
  sense_batch(sc.truth.data(), kR, kC, kMin, kMin, kMax, kMax, ag, nullptr, 0, nullptr, 0, bp,
              kLocc, kLfree, kLclamp, flips.data(), 1);
  EXPECT_EQ(mchg[0], 0);
  EXPECT_EQ(mchg[1], 0);
}

TEST(NavBeliefMaterial, OcclusionHidesMaterialBehindABuilding) {
  // Agent 2 at tick 0 cannot see past building B; the disc rule can. The ray-cast
  // measures from the agent's cell, so it fits inside a disc one cell wider.
  const Scene sc;
  const int hw = kR * kC;
  std::vector<double> pos, head, rng(kN, kRange);
  tick_pose(0, pos, head);
  const SenseRun s = sense_once(sc, pos.data(), head.data(), rng.data(), nullptr, true);
  std::vector<std::uint8_t> disc(hw, 0), wide(hw, 0);
  seen_disc(pos[4], pos[5], kRange, kR, kC, kMin, kMin, kMax, kMax, disc.data());
  seen_disc(pos[4], pos[5], kRange + 1.0, kR, kC, kMin, kMin, kMax, kMax, wide.data());
  int ray_only = 0, disc_only = 0;
  for (int k = 0; k < hw; ++k) {
    ray_only += s.ms[hw + k] && !wide[k];
    disc_only += disc[k] && !s.ms[hw + k];
  }
  EXPECT_EQ(ray_only, 0);
  EXPECT_GT(disc_only, 0);
  // Behind building B from (9.6, 27.1): the cell at (x 5, y 17) is in the disc, not in view.
  EXPECT_EQ(disc[17 * kC + 5], 1);
  EXPECT_EQ(s.ms[hw + 17 * kC + 5], 0);
}

// ── the golden ───────────────────────────────────────────────────────────────

TEST(NavBeliefMaterial, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "belief_material_golden.json";
  const Scene sc;
  const std::size_t hw = static_cast<std::size_t>(kR) * kC;
  const std::vector<Case> cases = golden_cases();
  std::vector<std::vector<TickOut>> runs;
  for (const Case &c : cases)
    runs.push_back(run_case(sc, c.p));

  std::ostringstream js;
  js << "{\n \"about\": \"belief (observed) material, libcvc NavBeliefMaterial.Golden; spec "
        "docs/roadmap/NAV-BELIEF-MATERIAL-ROADMAP.md. Hashes are SHA-256 of each raster's "
        "little-endian bytes (u8 masks, f32 planes), row-major, row 0 at min_y.\",\n";
  js << " \"grid\": {\"rows\": " << kR << ", \"cols\": " << kC << ", \"bounds\": [" << num(kMin)
     << ", " << num(kMin) << ", " << num(kMax) << ", " << num(kMax) << "]},\n";
  js << " \"sense\": {\"range_m\": " << num(kRange) << ", \"n_rays\": " << kRays
     << ", \"fov_rad\": " << num(kFov) << ", \"l_occ\": " << num(kLocc)
     << ", \"l_free\": " << num(kLfree) << ", \"l_clamp\": " << num(kLclamp)
     << ", \"num_threads_any\": true},\n";
  js << " \"occupancy\": {\"policy\": \"optimistic\", \"p_thresh\": " << num(kPthresh)
     << ", \"band\": " << num(kBand) << "},\n";
  js << " \"material_build\": {\"cell_w\": " << num(kCellW) << ", \"scale\": " << num(kScale)
     << ", \"sigma\": " << num(kSigma) << "},\n";
  const gate_params g = golden_gate();
  js << " \"gate\": {\"primitive_count\": " << g.primitive_count
     << ", \"horizon_cells\": " << g.horizon_cells
     << ", \"hard_margin_m\": " << num(g.hard_margin_m)
     << ", \"improvement_margin\": " << num(g.improvement_margin)
     << ", \"material_trigger\": " << num(g.material_trigger)
     << ", \"progress_slack_cells\": " << num(g.progress_slack_cells)
     << ", \"positions\": \"continuous cell (row, col) = ((y - min_y), (x - min_x)) * (n - 1) / "
        "(max - min)\"},\n";
  js << " \"truth\": " << arr(sc.truth.data(), hw) << ",\n";
  js << " \"risk_raw\": " << arr(sc.risk.data(), hw) << ",\n";
  js << " \"hard\": " << arr(sc.hard.data(), hw) << ",\n";
  js << " \"agent_map\": " << arr(kMap, kN) << ",\n";
  js << " \"poses\": [";
  for (int t = 0; t < kTicks; ++t) {
    js << (t ? ", " : "") << "[";
    for (int i = 0; i < kN; ++i)
      js << (i ? ", " : "") << arr(kPose[t][i], 3);
    js << "]";
  }
  js << "],\n \"goals\": [";
  for (int i = 0; i < kN; ++i)
    js << (i ? ", " : "") << arr(kGoal[i], 2);
  js << "],\n \"cases\": [\n";
  for (std::size_t ci = 0; ci < cases.size(); ++ci) {
    const belief_material_params &p = cases[ci].p;
    js << "  {\"name\": \"" << cases[ci].name
       << "\", \"params\": {\"unknown_risk\": " << num(p.unknown_risk)
       << ", \"unknown_hard\": " << int(p.unknown_hard)
       << ", \"unknown_gate_hard\": " << (p.unknown_gate_hard ? 1 : 0) << ", \"seen_rule\": \""
       << (p.rule == seen_rule::raycast ? "raycast" : "disc")
       << "\", \"material_range_m\": " << num(p.material_range_m) << "},\n   \"ticks\": [\n";
    for (int t = 0; t < kTicks; ++t) {
      const TickOut &to = runs[ci][t];
      js << "    {\"planes\": [";
      for (int m = 0; m < kM; ++m) {
        js << (m ? ", " : "") << "{";
        const auto h = plane_hashes(to.pl[m]);
        for (std::size_t k = 0; k < h.size(); ++k)
          js << (k ? ", " : "") << "\"" << h[k].first << "\": \"" << h[k].second << "\"";
        js << "}";
      }
      js << "],\n     \"gate\": {\"active\": " << arr(to.active, kN)
         << ", \"nominal\": " << arr(to.nominal, kN) << ", \"best\": " << arr(to.best, kN)
         << ", \"count\": " << arr(to.count, kN) << "}}" << (t + 1 < kTicks ? "," : "") << "\n";
    }
    js << "   ]}" << (ci + 1 < cases.size() ? "," : "") << "\n";
  }
  js << " ],\n";
  // The defaults' last tick, plane 0, in full: for diffing a mismatch.
  const PlaneOut &full = runs[0][kTicks - 1].pl[0];
  js << " \"full\": {\"case\": \"defaults\", \"tick\": " << kTicks - 1 << ", \"plane\": 0"
     << ",\n  \"material_seen\": " << arr(full.mask.data(), hw)
     << ",\n  \"occ\": " << arr(full.occ.data(), hw)
     << ",\n  \"risk_b\": " << arr(full.risk_b.data(), hw)
     << ",\n  \"hard_b\": " << arr(full.hard_b.data(), hw)
     << ",\n  \"risk\": " << arr(full.planes6.data(), hw)
     << ",\n  \"phi_m\": " << arr(full.planes6.data() + hw, hw)
     << ",\n  \"gate_hard\": " << arr(full.gate_hard.data(), hw)
     << ",\n  \"clear_m\": " << arr(full.clear_m.data(), hw) << "}\n}\n";

  const char *write = std::getenv("CVC_BELIEF_MATERIAL_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::size_t ci = 0;
  for (const auto &c : want.get_child("cases")) {
    ASSERT_LT(ci, cases.size());
    EXPECT_EQ(c.second.get<std::string>("name"), cases[ci].name);
    int t = 0;
    for (const auto &tk : c.second.get_child("ticks")) {
      ASSERT_LT(t, kTicks);
      const TickOut &to = runs[ci][t];
      int m = 0;
      for (const auto &pl : tk.second.get_child("planes")) {
        for (const auto &kv : plane_hashes(to.pl[m]))
          EXPECT_EQ(pl.second.get<std::string>(kv.first), kv.second)
              << cases[ci].name << " tick " << t << " plane " << m << " " << kv.first;
        ++m;
      }
      EXPECT_EQ(m, kM);
      const auto &gt = tk.second.get_child("gate");
      int i = 0;
      for (const auto &v : gt.get_child("active"))
        EXPECT_EQ(v.second.get_value<int>(), to.active[i++]) << cases[ci].name << " tick " << t;
      i = 0;
      for (const auto &v : gt.get_child("count"))
        EXPECT_EQ(v.second.get_value<int>(), to.count[i++]) << cases[ci].name << " tick " << t;
      i = 0;
      for (const auto &v : gt.get_child("nominal")) {
        EXPECT_TRUE(same_double(v.second.data(), to.nominal[i]))
            << cases[ci].name << " tick " << t << ": " << v.second.data() << " vs "
            << to.nominal[i];
        ++i;
      }
      i = 0;
      for (const auto &v : gt.get_child("best")) {
        EXPECT_TRUE(same_double(v.second.data(), to.best[i]))
            << cases[ci].name << " tick " << t << ": " << v.second.data() << " vs " << to.best[i];
        ++i;
      }
      ++t;
    }
    EXPECT_EQ(t, kTicks);
    ++ci;
  }
  EXPECT_EQ(ci, cases.size());
}
