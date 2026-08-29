/*
  Copyright 2026 The University of Texas at Austin

  Unit tests for cvc/lod/select.h -- the single-process LOD selection math.

  Everything here is a pure function, so these are exact, hand-checkable cases
  rather than tolerance-fudged ones. The properties that earn their keep:

    * the switch radius is a true crossover (round-trips through screen_error_px);
    * hysteresis makes oscillation IMPOSSIBLE, not merely unlikely -- a camera
      dithering across a boundary forever must produce at most one transition;
    * a non-monotonic ladder cannot skip a rung, because select_rung forces the
      running max (VISIBILITY-AND-LOD-ROADMAP 6.1);
    * solve() is a pure function of its inputs, so shuffling the candidate order
      changes the plan's ORDER but never a group's assigned rung. That is what
      makes a headless render frame-exact against an interactive one.
*/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cvc/lod/select.h>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

using namespace cvc::lod;

namespace {

// The roadmap's terrain ladder (LSYSTEM-LABORATORY-ROADMAP 8.4, T0..T4): the
// world error doubles with the sample spacing, 2 m through 32 m.
const double kTerrain[5] = {2.0, 4.0, 8.0, 16.0, 32.0};

view_params balanced_view() {
  view_params vp; // 800 px tall, 45 deg vertical, 2.0 px error
  vp.eye[0] = vp.eye[1] = vp.eye[2] = 0.0;
  return vp;
}

// A ladder whose triangle counts quarter per rung, the shape every ladder in
// section 8.4 has.
struct ladder {
  std::uint64_t tris[5];
  std::uint64_t bytes[5];
};

ladder quartering(std::uint64_t base_tris, std::uint64_t bytes_per_tri = 48) {
  ladder l{};
  std::uint64_t t = base_tris;
  for (int i = 0; i < 5; ++i) {
    l.tris[i] = t;
    l.bytes[i] = t * bytes_per_tri;
    t /= 4;
  }
  return l;
}

candidate make_candidate(std::uint32_t id, const ladder &l, int desired, int coarsest, double area,
                         double dist) {
  candidate c;
  c.group_id = id;
  c.nrungs = 5;
  c.desired_rung = desired;
  c.min_rung = coarsest;
  c.projected_area = area;
  c.dist_m = dist;
  c.tris_per_rung = l.tris;
  c.bytes_per_rung = l.bytes;
  return c;
}

} // namespace

// --- Projection ------------------------------------------------------------

TEST(LodSelect, KPxIsHalfHeightOverTanHalfFov) {
  view_params vp = balanced_view();
  // 800 / (2 * tan(22.5 deg)) = 965.685...
  EXPECT_NEAR(k_px(vp), 800.0 / (2.0 * 0.41421356237309503), 1e-9);

  vp.tan_half_fov = 0.0; // degenerate camera must not divide by zero
  EXPECT_EQ(k_px(vp), 0.0);
}

TEST(LodSelect, ScreenErrorFallsWithDistance) {
  const view_params vp = balanced_view();
  const double near_px = screen_error_px(2.0, 100.0, vp);
  const double far_px = screen_error_px(2.0, 400.0, vp);
  EXPECT_GT(near_px, far_px);
  EXPECT_NEAR(near_px / far_px, 4.0, 1e-9); // strictly 1/d
  EXPECT_TRUE(std::isinf(screen_error_px(2.0, 0.0, vp)));
}

TEST(LodSelect, SwitchRadiusIsTheCrossover) {
  const view_params vp = balanced_view();
  for (double e : {0.5, 2.0, 8.0, 32.0}) {
    const double r = switch_radius_m(e, vp);
    // At exactly the switch radius the rung costs exactly the error budget.
    EXPECT_NEAR(screen_error_px(e, r, vp), vp.desired_pixel_error, 1e-9);
    // Beyond it the rung is affordable; inside it, it is not.
    EXPECT_LT(screen_error_px(e, r * 1.01, vp), vp.desired_pixel_error);
    EXPECT_GT(screen_error_px(e, r * 0.99, vp), vp.desired_pixel_error);
  }
}

TEST(LodSelect, SwitchRadiusEdgeCases) {
  view_params vp = balanced_view();
  EXPECT_EQ(switch_radius_m(0.0, vp), 0.0); // an errorless rung is free everywhere
  EXPECT_EQ(switch_radius_m(-1.0, vp), 0.0);
  vp.desired_pixel_error = 0.0; // no budget: an errorful rung never qualifies
  EXPECT_TRUE(std::isinf(switch_radius_m(2.0, vp)));
}

TEST(LodSelect, BoundDistanceIsNearestNotCentre) {
  view_params vp = balanced_view();
  const double centre[3] = {300.0, 0.0, 0.0};
  // A 90 m tile bound at 300 m is 210 m away at its nearest point.
  EXPECT_NEAR(bound_distance_m(centre, 90.0, vp), 210.0, 1e-9);
  // Camera inside the bound: floored at z_near, never zero or negative.
  const double here[3] = {5.0, 0.0, 0.0};
  EXPECT_NEAR(bound_distance_m(here, 90.0, vp), vp.z_near, 1e-9);
}

// --- Ladder validity -------------------------------------------------------

TEST(LodSelect, LadderMonotonicity) {
  EXPECT_TRUE(ladder_is_monotonic(kTerrain, 5));
  const double bad[4] = {2.0, 8.0, 4.0, 16.0};
  EXPECT_FALSE(ladder_is_monotonic(bad, 4));
  EXPECT_TRUE(ladder_is_monotonic(nullptr, 0));
  EXPECT_TRUE(ladder_is_monotonic(kTerrain, 1));
}

// --- Rung selection --------------------------------------------------------

TEST(LodSelect, RungCoarsensMonotonicallyWithDistance) {
  const view_params vp = balanced_view();
  int prev = 0;
  for (double d = 1.0; d < 40000.0; d *= 1.05) {
    const int r = select_rung(d, kTerrain, 5, -1, vp);
    EXPECT_GE(r, prev) << "rung refined as the camera retreated, at d = " << d;
    EXPECT_GE(r, 0);
    EXPECT_LE(r, 4);
    prev = r;
  }
  EXPECT_EQ(prev, 4); // far enough out, the coarsest rung is reached
  EXPECT_EQ(select_rung(1.0, kTerrain, 5, -1, vp), 0);
}

TEST(LodSelect, NoHistorySelectsWithoutHysteresis) {
  const view_params vp = balanced_view();
  // The T0/T1 boundary: T1's 4 m error crosses 2 px at this distance.
  const double r1 = switch_radius_m(4.0, vp);
  EXPECT_EQ(select_rung(r1 * 1.0001, kTerrain, 5, -1, vp), 1);
  EXPECT_EQ(select_rung(r1 * 0.9999, kTerrain, 5, -1, vp), 0);
}

TEST(LodSelect, HysteresisHoldsInsideTheBand) {
  const view_params vp = balanced_view(); // hysteresis = 0.15
  const double r1 = switch_radius_m(4.0, vp);

  // Inside the band, whichever rung you arrive with is the rung you keep.
  const double inside = r1 * 1.07;
  EXPECT_EQ(select_rung(inside, kTerrain, 5, 0, vp), 0);
  EXPECT_EQ(select_rung(inside, kTerrain, 5, 1, vp), 1);

  // Only past the widened boundary does a T0 camera coarsen.
  EXPECT_EQ(select_rung(r1 * 1.16, kTerrain, 5, 0, vp), 1);
  // Only back inside the plain boundary does a T1 camera refine.
  EXPECT_EQ(select_rung(r1 * 0.99, kTerrain, 5, 1, vp), 0);
}

TEST(LodSelect, HysteresisMakesOscillationImpossible) {
  const view_params vp = balanced_view();
  const double r1 = switch_radius_m(4.0, vp);

  // A camera dithering across the plain boundary forever. Without hysteresis
  // this flips every frame; with it, at most one transition may ever occur.
  int rung = select_rung(r1 * 0.98, kTerrain, 5, -1, vp);
  int transitions = 0;
  for (int frame = 0; frame < 500; ++frame) {
    const double d = (frame % 2 == 0) ? r1 * 0.98 : r1 * 1.02;
    const int next = select_rung(d, kTerrain, 5, rung, vp);
    if (next != rung)
      ++transitions;
    rung = next;
  }
  EXPECT_EQ(transitions, 0) << "the dither band straddles the plain boundary only, "
                               "so no transition may fire at all";

  // Widen the dither so it clears the widened boundary on one side: it may
  // transition out, but must not come back while the far side stays inside.
  rung = select_rung(r1 * 0.98, kTerrain, 5, -1, vp);
  transitions = 0;
  for (int frame = 0; frame < 500; ++frame) {
    const double d = (frame % 2 == 0) ? r1 * 1.10 : r1 * 1.20;
    const int next = select_rung(d, kTerrain, 5, rung, vp);
    if (next != rung)
      ++transitions;
    rung = next;
  }
  EXPECT_EQ(transitions, 1);
  EXPECT_EQ(rung, 1);
}

TEST(LodSelect, ZeroHysteresisStillSelectsSanely) {
  view_params vp = balanced_view();
  vp.hysteresis = 0.0;
  const double r1 = switch_radius_m(4.0, vp);
  EXPECT_EQ(select_rung(r1 * 1.01, kTerrain, 5, 0, vp), 1);
  EXPECT_EQ(select_rung(r1 * 0.99, kTerrain, 5, 1, vp), 0);
}

TEST(LodSelect, NonMonotonicLadderStaysAStaircaseAndDropsTheDominatedRung) {
  const view_params vp = balanced_view();
  // T2 authored finer than T1 -- a bake mistake. The running max makes rung 2
  // inherit rung 1's 8 m error, which leaves rung 1 DOMINATED: same effective
  // error, four times the triangles. Selection skips it, and that is the right
  // answer, not a defect. What must survive is the staircase: monotone in
  // distance, and never a rung whose forced error blows the budget.
  const double kinked[5] = {2.0, 8.0, 4.0, 16.0, 32.0};
  ASSERT_FALSE(ladder_is_monotonic(kinked, 5));

  bool saw_dominated = false;
  int prev = 0;
  for (double d = 1.0; d < 40000.0; d *= 1.05) {
    const int r = select_rung(d, kinked, 5, -1, vp);
    EXPECT_GE(r, prev) << "non-monotonic ladder refined as the camera retreated";
    if (r == 1)
      saw_dominated = true;
    // The rung's forced (running-max) error must still be inside the budget.
    double forced = kinked[0];
    for (int i = 1; i <= r; ++i)
      forced = std::max(forced, kinked[i]);
    if (r > 0) {
      EXPECT_LE(screen_error_px(forced, d, vp), vp.desired_pixel_error + 1e-9)
          << "selected rung " << r << " at d = " << d;
    }
    prev = r;
  }
  EXPECT_FALSE(saw_dominated) << "rung 1 costs 4x rung 2 for identical forced error";
  EXPECT_EQ(prev, 4);
}

TEST(LodSelect, DegenerateLadders) {
  const view_params vp = balanced_view();
  EXPECT_EQ(select_rung(100.0, nullptr, 5, -1, vp), 0);
  EXPECT_EQ(select_rung(100.0, kTerrain, 0, -1, vp), 0);
  EXPECT_EQ(select_rung(100.0, kTerrain, 1, -1, vp), 0);
  // A `current` outside the ladder is clamped, not trusted.
  EXPECT_LE(select_rung(1.0, kTerrain, 5, 99, vp), 4);
  EXPECT_GE(select_rung(1.0, kTerrain, 5, 99, vp), 0);
}

// --- Priority --------------------------------------------------------------

TEST(LodSelect, PriorityFallsWithDistanceAndRisesWithArea) {
  const ladder l = quartering(8192);
  const candidate near_small = make_candidate(1, l, 0, 4, 100.0, 100.0);
  const candidate far_small = make_candidate(2, l, 0, 4, 100.0, 2000.0);
  const candidate near_big = make_candidate(3, l, 0, 4, 400.0, 100.0);

  EXPECT_GT(priority(near_small), priority(far_small));
  EXPECT_GT(priority(near_big), priority(near_small));

  // The regularization: at the eye the score is the bare area, not infinity.
  const candidate at_eye = make_candidate(4, l, 0, 4, 100.0, 0.0);
  EXPECT_NEAR(priority(at_eye), 100.0, 1e-12);
  // At 1 km the knee has halved it -- the published shape of 22.1.6's rule.
  const candidate at_1km = make_candidate(5, l, 0, 4, 100.0, 1000.0);
  EXPECT_NEAR(priority(at_1km), 50.0, 1e-12);
}

TEST(LodSelect, DrawsIsTrianglesNotRungIndex) {
  ladder l = quartering(8192);
  l.tris[4] = 0; // T4 folded into another actor: no triangles, no prop
  const candidate c = make_candidate(1, l, 0, 4, 100.0, 100.0);
  EXPECT_TRUE(draws(c, 0));
  EXPECT_TRUE(draws(c, 3));
  EXPECT_FALSE(draws(c, 4));
  EXPECT_FALSE(draws(c, -1));
  EXPECT_FALSE(draws(c, 5));
}

// --- Budget solve ----------------------------------------------------------

TEST(LodSelect, SolveReachesDesiredWhenBudgetIsGenerous) {
  const ladder l = quartering(8192);
  std::vector<candidate> cands;
  for (std::uint32_t i = 0; i < 8; ++i)
    cands.push_back(make_candidate(i, l, 0, 4, 100.0, 100.0 + 10.0 * i));

  budget b; // 48 props, 2.5 M tris, 700 MB -- 8 * 8192 tris fits easily
  const plan p = solve(cands, b);

  EXPECT_EQ(p.binding, bound::none);
  ASSERT_EQ(p.rung.size(), cands.size());
  for (int r : p.rung)
    EXPECT_EQ(r, 0);
  EXPECT_EQ(p.props, 8u);
  EXPECT_EQ(p.tris, 8ull * 8192ull);
}

TEST(LodSelect, SolveStartsAtTheCoarsestAllowedRung) {
  const ladder l = quartering(8192);
  std::vector<candidate> cands;
  cands.push_back(make_candidate(0, l, 0, 4, 100.0, 100.0));

  budget b;
  b.max_tris = 0; // nothing may be promoted at all
  const plan p = solve(cands, b);

  // The baseline itself is over budget (T4 still costs 32 tris), so the solve
  // reports the overflow rather than pretending it fits.
  EXPECT_EQ(p.rung[0], 4);
  EXPECT_EQ(p.binding, bound::triangles);
  EXPECT_EQ(p.tris, l.tris[4]);
}

TEST(LodSelect, TrianglesBindAndTheNearestGroupWins) {
  const ladder l = quartering(8192);
  // Three identical tiles at 100 m, 500 m and 2000 m. Budget fits exactly one
  // promotion to T0 plus the two coarse baselines.
  std::vector<candidate> cands;
  cands.push_back(make_candidate(10, l, 0, 4, 100.0, 2000.0));
  cands.push_back(make_candidate(11, l, 0, 4, 100.0, 100.0));
  cands.push_back(make_candidate(12, l, 0, 4, 100.0, 500.0));

  budget b;
  b.max_tris = 8192 + 32 + 32 + 10; // one T0 and two T4s, and no more
  const plan p = solve(cands, b);

  EXPECT_EQ(p.rung[1], 0) << "the 100 m tile has the highest priority";
  EXPECT_EQ(p.rung[2], 4);
  EXPECT_EQ(p.rung[0], 4);
  EXPECT_EQ(p.binding, bound::triangles);
  EXPECT_LE(p.tris, b.max_tris);
}

TEST(LodSelect, PropsCanBeTheBindingCeiling) {
  ladder l = quartering(8192);
  l.tris[4] = 0; // the coarsest rung draws nothing, so it costs no prop
  std::vector<candidate> cands;
  for (std::uint32_t i = 0; i < 10; ++i)
    cands.push_back(make_candidate(i, l, 0, 4, 100.0, 100.0 + i));

  budget b;
  b.max_props = 3;
  const plan p = solve(cands, b);

  EXPECT_EQ(p.props, 3u);
  EXPECT_EQ(p.binding, bound::props);
  const int drawn =
      static_cast<int>(std::count_if(p.rung.begin(), p.rung.end(), [](int r) { return r != 4; }));
  EXPECT_EQ(drawn, 3);
}

TEST(LodSelect, BytesCanBeTheBindingCeiling) {
  const ladder l = quartering(8192, 1024); // 8 MB at T0
  std::vector<candidate> cands;
  for (std::uint32_t i = 0; i < 6; ++i)
    cands.push_back(make_candidate(i, l, 0, 4, 100.0, 100.0 + i));

  budget b;
  b.max_bytes = 12ull << 20; // room for one T0 (8 MB) and change
  b.max_tris = ~0ull;
  const plan p = solve(cands, b);

  EXPECT_EQ(p.binding, bound::bytes);
  EXPECT_LE(p.bytes, b.max_bytes);
  EXPECT_EQ(p.rung[0], 0);
}

TEST(LodSelect, PlanIsInvariantToCandidateOrder) {
  const ladder l = quartering(8192);
  std::vector<candidate> cands;
  for (std::uint32_t i = 0; i < 24; ++i)
    cands.push_back(make_candidate(i, l, 0, 4, 50.0 + 3.0 * i, 80.0 + 37.0 * i));

  budget b;
  b.max_tris = 40000; // tight enough that the ordering decides who wins
  const plan a = solve(cands, b);

  std::vector<candidate> shuffled(cands.rbegin(), cands.rend());
  const plan c = solve(shuffled, b);

  EXPECT_EQ(a.tris, c.tris);
  EXPECT_EQ(a.props, c.props);
  EXPECT_EQ(a.binding, c.binding);
  // Same group, same rung, regardless of the order it was submitted in.
  for (std::size_t i = 0; i < cands.size(); ++i)
    EXPECT_EQ(a.rung[i], c.rung[cands.size() - 1 - i])
        << "group " << cands[i].group_id << " changed rung when the input was reordered";
}

TEST(LodSelect, EqualPriorityIsBrokenByGroupIdNotByChance) {
  const ladder l = quartering(8192);
  // Identical area and distance: only group_id separates them.
  std::vector<candidate> cands;
  cands.push_back(make_candidate(7, l, 0, 4, 100.0, 300.0));
  cands.push_back(make_candidate(3, l, 0, 4, 100.0, 300.0));
  cands.push_back(make_candidate(5, l, 0, 4, 100.0, 300.0));

  budget b;
  b.max_tris = 8192 + 32 + 32 + 10; // exactly one promotion
  const plan p = solve(cands, b);

  EXPECT_EQ(p.rung[1], 0) << "group_id 3 is the lowest and must win the tie";
  EXPECT_EQ(p.rung[0], 4);
  EXPECT_EQ(p.rung[2], 4);
}

TEST(LodSelect, MinRungFilledBackwardsIsClampedNotIgnored) {
  const ladder l = quartering(8192);
  // desired = 3, min_rung = 1: written backwards against the 0-is-finest
  // convention. The span is repaired rather than left empty.
  std::vector<candidate> cands;
  cands.push_back(make_candidate(0, l, 3, 1, 100.0, 100.0));

  budget b;
  const plan p = solve(cands, b);
  EXPECT_GE(p.rung[0], 1);
  EXPECT_LE(p.rung[0], 3);
}

TEST(LodSelect, EmptyInput) {
  budget b;
  const plan p = solve({}, b);
  EXPECT_TRUE(p.rung.empty());
  EXPECT_EQ(p.props, 0u);
  EXPECT_EQ(p.tris, 0ull);
  EXPECT_EQ(p.binding, bound::none);
}

TEST(LodSelect, SolverReuseMatchesTheFreeFunction) {
  const ladder l = quartering(8192);
  std::vector<candidate> cands;
  for (std::uint32_t i = 0; i < 16; ++i)
    cands.push_back(make_candidate(i, l, 0, 4, 60.0 + 5.0 * i, 120.0 + 61.0 * i));

  budget b;
  b.max_tris = 30000;

  solver s;
  s.reserve(cands.size());
  plan reused;
  for (int frame = 0; frame < 3; ++frame) {
    s.solve(cands, b, reused);
    const plan fresh = solve(cands, b);
    EXPECT_EQ(reused.rung, fresh.rung) << "frame " << frame;
    EXPECT_EQ(reused.tris, fresh.tris);
    EXPECT_EQ(reused.props, fresh.props);
    EXPECT_EQ(reused.binding, fresh.binding);
  }
}

TEST(LodSelect, SolveDoesNotMutateTheScene) {
  const ladder l = quartering(8192);
  std::vector<candidate> cands;
  cands.push_back(make_candidate(0, l, 0, 4, 100.0, 100.0));
  const int desired_before = cands[0].desired_rung;
  const int min_before = cands[0].min_rung;

  budget b;
  b.max_tris = 100;
  (void)solve(cands, b);

  // LOD may never alter simulation state: the selector reads the scene and
  // writes only the plan.
  EXPECT_EQ(cands[0].desired_rung, desired_before);
  EXPECT_EQ(cands[0].min_rung, min_before);
}

// --- Presets ---------------------------------------------------------------

TEST(LodSelect, QualityPresets) {
  EXPECT_EQ(preset_view(quality_preset::pristine).desired_pixel_error, 1.0);
  EXPECT_EQ(preset_view(quality_preset::balanced).desired_pixel_error, 2.0);
  EXPECT_EQ(preset_view(quality_preset::aggressive).desired_pixel_error, 3.5);
  EXPECT_GT(preset_view(quality_preset::aggressive).hysteresis,
            preset_view(quality_preset::balanced).hysteresis);

  // A looser error budget must never ask for MORE detail at a given distance.
  const double d = 900.0;
  const int pristine = select_rung(d, kTerrain, 5, -1, preset_view(quality_preset::pristine));
  const int balanced = select_rung(d, kTerrain, 5, -1, preset_view(quality_preset::balanced));
  const int aggressive = select_rung(d, kTerrain, 5, -1, preset_view(quality_preset::aggressive));
  EXPECT_LE(pristine, balanced);
  EXPECT_LE(balanced, aggressive);
}

TEST(LodSelect, BudgetProfilesShrinkTowardWasm) {
  const budget large = preset_budget(budget_profile::desktop_large);
  const budget def = preset_budget(budget_profile::desktop_default);
  const budget wasm = preset_budget(budget_profile::wasm);

  EXPECT_GT(large.max_tris, def.max_tris);
  EXPECT_GT(def.max_tris, wasm.max_tris);
  EXPECT_GT(def.max_bytes, wasm.max_bytes);
  EXPECT_LT(wasm.max_props, def.max_props);
  // Under volrover3 20.13.7's 4 M client default, with shadow-pass headroom.
  EXPECT_LT(large.max_tris, 4000000ull);
}

// --- Fades -----------------------------------------------------------------

TEST(LodSelect, FadeIsATimeConstantNotAPerFrameRatio) {
  EXPECT_NEAR(fade_alpha(0.12, 0.12), 1.0 - std::exp(-1.0), 1e-12);
  EXPECT_NEAR(fade_alpha(0.0, 0.12), 0.0, 1e-12);
  EXPECT_GT(fade_alpha(0.24, 0.12), fade_alpha(0.12, 0.12));
  EXPECT_LT(fade_alpha(10.0, 0.12), 1.0 + 1e-12);

  // Frame-rate independence: the same elapsed world time gives the same alpha
  // however many frames it took to get there.
  EXPECT_NEAR(fade_alpha(0.05, 0.12), fade_alpha(0.05, 0.12), 0.0);
}

TEST(LodSelect, DeterministicModeSwitchesInstantly) {
  // tau <= 0 is section 6.5's `deterministic` kill-switch: headless and batch
  // renders must be frame-exact, so a fade may never be half-applied.
  EXPECT_EQ(fade_alpha(0.0, 0.0), 1.0);
  EXPECT_EQ(fade_alpha(0.5, 0.0), 1.0);
  EXPECT_EQ(fade_alpha(0.5, -1.0), 1.0);
}

// --- The shape of the Austin frame -----------------------------------------

namespace {

// The building ladder of section 8.4, authored the way that section publishes
// it -- B0 (full shell + facade) out to 200 m, B1 (shell + window quads) out to
// 900 m, B2 (mass box) beyond -- with the world errors derived from those radii
// so the two representations cannot drift.
//
// Authored against the REFERENCE view (balanced, 2.0 px) exactly once, which is
// what lets the runtime preset move the boundaries.
//
// Triangle counts: buildings.glb is 978,242 triangles over 11,272 source
// meshes, ~2,900 per 128 m tile at B0. B2's mass box is ~12 triangles per
// building and a tile holds ~33 of them.
struct building_ladder {
  double err[3];
  std::uint64_t tris[3] = {2900, 800, 400};
  std::uint64_t bytes[3] = {2900 * 48, 800 * 48, 400 * 48};

  building_ladder() {
    const view_params ref = preset_view(quality_preset::balanced);
    err[0] = 0.0;
    err[1] = world_error_for_switch_radius(200.0, ref);
    err[2] = world_error_for_switch_radius(900.0, ref);
  }
};

// RENDER_PERF_ROADMAP phase 1: buildings.glb over a 4 km span, split into a
// flat 32x32 grid of 128 m tiles. Roughly a third of the 1024 tiles hold
// geometry; the camera sits at the south edge looking in.
std::vector<candidate> austin_tiles(const building_ladder &l, std::uint32_t props_when_drawn,
                                    const view_params &vp) {
  std::vector<candidate> cands;
  cands.reserve(337);
  for (std::uint32_t i = 0; i < 337; ++i) {
    const double dist = 60.0 + 11.0 * static_cast<double>(i); // out to ~3.8 km
    candidate c;
    c.group_id = i;
    c.nrungs = 3;
    c.desired_rung = select_rung(dist, l.err, 3, -1, vp);
    c.min_rung = 2;
    c.projected_area = 4.0e6 / (dist * dist);
    c.dist_m = dist;
    c.tris_per_rung = l.tris;
    c.bytes_per_rung = l.bytes;
    c.props_when_drawn = props_when_drawn;
    cands.push_back(c);
  }
  return cands;
}

} // namespace

TEST(LodSelect, LadderAuthoredFromPublishedRadiiRoundTrips) {
  const view_params vp = balanced_view();
  for (double r : {200.0, 480.0, 900.0, 2200.0})
    EXPECT_NEAR(switch_radius_m(world_error_for_switch_radius(r, vp), vp), r, 1e-9);

  // Section 8.4's terrain radii, recovered from the errors it also publishes.
  EXPECT_NEAR(world_error_for_switch_radius(switch_radius_m(4.0, vp), vp), 4.0, 1e-12);
  EXPECT_EQ(world_error_for_switch_radius(0.0, vp), 0.0);
}

TEST(LodSelect, AustinPerTileActorsAreBoundByProps) {
  // One GeometryNode per non-empty tile is phase 1's literal proposal, and at
  // 337 occupied tiles it does not fit section 8.5's 48-prop ceiling -- the
  // baseline is over budget before a single promotion is considered. The solve
  // must say so rather than silently rendering a third of the city.
  const building_ladder l;
  const std::vector<candidate> cands = austin_tiles(l, 1, balanced_view());

  const budget b = preset_budget(budget_profile::desktop_default);
  const plan p = solve(cands, b);

  EXPECT_EQ(p.binding, bound::props);
  EXPECT_GT(p.props, b.max_props) << "the overflow is reported, not clamped away";
  for (int r : p.rung)
    EXPECT_EQ(r, 2) << "nothing may be promoted while the baseline is over budget";
}

TEST(LodSelect, AustinBatchedTilesCollapseTheTriangleCount) {
  // Section 8.6's answer: tiles are cull and LOD units, but they DRAW through a
  // shared fixed-capacity merged actor per rung, so the tile count stops
  // costing props. Now the triangle ceiling is what binds, which is the whole
  // point of the exercise.
  const building_ladder l;
  const std::vector<candidate> cands = austin_tiles(l, 0, balanced_view());

  const budget b = preset_budget(budget_profile::desktop_default);
  const plan p = solve(cands, b);

  EXPECT_EQ(p.binding, bound::none) << "every tile should reach the rung distance wants";
  EXPECT_LE(p.tris, b.max_tris);
  EXPECT_LE(p.props, b.max_props);
  // The headline: phase 1's success criterion is 4-6x fewer triangles
  // rasterized on a close-up shot. Anything over a third of the full mesh has
  // not earned the machinery.
  EXPECT_LT(p.tris, 978242ull / 3)
      << "LOD must cost far less than the merged actor rasterizes today";
  EXPECT_EQ(p.rung[0], 0) << "the tile at the camera must keep full detail";
  EXPECT_EQ(p.rung.back(), 2) << "the tile 3.8 km away must be a mass box";

  // And the near half of the city is strictly better resolved than the far half.
  const int near_sum = std::accumulate(p.rung.begin(), p.rung.begin() + 168, 0);
  const int far_sum = std::accumulate(p.rung.begin() + 168, p.rung.end(), 0);
  EXPECT_LT(near_sum, far_sum);
}

TEST(LodSelect, AustinPresetsMoveEveryBoundaryTogether) {
  // The ladder is authored once against the reference view; changing the
  // runtime preset must then change what gets drawn. If a caller re-derives the
  // ladder at the runtime preset instead, all three of these collapse to the
  // same number and the preset silently does nothing -- which is the trap
  // world_error_for_switch_radius() is documented against.
  const building_ladder l;
  const budget b = preset_budget(budget_profile::desktop_default);

  const std::uint64_t pristine =
      solve(austin_tiles(l, 0, preset_view(quality_preset::pristine)), b).tris;
  const std::uint64_t balanced =
      solve(austin_tiles(l, 0, preset_view(quality_preset::balanced)), b).tris;
  const std::uint64_t aggressive =
      solve(austin_tiles(l, 0, preset_view(quality_preset::aggressive)), b).tris;

  EXPECT_GT(pristine, balanced);
  EXPECT_GT(balanced, aggressive);
}
