/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Unit tests for cvc::tf's overlay layers (cvc/tf/overlays.h): the state-string codecs, the
// request bits, components_at, compute_overlays on small synthetic volumes, and the
// publish / read round trip through a cvc::app's state.

#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/tf/overlays.h>
#include <cvc/volume/volume.h>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::tf;

namespace {

double sum(const std::vector<double> &v) {
  double s = 0.0;
  for (const double x : v)
    s += x;
  return s;
}

bool contains(const std::string &s, const std::string &what) {
  return s.find(what) != std::string::npos;
}

cvc::volume make_volume(cvc::app &ctx, cvc::uint64 nx, cvc::uint64 ny, cvc::uint64 nz,
                        const std::function<double(double, double, double)> &f) {
  cvc::volume vol(ctx, cvc::dimension(nx, ny, nz), cvc::Float);
  for (cvc::uint64 k = 0; k < nz; ++k)
    for (cvc::uint64 j = 0; j < ny; ++j)
      for (cvc::uint64 i = 0; i < nx; ++i)
        vol(i, j, k, f(double(i), double(j), double(k)));
  vol.unsetMinMax();
  return vol;
}

// Two maxima (1.0 at x = 4, 0.9 at x = 12) whose level sets merge near 0.26.
double two_blobs(double x, double y, double z) {
  auto g = [](double dx, double dy, double dz) {
    return std::exp(-(dx * dx + dy * dy + dz * dz) / 8.0);
  };
  return g(x - 4, y - 8, z - 8) + 0.9 * g(x - 12, y - 8, z - 8);
}

// Two maxima (10 and 8) joining at a saddle (5) above a minimum (0).
tree_overlay two_peaks() {
  tree_overlay t;
  t.vertices = {{0.5f, 10.0}, {0.2f, 8.0}, {0.8f, 5.0}, {0.5f, 0.0}};
  t.edges = {{0, 2}, {1, 2}, {2, 3}};
  return t;
}

spectrum_overlay small_spectrum() {
  spectrum_overlay s;
  s.area = {0.0, 1.5, 0.1};
  s.min_volume = {0.0, 0.25, 1.0};
  s.max_volume = {1.0, 0.75, 0.0};
  s.gradient = {1e-12, 3.0e7, 2.0 / 3.0};
  return s;
}

} // namespace

// --- codecs -------------------------------------------------------------------------------

TEST(TfOverlaysCodec, RangeRoundTripsAndRejectsMalformed) {
  double lo = 0.0, hi = 0.0;
  EXPECT_EQ(range_to_string(0.5, 1.5), "0.5,1.5");
  ASSERT_TRUE(range_from_string(range_to_string(0.1, 12345.678901234567), lo, hi));
  EXPECT_EQ(lo, 0.1);
  EXPECT_EQ(hi, 12345.678901234567);
  ASSERT_TRUE(range_from_string(range_to_string(-1e-300, 1e300), lo, hi));
  EXPECT_EQ(lo, -1e-300);
  EXPECT_EQ(hi, 1e300);
  ASSERT_TRUE(range_from_string(" 1.5 ,\t2.5 \n", lo, hi));
  EXPECT_EQ(lo, 1.5);
  EXPECT_EQ(hi, 2.5);
  ASSERT_TRUE(range_from_string("[3, 4]", lo, hi));
  EXPECT_EQ(lo, 3.0);
  EXPECT_EQ(hi, 4.0);
  EXPECT_TRUE(range_from_string("5,5", lo, hi)); // a constant volume's range

  for (const char *bad : {"", "  ", "1", "1,2,3", "a,b", "2,1", "1,", ",2", "1 2", "[1,2", "nan,1",
                          "1,1e999", "{\"lo\":1}", "1,2x"}) {
    lo = -7.0;
    hi = -8.0;
    EXPECT_FALSE(range_from_string(bad, lo, hi)) << bad;
    EXPECT_EQ(lo, -7.0) << bad; // untouched
    EXPECT_EQ(hi, -8.0) << bad;
  }
}

TEST(TfOverlaysCodec, HistogramRoundTripsAndRejectsMalformed) {
  const std::vector<double> bins = {0, 1, 2.5, 1e10, 123456789, 0.1};
  std::vector<double> back;
  ASSERT_TRUE(histogram_from_string(histogram_to_string(bins), back));
  EXPECT_EQ(back, bins);
  EXPECT_EQ(histogram_to_string({3, 0, 7}), "[3,0,7]");
  EXPECT_TRUE(histogram_from_string(histogram_to_string({}), back));
  EXPECT_TRUE(back.empty());
  ASSERT_TRUE(histogram_from_string(" [ 1 ,\n 2 ]\t", back));
  EXPECT_EQ(back, (std::vector<double>{1, 2}));

  for (const char *bad :
       {"", "[1,-1]", "[1,\"a\"]", "{}", "[1,", "[[1]]", "[1,null]", "1,2", "[1] x"}) {
    back = {9.0};
    EXPECT_FALSE(histogram_from_string(bad, back)) << bad;
    EXPECT_TRUE(back.empty()) << bad;
  }
}

TEST(TfOverlaysCodec, SpectrumRoundTripsAndRejectsMalformed) {
  const spectrum_overlay s = small_spectrum();
  spectrum_overlay back;
  ASSERT_TRUE(spectrum_from_string(spectrum_to_string(s), back));
  EXPECT_EQ(back.area, s.area);
  EXPECT_EQ(back.min_volume, s.min_volume);
  EXPECT_EQ(back.max_volume, s.max_volume);
  EXPECT_EQ(back.gradient, s.gradient);
  EXPECT_EQ(back.size(), 3u);

  // Whitespace, key order and extra members are fine.
  ASSERT_TRUE(spectrum_from_string(" { \"gradient\" : [4], \"isovalue\": [0], \"max_volume\":[3],"
                                   "\n\"min_volume\":[2], \"area\":[1] } ",
                                   back));
  EXPECT_EQ(back.area, std::vector<double>{1});
  EXPECT_EQ(back.gradient, std::vector<double>{4});

  for (const char *bad :
       {"", "[]", "{}", "{\"area\":[1],\"min_volume\":[1],\"max_volume\":[1]}",
        "{\"area\":[1,2],\"min_volume\":[1],\"max_volume\":[1],\"gradient\":[1]}",
        "{\"area\":[1,\"x\"],\"min_volume\":[1,2],\"max_volume\":[1,2],\"gradient\":[1,2]}",
        "{\"area\":1,\"min_volume\":1,\"max_volume\":1,\"gradient\":1}"}) {
    back = s;
    EXPECT_FALSE(spectrum_from_string(bad, back)) << bad;
    EXPECT_TRUE(back.empty()) << bad;
    EXPECT_TRUE(back.gradient.empty()) << bad;
  }
}

TEST(TfOverlaysCodec, TreeRoundTripsAndRejectsMalformed) {
  tree_overlay t;
  t.vertices = {{0.25f, -3.5}, {0.123456789f, 1e-7}, {1.f, 42.0 / 9.0}};
  t.edges = {{0, 1}, {1, 2}};
  tree_overlay back;
  ASSERT_TRUE(tree_from_string(tree_to_string(t), back));
  ASSERT_EQ(back.vertices.size(), 3u);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(back.vertices[i].x, t.vertices[i].x);
    EXPECT_EQ(back.vertices[i].value, t.vertices[i].value);
  }
  EXPECT_EQ(back.edges, t.edges);

  ASSERT_TRUE(tree_from_string(" {\"edges\": [ [1, 0] ],\n \"vertices\": [[0,1],[1,2]]}", back));
  EXPECT_EQ(back.vertices.size(), 2u);
  ASSERT_EQ(back.edges.size(), 1u);
  EXPECT_EQ(back.edges[0], std::make_pair(1, 0));
  EXPECT_TRUE(tree_from_string("{\"vertices\":[],\"edges\":[]}", back));
  EXPECT_TRUE(back.empty());

  for (const char *bad :
       {"", "[]", "{\"vertices\":[[0,1]]}", "{\"edges\":[]}",
        "{\"vertices\":[[0,1],[1,2]],\"edges\":[[0,2]]}",
        "{\"vertices\":[[0,1],[1,2]],\"edges\":[[-1,0]]}",
        "{\"vertices\":[[0,1],[1,2]],\"edges\":[[0.5,1]]}",
        "{\"vertices\":[[0,1],[1,2]],\"edges\":[[0,1,1]]}",
        "{\"vertices\":[[0],[1,2]],\"edges\":[[0,1]]}", "{\"vertices\":[0,1],\"edges\":[[0,1]]}"}) {
    back = t;
    EXPECT_FALSE(tree_from_string(bad, back)) << bad;
    EXPECT_TRUE(back.vertices.empty()) << bad;
    EXPECT_TRUE(back.edges.empty()) << bad;
  }
}

TEST(TfOverlaysCodec, RequestBitsCsv) {
  EXPECT_EQ(overlay_bits_from_csv(""), 0u);
  EXPECT_EQ(overlay_bits_from_csv("histogram"), unsigned(overlay_histogram));
  EXPECT_EQ(overlay_bits_from_csv("contour_spectrum,contour_tree"),
            unsigned(overlay_contour_spectrum | overlay_contour_tree));
  EXPECT_EQ(overlay_bits_from_csv(" histogram , contour_tree "),
            unsigned(overlay_histogram | overlay_contour_tree));
  EXPECT_EQ(overlay_bits_from_csv("bogus,contour_tree,,histogram"),
            unsigned(overlay_histogram | overlay_contour_tree));
  EXPECT_EQ(overlay_bits_to_csv(0), "");
  EXPECT_EQ(overlay_bits_to_csv(overlay_all), "histogram,contour_spectrum,contour_tree");
  EXPECT_EQ(overlay_bits_to_csv(overlay_contour_tree), "contour_tree");
  for (unsigned b = 0; b <= overlay_all; ++b)
    EXPECT_EQ(overlay_bits_from_csv(overlay_bits_to_csv(b)), b);
}

// --- components_at ------------------------------------------------------------------------

TEST(TfOverlaysTree, ComponentsAtCountsSpanningArcs) {
  tree_overlay t = two_peaks();
  EXPECT_EQ(t.components_at(9.0), 1);  // only the higher peak
  EXPECT_EQ(t.components_at(8.0), 1);  // the lower peak's arc ends here (half-open)
  EXPECT_EQ(t.components_at(7.0), 2);  // both peaks
  EXPECT_EQ(t.components_at(5.0), 2);  // the saddle belongs to the arcs above it
  EXPECT_EQ(t.components_at(3.0), 1);  // merged
  EXPECT_EQ(t.components_at(0.0), 1);  // the minimum
  EXPECT_EQ(t.components_at(10.0), 1); // the global maximum: the arcs ending there
  EXPECT_EQ(t.components_at(-1.0), 0);
  EXPECT_EQ(t.components_at(11.0), 0);
  EXPECT_EQ(t.components_at(std::numeric_limits<double>::quiet_NaN()), 0);

  for (auto &e : t.edges) // orientation does not matter
    std::swap(e.first, e.second);
  EXPECT_EQ(t.components_at(7.0), 2);
  EXPECT_EQ(t.components_at(3.0), 1);

  t.edges.emplace_back(0, 99); // a bad edge is ignored
  EXPECT_EQ(t.components_at(7.0), 2);
  EXPECT_EQ(tree_overlay().components_at(1.0), 0);
}

// --- compute / publish (need an app) -------------------------------------------------------

class TfOverlaysVolume : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(TfOverlaysVolume, ComputeOnARamp) {
  const cvc::volume vol =
      make_volume(ctx, 8, 8, 8, [](double x, double y, double z) { return x + y + z; });
  overlay_options opt;
  opt.spectrum_samples = 32;
  const overlays ov = compute_overlays(vol, overlay_all, opt);
  EXPECT_EQ(ov.status, "");
  ASSERT_TRUE(ov.has_range);
  EXPECT_DOUBLE_EQ(ov.range_lo, 0.0);
  EXPECT_DOUBLE_EQ(ov.range_hi, 21.0);

  ASSERT_EQ(ov.histogram.size(), 256u);
  EXPECT_DOUBLE_EQ(sum(ov.histogram), 512.0);  // every voxel binned
  EXPECT_DOUBLE_EQ(ov.histogram.front(), 1.0); // only the corner at 0
  EXPECT_DOUBLE_EQ(ov.histogram.back(), 1.0);  // only the corner at 21

  ASSERT_EQ(ov.spectrum.size(), 32u);
  EXPECT_EQ(ov.spectrum.min_volume.size(), 32u);
  EXPECT_EQ(ov.spectrum.max_volume.size(), 32u);
  EXPECT_EQ(ov.spectrum.gradient.size(), 32u);
  EXPECT_LT(ov.spectrum.min_volume.front(), ov.spectrum.min_volume.back());

  ASSERT_FALSE(ov.tree.empty());
  const int nv = static_cast<int>(ov.tree.vertices.size());
  for (const auto &v : ov.tree.vertices) {
    EXPECT_GE(v.value, 0.0);
    EXPECT_LE(v.value, 21.0);
    EXPECT_GE(v.x, 0.f);
    EXPECT_LE(v.x, 1.f);
  }
  for (const auto &e : ov.tree.edges) {
    EXPECT_GE(e.first, 0);
    EXPECT_LT(e.first, nv);
    EXPECT_GE(e.second, 0);
    EXPECT_LT(e.second, nv);
  }
  EXPECT_EQ(ov.tree.components_at(10.0), 1); // a ramp's level sets are connected

  // A coarser resampling still yields both costly layers.
  opt.max_dim = 4;
  const overlays coarse =
      compute_overlays(vol, overlay_contour_spectrum | overlay_contour_tree, opt);
  EXPECT_EQ(coarse.status, "");
  EXPECT_TRUE(coarse.histogram.empty()); // not requested
  EXPECT_EQ(coarse.spectrum.size(), 32u);
  EXPECT_FALSE(coarse.tree.empty());
}

TEST_F(TfOverlaysVolume, ComputeOnlyWhatIsRequested) {
  const cvc::volume vol =
      make_volume(ctx, 6, 5, 4, [](double x, double y, double z) { return x * y - z; });
  overlay_options opt;
  opt.histogram_bins = 16;
  overlays ov = compute_overlays(vol, overlay_histogram, opt);
  EXPECT_TRUE(ov.has_range);
  ASSERT_EQ(ov.histogram.size(), 16u);
  EXPECT_DOUBLE_EQ(sum(ov.histogram), 120.0);
  EXPECT_TRUE(ov.spectrum.empty());
  EXPECT_TRUE(ov.tree.empty());
  EXPECT_EQ(ov.status, "");

  ov = compute_overlays(vol, 0u, opt);
  EXPECT_TRUE(ov.has_range);
  EXPECT_DOUBLE_EQ(ov.range_lo, -3.0);
  EXPECT_DOUBLE_EQ(ov.range_hi, 20.0);
  EXPECT_TRUE(ov.histogram.empty());
}

TEST_F(TfOverlaysVolume, ContourTreeOfTwoBlobsSeesBothPeaks) {
  const cvc::volume vol = make_volume(ctx, 16, 16, 16, two_blobs);
  const overlays ov = compute_overlays(vol, overlay_contour_tree);
  EXPECT_EQ(ov.status, "");
  ASSERT_FALSE(ov.tree.empty());
  EXPECT_EQ(ov.tree.components_at(0.6), 2);
  EXPECT_EQ(ov.tree.components_at(0.95), 1);
  EXPECT_LE(ov.tree.vertices.size(), 1500u);
}

TEST_F(TfOverlaysVolume, DegenerateVolumesReportStatusAndNeverThrow) {
  // Constant: the range collapses, the histogram is one bin, there is no tree.
  const cvc::volume flat = make_volume(ctx, 4, 4, 4, [](double, double, double) { return 3.0; });
  overlays ov = compute_overlays(flat);
  ASSERT_TRUE(ov.has_range);
  EXPECT_DOUBLE_EQ(ov.range_lo, 3.0);
  EXPECT_DOUBLE_EQ(ov.range_hi, 3.0);
  ASSERT_EQ(ov.histogram.size(), 256u);
  EXPECT_DOUBLE_EQ(sum(ov.histogram), 64.0);
  EXPECT_DOUBLE_EQ(ov.histogram[128], 64.0); // voxels::histogram's middle bin
  EXPECT_TRUE(ov.tree.empty());
  EXPECT_TRUE(contains(ov.status, "unavailable"));
  EXPECT_TRUE(contains(ov.status, "contour tree"));

  // A single slice has no cells for the spectrum.
  const cvc::volume slab =
      make_volume(ctx, 6, 6, 1, [](double x, double y, double) { return x - y; });
  ov = compute_overlays(slab);
  EXPECT_TRUE(ov.has_range);
  EXPECT_DOUBLE_EQ(sum(ov.histogram), 36.0);
  EXPECT_TRUE(ov.spectrum.empty());
  EXPECT_TRUE(contains(ov.status, "contour spectrum"));

  // A NaN voxel is left out of the histogram, and the tree refuses it.
  cvc::volume holes =
      make_volume(ctx, 4, 4, 4, [](double x, double y, double z) { return x + 2 * y + 4 * z; });
  holes(1, 2, 3, std::numeric_limits<double>::quiet_NaN());
  holes.unsetMinMax();
  ov = compute_overlays(holes, overlay_histogram | overlay_contour_tree);
  EXPECT_TRUE(ov.has_range);
  EXPECT_DOUBLE_EQ(sum(ov.histogram), 63.0);
  EXPECT_TRUE(ov.tree.empty());
  EXPECT_TRUE(contains(ov.status, "contour tree"));
}

TEST_F(TfOverlaysVolume, PublishAndReadRoundTrip) {
  cvc::state &tf = cvc::state::instance(ctx)("tf_overlays_test.publish.transfer_function");
  overlays ov;
  ov.has_range = true;
  ov.range_lo = -1.25;
  ov.range_hi = 1e3 / 3.0;
  ov.histogram = {5, 0, 2, 9};
  ov.spectrum = small_spectrum();
  ov.tree = two_peaks();
  publish_overlays(tf, ov);
  EXPECT_EQ(tf("range").value(), range_to_string(ov.range_lo, ov.range_hi));
  EXPECT_EQ(tf("histogram").value(), histogram_to_string(ov.histogram));
  EXPECT_EQ(tf("overlay_status").value(), "");

  overlays back = read_overlays(tf);
  ASSERT_TRUE(back.has_range);
  EXPECT_EQ(back.range_lo, ov.range_lo);
  EXPECT_EQ(back.range_hi, ov.range_hi);
  EXPECT_EQ(back.histogram, ov.histogram);
  EXPECT_EQ(back.spectrum.area, ov.spectrum.area);
  EXPECT_EQ(back.spectrum.gradient, ov.spectrum.gradient);
  ASSERT_EQ(back.tree.vertices.size(), ov.tree.vertices.size());
  EXPECT_EQ(back.tree.edges, ov.tree.edges);
  EXPECT_EQ(back.status, "");

  // A later publish of one layer keeps the others (empty members are not written).
  overlays later;
  later.tree.vertices = {{0.f, 1.0}, {1.f, 2.0}};
  later.tree.edges = {{1, 0}};
  later.status = "computing contour spectrum";
  publish_overlays(tf, later);
  back = read_overlays(tf);
  EXPECT_TRUE(back.has_range);
  EXPECT_EQ(back.histogram, ov.histogram);
  EXPECT_EQ(back.spectrum.area, ov.spectrum.area);
  EXPECT_EQ(back.tree.edges, later.tree.edges);
  EXPECT_EQ(back.status, "computing contour spectrum");
  publish_overlays(tf, overlays()); // idle again
  EXPECT_EQ(read_overlays(tf).status, "");

  // A computed result survives the trip too.
  const cvc::volume vol =
      make_volume(ctx, 6, 6, 6, [](double x, double y, double z) { return x * x + y - z; });
  overlay_options opt;
  opt.spectrum_samples = 16;
  const overlays computed = compute_overlays(vol, overlay_all, opt);
  cvc::state &tf2 = cvc::state::instance(ctx)("tf_overlays_test.computed.transfer_function");
  publish_overlays(tf2, computed);
  back = read_overlays(tf2);
  EXPECT_EQ(back.range_hi, computed.range_hi);
  EXPECT_EQ(back.histogram, computed.histogram);
  EXPECT_EQ(back.spectrum.min_volume, computed.spectrum.min_volume);
  ASSERT_EQ(back.tree.vertices.size(), computed.tree.vertices.size());
  for (std::size_t i = 0; i < back.tree.vertices.size(); ++i) {
    EXPECT_EQ(back.tree.vertices[i].x, computed.tree.vertices[i].x);
    EXPECT_EQ(back.tree.vertices[i].value, computed.tree.vertices[i].value);
  }
  EXPECT_EQ(back.status, computed.status);
}

// --- overlay_job: compute_overlays a slice at a time ----------------------------------------

namespace {

// A deterministic hash in [0, 1).
double hash01(double x, double y, double z) {
  const double s = std::sin(x * 12.9898 + y * 78.233 + z * 37.719) * 43758.5453;
  return s - std::floor(s);
}

// Two blobs under noise: many small extrema, so the contour tree needs simplifying.
double noisy_blobs(double x, double y, double z) {
  return two_blobs(x, y, z) + 0.08 * hash01(x, y, z);
}

void expect_same(const overlays &a, const overlays &b, const std::string &what) {
  EXPECT_EQ(a.has_range, b.has_range) << what;
  EXPECT_EQ(a.range_lo, b.range_lo) << what;
  EXPECT_EQ(a.range_hi, b.range_hi) << what;
  EXPECT_EQ(a.histogram, b.histogram) << what;
  EXPECT_EQ(a.spectrum.area, b.spectrum.area) << what;
  EXPECT_EQ(a.spectrum.min_volume, b.spectrum.min_volume) << what;
  EXPECT_EQ(a.spectrum.max_volume, b.spectrum.max_volume) << what;
  EXPECT_EQ(a.spectrum.gradient, b.spectrum.gradient) << what;
  ASSERT_EQ(a.tree.vertices.size(), b.tree.vertices.size()) << what;
  for (std::size_t i = 0; i < a.tree.vertices.size(); ++i) {
    EXPECT_EQ(a.tree.vertices[i].x, b.tree.vertices[i].x) << what << " vertex " << i;
    EXPECT_EQ(a.tree.vertices[i].value, b.tree.vertices[i].value) << what << " vertex " << i;
  }
  EXPECT_EQ(a.tree.edges, b.tree.edges) << what;
  EXPECT_EQ(a.status, b.status) << what;
}

// Step a job to the end with budgets from `next_budget`; the number of steps it took.
std::size_t run_job(overlay_job &job, const std::function<std::size_t()> &next_budget) {
  std::size_t steps = 1;
  while (!job.step(next_budget())) {
    ++steps;
    if (steps > 50000000u) {
      ADD_FAILURE() << "the job never finished";
      break;
    }
  }
  return steps;
}

// The coarse group a phase() belongs to.
std::string phase_group(const std::string &phase) {
  if (phase == "contour tree: simplifying" || phase == "contour tree: laying out")
    return phase;
  const std::size_t colon = phase.find(':');
  return colon == std::string::npos ? phase : phase.substr(0, colon);
}

} // namespace

TEST_F(TfOverlaysVolume, JobMatchesComputeForEveryLayerSubsetAndBudget) {
  // Non-cubic, resampled (max_dim 10 caps two axes), noisy enough to need simplifying.
  cvc::volume vol = make_volume(ctx, 13, 11, 9, noisy_blobs);
  overlay_options opt;
  opt.histogram_bins = 32;
  opt.spectrum_samples = 24;
  opt.max_dim = 10;
  opt.tree_max_nodes = 12;
  std::mt19937 rng(20261025u);
  std::uniform_int_distribution<std::size_t> any_budget(1, 3000);
  for (unsigned bits = 0; bits <= overlay_all; ++bits) {
    const overlays ref = compute_overlays(vol, bits, opt);
    EXPECT_FALSE(vol.minIsSet()) << "compute_overlays wrote the caller's min/max cache";
    for (const std::size_t budget :
         {std::size_t(1), std::size_t(97), std::size_t(4096), std::size_t(0), std::size_t(-1)}) {
      overlay_job job(vol, bits, opt);
      const bool random = budget == std::size_t(-1);
      const std::size_t steps = run_job(job, [&] { return random ? any_budget(rng) : budget; });
      const std::string what = "bits " + std::to_string(bits) + " budget " +
                               (random ? std::string("random") : std::to_string(budget));
      EXPECT_TRUE(job.done()) << what;
      EXPECT_FALSE(job.cancelled()) << what;
      EXPECT_EQ(job.progress(), 1.0) << what;
      EXPECT_STREQ(job.phase(), "done") << what;
      EXPECT_EQ(job.status(), ref.status) << what;
      if (budget == 0)
        EXPECT_EQ(steps, 1u) << what;
      overlays out = job.take();
      expect_same(out, ref, what);
      EXPECT_FALSE(vol.minIsSet()) << what << ": the job wrote the caller's min/max cache";
    }
  }
  EXPECT_EQ(overlay_job::instances(), 0u);
}

TEST_F(TfOverlaysVolume, JobSlicesEveryPhaseAndReportsProgress) {
  cvc::volume vol = make_volume(ctx, 16, 16, 16, noisy_blobs);
  overlay_options opt;
  opt.spectrum_samples = 32;
  opt.tree_max_nodes = 8;
  overlay_job job(vol, overlay_all, opt);
  EXPECT_EQ(overlay_job::instances(), 1u);
  EXPECT_FALSE(job.done());
  EXPECT_EQ(job.bits(), unsigned(overlay_all));
  EXPECT_EQ(job.progress(), 0.0);

  std::map<std::string, int> steps_in; // steps taken per phase group
  std::vector<std::string> order;      // the groups, in the order they ran
  double last = 0.0;
  bool done = false;
  while (!done) {
    const std::string phase = job.phase();
    const std::string group = phase_group(phase);
    if (order.empty() || order.back() != group)
      order.push_back(group);
    ++steps_in[group];
    const std::string status = job.status();
    EXPECT_EQ(status.compare(0, phase.size(), phase), 0) << status;
    if (group != "contour tree: laying out")
      EXPECT_EQ(status.back(), '%') << status;
    // Each layer appears in result() only once complete.
    if (group == "range")
      EXPECT_FALSE(job.result().has_range);
    if (group == "histogram")
      EXPECT_TRUE(job.result().histogram.empty());
    if (group == "contour spectrum") {
      EXPECT_FALSE(job.result().histogram.empty());
      EXPECT_TRUE(job.result().spectrum.empty());
    }
    if (group == "contour tree" || group == "contour tree: simplifying") {
      EXPECT_FALSE(job.result().spectrum.empty());
      EXPECT_TRUE(job.result().tree.empty());
    }
    done = job.step(97);
    const double p = job.progress();
    EXPECT_GE(p, last);
    EXPECT_LE(p, 1.0);
    last = p;
  }
  EXPECT_EQ(last, 1.0);
  const std::vector<std::string> expected = {"range",
                                             "histogram",
                                             "contour spectrum",
                                             "contour tree",
                                             "contour tree: simplifying",
                                             "contour tree: laying out"};
  EXPECT_EQ(order, expected);
  for (const std::string &g : expected)
    if (g != "contour tree: laying out") // one-shot by design
      EXPECT_GT(steps_in[g], 1) << g << " took a single step";
  std::printf("[overlay_job] steps at budget 97: range %d, histogram %d, spectrum %d, tree %d, "
              "simplify %d\n",
              steps_in["range"], steps_in["histogram"], steps_in["contour spectrum"],
              steps_in["contour tree"], steps_in["contour tree: simplifying"]);
  EXPECT_EQ(job.status(), "");
  const overlays out = job.take();
  expect_same(out, compute_overlays(vol, overlay_all, opt), "budget 97");
  EXPECT_LE(out.tree.vertices.size(), 8u);
}

TEST_F(TfOverlaysVolume, JobCancelStopsAtTheNextCheck) {
  cvc::volume vol = make_volume(ctx, 16, 16, 16, noisy_blobs);
  overlay_options opt;
  opt.spectrum_samples = 32;

  // Mid-way: what is complete stays, nothing more is computed.
  overlay_job job(vol, overlay_all, opt);
  while (phase_group(job.phase()) != "contour spectrum")
    ASSERT_FALSE(job.step(500));
  ASSERT_FALSE(job.step(500));
  job.cancel();
  EXPECT_TRUE(job.cancelled());
  EXPECT_TRUE(job.done());
  EXPECT_STREQ(job.phase(), "cancelled");
  EXPECT_EQ(job.status(), "cancelled");
  const double frozen = job.progress();
  EXPECT_GT(frozen, 0.0);
  EXPECT_LT(frozen, 1.0);
  EXPECT_TRUE(job.step(500));
  EXPECT_TRUE(job.step(0));
  EXPECT_EQ(job.progress(), frozen);
  EXPECT_TRUE(job.result().has_range);
  EXPECT_FALSE(job.result().histogram.empty());
  EXPECT_TRUE(job.result().spectrum.empty());
  EXPECT_TRUE(job.result().tree.empty());

  // Before the first step.
  overlay_job early(vol, overlay_all, opt);
  early.cancel();
  EXPECT_TRUE(early.step(1));
  EXPECT_FALSE(early.result().has_range);
  EXPECT_TRUE(early.result().histogram.empty());
}

TEST_F(TfOverlaysVolume, JobRangeIsVolumeMinMaxForEveryVoxelType) {
  const cvc::data_type types[] = {cvc::UChar,  cvc::UShort, cvc::UInt, cvc::Float, cvc::Double,
                                  cvc::UInt64, cvc::Char,   cvc::Int,  cvc::Int64};
  for (const cvc::data_type type : types) {
    cvc::volume vol(ctx, cvc::dimension(7, 5, 3), type);
    for (cvc::uint64 i = 0; i < 105; ++i) {
      const double v = static_cast<double>((i * 37) % 101) + 3.0;
      const bool is_signed = type == cvc::Char || type == cvc::Int || type == cvc::Int64 ||
                             type == cvc::Float || type == cvc::Double;
      vol(i, is_signed ? v - 60.0 : v);
    }
    if (type == cvc::UInt64) // voxel 0 above 2^53, where a double no longer holds every integer
      reinterpret_cast<cvc::uint64 *>(*vol)[0] = (cvc::uint64(1) << 53) + 1;
    vol.unsetMinMax();
    cvc::volume probe(vol); // what vol.min()/max() would say, without writing vol's cache
    const double lo = probe.min(), hi = probe.max();
    overlay_job job(vol, overlay_histogram);
    run_job(job, [] { return std::size_t(5); });
    const overlays &ov = job.result();
    ASSERT_TRUE(ov.has_range) << "type " << int(type);
    EXPECT_EQ(ov.range_lo, lo) << "type " << int(type);
    EXPECT_EQ(ov.range_hi, hi) << "type " << int(type);
    EXPECT_FALSE(vol.minIsSet()) << "type " << int(type);
    EXPECT_FALSE(vol.maxIsSet()) << "type " << int(type);
    expect_same(ov, compute_overlays(vol, overlay_histogram), "type " + std::to_string(type));
  }

  // A set min/max (narrower than the data, as a file header may give) is used as it is.
  cvc::volume ramp =
      make_volume(ctx, 8, 8, 8, [](double x, double y, double z) { return x + y + z; });
  ramp.min(2.0);
  ramp.max(9.0);
  overlay_job job(ramp, overlay_histogram);
  run_job(job, [] { return std::size_t(64); });
  EXPECT_EQ(job.result().range_lo, 2.0);
  EXPECT_EQ(job.result().range_hi, 9.0);
  expect_same(job.result(), compute_overlays(ramp, overlay_histogram), "set min/max");
}

TEST_F(TfOverlaysVolume, JobRangeScansEveryVoxelTypeASliceAtATime) {
  // voxels::calcMinMax scans every voxel type, so no type's range is O(1): each is scanned a slice
  // at a time, a unit a voxel, and shows in progress() (here half the work: range + histogram).
  const cvc::data_type types[] = {cvc::UChar,  cvc::UShort, cvc::UInt, cvc::Float, cvc::Double,
                                  cvc::UInt64, cvc::Char,   cvc::Int,  cvc::Int64};
  const cvc::uint64 n = 16 * 16 * 16;
  for (const cvc::data_type type : types) {
    cvc::volume vol(ctx, cvc::dimension(16, 16, 16), type);
    for (cvc::uint64 i = 0; i < n; ++i)
      vol(i, static_cast<double>((i * 37) % 101) + 3.0);
    vol.unsetMinMax();
    const std::string what = "type " + std::to_string(int(type));
    overlay_job job(vol, overlay_histogram);
    ASSERT_FALSE(job.step(97)) << what;
    EXPECT_STREQ(job.phase(), "range: scanning") << what << ": the whole range took one step";
    EXPECT_GT(job.progress(), 0.0) << what;
    std::size_t range_steps = 1;
    while (std::string(job.phase()) == "range: scanning") {
      ASSERT_FALSE(job.step(97)) << what;
      ++range_steps;
    }
    EXPECT_GE(range_steps, n / 97) << what;
    EXPECT_DOUBLE_EQ(job.progress(), 0.5) << what;
    run_job(job, [] { return std::size_t(97); });
    cvc::volume probe(vol); // what vol.min()/max() would say, without writing vol's cache
    EXPECT_EQ(job.result().range_lo, probe.min()) << what;
    EXPECT_EQ(job.result().range_hi, probe.max()) << what;
    EXPECT_FALSE(vol.minIsSet()) << what;
  }
}

TEST_F(TfOverlaysVolume, JobRangeOfSixtyFourBitIntegersIsExactAtTheirLimits) {
  // calcMinMax compares 64-bit integers in their own type, from voxel 0 on; so does the job, even
  // where a double cannot hold the value: 2^64 - 1 rounds to 2^64, which no uint64 holds.
  const cvc::uint64 umax = std::numeric_limits<cvc::uint64>::max();
  const cvc::uint64 step = cvc::uint64(1) << 50;
  const cvc::dimension dim(5, 4, 3);
  cvc::volume high(ctx, dim, cvc::UInt64); // every voxel above 2^63, voxel 0 at 2^64 - 1
  cvc::volume wide(ctx, dim, cvc::UInt64); // 2^64 - 1 at voxel 0 over small values
  cvc::volume ends(ctx, dim, cvc::Int64);  // both int64 limits
  cvc::uint64 *const hp = reinterpret_cast<cvc::uint64 *>(*high);
  cvc::uint64 *const wp = reinterpret_cast<cvc::uint64 *>(*wide);
  cvc::int64 *const ep = reinterpret_cast<cvc::int64 *>(*ends);
  for (cvc::uint64 i = 0; i < 60; ++i) {
    hp[i] = umax - i * step;
    wp[i] = i + 3;
    ep[i] = static_cast<cvc::int64>(i) - 30;
  }
  wp[0] = umax;
  wp[7] = umax - 1;
  ep[0] = std::numeric_limits<cvc::int64>::max();
  ep[9] = std::numeric_limits<cvc::int64>::min();
  for (cvc::volume *v : {&high, &wide, &ends})
    v->unsetMinMax();
  const struct {
    const char *name;
    const cvc::volume *vol;
    double lo, hi;
  } cases[] = {{"uint64 above 2^63", &high, static_cast<double>(umax - 59 * step),
                static_cast<double>(umax)},
               {"uint64 to 2^64 - 1", &wide, 4.0, static_cast<double>(umax)},
               {"int64 limits", &ends, static_cast<double>(std::numeric_limits<cvc::int64>::min()),
                static_cast<double>(std::numeric_limits<cvc::int64>::max())}};
  for (const auto &c : cases) {
    cvc::volume probe(*c.vol); // what vol.min()/max() would say, without writing vol's cache
    ASSERT_EQ(probe.min(), c.lo) << c.name;
    ASSERT_EQ(probe.max(), c.hi) << c.name;
    for (const std::size_t budget : {std::size_t(1), std::size_t(7), std::size_t(0)}) {
      overlay_job job(*c.vol, overlay_histogram);
      run_job(job, [&] { return budget; });
      ASSERT_TRUE(job.result().has_range) << c.name << " budget " << budget;
      EXPECT_EQ(job.result().range_lo, c.lo) << c.name << " budget " << budget;
      EXPECT_EQ(job.result().range_hi, c.hi) << c.name << " budget " << budget;
    }
    EXPECT_FALSE(c.vol->minIsSet()) << c.name;
  }
}

TEST_F(TfOverlaysVolume, ReadingNothingCreatesNothing) {
  cvc::state &tf = cvc::state::instance(ctx)("tf_overlays_test.fresh.transfer_function");
  const overlays none = read_overlays(tf);
  EXPECT_FALSE(none.has_range);
  EXPECT_TRUE(none.histogram.empty());
  EXPECT_TRUE(none.spectrum.empty());
  EXPECT_TRUE(none.tree.empty());
  EXPECT_EQ(none.status, "");
  EXPECT_EQ(tf.numChildren(), 0u);

  tf("histogram").value(std::string("[1,")); // malformed: reads as absent
  tf("range").value(std::string("4,2"));
  const overlays bad = read_overlays(tf);
  EXPECT_TRUE(bad.histogram.empty());
  EXPECT_FALSE(bad.has_range);
}
