/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph's SE(2) trainer (inc/cvc/ph/se2_train.h): the hand-written reverse pass and train.py's
// optimizer.
//
// - The gradient against central differences (no golden needed), and against JAX's autodiff of
//   the same objective (rollout loss + beta KL, one fixed draw z, two 21-point held-out windows,
//   the Humvee fit) from tests/data/ph_se2_golden.json.
// - The optimizer against optax: train.py's chain on a 4-feature parameter tree for 300 steps of
//   gradients given by formula, and Prodigy alone where its d grows
//   (tests/data/ph_optim_golden.json).
// - The objective is the same bit for bit on one thread or many, and a short fit to flights of a
//   known model lowers the loss.

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/core/thread_pool.h>
#include <cvc/ph/se2_train.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <map>
#include <string>
#include <vector>

namespace {

using namespace cvc::ph;
namespace fs = std::filesystem;
namespace pt = boost::property_tree;

fs::path data_dir() { return fs::path(__FILE__).parent_path() / "data"; }

std::vector<double> numbers(const pt::ptree &node) {
  std::vector<double> out;
  for (const auto &kv : node)
    out.push_back(kv.second.get_value<double>());
  return out;
}

std::vector<double> flat(const se2_params &p) {
  std::vector<double> out;
  p.for_each_leaf([&](const char *, const double *d, std::size_t n, se2_leaf_group) {
    out.insert(out.end(), d, d + n);
  });
  return out;
}

// A small model with every piece switched on and its levels spread, for the gradient checks.
se2_model small_model(std::uint64_t seed) {
  se2_init init;
  init.features = 8;
  init.initial_randomness = 0.3;
  init.initial_control_gp_mean_scale = 1.0;
  init.initial_log_std = -2.0;
  se2_model m = se2_model::initialize(init, seed, {6.0, 0.5, 0.3}, {1.0, 1.0}, 0.02);
  rng r(seed + 99);
  for (se2_gp *g : {&m.params.D, &m.params.G, &m.params.V})
    for (auto &x : g->mean)
      x = 0.3 * r.normal();
  m.setup.v_length = 5.0; // a potential that bends within a window
  return m;
}

// Windows rolled out from `truth` under smooth inputs, with noise.
se2_windows synthetic_windows(const se2_model &truth, int flights, int rows, double noise,
                              std::uint64_t seed) {
  const se2_vehicle<double> veh(truth.setup, se2_draw<double>(truth), truth.twist_scale);
  se2_flights f;
  f.count = flights;
  f.rows = rows;
  f.width = 8;
  f.data.resize(static_cast<std::size_t>(flights) * rows * 8);
  rng r(seed);
  for (int fl = 0; fl < flights; ++fl) {
    double s[6] = {r.normal(),       r.normal(),       r.normal(),
                   4.0 + r.normal(), 0.1 * r.normal(), 0.1 * r.normal()};
    const double a = 0.5 * r.normal(), b = 0.5 * r.normal(), w = 1.0 + r.uniform();
    for (int k = 0; k < rows; ++k) {
      double *row = f.data.data() + (static_cast<std::size_t>(fl) * rows + k) * 8;
      const double u[2] = {a * std::sin(w * 0.02 * k), b * std::cos(1.7 * w * 0.02 * k)};
      if (k > 0) {
        double next[6];
        veh.interval_step(s, u, truth.interval, truth.substeps, nullptr, next);
        std::copy(next, next + 6, s);
      }
      std::copy(s, s + 6, row);
      row[6] = k ? u[0] : 0.0;
      row[7] = k ? u[1] : 0.0;
    }
  }
  return cut_windows(noise > 0 ? add_observation_noise(f, noise, seed + 1) : f, rows, rows);
}

// The analytic gradient against central differences, on every level and on a sample of the
// weights and log-scales, with a draw and the KL term on.
TEST(PhSe2Train, GradientMatchesFiniteDifferences) {
  const se2_model m0 = small_model(3);
  const se2_windows w = synthetic_windows(small_model(4), 3, 16, 0.02, 5);
  rng r(6);
  const se2_draw_noise z = se2_draw_noise::standard_normal(m0, r);
  const double beta_scale = 0.01;
  se2_params grad;
  const se2_objective base = se2_rollout_objective(m0, w, nullptr, &z, beta_scale, &grad);
  ASSERT_EQ(base.healthy, w.count);
  const std::vector<double> g = flat(grad);
  std::vector<std::string> names;
  std::vector<std::size_t> offsets;
  {
    std::size_t off = 0;
    m0.params.for_each_leaf([&](const char *name, const double *, std::size_t n, se2_leaf_group) {
      names.emplace_back(name);
      offsets.push_back(off);
      off += n;
    });
  }
  int checked = 0;
  double worst = 0.0;
  std::size_t leaf = 0;
  se2_model m = m0;
  m.params.for_each_leaf([&](const char *name, double *d, std::size_t n, se2_leaf_group) {
    const std::size_t stride = n > 12 ? n / 7 : 1;
    for (std::size_t i = 0; i < n; i += stride) {
      const double keep = d[i], h = 1e-6 * std::max(1.0, std::fabs(keep));
      d[i] = keep + h;
      const double fp = se2_rollout_objective(m, w, nullptr, &z, beta_scale).value;
      d[i] = keep - h;
      const double fm = se2_rollout_objective(m, w, nullptr, &z, beta_scale).value;
      d[i] = keep;
      const double fd = (fp - fm) / (2.0 * h);
      const double an = g[offsets[leaf] + i];
      const double err = std::fabs(an - fd) / (1e-6 + std::fabs(fd) + std::fabs(an));
      worst = std::max(worst, err);
      EXPECT_LT(err, 2e-5) << name << "[" << i << "]: analytic " << an << ", central " << fd;
      ++checked;
    }
    ++leaf;
  });
  std::printf("  %d entries against central differences: worst relative %.2e\n", checked, worst);
}

// JAX's autodiff of the same objective, on the Humvee fit and two held-out windows.
TEST(PhSe2Train, GradientMatchesJax) {
  const fs::path path = data_dir() / "ph_se2_golden.json";
  pt::ptree doc;
  pt::read_json(path.string(), doc);
  const se2_model m = se2_model::load(path.string());
  const pt::ptree &gnode = doc.get_child("gradient");
  const int points = gnode.get<int>("window_points");
  const int count = gnode.get<int>("window_count");
  // the golden's windows are time-major (points x count x 8)
  const std::vector<double> tm = numbers(gnode.get_child("windows"));
  se2_windows w;
  w.points = points;
  w.width = 8;
  w.count = count;
  w.data.resize(tm.size());
  for (int t = 0; t < points; ++t)
    for (int b = 0; b < count; ++b)
      for (int c = 0; c < 8; ++c)
        w.data[(static_cast<std::size_t>(b) * points + t) * 8 + c] = tm[(t * count + b) * 8 + c];
  se2_draw_noise z;
  z.zD = numbers(gnode.get_child("z.D"));
  z.zG = numbers(gnode.get_child("z.G"));
  z.zV = numbers(gnode.get_child("z.V"));
  const double beta_scale = gnode.get<double>("beta_scale");
  se2_params grad;
  const se2_objective obj = se2_rollout_objective(m, w, nullptr, &z, beta_scale, &grad);
  EXPECT_NEAR(obj.rollout, gnode.get<double>("rollout_loss"), 1e-13);
  EXPECT_NEAR(obj.kl, gnode.get<double>("kl"), 1e-9);
  EXPECT_NEAR(obj.value, gnode.get<double>("objective"), 1e-13);
  const std::map<std::string, std::string> key = {{"D.level", "D.level"},
                                                  {"D.log_std", "D.log_std"},
                                                  {"D.mean", "D.mean"},
                                                  {"G.level", "G.level"},
                                                  {"G.log_std", "G.log_std"},
                                                  {"G.mean", "G.mean"},
                                                  {"Minv.level", "Minv.level"},
                                                  {"V.level", "V.level"},
                                                  {"V.log_std", "V.log_std"},
                                                  {"V.mean", "V.mean"},
                                                  {"likelihood.log_sigma", "likelihood.log_sigma"},
                                                  {"process.log_sigma", "process.log_sigma"}};
  double worst = 0.0;
  grad.for_each_leaf([&](const char *name, const double *d, std::size_t n, se2_leaf_group) {
    const std::vector<double> want = numbers(gnode.get_child("grad." + key.at(name)));
    ASSERT_EQ(want.size(), n) << name;
    double scale = 0.0;
    for (double x : want)
      scale = std::max(scale, std::fabs(x));
    for (std::size_t i = 0; i < n; ++i) {
      const double err = std::fabs(d[i] - want[i]) / (1e-12 + scale);
      worst = std::max(worst, err);
      EXPECT_LT(err, 1e-10) << name << "[" << i << "]: C++ " << d[i] << ", JAX " << want[i];
    }
  });
  std::printf("  1,691 entries against JAX: worst |d| / max|leaf| %.2e\n", worst);
}

// Every leaf of a 4-feature parameter tree after 300 steps of train.py's optimizer.
TEST(PhSe2Train, OptimizerMatchesOptax) {
  pt::ptree doc;
  pt::read_json((data_dir() / "ph_optim_golden.json").string(), doc);
  se2_init init;
  init.features = 4;
  se2_model m = se2_model::initialize(init, 1, {1.0, 1.0, 1.0}, {1.0, 1.0}, 0.02);
  const auto golden_name = [](const std::string &name) {
    const auto dot = name.find('.');
    return "['" + name.substr(0, dot) + "']['" + name.substr(dot + 1) + "']";
  };
  m.params.for_each_leaf([&](const char *name, double *d, std::size_t n, se2_leaf_group) {
    const auto v =
        numbers(doc.get_child("params0").get_child(pt::ptree::path_type(golden_name(name), '|')));
    ASSERT_EQ(v.size(), n) << name;
    std::copy(v.begin(), v.end(), d);
  });
  se2_optimizer_config cfg;
  cfg.total_steps = doc.get<int>("config.training.total_steps");
  cfg.learning_rate = doc.get<double>("config.optimizer.learning_rate");
  cfg.final_learning_fraction = doc.get<double>("config.optimizer.final_learning_fraction");
  cfg.level_learning_rate = doc.get<double>("config.optimizer.level_learning_rate");
  cfg.gradient_clip_norm = doc.get<double>("config.optimizer.gradient_clip_norm");
  se2_optimizer opt(m.params, cfg);
  const std::vector<double> scales = numbers(doc.get_child("gradient_rule.scales"));
  const int steps = doc.get<int>("gradient_rule.steps");
  auto record = doc.get_child("steps").begin();
  int compared = 0;
  double worst = 0.0;
  for (int t = 0; t < steps; ++t) {
    se2_params g = m.params;
    std::size_t index = 0;
    g.for_each_leaf([&](const char *name, double *d, std::size_t n, se2_leaf_group) {
      for (std::size_t i = 0; i < n; ++i, ++index) {
        const double x = static_cast<double>(index);
        d[i] = scales[t % scales.size()] *
               (std::sin(0.7 * x + 0.3) + 0.3 * std::cos(1.3 * x + 0.9 * t));
        if (t == 3 && std::string(name) == "process.log_sigma")
          d[i] = 0.0;
      }
    });
    opt.update(m.params, g);
    if (record != doc.get_child("steps").end() && record->second.get<int>("step") == t + 1) {
      const pt::ptree &rec = record->second;
      m.params.for_each_leaf([&](const char *name, const double *d, std::size_t n, se2_leaf_group) {
        const auto want = numbers(
            rec.get_child("params").get_child(pt::ptree::path_type(golden_name(name), '|')));
        for (std::size_t i = 0; i < n; ++i) {
          worst = std::max(worst, std::fabs(d[i] - want[i]) / (1.0 + std::fabs(want[i])));
          EXPECT_NEAR(d[i], want[i], 1e-12 * (1.0 + std::fabs(want[i])))
              << "step " << t + 1 << " " << name << "[" << i << "]";
        }
      });
      for (auto [group, label] : {std::pair{se2_leaf_group::D_level, "D_level"},
                                  {se2_leaf_group::G_level, "G_level"},
                                  {se2_leaf_group::V_level, "V_level"},
                                  {se2_leaf_group::Minv_level, "Minv_level"}})
        EXPECT_NEAR(opt.prodigy_d(group), rec.get<double>(std::string("prodigy_d.") + label), 1e-15)
            << "step " << t + 1 << " " << label;
      ++compared;
      ++record;
    }
  }
  EXPECT_EQ(compared, 13);
  std::printf("  300 optimizer steps against optax: worst %.2e at %d checkpoints\n", worst,
              compared);
}

// Prodigy alone, through the steps where its d grows from 1e-3 to about 2.
TEST(PhSe2Train, ProdigyMatchesOptax) {
  pt::ptree doc;
  pt::read_json((data_dir() / "ph_optim_golden.json").string(), doc);
  const pt::ptree &g = doc.get_child("prodigy_alone");
  std::vector<double> p = numbers(g.get_child("params0"));
  prodigy opt;
  opt.d0 = g.get<double>("estim_lr0");
  opt.eps = g.get<double>("eps");
  opt.safeguard_warmup = true;
  opt.init(p.data(), p.size());
  auto record = g.get_child("steps").begin();
  for (int t = 0; t < 300; ++t) {
    std::vector<double> grad(p.size());
    for (std::size_t i = 0; i < p.size(); ++i)
      grad[i] = std::sin(0.7 * i + 0.3) + 0.3 * std::cos(1.3 * i + 0.9 * t);
    opt.update(p.data(), grad.data(), p.size(), g.get<double>("learning_rate"));
    if (record != g.get_child("steps").end() && record->second.get<int>("step") == t + 1) {
      const double want_d = record->second.get<double>("d");
      EXPECT_NEAR(opt.d, want_d, 1e-11 * want_d) << "step " << t + 1;
      const auto want = numbers(record->second.get_child("params"));
      for (std::size_t i = 0; i < p.size(); ++i)
        EXPECT_NEAR(p[i], want[i], 1e-10 * (1.0 + std::fabs(want[i]))) << "step " << t + 1;
      ++record;
    }
  }
  EXPECT_GT(opt.d, 1.0); // it grew
}

TEST(PhSe2Train, ObjectiveIsTheSameOnAnyThreadCount) {
  const se2_model m = small_model(8);
  const se2_windows w = synthetic_windows(small_model(9), 12, 21, 0.02, 10);
  rng r(11);
  const se2_draw_noise z = se2_draw_noise::standard_normal(m, r);
  se2_params g1, g8;
  const auto o1 = se2_rollout_objective(m, w, nullptr, &z, 0.01, &g1);
  cvc::thread_pool pool(7);
  const auto o8 = se2_rollout_objective(m, w, nullptr, &z, 0.01, &g8, &pool);
  EXPECT_EQ(o1.value, o8.value);
  EXPECT_EQ(flat(g1), flat(g8));
}

// A window whose rollout leaves the finite numbers is left out of the loss and of the gradient
// (LieSPHGP's rollout_loss means to drop it too, but its gradient still carries the NaN, and
// train.py aborts).
TEST(PhSe2Train, ADivergingWindowIsLeftOut) {
  const se2_model m = small_model(17);
  const se2_windows healthy = synthetic_windows(small_model(18), 3, 16, 0.02, 19);
  se2_windows with_bad = healthy;
  with_bad.count += 1;
  with_bad.data.insert(with_bad.data.end(), healthy.window(0),
                       healthy.window(0) + healthy.points * healthy.width);
  with_bad.data[3 * healthy.points * healthy.width + 3] = 1e300; // v_x at its start
  rng r(20);
  const se2_draw_noise z = se2_draw_noise::standard_normal(m, r);
  se2_params g_ok, g_bad;
  const auto ok = se2_rollout_objective(m, healthy, nullptr, &z, 0.01, &g_ok);
  const auto bad = se2_rollout_objective(m, with_bad, nullptr, &z, 0.01, &g_bad);
  EXPECT_EQ(ok.healthy, 3u);
  EXPECT_EQ(bad.healthy, 3u);
  EXPECT_EQ(bad.value, ok.value);
  EXPECT_EQ(flat(g_bad), flat(g_ok));
}

// A fresh model fit for 1,200 steps to flights of another one: the rollout loss falls well below
// where it started, on windows it never trained on as well.
TEST(PhSe2Train, ShortFitLowersTheLoss) {
  const se2_model truth = small_model(12);
  const se2_windows train = synthetic_windows(truth, 40, 26, 0.01, 13);
  const se2_windows test = synthetic_windows(truth, 10, 26, 0.01, 14);
  se2_init init;
  init.features = 8;
  se2_model m = se2_model::initialize(init, 15, truth.twist_scale, {1.0, 1.0}, truth.interval);
  m.setup.v_length = truth.setup.v_length;
  const double before = se2_rollout_objective(m, test, nullptr, nullptr, 0.0).rollout;
  se2_train_config cfg;
  cfg.seed = 16;
  cfg.total_steps = 1200;
  cfg.batch_size = 8;
  cfg.kl_anneal_steps = 100;
  cfg.eval_every = 400;
  cfg.eval_windows = 10;
  cfg.threads = 4;
  int events = 0, evaluated = 0;
  double last_test = 0.0;
  se2_train_rollout(m, train, &test, cfg, [&](const se2_train_event &ev) {
    ++events;
    if (ev.evaluated) {
      ++evaluated;
      last_test = ev.test_loss;
    }
  });
  EXPECT_EQ(events, 1200);
  EXPECT_EQ(evaluated, 3);
  const double after = se2_rollout_objective(m, test, nullptr, nullptr, 0.0).rollout;
  EXPECT_DOUBLE_EQ(after, last_test);
  std::printf("  held-out rollout loss %.4f -> %.4f\n", before, after);
  EXPECT_LT(after, 0.1 * before);
  // and the fitted model survives its file
  const se2_model back = se2_model::from_json(m.to_json());
  EXPECT_EQ(se2_rollout_objective(back, test, nullptr, nullptr, 0.0).rollout, after);
}

TEST(PhSe2Train, CutsWindowsAsLieSphgpDoes) {
  se2_flights f;
  f.count = 2;
  f.rows = 250;
  f.width = 8;
  f.data.resize(2 * 250 * 8);
  for (std::size_t i = 0; i < f.data.size(); ++i)
    f.data[i] = static_cast<double>(i);
  const se2_windows w = cut_windows(f, 101, 75); // starts 0 and 75
  ASSERT_EQ(w.count, 4u);
  EXPECT_EQ(w.window(1)[0], 75.0 * 8);
  EXPECT_EQ(w.window(2)[0], 250.0 * 8);
  EXPECT_EQ(w.window(3)[100 * 8 + 7], (250.0 + 75 + 100) * 8 + 7);
  const auto t = twist_rms(f);
  EXPECT_GT(t[0], 0.0);
  const auto c = control_rms(f);
  ASSERT_EQ(c.size(), 2u);
  const se2_flights s = scale_controls(f, c);
  EXPECT_NEAR(control_rms(s)[0], 1.0, 1e-12);
}

} // namespace
