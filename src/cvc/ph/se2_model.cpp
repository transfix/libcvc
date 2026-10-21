/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::ph::se2_model: initialization and the JSON model file (format "cvc-ph-se2/1").
//
// The file's layout is LieSPHGP's SE(2) checkpoint written out: the features (setup), the
// parameters by subnetwork, the twist and control scales, the sample interval. Arrays are flat
// and row-major. Numbers are written with 17 significant digits, so a save and load round-trips
// every double exactly.

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdio>
#include <cvc/ph/se2_vehicle.h>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cvc {
namespace ph {

namespace {

namespace pt = boost::property_tree;

constexpr const char *kFormat = "cvc-ph-se2/1";

// Random streams, one per purpose (rng::derive tags).
enum stream : std::uint64_t { features_D = 1, features_G, features_V, levels, means };

std::array<double, 6> spd_level(double value) {
  const double l = 0.5 * std::log(value);
  return {l, l, l, 0.0, 0.0, 0.0};
}

void put_number(std::ostringstream &o, double x) {
  if (!std::isfinite(x))
    throw std::runtime_error("se2_model: cannot write a non-finite number");
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", x);
  o << buf;
}

template <class C> void put_array(std::ostringstream &o, const C &values) {
  o << '[';
  bool first = true;
  for (double x : values) {
    if (!first)
      o << ',';
    first = false;
    put_number(o, x);
  }
  o << ']';
}

std::string quoted(const std::string &s) {
  std::string out = "\"";
  for (char ch : s) {
    if (ch == '"' || ch == '\\')
      out += '\\';
    if (static_cast<unsigned char>(ch) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned char>(ch));
      out += buf;
      continue;
    }
    out += ch;
  }
  return out + "\"";
}

std::vector<double> get_array(const pt::ptree &node, const std::string &path,
                              std::size_t expect = 0) {
  const auto child = node.get_child_optional(path);
  if (!child)
    throw std::runtime_error(std::string("se2_model: ") + path + " is missing");
  std::vector<double> out;
  out.reserve(child->size());
  for (const auto &kv : *child)
    out.push_back(std::stod(kv.second.data()));
  if (expect && out.size() != expect)
    throw std::runtime_error("se2_model: " + path + " has " + std::to_string(out.size()) +
                             " values, expected " + std::to_string(expect));
  return out;
}

rff get_rff(const pt::ptree &node, const std::string &name, int input, int count) {
  rff f;
  f.input = input;
  f.count = count;
  f.frequency =
      get_array(node, "setup." + name + ".frequencies", static_cast<std::size_t>(input) * count);
  f.phase = get_array(node, "setup." + name + ".phases", static_cast<std::size_t>(count));
  return f;
}

se2_gp get_gp(const pt::ptree &node, const std::string &name, std::size_t weights,
              std::size_t levels) {
  se2_gp g;
  g.mean = get_array(node, "params." + name + ".mean", weights);
  g.log_std = get_array(node, "params." + name + ".log_std", weights);
  g.level = get_array(node, "params." + name + ".level", levels);
  return g;
}

} // namespace

std::size_t se2_params::size() const {
  std::size_t n = 0;
  for_each_leaf([&](const char *, const double *, std::size_t size, se2_leaf_group) { n += size; });
  return n;
}

se2_params se2_params::zeros_like() const {
  se2_params z = *this;
  z.for_each_leaf([](const char *, double *data, std::size_t size, se2_leaf_group) {
    for (std::size_t i = 0; i < size; ++i)
      data[i] = 0.0;
  });
  return z;
}

se2_model se2_model::initialize(const se2_init &init, std::uint64_t seed,
                                const std::array<double, 3> &twist_scale,
                                const std::vector<double> &control_scale, double interval,
                                int substeps) {
  if (init.control_dim < 1 || init.control_dim > kSe2MaxControls)
    throw std::invalid_argument("se2_model: control_dim must be 1-4");
  if (static_cast<int>(control_scale.size()) != init.control_dim)
    throw std::invalid_argument("se2_model: one control scale per input");
  if (init.features < 1 || interval <= 0.0 || substeps < 1)
    throw std::invalid_argument("se2_model: features, interval and substeps must be positive");
  se2_model m;
  se2_setup &s = m.setup;
  s.control_dim = init.control_dim;
  s.features = init.features;
  s.matern_nu = init.matern_nu;
  s.matern_length = init.matern_length;
  s.v_gp = init.v_gp;
  s.v_length = init.v_length;
  {
    rng r = rng::derive(seed, features_D);
    s.D = rff::draw(r, 3, init.features, init.matern_nu, init.matern_length);
  }
  {
    rng r = rng::derive(seed, features_G);
    s.G = rff::draw(r, 3, init.features, init.matern_nu, init.matern_length);
  }
  {
    rng r = rng::derive(seed, features_V);
    s.V = rff::draw(r, 2, init.features, init.matern_nu, init.matern_length);
  }

  rng lv = rng::derive(seed, levels);
  const auto noise = [&]() { return init.initial_randomness * lv.normal(); };
  se2_params &p = m.params;
  const auto minv = spd_level(init.initial_inverse_mass);
  for (int i = 0; i < 6; ++i)
    p.minv_level[i] = minv[i] + noise();
  const auto d = spd_level(init.initial_dissipation);
  p.D.level.resize(6);
  for (int i = 0; i < 6; ++i)
    p.D.level[i] = d[i] + noise();
  p.G.level.resize(3 * static_cast<std::size_t>(init.control_dim));
  for (auto &x : p.G.level)
    x = noise();
  p.V.level.resize(2);
  for (auto &x : p.V.level)
    x = noise();
  if (init.with_likelihood) {
    p.likelihood_log_sigma.resize(4);
    for (auto &x : p.likelihood_log_sigma)
      x = std::log(init.initial_observation_sigma) + noise();
  }
  for (auto &x : p.process_log_sigma)
    x = std::log(init.initial_diffusion) + noise();

  rng mv = rng::derive(seed, means);
  const auto gp = [&](se2_gp &g, int outputs, double scale) {
    const std::size_t n = static_cast<std::size_t>(init.features) * outputs;
    g.mean.resize(n);
    g.log_std.assign(n, init.initial_log_std);
    for (auto &x : g.mean)
      x = scale * mv.normal() / init.features;
  };
  gp(p.D, 6, 1.0);
  gp(p.G, 3 * init.control_dim, init.initial_control_gp_mean_scale);
  gp(p.V, 1, 1.0);

  m.twist_scale = twist_scale;
  m.control_scale = control_scale;
  m.interval = interval;
  m.substeps = substeps;
  return m;
}

std::string se2_model::to_json() const {
  std::ostringstream o;
  o << "{\"format\":\"" << kFormat << "\",\"provenance\":{";
  for (std::size_t i = 0; i < provenance.size(); ++i)
    o << (i ? "," : "") << quoted(provenance[i].first) << ':' << quoted(provenance[i].second);
  o << "},\"control_dim\":" << setup.control_dim << ",\"feature_count\":" << setup.features
    << ",\"G_input\":\"twist\",\"V_gp\":" << (setup.v_gp ? "true" : "false")
    << ",\"V_length_scale\":";
  put_number(o, setup.v_length);
  o << ",\"matern_smoothness\":";
  put_number(o, setup.matern_nu);
  o << ",\"matern_length_scale\":";
  put_number(o, setup.matern_length);
  o << ",\"interval\":";
  put_number(o, interval);
  o << ",\"substeps\":" << substeps << ",\"twist_scale\":";
  put_array(o, twist_scale);
  o << ",\"control_scale\":";
  put_array(o, control_scale);
  o << ",\"setup\":{";
  const std::pair<const char *, const rff *> feats[] = {
      {"D", &setup.D}, {"G", &setup.G}, {"V", &setup.V}};
  for (int i = 0; i < 3; ++i) {
    o << (i ? "," : "") << '"' << feats[i].first << "\":{\"frequencies\":";
    put_array(o, feats[i].second->frequency);
    o << ",\"phases\":";
    put_array(o, feats[i].second->phase);
    o << '}';
  }
  o << "},\"params\":{\"Minv\":{\"level\":";
  put_array(o, params.minv_level);
  o << '}';
  const std::pair<const char *, const se2_gp *> gps[] = {
      {"D", &params.D}, {"G", &params.G}, {"V", &params.V}};
  for (const auto &g : gps) {
    o << ",\"" << g.first << "\":{\"mean\":";
    put_array(o, g.second->mean);
    o << ",\"log_std\":";
    put_array(o, g.second->log_std);
    o << ",\"level\":";
    put_array(o, g.second->level);
    o << '}';
  }
  o << ",\"process\":{\"log_sigma\":";
  put_array(o, params.process_log_sigma);
  o << '}';
  if (!params.likelihood_log_sigma.empty()) {
    o << ",\"likelihood\":{\"log_sigma\":";
    put_array(o, params.likelihood_log_sigma);
    o << '}';
  }
  o << "}}\n";
  return o.str();
}

void se2_model::save(const std::string &path) const {
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("se2_model: cannot write " + path);
  out << to_json();
  if (!out)
    throw std::runtime_error("se2_model: writing " + path + " failed");
}

se2_model se2_model::from_json(const std::string &text) {
  pt::ptree root;
  std::istringstream in(text);
  try {
    pt::read_json(in, root);
  } catch (const pt::json_parser_error &e) {
    throw std::runtime_error(std::string("se2_model: ") + e.what());
  }
  // A golden file wraps the model in "model".
  const pt::ptree &node = root.get_child_optional("model") ? root.get_child("model") : root;
  if (node.get<std::string>("format", "") != kFormat)
    throw std::runtime_error(std::string("se2_model: not a ") + kFormat + " model");
  if (node.get<std::string>("G_input", "twist") != "twist")
    throw std::runtime_error("se2_model: only G_input twist is supported");
  se2_model m;
  se2_setup &s = m.setup;
  s.control_dim = node.get<int>("control_dim");
  s.features = node.get<int>("feature_count");
  if (s.control_dim < 1 || s.control_dim > kSe2MaxControls || s.features < 1)
    throw std::runtime_error("se2_model: control_dim 1-4 and a positive feature count");
  s.v_gp = node.get<bool>("V_gp");
  s.v_length = node.get<double>("V_length_scale");
  s.matern_nu = node.get<double>("matern_smoothness", 2.5);
  s.matern_length = node.get<double>("matern_length_scale", 1.0);
  s.D = get_rff(node, "D", 3, s.features);
  s.G = get_rff(node, "G", 3, s.features);
  s.V = get_rff(node, "V", 2, s.features);
  const std::size_t n = static_cast<std::size_t>(s.features);
  se2_params &p = m.params;
  const auto minv = get_array(node, "params.Minv.level", 6);
  std::copy(minv.begin(), minv.end(), p.minv_level.begin());
  p.D = get_gp(node, "D", n * 6, 6);
  p.G = get_gp(node, "G", n * 3 * s.control_dim, 3 * static_cast<std::size_t>(s.control_dim));
  p.V = get_gp(node, "V", n, 2);
  if (node.get_child_optional("params.process")) {
    const auto proc = get_array(node, "params.process.log_sigma", 3);
    std::copy(proc.begin(), proc.end(), p.process_log_sigma.begin());
  }
  if (node.get_child_optional("params.likelihood"))
    p.likelihood_log_sigma = get_array(node, "params.likelihood.log_sigma", 4);
  const auto ts = get_array(node, "twist_scale", 3);
  std::copy(ts.begin(), ts.end(), m.twist_scale.begin());
  m.control_scale = get_array(node, "control_scale", static_cast<std::size_t>(s.control_dim));
  m.interval = node.get<double>("interval");
  m.substeps = node.get<int>("substeps", 1);
  if (const auto prov = node.get_child_optional("provenance"))
    for (const auto &kv : *prov)
      m.provenance.emplace_back(kv.first, kv.second.data());
  return m;
}

se2_model se2_model::load(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("se2_model: cannot read " + path);
  std::ostringstream text;
  text << in.rdbuf();
  return from_json(text.str());
}

} // namespace ph
} // namespace cvc
