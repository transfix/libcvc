/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of VolMagick. LGPL 2.1 (see other headers).
*/

// The belief-material controller's contract and inputs (belief_material_controller.h, C1 of
// docs/roadmap/NAV-BELIEF-MATERIAL-CONTROLLER-ROADMAP.md) against GRL-SNAM's Python controller.
// The golden, src/cvc/tests/data/belief_material_controller_golden.{json,cvcnm}, is written by
// GRL-SNAM (python -m grl_snam.tools.belief_material_controller_golden --out DIR); this side
// reproduces it:
//   G1  contract parse and to_meta          BIT (values, strings); refusals by verdict
//   G2  header check                        the refused fields, in order
//   G3  runtime check, material_config_for  the refused fields; the derived gate constants
//   G4  tokens and the belief patch         BIT (the goal token's dist: 1 ulp)
//   G5  visible obstacles                   BIT
// Float32 values in the golden are raw bits. The planes the tokens are cropped from are carried in
// the golden, and their SHA-256 must equal belief_material_golden.json's, so they are the planes
// libcvc's own belief kernels build (NavBeliefMaterial.Golden).

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cvc/nav/belief_material_controller.h>
#include <cvc/state/state_blob_store.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <vector>

using namespace cvc::nav;
namespace pt = boost::property_tree;
namespace fs = std::filesystem;

namespace {

const fs::path kData = fs::path(__FILE__).parent_path() / "data";

const pt::ptree &golden() {
  static const pt::ptree t = [] {
    pt::ptree r;
    pt::read_json((kData / "belief_material_controller_golden.json").string(), r);
    return r;
  }();
  return t;
}

const pt::ptree &scene() {
  static const pt::ptree t = [] {
    pt::ptree r;
    pt::read_json((kData / "belief_material_golden.json").string(), r);
    return r;
  }();
  return t;
}

std::string cvcnm_path() { return (kData / golden().get<std::string>("net.file")).string(); }

// A JSON number exactly as Python wrote it (repr round-trips): strtod, not stream extraction,
// which refuses subnormals on some standard libraries.
double num(const pt::ptree &n) { return std::strtod(n.get_value<std::string>().c_str(), nullptr); }

std::vector<std::uint32_t> u32s(const pt::ptree &arr) {
  std::vector<std::uint32_t> v;
  for (const auto &c : arr)
    v.push_back(static_cast<std::uint32_t>(std::stoul(c.second.get_value<std::string>())));
  return v;
}

std::vector<std::uint8_t> u8s(const pt::ptree &arr) {
  std::vector<std::uint8_t> v;
  for (const auto &c : arr)
    v.push_back(static_cast<std::uint8_t>(std::stoi(c.second.get_value<std::string>())));
  return v;
}

std::vector<float> f32s(const pt::ptree &arr) {
  const std::vector<std::uint32_t> b = u32s(arr);
  std::vector<float> v(b.size());
  if (!b.empty())
    std::memcpy(v.data(), b.data(), b.size() * sizeof(float));
  return v;
}

std::vector<std::uint32_t> bits_of(const float *f, std::size_t n) {
  std::vector<std::uint32_t> v(n);
  if (n)
    std::memcpy(v.data(), f, n * sizeof(float));
  return v;
}

std::vector<std::string> strings(const pt::ptree &arr) {
  std::vector<std::string> v;
  for (const auto &c : arr)
    v.push_back(c.second.get_value<std::string>());
  return v;
}

std::vector<std::string> fields(const std::vector<belief_contract_mismatch> &m) {
  std::vector<std::string> v;
  for (const auto &x : m)
    v.push_back(x.field);
  return v;
}

// belief_material_golden.json's case params, by name.
belief_material_params case_params(const std::string &name) {
  for (const auto &c : scene().get_child("cases"))
    if (c.second.get<std::string>("name") == name) {
      const pt::ptree &p = c.second.get_child("params");
      belief_material_params bp;
      bp.unknown_risk = static_cast<float>(num(p.get_child("unknown_risk")));
      bp.unknown_hard = static_cast<std::uint8_t>(num(p.get_child("unknown_hard")));
      bp.unknown_gate_hard = num(p.get_child("unknown_gate_hard")) != 0.0;
      bp.rule = p.get<std::string>("seen_rule") == "disc" ? seen_rule::disc : seen_rule::raycast;
      bp.material_range_m = num(p.get_child("material_range_m"));
      return bp;
    }
  ADD_FAILURE() << "no belief golden case " << name;
  return {};
}

int ulp_distance(float a, float b) {
  std::int32_t ia, ib;
  std::memcpy(&ia, &a, 4);
  std::memcpy(&ib, &b, 4);
  if ((ia < 0) != (ib < 0))
    return a == b ? 0 : std::numeric_limits<int>::max();
  return std::abs(ia - ib);
}

} // namespace

// ── G1: the contract ─────────────────────────────────────────────────────────

TEST(NavBeliefMaterialController, ContractGolden) {
  int accepted = 0, refused = 0;
  for (const auto &c : golden().get_child("contract")) {
    const std::string name = c.second.get<std::string>("name");
    const std::string meta = c.second.get<std::string>("meta");
    if (!c.second.get<bool>("accepted")) {
      EXPECT_THROW(belief_material_contract::from_meta(meta), std::invalid_argument) << name;
      ++refused;
      continue;
    }
    ++accepted;
    belief_material_contract got;
    ASSERT_NO_THROW(got = belief_material_contract::from_meta(meta)) << name;
    const pt::ptree &v = c.second.get_child("values");
    EXPECT_EQ(got.feature_frame, v.get<std::string>("feature_frame")) << name;
    EXPECT_EQ(got.feature_scale_m, num(v.get_child("feature_scale_m"))) << name;
    EXPECT_EQ(got.k_sharp, num(v.get_child("k_sharp"))) << name;
    EXPECT_EQ(got.d_hat_sdf_m, num(v.get_child("d_hat_sdf_m"))) << name;
    EXPECT_EQ(got.material_gate_trigger, num(v.get_child("material_gate_trigger"))) << name;
    EXPECT_EQ(got.geometry_d_hat_m, num(v.get_child("geometry_d_hat_m"))) << name;
    EXPECT_EQ(got.to_meta(), c.second.get<std::string>("to_meta")) << name;
    EXPECT_EQ(belief_material_contract::from_meta(got.to_meta()), got) << name;
  }
  EXPECT_GE(accepted, 5);
  EXPECT_GE(refused, 10);
  EXPECT_EQ(belief_material_contract().to_meta(), golden().get<std::string>("net.meta"));
}

TEST(NavBeliefMaterialController, ContractRefusalsNameTheSourceAndField) {
  try {
    belief_material_contract::from_meta("", "net.cvcnm");
    FAIL();
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("net.cvcnm: no 'grl_snam.belief_material' record"),
              std::string::npos)
        << e.what();
  }
  try {
    belief_material_contract::from_meta(R"({"grl_snam.belief_material": {"zz": 1, "aa": 2}})");
    FAIL();
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("fields ['aa', 'zz']"), std::string::npos) << e.what();
  }
  try {
    belief_material_contract::from_meta(R"({"grl_snam.belief_material": {"k_sharp": "5"}})");
    FAIL();
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("k_sharp must be a number, got '5'"), std::string::npos)
        << e.what();
  }
  belief_material_contract bad;
  bad.k_sharp = 0.0;
  EXPECT_THROW(bad.to_meta(), std::invalid_argument);
  EXPECT_THROW(bad.validate(), std::invalid_argument);
  // The parser is locale-free and accepts what json.loads accepts around the record.
  // JSON's -0 is Python's int 0, so +0.0; -0.0 stays -0.0, as float("-0.0") does.
  EXPECT_NE(belief_material_contract::from_meta(
                R"({"grl_snam.belief_material": {"material_gate_trigger": -0}})")
                .to_meta()
                .find("\"material_gate_trigger\": 0.0}"),
            std::string::npos);
  EXPECT_NE(belief_material_contract::from_meta(
                R"({"grl_snam.belief_material": {"material_gate_trigger": -0.0}})")
                .to_meta()
                .find("\"material_gate_trigger\": -0.0}"),
            std::string::npos);
  EXPECT_EQ(
      belief_material_contract::from_meta(R"({"grl_snam.belief_material": {"k_sharp": 7.25}})")
          .k_sharp,
      7.25);
}

TEST(NavBeliefMaterialController, PythonFloatRepr) {
  const std::pair<double, const char *> cases[] = {
      {20.0, "20.0"},
      {0.3, "0.3"},
      {0.1, "0.1"},
      {1e-05, "1e-05"},
      {0.0001, "0.0001"},
      {1e16, "1e+16"},
      {9999999999999998.0, "9999999999999998.0"},
      {123456789.125, "123456789.125"},
      {0.30000000000000004, "0.30000000000000004"},
      {1.7976931348623157e308, "1.7976931348623157e+308"},
      {5e-324, "5e-324"},
      {2.5e-5, "2.5e-05"},
      {1.5e300, "1.5e+300"},
      {0.0, "0.0"},
      {-0.0, "-0.0"},
      {-2.0, "-2.0"},
      {1e22, "1e+22"},
      {123.0, "123.0"}};
  for (const auto &c : cases)
    EXPECT_EQ(python_float_repr(c.first), c.second) << c.second;
}

// ── G2 / G3: the checks ──────────────────────────────────────────────────────

TEST(NavBeliefMaterialController, HeaderCheckGolden) {
  const coef_energy_net net = coef_energy_net::load(cvcnm_path());
  for (const auto &c : golden().get_child("header_check")) {
    const std::string name = c.second.get<std::string>("name");
    const pt::ptree &b = c.second.get_child("belief");
    belief_material_params bp;
    bp.unknown_risk = static_cast<float>(num(b.get_child("unknown_risk")));
    bp.unknown_hard = static_cast<std::uint8_t>(num(b.get_child("unknown_hard")));
    bp.unknown_gate_hard = num(b.get_child("unknown_gate_hard")) != 0.0;
    bp.rule = b.get<std::string>("seen_rule") == "disc" ? seen_rule::disc : seen_rule::raycast;
    bp.material_range_m = num(b.get_child("material_range_m"));
    const int patch_size = c.second.get<int>("patch_size");
    const double extent = num(c.second.get_child("patch_extent_m"));
    const std::vector<std::string> want = strings(c.second.get_child("mismatches"));
    EXPECT_EQ(fields(belief_header_mismatches(net, bp, patch_size, extent)), want) << name;
    if (want.empty()) {
      const belief_material_controller ctl(net, bp, patch_size, extent);
      EXPECT_EQ(ctl.contract(), belief_material_contract()) << name;
    } else {
      EXPECT_THROW(belief_material_controller(net, bp, patch_size, extent), std::invalid_argument)
          << name;
    }
  }
}

TEST(NavBeliefMaterialController, ControllerRefusals) {
  coef_energy_net net = coef_energy_net::load(cvcnm_path());
  belief_material_params bp;
  const belief_material_controller ok =
      belief_material_controller::from_cvcnm(cvcnm_path(), bp, 9, 36.0);
  EXPECT_EQ(ok.patch_size(), 9);
  EXPECT_EQ(ok.patch_extent_m(), 36.0);
  EXPECT_EQ(ok.net().risk_channels(), 3);

  // No contract in the meta: refused after the header check, naming the file.
  coef_energy_net bare = net;
  bare.set_meta("");
  try {
    belief_material_controller(bare, bp, 9, 36.0, "bare.cvcnm");
    FAIL();
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("bare.cvcnm: no 'grl_snam.belief_material'"),
              std::string::npos)
        << e.what();
  }
  // A contract for another frame is refused.
  coef_energy_net wrong_meta = net;
  wrong_meta.set_meta(R"({"grl_snam.belief_material": {"feature_frame": "absolute"}})");
  EXPECT_THROW(belief_material_controller(wrong_meta, bp, 9, 36.0), std::invalid_argument);
  // The settings themselves: a disc seen rule (after the header, which also names it), a patch
  // size below 2 and a non-finite extent.
  EXPECT_THROW(belief_material_controller(net, bp, 1, 36.0), std::invalid_argument);
  EXPECT_THROW(belief_material_controller(net, bp, 9, std::nan("")), std::invalid_argument);
  // A header check failure lists every field in one message.
  belief_material_params off;
  off.unknown_risk = 0.25f;
  off.material_range_m = 7.0;
  try {
    belief_material_controller(net, off, 9, 36.0, "x.cvcnm");
    FAIL();
  } catch (const std::invalid_argument &e) {
    const std::string w = e.what();
    EXPECT_NE(w.find("x.cvcnm: incompatible belief checkpoint: unknown_risk=0.0 (expected 0.25)"),
              std::string::npos)
        << w;
    EXPECT_NE(w.find("material_range_m=0.0 (expected 7.0)"), std::string::npos) << w;
  }
}

TEST(NavBeliefMaterialController, RuntimeCheckGolden) {
  const belief_material_controller ctl =
      belief_material_controller::from_cvcnm(cvcnm_path(), {}, 9, 36.0);
  for (const auto &c : golden().get_child("runtime_check")) {
    const std::string name = c.second.get<std::string>("name");
    material_config mc;
    mc.k_sharp = static_cast<float>(num(c.second.get_child("k_sharp")));
    mc.d_hat_m = static_cast<float>(num(c.second.get_child("d_hat_m")));
    mc.gate.material_trigger = num(c.second.get_child("material_trigger"));
    // The golden holds k_sharp and d_hat_m already rounded to float32, as material_config does.
    EXPECT_EQ(static_cast<double>(mc.k_sharp), num(c.second.get_child("k_sharp"))) << name;
    const std::vector<std::string> want = strings(c.second.get_child("mismatches"));
    EXPECT_EQ(fields(belief_runtime_mismatches(ctl.contract(), mc)), want) << name;
    if (want.empty())
      EXPECT_NO_THROW(ctl.check_runtime(mc)) << name;
    else
      EXPECT_THROW(ctl.check_runtime(mc), std::invalid_argument) << name;
  }
  // The library default material_config is not the contract's.
  EXPECT_EQ(fields(belief_runtime_mismatches(ctl.contract(), material_config())),
            (std::vector<std::string>{"k_sharp", "d_hat_sdf_m", "material_trigger"}));
}

TEST(NavBeliefMaterialController, MaterialConfigForGolden) {
  const belief_material_contract contract;
  for (const auto &c : golden().get_child("material_config_for")) {
    const double cell_w = num(c.second.get_child("cell_w"));
    const material_config mc = material_config_for(contract, 1.5f, 1.0f, cell_w);
    EXPECT_EQ(mc.gate.horizon_cells, c.second.get<int>("horizon_cells")) << cell_w;
    // 0 here; sim_world resolves a margin <= 0 to 2 cells, which is Python's value.
    EXPECT_LE(mc.gate.hard_margin_m, 0.0) << cell_w;
    EXPECT_EQ(2.0 * cell_w, num(c.second.get_child("hard_margin_m"))) << cell_w;
    EXPECT_EQ(mc.gate.primitive_count, c.second.get<int>("primitive_count")) << cell_w;
    EXPECT_EQ(mc.gate.improvement_margin, num(c.second.get_child("improvement_margin")));
    EXPECT_EQ(mc.gate.progress_slack_cells, num(c.second.get_child("progress_slack_cells")));
    EXPECT_EQ(mc.gate.material_trigger, num(c.second.get_child("material_trigger")));
    EXPECT_EQ(static_cast<double>(mc.k_sharp), num(c.second.get_child("k_sharp")));
    EXPECT_EQ(static_cast<double>(mc.d_hat_m), num(c.second.get_child("d_hat_sdf_m")));
    EXPECT_EQ(mc.sigma, num(c.second.get_child("sigma")));
    EXPECT_EQ(mc.lam_soft, 1.5f);
    EXPECT_EQ(mc.lam_hard, 1.0f);
    EXPECT_TRUE(belief_runtime_mismatches(contract, mc).empty());
  }
  EXPECT_THROW(material_config_for(contract, 1.5f, 1.0f, 0.0), std::invalid_argument);
  EXPECT_THROW(material_config_for(contract, 1.5f, 1.0f, std::nan("")), std::invalid_argument);
}

// ── G5: visible obstacles ────────────────────────────────────────────────────

namespace {

void expect_obstacles(const pt::ptree &want, const std::uint8_t *last_visible,
                      const std::uint8_t *truth, int rows, int cols, const double *b, double px,
                      double py, const std::string &what) {
  const int mo = want.get<int>("max_obstacles");
  std::vector<float> C(static_cast<std::size_t>(2) * mo), R(mo), W(mo);
  const int k = visible_obstacles(last_visible, truth, rows, cols, b[0], b[1], b[2], b[3], px, py,
                                  mo, C.data(), R.data(), W.data());
  const std::vector<std::uint32_t> wc = u32s(want.get_child("centers"));
  ASSERT_EQ(static_cast<std::size_t>(2) * k, wc.size()) << what;
  EXPECT_EQ(bits_of(C.data(), wc.size()), wc) << what;
  EXPECT_EQ(bits_of(R.data(), k), u32s(want.get_child("radii"))) << what;
  EXPECT_EQ(bits_of(W.data(), k), u32s(want.get_child("weights"))) << what;
}

} // namespace

TEST(NavBeliefMaterialController, VisibleObstaclesGolden) {
  const int rows = scene().get<int>("grid.rows"), cols = scene().get<int>("grid.cols");
  double b[4];
  int i = 0;
  for (const auto &v : scene().get_child("grid.bounds"))
    b[i++] = num(v.second);
  const std::vector<std::uint8_t> truth = u8s(scene().get_child("truth"));
  int checked = 0;
  for (const auto &o : golden().get_child("obstacles")) {
    const std::vector<std::uint8_t> lv = u8s(o.second.get_child("last_visible"));
    ASSERT_EQ(lv.size(), truth.size());
    std::vector<double> pos;
    for (const auto &p : o.second.get_child("position"))
      pos.push_back(num(p.second));
    const std::string what = "tick " + o.second.get<std::string>("tick") + " agent " +
                             o.second.get<std::string>("agent");
    for (const auto &c : o.second.get_child("cases")) {
      expect_obstacles(c.second, lv.data(), truth.data(), rows, cols, b, pos[0], pos[1], what);
      ++checked;
    }
  }
  EXPECT_EQ(checked, 18);
  for (const auto &t : golden().get_child("obstacle_ties")) {
    const int r = t.second.get<int>("rows"), c = t.second.get<int>("cols");
    double tb[4];
    int j = 0;
    for (const auto &v : t.second.get_child("bounds"))
      tb[j++] = num(v.second);
    std::vector<double> pos;
    for (const auto &p : t.second.get_child("position"))
      pos.push_back(num(p.second));
    const std::vector<std::uint8_t> ones(static_cast<std::size_t>(r) * c, 1);
    expect_obstacles(t.second, ones.data(), ones.data(), r, c, tb, pos[0], pos[1], "tie scene");
  }
}

TEST(NavBeliefMaterialController, VisibleObstaclesRefusals) {
  const std::uint8_t m[4] = {1, 1, 1, 1};
  float C[8], R[4], W[4];
  EXPECT_EQ(visible_obstacles(m, m, 2, 2, 0, 0, 1, 1, 0, 0, 0, nullptr, nullptr, nullptr), 0);
  EXPECT_EQ(visible_obstacles(m, m, 2, 2, 0, 0, 1, 1, 0, 0, 4, C, R, W), 4);
  EXPECT_EQ(R[0], 0.5f);
  EXPECT_THROW(visible_obstacles(m, m, 1, 4, 0, 0, 1, 1, 0, 0, 4, C, R, W), std::invalid_argument);
  EXPECT_THROW(visible_obstacles(m, m, 2, 2, 0, 0, 1, 1, 0, 0, -1, C, R, W), std::invalid_argument);
  EXPECT_THROW(visible_obstacles(m, m, 2, 2, 0, 0, 0, 1, 0, 0, 4, C, R, W), std::invalid_argument);
  EXPECT_THROW(visible_obstacles(m, m, 2, 2, 0, 0, 1, 1, std::nan(""), 0, 4, C, R, W),
               std::invalid_argument);
}

// ── G4: tokens ───────────────────────────────────────────────────────────────

TEST(NavBeliefMaterialController, TokensGolden) {
  const pt::ptree &g = golden();
  const int rows = g.get<int>("scene.rows"), cols = g.get<int>("scene.cols");
  const std::size_t hw = static_cast<std::size_t>(rows) * cols;
  double b[4];
  int i = 0;
  for (const auto &v : g.get_child("scene.bounds"))
    b[i++] = num(v.second);
  const int P = g.get<int>("net.patch_size");
  const double extent = num(g.get_child("net.patch_extent_m"));

  // The planes, checked against belief_material_golden.json's hashes of the same case, tick and
  // plane: they are the rasters libcvc's belief kernels build.
  struct planes_t {
    std::vector<float> risk;
    std::vector<std::uint8_t> hard_b, seen;
  };
  std::map<std::tuple<std::string, int, int>, planes_t> planes;
  for (const auto &p : g.get_child("planes")) {
    const std::string cname = p.second.get<std::string>("case");
    const int tick = p.second.get<int>("tick"), plane = p.second.get<int>("plane");
    planes_t pl{f32s(p.second.get_child("risk")), u8s(p.second.get_child("hard_b")),
                u8s(p.second.get_child("material_seen"))};
    ASSERT_EQ(pl.risk.size(), hw);
    ASSERT_EQ(pl.hard_b.size(), hw);
    ASSERT_EQ(pl.seen.size(), hw);
    bool found = false;
    for (const auto &c : scene().get_child("cases")) {
      if (c.second.get<std::string>("name") != cname)
        continue;
      int t = 0;
      for (const auto &tk : c.second.get_child("ticks")) {
        if (t++ != tick)
          continue;
        int m = 0;
        for (const auto &want : tk.second.get_child("planes")) {
          if (m++ != plane)
            continue;
          found = true;
          const auto *rb = reinterpret_cast<const unsigned char *>(pl.risk.data());
          EXPECT_EQ(cvc::sha256_hex(rb, hw * sizeof(float)), want.second.get<std::string>("risk"))
              << cname << " " << tick << " " << plane;
          EXPECT_EQ(cvc::sha256_hex(pl.hard_b.data(), hw), want.second.get<std::string>("hard_b"))
              << cname << " " << tick << " " << plane;
          EXPECT_EQ(cvc::sha256_hex(pl.seen.data(), hw),
                    want.second.get<std::string>("material_seen"))
              << cname << " " << tick << " " << plane;
        }
      }
    }
    EXPECT_TRUE(found) << cname << " " << tick << " " << plane;
    planes.emplace(std::make_tuple(cname, tick, plane), std::move(pl));
  }

  // Each agent's obstacles (max_obstacles 36) from G5.
  std::map<std::pair<int, int>, std::vector<float>> centers, radii, weights;
  for (const auto &o : g.get_child("obstacles")) {
    const auto key = std::make_pair(o.second.get<int>("tick"), o.second.get<int>("agent"));
    const pt::ptree &c = o.second.get_child("cases").begin()->second;
    centers[key] = f32s(c.get_child("centers"));
    radii[key] = f32s(c.get_child("radii"));
    weights[key] = f32s(c.get_child("weights"));
  }

  int checked = 0;
  for (const auto &t : g.get_child("tokens")) {
    const pt::ptree &tk = t.second;
    const std::string cname = tk.get<std::string>("case");
    const int tick = tk.get<int>("tick"), agent = tk.get<int>("agent");
    const std::string what = cname + " tick " + std::to_string(tick) + " agent " +
                             std::to_string(agent) + " " + tk.get<std::string>("variant");
    const planes_t &pl = planes.at(std::make_tuple(cname, tick, tk.get<int>("plane")));
    const belief_material_view view{
        pl.risk.data(), pl.hard_b.data(), pl.seen.data(), rows, cols, b[0], b[1], b[2], b[3]};
    std::vector<double> pos, goal;
    for (const auto &p : tk.get_child("position"))
      pos.push_back(num(p.second));
    for (const auto &p : tk.get_child("goal"))
      goal.push_back(num(p.second));
    const int N = tk.get<int>("n_obstacles");
    const auto key = std::make_pair(tick, agent);
    const bool none = tk.get<std::string>("variant") == "no_obstacles";
    ASSERT_EQ(none ? 0u : radii[key].size(), static_cast<std::size_t>(N)) << what;

    belief_material_controller_inputs in;
    build_belief_material_controller_inputs(
        pos[0], pos[1], goal[0], goal[1], none ? nullptr : centers[key].data(),
        none ? nullptr : radii[key].data(), none ? nullptr : weights[key].data(), N, view, extent,
        P, num(tk.get_child("feature_scale_m")), case_params(cname), in);

    EXPECT_EQ(bits_of(in.obs_feats.data(), in.obs_feats.size()), u32s(tk.get_child("obs_feats")))
        << what;
    EXPECT_EQ(in.obs_mask, std::vector<std::uint8_t>(N, 1)) << what;
    const std::vector<float> gf = f32s(tk.get_child("goal_feats"));
    ASSERT_EQ(gf.size(), 4u);
    EXPECT_EQ(bits_of(in.goal_feats, 2), bits_of(gf.data(), 2)) << what;
    EXPECT_LE(ulp_distance(in.goal_feats[2], gf[2]), 1) << what; // numpy's norm: BLAS sdot
    EXPECT_EQ(bits_of(&in.goal_feats[3], 1), bits_of(&gf[3], 1)) << what;
    EXPECT_EQ(bits_of(in.risk_patch.data(), in.risk_patch.size()), u32s(tk.get_child("risk_patch")))
        << what;
    ++checked;
  }
  EXPECT_EQ(checked, 31);
}

TEST(NavBeliefMaterialController, TokensRefusals) {
  const float risk[4] = {0, 0, 0, 0};
  const std::uint8_t m[4] = {1, 1, 1, 1};
  const belief_material_view view{risk, m, m, 2, 2, 0.0, 0.0, 1.0, 1.0};
  belief_material_controller_inputs in;
  const belief_material_params bp;
  EXPECT_NO_THROW(build_belief_material_controller_inputs(0, 0, 1, 1, nullptr, nullptr, nullptr, 0,
                                                          view, 2.0, 3, 20.0, bp, in));
  EXPECT_TRUE(in.obs_feats.empty());
  EXPECT_EQ(in.risk_patch.size(), 27u);
  EXPECT_EQ(in.goal_feats[3], 1.0f);
  EXPECT_THROW(build_belief_material_controller_inputs(0, 0, 1, 1, nullptr, nullptr, nullptr, 0,
                                                       view, 2.0, 3, 0.0, bp, in),
               std::invalid_argument);
  EXPECT_THROW(build_belief_material_controller_inputs(std::nan(""), 0, 1, 1, nullptr, nullptr,
                                                       nullptr, 0, view, 2.0, 3, 20.0, bp, in),
               std::invalid_argument);
  EXPECT_THROW(build_belief_material_controller_inputs(0, 0, 1, 1, nullptr, nullptr, nullptr, 2,
                                                       view, 2.0, 3, 20.0, bp, in),
               std::invalid_argument);
  belief_material_view unset = view;
  unset.seen = nullptr;
  EXPECT_THROW(build_belief_material_controller_inputs(0, 0, 1, 1, nullptr, nullptr, nullptr, 0,
                                                       unset, 2.0, 3, 20.0, bp, in),
               std::invalid_argument);
}
