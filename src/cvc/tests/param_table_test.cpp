/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::param_table (cvc/core/param_table.h) and its generators: the state keys and their
// properties, the CLI flags (param_cli.h), the DSL dict and the app's registry behind
// (params.defaults) / (params.check) (param_dsl.h). These are the VolumeRover ports' T8 rows for
// the shared piece itself (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.5, §1.7): every field
// round-trips struct -> tree -> struct, defaults are seeded, properties written and a bad value
// reported. Runs natively and as a wasm-mt binary under node (ci.yml's ports lane).

#include <boost/describe/enum.hpp>
#include <boost/program_options.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/core/param_cli.h>
#include <cvc/core/param_dsl.h>
#include <cvc/core/param_table.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/volume/contour_tree_params.h>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace demo {

enum class edge_mode { clamp, wrap, zero };
BOOST_DESCRIBE_ENUM(edge_mode, clamp, wrap, zero)

// Every kind of field a table may hold.
struct blur_params {
  double sigma = 1.5;
  int passes = 2;
  std::uint64_t max_voxels = 1u << 20;
  std::int64_t offset = -3;
  float gain = 0.25f;
  bool normalize = true;
  std::string label = "blurred";
  edge_mode edges = edge_mode::clamp;
  std::vector<double> weights{0.25, 0.5, 0.25};
  std::vector<std::string> tags{"a", "b c"};
  std::vector<edge_mode> per_axis{edge_mode::clamp, edge_mode::wrap, edge_mode::zero};
  int smooth_iterations = 4; // a dotted key: smooth.iterations
  double low = 0.0, high = 1.0;
};

bool same_double(double a, double b) {
  return std::memcmp(&a, &b, sizeof a) == 0; // bit for bit (signed zero, payloads)
}

bool operator==(const blur_params &a, const blur_params &b) {
  bool w = a.weights.size() == b.weights.size();
  for (std::size_t i = 0; w && i < a.weights.size(); ++i)
    w = same_double(a.weights[i], b.weights[i]);
  return same_double(a.sigma, b.sigma) && a.passes == b.passes && a.max_voxels == b.max_voxels &&
         a.offset == b.offset && std::memcmp(&a.gain, &b.gain, sizeof a.gain) == 0 &&
         a.normalize == b.normalize && a.label == b.label && a.edges == b.edges && w &&
         a.tags == b.tags && a.per_axis == b.per_axis &&
         a.smooth_iterations == b.smooth_iterations && same_double(a.low, b.low) &&
         same_double(a.high, b.high);
}

} // namespace demo

template <> struct cvc::param_table<demo::blur_params> : cvc::param_fields<demo::blur_params> {
  using P = demo::blur_params;
  static constexpr auto fields = std::tuple{
      field("sigma", &P::sigma, unit("voxel"), range(0.0, 64.0),
            doc("the Gaussian's standard deviation")),
      field("passes", &P::passes, range(1, 16), doc("how many times it runs")),
      field("max_voxels", &P::max_voxels, doc("refuse larger volumes")),
      field("offset", &P::offset, range(-100, 100), doc("added to every voxel")),
      field("gain", &P::gain, unit("1"), doc("scales the result")),
      field("normalize", &P::normalize, doc("rescale to the input's range")),
      field("label", &P::label, doc("the output's description")),
      field("edges", &P::edges, doc("what lies past the edge")),
      field("weights", &P::weights, range(0.0, 1.0), doc("the kernel's taps")),
      field("tags", &P::tags, doc("free-form tags")),
      field("per_axis", &P::per_axis, doc("edges per axis")),
      field("smooth.iterations", &P::smooth_iterations, range(0, 100), doc("smoothing passes")),
      field("low", &P::low, doc("the window's low end")),
      field("high", &P::high, doc("the window's high end")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (p.low > p.high)
      problems.push_back("low = " + std::to_string(p.low) +
                         " is above high = " + std::to_string(p.high));
  }
};

namespace {

namespace se = cvc::state_exec;
using demo::blur_params;
using demo::edge_mode;

// A struct with every field away from its default, with values whose text is hard to get right.
blur_params odd_values() {
  blur_params p;
  p.sigma = 0.1 + 0.2; // 0.30000000000000004
  p.passes = 16;
  p.max_voxels = std::numeric_limits<std::uint64_t>::max();
  p.offset = -100;
  p.gain = 1.0f / 3.0f;
  p.normalize = false;
  p.label = "with \"quotes\", commas and spaces";
  p.edges = edge_mode::zero;
  p.weights = {1e-300, 0.1, 1.0 / 3.0, -0.0};
  p.tags = {"x, y", "\"q\"", "", "back\\slash"};
  p.per_axis = {edge_mode::wrap};
  p.smooth_iterations = 0;
  p.low = -1.5e10;
  p.high = 6.02214076e23;
  return p;
}

TEST(ParamTable, NamesAndInfos) {
  const std::vector<std::string> names = cvc::param_names<blur_params>();
  ASSERT_EQ(names.size(), 14u);
  EXPECT_EQ(names.front(), "sigma");
  EXPECT_EQ(names[11], "smooth.iterations");
  const std::vector<cvc::param_field_info> infos = cvc::param_field_infos<blur_params>();
  EXPECT_EQ(infos[0].kind, "number");
  EXPECT_EQ(infos[0].unit, "voxel");
  EXPECT_EQ(infos[0].min, 0.0);
  EXPECT_EQ(infos[0].max, 64.0);
  EXPECT_EQ(infos[0].default_text, "1.5");
  EXPECT_EQ(infos[1].kind, "int");
  EXPECT_EQ(infos[5].kind, "bool");
  EXPECT_EQ(infos[5].default_text, "1");
  EXPECT_EQ(infos[6].kind, "string");
  EXPECT_EQ(infos[7].kind, "enum");
  EXPECT_EQ(infos[7].choices, (std::vector<std::string>{"clamp", "wrap", "zero"}));
  EXPECT_EQ(infos[8].kind, "list");
  EXPECT_EQ(infos[8].elem_kind, "number");
  EXPECT_EQ(infos[8].default_text, "[0.25, 0.5, 0.25]");
  EXPECT_EQ(infos[10].elem_kind, "enum");
  EXPECT_EQ(infos[10].choices.size(), 3u);
  EXPECT_TRUE(std::isnan(infos[2].min)) << "no range given";
  EXPECT_EQ(cvc::param_flag_name("smooth.iterations"), "smooth-iterations");
  EXPECT_EQ(cvc::param_flag_name("max_voxels"), "max-voxels");
  EXPECT_EQ(cvc::param_keyword_name("smooth.iterations"), "smooth_iterations");
}

// Every field's text reads back as the same value, bit for bit.
TEST(ParamTable, TextRoundTripsEveryField) {
  const blur_params p = odd_values();
  blur_params q; // defaults
  for (const std::string &name : cvc::param_names<blur_params>()) {
    std::string text, why;
    ASSERT_TRUE(cvc::param_get_text(p, name, text)) << name;
    ASSERT_TRUE(cvc::param_set_text(q, name, text, why)) << name << ": " << why;
  }
  EXPECT_TRUE(p == q);
  std::string t;
  ASSERT_TRUE(cvc::param_get_text(p, "sigma", t));
  EXPECT_EQ(t, "0.30000000000000004") << "the shortest round-trip form";
  ASSERT_TRUE(cvc::param_get_text(p, "max_voxels", t));
  EXPECT_EQ(t, "18446744073709551615");
  ASSERT_TRUE(cvc::param_get_text(p, "edges", t));
  EXPECT_EQ(t, "zero");
  ASSERT_TRUE(cvc::param_get_text(p, "per_axis", t));
  EXPECT_EQ(t, "[\"wrap\"]");
}

TEST(ParamTable, TextFormsAccepted) {
  blur_params p;
  std::string why;
  EXPECT_TRUE(cvc::param_set_text(p, "weights", "(1 2.5 3)", why)) << why; // DSL-printed
  EXPECT_EQ(p.weights, (std::vector<double>{1, 2.5, 3}));
  EXPECT_TRUE(cvc::param_set_text(p, "weights", "1,2, 4", why)) << why; // bare
  EXPECT_EQ(p.weights, (std::vector<double>{1, 2, 4}));
  EXPECT_TRUE(cvc::param_set_text(p, "weights", "[]", why)) << why;
  EXPECT_TRUE(p.weights.empty());
  EXPECT_TRUE(cvc::param_set_text(p, "normalize", "yes", why));
  EXPECT_TRUE(p.normalize);
  EXPECT_TRUE(cvc::param_set_text(p, "normalize", "false", why));
  EXPECT_FALSE(p.normalize);
  EXPECT_TRUE(cvc::param_set_text(p, "passes", "3.0", why)) << "an integral number is an integer";
  EXPECT_EQ(p.passes, 3);
  EXPECT_TRUE(cvc::param_set_text(p, "sigma", " 2.25 ", why));
  EXPECT_EQ(p.sigma, 2.25);
  EXPECT_TRUE(cvc::param_set_text(p, "tags", "a, b", why));
  EXPECT_EQ(p.tags, (std::vector<std::string>{"a", "b"}));
}

TEST(ParamTable, BadTextIsRefusedAndNamed) {
  blur_params p;
  std::string why;
  EXPECT_FALSE(cvc::param_set_text(p, "passes", "abc", why));
  EXPECT_EQ(p.passes, 2) << "left as it was";
  EXPECT_NE(why.find("passes"), std::string::npos) << why;
  EXPECT_NE(why.find("'abc' is not an integer"), std::string::npos) << why;
  EXPECT_FALSE(cvc::param_set_text(p, "passes", "2.5", why));
  EXPECT_FALSE(cvc::param_set_text(p, "max_voxels", "-1", why));
  EXPECT_NE(why.find("non-negative"), std::string::npos) << why;
  EXPECT_FALSE(cvc::param_set_text(p, "edges", "mirror", why));
  EXPECT_NE(why.find("one of clamp, wrap, zero"), std::string::npos) << why;
  EXPECT_FALSE(cvc::param_set_text(p, "weights", "[1, x]", why));
  EXPECT_NE(why.find("element 1"), std::string::npos) << why;
  EXPECT_FALSE(cvc::param_set_text(p, "weights", "[1, 2", why));
  EXPECT_FALSE(cvc::param_set_text(p, "tags", "[\"open]", why));
  EXPECT_FALSE(cvc::param_set_text(p, "normalize", "maybe", why));
  EXPECT_FALSE(cvc::param_set_text(p, "nope", "1", why));
  EXPECT_NE(why.find("unknown parameter 'nope'; valid: sigma, passes"), std::string::npos) << why;
}

TEST(ParamTable, ValidateListsEveryProblem) {
  EXPECT_TRUE(cvc::params_validate(blur_params{}).empty());
  EXPECT_TRUE(cvc::params_validate(odd_values()).empty());
  blur_params p;
  p.sigma = -1.0;               // below
  p.passes = 17;                // above
  p.weights = {0.5, 2.0, -0.1}; // two elements out of range
  p.edges = static_cast<edge_mode>(7);
  p.low = 2.0;                                      // the table's own check
  p.gain = std::numeric_limits<float>::quiet_NaN(); // NaN, though it has no range
  const std::vector<std::string> v = cvc::params_validate(p);
  ASSERT_EQ(v.size(), 7u) << ::testing::PrintToString(v);
  EXPECT_EQ(v[0], "sigma = -1 is below its minimum 0");
  EXPECT_EQ(v[1], "passes = 17 is above its maximum 16");
  EXPECT_EQ(v[2], "gain is not a number");
  EXPECT_EQ(v[3], "edges = 7 is not one of clamp, wrap, zero");
  EXPECT_EQ(v[4], "weights[1] = 2 is above its maximum 1");
  EXPECT_EQ(v[5], "weights[2] = -0.1 is below its minimum 0");
  EXPECT_NE(v[6].find("is above high"), std::string::npos);
}

TEST(ParamTable, StringsMapWithErrors) {
  std::map<std::string, std::string> m{{"sigma", "3"}, {"passes", "x"}, {"bogus", "1"}};
  const cvc::param_result<blur_params> r = cvc::params_from_strings<blur_params>(m);
  EXPECT_EQ(r.value.sigma, 3.0);
  EXPECT_EQ(r.value.passes, 2);
  ASSERT_EQ(r.errors.size(), 2u);
  EXPECT_FALSE(r.ok());
  EXPECT_NE(r.message().find("; "), std::string::npos);
}

// --- the state tree (T8) ------------------------------------------------------------------------

TEST(ParamTableState, StructToTreeToStructRoundTrips) {
  cvc::app a;
  cvc::state &node = a.root()("doc.grid.segment");
  const blur_params p = odd_values();
  cvc::params_to_state(p, node);
  EXPECT_EQ(node("sigma").value(), "0.30000000000000004");
  EXPECT_EQ(node("smooth")("iterations").value(), "0") << "a dotted name is a nested key";
  EXPECT_EQ(node("edges").value(), "zero");
  const cvc::param_result<blur_params> r = cvc::params_from_state<blur_params>(node);
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_TRUE(r.value == p);
  // And the defaults the same way.
  cvc::state &other = a.root()("doc.other");
  cvc::params_to_state(blur_params{}, other);
  EXPECT_TRUE(cvc::params_from_state<blur_params>(other).value == blur_params{});
}

TEST(ParamTableState, MissingKeysAreSeededWithDefaultsAndProperties) {
  cvc::app a;
  cvc::state &node = a.root()("n");
  node("passes").value("5"); // one key set by hand
  const cvc::param_result<blur_params> r = cvc::params_from_state<blur_params>(node);
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_EQ(r.value.passes, 5);
  EXPECT_EQ(r.value.sigma, 1.5);
  EXPECT_EQ(node("sigma").value(), "1.5") << "seeded";
  EXPECT_EQ(node("weights").value(), "[0.25, 0.5, 0.25]");
  EXPECT_EQ(node("tags").value(), "[\"a\", \"b c\"]");
  EXPECT_EQ(node("passes").value(), "5") << "a key that was set is left alone";
  cvc::state &s = node("sigma");
  EXPECT_EQ(s.property("unit"), "voxel");
  EXPECT_EQ(s.property("min"), "0");
  EXPECT_EQ(s.property("max"), "64");
  EXPECT_EQ(s.property("doc"), "the Gaussian's standard deviation");
  EXPECT_EQ(s.comment(), "the Gaussian's standard deviation");
  EXPECT_EQ(s.property("default"), "1.5");
  EXPECT_EQ(s.property("type"), "number");
  EXPECT_EQ(s.property("access"), "build");
  EXPECT_EQ(node("edges").property("choices"), "clamp,wrap,zero");
  EXPECT_EQ(node("per_axis").property("choices"), "clamp,wrap,zero");
  EXPECT_EQ(node("per_axis").property("type"), "list");
  EXPECT_FALSE(node("label").has_property("unit"));
  // A string key described and then emptied on purpose stays empty.
  node("label").value("");
  EXPECT_EQ(cvc::params_from_state<blur_params>(node).value.label, "");
}

TEST(ParamTableState, BadValueIsReportedNotReplaced) {
  cvc::app a;
  cvc::state &node = a.root()("n");
  cvc::params_to_state(blur_params{}, node);
  node("passes").value("lots");
  node("sigma").value("99");
  node("edges").value("mirror");
  const cvc::param_result<blur_params> r = cvc::params_from_state<blur_params>(node);
  ASSERT_EQ(r.errors.size(), 3u) << r.message();
  EXPECT_EQ(r.errors[0], "passes: 'lots' is not an integer");
  EXPECT_EQ(r.errors[1], "edges: 'mirror' is not one of clamp, wrap, zero");
  EXPECT_EQ(r.errors[2], "sigma = 99 is above its maximum 64");
  EXPECT_EQ(node("passes").value(), "lots") << "the key keeps what the user wrote";
  EXPECT_EQ(r.value.passes, 2) << "the field keeps the default";
}

TEST(ParamTableState, DescribeWritesPropertiesOnly) {
  cvc::app a;
  cvc::state &node = a.root()("n");
  node("sigma").value("7");
  cvc::params_describe_state<blur_params>(node);
  EXPECT_EQ(node("sigma").value(), "7");
  EXPECT_EQ(node("sigma").property("unit"), "voxel");
  EXPECT_EQ(node("passes").property("max"), "16");
}

TEST(ParamTable, MarkdownHasEveryField) {
  const std::string md = cvc::params_markdown<blur_params>();
  EXPECT_NE(md.find("| `sigma` | number | `1.5` | voxel | 0 .. 64 | the Gaussian's standard "
                    "deviation |"),
            std::string::npos)
      << md;
  EXPECT_NE(md.find("| `edges` | enum: `clamp`, `wrap`, `zero` |"), std::string::npos) << md;
  EXPECT_NE(md.find("`smooth.iterations`"), std::string::npos);
}

// --- the CLI ------------------------------------------------------------------------------------

cvc::param_result<blur_params> parse_cli(std::vector<std::string> argv_s,
                                         const std::string &prefix = std::string(),
                                         std::string *help = nullptr) {
  namespace po = boost::program_options;
  po::options_description desc("demo");
  desc.add_options()("help,h", "show help");
  cvc::add_param_options<blur_params>(desc, prefix);
  if (help) {
    std::ostringstream os;
    os << desc;
    *help = os.str();
  }
  std::vector<const char *> argv{"cvc"};
  for (const std::string &s : argv_s)
    argv.push_back(s.c_str());
  po::variables_map vm;
  po::store(po::parse_command_line(static_cast<int>(argv.size()), argv.data(), desc), vm);
  po::notify(vm);
  return cvc::params_from_options<blur_params>(vm, prefix);
}

TEST(ParamTableCli, FlagsSetFields) {
  const cvc::param_result<blur_params> r =
      parse_cli({"--sigma", "2.5", "--smooth-iterations=7", "--edges", "wrap", "--weights",
                 "0.1,0.2", "--normalize=false", "--max-voxels", "64"});
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_EQ(r.value.sigma, 2.5);
  EXPECT_EQ(r.value.smooth_iterations, 7);
  EXPECT_EQ(r.value.edges, edge_mode::wrap);
  EXPECT_EQ(r.value.weights, (std::vector<double>{0.1, 0.2}));
  EXPECT_FALSE(r.value.normalize);
  EXPECT_EQ(r.value.max_voxels, 64u);
  EXPECT_EQ(r.value.passes, 2) << "a flag not given keeps the default";
}

TEST(ParamTableCli, BoolFlagAloneIsTrueAndPrefixApplies) {
  blur_params off;
  off.normalize = false;
  namespace po = boost::program_options;
  po::options_description desc("demo");
  cvc::add_param_options<blur_params>(desc, "seg-");
  const char *argv[] = {"cvc", "--seg-normalize", "--seg-passes", "3"};
  po::variables_map vm;
  po::store(po::parse_command_line(4, argv, desc), vm);
  po::notify(vm);
  const cvc::param_result<blur_params> r = cvc::params_from_options<blur_params>(vm, "seg-", off);
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_TRUE(r.value.normalize);
  EXPECT_EQ(r.value.passes, 3);
}

TEST(ParamTableCli, BadFlagsAreReported) {
  const cvc::param_result<blur_params> r = parse_cli({"--passes", "x", "--sigma", "100"});
  ASSERT_EQ(r.errors.size(), 2u) << r.message();
  EXPECT_EQ(r.errors[0], "--passes: 'x' is not an integer");
  EXPECT_EQ(r.errors[1], "sigma = 100 is above its maximum 64");
}

TEST(ParamTableCli, HelpCarriesDocUnitRangeAndDefault) {
  const std::vector<cvc::param_field_info> infos = cvc::param_field_infos<blur_params>();
  EXPECT_EQ(cvc::param_help(infos[0]),
            "the Gaussian's standard deviation [unit voxel; 0 .. 64; default 1.5]");
  EXPECT_EQ(cvc::param_help(infos[5]), "rescale to the input's range [default true]");
  EXPECT_EQ(cvc::param_help(infos[7]),
            "what lies past the edge [one of clamp, wrap, zero; default clamp]");
  EXPECT_EQ(cvc::param_help(infos[8]),
            "the kernel's taps [0 .. 1; a list: [a, b] or a,b; default [0.25, 0.5, 0.25]]");
  std::string help; // program_options wraps it, so only the flags are looked for
  parse_cli({}, std::string(), &help);
  for (const char *flag : {"--sigma", "--passes", "--max-voxels", "--normalize [=arg(=true)]",
                           "--smooth-iterations", "--per-axis"})
    EXPECT_NE(help.find(flag), std::string::npos) << flag << "\n" << help;
}

// --- the DSL ------------------------------------------------------------------------------------

TEST(ParamTableDsl, DictRoundTrips) {
  const blur_params p = odd_values();
  const se::value_t d = cvc::params_to_dict(p);
  const cvc::param_result<blur_params> r = cvc::params_from_value<blur_params>(d);
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_TRUE(r.value == p);
  const se::value_t *edges = se::field(d, "edges");
  ASSERT_NE(edges, nullptr);
  EXPECT_EQ(std::get<std::string>(edges->v), "zero");
  const se::value_t *mv = se::field(d, "max_voxels");
  ASSERT_NE(mv, nullptr);
  ASSERT_TRUE(std::holds_alternative<std::string>(mv->v)) << "past int64: its text";
  EXPECT_EQ(std::get<std::string>(mv->v), "18446744073709551615");
  EXPECT_TRUE(cvc::params_from_value<blur_params>(se::nil_value()).value == blur_params{});
}

TEST(ParamTableDsl, KindsConvertAndBadOnesAreNamed) {
  const se::value_t ok =
      se::make_dict({{"sigma", se::value_t(std::int64_t(3))},
                     {"passes", se::value_t(4.0)},
                     {"edges", se::value_t(se::symbol{"wrap"})},
                     {"normalize", se::value_t(std::int64_t(0))},
                     {"weights", se::make_list({se::value_t(std::int64_t(1)), se::value_t(0.5)})},
                     {"label", se::value_t("x")}});
  const cvc::param_result<blur_params> r = cvc::params_from_value<blur_params>(ok);
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_EQ(r.value.sigma, 3.0);
  EXPECT_EQ(r.value.passes, 4);
  EXPECT_EQ(r.value.edges, edge_mode::wrap);
  EXPECT_FALSE(r.value.normalize);
  EXPECT_EQ(r.value.weights, (std::vector<double>{1.0, 0.5}));

  const se::value_t bad = se::make_dict({{"passes", se::value_t(2.5)},
                                         {"tags", se::value_t(std::int64_t(1))},
                                         {"sgima", se::value_t(1.0)},
                                         {"weights", se::make_list({se::value_t("x")})}});
  const cvc::param_result<blur_params> b = cvc::params_from_value<blur_params>(bad);
  ASSERT_EQ(b.errors.size(), 4u) << b.message();
  EXPECT_EQ(b.errors[0], "passes: 2.5 is not an integer");
  EXPECT_EQ(b.errors[1], "tags: an integer is not a list");
  EXPECT_NE(b.errors[2].find("unknown parameter 'sgima'; valid: sigma"), std::string::npos);
  EXPECT_EQ(b.errors[3], "weights: element 0: 'x' is not a number");
  EXPECT_FALSE(cvc::params_from_value<blur_params>(se::value_t(1.0)).ok()) << "not a dict";
}

class ParamTableIntrinsics : public ::testing::Test {
protected:
  void SetUp() override {
    env = se::builtins::make_default_environment();
    cvc::register_param_intrinsics(env, app);
    cvc::register_params<blur_params>(app, "demo.blur");
  }
  se::value_t eval(const std::string &src) {
    se::stackless_evaluator ev(env);
    return ev.evaluate_script(src);
  }
  cvc::app app;
  se::environment_ptr env;
};

TEST_F(ParamTableIntrinsics, DefaultsAndCheck) {
  EXPECT_EQ(se::to_string(eval("(get-attr (params.defaults \"demo.blur\") \"passes\")")), "2");
  EXPECT_EQ(se::to_string(eval("(get-attr (params.defaults \"demo.blur\") \"edges\")")),
            "\"clamp\"");
  EXPECT_EQ(se::to_string(eval("(params.check \"demo.blur\" (dict \"sigma\" 2))")), "()");
  EXPECT_EQ(se::to_string(eval("(params.check \"demo.blur\" (params.defaults \"demo.blur\"))")),
            "()");
  const se::value_t problems =
      eval("(params.check \"demo.blur\" (dict \"sigma\" -1 \"passes\" \"x\"))");
  EXPECT_EQ(se::to_string(problems),
            "(\"passes: 'x' is not an integer\" \"sigma = -1 is below its minimum 0\")");
  EXPECT_EQ(se::to_string(eval("(try (params.defaults \"nope\") (catch (ValueError e) "
                               "(get-attr e \"message\")))")),
            "\"params.defaults: no parameter table named 'nope' (registered: demo.blur)\"");
  EXPECT_EQ(se::to_string(eval("(try (params.check 3 (dict)) (catch (TypeError e) 1))")), "1");
}

// The registry is the app's: another app has none of its tables (T6's two-apps rule).
TEST(ParamTableRegistry, PerApp) {
  cvc::app a, b;
  cvc::register_params<blur_params>(a, "demo.blur");
  EXPECT_NE(cvc::find_param_table(a, "demo.blur"), nullptr);
  EXPECT_EQ(cvc::find_param_table(b, "demo.blur"), nullptr);
  EXPECT_EQ(cvc::param_table_names(a), (std::vector<std::string>{"demo.blur"}));
  EXPECT_TRUE(cvc::param_table_names(b).empty());
  const auto t = cvc::find_param_table(a, "demo.blur");
  EXPECT_EQ(t->fields.size(), 14u);
  EXPECT_TRUE(t->check(se::make_dict({{"passes", se::value_t(std::int64_t(3))}})).empty());
}

// A real table: compute_contour_tree's options (cvc/volume/contour_tree_params.h).
TEST(ParamTable, ContourTreeOptionsTable) {
  cvc::contour_tree_options o;
  o.max_dim = 64;
  o.quantize_u8 = true;
  cvc::app a;
  cvc::params_to_state(o, a.root()("ct"));
  const cvc::param_result<cvc::contour_tree_options> r =
      cvc::params_from_state<cvc::contour_tree_options>(a.root()("ct"));
  EXPECT_TRUE(r.ok()) << r.message();
  EXPECT_EQ(r.value.max_dim, 64u);
  EXPECT_TRUE(r.value.quantize_u8);
  EXPECT_EQ(cvc::param_names<cvc::contour_tree_options>(),
            (std::vector<std::string>{"max_dim", "quantize_u8"}));
}

} // namespace
