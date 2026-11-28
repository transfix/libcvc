/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Parameter panels from parameter tables (cvc/core/param_panel.h; PP1 in
// docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §7.5):
//   - the widget each kind of field gets, and the text a table becomes;
//   - the shipped panels (src/cvc/ariadne/components/params/) are what the generator writes for
//     every table listed below, and every cvc::param_table in inc/ is listed;
//   - every shipped panel loads clean through cvc://, as a window and as widgets in a window of
//     one's own, with its fields bound to <keys>.<field>;
//   - an edit in a panel writes the bound key, which params_from_state reads, and a state change
//     (params_to_state) shows in the widget;
//   - two apps with the same panel do not share its state.
// The loading cases skip on a build without yaml-cpp. Natively and as a wasm-mt binary under node
// (src/cvc/tests/port_tests.cmake).
//
// To write the panels again after a table changes (or to add one: a line in shipped_panels()):
//   CVC_WRITE_PARAM_PANELS=1 param_panel_test --gtest_filter=ParamPanel.ShippedPanelsAreCurrent

#include <boost/describe/enum.hpp>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/uri.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/param_panel.h>
#include <cvc/core/param_table.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/pocket_tunnel_params.h>
#include <cvc/geometry/secondary_structures_params.h>
#include <cvc/geometry/skeleton_params.h>
#include <cvc/geometry/tiler_params.h>
#include <cvc/mol/blur.h>
#include <cvc/mol/molecule.h>
#include <cvc/state/state.h>
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/contour_tree_params.h>
#include <cvc/volume/gradient_vector_diffusion_params.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/seed_generators_params.h>
#include <cvc/volume/segmentation_params.h>
#include <cvc/volume/volume_filter.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_contours.h>
#include <cvc/volume/volume_grid_segment.h>
#include <cvc/volume/volume_slice.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <iterator>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace ari = cvc::ariadne;
namespace fs = std::filesystem;

// A table with a field of every kind, for the generator's own cases.
namespace pp1_demo {
enum class mode { fast, careful, very_careful };
BOOST_DESCRIBE_ENUM(mode, fast, careful, very_careful)

struct demo_params {
  bool enabled = true;
  mode how = mode::careful;
  std::string label = "a \"quoted\" name";
  int passes = 3;                                          // 1 .. 16: a slider
  std::int64_t budget = 100000;                            // 0 .. 1e9: a field
  int offset = -2;                                         // no range: a field
  double ratio = 0.25;                                     // 0 .. 1
  double sigma = 2.0;                                      // 0.01 .. 100: log
  double scale = 4.0;                                      // 0 .. (open)
  double spread = 0.5;                                     // 0 .. 1e300 (a stand-in)
  double floor = -std::numeric_limits<double>::infinity(); // no range, off by default
  std::vector<double> weights{1.0, 2.5};
  double extra_gain = 1.0; // extra.*: a group of its own
  bool extra_on = false;
};
} // namespace pp1_demo

template <>
struct cvc::param_table<pp1_demo::demo_params> : cvc::param_fields<pp1_demo::demo_params> {
  using P = pp1_demo::demo_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("enabled", &P::enabled, doc("whether it runs")),
      field("how", &P::how, doc("how hard it tries")),
      field("label", &P::label, doc("what it is called")),
      field("passes", &P::passes, range(1, 16), doc("how many times")),
      field("budget", &P::budget, range(0, 1e9), doc("work per step")),
      field("offset", &P::offset, doc("an offset")),
      field("ratio", &P::ratio, range(0.0, 1.0), doc("a fraction")),
      field("sigma", &P::sigma, unit("voxel"), range(0.01, 100.0), doc("a width")),
      field("scale", &P::scale, range(0.0, none), doc("a size")),
      field("spread", &P::spread, range(0.0, 1e300), doc("(a spread)")),
      field("floor", &P::floor, unit("data"), doc("values below this are dropped")),
      field("weights", &P::weights, doc("one per class")),
      field("extra.gain", &P::extra_gain, range(0.0, 2.0), doc("a gain")),
      field("extra.on", &P::extra_on, doc("whether the extra runs")),
  };
};

namespace {

using cvc::param_field_info;
using cvc::param_panel_spec;
using cvc::param_widget;

// ---- the shipped panels ----

using table_panel = cvc::param_panel_table;

template <class P>
table_panel panel(const char *type, const char *stem, const char *keys, const char *title,
                  const char *header, const char *where) {
  table_panel t;
  t.spec.stem = stem;
  t.spec.keys = keys;
  t.spec.title = title;
  t.spec.table = type;
  t.spec.header = header;
  t.spec.where = where;
  t.fields = cvc::param_field_infos<P>();
  return t;
}
#define PANEL(T, ...) panel<T>(#T, __VA_ARGS__)

// Every cvc::param_table in inc/ (EveryTableHasAPanel checks), as: the struct, the stem, the key
// root its fields sit under by default, the window's title, its header, and where its keys live
// on a node ("" for none yet). Grouped as docs/PORT_PARAMETERS.md is, then the shared types.
std::vector<table_panel> shipped_panels() {
  return {
      PANEL(cvc::skeleton_params, "skeleton", "graphics.root.children.mesh.mesh_ops.skeleton",
            "Skeleton", "inc/cvc/geometry/skeleton_params.h", "mesh_lab's mesh_ops.skeleton.*"),
      PANEL(cvc::tile_params, "tile", "graphics.root.children.contours.tiler", "Contour tiling",
            "inc/cvc/geometry/tiler_params.h", "contour_lab's tiler.*, volume_grid's grid.tile.*"),
      PANEL(cvc::seeded_segmentation_params, "seeded_segmentation",
            "graphics.root.children.grid.grid.segment", "Seeded segmentation",
            "inc/cvc/volume/segmentation_params.h", "volume_grid's grid.segment.*"),
      PANEL(cvc::gradient_vector_diffusion_params, "gradient_vector_diffusion",
            "gradient_vector_diffusion", "Gradient vector diffusion",
            "inc/cvc/volume/gradient_vector_diffusion_params.h", ""),
      PANEL(cvc::capsid_seed_params, "capsid_seed", "capsid_seed", "Capsid seeds",
            "inc/cvc/volume/seed_generators_params.h", ""),
      PANEL(cvc::rotation_seed_params, "rotation_seed", "rotation_seed", "Rotation seeds",
            "inc/cvc/volume/seed_generators_params.h", ""),
      PANEL(cvc::volume_slice_params, "volume_slice", "graphics.root.children.slice_z.slice",
            "Volume slice", "inc/cvc/volume/volume_slice.h", "volume_slice's slice.*"),
      PANEL(cvc::volume_grid_params, "volume_grid", "graphics.root.children.grid.grid",
            "Volume grid", "inc/cvc/volume/volume_grid.h",
            "volume_grid's grid.view, grid.tool and grid.slice.*"),
      PANEL(cvc::volume_grid_display_params, "volume_grid_display",
            "graphics.root.children.grid.grid.display", "Volume grid display",
            "inc/cvc/volume/volume_grid.h", "volume_grid's grid.display.*"),
      PANEL(cvc::volume_grid_source_params, "volume_grid_source",
            "graphics.root.children.grid.grid.source", "Volume grid source",
            "inc/cvc/volume/volume_grid.h", "volume_grid's grid.source.*"),
      PANEL(cvc::volume_grid_segment_view_params, "volume_grid_segment_view",
            "graphics.root.children.grid.grid.segment", "Segmentation view",
            "inc/cvc/volume/volume_grid_segment.h",
            "volume_grid's grid.segment.* (how the result draws)"),
      PANEL(cvc::class_surface_params, "class_surface",
            "graphics.root.children.grid.grid.segment.mesh", "Class surfaces",
            "inc/cvc/volume/class_surfaces.h", "volume_grid's grid.segment.mesh.*"),
      PANEL(cvc::volume_grid_contour_params, "volume_grid_contour",
            "graphics.root.children.grid.grid.contour", "Grid contours",
            "inc/cvc/volume/volume_grid_contours.h", "volume_grid's grid.contour.*"),
      PANEL(cvc::grid_contour_object, "grid_contour_object",
            "graphics.root.children.grid.grid.objects.0", "Grid contour object",
            "inc/cvc/volume/volume_grid_contours.h", "volume_grid's grid.objects.<i>.* rows"),
      PANEL(cvc::pocket_tunnel_params, "pocket_tunnel", "pocket_tunnel", "Pockets and tunnels",
            "inc/cvc/geometry/pocket_tunnel_params.h", ""),
      PANEL(cvc::secondary_params, "secondary_structures", "secondary_structures",
            "Secondary structures", "inc/cvc/geometry/secondary_structures_params.h", ""),
      PANEL(cvc::volume_filter_params, "volume_filter", "volume_filter", "Volume filter",
            "inc/cvc/volume/volume_filter.h", ""),
      PANEL(cvc::contour_tree_options, "contour_tree", "contour_tree", "Contour tree",
            "inc/cvc/volume/contour_tree_params.h", ""),
      PANEL(cvc::mol::blur_params, "mol_blur", "mol_blur", "Molecular blur", "inc/cvc/mol/blur.h",
            ""),
      PANEL(cvc::mol::molecule_read_params, "molecule_read", "molecule_read", "Molecule reading",
            "inc/cvc/mol/molecule.h", ""),
      PANEL(cvc::contour_read_params, "contour_read", "graphics.root.children.contours.tiler.read",
            "Contour reading", "inc/cvc/geometry/contours.h", "contour_lab's tiler.read.*"),
      PANEL(cvc::contour_component, "contour_component",
            "graphics.root.children.contours.tiler.components.0", "Contour component",
            "inc/cvc/geometry/contours.h", "contour_lab's tiler.components.<i>.* rows"),
      PANEL(cvc::point_class_read_params, "point_class_read", "point_class_read",
            "Point class reading", "inc/cvc/volume/point_classes.h", ""),
      PANEL(cvc::point_class, "point_class", "graphics.root.children.grid.grid.classes.0",
            "Point class", "inc/cvc/volume/point_classes.h",
            "volume_grid's grid.classes.<i>.* rows"),
  };
}

// Without its ".." segments, so the cvc:// search path holds no "..".
#ifdef CVC_REPO_DIR
fs::path repo_dir() { return fs::path(CVC_REPO_DIR).lexically_normal(); }
#else
fs::path repo_dir() {
  return fs::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
}
#endif

fs::path ariadne_dir() { return repo_dir() / "src" / "cvc" / "ariadne"; }
fs::path params_dir() { return ariadne_dir() / "components" / "params"; }

bool read_file(const fs::path &p, std::string &out) {
  std::ifstream in(p, std::ios::binary);
  if (!in)
    return false;
  out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  return true;
}

// The file's text with CRLF as LF (a Windows checkout with autocrlf).
std::string unix_lines(std::string s) {
  std::string out;
  out.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i)
    if (!(s[i] == '\r' && i + 1 < s.size() && s[i + 1] == '\n'))
      out += s[i];
  return out;
}

bool writing() {
  const char *w = std::getenv("CVC_WRITE_PARAM_PANELS");
  return w && *w && std::string(w) != "0";
}

// The loader's cvc:// against this source tree's components only.
void register_source(cvc::app &app) {
  ari::register_cvc_uri_handler(app, {ariadne_dir().string()});
}

param_field_info info(const char *name, const char *kind, double lo, double hi, const char *def) {
  param_field_info f;
  f.name = name;
  f.kind = kind;
  f.min = lo;
  f.max = hi;
  f.default_text = def;
  return f;
}

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// ---- a backend that shows what each widget holds and makes the edits a test asks for ----

struct Recorder : ari::Backend {
  std::map<std::string, std::string> shown;                // label -> its value this frame
  std::map<std::string, std::pair<double, double>> ranges; // label -> a slider's range
  std::map<std::string, std::vector<std::string>> options; // label -> a combo's options
  std::map<std::string, std::string> edits;                // label -> what to enter (once)
  std::vector<std::string> windows;

  bool take(const char *label, std::string &v) {
    const auto it = edits.find(label);
    if (it == edits.end())
      return false;
    v = it->second;
    edits.erase(it);
    return true;
  }
  ari::Capabilities capabilities() const override {
    ari::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *title, const char *, const ari::Size &, float, bool *) override {
    windows.emplace_back(title);
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const ari::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *) override {}
  void text_value(const char *, const std::string &) override {}
  void separator() override {}
  bool button(const char *label) override {
    std::string v;
    return take(label, v);
  }
  bool menu_item_action(const char *) override { return false; }
  ari::BoolEdit menu_item_toggle(const char *, bool current) override {
    ari::BoolEdit e;
    e.value = current;
    return e;
  }
  ari::BoolEdit checkbox(const char *label, bool current) override {
    shown[label] = current ? "1" : "0";
    ari::BoolEdit e;
    e.value = current;
    std::string v;
    if (take(label, v)) {
      e.changed = e.committed = true;
      e.value = v == "1";
    }
    return e;
  }
  ari::IntEdit slider_int(const char *label, const char *, int current, int lo, int hi) override {
    shown[label] = std::to_string(current);
    ranges[label] = {lo, hi};
    ari::IntEdit e;
    e.value = current;
    std::string v;
    if (take(label, v)) {
      e.changed = e.committed = true;
      e.value = std::stoi(v);
    }
    return e;
  }
  ari::DoubleEdit slider_double(const char *label, const char *, double current, double lo,
                                double hi, const char *) override {
    shown[label] = cvc::param_detail::format_double(current);
    ranges[label] = {lo, hi};
    ari::DoubleEdit e;
    e.value = current;
    std::string v;
    if (take(label, v)) {
      e.changed = e.committed = true;
      e.value = std::stod(v);
    }
    return e;
  }
  ari::DoubleEdit slider_double_log(const char *label, const char *key, double current, double lo,
                                    double hi, const char *fmt) override {
    return slider_double(label, key, current, lo, hi, fmt);
  }
  ari::IndexEdit combo(const char *label, int current,
                       const std::vector<std::string> &opts) override {
    shown[label] = current >= 0 && current < static_cast<int>(opts.size()) ? opts[current] : "";
    options[label] = opts;
    ari::IndexEdit e;
    std::string v;
    if (take(label, v))
      for (std::size_t i = 0; i < opts.size(); ++i)
        if (opts[i] == v) {
          e.changed = e.committed = true;
          e.index = static_cast<int>(i);
        }
    return e;
  }
  ari::StringEdit input_text(const char *label, const char *, const std::string &current,
                             bool) override {
    shown[label] = current;
    ari::StringEdit e;
    e.value = current;
    std::string v;
    if (take(label, v)) {
      e.changed = e.committed = true;
      e.value = v;
    }
    return e;
  }
};

// The demo table's panel, keyed under `keys`.
std::string demo_panel_text(const std::string &keys) {
  param_panel_spec s;
  s.stem = "demo";
  s.keys = keys;
  s.title = "Demo";
  s.table = "pp1_demo::demo_params";
  return cvc::param_panel_ari<pp1_demo::demo_params>(s);
}

// The document a panel's text makes: the text, then a window including its window unit.
std::string doc_with_panel(const std::string &panel_text, const std::string &window_unit) {
  return panel_text + "\nmeta: { min_libcvc: \"0.0.0\" }\nwindows:\n  - include: " + window_unit +
         "\n";
}

// The first widget of kind `k` under `w`, depth first.
const ari::Widget *find_kind(const ari::Widget &w, ari::Kind k) {
  if (w.kind == k)
    return &w;
  for (const ari::Widget &c : w.children)
    if (const ari::Widget *f = find_kind(c, k))
      return f;
  return nullptr;
}

// Every widget under `w` that binds a key, depth first.
void bound_widgets(const ari::Widget &w, std::vector<const ari::Widget *> &out) {
  if (!w.bind.empty() && w.kind != ari::Kind::Window)
    out.push_back(&w);
  for (const ari::Widget &c : w.children)
    bound_widgets(c, out);
}

ari::Kind kind_of_type(const std::string &type) {
  if (type == "checkbox")
    return ari::Kind::Checkbox;
  if (type == "combo")
    return ari::Kind::Combo;
  if (type == "input_text")
    return ari::Kind::InputText;
  if (type == "slider_int")
    return ari::Kind::SliderInt;
  if (type == "input_int")
    return ari::Kind::InputInt;
  return ari::Kind::SliderFloat;
}

std::string key(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

} // namespace

// ---- the widget each field gets ----

TEST(ParamPanel, WidgetForEachKind) {
  param_field_info b = info("smooth.enabled", "bool", kNone, kNone, "1");
  param_widget w = cvc::param_panel_widget(b);
  EXPECT_EQ(w.type, "checkbox");
  EXPECT_EQ(w.label, "Smooth enabled");
  EXPECT_EQ(w.def, "true");

  param_field_info e = info("cp_field", "enum", kNone, kNone, "gradient_magnitude");
  e.choices = {"image", "gradient_magnitude"};
  w = cvc::param_panel_widget(e);
  EXPECT_EQ(w.type, "combo");
  EXPECT_EQ(w.options, (std::vector<std::string>{"Image", "Gradient magnitude"}));
  EXPECT_EQ(w.values, (std::vector<std::string>{"image", "gradient_magnitude"}));
  EXPECT_EQ(w.def, "gradient_magnitude");

  EXPECT_EQ(cvc::param_panel_widget(info("file", "string", kNone, kNone, "")).type, "input_text");
  param_field_info list = info("weights", "list", kNone, kNone, "[1, 2]");
  list.elem_kind = "number";
  w = cvc::param_panel_widget(list);
  EXPECT_EQ(w.type, "input_text");
  EXPECT_EQ(w.def, "[1, 2]");
  EXPECT_NE(w.tooltip.find("a list"), std::string::npos);

  // Ints: a slider over a short range of the table's, a field otherwise.
  w = cvc::param_panel_widget(info("radius", "int", 0, 16, "2"));
  EXPECT_EQ(w.type, "slider_int");
  EXPECT_EQ(w.lo, 0);
  EXPECT_EQ(w.hi, 16);
  EXPECT_TRUE(w.range_from_key);
  w = cvc::param_panel_widget(info("sheet_count", "int", 0, 1e9, "2"));
  EXPECT_EQ(w.type, "input_int");
  EXPECT_EQ(w.lo, 0);
  EXPECT_EQ(w.hi, 1e9);
  w = cvc::param_panel_widget(info("timestep", "int", 0, kNone, "0"));
  EXPECT_EQ(w.type, "input_int");
  EXPECT_EQ(w.lo, 0);
  EXPECT_EQ(w.hi, std::numeric_limits<int>::max()) << "an open end: as far as an int goes";
  w = cvc::param_panel_widget(info("offset", "int", kNone, kNone, "0"));
  EXPECT_EQ(w.type, "input_int");
  EXPECT_TRUE(std::isnan(w.lo) && std::isnan(w.hi)) << "no range: no clamp";

  // Numbers: the table's range when it has both ends; log over three decades from above 0.
  w = cvc::param_panel_widget(info("gamma", "number", -1, 0, "-0.12"));
  EXPECT_EQ(w.type, "slider_float");
  EXPECT_EQ(w.lo, -1);
  EXPECT_EQ(w.hi, 0);
  EXPECT_FALSE(w.log);
  EXPECT_FALSE(w.guessed_range);
  EXPECT_TRUE(w.range_from_key);
  EXPECT_EQ(w.def, "-0.12");
  w = cvc::param_panel_widget(info("sigma", "number", 0.01, 100, "1"));
  EXPECT_TRUE(w.log);
  EXPECT_FALSE(cvc::param_panel_widget(info("x", "number", 0, 100, "1")).log) << "from 0: linear";

  // An open end: a span from the default, said in the tooltip. range: key still applies (the key
  // has no "max" for the open end, so the guess stands until a node writes one).
  w = cvc::param_panel_widget(info("radiometric_sigma", "number", 1e-9, kNone, "200"));
  EXPECT_TRUE(w.guessed_range);
  EXPECT_EQ(w.lo, 1e-9);
  EXPECT_EQ(w.hi, 2000);
  EXPECT_TRUE(w.range_from_key);
  EXPECT_NE(w.tooltip.find("guess"), std::string::npos);
  w = cvc::param_panel_widget(info("t_low", "number", kNone, kNone, "0"));
  EXPECT_EQ(w.lo, -1);
  EXPECT_EQ(w.hi, 1);
  w = cvc::param_panel_widget(info("depth", "number", kNone, 0, "-5"));
  EXPECT_EQ(w.lo, -50);
  EXPECT_EQ(w.hi, 0);
  // The 1e300 stand-in is no range, and its "max" property is not one either.
  w = cvc::param_panel_widget(info("big_ball_ratio", "number", 0, 1e300, "0.0625"));
  EXPECT_TRUE(w.guessed_range);
  EXPECT_EQ(w.hi, 1);
  EXPECT_FALSE(w.range_from_key);
  // An infinite default (a threshold that is off): no slider holds it, so it is text, and the
  // widget seeds the key with it.
  w = cvc::param_panel_widget(info("cp_feature_threshold", "number", kNone, kNone, "-inf"));
  EXPECT_EQ(w.type, "input_text");
  EXPECT_EQ(w.def, "-inf");

  // Numbers read the same on every platform: an integer as an integer, else %g's shortest exact
  // form, whatever the table's text was (MSVC's to_chars writes "1e-04" and "1e+05", wasm's
  // libc++ "0.0001" and "100000"). A made-up end is rounded to 6 digits.
  w = cvc::param_panel_widget(info("lambda", "number", 1e-9, kNone, "1e-04"));
  EXPECT_EQ(w.def, "0.0001");
  EXPECT_NE(w.tooltip.find("(1e-09 .. )"), std::string::npos) << w.tooltip;
  EXPECT_NE(cvc::param_panel_widget(info("n", "int", 0, 1e5, "10")).tooltip.find("(0 .. 100000)"),
            std::string::npos);
  EXPECT_EQ(cvc::param_panel_widget(info("flatness_ratio", "number", 0, 1e300, "1.44")).hi, 14.4);

  // The tooltip leads with the name, so a doc in parentheses never reads as an expression.
  param_field_info p = info("spread", "number", 0, 1, "0.5");
  p.doc = "(a spread)";
  p.unit = "voxel";
  w = cvc::param_panel_widget(p);
  EXPECT_EQ(w.tooltip, "spread: (a spread) (voxel; 0 .. 1)");
}

TEST(ParamPanel, ATableBecomesItsPanel) {
  const std::string t = demo_panel_text("demo");
  EXPECT_NE(t.find("GENERATED from cvc::param_table<pp1_demo::demo_params>"), std::string::npos);
  EXPECT_NE(t.find("  demo_params:\n    args: { keys: \"demo\" }\n    type: group\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - checkbox: \"Enabled\"\n        bind: \"{keys}.enabled\"\n"
                   "        def: true\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - combo: \"How\"\n        bind: \"{keys}.how\"\n"
                   "        options: [\"Fast\", \"Careful\", \"Very careful\"]\n"
                   "        values: [\"fast\", \"careful\", \"very_careful\"]\n"
                   "        def: \"careful\"\n"),
            std::string::npos);
  EXPECT_NE(t.find("        def: \"a \\\"quoted\\\" name\"\n"), std::string::npos);
  EXPECT_NE(t.find("      - slider_int: \"Passes\"\n        bind: \"{keys}.passes\"\n"
                   "        lo: 1\n        hi: 16\n        range: key\n        def: 3\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - input_int: \"Budget\"\n        bind: \"{keys}.budget\"\n"
                   "        lo: 0\n        hi: 1000000000\n        def: 100000\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - slider_float: \"Sigma\"\n        bind: \"{keys}.sigma\"\n"
                   "        lo: 0.01\n        hi: 100\n        log: true\n        range: key\n"
                   "        fmt: \"%g\"\n        def: 2\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - input_text: \"Floor\"\n        bind: \"{keys}.floor\"\n"
                   "        def: \"-inf\"\n"),
            std::string::npos);
  // The extra.* fields' group, alone.
  EXPECT_EQ(cvc::param_panel_groups(cvc::param_field_infos<pp1_demo::demo_params>()),
            std::vector<std::string>{"extra"});
  EXPECT_NE(t.find("  demo_extra_params:\n    args: { keys: \"demo\" }\n    type: group\n"
                   "    children:\n      - slider_float: \"Extra gain\"\n"
                   "        bind: \"{keys}.extra.gain\"\n"),
            std::string::npos);
  EXPECT_NE(t.find("      - checkbox: \"Extra on\"\n        bind: \"{keys}.extra.on\"\n"),
            std::string::npos);
  EXPECT_NE(t.find("  demo_panel:\n    args: { keys: \"demo\", title: \"Demo\", id: demo_panel }\n"
                   "    window: \"{title}\"\n    id: \"{id}\"\n    children:\n"
                   "      - slot: before\n      - include: demo_params\n"
                   "        args: { keys: \"{keys}\" }\n      - slot: after\n"),
            std::string::npos);
}

// ---- the shipped panels ----

TEST(ParamPanel, ShippedPanelsAreCurrent) {
  const fs::path dir = params_dir();
  if (!fs::is_directory(ariadne_dir() / "components"))
    GTEST_SKIP() << "component sources not found at " << ariadne_dir().string();
  const std::vector<table_panel> panels = shipped_panels();
  std::map<std::string, std::string> want; // file name -> text
  for (const table_panel &t : panels)
    want[t.spec.stem + "_panel.ari"] = cvc::param_panel_ari(t.spec, t.fields);
  want["INDEX.ari"] = cvc::param_panels_index(panels);

  if (writing()) {
    fs::create_directories(dir);
    for (const fs::directory_entry &e : fs::directory_iterator(dir))
      if (e.is_regular_file() && e.path().extension() == ".ari" &&
          !want.count(e.path().filename().string()))
        fs::remove(e.path()); // a table that is gone
    for (const auto &[file, text] : want) {
      std::ofstream out(dir / file, std::ios::binary);
      out << text;
      ASSERT_TRUE(out.good()) << (dir / file).string();
    }
    std::printf("wrote %zu files to %s\n", want.size(), dir.string().c_str());
    return;
  }
  for (const auto &[file, text] : want) {
    SCOPED_TRACE(file);
    std::string have;
    ASSERT_TRUE(read_file(dir / file, have))
        << (dir / file).string() << " is missing: run this test with CVC_WRITE_PARAM_PANELS=1";
    EXPECT_EQ(unix_lines(have), text)
        << file << " is not what the generator writes: run this test with CVC_WRITE_PARAM_PANELS=1";
  }
  for (const fs::directory_entry &e : fs::directory_iterator(dir))
    EXPECT_TRUE(want.count(e.path().filename().string()))
        << e.path().filename().string() << " is in params/ but no table makes it";
}

// Every cvc::param_table specialization in inc/ has a panel, so a port's new table fails here
// until it is listed in shipped_panels().
TEST(ParamPanel, EveryTableHasAPanel) {
  const fs::path inc = repo_dir() / "inc" / "cvc";
  if (!fs::is_directory(inc))
    GTEST_SKIP() << "headers not found at " << inc.string();
  std::set<std::string> listed;
  for (const table_panel &t : shipped_panels())
    listed.insert(t.spec.table);
  const std::regex spec_re(
      R"(param_table\s*<\s*([A-Za-z_][A-Za-z0-9_:]*)\s*>\s*:\s*(cvc::)?param_fields)");
  int found = 0;
  for (const fs::directory_entry &e : fs::recursive_directory_iterator(inc)) {
    if (!e.is_regular_file() || e.path().extension() != ".h")
      continue;
    std::string raw;
    ASSERT_TRUE(read_file(e.path(), raw));
    // The code without its // comments (param_table.h's own example is one).
    std::string text;
    std::istringstream lines(raw);
    for (std::string line; std::getline(lines, line);)
      text += line.substr(0, line.find("//")) + "\n";
    for (std::sregex_iterator it(text.begin(), text.end(), spec_re), end; it != end; ++it) {
      std::string name = (*it)[1].str();
      if (name.rfind("cvc::", 0) == 0)
        name = name.substr(5);
      if (name == "no_params") // an op's empty table (job_intrinsics.h)
        continue;
      ++found;
      EXPECT_TRUE(listed.count("cvc::" + name))
          << "cvc::param_table<" << name << "> (" << e.path().string()
          << ") has no panel: add it to shipped_panels() in " << __FILE__;
    }
  }
  EXPECT_EQ(found, static_cast<int>(listed.size()))
      << "a listed table is not in inc/, or a table is declared twice";
}

TEST(ParamPanel, EveryShippedPanelLoadsClean) {
  if (!ari::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  if (!fs::is_directory(params_dir()))
    GTEST_SKIP() << "panel sources not found at " << params_dir().string();
  cvc::app app;
  register_source(app);
  std::string all = "meta: { min_libcvc: \"0.0.0\" }\nimport:\n";
  std::string all_windows = "windows:\n";
  for (const table_panel &t : shipped_panels()) {
    const std::string window = t.spec.stem + "_panel", group = t.spec.stem + "_params";
    SCOPED_TRACE(window);
    // As a window, and as widgets in a window of one's own with its buttons in the slots.
    std::string doc = "meta: { min_libcvc: \"0.0.0\" }\n";
    doc += "import: cvc://components/params/" + window + ".ari\n";
    doc += "windows:\n";
    doc += "  - include: " + window + "\n";
    doc += "  - include: " + window + "\n";
    doc += "    args: { keys: mine, title: Mine, id: mine }\n"
           "    slots:\n"
           "      before: [{ text: Input }]\n"
           "      after:\n"
           "        - { button: Run, on: (state-set \"mine.request\" \"run\") }\n"
           "        - { button: Stop, on: (state-set \"mine.request\" \"stop\") }\n";
    doc += "  - window: Own\n    children:\n";
    doc += "      - include: " + group + "\n        args: { keys: own }\n";
    const ari::LoadResult r = ari::load_string(app, doc);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_TRUE(r.schema_issues.empty()) << r.schema_issues.front().text();
    for (const std::string &w : r.warnings)
      ADD_FAILURE() << w;
    ASSERT_EQ(r.root.children.size(), 3u);

    // Each field is one widget of its kind, bound to <keys>.<field>, in table order.
    const char *const roots[] = {nullptr, "mine", "own"};
    for (std::size_t i = 0; i < 3; ++i) {
      const ari::Widget &win = r.root.children[i];
      EXPECT_EQ(win.kind, ari::Kind::Window);
      std::vector<const ari::Widget *> bound;
      bound_widgets(win, bound);
      ASSERT_EQ(bound.size(), t.fields.size());
      const std::string root = roots[i] ? roots[i] : t.spec.keys;
      for (std::size_t f = 0; f < t.fields.size(); ++f) {
        EXPECT_EQ(bound[f]->bind, root + "." + t.fields[f].name);
        EXPECT_EQ(bound[f]->kind, kind_of_type(cvc::param_panel_widget(t.fields[f]).type))
            << t.fields[f].name;
      }
    }
    EXPECT_EQ(r.root.children[0].label, t.spec.title);
    // The slots: Input above the widgets, Run and Stop below.
    const ari::Widget &mine = r.root.children[1];
    ASSERT_GE(mine.children.size(), 4u);
    EXPECT_EQ(mine.children.front().kind, ari::Kind::Text);
    EXPECT_EQ(mine.children[mine.children.size() - 2].label, "Run");
    EXPECT_EQ(mine.children.back().label, "Stop");

    // Each group of dotted fields alone: those fields, in table order.
    for (const std::string &g : cvc::param_panel_groups(t.fields)) {
      SCOPED_TRACE(g);
      const ari::LoadResult gr = ari::load_string(
          app, "meta: { min_libcvc: \"0.0.0\" }\nimport: cvc://components/params/" + window +
                   ".ari\nwindows:\n  - window: Group\n    children:\n      - include: " +
                   t.spec.stem + "_" + g + "_params\n        args: { keys: grp }\n");
      ASSERT_TRUE(gr.ok) << gr.error;
      for (const std::string &w : gr.warnings)
        ADD_FAILURE() << w;
      std::vector<const ari::Widget *> bound;
      bound_widgets(gr.root, bound);
      std::vector<std::string> want;
      for (const param_field_info &f : t.fields)
        if (f.name.rfind(g + ".", 0) == 0)
          want.push_back("grp." + f.name);
      ASSERT_EQ(bound.size(), want.size());
      for (std::size_t f = 0; f < want.size(); ++f)
        EXPECT_EQ(bound[f]->bind, want[f]);
    }

    all += "  - cvc://components/params/" + window + ".ari\n";
    all_windows += "  - include: " + window + "\n";
  }
  // Every panel in one document: no unit name is defined twice.
  const ari::LoadResult r = ari::load_string(app, all + all_windows);
  ASSERT_TRUE(r.ok) << r.error;
  for (const std::string &w : r.warnings)
    ADD_FAILURE() << w;
  EXPECT_EQ(r.root.children.size(), shipped_panels().size());
}

// No panel unit has the name of a unit in a shipped component.
TEST(ParamPanel, UnitNamesAreTheirOwn) {
  const fs::path comps = ariadne_dir() / "components";
  if (!fs::is_directory(comps))
    GTEST_SKIP() << "component sources not found at " << comps.string();
  std::set<std::string> panel_units;
  for (const table_panel &t : shipped_panels()) {
    EXPECT_TRUE(panel_units.insert(t.spec.stem + "_panel").second) << t.spec.stem;
    EXPECT_TRUE(panel_units.insert(t.spec.stem + "_params").second) << t.spec.stem;
    for (const std::string &g : cvc::param_panel_groups(t.fields))
      EXPECT_TRUE(panel_units.insert(t.spec.stem + "_" + g + "_params").second)
          << t.spec.stem << " " << g;
  }
  const std::regex unit_re(R"(^  ([A-Za-z_][A-Za-z0-9_]*):\s*$)");
  int files = 0;
  for (const fs::directory_entry &e : fs::recursive_directory_iterator(comps)) {
    if (!e.is_regular_file() || e.path().extension() != ".ari" ||
        e.path().parent_path().filename() == "params")
      continue;
    std::string text;
    ASSERT_TRUE(read_file(e.path(), text));
    const std::size_t units = text.find("\nunits:");
    if (units == std::string::npos)
      continue;
    ++files;
    std::istringstream in(unix_lines(text.substr(units + 1)));
    std::string line;
    std::getline(in, line); // units:
    while (std::getline(in, line)) {
      if (!line.empty() && line[0] != ' ' && line[0] != '#')
        break; // the next top-level key
      std::smatch m;
      if (std::regex_match(line, m, unit_re))
        EXPECT_FALSE(panel_units.count(m[1].str()))
            << e.path().filename().string() << " defines " << m[1].str() << ", a panel's unit";
    }
  }
  EXPECT_GT(files, 10);
}

// ---- the panel and the state tree ----

TEST(ParamPanel, AnEditWritesTheKeyAndAStateChangeShowsInTheWidget) {
  if (!ari::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  const ari::LoadResult r =
      ari::load_string(app, doc_with_panel(demo_panel_text("demo"), "demo_panel"));
  ASSERT_TRUE(r.ok) << r.error;
  for (const std::string &w : r.warnings)
    ADD_FAILURE() << w;
  ari::Runtime rt(app, "");
  Recorder b;
  rt.set_backend(&b);
  rt.set_root(r.root);

  // The first frame: every widget shows its field's default and seeds its key with it, so the
  // table reads its defaults back.
  rt.render();
  ASSERT_EQ(b.windows, std::vector<std::string>{"Demo"});
  EXPECT_EQ(b.shown["Enabled"], "1");
  EXPECT_EQ(b.shown["How"], "Careful");
  EXPECT_EQ(b.options["How"], (std::vector<std::string>{"Fast", "Careful", "Very careful"}));
  EXPECT_EQ(b.shown["Label"], "a \"quoted\" name");
  EXPECT_EQ(b.shown["Passes"], "3");
  EXPECT_EQ(b.ranges["Passes"], std::make_pair(1.0, 16.0));
  EXPECT_EQ(b.shown["Budget"], "100000");
  EXPECT_EQ(b.shown["Ratio"], "0.25");
  EXPECT_EQ(b.shown["Floor"], "-inf");
  EXPECT_EQ(b.shown["Weights"], "[1, 2.5]");
  cvc::state &node = cvc::state::instance(app)("demo");
  cvc::param_result<pp1_demo::demo_params> back =
      cvc::params_from_state<pp1_demo::demo_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_EQ(back.value.how, pp1_demo::mode::careful);
  EXPECT_TRUE(std::isinf(back.value.floor) && back.value.floor < 0);

  // An edit in each widget writes its key, and the table reads what was entered.
  b.edits = {{"Enabled", "0"},  {"How", "Very careful"}, {"Label", "renamed"}, {"Passes", "9"},
             {"Budget", "250"}, {"Offset", "7"},         {"Ratio", "0.75"},    {"Sigma", "12.5"},
             {"Scale", "33"},   {"Weights", "[3, 4]"}};
  rt.render();
  EXPECT_TRUE(b.edits.empty()) << "a widget was not drawn: " << b.edits.begin()->first;
  EXPECT_EQ(key(app, "demo.enabled"), "0");
  EXPECT_EQ(key(app, "demo.how"), "very_careful") << "the combo stores the name, not the words";
  back = cvc::params_from_state<pp1_demo::demo_params>(node);
  ASSERT_TRUE(back.ok()) << back.message();
  EXPECT_FALSE(back.value.enabled);
  EXPECT_EQ(back.value.how, pp1_demo::mode::very_careful);
  EXPECT_EQ(back.value.label, "renamed");
  EXPECT_EQ(back.value.passes, 9);
  EXPECT_EQ(back.value.budget, 250);
  EXPECT_EQ(back.value.offset, 7);
  EXPECT_EQ(back.value.ratio, 0.75);
  EXPECT_EQ(back.value.sigma, 12.5);
  EXPECT_EQ(back.value.scale, 33.0);
  EXPECT_EQ(back.value.weights, (std::vector<double>{3.0, 4.0}));

  // A state change from outside the panel (a node, a DSL program, pycvc) shows in the widgets.
  pp1_demo::demo_params p;
  p.enabled = true;
  p.how = pp1_demo::mode::fast;
  p.passes = 12;
  p.ratio = 0.125;
  p.label = "from the tree";
  p.floor = 4.5;
  cvc::params_to_state(p, node);
  rt.render();
  EXPECT_EQ(b.shown["Enabled"], "1");
  EXPECT_EQ(b.shown["How"], "Fast");
  EXPECT_EQ(b.shown["Passes"], "12");
  EXPECT_EQ(b.shown["Ratio"], "0.125");
  EXPECT_EQ(b.shown["Label"], "from the tree");
  EXPECT_EQ(b.shown["Floor"], "4.5");
  EXPECT_EQ(b.ranges["Passes"], std::make_pair(1.0, 16.0)) << "the table's min and max";
}

// A shipped panel on its node's keys: volume_grid's segmentation parameters.
TEST(ParamPanel, AShippedPanelDrivesItsTablesKeys) {
  if (!ari::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  if (!fs::is_directory(params_dir()))
    GTEST_SKIP() << "panel sources not found at " << params_dir().string();
  cvc::app app;
  register_source(app);
  const ari::LoadResult r =
      ari::load_string(app, "meta: { min_libcvc: \"0.0.0\" }\n"
                            "import: cvc://components/params/seeded_segmentation_panel.ari\n"
                            "windows:\n  - include: seeded_segmentation_panel\n"
                            "    args: { keys: graphics.root.children.vg.grid.segment }\n");
  ASSERT_TRUE(r.ok) << r.error;
  ari::Runtime rt(app, "");
  Recorder b;
  rt.set_backend(&b);
  rt.set_root(r.root);
  // The node describes its keys first (as VolumeGridSegmentation does at realize time).
  cvc::state &seg = cvc::state::instance(app)("graphics.root.children.vg.grid.segment");
  cvc::params_describe_state<cvc::seeded_segmentation_params>(seg);
  rt.render();
  EXPECT_EQ(b.shown["Gamma"], "-0.12");
  EXPECT_EQ(b.ranges["Gamma"], std::make_pair(-1.0, 0.0));
  EXPECT_EQ(b.shown["Automatic"], "Local maxima");
  EXPECT_EQ(b.shown["Cp feature threshold"], "-inf");
  ASSERT_TRUE(cvc::params_from_state<cvc::seeded_segmentation_params>(seg).ok());

  b.edits = {{"Gamma", "-0.5"}, {"Automatic", "Critical points"}, {"Cp iterations", "40"}};
  rt.render();
  const auto p = cvc::params_from_state<cvc::seeded_segmentation_params>(seg);
  ASSERT_TRUE(p.ok()) << p.message();
  EXPECT_EQ(p.value.gamma, -0.5);
  EXPECT_EQ(p.value.automatic, cvc::seed_mode::critical_points);
  EXPECT_EQ(key(app, "graphics.root.children.vg.grid.segment.automatic"), "critical_points");
  EXPECT_EQ(p.value.cp_iterations, 40);

  // The node rescales a key's range (from its data): range: key follows.
  seg("gamma").property("min", "-0.25");
  rt.render();
  EXPECT_EQ(b.ranges["Gamma"], std::make_pair(-0.25, 0.0));
}

TEST(ParamPanel, TwoAppsDoNotShareState) {
  if (!ari::have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app a, b;
  const std::string doc = doc_with_panel(demo_panel_text("demo"), "demo_panel");
  const ari::LoadResult ra = ari::load_string(a, doc), rb = ari::load_string(b, doc);
  ASSERT_TRUE(ra.ok && rb.ok) << ra.error << rb.error;
  ari::Runtime rta(a, ""), rtb(b, "");
  Recorder ba, bb;
  rta.set_backend(&ba);
  rtb.set_backend(&bb);
  rta.set_root(ra.root);
  rtb.set_root(rb.root);
  rta.render();
  rtb.render();

  ba.edits = {{"Passes", "15"}, {"How", "Fast"}};
  rta.render();
  rtb.render();
  EXPECT_EQ(key(a, "demo.passes"), "15");
  EXPECT_EQ(key(b, "demo.passes"), "3") << "the other app keeps its default";
  EXPECT_EQ(key(b, "demo.how"), "careful");
  EXPECT_EQ(bb.shown["Passes"], "3");

  cvc::state::instance(b)("demo.ratio").value("0.5");
  rta.render();
  rtb.render();
  EXPECT_EQ(ba.shown["Ratio"], "0.25");
  EXPECT_EQ(bb.shown["Ratio"], "0.5");
}
