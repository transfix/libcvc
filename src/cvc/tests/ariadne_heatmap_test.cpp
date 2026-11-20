// Ariadne heatmap: (Kind::Heatmap) through the real Runtime walk, with a backend that records the
// HeatmapSpec it is handed and can report a hovered or clicked cell. Covers the ramp's colours
// (mixed, clamped, no value), the subtree's cells and labels, reading it again only after a write
// to it, a pick's writes and its on:, the handlers' `event`, the tooltip (a cell's tip, or the
// document's with the cell's tokens, or computed), the default text heatmap, the loader's keys and
// warnings, and the FTXUI backend's coloured cells (a snapshot), its pointer and its keyboard.

#include <cmath>
#include <cstdint>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;

namespace {

// The least backend the walk needs, recording each heatmap's spec and the tooltips it is asked to
// show. `use_default` draws it with Backend::heatmap (the text heatmap), so its lines land in
// `lines`. `report` is what the next heatmap call returns.
struct HeatBackend : Backend {
  std::vector<HeatmapSpec> specs;
  std::vector<std::string> lines, tips;
  HeatmapEvent report{};
  bool use_default = false;

  Capabilities capabilities() const override { return {}; }
  bool begin_main_menu_bar() override { return true; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return true; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const Size &, float, bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *t) override { lines.emplace_back(t); }
  void text_value(const char *l, const std::string &v) override {
    lines.push_back(std::string(l) + ": " + v);
  }
  void separator() override {}
  bool button(const char *) override { return false; }
  bool menu_item_action(const char *) override { return false; }
  BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  BoolEdit checkbox(const char *, bool) override { return {}; }
  IntEdit slider_int(const char *, const char *, int, int, int) override { return {}; }
  DoubleEdit slider_double(const char *, const char *, double, double, double,
                           const char *) override {
    return {};
  }
  IndexEdit combo(const char *, int, const std::vector<std::string> &) override { return {}; }
  StringEdit input_text(const char *, const char *, const std::string &, bool) override {
    return {};
  }
  void set_tooltip(const char *t) override { tips.emplace_back(t ? t : ""); }
  bool item_clicked() override { return report.clicked; } // a generic on_click would fire here
  bool item_hovered() override { return report.hovered; }
  HeatmapEvent heatmap(const char *label, const HeatmapSpec &spec) override {
    specs.push_back(spec);
    if (use_default)
      return Backend::heatmap(label, spec);
    return report;
  }
  const HeatmapSpec &last() const { return specs.back(); }
};

void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}
std::string get(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// A Runtime over a HeatBackend. Member order: the backend outlives the Runtime.
struct Doc {
  cvc::app app;
  HeatBackend be;
  Runtime rt{app, "doc"};
  explicit Doc(Widget w) {
    rt.set_backend(&be);
    rt.set_root(group({window("W", {std::move(w)})}));
  }
  const HeatmapSpec &frame() {
    be.specs.clear();
    be.tips.clear();
    rt.render();
    EXPECT_EQ(be.specs.size(), 1u);
    return be.last();
  }
  // Drain until `path` holds `want` (a program's writes land a pump or two after it is queued).
  void drain_until(const std::string &path, const std::string &want) {
    for (int i = 0; i < 20 && get(app, path) != want; ++i)
      rt.drain();
  }
};

HeatmapStop stop(double at, float r, float g, float b) {
  HeatmapStop s;
  s.at = at;
  s.rgb[0] = r;
  s.rgb[1] = g;
  s.rgb[2] = b;
  return s;
}

// Red at 0, yellow at 10, green at 20.
std::vector<HeatmapStop> traffic() {
  return {stop(0, 1, 0, 0), stop(10, 1, 1, 0), stop(20, 0, 1, 0)};
}

// A 2 x 3 matrix under doc.m: values, a text, a mark, a tip, labels; (1, 2) has no value.
void fill(cvc::app &app) {
  put(app, "doc.m.rows", "2");
  put(app, "doc.m.cols", "3");
  put(app, "doc.m.cells.0.0.v", "0");
  put(app, "doc.m.cells.0.1.v", "5");
  put(app, "doc.m.cells.0.1.text", "5!");
  put(app, "doc.m.cells.0.2.v", "20");
  put(app, "doc.m.cells.0.2.mark", "1");
  put(app, "doc.m.cells.1.0.v", "-4");
  put(app, "doc.m.cells.1.0.tip", "below the ramp");
  put(app, "doc.m.cells.1.1", "15"); // a cell with no children: its own value
  put(app, "doc.m.row_labels.0", "a");
  put(app, "doc.m.row_labels.1", "b");
  put(app, "doc.m.col_labels", "x, y, z");
}

bool has_warning(const std::vector<std::string> &warns, const std::string &needle) {
  for (const std::string &w : warns)
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

} // namespace

// Between two stops a value's colour is mixed in proportion; outside the ramp it is the end
// stop's; a value that is not a number has none.
TEST(AriadneHeatmap, TheRampMixesAndClamps) {
  const std::vector<HeatmapStop> ramp = traffic();
  float rgb[3];
  ASSERT_TRUE(heatmap_color(ramp, 5.0, rgb));
  EXPECT_FLOAT_EQ(rgb[0], 1.0f);
  EXPECT_FLOAT_EQ(rgb[1], 0.5f);
  EXPECT_FLOAT_EQ(rgb[2], 0.0f);
  ASSERT_TRUE(heatmap_color(ramp, 15.0, rgb));
  EXPECT_FLOAT_EQ(rgb[0], 0.5f);
  EXPECT_FLOAT_EQ(rgb[1], 1.0f);
  ASSERT_TRUE(heatmap_color(ramp, -100.0, rgb));
  EXPECT_FLOAT_EQ(rgb[0], 1.0f);
  EXPECT_FLOAT_EQ(rgb[1], 0.0f) << "below the first stop: the first stop's colour";
  ASSERT_TRUE(heatmap_color(ramp, 1e9, rgb));
  EXPECT_FLOAT_EQ(rgb[0], 0.0f);
  EXPECT_FLOAT_EQ(rgb[1], 1.0f) << "above the last: the last's";
  ASSERT_TRUE(heatmap_color(ramp, 10.0, rgb));
  EXPECT_FLOAT_EQ(rgb[1], 1.0f) << "on a stop: its colour";
  rgb[0] = 0.25f;
  EXPECT_FALSE(heatmap_color(ramp, std::numeric_limits<double>::quiet_NaN(), rgb));
  EXPECT_FLOAT_EQ(rgb[0], 0.25f) << "untouched";
  ASSERT_TRUE(heatmap_color({}, 3.0, rgb));
  EXPECT_FLOAT_EQ(rgb[0], 0.5f) << "no stops: one grey";
}

// The subtree's cells, row after row: each value coloured from the ramp, its text, mark and tip;
// a cell's own value when it has no children; no value where there is none; both label forms.
TEST(AriadneHeatmap, ReadsTheSubtree) {
  Doc d(heatmap("Q", "m", traffic()));
  fill(d.app);
  const HeatmapSpec &s = d.frame();
  ASSERT_EQ(s.rows, 2);
  ASSERT_EQ(s.cols, 3);
  ASSERT_EQ(s.cells.size(), 6u);
  EXPECT_TRUE(s.cells[0].has_value);
  EXPECT_FLOAT_EQ(s.cells[0].rgb[0], 1.0f);
  EXPECT_FLOAT_EQ(s.cells[0].rgb[1], 0.0f);
  EXPECT_EQ(s.cells[1].text, "5!");
  EXPECT_FLOAT_EQ(s.cells[1].rgb[1], 0.5f);
  EXPECT_TRUE(s.cells[2].mark);
  EXPECT_FLOAT_EQ(s.cells[2].rgb[0], 0.0f);
  EXPECT_DOUBLE_EQ(s.cells[3].v, -4.0);
  EXPECT_FLOAT_EQ(s.cells[3].rgb[0], 1.0f) << "clamped to the first stop";
  EXPECT_TRUE(s.cells[4].has_value) << "the cell's own value";
  EXPECT_DOUBLE_EQ(s.cells[4].v, 15.0);
  EXPECT_FALSE(s.cells[5].has_value);
  EXPECT_EQ(s.row_labels, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(s.col_labels, (std::vector<std::string>{"x", "y", "z"}));
  EXPECT_FALSE(s.interactive) << "no pick or on_click";
}

// It reads the subtree again only after a write at or under it: not each frame, and not for a
// write elsewhere. A ramp change recolours the cells.
TEST(AriadneHeatmap, ReadsAgainOnlyAfterAWrite) {
  Doc d(heatmap("Q", "m", traffic()));
  fill(d.app);
  const std::uint64_t r0 = d.frame().revision;
  EXPECT_EQ(d.frame().revision, r0) << "nothing written: not read again";
  put(d.app, "doc.other", "1");
  put(d.app, "doc.m2.cells", "1"); // a sibling whose name starts the same
  EXPECT_EQ(d.frame().revision, r0) << "a write elsewhere";
  put(d.app, "doc.m.cells.0.0.v", "20");
  const HeatmapSpec &s = d.frame();
  EXPECT_EQ(s.revision, r0 + 1);
  EXPECT_FLOAT_EQ(s.cells[0].rgb[0], 0.0f) << "the new value's colour";
  EXPECT_EQ(d.frame().revision, r0 + 1);
  put(d.app, "doc.m.rows", "1");
  EXPECT_EQ(d.frame().cells.size(), 3u);

  Doc e(heatmap("Q", "m", traffic()));
  fill(e.app);
  e.frame();
  e.rt.set_root(group({window("W", {heatmap("Q", "m", {stop(0, 0, 0, 1)})})}));
  const HeatmapSpec &t = e.frame();
  EXPECT_FLOAT_EQ(t.cells[0].rgb[2], 1.0f) << "a new ramp recolours";
}

// A subtree reached through a link is read every frame: a write to the link's target is
// signalled under the target's path.
TEST(AriadneHeatmap, ThroughALinkItIsReadEachFrame) {
  Doc d(heatmap("Q", "m", traffic()));
  put(d.app, "data.rows", "1");
  put(d.app, "data.cols", "1");
  put(d.app, "data.cells.0.0.v", "0");
  cvc::state &root = cvc::state::instance(d.app);
  root("doc.m").linkTo("data", cvc::state::link_mode::transparent);
  const HeatmapSpec &s = d.frame();
  ASSERT_EQ(s.cells.size(), 1u);
  EXPECT_FLOAT_EQ(s.cells[0].rgb[1], 0.0f);
  put(d.app, "data.cells.0.0.v", "20");
  EXPECT_FLOAT_EQ(d.frame().cells[0].rgb[1], 1.0f) << "the target's write is seen";
}

// A click writes the cell to the pick's key (row, col, then "row,col"), the picked cell is
// handed back to be outlined, and on: runs after with the cell in `event`.
TEST(AriadneHeatmap, APickWritesTheCellAndRunsOn) {
  Widget w = heatmap("Q", "m", traffic());
  w.pick = "sel";
  w.on =
      R"((state-set "seen" (string.join (list (str (get-attr event "row")) (str (get-attr event "col")) (str (get-attr event "v"))) " ")))";
  Doc d(std::move(w));
  fill(d.app);
  EXPECT_TRUE(d.frame().interactive);
  EXPECT_EQ(d.be.last().sel_row, -1);
  d.be.report.clicked = true;
  d.be.report.click_row = 0;
  d.be.report.click_col = 2;
  d.frame();
  d.be.report = {};
  EXPECT_EQ(get(d.app, "doc.sel.row"), "0");
  EXPECT_EQ(get(d.app, "doc.sel.col"), "2");
  EXPECT_EQ(get(d.app, "doc.sel"), "0,2");
  const HeatmapSpec &s = d.frame();
  EXPECT_EQ(s.sel_row, 0);
  EXPECT_EQ(s.sel_col, 2);
  d.drain_until("doc.seen", "0 2 20");
  EXPECT_EQ(get(d.app, "doc.seen"), "0 2 20");
}

// on_click and on_hover get the cell as `event`: row and col as numbers, v (nil with no value),
// text, row_label, col_label and mark. The generic pointer handlers do not also fire.
TEST(AriadneHeatmap, HandlersGetTheCell) {
  Widget w = heatmap("Q", "m", traffic());
  w.on_click =
      R"((state-set "clicked" (string.join (list (str (+ (get-attr event "row") 10)) (get-attr event "row_label") (get-attr event "col_label") (get-attr event "text")) " ")))";
  w.on_hover = R"((state-set "hover" (if (get-attr event "v") (str (get-attr event "v")) "none")))";
  Doc d(std::move(w));
  fill(d.app);
  d.be.report.hovered = true;
  d.be.report.row = 1;
  d.be.report.col = 2;
  d.frame();
  d.drain_until("doc.hover", "none");
  EXPECT_EQ(get(d.app, "doc.hover"), "none") << "(1, 2) has no value";
  d.be.report.row = 0;
  d.be.report.col = 1;
  d.be.report.clicked = true;
  d.be.report.click_row = 0;
  d.be.report.click_col = 1;
  d.frame();
  d.be.report = {};
  d.drain_until("doc.clicked", "10 a y 5!");
  EXPECT_EQ(get(d.app, "doc.clicked"), "10 a y 5!");
  EXPECT_EQ(get(d.app, "doc.hover"), "5");
}

// The hovered cell's tooltip: its own tip, else nothing; the document's tooltip: with the cell's
// tokens filled in, or computed after them.
TEST(AriadneHeatmap, TheTooltipIsTheHoveredCells) {
  Doc d(heatmap("Q", "m", traffic()));
  fill(d.app);
  d.be.report.hovered = true;
  d.be.report.row = 1;
  d.be.report.col = 0;
  d.frame();
  EXPECT_EQ(d.be.tips, (std::vector<std::string>{"below the ramp"}));
  d.be.report.col = 1;
  d.frame();
  EXPECT_TRUE(d.be.tips.empty()) << "no tip";

  Widget w = heatmap("Q", "m", traffic());
  w.tooltip = "{row_label} to {col_label}: {v} ({row},{col}) mark={mark} [{text}]";
  Doc t(std::move(w));
  fill(t.app);
  t.be.report.hovered = true;
  t.be.report.row = 0;
  t.be.report.col = 2;
  t.frame();
  EXPECT_EQ(t.be.tips, (std::vector<std::string>{"a to z: 20 (0,2) mark=1 []"}));

  Widget c = heatmap("Q", "m", traffic());
  c.tooltip = R"((string.join (list "pair" (state-get "m.cells.{row}.{col}.v")) " "))";
  Doc e(std::move(c));
  fill(e.app);
  e.be.report.hovered = true;
  e.be.report.row = 0;
  e.be.report.col = 1;
  e.frame();
  EXPECT_EQ(e.be.tips, (std::vector<std::string>{"pair 5"}));
}

// The default heatmap (every backend without one): the label, the column labels, a line for each
// row with each cell's text (else its value), * after a mark and . for no value, then the stops.
TEST(AriadneHeatmap, TheDefaultIsATextGrid) {
  Widget w = heatmap("Quality", "m", traffic());
  w.heatmap.legend = HeatmapLegend::Bottom;
  Doc d(std::move(w));
  d.be.use_default = true;
  fill(d.app);
  d.frame();
  const std::vector<std::string> expected = {
      "Quality",
      "   x  y   z",
      "a  0 5! 20*",
      "b -4 15   .",
      "legend: 0 #ff0000, 10 #ffff00, 20 #00ff00",
  };
  EXPECT_EQ(d.be.lines, expected);
}

// has? sees the built-in type, so a document can fall back where a host predates it.
TEST(AriadneHeatmap, HasHeatmap) {
  cvc::app app;
  Runtime rt(app, "");
  EXPECT_TRUE(rt.has("heatmap"));
  EXPECT_TRUE(is_builtin_widget_type("heatmap"));
}

// The loader's keys reach the widget and the spec: the ramp (sorted; "#rrggbb", "#rgb" and
// [r, g, b]), the layout, the labels, the legend, the mark's colour, the pick, on: and the
// per-cell handlers and tooltip; the short form and `type: heatmap` alike.
TEST(AriadneHeatmap, FromADocument) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  LoadResult r = load_string(app, R"(
meta: { name: T, min_libcvc: 3.4.0 }
windows:
  - window: W
    children:
      - heatmap: Links
        bind: links
        ramp: [[25, "#27ae60"], [0, "#c0392b"], [12.5, [0.9, 0.6, 0.2]]]
        cell_size: 20
        gap: 1
        labels: {rows: true, cols: false}
        legend: right
        mark_color: "#fff"
        pick: sel
        on: (state-set "picked" "1")
        on_click: (state-set "clicked" "1")
        tooltip: "{row_label}: {v}"
      - type: heatmap
        title: Plain
        bind: plain
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.warnings.empty()) << (r.warnings.empty() ? "" : r.warnings[0]);
  HeatBackend be;
  Runtime rt(app, "doc");
  rt.set_backend(&be);
  rt.set_root(r.root);
  put(app, "doc.links.rows", "1");
  put(app, "doc.links.cols", "1");
  put(app, "doc.links.cells.0.0.v", "12.5");
  put(app, "doc.links.row_labels", "u1");
  put(app, "doc.links.col_labels", "u1");
  rt.render();
  ASSERT_EQ(be.specs.size(), 2u);
  const HeatmapSpec &s = be.specs[0];
  ASSERT_EQ(s.ramp.size(), 3u);
  EXPECT_DOUBLE_EQ(s.ramp[0].at, 0.0);
  EXPECT_DOUBLE_EQ(s.ramp[1].at, 12.5);
  EXPECT_DOUBLE_EQ(s.ramp[2].at, 25.0);
  EXPECT_NEAR(s.ramp[0].rgb[0], 0xc0 / 255.0f, 1e-6);
  EXPECT_NEAR(s.cells[0].rgb[0], 0.9f, 1e-6) << "on the middle stop";
  EXPECT_FLOAT_EQ(s.cell_size, 20.0f);
  EXPECT_FLOAT_EQ(s.gap, 1.0f);
  EXPECT_EQ(s.row_labels, (std::vector<std::string>{"u1"}));
  EXPECT_TRUE(s.col_labels.empty()) << "labels: {cols: false}";
  EXPECT_EQ(s.legend, HeatmapLegend::Right);
  EXPECT_FLOAT_EQ(s.mark_rgb[0], 1.0f);
  EXPECT_TRUE(s.interactive);
  const HeatmapSpec &p = be.specs[1];
  EXPECT_TRUE(p.ramp.empty());
  EXPECT_EQ(p.legend, HeatmapLegend::Off);
  EXPECT_FLOAT_EQ(p.cell_size, 28.0f) << "the defaults";
  EXPECT_FALSE(p.interactive);
}

// What the loader warns about: no bind, a stop it cannot read, a bad size, an unknown legend or
// mark colour, and on: with no pick.
TEST(AriadneHeatmap, TheLoaderWarns) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  cvc::app app;
  LoadResult r = load_string(app, R"(
meta: { name: T, min_libcvc: 3.4.0 }
windows:
  - window: W
    children:
      - heatmap: A
        ramp: [[0, "#c0392b"], [1, "red"], "x"]
        cell_size: 0
        gap: -1
        legend: top
        mark_color: "#12"
        on: (state-set "x" "1")
)");
  ASSERT_TRUE(r.ok) << r.error;
  const std::vector<std::string> &w = r.warnings;
  EXPECT_TRUE(has_warning(w, "heatmap 'A' has no bind:"));
  EXPECT_TRUE(has_warning(w, "has a ramp stop that is not"));
  EXPECT_TRUE(has_warning(w, "cell_size must be > 0"));
  EXPECT_TRUE(has_warning(w, "gap must be >= 0"));
  EXPECT_TRUE(has_warning(w, "legend: 'top' is not bottom, right or none"));
  EXPECT_TRUE(has_warning(w, "mark_color: is not"));
  EXPECT_TRUE(has_warning(w, "has on: but no pick:"));
}

// ---- FTXUI: coloured cells in a terminal ------------------------------------------------------

namespace {

std::string sgr_bg(int r, int g, int b) {
  return "48;2;" + std::to_string(r) + ";" + std::to_string(g) + ";" + std::to_string(b);
}

struct Term {
  cvc::app app;
  FtxuiBackend be;
  Runtime rt{app, ""};
  explicit Term(Widget w, FtxuiBackend::Options opts = {}) : be(opts) {
    rt.set_backend(&be);
    rt.set_root(group({std::move(w)}));
  }
  std::string draw(bool ansi = false) { return be.render_to_string(rt, 40, 10, ansi); }
};

// The cell (column, row) where `needle` first appears on a plain screen.
bool find_cell(const std::string &screen, const std::string &needle, int &x, int &y) {
  int row = 0;
  for (std::size_t start = 0; start < screen.size(); ++row) {
    const std::size_t nl = screen.find('\n', start);
    const std::string line = screen.substr(start, nl == std::string::npos ? nl : nl - start);
    if (const std::size_t at = line.find(needle); at != std::string::npos) {
      int col = 0;
      for (std::size_t i = 0; i < at; ++i)
        col += (static_cast<unsigned char>(line[i]) & 0xC0) != 0x80;
      x = col;
      y = row;
      return true;
    }
    if (nl == std::string::npos)
      break;
    start = nl + 1;
  }
  return false;
}

// The screen's lines, each without its trailing spaces, and without the blank lines at the end.
std::vector<std::string> screen_lines(const std::string &screen) {
  std::vector<std::string> out;
  for (std::size_t i = 0; i <= screen.size();) {
    const std::size_t nl = std::min(screen.find('\n', i), screen.size());
    std::string line = screen.substr(i, nl - i);
    while (!line.empty() && line.back() == ' ')
      line.pop_back();
    out.push_back(line);
    i = nl + 1;
  }
  while (!out.empty() && out.back().empty())
    out.pop_back();
  return out;
}

void fill_term(cvc::app &app) {
  put(app, "m.rows", "2");
  put(app, "m.cols", "2");
  put(app, "m.cells.0.0.v", "0");
  put(app, "m.cells.0.1.v", "20");
  put(app, "m.cells.0.1.text", "ok");
  put(app, "m.cells.0.1.mark", "1");
  put(app, "m.cells.1.0.v", "10");
  put(app, "m.cells.1.0.tip", "half way");
  put(app, "m.row_labels", "a,b");
  put(app, "m.col_labels", "x,y");
}

} // namespace

// The snapshot: the label, the column labels over their cells, a row label before each row, each
// cell's text centred (a mark's glyph at its end), · for no value, and the legend under it.
TEST(AriadneHeatmap, FtxuiDrawsColouredCells) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  Widget w = heatmap("Links", "m", traffic());
  w.heatmap.legend = HeatmapLegend::Bottom;
  Term t(std::move(w));
  fill_term(t.app);
  const std::vector<std::string> got = screen_lines(t.draw());
  // The legend's bar is spaces on colour, so it is a blank line here.
  const std::vector<std::string> want = {
      "Links", "   x   y", "a     ok\xE2\x97\xA5", "b      \xC2\xB7", "", "  0      10     20",
  };
  EXPECT_EQ(got, want);

  // The colours: each cell's ground is the ramp's at its value (red, green, yellow), the text on
  // the green cell is black, and the bar runs red to green.
  // Printed as for a kitty terminal, which has true colour, so FTXUI keeps the RGB colours rather
  // than the palette's nearest (what it does on a terminal it finds has fewer).
  FtxuiBackend::Options opts;
  opts.image_color = FtxuiBackend::Options::ImageColor::Kitty;
  Widget w2 = heatmap("Links", "m", traffic());
  w2.heatmap.legend = HeatmapLegend::Bottom;
  Term k(std::move(w2), opts);
  fill_term(k.app);
  const std::string ansi = k.draw(/*ansi=*/true);
  EXPECT_NE(ansi.find(sgr_bg(255, 0, 0)), std::string::npos) << "0: red";
  EXPECT_NE(ansi.find(sgr_bg(0, 255, 0)), std::string::npos) << "20: green";
  EXPECT_NE(ansi.find(sgr_bg(255, 255, 0)), std::string::npos) << "10: yellow";
  EXPECT_NE(ansi.find("38;2;0;0;0"), std::string::npos) << "black text on green";
  EXPECT_NE(ansi.find("38;2;255;255;255"), std::string::npos) << "the mark, white";
}

// The pointer's cell is the hovered one (its tip on the tooltip line); a click on it writes the
// pick; the arrows move the keyboard's cell and Enter clicks it.
TEST(AriadneHeatmap, FtxuiPointerAndKeys) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  Widget w = heatmap("Links", "m", traffic());
  w.pick = "sel";
  Term t(std::move(w));
  fill_term(t.app);
  const std::string screen = t.draw();
  int x = 0, y = 0;
  ASSERT_TRUE(find_cell(screen, "b ", x, y)) << screen;
  // (1, 0): the first cell after "b ".
  t.be.feed(t.rt, TermEvent::mouse_event(TermEvent::Mouse::Move, x + 2, y));
  t.draw();
  EXPECT_EQ(t.be.tooltip(), "half way");
  t.be.feed(t.rt, TermEvent::mouse_event(TermEvent::Mouse::Press, x + 2, y));
  t.be.feed(t.rt, TermEvent::mouse_event(TermEvent::Mouse::Release, x + 2, y));
  t.draw();
  EXPECT_EQ(get(t.app, "sel"), "1,0");
  // The keyboard: the cursor is where the click left it; Up then Right is (0, 1).
  t.be.feed(t.rt, TermEvent::mouse_event(TermEvent::Mouse::Move, 39, 9)); // off it
  t.be.feed(t.rt, TermEvent::key_event("Up"));
  t.be.feed(t.rt, TermEvent::key_event("Right"));
  t.be.feed(t.rt, TermEvent::key_event("Return"));
  t.draw();
  EXPECT_EQ(get(t.app, "sel"), "0,1");
}
