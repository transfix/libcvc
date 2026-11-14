// Ariadne chart: (Kind::Chart) through the real Runtime walk, with a backend that records the
// ChartSpec it is handed and can report a pick. Covers each series' own x, gaps, the log axis
// dropping what it cannot show, bands, markers, the cursor, a pick's write and its on:, the parse
// cache, `series:` entries over plain comma text, the loader's keys, and the default text chart
// every backend without one draws.

#include <cmath>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;

namespace {

// The least backend the walk needs, recording each chart's spec. `use_default_chart` draws it with
// Backend::chart (the text chart) instead, so its lines land in `lines`.
struct ChartBackend : Backend {
  std::vector<std::pair<std::string, ChartSpec>> charts;
  std::vector<std::string> lines;
  ChartEvent pick_ret{};
  bool use_default_chart = false;

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
  const double *first_y = nullptr; // the first series' y as handed over (the cache test)
  ChartEvent chart(const char *label, const ChartSpec &spec) override {
    charts.emplace_back(label ? label : "", spec);
    first_y = spec.series.empty() ? nullptr : spec.series[0].y.data();
    if (use_default_chart)
      return Backend::chart(label, spec);
    const ChartEvent e = pick_ret;
    pick_ret = {};
    return e;
  }
  const ChartSpec &last() const { return charts.back().second; }
};

void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}
std::string get(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// A Runtime over a ChartBackend. Member order: the backend outlives the Runtime.
struct Doc {
  cvc::app app;
  ChartBackend be;
  Runtime rt{app, "doc"};
  explicit Doc(Widget w) {
    rt.set_backend(&be);
    rt.set_root(group({window("W", {std::move(w)})}));
  }
  const ChartSpec &frame() {
    be.charts.clear();
    rt.render();
    EXPECT_EQ(be.charts.size(), 1u);
    return be.last();
  }
};

bool has_warning(Runtime &rt, const std::string &needle) {
  for (const std::string &w : rt.take_reactive_warnings())
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

} // namespace

// Each series of a plot subtree keeps its own x, so a curve sampled every 10 steps and one every
// 5 share the axis. The box spans both.
TEST(AriadneChart, EachSeriesHasItsOwnX) {
  Doc d(chart("Loss", "plot.loss"));
  put(d.app, "doc.plot.loss.s.0.label", "train");
  put(d.app, "doc.plot.loss.s.0.x", "0,10,20");
  put(d.app, "doc.plot.loss.s.0.y", "3,2,1");
  put(d.app, "doc.plot.loss.s.1.label", "held-out");
  put(d.app, "doc.plot.loss.s.1.x", "5,15");
  put(d.app, "doc.plot.loss.s.1.y", "4,5");
  const ChartSpec &s = d.frame();
  ASSERT_EQ(s.series.size(), 2u);
  EXPECT_EQ(s.series[0].label, "train");
  EXPECT_EQ(s.series[0].x, (std::vector<double>{0, 10, 20}));
  EXPECT_EQ(s.series[0].y, (std::vector<double>{3, 2, 1}));
  EXPECT_EQ(s.series[1].label, "held-out");
  EXPECT_EQ(s.series[1].x, (std::vector<double>{5, 15}));
  const ChartBounds b = chart_bounds(s);
  ASSERT_TRUE(b.ok);
  EXPECT_EQ(b.x0, 0.0);
  EXPECT_EQ(b.x1, 20.0);
  EXPECT_EQ(b.y0, 1.0);
  EXPECT_EQ(b.y1, 5.0);
  // Hiding the second series (a legend click) leaves it out of the box.
  const ChartBounds only_train = chart_bounds(s, {1, 0});
  EXPECT_EQ(only_train.x1, 20.0);
  EXPECT_EQ(only_train.y1, 3.0);
}

// An empty or unreadable item is a gap that keeps its place: the x stay aligned with the y.
TEST(AriadneChart, NaNAndEmptyItemsAreGaps) {
  Doc d(chart("C", "p"));
  put(d.app, "doc.p.s.0.y", "1,,3,nan,5,abc");
  const ChartSpec &s = d.frame();
  ASSERT_EQ(s.series.size(), 1u);
  const ChartSeries &c = s.series[0];
  ASSERT_EQ(c.y.size(), 6u);
  EXPECT_EQ(c.x, (std::vector<double>{0, 1, 2, 3, 4, 5})) << "no x: 0, 1, 2, ...";
  EXPECT_EQ(c.y[0], 1.0);
  EXPECT_TRUE(std::isnan(c.y[1]));
  EXPECT_EQ(c.y[2], 3.0);
  EXPECT_TRUE(std::isnan(c.y[3]));
  EXPECT_EQ(c.y[4], 5.0);
  EXPECT_TRUE(std::isnan(c.y[5]));
  // A point with no x is a gap too.
  put(d.app, "doc.p.s.0.x", "0,1,,3,4,5");
  const ChartSpec &s2 = d.frame();
  EXPECT_TRUE(std::isnan(s2.series[0].y[2]));
}

// On a log axis a y <= 0 is dropped (a gap), and so is a band point whose hi is <= 0; a band lo <=
// 0 stays for the backend to take to the axis' bottom. The box is in decades.
TEST(AriadneChart, LogYDropsValuesAtOrBelowZero) {
  Widget w = chart("L", "p");
  w.log_y = "1";
  Doc d(std::move(w));
  put(d.app, "doc.p.s.0.y", "1,0,-2,100");
  put(d.app, "doc.p.s.0.lo", "0.5,-1,-3,-5");
  put(d.app, "doc.p.s.0.hi", "2,1,-1,200");
  const ChartSpec &s = d.frame();
  EXPECT_TRUE(s.log_y);
  const ChartSeries &c = s.series[0];
  EXPECT_EQ(c.y[0], 1.0);
  EXPECT_TRUE(std::isnan(c.y[1]));
  EXPECT_TRUE(std::isnan(c.y[2]));
  EXPECT_EQ(c.y[3], 100.0);
  EXPECT_EQ(c.lo[1], -1.0) << "a lo <= 0 reaches the axis' bottom";
  EXPECT_TRUE(std::isnan(c.hi[2]) && std::isnan(c.lo[2])) << "a hi <= 0: no band there";
  const ChartBounds b = chart_bounds(s);
  ASSERT_TRUE(b.ok);
  EXPECT_DOUBLE_EQ(b.y0, std::log10(0.5));
  EXPECT_DOUBLE_EQ(b.y1, std::log10(200.0));
}

// log_y may name a state path, read each frame: a checkbox switches the axis, and the values it
// dropped come back.
TEST(AriadneChart, LogYCanFollowAStatePath) {
  Widget w = chart("L", "p");
  w.log_y = "view.log";
  Doc d(std::move(w));
  put(d.app, "doc.p.s.0.y", "0,1,10");
  put(d.app, "doc.view.log", "1");
  EXPECT_TRUE(d.frame().log_y);
  EXPECT_TRUE(std::isnan(d.be.last().series[0].y[0]));
  put(d.app, "doc.view.log", "0");
  const ChartSpec &s = d.frame();
  EXPECT_FALSE(s.log_y);
  EXPECT_EQ(s.series[0].y[0], 0.0);
}

// Markers come from the subtree's m.<j>: x, label and kind. An unknown kind is a note, warned of;
// one with no x is left out.
TEST(AriadneChart, MarkersAndTheirKinds) {
  Doc d(chart("C", "p"));
  put(d.app, "doc.p.s.0.y", "3,2,1");
  put(d.app, "doc.p.m.0.x", "1200");
  put(d.app, "doc.p.m.0.label", "held-out min");
  put(d.app, "doc.p.m.0.kind", "best");
  put(d.app, "doc.p.m.1.x", "300");
  put(d.app, "doc.p.m.1.kind", "elbow");
  put(d.app, "doc.p.m.2.x", "800");
  put(d.app, "doc.p.m.2.kind", "plateau");
  put(d.app, "doc.p.m.3.x", "900");
  put(d.app, "doc.p.m.3.kind", "bogus");
  put(d.app, "doc.p.m.4.label", "no x");
  const ChartSpec &s = d.frame();
  ASSERT_EQ(s.markers.size(), 4u);
  EXPECT_EQ(s.markers[0].x, 1200.0);
  EXPECT_EQ(s.markers[0].label, "held-out min");
  EXPECT_EQ(s.markers[0].kind, ChartMarkerKind::Best);
  EXPECT_EQ(s.markers[1].kind, ChartMarkerKind::Elbow);
  EXPECT_EQ(s.markers[2].kind, ChartMarkerKind::Plateau);
  EXPECT_EQ(s.markers[3].kind, ChartMarkerKind::Note);
  EXPECT_TRUE(has_warning(d.rt, "marker kind 'bogus'"));
  EXPECT_EQ(chart_bounds(s).x1, 1200.0) << "the markers widen the x range";
}

// A pick writes the x the backend reports to the pick path, then runs the chart's on:, which
// reads it.
TEST(AriadneChart, APickWritesTheXAndRunsOn) {
  Widget w = chart("C", "p");
  w.pick = "sel.step";
  w.on = R"((state-set "sel.seen" (state-get "sel.step")))";
  Doc d(std::move(w));
  put(d.app, "doc.p.s.0.x", "0,250,500");
  put(d.app, "doc.p.s.0.y", "1,2,3");
  EXPECT_TRUE(d.frame().pick);
  d.be.pick_ret = {true, 250.0};
  d.frame();
  EXPECT_EQ(get(d.app, "doc.sel.step"), "250");
  d.rt.drain();
  for (int i = 0; i < 20 && get(d.app, "doc.sel.seen") != "250"; ++i)
    d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.sel.seen"), "250");
  d.be.pick_ret = {true, 0.125};
  d.frame();
  EXPECT_EQ(get(d.app, "doc.sel.step"), "0.125") << "the shortest text that reads back";
}

// The cursor is the number at its path, each frame; none while it holds no number.
TEST(AriadneChart, CursorFollowsItsPath) {
  Widget w = chart("C", "p");
  w.cursor = "play.t";
  Doc d(std::move(w));
  put(d.app, "doc.p.s.0.y", "1,2");
  EXPECT_FALSE(d.frame().has_cursor);
  put(d.app, "doc.play.t", "0.5");
  const ChartSpec &s = d.frame();
  EXPECT_TRUE(s.has_cursor);
  EXPECT_EQ(s.cursor, 0.5);
  put(d.app, "doc.play.t", "x");
  EXPECT_FALSE(d.frame().has_cursor);
}

// `series:` entries name a key of comma text (host.set_series') or a series subtree; the
// document's label and style win over the subtree's own. They follow the bound subtree's series.
TEST(AriadneChart, SeriesEntriesAndStyles) {
  Widget w = chart("C", "p", {{"train", "Train", "c2 dashed"}, {"held", "", ""}});
  Doc d(std::move(w));
  put(d.app, "doc.p.s.0.y", "9");
  put(d.app, "doc.p.s.0.style", "#ff8000 points");
  put(d.app, "doc.train", "1,2,3");
  put(d.app, "doc.held.y", "4,5");
  put(d.app, "doc.held.label", "Held out");
  put(d.app, "doc.held.style", "dotted");
  const ChartSpec &s = d.frame();
  ASSERT_EQ(s.series.size(), 3u);
  EXPECT_TRUE(s.series[0].has_rgb);
  EXPECT_FLOAT_EQ(s.series[0].rgb[0], 1.0f);
  EXPECT_NEAR(s.series[0].rgb[1], 128.0f / 255.0f, 1e-6);
  EXPECT_EQ(s.series[0].line, ChartLine::Points);
  EXPECT_EQ(s.series[1].label, "Train");
  EXPECT_EQ(s.series[1].x, (std::vector<double>{0, 1, 2}));
  EXPECT_EQ(s.series[1].line, ChartLine::Dashed);
  EXPECT_EQ(s.series[1].color, 2);
  EXPECT_EQ(s.series[2].label, "Held out");
  EXPECT_EQ(s.series[2].line, ChartLine::Dotted);
  EXPECT_EQ(s.series[2].color, -1) << "no slot: its place in the list";
  put(d.app, "doc.train", "1,2,3,4");
  EXPECT_EQ(d.frame().series[1].y.size(), 4u);
}

// What the cache saves: a frame that changed nothing hands the backend the same parsed arrays; a
// write to one series re-parses it.
TEST(AriadneChart, SeriesAreParsedOnlyWhenTheirTextChanges) {
  Doc d(chart("C", "p"));
  put(d.app, "doc.p.s.0.y", "1,2,3");
  put(d.app, "doc.p.s.1.y", "4,5,6");
  d.frame();
  const double *y0 = d.be.first_y;
  d.frame();
  EXPECT_EQ(y0, d.be.first_y) << "nothing changed: the spec was not rebuilt";
  put(d.app, "doc.p.s.1.y", "4,5,6,7");
  const ChartSpec &s = d.frame();
  EXPECT_EQ(s.series[1].y.size(), 4u);
  EXPECT_EQ(s.series[0].y, (std::vector<double>{1, 2, 3}));
}

// Mismatched lengths draw the first of each and warn; a band of the wrong length is dropped.
TEST(AriadneChart, MismatchedLengthsWarn) {
  Doc d(chart("C", "p"));
  put(d.app, "doc.p.s.0.label", "a");
  put(d.app, "doc.p.s.0.x", "0,1,2,3");
  put(d.app, "doc.p.s.0.y", "5,6");
  put(d.app, "doc.p.s.0.lo", "1");
  put(d.app, "doc.p.s.0.hi", "9");
  const ChartSpec &s = d.frame();
  EXPECT_EQ(s.series[0].x.size(), 2u);
  EXPECT_TRUE(s.series[0].lo.empty());
  const std::vector<std::string> warns = d.rt.take_reactive_warnings();
  bool xw = false, bw = false;
  for (const std::string &m : warns) {
    xw = xw || m.find("series 'a' has 4 x for 2 y") != std::string::npos;
    bw = bw || m.find("no band drawn") != std::string::npos;
  }
  EXPECT_TRUE(xw && bw);
}

// The default chart (every backend without one: the terminals): a heading with the axis, a
// sparkline for each series binned on the shared x, gaps as spaces, then the markers and the
// cursor.
TEST(AriadneChart, TheDefaultIsATextChart) {
  Widget w = chart("Loss", "p");
  w.log_y = "1";
  w.x_label = "step";
  w.cursor = "t";
  Doc d(std::move(w));
  d.be.use_default_chart = true;
  put(d.app, "doc.p.s.0.label", "train");
  put(d.app, "doc.p.s.0.x", "0,100,200,300");
  put(d.app, "doc.p.s.0.y", "100,10,1,0");
  put(d.app, "doc.p.m.0.x", "200");
  put(d.app, "doc.p.m.0.kind", "best");
  put(d.app, "doc.p.m.0.label", "min");
  put(d.app, "doc.t", "150");
  d.frame();
  const std::vector<std::string> &l = d.be.lines;
  ASSERT_EQ(l.size(), 4u);
  // The 0 at step 300 is dropped on the log axis, so the box ends at 200: 100 is drawn at column
  // 0, 10 at 16 and 1 at 31.
  EXPECT_EQ(l[0], "Loss (log y; step 0..200)");
  EXPECT_EQ(l[1], "train: [#               =              _] 1 (1..100)");
  EXPECT_EQ(l[2], "best: x=200 min");
  EXPECT_EQ(l[3], "cursor: x=150");
}

// has? sees the built-in type, so a document can fall back to plot: where a host predates it.
TEST(AriadneChart, HasChart) {
  cvc::app app;
  Runtime rt(app, "");
  EXPECT_TRUE(rt.has("chart"));
  EXPECT_TRUE(is_builtin_widget_type("chart"));
}

// End to end from a document: the loader's keys reach the spec.
TEST(AriadneChart, FromADocument) {
  if (!have_yaml())
    GTEST_SKIP() << "libcvc built without yaml-cpp";
  LoadResult r = load_string(R"(
meta: { name: T, min_libcvc: 3.4.0 }
windows:
  - window: W
    children:
      - chart: Loss
        bind: learn.loss
        series:
          - {bind: extra, label: Extra, style: c3}
        log_y: true
        legend: right
        x_label: step
        cursor: play.t
        pick: play.seek
        height: 220
)");
  ASSERT_TRUE(r.ok) << r.error;
  cvc::app app;
  ChartBackend be;
  Runtime rt(app, "doc");
  rt.set_backend(&be);
  rt.set_root(r.root);
  put(app, "doc.learn.loss.s.0.y", "1,0.1");
  put(app, "doc.extra", "2,0.2");
  put(app, "doc.play.t", "1");
  rt.render();
  ASSERT_EQ(be.charts.size(), 1u);
  const ChartSpec &s = be.last();
  EXPECT_EQ(be.charts[0].first, "Loss");
  ASSERT_EQ(s.series.size(), 2u);
  EXPECT_EQ(s.series[1].label, "Extra");
  EXPECT_EQ(s.series[1].color, 3);
  EXPECT_TRUE(s.log_y);
  EXPECT_EQ(s.legend, ChartLegend::Right);
  EXPECT_EQ(s.x_label, "step");
  EXPECT_TRUE(s.has_cursor);
  EXPECT_TRUE(s.pick);
  EXPECT_FLOAT_EQ(s.height, 220.0f);
}
