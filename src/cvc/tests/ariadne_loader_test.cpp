// Comprehensive tests for the Ariadne .ari loader (cvc::ariadne): widget parsing,
// the meta/min_libcvc gate, semantic validation, and §3.0.3b size/layout/frame.
//
// Parsing tests are skipped when the build has no yaml-cpp (the loader is then a
// stub); the version/schema surface is tested unconditionally.

#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>

#include <gtest/gtest.h>

#include <functional>
#include <string>

using namespace cvc::ariadne;

namespace {

// Depth-first search for the first widget of a kind, optionally matching a label.
const Widget *find(const Widget &w, Kind k, const std::string &label = std::string()) {
  if (w.kind == k && (label.empty() || w.label == label))
    return &w;
  for (const Widget &c : w.children)
    if (const Widget *r = find(c, k, label))
      return r;
  return nullptr;
}

int count(const Widget &w, Kind k) {
  int n = (w.kind == k) ? 1 : 0;
  for (const Widget &c : w.children)
    n += count(c, k);
  return n;
}

bool has_warning(const LoadResult &r, const std::string &needle) {
  for (const std::string &w : r.warnings)
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

#define SKIP_WITHOUT_YAML()                                                                          \
  do {                                                                                               \
    if (!have_yaml())                                                                                \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                               \
  } while (0)

} // namespace

// ---- version + schema surface (no yaml needed) ----------------------------

TEST(AriadneVersion, AtLeast) {
  EXPECT_TRUE(version_at_least("3.4.0", "3.4.0"));
  EXPECT_TRUE(version_at_least("3.4.0", "3.3.9"));
  EXPECT_TRUE(version_at_least("3.4.0", "3.4"));
  EXPECT_TRUE(version_at_least("3.4.1", "3.4.0"));
  EXPECT_TRUE(version_at_least("4.0.0", "3.9.9"));
  EXPECT_FALSE(version_at_least("3.4.0", "3.4.1"));
  EXPECT_FALSE(version_at_least("3.4.0", "9.0.0"));
  EXPECT_FALSE(version_at_least("3.4.0", "3.5"));
  // Tolerates a leading 'v' and a pre-release/build suffix.
  EXPECT_TRUE(version_at_least("v3.4.0", "3.4.0"));
  EXPECT_TRUE(version_at_least("3.4.0-rc1", "3.4.0"));
}

TEST(AriadneVersion, LibcvcAndSchema) {
  EXPECT_FALSE(libcvc_version().empty());
  EXPECT_NE(libcvc_version(), "3.0.0"); // the stale fallback is gone (§17/config.h fix)
  EXPECT_NE(ari_schema_json().find("min_libcvc"), std::string::npos);
  EXPECT_NE(ari_schema_json().find("$schema"), std::string::npos);
}

// ---- widget parsing --------------------------------------------------------

TEST(AriadneLoader, FullDocumentStructure) {
  SKIP_WITHOUT_YAML();
  const char *doc = R"(
meta: { name: T, min_libcvc: "3.4.0" }
menubar:
  - menu: Sim
    items:
      - menu_item: Paused
        bind: demo.paused
      - separator
      - menu_item: Quit
        on: quit
windows:
  - window: Controls
    id: ctrl
    children:
      - text: "hello"
      - slider_int: N
        bind: demo.n
        lo: 1
        hi: 100
        def: 10
      - combo: Belief
        bind: demo.b
        options: [a, b, c]
        default: b
      - checkbox: Wire
        bind: demo.w
      - button: Go
        on: go
)";
  LoadResult r = load_string(doc);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.meta.name, "T");
  EXPECT_EQ(r.meta.min_libcvc, "3.4.0");
  EXPECT_EQ(count(r.root, Kind::Menubar), 1);
  EXPECT_EQ(count(r.root, Kind::Menu), 1);
  EXPECT_EQ(count(r.root, Kind::MenuItemToggle), 1);
  EXPECT_EQ(count(r.root, Kind::MenuItemAction), 1);
  EXPECT_EQ(count(r.root, Kind::Separator), 1);
  EXPECT_EQ(count(r.root, Kind::Window), 1);
  EXPECT_EQ(count(r.root, Kind::SliderInt), 1);
  EXPECT_EQ(count(r.root, Kind::Combo), 1);
  EXPECT_EQ(count(r.root, Kind::Checkbox), 1);
  EXPECT_EQ(count(r.root, Kind::Button), 1);

  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  EXPECT_EQ(win->id, "ctrl");

  const Widget *si = find(r.root, Kind::SliderInt);
  ASSERT_NE(si, nullptr);
  EXPECT_EQ(si->bind, "demo.n");
  EXPECT_EQ(si->ilo, 1);
  EXPECT_EQ(si->ihi, 100);
  EXPECT_EQ(si->idef, 10);

  const Widget *cb = find(r.root, Kind::Combo);
  ASSERT_NE(cb, nullptr);
  ASSERT_EQ(cb->options.size(), 3u);
  EXPECT_EQ(cb->options[1], "b");
  EXPECT_EQ(cb->sdef, "b");

  const Widget *tg = find(r.root, Kind::MenuItemToggle);
  ASSERT_NE(tg, nullptr);
  EXPECT_EQ(tg->bind, "demo.paused");

  const Widget *bt = find(r.root, Kind::Button);
  ASSERT_NE(bt, nullptr);
  EXPECT_EQ(bt->on, "go");
}

TEST(AriadneLoader, BareScalarsAndLiteralVsBoundText) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    children:
      - separator
      - text: "a literal caption"
      - text: "value ="
        bind: demo.v
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(count(r.root, Kind::Separator), 1);
  // Two Text widgets: one literal, one bound.
  EXPECT_EQ(count(r.root, Kind::Text), 2);
  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  bool saw_literal = false, saw_bound = false;
  for (const Widget &c : win->children) {
    if (c.kind == Kind::Text && c.literal_text)
      saw_literal = true;
    if (c.kind == Kind::Text && !c.literal_text && c.bind == "demo.v")
      saw_bound = true;
  }
  EXPECT_TRUE(saw_literal);
  EXPECT_TRUE(saw_bound);
}

// ---- the min_libcvc gate ---------------------------------------------------

TEST(AriadneLoaderGate, Passes) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("meta: { min_libcvc: \"0.0.1\" }\nwindows: []\n");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.error.empty());
}

TEST(AriadneLoaderGate, FailsWhenTooNew) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("meta: { min_libcvc: \"999.0.0\" }\nwindows: []\n");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("requires libcvc"), std::string::npos);
  // A failed gate never builds a tree.
  EXPECT_TRUE(r.root.children.empty());
}

TEST(AriadneLoaderGate, MissingMinLibcvcIsWarningNotFatal) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows: [ { window: W, children: [] } ]\n");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "min_libcvc"));
}

// ---- semantic validation (Layer 3) ----------------------------------------

TEST(AriadneLoaderValidation, Warnings) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: Bad
    children:
      - slider_int: NoBind
        lo: 10
        hi: 5
      - combo: Empty
        bind: demo.c
      - button: Dud
      - frobnicate: Whatsit
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "no bind"));          // slider with no bind
  EXPECT_TRUE(has_warning(r, "lo >= hi"));         // bad slider range
  EXPECT_TRUE(has_warning(r, "no options"));       // combo with no options
  EXPECT_TRUE(has_warning(r, "no on: action"));    // button with no action
  EXPECT_TRUE(has_warning(r, "unrecognized widget key")); // frobnicate
}

TEST(AriadneLoaderValidation, UnknownExplicitType) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
root:
  - type: hologram
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "unknown widget type"));
}

// ---- errors + forward-compat ----------------------------------------------

TEST(AriadneLoaderErrors, MalformedYaml) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows: [ { window: W,,, ] : bad");
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}

TEST(AriadneLoaderErrors, EmptyDocLoads) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.root.children.empty());
}

TEST(AriadneLoaderErrors, UnknownTopLevelKeysIgnored) {
  SKIP_WITHOUT_YAML();
  // meta carries an unknown field; a future top-level key is ignored (forward-compat).
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0", future_field: 42 }
some_future_section: { a: 1 }
windows: [ { window: W, children: [] } ]
)");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_EQ(count(r.root, Kind::Window), 1);
}

// ---- §3.0.3b size / layout / frame ----------------------------------------

TEST(AriadneLayout, WindowSizePercentPxAutoAndMinMax) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: Inspector
    size: { hint: ["50%", "100%"], min: [280, 0], max: ["80%", 900] }
    frame: { border: 2 }
    children: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->size.w.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.w.value, 50.0f);
  EXPECT_EQ(w->size.h.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.h.value, 100.0f);
  EXPECT_EQ(w->size.min_w.unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->size.min_w.value, 280.0f);
  EXPECT_EQ(w->size.min_h.unit, Unit::Auto); // 0 with a min sequence -> px 0? see note
  EXPECT_EQ(w->size.max_w.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.max_w.value, 80.0f);
  EXPECT_EQ(w->size.max_h.unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->size.max_h.value, 900.0f);
  EXPECT_FLOAT_EQ(w->frame_border, 2.0f);
}

TEST(AriadneLayout, SizeSequenceAndFractionPercent) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    size: [0.25, 300]
    children: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->size.w.unit, Unit::Percent); // 0.25 fraction -> 25%
  EXPECT_FLOAT_EQ(w->size.w.value, 25.0f);
  EXPECT_EQ(w->size.h.unit, Unit::Px); // 300 -> px
  EXPECT_FLOAT_EQ(w->size.h.value, 300.0f);
}

TEST(AriadneLayout, GridColTracksBordersResizable) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    layout:
      kind: grid
      col_widths: [200, "auto", "30%"]
      resizable: true
      borders: { show: inner, color: [0.3, 0.3, 0.35, 1.0] }
    children: [ { text: "a" }, { text: "b" }, { text: "c" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->layout.kind, LayoutKind::Grid);
  ASSERT_EQ(w->layout.col_widths.size(), 3u);
  EXPECT_EQ(w->layout.col_widths[0].unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->layout.col_widths[0].value, 200.0f);
  EXPECT_EQ(w->layout.col_widths[1].unit, Unit::Auto);
  EXPECT_EQ(w->layout.col_widths[2].unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->layout.col_widths[2].value, 30.0f);
  EXPECT_TRUE(w->layout.resizable);
  EXPECT_EQ(w->layout.borders, BorderShow::Inner);
  EXPECT_TRUE(w->layout.has_border_color);
  EXPECT_FLOAT_EQ(w->layout.border_color[3], 1.0f);
  EXPECT_FALSE(w->layout.is_row_split()); // no row tracks
}

TEST(AriadneLayout, RowTracksAndRowSplit) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    layout:
      kind: grid
      row_heights: ["30%", 120, "auto"]
      resizable: true
    children: [ { text: "top" }, { text: "mid" }, { text: "bot" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  ASSERT_EQ(w->layout.row_heights.size(), 3u);
  EXPECT_EQ(w->layout.row_heights[0].unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->layout.row_heights[0].value, 30.0f);
  EXPECT_EQ(w->layout.row_heights[1].unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->layout.row_heights[1].value, 120.0f);
  EXPECT_EQ(w->layout.row_heights[2].unit, Unit::Auto);
  // resizable + row tracks -> a row split-pane layout (§3.0.3b increment 2).
  EXPECT_TRUE(w->layout.is_row_split());
  EXPECT_TRUE(w->layout.is_set());
}

TEST(AriadneLayout, GroupCanBeAGrid) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
root:
  - type: group
    layout: { kind: grid, col_widths: ["50%", "50%"] }
    children: [ { text: "l" }, { text: "r" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *g = find(r.root, Kind::Group, "");
  // The document root is itself a Group; find a nested grid Group.
  bool found_grid = false;
  std::function<void(const Widget &)> walk = [&](const Widget &w) {
    if (w.kind == Kind::Group && w.layout.kind == LayoutKind::Grid &&
        w.layout.col_widths.size() == 2)
      found_grid = true;
    for (const Widget &c : w.children)
      walk(c);
  };
  walk(r.root);
  (void)g;
  EXPECT_TRUE(found_grid);
}

TEST(AriadneLayout, NoLayoutMeansPlainVertical) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows: [ { window: W, children: [ { text: "x" } ] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->layout.kind, LayoutKind::Vertical);
  EXPECT_FALSE(w->layout.is_set());
  EXPECT_FALSE(w->size.any());
  EXPECT_LT(w->frame_border, 0.0f); // unset
}
