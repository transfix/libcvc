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

// ---------------------------------------------------------------------------
// §9 scene binding — the `scene:` block parses into LoadResult::scene.
// ---------------------------------------------------------------------------

namespace {
const SceneNode *find_scene_node(const std::vector<SceneNode> &nodes, const std::string &id) {
  for (const SceneNode &n : nodes) {
    if (n.id == id)
      return &n;
    if (const SceneNode *r = find_scene_node(n.children, id))
      return r;
  }
  return nullptr;
}
} // namespace

TEST(AriadneScene, NoSceneBlockIsEmpty) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows: [ { window: W, children: [ { text: "x" } ] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(r.scene.any());
  EXPECT_TRUE(r.scene.nodes.empty());
  EXPECT_TRUE(r.scene.lights.empty());
}

TEST(AriadneScene, GeometryNodeSourceMaterialTransform) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: bunny
      type: geometry
      source: { file: bunny.obj }
      material: { color: [0.9, 0.1, 0.2], ambient: 0.3, diffuse: 0.7 }
      transform: { position: [1, 2, 3], rotation: [0, 90, 0], scale: [2, 2, 2] }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_TRUE(r.scene.any());
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.id, "bunny");
  EXPECT_EQ(n.type, "geometry");
  EXPECT_EQ(n.source_file, "bunny.obj");
  ASSERT_TRUE(n.has_material);
  EXPECT_FLOAT_EQ(n.color[0], 0.9f);
  EXPECT_FLOAT_EQ(n.color[2], 0.2f);
  EXPECT_FLOAT_EQ(n.ambient, 0.3f);
  EXPECT_FLOAT_EQ(n.diffuse, 0.7f);
  ASSERT_TRUE(n.has_transform);
  EXPECT_FLOAT_EQ(n.position[0], 1.0f);
  EXPECT_FLOAT_EQ(n.position[2], 3.0f);
  EXPECT_FLOAT_EQ(n.rotation[1], 90.0f);
  EXPECT_FLOAT_EQ(n.scale[0], 2.0f);
  EXPECT_TRUE(n.visible_default); // default visible, no bind
  EXPECT_TRUE(n.visible_bind.empty());
}

TEST(AriadneScene, ScalarScaleIsUniform) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: s
      source: { file: s.obj }
      transform: { scale: 3 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  ASSERT_TRUE(n.has_transform);
  EXPECT_FLOAT_EQ(n.scale[0], 3.0f);
  EXPECT_FLOAT_EQ(n.scale[1], 3.0f);
  EXPECT_FLOAT_EQ(n.scale[2], 3.0f);
  EXPECT_EQ(n.type, "geometry"); // default type
}

TEST(AriadneScene, VisibleLiteralVsBind) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: hidden
      source: { file: a.obj }
      visible: false
    - node: bound
      source: { file: b.obj }
      visible: demo.show_mesh
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 2u);
  const SceneNode *hidden = find_scene_node(r.scene.nodes, "hidden");
  ASSERT_NE(hidden, nullptr);
  EXPECT_FALSE(hidden->visible_default);
  EXPECT_TRUE(hidden->visible_bind.empty());
  const SceneNode *bound = find_scene_node(r.scene.nodes, "bound");
  ASSERT_NE(bound, nullptr);
  EXPECT_EQ(bound->visible_bind, "demo.show_mesh");
}

TEST(AriadneScene, VisibleMapFormBindAndDefault) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: hiddenbound
      source: { file: a.obj }
      visible: { bind: demo.show, default: false }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.visible_bind, "demo.show"); // bound...
  EXPECT_FALSE(n.visible_default);        // ...AND starts hidden (map form)
}

TEST(AriadneScene, GroupNestsChildren) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: convoy
      type: group
      children:
        - node: truck1
          source: { file: truck.obj }
        - node: truck2
          source: { file: truck.obj }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &g = r.scene.nodes[0];
  EXPECT_EQ(g.type, "group");
  ASSERT_EQ(g.children.size(), 2u);
  EXPECT_EQ(g.children[0].id, "truck1");
  EXPECT_NE(find_scene_node(r.scene.nodes, "truck2"), nullptr);
}

TEST(AriadneScene, LightsAndShadows) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: g
      source: { file: g.obj }
  lights:
    - light: key
      kind: spot
      pos: [10, 20, 30]
      target: [0, 0, 0]
      cone: 30
      intensity: 1.5
    - light: soft
      rig: three_point
  shadows: { enabled: true }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.lights.size(), 2u);
  const SceneLight &key = r.scene.lights[0];
  EXPECT_EQ(key.id, "key");
  EXPECT_EQ(key.kind, "spot");
  EXPECT_FLOAT_EQ(key.pos[1], 20.0f);
  EXPECT_FLOAT_EQ(key.cone, 30.0f);
  EXPECT_FLOAT_EQ(key.intensity, 1.5f);
  EXPECT_EQ(r.scene.lights[1].rig, "three_point");
  EXPECT_TRUE(r.scene.has_shadows);
  EXPECT_TRUE(r.scene.shadows_enabled);
}

TEST(AriadneScene, WidgetsAndSceneCoexist) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: mesh
      source: { file: m.obj }
windows:
  - window: Controls
    children:
      - checkbox: Show
        bind: demo.show_mesh
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.scene.nodes.size(), 1u);
  const Widget *cb = find(r.root, Kind::Checkbox);
  ASSERT_NE(cb, nullptr);
  EXPECT_EQ(cb->bind, "demo.show_mesh");
}
