// Ariadne settings editor (Kind::Settings): the core's walk of a state subtree, each key's editor
// from its metadata (state_key_meta's access / unit / min / max, param_table's type / choices /
// default, a scale), the groups, the filter and the search, the writes and resets it makes through
// the committed-write path, and its re-read when something under its root changes.
#include <cmath>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_key_meta.h>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::ariadne;

namespace {

// The least backend the walk needs, recording each editor's spec and returning `next` once.
// `use_default` draws it with Backend::settings (the flat list) instead, its lines in `lines`.
struct SettingsBackend : Backend {
  std::vector<SettingsSpec> specs;
  std::vector<std::string> lines;
  SettingsEdit next{};
  bool use_default = false;
  const SettingsNode *first_node = nullptr; // the spec's storage as handed over (the cache test)

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
  bool button(const char *l) override {
    lines.push_back(std::string("(") + l + ")");
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  BoolEdit checkbox(const char *l, bool v) override {
    lines.push_back(std::string(v ? "[x] " : "[ ] ") + l);
    return {};
  }
  IntEdit slider_int(const char *l, const char *, int v, int lo, int hi) override {
    lines.push_back(std::string(l) + ": " + std::to_string(v) + " in " + std::to_string(lo) + ".." +
                    std::to_string(hi));
    return {};
  }
  DoubleEdit slider_double(const char *l, const char *, double v, double, double,
                           const char *fmt) override {
    char b[64];
    std::snprintf(b, sizeof b, fmt, v);
    lines.push_back(std::string(l) + ": " + b);
    return {};
  }
  DoubleEdit slider_double_log(const char *l, const char *k, double v, double lo, double hi,
                               const char *fmt) override {
    slider_double(l, k, v, lo, hi, fmt);
    lines.back() += " (log)";
    return {};
  }
  IndexEdit combo(const char *l, int i, const std::vector<std::string> &o) override {
    lines.push_back(std::string(l) + ": <" + (i >= 0 ? o[static_cast<std::size_t>(i)] : "") + ">");
    return {};
  }
  StringEdit input_text(const char *l, const char *, const std::string &v, bool) override {
    lines.push_back(std::string(l) + ": [" + v + "]");
    return {};
  }
  SettingsEdit settings(const char *label, const char *id, const SettingsSpec &spec) override {
    specs.push_back(spec);
    first_node = spec.nodes.empty() ? nullptr : spec.nodes.data();
    if (use_default)
      return Backend::settings(label, id, spec);
    const SettingsEdit e = next;
    next = {};
    return e;
  }
};

cvc::state &st(cvc::app &app, const std::string &path) { return cvc::state::instance(app)(path); }
void put(cvc::app &app, const std::string &path, const std::string &v) { st(app, path).value(v); }
std::string get(cvc::app &app, const std::string &path) { return st(app, path).value(); }

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// A Runtime over a SettingsBackend. Member order: the backend outlives the Runtime.
struct Doc {
  cvc::app app;
  SettingsBackend be;
  Runtime rt{app, "doc"};
  explicit Doc(Widget w) {
    rt.set_backend(&be);
    rt.set_root(group({window("W", {std::move(w)})}));
  }
  const SettingsSpec &frame() {
    be.specs.clear();
    rt.render();
    EXPECT_EQ(be.specs.size(), 1u);
    return be.specs.back();
  }
  // The node at `id`, or null.
  const SettingsNode *node(const std::string &id) {
    for (const SettingsNode &n : be.specs.back().nodes)
      if (n.id == id)
        return &n;
    return nullptr;
  }
  std::vector<std::string> ids() const {
    std::vector<std::string> out;
    for (const SettingsNode &n : be.specs.back().nodes)
      out.push_back(std::string(static_cast<std::size_t>(n.depth), ' ') + (n.group ? "+" : "") +
                    n.id);
    return out;
  }
  int index(const std::string &id) {
    const std::vector<SettingsNode> &nodes = be.specs.back().nodes;
    for (std::size_t i = 0; i < nodes.size(); ++i)
      if (nodes[i].id == id)
        return static_cast<int>(i);
    return -1;
  }
};

bool has_warning(Runtime &rt, const std::string &needle) {
  for (const std::string &w : rt.take_reactive_warnings())
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

} // namespace

// Each key's editor comes from its description: a binding's state_key_meta (access, unit, range),
// a params table's properties (type, choices, default), and a scale.
TEST(AriadneSettings, EachKeysEditorComesFromItsMetadata) {
  Doc d(settings("Tuning", "cfg"));
  const cvc::state_key_meta table[] = {
      {"vmax", "knob", "m/s", 0.5, 40.0, "the top speed"},
      {"gain", "knob", "1", 0.01, 10.0, "a ratio over three decades"},
      {"agents", "build", "", 1.0, 16.0, "how many there are"}, // unit "": a count
      {"on", "knob", "", 0.0, 1.0, "a flag"},
      {"wait", "knob", "tick", 1.0, 100.0, "a count of steps"},
      {"speed", "telemetry", "m/s", kNone, kNone, "what the run measures"},
      {"label", "knob", nullptr, kNone, kNone, "a name"},
      {"free", "knob", "m", kNone, kNone, "a length with no range"},
  };
  for (const auto &m : table)
    put(d.app, std::string("doc.cfg.") + m.key, "1");
  put(d.app, "doc.cfg.label", "alpha");
  cvc::describe_state_keys(d.app, "doc.cfg", table);
  put(d.app, "doc.cfg.mode", "b");
  st(d.app, "doc.cfg.mode").property("choices", "a, b,c");
  put(d.app, "doc.cfg.linear", "5");
  st(d.app, "doc.cfg.linear").property("unit", "m").property("min", "0.1").property("max", "100");
  st(d.app, "doc.cfg.linear").property("scale", "linear");
  put(d.app, "doc.cfg.narrow", "5");
  st(d.app, "doc.cfg.narrow").property("unit", "m").property("min", "1").property("max", "10");
  st(d.app, "doc.cfg.narrow").property("scale", "log");
  put(d.app, "doc.cfg.typed_bool", "true");
  st(d.app, "doc.cfg.typed_bool").property("type", "bool");
  put(d.app, "doc.cfg.typed_int", "3");
  st(d.app, "doc.cfg.typed_int").property("type", "int");
  put(d.app, "doc.cfg.typed_str", "3");
  st(d.app, "doc.cfg.typed_str").property("type", "string");
  put(d.app, "doc.cfg.locked", "7");
  st(d.app, "doc.cfg.locked").property("unit", "m").property("min", "0").property("max", "9");
  st(d.app, "doc.cfg.locked").readOnly(true);
  put(d.app, "doc.cfg.odd", "fast"); // described as a number, holding a word
  st(d.app, "doc.cfg.odd").property("unit", "m/s");

  d.frame();
  const auto kind = [&](const std::string &id) {
    const SettingsNode *n = d.node(id);
    EXPECT_NE(n, nullptr) << id;
    return n ? n->kind : SettingKind::Readout;
  };
  EXPECT_EQ(kind("vmax"), SettingKind::Number);
  EXPECT_FALSE(d.node("vmax")->log) << "0.5..40 spans 80x";
  EXPECT_EQ(d.node("vmax")->fmt, "%.4g m/s");
  EXPECT_TRUE(d.node("vmax")->has_range);
  EXPECT_EQ(d.node("vmax")->min, 0.5);
  EXPECT_EQ(d.node("vmax")->max, 40.0);
  EXPECT_EQ(kind("gain"), SettingKind::Number);
  EXPECT_TRUE(d.node("gain")->log) << "a positive range over 100x moves by ratio";
  EXPECT_EQ(d.node("gain")->fmt, "%.4g") << "a ratio shows no unit";
  EXPECT_EQ(kind("agents"), SettingKind::Int);
  EXPECT_EQ(kind("on"), SettingKind::Toggle);
  EXPECT_EQ(kind("wait"), SettingKind::Int);
  EXPECT_EQ(d.node("wait")->fmt, "%d ticks");
  EXPECT_TRUE(d.node("wait")->log);
  EXPECT_EQ(kind("speed"), SettingKind::Readout);
  EXPECT_EQ(kind("label"), SettingKind::Text);
  EXPECT_EQ(kind("free"), SettingKind::Number);
  EXPECT_FALSE(d.node("free")->has_range);
  EXPECT_EQ(kind("mode"), SettingKind::Choice);
  EXPECT_EQ(d.node("mode")->choices, (std::vector<std::string>{"a", "b", "c"}));
  EXPECT_FALSE(d.node("linear")->log) << "scale: linear over 1000x";
  EXPECT_TRUE(d.node("narrow")->log) << "scale: log over 10x";
  EXPECT_EQ(kind("typed_bool"), SettingKind::Toggle);
  EXPECT_EQ(kind("typed_int"), SettingKind::Int);
  EXPECT_EQ(kind("typed_str"), SettingKind::Text);
  EXPECT_EQ(kind("locked"), SettingKind::Readout) << "a read-only node";
  EXPECT_EQ(kind("odd"), SettingKind::Text) << "a number's description, a word's value";

  // The tooltip: the path, the description, unit, range, default and when it applies.
  const std::string &tip = d.node("vmax")->tooltip;
  EXPECT_NE(tip.find("vmax\nthe top speed"), std::string::npos) << tip;
  EXPECT_NE(tip.find("unit: m/s"), std::string::npos) << tip;
  EXPECT_NE(tip.find("recommended: 0.5 .. 40"), std::string::npos) << tip;
  EXPECT_NE(tip.find("when first shown: 1"), std::string::npos) << tip;
  EXPECT_NE(tip.find("applies live"), std::string::npos) << tip;
  EXPECT_NE(d.node("agents")->tooltip.find("applies when it is next built"), std::string::npos);
  EXPECT_NE(d.node("speed")->tooltip.find("read-only"), std::string::npos);
  EXPECT_FALSE(d.node("speed")->has_default) << "a read-out has no reset";
}

// A branch is a group, its header and then what is under it one deeper. Each level lists its keys
// first, in name order, then its branches: groups: puts the listed ones first, in its order, under
// their titles, and the rest follow in name order. A branch with nothing shown under it is left
// out.
TEST(AriadneSettings, GroupsNestInTheirOrderUnderTheirTitles) {
  Widget w = settings("S", "cfg");
  w.settings_groups = {{"world", "The world", "what the world is made of", true},
                       {"nav", "", "", false}};
  Doc d(w);
  put(d.app, "doc.cfg.alpha", "1");
  put(d.app, "doc.cfg.nav.vmax", "2");
  put(d.app, "doc.cfg.nav.plant.lag", "3");
  put(d.app, "doc.cfg.world.seed", "4");
  st(d.app, "doc.cfg.nav").comment("how it drives");
  put(d.app, "doc.cfg.empty.hidden", "5");
  st(d.app, "doc.cfg.empty.hidden").hidden(true);
  d.frame();
  EXPECT_EQ(d.ids(), (std::vector<std::string>{"alpha", "+world", " world.seed", "+nav",
                                               " nav.vmax", " +nav.plant", "  nav.plant.lag"}))
      << "keys first, then branches by groups: and name; a branch of hidden keys is left out";
  const SettingsNode *world = d.node("world");
  ASSERT_NE(world, nullptr);
  EXPECT_TRUE(world->group);
  EXPECT_EQ(world->name, "The world");
  EXPECT_TRUE(world->open);
  EXPECT_EQ(world->tooltip, "world\nwhat the world is made of");
  EXPECT_EQ(d.node("nav")->name, "nav");
  EXPECT_FALSE(d.node("nav")->open);
  EXPECT_EQ(d.node("nav")->tooltip, "nav\nhow it drives") << "a branch's comment is its help";
  EXPECT_EQ(d.node("nav.plant.lag")->name, "lag");
  EXPECT_EQ(d.node("nav.plant.lag")->path, "doc.cfg.nav.plant.lag");
}

// filter: a substring or a glob over the key's path under the root; '-' leaves out what it
// matches. The search box narrows by words in the path or the description, opening every group.
TEST(AriadneSettings, FilterAndSearchNarrowTheKeys) {
  Widget w = settings("S", "cfg");
  w.settings_filter = {"nav.*", "seed", "-nav.plant.*"};
  w.settings_search = true;
  w.id = "tune";
  Doc d(w);
  put(d.app, "doc.cfg.nav.vmax", "2");
  put(d.app, "doc.cfg.nav.a_max", "2");
  st(d.app, "doc.cfg.nav.a_max").comment("the braking it commands");
  put(d.app, "doc.cfg.nav.plant.lag", "3");
  put(d.app, "doc.cfg.world.seed", "4");
  put(d.app, "doc.cfg.world.half", "4");
  const SettingsSpec &s = d.frame();
  EXPECT_TRUE(s.narrowed) << "a filter narrows it: groups drawn open";
  EXPECT_TRUE(s.search);
  EXPECT_EQ(d.ids(),
            (std::vector<std::string>{"+nav", " nav.a_max", " nav.vmax", "+world", " world.seed"}));

  put(d.app, "doc.tree.tune.search", "BRAKING");
  d.frame();
  EXPECT_EQ(d.ids(), (std::vector<std::string>{"+nav", " nav.a_max"}))
      << "a word of the description, in any case";
  EXPECT_EQ(d.be.specs.back().search_text, "BRAKING");

  // The backend's search box writes the text where the editor keeps it.
  d.be.next.search_changed = true;
  d.be.next.search = "-vmax nav";
  d.frame();
  EXPECT_EQ(get(d.app, "doc.tree.tune.search"), "-vmax nav");
  d.frame();
  EXPECT_EQ(d.ids(), (std::vector<std::string>{"+nav", " nav.a_max"})) << "-word leaves out";

  put(d.app, "doc.tree.tune.search", "nothing-like-it");
  d.frame();
  EXPECT_TRUE(d.be.specs.back().nodes.empty());
  EXPECT_EQ(d.be.specs.back().empty, "no setting matches the search");
}

// A filter of '-' patterns only leaves keys out, so it does not open the groups: groups: and
// open: still decide, and a user can close one. A search, or a pattern that picks keys, does.
TEST(AriadneSettings, AFilterThatOnlyLeavesKeysOutKeepsTheGroupsClosable) {
  Widget w = settings("S", "cfg");
  w.settings_filter = {"-nav.plant.*", "-world.half"};
  w.settings_search = true;
  w.id = "tune";
  Doc d(w);
  put(d.app, "doc.cfg.nav.vmax", "2");
  put(d.app, "doc.cfg.nav.plant.lag", "3");
  put(d.app, "doc.cfg.world.seed", "4");
  put(d.app, "doc.cfg.world.half", "4");
  EXPECT_FALSE(d.frame().narrowed) << "only '-' patterns: the groups keep their own state";
  EXPECT_EQ(d.ids(), (std::vector<std::string>{"+nav", " nav.vmax", "+world", " world.seed"}));
  EXPECT_FALSE(d.node("nav")->open) << "open: false";

  put(d.app, "doc.tree.tune.search", "seed");
  EXPECT_TRUE(d.frame().narrowed) << "a search narrows it";
  put(d.app, "doc.tree.tune.search", "");
  EXPECT_FALSE(d.frame().narrowed);

  Widget picks = settings("S", "cfg");
  picks.settings_filter = {"-world.half", "world.*"};
  Doc p(picks);
  put(p.app, "doc.cfg.world.seed", "4");
  EXPECT_TRUE(p.frame().narrowed) << "a pattern that picks keys narrows it";
}

// An edit is written through the committed-write path, as each bound widget's: live while a
// slider moves, `on:` after the commit; a state watch on the key sees each write. A value the key's
// kind does not take is refused with a warning.
TEST(AriadneSettings, EditsWriteTheKeyAndFireOn) {
  Widget w = settings("S", "cfg");
  w.on = "tuned";
  Doc d(w);
  int fired = 0;
  d.rt.on("tuned", [&] { ++fired; });
  put(d.app, "doc.cfg.vmax", "10");
  st(d.app, "doc.cfg.vmax").property("unit", "m/s").property("min", "1").property("max", "40");
  put(d.app, "doc.cfg.agents", "4");
  st(d.app, "doc.cfg.agents").property("min", "1").property("max", "16");
  int watched = 0;
  st(d.app, "doc.cfg.vmax").valueChanged.connect([&] { ++watched; });
  d.frame();

  d.be.next = {d.index("vmax"), "12.5", true, false};
  d.frame();
  d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.cfg.vmax"), "12.5") << "a drag writes as it moves";
  EXPECT_EQ(fired, 0) << "on: waits for the release";
  EXPECT_EQ(watched, 1);

  d.be.next = {d.index("vmax"), "13", true, true};
  d.frame();
  d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.cfg.vmax"), "13");
  EXPECT_EQ(fired, 1);
  EXPECT_EQ(watched, 2);
  d.frame();
  EXPECT_EQ(d.node("vmax")->value, "13") << "the editor shows what it wrote";

  d.be.next = {d.index("agents"), "many", true, true};
  d.frame();
  d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.cfg.agents"), "4");
  EXPECT_EQ(fired, 1);
  EXPECT_TRUE(has_warning(d.rt, "'many' is not a value agents takes"));
  d.be.next = {d.index("agents"), "6.4", true, true};
  d.frame();
  EXPECT_EQ(get(d.app, "doc.cfg.agents"), "6") << "an integer key takes a number rounded";
}

// commit: release: a slider writes once, when it is let go.
TEST(AriadneSettings, CommitOnReleaseWritesOnce) {
  Widget w = settings("S", "cfg");
  w.commit_on_release = true;
  Doc d(w);
  put(d.app, "doc.cfg.vmax", "10");
  st(d.app, "doc.cfg.vmax").property("unit", "m/s").property("min", "1").property("max", "40");
  d.frame();
  d.be.next = {d.index("vmax"), "12", true, false};
  d.frame();
  EXPECT_EQ(get(d.app, "doc.cfg.vmax"), "10");
  d.be.next = {d.index("vmax"), "14", true, true};
  d.frame();
  EXPECT_EQ(get(d.app, "doc.cfg.vmax"), "14");
}

// A key's reset writes its default: its `default` property, else the value the editor first saw.
// A group's reset puts back each key under it that differs. on: runs after either.
TEST(AriadneSettings, ResetsPutTheDefaultsBack) {
  Widget w = settings("S", "cfg");
  w.on = "tuned";
  Doc d(w);
  int fired = 0;
  d.rt.on("tuned", [&] { ++fired; });
  put(d.app, "doc.cfg.nav.vmax", "10");
  st(d.app, "doc.cfg.nav.vmax").property("default", "8").property("unit", "m/s");
  put(d.app, "doc.cfg.nav.gain", "0.5");
  st(d.app, "doc.cfg.nav.gain").property("unit", "1");
  put(d.app, "doc.cfg.nav.out", "3");
  st(d.app, "doc.cfg.nav.out").property("access", "telemetry");
  d.frame();
  EXPECT_TRUE(d.node("nav.vmax")->changed) << "10 is not its default 8";
  EXPECT_EQ(d.node("nav.vmax")->def, "8");
  EXPECT_FALSE(d.node("nav.gain")->changed) << "its default is its first value";
  EXPECT_EQ(d.node("nav")->resettable, 1);

  put(d.app, "doc.cfg.nav.gain", "0.75");
  put(d.app, "doc.cfg.nav.out", "4");
  d.frame();
  EXPECT_TRUE(d.node("nav.gain")->changed);
  EXPECT_EQ(d.node("nav.gain")->def, "0.5");
  EXPECT_EQ(d.node("nav")->resettable, 2);

  d.be.next.reset = d.index("nav.gain");
  d.frame();
  d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.cfg.nav.gain"), "0.5");
  EXPECT_EQ(get(d.app, "doc.cfg.nav.vmax"), "10") << "a key's reset is only its own";
  EXPECT_EQ(fired, 1);

  d.be.next.reset = d.index("nav");
  d.frame();
  d.rt.drain();
  EXPECT_EQ(get(d.app, "doc.cfg.nav.vmax"), "8");
  EXPECT_EQ(get(d.app, "doc.cfg.nav.out"), "4") << "a read-out is not reset";
  EXPECT_EQ(fired, 2);
  d.frame();
  EXPECT_EQ(d.node("nav")->resettable, 0);
}

// The editor reads its subtree again only when something under the root was written: a value, a
// comment or a property. A write elsewhere, or a frame with none, reuses what it built.
TEST(AriadneSettings, ItRereadsOnlyWhenItsSubtreeChanges) {
  Doc d(settings("S", "cfg"));
  put(d.app, "doc.cfg.vmax", "10");
  put(d.app, "doc.other", "1");
  d.frame();
  const SettingsNode *built = d.be.first_node;
  ASSERT_NE(built, nullptr);
  EXPECT_EQ(d.node("vmax")->kind, SettingKind::Text) << "nothing describes it yet";
  d.frame();
  EXPECT_EQ(d.be.first_node, built) << "no write: the same spec";
  put(d.app, "doc.other", "2");
  d.frame();
  EXPECT_EQ(d.be.first_node, built) << "a write outside the root";

  st(d.app, "doc.cfg.vmax").property("unit", "m/s");
  d.frame();
  EXPECT_NE(d.be.first_node, built) << "a property under the root";
  EXPECT_EQ(d.node("vmax")->kind, SettingKind::Number);
  built = d.be.first_node;
  st(d.app, "doc.cfg.vmax").comment("the top speed");
  d.frame();
  EXPECT_NE(d.be.first_node, built) << "a comment";
  EXPECT_NE(d.node("vmax")->tooltip.find("the top speed"), std::string::npos);
  built = d.be.first_node;
  put(d.app, "doc.cfg.added", "x");
  d.frame();
  EXPECT_NE(d.be.first_node, built) << "a key added";
  EXPECT_NE(d.node("added"), nullptr);
}

// Several roots: each is a group of its own, its keys' paths starting with the root as written.
// A root that is one key is a row; readouts: false leaves out the read-only keys.
TEST(AriadneSettings, SeveralRootsAreGroupsOfTheirOwn) {
  Widget w = settings("S", "nav.vehicle");
  w.settings_roots.push_back("nav.reach_tol");
  w.settings_roots.push_back("/plant");
  w.settings_readouts = false;
  Doc d(w);
  put(d.app, "doc.nav.vehicle.vmax", "10");
  put(d.app, "doc.nav.vehicle.speed", "3");
  st(d.app, "doc.nav.vehicle.speed").property("access", "telemetry");
  put(d.app, "doc.nav.reach_tol", "2");
  put(d.app, "plant.latency", "0.1");
  d.frame();
  EXPECT_EQ(d.ids(), (std::vector<std::string>{"+nav.vehicle", " nav.vehicle.vmax", "nav.reach_tol",
                                               "+plant", " plant.latency"}));
  EXPECT_EQ(d.node("plant.latency")->path, "plant.latency");
}

// The default editor (a backend without its own) is a flat list of the backend's widgets: a group
// as a "[title]" line with a reset while it has keys to reset, each key labelled with its path, a
// reset beside a changed one.
TEST(AriadneSettings, TheDefaultIsAFlatListOfTheBackendsWidgets) {
  Widget w = settings("Tuning", "cfg");
  w.settings_groups = {{"world", "World", "", false}};
  Doc d(w);
  d.be.use_default = true;
  put(d.app, "doc.cfg.nav.vmax", "10");
  st(d.app, "doc.cfg.nav.vmax").property("unit", "m/s").property("min", "1").property("max", "40");
  put(d.app, "doc.cfg.nav.gain", "0.5");
  st(d.app, "doc.cfg.nav.gain").property("unit", "1").property("min", "0.01").property("max", "10");
  put(d.app, "doc.cfg.world.on", "1");
  st(d.app, "doc.cfg.world.on").property("min", "0").property("max", "1");
  put(d.app, "doc.cfg.world.mode", "b");
  st(d.app, "doc.cfg.world.mode").property("choices", "a,b");
  put(d.app, "doc.cfg.name", "alpha");
  put(d.app, "doc.cfg.out", "3");
  st(d.app, "doc.cfg.out").property("access", "telemetry").property("unit", "m");
  d.frame();
  put(d.app, "doc.cfg.nav.vmax", "12");
  d.be.lines.clear();
  d.frame();
  EXPECT_EQ(d.be.lines,
            (std::vector<std::string>{"Tuning", "name: [alpha]", "out: 3 m", "[World]",
                                      "world.mode: <b>", "[x] world.on", "[nav]", "(reset all)",
                                      "nav.gain: 0.5 (log)", "nav.vmax: 12 m/s", "(reset)"}));
}

// A repeated editor edits each instance's own subtree: {i} in root: and filter:.
TEST(AriadneSettings, ARepeatedEditorEditsEachInstancesSubtree) {
  Widget w = settings("Vehicle {i}", "veh.{i}");
  w.settings_filter = {"{i}"};
  w.repeat = "2";
  Doc d(w);
  put(d.app, "doc.veh.0.v0", "1");
  put(d.app, "doc.veh.0.v1", "1");
  put(d.app, "doc.veh.1.v0", "2");
  put(d.app, "doc.veh.1.v1", "2");
  d.be.specs.clear();
  d.rt.render();
  ASSERT_EQ(d.be.specs.size(), 2u);
  ASSERT_EQ(d.be.specs[0].nodes.size(), 1u);
  EXPECT_EQ(d.be.specs[0].nodes[0].path, "doc.veh.0.v0");
  ASSERT_EQ(d.be.specs[1].nodes.size(), 1u);
  EXPECT_EQ(d.be.specs[1].nodes[0].path, "doc.veh.1.v1");
}
