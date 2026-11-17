/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Ariadne's `tabs:` and `list:` widgets (the side-panel widgets VolumeGridRover's component needs,
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 item 5, P7), through the core walker with a
// scripted backend:
//   - tabs over children: only the selected tab's subtree walked, a pick written to the bound key
//     (and on: run), a write from outside selecting another tab, the selection kept per widget
//     without a bind;
//   - tabs over options: (a switch), with values:;
//   - list: rows of a state array, a selection written to <bind>.current (or current:), Add with
//     new_item: and {n}, Remove shifting the rows after it (nested fields too), on_add / on_remove;
//   - a backend with no tabs or list draws them as radios and buttons;
//   - the loader: the keys, tab titles from tab: / label: / the place, and its warnings (skipped
//     on a build without yaml-cpp, as the wasm-mt ports lane is).
// The widgets are built in C++ (widget.h's builders), so it runs natively and under node.

#include <algorithm>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <gtest/gtest.h>
#include <map>
#include <string>
#include <vector>

namespace ari = cvc::ariadne;

namespace {

// Records the walk; answers tab_bar / list / combo / button as the test scripts them.
struct Scripted : ari::Backend {
  std::vector<std::string> log;
  bool native = true; // false: use the Backend defaults for tab_bar and list
  int tab_pick = -1;  // the next tab_bar returns this pick (once)
  ari::ListEdit list_ret;
  std::string list_for; // the list (by label) list_ret answers; empty: the first drawn
  int combo_pick = -1;
  std::string click;
  std::map<std::string, std::vector<std::string>> items_of; // by list label, last frame's
  std::map<std::string, int> current_of;

  void rec(const std::string &s) { log.push_back(s); }
  bool saw(const std::string &s) const { return std::find(log.begin(), log.end(), s) != log.end(); }
  ari::Capabilities capabilities() const override {
    ari::Capabilities c;
    c.windows = true;
    return c;
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *, const char *, const ari::Size &, float, bool *) override {
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const ari::Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *t) override { rec(std::string("text:") + t); }
  void text_value(const char *l, const std::string &v) override {
    rec(std::string("value:") + l + "=" + v);
  }
  void separator() override {}
  bool button(const char *l) override {
    rec(std::string("button:") + l);
    if (click == l) {
      click.clear();
      return true;
    }
    return false;
  }
  bool menu_item_action(const char *) override { return false; }
  ari::BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  ari::BoolEdit checkbox(const char *, bool) override { return {}; }
  ari::IntEdit slider_int(const char *, const char *, int, int, int) override { return {}; }
  ari::DoubleEdit slider_double(const char *, const char *, double, double, double,
                                const char *) override {
    return {};
  }
  ari::IndexEdit combo(const char *l, int idx, const std::vector<std::string> &opts) override {
    std::string s = std::string("combo:") + l + "=" + std::to_string(idx) + ":";
    for (const std::string &o : opts)
      s += o + "|";
    rec(s);
    ari::IndexEdit e;
    e.index = idx;
    if (combo_pick >= 0) {
      e.index = combo_pick;
      e.changed = e.committed = true;
      combo_pick = -1;
    }
    return e;
  }
  ari::StringEdit input_text(const char *, const char *, const std::string &, bool) override {
    return {};
  }
  ari::IndexEdit tab_bar(const char *label, const char *id, int current,
                         const std::vector<std::string> &labels) override {
    if (!native)
      return Backend::tab_bar(label, id, current, labels);
    std::string s = "tabs:" + std::to_string(current) + ":";
    for (const std::string &t : labels)
      s += t + "|";
    rec(s);
    ari::IndexEdit e;
    e.index = current;
    if (tab_pick >= 0) {
      e.index = tab_pick;
      e.changed = e.committed = true;
      tab_pick = -1;
    }
    return e;
  }
  ari::ListEdit list(const char *label, const char *id, int current,
                     const std::vector<std::string> &items, const ari::ListOptions &o) override {
    if (!native)
      return Backend::list(label, id, current, items, o);
    items_of[label] = items;
    current_of[label] = current;
    rec(std::string("list:") + (o.add ? "add" : "") + (o.remove ? "remove" : ""));
    if (!list_for.empty() && list_for != label)
      return {};
    const ari::ListEdit e = list_ret;
    list_ret = {};
    return e;
  }
};

struct Doc {
  cvc::app app;
  ari::Runtime rt{app, "ui"};
  Scripted be;
  explicit Doc(ari::Widget root) {
    rt.set_backend(&be);
    rt.set_root(std::move(root));
  }
  void frame() {
    be.log.clear();
    rt.render();
    rt.drain();
  }
  std::string val(const std::string &k) { return cvc::state::instance(app)("ui." + k).value(); }
  void set(const std::string &k, const std::string &v) {
    cvc::state::instance(app)("ui." + k).value(v);
  }
};

// The panel's tabs (bound, with on:), tabs with no bind, and a switch over options with values.
ari::Widget tabs_doc() {
  ari::Widget side =
      ari::tabs("grid.panel", {"Probe", "Classes", "Tab 3"},
                {ari::group({ari::text("probe body")}), ari::group({ari::text("classes body")}),
                 ari::group({ari::text("third body")})});
  side.id = "side";
  side.on = "(state-set \"picked\" (state-get \"grid.panel\"))";
  ari::Widget free = ari::tabs(
      "", {"A", "B"}, {ari::group({ari::text("a body")}), ari::group({ari::text("b body")})});
  free.id = "free";
  ari::Widget view = ari::tab_bar("View", "grid.view", {"XY", "XZ", "ZY"});
  view.values = {"xy", "xz", "zy"};
  return ari::window("Panel", {side, free, view});
}

// A list with Add / Remove / new_item / handlers, and one on another field and selection key.
ari::Widget list_doc() {
  ari::Widget classes = ari::list("Point classes", "grid.classes", true, true);
  classes.list_new = {{"name", "class {n}"}, {"color", "1,0,0"}, {"radius", "2"}};
  classes.on = "(state-set \"sel\" (state-get \"grid.classes.current\"))";
  classes.on_add = "(state-set \"added\" \"1\")";
  classes.on_remove = "(state-set \"removed\" \"1\")";
  ari::Widget objects = ari::list("Objects", "grid.objects");
  objects.list_field = "title";
  objects.list_current = "grid.object";
  return ari::window("Classes", {classes, objects});
}

} // namespace

TEST(AriadneTabs, OverChildrenTheSelectedSubtreeAndTheKey) {
  Doc d(tabs_doc());
  d.frame();
  EXPECT_TRUE(d.be.saw("tabs:0:Probe|Classes|Tab 3|"));
  EXPECT_EQ(d.val("grid.panel"), "Probe") << "the first tab seeded";
  EXPECT_TRUE(d.be.saw("text:probe body"));
  EXPECT_FALSE(d.be.saw("text:classes body")) << "only the selected tab's subtree is walked";
  // A pick: the key takes the tab's title, on: runs, its subtree shows.
  d.be.tab_pick = 1;
  d.frame();
  EXPECT_EQ(d.val("grid.panel"), "Classes");
  EXPECT_EQ(d.val("picked"), "Classes") << "on: ran after the write";
  EXPECT_TRUE(d.be.saw("text:classes body")) << "the new tab shows in the same frame";
  // A write from outside selects another tab.
  d.set("grid.panel", "Tab 3");
  d.frame();
  EXPECT_TRUE(d.be.saw("tabs:2:Probe|Classes|Tab 3|"));
  EXPECT_TRUE(d.be.saw("text:third body"));
  // Without a bind the selection is kept under tree.<id>.tab.
  EXPECT_EQ(d.val("tree.free.tab"), "A");
  EXPECT_TRUE(d.be.saw("text:a body") && !d.be.saw("text:b body"));
}

TEST(AriadneTabs, OverOptionsASwitchWithValues) {
  Doc d(tabs_doc());
  d.frame();
  EXPECT_TRUE(d.be.saw("tabs:0:XY|XZ|ZY|"));
  EXPECT_EQ(d.val("grid.view"), "xy") << "the key holds the value, the tab shows the option";
  d.set("grid.view", "zy");
  d.frame();
  EXPECT_TRUE(d.be.saw("tabs:2:XY|XZ|ZY|"));
}

TEST(AriadneList, RowsSelectionAddAndRemove) {
  Doc d(list_doc());
  // Three rows, with a nested field each.
  d.set("grid.classes", "3");
  for (int i = 0; i < 3; ++i) {
    const std::string r = "grid.classes." + std::to_string(i);
    d.set(r + ".name", "c" + std::to_string(i));
    d.set(r + ".style.color", "col" + std::to_string(i));
  }
  d.frame();
  EXPECT_TRUE(d.be.saw("list:addremove"));
  EXPECT_EQ(d.be.items_of["Point classes"], (std::vector<std::string>{"c0", "c1", "c2"}));
  EXPECT_EQ(d.be.current_of["Point classes"], -1) << "nothing selected yet";
  // Select row 1: <bind>.current, then on:.
  d.be.list_ret.selected = true;
  d.be.list_ret.index = 1;
  d.frame();
  EXPECT_EQ(d.val("grid.classes.current"), "1");
  EXPECT_EQ(d.val("sel"), "1");
  // Remove it: row 2 moves up (nested fields too), the last goes, the count drops.
  d.be.list_ret.remove = true;
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "2");
  EXPECT_EQ(d.val("grid.classes.0.name"), "c0");
  EXPECT_EQ(d.val("grid.classes.1.name"), "c2");
  EXPECT_EQ(d.val("grid.classes.1.style.color"), "col2");
  EXPECT_FALSE(cvc::state::instance(d.app).findThroughLinks("ui.grid.classes.2"))
      << "the old last row is gone";
  EXPECT_EQ(d.val("grid.classes.current"), "1") << "the selection stays in range";
  EXPECT_EQ(d.val("removed"), "1");
  // Add: the new_item fields with {n}, the count, the selection on it, on_add.
  d.be.list_ret.add = true;
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "3");
  EXPECT_EQ(d.val("grid.classes.2.name"), "class 3");
  EXPECT_EQ(d.val("grid.classes.2.color"), "1,0,0");
  EXPECT_EQ(d.val("grid.classes.2.radius"), "2");
  EXPECT_EQ(d.val("grid.classes.current"), "2");
  EXPECT_EQ(d.val("added"), "1");
  d.frame();
  EXPECT_EQ(d.be.items_of["Point classes"], (std::vector<std::string>{"c0", "c2", "class 3"}));
  // Remove the last row: the selection moves to the new last.
  d.be.list_ret.remove = true;
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "2");
  EXPECT_EQ(d.val("grid.classes.current"), "1");
  // Remove everything: -1.
  d.be.list_ret.remove = true;
  d.frame();
  d.be.list_ret.remove = true;
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "0");
  EXPECT_EQ(d.val("grid.classes.current"), "-1");
  d.be.list_ret.remove = true; // nothing selected: nothing happens
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "0");
}

TEST(AriadneList, AnotherFieldAndCurrentKey) {
  Doc d(list_doc());
  d.set("grid.objects", "2");
  d.set("grid.objects.0.title", "outer");
  d.set("grid.objects.1.title", "inner");
  d.set("grid.object", "1");
  d.frame();
  EXPECT_EQ(d.be.items_of["Objects"], (std::vector<std::string>{"outer", "inner"}));
  EXPECT_EQ(d.be.current_of["Objects"], 1) << "current: names the selection's key";
  // Without add: / remove: their presses do nothing.
  d.be.list_for = "Objects";
  d.be.list_ret.add = true;
  d.frame();
  EXPECT_EQ(d.val("grid.objects"), "2");
}

TEST(AriadneTabsList, ABackendWithoutThemDrawsRadiosAndButtons) {
  Doc d(list_doc());
  d.be.native = false;
  d.set("grid.classes", "2");
  d.set("grid.classes.0.name", "a");
  d.set("grid.classes.1.name", "b");
  d.set("grid.classes.current", "0");
  d.frame();
  EXPECT_TRUE(d.be.saw("combo:Point classes=0:a|b|")) << "the rows as a radio (a combo here)";
  EXPECT_TRUE(d.be.saw("button:Add") && d.be.saw("button:Remove"));
  d.be.click = "Add";
  d.frame();
  EXPECT_EQ(d.val("grid.classes"), "3");
  d.be.combo_pick = 1;
  d.frame();
  EXPECT_EQ(d.val("grid.classes.current"), "1");

  Doc t(tabs_doc());
  t.be.native = false;
  t.frame();
  EXPECT_TRUE(t.be.saw("combo:=0:Probe|Classes|Tab 3|")) << "the tabs as a radio row";
  t.be.combo_pick = 1;
  t.frame();
  EXPECT_EQ(t.val("grid.panel"), "Classes");
}

TEST(AriadneTabsList, TheLoader) {
  cvc::app app;
  const ari::LoadResult r = ari::load_string(app, R"(meta: { name: w, min_libcvc: 3.7.0 }
windows:
  - window: W
    children:
      - type: tabs
        id: side
        bind: grid.panel
        children:
          - { type: group, tab: Probe, children: [ { text: x } ] }
          - { type: group, label: Classes, children: [ { text: y } ] }
          - { type: group, children: [ { text: z } ] }
      - tabs: View
        bind: grid.view
        options: [XY, XZ]
        values: [xy, xz]
      - list: Point classes
        bind: grid.classes
        field: title
        current: grid.sel
        add: true
        remove: true
        rows: 4
        new_item: { name: "class {n}", radius: 2 }
        on_add: (state-set "a" 1)
      - tabs: Empty
      - list: NoBind
      - list: Bad
        bind: x
        new_item: [1, 2]
)");
  if (!r.ok && r.error.find("yaml") != std::string::npos)
    GTEST_SKIP() << "built without yaml-cpp: " << r.error;
  ASSERT_TRUE(r.ok) << r.error;
  const ari::Widget &win = r.root.children.at(0);
  const ari::Widget &side = win.children.at(0);
  EXPECT_EQ(side.kind, ari::Kind::Tabs);
  EXPECT_EQ(side.tab_titles, (std::vector<std::string>{"Probe", "Classes", "Tab 3"}))
      << "titles from tab:, label:, the place";
  EXPECT_EQ(side.children.size(), 3u);
  const ari::Widget &view = win.children.at(1);
  EXPECT_EQ(view.kind, ari::Kind::Tabs);
  EXPECT_EQ(view.options, (std::vector<std::string>{"XY", "XZ"}));
  EXPECT_EQ(view.values, (std::vector<std::string>{"xy", "xz"}));
  const ari::Widget &lst = win.children.at(2);
  EXPECT_EQ(lst.kind, ari::Kind::List);
  EXPECT_EQ(lst.label, "Point classes");
  EXPECT_EQ(lst.list_field, "title");
  EXPECT_EQ(lst.list_current, "grid.sel");
  EXPECT_TRUE(lst.list_add && lst.list_remove);
  EXPECT_EQ(lst.list_rows, 4);
  EXPECT_EQ(lst.list_new.size(), 2u);
  EXPECT_EQ(lst.on_add, "(state-set \"a\" 1)");
  auto has = [&r](const std::string &s) {
    for (const std::string &w : r.warnings)
      if (w.find(s) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(has("tabs 'Empty' has neither children: nor options:"));
  EXPECT_TRUE(has("NoBind"));
  EXPECT_TRUE(has("new_item: that is not a map"));
}
