// Tests for the shared Ariadne state-binding primitives (cvc::ariadne, bind.h):
// resolve_bind (the ONE resolution rule widget binds and scene `visible:` binds
// share), read_or_seed / write, and sync_scene_visibility (the §9 increment-2
// visibility poll that mirrors a bound source path into a node's `.visible` key).
// All pure cvc::state — no VTK — so the mirror contract is verified headlessly; the
// node's state-key -> setVisible reactivity lives in cvc::gl::SceneNode.

#include <cvc/ariadne/bind.h>
#include <cvc/core/app.h>
#include <cvc/core/state.h>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace cvc::ariadne;

namespace {

std::string sval(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

} // namespace

// --- resolve_bind: the rule that MUST match the widget Runtime -----------------

TEST(AriadneBind, ResolveEmptyPrefixIsIdentity) {
  EXPECT_EQ(resolve_bind("", "a.b"), "a.b");
}

TEST(AriadneBind, ResolvePrefixSplices) {
  EXPECT_EQ(resolve_bind("ui.demo", "wire"), "ui.demo.wire");
  EXPECT_EQ(resolve_bind("ui.demo", "a.b.c"), "ui.demo.a.b.c");
}

TEST(AriadneBind, ResolveLeadingSlashEscapesPrefix) {
  EXPECT_EQ(resolve_bind("ui.demo", "/global.flag"), "global.flag");
  EXPECT_EQ(resolve_bind("", "/x"), "x");
}

TEST(AriadneBind, ResolveEmptyBindStaysEmpty) {
  EXPECT_EQ(resolve_bind("ui.demo", ""), "");
}

// --- read_or_seed / write ------------------------------------------------------

TEST(AriadneBind, ReadOrSeedSeedsWhenEmpty) {
  cvc::app app;
  EXPECT_EQ(read_or_seed<int>(app, "t.seed", 7), 7);
  EXPECT_EQ(sval(app, "t.seed"), "7"); // seeded into the tree
}

TEST(AriadneBind, ReadOrSeedReadsExistingNotDefault) {
  cvc::app app;
  cvc::state::instance(app)("t.existing").value(3);
  EXPECT_EQ(read_or_seed<int>(app, "t.existing", 9), 3);
}

TEST(AriadneBind, WriteRoundTrips) {
  cvc::app app;
  write<int>(app, "t.w", 5);
  EXPECT_EQ(sval(app, "t.w"), "5");
  write<int>(app, "t.w", 5); // equal write is a no-op; value stays
  EXPECT_EQ(sval(app, "t.w"), "5");
}

// --- sync_scene_visibility: source path -> node `.visible` key -----------------

TEST(AriadneBind, SyncMirrorsSourceToTargetKey) {
  cvc::app app;
  cvc::state::instance(app)("src.vis").value(0);
  std::vector<SceneVisibilityBinding> binds = {{"src.vis", "node.visible", true}};

  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "node.visible"), "0"); // mirrored the hidden source

  cvc::state::instance(app)("src.vis").value(1);
  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "node.visible"), "1"); // tracks the change on the next poll
}

TEST(AriadneBind, SyncSeedsSourceFromDefaultWhenEmpty) {
  cvc::app app;
  // Source has no value yet: the poll seeds it from the node's default and mirrors.
  std::vector<SceneVisibilityBinding> hidden = {{"src.empty.a", "na.visible", false}};
  sync_scene_visibility(app, hidden);
  EXPECT_EQ(sval(app, "src.empty.a"), "0"); // seeded from default_visible=false
  EXPECT_EQ(sval(app, "na.visible"), "0");

  std::vector<SceneVisibilityBinding> shown = {{"src.empty.b", "nb.visible", true}};
  sync_scene_visibility(app, shown);
  EXPECT_EQ(sval(app, "src.empty.b"), "1");
  EXPECT_EQ(sval(app, "nb.visible"), "1");
}

TEST(AriadneBind, SyncHandlesMultipleBindingsIndependently) {
  cvc::app app;
  cvc::state::instance(app)("a.vis").value(1);
  cvc::state::instance(app)("b.vis").value(0);
  std::vector<SceneVisibilityBinding> binds = {
      {"a.vis", "na.visible", true},
      {"b.vis", "nb.visible", true},
  };
  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "na.visible"), "1");
  EXPECT_EQ(sval(app, "nb.visible"), "0");
}

TEST(AriadneBind, SyncIsIdempotentAcrossFrames) {
  cvc::app app;
  cvc::state::instance(app)("c.vis").value(1);
  std::vector<SceneVisibilityBinding> binds = {{"c.vis", "nc.visible", true}};
  for (int i = 0; i < 5; ++i)
    sync_scene_visibility(app, binds); // steady state: stable, no throw
  EXPECT_EQ(sval(app, "nc.visible"), "1");
}

// A checkbox writes int 0/1 to the same resolved path a scene node binds to; verify
// the resolved paths collide so the two stay in lockstep (the whole point of §9).
TEST(AriadneBind, WidgetAndSceneBindShareOneKey) {
  cvc::app app;
  const std::string prefix = "ui.demo";
  const std::string widget_path = resolve_bind(prefix, "show_mesh"); // checkbox bind
  const std::string scene_path = resolve_bind(prefix, "show_mesh");  // scene visible:
  EXPECT_EQ(widget_path, scene_path);

  // Simulate the checkbox committing 0, then the scene poll reading the same key.
  write<int>(app, widget_path, 0);
  std::vector<SceneVisibilityBinding> binds = {{scene_path, "mesh.visible", true}};
  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "mesh.visible"), "0");
}
