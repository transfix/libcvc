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

TEST(AriadneBind, ResolveEmptyPrefixIsIdentity) { EXPECT_EQ(resolve_bind("", "a.b"), "a.b"); }

TEST(AriadneBind, ResolvePrefixSplices) {
  EXPECT_EQ(resolve_bind("ui.demo", "wire"), "ui.demo.wire");
  EXPECT_EQ(resolve_bind("ui.demo", "a.b.c"), "ui.demo.a.b.c");
}

TEST(AriadneBind, ResolveLeadingSlashEscapesPrefix) {
  EXPECT_EQ(resolve_bind("ui.demo", "/global.flag"), "global.flag");
  EXPECT_EQ(resolve_bind("", "/x"), "x");
}

TEST(AriadneBind, ResolveEmptyBindStaysEmpty) { EXPECT_EQ(resolve_bind("ui.demo", ""), ""); }

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

TEST(AriadneBind, ReadOrDoesNotSeed) {
  cvc::app app;
  EXPECT_EQ(read_or<int>(app, "ro.unset", 5), 5);
  EXPECT_EQ(sval(app, "ro.unset"), ""); // returns default WITHOUT writing the key
  cvc::state::instance(app)("ro.set").value(2);
  EXPECT_EQ(read_or<int>(app, "ro.set", 5), 2);
}

// A bool key seeded by state_exec ("true"/"false") or as an int ("1"/"0") must read the same 0/1 —
// an int-backed bool widget must not silently desync on the "true"/"false" spelling. to_bool_int
// tolerates both (+ yes/no/on/off + numeric truthiness); a non-bool string -> the default.
TEST(AriadneBind, BoolReadTolerantOfTrueFalseSpelling) {
  EXPECT_EQ(to_bool_int("true", 0), 1);
  EXPECT_EQ(to_bool_int("false", 1), 0);
  EXPECT_EQ(to_bool_int("True", 0), 1); // case-insensitive
  EXPECT_EQ(to_bool_int(" false ", 1), 0);
  EXPECT_EQ(to_bool_int("1", 0), 1);
  EXPECT_EQ(to_bool_int("0", 1), 0);
  EXPECT_EQ(to_bool_int("yes", 0), 1);
  EXPECT_EQ(to_bool_int("off", 1), 0);
  EXPECT_EQ(to_bool_int("7", 0), 1);    // any nonzero numeric is truthy
  EXPECT_EQ(to_bool_int("junk", 1), 1); // unparsable -> the default
  EXPECT_EQ(to_bool_int("junk", 0), 0);

  cvc::app app;
  // read_bool_or_seed reads a "true" seed as checked (1), unlike read_or_seed<int> which would
  // throw and fall back to the default (leaving the widget unchecked while the bound scene node is
  // shown).
  cvc::state::instance(app)("b.t").value(std::string("true"));
  EXPECT_EQ(read_bool_or_seed(app, "b.t", 0), 1);
  EXPECT_EQ(read_bool_or(app, "b.t", 0), 1);
  // A slider_int value is NOT bool-coerced by these (it stays on read_or_seed<int>) — guard the
  // intent: to_bool_int would wrongly map 128 -> 1, so bool reads must never wrap a real int field.
  EXPECT_EQ(read_or_seed<int>(app, "b.count", 0),
            0); // unset -> default; (documents that sliders use the int read, not the bool read)
  // An unset bool key seeds its default like read_or_seed.
  EXPECT_EQ(read_bool_or_seed(app, "b.seed", 1), 1);
  EXPECT_EQ(sval(app, "b.seed"), "1");
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

TEST(AriadneBind, SyncFallsBackToDefaultWithoutSeedingSource) {
  cvc::app app;
  // Source has no value yet (no widget owns it): the node falls back to its own
  // default, and the source is NOT seeded (the scene bind is a follower, not owner).
  std::vector<SceneVisibilityBinding> hidden = {{"src.empty.a", "na.visible", false}};
  sync_scene_visibility(app, hidden);
  EXPECT_EQ(sval(app, "src.empty.a"), ""); // NOT seeded
  EXPECT_EQ(sval(app, "na.visible"), "0"); // fallback to default_visible=false

  std::vector<SceneVisibilityBinding> shown = {{"src.empty.b", "nb.visible", true}};
  sync_scene_visibility(app, shown);
  EXPECT_EQ(sval(app, "src.empty.b"), ""); // NOT seeded
  EXPECT_EQ(sval(app, "nb.visible"), "1"); // fallback to default_visible=true
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

// Regression (review §9 inc2): the scene `visible:` bind is a FOLLOWER — sync must
// NOT seed the source key, so it can never pre-empt the widget that owns it.
TEST(AriadneBind, SyncDoesNotSeedTheSource) {
  cvc::app app;
  std::vector<SceneVisibilityBinding> binds = {{"owner.key", "node.visible", true}};
  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "owner.key"), "");     // source left untouched (not seeded)
  EXPECT_EQ(sval(app, "node.visible"), "1"); // node falls back to its own default
}

// Regression: the node's default must NOT override the key owner's value. A checkbox
// with def:false seeds the key to 0; the bound node's default_visible=true must lose.
TEST(AriadneBind, SyncFollowsKeyOwnerNotNodeDefault) {
  cvc::app app;
  write<int>(app, "owner.key", 0); // the widget (owner) committed 0
  std::vector<SceneVisibilityBinding> binds = {{"owner.key", "node.visible", true}};
  sync_scene_visibility(app, binds);
  EXPECT_EQ(sval(app, "node.visible"), "0"); // owner's 0 wins over node default true
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

// --- sync_scene_clip: bound clip `offset:`s -> the node's `clip_planes` key ----

namespace {
SceneClipBinding::Plane clipPlane(double ox, double oy, double oz, double nx, double ny, double nz,
                                  double offset = 0.0, const std::string &source = "") {
  SceneClipBinding::Plane p;
  p.origin[0] = ox, p.origin[1] = oy, p.origin[2] = oz;
  p.normal[0] = nx, p.normal[1] = ny, p.normal[2] = nz;
  p.offset = offset;
  p.offset_source = source;
  return p;
}
} // namespace

TEST(AriadneBind, ClipSyncSlidesABoundPlaneAlongItsNormal) {
  cvc::app app;
  // A literal plane, then one bound to sim.cut through (1, 0, 0) along +z.
  std::vector<SceneClipBinding> binds = {
      {"node.clip_planes",
       {clipPlane(0, 0, 0, 1, 0, 0), clipPlane(1, 0, 0, 0, 0, 1, 0.5, "sim.cut")},
       ""}};
  sync_scene_clip(app, binds);
  // Unset source: the default offset 0.5, and the source is NOT seeded (a follower).
  EXPECT_EQ(sval(app, "node.clip_planes"), "0,0,0,1,0,0,1,0,0.5,0,0,1");
  EXPECT_EQ(sval(app, "sim.cut"), "");

  cvc::state::instance(app)("sim.cut").value(-2.25);
  sync_scene_clip(app, binds);
  EXPECT_EQ(sval(app, "node.clip_planes"), "0,0,0,1,0,0,1,0,-2.25,0,0,1");

  // Unchanged: nothing is written (an external write to the key stays until the offset moves).
  cvc::state::instance(app)("node.clip_planes").value(std::string("external"));
  sync_scene_clip(app, binds);
  EXPECT_EQ(sval(app, "node.clip_planes"), "external");
  cvc::state::instance(app)("sim.cut").value(3);
  sync_scene_clip(app, binds);
  EXPECT_EQ(sval(app, "node.clip_planes"), "0,0,0,1,0,0,1,0,3,0,0,1");
}

TEST(AriadneBind, ClipSyncTreatsANonNumberAsUnset) {
  cvc::app app;
  std::vector<SceneClipBinding> binds = {
      {"n.clip_planes", {clipPlane(0, 0, 0, 0, 1, 0, 0.75, "sim.cut")}, ""}};
  cvc::state::instance(app)("sim.cut").value(std::string("not a number"));
  sync_scene_clip(app, binds);
  EXPECT_EQ(sval(app, "n.clip_planes"), "0,0.75,0,0,1,0"); // the default offset
}

TEST(AriadneBind, ClipPlanesCsvRoundTripsDoubles) {
  const std::vector<SceneClipBinding::Plane> planes = {clipPlane(0.1, 0, 0, 1, 0, 0)};
  EXPECT_EQ(clip_planes_csv(planes, {0.0}), "0.10000000000000001,0,0,1,0,0");
  EXPECT_EQ(clip_planes_csv(planes, {}), "0.10000000000000001,0,0,1,0,0"); // the plane's own offset
}
