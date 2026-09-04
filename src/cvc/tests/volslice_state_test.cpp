/*
  Unit tests for cvc::volslice::state_settings -- the app-state-tree binding
  for the view-aligned slice renderer.

  Handlers are SYNCHRONOUS (setInstanceThreading(false)), so a state write
  applies before the write call returns; seedState() runs in the constructor
  under a re-entry guard and does NOT invoke the apply callback.  Every test
  uses a fresh cvc::app and a distinct state path so tests stay independent
  (the volren_state_test conventions).
*/
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/volslice/state_settings.h>
#include <gtest/gtest.h>
#include <string>

using cvc::volslice::render_settings;
using cvc::volslice::state_settings;

namespace {

cvc::state &treeKey(cvc::app &ctx, const std::string &path, const std::string &key) {
  return cvc::state::instance(ctx)(path + "." + key);
}

} // namespace

TEST(VolsliceStateSettings, SeedWritesDefaults) {
  cvc::app ctx;
  const std::string path = "t1.volslice";
  state_settings ss(ctx, path);

  EXPECT_EQ(state_settings::sceneStatePath("scene"), "scene.volslice");

  EXPECT_NEAR(treeKey(ctx, path, "quality").value<double>(), cvc::volslice::defaults::quality,
              1e-12);
  EXPECT_EQ(treeKey(ctx, path, "max_planes").value<int>(), cvc::volslice::defaults::max_planes);
  EXPECT_NEAR(treeKey(ctx, path, "near_plane").value<double>(), cvc::volslice::defaults::near_plane,
              1e-12);
  EXPECT_EQ(treeKey(ctx, path, "interpolation").value<int>(), int(cvc::volslice::defaults::filter));
  EXPECT_EQ(treeKey(ctx, path, "opacity_correction").value<int>(), 0);
  EXPECT_EQ(treeKey(ctx, path, "tf_auto_domain").value<int>(), 1);
  EXPECT_EQ(treeKey(ctx, path, "window").value(), "");
  EXPECT_EQ(treeKey(ctx, path, "transfer_function.color").value(), "");
  EXPECT_EQ(treeKey(ctx, path, "transfer_function.opacity").value(), "");
}

TEST(VolsliceStateSettings, SetRoundTripsThroughState) {
  cvc::app ctx;
  state_settings ss(ctx, "t2.volslice");

  render_settings s;
  s.slices.quality = 0.75;
  s.slices.max_planes = 500;
  s.slices.near_plane = 0.1;
  s.filter = cvc::volslice::interpolation::nearest;
  s.opacity_correction = true;
  s.tf_auto_domain = false;
  s.window_min = -1.0;
  s.window_max = 3.0;
  s.tf.add({0.0, 1.f, 0.f, 0.f, 0.f});
  s.tf.add({2.0, 0.f, 1.f, 0.f, 0.5f});
  ss.set(s);

  const render_settings back = ss.get();
  EXPECT_NEAR(back.slices.quality, 0.75, 1e-12);
  EXPECT_EQ(back.slices.max_planes, 500);
  EXPECT_NEAR(back.slices.near_plane, 0.1, 1e-12);
  EXPECT_EQ(back.filter, cvc::volslice::interpolation::nearest);
  EXPECT_TRUE(back.opacity_correction);
  EXPECT_FALSE(back.tf_auto_domain);
  EXPECT_NEAR(back.window_min, -1.0, 1e-12);
  EXPECT_NEAR(back.window_max, 3.0, 1e-12);
  ASSERT_EQ(back.tf.points().size(), 2u);
  EXPECT_NEAR(back.tf.points()[1].value, 2.0, 1e-12);
  EXPECT_NEAR(double(back.tf.points()[1].a), 0.5, 1e-6);

  // The tree carries the encoded values (an external writer's view).
  EXPECT_EQ(treeKey(ctx, "t2.volslice", "window").value(), "-1,3");
  EXPECT_EQ(treeKey(ctx, "t2.volslice", "interpolation").value<int>(), 1);
}

TEST(VolsliceStateSettings, StateDrivesApply) {
  cvc::app ctx;
  int applies = 0;
  render_settings last;
  state_settings ss(ctx, "t3.volslice", [&](const render_settings &s) {
    ++applies;
    last = s;
  });
  EXPECT_EQ(applies, 0); // seeding must not fire the callback

  treeKey(ctx, "t3.volslice", "quality").value(std::string("0.9"));
  EXPECT_GE(applies, 1);
  EXPECT_NEAR(last.slices.quality, 0.9, 1e-12);
  EXPECT_NEAR(ss.get().slices.quality, 0.9, 1e-12);
}

TEST(VolsliceStateSettings, SharedTfEncodingParses) {
  // The VolumeNode/volren flat-CSV ramps: color (value,r,g,b) and opacity
  // (value,a), merged at the union of scalars.
  cvc::app ctx;
  state_settings ss(ctx, "t4.volslice");
  treeKey(ctx, "t4.volslice", "transfer_function.color").value(std::string("0,1,0,0,1,0,0,1"));
  treeKey(ctx, "t4.volslice", "transfer_function.opacity").value(std::string("0,0,0.5,0.8,1,0"));

  const render_settings s = ss.get();
  ASSERT_EQ(s.tf.points().size(), 3u); // scalars {0, 0.5, 1}
  EXPECT_NEAR(s.tf.points()[1].value, 0.5, 1e-12);
  EXPECT_NEAR(double(s.tf.points()[1].a), 0.8, 1e-6);
  // Color at 0.5 is the midpoint of the red->blue ramp.
  EXPECT_NEAR(double(s.tf.points()[1].r), 0.5, 1e-6);
  EXPECT_NEAR(double(s.tf.points()[1].b), 0.5, 1e-6);
}

TEST(VolsliceStateSettings, MalformedStateKeepsLastGood) {
  cvc::app ctx;
  state_settings ss(ctx, "t5.volslice");
  treeKey(ctx, "t5.volslice", "quality").value(std::string("0.6"));
  EXPECT_NEAR(ss.get().slices.quality, 0.6, 1e-12);

  // Junk in one key: the WHOLE read is rejected, nothing partial applies.
  treeKey(ctx, "t5.volslice", "window").value(std::string("banana"));
  EXPECT_NEAR(ss.get().slices.quality, 0.6, 1e-12);
  EXPECT_NEAR(ss.get().window_min, 0.0, 1e-12);

  // Unknown enum value: rejected, last good kept.
  treeKey(ctx, "t5.volslice", "interpolation").value(7);
  EXPECT_EQ(ss.get().filter, cvc::volslice::defaults::filter);
}

TEST(VolsliceStateSettings, MaxPlanesClampsOnRead) {
  cvc::app ctx;
  state_settings ss(ctx, "t6.volslice");
  treeKey(ctx, "t6.volslice", "max_planes").value(1000000);
  EXPECT_EQ(ss.get().slices.max_planes, cvc::volslice::limits::max_max_planes);
  treeKey(ctx, "t6.volslice", "max_planes").value(-5);
  EXPECT_EQ(ss.get().slices.max_planes, cvc::volslice::limits::min_max_planes);
}
