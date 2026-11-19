// A swarm of navigating vehicles, run headless (cvc::ariadne::SceneSwarm, scene_swarm.h): the
// simulation of a document's `type: nav_swarm` node, on the synthetic city, on the app's world
// clock, steered through its keys.

#include <chrono>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/scene_swarm.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/world_clock.h>
#include <gtest/gtest.h>
#include <string>
#include <thread>

namespace ari = cvc::ariadne;

namespace {

ari::swarm_spec small_spec() {
  ari::swarm_spec s;
  s.agents = 32;
  s.grid = 48;
  s.targets = 4;
  s.belief = "grouped";
  s.fog = true;
  s.plant = "kinematic";
  s.policy = "default"; // the built-in net: no weights file needed
  return s;
}

// update() until `done` holds, the clock run `quanta` a frame (0: none); false after 60 s.
template <class F> bool run_until(cvc::app &app, ari::SceneSwarm &s, int quanta, F done) {
  const auto t0 = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(60)) {
    if (quanta > 0)
      app.world_clock().advance_ticks(quanta);
    s.update(/*wait=*/true);
    if (done())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

} // namespace

TEST(AriadneSceneSwarm, PropsMakeTheSpec) {
  ari::Value p;
  p.kind = ari::Value::Kind::Map;
  const auto set = [&](const char *k, const char *v) {
    ari::Value x;
    x.scalar = v;
    p.entries.emplace_back(k, x);
  };
  set("agents", "60");
  set("belief", "Private");
  set("fog", "false");
  set("targets", "6");
  set("plant", "humvee");
  set("groups", "convoys");
  const ari::swarm_spec s = ari::swarm_spec::from_props(p);
  EXPECT_EQ(s.agents, 60);
  EXPECT_EQ(s.belief, "private");
  EXPECT_FALSE(s.fog);
  EXPECT_EQ(s.targets, 6);
  EXPECT_EQ(s.plant, "humvee");
  EXPECT_EQ(s.groups, "convoys");
  EXPECT_EQ(s.spawn, "opposed"); // the demos' default, unset here
}

TEST(AriadneSceneSwarm, TheSyntheticCityIsWalled) {
  const ari::swarm_city c = ari::synthetic_swarm_city(48);
  ASSERT_EQ(c.rows, 48);
  ASSERT_EQ(c.cols, 48);
  ASSERT_EQ(c.occ.size(), 48u * 48u);
  EXPECT_FALSE(c.real);
  for (int k = 0; k < c.cols; ++k) {
    EXPECT_NE(c.occ[k], 0);
    EXPECT_NE(c.occ[static_cast<std::size_t>(c.rows - 1) * c.cols + k], 0);
  }
  EXPECT_GT(c.span(), 0.0);
  EXPECT_DOUBLE_EQ(c.ground(0.0, 0.0), 0.0); // flat
}

TEST(AriadneSceneSwarm, BuildsOnTheCityAndColoursItsTargets) {
  cvc::app app;
  ari::SceneSwarm s(app, "doc.swarm", small_spec());
  EXPECT_FALSE(s.running()); // nothing until it has a city
  s.set_city(ari::synthetic_swarm_city(48));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  EXPECT_EQ(s.size(), 32);
  EXPECT_EQ(s.planes(), 2); // grouped: max(2, 32 / 64)
  EXPECT_EQ(s.targets().size(), 2u * 4u);
  EXPECT_EQ(s.target_colors().size(), 3u * 4u);
  ASSERT_EQ(s.colors().size(), 3u * 32u);
  // Each agent wears its target's colour and drives to it.
  for (int i = 0; i < 32; ++i) {
    const int t = s.agent_targets()[i];
    ASSERT_GE(t, 0);
    ASSERT_LT(t, 4);
    for (int k = 0; k < 3; ++k)
      EXPECT_FLOAT_EQ(s.colors()[3 * i + k], s.target_colors()[3 * t + k]);
    EXPECT_NEAR(s.goals()[2 * i], s.targets()[2 * t], 1e-3);
    EXPECT_NEAR(s.goals()[2 * i + 1], s.targets()[2 * t + 1], 1e-3);
  }
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.stats.ready", 0), 1);
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.stats.agents", 0), 32);
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.targets.count", 0), 4);
  EXPECT_FALSE(ari::read_or<std::string>(app, "doc.swarm.targets.0.color", "").empty());
}

TEST(AriadneSceneSwarm, StepsOnTheWorldClock) {
  cvc::app app;
  ari::SceneSwarm s(app, "doc.swarm", small_spec());
  s.set_city(ari::synthetic_swarm_city(48));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  const ari::SceneSwarm::frame start = s.now();
  // One navigator step is 6 quanta of the suite's 0.01 s: 60 quanta are 10 steps.
  ASSERT_TRUE(run_until(app, s, 60, [&] { return s.now().tick >= 50; }));
  const ari::SceneSwarm::frame &f = s.now();
  ASSERT_EQ(f.n, 32);
  double moved = 0.0;
  for (int i = 0; i < f.n; ++i)
    moved += std::hypot(f.pos[2 * i] - start.pos[2 * i], f.pos[2 * i + 1] - start.pos[2 * i + 1]);
  EXPECT_GT(moved, 0.0);
  // The fog: some of the city seen.
  ASSERT_TRUE(run_until(app, s, 60, [&] {
    for (std::uint8_t c : s.coverage())
      if (c)
        return true;
    return false;
  }));
}

TEST(AriadneSceneSwarm, KeysMoveATargetAndFollowAVehicle) {
  cvc::app app;
  ari::SceneSwarm s(app, "doc.swarm", small_spec());
  s.set_city(ari::synthetic_swarm_city(48));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  const std::uint64_t rev = s.targets_revision();
  const ari::swarm_city &c = s.city();
  const double x = c.min_x + 0.25 * c.span(), y = c.min_y + 0.5 * (c.max_y - c.min_y);
  ari::write(app, "doc.swarm.targets.0.x", x);
  ari::write(app, "doc.swarm.targets.0.y", y);
  s.update();
  EXPECT_GT(s.targets_revision(), rev);
  EXPECT_NEAR(s.targets()[0], x, 1e-3);
  for (int i = 0; i < s.size(); ++i)
    if (s.agent_targets()[i] == 0) {
      EXPECT_NEAR(s.goals()[2 * i], x, 1e-3);
      EXPECT_NEAR(s.goals()[2 * i + 1], y, 1e-3);
    }
  // follow_at: the agent nearest the point, and the request consumed.
  const ari::SceneSwarm::frame &f = s.now();
  ASSERT_GT(f.n, 3);
  ari::write(app, "doc.swarm.follow_at", std::to_string(f.pos[6]) + "," + std::to_string(f.pos[7]));
  s.update();
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.follow", -1), s.nearest_agent(f.pos[6], f.pos[7]));
  EXPECT_EQ(ari::read_or<std::string>(app, "doc.swarm.follow_at", ""), ""); // consumed
}

TEST(AriadneSceneSwarm, ARestartBuildsWhatTheKeysSay) {
  cvc::app app;
  ari::SceneSwarm s(app, "doc.swarm", small_spec());
  s.set_city(ari::synthetic_swarm_city(48));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  const std::uint64_t gen = s.generation();
  ari::write(app, "doc.swarm.agents", 20);
  ari::write(app, "doc.swarm.belief", std::string("shared"));
  ari::write(app, "doc.swarm.request", std::string("reseed"));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.generation() != gen; }));
  EXPECT_EQ(s.size(), 20);
  EXPECT_EQ(s.planes(), 1);
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.seed", 0), 8);              // reseed: one more than 7
  EXPECT_EQ(ari::read_or<std::string>(app, "doc.swarm.request", ""), ""); // consumed
}
