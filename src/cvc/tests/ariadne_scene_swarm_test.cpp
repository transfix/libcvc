// A swarm of navigating vehicles, run headless (cvc::ariadne::SceneSwarm, scene_swarm.h): the
// simulation of a document's `type: nav_swarm` node, on the synthetic city, on the app's world
// clock, steered through its keys; and the fog-of-war ghost story on an arena (a stale map's
// phantom wall, listed vehicles, belief routes).

#include <algorithm>
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
  // One navigator step is 7 of the suite's 1/120 s quanta (cvc::kSimTickQuanta).
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

// Without a worker (a browser without threads), the frame steps the navigator on the clock: its
// tick, 7 of the suite's quanta. It followed the training scene's 0.06 s, 7.2 quanta, which the
// clock refused, so the swarm never moved.
TEST(AriadneSceneSwarm, StepsOnTheFramesThreadWithoutAWorker) {
  cvc::app app;
  ari::swarm_spec spec = small_spec();
  spec.worker = false;
  ari::SceneSwarm s(app, "doc.swarm", spec);
  s.set_city(ari::synthetic_swarm_city(48));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  const ari::SceneSwarm::frame start = s.now();
  // 70 quanta are 10 navigator steps.
  app.world_clock().advance_ticks(70);
  s.update(/*wait=*/true);
  const ari::SceneSwarm::frame &f = s.now();
  ASSERT_EQ(f.n, 32);
  EXPECT_EQ(f.tick, start.tick + 10) << "one step per navigator tick, on the frame's thread";
  double moved = 0.0;
  for (int i = 0; i < f.n; ++i)
    moved += std::hypot(f.pos[2 * i] - start.pos[2 * i], f.pos[2 * i + 1] - start.pos[2 * i + 1]);
  EXPECT_GT(moved, 0.0);
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

// ---- the ghost story: an arena, a stale map, listed vehicles, belief routes ------------------

namespace {

// A map node from (key, value) pairs; a value is a scalar's text, or a Value.
ari::Value scalar(const std::string &v) {
  ari::Value x;
  x.scalar = v;
  return x;
}
ari::Value seq(std::initializer_list<ari::Value> items) {
  ari::Value x;
  x.kind = ari::Value::Kind::Sequence;
  x.items.assign(items.begin(), items.end());
  return x;
}
ari::Value nums(std::initializer_list<double> v) {
  ari::Value x;
  x.kind = ari::Value::Kind::Sequence;
  for (double d : v)
    x.items.push_back(scalar(std::to_string(d)));
  return x;
}
ari::Value map(std::initializer_list<std::pair<std::string, ari::Value>> entries) {
  ari::Value x;
  x.kind = ari::Value::Kind::Map;
  for (const auto &e : entries)
    x.entries.emplace_back(e.first, e.second);
  return x;
}

// The fog-ghost arena: 200 m square, an open field inside its border, and a stale map with a
// phantom wall across the middle, 100 m long.
ari::swarm_arena ghost_arena() {
  ari::swarm_arena a;
  a.phantoms.push_back({-0.5, -50.0, 0.5, 50.0});
  return a;
}

// One vehicle crossing the arena left to right through the phantom, on a belief route.
ari::swarm_spec ghost_spec(bool worker) {
  ari::swarm_spec s;
  s.grid = 120;
  s.belief = "shared";
  s.fog = true;
  s.plant = "kinematic";
  s.policy = "default";
  s.reach = 0.2;
  s.range = 35.0;
  s.sense_ticks = 2; // as the C++ demo sensed: enough rays to wear the phantom past half
  s.route = "belief";
  s.worker = worker;
  ari::swarm_vehicle v;
  v.start[0] = -80.0;
  v.start[1] = -6.0;
  v.goal[0] = 80.0;
  v.goal[1] = 6.0;
  s.vehicles.push_back(v);
  s.agents = 1;
  s.has_arena = true;
  s.arena = ghost_arena();
  return s;
}

// The farthest any of the route's waypoints is from the straight line y = 0, in metres.
double route_bow(const std::vector<float> &r) {
  double most = 0.0; // not `far`: windows.h defines it away
  for (std::size_t k = 0; k + 1 < r.size(); k += 2)
    most = std::max(most, std::fabs(static_cast<double>(r[k + 1])));
  return most;
}

} // namespace

TEST(AriadneSceneSwarm, TheArenaIsWalledAndItsPhantomsAreInItsMapOnly) {
  ari::swarm_arena a = ghost_arena();
  a.walls.push_back({30.0, 30.0, 30.2, 40.0}); // thinner than a cell: still a column of it
  const ari::swarm_city c = ari::arena_swarm_city(a, 120);
  ASSERT_EQ(c.rows, 120);
  ASSERT_EQ(c.cols, 120);
  EXPECT_TRUE(c.arena);
  EXPECT_FALSE(c.real);
  EXPECT_DOUBLE_EQ(c.wall_h, 6.0);
  EXPECT_DOUBLE_EQ(c.scale, 0.05); // metres at the trained metric
  ASSERT_EQ(c.occ.size(), 120u * 120u);
  ASSERT_EQ(c.prior.size(), c.occ.size());
  for (int k = 0; k < c.cols; ++k) {
    EXPECT_NE(c.occ[k], 0);
    EXPECT_NE(c.occ[static_cast<std::size_t>(c.rows - 1) * c.cols + k], 0);
  }
  // The phantom: column 60 (x 0.84), rows 30-89 (y -49.6 to 49.6), in the map and not the arena.
  int phantoms = 0, wall = 0;
  for (std::size_t i = 0; i < c.occ.size(); ++i) {
    ASSERT_TRUE(!c.occ[i] || c.prior[i]) << "the map has every wall the arena has";
    if (c.prior[i] && !c.occ[i]) {
      ++phantoms;
      EXPECT_EQ(static_cast<int>(i % c.cols), 60);
      const int r = static_cast<int>(i / c.cols);
      EXPECT_GE(r, 30);
      EXPECT_LE(r, 89);
    }
    const int r = static_cast<int>(i / c.cols), k = static_cast<int>(i % c.cols);
    if (c.occ[i] && r > 0 && k > 0 && r < c.rows - 1 && k < c.cols - 1)
      ++wall;
  }
  EXPECT_EQ(phantoms, 60);
  EXPECT_EQ(wall, 6) << "the thin wall: one column, rows 78-83";
  // No phantoms: no stale map.
  EXPECT_TRUE(ari::arena_swarm_city(ari::swarm_arena{}, 64).prior.empty());
  // A wide arena: square cells, its longer side the grid.
  ari::swarm_arena wide;
  wide.min_x = -100.0;
  wide.max_x = 100.0;
  wide.min_y = -50.0;
  wide.max_y = 50.0;
  const ari::swarm_city w = ari::arena_swarm_city(wide, 101);
  EXPECT_EQ(w.cols, 101);
  EXPECT_EQ(w.rows, 51);
  EXPECT_NEAR((w.max_x - w.min_x) / (w.cols - 1), (w.max_y - w.min_y) / (w.rows - 1), 1e-9);
}

TEST(AriadneSceneSwarm, PropsMakeAnArenaListedVehiclesAndARoute) {
  const ari::Value p = map({
      {"arena", map({{"size", nums({160, 120})},
                     {"walls", seq({nums({-10, -10, 10, 10})})},
                     {"phantoms", seq({nums({-1, -40, 1, 40}), nums({1, 2, 3})})}, // one too short
                     {"wall_height", scalar("4")},
                     {"name", scalar("the yard")}})},
      {"vehicles", seq({map({{"start", nums({-70, 0})},
                             {"goal", nums({70, 0})},
                             {"color", nums({1, 0.5, 0})},
                             {"name", scalar("hero")}}),
                        map({{"start", nums({0, 50})}, {"goal", nums({0, -50})}}),
                        map({{"start", nums({0, 50})}})})}, // no goal: skipped
      {"route", scalar("Belief")},
      {"range", scalar("35")},
  });
  const ari::swarm_spec s = ari::swarm_spec::from_props(p);
  ASSERT_TRUE(s.has_arena);
  EXPECT_DOUBLE_EQ(s.arena.min_x, -80.0);
  EXPECT_DOUBLE_EQ(s.arena.max_y, 60.0);
  EXPECT_EQ(s.arena.walls.size(), 1u);
  EXPECT_EQ(s.arena.phantoms.size(), 1u);
  EXPECT_DOUBLE_EQ(s.arena.wall_height, 4.0);
  EXPECT_EQ(s.arena.name, "the yard");
  ASSERT_EQ(s.vehicles.size(), 2u);
  EXPECT_TRUE(s.vehicles[0].has_color);
  EXPECT_FLOAT_EQ(s.vehicles[0].color[1], 0.5f);
  EXPECT_EQ(s.vehicles[0].name, "hero");
  EXPECT_FALSE(s.vehicles[1].has_color);
  EXPECT_EQ(s.agents, 2) << "agents defaults to the vehicles listed";
  EXPECT_EQ(s.route, "belief");
  EXPECT_DOUBLE_EQ(s.range, 35.0);
  EXPECT_EQ(ari::swarm_spec::from_props(map({{"route", scalar("astar")}})).route, "none");
}

TEST(AriadneSceneSwarm, ListedVehiclesAreTheirOwnTargets) {
  cvc::app app;
  ari::swarm_spec spec = ghost_spec(true);
  spec.route = "none";
  ari::swarm_vehicle crosser;
  crosser.start[0] = -10.0;
  crosser.start[1] = 78.0;
  crosser.goal[0] = -10.0;
  crosser.goal[1] = -78.0;
  crosser.color[0] = 0.35f;
  crosser.color[1] = 0.72f;
  crosser.color[2] = 0.95f;
  crosser.has_color = true;
  spec.vehicles.push_back(crosser);
  spec.agents = 2;
  ari::SceneSwarm s(app, "doc.swarm", spec);
  s.set_city(ari::arena_swarm_city(spec.arena, spec.grid));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  ASSERT_EQ(s.size(), 2);
  EXPECT_EQ(s.planes(), 1);
  ASSERT_EQ(s.targets().size(), 4u);
  EXPECT_NEAR(s.goals()[0], 80.0, 0.01);
  EXPECT_NEAR(s.goals()[1], 6.0, 0.01);
  EXPECT_NEAR(s.goals()[3], -78.0, 0.01);
  EXPECT_FLOAT_EQ(s.colors()[3], 0.35f);
  EXPECT_FLOAT_EQ(s.colors()[5], 0.95f);
  EXPECT_EQ(s.agent_targets()[1], 1);
  EXPECT_DOUBLE_EQ(s.vehicle_length(), 4.72) << "an arena is in metres";
  EXPECT_DOUBLE_EQ(s.wall_height(), 6.0);
  EXPECT_DOUBLE_EQ(s.sensor_range(), 35.0);
  ASSERT_TRUE(run_until(app, s, 60, [&] { return s.now().tick > 3; }));
  EXPECT_NEAR(s.now().pos[0], -80.0, 10.0);
  EXPECT_EQ(s.now().carrot.size(), 4u) << "each agent's carrot in the frame";
  // Without the phantoms key the agents set out knowing the arena: no ghosts.
  EXPECT_EQ(s.ghosts(), 60);
  ari::write(app, "doc.swarm.phantoms", 0);
  ari::write(app, "doc.swarm.agents", 1);
  const std::uint64_t gen = s.generation();
  ari::write(app, "doc.swarm.request", std::string("restart"));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.generation() != gen; }));
  EXPECT_EQ(s.size(), 1) << "agents: the first of the listed vehicles";
  EXPECT_EQ(s.ghosts(), 0);
  EXPECT_EQ(s.ghosts_at_build(), 0);
}

namespace {

// The ghost story: the vehicle believes a wall that is not there, so its route bows round it;
// as its sensor clears the phantom the route is planned again, straight; and it arrives.
void run_ghost_story(bool worker) {
  cvc::app app;
  const ari::swarm_spec spec = ghost_spec(worker);
  ari::SceneSwarm s(app, "doc.swarm", spec);
  s.set_city(ari::arena_swarm_city(spec.arena, spec.grid));
  ASSERT_TRUE(run_until(app, s, 0, [&] { return s.running(); }));
  EXPECT_EQ(s.ghosts_at_build(), 60);
  EXPECT_EQ(s.ghosts(), 60);
  EXPECT_EQ(ari::read_or<int>(app, "doc.swarm.stats.ghosts0", 0), 60);
  ASSERT_EQ(s.routes().size(), 1u);
  const std::vector<float> first = s.routes()[0];
  ASSERT_GE(first.size(), 4u);
  EXPECT_NEAR(first[first.size() - 2], 80.0, 0.01) << "a route ends at its goal";
  EXPECT_GT(route_bow(first), 45.0) << "the believed wall bends the route round its end";
  ASSERT_EQ(s.route_cursor().size(), 1u);
  const std::uint64_t rev = s.routes_revision();
  // Belief: amber where the map blocks and the arena does not.
  const auto &b = s.belief();
  ASSERT_EQ(b.size(), 120u * 120u);
  EXPECT_EQ(b[static_cast<std::size_t>(60) * 120 + 60], 2) << "a ghost";
  EXPECT_EQ(b[0], 1) << "the border: a wall it knows";
  EXPECT_EQ(b[static_cast<std::size_t>(60) * 120 + 30], 0);

  // Drive: the sensor clears the phantom, the route straightens, and the vehicle gets there.
  int least = 60;
  double straight = 1e9;
  ASSERT_TRUE(run_until(app, s, 35,
                        [&] {
                          least = std::min(least, s.ghosts());
                          if (s.routes_revision() != rev && !s.routes().empty())
                            straight = std::min(straight, route_bow(s.routes()[0]));
                          return s.now().arrived == 1;
                        }))
      << "ghosts " << s.ghosts() << ", at " << s.now().pos[0] << ", " << s.now().pos[1];
  // Its sensor wears away the end it heads for, and the route goes straight through the gap; the
  // far end never comes within the sensor's range, and stands.
  EXPECT_LE(least, 45) << "the sensor wore a quarter of the phantom away";
  EXPECT_GT(s.ghosts(), 0) << "the far end, out of the sensor's range, still stands";
  EXPECT_LT(straight, 20.0) << "once it cleared, the route went through where it stood";
  EXPECT_GT(ari::read_or<int>(app, "doc.swarm.stats.replans", 0), 1);
  EXPECT_NEAR(s.now().pos[0], 80.0, 6.0);
  EXPECT_NEAR(s.now().pos[1], 6.0, 6.0);
  EXPECT_GT(ari::read_or<double>(app, "doc.swarm.stats.time", 0.0), 1.0);
}

} // namespace

TEST(AriadneSceneSwarm, AStaleMapsGhostBendsTheRouteUntilSensedClear) { run_ghost_story(true); }

TEST(AriadneSceneSwarm, AStaleMapsGhostBendsTheRouteOnTheFramesThread) { run_ghost_story(false); }
