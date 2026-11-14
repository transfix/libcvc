/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// The world's rules (ARIADNE_WORLDS.md WD2): the world: block, revisioned sections, and the four
// rules sections on a room's wire.

#include <cvc/ariadne/conference/room_rules.h>
#include <cvc/ariadne/loader.h> // load_string, load_file
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/section.h>
#include <cvc/ariadne/value.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/core/app.h>
#include <gtest/gtest.h>
#include <string>
#include <utility>
#include <vector>

namespace ca = cvc::ariadne;
namespace rc = cvc::ariadne::conference;

namespace {
// A document checked against a fresh app's registries: the built-in widget types, blocks and
// decoders only. (A test that registers its own loads against that app instead.)
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load(const std::string &yaml) {
  cvc::app app;
  return cvc::ariadne::load_string(app, yaml);
}
[[maybe_unused]] cvc::ariadne::LoadResult fresh_load_file(const std::string &path) {
  cvc::app app;
  return cvc::ariadne::load_file(app, path);
}
} // namespace

namespace {

bool any_warning(const std::vector<std::string> &w, const std::string &needle) {
  for (const std::string &s : w)
    if (s.find(needle) != std::string::npos)
      return true;
  return false;
}

// A document's rules, or "" and the error in WHY.
ca::world_rules rules_of(const std::string &yaml, std::string &why,
                         std::vector<std::string> *warnings = nullptr) {
  cvc::app app;
  ca::register_world_block(app);
  const ca::LoadResult lr = ca::load_string(app, yaml);
  ca::world_rules r;
  if (!lr.ok) {
    why = lr.error;
    return r;
  }
  why = ca::world_rules_from(lr.scene, ca::find_world_block(lr), r, warnings);
  return r;
}

} // namespace

// What the conference had compiled in before WD2 is the default world's rules.
TEST(WorldRulesTest, TheDefaultsAreWhatTheConferenceHadCompiledIn) {
  const ca::world_rules r = ca::default_world_rules();
  EXPECT_EQ(ca::check_world_rules(r), "");
  EXPECT_EQ(r.avatar().id, "person");
  EXPECT_EQ(r.avatar().body.mass, 70.0);
  EXPECT_EQ(r.avatar().body.width, 0.5);
  EXPECT_EQ(r.avatar().body.height, 1.7);
  EXPECT_EQ(r.controller().id, "walker@1");
  EXPECT_EQ(r.controller().kernel, "walker");
  EXPECT_EQ(r.controller().version, 1);
  EXPECT_EQ(r.controller().param("walk_speed", 0.0), 2.0);
  EXPECT_EQ(r.policy.max_speed, 4.0);
  EXPECT_FALSE(r.policy.allow_vertical);
  EXPECT_TRUE(r.policy.energy == ca::avatar_energy_policy{});
  EXPECT_EQ(r.physics.gravity.z, -9.80665);
  EXPECT_TRUE(ca::controller_kernel_known("walker", 1));
  EXPECT_FALSE(ca::controller_kernel_known("walker", 2));
}

TEST(WorldRulesTest, TheWorldBlockSetsTheRules) {
  if (!ca::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  std::string why;
  std::vector<std::string> warnings;
  const ca::world_rules r = rules_of(R"(
scene:
  physics: { gravity: [0, 0, -3.72] }
world:
  avatars:
    archetypes:
      pilot: { body: { mass: 90, width: 0.6, height: 1.9 }, controllers: [stroll@1, walker@1] }
      child: { body: { mass: 30, height: 1.2 } }
    policy:
      max_speed: 6
      allow_vertical: yes
      energy: { reserve_max: 5000, empty: stop }
  controllers:
    stroll@1: { kernel: walker, params: { walk_speed: 1.2 } }
)",
                                     why, &warnings);
  ASSERT_EQ(why, "");
  EXPECT_TRUE(warnings.empty()) << warnings.front();
  EXPECT_EQ(r.physics.gravity.z, -3.72) << "gravity is the scene's physics:";
  ASSERT_EQ(r.archetypes.size(), 2u) << "the document's archetypes replace the default";
  EXPECT_EQ(r.avatar().id, "pilot") << "the first is every avatar's";
  EXPECT_EQ(r.avatar().body.mass, 90.0);
  EXPECT_EQ(r.avatar().body.height, 1.9);
  EXPECT_EQ(r.archetypes[1].body.width, 0.5) << "a body key left out keeps the default";
  EXPECT_EQ(r.archetypes[1].controllers, std::vector<std::string>{"walker@1"})
      << "and so do controllers left out";
  ASSERT_EQ(r.controllers.size(), 2u) << "the document's controllers join the default walker@1";
  EXPECT_EQ(r.controller().id, "stroll@1") << "the avatar's: the first its archetype allows";
  EXPECT_EQ(r.controller().kernel, "walker");
  EXPECT_EQ(r.controller().version, 1) << "the version from the id";
  EXPECT_EQ(r.controller().param("walk_speed", 0.0), 1.2);
  EXPECT_EQ(r.policy.max_speed, 6.0);
  EXPECT_TRUE(r.policy.allow_vertical);
  EXPECT_EQ(r.policy.energy.reserve_max, 5000.0);
  EXPECT_EQ(r.policy.energy.empty, ca::avatar_energy_policy::when_empty::stop);
  EXPECT_EQ(r.policy.energy.force_max, ca::avatar_energy_policy{}.force_max)
      << "an energy key left out keeps the default";
}

TEST(WorldRulesTest, WhatItDoesNotKnowIsAWarningAndWhatCannotRunAnError) {
  if (!ca::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  std::string why;
  std::vector<std::string> warnings;
  rules_of(R"(
scene:
  physics: { gravity: [0, 0, -9.8], fixed_dt: 0.01, materials: { ice: { friction: 0.05 } } }
world:
  weather: rain
  avatars:
    catalog: { colors: auto }
    policy: { max_speed: 4, sprint: 9 }
)",
           why, &warnings);
  EXPECT_EQ(why, "");
  EXPECT_TRUE(any_warning(warnings, "world: unknown key 'weather'"));
  EXPECT_TRUE(any_warning(warnings, "catalog: is not read yet"));
  EXPECT_TRUE(any_warning(warnings, "policy: unknown key 'sprint'"));
  EXPECT_TRUE(any_warning(warnings, "fixed_dt 0.01 is ignored"));
  EXPECT_TRUE(any_warning(warnings, "materials wait for the world runtime"));

  const std::pair<const char *, const char *> bad[] = {
      {"world: { avatars: { archetypes: { a: { controllers: [runner@1] } } } }",
       "allows controller 'runner@1', which controllers: does not define"},
      {"world: { controllers: { runner@1: {} } }", "this build has no kernel runner@1"},
      {"world: { controllers: { walker@2: {} } }", "this build has no kernel walker@2"},
      {"world: { avatars: { archetypes: { a: { body: { mass: -1 } } } } }",
       "body mass, width and height must be positive"},
      {"world: { avatars: { archetypes: { a: { body: { height: tall } } } } }",
       "height: expects a number"},
      {"world: { avatars: { archetypes: {} } }", "expects a map of archetypes by id"},
      {"world: { avatars: { archetypes: { a: { controllers: [] } } } }", "allows no controller"},
      {"world: { avatars: { policy: { max_speed: fast } } }", "max_speed: expects a number"},
      {"world: { avatars: { policy: { max_speed: -1 } } }", "max_speed must be zero or more"},
      {"world: { avatars: { policy: { allow_vertical: maybe } } }", "expects true or false"},
      {"world: { avatars: { policy: { energy: { empty: sleep } } } }", "expects rest or stop"},
      {"world: { avatars: { policy: { energy: { recover_fraction: 2 } } } }",
       "recover_fraction must be from 0 to 1"},
      {"world: [a, b]", "world: expects a map"},
  };
  for (const auto &[yaml, needle] : bad) {
    SCOPED_TRACE(yaml);
    rules_of(yaml, why);
    EXPECT_NE(why.find(needle), std::string::npos) << why;
  }
}

// The stock worlds state the default rules in their world: blocks, which is what the relay runs
// when it seeds a world's C++ twin.
TEST(WorldRulesTest, TheStockWorldsRunTheDefaultRules) {
  cvc::app app;
  if (!ca::have_yaml())
    GTEST_SKIP() << "built without yaml-cpp";
  ca::register_world_block(app);
  for (const char *file : {"default.ari", "large.ari"}) {
    SCOPED_TRACE(file);
    const ca::LoadResult lr = ca::load_file(app, std::string(CVC_ARIADNE_WORLDS_DIR) + "/" + file);
    ASSERT_TRUE(lr.ok) << lr.error;
    ASSERT_NE(ca::find_world_block(lr), nullptr) << "it has a world: block";
    std::vector<std::string> warnings;
    ca::world_rules r;
    ASSERT_EQ(ca::world_rules_from(lr.scene, ca::find_world_block(lr), r, &warnings), "");
    EXPECT_TRUE(warnings.empty()) << warnings.front();
    EXPECT_TRUE(r == ca::default_world_rules());
  }
}

// A commit writes the section's epoch, its activation tick and, last, its revision; a new epoch (a
// restarted relay) starts the revisions again.
TEST(SectionTest, ACommitStampsTheSection) {
  cvc::app app;
  const std::string base = "conf.rooms.r.thing";
  EXPECT_FALSE(ca::read_section_stamp(app, base).written());
  ca::section_writer w(app, base);
  w.set({"name"}, "a");
  w.set_double({"size"}, 0.1);
  w.set_uint({"n"}, 7);
  w.set_flag({"on"}, true);
  const ca::section_stamp s1 = w.commit(40);
  EXPECT_EQ(s1.rev, 1u);
  EXPECT_EQ(s1.at, 40u);
  EXPECT_EQ(s1.epoch, ca::section_epoch(app));
  EXPECT_NE(ca::section_epoch(app), 0u);
  EXPECT_LT(ca::section_epoch(app), std::uint64_t{1} << 53);
  const ca::section_stamp read1 = ca::read_section_stamp(app, base);
  EXPECT_TRUE(read1.same(s1));
  EXPECT_EQ(read1.at, 40u);

  const ca::section_reader r(app, base);
  EXPECT_EQ(r.text({"name"}), "a");
  EXPECT_EQ(r.number({"size"}, 0.0), 0.1) << "a double round-trips exactly";
  EXPECT_EQ(r.uint({"n"}, 0), 7u);
  EXPECT_TRUE(r.flag({"on"}, false));
  EXPECT_FALSE(r.has({"missing"}));
  EXPECT_EQ(r.number({"missing"}, 2.5), 2.5);

  const ca::section_stamp s2 = w.commit(0);
  EXPECT_EQ(s2.rev, 2u);
  EXPECT_FALSE(s2.same(s1));
  const ca::section_stamp other = w.commit(0, ca::section_epoch(app) ^ 1);
  EXPECT_EQ(other.rev, 1u) << "another run's revisions start at 1";
  EXPECT_FALSE(other.same(s1)) << "and its epoch tells them apart";
}

TEST(RoomRulesTest, EachSectionRoundTrips) {
  cvc::app app;
  ca::world_rules r = ca::default_world_rules();
  r.physics.gravity = cvc::physics::vec3{0.1, -0.2, -1.62};
  ca::avatar_archetype tall;
  tall.id = "tall";
  tall.body = ca::avatar_body{95.0, 0.55, 2.1};
  tall.controllers = {"stroll@1", "walker@1"};
  r.archetypes.push_back(tall);
  ca::avatar_controller stroll;
  stroll.id = "stroll@1";
  stroll.params = {{"walk_speed", 1.1}, {"turn", 0.3}};
  r.controllers.push_back(stroll);
  r.policy.max_speed = 5.5;
  r.policy.allow_vertical = true;
  r.policy.energy.enabled = false;
  r.policy.energy.empty = ca::avatar_energy_policy::when_empty::stop;
  r.policy.energy.transport_cost = 0.125;

  ca::world_rules none;
  EXPECT_FALSE(rc::read_room_physics(app, "r", none.physics)) << "none written yet";
  EXPECT_FALSE(rc::read_avatar_policy(app, "r", none.policy));
  EXPECT_TRUE(rc::read_room_rules(app, "r") == ca::default_world_rules());

  rc::write_room_rules(app, "r", r, 120);
  EXPECT_TRUE(rc::read_room_rules(app, "r") == r);
  for (const ca::section_stamp &s :
       {rc::room_physics_stamp(app, "r"), rc::avatar_archetypes_stamp(app, "r"),
        rc::avatar_policy_stamp(app, "r"), rc::room_controllers_stamp(app, "r")}) {
    EXPECT_EQ(s.rev, 1u);
    EXPECT_EQ(s.at, 120u);
    EXPECT_EQ(s.epoch, ca::section_epoch(app));
  }
  // Each section has its own revision.
  r.policy.max_speed = 3.0;
  EXPECT_EQ(rc::write_avatar_policy(app, "r", r.policy, 300).rev, 2u);
  EXPECT_EQ(rc::room_physics_stamp(app, "r").rev, 1u);
  ca::avatar_policy p;
  ASSERT_TRUE(rc::read_avatar_policy(app, "r", p));
  EXPECT_EQ(p.max_speed, 3.0);
  EXPECT_EQ(rc::avatar_policy_stamp(app, "r").at, 300u);
}

// A list is read by its count, so a shorter version leaves nothing of the longer one behind.
TEST(RoomRulesTest, AShorterListLeavesNothingBehind) {
  cvc::app app;
  ca::world_rules r = ca::default_world_rules();
  ca::avatar_archetype extra;
  extra.id = "extra";
  r.archetypes.push_back(extra);
  r.controllers[0].params = {{"walk_speed", 2.0}, {"turn", 1.0}};
  rc::write_room_rules(app, "r", r);
  r.archetypes.pop_back();
  r.controllers[0].params = {{"walk_speed", 2.5}};
  rc::write_room_rules(app, "r", r);
  const ca::world_rules back = rc::read_room_rules(app, "r");
  EXPECT_EQ(back.archetypes.size(), 1u);
  ASSERT_EQ(back.controllers.size(), 1u);
  EXPECT_EQ(back.controllers[0].params.size(), 1u);
  EXPECT_EQ(back.controller().param("walk_speed", 0.0), 2.5);
  EXPECT_TRUE(back == r);
}
