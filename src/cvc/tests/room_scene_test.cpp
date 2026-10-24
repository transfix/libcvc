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

// H1 slice 5: the shared room scene (conf.rooms.<room>.scene.*). The server-authoritative scene
// round-trips through scalar leaves back into the .ari Scene types, removal is soft, seeding is
// idempotent, and the seeded scene replicates to a client over the existing "conf" shard.

#include <cstdint>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace rs = cvc::ariadne::conference;
using cvc::ariadne::SceneLight;
using cvc::ariadne::SceneNode;

namespace {
void expect_v3_eq(const float a[3], const float b[3]) {
  EXPECT_FLOAT_EQ(a[0], b[0]);
  EXPECT_FLOAT_EQ(a[1], b[1]);
  EXPECT_FLOAT_EQ(a[2], b[2]);
}
const SceneNode *find_node(const std::vector<SceneNode> &v, const std::string &id) {
  for (const SceneNode &n : v)
    if (n.id == id)
      return &n;
  return nullptr;
}
} // namespace

// ── one app: the schema round-trips ─────────────────────────────────────────

TEST(RoomSceneTest, NothingWrittenReadsEmpty) {
  cvc::app app;
  rs::room_scene_env env;
  EXPECT_FALSE(rs::read_scene_env(app, "r", env));
  EXPECT_TRUE(rs::read_scene_objects(app, "r").empty());
  EXPECT_EQ(rs::scene_revision(app, "r"), 0u);
  EXPECT_FALSE(rs::read_room_scene(app, "r").any());
}

TEST(RoomSceneTest, EnvRoundTrips) {
  cvc::app app;
  rs::room_scene_env env;
  env.background_gradient = true;
  env.background_top[0] = 0.25f, env.background_top[1] = 0.5f, env.background_top[2] = 0.75f;
  env.chrome = false;
  SceneLight sun;
  sun.id = "sun";
  sun.azimuth = -35.0f;
  sun.elevation = 55.0f;
  sun.intensity = 1.2f;
  sun.color[2] = 0.9f;
  SceneLight spot;
  spot.id = "spot";
  spot.kind = "spot";
  spot.pos[0] = 1.0f, spot.pos[1] = 2.0f, spot.pos[2] = 3.0f;
  spot.cone = 30.0f;
  env.lights = {sun, spot};
  rs::write_scene_env(app, "r", env);

  rs::room_scene_env out;
  ASSERT_TRUE(rs::read_scene_env(app, "r", out));
  EXPECT_TRUE(out.background_gradient);
  expect_v3_eq(out.background_top, env.background_top);
  expect_v3_eq(out.background_bottom, env.background_bottom);
  EXPECT_FALSE(out.chrome);
  ASSERT_EQ(out.lights.size(), 2u);
  const SceneLight &s = out.lights[1].id == "sun" ? out.lights[1] : out.lights[0];
  const SceneLight &p = out.lights[1].id == "sun" ? out.lights[0] : out.lights[1];
  EXPECT_EQ(s.id, "sun");
  EXPECT_EQ(s.kind, "directional");
  EXPECT_FLOAT_EQ(s.azimuth, -35.0f);
  EXPECT_FLOAT_EQ(s.elevation, 55.0f);
  EXPECT_FLOAT_EQ(s.intensity, 1.2f);
  expect_v3_eq(s.color, sun.color);
  EXPECT_EQ(p.id, "spot");
  EXPECT_EQ(p.kind, "spot");
  expect_v3_eq(p.pos, spot.pos);
  EXPECT_FLOAT_EQ(p.cone, 30.0f);
}

TEST(RoomSceneTest, LightDroppedFromEnvIsSoftRemoved) {
  cvc::app app;
  rs::room_scene_env env = rs::default_scene_env();
  SceneLight fill;
  fill.id = "fill";
  fill.kind = "fill";
  env.lights.push_back(fill);
  rs::write_scene_env(app, "r", env);
  env.lights.pop_back(); // rewrite without "fill"
  rs::write_scene_env(app, "r", env);

  rs::room_scene_env out;
  ASSERT_TRUE(rs::read_scene_env(app, "r", out));
  ASSERT_EQ(out.lights.size(), 1u);
  EXPECT_EQ(out.lights[0].id, "sun");
}

TEST(RoomSceneTest, ObjectRoundTrips) {
  cvc::app app;
  SceneNode box;
  box.id = "crate";
  box.source_primitive = "box";
  box.box_size[0] = 2.0f, box.box_size[1] = 0.5f, box.box_size[2] = 1.25f;
  box.position[0] = 3.0f, box.position[1] = -4.0f, box.position[2] = 0.625f;
  box.rotation[2] = 45.0f;
  box.scale[0] = 1.5f;
  box.color[0] = 0.1f, box.color[1] = 0.2f, box.color[2] = 0.3f;
  box.ambient = 0.4f;
  box.diffuse = 0.6f;
  box.visible_default = false;

  SceneNode mesh;
  mesh.id = "bunny";
  mesh.source_file = "stanford.bunny";
  mesh.has_specular = true;
  mesh.specular = 0.5f;
  mesh.specular_power = 12.0f;

  rs::write_scene_object(app, "r", box);
  rs::write_scene_object(app, "r", mesh);
  const std::vector<SceneNode> out = rs::read_scene_objects(app, "r");
  ASSERT_EQ(out.size(), 2u);

  const SceneNode *b = find_node(out, "crate");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->type, "geometry");
  EXPECT_EQ(b->source_primitive, "box");
  EXPECT_TRUE(b->source_file.empty());
  expect_v3_eq(b->box_size, box.box_size);
  EXPECT_TRUE(b->has_transform);
  expect_v3_eq(b->position, box.position);
  expect_v3_eq(b->rotation, box.rotation);
  expect_v3_eq(b->scale, box.scale);
  EXPECT_TRUE(b->has_material);
  expect_v3_eq(b->color, box.color);
  EXPECT_FLOAT_EQ(b->ambient, 0.4f);
  EXPECT_FLOAT_EQ(b->diffuse, 0.6f);
  EXPECT_FALSE(b->has_specular); // unset stays unset (GeometryNode keeps its defaults)
  EXPECT_FALSE(b->visible_default);

  const SceneNode *m = find_node(out, "bunny");
  ASSERT_NE(m, nullptr);
  EXPECT_EQ(m->source_file, "stanford.bunny");
  EXPECT_TRUE(m->source_primitive.empty());
  EXPECT_TRUE(m->has_specular);
  EXPECT_FLOAT_EQ(m->specular, 0.5f);
  EXPECT_FLOAT_EQ(m->specular_power, 12.0f);
  EXPECT_TRUE(m->visible_default);
}

TEST(RoomSceneTest, RemoveIsSoftAndBumpsRevision) {
  cvc::app app;
  for (const SceneNode &n : rs::default_scene_objects())
    rs::write_scene_object(app, "r", n);
  const std::uint64_t before = rs::scene_revision(app, "r");
  EXPECT_EQ(before, 2u);

  rs::remove_scene_object(app, "r", "ref");
  EXPECT_EQ(rs::scene_revision(app, "r"), before + 1);
  const std::vector<SceneNode> out = rs::read_scene_objects(app, "r");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].id, "ground");

  rs::remove_scene_object(app, "r", "never-written"); // a no-op, no revision bump
  EXPECT_EQ(rs::scene_revision(app, "r"), before + 1);

  // Re-writing a soft-removed object brings it back.
  rs::write_scene_object(app, "r", rs::default_scene_objects()[1]);
  EXPECT_EQ(rs::read_scene_objects(app, "r").size(), 2u);
}

TEST(RoomSceneTest, RejectsIdsContainingTheSeparator) {
  cvc::app app;
  SceneNode bad;
  bad.id = "a.b";
  EXPECT_THROW(rs::write_scene_object(app, "r", bad), std::invalid_argument);
  bad.id = "";
  EXPECT_THROW(rs::write_scene_object(app, "r", bad), std::invalid_argument);
  EXPECT_THROW(rs::remove_scene_object(app, "r", "a.b"), std::invalid_argument);
  rs::room_scene_env env;
  SceneLight l;
  l.id = "x.y";
  env.lights.push_back(l);
  EXPECT_THROW(rs::write_scene_env(app, "r", env), std::invalid_argument);
}

TEST(RoomSceneTest, SeedIsIdempotentAndNeverClobbers) {
  cvc::app app;
  ASSERT_TRUE(rs::seed_default_scene(app, "r"));
  const std::uint64_t seeded = rs::scene_revision(app, "r");
  EXPECT_EQ(seeded, 3u); // env + ground + ref
  EXPECT_EQ(rs::read_scene_objects(app, "r").size(), 2u);

  // The scene is edited after seeding; a relay restart's re-seed must not undo the edit.
  rs::remove_scene_object(app, "r", "ref");
  EXPECT_FALSE(rs::seed_default_scene(app, "r"));
  EXPECT_EQ(rs::scene_revision(app, "r"), seeded + 1);
  EXPECT_EQ(rs::read_scene_objects(app, "r").size(), 1u);

  // Rooms are independent.
  EXPECT_TRUE(rs::seed_default_scene(app, "other"));
}

TEST(RoomSceneTest, ReadRoomSceneIsARealizableAriadneScene) {
  cvc::app app;
  ASSERT_TRUE(rs::seed_default_scene(app, "r"));
  const cvc::ariadne::Scene s = rs::read_room_scene(app, "r");
  EXPECT_TRUE(s.any());
  ASSERT_EQ(s.nodes.size(), 2u);
  const SceneNode *ground = find_node(s.nodes, "ground");
  ASSERT_NE(ground, nullptr);
  EXPECT_EQ(ground->source_primitive, "plane");
  EXPECT_FLOAT_EQ(ground->plane_size, 20.0f);
  const SceneNode *ref = find_node(s.nodes, "ref");
  ASSERT_NE(ref, nullptr);
  EXPECT_EQ(ref->source_primitive, "box");
  EXPECT_FLOAT_EQ(ref->position[2], 0.5f); // stands on the ground
  ASSERT_EQ(s.lights.size(), 1u);
  EXPECT_EQ(s.lights[0].id, "sun");
  EXPECT_TRUE(s.has_chrome);
  EXPECT_TRUE(s.chrome_visible);
  EXPECT_TRUE(s.has_background);
  EXPECT_TRUE(s.background_gradient);
}

// ── end-to-end: the server's scene replicates to a client ───────────────────

TEST(RoomSceneTest, SeededSceneReplicatesToAClientAndTracksEdits) {
  cvc::app server_app, client_app;
  cvc::state_transport_inproc tp;
  cvc::state_cluster_shard ss(server_app, "conf", "server");
  cvc::state_cluster_shard cs(client_app, "conf", "client");
  ss.attach();
  cs.attach();
  tp.register_shard(&ss);
  tp.register_shard(&cs);

  ASSERT_TRUE(rs::seed_default_scene(server_app, "r"));
  tp.pump_all();

  EXPECT_EQ(rs::scene_revision(client_app, "r"), rs::scene_revision(server_app, "r"));
  const cvc::ariadne::Scene got = rs::read_room_scene(client_app, "r");
  ASSERT_EQ(got.nodes.size(), 2u);
  ASSERT_NE(find_node(got.nodes, "ground"), nullptr);
  ASSERT_NE(find_node(got.nodes, "ref"), nullptr);
  ASSERT_EQ(got.lights.size(), 1u);

  // The server moves the reference object and removes the ground; the client follows.
  SceneNode moved = rs::default_scene_objects()[1];
  moved.position[0] = 4.0f;
  rs::write_scene_object(server_app, "r", moved);
  rs::remove_scene_object(server_app, "r", "ground");
  tp.pump_all();

  EXPECT_EQ(rs::scene_revision(client_app, "r"), rs::scene_revision(server_app, "r"));
  const std::vector<SceneNode> objs = rs::read_scene_objects(client_app, "r");
  ASSERT_EQ(objs.size(), 1u);
  EXPECT_EQ(objs[0].id, "ref");
  EXPECT_FLOAT_EQ(objs[0].position[0], 4.0f);
}
