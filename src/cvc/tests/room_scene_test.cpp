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
// idempotent, and the seeded scene replicates to a client over the existing "conf" shard. The
// large demo scene seeds mesh objects that physics never sees, and leaves the default one as it
// was.

#include <cmath>
#include <cstdint>
#include <cvc/ariadne/conference/room_scene.h>
#include <cvc/ariadne/holon_sync.h> // room_statics
#include <cvc/ariadne/scene.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_cluster_shard.h>
#include <cvc/state/state_transport_inproc.h>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
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
const SceneLight *find_light(const std::vector<SceneLight> &v, const std::string &id) {
  for (const SceneLight &l : v)
    if (l.id == id)
      return &l;
  return nullptr;
}
// The leaf paths under PATH, relative to it ("source.box.x", ...).
std::set<std::string> leaves_under(cvc::app &app, const std::string &path) {
  std::set<std::string> all, leaves;
  cvc::state *n = app.root().findDescendant(path);
  if (!n)
    return leaves;
  const std::string prefix = path + cvc::state::SEPARATOR;
  for (const std::string &full : n->children())
    if (full.rfind(prefix, 0) == 0)
      all.insert(full.substr(prefix.size()));
  for (const std::string &p : all) {
    bool inner = false;
    for (const std::string &q : all)
      inner = inner || q.rfind(p + cvc::state::SEPARATOR, 0) == 0;
    if (!inner)
      leaves.insert(p);
  }
  return leaves;
}
std::string object_path(const std::string &room, const std::string &id) {
  return rs::room_scene_path(room) + ".objects." + id;
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
  env.shadows = true;
  env.shadow_resolution = 4096;
  rs::write_scene_env(app, "r", env);

  rs::room_scene_env out;
  ASSERT_TRUE(rs::read_scene_env(app, "r", out));
  EXPECT_TRUE(out.background_gradient);
  expect_v3_eq(out.background_top, env.background_top);
  expect_v3_eq(out.background_bottom, env.background_bottom);
  EXPECT_FALSE(out.chrome);
  EXPECT_TRUE(out.shadows);
  EXPECT_EQ(out.shadow_resolution, 4096);
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
  SceneLight sun;
  sun.id = "sun";
  env.lights.push_back(sun);
  SceneLight fill;
  fill.id = "fill";
  fill.kind = "fill";
  env.lights.push_back(fill);
  rs::write_scene_env(app, "r", env);
  env.lights.pop_back(); // rewrite without "fill"
  rs::write_scene_env(app, "r", env);

  rs::room_scene_env out;
  ASSERT_TRUE(rs::read_scene_env(app, "r", out));
  ASSERT_EQ(out.lights.size(), 1u); // "fill" is gone
  EXPECT_TRUE(find_light(out.lights, "sun"));
  EXPECT_FALSE(find_light(out.lights, "fill"));
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
  EXPECT_TRUE(s.lights.empty()) << "the light rig is the client document's, not the room's";
  EXPECT_FALSE(s.has_shadows) << "a room that leaves shadows off says nothing about them";
  EXPECT_TRUE(s.has_chrome);
  EXPECT_FALSE(s.chrome_visible) << "the debug overlay is off in a conference room";
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
  EXPECT_TRUE(got.lights.empty());

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

// ── a mesh's fit: ───────────────────────────────────────────────────────────

TEST(RoomSceneTest, FitRoundTripsAndClears) {
  cvc::app app;
  SceneNode m;
  m.id = "m";
  m.source_file = "x.bunny";
  m.has_fit = true;
  m.fit_up_y = true;
  m.fit_height = 2.5f;
  rs::write_scene_object(app, "r", m);
  std::vector<SceneNode> out = rs::read_scene_objects(app, "r");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_TRUE(out[0].has_fit);
  EXPECT_TRUE(out[0].fit_up_y);
  EXPECT_FLOAT_EQ(out[0].fit_height, 2.5f);

  // Rewritten without a fit: cleared, not left behind.
  m.has_fit = false;
  rs::write_scene_object(app, "r", m);
  out = rs::read_scene_objects(app, "r");
  ASSERT_EQ(out.size(), 1u);
  EXPECT_FALSE(out[0].has_fit);

  // An object that never had a fit writes no fit leaves at all.
  SceneNode p;
  p.id = "p";
  p.source_primitive = "plane";
  rs::write_scene_object(app, "r", p);
  EXPECT_EQ(app.root().findDescendant(object_path("r", "p") + ".source.fit"), nullptr);
  out = rs::read_scene_objects(app, "r");
  const SceneNode *plane = find_node(out, "p");
  ASSERT_NE(plane, nullptr);
  EXPECT_FALSE(plane->has_fit);
}

// ── the large demo scene (--scene large) ─────────────────────────────────────

TEST(RoomSceneTest, LargeDemoSceneSeedsMeshes) {
  cvc::app app;
  ASSERT_TRUE(rs::seed_large_demo_scene(app, "r"));
  EXPECT_FALSE(rs::seed_large_demo_scene(app, "r")) << "idempotent";
  EXPECT_FALSE(rs::seed_default_scene(app, "r")) << "a seeded room is never re-seeded";

  // 100 m plane, the reference box, and a bunny in every cell of the 5 x 5 lattice but the
  // middle one, which is the box's.
  const std::vector<SceneNode> objs = rs::read_scene_objects(app, "r");
  ASSERT_EQ(objs.size(), 26u);
  EXPECT_EQ(rs::scene_revision(app, "r"), 27u); // env + 26 objects
  const SceneNode *ground = find_node(objs, "ground");
  ASSERT_NE(ground, nullptr);
  EXPECT_EQ(ground->source_primitive, "plane");
  EXPECT_FLOAT_EQ(ground->plane_size, 100.0f);
  EXPECT_FALSE(ground->has_fit);
  const SceneNode *ref = find_node(objs, "ref");
  ASSERT_NE(ref, nullptr);
  EXPECT_EQ(ref->source_primitive, "box");
  EXPECT_FLOAT_EQ(ref->position[2], 0.5f);
  EXPECT_FALSE(ref->has_fit);

  std::set<std::pair<long, long>> cells;
  for (const SceneNode &n : objs) {
    if (n.id == "ground" || n.id == "ref")
      continue;
    EXPECT_EQ(n.id.rfind("bunny_", 0), 0u) << n.id;
    EXPECT_EQ(n.source_file, "x.bunny") << n.id;
    EXPECT_TRUE(n.source_primitive.empty()) << n.id;
    // The size comes from the fit (baked into the mesh), not a transform scale, so the mesh's
    // own units are metres.
    EXPECT_TRUE(n.has_fit && n.fit_up_y) << n.id;
    EXPECT_FLOAT_EQ(n.fit_height, 1.7f) << n.id;
    expect_v3_eq(n.scale, SceneNode().scale);
    EXPECT_FLOAT_EQ(n.position[2], 0.0f) << n.id << ": the fit stands it on the ground";
    // Cell centres 20 m apart, inside the plane, never the origin.
    const double gx = (n.position[0] + 40.0) / 20.0, gy = (n.position[1] + 40.0) / 20.0;
    EXPECT_NEAR(gx, std::round(gx), 1e-4) << n.id;
    EXPECT_NEAR(gy, std::round(gy), 1e-4) << n.id;
    EXPECT_TRUE(gx > -0.5 && gx < 4.5 && gy > -0.5 && gy < 4.5) << n.id;
    EXPECT_FALSE(std::lround(gx) == 2 && std::lround(gy) == 2) << n.id;
    EXPECT_TRUE(n.visible_default);
    cells.insert({std::lround(gx), std::lround(gy)});
  }
  EXPECT_EQ(cells.size(), 24u) << "one bunny per cell";

  // A realizable Scene, in the default room's environment.
  const cvc::ariadne::Scene s = rs::read_room_scene(app, "r");
  EXPECT_EQ(s.nodes.size(), 26u);
  EXPECT_TRUE(s.lights.empty());
  EXPECT_FALSE(s.has_shadows);
  EXPECT_TRUE(s.has_chrome && !s.chrome_visible);
  EXPECT_TRUE(s.background_gradient);
}

TEST(RoomSceneTest, LargeDemoSceneLattice) {
  // An even grid has no middle cell: every cell gets a bunny.
  const std::vector<SceneNode> even = rs::large_demo_scene_objects(60.0, 2);
  ASSERT_EQ(even.size(), 2u + 4u);
  EXPECT_FLOAT_EQ(even[0].plane_size, 60.0f);
  for (std::size_t k = 2; k < even.size(); ++k) {
    EXPECT_FLOAT_EQ(std::fabs(even[k].position[0]), 15.0f);
    EXPECT_FLOAT_EQ(std::fabs(even[k].position[1]), 15.0f);
  }
  EXPECT_NE(find_node(even, "bunny_1_0"), nullptr);
  // Odd: the middle is the box's. One cell: no bunny at all.
  EXPECT_EQ(rs::large_demo_scene_objects(60.0, 3).size(), 2u + 8u);
  EXPECT_EQ(find_node(rs::large_demo_scene_objects(60.0, 3), "bunny_1_1"), nullptr);
  EXPECT_EQ(rs::large_demo_scene_objects(10.0, 1).size(), 2u);

  EXPECT_THROW(rs::large_demo_scene_objects(0.0, 5), std::invalid_argument);
  EXPECT_THROW(rs::large_demo_scene_objects(std::numeric_limits<double>::quiet_NaN(), 5),
               std::invalid_argument);
  EXPECT_THROW(rs::large_demo_scene_objects(100.0, 0), std::invalid_argument);
  cvc::app app;
  EXPECT_THROW(rs::seed_large_demo_scene(app, "r", -1.0), std::invalid_argument);
  EXPECT_EQ(rs::scene_revision(app, "r"), 0u) << "a refused seed writes nothing";
}

// Physics statics are the primitives only: the bunnies are render meshes (and LOD proxies on the
// clients), so the server's and every client's prediction world stay the plane and the box.
TEST(RoomSceneTest, RoomStaticsIgnoreMeshes) {
  cvc::app app;
  ASSERT_TRUE(rs::seed_large_demo_scene(app, "r"));
  const std::vector<cvc::ariadne::avatar_static> st = cvc::ariadne::room_statics(app, "r");
  ASSERT_EQ(st.size(), 2u) << "the plane and the box; the 24 bunnies have no physical shape";
  const cvc::ariadne::avatar_static &ground = st[0].half_extents.x > 5.0 ? st[0] : st[1];
  const cvc::ariadne::avatar_static &cube = st[0].half_extents.x > 5.0 ? st[1] : st[0];
  EXPECT_EQ(ground.type, cvc::ariadne::avatar_static::kind::box);
  EXPECT_DOUBLE_EQ(ground.half_extents.x, 50.0);
  EXPECT_DOUBLE_EQ(ground.half_extents.y, 50.0);
  EXPECT_NEAR(ground.center.z + ground.half_extents.z, 0.0, 1e-9) << "the slab's top is the floor";
  EXPECT_EQ(cube.type, cvc::ariadne::avatar_static::kind::box);
  EXPECT_DOUBLE_EQ(cube.half_extents.x, 0.5);
  EXPECT_DOUBLE_EQ(cube.center.z, 0.5);
}

// The default seed is what it always was: the same two objects, the same leaves (no fit), the
// same revision -- the live 20 m room does not change with the large scene's arrival.
TEST(RoomSceneTest, DefaultSceneUnchanged) {
  cvc::app app;
  ASSERT_TRUE(rs::seed_default_scene(app, "r"));
  EXPECT_EQ(rs::scene_revision(app, "r"), 3u);
  const std::vector<SceneNode> objs = rs::read_scene_objects(app, "r");
  ASSERT_EQ(objs.size(), 2u);
  const SceneNode *ground = find_node(objs, "ground");
  const SceneNode *ref = find_node(objs, "ref");
  ASSERT_NE(ground, nullptr);
  ASSERT_NE(ref, nullptr);
  EXPECT_EQ(ground->source_primitive, "plane");
  EXPECT_FLOAT_EQ(ground->plane_size, 20.0f);
  EXPECT_EQ(ref->source_primitive, "box");
  expect_v3_eq(ref->box_size, rs::default_scene_objects()[1].box_size);
  EXPECT_FLOAT_EQ(ref->position[2], 0.5f);
  EXPECT_FALSE(ground->has_fit || ref->has_fit);

  const std::set<std::string> expected = {"material.ambient",
                                          "material.color.b",
                                          "material.color.g",
                                          "material.color.r",
                                          "material.diffuse",
                                          "material.specular",
                                          "material.specular_power",
                                          "present",
                                          "source.box.x",
                                          "source.box.y",
                                          "source.box.z",
                                          "source.plane_size",
                                          "source.primitive",
                                          "source.radius",
                                          "source.uri",
                                          "transform.pos.x",
                                          "transform.pos.y",
                                          "transform.pos.z",
                                          "transform.rot.x",
                                          "transform.rot.y",
                                          "transform.rot.z",
                                          "transform.scale.x",
                                          "transform.scale.y",
                                          "transform.scale.z",
                                          "visible"};
  EXPECT_EQ(leaves_under(app, object_path("r", "ground")), expected);
  EXPECT_EQ(leaves_under(app, object_path("r", "ref")), expected);
  // The large scene's ground and box are the default's, bar the plane's size.
  const std::vector<SceneNode> large = rs::large_demo_scene_objects();
  ASSERT_GE(large.size(), 2u);
  EXPECT_EQ(large[0].id, "ground");
  EXPECT_EQ(large[1].id, "ref");
  expect_v3_eq(large[1].position, ref->position);
  expect_v3_eq(large[1].color, ref->color);
}

// A room that turns shadows on asserts them (re-baked every frame, for moving avatars).
TEST(RoomSceneTest, RoomShadowsAreAssertedOnlyWhenOn) {
  cvc::app app;
  rs::room_scene_env env = rs::default_scene_env();
  env.shadows = true;
  env.shadow_resolution = 4096;
  rs::write_scene_env(app, "r", env);
  const cvc::ariadne::Scene s = rs::read_room_scene(app, "r");
  EXPECT_TRUE(s.has_shadows && s.shadows_enabled);
  EXPECT_TRUE(s.has_shadow_resolution && s.shadow_resolution == 4096);
  EXPECT_TRUE(s.has_shadow_interval && s.shadow_interval == 1);
}
