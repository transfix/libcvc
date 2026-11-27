/*
  Copyright 2026 CyberPC Angel, LLC

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

// A world vehicle's parted look (vehicle_view.h; docs/ARIADNE.md, "A model in parts"): a chassis
// mesh, a wheel mesh and a rig, each wheel a node of its own that rolls at the vehicle's speed and
// turns with its steer. Checked:
//   1. headless: a `pose` entity its keys drive at a speed and a steer is drawn as its chassis and
//      four wheels, fitted to its vehicle by the rig (per axis: wheelbase, track, wheel radius);
//      each wheel's node is at its hub, its front ones turned by the pose's steer and every one
//      rolled by its spin, which is the speed times the time over the radius; a right wheel is the
//      mesh turned half a turn inside its node, or with mirror: reflect the mesh mirrored;
//   2. parted_wheel_poses alone: a plant's own per-wheel steer (an inner wheel turned more than an
//      outer one), its suspension's heights, and a rig that steers a wheel the plant does not;
//   3. the parts from a data: package (a local directory, as a cvcpkg:// package unpacks): pending
//      while it loads, then ready;
//   4. the pixels, offscreen: from the side, a front wheel (a paddle, so its spin shows) stands
//      upright after a quarter turn; from above, it turns with the steer while the rear one keeps
//      still;
//   5. the default Humvee: a `vehicle: humvee` archetype with no look: is drawn as its chassis and
//      four wheels where vehicle-humvee's humvee/ parts are installed (a prefix of the fixture's
//      meshes as .glb files), the vehicle_preview too; as the one-piece Humvee.glb where only that
//      is; as its box where neither is.
// The dynamics need Jolt and YAML (5: and a .glb writer and reader); the pixels skip (rc 0) where
// nothing rasterises, unless CVC_REQUIRE_RENDER=1. Synthetic meshes only.
#include "streaming_test_util.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/model/model_file_io.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <system_error>
#include <vector>
#include <vtkMatrix4x4.h>

using cvcgl_test::check;
namespace ari = cvc::gl::ariadne;
namespace fs = std::filesystem;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// An axis-aligned box [lo, hi] as OBJ text, outward-wound, offset by `at`.
std::string obj_box(std::array<double, 3> lo, std::array<double, 3> hi,
                    std::array<double, 3> at = {0.0, 0.0, 0.0}) {
  std::string out;
  for (int k = 0; k < 8; ++k)
    out += "v " + std::to_string(at[0] + (k & 1 ? hi[0] : lo[0])) + " " +
           std::to_string(at[1] + (k & 2 ? hi[1] : lo[1])) + " " +
           std::to_string(at[2] + (k & 4 ? hi[2] : lo[2])) + "\n";
  const int f[12][3] = {{1, 3, 4}, {1, 4, 2}, {5, 6, 8}, {5, 8, 7}, {1, 2, 6}, {1, 6, 5},
                        {3, 7, 8}, {3, 8, 4}, {1, 5, 7}, {1, 7, 3}, {2, 4, 8}, {2, 8, 6}};
  for (const auto &t : f)
    out += "f " + std::to_string(t[0]) + " " + std::to_string(t[1]) + " " + std::to_string(t[2]) +
           "\n";
  return out;
}

// The fixture, in the rig's frame (x forward, y left, z up, the ground at z 0): a chassis box
// between the wheels and a wheel that is a paddle 0.9 m across its axle and 0.24 m thick (so its
// spin shows), filed away from the origin (the look takes it about its middle); and a rig file in
// the shape a splitter writes: provenance, the radius under wheel:, the hubs under hubs: xyz:.
struct fixture {
  fs::path dir;
  std::string chassis, wheel, rig;
};

fixture write_fixture(const fs::path &dir) {
  std::error_code ec;
  fs::create_directories(dir, ec);
  fixture f;
  f.dir = dir;
  f.chassis = (dir / "chassis.obj").generic_string();
  f.wheel = (dir / "wheel.obj").generic_string();
  f.rig = (dir / "rig.json").generic_string();
  std::ofstream(f.chassis) << obj_box({-2.0, -0.6, 0.7}, {2.0, 0.6, 1.7});
  std::ofstream(f.wheel) << obj_box({-0.45, -0.15, -0.12}, {0.45, 0.15, 0.12}, {3.0, 3.0, 3.0});
  std::ofstream(f.rig) << R"({
  "source": { "title": "a test rover", "license": "CC0-1.0" },
  "frame": "metres; x forward, y left, z up; the ground under the tyres at z 0",
  "wheel": { "radius": 0.45, "width": 0.3 },
  "hubs": {
    "order": ["front_left", "front_right", "rear_left", "rear_right"],
    "xyz": [[1.6, 0.95, 0.45], [1.6, -0.95, 0.45], [-1.6, 0.95, 0.45], [-1.6, -0.95, 0.45]]
  }
}
)";
  return f;
}

cvc::ariadne::vehicle_look parted_look(const fixture &f) {
  cvc::ariadne::vehicle_look look;
  look.kind = "file";
  look.fit = true;
  look.chassis = f.chassis;
  look.wheel = f.wheel;
  look.rig_file = f.rig;
  return look;
}

// A node's local matrix, row-major.
std::array<double, 16> matrix_of(const cvc::gl::GraphicsNode &n) {
  std::array<double, 16> m{};
  const vtkMatrix4x4 *t = n.getTransform();
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      m[4 * r + c] = t->GetElement(r, c);
  return m;
}

bool same(const std::array<double, 16> &a, const std::array<double, 16> &b, double tol) {
  for (int i = 0; i < 16; ++i)
    if (!near(a[i], b[i], tol))
      return false;
  return true;
}

// 1. A `pose` entity drawn as a parted look, driven by its keys.
void test_pose_entity_wheels(const fixture &f) {
  std::printf("test_pose_entity_wheels\n");
  if (!cvc::ariadne::have_yaml() || !cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no YAML or no Jolt in this build\n");
    return;
  }
  // The parts by path (the files' own), and a two-wheeled rig given inline that reflects.
  const std::string parts = "chassis: \"" + f.chassis + "\", wheel: \"" + f.wheel + "\"";
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      rover: { vehicle: humvee, controllers: [pose], look: { )" +
                          parts + ", rig: \"" + f.rig + R"(" } }
      mirrored:
        vehicle: humvee
        controllers: [pose]
        look:
          { )" + parts + R"(,
            rig: { radius: 0.45, hubs: [[1.6, 0.95, 0.45], [1.6, -0.95, 0.45]], mirror: reflect } }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: shown
      type: vehicle_preview
      spin: 0
      transform: { position: [0, 40, 0] }
)";
  cvc::app app;
  cvc::ariadne::register_world_block(app);
  ari::register_vehicle_preview_node(app);
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
  check(r.ok, "the document loads", r.error);
  if (!r.ok)
    return;
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  const std::string e = cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules);
  check(e.empty(), "its world: block parses", e);
  r.scene.vehicles = rules.vehicles;
  if (rules.vehicles.size() != 2)
    return;
  cvc::gl::SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  check(out.world && out.world->vehicles(), "the scene has a world with vehicles");
  if (!out.world || !out.world->vehicles())
    return;
  cvc::ariadne::WorldVehicles &v = *out.world->vehicles();

  // Its keys drive it at 6 m/s along +x, steered 0.3 rad left.
  const double speed = 6.0, steer = 0.3;
  cvc::state &st = cvc::state::instance(app);
  st("vehicles.spawn.controller").value("pose");
  st("vehicles.spawn.request").value("1");
  for (int i = 0; i < 90; ++i) {
    if (i >= 1) {
      st("entities.1.pose.x").value(std::to_string(-20.0 + speed * i / 60.0));
      st("entities.1.pose.y").value("0");
      st("entities.1.pose.heading").value("0");
      st("entities.1.pose.speed").value(std::to_string(speed));
      st("entities.1.pose.steer").value(std::to_string(steer));
    }
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }
  check(v.size() == 1, "one entity");
  const std::shared_ptr<cvc::gl::GraphicsNode> node = sg.getGraphics("vehicle_1");
  check(node != nullptr, "drawn as vehicle_1");
  if (!node)
    return;
  const auto chassis =
      std::dynamic_pointer_cast<cvc::gl::GeometryNode>(node->findChildByName("vehicle_1_chassis"));
  check(chassis != nullptr, "its chassis is a node of its own");
  check(node->findChildByName("vehicle_1_mesh") == nullptr, "not one merged model");
  const auto drawn = out.vehicle_wheels.find("vehicle_1");
  check(drawn != out.vehicle_wheels.end() && drawn->second.size() == 4, "four wheels drawn");
  if (!chassis || drawn == out.vehicle_wheels.end() || drawn->second.size() != 4)
    return;

  // Fitted by the rig, each axis its own scale; the rig's ground on the ground under the chassis.
  const cvc::physics::vehicle_desc desc = cvc::ariadne::vehicle_desc_of(rules.vehicles[0]);
  const double r0 = desc.axles.front().wheel_radius;
  const double sx = desc.wheelbase() / 3.2, sy = desc.track() / 1.9, sz = r0 / 0.45;
  const double base = -cvc::ariadne::stand_height(desc);
  const std::array<double, 16> cm = matrix_of(*chassis);
  check(near(cm[0], sx, 1e-9) && near(cm[5], sy, 1e-9) && near(cm[10], sz, 1e-9),
        "the chassis scaled to the wheelbase, the track and the wheel radius");
  check(near(cm[11], base, 1e-9), "the chassis's ground on the vehicle's",
        std::to_string(cm[11]) + " against " + std::to_string(base));
  const auto chassis_bounds = chassis->getGeometry()->const_points();
  check(!chassis_bounds.empty(), "the chassis's mesh is the file's");

  const std::vector<cvc::ariadne::entity_wheel> plant = v.wheels(1);
  check(plant.size() == 4, "the pose entity has four wheels");
  if (plant.size() != 4)
    return;
  double front = desc.axles.front().x, back = front; // the vehicle's axles' middle
  for (const cvc::physics::axle_desc &a : desc.axles) {
    front = std::max(front, a.x);
    back = std::min(back, a.x);
  }
  const double axle_x = 0.5 * (front + back); // the rig's is 0
  check(near(cm[3], axle_x, 1e-9), "the rig's axles' middle on the vehicle's");
  for (std::size_t k = 0; k < 4; ++k) {
    const ari::drawn_wheel &d = drawn->second[k];
    const double hx = k < 2 ? 1.6 : -1.6, hy = k % 2 == 0 ? 0.95 : -0.95;
    check(near(d.x, sx * hx + axle_x, 1e-9) && near(d.y, sy * hy, 1e-9),
          "wheel " + std::to_string(k) + " at its fitted hub");
    check(near(d.rest_z, base + r0, 1e-9), "wheel " + std::to_string(k) + " a radius up at rest");
    check(d.steers == (k < 2), "wheel " + std::to_string(k) + (k < 2 ? " steers" : " does not"));
    const std::shared_ptr<cvc::gl::GraphicsNode> w = d.node.lock();
    check(w != nullptr && w->getName() == "vehicle_1_wheel" + std::to_string(k),
          "wheel " + std::to_string(k) + "'s node");
    if (!w)
      continue;
    // Its pose: Rz(steer) Ry(spin) at the hub, on the ground the pose entity's wheel sits on.
    const double turn = k < 2 ? steer : 0.0, spin = plant[k].spin;
    const double c = std::cos(turn), s = std::sin(turn), cs = std::cos(spin), ss = std::sin(spin);
    const std::array<double, 16> want = {c * cs, -s,  c * ss, d.x,        s * cs, c,   s * ss, d.y,
                                         -ss,    0.0, cs,     plant[k].z, 0.0,    0.0, 0.0,    1.0};
    check(same(matrix_of(*w), want, 1e-9), "wheel " + std::to_string(k) + " steered and rolled");
    check(same(matrix_of(*w), ari::parted_wheel_poses(drawn->second, plant)[k], 0.0),
          "wheel " + std::to_string(k) + " as parted_wheel_poses has it");
    const auto mesh = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(
        w->findChildByName(w->getName() + "_mesh"));
    check(mesh != nullptr, "wheel " + std::to_string(k) + "'s mesh under its node");
    if (!mesh)
      continue;
    const std::array<double, 16> mm = matrix_of(*mesh);
    const double sign = k % 2 == 0 ? 1.0 : -1.0; // a right one turned half a turn about z
    check(near(mm[0], sign * sz, 1e-9) && near(mm[5], sign * sz, 1e-9) && near(mm[10], sz, 1e-9),
          "wheel " + std::to_string(k) + " scaled to the vehicle's radius" +
              (k % 2 ? ", turned about its upright" : ""));
    // About its middle, though its file has it elsewhere.
    double lo = 1e300, hi = -1e300;
    for (const auto &p : mesh->getGeometry()->const_points()) {
      lo = std::min(lo, static_cast<double>(p[0]));
      hi = std::max(hi, static_cast<double>(p[0]));
    }
    check(near(lo, -0.45, 1e-6) && near(hi, 0.45, 1e-6),
          "wheel " + std::to_string(k) + " about its middle");
  }
  // Rolled by the speed over the time it has driven, over its radius.
  const double rolled = std::remainder(plant[0].spin - plant[2].spin, 2.0 * kPi);
  check(plant[0].spin != 0.0 && near(rolled, 0.0, 1e-12), "the wheels roll, together");
  const double before = plant[0].spin;
  const std::uint64_t t0 = out.physics->tick();
  for (int i = 90; i < 96; ++i) {
    st("entities.1.pose.x").value(std::to_string(-20.0 + speed * i / 60.0));
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }
  const double steps = static_cast<double>(out.physics->tick() - t0);
  const double turned = std::remainder(v.wheels(1)[0].spin - before, 2.0 * kPi);
  check(steps > 0.0 && near(turned, speed * steps * out.physics->config().fixed_dt / r0, 1e-9),
        "each step turns a wheel v dt / r", std::to_string(turned));
  const std::array<double, 16> after = matrix_of(*drawn->second[0].node.lock());
  check(near(after[8], -std::sin(v.wheels(1)[0].spin), 1e-9), "and its node with it");

  // The vehicle_preview draws the selected archetype from the catalog's keys: its parts, each
  // wheel at its fitted hub, at rest.
  const std::shared_ptr<cvc::gl::GraphicsNode> shown = sg.getGraphics("shown__vehicle");
  check(shown && shown->findChildByName("shown__vehicle_chassis"),
        "the preview draws the parted look's chassis");
  for (std::size_t k = 0; shown && k < 4; ++k) {
    const std::shared_ptr<cvc::gl::GraphicsNode> w =
        shown->findChildByName("shown__vehicle_wheel" + std::to_string(k));
    check(w != nullptr, "the preview's wheel " + std::to_string(k));
    if (!w)
      continue;
    const std::array<double, 16> m = matrix_of(*w);
    check(near(m[3], drawn->second[k].x - axle_x, 1e-6) && near(m[7], drawn->second[k].y, 1e-6) &&
              near(m[11], r0, 1e-6) && near(m[0], 1.0, 1e-12),
          "the preview's wheel " + std::to_string(k) + " at its hub, a radius up, at rest");
  }

  // mirror: reflect: the right wheel is the mesh mirrored, not turned.
  st("vehicles.selected").value("mirrored");
  st("vehicles.spawn.request").value("2");
  for (int i = 0; i < 4; ++i) {
    st("entities.2.pose.x").value("20");
    st("entities.2.pose.y").value("0");
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }
  const auto two = out.vehicle_wheels.find("vehicle_2");
  check(two != out.vehicle_wheels.end() && two->second.size() == 2, "a two-wheeled rig");
  if (two != out.vehicle_wheels.end() && two->second.size() == 2) {
    const auto left = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(
        two->second[0].node.lock()->findChildByName("vehicle_2_wheel0_mesh"));
    const auto right = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(
        two->second[1].node.lock()->findChildByName("vehicle_2_wheel1_mesh"));
    check(left && right, "both wheels' meshes");
    if (left && right) {
      check(near(matrix_of(*right)[0], matrix_of(*left)[0], 1e-12), "the right one not turned");
      const auto a = left->getGeometry()->const_points(), b = right->getGeometry()->const_points();
      const auto ta = left->getGeometry()->const_tris(), tb = right->getGeometry()->const_tris();
      check(!a.empty() && a.size() == b.size() && near(b[0][1], -a[0][1], 1e-12) &&
                near(b[0][0], a[0][0], 1e-12),
            "the right one mirrored across its xz plane");
      check(!ta.empty() && tb[0][1] == ta[0][2] && tb[0][2] == ta[0][1],
            "its triangles wound again");
    }
  }
}

// 2. parted_wheel_poses alone, against a plant's own wheels.
void test_poses_from_a_plant() {
  std::printf("test_poses_from_a_plant\n");
  std::vector<ari::drawn_wheel> hubs(4);
  for (std::size_t k = 0; k < 4; ++k) {
    hubs[k].x = k < 2 ? 1.6 : -1.6;
    hubs[k].y = k % 2 == 0 ? 0.9 : -0.9;
    hubs[k].rest_z = -0.5;
    hubs[k].steers = k < 2;
  }
  // A plant turning left: its inner (left) front wheel turned more than its outer one, the left
  // side's suspension compressed, the rear wheels rolled further than the front ones (slip).
  std::vector<cvc::ariadne::entity_wheel> plant(4);
  for (std::size_t k = 0; k < 4; ++k) {
    plant[k].x = k < 2 ? 1.65 : -1.65;
    plant[k].y = k % 2 == 0 ? 0.91 : -0.91;
    plant[k].z = k % 2 == 0 ? -0.42 : -0.55;
    plant[k].steered = k < 2;
    plant[k].steer = k == 0 ? 0.35 : (k == 1 ? 0.28 : 0.0);
    plant[k].spin = k < 2 ? 1.0 : 1.2;
  }
  const std::vector<std::array<double, 16>> m = ari::parted_wheel_poses(hubs, plant);
  check(m.size() == 4, "a pose per wheel");
  if (m.size() != 4)
    return;
  for (std::size_t k = 0; k < 4; ++k) {
    check(near(m[k][3], hubs[k].x, 0.0) && near(m[k][7], hubs[k].y, 0.0),
          "wheel " + std::to_string(k) + " at its hub");
    check(near(m[k][11], plant[k].z, 0.0), "wheel " + std::to_string(k) + " on its suspension");
    check(near(m[k][8], -std::sin(plant[k].spin), 1e-12),
          "wheel " + std::to_string(k) + " rolled by its own spin");
    check(near(m[k][4], std::sin(plant[k].steer) * std::cos(plant[k].spin), 1e-12),
          "wheel " + std::to_string(k) + " turned by its own steer");
  }
  // A rig that steers its rear wheels too, on a plant that steers only its front ones: each rear
  // wheel takes the steer of the steered wheel on its side; a plant that steers none, nothing.
  hubs[2].steers = hubs[3].steers = true;
  const std::vector<std::array<double, 16>> four = ari::parted_wheel_poses(hubs, plant);
  check(near(four[2][4], std::sin(0.35) * std::cos(1.2), 1e-12) &&
            near(four[3][4], std::sin(0.28) * std::cos(1.2), 1e-12),
        "a rear wheel the rig steers takes its side's steered wheel's steer");
  for (cvc::ariadne::entity_wheel &w : plant)
    w.steered = false;
  check(near(ari::parted_wheel_poses(hubs, plant)[0][4], 0.0, 0.0), "no steered wheel: no steer");
  // No plant wheels: at rest.
  const std::vector<std::array<double, 16>> rest = ari::parted_wheel_poses(hubs, {});
  check(rest.size() == 4 && near(rest[0][11], -0.5, 0.0) && near(rest[0][0], 1.0, 0.0),
        "with no plant wheels, each at rest");
}

// 3. The parts from a data: package.
void test_parts_from_a_package(const fixture &f) {
  std::printf("test_parts_from_a_package\n");
  if (!cvc::can_read_model_from_memory(".obj")) {
    std::printf("  skipped: no model reader for .obj from memory in this build\n");
    return;
  }
  cvc::app app;
  std::error_code ec;
  const fs::path pkg = f.dir / "pkg";
  const fs::path share = pkg / "share" / "cvc-scenes" / "rover";
  fs::create_directories(share, ec);
  for (const char *file : {"chassis.obj", "wheel.obj", "rig.json"})
    fs::copy_file(f.dir / file, share / file, fs::copy_options::overwrite_existing, ec);
  cvc::ariadne::vehicle_look look;
  look.kind = "data";
  look.fit = true;
  look.uri = pkg.generic_string();
  look.subpath = "share/cvc-scenes/rover";
  look.chassis = "chassis.obj";
  look.wheel = "wheel.obj";
  look.rig_file = "rig.json";
  const std::shared_ptr<ari::vehicle_model_cache> cache = ari::make_vehicle_model_cache();
  const ari::vehicle_model_status first = ari::parted_model_state(app, *cache, look);
  check(first != ari::vehicle_model_status::failed, "loading, or loaded");
  cvc::ariadne::DataSpec spec;
  spec.uri = look.uri;
  spec.path = look.subpath;
  const std::shared_ptr<cvc::ariadne::DataStore> store = cvc::ariadne::DataStore::of(app);
  check(store->wait(store->load(spec), 30.0), "the package loads");
  std::vector<std::string> why;
  check(ari::parted_model_state(app, *cache, look, &why) == ari::vehicle_model_status::ready,
        "then the parts are ready", why.empty() ? "" : why.front());
  check(ari::vehicle_model_state(app, *cache, look, 4.0) == ari::vehicle_model_status::ready,
        "vehicle_model_state says so too");
  cvc::gl::SceneGraph sg(app);
  std::shared_ptr<cvc::gl::GraphicsNode> group = sg.addGraphics("rover");
  std::vector<ari::drawn_wheel> wheels;
  const float yellow[3] = {0.9f, 0.75f, 0.2f};
  check(ari::attach_parted_model(app, *cache, *group, "rover", look, ari::vehicle_shape{}, -0.9,
                                 yellow, &wheels),
        "drawn from the package");
  check(wheels.size() == 4, "with its four wheels");
  // A wheel that cannot be read (a file look): failed, and the warning names it.
  cvc::ariadne::vehicle_look unread = parted_look(f);
  unread.wheel = (f.dir / "no_such_wheel.obj").generic_string();
  why.clear();
  check(ari::parted_model_state(app, *cache, unread, &why) == ari::vehicle_model_status::failed,
        "an unreadable wheel: failed");
  check(!why.empty() && why.front().find("no_such_wheel.obj") != std::string::npos,
        "the warning names the wheel", why.empty() ? "" : why.front());
  std::shared_ptr<cvc::gl::GraphicsNode> none = sg.addGraphics("none");
  check(!ari::attach_vehicle_model(app, *cache, *none, "none_mesh", unread, ari::vehicle_shape{},
                                   0.0, yellow),
        "and it is not drawn (the box look then)");
  // A package that is not there: failed, and why.
  cvc::ariadne::vehicle_look lost = look;
  lost.uri = (f.dir / "no_such_pkg").generic_string();
  cvc::ariadne::DataSpec gone = spec;
  gone.uri = lost.uri;
  store->wait(store->load(gone), 30.0);
  why.clear();
  check(ari::parted_model_state(app, *ari::make_vehicle_model_cache(), lost, &why) ==
            ari::vehicle_model_status::failed,
        "a package that cannot load: failed");
  check(!why.empty(), "with a warning", why.empty() ? "" : why.front());
}

// 4. The pixels: a front wheel's spin and steer, drawn offscreen.
struct box2 {
  int x0 = 1 << 30, y0 = 1 << 30, x1 = -1, y1 = -1;
  int w() const { return x1 < x0 ? 0 : x1 - x0 + 1; }
  int h() const { return y1 < y0 ? 0 : y1 - y0 + 1; }
};

// The dark (tyre) pixels' bounds in the window around a world point (half its size in pixels).
box2 dark_around(const cvcgl_test::Frame &f, std::array<double, 2> c, int half) {
  box2 b;
  for (int y = int(c[1]) - half; y <= int(c[1]) + half; ++y)
    for (int x = int(c[0]) - half; x <= int(c[0]) + half; ++x) {
      if (x < 0 || y < 0 || x >= f.w || y >= f.h)
        continue;
      const unsigned char *p = f.at(x, y);
      if (p[0] < 45 && p[1] < 45 && p[2] < 45) { // a tyre's, lit or not; the chassis is pale
        b.x0 = std::min(b.x0, x);
        b.y0 = std::min(b.y0, y);
        b.x1 = std::max(b.x1, x);
        b.y1 = std::max(b.y1, y);
      }
    }
  return b;
}

void test_pixels(const fixture &f) {
  std::printf("test_pixels\n");
  cvc::app app;
  if (!cvcgl_test::renderAvailable(app))
    return;
  cvc::gl::SceneGraph sg(app, "wheels");
  sg.setDiagnosticChromeVisible(false);
  std::shared_ptr<cvc::gl::GraphicsNode> group = sg.addGraphics("rover");
  // A vehicle the rig fits at scale 1: its wheelbase, track and wheels the rig's.
  ari::vehicle_shape shape;
  shape.wheelbase = 3.2;
  shape.track = 1.9;
  shape.wheel_radius = 0.45;
  std::vector<ari::drawn_wheel> wheels;
  const float pale[3] = {0.95f, 0.95f, 0.9f};
  const std::shared_ptr<ari::vehicle_model_cache> cache = ari::make_vehicle_model_cache();
  std::vector<std::string> warnings;
  check(ari::attach_parted_model(app, *cache, *group, "rover", parted_look(f), shape, 0.0, pale,
                                 &wheels, &warnings),
        "the parts are drawn", warnings.empty() ? "" : warnings.front());
  if (wheels.size() != 4)
    return;
  // The plant's wheels, a radius over the ground, spun and steered as asked.
  const auto pose = [&](double spin, double steer) {
    std::vector<cvc::ariadne::entity_wheel> plant(4);
    for (std::size_t k = 0; k < 4; ++k) {
      plant[k].x = k < 2 ? 1.6 : -1.6;
      plant[k].y = k % 2 == 0 ? 0.95 : -0.95;
      plant[k].z = 0.45;
      plant[k].steered = k < 2;
      plant[k].steer = k < 2 ? steer : 0.0;
      plant[k].spin = spin;
    }
    const std::vector<std::array<double, 16>> m = ari::parted_wheel_poses(wheels, plant);
    for (std::size_t k = 0; k < m.size(); ++k)
      wheels[k].node.lock()->setPoseMatrix(m[k].data());
  };
  cvc::gl::SceneRenderer sr(sg, 480, 360, /*offscreen=*/true);
  sr.setBackground(1.0, 1.0, 1.0);
  const auto shot = [&]() {
    for (int i = 0; i < 2; ++i) {
      sg.processEvents();
      sr.render();
    }
    return cvcgl_test::grab(sr);
  };
  const auto ortho = [&](double scale) {
    vtkCamera *cam = sr.renderer()->GetActiveCamera();
    cam->ParallelProjectionOn();
    cam->SetParallelScale(scale);
  };

  // From the left side: the front left wheel, a paddle lying along x, stands up after a quarter
  // turn of spin.
  sr.setCamera(0.0, 30.0, 0.6, 0.0, 0.0, 0.6, 0.0, 0.0, 1.0, 30.0, 1.0, 100.0);
  ortho(2.5);
  pose(0.0, 0.0);
  const cvcgl_test::Frame flat = shot(); // a frame first: the camera's projection is then set
  const std::array<double, 2> hub = cvcgl_test::toPx(sr, 1.6, 0.95, 0.45);
  const int half = int(std::fabs(cvcgl_test::toPx(sr, 1.6 + 0.7, 0.95, 0.45)[0] - hub[0]));
  const box2 a = dark_around(flat, hub, half);
  pose(0.5 * kPi, 0.0);
  const cvcgl_test::Frame up = shot();
  const box2 b = dark_around(up, hub, half);
  std::printf(
      "  the front left wheel from the side: %d x %d px lying, %d x %d px a quarter turn on\n",
      a.w(), a.h(), b.w(), b.h());
  check(a.w() > 0 && a.w() > 2 * a.h(), "unspun, the paddle lies along x");
  check(b.h() > 0 && b.h() > 2 * b.w(), "a quarter turn on, it stands up");
  check(cvcgl_test::diffPixels(flat, up) > 100, "the frames differ");

  // From above: the front left wheel turns with the steer; the rear left one keeps still.
  sr.setCamera(0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 30.0, 1.0, 100.0);
  ortho(3.0);
  pose(0.0, 0.0);
  const cvcgl_test::Frame straight = shot();
  const std::array<double, 2> front = cvcgl_test::toPx(sr, 1.6, 0.95, 0.9);
  const std::array<double, 2> rear = cvcgl_test::toPx(sr, -1.6, 0.95, 0.9);
  const int top_half = int(std::fabs(cvcgl_test::toPx(sr, 1.6, 0.95 + 0.6, 0.9)[0] - front[0]));
  const box2 s0 = dark_around(straight, front, top_half),
             r0 = dark_around(straight, rear, top_half);
  pose(0.0, 0.8);
  const cvcgl_test::Frame turned = shot();
  const box2 s1 = dark_around(turned, front, top_half), r1 = dark_around(turned, rear, top_half);
  std::printf(
      "  from above: the front left wheel %d x %d px straight, %d x %d px steered; the rear "
      "%d x %d px, then %d x %d px\n",
      s0.w(), s0.h(), s1.w(), s1.h(), r0.w(), r0.h(), r1.w(), r1.h());
  // Screen up is +x: a straight wheel is tall and narrow; steered 0.8 rad, nearly square.
  check(s0.h() > 0 && s0.h() > 2 * s0.w(), "straight, the front wheel lies along x");
  check(s1.w() > 0 && s1.h() < 1.5 * s1.w(), "steered, it turns");
  check(r0.w() == r1.w() && r0.h() == r1.h() && r0.w() > 0, "the rear one does not steer");
}

// 5. The default Humvee, in parts where they are installed.

// An environment variable set for a scope, and put back after (unset when it was).
struct env_for {
  std::string name, old;
  bool had = false;
  env_for(const char *n, const std::string &value) : name(n) {
    if (const char *v = std::getenv(n)) {
      had = true;
      old = v;
    }
    set(value);
  }
  ~env_for() { set(had ? old : std::string()); }
  env_for(const env_for &) = delete;
  env_for &operator=(const env_for &) = delete;
  void set(const std::string &v) {
#ifdef _WIN32
    _putenv_s(name.c_str(), v.c_str());
#else
    if (v.empty())
      unsetenv(name.c_str());
    else
      setenv(name.c_str(), v.c_str(), 1);
#endif
  }
};

// A model file (the fixture's OBJ) written again as a .glb at `to`; false, and why, when it will
// not.
bool write_glb(cvc::app &app, const std::string &from, const fs::path &to) {
  try {
    const cvc::model m = cvc::read_model(app, from);
    std::ofstream(to, std::ios::binary) << cvc::write_model_to_memory(app, m, ".glb");
    return fs::file_size(to) > 0;
  } catch (const std::exception &e) {
    std::printf("  %s as %s: %s\n", from.c_str(), to.string().c_str(), e.what());
    return false;
  }
}

// What a `vehicle: humvee` archetype with no look: is drawn as, spawned once as a `pose` entity.
struct drawn_humvee {
  cvc::ariadne::vehicle_look look;                  // as WorldVehicles resolved it
  bool chassis = false, mesh = false, body = false; // vehicle_1_chassis, _mesh (one piece), _body
  std::size_t wheels = 0;                           // a parted look's drawn wheels
  bool preview_chassis = false;                     // the vehicle_preview's parted chassis
};

drawn_humvee draw_default_humvee() {
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [pose] }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [120, 120, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
    - node: shown
      type: vehicle_preview
      spin: 0
      transform: { position: [0, 40, 0] }
)";
  drawn_humvee d;
  cvc::app app;
  cvc::ariadne::register_world_block(app);
  ari::register_vehicle_preview_node(app);
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, doc);
  check(r.ok, "the document loads", r.error);
  if (!r.ok)
    return d;
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  const std::string e = cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules);
  check(e.empty(), "its world: block parses", e);
  check(rules.vehicles.size() == 1 && rules.vehicles[0].look.kind.empty(), "a humvee, no look:");
  r.scene.vehicles = rules.vehicles; // as the loader leaves it: WorldVehicles resolves the look
  cvc::gl::SceneGraph sg(app);
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  check(out.world && out.world->vehicles(), "the scene has a world with vehicles");
  if (!out.world || !out.world->vehicles())
    return d;
  cvc::ariadne::WorldVehicles &v = *out.world->vehicles();
  if (!v.archetypes().empty())
    d.look = v.archetypes().front().look;
  cvc::state &st = cvc::state::instance(app);
  st("vehicles.spawn.controller").value("pose");
  st("vehicles.spawn.request").value("1");
  for (int i = 0; i < 6; ++i) {
    if (i >= 1) {
      st("entities.1.pose.x").value(std::to_string(-20.0 + i / 10.0));
      st("entities.1.pose.y").value("0");
    }
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  }
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  check(v.size() == 1, "one entity");
  if (const std::shared_ptr<cvc::gl::GraphicsNode> node = sg.getGraphics("vehicle_1")) {
    d.chassis = node->findChildByName("vehicle_1_chassis") != nullptr;
    d.mesh = node->findChildByName("vehicle_1_mesh") != nullptr;
    d.body = node->findChildByName("vehicle_1_body") != nullptr;
  }
  if (const auto w = out.vehicle_wheels.find("vehicle_1"); w != out.vehicle_wheels.end())
    d.wheels = w->second.size();
  if (const std::shared_ptr<cvc::gl::GraphicsNode> shown = sg.getGraphics("shown__vehicle"))
    d.preview_chassis = shown->findChildByName("shown__vehicle_chassis") != nullptr;
  return d;
}

void test_default_humvee(const fixture &f) {
  std::printf("test_default_humvee\n");
  if (!cvc::ariadne::have_yaml() || !cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no YAML or no Jolt in this build\n");
    return;
  }
  if (!cvc::can_write_model_to_memory(".glb") || !cvc::can_read_model_from_memory(".glb")) {
    std::printf("  skipped: no .glb writer or reader in this build\n");
    return;
  }
  // A prefix laid out as vehicle-humvee 1.0.0+cvc.5 installs it: Humvee.glb, and beside it
  // humvee/chassis.glb, humvee/wheel.glb and humvee/rig.json (the fixture's meshes and rig).
  std::error_code ec;
  const fs::path prefix = f.dir / "humvee_prefix";
  const fs::path shared = prefix / "share" / "cvc-scenes" / "shared";
  fs::create_directories(shared / "humvee", ec);
  cvc::app app;
  const bool glb = write_glb(app, f.chassis, shared / "Humvee.glb") &&
                   write_glb(app, f.chassis, shared / "humvee" / "chassis.glb") &&
                   write_glb(app, f.wheel, shared / "humvee" / "wheel.glb");
  check(glb, "the fixture's meshes written as .glb files");
  fs::copy_file(f.rig, shared / "humvee" / "rig.json", fs::copy_options::overwrite_existing, ec);
  if (!glb)
    return;
  const env_for bundle("CVC_NAV_BUNDLE", ""), at("CVC_PREFIX", prefix.string()),
      cvcpkg("CVCPKG_PREFIX", ""), conda("CONDA_PREFIX", "");

  // The parts installed: its chassis and four wheels, which turn; the one piece kept beside.
  const drawn_humvee parts = draw_default_humvee();
  check(parts.look.parted() && parts.look.kind == "file" &&
            fs::path(parts.look.chassis) == shared / "humvee" / "chassis.glb" &&
            fs::path(parts.look.wheel) == shared / "humvee" / "wheel.glb" &&
            fs::path(parts.look.rig_file) == shared / "humvee" / "rig.json",
        "with the parts installed, the look is the Humvee in parts", parts.look.chassis);
  check(fs::path(parts.look.path) == shared / "Humvee.glb", "the one piece kept in its path",
        parts.look.path);
  check(parts.chassis, "drawn as vehicle_1_chassis");
  check(parts.wheels == 4, "and four wheels", std::to_string(parts.wheels));
  check(!parts.mesh && !parts.body, "not the one piece, not the box");
  check(parts.preview_chassis, "the vehicle_preview draws the parts too");

  // Only the one piece (an older package, no humvee/): Humvee.glb, its wheels in it.
  fs::remove_all(shared / "humvee", ec);
  if (!cvc::ariadne::find_vehicle_model("humvee/chassis.glb").empty()) {
    std::printf("  one piece: skipped (this program's own prefix has the parts)\n");
    return;
  }
  const drawn_humvee one = draw_default_humvee();
  check(!one.look.parted() && one.look.kind == "file" &&
            fs::path(one.look.path) == shared / "Humvee.glb",
        "without the parts, the look is the one-piece Humvee.glb", one.look.path);
  check(one.mesh, "drawn as vehicle_1_mesh");
  check(!one.chassis && one.wheels == 0 && !one.body, "no parts, no box");

  // Neither: the box.
  fs::remove(shared / "Humvee.glb", ec);
  if (!cvc::ariadne::find_vehicle_model("humvee").empty()) {
    std::printf("  box: skipped (this program's own prefix has Humvee.glb)\n");
    return;
  }
  const drawn_humvee box = draw_default_humvee();
  check(box.look.kind == "box", "with neither, the box look", box.look.kind);
  check(box.body && !box.mesh && !box.chassis, "drawn as its box");
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  std::error_code ec;
  const fixture f = write_fixture(fs::temp_directory_path(ec) / "cvcgl_ariadne_wheeled_look");
  test_pose_entity_wheels(f);
  test_poses_from_a_plant();
  test_parts_from_a_package(f);
  test_pixels(f);
  test_default_humvee(f);
  fs::remove_all(f.dir, ec);
  return cvcgl_test::finish("cvcgl_ariadne_wheeled_look");
}
