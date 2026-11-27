/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ariadne_entities -- the entity view (entity_view.h, FLEETS_AND_NIGHT_ROADMAP.md FL2): a
// world's vehicles drawn at scale, in both holons.
//
//   implicit   40 kinematic vehicles and no entity_layer: one mesh for the archetype, the nearest
//              8 to the camera drawn as their own nodes (vehicle_view), the rest as copies in the
//              mesh, each where its chassis is;
//   follow     a camera's track.target names a far one: it gets its node;
//   keep       a camera that moves a little keeps who is promoted;
//   threshold  24 vehicles: each its own node, no mesh, as before the entity view;
//   of         an entity_layer of one fleet: that fleet at scale, the other fleet's vehicles nodes;
//   worker     the world stepped by a local holon on a thread of its own (world_worker): the same
//              copies, where the frame's own stepping puts them;
//   room       a relay's entities on a client (room_view, its entity_playout): 40 at scale, the
//              one the client drives promoted with the nearest;
//   parted     a parted look (a chassis, a wheel and a rig): its mesh the chassis and four wheels
//              at rest where its nodes draw them, the wheels dark; 40 of them at scale.
//   boats      a boat's box look's mesh is its hull, each point where add_hull_node draws it;
//              40 launches on a lake at scale, the nearest 8 their hulls' nodes, the water kept
//              out of the nearest hulls; 40 tactical boats with a one-piece model, its copies
//              fitted to their length overall, their keels on their hulls' bottoms, as a
//              promoted one's node draws it.
//
// Skips (rc 0) without Jolt, or where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/conference/room_entities.h>
#include <cvc/ariadne/holon.h>
#include <cvc/ariadne/holon_sync.h> // entity_playout
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h> // the world: block
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/ariadne/world_worker.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/entity_view.h>
#include <cvc/gl/ariadne/room_view.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/vehicle_view.h>
#include <cvc/gl/ariadne/world_nodes.h> // register_water_node_types
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/dyn_boat_plant.h> // a boat preset's hull
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkDataArray.h>
#include <vtkMapper.h>
#include <vtkMatrix4x4.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>

using namespace cvcgl_test;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::gl::ariadne;
namespace ca = cvc::ariadne;

namespace {

constexpr int W = 320, H = 240;

// A world of `fleets` (each "name:count") of kinematic Humvees on a 240 m square, and `extra`
// scene nodes; the Humvee in `look` when it names one (a look: map), else the box look.
std::string world_doc(const std::vector<std::pair<std::string, int>> &fleets,
                      const std::string &extra = "", const std::string &look = "") {
  std::string start;
  int seed = 3;
  for (const auto &[name, count] : fleets)
    start += "      - { fleet: " + name +
             ", archetype: humvee, plant: kinematic, count: " + std::to_string(count) +
             ", spawn: opposed, seed: " + std::to_string(seed++) +
             ", rally: { count: 4 }, policy: builtin, keys: summary }\n";
  return R"(meta: { min_libcvc: "0.0.0" }
requires: [scene.physics]
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee, controllers: [nav@1, none])" +
         (look.empty() ? std::string() : ", look: " + look) + R"( }
    start:
)" + start +
         R"(scene:
  clock: {}
  physics: {}
  world: { source: square, cell: 4 }
  nodes:
    - node: square
      world: square
      source: { box: { size: [240, 240, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)" + extra;
}

// The points a mesh node draws now (streamed ones too: updateVertices writes its polydata).
std::vector<double> drawn(const std::shared_ptr<GraphicsNode> &n) {
  std::vector<double> out;
  auto g = std::dynamic_pointer_cast<GeometryNode>(n);
  vtkActor *actor = g ? vtkActor::SafeDownCast(g->prop()) : nullptr;
  vtkPolyData *pd = actor && actor->GetMapper()
                        ? vtkPolyData::SafeDownCast(actor->GetMapper()->GetInputAsDataSet())
                        : nullptr;
  if (!pd || !pd->GetPoints())
    return out;
  for (vtkIdType i = 0; i < pd->GetNumberOfPoints(); ++i) {
    double p[3];
    pd->GetPoint(i, p);
    out.insert(out.end(), {p[0], p[1], p[2]});
  }
  return out;
}

// Each copy's centre in a mesh of copies of `per` points (the collapsed ones left out).
std::vector<std::array<double, 2>> copies(const std::vector<double> &pts, std::size_t per) {
  std::vector<std::array<double, 2>> out;
  if (per == 0)
    return out;
  for (std::size_t c = 0; (c + 1) * per * 3 <= pts.size(); ++c) {
    double x = 0.0, y = 0.0, lo = 1e30, hi = -1e30;
    for (std::size_t k = 0; k < per; ++k) {
      const double *p = &pts[(c * per + k) * 3];
      x += p[0];
      y += p[1];
      lo = std::min(lo, p[0]);
      hi = std::max(hi, p[0]);
    }
    if (hi - lo < 1e-9)
      continue; // collapsed: no entity in this slot
    out.push_back({x / static_cast<double>(per), y / static_cast<double>(per)});
  }
  return out;
}

// The number of the scene's vehicles drawn as their own nodes.
std::size_t own_nodes(const ari::RealizedScene &r) { return r.vehicle_markers.size(); }

struct world_run {
  cvc::app app;
  SceneGraph sg{app, "entities"};
  std::unique_ptr<SceneRenderer> sr;
  ari::RealizedScene rs;
  std::unique_ptr<ca::world_worker> ww;
  bool ok = false;

  world_run(const std::string &doc, bool threaded) {
    ari::register_entity_node_types(app);
    ari::register_water_node_types(app); // a lake's water node (the boats)
    ca::register_world_block(app);
    ca::LoadResult r = ca::load_string(app, doc);
    if (!check(r.ok, "the document loads", r.error))
      return;
    // The world: block's vehicles to the scene, as AppRuntime gives them.
    if (const ca::Value *world = ca::find_world_block(r)) {
      ca::world_rules rules = ca::default_world_rules();
      if (!check(ca::parse_world_block(*world, rules).empty(), "the world: block parses"))
        return;
      r.scene.vehicles = rules.vehicles;
      r.scene.spawns = rules.spawns;
      r.scene.vehicle_starts = rules.vehicle_starts;
    }
    std::vector<std::string> warnings;
    rs = ari::realize_scene(sg, r.scene, sg.getStatePrefix(), &warnings);
    if (!check(rs.world != nullptr && rs.physics_binding != nullptr, "the world is realized"))
      return;
    sr = std::make_unique<SceneRenderer>(sg, W, H, /*offscreen=*/true);
    vtkCamera *cam = sr->renderer()->GetActiveCamera();
    cam->SetPosition(-60.0, -150.0, 90.0);
    cam->SetFocalPoint(0.0, 0.0, 0.0);
    cam->SetViewUp(0.0, 0.0, 1.0);
    cam->SetViewAngle(110.0);                        // the whole square on screen
    ari::tick_scene(rs, sr->renderer(), 1.0 / 60.0); // anchors the world on the clock
    if (threaded) {
      ca::world_worker::options o;
      ari::RealizedScene *p = &rs;
      o.capture = [p] {
        if (p->physics_binding)
          p->physics_binding->tracker().capture();
      };
      try {
        ww = std::make_unique<ca::world_worker>(*rs.world, app.world_clock(), o);
        rs.world_worker = ww.get();
      } catch (const std::exception &e) {
        check(false, "worker: the world goes on a local holon", e.what());
        return;
      }
    }
    ok = true;
  }
  ~world_run() {
    rs.world_worker = nullptr;
    ww.reset();
  }

  void frame(double dt = 1.0 / 60.0) {
    if (!ww) {
      ari::tick_scene(rs, sr->renderer(), dt);
      return;
    }
    {
      const ca::world_worker::frame_lock held = ww->lock();
      ari::tick_scene(rs, sr->renderer(), dt);
    }
    ww->wait_caught_up(30.0);
  }

  // Frames until every vehicle is placed, then `more`.
  bool settle(std::size_t want, int more = 30) {
    for (int i = 0; i < 600; ++i) {
      std::size_t placed = 0;
      if (const ca::WorldVehicles *v = rs.world ? rs.world->vehicles() : nullptr)
        for (const ca::entity_info &e : v->entities())
          placed += e.placed ? 1 : 0;
      if (placed == want)
        break;
      frame();
    }
    for (int i = 0; i < more; ++i)
      frame();
    const ari::entity_view_stats s = ari::entity_view_stats_of(rs);
    if (s.members != want) {
      std::size_t placed = 0, all = 0;
      if (const ca::WorldVehicles *v = rs.world ? rs.world->vehicles() : nullptr)
        for (const ca::entity_info &e : v->entities()) {
          ++all;
          placed += e.placed ? 1 : 0;
        }
      std::printf(
          "  %zu entities, %zu placed; the view has %zu members, %zu layers, %zu rebuilds\n", all,
          placed, s.members, s.layers, s.rebuilds);
    }
    return s.members == want;
  }

  std::vector<ca::entity_info> entities() const { return rs.world->vehicles()->entities(); }
};

// Every copy in the mesh is within `tol` m of an entity that has no node of its own, and every
// such entity on screen is a copy. `per`: a copy's points (the box look: six boxes of 24).
bool copies_where_entities_are(world_run &w, const std::string &mesh, double tol,
                               std::size_t *count = nullptr, std::size_t per = 144) {
  std::shared_ptr<GraphicsNode> node = w.sg.getGraphics("entities");
  std::shared_ptr<GraphicsNode> m = node ? node->findChildByName(mesh) : nullptr;
  if (!m)
    return false;
  const std::vector<std::array<double, 2>> c = copies(drawn(m), per);
  if (count)
    *count = c.size();
  std::set<std::uint64_t> drawn_as_node;
  for (const auto &kv : w.rs.vehicle_markers)
    drawn_as_node.insert(std::stoull(kv.first.substr(8)));
  for (const auto &p : c) {
    bool near = false;
    for (const ca::entity_info &e : w.entities())
      near = near || (!drawn_as_node.count(e.id) && std::hypot(e.x - p[0], e.y - p[1]) < tol);
    if (!near)
      return false;
  }
  return !c.empty();
}

void test_implicit_follow_keep() {
  std::printf("implicit, follow, keep\n");
  world_run w(world_doc({{"city", 40}}), false);
  if (!w.ok || !check(w.settle(40), "implicit: the view takes all 40"))
    return;
  const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
  check(s.layers == 1 && s.meshes == 1, "implicit: one layer, one mesh for the box look");
  check(s.promoted == 8 && own_nodes(w.rs) == 8, "implicit: the nearest 8 have nodes of their own",
        std::to_string(s.promoted) + " promoted, " + std::to_string(own_nodes(w.rs)) + " nodes");
  check(s.instanced >= 24 && s.instanced <= 32, "implicit: the rest are copies in the mesh",
        std::to_string(s.instanced) + " of 32 on screen");
  // The 8 are the nearest to the camera.
  double eye[3];
  w.sr->renderer()->GetActiveCamera()->GetPosition(eye);
  std::vector<std::pair<double, std::uint64_t>> by;
  for (const ca::entity_info &e : w.entities())
    by.push_back({std::hypot(std::hypot(e.x - eye[0], e.y - eye[1]), e.z - eye[2]), e.id});
  std::sort(by.begin(), by.end());
  std::size_t nearest_have = 0;
  for (std::size_t k = 0; k < 8; ++k)
    nearest_have += ari::entity_drawn_as_node(w.rs, by[k].second) ? 1 : 0;
  check(nearest_have >= 7, "implicit: the promoted are the nearest (one may be on the edge)",
        std::to_string(nearest_have) + " of the 8 nearest");
  std::size_t n = 0;
  const bool where = copies_where_entities_are(w, "entities_humvee_lod0", 2.5, &n);
  check(where && n == s.instanced, "implicit: each copy where an unpromoted vehicle is",
        std::to_string(n) + " copies");
  // A camera that follows the farthest one gives it a node.
  const std::uint64_t far = by.back().second;
  cvc::state::instance(w.app)(w.sg.getStatePrefix() + ".viewers.main.camera.track.target")
      .value("vehicle_" + std::to_string(far));
  w.frame();
  w.frame();
  check(ari::entity_drawn_as_node(w.rs, far) &&
            w.sg.getGraphics("vehicle_" + std::to_string(far)) != nullptr,
        "follow: the followed one has its own node");
  check(ari::entity_view_stats_of(w.rs).promoted == 8, "follow: it counts among the 8");
  // A camera moving a metre keeps them.
  w.frame();
  vtkCamera *cam = w.sr->renderer()->GetActiveCamera();
  double p[3];
  cam->GetPosition(p);
  cam->SetPosition(p[0] + 1.0, p[1], p[2]);
  w.frame();
  check(!ari::entity_nodes_changed(w.rs), "keep: a camera a metre on keeps who is promoted");
}

void test_threshold() {
  std::printf("threshold\n");
  world_run w(world_doc({{"city", 24}}), false);
  if (!w.ok || !check(w.settle(24), "threshold: the view takes all 24"))
    return;
  const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
  check(s.promoted == 24 && own_nodes(w.rs) == 24 && s.meshes == 0,
        "threshold: 24 each have their own node, no mesh");
  check(w.sg.getGraphics("entities") == nullptr, "threshold: no layer node is made");
}

void test_of_fleet() {
  std::printf("of\n");
  world_run w(world_doc({{"city", 40}, {"escort", 10}}, R"(    - node: city_bodies
      type: entity_layer
      of: { fleet: city }
      promote: 4
)"),
              false);
  if (!w.ok || !check(w.settle(40), "of: the layer takes the city's 40"))
    return;
  const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
  check(s.layers == 1 && s.promoted == 4, "of: the layer promotes 4");
  check(own_nodes(w.rs) == 14, "of: the escort's 10 and the city's 4 are nodes",
        std::to_string(own_nodes(w.rs)));
  bool escort_nodes = true;
  for (const ca::entity_info &e : w.entities())
    if (e.fleet == "escort")
      escort_nodes = escort_nodes && ari::entity_drawn_as_node(w.rs, e.id);
  check(escort_nodes, "of: every escort vehicle has its own node");
  std::shared_ptr<GraphicsNode> layer = w.sg.getGraphics("city_bodies");
  check(layer && layer->findChildByName("city_bodies_humvee_lod0") != nullptr,
        "of: the mesh is under the layer's node");
}

void test_worker() {
  std::printf("worker\n");
  if (!ca::world_worker::available()) {
    std::printf("  skipped: no threads\n");
    return;
  }
  world_run w(world_doc({{"city", 40}}), true);
  if (!w.ok || !check(w.settle(40), "worker: the view takes all 40"))
    return;
  const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
  check(s.meshes == 1 && s.promoted == 8 && own_nodes(w.rs) == 8,
        "worker: the nearest 8 nodes, the rest in the mesh");
  w.frame(0.0); // what the last steps ran, drawn
  std::size_t n = 0;
  const bool where = copies_where_entities_are(w, "entities_humvee_lod0", 2.5, &n);
  check(where && n >= 24, "worker: each copy where the local holon's world has its vehicle",
        std::to_string(n) + " copies");
  check(w.ww && w.ww->error().empty() && w.ww->steps() > 0, "worker: the holon stepped it");
}

void test_room() {
  std::printf("room\n");
  using cvc::physics::body_state;
  using cvc::physics::vec3;
  namespace rs = cvc::ariadne::conference;
  cvc::app app;
  SceneGraph sg(app, "room_entities");
  SceneRenderer view(sg, W, H, /*offscreen=*/true, "main");
  vtkCamera *cam = view.renderer()->GetActiveCamera();
  cam->SetPosition(0.0, -120.0, 60.0);
  cam->SetFocalPoint(0.0, 0.0, 0.0);
  cam->SetViewUp(0.0, 0.0, 1.0);
  ca::holon h(ca::holon_role::client);
  body_state a0;
  h.add_participant("A", /*local=*/true, a0);
  ca::entity_playout playout;
  std::vector<rs::room_entity> ents;
  for (int i = 0; i < 40; ++i) {
    rs::room_entity e;
    e.id = static_cast<std::uint64_t>(100 + i);
    e.name = "car " + std::to_string(i);
    e.archetype = "rover";
    e.size = {4.0, 2.0, 1.2};
    e.wheels = {2.6, 1.7, 0.4};
    if (i == 39)
      e.driver = "A"; // the one this client drives, the farthest
    ents.push_back(e);
  }
  rs::write_room_entities(app, "ents", ents);
  const double dt = h.clock().fixed_dt();
  ari::room_view rv(view, app, "ents", "A", sg.getStatePrefix());
  for (int f = 0; f < 4; ++f) {
    for (int i = 0; i < 40; ++i) {
      body_state s;
      s.position = vec3{-80.0 + 4.0 * i, 2.0 * i, 0.6}; // farther as i grows
      playout.apply(rs::entity_pid(static_cast<std::uint64_t>(100 + i)), s, h.tick());
    }
    h.advance(dt);
    playout.pump(app, "ents", dt);
    rv.update(h, dt);
    rv.update_entities(playout);
    view.render();
  }
  const ari::entity_view_stats s = rv.entity_stats();
  check(s.members == 40 && s.meshes == 1, "room: the 40 at scale, one mesh",
        std::to_string(s.members) + " members, " + std::to_string(s.meshes) + " meshes");
  const std::vector<std::uint64_t> own = rv.entity_ids();
  check(own.size() == 8, "room: 8 have nodes of their own", std::to_string(own.size()));
  check(std::find(own.begin(), own.end(), 139u) != own.end(),
        "room: the one this client drives is among them");
  std::shared_ptr<GraphicsNode> layer = sg.getGraphics("room_entities");
  std::shared_ptr<GraphicsNode> mesh =
      layer ? layer->findChildByName("room_entities_rover_lod0") : nullptr;
  const std::vector<std::array<double, 2>> c = copies(drawn(mesh), 144);
  bool where = !c.empty();
  for (const auto &p : c) {
    bool near = false;
    for (int i = 0; i < 40; ++i)
      near = near || std::hypot(-80.0 + 4.0 * i - p[0], 2.0 * i - p[1]) < 2.5;
    where = where && near;
  }
  check(where && c.size() + own.size() >= 36, "room: each copy where the playout shows its entity",
        std::to_string(c.size()) + " copies");
}

// A box from lo to hi, offset by `at`, as an OBJ file (8 points, 12 triangles).
void box_obj(const std::filesystem::path &p, const std::array<double, 3> &lo,
             const std::array<double, 3> &hi, const std::array<double, 3> &at = {0.0, 0.0, 0.0}) {
  std::ofstream out(p);
  for (int k = 0; k < 8; ++k)
    out << "v " << at[0] + (k & 1 ? hi[0] : lo[0]) << " " << at[1] + (k & 2 ? hi[1] : lo[1]) << " "
        << at[2] + (k & 4 ? hi[2] : lo[2]) << "\n";
  const int f[12][3] = {{1, 3, 4}, {1, 4, 2}, {5, 6, 8}, {5, 8, 7}, {1, 2, 6}, {1, 6, 5},
                        {3, 7, 8}, {3, 8, 4}, {1, 5, 7}, {1, 7, 3}, {2, 4, 8}, {2, 8, 6}};
  for (const auto &t : f)
    out << "f " << t[0] << " " << t[1] << " " << t[2] << "\n";
}

// The colour of each point a mesh node draws now, 0 to 255.
std::vector<double> drawn_colors(const std::shared_ptr<GraphicsNode> &n) {
  std::vector<double> out;
  auto g = std::dynamic_pointer_cast<GeometryNode>(n);
  vtkActor *actor = g ? vtkActor::SafeDownCast(g->prop()) : nullptr;
  vtkPolyData *pd = actor && actor->GetMapper()
                        ? vtkPolyData::SafeDownCast(actor->GetMapper()->GetInputAsDataSet())
                        : nullptr;
  vtkDataArray *c = pd ? pd->GetPointData()->GetScalars() : nullptr;
  if (!c || c->GetNumberOfComponents() < 3)
    return out;
  const double scale = c->GetDataType() == VTK_UNSIGNED_CHAR ? 1.0 : 255.0;
  for (vtkIdType i = 0; i < c->GetNumberOfTuples(); ++i)
    for (int k = 0; k < 3; ++k)
      out.push_back(scale * c->GetComponent(i, k));
  return out;
}

// A node's local matrix applied to p.
std::array<double, 3> node_point(const GraphicsNode &n, const std::array<double, 3> &p) {
  const vtkMatrix4x4 *m = n.getTransform();
  std::array<double, 3> q{};
  for (int r = 0; r < 3; ++r)
    q[r] = m->GetElement(r, 0) * p[0] + m->GetElement(r, 1) * p[1] + m->GetElement(r, 2) * p[2] +
           m->GetElement(r, 3);
  return q;
}

// A parted look (look: { chassis:, wheel:, rig: }) at scale: its mesh is its chassis and its four
// wheels at rest, each point where attach_parted_model's nodes draw it (a right wheel turned, or
// reflected), the chassis the model's (its entity's colour, untextured) and the wheels a tyre's
// dark; a world of them is drawn as the model's four rungs, the wheels dark at each, and with
// lod: false each copy's chassis in its colour and its wheels dark.
void test_parted() {
  std::printf("parted\n");
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ariadne_entities_parted";
  fs::create_directories(dir, ec);
  // In the rig's frame (x forward, y left, z up, the ground at z 0); the wheel filed away from the
  // origin (the look takes it about its middle).
  box_obj(dir / "chassis.obj", {-2.0, -0.6, 0.7}, {2.0, 0.6, 1.7});
  box_obj(dir / "wheel.obj", {-0.45, -0.15, -0.12}, {0.45, 0.15, 0.12}, {3.0, 3.0, 3.0});
  const std::string chassis = (dir / "chassis.obj").generic_string(),
                    wheel = (dir / "wheel.obj").generic_string();
  std::size_t per = 0, body = 0;
  for (const std::string mirror : {"turn", "reflect"}) {
    cvc::app app;
    SceneGraph sg(app, "parted");
    ca::vehicle_look look;
    look.kind = "file";
    look.fit = true;
    look.chassis = chassis;
    look.wheel = wheel;
    look.rig.radius = 0.45;
    look.rig.hubs = {
        {1.6, 0.95, 0.45}, {1.6, -0.95, 0.45}, {-1.6, 0.95, 0.45}, {-1.6, -0.95, 0.45}};
    look.rig.mirror = mirror;
    ari::vehicle_shape s;
    s.wheelbase = 3.3;
    s.track = 1.82;
    s.wheel_radius = 0.47;
    s.axle_x = 0.1;
    const double base_z = -0.92;
    const std::shared_ptr<ari::vehicle_model_cache> cache = ari::make_vehicle_model_cache();
    std::vector<std::string> warnings;
    ari::vehicle_look_mesh mesh;
    const ari::vehicle_model_status st =
        ari::vehicle_look_mesh_of(app, *cache, look, s, base_z, mesh, &warnings);
    const std::string at = "parted (" + mirror + "): ";
    check(st == ari::vehicle_model_status::ready && mesh.model && !mesh.texture,
          at + "its mesh is ready, a model's, untextured");
    std::shared_ptr<GraphicsNode> group = sg.addGraphics("v");
    const float color[3] = {0.5f, 0.5f, 0.5f};
    if (!check(group && ari::attach_parted_model(app, *cache, *group, "v", look, s, base_z, color,
                                                 nullptr, &warnings),
               at + "attach_parted_model draws it as nodes"))
      return;
    // Each part's points where the group draws them: the chassis by its node, a wheel by its hub's
    // node and its mesh's under it.
    std::vector<std::array<double, 3>> want;
    if (auto c = std::dynamic_pointer_cast<GeometryNode>(group->findChildByName("v_chassis")))
      for (const auto &p : c->getGeometry()->const_points())
        want.push_back(node_point(*c, {p[0], p[1], p[2]}));
    body = want.size();
    for (int k = 0; k < 4; ++k) {
      const std::string w = "v_wheel" + std::to_string(k);
      std::shared_ptr<GraphicsNode> hub = group->findChildByName(w);
      auto m = hub ? std::dynamic_pointer_cast<GeometryNode>(hub->findChildByName(w + "_mesh"))
                   : nullptr;
      if (!check(m != nullptr, at + "node " + w + "_mesh"))
        return;
      for (const auto &p : m->getGeometry()->const_points())
        want.push_back(node_point(*hub, node_point(*m, {p[0], p[1], p[2]})));
    }
    per = want.size();
    double worst = body > 0 && mesh.verts.size() == 3 * per ? 0.0 : 1e30;
    for (std::size_t i = 0; worst < 1e30 && i < per; ++i)
      for (int c = 0; c < 3; ++c)
        worst = std::max(worst, std::fabs(mesh.verts[3 * i + c] - want[i][c]));
    check(worst < 1e-9, at + "the chassis and its four wheels where the nodes draw them",
          std::to_string(worst) + " m, " + std::to_string(mesh.verts.size() / 3) + " points, " +
              std::to_string(per) + " drawn by the nodes");
    bool roles = mesh.role.size() == per && mesh.own.size() == 3 * per;
    for (std::size_t i = 0; roles && i < per; ++i)
      roles = i < body ? mesh.role[i] == 3
                       : mesh.role[i] == 2 && mesh.own[3 * i] == 18 && mesh.own[3 * i + 2] == 20;
    check(roles, at + "the chassis the model's, the wheels a tyre's dark");
  }
  if (per == 0)
    return;
  const std::string look = "{ chassis: \"" + chassis + "\", wheel: \"" + wheel +
                           "\", rig: { radius: 0.45, hubs: [[1.6, 0.95, 0.45], [1.6, -0.95, 0.45], "
                           "[-1.6, 0.95, 0.45], [-1.6, -0.95, 0.45]] } }";
  const auto is_tyre = [](const double *rgb) {
    return std::fabs(rgb[0] - 18.0) < 1.0 && std::fabs(rgb[2] - 20.0) < 1.0;
  };
  {
    // The model's four rungs: at each, the copies' wheels dark and their chassis not (each colour
    // decimated on its own).
    world_run w(world_doc({{"city", 40}}, "", look), false);
    if (!w.ok || !check(w.settle(40), "parted: the view takes all 40"))
      return;
    const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
    check(s.meshes == 4 && s.promoted == 8 && own_nodes(w.rs) == 8,
          "parted: the nearest 8 nodes, the rest in the model's four rungs",
          std::to_string(s.meshes) + " meshes, " + std::to_string(own_nodes(w.rs)) + " nodes");
    std::shared_ptr<GraphicsNode> layer = w.sg.getGraphics("entities");
    std::size_t rungs = 0;
    bool both = true;
    std::string seen;
    for (int k = 0; k < 4; ++k) {
      std::shared_ptr<GraphicsNode> m =
          layer ? layer->findChildByName("entities_humvee_lod" + std::to_string(k)) : nullptr;
      const std::vector<double> pts = drawn(m), cols = drawn_colors(m);
      // The slots past the copies are collapsed onto the first copy's place (or the origin
      // before any): the tail of points all at the last one's.
      std::size_t end = pts.size();
      while (end >= 6 && pts[end - 6] == pts[pts.size() - 3] &&
             pts[end - 5] == pts[pts.size() - 2] && pts[end - 4] == pts[pts.size() - 1])
        end -= 3;
      if (end < pts.size())
        end -= 3; // the tail's first point too
      std::size_t dark = 0, light = 0;
      for (std::size_t i = 0; cols.size() == pts.size() && i + 2 < end; i += 3)
        (is_tyre(&cols[i]) ? dark : light) += 1;
      seen += " lod" + std::to_string(k) + ": " + std::to_string(dark) + " dark, " +
              std::to_string(light) + " not;";
      if (dark + light == 0)
        continue;
      ++rungs;
      both = both && dark > 0 && light > 0;
    }
    check(rungs > 0 && both, "parted: at every rung drawn, dark wheels on chassis in colour", seen);
  }
  // One rung (lod: false), the model's whole mesh: in each copy, its chassis's points in its
  // colour and its wheels' dark.
  world_run w(world_doc({{"city", 40}}, R"(    - node: bodies
      type: entity_layer
      lod: false
)",
                        look),
              false);
  if (!w.ok || !check(w.settle(40), "parted, one rung: the layer takes all 40"))
    return;
  std::shared_ptr<GraphicsNode> layer = w.sg.getGraphics("bodies");
  std::shared_ptr<GraphicsNode> lod0 =
      layer ? layer->findChildByName("bodies_humvee_lod0") : nullptr;
  const std::vector<double> pts = drawn(lod0), cols = drawn_colors(lod0);
  std::size_t placed = 0;
  bool dark = cols.size() == pts.size() && !pts.empty();
  for (std::size_t c = 0; dark && (c + 1) * per * 3 <= pts.size(); ++c) {
    double lo = 1e30, hi = -1e30;
    for (std::size_t k = 0; k < per; ++k) {
      lo = std::min(lo, pts[(c * per + k) * 3]);
      hi = std::max(hi, pts[(c * per + k) * 3]);
    }
    if (hi - lo < 1e-9)
      continue; // collapsed: no entity in this slot
    ++placed;
    for (std::size_t k = 0; dark && k < per; ++k)
      dark = k < body ? !is_tyre(&cols[(c * per + k) * 3]) : is_tyre(&cols[(c * per + k) * 3]);
  }
  check(dark && placed >= 24, "parted, one rung: each copy's chassis in its colour, wheels dark",
        std::to_string(placed) + " copies");
}

// A lake (the terrain's 6 m bowl, the water at -1 m over it, about 130 m across) and a fleet of
// `count` boats of `preset` on the dynamicboat plant, their look `look` when it names one (else
// the preset's: a launch's is its hull), and `extra` scene nodes.
std::string lake_doc(const std::string &preset, int count, const std::string &look = "",
                     const std::string &extra = "") {
  return R"(meta: { min_libcvc: "0.0.0" }
requires: [scene.physics]
world:
  vehicles:
    archetypes:
      boat: { plant: "dynamicboat:)" +
         preset + R"(", controllers: [nav@1, none])" +
         (look.empty() ? std::string() : ", look: " + look) + R"( }
    start:
      - { fleet: harbour, archetype: boat, count: )" +
         std::to_string(count) + R"(, spawn: scattered, seed: 7, policy: builtin, keys: summary }
scene:
  clock: {}
  physics: {}
  world: { source: lake, cell: 2 }
  nodes:
    - node: terrain
      world: lake
      source:
        heightfield:
          size: 300
          resolution: 151
          layers:
            - { kind: dome, amplitude: -6, radius: 50 }
      body: static
    - node: water
      type: water
      ground: terrain
      level: -1
      wind_speed: 0.5
      wave_amp: 0.1
      body: {}
)" + extra;
}

// A boat's box look at scale is its hull (vehicle_look_mesh_of with its hull): each point where
// add_hull_node draws it, its topsides in role 0 and its stripe in role 1 (where the node has the
// colours it was given), the rest (its wet bottom, a box hull's teak sole) in role 2, the node's
// colour; a box hull (the launch) open with its sole, a V hull (the tactical boat) without one.
// 40 launches on a lake: one mesh, the nearest 8 their hulls' nodes, each copy where a launch is,
// and the water kept out of the 4 nearest hulls as when each boat had its node. 40 tactical boats
// with a one-piece model: fitted as the boat node fits it (boat_fit_of: its length overall, its
// keel on its hull's bottom), the same in the promoted boats' nodes and in the mesh's copies.
void test_boats() {
  std::printf("boats\n");
  namespace ph = cvc::physics;
  std::size_t launch_points = 0;
  ari::boat_fit tactical;
  for (const std::string preset : {"launch", "tactical"}) {
    cvc::app app;
    SceneGraph sg(app, "boats");
    const ph::dyn_boat_desc desc = ph::dyn_boat_desc::preset(preset);
    const ari::boat_fit fit = ari::boat_fit_of(desc);
    double keel = 1e30;
    for (const ph::vec3 &v : desc.boat.hull.hull->vertices)
      keel = std::min(keel, v.z);
    const std::string at = "boats (" + preset + "): ";
    check(fit.ok && fit.keel == keel && keel < 0.0 &&
              (desc.length_overall <= 0.0 || fit.shape.length == desc.length_overall),
          at + "fitted to its length overall, its keel its hull's bottom",
          std::to_string(fit.shape.length) + " m, its keel at " + std::to_string(fit.keel));
    if (preset == "tactical")
      tactical = fit;
    const std::shared_ptr<ari::vehicle_model_cache> cache = ari::make_vehicle_model_cache();
    ca::vehicle_look box;
    box.kind = "box";
    ari::vehicle_look_mesh mesh;
    const ari::vehicle_model_status st =
        ari::vehicle_look_mesh_of(app, *cache, box, fit.shape, fit.keel, mesh, nullptr, &desc.boat);
    check(st == ari::vehicle_model_status::ready && !mesh.model && !mesh.texture,
          at + "its box look's mesh is ready: its hull, not a model");
    const float topsides[3] = {0.2f, 0.4f, 0.6f}, stripe[3] = {0.8f, 0.1f, 0.9f};
    std::shared_ptr<GeometryNode> hull =
        ari::add_hull_node(sg, "hull", nullptr, desc.boat, topsides, stripe);
    if (!check(hull && hull->getGeometry(), at + "add_hull_node draws its hull"))
      continue;
    const cvc::geometry &g = *hull->getGeometry();
    const std::size_t n = g.const_points().size();
    bool same = n > 0 && mesh.verts.size() == 3 * n && mesh.normals.size() == 3 * n &&
                mesh.role.size() == n && mesh.own.size() == 3 * n && g.const_colors().size() == n &&
                g.const_normals().size() == n && mesh.tris.size() == 3 * g.const_tris().size();
    std::size_t roles[3] = {0, 0, 0}, teak = 0;
    for (std::size_t i = 0; same && i < n; ++i) {
      const unsigned char r = mesh.role[i];
      if (r > 2) {
        same = false;
        break;
      }
      ++roles[r];
      const float *given = r == 0 ? topsides : r == 1 ? stripe : nullptr;
      for (int k = 0; k < 3; ++k) {
        same = same && std::fabs(mesh.verts[3 * i + k] - g.const_points()[i][k]) < 1e-9 &&
               std::fabs(mesh.normals[3 * i + k] - g.const_normals()[i][k]) < 1e-9;
        const double want = given ? given[k] : mesh.own[3 * i + k] / 255.0;
        same = same && std::fabs(g.const_colors()[i][k] - want) < 0.003;
      }
      teak += r == 2 && mesh.own[3 * i] == 140 ? 1 : 0; // the teak's 0.55 red (the bottom's 0.36)
    }
    for (std::size_t t = 0; same && t < g.const_tris().size(); ++t)
      for (int k = 0; k < 3; ++k)
        same = same && mesh.tris[3 * t + k] == g.const_tris()[t][k];
    // A box hull's corners are at its bottom and its sheer, none in the stripe's band.
    const bool box_hull = preset == "launch";
    check(same && roles[0] > 0 && (box_hull || roles[1] > 0) && roles[2] > 0,
          at + "each point where add_hull_node draws it: topsides role 0, stripe 1, the rest 2",
          std::to_string(n) + " points (" + std::to_string(roles[0]) + ", " +
              std::to_string(roles[1]) + ", " + std::to_string(roles[2]) + ")");
    check(box_hull ? teak == 4 : teak == 0,
          at + (box_hull ? "a box hull, open, with its teak sole" : "a V hull: no sole"),
          std::to_string(teak) + " teak points");
    if (box_hull)
      launch_points = n;
  }
  if (launch_points == 0 || !tactical.ok)
    return;
  {
    world_run w(lake_doc("launch", 40), false);
    if (!w.ok || !check(w.settle(40), "boats: the view takes all 40 launches"))
      return;
    const ari::entity_view_stats s = ari::entity_view_stats_of(w.rs);
    check(s.meshes == 1 && s.promoted == 8 && own_nodes(w.rs) == 8,
          "boats: one mesh (the hull), the nearest 8 their own nodes",
          std::to_string(s.meshes) + " meshes, " + std::to_string(own_nodes(w.rs)) + " nodes");
    bool hulls = own_nodes(w.rs) == 8;
    for (const auto &kv : w.rs.vehicle_markers) {
      std::shared_ptr<GraphicsNode> node = w.sg.getGraphics(kv.first);
      hulls = hulls && node && node->findChildByName(kv.first + "_hull") != nullptr;
    }
    check(hulls, "boats: a promoted launch's node is its hull, as before");
    std::size_t n = 0;
    const bool where = copies_where_entities_are(w, "entities_boat_lod0", 3.0, &n, launch_points);
    check(where && n >= 24, "boats: each copy where an unpromoted launch is",
          std::to_string(n) + " copies");
    // The water reads where the world's boats are, not their nodes: the 4 nearest kept out.
    std::shared_ptr<GraphicsNode> water = w.sg.getGraphics("water");
    auto far =
        water ? std::dynamic_pointer_cast<GeometryNode>(water->findChildByName("far")) : nullptr;
    std::array<float, 3> e0{}, e3{};
    const bool kept = far && far->shaderUniform3f("uHullE0", e0) &&
                      far->shaderUniform3f("uHullE3", e3) && std::fabs(e0[0] - 2.0f) < 1e-5 &&
                      std::fabs(e3[0] - 2.0f) < 1e-5;
    check(kept, "boats: the water keeps out of the 4 nearest launches' hulls",
          std::to_string(e0[0]) + ", " + std::to_string(e3[0]) + " m half lengths");
  }
  // A one-piece model: a box filed 4 m long, on z 0 (fit_vehicle_geometry stands it so).
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) / "cvcgl_ariadne_entities_boats";
  fs::create_directories(dir, ec);
  box_obj(dir / "boat.obj", {-2.0, -0.6, 0.0}, {2.0, 0.6, 0.8});
  const std::string file = (dir / "boat.obj").generic_string();
  std::size_t per = 0;
  {
    cvc::app app;
    const std::shared_ptr<ari::vehicle_model_cache> cache = ari::make_vehicle_model_cache();
    ca::vehicle_look look;
    look.kind = "file";
    look.path = file;
    look.fit = true;
    ari::vehicle_look_mesh mesh;
    if (ari::vehicle_look_mesh_of(app, *cache, look, tactical.shape, tactical.keel, mesh) ==
        ari::vehicle_model_status::ready)
      per = mesh.verts.size() / 3;
  }
  if (!check(per > 0, "boats: the model's mesh is ready"))
    return;
  world_run w(lake_doc("tactical", 40, "{ file: \"" + file + "\", fit: true }",
                       R"(    - node: bodies
      type: entity_layer
      lod: false
)"),
              false);
  if (!w.ok || !check(w.settle(40), "boats, a model: the layer takes all 40"))
    return;
  check(own_nodes(w.rs) == 8, "boats, a model: the nearest 8 their own nodes");
  bool fitted = own_nodes(w.rs) == 8;
  for (const auto &kv : w.rs.vehicle_markers) {
    std::shared_ptr<GraphicsNode> node = w.sg.getGraphics(kv.first);
    std::shared_ptr<GraphicsNode> m = node ? node->findChildByName(kv.first + "_mesh") : nullptr;
    fitted = fitted && m && !node->findChildByName(kv.first + "_hull") &&
             std::fabs(m->getTransform()->GetElement(2, 3) - tactical.keel) < 1e-9;
  }
  check(fitted,
        "boats, a model: a promoted boat's node is its model, its keel on its hull's bottom");
  // Each copy: its lowest point its boat's keel, its length along its heading the boat's overall.
  std::shared_ptr<GraphicsNode> layer = w.sg.getGraphics("bodies");
  std::shared_ptr<GraphicsNode> lod0 = layer ? layer->findChildByName("bodies_boat_lod0") : nullptr;
  const std::vector<double> pts = drawn(lod0);
  std::set<std::uint64_t> drawn_as_node;
  for (const auto &kv : w.rs.vehicle_markers)
    drawn_as_node.insert(std::stoull(kv.first.substr(8)));
  const std::vector<ca::entity_info> ents = w.entities();
  std::size_t placed = 0;
  double worst_keel = 0.0, worst_length = 0.0;
  for (std::size_t c = 0; (c + 1) * per * 3 <= pts.size(); ++c) {
    double cx = 0.0, cy = 0.0, lo = 1e30, hi = -1e30;
    for (std::size_t k = 0; k < per; ++k) {
      const double *p = &pts[(c * per + k) * 3];
      cx += p[0] / static_cast<double>(per);
      cy += p[1] / static_cast<double>(per);
      lo = std::min(lo, p[0]);
      hi = std::max(hi, p[0]);
    }
    if (hi - lo < 1e-9)
      continue; // collapsed: no entity in this slot
    const ca::entity_info *e = nullptr;
    for (const ca::entity_info &x : ents)
      if (!drawn_as_node.count(x.id) &&
          (!e || std::hypot(x.x - cx, x.y - cy) < std::hypot(e->x - cx, e->y - cy)))
        e = &x;
    if (!e)
      break;
    double low = 1e30, a0 = 1e30, a1 = -1e30;
    for (std::size_t k = 0; k < per; ++k) {
      const double *p = &pts[(c * per + k) * 3];
      low = std::min(low, p[2]);
      const double along = (p[0] - e->x) * std::cos(e->yaw) + (p[1] - e->y) * std::sin(e->yaw);
      a0 = std::min(a0, along);
      a1 = std::max(a1, along);
    }
    ++placed;
    worst_keel = std::max(worst_keel, std::fabs(low - (e->z + tactical.keel)));
    worst_length = std::max(worst_length, std::fabs(a1 - a0 - tactical.shape.length));
  }
  check(placed >= 24 && worst_keel < 0.25 && worst_length < 0.2,
        "boats, a model: each copy's keel on its hull's bottom, its length overall the boat's",
        std::to_string(placed) + " copies, keels within " + std::to_string(worst_keel) +
            " m, lengths within " + std::to_string(worst_length) + " m of " +
            std::to_string(tactical.shape.length));
}

} // namespace

int main() {
  installErrorCounter();
  std::printf("cvcgl_ariadne_entities\n");
  {
    cvc::app probe;
    if (!renderAvailable(probe))
      return finish("cvcgl_ariadne_entities");
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return finish("cvcgl_ariadne_entities");
  }
  test_implicit_follow_keep();
  test_threshold();
  test_of_fleet();
  test_worker();
  test_room();
  test_parted();
  test_boats();
  return finish("cvcgl_ariadne_entities");
}
