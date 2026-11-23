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
//              one the client drives promoted with the nearest.
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
#include <cvc/gl/physics_scene_binding.h>
#include <cvc/physics/backend.h>
#include <cvc/state/state.h>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkMapper.h>
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
// scene nodes.
std::string world_doc(const std::vector<std::pair<std::string, int>> &fleets,
                      const std::string &extra = "") {
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
      humvee: { vehicle: humvee, controllers: [nav@1, none] }
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
// such entity on screen is a copy.
bool copies_where_entities_are(world_run &w, const std::string &mesh, double tol,
                               std::size_t *count = nullptr) {
  std::shared_ptr<GraphicsNode> node = w.sg.getGraphics("entities");
  std::shared_ptr<GraphicsNode> m = node ? node->findChildByName(mesh) : nullptr;
  if (!m)
    return false;
  // The box look: six boxes of 24 points.
  const std::vector<std::array<double, 2>> c = copies(drawn(m), 144);
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
  return finish("cvcgl_ariadne_entities");
}
