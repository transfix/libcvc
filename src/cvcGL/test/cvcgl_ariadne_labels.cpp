/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_ariadne_labels -- a scene node's `label:` and the scene's `entity_labels:`
// (scene_labels.h, LabelLayer.h, ARIADNE.md §5.21), loaded from a document and rendered for real:
//
//   fade     label_fade: whole nearer than near, gone past far, linear between;
//   pixels   a label draws its text over its backdrop inside its edge, above its node;
//   cull     a label past its far distance is not drawn, one in its band is drawn fainter;
//   order    labels are drawn farthest first, so the nearer one lies on top;
//   bind     bind: is the text, read as the label draws; scale: and focus: scale it;
//   views    a hidden node's label is not drawn, nor one whose node a renderer leaves out;
//   still    a still frame rasterises no text again, and writes no state;
//   entity   entity_labels: gives a world entity its name as a label.
//
// Skips (rc 0) where nothing rasterises unless CVC_REQUIRE_RENDER=1. Synthetic data only.

#include "streaming_test_util.h"

#include <algorithm>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/scene_world.h>
#include <cvc/ariadne/world_rules.h>
#include <cvc/ariadne/world_vehicles.h>
#include <cvc/gl/LabelLayer.h>
#include <cvc/gl/ariadne/scene_labels.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/physics/world.h>
#include <cvc/state/state.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkCamera.h>
#include <vtkRenderer.h>

using namespace cvcgl_test;
using cvc::gl::LabelLayerNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
namespace ari = cvc::gl::ariadne;

namespace {

const std::string kPrefix = "doc";
constexpr int W = 320, H = 240;

void put(cvc::app &app, const std::string &k, const std::string &v) {
  cvc::ariadne::write_node(app, kPrefix + "." + k).value(v);
}

// The camera at (0, -100, 0) looking along +y: a node at y has distance ~ y + 100.
const char *kDoc = R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: near
      source: { box: 2 }
      transform: { position: [-10, -60, -5] }
      material: { color: [0.3, 0.3, 0.3] }
      label:
        text: NEAR
        color: [1, 0, 0]
        backdrop: [0, 0, 1]
        backdrop_opacity: 1
        edge: [0, 1, 0]
        edge_width: 2
        offset: 1
        size: 28
        fade: [100, 200]
    - node: banded
      source: { box: 2 }
      transform: { position: [30, 50, -5] }
      label: { text: BAND, fade: [100, 200] }
    - node: culled
      source: { box: 2 }
      transform: { position: [0, 150, -5] }
      label: { text: GONE, fade: [100, 200] }
    - node: behind
      source: { box: 2 }
      transform: { position: [-40, -40, -5] }
      label: { text: BACK, size: 20 }
    - node: bound
      source: { box: 2 }
      transform: { position: [0, -20, 10] }
      label:
        bind: names.bound
        scale: sizes.bound
        focus: { key: cam.follow, scale: 2 }
    - node: plain
      source: { box: 2 }
      transform: { position: [30, -20, 10] }
      label: Plain
    - node: hidden
      source: { box: 2 }
      transform: { position: [-30, -20, 10] }
      visible: false
      label: HIDDEN
    - node: unlabelled
      source: { box: 2 }
      transform: { position: [0, 0, 20] }
)";

const LabelLayerNode::Drawn *drawn(const std::vector<LabelLayerNode::Drawn> &d,
                                   const std::string &text) {
  for (const auto &x : d)
    if (x.text == text)
      return &x;
  return nullptr;
}

void test_fade_rule() {
  std::printf("fade: label_fade\n");
  check(cvc::gl::label_fade(50, 100, 200) == 1.0 && cvc::gl::label_fade(100, 100, 200) == 1.0,
        "fade: whole up to near");
  check(std::fabs(cvc::gl::label_fade(150, 100, 200) - 0.5) < 1e-12 &&
            std::fabs(cvc::gl::label_fade(175, 100, 200) - 0.25) < 1e-12,
        "fade: linear across the band");
  check(cvc::gl::label_fade(200, 100, 200) == 0.0 && cvc::gl::label_fade(1e6, 100, 200) == 0.0,
        "fade: gone at and past far");
  check(cvc::gl::label_fade(1e6, 0, 0) == 1.0, "fade: no fade: is never faded");
  std::vector<std::string> warnings;
  cvc::ariadne::Value v;
  v.kind = cvc::ariadne::Value::Kind::Map;
  cvc::ariadne::Value fade;
  fade.kind = cvc::ariadne::Value::Kind::Sequence;
  for (const char *x : {"300", "600"}) {
    cvc::ariadne::Value s;
    s.scalar = x;
    fade.items.push_back(s);
  }
  v.entries.emplace_back("fade", fade);
  const cvc::gl::LabelStyle st = ari::parse_label_style(v, "t", &warnings);
  check(st.fade_near == 300 && st.fade_far == 600 && warnings.empty(),
        "fade: [300, 600] parses as near 300, far 600");
}

void test_scene(cvc::app &app) {
  std::printf("scene: label: on nodes\n");
  cvc::ariadne::LoadResult doc = cvc::ariadne::load_string(app, kDoc);
  if (!check(doc.ok, "the document loads", doc.error))
    return;
  put(app, "names.bound", "Bravo");
  SceneGraph sg(app, "labels");
  sg.setDiagnosticChromeVisible(false);
  std::vector<std::string> warnings;
  ari::RealizedScene rs = ari::realize_scene(sg, doc.scene, kPrefix, &warnings);
  for (const std::string &w : warnings)
    std::printf("  warning: %s\n", w.c_str());
  check(warnings.empty(), "realize: no warnings");
  std::shared_ptr<LabelLayerNode> layer = LabelLayerNode::of(sg);
  check(layer->size() == 7, "realize: seven labels, on the labelled nodes",
        std::to_string(layer->size()));
  check(layer->labelsOf(sg.getGraphics("unlabelled").get()).empty(),
        "realize: a node with no label: has none");
  check(std::find(rs.created.begin(), rs.created.end(), LabelLayerNode::kDefaultName) !=
            rs.created.end(),
        "realize: the layer is one of the scene's nodes");

  SceneRenderer sr(sg, W, H, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  sr.setCamera(0, -100, 0, 0, 0, 0, 0, 0, 1, 60.0, 0.5, 2000.0);
  const auto frame = [&]() {
    cvc::ariadne::sync_scene_visibility(app, rs.visibility);
    ari::tick_scene(rs, sr.renderer(), 1.0 / 60.0);
    return grab(sr);
  };
  Frame f = frame();
  std::vector<LabelLayerNode::Drawn> d = layer->lastDrawn();

  // Fade and cull.
  const LabelLayerNode::Drawn *nearL = drawn(d, "NEAR"), *band = drawn(d, "BAND");
  check(nearL && nearL->opacity == 1.0, "fade: a label nearer than near is whole");
  check(band && band->opacity > 0.0 && band->opacity < 1.0 &&
            std::fabs(band->opacity - cvc::gl::label_fade(band->distance, 100, 200)) < 1e-9,
        "fade: a label in the band is drawn as faint as label_fade says",
        band ? std::to_string(band->distance) + " m, " + std::to_string(band->opacity) : "");
  check(!drawn(d, "GONE"), "cull: a label past far is not drawn");
  check(!drawn(d, "HIDDEN"), "views: a hidden node's label is not drawn");

  // Depth order: farthest first.
  bool sorted = true;
  for (std::size_t i = 1; i < d.size(); ++i)
    sorted = sorted && d[i - 1].distance >= d[i].distance;
  check(sorted && d.size() >= 4, "order: drawn farthest first");

  // Bound text, scale and focus.
  const LabelLayerNode::Drawn *b = drawn(d, "Bravo");
  check(b && b->size == 14, "bind: the bound key is the text; size 14 at scale 1");
  check(drawn(d, "Plain") != nullptr, "label: <text> is the text");

  // Pixels: text over its backdrop inside its edge, just above the node.
  if (nearL) {
    int red = 0, blue = 0, green = 0;
    for (int y = int(nearL->y); y < int(nearL->y) + 40; ++y)
      for (int x = int(nearL->x) - 50; x <= int(nearL->x) + 50; ++x) {
        const unsigned char *p = f.at(x, y);
        red += isHue(p, 0) ? 1 : 0;
        green += isHue(p, 1) ? 1 : 0;
        blue += isHue(p, 2) ? 1 : 0;
      }
    check(blue > 200 && red > 30 && green > 30,
          "pixels: red text on a blue backdrop in a green edge, over its node",
          "blue " + std::to_string(blue) + " red " + std::to_string(red) + " green " +
              std::to_string(green));
    int below = 0;
    for (int y = int(nearL->y) - 30; y < int(nearL->y) - 6; ++y)
      for (int x = int(nearL->x) - 50; x <= int(nearL->x) + 50; ++x)
        below += isHue(f.at(x, y), 2) ? 1 : 0;
    check(below == 0, "pixels: nothing of it below its anchor");
  }

  // A still frame: no text rasterised again.
  const std::uint64_t r0 = layer->rasterisations();
  frame();
  check(layer->rasterisations() == r0, "still: a still frame rasterises nothing again");

  // Bound changes, read as it draws.
  put(app, "names.bound", "Charlie");
  put(app, "sizes.bound", "1.5");
  frame();
  d = layer->lastDrawn();
  b = drawn(d, "Charlie");
  check(b && b->size == 21, "bind: a new text and scale: (1.5) next frame",
        b ? std::to_string(b->size) : "not drawn");
  put(app, "cam.follow", "bound");
  frame();
  b = drawn(layer->lastDrawn(), "Charlie");
  check(b && b->size == 42, "focus: the followed node's label is focus.scale (2) larger",
        b ? std::to_string(b->size) : "not drawn");
  put(app, "cam.follow", "plain");
  frame();
  b = drawn(layer->lastDrawn(), "Charlie");
  check(b && b->size == 21, "focus: and back once the key moves on");

  // Moving the camera closer brings the band label in whole, and the culled one back.
  sr.setCamera(0, 60, 0, 0, 200, 0, 0, 0, 1, 60.0, 0.5, 2000.0); // 90 m from it
  frame();
  d = layer->lastDrawn();
  check(drawn(d, "GONE") && drawn(d, "GONE")->opacity == 1.0, "cull: closer, it is drawn again");

  // A renderer that leaves a node out leaves its label out too (a view's tags).
  sr.setCamera(0, -100, 0, 0, 0, 0, 0, 0, 1, 60.0, 0.5, 2000.0);
  vtkProp *plainProp = nullptr;
  {
    std::vector<vtkProp *> props;
    sg.getGraphics("plain")->collectViewProps(props);
    plainProp = props.empty() ? nullptr : props[0];
  }
  if (check(plainProp != nullptr, "views: the plain node has a prop")) {
    plainProp->Register(nullptr);
    sr.renderer()->RemoveViewProp(plainProp);
    frame();
    check(!drawn(layer->lastDrawn(), "Plain"), "views: its label goes with its prop");
    sr.renderer()->AddViewProp(plainProp);
    plainProp->UnRegister(nullptr);
    frame();
    check(drawn(layer->lastDrawn(), "Plain") != nullptr, "views: and comes back with it");
  }

  // A moved node's label follows it, read from its transform.
  sg.getGraphics("plain")->setPosition(-10, -20, 10);
  frame();
  const LabelLayerNode::Drawn *moved = drawn(layer->lastDrawn(), "Plain");
  const LabelLayerNode::Drawn *still = drawn(layer->lastDrawn(), "Charlie");
  check(moved && still && moved->x < still->x, "follow: the label moves with its node");
}

void test_entities(cvc::app &app) {
  std::printf("entity: entity_labels:\n");
  {
    // The label an entity's node gets, without a world: its fields filled into the text.
    SceneGraph sg(app, "entity_direct");
    std::shared_ptr<cvc::gl::GraphicsNode> node = sg.addGraphics("vehicle_7");
    cvc::ariadne::Value style;
    style.kind = cvc::ariadne::Value::Kind::Map;
    cvc::ariadne::Value text;
    text.scalar = "{name} ({team})";
    style.entries.emplace_back("text", text);
    cvc::ariadne::Value focus;
    focus.scalar = "cam.follow";
    style.entries.emplace_back("focus", focus);
    std::vector<std::string> warnings;
    ari::apply_entity_label(sg, node, style, {{"name", "Alpha"}, {"id", "7"}, {"team", "2"}}, "doc",
                            &warnings);
    std::shared_ptr<LabelLayerNode> layer = LabelLayerNode::of(sg);
    const std::vector<LabelLayerNode::Id> ids = layer->labelsOf(node.get());
    check(ids.size() == 1 && layer->text(ids[0]) == "Alpha (2)" && warnings.empty(),
          "entity: its fields fill the text", ids.empty() ? "none" : layer->text(ids[0]));
    check(!ids.empty() && layer->style(ids[0]).focus_key == "doc.cam.follow" &&
              layer->style(ids[0]).focus_value == "vehicle_7",
          "entity: focus: follows its node's id, under the scene's prefix");
    cvc::ariadne::Value on;
    on.scalar = "true";
    std::shared_ptr<cvc::gl::GraphicsNode> unnamed = sg.addGraphics("vehicle_8");
    ari::apply_entity_label(sg, unnamed, on, {{"name", ""}, {"id", "8"}}, "doc", &warnings);
    const std::vector<LabelLayerNode::Id> u = layer->labelsOf(unnamed.get());
    check(u.size() == 1 && layer->text(u[0]) == "vehicle_8",
          "entity: entity_labels: true labels an unnamed entity with its node's id");
  }
  if (!cvc::physics::backend_available(cvc::physics::backend::jolt)) {
    std::printf("  skipped: no Jolt in this build\n");
    return;
  }
  cvc::ariadne::register_world_block(app);
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(app, R"(
meta: { min_libcvc: "0.0.0" }
world:
  vehicles:
    archetypes:
      humvee: { vehicle: humvee }
scene:
  physics: {}
  world: { source: square, cell: 2 }
  entity_labels: { text: "{name} #{id}", offset: 2, fade: [300, 600] }
  nodes:
    - node: square
      world: square
      source: { box: { size: [80, 80, 1] } }
      transform: { position: [0, 0, -0.5] }
      body: static
)");
  if (!check(r.ok, "entity: the document loads", r.error))
    return;
  check(r.scene.entity_labels.is_map(), "entity: entity_labels: parses");
  cvc::ariadne::world_rules rules = cvc::ariadne::default_world_rules();
  cvc::ariadne::parse_world_block(*cvc::ariadne::find_world_block(r), rules);
  r.scene.vehicles = rules.vehicles;
  r.scene.spawns = rules.spawns;
  SceneGraph sg(app, "entities");
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, "", &warnings);
  if (!check(out.world && out.world->vehicles(), "entity: the world has vehicles"))
    return;
  cvc::state &st = cvc::state::instance(app);
  ari::tick_scene(out, nullptr, 1.0 / 60.0);
  st("vehicles.spawn.at").value("5,-3,0");
  st("vehicles.spawn.name").value("Alpha");
  st("vehicles.spawn.request").value("1");
  for (int i = 0; i < 30; ++i)
    ari::tick_scene(out, nullptr, 1.0 / 60.0);
  std::shared_ptr<cvc::gl::GraphicsNode> node = sg.getGraphics("vehicle_1");
  std::shared_ptr<LabelLayerNode> layer = LabelLayerNode::of(sg);
  const std::vector<LabelLayerNode::Id> ids =
      node ? layer->labelsOf(node.get()) : std::vector<LabelLayerNode::Id>();
  check(ids.size() == 1 && layer->text(ids[0]) == "Alpha #1",
        "entity: a spawned entity's node has its name as a label",
        ids.empty() ? "none" : layer->text(ids[0]));
  if (!ids.empty())
    check(layer->style(ids[0]).fade_far == 600 && layer->style(ids[0]).offset[2] == 2,
          "entity: in entity_labels:'s style");
}

} // namespace

int main() {
  installErrorCounter();
  cvc::app app;
  std::printf("cvcgl_ariadne_labels\n");
  test_fade_rule();
  if (renderAvailable(app))
    test_scene(app);
  test_entities(app);
  check(ErrorCounter::errors() == 0, "no VTK errors");
  return finish("cvcgl_ariadne_labels");
}
