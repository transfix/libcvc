// Prototypes (docs/ARIADNE.md §5.18, "Prototypes"): a node's `use: <name>` stands in for a node of
// the document's `prototypes:`, realized by realize_scene with no window.
//
// Pins:
//   - each instance is its own node tree, under its own id, at its own transform, its parts its
//     own GeometryNodes (each with its own material);
//   - every mesh part is read once for all its instances (two in the scene, three repeat: copies,
//     one in a scenes: entry), and the instances draw from one point buffer and one set of VTK
//     arrays (GeometryNode::shareArraysFrom);
//   - a part whose source holds an arg is read per instance, at that instance's size;
//   - an instance's args reach its parts (a visible: bind of its own).
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkProperty.h>

namespace {

namespace gla = cvc::gl::ariadne;
using cvc::gl::GeometryNode;
using cvc::gl::GraphicsNode;
using cvcgl_test::check;

const char *kDoc = R"(meta: { min_libcvc: "0.0.0" }
prototypes:
  lamp:
    type: group
    args: { switch: lamps.on }
    children:
      - node: pole
        source: { box: { size: [0.2, 0.2, 5] } }
        transform: { position: [0, 0, 2.5] }
        material: { color: [0.2, 0.2, 0.2] }
      - node: bulb
        source: { sphere: 0.4 }
        transform: { position: [0, 0, 5.2] }
        visible: "{switch}"
        material: { color: [1, 0.9, 0.6] }
  shade:
    args: { r: 0.5 }
    source: { sphere: "{r}" }
scene:
  nodes:
    - { node: lamp_a, use: lamp, transform: { position: [1, 2, 0] } }
    - node: lamp_b
      use: lamp
      args: { switch: lamps.north }
      transform: { position: [10, 0, 0] }
    - { node: "row_{i}", use: lamp, repeat: 3, transform: { position: [0, -5, 0] } }
    - { node: small, use: shade }
    - { node: large, use: shade, args: { r: 2 }, material: { color: [0, 1, 0] } }
scenes:
  - scene: other
    nodes:
      - { node: lamp_c, use: lamp }
)";

std::shared_ptr<GeometryNode> part(const std::shared_ptr<GraphicsNode> &lamp,
                                   const std::string &name) {
  return lamp ? std::dynamic_pointer_cast<GeometryNode>(lamp->findChildByName(name)) : nullptr;
}

bool world_at(const std::shared_ptr<GraphicsNode> &n, double x, double y, double z) {
  if (!n)
    return false;
  const double o[3] = {0, 0, 0};
  double w[3] = {0, 0, 0};
  n->localToWorld(o, w);
  return std::fabs(w[0] - x) < 1e-4 && std::fabs(w[1] - y) < 1e-4 && std::fabs(w[2] - z) < 1e-4;
}

double extent_x(const std::shared_ptr<GeometryNode> &g) {
  const cvc::geometry *geom = g ? g->getGeometry() : nullptr;
  if (!geom || geom->num_points() == 0)
    return 0.0;
  const cvc::bounding_box b = geom->extents();
  return b.maxx - b.minx;
}

} // namespace

int main() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  const cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, kDoc);
  check(lr.ok, "the document loads", lr.error);
  for (const std::string &w : lr.warnings)
    check(false, "the document loads without warnings", w);
  if (!lr.ok)
    return cvcgl_test::finish("cvcgl_ariadne_prototypes");

  cvc::gl::SceneGraph sg(app, "proto"), other(app, "proto_other");
  gla::SceneAssetCache cache;
  gla::RealizeOptions o;
  o.assets = &cache;
  std::vector<std::string> warns;
  gla::RealizedScene rs = gla::realize_scene(sg, lr.scene, "proto", &warns, o);
  gla::RealizedScene rs2 = gla::realize_scene(other, lr.scenes[0], "proto", &warns, o);
  for (int i = 0; i < 3; ++i) // the repeat: copies are made by the scene's first ticks
    gla::sync_scene_repeats(rs, &warns);
  sg.processEvents();
  other.processEvents();
  for (const std::string &w : warns)
    check(false, "it realizes without warnings", w);

  std::printf("Each instance is its own node tree\n");
  const std::shared_ptr<GraphicsNode> a = sg.getGraphics("lamp_a"), b = sg.getGraphics("lamp_b"),
                                      c = other.getGraphics("lamp_c");
  check(a && b && c, "lamp_a, lamp_b and the other scene's lamp_c exist");
  check(world_at(a, 1, 2, 0) && world_at(b, 10, 0, 0), "each at its own transform");
  const std::shared_ptr<GeometryNode> pa = part(a, "pole"), pb = part(b, "pole"),
                                      pc = part(c, "pole"), ba = part(a, "bulb"),
                                      bb = part(b, "bulb");
  check(pa && pb && pc && ba && bb && pa != pb, "each has its own pole and bulb");
  check(world_at(pa, 1, 2, 2.5) && world_at(pb, 10, 0, 2.5), "a part placed in its instance");
  std::shared_ptr<GraphicsNode> rows[3];
  for (int i = 0; i < 3; ++i)
    rows[i] = sg.getGraphics("row_" + std::to_string(i));
  check(rows[0] && rows[1] && rows[2] && part(rows[2], "pole"), "repeat: makes three instances");

  std::printf("Meshes are read once and shared\n");
  // pole and bulb: one read each, for six lamps in two scenes; small and large: one each.
  check(cache.reads == 2, "each shared part is read once", std::to_string(cache.reads));
  const cvc::geometry *ga = pa ? pa->getGeometry() : nullptr;
  const cvc::geometry *gb = pb ? pb->getGeometry() : nullptr;
  const cvc::geometry *gc = pc ? pc->getGeometry() : nullptr;
  const cvc::geometry *gr = rows[1] ? part(rows[1], "pole")->getGeometry() : nullptr;
  check(ga && gb && gc && gr && ga->num_points() > 0 && ga->points_ptr() == gb->points_ptr() &&
            ga->points_ptr() == gc->points_ptr() && ga->points_ptr() == gr->points_ptr(),
        "every instance's pole draws the same point buffer");
  check(pa && pb && pc && pa->sharesArrays() && pb->sharesArrays() && pc->sharesArrays(),
        "and the same VTK arrays");
  const cvc::geometry *gba = ba ? ba->getGeometry() : nullptr;
  check(gba && ga && gba->points_ptr() != ga->points_ptr(), "the bulb is a mesh of its own");

  std::printf("Each instance keeps what is its own\n");
  auto color_of = [](const std::shared_ptr<cvc::gl::GraphicsNode> &n) {
    auto *actor = n ? vtkActor::SafeDownCast(n->prop()) : nullptr;
    return actor ? actor->GetProperty()->GetColor()[1] : -1.0;
  };
  check(std::fabs(color_of(pa) - 0.2) < 1e-6 && std::fabs(color_of(ba) - 0.9) < 1e-6, // floats
        "each part has the prototype's material",
        std::to_string(color_of(pa)) + ", " + std::to_string(color_of(ba)));
  const std::shared_ptr<GeometryNode> small =
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("small"));
  const std::shared_ptr<GeometryNode> large =
      std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("large"));
  check(extent_x(small) > 0.5 && std::fabs(extent_x(large) / extent_x(small) - 4.0) < 0.02,
        "an arg in a source: each instance its own size (radius 2 against 0.5)",
        std::to_string(extent_x(small)) + ", " + std::to_string(extent_x(large)));
  check(std::fabs(color_of(large) - 1.0) < 1e-6, "and the use site's material");
  bool on = false, north = false;
  for (const cvc::ariadne::SceneVisibilityBinding &v : rs.visibility) {
    on = on || v.source_path == "proto.lamps.on";
    north = north || v.source_path == "proto.lamps.north";
  }
  check(on && north, "each instance's bulb follows its own switch (the arg)");

  return cvcgl_test::finish("cvcgl_ariadne_prototypes");
}
