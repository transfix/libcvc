// cvc::gl::add_world_geometry -- the world's geometry as scene-graph nodes, the
// same objects the physics world collides with. Headless. These checks pin that:
//   1. one node per render item, named prefix + item name; other roles are skipped;
//   2. a node's geometry IS the item's (the arrays are shared, not copied: the
//      same points buffer), plus colours where the item had none;
//   3. colours come from the surfaces: a terrain cell's albedo, a mesh's surface;
//   4. a mesh's placement becomes the node's pose; a height field is never moved;
//   5. under a frame node, the nodes are its children, so its transform moves the
//      whole world (world = frame x placement).
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/world_geometry_nodes.h>
#include <cvc/world/world_geometry.h>
#include <memory>
#include <vtkMatrix4x4.h>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using namespace cvc::world;

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

world_geometry sample_world() {
  world_geometry w;
  const surface_registry &s = w.surfaces();
  // A 3 x 3 terrain: the west cells asphalt, the east cells grass.
  auto field = std::make_shared<cvc::height_field>(
      3, 3, 0.0, 0.0, 1.0, 1.0, std::vector<float>(9, 0.0f),
      std::vector<cvc::face_material_t>{s.by_name("asphalt"), s.by_name("grass"),
                                        s.by_name("asphalt"), s.by_name("grass")});
  w.add({"terrain", field});
  auto roof = std::make_shared<cvc::geometry>();
  roof->points() = {{{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}}};
  roof->tris().push_back({{0, 1, 2}});
  geometry_item placed{"roof", roof, s.by_name("building_brick")};
  placed.position = {{5.0, -2.0, 3.0}};
  w.add(placed);
  geometry_item hidden{"collider", roof};
  hidden.roles = role_collide;
  w.add(hidden);
  return w;
}

void test_one_node_per_render_item() {
  std::printf("test_one_node_per_render_item\n");
  cvc::app app;
  SceneGraph sg(app);
  const world_geometry w = sample_world();
  const auto nodes = cvc::gl::add_world_geometry(sg, w, "city.");
  sg.processEvents();
  CHECK(nodes.size() == 2);
  CHECK(sg.getGraphics("city.terrain") != nullptr);
  CHECK(sg.getGraphics("city.roof") != nullptr);
  CHECK(sg.getGraphics("city.collider") == nullptr);
}

void test_nodes_share_the_items_arrays_and_take_surface_colours() {
  std::printf("test_nodes_share_the_items_arrays_and_take_surface_colours\n");
  cvc::app app;
  SceneGraph sg(app);
  const world_geometry w = sample_world();
  const auto nodes = cvc::gl::add_world_geometry(sg, w);
  sg.processEvents();
  auto terrain = std::dynamic_pointer_cast<GeometryNode>(nodes[0]);
  CHECK(terrain != nullptr);
  if (!terrain)
    return;
  const cvc::geometry *g = terrain->getGeometry();
  CHECK(g != nullptr);
  if (!g)
    return;
  const auto field = w.terrain();
  // The same points buffer: the node shares the item's arrays.
  CHECK(&g->const_points() == &field->const_points());
  CHECK(g->num_tris() == field->num_tris());
  // West edge vertices touch only asphalt cells; the middle column touches both.
  const surface_registry &s = w.surfaces();
  const float *asphalt = s[s.by_name("asphalt")].albedo;
  const float *grass = s[s.by_name("grass")].albedo;
  CHECK(g->const_colors().size() == g->num_points());
  const auto &west = g->const_colors()[field->vertex_of(1, 0)];
  CHECK(near(west[0], asphalt[0], 1e-6) && near(west[2], asphalt[2], 1e-6));
  const auto &east = g->const_colors()[field->vertex_of(1, 2)];
  CHECK(near(east[1], grass[1], 1e-6));
  const auto &mid = g->const_colors()[field->vertex_of(1, 1)];
  CHECK(near(mid[1], 0.5 * (asphalt[1] + grass[1]), 1e-6));
  // The item's own geometry gained no colours: only the node's copy did.
  CHECK(field->const_colors().empty());
}

void test_placement_becomes_the_pose() {
  std::printf("test_placement_becomes_the_pose\n");
  cvc::app app;
  SceneGraph sg(app);
  const auto nodes = cvc::gl::add_world_geometry(sg, sample_world());
  sg.processEvents();
  const vtkMatrix4x4 *roof = nodes[1]->getTransform();
  CHECK(roof && near(roof->GetElement(0, 3), 5.0) && near(roof->GetElement(1, 3), -2.0) &&
        near(roof->GetElement(2, 3), 3.0));
  const vtkMatrix4x4 *terrain = nodes[0]->getTransform();
  CHECK(terrain && near(terrain->GetElement(0, 3), 0.0) && near(terrain->GetElement(2, 3), 0.0));
}

void test_a_frame_node_carries_the_world() {
  std::printf("test_a_frame_node_carries_the_world\n");
  cvc::app app;
  SceneGraph sg(app);
  auto frame = sg.addGraphics("origin");
  const double shift[16] = {1, 0, 0, -1000, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  frame->setTransform(shift);
  const auto nodes = cvc::gl::add_world_geometry(sg, sample_world(), "", frame);
  sg.processEvents();
  CHECK(nodes.size() == 2);
  CHECK(frame->findChildByName("terrain") == nodes[0]); // a child of the frame
  CHECK(sg.getGraphics("terrain") == nullptr);          // not top-level
  const vtkMatrix4x4 *roof = nodes[1]->getTransform();  // local: the placement
  CHECK(roof && near(roof->GetElement(0, 3), 5.0));
  const auto world = nodes[1]->getWorldTransform(); // world: frame x placement
  CHECK(near(world->GetElement(0, 3), -995.0) && near(world->GetElement(2, 3), 3.0));
  CHECK(near(nodes[0]->getWorldTransform()->GetElement(0, 3), -1000.0));
}

} // namespace

int main() {
  test_one_node_per_render_item();
  test_nodes_share_the_items_arrays_and_take_surface_colours();
  test_placement_becomes_the_pose();
  test_a_frame_node_carries_the_world();
  if (g_failures) {
    std::printf("cvcgl_world_geometry_nodes: %d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_world_geometry_nodes: OK\n");
  return 0;
}
