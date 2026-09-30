// cvcgl_lod_node -- LodGraphicsNode picks the right rung by camera distance.
//
// Headless: builds a 3-rung LOD node (a unit box at each rung, with a
// world-error ladder 0 / 1 / 4 m) and calls select(view) from a series of
// eye distances. No renderer, so it exercises the whole selection path without a
// GL context. Asserts: a close camera picks the finest rung (0), a very far one
// picks the coarsest (2), the choice coarsens monotonically with distance, and
// exactly one rung is selected. select() is the seam between cvc::lod's pure math
// and the scene graph, so this is the integration gate for that seam.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>

namespace {

cvc::geometry unit_box(cvc::app &app) {
  cvc::geometry g(app);
  cvc::geometry::points_t &P = g.points();
  for (int i = 0; i < 8; ++i)
    P.push_back({(i & 1) ? 0.5 : -0.5, (i & 2) ? 0.5 : -0.5, (i & 4) ? 0.5 : -0.5});
  const int faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                            {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  cvc::geometry::tris_t &T = g.tris();
  for (const auto &f : faces)
    T.push_back({std::uint64_t(f[0]), std::uint64_t(f[1]), std::uint64_t(f[2])});
  g.set_geometry_type(cvc::geometry::SURFACE_TRI);
  return g;
}

} // namespace

int main() {
  cvc::app app;

  // 3 rungs (all the same unit box; only the error ladder drives selection).
  cvc::lod::mesh_pyramid pyr;
  for (int k = 0; k < 3; ++k)
    pyr.rungs.push_back(unit_box(app));
  pyr.world_error_m = {0.0, 1.0, 4.0};

  auto node = std::make_shared<cvc::gl::LodGraphicsNode>(app, "test.lod", "lod");
  node->setPyramid(pyr);
  assert(node->rungCount() == 3 && "three rungs installed");

  cvc::lod::view_params view = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  auto pick = [&](double dist) {
    view.eye[0] = 0.0;
    view.eye[1] = 0.0;
    view.eye[2] = dist; // box centre is the origin
    return node->select(view);
  };

  const int at_near = pick(1.0);
  const int at_far = pick(1.0e6);
  assert(at_near == 0 && "a close camera must draw the finest rung");
  assert(at_far == 2 && "a very far camera must drop to the coarsest rung");
  assert(node->selectedRung() == at_far && "selectedRung tracks the last select()");

  // Monotone coarsening as the camera retreats.
  int prev = pick(0.6); // right at the surface
  const double dists[] = {2.0, 10.0, 50.0, 200.0, 1000.0, 20000.0, 1.0e6};
  for (double d : dists) {
    int r = pick(d);
    assert(r >= prev && "rung must not get FINER as the camera retreats");
    assert(r >= 0 && r < node->rungCount());
    prev = r;
  }

  // SceneGraph::selectLOD drives every LodGraphicsNode in the scene at once.
  {
    cvc::gl::SceneGraph sg(app, "lodtest");
    auto lod = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::LodGraphicsNode>("lodnode");
    lod->setPyramid(pyr);
    view.eye[0] = 0.0;
    view.eye[1] = 0.0;
    view.eye[2] = 1.0e6; // far
    const int visited = sg.selectLOD(view);
    assert(visited == 1 && "selectLOD visits the one LOD node");
    assert(lod->selectedRung() == 2 && "far view -> coarsest rung via selectLOD");
    view.eye[2] = 1.0; // near
    sg.selectLOD(view);
    assert(lod->selectedRung() == 0 && "near view -> finest rung via selectLOD");
  }

  std::printf("cvcgl_lod_node OK\n");
  return 0;
}
