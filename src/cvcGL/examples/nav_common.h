// nav_common.h — shared helpers for the cvc::nav cvcGL demos (city_swarm / fog /
// finale). The load-bearing piece is AgentGlyphs: ONE merged mesh of N flat-arrow
// glyphs streamed each frame via GeometryNode::updateVertices — the path that scales
// to thousands of agents (per-node actors die at ~63 nodes / 3 FPS). occupancy_to_walls
// turns a cvc::nav occupancy grid into the static scene, and orbit_camera drives a
// scripted capture orbit. See src/cvcGL/examples/NAV_DEMOS.md.

#ifndef CVC_NAV_DEMO_NAV_COMMON_H
#define CVC_NAV_DEMO_NAV_COMMON_H

#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <vector>

namespace cvc {
class app;
}
class SceneRenderer; // cvcGL (global namespace)

namespace navdemo {

// World rect an occupancy grid maps onto (matches sim_world::config bounds).
struct Bounds {
  double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
  double cx() const { return 0.5 * (min_x + max_x); }
  double cy() const { return 0.5 * (min_y + max_y); }
  double radius() const {
    const double dx = max_x - min_x, dy = max_y - min_y;
    return 0.5 * std::sqrt(dx * dx + dy * dy);
  }
};

// Merged extruded-box mesh for every occupied cell (occ != 0), each box centred on
// its cell over the [min,max] world rect and spanning z in [0, height]. One
// geometry -> one GeometryNode (blocky "buildings"/walls). Colour = wall_rgb[3].
cvc::geometry occupancy_to_walls(const std::uint8_t *occ, int rows, int cols, const Bounds &b,
                                 double height, const double wall_rgb[3]);

// A single flat ground quad over the world rect at height z, colour rgb — the
// terrain the agents drive on. (Textured later for the fog belief map.)
cvc::geometry ground_quad(const Bounds &b, double z, const double rgb[3]);

// One merged flat-arrow glyph per agent (points +x at heading 0, length `size`,
// sitting at height `z`), per-vertex-coloured from sim_world's color[n*3]. Build the
// mesh ONCE, then pack() each frame from the sim snapshot and feed the result to
// GeometryNode::updateVertices — positions stream, colours/topology stay fixed.
class AgentGlyphs {
public:
  // Build the merged geometry for `n` agents. `color` is [n*3] in [0,1] (may be
  // null -> a default teal). Call once; N and the glyph are fixed for its life.
  cvc::geometry build(cvc::app &app, int n, const float *color, double size = 6.0, double z = 0.6);
  // Transform each glyph by (pos_world[i*2..], heading[i]) into a flat [x,y,z,...]
  // buffer sized 3 * n * verts_per_glyph — hand straight to updateVertices().
  const std::vector<double> &pack(const float *pos_world, const float *heading);

  int point_count() const { return n_ * kVerts; }

private:
  static constexpr int kVerts = 3; // triangle glyph
  int n_ = 0;
  double size_ = 6.0, z_ = 0.6;
  std::vector<double> xyz_; // scratch, reused each pack()
};

// Scripted capture camera: orbit an eye around the bbox centre (at world-z `zc`) at
// `azimuth` + `elevation` (radians), distance = bbox.radius() * dist_scale. Z-up.
// Writes eye[3] and focal[3] (the centre) for SceneRenderer::setCamera.
void orbit_camera(const Bounds &b, double zc, double azimuth, double elevation, double dist_scale,
                  double eye[3], double focal[3]);

// Configure the renderer's camera as a fixed TOP-DOWN ORTHOGRAPHIC map (+y up,
// looking straight down -z, parallel projection framing [min,max] + margin) — the
// 2-D "matplotlib" view. Call once; the camera stays put. Pairs with flat lighting
// (high ambient, shadows off) for a clean 2-D read.
void set_ortho_topdown(SceneRenderer &view, const Bounds &b, double margin = 6.0);

// Stamp a 1-cell occupied border around a rows*cols occupancy grid in place, so
// reactive agents can't drive off the open edge (and it renders as an outer wall).
void add_border(std::uint8_t *occ, int rows, int cols);

// Rasterize a mesh's XY footprint into a rows*cols (ny*nx) uint8 occupancy grid
// (occupied where any triangle projects onto the ground), over the [min,max] world
// rect — the C++ analog of GRL-SNAM's building_occupancy. CPU scan-conversion of
// every triangle projected to XY, then `inflate` passes of 4-connected dilation.
// Grid convention: r->y (row 0 = min_y, bottom-up), c->x; index [r*nx + c].
std::vector<std::uint8_t> occupancy_from_model(const cvc::geometry &mesh, const Bounds &b, int nx,
                                               int ny, int inflate = 0);

} // namespace navdemo

#endif // CVC_NAV_DEMO_NAV_COMMON_H
