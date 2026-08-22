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
namespace cvc {
namespace gl {
class CameraController;
}
} // namespace cvc

// Does this build have a BACKGROUND SIM WORKER?
//
// Natively, always. In the browser, only when compiled -pthread
// (-DCVC_WASM_PTHREADS=ON): a wasm worker shares memory through
// SharedArrayBuffer, which the browser only hands out on a cross-origin-isolated
// page (COOP: same-origin + COEP: require-corp). Without that the sim has to
// step inline on the render thread.
//
// Gate on THIS, never on __EMSCRIPTEN__ alone — that is the trap this replaces.
// `-pthread` sets __EMSCRIPTEN_PTHREADS__ but leaves __EMSCRIPTEN__ defined, so
// an `#ifndef __EMSCRIPTEN__` guard keeps the sim on the main thread even in a
// threaded build, and the whole point of the threaded build is lost silently.
#if !defined(__EMSCRIPTEN__) || defined(__EMSCRIPTEN_PTHREADS__)
#define CVC_NAV_DEMO_SIM_WORKER 1
#else
#define CVC_NAV_DEMO_SIM_WORKER 0
#endif

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
// vary > 0 gives each 4-connected BUILDING a hashed height in
// [1-vary, 1+vary] x height and a subtle tint (a city, not a maze); faces are
// flat-shaded and interior faces between occupied neighbours are culled.
cvc::geometry occupancy_to_walls(const std::uint8_t *occ, int rows, int cols, const Bounds &b,
                                 double height, const double wall_rgb[3], double vary = 0.0);

// A single flat ground quad over the world rect at height z, colour rgb — the
// terrain the agents drive on. (Textured later for the fog belief map.)
cvc::geometry ground_quad(const Bounds &b, double z, const double rgb[3]);

// One merged flat-arrow glyph per agent (points +x at heading 0, length `size`,
// sitting at height `z`), per-vertex-coloured from sim_world's color[n*3]. Build the
// mesh ONCE, then pack() each frame from the sim snapshot and feed the result to
// GeometryNode::updateVertices — positions stream, colours/topology stay fixed.
class AgentGlyphs {
public:
  // Instance a flat-arrow glyph per agent (default). `color` is [n*3] in [0,1] (may
  // be null -> a default teal). Call once; N and the mesh are fixed for its life.
  cvc::geometry build(cvc::app &app, int n, const float *color, double size = 6.0, double z = 0.6);
  // Instance an arbitrary TEMPLATE mesh per agent (e.g. a Humvee): `verts` is [3*V]
  // in a canonical local frame (forward +x, up +z, resting on z >= 0), `tris` is
  // [3*T] indices into V. Streams exactly like the arrow — one merged mesh, one
  // updateVertices per frame.
  cvc::geometry build_template(cvc::app &app, int n, const float *color,
                               const std::vector<double> &verts,
                               const std::vector<std::uint32_t> &tris, double z = 0.0);
  // Transform each instance by (pos_world[i*2..], heading[i]) into a flat [x,y,z,...]
  // buffer sized 3 * n * V — hand straight to updateVertices().
  const std::vector<double> &pack(const float *pos_world, const float *heading);

  int point_count() const { return n_ * v_; }

private:
  cvc::geometry assemble(cvc::app &app, int n, const float *color);
  int n_ = 0, v_ = 0;                   // agents, verts per instance
  double z_ = 0.0;                      // height offset added to every local vert
  std::vector<double> tmpl_;            // [3*V] local template verts (forward +x)
  std::vector<std::uint32_t> tmplTris_; // [3*T] template triangle indices
  std::vector<double> xyz_;             // scratch, reused each pack()
};

// A small pyramid marker: apex at the LOCAL origin pointing down, square base up
// at +h — position it hovering and it points at a spot on the ground (goal /
// pursuit-target markers; a silhouette deliberately distinct from every vehicle
// glyph, so a marker can't be mistaken for another agent). Per-vertex rgb.
cvc::geometry pyramid_marker(double half, double h, const double rgb[3]);

// A flat disc (triangle fan) of `radius` at height z — start pads, arrival pads.
cvc::geometry disc_marker(double radius, double z, const double rgb[3], int seg = 40);

// Wall-clock -> fixed-dt sim pacing. Accumulate real seconds * speed and hand
// back whole sim ticks to run this frame (capped so a hitch can't spiral).
// Keeps world time honest on ANY display rate — fixed steps-per-frame made the
// same story run twice as fast on a 60 Hz display as on a 30 fps capture.
struct SimPacer {
  double carry = 0.0;
  int ticks(double wall_dt, double sim_dt, double speed, int cap = 8) {
    if (!(wall_dt > 0.0) || !(sim_dt > 0.0))
      return 0;
    carry += wall_dt * speed;
    int n = static_cast<int>(carry / sim_dt);
    if (n > cap) {
      n = cap; // dropped time, not a tick burst — a stall must not fast-forward
      carry = 0.0;
    } else {
      carry -= n * sim_dt;
    }
    return n;
  }
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
// Pass the viewer's CameraController to make the map INTERACTIVE the 2-D way:
// drag pans, wheel zooms, and rotation is impossible (CameraController::Map).
// Without it the camera is simply parked (scripted captures).
void set_ortho_topdown(SceneRenderer &view, const Bounds &b, double margin = 6.0,
                       cvc::gl::CameraController *cam = nullptr);

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

// Bounds-safe RGB compositing on a raw interleaved [dw*dh*3] u8 destination. EVERY
// write is clipped to [0,dw) x [0,dh), so no input — an oversized source, an
// off-screen centre, a negative origin — can write out of range. These back the
// finale's 2-D picture-in-picture minimap; the clipping is what prevents the heap
// overflow (and crash) when the minimap is larger than the frame it is drawn onto.

// Paste `src` (interleaved [sw*sh*schan], schan 3 or 4) with its top-left at
// (x0, y0), multiplying each channel by `dim`.
void blit_clamped(unsigned char *dst, int dw, int dh, const unsigned char *src, int sw, int sh,
                  int schan, int x0, int y0, double dim = 1.0);

// Fill a filled disc of radius `rad` centred at (cx, cy) with colour (r, g, b).
void plot_disc(unsigned char *dst, int dw, int dh, int cx, int cy, int rad, unsigned char r,
               unsigned char g, unsigned char b);

// Draw a 1-px Bresenham line from (x0, y0) to (x1, y1); every pixel clipped —
// endpoints may be anywhere (routes to off-map targets just clip at the edge).
void plot_line(unsigned char *dst, int dw, int dh, int x0, int y0, int x1, int y1, unsigned char r,
               unsigned char g, unsigned char b);

} // namespace navdemo

#endif // CVC_NAV_DEMO_NAV_COMMON_H
