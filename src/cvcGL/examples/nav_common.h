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
#include <cvc/image/image.h> // vehicle template's base-color texture
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
}
class SceneRenderer; // cvcGL (global namespace)
class SceneGraph;    // cvcGL (global namespace)
namespace cvc {
namespace gl {
class CameraController;
class StageLighting;
} // namespace gl
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

// Heightmap from a bundle's terrain.json: row-major elevation samples over the
// bundle's bounds. sample(x, y) bilinearly interpolates the elevation at any
// world (x, y), so vehicles + goal beacons + city walls can rest on the ground
// instead of at z = 0.
struct Terrain {
  int rows = 0, cols = 0;
  Bounds bounds;
  std::vector<double> grid; // rows*cols, elevation in world units (z metres)
  bool empty() const { return rows <= 0 || cols <= 0 || grid.empty(); }
  double sample(double x, double y) const;
};

// A tessellated ground mesh from a Terrain (rows*cols verts, 2*(rows-1)*(cols-1)
// triangles, flat-shaded). If the terrain is empty, falls back to ground_quad.
cvc::geometry terrain_mesh(const Terrain &t, const double rgb[3]);

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
                               const std::vector<std::uint32_t> &tris, double z = 0.0,
                               const std::vector<float> *uvs = nullptr);
  // Transform each instance by (pos_world[i*2..], heading[i]) into a flat [x,y,z,...]
  // buffer sized 3 * n * V — hand straight to updateVertices().
  const std::vector<double> &pack(const float *pos_world, const float *heading);
  // Same, but ALSO lift each instance's z by z_off[i] (e.g. terrain elevation at
  // that agent's world (x,y)), so a flat-ground template rests on real terrain.
  const std::vector<double> &pack_z(const float *pos_world, const float *heading,
                                    const double *z_off);

  // LOD pack: transform only the `count` agents named by `idx[0..count)` into the
  // first `count` slots (each at its own world pose from pos_world/heading/z_off,
  // indexed by the agent id in idx[]), and COLLAPSE every remaining slot to a
  // degenerate point so it rasterizes to nothing. `count` is clamped to this
  // glyph's capacity (n_). This is how the distance-LOD demo keeps a bounded
  // budget of expensive near-glyphs (a small-capacity Humvee node) separate from
  // the cheap far-glyphs (a large-capacity arrow node): each frame the selector
  // partitions agents by on-screen size and hands each node its own index list.
  // Point count is unchanged (n_ * v_), so updateVertices() still matches.
  const std::vector<double> &pack_lod(const float *pos_world, const float *heading,
                                      const double *z_off, const std::uint32_t *idx, int count);

  int capacity() const { return n_; }
  int verts_per_instance() const { return v_; }
  int point_count() const { return n_ * v_; }

private:
  cvc::geometry assemble(cvc::app &app, int n, const float *color);
  int n_ = 0, v_ = 0;                   // agents, verts per instance
  double z_ = 0.0;                      // height offset added to every local vert
  std::vector<double> tmpl_;            // [3*V] local template verts (forward +x)
  std::vector<std::uint32_t> tmplTris_; // [3*T] template triangle indices
  std::vector<float> tmplUvs_;          // [2*V] per-vert UVs (optional; empty = no texture)
  std::vector<double> xyz_;             // scratch, reused each pack()
};

// Build a StageLighting rig aimed at a nav scene's acting area (its occupancy
// bounds), instead of scene-spanning directional lights. This is the fix for the
// city demos' shadow mush: VTK bakes a DIRECTIONAL light's shadow map with a
// parallel projection fitted to the WHOLE scene bbox (ground + building height +
// empty air), so texels are wasted and thin casters (vehicles) alias onto tall
// receivers (rooftops). Aimed spots bake a perspective map that lands texels on
// the subject — see cvc/gl/StageLighting.h. `subjectHeight` is the tallest
// geometry (buildings); the stage is lifted to mid-height so the cones cover the
// rooftops, not just the street. The rig is a live cvc::state object, so
// cvc::gl::ui::StageLightingPanel(*rig, ...) drives it — the shared lighting UI.
std::unique_ptr<cvc::gl::StageLighting> make_stage_rig(SceneGraph &sg, const Bounds &b,
                                                       double subjectHeight = 0.0);

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

// Load a city bundle (a directory with terrain.json for the world bounds +
// buildings.glb for the geometry) into a nx*ny occupancy grid + bounds — the way
// nav_finale drives a REAL Austin scene, generalized so nav_city_swarm can too.
// Returns true on success (false = missing/empty bundle, caller falls back to the
// synthetic city). `mesh`, if non-null, receives the merged building mesh so the
// caller can render the true geometry instead of blocks extruded from occupancy.
// Grid convention matches occupancy_from_model / sim_world (r->y, row 0 = min_y).
// prefer_flat=true tries buildings_flat.glb (the low-poly extrusion) first and
// falls back to buildings.glb — 4x fewer tris, keeps VTK's shadow pass fast on
// laptop GPUs.
bool load_city_bundle(const std::string &dir, int nx, int ny, Bounds &bounds,
                      std::vector<std::uint8_t> &occ, cvc::geometry *mesh = nullptr,
                      Terrain *terrain = nullptr, bool prefer_flat = false);

// Load a mesh (.glb/.obj/...) as a canonical per-agent VEHICLE template for
// AgentGlyphs::build_template: forward -> +x, width -> +y, height -> +z, centred
// in XY, resting on z=0, scaled so the longest side is `target_len` metres. The
// height axis is taken as the smallest extent (robust to Y-up vs Z-up sources).
// Returns false if the mesh can't be read. Lets a demo render a real Humvee
// instead of a flat arrow glyph (nav_finale's loader, generalized).
bool load_vehicle_template(const std::string &path, double target_len, std::vector<double> &verts,
                           std::vector<std::uint32_t> &tris, std::vector<float> *uvs = nullptr,
                           cvc::image *texture = nullptr);

// One agent's global A* route over an occupancy grid: world waypoints + a cursor
// into them (`idx` is the waypoint a follower is currently steering toward).
struct Route {
  std::vector<std::array<double, 2>> wp;
  std::size_t idx = 0;
};

// Plan a string-pulled 8-connected A* route over `occ` (row-major rows*cols,
// nonzero = blocked) from world (sx,sy) to (gx,gy), returned as world waypoints
// that ALWAYS end at the true goal (so a blocked/unreachable plan still drives
// straight at it). `inflate_cells` > 0 dilates the occupancy first so the route
// keeps clearance from walls the reactive drive still hugs (too much clearance
// seals narrow gaps). This is the C++ analog of GRL-SNAM's global belief-route
// spine — the piece a purely reactive port drops; the finale and fog demos plan
// on their BELIEF occupancy so a believed wall bends the global plan, and a
// sense that clears it replans straight. grid convention: r->y (row 0 = min_y).
//
// `standoff_cells` > 0 uses cvc::nav::clearance_cost instead: a per-cell A*
// SURCHARGE that prices proximity to a wall rather than forbidding it. Prefer it
// to `inflate_cells`. Dilation is binary, which is why it "seals narrow gaps" --
// an alley thinner than the dilation radius simply stops existing and the route
// either detours absurdly or fails. The surcharge never forbids a cell, so a
// corridor tighter than `standoff_cells` throughout degrades to the shortest
// path instead of vanishing.
//
// It also turns the line-of-sight string-pull OFF, and that is the point rather
// than a side effect: simplification straightens the route back toward the walls
// and gives away exactly what the surcharge just bought. Measured on a block
// world, minimum clearance ALONG the path: 6.40 cells un-pulled, 1.00 pulled --
// and 1.00 is what the plain shortest path scores.
Route plan_route(const std::uint8_t *occ, int rows, int cols, const Bounds &b, double sx, double sy,
                 double gx, double gy, int inflate_cells = 0, double standoff_cells = 0.0,
                 double standoff_gamma = 1.5);

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
