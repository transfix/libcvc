// nav_common.cpp — see nav_common.h.

#include "nav_common.h"

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/nav/grid_nav.h> // astar / simplify / inflate — the shared route planner
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

namespace navdemo {

namespace {
// Push one axis-aligned box [x0,x1]x[y0,y1]x[z0,z1] as 8 shared verts + 12 tris into
// an existing geometry, colouring every vertex rgb. Returns nothing; grows in place.
void push_box(cvc::geometry &g, double x0, double y0, double z0, double x1, double y1, double z1,
              const double rgb[3]) {
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  const cvc::geometry::index_t base = static_cast<cvc::geometry::index_t>(pts.size());
  const double corner[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                               {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
  for (int k = 0; k < 8; ++k) {
    pts.push_back({corner[k][0], corner[k][1], corner[k][2]});
    cols.push_back({rgb[0], rgb[1], rgb[2]});
  }
  // 12 tris (outward winding): bottom, top, and 4 sides.
  const int face[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                           {1, 2, 6}, {1, 6, 5}, {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}};
  for (auto &f : face)
    tris.push_back({base + f[0], base + f[1], base + f[2]});
}
} // namespace

cvc::geometry occupancy_to_walls(const std::uint8_t *occ, int rows, int cols, const Bounds &b,
                                 double height, const double wall_rgb[3], double vary) {
  cvc::geometry g;
  if (rows < 2 || cols < 2)
    return g;
  // Cell centres span [min,max] exactly (matches sim_world's cell_to_on mapping);
  // each box is one cell wide/tall so occupied neighbours touch into solid walls.
  const double dx = (b.max_x - b.min_x) / (cols - 1);
  const double dy = (b.max_y - b.min_y) / (rows - 1);
  const auto at = [&](int r, int c) -> bool {
    return r >= 0 && c >= 0 && r < rows && c < cols &&
           occ[static_cast<std::size_t>(r) * cols + c] != 0;
  };

  // With vary > 0, label 4-connected components so each BUILDING (not cell) gets
  // one hashed height/tint — adjacent cells always share a component, so culled
  // interior faces can never open a gap between different heights.
  std::vector<std::int32_t> comp(static_cast<std::size_t>(rows) * cols, -1);
  std::vector<std::uint32_t> seed; // component -> its seed cell (hash key)
  if (vary > 0.0) {
    std::vector<std::uint32_t> stack;
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c) {
        const std::size_t i = static_cast<std::size_t>(r) * cols + c;
        if (!occ[i] || comp[i] >= 0)
          continue;
        const std::int32_t id = static_cast<std::int32_t>(seed.size());
        seed.push_back(static_cast<std::uint32_t>(i));
        comp[i] = id;
        stack.assign(1, static_cast<std::uint32_t>(i));
        while (!stack.empty()) {
          const std::uint32_t j = stack.back();
          stack.pop_back();
          const int jr = static_cast<int>(j) / cols, jc = static_cast<int>(j) % cols;
          const int nb[4][2] = {{jr - 1, jc}, {jr + 1, jc}, {jr, jc - 1}, {jr, jc + 1}};
          for (auto &n : nb) {
            if (!at(n[0], n[1]))
              continue;
            const std::size_t k = static_cast<std::size_t>(n[0]) * cols + n[1];
            if (comp[k] < 0) {
              comp[k] = id;
              stack.push_back(static_cast<std::uint32_t>(k));
            }
          }
        }
      }
  }
  const auto hash01 = [](std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return (x & 0xffffffU) / static_cast<double>(0x1000000U);
  };

  // Emit UNSHARED verts per face (flat per-face normals from ensureNormals — the
  // crisp-edged city look, not corner-averaged mush), and only EXPOSED faces:
  // tops always, sides only against free space (interior faces between occupied
  // neighbours are invisible and z-fight), bottoms never.
  auto face_quad = [&](const double p0[3], const double p1[3], const double p2[3],
                       const double p3[3], const double rgb[3]) {
    auto &pts = g.points();
    auto &cols_ = g.colors();
    const cvc::geometry::index_t base = static_cast<cvc::geometry::index_t>(pts.size());
    for (const double *p : {p0, p1, p2, p3}) {
      pts.push_back({p[0], p[1], p[2]});
      cols_.push_back({rgb[0], rgb[1], rgb[2]});
    }
    g.tris().push_back({base, static_cast<cvc::geometry::index_t>(base + 1),
                        static_cast<cvc::geometry::index_t>(base + 2)});
    g.tris().push_back({base, static_cast<cvc::geometry::index_t>(base + 2),
                        static_cast<cvc::geometry::index_t>(base + 3)});
  };

  for (int r = 0; r < rows; ++r) {
    const double yc = b.min_y + static_cast<double>(r) / (rows - 1) * (b.max_y - b.min_y);
    for (int c = 0; c < cols; ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * cols + c;
      if (!occ[i])
        continue;
      const double xc = b.min_x + static_cast<double>(c) / (cols - 1) * (b.max_x - b.min_x);
      const double x0 = xc - 0.5 * dx, x1 = xc + 0.5 * dx;
      const double y0 = yc - 0.5 * dy, y1 = yc + 0.5 * dy;
      double h = height;
      double rgb[3] = {wall_rgb[0], wall_rgb[1], wall_rgb[2]};
      if (vary > 0.0 && comp[i] >= 0) {
        const double u = hash01(seed[comp[i]]);
        h = height * (1.0 - vary + 2.0 * vary * u); // per-building height
        const double tint = 0.9 + 0.2 * hash01(seed[comp[i]] * 2654435761U);
        for (double &v : rgb)
          v = std::min(1.0, v * tint); // subtle per-building tint
      }
      const double t0[3] = {x0, y0, h}, t1[3] = {x1, y0, h}, t2[3] = {x1, y1, h},
                   t3[3] = {x0, y1, h};
      face_quad(t0, t1, t2, t3, rgb); // top (+z)
      if (!at(r, c - 1)) {            // west (-x)
        const double p0[3] = {x0, y0, 0}, p1[3] = {x0, y0, h}, p2[3] = {x0, y1, h},
                     p3[3] = {x0, y1, 0};
        face_quad(p0, p1, p2, p3, rgb);
      }
      if (!at(r, c + 1)) { // east (+x)
        const double p0[3] = {x1, y0, 0}, p1[3] = {x1, y1, 0}, p2[3] = {x1, y1, h},
                     p3[3] = {x1, y0, h};
        face_quad(p0, p1, p2, p3, rgb);
      }
      if (!at(r - 1, c)) { // south (-y)
        const double p0[3] = {x0, y0, 0}, p1[3] = {x1, y0, 0}, p2[3] = {x1, y0, h},
                     p3[3] = {x0, y0, h};
        face_quad(p0, p1, p2, p3, rgb);
      }
      if (!at(r + 1, c)) { // north (+y)
        const double p0[3] = {x0, y1, 0}, p1[3] = {x0, y1, h}, p2[3] = {x1, y1, h},
                     p3[3] = {x1, y1, 0};
        face_quad(p0, p1, p2, p3, rgb);
      }
    }
  }
  return g;
}

cvc::geometry ground_quad(const Bounds &b, double z, const double rgb[3]) {
  cvc::geometry g;
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &uvs = g.uvs();
  auto &tris = g.tris();
  const double corner[4][2] = {
      {b.min_x, b.min_y}, {b.max_x, b.min_y}, {b.max_x, b.max_y}, {b.min_x, b.max_y}};
  const double uv[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  for (int k = 0; k < 4; ++k) {
    pts.push_back({corner[k][0], corner[k][1], z});
    cols.push_back({rgb[0], rgb[1], rgb[2]});
    uvs.push_back({uv[k][0], uv[k][1]});
  }
  tris.push_back({0, 1, 2});
  tris.push_back({0, 2, 3});
  return g;
}

cvc::geometry AgentGlyphs::build(cvc::app &app, int n, const float *color, double size, double z) {
  // Flat arrow pointing +x at heading 0 (tip forward, two swept-back tail corners),
  // in a local z=0 frame (z_ adds the height). A REAL triangle so ensureNormals gets
  // valid normals — updateVertices keeps normals at bind pose.
  const double tip = size, back = -0.45 * size, half = 0.42 * size;
  tmpl_ = {tip, 0.0, 0.0, back, half, 0.0, back, -half, 0.0};
  tmplTris_ = {0, 1, 2};
  z_ = z;
  return assemble(app, n, color);
}

cvc::geometry AgentGlyphs::build_template(cvc::app &app, int n, const float *color,
                                          const std::vector<double> &verts,
                                          const std::vector<std::uint32_t> &tris, double z) {
  tmpl_ = verts;
  tmplTris_ = tris;
  z_ = z;
  return assemble(app, n, color);
}

cvc::geometry AgentGlyphs::assemble(cvc::app &app, int n, const float *color) {
  n_ = n;
  v_ = static_cast<int>(tmpl_.size() / 3);
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  pts.reserve(static_cast<std::size_t>(n) * v_);
  cols.reserve(static_cast<std::size_t>(n) * v_);
  for (int i = 0; i < n; ++i) {
    const cvc::geometry::index_t base = static_cast<cvc::geometry::index_t>(i) * v_;
    for (int k = 0; k < v_; ++k)
      pts.push_back({tmpl_[3 * k], tmpl_[3 * k + 1], tmpl_[3 * k + 2] + z_});
    const double r = color ? color[3 * i + 0] : 0.20;
    const double gg = color ? color[3 * i + 1] : 0.80;
    const double bb = color ? color[3 * i + 2] : 0.75;
    for (int k = 0; k < v_; ++k)
      cols.push_back({r, gg, bb});
    for (std::size_t t = 0; t + 2 < tmplTris_.size(); t += 3)
      tris.push_back({base + tmplTris_[t], base + tmplTris_[t + 1], base + tmplTris_[t + 2]});
  }
  xyz_.assign(static_cast<std::size_t>(n) * v_ * 3, 0.0);
  return g;
}

const std::vector<double> &AgentGlyphs::pack(const float *pos_world, const float *heading) {
  // Yaw each instance about +z by its heading and translate to its world pose.
  for (int i = 0; i < n_; ++i) {
    const double ox = pos_world[2 * i], oy = pos_world[2 * i + 1];
    const double ch = std::cos(heading[i]), sh = std::sin(heading[i]);
    for (int k = 0; k < v_; ++k) {
      const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2];
      const std::size_t o = (static_cast<std::size_t>(i) * v_ + k) * 3;
      xyz_[o + 0] = ox + ch * lx - sh * ly;
      xyz_[o + 1] = oy + sh * lx + ch * ly;
      xyz_[o + 2] = z_ + lz;
    }
  }
  return xyz_;
}

void orbit_camera(const Bounds &b, double zc, double azimuth, double elevation, double dist_scale,
                  double eye[3], double focal[3]) {
  focal[0] = b.cx();
  focal[1] = b.cy();
  focal[2] = zc;
  const double dist = std::max(1.0, b.radius() * dist_scale);
  eye[0] = focal[0] + dist * std::cos(elevation) * std::cos(azimuth);
  eye[1] = focal[1] + dist * std::cos(elevation) * std::sin(azimuth);
  eye[2] = focal[2] + dist * std::sin(elevation);
}

void set_ortho_topdown(SceneRenderer &view, const Bounds &b, double margin,
                       cvc::gl::CameraController *cam) {
  const double halfH = 0.5 * (b.max_y - b.min_y) + margin;
  const double halfW = 0.5 * (b.max_x - b.min_x) + margin;
  if (cam) {
    // Interactive 2-D map: pan + zoom only, no rotation. Passing the half-width
    // too fits the WHOLE map in a narrow viewport (a phone in portrait) instead
    // of cropping its sides, and keeps it fitted across rotation/fullscreen.
    cam->frameMap(b.cx(), b.cy(), halfH, halfW);
    return;
  }
  // Eye straight above the centre, looking down -z, +y up.
  view.setCamera(b.cx(), b.cy(), 800.0, b.cx(), b.cy(), 0.0, 0.0, 1.0, 0.0, 30.0, 1.0, 4000.0);
  vtkRenderer *r = view.renderer();
  if (!r)
    return;
  vtkCamera *vcam = r->GetActiveCamera();
  vcam->ParallelProjectionOn();
  // Same width fit on the offscreen capture path (fixed frame, no controller).
  // Ask the WINDOW, not the renderer: a renderer reports its size only once it
  // has rendered, so at setup time it would answer with a stale default and the
  // fit would silently fall back to a height-only crop.
  const int *sz = view.renderWindow() ? view.renderWindow()->GetSize() : nullptr;
  const double aspect = (sz && sz[0] > 0 && sz[1] > 0) ? static_cast<double>(sz[0]) / sz[1] : 1.0;
  vcam->SetParallelScale(std::max(halfH, halfW / std::max(1e-6, aspect)));
}

void add_border(std::uint8_t *occ, int rows, int cols) {
  for (int c = 0; c < cols; ++c) {
    occ[c] = 1;
    occ[static_cast<std::size_t>(rows - 1) * cols + c] = 1;
  }
  for (int r = 0; r < rows; ++r) {
    occ[static_cast<std::size_t>(r) * cols] = 1;
    occ[static_cast<std::size_t>(r) * cols + (cols - 1)] = 1;
  }
}

static std::vector<std::uint8_t> occ_dilated(std::vector<std::uint8_t> occ, int nx, int ny,
                                             int inflate); // defined below

std::vector<std::uint8_t> occupancy_from_model(const cvc::geometry &mesh, const Bounds &b, int nx,
                                               int ny, int inflate) {
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(nx) * ny, 0);
  if (nx < 2 || ny < 2 || b.max_x <= b.min_x || b.max_y <= b.min_y)
    return occ;
  const auto &pts = mesh.points();
  const auto &tris = mesh.tris();
  // world -> continuous cell coords (cell centres span [min,max]).
  const double sx = (nx - 1) / (b.max_x - b.min_x), sy = (ny - 1) / (b.max_y - b.min_y);
  for (const auto &tri : tris) {
    double px[3], py[3];
    for (int k = 0; k < 3; ++k) {
      const auto &p = pts[tri[k]];
      px[k] = (p[0] - b.min_x) * sx; // -> column
      py[k] = (p[1] - b.min_y) * sy; // -> row
    }
    int c0 = static_cast<int>(std::floor(std::min({px[0], px[1], px[2]})));
    int c1 = static_cast<int>(std::ceil(std::max({px[0], px[1], px[2]})));
    int r0 = static_cast<int>(std::floor(std::min({py[0], py[1], py[2]})));
    int r1 = static_cast<int>(std::ceil(std::max({py[0], py[1], py[2]})));
    c0 = std::max(0, c0);
    r0 = std::max(0, r0);
    c1 = std::min(nx - 1, c1);
    r1 = std::min(ny - 1, r1);
    // Edge functions; fill any cell centre inside the projected triangle (either
    // winding), so a solid roof/wall soup rasterizes to a filled footprint.
    const double d = (py[1] - py[2]) * (px[0] - px[2]) + (px[2] - px[1]) * (py[0] - py[2]);
    if (std::fabs(d) < 1e-12)
      continue;
    const double inv = 1.0 / d;
    // Mark a node when its cell centre is inside the triangle. A tight test (not a
    // conservative fatten) keeps the street gaps open — an over-filled occupancy seals
    // dense downtown so A* can't route and the reactive drive is repelled everywhere.
    for (int r = r0; r <= r1; ++r)
      for (int c = c0; c <= c1; ++c) {
        const double x = c, y = r;
        const double a = ((py[1] - py[2]) * (x - px[2]) + (px[2] - px[1]) * (y - py[2])) * inv;
        const double be = ((py[2] - py[0]) * (x - px[2]) + (px[0] - px[2]) * (y - py[2])) * inv;
        if (a >= 0.0 && be >= 0.0 && 1.0 - a - be >= 0.0)
          occ[static_cast<std::size_t>(r) * nx + c] = 1;
      }
  }
  return occ_dilated(occ, nx, ny, inflate);
}

std::unique_ptr<cvc::gl::StageLighting> make_stage_rig(SceneGraph &sg, const Bounds &b,
                                                      double subjectHeight) {
  auto rig = std::make_unique<cvc::gl::StageLighting>(sg);
  // Acting area = the city footprint, lifted to mid-building height so the cones
  // (and thus each light's shadow-map frustum) cover the rooftops, not only the
  // street. radius = footprint half-diagonal — the subject IS the whole city.
  rig->setStage(b.cx(), b.cy(), 0.5 * subjectHeight, b.radius());
  rig->applyPreset(cvc::gl::StageLighting::Preset::ThreePoint);
  // Keep the old warm-key / cool-fill feel (the two directional lights this
  // replaces were a warm key from the SW-high and a cool fill from the NE).
  rig->setKey(1.15, -42.0, 58.0, 62.0); // intensity, azimuth, elevation, cone(deg)
  rig->setFill(0.55);
  rig->setWarmth(0.35);
  rig->setEnvironment(0.5); // lift open ground / map edges the cones don't reach
  rig->setAmbient(0.32);
  rig->apply();
  return rig;
}

Route plan_route(const std::uint8_t *occ, int rows, int cols, const Bounds &b, double sx, double sy,
                 double gx, double gy, int inflate_cells) {
  Route rt;
  if (!occ || rows < 2 || cols < 2 || b.max_x <= b.min_x || b.max_y <= b.min_y) {
    rt.wp.push_back({gx, gy});
    return rt;
  }
  // Optionally dilate so the GLOBAL route keeps clearance from walls the local
  // reactive barrier still hugs; the raw occupancy is what the SDF is built from.
  std::vector<std::uint8_t> dil;
  const std::uint8_t *planOcc = occ;
  if (inflate_cells > 0) {
    dil = cvc::nav::inflate(occ, rows, cols, inflate_cells);
    planOcc = dil.data();
  }
  auto w2c = [&](double x, double y, int &r, int &c) {
    c = static_cast<int>(std::lround((x - b.min_x) / (b.max_x - b.min_x) * (cols - 1)));
    r = static_cast<int>(std::lround((y - b.min_y) / (b.max_y - b.min_y) * (rows - 1)));
    c = std::max(0, std::min(cols - 1, c));
    r = std::max(0, std::min(rows - 1, r));
  };
  int sr, sc, gr, gc;
  w2c(sx, sy, sr, sc);
  w2c(gx, gy, gr, gc);
  const auto path = cvc::nav::astar(planOcc, rows, cols, sr, sc, gr, gc);
  if (path.size() >= 4) {
    const auto sp = cvc::nav::simplify(planOcc, rows, cols, path.data(),
                                       static_cast<int>(path.size() / 2));
    for (std::size_t i = 0; i + 1 < sp.size(); i += 2)
      rt.wp.push_back(
          {b.min_x + static_cast<double>(sp[i + 1]) / (cols - 1) * (b.max_x - b.min_x),
           b.min_y + static_cast<double>(sp[i]) / (rows - 1) * (b.max_y - b.min_y)});
  }
  rt.wp.push_back({gx, gy}); // always end at the true goal
  return rt;
}

void blit_clamped(unsigned char *dst, int dw, int dh, const unsigned char *src, int sw, int sh,
                  int schan, int x0, int y0, double dim) {
  if (!dst || !src || dw <= 0 || dh <= 0 || sw <= 0 || sh <= 0 || schan < 3)
    return;
  for (int sr = 0; sr < sh; ++sr) {
    const int py = y0 + sr;
    if (py < 0 || py >= dh)
      continue;
    for (int sc = 0; sc < sw; ++sc) {
      const int px = x0 + sc;
      if (px < 0 || px >= dw)
        continue;
      const long s = (static_cast<long>(sr) * sw + sc) * schan;
      const long d = (static_cast<long>(py) * dw + px) * 3;
      dst[d] = static_cast<unsigned char>(dim * src[s]);
      dst[d + 1] = static_cast<unsigned char>(dim * src[s + 1]);
      dst[d + 2] = static_cast<unsigned char>(dim * src[s + 2]);
    }
  }
}

cvc::geometry pyramid_marker(double half, double h, const double rgb[3]) {
  cvc::geometry g;
  auto add = [&](double x, double y, double z) {
    g.points().push_back({x, y, z});
    g.colors().push_back({rgb[0], rgb[1], rgb[2]});
  };
  add(0, 0, 0); // apex, pointing down at the spot it marks
  add(-half, -half, h);
  add(half, -half, h);
  add(half, half, h);
  add(-half, half, h);
  using I = cvc::geometry::index_t;
  auto tri = [&](int a, int b, int c) {
    g.tris().push_back({static_cast<I>(a), static_cast<I>(b), static_cast<I>(c)});
  };
  tri(0, 1, 2);
  tri(0, 2, 3);
  tri(0, 3, 4);
  tri(0, 4, 1);
  tri(1, 3, 2); // base cap
  tri(1, 4, 3);
  return g;
}

cvc::geometry disc_marker(double radius, double z, const double rgb[3], int seg) {
  cvc::geometry g;
  auto add = [&](double x, double y) {
    g.points().push_back({x, y, z});
    g.colors().push_back({rgb[0], rgb[1], rgb[2]});
  };
  add(0, 0); // centre
  for (int i = 0; i < seg; ++i) {
    const double a = 2.0 * 3.14159265358979323846 * i / seg;
    add(radius * std::cos(a), radius * std::sin(a));
  }
  using I = cvc::geometry::index_t;
  for (int i = 0; i < seg; ++i)
    g.tris().push_back(
        {static_cast<I>(0), static_cast<I>(1 + i), static_cast<I>(1 + (i + 1) % seg)});
  return g;
}

void plot_disc(unsigned char *dst, int dw, int dh, int cx, int cy, int rad, unsigned char r,
               unsigned char g, unsigned char b) {
  if (!dst || dw <= 0 || dh <= 0)
    return;
  for (int dy = -rad; dy <= rad; ++dy)
    for (int dx = -rad; dx <= rad; ++dx) {
      if (dx * dx + dy * dy > rad * rad)
        continue; // an actual disc, not its bounding square
      const int px = cx + dx, py = cy + dy;
      if (px < 0 || py < 0 || px >= dw || py >= dh)
        continue;
      const long d = (static_cast<long>(py) * dw + px) * 3;
      dst[d] = r;
      dst[d + 1] = g;
      dst[d + 2] = b;
    }
}

void plot_line(unsigned char *dst, int dw, int dh, int x0, int y0, int x1, int y1, unsigned char r,
               unsigned char g, unsigned char b) {
  if (!dst || dw <= 0 || dh <= 0)
    return;
  // Classic integer Bresenham; every pixel individually clipped so arbitrary
  // endpoints (including far off-screen) are safe.
  const int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  // Cap the walk so absurd endpoints can't spin for millions of steps.
  long steps = static_cast<long>(dx) - dy + 1;
  if (steps > 100000)
    return;
  int err = dx + dy;
  int x = x0, y = y0;
  for (;;) {
    if (x >= 0 && y >= 0 && x < dw && y < dh) {
      const long d = (static_cast<long>(y) * dw + x) * 3;
      dst[d] = r;
      dst[d + 1] = g;
      dst[d + 2] = b;
    }
    if (x == x1 && y == y1)
      break;
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
}

// 4-connected dilation of a rows*cols (ny*nx) occupancy `it` times.
static std::vector<std::uint8_t> occ_dilated(std::vector<std::uint8_t> occ, int nx, int ny,
                                             int inflate) {
  for (int it = 0; it < inflate; ++it) {
    std::vector<std::uint8_t> next = occ;
    for (int r = 0; r < ny; ++r)
      for (int c = 0; c < nx; ++c)
        if (occ[static_cast<std::size_t>(r) * nx + c]) {
          if (r > 0)
            next[static_cast<std::size_t>(r - 1) * nx + c] = 1;
          if (r < ny - 1)
            next[static_cast<std::size_t>(r + 1) * nx + c] = 1;
          if (c > 0)
            next[static_cast<std::size_t>(r) * nx + (c - 1)] = 1;
          if (c < nx - 1)
            next[static_cast<std::size_t>(r) * nx + (c + 1)] = 1;
        }
    occ.swap(next);
  }
  return occ;
}

} // namespace navdemo
