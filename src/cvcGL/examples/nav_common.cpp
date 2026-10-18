// nav_common.cpp — see nav_common.h.

#include "nav_common.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/model/model_file_io.h> // read_model (buildings.glb -> mesh)
#include <cvc/nav/grid_nav.h>        // astar / simplify / inflate — the shared route planner
#include <cvc/nav/land_cover.h>      // land cover as world surfaces
#include <cvc/nav/world_planes.h>    // the navigator's planes from the world
#include <cvc/world/city_bundle.h>   // the library's one reader of city bundles
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;

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
                                          const std::vector<std::uint32_t> &tris, double z,
                                          const std::vector<float> *uvs) {
  tmpl_ = verts;
  tmplTris_ = tris;
  z_ = z;
  tmplUvs_.clear();
  if (uvs && uvs->size() * 3 == verts.size() * 2) // 2 UVs per 3-coord vert
    tmplUvs_ = *uvs;
  return assemble(app, n, color);
}

cvc::geometry AgentGlyphs::assemble(cvc::app &app, int n, const float *color) {
  n_ = n;
  v_ = static_cast<int>(tmpl_.size() / 3);
  cvc::geometry g(app);
  auto &pts = g.points();
  auto &cols = g.colors();
  auto &tris = g.tris();
  const bool haveUv = tmplUvs_.size() == static_cast<std::size_t>(v_) * 2;
  pts.reserve(static_cast<std::size_t>(n) * v_);
  cols.reserve(static_cast<std::size_t>(n) * v_);
  if (haveUv)
    g.uvs().reserve(static_cast<std::size_t>(n) * v_);
  for (int i = 0; i < n; ++i) {
    const cvc::geometry::index_t base = static_cast<cvc::geometry::index_t>(i) * v_;
    for (int k = 0; k < v_; ++k)
      pts.push_back({tmpl_[3 * k], tmpl_[3 * k + 1], tmpl_[3 * k + 2] + z_});
    const double r = color ? color[3 * i + 0] : 0.20;
    const double gg = color ? color[3 * i + 1] : 0.80;
    const double bb = color ? color[3 * i + 2] : 0.75;
    for (int k = 0; k < v_; ++k)
      cols.push_back({r, gg, bb});
    if (haveUv)
      for (int k = 0; k < v_; ++k)
        g.uvs().push_back({tmplUvs_[2 * k], tmplUvs_[2 * k + 1]});
    for (std::size_t t = 0; t + 2 < tmplTris_.size(); t += 3)
      tris.push_back({base + tmplTris_[t], base + tmplTris_[t + 1], base + tmplTris_[t + 2]});
  }
  xyz_.assign(static_cast<std::size_t>(n) * v_ * 3, 0.0);
  return g;
}

const std::vector<double> &AgentGlyphs::pack(const float *pos_world, const float *heading) {
  return pack_z(pos_world, heading, nullptr);
}

const std::vector<double> &AgentGlyphs::pack_z(const float *pos_world, const float *heading,
                                               const double *z_off) {
  // Yaw each instance about +z by its heading and translate to its world pose.
  // If z_off is non-null, lift each instance's z by z_off[i] so a template
  // built at local z=0 rests on real terrain.
  for (int i = 0; i < n_; ++i) {
    const double ox = pos_world[2 * i], oy = pos_world[2 * i + 1];
    const double oz = z_off ? z_off[i] : 0.0;
    const double ch = std::cos(heading[i]), sh = std::sin(heading[i]);
    for (int k = 0; k < v_; ++k) {
      const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2];
      const std::size_t o = (static_cast<std::size_t>(i) * v_ + k) * 3;
      xyz_[o + 0] = ox + ch * lx - sh * ly;
      xyz_[o + 1] = oy + sh * lx + ch * ly;
      xyz_[o + 2] = oz + z_ + lz;
    }
  }
  return xyz_;
}

const std::vector<double> &AgentGlyphs::pack_basis(const float *pos_world, const double *z,
                                                   const double *basis) {
  // Full per-instance orientation: rotate the template by a 3x3 world basis (its
  // columns are the instance's world forward / side / up axes) then translate to
  // (pos_world, z). This is how a vehicle conforms to terrain — yaw from heading,
  // pitch+roll from the surface normal — instead of yaw-only (pack_z). `basis` is
  // [9*n]: {fx,fy,fz, sx,sy,sz, ux,uy,uz} per instance, each a unit world vector.
  for (int i = 0; i < n_; ++i) {
    const double ox = pos_world[2 * i], oy = pos_world[2 * i + 1], oz = z ? z[i] : 0.0;
    const double *R = basis + static_cast<std::size_t>(9) * i;
    for (int k = 0; k < v_; ++k) {
      const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2] + z_;
      const std::size_t o = (static_cast<std::size_t>(i) * v_ + k) * 3;
      xyz_[o + 0] = ox + R[0] * lx + R[3] * ly + R[6] * lz;
      xyz_[o + 1] = oy + R[1] * lx + R[4] * ly + R[7] * lz;
      xyz_[o + 2] = oz + R[2] * lx + R[5] * ly + R[8] * lz;
    }
  }
  return xyz_;
}

const std::vector<double> &AgentGlyphs::pack_lod(const float *pos_world, const float *heading,
                                                 const double *z_off, const std::uint32_t *idx,
                                                 int count) {
  if (count < 0)
    count = 0;
  if (count > n_)
    count = n_; // never exceed this node's fixed capacity

  // Used slots: transform the selected agent's template into slot s.
  for (int s = 0; s < count; ++s) {
    const int i = static_cast<int>(idx[s]);
    const double ox = pos_world[2 * i], oy = pos_world[2 * i + 1];
    const double oz = z_off ? z_off[i] : 0.0;
    const double ch = std::cos(heading[i]), sh = std::sin(heading[i]);
    for (int k = 0; k < v_; ++k) {
      const double lx = tmpl_[3 * k], ly = tmpl_[3 * k + 1], lz = tmpl_[3 * k + 2];
      const std::size_t o = (static_cast<std::size_t>(s) * v_ + k) * 3;
      xyz_[o + 0] = ox + ch * lx - sh * ly;
      xyz_[o + 1] = oy + sh * lx + ch * ly;
      xyz_[o + 2] = oz + z_ + lz;
    }
  }

  // Unused slots: collapse to a single degenerate point so every triangle has
  // zero area and is discarded by the rasterizer. Park it at the first used
  // agent's position (not the origin) so the merged actor's bounding box stays
  // tight -- a loose bbox makes the shadow map fit the wrong volume and the
  // frustum culler pessimistic (the fixed_mesh discipline from the LOD roadmap).
  double px = 0.0, py = 0.0, pz = 0.0;
  if (count > 0) {
    const int i0 = static_cast<int>(idx[0]);
    px = pos_world[2 * i0];
    py = pos_world[2 * i0 + 1];
    pz = (z_off ? z_off[i0] : 0.0) + z_;
  }
  for (int s = count; s < n_; ++s) {
    for (int k = 0; k < v_; ++k) {
      const std::size_t o = (static_cast<std::size_t>(s) * v_ + k) * 3;
      xyz_[o + 0] = px;
      xyz_[o + 1] = py;
      xyz_[o + 2] = pz;
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

bool load_vehicle_template(const std::string &path, double target_len, std::vector<double> &verts,
                           std::vector<std::uint32_t> &tris, std::vector<float> *out_uvs,
                           cvc::image *out_texture) {
  if (!std::filesystem::exists(path))
    return false;
  cvc::model m = cvc::read_model(path);
  cvc::geometry g = m.merged();
  if (g.num_tris() == 0)
    return false;
  // Grab the first material's base-color texture if the caller asked for one.
  if (out_texture) {
    *out_texture = cvc::image();
    for (const auto &mat : m.materials)
      if (mat.has_base_color_texture()) {
        *out_texture = mat.base_color_texture;
        break;
      }
  }
  const auto &pts = g.points();
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &v : pts)
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], v[k]);
      hi[k] = std::max(hi[k], v[k]);
    }
  const double ext[3] = {hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]};
  int up = 0; // height = smallest extent
  for (int k = 1; k < 3; ++k)
    if (ext[k] < ext[up])
      up = k;
  int fwd = (up + 1) % 3, side = (up + 2) % 3; // forward = longer of the remaining two
  if (ext[side] > ext[fwd])
    std::swap(fwd, side);
  const double s = ext[fwd] > 1e-9 ? target_len / ext[fwd] : 1.0;
  const double cf = 0.5 * (lo[fwd] + hi[fwd]), cs = 0.5 * (lo[side] + hi[side]);
  // Auto-orient front vs rear along the forward axis. For a passenger vehicle
  // (Humvee, sedan) the FRONT half usually has less vertex mass than the rear
  // (rear cargo/cabin dominates the mesh), while the front's TOP is lower
  // (hood < roof). Combine both signals: pick the direction that makes the
  // FRONT half both lighter (fewer verts) AND lower (mean-z-of-top ~ hood).
  // Falls back gracefully when signals disagree.
  int front_sign = +1; // assume +axis is the nose
  {
    long n_pos = 0, n_neg = 0;
    double sum_top_pos = 0.0, sum_top_neg = 0.0;
    long tt_pos = 0, tt_neg = 0;
    const double top_thresh = lo[up] + 0.6 * ext[up]; // upper 40% of height
    for (const auto &v : pts) {
      const double f = v[fwd] - cf;
      const bool pos = f > 0;
      if (pos)
        ++n_pos;
      else
        ++n_neg;
      if (v[up] >= top_thresh) {
        if (pos) {
          sum_top_pos += f;
          ++tt_pos;
        } else {
          sum_top_neg += f;
          ++tt_neg;
        }
      }
    }
    // Front-is-lighter score: which half has FEWER verts?
    const int lighter = (n_pos < n_neg) ? +1 : -1;
    // Front-is-lower-top score: which side's top-verts sit closer to axis center?
    // (Higher body/cabin mass on the rear side pulls its top centroid further out.)
    int lower_top = 0;
    if (tt_pos > 8 && tt_neg > 8) {
      const double meanTopPos = sum_top_pos / tt_pos;
      const double meanTopNeg = std::abs(sum_top_neg / tt_neg);
      lower_top = (meanTopPos < meanTopNeg) ? +1 : -1;
    }
    // Trust the hood-is-lower signal (the front's top sits lower than the rear
    // cabin/roof) — it is the physically robust cue for a passenger vehicle; fall
    // back to vertex-mass only when the top signal is unavailable/degenerate.
    // (The previous combine was a no-op that always used vertex-mass, which put a
    // detailed-front Humvee in backwards.)
    front_sign = (lower_top != 0) ? lower_top : lighter;
  }
  verts.clear();
  verts.reserve(pts.size() * 3);
  for (const auto &v : pts) {
    verts.push_back((v[fwd] - cf) * s * front_sign); // -> +x forward (autodetected)
    verts.push_back((v[side] - cs) * s);             // -> +y side
    verts.push_back((v[up] - lo[up]) * s);           // -> +z up, resting on 0
  }
  tris.clear();
  for (const auto &t : g.tris())
    for (int k = 0; k < 3; ++k)
      tris.push_back(static_cast<std::uint32_t>(t[k]));
  // Forward per-vertex UVs when the model has them AND the caller wants them.
  if (out_uvs) {
    out_uvs->clear();
    const auto &muvs = g.uvs();
    if (muvs.size() == pts.size()) {
      out_uvs->reserve(pts.size() * 2);
      for (const auto &uv : muvs) {
        out_uvs->push_back(static_cast<float>(uv[0]));
        out_uvs->push_back(static_cast<float>(uv[1]));
      }
    }
  }
  return true;
}

double Terrain::sample(double x, double y) const {
  if (!field)
    return 0.0;
  const double h = field->height(std::clamp(x, bounds.min_x, bounds.max_x),
                                 std::clamp(y, bounds.min_y, bounds.max_y));
  return std::isnan(h) ? 0.0 : h; // a hole in the terrain reads as sea level
}

cvc::geometry terrain_mesh(const Terrain &t, const double rgb[3]) {
  if (t.empty())
    return ground_quad(t.bounds, 0.0, rgb);
  // The world's own terrain mesh (shared arrays, plus a colour per vertex), with
  // planar UVs over the bounds as ground_quad has, so an aerial drapes the same way.
  cvc::geometry g = *t.field;
  g.colors().assign(g.num_points(), cvc::geometry::color_t{{rgb[0], rgb[1], rgb[2]}});
  const double su = t.bounds.max_x > t.bounds.min_x ? 1.0 / (t.bounds.max_x - t.bounds.min_x) : 0.0;
  const double sv = t.bounds.max_y > t.bounds.min_y ? 1.0 / (t.bounds.max_y - t.bounds.min_y) : 0.0;
  auto &uvs = g.uvs();
  uvs.clear();
  uvs.reserve(g.num_points());
  for (const auto &p : g.const_points())
    uvs.push_back({(p[0] - t.bounds.min_x) * su, (p[1] - t.bounds.min_y) * sv});
  return g;
}

bool load_city_bundle(const std::string &dir, int nx, int ny, Bounds &bounds,
                      std::vector<std::uint8_t> &occ, cvc::geometry *mesh, Terrain *terrain,
                      bool prefer_flat, cvc::world::world_geometry *world) {
  // Real Austin bundles ship a low-poly buildings_flat.glb (~6 MB, flat-shaded
  // extrusions from the footprint polygons) alongside the full detail
  // buildings.glb (~26 MB, per-vertex normals + smoothed corners). The full
  // mesh is ~500k tris on Austin south — enough to drop VTK's shadow-map pass
  // to 15-20 fps on a laptop GPU. Callers pass prefer_flat=true (--lite on the
  // demos) to load the lower-poly variant when present.
  cvc::world::city_bundle_options opt;
  opt.prefer_flat = prefer_flat;
  // Terrain cells carry the land cover of the bundle's masks or satellite image.
  opt.land_cover = cvc::nav::land_cover_provider(cvc::world::surface_registry::builtin());
  cvc::world::city_bundle city;
  try {
    city = cvc::world::load_city_bundle(dir, opt);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "  %s\n", e.what());
    return false;
  }
  bounds = {city.min_x, city.min_y, city.max_x, city.max_y};
  const auto field = city.geometry.terrain();
  if (terrain && field) {
    terrain->bounds = bounds;
    terrain->field = field;
  }
  if (world)
    *world = city.geometry;
  // buildings.glb -> occupancy + optional wall mesh. A missing or unreadable glb
  // (or a wasm assimp-side glTF version rejection) still returns true: the demo
  // renders real terrain + satellite with a synthetic occupancy fallback rather
  // than dropping the whole bundle.
  const cvc::world::geometry_item *b = city.geometry.find("buildings");
  if (b && b->geometry->num_tris() > 0) {
    // The navigator's occupancy from the same world the renderer draws: the
    // buildings' footprints (and any hard terrain surface) on the nx*ny grid.
    occ = cvc::nav::rasterize(city.geometry, ny, nx, bounds.min_x, bounds.min_y, bounds.max_x,
                              bounds.max_y)
              .occupancy;
    if (mesh)
      *mesh = *b->geometry;
    return true;
  }
  if (!city.buildings_error.empty())
    std::fprintf(stderr, "  buildings: %s (continuing without wall mesh)\n",
                 city.buildings_error.c_str());
  // No mesh -> mark occ empty; the caller falls back to city_scene occupancy.
  // The terrain is still returned to *terrain, so even without the buildings
  // mesh the demo gets the real Austin ground.
  occ.clear();
  return true;
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
  // Texture-unit budget on 16-unit iGPUs: every shadow-casting spot adds a
  // shadow-map sampler to the fragment stage. StageLighting's setBack is a
  // Kind::Spot (cone < 90) which VTK's LightCreatesShadow accepts, so it
  // baked a second shadow map on top of key; combined with satellite +
  // Humvee albedo + fog overlay + ImGui atlas + VTK's internal 2-D pass
  // textures, the shader link fails at runtime with "Hardware does not
  // support the number of textures defined". Kill back AND wash entirely on
  // the nav scenes — aerial city footprints don't need stage separation, and
  // key + fill is enough to see form.
  rig->setBack(0.0);
  rig->setWash(0.0, 0, 1.0);
  rig->apply();
  return rig;
}

Route plan_route(const std::uint8_t *occ, int rows, int cols, const Bounds &b, double sx, double sy,
                 double gx, double gy, int inflate_cells, double standoff_cells,
                 double standoff_gamma) {
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
  std::vector<double> surcharge;
  if (standoff_cells > 0.0)
    surcharge = cvc::nav::clearance_cost(planOcc, rows, cols, standoff_cells, standoff_gamma);
  const auto path = cvc::nav::astar(planOcc, rows, cols, sr, sc, gr, gc,
                                    surcharge.empty() ? nullptr : surcharge.data());
  if (path.size() >= 4) {
    // NO string-pull when a standoff surcharge is in play: line of sight cuts
    // every corner the surcharge paid to round, taking the route back to the
    // walls (measured 6.40 -> 1.00 cells of clearance, i.e. all the way back to
    // the shortest path). Keep the A* cells as the waypoints.
    std::vector<int> sp;
    if (surcharge.empty())
      sp = cvc::nav::simplify(planOcc, rows, cols, path.data(), static_cast<int>(path.size() / 2));
    else
      sp = path;
    for (std::size_t i = 0; i + 1 < sp.size(); i += 2)
      rt.wp.push_back({b.min_x + static_cast<double>(sp[i + 1]) / (cols - 1) * (b.max_x - b.min_x),
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

} // namespace navdemo
