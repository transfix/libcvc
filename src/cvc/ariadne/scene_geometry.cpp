/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// A scene node's geometry from data alone (scene_geometry.h). Moved from cvcGL's scene realizer so
// a headless host builds the same bodies a window draws.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/scene_geometry.h>
#include <cvc/ariadne/uri.h> // a node's source: URI to a local path (temp-file bridge)
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/height_field.h>
#include <cvc/world/generated_world.h> // source.heightfield.generator: a cvc::world terrain
#include <exception>
#include <stdexcept>

namespace cvc {
namespace ariadne {

namespace {

void warn(std::vector<std::string> *w, const std::string &m) {
  if (w)
    w->push_back(m);
}

// §13.4: resolve a node's source: URI to a local path a reader (read_geometry / cvc::volume) can
// open — file:// in place, any other scheme spilled to an owned temp file. On failure, warn with
// the node id and return an unset ResolvedFile (ok=false); the caller returns early. The returned
// object must stay alive for the whole read (it owns any temp file).
ResolvedFile resolve_source(cvc::app &app, const SceneNode &n, std::vector<std::string> *warnings) {
  ResolvedFile rf = resolve_to_file(app, n.source_file);
  if (!rf.ok)
    warn(warnings, "ari: scene node '" + n.id + "': " + rf.error);
  return rf;
}

} // namespace

// A flat quad with UP normals spanning [-size/2, +size/2]² at z=0 — the built-in `plane` primitive
// (source: { plane: { size } }): a ground / shadow receiver that needs no asset. Colour comes from
// the node's material (single-colour), so no per-vertex colours are baked.
cvc::geometry make_plane_geometry(float size) {
  cvc::geometry g;
  const double h = 0.5 * static_cast<double>(size);
  const double v[4][3] = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0}};
  for (const auto &p : v) {
    g.points().push_back({p[0], p[1], p[2]});
    g.normals().push_back({0, 0, 1});
  }
  g.tris().push_back({0, 1, 2});
  g.tris().push_back({0, 2, 3});
  return g;
}

// A box centred on the origin with edge lengths `size` (source: { box: { size } }): 24 vertices so
// each face keeps its own outward normal, 12 triangles wound counter-clockwise seen from outside.
cvc::geometry make_box_geometry(const float size[3]) {
  cvc::geometry g;
  const double h[3] = {0.5 * size[0], 0.5 * size[1], 0.5 * size[2]};
  // Per face: the outward axis, its sign, and two in-face axes u, v with u x v outward, so the
  // corners (-,-), (+,-), (+,+), (-,+) in (u, v) run counter-clockwise seen from outside.
  const int faces[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 2, 0}, {1, 0, 2}, {2, 0, 1}, {2, 1, 0}};
  const double corners[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int f = 0; f < 6; ++f) {
    const int a = faces[f][0], u = faces[f][1], v = faces[f][2];
    const double sign = (f % 2 == 0) ? 1.0 : -1.0;
    const auto base = static_cast<unsigned>(g.points().size());
    for (const auto &c : corners) {
      double p[3], n[3] = {0, 0, 0};
      p[a] = sign * h[a];
      p[u] = c[0] * h[u];
      p[v] = c[1] * h[v];
      n[a] = sign;
      g.points().push_back({p[0], p[1], p[2]});
      g.normals().push_back({n[0], n[1], n[2]});
    }
    g.tris().push_back({base, base + 1, base + 2});
    g.tris().push_back({base, base + 2, base + 3});
  }
  return g;
}

// A UV sphere of `radius` about the origin (source: { sphere: { radius } }), outward normals,
// counter-clockwise seen from outside, no degenerate triangles at the poles.
cvc::geometry make_sphere_geometry(float radius) {
  constexpr double kPi = 3.141592653589793;
  constexpr unsigned kRings = 16, kSegments = 24;
  cvc::geometry g;
  const double r = static_cast<double>(radius);
  for (unsigned i = 0; i <= kRings; ++i) {
    const double th = kPi * i / kRings; // from the north pole
    for (unsigned j = 0; j <= kSegments; ++j) {
      const double ph = 2.0 * kPi * j / kSegments;
      const double n[3] = {std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)};
      g.points().push_back({r * n[0], r * n[1], r * n[2]});
      g.normals().push_back({n[0], n[1], n[2]});
    }
  }
  const unsigned row = kSegments + 1;
  for (unsigned i = 0; i < kRings; ++i)
    for (unsigned j = 0; j < kSegments; ++j) {
      const unsigned a = i * row + j, b = a + row; // b: the same longitude, one ring south
      if (i != 0)
        g.tris().push_back({a, b, a + 1});
      if (i + 1 != kRings)
        g.tris().push_back({a + 1, b, b + 1});
    }
  return g;
}

// A procedural heightfield (source: { heightfield: {...} }), the reusable terrain primitive, as
// the world's own cvc::height_field: a res x res grid over [-size/2, size/2]^2 in XY (row 0 at -y),
// displaced in Z by the sum of the dome/wave layers. The field's mesh is what the node draws, so
// the renderer, a body resting on it and a forest planted on it see one surface: each cell split
// south-west to north-east, normals from the grid. World->[0,1]^2 UVs and optional per-vertex
// band colours ride on the mesh. All CPU, no asset. With a generator the ground starts as that
// cvc::world terrain (`dem_path`: its resolved .npy) and the layers add on top. Throws
// cvc::geometry_error for a size that is not > 0, std::invalid_argument for an unknown generator.
std::shared_ptr<cvc::height_field> make_height_field(const SceneHeightfield &hf,
                                                     const std::string &dem_path) {
  constexpr double kTwoPi = 6.283185307179586;
  const int res = std::max(2, hf.resolution);
  const double half = 0.5 * static_cast<double>(hf.size);
  const double step = static_cast<double>(hf.size) / (res - 1);
  std::shared_ptr<const cvc::world::terrain_source> ground;
  if (!hf.generator.empty()) {
    cvc::world::generated_world_params p;
    p.half_m = half;
    p.seed = static_cast<std::uint64_t>(std::max(0.0, hf.seed));
    p.amp_m = hf.amp;
    p.water_level_m = hf.water_level;
    p.preset = cvc::world::world_preset_from_name(hf.preset);
    p.terrain = hf.generator;
    p.terrain_file = dem_path;
    ground = cvc::world::make_terrain(p);
  }

  const auto height_at = [&](double x, double y) {
    double h = ground ? ground->sample(x, y) : 0.0;
    for (const SceneHeightLayer &L : hf.layers) {
      if (L.kind == "dome") {
        const double r = std::sqrt(x * x + y * y);
        const double rr = (L.radius > 0.0f) ? (r / L.radius) : 0.0;
        h += L.amplitude * std::exp(-rr * rr);
      } else if (L.kind == "wave") {
        double dx = L.direction[0], dy = L.direction[1];
        const double dl = std::sqrt(dx * dx + dy * dy);
        if (dl > 1e-9) {
          dx /= dl;
          dy /= dl;
        }
        const double d = x * dx + y * dy;
        const double wl = (L.wavelength != 0.0f) ? L.wavelength : 1.0;
        h += L.amplitude * std::sin(kTwoPi * d / wl + L.phase);
      }
    }
    return h;
  };
  const auto band_color = [&](double z) -> cvc::geometry::color_t {
    const float *col =
        hf.colors.back().color; // bands authored low->high; last = the "above all" cap
    for (const SceneHeightColorBand &b : hf.colors)
      if (z <= b.max_height) {
        col = b.color;
        break;
      }
    return {col[0], col[1], col[2]};
  };

  std::vector<float> heights(static_cast<std::size_t>(res) * res);
  // Each sample is a pure function of its position (a generated ground is a few million fBm
  // samples at a 2049 grid): the rows fill in parallel.
#ifdef CVC_USING_OPENMP
#pragma omp parallel for schedule(static)
#endif
  for (int j = 0; j < res; ++j)
    for (int i = 0; i < res; ++i)
      heights[static_cast<std::size_t>(j) * res + i] =
          static_cast<float>(height_at(-half + i * step, -half + j * step));
  auto field =
      std::make_shared<cvc::height_field>(res, res, -half, -half, step, step, std::move(heights));
  // UVs and band colours, one per vertex. The mesh's points and triangles follow the grid; these
  // attributes are the node's to set.
  const double inv = 1.0 / (res - 1);
  auto &uvs = field->uvs();
  uvs.resize(field->num_points());
  for (int j = 0; j < res; ++j)
    for (int i = 0; i < res; ++i)
      uvs[static_cast<std::size_t>(field->vertex_of(j, i))] = {i * inv, j * inv};
  if (!hf.colors.empty()) {
    auto &colors = field->colors();
    colors.resize(field->num_points());
    for (int j = 0; j < res; ++j)
      for (int i = 0; i < res; ++i)
        colors[static_cast<std::size_t>(field->vertex_of(j, i))] =
            band_color(field->height_at(j, i));
  }
  return field;
}

// Up axis y: (x, y, z) -> (x, -z, y), +90 degrees about x. Up axis x: the cyclic (x, y, z) ->
// (y, z, x). Flip: a half turn about x, (x, y, z) -> (x, -y, -z). All proper rotations, so
// winding and normals stay valid.
void turn_up_point(char up, bool flip, const double in[3], double out[3]) {
  const double x = in[0], y = in[1], z = in[2];
  if (up == 'y') {
    out[0] = x;
    out[1] = -z;
    out[2] = y;
  } else if (up == 'x') {
    out[0] = y;
    out[1] = z;
    out[2] = x;
  } else {
    out[0] = x;
    out[1] = y;
    out[2] = z;
  }
  if (flip) {
    out[1] = -out[1];
    out[2] = -out[2];
  }
}

char fit_up_axis(const SceneNode &n) {
  if (n.fit_up_y || n.fit_up == "y")
    return 'y';
  return n.fit_up == "x" ? 'x' : 'z';
}

cvc::geometry turn_up(const cvc::geometry &raw, char up, bool flip) {
  cvc::geometry g;
  g.copy(raw, true);
  if ((up != 'x' && up != 'y') && !flip)
    return g;
  for (auto &p : g.points()) {
    const double in[3] = {p[0], p[1], p[2]};
    double o[3];
    turn_up_point(up, flip, in, o);
    p = {{o[0], o[1], o[2]}};
  }
  for (auto &v : g.normals()) {
    const double in[3] = {v[0], v[1], v[2]};
    double o[3];
    turn_up_point(up, flip, in, o);
    v = {{o[0], o[1], o[2]}};
  }
  return g;
}

cvc::geometry fit_to_ground(const cvc::geometry &raw, char up, bool flip, float height) {
  const auto &P = raw.points();
  const auto &N = raw.normals();
  const auto &C = raw.colors();
  const auto rot = [up, flip](double x, double y, double z, double o[3]) {
    const double in[3] = {x, y, z};
    turn_up_point(up, flip, in, o);
  };
  double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
  for (const auto &p : P) {
    double w[3];
    rot(p[0], p[1], p[2], w);
    for (int k = 0; k < 3; ++k) {
      lo[k] = std::min(lo[k], w[k]);
      hi[k] = std::max(hi[k], w[k]);
    }
  }
  const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  const double s = ext > 0 ? static_cast<double>(height) / ext : 1.0;
  const double cx = 0.5 * (lo[0] + hi[0]), cy = 0.5 * (lo[1] + hi[1]);
  cvc::geometry g;
  for (std::size_t i = 0; i < P.size(); ++i) {
    double w[3];
    rot(P[i][0], P[i][1], P[i][2], w);
    g.points().push_back({(w[0] - cx) * s, (w[1] - cy) * s, (w[2] - lo[2]) * s});
    if (i < N.size()) {
      double nn[3];
      rot(N[i][0], N[i][1], N[i][2], nn); // a proper rotation preserves unit length
      g.normals().push_back({nn[0], nn[1], nn[2]});
    }
    if (i < C.size())
      g.colors().push_back(C[i]);
  }
  for (const auto &t : raw.tris())
    g.tris().push_back(t);
  return g;
}

bool read_scene_node_geometry(cvc::app &ctx, const SceneNode &n, cvc::geometry &geom,
                              std::shared_ptr<const cvc::height_field> &field,
                              std::vector<std::string> *warnings) {
  if (n.has_heightfield) {
    // A procedural displaced grid, no asset: the world's own height field, drawn as its mesh.
    try {
      ResolvedFile dem; // alive while the field is made (a fetched URI's temp file)
      if (!n.heightfield.file.empty()) {
        dem = resolve_to_file(ctx, n.heightfield.file, n.base_dir);
        if (!dem.ok)
          throw std::runtime_error("file " + dem.error);
      }
      std::shared_ptr<cvc::height_field> f = make_height_field(n.heightfield, dem.path);
      geom = *f;
      field = f;
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': heightfield: " + e.what());
      return false;
    }
  } else if (n.source_primitive == "plane") {
    geom = make_plane_geometry(n.plane_size); // a procedural ground quad — no asset
  } else if (n.source_primitive == "box") {
    geom = make_box_geometry(n.box_size);
  } else if (n.source_primitive == "sphere") {
    geom = make_sphere_geometry(n.sphere_radius);
  } else {
    if (n.source_file.empty()) {
      warn(warnings, "ari: scene node '" + n.id + "' (" + n.type +
                         ") has no source (a file/URI or a { plane } primitive)");
      return false;
    }
    ResolvedFile src = resolve_source(ctx, n, warnings);
    if (!src.ok)
      return false;
    try {
      geom = cvc::read_geometry(ctx, src.path);
    } catch (const std::exception &e) {
      warn(warnings, "ari: scene node '" + n.id + "': " + e.what());
      return false;
    }
  }
  if (n.has_fit) {
    // `fit:` — bake center/ground/scale (and optional Y-up→Z-up) into a LOADED mesh. A built-in
    // primitive is already canonically placed + sized (source.plane.size), and up:y would rotate
    // the ground quad into a vertical wall — so ignore fit on a primitive and say so.
    if (n.has_heightfield)
      warn(warnings, "ari: scene node '" + n.id +
                         "': `fit` is ignored on a heightfield (already centered on z=0; size it "
                         "via source.heightfield.size)");
    else if (n.source_primitive.empty())
      geom = fit_to_ground(geom, fit_up_axis(n), n.fit_flip, n.fit_height);
    else
      warn(warnings, "ari: scene node '" + n.id + "': `fit` is ignored on the built-in '" +
                         n.source_primitive +
                         "' primitive (already placed and sized; size it via source." +
                         n.source_primitive + ")");
  }
  return true;
}

void multiply_matrix4(const double a[16], const double b[16], double out[16]) {
  double r[16];
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k)
        s += a[i * 4 + k] * b[k * 4 + j];
      r[i * 4 + j] = s;
    }
  std::copy(r, r + 16, out);
}

void scene_node_local_matrix(const SceneNode &n, double out[16]) {
  for (int k = 0; k < 16; ++k)
    out[k] = (k % 5 == 0) ? 1.0 : 0.0;
  if (!n.has_transform)
    return;
  // Rz * Ry * Rx, as vtkTransform's RotateZ, RotateY, RotateX build it (pre-multiplied).
  constexpr double kDeg = 3.141592653589793 / 180.0;
  const auto rot = [](int axis, double rad, double m[16]) {
    for (int k = 0; k < 16; ++k)
      m[k] = (k % 5 == 0) ? 1.0 : 0.0;
    const double c = std::cos(rad), s = std::sin(rad);
    const int a = (axis + 1) % 3, b = (axis + 2) % 3;
    m[a * 4 + a] = c;
    m[a * 4 + b] = -s;
    m[b * 4 + a] = s;
    m[b * 4 + b] = c;
  };
  double rz[16], ry[16], rx[16], r[16];
  rot(2, n.rotation[2] * kDeg, rz);
  rot(1, n.rotation[1] * kDeg, ry);
  rot(0, n.rotation[0] * kDeg, rx);
  multiply_matrix4(rz, ry, r);
  multiply_matrix4(r, rx, r);
  // The scene graph's setScale scales each row of the rotation.
  const double s[3] = {n.scale[0], n.scale[1], n.scale[2]};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out[i * 4 + j] = r[i * 4 + j] * s[i];
  out[3] = n.position[0];
  out[7] = n.position[1];
  out[11] = n.position[2];
}

} // namespace ariadne
} // namespace cvc
