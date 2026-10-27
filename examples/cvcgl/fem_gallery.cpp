// fem_gallery — renders finite-element and geometry-processing results on a dataset to a PNG.
//
// One run = one picture: load a surface mesh or a scalar volume, build what the problem needs,
// solve it, colour the result, frame the camera, write the image. Everything happens
// synchronously before the first frame, so a capture never races a background job (unlike an
// .ari scene, whose mesh_lab / fe_lab nodes compute off the render thread).
//
// Domains:
//   --mesh FILE     any geometry file (or "stanford.bunny"). Volume problems mesh its interior:
//                   an SDF (cvc::sdf, SDF_IGL) at --dim, negated, through LBIE.
//   --volume FILE   a scalar volume (resampled to --dim when given); volume problems mesh
//                   {v > --iso} (--inside above, a density) or {v < --iso} (--inside below, an
//                   SDF).
// Problems (--problem):
//   volume:  poisson (-lap u = 1, u = 0 on the boundary), heat (backward Euler from a hot floor:
//            boundary vertices in the lowest 5% held at 1), modes (Neumann Laplacian eigenmode
//            --mode k of the solid, k >= 1), mesh (LBIE's elements themselves, see below)
//   surface: curvature (mean curvature), geodesic (heat-method distance from --source),
//            surface-modes (Laplace-Beltrami eigenfunction --mode k)
// LBIE meshing, for every volume problem:
//   --elements tet|hex   tetrahedra (FEM needs these) or hexahedra (--problem mesh only)
//   --iso2 V             mesh the interval between --iso and --iso2 instead of one side of
//                        --iso: a tissue layer between two CT densities, or a hollow shell
//                        (--mesh: the SDF is negative inside, so --iso2 -0.01 is 0.01 deep)
//   --improve M          quality improvement: geoflow | contract | joeliu | minvol | optimize,
//                        --improve-iters passes
// Views: the field on the outer boundary (translucent while a cut shows) and, for volume
// problems, a planar cut through the solid (--slice x|y|z|none at --slice-offset in -1..1).
// --problem mesh instead cuts the element mesh away: the elements beyond the plane (seen from the
// camera) stay whole, every exposed face flat-coloured by its element's --color-by value and
// outlined, so the cut face shows LBIE's octree adaptivity or its element quality.
//
//   fem_gallery --mesh stanford.bunny --up y --problem heat --steps 40 --png heat.png
//   fem_gallery --volume ct.rawiv --iso 45 --inside above --problem poisson --png body.png
//   fem_gallery --mesh stanford.bunny --up y --problem mesh --elements hex --color-by quality
//
// Prints one "fem_gallery:" line of stats (elements, timings, field range) per run.

#include <algorithm>
#include <array>
#include <boost/program_options.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/geometry/fem.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/mesh_ops.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/StageLighting.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/volume.h>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::SceneRenderer;
using cvc::gl::StageLighting;

namespace {

constexpr double kPi = 3.14159265358979323846;

using clock_t_ = std::chrono::steady_clock;
double ms_since(clock_t_::time_point t0) {
  return std::chrono::duration<double, std::milli>(clock_t_::now() - t0).count();
}

// Rotate a point so the dataset's own up axis becomes world +z, then optionally flip it upside
// down (a scan stored head-down) and turn it about z (which side faces the camera).
struct pose {
  char up = 'z';
  bool flip = false;
  double yaw_deg = 0.0;
  cvc::point_t operator()(const cvc::point_t &p) const {
    double x = p[0], y = p[1], z = p[2];
    if (up == 'y') { // (x, y, z) -> (x, -z, y)
      const double t = y;
      y = -z;
      z = t;
    } else if (up == 'x') { // a cyclic permutation: x becomes z
      const double t = x;
      x = y;
      y = z;
      z = t;
    }
    if (flip) {
      y = -y;
      z = -z;
    }
    const double a = yaw_deg * kPi / 180.0, c = std::cos(a), s = std::sin(a);
    return {{c * x - s * y, s * x + c * y, z}};
  }
};

void apply_pose(cvc::geometry &g, const pose &P) {
  for (auto &p : g.points())
    p = P(p);
  g.normals().clear(); // recomputed for display
}

// -v as a Float volume on the same grid: LBIE meshes the side ABOVE its isovalue.
cvc::volume negated(const cvc::volume &v) {
  const std::uint64_t n = v.XDim() * v.YDim() * v.ZDim();
  std::vector<float> buf(n);
  for (std::uint64_t i = 0; i < n; ++i)
    buf[i] = static_cast<float>(-v(i));
  return cvc::volume(v.ctx(), reinterpret_cast<const unsigned char *>(buf.data()),
                     v.voxel_dimensions(), cvc::Float, v.boundingBox());
}

// The outer boundary as a compact triangle surface (only the referenced vertices), with the
// field carried along: tet_boundary_surface keeps every tet point, which a display-only smooth
// would mishandle and which wastes the GPU's vertex budget.
cvc::geometry compact_surface(const cvc::geometry &tm, const std::vector<double> &field,
                              std::vector<double> &out_field) {
  const cvc::geometry full = cvc::tet_boundary_surface(tm);
  std::vector<std::int64_t> remap(full.const_points().size(), -1);
  cvc::geometry s;
  out_field.clear();
  for (const auto &t : full.const_tris()) {
    cvc::geometry::tri_t nt;
    for (int k = 0; k < 3; ++k) {
      auto &r = remap[t[k]];
      if (r < 0) {
        r = static_cast<std::int64_t>(s.points().size());
        s.points().push_back(full.const_points()[t[k]]);
        out_field.push_back(t[k] < field.size() ? field[t[k]] : 0.0);
      }
      nt[k] = static_cast<cvc::index_t>(r);
    }
    s.tris().push_back(nt);
  }
  return s;
}

// f (on src's vertices) sampled at dst's vertices: the closest point on src, blended across its
// triangle. Puts a solution computed on the tet mesh's terraced boundary onto the smooth input
// surface it approximates.
std::vector<double> transfer_field(const cvc::geometry &src, const std::vector<double> &f,
                                   const cvc::geometry &dst) {
  const auto cp = cvc::closest_points(src, dst.const_points());
  const auto &P = src.const_points();
  std::vector<double> out(dst.const_points().size(), 0.0);
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (cp.face[i] < 0)
      continue;
    const auto &t = src.const_tris()[static_cast<std::size_t>(cp.face[i])];
    double e0[3], e1[3], e2[3];
    for (int k = 0; k < 3; ++k) {
      e0[k] = P[t[1]][k] - P[t[0]][k];
      e1[k] = P[t[2]][k] - P[t[0]][k];
      e2[k] = cp.points[i][k] - P[t[0]][k];
    }
    const auto dot = [](const double *a, const double *b) {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    };
    const double d00 = dot(e0, e0), d01 = dot(e0, e1), d11 = dot(e1, e1), d20 = dot(e2, e0),
                 d21 = dot(e2, e1), den = d00 * d11 - d01 * d01;
    if (!(den > 0)) { // a degenerate triangle: take its first corner
      out[i] = f[t[0]];
      continue;
    }
    const double b1 = (d11 * d20 - d01 * d21) / den, b2 = (d00 * d21 - d01 * d20) / den;
    out[i] = (1.0 - b1 - b2) * f[t[0]] + b1 * f[t[1]] + b2 * f[t[2]];
  }
  return out;
}

// One element's --color-by value: size = log2 of its edge scale (the cube root of its volume),
// so each LBIE octree level is one colour band; quality = the minimum dihedral angle of a tet
// (degrees) or the scaled Jacobian of a hex (1 = a cube).
double element_value(const cvc::geometry &m, std::size_t e, bool hex, bool quality) {
  const auto &X = m.const_points();
  if (hex) {
    cvc::geometry::point_t v[8];
    for (int k = 0; k < 8; ++k)
      v[k] = X[m.const_hexs()[e][k]];
    return quality ? cvc::hex_scaled_jacobian(v)
                   : std::log2(std::cbrt(std::abs(cvc::hex_volume(v))) + 1e-300);
  }
  const auto &t = m.const_tets()[e];
  const auto &a = X[t[0]], &b = X[t[1]], &c = X[t[2]], &d = X[t[3]];
  if (quality)
    return cvc::tet_min_dihedral_angle(a, b, c, d);
  const double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]},
               v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]},
               w[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
  const double vol =
      std::abs(u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) +
               u[2] * (v[0] * w[1] - v[1] * w[0])) /
      6.0;
  return std::log2(std::cbrt(vol) + 1e-300);
}

// An element mesh cut away: the elements whose centroid lies on the kept side of the plane
// x[axis] = at (side * (c - at) <= 0; axis < 0 keeps them all), drawn as their exposed faces.
// Each face gets its own copies of its corners so it carries its element's value as a flat
// colour; edges holds the outline of every exposed face (no quad diagonals).
struct cutaway {
  cvc::geometry faces, edges;
  std::vector<double> values; // per point of faces
};

cutaway cut_away(const cvc::geometry &m, int axis, double at, double side, bool quality) {
  static const int kTetFaces[4][4] = {{1, 2, 3, 0}, {0, 3, 2, 0}, {0, 1, 3, 0}, {0, 2, 1, 0}};
  static const int kHexFaces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                      {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
  const auto &X = m.const_points();
  const bool hex = !m.const_hexs().empty();
  const std::size_t n = hex ? m.const_hexs().size() : m.const_tets().size();
  const int nc = hex ? 8 : 4, nf = hex ? 6 : 4, fv = hex ? 4 : 3;
  const auto corner = [&](std::size_t e, int k) {
    return hex ? m.const_hexs()[e][k] : m.const_tets()[e][k];
  };
  const auto face_corner = [&](std::size_t e, int f, int i) {
    return corner(e, hex ? kHexFaces[f][i] : kTetFaces[f][i]);
  };

  std::vector<cvc::point_t> centre(n);
  std::vector<char> keep(n);
  for (std::size_t e = 0; e < n; ++e) {
    cvc::point_t c{{0, 0, 0}};
    for (int k = 0; k < nc; ++k)
      for (int j = 0; j < 3; ++j)
        c[j] += X[corner(e, k)][j] / nc;
    centre[e] = c;
    keep[e] = axis < 0 || side * (c[axis] - at) <= 0;
  }

  // A face is exposed when exactly one kept element has it.
  using key_t = std::array<cvc::index_t, 4>;
  struct key_hash {
    std::size_t operator()(const key_t &k) const {
      std::size_t h = 0;
      for (cvc::index_t v : k)
        h = h * 1000003u ^ std::hash<cvc::index_t>{}(v);
      return h;
    }
  };
  const auto key = [&](std::size_t e, int f) {
    key_t k;
    k.fill(std::numeric_limits<cvc::index_t>::max());
    for (int i = 0; i < fv; ++i)
      k[i] = face_corner(e, f, i);
    std::sort(k.begin(), k.begin() + fv);
    return k;
  };
  std::unordered_map<key_t, int, key_hash> count;
  count.reserve(n * nf);
  for (std::size_t e = 0; e < n; ++e)
    if (keep[e])
      for (int f = 0; f < nf; ++f)
        ++count[key(e, f)];

  cutaway cw;
  auto &P = cw.faces.points();
  for (std::size_t e = 0; e < n; ++e) {
    if (!keep[e])
      continue;
    const double value = element_value(m, e, hex, quality);
    for (int f = 0; f < nf; ++f) {
      if (count[key(e, f)] != 1)
        continue;
      cvc::point_t p[4];
      for (int i = 0; i < fv; ++i)
        p[i] = X[face_corner(e, f, i)];
      // Wind it outward (away from its element's centre) so it lights from outside. The normal
      // of the winding p0 p1 p2 (p3): (p1 - p0) x (p2 - p0), or for a quad its diagonals'
      // (p2 - p0) x (p3 - p1).
      double u[3], w[3];
      for (int j = 0; j < 3; ++j) {
        u[j] = fv == 4 ? p[2][j] - p[0][j] : p[1][j] - p[0][j];
        w[j] = fv == 4 ? p[3][j] - p[1][j] : p[2][j] - p[0][j];
      }
      const double nrm[3] = {u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2],
                             u[0] * w[1] - u[1] * w[0]};
      double out = 0;
      for (int j = 0; j < 3; ++j)
        out += nrm[j] * (p[0][j] - centre[e][j]);
      if (out < 0)
        std::reverse(p, p + fv);
      const auto base = static_cast<cvc::index_t>(P.size());
      for (int i = 0; i < fv; ++i) {
        P.push_back(p[i]);
        cw.values.push_back(value);
        cw.edges.lines().push_back({{base + i, base + (i + 1) % fv}});
      }
      cw.faces.tris().push_back({{base, base + 1, base + 2}});
      if (fv == 4)
        cw.faces.tris().push_back({{base, base + 2, base + 3}});
    }
  }
  cw.edges.points() = P;
  return cw;
}

// The colour range: the [clip, 100 - clip] percentiles of the finite values, so a few spikes
// (curvature at a crease) don't wash out the rest of the map; clip = 0 is [min, max].
std::pair<double, double> color_range(const std::vector<double> &f, double clip) {
  const double q = std::clamp(clip, 0.0, 49.0) / 100.0;
  return cvc::robust_range(f, q, 1.0 - q);
}

} // namespace

int main(int argc, char **argv) {
  namespace po = boost::program_options;
  std::string mesh_path, volume_path, problem = "poisson", inside, colormap_name, slice = "y",
                                      png = "fem_gallery.png", up = "z", boundary = "input",
                                      elements = "tet", improve = "none", color_by = "size", range;
  double iso = 0.0, iso2 = 0.0, slice_offset = 0.0, opacity = 0.3, dt = 0.002, azimuth = -60.0,
         elevation = 20.0, zoom = 1.0, yaw = 0.0, smooth_lambda = 0.0, clip = -1.0, ghost = 0.0;
  int dim = 64, steps = 40, mode = 1, width = 1280, height = 900, source = -1, improve_iters = 1;
  bool flip = false;
  po::options_description desc("fem_gallery options");
  auto opt = desc.add_options();
  opt("help,h", "show help");
  opt("mesh", po::value<std::string>(&mesh_path), "surface mesh file (or stanford.bunny)");
  opt("volume", po::value<std::string>(&volume_path), "scalar volume file");
  opt("iso", po::value<double>(&iso), "volume isovalue bounding the domain");
  opt("iso2", po::value<double>(&iso2), "second isovalue: mesh the interval between the two");
  opt("inside", po::value<std::string>(&inside), "above (density) | below (SDF)");
  opt("dim", po::value<int>(&dim), "SDF resolution (--mesh) / resample to (--volume)");
  opt("elements", po::value<std::string>(&elements), "tet | hex (hex: --problem mesh only)");
  opt("improve", po::value<std::string>(&improve),
      "LBIE quality improvement: none | geoflow | contract | joeliu | minvol | optimize");
  opt("improve-iters", po::value<int>(&improve_iters), "LBIE improvement passes");
  opt("problem", po::value<std::string>(&problem),
      "poisson | heat | modes | mesh | curvature | geodesic | surface-modes");
  opt("steps", po::value<int>(&steps), "heat: backward-Euler steps");
  opt("dt", po::value<double>(&dt), "heat: step (diffusivity is extent^2, so dt is relative)");
  opt("mode", po::value<int>(&mode), "modes / surface-modes: which eigenfunction (>= 1)");
  opt("source", po::value<int>(&source), "geodesic: source vertex (default: the top vertex)");
  opt("color-by", po::value<std::string>(&color_by), "mesh: size | quality");
  opt("ghost", po::value<double>(&ghost), "mesh: opacity of the whole outer surface (0 = off)");
  opt("slice", po::value<std::string>(&slice), "x | y | z | none");
  opt("slice-offset", po::value<double>(&slice_offset), "-1..1 across the domain");
  opt("opacity", po::value<double>(&opacity), "boundary opacity while a slice shows");
  opt("boundary", po::value<std::string>(&boundary),
      "--mesh volume problems: show the field on the input surface (input) or on the tet "
      "mesh's own boundary (tet)");
  opt("colormap", po::value<std::string>(&colormap_name), "viridis | turbo | parula | ...");
  opt("clip", po::value<double>(&clip),
      "colour range = [clip, 100 - clip] percentiles (default: 2 for curvature, else 0)");
  opt("range", po::value<std::string>(&range), "fixed colour range LO,HI (overrides --clip)");
  opt("smooth", po::value<double>(&smooth_lambda),
      "cotan smoothing (e.g. 2e-4; 0 = off): of the boundary for display (volume problems), "
      "of the surface before solving (surface problems)");
  opt("up", po::value<std::string>(&up), "the dataset's up axis: x | y | z");
  opt("flip", po::bool_switch(&flip), "turn the dataset upside down");
  opt("yaw", po::value<double>(&yaw), "turn the dataset about the vertical (degrees)");
  opt("azimuth", po::value<double>(&azimuth), "camera azimuth (degrees)");
  opt("elevation", po::value<double>(&elevation), "camera elevation (degrees)");
  opt("zoom", po::value<double>(&zoom), "camera zoom (> 1 closer)");
  opt("width", po::value<int>(&width), "image width");
  opt("height", po::value<int>(&height), "image height");
  opt("png", po::value<std::string>(&png), "output image");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n" << desc << "\n";
    return 2;
  }
  if (vm.count("help") || (mesh_path.empty() == volume_path.empty())) {
    std::cout << desc << "\nexactly one of --mesh / --volume is required\n";
    return vm.count("help") ? 0 : 2;
  }
  if (!cvc::mesh_ops_available()) {
    std::cerr << "fem_gallery: this libcvc was built without CVC_ENABLE_LIBIGL\n";
    return 1;
  }
  const bool surface_problem =
      problem == "curvature" || problem == "geodesic" || problem == "surface-modes";
  const bool mesh_problem = problem == "mesh";
  const bool hex = elements == "hex";
  const bool interval = vm.count("iso2") > 0;
  if (surface_problem && mesh_path.empty()) {
    std::cerr << "fem_gallery: --problem " << problem << " needs --mesh\n";
    return 2;
  }
  if (problem == "poisson" && slice == "none") { // the boundary is where u = 0 is imposed
    std::cerr << "fem_gallery: --problem poisson needs a --slice; the boundary is all u = 0\n";
    return 2;
  }
  if (hex && (!mesh_problem || interval)) {
    std::cerr << "fem_gallery: --elements hex is for --problem mesh, without --iso2\n";
    return 2;
  }
  static const std::pair<const char *, cvc::improvement_method> kImprove[] = {
      {"none", cvc::NO_IMPROVE}, {"geoflow", cvc::GEO_FLOW},   {"contract", cvc::EDGE_CONTRACT},
      {"joeliu", cvc::JOE_LIU},  {"minvol", cvc::MINIMAL_VOL}, {"optimize", cvc::OPTIMIZATION}};
  cvc::improvement_method improve_method = cvc::NO_IMPROVE;
  bool improve_known = false;
  for (const auto &[name, m] : kImprove)
    if (improve == name) {
      improve_method = m;
      improve_known = true;
    }
  if (!improve_known) {
    std::cerr << "fem_gallery: unknown --improve " << improve << "\n";
    return 2;
  }
  double range_lo = 0, range_hi = 0;
  if (!range.empty() && std::sscanf(range.c_str(), "%lf,%lf", &range_lo, &range_hi) != 2) {
    std::cerr << "fem_gallery: --range wants LO,HI\n";
    return 2;
  }
  if (colormap_name.empty())
    colormap_name = problem == "heat"      ? "turbo"
                    : problem == "poisson" ? "parula"
                    : mesh_problem         ? (color_by == "quality" ? "inferno" : "turbo")
                                           : "viridis";
  cvc::colormap_kind cm = cvc::colormap_kind::VIRIDIS;
  if (!cvc::colormap_from_string(colormap_name, cm)) {
    std::cerr << "fem_gallery: unknown colormap " << colormap_name << "\n";
    return 2;
  }
  const pose P{up.empty() ? 'z' : up[0], flip, yaw};
  const double az = azimuth * kPi / 180.0, el = elevation * kPi / 180.0;
  const double view_dir[3] = {std::cos(el) * std::cos(az), std::cos(el) * std::sin(az),
                              std::sin(el)}; // from the target toward the camera
  const int axis = slice == "x" ? 0 : slice == "y" ? 1 : slice == "z" ? 2 : -1;

  cvc::app app;
  std::vector<double> field;
  std::shared_ptr<cvc::geometry> display; // what the camera frames
  cvc::geometry slice_geom, edges, ghost_geom;
  std::string stats;
  try {
    if (surface_problem) {
      // ---- a field on the surface itself ----
      cvc::geometry g = cvc::read_geometry(mesh_path);
      apply_pose(g, P);
      if (smooth_lambda > 0) { // a terraced scan surface would put its steps into the field
        cvc::smooth_params sp;
        sp.lambda = smooth_lambda;
        sp.iterations = 1;
        cvc::smooth(g, sp);
      }
      const auto t0 = clock_t_::now();
      if (problem == "curvature") {
        field = cvc::vertex_curvature(g, cvc::curvature_kind::MEAN);
      } else if (problem == "geodesic") {
        std::size_t src = 0;
        if (source >= 0)
          src = static_cast<std::size_t>(source);
        else // the top vertex: the crown of a head, the ears of a bunny
          for (std::size_t i = 1; i < g.const_points().size(); ++i)
            if (g.const_points()[i][2] > g.const_points()[src][2])
              src = i;
        field = cvc::geodesic_distance(g, {static_cast<cvc::index_t>(src)});
      } else {
        const auto r =
            cvc::fem::laplacian_eigenmodes(g, std::max(1, mode) + 1, cvc::fem::domain::SURFACE);
        field = r.vectors.at(static_cast<std::size_t>(std::max(1, mode)));
      }
      stats = "surface " + std::to_string(g.const_points().size()) + " verts, " +
              std::to_string(g.const_tris().size()) + " tris; " + problem + " " +
              std::to_string(static_cast<int>(ms_since(t0))) + " ms";
      display = std::make_shared<cvc::geometry>(g);
    } else {
#if !defined(CVC_ENABLE_MESHER) || !defined(CVC_ENABLE_SDF)
      throw std::runtime_error("volume problems need CVC_ENABLE_MESHER and CVC_ENABLE_SDF");
#else
      // ---- a solid: mesh it with LBIE, then solve and show boundary + cut, or show the mesh ----
      const auto tm0 = clock_t_::now();
      cvc::volume vol(app);
      cvc::geometry input; // the --mesh surface, when the field is shown on it
      bool below = false;
      if (!mesh_path.empty()) {
        cvc::geometry g = cvc::read_geometry(mesh_path);
        if (boundary == "input" || (mesh_problem && ghost > 0))
          input = g;
        const cvc::bounding_box e = g.extents();
        const double ext = std::max({e.maxx - e.minx, e.maxy - e.miny, e.maxz - e.minz});
        const double h = 0.55 * ext; // 10% margin so the zero level never touches the box
        const double cx = 0.5 * (e.minx + e.maxx), cy = 0.5 * (e.miny + e.maxy),
                     cz = 0.5 * (e.minz + e.maxz);
        vol = cvc::sdf(app, g, cvc::dimension(dim, dim, dim),
                       cvc::bounding_box(cx - h, cy - h, cz - h, cx + h, cy + h, cz + h),
                       cvc::SDF_IGL);
        below = true; // an SDF is negative inside
        iso = 0.0;
      } else {
        vol.read(volume_path);
        if (vm.count("dim"))
          vol.resize(cvc::dimension(dim, dim, dim));
        below = inside == "below";
      }
      const cvc::volume v = below ? negated(vol) : vol;
      const double a = below ? -iso : iso, b = below ? -iso2 : iso2;
      cvc::geometry tm;
      if (interval) // LBIE takes the interval's upper isovalue first
        tm = cvc::tetrahedralize2(v, std::max(a, b), std::min(a, b), cvc::DUALLIB, improve_method,
                                  cvc::BSPLINE_CONVOLUTION, improve_iters);
      else if (hex)
        tm = cvc::hexahedralize(v, a, cvc::DUALLIB, improve_method, cvc::BSPLINE_CONVOLUTION,
                                improve_iters);
      else
        tm = cvc::tetrahedralize(v, a, cvc::DUALLIB, improve_method, cvc::BSPLINE_CONVOLUTION,
                                 improve_iters);
      if (tm.const_tets().empty() && tm.const_hexs().empty())
        throw std::runtime_error("no elements: the isovalue(s) do not cut the volume");
      if (!hex)
        cvc::orient_tets(tm);
      apply_pose(tm, P);
      if (!input.const_points().empty())
        apply_pose(input, P);
      const double mesh_ms = ms_since(tm0);
      const auto &X = tm.const_points();
      const std::size_t n_elem = hex ? tm.const_hexs().size() : tm.const_tets().size();
      stats = std::to_string(n_elem) + (hex ? " hexes, " : " tets, ") + std::to_string(X.size()) +
              " verts; mesh " + std::to_string(static_cast<int>(mesh_ms)) + " ms";
      double lo[3], hi[3];
      for (int k = 0; k < 3; ++k) {
        lo[k] = std::numeric_limits<double>::infinity();
        hi[k] = -lo[k];
      }
      for (const auto &p : X)
        for (int k = 0; k < 3; ++k) {
          lo[k] = std::min(lo[k], p[k]);
          hi[k] = std::max(hi[k], p[k]);
        }
      const double cut_at =
          axis < 0 ? 0.0 : 0.5 * (lo[axis] + hi[axis]) + 0.5 * slice_offset * (hi[axis] - lo[axis]);

      if (mesh_problem) {
        // The half beyond the plane stays, so its cut face looks at the camera.
        const double side = axis < 0 ? 1.0 : (view_dir[axis] >= 0 ? 1.0 : -1.0);
        cutaway cw = cut_away(tm, axis, cut_at, side, color_by == "quality");
        const auto q =
            hex ? cvc::compute_hex_quality_stats(tm.const_hexs(), X, cvc::HEX_SCALED_JACOBIAN)
                : cvc::compute_tet_quality_stats(tm.const_tets(), X, cvc::TET_MIN_ANGLE);
        char buf[160];
        std::snprintf(buf, sizeof buf, "; %s min %.3g mean %.3g",
                      hex ? "scaled Jacobian" : "min dihedral (deg)", q.min, q.mean);
        stats += buf;
        display = std::make_shared<cvc::geometry>(std::move(cw.faces));
        field = std::move(cw.values);
        edges = std::move(cw.edges);
        if (ghost > 0)
          ghost_geom = !input.const_points().empty() ? input : cut_away(tm, -1, 0, 1, false).faces;
        smooth_lambda = 0; // the elements are the point
      } else {
        if (hex)
          throw std::runtime_error("FEM problems need --elements tet");
        const auto ts = clock_t_::now();
        const cvc::fem::domain D = cvc::fem::domain::VOLUME;
        const auto bnd = cvc::fem::boundary_vertices(tm, D);
        if (problem == "heat") {
          cvc::fem::dirichlet bc;
          for (cvc::index_t bv : bnd)
            if (X[bv][2] <= lo[2] + 0.05 * (hi[2] - lo[2]))
              bc.vertices.push_back(bv);
          bc.values.push_back(1.0);
          const double ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
          cvc::fem::heat_params hp;
          hp.kappa = ext * ext;
          hp.dt = dt;
          hp.steps = std::max(1, steps);
          field = cvc::fem::solve_heat(tm, std::vector<double>(X.size(), 0.0), hp, bc, D);
        } else if (problem == "modes") {
          const auto r = cvc::fem::laplacian_eigenmodes(tm, std::max(1, mode) + 1, D);
          field = r.vectors.at(static_cast<std::size_t>(std::max(1, mode)));
        } else {
          cvc::fem::dirichlet bc;
          bc.vertices = bnd;
          bc.values.push_back(0.0);
          field = cvc::fem::solve_poisson(tm, {1.0}, bc, D);
        }
        stats += ", " + problem + " " + std::to_string(static_cast<int>(ms_since(ts))) + " ms";

        std::vector<double> sfield;
        display = std::make_shared<cvc::geometry>(compact_surface(tm, field, sfield));
        if (!input.const_points().empty() && !interval) {
          sfield = transfer_field(*display, sfield, input);
          display = std::make_shared<cvc::geometry>(std::move(input));
          smooth_lambda = 0; // the input surface is already the smooth one
        }
        if (axis >= 0) {
          cvc::geometry::vector_t nrm{{0, 0, 0}};
          nrm[axis] = 1.0;
          tm.functions() = field;
          slice_geom = cvc::slice_tets(tm, nrm, cut_at);
        }
        field = std::move(sfield);
      }
#endif
    }
  } catch (const std::exception &e) {
    std::cerr << "fem_gallery: " << e.what() << "\n";
    return 1;
  }

  if (smooth_lambda > 0 && !surface_problem) { // display only: the field rides along
    cvc::smooth_params sp;
    sp.lambda = smooth_lambda;
    sp.iterations = 1;
    cvc::smooth(*display, sp);
  }
  cvc::compute_vertex_normals(*display);
  if (clip < 0)
    clip = problem == "curvature" ? 2.0 : 0.0;
  const auto [flo, fhi] = color_range(field, clip);
  double slo = flo, shi = fhi;
  if (!slice_geom.const_functions().empty()) {
    const auto [a, b] = color_range(slice_geom.const_functions(), clip);
    slo = std::min(slo, a);
    shi = std::max(shi, b);
  }
  if (!range.empty()) {
    slo = range_lo;
    shi = range_hi;
  }
  std::printf("fem_gallery: %s; field [%g, %g]\n", stats.c_str(), slo, shi);

  // ---- scene ----
  const cvc::bounding_box b = display->extents();
  const double cx = 0.5 * (b.minx + b.maxx), cy = 0.5 * (b.miny + b.maxy),
               cz = 0.5 * (b.minz + b.maxz);
  const double r = 0.5 * std::sqrt((b.maxx - b.minx) * (b.maxx - b.minx) +
                                   (b.maxy - b.miny) * (b.maxy - b.miny) +
                                   (b.maxz - b.minz) * (b.maxz - b.minz));
  SceneGraph sg(app, "fem_gallery");
  SceneRenderer view(sg, width, height, /*offscreen=*/true, "main");
  view.setBackground(0.06, 0.07, 0.09);
  sg.setDiagnosticChromeVisible(false); // no grid / axes / bounding boxes in a figure
  const bool cut = !slice_geom.const_tris().empty();
  auto surf = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("surface", *display));
  surf->setUseSingleColor(false);
  surf->setScalarField(field, cm, slo, shi);
  surf->setAmbient(0.25);
  surf->setDiffuse(0.85);
  if (cut)
    surf->setOpacity(opacity);
  if (cut) {
    cvc::compute_vertex_normals(slice_geom);
    auto sn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("slice", slice_geom));
    sn->setUseSingleColor(false);
    sn->setScalarField(slice_geom.const_functions(), cm, slo, shi);
    sn->setAmbient(0.45);
    sn->setDiffuse(0.7);
  }
  if (!edges.const_lines().empty()) {
    // Lifted a hair toward the camera so the outlines win the depth test against their faces.
    const double lift = 1.5e-3 * r;
    for (auto &p : edges.points())
      for (int k = 0; k < 3; ++k)
        p[k] += lift * view_dir[k];
    auto en = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("edges", edges));
    en->setRenderMode(cvc::gl::GeometryRenderMode::LINES);
    en->setUseSingleColor(true);
    en->setColor(0.04, 0.04, 0.05);
    en->setLineWidth(1.0);
  }
  if (!ghost_geom.const_tris().empty()) {
    cvc::compute_vertex_normals(ghost_geom);
    auto gn = std::dynamic_pointer_cast<GeometryNode>(sg.addGraphics("ghost", ghost_geom));
    gn->setUseSingleColor(true);
    gn->setColor(0.8, 0.82, 0.86);
    gn->setOpacity(ghost);
  }

  StageLighting rig(sg);
  rig.setStage(cx, cy, cz, r);
  rig.applyPreset(StageLighting::Preset::ThreePoint);

  const double dist = r / std::tan(0.5 * 30.0 * kPi / 180.0) / std::max(0.1, zoom) * 1.05;
  view.setCamera(cx + dist * view_dir[0], cy + dist * view_dir[1], cz + dist * view_dir[2], cx, cy,
                 cz, 0, 0, 1, 30.0, dist * 0.01, dist * 10.0);
  for (int f = 0; f < 4; ++f) // a few frames so the shadow maps bake
    view.render();
  view.writePNG(png);
  std::printf("fem_gallery: wrote %s\n", png.c_str());
  return 0;
}
