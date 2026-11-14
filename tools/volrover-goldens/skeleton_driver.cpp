// Runs the legacy Skeletonization (transfix/volrover src/Skeletonization, at the sha in
// ../volrover.sha) on a point cloud, as VolumeRover2 called it, and writes what it returned:
//
//   skeleton_legacy <points.pts> <out.txt> [name=value ...]
//
// points.pts holds one point a line, "x y z"; the legacy Geometry keeps floats, so the points are
// read as floats (the committed inputs are float-exact). The parameters are the legacy
// Parameters' (robust, bb_ratio, theta_ff, theta_if, flatness_ratio, cocone_phi, flat_phi, theta,
// medial_ratio, threshold, pl_cnt, discard_by_threshold), the rest at their defaults.
//
// out.txt has a line per element of the returned Simple_skel, coordinates with 17 digits:
//   S x y z x y z            a star segment (the line strips coloured (0, 0, 1, 0.5))
//   L x y z x y z            a polyline segment (coloured (0, 1, 0, 1))
//   H n m x y z ...          a helix strip of n vertices, each with m = 1 at a maximum (the
//                            CMakeLists.txt patch gives these strips alpha 0.75, so they are
//                            told apart from the polyline segments)
//   P n x y z ...            a sheet polygon of n vertices
// in the order of the legacy std::sets. Part of tools/volrover-goldens: never built into libcvc.

#include <Skeletonization/Skeletonization.h>
#include <boost/shared_ptr.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
double get(const Skeletonization::Point &p, int k) { return CGAL::to_double(p[k]); }
} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "usage: skeleton_legacy <points.pts> <out.txt> [name=value ...]\n";
    return 2;
  }
  std::vector<float> xyz;
  {
    std::ifstream in(argv[1]);
    float x, y, z;
    while (in >> x >> y >> z) {
      xyz.push_back(x);
      xyz.push_back(y);
      xyz.push_back(z);
    }
  }
  Skeletonization::Parameters params;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    const std::size_t eq = a.find('=');
    if (eq == std::string::npos) {
      std::cerr << "not name=value: " << a << "\n";
      return 2;
    }
    const std::string k = a.substr(0, eq);
    const double v = std::atof(a.c_str() + eq + 1);
    if (k == "robust")
      params.b_robust(v != 0);
    else if (k == "bb_ratio")
      params.bb_ratio(v);
    else if (k == "theta_ff")
      params.theta_ff(v);
    else if (k == "theta_if")
      params.theta_if(v);
    else if (k == "flatness_ratio")
      params.flatness_ratio(v);
    else if (k == "cocone_phi")
      params.cocone_phi(v);
    else if (k == "flat_phi")
      params.flat_phi(v);
    else if (k == "theta")
      params.theta(v);
    else if (k == "medial_ratio")
      params.medial_ratio(v);
    else if (k == "threshold")
      params.threshold(v);
    else if (k == "pl_cnt")
      params.pl_cnt(int(v));
    else if (k == "discard_by_threshold")
      params.discard_by_threshold(v != 0);
    else {
      std::cerr << "unknown parameter: " << k << "\n";
      return 2;
    }
  }
  boost::shared_ptr<Geometry> geom(new Geometry);
  geom->m_NumTriVerts = static_cast<unsigned int>(xyz.size() / 3);
  geom->m_TriVerts.reset(new float[xyz.size()]);
  std::copy(xyz.begin(), xyz.end(), geom->m_TriVerts.get());

  const Skeletonization::Simple_skel skel = Skeletonization::skeletonize(geom, params);

  std::FILE *out = std::fopen(argv[2], "wb");
  if (!out) {
    std::cerr << "cannot write " << argv[2] << "\n";
    return 1;
  }
  const Skeletonization::Line_strip_set &strips = boost::get<0>(skel);
  const Skeletonization::Polygon_set &polygons = boost::get<1>(skel);
  for (const Skeletonization::Simple_line_strip &s : strips) {
    if (s.empty())
      continue;
    const double alpha = boost::get<3>(boost::get<1>(s[0]));
    if (alpha == 0.75) {
      std::fprintf(out, "H %zu", s.size());
      for (const Skeletonization::Simple_vertex &v : s) {
        const Skeletonization::Point &p = boost::get<0>(v);
        std::fprintf(out, " %d %.17g %.17g %.17g", boost::get<0>(boost::get<1>(v)) == 1.0 ? 1 : 0,
                     get(p, 0), get(p, 1), get(p, 2));
      }
      std::fprintf(out, "\n");
      continue;
    }
    if (s.size() != 2) {
      std::cerr << "a line strip of " << s.size() << " vertices that is not a helix strip\n";
      return 1;
    }
    std::fprintf(out, "%c", alpha == 0.5 ? 'S' : 'L');
    for (const Skeletonization::Simple_vertex &v : s) {
      const Skeletonization::Point &p = boost::get<0>(v);
      std::fprintf(out, " %.17g %.17g %.17g", get(p, 0), get(p, 1), get(p, 2));
    }
    std::fprintf(out, "\n");
  }
  for (const Skeletonization::Simple_polygon &poly : polygons) {
    std::fprintf(out, "P %zu", poly.size());
    for (const Skeletonization::Simple_vertex &v : poly) {
      const Skeletonization::Point &p = boost::get<0>(v);
      std::fprintf(out, " %.17g %.17g %.17g", get(p, 0), get(p, 1), get(p, 2));
    }
    std::fprintf(out, "\n");
  }
  std::fclose(out);
  return 0;
}
