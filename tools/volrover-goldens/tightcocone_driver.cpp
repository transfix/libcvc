// Runs the legacy TightCocone (transfix/volrover src/TightCocone, at the sha in ../volrover.sha) on
// a point cloud, as VolumeRover2's Tools > Tight Cocone called it (surfaceReconstruction(const
// cvcgeom_t &, const Parameters &)), and writes what it returned:
//
//   tightcocone_legacy <points.pts> <out.txt> [name=value ...]
//
// points.pts holds one point a line, "x y z", read as doubles (the committed inputs are
// float-exact, so float and double parsers read the same values). The parameters are the legacy
// Parameters' (robust, bb_ratio, theta_ff, theta_if, flatness_ratio, cocone_phi, flat_phi), the
// rest at their defaults; reset_box=1 sets the patch CMakeLists.txt adds (tight_cocone.cpp's robust
// branch resets its min-max span before the second triangulation, as the port does).
//
// out.txt, coordinates with 17 digits:
//   points N                  the input points
//   robust 0|1
//   box x0 x1 y0 y1 z0 z1     the legacy global bounding_box's first six entries (what its readers
//                             read; the call appends six more each time)
//   vertices V                the returned geometry's points (every finite vertex), then
//   v x y z                   each of them, in the order returned
//   triangles T               then
//   t i j k                   each triangle by its corners' indices, in the order and orientation
//                             returned
// Part of tools/volrover-goldens: never built into libcvc.

#include <TightCocone/tight_cocone.h>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace TightCocone {
extern std::vector<double> bounding_box;
extern int tightcocone_driver_reset_box;
} // namespace TightCocone

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "usage: tightcocone_legacy <points.pts> <out.txt> [name=value ...]\n";
    return 2;
  }
  CVCGEOM_NAMESPACE::cvcgeom_t in;
  {
    std::ifstream f(argv[1]);
    double x, y, z;
    while (f >> x >> y >> z)
      in.points().push_back({x, y, z});
  }
  TightCocone::Parameters params;
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
    else if (k == "reset_box")
      TightCocone::tightcocone_driver_reset_box = v != 0;
    else {
      std::cerr << "unknown parameter: " << k << "\n";
      return 2;
    }
  }
  const CVCGEOM_NAMESPACE::cvcgeom_t out = TightCocone::surfaceReconstruction(in, params);

  std::FILE *f = std::fopen(argv[2], "wb");
  if (!f) {
    std::cerr << "cannot write " << argv[2] << "\n";
    return 1;
  }
  std::fprintf(f, "points %zu\nrobust %d\nbox", in.points().size(), params.b_robust() ? 1 : 0);
  for (std::size_t i = 0; i < 6 && i < TightCocone::bounding_box.size(); ++i)
    std::fprintf(f, " %.17g", TightCocone::bounding_box[i]);
  std::fprintf(f, "\nvertices %zu\n", out.points().size());
  for (const auto &p : out.points())
    std::fprintf(f, "v %.17g %.17g %.17g\n", p[0], p[1], p[2]);
  std::fprintf(f, "triangles %zu\n", out.triangles().size());
  for (const auto &t : out.triangles())
    std::fprintf(f, "t %llu %llu %llu\n", static_cast<unsigned long long>(t[0]),
                 static_cast<unsigned long long>(t[1]), static_cast<unsigned long long>(t[2]));
  std::fclose(f);
  return 0;
}
