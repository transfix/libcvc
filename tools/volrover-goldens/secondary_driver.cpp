// Runs the legacy SecondaryStructures (transfix/volrover src/SecondaryStructures, at the sha in
// ../volrover.sha) on a surface's points, as TexMol's SecondaryStructureData called it
// (compute_secondary_structures, then buildAllGeometry with its GUI's defaults), and writes what
// it found:
//
//   secondary_legacy <points.pts> <out.txt> [name=value ...]
//
// points.pts holds one point a line, "x y z" (the legacy surface has no triangles here, so every
// point is used; the committed inputs are float-exact). The parameters are buildAllGeometry's:
// alpha_count (20), beta_count (2), alpha_min (2.0), alpha_max (3.0), beta_min (2.0) and beta_max
// (3.0); the skeleton's own are hard-coded in compute_secondary_structures.
//
// out.txt, coordinates with 17 digits:
//   N nv ne nf components helices alpha_count beta_count max_sheets   the counts
//   A area                   a component's area, in component order
//   W width                  a helix-histogram width (get_alpha_histogram_data: the U2 edges
//                            outside the components), in edge order
//   F width                  a polygon's width (get_beta_histogram_data), in polygon order
//   R n x y z ...            a run of candidate points (_curve_comps), in the legacy order
//   H x y z x y z            a helix axis, start then end (the cylinder's two centres), longest
//                            first, every helix the fit kept (the first alpha_count are drawn)
//   P n x y z area           a polygon of the sheets drawn (sheetGeom): its vertex count, the
//                            centre it is fanned from and the fan's area, in the legacy order
// The skeleton's private data members are read by compiling Skel's header with `private` as
// `public` (after every other header, so only Skel changes). Its private member functions cannot
// be called that way (MSVC names a member function by its access). Part of tools/volrover-goldens:
// never built into libcvc.

// Every header but the library's own first, so `private` changes nothing in them.
#include <Histogram/histogram_data.h>
#include <SecondaryStructures/datastruct_ss.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cvcraw_geometry/cvcgeom.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#define _ALLOW_KEYWORD_MACROS
#define private public
#include <SecondaryStructures/skel.h>
#undef private

namespace {
double d(const SecondaryStructures::Point &p, int k) { return CGAL::to_double(p[k]); }
} // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "usage: secondary_legacy <points.pts> <out.txt> [name=value ...]\n";
    return 2;
  }
  int alpha_count = 20, beta_count = 2;
  double alpha_min = 2.0, alpha_max = 3.0, beta_min = 2.0, beta_max = 3.0;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    const std::size_t eq = a.find('=');
    if (eq == std::string::npos) {
      std::cerr << "not name=value: " << a << "\n";
      return 2;
    }
    const std::string k = a.substr(0, eq);
    const double v = std::atof(a.c_str() + eq + 1);
    if (k == "alpha_count")
      alpha_count = int(v);
    else if (k == "beta_count")
      beta_count = int(v);
    else if (k == "alpha_min")
      alpha_min = v;
    else if (k == "alpha_max")
      alpha_max = v;
    else if (k == "beta_min")
      beta_min = v;
    else if (k == "beta_max")
      beta_max = v;
    else {
      std::cerr << "unknown parameter: " << k << "\n";
      return 2;
    }
  }
  cvcraw_geometry::cvcgeom_t input;
  {
    std::ifstream in(argv[1]);
    float x, y, z;
    while (in >> x >> y >> z)
      input.points().push_back({x, y, z});
  }
  Skel skel;
  skel.compute_secondary_structures(&input);
  skel.buildAllGeometry(alpha_count, beta_count, float(alpha_min), float(alpha_max),
                        float(beta_min), float(beta_max), true, true);

  std::FILE *out = std::fopen(argv[2], "wb");
  if (!out) {
    std::cerr << "cannot write " << argv[2] << "\n";
    return 1;
  }
  std::fprintf(out, "N %d %d %d %d %zu %d %d %d\n", skel.get_nv(), skel.get_ne(), skel.get_nf(),
               skel.comp_cnt, skel.helices.size(), skel.helix_cnt, skel.beta_cnt, skel._max_sheets);
  for (double a : skel.A)
    std::fprintf(out, "A %.17g\n", a);
  for (double w : skel.get_alpha_histogram_data().points) // shim/Histogram keeps the widths
    std::fprintf(out, "W %.17g\n", w);
  for (double w : skel.get_beta_histogram_data().points)
    std::fprintf(out, "F %.17g\n", w);
  for (const std::vector<int> &run : skel._curve_comps) {
    std::fprintf(out, "R %zu", run.size());
    for (int k : run) {
      const SecondaryStructures::Point &p = skel._curve.vert_list[std::size_t(k)].pos;
      std::fprintf(out, " %.17g %.17g %.17g", d(p, 0), d(p, 1), d(p, 2));
    }
    std::fprintf(out, "\n");
  }
  for (const std::vector<SecondaryStructures::Point> &h : skel.helices) {
    const SecondaryStructures::Point &a = h[h.size() - 2], &b = h[h.size() - 1];
    std::fprintf(out, "H %.17g %.17g %.17g %.17g %.17g %.17g\n", d(a, 0), d(a, 1), d(a, 2), d(b, 0),
                 d(b, 1), d(b, 2));
  }
  // buildSheetGeometry fans each polygon drawn from its centre: the centre's vertex, then the
  // polygon's, and a triangle (vertex, next vertex, centre) per vertex. So a polygon is the run of
  // triangles sharing a third vertex.
  const cvcraw_geometry::cvcgeom_t &sheets = *skel.sheetGeom;
  const auto &sp = sheets.points();
  const auto &st = sheets.triangles();
  for (std::size_t i = 0; i < st.size();) {
    const unsigned int mid = st[i][2];
    std::size_t j = i;
    double area = 0;
    while (j < st.size() && st[j][2] == mid) {
      const auto &p = sp[st[j][0]], &q = sp[st[j][1]], &r = sp[st[j][2]];
      const double u[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
      const double v[3] = {r[0] - p[0], r[1] - p[1], r[2] - p[2]};
      const double c[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                           u[0] * v[1] - u[1] * v[0]};
      area += 0.5 * std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
      ++j;
    }
    std::fprintf(out, "P %zu %.17g %.17g %.17g %.17g\n", j - i, sp[mid][0], sp[mid][1], sp[mid][2],
                 area);
    i = j;
  }
  std::fclose(out);
  return 0;
}
