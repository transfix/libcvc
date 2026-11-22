// Runs the legacy PocketTunnel (transfix/volrover src/PocketTunnel, at the sha in ../volrover.sha)
// on a point cloud, as VolumeRover2's dialog and TexMol called it, and writes what it found:
//
//   pockettunnel_legacy <points.pts> <out.txt> [num_pockets=N] [num_tunnels=N]
//
// points.pts holds one point a line, "x y z"; pocket_tunnel_fromsurf reads its input's points as
// floats, so the committed inputs are float-exact. num_pockets and num_tunnels default to the
// dialog's 3 and 3.
//
// The driver calls pocket_tunnel_fromsurf() once. That function keeps its triangulation and
// clusters in locals, so CMakeLists.txt patches it to call pocket_tunnel_driver_report() (defined
// here) just before it converts them to its geometry; the hook reads them and changes nothing.
// One run a process: the legacy result depends on the cells' addresses (CGAL's facet iterator
// reports a facet from the cell at the lower address, and Tight Cocone's peeling starts and walks
// in that order), so a second triangulation in the same process, placed elsewhere in the heap,
// can be marked differently.
//
// out.txt, coordinates and volumes with 17 digits:
//   F kind mouths volume cells maxima   every cluster outside the surface, in the legacy order
//                                       (by volume, the largest first); kind is V (no mouth),
//                                       P (one) or T (more)
//   G points triangles                  the returned geometry's sizes
//   V x y z r g b                       its points (the triangulation's finite vertices, in
//                                       CGAL's order) and their colours
//   T a b c feature surface             its triangles, in its order (indices into the V lines),
//                                       each with the F line (from 0) of the cluster it bounds and
//                                       whether the legacy colour rule counts it on the surface
//                                       (1) or not (0)
//   O a b c d                           a finite cell Tight Cocone marked outside (its vertices,
//                                       ascending)
//   K a b c                             a finite facet Tight Cocone left on the surface (its
//                                       cocone flag; its vertices, ascending)
// The O and K lines are Tight Cocone's marking, from which the port's own stages (S_max, the
// mouths, the output) are compared exactly (pocket_tunnel_test). Part of tools/volrover-goldens:
// never built into libcvc.

#include <PocketTunnel/pocket_tunnel.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {

struct report {
  bool called = false;
  std::vector<std::string> features; // F lines
  std::vector<std::array<int, 3>> tri_v;
  std::vector<int> tri_feature, tri_surface;
  std::vector<std::array<int, 4>> outside;
  std::vector<std::array<int, 3>> cocone;
  long asymmetric = 0; // facets whose cocone flag differs between its two sides
  int num_pockets = 3, num_tunnels = 3;
} g_report;

} // namespace

namespace PocketTunnel {

// Called by the patched pocket_tunnel_fromsurf() after detect_handle(), before
// convert_pocket_tunnel_to_rawc_geometry(); reads the triangulation and the clusters.
void pocket_tunnel_driver_report(Triangulation &triang, map<int, cell_cluster> &cluster_set,
                                 const vector<int> &sorted) {
  report &r = g_report;
  r.called = true;

  // The clusters: volume and cells summed as calc_cluster_volume_and_store_with_cluster_rep
  // does (finite cells in order), and the maxima merged into each.
  std::map<int, int> rank;
  for (std::size_t k = 0; k < sorted.size(); ++k)
    rank[sorted[k]] = int(k);
  std::vector<double> volume(sorted.size(), 0.0);
  std::vector<long> cells(sorted.size(), 0), maxima(sorted.size(), 0);
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    const cell_cluster &c = cluster_set[cit->id];
    if (!c.in_cluster || !c.outside)
      continue;
    std::map<int, int>::const_iterator k = rank.find(c.find());
    if (k == rank.end()) {
      std::cerr << "an outside cell in no sorted cluster\n";
      std::exit(1);
    }
    volume[k->second] += cell_volume(cit);
    cells[k->second] += 1;
    if (is_maxima(cit))
      maxima[k->second] += 1;
  }
  for (std::size_t k = 0; k < sorted.size(); ++k) {
    const int m = cluster_set[sorted[k]].mouth_cnt;
    char buf[160];
    std::snprintf(buf, sizeof buf, "F %c %d %.17g %ld %ld",
                  m == 0   ? 'V'
                  : m == 1 ? 'P'
                           : 'T',
                  m, volume[k], cells[k], maxima[k]);
    r.features.push_back(buf);
  }

  // The triangles' clusters and classes: convert_pocket_tunnel_to_rawc_geometry's loops (op.cpp),
  // with the same shown clusters and facets in the same order; main() checks them against the
  // geometry it returns.
  for (int pass = 0; pass < 2; ++pass) {
    const int want = pass == 0 ? r.num_tunnels : r.num_pockets;
    int got = 0;
    for (std::size_t k = 0; k < sorted.size() && got < want && got < 100; ++k) {
      const int cl_id = sorted[k];
      const int m = cluster_set[cl_id].mouth_cnt;
      if ((m == 1) != (pass == 1))
        continue;
      ++got;
      for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
        Cell_handle c[2] = {(*fit).first, (*fit).first->neighbor((*fit).second)};
        int id[2] = {c[0]->index(c[1]), c[1]->index(c[0])};
        if (cluster_set[c[0]->id].find() == cluster_set[c[1]->id].find())
          continue;
        if (cluster_set[c[0]->id].find() != cl_id && cluster_set[c[1]->id].find() != cl_id)
          continue;
        Cell_handle temp_c = c[0];
        int temp_id = c[0]->index(c[1]);
        if (cluster_set[c[0]->id].find() != cl_id) {
          temp_c = c[1];
          temp_id = c[1]->index(c[0]);
        }
        int vid[3] = {temp_c->vertex((temp_id + 1) % 4)->id, temp_c->vertex((temp_id + 2) % 4)->id,
                      temp_c->vertex((temp_id + 3) % 4)->id};
        Tetrahedron tet(
            temp_c->vertex((temp_id + 1) % 4)->point(), temp_c->vertex((temp_id + 2) % 4)->point(),
            temp_c->vertex((temp_id + 3) % 4)->point(), temp_c->vertex(temp_id)->point());
        if (CGAL::to_double(tet.volume()) > 0)
          r.tri_v.push_back({{vid[0], vid[2], vid[1]}});
        else
          r.tri_v.push_back({{vid[0], vid[1], vid[2]}});
        r.tri_feature.push_back(int(k));
        if (m == 0)
          r.tri_surface.push_back(c[0]->cocone_flag(id[0]) ? 1 : 0);
        else
          r.tri_surface.push_back(
              cluster_set[c[0]->id].outside != cluster_set[c[1]->id].outside ? 1 : 0);
      }
    }
  }

  // Tight Cocone's marking.
  for (FCI cit = triang.finite_cells_begin(); cit != triang.finite_cells_end(); cit++) {
    if (!cit->outside)
      continue;
    std::array<int, 4> v = {
        {cit->vertex(0)->id, cit->vertex(1)->id, cit->vertex(2)->id, cit->vertex(3)->id}};
    std::sort(v.begin(), v.end());
    r.outside.push_back(v);
  }
  for (FFI fit = triang.finite_facets_begin(); fit != triang.finite_facets_end(); fit++) {
    Cell_handle c = (*fit).first;
    const int i = (*fit).second;
    Cell_handle n = c->neighbor(i);
    const bool a = c->cocone_flag(i), b = n->cocone_flag(n->index(c));
    if (a != b)
      ++r.asymmetric;
    if (!a && !b)
      continue;
    std::array<int, 3> v = {
        {c->vertex((i + 1) % 4)->id, c->vertex((i + 2) % 4)->id, c->vertex((i + 3) % 4)->id}};
    std::sort(v.begin(), v.end());
    r.cocone.push_back(v);
  }
  std::sort(r.outside.begin(), r.outside.end());
  std::sort(r.cocone.begin(), r.cocone.end());
}

} // namespace PocketTunnel

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "usage: pockettunnel_legacy <points.pts> <out.txt> [num_pockets=N] "
                 "[num_tunnels=N]\n";
    return 2;
  }
  report &r = g_report;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    const std::size_t eq = a.find('=');
    const std::string k = eq == std::string::npos ? a : a.substr(0, eq);
    const int v = eq == std::string::npos ? 0 : std::atoi(a.c_str() + eq + 1);
    if (k == "num_pockets")
      r.num_pockets = v;
    else if (k == "num_tunnels")
      r.num_tunnels = v;
    else {
      std::cerr << "unknown parameter: " << a << "\n";
      return 2;
    }
  }
  CVCGEOM_NAMESPACE::cvcgeom_t in;
  {
    std::ifstream f(argv[1]);
    float x, y, z;
    while (f >> x >> y >> z)
      in.points().push_back({{x, y, z}});
  }

  CVCGEOM_NAMESPACE::cvcgeom_t *ptv =
      PocketTunnel::pocket_tunnel_fromsurf(&in, r.num_pockets, r.num_tunnels);
  if (!r.called) {
    std::cerr << "pocket_tunnel_fromsurf did not call the driver's hook (is it patched?)\n";
    return 1;
  }
  bool same = r.tri_v.size() == ptv->triangles().size();
  for (std::size_t t = 0; same && t < r.tri_v.size(); ++t)
    for (int k = 0; k < 3; ++k)
      same = same && std::uint64_t(r.tri_v[t][k]) == ptv->triangles()[t][k];
  if (!same) {
    std::cerr << "the driver's triangles are not convert_pocket_tunnel_to_rawc_geometry's\n";
    return 1;
  }
  if (r.asymmetric)
    std::cerr << r.asymmetric << " facets have a cocone flag on one side only\n";

  std::FILE *out = std::fopen(argv[2], "wb");
  if (!out) {
    std::cerr << "cannot write " << argv[2] << "\n";
    return 1;
  }
  for (const std::string &f : r.features)
    std::fprintf(out, "%s\n", f.c_str());
  std::fprintf(out, "G %zu %zu\n", ptv->points().size(), ptv->triangles().size());
  for (std::size_t i = 0; i < ptv->points().size(); ++i) {
    const CVCGEOM_NAMESPACE::cvcgeom_t::point_t &p = ptv->points()[i];
    const CVCGEOM_NAMESPACE::cvcgeom_t::color_t &c = ptv->colors()[i];
    std::fprintf(out, "V %.17g %.17g %.17g %g %g %g\n", p[0], p[1], p[2], c[0], c[1], c[2]);
  }
  for (std::size_t t = 0; t < r.tri_v.size(); ++t)
    std::fprintf(out, "T %d %d %d %d %d\n", r.tri_v[t][0], r.tri_v[t][1], r.tri_v[t][2],
                 r.tri_feature[t], r.tri_surface[t]);
  for (const std::array<int, 4> &v : r.outside)
    std::fprintf(out, "O %d %d %d %d\n", v[0], v[1], v[2], v[3]);
  for (const std::array<int, 3> &v : r.cocone)
    std::fprintf(out, "K %d %d %d\n", v[0], v[1], v[2]);
  std::fclose(out);
  delete ptv;
  return 0;
}
