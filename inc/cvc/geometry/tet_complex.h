/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvc::tet_complex: the 3D Delaunay tetrahedralization of a point cloud, as a tetrahedral complex
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2). The skeletonization port's cocone code runs
// on it; it replaces CGAL's Delaunay_triangulation_3 and Triangulation_data_structure_3, which are
// GPL.
//
// Conventions are TDS_3's, so the cocone code that was written against CGAL maps onto it closely:
//   - Vertices are ints. Vertex i < num_points() is input point i (the same index); a point equal
//     to an earlier one is not a vertex (duplicate_of() names the vertex it coincides with). One
//     more vertex, infinite_vertex() == num_points(), closes the convex hull.
//   - Cells are ints, each with four vertices and four neighbours: neighbor(c, i) is the cell
//     across the facet opposite vertex(c, i), and mirror_index(c, i) is that facet's index in the
//     neighbour. Every facet of the convex hull has an infinite cell on its outer side, so every
//     cell has four neighbours.
//   - A finite cell is positively oriented as CGAL counts it: CGAL::orientation(v0, v1, v2, v3) is
//     POSITIVE, which is Shewchuk's orient3d(v0, v1, v2, v3) < 0. An infinite cell is oriented so
//     that putting a point outside the hull in place of the infinite vertex makes it positive.
//   - A facet is (cell, index of the opposite vertex); an edge is (cell, i, j).
//
// The triangulation is exactly Delaunay, by exact predicates: orientation and in-sphere tests are
// Shewchuk's adaptive predicates (cvcpkg's shewchuk-predicates), which are exact for any double
// input. Where points are cospherical (or coplanar on the hull) several triangulations are
// Delaunay, and the tie is broken by CGAL's symbolic perturbation (Devillers and Teillaud,
// "Perturbations and vertex removal in a 3D Delaunay triangulation", SODA 2003): points are
// perturbed in lexicographic (x, y, z) order. That makes the triangulation unique for a point set
// -- the same whatever the insertion order -- and the same one CGAL's Delaunay_triangulation_3
// builds, cell for cell, in general position and on degenerate input alike (tools/cgal_ab checks
// it). No cell is flat.
//
// Built by tet_complex_builder, a resumable stepper (§1.9): a spatial sort, then the points
// inserted one at a time (Bowyer-Watson: locate by a visibility walk from the last new cell, grow
// the conflict region, re-star its boundary from the new point), then the cells compacted and
// their circumcentres cached. Needs libcvc built with Shewchuk's predicates (CVC_WITH_SHEWCHUK;
// tet_complex_available() says); without them the builder throws.

#ifndef CVC_GEOMETRY_TET_COMPLEX_H
#define CVC_GEOMETRY_TET_COMPLEX_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

// What the builder throws: bad input (fewer than four distinct points, all of them coplanar, a
// NaN or infinite coordinate), a build without Shewchuk's predicates, or "cancelled".
class tet_complex_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// Whether this libcvc can build a tet_complex (it was built with Shewchuk's predicates).
bool tet_complex_available();

class tet_complex {
public:
  using point = std::array<double, 3>;
  struct cell {
    std::array<int, 4> v; // vertices
    std::array<int, 4> n; // n[i]: the neighbour across the facet opposite v[i]
  };

  tet_complex() = default;

  // ---- vertices ----
  int num_points() const { return int(_points.size()); }
  const std::vector<point> &points() const { return _points; }
  const point &point_of(int v) const { return _points[std::size_t(v)]; }
  int infinite_vertex() const { return int(_points.size()); }
  bool is_infinite(int v) const { return v == infinite_vertex(); }
  // The vertex input point i coincides with: i itself, or an earlier point with the same
  // coordinates (which is the vertex; point i then has no cells).
  int duplicate_of(int i) const { return _duplicate_of[std::size_t(i)]; }
  bool is_vertex(int i) const { return _duplicate_of[std::size_t(i)] == i; }
  int num_vertices() const { return _num_vertices; } // finite vertices: distinct points
  // A cell with vertex v (v a vertex, finite or infinite).
  int incident_cell(int v) const { return _vertex_cell[std::size_t(v)]; }
  // Whether vertex v is on the convex hull (it has an infinite cell).
  bool on_convex_hull(int v) const;

  // ---- cells ----
  int num_cells() const { return int(_cells.size()); } // finite and infinite
  int num_finite_cells() const { return _num_finite_cells; }
  const std::vector<cell> &cells() const { return _cells; }
  int vertex(int c, int i) const { return _cells[std::size_t(c)].v[std::size_t(i)]; }
  int neighbor(int c, int i) const { return _cells[std::size_t(c)].n[std::size_t(i)]; }
  // The index of vertex v in cell c, or -1 when v is not one of its vertices.
  int index(int c, int v) const;
  bool has_vertex(int c, int v) const { return index(c, v) >= 0; }
  // The index of cell n in c's neighbours, or -1.
  int neighbor_index(int c, int n) const;
  // The facet (c, i) seen from the other side: its index in neighbor(c, i), and the vertex there
  // opposite it.
  int mirror_index(int c, int i) const { return neighbor_index(neighbor(c, i), c); }
  int mirror_vertex(int c, int i) const { return vertex(neighbor(c, i), mirror_index(c, i)); }
  bool is_infinite_cell(int c) const { return index(c, infinite_vertex()) >= 0; }
  // The facet of c opposite vertex i, the edge of c through vertices i and j.
  bool is_infinite_facet(int c, int i) const;
  bool is_infinite_edge(int c, int i, int j) const;

  // ---- traversal ----
  // Every cell with vertex v, finite and infinite, each once, in breadth-first order from
  // incident_cell(v).
  void incident_cells(int v, std::vector<int> &out) const;
  // The cells around the edge (c, i, j), starting with c, in the order of CGAL's
  // Cell_circulator: each next one is the neighbour across the facet that contains the edge and
  // not the previous cell.
  void cells_around_edge(int c, int i, int j, std::vector<int> &out) const;

  // ---- geometry ----
  // Cached circumcentres (the Voronoi vertices), when the builder made them; NaN for infinite
  // cells.
  bool has_circumcentres() const { return _circumcentres.size() == _cells.size(); }
  const point &circumcentre(int c) const { return _circumcentres[std::size_t(c)]; }
  // Computed, not cached: the circumcentre of finite cell c, and its squared radius.
  point compute_circumcentre(int c, double *squared_radius = nullptr) const;

  // ---- checks ----
  // What is wrong with the complex, in words; empty when it is valid. Always checks the
  // combinatorics: four distinct vertices per cell, neighbours that point back across the same
  // three vertices, every finite cell positively oriented and no infinite cell inside the hull,
  // every vertex's incident cell, and the Euler characteristic of the closed complex (V - E + F -
  // C = 0 over finite and infinite simplices alike, the boundary of a 4-simplex's worth of
  // sphere). With `delaunay`, also that every finite cell's circumsphere is empty of every vertex
  // by the exact in-sphere test, including the perturbation, which is O(cells x vertices): for
  // tests on small inputs. Stops after `max_problems`.
  std::vector<std::string> validate(bool delaunay = false, std::size_t max_problems = 20) const;

  // The finite cells as sorted vertex quadruples, sorted: the canonical form two triangulations
  // of the same points are compared in.
  std::vector<std::array<int, 4>> finite_tetrahedra() const;

private:
  friend class tet_complex_builder;
  std::vector<point> _points;
  std::vector<int> _duplicate_of;
  std::vector<int> _vertex_cell; // per vertex, the infinite one last; -1 for a duplicate
  std::vector<cell> _cells;
  std::vector<point> _circumcentres;
  int _num_vertices = 0;
  int _num_finite_cells = 0;
};

struct tet_complex_options {
  // Cache every cell's circumcentre in a last phase.
  bool circumcentres = true;
  // Seeds the insertion order's random rounds (BRIO) and the walk's choice of facet. The
  // triangulation does not depend on it (it is unique, see above); only the cell numbering and
  // the time do.
  std::uint64_t seed = 0x5eed2026;
};

// What a build did, for tests and timing.
struct tet_complex_stats {
  std::size_t inserted = 0;   // points inserted
  std::size_t duplicates = 0; // points equal to an earlier one
  std::size_t walk_steps = 0; // cells the locating walks went through
  std::size_t conflict_cells = 0;
  std::size_t created_cells = 0;
  std::size_t perturbed = 0;    // in-sphere ties broken by the perturbation
  std::size_t max_conflict = 0; // the largest conflict region (cells)
};

// The Delaunay tetrahedralization, a slice at a time (§1.9). A unit is one item of the loop
// running: a point checked or keyed ("checking"), a key moved in one radix pass ("ordering"), a
// point inserted ("inserting": one walk plus one conflict region, which is about 25 cells on
// average and bounded only by the point set; a step may overrun its budget by that one point), a
// cell compacted ("compacting") or a circumcentre computed ("circumcentres"). The phases run in
// that order; "inserting" begins by finding four points not in one plane, one point per unit.
// Constructing it only stores the input. Throws tet_complex_error from step() for bad input (the
// checks run in the first phase) and on a build without Shewchuk's predicates; after a throw it is
// done, phase() is "failed", and step() and take() rethrow. Not thread-safe, except cancel(): the
// running step (or the next) stops and returns true without throwing, so a result_job around it
// ends "cancelled"; the builder is then done with phase() "failed", and take() throws "cancelled".
// It may move between threads between steps.
class tet_complex_builder {
public:
  explicit tet_complex_builder(std::shared_ptr<const std::vector<std::array<double, 3>>> points,
                               const tet_complex_options &opt = {});
  explicit tet_complex_builder(std::vector<std::array<double, 3>> points,
                               const tet_complex_options &opt = {});
  ~tet_complex_builder();
  tet_complex_builder(tet_complex_builder &&) noexcept;
  tet_complex_builder &operator=(tet_complex_builder &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  // "checking", "ordering", "inserting", "compacting", "circumcentres", "done" or "failed".
  const char *phase() const;
  double progress() const; // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel(); // thread-safe
  const tet_complex_stats &stats() const;
  tet_complex take(); // once done; moved out, so call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The blocking build: tet_complex_builder run with budget 0.
tet_complex delaunay_tet_complex(std::vector<std::array<double, 3>> points,
                                 const tet_complex_options &opt = {});

} // namespace cvc

#endif // CVC_GEOMETRY_TET_COMPLEX_H
