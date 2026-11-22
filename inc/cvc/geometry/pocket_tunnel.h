/*
  Copyright 2008, 2026 The University of Texas at Austin

        Authors: Samrat Goswami <samrat@ices.utexas.edu> (PocketTunnel)
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

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

// cvc::pocket_tunnel: the pockets, tunnels and voids of a surface, by the stable manifolds of the
// distance function's maxima outside it. Ported from VolumeRover's (and TexMol's) PocketTunnel
// (Goswami and Bajaj, UT 2008; transfix/volrover src/PocketTunnel) per
// docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2 (T5), on the cocone core the skeleton port
// shares (VOLROVER-LIBRARY-PORTS-ROADMAP.md §2). Its stages:
//   1. the 3D Delaunay triangulation of the surface's points (cvc::tet_complex, exact, CGAL's);
//   2. Tight Cocone: a watertight surface through the points, which splits the cells into inside
//      and outside;
//   3. S_max: from every maximum of the distance function (a Delaunay cell that contains its own
//      circumcentre, so no facet lets the flow out), the cells that flow into it, by the
//      circumcentres' flow across facets; touching clusters outside the surface are merged;
//   4. the mouths: a cluster outside the surface meets the cells that flow away to infinity in
//      patches of facets, its mouths. None makes it a void, one a pocket, more a tunnel;
//   5. the output: the clusters by volume, and the boundary facets of the largest few.
//
// The result keeps every cluster with its kind, mouths and volume, which the legacy library only
// printed, and has none of its process-wide output: it wrote progress to stderr and, in debug
// builds, files under debug_output/. Its caps of 100 tunnels and 100 pockets are gone.
//
// Resumable (§1.9): pocket_tunnel_stepper runs every stage as a phase, a unit at a time;
// pocket_tunnel() is the stepper at budget 0, and pocket_tunnel_job() its result_job. Needs
// libcvc built with Shewchuk's predicates (CVC_WITH_SHEWCHUK, as cvc::tet_complex does); without
// them every entry throws pocket_tunnel_error.

#ifndef CVC_GEOMETRY_POCKET_TUNNEL_H
#define CVC_GEOMETRY_POCKET_TUNNEL_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace cvc {

class pocket_tunnel_error : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// The defaults are VolumeRover2's dialog's (3 and 3) and the legacy constants
// (pocket_tunnel.cpp). Angles are in radians. A param_table (cvc/geometry/pocket_tunnel_params.h)
// gives the state keys, CLI flags, DSL dict and pycvc keywords.
struct pocket_tunnel_params {
  // How many of the largest pockets, and of the largest tunnels and voids together, `surface`
  // shows. Every cluster is listed in `features` either way.
  int pockets = 3;
  int tunnels = 3;
  // Tight Cocone.
  double cocone_phi = 3.14159265358979323846 / 8.0;      // half the cocone's opening angle
  double sharp_phi = 2.0 * 3.14159265358979323846 / 3.0; // the angle of sharp edges
  double flat_phi = 3.14159265358979323846 / 3.0;        // the flatness test's angle
  double flatness_ratio = 1.2 * 1.2;                     // the squared thinness factor
};

// A cluster of Delaunay cells outside the surface: the stable manifold of one or more maxima.
enum class pocket_kind {
  void_,  // no mouth: a cavity the outside cannot reach
  pocket, // one mouth
  tunnel, // two or more mouths
};
const char *to_string(pocket_kind k);

struct pocket_feature {
  pocket_kind kind = pocket_kind::void_;
  int mouths = 0;         // its mouths: patches of facets where it meets cells flowing to infinity
  double volume = 0.0;    // its cells' total volume
  std::size_t cells = 0;  // its Delaunay cells
  std::size_t maxima = 0; // the maxima whose stable manifolds were merged into it
  int shown = -1;         // its rank among the features `surface` shows (0 first), or -1
};

struct pocket_tunnels {
  // The legacy library's result: every input point (so a triangle's corners index the input),
  // coloured, and the boundary facets of the shown features as triangles, each facing away from
  // its feature. Shown, in order: the largest `tunnels` tunnels and voids, then the largest
  // `pockets` pockets. Colours, as the legacy library's:
  //   - a tunnel's facets on the surface yellow, its others (its mouths) red;
  //   - a void's facets on the reconstructed surface (cocone facets) purple, its others blue;
  //   - a pocket's facets on the surface green, its others (its mouth) purple;
  //   - every other point white.
  // A point on several features' facets takes the colour of the last feature shown, except that a
  // point red from a tunnel keeps red among the tunnels and voids, and a purple one keeps purple
  // among the pockets. Within one feature red and purple win; the legacy library let a void's
  // facets overwrite each other in its cells' memory order, and here purple wins there too.
  geometry surface;
  // Per surface triangle: the feature it bounds (an index into `features`), and whether it is on
  // the surface by the colour rule above (a pocket's or tunnel's facet between a cluster outside
  // the surface and one inside it; a void's cocone facet).
  std::vector<int> feature_of_tri;
  std::vector<bool> tri_on_surface;
  // Every cluster outside the surface, the largest volume first (ties in the order of their
  // first cell).
  std::vector<pocket_feature> features;
  int pockets = 0, tunnels = 0, voids = 0; // features of each kind
  int points = 0;                          // distinct input points
  std::vector<std::string> warnings;
};

// The pockets, tunnels and voids of the surface through `points`. Throws pocket_tunnel_error for
// fewer than four distinct points, points all in one plane or on one line, a coordinate that is
// not finite, invalid parameters, or a build without Shewchuk's predicates.
pocket_tunnels pocket_tunnel(const std::vector<std::array<double, 3>> &points,
                             const pocket_tunnel_params &params = {});
// The same for a surface mesh: only its points are read, as the legacy library read them.
pocket_tunnels pocket_tunnel(const geometry &surface, const pocket_tunnel_params &params = {});

// pocket_tunnel, a slice at a time (§1.9). Phases: the triangulation's ("checking", "ordering",
// "inserting", "compacting", "circumcentres"), then "building", "voronoi", "tight cocone",
// "maxima", "merging", "volumes", "sorting", "mouths" and "output". A unit is one iteration of
// the loop running: a point, cell or facet visited, or one step of a walk (Tight Cocone's
// umbrella and boundary walks, a maximum's growth, a mouth's facets around an edge), each bounded
// by the local degree; 16 of the triangulation's own units. A failure is a pocket_tunnel_error
// thrown from step() (and rethrown by later steps and take()). cancel() is the one thread-safe
// member: the step running (or the next) ends the stepper with no result; a result_job around it
// ends "cancelled". Not thread-safe otherwise, but it may move between threads between steps.
// Constructing one only stores its input.
struct pocket_tunnel_stepper_access;
class pocket_tunnel_stepper {
public:
  pocket_tunnel_stepper(std::vector<std::array<double, 3>> points,
                        pocket_tunnel_params params = {});
  pocket_tunnel_stepper(const geometry &surface, pocket_tunnel_params params = {});
  ~pocket_tunnel_stepper();
  pocket_tunnel_stepper(pocket_tunnel_stepper &&) noexcept;
  pocket_tunnel_stepper &operator=(pocket_tunnel_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // as listed above, "done", or "failed"
  double progress() const;   // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  pocket_tunnels take(); // once done; moved out, so call it once

private:
  friend struct pocket_tunnel_stepper_access; // libcvc's legacy A/B entry (internal)
  struct impl;
  std::unique_ptr<impl> _impl;
};

std::unique_ptr<pocket_tunnel_stepper>
pocket_tunnel_stepper_for(std::vector<std::array<double, 3>> points,
                          pocket_tunnel_params params = {});
std::unique_ptr<result_job<pocket_tunnels>>
pocket_tunnel_job(std::vector<std::array<double, 3>> points, pocket_tunnel_params params = {});
std::unique_ptr<result_job<pocket_tunnels>> pocket_tunnel_job(const geometry &surface,
                                                              pocket_tunnel_params params = {});

} // namespace cvc

#endif // CVC_GEOMETRY_POCKET_TUNNEL_H
