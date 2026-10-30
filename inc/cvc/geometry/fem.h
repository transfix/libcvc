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

// fem.h -- linear (P1) finite elements on cvc::geometry: tetrahedral volumes
// (e.g. cvc::tetrahedralize() output) and triangle surfaces. Assembly uses
// libigl's cotangent stiffness and mass matrices; solves use Eigen's sparse
// Cholesky (igl::min_quad_with_fixed for Dirichlet conditions). Like
// mesh_ops.h this header is Eigen-free, every function is always declared,
// and every function throws cvc::mesh_ops_unavailable when libcvc was built
// without CVC_ENABLE_LIBIGL.
//
// Scope: scalar PDEs -- Poisson, Laplace (harmonic interpolation), heat
// diffusion, and Laplacian eigenmodes. Vector-valued elasticity is not here
// (libigl has no linear-elastic / neo-Hookean assembly); see
// docs/roadmap/LIBIGL-GEOMETRY-FE-ROADMAP.md.
//
// Elements. With domain::VOLUME (or AUTO and non-empty tets()) the elements
// are tets() -- of either orientation; an internal copy is oriented. With
// domain::SURFACE (or AUTO and no tets) they are the triangle surface
// (tris() plus quads split as in mesh_ops.h). Degenerate elements are dropped
// from assembly: |volume| or area below 1e-12 of the bounding-box scale, or
// too flat for libigl's cotangent weights, which it computes from edge lengths
// (a triangle with height/length below ~7e-7; a tet with |6 volume| below
// 1e-6 of its longest edge cubed). A vertex left with no element is held at 0
// (or its Dirichlet value). More than INT_MAX/16 elements throws
// cvc::mesh_ops_error (libigl's 32-bit index arithmetic would overflow).
//
// Units. Fields are per vertex, aligned with points(). The operators are in
// the mesh's own length unit: K has units of length^(d-2), M of length^d
// (d = 2 on a surface, 3 in a volume).

#ifndef __CVC_GEOMETRY_FEM_H__
#define __CVC_GEOMETRY_FEM_H__

#include <cstddef>
#include <cstdint>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/mesh_ops.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace fem {

enum class domain {
  AUTO,    // VOLUME when tets() is non-empty, else SURFACE
  SURFACE, // triangle surface (tris + split quads)
  VOLUME   // tetrahedra
};

// Dirichlet (essential) boundary condition: u[vertices[i]] = values[i]. A
// single value is broadcast to every listed vertex.
struct dirichlet {
  std::vector<index_t> vertices;
  std::vector<double> values;
};

// Sparse matrix in coordinate (triplet) form; duplicates already summed.
struct sparse_matrix {
  std::size_t rows = 0, cols = 0;
  std::vector<std::size_t> row, col;
  std::vector<double> value;
};

// The vertices on the domain boundary: vertices of boundary facets for a tet
// mesh (igl::boundary_facets), vertices on boundary loops for a surface.
// Sorted ascending, unique.
std::vector<index_t> boundary_vertices(const geometry &mesh, domain d = domain::AUTO);

// Stiffness matrix K = -L (L = igl::cotmatrix): symmetric positive
// semi-definite, rows sum to zero. #V x #V.
sparse_matrix stiffness_matrix(const geometry &mesh, domain d = domain::AUTO);

// Mass matrix: lumped (diagonal; Voronoi on triangles, barycentric on tets)
// or consistent (full P1 mass). #V x #V.
sparse_matrix mass_matrix(const geometry &mesh, domain d = domain::AUTO, bool lumped = true);

// Per-element gradient of a P1 field (piecewise constant), 3 values per
// element: [e0x, e0y, e0z, e1x, ...]. Elements are tets (VOLUME) or the
// triangles (SURFACE), in element order.
std::vector<double> gradient(const geometry &mesh, const std::vector<double> &u,
                             domain d = domain::AUTO);

// Solve -lap(u) = f  (weak form K u = M f) with Dirichlet conditions.
// f is per vertex, or a single value for a constant source. Throws
// cvc::mesh_ops_error if bc is empty (the problem would be singular).
std::vector<double> solve_poisson(const geometry &mesh, const std::vector<double> &f,
                                  const dirichlet &bc, domain d = domain::AUTO);

// Harmonic interpolation: lap(u) = 0 with Dirichlet conditions.
std::vector<double> solve_laplace(const geometry &mesh, const dirichlet &bc,
                                  domain d = domain::AUTO);

struct heat_params {
  double kappa = 1.0; // diffusivity (length^2 / time)
  double dt = 1e-2;   // time step
  int steps = 1;      // backward-Euler steps: (M + dt*kappa*K) u+ = M u
};

// Called after every step with (step index 1..steps, current field); return
// false to stop early (the field at that step is returned).
typedef std::function<bool(int, const std::vector<double> &)> heat_progress;

// Heat diffusion from initial field u0 (per vertex). Vertices listed in bc are
// held at their values every step (an empty bc means insulated / Neumann).
// One factorization is reused for every step.
std::vector<double> solve_heat(const geometry &mesh, const std::vector<double> &u0,
                               const heat_params &p, const dirichlet &bc = dirichlet(),
                               domain d = domain::AUTO,
                               const heat_progress &progress = heat_progress());

// The k smallest eigenpairs of the generalized problem K x = lambda M x
// (lumped M): the Laplace-Beltrami spectrum on a surface, the free-vibration
// (Neumann) modes in a volume. vectors[i] is per vertex, M-normalized.
// Intended for small k (<= ~20). Above 400 vertices the modes come from an
// iterative solver, which throws cvc::mesh_ops_error if it has not converged
// after 500 iterations.
struct eigen_result {
  std::vector<double> values;
  std::vector<std::vector<double>> vectors;
};
eigen_result laplacian_eigenmodes(const geometry &mesh, int k, domain d = domain::AUTO);

// --- solves a slice at a time -------------------------------------------------------------------
//
// solve_poisson, solve_heat and laplacian_eigenmodes as resumable jobs, for a caller that must not
// block: fe_lab steps one on the app's scheduler (cvc/core/sliced_task.h), a few milliseconds a
// frame where there are no threads. Every phase is a loop that stops at its budget: selecting the
// elements, assembling K and M (in chunks of elements, with the same libigl formulas), building the
// sparse matrix, eliminating the Dirichlet vertices, the factorization (a sparse LDL^T over the AMD
// ordering Eigen's SimplicialLDLT uses, a column at a time; the ordering itself is one step of a
// few milliseconds) and each solve. So the results agree with the one-shot functions to rounding.
//
// step(budget) does about `budget` units (each about a multiply-add or an element's assembly, at
// most ~1 us) and returns true once done; budget 0 runs to the end. It throws mesh_ops_error where
// the one-shot function would (bad input, a singular system, an eigensolve that does not
// converge); after a throw every later step rethrows it. phase() names the work next: "preparing",
// "assembling", "building", "constraining", "ordering", "factoring", "solving", "stepping" (heat)
// or "iterating" (modes), then "done". The mesh is shared, so the job may outlive its caller's.
class stepped_solve {
public:
  // solve_poisson: f is per vertex or a single value.
  static std::unique_ptr<stepped_solve> poisson(std::shared_ptr<const geometry> mesh,
                                                std::vector<double> f, dirichlet bc,
                                                domain d = domain::AUTO);
  // solve_heat. The field after each step is field(); stop() ends it after the current step.
  static std::unique_ptr<stepped_solve> heat(std::shared_ptr<const geometry> mesh,
                                             std::vector<double> u0, heat_params p, dirichlet bc,
                                             domain d = domain::AUTO);
  // laplacian_eigenmodes.
  static std::unique_ptr<stepped_solve> modes(std::shared_ptr<const geometry> mesh, int k,
                                              domain d = domain::AUTO);

  ~stepped_solve();
  stepped_solve(const stepped_solve &) = delete;
  stepped_solve &operator=(const stepped_solve &) = delete;

  bool step(std::size_t budget);
  bool done() const;
  double progress() const; // in [0,1], monotone; 1 once done
  const char *phase() const;
  void stop();            // heat: end after the current step (its field is the result)
  int steps_done() const; // heat: backward-Euler steps taken
  // The result once done: the solution, or heat's field after its last step (also while it runs,
  // after each step). modes: the eigenpairs.
  const std::vector<double> &field() const;
  eigen_result take_modes();

private:
  stepped_solve();
  struct impl;
  std::unique_ptr<impl> _impl;
};

} // namespace fem
} // namespace cvc

#endif // __CVC_GEOMETRY_FEM_H__
