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

// mesh_nodes.h -- custom Ariadne scene node types for geometry processing and finite elements,
// backed by cvc::mesh_ops / cvc::fem (libigl). The components mesh_tools.ari and fe_controls.ari
// drive them with no host C++.
//
//   type: mesh_lab -- a triangle mesh read like a `geometry` node (source + fit + material). Its
//     parameters and commands are node-scoped state keys under
//     "<node>.mesh_ops.*" (node->stateName("mesh_ops")), seeded from the node's props:
//       request          smooth | decimate | repair | orient | curvature | reset (consumed: the
//                        tick writes "" back; while a job runs the request waits)
//       busy, status, stats.vertices, stats.faces                      (published)
//       smooth.method (cotan | uniform | taubin), smooth.iterations, smooth.lambda,
//       smooth.fix_boundary, decimate.target_faces (cvc::simplify), repair.weld,
//       repair.remove_degenerate, repair.remove_unreferenced, repair.orient
//       color.mode (none | mean | gaussian | k1 | k2 | geodesic), color.colormap,
//       geodesic.source (clamped to #V-1)                             (followed live)
//       color.min, color.max                                          (published)
//
//   type: fe_lab -- a volume read like a `volren` node (source: { file } or { sdf }), tet-meshed
//     with the LBIE mesher (cvc::tetrahedralize) and solved with P1 finite elements (cvc::fem).
//     The node shows the field on the tet boundary surface; a child GeometryNode "<id>_slice"
//     shows it on an axis-aligned cut (cvc::slice_tets), hidden with the fe_lab node itself.
//     Keys under "<node>.fe.*":
//       request          mesh | solve | stop (stop interrupts a heat solve only; tet meshing and
//                        a Poisson solve run to the end, and the status says so). A request
//                        written while a job runs waits; auto_solve queues a solve after a mesh
//                        only when no request is waiting
//       busy, stoppable (1 while the running job honours stop), status, progress (0-100),
//       stats.tets, stats.vertices, field.min, field.max                (published)
//       mesh.isovalue, mesh.inside (below | above: which side of the isovalue is meshed),
//       mesh.improve (cvc::improvement_method), mesh.improve_iterations,
//       problem (poisson | heat), poisson.f, heat.kappa, heat.dt, heat.steps,
//       view.colormap, view.show_surface, view.show_slice, view.slice_axis (0 | 1 | 2),
//       view.slice_offset (-1..1 across the mesh bounds), view.surface_opacity
//
// Counts and indices read from state are range-checked (NaN = the default): smooth.iterations
// 1..1000, decimate.target_faces >= 1, mesh.improve 0..5, mesh.improve_iterations 0..100,
// heat.steps 1..1e6, view.slice_axis 0..2.
//
// Heavy work runs on one cvc::async_lane per node (deferred, on the render thread, in a wasm build
// without threads); results are applied by the node's per-frame tick on the render thread.

#ifndef CVC_GL_ARIADNE_MESH_NODES_H
#define CVC_GL_ARIADNE_MESH_NODES_H

namespace cvc {
namespace gl {
namespace ariadne {

// Register the "mesh_lab" and "fe_lab" scene node types. A no-op on a build without
// CVC_ENABLE_LIBIGL, so a document that declares them in `customs:` with `required: true` fails
// fast there. Idempotent; call before realize_scene (register_cvcgl_extensions does).
void register_mesh_node_types();

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_MESH_NODES_H
