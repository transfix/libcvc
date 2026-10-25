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
//       request          smooth | decimate | repair | orient | curvature | reset | load | reload
//                        (consumed: the tick writes "" back; while a job runs the request waits).
//                        Several commands separated by spaces run in order ("load smooth").
//       busy, status, stats.vertices, stats.faces, stats.work_ms       (published)
//       source.file (a relative path resolves against the document's directory), source.up
//       (x | y | z), source.flip, source.height (> 0: fit) -- what `load` reads: a new file
//       when they changed, else back to the loaded mesh
//       smooth.method (cotan | uniform | taubin), smooth.iterations, smooth.lambda,
//       smooth.fix_boundary, decimate.target_faces (cvc::simplify), repair.weld,
//       repair.remove_degenerate, repair.remove_unreferenced, repair.orient
//       color.mode (none | mean | gaussian | k1 | k2 | geodesic | modes), color.colormap,
//       geodesic.source (clamped to #V-1; -1 = the highest vertex), modes.index (the
//       Laplace-Beltrami eigenfunction, >= 1)                         (followed live)
//       color.min, color.max                                          (published)
//
//   type: fe_lab -- a volume read like a `volren` node (source: { file } or { sdf }), meshed
//     with the LBIE mesher (cvc::tetrahedralize, tetrahedralize2 for an interval, hexahedralize)
//     and solved with P1 finite elements (cvc::fem). The node shows the field on the boundary (the
//     surface the elements approximate, or their own boundary); a child GeometryNode
//     "<id>_slice" shows it on an axis-aligned cut (cvc::slice_tets). In the elements view the
//     node shows the elements cut away, "<id>_edges" their outlines and "<id>_ghost" the outer
//     surface. The children hide with the fe_lab node itself. Keys under "<node>.fe.*":
//       request          load | reload | mesh | solve | stop (stop interrupts a heat solve only;
//                        the other jobs run to the end, and the status says so). A request
//                        written while a job runs waits; commands separated by spaces run in
//                        order. load reads the source again when source.* changed, then meshes
//                        (auto.mesh, when the mesh keys or the source changed) or solves
//                        (auto.solve); auto.solve also queues a solve after a mesh, only when
//                        no request is waiting
//       busy, stoppable (1 while the running job honours stop), status, progress (0-100),
//       stats.tets, stats.hexes, stats.vertices, stats.quality_min, stats.quality_mean,
//       stats.work_ms, field.min, field.max                            (published)
//       source.type (sdf | volume), source.file, source.dim, source.algorithm (v1 | v2 | igl),
//       source.up, source.flip, source.height, auto.mesh, auto.solve
//       mesh.isovalue, mesh.inside (below | above: which side of the isovalue is meshed),
//       mesh.interval, mesh.isovalue2, mesh.elements (tet | hex),
//       mesh.improve (cvc::improvement_method), mesh.improve_iterations,
//       problem (poisson | heat | modes), poisson.f, heat.kappa, heat.dt, heat.steps,
//       modes.index, view.mode (field | elements), view.boundary (input | mesh),
//       view.colormap, view.show_surface, view.show_slice, view.slice_axis (0 | 1 | 2),
//       view.slice_offset (-1..1 across the mesh bounds), view.surface_opacity,
//       view.color_by (size | quality), view.cut_side (1 | -1), view.ghost, view.fixed_range,
//       view.range_min, view.range_max
//
// Counts and indices read from state are range-checked (NaN = the default): smooth.iterations
// 1..1000, decimate.target_faces >= 1, mesh.improve 0..5, mesh.improve_iterations 0..100,
// heat.steps 1..1e6, modes.index 1..50, source.dim 0..1024, view.slice_axis 0..2.
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
