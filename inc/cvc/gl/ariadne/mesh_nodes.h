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
//     and <node>.field_colors (below).
//
//   type: fe_lab -- a volume read like a `volren` node (source: { file } or { sdf }), meshed
//     with the LBIE mesher (cvc::tetrahedralize, tetrahedralize_interval below, hexahedralize)
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
//       last_mesh (published: the region and bounds the last mesh job started with, as
//       fe_mesh_signature writes them; "" before the first)
//       source.type (sdf | volume), source.file, source.dim, source.algorithm (v1 | v2 | igl),
//       source.up, source.flip, source.height, auto.mesh, auto.solve
//       mesh.region (solid | interval: what is meshed), mesh.isovalue,
//       mesh.inside (below | above: which side of the isovalue a solid mesh fills),
//       mesh.isovalue2 (an interval's second bound: the inclusive shell between the two
//       isovalues, in either order, whatever mesh.inside says; a bound may lie past the volume's
//       range -- see tetrahedralize_interval), mesh.elements (tet | hex; an interval is
//       tetrahedra only), mesh.improve (cvc::improvement_method), mesh.improve_iterations,
//       problem (poisson | heat | modes), poisson.f, heat.kappa, heat.dt, heat.steps,
//       modes.index, view.mode (field | elements), view.boundary (input | mesh),
//       view.colormap, view.show_surface, view.show_slice, view.slice_axis (0 | 1 | 2),
//       view.slice_offset (-1..1 across the mesh bounds), view.surface_opacity,
//       view.color_by (size | quality), view.cut_side (1 | -1), view.ghost, view.fixed_range,
//       view.range_min, view.range_max
//     The mesh keys are seeded from props: region (default solid), isovalue (0), isovalue2, inside
//     (below for an sdf source, else above), and `interval: [a, b]`, shorthand for region interval,
//     isovalue a, isovalue2 b (`interval: true` alone is region interval); an explicit region /
//     isovalue / isovalue2 prop beside it wins. With no isovalue2 the second bound is a tenth of
//     the volume's value range from isovalue on the side a solid mesh fills (below it for inside
//     below: an SDF's inner skin; above it for a density), but no more than half way to that end
//     of the range; on the other side when isovalue sits at (or past) that end. (An SDF source is
//     computed after realize: its load sets that default, unless mesh.isovalue2 was set first.)
//     <node>.transfer_function is bound (tf_binding.h): its isocontours drive the bounds, over
//     the range of the volume the last load brought.
//
//   Field colours (both types): "<node>.field_colors" holds a cvc::tf::model (a tf_editor widget
//   edits it) whose colour ramp the shown values map through -- mesh_lab's colour field; fe_lab's
//   field on the boundary and the cut, and its elements' size or quality -- over the range they
//   are coloured with: the model's explicit domain, else (fe_lab) view.fixed_range's, else their
//   own. Its opacity curve and isocontours are not used. The colormap key (color.colormap,
//   view.colormap) seeds it and, whenever it changes, resets it to that colormap's ramp (auto
//   domain); otherwise an edit stands. Beside it the node publishes .range (the colours' span
//   under an auto domain) and .histogram (128 bins of the values over it; fe_lab: the whole
//   solution, or the shown elements' values). mesh_lab's color.min / color.max are the range used.
//
// Counts and indices read from state are range-checked (NaN = the default): smooth.iterations
// 1..1000, decimate.target_faces >= 1, mesh.improve 0..5, mesh.improve_iterations 0..100,
// heat.steps 1..1e6, modes.index 1..50, source.dim 0..1024, view.slice_axis 0..2.
//
// Heavy work runs on one cvc::async_lane per node (deferred, on the render thread, in a wasm build
// without threads); results are applied by the node's per-frame tick on the render thread.

#ifndef CVC_GL_ARIADNE_MESH_NODES_H
#define CVC_GL_ARIADNE_MESH_NODES_H

#include <cvc/geometry/geometry.h>
#include <cvc/volume/volume.h>
#include <string>

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register the "mesh_lab" and "fe_lab" scene node types. A no-op on a build without
// CVC_ENABLE_LIBIGL, so a document that declares them in `customs:` with `required: true` fails
// fast there. Idempotent; call before realize_scene (register_cvcgl_extensions does).
void register_mesh_node_types();

// fe_lab's interval mesher (mesh.region: interval): a tet mesh of the shell
// min(a, b) <= v <= max(a, b) of `vol` -- the bounds in either order -- whose boundary is both
// isosurfaces, an outer and an inner sheet where the shell encloses a cavity. It is
// cvc::tetrahedralize2's interval mesh, with the bounds chosen as follows. The interval is
// inclusive: a voxel whose value equals a bound (integer or plateau data, a bar at an end of an
// editor's axis) is inside it, so [1, 3] on a label volume keeps labels 1 and 3. Such a bound is
// meshed half way to the nearest value beyond it -- on integer data the half-integer: [1, 3] on
// labels gives the same mesh as [0.5, 3.5] (cvc::tetrahedralize2 itself counts a voxel on a bound
// as outside). A bound no voxel lies beyond (at or past an end of the value range) bounds
// nothing: [0.5, 1] and [0.5, 2] mesh the same {v >= 0.5} of a 0/1 mask, crossing at 0.5 like a
// solid mesh; with neither bound passed it is the whole box. Values and bounds are compared in
// float, as LBIE reads them. The tets come as LBIE writes them (orient them with cvc::orient_tets
// before a solve), at `vol`'s positions. `improve` is a cvc::improvement_method, run
// `improve_iterations` times. Throws std::runtime_error "no tets: the interval [lo, hi] ..." when
// the interval is empty or holds no voxel (the message gives the volume's value range) or LBIE
// finds nothing in it, and "mesher unavailable" in a build without CVC_ENABLE_MESHER. Available
// without CVC_ENABLE_LIBIGL.
cvc::geometry tetrahedralize_interval(const cvc::volume &vol, double a, double b, int improve = 0,
                                      int improve_iterations = 1);

// fe_lab: the region and bounds the keys under `fe_base` ("<node>.fe") hold now, read as fe_lab
// reads them for a mesh job: "solid <isovalue>" or "interval <lo> <hi>" (the bounds sorted; %.17g).
// fe_lab publishes it at <fe_base>.last_mesh for each mesh job it starts, whoever asked for it, so
// the transfer-function binding drops a debounced remesh some other request already did.
std::string fe_mesh_signature(cvc::app &app, const std::string &fe_base);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_MESH_NODES_H
