/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef CVC_GL_ARIADNE_MOLECULE_NODES_H
#define CVC_GL_ARIADNE_MOLECULE_NODES_H

// The `molecule` scene node type (TexMol plan M0 part 2,
// docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §6): a molecule the app put on the state tree, drawn
// as ball-and-stick, space-filling spheres, sticks or a secondary-structure cartoon, coloured by
// element, chain, residue, B-factor or structure.
//
//   data:
//     mol: ../data/molecules/1grl.pdb          # the app loads it (the `molecule` decoder)
//   scene:
//     nodes:
//       - node: protein
//         type: molecule
//         source: { data: mol }                # or { state: <key> }, or { node: <id> }
//         style: cartoon                       # ball_and_stick, spacefill, sticks, cartoon
//         color_by: structure                  # element, chain, residue, b_factor, structure
//         spheres: auto                        # auto, impostor, mesh
//         frame: true                          # frame the main view on it once drawn
//
// It reads no file (docs/roadmap/DATA_FROM_STATE_TREE.md): its source names a key holding a
// std::shared_ptr<const cvc::mol::molecule> (a data: entry's, decoded `as: molecule`, which is the
// default for .pdb, .ent, .pqr and .xyzr), a std::shared_ptr<cvc::mol::molecule> or a
// cvc::mol::molecule. It keeps no copy and follows the key: a new molecule there is drawn on the
// next tick. Two nodes on one source share one molecule (a cartoon and its ligands' sticks, say).
//
// Keys, under the node's state path:
//   source.molecule   the key it follows (relative to the document's scope, '/'-rooted the
//                     root's), seeded from source:; a program may point it elsewhere
//   source.status     "" while a molecule is shown, else what it waits for or why it shows none
//   molecule          published (data channel): the molecule shown, the very pointer
//   style color_by atoms hydrogens water atom_scale bond_radius bond_tolerance helix_width
//   sheet_width ribbon_thickness coil_radius subdivisions sides sphere_detail b_range
//                     the view's parameters (cvc/mol/view.h's table), seeded from the props of
//                     those names; a change rebuilds the view
//   spheres           auto (impostors, tessellated spheres where impostors can't run), impostor
//                     or mesh
//   chains            how many chains; chains.<i>.id ("" for atoms with none), .label (the id, or
//                     "(none)"), .visible (1 drawn, 0 hidden), .atoms and .color (its chain
//                     colour, "r,g,b" in 0..1), as a list: or repeat: binds them
//   info.atoms info.drawn info.bonds info.residues info.helices info.sheets info.id info.title
//                     (info.bonds is "-" for a style that finds no bonds: spacefill, cartoon)
//   info.spheres      what draws the spheres now: impostor, mesh, or "" for none
//   info.bounds       what it draws, "min_x min_y min_z max_x max_y max_z" (a camera's frame
//                     request, so a program can frame any view on it), "" for nothing
//   busy stoppable status progress phase stats.work_ms
//                     the view's build, a sliced job (cvc/core/job_keys.h); `stop` is not needed:
//                     a new build replaces the one running
//
// `frame: true` (or `first`) frames the main view's camera on what the node drew the first time it
// draws, and `frame: always` each time, as a data node's frame: does (docs/ARIADNE.md §5.17).
//
// A pick on the node (an `input: pick` viewport, its own on_pick:) reports the atom it landed on:
// atom, serial, name, element, residue, residue_seq, chain, model, b_factor, hetero and structure
// beside the usual x, y, z and node (MoleculeNode::atomFields).
//
// register_molecule_node_type() is part of the cvcGL extension bundle (register_cvcgl_extensions).

namespace cvc {
class app;
namespace gl {
namespace ariadne {

// Register `molecule` on `app`. Idempotent; call it before realize_scene().
void register_molecule_node_type(cvc::app &app);

} // namespace ariadne
} // namespace gl
} // namespace cvc

#endif // CVC_GL_ARIADNE_MOLECULE_NODES_H
