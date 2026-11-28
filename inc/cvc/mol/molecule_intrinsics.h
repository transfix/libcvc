/*
  Copyright 2026 The University of Texas at Austin
  This file is part of libcvc.
  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/
// cvc::mol in the DSL (TexMol plan M0 part 2; docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.6):
//
//   (mol.info m)                     a dict: id, title, atoms, models, chains (their ids, a
//                                    string), residues, helices, sheets, turns, hetero (atoms),
//                                    bounds (min x y z, max x y z, a list); at once
//   (mol.torsions m)                 a list, a dict a residue: chain, seq, insertion, residue,
//                                    kind (amino_acid, nucleotide or other), and its angles
//                                    (radians; nil where undefined): an amino acid's phi, psi and
//                                    omega, a nucleotide's alpha .. zeta, nu (a list of five),
//                                    chi, eta and theta (cvc/mol/torsions.h)
//   (mol.blur m [params])            its electron density, a cvc::volume (cvc/mol/blur.h); params
//                                    blur_params' fields
//   (mol.view m [params])            a dict: spheres, sticks and cartoon (cvc::geometry, the meshes
//                                    cvc/mol/view.h builds; spheres only with sphere_mesh), atoms,
//                                    bonds and residues; params view_params' fields
//
// mol.torsions, mol.blur and mol.view are jobs, each in cvc/core/job_intrinsics.h's three forms:
// `(op ...)` waits, `(op-async ...)` gives a future, `(op-start ... target)` the task's pid at
// once, the task publishing its keys under `target` and its result at `target.result` (the form for
// an Ariadne action). `m` is a molecule: a data object holding one, or a state path naming the node
// whose data channel does (a document's data.<name>.molecule, a molecule node's published
// molecule). The results are fresh values; the molecule is never written. A program stores them
// where nodes follow them:
//
//   (mol.blur-start "data.grl.molecule" (dict "resolution" 2.0 "blobbiness" -0.2) "blurring")
//   ... once (state-get "blurring.status") is "done":
//   (data-put "density" "volume" (state-data-get "blurring.result"))
//
// Like every intrinsic, none of these reads or writes a file.
#ifndef CVC_MOL_MOLECULE_INTRINSICS_H
#define CVC_MOL_MOLECULE_INTRINSICS_H
#include <cvc/core/job_intrinsics.h>
#include <cvc/core/sliced_task.h>
#include <memory>
namespace cvc {
class app;
namespace state_exec {
struct intrinsics_context;
}
// The ops, for a host that registers ops itself.
job_op molecule_torsions_op(sliced_threading threading = sliced_threading::automatic);
job_op molecule_blur_op(sliced_threading threading = sliced_threading::automatic);
job_op molecule_view_op(sliced_threading threading = sliced_threading::automatic);
// mol.info, the three ops' forms, and job.cancel and params.*, in `env`.
void register_molecule_intrinsics(state_exec::environment_ptr env,
                                  state_exec::intrinsics_context &ictx, cvc::app &app);
// The same for every program lane of `app`'s Ariadne documents: a provider registered on the app
// under the key "cvc.molecule" (registering again replaces it), so no other app sees them. cvcGL's
// register_cvcgl_extensions installs it.
void install_molecule_intrinsics(cvc::app &app);
} // namespace cvc
#endif // CVC_MOL_MOLECULE_INTRINSICS_H
