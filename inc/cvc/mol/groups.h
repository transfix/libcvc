/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's group tree: TexMol's PDBParser hierarchy (its GroupOfAtoms tree) as a view over a
// flat molecule (TexMol plan M0 part 2, docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2).
//
//   collection          the file
//     model             one per MODEL (one for a file with none)
//       chain           a run of atoms with one chain id (a chain id that comes back starts
//       another)
//         structure     a helix, a sheet (every strand of it, in one group), a turn, or none
//           residue     a run of atoms with one residue number and insertion code
//             atoms     indices into the molecule's atoms, in file order
//
//   cvc::mol::group_tree t = cvc::mol::build_group_tree(m);
//   for (std::size_t c : t.groups[t.groups[0].children[0]].children) ...   // model 1's chains
//   std::vector<std::size_t> order = cvc::mol::flatten_atoms(t);          // TexMol's atom order
//
// It is built by PDBParser's rules (parserPDBtoGOA::parseATOM), so the tree and the order its
// depth-first walk gives (FlattenGOA's) are TexMol's, which the ports that compare with TexMol's
// output need (blur's A/B, F2Dock's inputs):
//   - HELIX and TURN records are matched in file order: the next one starts a group when a residue
//     equals its first residue, and the group takes every residue that follows in the chain until
//     its last;
//   - a SHEET record with strand 1 starts a sheet and the strands after it join it; a residue in
//     any of its strands (within the sheet's first strand's chain, by number) joins the one sheet
//     group, which goes into the chain once, where its first residue is. So a sheet's strands come
//     together in the walk (1GRL's residues 48-50 right after 38-40);
//   - a residue inside a helix's range never joins a sheet;
//   - a residue in no structure joins a `none` group, a new one after each structure;
//   - only the first model has structures (TexMol forgot them after it): later models' chains hold
//     one `none` group each.
// Two departures, where PDBParser's rule was a slip: a new chain always starts a new residue
// (TexMol put a chain's first atoms into the previous chain's last residue when the numbers
// matched), and a chain is a nucleic acid's when its first residue is a nucleotide (TexMol counted
// adenine as an amino acid).
//
// group_tree_stepper builds it a slice at a time (a unit per atom), so a large assembly never
// stalls a frame. Everything here holds no state of its own.

#ifndef CVC_MOL_GROUPS_H
#define CVC_MOL_GROUPS_H

#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/mol/molecule.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace mol {

struct group {
  enum class kind { collection, model, chain, structure, residue };
  enum class structure_kind { helix, sheet, turn, none };

  kind type = kind::collection;
  std::vector<std::size_t> children; // indices into group_tree::groups, in order
  std::vector<std::size_t> atoms;    // a residue's atoms (indices into the molecule's), in order
  // model: its number (an atom's `model`); chain: its id and kind; residue: its chain, number and
  // insertion code, and its name (its first atom's)
  int model = 0;
  char chain = ' ';
  bool nucleic = false;
  structure_kind structure = structure_kind::none;
  int record = -1; // a helix, sheet or turn: its (first) record, an index into structures
  int residue_seq = 0;
  char insertion = ' ';
  std::string residue;
};

struct group_tree {
  std::vector<group> groups; // groups[0] is the collection; empty for no atoms
};

// The molecule's atoms in the order a depth-first walk of the first model gives (TexMol's
// FlattenGOA, without its biomolecule copies).
std::vector<std::size_t> flatten_atoms(const group_tree &t);

// The tree in parser_legacy's text (tools/texmol-goldens/parser_driver.cpp), one line a group,
// indented a blank a level, without its residues' torsions: what molecule_groups_test compares with
// TexMol's own tree.
std::string describe(const group_tree &t);

// The tree a slice at a time. The molecule is shared, read-only, for as long as the stepper lives.
// cancel() is the one thread-safe member.
class group_tree_stepper {
public:
  explicit group_tree_stepper(std::shared_ptr<const molecule> m);
  ~group_tree_stepper();
  group_tree_stepper(group_tree_stepper &&) noexcept;
  group_tree_stepper &operator=(group_tree_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // "grouping", "done" or "cancelled"
  double progress() const;
  void cancel();
  group_tree take(); // once done; moved out, call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The tree at once: group_tree_stepper run to the end.
group_tree build_group_tree(const molecule &m);
std::unique_ptr<result_job<group_tree>> group_tree_job(std::shared_ptr<const molecule> m);

} // namespace mol
} // namespace cvc

#endif // CVC_MOL_GROUPS_H
