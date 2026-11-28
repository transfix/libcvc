/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol: molecules read from PDB, PQR and XYZR files, the base of the TexMol library ports
// (docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.2, M0).
//
//   cvc::mol::molecule m = cvc::mol::read_molecule(app, "1grl.pdb");
//   for (const cvc::mol::atom &a : m.atoms)
//     use(a.x, a.y, a.z, a.radius);
//   int s = m.structure_of(m.atoms[0]); // its helix or sheet, -1 for none
//
// The model is flat: every atom read, in file order, with its residue, chain, model and the
// secondary structures (HELIX, SHEET, TURN) the file lists by residue range. TexMol's
// PDBParser kept the same atoms in a tree of groups (collection, protein, chain, secondary
// structure, residue); the tree is a view over this list, for the ports that need it.
//
// Radii and charges. A PDB file has neither, so they come from the residue-atom table TexMol's
// PDBParser carries (L. Kapcha and P. Rossky's, ElementInformation.h): one row per standard amino
// acid and nucleotide atom, keyed by atom name and residue name. An atom the table doesn't have
// (a ligand, water, a terminal OXT) gets molecule_read_params::unknown_radius. A PQR file gives
// both per atom, and an XYZR file the radius.
//
// Files. The readers are each app's own (molecule_handlers(app), a cvc::file_handler_registry),
// so two apps never see each other's (VOLROVER plan §1.3). The built-in formats:
//   .pdb, .ent   Protein Data Bank, by its fixed columns (v3.3, and the older files whose columns
//                73-80 hold the entry and a line number);
//   .pqr         PDB2PQR's: an ATOM line's fields by whitespace, charge and radius last;
//   .xyzr        one atom a line: x y z radius.
// Each writes the format back too (write_molecule; write_pdb, write_pqr, write_xyzr).
//
// Everything here holds no state of its own: no statics, no globals.

#ifndef CVC_MOL_MOLECULE_H
#define CVC_MOL_MOLECULE_H

#include <array>
#include <cstddef>
#include <cvc/core/file_handler_registry.h>
#include <cvc/core/param_table.h>
#include <iosfwd>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace cvc {
class app;

namespace mol {

// A file that can't be read: missing, malformed, or of a format no handler serves. The message
// names the file, and the line where it went wrong.
class molecule_file_error : public std::runtime_error {
public:
  explicit molecule_file_error(const std::string &what) : std::runtime_error(what) {}
};

// Which radius the table gives: the van der Waals radius, or the solvent-enlarged one (TexMol's
// RADIUS_TYPE: VDW_RADIUS, ROUX_RADIUS).
enum class radius_kind { van_der_waals, solvent_enlarged };
#ifndef SWIG
BOOST_DESCRIBE_ENUM(radius_kind, van_der_waals, solvent_enlarged)
#endif

// One row of the residue-atom table (TexMol's ElementInformation).
struct residue_atom_info {
  std::string_view atom;         // as TexMol wrote it, blank-padded: " CA "
  std::string_view residue;      // "ALA", "  G"
  float radius;                  // van der Waals, in Å
  float red, green, blue;        // the colour TexMol drew it in
  int hydrophobicity;            // per atom: 1 or -1 (L. Kapcha, P. Rossky)
  double residue_hydrophobicity; // per residue, scaled log P (Black and Mould 1991)
  float interface_propensity;    // per residue (Jones and Thornton 1997)
  float solvent_enlarged_radius; // in Å; 0 for hydrogens
  int residue_index;             // TexMol's residue number, 1 (ALA) to 25 (U)
  float charge;                  // partial charge
};

// The table, all of its rows. Row 0 is TexMol's placeholder (atom "XXXX", residue "XXX").
const residue_atom_info *residue_atom_table(std::size_t *rows);

// The row for an atom and residue name, compared without their blanks (" CA " and "CA" match), or
// -1 when the table has no such pair.
int find_residue_atom(std::string_view atom, std::string_view residue);

struct atom {
  int serial = 0;
  std::string name;    // without blanks: "CA"
  char alt_loc = ' ';  // the alternate location, ' ' for none
  std::string residue; // without blanks: "GLY"
  char chain = ' ';
  int residue_seq = 0;
  char insertion = ' '; // the residue's insertion code
  double x = 0.0, y = 0.0, z = 0.0;
  double occupancy = 1.0;
  double b_factor = 0.0;
  std::string element; // the element symbol: the file's, or the atom name's
  bool hetero = false; // a HETATM record
  int model = 0; // the MODEL it is in, counting from 1 in file order; 0 when the file has none
  double radius = 0.0; // in Å
  double charge = 0.0;
  int table_row = -1; // its residue-atom table row, -1 when the table has none (or a PQR gave both)
};

struct secondary_structure {
  enum class kind { helix, sheet, turn };
  kind type = kind::helix;
  std::string id; // the helix's, sheet's or turn's identifier
  char chain = ' ';
  int start_seq = 0;
  char start_insertion = ' ';
  int end_seq = 0;
  char end_insertion = ' ';
  int helix_class = 0; // a helix: 1 right-handed alpha, ..., 5 right-handed 3-10 (PDB's classes)
  int strand = 0;      // a sheet: the strand's number in it
  int sense = 0;       // a sheet: 0 its first strand, 1 parallel to the one before, -1 anti

  // Whether the residue `seq` (with its insertion code) of `chain` lies in the range.
  bool contains(char chain, int seq, char insertion = ' ') const;
};

struct molecule {
  std::string id;    // the entry's id (HEADER's columns 63-66), when the file has one
  std::string title; // TITLE, its continuation lines joined
  std::vector<atom> atoms;
  std::vector<secondary_structure> structures;
  int models = 0; // how many MODEL records the file had (whether read or not), 0 when none

  bool empty() const { return atoms.empty(); }
  // The atoms' bounding box, xmin ymin zmin xmax ymax zmax: of their centres, or with
  // `with_radii` of their spheres. All zero for no atoms.
  std::array<double, 6> bounds(bool with_radii = false) const;
  // The index of the secondary structure the atom's residue lies in, or -1.
  int structure_of(const atom &a) const;
};

// How read_molecule reads. The defaults read what TexMol's PDBParser kept for its blur maps.
struct molecule_read_params {
  int model = 1;         // which MODEL to read, counting from 1 in file order; 0 reads every one
  bool hetero = true;    // HETATM records
  bool water = true;     // HOH residues (TexMol's deleteWater off)
  bool hydrogens = true; // atoms whose element is H or D
  bool alt_locs = true;  // every alternate location; off keeps ' ' and the first one given
  radius_kind radii = radius_kind::van_der_waals;
  double unknown_radius = 2.0; // for an atom the table hasn't (TexMol's defaultInformation)
};

#ifndef SWIG
} // namespace mol
template <>
struct param_table<mol::molecule_read_params> : param_fields<mol::molecule_read_params> {
  using P = mol::molecule_read_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("model", &P::model, range(0, none),
            doc("which MODEL to read, counting from 1 in file order; 0 reads every model")),
      field("hetero", &P::hetero, doc("read HETATM records")),
      field("water", &P::water, doc("read water (HOH) residues")),
      field("hydrogens", &P::hydrogens, doc("read hydrogen (and deuterium) atoms")),
      field("alt_locs", &P::alt_locs,
            doc("read every alternate location; off keeps the first one an atom gives")),
      field("radii", &P::radii,
            doc("the table's radius: van der Waals or solvent-enlarged (TexMol's Roux)")),
      field("unknown_radius", &P::unknown_radius, unit("Å"), range(0.0, none),
            doc("the radius of an atom the residue-atom table hasn't")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (!(p.unknown_radius > 0))
      problems.push_back("unknown_radius: must be positive");
  }
};
namespace mol {
#endif // SWIG

// One format's reader and writer, as point_class_file_io is (cvc/volume/point_classes.h).
struct molecule_file_io {
  virtual ~molecule_file_io() = default;
  virtual const std::string &id() const = 0;
  virtual const std::vector<std::string> &extensions() const = 0;
  virtual bool can_read() const = 0;
  virtual bool can_write() const { return false; }
  // The molecule in `filename`. Throws molecule_file_error. The default throws "can't read".
  virtual molecule read(app &ctx, const std::string &filename, const molecule_read_params &p) const;
  // Write `m` to `filename`. Throws molecule_file_error. The default throws "can't write".
  virtual void write(app &ctx, const molecule &m, const std::string &filename) const;
};

// The app's molecule formats, the built-in ones registered the first time an app asks. Safe from
// any thread.
file_handler_registry<molecule_file_io> &molecule_handlers(app &ctx);

std::vector<std::string> molecule_read_extensions(app &ctx);
std::vector<std::string> molecule_write_extensions(app &ctx);

// Read the molecule in `filename` with the first of the app's handlers for its extension that
// succeeds. Throws std::invalid_argument when `p` isn't valid, and molecule_file_error when no
// handler can read the file or it holds no atoms.
molecule read_molecule(app &ctx, const std::string &filename, const molecule_read_params &p = {});

// The readers themselves, on a stream; `name` is what an error message calls it. They throw
// molecule_file_error on a malformed line, but not for a molecule with no atoms.
molecule read_pdb(std::istream &in, const molecule_read_params &p = {},
                  const std::string &name = "<pdb>");
molecule read_pqr(std::istream &in, const molecule_read_params &p = {},
                  const std::string &name = "<pqr>");
molecule read_xyzr(std::istream &in, const std::string &name = "<xyzr>");

// Write `m` to `filename` with the first of the app's handlers for its extension that can write.
// Throws molecule_file_error when none can, or the file can't be written.
void write_molecule(app &ctx, const molecule &m, const std::string &filename);

// The writers themselves, on a stream. Each writes what its reader reads back:
//   write_pdb   HEADER (the id), TITLE, HELIX / SHEET / TURN (their residues' names looked up in
//               the atoms), then the atoms as ATOM / HETATM in v3.3's columns (coordinates to
//               0.001 Å, occupancy and B-factor to 0.01, the element in columns 77-78), inside
//               MODEL / ENDMDL when they come from models, and END. A serial past 99,999 overflows
//               its columns, as other writers' do;
//   write_pqr   ATOM / HETATM lines PDB2PQR's way, fields by whitespace: serial, name, residue,
//               chain (left out for none), residue number (with its insertion code), x y z (to
//               0.001 Å), charge and radius (to 0.0001), inside MODEL / ENDMDL as above;
//   write_xyzr  x y z radius a line, each number in its shortest form that reads back exactly.
void write_pdb(std::ostream &out, const molecule &m);
void write_pqr(std::ostream &out, const molecule &m);
void write_xyzr(std::ostream &out, const molecule &m);

} // namespace mol
} // namespace cvc

#endif // CVC_MOL_MOLECULE_H
