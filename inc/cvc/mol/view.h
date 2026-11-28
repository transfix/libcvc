/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol views: what a molecule looks like drawn, as data a renderer uploads (TexMol plan M0
// part 2, docs/roadmap/TEXMOL-F2DOCK-ARISRV-ROADMAP.md §2.4, VOLROVER-TEXMOL-APPS-ROADMAP.md §6).
// cvcGL's `molecule` scene node draws one; pycvc and a headless tool can build one too and write
// its meshes out.
//
//   cvc::mol::view_params p;
//   p.style = cvc::mol::view_style::cartoon;
//   p.color_by = cvc::mol::color_scheme::structure;
//   cvc::mol::molecule_view v = cvc::mol::build_view(app, m, p);
//   draw_spheres(v.sphere_atoms, v.centers, v.radii, v.colors);    // impostors, or v.spheres
//   draw_mesh(v.sticks); draw_mesh(v.cartoon);
//
// The styles:
//   ball_and_stick  each atom a sphere of atom_scale times its radius, each bond a cylinder of
//                   bond_radius, half in each atom's colour;
//   spacefill       each atom a sphere of its radius (van der Waals, or what the file gave);
//   sticks          the bonds alone, with a sphere of bond_radius at each atom to round the joints;
//   cartoon         each chain's trace (C-alphas of amino acids, phosphorus of nucleotides) as a
//                   smooth tube, wide and flat through the file's HELIX ranges, flat with an arrow
//                   at the end through its SHEET strands.
//
// Bonds. A PDB file lists few bonds (CONECT records only for ligands), so they are found by
// distance: two atoms of one model are bonded when they lie closer than the sum of their elements'
// covalent radii plus bond_tolerance (and farther than 0.4 Å). Atoms in different alternate
// locations never bond, and metal ions bond to nothing.
//
// Colours. By element (each element its own colour, carbon grey), chain (a palette, in the order
// the chains come in the file), residue (each residue its own colour), B-factor (blue through
// white to red over b_range, by default the drawn atoms' range) or secondary structure (helix,
// sheet, coil). A cartoon takes its residue's colour from the residue's trace atom.
//
// build_view_stepper builds it a slice at a time (VOLROVER plan §1.9), so a large assembly never
// stalls a frame: phases "selecting" (a unit per atom), "bonding" (a unit per atom), "spheres" (a
// unit per sphere, when tessellated spheres are asked for), "sticks" (a unit per bond) and
// "cartoon" (a unit per residue). Each unit is a few microseconds or less.
//
// The colours, radii and palettes here are written for libcvc. Nothing comes from TexMol's
// renderers, libmol or its element table (TexMol plan §2.4 and T0). Everything here holds no state
// of its own: no statics, no globals.

#ifndef CVC_MOL_VIEW_H
#define CVC_MOL_VIEW_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/core/param_table.h>
#include <cvc/core/result_job.h>
#include <cvc/geometry/geometry.h>
#include <cvc/mol/molecule.h>
#include <limits>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace cvc {
class app;

namespace mol {

enum class view_style { ball_and_stick, spacefill, sticks, cartoon };
enum class color_scheme { element, chain, residue, b_factor, structure };
// Which atoms are drawn: all of them, the polymer's (ATOM records), or the rest (HETATM: ligands,
// ions, water).
enum class atom_set { all, polymer, hetero };
#ifndef SWIG
BOOST_DESCRIBE_ENUM(view_style, ball_and_stick, spacefill, sticks, cartoon)
BOOST_DESCRIBE_ENUM(color_scheme, element, chain, residue, b_factor, structure)
BOOST_DESCRIBE_ENUM(atom_set, all, polymer, hetero)
#endif

struct view_params {
  view_style style = view_style::ball_and_stick;
  color_scheme color_by = color_scheme::element;
  atom_set atoms = atom_set::all;
  bool hydrogens = true;         // draw hydrogen (and deuterium) atoms
  bool water = true;             // draw water (HOH) residues
  double atom_scale = 0.25;      // ball_and_stick: a sphere's radius, times the atom's radius
  double bond_radius = 0.15;     // Å; sticks: also the joints' spheres
  double bond_tolerance = 0.45;  // Å past the covalent radii's sum that two atoms still bond
  double helix_width = 2.4;      // Å, a cartoon's helix ribbon across
  double sheet_width = 2.0;      // Å, a cartoon's strand across (its arrow 1.6 times that)
  double ribbon_thickness = 0.5; // Å, both
  double coil_radius = 0.3;      // Å, a cartoon's tube between them
  int subdivisions = 8;          // a cartoon's samples between two residues
  int sides = 12;                // a cartoon's ring: vertices round its cross-section
  int sphere_detail = 1;         // tessellated spheres: an icosahedron subdivided this often
  bool sphere_mesh = false;      // make tessellated spheres (`spheres`) as well as the impostors'
  std::vector<double> b_range;   // b_factor colours: [blue, red]; empty for the drawn atoms' range
  // Chains not drawn, each a character of the string (' ' for atoms with none). Not a state key:
  // cvcGL's node keeps a visible key per chain and fills this from them.
  std::string hidden_chains;
};

} // namespace mol

#ifndef SWIG
template <> struct param_table<mol::view_params> : param_fields<mol::view_params> {
  using P = mol::view_params;
  static constexpr double none = std::numeric_limits<double>::quiet_NaN();
  static constexpr auto fields = std::tuple{
      field("style", &P::style, doc("ball_and_stick, spacefill, sticks or cartoon")),
      field("color_by", &P::color_by, doc("element, chain, residue, b_factor or structure")),
      field("atoms", &P::atoms, doc("which atoms: all, polymer (ATOM) or hetero (HETATM)")),
      field("hydrogens", &P::hydrogens, doc("draw hydrogen atoms")),
      field("water", &P::water, doc("draw water (HOH) residues")),
      field("atom_scale", &P::atom_scale, range(0.01, 2.0),
            doc("ball_and_stick: a sphere's radius as a fraction of the atom's")),
      field("bond_radius", &P::bond_radius, unit("Å"), range(0.01, 2.0),
            doc("a bond cylinder's radius")),
      field("bond_tolerance", &P::bond_tolerance, unit("Å"), range(0.0, 2.0),
            doc("how far past the covalent radii's sum two atoms still bond")),
      field("helix_width", &P::helix_width, unit("Å"), range(0.1, 10.0),
            doc("a cartoon helix's width")),
      field("sheet_width", &P::sheet_width, unit("Å"), range(0.1, 10.0),
            doc("a cartoon strand's width (its arrow 1.6 times that)")),
      field("ribbon_thickness", &P::ribbon_thickness, unit("Å"), range(0.01, 5.0),
            doc("a cartoon helix's and strand's thickness")),
      field("coil_radius", &P::coil_radius, unit("Å"), range(0.01, 5.0),
            doc("a cartoon coil's tube radius")),
      field("subdivisions", &P::subdivisions, range(1, 32),
            doc("a cartoon's samples between two residues")),
      field("sides", &P::sides, range(4, 64), doc("a cartoon's vertices round a cross-section")),
      field("sphere_detail", &P::sphere_detail, range(0, 4),
            doc("tessellated spheres: icosahedron subdivisions (0: 20 faces, 1: 80, 2: 320)")),
      field("sphere_mesh", &P::sphere_mesh,
            doc("make tessellated spheres too, where impostors can't be drawn")),
      field("b_range", &P::b_range,
            doc("b_factor colours: [blue, red] in Å^2; empty for the drawn atoms' range")),
  };
  static void check(const P &p, std::vector<std::string> &problems) {
    if (!p.b_range.empty() && (p.b_range.size() != 2 || !(p.b_range[0] < p.b_range[1])))
      problems.push_back("b_range: two values, the first below the second, or none");
  }
};
#endif // SWIG

namespace mol {

using rgb8 = std::array<std::uint8_t, 3>;

// The palettes. An element is matched without regard to case ("FE", "Fe"); one the table lacks
// is pink. A chain's colour depends on where it comes among the molecule's chains (chain_order).
rgb8 element_color(const std::string &element);
rgb8 chain_color(std::size_t chain_order);
rgb8 residue_color(const std::string &residue);
rgb8 structure_color(int kind); // -1 coil, else secondary_structure::kind as an int
// t in [0, 1]: blue, white, red.
rgb8 b_factor_color(double t);

// An element's covalent radius in Å (a single bond), 0.77 for one the table lacks.
double covalent_radius(const std::string &element);
// Whether atoms of `element` are drawn without bonds (metal ions).
bool is_metal(const std::string &element);

// Each atom's colour under `scheme`. b_range as in view_params (empty: the atoms' range).
std::vector<rgb8> atom_colors(const molecule &m, color_scheme scheme,
                              const std::vector<double> &b_range = {});

// The molecule's chains, each once, in the order the file first gives them.
std::string chain_ids(const molecule &m);

struct bond {
  int a = 0, b = 0; // atom indices, a < b
};

// Every bond by distance (see the top), in order of their first atom, then their second. A
// cell grid makes it linear in the atoms.
std::vector<bond> find_bonds(const molecule &m, double tolerance = 0.45);

// One residue of a chain's trace, as a cartoon draws it.
struct trace_residue {
  int atom = -1;      // its trace atom (CA, or P): an index into the molecule's atoms
  int guide = -1;     // its carbonyl oxygen (O), -1 for none
  int structure = -1; // the secondary structure it lies in (an index into structures), or -1
  bool start = false; // it starts a new piece of trace (a chain, a model, or a gap)
};

// The trace a cartoon follows: per model, per chain, every amino acid's C-alpha and every
// nucleotide's phosphorus, in file order, a new piece starting wherever two neighbours lie
// farther apart than a peptide (4.2 Å between C-alphas) or a nucleotide (8 Å) allows.
std::vector<trace_residue> chain_trace(const molecule &m);

// What a molecule looks like drawn.
struct molecule_view {
  // The spheres (ball_and_stick, spacefill, sticks): for each, its atom (an index into the
  // molecule's atoms), its centre (three floats), radius and colour. What impostors draw.
  std::vector<int> sphere_atoms;
  std::vector<float> centers;
  std::vector<float> radii;
  std::vector<rgb8> colors;
  // The same spheres tessellated, when sphere_mesh asked for them (else empty): triangles with
  // normals and colours.
  geometry spheres;
  // The bonds drawn (between drawn atoms), and their cylinders, half in each atom's colour.
  std::vector<bond> bonds;
  geometry sticks;
  // The cartoon's tube: triangles with normals and colours.
  geometry cartoon;
  // Per cartoon vertex, the residue's trace atom it belongs to (for picking).
  std::vector<int> cartoon_atoms;

  std::array<double, 6> bounds{}; // of everything drawn, radii included; zero for nothing
  std::size_t atoms_drawn = 0;
  std::size_t residues = 0; // the trace's residues (cartoon) or the drawn atoms' residues
  std::string chains;       // every chain of the molecule, as chain_ids gives them
  std::vector<std::string> warnings;
};

// A view a slice at a time. A failure (parameters that aren't valid) is a std::invalid_argument
// thrown from step() and rethrown by later steps and take(). cancel() is the one thread-safe
// member. Not thread-safe otherwise, but it may move between threads between steps. The molecule
// is shared, read-only, for as long as the stepper lives.
class build_view_stepper {
public:
  build_view_stepper(app &ctx, std::shared_ptr<const molecule> m, view_params params = {});
  ~build_view_stepper();
  build_view_stepper(build_view_stepper &&) noexcept;
  build_view_stepper &operator=(build_view_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  // "selecting", "bonding", "spheres", "sticks", "cartoon", "done", "failed" or "cancelled"
  const char *phase() const;
  double progress() const; // in [0, 1], monotone; 1 once done
  std::string status() const;
  void cancel();
  molecule_view take(); // once done; moved out, call it once

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// The view at once: build_view_stepper run to the end. The overload taking a molecule by
// reference copies it.
molecule_view build_view(app &ctx, std::shared_ptr<const molecule> m, const view_params &p = {});
molecule_view build_view(app &ctx, const molecule &m, const view_params &p = {});

std::unique_ptr<result_job<molecule_view>>
build_view_job(app &ctx, std::shared_ptr<const molecule> m, const view_params &p = {});

// The atom a world ray first meets among a view's spheres: origin + t dir with t >= 0 and the
// least such t. -1 when it meets none. `t` and `hit` (three doubles) say where, when not null.
int pick_sphere(const molecule_view &v, const double origin[3], const double dir[3],
                double *t = nullptr, double *hit = nullptr);

// The drawn atom nearest a point (a spheres' or trace atom for a cartoon), or -1 for none drawn.
int nearest_drawn_atom(const molecule &m, const molecule_view &v, const double p[3]);

} // namespace mol
} // namespace cvc

#endif // CVC_MOL_VIEW_H
