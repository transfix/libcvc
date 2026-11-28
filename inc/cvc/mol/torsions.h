/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's torsion angles: each residue's backbone dihedrals (TexMol plan M0 part 2; PDBParser's
// GOAMolGeometry), in radians, (-pi, pi], with IUPAC's sign (positive is clockwise, looking from
// the second atom to the third).
//
//   for (const cvc::mol::residue_torsions &r : cvc::mol::torsions(m))
//     if (r.type == cvc::mol::residue_torsions::kind::amino_acid) use(r.phi, r.psi);
//
// An amino acid (a residue with N, CA and C) has
//   phi    C(i-1) N  CA C          psi   N  CA C  N(i+1)        omega  CA(i-1) C(i-1) N CA
// and a nucleotide the angles PDBParser computed (atom names with ' or *: O5' or O5*):
//   alpha   O3'(i-1) P   O5'  C5'        beta   P   O5'  C5'  C4'
//   gamma   O5'  C5'  C4'  C3'           delta  C5'  C4'  C3'  O3'
//   epsilon C4'  C3'  O3'  P(i+1)        zeta   C3'  O3'  P(i+1) O5'(i+1)
//   nu0..4  the sugar ring: C4' O4' C1' C2', O4' C1' C2' C3', C1' C2' C3' C4', C2' C3' C4' O4',
//           C3' C4' O4' C1'
//   chi     O4' C1' N9 C4 (a purine) or O4' C1' N1 C2 (a pyrimidine)
//   eta     C4'(i-1) P  C4'  P(i+1)       theta  P   C4'  P(i+1) C4'(i+1)
// An angle no atoms define is NaN: at a chain's end, across a gap, or where an atom is missing.
//
// The neighbours i-1 and i+1 are the residues before and after in the file, in the same model and
// chain, and only when bonded to it: C(i-1) to N within 2 Å for amino acids, O3'(i-1) to P for
// nucleotides. PDBParser instead took the residue before or after in its group tree, which puts a
// sheet's strands together, so at a strand's ends it measured across to another strand (1GRL's
// residue 48 took 40 as its previous residue); and it wrote 0 for an angle it could not measure.
// molecule_groups_test checks these against TexMol's own angles where its neighbours are the
// real ones.
//
// torsion_stepper measures them a slice at a time (a unit per residue). Everything here holds no
// state of its own.

#ifndef CVC_MOL_TORSIONS_H
#define CVC_MOL_TORSIONS_H

#include <array>
#include <cstddef>
#include <cvc/core/result_job.h>
#include <cvc/mol/molecule.h>
#include <memory>
#include <string>
#include <vector>

namespace cvc {
namespace mol {

struct residue_torsions {
  enum class kind { amino_acid, nucleotide, other };
  kind type = kind::other;
  std::size_t first_atom = 0; // the residue's first atom, an index into the molecule's
  int model = 0;
  char chain = ' ';
  int residue_seq = 0;
  char insertion = ' ';
  std::string residue;
  double phi, psi, omega;                               // amino acids
  double alpha, beta, gamma, delta, epsilon, zeta, chi; // nucleotides
  std::array<double, 5> nu;                             // nucleotides: the sugar ring
  double eta, theta;                                    // nucleotides: pseudo-torsions
  residue_torsions();
};

// The dihedral of four points, radians in (-pi, pi], IUPAC's sign; NaN when three are collinear.
double dihedral(const std::array<double, 3> &a, const std::array<double, 3> &b,
                const std::array<double, 3> &c, const std::array<double, 3> &d);

// Each residue's torsions, a slice at a time. The molecule is shared, read-only, for as long as the
// stepper lives. cancel() is the one thread-safe member.
class torsion_stepper {
public:
  explicit torsion_stepper(std::shared_ptr<const molecule> m);
  ~torsion_stepper();
  torsion_stepper(torsion_stepper &&) noexcept;
  torsion_stepper &operator=(torsion_stepper &&) noexcept;

  bool step(std::size_t budget); // true once done; budget 0 runs to the end
  bool done() const;
  const char *phase() const; // "residues", "measuring", "done" or "cancelled"
  double progress() const;
  void cancel();
  std::vector<residue_torsions> take(); // once done; one a residue, in file order

private:
  struct impl;
  std::unique_ptr<impl> _impl;
};

// Every residue's torsions at once: torsion_stepper run to the end.
std::vector<residue_torsions> torsions(const molecule &m);
std::unique_ptr<result_job<std::vector<residue_torsions>>>
torsions_job(std::shared_ptr<const molecule> m);

} // namespace mol
} // namespace cvc

#endif // CVC_MOL_TORSIONS_H
