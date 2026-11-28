/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's group tree and torsion angles (cvc/mol/groups.h, cvc/mol/torsions.h; TexMol plan M0
// part 2):
//   - T1, the A/B against TexMol's PDBParser (tools/texmol-goldens, parser_legacy on 1GRL): the
//     group tree, line for line; the atom order its walk gives (blur's 1grl.atoms.txt); and phi
//     and psi on every residue where TexMol's neighbours are the residue's real ones, and where
//     they are not, that TexMol's angle is the one to its wrong neighbour;
//   - the tree's rules on small molecules: helices and turns in file order, a sheet's strands in
//   one
//     group, a helix's range kept out of sheets, `none` groups, structures in the first model
//     only, chains and nucleic chains (T2, T3);
//   - dihedrals, the angles' atoms, ends and gaps, nucleotides (T2, T3);
//   - on 1GRL, the angles' shape: trans peptide bonds, helices' and strands' phi and psi (T4);
//   - budgets 1, 7 and 0 give the same tree and angles; a cancel; two threads (T6, T7).

#include <algorithm>
#include <cmath>
#include <cvc/mol/groups.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/torsions.h>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace mol = cvc::mol;

namespace {

const std::string kRepo = CVC_REPO_DIR;
const std::string k1grl = kRepo + "/examples/data/molecules/1grl.pdb";
const std::string kGoldens = kRepo + "/src/cvc/tests/data/texmol-goldens";
constexpr double kPi = 3.14159265358979323846;

mol::molecule grl() {
  std::ifstream in(k1grl, std::ios::binary);
  EXPECT_TRUE(in.good()) << k1grl;
  return mol::read_pdb(in, {}, k1grl);
}

std::vector<std::string> lines_of(const std::string &path) {
  std::ifstream in(path, std::ios::binary);
  EXPECT_TRUE(in.good()) << path;
  std::vector<std::string> out;
  for (std::string l; std::getline(in, l);) {
    if (!l.empty() && l.back() == '\r')
      l.pop_back();
    out.push_back(l);
  }
  return out;
}

std::vector<std::string> split(const std::string &s) {
  std::istringstream in(s);
  std::vector<std::string> t;
  for (std::string w; in >> w;)
    t.push_back(w);
  return t;
}

mol::atom make_atom(const std::string &name, double x, double y, double z, char chain, int seq,
                    const std::string &residue = "ALA", int model = 0) {
  mol::atom a;
  a.name = name;
  a.residue = residue;
  a.chain = chain;
  a.residue_seq = seq;
  a.model = model;
  a.x = x;
  a.y = y;
  a.z = z;
  a.element = name.substr(0, 1);
  return a;
}

// One residue a seq, each its own CA, along x.
mol::molecule residues(char chain, int from, int to, int model = 0) {
  mol::molecule m;
  for (int s = from; s <= to; ++s)
    m.atoms.push_back(make_atom("CA", 3.8 * s, 0, 0, chain, s, "ALA", model));
  return m;
}

mol::secondary_structure record(mol::secondary_structure::kind k, char chain, int from, int to,
                                int strand = 0) {
  mol::secondary_structure s;
  s.type = k;
  s.chain = chain;
  s.start_seq = from;
  s.end_seq = to;
  s.strand = strand;
  return s;
}

// The tree's structure groups under model `mi`'s chain `ci`: their kinds and residue numbers.
std::vector<std::pair<std::string, std::vector<int>>>
structures_of(const mol::group_tree &t, std::size_t mi = 0, std::size_t ci = 0) {
  std::vector<std::pair<std::string, std::vector<int>>> out;
  const mol::group &model = t.groups[t.groups[0].children.at(mi)];
  const mol::group &chain = t.groups[model.children.at(ci)];
  for (std::size_t s : chain.children) {
    const mol::group &g = t.groups[s];
    const char *k = g.structure == mol::group::structure_kind::helix   ? "helix"
                    : g.structure == mol::group::structure_kind::sheet ? "sheet"
                    : g.structure == mol::group::structure_kind::turn  ? "turn"
                                                                       : "none";
    std::vector<int> seqs;
    for (std::size_t r : g.children)
      seqs.push_back(t.groups[r].residue_seq);
    out.push_back({k, seqs});
  }
  return out;
}

using structures = std::vector<std::pair<std::string, std::vector<int>>>;

} // namespace

// ---- T1: TexMol's own tree and angles on 1GRL ----

TEST(MolGroupsLegacy, TheTreeIsTexMols) {
  const mol::group_tree t = mol::build_group_tree(grl());
  const std::vector<std::string> legacy = lines_of(kGoldens + "/parser/1grl.tree.txt");
  std::vector<std::string> ours;
  {
    std::istringstream in(mol::describe(t));
    for (std::string l; std::getline(in, l);)
      ours.push_back(l);
  }
  ASSERT_EQ(ours.size(), legacy.size());
  for (std::size_t i = 0; i < ours.size(); ++i) {
    // A residue's line ends with TexMol's phi and psi, which describe() leaves out.
    std::string want = legacy[i];
    if (want.find("residue ") != std::string::npos)
      for (int k = 0; k < 2; ++k)
        want = want.substr(0, want.find_last_of(' '));
    ASSERT_EQ(ours[i], want) << "line " << i + 1;
  }
}

TEST(MolGroupsLegacy, TheWalkGivesTexMolsAtomOrder) {
  const mol::molecule m = grl();
  const std::vector<std::size_t> order = mol::flatten_atoms(mol::build_group_tree(m));
  const std::vector<std::string> legacy = lines_of(kGoldens + "/blur/1grl.atoms.txt");
  ASSERT_EQ(order.size(), m.atoms.size());
  ASSERT_EQ(legacy.size(), order.size());
  std::size_t moved = 0;
  for (std::size_t i = 0; i < order.size(); ++i) {
    const std::vector<std::string> t = split(legacy[i]);
    ASSERT_EQ(t.size(), 4u);
    const mol::atom &a = m.atoms[order[i]];
    // TexMol kept float positions.
    EXPECT_NEAR(a.x, std::stod(t[0]), 1e-4) << "atom " << i;
    EXPECT_NEAR(a.y, std::stod(t[1]), 1e-4) << "atom " << i;
    EXPECT_NEAR(a.z, std::stod(t[2]), 1e-4) << "atom " << i;
    moved += order[i] != i ? 1 : 0;
  }
  EXPECT_GT(moved, 0u); // the sheets' strands came together
}

TEST(MolGroupsLegacy, PhiAndPsiAreTexMolsWhereItsNeighboursAreReal) {
  const mol::molecule m = grl();
  const mol::group_tree t = mol::build_group_tree(m);
  const std::vector<mol::residue_torsions> ours = mol::torsions(m);
  std::map<std::tuple<char, int, char>, const mol::residue_torsions *> by_key;
  for (const mol::residue_torsions &r : ours)
    by_key[{r.chain, r.residue_seq, r.insertion}] = &r;
  // The residues in TexMol's tree order, with its angles.
  struct legacy_residue {
    char chain, insertion;
    int seq;
    double phi, psi;
  };
  std::vector<legacy_residue> legacy;
  for (const std::string &l : lines_of(kGoldens + "/parser/1grl.tree.txt")) {
    const std::vector<std::string> tk = split(l);
    if (tk.empty() || tk[0] != "residue")
      continue;
    const auto ch = [](const std::string &s) { return s == "_" ? ' ' : s[0]; };
    legacy.push_back({ch(tk[1]), ch(tk[3]), std::stoi(tk[2]), std::stod(tk[5]), std::stod(tk[6])});
  }
  // The residue groups by key, to measure TexMol's angle to its own neighbour.
  std::map<std::tuple<char, int, char>, const mol::group *> group_of;
  for (const mol::group &g : t.groups)
    if (g.type == mol::group::kind::residue)
      group_of[{g.chain, g.residue_seq, g.insertion}] = &g;
  const auto atom_named = [&](const legacy_residue &r, const char *name) -> const mol::atom * {
    const mol::group *g = group_of.at({r.chain, r.seq, r.insertion});
    for (std::size_t i : g->atoms)
      if (m.atoms[i].name == name)
        return &m.atoms[i];
    return nullptr;
  };
  const auto pt = [](const mol::atom *a) { return std::array<double, 3>{a->x, a->y, a->z}; };
  std::size_t same = 0, wrong_neighbour = 0, unmeasured = 0;
  for (std::size_t i = 0; i < legacy.size(); ++i) {
    const legacy_residue &r = legacy[i];
    const mol::residue_torsions *o = by_key.at({r.chain, r.seq, r.insertion});
    // phi: TexMol's previous residue is the one before in its tree (none for the first).
    if (i > 0) {
      const legacy_residue &p = legacy[i - 1];
      const bool real = p.chain == r.chain && p.seq == r.seq - 1;
      if (!std::isnan(o->phi) && real) {
        EXPECT_NEAR(o->phi, r.phi, 2e-5) << "phi of " << r.seq;
        ++same;
      } else if (!real) {
        const mol::atom *pc = atom_named(p, "C"), *n = atom_named(r, "N"),
                        *ca = atom_named(r, "CA"), *c = atom_named(r, "C");
        if (pc && n && ca && c) {
          EXPECT_NEAR(mol::dihedral(pt(pc), pt(n), pt(ca), pt(c)), r.phi, 2e-5)
              << "TexMol's phi of " << r.seq << " is to residue " << p.seq;
          ++wrong_neighbour;
        } else {
          EXPECT_EQ(r.phi, 0.0) << "TexMol's unmeasured phi of " << r.seq;
          ++unmeasured;
        }
      } else {
        EXPECT_EQ(r.phi, 0.0) << "an atom missing: TexMol wrote 0 for " << r.seq;
        ++unmeasured;
      }
    }
    if (i + 1 < legacy.size()) {
      const legacy_residue &n = legacy[i + 1];
      const bool real = n.chain == r.chain && n.seq == r.seq + 1;
      if (!std::isnan(o->psi) && real) {
        EXPECT_NEAR(o->psi, r.psi, 2e-5) << "psi of " << r.seq;
        ++same;
      } else if (!real) {
        ++wrong_neighbour;
      } else {
        EXPECT_EQ(r.psi, 0.0) << "an atom missing: TexMol wrote 0 for " << r.seq;
        ++unmeasured;
      }
    }
  }
  EXPECT_GT(same, 850u); // phi and psi of 459 residues with a backbone, less the strands' ends
  EXPECT_GT(wrong_neighbour, 10u);
  RecordProperty("same", static_cast<int>(same));
  RecordProperty("wrong_neighbour", static_cast<int>(wrong_neighbour));
  RecordProperty("unmeasured", static_cast<int>(unmeasured));
}

// ---- the tree's rules ----

TEST(MolGroups, HelicesTurnsAndSheets) {
  mol::molecule m = residues('A', 1, 14);
  using K = mol::secondary_structure::kind;
  m.structures.push_back(record(K::helix, 'A', 2, 4));
  m.structures.push_back(record(K::turn, 'A', 6, 6));
  // A sheet of two strands: 8-9 and 12-13. Strand 2 joins strand 1's sheet.
  m.structures.push_back(record(K::sheet, 'A', 8, 9, 1));
  m.structures.push_back(record(K::sheet, 'A', 12, 13, 2));
  const mol::group_tree t = mol::build_group_tree(m);
  EXPECT_EQ(structures_of(t), (structures{{"none", {1}},
                                          {"helix", {2, 3, 4}},
                                          {"none", {5}},
                                          {"turn", {6}},
                                          {"none", {7}},
                                          {"sheet", {8, 9, 12, 13}},
                                          {"none", {10, 11}},
                                          {"none", {14}}}));
  // The walk takes the sheet's strands together.
  std::vector<int> walk;
  for (std::size_t i : mol::flatten_atoms(t))
    walk.push_back(m.atoms[i].residue_seq);
  EXPECT_EQ(walk, (std::vector<int>{1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 10, 11, 14}));
}

TEST(MolGroups, AHelixWinsOverASheetAndRecordsGoInOrder) {
  using K = mol::secondary_structure::kind;
  mol::molecule m = residues('A', 1, 8);
  m.structures.push_back(record(K::helix, 'A', 2, 4));
  m.structures.push_back(record(K::sheet, 'A', 3, 6, 1)); // overlaps the helix
  const mol::group_tree t = mol::build_group_tree(m);
  EXPECT_EQ(structures_of(t),
            (structures{{"none", {1}}, {"helix", {2, 3, 4}}, {"sheet", {5, 6}}, {"none", {7, 8}}}));
  // A helix listed before one that comes earlier in the chain: PDBParser matched them in file
  // order, so the first listed is seen and the other never is.
  mol::molecule n = residues('A', 1, 8);
  n.structures.push_back(record(K::helix, 'A', 6, 7));
  n.structures.push_back(record(K::helix, 'A', 2, 3));
  EXPECT_EQ(structures_of(mol::build_group_tree(n)),
            (structures{{"none", {1, 2, 3, 4, 5}}, {"helix", {6, 7}}, {"none", {8}}}));
}

TEST(MolGroups, ChainsModelsAndNucleicAcids) {
  using K = mol::secondary_structure::kind;
  mol::molecule m = residues('A', 1, 3, 1);
  for (const mol::atom &a : residues('B', 1, 2, 1).atoms)
    m.atoms.push_back(a);
  for (const mol::atom &a : residues('A', 1, 3, 2).atoms)
    m.atoms.push_back(a);
  m.structures.push_back(record(K::helix, 'A', 1, 2));
  m.atoms.push_back(make_atom("P", 0, 5, 0, 'R', 1, "U", 2));
  const mol::group_tree t = mol::build_group_tree(m);
  ASSERT_EQ(t.groups[0].children.size(), 2u); // two models
  const mol::group &m1 = t.groups[t.groups[0].children[0]];
  const mol::group &m2 = t.groups[t.groups[0].children[1]];
  ASSERT_EQ(m1.children.size(), 2u);
  EXPECT_EQ(t.groups[m1.children[1]].chain, 'B');
  EXPECT_EQ(structures_of(t, 0, 0), (structures{{"helix", {1, 2}}, {"none", {3}}}));
  // Model 2: no structures (PDBParser forgot them), and a nucleic chain.
  ASSERT_EQ(m2.children.size(), 2u);
  EXPECT_EQ(structures_of(t, 1, 0), (structures{{"none", {1, 2, 3}}}));
  EXPECT_FALSE(t.groups[m2.children[0]].nucleic);
  EXPECT_TRUE(t.groups[m2.children[1]].nucleic);
  // The walk is of the first model only.
  EXPECT_EQ(mol::flatten_atoms(t).size(), 5u);
  // A chain id that comes back starts another chain.
  mol::molecule back = residues('A', 1, 1);
  back.atoms.push_back(make_atom("CA", 0, 0, 0, 'B', 1));
  back.atoms.push_back(make_atom("CA", 0, 0, 0, 'A', 2));
  EXPECT_EQ(t.groups.empty(), false);
  EXPECT_EQ(mol::build_group_tree(back).groups[1].children.size(), 3u);
  // No atoms: an empty tree.
  EXPECT_TRUE(mol::build_group_tree(mol::molecule()).groups.empty());
  EXPECT_TRUE(mol::flatten_atoms(mol::group_tree()).empty());
}

// ---- torsions ----

TEST(MolTorsions, Dihedrals) {
  using P = std::array<double, 3>;
  const P a{1, 0, 0}, b{0, 0, 0}, c{0, 0, 1};
  // d turned about the b-c axis (z) from a's direction, anticlockwise seen from above: clockwise
  // seen from b towards c, so positive (IUPAC's sign, which TexMol's angles also have).
  for (const double deg : {0.0, 60.0, -60.0, 120.0, -150.0, 179.0}) {
    const double r = deg * kPi / 180.0;
    const P d{std::cos(r), std::sin(r), 1};
    EXPECT_NEAR(mol::dihedral(a, b, c, d) * 180.0 / kPi, deg, 1e-9) << deg;
  }
  EXPECT_NEAR(std::abs(mol::dihedral(a, b, c, P{-1, 0, 1})), kPi, 1e-12);
  EXPECT_TRUE(std::isnan(mol::dihedral(a, b, P{2, 0, 0}, c))); // three collinear
}

TEST(MolTorsions, AminoAcidsEndsAndGaps) {
  // Three residues of an ideal-ish backbone, then a fourth far away (a gap), then a residue with
  // no C (unmeasured).
  mol::molecule m;
  const auto add = [&](int seq, double x0) {
    m.atoms.push_back(make_atom("N", x0, 0.5, 0, 'A', seq));
    m.atoms.push_back(make_atom("CA", x0 + 1.2, 0, 0.3, 'A', seq));
    m.atoms.push_back(make_atom("C", x0 + 2.4, 0.6, -0.2, 'A', seq));
  };
  add(1, 0);
  add(2, 3.7);
  add(3, 7.4);
  add(4, 40);
  m.atoms.push_back(make_atom("N", 50, 0, 0, 'A', 5));
  const std::vector<mol::residue_torsions> t = mol::torsions(m);
  ASSERT_EQ(t.size(), 5u);
  using K = mol::residue_torsions::kind;
  EXPECT_EQ(t[0].type, K::amino_acid);
  EXPECT_TRUE(std::isnan(t[0].phi)); // a chain's start
  EXPECT_FALSE(std::isnan(t[0].psi));
  EXPECT_FALSE(std::isnan(t[1].phi));
  EXPECT_FALSE(std::isnan(t[1].omega));
  EXPECT_TRUE(std::isnan(t[2].psi)); // residue 4 is too far to be bonded
  EXPECT_TRUE(std::isnan(t[3].phi));
  EXPECT_EQ(t[4].type, K::other); // no CA, no C
  // The angles are the dihedrals of their atoms.
  const auto p = [&](std::size_t i) {
    return std::array<double, 3>{m.atoms[i].x, m.atoms[i].y, m.atoms[i].z};
  };
  EXPECT_DOUBLE_EQ(t[1].phi, mol::dihedral(p(2), p(3), p(4), p(5)));
  EXPECT_DOUBLE_EQ(t[1].psi, mol::dihedral(p(3), p(4), p(5), p(6)));
  EXPECT_DOUBLE_EQ(t[1].omega, mol::dihedral(p(1), p(2), p(3), p(4)));
  EXPECT_EQ(t[1].residue_seq, 2);
  EXPECT_EQ(t[1].first_atom, 3u);
}

TEST(MolTorsions, Nucleotides) {
  // Two uridines, the second's P bonded to the first's O3' (written O3*, the older way), each atom
  // where only its angles matter.
  mol::molecule m;
  const auto add = [&](int seq, double x0, const char *o3) {
    for (const auto &[name, x, y, z] :
         std::vector<std::tuple<std::string, double, double, double>>{{"P", 0, 0, 0},
                                                                      {"O5'", 1, 1, 0},
                                                                      {"C5'", 2, 1, 1},
                                                                      {"C4'", 3, 2, 1},
                                                                      {"C3'", 3, 3, 2},
                                                                      {o3, 4, 4, 2},
                                                                      {"O4'", 4, 2, 0},
                                                                      {"C1'", 5, 2, 1},
                                                                      {"C2'", 4.5, 3, 2},
                                                                      {"N1", 6, 2, 1.5},
                                                                      {"C2", 6.5, 3, 1}})
      m.atoms.push_back(make_atom(name, x0 + x, y, z, 'R', seq, "U"));
  };
  add(1, 0, "O3*");
  add(2, 4.6, "O3'"); // its P lies 0.6 Å along x from the first's O3*, at (4.6, 0, 0) vs (4, 4, 2)
  // Put the second's P within bonding reach of the first's O3*.
  m.atoms[11].x = 4.5;
  m.atoms[11].y = 4.8;
  m.atoms[11].z = 2.6;
  const std::vector<mol::residue_torsions> t = mol::torsions(m);
  ASSERT_EQ(t.size(), 2u);
  using K = mol::residue_torsions::kind;
  ASSERT_EQ(t[0].type, K::nucleotide);
  ASSERT_EQ(t[1].type, K::nucleotide);
  const auto p = [&](std::size_t i) {
    return std::array<double, 3>{m.atoms[i].x, m.atoms[i].y, m.atoms[i].z};
  };
  EXPECT_TRUE(std::isnan(t[0].alpha)); // no residue before
  EXPECT_DOUBLE_EQ(t[0].beta, mol::dihedral(p(0), p(1), p(2), p(3)));
  EXPECT_DOUBLE_EQ(t[0].epsilon, mol::dihedral(p(3), p(4), p(5), p(11)));
  EXPECT_DOUBLE_EQ(t[0].zeta, mol::dihedral(p(4), p(5), p(11), p(12)));
  EXPECT_DOUBLE_EQ(t[0].nu[0], mol::dihedral(p(3), p(6), p(7), p(8)));
  EXPECT_DOUBLE_EQ(t[0].chi, mol::dihedral(p(6), p(7), p(9), p(10))); // a pyrimidine: N1, C2
  EXPECT_DOUBLE_EQ(t[1].alpha, mol::dihedral(p(5), p(11), p(12), p(13)));
  EXPECT_TRUE(std::isnan(t[1].eta)); // eta needs a next P: none for the last
  EXPECT_TRUE(std::isnan(t[1].epsilon));
  EXPECT_FALSE(std::isnan(t[0].theta));
}

TEST(MolTorsions, OneGrlsShape) {
  const mol::molecule m = grl();
  const std::vector<mol::residue_torsions> t = mol::torsions(m);
  std::size_t peptides = 0, trans = 0, helix = 0, helical = 0, sheet = 0, extended = 0;
  for (const mol::residue_torsions &r : t) {
    if (!std::isnan(r.omega)) {
      ++peptides;
      trans += std::abs(r.omega) > 150.0 * kPi / 180.0 ? 1 : 0;
    }
    if (std::isnan(r.phi) || std::isnan(r.psi))
      continue;
    const int s = m.structure_of(m.atoms[r.first_atom]);
    if (s < 0)
      continue;
    const double phi = r.phi * 180.0 / kPi, psi = r.psi * 180.0 / kPi;
    if (m.structures[s].type == mol::secondary_structure::kind::helix) {
      ++helix;
      helical += phi < -30 && phi > -100 && psi < 0 && psi > -80 ? 1 : 0;
    } else if (m.structures[s].type == mol::secondary_structure::kind::sheet) {
      ++sheet;
      extended += phi < -60 && (psi > 80 || psi < -150) ? 1 : 0;
    }
  }
  EXPECT_GT(peptides, 400u);
  EXPECT_GT(trans, peptides * 95 / 100);
  EXPECT_GT(helical, helix * 85 / 100);
  EXPECT_GT(extended, sheet * 70 / 100);
}

// ---- steppers ----

TEST(MolGroups, BudgetsCancelAndTwoThreads) {
  auto m = std::make_shared<const mol::molecule>(grl());
  const mol::group_tree whole = mol::build_group_tree(*m);
  const std::vector<mol::residue_torsions> angles = mol::torsions(*m);
  for (const std::size_t budget : {std::size_t(1), std::size_t(7)}) {
    mol::group_tree_stepper g(m);
    double last = 0.0;
    while (!g.step(budget)) {
      EXPECT_GE(g.progress(), last);
      last = g.progress();
    }
    EXPECT_EQ(mol::describe(g.take()), mol::describe(whole));
    mol::torsion_stepper s(m);
    while (!s.step(budget)) {
    }
    const std::vector<mol::residue_torsions> got = s.take();
    ASSERT_EQ(got.size(), angles.size());
    for (std::size_t i = 0; i < got.size(); ++i) {
      EXPECT_EQ(std::isnan(got[i].phi), std::isnan(angles[i].phi));
      if (!std::isnan(got[i].phi))
        EXPECT_EQ(got[i].phi, angles[i].phi);
    }
  }
  mol::group_tree_stepper c(m);
  c.step(10);
  c.cancel();
  EXPECT_TRUE(c.step(10));
  EXPECT_STREQ(c.phase(), "cancelled");
  EXPECT_THROW(c.take(), std::logic_error);
  // Two threads, each its own, give what one gives alone.
  std::string a, b;
  std::thread ta([&] { a = mol::describe(mol::build_group_tree(*m)); });
  std::thread tb([&] { b = mol::describe(mol::build_group_tree(*m)); });
  ta.join();
  tb.join();
  EXPECT_EQ(a, mol::describe(whole));
  EXPECT_EQ(b, a);
  // The job form.
  auto job = mol::torsions_job(m);
  while (!job->step(100)) {
  }
  EXPECT_EQ(job->take_result().size(), angles.size());
}
