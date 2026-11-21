/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol's molecules and readers (cvc/mol/molecule.h; TexMol roadmap M0):
//   - the residue-atom table is TexMol's, row for row, and is looked up without blanks;
//   - read_pdb reads the PDB's columns: atoms, HETATM, alternate locations, models, HELIX /
//     SHEET / TURN, HEADER and TITLE, element symbols from columns 77-78 or the atom name, CRLF
//     lines, and the older files whose columns 73-80 hold the entry and a line number;
//   - radii and charges come from the table, or unknown_radius for an atom it hasn't;
//   - the filters (models, HETATM, water, hydrogens, alternate locations);
//   - read_pqr and read_xyzr, and every reader's errors name the line (T5);
//   - 1GRL, a GroEL subunit, read whole: its atoms, hydrogens, helices and sheets;
//   - read_molecule by extension through the app's handlers, which are each app's own (T6).

#include <cmath>
#include <cvc/core/app.h>
#include <cvc/mol/molecule.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <sstream>
#include <string>

namespace mol = cvc::mol;
namespace fs = std::filesystem;

namespace {

const std::string kRepo = CVC_REPO_DIR;
const std::string k1grl = kRepo + "/src/cvc/tests/data/mol/1grl.pdb";

// A small PDB with everything read_pdb handles, in v3.3 columns.
const char *const kPdb =
    "HEADER    TEST PROTEIN                            20-NOV-26   9ZZZ              \n"
    "TITLE     A TEST MOLECULE                                                       \n"
    "TITLE    2 WITH TWO TITLE LINES                                                 \n"
    "HELIX    1   1 ALA A    1  GLY A    2  1                                   2    \n"
    "SHEET    1   S 2 SER B  10  SER B  11  0                                        \n"
    "TURN     1 T1 ALA A   3  ALA A   3                                              \n"
    "MODEL        1                                                                  \n"
    "ATOM      1  N   ALA A   1      11.104   6.134  -6.504  1.00  0.00           N  \n"
    "ATOM      2  CA  ALA A   1      11.639   6.071  -5.147  1.00  0.00           C  \n"
    "ATOM      3  CA AGLY A   2      12.000   7.000  -4.000  0.60 10.00           C  \n"
    "ATOM      4  CA BGLY A   2      12.100   7.100  -4.100  0.40 10.00           C  \n"
    "ATOM      5  H   GLY A   2      12.500   7.500  -3.500  1.00  0.00           H  \n"
    "ATOM      6  CA  SER B  10      20.000  20.000  20.000  1.00  0.00              \n"
    "HETATM    7 FE   HEM A 100      15.000  15.000  15.000  1.00  0.00          FE  \n"
    "HETATM    8  O   HOH A 201      30.000  30.000  30.000  1.00  0.00           O  \n"
    "ENDMDL                                                                          \n"
    "MODEL        2                                                                  \n"
    "ATOM      9  N   ALA A   1      11.204   6.234  -6.604  1.00  0.00           N  \n"
    "ENDMDL                                                                          \n"
    "END                                                                             \n";

mol::molecule pdb(const std::string &text, const mol::molecule_read_params &p = {}) {
  std::istringstream in(text);
  return mol::read_pdb(in, p, "test.pdb");
}

mol::molecule pqr(const std::string &text, const mol::molecule_read_params &p = {}) {
  std::istringstream in(text);
  return mol::read_pqr(in, p, "test.pqr");
}

// The message of the molecule_file_error `f` throws, or "" when it throws none.
template <class F> std::string error_of(F f) {
  try {
    f();
  } catch (const mol::molecule_file_error &e) {
    return e.what();
  }
  return "";
}

fs::path temp_dir() {
  const fs::path d =
      fs::temp_directory_path() / (std::string("cvc_molecule_test_") +
                                   ::testing::UnitTest::GetInstance()->current_test_info()->name());
  fs::create_directories(d);
  return d;
}

void write(const fs::path &p, const std::string &text) {
  std::ofstream out(p, std::ios::binary);
  out << text;
}

} // namespace

// ---- the table ----

TEST(MolTable, IsTexMolsRowForRow) {
  std::size_t rows = 0;
  const mol::residue_atom_info *t = mol::residue_atom_table(&rows);
  ASSERT_EQ(rows, 410u);
  EXPECT_EQ(t[0].atom, "XXXX"); // TexMol's placeholder
  EXPECT_EQ(t[1].atom, " N  ");
  EXPECT_EQ(t[1].residue, "ALA");
  EXPECT_FLOAT_EQ(t[1].radius, 1.625f);
  EXPECT_FLOAT_EQ(t[1].solvent_enlarged_radius, 2.230f);
  EXPECT_FLOAT_EQ(t[1].charge, -0.5f);
  EXPECT_EQ(t[409].atom, " C6 ");
  EXPECT_EQ(t[409].residue, "  U");
}

TEST(MolTable, FindsAnAtomWithoutItsBlanks) {
  const int ca = mol::find_residue_atom(" CA ", "ALA");
  ASSERT_GE(ca, 0);
  EXPECT_EQ(mol::find_residue_atom("CA", "ALA"), ca);
  EXPECT_EQ(mol::find_residue_atom("CA", " ALA "), ca);
  std::size_t rows = 0;
  EXPECT_FLOAT_EQ(mol::residue_atom_table(&rows)[ca].radius, 1.75f);
  EXPECT_GE(mol::find_residue_atom("P", "G"), 0); // a nucleotide, residue "  G"
  EXPECT_EQ(mol::find_residue_atom("OXT", "ASP"), -1);
  EXPECT_EQ(mol::find_residue_atom("FE", "HEM"), -1);
  EXPECT_EQ(mol::find_residue_atom("", "ALA"), -1);
}

// ---- read_pdb ----

TEST(MolPdb, ReadsTheFirstModelsAtomsAndTheRecords) {
  const mol::molecule m = pdb(kPdb);
  EXPECT_EQ(m.id, "9ZZZ");
  EXPECT_EQ(m.title, "A TEST MOLECULE WITH TWO TITLE LINES");
  EXPECT_EQ(m.models, 2);
  ASSERT_EQ(m.atoms.size(), 8u); // model 1's eight; model 2's N is not read

  const mol::atom &n = m.atoms[0];
  EXPECT_EQ(n.serial, 1);
  EXPECT_EQ(n.name, "N");
  EXPECT_EQ(n.residue, "ALA");
  EXPECT_EQ(n.chain, 'A');
  EXPECT_EQ(n.residue_seq, 1);
  EXPECT_DOUBLE_EQ(n.x, 11.104);
  EXPECT_DOUBLE_EQ(n.y, 6.134);
  EXPECT_DOUBLE_EQ(n.z, -6.504);
  EXPECT_EQ(n.element, "N");
  EXPECT_EQ(n.model, 1);
  EXPECT_FALSE(n.hetero);
  EXPECT_FLOAT_EQ(static_cast<float>(n.radius), 1.625f); // the table's ALA N
  EXPECT_FLOAT_EQ(static_cast<float>(n.charge), -0.5f);

  EXPECT_EQ(m.atoms[2].alt_loc, 'A');
  EXPECT_DOUBLE_EQ(m.atoms[2].occupancy, 0.6);
  EXPECT_DOUBLE_EQ(m.atoms[2].b_factor, 10.0);
  EXPECT_EQ(m.atoms[3].alt_loc, 'B');
  EXPECT_EQ(m.atoms[4].element, "H");
  EXPECT_EQ(m.atoms[5].element, "C"); // no element columns: from the name " CA "

  const mol::atom &fe = m.atoms[6];
  EXPECT_TRUE(fe.hetero);
  EXPECT_EQ(fe.name, "FE");
  EXPECT_EQ(fe.element, "FE");
  EXPECT_EQ(fe.table_row, -1);
  EXPECT_DOUBLE_EQ(fe.radius, 2.0); // unknown_radius
  EXPECT_DOUBLE_EQ(fe.charge, 0.0);
  EXPECT_EQ(m.atoms[7].residue, "HOH");
}

TEST(MolPdb, ReadsTheSecondaryStructures) {
  const mol::molecule m = pdb(kPdb);
  ASSERT_EQ(m.structures.size(), 3u);
  using kind = mol::secondary_structure::kind;
  const mol::secondary_structure &h = m.structures[0];
  EXPECT_EQ(h.type, kind::helix);
  EXPECT_EQ(h.id, "1");
  EXPECT_EQ(h.chain, 'A');
  EXPECT_EQ(h.start_seq, 1);
  EXPECT_EQ(h.end_seq, 2);
  EXPECT_EQ(h.helix_class, 1);
  const mol::secondary_structure &s = m.structures[1];
  EXPECT_EQ(s.type, kind::sheet);
  EXPECT_EQ(s.id, "S");
  EXPECT_EQ(s.strand, 1);
  EXPECT_EQ(s.chain, 'B');
  EXPECT_EQ(s.start_seq, 10);
  EXPECT_EQ(s.end_seq, 11);
  EXPECT_EQ(s.sense, 0);
  const mol::secondary_structure &t = m.structures[2];
  EXPECT_EQ(t.type, kind::turn);
  EXPECT_EQ(t.id, "T1");
  EXPECT_EQ(t.start_seq, 3);

  EXPECT_EQ(m.structure_of(m.atoms[0]), 0);  // ALA A 1: the helix
  EXPECT_EQ(m.structure_of(m.atoms[2]), 0);  // GLY A 2
  EXPECT_EQ(m.structure_of(m.atoms[5]), 1);  // SER B 10: the sheet
  EXPECT_EQ(m.structure_of(m.atoms[6]), -1); // HEM A 100

  mol::secondary_structure r;
  r.chain = 'A';
  r.start_seq = 5;
  r.end_seq = 7;
  r.end_insertion = 'B';
  EXPECT_TRUE(r.contains('A', 7, 'A'));
  EXPECT_TRUE(r.contains('A', 7, 'B'));
  EXPECT_FALSE(r.contains('A', 7, 'C'));
  EXPECT_FALSE(r.contains('A', 4));
  EXPECT_FALSE(r.contains('B', 6));
}

TEST(MolPdb, TheFiltersAndRadii) {
  mol::molecule_read_params p;
  p.model = 2;
  EXPECT_EQ(pdb(kPdb, p).atoms.size(), 1u);
  p.model = 0;
  EXPECT_EQ(pdb(kPdb, p).atoms.size(), 9u);
  p.model = 3;
  EXPECT_TRUE(pdb(kPdb, p).atoms.empty());

  p = {};
  p.hetero = false;
  EXPECT_EQ(pdb(kPdb, p).atoms.size(), 6u);
  p = {};
  p.water = false;
  EXPECT_EQ(pdb(kPdb, p).atoms.size(), 7u);
  p = {};
  p.hydrogens = false;
  EXPECT_EQ(pdb(kPdb, p).atoms.size(), 7u);
  p = {};
  p.alt_locs = false;
  const mol::molecule one = pdb(kPdb, p);
  ASSERT_EQ(one.atoms.size(), 7u);
  EXPECT_EQ(one.atoms[2].alt_loc, 'A'); // the first one given; B is left out

  p = {};
  p.radii = mol::radius_kind::solvent_enlarged;
  p.unknown_radius = 1.5;
  const mol::molecule r = pdb(kPdb, p);
  EXPECT_FLOAT_EQ(static_cast<float>(r.atoms[0].radius), 2.230f);
  EXPECT_DOUBLE_EQ(r.atoms[6].radius, 1.5);
}

TEST(MolPdb, OlderColumnsAndCrlf) {
  // 1995-style lines: columns 73-80 are the entry and a line number, so 77-78 hold no element.
  const std::string text =
      "ATOM      1  N   VAL     6     -23.265  20.257 -15.615  1.00 18.80      1GRL 203\r\n"
      "ATOM      2 1HZ  LYS     7     -22.384  21.361 -15.112  1.00 18.80      1GRL4204\r\n"
      "ATOM      3  HG2 GLU     8     -22.000  21.000 -15.000  1.00 18.80      1GRL4205\r\n";
  const mol::molecule m = pdb(text);
  ASSERT_EQ(m.atoms.size(), 3u);
  EXPECT_EQ(m.atoms[0].element, "N");
  EXPECT_EQ(m.atoms[0].chain, ' ');
  EXPECT_DOUBLE_EQ(m.atoms[0].z, -15.615);
  EXPECT_EQ(m.atoms[1].name, "1HZ");
  EXPECT_EQ(m.atoms[1].element, "H");
  EXPECT_EQ(m.atoms[2].element, "H");
  EXPECT_EQ(m.atoms[0].model, 0);
}

TEST(MolPdb, AMalformedLineNamesItself) {
  const std::string bad_x =
      "ATOM      1  N   ALA A   1      11.1x4   6.134  -6.504  1.00  0.00           N\n";
  EXPECT_NE(error_of([&] { pdb(bad_x); }).find("test.pdb:1"), std::string::npos);
  EXPECT_NE(error_of([&] { pdb(bad_x); }).find("'11.1x4' is not a number"), std::string::npos);
  const std::string no_z = "ATOM      1  N   ALA A   1      11.104   6.134\n";
  EXPECT_NE(error_of([&] { pdb(no_z); }).find("has no z"), std::string::npos);
  const std::string bad_seq =
      "HELIX    1   1 ALA A    x  GLY A    2  1                                   2\n";
  EXPECT_NE(error_of([&] { pdb(bad_seq); }).find("HELIX's first residue"), std::string::npos);
  mol::molecule_read_params p;
  p.unknown_radius = 0;
  EXPECT_THROW(pdb(kPdb, p), std::invalid_argument);
}

// ---- read_pqr, read_xyzr ----

TEST(MolPqr, ReadsChargeAndRadiusFromTheFile) {
  const std::string text = "REMARK   1 PQR file\n"
                           "ATOM      1  N   ALA     1      -0.677  -1.230  -0.491 -0.3000 1.8240\n"
                           "ATOM      2  CA  ALA A   2       0.000   0.000   0.000  0.0337 1.9080\n"
                           "HETATM    3  O   HOH W  52A     1.000   2.000   3.000 -0.8340 1.6612\n"
                           "TER\nEND\n";
  const mol::molecule m = pqr(text);
  ASSERT_EQ(m.atoms.size(), 3u);
  EXPECT_EQ(m.atoms[0].chain, ' ');
  EXPECT_EQ(m.atoms[0].residue_seq, 1);
  EXPECT_DOUBLE_EQ(m.atoms[0].charge, -0.3);
  EXPECT_DOUBLE_EQ(m.atoms[0].radius, 1.824);
  EXPECT_EQ(m.atoms[0].table_row, -1);
  EXPECT_EQ(m.atoms[0].element, "N");
  EXPECT_EQ(m.atoms[1].chain, 'A');
  EXPECT_EQ(m.atoms[1].element, "C");
  EXPECT_TRUE(m.atoms[2].hetero);
  EXPECT_EQ(m.atoms[2].residue_seq, 52);
  EXPECT_EQ(m.atoms[2].insertion, 'A');
  EXPECT_DOUBLE_EQ(m.atoms[2].z, 3.0);

  mol::molecule_read_params p;
  p.water = false;
  EXPECT_EQ(pqr(text, p).atoms.size(), 2u);

  EXPECT_NE(error_of([] { pqr("ATOM 1 N ALA 1 0 0 0 0\n"); }).find("test.pqr:1"),
            std::string::npos);
  EXPECT_NE(error_of([] { pqr("ATOM 1 N ALA 1 0 0 zz 0 1.5\n"); }).find("'zz' is not a number"),
            std::string::npos);
  EXPECT_NE(error_of([] { pqr("ATOM 1 N ALA 1 0 0 0 0 -1\n"); }).find("negative"),
            std::string::npos);
}

TEST(MolXyzr, OneAtomALine) {
  std::istringstream in("# x y z r\n1 2 3 1.5\n\n-4.5 5 6e-1 2\n");
  const mol::molecule m = mol::read_xyzr(in, "t.xyzr");
  ASSERT_EQ(m.atoms.size(), 2u);
  EXPECT_DOUBLE_EQ(m.atoms[1].x, -4.5);
  EXPECT_DOUBLE_EQ(m.atoms[1].z, 0.6);
  EXPECT_DOUBLE_EQ(m.atoms[1].radius, 2.0);
  std::istringstream bad("1 2 3\n");
  EXPECT_NE(error_of([&] { mol::read_xyzr(bad, "t.xyzr"); }).find("t.xyzr:1"), std::string::npos);
}

// ---- a real entry: 1GRL, a GroEL subunit (PDB, public domain) ----

TEST(Mol1grl, AGroElSubunit) {
  cvc::app app;
  const mol::molecule m = mol::read_molecule(app, k1grl);
  EXPECT_EQ(m.id, "1GRL");
  EXPECT_EQ(m.models, 0);
  ASSERT_EQ(m.atoms.size(), 4186u);
  std::size_t hydrogens = 0, unknown_heavy = 0, helices = 0, sheets = 0;
  double radius_sum = 0.0;
  for (const mol::atom &a : m.atoms) {
    hydrogens += a.element == "H";
    unknown_heavy += a.element != "H" && a.table_row < 0;
    radius_sum += a.radius;
    EXPECT_EQ(a.chain, ' ');
    EXPECT_EQ(a.model, 0);
  }
  for (const mol::secondary_structure &s : m.structures) {
    helices += s.type == mol::secondary_structure::kind::helix;
    sheets += s.type == mol::secondary_structure::kind::sheet;
  }
  EXPECT_EQ(hydrogens, 757u);   // its polar hydrogens
  EXPECT_EQ(unknown_heavy, 1u); // the C terminus's OXT
  EXPECT_NEAR(radius_sum, 6844.815, 1e-3);
  EXPECT_EQ(helices, 19u);
  EXPECT_EQ(sheets, 11u);
  EXPECT_EQ(m.atoms.front().residue, "VAL");
  EXPECT_EQ(m.atoms.back().name, "H");
  // Its first helix is GLY 9 to THR 30, its first sheet strand VAL 38 to LEU 40.
  for (const mol::atom &a : m.atoms) {
    if (a.residue_seq >= 9 && a.residue_seq <= 30)
      EXPECT_EQ(m.structure_of(a), 0) << a.residue << ' ' << a.residue_seq;
    if (a.residue_seq >= 38 && a.residue_seq <= 40)
      EXPECT_EQ(m.structures[static_cast<std::size_t>(m.structure_of(a))].type,
                mol::secondary_structure::kind::sheet);
  }

  mol::molecule_read_params heavy;
  heavy.hydrogens = false;
  EXPECT_EQ(mol::read_molecule(app, k1grl, heavy).atoms.size(), 4186u - 757u);

  const std::array<double, 6> b = m.bounds(), br = m.bounds(true);
  for (int k = 0; k < 3; ++k) {
    EXPECT_LT(b[k], b[k + 3]);
    EXPECT_LT(br[k], b[k]);
    EXPECT_GT(br[k + 3], b[k + 3]);
  }
}

// ---- files and handlers ----

TEST(MolFiles, ReadsByExtension) {
  cvc::app app;
  const std::vector<std::string> ext = mol::molecule_read_extensions(app);
  EXPECT_EQ(ext, (std::vector<std::string>{".ent", ".pdb", ".pqr", ".xyzr"}));
  const fs::path d = temp_dir();
  write(d / "a.PQR", "ATOM 1 CA ALA 1 1 2 3 0.5 1.7\n");
  write(d / "a.xyzr", "1 2 3 1.5\n4 5 6 1.5\n");
  write(d / "empty.pdb", "REMARK nothing\n");
  write(d / "a.mol2", "@<TRIPOS>MOLECULE\n");
  EXPECT_EQ(mol::read_molecule(app, (d / "a.PQR").string()).atoms.size(), 1u);
  EXPECT_EQ(mol::read_molecule(app, (d / "a.xyzr").string()).atoms.size(), 2u);
  EXPECT_NE(
      error_of([&] { mol::read_molecule(app, (d / "empty.pdb").string()); }).find("has no atoms"),
      std::string::npos);
  EXPECT_NE(error_of([&] {
              mol::read_molecule(app, (d / "a.mol2").string());
            }).find("no reader for '.mol2'"),
            std::string::npos);
  EXPECT_NE(
      error_of([&] { mol::read_molecule(app, (d / "missing.pdb").string()); }).find("can't open"),
      std::string::npos);
  mol::molecule_read_params p;
  p.model = 2;
  EXPECT_NE(error_of([&] { mol::read_molecule(app, k1grl, p); }).find("no MODEL records"),
            std::string::npos);
  fs::remove_all(d);
}

namespace {
struct mol2_stub : mol::molecule_file_io {
  const std::string &id() const override { return _id; }
  const std::vector<std::string> &extensions() const override { return _ext; }
  bool can_read() const override { return true; }
  mol::molecule read(cvc::app &, const std::string &,
                     const mol::molecule_read_params &) const override {
    mol::molecule m;
    m.atoms.emplace_back();
    m.atoms.back().radius = 1.0;
    return m;
  }
  std::string _id = "mol2 stub";
  std::vector<std::string> _ext{".mol2"};
};
} // namespace

TEST(MolFiles, EachAppHasItsOwnHandlers) {
  cvc::app a, b;
  ASSERT_TRUE(
      mol::molecule_handlers(a).insert(boost::shared_ptr<mol::molecule_file_io>(new mol2_stub)));
  EXPECT_EQ(mol::molecule_read_extensions(a).size(), 5u);
  EXPECT_EQ(mol::molecule_read_extensions(b).size(), 4u);
  const fs::path d = temp_dir();
  write(d / "x.mol2", "");
  EXPECT_EQ(mol::read_molecule(a, (d / "x.mol2").string()).atoms.size(), 1u);
  EXPECT_THROW(mol::read_molecule(b, (d / "x.mol2").string()), mol::molecule_file_error);
  fs::remove_all(d);
}
