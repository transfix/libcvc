/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol views (cvc/mol/view.h; TexMol roadmap M0 part 2), the data the `molecule` scene node
// draws:
//   - the palettes and the covalent radii (T2);
//   - bonds by distance: covalent radii and the tolerance, models, alternate locations and metal
//     ions (T2, T3), and on 1GRL every backbone bond (T3);
//   - the cartoon's trace: a C-alpha a residue, pieces at chain breaks and gaps, each residue's
//     secondary structure (T3);
//   - each style's spheres, sticks and cartoon, the colour schemes and the atom selection (T3, T4:
//     unit normals, indices in range, bounds holding everything drawn);
//   - picking a sphere and the drawn atom nearest a point;
//   - bad parameters (T5);
//   - two apps on two threads give what each gives alone (T6);
//   - budgets 1, 64, random and 0 give the same view bit for bit, progress is monotone and the
//     phases are named; a cancel leaves no result (T7);
//   - the parameters round-trip through the state tree (T8).

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/param_table.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/view.h>
#include <cvc/state/state.h>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>

namespace mol = cvc::mol;

namespace {

const std::string kRepo = CVC_REPO_DIR;
const std::string k1grl = kRepo + "/examples/data/molecules/1grl.pdb";

std::shared_ptr<const mol::molecule> grl() {
  std::ifstream in(k1grl, std::ios::binary);
  EXPECT_TRUE(in.good()) << k1grl;
  return std::make_shared<const mol::molecule>(mol::read_pdb(in, {}, k1grl));
}

mol::atom make_atom(const std::string &name, const std::string &element, double x, double y,
                    double z, char chain = 'A', int seq = 1, const std::string &residue = "ALA") {
  mol::atom a;
  a.name = name;
  a.element = element;
  a.x = x;
  a.y = y;
  a.z = z;
  a.chain = chain;
  a.residue_seq = seq;
  a.residue = residue;
  a.radius = element == "H" ? 1.2 : 1.7;
  return a;
}

// Two short peptides' backbones (chains A and B, three residues each, 3.8 Å between C-alphas),
// the first residue of A in a helix, plus a water and a zinc ion.
std::shared_ptr<const mol::molecule> peptides() {
  mol::molecule m;
  for (char chain : {'A', 'B'}) {
    const double y = chain == 'A' ? 0.0 : 20.0;
    for (int r = 0; r < 3; ++r) {
      const double x = 3.8 * r;
      m.atoms.push_back(make_atom("N", "N", x - 1.0, y + 0.8, 0.0, chain, r + 1));
      m.atoms.push_back(make_atom("CA", "C", x, y, 0.0, chain, r + 1));
      m.atoms.push_back(make_atom("C", "C", x + 1.2, y + 0.6, 0.0, chain, r + 1));
      m.atoms.push_back(make_atom("O", "O", x + 1.4, y + 1.8, 0.0, chain, r + 1));
      m.atoms.back().b_factor = 10.0 * r;
    }
  }
  mol::atom w = make_atom("O", "O", 0.0, 50.0, 0.0, 'A', 100, "HOH");
  w.hetero = true;
  m.atoms.push_back(w);
  mol::atom zn = make_atom("ZN", "ZN", 2.0, -3.0, 0.0, 'A', 101, "ZN");
  zn.hetero = true;
  m.atoms.push_back(zn);
  mol::secondary_structure h;
  h.type = mol::secondary_structure::kind::helix;
  h.chain = 'A';
  h.start_seq = 1;
  h.end_seq = 2;
  m.structures.push_back(h);
  return std::make_shared<const mol::molecule>(std::move(m));
}

void expect_same_geometry(const cvc::geometry &a, const cvc::geometry &b, const char *what) {
  ASSERT_EQ(a.points().size(), b.points().size()) << what;
  ASSERT_EQ(a.tris().size(), b.tris().size()) << what;
  for (std::size_t i = 0; i < a.points().size(); ++i)
    for (int c = 0; c < 3; ++c) {
      ASSERT_EQ(a.points()[i][c], b.points()[i][c]) << what << " point " << i;
      ASSERT_EQ(a.normals()[i][c], b.normals()[i][c]) << what << " normal " << i;
      ASSERT_EQ(a.colors()[i][c], b.colors()[i][c]) << what << " colour " << i;
    }
  for (std::size_t i = 0; i < a.tris().size(); ++i)
    for (int c = 0; c < 3; ++c)
      ASSERT_EQ(a.tris()[i][c], b.tris()[i][c]) << what << " triangle " << i;
}

void expect_same_view(const mol::molecule_view &a, const mol::molecule_view &b) {
  EXPECT_EQ(a.sphere_atoms, b.sphere_atoms);
  EXPECT_EQ(a.centers, b.centers);
  EXPECT_EQ(a.radii, b.radii);
  EXPECT_EQ(a.colors, b.colors);
  ASSERT_EQ(a.bonds.size(), b.bonds.size());
  for (std::size_t i = 0; i < a.bonds.size(); ++i) {
    EXPECT_EQ(a.bonds[i].a, b.bonds[i].a);
    EXPECT_EQ(a.bonds[i].b, b.bonds[i].b);
  }
  expect_same_geometry(a.spheres, b.spheres, "spheres");
  expect_same_geometry(a.sticks, b.sticks, "sticks");
  expect_same_geometry(a.cartoon, b.cartoon, "cartoon");
  EXPECT_EQ(a.cartoon_atoms, b.cartoon_atoms);
  EXPECT_EQ(a.bounds, b.bounds);
  EXPECT_EQ(a.atoms_drawn, b.atoms_drawn);
  EXPECT_EQ(a.residues, b.residues);
}

// A mesh's triangles index its points, its normals are unit and it lies inside the bounds.
void expect_sound_mesh(const cvc::geometry &g, const std::array<double, 6> &bounds,
                       const char *what) {
  ASSERT_EQ(g.normals().size(), g.points().size()) << what;
  ASSERT_EQ(g.colors().size(), g.points().size()) << what;
  for (const auto &t : g.tris())
    for (int c = 0; c < 3; ++c)
      ASSERT_LT(t[c], g.points().size()) << what;
  for (std::size_t i = 0; i < g.points().size(); ++i) {
    const auto &n = g.normals()[i];
    ASSERT_NEAR(std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]), 1.0, 1e-6) << what << " " << i;
    for (int c = 0; c < 3; ++c) {
      ASSERT_GE(g.points()[i][c], bounds[c] - 1e-9) << what;
      ASSERT_LE(g.points()[i][c], bounds[c + 3] + 1e-9) << what;
    }
  }
}

bool bonded(const std::vector<mol::bond> &bonds, int a, int b) {
  if (a > b)
    std::swap(a, b);
  for (const mol::bond &x : bonds)
    if (x.a == a && x.b == b)
      return true;
  return false;
}

} // namespace

TEST(MolView, Palettes) {
  const mol::rgb8 c = mol::element_color("C");
  EXPECT_EQ(c[0], c[1]);
  EXPECT_EQ(c[1], c[2]);                                             // carbon grey
  EXPECT_GT(mol::element_color("N")[2], mol::element_color("N")[0]); // nitrogen blue
  EXPECT_GT(mol::element_color("O")[0], mol::element_color("O")[2]); // oxygen red
  EXPECT_EQ(mol::element_color("fe"), mol::element_color("FE"));
  EXPECT_EQ(mol::element_color("XX"), (mol::rgb8{240, 120, 220}));
  EXPECT_EQ(mol::chain_color(0), mol::chain_color(12));
  EXPECT_NE(mol::chain_color(0), mol::chain_color(1));
  EXPECT_NE(mol::residue_color("ASP"), mol::residue_color("LYS"));
  EXPECT_EQ(mol::residue_color("asp"), mol::residue_color("ASP"));
  const mol::rgb8 lo = mol::b_factor_color(0.0), mid = mol::b_factor_color(0.5),
                  hi = mol::b_factor_color(1.0);
  EXPECT_GT(lo[2], lo[0]);
  EXPECT_GT(hi[0], hi[2]);
  EXPECT_EQ(mid[0], mid[2]);
  EXPECT_EQ(mol::b_factor_color(-3.0), lo);
  EXPECT_EQ(mol::b_factor_color(7.0), hi);
  EXPECT_NE(mol::structure_color(0), mol::structure_color(1));
  EXPECT_NE(mol::structure_color(1), mol::structure_color(-1));
  EXPECT_DOUBLE_EQ(mol::covalent_radius("C"), 0.76f);
  EXPECT_DOUBLE_EQ(mol::covalent_radius("Q"), 0.77f);
  EXPECT_TRUE(mol::is_metal("ZN"));
  EXPECT_TRUE(mol::is_metal("Fe"));
  EXPECT_FALSE(mol::is_metal("S"));
}

TEST(MolView, BondsByDistance) {
  mol::molecule m;
  m.atoms.push_back(make_atom("C1", "C", 0.0, 0.0, 0.0));   // 0
  m.atoms.push_back(make_atom("C2", "C", 1.54, 0.0, 0.0));  // 1: bonded to 0
  m.atoms.push_back(make_atom("C3", "C", 4.0, 0.0, 0.0));   // 2: too far from 1
  m.atoms.push_back(make_atom("H1", "H", 0.0, 1.09, 0.0));  // 3: bonded to 0
  m.atoms.push_back(make_atom("H2", "H", 0.0, 1.09, 1.8));  // 4: H-H 1.8 apart, no bond
  m.atoms.push_back(make_atom("ZN", "ZN", 0.0, -1.5, 0.0)); // 5: a metal, no bonds
  mol::atom other = make_atom("C9", "C", 1.5, 0.3, 0.0);    // 6: another model
  other.model = 2;
  m.atoms.push_back(other);
  mol::atom alt_a = make_atom("CA", "C", 10.0, 0.0, 0.0); // 7, 8: two locations of one atom
  alt_a.alt_loc = 'A';
  mol::atom alt_b = make_atom("CA", "C", 10.6, 0.0, 0.0);
  alt_b.alt_loc = 'B';
  m.atoms.push_back(alt_a);
  m.atoms.push_back(alt_b);
  m.atoms.push_back(make_atom("CB", "C", 11.5, 0.0, 0.0)); // 9: ' ' bonds to both
  const std::vector<mol::bond> b = mol::find_bonds(m);
  EXPECT_TRUE(bonded(b, 0, 1));
  EXPECT_TRUE(bonded(b, 0, 3));
  EXPECT_FALSE(bonded(b, 1, 2));
  EXPECT_FALSE(bonded(b, 3, 4));
  for (const mol::bond &x : b) {
    EXPECT_NE(x.a, 5);
    EXPECT_NE(x.b, 5);
    EXPECT_FALSE(x.a == 6 || x.b == 6) << "a bond across models";
    EXPECT_LT(x.a, x.b);
  }
  EXPECT_FALSE(bonded(b, 7, 8));
  EXPECT_TRUE(bonded(b, 7, 9));
  EXPECT_TRUE(bonded(b, 8, 9));
  // In order of the first atom, then the second.
  for (std::size_t i = 1; i < b.size(); ++i)
    EXPECT_TRUE(b[i - 1].a < b[i].a || (b[i - 1].a == b[i].a && b[i - 1].b < b[i].b));
  // No tolerance: C-C at 1.54 still bonds (1.52 + 0.02 past), C-H at 1.09 too.
  EXPECT_FALSE(bonded(mol::find_bonds(m, 0.0), 0, 1));
  EXPECT_TRUE(bonded(mol::find_bonds(m, 0.1), 0, 1));
}

TEST(MolView, OneGrlBackbone) {
  const auto m = grl();
  const std::vector<mol::bond> b = mol::find_bonds(*m);
  EXPECT_GT(b.size(), m->atoms.size() * 9 / 10);
  EXPECT_LT(b.size(), m->atoms.size() * 12 / 10);
  for (const mol::bond &x : b) {
    const mol::atom &p = m->atoms[x.a], &q = m->atoms[x.b];
    const double d = std::hypot(p.x - q.x, p.y - q.y, p.z - q.z);
    EXPECT_LT(d, 2.2) << p.name << " " << q.name;
  }
  // Every residue's N-CA and CA-C, and the peptide bond to the next residue where it is one. (A
  // stretch of 1GRL is C-alphas alone, 3.8 Å apart: no bonds there.)
  std::size_t peptide = 0, residues = 0;
  for (std::size_t i = 0; i < m->atoms.size(); ++i) {
    const mol::atom &a = m->atoms[i];
    if (a.name != "CA")
      continue;
    int n = -1, c = -1;
    for (std::size_t j = (i > 4 ? i - 4 : 0); j < std::min(m->atoms.size(), i + 4); ++j) {
      const mol::atom &o = m->atoms[j];
      if (o.residue_seq != a.residue_seq || o.chain != a.chain)
        continue;
      if (o.name == "N")
        n = static_cast<int>(j);
      if (o.name == "C")
        c = static_cast<int>(j);
    }
    if (n < 0 || c < 0)
      continue;
    ++residues;
    EXPECT_TRUE(bonded(b, n, static_cast<int>(i))) << a.residue_seq;
    EXPECT_TRUE(bonded(b, static_cast<int>(i), c)) << a.residue_seq;
    for (std::size_t j = i + 1; j < std::min(m->atoms.size(), i + 30); ++j) {
      const mol::atom &o = m->atoms[j];
      if (o.name == "N" && o.residue_seq == a.residue_seq + 1) {
        const mol::atom &cc = m->atoms[c];
        if (std::hypot(o.x - cc.x, o.y - cc.y, o.z - cc.z) < 1.5) {
          EXPECT_TRUE(bonded(b, c, static_cast<int>(j))) << a.residue_seq;
          ++peptide;
        }
        break;
      }
    }
  }
  EXPECT_EQ(residues, 459u);
  EXPECT_GT(peptide, 440u);
}

TEST(MolView, TheTrace) {
  const auto m = grl();
  const std::vector<mol::trace_residue> t = mol::chain_trace(*m);
  std::size_t cas = 0;
  for (const mol::atom &a : m->atoms)
    if (a.name == "CA")
      ++cas;
  ASSERT_EQ(t.size(), cas);
  ASSERT_FALSE(t.empty());
  EXPECT_TRUE(t.front().start);
  std::size_t helix = 0, sheet = 0, starts = 0, guides = 0;
  for (const mol::trace_residue &r : t) {
    EXPECT_EQ(m->atoms[r.atom].name, "CA");
    if (r.guide >= 0) { // the C-alpha-only stretch has none
      ++guides;
      EXPECT_EQ(m->atoms[r.guide].name, "O");
      EXPECT_EQ(m->atoms[r.guide].residue_seq, m->atoms[r.atom].residue_seq);
    }
    EXPECT_EQ(r.structure, m->structure_of(m->atoms[r.atom]));
    if (r.structure >= 0 &&
        m->structures[r.structure].type == mol::secondary_structure::kind::helix)
      ++helix;
    if (r.structure >= 0 &&
        m->structures[r.structure].type == mol::secondary_structure::kind::sheet)
      ++sheet;
    starts += r.start ? 1 : 0;
  }
  EXPECT_EQ(guides, 459u);
  EXPECT_GT(helix, 200u);
  EXPECT_GT(sheet, 40u);
  EXPECT_LT(starts, 10u); // one chain, with a few gaps the crystal didn't resolve
  // A gap: two peptides' C-alphas far apart start a new piece.
  const auto p = peptides();
  const std::vector<mol::trace_residue> pt = mol::chain_trace(*p);
  ASSERT_EQ(pt.size(), 6u);
  EXPECT_TRUE(pt[0].start);
  EXPECT_FALSE(pt[1].start);
  EXPECT_FALSE(pt[2].start);
  EXPECT_TRUE(pt[3].start); // chain B
  EXPECT_EQ(pt[0].structure, 0);
  EXPECT_EQ(pt[2].structure, -1);
}

TEST(MolView, TheStyles) {
  cvc::app app;
  const auto m = grl();
  mol::view_params p;
  p.style = mol::view_style::spacefill;
  const mol::molecule_view sf = mol::build_view(app, m, p);
  ASSERT_EQ(sf.sphere_atoms.size(), m->atoms.size());
  EXPECT_EQ(sf.atoms_drawn, m->atoms.size());
  for (std::size_t i = 0; i < sf.radii.size(); ++i) {
    EXPECT_FLOAT_EQ(sf.radii[i], static_cast<float>(m->atoms[sf.sphere_atoms[i]].radius));
    EXPECT_FLOAT_EQ(sf.centers[3 * i], static_cast<float>(m->atoms[sf.sphere_atoms[i]].x));
  }
  EXPECT_TRUE(sf.bonds.empty());
  EXPECT_TRUE(sf.sticks.points().empty());
  EXPECT_TRUE(sf.cartoon.points().empty());
  EXPECT_TRUE(sf.spheres.points().empty()); // tessellated only on request
  const std::array<double, 6> box = m->bounds(true);
  for (int c = 0; c < 6; ++c)
    EXPECT_NEAR(sf.bounds[c], box[c], 1e-5);

  p.style = mol::view_style::ball_and_stick;
  const mol::molecule_view bs = mol::build_view(app, m, p);
  ASSERT_EQ(bs.radii.size(), m->atoms.size());
  EXPECT_FLOAT_EQ(bs.radii[0], static_cast<float>(0.25 * m->atoms[0].radius));
  const std::vector<mol::bond> all = mol::find_bonds(*m);
  ASSERT_EQ(bs.bonds.size(), all.size());
  for (std::size_t i = 0; i < all.size(); ++i)
    EXPECT_TRUE(bs.bonds[i].a == all[i].a && bs.bonds[i].b == all[i].b);
  // Two half cylinders a bond, 12 sides each: four rings of 12, 48 triangles.
  EXPECT_EQ(bs.sticks.points().size(), all.size() * 48);
  EXPECT_EQ(bs.sticks.tris().size(), all.size() * 48);
  expect_sound_mesh(bs.sticks, bs.bounds, "sticks");

  p.style = mol::view_style::sticks;
  const mol::molecule_view st = mol::build_view(app, m, p);
  for (float r : st.radii)
    EXPECT_FLOAT_EQ(r, 0.15f);
  EXPECT_EQ(st.bonds.size(), all.size());

  p.style = mol::view_style::cartoon;
  p.color_by = mol::color_scheme::structure;
  const mol::molecule_view ct = mol::build_view(app, m, p);
  EXPECT_TRUE(ct.radii.empty());
  EXPECT_TRUE(ct.bonds.empty());
  ASSERT_FALSE(ct.cartoon.tris().empty());
  EXPECT_EQ(ct.cartoon_atoms.size(), ct.cartoon.points().size());
  EXPECT_EQ(ct.residues, mol::chain_trace(*m).size());
  expect_sound_mesh(ct.cartoon, ct.bounds, "cartoon");
  // A helix residue's vertices are in the helix colour, a sheet's in the sheet's.
  const mol::rgb8 hc = mol::structure_color(0), sc = mol::structure_color(1);
  std::size_t in_helix = 0, in_sheet = 0;
  for (std::size_t i = 0; i < ct.cartoon_atoms.size(); ++i) {
    const int s = m->structure_of(m->atoms[ct.cartoon_atoms[i]]);
    if (s < 0)
      continue;
    const mol::rgb8 want = m->structures[s].type == mol::secondary_structure::kind::helix ? hc : sc;
    const auto &c = ct.cartoon.colors()[i];
    ASSERT_NEAR(c[0], want[0] / 255.0, 1e-9);
    ASSERT_NEAR(c[2], want[2] / 255.0, 1e-9);
    (m->structures[s].type == mol::secondary_structure::kind::helix ? in_helix : in_sheet)++;
  }
  EXPECT_GT(in_helix, 0u);
  EXPECT_GT(in_sheet, 0u);
}

TEST(MolView, ColoursAndSelection) {
  cvc::app app;
  const auto m = peptides();
  mol::view_params p;
  p.style = mol::view_style::spacefill;
  p.color_by = mol::color_scheme::chain;
  mol::molecule_view v = mol::build_view(app, m, p);
  ASSERT_EQ(v.radii.size(), m->atoms.size());
  EXPECT_EQ(v.chains, "AB");
  for (std::size_t i = 0; i < v.sphere_atoms.size(); ++i)
    EXPECT_EQ(v.colors[i], mol::chain_color(m->atoms[v.sphere_atoms[i]].chain == 'A' ? 0 : 1));
  // B-factors over a given range: 0 blue, 20 (past 10) red.
  p.color_by = mol::color_scheme::b_factor;
  p.b_range = {0.0, 10.0};
  v = mol::build_view(app, m, p);
  EXPECT_EQ(v.colors[3], mol::b_factor_color(0.0));  // residue 1's O, B 0
  EXPECT_EQ(v.colors[11], mol::b_factor_color(1.0)); // residue 3's O, B 20
  // The selection.
  p.color_by = mol::color_scheme::element;
  p.b_range.clear();
  p.water = false;
  EXPECT_EQ(mol::build_view(app, m, p).atoms_drawn, m->atoms.size() - 1);
  p.water = true;
  p.atoms = mol::atom_set::hetero;
  EXPECT_EQ(mol::build_view(app, m, p).atoms_drawn, 2u);
  p.atoms = mol::atom_set::polymer;
  EXPECT_EQ(mol::build_view(app, m, p).atoms_drawn, 24u);
  p.hidden_chains = "B";
  v = mol::build_view(app, m, p);
  EXPECT_EQ(v.atoms_drawn, 12u);
  for (int a : v.sphere_atoms)
    EXPECT_EQ(m->atoms[a].chain, 'A');
  // Ball and stick: the zinc gets no bond.
  p = mol::view_params();
  v = mol::build_view(app, m, p);
  for (const mol::bond &b : v.bonds)
    EXPECT_FALSE(b.a == 25 || b.b == 25);
  // Hydrogens off.
  mol::molecule h = *m;
  h.atoms.push_back(make_atom("H", "H", 0.0, 0.0, 1.0));
  p.hydrogens = false;
  EXPECT_EQ(mol::build_view(app, h, p).atoms_drawn, m->atoms.size());
}

TEST(MolView, TessellatedSpheres) {
  cvc::app app;
  const auto m = peptides();
  mol::view_params p;
  p.style = mol::view_style::spacefill;
  p.sphere_mesh = true;
  p.sphere_detail = 0;
  mol::molecule_view v = mol::build_view(app, m, p);
  EXPECT_EQ(v.spheres.points().size(), 12 * m->atoms.size());
  EXPECT_EQ(v.spheres.tris().size(), 20 * m->atoms.size());
  p.sphere_detail = 1;
  v = mol::build_view(app, m, p);
  EXPECT_EQ(v.spheres.points().size(), 42 * m->atoms.size());
  EXPECT_EQ(v.spheres.tris().size(), 80 * m->atoms.size());
  expect_sound_mesh(v.spheres, v.bounds, "spheres");
  // The impostors' data is there too, the same either way.
  EXPECT_EQ(v.radii.size(), m->atoms.size());
}

TEST(MolView, ACartoonOfThePeptides) {
  cvc::app app;
  const auto m = peptides();
  mol::view_params p;
  p.style = mol::view_style::cartoon;
  p.subdivisions = 4;
  p.sides = 8;
  const mol::molecule_view v = mol::build_view(app, m, p);
  // Each chain: 2 segments of 4 rings and the last residue's, 9 rings of 8, plus two caps of 9.
  EXPECT_EQ(v.residues, 6u);
  EXPECT_EQ(v.cartoon.points().size(), 2u * (9 * 8 + 2 * 9));
  EXPECT_EQ(v.cartoon.tris().size(), 2u * (8 * 8 * 2 + 2 * 8));
  expect_sound_mesh(v.cartoon, v.bounds, "cartoon");
}

TEST(MolView, PickingASphere) {
  cvc::app app;
  const auto m = peptides();
  mol::view_params p;
  p.style = mol::view_style::spacefill;
  const mol::molecule_view v = mol::build_view(app, m, p);
  // Straight down onto chain A's first C-alpha (at the origin, radius 1.7).
  const double o[3] = {0.0, 0.0, 100.0}, d[3] = {0.0, 0.0, -1.0};
  double t = 0.0, hit[3];
  const int a = mol::pick_sphere(v, o, d, &t, hit);
  ASSERT_EQ(a, 1);
  EXPECT_NEAR(t, 100.0 - 1.7, 1e-6); // radii are floats
  EXPECT_NEAR(hit[2], 1.7, 1e-6);    // radii are floats
  // From inside a sphere (the water's, alone), the way out.
  const double inside[3] = {0.0, 50.0, 0.0};
  EXPECT_EQ(mol::pick_sphere(v, inside, d, &t), 24);
  EXPECT_NEAR(t, 1.7, 1e-6); // radii are floats
  // A miss.
  const double away[3] = {500.0, 500.0, 100.0};
  EXPECT_EQ(mol::pick_sphere(v, away, d), -1);
  // Nearest drawn atom.
  const double q[3] = {7.7, 0.1, 0.0};
  EXPECT_EQ(mol::nearest_drawn_atom(*m, v, q), 9); // chain A's third C-alpha
}

TEST(MolView, BadParameters) {
  cvc::app app;
  const auto m = peptides();
  mol::view_params p;
  p.atom_scale = 0.0;
  EXPECT_THROW(mol::build_view(app, m, p), std::invalid_argument);
  p = mol::view_params();
  p.b_range = {5.0};
  EXPECT_THROW(mol::build_view(app, m, p), std::invalid_argument);
  p.b_range = {5.0, 1.0};
  try {
    mol::build_view(app, m, p);
    FAIL() << "no throw";
  } catch (const std::invalid_argument &e) {
    EXPECT_NE(std::string(e.what()).find("b_range"), std::string::npos) << e.what();
  }
  EXPECT_THROW(mol::build_view(app, std::shared_ptr<const mol::molecule>(), {}),
               std::invalid_argument);
  // A failed stepper rethrows from later steps and from take().
  p = mol::view_params();
  p.sides = 2;
  mol::build_view_stepper s(app, m, p);
  EXPECT_THROW(s.step(0), std::invalid_argument);
  EXPECT_STREQ(s.phase(), "failed");
  EXPECT_THROW(s.step(1), std::invalid_argument);
  EXPECT_THROW(s.take(), std::invalid_argument);
  // An empty molecule draws nothing, without an error.
  const mol::molecule_view e = mol::build_view(app, mol::molecule(), {});
  EXPECT_EQ(e.atoms_drawn, 0u);
  EXPECT_EQ(e.bounds, (std::array<double, 6>{0, 0, 0, 0, 0, 0}));
}

TEST(MolView, BudgetsGiveTheSameView) {
  cvc::app app;
  const auto m = grl();
  for (mol::view_style style : {mol::view_style::ball_and_stick, mol::view_style::cartoon}) {
    mol::view_params p;
    p.style = style;
    p.sphere_mesh = true;
    p.color_by = mol::color_scheme::b_factor;
    const mol::molecule_view whole = mol::build_view(app, m, p);
    const std::set<std::string> phases = {"selecting", "bonding", "spheres",
                                          "sticks",    "cartoon", "done"};
    std::mt19937 rng(11);
    for (int mode = 0; mode < 3; ++mode) {
      mol::build_view_stepper s(app, m, p);
      double last = 0.0;
      std::size_t steps = 0;
      for (;;) {
        const std::size_t budget = mode == 0 ? 1 : mode == 1 ? 64 : 1 + rng() % 3000;
        const bool done = s.step(budget);
        ++steps;
        EXPECT_TRUE(phases.count(s.phase())) << s.phase();
        const double pr = s.progress();
        EXPECT_GE(pr, last);
        EXPECT_LE(pr, 1.0);
        last = pr;
        if (done)
          break;
      }
      EXPECT_EQ(s.progress(), 1.0);
      EXPECT_GT(steps, 2u);
      expect_same_view(s.take(), whole);
    }
  }
}

TEST(MolView, Cancel) {
  cvc::app app;
  const auto m = grl();
  mol::build_view_stepper s(app, m, {});
  EXPECT_FALSE(s.step(100));
  s.cancel();
  EXPECT_TRUE(s.step(100));
  EXPECT_STREQ(s.phase(), "cancelled");
  EXPECT_FALSE(s.done());
  EXPECT_THROW(s.take(), std::logic_error);
  // The job form: a cancel leaves no result.
  std::unique_ptr<cvc::result_job<mol::molecule_view>> job = mol::build_view_job(app, m, {});
  job->step(10);
  job->cancel();
  job->step(10);
  EXPECT_TRUE(job->cancelled());
  EXPECT_FALSE(job->has_result());
  // Run to the end, it gives build_view's.
  job = mol::build_view_job(app, m, {});
  while (!job->step(500)) {
  }
  expect_same_view(job->take_result(), mol::build_view(app, m, {}));
}

TEST(MolView, TwoAppsAtOnce) {
  const auto m = grl();
  mol::view_params p;
  p.style = mol::view_style::cartoon;
  cvc::app a, b;
  mol::view_params q;
  q.style = mol::view_style::ball_and_stick;
  q.color_by = mol::color_scheme::chain;
  const mol::molecule_view alone_a = mol::build_view(a, m, p);
  const mol::molecule_view alone_b = mol::build_view(b, m, q);
  mol::molecule_view va, vb;
  std::thread ta([&] { va = mol::build_view(a, m, p); });
  std::thread tb([&] { vb = mol::build_view(b, m, q); });
  ta.join();
  tb.join();
  expect_same_view(va, alone_a);
  expect_same_view(vb, alone_b);
  // B then A gives the same as A then B.
  expect_same_view(mol::build_view(b, m, p), alone_a);
}

TEST(MolView, ParametersOnTheTree) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("mol_view_test.node");
  // Missing keys are seeded with the defaults and described.
  cvc::param_result<mol::view_params> r = cvc::params_from_state<mol::view_params>(node);
  EXPECT_TRUE(r.errors.empty());
  EXPECT_EQ(node("style").value(), "ball_and_stick");
  EXPECT_EQ(node("color_by").value(), "element");
  EXPECT_FALSE(node("atom_scale").property("doc").empty());
  // A struct round-trips.
  mol::view_params p;
  p.style = mol::view_style::cartoon;
  p.color_by = mol::color_scheme::b_factor;
  p.atoms = mol::atom_set::hetero;
  p.water = false;
  p.bond_radius = 0.2;
  p.b_range = {5.0, 40.0};
  p.subdivisions = 5;
  cvc::params_to_state(p, node);
  r = cvc::params_from_state<mol::view_params>(node);
  ASSERT_TRUE(r.errors.empty());
  EXPECT_EQ(r.value.style, p.style);
  EXPECT_EQ(r.value.color_by, p.color_by);
  EXPECT_EQ(r.value.atoms, p.atoms);
  EXPECT_EQ(r.value.water, p.water);
  EXPECT_EQ(r.value.bond_radius, p.bond_radius);
  EXPECT_EQ(r.value.b_range, p.b_range);
  EXPECT_EQ(r.value.subdivisions, p.subdivisions);
  // A value that does not parse is reported, not replaced.
  node("style").value(std::string("ribbons"));
  r = cvc::params_from_state<mol::view_params>(node);
  ASSERT_FALSE(r.errors.empty());
  EXPECT_NE(r.errors[0].find("style"), std::string::npos);
  EXPECT_EQ(node("style").value(), "ribbons");
}
