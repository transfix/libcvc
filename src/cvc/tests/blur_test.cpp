/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::mol::blur, TexMol's GOABlur electron density (cvc/mol/blur.h; TexMol roadmap T1):
//   - the kernel: every grid point an atom reaches holds exp(B (r^2/R^2 - 1)) from it, with its
//     centre snapped to the nearest grid point (or not, with snap_centres off), and nothing past
//     its reach; 1 at r = R;
//   - the box (the centres padded by `padding` largest radii) and the grid (extent / resolution +
//     3 points, or `dims`);
//   - densities add, atom by atom;
//   - T7: budgets 1, 64, random and 0 give the same volume bit for bit; progress is monotone;
//     cancel ends it with no result; the result_job;
//   - T5: parameters that aren't valid, no atoms;
//   - T6: two apps blurring at once on two threads get the same volume;
//   - T1: 1GRL's atoms, grid and density are TexMol's (tools/texmol-goldens): the density to 1e-5
//     relative, most voxels bit for bit.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cvc/core/app.h>
#include <cvc/mol/blur.h>
#include <cvc/mol/molecule.h>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <thread>

namespace mol = cvc::mol;

namespace {

const std::string kRepo = CVC_REPO_DIR;
const std::string k1grl = kRepo + "/examples/data/molecules/1grl.pdb";

mol::molecule atoms(std::initializer_list<std::array<double, 4>> xyzr) {
  mol::molecule m;
  for (const auto &a : xyzr) {
    mol::atom at;
    at.x = a[0];
    at.y = a[1];
    at.z = a[2];
    at.radius = a[3];
    m.atoms.push_back(at);
  }
  return m;
}

double at(const cvc::volume &v, std::uint64_t i, std::uint64_t j, std::uint64_t k) {
  return v(i, j, k);
}

// The grid's spacing along an axis, as blur computes it (float).
float span_of(const cvc::volume &v, int axis) {
  const auto &b = v.boundingBox();
  const float lo[3] = {static_cast<float>(b.minx), static_cast<float>(b.miny),
                       static_cast<float>(b.minz)};
  const float hi[3] = {static_cast<float>(b.maxx), static_cast<float>(b.maxy),
                       static_cast<float>(b.maxz)};
  const std::uint64_t d[3] = {v.XDim(), v.YDim(), v.ZDim()};
  return (hi[axis] - lo[axis]) / static_cast<float>(d[axis] - 1);
}

// The grid point's position, as blur computes it (float).
float point(const cvc::volume &v, int axis, std::uint64_t i) {
  const auto &b = v.boundingBox();
  const float lo[3] = {static_cast<float>(b.minx), static_cast<float>(b.miny),
                       static_cast<float>(b.minz)};
  const float hi[3] = {static_cast<float>(b.maxx), static_cast<float>(b.maxy),
                       static_cast<float>(b.maxz)};
  const std::uint64_t d[3] = {v.XDim(), v.YDim(), v.ZDim()};
  const float span = (hi[axis] - lo[axis]) / static_cast<float>(d[axis] - 1);
  return lo[axis] + static_cast<float>(i) * span;
}

bool same(const cvc::volume &a, const cvc::volume &b) {
  if (a.XDim() != b.XDim() || a.YDim() != b.YDim() || a.ZDim() != b.ZDim())
    return false;
  for (std::uint64_t k = 0; k < a.ZDim(); ++k)
    for (std::uint64_t j = 0; j < a.YDim(); ++j)
      for (std::uint64_t i = 0; i < a.XDim(); ++i)
        if (at(a, i, j, k) != at(b, i, j, k))
          return false;
  return true;
}

mol::molecule heavy_1grl(cvc::app &app) {
  mol::molecule_read_params p;
  p.hydrogens = false;
  return mol::read_molecule(app, k1grl, p);
}

} // namespace

TEST(MolBlur, OneAtomIsTheKernel) {
  cvc::app app;
  const double R = 1.5, B = -2.3442;
  const mol::molecule m = atoms({{0.3, -0.2, 0.1, R}});
  mol::blur_params p;
  p.resolution = 0.5;
  const cvc::volume v = mol::blur(app, m, p);
  // The box: the centre padded by four radii; extent / resolution + 3 points.
  const auto &b = v.boundingBox();
  EXPECT_FLOAT_EQ(static_cast<float>(b.minx), 0.3f - 6.0f);
  EXPECT_FLOAT_EQ(static_cast<float>(b.maxz), 0.1f + 6.0f);
  EXPECT_EQ(v.XDim(), static_cast<std::uint64_t>(12.0 / 0.5 + 3));
  // Every point: the kernel around the snapped centre within the atom's cube, nothing outside it.
  // GOABlur splats a cube, corners and all, from int(c - reach/span - 1) to int(c + reach/span + 1)
  // (c the centre's grid index), the far end excluded.
  const double reach = R * std::sqrt(1.0 + std::log(static_cast<double>(1e-3f)) / B);
  double c[3];
  long lo_i[3], hi_i[3];
  for (int a = 0; a < 3; ++a) {
    const float lo = point(v, a, 0), span = span_of(v, a);
    const float pos[3] = {0.3f, -0.2f, 0.1f};
    const double idx = std::round((pos[a] - lo) / span);
    c[a] = lo + idx * span;
    lo_i[a] = std::max(0L, static_cast<long>(idx - reach / span - 1));
    hi_i[a] = static_cast<long>(idx + reach / span + 1);
  }
  double peak = 0;
  for (std::uint64_t k = 0; k < v.ZDim(); ++k)
    for (std::uint64_t j = 0; j < v.YDim(); ++j)
      for (std::uint64_t i = 0; i < v.XDim(); ++i) {
        const long ijk[3] = {static_cast<long>(i), static_cast<long>(j), static_cast<long>(k)};
        bool inside = true;
        for (int a = 0; a < 3; ++a)
          inside = inside && ijk[a] >= lo_i[a] && ijk[a] < hi_i[a];
        const double d[3] = {c[0] - point(v, 0, i), c[1] - point(v, 1, j), c[2] - point(v, 2, k)};
        const double r2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
        const double got = at(v, i, j, k);
        if (inside)
          EXPECT_NEAR(got, std::exp(B * (r2 / (R * R) - 1)),
                      1e-5 * std::max(1e-7, std::exp(B * (r2 / (R * R) - 1))))
              << i << ' ' << j << ' ' << k;
        else
          EXPECT_EQ(got, 0.0) << "outside the cube at " << i << ' ' << j << ' ' << k;
        peak = std::max(peak, got);
      }
  EXPECT_NEAR(peak, std::exp(-B), 1e-4); // at the centre itself, a grid point
}

TEST(MolBlur, SnapCentresOffKeepsTheCentre) {
  cvc::app app;
  const mol::molecule m = atoms({{0.3, -0.2, 0.1, 1.5}});
  mol::blur_params p;
  p.resolution = 0.5;
  p.snap_centres = false;
  const cvc::volume v = mol::blur(app, m, p);
  for (std::uint64_t k = 0; k < v.ZDim(); k += 3)
    for (std::uint64_t j = 0; j < v.YDim(); j += 3)
      for (std::uint64_t i = 0; i < v.XDim(); i += 3) {
        const double dx = 0.3f - point(v, 0, i), dy = -0.2f - point(v, 1, j),
                     dz = 0.1f - point(v, 2, k);
        const double r2 = dx * dx + dy * dy + dz * dz;
        if (r2 < 4.0)
          EXPECT_NEAR(at(v, i, j, k), std::exp(-2.3442 * (r2 / 2.25 - 1)), 1e-4);
      }
}

TEST(MolBlur, DensitiesAdd) {
  cvc::app app;
  // Two atoms of one radius; each alone is the pair with the other's radius 0, which adds nothing
  // and keeps the box (the centres and the largest radius) and so the grid.
  const mol::molecule both = atoms({{0, 0, 0, 1.4}, {1.9, 0.4, -0.3, 1.4}});
  mol::blur_params p;
  p.dims = {33, 29, 31};
  const cvc::volume v = mol::blur(app, both, p);
  mol::molecule a = both, b = both;
  a.atoms[1].radius = 0.0;
  b.atoms[0].radius = 0.0;
  const cvc::volume va = mol::blur(app, a, p), vb = mol::blur(app, b, p);
  ASSERT_EQ(va.XDim(), v.XDim());
  for (std::uint64_t k = 0; k < v.ZDim(); ++k)
    for (std::uint64_t j = 0; j < v.YDim(); ++j)
      for (std::uint64_t i = 0; i < v.XDim(); ++i)
        EXPECT_FLOAT_EQ(static_cast<float>(at(v, i, j, k)),
                        static_cast<float>(at(va, i, j, k) + at(vb, i, j, k)));
}

TEST(MolBlur, BudgetsGiveTheSameVolume) {
  cvc::app app;
  const mol::molecule m = heavy_1grl(app);
  mol::blur_params p;
  p.resolution = 1.5;
  const cvc::volume whole = mol::blur(app, m, p);
  std::mt19937 rng(7);
  for (int mode = 0; mode < 3; ++mode) {
    mol::blur_stepper s(app, m, p);
    double last = -1;
    std::string phases;
    while (true) {
      const std::size_t budget = mode == 0 ? 1 : mode == 1 ? 64 : 1 + rng() % 5000;
      const bool done = s.step(budget);
      EXPECT_GE(s.progress(), last);
      last = s.progress();
      if (phases.empty() || phases.substr(phases.rfind(' ') + 1) != s.phase())
        phases += std::string(" ") + s.phase();
      if (done)
        break;
    }
    EXPECT_EQ(phases, " checking splatting done");
    EXPECT_EQ(s.progress(), 1.0);
    EXPECT_TRUE(same(s.take(), whole)) << "mode " << mode;
  }
  auto job = mol::blur_job(app, m, p);
  while (!job->step(10000)) {
  }
  EXPECT_TRUE(same(job->take_result(), whole));
}

TEST(MolBlur, Cancel) {
  cvc::app app;
  const mol::molecule m = heavy_1grl(app);
  mol::blur_stepper s(app, m, {});
  s.step(5000);
  s.cancel();
  EXPECT_TRUE(s.step(5000));
  EXPECT_STREQ(s.phase(), "cancelled");
  EXPECT_THROW(s.take(), std::logic_error);
}

TEST(MolBlur, BadParametersAndNoAtoms) {
  cvc::app app;
  const mol::molecule m = atoms({{0, 0, 0, 1.5}});
  const auto fails = [&](mol::blur_params p) {
    EXPECT_THROW(mol::blur(app, m, p), std::invalid_argument);
  };
  mol::blur_params p;
  p.blobbiness = 0.5;
  fails(p);
  p = {};
  p.resolution = 0;
  fails(p);
  p = {};
  p.dims = {10, 10};
  fails(p);
  p = {};
  p.cutoff = 1.5;
  fails(p);
  EXPECT_THROW(mol::blur(app, mol::molecule{}, {}), std::invalid_argument);
  mol::blur_stepper s(app, mol::molecule{}, {});
  EXPECT_THROW(s.step(0), std::invalid_argument);
  EXPECT_STREQ(s.phase(), "failed");
  EXPECT_THROW(s.step(0), std::invalid_argument); // rethrown
}

TEST(MolBlur, TwoAppsAtOnce) {
  cvc::app a, b;
  const mol::molecule m = heavy_1grl(a);
  mol::blur_params p;
  p.resolution = 2.0;
  cvc::volume va(a), vb(b);
  std::thread ta([&] { va = mol::blur(a, m, p); });
  std::thread tb([&] { vb = mol::blur(b, m, p); });
  ta.join();
  tb.join();
  EXPECT_TRUE(same(va, vb));
}

// ---- T1: TexMol's own blur maps (tools/texmol-goldens) ----

namespace {

struct golden_grid {
  int dims[3] = {0, 0, 0};
  float lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
};

const std::string kGoldens = kRepo + "/src/cvc/tests/data/texmol-goldens/blur/";

golden_grid read_grid(const std::string &name) {
  std::ifstream in(kGoldens + name + ".grid.txt");
  golden_grid g;
  in >> g.dims[0] >> g.dims[1] >> g.dims[2] >> g.lo[0] >> g.lo[1] >> g.lo[2] >> g.hi[0] >>
      g.hi[1] >> g.hi[2];
  EXPECT_TRUE(in) << name << ".grid.txt";
  return g;
}

std::vector<float> read_f32(const std::string &name, std::size_t n) {
  std::vector<float> v(n);
  std::ifstream in(kGoldens + name + ".f32", std::ios::binary);
  in.read(reinterpret_cast<char *>(v.data()), static_cast<std::streamsize>(n * sizeof(float)));
  EXPECT_TRUE(in) << name << ".f32";
  return v;
}

// 1GRL as TexMol's parser read it: every atom, and TexMol's radius for one its table lacks, which
// is the atom's occupancy (addAtom's argument order; 1.00 for every atom in 1GRL).
mol::molecule texmol_1grl(cvc::app &app) {
  mol::molecule_read_params p;
  p.unknown_radius = 1.0;
  return mol::read_molecule(app, k1grl, p);
}

void expect_texmols(const std::string &name, double resolution, double blobbiness) {
  cvc::app app;
  const mol::molecule m = texmol_1grl(app);
  mol::blur_params p;
  p.resolution = resolution;
  p.blobbiness = blobbiness;
  const cvc::volume v = mol::blur(app, m, p);
  const golden_grid g = read_grid(name);
  ASSERT_EQ(v.XDim(), static_cast<std::uint64_t>(g.dims[0]));
  ASSERT_EQ(v.YDim(), static_cast<std::uint64_t>(g.dims[1]));
  ASSERT_EQ(v.ZDim(), static_cast<std::uint64_t>(g.dims[2]));
  const auto &b = v.boundingBox();
  EXPECT_FLOAT_EQ(static_cast<float>(b.minx), g.lo[0]);
  EXPECT_FLOAT_EQ(static_cast<float>(b.miny), g.lo[1]);
  EXPECT_FLOAT_EQ(static_cast<float>(b.minz), g.lo[2]);
  EXPECT_FLOAT_EQ(static_cast<float>(b.maxx), g.hi[0]);
  EXPECT_FLOAT_EQ(static_cast<float>(b.maxy), g.hi[1]);
  EXPECT_FLOAT_EQ(static_cast<float>(b.maxz), g.hi[2]);
  const std::size_t n = v.XDim() * v.YDim() * v.ZDim();
  const std::vector<float> want = read_f32(name, n);
  std::size_t exact = 0, off = 0;
  double worst = 0;
  std::size_t i = 0;
  for (std::uint64_t z = 0; z < v.ZDim(); ++z)
    for (std::uint64_t y = 0; y < v.YDim(); ++y)
      for (std::uint64_t x = 0; x < v.XDim(); ++x, ++i) {
        const float got = static_cast<float>(at(v, x, y, z));
        if (got == want[i]) {
          ++exact;
          continue;
        }
        const double rel = std::abs(got - want[i]) / std::max(1e-30, std::abs(double(want[i])));
        worst = std::max(worst, rel);
        off += rel > 1e-5;
      }
  // Most voxels are TexMol's bit for bit. The rest are float sums of the same terms in another
  // order (TexMol's tree order, TheAtomsAreTexMols): a voxel where hundreds of blobs overlap, in a
  // low-resolution map, can round some 1e-6 apart.
  std::printf("  %s: %zu of %zu voxels exactly TexMol's, worst relative difference %.3g\n",
              name.c_str(), exact, n, worst);
  EXPECT_EQ(off, 0u) << name << ": voxels more than 1e-5 off TexMol's";
  EXPECT_GT(exact, n * 9 / 10) << name;
}

} // namespace

// The same atoms with the same radii. TexMol's order differs: its group tree keeps a sheet's
// strands together (1GRL's residues 48-50 follow 38-40), and flattening walks the tree. Only the
// densities' float sums can feel that, by a rounding.
TEST(MolBlurLegacy, TheAtomsAreTexMols) {
  cvc::app app;
  const mol::molecule m = texmol_1grl(app);
  std::vector<std::array<float, 4>> ours, theirs;
  for (const mol::atom &a : m.atoms)
    ours.push_back({static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z),
                    static_cast<float>(a.radius)});
  std::ifstream in(kGoldens + "1grl.atoms.txt");
  for (float x, y, z, r; in >> x >> y >> z >> r;)
    theirs.push_back({x, y, z, r});
  std::sort(ours.begin(), ours.end());
  std::sort(theirs.begin(), theirs.end());
  ASSERT_EQ(ours.size(), theirs.size());
  EXPECT_TRUE(ours == theirs);
}

TEST(MolBlurLegacy, TexMolsDefaultBlobbiness) { expect_texmols("1grl_r1.5", 1.5, -2.3442); }

TEST(MolBlurLegacy, ALowResolutionMap) { expect_texmols("1grl_coarse", 2.0, -0.5); }
