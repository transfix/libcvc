/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::geometry face materials and cvc::height_field: the first step of one
// world representation (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G1).

#include <boost/any.hpp>
#include <cmath>
#include <cvc/core/exception.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/state/state_codec_registry.h>
#include <cvc/state/state_volume_codec.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <random>
#include <vector>

using cvc::geometry;
using cvc::height_field;

namespace {

const float kHole = std::numeric_limits<float>::quiet_NaN();

geometry::point_t pt(double x, double y, double z) {
  geometry::point_t p;
  p[0] = x;
  p[1] = y;
  p[2] = z;
  return p;
}

geometry::tri_t tri(cvc::index_t a, cvc::index_t b, cvc::index_t c) {
  geometry::tri_t t;
  t[0] = a;
  t[1] = b;
  t[2] = c;
  return t;
}

geometry::quad_t quad(cvc::index_t a, cvc::index_t b, cvc::index_t c, cvc::index_t d) {
  geometry::quad_t q;
  q[0] = a;
  q[1] = b;
  q[2] = c;
  q[3] = d;
  return q;
}

// Two triangles and a quad over six points, with materials 1, 2 (tris) and 3 (quad).
geometry tris_and_quad(bool with_materials) {
  geometry g;
  for (int i = 0; i < 6; ++i)
    g.points().push_back(pt(i, i % 2, 0));
  g.tris().push_back(tri(0, 1, 2));
  g.tris().push_back(tri(1, 2, 3));
  g.quads().push_back(quad(2, 3, 4, 5));
  if (with_materials)
    g.face_materials() = {1, 2, 3};
  return g;
}

// A 4 x 5 grid over [10, 14] x [-2, 1] with uneven heights and a material per cell.
height_field sample_field() {
  std::vector<float> h;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 5; ++c)
      h.push_back(0.25f * r * r - 0.5f * c + 0.125f * (r * c % 3));
  std::vector<cvc::face_material_t> m;
  for (int k = 0; k < 3 * 4; ++k)
    m.push_back(static_cast<cvc::face_material_t>(10 + k));
  return height_field(4, 5, 10.0, -2.0, 1.0, 1.0, h, m);
}

// z of the mesh at (x, y) by brute force: the triangle whose xy-projection holds
// the point, by barycentric coordinates. NaN if none does.
double mesh_height(const geometry &g, double x, double y) {
  for (const auto &t : g.tris()) {
    const auto &a = g.points()[t[0]], &b = g.points()[t[1]], &c = g.points()[t[2]];
    const double d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    const double l1 = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d;
    const double l2 = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d;
    const double l3 = 1.0 - l1 - l2;
    const double e = -1e-12;
    if (l1 >= e && l2 >= e && l3 >= e)
      return l1 * a[2] + l2 * b[2] + l3 * c[2];
  }
  return std::numeric_limits<double>::quiet_NaN();
}

} // namespace

// --- face materials --------------------------------------------------------

TEST(GeometryFaceMaterials, CopiesShareThenDetach) {
  geometry g = tris_and_quad(true);
  EXPECT_EQ(g.num_faces(), 3u);
  EXPECT_TRUE(g.has_face_materials());
  geometry copy = g;
  copy.face_materials()[0] = 9;
  EXPECT_EQ(g.face_material(0), 1u) << "copy-on-write: the original keeps its materials";
  EXPECT_EQ(copy.face_material(0), 9u);
  geometry deep;
  deep.copy(g, true);
  EXPECT_EQ(deep.face_material(2), 3u);
}

TEST(GeometryFaceMaterials, AStaleArrayReadsAsAbsent) {
  geometry g = tris_and_quad(true);
  g.tris().push_back(tri(3, 4, 5)); // a face added without a material
  EXPECT_FALSE(g.has_face_materials());
  EXPECT_EQ(g.face_material(0), 0u);
  geometry none = tris_and_quad(false);
  EXPECT_FALSE(none.has_face_materials());
  EXPECT_EQ(none.face_material(1), 0u);
}

TEST(GeometryFaceMaterials, MergeKeepsTheMergedFaceOrder) {
  geometry a = tris_and_quad(true); // tris 1, 2; quad 3
  geometry b = tris_and_quad(false);
  b.face_materials().clear();
  a.merge(b);
  // [a's tris, b's tris, a's quad, b's quad]; b had none, so 0.
  ASSERT_TRUE(a.has_face_materials());
  EXPECT_EQ(a.face_materials(), (geometry::face_materials_t{1, 2, 0, 0, 3, 0}));
  // b's indices were offset and its points appended, as before.
  EXPECT_EQ(a.num_points(), 12u);
  EXPECT_EQ(a.tris()[2][0], 6u);

  geometry c = tris_and_quad(false), d = tris_and_quad(false);
  c.merge(d);
  EXPECT_TRUE(c.face_materials().empty()) << "no materials on either side: still none";
}

TEST(GeometryFaceMaterials, TriSurfaceGivesASplitQuadsMaterialToBothHalves) {
  geometry g = tris_and_quad(true);
  geometry s = g.tri_surface();
  ASSERT_EQ(s.num_tris(), 4u);
  EXPECT_EQ(s.face_materials(), (geometry::face_materials_t{1, 2, 3, 3}));

  // With boundary flags, only faces whose vertices are all on the boundary stay.
  geometry b = tris_and_quad(true);
  b.boundary().resize(6);
  for (int i = 1; i < 6; ++i)
    b.boundary()[i] = true; // vertex 0 is interior: the first triangle goes
  geometry sb = b.tri_surface();
  ASSERT_EQ(sb.num_tris(), 3u);
  EXPECT_EQ(sb.face_materials(), (geometry::face_materials_t{2, 3, 3}));
}

TEST(GeometryFaceMaterials, TheStateCodecCarriesThem) {
  cvc::state_codec_registry reg;
  cvc::register_volume_geometry_codecs(reg);
  const geometry g = tris_and_quad(true);
  const auto bytes = reg.encode("cvc::geometry", boost::any(g));
  const auto back = boost::any_cast<geometry>(reg.decode("cvc::geometry", bytes));
  EXPECT_EQ(back.const_face_materials(), g.const_face_materials());
  EXPECT_EQ(back.num_quads(), 1u);

  // Without materials there is no trailer, and a blob written before the
  // trailer existed (cut where it would start) still decodes.
  const auto plain = reg.encode("cvc::geometry", boost::any(tris_and_quad(false)));
  EXPECT_EQ(plain.size(), bytes.size() - 4 - 8 - 2 * 3);
  const std::vector<unsigned char> old(bytes.begin(), bytes.begin() + plain.size());
  const auto old_back = boost::any_cast<geometry>(reg.decode("cvc::geometry", old));
  EXPECT_TRUE(old_back.const_face_materials().empty());
  EXPECT_EQ(old_back.num_tris(), 2u);

  // A height field travels as its mesh, materials included.
  const geometry mesh = sample_field();
  const auto hf = boost::any_cast<geometry>(
      reg.decode("cvc::geometry", reg.encode("cvc::geometry", boost::any(mesh))));
  EXPECT_EQ(hf.num_tris(), 24u);
  EXPECT_EQ(hf.face_material(23), mesh.face_material(23));
}

TEST(GeometryFaceMaterials, SubtypesDestroyThroughTheBase) {
  std::unique_ptr<geometry> g(new height_field(2, 2, 0, 0, 1, 1));
  EXPECT_EQ(g->num_tris(), 2u); // and the destructor runs height_field's (ASan/leak checks)
}

// --- height_field -----------------------------------------------------------

TEST(HeightField, TheMeshIsTheGrid) {
  const height_field t = sample_field();
  EXPECT_EQ(t.rows(), 4);
  EXPECT_EQ(t.cols(), 5);
  EXPECT_DOUBLE_EQ(t.max_x(), 14.0);
  EXPECT_DOUBLE_EQ(t.max_y(), 1.0);
  ASSERT_EQ(t.num_points(), 20u);
  ASSERT_EQ(t.num_tris(), 2u * 3 * 4);
  EXPECT_EQ(t.get_geometry_type(), geometry::SURFACE_TRI);
  // Sample (r, c) at (x0 + c dx, y0 + r dy), row 0 at the minimum y.
  const auto &p = t.points()[t.vertex_of(2, 3)];
  EXPECT_DOUBLE_EQ(p[0], 13.0);
  EXPECT_DOUBLE_EQ(p[1], 0.0);
  EXPECT_EQ(p[2], t.height_at(2, 3));
  EXPECT_DOUBLE_EQ(t.uvs()[t.vertex_of(3, 4)][0], 1.0);
  EXPECT_DOUBLE_EQ(t.uvs()[t.vertex_of(3, 4)][1], 1.0);
  ASSERT_EQ(t.normals().size(), t.num_points());
  // Every triangle faces up (counter-clockwise seen from above) and carries its
  // cell's material.
  ASSERT_TRUE(t.has_face_materials());
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) {
      const auto f = t.first_face_of(r, c);
      ASSERT_GE(f, 0);
      for (int k = 0; k < 2; ++k) {
        const auto &tr = t.tris()[f + k];
        const auto &a = t.points()[tr[0]], &b = t.points()[tr[1]], &d = t.points()[tr[2]];
        const double z = (b[0] - a[0]) * (d[1] - a[1]) - (b[1] - a[1]) * (d[0] - a[0]);
        EXPECT_GT(z, 0.0);
        EXPECT_EQ(t.face_material(f + k), t.cell_material(r, c));
      }
    }
  for (const auto &n : t.normals())
    EXPECT_GT(n[2], 0.0);
}

TEST(HeightField, HeightIsTheMeshHeightEverywhere) {
  const height_field t = sample_field();
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ux(10.0, 14.0), uy(-2.0, 1.0);
  for (int i = 0; i < 500; ++i) {
    const double x = ux(rng), y = uy(rng);
    EXPECT_NEAR(t.height(x, y), mesh_height(t, x, y), 1e-9) << x << ", " << y;
  }
  // Exact at the samples, and on the diagonal the mean of its two ends.
  EXPECT_DOUBLE_EQ(t.height(12.0, -1.0), t.height_at(1, 2));
  EXPECT_DOUBLE_EQ(t.height(12.5, -0.5), 0.5 * (t.height_at(1, 2) + t.height_at(2, 3)));
  // The last row and column of samples belong to the last cell.
  EXPECT_DOUBLE_EQ(t.height(14.0, 1.0), t.height_at(3, 4));
  EXPECT_TRUE(std::isnan(t.height(14.001, 0.0)));
  EXPECT_TRUE(std::isnan(t.height(9.999, 0.0)));
}

TEST(HeightField, NormalIsTheTrianglesNormal) {
  const height_field t = sample_field();
  const double x = 11.7, y = -1.6; // below the diagonal of cell (0, 1): SW-SE-NE
  const auto n = t.normal(x, y);
  const auto f = t.first_face_of(0, 1);
  const auto &tr = t.tris()[f];
  const auto &a = t.points()[tr[0]], &b = t.points()[tr[1]], &c = t.points()[tr[2]];
  double u[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
  double v[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
  double w[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
  const double len = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(n[i], w[i] / len, 1e-12);
  EXPECT_TRUE(std::isnan(t.normal(20.0, 0.0)[2]));
}

TEST(HeightField, HolesHaveNoVerticesAndNoCells) {
  std::vector<float> h(16, 1.0f);
  h[1 * 4 + 1] = kHole; // sample (1, 1): the four cells around it go
  const height_field t(4, 4, 0, 0, 1, 1, h);
  EXPECT_EQ(t.num_points(), 15u);
  EXPECT_EQ(t.vertex_of(1, 1), -1);
  EXPECT_EQ(t.num_tris(), 2u * (9 - 4));
  EXPECT_EQ(t.first_face_of(0, 0), -1);
  EXPECT_EQ(t.first_face_of(1, 1), -1);
  EXPECT_GE(t.first_face_of(2, 2), 0);
  EXPECT_TRUE(std::isnan(t.height(0.5, 0.5)));
  EXPECT_DOUBLE_EQ(t.height(2.5, 2.5), 1.0);
  for (const auto &tr : t.tris())
    for (int k = 0; k < 3; ++k)
      EXPECT_LT(tr[k], t.num_points());
}

TEST(HeightField, InPlaceEditsMatchAFreshBuild) {
  height_field t = sample_field();
  t.set_height(2, 2, 5.0f);
  t.set_cell_material(1, 3, 99);
  std::vector<float> h = t.heights();
  std::vector<cvc::face_material_t> m = t.cell_materials();
  const height_field fresh(4, 5, 10.0, -2.0, 1.0, 1.0, h, m);
  ASSERT_EQ(t.num_points(), fresh.num_points());
  for (std::size_t i = 0; i < t.num_points(); ++i)
    for (int k = 0; k < 3; ++k) {
      EXPECT_EQ(t.const_points()[i][k], fresh.points()[i][k]);
      EXPECT_NEAR(t.const_normals()[i][k], fresh.normals()[i][k], 1e-15) << "vertex " << i;
    }
  EXPECT_EQ(t.const_face_materials(), fresh.face_materials());
  EXPECT_EQ(t.material(13.5, -0.5), 99u);
  EXPECT_DOUBLE_EQ(t.height(12.0, 0.0), 5.0);

  // Making a hole, or filling one, changes the mesh's topology: a rebuild.
  t.set_height(0, 0, kHole);
  EXPECT_EQ(t.num_points(), 19u);
  EXPECT_EQ(t.first_face_of(0, 0), -1);
  t.set_height(0, 0, 0.0f);
  EXPECT_EQ(t.num_points(), 20u);
  EXPECT_GE(t.first_face_of(0, 0), 0);

  // A field without materials gains them from the first non-zero one.
  height_field plain(3, 3, 0, 0, 1, 1);
  EXPECT_FALSE(plain.has_face_materials());
  plain.set_cell_material(1, 0, 4);
  ASSERT_TRUE(plain.has_face_materials());
  EXPECT_EQ(plain.material(0.5, 1.5), 4u);
  EXPECT_EQ(plain.material(1.5, 1.5), 0u);
}

TEST(HeightField, TheMeshIsReadOnlyButCopiesAreNot) {
  height_field t = sample_field();
  EXPECT_THROW(t.points(), cvc::read_only_error);
  EXPECT_THROW(t.tris(), cvc::read_only_error);
  EXPECT_THROW(t.face_materials(), cvc::read_only_error);
  EXPECT_THROW(t.merge(t), cvc::read_only_error);
  // Per-vertex colours stay writable (say, a satellite tint) and survive an
  // in-place edit and a rebuild that keeps the vertex count.
  t.colors().assign(t.num_points(), geometry::color_t{{0.2, 0.4, 0.6}});
  t.set_height(1, 1, 2.0f);
  t.set_cell_materials(std::vector<cvc::face_material_t>(12, 5));
  EXPECT_EQ(t.colors().size(), t.num_points());
  t.set_height(3, 3, kHole); // one vertex fewer: the colours no longer line up
  EXPECT_TRUE(t.colors().empty());

  // A plain geometry copy is an ordinary mesh; editing it leaves the field alone.
  geometry g = t;
  const double z = t.const_points()[0][2];
  g.points()[0][2] = 100.0;
  EXPECT_EQ(t.const_points()[0][2], z);
  EXPECT_EQ(g.num_tris(), t.num_tris());
  // A height_field copy is a height field, sharing until one of them changes.
  height_field u = t;
  u.set_height(0, 0, 7.0f);
  EXPECT_NE(t.height_at(0, 0), 7.0f);
  EXPECT_EQ(u.height(10.0, -2.0), 7.0);
  u.clear();
  EXPECT_EQ(u.rows(), 0);
  EXPECT_TRUE(u.empty());
  EXPECT_TRUE(std::isnan(u.height(10.0, -2.0)));
}

TEST(HeightField, BadGridsAreRefused) {
  EXPECT_THROW(height_field(1, 5, 0, 0, 1, 1), cvc::geometry_error);
  EXPECT_THROW(height_field(3, 3, 0, 0, 0.0, 1), cvc::geometry_error);
  EXPECT_THROW(height_field(3, 3, 0, 0, 1, std::nan("")), cvc::geometry_error);
  EXPECT_THROW(height_field(3, 3, 0, 0, 1, 1, std::vector<float>(8)), cvc::geometry_error);
  EXPECT_THROW(height_field(3, 3, 0, 0, 1, 1, {}, std::vector<cvc::face_material_t>(3)),
               cvc::geometry_error);
  std::vector<float> inf(9, 0.0f);
  inf[4] = std::numeric_limits<float>::infinity();
  EXPECT_THROW(height_field(3, 3, 0, 0, 1, 1, inf), cvc::geometry_error);
  height_field t(3, 3, 0, 0, 1, 1);
  EXPECT_THROW(t.set_height(3, 0, 1.0f), cvc::index_out_of_bounds);
  EXPECT_THROW(t.set_cell_material(0, 2, 1), cvc::index_out_of_bounds);
  EXPECT_THROW(t.set_heights(std::vector<float>(4)), cvc::geometry_error);

  const height_field b = height_field::from_bounds(5, 3, -10, 0, 10, 40);
  EXPECT_DOUBLE_EQ(b.dx(), 10.0);
  EXPECT_DOUBLE_EQ(b.dy(), 10.0);
  EXPECT_DOUBLE_EQ(b.max_x(), 10.0);
  EXPECT_DOUBLE_EQ(b.max_y(), 40.0);
}
