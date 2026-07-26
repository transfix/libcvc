// Tests for cvc::model (Phase-3 mesh/model surface): the value type
// (merged()/extents()), the model_file_io registry dispatch, and the
// Assimp-backed loader (guarded on CVC_ENABLE_ASSIMP) for OBJ (UVs + material +
// texture), the read_geometry() flatten path, and STL (no UVs).

#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <fstream>
#include <gtest/gtest.h>
#include <string>

#ifdef CVC_ENABLE_ASSIMP
#include <cvc/image/image.h>
#endif

using cvc::geometry;
using cvc::model;

namespace {

// A single triangle in the z=0 plane with indices (0,1,2).
geometry make_triangle() {
  geometry g;
  cvc::point_t p;
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  g.points().push_back(p);
  p[0] = 1;
  p[1] = 0;
  p[2] = 0;
  g.points().push_back(p);
  p[0] = 0;
  p[1] = 1;
  p[2] = 0;
  g.points().push_back(p);
  cvc::tri_t t;
  t[0] = 0;
  t[1] = 1;
  t[2] = 2;
  g.tris().push_back(t);
  return g;
}

// A non-planar 4-point cloud whose bbox is min..max (has non-zero volume, so the
// bounding_box union does not treat it as the null/identity box).
geometry make_box_corner(double ox, double oy, double oz) {
  geometry g;
  const double pts[4][3] = {{ox, oy, oz}, {ox + 1, oy, oz}, {ox, oy + 1, oz}, {ox, oy, oz + 1}};
  for (int i = 0; i < 4; ++i) {
    cvc::point_t p;
    p[0] = pts[i][0];
    p[1] = pts[i][1];
    p[2] = pts[i][2];
    g.points().push_back(p);
  }
  return g;
}

} // namespace

// ── value-type tests (no Assimp needed) ──────────────────────────────────────

TEST(ModelTest, DefaultIsEmpty) {
  model m;
  EXPECT_TRUE(m.empty());
  EXPECT_EQ(m.num_meshes(), 0u);
}

TEST(ModelTest, MergedOffsetsIndices) {
  model m;
  model::mesh a;
  a.geom = make_triangle();
  model::mesh b;
  b.geom = make_triangle();
  m.meshes.push_back(a);
  m.meshes.push_back(b);
  EXPECT_EQ(m.num_meshes(), 2u);

  geometry merged = m.merged();
  ASSERT_EQ(merged.num_points(), 6u);
  ASSERT_EQ(merged.num_tris(), 2u);
  // First mesh's triangle is unshifted.
  EXPECT_EQ(merged.const_tris()[0][0], 0u);
  EXPECT_EQ(merged.const_tris()[0][1], 1u);
  EXPECT_EQ(merged.const_tris()[0][2], 2u);
  // Second mesh's triangle is offset by the running vertex count (3).
  EXPECT_EQ(merged.const_tris()[1][0], 3u);
  EXPECT_EQ(merged.const_tris()[1][1], 4u);
  EXPECT_EQ(merged.const_tris()[1][2], 5u);
}

TEST(ModelTest, ExtentsUnion) {
  model m;
  model::mesh a;
  a.geom = make_box_corner(0, 0, 0); // bbox 0,0,0 .. 1,1,1
  model::mesh b;
  b.geom = make_box_corner(2, 2, 2); // bbox 2,2,2 .. 3,3,3
  m.meshes.push_back(a);
  m.meshes.push_back(b);

  cvc::bounding_box bb = m.extents();
  EXPECT_DOUBLE_EQ(bb.XMin(), 0.0);
  EXPECT_DOUBLE_EQ(bb.YMin(), 0.0);
  EXPECT_DOUBLE_EQ(bb.ZMin(), 0.0);
  EXPECT_DOUBLE_EQ(bb.XMax(), 3.0);
  EXPECT_DOUBLE_EQ(bb.YMax(), 3.0);
  EXPECT_DOUBLE_EQ(bb.ZMax(), 3.0);
}

TEST(ModelTest, UnsupportedExtensionThrows) {
  EXPECT_ANY_THROW(cvc::read_model("/no/such/file.qwerty"));
}

// ── Assimp-backed loader tests ───────────────────────────────────────────────

#ifdef CVC_ENABLE_ASSIMP

namespace {

// Write a small OBJ + MTL + PNG texture into TempDir and return the .obj path.
// Returns "" (with a skip flag set) if the PNG could not be written (no image
// delegate) — the caller GTEST_SKIP()s in that case.
std::string write_obj_fixture(bool &texture_ok) {
  const std::string dir = ::testing::TempDir();
  const std::string obj = dir + "cvc_model_test_mesh.obj";
  const std::string mtl = dir + "cvc_model_test_mesh.mtl";
  const std::string png = dir + "cvc_model_test_tex.png";

  // 4x4 RGBA texture via cvc::image (needs the ImageMagick handler).
  texture_ok = false;
  try {
    cvc::image tex(4, 4, cvc::image::pixel_format::RGBA, cvc::image::data_type::u8);
    unsigned char *p = tex.data();
    for (int i = 0; i < 4 * 4; ++i) {
      p[i * 4 + 0] = 200;
      p[i * 4 + 1] = 100;
      p[i * 4 + 2] = 50;
      p[i * 4 + 3] = 255;
    }
    tex.save(png);
    texture_ok = true;
  } catch (const std::exception &) {
    texture_ok = false;
  }

  {
    std::ofstream om(mtl.c_str());
    om << "newmtl cvc_mat\n";
    om << "Kd 0.8 0.2 0.1\n";
    om << "d 1.0\n";
    om << "map_Kd cvc_model_test_tex.png\n";
  }
  {
    std::ofstream oo(obj.c_str());
    oo << "mtllib cvc_model_test_mesh.mtl\n";
    oo << "v 0 0 0\n";
    oo << "v 1 0 0\n";
    oo << "v 0 1 0\n";
    oo << "vt 0 0\n";
    oo << "vt 1 0\n";
    oo << "vt 0 1\n";
    oo << "usemtl cvc_mat\n";
    oo << "f 1/1 2/2 3/3\n";
  }
  return obj;
}

bool uv_present(const geometry::uvs_t &uvs, double u, double v) {
  for (std::size_t i = 0; i < uvs.size(); ++i)
    if (std::abs(uvs[i][0] - u) < 1e-5 && std::abs(uvs[i][1] - v) < 1e-5)
      return true;
  return false;
}

} // namespace

TEST(ModelTest, RegistryHasObjExtension) {
  std::vector<std::string> exts = cvc::model_file_io::get_extensions();
  bool has_obj = false;
  for (std::size_t i = 0; i < exts.size(); ++i)
    if (exts[i] == ".obj")
      has_obj = true;
  EXPECT_TRUE(has_obj);
}

TEST(ModelTest, ObjWithUvsMaterialTexture) {
  bool texture_ok = false;
  std::string obj = write_obj_fixture(texture_ok);
  if (!texture_ok)
    GTEST_SKIP() << "No image delegate available to write the OBJ texture PNG";

  model m = cvc::read_model(obj);
  ASSERT_GE(m.num_meshes(), 1u);
  const model::mesh &mesh = m.meshes[0];

  EXPECT_EQ(mesh.geom.num_points(), 3u);
  EXPECT_EQ(mesh.geom.num_tris(), 1u);

  // UVs present and matching the authored (unflipped) coordinates.
  const geometry::uvs_t &uvs = mesh.geom.const_uvs();
  ASSERT_EQ(uvs.size(), 3u);
  EXPECT_TRUE(uv_present(uvs, 0, 0));
  EXPECT_TRUE(uv_present(uvs, 1, 0));
  EXPECT_TRUE(uv_present(uvs, 0, 1));

  // Material: diffuse Kd -> base_color rgb, d=1 -> alpha, texture path + image.
  ASSERT_GE(mesh.material, 0);
  ASSERT_LT(static_cast<std::size_t>(mesh.material), m.materials.size());
  const cvc::material &mat = m.materials[mesh.material];
  EXPECT_NEAR(mat.base_color[0], 0.8, 1e-3);
  EXPECT_NEAR(mat.base_color[1], 0.2, 1e-3);
  EXPECT_NEAR(mat.base_color[2], 0.1, 1e-3);
  EXPECT_NEAR(mat.base_color[3], 1.0, 1e-3);
  EXPECT_EQ(mat.base_color_texture_path, "cvc_model_test_tex.png");
  ASSERT_TRUE(mat.has_base_color_texture());
  EXPECT_EQ(mat.base_color_texture.width(), 4);
  EXPECT_EQ(mat.base_color_texture.height(), 4);
}

TEST(ModelTest, ReadGeometryObjFlatten) {
  bool texture_ok = false;
  std::string obj = write_obj_fixture(texture_ok);
  // The flatten path does not require the texture; only the mesh matters.
  geometry g = cvc::read_geometry(obj);
  EXPECT_EQ(g.num_points(), 3u);
  EXPECT_EQ(g.num_tris(), 1u);
}

TEST(ModelTest, StlNoUvs) {
  const std::string dir = ::testing::TempDir();
  const std::string stl = dir + "cvc_model_test_tri.stl";
  {
    std::ofstream os(stl.c_str());
    os << "solid cvc\n";
    os << " facet normal 0 0 1\n";
    os << "  outer loop\n";
    os << "   vertex 0 0 0\n";
    os << "   vertex 1 0 0\n";
    os << "   vertex 0 1 0\n";
    os << "  endloop\n";
    os << " endfacet\n";
    os << "endsolid cvc\n";
  }

  model m = cvc::read_model(stl);
  ASSERT_GE(m.num_meshes(), 1u);
  const model::mesh &mesh = m.meshes[0];
  EXPECT_GE(mesh.geom.num_points(), 3u);
  EXPECT_EQ(mesh.geom.num_tris(), 1u);
  EXPECT_TRUE(mesh.geom.const_uvs().empty());
}

#endif // CVC_ENABLE_ASSIMP
