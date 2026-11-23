// Tests for cvc::model (Phase-3 mesh/model surface): the value type
// (merged()/extents()), the model_file_io registry dispatch, and the
// Assimp-backed loader (guarded on CVC_ENABLE_ASSIMP) for OBJ (UVs + material +
// texture), the read_geometry() flatten path, and STL (no UVs). The node tree
// (model::nodes, instances(), placed()): meshes placed and repeated by their
// nodes, a mirror rewound, and glTF, glb and Collada files read with theirs.

#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/model/model.h>
#include <cvc/model/model_file_io.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

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

// The same triangle as make_triangle() but carrying per-vertex UVs (distinct
// values so a misalignment after merge is detectable).
geometry make_triangle_with_uvs() {
  geometry g = make_triangle();
  const double uvs[3][2] = {{0.10, 0.20}, {0.30, 0.40}, {0.50, 0.60}};
  for (int i = 0; i < 3; ++i) {
    cvc::uv_t uv;
    uv[0] = uvs[i][0];
    uv[1] = uvs[i][1];
    g.uvs().push_back(uv);
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

// Regression for the merged() attribute-desync blocker: geometry::merge appends
// per-vertex arrays without padding, so merging a UV'd mesh with a UV-less one
// would leave uvs shorter than points and misaligned against the offset indices.
// merged() must normalize (pad) so every present per-vertex array stays exactly
// num_points() long and index-aligned regardless of mesh order.
TEST(ModelTest, MergedPadsHeterogeneousAttributes) {
  model m;
  model::mesh a;
  a.geom = make_triangle_with_uvs(); // 3 verts WITH uvs
  model::mesh b;
  b.geom = make_triangle(); // 3 verts, NO uvs
  m.meshes.push_back(a);
  m.meshes.push_back(b);

  geometry merged = m.merged();
  ASSERT_EQ(merged.num_points(), 6u);
  // Padded to full length (would be 3 without the fix).
  ASSERT_EQ(merged.const_uvs().size(), 6u);
  // Mesh A's authored UVs stay attached to vertices 0..2.
  EXPECT_NEAR(merged.const_uvs()[0][0], 0.10, 1e-6);
  EXPECT_NEAR(merged.const_uvs()[1][0], 0.30, 1e-6);
  EXPECT_NEAR(merged.const_uvs()[2][0], 0.50, 1e-6);
  // Mesh B (no UVs) gets neutral (0,0) padding at 3..5 — NOT A's values.
  EXPECT_NEAR(merged.const_uvs()[3][0], 0.0, 1e-6);
  EXPECT_NEAR(merged.const_uvs()[4][1], 0.0, 1e-6);

  // Reverse order: a UV-less mesh first must not slide A's UVs onto wrong verts.
  model m2;
  model::mesh b2;
  b2.geom = make_triangle();
  m2.meshes.push_back(b2);
  model::mesh a2;
  a2.geom = make_triangle_with_uvs();
  m2.meshes.push_back(a2);
  geometry merged2 = m2.merged();
  ASSERT_EQ(merged2.const_uvs().size(), 6u);
  EXPECT_NEAR(merged2.const_uvs()[0][0], 0.0, 1e-6);  // padded (mesh B)
  EXPECT_NEAR(merged2.const_uvs()[3][0], 0.10, 1e-6); // mesh A vertex 0
  EXPECT_NEAR(merged2.const_uvs()[5][0], 0.50, 1e-6); // mesh A vertex 2
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

TEST(ModelTest, AuthoringUnitsDefaultToMetres) {
  model m;
  EXPECT_DOUBLE_EQ(m.metres_per_source_unit, 1.0);
  m.meshes.push_back({make_box_corner(0, 0, 0), -1, ""}); // bbox 0..1
  // With the default 1.0, extents_metres() equals extents() exactly.
  cvc::bounding_box raw = m.extents();
  cvc::bounding_box met = m.extents_metres();
  EXPECT_DOUBLE_EQ(met.XMin(), raw.XMin());
  EXPECT_DOUBLE_EQ(met.XMax(), raw.XMax());
}

TEST(ModelTest, ExtentsMetresScalesByAuthoringUnit) {
  model m;
  m.metres_per_source_unit = 0.01; // authored in centimetres
  model::mesh a;
  a.geom = make_box_corner(0, 0, 0); // 0,0,0 .. 1,1,1 in cm
  model::mesh b;
  b.geom = make_box_corner(2, 2, 2); // 2,2,2 .. 3,3,3 in cm
  m.meshes.push_back(a);
  m.meshes.push_back(b);

  cvc::bounding_box bb = m.extents_metres(); // 300 cm span -> 0.03 m
  EXPECT_DOUBLE_EQ(bb.XMin(), 0.0);
  EXPECT_DOUBLE_EQ(bb.XMax(), 0.03);
  EXPECT_DOUBLE_EQ(bb.YMax(), 0.03);
  EXPECT_DOUBLE_EQ(bb.ZMax(), 0.03);
  // The authoring-space extents are unchanged; only the metres view scales.
  EXPECT_DOUBLE_EQ(m.extents().XMax(), 3.0);
}

TEST(ModelTest, ExtentsMetresOfAnEmptyModelIsNull) {
  model m;
  m.metres_per_source_unit = 0.001;
  EXPECT_TRUE(m.extents_metres().isNull()); // no corners to scale
}

TEST(ModelTest, UnsupportedExtensionThrows) {
  cvc::app app;
  EXPECT_ANY_THROW(cvc::read_model(app, "/no/such/file.qwerty"));
}

// ── the node tree (no Assimp needed) ─────────────────────────────────────────

namespace {

model::transform_t translation(double x, double y, double z) {
  model::transform_t t = model::identity_transform();
  t[3] = x;
  t[7] = y;
  t[11] = z;
  return t;
}

model::transform_t scaling(double sx, double sy, double sz) {
  model::transform_t t = model::identity_transform();
  t[0] = sx;
  t[5] = sy;
  t[10] = sz;
  return t;
}

// A quarter turn about +z: (x, y, z) -> (-y, x, z).
model::transform_t quarter_turn_z() {
  model::transform_t t = model::identity_transform();
  t[0] = 0;
  t[1] = -1;
  t[4] = 1;
  t[5] = 0;
  return t;
}

model::node make_node(const std::string &name, const model::transform_t &t, std::vector<int> meshes,
                      std::vector<int> children = {}) {
  model::node n;
  n.name = name;
  n.transform = t;
  n.meshes = std::move(meshes);
  n.children = std::move(children);
  return n;
}

// make_triangle() facing +z, with its normals and a tangent along +x (handedness +1).
geometry make_lit_triangle() {
  geometry g = make_triangle();
  for (int i = 0; i < 3; ++i) {
    g.normals().push_back({{0, 0, 1}});
    g.tangents().push_back({{1, 0, 0, 1}});
  }
  return g;
}

bool near_point(const cvc::point_t &p, double x, double y, double z, double tol = 1e-4) {
  return std::abs(p[0] - x) < tol && std::abs(p[1] - y) < tol && std::abs(p[2] - z) < tol;
}

// Whether `g` has a point within 1e-4 of (x, y, z).
bool has_point(const geometry &g, double x, double y, double z) {
  for (const cvc::point_t &p : g.const_points())
    if (near_point(p, x, y, z))
      return true;
  return false;
}

} // namespace

TEST(ModelNodes, WithoutNodesEachMeshIsDrawnOnceAsItIs) {
  model m;
  m.meshes.push_back({make_triangle(), -1, "a"});
  m.meshes.push_back({make_box_corner(2, 2, 2), -1, "b"});
  const std::vector<model::instance> in = m.instances();
  ASSERT_EQ(in.size(), 2u);
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(in[i].mesh, i);
    EXPECT_EQ(in[i].node, -1);
    EXPECT_TRUE(model::is_identity(in[i].transform));
  }
  EXPECT_EQ(m.merged().num_points(), 7u);
}

// One mesh drawn by two nodes under a moved root: each copy is placed by its node and the root,
// its normals and tangents turned with it.
TEST(ModelNodes, NodesPlaceAndRepeatTheirMeshes) {
  model m;
  m.meshes.push_back({make_lit_triangle(), -1, "tri"});
  m.nodes.push_back(make_node("root", translation(10, 0, 0), {}, {1, 2}));
  m.nodes.push_back(make_node("left", translation(0, 5, 0), {0}));
  m.nodes.push_back(make_node("right", model::compose(quarter_turn_z(), scaling(2, 2, 2)), {0}));

  const std::vector<model::instance> in = m.instances();
  ASSERT_EQ(in.size(), 2u);
  EXPECT_EQ(in[0].node, 1);
  EXPECT_EQ(in[1].node, 2);
  EXPECT_EQ(in[0].mesh, 0);
  EXPECT_EQ(in[1].mesh, 0);

  const geometry g = m.merged();
  ASSERT_EQ(g.num_points(), 6u);
  const auto &p = g.const_points();
  EXPECT_TRUE(near_point(p[0], 10, 5, 0)); // left: root and its own translation
  EXPECT_TRUE(near_point(p[1], 11, 5, 0));
  EXPECT_TRUE(near_point(p[2], 10, 6, 0));
  EXPECT_TRUE(near_point(p[3], 10, 0, 0)); // right: doubled, turned a quarter, then moved
  EXPECT_TRUE(near_point(p[4], 10, 2, 0));
  EXPECT_TRUE(near_point(p[5], 8, 0, 0));
  ASSERT_EQ(g.const_normals().size(), 6u);
  for (const auto &n : g.const_normals()) {
    EXPECT_NEAR(n[2], 1.0, 1e-9) << "a turn about z keeps a +z normal, unit after the scale";
    EXPECT_NEAR(std::hypot(n[0], n[1]), 0.0, 1e-9);
  }
  ASSERT_EQ(g.const_tangents().size(), 6u);
  EXPECT_NEAR(g.const_tangents()[0][0], 1.0, 1e-9);
  EXPECT_NEAR(g.const_tangents()[3][1], 1.0, 1e-9) << "+x turned a quarter is +y";
  EXPECT_NEAR(g.const_tangents()[3][3], 1.0, 1e-9);
  ASSERT_EQ(g.num_tris(), 2u);
  EXPECT_EQ(g.const_tris()[1][0], 3u); // the second copy's triangle: offset, wound as authored
  EXPECT_EQ(g.const_tris()[1][1], 4u);
  EXPECT_EQ(g.const_tris()[1][2], 5u);
}

TEST(ModelNodes, ExtentsAreOfThePlacedMeshes) {
  model m;
  m.meshes.push_back({make_box_corner(0, 0, 0), -1, "corner"}); // 0..1 on each axis
  m.nodes.push_back(make_node("root", model::identity_transform(), {}, {1, 2}));
  m.nodes.push_back(make_node("moved", translation(5, 0, 0), {0}));
  m.nodes.push_back(make_node("doubled", scaling(2, 2, 2), {0}));
  const cvc::bounding_box bb = m.extents();
  EXPECT_DOUBLE_EQ(bb.XMin(), 0.0);
  EXPECT_DOUBLE_EQ(bb.XMax(), 6.0);
  EXPECT_DOUBLE_EQ(bb.YMax(), 2.0);
  EXPECT_DOUBLE_EQ(bb.ZMax(), 2.0);
}

// A mirror (negative determinant) would turn the copy inside out: its triangle is rewound so the
// face still points along its normal, and its tangent's handedness flips.
TEST(ModelNodes, AMirrorRewindsFacesAndFlipsHandedness) {
  model m;
  m.meshes.push_back({make_lit_triangle(), -1, "tri"});
  m.nodes.push_back(make_node("mirror", scaling(-1, 1, 1), {0}));
  const geometry g = m.merged();
  ASSERT_EQ(g.num_points(), 3u);
  const auto &p = g.const_points();
  EXPECT_TRUE(near_point(p[1], -1, 0, 0));
  const cvc::tri_t t = g.const_tris()[0];
  EXPECT_EQ(t[0], 0u);
  EXPECT_EQ(t[1], 2u);
  EXPECT_EQ(t[2], 1u);
  // The face normal of the triangle as wound, against the vertex normal.
  const double e1[3] = {p[t[1]][0] - p[t[0]][0], p[t[1]][1] - p[t[0]][1], p[t[1]][2] - p[t[0]][2]};
  const double e2[3] = {p[t[2]][0] - p[t[0]][0], p[t[2]][1] - p[t[0]][1], p[t[2]][2] - p[t[0]][2]};
  const double face_z = e1[0] * e2[1] - e1[1] * e2[0];
  EXPECT_GT(face_z, 0.0) << "the rewound face points +z";
  EXPECT_NEAR(g.const_normals()[0][2], 1.0, 1e-9) << "and so does its normal";
  EXPECT_NEAR(g.const_tangents()[0][0], -1.0, 1e-9);
  EXPECT_NEAR(g.const_tangents()[0][3], -1.0, 1e-9);
}

TEST(ModelNodes, OnlyTheMeshesANodeDrawsAreDrawn) {
  model m;
  m.meshes.push_back({make_triangle(), -1, "undrawn"});
  m.meshes.push_back({make_box_corner(2, 2, 2), -1, "drawn"});
  m.nodes.push_back(make_node("root", model::identity_transform(), {1}));
  const geometry g = m.merged();
  EXPECT_EQ(g.num_points(), 4u);
  EXPECT_TRUE(has_point(g, 2, 2, 2));
  EXPECT_FALSE(has_point(g, 1, 0, 0));
}

TEST(ModelNodes, ANodeTwoParentsListIsDrawnUnderEach) {
  model m;
  m.meshes.push_back({make_triangle(), -1, "tri"});
  m.nodes.push_back(make_node("root", model::identity_transform(), {}, {1, 2}));
  m.nodes.push_back(make_node("a", translation(1, 0, 0), {}, {3}));
  m.nodes.push_back(make_node("b", translation(0, 1, 0), {}, {3}));
  m.nodes.push_back(make_node("shared", model::identity_transform(), {0}));
  const std::vector<model::instance> in = m.instances();
  ASSERT_EQ(in.size(), 2u);
  EXPECT_DOUBLE_EQ(in[0].transform[3], 1.0);
  EXPECT_DOUBLE_EQ(in[1].transform[7], 1.0);
}

TEST(ModelNodes, CyclesAndIndicesOutOfRangeAreSkipped) {
  model m;
  m.meshes.push_back({make_triangle(), -1, "tri"});
  m.nodes.push_back(make_node("root", model::identity_transform(), {0, 5, -2}, {1, 7, -1}));
  m.nodes.push_back(make_node("loop", translation(1, 0, 0), {0}, {0, 1}));
  const std::vector<model::instance> in = m.instances();
  ASSERT_EQ(in.size(), 2u);
  EXPECT_EQ(in[0].node, 0);
  EXPECT_EQ(in[1].node, 1);
  EXPECT_EQ(m.merged().num_points(), 6u);
}

// A file's tree can be deep: the walk keeps its own stack, so 100,000 levels do not overflow the
// thread's.
TEST(ModelNodes, ADeepTreeIsWalkedWithoutDeepRecursion) {
  model m;
  m.meshes.push_back({make_triangle(), -1, "tri"});
  const int depth = 100000;
  for (int i = 0; i < depth; ++i)
    m.nodes.push_back(make_node("n", translation(0, 0, 1),
                                i == depth - 1 ? std::vector<int>{0} : std::vector<int>{},
                                i == depth - 1 ? std::vector<int>{} : std::vector<int>{i + 1}));
  const std::vector<model::instance> in = m.instances();
  ASSERT_EQ(in.size(), 1u);
  EXPECT_DOUBLE_EQ(in[0].transform[11], static_cast<double>(depth));
}

TEST(ModelNodes, PlacingACopyLeavesTheMeshAlone) {
  const geometry g = make_lit_triangle();
  const geometry moved = model::placed(g, translation(3, 0, 0));
  EXPECT_TRUE(near_point(moved.const_points()[0], 3, 0, 0));
  EXPECT_TRUE(near_point(g.const_points()[0], 0, 0, 0)) << "the source is shared, not changed";
}

// ── Assimp-backed loader tests ───────────────────────────────────────────────

#ifdef CVC_ENABLE_ASSIMP

namespace {

// Write a small OBJ + MTL + PNG texture into TempDir and return the .obj path.
// Returns "" (with a skip flag set) if the PNG could not be written (no image
// delegate) — the caller GTEST_SKIP()s in that case.
std::string write_obj_fixture(cvc::app &app, bool &texture_ok) {
  // A directory per test case: ctest runs the cases as parallel processes, and two cases writing
  // the same fixture files raced (one read the PNG while the other was rewriting it, "unknown
  // image type"). The file names inside stay the same, so the .mtl/.obj references do too.
  const ::testing::TestInfo *test = ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string dir =
      ::testing::TempDir() + "cvc_model_test_" + (test ? test->name() : "fixture") + "/";
  std::filesystem::create_directories(dir);
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
    tex.save(app, png);
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
  cvc::app app;
  std::vector<std::string> exts = app.model_handlers().extensions();
  bool has_obj = false;
  for (std::size_t i = 0; i < exts.size(); ++i)
    if (exts[i] == ".obj")
      has_obj = true;
  EXPECT_TRUE(has_obj);
}

TEST(ModelTest, ObjWithUvsMaterialTexture) {
  cvc::app app;
  bool texture_ok = false;
  std::string obj = write_obj_fixture(app, texture_ok);
  if (!texture_ok)
    GTEST_SKIP() << "No image delegate available to write the OBJ texture PNG";

  model m = cvc::read_model(app, obj);
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
  // Decoded pixels round-trip (PNG is lossless; the texture is a uniform color so
  // this is flip-invariant) — guards the channel order / decode path, not just dims.
  const unsigned char *tp = mat.base_color_texture.data();
  ASSERT_TRUE(tp != NULL);
  EXPECT_EQ(static_cast<int>(tp[0]), 200);
  EXPECT_EQ(static_cast<int>(tp[1]), 100);
  EXPECT_EQ(static_cast<int>(tp[2]), 50);
  EXPECT_EQ(static_cast<int>(tp[3]), 255);

  // The mesh has UVs, so aiProcess_CalcTangentSpace produced tangents: full-length
  // and unit-handedness (w == +/-1), which is how build_geometry reconstructs w.
  const geometry::tangents_t &tan = mesh.geom.const_tangents();
  ASSERT_EQ(tan.size(), mesh.geom.num_points());
  for (std::size_t i = 0; i < tan.size(); ++i)
    EXPECT_NEAR(std::abs(tan[i][3]), 1.0, 1e-6);
}

TEST(ModelTest, ReadGeometryObjFlatten) {
  cvc::app app;
  bool texture_ok = false;
  std::string obj = write_obj_fixture(app, texture_ok);
  // The flatten path does not require the texture; only the mesh matters.
  geometry g = cvc::read_geometry(app, obj);
  EXPECT_EQ(g.num_points(), 3u);
  EXPECT_EQ(g.num_tris(), 1u);
}

TEST(ModelTest, StlNoUvs) {
  cvc::app app;
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

  model m = cvc::read_model(app, stl);
  ASSERT_GE(m.num_meshes(), 1u);
  const model::mesh &mesh = m.meshes[0];
  EXPECT_GE(mesh.geom.num_points(), 3u);
  EXPECT_EQ(mesh.geom.num_tris(), 1u);
  EXPECT_TRUE(mesh.geom.const_uvs().empty());
}

// ── files with a node tree ───────────────────────────────────────────────────

namespace {

std::string base64(const std::string &in) {
  static const char *k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  const auto byte = [&](std::size_t i) {
    return static_cast<unsigned int>(static_cast<unsigned char>(in[i]));
  };
  std::string out;
  std::size_t i = 0;
  for (; i + 2 < in.size(); i += 3) {
    const unsigned int v = (byte(i) << 16) | (byte(i + 1) << 8) | byte(i + 2);
    out += k[(v >> 18) & 63];
    out += k[(v >> 12) & 63];
    out += k[(v >> 6) & 63];
    out += k[v & 63];
  }
  if (i + 1 == in.size()) {
    const unsigned int v = byte(i) << 16;
    out += k[(v >> 18) & 63];
    out += k[(v >> 12) & 63];
    out += "==";
  } else if (i + 2 == in.size()) {
    const unsigned int v = (byte(i) << 16) | (byte(i + 1) << 8);
    out += k[(v >> 18) & 63];
    out += k[(v >> 12) & 63];
    out += k[(v >> 6) & 63];
    out += '=';
  }
  return out;
}

// A glTF 2.0 file with one triangle mesh, (0,0,0) (1,0,0) (0,1,0), drawn by three nodes under a
// root moved 10 along x: `left` moved 5 along y; `right` doubled and turned a quarter about z (a
// TRS node); `child`, below `right`, moved 3 along z (a matrix node, column-major as glTF writes
// it). Its buffer is a data: URI, so the file stands alone.
std::string three_node_gltf() {
  std::string bin;
  const float pos[9] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
  const unsigned short idx[4] = {0, 1, 2, 0}; // the last is padding to 4 bytes
  bin.append(reinterpret_cast<const char *>(pos), sizeof pos);
  bin.append(reinterpret_cast<const char *>(idx), sizeof idx);
  return std::string(R"({"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}],
 "nodes": [
  {"name": "root", "translation": [10, 0, 0], "children": [1, 2]},
  {"name": "left", "translation": [0, 5, 0], "mesh": 0},
  {"name": "right", "rotation": [0, 0, 0.70710678118654752, 0.70710678118654752],
   "scale": [2, 2, 2], "mesh": 0, "children": [3]},
  {"name": "child", "matrix": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 3, 1], "mesh": 0}],
 "meshes": [{"name": "tri", "primitives": [{"attributes": {"POSITION": 0}, "indices": 1}]}],
 "accessors": [
  {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3",
   "min": [0, 0, 0], "max": [1, 1, 0]},
  {"bufferView": 1, "componentType": 5123, "count": 3, "type": "SCALAR"}],
 "bufferViews": [{"buffer": 0, "byteOffset": 0, "byteLength": 36},
                 {"buffer": 0, "byteOffset": 36, "byteLength": 6}],
 "buffers": [{"byteLength": 44, "uri": "data:application/octet-stream;base64,)") +
         base64(bin) + "\"}]}";
}

// The nine points three_node_gltf() draws, placed by its nodes.
void expect_three_placed_triangles(const geometry &g) {
  EXPECT_EQ(g.num_points(), 9u);
  EXPECT_EQ(g.num_tris(), 3u);
  const double want[9][3] = {{10, 5, 0}, {11, 5, 0}, {10, 6, 0}, // left
                             {10, 0, 0}, {10, 2, 0}, {8, 0, 0},  // right
                             {10, 0, 6}, {10, 2, 6}, {8, 0, 6}}; // child: 3 up, doubled
  for (const auto &w : want)
    EXPECT_TRUE(has_point(g, w[0], w[1], w[2])) << w[0] << " " << w[1] << " " << w[2];
}

} // namespace

TEST(ModelNodes, AGltfFilesNodesPlaceItsMeshes) {
  cvc::app app;
  const std::string text = three_node_gltf();
  cvc::model_read_options exact;
  exact.exact = true;
  const model m = cvc::read_model_from_memory(app, text.data(), text.size(), "gltf", exact);
  ASSERT_EQ(m.meshes.size(), 1u) << "one mesh, drawn three times";
  ASSERT_FALSE(m.nodes.empty());
  std::vector<std::string> names;
  for (const model::node &n : m.nodes)
    names.push_back(n.name);
  for (const char *name : {"left", "right", "child"})
    EXPECT_NE(std::find(names.begin(), names.end(), name), names.end()) << name;
  EXPECT_EQ(m.instances().size(), 3u);
  expect_three_placed_triangles(m.merged());

  // read_geometry (the flattening handler) draws the same, through the default import.
  const ::testing::TestInfo *test = ::testing::UnitTest::GetInstance()->current_test_info();
  const std::string dir = ::testing::TempDir() + "cvc_model_test_" + test->name() + "/";
  std::filesystem::create_directories(dir);
  const std::string path = dir + "three_nodes.gltf";
  std::ofstream(path, std::ios::binary) << text;
  expect_three_placed_triangles(cvc::read_geometry(app, path));
}

// A model's node tree, a mirror among its nodes, through a .glb and back.
TEST(ModelNodes, AGlbKeepsItsNodeTreeThroughAWriteAndARead) {
  cvc::app app;
  model m;
  m.meshes.push_back({make_triangle(), -1, "tri"});
  m.nodes.push_back(make_node("root", translation(10, 0, 0), {}, {1, 2}));
  m.nodes.push_back(make_node("left", translation(0, 5, 0), {0}));
  m.nodes.push_back(make_node("mirror", scaling(-1, 1, 1), {0}));
  if (!cvc::can_write_model_to_memory(".glb"))
    GTEST_SKIP() << "no glb writer";
  const std::string bytes = cvc::write_model_to_memory(app, m, ".glb");
  cvc::model_read_options exact;
  exact.exact = true;
  const model back = cvc::read_model_from_memory(app, bytes.data(), bytes.size(), "glb", exact);
  std::vector<std::string> names;
  for (const model::node &n : back.nodes)
    names.push_back(n.name);
  EXPECT_NE(std::find(names.begin(), names.end(), "mirror"), names.end());
  EXPECT_EQ(back.instances().size(), 2u);
  const geometry g = back.merged();
  ASSERT_EQ(g.num_points(), 6u);
  const double want[6][3] = {{10, 5, 0}, {11, 5, 0}, {10, 6, 0}, {10, 0, 0}, {9, 0, 0}, {10, 1, 0}};
  for (const auto &w : want)
    EXPECT_TRUE(has_point(g, w[0], w[1], w[2])) << w[0] << " " << w[1] << " " << w[2];
}

// A Collada file says it is Z-up in centimetres. Assimp would turn it to Y-up and scale it to
// metres through its root node; the model keeps the file's own axes and units, and only the
// file's node transform (a translation) places the mesh.
TEST(ModelNodes, AZUpColladaFileKeepsItsAxesAndUnits) {
  cvc::app app;
  const std::string dae = R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="centimeter" meter="0.01"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries>
    <geometry id="tri" name="tri"><mesh>
      <source id="tri-pos">
        <float_array id="tri-pos-array" count="9">0 0 0 1 0 0 0 1 0</float_array>
        <technique_common><accessor source="#tri-pos-array" count="3" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="tri-vtx"><input semantic="POSITION" source="#tri-pos"/></vertices>
      <triangles count="1"><input semantic="VERTEX" source="#tri-vtx" offset="0"/><p>0 1 2</p></triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes>
    <visual_scene id="scene" name="scene">
      <node id="placed" name="placed"><translate>1 2 3</translate><instance_geometry url="#tri"/></node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
)";
  cvc::model_read_options exact;
  exact.exact = true;
  const model m = cvc::read_model_from_memory(app, dae.data(), dae.size(), "dae", exact);
  const geometry g = m.merged();
  ASSERT_EQ(g.num_points(), 3u);
  EXPECT_TRUE(has_point(g, 1, 2, 3));
  EXPECT_TRUE(has_point(g, 2, 2, 3));
  EXPECT_TRUE(has_point(g, 1, 3, 3));
}

#endif // CVC_ENABLE_ASSIMP
