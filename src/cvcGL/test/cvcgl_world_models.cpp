/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// cvcgl_world_models -- a generated world's theme models, drawn (cvc/gl/ariadne/world_models.h):
//
//   * the glTF frame (+Y up, front +Z) turns into the world's (+Z up, front +Y), points and
//     normals, and an instance's matrix scales, yaws and translates;
//   * an asset's LODs read from its glbs into one ladder, with the sidecar's errors, cached by file;
//     a missing coarse rung ends the ladder, a missing LOD 0 is an error;
//   * every placed copy is a LodGraphicsNode on that ladder whose rungs draw from the first copy's
//     arrays, its errors scaled with it, choosing its rung by its own distance.
//
// Headless: a bare scene graph, nothing rendered. NOT assert() (Release builds drop it).

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/LodGraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/world_models.h>
#include <cvc/lod/select.h>
#include <cvc/world/model_assets.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <vtkNew.h>
#include <vtkRenderer.h>

namespace fs = std::filesystem;
using cvc::gl::LodGraphicsNode;
using cvc::gl::SceneGraph;
namespace gla = cvc::gl::ariadne;

namespace {

int g_failures = 0;
void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

// A glb of `tris` triangles of a unit box (+Y up, base at y = 0), with uvs, no material.
void write_box_glb(const fs::path &p, int tris) {
  const float pos[8][3] = {{-0.5f, 0, -0.5f}, {0.5f, 0, -0.5f}, {-0.5f, 1, -0.5f}, {0.5f, 1, -0.5f},
                           {-0.5f, 0, 0.5f},  {0.5f, 0, 0.5f},  {-0.5f, 1, 0.5f},  {0.5f, 1, 0.5f}};
  float uv[8][2];
  for (int i = 0; i < 8; ++i) {
    uv[i][0] = pos[i][0] + 0.5f;
    uv[i][1] = pos[i][2] + 0.5f;
  }
  const std::uint32_t faces[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                                      {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  std::vector<unsigned char> bin(sizeof pos + sizeof uv + 12u * tris);
  std::memcpy(bin.data(), pos, sizeof pos);
  std::memcpy(bin.data() + sizeof pos, uv, sizeof uv);
  std::memcpy(bin.data() + sizeof pos + sizeof uv, faces, 12u * tris);
  const std::size_t ib = 12u * tris;
  std::string js =
      "{\"asset\":{\"version\":\"2.0\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],"
      "\"nodes\":[{\"mesh\":0}],\"meshes\":[{\"primitives\":[{\"attributes\":{\"POSITION\":0,"
      "\"TEXCOORD_0\":1},\"indices\":2}]}],\"accessors\":["
      "{\"bufferView\":0,\"componentType\":5126,\"count\":8,\"type\":\"VEC3\","
      "\"min\":[-0.5,0,-0.5],\"max\":[0.5,1,0.5]},"
      "{\"bufferView\":1,\"componentType\":5126,\"count\":8,\"type\":\"VEC2\"},"
      "{\"bufferView\":2,\"componentType\":5125,\"count\":" +
      std::to_string(3 * tris) +
      ",\"type\":\"SCALAR\"}],"
      "\"bufferViews\":[{\"buffer\":0,\"byteOffset\":0,\"byteLength\":96},"
      "{\"buffer\":0,\"byteOffset\":96,\"byteLength\":64},"
      "{\"buffer\":0,\"byteOffset\":160,\"byteLength\":" +
      std::to_string(ib) + "}],\"buffers\":[{\"byteLength\":" + std::to_string(bin.size()) + "}]}";
  while (js.size() % 4)
    js += ' ';
  while (bin.size() % 4)
    bin.push_back(0);
  std::ofstream out(p, std::ios::binary);
  auto u32 = [&out](std::uint32_t v) { out.write(reinterpret_cast<const char *>(&v), 4); };
  u32(0x46546C67);
  u32(2);
  u32(std::uint32_t(12 + 8 + js.size() + 8 + bin.size()));
  u32(std::uint32_t(js.size()));
  u32(0x4E4F534A);
  out.write(js.data(), std::streamsize(js.size()));
  u32(std::uint32_t(bin.size()));
  u32(0x004E4942);
  out.write(reinterpret_cast<const char *>(bin.data()), std::streamsize(bin.size()));
}

// An asset directory: Box.glb (12 triangles), Box.lod1.glb (2), Box.asset.json.
fs::path box_asset(const std::string &tag, bool with_lod1 = true) {
  const fs::path d = fs::temp_directory_path() / ("cvcgl_world_models_" + tag);
  std::error_code ec;
  fs::remove_all(d, ec);
  fs::create_directories(d);
  write_box_glb(d / "Box.glb", 12);
  if (with_lod1)
    write_box_glb(d / "Box.lod1.glb", 2);
  std::ofstream(d / "Box.asset.json")
      << "{\"schema\": \"cvc-model-asset/1\", \"name\": \"Box\", \"kind\": \"building\","
         " \"bounds\": {\"min\": [-0.5, 0, -0.5], \"max\": [0.5, 1, 0.5]}, \"lods\": ["
         "{\"file\": \"Box.glb\", \"triangles\": 12, \"world_error_m\": 0},"
         "{\"file\": \"Box.lod1.glb\", \"triangles\": 2, \"world_error_m\": 0.5}]}";
  return d;
}

cvc::lod::view_params view_at(double x, double y, double z) {
  cvc::lod::view_params v = cvc::lod::preset_view(cvc::lod::quality_preset::balanced);
  v.eye[0] = x;
  v.eye[1] = y;
  v.eye[2] = z;
  return v;
}

void test_frames(cvc::app &app) {
  std::printf("frames\n");
  cvc::geometry g(app);
  g.points().push_back({1.0, 2.0, 3.0});
  g.normals().push_back({0.0, 0.0, 1.0}); // glTF's front
  const cvc::geometry w = gla::model_frame_to_world(g);
  CHECK(w.const_points()[0][0] == -1.0 && w.const_points()[0][1] == 3.0 &&
        w.const_points()[0][2] == 2.0);
  CHECK(w.const_normals()[0][1] == 1.0); // the world's front, +Y
  cvc::world::generated_world::model_instance m;
  m.x = 10.0;
  m.y = -4.0;
  m.z = 2.0;
  m.yaw = 0.5 * 3.14159265358979323846;
  m.scale = 2.0;
  double M[16];
  gla::model_instance_matrix(m, M);
  // (1, 0, 1) scaled by 2, turned a quarter about +Z, moved: (10, -2, 4).
  const double p[3] = {1.0, 0.0, 1.0};
  double q[3];
  for (int r = 0; r < 3; ++r)
    q[r] = M[4 * r] * p[0] + M[4 * r + 1] * p[1] + M[4 * r + 2] * p[2] + M[4 * r + 3];
  CHECK(std::fabs(q[0] - 10.0) < 1e-9 && std::fabs(q[1] + 2.0) < 1e-9 &&
        std::fabs(q[2] - 4.0) < 1e-9);
}

void test_loading(cvc::app &app) {
  std::printf("loading\n");
  const fs::path d = box_asset("load");
  const cvc::world::model_asset a = cvc::world::read_model_asset((d / "Box.asset.json").string());
  std::string why;
  std::shared_ptr<gla::model_mesh> mesh = gla::load_model_mesh(app, a, &why);
  if (!mesh && why.find("handler") != std::string::npos) {
    std::printf("  SKIP: no glb reader in this build (%s)\n", why.c_str());
    return;
  }
  CHECK(mesh != nullptr);
  if (!mesh)
    return;
  CHECK(mesh->name == "Box");
  CHECK(mesh->pyr.rungs.size() == 2);
  CHECK(mesh->pyr.world_error_m.size() == 2);
  CHECK(mesh->pyr.world_error_m[0] == 0.0 && mesh->pyr.world_error_m[1] == 0.5);
  CHECK(mesh->donors.size() == 2 && mesh->textures.size() == 2);
  if (mesh->pyr.rungs.size() == 2) {
    CHECK(mesh->pyr.rungs[0].num_tris() == 12);
    CHECK(mesh->pyr.rungs[1].num_tris() == 2);
    // In the world frame: glTF's height (+Y, 0..1) is the world's z.
    const cvc::bounding_box bb = mesh->pyr.rungs[0].extents();
    CHECK(std::fabs(bb.minz) < 1e-6 && std::fabs(bb.maxz - 1.0) < 1e-6);
    CHECK(mesh->pyr.rungs[0].const_normals().size() == mesh->pyr.rungs[0].const_points().size());
  }
  CHECK(gla::load_model_mesh(app, a) == mesh); // cached by file

  // A missing coarse rung ends the ladder; a missing LOD 0 is an error.
  const fs::path d1 = box_asset("nolod1", false);
  const cvc::world::model_asset b = cvc::world::read_model_asset((d1 / "Box.asset.json").string());
  std::shared_ptr<gla::model_mesh> one = gla::load_model_mesh(app, b);
  CHECK(one && one->pyr.rungs.size() == 1);
  cvc::world::model_asset gone = b;
  gone.name = "Gone";
  gone.lods[0].file = "Missing.glb";
  why.clear();
  CHECK(gla::load_model_mesh(app, gone, &why) == nullptr);
  CHECK(!why.empty());
  std::error_code ec;
  fs::remove_all(d, ec);
  fs::remove_all(d1, ec);
}

void test_instances(cvc::app &app) {
  std::printf("instances\n");
  const fs::path d = box_asset("inst");
  const cvc::world::model_asset a = cvc::world::read_model_asset((d / "Box.asset.json").string());
  std::shared_ptr<gla::model_mesh> mesh = gla::load_model_mesh(app, a);
  if (!mesh) {
    std::printf("  SKIP: the asset did not read\n");
    return;
  }
  vtkNew<vtkRenderer> ren;
  SceneGraph sg(app, "models");
  sg.setRenderer(ren);
  auto group = sg.addGraphics("group");
  cvc::world::generated_world::model_instance m1, m2;
  m1.x = 100.0;
  m2.x = -5000.0; // far enough that its coarser rung is under the pixel budget from m1
  m2.scale = 3.0;
  std::shared_ptr<LodGraphicsNode> n1 = gla::make_model_instance(*group, "m1", *mesh, m1, true);
  std::shared_ptr<LodGraphicsNode> n2 = gla::make_model_instance(*group, "m2", *mesh, m2, true);
  CHECK(n1 && n2);
  if (!n1 || !n2)
    return;
  CHECK(!n1->isVisible() && !n2->isVisible()); // hidden until the world swaps in
  CHECK(n1->rungCount() == 2 && n2->rungCount() == 2);
  CHECK(n1->rungError(1) == 0.5);
  CHECK(std::fabs(n2->rungError(1) - 1.5) < 1e-12); // three times the size, three times the error
  // The second copy draws from the first's arrays, rung by rung.
  CHECK(n2->rung(0)->sharesArrays() && n2->rung(1)->sharesArrays());
  CHECK(mesh->donors[0].lock() == n1->rung(0));
  // Posed: its world bounds are around its place.
  const cvc::bounding_box bb = n1->getWorldBoundingBox();
  CHECK(bb.minx > 98.0 && bb.maxx < 102.0);
  // Each chooses by its own distance: near m1, m1 draws LOD 0 and m2 (5 km off) its coarser rung.
  n1->setVisible(true);
  n2->setVisible(true);
  const cvc::lod::view_params near1 = view_at(100.0, -2.0, 1.0);
  CHECK(n1->select(near1) == 0);
  CHECK(n2->select(near1) == 1);
  CHECK(n1->select(view_at(100.0, -20000.0, 1.0)) == 1); // far: the coarser rung
  std::error_code ec;
  fs::remove_all(d, ec);
}

} // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  cvc::app app;
  test_frames(app);
  test_loading(app);
  test_instances(app);
  std::printf("%s: cvcgl_world_models (%d check%s failed)\n", g_failures == 0 ? "PASS" : "FAIL",
              g_failures, g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
