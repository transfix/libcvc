// A model's own clip in a scene (docs/ARIADNE.md §5.4, "Animations", clip: tracks): an animation's
// `{ node: bar, clip: bend }` draws the node's model (the glTF file its source: names) posed as far
// into the clip as the animation has run, its skin following its bones; played by realize_scene
// and tick_scene with no window, the scene clock stepped a fixed 0.05 s a frame.
//
// Pins:
//   - at realize the node draws the clip's first pose; the animation lasts as long as the clip;
//   - half way and at the end, the mesh's points are where the skinned model puts them, and the
//     animation stops at the clip's end;
//   - the node's fit: holds for the posed points;
//   - a clip the model does not have, and a node with no model file, are refused with a warning.
#include "streaming_test_util.h"

#include <cmath>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkDataSet.h>
#include <vtkMapper.h>

namespace {

namespace fs = std::filesystem;
namespace gla = cvc::gl::ariadne;
using cvc::gl::GraphicsNode;
using cvcgl_test::check;

std::string base64(const std::string &in) {
  static const char *k = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  std::size_t i = 0;
  const auto byte = [&](std::size_t j) {
    return static_cast<unsigned int>(static_cast<unsigned char>(in[j]));
  };
  for (; i + 2 < in.size(); i += 3) {
    const unsigned int v = (byte(i) << 16) | (byte(i + 1) << 8) | byte(i + 2);
    out += {k[(v >> 18) & 63], k[(v >> 12) & 63], k[(v >> 6) & 63], k[v & 63]};
  }
  if (i + 1 == in.size()) {
    const unsigned int v = byte(i) << 16;
    out += {k[(v >> 18) & 63], k[(v >> 12) & 63], '=', '='};
  } else if (i + 2 == in.size()) {
    const unsigned int v = (byte(i) << 16) | (byte(i + 1) << 8);
    out += {k[(v >> 18) & 63], k[(v >> 12) & 63], k[(v >> 6) & 63], '='};
  }
  return out;
}

// A bar of two triangles, (0,0,0) (1,0,0) at its foot on joint j0 and (0,2,0) (1,2,0) at its head
// on joint j1, j1 one up j0; its clip "bend" turns j1 a quarter about z over 1 s, so the head
// swings about (0,1,0). A glTF 2.0 file with its buffer in a data: URI.
std::string skin_gltf() {
  std::string bin, views, accessors;
  int n = 0;
  const auto add = [&](const void *p, std::size_t bytes, const std::string &meta) {
    while (bin.size() % 4)
      bin.push_back('\0');
    const std::size_t off = bin.size();
    bin.append(static_cast<const char *>(p), bytes);
    views += std::string(n ? "," : "") + "{\"buffer\": 0, \"byteOffset\": " + std::to_string(off) +
             ", \"byteLength\": " + std::to_string(bytes) + "}";
    accessors +=
        std::string(n ? "," : "") + "{\"bufferView\": " + std::to_string(n) + ", " + meta + "}";
    return std::to_string(n++);
  };
  const float h = 0.70710678f;
  const float pos[12] = {0, 0, 0, 1, 0, 0, 0, 2, 0, 1, 2, 0};
  const unsigned short idx[6] = {0, 1, 2, 1, 3, 2};
  const unsigned char joints[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  const float weights[16] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
  const float ibm[32] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,  0, 1,
                         1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1};
  const float t1[2] = {0, 1};
  const float turn[8] = {0, 0, 0, 1, 0, 0, h, h};
  const std::string p = add(
      pos, sizeof pos,
      R"("componentType": 5126, "count": 4, "type": "VEC3", "min": [0, 0, 0], "max": [1, 2, 0])");
  const std::string i =
      add(idx, sizeof idx, R"("componentType": 5123, "count": 6, "type": "SCALAR")");
  const std::string j =
      add(joints, sizeof joints, R"("componentType": 5121, "count": 4, "type": "VEC4")");
  const std::string w =
      add(weights, sizeof weights, R"("componentType": 5126, "count": 4, "type": "VEC4")");
  const std::string m =
      add(ibm, sizeof ibm, R"("componentType": 5126, "count": 2, "type": "MAT4")");
  const std::string t =
      add(t1, sizeof t1,
          R"("componentType": 5126, "count": 2, "type": "SCALAR", "min": [0], "max": [1])");
  const std::string r =
      add(turn, sizeof turn, R"("componentType": 5126, "count": 2, "type": "VEC4")");
  while (bin.size() % 4)
    bin.push_back('\0');
  return R"({"asset": {"version": "2.0"}, "scene": 0, "scenes": [{"nodes": [0]}],
 "nodes": [{"name": "root", "children": [1, 3]}, {"name": "j0", "children": [2]},
           {"name": "j1", "translation": [0, 1, 0]}, {"name": "bar", "mesh": 0, "skin": 0}],
 "skins": [{"joints": [1, 2], "inverseBindMatrices": )" +
         m + R"(, "skeleton": 1}],
 "meshes": [{"name": "bar", "primitives": [{"attributes": {"POSITION": )" +
         p + R"(, "JOINTS_0": )" + j + R"(, "WEIGHTS_0": )" + w + R"(}, "indices": )" + i + R"(}]}],
 "animations": [{"name": "bend", "channels": [{"sampler": 0, "target": {"node": 2, "path": "rotation"}}],
  "samplers": [{"input": )" +
         t + R"(, "output": )" + r + R"(}]}],
 "bufferViews": [)" +
         views + R"(], "accessors": [)" + accessors + R"(],
 "buffers": [{"byteLength": )" +
         std::to_string(bin.size()) + R"(, "uri": "data:application/octet-stream;base64,)" +
         base64(bin) + R"("}]})";
}

// The points a node draws (its VTK input: what a clip track updates in place).
bool draws_point(const std::shared_ptr<GraphicsNode> &n, double x, double y, double z) {
  auto *actor = n ? vtkActor::SafeDownCast(n->prop()) : nullptr;
  vtkDataSet *data =
      actor && actor->GetMapper() ? actor->GetMapper()->GetInputAsDataSet() : nullptr;
  if (!data)
    return false;
  for (vtkIdType i = 0; i < data->GetNumberOfPoints(); ++i) {
    double p[3];
    data->GetPoint(i, p);
    if (std::fabs(p[0] - x) < 1e-4 && std::fabs(p[1] - y) < 1e-4 && std::fabs(p[2] - z) < 1e-4)
      return true;
  }
  return false;
}

bool warned(const std::vector<std::string> &w, const std::string &needle) {
  for (const std::string &s : w)
    if (s.find(needle) != std::string::npos)
      return true;
  return false;
}

} // namespace

int main() {
  const fs::path dir = fs::temp_directory_path() / "cvcgl_ariadne_clip";
  fs::create_directories(dir);
  const fs::path file = dir / "bar.gltf";
  std::ofstream(file, std::ios::binary) << skin_gltf();
  const std::string doc = R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - { node: bar, source: { file: ")" +
                          file.generic_string() +
                          R"(" } }
    - { node: big, source: { file: ")" +
                          file.generic_string() +
                          R"(" }, fit: { height: 4 }, transform: { position: [10, 0, 0] } }
    - { node: crate, source: { box: 1 } }
  animations:
    - animation: bend
      tracks:
        - { node: bar, clip: bend }
        - { node: big, clip: "0" }
    - animation: wrong
      tracks:
        - { node: bar, clip: wave }
        - { node: crate, clip: bend }
)";

  cvc::app app;
  app.properties("system.log_verbosity", "0");
  const cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, doc);
  check(lr.ok, "the document loads", lr.error);
  for (const std::string &w : lr.warnings)
    check(false, "the document loads without warnings", w);
  if (!lr.ok)
    return cvcgl_test::finish("cvcgl_ariadne_clip");
  cvc::gl::SceneGraph sg(app, "clip");
  std::vector<std::string> warns;
  gla::RealizedScene rs = gla::realize_scene(sg, lr.scene, "clip", &warns);
  const auto tick = [&](int frames) {
    for (int f = 0; f < frames; ++f)
      gla::tick_scene(rs, nullptr, 0.05);
    sg.processEvents();
  };
  const auto text = [&](const std::string &k) { return cvc::state::instance(app)(k).value(); };
  const std::shared_ptr<GraphicsNode> bar = sg.getGraphics("bar"), big = sg.getGraphics("big");

  std::printf("Refused\n");
  check(warned(warns, "clip 'wave': node 'bar''s model has no such clip (it has bend)"),
        "a clip the model lacks, with the ones it has");
  check(warned(warns, "clip 'bend': scene node 'crate' draws no model file"),
        "a node with no model file");

  std::printf("At realize\n");
  check(bar && big, "the nodes exist");
  check(draws_point(bar, 0, 2, 0) && draws_point(bar, 1, 2, 0), "bar draws the clip's first pose");
  // big: the same bar fitted 4 tall: (x - 0.5, y - 1) * 2, so its head (0,2,0) is (-1, 2, 0)
  check(draws_point(big, -1, 2, 0), "big, fitted, draws it 4 tall");

  std::printf("Half way\n");
  tick(10);
  check(draws_point(bar, -0.70710678, 1.70710678, 0) && draws_point(bar, 0, 0, 0),
        "the head 45 degrees about (0,1,0), the foot where it was");
  check(text("clip.animations.bend.playing") == "1", "still playing");

  std::printf("At the end\n");
  tick(20);
  check(draws_point(bar, -1, 1, 0) && draws_point(bar, -1, 2, 0) && draws_point(bar, 1, 0, 0),
        "the head a quarter turned");
  check(draws_point(big, -3, 0, 0), "big's posed points fitted too: (-1, 1) to (-3, 0)");
  check(text("clip.animations.bend.playing") == "0" && text("clip.animations.bend.time") == "1",
        "the animation lasted as long as the clip, and stopped", text("clip.animations.bend.time"));

  std::printf("A seek back\n");
  cvc::state::instance(app)("clip.animations.bend.time").value(std::string("0"));
  tick(1);
  check(draws_point(bar, 0, 2, 0), "time 0: the first pose again");

  return cvcgl_test::finish("cvcgl_ariadne_clip");
}
