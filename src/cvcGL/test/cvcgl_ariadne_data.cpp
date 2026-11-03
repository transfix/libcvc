// Ariadne data in the scene (docs/roadmap/ARIADNE_DATA_ROADMAP.md): a `source: { data }` node draws
// what its dataset decoded, and draws it again when the dataset is replaced. Headless. These checks
// pin that:
//   1. at realize the node is an empty group: nothing waits for a load;
//   2. once the dataset is ready, sync_scene_data draws it: a city bundle's world as its items'
//      nodes, the terrain kept as the node's height field and, with body:, in the physics world;
//      a mesh dataset as one mesh node;
//   3. re-pointing the document's link at another dataset ((data-load) does) redraws the node from
//      it, and what it drew before is gone, its bodies too;
//   4. a dataset with nothing to draw warns, never throws.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <chrono>
#include <cstdio>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/geometry/height_field.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/utility/archive.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

using cvc::gl::SceneGraph;
namespace ari = cvc::gl::ariadne;
namespace fs = std::filesystem;

static int g_failures = 0;
static void check_impl(bool ok, const char *expr, int line) {
  if (!ok) {
    std::printf("  FAIL line %d: %s\n", line, expr);
    ++g_failures;
  }
}
#define CHECK(cond) check_impl(static_cast<bool>(cond), #cond, __LINE__)

namespace {

std::string terrain_json(int n) { // an n x n grid over [-10, 10]^2, rising to the north
  std::string grid;
  for (int r = 0; r < n; ++r) {
    grid += r ? ", [" : "[";
    for (int c = 0; c < n; ++c)
      grid += (c ? ", " : "") + std::to_string(n - 1 - r);
    grid += "]";
  }
  return R"({"bounds": {"min_x": -10, "min_y": -10, "max_x": 10, "max_y": 10}, "rows": )" +
         std::to_string(n) + R"(, "cols": )" + std::to_string(n) + R"(, "grid": [)" + grid + "]}";
}

struct TempDir {
  fs::path dir;
  TempDir() {
    std::random_device rd;
    dir = fs::temp_directory_path() / ("cvcgl-data-" + std::to_string(rd()) + std::to_string(rd()));
    fs::create_directories(dir);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
  std::string put(const std::string &name, const std::string &bytes) const {
    std::ofstream(dir / name, std::ios::binary)
        .write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    return (dir / name).generic_string();
  }
};

// A city bundle as a cvcpkg package holds it: a gzip tar, the bundle under share/cvc-scenes/.
std::string package(int n) {
  return cvc::archive::gzip(
      cvc::archive::pack_tar({{"share/libcvc-deps/scene-test/manifest.yaml", "m"},
                              {"share/cvc-scenes/test/terrain.json", terrain_json(n)}}));
}

bool wait_ready(cvc::app &app, const std::string &path) {
  return cvc::ariadne::DataStore::of(app)->wait(path, 30.0);
}

std::vector<std::string> child_names(const cvc::gl::GraphicsNode &g) {
  std::vector<std::string> out;
  for (const auto &c : g.getGraphicsChildren())
    out.push_back(c->getName());
  return out;
}

void test_a_dataset_draws_into_its_node_and_redraws_when_replaced() {
  std::printf("test_a_dataset_draws_into_its_node_and_redraws_when_replaced\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  TempDir t;
  const std::string small = t.put("small.tar.zst", package(3));
  const std::string big = t.put("big.tgz", package(5));
  t.put("tri.off", "OFF\n3 1 0\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n");
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
data:
  city: { uri: small.tar.zst, path: share/cvc-scenes/test, as: city_bundle }
  tri: tri.off
  notes: { uri: small.tar.zst }
scene:
  physics: {}
  nodes:
    - node: city
      source: { data: city }
      body: static
    - node: tri
      source: { data: tri }
    - node: nothing
      source: { data: notes }
)";
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(doc);
  CHECK(r.ok);
  if (!r.ok) {
    std::printf("  %s\n", r.error.c_str());
    return;
  }
  CHECK(r.data.size() == 3);
  cvc::app app;
  SceneGraph sg(app);
  const std::string prefix = sg.getStatePrefix();
  auto store = cvc::ariadne::DataStore::of(app);
  for (cvc::ariadne::DataSpec d : r.data) {
    d.base = t.dir.generic_string(); // the document's directory
    store->bind(prefix, d);
  }
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, prefix, &warnings);
  CHECK(out.data_nodes.size() == 3);
  std::shared_ptr<cvc::gl::GraphicsNode> city = sg.getGraphics("city");
  CHECK(city != nullptr);
  if (!city || out.data_nodes.size() != 3)
    return;

  const std::string city_path = prefix + ".data.city";
  CHECK(wait_ready(app, city_path));
  CHECK(wait_ready(app, prefix + ".data.tri"));
  CHECK(wait_ready(app, prefix + ".data.notes"));
  CHECK(ari::sync_scene_data(out, &warnings) == 3);
  const std::vector<std::string> drawn = child_names(*city);
  CHECK(drawn.size() == 1 && drawn[0] == "city_terrain");
  CHECK(out.height_fields.count("city") == 1);
  if (out.height_fields.count("city"))
    CHECK(out.height_fields.at("city")->rows() == 3);
  CHECK(out.physics != nullptr);
  CHECK(out.data_nodes[0].bodies.size() == 1); // the terrain, static
  if (auto tri = sg.getGraphics("tri")) {
    const std::vector<std::string> mesh = child_names(*tri);
    CHECK(mesh.size() == 1 && mesh[0] == "tri_geometry");
  } else {
    CHECK(false);
  }
  bool warned = false;
  for (const std::string &w : warnings)
    warned = warned || w.find("nothing to draw") != std::string::npos;
  CHECK(warned); // notes is files only

  // Nothing changed: nothing is redrawn.
  CHECK(ari::sync_scene_data(out, &warnings) == 0);

  // Replace the small city with the big one, as (data-load "city" ...) does.
  cvc::ariadne::DataSpec bigger;
  bigger.name = "city";
  bigger.uri = big;
  bigger.path = "share/cvc-scenes/test";
  bigger.as = "city_bundle";
  store->bind(prefix, bigger);
  CHECK(wait_ready(app, city_path));
  CHECK(ari::sync_scene_data(out, &warnings) == 1);
  CHECK(child_names(*city).size() == 1);
  if (out.height_fields.count("city"))
    CHECK(out.height_fields.at("city")->rows() == 5);
  CHECK(out.data_nodes[0].bodies.size() == 1); // the old terrain's body went with it
}

} // namespace

int main() {
  test_a_dataset_draws_into_its_node_and_redraws_when_replaced();
  if (g_failures) {
    std::printf("cvcgl_ariadne_data: %d FAILED\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_ariadne_data: OK\n");
  return 0;
}
