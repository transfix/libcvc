// Ariadne data in the scene (docs/roadmap/ARIADNE_DATA_ROADMAP.md): a `source: { data }` node draws
// what its dataset decoded, and draws it again when the dataset is replaced. Headless. These checks
// pin that:
//   1. at realize the node is an empty group: nothing waits for a load;
//   2. once the dataset is ready, sync_scene_data draws it: a city bundle's world as its items'
//      nodes, the terrain wearing the bundle's satellite.png (north at the image's top), kept as
//      the node's height field and, with body:, in the physics world;
//      a mesh dataset as one mesh node;
//   3. re-pointing the document's link at another dataset ((data-load) does) redraws the node from
//      it, and what it drew before is gone, its bodies too;
//   4. a dataset with nothing to draw warns, never throws;
//   5. material: { uv } orients a texture in texture space;
//   6. frame: asks the main view's camera to frame what the dataset drew, once (frame: true);
//   7. a hidden node draws nothing when its dataset arrives, and shows it with the node.
//
// NOT assert(): built Release, where NDEBUG makes assert() a no-op.
#include <boost/any.hpp>
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
#include <cvc/image/image.h>
#include <cvc/state/state.h>
#include <cvc/utility/archive.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <vtkNew.h>
#include <vtkRenderer.h>

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

// A 2 x 2 RGB PNG (red, green / blue, white), standing in for a bundle's satellite.png.
const unsigned char kSatellitePng[] = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44,
    0x52, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x08, 0x02, 0x00, 0x00, 0x00, 0xfd,
    0xd4, 0x9a, 0x73, 0x00, 0x00, 0x00, 0x12, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9c, 0x63, 0xf8,
    0xcf, 0xc0, 0xc0, 0x00, 0xc2, 0x0c, 0xff, 0x81, 0x00, 0x00, 0x1f, 0xee, 0x05, 0xfb, 0x0b,
    0xd9, 0x68, 0x8b, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82};

// A city bundle as a cvcpkg package holds it: a gzip tar, the bundle under share/cvc-scenes/, with
// an aerial image when `satellite`.
std::string package(int n, bool satellite) {
  std::vector<std::pair<std::string, std::string>> files = {
      {"share/libcvc-deps/scene-test/manifest.yaml", "m"},
      {"share/cvc-scenes/test/terrain.json", terrain_json(n)}};
  if (satellite)
    files.push_back(
        {"share/cvc-scenes/test/satellite.png",
         std::string(reinterpret_cast<const char *>(kSatellitePng), sizeof kSatellitePng)});
  return cvc::archive::gzip(cvc::archive::pack_tar(files));
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
  const std::string small = t.put("small.tar.zst", package(3, true));
  const std::string big = t.put("big.tgz", package(5, false));
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
      frame: true
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
  CHECK(out.world != nullptr);
  if (out.world) { // the scene's world takes the dataset's bodies at its next sync
    out.world->sync(&warnings);
    CHECK(out.world->bodies_of("city").size() == 5); // the terrain and its 4 edge walls, static
  }
  // frame: true — the main view's camera was asked to frame the city's bounds.
  const std::string frame_key = prefix + ".viewers.main.camera.frame";
  const std::string framed = cvc::state::instance(app)(frame_key).value();
  {
    std::istringstream in(framed);
    double b[6] = {0, 0, 0, 0, 0, 0};
    CHECK(static_cast<bool>(in >> b[0] >> b[1] >> b[2] >> b[3] >> b[4] >> b[5]));
    CHECK(b[0] == -10.0 && b[1] == -10.0 && b[3] == 10.0 && b[4] == 10.0);
    CHECK(b[2] == 0.0 && b[5] == 2.0); // the terrain's lowest and highest
  }
  // The aerial image, decoded with the bundle, drapes the terrain north up (UV (0, 0) is the
  // image's bottom-left, so the north-west corner is (0, 1)).
  const auto *sat = cvc::state::instance(app).findThroughLinks(city_path + ".satellite");
  CHECK(sat != nullptr);
  if (sat) {
    std::shared_ptr<const cvc::image> img;
    try {
      img =
          boost::any_cast<std::shared_ptr<const cvc::image>>(const_cast<cvc::state *>(sat)->data());
    } catch (const boost::bad_any_cast &) {
    }
    CHECK(img && img->width() == 2 && img->height() == 2);
  }
  if (auto terrain =
          std::dynamic_pointer_cast<cvc::gl::GeometryNode>(city->findChildByName("city_terrain"));
      terrain && terrain->getGeometry()) {
    CHECK(terrain->hasTexture()); // the 2 x 2 image, draped
    CHECK(terrain->textureWidth() == 2 && terrain->textureHeight() == 2);
    const cvc::geometry &g = *terrain->getGeometry();
    CHECK(g.const_uvs().size() == g.const_points().size());
    int corners = 0;
    for (std::size_t i = 0; i < g.const_points().size() && i < g.const_uvs().size(); ++i) {
      const auto &p = g.const_points()[i];
      const auto &uv = g.const_uvs()[i];
      if (p[0] == -10.0 && p[1] == 10.0)
        corners += (uv[0] == 0.0 && uv[1] == 1.0) ? 1 : 100; // north-west: the image's top-left
      if (p[0] == 10.0 && p[1] == -10.0)
        corners += (uv[0] == 1.0 && uv[1] == 0.0) ? 1 : 100; // south-east: its bottom-right
    }
    CHECK(corners == 2);
  } else {
    CHECK(false);
  }
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
  if (out.world) { // the old terrain's body went with it
    const std::size_t before = out.world->world().body_count();
    out.world->sync(&warnings);
    CHECK(out.world->bodies_of("city").size() == 5);
    CHECK(out.world->world().body_count() == before);
  }
  CHECK(cvc::state::instance(app)(frame_key).value() == framed); // frame: true frames only once
  if (auto terrain =
          std::dynamic_pointer_cast<cvc::gl::GeometryNode>(city->findChildByName("city_terrain")))
    CHECK(!terrain->hasTexture()); // the big city's package has no aerial image
}

// A hidden `source: { data }` node draws nothing when its dataset arrives. What it draws keeps its
// own flag (shown) but stays out of the renderer while the node is hidden; showing the node puts it
// in, hiding it takes it out again, and a dataset that replaces the old one while the node is
// hidden arrives hidden too.
void test_a_hidden_data_node_draws_nothing() {
  std::printf("test_a_hidden_data_node_draws_nothing\n");
  if (!cvc::ariadne::have_yaml()) {
    std::printf("  skipped: no YAML in this build\n");
    return;
  }
  TempDir t;
  t.put("small.tar.zst", package(3, true));
  const std::string big = t.put("big.tgz", package(5, false));
  t.put("tri.off", "OFF\n3 1 0\n0 0 0\n1 0 0\n0 1 0\n3 0 1 2\n");
  const std::string doc = R"(
meta: { min_libcvc: "0.0.0" }
data:
  city: { uri: small.tar.zst, path: share/cvc-scenes/test, as: city_bundle }
  tri: tri.off
scene:
  nodes:
    - node: city
      source: { data: city }
      visible: false
    - node: tri
      source: { data: tri }
      visible: false
)";
  cvc::ariadne::LoadResult r = cvc::ariadne::load_string(doc);
  CHECK(r.ok);
  if (!r.ok)
    return;
  cvc::app app;
  SceneGraph sg(app);
  vtkNew<vtkRenderer> ren; // no window: the prop bookkeeping is what is checked
  sg.setRenderer(ren);
  const std::string prefix = sg.getStatePrefix();
  auto store = cvc::ariadne::DataStore::of(app);
  for (cvc::ariadne::DataSpec d : r.data) {
    d.base = t.dir.generic_string();
    store->bind(prefix, d);
  }
  std::vector<std::string> warnings;
  ari::RealizedScene out = ari::realize_scene(sg, r.scene, prefix, &warnings);
  std::shared_ptr<cvc::gl::GraphicsNode> city = sg.getGraphics("city"), tri = sg.getGraphics("tri");
  CHECK(city && tri);
  if (!city || !tri)
    return;
  CHECK(!city->isVisible() && !tri->isVisible());
  CHECK(wait_ready(app, prefix + ".data.city"));
  CHECK(wait_ready(app, prefix + ".data.tri"));
  CHECK(ari::sync_scene_data(out, &warnings) == 2);
  auto drawn = [&](cvc::gl::GraphicsNode &node, const char *child) {
    std::shared_ptr<cvc::gl::GraphicsNode> c = node.findChildByName(child);
    return c && c->prop() && ren->HasViewProp(c->prop());
  };
  auto own_flag = [&](cvc::gl::GraphicsNode &node, const char *child) {
    std::shared_ptr<cvc::gl::GraphicsNode> c = node.findChildByName(child);
    return c && c->isVisible() && !c->isVisibleInHierarchy();
  };
  CHECK(!drawn(*city, "city_terrain") && !drawn(*tri, "tri_geometry")); // the nodes are hidden
  CHECK(own_flag(*city, "city_terrain") && own_flag(*tri, "tri_geometry"));
  city->setVisible(true);
  CHECK(drawn(*city, "city_terrain") && !drawn(*tri, "tri_geometry"));
  tri->setVisible(true);
  CHECK(drawn(*tri, "tri_geometry"));
  city->setVisible(false);
  tri->setVisible(false);
  CHECK(!drawn(*city, "city_terrain") && !drawn(*tri, "tri_geometry"));

  // A new dataset while the node is hidden: drawn into it, hidden; shown with the node.
  cvc::ariadne::DataSpec bigger;
  bigger.name = "city";
  bigger.uri = big;
  bigger.path = "share/cvc-scenes/test";
  bigger.as = "city_bundle";
  store->bind(prefix, bigger);
  CHECK(wait_ready(app, prefix + ".data.city"));
  CHECK(ari::sync_scene_data(out, &warnings) == 1);
  CHECK(!drawn(*city, "city_terrain"));
  city->setVisible(true);
  CHECK(drawn(*city, "city_terrain"));
  sg.setRenderer(nullptr);
}

// material: { uv: ... } turns a texture's UVs: flips, then quarter turns (the image
// counter-clockwise), then scale and offset.
void test_a_uv_block_orients_the_texture() {
  std::printf("test_a_uv_block_orients_the_texture\n");
  cvc::app app;
  SceneGraph sg(app);
  cvc::geometry quad;
  quad.points() = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  quad.tris() = {{0, 1, 2}};
  quad.uvs() = {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}};
  auto g = std::dynamic_pointer_cast<cvc::gl::GeometryNode>(sg.addGraphics("quad", quad));
  CHECK(g != nullptr);
  if (!g)
    return;
  cvc::ariadne::SceneNode n;
  ari::apply_uv_transform(*g, n); // no uv block: nothing changes
  CHECK(g->getGeometry()->const_uvs()[1][0] == 1.0);
  n.has_uv_transform = true;
  n.uv_rotate = 90; // the image's top-left now shows at the surface's bottom-left
  ari::apply_uv_transform(*g, n);
  const auto &a = g->getGeometry()->const_uvs();
  CHECK(a[0][0] == 0.0 && a[0][1] == 1.0);
  CHECK(a[1][0] == 0.0 && a[1][1] == 0.0);
  CHECK(a[2][0] == 1.0 && a[2][1] == 1.0);
  cvc::ariadne::SceneNode m;
  m.has_uv_transform = true;
  m.uv_flip_v = true;
  m.uv_scale[0] = 2.0f;
  m.uv_offset[1] = 0.5f;
  ari::apply_uv_transform(*g, m); // (0, 1) -> flip (0, 0) -> (0, 0.5)
  const auto &b = g->getGeometry()->const_uvs();
  CHECK(b[0][0] == 0.0 && b[0][1] == 0.5);
  CHECK(b[2][0] == 2.0 && b[2][1] == 0.5);
}

} // namespace

int main() {
  test_a_dataset_draws_into_its_node_and_redraws_when_replaced();
  test_a_hidden_data_node_draws_nothing();
  test_a_uv_block_orients_the_texture();
  if (g_failures) {
    std::printf("cvcgl_ariadne_data: %d FAILED\n", g_failures);
    return 1;
  }
  std::printf("cvcgl_ariadne_data: OK\n");
  return 0;
}
