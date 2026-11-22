// cvcgl_shared_volume — scene nodes that view a volume the app holds on the state tree
// (docs/roadmap/DATA_FROM_STATE_TREE.md, D1; node_data.h), headless:
//   - one data: entry shown by a volren, a volume_grid's planes and a volume_slice: the volren's
//     <node>.volume, the grid's grid.volume and the plane's volume are the dataset's very
//     pointer, the voxels never copied;
//   - a new volume written at the dataset's key (as a filter's result is stored) is followed by all
//     of them, and so is the dataset loaded again; the volren keeps one volume and its settings;
//   - the volren's transfer function is seeded once its first volume arrives, its isosurface placed
//     on that volume's range (not seeded before);
//   - source: { node } follows another node's published volume, volume_of: follows its plane, and
//     { state } follows a program's key, pointed elsewhere by writing source.volume; a key that
//     holds no volume is said in source.status;
//   - two apps: each app's nodes hold that app's volume, and a new volume in one leaves the other's
//     nodes as they were.

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/volume/volume_slice.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

// Voxel (i, j, k) of an nx x ny x nz volume one unit apart holds i + 10 j + 100 k, plus `add`.
cvc::volume coded(cvc::app &app, int nx, int ny, int nz, double add = 0.0) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), cvc::Float,
                cvc::bounding_box(0.0, 0.0, 0.0, nx - 1.0, ny - 1.0, nz - 1.0));
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i)
        v(i, j, k, add + i + 10.0 * j + 100.0 * k);
  return v;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

std::shared_ptr<const cvc::volume> volume_at(cvc::app &app, const std::string &path) {
  cvc::state *s = cvc::state::instance(app).findThroughLinks(path);
  if (!s)
    return nullptr;
  const boost::any d = s->data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&d))
    return *p;
  return nullptr;
}

std::string doc(const std::string &file) {
  return "meta:\n  name: shared volume\n  min_libcvc: 4.0.0\n"
         "data:\n  head: " +
         file +
         "\n"
         "scene:\n  nodes:\n"
         "    - node: render\n      type: volren\n      source: { data: head }\n"
         "      volren:\n"
         "        ambient: 0.4\n"
         "        isosurfaces:\n"
         "          - { value: 150, color: [1, 1, 1], opacity: 1 }\n"
         "    - node: grid\n      type: volume_grid\n      source: { data: head }\n"
         "    - node: cut\n      type: volume_slice\n      source: { node: render }\n"
         "      axis: z\n"
         "    - node: cut_x\n      type: volume_slice\n      volume_of: cut\n      axis: x\n"
         "    - node: plane\n      type: volume_slice\n      source: { data: head }\n"
         "    - node: work\n      type: volren\n      source: { state: /work.volume }\n"
         "      volren:\n"
         "        ambient: 0.4\n"
         "        isosurfaces:\n"
         "          - { value: 1, color: [1, 1, 1], opacity: 1 }\n";
}

struct shown {
  cvc::app &app;
  cvc::gl::SceneGraph sg;
  cvc::gl::ariadne::RealizedScene rs;
  std::vector<std::string> warnings;
  std::string prefix, head;
  std::vector<cvc::ariadne::DataSpec> data;
  shown(cvc::app &a, const std::string &name, const std::string &text) : app(a), sg(a, name) {
    sg.setDiagnosticChromeVisible(false);
    prefix = sg.getStatePrefix();
    cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, text);
    for (const std::string &w : lr.warnings)
      warnings.push_back(w);
    data = lr.data;
    head = cvc::ariadne::resolve_bind(prefix, "data.head");
    rs = cvc::gl::ariadne::realize_scene(sg, lr.scene, prefix, &warnings);
  }
  // The app loads the document's data (a host binds its data: block so): here after the scene is
  // realized, so the nodes are seen waiting for it.
  void load() {
    for (const cvc::ariadne::DataSpec &d : data)
      cvc::ariadne::DataStore::of(app)->bind(prefix, d);
  }
  void tick(int n = 1) {
    for (int i = 0; i < n; ++i)
      cvc::gl::ariadne::tick_scene(rs, nullptr, 1.0 / 30.0);
  }
  std::string node(const std::string &id) {
    const std::shared_ptr<cvc::gl::GraphicsNode> g = sg.getGraphics(id);
    return g ? g->stateName() : std::string();
  }
  std::shared_ptr<const cvc::volume> dataset() { return volume_at(app, head + ".volume"); }
  std::shared_ptr<const cvc::volume> of(const std::string &id, const char *key = "volume") {
    return volume_at(app, node(id) + "." + key);
  }
  // The store node's volume key, where a program stores a filter's result (state-data-set).
  std::string dataset_key() {
    cvc::state *s = cvc::state::instance(app).findThroughLinks(head);
    return s ? s->fullName() + ".volume" : std::string();
  }
};

void run(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string file = (dir / "head.rawiv").generic_string();
  cvc::writeVolumeFile(app, coded(app, 8, 6, 4), file);

  shown d(app, "shared_a", doc(file));
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto render = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(d.sg.getGraphics("render"));
  auto grid = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("grid"));
  auto cut = std::dynamic_pointer_cast<cvc::gl::VolumeSliceNode>(d.sg.getGraphics("cut"));
  auto cut_x = std::dynamic_pointer_cast<cvc::gl::VolumeSliceNode>(d.sg.getGraphics("cut_x"));
  auto plane = std::dynamic_pointer_cast<cvc::gl::VolumeSliceNode>(d.sg.getGraphics("plane"));
  auto work = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(d.sg.getGraphics("work"));
  check(render && grid && cut && cut_x && plane && work,
        "a volren, a volume_grid and volume_slices with source: { data | node | state } realize "
        "as their own nodes (not as generic data groups)");
  if (!render || !grid || !cut || !cut_x || !plane || !work)
    return;

  // Before the dataset is in: nothing shown, nothing seeded.
  const std::string tf = d.node("render") + ".transfer_function";
  d.tick();
  check(render->volumeCount() == 0 && val(app, tf).empty() && !d.of("grid", "grid.volume"),
        "until the app loads the dataset the volren draws nothing and its transfer function waits",
        val(app, tf));
  check(val(app, d.node("render") + ".source.status") == "waiting for a volume at data.head.volume",
        "and says what it waits for", val(app, d.node("render") + ".source.status"));
  d.load();
  check(cvc::ariadne::DataStore::of(app)->wait(d.head, 30.0), "the app loads the volume",
        cvc::ariadne::data_status(app, d.head));
  d.tick(2);

  // ---- one volume, shared ----
  const std::shared_ptr<const cvc::volume> v1 = d.dataset();
  check(v1 && v1->XDim() == 8, "the dataset holds the volume");
  check(v1 && d.of("render") == v1, "the volren's volume is the dataset's very pointer");
  check(v1 && d.of("grid", "grid.volume") == v1,
        "the grid's grid.volume is the same pointer: the main scene and the grid show one volume");
  check(v1 && d.of("cut") == v1, "source: { node: render } cuts the volren's published volume");
  check(v1 && d.of("plane") == v1, "a volume_slice with source: { data } cuts it too");
  check(render->volumeCount() == 1, "the volren raycasts one volume");
  check(cut->source() && cut->source()->in_memory_volume() &&
            cut->source()->in_memory_volume()->XDim() == 8 && cut_x->source() == cut->source(),
        "volume_of: cut shares cut's slices");
  check(val(app, d.node("render") + ".source.status").empty(), "a shown volume clears the status",
        val(app, d.node("render") + ".source.status"));

  // The transfer function, seeded on the first volume's range [0, 357]: 150 sits at 150/357.
  cvc::tf::model m;
  const bool parsed = cvc::tf::from_string(val(app, tf), m);
  check(parsed && m.isocontours.size() == 1 &&
            std::fabs(m.isocontours[0].at - 150.0 / 357.0) < 1e-3,
        "the volren's transfer function is seeded once its volume arrives, the isosurface at its "
        "value on that volume's range",
        val(app, tf));

  // ---- a filter's result put as the dataset's volume: everything follows ----
  // As a program's (data-put "head" "volume" result) puts it: in the store node, its rev up.
  const std::shared_ptr<cvc::ariadne::DataStore> store = cvc::ariadne::DataStore::of(app);
  const std::string rev = val(app, d.dataset_key().substr(0, d.dataset_key().rfind('.')) + ".rev");
  const auto v2 = std::make_shared<const cvc::volume>(coded(app, 8, 6, 4, 1000.0));
  store->set_output(d.head, "volume", boost::any(v2));
  check(d.dataset() == v2 &&
            val(app, d.dataset_key().substr(0, d.dataset_key().rfind('.')) + ".rev") !=
                rev,
        "set_output puts the volume in the dataset's store node, its rev up");
  d.tick(2);
  check(d.of("render") == v2 && d.of("grid", "grid.volume") == v2,
        "a new volume at the key: the volren and the grid both follow it, still one pointer");
  check(d.of("cut") == v2 && d.of("plane") == v2 && cut_x->source() == cut->source() &&
            cut->source()->in_memory_volume() &&
            cut->source()->in_memory_volume()->min() == 1000.0,
        "and so do the planes, volume_of: included");
  check(render->volumeCount() == 1, "the volren replaced its volume, not added one");
  check(grid->histogramVolume() == v2, "the grid's transfer function follows it too");
  // Written again within one clock tick: still a change (the pointee tells it).
  const auto v3 = std::make_shared<const cvc::volume>(coded(app, 8, 6, 4, 2000.0));
  const auto v4 = std::make_shared<const cvc::volume>(coded(app, 8, 6, 4, 3000.0));
  cvc::state::instance(app)(d.dataset_key()).data(boost::any(v3));
  cvc::state::instance(app)(d.dataset_key()).data(boost::any(v4));
  d.tick();
  check(d.of("render") == v4 && d.of("grid", "grid.volume") == v4,
        "two writes in one tick: the last is followed");

  // ---- the dataset loaded again: a new volume, followed ----
  cvc::ariadne::DataStore::of(app)->reload(d.head);
  check(cvc::ariadne::DataStore::of(app)->wait(d.head, 30.0), "the app loads the volume again");
  d.tick(2);
  const std::shared_ptr<const cvc::volume> v5 = d.dataset();
  check(v5 && v5 != v4 && d.of("render") == v5 && d.of("grid", "grid.volume") == v5 &&
            d.of("cut") == v5,
        "the dataset reloaded: every view follows the new volume");

  // ---- { state }: a program's key, pointed elsewhere ----
  const std::string w = d.node("work");
  check(work->volumeCount() == 0 &&
            val(app, w + ".source.status") == "waiting for a volume at /work.volume",
        "source: { state } waits for the key", val(app, w + ".source.status"));
  const auto mine = std::make_shared<const cvc::volume>(coded(app, 3, 3, 3));
  cvc::state::instance(app)("work.volume").data(boost::any(mine));
  d.tick();
  check(work->volumeCount() == 1 && d.of("work") == mine, "and shows the volume written there");
  // A program's state-data-set stores the DSL's data object (volume.filter's result): its payload
  // is the volume followed, the very pointer.
  {
    const auto filtered = std::make_shared<const cvc::volume>(coded(app, 3, 3, 3, 4000.0));
    auto obj = std::make_shared<cvc::state_exec::data_object>();
    obj->payload = filtered;
    obj->type_name = "cvc::volume";
    cvc::state::instance(app)("work.volume").data(boost::any(cvc::state_exec::value_t(obj)));
    d.tick();
    check(d.of("work") == filtered && work->volumeCount() == 1,
          "a data object a program stored at the key is followed: its volume, the very pointer");
  }
  cvc::state::instance(app)(w + ".source.volume").value(d.dataset_key().insert(0, "/"));
  d.tick();
  check(d.of("work") == v5, "source.volume pointed at the dataset: it follows that instead");
  cvc::state::instance(app)("work.text").value(std::string("not a volume"));
  cvc::state::instance(app)("work.text").data(boost::any(std::string("not a volume")));
  cvc::state::instance(app)(w + ".source.volume").value(std::string("/work.text"));
  d.tick();
  check(val(app, w + ".source.status") == "error: /work.text holds no volume" &&
            d.of("work") == v5,
        "a key that holds no volume is said, and the volume shown is kept",
        val(app, w + ".source.status"));

  // ---- two apps ----
  cvc::app other;
  other.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(other);
  shown e(other, "shared_b", doc(file));
  e.load();
  check(cvc::ariadne::DataStore::of(other)->wait(e.head, 30.0), "the other app loads its volume");
  e.tick(2);
  const std::shared_ptr<const cvc::volume> b1 = e.dataset();
  check(b1 && b1 != d.dataset() && e.of("render") == b1 && e.of("grid", "grid.volume") == b1,
        "the other app's nodes show the other app's volume");
  const auto v6 = std::make_shared<const cvc::volume>(coded(app, 8, 6, 4, 5000.0));
  cvc::state::instance(app)(d.dataset_key()).data(boost::any(v6));
  d.tick();
  e.tick();
  check(d.of("render") == v6 && e.of("render") == b1 && e.of("grid", "grid.volume") == b1,
        "a new volume in one app leaves the other app's nodes as they were");
}

} // namespace

int main() {
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvcgl_shared_volume_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  run(dir);
  std::error_code ec;
  fs::remove_all(dir, ec);
  return cvcgl_test::finish("cvcgl_shared_volume");
}
