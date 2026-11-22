// cvcgl_volume_render — the `volume_render` scene node (VolumeRenderNode.h,
// volume_render_nodes.h) and the volume_grid's 3D volume (grid.display.render), headless, driven
// through their state keys:
//   - a document's volume_render draws the volume the app holds at its source's key (state, data,
//     or another node's), the very pointer, published as <node>.volume, and follows a new one;
//   - <node>.renderer switches between VTK's mapper, volren and volslice (and none) while it runs,
//     each renderer a child named for its kind; one transfer-function model at
//     <node>.transfer_function is bound to whichever draws, and an edit carries over a switch;
//   - a name that is no renderer draws nothing and says so; with none, the model keeps its range;
//   - volume_grid's display.render draws the grid's own volume in the 3D view, inside the surfaces
//     group the slice views hide, through the grid's transfer function;
//   - T6: a second app without cvcGL's extensions has no volume_render type.
// Then, offscreen (skipped where nothing rasterises unless CVC_REQUIRE_RENDER=1): each renderer
// draws the volume.

#include "streaming_test_util.h"

#include <algorithm>
#include <boost/any.hpp>
#include <chrono>
#include <cvc/ariadne/bind.h>
#include <cvc/ariadne/data.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolRenNode.h>
#include <cvc/gl/VolSliceNode.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeNode.h>
#include <cvc/gl/VolumeRenderNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/gl/ariadne/tf_binding.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using cvc::gl::VolumeRenderNode;
using cvcgl_test::check;

namespace {

// Voxel (i, j, k) of an nx x ny x nz volume one unit apart holds i + 10 j + 100 k.
cvc::volume coded(cvc::app &app, int nx, int ny, int nz) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), cvc::Float,
                cvc::bounding_box(0.0, 0.0, 0.0, nx - 1.0, ny - 1.0, nz - 1.0));
  for (int k = 0; k < nz; ++k)
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i)
        v(i, j, k, i + 10.0 * j + 100.0 * k);
  return v;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

void set(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

std::shared_ptr<const cvc::volume> volume_at(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&d);
  return p ? *p : nullptr;
}

struct realized_doc {
  cvc::gl::SceneGraph sg;
  cvc::gl::ariadne::RealizedScene rs;
  std::vector<std::string> warnings;
  realized_doc(cvc::app &app, const std::string &name, const std::string &text) : sg(app, name) {
    sg.setDiagnosticChromeVisible(false);
    cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, text);
    for (const std::string &w : lr.warnings)
      warnings.push_back(w);
    rs = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &warnings);
  }
  void tick(int n = 1) {
    for (int i = 0; i < n; ++i)
      cvc::gl::ariadne::tick_scene(rs, nullptr, 1.0 / 30.0);
  }
};

const char *kDoc = R"(meta:
  name: volume render
  min_libcvc: 4.0.0
scene:
  nodes:
    - node: v
      type: volume_render
      source: { state: /vols.v }
      renderer: volren
      transfer_function: { color: [[0, 0, 0, 1], [1, 1, 1, 1]] }
    - node: g
      type: volume_grid
      source: { state: /vols.v }
    - node: w
      type: volume_render
      source: { node: g }
      renderer: vtk
)";

void run() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  cvc::state &root = cvc::state::instance(app);
  const auto vol = std::make_shared<const cvc::volume>(coded(app, 6, 5, 4));
  root("vols.v").data(boost::any(vol));
  realized_doc d(app, "vr", kDoc);
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  d.tick();

  auto v = std::dynamic_pointer_cast<VolumeRenderNode>(d.sg.getGraphics("v"));
  if (!check(v != nullptr, "type: volume_render realizes a VolumeRenderNode"))
    return;
  const std::string k = v->stateName();
  const std::string model = v->stateName("transfer_function");
  check(volume_at(app, k + ".volume") == vol && v->volume() == vol,
        "it draws the volume the app holds at its source's key: the very pointer, published");
  check(std::dynamic_pointer_cast<cvc::gl::VolRenNode>(v->renderer()) != nullptr &&
            v->rendererKind() == VolumeRenderNode::kind::volren &&
            v->renderer()->getName() == "volren" && val(app, k + ".status") == "drawn by volren",
        "renderer: volren draws it with a VolRenNode named volren", val(app, k + ".status"));
  check(!val(app, model).empty() && cvc::gl::ariadne::find_transfer_function(app, model),
        "one transfer-function model at <node>.transfer_function, bound");
  check(!val(app, v->renderer()->stateName("volren.volumes.0.transfer_function.color")).empty(),
        "and lowered into volren's tables");

  // ---- switching ----
  const std::string edited = cvc::tf::to_string(cvc::tf::model::preset("grayscale"));
  set(app, model, edited); // an edit before the switch
  d.tick();
  set(app, k + ".renderer", "vtk");
  d.tick();
  auto vn = std::dynamic_pointer_cast<cvc::gl::VolumeNode>(v->renderer());
  check(vn && vn->getVolume() && vn->getVolume()->XDim() == 6 && v->renderer()->getName() == "vtk",
        "renderer vtk: VTK's mapper draws it, a child named vtk", val(app, k + ".status"));
  check(val(app, model) == edited && cvc::gl::ariadne::find_transfer_function(app, model) &&
            !val(app, v->renderer()->stateName("transfer_function.color")).empty(),
        "the edited model carries over the switch, bound to the new renderer and lowered");
  set(app, k + ".renderer", "volslice");
  d.tick();
  check(std::dynamic_pointer_cast<cvc::gl::VolSliceNode>(v->renderer()) != nullptr &&
            !val(app, v->renderer()->stateName("volslice.transfer_function.color")).empty() &&
            v->getGraphicsChildren().size() == 1,
        "renderer volslice: texture slices draw it, through the model; one renderer at a time");
  set(app, k + ".renderer", "none");
  d.tick();
  check(!v->renderer() && v->getGraphicsChildren().empty() &&
            val(app, k + ".status") == "none: nothing drawn",
        "renderer none: nothing drawn", val(app, k + ".status"));
  check(cvc::gl::ariadne::find_transfer_function(app, model) && !val(app, model + ".range").empty(),
        "and the model keeps its binding and its range", val(app, model + ".range"));
  set(app, k + ".renderer", "raytracer");
  d.tick();
  check(!v->renderer() && val(app, k + ".status").find("is not vtk, volren, volslice or none") !=
                              std::string::npos,
        "a name that is no renderer draws nothing and says so", val(app, k + ".status"));

  // ---- the volume changes ----
  set(app, k + ".renderer", "volren");
  d.tick();
  const std::shared_ptr<cvc::gl::GraphicsNode> was = v->renderer();
  const auto bigger = std::make_shared<const cvc::volume>(coded(app, 8, 7, 3));
  root("vols.v").data(boost::any(bigger)); // a filter's output, say
  d.tick();
  auto vr = std::dynamic_pointer_cast<cvc::gl::VolRenNode>(v->renderer());
  check(v->volume() == bigger && volume_at(app, k + ".volume") == bigger && vr && vr == was &&
            vr->volumeCount() == 1,
        "a new volume at the key is followed: the renderer drawing keeps drawing, the new one");

  // ---- source: { node: g }: the grid's volume ----
  auto w = std::dynamic_pointer_cast<VolumeRenderNode>(d.sg.getGraphics("w"));
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  if (!check(w && g, "the grid and the node following it realize"))
    return;
  d.tick();
  check(w->volume() == bigger && volume_at(app, g->stateName("grid.volume")) == bigger &&
            std::dynamic_pointer_cast<cvc::gl::VolumeNode>(w->renderer()) != nullptr,
        "source: { node: g } follows the grid's grid.volume: one volume, three views",
        val(app, w->stateName("status")));

  // ---- volume_grid's 3D volume ----
  const std::shared_ptr<VolumeRenderNode> gv = g->volumeRender();
  check(gv && !gv->renderer(), "a grid draws no volume in the 3D view until asked");
  const auto &inside = g->surfaces()->getGraphicsChildren();
  check(std::find(inside.begin(), inside.end(), gv) != inside.end(),
        "its volume is in the surfaces group, which the slice views hide");
  set(app, g->stateName("grid.display.render"), "volren");
  d.tick();
  check(gv && std::dynamic_pointer_cast<cvc::gl::VolRenNode>(gv->renderer()) != nullptr &&
            gv->volume() == bigger,
        "grid.display.render volren draws the grid's own volume",
        val(app, gv->stateName("status")));
  const std::string gmodel = g->stateName("transfer_function");
  check(
      cvc::gl::ariadne::find_transfer_function(app, gmodel) &&
          !val(app, gv->renderer()->stateName("volren.volumes.0.transfer_function.color")).empty(),
      "through the grid's transfer function, the one its planes colour through");
  set(app, g->stateName("grid.display.render"), "none");
  d.tick();
  check(!gv->renderer() && cvc::gl::ariadne::find_transfer_function(app, gmodel),
        "and none takes it away, the planes' binding back");

  // ---- two apps ----
  cvc::app other;
  check(!cvc::gl::ariadne::has_scene_node_type(other, "volume_render") &&
            cvc::gl::ariadne::has_scene_node_type(app, "volume_render"),
        "an app that has not registered cvcGL's extensions has no volume_render type");
}

// Offscreen: each renderer draws the volume (some pixels lit over a black background).
void rendered() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!cvcgl_test::renderAvailable(app))
    return;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  cvc::state::instance(app)("vols.v").data(
      boost::any(std::make_shared<const cvc::volume>(coded(app, 6, 5, 4))));
  realized_doc d(app, "vr_draw", R"(meta:
  name: volume render draw
  min_libcvc: 4.0.0
scene:
  nodes:
    - node: v
      type: volume_render
      source: { state: /vols.v }
      renderer: vtk
      transfer_function: { color: [[0, 1, 1, 1], [1, 1, 1, 1]] }
)");
  for (const std::string &w : d.warnings)
    check(false, "the drawn document realizes without warnings", w);
  auto v = std::dynamic_pointer_cast<VolumeRenderNode>(d.sg.getGraphics("v"));
  if (!check(v != nullptr, "the drawn node exists"))
    return;
  cvc::gl::SceneRenderer sr(d.sg, 200, 200, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  cvcgl_test::topView(sr, 2.5, 2.0, 4.5);
  for (const char *r : {"vtk", "volren", "volslice"}) {
    set(app, v->stateName("renderer"), r);
    long lit = 0;
    // volren's raycast lands on its own thread: a few frames until it does.
    for (int i = 0; i < 200 && lit < 200; ++i) {
      d.tick();
      lit = cvcgl_test::litPixels(cvcgl_test::grab(sr));
      if (lit < 200)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    check(lit >= 200, std::string("renderer ") + r + " draws the volume",
          std::to_string(lit) + " lit px");
  }
}

} // namespace

int main() {
  run();
  rendered();
  return cvcgl_test::finish("cvcgl_volume_render");
}
