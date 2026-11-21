// cvcgl_volume_grid — the `volume_grid` scene node (VolumeGridNode.h, volume_grid_nodes.h;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P8), headless, driven through its state keys
// as a document, the DSL or pycvc drives it (T13's scripted picks):
//   - a document's volume_grid realizes three volume_slice planes, tagged for the views, its props
//     seeding <node>.grid.* (slices resolved and given their "max", classes as rows) and its
//     transfer function, which all three planes colour through (or the grey model);
//   - picks written to grid.pick.* run the gestures: a click marks and unmarks a point, a drag is
//     a rubber band, Shift+Ctrl drags the selection, navigate moves the other planes; the overlays
//     draw the points, the rings, the crosshairs and the band; the pointer fills the probe;
//   - requests: undo, delete_selected, select_all, step, save / load classes and points, save /
//     load a session, load another volume, and a bad one reported in status;
//   - rows edited, added and removed edit the classes; a point_classes written to grid.points from
//     outside replaces them;
//   - T6: two grids in one document share nothing (one read plane by plane); a second app without
//     cvcGL's extensions has no volume_grid type.

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;
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

// A pick as a slice view writes it (seq last), then a tick to consume it.
struct picker {
  cvc::app &app;
  realized_doc &d;
  std::string key;
  int seq = 0;
  void operator()(const char *phase, int i, int j, int k, const char *axis = "z",
                  const char *mods = "") {
    set(app, key + ".phase", phase);
    set(app, key + ".i", std::to_string(i));
    set(app, key + ".j", std::to_string(j));
    set(app, key + ".k", std::to_string(k));
    set(app, key + ".x", std::to_string(i));
    set(app, key + ".y", std::to_string(j));
    set(app, key + ".z", std::to_string(k));
    set(app, key + ".value", std::to_string(i + 10 * j + 100 * k));
    set(app, key + ".axis", axis);
    set(app, key + ".mods", mods);
    set(app, key + ".seq", std::to_string(++seq));
    d.tick();
  }
  void click(int i, int j, int k, const char *axis = "z", const char *mods = "") {
    (*this)("down", i, j, k, axis, mods);
    (*this)("up", i, j, k, axis, mods);
  }
};

std::shared_ptr<const cvc::point_classes> points_of(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const cvc::point_classes>>(&d))
    return *p;
  return nullptr;
}

std::size_t lines_in(const std::shared_ptr<cvc::gl::GeometryNode> &g) {
  return g && g->isVisible() && g->getGeometry() ? g->getGeometry()->num_lines() : 0;
}

std::size_t tris_in(const std::shared_ptr<cvc::gl::GeometryNode> &g) {
  return g && g->isVisible() && g->getGeometry() ? g->getGeometry()->num_tris() : 0;
}

std::string doc(const std::string &file, const std::string &file2) {
  return "meta:\n  name: grid test\n  min_libcvc: 3.7.0\nscene:\n  nodes:\n"
         "    - node: g\n      type: volume_grid\n      source: { file: \"" +
         file +
         "\" }\n"
         "      view: xy\n      slice: { z: 2 }\n"
         "      display: { point_scale: 1 }\n"
         "      classes:\n"
         "        - { name: membrane, color: [1, 0, 0], radius: 1.5 }\n"
         "        - { name: background, color: [0, 0, 1] }\n"
         "      transfer_function: { preset: grayscale }\n"
         "    - node: h\n      type: volume_grid\n      source: { file: \"" +
         file2 + "\" }\n      region_reads: true\n";
}

void run(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  check(cvc::gl::ariadne::has_scene_node_type(app, "volume_grid"),
        "register_cvcgl_extensions installs volume_grid");
  const std::string file = (dir / "coded.rawiv").generic_string();
  const std::string file2 = (dir / "other.rawiv").generic_string();
  const std::string file3 = (dir / "cube.rawiv").generic_string();
  cvc::writeVolumeFile(app, coded(app, 8, 6, 4), file);
  cvc::writeVolumeFile(app, coded(app, 4, 4, 4), file2);
  cvc::writeVolumeFile(app, coded(app, 5, 5, 5), file3);

  realized_doc d(app, "vg", doc(file, file2));
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  auto h = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("h"));
  check(g && h, "a volume_grid node is a VolumeGridNode");
  if (!g || !h)
    return;
  d.tick(2);
  const std::string k = g->stateName() + ".grid";

  // ---- realize: the planes, their tags, the keys ----
  using cvc::slice_axis;
  check(g->slice(slice_axis::z) && g->slice(slice_axis::y) && g->slice(slice_axis::x),
        "three planes");
  bool tagged = false, surfaces = false;
  for (const auto &t : d.rs.nodes) {
    if (t.id == "g_slice_xy")
      tagged = t.tags == std::vector<std::string>{"g", "slice_xy"} &&
               t.node.lock() == g->slice(slice_axis::z);
    if (t.id == "g_surfaces")
      surfaces = t.tags == std::vector<std::string>{"g", "surfaces"};
  }
  check(tagged, "the xy plane is realized as g_slice_xy, tagged g and slice_xy");
  check(surfaces, "the surfaces group is g_surfaces, tagged g and surfaces");
  check(val(app, k + ".view") == "xy" && val(app, k + ".tool") == "points", "view and tool seeded");
  check(val(app, k + ".slice.z") == "2" && val(app, k + ".slice.x") == "3" &&
            val(app, k + ".slice.y") == "2",
        "slice.z from the prop, x and y the middle planes",
        val(app, k + ".slice.x") + " " + val(app, k + ".slice.y"));
  check(cvc::state::instance(app)(k + ".slice.x").property("max") == "7",
        "slice.x's max is its last plane");
  check(val(app, g->slice(slice_axis::z)->stateName() + ".slice.index") == "2" &&
            val(app, g->slice(slice_axis::x)->stateName() + ".slice.index") == "3",
        "the planes follow the grid's slice keys");
  check(val(app, k + ".info.dims") == "8 6 4" && val(app, k + ".info.max") == "357",
        "info published", val(app, k + ".info.dims"));
  check(val(app, k + ".classes") == "2" && val(app, k + ".classes.0.name") == "membrane" &&
            val(app, k + ".classes.0.color") == "1,0,0" &&
            val(app, k + ".classes.0.radius") == "1.5",
        "the classes prop seeds the rows");
  check(val(app, k + ".classes.current") == "0", "the first class is current");
  auto pts = points_of(app, k + ".points");
  check(pts && pts->classes.size() == 2 && pts->classes[1].name == "background",
        "grid.points holds the classes");
  check(g->slice(slice_axis::z)->modelKey() == g->stateName("transfer_function"),
        "the planes colour through the grid's transfer function");
  check(!val(app, g->stateName("transfer_function")).empty(), "which is bound and seeded");
  check(lines_in(g->linesOverlay(slice_axis::z)) == 2, "crosshairs on the xy plane",
        std::to_string(lines_in(g->linesOverlay(slice_axis::z))));
  check(g->source()->in_memory() && !h->source()->in_memory(),
        "g is read whole, h (region_reads: true) plane by plane");
  check(points_of(app, k + ".volume") == nullptr &&
            !cvc::state::instance(app)(k + ".volume").data().empty(),
        "grid.volume holds the volume in memory");

  // ---- picks ----
  picker pick{app, d, k + ".pick"};
  pick.click(3, 4, 2);
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[0].points == std::vector<int>{3, 4, 2}, "a click marks a point");
  check(val(app, k + ".status") == "added (3, 4, 2) to membrane", "and says so",
        val(app, k + ".status"));
  check(val(app, k + ".classes.0.points") == "1", "the row counts it");
  check(tris_in(g->pointsOverlay(slice_axis::z)) == 16, "a disk on the xy plane",
        std::to_string(tris_in(g->pointsOverlay(slice_axis::z))));
  check(tris_in(g->pointsOverlay(slice_axis::y)) == 0, "none on the xz plane (y 2, not 4)");
  check(val(app, k + ".cursor.i") == "3" && val(app, k + ".cursor.value") == "243" &&
            val(app, k + ".cursor.inside") == "1",
        "the pick is the probe");
  pick.click(4, 4, 2); // radius 1.5: the point one voxel off
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[0].points.empty(), "a click on a point unmarks it");
  pick.click(3, 4, 2);
  pick.click(6, 1, 2);
  set(app, k + ".classes.current", "1");
  d.tick();
  pick.click(1, 1, 2);
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[0].size() == 2 && pts->classes[1].points == std::vector<int>{1, 1, 2},
        "the current class takes the clicks");

  // A rubber band over the left half: (3, 4) and (1, 1) selected, the band drawn while dragged.
  pick("down", 0, 0, 2);
  pick("drag", 4, 5, 2);
  check(lines_in(g->linesOverlay(slice_axis::z)) == 6, "the band draws while dragged",
        std::to_string(lines_in(g->linesOverlay(slice_axis::z))));
  pick("up", 4, 5, 2);
  check(val(app, k + ".selection.count") == "2", "a band selects", val(app, k + ".status"));
  check(lines_in(g->linesOverlay(slice_axis::z)) == 2 + 2 * 16, "rings round the selection");
  auto sel = points_of(app, k + ".selection");
  check(sel && sel->point_count() == 2, "grid.selection holds them");
  // Shift+Ctrl drags them two voxels right.
  pick("down", 3, 4, 2, "z", "shift,ctrl");
  pick("drag", 4, 4, 2, "z", "shift,ctrl");
  pick("drag", 5, 4, 2, "z", "shift,ctrl");
  pick("up", 5, 4, 2, "z", "shift,ctrl");
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[0].contains(5, 4, 2) && pts->classes[1].contains(3, 1, 2) &&
            pts->classes[0].contains(6, 1, 2),
        "Shift+Ctrl drags the selection");
  set(app, k + ".request", "undo");
  d.tick();
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[0].contains(3, 4, 2) && val(app, k + ".request").empty(),
        "undo puts the drag back as one step; the request is consumed");
  set(app, k + ".request", "select_all");
  d.tick();
  check(val(app, k + ".selection.count") == "3", "select_all");
  set(app, k + ".request", "delete_selected");
  d.tick();
  pts = points_of(app, k + ".points");
  check(pts && pts->point_count() == 0 && val(app, k + ".status") == "deleted 3 points",
        "delete_selected", val(app, k + ".status"));
  set(app, k + ".request", "undo");
  d.tick();

  // Navigate: a click in the xy view moves the x and y planes there.
  set(app, k + ".tool", "navigate");
  d.tick();
  pick.click(6, 1, 2);
  check(val(app, k + ".slice.x") == "6" && val(app, k + ".slice.y") == "1" &&
            val(app, g->slice(slice_axis::x)->stateName() + ".slice.index") == "6",
        "navigate moves the other planes to the voxel");
  pts = points_of(app, k + ".points");
  check(pts && pts->point_count() == 3, "and marks nothing");
  set(app, k + ".tool", "points");
  set(app, k + ".request", "step 1");
  d.tick();
  check(val(app, k + ".slice.z") == "3", "step 1 moves the shown view's plane");
  set(app, k + ".request", "step 10");
  d.tick();
  check(val(app, k + ".slice.z") == "3", "clamped at the last plane");
  set(app, k + ".slice.z", "2");
  d.tick();

  // The pointer: the probe follows it.
  set(app, k + ".pointer.i", "7");
  set(app, k + ".pointer.j", "5");
  set(app, k + ".pointer.k", "2");
  set(app, k + ".pointer.value", "257");
  set(app, k + ".pointer.seq", "1");
  d.tick();
  check(val(app, k + ".cursor.i") == "7" && val(app, k + ".cursor.value") == "257",
        "the pointer fills the probe");

  // ---- rows ----
  set(app, k + ".classes.1.name", "bg");
  d.tick();
  pts = points_of(app, k + ".points");
  check(pts && pts->classes[1].name == "bg" && pts->classes[1].size() == 1,
        "a renamed row renames its class and keeps its points");
  set(app, k + ".classes.2.name", "extra");
  set(app, k + ".classes", "3");
  d.tick();
  pts = points_of(app, k + ".points");
  std::string palette;
  for (double c : cvc::point_class_palette(2))
    palette += (palette.empty() ? "" : ",") + cvc::param_detail::format_double(c);
  check(pts && pts->classes.size() == 3 && pts->classes[2].name == "extra" &&
            val(app, k + ".classes.2.color") == palette,
        "an added row adds a class, its colour from the palette", val(app, k + ".classes.2.color"));

  // ---- the data channel ----
  {
    cvc::point_classes seeds;
    seeds.add("seeds", 1, 2, 3);
    seeds.add("seeds", 2, 2, 3);
    cvc::state::instance(app)(k + ".points")
        .data(boost::any(std::make_shared<const cvc::point_classes>(seeds)));
    d.tick();
    check(val(app, k + ".classes") == "1" && val(app, k + ".classes.0.name") == "seeds" &&
              val(app, k + ".classes.0.points") == "2",
          "a point_classes written to grid.points replaces the classes and rows");
    set(app, k + ".request", "undo");
    d.tick();
    check(val(app, k + ".classes") == "3", "undoable too");
  }

  // ---- files ----
  const std::string vgr = (dir / "classes.vgr").generic_string();
  set(app, k + ".request", "save_classes " + vgr);
  d.tick();
  check(fs::exists(vgr), "save_classes writes the file", val(app, k + ".status"));
  set(app, k + ".request", "delete_selected");
  set(app, k + ".request", "select_all");
  d.tick();
  set(app, k + ".request", "delete_selected");
  d.tick();
  set(app, k + ".request", "load_classes " + vgr);
  d.tick();
  pts = points_of(app, k + ".points");
  check(pts && pts->point_count() == 3 && pts->classes.size() == 3, "load_classes reads it back",
        val(app, k + ".status"));
  set(app, k + ".request", "load_points " + vgr);
  d.tick();
  pts = points_of(app, k + ".points");
  check(pts && pts->point_count() == 3 && pts->classes.size() == 3,
        "load_points adds a file's points to the classes of those names (nothing new here)");
  set(app, k + ".request", "load_classes " + (dir / "missing.vgr").generic_string());
  d.tick();
  check(val(app, k + ".status").rfind("error: load_classes:", 0) == 0,
        "a file that can't be read is reported", val(app, k + ".status"));
  set(app, k + ".request", "frobnicate");
  d.tick();
  check(val(app, k + ".status") == "error: frobnicate: 'frobnicate' is not a request",
        "a bad request is reported", val(app, k + ".status"));

  // ---- sessions ----
  const std::string session = (dir / "grid.session.json").generic_string();
  set(app, k + ".view", "zy");
  set(app, k + ".display.point_scale", "2");
  d.tick();
  set(app, k + ".request", "save_session " + session);
  d.tick();
  check(fs::exists(session), "save_session writes the file", val(app, k + ".status"));
  set(app, k + ".view", "xy");
  set(app, k + ".display.point_scale", "1");
  set(app, k + ".request", "select_all");
  d.tick();
  set(app, k + ".request", "delete_selected");
  d.tick();
  set(app, k + ".request", "load_session " + session);
  d.tick();
  pts = points_of(app, k + ".points");
  check(val(app, k + ".view") == "zy" && val(app, k + ".display.point_scale") == "2" && pts &&
            pts->point_count() == 3,
        "load_session puts the keys and the classes back", val(app, k + ".status"));

  // ---- grey ----
  set(app, k + ".display.grey", "1");
  d.tick();
  check(g->slice(slice_axis::y)->modelKey() == g->stateName("grey_transfer_function"),
        "grey: the planes colour through the grey model");
  set(app, k + ".display.grey", "0");
  d.tick();
  check(g->slice(slice_axis::y)->modelKey() == g->stateName("transfer_function"), "and back");

  // ---- T6: the other grid ----
  const std::string hk = h->stateName() + ".grid";
  auto hpts = points_of(app, hk + ".points");
  check(hpts && hpts->empty() && val(app, hk + ".classes") == "0" &&
            val(app, hk + ".classes.current") == "-1",
        "the other grid has no classes: nothing above touched it");
  check(val(app, hk + ".info.dims") == "4 4 4" && val(app, hk + ".view") == "xy",
        "and its own volume and keys");
  check(cvc::state::instance(app)(hk + ".volume").data().empty(),
        "a grid read plane by plane has no volume in memory");
  picker hpick{app, d, hk + ".pick"};
  hpick.click(0, 0, 1);
  hpts = points_of(app, hk + ".points");
  check(hpts && hpts->empty() && val(app, hk + ".status").find("no class") != std::string::npos,
        "a click with no class marks nothing and says so", val(app, hk + ".status"));
  pts = points_of(app, k + ".points");
  check(pts && pts->point_count() == 3, "and leaves g alone");

  // ---- load another volume ----
  set(app, k + ".source.file", file3);
  set(app, k + ".request", "load");
  d.tick();
  check(val(app, k + ".info.dims") == "5 5 5" && val(app, k + ".slice.x") == "4" &&
            cvc::state::instance(app)(k + ".slice.x").property("max") == "4",
        "load reads grid.source.file; the planes are clamped to it",
        val(app, k + ".info.dims") + " " + val(app, k + ".slice.x") + " " +
            val(app, k + ".status"));
  set(app, k + ".source.file", (dir / "nope.rawiv").generic_string());
  set(app, k + ".request", "load");
  d.tick();
  check(val(app, k + ".status").rfind("error: load:", 0) == 0 &&
            val(app, k + ".info.dims") == "5 5 5",
        "a load that fails is reported and keeps the volume", val(app, k + ".status"));

  // ---- two apps ----
  cvc::app other;
  check(!cvc::gl::ariadne::has_scene_node_type(other, "volume_grid"),
        "an app that has not registered cvcGL's extensions has no volume_grid type");
}

// Rendered (offscreen): the plane's overlays draw on top of it, the crosshair lines and a point's
// disk in their colours. Skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.
void rendered(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!cvcgl_test::renderAvailable(app))
    return;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string file = (dir / "flat.rawiv").generic_string();
  cvc::writeVolumeFile(app, coded(app, 9, 9, 3), file);
  realized_doc d(app, "vg_draw",
                 R"(meta:
  name: grid draw
  min_libcvc: 3.7.0
scene:
  nodes:
    - node: g
      type: volume_grid
      source: { file: ")" +
                     file + R"(" }
      slice: { x: 2, y: 6, z: 1 }
      classes: [ { name: c, color: [0, 1, 0], radius: 1 } ]
      transfer_function: { color: [[0, 0, 0, 1], [1, 0, 0, 1]] }
)");
  for (const std::string &w : d.warnings)
    check(false, "the drawn document realizes without warnings", w);
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  if (!g) {
    check(false, "the drawn grid exists");
    return;
  }
  picker pick{app, d, g->stateName() + ".grid.pick"};
  pick.click(6, 2, 1); // a green disk at (6, 2) on the z = 1 plane
  cvc::gl::SceneRenderer sr(d.sg, 240, 240, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  cvcgl_test::topView(sr, 4.0, 4.0, 5.5);
  d.tick();
  const cvcgl_test::Frame f = cvcgl_test::grab(sr);
  const auto px = [&sr](double x, double y) { return cvcgl_test::toPx(sr, x, y, 1.0); };
  check(cvcgl_test::isHue(f.at(int(px(4.0, 4.0)[0]), int(px(4.0, 4.0)[1])), 2),
        "the z plane draws in its transfer function's blue");
  // A crosshair takes the colour of the axis it runs along: where the x plane crosses (x = 2) the
  // line runs along y, green; where the y plane crosses (y = 6), along x, red.
  check(cvcgl_test::hueNear(f, px(2.0, 3.5), 1, 1),
        "the crosshair where the x plane crosses draws (green, along y) on top of the plane");
  check(cvcgl_test::hueNear(f, px(4.5, 6.0), 1, 0),
        "the crosshair where the y plane crosses draws (red, along x) on top of the plane");
  check(cvcgl_test::isHue(f.at(int(px(6.0, 2.0)[0]), int(px(6.0, 2.0)[1])), 1),
        "the marked point draws as a green disk on top of the plane");
}

} // namespace

int main() {
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvcgl_volume_grid_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  run(dir);
  rendered(dir);
  std::error_code ec;
  fs::remove_all(dir, ec);
  return cvcgl_test::finish("cvcgl_volume_grid");
}
