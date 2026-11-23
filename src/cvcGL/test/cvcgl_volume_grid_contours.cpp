// cvcgl_volume_grid_contours — the `volume_grid` node's contours (VolumeGridContours.h;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P10), headless, driven through its state keys
// as a document, the DSL or pycvc drives it (T13's scripted picks):
//   - the props seed the object rows, grid.contour.* and grid.tile.*;
//   - with the contours tool, picks written to grid.pick.* draw a curve of the current object,
//     close it on its first point, move a point, select with a band; the overlays draw the curves
//     and their control points on their plane, and every plane's curves in the 3D view;
//   - requests: new_curve, close_curve, delete_curve, clear_object, and delete_selected / undo
//     acting on the contours under the contours tool and on the point classes under the points
//     tool;
//   - rows edited (spline, hidden), added and removed edit the objects;
//   - the data on the state tree: grid.curves, grid.contours (the z planes' stack, which a
//     contour_lab with source: { node: <grid> } follows), contours the app put at
//     grid.contour.source (a stack, converted onto the grid's planes) and a grid_contours written
//     to grid.curves from outside;
//   - tile: the tiler as a sliced task under grid.tile.*, a surface per object in the 3D view in
//     its colour, on the data channel at grid.surfaces.<object>; stop; auto_tile; the scene's
//     pending work while it runs;
//   - the session carries the contours;
//   - T6: two grids in one document share nothing.

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/core/sliced_task.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridContours.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid.h>
#include <cvc/volume/volume_grid_contours.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using cvcgl_test::check;

namespace {

constexpr double pi = 3.14159265358979323846;

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
  cvc::app &app;
  cvc::gl::SceneGraph sg;
  cvc::gl::ariadne::RealizedScene rs;
  std::vector<std::string> warnings;
  realized_doc(cvc::app &a, const std::string &name, const std::string &text)
      : app(a), sg(a, name) {
    sg.setDiagnosticChromeVisible(false);
    cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(a, text);
    for (const std::string &w : lr.warnings)
      warnings.push_back(w);
    rs = cvc::gl::ariadne::realize_scene(sg, lr.scene, sg.getStatePrefix(), &warnings);
  }
  void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
      for (const auto &t : rs.custom_ticks)
        t(nullptr);
      cvc::pump_exec_frame(app.exec_scheduler());
    }
  }
  bool pending() const {
    for (const auto &p : rs.pending_work)
      if (p && p())
        return true;
    return false;
  }
  bool settle() {
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (!pending())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
};

// A pick as a slice view writes it (seq last), at world (x, y) on the plane z, then a tick.
struct picker {
  cvc::app &app;
  realized_doc &d;
  std::string key;
  int seq = 0;
  void operator()(const char *phase, double x, double y, int z, const char *mods = "") {
    set(app, key + ".phase", phase);
    set(app, key + ".i", std::to_string(static_cast<int>(std::lround(x))));
    set(app, key + ".j", std::to_string(static_cast<int>(std::lround(y))));
    set(app, key + ".k", std::to_string(z));
    set(app, key + ".x", std::to_string(x));
    set(app, key + ".y", std::to_string(y));
    set(app, key + ".z", std::to_string(z));
    set(app, key + ".axis", "z");
    set(app, key + ".mods", mods);
    set(app, key + ".seq", std::to_string(++seq));
    d.tick();
  }
  void click(double x, double y, int z, const char *mods = "") {
    (*this)("down", x, y, z, mods);
    (*this)("up", x, y, z, mods);
  }
};

template <class T> std::shared_ptr<const T> data_at(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&d))
    return *p;
  return nullptr;
}

std::size_t lines_in(const std::shared_ptr<cvc::gl::GeometryNode> &g) {
  return g && g->isVisible() && g->getGeometry() ? g->getGeometry()->num_lines() : 0;
}

std::size_t tris_in(const std::shared_ptr<cvc::gl::GeometryNode> &g) {
  return g && g->isVisible() && g->getGeometry() ? g->getGeometry()->num_tris() : 0;
}

std::string doc() {
  return "meta:\n  name: contours test\n  min_libcvc: 3.7.0\nscene:\n  nodes:\n"
         "    - node: g\n      type: volume_grid\n      source: { state: /vols.g }\n"
         "      tool: contours\n      slice: { z: 4 }\n"
         "      classes: [ { name: membrane, color: [1, 0, 0] } ]\n"
         "      objects:\n"
         "        - { name: axon, color: [0.2, 0.8, 0.3] }\n"
         "        - { name: mito, color: [0.9, 0.3, 0.6], interpolation: spline, samples: 4 }\n"
         "      contour: { point_size: 0.5 }\n"
         "      tile: { curation_delta: 0.001 }\n"
         "    - node: h\n      type: volume_grid\n      source: { state: /vols.g }\n"
         "    - node: lab\n      type: contour_lab\n      source: { node: g }\n";
}

// Six points around (cx, cy) on plane z, turned by `phase` and pulled out of round (the tiler's
// untiled-region fill gives up on a regular polygon stacked on its copy), then a click on the
// first to close the curve.
void draw_ring(picker &pick, double cx, double cy, int z, double r = 4.0, double phase = 0) {
  double fx = 0, fy = 0;
  for (int i = 0; i < 6; ++i) {
    const double t = 2 * pi * i / 6 + phase;
    const double w = r * (1 + 0.15 * std::cos(2 * t + z));
    const double x = cx + w * std::cos(t), y = cy + 0.8 * w * std::sin(t);
    if (i == 0) {
      fx = x;
      fy = y;
    }
    pick.click(x, y, z);
  }
  pick.click(fx, fy, z);
}

void run() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  cvc::state &root = cvc::state::instance(app);
  root("vols.g").data(boost::any(std::make_shared<const cvc::volume>(coded(app, 32, 32, 12))));

  realized_doc d(app, "vgc", doc());
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  auto h = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("h"));
  check(g && h, "the grids are VolumeGridNodes");
  if (!g || !h)
    return;
  d.tick(2);
  const std::string k = g->stateName() + ".grid";
  cvc::gl::VolumeGridContours &gc = g->contours();
  using cvc::slice_axis;

  // ---- the props, the keys ----
  check(val(app, k + ".tool") == "contours", "the tool prop", val(app, k + ".tool"));
  check(val(app, k + ".objects") == "2" && val(app, k + ".objects.0.name") == "axon" &&
            val(app, k + ".objects.1.interpolation") == "spline" &&
            val(app, k + ".objects.1.samples") == "4" &&
            val(app, k + ".objects.0.color") == "0.2,0.8,0.3",
        "the objects prop seeds the rows", val(app, k + ".objects.0.color"));
  check(val(app, k + ".objects.current") == "0", "the first object is current");
  check(val(app, k + ".contour.point_size") == "0.5" && val(app, k + ".contour.show") == "1",
        "grid.contour.* from the prop and the table's defaults",
        val(app, k + ".contour.point_size"));
  check(val(app, k + ".tile.curation_delta") == "0.001" && val(app, k + ".tile.cap_ends") == "1",
        "grid.tile.* from the prop and the defaults", val(app, k + ".tile.curation_delta"));
  check(val(app, k + ".tile.busy") == "0", "the tile job's keys, idle, under grid.tile");
  check(gc.curvesOverlay(slice_axis::z) && gc.pointsOverlay(slice_axis::z) && gc.curves3d() &&
            gc.tiles(),
        "the contours' overlays on each plane and their children in the surfaces group");
  check(gc.curvesOverlay(slice_axis::z)->isVisible(), "shown from the first build (empty)");
  check(!gc.curves3d()->isVisible() && !gc.tiles()->isVisible(),
        "the 3D curves and the tiles stay hidden while empty (they frame no view)");

  // ---- drawing ----
  picker pick{app, d, g->stateName() + ".grid.pick"};
  pick.click(12, 12, 4);
  check(val(app, k + ".status").find("started a curve of axon on z = 4") != std::string::npos,
        "a click starts a curve of the current object", val(app, k + ".status"));
  for (int i = 1; i < 6; ++i) {
    const double t = 2 * pi * i / 6;
    pick.click(8 + 4 * std::cos(t), 12 + 4 * std::sin(t), 4);
  }
  check(val(app, k + ".contour.active").find("axon on z = 4, 6 points") != std::string::npos,
        "grid.contour.active names the curve being drawn", val(app, k + ".contour.active"));
  check(tris_in(gc.pointsOverlay(slice_axis::z)) == 6u * 12, "six control points drawn as disks",
        std::to_string(tris_in(gc.pointsOverlay(slice_axis::z))));
  const std::size_t open_lines = lines_in(gc.curvesOverlay(slice_axis::z));
  pick.click(12, 12, 4); // the first point: it closes
  auto curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects.size() == 2 && curves->objects[0].curves.size() == 1 &&
            curves->objects[0].curves[0].closed && curves->objects[0].curves[0].size() == 6,
        "grid.curves holds the closed curve");
  check(val(app, k + ".status").find("closed a curve of axon") != std::string::npos,
        "the click on the first point closes it", val(app, k + ".status"));
  check(open_lines == 5 + 12 && lines_in(gc.curvesOverlay(slice_axis::z)) == 6 + 12,
        "open: five spans and the active end's ring; closed: six spans and the selected "
        "point's ring",
        std::to_string(open_lines) + " " +
            std::to_string(lines_in(gc.curvesOverlay(slice_axis::z))));
  check(lines_in(gc.curvesOverlay(slice_axis::y)) == 0, "nothing on the y plane",
        std::to_string(lines_in(gc.curvesOverlay(slice_axis::y))));
  auto stack = data_at<cvc::contour_stack>(app, k + ".contours");
  check(stack && stack->polygon_count() == 1 && stack->indices() == std::vector<int>{4} &&
            stack->components.size() == 2,
        "grid.contours holds the z planes' stack, a component per object");
  check(val(app, k + ".objects.0.curves") == "1" && val(app, k + ".objects.0.points") == "6",
        "the rows count each object's curves and points");
  check(gc.curves3d()->isVisible() && lines_in(gc.curves3d()) == 6,
        "every plane's curves in the 3D view", std::to_string(lines_in(gc.curves3d())));
  check(val(app, k + ".contour.info") == "2 objects, 1 curve, 6 points", "grid.contour.info",
        val(app, k + ".contour.info"));

  // Move a point by dragging it.
  pick("down", 4, 12, 4); // point 3, at (4, 12)
  pick("drag", 3, 13, 4);
  pick("drag", 2, 13, 4);
  pick("up", 2, 13, 4);
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves[0].ru[6] == 2 &&
            curves->objects[0].curves[0].ru[7] == 13,
        "a drag from a control point moves it",
        curves ? std::to_string(curves->objects[0].curves[0].ru[6]) : "");
  set(app, k + ".request", "undo");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves[0].ru[6] == 4, "undo (the contours tool) puts it back");

  // The same ring on three more planes, and one of mito on z = 6.
  for (int z : {5, 6, 7}) {
    set(app, k + ".slice.z", std::to_string(z));
    d.tick();
    draw_ring(pick, 8 + 0.25 * (z - 4), 12, z, 4.0 + 0.5 * (z % 2), 0.3 * z);
  }
  set(app, k + ".objects.current", "1");
  d.tick();
  set(app, k + ".slice.z", "6");
  d.tick();
  draw_ring(pick, 22, 20, 6, 3.0);
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.size() == 4 && curves->objects[1].curves.size() == 1,
        "four rings of axon, one of mito");
  check(tris_in(gc.pointsOverlay(slice_axis::z)) == 12u * 12,
        "on z = 6: axon's and mito's control points",
        std::to_string(tris_in(gc.pointsOverlay(slice_axis::z))));
  // mito is a spline of 4 samples per span: 24 segments, closed.
  stack = data_at<cvc::contour_stack>(app, k + ".contours");
  check(stack && stack->polygons(6, "mito").size() == 1 &&
            stack->polygons(6, "mito")[0].size() == 24,
        "a spline object's stack polygon is its spline drawn");

  // contour_lab follows the grid's contours.
  d.tick(2);
  const std::string lab = d.sg.getGraphics("lab")->stateName() + ".tiler";
  check(val(app, lab + ".info.contours") == "5",
        "a contour_lab with source: { node: g } follows grid.contours",
        val(app, lab + ".info.contours") + " / " + val(app, lab + ".status"));

  // ---- tile ----
  set(app, k + ".request", "tile");
  d.tick();
  check(val(app, k + ".tile.busy") == "1" && d.pending(),
        "tile runs as a sliced task: busy under grid.tile, pending work for the scene");
  check(d.settle(), "the tile finishes");
  check(val(app, k + ".tile.status").rfind("tiled 2 objects", 0) == 0,
        "grid.tile.status says what it tiled", val(app, k + ".tile.status"));
  const boost::any as = root(k + ".surfaces.axon").data();
  const cvc::geometry *surface = boost::any_cast<cvc::geometry>(&as);
  check(surface && surface->num_tris() > 50, "grid.surfaces.axon holds the surface",
        (surface ? std::to_string(surface->num_tris()) : "none") + " / " +
            val(app, k + ".tile.warnings") + " " + val(app, k + ".tile.warnings.0") + " " +
            val(app, k + ".tile.warnings.1"));
  check(gc.tiles()->isVisible() && gc.tiles()->getGraphicsChildren().size() == 2,
        "a surface node per object in the 3D view's group");
  bool coloured = false;
  for (const auto &c : gc.tiles()->getGraphicsChildren())
    if (c->getName() == "axon")
      coloured = std::abs(std::stod(val(app, c->stateName("color_g"))) - 0.8) < 1e-6 &&
                 std::abs(std::stod(val(app, c->stateName("color_r"))) - 0.2) < 1e-6;
  check(coloured, "in its object's colour");
  check(val(app, k + ".objects.0.triangles") == std::to_string(surface ? surface->num_tris() : 0),
        "the row counts its surface's triangles", val(app, k + ".objects.0.triangles"));
  // Hidden objects hide their surfaces and curves.
  set(app, k + ".objects.0.visible", "0");
  d.tick();
  bool hidden = false;
  for (const auto &c : gc.tiles()->getGraphicsChildren())
    if (c->getName() == "axon")
      hidden = !c->isVisible();
  check(hidden, "a hidden object's surface hides");
  check(tris_in(gc.pointsOverlay(slice_axis::z)) == 6u * 12, "and its control points");
  set(app, k + ".objects.0.visible", "1");
  set(app, k + ".contour.surface_opacity", "0.5");
  d.tick();

  // ---- requests ----
  set(app, k + ".objects.current", "0");
  set(app, k + ".slice.z", "4");
  d.tick();
  set(app, k + ".request", "new_curve");
  d.tick();
  pick.click(25, 5, 4);
  pick.click(28, 5, 4);
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.size() == 5 && curves->objects[0].curves[4].size() == 2,
        "new_curve: the next click starts a curve");
  set(app, k + ".request", "delete_curve");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.size() == 4, "delete_curve takes the active one");
  // delete_selected acts on the tool's: the contours' selection, then the classes'.
  pick("down", 1, 1, 4);
  pick("drag", 30, 30, 4);
  pick("up", 30, 30, 4);
  check(val(app, k + ".contour.selected") == "6", "a band selects the control points on z = 4",
        val(app, k + ".contour.selected"));
  set(app, k + ".request", "delete_selected");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.size() == 3, "delete_selected deletes them");
  set(app, k + ".tool", "points");
  d.tick();
  pick.click(3, 3, 4);
  check(val(app, k + ".classes.0.points") == "1", "the points tool marks a point class");
  set(app, k + ".request", "undo");
  d.tick();
  check(val(app, k + ".classes.0.points") == "0", "undo under the points tool undoes the mark");
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.size() == 3, "and leaves the contours");
  set(app, k + ".tool", "contours");
  set(app, k + ".request", "clear_object");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects[0].curves.empty() && curves->objects[1].curves.size() == 1,
        "clear_object clears the current object's curves");

  // ---- rows ----
  set(app, k + ".objects.2.name", "dendrite");
  set(app, k + ".objects", "3");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects.size() == 3 && curves->objects[2].name == "dendrite" &&
            curves->objects[1].curves.size() == 1,
        "a row added adds an object; the others keep their curves");

  // ---- the data on the state tree ----
  cvc::contour_stack traced;
  traced.z_spacing = 1.0;
  for (int z = 2; z <= 6; ++z)
    traced.add(z, "traced", {10, 10, 16, 10, 16, 16, 10, 16});
  root("traced").data(boost::any(std::make_shared<const cvc::contour_stack>(traced)));
  set(app, k + ".contour.auto_tile", "1");
  set(app, k + ".contour.source", "/traced");
  d.tick();
  curves = data_at<cvc::grid_contours>(app, k + ".curves");
  check(curves && curves->objects.size() == 1 && curves->objects[0].name == "traced" &&
            curves->objects[0].curves.size() == 5 && curves->objects[0].curves[0].index == 2,
        "a stack at grid.contour.source replaces the contours, on the grid's planes");
  check(d.settle(), "auto_tile tiles them");
  check(val(app, k + ".tile.status").rfind("tiled 1 object", 0) == 0 &&
            !root(k + ".surfaces.traced").data().empty() &&
            root(k + ".surfaces.axon").data().empty(),
        "the new surfaces replace the old", val(app, k + ".tile.status"));
  // A grid_contours written to grid.curves from outside (a program, pycvc).
  cvc::grid_contours mine = *data_at<cvc::grid_contours>(app, k + ".curves");
  mine.objects[0].name = "renamed";
  root(k + ".curves").data(boost::any(std::make_shared<const cvc::grid_contours>(mine)));
  d.tick();
  check(val(app, k + ".objects.0.name") == "renamed", "a write to grid.curves replaces them");
  check(d.settle(), "and, with auto_tile, tiles them");

  // ---- stop ----
  set(app, k + ".contour.auto_tile", "0");
  cvc::grid_contours big;
  big.frame = gc.editor().frame();
  cvc::grid_contour_object o;
  o.name = "big";
  for (int z = 0; z < 12; ++z) {
    cvc::grid_curve cv;
    cv.index = z;
    for (int i = 0; i < 400; ++i)
      cv.ru.insert(cv.ru.end(),
                   {16 + 10 * std::cos(2 * pi * i / 400) * (1 + 0.1 * std::sin(7 * i + z)),
                    16 + 9 * std::sin(2 * pi * i / 400)});
    cv.closed = true;
    o.curves.push_back(cv);
  }
  big.objects.push_back(o);
  gc.setContours(big, "the test");
  set(app, k + ".request", "tile");
  d.tick();
  set(app, k + ".request", "stop");
  d.tick();
  check(d.settle(), "stop ends the tile");
  check(val(app, k + ".tile.status") == "cancelled", "stopped: cancelled",
        val(app, k + ".tile.status"));
  check(!root(k + ".surfaces.renamed").data().empty(), "the surfaces the grid had stay");

  // ---- trace: VolumeGridRover's isocontouring ----
  // The coded volume holds i + 10 j + 100 k: on z = 4 the samples below 470 lie below the line
  // i + 10 j = 70, a corner of the plane; traced, it is one loop around them.
  set(app, k + ".objects.current", "0");
  set(app, k + ".slice.z", "4");
  set(app, k + ".view", "xy");
  d.tick();
  const std::size_t before = gc.contours().objects[0].curves.size();
  set(app, k + ".request", "trace 470");
  d.tick();
  check(val(app, k + ".status").find("traced 1 curve of") != std::string::npos,
        "trace adds the shown plane's isocontour to the current object", val(app, k + ".status"));
  set(app, k + ".request", "trace 470 2 8 2");
  d.tick();
  // On planes 2 and 4 the corner below 470 is i + 10 j < 270 and < 70; 6 and 8 lie wholly above.
  check(gc.contours().objects[0].curves.size() == before + 3 &&
            gc.contours().objects[0].curves.back().index == 4 &&
            gc.contours().objects[0].curves.back().closed &&
            val(app, k + ".status").find("on 4 planes") != std::string::npos,
        "trace iso first last step: planes 2, 4, 6 and 8 traced, a loop where they cross",
        std::to_string(gc.contours().objects[0].curves.size() - before) + " " +
            val(app, k + ".status"));
  set(app, k + ".request", "trace");
  d.tick();
  check(val(app, k + ".status").rfind("error: trace takes", 0) == 0, "a bad trace says why",
        val(app, k + ".status"));

  // ---- the session ----
  const cvc::volume_grid_session s = g->session();
  check(s.contours == gc.contours() && s.keys.count("grid.tile.curation_delta") &&
            s.keys.count("grid.objects.current") && !s.keys.count("grid.contour.source"),
        "the session carries the contours and their keys (not the document's source)");

  // ---- T6 ----
  auto hc = data_at<cvc::grid_contours>(app, h->stateName() + ".grid.curves");
  check(hc && hc->objects.empty() && val(app, h->stateName() + ".grid.objects") == "0" &&
            h->contours().tiles()->getGraphicsChildren().empty(),
        "the other grid shares nothing");
}

} // namespace

int main() {
  run();
  return cvcgl_test::finish("cvcgl_volume_grid_contours");
}
