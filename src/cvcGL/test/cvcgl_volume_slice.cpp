// cvcgl_volume_slice — the `volume_slice` scene node (VolumeSliceNode.h, volume_slice_nodes.h;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6 items 1 and 4):
//   - a document's volume_slice node realizes a VolumeSliceNode from a file, its props seeding
//     <node>.slice.* and its transfer function, and publishes info.* and slice.{count,position};
//   - its keys are live: a new index cuts another plane into the same texture (zero copy), a new
//     axis makes a texture of the new plane's size, an index past the last plane is clamped and
//     written back, a key that does not parse is reported in slice.status, and an edit of
//     <node>.transfer_function recolours the plane;
//   - region_reads: true reads the file one plane at a time and draws the same pixels;
//   - the quad covers the plane's voxels with half a voxel to spare, at the plane's position;
//   - pickRay: a ray meets the plane at the voxel it should, with that voxel's value, through the
//     node's transform; a ray along the plane misses;
//   - two apps: one that has not registered cvcGL's extensions has no volume_slice type;
//   - rendered (offscreen): the plane's voxels draw in their transfer-function colours, square
//     with the nearest filter, and an overlay in the plane (a child node) draws on top of it.
// The rendered part skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.

#include "streaming_test_util.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/tf/model.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkMapper.h>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

// Voxel (i, j, k) of an nx x ny x nz float volume over [0, nx-1] x [0, ny-1] x [0, 2(nz-1)]
// holds i + 10 j + 100 k.
cvc::volume coded(cvc::app &app, int nx, int ny, int nz) {
  cvc::volume v(app, cvc::dimension(nx, ny, nz), cvc::Float,
                cvc::bounding_box(0.0, 0.0, 0.0, nx - 1.0, ny - 1.0, 2.0 * (nz - 1)));
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

std::shared_ptr<cvc::gl::VolumeSliceNode> slice_node(cvc::gl::SceneGraph &sg,
                                                     const std::string &id) {
  return std::dynamic_pointer_cast<cvc::gl::VolumeSliceNode>(sg.getGraphics(id));
}

std::string doc(const std::string &file, const std::string &node_extra) {
  return "meta:\n  name: slice test\n  min_libcvc: 3.7.0\nscene:\n  nodes:\n"
         "    - node: s\n      type: volume_slice\n      source: { file: \"" +
         file + "\" }\n" + node_extra;
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
  void tick() { cvc::gl::ariadne::tick_scene(rs, nullptr, 1.0 / 30.0); }
};

void headless(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  check(cvc::gl::ariadne::has_scene_node_type(app, "volume_slice"),
        "register_cvcgl_extensions installs volume_slice");
  const std::string file = (dir / "coded.rawiv").generic_string();
  cvc::writeVolumeFile(app, coded(app, 6, 5, 4), file);

  realized_doc d(app, "slice", doc(file, "      axis: xz\n      index: 2\n"));
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto n = slice_node(d.sg, "s");
  check(n != nullptr, "a volume_slice node is a VolumeSliceNode");
  if (!n)
    return;
  const std::string k = n->stateName();
  check(val(app, k + ".slice.axis") == "y", "axis: xz is the plane down y",
        val(app, k + ".slice.axis"));
  check(n->params().axis == cvc::slice_axis::y && n->params().index == 2, "drawn: y, plane 2");
  check(val(app, k + ".slice.count") == "5", "slice.count: 5 planes down y");
  check(val(app, k + ".slice.position") == "2", "slice.position: y = 2",
        val(app, k + ".slice.position"));
  check(val(app, k + ".info.dims") == "6 5 4", "info.dims", val(app, k + ".info.dims"));
  check(val(app, k + ".info.min") == "0" && val(app, k + ".info.max") == "345",
        "info.min / info.max", val(app, k + ".info.min") + " " + val(app, k + ".info.max"));
  check(val(app, k + ".info.span") == "1 1 2", "info.span", val(app, k + ".info.span"));
  check(cvc::state::instance(app)(k + ".slice.index").property("max") == "4",
        "slice.index's max property is the last plane");
  check(!val(app, k + ".transfer_function").empty(), "the transfer function key is seeded");
  // The plane down y: x right (6), z up (4), grayscale by default: the top-right voxel (5, 2, 3)
  // is the brightest of the plane, its bottom-left (0, 2, 0) the darkest.
  check(n->textureWidth() == 6 && n->textureHeight() == 4, "a 6 x 4 texture",
        std::to_string(n->textureWidth()) + "x" + std::to_string(n->textureHeight()));
  const std::vector<std::uint8_t> &px = n->pixels();
  check(px.size() == 6u * 4u * 4u, "its pixels");
  if (px.size() == 96) {
    const int top_right = px[(0 * 6 + 5) * 4], bottom_left = px[(3 * 6 + 0) * 4];
    const double want_tr = (5 + 20 + 300) / 345.0 * 255.0, want_bl = 20 / 345.0 * 255.0;
    check(std::abs(top_right - want_tr) <= 1.5 && std::abs(bottom_left - want_bl) <= 1.5,
          "grayscale over the data range",
          std::to_string(top_right) + " " + std::to_string(bottom_left));
    check(px[3] == 255, "opaque by default");
  }
  // The quad: at y = 2, half a voxel past the outer voxels in x and z.
  const cvc::bounding_box bb = n->getBoundingBox();
  check(std::fabs(bb.miny - 2.0) < 1e-9 && std::fabs(bb.maxy - 2.0) < 1e-9,
        "the quad lies at y = 2");
  check(std::fabs(bb.minx + 0.5) < 1e-9 && std::fabs(bb.maxx - 5.5) < 1e-9 &&
            std::fabs(bb.minz + 1.0) < 1e-9 && std::fabs(bb.maxz - 7.0) < 1e-9,
        "half a voxel to spare on each side",
        std::to_string(bb.minx) + " " + std::to_string(bb.maxx) + " " + std::to_string(bb.minz) +
            " " + std::to_string(bb.maxz));

  // A new index: the same texture, new pixels in it.
  const std::size_t up0 = n->uploads();
  set(app, k + ".slice.index", "4");
  d.tick();
  check(n->params().index == 4 && n->slice() && n->slice()->index == 4, "index 4 is drawn");
  check(n->uploads() == up0 + 1 && n->textureWidth() == 6, "one upload, into the same texture");
  check(val(app, k + ".slice.position") == "4", "slice.position follows");
  d.tick();
  check(n->uploads() == up0 + 1, "nothing changed: nothing uploaded");
  // Past the last plane: drawn as the last, written back.
  set(app, k + ".slice.index", "99");
  d.tick();
  check(n->params().index == 4 && val(app, k + ".slice.index") == "4",
        "an index past the last plane is clamped and written back", val(app, k + ".slice.index"));
  // A new axis: a texture of the new plane's size (down x: z right 4, y up 5).
  set(app, k + ".slice.axis", "x");
  set(app, k + ".slice.index", "-1");
  d.tick();
  check(n->params().axis == cvc::slice_axis::x && n->params().index == 2,
        "axis x, index -1 is the middle plane", std::to_string(n->params().index));
  check(val(app, k + ".slice.index") == "2", "the middle written back");
  check(n->textureWidth() == 4 && n->textureHeight() == 5, "a 4 x 5 texture down x",
        std::to_string(n->textureWidth()) + "x" + std::to_string(n->textureHeight()));
  check(cvc::state::instance(app)(k + ".slice.index").property("max") == "5",
        "slice.index's max follows the axis");
  // A bad key: reported, the last good value kept.
  set(app, k + ".slice.axis", "sideways");
  d.tick();
  check(!val(app, k + ".slice.status").empty() && n->params().axis == cvc::slice_axis::x,
        "a bad axis is reported in slice.status and the plane stays",
        val(app, k + ".slice.status"));
  set(app, k + ".slice.axis", "x");
  d.tick();
  check(val(app, k + ".slice.status").empty(), "fixed: slice.status is empty again");
  // The transfer function: an edit recolours the plane.
  const std::vector<std::uint8_t> gray = n->pixels();
  cvc::tf::model hot = cvc::tf::model::preset("hot");
  set(app, k + ".transfer_function", cvc::tf::to_string(hot));
  d.tick();
  check(n->pixels() != gray, "a transfer-function edit recolours the plane");
  // filter / opaque reach the texture and the actor.
  set(app, k + ".slice.filter", "linear");
  set(app, k + ".slice.opaque", "0");
  d.tick();
  check(n->textureInterpolate(), "filter: linear samples linearly");
  auto *actor = vtkActor::SafeDownCast(n->prop());
  check(actor && actor->GetForceTranslucent() && !actor->GetForceOpaque(),
        "opaque: 0 blends the plane");
  // The overlay offset: the quad pushed back.
  double factor = 0.0, units = 0.0;
  if (actor && actor->GetMapper())
    actor->GetMapper()->GetRelativeCoincidentTopologyPolygonOffsetParameters(factor, units);
  check(factor == 1.0 && units == 2.0, "the quad is pushed back by the overlay offset",
        std::to_string(factor) + " " + std::to_string(units));

  // pickRay: down -z onto a plane down z through voxel (3, 1, 2), then through a transform.
  set(app, k + ".slice.axis", "z");
  set(app, k + ".slice.index", "2");
  d.tick();
  {
    const double o[3] = {3.2, 0.9, 50.0}, dir[3] = {0.0, 0.0, -1.0};
    double w[3];
    std::int64_t ijk[3];
    double v = 0.0;
    bool inside = false;
    const bool hit = n->pickRay(o, dir, w, ijk, v, inside);
    check(hit && inside && ijk[0] == 3 && ijk[1] == 1 && ijk[2] == 2 && v == 3 + 10 + 200,
          "a ray down z meets voxel (3, 1, 2) and its value",
          std::to_string(ijk[0]) + " " + std::to_string(ijk[1]) + " " + std::to_string(ijk[2]) +
              " = " + std::to_string(v));
    check(std::fabs(w[2] - 4.0) < 1e-12 && std::fabs(w[0] - 3.2) < 1e-12, "at the plane, exactly");
    const double along[3] = {1.0, 0.0, 0.0};
    check(!n->pickRay(o, along, w, ijk, v, inside), "a ray along the plane misses");
    const double past[3] = {40.0, 0.9, 50.0};
    check(n->pickRay(past, dir, w, ijk, v, inside) && !inside && std::isnan(v),
          "outside the volume: a point on the plane, no voxel");
    n->setPosition(100.0, 0.0, 0.0);
    const double moved[3] = {103.2, 0.9, 50.0};
    const bool through = n->pickRay(moved, dir, w, ijk, v, inside);
    check(through && inside && ijk[0] == 3 && std::fabs(w[0] - 103.2) < 1e-9,
          "through the node's transform", std::to_string(ijk[0]) + " at x " + std::to_string(w[0]));
    n->setPosition(0.0, 0.0, 0.0);
  }

  // region_reads: true reads plane by plane and draws the same pixels.
  {
    realized_doc r(app, "slice_region",
                   doc(file, "      axis: y\n      index: 3\n      region_reads: true\n"
                             "      transfer_function: { preset: hot }\n"));
    for (const std::string &w : r.warnings)
      check(false, "the region-read document realizes without warnings", w);
    realized_doc m(app, "slice_whole",
                   doc(file, "      axis: y\n      index: 3\n      region_reads: false\n"
                             "      transfer_function: { preset: hot }\n"));
    auto rn = slice_node(r.sg, "s"), mn = slice_node(m.sg, "s");
    check(rn && mn && rn->source() && !rn->source()->in_memory() && mn->source()->in_memory(),
          "region_reads: true reads the file plane by plane; false reads it whole");
    check(rn && mn && rn->pixels() == mn->pixels() && !rn->pixels().empty(),
          "the same pixels either way");
    check(rn && rn->source()->reads() >= 1, "a region read happened");
    cvc::tf::model seeded;
    check(rn && cvc::tf::from_string(val(app, rn->stateName("transfer_function")), seeded) &&
              seeded == cvc::tf::model::preset("hot"),
          "transfer_function: { preset: hot } seeds the model");
  }

  // Bad input: no source, a missing file, an axis that is not one.
  {
    realized_doc b(app, "slice_bad",
                   "meta:\n  name: bad\n  min_libcvc: 3.7.0\nscene:\n  nodes:\n"
                   "    - { node: a, type: volume_slice }\n"
                   "    - { node: b, type: volume_slice, source: { file: \"" +
                       (dir / "nope.rawiv").generic_string() +
                       "\" } }\n"
                       "    - { node: c, type: volume_slice, axis: w, source: { file: \"" +
                       file + "\" } }\n");
    bool no_source = false, missing = false, axis = false;
    for (const std::string &w : b.warnings) {
      no_source = no_source || w.find("'a' (volume_slice) has no source") != std::string::npos;
      missing = missing || w.find("nope.rawiv") != std::string::npos;
      axis = axis || w.find("axis 'w'") != std::string::npos;
    }
    check(no_source && missing && axis, "no source, a missing file and a bad axis each warn");
    check(!b.sg.getGraphics("a") && !b.sg.getGraphics("b") && slice_node(b.sg, "c"),
          "and only the good node is realized");
  }

  // Two apps: the type is the app's that registered it.
  cvc::app other;
  check(!cvc::gl::ariadne::has_scene_node_type(other, "volume_slice"),
        "another app has no volume_slice type until it registers cvcGL's extensions");
}

void rendered(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!cvcgl_test::renderAvailable(app))
    return;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  // A 4 x 4 x 2 volume whose plane k = 0 holds 0..15 row by row (k = 1 holds 15), through a step
  // transfer function: values below 8 red, the rest blue.
  cvc::volume v(app, cvc::dimension(4, 4, 2), cvc::Float, cvc::bounding_box(0, 0, 0, 3, 3, 1));
  for (int j = 0; j < 4; ++j)
    for (int i = 0; i < 4; ++i) {
      v(i, j, 0, j * 4.0 + i);
      v(i, j, 1, 15.0);
    }
  const std::string file = (dir / "step.rawiv").generic_string();
  cvc::writeVolumeFile(app, v, file);
  realized_doc d(app, "slice_draw",
                 doc(file, "      index: 0\n"
                           "      transfer_function:\n"
                           "        color: [[0, 1, 0, 0], [0.5, 1, 0, 0], [0.5001, 0, 0, 1], "
                           "[1, 0, 0, 1]]\n"
                           "      children:\n"
                           "        - node: mark\n"
                           "          source: { plane: { size: 0.5 } }\n"
                           "          transform: { position: [1.5, 3.0, 0.0] }\n"
                           "          material: { color: [0, 1, 0], ambient: 1, diffuse: 0 }\n"));
  for (const std::string &w : d.warnings)
    check(false, "the drawn document realizes without warnings", w);
  auto n = slice_node(d.sg, "s");
  if (!n) {
    check(false, "the drawn slice exists");
    return;
  }
  cvc::gl::SceneRenderer sr(d.sg, 200, 200, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  cvcgl_test::topView(sr, 1.5, 1.5, 2.5);
  d.tick();
  cvcgl_test::Frame f = cvcgl_test::grab(sr);
  // Voxel (i, j) is the square around (i, j): rows 0-1 red (values 0-7), rows 2-3 blue.
  check(cvcgl_test::isHue(f.at(int(cvcgl_test::toPx(sr, 0.0, 0.0, 0)[0]),
                               int(cvcgl_test::toPx(sr, 0.0, 0.0, 0)[1])),
                          0),
        "voxel (0, 0) draws red");
  const auto b21 = cvcgl_test::toPx(sr, 2.0, 2.0, 0);
  check(cvcgl_test::isHue(f.at(int(b21[0]), int(b21[1])), 2), "voxel (2, 2) draws blue");
  // Nearest: just inside a voxel's square, its own colour, not a blend with the next row's.
  const auto edge = cvcgl_test::toPx(sr, 1.0, 1.4, 0);
  check(cvcgl_test::isHue(f.at(int(edge[0]), int(edge[1])), 0),
        "the nearest filter keeps a voxel's square its own colour up to its edge");
  // The overlay exactly in the plane (a flat square at z = 0 round (1.5, 3): the top row) draws on
  // top of the blue voxels.
  const auto mark = cvcgl_test::toPx(sr, 1.5, 3.0, 0);
  check(cvcgl_test::isHue(f.at(int(mark[0]), int(mark[1])), 1),
        "the overlay in the plane draws on top of it");
  // Outside the quad (past half a voxel): the background.
  const auto out = cvcgl_test::toPx(sr, -0.8, 1.5, 0);
  check(cvcgl_test::isBg(f.at(int(out[0]), int(out[1]))), "nothing past the quad");
}

} // namespace

int main() {
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvcgl_volume_slice_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  headless(dir);
  rendered(dir);
  std::error_code ec;
  fs::remove_all(dir, ec);
  return cvcgl_test::finish("cvcgl_volume_slice");
}
