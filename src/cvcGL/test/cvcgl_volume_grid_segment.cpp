// cvcgl_volume_grid_segment — the volume_grid node's Segmentation tab (VolumeGridSegmentation.h;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, §5, P9), driven through its state keys as a
// document, the DSL or pycvc drives it (T13's scripted picks), with the app's scheduler pumped as a
// host pumps it:
//   - the keys: the three parameter tables seeded and described under grid.segment, a document's
//     segment: props seeding them, the thresholds' sliders spanning the volume's range;
//   - seeds marked by picks, then `segment`: the job keys rise and settle, the result lands on the
//     data channel (labels, the classes' volumes named after them, the border, each class's
//     surface), the rows take the point classes' colours, the info and status say what it did;
//   - the labels draw on each plane as a see-through layer following the plane; the surfaces in
//     the `segments` group of the 3D view; show, opacity, show_surfaces and a row's colour and show
//     change them; while it marches (a cooperative run) the labels so far draw;
//   - `stop` keeps the last result; `segment` while one runs says so; `stop` with none runs on;
//     a bad parameter and no seeds are reported; a new volume clears the result;
//   - a session keeps the tab's parameters; a document's `request: segment` opens segmented;
//   - T6: two apps, each with a grid, segment at once and share nothing;
//   - rendered (offscreen): each class's labels show over the plane in its colour.

#include "streaming_test_util.h"

#include <array>
#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/geometry.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/NullGraphicNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/VolumeGridNode.h>
#include <cvc/gl/VolumeGridSegmentation.h>
#include <cvc/gl/VolumeSliceNode.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <cvc/volume/point_classes.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid.h>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using cvcgl_test::check;

namespace {

constexpr int N = 24;

// Two Gaussian blobs of height 1 on an n^3 grid one unit apart, at x = n/4 and 3n/4 (6 and 18 for
// N), y = z = n/2, sigma n/8.
cvc::volume blobs(cvc::app &app, int n = N) {
  cvc::volume v(app, cvc::dimension(n, n, n), cvc::Float,
                cvc::bounding_box(0.0, 0.0, 0.0, n - 1.0, n - 1.0, n - 1.0));
  const double c = n / 2.0, sigma = n / 8.0;
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i) {
        double s = 0.0;
        for (const double cx : {n / 4.0, 3.0 * n / 4.0}) {
          const double d2 = (i - cx) * (i - cx) + (j - c) * (j - c) + (k - c) * (k - c);
          s += std::exp(-d2 / (2.0 * sigma * sigma));
        }
        v(i, j, k, s);
      }
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
  // The nodes tick, then the scheduler steps its tasks (as a host's frame does).
  void tick(int n = 1) {
    for (int i = 0; i < n; ++i) {
      cvc::gl::ariadne::tick_scene(rs, nullptr, 1.0 / 30.0);
      cvc::pump_exec_frame(app.exec_scheduler());
    }
  }
  // The nodes alone, the scheduler not stepped: a cooperative run does not advance, so a request
  // made next reaches it running (a fast machine ends a 24^3 run in a frame or two).
  void tick_nodes() { cvc::gl::ariadne::tick_scene(rs, nullptr, 1.0 / 30.0); }
  bool pending() const {
    for (const auto &p : rs.pending_work)
      if (p && p())
        return true;
    return false;
  }
  // Tick until no node has work in flight (a segmentation running); bounded at about two minutes.
  // `each` (if given) runs after every tick.
  bool settle(const std::function<void()> &each = {}) {
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (each)
        each();
      if (!pending()) {
        tick(); // the result applied, published and drawn
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
};

// A click as a slice view writes it (down, then up), each a tick.
struct picker {
  cvc::app &app;
  realized_doc &d;
  std::string key;
  int seq = 0;
  void write(const char *phase, int i, int j, int k) {
    set(app, key + ".phase", phase);
    set(app, key + ".i", std::to_string(i));
    set(app, key + ".j", std::to_string(j));
    set(app, key + ".k", std::to_string(k));
    set(app, key + ".x", std::to_string(i));
    set(app, key + ".y", std::to_string(j));
    set(app, key + ".z", std::to_string(k));
    set(app, key + ".axis", "z");
    set(app, key + ".mods", "");
    set(app, key + ".seq", std::to_string(++seq));
    d.tick();
  }
  void click(int i, int j, int k) {
    write("down", i, j, k);
    write("up", i, j, k);
  }
};

template <class T> std::shared_ptr<const T> data_at(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  if (const auto *p = boost::any_cast<std::shared_ptr<const T>>(&d))
    return *p;
  return nullptr;
}

std::string doc(const std::string &name, const std::string &vol, const std::string &extra = "",
                const std::string &segment_extra = "") {
  return "meta:\n  name: " + name +
         "\n  min_libcvc: 4.0.0\nscene:\n  nodes:\n"
         "    - node: g\n      type: volume_grid\n      source: { state: " +
         vol +
         " }\n"
         "      slice: { z: 12 }\n"
         "      classes:\n"
         "        - { name: left, color: [1, 0, 0], radius: 1 }\n"
         "        - { name: right, color: [0, 0.5, 1], radius: 1 }\n"
         "      segment: { t_low: 0.15, t_high: 0.9, gamma: -0.04, opacity: 0.8,\n"
         "                 mesh: { smoothing: 4 }" +
         segment_extra +
         " }\n"
         "      transfer_function: { preset: grayscale }\n" +
         extra;
}

// Two classes, a seed at each blob's centre of an n^3 blobs volume.
std::shared_ptr<const cvc::point_classes> blob_seeds(int n = N) {
  auto pc = std::make_shared<cvc::point_classes>();
  cvc::point_class l, r;
  l.name = "left";
  l.points = {n / 4, n / 2, n / 2};
  r.name = "right";
  r.points = {3 * n / 4, n / 2, n / 2};
  pc->classes = {l, r};
  return pc;
}

// The seeds: a click on each blob's centre on the z = 12 plane, the first class then the second.
void mark_seeds(cvc::app &app, realized_doc &d, const std::string &k) {
  picker pick{app, d, k + ".pick"};
  set(app, k + ".classes.current", "0");
  d.tick();
  pick.click(6, 12, 12);
  set(app, k + ".classes.current", "1");
  d.tick();
  pick.click(18, 12, 12);
}

void run() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const auto vol = std::make_shared<const cvc::volume>(blobs(app));
  cvc::state::instance(app)("vols.b").data(boost::any(vol));
  realized_doc d(app, "seg", doc("segmentation test", "/vols.b"));
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  check(static_cast<bool>(g), "a volume_grid node");
  if (!g)
    return;
  d.tick(2);
  const std::string k = g->stateName() + ".grid";
  cvc::gl::VolumeGridSegmentation &seg = g->segmentation();
  cvc::state &root = cvc::state::instance(app);

  // ---- the keys ----
  check(val(app, k + ".segment.t_low") == "0.15" && val(app, k + ".segment.gamma") == "-0.04" &&
            val(app, k + ".segment.opacity") == "0.8" &&
            val(app, k + ".segment.mesh.smoothing") == "4",
        "the document's segment: props seed the tab's keys", val(app, k + ".segment.t_low"));
  check(val(app, k + ".segment.automatic") == "local_maxima" &&
            val(app, k + ".segment.show") == "1" && val(app, k + ".segment.live") == "1" &&
            val(app, k + ".segment.mesh.close") == "1",
        "the rest take their defaults");
  check(root(k + ".segment.gamma").property("min") == "-1" &&
            !root(k + ".segment.opacity").property("doc").empty() &&
            root(k + ".segment.mesh.smoothing").property("max") == "1000",
        "the keys are described (ranges, docs)");
  check(std::abs(std::stod(root(k + ".segment.t_low").property("min")) - g->source()->min()) <
                1e-9 &&
            std::abs(std::stod(root(k + ".segment.t_high").property("max")) - g->source()->max()) <
                1e-9 &&
            std::abs(g->source()->max() - (1.0 + std::exp(-8.0))) < 1e-6,
        "the thresholds' sliders span the volume's data range",
        root(k + ".segment.t_low").property("min") + " .. " +
            root(k + ".segment.t_low").property("max"));
  set(app, k + ".segment.normalized_thresholds", "1");
  d.tick();
  check(root(k + ".segment.t_low").property("max") == "255",
        "or GenSeg's 0..255 scale with legacy thresholds");
  set(app, k + ".segment.normalized_thresholds", "0");
  d.tick();
  check(val(app, k + ".segment.rows") == "0" && val(app, k + ".files.segments") == "segments.rawv",
        "no result yet; Save's file has a default");
  check(!seg.labelsOverlay(cvc::slice_axis::z), "no label layer before a result");

  // ---- errors before a run ----
  set(app, k + ".request", "stop");
  d.tick();
  check(val(app, k + ".status") == "nothing to stop", "stop with nothing running says so",
        val(app, k + ".status"));
  set(app, k + ".request", "segment"); // the classes have no points yet
  d.settle();
  check(val(app, k + ".status").find("no seeds") != std::string::npos &&
            val(app, k + ".busy") == "0",
        "segment with no seeds fails and says why", val(app, k + ".status"));
  set(app, k + ".segment.t_low", "lots");
  set(app, k + ".request", "segment");
  d.tick();
  check(val(app, k + ".status").find("error: segment.t_low") == 0 && !seg.running(),
        "a parameter that does not parse is reported, and nothing runs", val(app, k + ".status"));
  set(app, k + ".segment.t_low", "0.15");

  // ---- seeds by picks, then a run ----
  mark_seeds(app, d, k);
  check(val(app, k + ".classes.0.points") == "1" && val(app, k + ".classes.1.points") == "1",
        "a seed in each class");
  set(app, k + ".request", "segment");
  d.tick_nodes();
  check(seg.running() && val(app, k + ".busy") == "1" && val(app, k + ".stoppable") == "1",
        "segment starts the run, published as the job keys", val(app, k + ".status"));
  check(d.pending(), "the scene has work in flight while it runs");
  set(app, k + ".request", "segment");
  d.tick_nodes();
  check(val(app, k + ".status").find("segmenting already") == 0 || !seg.running(),
        "segment while one runs is left alone, and says so", val(app, k + ".status"));
  check(d.settle(), "the run ends");
  check(val(app, k + ".busy") == "0" && val(app, k + ".progress") == "100" &&
            val(app, k + ".phase").empty(),
        "the job keys settle", val(app, k + ".busy") + " " + val(app, k + ".progress"));
  check(val(app, k + ".status").find("segmented: 2 classes") == 0 &&
            val(app, k + ".status").find("2 surfaces") != std::string::npos,
        "status says what the run did", val(app, k + ".status"));

  // The data channel.
  const auto labels = data_at<cvc::volume>(app, k + ".segment.labels");
  check(labels && labels->voxelType() == cvc::UChar && labels->XDim() == cvc::uint64(N) &&
            labels->boundingBox() == vol->boundingBox(),
        "grid.segment.labels: a UChar volume over the grid's volume");
  if (labels) {
    check((*labels)(6, 12, 12) == 0.0 && (*labels)(18, 12, 12) == 1.0 &&
              (*labels)(0, 0, 0) == 255.0,
          "each blob's core is its seed's class; a far corner none");
  }
  const auto classes = data_at<std::vector<cvc::volume>>(app, k + ".segment.classes");
  check(classes && classes->size() == 2 && (*classes)[0].desc() == "left" &&
            (*classes)[1].desc() == "right" &&
            std::abs((*classes)[0](6, 12, 12) - (*vol)(6, 12, 12)) < 1e-6 &&
            (*classes)[0](18, 12, 12) == 0.0,
        "grid.segment.classes: each class's voxels, zero elsewhere, named after the class");
  const auto border = data_at<cvc::point_classes>(app, k + ".segment.border");
  check(border && border->classes.size() == 1 && border->classes[0].name == "border" &&
            !border->classes[0].points.empty(),
        "grid.segment.border: the voxels where a front stopped");
  const boost::any s0 = root(k + ".segment.surfaces.0").data();
  const auto *surf0 = boost::any_cast<cvc::geometry>(&s0);
  check(surf0 && surf0->num_tris() > 0, "grid.segment.surfaces.0: the first class's surface");

  // The rows and the info.
  check(val(app, k + ".segment.rows") == "2" && val(app, k + ".segment.rows.0.name") == "left" &&
            val(app, k + ".segment.rows.0.color") == "1,0,0" &&
            val(app, k + ".segment.rows.1.color") == "0,0.5,1" &&
            val(app, k + ".segment.rows.1.visible") == "1",
        "the rows: the classes, in their point classes' colours");
  check(std::stoul(val(app, k + ".segment.rows.0.voxels")) > 20 &&
            std::stoul(val(app, k + ".segment.rows.0.triangles")) == surf0->num_tris(),
        "each row's voxels and triangles", val(app, k + ".segment.rows.0.voxels"));
  check(val(app, k + ".segment.info.classes") == "2" &&
            std::stoul(val(app, k + ".segment.info.border")) == border->classes[0].size(),
        "the info");

  // What draws: a label layer on each plane, the surfaces in the 3D group.
  using cvc::slice_axis;
  for (slice_axis a : {slice_axis::x, slice_axis::y, slice_axis::z}) {
    const auto layer = seg.labelsOverlay(a);
    check(layer && layer->isVisible() && layer->hasTexture(),
          std::string("the ") + cvc::slice_axis_name(a) + " plane shows its labels");
  }
  const auto zl = seg.labelsOverlay(slice_axis::z);
  check(zl && zl->textureWidth() == N, "the z layer's texture is a texel a voxel");
  auto segments = seg.segments();
  check(segments && segments->getGraphicsChildren().size() == 2, "two class surfaces in 3D");
  const auto child =
      [&segments](const std::string &name) -> std::shared_ptr<cvc::gl::GeometryNode> {
    if (segments)
      for (const auto &c : segments->getGraphicsChildren())
        if (c->getName() == name)
          return std::dynamic_pointer_cast<cvc::gl::GeometryNode>(c);
    return nullptr;
  };
  const auto c0 = child("class_0"), c1 = child("class_1");
  check(c0 && c1 && c0->isVisible() && c1->isVisible(), "class_0 and class_1, shown");
  check(segments && segments->getParentNode() == g->surfaces().get(),
        "the surfaces sit in the grid's surfaces group (slice views hide it)");

  // The plane moves: its layer follows.
  const std::size_t uploads = seg.layerUploads();
  set(app, k + ".slice.z", "10");
  d.tick(2);
  check(seg.layerUploads() > uploads, "a plane moved recolours its layer");
  // Display keys.
  set(app, k + ".segment.show", "0");
  d.tick();
  check(zl && !zl->isVisible(), "show 0 hides the labels");
  set(app, k + ".segment.show", "1");
  set(app, k + ".segment.opacity", "0.3");
  const std::size_t before_opacity = seg.layerUploads();
  d.tick();
  check(zl && zl->isVisible() && seg.layerUploads() > before_opacity,
        "show 1 and a new opacity redraw them");
  set(app, k + ".segment.show_surfaces", "0");
  d.tick();
  check(c0 && !c0->isVisible() && !c1->isVisible(), "show_surfaces 0 hides the surfaces");
  set(app, k + ".segment.show_surfaces", "1");
  set(app, k + ".segment.rows.1.visible", "0");
  const std::size_t before_row = seg.layerUploads();
  d.tick();
  check(c0 && c0->isVisible() && !c1->isVisible(), "a row's show hides that class's surface");
  check(seg.layerUploads() > before_row && !seg.rows()[1].visible,
        "and its labels (the layers redrawn)");
  set(app, k + ".segment.rows.1.visible", "1");
  set(app, k + ".segment.rows.0.color", "0,1,0");
  d.tick();
  check(seg.rows()[0].color == (std::array<double, 3>{0, 1, 0}), "a row's colour recolours it");

  // ---- a session keeps the tab's parameters ----
  const cvc::volume_grid_session sess = g->session();
  check(sess.keys.count("grid.segment.t_low") && sess.keys.count("grid.segment.mesh.smoothing") &&
            sess.keys.count("grid.segment.opacity") && !sess.keys.count("grid.segment.rows") &&
            !sess.keys.count("grid.segment.info.classes"),
        "a session keeps the tab's parameters, not its result");

  // ---- stop keeps the last result ----
  // The scheduler does not step between segment and stop: a cooperative run has not begun when
  // stop comes, and a threaded one has had only the moment between the two requests.
  set(app, k + ".request", "segment");
  d.tick_nodes();
  set(app, k + ".request", "stop");
  d.tick_nodes();
  d.settle();
  check(val(app, k + ".status") == "cancelled" && !seg.running(), "stop ends the run",
        val(app, k + ".status"));
  check(data_at<cvc::volume>(app, k + ".segment.labels") == labels &&
            val(app, k + ".segment.rows") == "2" && zl && zl->isVisible(),
        "and keeps the last result");

  // ---- a new volume clears it ----
  root("vols.b").data(boost::any(std::make_shared<const cvc::volume>(blobs(app))));
  d.tick(2);
  check(!data_at<cvc::volume>(app, k + ".segment.labels") && val(app, k + ".segment.rows") == "0" &&
            (!segments || segments->getGraphicsChildren().empty()) && zl && !zl->isVisible(),
        "a new volume clears the result, whose labels no longer fit");
}

// A document that opens segmented, and two apps segmenting at once.
void opens_segmented_and_two_apps() {
  cvc::app a1, a2;
  for (cvc::app *a : {&a1, &a2}) {
    a->properties("system.log_verbosity", "0");
    cvc::gl::ariadne::register_cvcgl_extensions(*a);
    cvc::state::instance (*a)("vols.b").data(
        boost::any(std::make_shared<const cvc::volume>(blobs(*a))));
    cvc::state::instance (*a)("vols.p").data(boost::any(blob_seeds()));
  }
  // a2's grid makes no surfaces: the two runs differ. (request: segment runs as the grid takes its
  // volume, during realize: the parameters are the props'.)
  const std::string opens = "      points: { state: /vols.p }\n      request: segment\n";
  realized_doc d1(a1, "one", doc("one", "/vols.b", opens));
  realized_doc d2(a2, "two", doc("two", "/vols.b", opens, ", make_surfaces: false"));
  auto g1 = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d1.sg.getGraphics("g"));
  auto g2 = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d2.sg.getGraphics("g"));
  if (!g1 || !g2) {
    check(false, "both grids exist");
    return;
  }
  // Both run, interleaved frame by frame.
  bool done1 = false, done2 = false;
  for (int i = 0; i < 12000 && !(done1 && done2); ++i) {
    d1.tick();
    d2.tick();
    done1 = done1 || (i > 2 && !d1.pending());
    done2 = done2 || (i > 2 && !d2.pending());
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  d1.tick();
  d2.tick();
  const std::string k1 = g1->stateName() + ".grid", k2 = g2->stateName() + ".grid";
  check(val(a1, k1 + ".segment.rows") == "2" && val(a2, k2 + ".segment.rows") == "2",
        "request: segment opens each document segmented", val(a1, k1 + ".status"));
  check(g1->segmentation().segments() && !g2->segmentation().segments(),
        "each app's grid made what its keys asked (surfaces in one, none in the other)");
  const auto l1 = data_at<cvc::volume>(a1, k1 + ".segment.labels");
  const auto l2 = data_at<cvc::volume>(a2, k2 + ".segment.labels");
  check(l1 && l2 && l1 != l2 && &l1->ctx() == &a1 && &l2->ctx() == &a2,
        "each result is its own app's");
  if (l1 && l2)
    check(static_cast<const cvc::voxels &>(*l1) == static_cast<const cvc::voxels &>(*l2),
          "and the same labels: the runs share nothing that changes them");
}

// While the fronts advance, the planes show the labels so far. A cooperative run (as
// single-threaded wasm runs it, and CVC_SLICED_THREADING=cooperative here) gives the job about
// 4 ms a frame, so a run on a larger volume spans many frames, and some show the labels so far;
// a threaded run on the pool may finish within one chunk, so there it is reported, not required.
void live_labels() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  constexpr int n = 56;
  cvc::state::instance(app)("vols.b").data(
      boost::any(std::make_shared<const cvc::volume>(blobs(app, n))));
  cvc::state::instance(app)("vols.p").data(boost::any(blob_seeds(n)));
  realized_doc d(app, "live",
                 doc("live", "/vols.b",
                     "      points: { state: /vols.p }\n      request: segment\n",
                     ", make_surfaces: false"));
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  if (!g) {
    check(false, "the live grid exists");
    return;
  }
  cvc::gl::VolumeGridSegmentation &seg = g->segmentation();
  bool saw_live = false, live_drawn = false;
  const bool settled = d.settle([&] {
    if (seg.showingLiveLabels()) {
      saw_live = true;
      const auto layer = seg.labelsOverlay(cvc::slice_axis::z);
      live_drawn = live_drawn || (layer && layer->isVisible() && layer->hasTexture());
    }
  });
  check(settled, "the larger run ends");
  const char *mode = std::getenv("CVC_SLICED_THREADING");
  if (mode && std::string(mode) == "cooperative") {
    check(saw_live, "a cooperative run shows the labels so far while it marches");
    check(live_drawn, "drawn on the planes");
  }
  std::printf("  [info] labels so far shown while marching (%d^3, %s): %s\n", n,
              mode ? mode : "automatic", saw_live ? "yes" : "no");
  check(!seg.showingLiveLabels() && seg.shownLabels() &&
            seg.shownLabels() == data_at<cvc::volume>(app, g->stateName() + ".grid.segment.labels"),
        "once done, the planes show the result's labels");
}

// Rendered (offscreen): each class's labels show over the plane in its colour, the plane's grey
// where nothing is labelled. Skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.
void rendered() {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  if (!cvcgl_test::renderAvailable(app))
    return;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  cvc::state::instance(app)("vols.b").data(
      boost::any(std::make_shared<const cvc::volume>(blobs(app))));
  cvc::state::instance(app)("vols.p").data(boost::any(blob_seeds()));
  realized_doc d(app, "seg_draw",
                 doc("segmentation draw", "/vols.b",
                     "      points: { state: /vols.p }\n      request: segment\n"
                     "      display: { crosshairs: false, show_points: false }\n"));
  for (const std::string &w : d.warnings)
    check(false, "the drawn document realizes without warnings", w);
  auto g = std::dynamic_pointer_cast<cvc::gl::VolumeGridNode>(d.sg.getGraphics("g"));
  if (!g) {
    check(false, "the drawn grid exists");
    return;
  }
  const std::string k = g->stateName() + ".grid";
  check(d.settle(), "the drawn document's segmentation ends");
  // The classes' colours: red and blue, as the rows say.
  set(app, k + ".segment.rows.0.color", "1,0,0");
  set(app, k + ".segment.rows.1.color", "0,0,1");
  set(app, k + ".segment.show_surfaces", "0"); // the 3D surfaces would cover the plane
  d.tick();
  cvc::gl::SceneRenderer sr(d.sg, 240, 240, /*offscreen=*/true);
  sr.setBackground(0, 0, 0);
  cvcgl_test::topView(sr, 11.5, 11.5, 13.0);
  d.tick();
  const cvcgl_test::Frame f = cvcgl_test::grab(sr);
  const auto px = [&sr](double x, double y) { return cvcgl_test::toPx(sr, x, y, 12.0); };
  // Points of each blob labelled by its class, off the seeds; and a corner labelled by none.
  check(cvcgl_test::hueNear(f, px(6.0, 14.0), 1, 0), "the left class's labels draw red");
  check(cvcgl_test::hueNear(f, px(18.0, 14.0), 1, 2), "the right class's labels draw blue");
  const unsigned char *corner = f.at(int(px(1.0, 1.0)[0]), int(px(1.0, 1.0)[1]));
  check(!cvcgl_test::isHue(corner, 0) && !cvcgl_test::isHue(corner, 2),
        "an unlabelled voxel shows the plane alone");
}

} // namespace

int main() {
  run();
  opens_segmented_and_two_apps();
  live_labels();
  rendered();
  return cvcgl_test::finish("cvcgl_volume_grid_segment");
}
