/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvcgl_contour_lab — the `contour_lab` scene node (contour_lab_nodes.h;
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3 and §7, P4), headless:
//   - a document's contour_lab reads a Reconstruct series at realize: its components' rows
//     (name, colour, visible, tile; the document's components: entry over the file's colour),
//     info.*, a polyline node per component under <id>_contours (tag contours), the stack on the
//     data channel at tiler.contours, and every parameter key seeded with its properties;
//   - request "tile" runs the tiler as a sliced task: busy while it runs, then a surface node per
//     component under <id>_tiles (tag tiles), the surfaces at tiler.surfaces.<component>, equal
//     to cvc::tile_contours on the same stack, the counts and the status;
//   - an unticked component is left out of the next tile, and its surface is gone;
//   - "stop" before the job's first step ends it cancelled, keeping the surfaces the node had;
//   - a row's colour and visibility, view.tiles and view.opacity reach the nodes;
//   - source: { node } reads another node's contours, and source: { state } waits for a stack on
//     a key's data channel and follows it when it changes;
//   - bad input warns: no source, a missing file, an unknown parameter, a node naming itself;
//   - RealizedScene::pending_work says busy exactly while a request waits or a job runs;
//   - two apps: one without cvcGL's extensions has no contour_lab type, and two apps tiling the
//     same document at once get the same surfaces under their own keys.
//   - cooperative mode (CVC_SLICED_THREADING=cooperative, as single-threaded wasm runs): a tile
//     spans many frames, each frame returns, and its progress rises.

#include "streaming_test_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h>
#include <cvc/geometry/contours.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/tiler.h>
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using cvcgl_test::check;

namespace {

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

void set(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

boost::any data(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).data();
}

// A closed circle of n vertices about (cx, cy), its first vertex at angle `phase`.
std::vector<double> circle(double cx, double cy, double r, int n = 16, double phase = 0.0) {
  std::vector<double> xy;
  for (int i = 0; i < n; ++i) {
    const double t = phase + 2.0 * 3.14159265358979323846 * i / n;
    xy.push_back(cx + r * std::cos(t));
    xy.push_back(cy + r * std::sin(t));
  }
  return xy;
}

// "tube": a circle of radius about 2 on slices 0-4; "rod": one of radius about 0.5 beside it on
// 1-3. Each slice's circle drifts, grows a little and turns, as traced sections do (circles
// stacked exactly over one another leave the tiler only untiled regions).
cvc::contour_stack two_components() {
  cvc::contour_stack s;
  s.z_spacing = 1.0;
  for (int k = 0; k <= 4; ++k)
    s.add(k, "tube", circle(0.25 * k, 0.0, 2.0 + 0.125 * (k % 3), 16, 0.3 * k));
  for (int k = 1; k <= 3; ++k)
    s.add(k, "rod", circle(6.0 + 0.0625 * k, 0.0, 0.5 + 0.0625 * (k % 2), 12, 0.4 * k));
  cvc::contour_component tube{"tube", {0.2, 0.6, 1.0}, true};
  cvc::contour_component rod{"rod", {0.9, 0.9, 0.1}, true};
  s.set_component(tube);
  s.set_component(rod);
  return s;
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
  // The node ticks alone (no scheduler pump: a job launched here has not stepped yet).
  void tick_only() {
    for (const auto &t : rs.custom_ticks)
      t(nullptr);
  }
  void tick() {
    tick_only();
    cvc::pump_exec_frame(app.exec_scheduler());
  }
  bool pending() const {
    for (const auto &p : rs.pending_work)
      if (p && p())
        return true;
    return false;
  }
  // Tick until no node has work in flight; bounded at about two minutes.
  bool settle() {
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (!pending())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }
};

std::shared_ptr<cvc::gl::GraphicsNode> child(const std::shared_ptr<cvc::gl::GraphicsNode> &g,
                                             const std::string &name) {
  if (!g)
    return nullptr;
  for (const auto &c : g->getGraphicsChildren())
    if (c->getName() == name)
      return c;
  return nullptr;
}

std::shared_ptr<cvc::gl::GeometryNode> geom(const std::shared_ptr<cvc::gl::GraphicsNode> &g,
                                            const std::string &name) {
  return std::dynamic_pointer_cast<cvc::gl::GeometryNode>(child(g, name));
}

bool has_tag(const cvc::gl::ariadne::RealizedScene &rs, const std::string &id,
             const std::string &tag) {
  for (const auto &t : rs.nodes)
    if (t.id == id)
      for (const std::string &x : t.tags)
        if (x == tag)
          return true;
  return false;
}

std::string lab_doc(const std::string &file, const std::string &extra = "") {
  return "meta:\n  name: contour lab test\n  min_libcvc: 4.0.0\nscene:\n  nodes:\n"
         "    - node: lab\n      type: contour_lab\n      source: { file: \"" +
         file +
         "\" }\n      components:\n        rod: { color: [1, 0, 0] }\n"
         "      tiler: { curation_delta: 0.001 }\n" +
         extra;
}

void headless(const fs::path &dir) {
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  check(cvc::gl::ariadne::has_scene_node_type(app, "contour_lab"),
        "register_cvcgl_extensions installs contour_lab");
  const std::string file = (dir / "two.ser").generic_string();
  cvc::write_contours(app, two_components(), file);
  const cvc::contour_stack stack = cvc::read_contours(app, file);

  realized_doc d(app, "lab", lab_doc(file));
  for (const std::string &w : d.warnings)
    check(false, "the document realizes without warnings", w);
  auto lab = d.sg.getGraphics("lab");
  check(lab != nullptr, "the contour_lab node is realized");
  if (!lab)
    return;
  const std::string k = lab->stateName("tiler");

  // At realize: rows, info, polylines, the stack's data, the parameter keys.
  check(val(app, k + ".components") == "2", "two component rows", val(app, k + ".components"));
  const int tube = val(app, k + ".components.0.name") == "tube" ? 0 : 1, rod = 1 - tube;
  const std::string T = k + ".components." + std::to_string(tube),
                    R = k + ".components." + std::to_string(rod);
  check(val(app, T + ".name") == "tube" && val(app, R + ".name") == "rod", "rows name them");
  check(val(app, R + ".color") == "1,0,0", "the document's components: colour wins over the file's",
        val(app, R + ".color"));
  check(val(app, T + ".color") == "0.2,0.6,1", "the file's colour otherwise",
        val(app, T + ".color"));
  check(val(app, T + ".visible") == "1" && val(app, T + ".tile") == "1", "shown and ticked");
  check(val(app, k + ".info.slices") == "5" && val(app, k + ".info.first") == "0" &&
            val(app, k + ".info.last") == "4",
        "info: slices 0 - 4",
        val(app, k + ".info.slices") + " " + val(app, k + ".info.first") + " " +
            val(app, k + ".info.last"));
  check(val(app, k + ".info.contours") == "8", "info.contours", val(app, k + ".info.contours"));
  check(val(app, k + ".busy") == "0" && val(app, k + ".progress") == "0", "idle at realize");
  {
    const boost::any a = data(app, k + ".contours");
    const auto *p = boost::any_cast<std::shared_ptr<const cvc::contour_stack>>(&a);
    check(p && *p && **p == stack, "tiler.contours holds the stack read");
  }
  check(val(app, k + ".curation_delta") == "0.001", "tiler: seeds a parameter",
        val(app, k + ".curation_delta"));
  check(val(app, k + ".collinear_epsilon") == "0", "the rest take their defaults",
        val(app, k + ".collinear_epsilon"));
  check(!cvc::state::instance(app)(k + ".curation_delta").property("doc").empty() &&
            cvc::state::instance(app)(k + ".z_first").property("min") == "-1",
        "and carry their properties");
  check(val(app, k + ".read.min_points") == "3", "read.* seeded", val(app, k + ".read.min_points"));
  auto cg = child(lab, "lab_contours"), tg = child(lab, "lab_tiles");
  check(cg && tg, "the contours and tiles groups");
  check(has_tag(d.rs, "lab_contours", "contours") && has_tag(d.rs, "lab_tiles", "tiles"),
        "tagged contours and tiles");
  check(cg && cg->getGraphicsChildren().size() == 2, "a polyline node per component");
  if (auto tn = geom(cg, "tube")) {
    check(tn->getRenderMode() == cvc::gl::GeometryRenderMode::LINES, "drawn as lines");
    check(tn->getGeometry() && tn->getGeometry()->num_lines() == 5 * 16,
          "every edge of the tube's five circles");
  } else {
    check(false, "a polyline node named tube");
  }
  check(!d.pending(), "nothing pending at realize");

  // Tile.
  set(app, k + ".request", "tile");
  check(d.pending(), "a waiting request is pending work");
  d.tick_only();
  check(val(app, k + ".busy") == "1" && val(app, k + ".stoppable") == "1", "busy while it tiles");
  check(d.settle(), "the tile finishes");
  check(val(app, k + ".busy") == "0" && val(app, k + ".progress") == "100" &&
            val(app, k + ".phase").empty(),
        "idle again: progress 100, no phase");
  check(val(app, k + ".status").rfind("tiled 2 components", 0) == 0, "status says what it tiled",
        val(app, k + ".status"));
  check(val(app, k + ".info.surfaces") == "2", "two surfaces", val(app, k + ".info.surfaces"));
  const int nwarn = std::atoi(val(app, k + ".warnings").c_str());
  for (int i = 0; i < nwarn; ++i)
    std::printf("     tiler warning %d: %s\n", i,
                val(app, k + ".warnings." + std::to_string(i)).c_str());
  cvc::tile_params p;
  p.curation_delta = 0.001;
  const cvc::tile_result direct = cvc::tile_contours(stack, p);
  for (const char *name : {"tube", "rod"}) {
    const boost::any a = data(app, k + ".surfaces." + name);
    const auto *g = boost::any_cast<cvc::geometry>(&a);
    const auto it = direct.components.find(name);
    check(g && it != direct.components.end() && g->num_tris() > 0 &&
              g->num_tris() == it->second.num_tris() &&
              g->const_points() == it->second.const_points(),
          std::string("surfaces.") + name + " is tile_contours' surface");
    const int row = std::string(name) == "tube" ? tube : rod;
    check(val(app, k + ".components." + std::to_string(row) + ".triangles") ==
              std::to_string(g ? g->num_tris() : 0),
          std::string(name) + "'s row counts its triangles");
  }
  check(tg && tg->getGraphicsChildren().size() == 2, "a surface node per component");
  if (auto rn = geom(tg, "rod"))
    check(val(app, rn->stateName("color_r")) == "1" && val(app, rn->stateName("color_g")) == "0",
          "the rod's surface in its row's colour");

  // Untick the rod: the next tile leaves it out, and its surface goes.
  set(app, R + ".tile", "0");
  set(app, k + ".request", "tile");
  check(d.settle(), "the second tile finishes");
  check(val(app, k + ".info.surfaces") == "1" && data(app, k + ".surfaces.rod").empty() &&
            !data(app, k + ".surfaces.tube").empty() && val(app, R + ".triangles") == "-",
        "an unticked component is left out");
  check(tg && tg->getGraphicsChildren().size() == 1, "one surface node");
  set(app, R + ".tile", "1");

  // Stop before the job's first step: cancelled, the tube's surface kept.
  set(app, k + ".request", "tile stop");
  d.tick_only(); // launches the job; the scheduler has not stepped it
  d.tick_only(); // takes the stop
  check(d.settle(), "the stopped job ends");
  check(val(app, k + ".status") == "cancelled" && val(app, k + ".busy") == "0",
        "a stop ends it cancelled", val(app, k + ".status"));
  check(val(app, k + ".info.surfaces") == "1" && !data(app, k + ".surfaces.tube").empty(),
        "and the node keeps the surfaces it had");

  // Rows and view keys reach the nodes.
  set(app, T + ".color", "0,0,1");
  set(app, R + ".visible", "0");
  set(app, k + ".view.opacity", "0.5");
  d.tick();
  if (auto tn = geom(cg, "tube"))
    check(val(app, tn->stateName("color_b")) == "1" && val(app, tn->stateName("color_r")) == "0",
          "a row's colour recolours its polylines");
  if (auto rn = geom(cg, "rod"))
    check(!rn->isVisible(), "a hidden row hides its polylines");
  if (auto ts = geom(tg, "tube")) {
    check(val(app, ts->stateName("color_b")) == "1", "and its surface");
    check(val(app, ts->stateName("opacity")) == "0.5", "view.opacity reaches the surfaces");
    set(app, k + ".view.tiles", "0");
    d.tick();
    check(!ts->isVisible(), "view.tiles 0 hides the surfaces");
  }

  // Bad input.
  {
    realized_doc b(app, "lab_bad",
                   "meta:\n  name: bad\n  min_libcvc: 4.0.0\nscene:\n  nodes:\n"
                   "    - { node: a, type: contour_lab }\n"
                   "    - { node: b, type: contour_lab, source: { file: \"" +
                       (dir / "nope.ser").generic_string() +
                       "\" } }\n"
                       "    - { node: c, type: contour_lab, source: { file: \"" +
                       file +
                       "\" }, tiler: { nope: 1 } }\n"
                       "    - { node: e, type: contour_lab, source: { node: e } }\n");
    bool no_source = false, missing = false, unknown = false, self = false;
    for (const std::string &w : b.warnings) {
      no_source = no_source || w.find("'a' (contour_lab): no source") != std::string::npos;
      missing = missing || w.find("nope.ser") != std::string::npos;
      unknown = unknown || w.find("no parameter 'nope'") != std::string::npos;
      self = self || w.find("is this node") != std::string::npos;
    }
    check(no_source && missing && unknown && self,
          "no source, a missing file, an unknown parameter and a node naming itself each warn");
    check(val(app, b.sg.getGraphics("c")->stateName("tiler.components")) == "2",
          "a node with a bad parameter still reads its contours");
  }

  // Another node's contours, and a state key's.
  {
    realized_doc s(app, "lab_src",
                   lab_doc(file, "    - { node: copy, type: contour_lab, source: { node: lab } }\n"
                                 "    - { node: later, type: contour_lab, source: { state: "
                                 "demo.stack } }\n"));
    for (const std::string &w : s.warnings)
      check(false, "the sourced document realizes without warnings", w);
    const std::string ck = s.sg.getGraphics("copy")->stateName("tiler");
    const std::string lk = s.sg.getGraphics("later")->stateName("tiler");
    check(val(app, ck + ".components") == "2" && val(app, ck + ".info.contours") == "8",
          "source: { node } reads that node's tiler.contours");
    check(val(app, lk + ".components") == "0" &&
              val(app, lk + ".status").find("waiting for contours") != std::string::npos,
          "source: { state } waits for its stack", val(app, lk + ".status"));
    cvc::contour_stack one;
    one.add(0, "dot", circle(0, 0, 1));
    one.add(1, "dot", circle(0, 0, 1));
    cvc::state::instance(app)(s.sg.getStatePrefix().empty() ? std::string("demo.stack")
                                                            : s.sg.getStatePrefix() + ".demo.stack")
        .data(boost::any(std::make_shared<const cvc::contour_stack>(one)));
    s.tick();
    check(val(app, lk + ".components") == "1" && val(app, lk + ".components.0.name") == "dot",
          "and loads it once it lands", val(app, lk + ".status"));
    one.add(2, "dot", circle(0, 0, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(2)); // a later lastMod
    cvc::state::instance(app)(s.sg.getStatePrefix().empty() ? std::string("demo.stack")
                                                            : s.sg.getStatePrefix() + ".demo.stack")
        .data(boost::any(std::make_shared<const cvc::contour_stack>(one)));
    s.tick();
    check(val(app, lk + ".info.slices") == "3", "and again when it changes",
          val(app, lk + ".info.slices"));
  }

  // Two apps: the type is the app's that registered it, and two apps tile apart.
  cvc::app other;
  check(!cvc::gl::ariadne::has_scene_node_type(other, "contour_lab"),
        "another app has no contour_lab type until it registers cvcGL's extensions");
  {
    cvc::app a1, a2;
    a1.properties("system.log_verbosity", "0");
    a2.properties("system.log_verbosity", "0");
    cvc::gl::ariadne::register_cvcgl_extensions(a1);
    cvc::gl::ariadne::register_cvcgl_extensions(a2);
    realized_doc d1(a1, "lab", lab_doc(file, "      auto_tile: true\n"));
    realized_doc d2(a2, "lab", lab_doc(file, "      auto_tile: true\n"));
    bool ok = true;
    for (int i = 0; i < 12000 && (d1.pending() || d2.pending()); ++i) {
      d1.tick();
      d2.tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ok = !d1.pending() && !d2.pending();
    check(ok, "two apps tile the same document at once (auto_tile)");
    const std::string k1 = d1.sg.getGraphics("lab")->stateName("tiler");
    const std::string k2 = d2.sg.getGraphics("lab")->stateName("tiler");
    const boost::any s1 = data(a1, k1 + ".surfaces.tube"), s2 = data(a2, k2 + ".surfaces.tube");
    const auto *g1 = boost::any_cast<cvc::geometry>(&s1);
    const auto *g2 = boost::any_cast<cvc::geometry>(&s2);
    check(g1 && g2 && g1->const_points() == g2->const_points() &&
              g1->num_tris() == g2->num_tris() && g1->num_tris() > 0,
          "and get the same surfaces, each under its own app's keys");
    check(val(app, k + ".info.surfaces") == "1", "while the first app's node keeps its own");
  }
}

void set_env(const char *name, const char *value) {
#ifdef _WIN32
  _putenv_s(name, value ? value : "");
#else
  if (value)
    setenv(name, value, 1);
  else
    unsetenv(name);
#endif
}

// Cooperative mode, as single-threaded wasm runs (CVC_SLICED_THREADING=cooperative): a tile of a
// bigger stack spans many frames, every frame returns, and the progress keys rise while it runs.
// The slowest frame is printed, not bounded: a Debug or sanitizer build is slow everywhere, and
// the tiler's curating and augmenting units are a slice or a pair of slices each (tens of ms here
// in Release, in the first frames), far coarser than §1.9's microsecond.
void cooperative(const fs::path &dir) {
  cvc::contour_stack s;
  s.z_spacing = 0.5;
  for (int k = 0; k < 40; ++k)
    for (int c = 0; c < 3; ++c)
      s.add(k, "c" + std::to_string(c),
            circle(8.0 * c + 0.05 * k, 0.03 * k, 2.0 + 0.25 * std::sin(0.3 * k + c), 64, 0.2 * k));
  cvc::app app;
  app.properties("system.log_verbosity", "0");
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  const std::string file = (dir / "coop.cvcc").generic_string();
  cvc::write_contours(app, s, file);
  set_env("CVC_SLICED_THREADING", "cooperative");
  realized_doc d(app, "coop",
                 "meta:\n  name: coop\n  min_libcvc: 4.0.0\nscene:\n  nodes:\n"
                 "    - { node: lab, type: contour_lab, auto_tile: true, source: { file: \"" +
                     file + "\" } }\n");
  const std::string k = d.sg.getGraphics("lab")->stateName("tiler");
  int frames = 0, rising = 0, last = -1;
  double slowest = 0.0;
  std::string slowest_phase;
  while (d.pending() && frames < 200000) {
    const std::string before = val(app, k + ".phase");
    const auto t0 = std::chrono::steady_clock::now();
    d.tick();
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (ms > slowest) {
      slowest = ms;
      slowest_phase = before;
    }
    ++frames;
    const int p = std::atoi(val(app, k + ".progress").c_str());
    rising += p > last && last >= 0 ? 1 : 0;
    last = p;
  }
  set_env("CVC_SLICED_THREADING", nullptr);
  std::printf("     cooperative tile: %d frames, the slowest %.1f ms (after '%s')\n", frames,
              slowest, slowest_phase.c_str());
  check(!d.pending() && val(app, k + ".info.surfaces") == "3", "the cooperative tile finishes",
        val(app, k + ".status"));
  check(frames > 10, "over many frames (sliced on the frame loop)", std::to_string(frames));
  check(rising > 2, "its progress rising as it goes", std::to_string(rising));
}

} // namespace

int main() {
  const fs::path dir =
      fs::temp_directory_path() /
      ("cvcgl_contour_lab_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  headless(dir);
  cooperative(dir);
  std::error_code ec;
  fs::remove_all(dir, ec);
  return cvcgl_test::finish("cvcgl_contour_lab");
}
