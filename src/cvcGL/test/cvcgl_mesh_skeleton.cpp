/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// mesh_lab's `skeleton` request (mesh_nodes.h; docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2,
// phase P2), run headless as cvcgl_fe_interval runs fe_lab: the node's ticks called directly and
// the app's scheduler pumped after them. The mesh is a torus (a lat-long mesh, jittered into
// general position), whose skeleton is its core circle. Checked:
//   - realize: the parameter keys under <node>.mesh_ops.skeleton are there, seeded from the node's
//     `skeleton:` prop (else the defaults), with their properties; view.* on, info.* "-", status
//     idle; the children <id>_strands / <id>_sheets hidden; the view tags (the surface's own entry
//     `surface`, the children's `skeleton`) (T8);
//   - a run: skeleton.busy, mesh_ops.busy and the RealizedScene::pending_work probe hold while it
//     runs and its progress never falls; then the children hold exactly cvc::skeletonize's parts
//     of the mesh's vertices, shown, info.* and status count them, and the data channel
//     <node>.skeleton.{strands, sheets} holds them (T8: the result at the documented keys);
//   - the view keys hide each part;
//   - a stop before its first step ends it "stopped" and keeps the last skeleton; with nothing
//     running a stop says so. Under CVC_SLICED_THREADING=cooperative (the second ctest entry) a
//     stop part-way too, and no frame the run spans takes long (T7: the frame loop keeps drawing);
//   - a value that does not parse, or is out of range, is reported in status and nothing runs; a
//     bad `skeleton:` prop is a realize warning;
//   - a parameter written while it runs: the status says it applies to the next run;
//   - a new mesh (smooth) clears the skeleton;
//   - two apps in one process, each with its own node ticked in turn: each draws the skeleton of
//     its own mesh, the same as skeletonize alone (T6).
// Without Shewchuk's predicates (CVC_WITH_SHEWCHUK off) the request reports skeletonize's error,
// and the rest is skipped. Without CVC_ENABLE_LIBIGL there is no mesh_lab: the test skips (rc 0).
// Plain main, non-zero on failure (NDEBUG is defined in Release, so no assert()).

#include <algorithm>
#include <array>
#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // cvc::pump_exec_frame
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/tet_complex.h> // tet_complex_available
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneNode;
using cvc::ariadne::Value;
using cvc::gl::GeometryNode;
using cvc::gl::SceneGraph;
using cvc::gl::ariadne::RealizedScene;

static int fails = 0;
static void chk(bool ok, const std::string &what) {
  std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok)
    ++fails;
}

namespace {

namespace fs = std::filesystem;
constexpr double kPi = 3.14159265358979323846;

// A torus about z, core radius R and tube radius r: a lat-long mesh of nu x nv quads, each split
// into two triangles, its samples jittered in (u, v) by a seeded generator.
cvc::geometry torus_mesh(int nu, int nv, double R, double r, std::uint32_t seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<double> j(-0.15, 0.15);
  cvc::geometry m;
  for (int i = 0; i < nu; ++i)
    for (int k = 0; k < nv; ++k) {
      const double u = 2 * kPi * (i + j(g)) / nu, v = 2 * kPi * (k + j(g)) / nv;
      m.points().push_back({(R + r * std::cos(v)) * std::cos(u),
                            (R + r * std::cos(v)) * std::sin(u), r * std::sin(v)});
    }
  const auto at = [&](int i, int k) { return static_cast<cvc::index_t>((i % nu) * nv + (k % nv)); };
  for (int i = 0; i < nu; ++i)
    for (int k = 0; k < nv; ++k) {
      m.tris().push_back({at(i, k), at(i + 1, k), at(i + 1, k + 1)});
      m.tris().push_back({at(i, k), at(i + 1, k + 1), at(i, k + 1)});
    }
  return m;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}
void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}
double num(cvc::app &app, const std::string &path) {
  try {
    return cvc::state::instance(app)(path).value<double>();
  } catch (...) {
    return std::nan("");
  }
}
bool starts(const std::string &s, const std::string &prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}
bool has(const std::string &s, const std::string &part) {
  return s.find(part) != std::string::npos;
}

Value scalar(const std::string &s) {
  Value v;
  v.kind = Value::Kind::Scalar;
  v.scalar = s;
  return v;
}
// Props with one entry, `key`: a map of the given pairs.
Value props_with_map(const std::string &key,
                     const std::vector<std::pair<std::string, std::string>> &pairs) {
  Value m;
  m.kind = Value::Kind::Map;
  for (const auto &p : pairs)
    m.entries.emplace_back(p.first, scalar(p.second));
  Value props;
  props.kind = Value::Kind::Map;
  props.entries.emplace_back(key, m);
  return props;
}

// Whether two geometries have the same points, lines, triangles and colours, exactly.
bool same(const cvc::geometry &a, const cvc::geometry &b) {
  return a.const_points() == b.const_points() && a.const_lines() == b.const_lines() &&
         a.const_tris() == b.const_tris() && a.const_colors() == b.const_colors();
}
bool same(const GeometryNode *n, const cvc::geometry &b) {
  return n && n->getGeometry() && same(*n->getGeometry(), b);
}

std::string describe(const cvc::skeleton &s) {
  return std::to_string(s.polylines) + " strands (" +
         std::to_string(s.strands.const_lines().size()) + " lines), " +
         std::to_string(s.sheet_clusters) + " sheets (" +
         std::to_string(s.sheets.const_tris().size()) + " triangles), " +
         std::to_string(s.clusters) + " clusters";
}

Scene one_lab(const fs::path &dir, const std::string &file, Value props) {
  Scene scene;
  SceneNode m;
  m.id = "mesh";
  m.type = "mesh_lab";
  m.source_file = file;
  m.base_dir = dir.string();
  m.props = std::move(props);
  scene.nodes.push_back(m);
  return scene;
}

// One mesh_lab node reading `file` (relative to `dir`) on its own scene graph in `app`, ticked
// headless.
struct Lab {
  cvc::app &app;
  SceneGraph sg;
  std::vector<std::string> warnings;
  RealizedScene r;
  std::shared_ptr<GeometryNode> mesh, strands, sheets;
  std::string ops, sk, data; // <node>.mesh_ops, <node>.mesh_ops.skeleton, <node>.skeleton

  Lab(cvc::app &a, const std::string &prefix, const fs::path &dir, const std::string &file,
      Value props = {})
      : app(a), sg(a, prefix), r(cvc::gl::ariadne::realize_scene(
                                   sg, one_lab(dir, file, std::move(props)), prefix, &warnings)) {
    for (const std::string &w : warnings)
      std::printf("     warning: %s\n", w.c_str());
    mesh = std::dynamic_pointer_cast<GeometryNode>(sg.getGraphics("mesh"));
    if (!mesh)
      return;
    strands = std::dynamic_pointer_cast<GeometryNode>(mesh->findChildByName("mesh_strands"));
    sheets = std::dynamic_pointer_cast<GeometryNode>(mesh->findChildByName("mesh_sheets"));
    ops = mesh->stateName("mesh_ops");
    sk = ops + ".skeleton";
    data = mesh->stateName("skeleton");
  }
  std::string get(const std::string &key) const { return val(app, key); }
  void set(const std::string &key, const std::string &v) { put(app, key, v); }
  void ticks() {
    for (const auto &t : r.custom_ticks)
      t(nullptr);
  }
  void tick() {
    ticks();
    cvc::pump_exec_frame(app.exec_scheduler()); // the skeleton is a sliced task on it
  }
  bool settling() const {
    for (const auto &p : r.pending_work)
      if (p && p())
        return true;
    return false;
  }
  // Tick until the node has nothing left to do; bounded at about two minutes.
  bool settle() {
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (!settling())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }
  // The geometry on the data channel at <node>.skeleton.<part> (null when none).
  std::shared_ptr<const cvc::geometry> part(const std::string &name) const {
    const boost::any a = cvc::state::instance(app)(data + "." + name).data();
    if (const cvc::geometry *g = boost::any_cast<cvc::geometry>(&a))
      return std::make_shared<const cvc::geometry>(*g);
    return nullptr;
  }
};

// A TaggedNode entry of the realized scene, by id and subtree flag (null when there is none).
const RealizedScene::TaggedNode *entry(const RealizedScene &r, const std::string &id,
                                       bool subtree) {
  for (const RealizedScene::TaggedNode &t : r.nodes)
    if (t.id == id && t.subtree == subtree)
      return &t;
  return nullptr;
}

} // namespace

int main() {
  cvc::app app;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  if (!cvc::gl::ariadne::has_scene_node_type(app, "mesh_lab")) {
    std::printf("SKIP: no mesh_lab node type (libcvc built without CVC_ENABLE_LIBIGL)\n");
    return 0;
  }
  const char *threading = std::getenv("CVC_SLICED_THREADING");
  const bool cooperative = threading && std::string(threading) == "cooperative";
  std::printf("== mesh_lab skeleton (%s) ==\n", cooperative ? "cooperative" : "automatic");

  std::error_code ec;
  const fs::path dir = fs::temp_directory_path(ec) /
                       (cooperative ? "cvcgl_mesh_skeleton_coop" : "cvcgl_mesh_skeleton");
  fs::create_directories(dir, ec);
  cvc::write_geometry(app, torus_mesh(48, 16, 4.0, 1.0, 7), (dir / "torus.off").string());
  cvc::write_geometry(app, torus_mesh(40, 14, 3.0, 0.8, 11), (dir / "torus2.off").string());
  // What the node reads, so the reference skeleton is of the same points (the writer rounds).
  const cvc::geometry input = cvc::read_geometry(app, (dir / "torus.off").string());
  const cvc::geometry input2 = cvc::read_geometry(app, (dir / "torus2.off").string());
  const bool exact = cvc::tet_complex_available();

  // ── realize ──────────────────────────────────────────────────────────────────────────────────
  std::printf("== realize: keys, children, tags ==\n");
  Lab lab(app, "skl", dir, "torus.off",
          props_with_map("skeleton", {{"sheet_count", "3"}, {"medial_ratio", "50"}}));
  chk(lab.mesh && lab.strands && lab.sheets, "mesh_lab realized its node and its two children");
  if (!lab.mesh || !lab.strands || !lab.sheets)
    return 1;
  chk(lab.warnings.empty(), "no realize warnings");
  const std::string &sk = lab.sk, &ops = lab.ops;
  chk(lab.get(sk + ".sheet_count") == "3" && num(app, sk + ".medial_ratio") == 50.0,
      "skeleton: { sheet_count, medial_ratio } seeds those keys");
  chk(std::fabs(num(app, sk + ".cocone_phi") - kPi / 8) < 1e-12 && lab.get(sk + ".robust") == "0",
      "a field the prop leaves out is seeded with its default (cocone_phi pi/8, robust 0)");
  {
    cvc::state &phi = cvc::state::instance(app)(sk + ".cocone_phi");
    cvc::state &count = cvc::state::instance(app)(sk + ".sheet_count");
    chk(phi.property("unit") == "rad" && !phi.property("doc").empty() &&
            !phi.property("max").empty() && count.property("default") == "2",
        "the parameter keys carry their properties (unit, doc, max, default)");
  }
  chk(lab.get(sk + ".busy") == "0" && lab.get(sk + ".status") == "idle" &&
          lab.get(sk + ".info.strands") == "-" && lab.get(sk + ".info.sheets") == "-",
      "idle at realize: busy 0, status idle, info.* '-'");
  chk(lab.get(sk + ".view.strands") == "1" && lab.get(sk + ".view.sheets") == "1",
      "view.strands and view.sheets are seeded on");
  lab.tick();
  chk(!lab.strands->isVisible() && !lab.sheets->isVisible(),
      "the children hide until a skeleton is drawn");
  chk(lab.strands->getRenderMode() == cvc::gl::GeometryRenderMode::LINES,
      "the strands child draws lines");
  {
    const RealizedScene::TaggedNode *own = entry(lab.r, "mesh", false),
                                    *node = entry(lab.r, "mesh", true),
                                    *st = entry(lab.r, "mesh_strands", true),
                                    *sh = entry(lab.r, "mesh_sheets", true);
    const std::vector<std::string> surface{"surface"}, skeleton{"skeleton"};
    chk(own && own->tags == surface && own->node.lock() == lab.mesh,
        "view tags: the node's own surface is tagged surface, its props alone (subtree false)");
    chk(node && node->tags.empty(), "view tags: the node's own (untagged) entry holds its subtree");
    chk(st && st->tags == skeleton && st->node.lock() == lab.strands && sh &&
            sh->tags == skeleton && sh->node.lock() == lab.sheets,
        "view tags: the children are tagged skeleton");
  }

  // ── a run ────────────────────────────────────────────────────────────────────────────────────
  std::printf("== skeleton: a run ==\n");
  cvc::skeleton_params params;
  params.sheet_count = 3;
  params.medial_ratio = 50.0;
  lab.set(ops + ".request", "skeleton");
  if (!exact) {
    chk(lab.settle(), "the request settles");
    chk(starts(lab.get(sk + ".status"), "error: ") && lab.get(sk + ".busy") == "0",
        "without Shewchuk's predicates the request reports skeletonize's error: " +
            lab.get(sk + ".status"));
    std::printf("SKIP: the rest needs Shewchuk's predicates (CVC_WITH_SHEWCHUK)\n");
    return fails ? 1 : 0;
  }
  const cvc::skeleton expected = cvc::skeletonize(input, params);
  std::printf("     skeletonize: %s, %zu warnings\n", describe(expected).c_str(),
              expected.warnings.size());
  lab.tick(); // takes the request and starts the task
  const bool busy = lab.get(sk + ".busy") == "1" && lab.get(ops + ".busy") == "1" &&
                    lab.get(sk + ".stoppable") == "1" && lab.settling();
  int last = -1, frames = 0;
  bool monotone = true;
  double worst_ms = 0.0;
  while (lab.get(sk + ".busy") == "1" && frames < 60000) {
    const auto t0 = std::chrono::steady_clock::now();
    lab.tick();
    worst_ms = std::max(
        worst_ms,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    const int p = static_cast<int>(num(app, sk + ".progress"));
    monotone = monotone && p >= last;
    last = p;
    ++frames;
    if (!cooperative)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::printf("     %d frames while it ran, the slowest %.1f ms\n", frames, worst_ms);
  chk(busy, "while it runs: skeleton.busy, stoppable and mesh_ops.busy are 1, and the node is "
            "settling");
  chk(monotone, "its progress never falls");
  if (cooperative) {
    chk(frames >= 2, "cooperative: the run spans frames (" + std::to_string(frames) + ")");
    chk(worst_ms < 200.0, "cooperative: no frame while it runs takes long (the slowest " +
                              std::to_string(static_cast<int>(worst_ms)) + " ms)");
  }
  chk(lab.settle(), "the run settles");
  chk(lab.get(sk + ".busy") == "0" && lab.get(ops + ".busy") == "0" &&
          lab.get(sk + ".progress") == "100" && lab.get(sk + ".phase").empty() &&
          lab.get(sk + ".stoppable") == "0",
      "done: busy 0, progress 100, phase '', stoppable 0");
  chk(same(lab.strands.get(), expected.strands) && same(lab.sheets.get(), expected.sheets),
      "the children hold skeletonize's strands and sheets of the mesh's vertices, exactly");
  chk(expected.strands.const_lines().size() > 0 && expected.polylines >= 1,
      "the torus has a skeleton (" + describe(expected) + ")");
  chk(lab.strands->isVisible() && lab.sheets->isVisible(), "the children show it");
  chk(num(app, sk + ".info.points") == double(input.const_points().size()) &&
          num(app, sk + ".info.strands") == expected.polylines &&
          num(app, sk + ".info.sheets") == expected.sheet_clusters &&
          num(app, sk + ".info.clusters") == expected.clusters &&
          num(app, sk + ".info.lines") == double(expected.strands.const_lines().size()) &&
          num(app, sk + ".info.triangles") == double(expected.sheets.const_tris().size()),
      "info.* count it");
  const std::string done = "done: " + std::to_string(expected.polylines) + " strands, " +
                           std::to_string(expected.sheet_clusters) + " sheets (" +
                           std::to_string(expected.clusters) + " clusters) from " +
                           std::to_string(input.const_points().size()) + " points";
  chk(starts(lab.get(sk + ".status"), done), "status: '" + lab.get(sk + ".status") + "'");
  chk(lab.get(sk + ".warnings").empty() == expected.warnings.empty(),
      "warnings: one line per warning");
  {
    const auto s = lab.part("strands"), h = lab.part("sheets");
    chk(s && h && same(*s, expected.strands) && same(*h, expected.sheets),
        "the data channel holds them at <node>.skeleton.strands and .sheets");
  }
  chk(num(app, sk + ".stats.work_ms") >= 0.0, "stats.work_ms is published");

  // ── the view keys ────────────────────────────────────────────────────────────────────────────
  std::printf("== the view keys ==\n");
  lab.set(sk + ".view.sheets", "0");
  lab.tick();
  chk(lab.strands->isVisible() && !lab.sheets->isVisible(), "view.sheets 0 hides the sheets");
  lab.set(sk + ".view.sheets", "1");
  lab.set(sk + ".view.strands", "0");
  lab.tick();
  chk(!lab.strands->isVisible() && lab.sheets->isVisible(), "view.strands 0 hides the strands");
  lab.set(sk + ".view.strands", "1");
  lab.tick();
  chk(lab.strands->isVisible() && lab.sheets->isVisible(), "both on again");

  // ── stop ─────────────────────────────────────────────────────────────────────────────────────
  std::printf("== stop ==\n");
  lab.set(ops + ".request", "skeleton");
  lab.ticks(); // started, not stepped: the scheduler is not pumped
  chk(lab.get(sk + ".busy") == "1", "a second run starts");
  lab.set(ops + ".request", "stop");
  lab.ticks(); // stopped before its first step
  chk(lab.get(sk + ".status") == "stopping..." && lab.get(ops + ".request").empty(),
      "a stop is taken while it runs: status 'stopping...'");
  chk(lab.settle(), "the stopped run settles");
  chk(lab.get(sk + ".status") == "stopped" && lab.get(sk + ".busy") == "0" &&
          lab.get(ops + ".busy") == "0",
      "it ends 'stopped', busy 0");
  chk(same(lab.strands.get(), expected.strands) && lab.strands->isVisible() &&
          num(app, sk + ".info.strands") == expected.polylines,
      "a stop keeps the last skeleton");
  lab.set(ops + ".request", "stop");
  const bool idle_stop = lab.settle();
  chk(idle_stop && lab.get(ops + ".status") == "nothing to stop",
      "with nothing running, a stop says so");
  if (cooperative) {
    lab.set(ops + ".request", "skeleton");
    lab.tick(); // takes it: busy 1, progress 0
    int n = 0;
    while (num(app, sk + ".progress") <= 0.0 && lab.get(sk + ".busy") == "1" && n++ < 20000)
      lab.tick();
    const bool partway = lab.get(sk + ".busy") == "1" && num(app, sk + ".progress") > 0.0;
    const std::string at = lab.get(sk + ".progress") + "%, " + lab.get(sk + ".phase");
    lab.set(ops + ".request", "stop");
    chk(lab.settle(), "cooperative: a run stopped part-way settles");
    chk(partway && lab.get(sk + ".status") == "stopped" &&
            same(lab.strands.get(), expected.strands),
        "cooperative: a stop part-way (at " + at + ") ends it and keeps the last skeleton");
  }

  // ── bad values ───────────────────────────────────────────────────────────────────────────────
  std::printf("== bad values ==\n");
  lab.set(sk + ".sheet_count", "abc");
  lab.set(ops + ".request", "skeleton");
  chk(lab.settle(), "a run with a bad value settles");
  chk(starts(lab.get(sk + ".status"), "error: ") && has(lab.get(sk + ".status"), "sheet_count") &&
          lab.get(sk + ".busy") == "0" && lab.get(ops + ".busy") == "0",
      "a value that does not parse is reported, and nothing runs: " + lab.get(sk + ".status"));
  lab.set(sk + ".sheet_count", "3");
  lab.set(sk + ".threshold", "5");
  lab.set(ops + ".request", "skeleton");
  const bool ranged = lab.settle();
  chk(ranged && starts(lab.get(sk + ".status"), "error: ") &&
          has(lab.get(sk + ".status"), "threshold"),
      "a value out of range is reported: " + lab.get(sk + ".status"));
  lab.set(sk + ".threshold", "0.1");
  chk(same(lab.strands.get(), expected.strands), "the last skeleton stays");
  {
    Lab bad(app, "skb", dir, "torus.off", props_with_map("skeleton", {{"sheet_count", "many"}}));
    bool warned = false;
    for (const std::string &w : bad.warnings)
      warned = warned || (has(w, "skeleton:") && has(w, "sheet_count"));
    chk(warned && bad.get(bad.sk + ".sheet_count") == "2",
        "a bad skeleton: prop is a realize warning, and the default stands");
  }

  // ── a parameter written while it runs ────────────────────────────────────────────────────────
  std::printf("== a parameter written while it runs ==\n");
  lab.set(ops + ".request", "skeleton");
  lab.ticks(); // started with medial_ratio 50
  lab.set(sk + ".medial_ratio", "60");
  chk(lab.settle(), "the run settles");
  chk(has(lab.get(sk + ".status"), "the parameters changed while it ran") &&
          same(lab.strands.get(), expected.strands),
      "it ran with the parameters it started with, and its status says they changed: " +
          lab.get(sk + ".status"));
  lab.set(sk + ".medial_ratio", "50");

  // ── a new mesh ───────────────────────────────────────────────────────────────────────────────
  std::printf("== a new mesh ==\n");
  lab.set(ops + ".request", "smooth");
  chk(lab.settle(), "a smooth settles");
  chk(!lab.strands->isVisible() && !lab.sheets->isVisible() &&
          lab.get(sk + ".info.strands") == "-" && !lab.part("strands") &&
          starts(lab.get(sk + ".status"), "cleared"),
      "a new mesh clears the skeleton: children hidden, info '-', no data, status '" +
          lab.get(sk + ".status") + "'");

  // ── two apps ─────────────────────────────────────────────────────────────────────────────────
  std::printf("== two apps ==\n");
  {
    cvc::app one, two;
    cvc::gl::ariadne::register_cvcgl_extensions(one);
    cvc::gl::ariadne::register_cvcgl_extensions(two);
    Lab a(one, "one", dir, "torus.off"), b(two, "two", dir, "torus2.off");
    chk(a.mesh && b.mesh, "each app realized its own mesh_lab");
    if (a.mesh && b.mesh) {
      a.set(a.ops + ".request", "skeleton");
      b.set(b.ops + ".request", "skeleton");
      bool settled = false;
      for (int i = 0; i < 12000 && !settled; ++i) {
        a.tick();
        b.tick();
        settled = !a.settling() && !b.settling();
        if (!settled)
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      chk(settled, "both settle");
      const cvc::skeleton ea = cvc::skeletonize(input), eb = cvc::skeletonize(input2);
      chk(same(a.strands.get(), ea.strands) && same(a.sheets.get(), ea.sheets) &&
              same(b.strands.get(), eb.strands) && same(b.sheets.get(), eb.sheets),
          "each app's node drew the skeleton of its own mesh, as skeletonize alone (" +
              describe(ea) + " | " + describe(eb) + ")");
      chk(starts(a.get(a.sk + ".status"), "done: ") && starts(b.get(b.sk + ".status"), "done: "),
          "each app's keys are its own");
    }
  }

  std::printf("%s (%d failed)\n", fails ? "FAIL" : "PASS", fails);
  return fails ? 1 : 0;
}
