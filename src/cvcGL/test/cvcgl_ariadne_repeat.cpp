// A scene node's `repeat:` (docs/ARIADNE.md §5.4): a template drawn as copies, as many as its
// count says, the count a number, a state key or a read-lane expression.
//
// Part 1, realize_scene and tick_scene (no window):
//   - a key's count draws that many copies, named and tagged by {i}, each with its children, its
//     transform and its own visible: bind; a change of the count adds or removes copies at the next
//     tick_scene, and RealizedScene's created, nodes and visibility lists follow;
//   - a copy made when the count grows starts at the pose its node's keys hold;
//   - a custom node type's per-frame ticks come and go with its copies (and what they hold is
//     released), while a later node's tick keeps running;
//   - copies under a group are its children; an expression is counted through
//     RealizeOptions::count (a Runtime's eval_count), and without one it draws none and says so;
//   - a repeat: inside a repeated node counts its own copies in each, and goes with its outer copy.
// Part 2, a document in AriRuntime: a `repeat: (state-get "n")` node follows the document's state.
// Part 2 skips (rc 0) where there is no ImGui overlay, unless CVC_REQUIRE_RENDER=1.
#include "streaming_test_util.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h>
#include <cvc/core/app.h>
#include <cvc/gl/CameraController.h>
#include <cvc/gl/GraphicsNode.h>
#include <cvc/gl/ImGuiOverlay.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/AriRuntime.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;
namespace gla = cvc::gl::ariadne;
using cvcgl_test::check;

void set(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

bool has_node(const gla::RealizedScene &r, const std::string &id) {
  for (const auto &t : r.nodes)
    if (t.id == id && !t.node.expired())
      return true;
  return false;
}

int count_nodes(const gla::RealizedScene &r, const std::string &prefix) {
  int n = 0;
  for (const auto &t : r.nodes)
    n += t.id.compare(0, prefix.size(), prefix) == 0 ? 1 : 0;
  return n;
}

bool has_created(const gla::RealizedScene &r, const std::string &id) {
  return std::find(r.created.begin(), r.created.end(), id) != r.created.end();
}

bool binds(const gla::RealizedScene &r, const std::string &source) {
  for (const auto &v : r.visibility)
    if (v.source_path == source)
      return true;
  return false;
}

// A node's world position (its origin through its world transform).
bool world_at(const std::shared_ptr<cvc::gl::GraphicsNode> &n, double x, double y, double z) {
  if (!n)
    return false;
  const double o[3] = {0, 0, 0};
  double w[3] = {0, 0, 0};
  n->localToWorld(o, w);
  return std::fabs(w[0] - x) < 1e-4 && std::fabs(w[1] - y) < 1e-4 && std::fabs(w[2] - z) < 1e-4;
}

std::size_t children(const std::shared_ptr<cvc::gl::GraphicsNode> &n, const std::string &prefix) {
  std::size_t k = 0;
  if (n)
    for (const auto &c : n->getGraphicsChildren())
      k += c && c->getName().compare(0, prefix.size(), prefix) == 0 ? 1 : 0;
  return k;
}

// What the custom node type's ticks touch: how many ran, and a token each tick holds.
struct Ticks {
  int ran = 0;
  std::shared_ptr<int> token = std::make_shared<int>(0);
};

const char *kDoc = R"(meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: disc
      repeat: pool.n
      source: { sphere: 0.5 }
      tags: [pool, "p{i}"]
      visible: pool.{i}.on
      transform: { position: [1, 2, 3] }
      children:
        - node: ring
          source: { box: 0.2 }
          tags: [ring]
    - node: lamp_{i}
      type: ticker
      repeat: lamps
    - node: last
      type: ticker
    - node: stage
      type: group
      transform: { position: [10, 0, 0] }
      children:
        - node: pin_{i}
          repeat: (int (state-get "pins"))
          source: { box: 0.1 }
    - node: row_{i}
      type: group
      repeat: rows
      children:
        - node: cell_{i}
          repeat: cells
          source: { box: 0.1 }
)";

const char *kHostDoc = R"(meta: { min_libcvc: "0.0.0" }
init: (state-set "n" 2)
windows:
  - window: Pool
    id: pool
    children:
      - button: More
        on: (state-set "n" (+ (int (state-get "n")) 1))
scene:
  chrome: false
  nodes:
    - node: dot_{i}
      repeat: (int (state-get "n"))
      source: { sphere: 0.3 }
      transform: { position: [0, 0, 0] }
)";

} // namespace

int main() {
  const bool require = std::getenv("CVC_REQUIRE_RENDER") != nullptr;
  cvc::app app;
  app.properties("system.log_verbosity", "0");

  // ── part 1: realize_scene and tick_scene ───────────────────────────────────────────────────
  {
    Ticks ticks;
    gla::register_scene_node_type(
        app, "ticker",
        [&ticks](cvc::gl::SceneGraph &sg, const cvc::ariadne::SceneNode &n,
                 cvc::gl::GraphicsNode *parent, gla::RealizedScene &out,
                 std::vector<std::string> *) -> std::shared_ptr<cvc::gl::GraphicsNode> {
          auto node = parent ? parent->createChild(n.id) : sg.addGraphics(n.id);
          out.custom_ticks.push_back(
              [&ticks, token = ticks.token,
               w = std::weak_ptr<cvc::gl::GraphicsNode>(node)](vtkRenderer *) {
                if (!w.expired())
                  ++ticks.ran;
              });
          return node;
        });

    cvc::ariadne::LoadResult lr = cvc::ariadne::load_string(app, kDoc);
    check(lr.ok, "the repeat document loads", lr.error);
    cvc::gl::SceneGraph sg(app, "rep");
    const std::string pre = "rep";
    set(app, pre + ".pool.n", "3");
    set(app, pre + ".lamps", "2");
    set(app, pre + ".pins", "2");
    set(app, pre + ".rows", "2");
    set(app, pre + ".cells", "2");
    cvc::ariadne::Runtime rt(app, pre); // the document's read lane, for the pins' expression
    gla::RealizeOptions opts;
    opts.count = [&rt](const std::string &e) { return rt.eval_count(e); };
    std::vector<std::string> warns;
    gla::RealizedScene r = gla::realize_scene(sg, lr.scene, pre, &warns, opts);
    sg.processEvents();
    for (const std::string &w : warns)
      check(false, "realizes without warnings", w);

    std::printf("== a key's count ==\n");
    check(sg.hasGraphics("disc_0") && sg.hasGraphics("disc_1") && sg.hasGraphics("disc_2") &&
              !sg.hasGraphics("disc_3"),
          "pool.n = 3 draws disc_0..disc_2 (an id without {i} gets _{i})");
    check(!sg.hasGraphics("disc") && !sg.hasGraphics("disc_{i}") && !has_created(r, "disc_{i}"),
          "the template itself is no node");
    check(has_created(r, "disc_0") && has_created(r, "disc_2"), "the copies are in created");
    check(count_nodes(r, "disc_") == 3 && count_nodes(r, "ring") == 3,
          "each copy and its child are in nodes", std::to_string(count_nodes(r, "disc_")));
    bool tagged = false;
    for (const auto &t : r.nodes)
      if (t.id == "disc_1")
        tagged = t.tags == std::vector<std::string>{"pool", "p1"};
    check(tagged, "a copy keeps its tags, {i} filled in");
    check(binds(r, pre + ".pool.0.on") && binds(r, pre + ".pool.2.on") &&
              !binds(r, pre + ".pool.3.on"),
          "each copy binds its own visible: key");
    auto d1 = sg.getGraphics("disc_1");
    check(world_at(d1, 1, 2, 3), "a copy keeps its transform:");
    check(d1 && d1->findChildByName("ring") != nullptr, "a copy keeps its children");

    set(app, pre + ".pool.1.on", "0");
    cvc::ariadne::sync_scene_visibility(app, r.visibility);
    sg.processEvents();
    check(d1 && !d1->isVisible() && sg.getGraphics("disc_0")->isVisible(),
          "a copy's visible: follows its own key");

    std::printf("== the count changes ==\n");
    const std::size_t vis3 = r.visibility.size();
    set(app, pre + ".pool.n", "5");
    gla::tick_scene(r, nullptr);
    sg.processEvents();
    check(sg.hasGraphics("disc_3") && sg.hasGraphics("disc_4") && !sg.hasGraphics("disc_5"),
          "pool.n = 5 adds disc_3 and disc_4 at the next tick");
    check(count_nodes(r, "disc_") == 5 && count_nodes(r, "ring") == 5 &&
              r.visibility.size() == vis3 + 2 && binds(r, pre + ".pool.4.on"),
          "their nodes and visible: binds are added");
    check(d1 == sg.getGraphics("disc_1"), "the copies already drawn stay as they were");
    set(app, pre + ".pool.n", "1");
    gla::tick_scene(r, nullptr);
    sg.processEvents();
    check(sg.hasGraphics("disc_0") && !sg.hasGraphics("disc_1") && !sg.hasGraphics("disc_4"),
          "pool.n = 1 leaves disc_0 alone");
    check(count_nodes(r, "disc_") == 1 && count_nodes(r, "ring") == 1 &&
              r.visibility.size() == vis3 - 2 && !binds(r, pre + ".pool.1.on") &&
              !has_created(r, "disc_1"),
          "the copies' nodes, binds and created entries go with them");
    set(app, pre + ".pool.n", "nonsense");
    gla::tick_scene(r, nullptr);
    check(!sg.hasGraphics("disc_0"), "a key that holds no number counts 0");

    std::printf("== a copy made later keeps its pose ==\n");
    const std::string d4 = sg.getGraphicsRoot()->getState().fullName() + ".children.disc_4";
    set(app, d4 + ".position", "7,8,9");
    set(app, pre + ".pool.n", "5");
    gla::tick_scene(r, nullptr);
    sg.processEvents();
    check(world_at(sg.getGraphics("disc_4"), 7, 8, 9),
          "disc_4, placed before it existed, starts there");
    check(world_at(sg.getGraphics("disc_3"), 1, 2, 3),
          "disc_3, whose last copy stood at the template's transform, starts there");

    std::printf("== a custom type's ticks ==\n");
    check(sg.hasGraphics("lamp_0") && sg.hasGraphics("lamp_1") && sg.hasGraphics("last"),
          "two ticker copies and a plain ticker");
    const std::size_t ticks2 = r.custom_ticks.size();
    ticks.ran = 0;
    gla::tick_scene(r, nullptr);
    check(ticks.ran == 3, "tick_scene runs both copies' ticks and the plain one's",
          std::to_string(ticks.ran));
    set(app, pre + ".lamps", "4");
    gla::tick_scene(r, nullptr);
    check(r.custom_ticks.size() == ticks2 + 2, "two more copies, two more ticks");
    ticks.ran = 0;
    gla::tick_scene(r, nullptr);
    check(ticks.ran == 5, "all five run", std::to_string(ticks.ran));
    const long held4 = ticks.token.use_count();
    set(app, pre + ".lamps", "0");
    gla::tick_scene(r, nullptr);
    check(r.custom_ticks.size() == ticks2 - 2, "lamps = 0 takes every copy's tick",
          std::to_string(r.custom_ticks.size()));
    check(ticks.token.use_count() == held4 - 4, "and releases what those ticks held",
          std::to_string(ticks.token.use_count()));
    ticks.ran = 0;
    gla::tick_scene(r, nullptr);
    check(ticks.ran == 1 && sg.hasGraphics("last") && !sg.hasGraphics("lamp_0"),
          "the plain ticker's tick, after them in the list, still runs", std::to_string(ticks.ran));
    set(app, pre + ".lamps", "1");
    gla::tick_scene(r, nullptr);
    ticks.ran = 0;
    gla::tick_scene(r, nullptr);
    check(ticks.ran == 2, "a copy made again ticks again", std::to_string(ticks.ran));

    std::printf("== under a group, by an expression ==\n");
    auto stage = sg.getGraphics("stage");
    check(children(stage, "pin_") == 2, "(int (state-get \"pins\")) = 2: two pins in the stage");
    check(stage && world_at(stage->findChildByName("pin_1"), 10, 0, 0),
          "a copy under a group is its child (its transform local to it)");
    set(app, pre + ".pins", "4");
    gla::tick_scene(r, nullptr);
    check(children(stage, "pin_") == 4 && count_nodes(r, "pin_") == 4,
          "the expression's new value adds two");
    set(app, pre + ".pins", "0");
    gla::tick_scene(r, nullptr);
    check(children(stage, "pin_") == 0 && count_nodes(r, "pin_") == 0, "and 0 takes them all");

    std::printf("== a repeat: in a repeated node ==\n");
    auto row0 = sg.getGraphics("row_0"), row1 = sg.getGraphics("row_1");
    check(children(row0, "cell_") == 2 && children(row1, "cell_") == 2,
          "rows = 2, cells = 2: two cells in each row");
    const std::size_t repeats = r.repeats.size();
    set(app, pre + ".cells", "3");
    gla::tick_scene(r, nullptr);
    check(children(row0, "cell_") == 3 && children(row1, "cell_") == 3,
          "cells = 3: a third cell in each row");
    set(app, pre + ".rows", "1");
    gla::tick_scene(r, nullptr);
    check(!sg.hasGraphics("row_1") && count_nodes(r, "cell_") == 3 &&
              r.repeats.size() == repeats - 1,
          "rows = 1: row_1 goes with its cells and its cells' repeat:",
          std::to_string(count_nodes(r, "cell_")));
    set(app, pre + ".rows", "2");
    gla::tick_scene(r, nullptr);
    check(children(sg.getGraphics("row_1"), "cell_") == 3 && r.repeats.size() == repeats,
          "rows = 2 again: row_1 is back with three cells");

    std::printf("== an expression with no read lane ==\n");
    cvc::gl::SceneGraph bare(app, "bare");
    std::vector<std::string> w2;
    gla::RealizedScene r2 = gla::realize_scene(bare, lr.scene, "bare", &w2);
    bool said = false;
    for (const std::string &w : w2)
      said = said || w.find("pin_{i}': repeat: (int (state-get \"pins\")) is an expression, and "
                            "this host counts none") != std::string::npos;
    check(said && children(bare.getGraphics("stage"), "pin_") == 0,
          "without RealizeOptions::count it draws none and says so");
  }

  // ── part 2: a document in a host ───────────────────────────────────────────────────────────
  const fs::path dir = // a directory of its own: runs side by side do not share the file
      fs::temp_directory_path() /
      ("cvcgl_ariadne_repeat_" +
       std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  fs::create_directories(dir);
  const fs::path doc = dir / "pool.ari";
  std::ofstream(doc) << kHostDoc;

  cvc::gl::SceneGraph sg(app, "pool");
  sg.setDiagnosticChromeVisible(false);
  cvc::gl::SceneRenderer view(sg, 320, 240, /*offscreen=*/true, "main");
  cvc::gl::CameraController cam(view);
  cvc::gl::ImGuiOverlay ui(view);
  if (!ui.enabled()) {
    std::printf("SKIP: no ImGui overlay\n");
    return require ? 1 : cvcgl_test::finish("cvcgl_ariadne_repeat");
  }
  ui.attachCamera(cam);
  ui.setVisible(true);
  ui.setFixedDeltaTime(1.0f / 30.0f);
  {
    gla::AriRuntime rt(view, cam, ui);
    for (const std::string &w : rt.load(doc.string()))
      check(false, "the host document loads without warnings", w);
    for (int f = 0; f < 3; ++f)
      rt.step(1.0 / 30.0);
    std::printf("== in AriRuntime ==\n");
    check(sg.hasGraphics("dot_0") && sg.hasGraphics("dot_1") && !sg.hasGraphics("dot_2"),
          "init: n = 2 draws two dots, counted on the document's read lane");
    set(app, "pool.n", "5");
    for (int f = 0; f < 2; ++f)
      rt.step(1.0 / 30.0);
    check(sg.hasGraphics("dot_4") && !sg.hasGraphics("dot_5"), "n = 5: five dots");
    set(app, "pool.n", "1");
    rt.step(1.0 / 30.0);
    check(sg.hasGraphics("dot_0") && !sg.hasGraphics("dot_1"), "n = 1: one");
  }
  return cvcgl_test::finish("cvcgl_ariadne_repeat");
}
