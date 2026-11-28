// cvcgl_molecule — the `molecule` scene node type (cvc/gl/ariadne/molecule_nodes.h, MoleculeNode):
//   - realize: its keys seeded from its props, the view's parameters described, its parts tagged
//     with its id; a source that names nothing yet waits for it;
//   - it follows a molecule on the state tree (no copy: it publishes the very pointer) and builds
//     its view as a sliced job (busy, progress, the RealizedScene::pending_work probe), drawing
//     1GRL as impostor spheres and sticks, then as a cartoon, then as tessellated spheres;
//   - per-chain visibility, a new molecule at the key, a parameter that does not parse;
//   - picking: a world ray meets the atom it should, through the node's transform, with its fields;
//   - two apps in one process, each with its own node and molecule (T6);
//   - drawn: impostor spheres depth-test as spheres (where two overlap, the nearer surface shows),
//     their shader builds (no fallback), and the tessellated path draws the same picture.
// Headless but for the last part, which skips where nothing rasterises (CVC_REQUIRE_RENDER=1 makes
// that a failure). Run again with CVC_SLICED_THREADING=cooperative (cvcgl_molecule_cooperative).

#include "streaming_test_util.h"

#include <boost/any.hpp>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cvc/ariadne/loader.h> // parse_value
#include <cvc/ariadne/scene.h>
#include <cvc/ariadne/value.h>
#include <cvc/core/app.h>
#include <cvc/core/async_lane.h> // cvc::pump_exec_frame
#include <cvc/gl/GeometryNode.h>
#include <cvc/gl/MoleculeNode.h>
#include <cvc/gl/SceneGraph.h>
#include <cvc/gl/SceneRenderer.h>
#include <cvc/gl/ariadne/extensions.h>
#include <cvc/gl/ariadne/scene_realize.h>
#include <cvc/mol/molecule.h>
#include <cvc/mol/view.h>
#include <cvc/state/state.h>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vtkActor.h>

using cvc::ariadne::Scene;
using cvc::ariadne::SceneNode;
using cvc::ariadne::Value;
using cvc::gl::GeometryNode;
using cvc::gl::MoleculeNode;
using cvc::gl::SceneGraph;
using cvc::gl::ariadne::RealizedScene;
using cvcgl_test::check;

namespace {

const std::string kRepo = CVC_REPO_DIR;

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}
void put(cvc::app &app, const std::string &path, const std::string &v) {
  cvc::state::instance(app)(path).value(v);
}

std::shared_ptr<const cvc::mol::molecule> grl() {
  const std::string f = kRepo + "/examples/data/molecules/1grl.pdb";
  std::ifstream in(f, std::ios::binary);
  return std::make_shared<const cvc::mol::molecule>(cvc::mol::read_pdb(in, {}, f));
}

cvc::mol::atom atom_at(const std::string &element, double x, double y, double z, double r,
                       char chain) {
  cvc::mol::atom a;
  a.name = element;
  a.element = element;
  a.residue = "LIG";
  a.chain = chain;
  a.x = x;
  a.y = y;
  a.z = z;
  a.radius = r;
  a.hetero = true;
  return a;
}

// One molecule node, its props from YAML, on its own scene graph in `app`, ticked headless.
struct Lab {
  cvc::app &app;
  SceneGraph sg;
  std::vector<std::string> warnings;
  RealizedScene r;
  std::shared_ptr<MoleculeNode> node;
  std::string base; // the node's state path

  static Scene one(const std::string &props, const std::string &frame) {
    Scene scene;
    SceneNode n;
    n.id = "protein";
    n.type = "molecule";
    n.frame = frame; // what the loader makes of frame:
    std::string why;
    if (!cvc::ariadne::parse_value(props, n.props, &why))
      check(false, "the props parse", why);
    scene.nodes.push_back(n);
    return scene;
  }

  Lab(cvc::app &a, const std::string &prefix, const std::string &props,
      const std::string &frame = std::string())
      : app(a), sg(a, prefix),
        r(cvc::gl::ariadne::realize_scene(sg, one(props, frame), prefix, &warnings)) {
    for (const std::string &w : warnings)
      std::printf("     warning: %s\n", w.c_str());
    node = std::dynamic_pointer_cast<MoleculeNode>(sg.getGraphics("protein"));
    if (node)
      base = node->stateName();
  }
  std::string get(const std::string &key) const { return val(app, base + "." + key); }
  void set(const std::string &key, const std::string &v) { put(app, base + "." + key, v); }
  void tick() {
    for (const auto &t : r.custom_ticks)
      t(nullptr);
    cvc::pump_exec_frame(app.exec_scheduler());
  }
  bool settling() const {
    for (const auto &p : r.pending_work)
      if (p && p())
        return true;
    return false;
  }
  // Tick until the node has nothing left to do (two ticks at least, so a key just written is
  // read); bounded at about two minutes.
  bool settle() {
    tick();
    for (int i = 0; i < 12000; ++i) {
      tick();
      if (!settling())
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
};

} // namespace

int main() {
  const char *threading = std::getenv("CVC_SLICED_THREADING");
  const bool cooperative = threading && std::string(threading) == "cooperative";
  std::printf("== molecule (%s) ==\n", cooperative ? "cooperative" : "automatic");
  cvc::app app;
  cvc::gl::ariadne::register_cvcgl_extensions(app);
  check(cvc::gl::ariadne::has_scene_node_type(app, "molecule"), "the molecule type is registered");

  // ── realize ─────────────────────────────────────────────────────────────────────────────────
  std::printf("== realize, and waiting for a molecule ==\n");
  {
    Lab lab(app, "m1", "{ source: { state: /mol.grl }, style: ball_and_stick, color_by: chain }");
    check(lab.node != nullptr, "the node is a MoleculeNode");
    if (!lab.node)
      return cvcgl_test::finish("cvcgl_molecule");
    check(lab.warnings.empty(), "no warnings");
    check(lab.get("source.molecule") == "/mol.grl", "source.molecule seeded from source:");
    check(lab.get("style") == "ball_and_stick", "style seeded from its prop");
    check(lab.get("color_by") == "chain", "color_by seeded from its prop");
    check(lab.get("bond_radius") == "0.15", "the other parameters seeded with their defaults",
          lab.get("bond_radius"));
    check(!cvc::state::instance(app)(lab.base + ".atom_scale").property("doc").empty(),
          "the parameters are described");
    check(lab.get("spheres") == "auto", "spheres: auto by default");
    lab.settle();
    check(lab.get("source.status") == "waiting for a molecule at /mol.grl", "it waits",
          lab.get("source.status"));
    check(!lab.settling(), "nothing pending while there is no molecule");
    int parts = 0;
    for (const RealizedScene::TaggedNode &t : lab.r.nodes)
      if (t.id == "protein" && t.node.lock() != lab.node)
        ++parts;
    check(parts == 3, "its three parts are tagged with its id", std::to_string(parts));
  }

  // ── 1GRL ────────────────────────────────────────────────────────────────────────────────────
  std::printf("== 1GRL: ball and stick, cartoon, spheres, chains ==\n");
  const auto m = grl();
  cvc::state::instance(app)("mol.grl").data(boost::any(m));
  {
    Lab lab(app, "m2", "{ source: { state: /mol.grl }, style: ball_and_stick }", "first");
    MoleculeNode &n = *lab.node;
    check(lab.settling(), "a molecule already there starts a build at realize");
    check(lab.settle(), "the build finishes");
    check(lab.get("source.status").empty(), "source.status empty", lab.get("source.status"));
    check(lab.get("info.atoms") == "4186", "info.atoms", lab.get("info.atoms"));
    check(lab.get("info.drawn") == "4186", "info.drawn", lab.get("info.drawn"));
    check(lab.get("info.helices") == "19" && lab.get("info.sheets") == "11",
          "info.helices and sheets", lab.get("info.helices") + " " + lab.get("info.sheets"));
    check(lab.get("info.id") == "1GRL", "info.id", lab.get("info.id"));
    check(lab.get("chains") == "1" && lab.get("chains.0.visible") == "1" &&
              lab.get("chains.0.atoms") == "4186",
          "one chain, visible", lab.get("chains"));
    check(std::stoi(lab.get("info.bonds")) > 3800, "bonds found", lab.get("info.bonds"));
    check(lab.get("busy") == "0" && lab.get("progress") == "100", "busy 0, progress 100");
    check(lab.get("status").find("4186 atoms") != std::string::npos, "status says what it drew",
          lab.get("status"));
    const boost::any pub = cvc::state::instance(app)(lab.base + ".molecule").data();
    const auto *p = boost::any_cast<std::shared_ptr<const cvc::mol::molecule>>(&pub);
    check(p && *p == m, "it publishes the very molecule it follows");
    check(n.molecule() == m, "and draws it, no copy");
    check(n.impostorCount() == 4186, "an impostor an atom", std::to_string(n.impostorCount()));
    check(n.impostorActor()->GetVisibility() == 1, "the impostors show");
    check(n.sticks() && n.sticks()->isVisible() && n.sticks()->getGeometry() &&
              !n.sticks()->getGeometry()->tris().empty(),
          "the sticks show");
    check(n.cartoon() && !n.cartoon()->isVisible(), "no cartoon");
    check(lab.get("info.spheres") == "impostor", "info.spheres impostor");
    const cvc::bounding_box bb = n.getBoundingBox();
    const std::array<double, 6> want = m->bounds(false);
    check(bb.minx < want[0] && bb.maxx > want[3], "its bounds hold the spheres");
    const std::string box = lab.get("info.bounds");
    check(!box.empty() && val(app, "m2.viewers.main.camera.frame") == box,
          "frame: first framed the main view on info.bounds", box);
    put(app, "m2.viewers.main.camera.frame", "");

    // Picking: down onto the highest atom, then the same through a transform.
    int top = 0;
    for (std::size_t i = 0; i < m->atoms.size(); ++i)
      if (m->atoms[i].z > m->atoms[top].z)
        top = static_cast<int>(i);
    const cvc::mol::atom &ta = m->atoms[top];
    const double o[3] = {ta.x, ta.y, ta.z + 50.0}, down[3] = {0.0, 0.0, -1.0};
    double w[3], t = 0.0;
    check(n.pickRay(o, down, w, &t) == top, "a ray down meets the highest atom");
    check(std::abs(t - (50.0 - 0.25 * ta.radius)) < 1e-4, "at its sphere's top");
    const auto f = n.atomFields(top);
    check(f.size() == 11 && f[0].first == "atom" && f[0].second == std::to_string(top) &&
              f[1].second == std::to_string(ta.serial) && f[4].second == ta.residue,
          "its fields");
    n.setPosition(100.0, 0.0, 0.0);
    const double o2[3] = {ta.x + 100.0, ta.y, ta.z + 50.0};
    check(n.pickRay(o2, down, w, &t) == top && std::abs(w[0] - (ta.x + 100.0)) < 1e-6,
          "through the node's transform");
    check(n.pickRay(o, down) != top, "and not where it was");
    n.setPosition(0.0, 0.0, 0.0);
    const double near_top[3] = {ta.x + 0.1, ta.y, ta.z};
    check(n.atomNear(near_top) == top, "the atom nearest a point");

    // Cartoon.
    lab.set("style", "cartoon");
    lab.set("color_by", "structure");
    check(lab.settle(), "the cartoon builds");
    check(n.impostorCount() == 0 && n.impostorActor()->GetVisibility() == 0, "no spheres");
    check(n.cartoon()->isVisible() && !n.cartoon()->getGeometry()->tris().empty(),
          "the cartoon shows");
    check(!n.sticks()->isVisible(), "no sticks");
    check(lab.get("info.residues") == "518", "info.residues: the trace", lab.get("info.residues"));
    check(val(app, "m2.viewers.main.camera.frame").empty(), "frame: first frames only once");
    check(lab.get("info.spheres").empty(), "info.spheres empty");
    check(n.atomNear(near_top) >= 0, "a cartoon's pick names a trace atom");

    // Tessellated spheres.
    lab.set("style", "spacefill");
    lab.set("spheres", "mesh");
    lab.set("sphere_detail", "0");
    check(lab.settle(), "tessellated spheres build");
    check(n.spherePath() == MoleculeNode::SpherePath::Mesh, "the mesh path");
    check(n.impostorActor()->GetVisibility() == 0, "the impostors hide");
    check(n.spheresMesh()->isVisible() &&
              n.spheresMesh()->getGeometry()->points().size() == 12 * 4186,
          "twelve vertices a sphere at detail 0");
    check(lab.get("info.spheres") == "mesh", "info.spheres mesh");
    check(n.pickRay(o, down) == top, "picks meet tessellated spheres too");
    lab.set("spheres", "impostor");
    check(lab.settle(), "back to impostors");
    check(n.impostorActor()->GetVisibility() == 1 && !n.spheresMesh()->isVisible(),
          "the impostors show again");

    // A chain hidden.
    lab.set("chains.0.visible", "0");
    check(lab.settle(), "hiding the chain rebuilds");
    check(lab.get("info.drawn") == "0" && n.impostorCount() == 0, "nothing drawn");
    lab.set("chains.0.visible", "1");
    check(lab.settle(), "showing it again");
    check(lab.get("info.drawn") == "4186", "all drawn again");

    // A parameter that does not parse: reported, nothing rebuilt, nothing pending.
    lab.set("style", "ribbons");
    lab.tick();
    lab.tick();
    check(lab.get("status").rfind("error: style", 0) == 0, "a bad style is reported",
          lab.get("status"));
    check(!lab.settling(), "and nothing is pending");
    check(n.impostorCount() == 4186, "what was drawn stays");
    lab.set("style", "spacefill");
    check(lab.settle(), "a good value builds again");

    // A new molecule at the key: two chains, and the node follows.
    cvc::mol::molecule two;
    two.atoms.push_back(atom_at("O", 0.0, 0.0, 0.0, 3.0, 'A'));
    two.atoms.push_back(atom_at("N", 10.0, 0.0, 0.0, 3.0, 'B'));
    cvc::state::instance(app)("mol.grl").data(
        boost::any(std::make_shared<const cvc::mol::molecule>(two)));
    check(lab.settle(), "the new molecule builds");
    check(lab.get("info.atoms") == "2" && lab.get("chains") == "2" && lab.get("chains.1.id") == "B",
          "its chains", lab.get("chains"));
    check(n.impostorCount() == 2, "two spheres");
    cvc::state::instance(app)("mol.grl").data(boost::any(m));
    check(lab.settle(), "and back");
    check(lab.get("chains") == "1", "one chain again", lab.get("chains"));
  }

  // ── two apps ────────────────────────────────────────────────────────────────────────────────
  std::printf("== two apps ==\n");
  {
    cvc::app other;
    cvc::gl::ariadne::register_cvcgl_extensions(other);
    cvc::mol::molecule small;
    small.atoms.push_back(atom_at("C", 0.0, 0.0, 0.0, 1.7, 'Z'));
    cvc::state::instance(other)("mol.grl").data(
        boost::any(std::make_shared<const cvc::mol::molecule>(small)));
    Lab a(app, "m3", "{ source: { state: /mol.grl }, style: cartoon }");
    Lab b(other, "m3", "{ source: { state: /mol.grl }, style: spacefill }");
    for (int i = 0; i < 2000 && (a.settling() || b.settling()); ++i) {
      a.tick();
      b.tick();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    check(!a.settling() && !b.settling(), "both builds finish, interleaved");
    check(a.get("info.atoms") == "4186" && b.get("info.atoms") == "1",
          "each app's node draws its own app's molecule");
    check(a.node->molecule() == m && b.node->molecule() != m, "no molecule shared across apps");
    check(b.get("chains.0.id") == "Z", "the other app's chains are its own");
    check(a.node->impostorCount() == 0 && b.node->impostorCount() == 1, "each its own style");
  }

  // ── drawn ───────────────────────────────────────────────────────────────────────────────────
  std::printf("== drawn: impostors depth-test as spheres; tessellated spheres alike ==\n");
  if (cvcgl_test::renderAvailable(app)) {
    // Two spheres seen from above: red (oxygen) at the origin, radius 3; blue (nitrogen) at
    // (2, 0, -1), radius 3. Along y = 0 the red surface is the higher until x ~ 2.4, then blue.
    cvc::mol::molecule pair;
    pair.atoms.push_back(atom_at("O", 0.0, 0.0, 0.0, 3.0, 'A'));
    pair.atoms.push_back(atom_at("N", 2.0, 0.0, -1.0, 3.0, 'A'));
    cvc::state::instance(app)("mol.pair")
        .data(boost::any(std::make_shared<const cvc::mol::molecule>(pair)));
    cvcgl_test::Frame frames[2];
    int k = 0;
    for (const char *path : {"impostor", "mesh"}) {
      Lab lab(app, std::string("m4") + path,
              std::string("{ source: { state: /mol.pair }, style: spacefill, sphere_detail: 4, "
                          "spheres: ") +
                  path + " }");
      lab.settle();
      lab.sg.setDiagnosticChromeVisible(false);
      lab.node->setLighting(1.0, 0.0, 0.0, 1.0); // flat colours
      cvc::gl::SceneRenderer sr(lab.sg, 200, 200, /*offscreen=*/true);
      sr.setBackground(0, 0, 0);
      cvcgl_test::topView(sr, 1.0, 0.0, 5.0);
      const cvcgl_test::Frame fr = cvcgl_test::grab(sr);
      frames[k++] = fr;
      const std::string tag = std::string(path) + ": ";
      check(!lab.node->impostorsFailed(), tag + "the impostor shader builds here");
      const auto at = [&](double x, double y) { return cvcgl_test::toPx(sr, x, y, 0.0); };
      check(cvcgl_test::hueNear(fr, at(0.0, 0.0), 1, 0), tag + "red at the red sphere's top");
      check(cvcgl_test::hueNear(fr, at(1.9, 0.0), 1, 0), tag + "red where it is the higher");
      check(cvcgl_test::hueNear(fr, at(2.9, 0.0), 1, 2), tag + "blue where it is the higher");
      check(cvcgl_test::hueNear(fr, at(4.6, 0.0), 1, 2), tag + "blue past the red sphere");
      check(cvcgl_test::bgNear(fr, at(-3.6, 0.0), 1), tag + "nothing past both");
      check(lab.get("info.spheres") == path, tag + "info.spheres", lab.get("info.spheres"));
    }
    // The same picture both ways, but for the facets at the silhouettes and where the two meet
    // (about 3% of the pixels, measured; a broken path differs in most).
    const long lit = cvcgl_test::litPixels(frames[0]);
    const long diff = cvcgl_test::diffPixels(frames[0], frames[1]);
    check(lit > 10000 && diff >= 0 && diff < lit / 20, "impostors and tessellated spheres agree",
          std::to_string(diff) + " of " + std::to_string(lit) + " pixels differ");
  }
  return cvcgl_test::finish("cvcgl_molecule");
}
